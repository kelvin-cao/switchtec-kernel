// SPDX-License-Identifier: GPL-2.0
/*
 * Microsemi Switchtec(tm) DMA Controller Driver
 * Copyright (c) 2019, Kelvin Cao <Kelvin.cao@microchip.com>
 * Copyright (c) 2019, Microsemi Corporation
 */

#include "dmaengine.h"

#include <linux/circ_buf.h>
#include <linux/dmaengine.h>
#include <linux/kref.h>
#include <linux/list.h>
#include <linux/module.h>
#include <linux/pci.h>

MODULE_DESCRIPTION("Switchtec PCIe Switch DMA Engine");
MODULE_VERSION("0.1");
MODULE_LICENSE("GPL");
MODULE_AUTHOR("Kelvin Cao");

enum switchtec_reg_offset {
	SWITCHTEC_DMAC_VERSION_OFFSET = 0,
	SWITCHTEC_DMAC_CAPABILITY_OFFSET = 0x80,
	SWITCHTEC_DMAC_STATUS_OFFSET = 0x100,
	SWITCHTEC_DMAC_CONTROL_OFFSET = 0x180,
	SWITCHTEC_DMAC_CHAN_CTRL_OFFSET = 0x1000,
	SWITCHTEC_DMAC_CHAN_CFG_STS_OFFSET = 0x160000,
};

#define SWITCHTEC_DESC_MAX_SIZE 0x100000

struct plx_dma_desc {
	struct dma_async_tx_descriptor txd;
	struct plx_dma_hw_std_desc *hw;
	u32 orig_size;
};

struct dmac_version_regs {
	u32 fw_ver;
	u32 dma_prot_ver;
	u32 hw_dmac_ver;
} __packed;

struct dmac_capability_regs {
	u32 cap;
	u32 chan_cnt;
	u32 rsvd;
	u32 cplt_tmo;
	u32 tag_limit;
	u16 ctrl_vect1;
	u16 ctrl_vect2;
	u16 se_buf_cnt;
	u16 se_buf_base;
} __packed;

struct dmac_status_regs {
	u32 state;
	u32 internal_err;
	u32 chan_halt_sum_lo;
	u32 chan_halt_sum_hi;
	u32 rsvd[2];
	u32 chan_paused_sum_lo;
	u32 chan_paused_sum_hi;
} __packed;

struct dmac_control_regs {
	u32 reset_halt;
} __packed;

#define SWITCHTEC_CHAN_CTRL_PAUSE     BIT(0)
#define SWITCHTEC_CHAN_CTRL_HALT      BIT(1)
#define SWITCHTEC_CHAN_CTRL_RESET     BIT(2)
#define SWITCHTEC_CHAN_CTRL_ERR_PAUSE BIT(3)

struct chan_hw_regs {
	u32 cq_head;
	u32 sq_tail;
	u32 ctrl;
	u32 status;
	u32 rsvd[124];
} __packed;

enum {
	PERF_BURST_SCALE = 0x1,
	PERF_BURST_SIZE = 0x6,
	PERF_INTERVAL = 0x1,
	PERF_MRRS = 0x3,
	PERF_ARB_WEIGHT = 0x1,
};

enum {
	PERF_BURST_SCALE_SHIFT = 2,
	PERF_BURST_SCALE_MASK = 0x11,
	PERF_MRRS_SHIFT = 4,
	PERF_MRRS_MASK = 0x7,
	PERF_INTERNAL_SHIFT = 8,
	PERF_INTERNAL_MASK = 0x7,
	PERF_BURST_SIZE_SHIFT = 12,
	PERF_BURST_SIZE_MASK = 0x7,
	PERF_ARB_WEIGHT_SHIFT = 24,
	PERF_ARB_WEIGHT_MASK = 0xff,
};

enum {
	SE_BUF_BASE_SHIFT = 3,
	SE_BUF_BASE_MASK = 0x1ff,
	SE_BUF_LEN_SHIFT = 12,
	SE_BUF_LEN_MASK = 0x1ff,
	SE_THRESH_SHIFT = 23,
	SE_THRESH_MASK = 0x1ff,
};

struct chan_fw_regs {
	u32 valid_en_se;
	u32 cq_base_lo;
	u32 cq_base_hi;
	u32 cq_size;
	u32 sq_base_lo;
	u32 sq_base_hi;
	u32 sq_size;
	u32 int_vec;
	u32 perf_cfg;
	u32 pmon_cfg;
	u32 perf_fetched_se_cnt_lo;
	u32 perf_fetched_se_cnt_hi;
	u32 perf_byte_cnt_lo;
	u32 perf_byte_cnt_hi;
	u32 rsvd1;
	u32 perf_se_queue;
	u32 rsvd2[2];
	u32 perf_lat_max;
	u32 perf_lat_min;
	u32 perf_lat_last;
	u32 rsvd3[76];
	u32 sq_current;
	u32 cq_current;
	u32 rsvd4[6];
} __packed;

#define SWITCHTEC_CHAN_INTERVAL 1
#define SWITCHTEC_CHAN_BURST_SZ 1
#define SWITCHTEC_CHAN_BURST_SCALE 1
#define SWITCHTEC_CHAN_MRRS 1

struct switchtec_dma_chan {
	struct switchtec_dma_dev *swdma_dev;
	struct dma_chan dma_chan;
	struct chan_hw_regs __iomem *mmio_chan_hw;
	struct chan_fw_regs __iomem *mmio_chan_fw;

	struct tasklet_struct desc_task;
	spinlock_t ring_lock;
	bool ring_active;
	int cid;
	/*
	 * In driver context, head is advanced by producer while
	 * tail is advanced by consumer.
	 */
	int head;
	int tail;
	struct switchtec_dma_desc **desc_ring;
	struct switchtec_dma_hw_se_desc *hw_sq;
//	int cq_head;
	int cq_tail;
	struct switchtec_dma_hw_ce *hw_cq;
};

struct switchtec_dma_dev {
	struct dma_device dma_dev;
	struct pci_dev __rcu *pdev;
	struct switchtec_dma_chan **swdma_chan;
	int chan_cnt;

	struct dmac_version_regs __iomem *mmio_dmac_ver;
	struct dmac_capability_regs __iomem *mmio_dmac_cap;
	struct dmac_status_regs __iomem *mmio_dmac_status;
	struct dmac_control_regs __iomem *mmio_dmac_ctrl;
	struct chan_hw_regs __iomem *mmio_chan_hw_all;
	struct chan_fw_regs __iomem *mmio_chan_fw_all;

	struct kref ref;
	struct work_struct release_work;
};

static struct switchtec_dma_chan *chan_to_switchtec_dma_chan(struct dma_chan *c)
{
	return container_of(c, struct switchtec_dma_chan, dma_chan);
}

enum switchtec_dma_opcode {
	SWITCHTEC_DMA_OPC_MEMCPY = 0x0,
	SWITCHTEC_DMA_OPC_RDIMM = 0x1,
	SWITCHTEC_DMA_OPC_WRIMM = 0x2,
	SWITCHTEC_DMA_OPC_RHI = 0x6,
	SWITCHTEC_DMA_OPC_NOP = 0x7,
};

struct switchtec_dma_hw_se_desc {
	u8 opc;
	u8 ctrl;
	__le16 tlp_setting;
	__le16 rsvd1;
	__le16 cid;
	__le32 byte_cnt;
	__le32 saddr_widata_lo;
	__le32 saddr_widata_hi;
	__le32 daddr_lo;
	__le32 daddr_hi;
	__le16 dfid_connid;
	__le16 sfid;
};

#define SWITCHTEC_SE_LIOF               BIT(14)
#define SWITCHTEC_SE_CID_MASK           0x3f

#define SWITCHTEC_CE_SC_LEN_ERR         BIT(0)
#define SWITCHTEC_CE_SC_UR              BIT(1)
#define SWITCHTEC_CE_SC_CA              BIT(2)
#define SWITCHTEC_CE_SC_RSVD_CPL        BIT(3)
#define SWITCHTEC_CE_SC_ECRC_ERR        BIT(4)
#define SWITCHTEC_CE_SC_EP_SET          BIT(5)
#define SWITCHTEC_CE_SC_D_RD_CTO        BIT(8)
#define SWITCHTEC_CE_SC_D_RIMM_UR       BIT(9)
#define SWITCHTEC_CE_SC_D_RIMM_CA       BIT(10)
#define SWITCHTEC_CE_SC_D_RIMM_RSVD_CPL BIT(11)
#define SWITCHTEC_CE_SC_D_ECRC          BIT(12)
#define SWITCHTEC_CE_SC_D_EP_SET        BIT(13)
#define SWITCHTEC_CE_SC_D_BAD_CONNID    BIT(14)
#define SWITCHTEC_CE_SC_D_BAD_RHI_ADDR  BIT(15)
#define SWITCHTEC_CE_SC_D_INVD_CMD      BIT(16)

struct switchtec_dma_hw_ce {
	__le32 rdimm_cpl_dw0;
	__le32 rdimm_cpl_dw1;
	__le32 rsvd1;
	__le32 cpl_byte_cnt;
	__le16 sq_head;
	__le16 rsvd2;
	__le32 rsvd3;
	__le16 sts_code;
	__le16 rsvd4;
	__le16 cid;
	__le16 phase_tag;
};

struct switchtec_dma_desc {
	struct dma_async_tx_descriptor txd;
	struct switchtec_dma_hw_se_desc *hw;
	u32 index;
	u32 orig_size;
};

static struct switchtec_dma_desc *to_switchtec_desc(
		struct dma_async_tx_descriptor *txd)
{
	return container_of(txd, struct switchtec_dma_desc, txd);
}

#define SWITCHTEC_DMA_SQ_SIZE (16 * 1024)
#define SWITCHTEC_DMA_CQ_SIZE (16 * 1024)

#define SWITCHTEC_DMA_RING_SIZE SWITCHTEC_DMA_SQ_SIZE

static struct switchtec_dma_desc *
switchtec_dma_get_desc(struct switchtec_dma_chan *swdma_chan, int i)
{
	return swdma_chan->desc_ring[i & (SWITCHTEC_DMA_SQ_SIZE - 1)];
}

static struct switchtec_dma_hw_ce *
switchtec_dma_get_ce(struct switchtec_dma_chan *swdma_chan, int i)
{
	return &swdma_chan->hw_cq[i];
}

static void switchtec_dma_process_desc(struct switchtec_dma_chan *swdma_chan)
{
	struct dmaengine_result res;
	struct switchtec_dma_desc *desc;
	static struct switchtec_dma_hw_ce *ce;
	u16 cq_head;
	u16 cq_tail;
	int cid;

	spin_lock_bh(&swdma_chan->ring_lock);

	cq_head = readw(&swdma_chan->mmio_chan_fw->cq_current);
	cq_tail = swdma_chan->cq_tail;
	while (CIRC_CNT(cq_head, cq_tail, SWITCHTEC_DMA_CQ_SIZE) >= 1) {
		desc = switchtec_dma_get_desc(swdma_chan, swdma_chan->tail);
		ce = switchtec_dma_get_ce(swdma_chan, cq_tail);
		cid = ce->cid;

		if (cid != desc->hw->cid)
			printk("cid != desc->hw->cid !!\n");


//		flags = le32_to_cpu(READ_ONCE(desc->hw->flags_and_size));
//
//		if (flags & PLX_DESC_FLAG_VALID)
//			break;

//		res.residue = desc->orig_size - (flags & PLX_DESC_SIZE_MASK);

//		if (flags & PLX_DESC_WB_SUCCESS)
//			res.result = DMA_TRANS_NOERROR;
//		else if (flags & PLX_DESC_WB_WR_FAIL)
//			res.result = DMA_TRANS_WRITE_FAILED;
//		else
//			res.result = DMA_TRANS_READ_FAILED;

		res.residue = desc->orig_size - ce->cpl_byte_cnt;
		if (!ce->sts_code)
			res.result = DMA_TRANS_NOERROR;
		else if (ce->sts_code & SWITCHTEC_CE_SC_D_RD_CTO)
			res.result = DMA_TRANS_READ_FAILED;
		else
			res.result = DMA_TRANS_WRITE_FAILED;

		dma_cookie_complete(&desc->txd);
		dma_descriptor_unmap(&desc->txd);
		dmaengine_desc_get_callback_invoke(&desc->txd, &res);
		desc->txd.callback = NULL;
		desc->txd.callback_result = NULL;

		swdma_chan->tail++;
	}

	spin_unlock_bh(&swdma_chan->ring_lock);
}

static void switchtec_dma_abort_desc(struct switchtec_dma_chan *swdma_chan)
{
	struct dmaengine_result res;
	struct switchtec_dma_desc *desc;
	u16 cq_head;
	u16 cq_tail;

	switchtec_dma_process_desc(swdma_chan);

	spin_lock_bh(&swdma_chan->ring_lock);

	cq_head = readw(&swdma_chan->mmio_chan_fw->cq_current);
	cq_tail = swdma_chan->cq_tail;
	while (CIRC_CNT(cq_head, cq_tail, SWITCHTEC_DMA_CQ_SIZE) >= 1) {
		desc = switchtec_dma_get_desc(swdma_chan, swdma_chan->tail);

		res.residue = desc->orig_size;
		res.result = DMA_TRANS_ABORTED;

		dma_cookie_complete(&desc->txd);
		dma_descriptor_unmap(&desc->txd);
		dmaengine_desc_get_callback_invoke(&desc->txd, &res);
		desc->txd.callback = NULL;
		desc->txd.callback_result = NULL;

		swdma_chan->tail++;
	}

	spin_unlock_bh(&swdma_chan->ring_lock);
}

static void __switchtec_dma_chan_stop(struct switchtec_dma_chan *swdma_chan)
{
	writeb(SWITCHTEC_CHAN_CTRL_RESET, &swdma_chan->mmio_chan_hw->ctrl);

	writel(0, &swdma_chan->mmio_chan_fw->sq_base_lo);
	writel(0, &swdma_chan->mmio_chan_fw->sq_base_hi);
	writel(0, &swdma_chan->mmio_chan_fw->cq_base_lo);
	writel(0, &swdma_chan->mmio_chan_fw->cq_base_hi);
}

static void switchtec_dma_chan_stop(struct switchtec_dma_chan *swdma_chan)
{
	rcu_read_lock();
	if (!rcu_dereference(swdma_chan->swdma_dev->pdev)) {
		rcu_read_unlock();
		return;
	}

	__switchtec_dma_chan_stop(swdma_chan);

	rcu_read_unlock();
}

static void __switchtec_dma_stop(struct switchtec_dma_dev *swdma_dev)
{
	int chan_cnt = swdma_dev->dma_dev.chancnt;
	int i;

	for (i = 0; i < chan_cnt; i++) {
		__switchtec_dma_chan_stop(swdma_dev->swdma_chan[i]);
	}
#if 0
	unsigned long timeout = jiffies + msecs_to_jiffies(1000);
	u32 val;

	val = readl(plxdev->bar + PLX_REG_CTRL);
	if (!(val & ~PLX_REG_CTRL_GRACEFUL_PAUSE))
		return;

	writel(PLX_REG_CTRL_RESET_VAL | PLX_REG_CTRL_GRACEFUL_PAUSE,
	       plxdev->bar + PLX_REG_CTRL);

	while (!time_after(jiffies, timeout)) {
		val = readl(plxdev->bar + PLX_REG_CTRL);
		if (val & PLX_REG_CTRL_GRACEFUL_PAUSE_DONE)
			break;

		cpu_relax();
	}

	if (!(val & PLX_REG_CTRL_GRACEFUL_PAUSE_DONE))
		dev_err(plxdev->dma_dev.dev,
			"Timeout waiting for graceful pause!\n");

	writel(PLX_REG_CTRL_RESET_VAL | PLX_REG_CTRL_GRACEFUL_PAUSE,
	       plxdev->bar + PLX_REG_CTRL);

	writel(0, plxdev->bar + PLX_REG_DESC_RING_COUNT);
	writel(0, plxdev->bar + PLX_REG_DESC_RING_ADDR);
	writel(0, plxdev->bar + PLX_REG_DESC_RING_ADDR_HI);
	writel(0, plxdev->bar + PLX_REG_DESC_RING_NEXT_ADDR);
#endif
}

#if 1
static void switchtec_dma_stop(struct switchtec_dma_dev *swdma_dev)
{
	rcu_read_lock();
	if (!rcu_dereference(swdma_dev->pdev)) {
		rcu_read_unlock();
		return;
	}

	__switchtec_dma_stop(swdma_dev);

	rcu_read_unlock();
}
#endif
static void switchtec_dma_desc_task(unsigned long data)
{
	struct switchtec_dma_chan *swdma_chan = (void *)data;

	switchtec_dma_process_desc(swdma_chan);
}

static struct dma_async_tx_descriptor *switchtec_dma_prep_memcpy(struct dma_chan *c,
		dma_addr_t dma_dst, dma_addr_t dma_src, size_t len,
		unsigned long flags)
	__acquires(plxdev->ring_lock)
{
	struct switchtec_dma_chan *swdma_chan = chan_to_switchtec_dma_chan(c);
	struct switchtec_dma_desc *swdma_desc;

	spin_lock_bh(&swdma_chan->ring_lock);
	if (!swdma_chan->ring_active)
		goto err_unlock;

	if (!CIRC_SPACE(swdma_chan->head, swdma_chan->tail, SWITCHTEC_DMA_RING_SIZE))
		goto err_unlock;

	if (len > SWITCHTEC_DESC_MAX_SIZE)
		goto err_unlock;

	swdma_desc = switchtec_dma_get_desc(swdma_chan, swdma_chan->head);

	swdma_desc->hw->opc = SWITCHTEC_DMA_OPC_MEMCPY;
	swdma_desc->hw->daddr_lo = cpu_to_le32(lower_32_bits(dma_dst));
	swdma_desc->hw->daddr_hi = cpu_to_le32(lower_32_bits(dma_dst));
	swdma_desc->hw->saddr_widata_lo = cpu_to_le32(lower_32_bits(dma_src));
	swdma_desc->hw->saddr_widata_hi = cpu_to_le32(lower_32_bits(dma_src));
	swdma_desc->hw->tlp_setting = 0;
	swdma_chan->cid++;
	swdma_chan->cid &= SWITCHTEC_SE_CID_MASK;
	swdma_desc->hw->cid = swdma_chan->cid;
	swdma_desc->index = swdma_chan->head;

	swdma_desc->orig_size = len;

	if (flags & DMA_PREP_INTERRUPT)
		swdma_desc->hw->ctrl |= SWITCHTEC_SE_LIOF;

	swdma_desc->txd.flags = flags;

	swdma_chan->head++;

	/* return with the lock held, it will be released in tx_submit */

	return &swdma_desc->txd;

err_unlock:
	/*
	 * Keep sparse happy by restoring an even lock count on
	 * this lock.
	 */
	__acquire(swdma_chan->ring_lock);

	spin_unlock_bh(&swdma_chan->ring_lock);
	return NULL;
}

static dma_cookie_t switchtec_dma_tx_submit(struct dma_async_tx_descriptor *desc)
	__releases(swdma_dev->ring_lock)
{
	struct switchtec_dma_chan *swdma_chan =
		chan_to_switchtec_dma_chan(desc->chan);
//	struct switchtec_dma_desc *swdma_desc = to_switchtec_desc(desc);
	dma_cookie_t cookie;

	cookie = dma_cookie_assign(desc);

//	/*
//	 * Ensure the descriptor updates are visible to the dma device
//	 * before setting the valid bit.
//	 */
//	wmb();

//	writew(swdma_desc->index, &swdma_chan->mmio_chan_hw->cq_head);

	spin_unlock_bh(&swdma_chan->ring_lock);

	return cookie;
}

static enum dma_status switchtec_dma_tx_status(struct dma_chan *chan,
		dma_cookie_t cookie, struct dma_tx_state *txstate)
{
	struct switchtec_dma_chan *swdma_chan =
		chan_to_switchtec_dma_chan(chan);
	enum dma_status ret;

	ret = dma_cookie_status(chan, cookie, txstate);
	if (ret == DMA_COMPLETE)
		return ret;

	switchtec_dma_process_desc(swdma_chan);

	return dma_cookie_status(chan, cookie, txstate);
}

static void switchtec_dma_issue_pending(struct dma_chan *chan)
{
	struct switchtec_dma_chan *swdma_chan =
		chan_to_switchtec_dma_chan(chan);
	struct switchtec_dma_dev *swdma_dev = swdma_chan->swdma_dev;

	rcu_read_lock();
	if (!rcu_dereference(swdma_dev->pdev)) {
		rcu_read_unlock();
		return;
	}

	/*
	 * Ensure the desc updates are visible before starting the
	 * DMA engine.
	 */
	wmb();

	/*
	 * The sq_tail register is actually for the head of the
	 * submisssion queue. Chip has the opposite define of head/tail
	 * to the Linux kernel.
	 */
	writew(swdma_chan->head - 1, &swdma_chan->mmio_chan_hw->sq_tail);

	rcu_read_unlock();
}

static irqreturn_t switchtec_dma_isr(int irq, void *chan)
{
	struct switchtec_dma_chan *swdma_chan = chan;

//	status = readw(plxdev->bar + PLX_REG_INTR_STATUS);
//
//	if (!status)
//		return IRQ_NONE;
//
	if (swdma_chan->ring_active)
		tasklet_schedule(&swdma_chan->desc_task);
//
//	writew(status, plxdev->bar + PLX_REG_INTR_STATUS);

	return IRQ_HANDLED;
}

static void switchtec_dma_release_work(struct work_struct *work)
{
	struct switchtec_dma_dev *swdma_dev =
		container_of(work,struct switchtec_dma_dev, release_work);

	dma_async_device_unregister(&swdma_dev->dma_dev);
	put_device(swdma_dev->dma_dev.dev);
	kfree(swdma_dev);
}

static void switchtec_dma_release(struct kref *ref)
{
	struct switchtec_dma_dev *swdma_dev = container_of(ref, struct switchtec_dma_dev, ref);

	/*
	 * The dmaengine reference counting and locking is a bit of a
	 * mess so we have to work around it a bit here. We might put
	 * the reference while the dmaengine holds the dma_list_mutex
	 * which means we can't call dma_async_device_unregister() directly
	 * here and it must be delayed.
	 */
	schedule_work(&swdma_dev->release_work);
}

static void switchtec_dma_put(struct switchtec_dma_dev *swdma_dev)
{
	kref_put(&swdma_dev->ref, switchtec_dma_release);
}

static int switchtec_dma_alloc_desc(struct switchtec_dma_chan *swdma_chan)
{
	struct switchtec_dma_desc *desc;
	int i, j;

	swdma_chan->desc_ring = kcalloc(SWITCHTEC_DMA_SQ_SIZE,
					sizeof(*swdma_chan->desc_ring),
					GFP_KERNEL);
	if (!swdma_chan->desc_ring)
		return -ENOMEM;

	for (i = 0; i < SWITCHTEC_DMA_SQ_SIZE; i++) {
		desc = kzalloc(sizeof(*desc), GFP_KERNEL);
		if (!desc)
			goto free_and_exit;

		dma_async_tx_descriptor_init(&desc->txd, &swdma_chan->dma_chan);
		desc->txd.tx_submit = switchtec_dma_tx_submit;
		desc->hw = &swdma_chan->hw_sq[i];

		swdma_chan->desc_ring[i] = desc;
	}

	return 0;

free_and_exit:
	for (j = 0; j < i; j++)
		kfree(swdma_chan->desc_ring[j]);
	kfree(swdma_chan->desc_ring);
	return -ENOMEM;
}

static int switchtec_dma_alloc_chan_resources(struct dma_chan *chan)
{
	struct switchtec_dma_chan *swdma_chan =
		chan_to_switchtec_dma_chan(chan);
	struct switchtec_dma_dev *swdma_dev = swdma_chan->swdma_dev;
	size_t sq_sz = SWITCHTEC_DMA_SQ_SIZE * sizeof(*swdma_chan->hw_sq);
	size_t cq_sz = SWITCHTEC_DMA_CQ_SIZE * sizeof(*swdma_chan->hw_cq);
	dma_addr_t dma_addr_sq;
	dma_addr_t dma_addr_cq;
	int rc;

	rcu_read_lock();
	if (!rcu_dereference(swdma_dev->pdev)) {
		rcu_read_unlock();
		return -ENODEV;
	}

	kref_get(&swdma_dev->ref);

//	writel(PLX_REG_CTRL_RESET_VAL, plxdev->bar + PLX_REG_CTRL);

	swdma_chan->hw_sq = dmam_alloc_coherent(swdma_dev->dma_dev.dev,
						sq_sz, &dma_addr_sq,
						GFP_KERNEL);
	if (!swdma_chan->hw_sq) {
		rcu_read_unlock();
		return -ENOMEM;
	}

	swdma_chan->hw_cq = dmam_alloc_coherent(swdma_dev->dma_dev.dev,
						cq_sz, &dma_addr_cq,
						GFP_KERNEL);
	if (!swdma_chan->hw_cq) {
		kfree(swdma_chan->hw_cq);
		rcu_read_unlock();
		return -ENOMEM;
	}

	swdma_chan->head = swdma_chan->tail = 0;

	rc = switchtec_dma_alloc_desc(swdma_chan);
	if (rc) {
		switchtec_dma_put(swdma_dev);
		rcu_read_unlock();
		return rc;
	}

	writel(lower_32_bits(dma_addr_sq),
	       &swdma_chan->mmio_chan_fw->sq_base_lo);
	writel(upper_32_bits(dma_addr_sq),
	       &swdma_chan->mmio_chan_fw->sq_base_hi);
	writel(lower_32_bits(dma_addr_cq),
	       &swdma_chan->mmio_chan_fw->cq_base_lo);
	writel(upper_32_bits(dma_addr_cq),
	       &swdma_chan->mmio_chan_fw->cq_base_hi);

	swdma_chan->ring_active = true;

	rcu_read_unlock();

	return SWITCHTEC_DMA_SQ_SIZE;
}

static void switchtec_dma_free_chan_resources(struct dma_chan *chan)
{
	struct switchtec_dma_chan *swdma_chan =
		chan_to_switchtec_dma_chan(chan);
	struct pci_dev *pdev;
	int i;

	spin_lock_bh(&swdma_chan->ring_lock);
	swdma_chan->ring_active = false;
	spin_unlock_bh(&swdma_chan->ring_lock);

	switchtec_dma_chan_stop(swdma_chan);

	rcu_read_lock();
	pdev = rcu_dereference(swdma_chan->swdma_dev->pdev);
	if (pdev)
		synchronize_irq(pci_irq_vector(pdev, 0));
	rcu_read_unlock();

	tasklet_kill(&swdma_chan->desc_task);

	switchtec_dma_abort_desc(swdma_chan);

	for (i = 0; i < SWITCHTEC_DMA_SQ_SIZE; i++)
		kfree(swdma_chan->desc_ring[i]);

	kfree(swdma_chan->desc_ring);

//	plx_dma_put(plxdev);
}

static int switchtec_dma_chans_create(struct switchtec_dma_dev *swdma_dev)
{
	struct switchtec_dma_chan *swdma_chan;
	struct dma_device *dma = &swdma_dev->dma_dev;
	struct dma_chan *chan;
	struct chan_fw_regs *chan_fw;
	struct chan_hw_regs *chan_hw;
	int chan_cnt;
	u32 value = 0;
	u32 thresh;
//	int base;
//	int cnt;
	int vec;
	int i, j;
	int rc;

	chan_cnt = le32_to_cpu(readl(&swdma_dev->mmio_dmac_cap->chan_cnt));

	if (!chan_cnt) {
		pci_err(swdma_dev->pdev, "No channel configured.\n");
		return -ENXIO;
	}

	swdma_dev->swdma_chan =
		kcalloc(chan_cnt, sizeof(*swdma_dev->swdma_chan), GFP_KERNEL);

	if (!swdma_dev->swdma_chan)
		return -ENOMEM;

#if 0 //user need to setup se buf base/cnt in the config file for each channel
	base = readl(&swdma_dev->mmio_dmac_cap->cap.se_buf_base);
	cnt = readl(&swdma_dev->mmio_dmac_cap->cap.se_buf_cnt) / chan_cnt;
#endif
	INIT_LIST_HEAD(&dma->channels);

	for (i = 0; i < chan_cnt; i++) {
		chan_fw = &swdma_dev->mmio_chan_fw_all[i];
		chan_hw = &swdma_dev->mmio_chan_hw_all[i];

#if 0
		/* init se buffer base/count */
		writel(cnt, &chan_cfg_sts->se_buf_cnt);
		writel(base + cnt * i, &chan_cfg_sts->se_buf_base);
#endif
		/* init perf tuner */
		value = PERF_BURST_SCALE << PERF_BURST_SCALE_SHIFT;
		value |= PERF_MRRS << PERF_MRRS_SHIFT;
		value |= PERF_INTERVAL << PERF_INTERNAL_SHIFT;
		value |= PERF_BURST_SIZE << PERF_BURST_SCALE_SHIFT;
		value |= PERF_ARB_WEIGHT << PERF_ARB_WEIGHT_SHIFT;
		writel(value, &chan_fw->perf_cfg);

		value = readl(&chan_fw->valid_en_se);
		thresh = ((value >> SE_BUF_LEN_SHIFT) & SE_BUF_LEN_MASK) / 2;
		value |= (thresh & SE_THRESH_MASK) << SE_THRESH_SHIFT;
		writel(value, &chan_fw->valid_en_se);

		swdma_chan = kzalloc(sizeof(*swdma_chan), GFP_KERNEL);
		if (!swdma_chan)
			goto error_exit;

		swdma_dev->swdma_chan[i] = swdma_chan;
		swdma_chan->mmio_chan_fw = chan_fw;
		swdma_chan->mmio_chan_hw = chan_hw;

		/* request irqs */
		vec = readl(&chan_fw->int_vec);
		rc = request_irq(pci_irq_vector(swdma_dev->pdev, vec),
				 switchtec_dma_isr, 0, KBUILD_MODNAME,
				 swdma_chan);
		if (rc) {
			kfree(swdma_dev->swdma_chan[i]);
			goto error_exit;
		}
		spin_lock_init(&swdma_chan->ring_lock);
		tasklet_init(&swdma_chan->desc_task, switchtec_dma_desc_task,
			     (unsigned long)swdma_chan);

		chan = &swdma_chan->dma_chan;
		chan->device = dma;
		dma_cookie_init(chan);
		list_add_tail(&chan->device_node, &dma->channels);
	}

	dma->chancnt = chan_cnt;

	return dma->chancnt;

error_exit:
	for (j = 0; j < i; j++) {
		kfree(swdma_dev->swdma_chan[j]);
		free_irq(pci_irq_vector(swdma_dev->pdev, 0),
			 swdma_dev->swdma_chan[j]);
	}
	kfree(swdma_dev->swdma_chan);

	return -ENOMEM;
}

static int switchtec_dma_chans_release(struct switchtec_dma_dev *swdma_dev)
{
	int i;

	for (i = 0; i < swdma_dev->dma_dev.chancnt; i++) {
		kfree(swdma_dev->swdma_chan[i]);
		free_irq(pci_irq_vector(swdma_dev->pdev, 0),
			 swdma_dev->swdma_chan[i]);
	}
	kfree(swdma_dev->swdma_chan);

	return 0;
}

static int switchtec_dma_create(struct pci_dev *pdev)
{
	struct switchtec_dma_dev *swdma_dev;
	struct dma_device *dma;
	void __iomem *bar;
	int chan_cnt;
	int rc;

	/*
	 * Create the switchtec dma device
	 */
	swdma_dev = kzalloc(sizeof(*swdma_dev), GFP_KERNEL);
	if (!swdma_dev)
		return -ENOMEM;

	bar = pcim_iomap_table(pdev)[0];

	swdma_dev->mmio_dmac_ver = bar + SWITCHTEC_DMAC_VERSION_OFFSET;
	swdma_dev->mmio_dmac_cap = bar + SWITCHTEC_DMAC_CAPABILITY_OFFSET;
	swdma_dev->mmio_dmac_status = bar + SWITCHTEC_DMAC_STATUS_OFFSET;
	swdma_dev->mmio_dmac_ctrl = bar + SWITCHTEC_DMAC_CONTROL_OFFSET;
	swdma_dev->mmio_chan_hw_all = bar + SWITCHTEC_DMAC_CHAN_CTRL_OFFSET;
	swdma_dev->mmio_chan_fw_all = bar + SWITCHTEC_DMAC_CHAN_CFG_STS_OFFSET;

	chan_cnt = switchtec_dma_chans_create(swdma_dev);
	if (chan_cnt < 0) {
		kfree(swdma_dev);
		pci_err(pdev, "Failed to create dma channels: %d\n", chan_cnt);
		return chan_cnt;
	}

	dma = &swdma_dev->dma_dev;
	dma->chancnt = chan_cnt;
	kref_init(&swdma_dev->ref);
	INIT_WORK(&swdma_dev->release_work, switchtec_dma_release_work);
	RCU_INIT_POINTER(swdma_dev->pdev, pdev);

	dma_cap_set(DMA_MEMCPY|DMA_PRIVATE, dma->cap_mask);
	dma->copy_align = DMAENGINE_ALIGN_1_BYTE;
	dma->dev = get_device(&pdev->dev);

	dma->device_alloc_chan_resources = switchtec_dma_alloc_chan_resources;
	dma->device_free_chan_resources = switchtec_dma_free_chan_resources;
	dma->device_prep_dma_memcpy = switchtec_dma_prep_memcpy;
	dma->device_issue_pending = switchtec_dma_issue_pending;
	dma->device_tx_status = switchtec_dma_tx_status;

	rc = dma_async_device_register(dma);
	if (rc) {
		pci_err(pdev, "Failed to register dma device: %d\n", rc);
		kfree(swdma_dev);
		switchtec_dma_chans_release(swdma_dev);
		return rc;
	}

	pci_set_drvdata(pdev, swdma_dev);

	return 0;
}

static int switchtec_dma_probe(struct pci_dev *pdev,
			       const struct pci_device_id *id)
{
	int rc;

	rc = pcim_enable_device(pdev);
	if (rc)
		return rc;

	rc = pci_set_dma_mask(pdev, DMA_BIT_MASK(48));
	if (rc)
		rc = pci_set_dma_mask(pdev, DMA_BIT_MASK(32));
	if (rc)
		return rc;

	rc = pci_set_consistent_dma_mask(pdev, DMA_BIT_MASK(48));
	if (rc)
		rc = pci_set_consistent_dma_mask(pdev, DMA_BIT_MASK(32));
	if (rc)
		return rc;

	rc = pcim_iomap_regions(pdev, 1, KBUILD_MODNAME);
	if (rc)
		return rc;

	/*
	 * Get number of channels, and allocate irq per channel number
	 */
	rc = pci_alloc_irq_vectors(pdev, 1, 1, PCI_IRQ_ALL_TYPES);
	if (rc <= 0)
		return rc;

	pci_set_master(pdev);

	rc = switchtec_dma_create(pdev);
	if (rc)
		goto err_free_irq_vectors;

	pci_info(pdev, "PLX DMA Channel Registered\n");

	return 0;

err_free_irq_vectors:
	pci_free_irq_vectors(pdev);
	return rc;
}

static void switchtec_dma_remove(struct pci_dev *pdev)
{
	struct switchtec_dma_dev *swdma_dev = pci_get_drvdata(pdev);
	struct switchtec_dma_chan *swdma_chan;
	int i;

	free_irq(pci_irq_vector(pdev, 0),  swdma_dev);

	rcu_assign_pointer(swdma_dev->pdev, NULL);
	synchronize_rcu();

	for (i = 0; i < SWITCHTEC_DMA_SQ_SIZE; i++) {
		swdma_chan = swdma_dev->swdma_chan[i];

		spin_lock_bh(&swdma_chan->ring_lock);
		swdma_chan->ring_active = false;
		spin_unlock_bh(&swdma_chan->ring_lock);
		switchtec_dma_abort_desc(swdma_chan);
	}

	__switchtec_dma_stop(swdma_dev);

	swdma_dev->mmio_dmac_ver = NULL;
	swdma_dev->mmio_dmac_cap = NULL;
	swdma_dev->mmio_dmac_status = NULL;
	swdma_dev->mmio_dmac_ctrl = NULL;
	swdma_dev->mmio_chan_hw_all = NULL;
	swdma_dev->mmio_chan_fw_all = NULL;
	switchtec_dma_put(swdma_dev);

	pci_free_irq_vectors(pdev);
}

#define MICROSEMI_VENDOR_ID         0x11f8

#define SWITCHTEC_PCI_DEVICE(device_id) \
	{ \
		.vendor     = MICROSEMI_VENDOR_ID, \
		.device     = device_id, \
		.subvendor  = PCI_ANY_ID, \
		.subdevice  = PCI_ANY_ID, \
		.class      = PCI_CLASS_SYSTEM_OTHER << 8, \
		.class_mask = 0xFFFFFFFF, \
	}

static const struct pci_device_id switchtec_dma_pci_tbl[] = {
	SWITCHTEC_PCI_DEVICE(0x4000),  //PFX 100XG4
	SWITCHTEC_PCI_DEVICE(0x4084),  //PFX 84XG4
	SWITCHTEC_PCI_DEVICE(0x4068),  //PFX 68XG4
	SWITCHTEC_PCI_DEVICE(0x4052),  //PFX 52XG4
	SWITCHTEC_PCI_DEVICE(0x4036),  //PFX 36XG4
	SWITCHTEC_PCI_DEVICE(0x4028),  //PFX 28XG4
	SWITCHTEC_PCI_DEVICE(0x4100),  //PSX 100XG4
	SWITCHTEC_PCI_DEVICE(0x4184),  //PSX 84XG4
	SWITCHTEC_PCI_DEVICE(0x4168),  //PSX 68XG4
	SWITCHTEC_PCI_DEVICE(0x4152),  //PSX 52XG4
	SWITCHTEC_PCI_DEVICE(0x4136),  //PSX 36XG4
	SWITCHTEC_PCI_DEVICE(0x4128),  //PSX 28XG4
	SWITCHTEC_PCI_DEVICE(0x4200),  //PAX 100XG4
	SWITCHTEC_PCI_DEVICE(0x4284),  //PAX 84XG4
	SWITCHTEC_PCI_DEVICE(0x4268),  //PAX 68XG4
	SWITCHTEC_PCI_DEVICE(0x4252),  //PAX 52XG4
	SWITCHTEC_PCI_DEVICE(0x4236),  //PAX 36XG4
	SWITCHTEC_PCI_DEVICE(0x4228),  //PAX 28XG4
	{0}
};
MODULE_DEVICE_TABLE(pci, switchtec_dma_pci_tbl);

static struct pci_driver switchtec_dma_pci_driver = {
	.name           = KBUILD_MODNAME,
	.id_table       = switchtec_dma_pci_tbl,
	.probe          = switchtec_dma_probe,
	.remove		= switchtec_dma_remove,
};
module_pci_driver(switchtec_dma_pci_driver);
