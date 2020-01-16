// SPDX-License-Identifier: GPL-2.0
/*
 * Microchip Switchtec(tm) DMA Controller Driver
 * Copyright (c) 2019, Kelvin Cao <kelvin.cao@microchip.com>
 * Copyright (c) 2019, Microchip Corporation
 */

#include "dmaengine.h"

#include <linux/circ_buf.h>
#include <linux/dmaengine.h>
#include <linux/kref.h>
#include <linux/list.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/pci.h>
#include <linux/delay.h>

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

#define SWITCHTEC_CHAN_STS_HALTED     BIT(10)

struct chan_hw_regs {
	u16 cq_head;
	u16 rsvd1;
	u16 sq_tail;
	u16 rsvd2;
	u8 ctrl;
	u8 rsvd3[3];
	u16 status;
	u16 rsvd4;
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
	PERF_BURST_SCALE_MASK = 0x3,
	PERF_MRRS_SHIFT = 4,
	PERF_MRRS_MASK = 0x7,
	PERF_INTERVAL_SHIFT = 8,
	PERF_INTERVAL_MASK = 0x7,
	PERF_BURST_SIZE_SHIFT = 12,
	PERF_BURST_SIZE_MASK = 0x7,
	PERF_ARB_WEIGHT_SHIFT = 24,
	PERF_ARB_WEIGHT_MASK = 0xff,
};

enum {
	PERF_MIN_INTERVAL = 0,
	PERF_MAX_INTERVAL = 7,
	PERF_MIN_BURST_SIZE = 0,
	PERF_MAX_BURST_SIZE = 7,
	PERF_MIN_BURST_SCALE = 0,
	PERF_MAX_BURST_SCALE = 2,
	PERF_MIN_MRRS = 0,
	PERF_MAX_MRRS = 7,
};

enum {
	SE_BUF_BASE_SHIFT = 3,
	SE_BUF_BASE_MASK = 0x1ff,
	SE_BUF_LEN_SHIFT = 12,
	SE_BUF_LEN_MASK = 0x1ff,
	SE_THRESH_SHIFT = 23,
	SE_THRESH_MASK = 0x1ff,
};

#define SWITCHTEC_CHAN_ENABLE BIT(1)

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
	u32 rsvd1;
	u32 pmon_ctrl;
	u32 perf_fetched_se_cnt_lo;
	u32 perf_fetched_se_cnt_hi;
	u32 perf_byte_cnt_lo;
	u32 perf_byte_cnt_hi;
	u32 rsvd2;
	u32 perf_se_queue;
	u32 rsvd3;
	u32 perf_lat_max;
	u32 perf_lat_min;
	u32 perf_lat_last;
	u16 sq_current;
	u16 sq_phase;	//bit 15 is the phase bit
	u16 cq_current;
	u16 cq_phase;	//bit 15 is the phase bit
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

	/* channel index */
	int index;

	/*
	 * In driver context, head is advanced by producer while
	 * tail is advanced by consumer.
	 */

	/* the head and tail for both desc_ring and hw_sq */
	int head;
	int tail;
	struct switchtec_dma_desc **desc_ring;
	struct switchtec_dma_hw_se_desc *hw_sq;

	/* the head and tail for both hw_cq */
	int cq_head;
	int cq_tail;
	struct switchtec_dma_hw_ce *hw_cq;

	struct kobject kobj;
};

struct switchtec_dma_dev {
	struct dma_device dma_dev;
	struct pci_dev __rcu *pdev;
	struct switchtec_dma_chan **swdma_chans;
	int chan_cnt;

	struct dmac_version_regs __iomem *mmio_dmac_ver;
	struct dmac_capability_regs __iomem *mmio_dmac_cap;
	struct dmac_status_regs __iomem *mmio_dmac_status;
	struct dmac_control_regs __iomem *mmio_dmac_ctrl;
	void __iomem *mmio_chan_hw_all;
	void __iomem *mmio_chan_fw_all;

	struct kref ref;
	struct work_struct release_work;
};

static struct switchtec_dma_chan *to_switchtec_dma_chan(struct dma_chan *c)
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

#define SWITCHTEC_SE_LIOF               BIT(6)
#define SWITCHTEC_SE_CID_MASK           0xffff

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
void __iomem *global_bar;
#if 0
static struct switchtec_dma_desc *to_switchtec_desc(
		struct dma_async_tx_descriptor *txd)
{
	return container_of(txd, struct switchtec_dma_desc, txd);
}
#endif
#define HALT_RETRY 100
static int halt_channel(struct switchtec_dma_chan *swdma_chan)
{
	u8 ctrl;
	u32 status;
	struct chan_hw_regs *chan_hw = swdma_chan->mmio_chan_hw;
	int retry = HALT_RETRY;
	struct pci_dev *pdev = swdma_chan->swdma_dev->pdev;

	ctrl = readb(&chan_hw->ctrl);

	ctrl |= SWITCHTEC_CHAN_CTRL_HALT | SWITCHTEC_CHAN_CTRL_RESET;
	pci_dbg(pdev, "chan %d: halt channel, ctrl 0x%x\n",
		swdma_chan->index, ctrl);
	writeb(ctrl, &chan_hw->ctrl);

	do {
		status = readl(&chan_hw->status);
		pci_dbg(pdev, "chan %d: status: %x\n",
			swdma_chan->index, status);

		if (status & SWITCHTEC_CHAN_STS_HALTED)
			return 0;
		else
			msleep(1);
	} while (retry--);

	return 1;
}

static int unhalt_channel(struct switchtec_dma_chan *swdma_chan)
{
	u8 ctrl;
	u32 status;
	struct chan_hw_regs *chan_hw = swdma_chan->mmio_chan_hw;
	int retry = HALT_RETRY;
	struct pci_dev *pdev = swdma_chan->swdma_dev->pdev;

	ctrl = readb(&chan_hw->ctrl);
	ctrl &= !SWITCHTEC_CHAN_CTRL_HALT;
	writeb(ctrl, &chan_hw->ctrl);

	pci_dbg(pdev, "chan %d: unhalt channel, ctrl 0x%x\n",
		swdma_chan->index, ctrl);
	do {
		status = readl(&chan_hw->status);
		pci_dbg(pdev, "chan %d: status 0x%x\n",
			swdma_chan->index, status);
		if (!(status & SWITCHTEC_CHAN_STS_HALTED)) {
			pci_dbg(pdev, "chan %d: unhaletd\n", swdma_chan->index);
			return 0;
		} else
			msleep(1);
	} while (retry--);

	pci_dbg(pdev, "chan %d: unhalt channel failed\n", swdma_chan->index);
	return 1;
}

static int reset_channel(struct switchtec_dma_chan *swdma_chan)
{
	u8 ctrl;
	struct chan_hw_regs *chan_hw = swdma_chan->mmio_chan_hw;
	struct pci_dev *pdev = swdma_chan->swdma_dev->pdev;

	ctrl = readb(&chan_hw->ctrl);
	ctrl &= !SWITCHTEC_CHAN_CTRL_RESET;

	writel(ctrl, &chan_hw->ctrl);
	pci_dbg(pdev, "chan %d: reset channel, ctrl 0x%x\n",
		swdma_chan->index, ctrl);

	return 0;
}

static int enable_channel(struct switchtec_dma_chan *swdma_chan)
{
	u32 valid_en_se;
	struct chan_fw_regs *chan_fw = swdma_chan->mmio_chan_fw;
	struct pci_dev *pdev = swdma_chan->swdma_dev->pdev;

	valid_en_se = readl(&chan_fw->valid_en_se);
	valid_en_se |= SWITCHTEC_CHAN_ENABLE;

	writel(valid_en_se, &chan_fw->valid_en_se);
	pci_dbg(pdev, "chan %d: enable channel, valid_en_se 0x%x\n",
		swdma_chan->index, valid_en_se);

	return 0;
}

#define SWITCHTEC_DMA_SQ_SIZE 10240 //(16 * 1024)
#define SWITCHTEC_DMA_CQ_SIZE 10240 //(16 * 1024)

#define SWITCHTEC_DMA_RING_SIZE SWITCHTEC_DMA_SQ_SIZE

static struct switchtec_dma_desc *
switchtec_dma_get_desc(struct switchtec_dma_chan *swdma_chan, int i)
{
//	return swdma_chan->desc_ring[i & (SWITCHTEC_DMA_SQ_SIZE - 1)];
	return swdma_chan->desc_ring[i];
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
//	u16 cq_tail;
	int cid;
	int cnt = 0;

	dev_dbg(&swdma_chan->dma_chan.dev->device, "%s\n", __FUNCTION__);

#if 1
	spin_lock_bh(&swdma_chan->ring_lock);
#endif

	cq_head = readw(&swdma_chan->mmio_chan_fw->cq_current);
//	printk("cq_head is %x\n", cq_head);
	while ((cnt = CIRC_CNT(cq_head, swdma_chan->cq_tail, SWITCHTEC_DMA_CQ_SIZE)) >= 1) {
//		if (i++ > 100)
//			break;
//		printk("cnt is 0x%x\n", cnt);
		/* should get the desc with cid? */
		desc = switchtec_dma_get_desc(swdma_chan, swdma_chan->tail);
		ce = switchtec_dma_get_ce(swdma_chan, swdma_chan->cq_tail);
		cid = ce->cid;
//		printk("cid:           0x%x\n", cid);
//		printk("desc->hw->cid: 0x%x\n", desc->hw->cid);

		if (cid != desc->hw->cid)
			dev_emerg(&swdma_chan->dma_chan.dev->device,
				   "cid != desc->hw->cid !!\n");

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

		swdma_chan->cq_tail++;
		if (swdma_chan->cq_tail == SWITCHTEC_DMA_CQ_SIZE)
			swdma_chan->cq_tail = 0;
		writew(swdma_chan->cq_tail, &swdma_chan->mmio_chan_hw->cq_head);
//		printk("swdma_chan->cq_tail is: %x\n", swdma_chan->cq_tail);
		swdma_chan->tail++;
		if (swdma_chan->tail == SWITCHTEC_DMA_SQ_SIZE)
			swdma_chan->tail = 0;
	}
#if 1
	spin_unlock_bh(&swdma_chan->ring_lock);
#endif
//	if (i > 90)
//		printk("possible definite while loop\n");
}

static void switchtec_dma_abort_desc(struct switchtec_dma_chan *swdma_chan)
{
	struct dmaengine_result res;
	struct switchtec_dma_desc *desc;
	u16 cq_head;
	u16 cq_tail;

//	switchtec_dma_process_desc(swdma_chan);

	spin_lock_bh(&swdma_chan->ring_lock);

	cq_head = readw(&swdma_chan->mmio_chan_fw->cq_current);
	cq_tail = swdma_chan->cq_tail;
	dev_dbg(&swdma_chan->dma_chan.dev->device,
		"%s: cq_head: %d, cq_tail: %d\n",
		__FUNCTION__, cq_head, cq_tail);

	spin_unlock_bh(&swdma_chan->ring_lock);
	return;

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
	dev_dbg(&swdma_chan->dma_chan.dev->device, "%s\n", __FUNCTION__);

	writeb(SWITCHTEC_CHAN_CTRL_RESET, &swdma_chan->mmio_chan_hw->ctrl);

	writel(0, &swdma_chan->mmio_chan_fw->sq_base_lo);
	writel(0, &swdma_chan->mmio_chan_fw->sq_base_hi);
	writel(0, &swdma_chan->mmio_chan_fw->cq_base_lo);
	writel(0, &swdma_chan->mmio_chan_fw->cq_base_hi);
}

static void switchtec_dma_chan_stop(struct switchtec_dma_chan *swdma_chan)
{
	dev_dbg(&swdma_chan->dma_chan.dev->device, "%s\n", __FUNCTION__);
	rcu_read_lock();
	if (!rcu_dereference(swdma_chan->swdma_dev->pdev)) {
		rcu_read_unlock();
		return;
	}

	__switchtec_dma_chan_stop(swdma_chan);

	rcu_read_unlock();
}

#if 0
static void __switchtec_dma_stop(struct switchtec_dma_dev *swdma_dev)
{
	int chan_cnt = swdma_dev->dma_dev.chancnt;
	int i;

	for (i = 0; i < chan_cnt; i++) {
		__switchtec_dma_chan_stop(swdma_dev->swdma_chans[i]);
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
#endif
#if 0
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
	struct switchtec_dma_chan *swdma_chan = to_switchtec_dma_chan(c);
	struct switchtec_dma_desc *swdma_desc;

	dev_dbg(&swdma_chan->dma_chan.dev->device, "%s\n", __FUNCTION__);

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
	swdma_desc->hw->daddr_hi = cpu_to_le32(upper_32_bits(dma_dst));
	swdma_desc->hw->saddr_widata_lo = cpu_to_le32(lower_32_bits(dma_src));
	swdma_desc->hw->saddr_widata_hi = cpu_to_le32(upper_32_bits(dma_src));
	swdma_desc->hw->byte_cnt = len;
	swdma_desc->hw->tlp_setting = 0;
	swdma_chan->cid++;
	swdma_chan->cid &= SWITCHTEC_SE_CID_MASK;
	swdma_desc->hw->cid = swdma_chan->cid;
	swdma_desc->index = swdma_chan->head;

	swdma_desc->orig_size = len;

	if (flags & DMA_PREP_INTERRUPT) {
		swdma_desc->hw->ctrl |= SWITCHTEC_SE_LIOF;
	}

	swdma_desc->txd.flags = flags;

	swdma_chan->head++;
	if (swdma_chan->head == SWITCHTEC_DMA_RING_SIZE)
		swdma_chan->head = 0;

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
		to_switchtec_dma_chan(desc->chan);
//	struct switchtec_dma_desc *swdma_desc = to_switchtec_desc(desc);
	dma_cookie_t cookie;

	cookie = dma_cookie_assign(desc);

//	/*
//	 * Ensure the descriptor updates are visible to the dma device
//	 * before setting the valid bit.
//	 */
//	wmb();

//	writew(swdma_desc->index, &swdma_chan->mmio_chan_hw->sq_tail);

	spin_unlock_bh(&swdma_chan->ring_lock);

	return cookie;
}

static enum dma_status switchtec_dma_tx_status(struct dma_chan *chan,
		dma_cookie_t cookie, struct dma_tx_state *txstate)
{
	struct switchtec_dma_chan *swdma_chan = to_switchtec_dma_chan(chan);
	enum dma_status ret;

	dev_dbg(&swdma_chan->dma_chan.dev->device, "%s\n", __FUNCTION__);

	ret = dma_cookie_status(chan, cookie, txstate);
	if (ret == DMA_COMPLETE)
		return ret;

	switchtec_dma_process_desc(swdma_chan);

	return dma_cookie_status(chan, cookie, txstate);
}

static void switchtec_dma_issue_pending(struct dma_chan *chan)
{
	struct switchtec_dma_chan *swdma_chan = to_switchtec_dma_chan(chan);
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
	dev_dbg(&swdma_chan->dma_chan.dev->device, "%s, head 0x%x\n",
		__FUNCTION__, swdma_chan->head);

	writew(swdma_chan->head, &swdma_chan->mmio_chan_hw->sq_tail);
	rcu_read_unlock();
}

static irqreturn_t switchtec_dma_isr(int irq, void *chan)
{
	struct switchtec_dma_chan *swdma_chan = chan;
	static int count = 0;

//	status = readw(plxdev->bar + PLX_REG_INTR_STATUS);
//
//	if (!status)
//		return IRQ_NONE;
//
	if (swdma_chan->ring_active) {
		if (count++ > 100) {
			tasklet_schedule(&swdma_chan->desc_task);
			count = 0;
		}
	}
//
//	writew(status, plxdev->bar + PLX_REG_INTR_STATUS);

	return IRQ_HANDLED;
}

void switchtec_kobject_del(struct switchtec_dma_dev *swdma_dev);

static void switchtec_dma_release_work(struct work_struct *work)
{
	struct switchtec_dma_dev *swdma_dev =
		container_of(work,struct switchtec_dma_dev, release_work);

	dev_dbg(swdma_dev->dma_dev.dev, "%s\n", __FUNCTION__);

	dma_async_device_unregister(&swdma_dev->dma_dev);
	switchtec_kobject_del(swdma_dev);
	put_device(swdma_dev->dma_dev.dev);
//	kfree(swdma_dev);
}

static void switchtec_dma_release(struct kref *ref)
{
	struct switchtec_dma_dev *swdma_dev = container_of(ref, struct switchtec_dma_dev, ref);

	dev_dbg(swdma_dev->dma_dev.dev, "%s\n", __FUNCTION__);
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

static void switchtec_dma_free_desc(struct switchtec_dma_chan *swdma_chan)
{
	int i;

	if (swdma_chan->hw_sq)
		kfree(swdma_chan->hw_sq);

	if (swdma_chan->hw_cq)
		kfree(swdma_chan->hw_cq);

	if (swdma_chan->desc_ring) {
		for (i = 0; i < SWITCHTEC_DMA_SQ_SIZE; i++)
			if (swdma_chan->desc_ring[i])
				kfree(swdma_chan->desc_ring[i]);

		kfree(swdma_chan->desc_ring);
	}
}

static int switchtec_dma_alloc_desc(struct switchtec_dma_chan *swdma_chan)
{
	struct switchtec_dma_dev *swdma_dev = swdma_chan->swdma_dev;
	struct chan_fw_regs *chan_fw = swdma_chan->mmio_chan_fw;
	size_t size;
	struct switchtec_dma_desc *desc;
	dma_addr_t dma_addr_sq;
	dma_addr_t dma_addr_cq;
	int i;

	swdma_chan->head = swdma_chan->tail = 0;
	swdma_chan->cq_head = swdma_chan->cq_tail = 0;

	size = SWITCHTEC_DMA_SQ_SIZE * sizeof(*swdma_chan->hw_sq);
	swdma_chan->hw_sq = dmam_alloc_coherent(swdma_dev->dma_dev.dev, size,
						&dma_addr_sq, GFP_KERNEL);
	if (!swdma_chan->hw_sq)
		goto free_and_exit;

	size = SWITCHTEC_DMA_CQ_SIZE * sizeof(*swdma_chan->hw_cq);
	swdma_chan->hw_cq = dmam_alloc_coherent(swdma_dev->dma_dev.dev, size,
						&dma_addr_cq, GFP_KERNEL);
	if (!swdma_chan->hw_cq)
		goto free_and_exit;

	size = sizeof(*swdma_chan->desc_ring);
	swdma_chan->desc_ring = kcalloc(SWITCHTEC_DMA_RING_SIZE,
					size, GFP_KERNEL);
	if (!swdma_chan->desc_ring)
		goto free_and_exit;

	memset(swdma_chan->desc_ring, 0, SWITCHTEC_DMA_SQ_SIZE * size);

	for (i = 0; i < SWITCHTEC_DMA_SQ_SIZE; i++) {
		desc = kzalloc(sizeof(*desc), GFP_KERNEL);
		if (!desc)
			goto free_and_exit;

		dma_async_tx_descriptor_init(&desc->txd, &swdma_chan->dma_chan);
		desc->txd.tx_submit = switchtec_dma_tx_submit;
		desc->hw = &swdma_chan->hw_sq[i];

		swdma_chan->desc_ring[i] = desc;
	}

	/* set sq/cq */
	writel(lower_32_bits(dma_addr_sq), &chan_fw->sq_base_lo);
	writel(upper_32_bits(dma_addr_sq), &chan_fw->sq_base_hi);
	writel(lower_32_bits(dma_addr_cq), &chan_fw->cq_base_lo);
	writel(upper_32_bits(dma_addr_cq), &chan_fw->cq_base_hi);

	writel(cpu_to_le16(SWITCHTEC_DMA_SQ_SIZE),
	       &swdma_chan->mmio_chan_fw->sq_size);
	writel(cpu_to_le16(SWITCHTEC_DMA_CQ_SIZE),
	       &swdma_chan->mmio_chan_fw->cq_size);

	return 0;

free_and_exit:
#if 0
	if (swdma_chan->hw_sq)
		kfree(swdma_chan->hw_sq);

	if (swdma_chan->hw_cq)
		kfree(swdma_chan->hw_cq);

	if (swdma_chan->desc_ring) {
		for (i = 0; i < SWITCHTEC_DMA_SQ_SIZE; i++) {
			if (swdma_chan->desc_ring[j])
				kfree(swdma_chan->desc_ring[j]);

		kfree(swdma_chan->desc_ring);
	}
#endif
	switchtec_dma_free_desc(swdma_chan);

	return -ENOMEM;
}

static int switchtec_dma_alloc_chan_resources(struct dma_chan *chan)
{
	struct switchtec_dma_chan *swdma_chan = to_switchtec_dma_chan(chan);
	struct switchtec_dma_dev *swdma_dev = swdma_chan->swdma_dev;
	u32 perf_cfg;
	int rc;

	dev_dbg(&swdma_chan->dma_chan.dev->device, "%s\n", __FUNCTION__);

	rcu_read_lock();
	if (!rcu_dereference(swdma_dev->pdev)) {
		rcu_read_unlock();
		return -ENODEV;
	}

	kref_get(&swdma_dev->ref);

	rc = switchtec_dma_alloc_desc(swdma_chan);
	if (rc) {
		switchtec_dma_put(swdma_dev);
		rcu_read_unlock();
		return rc;
	}

	enable_channel(swdma_chan);
	reset_channel(swdma_chan);
	unhalt_channel(swdma_chan);

	swdma_chan->ring_active = true;
	swdma_chan->cid = 0;

	dma_cookie_init(chan);

	rcu_read_unlock();

	perf_cfg = readl(&swdma_chan->mmio_chan_fw->perf_cfg);

	dev_info(&chan->dev->device, "burst size  0x%x",
		 (perf_cfg >> PERF_BURST_SIZE_SHIFT) & PERF_BURST_SIZE_MASK);

	dev_info(&chan->dev->device, "burst scale 0x%x",
		 (perf_cfg >> PERF_BURST_SCALE_SHIFT) & PERF_BURST_SCALE_MASK);

	dev_info(&chan->dev->device, "interval    0x%x",
		 (perf_cfg >> PERF_INTERVAL_SHIFT) & PERF_INTERVAL_MASK);

	dev_info(&chan->dev->device, "arb weight  0x%x",
		 (perf_cfg >> PERF_ARB_WEIGHT_SHIFT) & PERF_ARB_WEIGHT_MASK);

	return SWITCHTEC_DMA_SQ_SIZE;
}

static void switchtec_dma_free_chan_resources(struct dma_chan *chan)
{
	struct switchtec_dma_chan *swdma_chan = to_switchtec_dma_chan(chan);
	struct chan_fw_regs *chan_fw = swdma_chan->mmio_chan_fw;
	u32 valid_en_se;
	struct pci_dev *pdev;
	int i;

	dev_dbg(&swdma_chan->dma_chan.dev->device, "%s\n", __FUNCTION__);

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

	/* Disable the channle */
	valid_en_se = readl(&chan_fw->valid_en_se);
	valid_en_se &= !SWITCHTEC_CHAN_ENABLE;
#if 0
	writel(valid_en_se, &chan_fw->valid_en_se);

#endif
	switchtec_dma_put(swdma_chan->swdma_dev);
}

#define SWITCHTEC_DMA_CHAN_HW_REGS_SIZE 0x1000
#define SWITCHTEC_DMA_CHAN_FW_REGS_SIZE 0x80

static int switchtec_dma_chan_init(struct switchtec_dma_dev *swdma_dev,
		struct switchtec_dma_chan *swdma_chan, int i)
{
	struct dma_device *dma = &swdma_dev->dma_dev;
	struct device *dev = &swdma_dev->pdev->dev;
	struct dma_chan *chan;
	struct chan_fw_regs *chan_fw;
	struct chan_hw_regs *chan_hw;
	u32 perf_cfg = 0;
	u32 valid_en_se;
	u32 thresh;
	int se_buf_len;
	int irq;
	int rc;
	size_t offset;

	swdma_chan->index = i;

	offset =  i * SWITCHTEC_DMA_CHAN_FW_REGS_SIZE;
	chan_fw = (struct chan_fw_regs *)(swdma_dev->mmio_chan_fw_all + offset);

	offset =  i * SWITCHTEC_DMA_CHAN_HW_REGS_SIZE;
	chan_hw = (struct chan_hw_regs *)(swdma_dev->mmio_chan_hw_all + offset);

	swdma_dev->swdma_chans[i] = swdma_chan;
	swdma_chan->mmio_chan_fw = chan_fw;
	swdma_chan->mmio_chan_hw = chan_hw;
	swdma_chan->swdma_dev = swdma_dev;

	/* halt channel first */
	rc = halt_channel(swdma_chan);
	if (rc) {
		dev_err(dev, "chan %d: halt channel failed\n", i);
		return -EIO;
	}

	/* init se buffer base/count */
	valid_en_se = (0 << SE_BUF_BASE_SHIFT) | (5 << SE_BUF_LEN_SHIFT);
	writel(valid_en_se, &chan_fw->valid_en_se);

	perf_cfg = readl(&chan_fw->perf_cfg);
	dev_dbg(dev, "chan %d: perf_cfg 0x%x (read)\n", i, perf_cfg);

	/* init perf tuner */
	perf_cfg = PERF_BURST_SCALE << PERF_BURST_SCALE_SHIFT;
	perf_cfg |= PERF_MRRS << PERF_MRRS_SHIFT;
	perf_cfg |= PERF_INTERVAL << PERF_INTERVAL_SHIFT;
	perf_cfg |= PERF_BURST_SIZE << PERF_BURST_SIZE_SHIFT;
	perf_cfg |= PERF_ARB_WEIGHT << PERF_ARB_WEIGHT_SHIFT;

	dev_dbg(dev, "chan %d: perf_cfg 0x%x (write)\n", i, perf_cfg);
	writel(perf_cfg, &chan_fw->perf_cfg);

	valid_en_se = readl(&chan_fw->valid_en_se);

	dev_dbg(dev, "chan %d: valid_en_se 0x%x (read)\n", i, valid_en_se);
	dev_dbg(dev, "chan %d: SE buf base 0x%x (read)\n",
		i, (valid_en_se >> SE_BUF_BASE_SHIFT) & SE_BUF_BASE_MASK);

	se_buf_len = (valid_en_se >> SE_BUF_LEN_SHIFT) & SE_BUF_LEN_MASK;
	dev_dbg(dev, "chan %d: SE buf cnt  0x%d (read)\n", i, se_buf_len);

	thresh = se_buf_len / 2;
	valid_en_se |= (thresh & SE_THRESH_MASK) << SE_THRESH_SHIFT;
	writel(valid_en_se , &chan_fw->valid_en_se);

	/* request irqs */
	irq = readl(&chan_fw->int_vec);
	dev_dbg(dev, "chan %d: irq vec 0x%x\n", i, irq);

	irq = pci_irq_vector(swdma_dev->pdev, irq);
	if (irq < 0)
		return irq;

	rc = devm_request_irq(&swdma_dev->pdev->dev, irq,
			      switchtec_dma_isr, 0, KBUILD_MODNAME,
			      swdma_chan);
	if (rc)
		return rc;

	spin_lock_init(&swdma_chan->ring_lock);
	tasklet_init(&swdma_chan->desc_task, switchtec_dma_desc_task,
		     (unsigned long)swdma_chan);

	chan = &swdma_chan->dma_chan;
	chan->device = dma;
	dma_cookie_init(chan);
	list_add_tail(&chan->device_node, &dma->channels);

	return 0;
}

static int switchtec_dma_chans_enumerate(struct switchtec_dma_dev *swdma_dev)
{
	struct switchtec_dma_chan *swdma_chan;
	struct dma_device *dma = &swdma_dev->dma_dev;
	struct device *dev = &swdma_dev->pdev->dev;
	int chan_cnt;
	int base;
	int cnt;
	int rc;
	int i;

	dev_dbg(dev, "0x1000: %x\n", readb(global_bar + 0x1000));

	chan_cnt = le32_to_cpu(readl(&swdma_dev->mmio_dmac_cap->chan_cnt));

	if (!chan_cnt) {
		pci_err(swdma_dev->pdev, "No channel configured.\n");
		return -ENXIO;
	}

	swdma_dev->swdma_chans = devm_kcalloc(dev, chan_cnt,
					      sizeof(*swdma_dev->swdma_chans),
					      GFP_KERNEL);

	if (!swdma_dev->swdma_chans)
		return -ENOMEM;

#if 1 //user need to setup se buf base/cnt in the config file for each channel
	base = readw(&swdma_dev->mmio_dmac_cap->se_buf_base);
	cnt = readw(&swdma_dev->mmio_dmac_cap->se_buf_cnt);
	dev_dbg(dev, "EP SE buf base: %x\n", base);
	dev_dbg(dev, "EP SE buf cnt:  %d\n", cnt);
#endif
	INIT_LIST_HEAD(&dma->channels);

	for (i = 0; i < chan_cnt; i++) {
		swdma_chan = devm_kzalloc(dev, sizeof(*swdma_chan), GFP_KERNEL);
		if (!swdma_chan)
			return -ENOMEM;

		rc = switchtec_dma_chan_init(swdma_dev, swdma_chan, i);
		if (rc) {
			dev_err(dev, "chan %d: init channel failed\n", i);
			return rc;
		}
	}

	dma->chancnt = chan_cnt;

	return dma->chancnt;
#if 0
error_exit:
	for (j = 0; j < i; j++) {
		kfree(swdma_dev->swdma_chans[j]);
		free_irq(pci_irq_vector(swdma_dev->pdev, 0),
			 swdma_dev->swdma_chans[j]);
	}
	kfree(swdma_dev->swdma_chans);

	return -ENOMEM;
#endif
}

static int switchtec_dma_chans_release(struct switchtec_dma_dev *swdma_dev)
{
	int i;

	for (i = 0; i < swdma_dev->dma_dev.chancnt; i++) {
		kfree(swdma_dev->swdma_chans[i]);
		free_irq(pci_irq_vector(swdma_dev->pdev, 0),
			 swdma_dev->swdma_chans[i]);
	}
	kfree(swdma_dev->swdma_chans);

	return 0;
}

struct switchtec_sysfs_entry {
	struct attribute attr;
	ssize_t (*show)(struct dma_chan *, char *);
	ssize_t (*store)(struct dma_chan *, const char *, size_t);
};

static ssize_t burst_scale_show(struct dma_chan *chan, char *page)
{
	struct switchtec_dma_chan *swdma_chan = to_switchtec_dma_chan(chan);
	struct chan_fw_regs *chan_fw = swdma_chan->mmio_chan_fw;
	u32 perf_cfg = 0;

	perf_cfg = readl(&chan_fw->perf_cfg);
	perf_cfg >>= PERF_BURST_SCALE_SHIFT;
	perf_cfg &= PERF_BURST_SCALE_MASK;

	return sprintf(page, "0x%x\n", perf_cfg);
}

static ssize_t burst_scale_store(struct dma_chan *chan, const char *page, size_t count)
{
	int burst_scale;
	struct switchtec_dma_chan *swdma_chan = to_switchtec_dma_chan(chan);
	struct chan_fw_regs *chan_fw = swdma_chan->mmio_chan_fw;
	u32 perf_cfg;
	ssize_t ret = count;

	rcu_read_lock();
	if (!rcu_dereference(swdma_chan->swdma_dev->pdev)) {
		goto err_unlock;
	}

	if (chan->client_count) {
		goto err_unlock;
	}

	if (sscanf(page, "%du", &burst_scale) != -1) {
		if ((burst_scale < PERF_MAX_BURST_SCALE) || (burst_scale > PERF_MAX_BURST_SCALE)) {
			ret = -EINVAL;
			goto err_unlock;
		}
		perf_cfg = readl(&chan_fw->perf_cfg);
		perf_cfg &= ~(PERF_BURST_SCALE_MASK << PERF_BURST_SCALE_SHIFT);
		perf_cfg |= burst_scale << PERF_BURST_SCALE_SHIFT;
		writel(perf_cfg, &chan_fw->perf_cfg);
	}

err_unlock:
	rcu_read_unlock();

	return ret;
}
struct switchtec_sysfs_entry burst_scale_attr = __ATTR_RW(burst_scale);

static ssize_t burst_size_show(struct dma_chan *chan, char *page)
{
	struct switchtec_dma_chan *swdma_chan = to_switchtec_dma_chan(chan);
	struct chan_fw_regs *chan_fw = swdma_chan->mmio_chan_fw;
	u32 burst_size = 0;

	burst_size = readl(&chan_fw->perf_cfg);
	burst_size >>= PERF_BURST_SIZE_SHIFT;
	burst_size &= PERF_BURST_SIZE_MASK;

	return sprintf(page, "0x%x\n", burst_size);
}

static ssize_t burst_size_store(struct dma_chan *chan, const char *page, size_t count)
{
	int burst_size;
	struct switchtec_dma_chan *swdma_chan = to_switchtec_dma_chan(chan);
	struct chan_fw_regs *chan_fw = swdma_chan->mmio_chan_fw;
	u32 perf_cfg;
	ssize_t ret = count;

	rcu_read_lock();
	if (!rcu_dereference(swdma_chan->swdma_dev->pdev)) {
		goto err_unlock;
	}

	if (chan->client_count) {
		goto err_unlock;
	}

	if (sscanf(page, "%du", &burst_size) != -1) {
		if ((burst_size < PERF_MIN_BURST_SIZE) || (burst_size > PERF_MAX_BURST_SIZE)) {
			ret = -EINVAL;
			goto err_unlock;
		}
		perf_cfg = readl(&chan_fw->perf_cfg);
		perf_cfg &= ~(PERF_BURST_SIZE_MASK << PERF_BURST_SIZE_SHIFT);
		perf_cfg |= burst_size << PERF_BURST_SIZE_SHIFT;
		writel(perf_cfg, &chan_fw->perf_cfg);
	}

err_unlock:
	rcu_read_unlock();

	return ret;
}


struct switchtec_sysfs_entry burst_size_attr = __ATTR_RW(burst_size);

static ssize_t arb_weight_show(struct dma_chan *chan, char *page)
{
	struct switchtec_dma_chan *swdma_chan = to_switchtec_dma_chan(chan);
	struct chan_fw_regs *chan_fw = swdma_chan->mmio_chan_fw;
	u32 weight = 0;

	weight = readl(&chan_fw->perf_cfg);
	weight >>= PERF_ARB_WEIGHT_SHIFT;
	weight &= PERF_ARB_WEIGHT_MASK;

	return sprintf(page, "0x%x\n", weight);
}

static ssize_t arb_weight_store(struct dma_chan *chan, const char *page, size_t count)
{
	int weight;
	struct switchtec_dma_chan *swdma_chan = to_switchtec_dma_chan(chan);
	struct chan_fw_regs *chan_fw = swdma_chan->mmio_chan_fw;
	u32 perf_cfg;
	ssize_t ret = count;

	rcu_read_lock();
	if (!rcu_dereference(swdma_chan->swdma_dev->pdev)) {
		goto err_unlock;
	}

	if (chan->client_count) {
		goto err_unlock;
	}

	if (sscanf(page, "%du", &weight) != -1) {
		if (weight < 0) {
			ret = -EINVAL;
			goto err_unlock;
		}
		perf_cfg = readl(&chan_fw->perf_cfg);
		perf_cfg &= ~(PERF_ARB_WEIGHT_MASK << PERF_ARB_WEIGHT_SHIFT);
		perf_cfg |= weight << PERF_ARB_WEIGHT_SHIFT;
		writel(perf_cfg, &chan_fw->perf_cfg);
	}

err_unlock:
	rcu_read_unlock();

	return ret;
}
struct switchtec_sysfs_entry arb_weight_attr = __ATTR_RW(arb_weight);

static ssize_t interval_show(struct dma_chan *chan, char *page)
{
	struct switchtec_dma_chan *swdma_chan = to_switchtec_dma_chan(chan);
	struct chan_fw_regs *chan_fw = swdma_chan->mmio_chan_fw;
	u32 interval = 0;

	interval = readl(&chan_fw->perf_cfg);
	interval >>= PERF_INTERVAL_SHIFT;
	interval &= PERF_INTERVAL_MASK;

	return sprintf(page, "%d\n", interval);
}

static ssize_t interval_store(struct dma_chan *chan, const char *page, size_t count)
{
	int interval;
	struct switchtec_dma_chan *swdma_chan = to_switchtec_dma_chan(chan);
	struct chan_fw_regs *chan_fw = swdma_chan->mmio_chan_fw;
	u32 perf_cfg;
	ssize_t ret = count;

	rcu_read_lock();
	if (!rcu_dereference(swdma_chan->swdma_dev->pdev)) {
		goto err_unlock;
	}

	if (chan->client_count) {
		goto err_unlock;
	}

	if (sscanf(page, "%du", &interval) != -1) {
		if ((interval < PERF_MIN_INTERVAL) || (interval > PERF_MAX_INTERVAL)) {
			ret = -EINVAL;
			goto err_unlock;
		}
		perf_cfg = readl(&chan_fw->perf_cfg);
		perf_cfg &= ~(PERF_INTERVAL_MASK << PERF_INTERVAL_SHIFT);
		perf_cfg |= interval << PERF_INTERVAL_SHIFT;
		writel(perf_cfg, &chan_fw->perf_cfg);
	}

err_unlock:
	rcu_read_unlock();

	return ret;
}

struct switchtec_sysfs_entry interval_attr = __ATTR_RW(interval);

static ssize_t
switchtec_attr_show(struct kobject *kobj, struct attribute *attr, char *page)
{
	struct switchtec_sysfs_entry *entry;
	struct switchtec_dma_chan *swdma_chan;

	entry = container_of(attr, struct switchtec_sysfs_entry, attr);
	swdma_chan = container_of(kobj, struct switchtec_dma_chan, kobj);

	if (!entry->show)
		return -EIO;
	return entry->show(&swdma_chan->dma_chan, page);
}

static ssize_t
switchtec_attr_store(struct kobject *kobj, struct attribute *attr,
const char *page, size_t count)
{
	struct switchtec_sysfs_entry *entry;
	struct switchtec_dma_chan *swdma_chan;

	entry = container_of(attr, struct switchtec_sysfs_entry, attr);
	swdma_chan = container_of(kobj, struct switchtec_dma_chan, kobj);

	if (!entry->store)
		return -EIO;
	return entry->store(&swdma_chan->dma_chan, page, count);
}

const struct sysfs_ops switchtec_sysfs_ops = {
	.show	= switchtec_attr_show,
	.store  = switchtec_attr_store,
};

static struct attribute *switchtec_attrs[] = {
	&burst_scale_attr.attr,
	&burst_size_attr.attr,
	&interval_attr.attr,
	&arb_weight_attr.attr,
	NULL,
};

struct kobj_type switchtec_ktype = {
	.sysfs_ops = &switchtec_sysfs_ops,
	.default_attrs = switchtec_attrs,
};

void switchtec_kobject_add(struct switchtec_dma_dev *swdma_dev, struct kobj_type *type)
{
	struct dma_device *dma = &swdma_dev->dma_dev;
	struct dma_chan *c;

	list_for_each_entry(c, &dma->channels, device_node) {
		struct switchtec_dma_chan *swdma_chan = to_switchtec_dma_chan(c);
		struct kobject *parent = &c->dev->device.kobj;
		int err;

		err = kobject_init_and_add(&swdma_chan->kobj, type,
					   parent, "perf_cfg");
		if (err) {
			dev_warn(&swdma_chan->dma_chan.dev->device,
				 "sysfs init error (%d), continuing...\n", err);
			kobject_put(&swdma_chan->kobj);
		}
	}
}

void switchtec_kobject_del(struct switchtec_dma_dev *swdma_dev)
{
	struct dma_device *dma = &swdma_dev->dma_dev;
	struct dma_chan *c;

	list_for_each_entry(c, &dma->channels, device_node) {
		struct switchtec_dma_chan *swdma_chan = to_switchtec_dma_chan(c);

//		if (!test_bit(IOAT_KOBJ_INIT_FAIL, &ioat_chan->state)) {
		if (swdma_chan->kobj.state_initialized) {
			kobject_del(&swdma_chan->kobj);
			kobject_put(&swdma_chan->kobj);
		}
//		}
	}
}

static int switchtec_dma_create(struct pci_dev *pdev)
{
	struct switchtec_dma_dev *swdma_dev;
	struct dma_device *dma;
	struct dma_chan *chan;
	struct device *dev = &pdev->dev;
	void __iomem *bar;
	int chan_cnt;
	int rc;

	/*
	 * Create the switchtec dma device
	 */
	swdma_dev = devm_kzalloc(dev, sizeof(*swdma_dev), GFP_KERNEL);
	if (!swdma_dev)
		return -ENOMEM;

	bar = pcim_iomap_table(pdev)[0];
	global_bar = bar;

	pci_dbg(pdev, "bar: 0x%lx\n", (unsigned long)bar);
	swdma_dev->mmio_dmac_ver = bar + SWITCHTEC_DMAC_VERSION_OFFSET;
	swdma_dev->mmio_dmac_cap = bar + SWITCHTEC_DMAC_CAPABILITY_OFFSET;
	swdma_dev->mmio_dmac_status = bar + SWITCHTEC_DMAC_STATUS_OFFSET;
	swdma_dev->mmio_dmac_ctrl = bar + SWITCHTEC_DMAC_CONTROL_OFFSET;
	swdma_dev->mmio_chan_hw_all = bar + SWITCHTEC_DMAC_CHAN_CTRL_OFFSET;
	swdma_dev->mmio_chan_fw_all = bar + SWITCHTEC_DMAC_CHAN_CFG_STS_OFFSET;

	RCU_INIT_POINTER(swdma_dev->pdev, pdev);

	chan_cnt = switchtec_dma_chans_enumerate(swdma_dev);
	if (chan_cnt < 0) {
		kfree(swdma_dev);
		pci_err(pdev, "Failed to enumerate dma channels: %d\n", chan_cnt);
		return chan_cnt;
	}

	dma = &swdma_dev->dma_dev;
	pci_info(pdev, "channel cnt: %d\n", dma->chancnt);
	dma->copy_align = DMAENGINE_ALIGN_1_BYTE;
	dma_cap_set(DMA_MEMCPY | DMA_PRIVATE, dma->cap_mask);
	dma->dev = get_device(&pdev->dev);

	kref_init(&swdma_dev->ref);
	INIT_WORK(&swdma_dev->release_work, switchtec_dma_release_work);

#if 1
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

	list_for_each_entry(chan, &dma->channels, device_node) {
		pci_dbg(pdev, "chan name: %s\n", dma_chan_name(chan));
	}

#endif
	pci_set_drvdata(pdev, swdma_dev);

	switchtec_kobject_add(swdma_dev, &switchtec_ktype);

	return 0;
}


static int switchtec_dma_probe(struct pci_dev *pdev,
			       const struct pci_device_id *id)
{
	int rc;

	rc = pcim_enable_device(pdev);
	if (rc)
		return rc;


	rc = pci_set_dma_mask(pdev, DMA_BIT_MASK(64));
	if (rc)
		rc = pci_set_dma_mask(pdev, DMA_BIT_MASK(32));
	if (rc)
		return rc;

	rc = pci_set_consistent_dma_mask(pdev, DMA_BIT_MASK(64));
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


	pci_info(pdev, "Switchtec DMA Channels Registered\n");

	return 0;

err_free_irq_vectors:
	pci_free_irq_vectors(pdev);
	return rc;
}

static void switchtec_dma_remove(struct pci_dev *pdev)
{
	struct switchtec_dma_dev *swdma_dev = pci_get_drvdata(pdev);
//	struct switchtec_dma_chan *swdma_chan;
//	int i;

//	free_irq(pci_irq_vector(pdev, 0),  swdma_dev);

	rcu_assign_pointer(swdma_dev->pdev, NULL);
	synchronize_rcu();

//	for (i = 0; i < SWITCHTEC_DMA_SQ_SIZE; i++) {
//		swdma_chan = swdma_dev->swdma_chans[i];
//
//		spin_lock_bh(&swdma_chan->ring_lock);
//		swdma_chan->ring_active = false;
//		spin_unlock_bh(&swdma_chan->ring_lock);
//		switchtec_dma_abort_desc(swdma_chan);
//	}

//	__switchtec_dma_stop(swdma_dev);

	swdma_dev->mmio_dmac_ver = NULL;
	swdma_dev->mmio_dmac_cap = NULL;
	swdma_dev->mmio_dmac_status = NULL;
	swdma_dev->mmio_dmac_ctrl = NULL;
	swdma_dev->mmio_chan_hw_all = NULL;
	swdma_dev->mmio_chan_fw_all = NULL;

	switchtec_dma_put(swdma_dev);

//	pci_free_irq_vectors(pdev);

	pci_info(pdev, "Switchtec DMA Channels Unregistered\n");
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
