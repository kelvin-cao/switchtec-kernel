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

enum {
	SWITCHTEC_CHAN_CFG_STS_OFFSET = 0x160000,
};

struct plx_dma_desc {
	struct dma_async_tx_descriptor txd;
	struct plx_dma_hw_std_desc *hw;
	u32 orig_size;
};

struct ctrl_sts_regs {
	union {
		struct {
			u32 fw_ver;
			u32 dma_prot_ver;
			u32 hw_dmac_ver;
		} ver;
		u32 rsvd1[32];
	};
	union {
		struct {
			u32 capability;
			u32 chan_cnt;
			u32 chan_weight_sum;
			u32 completion_tmo;
			u32 tag_limit;
			u32 ctrl_vect;
			u16 se_buf_cnt;
			u16 se_buf_base;
		} cap;
		u32 rsvd2[32];
	};
	union {
		struct {
			u32 ready;
			u32 paused;
		} status;
		u32 rsvd3[32];
	};
	union {
		struct {
			u32 dma_reset;
		} ctrl;
		u32 rsvd4[32];
	};
	union {
		struct {
		} intr;
		u32 rsvd5[32];
	};
} __packed;

struct chan_cfg_sts_regs {
	union {
		/* pseudo */
		struct {
			u32 se_buf_base;
			u32 se_buf_cnt;
			u32 burst_size;
			u32 burst_scale;
			u32 interval;
			u32 mrrs;
			u32 int_vec;
		};
		u32 rsvd[32];
	};
} __packed;

#define SWITCHTEC_CHAN_INTERVAL 1
#define SWITCHTEC_CHAN_BURST_SZ 1
#define SWITCHTEC_CHAN_BURST_SCALE 1
#define SWITCHTEC_CHAN_MRRS 1
struct switchtec_dma_chan {
	struct switchtec_dma_dev *swdma_dev;
	struct dma_chan dma_chan;

	struct tasklet_struct desc_task;
	spinlock_t ring_lock;
	bool ring_active;
	int head;
	int tail;
	struct switchtec_dma_hw_se_desc *hw_sq;
	struct switchtec_dma_hw_ce_desc *hw_cq;
	struct switchtec_dma_desc **desc_ring;
#if 0
	/* performance */
	u32 se_buf_cnt;
	u32 interval;
	u32 burst_sz;
	u32 burst_scale;
	u32 max_read_req_sz;
	u32 arb_weight;
#endif
};

static struct switchtec_dma_chan *chan_to_switchtec_dma_chan(struct dma_chan *c)
{
	return container_of(c, struct switchtec_dma_chan, dma_chan);
}
#if 0
static struct switchtec_dma_dev *chan_to_switchtec_dma_dev(struct dma_chan *c)
{
	struct switchtec_dma_chan *swdma_chan;

	swdma_chan = container_of(c, struct switchtec_dma_chan, dma_chan);
	return swdma_chan->swdma_dev;
}
#endif

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
	__le16 cmd_id;
	__le32 byte_cnt;
	__le32 saddrlo_widlo;
	__le32 saddrhi_widhi;
	__le32 daddrlo;
	__le32 daddrhi;
	__le16 dfid_connid;
	__le16 sfid;
};

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

struct switchtec_dma_hw_ce_desc {
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
	u32 orig_size;
};
#define SWITCHTEC_DMA_SQ_SIZE (16 * 1024)
#define SWITCHTEC_DMA_CQ_SIZE (16 * 1024)

struct switchtec_dma_dev {
	struct dma_device dma_dev;
	struct pci_dev __rcu *pdev;
	struct switchtec_dma_chan **swdma_chan;
	struct ctrl_sts_regs __iomem *mmio_ctrl_sts;
	struct chan_cfg_sts_regs __iomem *mmio_chan_cfg_sts_all;

	struct kref ref;
	struct work_struct release_work;
};

#if 0
static struct plx_dma_dev *chan_to_plx_dma_dev(struct dma_chan *c)
{
	return container_of(c, struct plx_dma_dev, dma_chan);
}
#endif

static struct switchtec_dma_desc *
switchtec_dma_get_desc(struct switchtec_dma_chan *swdma_chan, int i)
{
	return swdma_chan->desc_ring[i & (SWITCHTEC_DMA_SQ_SIZE - 1)];
}

#if 1
static void switchtec_dma_process_desc(struct switchtec_dma_chan *swdma_chan)
{
#if 0
	struct dmaengine_result res;
	struct plx_dma_desc *desc;
	u32 flags;

	spin_lock_bh(&plxdev->ring_lock);

	while (plxdev->tail != plxdev->head) {
		desc = plx_dma_get_desc(plxdev, plxdev->tail);

		flags = le32_to_cpu(READ_ONCE(desc->hw->flags_and_size));

		if (flags & PLX_DESC_FLAG_VALID)
			break;

		res.residue = desc->orig_size - (flags & PLX_DESC_SIZE_MASK);

		if (flags & PLX_DESC_WB_SUCCESS)
			res.result = DMA_TRANS_NOERROR;
		else if (flags & PLX_DESC_WB_WR_FAIL)
			res.result = DMA_TRANS_WRITE_FAILED;
		else
			res.result = DMA_TRANS_READ_FAILED;

		dma_cookie_complete(&desc->txd);
		dma_descriptor_unmap(&desc->txd);
		dmaengine_desc_get_callback_invoke(&desc->txd, &res);
		desc->txd.callback = NULL;
		desc->txd.callback_result = NULL;

		plxdev->tail++;
	}

	spin_unlock_bh(&plxdev->ring_lock);
#endif
}
#endif
static void switchtec_dma_abort_desc(struct switchtec_dma_chan *swdma_chan)
{
	struct dmaengine_result res;
	struct switchtec_dma_desc *desc;

	switchtec_dma_process_desc(swdma_chan);

	spin_lock_bh(&swdma_chan->ring_lock);

	while (swdma_chan->tail != swdma_chan->head) {
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

static void __switchtec_dma_stop(struct switchtec_dma_dev *swdma_dev)
{
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
static void plx_dma_desc_task(unsigned long data)
{
	struct plx_dma_dev *plxdev = (void *)data;

	plx_dma_process_desc(plxdev);
}

static struct dma_async_tx_descriptor *plx_dma_prep_memcpy(struct dma_chan *c,
		dma_addr_t dma_dst, dma_addr_t dma_src, size_t len,
		unsigned long flags)
	__acquires(plxdev->ring_lock)
{
	struct plx_dma_dev *plxdev = chan_to_plx_dma_dev(c);
	struct plx_dma_desc *plxdesc;

	spin_lock_bh(&plxdev->ring_lock);
	if (!plxdev->ring_active)
		goto err_unlock;

	if (!CIRC_SPACE(plxdev->head, plxdev->tail, PLX_DMA_RING_COUNT))
		goto err_unlock;

	if (len > PLX_DESC_SIZE_MASK)
		goto err_unlock;

	plxdesc = plx_dma_get_desc(plxdev, plxdev->head);
	plxdev->head++;

	plxdesc->hw->dst_addr_lo = cpu_to_le32(lower_32_bits(dma_dst));
	plxdesc->hw->dst_addr_hi = cpu_to_le16(upper_32_bits(dma_dst));
	plxdesc->hw->src_addr_lo = cpu_to_le32(lower_32_bits(dma_src));
	plxdesc->hw->src_addr_hi = cpu_to_le16(upper_32_bits(dma_src));

	plxdesc->orig_size = len;

	if (flags & DMA_PREP_INTERRUPT)
		len |= PLX_DESC_FLAG_INT_WHEN_DONE;

	plxdesc->hw->flags_and_size = cpu_to_le32(len);
	plxdesc->txd.flags = flags;

	/* return with the lock held, it will be released in tx_submit */

	return &plxdesc->txd;

err_unlock:
	/*
	 * Keep sparse happy by restoring an even lock count on
	 * this lock.
	 */
	__acquire(plxdev->ring_lock);

	spin_unlock_bh(&plxdev->ring_lock);
	return NULL;
}
#endif
static dma_cookie_t switchtec_dma_tx_submit(struct dma_async_tx_descriptor *desc)
	__releases(swdma_dev->ring_lock)
{
	struct switchtec_dma_chan *swdma_chan =
		chan_to_switchtec_dma_chan(desc->chan);
//	struct switchtec_dma_desc *swdma_desc = to_switchtec_desc(desc);
	dma_cookie_t cookie;

	cookie = dma_cookie_assign(desc);

	/*
	 * Ensure the descriptor updates are visible to the dma device
	 * before setting the valid bit.
	 */
	wmb();

//	plxdesc->hw->flags_and_size |= cpu_to_le32(PLX_DESC_FLAG_VALID);

	spin_unlock_bh(&swdma_chan->ring_lock);

	return cookie;
}
#if 0
static enum dma_status plx_dma_tx_status(struct dma_chan *chan,
		dma_cookie_t cookie, struct dma_tx_state *txstate)
{
	struct plx_dma_dev *plxdev = chan_to_plx_dma_dev(chan);
	enum dma_status ret;

	ret = dma_cookie_status(chan, cookie, txstate);
	if (ret == DMA_COMPLETE)
		return ret;

	plx_dma_process_desc(plxdev);

	return dma_cookie_status(chan, cookie, txstate);
}

static void plx_dma_issue_pending(struct dma_chan *chan)
{
	struct plx_dma_dev *plxdev = chan_to_plx_dma_dev(chan);

	rcu_read_lock();
	if (!rcu_dereference(plxdev->pdev)) {
		rcu_read_unlock();
		return;
	}

	/*
	 * Ensure the valid bits are visible before starting the
	 * DMA engine.
	 */
	wmb();

	writew(PLX_REG_CTRL_START_VAL, plxdev->bar + PLX_REG_CTRL);

	rcu_read_unlock();
}

#endif
static irqreturn_t switchtec_dma_isr(int irq, void *devid)
{
#if 0
	struct plx_dma_dev *plxdev = devid;
	u32 status;

	status = readw(plxdev->bar + PLX_REG_INTR_STATUS);

	if (!status)
		return IRQ_NONE;

	if (status & PLX_REG_INTR_STATUS_DESC_DONE && plxdev->ring_active)
		tasklet_schedule(&plxdev->desc_task);

	writew(status, plxdev->bar + PLX_REG_INTR_STATUS);
#endif
	return IRQ_HANDLED;
}

static void switchtec_dma_release_work(struct work_struct *work)
{
#if 0
	struct plx_dma_dev *plxdev = container_of(work, struct plx_dma_dev,
						  release_work);

	dma_async_device_unregister(&plxdev->dma_dev);
	put_device(plxdev->dma_dev.dev);
	kfree(plxdev);
#endif
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
	dma_addr_t dma_addr;
	int rc;

	rcu_read_lock();
	if (!rcu_dereference(swdma_dev->pdev)) {
		rcu_read_unlock();
		return -ENODEV;
	}

	kref_get(&swdma_dev->ref);

//	writel(PLX_REG_CTRL_RESET_VAL, plxdev->bar + PLX_REG_CTRL);

	swdma_chan->hw_sq = dmam_alloc_coherent(swdma_dev->dma_dev.dev,
						sq_sz, &dma_addr,
						GFP_KERNEL);
	if (!swdma_chan->hw_sq) {
		rcu_read_unlock();
		return -ENOMEM;
	}

	swdma_chan->hw_cq = dmam_alloc_coherent(swdma_dev->dma_dev.dev,
						cq_sz, &dma_addr,
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
#if 0
	writel(lower_32_bits(dma_addr), plxdev->bar + PLX_REG_DESC_RING_ADDR);
	writel(upper_32_bits(dma_addr),
	       plxdev->bar + PLX_REG_DESC_RING_ADDR_HI);
	writel(lower_32_bits(dma_addr),
	       plxdev->bar + PLX_REG_DESC_RING_NEXT_ADDR);
	writel(PLX_DMA_RING_COUNT, plxdev->bar + PLX_REG_DESC_RING_COUNT);
	writel(PLX_REG_PREF_LIMIT_PREF_FOUR, plxdev->bar + PLX_REG_PREF_LIMIT);
#endif
	swdma_chan->ring_active = true;

	rcu_read_unlock();

	return SWITCHTEC_DMA_SQ_SIZE;
}
#if 0
static void plx_dma_free_chan_resources(struct dma_chan *chan)
{
	struct plx_dma_dev *plxdev = chan_to_plx_dma_dev(chan);
	struct pci_dev *pdev;
	int i;

	spin_lock_bh(&plxdev->ring_lock);
	plxdev->ring_active = false;
	spin_unlock_bh(&plxdev->ring_lock);

	plx_dma_stop(plxdev);

	rcu_read_lock();
	pdev = rcu_dereference(plxdev->pdev);
	if (pdev)
		synchronize_irq(pci_irq_vector(pdev, 0));
	rcu_read_unlock();

	tasklet_kill(&plxdev->desc_task);

//	plx_dma_abort_desc(plxdev);

	for (i = 0; i < PLX_DMA_RING_COUNT; i++)
		kfree(plxdev->desc_ring[i]);

	kfree(plxdev->desc_ring);

//	plx_dma_put(plxdev);
}
#endif

static int switchtec_dma_chans_create(struct switchtec_dma_dev *swdma_dev)
{
	struct switchtec_dma_chan *swdma_chan;
	struct dma_device *dma = &swdma_dev->dma_dev;
	struct dma_chan *chan;
	struct chan_cfg_sts_regs *chan_cfg_sts;
	int chan_cnt = readl(&swdma_dev->mmio_ctrl_sts->cap.chan_cnt);
	int base;
	int cnt;
	int vec;
	int i;
	int rc;

	swdma_dev->swdma_chan =
		kcalloc(chan_cnt, sizeof(*swdma_dev->swdma_chan), GFP_KERNEL);

	base = readl(&swdma_dev->mmio_ctrl_sts->cap.se_buf_base);
	cnt = readl(&swdma_dev->mmio_ctrl_sts->cap.se_buf_cnt) / chan_cnt;

	INIT_LIST_HEAD(&dma->channels);

	for (i = 0; i < chan_cnt; i++) {
		chan_cfg_sts = &swdma_dev->mmio_chan_cfg_sts_all[i];

		/* init se buffer base/count */
		writel(cnt, &chan_cfg_sts->se_buf_cnt);
		writel(base + cnt * i, &chan_cfg_sts->se_buf_base);

		/* init perf tuner */
		writel(SWITCHTEC_CHAN_BURST_SZ, &chan_cfg_sts->burst_size);
		writel(SWITCHTEC_CHAN_BURST_SCALE, &chan_cfg_sts->burst_scale);
		writel(SWITCHTEC_CHAN_INTERVAL, &chan_cfg_sts->interval);
		writel(SWITCHTEC_CHAN_MRRS, &chan_cfg_sts->mrrs);

		swdma_chan = swdma_dev->swdma_chan[i];
		spin_lock_init(&swdma_chan->ring_lock);
//		tasklet_init(&plxdev->desc_task, plx_dma_desc_task,
//		     (unsigned long)swdma_chan);

		chan = &swdma_chan->dma_chan;
		chan->device = dma;
		dma_cookie_init(chan);
		list_add_tail(&chan->device_node, &dma->channels);

		/* request irqs */
		vec = readl(&chan_cfg_sts->int_vec);
		rc = request_irq(pci_irq_vector(swdma_dev->pdev, vec),
				 switchtec_dma_isr, 0,KBUILD_MODNAME, swdma_dev);
		if (rc) {
			kfree(swdma_dev->swdma_chan);
			return rc;
		}
	}

	dma->chancnt = chan_cnt;

	return dma->chancnt;
}

static int switchtec_dma_create(struct pci_dev *pdev)
{
	struct switchtec_dma_dev *swdma_dev;
	struct dma_device *dma;
	void __iomem *bar;
	int rc;

	/*
	 * Create the switchtec dma device
	 */
	swdma_dev = kzalloc(sizeof(*swdma_dev), GFP_KERNEL);
	if (!swdma_dev)
		return -ENOMEM;

	bar = pcim_iomap_table(pdev)[0];
	swdma_dev->mmio_ctrl_sts = bar;
	swdma_dev->mmio_chan_cfg_sts_all = bar + SWITCHTEC_CHAN_CFG_STS_OFFSET;

	switchtec_dma_chans_create(swdma_dev);

	kref_init(&swdma_dev->ref);
	INIT_WORK(&swdma_dev->release_work, switchtec_dma_release_work);
	RCU_INIT_POINTER(swdma_dev->pdev, pdev);

	dma = &swdma_dev->dma_dev;
	dma_cap_set(DMA_MEMCPY, dma->cap_mask);
	dma->copy_align = DMAENGINE_ALIGN_1_BYTE;
	dma->dev = get_device(&pdev->dev);

	dma->device_alloc_chan_resources = switchtec_dma_alloc_chan_resources;
//	dma->device_free_chan_resources = plx_dma_free_chan_resources;
//	dma->device_prep_dma_memcpy = plx_dma_prep_memcpy;
//	dma->device_issue_pending = plx_dma_issue_pending;
//	dma->device_tx_status = plx_dma_tx_status;

	rc = dma_async_device_register(dma);
	if (rc) {
		pci_err(pdev, "Failed to register dma device: %d\n", rc);
		free_irq(pci_irq_vector(pdev, 0),  swdma_dev);
		goto error_exit;
	}

	pci_set_drvdata(pdev, swdma_dev);

	return 0;

error_exit:
	kfree(swdma_dev->swdma_chan);
	kfree(swdma_dev);
	return rc;
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

	swdma_dev->mmio_ctrl_sts = NULL;
	swdma_dev->mmio_chan_cfg_sts_all = NULL;
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
