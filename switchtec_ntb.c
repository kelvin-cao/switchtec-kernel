/*
 * Microsemi Switchtec(tm) PCIe Management Driver
 * Copyright (c) 2017, Microsemi Corporation
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 */

#include <linux/switchtec.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/kthread.h>
#include <linux/ntb.h>

MODULE_DESCRIPTION("Microsemi Switchtec(tm) NTB Driver");
MODULE_VERSION("0.1");
MODULE_LICENSE("GPL");
MODULE_AUTHOR("Microsemi Corporation");

static ulong max_mw_size = SZ_2M;
module_param(max_mw_size, ulong, 0644);
MODULE_PARM_DESC(max_mw_size,
	"Limit the size of the memory windows reported to the upper layer");

static bool use_lut_mws;
module_param(use_lut_mws, bool, 0644);
MODULE_PARM_DESC(use_lut_mws,
		 "Enable the use of the LUT based memory windows");

#ifndef ioread64
#ifdef readq
#define ioread64 readq
#else
#define ioread64 _ioread64
static inline u64 _ioread64(void __iomem *mmio)
{
	u64 low, high;

	low = ioread32(mmio);
	high = ioread32(mmio + sizeof(u32));
	return low | (high << 32);
}
#endif
#endif

#ifndef iowrite64
#ifdef writeq
#define iowrite64 writeq
#else
#define iowrite64 _iowrite64
static inline void _iowrite64(u64 val, void __iomem *mmio)
{
	iowrite32(val, mmio);
	iowrite32(val >> 32, mmio + sizeof(u32));
}
#endif
#endif

#define SWITCHTEC_NTB_MAGIC 0x45CC0001

struct shared_mw {
	u32 magic;
	u32 link_sta;
	u32 partition_id;
	u32 spad[128];
};

#define MAX_DIRECT_MW ARRAY_SIZE(((struct ntb_ctrl_regs *)(0))->bar_entry)
#define LUT_SIZE SZ_64K

struct switchtec_ntb {
	struct ntb_dev ntb;
	struct switchtec_dev *stdev;

	int self_partition;
	int peer_partition;

	int doorbell_irq;
	int message_irq;

	struct ntb_info_regs __iomem *mmio_ntb;
	struct ntb_ctrl_regs __iomem *mmio_ctrl;
	struct ntb_dbmsg_regs __iomem *mmio_dbmsg;
	struct ntb_ctrl_regs __iomem *mmio_self_ctrl;
	struct ntb_ctrl_regs __iomem *mmio_peer_ctrl;
	struct ntb_dbmsg_regs __iomem *mmio_self_dbmsg;

	struct shared_mw *self_shared;
	struct shared_mw __iomem *peer_shared;
	dma_addr_t self_shared_dma;

	u64 db_mask;
	u64 db_valid_mask;
	int db_shift;
	int db_peer_shift;

	/* synchronize rmw access of db_mask and hw reg */
	spinlock_t db_mask_lock;

	int nr_direct_mw;
	int nr_lut_mw;
	int direct_mw_to_bar[MAX_DIRECT_MW];

	bool link_is_up;
	enum ntb_speed link_speed;
	enum ntb_width link_width;
	struct notifier_block link_notifier;
};

static struct switchtec_ntb *ntb_sndev(struct ntb_dev *ntb)
{
	return container_of(ntb, struct switchtec_ntb, ntb);
}

static int switchtec_ntb_part_op(struct switchtec_ntb *sndev,
				 struct ntb_ctrl_regs __iomem *ctl,
				 u32 op, int wait_status)
{
	static const char * const op_text[] = {
		[NTB_CTRL_PART_OP_LOCK] = "lock",
		[NTB_CTRL_PART_OP_CFG] = "configure",
		[NTB_CTRL_PART_OP_RESET] = "reset",
	};

	int i;
	u32 ps;
	int status;

	switch (op) {
	case NTB_CTRL_PART_OP_LOCK:
		status = NTB_CTRL_PART_STATUS_LOCKING;
		break;
	case NTB_CTRL_PART_OP_CFG:
		status = NTB_CTRL_PART_STATUS_CONFIGURING;
		break;
	case NTB_CTRL_PART_OP_RESET:
		status = NTB_CTRL_PART_STATUS_RESETTING;
		break;
	default:
		return -EINVAL;
	}

	iowrite32(op, &ctl->partition_op);

	for (i = 0; i < 1000; i++) {
		if (msleep_interruptible(50) != 0) {
			iowrite32(NTB_CTRL_PART_OP_RESET, &ctl->partition_op);
			return -EINTR;
		}

		ps = ioread32(&ctl->partition_status) & 0xFFFF;

		if (ps != status)
			break;
	}

	if (ps == wait_status)
		return 0;

	if (ps == status) {
		dev_err(&sndev->stdev->dev,
			"Timed out while peforming %s (%d). (%08x)",
			op_text[op], op,
			ioread32(&ctl->partition_status));

		return -ETIMEDOUT;
	}

	return -EIO;
}

static int switchtec_ntb_send_msg(struct switchtec_ntb *sndev, int idx,
				  u32 val)
{
	if (idx < 0 || idx >= ARRAY_SIZE(sndev->mmio_self_dbmsg->omsg))
		return -EINVAL;

	iowrite32(val, &sndev->mmio_self_dbmsg->omsg[idx].msg);

	return 0;
}

static int switchtec_ntb_mw_count(struct ntb_dev *ntb)
{
	struct switchtec_ntb *sndev = ntb_sndev(ntb);

	if (use_lut_mws)
		return sndev->nr_direct_mw + sndev->nr_lut_mw - 1;
	else
		return sndev->nr_direct_mw;
}

static int lut_index(struct switchtec_ntb *sndev, int mw_idx)
{
	return mw_idx - sndev->nr_direct_mw + 1;
}

static int switchtec_ntb_mw_direct_get_range(struct switchtec_ntb *sndev,
					     int idx, phys_addr_t *base,
					     resource_size_t *size,
					     resource_size_t *align,
					     resource_size_t *align_size)
{
	int bar = sndev->direct_mw_to_bar[idx];
	size_t offset = 0;

	if (bar < 0)
		return -EINVAL;

	if (idx == 0) {
		/*
		 * This is the direct BAR shared with the LUTs
		 * which means the actual window will be offset
		 * by the size of all the LUT entries.
		 */

		offset = LUT_SIZE * sndev->nr_lut_mw;
	}

	if (base)
		*base = pci_resource_start(sndev->ntb.pdev, bar) + offset;

	if (size) {
		*size = pci_resource_len(sndev->ntb.pdev, bar) - offset;
		if (offset && *size > offset)
			*size = offset;

		if (*size > max_mw_size)
			*size = max_mw_size;
	}

	if (align)
		*align = SZ_4K;

	if (align_size)
		*align_size = SZ_4K;

	return 0;
}

static int switchtec_ntb_mw_lut_get_range(struct switchtec_ntb *sndev,
					  int idx, phys_addr_t *base,
					  resource_size_t *size,
					  resource_size_t *align,
					  resource_size_t *align_size)
{
	int bar = sndev->direct_mw_to_bar[0];
	int offset;

	offset = LUT_SIZE * lut_index(sndev, idx);

	if (base)
		*base = pci_resource_start(sndev->ntb.pdev, bar) + offset;

	if (size)
		*size = LUT_SIZE;

	if (align)
		*align = LUT_SIZE;

	if (align_size)
		*align_size = LUT_SIZE;

	return 0;
}

static int switchtec_ntb_mw_get_range(struct ntb_dev *ntb, int idx,
				      phys_addr_t *base,
				      resource_size_t *size,
				      resource_size_t *align,
				      resource_size_t *align_size)
{
	struct switchtec_ntb *sndev = ntb_sndev(ntb);

	if (idx < sndev->nr_direct_mw)
		return switchtec_ntb_mw_direct_get_range(sndev, idx, base,
							 size, align,
							 align_size);
	else if (idx < switchtec_ntb_mw_count(ntb))
		return switchtec_ntb_mw_lut_get_range(sndev, idx, base,
						      size, align,
						      align_size);
	else
		return -EINVAL;
}

static void switchtec_ntb_mw_clr_direct(struct switchtec_ntb *sndev, int idx)
{
	struct ntb_ctrl_regs __iomem *ctl = sndev->mmio_peer_ctrl;
	int bar = sndev->direct_mw_to_bar[idx];
	u32 ctl_val;

	ctl_val = ioread32(&ctl->bar_entry[bar].ctl);
	ctl_val &= ~NTB_CTRL_BAR_DIR_WIN_EN;
	iowrite32(ctl_val, &ctl->bar_entry[bar].ctl);
	iowrite32(0, &ctl->bar_entry[bar].win_size);
	iowrite64(sndev->self_partition, &ctl->bar_entry[bar].xlate_addr);
}

static void switchtec_ntb_mw_clr_lut(struct switchtec_ntb *sndev, int idx)
{
	struct ntb_ctrl_regs __iomem *ctl = sndev->mmio_peer_ctrl;

	iowrite64(0, &ctl->lut_entry[lut_index(sndev, idx)]);
}

static void switchtec_ntb_mw_set_direct(struct switchtec_ntb *sndev, int idx,
					dma_addr_t addr, resource_size_t size)
{
	int xlate_pos = ilog2(size);
	int bar = sndev->direct_mw_to_bar[idx];
	struct ntb_ctrl_regs __iomem *ctl = sndev->mmio_peer_ctrl;
	u32 ctl_val;

	ctl_val = ioread32(&ctl->bar_entry[bar].ctl);
	ctl_val |= NTB_CTRL_BAR_DIR_WIN_EN;

	iowrite32(ctl_val, &ctl->bar_entry[bar].ctl);
	iowrite32(xlate_pos | size, &ctl->bar_entry[bar].win_size);
	iowrite64(sndev->self_partition | addr,
		  &ctl->bar_entry[bar].xlate_addr);
}

static void switchtec_ntb_mw_set_lut(struct switchtec_ntb *sndev, int idx,
				     dma_addr_t addr, resource_size_t size)
{
	struct ntb_ctrl_regs __iomem *ctl = sndev->mmio_peer_ctrl;

	iowrite64((NTB_CTRL_LUT_EN | (sndev->self_partition << 1) | addr),
		  &ctl->lut_entry[lut_index(sndev, idx)]);
}

static int switchtec_ntb_mw_set_trans(struct ntb_dev *ntb, int idx,
				      dma_addr_t addr, resource_size_t size)
{
	struct switchtec_ntb *sndev = ntb_sndev(ntb);
	struct ntb_ctrl_regs __iomem *ctl = sndev->mmio_peer_ctrl;
	int xlate_pos = ilog2(size);
	int rc;

	dev_dbg(&sndev->stdev->dev, "MW %d: %016llx %016llx", idx, addr, size);

	if (idx >= switchtec_ntb_mw_count(ntb))
		return -EINVAL;

	if (xlate_pos < 12)
		return -EINVAL;

	rc = switchtec_ntb_part_op(sndev, ctl, NTB_CTRL_PART_OP_LOCK,
				   NTB_CTRL_PART_STATUS_LOCKED);
	if (rc)
		return rc;

	if (addr == 0 || size == 0) {
		if (idx < sndev->nr_direct_mw)
			switchtec_ntb_mw_clr_direct(sndev, idx);
		else
			switchtec_ntb_mw_clr_lut(sndev, idx);
	} else {
		if (idx < sndev->nr_direct_mw)
			switchtec_ntb_mw_set_direct(sndev, idx, addr, size);
		else
			switchtec_ntb_mw_set_lut(sndev, idx, addr, size);
	}

	rc = switchtec_ntb_part_op(sndev, ctl, NTB_CTRL_PART_OP_CFG,
				   NTB_CTRL_PART_STATUS_NORMAL);

	if (rc == -EIO) {
		dev_err(&sndev->stdev->dev,
			"Hardware reported an error configuring mw %d: %08x",
			idx, ioread32(&ctl->bar_error));

		if (idx < sndev->nr_direct_mw)
			switchtec_ntb_mw_clr_direct(sndev, idx);
		else
			switchtec_ntb_mw_clr_lut(sndev, idx);

		switchtec_ntb_part_op(sndev, ctl, NTB_CTRL_PART_OP_CFG,
				      NTB_CTRL_PART_STATUS_NORMAL);
	}

	return rc;
}

static void switchtec_ntb_part_link_speed(struct switchtec_ntb *sndev,
					  int partition,
					  enum ntb_speed *speed,
					  enum ntb_width *width)
{
	struct switchtec_dev *stdev = sndev->stdev;

	u32 pff = ioread32(&stdev->mmio_part_cfg[partition].vep_pff_inst_id);
	u32 linksta = ioread32(&stdev->mmio_pff_csr[pff].pci_cap_region[13]);

	if (speed)
		*speed = (linksta >> 16) & 0xF;

	if (width)
		*width = (linksta >> 20) & 0x3F;
}

static void switchtec_ntb_set_link_speed(struct switchtec_ntb *sndev)
{
	enum ntb_speed self_speed, peer_speed;
	enum ntb_width self_width, peer_width;

	if (!sndev->link_is_up) {
		sndev->link_speed = NTB_SPEED_NONE;
		sndev->link_width = NTB_WIDTH_NONE;
		return;
	}

	switchtec_ntb_part_link_speed(sndev, sndev->self_partition,
				      &self_speed, &self_width);
	switchtec_ntb_part_link_speed(sndev, sndev->peer_partition,
				      &peer_speed, &peer_width);

	sndev->link_speed = min(self_speed, peer_speed);
	sndev->link_width = min(self_width, peer_width);
}

enum {
	LINK_MESSAGE = 0,
	MSG_LINK_UP = 1,
	MSG_LINK_DOWN = 2,
	MSG_CHECK_LINK = 3,
};

static void switchtec_ntb_check_link(struct switchtec_ntb *sndev)
{
	int link_sta;
	int old = sndev->link_is_up;

	link_sta = sndev->self_shared->link_sta;
	if (link_sta) {
		u64 peer = ioread64(&sndev->peer_shared->magic);

		if ((peer & 0xFFFFFFFF) == SWITCHTEC_NTB_MAGIC)
			link_sta = peer >> 32;
		else
			link_sta = 0;
	}

	sndev->link_is_up = link_sta;
	switchtec_ntb_set_link_speed(sndev);

	if (link_sta != old) {
		switchtec_ntb_send_msg(sndev, LINK_MESSAGE, MSG_CHECK_LINK);
		ntb_link_event(&sndev->ntb);
		dev_info(&sndev->stdev->dev, "ntb link %s",
			 link_sta ? "up" : "down");
	}
}

static void switchtec_ntb_link_event(struct switchtec_ntb *sndev, int msg)
{
	switchtec_ntb_check_link(sndev);
}

static int switchtec_ntb_link_notification(struct notifier_block *nb,
					   unsigned long event, void *data)
{
	struct switchtec_ntb *sndev = container_of(nb, struct switchtec_ntb,
						   link_notifier);

	dev_dbg(&sndev->stdev->dev, "link message received");
	switchtec_ntb_check_link(sndev);

	return NOTIFY_OK;
}

static int switchtec_ntb_init_link_notifier(struct switchtec_ntb *sndev)
{
	sndev->link_notifier.notifier_call = switchtec_ntb_link_notification;

	return blocking_notifier_chain_register(&sndev->stdev->link_notifier,
						&sndev->link_notifier);
}

static void switchtec_ntb_deinit_link_notifier(struct switchtec_ntb *sndev)
{
	blocking_notifier_chain_unregister(&sndev->stdev->link_notifier,
					   &sndev->link_notifier);
}

static int switchtec_ntb_link_is_up(struct ntb_dev *ntb,
				    enum ntb_speed *speed,
				    enum ntb_width *width)
{
	struct switchtec_ntb *sndev = ntb_sndev(ntb);

	if (speed)
		*speed = sndev->link_speed;
	if (width)
		*width = sndev->link_width;

	return sndev->link_is_up;
}

static int switchtec_ntb_link_enable(struct ntb_dev *ntb,
				     enum ntb_speed max_speed,
				     enum ntb_width max_width)
{
	struct switchtec_ntb *sndev = ntb_sndev(ntb);

	dev_dbg(&sndev->stdev->dev, "enabling link");

	sndev->self_shared->link_sta = 1;
	switchtec_ntb_send_msg(sndev, LINK_MESSAGE, MSG_LINK_UP);

	switchtec_ntb_check_link(sndev);

	return 0;
}

static int switchtec_ntb_link_disable(struct ntb_dev *ntb)
{
	struct switchtec_ntb *sndev = ntb_sndev(ntb);

	dev_dbg(&sndev->stdev->dev, "disabling link");

	sndev->self_shared->link_sta = 0;
	switchtec_ntb_send_msg(sndev, LINK_MESSAGE, MSG_LINK_UP);

	switchtec_ntb_check_link(sndev);

	return 0;
}

static u64 switchtec_ntb_db_valid_mask(struct ntb_dev *ntb)
{
	struct switchtec_ntb *sndev = ntb_sndev(ntb);

	return sndev->db_valid_mask;
}

static int switchtec_ntb_db_vector_count(struct ntb_dev *ntb)
{
	return 1;
}

static u64 switchtec_ntb_db_vector_mask(struct ntb_dev *ntb, int db_vector)
{
	struct switchtec_ntb *sndev = ntb_sndev(ntb);

	if (db_vector < 0 || db_vector > 1)
		return 0;

	return sndev->db_valid_mask;
}

static u64 switchtec_ntb_db_read(struct ntb_dev *ntb)
{
	u64 ret;
	struct switchtec_ntb *sndev = ntb_sndev(ntb);

	ret = ioread64(&sndev->mmio_self_dbmsg->idb) >> sndev->db_shift;

	return ret & sndev->db_valid_mask;
}

static int switchtec_ntb_db_clear(struct ntb_dev *ntb, u64 db_bits)
{
	struct switchtec_ntb *sndev = ntb_sndev(ntb);

	iowrite64(db_bits << sndev->db_shift, &sndev->mmio_self_dbmsg->idb);

	return 0;
}

static int switchtec_ntb_db_set_mask(struct ntb_dev *ntb, u64 db_bits)
{
	unsigned long irqflags;
	struct switchtec_ntb *sndev = ntb_sndev(ntb);

	if (db_bits & ~sndev->db_valid_mask)
		return -EINVAL;

	spin_lock_irqsave(&sndev->db_mask_lock, irqflags);
	{
		sndev->db_mask |= db_bits << sndev->db_shift;
		iowrite64(~sndev->db_mask, &sndev->mmio_self_dbmsg->idb_mask);
	}
	spin_unlock_irqrestore(&sndev->db_mask_lock, irqflags);

	return 0;
}

static int switchtec_ntb_db_clear_mask(struct ntb_dev *ntb, u64 db_bits)
{
	unsigned long irqflags;
	struct switchtec_ntb *sndev = ntb_sndev(ntb);

	if (db_bits & ~sndev->db_valid_mask)
		return -EINVAL;

	spin_lock_irqsave(&sndev->db_mask_lock, irqflags);
	{
		sndev->db_mask &= ~(db_bits << sndev->db_shift);
		iowrite64(~sndev->db_mask, &sndev->mmio_self_dbmsg->idb_mask);
	}
	spin_unlock_irqrestore(&sndev->db_mask_lock, irqflags);

	return 0;
}

static u64 switchtec_ntb_db_read_mask(struct ntb_dev *ntb)
{
	struct switchtec_ntb *sndev = ntb_sndev(ntb);

	return (sndev->db_mask >> sndev->db_shift) & sndev->db_valid_mask;
}

static int switchtec_ntb_peer_db_addr(struct ntb_dev *ntb,
				      phys_addr_t *db_addr,
				      resource_size_t *db_size)
{
	struct switchtec_ntb *sndev = ntb_sndev(ntb);
	unsigned long offset;

	offset = (unsigned long)sndev->mmio_self_dbmsg->odb -
		(unsigned long)sndev->stdev->mmio;

	offset += sndev->db_shift / 8;

	if (db_addr)
		*db_addr = pci_resource_start(ntb->pdev, 0) + offset;
	if (db_size)
		*db_size = sizeof(u32);

	return 0;
}

static int switchtec_ntb_peer_db_set(struct ntb_dev *ntb, u64 db_bits)
{
	struct switchtec_ntb *sndev = ntb_sndev(ntb);

	iowrite64(db_bits << sndev->db_peer_shift,
		  &sndev->mmio_self_dbmsg->odb);

	return 0;
}

static int switchtec_ntb_spad_count(struct ntb_dev *ntb)
{
	struct switchtec_ntb *sndev = ntb_sndev(ntb);

	return ARRAY_SIZE(sndev->self_shared->spad);
}

static u32 switchtec_ntb_spad_read(struct ntb_dev *ntb, int idx)
{
	struct switchtec_ntb *sndev = ntb_sndev(ntb);

	if (idx < 0 || idx >= ARRAY_SIZE(sndev->self_shared->spad))
		return 0;

	if (!sndev->self_shared)
		return 0;

	return sndev->self_shared->spad[idx];
}

static int switchtec_ntb_spad_write(struct ntb_dev *ntb, int idx, u32 val)
{
	struct switchtec_ntb *sndev = ntb_sndev(ntb);

	if (idx < 0 || idx >= ARRAY_SIZE(sndev->self_shared->spad))
		return -EINVAL;

	if (!sndev->self_shared)
		return -EIO;

	sndev->self_shared->spad[idx] = val;

	return 0;
}

static u32 switchtec_ntb_peer_spad_read(struct ntb_dev *ntb, int idx)
{
	struct switchtec_ntb *sndev = ntb_sndev(ntb);

	if (idx < 0 || idx >= ARRAY_SIZE(sndev->peer_shared->spad))
		return 0;

	if (!sndev->peer_shared)
		return 0;

	return ioread32(&sndev->peer_shared->spad[idx]);
}

static int switchtec_ntb_peer_spad_write(struct ntb_dev *ntb, int idx, u32 val)
{
	struct switchtec_ntb *sndev = ntb_sndev(ntb);

	if (idx < 0 || idx >= ARRAY_SIZE(sndev->peer_shared->spad))
		return -EINVAL;

	if (!sndev->peer_shared)
		return -EIO;

	iowrite32(val, &sndev->peer_shared->spad[idx]);

	return 0;
}

static int switchtec_ntb_peer_spad_addr(struct ntb_dev *ntb, int idx,
					phys_addr_t *spad_addr)
{
	struct switchtec_ntb *sndev = ntb_sndev(ntb);
	unsigned long offset;

	offset = (unsigned long)&sndev->peer_shared->spad[idx] -
		(unsigned long)sndev->stdev->mmio;

	if (spad_addr)
		*spad_addr = pci_resource_start(ntb->pdev, 0) + offset;

	return 0;
}

static const struct ntb_dev_ops switchtec_ntb_ops = {
	.mw_count		= switchtec_ntb_mw_count,
	.mw_get_range		= switchtec_ntb_mw_get_range,
	.mw_set_trans		= switchtec_ntb_mw_set_trans,
	.link_is_up		= switchtec_ntb_link_is_up,
	.link_enable		= switchtec_ntb_link_enable,
	.link_disable		= switchtec_ntb_link_disable,
	.db_valid_mask		= switchtec_ntb_db_valid_mask,
	.db_vector_count	= switchtec_ntb_db_vector_count,
	.db_vector_mask		= switchtec_ntb_db_vector_mask,
	.db_read		= switchtec_ntb_db_read,
	.db_clear		= switchtec_ntb_db_clear,
	.db_set_mask		= switchtec_ntb_db_set_mask,
	.db_clear_mask		= switchtec_ntb_db_clear_mask,
	.db_read_mask		= switchtec_ntb_db_read_mask,
	.peer_db_addr		= switchtec_ntb_peer_db_addr,
	.peer_db_set		= switchtec_ntb_peer_db_set,
	.spad_count		= switchtec_ntb_spad_count,
	.spad_read		= switchtec_ntb_spad_read,
	.spad_write		= switchtec_ntb_spad_write,
	.peer_spad_read		= switchtec_ntb_peer_spad_read,
	.peer_spad_write	= switchtec_ntb_peer_spad_write,
	.peer_spad_addr		= switchtec_ntb_peer_spad_addr,
};

static void switchtec_ntb_init_sndev(struct switchtec_ntb *sndev)
{
	u64 part_map;

	sndev->ntb.pdev = sndev->stdev->pdev;
	sndev->ntb.topo = NTB_TOPO_NONE;
	sndev->ntb.ops = &switchtec_ntb_ops;

	sndev->self_partition = sndev->stdev->partition;

	sndev->mmio_ntb = sndev->stdev->mmio_ntb;
	part_map = ioread64(&sndev->mmio_ntb->ep_map);
	part_map &= ~(1 << sndev->self_partition);
	sndev->peer_partition = ffs(part_map) - 1;

	dev_dbg(&sndev->stdev->dev, "Partition ID %d of %d (%llx)",
		sndev->self_partition, sndev->stdev->partition_count,
		part_map);

	sndev->mmio_ctrl = (void * __iomem)sndev->mmio_ntb +
		SWITCHTEC_NTB_REG_CTRL_OFFSET;
	sndev->mmio_dbmsg = (void * __iomem)sndev->mmio_ntb +
		SWITCHTEC_NTB_REG_DBMSG_OFFSET;

	sndev->mmio_self_ctrl = &sndev->mmio_ctrl[sndev->self_partition];
	sndev->mmio_peer_ctrl = &sndev->mmio_ctrl[sndev->peer_partition];
	sndev->mmio_self_dbmsg = &sndev->mmio_dbmsg[sndev->self_partition];
}

static void switchtec_ntb_init_mw(struct switchtec_ntb *sndev)
{
	int i;

	sndev->nr_direct_mw = 0;
	for (i = 0; i < ARRAY_SIZE(sndev->mmio_self_ctrl->bar_entry); i++) {
		u32 r = ioread32(&sndev->mmio_self_ctrl->bar_entry[i].ctl);

		if (r & NTB_CTRL_BAR_VALID)
			sndev->direct_mw_to_bar[sndev->nr_direct_mw++] = i;
	}

	sndev->nr_lut_mw = ioread16(&sndev->mmio_self_ctrl->lut_table_entries);
	sndev->nr_lut_mw = rounddown_pow_of_two(sndev->nr_lut_mw);

	dev_dbg(&sndev->stdev->dev, "MWs: %d direct, %d lut",
		sndev->nr_direct_mw, sndev->nr_lut_mw);
}

/*
 * There are 64 doorbells in the switch hardware but this is
 * shared among all partitions. So we must split them in half
 * (32 for each partition). However, the message interrupts are
 * also shared with the top 4 doorbells so we just limit this to
 * 28 doorbells per partition
 */
static void switchtec_ntb_init_db(struct switchtec_ntb *sndev)
{
	sndev->db_valid_mask = 0x0FFFFFFF;

	if (sndev->self_partition < sndev->peer_partition) {
		sndev->db_shift = 0;
		sndev->db_peer_shift = 32;
	} else {
		sndev->db_shift = 32;
		sndev->db_peer_shift = 0;
	}

	sndev->db_mask = 0x0FFFFFFFFFFFFFFFULL;
	iowrite64(~sndev->db_mask, &sndev->mmio_self_dbmsg->idb_mask);
	iowrite64(sndev->db_valid_mask << sndev->db_peer_shift,
		  &sndev->mmio_self_dbmsg->odb_mask);
}

static void switchtec_ntb_init_msgs(struct switchtec_ntb *sndev)
{
	int i;
	u32 msg_map = 0;

	for (i = 0; i < ARRAY_SIZE(sndev->mmio_self_dbmsg->imsg); i++) {
		int m = i | sndev->peer_partition << 2;

		msg_map |= m << i * 8;
	}

	iowrite32(msg_map, &sndev->mmio_self_dbmsg->msg_map);

	for (i = 0; i < ARRAY_SIZE(sndev->mmio_self_dbmsg->imsg); i++)
		iowrite64(NTB_DBMSG_IMSG_STATUS | NTB_DBMSG_IMSG_MASK,
			  &sndev->mmio_self_dbmsg->imsg[i]);
}

static int switchtec_ntb_init_req_id_table(struct switchtec_ntb *sndev)
{
	int rc = 0;
	u16 req_id = ioread16(&sndev->mmio_ntb->requester_id);
	u32 error;

	if (ioread32(&sndev->mmio_self_ctrl->req_id_table_size) < 2) {
		dev_err(&sndev->stdev->dev,
			"Not enough requester IDs available.");
		return -EFAULT;
	}

	rc = switchtec_ntb_part_op(sndev, sndev->mmio_self_ctrl,
				   NTB_CTRL_PART_OP_LOCK,
				   NTB_CTRL_PART_STATUS_LOCKED);
	if (rc)
		return rc;

	iowrite32(NTB_PART_CTRL_ID_PROT_DIS,
		  &sndev->mmio_self_ctrl->partition_ctrl);

	// Root Complex Requester ID
	iowrite32(0 << 16 | NTB_CTRL_REQ_ID_EN,
		  &sndev->mmio_self_ctrl->req_id_table[0]);

	// Host Bridge Requester ID
	iowrite32(req_id << 16 | NTB_CTRL_REQ_ID_EN,
		  &sndev->mmio_self_ctrl->req_id_table[1]);

	rc = switchtec_ntb_part_op(sndev, sndev->mmio_self_ctrl,
				   NTB_CTRL_PART_OP_CFG,
				   NTB_CTRL_PART_STATUS_NORMAL);

	if (rc == -EIO) {
		error = ioread32(&sndev->mmio_self_ctrl->req_id_error);
		dev_err(&sndev->stdev->dev,
			"Error setting up the requester ID table: %08x",
			error);
	}

	return rc;
}

static int switchtec_ntb_init_shared_mw(struct switchtec_ntb *sndev)
{
	struct ntb_ctrl_regs __iomem *ctl = sndev->mmio_peer_ctrl;
	int bar = sndev->direct_mw_to_bar[0];
	u32 ctl_val;
	int rc;

	sndev->self_shared = dma_zalloc_coherent(&sndev->stdev->pdev->dev,
						 LUT_SIZE,
						 &sndev->self_shared_dma,
						 GFP_KERNEL);
	if (!sndev->self_shared) {
		dev_err(&sndev->stdev->dev,
			"unable to allocate memory for shared mw");
		return -ENOMEM;
	}

	memset(sndev->self_shared, 0, LUT_SIZE);
	sndev->self_shared->magic = SWITCHTEC_NTB_MAGIC;
	sndev->self_shared->partition_id = sndev->stdev->partition;

	rc = switchtec_ntb_part_op(sndev, ctl, NTB_CTRL_PART_OP_LOCK,
				   NTB_CTRL_PART_STATUS_LOCKED);
	if (rc)
		goto unalloc_and_exit;

	ctl_val = ioread32(&ctl->bar_entry[bar].ctl);
	ctl_val |= NTB_CTRL_BAR_LUT_WIN_EN;
	ctl_val &= 0xFF;
	ctl_val |= ilog2(LUT_SIZE) << 8;
	ctl_val |= (sndev->nr_lut_mw - 1) << 14;
	iowrite32(ctl_val, &ctl->bar_entry[bar].ctl);

	iowrite64((NTB_CTRL_LUT_EN | (sndev->self_partition << 1) |
		   sndev->self_shared_dma),
		  &ctl->lut_entry[0]);

	rc = switchtec_ntb_part_op(sndev, ctl, NTB_CTRL_PART_OP_CFG,
				   NTB_CTRL_PART_STATUS_NORMAL);
	if (rc) {
		u32 bar_error, lut_error;

		bar_error = ioread32(&ctl->bar_error);
		lut_error = ioread32(&ctl->lut_error);
		dev_err(&sndev->stdev->dev,
			"Error setting up shared MW: %08x / %08x",
			bar_error, lut_error);
		goto unalloc_and_exit;
	}

	sndev->peer_shared = pci_iomap(sndev->stdev->pdev, bar, LUT_SIZE);
	if (!sndev->peer_shared) {
		rc = -ENOMEM;
		goto unalloc_and_exit;
	}

	dev_dbg(&sndev->stdev->dev, "Shared MW Ready");
	return 0;

unalloc_and_exit:
	dma_free_coherent(&sndev->stdev->pdev->dev, LUT_SIZE,
			  sndev->self_shared, sndev->self_shared_dma);

	return rc;
}

static void switchtec_ntb_deinit_shared_mw(struct switchtec_ntb *sndev)
{
	if (sndev->peer_shared)
		pci_iounmap(sndev->stdev->pdev, sndev->peer_shared);

	if (sndev->self_shared)
		dma_free_coherent(&sndev->stdev->pdev->dev, LUT_SIZE,
				  sndev->self_shared,
				  sndev->self_shared_dma);
}

static irqreturn_t switchtec_ntb_doorbell_isr(int irq, void *dev)
{
	struct switchtec_ntb *sndev = dev;

	dev_dbg(&sndev->stdev->dev, "doorbell\n");

	ntb_db_event(&sndev->ntb, 0);

	return IRQ_HANDLED;
}

static irqreturn_t switchtec_ntb_message_isr(int irq, void *dev)
{
	int i;
	struct switchtec_ntb *sndev = dev;

	for (i = 0; i < ARRAY_SIZE(sndev->mmio_self_dbmsg->imsg); i++) {
		u64 msg = ioread64(&sndev->mmio_self_dbmsg->imsg[i]);

		if (msg & NTB_DBMSG_IMSG_STATUS) {
			dev_dbg(&sndev->stdev->dev, "message: %d %08x\n", i,
				(u32)msg);
			iowrite8(1, &sndev->mmio_self_dbmsg->imsg[i].status);

			if (i == LINK_MESSAGE)
				switchtec_ntb_link_event(sndev, msg);
		}
	}

	return IRQ_HANDLED;
}

static int switchtec_ntb_init_db_msg_irq(struct switchtec_ntb *sndev)
{
	int i;
	int rc;
	int doorbell_irq = 0;
	int message_irq = 0;
	int event_irq;
	int idb_vecs = sizeof(sndev->mmio_self_dbmsg->idb_vec_map);

	event_irq = ioread32(&sndev->stdev->mmio_part_cfg->vep_vector_number);

	while (doorbell_irq == event_irq)
		doorbell_irq++;
	while (message_irq == doorbell_irq ||
	       message_irq == event_irq)
		message_irq++;

	dev_dbg(&sndev->stdev->dev, "irqs - event: %d, db: %d, msgs: %d",
		event_irq, doorbell_irq, message_irq);

	for (i = 0; i < idb_vecs - 4; i++)
		iowrite8(doorbell_irq,
			 &sndev->mmio_self_dbmsg->idb_vec_map[i]);

	for (; i < idb_vecs; i++)
		iowrite8(message_irq,
			 &sndev->mmio_self_dbmsg->idb_vec_map[i]);

	sndev->doorbell_irq = pci_irq_vector(sndev->stdev->pdev, doorbell_irq);
	sndev->message_irq = pci_irq_vector(sndev->stdev->pdev, message_irq);

	rc = request_irq(sndev->doorbell_irq,
			 switchtec_ntb_doorbell_isr, 0,
			 "switchtec_ntb_doorbell", sndev);
	if (rc)
		return rc;

	rc = request_irq(sndev->message_irq,
			 switchtec_ntb_message_isr, 0,
			 "switchtec_ntb_message", sndev);
	if (rc) {
		free_irq(sndev->doorbell_irq, sndev);
		return rc;
	}

	return 0;
}

static void switchtec_ntb_deinit_db_msg_irq(struct switchtec_ntb *sndev)
{
	free_irq(sndev->doorbell_irq, sndev);
	free_irq(sndev->message_irq, sndev);
}

static int switchtec_ntb_add(struct device *dev,
			     struct class_interface *class_intf)
{
	struct switchtec_dev *stdev = to_stdev(dev);
	struct switchtec_ntb *sndev;
	int rc;

	stdev->sndev = NULL;

	if (stdev->pdev->class != MICROSEMI_NTB_CLASSCODE)
		return -ENODEV;

	if (stdev->partition_count != 2)
		dev_warn(dev, "ntb driver only supports 2 partitions");

	sndev = kzalloc_node(sizeof(*sndev), GFP_KERNEL, dev_to_node(dev));
	if (!sndev)
		return -ENOMEM;

	sndev->stdev = stdev;
	switchtec_ntb_init_sndev(sndev);
	switchtec_ntb_init_mw(sndev);
	switchtec_ntb_init_db(sndev);
	switchtec_ntb_init_msgs(sndev);

	rc = switchtec_ntb_init_req_id_table(sndev);
	if (rc)
		goto free_and_exit;

	rc = switchtec_ntb_init_shared_mw(sndev);
	if (rc)
		goto free_and_exit;

	rc = switchtec_ntb_init_db_msg_irq(sndev);
	if (rc)
		goto deinit_shared_and_exit;

	rc = switchtec_ntb_init_link_notifier(sndev);
	if (rc)
		goto denit_db_msg_and_exit;

	rc = ntb_register_device(&sndev->ntb);
	if (rc)
		goto deinit_and_exit;

	stdev->sndev = sndev;
	dev_info(dev, "NTB device registered");

	return 0;

deinit_and_exit:
	switchtec_ntb_deinit_link_notifier(sndev);
denit_db_msg_and_exit:
	switchtec_ntb_deinit_db_msg_irq(sndev);
deinit_shared_and_exit:
	switchtec_ntb_deinit_shared_mw(sndev);
free_and_exit:
	kfree(sndev);
	dev_err(dev, "failed to register ntb device: %d", rc);
	return rc;
}

void switchtec_ntb_remove(struct device *dev,
			  struct class_interface *class_intf)
{
	struct switchtec_dev *stdev = to_stdev(dev);
	struct switchtec_ntb *sndev = stdev->sndev;

	if (!sndev)
		return;

	stdev->sndev = NULL;
	ntb_unregister_device(&sndev->ntb);
	switchtec_ntb_deinit_link_notifier(sndev);
	switchtec_ntb_deinit_db_msg_irq(sndev);
	switchtec_ntb_deinit_shared_mw(sndev);
	kfree(sndev);
	dev_info(dev, "ntb device unregistered");
}

static struct class_interface switchtec_interface  = {
	.add_dev = switchtec_ntb_add,
	.remove_dev = switchtec_ntb_remove,
};

static int __init switchtec_ntb_init(void)
{
	switchtec_interface.class = switchtec_class;
	return class_interface_register(&switchtec_interface);
}
module_init(switchtec_ntb_init);

static void __exit switchtec_ntb_exit(void)
{
	class_interface_unregister(&switchtec_interface);
}
module_exit(switchtec_ntb_exit);
