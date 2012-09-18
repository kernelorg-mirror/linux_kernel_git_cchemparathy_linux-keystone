/*
 * Keystone hardware queue driver
 *
 * Copyright (C) 2011 Texas Instruments Incorporated - http://www.ti.com
 * Contact: Prabhu Kuttiyam <pkuttiyam@ti.com>
 *	    Cyril Chemparathy <cyril@ti.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/io.h>
#include <linux/bitops.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/hwqueue.h>
#include <linux/platform_device.h>
#include <linux/dma-mapping.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_address.h>
#include <linux/firmware.h>
#include <linux/interrupt.h>

#include "hwqueue_internal.h"

#define DESC_SIZE_MASK	0xful
#define DESC_PTR_MASK	(~DESC_SIZE_MASK)

#define THRESH_GTE	BIT(7)
#define THRESH_LT	0

#define PDSP_CTRL_PC_MASK	0xffff0000
#define PDSP_CTRL_SOFT_RESET	BIT(0)
#define PDSP_CTRL_ENABLE	BIT(1)
#define PDSP_CTRL_RUNNING	BIT(15)

#define ACCUM_MAX_CHANNEL		48
#define ACCUM_DEFAULT_PERIOD		25 /* usecs */
#define ACCUM_DESCS_MAX			SZ_1K
#define ACCUM_DESCS_MASK		(ACCUM_DESCS_MAX - 1)
#define ACCUM_CHANNEL_INT_BASE		2

#define ACCUM_LIST_ENTRY_TYPE		1
#define ACCUM_LIST_ENTRY_WORDS		(1 << ACCUM_LIST_ENTRY_TYPE)
#define ACCUM_LIST_ENTRY_QUEUE_IDX	0
#define ACCUM_LIST_ENTRY_DESC_IDX	(ACCUM_LIST_ENTRY_WORDS - 1)

#define ACCUM_CMD_DISABLE_CHANNEL	0x80
#define ACCUM_CMD_ENABLE_CHANNEL	0x81
#define ACCUM_CFG_MULTI_QUEUE		BIT(21)

#define ACCUM_INTD_OFFSET_EOI		(0x0010)
#define ACCUM_INTD_OFFSET_COUNT(ch)	(0x0300 + 4 * (ch))
#define ACCUM_INTD_OFFSET_STATUS(ch)	(0x0200 + 4 * ((ch) / 32))

struct khwq_reg_config {
	u32		revision;
	u32		__pad1;
	u32		divert;
	u32		link_ram_base0;
	u32		link_ram_size0;
	u32		link_ram_base1;
	u32		__pad2[2];
	u32		starvation[0];
};

struct khwq_reg_region {
	u32		base;
	u32		start_index;
	u32		size_count;
	u32		__pad;
};

struct khwq_reg_queue {
	u32		entry_count;
	u32		byte_count;
	u32		packet_size;
	u32		ptr_size_thresh;
};

struct khwq_reg_pdsp_regs {
	u32		control;
	u32		status;
	u32		cycle_count;
	u32		stall_count;
};

struct khwq_reg_pdsp_command {
	u32		command;
	u32		queue_mask;
	u32		list_phys;
	u32		queue_num;
	u32		timer_config;
};

enum khwq_acc_result {
	ACCUM_RET_IDLE,
	ACCUM_RET_SUCCESS,
	ACCUM_RET_INVALID_COMMAND,
	ACCUM_RET_INVALID_CHANNEL,
	ACCUM_RET_INACTIVE_CHANNEL,
	ACCUM_RET_ACTIVE_CHANNEL,
	ACCUM_RET_INVALID_QUEUE,

	ACCUM_RET_INVALID_RET,
};

struct khwq_region {
	unsigned	 desc_size;
	unsigned	 num_desc;
	unsigned	 fixed_mem;
	dma_addr_t	 next_pool_addr;
	dma_addr_t	 dma_start, dma_end;
	void		*virt_start, *virt_end;
	unsigned	 link_index;
};

struct khwq_pool_info {
	const char		*name;
	struct khwq_region	*region;
	int			 region_offset;
	int			 num_desc;
	int			 desc_size;
	dma_addr_t		 start_addr;
	struct hwqueue		*queue;
	struct list_head	 list;
};

struct khwq_link_ram_block {
	dma_addr_t	 phys;
	void		*virt;
	size_t		 size;
};

struct khwq_pdsp_info {
	const char				*name;
	struct khwq_reg_pdsp_regs  __iomem	*regs;
	struct khwq_reg_pdsp_command  __iomem	*command;
	void __iomem				*intd;
	u32 __iomem				*iram;
	const char				*firmware;
	u32					 id;
	struct list_head			 list;
};

struct khwq_acc_info {
	u32			 pdsp_id;
	u32			 start_channel;
	u32			 list_entries;
	u32			 pacing_mode;
	u32			 timer_count;
	int			 mem_size;
	int			 list_size;
	struct khwq_pdsp_info	*pdsp;
};

struct khwq_acc_channel {
	u32			 channel;
	u32			 list_index;
	u32			 open_mask;
	u32			*list_cpu[2];
	dma_addr_t		 list_dma[2];
	char			 name[32];
	atomic_t		 retrigger_count;
};

struct khwq_range_info {
	const char		*name;
	struct khwq_device	*kdev;
	unsigned		 queue_base;
	unsigned		 num_queues;
	unsigned		 irq_base;
	unsigned		 flags;
	struct list_head	 list;
	struct khwq_acc_info	 acc_info;
	struct khwq_acc_channel	*acc;
};

#define RANGE_RESERVED		BIT(0)
#define RANGE_HAS_IRQ		BIT(1)
#define RANGE_HAS_ACCUMULATOR	BIT(2)
#define RANGE_MULTI_QUEUE	BIT(3)

struct khwq_qmgr_info {
	unsigned			 start_queue;
	unsigned			 num_queues;
	struct khwq_reg_config __iomem	*reg_config;
	struct khwq_reg_region __iomem	*reg_region;
	struct khwq_reg_queue __iomem	*reg_push, *reg_pop, *reg_peek;
	void __iomem			*reg_status;
	struct list_head		 list;
};

struct khwq_device {
	struct device			*dev;
	struct hwqueue_device		 hdev;
	struct khwq_region		*regions;
	resource_size_t			 start_region, num_regions;
	resource_size_t			 start_index, num_index;
	struct khwq_link_ram_block	 link_rams[2];
	unsigned			 num_queues;
	unsigned			 base_id;
	struct list_head		 queue_ranges;
	struct list_head		 pools;
	struct list_head		 pdsps;
	struct list_head		 qmgrs;
};

struct khwq_desc {
	u32			 val;
	unsigned		 size;
	struct list_head	 list;
};

struct khwq_instance {
	u32			*descs;
	atomic_t		 desc_head, desc_tail, desc_count;
	struct khwq_device	*kdev;
	struct khwq_range_info	*range;
	struct khwq_acc_channel	*acc;
	struct khwq_region	*last; /* cache last region used */
	struct khwq_qmgr_info	*qmgr; /* cache qmgr for the instance */
	int			 irq_num; /*irq num -ve for non-irq queues */
	char			 irq_name[32];
};

#define to_hdev(_kdev)		(&(_kdev)->hdev)
#define from_hdev(_hdev)	container_of(_hdev, struct khwq_device, hdev)

#define region_index(kdev, r)	((r) - (kdev)->regions + (kdev)->start_region)

#define for_each_region(kdev, r)				\
	for ((r) = (kdev)->regions;				\
	     (r) < (kdev)->regions + (kdev)->num_regions;	\
	     (r)++)

#define for_each_queue_range(kdev, range)			\
	list_for_each_entry(range, &kdev->queue_ranges, list)

#define first_queue_range(kdev)					\
	list_first_entry(&kdev->queue_ranges, struct khwq_range_info, list)

#define for_each_pool(kdev, pool)				\
	list_for_each_entry(pool, &kdev->pools, list)

#define for_each_pdsp(kdev, pdsp)				\
	list_for_each_entry(pdsp, &kdev->pdsps, list)

#define for_each_qmgr(kdev, qmgr)				\
	list_for_each_entry(qmgr, &kdev->qmgrs, list)

static inline int khwq_pdsp_wait(u32 * __iomem addr, unsigned timeout,
				 u32 flags)
{
	unsigned long end_time;
	u32 val = 0;
	int ret;

	end_time = jiffies + msecs_to_jiffies(timeout);
	while (jiffies < end_time) {
		val = __raw_readl(addr);
		if (flags)
			val &= flags;
		if (!val)
			break;
		cpu_relax();
	}
	ret = val ? -ETIMEDOUT : 0;

	return ret;
}

static inline struct khwq_pdsp_info *
khwq_find_pdsp(struct khwq_device *kdev, unsigned pdsp_id)
{
	struct khwq_pdsp_info *pdsp;

	for_each_pdsp(kdev, pdsp)
		if (pdsp_id == pdsp->id)
			return pdsp;
	return NULL;
}

static inline struct khwq_qmgr_info *
khwq_find_qmgr(struct hwqueue_instance *inst)
{
	struct khwq_device *kdev = from_hdev(inst->hdev);
	unsigned id = hwqueue_inst_to_id(inst);
	struct khwq_qmgr_info *qmgr;

	for_each_qmgr(kdev, qmgr) {
		if ((id >= qmgr->start_queue) &&
			(id < qmgr->start_queue + qmgr->num_queues))
			return qmgr;
	}

	return NULL;
}

static int khwq_match(struct hwqueue_instance *inst, unsigned flags)
{
	struct khwq_instance *kq = hwqueue_inst_to_priv(inst);
	struct khwq_range_info *range;
	int score = 0;

	if (!kq)
		return -ENOENT;

	range = kq->range;
	if (!range)
		return -ENOENT;

	if (range->flags & RANGE_RESERVED)
		score += 1000;

	if ((range->flags & RANGE_HAS_ACCUMULATOR) &&
	    !(flags & O_HIGHTHROUGHPUT))
		score += 100;
	if (!(range->flags & RANGE_HAS_ACCUMULATOR) &&
	    (flags & O_HIGHTHROUGHPUT))
		score += 100;

	if ((range->flags & RANGE_HAS_IRQ) &&
	    !(flags & (O_LOWLATENCY | O_HIGHTHROUGHPUT)))
		score += 100;
	if (!(range->flags & RANGE_HAS_IRQ) &&
	    (flags & (O_LOWLATENCY | O_HIGHTHROUGHPUT)))
		score += 100;

	return score;
}

static irqreturn_t khwq_int_handler(int irq, void *_instdata)
{
	struct hwqueue_instance *inst = _instdata;

	hwqueue_notify(inst);

	return IRQ_HANDLED;
}

static void __khwq_acc_notify(struct khwq_range_info *range,
			      struct khwq_acc_channel *acc)
{
	struct khwq_device *kdev = range->kdev;
	struct hwqueue_device *hdev = to_hdev(kdev);
	struct hwqueue_instance *inst;
	int range_base, queue;

	range_base = kdev->base_id + range->queue_base;

	if (range->flags & RANGE_MULTI_QUEUE) {
		for (queue = 0; queue < range->num_queues; queue++) {
			inst = hwqueue_id_to_inst(hdev, range_base + queue);
			dev_dbg(kdev->dev, "acc-irq: notifying %d\n",
				range_base + queue);
			hwqueue_notify(inst);
		}
	} else {
		queue = acc->channel - range->acc_info.start_channel;
		inst = hwqueue_id_to_inst(hdev, range_base + queue);
		dev_dbg(kdev->dev, "acc-irq: notifying %d\n",
			range_base + queue);
		hwqueue_notify(inst);
	}
}

static irqreturn_t khwq_acc_int_handler(int irq, void *_instdata)
{
	struct hwqueue_instance *inst = NULL;
	struct khwq_acc_channel *acc;
	struct khwq_instance *kq = NULL;
	struct khwq_range_info *range;
	struct hwqueue_device *hdev;
	struct khwq_pdsp_info *pdsp;
	struct khwq_acc_info *info;
	struct khwq_device *kdev;

	u32 *list, *list_cpu, val, idx, notifies;
	int range_base, channel, queue = 0;
	dma_addr_t list_dma;

	range = _instdata;
	info  = &range->acc_info;
	kdev  = range->kdev;
	hdev  = to_hdev(kdev);
	pdsp  = range->acc_info.pdsp;
	acc   = range->acc;

	range_base = kdev->base_id + range->queue_base;

	if ((range->flags & RANGE_MULTI_QUEUE) == 0) {
		queue	 = irq - range->irq_base;
		inst	 = hwqueue_id_to_inst(hdev, range_base + queue);
		kq	 = hwqueue_inst_to_priv(inst);
		acc	+= queue;
	}

	channel = acc->channel;
	list_dma = acc->list_dma[acc->list_index];
	list_cpu = acc->list_cpu[acc->list_index];

	dev_dbg(kdev->dev, "acc-irq: channel %d, list %d, virt %p, phys %x\n",
		channel, acc->list_index, list_cpu, list_dma);

	if (atomic_read(&acc->retrigger_count)) {
		atomic_dec(&acc->retrigger_count);
		__khwq_acc_notify(range, acc);

		__raw_writel(1, pdsp->intd + ACCUM_INTD_OFFSET_COUNT(channel));

		/* ack the interrupt */
		__raw_writel(ACCUM_CHANNEL_INT_BASE + channel,
			     pdsp->intd + ACCUM_INTD_OFFSET_EOI);

		return IRQ_HANDLED;
	}

	notifies = __raw_readl(pdsp->intd + ACCUM_INTD_OFFSET_COUNT(channel));
	WARN_ON(!notifies);

	dma_sync_single_for_cpu(kdev->dev, list_dma, info->list_size, DMA_FROM_DEVICE);

	for (list = list_cpu; list < list_cpu + (info->list_size / sizeof(u32));
	     list += ACCUM_LIST_ENTRY_WORDS) {

		if (ACCUM_LIST_ENTRY_WORDS == 1) {
			dev_dbg(kdev->dev, "acc-irq: list %d, entry @%p, "
				"%08x\n",
				acc->list_index, list, list[0]);
		} else if (ACCUM_LIST_ENTRY_WORDS == 2) {
			dev_dbg(kdev->dev, "acc-irq: list %d, entry @%p, "
				"%08x %08x\n",
				acc->list_index, list, list[0], list[1]);
		} else if (ACCUM_LIST_ENTRY_WORDS == 4) {
			dev_dbg(kdev->dev, "acc-irq: list %d, entry @%p, "
				"%08x %08x %08x %08x\n",
				acc->list_index, list,
				list[0], list[1], list[2], list[3]);
		}

		val = list[ACCUM_LIST_ENTRY_DESC_IDX];

		if (!val)
			break;

		if (range->flags & RANGE_MULTI_QUEUE) {
			queue = list[ACCUM_LIST_ENTRY_QUEUE_IDX] >> 16;
			if (queue < range_base || queue >= range_base + range->num_queues) {
				dev_err(kdev->dev, "bad queue %d, expecting %d-%d\n",
					queue, range_base, range_base + range->num_queues);
				break;
			}
			queue -= range_base;
			inst = hwqueue_id_to_inst(hdev, range_base + queue);
			kq = hwqueue_inst_to_priv(inst);
		}

		if (atomic_inc_return(&kq->desc_count) >= ACCUM_DESCS_MAX) {
			atomic_dec(&kq->desc_count);
			/* TODO: need a statistics counter for such drops */
			continue;
		}

		idx = atomic_inc_return(&kq->desc_tail) & ACCUM_DESCS_MASK;
		kq->descs[idx] = val;
		dev_dbg(kdev->dev, "acc-irq: enqueue %08x at %d, queue %d\n",
			val, idx, queue + range_base);
	}

	__khwq_acc_notify(range, acc);

	memset(list_cpu, 0, info->list_size);
	dma_sync_single_for_device(kdev->dev, list_dma, info->list_size,
				   DMA_TO_DEVICE);

	/* flip to the other list */
	acc->list_index ^= 1;

	/* reset the interrupt counter */
	__raw_writel(1, pdsp->intd + ACCUM_INTD_OFFSET_COUNT(channel));

	/* ack the interrupt */
	__raw_writel(ACCUM_CHANNEL_INT_BASE + channel,
		     pdsp->intd + ACCUM_INTD_OFFSET_EOI);

	return IRQ_HANDLED;
}

static void khwq_set_notify(struct hwqueue_instance *inst, bool enabled)
{
	struct khwq_range_info *range;
	struct khwq_instance *kq;
	struct khwq_device *kdev;
	u32 mask, offset;
	unsigned queue;

	kq    = hwqueue_inst_to_priv(inst);
	range = kq->range;
	kdev  = range->kdev;
	queue = hwqueue_inst_to_id(inst) - range->queue_base;

	if (range->flags & RANGE_HAS_ACCUMULATOR) {
		struct khwq_pdsp_info *pdsp = range->acc_info.pdsp;

		/*
		 * when enabling, we need to re-trigger an interrupt if we
		 * have descriptors pending
		 */
		if (!enabled || atomic_read(&kq->desc_count) <= 0)
			return;

		atomic_inc(&kq->acc->retrigger_count);
		mask = BIT(kq->acc->channel % 32);
		offset = ACCUM_INTD_OFFSET_STATUS(kq->acc->channel);

		dev_dbg(kdev->dev, "setup-notify: re-triggering irq for %s\n",
			kq->acc->name);
		__raw_writel(mask, pdsp->intd + offset);

		return;
	}

	if (range->flags & RANGE_HAS_IRQ) {
		if (enabled)
			enable_irq(range->irq_base + queue);
		else
			disable_irq_nosync(range->irq_base + queue);
		return;
	}

	hwqueue_set_poll(inst, enabled);
}

static int khwq_range_setup_acc_irq(struct khwq_range_info *range, int queue,
				    bool enabled)
{
	struct khwq_device *kdev = range->kdev;
	struct khwq_acc_channel *acc;
	int ret = 0, irq;
	u32 old, new;

	if (range->flags & RANGE_MULTI_QUEUE) {
		acc = range->acc;
		irq = range->irq_base;
	} else {
		acc = range->acc + queue;
		irq = range->irq_base + queue;
	}

	old = acc->open_mask;
	if (enabled)
		new = old | BIT(queue);
	else
		new = old & ~BIT(queue);
	acc->open_mask = new;

	dev_dbg(kdev->dev, "setup-acc-irq: open mask old %08x, new %08x, channel %s\n",
		old, new, acc->name);

	if (likely(new == old))
		return 0;

	if (new && !old) {
		dev_dbg(kdev->dev, "setup-acc-irq: requesting %s for channel %s\n",
			acc->name, acc->name);
		ret = request_irq(irq, khwq_acc_int_handler, 0, acc->name,
				  range);
	}

	if (old && !new) {
		dev_dbg(kdev->dev, "setup-acc-irq: freeing %s for channel %s\n",
			acc->name, acc->name);
		free_irq(irq, range);
	}

	return ret;
}

static int khwq_setup_irq(struct hwqueue_instance *inst)
{
	struct khwq_instance *kq = hwqueue_inst_to_priv(inst);
	struct khwq_range_info *range = kq->range;
	unsigned queue = hwqueue_inst_to_id(inst) - range->queue_base;
	int ret, irq;

	if ((range->flags & RANGE_HAS_IRQ) == 0)
		return 0;

	if (range->flags & RANGE_HAS_ACCUMULATOR)
		return khwq_range_setup_acc_irq(range, queue, true);

	irq = range->irq_base + queue;

	ret = request_irq(irq, khwq_int_handler, 0, kq->irq_name, inst);
	if (ret >= 0)
		disable_irq(irq);

	return ret;
}

static void khwq_free_irq(struct hwqueue_instance *inst)
{
	struct khwq_instance *kq = hwqueue_inst_to_priv(inst);
	struct khwq_range_info *range = kq->range;
	unsigned id = hwqueue_inst_to_id(inst) - range->queue_base;
	int irq;

	if ((range->flags & RANGE_HAS_IRQ) == 0)
		return;

	if (range->flags & RANGE_HAS_ACCUMULATOR) {
		khwq_range_setup_acc_irq(range, id, false);
		return;
	}

	irq = range->irq_base + id;
	free_irq(irq, inst);
}

static int khwq_open(struct hwqueue_instance *inst, unsigned flags)
{
	return khwq_setup_irq(inst);
}

static void khwq_close(struct hwqueue_instance *inst)
{
	khwq_free_irq(inst);
}

static const char *khwq_acc_result_str(enum khwq_acc_result result)
{
	static const char *result_str[] = {
		[ACCUM_RET_IDLE]		= "idle",
		[ACCUM_RET_SUCCESS]		= "success",
		[ACCUM_RET_INVALID_COMMAND]	= "invalid command",
		[ACCUM_RET_INVALID_CHANNEL]	= "invalid channel",
		[ACCUM_RET_INACTIVE_CHANNEL]	= "inactive channel",
		[ACCUM_RET_ACTIVE_CHANNEL]	= "active channel",
		[ACCUM_RET_INVALID_QUEUE]	= "invalid queue",

		[ACCUM_RET_INVALID_RET]		= "invalid return code",
	};

	if (result >= ARRAY_SIZE(result_str))
		return result_str[ACCUM_RET_INVALID_RET];
	else
		return result_str[result];
}

static enum khwq_acc_result
khwq_acc_write(struct khwq_device *kdev, struct khwq_pdsp_info *pdsp,
	       struct khwq_reg_pdsp_command *cmd)
{
	u32 result;

	/* TODO: acquire hwspinlock here */

	dev_dbg(kdev->dev, "acc command %08x %08x %08x %08x %08x\n",
		cmd->command, cmd->queue_mask, cmd->list_phys,
		cmd->queue_num, cmd->timer_config);

	__raw_writel(cmd->timer_config,	&pdsp->command->timer_config);
	__raw_writel(cmd->queue_num,	&pdsp->command->queue_num);
	__raw_writel(cmd->list_phys,	&pdsp->command->list_phys);
	__raw_writel(cmd->queue_mask,	&pdsp->command->queue_mask);
	__raw_writel(cmd->command,	&pdsp->command->command);

	/* wait for the command to clear */
	do {
		result = __raw_readl(&pdsp->command->command);
	} while ((result >> 8) & 0xff);

	/* TODO: release hwspinlock here */

	return (result >> 24) & 0xff;
}

static void khwq_acc_setup_cmd(struct khwq_device *kdev,
			       struct khwq_range_info *range,
			       struct khwq_reg_pdsp_command *cmd,
			       int queue)
{
	struct khwq_acc_info *info = &range->acc_info;
	struct khwq_acc_channel *acc;
	int queue_base;
	u32 queue_mask;

	if (range->flags & RANGE_MULTI_QUEUE) {
		acc = range->acc;
		queue_base = range->queue_base;
		queue_mask = BIT(range->num_queues) - 1;
	} else {
		acc = range->acc + queue;
		queue_base = range->queue_base + queue;
		queue_mask = 0;
	}

	memset(cmd, 0, sizeof(*cmd));
	cmd->command    = acc->channel;
	cmd->queue_mask = queue_mask;
	cmd->list_phys  = acc->list_dma[0];
	cmd->queue_num  = info->list_entries << 16;
	cmd->queue_num |= queue_base;

	cmd->timer_config = ACCUM_LIST_ENTRY_TYPE << 18;
	if (range->flags & RANGE_MULTI_QUEUE)
		cmd->timer_config |= ACCUM_CFG_MULTI_QUEUE;
	cmd->timer_config |= info->pacing_mode << 16;
	cmd->timer_config |= info->timer_count;
}

static void khwq_acc_stop(struct khwq_device *kdev,
			  struct khwq_range_info *range,
			  int queue)
{
	struct khwq_reg_pdsp_command cmd;
	struct khwq_acc_channel *acc;
	enum khwq_acc_result result;

	acc = range->acc + queue;

	khwq_acc_setup_cmd(kdev, range, &cmd, queue);
	cmd.command |= ACCUM_CMD_DISABLE_CHANNEL << 8;
	result = khwq_acc_write(kdev, range->acc_info.pdsp, &cmd);

	dev_dbg(kdev->dev, "stopped acc channel %s, result %s\n",
		 acc->name, khwq_acc_result_str(result));
}

static enum khwq_acc_result khwq_acc_start(struct khwq_device *kdev,
					   struct khwq_range_info *range,
					   int queue)
{
	struct khwq_reg_pdsp_command cmd;
	struct khwq_acc_channel *acc;
	enum khwq_acc_result result;

	acc = range->acc + queue;

	khwq_acc_setup_cmd(kdev, range, &cmd, queue);
	cmd.command |= ACCUM_CMD_ENABLE_CHANNEL << 8;
	result = khwq_acc_write(kdev, range->acc_info.pdsp, &cmd);

	dev_dbg(kdev->dev, "started acc channel %s, result %s\n",
		 acc->name, khwq_acc_result_str(result));

	return result;
}

static int khwq_acc_init(struct khwq_device *kdev,
			 struct khwq_range_info *range)
{
	struct khwq_acc_channel *acc;
	enum khwq_acc_result result;
	int queue;

	for (queue = 0; queue < range->num_queues; queue++) {
		acc = range->acc + queue;

		khwq_acc_stop(kdev, range, queue);
		acc->list_index = 0;
		result = khwq_acc_start(kdev, range, queue);

		if (result != ACCUM_RET_SUCCESS)
			return -EIO;

		if (range->flags & RANGE_MULTI_QUEUE)
			return 0;
	}
	return 0;
}

static inline struct khwq_region *
khwq_find_region_by_virt(struct khwq_device *kdev, struct khwq_instance *kq,
			 void *virt)
{
	struct khwq_region *region;

	if (likely(kq->last && kq->last->virt_start <= virt &&
		   kq->last->virt_end > virt))
		return kq->last;

	for_each_region(kdev, region) {
		if (region->virt_start <= virt && region->virt_end > virt) {
			kq->last = region;
			return kq->last;
		}
	}

	return NULL;
}

static inline struct khwq_region *
khwq_find_region_by_dma(struct khwq_device *kdev, struct khwq_instance *kq,
			dma_addr_t dma)
{
	struct khwq_region *region;

	if (likely(kq->last && kq->last->dma_start <= dma &&
		   kq->last->dma_end > dma))
		return kq->last;

	for_each_region(kdev, region) {
		if (region->dma_start <= dma && region->dma_end > dma) {
			kq->last = region;
			return kq->last;
		}
	}

	return NULL;
}

static int khwq_push(struct hwqueue_instance *inst, dma_addr_t dma,
		     unsigned size)
{
	unsigned id = hwqueue_inst_to_id(inst);
	struct khwq_qmgr_info *qmgr;
	u32 val;

	qmgr = khwq_find_qmgr(inst);
	if (!qmgr)
		return -ENODEV;

	val = (u32)dma | ((size / 16) - 1);

	__raw_writel(val, &qmgr->reg_push[id].ptr_size_thresh);

	return 0;
}

static dma_addr_t khwq_pop(struct hwqueue_instance *inst, unsigned *size)
{
	struct khwq_instance *kq = hwqueue_inst_to_priv(inst);
	struct khwq_device *kdev = from_hdev(inst->hdev);
	unsigned id = hwqueue_inst_to_id(inst);
	struct khwq_qmgr_info *qmgr;
	u32 val, desc_size, idx;
	dma_addr_t dma;

	qmgr = khwq_find_qmgr(inst);
	if (unlikely(!qmgr))
		return -ENODEV;

	/* are we accumulated? */
	if (kq->descs) {
		if (unlikely(atomic_dec_return(&kq->desc_count) < 0)) {
			atomic_inc(&kq->desc_count);
			dev_dbg(kdev->dev, "acc-pop empty queue %d\n", id);
			return 0;
		}

		idx  = atomic_inc_return(&kq->desc_head);
		idx &= ACCUM_DESCS_MASK;

		val = kq->descs[idx];

		dev_dbg(kdev->dev, "acc-pop %08x (at %d) from queue %d\n",
			val, idx, id);
	} else {
		val = __raw_readl(&qmgr->reg_pop[id].ptr_size_thresh);
		if (unlikely(!val))
			return 0;
	}

	dma = val & DESC_PTR_MASK;
	desc_size = ((val & DESC_SIZE_MASK) + 1) * 16;

	if (unlikely(size))
		*size = desc_size;

	return dma;
}

static int khwq_get_count(struct hwqueue_instance *inst)
{
	struct khwq_device *kdev = from_hdev(inst->hdev);
	struct khwq_instance *kq = hwqueue_inst_to_priv(inst);
	struct khwq_range_info *range = kq->range;
	unsigned id = hwqueue_inst_to_id(inst);
	struct khwq_qmgr_info *qmgr;
	int count;

	qmgr = khwq_find_qmgr(inst);
	if (unlikely(!qmgr))
		return -EINVAL;

	if (range->flags & RANGE_HAS_ACCUMULATOR) {
		count = atomic_read(&kq->desc_count);
		dev_dbg(kdev->dev, "count %d [acc]\n", count);
	} else {
		count = __raw_readl(&qmgr->reg_peek[id].entry_count);
		dev_dbg(kdev->dev, "count %d\n", count);
	}
	return count;
}

static int khwq_flush(struct hwqueue_instance *inst)
{
	struct khwq_instance *kq = hwqueue_inst_to_priv(inst);
	unsigned id = hwqueue_inst_to_id(inst);
	struct khwq_qmgr_info *qmgr;

	qmgr = kq->qmgr;
	qmgr = khwq_find_qmgr(inst);
	if (!qmgr)
		return -ENODEV;

	atomic_set(&kq->desc_count, 0);
	__raw_writel(0, &qmgr->reg_push[id].ptr_size_thresh);
	return 0;
}

static int khwq_map(struct hwqueue_instance *inst, void *data, unsigned size,
		    dma_addr_t *dma_ptr, unsigned *size_ptr)
{
	struct khwq_device *kdev = from_hdev(inst->hdev);
	struct khwq_instance *kq = hwqueue_inst_to_priv(inst);
	struct khwq_region *region;
	dma_addr_t dma;

	region = khwq_find_region_by_virt(kdev, kq, data);

	if (unlikely(!region || size > region->desc_size))
		return -EINVAL;

	size = min(size, region->desc_size);
	size = ALIGN(size, SMP_CACHE_BYTES);
	dma = region->dma_start + (data - region->virt_start);

	if (WARN_ON(dma & DESC_SIZE_MASK))
		return -EINVAL;
	dma_sync_single_for_device(kdev->dev, dma, size, DMA_TO_DEVICE);

	*dma_ptr = dma;
	*size_ptr = size;
	return 0;
}

static void *khwq_unmap(struct hwqueue_instance *inst, dma_addr_t dma,
			unsigned desc_size)
{
	struct khwq_device *kdev = from_hdev(inst->hdev);
	struct khwq_instance *kq = hwqueue_inst_to_priv(inst);
	struct khwq_region *region;
	void *data;

	region = khwq_find_region_by_dma(kdev, kq, dma);
	if (WARN_ON(!region))
		return NULL;

	desc_size = min(desc_size, region->desc_size);

	data = region->virt_start + (dma - region->dma_start);

	dma_sync_single_for_cpu(kdev->dev, dma, desc_size, DMA_FROM_DEVICE);

	prefetch(data);

	return data;
}

static struct hwqueue_device_ops khdev_ops = {
	.match		= khwq_match,
	.open		= khwq_open,
	.set_notify	= khwq_set_notify,
	.close		= khwq_close,
	.push		= khwq_push,
	.pop		= khwq_pop,
	.get_count	= khwq_get_count,
	.flush		= khwq_flush,
	.map		= khwq_map,
	.unmap		= khwq_unmap,
};

static __devinit struct khwq_region *
khwq_find_match_region(struct khwq_device *kdev, struct khwq_pool_info *pool)
{
	struct khwq_region *region;

	for_each_region(kdev, region) {
		if (!region->desc_size)
			continue;
		if (region->desc_size != pool->desc_size)
			continue;

		if (region->fixed_mem) {
			/* memory region contains fixed address pool */
			if (pool->start_addr) {
				if (pool->start_addr != region->next_pool_addr)
					/*
					 * the new pool is not adjacent to the
					 * previous pool
					 */
					continue;
			} else
				/* desc pool is not fixed memory type */
				continue;
		} else {
			/* memory region contains dynamically allocated pools */
			if (pool->start_addr)
				continue;
		}

		/* TODO: other checks here - e.g. mem types, fixed pools */

		return region;
	}
	return NULL;
}

static __devinit struct khwq_region *
khwq_find_free_region(struct khwq_device *kdev, struct khwq_pool_info *pool)
{
	struct khwq_region *region;

	for_each_region(kdev, region)
		if (!region->desc_size)
			return region;
	return NULL;
}

/* Map the requested pools to regions, creating regions as we go */
static void __devinit khwq_map_pools(struct khwq_device *kdev)
{
	struct khwq_region *region;
	struct khwq_pool_info *pool;
	unsigned size;

	for_each_pool(kdev, pool) {
		/* figure out alignment needs for descriptors */
		size = max(16, dma_get_cache_alignment());
		pool->desc_size	= ALIGN(pool->desc_size, size);

		/* recalculate pool size post alignment */
		size = pool->desc_size * pool->num_desc;

		/* realign pool to page size */
		size = ALIGN(size, PAGE_SIZE);

		/* finally recalculate number of descriptors... phew */
		pool->num_desc = size / pool->desc_size;

		/* find a matching region, otherwise find a free region */
		region = khwq_find_match_region(kdev, pool);
		if (!region)
			region = khwq_find_free_region(kdev, pool);

		if (!region) {
			dev_err(kdev->dev, "failed to set up pool %s\n",
				pool->name);
			continue;
		}

		/* link this pool to the region... */
		pool->region = region;
		pool->region_offset = region->num_desc;

		/* ... and inflate the region accordingly */
		region->desc_size = pool->desc_size;
		region->num_desc += pool->num_desc;

		/* For fixed memory region, save the next pool addr */
		if (pool->start_addr) {
			if (!region->next_pool_addr) {
				/* set the fixed start address of the region */
				region->dma_start = pool->start_addr;
				region->next_pool_addr = pool->start_addr;
				region->fixed_mem = 1;
			}
			region->next_pool_addr += pool->num_desc * pool->desc_size;
		}


		dev_dbg(kdev->dev, "pool %s: num:%d, size:%d, "
			"start:%08x, region:%d\n", pool->name, pool->num_desc,
			pool->desc_size, pool->start_addr,
			region_index(kdev, region));
	}
}

static int __devinit khwq_setup_region(struct khwq_device *kdev,
				       struct khwq_region *region,
				       unsigned start_index,
				       unsigned max_descs)
{
	unsigned hw_num_desc, hw_desc_size, size;
	int id = region_index(kdev, region);
	struct khwq_reg_region __iomem  *regs;
	struct khwq_qmgr_info *qmgr;
	struct page *page;

	/* unused region? */
	if (!region->num_desc)
		return 0;

	max_descs = rounddown_pow_of_two(max_descs);

	/* round up num_desc to hardware needs, i.e. 2^(n+5) */
	region->num_desc = max(32u, region->num_desc);
	hw_num_desc = ilog2(region->num_desc - 1) + 1;
	region->num_desc = 1 << hw_num_desc;

	/* force fit this region into available link ram index range */
	region->num_desc = min(max_descs, region->num_desc);

	/* did we force fit ourselves into nothingness? */
	if (region->num_desc < 32) {
		region->num_desc = 0;
		return 0;
	}

	/* reserve link ram index range for this region */
	region->link_index = start_index;

	size = region->num_desc * region->desc_size;
	region->virt_start = alloc_pages_exact(size, GFP_KERNEL | GFP_DMA);
	if (!region->virt_start) {
		region->num_desc = 0;
		return 0;
	}
	region->virt_end = region->virt_start + size;
	page = virt_to_page(region->virt_start);

	region->dma_start = dma_map_page(kdev->dev, page, 0, size,
					 DMA_BIDIRECTIONAL);
	if (dma_mapping_error(kdev->dev, region->dma_start)) {
		region->num_desc = 0;
		free_pages_exact(region->virt_start, size);
		return 0;
	}
	region->dma_end  = region->dma_start + size;

	dev_dbg(kdev->dev,
		"region %d: num:%d, size:%d, link:%d, phys:%08x, virt:%p\n",
		id, region->num_desc, region->desc_size, region->link_index,
		region->dma_start, region->virt_start);

	hw_desc_size = (region->desc_size / 16) - 1;
	hw_num_desc -= 5;

	for_each_qmgr(kdev, qmgr) {
		regs = qmgr->reg_region + id;
		__raw_writel(region->dma_start, &regs->base);
		__raw_writel(start_index, &regs->start_index);
		__raw_writel(hw_desc_size << 16 | hw_num_desc,
			     &regs->size_count);
	}

	return region->num_desc;
}

static int __devinit khwq_setup_regions(struct khwq_device *kdev)
{
	unsigned size, link_index = 0;
	struct khwq_region *region;

	size = kdev->num_regions * sizeof(struct khwq_region);
	kdev->regions = devm_kzalloc(kdev->dev, size, GFP_KERNEL);
	if (!kdev->regions)
		return -ENOMEM;

	khwq_map_pools(kdev);

	/* Next, we run through the regions and set things up */
	for_each_region(kdev, region) {
		if (kdev->num_index > link_index)
			link_index += khwq_setup_region(kdev, region,
						kdev->start_index + link_index,
						kdev->num_index - link_index);
	}

	return 0;
}

/* carve out descriptors and push into named queues */
static void __devinit khwq_setup_pools(struct khwq_device *kdev)
{
	struct khwq_region *region;
	struct khwq_pool_info *pool;
	dma_addr_t dma_addr;
	unsigned dma_size;
	int ret, i;

	for_each_pool(kdev, pool) {
		pool->queue = hwqueue_open(pool->name, HWQUEUE_ANY,
					   O_CREAT | O_RDWR | O_NONBLOCK);
		if (IS_ERR_OR_NULL(pool->queue)) {
			dev_err(kdev->dev,
				"failed to open queue for pool %s, error %ld\n",
				pool->name, PTR_ERR(pool->queue));
			pool->queue = NULL;
			continue;
		}

		region = pool->region;

		if (!region || !region->num_desc) {
			dev_err(kdev->dev, "no region for pool %s\n",
				pool->name);
			continue;
		}

		pool->desc_size = region->desc_size;
		for (i = 0; i < pool->num_desc; i++) {
			int index = pool->region_offset + i;
			void *desc;

			desc = region->virt_start + region->desc_size * index;
			ret = hwqueue_map(pool->queue, desc, pool->desc_size,
					  &dma_addr, &dma_size);
			if (ret < 0) {
				WARN_ONCE(ret, "failed map pool queue %s\n",
					  pool->name);
				continue;
			}
			ret = hwqueue_push(pool->queue, dma_addr, dma_size);
			WARN_ONCE(ret, "failed push to pool queue %s\n",
				  pool->name);
		}
	}
}

static int __devinit khwq_get_link_ram(struct khwq_device *kdev,
				       const char *name,
				       struct khwq_link_ram_block *block)
{
	struct platform_device *pdev = to_platform_device(kdev->dev);
	struct device_node *node = pdev->dev.of_node;
	u32 temp[2];

	/*
	 * Note: link ram resources are specified in "entry" sized units. In
	 * reality, although entries are ~40bits in hardware, we treat them as
	 * 64-bit entities here.
	 *
	 * For example, to specify the internal link ram for Keystone-I class
	 * devices, we would set the linkram0 resource to 0x80000-0x83fff.
	 *
	 * This gets a bit weird when other link rams are used.  For example,
	 * if the range specified is 0x0c000000-0x0c003fff (i.e., 16K entries
	 * in MSMC SRAM), the actual memory used is 0x0c000000-0x0c020000,
	 * which accounts for 64-bits per entry, for 16K entries.
	 */
	if (!of_property_read_u32_array(node, name , temp, 2)) {
		if (temp[0]) {
			/*
			 * queue_base specified => using internal or onchip
			 * link ram WARNING - we do not "reserve" this block
			 */
			block->phys = (dma_addr_t)temp[0];
			block->virt = NULL;
			block->size = temp[1];
		} else {
			block->size = temp[1];
			/* queue_base not specific => allocate requested size */
			block->virt = dmam_alloc_coherent(kdev->dev,
							  8 * block->size, &block->phys,
							  GFP_KERNEL);
			if (!block->virt) {
				dev_err(kdev->dev, "failed to alloc linkram\n");
				return -ENOMEM;
			}
		}
	} else
		return -ENODEV;
	return 0;
}

static int __devinit khwq_setup_link_ram(struct khwq_device *kdev)
{
	struct khwq_link_ram_block *block;
	struct khwq_qmgr_info *qmgr;

	for_each_qmgr(kdev, qmgr) {
		block = &kdev->link_rams[0];
		dev_dbg(kdev->dev, "linkram0: phys:%x, virt:%p, size:%x\n",
			block->phys, block->virt, block->size);
		__raw_writel(block->phys, &qmgr->reg_config->link_ram_base0);
		__raw_writel(block->size, &qmgr->reg_config->link_ram_size0);

		block++;
		if (!block->size)
			return 0;

		dev_dbg(kdev->dev, "linkram1: phys:%x, virt:%p, size:%x\n",
			block->phys, block->virt, block->size);
		__raw_writel(block->phys, &qmgr->reg_config->link_ram_base1);
	}

	return 0;
}

static const char *khwq_find_name(struct device_node *node)
{
	const char *name;

	if (of_property_read_string(node, "label", &name) < 0)
		name = node->name;
	if (!name)
		name = "unknown";
	return name;
}

static int khwq_init_acc_range(struct khwq_device *kdev,
			       struct device_node *node,
			       struct khwq_range_info *range)
{
	struct khwq_acc_channel *acc;
	struct khwq_pdsp_info *pdsp;
	struct khwq_acc_info *info;
	int ret, channel, channels;
	int list_size, mem_size;
	dma_addr_t list_dma;
	void *list_mem;
	u32 config[5];

	range->flags |= RANGE_HAS_ACCUMULATOR;
	info = &range->acc_info;

	ret = of_property_read_u32_array(node, "accumulator", config, 5);
	if (ret)
		return ret;

	info->pdsp_id		= config[0];
	info->start_channel	= config[1];
	info->list_entries	= config[2];
	info->pacing_mode	= config[3];
	info->timer_count	= config[4] / ACCUM_DEFAULT_PERIOD;

	if (info->start_channel > ACCUM_MAX_CHANNEL) {
		dev_err(kdev->dev, "channel %d invalid for range %s\n",
			info->start_channel, range->name);
		return -EINVAL;
	}

	if (info->pacing_mode > 3) {
		dev_err(kdev->dev, "pacing mode %d invalid for range %s\n",
			info->pacing_mode, range->name);
		return -EINVAL;
	}

	pdsp = khwq_find_pdsp(kdev, info->pdsp_id);
	if (!pdsp) {
		dev_err(kdev->dev, "pdsp id %d not found for range %s\n",
			info->pdsp_id, range->name);
		return -EINVAL;
	}
	info->pdsp = pdsp;

	channels = range->num_queues;

	if (of_get_property(node, "multi-queue", NULL)) {
		range->flags |= RANGE_MULTI_QUEUE;
		channels = 1;
		if (range->queue_base & (32 - 1)) {
			dev_err(kdev->dev,
				"misaligned multi-queue accumulator range %s\n",
				range->name);
			return -EINVAL;
		}
		if (range->num_queues > 32) {
			dev_err(kdev->dev,
				"too many queues in accumulator range %s\n",
				range->name);
			return -EINVAL;
		}
	}

	/* figure out list size */
	list_size  = info->list_entries;
	list_size *= ACCUM_LIST_ENTRY_WORDS * sizeof(u32);
	info->list_size = list_size;

	mem_size   = PAGE_ALIGN(list_size * 2);
	info->mem_size  = mem_size;

	range->acc = kzalloc(channels * sizeof(*range->acc), GFP_KERNEL);
	if (!range->acc)
		return -ENOMEM;

	for (channel = 0; channel < channels; channel++) {
		acc = range->acc + channel;
		acc->channel = info->start_channel + channel;

		/* allocate memory for the two lists */
		list_mem = alloc_pages_exact(mem_size, GFP_KERNEL | GFP_DMA);
		if (!list_mem)
			return -ENOMEM;

		list_dma = dma_map_single(kdev->dev, list_mem, mem_size,
					  DMA_BIDIRECTIONAL);
		if (dma_mapping_error(kdev->dev, list_dma)) {
			free_pages_exact(list_mem, mem_size);
			return -ENOMEM;
		}

		memset(list_mem, 0, mem_size);
		dma_sync_single_for_device(kdev->dev, list_dma, mem_size, DMA_TO_DEVICE);

		scnprintf(acc->name, sizeof(acc->name), "hwqueue-acc-%d", acc->channel);

		acc->list_cpu[0] = list_mem;
		acc->list_cpu[1] = list_mem + list_size;
		acc->list_dma[0] = list_dma;
		acc->list_dma[1] = list_dma + list_size;

		dev_dbg(kdev->dev, "%s: channel %d, phys %08x, virt %8p\n",
			 acc->name, acc->channel, list_dma, list_mem);
	}

	return 0;
}

static void khwq_free_acc_range(struct khwq_device *kdev,
				struct khwq_range_info *range)
{
	struct khwq_acc_channel *acc;
	struct khwq_acc_info *info;
	int channel, channels;

	info = &range->acc_info;

	if (range->flags & RANGE_MULTI_QUEUE)
		channels = 1;
	else
		channels = range->num_queues;

	for (channel = 0; channel < channels; channel++) {
		acc = range->acc + channel;
		if (!acc->list_cpu[0])
			continue;
		dma_unmap_single(kdev->dev, acc->list_dma[0],
				 info->mem_size, DMA_BIDIRECTIONAL);
		free_pages_exact(acc->list_cpu[0], info->mem_size);
	}
	kfree(range->acc);
}

static int khwq_init_queue_range(struct khwq_device *kdev,
				 struct device_node *node)
{
	struct device *dev = kdev->dev;
	struct khwq_range_info *range;
	struct khwq_qmgr_info *qmgr;
	u32 temp[2], start, end, id, index;
	int ret;

	range = devm_kzalloc(dev, sizeof(*range), GFP_KERNEL);
	if (!range) {
		dev_err(dev, "out of memory allocating range\n");
		return -ENOMEM;
	}

	range->kdev = kdev;
	range->name = khwq_find_name(node);

	ret = of_property_read_u32_array(node, "values", temp, 2);
	if (!ret) {
		range->queue_base = temp[0] - kdev->base_id;
		range->num_queues = temp[1];
	} else {
		dev_err(dev, "invalid queue range %s\n", range->name);
		devm_kfree(dev, range);
		return -EINVAL;
	}

	ret = of_property_read_u32(node, "irq-base", &range->irq_base);
	if (ret >= 0)
		range->flags |= RANGE_HAS_IRQ;

	if (of_get_property(node, "reserved", NULL))
		range->flags |= RANGE_RESERVED;

	if (of_get_property(node, "accumulator", NULL)) {
		ret = khwq_init_acc_range(kdev, node, range);
		if (ret < 0) {
			devm_kfree(dev, range);
			return ret;
		}
	}

	/* set threshold to 1, and flush out the queues */
	for_each_qmgr(kdev, qmgr) {
		start = max(qmgr->start_queue, range->queue_base);
		end   = min(qmgr->start_queue + qmgr->num_queues,
			    range->queue_base + range->num_queues);
		for (id = start; id < end; id++) {
			index = id - qmgr->start_queue;
			__raw_writel(THRESH_GTE | 1,
				     &qmgr->reg_peek[index].ptr_size_thresh);
			__raw_writel(0, &qmgr->reg_push[index].ptr_size_thresh);
		}
	}

	list_add_tail(&range->list, &kdev->queue_ranges);

	dev_dbg(dev, "added range %s: %d-%d, irqs %d-%d%s%s%s\n",
		range->name, range->queue_base,
		range->queue_base + range->num_queues - 1,
		range->irq_base,
		range->irq_base + range->num_queues - 1,
		(range->flags & RANGE_HAS_IRQ) ? ", has irq" : "",
		(range->flags & RANGE_RESERVED) ? ", reserved" : "",
		(range->flags & RANGE_HAS_ACCUMULATOR) ? ", acc" : "");

	return 0;
}

static void khwq_free_queue_range(struct khwq_device *kdev,
				  struct khwq_range_info *range)
{
	if (range->flags & RANGE_HAS_ACCUMULATOR)
		khwq_free_acc_range(kdev, range);
	list_del(&range->list);
	devm_kfree(kdev->dev, range);
}

static int khwq_init_queue_ranges(struct khwq_device *kdev,
				  struct device_node *queues)
{
	struct device_node *child;
	int ret;

	for_each_child_of_node(queues, child) {
		ret = khwq_init_queue_range(kdev, child);
		/* return value ignored, we init the rest... */
	}

	/* ... and barf if they all failed! */
	if (list_empty(&kdev->queue_ranges)) {
		dev_err(kdev->dev, "no valid queue range found\n");
		return -ENODEV;
	}

	return 0;
}

static void khwq_free_queue_ranges(struct khwq_device *kdev)
{
	struct khwq_range_info *range;

	for (;;) {
		range = first_queue_range(kdev);
		if (!range)
			break;
		khwq_free_queue_range(kdev, range);
	}
}

static int khwq_init_pools(struct khwq_device *kdev, struct device_node *pools)
{
	struct device *dev = kdev->dev;
	struct khwq_pool_info *pool;
	struct device_node *child;
	u32 temp[2];
	int ret;

	for_each_child_of_node(pools, child) {

		pool = devm_kzalloc(dev, sizeof(*pool), GFP_KERNEL);
		if (!pool) {
			dev_err(dev, "out of memory allocating pool\n");
			return -ENOMEM;
		}

		pool->name = khwq_find_name(child);

		ret = of_property_read_u32_array(child, "values", temp, 2);
		if (!ret) {
			pool->num_desc  = temp[0];
			pool->desc_size = temp[1];
		} else {
			dev_err(dev, "invalid queue pool %s\n", pool->name);
			devm_kfree(dev, pool);
			continue;
		}

		ret = of_property_read_u32(child, "address", &pool->start_addr);
		if (ret < 0)
			pool->start_addr = 0;	/* desc pool buffer allocated dynamically */

		list_add_tail(&pool->list, &kdev->pools);

		dev_info(dev, "added pool %s: %d descriptors of size %d\n",
			pool->name, pool->num_desc, pool->desc_size);
	}

	if (list_empty(&kdev->pools)) {
		dev_err(dev, "no valid descriptor pool found\n");
		return -ENODEV;
	}

	return 0;
}

static int khwq_init_qmgrs(struct khwq_device *kdev, struct device_node *qmgrs)
{
	struct device *dev = kdev->dev;
	struct khwq_qmgr_info *qmgr;
	struct device_node *child;
	u32 temp[2];
	int ret;

	for_each_child_of_node(qmgrs, child) {
		qmgr = devm_kzalloc(dev, sizeof(*qmgr), GFP_KERNEL);
		if (!qmgr) {
			dev_err(dev, "out of memory allocating qmgr\n");
			return -ENOMEM;
		}

		ret = of_property_read_u32_array(child, "managed-queues",
						 temp, 2);
		if (!ret) {
			qmgr->start_queue = temp[0];
			qmgr->num_queues = temp[1];
		} else {
			dev_err(dev, "invalid qmgr queue range\n");
			devm_kfree(dev, qmgr);
			continue;
		}

		dev_info(dev, "qmgr start queue %d, number of queues %d\n",
		       qmgr->start_queue, qmgr->num_queues);

		qmgr->reg_peek		= of_iomap(child, 0);
		qmgr->reg_status	= of_iomap(child, 1);
		qmgr->reg_config	= of_iomap(child, 2);
		qmgr->reg_region	= of_iomap(child, 3);
		qmgr->reg_push		= of_iomap(child, 4);
		qmgr->reg_pop		= of_iomap(child, 5);

		if (!qmgr->reg_peek || !qmgr->reg_status || !qmgr->reg_config ||
		    !qmgr->reg_region || !qmgr->reg_push || !qmgr->reg_pop) {
			dev_err(dev, "failed to map qmgr regs\n");
			if (qmgr->reg_peek)
				iounmap(qmgr->reg_peek);
			if (qmgr->reg_status)
				iounmap(qmgr->reg_status);
			if (qmgr->reg_config)
				iounmap(qmgr->reg_config);
			if (qmgr->reg_region)
				iounmap(qmgr->reg_region);
			if (qmgr->reg_push)
				iounmap(qmgr->reg_push);
			if (qmgr->reg_pop)
				iounmap(qmgr->reg_pop);
			kfree(qmgr);
			continue;
		}

		list_add_tail(&qmgr->list, &kdev->qmgrs);

		dev_info(dev, "added qmgr start queue %d, num of queues %d, "
				"reg_peek %p, reg_status %p, reg_config %p, "
				"reg_region %p, reg_push %p, reg_pop %p\n",
				qmgr->start_queue, qmgr->num_queues,
				qmgr->reg_peek, qmgr->reg_status,
				qmgr->reg_config, qmgr->reg_region,
				qmgr->reg_push, qmgr->reg_pop);
	}

	return 0;
}

static int khwq_init_pdsps(struct khwq_device *kdev, struct device_node *pdsps)
{
	struct device *dev = kdev->dev;
	struct khwq_pdsp_info *pdsp;
	struct device_node *child;
	int ret;

	for_each_child_of_node(pdsps, child) {

		pdsp = devm_kzalloc(dev, sizeof(*pdsp), GFP_KERNEL);
		if (!pdsp) {
			dev_err(dev, "out of memory allocating pdsp\n");
			return -ENOMEM;
		}

		pdsp->name = khwq_find_name(child);

		ret = of_property_read_string(child, "firmware",
					      &pdsp->firmware);
		if (ret < 0 || !pdsp->firmware) {
			dev_err(dev, "unknown firmware for pdsp %s\n",
				pdsp->name);
			kfree(pdsp);
			continue;
		}
		dev_dbg(dev, "pdsp name %s fw name :%s\n",
			pdsp->name, pdsp->firmware);

		pdsp->iram	= of_iomap(child, 0);
		pdsp->regs	= of_iomap(child, 1);
		pdsp->intd	= of_iomap(child, 2);
		pdsp->command	= of_iomap(child, 3);
		if (!pdsp->command || !pdsp->iram || !pdsp->regs || !pdsp->intd) {
			dev_err(dev, "failed to map pdsp %s regs\n",
				pdsp->name);
			if (pdsp->command)
				devm_iounmap(dev, pdsp->command);
			if (pdsp->iram)
				devm_iounmap(dev, pdsp->iram);
			if (pdsp->regs)
				devm_iounmap(dev, pdsp->regs);
			if (pdsp->intd)
				devm_iounmap(dev, pdsp->intd);
			kfree(pdsp);
			continue;
		}
		of_property_read_u32(child, "id", &pdsp->id);

		list_add_tail(&pdsp->list, &kdev->pdsps);

		dev_dbg(dev, "added pdsp %s: command %p, iram %p, "
			"regs %p, intd %p, firmware %s\n",
			pdsp->name, pdsp->command, pdsp->iram, pdsp->regs,
			pdsp->intd, pdsp->firmware);
	}

	return 0;
}


static int khwq_stop_pdsp(struct khwq_device *kdev,
			  struct khwq_pdsp_info *pdsp)
{
	u32 val, timeout = 1000;
	int ret;

	val = __raw_readl(&pdsp->regs->control) & ~PDSP_CTRL_ENABLE;
	__raw_writel(val, &pdsp->regs->control);

	ret = khwq_pdsp_wait(&pdsp->regs->control, timeout, PDSP_CTRL_RUNNING);

	if (ret < 0) {
		dev_err(kdev->dev, "timed out on pdsp %s stop\n", pdsp->name);
		return ret;
	}
	return 0;
}

static int khwq_load_pdsp(struct khwq_device *kdev,
			  struct khwq_pdsp_info *pdsp)
{
	int i, ret, fwlen;
	const struct firmware *fw;
	u32 *fwdata;

	ret = request_firmware(&fw, pdsp->firmware, kdev->dev);
	if (ret) {
		dev_err(kdev->dev, "failed to get firmware %s for pdsp %s\n",
			pdsp->firmware, pdsp->name);
		return ret;
	}

	/* download the firmware */
	fwdata = (u32 *)fw->data;
	fwlen = (fw->size + sizeof(u32) - 1) / sizeof(u32);

	for (i = 0; i < fwlen; i++)
		__raw_writel(fwdata[i], pdsp->iram + i);

	release_firmware(fw);

	return 0;
}

static int khwq_start_pdsp(struct khwq_device *kdev,
			   struct khwq_pdsp_info *pdsp)
{
	u32 val, timeout = 1000;
	int ret;

	/* write a command for sync */
	__raw_writel(0xffffffff, &pdsp->command->command);
	while (__raw_readl(&pdsp->command->command) != 0xffffffff)
		cpu_relax();

	/* soft reset the PDSP */
	val  = __raw_readl(&pdsp->regs->control);
	val &= ~(PDSP_CTRL_PC_MASK | PDSP_CTRL_SOFT_RESET);
	__raw_writel(val, &pdsp->regs->control);

	/* enable pdsp */
	val = __raw_readl(&pdsp->regs->control) | PDSP_CTRL_ENABLE;
	__raw_writel(val, &pdsp->regs->control);

	/* wait for command register to clear */
	ret = khwq_pdsp_wait(&pdsp->command->command, timeout, 0);

	if (ret < 0) {
		dev_err(kdev->dev, "timed out on pdsp %s command register wait\n",
			pdsp->name);
		return ret;
	}
	return 0;
}

static void khwq_stop_pdsps(struct khwq_device *kdev)
{
	struct khwq_pdsp_info *pdsp;

	/* disable all pdsps */
	for_each_pdsp(kdev, pdsp)
		khwq_stop_pdsp(kdev, pdsp);
}

static int khwq_start_pdsps(struct khwq_device *kdev)
{
	struct khwq_pdsp_info *pdsp;
	int ret;

	khwq_stop_pdsps(kdev);

	/* now load them all */
	for_each_pdsp(kdev, pdsp) {
		ret = khwq_load_pdsp(kdev, pdsp);
		if (ret < 0)
			return ret;
	}

	for_each_pdsp(kdev, pdsp) {
		ret = khwq_start_pdsp(kdev, pdsp);
		WARN_ON(ret);
	}

	return 0;
}

static int khwq_init_accs(struct khwq_device *kdev)
{
	struct khwq_range_info *range;
	int ret;

	for_each_queue_range(kdev, range) {
		if (!(range->flags & RANGE_HAS_ACCUMULATOR))
			continue;
		ret = khwq_acc_init(kdev, range);
		if (ret < 0)
			return ret;
	}
	return 0;
}

static int khwq_init_queue(struct khwq_device *kdev,
			   struct khwq_range_info *range,
			   struct hwqueue_instance *inst)
{
	struct khwq_instance *kq = hwqueue_inst_to_priv(inst);
	unsigned id = hwqueue_inst_to_id(inst) - range->queue_base;

	kq->kdev = kdev;
	kq->range = range;
	kq->irq_num = -1;

	scnprintf(kq->irq_name, sizeof(kq->irq_name),
		  "hwqueue-%d", range->queue_base + id);

	if (range->flags & RANGE_HAS_ACCUMULATOR) {
		kq->descs = kzalloc(ACCUM_DESCS_MAX * sizeof(u32), GFP_KERNEL);
		if (!kq->descs)
			return -ENOMEM;

		kq->acc = range->acc;
		if ((range->flags & RANGE_MULTI_QUEUE) == 0)
			kq->acc += id;
	}

	return 0;
}

static int khwq_init_queues(struct khwq_device *kdev)
{
	struct hwqueue_device *hdev = to_hdev(kdev);
	struct khwq_range_info *range;
	int id, ret;

	for_each_queue_range(kdev, range) {
		for (id = range->queue_base;
		     id < range->queue_base + range->num_queues; id++) {
			ret = khwq_init_queue(kdev, range,
					      hwqueue_id_to_inst(hdev, id));
			if (ret < 0)
				return ret;
		}
	}
	return 0;
}

static int __devinit khwq_probe(struct platform_device *pdev)
{
	struct device_node *node = pdev->dev.of_node;
	struct device_node *qmgrs, *pdsps, *descs, *queues;
	struct device *dev = &pdev->dev;
	struct hwqueue_device *hdev;
	struct khwq_device *kdev;
	u32 temp[2];
	int ret;

	if (!node) {
		dev_err(dev, "device tree info unavailable\n");
		return -ENODEV;
	}

	qmgrs =  of_find_child_by_name(node, "qmgrs");
	BUG_ON(!qmgrs);
	if (!qmgrs) {
		dev_err(dev, "queue manager info not specified\n");
		return -ENODEV;
	}

	queues = of_find_child_by_name(node, "queues");
	if (!queues) {
		dev_err(dev, "queues not specified\n");
		return -ENODEV;
	}
	pdsps =  of_find_child_by_name(node, "pdsps");
	BUG_ON(!pdsps);
	if (!pdsps) {
		dev_err(dev, "pdsp info not specified\n");
		return -ENODEV;
	}
	descs =  of_find_child_by_name(node, "descriptors");
	if (!descs) {
		dev_err(dev, "descriptor pools not specified\n");
		return -ENODEV;
	}

	kdev = devm_kzalloc(dev, sizeof(struct khwq_device), GFP_KERNEL);
	if (!kdev) {
		dev_err(dev, "memory allocation failed\n");
		return -ENOMEM;
	}

	platform_set_drvdata(pdev, kdev);
	kdev->dev = dev;
	INIT_LIST_HEAD(&kdev->queue_ranges);
	INIT_LIST_HEAD(&kdev->qmgrs);
	INIT_LIST_HEAD(&kdev->pools);
	INIT_LIST_HEAD(&kdev->pdsps);

	if (of_property_read_u32_array(node, "range", temp, 2)) {
		dev_err(dev, "hardware queue range not specified\n");
		return -ENODEV;
	}
	kdev->base_id    = temp[0];
	kdev->num_queues = temp[1];

	/* Initialize queue managers using device tree configuration */
	ret = khwq_init_qmgrs(kdev, qmgrs);
	if (ret)
		return ret;

	/*
	 * TODO: failure handling in this code is somewhere between moronic
	 * and non-existant - needs to be fixed
	 */

	/* get pdsp configuration values from device tree */
	if (pdsps) {
		ret = khwq_init_pdsps(kdev, pdsps);
		if (ret)
			return ret;

		ret = khwq_start_pdsps(kdev);
		if (ret)
			return ret;
	}

	/* get usable queue range values from device tree */
	ret = khwq_init_queue_ranges(kdev, queues);
	if (ret)
		return ret;

	/* Get descriptor pool values from device tree */
	ret = khwq_init_pools(kdev, descs);
	if (ret) {
		khwq_free_queue_ranges(kdev);
		khwq_stop_pdsps(kdev);
		return ret;
	}

	if (!of_property_read_u32_array(node, "regions", temp, 2)) {
		kdev->start_region = temp[0];
		kdev->num_regions = temp[1];
	}

	BUG_ON(!kdev->num_regions);
	dev_dbg(kdev->dev, "regions: %d-%d\n", kdev->start_region,
		kdev->start_region + kdev->num_regions - 1);

	if (!of_property_read_u32_array(node, "link-index", temp, 2)) {
		kdev->start_index = temp[0];
		kdev->num_index = temp[1];
	}

	BUG_ON(!kdev->num_index);
	dev_dbg(kdev->dev, "link-index: %d-%d\n", kdev->start_index,
		kdev->start_index + kdev->num_index - 1);

	ret = khwq_get_link_ram(kdev, "linkram0", &kdev->link_rams[0]);
	if (ret) {
		dev_err(kdev->dev, "could not setup linking ram\n");
		return ret;
	}

	ret = khwq_get_link_ram(kdev, "linkram1", &kdev->link_rams[1]);
	if (ret) {
		/*
		 * nothing really, we have one linking ram already, so we just
		 * live within our means
		 */
	}

	ret = khwq_setup_link_ram(kdev);
	if (ret)
		return ret;

	ret = khwq_setup_regions(kdev);
	if (ret)
		return ret;

	ret = khwq_init_accs(kdev);
	if (ret) {
		khwq_free_queue_ranges(kdev);
		khwq_stop_pdsps(kdev);
		return ret;
	}

	/* initialize hwqueue device data */
	hdev = to_hdev(kdev);
	hdev->dev	 = dev;
	hdev->base_id	 = kdev->base_id;
	hdev->num_queues = kdev->num_queues;
	hdev->priv_size	 = sizeof(struct khwq_instance);
	hdev->ops	 = &khdev_ops;

	/* register the hwqueue device */
	ret = hwqueue_device_register(hdev);
	if (ret < 0) {
		dev_err(dev, "hwqueue registration failed\n");
		return ret;
	}

	ret = khwq_init_queues(kdev);
	if (ret < 0) {
		dev_err(dev, "hwqueue initialization failed\n");
		return ret;
	}

	khwq_setup_pools(kdev);

	return 0;
}

static int __devexit khwq_remove(struct platform_device *pdev)
{
	struct khwq_device *kdev = platform_get_drvdata(pdev);
	struct hwqueue_device *hdev = to_hdev(kdev);
	int ret;

	ret = hwqueue_device_unregister(hdev);
	if (ret < 0) {
		dev_err(&pdev->dev, "hwqueue unregistration failed\n");
		return ret;
	}

	return 0;
}

/* Match table for of_platform binding */
static struct of_device_id __devinitdata keystone_hwqueue_of_match[] = {
	{ .compatible = "ti,keystone-hwqueue", },
	{},
};
MODULE_DEVICE_TABLE(of, keystone_hwqueue_of_match);

static struct platform_driver keystone_hwqueue_driver = {
	.probe		= khwq_probe,
	.remove		= __devexit_p(khwq_remove),
	.driver		= {
		.name	= "keystone-hwqueue",
		.owner	= THIS_MODULE,
		.of_match_table = keystone_hwqueue_of_match,
	},
};

static int __init keystone_hwqueue_init(void)
{
	return platform_driver_register(&keystone_hwqueue_driver);
}
subsys_initcall(keystone_hwqueue_init);

static void __exit keystone_hwqueue_exit(void)
{
	platform_driver_unregister(&keystone_hwqueue_driver);
}
module_exit(keystone_hwqueue_exit);

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("Hardware queue driver for Keystone devices");
