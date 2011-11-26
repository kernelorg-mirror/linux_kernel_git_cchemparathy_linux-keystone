/*
 * TI Common Platform Interrupt Distributor (cp_intd) driver
 *
 * Copyright (C) 2011 Texas Instruments
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation version 2.
 *
 * This program is distributed "as is" WITHOUT ANY WARRANTY of any
 * kind, whether express or implied; without even the implied warranty
 * of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/init.h>
#include <linux/irq.h>
#include <linux/io.h>

#include <mach/common.h>
#include <mach/cp_intd.h>
#include <mach/irqs.h>

static void __iomem *intd_base;
static bool debug_intd;

#define INTD_REVISION		0x000
#define INTD_ENABLE_SET		0x100
#define INTD_ENABLE_CLEAR	0x180
#define INTD_STATUS_SET		0x200
#define INTD_STATUS_CLEAR	0x280

#define INTD_REG(irq)		(BIT_WORD(irq) << 2)
#define INTD_BIT(irq)		(BIT_MASK(irq))
#define INTD_IRQ(x)		(davinci_soc_info.intd_irq_base + (x))

static inline u32 intd_readl(int offset)
{
	u32 value = __raw_readl(intd_base + offset);
	if (debug_intd)
		printk(KERN_NOTICE \
			 "intd: read offset %x = %x\n", offset, value);
	return value;
}

static inline void intd_writel(u32 value, int offset)
{
	if (debug_intd)
		printk(KERN_NOTICE \
			"intd: write offset %x = %x\n", offset, value);
	__raw_writel(value, intd_base + offset);
}

static void intd_mask_irq(struct irq_data *d)
{
	unsigned int irq = d->irq;

	irq -= davinci_soc_info.intd_irq_base;
	if (debug_intd)
		printk(KERN_NOTICE "iirq_ntd: mask %d\n", irq);
	intd_writel(INTD_BIT(irq), INTD_ENABLE_CLEAR + INTD_REG(irq));
	intd_readl(INTD_ENABLE_SET + INTD_REG(irq));
}

static void intd_unmask_irq(struct irq_data *d)
{
	unsigned int irq = d->irq;

	irq -= davinci_soc_info.intd_irq_base;
	if (debug_intd)
		printk(KERN_NOTICE "intd: unmask %d\n", irq);
	intd_writel(INTD_BIT(irq), INTD_ENABLE_SET + INTD_REG(irq));
	intd_readl(INTD_ENABLE_SET + INTD_REG(irq));
}

static void intd_ack_irq(struct irq_data *d)
{
	unsigned int irq = d->irq;

	irq -= davinci_soc_info.intd_irq_base;
	if (debug_intd)
		printk(KERN_NOTICE "intd: ack %d\n", irq);
	intd_writel(INTD_BIT(irq), INTD_STATUS_CLEAR + INTD_REG(irq));
	intd_readl(INTD_STATUS_SET + INTD_REG(irq));
}

static bool intd_irq_pending(unsigned int irq)
{
	bool enabled, pending;

	irq -= davinci_soc_info.intd_irq_base;
	enabled = (intd_readl(INTD_ENABLE_SET + INTD_REG(irq)) &
			INTD_BIT(irq)) ? true : false;
	pending = (intd_readl(INTD_STATUS_SET + INTD_REG(irq)) &
			INTD_BIT(irq)) ? true : false;
	if (debug_intd) {
		printk(KERN_NOTICE "intd: %d %s, %s\n", irq,
		       pending ? "pending" : "not pending",
		       enabled ? "enabled" : "not enabled");
	}
	return pending;
}

static struct irq_chip intd_chip = {
	.name	= "CP_INTD",
	.irq_ack	= intd_ack_irq,
	.irq_mask	= intd_mask_irq,
	.irq_unmask	= intd_unmask_irq,
};

static void intd_irq_handler(unsigned int irq, struct irq_desc *desc)
{
	unsigned int intd_irq = davinci_soc_info.intc_to_intd_map[irq];
	struct irq_chip *chip = irq_desc_get_chip(desc);
	struct irq_data *irq_data = irq_desc_get_irq_data(desc);

	chip->irq_mask(irq_data);
	chip->irq_ack(irq_data);
	if (debug_intd)
		printk(KERN_INFO "intd irq %d --> %d\n", irq, intd_irq);
	if (intd_irq && intd_irq_pending(intd_irq))
		generic_handle_irq(intd_irq);
	chip->irq_unmask(irq_data);
}

void __init cp_intd_init(void)
{
	u32 *intc_to_intd_map	= davinci_soc_info.intc_to_intd_map;
	unsigned long num_irq	= davinci_soc_info.intd_irq_num;
	unsigned long rev;
	int i;

	intd_base = ioremap(davinci_soc_info.intd_base, SZ_4K);
	BUG_ON(!intd_base);

	rev = intd_readl(INTD_REVISION);
	printk(KERN_INFO "IRQ: intd version %ld.%ld at %p\n",
	       (rev >> 8) & 0x3, rev & 0x3f, intd_base);

	for (i = 0; i < DIV_ROUND_UP(num_irq, 32); i++) {
		intd_writel(0xffffffff, INTD_STATUS_CLEAR + (4 * i));
		intd_writel(0xffffffff, INTD_ENABLE_CLEAR + (4 * i));
	}

	for (i = 0; i < num_irq; i++) {
		irq_set_chip_and_handler(INTD_IRQ(i),
					 &intd_chip, handle_edge_irq);
		set_irq_flags(INTD_IRQ(i), IRQF_VALID | IRQF_PROBE);
	}

	if (intc_to_intd_map) {
		for (i = 0; i < davinci_soc_info.intc_irq_num; i++) {
			if (intc_to_intd_map[i])
				irq_set_chained_handler(i, intd_irq_handler);
		}
	}
}
