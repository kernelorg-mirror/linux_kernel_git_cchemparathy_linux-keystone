/*
 * linux/arch/arm/mach-omap2/irq.c
 *
 * Interrupt handler for OMAP2 boards.
 *
 * Copyright (C) 2005 Nokia Corporation
 * Author: Paul Mundt <paul.mundt@nokia.com>
 *
 * This file is subject to the terms and conditions of the GNU General Public
 * License. See the file "COPYING" in the main directory of this archive
 * for more details.
 */
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/io.h>

#include <asm/mach/irq.h>

#include <mach/hardware.h>
#include <mach/common.h>

/* selected INTC register offsets */

#define INTC_REVISION		0x0000
#define INTC_SYSCONFIG		0x0010
#define INTC_SYSSTATUS		0x0014
#define INTC_SIR		0x0040
#define INTC_CONTROL		0x0048
#define INTC_PROTECTION		0x004C
#define INTC_IDLE		0x0050
#define INTC_THRESHOLD		0x0068
#define INTC_MIR0		0x0084
#define INTC_MIR_CLEAR0		0x0088
#define INTC_MIR_SET0		0x008c
#define INTC_PENDING_IRQ0	0x0098
#define INTC_PRIORITY_IRQ0	0x0100
/* Number of IRQ state bits in each MIR register */
#define IRQ_BITS_PER_REG	32

static bool debug_aintc;
#define debug_aintc_irq(irq)	(false)

static void intc_write_reg(u32 val, u16 reg)
{
	if (debug_aintc)
		printk(KERN_NOTICE "aintc: write reg = %x, value = %x\n",
			reg, val);
	__raw_writel(val, davinci_intc_base + reg);
}

static u32 intc_read_reg(u16 reg)
{
	u32 val = __raw_readl(davinci_intc_base + reg);
	if (debug_aintc)
		printk(KERN_NOTICE "aintc: read reg = %x, value = %x\n",
			reg, val);
	return val;
}

static void omap_ack_irq(struct irq_data *d)
{
	unsigned int irq = d->irq;

	if (debug_aintc || debug_aintc_irq(irq))
		printk(KERN_NOTICE "aintc: ack %d\n", irq);
	intc_write_reg(0x1, INTC_CONTROL);
}

static void omap_mask_irq(struct irq_data *d)
{
	unsigned int irq = d->irq;
	int offset = irq & (~(IRQ_BITS_PER_REG - 1));

	if (debug_aintc || debug_aintc_irq(irq))
		printk(KERN_NOTICE "aintc: mask %d\n", irq);

	irq &= (IRQ_BITS_PER_REG - 1);

	intc_write_reg(1 << irq, INTC_MIR_SET0 + offset);
}

static void omap_unmask_irq(struct irq_data *d)
{
	unsigned int irq = d->irq;
	int offset = irq & (~(IRQ_BITS_PER_REG - 1));

	if (debug_aintc || debug_aintc_irq(irq))
		printk(KERN_NOTICE "aintc: unmask %d\n", irq);

	irq &= (IRQ_BITS_PER_REG - 1);

	intc_write_reg(1 << irq, INTC_MIR_CLEAR0 + offset);
}

static struct irq_chip omap_irq_chip = {
	.name	= "OMAP-AINTC",
	.irq_ack	= omap_ack_irq,
	.irq_mask	= omap_mask_irq,
	.irq_unmask	= omap_unmask_irq,
};

void __init omap_aintc_init(void)
{
	unsigned long nr_of_irqs = davinci_soc_info.intc_irq_num;
	unsigned long tmp;
	int i;

	davinci_intc_type = DAVINCI_INTC_TYPE_OMAP_AINTC;
	davinci_intc_base = ioremap(davinci_soc_info.intc_base, SZ_8K);

	tmp = intc_read_reg(INTC_REVISION) & 0xff;
	printk(KERN_INFO "IRQ: Found an omap-aintc at 0x%p "
			 "(revision %ld.%ld) with %ld interrupts\n",
			 davinci_intc_base, tmp >> 4, tmp & 0xf,
			 davinci_soc_info.intc_irq_num);

	tmp = intc_read_reg(INTC_SYSCONFIG);
	tmp |= 1 << 1;	/* soft reset */
	intc_write_reg(tmp, INTC_SYSCONFIG);

	while (!(intc_read_reg(INTC_SYSSTATUS) & 0x1))
		/* Wait for reset to complete */;

	for (i = 0; i < nr_of_irqs; i++) {
		intc_write_reg(davinci_soc_info.intc_irq_prios[i],
			       INTC_PRIORITY_IRQ0 + 4 * i);
		irq_set_chip_and_handler(i, &omap_irq_chip, handle_level_irq);
		set_irq_flags(i, IRQF_VALID);
	}
}
