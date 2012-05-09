/*
 * Texas Instruments Keystone IPC IRQ chip
 *
 * Copyright (C) 2012 Texas Instruments, Inc.
 * Author: Sajesh Kumar Saran <sajesh@ti.com>
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

#include <linux/io.h>
#include <linux/of.h>
#include <linux/irq.h>
#include <linux/err.h>
#include <linux/bitops.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/of_address.h>
#include <linux/platform_device.h>

/* The source ID bits start from 4 to 31 (total 28 bits)*/
#define BIT_OFS			4
#define N_IPC_IRQ		(32 - BIT_OFS)
#define BIT_TO_IRQ(x)		(kipc->base + (x) - BIT_OFS)

static bool debug_keystone_ipc;

struct keystone_ipc_device {
	void __iomem		*grh;
	void __iomem		*arh;
	struct irq_chip		 chip;
	struct device		*dev;
	u32			 base;
	u32			 mask;
	u32			 irq;
};

#define from_irq_chip(ipc)		\
	container_of(ipc, struct keystone_ipc_device, chip)
#define to_irq_chip(ipc)		\
	(&(ipc)->chip)

static inline u32 keystone_ipc_readl(struct keystone_ipc_device *kipc)
{
	return __raw_readl(kipc->arh);
}

static inline void keystone_ipc_writel(struct keystone_ipc_device *kipc,
				       u32 value)
{
	__raw_writel(value, kipc->arh);
}

static void keystone_ipc_mask_irq(struct irq_data *d)
{
	struct keystone_ipc_device *kipc = from_irq_chip(d->chip);
	unsigned int irq = d->irq - kipc->base + BIT_OFS;

	kipc->mask |= BIT(irq);
	if (debug_keystone_ipc)
		dev_dbg(kipc->dev, "ipc irq: mask %d [%x]\n", irq, kipc->mask);
}

void keystone_ipc_unmask_irq(struct irq_data *d)
{
	struct keystone_ipc_device *kipc = from_irq_chip(d->chip);
	unsigned int irq = d->irq - kipc->base + BIT_OFS;

	kipc->mask &= ~BIT(irq);
	if (debug_keystone_ipc)
		dev_dbg(kipc->dev, "ipc irq: unmask %d [%x]\n",
			irq, kipc->mask);
}

void keystone_ipc_ack_irq(struct irq_data *d)
{
	struct keystone_ipc_device *kipc = from_irq_chip(d->chip);
	unsigned int irq = d->irq - kipc->base + BIT_OFS;

	/* nothing to do here */
	if (debug_keystone_ipc)
		dev_dbg(kipc->dev, "ipc irq: ack %d [%x]\n", irq, kipc->mask);
}

void keystone_ipc_irq_handler(unsigned irq, struct irq_desc *desc)
{
	struct irq_chip *chip = irq_desc_get_chip(desc);
	struct irq_data *irq_data = irq_desc_get_irq_data(desc);
	struct keystone_ipc_device *kipc = irq_desc_get_handler_data(desc);
	unsigned long pending;
	int src;

	if (debug_keystone_ipc)
		dev_dbg(kipc->dev, "ipc irq: start irq %d\n", irq);

	chip->irq_mask(irq_data);
	chip->irq_ack(irq_data);

	pending = keystone_ipc_readl(kipc);
	keystone_ipc_writel(kipc, pending);

	if (debug_keystone_ipc)
		dev_dbg(kipc->dev, "ipc irq: pending 0x%lx, mask 0x%x\n",
			pending, kipc->mask);

	pending &= ~kipc->mask;

	if (debug_keystone_ipc)
		dev_dbg(kipc->dev, "ipc irq: pending after mask 0x%lx\n",
			pending);

	for (src = BIT_OFS; src < 32; src++)
		if (BIT(src) & pending)
			generic_handle_irq(BIT_TO_IRQ(src));

	chip->irq_unmask(irq_data);

	if (debug_keystone_ipc)
		dev_dbg(kipc->dev, "ipc irq: end irq %d\n", irq);
}

static int __devinit keystone_ipc_probe(struct platform_device *pdev)
{
	struct device_node *node = pdev->dev.of_node;
	struct device *dev = &pdev->dev;
	struct keystone_ipc_device *kipc;
	int i;

	if (!node) {
		dev_err(dev, "device tree info unavailable\n");
		return -ENODEV;
	}

	kipc = devm_kzalloc(dev, sizeof(struct keystone_ipc_device),
			    GFP_KERNEL);
	if (!kipc) {
		dev_err(dev, "memory allocation failed\n");
		return -ENOMEM;
	}

	platform_set_drvdata(pdev, kipc);
	kipc->dev = dev;

	kipc->mask = ~0x0;

	kipc->chip.name		= "keystone-ipc-irq";
	kipc->chip.irq_ack	= keystone_ipc_ack_irq;
	kipc->chip.irq_mask	= keystone_ipc_mask_irq;
	kipc->chip.irq_unmask	= keystone_ipc_unmask_irq;

	kipc->arh = of_devm_iomap(dev, 0);
	if (!kipc->arh) {
		dev_err(dev, "ipc irq: No ack register in dt binding\n");
		return -ENODEV;
	}

	kipc->grh = of_devm_iomap(dev, 1);
	if (!kipc->grh) {
		dev_err(dev, "ipc irq: No gen register in dt binding\n");
		return -ENODEV;
	}

	if (of_property_read_u32(dev->of_node, "base", &kipc->base)) {
		dev_err(dev, "ipc irq: No base in dt bindings\n");
		return -ENODEV;
	}

	if (of_property_read_u32(dev->of_node, "irq", &kipc->irq)) {
		dev_err(dev, "ipc irq: No irq in dt bindings\n");
		return -ENODEV;
	}

	for (i = 0; i < N_IPC_IRQ; i++) {
		irq_set_chip_and_handler(kipc->base + i,
					 &kipc->chip, handle_level_irq);
		set_irq_flags(kipc->base + i, IRQF_VALID | IRQF_PROBE);
	}

	irq_set_chained_handler(kipc->irq, keystone_ipc_irq_handler);
	irq_set_handler_data(kipc->irq, kipc);

	/* clear all source bits */
	keystone_ipc_writel(kipc, ~0x0);

	dev_info(dev, "ipc irq: irqchip registered, range %d-%d\n",
		 kipc->base, kipc->base + N_IPC_IRQ);
	return 0;
}

static int __devexit keystone_ipc_remove(struct platform_device *pdev)
{
	int i;
	struct keystone_ipc_device *kipc = platform_get_drvdata(pdev);

	for (i = 0; i < N_IPC_IRQ; i++)
		irq_set_chip_and_handler(kipc->base + i, &no_irq_chip, NULL);

	irq_set_handler_data(kipc->irq, NULL);
	irq_set_chained_handler(kipc->irq, NULL);

	return 0;
}

/* Match table for of_platform binding */
static struct of_device_id __devinitdata keystone_ipc_of_match[] = {
	{ .compatible = "ti,keystone-ipc-irq", },
	{},
};
MODULE_DEVICE_TABLE(of, keystone_ipc_of_match);

static struct platform_driver keystone_ipc_driver = {
	.probe		= keystone_ipc_probe,
	.remove		= __devexit_p(keystone_ipc_remove),
	.driver		= {
		.name	= "keystone-ipc-irq",
		.owner	= THIS_MODULE,
		.of_match_table = keystone_ipc_of_match,
	},
};

static int __init keystone_ipc_init(void)
{
	return platform_driver_register(&keystone_ipc_driver);
}
subsys_initcall(keystone_ipc_init);

static void __exit keystone_ipc_exit(void)
{
	platform_driver_unregister(&keystone_ipc_driver);
}
module_exit(keystone_ipc_exit);

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("IPC interrupt driver for Keystone devices");
MODULE_AUTHOR("Sajesh Kumar Saran <sajesh@ti.com>");
