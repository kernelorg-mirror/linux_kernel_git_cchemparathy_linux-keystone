/*
 * Texas Instruments TCI6614 SoC devices
 *
 * Copyright (C) 2010 Texas Instruments
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
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/dma-mapping.h>
#include <linux/clk.h>
#include <linux/slab.h>
#include <linux/hwspinlock.h>

#include <asm/pmu.h>

#include <mach/common.h>
#include <mach/irqs.h>
#include <mach/edma.h>
#include <mach/tci6614.h>

#include "clock.h"

/* Base addresses for on-chip devices */
#define TCI6614_TPCC_BASE			0x01c00000
#define TCI6614_TPTC0_BASE			0x01c10000
#define TCI6614_TPTC1_BASE			0x01c10400
#define TCI6614_WDOG_BASE			0x02280000
#define TCI6614_I2C_BASE			0x02530000
#define TCI6614_SPI_BASE			0x20BF0000
#define TCI6614_SEM_BASE			0x02640000
#define TCI6614_ASYNC_EMIF_CNTRL_BASE		0x20C00000
#define TCI6614_ASYNC_EMIF_DATA_CE0_BASE	0x70000000
#define TCI6614_ASYNC_EMIF_DATA_CE1_BASE	0x74000000
#define TCI6614_ASYNC_EMIF_DATA_CE2_BASE	0x78000000
#define TCI6614_ASYNC_EMIF_DATA_CE3_BASE	0x7c000000

/* TCI6614 specific EDMA3 information */
#define EDMA_TCI6614_NUM_DMACH		64
#define EDMA_TCI6614_NUM_TCC		64
#define EDMA_TCI6614_NUM_PARAMENTRY	128
#define EDMA_TCI6614_NUM_EVQUE		2
#define EDMA_TCI6614_NUM_TC		2
#define EDMA_TCI6614_CHMAP_EXIST	0
#define EDMA_TCI6614_NUM_REGIONS	4
#define TCI6614_DMACH2EVENT_MAP0	0x3C0CE000u
#define TCI6614_DMACH2EVENT_MAP1	0x000FFFFFu

static struct resource tci6614_i2c_resources[] = {
	{
		.start	= TCI6614_I2C_BASE,
		.end	= TCI6614_I2C_BASE + SZ_4K - 1,
		.flags	= IORESOURCE_MEM,
	},
	{
		.start	= IRQ_TCI6614_I2CINT,
		.end	= IRQ_TCI6614_I2CINT,
		.flags	= IORESOURCE_IRQ,
	},
};

static struct platform_device tci6614_i2c_device = {
	.name		= "i2c_davinci",
	.id		= 1,
	.num_resources	= ARRAY_SIZE(tci6614_i2c_resources),
	.resource	= tci6614_i2c_resources,
};

static int __init tci6614_register_i2c(struct davinci_i2c_platform_data *pdata)
{
	struct platform_device *pdev = &tci6614_i2c_device;

	pdev->dev.platform_data = pdata;
	return platform_device_register(pdev);
}

static struct resource spi_resources[] = {
	[0] = {
		.start	= TCI6614_SPI_BASE,
		.end	= TCI6614_SPI_BASE + SZ_4K - 1,
		.flags	= IORESOURCE_MEM,
	},
	[1] = {
		.start	= IRQ_TCI6614_SPIINT0,
		.end	= IRQ_TCI6614_SPIINT0,
		.flags	= IORESOURCE_IRQ,
	},
/* Not using DMA */
};

static struct platform_device tci6614_spi_device = {
	.name		= "spi_davinci",
	.id		= 0,
	.num_resources	= ARRAY_SIZE(spi_resources),
	.resource	= spi_resources,
};

static int __init tci6614_register_spi(struct davinci_spi_platform_data *pdata)
{
	struct platform_device *pdev = &tci6614_spi_device;

	pdev->dev.platform_data = pdata;
	return platform_device_register(pdev);
}

static struct plat_serial8250_port serial_data[] = {
	{
		.mapbase	= TCI6614_UART0_BASE,
		.irq		= IRQ_TCI6614_UARTINT0,
		.flags		= UPF_BOOT_AUTOCONF | UPF_SKIP_TEST |
				  UPF_IOREMAP,
		.iotype		= UPIO_MEM32,
		.regshift	= 2,
	},
	{
		.mapbase	= TCI6614_UART1_BASE,
		.irq		= IRQ_TCI6614_UARTINT1,
		.flags		= UPF_BOOT_AUTOCONF | UPF_SKIP_TEST |
				  UPF_IOREMAP,
		.iotype		= UPIO_MEM32,
		.regshift	= 2,
	},
	{
		.flags	= 0,
	},
};

struct platform_device tci6614_serial_device = {
	.name			= "serial8250",
	.id			= PLAT8250_DEV_PLATFORM,
	.dev.platform_data	= serial_data,
};

static const u32 emif_windows[] = {
	TCI6614_ASYNC_EMIF_DATA_CE0_BASE, TCI6614_ASYNC_EMIF_DATA_CE1_BASE,
	TCI6614_ASYNC_EMIF_DATA_CE2_BASE, TCI6614_ASYNC_EMIF_DATA_CE3_BASE,
};

static const u32 emif_window_sizes[] = { SZ_128M, SZ_64M, SZ_64M, SZ_64M };

static struct resource wdt_resources[] = {
	{
		.start	= TCI6614_WDOG_BASE,
		.end	= TCI6614_WDOG_BASE + SZ_4K - 1,
		.flags	= IORESOURCE_MEM,
	},
};

struct platform_device tci6614_wdt_device = {
	.name		= "watchdog",
	.id		= -1,
	.num_resources	= ARRAY_SIZE(wdt_resources),
	.resource	= wdt_resources,
};

static struct hwspinlock_pdata sem_data = {
	.base_id	= 0,
};

static struct resource sem_resources[] = {
	{
		.start	= TCI6614_SEM_BASE,
		.end	= TCI6614_SEM_BASE + 0x7ff,
		.flags	= IORESOURCE_MEM,
	},
};

struct platform_device tci6614_sem_device = {
	.name			= "keystone_hwspinlock",
	.id			= -1,
	.num_resources		= ARRAY_SIZE(sem_resources),
	.resource		= sem_resources,
	.dev.platform_data	= &sem_data,
};

static struct resource pmu_resources[] = {
	{
		.start	= IRQ_TCI6614_BENCH,
		.end	= IRQ_TCI6614_BENCH,
		.flags	= IORESOURCE_IRQ,
	},
};

static struct platform_device tci6614_pmu_device = {
	.name		= "arm-pmu",
	.id		= ARM_PMU_DEVICE_CPU,
	.resource	= pmu_resources,
	.num_resources	= ARRAY_SIZE(pmu_resources),
};

static int __init nand_init(int chipsel, struct davinci_nand_pdata *data)
{
	struct resource res[2];
	struct platform_device *pdev;
	u32	range;
	int	ret;

	/* Figure out the resource range from the ale/cle masks */
	range = max(data->mask_cle, data->mask_ale);
	range = PAGE_ALIGN(range + 4) - 1;

	if (range >= emif_window_sizes[chipsel])
		return -EINVAL;

	pdev = kzalloc(sizeof(*pdev), GFP_KERNEL);
	if (!pdev)
		return -ENOMEM;

	pdev->name		= "davinci_nand";
	pdev->id		= chipsel;
	pdev->dev.platform_data	= data;

	memset(res, 0, sizeof(res));

	res[0].start	= emif_windows[chipsel];
	res[0].end	= res[0].start + range;
	res[0].flags	= IORESOURCE_MEM;

	res[1].start	= TCI6614_ASYNC_EMIF_CNTRL_BASE;
	res[1].end	= res[1].start + SZ_4K - 1;
	res[1].flags	= IORESOURCE_MEM;

	ret = platform_device_add_resources(pdev, res, ARRAY_SIZE(res));
	if (ret < 0) {
		kfree(pdev);
		return ret;
	}

	return platform_device_register(pdev);
}

void __init tci6614_devices_init(struct tci6614_device_info *info)
{
	int i;

	if (info->serial_config)
		davinci_serial_init(info->serial_config);

	platform_device_register(&tci6614_wdt_device);
	platform_device_register(&tci6614_sem_device);
	platform_device_register(&tci6614_pmu_device);

	for (i = 0; i < 4; i++)
		if (info->nand_config[i])
			nand_init(i, info->nand_config[i]);
	if (info->i2c_config)
		tci6614_register_i2c(info->i2c_config);

	if (info->spi_config)
		tci6614_register_spi(info->spi_config);
}
