/*
 * Texas Instruments TCI6614 EVM Board Support
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
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/console.h>
#include <linux/i2c.h>
#include <linux/i2c/at24.h>
#include <linux/dma-mapping.h>
#include <linux/interrupt.h>
#include <linux/gpio.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/ratelimit.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/partitions.h>
#include <linux/spi/spi.h>
#include <linux/spi/flash.h>
#include <linux/of_platform.h>

#include <asm/mach/arch.h>
#include <asm/mach-types.h>

#include <mach/irqs.h>
#include <mach/edma.h>
#include <mach/mux.h>
#include <mach/cp_intc.h>
#include <mach/tci6614.h>
#include <mach/aemif.h>

static struct mtd_partition nand_partitions[] = {
	/* U-Boot in first 1M */
	{
		.name		= "u-boot",
		.offset		= 0,
		.size		= (8 * SZ_128K),
		.mask_flags	= MTD_WRITEABLE,	/* force read-only */
	},
	/* bootloader params in the next 512K */
	{
		.name		= "params",
		.offset		= MTDPART_OFS_NXTBLK,
		.size		= (4 * SZ_128K),
		.mask_flags	= MTD_WRITEABLE,	/* force read-only */
	},
	/* kernel in the next 4M */
	{
		.name		= "kernel",
		.offset		= MTDPART_OFS_NXTBLK,
		.size		= SZ_4M,
		.mask_flags	= 0,
	},
	/* file system in the remaining */
	{
		.name		= "filesystem",
		.offset		= MTDPART_OFS_NXTBLK,
		.size		= MTDPART_SIZ_FULL,
		.mask_flags	= 0,
	}
};

static struct davinci_aemif_timing evm_nandflash_timing = {
	.wsetup		= 0xf,
	.wstrobe	= 0x3f,
	.whold		= 0x7,
	.rsetup		= 0xf,
	.rstrobe	= 0x3f,
	.rhold		= 7,
	.ta		= 3,
};

static struct davinci_nand_pdata nand_config = {
	.mask_cle	= 0x4000,
	.mask_ale	= 0x2000,
	.parts		= nand_partitions,
	.nr_parts	= ARRAY_SIZE(nand_partitions),
	.ecc_mode	= NAND_ECC_HW,
	.bbt_options	= NAND_BBT_USE_FLASH,
	.ecc_bits	= 4,
	.timing		= &evm_nandflash_timing,
};

static struct at24_platform_data at24_eeprom_data = {
	.byte_len	= 1024 * 1024 / 8,
	.page_size	= 128,
	.flags		= AT24_FLAG_ADDR16,
};

static struct i2c_board_info __initdata i2c_devices[] = {
	{
		I2C_BOARD_INFO("24c1024", 0x50),
		.platform_data = &at24_eeprom_data,
	},
};

static struct davinci_i2c_platform_data i2c_pdata = {
	.bus_freq	= 100,	/* kHz */
	.bus_delay	= 0,	/* usec */
};


static struct davinci_uart_config serial_config __initconst = {
	.enabled_uarts	= BIT(0) | BIT(1),
};

static struct mtd_partition spi_nor_partitions[] = {
	/* u-boot-spl in the first 512K */
	{
		.name		= "u-boot-spl",
		.offset		= 0,
		.size		= (4 * SZ_128K),
		.mask_flags	= MTD_WRITEABLE,	/* force read-only */
	},
	/* test block in the remaining */
	{
		.name		= "test",
		.offset		= MTDPART_OFS_NXTBLK,
		.size		= MTDPART_SIZ_FULL,
		.mask_flags	= 0,
	},
};

static struct flash_platform_data spi_nor_flash_data = {
	.name		= "m25p80",
	.parts		= spi_nor_partitions,
	.nr_parts	= ARRAY_SIZE(spi_nor_partitions),
};

static struct davinci_spi_config spi_nor_flash_cfg = {
	.io_type	= SPI_IO_TYPE_INTR,
#if 0
	.c2tdelay	= 0,
	.t2cdelay	= 0,
	.t2edelay	= 0,
	.c2edelay	= 0,
	.wdelay		= 0,
	.odd_parity	= 0,
	.parity_enable	= 0,
	.timer_disable	= 0,
#endif
};

static struct spi_board_info spi_devices[] = {
	/* NOR Flash, CS - 0 */
	{
		.modalias		= "n25q032",
		.platform_data		= &spi_nor_flash_data,
		.controller_data	= &spi_nor_flash_cfg,
		.mode			= SPI_MODE_0,
		.max_speed_hz		= 25000000,
		.bus_num		= 0,
		.chip_select		= 0,
	},
	/* EEPROM, CS - 1*/
	/* FPGA, CS - 2*/
	/* DAC, CS - 3 */
};

struct davinci_spi_platform_data spi_pdata = {
	.version	= SPI_VERSION_1,
	.intr_line	= 0,
	.num_chipselect = 4,
};

static struct tci6614_device_info evm_device_info __initconst = {
	.serial_config		= &serial_config,
	.nand_config[0]		= &nand_config,	/* chip select 0 */
	.i2c_config		= &i2c_pdata,
	.spi_config		= &spi_pdata,
};

static struct of_device_id tci6614_dt_match_table[] __initdata = {
	{ .compatible = "simple-bus",},
	{ .compatible = "ti,tci6614-bus",},
	{}
};

static __init void tci6614_evm_board_init(void)
{
	of_platform_populate(NULL, tci6614_dt_match_table, NULL, NULL);

	tci6614_devices_init(&evm_device_info);

	i2c_register_board_info(1, i2c_devices,
			ARRAY_SIZE(i2c_devices));

	spi_register_board_info(spi_devices, ARRAY_SIZE(spi_devices));
}

#ifdef CONFIG_SERIAL_8250_CONSOLE
static int __init tci6614_evm_console_init(void)
{
	return add_preferred_console("ttyS", 0, "115200");
}
console_initcall(tci6614_evm_console_init);
#endif

static void __init tci6614_evm_map_io(void)
{
	tci6614_init();
}

static const char* tci6614_dt_board_compat[] = {
	"ti,tci6614-evm",
	NULL
};

MACHINE_START(TCI6614_EVM, "TCI6614 EVM")
	.map_io		= tci6614_evm_map_io,
	.init_irq	= tci6614_intc_init,
	.timer		= &davinci_timer,
	.init_machine	= tci6614_evm_board_init,
	.dt_compat	= tci6614_dt_board_compat,
MACHINE_END
