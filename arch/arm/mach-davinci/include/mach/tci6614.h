/*
 * Texas Instruments TCI6614 SoC Specific Defines
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
#ifndef __ASM_ARCH_DAVINCI_TCI6614_H
#define __ASM_ARCH_DAVINCI_TCI6614_H

#include <asm/sizes.h>

#define TCI6614_DDR_BASE	0x80000000

/*
 * Fixed mapping for early init starts here. If low-level debug is enabled,
 * this area also gets mapped via io_pg_offset and io_phys by the boot code.
 * To fit in with the io_pg_offset calculation, the io base address selected
 * here _must_ be a multiple of 2^21.
 */
#define TCI6614_IO_BASE	0x02000000

#define TCI6614_N_GPIO	32

#ifndef __ASSEMBLY__

#include <linux/serial_8250.h>

#include <mach/nand.h>
#include <mach/serial.h>
#include <mach/i2c.h>
#include <mach/spi.h>


struct tci6614_device_info {
	struct davinci_uart_config	*serial_config;
	struct davinci_nand_pdata	*nand_config[4]; /* 4 chipsels */
	struct davinci_i2c_platform_data *i2c_config;
	struct davinci_spi_platform_data *spi_config;
};

extern void __init omap_aintc_init(void);
extern struct platform_device tci6614_serial_device;

extern void __init tci6614_init(void);
extern void __init tci6614_devices_init(struct tci6614_device_info *);
extern void __init tci6614_irq_init(void);
extern void __init tci6614_intc_init(void);
extern void tci6614_restart(char mode, const char *cmd);

#endif

#endif /* __ASM_ARCH_DAVINCI_TCI6614_H */
