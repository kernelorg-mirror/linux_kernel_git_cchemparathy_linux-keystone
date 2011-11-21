/*
 * Texas Instruments TCI6614 EVM Board Support
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
#include <linux/console.h>
#include <linux/dma-mapping.h>
#include <linux/interrupt.h>
#include <linux/gpio.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/ratelimit.h>

#include <asm/mach/arch.h>
#include <asm/mach-types.h>

#include <mach/irqs.h>
#include <mach/mux.h>
#include <mach/cp_intc.h>
#include <mach/tci6614.h>

static struct davinci_uart_config serial_config __initconst = {
	.enabled_uarts	= BIT(0) | BIT(1),
};

static struct tci6614_device_info sim_device_info __initconst = {
	.serial_config		= &serial_config,
};

static __init void tci6614_sim_board_init(void)
{
	tci6614_devices_init(&sim_device_info);
}

#ifdef CONFIG_SERIAL_8250_CONSOLE
static int __init tci6614_sim_console_init(void)
{
	return add_preferred_console("ttyS", 0, "115200");
}
console_initcall(tci6614_sim_console_init);
#endif

MACHINE_START(TCI6614_SIM, "TCI6614 Simulator")
	.map_io		= tci6614_init,
	.init_irq	= tci6614_intc_init,
	.timer		= &davinci_timer,
	.init_machine	= tci6614_sim_board_init,
MACHINE_END
