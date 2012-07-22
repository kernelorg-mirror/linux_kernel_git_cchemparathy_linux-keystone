/*
 * Copyright 2010-2012 Texas Instruments, Inc.
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
 * You should have received a copy of the GNU General Public License along with
 * this program.  If not, see <http://www.gnu.org/licenses/>.
 */
#include <linux/io.h>
#include <linux/of.h>
#include <linux/init.h>
#include <linux/of_irq.h>
#include <linux/of_platform.h>

#include <asm/setup.h>
#include <asm/mach/map.h>
#include <asm/mach/arch.h>
#include <asm/mach/time.h>
#include <asm/arch_timer.h>
#include <asm/hardware/gic.h>

static struct map_desc io_desc[] = {
	{
		.virtual        = 0xfe800000UL,
		.pfn            = __phys_to_pfn(0x02000000UL),
		.length         = 0x800000UL,
		.type           = MT_DEVICE
	},
};

static void __init keystone_map_io(void)
{
	iotable_init(io_desc, ARRAY_SIZE(io_desc));
}

static const struct of_device_id irq_match[] = {
	{ .compatible = "arm,cortex-a15-gic", .data = gic_of_init, },
	{}
};

static void __init keystone_init_irq(void)
{
	of_irq_init(irq_match);
}


static void __init keystone_timer_init(void)
{
	arch_timer_of_register();
	arch_timer_sched_clock_init();
}

static struct sys_timer keystone_timer = {
	.init = keystone_timer_init,
};


static void __init keystone_init(void)
{
	of_platform_populate(NULL, of_default_bus_match_table, NULL, NULL);
}

static const char *keystone_match[] __initconst = {
	"ti,keystone-evm",
	NULL,
};

DT_MACHINE_START(KEYSTONE, "Keystone")
	.map_io		= keystone_map_io,
	.init_irq	= keystone_init_irq,
	.timer		= &keystone_timer,
	.handle_irq	= gic_handle_irq,
	.init_machine	= keystone_init,
	.dt_compat	= keystone_match,
MACHINE_END
