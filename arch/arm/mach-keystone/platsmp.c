/*
 * Copyright 2012 Texas Instruments, Inc.
 *
 * Based on platsmp.c, Copyright 2010-2011 Calxeda, Inc.
 * Based on platsmp.c, Copyright (C) 2002 ARM Ltd.
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
#include <linux/init.h>
#include <linux/smp.h>
#include <linux/io.h>

#include <asm/smp_plat.h>
#include <asm/smp_ops.h>
#include <asm/hardware/gic.h>
#include <asm/cacheflush.h>
#include <asm/tlbflush.h>
#include <asm/memory.h>

#include "keystone.h"

static void __init keystone_smp_init_cpus(void)
{
	unsigned int i, ncores;

	ncores = 4;

	/* sanity check */
	if (ncores > NR_CPUS) {
		pr_warn("restricted to %d cpus\n", NR_CPUS);
		ncores = NR_CPUS;
	}

	for (i = 0; i < ncores; i++)
		set_cpu_possible(i, true);

	set_smp_cross_call(gic_raise_softirq);
}

static void __init keystone_smp_prepare_cpus(unsigned int max_cpus)
{
	/* nothing for now */
}

static void __cpuinit keystone_secondary_initmem(void)
{
#ifdef CONFIG_ARM_LPAE
	pgd_t *pgd0 = pgd_offset_k(0);
	cpu_set_ttbr(1, __pa(pgd0) + TTBR1_OFFSET);
	local_flush_tlb_all();
#endif
}

static void __cpuinit keystone_secondary_init(unsigned int cpu)
{
	gic_secondary_init(0);
	keystone_secondary_initmem();
}

static int __cpuinit
keystone_boot_secondary(unsigned int cpu, struct task_struct *idle)
{
	unsigned long *ptr = (unsigned long *)(PAGE_OFFSET + 0x1f0);

	ptr[cpu] = virt_to_idmap(&secondary_startup);
	__cpuc_flush_dcache_area(ptr, sizeof(ptr) * 4);

	return 0;
}

struct smp_ops keystone_smp_ops __initdata = {
	smp_init_ops(keystone)
	smp_secondary_ops(keystone)
};
