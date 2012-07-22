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
#include <linux/init.h>
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/of_platform.h>
#include <linux/of_address.h>
#include <linux/sched.h>

#include <asm/arch_timer.h>
#include <asm/mach/map.h>
#include <asm/mach/arch.h>
#include <asm/mach/time.h>
#include <asm/hardware/gic.h>
#include <asm/setup.h>
#include <asm/pgtable.h>
#include <asm/pgalloc.h>
#include <asm/cacheflush.h>
#include <asm/tlbflush.h>
#include <asm/cputype.h>
#include <asm/procinfo.h>

extern struct smp_ops keystone_smp_ops;

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
	iotable_init(io_desc, sizeof(io_desc)/sizeof(struct map_desc));
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

#ifdef CONFIG_ARM_LPAE

phys_addr_t keystone_phys_offset  = KEYSTONE_LOW_PHYS_START;

extern struct proc_info_list *lookup_processor_type(unsigned int);

static void __init keystone_init_meminfo(void)
{
	unsigned long map_start, map_end;
	struct proc_info_list *procinfo;
	phys_addr_t mem_start, mem_end;
	pgd_t *pgd0, *pgdk;
	pud_t *pud0, *pudk;
	pmd_t *pmd0, *pmdk;
	phys_addr_t phys;
	pmdval_t pmdprot;
	int i;

	BUG_ON(meminfo.nr_banks < 1);

	mem_start = meminfo.bank[0].start;
	mem_end   = mem_start + meminfo.bank[0].size - 1;

	/* nothing to do if we are running out of the <32-bit space */
	if (mem_start >= KEYSTONE_LOW_PHYS_START &&
	    mem_end   <= KEYSTONE_LOW_PHYS_END)
		return;

	BUG_ON(mem_start < KEYSTONE_HIGH_PHYS_START ||
	       mem_end   > KEYSTONE_HIGH_PHYS_END);

	/* remap kernel code and data */
	map_start = init_mm.start_code;
	map_end   = init_mm.brk;

	/* get a handle on things -  */
	pgd0 = pgd_offset_k(0);
	pud0 = pud_offset(pgd0, 0);
	pmd0 = pmd_offset(pud0, 0);

	pgdk = pgd_offset_k(map_start);
	pudk = pud_offset(pgdk, map_start);
	pmdk = pmd_offset(pudk, map_start);

	procinfo = lookup_processor_type(read_cpuid_id());
	pmdprot  = procinfo->__cpu_mm_mmu_flags;

	/* set the phys offset, all pa/va operations now use this */
	keystone_phys_offset = KEYSTONE_HIGH_PHYS_START;

	/* remap level 1 table */
	for (i = 0; i < PTRS_PER_PGD; i++) {
		*pud0++ = __pud(__pa(pmd0) | PMD_TYPE_TABLE | L_PGD_SWAPPER);
		pmd0 += PTRS_PER_PMD;
	}

	/* remap pmds for kernel mapping */
	phys = __pa(map_start) & PMD_MASK;
	do {
		*pmdk++ = __pmd(phys | pmdprot);
		phys += PMD_SIZE;
	} while (phys < map_end);

	flush_cache_all();
	cpu_set_ttbr(0, __pa(pgd0));
	cpu_set_ttbr(1, __pa(pgd0) + TTBR1_OFFSET);
	local_flush_tlb_all();

	pr_err("relocated to high address space\n");
}

#else

static void __init keystone_init_meminfo(void)
{
	/* nothing to do here */
}

#endif

DT_MACHINE_START(KEYSTONE, "Keystone")
	smp_ops(keystone_smp_ops)
	.map_io		= keystone_map_io,
	.init_irq	= keystone_init_irq,
	.timer		= &keystone_timer,
	.handle_irq	= gic_handle_irq,
	.init_machine	= keystone_init,
	.dt_compat	= keystone_match,
	.init_meminfo	= keystone_init_meminfo,
#ifdef CONFIG_ZONE_DMA
	.dma_zone_size	= SZ_2G,
#endif
	.nr_irqs	= 480,
MACHINE_END
