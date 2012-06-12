/*
 *  linux/arch/arm/include/asm/smp_ops.h
 *
 *  Copyright (C) 2011 ARM Ltd.
 *  All Rights Reserved
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef __ASM_ARM_SMP_OPS_H
#define __ASM_ARM_SMP_OPS_H

struct task_struct;

struct smp_init_ops {
#ifdef CONFIG_SMP
	/*
	 * Setup the set of possible CPUs (via set_cpu_possible)
	 */
	void (*smp_init_cpus)(void);
	/*
	 * Initialize cpu_possible map, and enable coherency
	 */
	void (*smp_prepare_cpus)(unsigned int max_cpus);
#endif
};

struct smp_secondary_ops {
#ifdef CONFIG_SMP
	/*
	 * Perform platform specific initialisation of the specified CPU.
	 */
	void (*smp_secondary_init)(unsigned int cpu);
	/*
	 * Boot a secondary CPU, and assign it the specified idle task.
	 * This also gives us the initial stack to use for this CPU.
	 */
	int  (*smp_boot_secondary)(unsigned int cpu, struct task_struct *idle);
#endif
};

struct smp_hotplug_ops {
#ifdef CONFIG_HOTPLUG_CPU
	int  (*cpu_kill)(unsigned int cpu);
	void (*cpu_die)(unsigned int cpu);
	int  (*cpu_disable)(unsigned int cpu);
#endif
};

struct smp_ops {
	struct smp_init_ops		init_ops;
	struct smp_secondary_ops	secondary_ops;
	struct smp_hotplug_ops		hotplug_ops;
};

#ifdef CONFIG_SMP
#define smp_init_ops(prefix)		.init_ops = {			\
		.smp_init_cpus		= prefix##_smp_init_cpus,	\
		.smp_prepare_cpus	= prefix##_smp_prepare_cpus,	\
	},

#define smp_secondary_ops(prefix)	.secondary_ops = {		\
		.smp_secondary_init	= prefix##_secondary_init,	\
		.smp_boot_secondary	= prefix##_boot_secondary,	\
	},

extern void smp_ops_register(struct smp_ops *);

#else
#define smp_init_ops(prefix)		.init_ops = {},
#define smp_secondary_ops(prefix)	.secondary_ops = {},
#define smp_ops_register(a)		do {} while(0)
#endif

#ifdef CONFIG_HOTPLUG_CPU
#define smp_hotplug_ops(prefix)		.hotplug_ops = {	\
		.cpu_kill	= prefix##_cpu_kill,		\
		.cpu_die	= prefix##_cpu_die,		\
		.cpu_disable	= prefix##_cpu_disable,		\
	},
#else
#define smp_hotplug_ops(prefix)		.hotplug_ops = {},
#endif

#endif	/* __ASM_ARM_SMP_OPS_H */
