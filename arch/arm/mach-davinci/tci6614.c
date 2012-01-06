/*
 * Texas Instruments TCI6614 SoC Support
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
#include <linux/clk.h>
#include <linux/io.h>
#include <linux/irq.h>
#include <linux/err.h>
#include <linux/platform_device.h>

#include <asm/mach/map.h>

#include <mach/vmalloc.h>
#include <mach/common.h>
#include <mach/time.h>
#include <mach/cputype.h>
#include <mach/psc.h>
#include <mach/cp_intc.h>
#include <mach/irqs.h>
#include <mach/gpio.h>
#include <mach/hardware.h>
#include <mach/cp_intd.h>
#include <mach/tci6614.h>

#include "clock.h"
#include "mux.h"

/* Base addresses for on-chip devices */
#define TCI6614_TIMER7_BASE			0x02270000
#define TCI6614_TIMER8_BASE			0x02280000
#define TCI6614_PLL_BASE			0x02310000
#define TCI6614_GPIO_BASE			0x02320000
#define TCI6614_PSC_BASE			0x02350000
#define TCI6614_CPINTC_BASE			0x0260c000
#define TCI6614_CPINTD_BASE			0x02610000
#define TCI6614_BOOT_CFG_BASE			0x02620000
#define TCI6614_AINTC_BASE			0x48200000

#define TCI6614_BOOT_CFG_DEVSTAT		(TCI6614_BOOT_CFG_BASE + 0x20)
#define TCI6614_MAINPLL_CTL0			(TCI6614_BOOT_CFG_BASE + 0x328)
#define TCI6614_MAINPLL_CTL0			(TCI6614_BOOT_CFG_BASE + 0x328)
#define TCI6614_PLLCTRL_PLLM			(TCI6614_PLL_BASE + 0x0110)
#define TCI6614_DEFAULT_IN_CLK			50000000
#define MAINPLL_CTL0_PLLM_MASK			0x7F000
#define MAINPLL_CTL0_PLLM_SHIFT			6
#define MAINPLL_CTL0_PLLD_MASK			0x3f
#define PLLCTRL_PLLM_MASK			0x3f

/* PSC control registers */
static u32 psc_regs[] = { TCI6614_PSC_BASE };

/* Host map for interrupt controller */
static u32 intc_host_map[] = { 0x01010000, 0x01010101, -1 };

static unsigned long tci6614_ref_clk_recalc(struct clk *clk)
{
	void __iomem *devstat;
	static unsigned long freq[] = { 50000000, 66666667, 80000000,
					100000000, 156250000, 250000000,
					312500000, 122880000 };
	int sel;

	devstat = ioremap(TCI6614_BOOT_CFG_DEVSTAT, 4);
	if (WARN_ON(!devstat))
		return TCI6614_DEFAULT_IN_CLK;
	sel = (__raw_readl(devstat) >> 11) & 0x7;

	iounmap(devstat);
	return freq[sel];
}

static unsigned long tci6614_main_pll_clk_recalc(struct clk *clk)
{
	unsigned long rate = 0;
	u32  pllm, plld, postdiv = 2, val;
	void __iomem *main_pll_ctl0, *pllctrl_pllm;

	main_pll_ctl0 = ioremap(TCI6614_MAINPLL_CTL0, 4);
	if (WARN_ON(!main_pll_ctl0))
		return rate;

	pllctrl_pllm = ioremap(TCI6614_PLLCTRL_PLLM, 4);
	if (WARN_ON(!pllctrl_pllm))
		return rate;

	/* get bit0-5 of PLLM from PLLM PLL control register */
	val = (__raw_readl(pllctrl_pllm) & PLLCTRL_PLLM_MASK);
	pllm = (val & PLLCTRL_PLLM_MASK);

	/* bit6-12 of PLLM is in Main PLL control register */
	val = __raw_readl(main_pll_ctl0);
	pllm |= ((val & MAINPLL_CTL0_PLLM_MASK) >> MAINPLL_CTL0_PLLM_SHIFT);
	plld = (val & MAINPLL_CTL0_PLLD_MASK);

	rate = clk->parent->rate;

	rate /= (plld + 1);
	rate = (rate * (pllm + 1));
	rate /= postdiv;

	printk(KERN_NOTICE "main_pll_clk rate is %ld, postdiv = %d, pllm = %d, plld = %d\n",
			rate, postdiv, pllm, plld);
	iounmap(main_pll_ctl0);
	iounmap(pllctrl_pllm);
	return rate;
}

static struct clk ref_clk = {
	.name		= "ref_clk",
	.recalc		= tci6614_ref_clk_recalc,
	.flags		= ALWAYS_ENABLED,
};

static struct pll_data main_pll_data = {
	.num		= 1,
	.div_ratio_mask = 0xff,
	.phys_base	= TCI6614_PLL_BASE,
	.flags		= PLL_HAS_PREDIV | PLL_HAS_POSTDIV,
};

static struct clk main_pll = {
	.name		= "main_pll",
	.parent		= &ref_clk,
	.recalc		= tci6614_main_pll_clk_recalc,
	.pll_data	= &main_pll_data,
	.flags		= CLK_PLL | ALWAYS_ENABLED,
};

#define define_pll_div_clk(__pll, __div, __name)		\
	static struct clk __name = {				\
		.name		= #__name,			\
		.parent		= &__pll,			\
		.flags		= CLK_PLL | ALWAYS_ENABLED,	\
		.div_reg	= PLLDIV##__div,		\
	}

define_pll_div_clk(main_pll,  1, main_div_chip_clk1);
define_pll_div_clk(main_pll,  2, main_div_gem_trace_clk);
define_pll_div_clk(main_pll,  3, main_div_chip_clk2);
define_pll_div_clk(main_pll,  4, main_div_chip_clk3);
define_pll_div_clk(main_pll,  5, main_div_stm_clk);
define_pll_div_clk(main_pll,  6, main_div_emif_ptv_clk);
define_pll_div_clk(main_pll,  7, main_div_chip_clk6);
define_pll_div_clk(main_pll,  8, main_div_slowsys_clk);
define_pll_div_clk(main_pll,  9, main_div_chip_smreflex_clk);
define_pll_div_clk(main_pll, 10, main_div_chip_clk3_srio);
define_pll_div_clk(main_pll, 11, main_div_psc_clk6);
define_pll_div_clk(main_pll, 12, main_div_chip_dftclk4);
define_pll_div_clk(main_pll, 13, main_div_chip_dftclk8);

#define __lpsc_clk(cname, _parent, mod, flg, dom)	\
	static struct clk clk_##cname = {		\
		.name		= #cname,		\
		.parent		= &_parent,		\
		.lpsc		= TCI6614_LPSC_##mod,	\
		.flags		= flg,			\
		.domain		= TCI6614_PD_##dom,	\
	}

#define lpsc_clk_enabled(cname, parent, mod)		\
	__lpsc_clk(cname, parent, mod, ALWAYS_ENABLED, ALWAYSON)

#define lpsc_clk(cname, parent, mod, dom)		\
	__lpsc_clk(cname, parent, mod, 0, dom)


/* Alawys on domains */
lpsc_clk_enabled(modrst0,		main_div_chip_clk6, MODRST0);
lpsc_clk_enabled(src3_pwr,		main_div_chip_smreflex_clk, SRC3_PWR);
lpsc_clk_enabled(emif4f,		main_div_chip_clk1, EMIF4F);
lpsc_clk_enabled(monza_rst_ctrl,	main_div_chip_clk1, MONZA_RST_CTRL);

/* There are 2 more clocks coming to some of the modules below and only
 * one of the clock is mentioned as parent clock. Assume they are
 * automatically enabled by gpsc
 */
lpsc_clk_enabled(timer0,		clk_modrst0, MODRST0);
lpsc_clk_enabled(timer1,		clk_modrst0, MODRST0);
lpsc_clk_enabled(uart0,			clk_modrst0, MODRST0);
lpsc_clk_enabled(uart1,			clk_modrst0, MODRST0);
lpsc_clk_enabled(aemif,			clk_modrst0, MODRST0);
lpsc_clk_enabled(usim,			clk_modrst0, MODRST0);
lpsc_clk_enabled(i2c,			clk_modrst0, MODRST0);
lpsc_clk_enabled(spi,			clk_modrst0, MODRST0);
lpsc_clk_enabled(gpio,			clk_modrst0, MODRST0);
lpsc_clk_enabled(key_mgr,		clk_modrst0, MODRST0);

/* SW controlled domains */
lpsc_clk(vusr,				main_div_chip_clk2, VUSR, ALWAYSON);
lpsc_clk(vcp2_a,			main_div_chip_clk3, VCP2_A, ALWAYSON);
lpsc_clk(debugss_trc,			main_div_chip_clk3, DEBUGSS_TRC, DEBUG_TRC);
lpsc_clk(tetb_trc,			main_div_chip_clk3, TETB_TRC, DEBUG_TRC);
lpsc_clk(pktproc,			main_div_chip_clk3, PKTPROC, PASS);
lpsc_clk(cpgmac,			main_div_chip_clk3, CPGMAC, PASS);
lpsc_clk(crypto,			main_div_chip_clk1, CRYPTO, PASS);
lpsc_clk(pciex,				main_div_chip_clk2, PCIEX, PCIEX);
lpsc_clk(srio,				main_div_chip_clk3_srio, SRIO, SRIO);
lpsc_clk(bcp,				main_div_chip_clk3, BCP,  BCP);
lpsc_clk(msmcsram,			main_div_chip_clk2, MSMCSRAM, MSMCSRAM);
lpsc_clk(rac,				main_div_chip_clk1, RAC, RAC_TAC);
lpsc_clk(tac,				main_div_chip_clk3, TAC, RAC_TAC);
lpsc_clk(fftc,				main_div_chip_clk3, FFTC, FFTC);
lpsc_clk(aif2,				main_div_chip_clk3, AIF2, AIF2);
lpsc_clk(tcp3d,				main_div_chip_clk2, TCP3D, TCP3D);
lpsc_clk(vcp2_b,			main_div_chip_clk3, VCP2_B, VCP_BCD);
lpsc_clk(vcp2_c,			main_div_chip_clk3, VCP2_C, VCP_BCD);
lpsc_clk(vcp2_d,			main_div_chip_clk3, VCP2_D, VCP_BCD);
lpsc_clk(gem0,				main_div_chip_clk1, GEM0, GEM0);
lpsc_clk(gem1,				main_div_chip_clk1, GEM1, GEM1);
lpsc_clk(rsax2_1,			main_div_chip_clk1, RSAX2_1, GEM1);
lpsc_clk(gem2,				main_div_chip_clk1, GEM2, GEM2);
lpsc_clk(rsax2_0,			main_div_chip_clk1, RSAX2_0, GEM2);
lpsc_clk(gem3,				main_div_chip_clk1, GEM3, GEM3);
lpsc_clk(tcp3d_b,			main_div_chip_clk2, TCP3D_B, TCP3D_B);

static struct clk_lookup clks[] = {
	CLK(NULL, "ref_clk",			&ref_clk),
	CLK(NULL, "main_pll",			&main_pll),

	CLK(NULL, "main_div_chip_clk1",		&main_div_chip_clk1),
	CLK(NULL, "main_div_gem_trace_clk",	&main_div_gem_trace_clk),
	CLK(NULL, "main_div_chip_clk2",		&main_div_chip_clk2),
	CLK(NULL, "main_div_chip_clk3",		&main_div_chip_clk3),
	CLK(NULL, "main_div_stm_clk",		&main_div_stm_clk),
	CLK(NULL, "main_div_emif_ptv_clk",	&main_div_emif_ptv_clk),
	CLK(NULL, "main_div_chip_clk6",		&main_div_chip_clk6),
	CLK(NULL, "main_div_slowsys_clk",	&main_div_slowsys_clk),
	CLK(NULL, "main_div_chip_smreflex_clk",	&main_div_chip_smreflex_clk),
	CLK(NULL, "main_div_chip_clk3_srio",	&main_div_chip_clk3_srio),
	CLK(NULL, "main_div_psc_clk6",		&main_div_psc_clk6),
	CLK(NULL, "main_div_chip_dftclk4",	&main_div_chip_dftclk4),
	CLK(NULL, "main_div_chip_dftclk8",	&main_div_chip_dftclk8),

	CLK(NULL,		"clk_modrst0",		&clk_modrst0),
	CLK(NULL,		"clk_src3_pwr",		&clk_src3_pwr),
	CLK(NULL,		"clk_emif4f",		&clk_emif4f),
	CLK(NULL,		"clk_vusr",		&clk_vusr),
	CLK(NULL,		"clk_vcp2_a",		&clk_vcp2_a),
	CLK(NULL,		"clk_debugss_trc",	&clk_debugss_trc),
	CLK(NULL,		"clk_tetb_trc",		&clk_tetb_trc),
	CLK(NULL,		"clk_pktproc",		&clk_pktproc),
	CLK(NULL,		"clk_cpgmac",		&clk_cpgmac),
	CLK(NULL,		"clk_crypto",		&clk_crypto),
	CLK(NULL,		"clk_pciex",		&clk_pciex),
	CLK(NULL,		"clk_srio",		&clk_srio),
	CLK(NULL,		"clk_bcp",		&clk_bcp),
	CLK(NULL,		"clk_monza_rst_ctrl",	&clk_monza_rst_ctrl),
	CLK(NULL,		"clk_msmcsram",		&clk_msmcsram),
	CLK(NULL,		"clk_rac",		&clk_rac),
	CLK(NULL,		"clk_tac",		&clk_tac),
	CLK(NULL,		"clk_fftc",		&clk_fftc),
	CLK(NULL,		"clk_aif2",		&clk_aif2),
	CLK(NULL,		"clk_tcp3d",		&clk_tcp3d),
	CLK(NULL,		"clk_vcp2_b",		&clk_vcp2_b),
	CLK(NULL,		"clk_vcp2_c",		&clk_vcp2_c),
	CLK(NULL,		"clk_vcp2_d",		&clk_vcp2_d),
	CLK("keystone-rproc.0",	NULL,                   &clk_gem0),
	CLK("keystone-rproc.1",	NULL,                   &clk_gem1),
	CLK("keystone-rproc.2",	NULL,                   &clk_gem2),
	CLK("keystone-rproc.3",	NULL,                   &clk_gem3),
	CLK(NULL,		"clk_rsax2_1",		&clk_rsax2_1),
	CLK(NULL,		"clk_rsax2_0",		&clk_rsax2_0),
	CLK(NULL,		"clk_tcp3d_b",		&clk_tcp3d_b),

	CLK(NULL,		"timer0",		&clk_timer0),
	CLK("watchdog",		NULL,			&clk_timer1),
	CLK(NULL,		"uart0",		&clk_uart0),
	CLK(NULL,		"uart1",		&clk_uart1),
	CLK(NULL,		"aemif",		&clk_aemif),
	CLK(NULL,		"usim",			&clk_usim),
	CLK("i2c_davinci.1",	NULL,			&clk_i2c),
	CLK("spi_davinci.0",	NULL,			&clk_spi),
	CLK(NULL,		"gpio",			&clk_gpio),
	CLK(NULL,		"key_mgr",		&clk_key_mgr),

	CLK(NULL,		NULL,			NULL),
};

/* FIQ are pri 0-1; otherwise 2-7, with 7 lowest priority */
static u8 aintc_irq_prios[TCI6614_N_AINTC_IRQ] = {
	/* fill in default priority 0 */
	[0 ... (TCI6614_N_AINTC_IRQ - 1)]	= 0,
	/* now override as needed, e.g. [xxx] = 5 */
};


/* Contents of JTAG ID register used to identify exact cpu type */
static struct davinci_id ids[] = {
	{
		.variant	= 0x0,
		.part_no	= 0xb962,
		.manufacturer	= 0x017,
		.cpu_id		= DAVINCI_CPU_ID_TCI6614,
		.name		= "tci6614 rev 1.0",
	},
};

static struct davinci_timer_instance timer_instance[2] = {
	{
		.base		= TCI6614_TIMER7_BASE,
		.bottom_irq	= IRQ_TCI6614_TINT7L,
		.top_irq	= IRQ_TCI6614_TINT7H,
	},
	{
		.base		= TCI6614_TIMER8_BASE,
		.bottom_irq	= IRQ_TCI6614_TINT8L,
		.top_irq	= IRQ_TCI6614_TINT8H,
	},
};

static struct davinci_timer_info timer_info = {
	.timers		= timer_instance,
	.clockevent_id	= T0_BOT,
	.clocksource_id	= T0_TOP,
};

/*
 * TCI6614 platforms do not use the static mappings from Davinci
 * IO_PHYS/IO_VIRT. This SOC's interesting MMRs are at different addresses,
 * and changing IO_PHYS would break away from existing Davinci SOCs.
 *
 * The primary impact of the current model is that IO_ADDRESS() is not to be
 * used to map registers on TCI6614.
 */
static struct map_desc io_desc[] = {
	{
		.virtual	= IO_VIRT,
		.pfn		= __phys_to_pfn(TCI6614_IO_BASE),
		.length		= IO_SIZE,
		.type		= MT_DEVICE
	},
};

static u32 aintc_to_intd_map[TCI6614_N_AINTC_IRQ] = {
	[7]  = IRQ_TCI6614_QM_INT_HIGH_1,
	[8]  = IRQ_TCI6614_IPC_H,
	[9]  = IRQ_TCI6614_QM_INT_HIGH_0,
	[11] = IRQ_TCI6614_QM_INT_HIGH_2,
	[12] = IRQ_TCI6614_QM_INT_HIGH_3,
	[13] = IRQ_TCI6614_QM_INT_HIGH_4,
	[14] = IRQ_TCI6614_QM_INT_HIGH_5,
	[15] = IRQ_TCI6614_QM_INT_HIGH_6,
	[16] = IRQ_TCI6614_QM_INT_HIGH_7,
	[17] = IRQ_TCI6614_QM_INT_HIGH_8,
	[18] = IRQ_TCI6614_QM_INT_HIGH_9,
	[19] = IRQ_TCI6614_QM_INT_HIGH_10,
	[20] = IRQ_TCI6614_QM_INT_HIGH_11,
	[21] = IRQ_TCI6614_QM_INT_HIGH_12,
	[22] = IRQ_TCI6614_QM_INT_HIGH_13,
	[23] = IRQ_TCI6614_QM_INT_HIGH_14,
	[24] = IRQ_TCI6614_QM_INT_HIGH_15,
	[25] = IRQ_TCI6614_QM_INT_HIGH_16,
	[26] = IRQ_TCI6614_QM_INT_HIGH_17,
	[27] = IRQ_TCI6614_QM_INT_HIGH_18,
	[28] = IRQ_TCI6614_QM_INT_HIGH_19,
	[29] = IRQ_TCI6614_QM_INT_HIGH_20,
	[30] = IRQ_TCI6614_QM_INT_HIGH_21,
	[31] = IRQ_TCI6614_QM_INT_HIGH_22,
	[32] = IRQ_TCI6614_QM_INT_HIGH_23,
	[33] = IRQ_TCI6614_QM_INT_HIGH_24,
	[34] = IRQ_TCI6614_QM_INT_HIGH_25,
	[35] = IRQ_TCI6614_QM_INT_HIGH_26,
	[36] = IRQ_TCI6614_QM_INT_HIGH_27,
	[37] = IRQ_TCI6614_QM_INT_HIGH_28,
	[38] = IRQ_TCI6614_QM_INT_HIGH_29,
	[39] = IRQ_TCI6614_QM_INT_HIGH_30,
	[40] = IRQ_TCI6614_QM_INT_HIGH_31,
	[51] = IRQ_TCI6614_VUSR_INT,
	[52] = IRQ_TCI6614_SEMERR7,
	[53] = IRQ_TCI6614_SEMINT7,
	[54] = IRQ_TCI6614_SRIO_INTDST20,
	[55] = IRQ_TCI6614_SRIO_INTDST21,
	[56] = IRQ_TCI6614_SRIO_INTDST22,
	[57] = IRQ_TCI6614_SRIO_INTDST23,
	[58] = IRQ_TCI6614_TINT4L,
	[59] = IRQ_TCI6614_TINT4H,
	[60] = IRQ_TCI6614_TINT5L,
	[61] = IRQ_TCI6614_TINT5H,
	[62] = IRQ_TCI6614_TINT6L,
	[63] = IRQ_TCI6614_TINT6H,
	[64] = IRQ_TCI6614_TINT7L,
	[65] = IRQ_TCI6614_TINT7H,
	[66] = IRQ_TCI6614_PCIE_ERR_INT,
	[67] = IRQ_TCI6614_PCIE_PM_INT,
	[68] = IRQ_TCI6614_PCIE_LEGACY_INTA,
	[69] = IRQ_TCI6614_PCIE_LEGACY_INTB,
	[70] = IRQ_TCI6614_PCIE_LEGACY_INTC,
	[71] = IRQ_TCI6614_PCIE_LEGACY_INTD,
	[72] = IRQ_TCI6614_PCIE_MSI_INT4,
	[73] = IRQ_TCI6614_PCIE_MSI_INT5,
	[74] = IRQ_TCI6614_PCIE_MSI_INT6,
	[75] = IRQ_TCI6614_PCIE_MSI_INT7,
	[76] = IRQ_TCI6614_TINT8L,
	[77] = IRQ_TCI6614_TINT8H,
	[78] = IRQ_TCI6614_TINT9L,
	[79] = IRQ_TCI6614_TINT9H,
	[80] = IRQ_TCI6614_TINT10L,
	[81] = IRQ_TCI6614_TINT10H,
	[82] = IRQ_TCI6614_TINT11L,
	[83] = IRQ_TCI6614_TINT11H,
	[84] = IRQ_TCI6614_TCP3D_A_REVT0,
	[85] = IRQ_TCI6614_TCP3D_A_REVT1,
	[86] = IRQ_TCI6614_TCP3D_B_REVT0,
	[87] = IRQ_TCI6614_TCP3D_B_REVT1,
	[88] = IRQ_TCI6614_CPU_3_1_TPCC_INT2,
	[89] = IRQ_TCI6614_CPU_3_1_TPCC_INT6,
	[90] = IRQ_TCI6614_CPU_3_2_TPCC_INT2,
	[91] = IRQ_TCI6614_CPU_3_2_TPCC_INT6,
	[92] = IRQ_TCI6614_CPU_2_TPCC_INT2,
	[93] = IRQ_TCI6614_CPU_2_TPCC_INT6,
	[127] = IRQ_TCI6614_WATCH_DOG_NMI,
};

static struct davinci_soc_info tci6614_soc_info = {
	.io_desc		= io_desc,
	.io_desc_num		= ARRAY_SIZE(io_desc),
	.ids			= ids,
	.ids_num		= ARRAY_SIZE(ids),
	.jtag_id_reg		= TCI6614_BOOT_CFG_BASE + 0x18,
	.cpu_clks		= clks,
	.psc_bases		= psc_regs,
	.psc_bases_num		= ARRAY_SIZE(psc_regs),
	.intc_type		= DAVINCI_INTC_TYPE_OMAP_AINTC,
	.intc_base		= TCI6614_AINTC_BASE,
	.intc_irq_prios		= aintc_irq_prios,
	.intc_irq_num		= TCI6614_N_AINTC_IRQ,
	.intc_host_map		= intc_host_map,
	.intd_base		= TCI6614_CPINTD_BASE,
	.intd_irq_base		= TCI6614_INTD_IRQ_BASE,
	.intc_to_intd_map	= aintc_to_intd_map,
	.intd_irq_num		= TCI6614_N_INTD_IRQ,
	.gpio_base		= TCI6614_GPIO_BASE,
	.gpio_type		= GPIO_TYPE_DAVINCI,
	.gpio_num		= TCI6614_N_GPIO,
	.gpio_unbanked		= TCI6614_N_GPIO,
	.gpio_irq		= IRQ_TCI6614_GPINT0,
	.timer_info		= &timer_info,
	.serial_dev		= &tci6614_serial_device,
	.reset_device		= &tci6614_wdt_device,
};

void __init tci6614_init(void)
{
	davinci_common_init(&tci6614_soc_info);
}

static void __iomem *cpintc_base;
static bool debug_cpintc;

#define CPINTC_REVISION			0x000
#define CPINTC_GLOBAL_ENABLE		0x010
#define CPINTC_STATUS_IDX_SET		0x020
#define CPINTC_STATUS_IDX_CLEAR		0x024
#define CPINTC_ENABLE_IDX_SET		0x028
#define CPINTC_ENABLE_IDX_CLEAR		0x02c
#define CPINTC_HOST_ENABLE_SET		0x034
#define CPINTC_HOST_ENABLE_CLEAR	0x038
#define CPINTC_GLOBAL_INDEX		0x080
#define CPINTC_STATUS_SET		0x200
#define CPINTC_STATUS_CLEAR		0x280
#define CPINTC_ENABLE_SET		0x300
#define CPINTC_ENABLE_CLEAR		0x380
#define CPINTC_CHANNEL_MAP		0x400
#define CPINTC_HOST_MAP			0x800

#define CPINTC_CHANS			8
#define CPINTC_REG(irq)		(BIT_WORD(irq) << 2)
#define CPINTC_BIT(irq)		(BIT_MASK(irq))

static inline u32 cpintc_readl(int offset)
{
	u32 value = __raw_readl(cpintc_base + offset);
	if (debug_cpintc)
		printk(KERN_NOTICE \
			 "cpintc: read offset %x = %x\n", offset, value);
	return value;
}

static inline void cpintc_writel(u32 value, int offset)
{
	if (debug_cpintc)
		printk(KERN_NOTICE \
			"cpintc: write offset %x = %x\n", offset, value);
	__raw_writel(value, cpintc_base + offset);
}

static void cpintc_mask_irq(struct irq_data *d)
{
	unsigned int irq = d->irq;

	irq -= TCI6614_CPINTC_IRQ_BASE;
	if (debug_cpintc)
		printk(KERN_NOTICE "cpintc: mask %d\n", irq);
	cpintc_writel(irq, CPINTC_ENABLE_IDX_CLEAR);
}

void cpintc_unmask_irq(struct irq_data *d)
{
	unsigned int irq = d->irq;

	irq -= TCI6614_CPINTC_IRQ_BASE;
	if (debug_cpintc)
		printk(KERN_NOTICE "cpintc: unmask %d\n", irq);
	cpintc_writel(irq, CPINTC_ENABLE_IDX_SET);
}

void cpintc_ack_irq(struct irq_data *d)
{
	unsigned int irq = d->irq;

	irq -= TCI6614_CPINTC_IRQ_BASE;
	if (debug_cpintc)
		printk(KERN_NOTICE "cpintc: ack %d\n", irq);
	cpintc_writel(irq, CPINTC_STATUS_IDX_CLEAR);
}

bool cpintc_irq_pending(unsigned int irq)
{
	bool enabled, pending;

	irq -= TCI6614_CPINTC_IRQ_BASE;
	enabled = (cpintc_readl(CPINTC_ENABLE_SET + CPINTC_REG(irq)) &
			CPINTC_BIT(irq)) ? true : false;
	pending = (cpintc_readl(CPINTC_STATUS_SET + CPINTC_REG(irq)) &
			CPINTC_BIT(irq)) ? true : false;
	if (debug_cpintc) {
		printk(KERN_NOTICE "cpintc: %d %s, %s\n", irq,
		       pending ? "pending" : "not pending",
		       enabled ? "enabled" : "not enabled");
	}
	return pending;
}

struct irq_chip cpintc_chip = {
	.name	= "CPINTC",
	.irq_ack	= cpintc_ack_irq,
	.irq_mask	= cpintc_mask_irq,
	.irq_unmask	= cpintc_unmask_irq,
};

void cpintc_irq_handler(unsigned irq, struct irq_desc *desc)
{
	struct irq_chip *chip = irq_desc_get_chip(desc);
	struct irq_data *irq_data = irq_desc_get_irq_data(desc);
	int cpintc_irq;

	if (debug_cpintc)
		printk(KERN_NOTICE "cpintc: start irq %d\n", irq);

	chip->irq_mask(irq_data);
	chip->irq_ack(irq_data);

	cpintc_irq = cpintc_readl(CPINTC_GLOBAL_INDEX);
	if (cpintc_irq_pending(TCI6614_CPINTC_IRQ(cpintc_irq)))
		generic_handle_irq(TCI6614_CPINTC_IRQ(cpintc_irq));
	chip->irq_unmask(irq_data);

	if (debug_cpintc)
		printk(KERN_NOTICE "cpintc: end irq %d\n", irq);
}

void __init cpintc_init(void)
{
	int i;
	unsigned long rev;

	cpintc_base = ioremap(TCI6614_CPINTC_BASE, SZ_8K);
	BUG_ON(!cpintc_base);

	rev = cpintc_readl(CPINTC_REVISION);
	printk(KERN_INFO "IRQ: cpintc version %ld.%ld at %p\n",
	       (rev >> 8) & 0x3, rev & 0x3f, cpintc_base);

	/* map all irqs to channel 0 */
	for (i = 0; i < DIV_ROUND_UP(TCI6614_N_CPINTC_IRQ, 4); i++)
		cpintc_writel(0, CPINTC_CHANNEL_MAP + (4 * i));

	/* map all channels to host 0 */
	for (i = 0; i < DIV_ROUND_UP(CPINTC_CHANS, 4); i++)
		cpintc_writel(0, CPINTC_HOST_MAP + (4 * i));

	cpintc_writel(0, CPINTC_HOST_ENABLE_SET);

	for (i = 0; i < TCI6614_N_CPINTC_IRQ; i++) {
		irq_set_chip_and_handler(TCI6614_CPINTC_IRQ(i),
					 &cpintc_chip, handle_edge_irq);
		set_irq_flags(TCI6614_CPINTC_IRQ(i), IRQF_VALID | IRQF_PROBE);
	}

	for (i = IRQ_TCI6614_INTC3_OUT0; i <= IRQ_TCI6614_INTC3_OUT32; i++)
		irq_set_chained_handler(i, cpintc_irq_handler);

	cpintc_writel(1, CPINTC_GLOBAL_ENABLE);
}

void __init tci6614_intc_init(void)
{
	omap_aintc_init();
	cp_intd_init();
	cpintc_init();
}
