/*
 * Low-level IRQ helper macros for TI DaVinci-based platforms
 *
 * Author: Kevin Hilman, MontaVista Software, Inc. <source@mvista.com>
 *
 * 2007 (c) MontaVista Software, Inc. This file is licensed under
 * the terms of the GNU General Public License version 2. This program
 * is licensed "as is" without any warranty of any kind, whether express
 * or implied.
 */
#include <mach/irqs.h>

#define INTCPS_SIR_IRQ_OFFSET	0x0040
#define	ACTIVEIRQ_MASK		0x7f

#if ((defined(CONFIG_AINTC) && defined(CONFIG_CP_INTC))		||	\
     (defined(CONFIG_CP_INTC) && defined(CONFIG_OMAP_AINTC))	||	\
     (defined(CONFIG_OMAP_AINTC) && defined(CONFIG_AINTC)))
# define MULTI
#endif

		.macro  get_irqnr_preamble, base, tmp
		ldr \base, =davinci_intc_base
		ldr \base, [\base]
		.endm

		.macro	get_irqnr_and_base, irqnr, irqstat, base, tmp
#if defined(MULTI)
		ldr \tmp, =davinci_intc_type
		ldr \tmp, [\tmp]
		cmp \tmp, #DAVINCI_INTC_TYPE_CP_INTC
		beq 1000f
		cmp \tmp, #DAVINCI_INTC_TYPE_OMAP_AINTC
		beq 2000f
#endif
#if defined(CONFIG_AINTC)
		ldr \tmp, [\base, #0x14]
		movs \tmp, \tmp, lsr #2
		sub \irqnr, \tmp, #1
		b 9999f
#endif
#if defined(CONFIG_CP_INTC)
1000:		ldr \irqnr, [\base, #0x80] /* get irq number */
		and \irqnr, \irqnr, #0xff  /* irq is in bits 0-9 */
		mov \tmp, \irqnr, lsr #3
		and \tmp, \tmp, #0xfc
		add \tmp, \tmp, #0x280 /* get the register offset */
		ldr \irqstat, [\base, \tmp] /* get the intc status */
		cmp \irqstat, #0x0
		b 9999f
#endif
#if defined(CONFIG_OMAP_AINTC)
2000:		ldr	\irqnr, [\base, #0x98] /* IRQ pending reg 1 */
		cmp	\irqnr, #0x0
		bne	2001f
		ldr	\irqnr, [\base, #0xb8] /* IRQ pending reg 2 */
		cmp	\irqnr, #0x0
		bne	2001f
		ldr	\irqnr, [\base, #0xd8] /* IRQ pending reg 3 */
		cmp	\irqnr, #0x0
		bne	2001f
		ldr	\irqnr, [\base, #0xf8] /* IRQ pending reg 4 */
		cmp	\irqnr, #0x0
2001:
		ldrne	\irqnr, [\base, #INTCPS_SIR_IRQ_OFFSET]
		and	\irqnr, \irqnr, #ACTIVEIRQ_MASK
#endif
9999:
		.endm
