/*
 * arch/arm/include/asm/runtime-patch.h
 * Note: this file should not be included by non-asm/.h files
 *
 * Copyright 2012 Texas Instruments, Inc.
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
#ifndef __ASM_ARM_RUNTIME_PATCH_H
#define __ASM_ARM_RUNTIME_PATCH_H

#include <linux/stringify.h>

#ifndef __ASSEMBLY__

#ifdef CONFIG_ARM_RUNTIME_PATCH

struct patch_info {
	void	*insn;
	u16	 type;
	u8	 insn_size;
	u8	 data_size;
	u32	 data[0];
};

#define patch_next(p)		((void *)(p) + sizeof(*(p)) + (p)->data_size)

#define PATCH_TYPE_MASK		0x00ff
#define PATCH_IMM8		0x0001

#define PATCH_EARLY		0x8000

#define patch_stub(type, code, patch_data, ...)				\
	__asm__("@ patch stub\n"					\
		"1:\n"							\
		code							\
		"2:\n"							\
		"	.pushsection .runtime.patch.table, \"a\"\n"	\
		"3:\n"							\
		"	.word 1b\n"					\
		"	.hword (" __stringify(type) ")\n"		\
		"	.byte (2b-1b)\n"				\
		"	.byte (5f-4f)\n"				\
		"4:\n"							\
		patch_data						\
		"	.align\n"					\
		"5:\n"							\
		"	.popsection\n"					\
		__VA_ARGS__)

#define early_patch_stub(type, code, patch_data, ...)			\
	__asm__("@ patch stub\n"					\
		"1:\n"							\
		"	b	6f\n"					\
		"2:\n"							\
		"	.pushsection .runtime.patch.table, \"a\"\n"	\
		"3:\n"							\
		"	.word 1b\n"					\
		"	.hword (" __stringify(type | PATCH_EARLY) ")\n"	\
		"	.byte (2b-1b)\n"				\
		"	.byte (5f-4f)\n"				\
		"4:\n"							\
		patch_data						\
		"	.align\n"					\
		"5:\n"							\
		"	.popsection\n"					\
		"	.pushsection .runtime.patch.code, \"ax\"\n"	\
		"6:\n"							\
		code							\
		"	b 2b\n"						\
		"	.popsection\n"					\
		__VA_ARGS__)

/* constant used to force encoding */
#define __IMM8		(0x81 << 24)

/*
 * patch_imm8() - init-time specialized binary operation (imm8 operand)
 *		  This effectively does: to = from "insn" sym,
 *		  where the value of sym is fixed at init-time, and is patched
 *		  in as an immediate operand.  This value must be
 *		  representible as an 8-bit quantity with an optional
 *		  rotation.
 *
 *		  The stub code produced by this variant is non-functional
 *		  prior to patching.  Use early_patch_imm8() if you need the
 *		  code to be functional early on in the init sequence.
 */
#define patch_imm8(insn, to, from, sym, offset)				\
	patch_stub(PATCH_IMM8,						\
		   /* code */						\
		   insn " %0, %1, %2\n",				\
		   /* patch_data */					\
		   ".long " __stringify(sym + offset) "\n"		\
		   insn " %0, %1, %2\n",				\
		   : "=r" (to)						\
		   : "r" (from), "I" (__IMM8), "m" (sym)		\
		   : "cc")

/*
 * patch_imm8_mov() - same as patch_imm8(), but for mov/mvn instructions
 */
#define patch_imm8_mov(insn, to, sym, offset)				\
	patch_stub(PATCH_IMM8,						\
		   /* code */						\
		   insn " %0, %1\n",					\
		   /* patch_data */					\
		   ".long " __stringify(sym + offset) "\n"		\
		   insn " %0, %1\n",					\
		   : "=r" (to)						\
		   : "I" (__IMM8), "m" (sym)				\
		   : "cc")

/*
 * early_patch_imm8() - early functional variant of patch_imm8() above.  The
 *			same restrictions on the constant apply here.  This
 *			version emits workable (albeit inefficient) code at
 *			compile-time, and therefore functions even prior to
 *			patch application.
 */
#define early_patch_imm8(insn, to, from, sym, offset)			\
	early_patch_stub(PATCH_IMM8,					\
			 /* code */					\
			 "ldr	%0, =" __stringify(sym + offset) "\n"	\
			 "ldr	%0, [%0]\n"				\
			 insn " %0, %1, %0\n",				\
			 /* patch_data */				\
			 ".long " __stringify(sym + offset) "\n"	\
			 insn " %0, %1, %2\n",				\
			 : "=&r" (to)					\
			 : "r" (from), "I" (__IMM8), "m" (sym)		\
			 : "cc")

#define early_patch_imm8_mov(insn, to, sym, offset)			\
	early_patch_stub(PATCH_IMM8,					\
			 /* code */					\
			 "ldr	%0, =" __stringify(sym + offset) "\n"	\
			 "ldr	%0, [%0]\n"				\
			 insn " %0, %0\n",				\
			 /* patch_data */				\
			 ".long " __stringify(sym + offset) "\n"	\
			 insn " %0, %1\n",				\
			 : "=&r" (to)					\
			 : "I" (__IMM8), "m" (sym)			\
			 : "cc")

int runtime_patch(const void *table, unsigned size);
void runtime_patch_kernel(void);

#else

static inline int runtime_patch(const void *table, unsigned size)
{
	return 0;
}

static inline void runtime_patch_kernel(void)
{
}

#endif /* CONFIG_ARM_RUNTIME_PATCH */

#endif /* __ASSEMBLY__ */

#endif /* __ASM_ARM_RUNTIME_PATCH_H */
