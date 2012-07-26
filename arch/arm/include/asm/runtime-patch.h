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

#define RUNTIME_PATCH_TYPE_IMM8		0x0001

#define runtime_patch_stub(type, code, patch_data, ...)			\
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

#define early_runtime_patch_stub(type, code, pad, patch_data, ...)	\
	__asm__("@ patch stub\n"					\
		"1:\n"							\
		"	b	6f\n"					\
		"	.fill	" __stringify(pad) ", 1, 0\n"		\
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
		"	.pushsection .runtime.patch.text, \"ax\"\n"	\
		"6:\n"							\
		code							\
		"	b 2b\n"						\
		"	.ltorg\n"					\
		"	.popsection\n"					\
		__VA_ARGS__)

/* constant used to force encoding */
#define __IMM8_CONST_DUMMY		(0x81 << 24)

/*
 * runtime_patch_imm8() - init-time specialized binary operation (imm8 operand)
 *		  This effectively does: to = from "insn" sym,
 *		  where the value of sym is fixed at init-time, and is patched
 *		  in as an immediate operand.  This value must be
 *		  representible as an 8-bit quantity with an optional
 *		  rotation.
 *
 *		  The stub code produced by this variant is non-functional
 *		  prior to patching.  Use early_runtime_patch_imm8() if you
 *		  need the code to be functional early on in the init sequence.
 */
#define runtime_patch_imm8(_insn, _to, _from, _sym)			\
	runtime_patch_stub(						\
		/* type */						\
			RUNTIME_PATCH_TYPE_IMM8,			\
		/* code */						\
			_insn "	%[to], %[from], %[imm]\n",		\
		/* patch_data */					\
			".long " __stringify(_sym) "\n"			\
			_insn "	%[to], %[from], %[imm]\n",		\
		/* operands */						\
			: [to]	 "=r" (_to)				\
			: [from] "r"  (_from),				\
			  [imm]	 "I"  (__IMM8_CONST_DUMMY),		\
				 "i"  (&(_sym)))

/*
 * runtime_patch_imm8_mov() - same as runtime_patch_imm8(), but for mov/mvn
 *			instructions
 */
#define runtime_patch_imm8_mov(_insn, _to, _sym)			\
	runtime_patch_stub(						\
		/* type */						\
			RUNTIME_PATCH_TYPE_IMM8,			\
		/* code */						\
			_insn "	%[to], %[imm]\n",			\
		/* patch_data */					\
			".long " __stringify(_sym) "\n"			\
			_insn "	%[to], %[imm]\n",			\
		/* operands */						\
			: [to]	"=r" (_to)				\
			: [imm]	"I"  (__IMM8_CONST_DUMMY),		\
				"i"  (&(_sym)))

/*
 * early_runtime_patch_imm8() - early functional variant of runtime_patch_imm8()
 *			above.  The same restrictions on the constant apply
 *			here.  This version emits workable (albeit inefficient)
 *			code at compile-time, and therefore functions even prior
 *			to patch application.
 */
#define early_runtime_patch_imm8(_insn, _to, _from, _sym)		\
do {									\
	unsigned long __tmp;						\
	early_runtime_patch_stub(					\
		/* type */						\
			RUNTIME_PATCH_TYPE_IMM8,			\
		/* code */						\
			"ldr	%[tmp], =" __stringify(_sym) "\n"	\
			"ldr	%[tmp], [%[tmp]]\n"			\
			_insn "	%[to], %[from], %[tmp]\n",		\
		/* pad */						\
			0,						\
		/* patch_data */					\
			".long " __stringify(_sym) "\n"			\
			_insn "	%[to], %[from], %[imm]\n",		\
		/* operands */						\
			: [to]	 "=r"  (_to),				\
			  [tmp]	 "=&r" (__tmp)				\
			: [from] "r"   (_from),				\
			  [imm]	 "I"   (__IMM8_CONST_DUMMY),		\
				 "i"   (&(_sym)));			\
} while (0)

#define early_runtime_patch_imm8_mov(_insn, _to, _sym)			\
do {									\
	unsigned long __tmp;						\
	early_runtime_patch_stub(					\
		/* type */						\
			RUNTIME_PATCH_TYPE_IMM8				\
		/* code */						\
			"ldr	%[tmp], =" __stringify(_sym) "\n"	\
			"ldr	%[tmp], [%[tmp]]\n"			\
			_insn "	%[to], %[tmp]\n",			\
		/* pad */						\
			0,						\
		/* patch_data */					\
			".long " __stringify(_sym) "\n"			\
			_insn " %[to], %[imm]\n",			\
		/* operands */						\
			: [to]	"=r"  (_to),				\
			  [tmp]	"=&r" (__tmp)				\
			: [imm]	"I"   (__IMM8_CONST_DUMMY),		\
				"i"   (&(_sym)));			\
} while (0)

int runtime_patch(const void *table, unsigned size);
void runtime_patch_kernel(void);

#else

static inline int runtime_patch(const void *table, unsigned size)
{
	return -ENOSYS;
}

static inline void runtime_patch_kernel(void)
{
}

#endif /* CONFIG_ARM_RUNTIME_PATCH */

#endif /* __ASSEMBLY__ */

#endif /* __ASM_ARM_RUNTIME_PATCH_H */
