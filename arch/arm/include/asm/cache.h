/*
 *  arch/arm/include/asm/cache.h
 */
#ifndef __ASMARM_CACHE_H
#define __ASMARM_CACHE_H

#define L1_CACHE_SHIFT		CONFIG_ARM_L1_CACHE_SHIFT
#define L1_CACHE_BYTES		(1 << L1_CACHE_SHIFT)

/*
 * Memory returned by kmalloc() may be used for DMA, so we must make
 * sure that all such allocations are cache aligned. Otherwise,
 * unrelated code may cause parts of the buffer to be read into the
 * cache before the transfer is done, causing old data to be seen by
 * the CPU.
 */
#define ARCH_DMA_MINALIGN	L1_CACHE_BYTES

/*
 * Minimum guaranted alignment in pgd_alloc().  The page table pointers passed
 * around in head.S and proc-*.S are shifted by this amount, in order to
 * leave spare high bits for systems with physical address extension.  This
 * does not fully accomodate the 40-bit addressing capability of ARM LPAE, but
 * gives us about 38-bits or so.
 */
#define ARCH_PGD_SHIFT		L1_CACHE_SHIFT

/*
 * With EABI on ARMv5 and above we must have 64-bit aligned slab pointers.
 */
#if defined(CONFIG_AEABI) && (__LINUX_ARM_ARCH__ >= 5)
#define ARCH_SLAB_MINALIGN 8
#endif

#define __read_mostly __attribute__((__section__(".data..read_mostly")))

#endif
