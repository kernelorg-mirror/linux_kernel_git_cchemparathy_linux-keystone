/*
 * Copyright (C) 2012 Texas Instruments Incorporated
 * Authors: Sandeep Paulraj <s-paulraj@ti.com>
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
#include <linux/sched.h>
#include <linux/io.h>
#include <linux/err.h>
#include <linux/delay.h>

#include "keystone_net.h"

#define SGMII_REG_BASE                  0x02090100
#define SGMII_SERDES_BASE		0x02620340
#define SGMII_SERDES_CFGPLL		0x0
#define SGMII_SERDES_CFGRX0		0x4
#define SGMII_SERDES_CFGTX0		0x8
#define SGMII_SERDES_CFGRX1		0xC
#define SGMII_SERDES_CFGTX1		0x10
#define SGMII_SERDES_SIZE		0x14

#define SGMII_SRESET_RESET		0x1
#define SGMII_SRESET_RTRESET		0x2
#define SGMII_CTL_AUTONEG		0x01
#define SGMII_CTL_LOOPBACK		0x10
#define SGMII_CTL_MASTER		0x20
#define SGMII_REG_STATUS_LOCK		BIT(4)
#define	SGMII_REG_STATUS_LINK		BIT(0)
#define SGMII_REG_STATUS_AUTONEG	BIT(2)
#define SGMII_REG_CONTROL_AUTONEG	BIT(0)

#define SGMII_OFFSET(x)	((x <= 1)? (x * 0x100): ((x * 0x100) + 0x100))
/*
 * SGMII registers
 */
#define SGMII_IDVER_REG(x)    (SGMII_OFFSET(x) + 0x000)
#define SGMII_SRESET_REG(x)   (SGMII_OFFSET(x) + 0x004)
#define SGMII_CTL_REG(x)      (SGMII_OFFSET(x) + 0x010)
#define SGMII_STATUS_REG(x)   (SGMII_OFFSET(x) + 0x014)
#define SGMII_MRADV_REG(x)    (SGMII_OFFSET(x) + 0x018)
#define SGMII_LPADV_REG(x)    (SGMII_OFFSET(x) + 0x020)
#define SGMII_TXCFG_REG(x)    (SGMII_OFFSET(x) + 0x030)
#define SGMII_RXCFG_REG(x)    (SGMII_OFFSET(x) + 0x034)
#define SGMII_AUXCFG_REG(x)   (SGMII_OFFSET(x) + 0x038)

struct sgmii_config {
	u32	interface;
	u32	mr_adv_ability;
	u32	control;
};

static inline void sgmii_write_reg(void __iomem *base, int reg, u32 val)
{
	__raw_writel(val, base + reg);
}

static inline u32 sgmii_read_reg(void __iomem *base, int reg)
{
	return __raw_readl(base + reg);
}

static inline void sgmii_write_reg_bit(void __iomem *base, int reg, u32 val)
{
	__raw_writel((__raw_readl(base + reg) | val),
			base + reg);
}

int serdes_init(void)
{
	void __iomem *sgmii_serdes;

	sgmii_serdes = ioremap(SGMII_SERDES_BASE, SGMII_SERDES_SIZE);

	sgmii_write_reg(sgmii_serdes, SGMII_SERDES_CFGPLL, 0x00000041);
	
	udelay(2000);

	sgmii_write_reg(sgmii_serdes, SGMII_SERDES_CFGRX0, 0x00700621);
	sgmii_write_reg(sgmii_serdes, SGMII_SERDES_CFGRX1, 0x00700621);

	sgmii_write_reg(sgmii_serdes, SGMII_SERDES_CFGTX0, 0x000108a1);
	sgmii_write_reg(sgmii_serdes, SGMII_SERDES_CFGTX1, 0x000108a1);

	udelay(2000);

	iounmap(sgmii_serdes);

	return 0;
}	

int keystone_sgmii_reset(void __iomem *sgmii_ofs, int port)
{
	/* Soft reset */
	sgmii_write_reg_bit(sgmii_ofs, SGMII_SRESET_REG(port), 0x1);
	while(sgmii_read_reg(sgmii_ofs, SGMII_SRESET_REG(port)) != 0x0);

	return 0;
}

int keystone_sgmii_link_status(void __iomem *sgmii_ofs, int ports)
{
	u32 status = 0, link = 0;
	u32 i;

	for (i = 0; i < ports; i++) {
		status = sgmii_read_reg(sgmii_ofs, SGMII_STATUS_REG(i));
		if ((status & SGMII_REG_STATUS_LINK) != 0)
			link |= BIT(i);
		else
			link &= ~BIT(i);
	}

	return link;
}

int keystone_sgmii_config(void __iomem *sgmii_ofs,
			  int port, u32 interface)
{
	unsigned int i, status, mask;
	struct sgmii_config *config;

	config = kzalloc(sizeof(*config), GFP_KERNEL);
	if (!config)
		return -ENOMEM;

	switch (interface) {
	case SGMII_LINK_MAC_MAC_AUTONEG:
		config->mr_adv_ability	= 0x9801;
		config->control		= 0x21;

		break;
	case SGMII_LINK_MAC_PHY:
		config->mr_adv_ability	= 1;
		config->control		= 1;

		break;
	case SGMII_LINK_MAC_MAC_FORCED:
		config->mr_adv_ability	= 0x9801;
		config->control		= 0x20;

		break;
	case SGMII_LINK_MAC_FIBER:
		config->mr_adv_ability	= 0x20;
		config->control		= 0x1;

		break;
	}

	sgmii_write_reg(sgmii_ofs, SGMII_CTL_REG(port), 0);

	/*
	 * Wait for the SerDes pll to lock,
	 * but don't trap if lock is never read
	 */
	for (i = 0; i < 1000; i++)  {
		udelay(2000);
		status = sgmii_read_reg(sgmii_ofs, SGMII_STATUS_REG(port));
		if ((status & SGMII_REG_STATUS_LOCK) != 0)
			break;
	}

	sgmii_write_reg(sgmii_ofs,
			SGMII_MRADV_REG(port), config->mr_adv_ability);
	sgmii_write_reg(sgmii_ofs,
			SGMII_CTL_REG(port), config->control);


	mask = SGMII_REG_STATUS_LINK;

	if (config->control & SGMII_REG_CONTROL_AUTONEG)
		mask |= SGMII_REG_STATUS_AUTONEG;

	for (i = 0; i < 1000; i++)  {
		status = sgmii_read_reg(sgmii_ofs, SGMII_STATUS_REG(port));
		if ((status & mask) == mask)
			break;
	}

	kfree(config);
	return 0;
}
