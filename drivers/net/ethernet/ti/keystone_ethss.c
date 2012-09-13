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

#include <linux/io.h>
#include <linux/of.h>
#include <linux/clk.h>
#include <linux/phy.h>
#include <linux/timer.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/uaccess.h>
#include <linux/if_vlan.h>
#include <linux/ethtool.h>
#include <linux/if_ether.h>
#include <linux/netdevice.h>
#include <linux/interrupt.h>
#include <linux/dmaengine.h>
#include <linux/dma-mapping.h>
#include <linux/scatterlist.h>
#include <linux/etherdevice.h>
#include <linux/platform_device.h>

#include "cpsw_ale.h"
#include "keystone_net.h"

#define NETCP_DRIVER_NAME	"TI KeyStone Ethernet Driver"
#define NETCP_DRIVER_VERSION	"v1.2.2"

#define CPSW_SGMII_IDENT(reg)		((reg >> 16) & 0xffff)
#define CPSW_MAJOR_VERSION(reg)		(reg >> 8 & 0x7)
#define CPSW_MINOR_VERSION(reg)		(reg & 0xff)
#define CPSW_RTL_VERSION(reg)		((reg >> 11) & 0x1f)

#define TCI6614_SS_BASE				0x02090000
#define DEVICE_N_GMACSL_PORTS			2
#define DEVICE_EMACSL_RESET_POLL_COUNT		100

#define	CPSW_TIMER_INTERVAL			(HZ / 10)

/* Soft reset register values */
#define SOFT_RESET_MASK				BIT(0)
#define SOFT_RESET				BIT(0)

#define MACSL_RX_ENABLE_EXT_CTL			BIT(18)
#define MACSL_ENABLE				BIT(5)
#define GMACSL_RET_WARN_RESET_INCOMPLETE	-2

#define CPSW_NUM_PORTS		                3
#define CPSW_CTL_P0_ENABLE			BIT(2)
#define CPSW_CTL_VLAN_AWARE			BIT(1)
#define CPSW_REG_VAL_STAT_ENABLE_ALL		0xf

#define CPSW_MASK_ALL_PORTS			7
#define CPSW_MASK_PHYS_PORTS			6
#define CPSW_MASK_NO_PORTS			0
#define CPSW_NON_VLAN_ADDR			-1

#define CPSW_STATSA_MODULE			0
#define CPSW_STATSB_MODULE			1

#define MAX_SIZE_STREAM_BUFFER		        9504

struct cpsw_slave {
	struct cpsw_slave_regs __iomem	*regs;
	struct cpsw_sliver_regs __iomem	*sliver;
	int				 slave_num;
	int				 port_num;
	u32				 mac_control;
	struct phy_device		*phy;
	const char			*phy_id;
	struct cpsw_ale			*ale;
	u32				 link_interface;
};

struct cpsw_ss_regs {
	u32	id_ver;
	u32	soft_reset;
	u32	control;
	u32	int_control;
	u32	rx_thresh_en;
	u32	rx_en;
	u32	tx_en;
	u32	misc_en;
	u32	mem_allign1[8];
	u32	rx_thresh_stat;
	u32	rx_stat;
	u32	tx_stat;
	u32	misc_stat;
	u32	mem_allign2[8];
	u32	rx_imax;
	u32	tx_imax;
};

struct cpsw_regs {
	u32	id_ver;
	u32	control;
	u32	soft_reset;
	u32	stat_port_en;
	u32	ptype;
	u32	soft_idle;
	u32	thru_rate;
	u32	gap_thresh;
	u32	tx_start_wds;
	u32	flow_control;
};

struct cpsw_slave_regs {
	u32	max_blks;
	u32	blk_cnt;
	u32	port_vlan;
	u32	tx_pri_map;
	u32	sa_lo;
	u32	sa_hi;
	u32	ts_ctl;
	u32	ts_seq_ltype;
	u32	ts_vlan;
};

struct cpsw_host_regs {
	u32	src_id;
	u32	port_vlan;
	u32	rx_pri_map;
	u32	rx_maxlen;
};

struct cpsw_sliver_regs {
	u32	id_ver;
	u32	mac_control;
	u32	mac_status;
	u32	soft_reset;
	u32	rx_maxlen;
	u32	__reserved_0;
	u32	rx_pause;
	u32	tx_pause;
	u32	__reserved_1;
	u32	rx_pri_map;
	u32	rsvd[6];
};

struct cpsw_hw_stats {
	u32	rx_good_frames;
	u32	rx_broadcast_frames;
	u32	rx_multicast_frames;
	u32	rx_pause_frames;
	u32	rx_crc_errors;
	u32	rx_align_code_errors;
	u32	rx_oversized_frames;
	u32	rx_jabber_frames;
	u32	rx_undersized_frames;
	u32	rx_fragments;
	u32	__pad_0[2];
	u32	rx_bytes;
	u32	tx_good_frames;
	u32	tx_broadcast_frames;
	u32	tx_multicast_frames;
	u32	tx_pause_frames;
	u32	tx_deferred_frames;
	u32	tx_collision_frames;
	u32	tx_single_coll_frames;
	u32	tx_mult_coll_frames;
	u32	tx_excessive_collisions;
	u32	tx_late_collisions;
	u32	tx_underrun;
	u32	tx_carrier_sense_errors;
	u32	tx_bytes;
	u32	tx_64byte_frames;
	u32	tx_65_to_127byte_frames;
	u32	tx_128_to_255byte_frames;
	u32	tx_256_to_511byte_frames;
	u32	tx_512_to_1023byte_frames;
	u32	tx_1024byte_frames;
	u32	net_bytes;
	u32	rx_sof_overruns;
	u32	rx_mof_overruns;
	u32	rx_dma_overruns;
};

struct cpsw_ale_regs {
	u32	ale_idver;
	u32	rsvd0;
	u32	ale_control;
	u32	rsvd1;
	u32	ale_prescale;
	u32	rsvd2;
	u32	ale_unknown_vlan;
	u32	rsvd3;
	u32	ale_tblctl;
	u32	rsvd4[4];
	u32	ale_tblw2;
	u32	ale_tblw1;
	u32	ale_tblw0;
	u32	ale_portctl[6];
};

struct cpsw_priv {
	struct device			*dev;
	struct net_device		*ndev;
	struct netcp_module_data	 module;
	struct clk			*cpgmac;
	u32				 num_slaves;
	u32				 ale_ageout;
	u32				 ale_entries;
	u32				 ale_ports;
	u32				 sgmii_module_ofs;
	u32				 switch_module_ofs;
	u32				 host_port_reg_ofs;
	u32				 slave_reg_ofs;
	u32				 sliver_reg_ofs;
	u32				 hw_stats_reg_ofs;
	u32				 ale_reg_ofs;

	int				 host_port;
	u32				 rx_packet_max;

	struct cpsw_regs __iomem	*regs;
	struct cpsw_ss_regs __iomem	*ss_regs;
	struct cpsw_hw_stats __iomem	*hw_stats[2];
	struct cpsw_host_regs __iomem	*host_port_regs;
	struct cpsw_ale_regs __iomem	*ale_reg;

	void __iomem			*sgmii_port_regs;

	u8				 mac_addr[ETH_ALEN];
	struct cpsw_slave		*slaves;
	struct cpsw_ale			*ale;

	u32				 link[5];
	u32				 sgmii_link;

	struct timer_list		 timer;

	u32				 tx_queue_depth;
	struct netcp_tx_pipe		 tx_pipe;
};

static struct cpsw_priv *priv;

/*
 * Statistic management
 */
struct netcp_ethtool_stat {
	char desc[ETH_GSTRING_LEN];
	int type;
	u32 size;
	int offset;
};

#define cpsw_from_module(data)	container_of(data, struct cpsw_priv, module)
#define cpsw_to_module(cpsw)	(&(cpsw)->module)

#define for_each_slave(priv, func, arg...)			\
	do {							\
		int idx;					\
		for (idx = 0; idx < (priv)->num_slaves; idx++)	\
			(func)((priv)->slaves + idx, ##arg);	\
	} while (0)

#define FIELDINFO(_struct, field)       FIELD_SIZEOF(_struct, field),	\
		                                offsetof(_struct, field)
#define CPSW_STATSA_INFO(field) 	"CPSW_A:"#field, CPSW_STATSA_MODULE,\
					FIELDINFO(struct cpsw_hw_stats,\
						field)
#define CPSW_STATSB_INFO(field) 	"CPSW_B:"#field, CPSW_STATSB_MODULE,\
					FIELDINFO(struct cpsw_hw_stats,\
						field)

static const struct netcp_ethtool_stat et_stats[] = {
	/* CPSW module A */
	{CPSW_STATSA_INFO(rx_good_frames)},
	{CPSW_STATSA_INFO(rx_broadcast_frames)},
	{CPSW_STATSA_INFO(rx_multicast_frames)},
	{CPSW_STATSA_INFO(rx_pause_frames)},
	{CPSW_STATSA_INFO(rx_crc_errors)},
	{CPSW_STATSA_INFO(rx_align_code_errors)},
	{CPSW_STATSA_INFO(rx_oversized_frames)},
	{CPSW_STATSA_INFO(rx_jabber_frames)},
	{CPSW_STATSA_INFO(rx_undersized_frames)},
	{CPSW_STATSA_INFO(rx_fragments)},
	{CPSW_STATSA_INFO(rx_bytes)},
	{CPSW_STATSA_INFO(tx_good_frames)},
	{CPSW_STATSA_INFO(tx_broadcast_frames)},
	{CPSW_STATSA_INFO(tx_multicast_frames)},
	{CPSW_STATSA_INFO(tx_pause_frames)},
	{CPSW_STATSA_INFO(tx_deferred_frames)},
	{CPSW_STATSA_INFO(tx_collision_frames)},
	{CPSW_STATSA_INFO(tx_single_coll_frames)},
	{CPSW_STATSA_INFO(tx_mult_coll_frames)},
	{CPSW_STATSA_INFO(tx_excessive_collisions)},
	{CPSW_STATSA_INFO(tx_late_collisions)},
	{CPSW_STATSA_INFO(tx_underrun)},
	{CPSW_STATSA_INFO(tx_carrier_sense_errors)},
	{CPSW_STATSA_INFO(tx_bytes)},
	{CPSW_STATSA_INFO(tx_64byte_frames)},
	{CPSW_STATSA_INFO(tx_65_to_127byte_frames)},
	{CPSW_STATSA_INFO(tx_128_to_255byte_frames)},
	{CPSW_STATSA_INFO(tx_256_to_511byte_frames)},
	{CPSW_STATSA_INFO(tx_512_to_1023byte_frames)},
	{CPSW_STATSA_INFO(tx_1024byte_frames)},
	{CPSW_STATSA_INFO(net_bytes)},
	{CPSW_STATSA_INFO(rx_sof_overruns)},
	{CPSW_STATSA_INFO(rx_mof_overruns)},
	{CPSW_STATSA_INFO(rx_dma_overruns)},
	/* CPSW module B */
	{CPSW_STATSB_INFO(rx_good_frames)},
	{CPSW_STATSB_INFO(rx_broadcast_frames)},
	{CPSW_STATSB_INFO(rx_multicast_frames)},
	{CPSW_STATSB_INFO(rx_pause_frames)},
	{CPSW_STATSB_INFO(rx_crc_errors)},
	{CPSW_STATSB_INFO(rx_align_code_errors)},
	{CPSW_STATSB_INFO(rx_oversized_frames)},
	{CPSW_STATSB_INFO(rx_jabber_frames)},
	{CPSW_STATSB_INFO(rx_undersized_frames)},
	{CPSW_STATSB_INFO(rx_fragments)},
	{CPSW_STATSB_INFO(rx_bytes)},
	{CPSW_STATSB_INFO(tx_good_frames)},
	{CPSW_STATSB_INFO(tx_broadcast_frames)},
	{CPSW_STATSB_INFO(tx_multicast_frames)},
	{CPSW_STATSB_INFO(tx_pause_frames)},
	{CPSW_STATSB_INFO(tx_deferred_frames)},
	{CPSW_STATSB_INFO(tx_collision_frames)},
	{CPSW_STATSB_INFO(tx_single_coll_frames)},
	{CPSW_STATSB_INFO(tx_mult_coll_frames)},
	{CPSW_STATSB_INFO(tx_excessive_collisions)},
	{CPSW_STATSB_INFO(tx_late_collisions)},
	{CPSW_STATSB_INFO(tx_underrun)},
	{CPSW_STATSB_INFO(tx_carrier_sense_errors)},
	{CPSW_STATSB_INFO(tx_bytes)},
	{CPSW_STATSB_INFO(tx_64byte_frames)},
	{CPSW_STATSB_INFO(tx_65_to_127byte_frames)},
	{CPSW_STATSB_INFO(tx_128_to_255byte_frames)},
	{CPSW_STATSB_INFO(tx_256_to_511byte_frames)},
	{CPSW_STATSB_INFO(tx_512_to_1023byte_frames)},
	{CPSW_STATSB_INFO(tx_1024byte_frames)},
	{CPSW_STATSB_INFO(net_bytes)},
	{CPSW_STATSB_INFO(rx_sof_overruns)},
	{CPSW_STATSB_INFO(rx_mof_overruns)},
	{CPSW_STATSB_INFO(rx_dma_overruns)},
};

#define ETHTOOL_STATS_NUM ARRAY_SIZE(et_stats)

static void keystone_get_drvinfo(struct net_device *ndev,
			     struct ethtool_drvinfo *info)
{
	strcpy(info->driver, NETCP_DRIVER_NAME);
	strcpy(info->version, NETCP_DRIVER_VERSION);
}

static u32 keystone_get_msglevel(struct net_device *ndev)
{
	struct netcp_priv *netcp = netdev_priv(ndev);
	return netcp->msg_enable;
}

static void keystone_set_msglevel(struct net_device *ndev, u32 value)
{
	struct netcp_priv *netcp = netdev_priv(ndev);
	netcp->msg_enable = value;
}

static void keystone_get_stat_strings(struct net_device *ndev,
				   uint32_t stringset, uint8_t *data)
{
	int i;

	switch (stringset) {
	case ETH_SS_STATS:
		for (i = 0; i < ETHTOOL_STATS_NUM; i++) {
			memcpy(data, et_stats[i].desc, ETH_GSTRING_LEN);
			data += ETH_GSTRING_LEN;
		}
		break;
	case ETH_SS_TEST:
		break;
	}
}

static int keystone_get_sset_count(struct net_device *ndev, int stringset)
{
	switch (stringset) {
	case ETH_SS_TEST:
		return 0;
	case ETH_SS_STATS:
		return ETHTOOL_STATS_NUM;
	default:
		return -EINVAL;
	}
}

static void cpsw_update_stats(struct cpsw_priv *cpsw_dev, uint64_t *data)
{
	struct cpsw_hw_stats __iomem *cpsw_statsa = cpsw_dev->hw_stats[0];
	struct cpsw_hw_stats __iomem *cpsw_statsb = cpsw_dev->hw_stats[1];
	struct netcp_priv *netcp = netdev_priv(cpsw_dev->ndev);
	void *p = NULL;
	u32 tmp = 0;
	int i;

	for (i = 0; i < ETHTOOL_STATS_NUM; i++) {
		switch (et_stats[i].type) {
		case CPSW_STATSA_MODULE:
			p = cpsw_statsa;
			break;
		case CPSW_STATSB_MODULE:
			p  = cpsw_statsb;
			break;
		}

		p = (u8 *)p + et_stats[i].offset;
		tmp = *(u32 *)p;
		data[i] = netcp->hw_stats[i] + tmp;
		netcp->hw_stats[i] = data[i];
		*(u32 *)p = tmp;
	}

	return;
}

static void keystone_get_ethtool_stats(struct net_device *ndev,
				       struct ethtool_stats *stats,
				       uint64_t *data)
{
	cpsw_update_stats(priv, data);

	return;
}

static const struct ethtool_ops keystone_ethtool_ops = {
	.get_drvinfo		= keystone_get_drvinfo,
	.get_link		= ethtool_op_get_link,
	.get_msglevel		= keystone_get_msglevel,
	.set_msglevel		= keystone_set_msglevel,
	.get_strings		= keystone_get_stat_strings,
	.get_sset_count		= keystone_get_sset_count,
	.get_ethtool_stats	= keystone_get_ethtool_stats,
};

void keystone_set_ethtool_ops(struct net_device *ndev)
{
	SET_ETHTOOL_OPS(ndev, &keystone_ethtool_ops);
}

#define mac_hi(mac)	(((mac)[0] << 0) | ((mac)[1] << 8) |	\
			 ((mac)[2] << 16) | ((mac)[3] << 24))
#define mac_lo(mac)	(((mac)[4] << 0) | ((mac)[5] << 8))

static void cpsw_set_slave_mac(struct cpsw_slave *slave,
			       struct cpsw_priv *priv)
{
	__raw_writel(mac_hi(priv->mac_addr), &slave->regs->sa_hi);
	__raw_writel(mac_lo(priv->mac_addr), &slave->regs->sa_lo);
}

static inline int cpsw_get_slave_port(struct cpsw_priv *priv, u32 slave_num)
{
	if (priv->host_port == 0)
		return slave_num + 1;
	else
		return slave_num;
}

static void _cpsw_adjust_link(struct cpsw_slave *slave, bool *link)
{
	struct phy_device *phy = slave->phy;
	u32 mac_control = 0;
	u32 slave_port;

	if (!phy)
		return;

	slave_port = slave->port_num;

	if (phy->link) {
		mac_control = slave->mac_control;
		mac_control |= MACSL_ENABLE | MACSL_RX_ENABLE_EXT_CTL;
		/* enable forwarding */
		cpsw_ale_control_set(slave->ale, slave_port,
				     ALE_PORT_STATE, ALE_PORT_STATE_FORWARD);

		if (phy->duplex)
			mac_control |= BIT(0);	/* FULLDUPLEXEN	*/
		else
			mac_control &= ~0x1;

		*link = true;
	} else {
		mac_control = 0;
		/* disable forwarding */
		cpsw_ale_control_set(slave->ale, slave_port,
				     ALE_PORT_STATE, ALE_PORT_STATE_DISABLE);
	}

	if (mac_control != slave->mac_control) {
		phy_print_status(phy);
		__raw_writel(mac_control, &slave->sliver->mac_control);
	}

	slave->mac_control = mac_control;
}

static void cpsw_adjust_link(struct net_device *n_dev, void *context)
{
	struct cpsw_slave *slave = (struct cpsw_slave *)context;
	struct netcp_priv *netcp = netdev_priv(n_dev);
	bool link = false;

	_cpsw_adjust_link(slave, &link);

	if (link)
		netcp->link_state |= BIT(slave->slave_num);
	else
		netcp->link_state &= ~BIT(slave->slave_num);
}

/*
 * Reset the the mac sliver
 * Soft reset is set and polled until clear, or until a timeout occurs
 */
static int cpsw_port_reset(struct cpsw_slave *slave)
{
	u32 i, v;

	/* Set the soft reset bit */
	__raw_writel(SOFT_RESET,
		     &slave->sliver->soft_reset);

	/* Wait for the bit to clear */
	for (i = 0; i < DEVICE_EMACSL_RESET_POLL_COUNT; i++) {
		v = __raw_readl(&slave->sliver->soft_reset);
		if ((v & SOFT_RESET_MASK) !=
		    SOFT_RESET)
			return 0;
	}

	/* Timeout on the reset */
	return GMACSL_RET_WARN_RESET_INCOMPLETE;
}

/*
 * Configure the mac sliver
 */
static void cpsw_port_config(struct cpsw_slave *slave, int max_rx_len)
{
	if (max_rx_len > MAX_SIZE_STREAM_BUFFER)
		max_rx_len = MAX_SIZE_STREAM_BUFFER;

	__raw_writel(max_rx_len, &slave->sliver->rx_maxlen);
	
	__raw_writel(MACSL_ENABLE | MACSL_RX_ENABLE_EXT_CTL,
			&slave->sliver->mac_control);
}

static void cpsw_slave_stop(struct cpsw_slave *slave, struct cpsw_priv *priv)
{
	cpsw_port_reset(slave);

	if (!slave->phy)
		return;

	phy_stop(slave->phy);
	phy_disconnect(slave->phy);
	slave->phy = NULL;
}

static int match_device(struct device *dev, void *data)
{
	struct cpsw_slave *slave = (struct cpsw_slave *)data;

	if (slave->link_interface == SGMII_LINK_MAC_PHY)
		return 1;
	else
		return 0;
}

static void cpsw_slave_link(struct cpsw_slave *slave, struct cpsw_priv *priv)
{
	struct netcp_priv *netcp = netdev_priv(priv->ndev);

	if (slave->link_interface == SGMII_LINK_MAC_PHY) {
		if (netcp->link_state)
			priv->sgmii_link |= BIT(slave->slave_num);
		else
			priv->sgmii_link &= ~BIT(slave->slave_num);
	}
}

static void cpsw_slave_open(struct cpsw_slave *slave, struct cpsw_priv *priv)
{
	char name[32];
	u32 slave_port;

	sprintf(name, "slave-%d", slave->slave_num);

	keystone_sgmii_reset(priv->sgmii_port_regs, slave->slave_num);

	keystone_sgmii_config(priv->sgmii_port_regs, slave->slave_num,
				slave->link_interface);

	cpsw_port_reset(slave);

	cpsw_port_config(slave, priv->rx_packet_max);

	cpsw_set_slave_mac(slave, priv);

	slave->mac_control = MACSL_ENABLE | MACSL_RX_ENABLE_EXT_CTL;

	slave_port = cpsw_get_slave_port(priv, slave->slave_num);

	slave->port_num = slave_port;
	slave->ale = priv->ale;

	/* enable forwarding */
	cpsw_ale_control_set(priv->ale, slave_port,
			     ALE_PORT_STATE, ALE_PORT_STATE_FORWARD);

	cpsw_ale_add_mcast(priv->ale, priv->ndev->broadcast,
			   1 << slave_port, 0, ALE_MCAST_FWD_2,
			   CPSW_NON_VLAN_ADDR);

	if (slave->link_interface == SGMII_LINK_MAC_PHY) {
		if (!slave->phy_id) {
			struct device *phy;

			phy = bus_find_device(&mdio_bus_type, NULL, slave,
				      match_device);
			if (phy)
				slave->phy_id = dev_name(phy);
		}

		if (slave->phy_id && *slave->phy_id) {
			slave->phy = phy_connect(priv->ndev, slave->phy_id,
					&cpsw_adjust_link, 0,
					PHY_INTERFACE_MODE_SGMII,
					slave);
			if (IS_ERR(slave->phy)) {
				dev_err(priv->dev, "phy %s not found on"
					" slave %d\n", slave->phy_id,
					slave->slave_num);
				slave->phy = NULL;
			} else {
				dev_info(priv->dev, "phy found: id is: 0x%s\n",
					slave->phy_id);
				phy_start(slave->phy);
			}
		}
	}
}

static void cpsw_init_host_port(struct cpsw_priv *priv)
{
	/* Max length register */
	__raw_writel(MAX_SIZE_STREAM_BUFFER,
		     &priv->host_port_regs->rx_maxlen);

	cpsw_ale_start(priv->ale);

	/* switch to vlan aware mode */
	cpsw_ale_control_set(priv->ale, 0, ALE_VLAN_AWARE, 1);

	cpsw_ale_control_set(priv->ale, 0, ALE_NO_PORT_VLAN, 1);

	cpsw_ale_control_set(priv->ale, priv->host_port,
			     ALE_PORT_STATE, ALE_PORT_STATE_FORWARD);

	cpsw_ale_control_set(priv->ale, 0,
			     ALE_PORT_UNKNOWN_VLAN_MEMBER,
			     CPSW_MASK_ALL_PORTS);

	cpsw_ale_control_set(priv->ale, 0,
			     ALE_PORT_UNKNOWN_MCAST_FLOOD,
			     CPSW_MASK_PHYS_PORTS);

	cpsw_ale_control_set(priv->ale, 0,
			     ALE_PORT_UNKNOWN_REG_MCAST_FLOOD,
			     CPSW_MASK_ALL_PORTS);

	cpsw_ale_control_set(priv->ale, 0,
			     ALE_PORT_UNTAGGED_EGRESS,
			     CPSW_MASK_ALL_PORTS);

	cpsw_ale_add_ucast(priv->ale, priv->mac_addr,
			   priv->host_port, 0, CPSW_NON_VLAN_ADDR);

	cpsw_ale_add_mcast(priv->ale, priv->ndev->broadcast,
			   1 << priv->host_port, 0,
			   ALE_MCAST_FWD_2, CPSW_NON_VLAN_ADDR);

}

static void cpsw_slave_init(struct cpsw_slave *slave, struct cpsw_priv *priv)
{
	void __iomem		*regs = priv->ss_regs;
	int			slave_num = slave->slave_num;

	slave->regs	= regs + priv->slave_reg_ofs + (0x30 * slave_num);
	slave->sliver	= regs + priv->sliver_reg_ofs + (0x40 * slave_num);
}

int cpsw_add_ucast(struct netcp_module_data *data, u8 *uc_list,
		   int uc_count, int vid)
{
	struct cpsw_priv *cpsw_dev = cpsw_from_module(data);
	u8 mac_addr[ETH_ALEN];
	int i;

	dev_dbg(cpsw_dev->dev, "uc count is = %d\n", uc_count);

	for (i = 0; i < uc_count; i++) {
		dev_dbg(cpsw_dev->dev, "uc address %x:%x:%x:%x:%x:%x\n",
			uc_list[(i * 6) + 0], uc_list[(i * 6) + 1],
			uc_list[(i * 6) + 2], uc_list[(i * 6) + 3],
			uc_list[(i * 6) + 4], uc_list[(i * 6) + 5]);

		memcpy(mac_addr, &uc_list[(i * 6)], ETH_ALEN);

		cpsw_ale_add_ucast(cpsw_dev->ale, mac_addr,
				   cpsw_dev->host_port, 0, vid);

	}

	return 0;
}

int cpsw_add_mcast(struct netcp_module_data *data, u8 *mc_list,
		   int mc_count, int vid)
{
	struct cpsw_priv *cpsw_dev = cpsw_from_module(data);
	u8 mac_addr[ETH_ALEN];
	int i;

	dev_dbg(cpsw_dev->dev, "mc count is = %d\n", mc_count);

	for (i = 0; i < mc_count; i++) {
		dev_dbg(cpsw_dev->dev, "mc address %x:%x:%x:%x:%x:%x\n",
			mc_list[(i * 6) + 0], mc_list[(i * 6) + 1],
			mc_list[(i * 6) + 2], mc_list[(i * 6) + 3],
			mc_list[(i * 6) + 4], mc_list[(i * 6) + 5]);

		memcpy(mac_addr, &mc_list[(i * 6)], ETH_ALEN);

		cpsw_ale_add_mcast(cpsw_dev->ale, mac_addr,
				   CPSW_MASK_ALL_PORTS, 0,
				   ALE_MCAST_FWD_2, vid);
	}

	return 0;
}

int cpsw_add_vid(struct netcp_module_data *data, int vid)
{
	struct cpsw_priv *cpsw_dev = cpsw_from_module(data);

	cpsw_ale_add_vlan(cpsw_dev->ale, vid, CPSW_MASK_ALL_PORTS,
			  CPSW_MASK_ALL_PORTS, CPSW_MASK_PHYS_PORTS,
			  CPSW_MASK_NO_PORTS);

	return 0;
}

int cpsw_del_vid(struct netcp_module_data *data, int vid)
{
	struct cpsw_priv *cpsw_dev = cpsw_from_module(data);

	cpsw_ale_del_vlan(cpsw_dev->ale, vid);

	return 0;
}

static void cpsw_timer(unsigned long arg)
{
	struct cpsw_priv *cpsw_dev = (struct cpsw_priv *)arg;
	uint64_t data[128];

	cpsw_dev->sgmii_link = keystone_sgmii_link_status(cpsw_dev->sgmii_port_regs,
						cpsw_dev->num_slaves);
	for_each_slave(cpsw_dev, cpsw_slave_link, cpsw_dev);

	if (cpsw_dev->sgmii_link) {
		/* link ON */
		if (!netif_carrier_ok(cpsw_dev->ndev))
			netif_carrier_on(cpsw_dev->ndev);
		/*
		 * reactivate the transmit queue if
		 * it is stopped
		 */
		if (netif_running(cpsw_dev->ndev) &&
			netif_queue_stopped(cpsw_dev->ndev))
			netif_wake_queue(cpsw_dev->ndev);
	} else {
		/* link OFF */
		if (netif_carrier_ok(cpsw_dev->ndev))
			netif_carrier_off(cpsw_dev->ndev);
		if (!netif_queue_stopped(cpsw_dev->ndev))
			netif_stop_queue(cpsw_dev->ndev);
	}

	cpsw_update_stats(cpsw_dev, data);

	cpsw_dev->timer.expires = jiffies;
	add_timer(&cpsw_dev->timer);

	return;
}

static int cpsw_tx_hook(int order, void *data, struct netcp_packet *p_info)
{
	struct cpsw_priv *cpsw_priv = data;

	p_info->tx_pipe = &cpsw_priv->tx_pipe;
	return 0;
}

#define	CPSW_TXHOOK_ORDER	0

static int cpsw_open_txchan(struct cpsw_priv *cpsw_dev)
{
	struct dma_keystone_info config;
	dma_cap_mask_t mask;
	const char *name;
	int err;

	dma_cap_zero(mask);
	dma_cap_set(DMA_SLAVE, mask);

	name = cpsw_dev->tx_pipe.dma_chan_name;
	cpsw_dev->tx_pipe.dma_channel = dma_request_channel_by_name(mask, name);
	if (IS_ERR_OR_NULL(cpsw_dev->tx_pipe.dma_channel))
		return PTR_ERR(cpsw_dev->tx_pipe.dma_channel);

	memset(&config, 0, sizeof(config));
	config.direction	= DMA_MEM_TO_DEV;
	config.tx_queue_depth	= cpsw_dev->tx_queue_depth;
	err = dma_keystone_config(cpsw_dev->tx_pipe.dma_channel, &config);
	if (err) {
		dev_err(cpsw_dev->dev, "%d error configuring TX channel\n",
			err);
		return err;
	}

	dmaengine_pause(cpsw_dev->tx_pipe.dma_channel);
	cpsw_dev->tx_pipe.dma_flow = 0;
	cpsw_dev->tx_pipe.dma_queue = dma_get_tx_queue(cpsw_dev->tx_pipe.dma_channel);
	cpsw_dev->tx_pipe.dma_poll_threshold = cpsw_dev->tx_queue_depth / 2;
	atomic_set(&cpsw_dev->tx_pipe.dma_poll_count,
		   cpsw_dev->tx_pipe.dma_poll_threshold);

	netcp_register_txhook(cpsw_dev->module.priv,
			      CPSW_TXHOOK_ORDER, cpsw_tx_hook, cpsw_dev);

	dev_dbg(cpsw_dev->dev, "opened TX channel: %p\n",
		cpsw_dev->tx_pipe.dma_channel);

	return 0;
}

static void cpsw_close_txchan(struct cpsw_priv *cpsw_dev)
{
	struct netcp_tx_pipe *tx_pipe = &cpsw_dev->tx_pipe;

	dmaengine_pause(tx_pipe->dma_channel);

	netcp_unregister_txhook(cpsw_dev->module.priv, CPSW_TXHOOK_ORDER,
				cpsw_tx_hook, cpsw_dev);

	if (tx_pipe->dma_channel) {
		dma_release_channel(tx_pipe->dma_channel);
		tx_pipe->dma_channel = NULL;
	}
}

static int cpsw_open(struct netcp_module_data *data, struct net_device *ndev)
{
	struct cpsw_priv *cpsw_dev = cpsw_from_module(data);
	struct cpsw_ale_params		ale_params;
	int i, ret = 0;
	u32 reg;

	cpsw_dev->cpgmac = clk_get(cpsw_dev->dev, "clk_cpgmac");
	if (IS_ERR(cpsw_dev->cpgmac)) {
		dev_err(cpsw_dev->dev, "unable to get Keystone CPGMAC clock\n");
		return -EBUSY;
	}
	else
		clk_prepare_enable(cpsw_dev->cpgmac);

	reg = __raw_readl(&cpsw_dev->regs->id_ver);

	dev_info(cpsw_dev->dev, "initializing cpsw version %d.%d (%d) "
		 "SGMII identification value 0x%x\n",
		 CPSW_MAJOR_VERSION(reg), CPSW_MINOR_VERSION(reg),
		 CPSW_RTL_VERSION(reg), CPSW_SGMII_IDENT(reg));

	cpsw_dev->ndev = ndev;

	ret = cpsw_open_txchan(cpsw_dev);
	if (ret)
		return ret;

	memset(&ale_params, 0, sizeof(ale_params));

	ale_params.dev			= cpsw_dev->dev;
	ale_params.ale_regs		= (void *)((u32)priv->ale_reg);
	ale_params.ale_ageout		= cpsw_dev->ale_ageout;
	ale_params.ale_entries		= cpsw_dev->ale_entries;
	ale_params.ale_ports		= cpsw_dev->ale_ports;

	cpsw_dev->ale = cpsw_ale_create(&ale_params);
	if (!cpsw_dev->ale) {
		dev_err(cpsw_dev->dev, "error initializing ale engine\n");
		ret = -ENODEV;
		goto out;
	} else
		dev_info(cpsw_dev->dev, "Created a cpsw ale engine\n");

	cpsw_dev->slaves = kzalloc(sizeof(struct cpsw_slave) *
				   cpsw_dev->num_slaves, GFP_KERNEL);

	if (!cpsw_dev->slaves) {
		ret = -EBUSY;
		goto clean_ale;
	}

	for (i = 0; i < cpsw_dev->num_slaves; i++) {
		cpsw_dev->slaves[i].slave_num = i;
		cpsw_dev->slaves[i].link_interface = cpsw_dev->link[i];
	}

	memcpy(cpsw_dev->mac_addr, ndev->dev_addr, ETH_ALEN);

	for_each_slave(cpsw_dev, cpsw_slave_init, cpsw_dev);

	for_each_slave(cpsw_dev, cpsw_slave_stop, cpsw_dev);

	/* Serdes init */
	serdes_init();

	/* initialize host and slave ports */
	cpsw_init_host_port(cpsw_dev);	

	/* disable priority elevation and enable statistics on all ports */
	__raw_writel(0, &cpsw_dev->regs->ptype);

	/* Control register */
	__raw_writel(CPSW_CTL_P0_ENABLE | CPSW_CTL_VLAN_AWARE,
			&cpsw_dev->regs->control);

	/* All statistics enabled by default */
	__raw_writel(CPSW_REG_VAL_STAT_ENABLE_ALL,
		     &cpsw_dev->regs->stat_port_en);

	for_each_slave(cpsw_dev, cpsw_slave_open, cpsw_dev);

	init_timer(&cpsw_dev->timer);
	cpsw_dev->timer.data		= (unsigned long)cpsw_dev;
	cpsw_dev->timer.function	= cpsw_timer;
	cpsw_dev->timer.expires		= jiffies + CPSW_TIMER_INTERVAL;
	add_timer(&cpsw_dev->timer);

	return 0;

clean_ale:
	cpsw_ale_destroy(cpsw_dev->ale);
out:
	cpsw_close_txchan(cpsw_dev);
	return ret;
}

static int cpsw_close(struct netcp_module_data *data)
{
	struct cpsw_priv *cpsw_dev = cpsw_from_module(data);

	del_timer_sync(&cpsw_dev->timer);

	cpsw_ale_stop(cpsw_dev->ale);
	
	for_each_slave(cpsw_dev, cpsw_slave_stop, cpsw_dev);

	if (cpsw_dev->cpgmac) {
		clk_disable_unprepare(cpsw_dev->cpgmac);
		clk_put(cpsw_dev->cpgmac);
	}

	kfree(cpsw_dev->slaves);
	cpsw_dev->cpgmac = NULL;

	return 0;
}

static int cpsw_remove(struct netcp_module_data *data)
{
	struct cpsw_priv *cpsw_dev = cpsw_from_module(data);

	cpsw_ale_destroy(cpsw_dev->ale);

	iounmap(cpsw_dev->ss_regs);
	kfree(cpsw_dev);
	cpsw_dev = NULL;

	return 0;
}

static int init_slave(struct cpsw_priv *cpsw_dev,
		      struct device_node *node, int slave_num)
{
	int ret = 0;

	ret = of_property_read_u32(node, "link-interface",
				   &cpsw_dev->link[slave_num]);
	if (ret < 0) {
		dev_err(cpsw_dev->dev,
			"missing link-interface value"
			"defaulting to mac-phy link\n");
		cpsw_dev->link[slave_num] = 1;
	}

	return 0;
}


static struct netcp_module_data *cpsw_probe(struct device *dev,
					    struct device_node *node)
{
	struct cpsw_priv *cpsw_dev;
	struct device_node *slaves, *slave;
	void __iomem *regs;
	int slave_num = 0;
	int ret = 0;

	cpsw_dev = devm_kzalloc(dev, sizeof(struct cpsw_priv), GFP_KERNEL);
	if (!cpsw_dev) {
		dev_err(dev, "memory allocation failed\n");
		ret = -ENOMEM;
		goto exit;
	}

	if (!node) {
		dev_err(dev, "device tree info unavailable\n");
		ret = -ENODEV;
		goto exit;
	}

	cpsw_dev->dev = dev;
	
	priv = cpsw_dev;

	regs = ioremap(TCI6614_SS_BASE, 0xf00);
	BUG_ON(!regs);

	ret = of_property_read_string(node, "tx-channel", &cpsw_dev->tx_pipe.dma_chan_name);
	if (ret < 0) {
		dev_err(dev, "missing \"tx-channel\" parameter, err %d\n", ret);
		cpsw_dev->tx_pipe.dma_chan_name = "nettx";
	}

	ret = of_property_read_u32(node, "tx_queue_depth", &cpsw_dev->tx_queue_depth);
	if (ret < 0) {
		dev_err(dev, "missing tx_queue_depth parameter, err %d\n", ret);
		cpsw_dev->tx_queue_depth = 32;
	}
	dev_dbg(dev, "tx_queue_depth %u\n", cpsw_dev->tx_queue_depth);

	ret = of_property_read_u32(node, "sgmii_module_ofs",
				   &cpsw_dev->sgmii_module_ofs);
	if (ret < 0)
		dev_err(dev, "missing sgmii module offset, err %d\n", ret);
	
	ret = of_property_read_u32(node, "switch_module_ofs",
				   &cpsw_dev->switch_module_ofs);
	if (ret < 0)
		dev_err(dev, "missing switch module offset, err %d\n", ret);

	ret = of_property_read_u32(node, "host_port_reg_ofs",
				   &cpsw_dev->host_port_reg_ofs);
	if (ret < 0)
		dev_err(dev, "missing host port reg offset, err %d\n", ret);

	ret = of_property_read_u32(node, "slave_reg_ofs",
				   &cpsw_dev->slave_reg_ofs);
	if (ret < 0)
		dev_err(dev, "missing slave reg offset, err %d\n", ret);

	ret = of_property_read_u32(node, "sliver_reg_ofs",
				   &cpsw_dev->sliver_reg_ofs);
	if (ret < 0)
		dev_err(dev, "missing sliver reg offset, err %d\n", ret);

	ret = of_property_read_u32(node, "hw_stats_reg_ofs",
				   &cpsw_dev->hw_stats_reg_ofs);
	if (ret < 0)
		dev_err(dev, "missing hw stats reg offset, err %d\n", ret);

	ret = of_property_read_u32(node, "ale_reg_ofs",
				   &cpsw_dev->ale_reg_ofs);
	if (ret < 0)
		dev_err(dev, "missing ale reg offset, err %d\n", ret);


	ret = of_property_read_u32(node, "num_slaves", &cpsw_dev->num_slaves);
	if (ret < 0) {
		dev_err(dev, "missing num_slaves parameter, err %d\n", ret);
		cpsw_dev->num_slaves = 2;
	}

	ret = of_property_read_u32(node, "ale_ageout", &cpsw_dev->ale_ageout);
	if (ret < 0) {
		dev_err(dev, "missing ale_ageout parameter, err %d\n", ret);
		cpsw_dev->ale_ageout = 10;
	}

	ret = of_property_read_u32(node, "ale_entries", &cpsw_dev->ale_entries);
	if (ret < 0) {
		dev_err(dev, "missing ale_entries parameter, err %d\n", ret);
		cpsw_dev->ale_entries = 1024;
	}

	ret = of_property_read_u32(node, "ale_ports", &cpsw_dev->ale_ports);
	if (ret < 0) {
		dev_err(dev, "missing ale_ports parameter, err %d\n", ret);
		cpsw_dev->ale_ports = 2;
	}

	cpsw_dev->ss_regs = regs;
	cpsw_dev->sgmii_port_regs	= regs + cpsw_dev->sgmii_module_ofs;
	cpsw_dev->regs = regs + cpsw_dev->switch_module_ofs;
	cpsw_dev->host_port_regs = regs + cpsw_dev->host_port_reg_ofs;
	cpsw_dev->hw_stats[0] = regs + cpsw_dev->hw_stats_reg_ofs;
	cpsw_dev->hw_stats[1] = regs + cpsw_dev->hw_stats_reg_ofs + 0x100;
	cpsw_dev->ale_reg	  = regs + cpsw_dev->ale_reg_ofs;

	cpsw_dev->host_port = 0;
	cpsw_dev->rx_packet_max = 9500;

	dev_info(dev, "num_slaves = %d\n", cpsw_dev->num_slaves);
	dev_info(dev, "ale_ageout = %d\n", cpsw_dev->ale_ageout);
	dev_info(dev, "ale_entries = %d\n", cpsw_dev->ale_entries);
	dev_info(dev, "ale_ports = %d\n", cpsw_dev->ale_ports);

	slaves = of_find_child_by_name(node, "slaves");
	if (!slaves) {
		dev_err(dev, "could not find slaves\n");
		ret = -ENODEV;
		goto exit;
	}

	for_each_child_of_node(slaves, slave) {
			init_slave(cpsw_dev, slave, slave_num);
			slave_num++;
	}

	of_node_put(slaves);
	cpsw_dev->module.open		= cpsw_open;
	cpsw_dev->module.close		= cpsw_close;
	cpsw_dev->module.remove		= cpsw_remove;
	cpsw_dev->module.add_mcast	= cpsw_add_mcast;
	cpsw_dev->module.add_ucast	= cpsw_add_ucast;
	cpsw_dev->module.add_vid	= cpsw_add_vid;
	cpsw_dev->module.del_vid	= cpsw_del_vid;

	return cpsw_to_module(cpsw_dev);
exit:
	return NULL;
}

static struct netcp_module cpsw_module = {
	.name	= "keystone-cpsw",
	.owner	= THIS_MODULE,
	.probe	= cpsw_probe,
};

static int __init keystone_cpsw_init(void)
{
	return netcp_register_module(&cpsw_module);
}
subsys_initcall(keystone_cpsw_init);

static void __exit keystone_cpsw_exit(void)
{
	netcp_unregister_module(&cpsw_module);
}
module_exit(keystone_cpsw_exit);

MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Sandeep Paulraj <s-paulraj@ti.com>");
MODULE_DESCRIPTION("CPSW driver for Keystone devices");
