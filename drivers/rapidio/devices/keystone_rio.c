/*
 * Copyright (C) 2010, 2011, 2012 Texas Instruments Incorporated
 * Authors: Aurelien Jacquiot <a-jacquiot@ti.com>
 *
 * Copyright (C) 2012 Texas Instruments Incorporated
 * WingMan Kwok <w-kwok2@ti.com>
 * - Updated for support on TI KeyStone platform.
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
#include <linux/of.h>
#include <linux/clk.h>
#include <linux/timer.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/platform_device.h>
#include <linux/interrupt.h>
#include <linux/dma-mapping.h>
#include <linux/delay.h>
#include <linux/hardirq.h>
#include <linux/rio.h>
#include <linux/rio_drv.h>
#include <linux/keystone-dma.h>
#include "keystone_rio.h"

#define DRIVER_VER	"v1.1"

/*
 * Main KeyStone RapidIO driver data
 */
struct keystone_rio_data {
	struct device		*dev;

	struct clk		*clk;
	struct completion	lsu_completion;
	struct mutex		lsu_lock;
	u32			rio_pe_feat;

	u32			ports_registering;
	u32			port_chk_cnt;
	struct work_struct	port_chk_task;
	struct timer_list	timer;
	struct tasklet_struct	task;
	unsigned long		rxu_map_bitmap[2];
#ifdef CONFIG_RIONET
	u32			rionet_started;
#endif

	struct dma_chan		*tx_channel;
	const char		*tx_chan_name;
	u32			tx_queue_depth;

	struct keystone_rio_mbox_info  tx_mbox[KEYSTONE_RIO_MAX_MBOX];
	struct keystone_rio_mbox_info  rx_mbox[KEYSTONE_RIO_MAX_MBOX];

	struct keystone_rio_rx_chan_info  rx_channels[KEYSTONE_RIO_MAX_MBOX];

	u32 __iomem					*jtagid_reg;
	u32 __iomem					*serdes_sts_reg;
	struct keystone_srio_serdes_regs __iomem	*serdes_regs;
	struct keystone_rio_regs	 __iomem	*regs;

	struct keystone_rio_car_csr_regs __iomem	*car_csr_regs;
	struct keystone_rio_serial_port_regs __iomem	*serial_port_regs;
	struct keystone_rio_err_mgmt_regs __iomem	*err_mgmt_regs;
	struct keystone_rio_phy_layer_regs __iomem	*phy_regs;
	struct keystone_rio_transport_layer_regs __iomem	*transport_regs;
	struct keystone_rio_pkt_buf_regs __iomem	*pkt_buf_regs;
	struct keystone_rio_evt_mgmt_regs __iomem	*evt_mgmt_regs;
	struct keystone_rio_port_write_regs __iomem	*port_write_regs;
	struct keystone_rio_link_layer_regs __iomem	*link_regs;
	struct keystone_rio_fabric_regs __iomem		*fabric_regs;
	u32						car_csr_regs_base;

	struct keystone_rio_board_controller_info	board_rio_cfg;
};

/*---------------------------- Direct I/O -------------------------------*/

static u32 keystone_rio_dio_get_lsu_cc(u32 lsu_id, u8 ltid, u8 *lcb,
				struct keystone_rio_data *krio_priv)
{
	u32 idx;
	u32 shift;
	u32 value;
	u32 cc;
	/* lSU shadow register status mapping */
	u32 lsu_index[8] = { 0, 9, 15, 20, 24, 33, 39, 44 };

	/* Compute LSU stat index from LSU id and LTID */
	idx   = (lsu_index[lsu_id] + ltid) >> 3;
	shift = ((lsu_index[lsu_id] + ltid) & 0x7) << 2;

	/* Get completion code and context */
	value  = __raw_readl(&(krio_priv->regs->lsu_stat_reg[idx]));
	cc     = (value >> (shift + 1)) & 0x7;
	*lcb   = (value >> shift) & 0x1;

	return cc;
}

/*---------------------- Maintenance Request Management  ---------------------*/

/**
 * maint_request - Perform a maintenance request
 * @index: ID of the RapidIO interface
 * @destid: destination ID of target device
 * @hopcount: hopcount for this request
 * @offset: offset in the RapidIO configuration space
 * @buff: dma address of the data on the host
 * @buff_len: length of the data
 * @size: 1 for 16bit, 0 for 8bit ID size
 * @type: packet type
 *
 * Returns %0 on success or %-EINVAL, %-EIO, %-EAGAIN or %-EBUSY on failure.
 */
static inline int maint_request(int index, u32 dest_id, u8 hopcount,
		u32 offset, dma_addr_t buff, int buff_len,
		u16 size, u16 type, struct keystone_rio_data *krio_priv)
{
	unsigned int count;
	unsigned int status = 0;
	unsigned int res    = 0;
	u8           context;
	u8           ltid;

	mutex_lock(&krio_priv->lsu_lock);

	/* Check is there is space in the LSU shadow reg and that it is free */
	count = 0;
	while (1) {
		status = __raw_readl(&(krio_priv->regs->lsu_reg[0].busy_full));
		if (((status & KEYSTONE_RIO_LSU_FULL_MASK) == 0x0)
		    && ((status & KEYSTONE_RIO_LSU_BUSY_MASK) == 0x0))
			break;
		count++;

		if (count >= KEYSTONE_RIO_TIMEOUT_CNT) {
			dev_dbg(krio_priv->dev,
				"no LSU available, status = 0x%x\n", status);
			res = -EIO;
			goto out;
		}
		ndelay(1000);
	}

	/* Get LCB and LTID, LSU reg 6 is already read */
	context = (status >> 4) & 0x1;
	ltid    = status & 0xf;

	/* LSU Reg 0 - MSB of RapidIO address */
	__raw_writel(0, &(krio_priv->regs->lsu_reg[0].addr_msb));

	/* LSU Reg 1 - LSB of destination */
	__raw_writel(offset, &(krio_priv->regs->lsu_reg[0].addr_lsb_cfg_ofs));

	/* LSU Reg 2 - source address */
	__raw_writel(buff, &(krio_priv->regs->lsu_reg[0].dsp_addr));

	/* LSU Reg 3 - byte count */
	__raw_writel(buff_len,
		&(krio_priv->regs->lsu_reg[0].dbell_val_byte_cnt));

	/* LSU Reg 4 - */
	__raw_writel(((index << 8)
			| (KEYSTONE_RIO_LSU_PRIO << 4)
			| (size ? (1 << 10) : 0)
			| ((u32) dest_id << 16)),
		&(krio_priv->regs->lsu_reg[0].destid));

	/* LSU Reg 5 */
	__raw_writel(((hopcount & 0xff) << 8) | (type & 0xff),
		&(krio_priv->regs->lsu_reg[0].dbell_info_fttype));

	/* Retrieve our completion code */
	count = 0;
	res   = 0;
	while (1) {
		u8 lcb;
		status = keystone_rio_dio_get_lsu_cc(0, ltid, &lcb, krio_priv);
		if (lcb == context)
			break;
		count++;
		if (count >= KEYSTONE_RIO_TIMEOUT_CNT) {
			dev_dbg(krio_priv->dev,
				"timeout %d, ltid = %d, context = %d, "
				"lcb = %d, cc = %d\n",
				count, ltid, context, lcb, status);
			res = -EIO;
			break;
		}
		ndelay(1000);
	}
out:
	mutex_unlock(&krio_priv->lsu_lock);

	if (res)
		return res;

	if (status)
		dev_err(krio_priv->dev, "transfer error = 0x%x\n", status);

	switch (status) {
	case KEYSTONE_RIO_LSU_CC_TIMEOUT:
	case KEYSTONE_RIO_LSU_CC_XOFF:
	case KEYSTONE_RIO_LSU_CC_ERROR:
	case KEYSTONE_RIO_LSU_CC_INVALID:
	case KEYSTONE_RIO_LSU_CC_DMA:
		return -EIO;
		break;
	case KEYSTONE_RIO_LSU_CC_RETRY:
		return -EBUSY;
		break;
	case KEYSTONE_RIO_LSU_CC_CANCELED:
		return -EAGAIN;
		break;
	default:
		break;
	}
	return 0;
}
/*------------------------- RapidIO hw controller setup ---------------------*/

/**
 * keystone_rio_hw_init - Configure a RapidIO controller
 * @mode: serdes configuration
 * @hostid: device id of the host
 */
static void keystone_rio_hw_init(u32 mode, struct keystone_rio_data *krio_priv)
{
	u32 val;
	u32 block;
	u32 port;
	struct keystone_serdes_config *serdes_config
		= &(krio_priv->board_rio_cfg.serdes_config[mode]);

	/* Set sRIO out of reset */
	__raw_writel(0x00000011, &krio_priv->regs->pcr);

	/* Clear BOOT_COMPLETE bit (allowing write) */
	__raw_writel(0x00000000, &krio_priv->regs->per_set_cntl);

	/* Enable blocks */
	__raw_writel(1, &krio_priv->regs->gbl_en);
	for (block = 0; block <= KEYSTONE_RIO_BLK_NUM; block++)
		__raw_writel(1, &(krio_priv->regs->blk[block].enable));

	/* Set control register 1 configuration */
	__raw_writel(0x00000000, &krio_priv->regs->per_set_cntl1);

	/* Set Control register */
	__raw_writel(serdes_config->cfg_cntl, &krio_priv->regs->per_set_cntl);

	/* Serdes main configuration */
	__raw_writel(serdes_config->serdes_cfg_pll,
		&krio_priv->serdes_regs->pll);

	/* Per-port SerDes configuration */
	for (port = 0; port < KEYSTONE_RIO_MAX_PORT; port++) {
		__raw_writel(serdes_config->rx_chan_config[port],
				&krio_priv->serdes_regs->channel[port].rx);
		__raw_writel(serdes_config->tx_chan_config[port],
				&krio_priv->serdes_regs->channel[port].tx);
	}

	/* Check for RIO SerDes PLL lock */
	do {
		val = __raw_readl(krio_priv->serdes_sts_reg);
    	} while ((val & 0x1) != 0x1);

	/* Set prescalar for ip_clk */
	__raw_writel(serdes_config->prescalar_srv_clk,
			&krio_priv->link_regs->prescalar_srv_clk);

	/* Peripheral-specific configuration and capabilities */
	__raw_writel(KEYSTONE_RIO_DEV_ID_VAL,
		&krio_priv->car_csr_regs->dev_id);
	__raw_writel(KEYSTONE_RIO_DEV_INFO_VAL,
		&krio_priv->car_csr_regs->dev_info);
	__raw_writel(KEYSTONE_RIO_ID_TI,
		&krio_priv->car_csr_regs->assembly_id);
	__raw_writel(KEYSTONE_RIO_EXT_FEAT_PTR,
		&krio_priv->car_csr_regs->assembly_info);

	krio_priv->rio_pe_feat = RIO_PEF_PROCESSOR
		| RIO_PEF_CTLS
		| KEYSTONE_RIO_PEF_FLOW_CONTROL
		| RIO_PEF_EXT_FEATURES 
		| RIO_PEF_ADDR_34
		| RIO_PEF_STD_RT
		| RIO_PEF_INB_DOORBELL
		| RIO_PEF_INB_MBOX;

	__raw_writel(krio_priv->rio_pe_feat,
		&krio_priv->car_csr_regs->pe_feature);

	__raw_writel(KEYSTONE_RIO_MAX_PORT << 8,
		&krio_priv->car_csr_regs->sw_port);

	__raw_writel((RIO_SRC_OPS_READ
		       | RIO_SRC_OPS_WRITE
		       | RIO_SRC_OPS_STREAM_WRITE
		       | RIO_SRC_OPS_WRITE_RESPONSE
		       | RIO_SRC_OPS_DATA_MSG
		       | RIO_SRC_OPS_DOORBELL
		       | RIO_SRC_OPS_ATOMIC_TST_SWP
		       | RIO_SRC_OPS_ATOMIC_INC
		       | RIO_SRC_OPS_ATOMIC_DEC
		       | RIO_SRC_OPS_ATOMIC_SET
		       | RIO_SRC_OPS_ATOMIC_CLR
		       | RIO_SRC_OPS_PORT_WRITE),
		&krio_priv->car_csr_regs->src_op);

	__raw_writel((RIO_DST_OPS_READ
		       | RIO_DST_OPS_WRITE
		       | RIO_DST_OPS_STREAM_WRITE
		       | RIO_DST_OPS_WRITE_RESPONSE
		       | RIO_DST_OPS_DATA_MSG
		       | RIO_DST_OPS_DOORBELL
		       | RIO_DST_OPS_PORT_WRITE),
		&krio_priv->car_csr_regs->dest_op);

	__raw_writel(RIO_PELL_ADDR_34,
		&krio_priv->car_csr_regs->pe_logical_ctl);

	val = (((KEYSTONE_RIO_SP_HDR_NEXT_BLK_PTR & 0xffff) << 16) | 
		KEYSTONE_RIO_SP_HDR_EP_REC_ID);
	__raw_writel(val, &krio_priv->serial_port_regs->sp_maint_blk_hdr);

	/* clear high bits of local config space base addr */
	__raw_writel(0x00000000, &krio_priv->car_csr_regs->local_cfg_hbar);

	/* set local config space base addr */
	__raw_writel(0x00520000, &krio_priv->car_csr_regs->local_cfg_bar);

	/* Enable HOST BIT(31) & MASTER_ENABLE BIT(30) bits */
	__raw_writel(0xc0000000, &krio_priv->serial_port_regs->sp_gen_ctl);

	/* set link timeout value */
	__raw_writel(0x000FFF00,
		&krio_priv->serial_port_regs->sp_link_timeout_ctl);

	/* set response timeout value */
	__raw_writel(0x000FFF00,
		&krio_priv->serial_port_regs->sp_rsp_timeout_ctl);

	/* allows SELF_RESET and PWDN_PORT resets to clear stcky reg bits */
	__raw_writel(0x00000001, &krio_priv->link_regs->reg_rst_ctl);

	/* Set error detection mode */
	/* clear all errors */
	__raw_writel(0x00000000, &krio_priv->err_mgmt_regs->err_det);
	/* enable all error detection */
	__raw_writel(0x00000000, &krio_priv->err_mgmt_regs->err_en);

	/* set err det block header */
	val = (((KEYSTONE_RIO_ERR_HDR_NEXT_BLK_PTR & 0xffff) << 16) |
		KEYSTONE_RIO_ERR_EXT_FEAT_ID);
	__raw_writel(val, &krio_priv->err_mgmt_regs->err_report_blk_hdr);

	/* clear msb of err catptured addr reg */ 
	__raw_writel(0x00000000, &krio_priv->err_mgmt_regs->h_addr_capt);

	/* clear lsb of err catptured addr reg */ 
	__raw_writel(0x00000000, &krio_priv->err_mgmt_regs->addr_capt);

	/* clear err catptured source and dest devID reg */ 
	__raw_writel(0x00000000, &krio_priv->err_mgmt_regs->id_capt);

	/* clear err catptured packet info */ 
	__raw_writel(0x00000000, &krio_priv->err_mgmt_regs->ctrl_capt);

	__raw_writel(0x41004141, &krio_priv->phy_regs->phy_sp[0].__rsvd[3]);

	/* Force all writes to finish */
	val = __raw_readl(&krio_priv->err_mgmt_regs->ctrl_capt);
}

/**
 * keystone_rio_start - Start RapidIO controller
 */
static void keystone_rio_start(struct keystone_rio_data *krio_priv)
{
	u32 val;

	/* Set PEREN bit to enable logical layer data flow */	
	val = (KEYSTONE_RIO_PER_EN | KEYSTONE_RIO_PER_FREE);
	__raw_writel(val, &krio_priv->regs->pcr);

	/* Set BOOT_COMPLETE bit */
	val = __raw_readl(&krio_priv->regs->per_set_cntl);
	__raw_writel(val | KEYSTONE_RIO_BOOT_COMPLETE, 
				&krio_priv->regs->per_set_cntl);
}

static int
keystone_rio_test_link(struct keystone_rio_data *krio_priv)
{
	u32 *tbuf;
	int res;
	dma_addr_t dma;
	struct device *dev = krio_priv->dev;
	size_t align_len = L1_CACHE_ALIGN(4);

	tbuf = kzalloc(align_len, GFP_KERNEL);
	if (!tbuf)
		return -ENOMEM;

	dma = dma_map_single(dev, tbuf, 4, DMA_FROM_DEVICE);

	/* Send a maint req to test the link */
	res = maint_request(0, 0xff, 0, 0, dma, 4,
			krio_priv->board_rio_cfg.size,
			KEYSTONE_RIO_PACKET_TYPE_MAINT_R,
			krio_priv);

	dma_unmap_single(dev, dma, 4, DMA_FROM_DEVICE);

	kfree(tbuf);

	return res;
}

/**
 * keystone_rio_port_status - Return if the port is OK or not
 * @port: index of the port
 *
 * Return %0 if the port is ready or %-EIO on failure.
 */
static int keystone_rio_port_status(int port,
		struct keystone_rio_data *krio_priv)
{
	unsigned int count, value, portok;
	int res = 0;

	count  = 0;
	portok = 0;

	if (port >= KEYSTONE_RIO_MAX_PORT)
		return -EINVAL;

	/* Check port status */
	value = __raw_readl(&(krio_priv->serial_port_regs->sp[port].err_stat));

	if ((value & RIO_PORT_N_ERR_STS_PORT_OK) != 0) {
		res = keystone_rio_test_link(krio_priv);
		if (0 != res)
			return -EIO;
		else
			return 0; /* port must be solid OK */
	} else
		return -EIO;
}

/**
 * keystone_rio_port_init - Configure a RapidIO port
 * @port: index of the port to configure
 * @mode: serdes configuration
 */
static int keystone_rio_port_init(u32 port, u32 mode,
			struct keystone_rio_data *krio_priv)
{
	u32 path_mode =
		krio_priv->board_rio_cfg.serdes_config[mode].path_mode[port];

	if (port >= KEYSTONE_RIO_MAX_PORT)
		return -EINVAL;

	/* Send both link request and PNA control symbols
	   (this will clear error states) */
	__raw_writel(0x2003f044,
		&krio_priv->phy_regs->phy_sp[port].long_cs_tx1);

	/* Disable packet forwarding */
	__raw_writel(0xffffffff, &(krio_priv->regs->pkt_fwd_cntl[port].pf_16b));
	__raw_writel(0x0003ffff, &(krio_priv->regs->pkt_fwd_cntl[port].pf_8b));

	/* Silence and discovery timers */
	__raw_writel(0x20000000,
		&(krio_priv->phy_regs->phy_sp[port].silence_timer));
	__raw_writel(0x20000000,
		&(krio_priv->phy_regs->phy_sp[port].discovery_timer));

	/* Enable port in input and output */
	__raw_writel(0x600000, &(krio_priv->serial_port_regs->sp[port].ctl));

	/* Program channel allocation to ports (1x, 2x or 4x) */
	__raw_writel(path_mode, &(krio_priv->phy_regs->phy_sp[port].path_ctl));

	return 0;
}

/**
 * keystone_rio_port_activate - Start using a RapidIO port
 * @port: index of the port to configure
 */
static int keystone_rio_port_activate(u32 port,
		struct keystone_rio_data *krio_priv)
{
	u32 val;

	/* Enable interrupt for reset request */
	val = __raw_readl(&(krio_priv->evt_mgmt_regs->evt_mgmt_rst_int_en));
	__raw_writel(val | (1 << port),
		&(krio_priv->evt_mgmt_regs->evt_mgmt_rst_int_en));

	/* Enable all PLM interrupts */
	__raw_writel(0xffffffff, 
		&(krio_priv->phy_regs->phy_sp[port].int_enable));
	__raw_writel(1, &(krio_priv->phy_regs->phy_sp[port].all_int_en));

	/* Enable all errors */
	__raw_writel(0xffffffff,
		&(krio_priv->err_mgmt_regs->sp_err[port].rate_en));

	/* Cleanup port error status */
	__raw_writel(KEYSTONE_RIO_PORT_ERROR_MASK, 
		&(krio_priv->serial_port_regs->sp[port].err_stat));
	__raw_writel(0, &(krio_priv->err_mgmt_regs->sp_err[port].det));

	/* Enable promiscuous */
	__raw_writel(0x00309000,
		&(krio_priv->transport_regs->transport_sp[port].control));

	/* Enable Port-write reception capture */
	__raw_writel(0, &(krio_priv->port_write_regs->port_wr_rx_capt[port]));

	return 0;
}

/*------------------------- Configuration space mngt  ----------------------*/

/**
 * keystone_local_config_read - Generate a KeyStone local config space read
 * @mport: RapidIO master port info
 * @index: ID of RapidIO interface
 * @offset: Offset into configuration space
 * @len: Length (in bytes) of the maintenance transaction
 * @data: Value to be read into
 *
 * Generates a KeyStone local configuration space read. Returns %0 on
 * success or %-EINVAL on failure.
 */
static int keystone_local_config_read(struct rio_mport *mport, 
			      int index, u32 offset, int len, u32 * data)
{
	struct keystone_rio_data *krio_priv = mport->priv;

	/* 
	 * Workaround for rionet: the processing element features must content
	 * RIO_PEF_INB_MBOX and RIO_PEF_INB_DOORBELL bits that cannot be set on
	 * KeyStone hardware. So cheat the read value in this case...
	 */
	if (unlikely(offset == RIO_PEF_CAR))
		*data = krio_priv->rio_pe_feat;
	else
		*data = __raw_readl((void __iomem *)
				(krio_priv->car_csr_regs_base + offset));
	
	dev_dbg(krio_priv->dev,
		"index %d offset 0x%x data 0x%x\n", index, offset, *data);
	return 0;
}

/**
 * keystone_local_config_write - Generate a KeyStone local config space write
 * @mport: RapidIO master port info
 * @index: ID of RapidIO interface
 * @offset: Offset into configuration space
 * @len: Length (in bytes) of the maintenance transaction
 * @data: Value to be written
 *
 * Generates a KeyStone local configuration space write. Returns %0 on
 * success or %-EINVAL on failure.
 */
static int keystone_local_config_write(struct rio_mport *mport, 
		       int index, u32 offset, int len, u32 data)
{
	struct keystone_rio_data *krio_priv = mport->priv;

	dev_dbg(krio_priv->dev,
		"index %d offset 0x%x data 0x%x\n", index, offset, data);
	__raw_writel(data, 
		(void __iomem *)(krio_priv->car_csr_regs_base + offset));

	return 0;
}

/**
 * keystone_rio_config_read - Generate a KeyStone read maintenance transaction
 * @mport: RapidIO master port info
 * @index: ID of RapidIO interface
 * @destid: Destination ID of transaction
 * @hopcount: Number of hops to target device
 * @offset: Offset into configuration space
 * @len: Length (in bytes) of the maintenance transaction
 * @val: Location to be read into
 *
 * Generates a KeyStone read maintenance transaction. Returns %0 on
 * success or %-EINVAL on failure.
 */
static int
keystone_rio_config_read(struct rio_mport *mport, int index, u16 destid,
			 u8 hopcount, u32 offset, int len, u32* val)
{
	u32* tbuf;
	int res;
	dma_addr_t dma;
	struct device *dev = ((struct keystone_rio_data *)(mport->priv))->dev;
	size_t align_len = L1_CACHE_ALIGN(len);

	tbuf = (u32*) kzalloc(align_len, GFP_KERNEL);
	if (!tbuf)
		return -ENOMEM;

	dma = dma_map_single(dev, tbuf, len, DMA_FROM_DEVICE);

	res = maint_request(index, destid, hopcount, offset, dma, len,
			    mport->sys_size, KEYSTONE_RIO_PACKET_TYPE_MAINT_R,
			    (struct keystone_rio_data *)(mport->priv));

	dma_unmap_single(dev, dma, len, DMA_FROM_DEVICE);

	/* Taking care of byteswap */
	switch (len) {
        case 1:
	        *val = *((u8*) tbuf);
                break;
        case 2:
	        *val = ntohs(*((u16*) tbuf));
                break;
        default:
                *val = ntohl(*((u32*) tbuf));
                break;
        }

	kfree(tbuf);

	dev_dbg(dev,
		"index %d destid %d hopcount %d offset 0x%x "
		"len %d val 0x%x res %d\n",
		index, destid, hopcount, offset, len, *val, res);

	return res;
}

/**
 * keystone__rio_config_write - Generate a KeyStone write
 * maintenance transaction
 * @mport: RapidIO master port info
 * @index: ID of RapidIO interface
 * @destid: Destination ID of transaction
 * @hopcount: Number of hops to target device
 * @offset: Offset into configuration space
 * @len: Length (in bytes) of the maintenance transaction
 * @val: Value to be written
 *
 * Generates an KeyStone write maintenance transaction. Returns %0 on
 * success or %-EINVAL on failure.
 */
static int
keystone_rio_config_write(struct rio_mport *mport, int index, u16 destid,
			u8 hopcount, u32 offset, int len, u32 val)
{
	u32* tbuf;
	int res;
	dma_addr_t dma;
	struct device *dev = ((struct keystone_rio_data *)(mport->priv))->dev;
	size_t align_len = L1_CACHE_ALIGN(len);

	tbuf = (u32*) kzalloc(align_len, GFP_KERNEL);
	if (!tbuf)
		return -ENOMEM;

	/* Taking care of byteswap */
	switch (len) {
        case 1:
                *tbuf = ((u8) val);
                break;
        case 2:
                *tbuf = htons((u16) val);
                break;
        default:
                *tbuf = htonl((u32) val);
                break;
        }

	dma = dma_map_single(dev, tbuf, len, DMA_TO_DEVICE);

	res = maint_request(index, destid, hopcount, offset, dma, len,
			    mport->sys_size,
			    KEYSTONE_RIO_PACKET_TYPE_MAINT_W,
			    (struct keystone_rio_data *)(mport->priv));

	dma_unmap_single(dev, dma, len, DMA_TO_DEVICE);

	dev_dbg(dev,
		"index %d destid %d hopcount %d offset 0x%x "
		"len %d val 0x%x res %d\n",
		index, destid, hopcount, offset, len, val, res);

	kfree(tbuf);

	return res;
}

/*------------------------- Message passing management  ----------------------*/

static void keystone_rio_rx_complete(void *data)
{
	struct keystone_rio_packet *p_info = data;
	struct keystone_rio_data *krio_priv = p_info->priv;
	struct keystone_rio_rx_chan_info *krx_chan;
	struct keystone_rio_mbox_info *p_mbox;
	int mbox;
	u32 src_id, dest_id;

	src_id = ((p_info->psdata[0] & 0xffff0000) >> 16);
	dest_id = (p_info->psdata[0] & 0x0000ffff);
	mbox = (p_info->psdata[1] & 0x3f);
	p_info->mbox = mbox;

	krx_chan = &(krio_priv->rx_channels[mbox]);

	p_info->status = dma_async_is_tx_complete(krx_chan->dma_channel,
						  p_info->cookie, NULL, NULL);
	WARN_ON(p_info->status != DMA_SUCCESS && p_info->status != DMA_ERROR);

	p_mbox = &(krio_priv->rx_mbox[mbox]);
	p_mbox->p_info_temp = p_info;

	dev_dbg(krio_priv->dev,
		"Received message for mbox = %d, src=%d, dest=%d\n",
		mbox, src_id, dest_id);

	if (p_mbox->running) {
		/* Client callback (slot is not used) */
		p_mbox->port->inb_msg[p_mbox->id].mcback(p_mbox->port,
					p_mbox->dev_id, p_mbox->id, 0);
	}
}

static void keystone_rio_chan_work_handler(unsigned long data)
{
	struct keystone_rio_data *krio_priv = (struct keystone_rio_data *)data;
	struct keystone_rio_rx_chan_info *krx_chan;
	int i;

	for (i=0; i<KEYSTONE_RIO_MAX_MBOX; i++) {
		krx_chan = &(krio_priv->rx_channels[i]);
		if (krx_chan->dma_channel) {
			dma_poll(krx_chan->dma_channel, -1);
			dmaengine_resume(krx_chan->dma_channel);
		}
	}
}

static void keystone_rio_rx_notify(struct dma_chan *chan, void *arg)
{
	struct keystone_rio_data *krio_priv = arg;
	struct keystone_rio_rx_chan_info *krx_chan;
	int i;

	for (i=0; i<KEYSTONE_RIO_MAX_MBOX; i++) {
		krx_chan = &(krio_priv->rx_channels[i]);
		if (krx_chan->dma_channel)
			dmaengine_pause(krx_chan->dma_channel);
	}
	tasklet_schedule(&krio_priv->task);

	return;
}

/* Release a free receive buffer */
static void keystone_rio_rxpool_free(void *arg, unsigned q_num,
		unsigned bufsize, struct dma_async_tx_descriptor *desc)
{
	struct keystone_rio_rx_chan_info *krx_chan = arg;
	struct keystone_rio_data *krio_priv = krx_chan->priv;
	struct keystone_rio_packet *p_info = desc->callback_param;

	dma_unmap_sg(krio_priv->dev, &p_info->sg[2], 1, DMA_FROM_DEVICE);
	p_info->buff = NULL;
	kfree(p_info);

	return;
}

/* Allocate a free receive buffer */
static struct dma_async_tx_descriptor *keystone_rio_rxpool_alloc(void *arg,
		unsigned q_num, unsigned bufsize)
{
	struct keystone_rio_rx_chan_info *krx_chan = arg;
	struct keystone_rio_data *krio_priv = krx_chan->priv;
	struct dma_async_tx_descriptor *desc = NULL;
	struct keystone_rio_packet *p_info;
	u32 err = 0;

	if (krx_chan->buff_temp == NULL)
		/* No inb_buffer added */
		return NULL;

	/* Allocate a primary receive queue entry */
	p_info = kzalloc(sizeof(*p_info), GFP_ATOMIC);
	if (!p_info) {
		dev_err(krio_priv->dev, "packet alloc failed\n");
		return NULL;
	}
	p_info->priv = krio_priv;
	p_info->buff = krx_chan->buff_temp;

	sg_init_table(p_info->sg, KEYSTONE_RIO_SGLIST_SIZE);
	sg_set_buf(&p_info->sg[0], p_info->epib, sizeof(p_info->epib));
	sg_set_buf(&p_info->sg[1], p_info->psdata, sizeof(p_info->psdata));
	sg_set_buf(&p_info->sg[2], krx_chan->buff_temp,
			krx_chan->buffer_sizes[q_num]);

	krx_chan->buff_temp = NULL;

	p_info->sg_ents = 2 + dma_map_sg(krio_priv->dev, &p_info->sg[2],
					 1, DMA_FROM_DEVICE);

	if (p_info->sg_ents != 3) {
		dev_err(krio_priv->dev, "dma map failed\n");
		p_info->buff = NULL;
		kfree(p_info);
		return NULL;
	}

	desc = dmaengine_prep_slave_sg(krx_chan->dma_channel, p_info->sg,
				       p_info->sg_ents, DMA_DEV_TO_MEM,
				       DMA_HAS_EPIB | DMA_HAS_PSINFO);

	if (IS_ERR_OR_NULL(desc)) {
		dma_unmap_sg(krio_priv->dev, &p_info->sg[2],
			1, DMA_FROM_DEVICE);
		p_info->buff = NULL;
		kfree(p_info);
		err = PTR_ERR(desc);
		if (err != -ENOMEM) {
			dev_err(krio_priv->dev,
				"dma prep failed, error %d\n", err);
		}
		return NULL;
	}

	desc->callback_param = p_info;
	desc->callback = keystone_rio_rx_complete;
	p_info->cookie = desc->cookie;

	return desc;
}

static void keystone_rio_mp_inb_exit(int mbox,
		struct keystone_rio_data *krio_priv)
{
	struct keystone_rio_rx_chan_info *krx_chan;

	krx_chan = &(krio_priv->rx_channels[mbox]);

	if (!(krx_chan->dma_channel))
		return;

	dmaengine_pause(krx_chan->dma_channel);
	dma_release_channel(krx_chan->dma_channel);
	krx_chan->dma_channel = NULL;
	return;
}

static int keystone_rio_mp_inb_init(int mbox,
		struct keystone_rio_data *krio_priv)
{
	struct keystone_rio_rx_chan_info *krx_chan;
	struct dma_keystone_info config;
	dma_cap_mask_t mask;
	int err = -ENODEV;
	int i;

	dma_cap_zero(mask);
	dma_cap_set(DMA_SLAVE, mask);

	/* DMA RX channel */
	krx_chan = &(krio_priv->rx_channels[mbox]);
	krx_chan->priv = krio_priv;
	krx_chan->chan_num = mbox;
	krx_chan->dma_channel = 
		dma_request_channel_by_name(mask, krx_chan->name);
	if (IS_ERR_OR_NULL(krx_chan->dma_channel))
		goto fail;

	memset(&config, 0, sizeof(config));
	config.direction		= DMA_DEV_TO_MEM;
	config.scatterlist_size		= KEYSTONE_RIO_SGLIST_SIZE;
	config.rxpool_allocator		= keystone_rio_rxpool_alloc;
	config.rxpool_destructor	= keystone_rio_rxpool_free;
	config.rxpool_param		= krx_chan;
	config.rxpool_thresh_enable	= DMA_THRESH_NONE;

	for (i = 0; i < KEYSTONE_QUEUES_PER_CHAN &&
		    krx_chan->queue_depths[i] &&
		    krx_chan->buffer_sizes[i]; ++i) {
		config.rxpools[i].pool_depth  = krx_chan->queue_depths[i];
		config.rxpools[i].buffer_size = krx_chan->buffer_sizes[i];
		dev_dbg(krio_priv->dev, "rx_pool[%d] depth %d, size %d\n", i,
				config.rxpools[i].pool_depth,
				config.rxpools[i].buffer_size);
	}
	config.rxpool_count = i;

	err = dma_keystone_config(krx_chan->dma_channel, &config);
	if (err) {
		dev_err(krio_priv->dev,
			"Error configuring RX channel, err %d\n", err);
		goto fail;
	}

	tasklet_init(&krio_priv->task, keystone_rio_chan_work_handler,
		     (unsigned long) krio_priv);

	dma_set_notify(krx_chan->dma_channel,
			keystone_rio_rx_notify,
			krio_priv);

	krx_chan->flow_num = dma_get_rx_flow(krx_chan->dma_channel);
	krx_chan->queue_num = dma_get_rx_queue(krx_chan->dma_channel);

	dev_info(krio_priv->dev,
		"Opened rx channel: %p (mbox=%d, flow=%d, rx_q=%d)\n",
		krx_chan->dma_channel, mbox, krx_chan->flow_num,
		krx_chan->queue_num);

	return 0;

fail:
	if (krx_chan->dma_channel) {
		dma_release_channel(krx_chan->dma_channel);
		krx_chan->dma_channel = NULL;
	}
	return err;
}

static int keystone_rio_get_rxu_map(struct keystone_rio_data *krio_priv)
{
	int id;
	unsigned long bit_sz = 2 * 8 * sizeof(u32);

	id = find_first_zero_bit(&(krio_priv->rxu_map_bitmap[0]), bit_sz);
	if (id >= bit_sz)
		return -1;

	__set_bit(id, &(krio_priv->rxu_map_bitmap[0]));
	return id;
}

static void keystone_rio_free_rxu_map(int id,
		struct keystone_rio_data *krio_priv)
{
	clear_bit(id, &(krio_priv->rxu_map_bitmap[0]));
}

/**
 * keystone_rio_map_mbox - Map a mailbox to a given queue.
 * for both type 11 and type 9 packets.
 * @mbox: mailbox to map
 * @queue: associated queue number
 *
 * Returns %0 on success or %-ENOMEM on failure.
 */
static int keystone_rio_map_mbox(int mbox, 
	int queue, int flowid, int size,
	struct keystone_rio_data *krio_priv)
{
	struct keystone_rio_mbox_info *rx_mbox = &krio_priv->rx_mbox[mbox];
	u32 mapping_entry_low;
	u32 mapping_entry_high;
	u32 mapping_entry_qid;
	int i;

	/* Map the multi-segment mailbox to the corresponding Rx
	   queue for type 11 */
	mapping_entry_low = ((mbox & 0x1f) << 16)
 		| (0x3f000000); /* Given mailbox, all letters, srcid = 0 */

	mapping_entry_high = KEYSTONE_RIO_MAP_FLAG_SEGMENT /* multi-segment messaging */
		| KEYSTONE_RIO_MAP_FLAG_SRC_PROMISC
		| KEYSTONE_RIO_MAP_FLAG_DST_PROMISC;       /* promiscuous (don't care about src/dst id) */

	/* Set TT flag */
	if (size) {
		mapping_entry_high |= KEYSTONE_RIO_MAP_FLAG_TT_16;
	}

	/* QMSS/PktDMA mapping */
	mapping_entry_qid = (queue & 0x3fff) | (flowid << 16);

	i = keystone_rio_get_rxu_map(krio_priv);

	if (i < 0)
		return -ENOMEM;

	rx_mbox->rxu_map_id[0] = i;
	dev_dbg(krio_priv->dev, 
		"Using RXU map %d @ 0x%08x: mbox = %d,"
		" flowid = %d, queue = %d\n", 
		i, (u32)&(krio_priv->regs->rxu_map[i]), mbox, flowid, queue);

	__raw_writel(mapping_entry_low,
		&(krio_priv->regs->rxu_map[i].ltr_mbox_src));

	__raw_writel(mapping_entry_high,
		&(krio_priv->regs->rxu_map[i].dest_prom_seg));

	__raw_writel(mapping_entry_qid,
		&(krio_priv->regs->rxu_map[i].flow_qid));

        /*
	 *  The RapidIO peripheral looks at the incoming RapidIO msgs
	 *  and if there is only one segment (the whole msg fits into one
	 *  RapidIO msg), the peripheral uses the single segment mapping
	 *  table. Therefore we need to map the single-segment mailbox too.
	 *  The same Rx CPPI Queue is used (as for the multi-segment
	 *  mailbox).
	 */
	mapping_entry_high &= ~KEYSTONE_RIO_MAP_FLAG_SEGMENT;
	
	i = keystone_rio_get_rxu_map(krio_priv);

	if (i < 0)
		return -ENOMEM;

	rx_mbox->rxu_map_id[1] = i;
	__raw_writel(mapping_entry_low,
		&(krio_priv->regs->rxu_map[i].ltr_mbox_src));

	__raw_writel(mapping_entry_high,
		&(krio_priv->regs->rxu_map[i].dest_prom_seg));

	__raw_writel(mapping_entry_qid,
		&(krio_priv->regs->rxu_map[i].flow_qid));

	return 0;
}

/**
 * keystone_rio_open_inb_mbox - Initialize KeyStone inbound mailbox
 * @mport: Master port implementing the inbound message unit
 * @dev_id: Device specific pointer to pass on event
 * @mbox: Mailbox to open
 * @entries: Number of entries in the inbound mailbox ring
 *
 * Initializes buffer ring, request the inbound message interrupt,
 * and enables the inbound message unit. Returns %0 on success
 * and %-EINVAL or %-ENOMEM on failure.
 */
static int keystone_rio_open_inb_mbox(
	struct rio_mport *mport, 
	void *dev_id, 
	int mbox, 
	int entries
)
{
	struct keystone_rio_data *krio_priv = mport->priv;
	struct keystone_rio_mbox_info *rx_mbox = &krio_priv->rx_mbox[mbox];
	struct keystone_rio_rx_chan_info *krx_chan = 
			&krio_priv->rx_channels[mbox];
	int res;

	dev_dbg(krio_priv->dev, 
		"open inb mbox: mport = 0x%x, dev_id = 0x%x,"
		" mbox = %d, entries = %d\n",
		(u32) mport, (u32) dev_id, mbox, entries);
	
	/* Check if the port is already registered in this queue */
	if (rx_mbox->port == mport)
		return 0;
	
	/* Initialization of RapidIO inbound MP */
	if (!(krx_chan->dma_channel)) {
		res = keystone_rio_mp_inb_init(mbox, krio_priv);
		if (res)
			return res;
	}

	rx_mbox->dev_id  = dev_id;
	rx_mbox->entries = entries;
	rx_mbox->port    = mport;
	rx_mbox->id      = mbox;
	rx_mbox->running = 1;

	/* Map the mailbox to queue/flow */
	res = keystone_rio_map_mbox(mbox,
		    krx_chan->queue_num,
		    krx_chan->flow_num,
		    mport->sys_size, krio_priv);
	if (res)
		return res;

	return 0;
}

/**
 * keystone_rio_close_inb_mbox - Shut down KeyStone inbound mailbox
 * @mport: Master port implementing the inbound message unit
 * @mbox: Mailbox to close
 *
 * Disables the outbound message unit, stop queues and free all resources
 */
static void keystone_rio_close_inb_mbox(struct rio_mport *mport, int mbox)
{
	struct keystone_rio_data *krio_priv = mport->priv;
	struct keystone_rio_mbox_info *rx_mbox = &krio_priv->rx_mbox[mbox];

	dev_info(krio_priv->dev, "close inb mbox: mport = 0x%x, mbox = %d\n",
			(u32) mport, mbox);

       	rx_mbox->running = 0;
	
	if (!(rx_mbox->port))
		return;

	rx_mbox->port = NULL;

	/* Release associated resource */
	keystone_rio_free_rxu_map(rx_mbox->rxu_map_id[0], krio_priv);
	keystone_rio_free_rxu_map(rx_mbox->rxu_map_id[1], krio_priv);

	keystone_rio_mp_inb_exit(mbox, krio_priv);

	return;
}

/**
 * keystone_rio_hw_add_inb_buffer - Add buffer to the KeyStone
 *   inbound message queue
 * @mport: Master port implementing the inbound message unit
 * @mbox: Inbound mailbox number
 * @buf: Buffer to add to inbound queue
 *
 * Adds the @buf buffer to the KeyStone inbound message queue. Returns
 * %0 on success or %-EINVAL on failure.
 */
static int keystone_rio_hw_add_inb_buffer(struct rio_mport *mport, 
					  int mbox, void *buffer)
{
	struct keystone_rio_data *krio_priv = mport->priv;
	struct keystone_rio_rx_chan_info *krx_chan =
				&krio_priv->rx_channels[mbox];

	krx_chan->buff_temp = buffer;
	dma_rxfree_refill(krx_chan->dma_channel);
		
	return 0;
}

/**
 * keystone_rio_hw_get_inb_message - Fetch inbound message from
 * the KeyStone message unit
 * @mport: Master port implementing the inbound message unit
 * @mbox: Inbound mailbox number
 *
 * Gets the next available inbound message from the inbound message queue.
 * A pointer to the message is returned on success or NULL on failure.
 */
static void *keystone_rio_hw_get_inb_message(struct rio_mport *mport, int mbox)
{
	struct keystone_rio_data *krio_priv = mport->priv;
	struct keystone_rio_mbox_info *p_mbox = &(krio_priv->rx_mbox[mbox]);
	struct keystone_rio_packet *p_info;
	void *buff;

	if (p_mbox->p_info_temp == NULL)
		return NULL;

	p_info = p_mbox->p_info_temp;
	buff = p_info->buff;

	p_mbox->p_info_temp = NULL;
	p_info->buff = NULL;

	dma_unmap_sg(krio_priv->dev, &p_info->sg[2], 1, DMA_FROM_DEVICE);
	kfree(p_info);

	return buff;
}

static void keystone_rio_mp_outb_exit(struct keystone_rio_data *krio_priv)
{
	if (!(krio_priv->tx_channel))
		return;

	dmaengine_pause(krio_priv->tx_channel);
	dma_release_channel(krio_priv->tx_channel);
	krio_priv->tx_channel = NULL;
	return;
}

static int keystone_rio_mp_outb_init(struct keystone_rio_data *krio_priv)
{
	struct dma_keystone_info config;
	dma_cap_mask_t mask;
	int err = -ENODEV;
	const char *name;

	dma_cap_zero(mask);
	dma_cap_set(DMA_SLAVE, mask);

	/* DMA TX channel */
	name = krio_priv->tx_chan_name;
	krio_priv->tx_channel = dma_request_channel_by_name(mask, name);
	if (IS_ERR_OR_NULL(krio_priv->tx_channel)) {
		dev_err(krio_priv->dev,
			"Error requesting TX channel, err %d\n", err);
		goto fail;
	}

	memset(&config, 0, sizeof(config));
	config.direction	= DMA_MEM_TO_DEV;
	config.tx_queue_depth	= krio_priv->tx_queue_depth;
	err = dma_keystone_config(krio_priv->tx_channel, &config);
	if (err) {
		dev_err(krio_priv->dev,
			"Error configuring TX channel, err %d\n", err);
		goto fail;
	}

	dev_info(krio_priv->dev, "Opened tx channel: %p\n",
		 krio_priv->tx_channel);

	return 0;

fail:
	if (krio_priv->tx_channel) {
		dma_release_channel(krio_priv->tx_channel);
		krio_priv->tx_channel = NULL;
	}

	return err;
}

/**
 * keystone_rio_open_outb_mbox - Initialize KeyStone outbound mailbox
 * @mport: Master port implementing the outbound message unit
 * @dev_id: Device specific pointer to pass on event
 * @mbox: Mailbox to open
 * @entries: Number of entries in the outbound mailbox ring
 *
 * Initializes buffer ring, request the outbound message interrupt,
 * and enables the outbound message unit. Returns %0 on success and
 * %-EINVAL or %-ENOMEM on failure.
 */
static int keystone_rio_open_outb_mbox (
	struct rio_mport *mport, 
	void *dev_id, 
	int mbox, 
	int entries
)
{
	struct keystone_rio_data *krio_priv = mport->priv;
	struct keystone_rio_mbox_info *tx_mbox = &(krio_priv->tx_mbox[mbox]);
	int res;

	if (mbox >= KEYSTONE_RIO_MAX_MBOX)
		return -EINVAL;

	dev_dbg(krio_priv->dev, 
		"open_outb_mbox: mport = 0x%x, dev_id = 0x%x,"
		"mbox = %d, entries = %d\n",
		(u32) mport, (u32) dev_id, mbox, entries);

	/* Check if already initialized */
	if (tx_mbox->port == mport)
		return 0;

	/* Initialization of RapidIO outbound MP */
	if (!(krio_priv->tx_channel)) {
		res = keystone_rio_mp_outb_init(krio_priv);
		if (res)
			return res;
	}

	tx_mbox->dev_id  = dev_id;
	tx_mbox->entries = entries;
	tx_mbox->port    = mport;
	tx_mbox->id      = mbox;
	tx_mbox->slot    = 0;
	tx_mbox->running = 1;

	return 0;
}

/**
 * keystone_rio_close_outb_mbox - Shut down KeyStone outbound mailbox
 * @mport: Master port implementing the outbound message unit
 * @mbox: Mailbox to close
 *
 * Disables the outbound message unit, stop queues and free all resources
 */
static void keystone_rio_close_outb_mbox(struct rio_mport *mport, int mbox)
{
	struct keystone_rio_data *krio_priv = mport->priv;
	struct keystone_rio_mbox_info *tx_mbox = &(krio_priv->tx_mbox[mbox]);

	if (mbox >= KEYSTONE_RIO_MAX_MBOX)
		return;

	dev_info(krio_priv->dev, "close_outb_mbox: mport = 0x%x, mbox = %d\n",
		(u32) mport, mbox);

	tx_mbox->port    = NULL;
	tx_mbox->running = 0;

	keystone_rio_mp_outb_exit(krio_priv);

	return;
}

static void keystone_rio_tx_complete(void *data)
{
	struct keystone_rio_packet *p_info	= data;
	struct keystone_rio_data *krio_priv	= p_info->priv;
	int mbox_id				= p_info->mbox;
	struct keystone_rio_mbox_info *mbox = &(krio_priv->tx_mbox[mbox_id]);
	struct rio_mport *port			= mbox->port;
	void *dev_id				= mbox->dev_id;

	dev_dbg(krio_priv->dev,
		"tx_complete: psdata[0] = %08x, psdata[1] = %08x\n",
		p_info->psdata[0], p_info->psdata[1]);

	p_info->status = dma_async_is_tx_complete(krio_priv->tx_channel,
				p_info->cookie, NULL, NULL);
	WARN_ON(p_info->status != DMA_SUCCESS && p_info->status != DMA_ERROR);

	dma_unmap_sg(krio_priv->dev, &p_info->sg[2], 1, DMA_TO_DEVICE);

	if (mbox->running) {
		/*
		 * Client is in charge of freeing the associated buffers
		 * Because we do not have explicit hardware ring but queues, we
		 * do not know where we are in the sw ring, let use fake slot.
		 * But the semantic hereafter is dangerous in case of re-order:
		 * bad buffer may be released...
		 */
		port->outb_msg[mbox_id].mcback(port, dev_id, 
					mbox_id, mbox->slot++);
		if (mbox->slot > mbox->entries)
			mbox->slot = 0;
	}

	kfree(p_info);
}

/**
 * keystone_rio_hw_add_outb_message - Add a message to the KeyStone
 * outbound message queue
 * @mport: Master port with outbound message queue
 * @rdev: Target of outbound message
 * @mbox: Outbound mailbox
 * @buffer: Message to add to outbound queue
 * @len: Length of message
 *
 * Adds the @buffer message to the KeyStone outbound message queue. Returns
 * %0 on success or %-EBUSY on failure.
 */
static int keystone_rio_hw_add_outb_message(
	struct rio_mport *mport, struct rio_dev *rdev, 
	int mbox, void *buffer, const size_t len)
{
	struct keystone_rio_data *krio_priv = mport->priv;
	struct dma_async_tx_descriptor *desc;
	struct keystone_rio_packet *p_info;
	struct dma_device *device;
	int ret = 0;
	/* Ensure that the number of bytes being transmitted is a multiple
	   of double-word.  This is as per the specification */
	u32  plen  = ((len + 7) & ~0x7);

	p_info = kzalloc(sizeof(*p_info), GFP_ATOMIC);
	if (!p_info) {
		dev_warn(krio_priv->dev, "failed to alloc packet info\n");
		ret = -ENOMEM;
		goto out;
	}

	p_info->priv = krio_priv;

	/* Word 1: source id and dest id (common to packet 11 and packet 9) */
	p_info->psdata[0] = (rdev->destid & 0xffff)
		| (mport->host_deviceid << 16);

	/* Packet type 11 case (Message) */

	/* Warning - Undocumented HW requirement:
		For type9, packet type MUST be set to 30 in 
		keystone_hw_desc.desc_info[29:25] bits.

		For type 11, setting packet type to 31 in 
		those bits is optional.
	*/
		
	/* Word 2: ssize = 32 dword, 4 retries, letter = 0, mbox */
	p_info->psdata[1] = (KEYSTONE_RIO_MSG_SSIZE << 17) | (4 << 21)
		| (mbox & 0x3f);

	if (rdev->net->hport->sys_size)
		p_info->psdata[1] |= KEYSTONE_RIO_DESC_FLAG_TT_16; /* tt */

	dev_dbg(krio_priv->dev,
		"packet type 11: psdata[0] = %08x, psdata[1] = %08x\n",
		p_info->psdata[0], p_info->psdata[1]);

	dev_dbg(krio_priv->dev, "buf(len=%d, plen=%d)\n", len, plen);

	p_info->mbox = mbox;
	p_info->buff = buffer;

	sg_init_table(p_info->sg, KEYSTONE_RIO_SGLIST_SIZE);
	sg_set_buf(&p_info->sg[0], p_info->epib, sizeof(p_info->epib));
	sg_set_buf(&p_info->sg[1], p_info->psdata, sizeof(p_info->psdata));
	sg_set_buf(&p_info->sg[2], buffer, plen);

	p_info->sg_ents = 2 + dma_map_sg(krio_priv->dev, &p_info->sg[2],
					 1, DMA_TO_DEVICE);

	if (p_info->sg_ents != KEYSTONE_RIO_SGLIST_SIZE) {
		kfree(p_info);
		dev_warn(krio_priv->dev, "failed to map transmit packet\n");
		ret = -ENXIO;
		goto out;
	}

	device = krio_priv->tx_channel->device;

	desc = dmaengine_prep_slave_sg(krio_priv->tx_channel, p_info->sg,
				       p_info->sg_ents, DMA_MEM_TO_DEV,
				       DMA_HAS_EPIB | DMA_HAS_PSINFO);

	if (IS_ERR_OR_NULL(desc)) {
		dma_unmap_sg(krio_priv->dev, &p_info->sg[2], 1, DMA_TO_DEVICE);
		kfree(p_info);
		dev_warn(krio_priv->dev, "failed to prep slave dma\n");
		ret = -ENOBUFS;
		goto out;
	}

	desc->callback_param = p_info;
	desc->callback = keystone_rio_tx_complete;
	p_info->cookie = dmaengine_submit(desc);

out:
	return ret;
}

/*------------------------ Main Linux driver functions -----------------------*/

struct rio_mport *keystone_rio_register_mport(u32 port_id, u32 size,
			struct keystone_rio_data *krio_priv)
{
	struct rio_ops   *ops;
	struct rio_mport *port;

	ops = kzalloc(sizeof(struct rio_ops), GFP_KERNEL);

	ops->lcread       = keystone_local_config_read;
	ops->lcwrite      = keystone_local_config_write;
	ops->cread        = keystone_rio_config_read;
	ops->cwrite       = keystone_rio_config_write;

	ops->open_outb_mbox       = keystone_rio_open_outb_mbox;
	ops->close_outb_mbox      = keystone_rio_close_outb_mbox;
	ops->open_inb_mbox        = keystone_rio_open_inb_mbox;
	ops->close_inb_mbox       = keystone_rio_close_inb_mbox;
	ops->add_outb_message     = keystone_rio_hw_add_outb_message;
	ops->add_inb_buffer       = keystone_rio_hw_add_inb_buffer;
	ops->get_inb_message      = keystone_rio_hw_get_inb_message;

	port = kzalloc(sizeof(struct rio_mport), GFP_KERNEL);
	port->id          = port_id;
	port->index       = port_id;
	port->priv        = krio_priv;
	INIT_LIST_HEAD(&port->dbells);

	/* Make a dummy per port region as ports are not
	   really separated on KeyStone */ 
	port->iores.start = krio_priv->board_rio_cfg.rio_regs_base + 
			(u32)(krio_priv->serial_port_regs) +
			offsetof(struct keystone_rio_serial_port_regs,
			sp[port_id].link_maint_req);

	port->iores.end   = krio_priv->board_rio_cfg.rio_regs_base + 
			(u32)(krio_priv->serial_port_regs) + 
			offsetof(struct keystone_rio_serial_port_regs,
			sp[port_id].ctl);

	port->iores.flags = IORESOURCE_MEM;

	rio_init_dbell_res(&port->riores[RIO_DOORBELL_RESOURCE], 0, 0xffff);
	rio_init_mbox_res(&port->riores[RIO_INB_MBOX_RESOURCE], 0, 0);
	rio_init_mbox_res(&port->riores[RIO_OUTB_MBOX_RESOURCE], 0, 0);

	sprintf(port->name, "RIO%d mport", port_id);

	port->ops           = ops;
	port->sys_size      = size;
	port->phy_type      = RIO_PHY_SERIAL;
	/* Hard coded here because in rio_disc_mport(), it is used in
	   rio_enum_complete() before it is retrieved in
	   rio_disc_peer() => rio_setup_device() */
	port->phys_efptr    = 0x100;

	rio_register_mport(port);

	return port;
}


static void keystone_rio_get_controller_defaults(struct device_node *node, 
				struct keystone_rio_data *krio_priv)
{
     	struct keystone_rio_board_controller_info *c =
				&krio_priv->board_rio_cfg;
	struct keystone_rio_rx_chan_info *krx_chan;
	u32 temp[15];
        int i;

	if(of_property_read_u32_array (node, "reg", (u32 *)&(temp[0]), 4)) {
		dev_err(krio_priv->dev, "Could not get default reg\n");
	} else {
		c->rio_regs_base = temp[0];
		c->rio_regs_size = temp[1];
		c->boot_cfg_regs_base = temp[2];
		c->boot_cfg_regs_size = temp[3];
	}

	if(of_property_read_u32 (node, "dev-id-size", (u32 *)&(c->size))) {
		dev_err(krio_priv->dev, "Could not get default dev-id-size\n");
	}

	if(of_property_read_u32 (node, "ports", (u32 *)&(c->ports))) {
		dev_err(krio_priv->dev, "Could not get default ports\n");
	}

	/* Serdes config */
	c->serdes_config_num = 1;  /* total number of serdes_config[] entries */
	c->mode = 0;  /* default serdes_config[] entry to be used */

	/* Mode 0: sRIO config 0: MPY = 5x, div rate = half,
	   link rate = 3.125 Gbps, mode 1x */
	c->serdes_config[0].cfg_cntl		= 0x0c053860;  /* setting control register config */
	c->serdes_config[0].serdes_cfg_pll	= 0x0229;      /* SerDes PLL configuration */
	c->serdes_config[0].prescalar_srv_clk	= 0x001e;      /* prescalar_srv_clk */
	
	/* serdes rx_chan_config */
	for (i=0; i<KEYSTONE_RIO_MAX_PORT; i++) {
		c->serdes_config[0].rx_chan_config[i] = 0x00440495;
	}

	/* serdes tx_chan_config */
	for (i=0; i<KEYSTONE_RIO_MAX_PORT; i++) {
		c->serdes_config[0].tx_chan_config[i] = 0x00180795;
	}

	/* path_mode */
	for (i=0; i<KEYSTONE_RIO_MAX_PORT; i++) {
		c->serdes_config[0].path_mode[i] = 0x00000000;
	}

	/* DMA tx chan config */
	if (of_property_read_string(node, "tx_channel",
		&krio_priv->tx_chan_name) < 0){
		dev_err(krio_priv->dev, 
			"missing \"tx_channel\" parameter\n");
		krio_priv->tx_chan_name = "riotx";
	}

	if (of_property_read_u32(node, "tx_queue_depth",
		&krio_priv->tx_queue_depth) < 0) {
		dev_err(krio_priv->dev, 
			"missing \"tx_queue_depth\" parameter\n");
		krio_priv->tx_queue_depth = 128;
	}

	/* DMA rx chan config */
	krx_chan = &(krio_priv->rx_channels[0]);
	if (of_property_read_string(node, "rx_channel", &krx_chan->name) < 0){
		dev_err(krio_priv->dev, 
			"missing \"rx_channel\" parameter\n");
		krx_chan->name = "riorx";
	}

	if (of_property_read_u32_array(node, "rx_queue_depth", 
			krx_chan->queue_depths, KEYSTONE_QUEUES_PER_CHAN) < 0) {
		dev_err(krio_priv->dev, 
			"missing \"rx_queue_depth\" parameter\n");
		krx_chan->queue_depths[0] = 128;
	}

	if (of_property_read_u32_array(node, "rx_buffer_size",
		krx_chan->buffer_sizes, KEYSTONE_QUEUES_PER_CHAN) < 0) {
		dev_err(krio_priv->dev,
			"missing \"rx_buffer_size\" parameter\n");
		krx_chan->buffer_sizes[0] = 1552;
	}
}

static void keystone_rio_port_status_timer(unsigned long data)
{
	struct keystone_rio_data *krio_priv = (struct keystone_rio_data *)data;
	u32 ports = krio_priv->ports_registering;

	if ((krio_priv->port_chk_cnt)++ >= 90) {
		dev_warn(krio_priv->dev,
			"RIO port register timeout, ports %08x not ready\n",
			ports);
		return;
	}

	schedule_work(&krio_priv->port_chk_task);
}

static void keystone_rio_port_chk_task(struct work_struct *work)
{
	struct keystone_rio_data *krio_priv =
		container_of(work, struct keystone_rio_data, port_chk_task);

	u32 ports = krio_priv->ports_registering;
	u32 size  = krio_priv->board_rio_cfg.size;
	struct rio_mport *mport;
#ifdef CONFIG_RIONET
	int has_port_ready = 0;
#endif

	krio_priv->ports_registering = 0;
	while (ports) {
		int status;
		u32 port = __ffs(ports);
		ports &= ~(1 << port);

		status = keystone_rio_port_status(port, krio_priv);
		if (status == 0) {
			/* Register this port  */
			mport = keystone_rio_register_mport(port,
						size, krio_priv);
			if (!mport)
				return;

			/* link is up, clear all errors */
			__raw_writel(0x00000000,
				&krio_priv->err_mgmt_regs->err_det);
			__raw_writel(0x00000000,
				&(krio_priv->err_mgmt_regs->sp_err[port].det));
			__raw_writel(
				__raw_readl(&(krio_priv->serial_port_regs->
						sp[port].err_stat)),
					    &(krio_priv->serial_port_regs->
						sp[port].err_stat));

#ifdef CONFIG_RIONET
			has_port_ready = 1;
#endif

			dev_info(krio_priv->dev,
				"RIO: port RIO%d host_deviceid %d registered\n",
				port, mport->host_deviceid);
		} else {
			krio_priv->ports_registering = (1 << port);

			dev_dbg(krio_priv->dev, "RIO: port %d not ready\n",
					 port);
		}
	}

	if (krio_priv->ports_registering) {
		/* setup and start a timer to poll status */
		krio_priv->timer.function = keystone_rio_port_status_timer;
		krio_priv->timer.data = (unsigned long)krio_priv;
		krio_priv->timer.expires = jiffies +
				KEYSTONE_RIO_REGISTER_DELAY;
		add_timer(&krio_priv->timer);
	}
#ifdef CONFIG_RIONET
	else if (has_port_ready) {
		rionet_init();
		krio_priv->rionet_started = 1;
	}
#endif
}

/*
 * Platform configuration setup
 */
static int __devinit keystone_rio_setup_controller(struct platform_device *pdev,
					 struct keystone_rio_data *krio_priv)
{
	u32 ports;
	u32 p;
	u32 mode;
	u32 size = 0;
	int res = 0;
#ifdef CONFIG_RIONET
	int has_port_ready = 0;
#endif
	struct rio_mport *mport;

	size   = krio_priv->board_rio_cfg.size;
	ports  = krio_priv->board_rio_cfg.ports;
	mode   = krio_priv->board_rio_cfg.mode;

	dev_dbg(&pdev->dev, "size = %d, ports = 0x%x, mode = %d\n",
		size, ports, mode);

	if (mode >= krio_priv->board_rio_cfg.serdes_config_num) {
		mode = 0;
		dev_warn(&pdev->dev,
			"RIO: invalid port mode, forcing it to %d\n", mode);
	}

	/* Hardware set up of the controller */
	keystone_rio_hw_init(mode, krio_priv);

	/* 
	 * Configure all ports even if we do not use all of them.
	 * This is needed for 2x and 4x modes.
	 */
	for (p = 0; p < KEYSTONE_RIO_MAX_PORT; p++) {
		res = keystone_rio_port_init(p, mode, krio_priv);
		if (res < 0) {
			dev_err(&pdev->dev,
				"RIO: initialization of port %d failed\n", p);
			return res;
		}
	}

	/* Start the controller */
	keystone_rio_start(krio_priv);

	/* Use and check ports status (but only the requested ones) */
	krio_priv->ports_registering = 0;
	while (ports) {
		int status;
		u32 port = __ffs(ports);
		ports &= ~(1 << port);

		/* Start the port */
		keystone_rio_port_activate(port, krio_priv);

		/* 
		 * Check the port status here before calling the generic RapidIO
		 * layer. Port status check is done in rio_mport_is_active() as 
		 * well but we need to do it our way first due to some delays in
		 * hw initialization.
		 */
		status = keystone_rio_port_status(port, krio_priv);
		if (status == 0) {
			/* Register this port  */
			mport = keystone_rio_register_mport(port, size,
							krio_priv);
			if (!mport)
				goto out;

#ifdef CONFIG_RIONET
			has_port_ready = 1;
#endif

			dev_info(&pdev->dev, 
				"RIO: port RIO%d host_deviceid %d registered\n",
				port, mport->host_deviceid);
		} else {
			dev_warn(&pdev->dev, "RIO: port %d not ready\n", port);
			krio_priv->ports_registering = (1 << port);
		}
	}

	if (krio_priv->ports_registering) {
		/* setup and start a timer to poll status */
		init_timer(&krio_priv->timer);
		krio_priv->port_chk_cnt = 0;
		krio_priv->timer.function = keystone_rio_port_status_timer;
		krio_priv->timer.data = (unsigned long)krio_priv;
		krio_priv->timer.expires = jiffies +
				KEYSTONE_RIO_REGISTER_DELAY;
		add_timer(&krio_priv->timer);
	}
#ifdef CONFIG_RIONET
	else if (has_port_ready) {
		rionet_init();
		krio_priv->rionet_started = 1;
	}
#endif

out:
	return res;
}

static int __init keystone_rio_probe(struct platform_device *pdev)	
{
	struct device_node *node = pdev->dev.of_node;
	struct keystone_rio_data *krio_priv;
	int res = 0;
	void __iomem *regs;

	if (!node) {
		dev_err(&pdev->dev, "could not find device info\n");
		return -EINVAL;
	}

	krio_priv = kzalloc(sizeof(struct keystone_rio_data), GFP_KERNEL);
	if (!krio_priv) {
		dev_err(&pdev->dev, "memory allocation failed\n");
		return -ENOMEM;
	}

	platform_set_drvdata(pdev, krio_priv);
	krio_priv->dev = &(pdev->dev);

	krio_priv->rionet_started = 0;

	/* Get default config from device tree */
        keystone_rio_get_controller_defaults(node, krio_priv);

	/* sRIO main driver (global ressources) */
	mutex_init(&krio_priv->lsu_lock);
	init_completion(&krio_priv->lsu_completion);
	INIT_WORK(&krio_priv->port_chk_task, keystone_rio_port_chk_task);

	regs = ioremap(krio_priv->board_rio_cfg.boot_cfg_regs_base, 
			krio_priv->board_rio_cfg.boot_cfg_regs_size);
	krio_priv->jtagid_reg = regs + 0x0018;
	krio_priv->serdes_sts_reg = regs + 0x154;
	krio_priv->serdes_regs = regs + 0x360;

	regs = ioremap(krio_priv->board_rio_cfg.rio_regs_base, 
			krio_priv->board_rio_cfg.rio_regs_size);
	krio_priv->regs			= regs;
	krio_priv->car_csr_regs		= regs + 0xb000;
	krio_priv->serial_port_regs	= regs + 0xb100;
	krio_priv->err_mgmt_regs	= regs + 0xc000;
	krio_priv->phy_regs		= regs + 0x1b000;
	krio_priv->transport_regs	= regs + 0x1b300;
	krio_priv->pkt_buf_regs		= regs + 0x1b600;
	krio_priv->evt_mgmt_regs	= regs + 0x1b900;
	krio_priv->port_write_regs	= regs + 0x1ba00;
	krio_priv->link_regs		= regs + 0x1bd00;
	krio_priv->fabric_regs		= regs + 0x1be00;
	krio_priv->car_csr_regs_base = (u32)regs + 0xb000;

	/* Enable srio clock */
	krio_priv->clk = clk_get(&pdev->dev, "clk_srio");
	if (IS_ERR(krio_priv->clk)) {
		dev_err(&pdev->dev, "Unable to get Keystone SRIO clock\n");
		return -EBUSY;
	}
	else {
		clk_prepare_enable(krio_priv->clk);
		ndelay(100);
		clk_disable_unprepare(krio_priv->clk);
		ndelay(100);
		clk_prepare_enable(krio_priv->clk);
	}

	dev_info(&pdev->dev, "KeyStone RapidIO driver %s\n", DRIVER_VER);

	/* Setup the sRIO controller */
	res = keystone_rio_setup_controller(pdev, krio_priv);
	if (res < 0)
		return res;

	return 0;
}

static void keystone_rio_shutdown(struct platform_device *pdev)
{
	struct keystone_rio_data *krio_priv = platform_get_drvdata(pdev);
	int i;

#ifdef CONFIG_RIONET
	if (krio_priv->rionet_started)
		rionet_exit();
#endif

	keystone_rio_mp_outb_exit(krio_priv);

	for (i=0; i<KEYSTONE_RIO_MAX_MBOX; i++)
		keystone_rio_mp_inb_exit(i, krio_priv);

	mdelay(1000);

	/* disable blocks */
	__raw_writel(0, &krio_priv->regs->gbl_en);
	for (i = 0; i <= KEYSTONE_RIO_BLK_NUM; i++)
		__raw_writel(0, &(krio_priv->regs->blk[i].enable));

	if (krio_priv->clk) {
		clk_disable_unprepare(krio_priv->clk);
		clk_put(krio_priv->clk);
	}

	platform_set_drvdata(pdev, NULL);

	kfree(krio_priv);
}

static int __exit keystone_rio_remove(struct platform_device *pdev)
{
	keystone_rio_shutdown(pdev);
	return 0;
}

static struct of_device_id __devinitdata of_match[] = {
	{ .compatible = "ti,keystone-rapidio", },
	{},
};

MODULE_DEVICE_TABLE(of, keystone_hwqueue_of_match);

static struct platform_driver keystone_rio_driver  = {
	.driver = {
		.name	        = "keystone-rapidio",
		.owner	        = THIS_MODULE,
		.of_match_table	= of_match,
	},
	.probe	= keystone_rio_probe,
	.remove = __exit_p(keystone_rio_remove),
	.shutdown = keystone_rio_shutdown,
};

static int keystone_rio_module_init(void)
{
	return platform_driver_register(&keystone_rio_driver);
}

static void __exit keystone_rio_module_exit(void)
{
	platform_driver_unregister(&keystone_rio_driver);
}

module_init(keystone_rio_module_init);
module_exit(keystone_rio_module_exit);

MODULE_AUTHOR("Aurelien Jacquiot");
MODULE_DESCRIPTION("TI KeyStone RapidIO device driver");
MODULE_LICENSE("GPL");
