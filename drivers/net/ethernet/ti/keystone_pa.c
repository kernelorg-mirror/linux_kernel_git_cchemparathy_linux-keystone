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
#include <linux/clk.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/types.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_address.h>
#include <linux/firmware.h>
#include <linux/spinlock.h>
#include <linux/interrupt.h>
#include <linux/dmaengine.h>
#include <linux/dma-mapping.h>
#include <linux/scatterlist.h>
#include <linux/byteorder/generic.h>
#include <linux/platform_device.h>
#include <linux/keystone-dma.h>
#include <linux/errqueue.h>

#include "keystone_net.h"
#include "keystone_pa.h"
#include "keystone_pasahost.h"

#define DEVICE_PA_PDSP02_FIRMWARE "keystone/pa_pdsp02_classify1.fw"
#define DEVICE_PA_PDSP3_FIRMWARE "keystone/pa_pdsp3_classify2.fw"
#define DEVICE_PA_PDSP45_FIRMWARE "keystone/pa_pdsp45_pam.fw"

#define PSTREAM_ROUTE_PDSP0	0

#define PA_PDSP_ALREADY_ACTIVE	0
#define PA_PDSP_RESET_RELEASED	1
#define PA_PDSP_NO_RESTART	2
#define PA_MAX_PDSP_ENABLE_LOOP_COUNT	100000

#define PA_STATE_RESET			0  /* Sub-system state reset */
#define PA_STATE_ENABLE			1  /* Sub-system state enable  */
#define PA_STATE_QUERY			2  /* Query the Sub-system state */
#define PA_STATE_INCONSISTENT		3  /* Sub-system is partially enabled */
#define PA_STATE_INVALID_REQUEST	4  /* Invalid state command to the Sub-system */
#define PA_STATE_ENABLE_FAILED		5  /*  The Sub-system did not respond after restart */

/* System Timestamp */
#define PAFRM_SRAM_SIZE			0x2000
#define PAFRM_SYS_TIMESTAMP_ADDR	0x6460

/* PDSP Versions */
#define PAFRM_PDSP_VERSION_BASE		0x7F04

#define DEVICE_PA_BASE				0x02000000
#define DEVICE_PA_REGION_SIZE			0x48000
#define DEVICE_PA_NUM_PDSPS			6

#define PA_MEM_PDSP_IRAM(pdsp)			((pdsp) * 0x8000)
#define PA_MEM_PDSP_SRAM(num)			((num) * 0x2000)
#define PA_REG_PKTID_SOFT_RESET	                0x00404
#define PA_REG_LUT2_SOFT_RESET	                0x00504
#define PA_REG_STATS_SOFT_RESET	                0x06004

#define PA_PDSP_CONST_REG_INDEX_C25_C24     0
#define PA_PDSP_CONST_REG_INDEX_C27_C26     1
#define PA_PDSP_CONST_REG_INDEX_C29_C28     2
#define PA_PDSP_CONST_REG_INDEX_C31_C30     3

/* The pdsp control register */
#define PA_REG_VAL_PDSP_CTL_DISABLE_PDSP	1
#define PA_REG_VAL_PDSP_CTL_RESET_PDSP	        0
#define PA_REG_VAL_PDSP_CTL_STATE               (1 << 15)
#define PA_REG_VAL_PDSP_CTL_ENABLE              (1 << 1)
#define PA_REG_VAL_PDSP_CTL_SOFT_RESET          (1 << 0)
#define PA_REG_VAL_PDSP_CTL_ENABLE_PDSP(pcval)	(((pcval) << 16)	\
						 | PA_REG_VAL_PDSP_CTL_ENABLE \
						 | PA_REG_VAL_PDSP_CTL_SOFT_RESET)

/* Number of mailbox slots for each PDPS */
#define PA_NUM_MAILBOX_SLOTS	4
#define TEST_SWINFO0_TIMESTAMP	0x12340002

#define PACKET_DROP	0
#define PACKET_PARSE	1
#define PACKET_HST	2

#define NT 32

#define PA_SGLIST_SIZE	3

const u32 pap_pdsp_const_reg_map[6][4] =
{
	/* PDSP0: C24-C31 */
	{
		0x0000007F,		/* C25-C24 */
		0x0000006E,		/* C27-C26 */
		0x00000000,		/* C29-C28 */
		0x00000000		/* C31-C30 */
	},
	/* PDSP1: C24-C31 */
	{
		0x0001007F,		/* C25-C24 */
		0x00480040,		/* C27-C26 */
		0x00000000,		/* C29-C28 */
		0x00000000		/* C31-C30 */
	},
	/* PDSP2: C24-C31 */
	{
		0x0002007F,		/* C25-C24 */
		0x00490044,		/* C27-C26 */
		0x00000000,		/* C29-C28 */
		0x00000000		/* C31-C30 */
	},
	/* PDSP3: C24-C31 */
	{
		0x0003007F,		/* C25-C24 */
		0x0000006E,		/* C27-C26 */
		0x00000000,		/* C29-C28 */
		0x00000000		/* C31-C30 */
	},
	/* PDSP4: C24-C31 */
	{
		0x0070007F,		/* C25-C24 */
		0x00000000,		/* C27-C26 */
		0x04080404,		/* C29-C28 */
		0x00000000		/* C31-C30 */
	},
	/* PDSP5: C24-C31 */
	{
		0x0071007F,		/* C25-C24 */
		0x00000000,		/* C27-C26 */
		0x04080404,		/* C29-C28 */
		0x00000000		/* C31-C30 */
	}
};

struct pa_mailbox_regs {
	u32 pdsp_mailbox_slot0;
	u32 pdsp_mailbox_slot1;
	u32 pdsp_mailbox_slot2;
	u32 pdsp_mailbox_slot3;
};

struct pa_packet_id_alloc_regs {
	u32 revision;
	u32 soft_reset;
	u32 range_limit;
	u32 idvalue;
};

struct pa_lut2_control_regs {
	u32 revision;
	u32 soft_reset;
	u32 rsvd[6];
	u32 add_data0;
	u32 add_data1;
	u32 add_data2;
	u32 add_data3;
	u32 add_del_key;
	u32 add_del_control;
};

struct pa_pdsp_control_regs {
	u32 control;
	u32 status;
	u32 wakeup_enable;
	u32 cycle_count;
	u32 stall_count;
	u32 rsvd[3];
	u32 const_tbl_blk_index0;
	u32 const_tbl_blk_index1;
	u32 const_tbl_prog_pointer0;
	u32 const_tbl_prog_pointer1;
	u32 rsvd1[52];
};

struct pa_pdsp_timer_regs {
	u32 timer_control;
	u32 timer_load;
	u32 timer_value;
	u32 timer_interrupt;
	u32 rsvd[60];
};

struct pa_statistics_regs {
	u32 revision;
	u32 soft_reset;
	u32 incr_flags;
	u32 stats_capture;
	u32 rsvd[4];
	u32 stats_red[32];
};

struct pa_device {
	struct device			*dev;
	struct netcp_module_data	 module;
	struct clk			*clk;
	struct dma_chan			*tx_channel;
	struct dma_chan			*rx_channel;
	const char			*tx_chan_name;
	const char			*rx_chan_name;
	unsigned			 cmd_flow_num;
	unsigned			 cmd_queue_num;
	unsigned			 data_flow_num;
	unsigned			 data_queue_num;

	u64				 pa2system_offset;

	struct pa_mailbox_regs __iomem	*reg_mailbox;
	struct pa_packet_id_alloc_regs __iomem	*reg_packet_id;
	struct pa_lut2_control_regs __iomem	*reg_lut2;
	struct pa_pdsp_control_regs __iomem	*reg_control;
	struct pa_pdsp_timer_regs   __iomem	*reg_timer;
	struct pa_statistics_regs   __iomem	*reg_stats;
	void __iomem			*pa_sram;
	void __iomem			*pa_iram;
	void __iomem			*streaming_switch;

	u8				*mc_list;
	u8				 addr_count;
	struct tasklet_struct		 task;
	spinlock_t			 lock;
	
	struct netcp_tx_pipe		 tx_pipe;
	u32				 tx_cmd_queue_depth;
	u32				 tx_data_queue_depth;
	u32				 rx_pool_depth;
	u32				 rx_buffer_size;
};

#define pa_from_module(data)	container_of(data, struct pa_device, module)
#define pa_to_module(pa)	(&(pa)->module)

struct pa_packet {
	struct scatterlist		 sg[PA_SGLIST_SIZE];
	int				 sg_ents;
	enum dma_status			 status;
	enum dma_transfer_direction	 direction;
	struct pa_device		*priv;
	struct dma_chan			*chan;
	struct dma_async_tx_descriptor	*desc;
	dma_cookie_t			 cookie;
	u32				 epib[4];
	u32				 psdata[6];
	struct completion		 complete;
	void				*data;
};

static void pdsp_fw_put(u32 *dest, const u32 *src, u32 wc)
{
	int i;

	for (i = 0; i < wc; i++)
		*dest++ = be32_to_cpu(*src++);
}

static inline void swizFwd (struct pa_frm_forward *fwd)
{
	fwd->flow_id = fwd->flow_id;
	fwd->queue   = cpu_to_be16(fwd->queue);

	if (fwd->forward_type == PAFRM_FORWARD_TYPE_HOST) {
		fwd->u.host.context      = cpu_to_be32(fwd->u.host.context);
		fwd->u.host.multi_route  = fwd->u.host.multi_route;
		fwd->u.host.multi_idx    = fwd->u.host.multi_idx;
		fwd->u.host.pa_pdsp_router = fwd->u.host.pa_pdsp_router;
	} else if (fwd->forward_type == PAFRM_FORWARD_TYPE_SA) {
		fwd->u.sa.sw_info_0 = cpu_to_be32(fwd->u.sa.sw_info_0);
		fwd->u.sa.sw_info_1 = cpu_to_be32(fwd->u.sa.sw_info_1);
	} else if (fwd->forward_type == PAFRM_FORWARD_TYPE_SRIO) {
		fwd->u.srio.ps_info0 = cpu_to_be32(fwd->u.srio.ps_info0);
		fwd->u.srio.ps_info1 = cpu_to_be32(fwd->u.srio.ps_info1);
		fwd->u.srio.pkt_type = fwd->u.srio.pkt_type;
	} else if (fwd->forward_type == PAFRM_FORWARD_TYPE_ETH) {
		fwd->u.eth.ps_flags	= fwd->u.eth.ps_flags;
	} else if (fwd->forward_type == PAFRM_FORWARD_TYPE_PA) {
		fwd->u.pa.pa_dest	= fwd->u.pa.pa_dest;
		fwd->u.pa.custom_type	= fwd->u.pa.custom_type;
		fwd->u.pa.custom_idx	= fwd->u.pa.custom_idx;
	}

	fwd->forward_type = fwd->forward_type;
}

static inline void swizFcmd (struct pa_frm_command *fcmd)
{
	fcmd->command_result =  cpu_to_be32(fcmd->command_result);
	fcmd->command	     =  fcmd->command;
	fcmd->magic          =  fcmd->magic;
	fcmd->com_id         =  cpu_to_be16(fcmd->com_id);
	fcmd->ret_context    =  cpu_to_be32(fcmd->ret_context);
	fcmd->reply_queue    =  cpu_to_be16(fcmd->reply_queue);
	fcmd->reply_dest     =  fcmd->reply_dest;
	fcmd->flow_id        =  fcmd->flow_id;
}

static inline void swizAl1 (struct pa_frm_cmd_add_lut1 *al1)
{
	al1->index         =  al1->index;
	al1->type          =  al1->type;
	al1->cust_index    =  al1->cust_index;

	if (al1->type == PAFRM_COM_ADD_LUT1_STANDARD) {
		al1->u.eth_ip.etype = cpu_to_be16(al1->u.eth_ip.etype);
		al1->u.eth_ip.vlan  = cpu_to_be16(al1->u.eth_ip.vlan);
		al1->u.eth_ip.spi   = cpu_to_be32(al1->u.eth_ip.spi);
		al1->u.eth_ip.flow  = cpu_to_be32(al1->u.eth_ip.flow);

		if (al1->u.eth_ip.key & PAFRM_LUT1_KEY_MPLS)
			al1->u.eth_ip.pm.mpls     =  cpu_to_be32(al1->u.eth_ip.pm.mpls);
		else {
			al1->u.eth_ip.pm.ports[0] =  cpu_to_be16(al1->u.eth_ip.pm.ports[0]);
			al1->u.eth_ip.pm.ports[1] =  cpu_to_be16(al1->u.eth_ip.pm.ports[1]);
		}

		al1->u.eth_ip.proto_next  =  al1->u.eth_ip.proto_next;
		al1->u.eth_ip.tos_tclass  =  al1->u.eth_ip.tos_tclass;
		al1->u.eth_ip.inport      =  al1->u.eth_ip.inport;
		al1->u.eth_ip.key         =  al1->u.eth_ip.key;
		al1->u.eth_ip.match_flags =  cpu_to_be16(al1->u.eth_ip.match_flags);
	} else if (al1->type == PAFRM_COM_ADD_LUT1_SRIO) {
		al1->u.srio.src_id	= cpu_to_be16(al1->u.srio.src_id);
		al1->u.srio.dest_id	= cpu_to_be16(al1->u.srio.dest_id);
		al1->u.srio.etype	= cpu_to_be16(al1->u.srio.etype);
		al1->u.srio.vlan	= cpu_to_be16(al1->u.srio.vlan);
		al1->u.srio.pri         = al1->u.srio.pri;
		al1->u.srio.type_param1 = cpu_to_be16(al1->u.srio.type_param1);
		al1->u.srio.type_param2 = al1->u.srio.type_param2;
		al1->u.srio.key         = al1->u.srio.key;
		al1->u.srio.match_flags = cpu_to_be16(al1->u.srio.match_flags);
		al1->u.srio.next_hdr    = al1->u.srio.next_hdr;
		al1->u.srio.next_hdr_offset = cpu_to_be16(al1->u.srio.next_hdr_offset);
	} else {
		al1->u.custom.etype		=  cpu_to_be16(al1->u.custom.etype);
		al1->u.custom.vlan		=  cpu_to_be16(al1->u.custom.vlan);
		al1->u.custom.key		=  al1->u.custom.key;
		al1->u.custom.match_flags	=  cpu_to_be16(al1->u.custom.match_flags);
	}

	swizFwd(&(al1->match));
	swizFwd(&(al1->next_fail));
}

static int pa_conv_routing_info(struct	pa_frm_forward *fwd_info,
			 struct	pa_route_info *route_info,
			 int cmd_dest, u16 fail_route)
{
	u8 *pcmd = NULL;
	fwd_info->flow_id = route_info->flow_id;
	fwd_info->queue   = route_info->queue;

	if (route_info->dest == PA_DEST_HOST) {
		fwd_info->forward_type   = PAFRM_FORWARD_TYPE_HOST;
		fwd_info->u.host.context = route_info->sw_info_0;

		if (route_info->m_route_index >= 0) {
			if (route_info->m_route_index >= PA_MAX_MULTI_ROUTE_SETS) {
				return (PA_ERR_CONFIG);
			}

			fwd_info->u.host.multi_route	= 1;
			fwd_info->u.host.multi_idx	= route_info->m_route_index;
			fwd_info->u.host.pa_pdsp_router	= PAFRM_DEST_PA_M_0;
		}
		pcmd = fwd_info->u.host.cmd;
	} else if (route_info->dest == PA_DEST_DISCARD)	{
		fwd_info->forward_type = PAFRM_FORWARD_TYPE_DISCARD;
	} else if (route_info->dest == PA_DEST_EMAC) {
		fwd_info->forward_type = PAFRM_FORWARD_TYPE_ETH;
		fwd_info->u.eth.ps_flags = (route_info->pkt_type_emac_ctrl &
					    PA_EMAC_CTRL_CRC_DISABLE)?
			PAFRM_ETH_PS_FLAGS_DISABLE_CRC:0;
		fwd_info->u.eth.ps_flags |= ((route_info->pkt_type_emac_ctrl &
					      PA_EMAC_CTRL_PORT_MASK) <<
					     PAFRM_ETH_PS_FLAGS_PORT_SHIFT);
	} else if (fail_route) {
		return (PA_ERR_CONFIG);

	} else if (((route_info->dest == PA_DEST_CONTINUE_PARSE_LUT1) &&
		    (route_info->custom_type != PA_CUSTOM_TYPE_LUT2)) ||
		   ((route_info->dest == PA_DEST_CONTINUE_PARSE_LUT2) &&
		    (route_info->custom_type != PA_CUSTOM_TYPE_LUT1))) {

		/* Custom Error check */
		if (((route_info->custom_type == PA_CUSTOM_TYPE_LUT1) &&
		     (route_info->custom_index >= PA_MAX_CUSTOM_TYPES_LUT1)) ||
		    ((route_info->custom_type == PA_CUSTOM_TYPE_LUT2) &&
		     (route_info->custom_index >= PA_MAX_CUSTOM_TYPES_LUT2)))
			return(PA_ERR_CONFIG);

		fwd_info->forward_type = PAFRM_FORWARD_TYPE_PA;
		fwd_info->u.pa.custom_type = (u8)route_info->custom_type;
		fwd_info->u.pa.custom_idx  = route_info->custom_index;

		if (route_info->dest == PA_DEST_CONTINUE_PARSE_LUT2) {
			fwd_info->u.pa.pa_dest = PAFRM_DEST_PA_C2;
		} else {
			/*
			 * cmd_dest is provided by calling function
			 * There is no need to check error case
			 */
			fwd_info->u.pa.pa_dest = (cmd_dest == PA_CMD_TX_DEST_0)?
				PAFRM_DEST_PA_C1_1:PAFRM_DEST_PA_C1_2;
		}
	} else if (route_info->dest == PA_DEST_SASS) {
		fwd_info->forward_type   = PAFRM_FORWARD_TYPE_SA;
		fwd_info->u.sa.sw_info_0 = route_info->sw_info_0;
		fwd_info->u.sa.sw_info_1 = route_info->sw_info_1;
		pcmd = fwd_info->u.sa.cmd;
	} else if (route_info->dest == PA_DEST_SRIO) {
		fwd_info->forward_type		= PAFRM_FORWARD_TYPE_SRIO;
		fwd_info->u.srio.ps_info0	= route_info->sw_info_0;
		fwd_info->u.srio.ps_info1	= route_info->sw_info_1;
		fwd_info->u.srio.pkt_type	= route_info->pkt_type_emac_ctrl;
	} else {
		return (PA_ERR_CONFIG);
	}

	if (pcmd && route_info->pcmd) {
		struct pa_cmd_info *pacmd = route_info->pcmd;
		struct pa_patch_info *patch_info;
		struct pa_cmd_set *cmd_set;

		switch (pacmd->cmd) {
		case PA_CMD_PATCH_DATA:
			patch_info = &pacmd->params.patch;
			if ((patch_info->n_patch_bytes > 2) ||
			    (patch_info->overwrite) ||
			    (patch_info->patch_data == NULL))
				return (PA_ERR_CONFIG);

			pcmd[0] = PAFRM_RX_CMD_CMDSET;
			pcmd[1] = patch_info->n_patch_bytes;
			pcmd[2] = patch_info->patch_data[0];
			pcmd[3] = patch_info->patch_data[1];
			break;

		case PA_CMD_CMDSET:
			cmd_set = &pacmd->params.cmd_set;
			if(cmd_set->index >= PA_MAX_CMD_SETS)
				return (PA_ERR_CONFIG);

			pcmd[0] = PAFRM_RX_CMD_CMDSET;
			pcmd[1] = (u8)cmd_set->index;
			break;
		default:
			return(PA_ERR_CONFIG);
		}
	}
	return (PA_OK);
}

static int keystone_pa_reset(struct pa_device *pa_dev)
{
	struct pa_packet_id_alloc_regs __iomem	*packet_id_regs = pa_dev->reg_packet_id;
	struct pa_lut2_control_regs __iomem	*lut2_regs = pa_dev->reg_lut2;
	struct pa_statistics_regs   __iomem	*stats_regs = pa_dev->reg_stats;
	u32 i;

	/* Reset and disable all PDSPs */
	for (i = 0; i < DEVICE_PA_NUM_PDSPS; i++) {
		struct pa_pdsp_control_regs __iomem *ctrl_reg = &pa_dev->reg_control[i];
		__raw_writel(PA_REG_VAL_PDSP_CTL_RESET_PDSP,
			     &ctrl_reg->control);

		while((__raw_readl(&ctrl_reg->control)
		       & PA_REG_VAL_PDSP_CTL_STATE));
	}

	/* Reset packet Id */
	__raw_writel(1, &packet_id_regs->soft_reset);

	/* Reset LUT2 */
	__raw_writel(1, &lut2_regs->soft_reset);

	/* Reset statistic */
	__raw_writel(1, &stats_regs->soft_reset);

	/* Reset timers */
	for (i = 0; i < DEVICE_PA_NUM_PDSPS; i++) {
		struct pa_pdsp_timer_regs __iomem *timer_reg = &pa_dev->reg_timer[i];
		__raw_writel(0, &timer_reg->timer_control);
	}

	return 0;
}

/*
 *  Convert a raw PA timer count to nanoseconds
 *
 *  This assumes the PA timer frequency is 163,840,000 Hz.
 *  This is true for the TCI6614 EVM with default PLL settings,
 *  but is NOT a good generic assumption! Fix this later.
 */
static u64 tstamp_raw_to_ns(u64 raw)
{
	/* 100000/(2^14) = 6.103515625 nanoseconds per tick */
	/* Take care not to exceed 64 bits! */

	return (raw * 50000ULL) >> 13;
}

static u64 pa_to_sys_time(u64 offset, u64 pa_ticks)
{	
	s64 temp;
	u64 result;

	/* we need to compute difference from wallclock
	*  to time from boot dynamically since
	*  it will change whenever absolute time is adjusted by
	*  protocols above (ntp, ptpv2)
	*/

	temp = ktime_to_ns(ktime_get_monotonic_offset());
	result = (u64)((s64)offset - temp + (s64)tstamp_raw_to_ns(pa_ticks));

	return result;
}

static inline u64 tstamp_get_raw(struct pa_device *pa_dev)
{
	struct pa_pdsp_timer_regs __iomem *timer_reg = &pa_dev->reg_timer[0];
	u32 low, high, high2;
	u64 raw;
	int count;

	count = 0;
	do {
		high  = __raw_readl(pa_dev->pa_sram + 0x6460);
		low   = __raw_readl(&timer_reg->timer_value);
		high2 = __raw_readl(pa_dev->pa_sram + 0x6460);
	} while((high != high2) && (++count < 32));

	raw = (((u64)high) << 16) | (u64)(0x0000ffff - (low & 0x0000ffff));

	return raw;
}

/*
 * calibrate the PA timer to the system time
 * ktime_get gives montonic time 
 * ktime_to_ns converts ktime to ns
 * this needs to be called before doing conversions
 */
static void pa_calibrate_with_system_timer(struct pa_device *pa_dev)
{
	ktime_t ktime1, ktime2;
	u64 pa_ticks;
	u64 pa_ns;
	u64 sys_ns1, sys_ns2;

	/* Get the two values with minimum delay between */
	ktime1 = ktime_get();
	pa_ticks = tstamp_get_raw(pa_dev);
	ktime2 = ktime_get();

	/* Convert both values to nanoseconds */
	sys_ns1 = ktime_to_ns(ktime1);
	pa_ns   = tstamp_raw_to_ns(pa_ticks);
	sys_ns2 = ktime_to_ns(ktime2);

	/* compute offset */
	pa_dev->pa2system_offset = sys_ns1 + ((sys_ns2 - sys_ns1) / 2) - pa_ns;
}

static int pa_config_timestamp(struct pa_device *pa_dev, int factor)
{
	struct pa_pdsp_timer_regs __iomem *timer_reg = &pa_dev->reg_timer[0];

	if (factor < PA_TIMESTAMP_SCALER_FACTOR_2 ||
	    factor > PA_TIMESTAMP_SCALER_FACTOR_8192)
		return -1;
	else {
		__raw_writel(0xffff, &timer_reg->timer_load);
		__raw_writel((PA_SS_TIMER_CNTRL_REG_GO |
			      PA_SS_TIMER_CNTRL_REG_MODE |
			      PA_SS_TIMER_CNTRL_REG_PSE |
			      (factor << PA_SS_TIMER_CNTRL_REG_PRESCALE_SHIFT)),
			      &timer_reg->timer_control);
	}

	return 0;
}

static void pa_get_version(struct pa_device *pa_dev)
{
	u32 version;

	version = __raw_readl(pa_dev->pa_sram + PAFRM_PDSP_VERSION_BASE);

	dev_info(pa_dev->dev, "Using Packet Accelerator Firmware version "
				"0x%08x\n", version);
}

static int pa_pdsp_run(struct pa_device *pa_dev, int pdsp)
{
	struct pa_pdsp_control_regs __iomem *ctrl_reg = &pa_dev->reg_control[pdsp];
	struct pa_mailbox_regs __iomem *mailbox_reg = &pa_dev->reg_mailbox[pdsp];
	u32 i, v;

	/* Check for enabled PDSP */
	v = __raw_readl(&ctrl_reg->control);
	if ((v & PA_REG_VAL_PDSP_CTL_ENABLE) ==
	    PA_REG_VAL_PDSP_CTL_ENABLE) {
		/* Already enabled */
		return (PA_PDSP_ALREADY_ACTIVE);
	}

	/* Clear the mailbox */
	__raw_writel(0, &mailbox_reg->pdsp_mailbox_slot0);

	/* Set PDSP PC to 0, enable the PDSP */
	__raw_writel(PA_REG_VAL_PDSP_CTL_ENABLE |
		     PA_REG_VAL_PDSP_CTL_SOFT_RESET,
		     &ctrl_reg->control);

	/* Wait for the mailbox to become non-zero */
	for (i = 0; i < PA_MAX_PDSP_ENABLE_LOOP_COUNT; i++)
		v = __raw_readl(&mailbox_reg->pdsp_mailbox_slot0);
		if (v != 0)
			return (PA_PDSP_RESET_RELEASED);

	return (PA_PDSP_NO_RESTART);
}

static int keystone_pa_reset_control(struct pa_device *pa_dev, int new_state)
{
	struct pa_mailbox_regs __iomem *mailbox_reg = &pa_dev->reg_mailbox[0];
	int do_global_reset = 1;
	int i, res;
	int ret;

	if (new_state == PA_STATE_ENABLE) {
		ret = PA_STATE_ENABLE;

		/*
		 * Do nothing if a pdsp is already out of reset.
		 * If any PDSPs are out of reset
		 * a global init is not performed
		 */
		for (i = 0; i < 6; i++) {
			res = pa_pdsp_run(pa_dev, i);

			if (res == PA_PDSP_ALREADY_ACTIVE)
				do_global_reset = 0;

			if (res == PA_PDSP_NO_RESTART) {
				ret = PA_STATE_ENABLE_FAILED;
				do_global_reset = 0;
			}
		}

		/* If global reset is required any PDSP can do it */
		if (do_global_reset) {
			__raw_writel(1, &mailbox_reg->pdsp_mailbox_slot1);
			__raw_writel(0, &mailbox_reg->pdsp_mailbox_slot0);

			while (__raw_readl(&mailbox_reg->pdsp_mailbox_slot1) != 0);

			for (i = 1; i < 6; i++) {
				struct pa_mailbox_regs __iomem *mbox_reg =
					&pa_dev->reg_mailbox[i];
				__raw_writel(0,
					     &mbox_reg->pdsp_mailbox_slot0);
			}
		} else {
			for (i = 0; i < 6; i++) {
				struct pa_mailbox_regs __iomem *mbox_reg =
					&pa_dev->reg_mailbox[i];
				__raw_writel(0,
					     &mbox_reg->pdsp_mailbox_slot0);
			}

		}

		return (ret);
	}

	return (PA_STATE_INVALID_REQUEST);
}

static int keystone_pa_set_firmware(struct pa_device *pa_dev,
			     int pdsp, const unsigned int *buffer, int len)
{
	struct pa_pdsp_control_regs __iomem *ctrl_reg = &pa_dev->reg_control[pdsp];

	if ((pdsp < 0) || (pdsp >= DEVICE_PA_NUM_PDSPS))
		return -EINVAL;

	pdsp_fw_put((u32 *)(pa_dev->pa_iram + PA_MEM_PDSP_IRAM(pdsp)), buffer,
		    len >> 2);

	__raw_writel(pap_pdsp_const_reg_map[pdsp][PA_PDSP_CONST_REG_INDEX_C25_C24],
		     &ctrl_reg->const_tbl_blk_index0);

	__raw_writel(pap_pdsp_const_reg_map[pdsp][PA_PDSP_CONST_REG_INDEX_C27_C26],
		     &ctrl_reg->const_tbl_blk_index1);

	__raw_writel(pap_pdsp_const_reg_map[pdsp][PA_PDSP_CONST_REG_INDEX_C29_C28],
		     &ctrl_reg->const_tbl_prog_pointer0);

	__raw_writel(pap_pdsp_const_reg_map[pdsp][PA_PDSP_CONST_REG_INDEX_C31_C30],
		     &ctrl_reg->const_tbl_prog_pointer1);

	return 0;
}

static struct pa_packet *pa_alloc_packet(struct pa_device *pa_dev,
					 unsigned cmd_size,
					 enum dma_transfer_direction direction)
{
	struct pa_packet *p_info;

	p_info = kzalloc(sizeof(*p_info) + cmd_size, GFP_KERNEL);
	if (!p_info)
		return NULL;

	p_info->priv = pa_dev;
	p_info->data = p_info + 1;
	p_info->direction = direction;
	p_info->chan = (direction == DMA_MEM_TO_DEV) ? pa_dev->tx_channel :
		pa_dev->rx_channel;

	sg_init_table(p_info->sg, PA_SGLIST_SIZE);
	sg_set_buf(&p_info->sg[0], p_info->epib, sizeof(p_info->epib));
	sg_set_buf(&p_info->sg[1], p_info->psdata, sizeof(p_info->psdata));
	sg_set_buf(&p_info->sg[2], p_info->data, cmd_size);

	return p_info;
}

static void pa_tx_dma_callback(void *data)
{
	struct pa_packet *p_info = data;
	struct pa_device *pa_dev = p_info->priv;

	dma_unmap_sg(pa_dev->dev, &p_info->sg[2], 1, p_info->direction);

	p_info->desc = NULL;

	kfree(p_info);
}

static int pa_submit_tx_packet(struct pa_packet *p_info)
{
	unsigned flags = DMA_HAS_EPIB | DMA_HAS_PSINFO;
	struct pa_device *pa_dev = p_info->priv;
	int ret;

	ret = dma_map_sg(pa_dev->dev, &p_info->sg[2], 1, p_info->direction);
	if (ret < 0)
		return ret;

	p_info->desc = dmaengine_prep_slave_sg(p_info->chan, p_info->sg, 3,
					       p_info->direction, flags);
	if (IS_ERR_OR_NULL(p_info->desc)) {
		dma_unmap_sg(pa_dev->dev, &p_info->sg[2], 1, p_info->direction);
		return PTR_ERR(p_info->desc);
	}

	p_info->desc->callback = pa_tx_dma_callback;
	p_info->desc->callback_param = p_info;
	p_info->cookie = dmaengine_submit(p_info->desc);

	return 0;
}

#define	PA_CONTEXT_MASK		0xffff0000
#define	PA_CONTEXT_CONFIG	0xdead0000
#define	PA_CONTEXT_TSTAMP	0xbeef0000

#define	TSTAMP_TIMEOUT	(HZ * 5)	/* 5 seconds (arbitrary) */

struct tstamp_pending {
	struct list_head	 list;
	u32			 context;
	struct sock		*sock;
	struct sk_buff		*skb;
	struct pa_device	*pa_dev;
	struct timer_list	 timeout;
};

static spinlock_t		 tstamp_lock;
static atomic_t			 tstamp_sequence = ATOMIC_INIT(0);
static struct list_head		 tstamp_pending = LIST_HEAD_INIT(tstamp_pending);

static struct tstamp_pending *tstamp_remove_pending(u32 context)
{
	struct tstamp_pending	*pend;

	spin_lock(&tstamp_lock);
	list_for_each_entry(pend, &tstamp_pending, list) {
		if (pend->context == context) {
			del_timer(&pend->timeout);
			list_del(&pend->list);
			spin_unlock(&tstamp_lock);
			return pend;
		}
	}
	spin_unlock(&tstamp_lock);
	
	return NULL;
}

static void tstamp_complete(u32, struct pa_packet *);

static void tstamp_purge_pending(struct pa_device *pa_dev)
{
	struct tstamp_pending	*pend;
	int			 found;

	/* This is ugly and inefficient, but very rarely executed */
	do {
		found = 0;

		spin_lock(&tstamp_lock);
		list_for_each_entry(pend, &tstamp_pending, list) {
			if (pend->pa_dev == pa_dev) {
				found = 1;
				break;
			}
		}
		spin_unlock(&tstamp_lock);
		
		if (found)
			tstamp_complete(pend->context, NULL);
	} while(found);
}

static void tstamp_timeout(unsigned long context)
{
	tstamp_complete((u32)context, NULL);
}

static int tstamp_add_pending(struct tstamp_pending *pend)
{
	init_timer(&pend->timeout);
	pend->timeout.expires = jiffies + TSTAMP_TIMEOUT;
	pend->timeout.function = tstamp_timeout;
	pend->timeout.data = (unsigned long)pend->context;

	spin_lock(&tstamp_lock);
	add_timer(&pend->timeout);
	list_add_tail(&pend->list, &tstamp_pending);
	spin_unlock(&tstamp_lock);

	return 0;
}

static void tstamp_complete(u32 context, struct pa_packet *p_info)
{
	struct tstamp_pending	*pend;
	struct sock_exterr_skb 	*serr;
	struct sk_buff 		*skb;
	struct skb_shared_hwtstamps *sh_hw_tstamps;
	u64			 tx_timestamp;
	u64			 sys_time;
	int			 err;

	pend = tstamp_remove_pending(context);
	if (!pend)
		return;

	
	skb = pend->skb;
	if (!p_info) {
		printk("%s(%p,NULL): timeout\n", __func__, pend);
		kfree_skb(skb);
	} else {
		tx_timestamp = p_info->epib[0];
		tx_timestamp |= ((u64)(p_info->epib[2] & 0x0000ffff)) << 32;

		sys_time = pa_to_sys_time(pend->pa_dev->pa2system_offset, tx_timestamp);

		sh_hw_tstamps = skb_hwtstamps(skb);
		memset(sh_hw_tstamps, 0, sizeof(*sh_hw_tstamps));
		sh_hw_tstamps->hwtstamp = ns_to_ktime(tstamp_raw_to_ns(tx_timestamp));
		sh_hw_tstamps->syststamp = ns_to_ktime(sys_time);

		serr = SKB_EXT_ERR(skb);
		memset(serr, 0, sizeof(*serr));
		serr->ee.ee_errno = ENOMSG;
		serr->ee.ee_origin = SO_EE_ORIGIN_TIMESTAMPING;

		err = sock_queue_err_skb(pend->sock, skb);
		if (err)
			kfree_skb(skb);
	}

	kfree(pend);
}

static void pa_rx_complete(void *param)
{
	struct pa_packet *p_info = param;
	struct pa_device *pa_dev = p_info->priv;
	struct pa_frm_command *fcmd;

	dma_unmap_sg(pa_dev->dev, &p_info->sg[2], 1, DMA_FROM_DEVICE);

	switch (p_info->epib[1] & PA_CONTEXT_MASK) {
	case PA_CONTEXT_CONFIG:
		fcmd = p_info->data;
		swizFcmd(fcmd);

		if (fcmd->command_result != PAFRM_COMMAND_RESULT_SUCCESS) {
			dev_dbg(pa_dev->dev, "Command Result = 0x%x\n", fcmd->command_result);
			dev_dbg(pa_dev->dev, "Command = 0x%x\n", fcmd->command);
			dev_dbg(pa_dev->dev, "Magic = 0x%x\n", fcmd->magic);
			dev_dbg(pa_dev->dev, "Com ID = 0x%x\n", fcmd->com_id);
			dev_dbg(pa_dev->dev, "ret Context = 0x%x\n", fcmd->ret_context);
			dev_dbg(pa_dev->dev, "Flow ID = 0x%x\n", fcmd->flow_id);
			dev_dbg(pa_dev->dev, "reply Queue = 0x%x\n", fcmd->reply_queue);
			dev_dbg(pa_dev->dev, "reply dest = 0x%x\n", fcmd->reply_dest);
		}
		dev_dbg(pa_dev->dev, "command response complete\n");
		break;

	case PA_CONTEXT_TSTAMP:
		tstamp_complete(p_info->epib[1], p_info);
		break;

	default:
		dev_warn(pa_dev->dev, "bad response context, got 0x%08x\n", p_info->epib[1]);
		break;
	}

	p_info->desc = NULL;
	kfree(p_info);
}

/* Release a free receive buffer */
static void pa_rxpool_free(void *arg, unsigned q_num, unsigned bufsize,
		struct dma_async_tx_descriptor *desc)
{
	struct pa_device *pa_dev = arg;
	struct pa_packet *p_info = desc->callback_param;

	dma_unmap_sg(pa_dev->dev, &p_info->sg[2], 1, DMA_FROM_DEVICE);

	p_info->desc = NULL;

	kfree(p_info);
}

static void pa_chan_work_handler(unsigned long data)
{
	struct pa_device *pa_dev = (struct pa_device *)data;

	dma_poll(pa_dev->rx_channel, -1);

	dma_rxfree_refill(pa_dev->rx_channel);

	dmaengine_resume(pa_dev->rx_channel);
}

static void pa_chan_notify(struct dma_chan *dma_chan, void *arg)
{
	struct pa_device *pa_dev = arg;

	dmaengine_pause(pa_dev->rx_channel);

	tasklet_schedule(&pa_dev->task);

	return;
}

/* Allocate a free receive buffer */
static struct dma_async_tx_descriptor *pa_rxpool_alloc(void *arg,
		unsigned q_num, unsigned bufsize)
{
	struct pa_device *pa_dev = arg;
	struct dma_async_tx_descriptor *desc;
	struct dma_device *device;
	u32 err = 0;

	struct pa_packet *rx;

	rx = pa_alloc_packet(pa_dev, bufsize, DMA_DEV_TO_MEM);
	if (!rx) {
		dev_err(pa_dev->dev, "could not allocate cmd rx packet\n");
		kfree(rx);
		return NULL;
	}

	rx->sg_ents = 2 + dma_map_sg(pa_dev->dev, &rx->sg[2],
				1, DMA_FROM_DEVICE);
	if (rx->sg_ents != 3) {
		dev_err(pa_dev->dev, "dma map failed\n");

		kfree(rx);
		return NULL;
	}

	device = rx->chan->device;

	desc = dmaengine_prep_slave_sg(rx->chan, rx->sg, 3, DMA_DEV_TO_MEM,
				       DMA_HAS_EPIB | DMA_HAS_PSINFO);

	if (IS_ERR_OR_NULL(desc)) {
		dma_unmap_sg(pa_dev->dev, &rx->sg[2], 1, DMA_FROM_DEVICE);
		kfree(rx);
		err = PTR_ERR(desc);
		if (err != -ENOMEM) {
			dev_err(pa_dev->dev,
				"dma prep failed, error %d\n", err);
		}
		
		return NULL;
	}

	desc->callback_param = rx;
	desc->callback = pa_rx_complete;
	rx->cookie = desc->cookie;

	return desc;
}

static int keystone_pa_add_mac(struct pa_device *priv, const u8 *mac,
			int rule, unsigned etype, int index)
{
	struct pa_route_info route_info, fail_info;
	struct pa_frm_command *fcmd;
	struct pa_frm_cmd_add_lut1 *al1;
	struct pa_packet *tx;
	u32 context = PA_CONTEXT_CONFIG;
	int size, ret;

	memset(&fail_info, 0, sizeof(fail_info));

	memset(&route_info, 0, sizeof(route_info));

	if (rule == PACKET_HST) {
		route_info.dest			= PA_DEST_HOST;
		route_info.flow_id		= priv->data_flow_num;
		route_info.queue		= priv->data_queue_num;
		route_info.m_route_index	= -1;
		fail_info.dest			= PA_DEST_HOST;
		fail_info.flow_id		= priv->data_flow_num;
		fail_info.queue			= priv->data_queue_num;
		fail_info.m_route_index		= -1;
	} else if (rule == PACKET_PARSE) {
		route_info.dest			= PA_DEST_CONTINUE_PARSE_LUT1;
		route_info.m_route_index	= -1;
		fail_info.dest			= PA_DEST_HOST;
		fail_info.flow_id		= priv->data_flow_num;
		fail_info.queue			= priv->data_queue_num;
		fail_info.m_route_index		= -1;
	} else if (rule == PACKET_DROP) {
		route_info.dest			= PA_DEST_DISCARD;
		route_info.m_route_index	= -1;
		fail_info.dest			= PA_DEST_DISCARD;
		fail_info.m_route_index		= -1;
	}

	size = (sizeof(struct pa_frm_command) +
		sizeof(struct pa_frm_cmd_add_lut1) + 4);
	tx = pa_alloc_packet(priv, size, DMA_MEM_TO_DEV);
	if (!tx) {
		dev_err(priv->dev, "could not allocate cmd tx packet\n");
		return -ENOMEM;
	}

	fcmd = tx->data;
	al1 = (struct pa_frm_cmd_add_lut1 *) &(fcmd->cmd);

	fcmd->command_result	= 0;
	fcmd->command		= PAFRM_CONFIG_COMMAND_ADDREP_LUT1;
	fcmd->magic		= PAFRM_CONFIG_COMMAND_SEC_BYTE;
	fcmd->com_id		= PA_COMID_L2;
	fcmd->ret_context	= context;
	fcmd->flow_id		= priv->cmd_flow_num;
	fcmd->reply_queue	= priv->cmd_queue_num;
	fcmd->reply_dest	= PAFRM_DEST_PKTDMA;

	al1->index		= index;
	al1->type		= PAFRM_COM_ADD_LUT1_STANDARD;

	if (etype) {
		al1->u.eth_ip.etype	= etype;
		al1->u.eth_ip.match_flags |= PAFRM_LUT1_CUSTOM_MATCH_ETYPE;
	}

	al1->u.eth_ip.vlan	= 0;
	al1->u.eth_ip.pm.mpls	= 0;

	if (mac) {
		al1->u.eth_ip.dmac[0] = mac[0];
		al1->u.eth_ip.dmac[1] = mac[1];
		al1->u.eth_ip.dmac[2] = mac[2];
		al1->u.eth_ip.dmac[3] = mac[3];
		al1->u.eth_ip.dmac[4] = mac[4];
		al1->u.eth_ip.dmac[5] = mac[5];
		al1->u.eth_ip.key |= PAFRM_LUT1_KEY_MAC;
	}

	al1->u.eth_ip.smac[0] = 0;
	al1->u.eth_ip.smac[1] = 0;
	al1->u.eth_ip.smac[2] = 0;
	al1->u.eth_ip.smac[3] = 0;
	al1->u.eth_ip.smac[4] = 0;
	al1->u.eth_ip.smac[5] = 0;
	ret = pa_conv_routing_info(&al1->match, &route_info, 0, 0);
	if (ret != 0)
		dev_err(priv->dev, "route info config failed\n");

	ret = pa_conv_routing_info(&al1->next_fail, &fail_info, 0, 1);
	if (ret != 0)
		dev_err(priv->dev, "fail info config failed\n");

	swizFcmd(fcmd);
	swizAl1((struct pa_frm_cmd_add_lut1 *)&(fcmd->cmd));

	tx->psdata[0] = ((u32)(4 << 5) << 24);

	tx->epib[1] = 0x11112222;
	tx->epib[2] = 0x33334444;
	tx->epib[3] = 0;

	pa_submit_tx_packet(tx);
	dev_dbg(priv->dev, "waiting for command transmit complete\n");

	return 0;
}

static int pa_fmtcmd_next_route(struct netcp_packet *p_info, const struct pa_cmd_next_route *route)
{
	struct pasaho_next_route	*nr;
	int	size;
	u16	pdest;
	
	/* Make sure the destination is valid */
	switch (route->dest) {
	case PA_DEST_HOST:
		pdest = PAFRM_DEST_PKTDMA;
		break;
	case PA_DEST_EMAC:
		pdest = PAFRM_DEST_ETH;
		break;
	default:
		return -EINVAL;
	}

	size = route->pkt_type_emac_ctrl ? sizeof(*nr) : (sizeof(*nr) - sizeof(nr->word1));
	nr = (struct pasaho_next_route *)netcp_push_psdata(p_info, size);
	if (!nr)
		return -ENOMEM;

	if (route->pkt_type_emac_ctrl) {
		u8 ps_flags;
		PASAHO_SET_E(nr, 1);

		ps_flags = (route->pkt_type_emac_ctrl & PA_EMAC_CTRL_CRC_DISABLE) ?
				PAFRM_ETH_PS_FLAGS_DISABLE_CRC : 0;

		ps_flags |= ((route->pkt_type_emac_ctrl & PA_EMAC_CTRL_PORT_MASK) <<
				PAFRM_ETH_PS_FLAGS_PORT_SHIFT);  

		PASAHO_SET_PKTTYPE(nr, ps_flags);
	}

	PASAHO_SET_CMDID(nr, PASAHO_PAMOD_NROUTE);
	PASAHO_SET_DEST(nr, pdest);
	PASAHO_SET_FLOW(nr, route->flow_id);
	PASAHO_SET_QUEUE (nr, route->queue);

	if (route->ctrl_bit_field & PA_NEXT_ROUTE_PROC_NEXT_CMD)
		PASAHO_SET_N  (nr, 1);

	nr->sw_info0 = route->sw_info_0;
	nr->sw_info1 = route->sw_info_1;
	
	return size;
}

static int pa_fmtcmd_tx_timestamp(struct netcp_packet *p_info, const struct pa_cmd_tx_timestamp *tx_ts)
{
	struct pasaho_report_timestamp	*rt_info;
	int				 size;

	size = sizeof(*rt_info);
	rt_info = (struct pasaho_report_timestamp *)netcp_push_psdata(p_info, size);
	if (!rt_info)
		return -ENOMEM;

	PASAHO_SET_CMDID(rt_info, PASAHO_PAMOD_REPORT_TIMESTAMP);
	PASAHO_SET_REPORT_FLOW(rt_info, (u8)tx_ts->flow_id);
	PASAHO_SET_REPORT_QUEUE(rt_info, tx_ts->dest_queue);
	rt_info->sw_info0 = tx_ts->sw_info0;
	
	return size;
}

static int pa_fmtcmd_align(struct netcp_packet *p_info, const unsigned bytes)
{
	struct pasaho_cmd_info	*paCmdInfo;
	int i;
	
	if ((bytes & 0x03) != 0)
		return -EINVAL;
	
	paCmdInfo = (struct pasaho_cmd_info *)netcp_push_psdata(p_info, bytes);

	for (i = bytes/sizeof(u32); i > 0; --i ) {
		PASAHO_SET_CMDID(paCmdInfo, PASAHO_PAMOD_DUMMY);
		++paCmdInfo;
	}
	
	return bytes;
}

#define	PA_TXHOOK_ORDER	10
#define	PA_RXHOOK_ORDER	10

static int pa_tx_hook(int order, void *data, struct netcp_packet *p_info)
{
	struct netcp_module_data *module = data;
	struct pa_device *pa_dev = pa_from_module(module);
	static const struct pa_cmd_next_route route_cmd = {
					0,		/*  ctrlBitfield */
					PA_DEST_EMAC,	/* Route - host */
					0,              /* pktType don't care */
					0,              /* flow Id */
					0,  		/* Queue */
					0,              /* SWInfo 0 */
					0,              /* SWInfo 1 */
					0,
					};
	struct pa_cmd_tx_timestamp tx_ts;
	int align, total = 0;

	total += pa_fmtcmd_next_route(p_info, &route_cmd);

	if ((skb_shinfo(p_info->skb)->tx_flags & SKBTX_HW_TSTAMP) && p_info->skb->sk) {
		struct tstamp_pending	*pend;
		
		pend = kzalloc(sizeof(*pend), GFP_ATOMIC);
		if (pend) {
			pend->skb = skb_clone(p_info->skb, GFP_ATOMIC);
			if (!pend->skb)
				kfree(pend);
			else {
				pend->sock = p_info->skb->sk;
				pend->pa_dev = pa_dev;
				pend->context =  PA_CONTEXT_TSTAMP | 
					(~PA_CONTEXT_MASK & atomic_inc_return(&tstamp_sequence));
				tstamp_add_pending(pend);

				memset(&tx_ts, 0, sizeof(tx_ts));
				tx_ts.dest_queue = pa_dev->cmd_queue_num;
				tx_ts.flow_id    = pa_dev->cmd_flow_num;
				tx_ts.sw_info0   = pend->context;

				total += pa_fmtcmd_tx_timestamp(p_info, &tx_ts);
			}
		}
	}

	align = netcp_align_psdata(p_info, 8);
	if (align)
		total += pa_fmtcmd_align(p_info, align);

	p_info->tx_pipe = &pa_dev->tx_pipe;
	return 0;
}

static int pa_rx_timestamp(int order, void *data, struct netcp_packet *p_info)
{
	struct netcp_module_data *module = data;
	struct pa_device *pa_dev = pa_from_module(module);
	struct netcp_priv *netcp = module->priv;
	struct sk_buff *skb = p_info->skb;
	struct skb_shared_hwtstamps *sh_hw_tstamps;
	u64 rx_timestamp;
	u64 sys_time;

	if (!netcp->hwts_rx_en)
		return 0;

	rx_timestamp = p_info->epib[0];
	rx_timestamp |= ((u64)(p_info->psdata[4] & 0x0000ffff)) << 32;

	sys_time = pa_to_sys_time(pa_dev->pa2system_offset, rx_timestamp);

	sh_hw_tstamps = skb_hwtstamps(skb);
	memset(sh_hw_tstamps, 0, sizeof(*sh_hw_tstamps));
	sh_hw_tstamps->hwtstamp = ns_to_ktime(tstamp_raw_to_ns(rx_timestamp));
	sh_hw_tstamps->syststamp = ns_to_ktime(sys_time);

	return 0;
}

static int pa_close(struct netcp_module_data *data)
{
	struct pa_device *pa_dev = pa_from_module(data);

	netcp_unregister_txhook(data->priv, PA_TXHOOK_ORDER, pa_tx_hook, data);
	netcp_unregister_rxhook(data->priv, PA_RXHOOK_ORDER, pa_rx_timestamp, data);

	tasklet_disable(&pa_dev->task);
	
	tstamp_purge_pending(pa_dev);

	if (pa_dev->tx_channel) {
		dmaengine_pause(pa_dev->tx_channel);
		dma_release_channel(pa_dev->tx_channel);
		pa_dev->tx_channel = NULL;
	}

	if (pa_dev->tx_pipe.dma_channel) {
		dmaengine_pause(pa_dev->tx_pipe.dma_channel);
		dma_release_channel(pa_dev->tx_pipe.dma_channel);
		pa_dev->tx_pipe.dma_channel = NULL;
	}

	if (pa_dev->rx_channel) {
		dmaengine_pause(pa_dev->rx_channel);
		dma_release_channel(pa_dev->rx_channel);
		pa_dev->rx_channel = NULL;
	}

	if (pa_dev->clk) {
		clk_disable_unprepare(pa_dev->clk);
		clk_put(pa_dev->clk);
	}
	pa_dev->clk = NULL;

	return 0;
}

static int pa_open(struct netcp_module_data *data, struct net_device *ndev)
{
	struct pa_device *pa_dev = pa_from_module(data);
	const struct firmware *fw;
	const u8 bcast_addr[6] = { 0xff, 0xff, 0xff, 0xff, 0xff, 0xff };
	struct dma_chan *chan;
	struct dma_keystone_info config;
	dma_cap_mask_t mask;
	int i, factor;
	int ret, err;

	pa_dev->clk = clk_get(pa_dev->dev, "clk_pa");
	if (IS_ERR_OR_NULL(pa_dev->clk)) {
		dev_err(pa_dev->dev, "unable to get Packet Accelerator clock\n");
		pa_dev->clk = NULL;
		return -EBUSY;
	}

	clk_prepare_enable(pa_dev->clk);

	keystone_pa_reset(pa_dev);

	/* Configure the streaming switch */
	__raw_writel(PSTREAM_ROUTE_PDSP0, pa_dev->streaming_switch);

	for (i = 0; i <= 5; i++) {
		if (i <= 2)
			ret = request_firmware(&fw, DEVICE_PA_PDSP02_FIRMWARE, pa_dev->dev);
		else if (i == 3)
			ret = request_firmware(&fw, DEVICE_PA_PDSP3_FIRMWARE, pa_dev->dev);
		else if (i > 3)
			ret = request_firmware(&fw, DEVICE_PA_PDSP45_FIRMWARE, pa_dev->dev);
		if (ret != 0) {
			dev_err(pa_dev->dev, "cannot find firmware for pdsp %d\n", i);
			ret = -ENODEV;
			goto fail;
		}

		/* Download the firmware to the PDSP */
		keystone_pa_set_firmware(pa_dev, i,
					 (const unsigned int*) fw->data,
					 fw->size);

		release_firmware(fw);
	}

	ret = keystone_pa_reset_control(pa_dev, PA_STATE_ENABLE);
	if (ret != 1) {
		dev_err(pa_dev->dev, "enabling failed, ret = %d\n", ret);
		ret = -ENODEV;
		goto fail;
	}

	pa_get_version(pa_dev);

	factor = PA_TIMESTAMP_SCALER_FACTOR_2;

	ret = pa_config_timestamp(pa_dev, factor);
	if (ret != 0) {
		dev_err(pa_dev->dev, "timestamp configuration failed, ret = %d\n", ret);
		ret = -ENODEV;
		goto fail;
	}

	pa_dev->pa2system_offset = 0;
	pa_calibrate_with_system_timer(pa_dev);

	dma_cap_zero(mask);
	dma_cap_set(DMA_SLAVE, mask);

	/* Open the PA Command transmit channel */
	pa_dev->tx_channel = dma_request_channel_by_name(mask, "patx-cmd");
	if (IS_ERR_OR_NULL(pa_dev->tx_channel)) {
		dev_err(pa_dev->dev, "Could not get PA TX command channel\n");
		pa_dev->tx_channel = NULL;
		ret = -ENODEV;
		goto fail;
	}

	memset(&config, 0, sizeof(config));

	config.direction	= DMA_MEM_TO_DEV;
	config.tx_queue_depth	= pa_dev->tx_cmd_queue_depth;

	err = dma_keystone_config(pa_dev->tx_channel, &config);
	if (err)
		goto fail;

	/* Open the PA Data transmit channel */
	pa_dev->tx_pipe.dma_chan_name = "patx-dat";
	pa_dev->tx_pipe.dma_channel = dma_request_channel_by_name(mask, pa_dev->tx_pipe.dma_chan_name);
	if (IS_ERR_OR_NULL(pa_dev->tx_pipe.dma_channel)) {
		dev_err(pa_dev->dev, "Could not get PA TX data channel\n");
		pa_dev->tx_pipe.dma_channel = NULL;
		ret = -ENODEV;
		goto fail;
	}

	memset(&config, 0, sizeof(config));

	config.direction	= DMA_MEM_TO_DEV;
	config.tx_queue_depth	= pa_dev->tx_data_queue_depth;

	err = dma_keystone_config(pa_dev->tx_pipe.dma_channel, &config);
	if (err)
		goto fail;

	pa_dev->tx_pipe.dma_queue = dma_get_tx_queue(pa_dev->tx_pipe.dma_channel);
	pa_dev->tx_pipe.dma_poll_threshold = config.tx_queue_depth / 2;
	atomic_set(&pa_dev->tx_pipe.dma_poll_count, pa_dev->tx_pipe.dma_poll_threshold);


	/* Open the PA common response channel */
	pa_dev->rx_channel = dma_request_channel_by_name(mask, "parx");
	if (IS_ERR_OR_NULL(pa_dev->rx_channel)) {
		dev_err(pa_dev->dev, "Could not get PA RX channel\n");
		pa_dev->rx_channel = NULL;
		ret = -ENODEV;
		goto fail;
	}

	memset(&config, 0, sizeof(config));

	config.direction		= DMA_DEV_TO_MEM;
	config.scatterlist_size		= PA_SGLIST_SIZE;
	config.rxpool_allocator		= pa_rxpool_alloc;
	config.rxpool_destructor	= pa_rxpool_free;
	config.rxpool_param		= pa_dev;
	config.rxpool_count		= 1;
	config.rxpool_thresh_enable	= DMA_THRESH_NONE;
	config.rxpools[0].pool_depth	= pa_dev->rx_pool_depth;
	config.rxpools[0].buffer_size	= pa_dev->rx_buffer_size;

	err = dma_keystone_config(pa_dev->rx_channel, &config);
	if (err)
		goto fail;

	tasklet_init(&pa_dev->task, pa_chan_work_handler,
		     (unsigned long) pa_dev);

	dma_set_notify(pa_dev->rx_channel, pa_chan_notify, pa_dev);

	chan = netcp_get_rx_chan(data->priv);

	pa_dev->data_flow_num = dma_get_rx_flow(chan);
	pa_dev->data_queue_num = dma_get_rx_queue(chan);
	pa_dev->cmd_flow_num = dma_get_rx_flow(pa_dev->rx_channel);
	pa_dev->cmd_queue_num = dma_get_rx_queue(pa_dev->rx_channel);

	dev_dbg(pa_dev->dev, "configuring command receive flow %d, queue %d\n",
		pa_dev->cmd_flow_num, pa_dev->cmd_queue_num);

	pa_dev->addr_count = 0;

	dma_rxfree_refill(pa_dev->rx_channel);

	ret = keystone_pa_add_mac(pa_dev, NULL,       PACKET_HST,  0,      63);
	ret = keystone_pa_add_mac(pa_dev, bcast_addr, PACKET_HST,  0,      62);
	ret = keystone_pa_add_mac(pa_dev, ndev->dev_addr,   PACKET_HST,  0,      61);
	ret = keystone_pa_add_mac(pa_dev, ndev->dev_addr,   PACKET_PARSE, 0x0800, 60);
	ret = keystone_pa_add_mac(pa_dev, ndev->dev_addr,   PACKET_PARSE, 0x86dd, 59);
	pa_dev->addr_count = 5;

	netcp_register_txhook(data->priv, PA_TXHOOK_ORDER, pa_tx_hook, data);
	netcp_register_rxhook(data->priv, PA_RXHOOK_ORDER, pa_rx_timestamp, data);
	return 0;

fail:
	pa_close(data);
	return ret;
}

static int pa_remove(struct netcp_module_data *data)
{
	struct pa_device *pa_dev = pa_from_module(data);
	struct device *dev = pa_dev->dev;

	devm_iounmap(dev, pa_dev->reg_mailbox);
	devm_iounmap(dev, pa_dev->reg_mailbox);
	devm_iounmap(dev, pa_dev->reg_packet_id);
	devm_iounmap(dev, pa_dev->reg_lut2);
	devm_iounmap(dev, pa_dev->reg_control);
	devm_iounmap(dev, pa_dev->reg_timer);
	devm_iounmap(dev, pa_dev->reg_stats);
	devm_iounmap(dev, pa_dev->pa_iram);
	devm_iounmap(dev, pa_dev->pa_sram);
	devm_iounmap(dev, pa_dev->streaming_switch);

	kfree(pa_dev);

	return 0;
}

static struct netcp_module_data *pa_probe(struct device *dev,
					  struct device_node *node)
{
	struct pa_device *pa_dev;
	int ret = 0;

	pa_dev = devm_kzalloc(dev, sizeof(struct pa_device), GFP_KERNEL);
	if (!pa_dev) {
		dev_err(dev, "memory allocation failed\n");
		ret = -ENOMEM;
		goto exit;
	}

	if (!node) {
		dev_err(dev, "device tree info unavailable\n");
		ret = -ENODEV;
		goto exit;
	}

	pa_dev->dev = dev;

	ret = of_property_read_u32(node, "tx_cmd_queue_depth",
				   &pa_dev->tx_cmd_queue_depth);
	if (ret < 0) {
		dev_err(dev, "missing tx_cmd_queue_depth parameter, err %d\n",
			ret);
		pa_dev->tx_cmd_queue_depth = 32;
	}
	dev_dbg(dev, "tx_cmd_queue_depth %u\n", pa_dev->tx_cmd_queue_depth);

	ret = of_property_read_u32(node, "tx_data_queue_depth",
				   &pa_dev->tx_data_queue_depth);
	if (ret < 0) {
		dev_err(dev, "missing tx_data_queue_depth parameter, err %d\n",
			ret);
		pa_dev->tx_data_queue_depth = 32;
	}
	dev_dbg(dev, "tx_data_queue_depth %u\n", pa_dev->tx_data_queue_depth);

	ret = of_property_read_u32(node, "rx_pool_depth",
				   &pa_dev->rx_pool_depth);
	if (ret < 0) {
		dev_err(dev, "missing rx_pool_depth parameter, err %d\n",
			ret);
		pa_dev->rx_pool_depth = 32;
	}
	dev_dbg(dev, "rx_pool_depth %u\n", pa_dev->rx_pool_depth);

	ret = of_property_read_u32(node, "rx_buffer_size",
				   &pa_dev->rx_buffer_size);
	if (ret < 0) {
		dev_err(dev, "missing rx_buffer_size parameter, err %d\n",
			ret);
		pa_dev->rx_buffer_size = 128;
	}
	dev_dbg(dev, "rx_buffer_size %u\n", pa_dev->rx_buffer_size);

	pa_dev->reg_mailbox	= devm_ioremap(dev, 0x2000000, 0x60);
	pa_dev->reg_packet_id	= devm_ioremap(dev, 0x2000400, 0x10);
	pa_dev->reg_lut2	= devm_ioremap(dev, 0x2000500, 0x40);
	pa_dev->reg_control	= devm_ioremap(dev, 0x2001000, 0x600);
	pa_dev->reg_timer	= devm_ioremap(dev, 0x2003000, 0x600);
	pa_dev->reg_stats	= devm_ioremap(dev, 0x2006000, 0x100);
	pa_dev->pa_iram		= devm_ioremap(dev, 0x2010000, 0x30000);
	pa_dev->pa_sram		= devm_ioremap(dev, 0x2040000, 0x8000);
	pa_dev->streaming_switch	= devm_ioremap(dev, 0x02000604, 4);

	if (!pa_dev->reg_mailbox || !pa_dev->reg_packet_id ||
	    !pa_dev->reg_lut2 || !pa_dev->reg_control ||
	    !pa_dev->reg_timer || !pa_dev->reg_stats ||
	    !pa_dev->pa_sram || !pa_dev->pa_iram ||
	    !pa_dev->streaming_switch) {
		dev_err(dev, "failed to set up register areas\n");
		ret = -ENOMEM;
		goto exit;
	}

	spin_lock_init(&pa_dev->lock);
	spin_lock_init(&tstamp_lock);

	pa_dev->module.open		= pa_open;
	pa_dev->module.close		= pa_close;
	pa_dev->module.remove		= pa_remove;

	return pa_to_module(pa_dev);
exit:
	return NULL;
}

static struct netcp_module pa_module = {
	.name	= "keystone-pa",
	.owner	= THIS_MODULE,
	.probe	= pa_probe,
};

static int __init keystone_pa_init(void)
{
	return netcp_register_module(&pa_module);
}
subsys_initcall(keystone_pa_init);

static void __exit keystone_pa_exit(void)
{
	netcp_unregister_module(&pa_module);
}
module_exit(keystone_pa_exit);

MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Sandeep Paulraj <s-paulraj@ti.com>");
MODULE_DESCRIPTION("Packet Accelerator driver for Keystone devices");
