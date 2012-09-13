/*
 * Copyright (C) 2012 Texas Instruments Incorporated
 * Authors: Cyril Chemparathy <cyril@ti.com>
 *	    Sandeep Paulraj <s-paulraj@ti.com>
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
#include <linux/net_tstamp.h>
#include <linux/of_net.h>
#include <linux/of_address.h>
#include <linux/dma-mapping.h>
#include <linux/scatterlist.h>
#include <linux/etherdevice.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/keystone-dma.h>

#include "keystone_net.h"

/* Read the e-fuse value as 32 bit values to be endian independent */
static inline int emac_arch_get_mac_addr(char *x,
					 void __iomem *efuse_mac)
{
	unsigned int addr0, addr1;

	addr1 = __raw_readl(efuse_mac + 4);
	addr0 = __raw_readl(efuse_mac);

	x[0] = (addr1 & 0x0000ff00) >> 8;
	x[1] = addr1 & 0x000000ff;
	x[2] = (addr0 & 0xff000000) >> 24;
	x[3] = (addr0 & 0x00ff0000) >> 16;
	x[4] = (addr0 & 0x0000ff00) >> 8;
	x[5] = addr0 & 0x000000ff;

	return 0;
}

static LIST_HEAD(netcp_modules);
static DEFINE_MUTEX(netcp_modules_lock);

static unsigned int sg_count(struct scatterlist *sg, unsigned int max_ents)
{
	unsigned int count;

	for (count = 0; sg && (count < max_ents); count++, sg = sg_next(sg))
		;

	return count;
}

#define for_each_netcp_module(module)			\
	list_for_each_entry(module, &netcp_modules, list)

int netcp_register_module(struct netcp_module *module)
{
	struct netcp_module *tmp;
	int ret;

	BUG_ON(!module->name);
	BUG_ON(!module->probe);

	mutex_lock(&netcp_modules_lock);

	ret = -EEXIST;
	for_each_netcp_module(tmp) {
		if (!strcasecmp(tmp->name, module->name))
			goto found;
	}

	list_add_tail(&module->list, &netcp_modules);

found:
	mutex_unlock(&netcp_modules_lock);
	return ret;
}
EXPORT_SYMBOL(netcp_register_module);

static struct netcp_module *netcp_find_module(const char *name)
{
	struct netcp_module *tmp;
	mutex_lock(&netcp_modules_lock);
	for_each_netcp_module(tmp) {
		if (!strcasecmp(tmp->name, name))
			goto found;
	}
	mutex_unlock(&netcp_modules_lock);
	return NULL;
found:
	mutex_unlock(&netcp_modules_lock);
	return tmp;
}

void netcp_unregister_module(struct netcp_module *module)
{
	if (module == netcp_find_module(module->name)) {
		mutex_lock(&netcp_modules_lock);
		list_del(&module->list);
		mutex_unlock(&netcp_modules_lock);
	}
}
EXPORT_SYMBOL(netcp_unregister_module);

struct netcp_hook_list {
	struct list_head	 list;
	netcp_hook_rtn		*hook_rtn;
	void			*hook_data;
	int			 order;
};


int netcp_register_txhook(struct netcp_priv *netcp_priv, int order,
		netcp_hook_rtn *hook_rtn, void *hook_data)
{
	struct netcp_hook_list	*entry;
	struct netcp_hook_list	*next;
	
	entry = kzalloc(sizeof(*entry), GFP_KERNEL);
	if (!entry)
		return -ENOMEM;

	entry->hook_rtn  = hook_rtn;
	entry->hook_data = hook_data;
	entry->order     = order;

	list_for_each_entry(next, &netcp_priv->txhook_list_head, list) {
		if (next->order > order)
			break;
	}
	__list_add(&entry->list, next->list.prev, &next->list);

	return 0;
}
EXPORT_SYMBOL(netcp_register_txhook);

int netcp_unregister_txhook(struct netcp_priv *netcp_priv, int order,
		netcp_hook_rtn *hook_rtn, void *hook_data)
{
	struct netcp_hook_list	*next;
	
	list_for_each_entry(next, &netcp_priv->txhook_list_head, list) {
		if ((next->order     == order) &&
		    (next->hook_rtn  == hook_rtn) &&
		    (next->hook_data == hook_data)) {
			list_del(&next->list);
			kfree(next);
			return 0;
		}
	}

	return -ENOENT;
}
EXPORT_SYMBOL(netcp_unregister_txhook);

int netcp_register_rxhook(struct netcp_priv *netcp_priv, int order,
		netcp_hook_rtn *hook_rtn, void *hook_data)
{
	struct netcp_hook_list	*entry;
	struct netcp_hook_list	*next;
	
	entry = kzalloc(sizeof(*entry), GFP_KERNEL);
	if (!entry)
		return -ENOMEM;

	entry->hook_rtn  = hook_rtn;
	entry->hook_data = hook_data;
	entry->order     = order;

	list_for_each_entry(next, &netcp_priv->rxhook_list_head, list) {
		if (next->order > order)
			break;
	}
	__list_add(&entry->list, next->list.prev, &next->list);

	return 0;
}
EXPORT_SYMBOL(netcp_register_rxhook);

int netcp_unregister_rxhook(struct netcp_priv *netcp_priv, int order,
		netcp_hook_rtn *hook_rtn, void *hook_data)
{
	struct netcp_hook_list	*next;
	
	list_for_each_entry(next, &netcp_priv->rxhook_list_head, list) {
		if ((next->order     == order) &&
		    (next->hook_rtn  == hook_rtn) &&
		    (next->hook_data == hook_data)) {
			list_del(&next->list);
			kfree(next);
			return 0;
		}
	}

	return -ENOENT;
}
EXPORT_SYMBOL(netcp_unregister_rxhook);

u32 *netcp_push_psdata(struct netcp_packet *p_info, unsigned bytes)
{
	u32		*buf;
	unsigned	 words;

	if ((bytes & 0x03) != 0)
		return NULL;
	words = bytes >> 2;
	
	if ((p_info->psdata_len + words) > NETCP_PSDATA_LEN)
		return NULL;

	p_info->psdata_len += words;
	buf = &p_info->psdata[NETCP_PSDATA_LEN - p_info->psdata_len];

	memset(buf, 0, bytes);
	
	return buf;
}
EXPORT_SYMBOL(netcp_push_psdata);

int netcp_align_psdata(struct netcp_packet *p_info, unsigned byte_align)
{
	int	padding;

	switch (byte_align) {
	case 0:
		padding = -1;
		break;
	case 1:
		padding = 0;
		break;
	default:
		padding = (p_info->psdata_len << 2) % byte_align;
		break;
	}
	
	return padding;
}
EXPORT_SYMBOL(netcp_align_psdata);

#define NETCP_DEBUG (NETIF_MSG_HW	| NETIF_MSG_WOL		|	\
		    NETIF_MSG_DRV	| NETIF_MSG_LINK	|	\
		    NETIF_MSG_IFUP	| NETIF_MSG_INTR	|	\
		    NETIF_MSG_PROBE	| NETIF_MSG_TIMER	|	\
		    NETIF_MSG_IFDOWN	| NETIF_MSG_RX_ERR	|	\
		    NETIF_MSG_TX_ERR	| NETIF_MSG_TX_DONE	|	\
		    NETIF_MSG_PKTDATA	| NETIF_MSG_TX_QUEUED	|	\
		    NETIF_MSG_RX_STATUS)

#define NETCP_NAPI_WEIGHT	128
#define NETCP_TX_TIMEOUT	40
#define NETCP_MIN_PACKET_SIZE	64
#define NETCP_MAX_PACKET_SIZE	(VLAN_ETH_FRAME_LEN + ETH_FCS_LEN)

static int netcp_rx_packet_max = NETCP_MAX_PACKET_SIZE;
static int netcp_debug_level;

#define for_each_module(netcp, module)			\
	list_for_each_entry(module, &netcp->modules, list)
#define for_each_module_safe(netcp, module, tmp)	\
	list_for_each_entry_safe(module, tmp, &netcp->modules, list)

static const char *netcp_rx_state_str(struct netcp_priv *netcp)
{
	static const char * const state_str[] = {
		[RX_STATE_POLL]		= "poll",
		[RX_STATE_SCHEDULED]	= "scheduled",
		[RX_STATE_TEARDOWN]	= "teardown",
		[RX_STATE_INTERRUPT]	= "interrupt",
		[RX_STATE_INVALID]	= "invalid",
	};

	if (netcp->rx_state < 0 || netcp->rx_state >= ARRAY_SIZE(state_str))
		return state_str[RX_STATE_INVALID];
	else
		return state_str[netcp->rx_state];
}

static inline void netcp_set_rx_state(struct netcp_priv *netcp,
				     enum netcp_rx_state state)
{
	netcp->rx_state = state;
	cpu_relax();
}

static inline bool netcp_is_alive(struct netcp_priv *netcp)
{
	return (netcp->rx_state == RX_STATE_POLL ||
		netcp->rx_state == RX_STATE_INTERRUPT);
}

static void netcp_dump_packet(struct netcp_packet *p_info, const char *cause)
{
	struct netcp_priv *netcp = p_info->netcp;
	struct sk_buff *skb = p_info->skb;
	unsigned char *head, *tail;

	head = skb->data;
	tail = skb_tail_pointer(skb) - 16;

	dev_dbg(netcp->dev, "packet %p %s, size %d (%d): "
		"%02x%02x%02x%02x%02x%02x%02x%02x"
		"%02x%02x%02x%02x%02x%02x%02x%02x"
		"%02x%02x%02x%02x%02x%02x%02x%02x"
		"%02x%02x%02x%02x%02x%02x%02x%02x\n",
		p_info, cause, skb->len, p_info->sg[2].length,
		head[0x00], head[0x01], head[0x02], head[0x03],
		head[0x04], head[0x05], head[0x06], head[0x07],
		head[0x08], head[0x09], head[0x0a], head[0x0b],
		head[0x0c], head[0x0d], head[0x0e], head[0x0f],
		tail[0x00], tail[0x01], tail[0x02], tail[0x03],
		tail[0x04], tail[0x05], tail[0x06], tail[0x07],
		tail[0x08], tail[0x09], tail[0x0a], tail[0x0b],
		tail[0x0c], tail[0x0d], tail[0x0e], tail[0x0f]);
}

static void netcp_rx_complete(void *data)
{
	struct netcp_packet *p_info = data;
	struct netcp_priv *netcp = p_info->netcp;
	struct sk_buff *skb = p_info->skb;
	struct scatterlist *sg;
	unsigned int frags;
	struct netcp_hook_list *rx_hook;

	p_info->status = dma_async_is_tx_complete(netcp->rx_channel,
						  p_info->cookie, NULL, NULL);
	WARN_ON(p_info->status != DMA_SUCCESS && p_info->status != DMA_ERROR);
	WARN_ON(netcp->rx_state != RX_STATE_INTERRUPT	&&
		netcp->rx_state != RX_STATE_POLL	&&
		netcp->rx_state != RX_STATE_TEARDOWN);

	/* sg[2] describes the buffer already attached to the sk_buff. */
	skb_put(skb, sg_dma_len(&p_info->sg[2]));

	/* Fill in the page fragment list from sg[3] and later */
	for (frags = 0, sg = sg_next(&p_info->sg[2]);
			frags < NETCP_SGLIST_SIZE-3 && sg;
			++frags, sg = sg_next(sg)) {
		skb_add_rx_frag(skb, frags, sg_page(sg), sg->offset,
				sg_dma_len(sg), sg_dma_len(sg));
	}

	dma_unmap_sg(netcp->dev, &p_info->sg[2], frags+1, DMA_FROM_DEVICE);

	if (unlikely(netcp->rx_state == RX_STATE_TEARDOWN)) {
		dev_dbg(netcp->dev,
			"receive: reclaimed packet %p, status %d, state %s\n",
			p_info, p_info->status, netcp_rx_state_str(netcp));
		dev_kfree_skb_any(skb);
		kfree(p_info);
		netcp->ndev->stats.rx_dropped++;
		return;
	}

	if (unlikely(p_info->status != DMA_SUCCESS)) {
		dev_warn(netcp->dev,
			 "receive: reclaimed packet %p, status %d, state %s\n",
			 p_info, p_info->status, netcp_rx_state_str(netcp));
		dev_kfree_skb_any(skb);
		kfree(p_info);
		netcp->ndev->stats.rx_errors++;
		return;
	}

	if (unlikely(!skb->len)) {
		dev_warn(netcp->dev, "receive: zero length packet\n");
		dev_kfree_skb_any(skb);
		kfree(p_info);
		netcp->ndev->stats.rx_errors++;
		return;
	}

	BUG_ON(netcp->rx_state != RX_STATE_POLL);


	netcp->ndev->last_rx = jiffies;

#ifdef DEBUG
	netcp_dump_packet(p_info, "rx");
#endif

	/* Call each of the RX hooks */
	list_for_each_entry(rx_hook, &netcp->rxhook_list_head, list) {
		int ret;
		ret = rx_hook->hook_rtn(rx_hook->order, rx_hook->hook_data, p_info);
		if (ret) {
			dev_err(netcp->dev, "RX hook %d failed: %d\n", rx_hook->order, ret);
			dev_kfree_skb_any(skb);
			kfree(p_info);
			return;
		}
	}

	netcp->ndev->stats.rx_packets++;
	netcp->ndev->stats.rx_bytes += skb->len;

	p_info->skb = NULL;
	kfree(p_info);

	/* push skb up the stack */
	skb->protocol = eth_type_trans(skb, netcp->ndev);
	netif_receive_skb(skb);
}

static void netcp_tx_complete(void *data)
{
	struct netcp_packet *p_info = data;
	struct netcp_priv *netcp = p_info->netcp;
	struct sk_buff *skb = p_info->skb;
	unsigned int sg_ents;

	p_info->status = dma_async_is_tx_complete(p_info->tx_pipe->dma_channel,
						  p_info->cookie, NULL, NULL);
	WARN_ON(p_info->status != DMA_SUCCESS && p_info->status != DMA_ERROR);

	sg_ents = sg_count(&p_info->sg[2], p_info->sg_ents);
	dma_unmap_sg(netcp->dev, &p_info->sg[2], sg_ents, DMA_TO_DEVICE);

	netcp_dump_packet(p_info, "txc");

	if (p_info->status != DMA_SUCCESS)
		netcp->ndev->stats.tx_errors++;

	dev_kfree_skb_any(skb);
	kfree(p_info);

	if (netif_queue_stopped(netcp->ndev) && netcp_is_alive(netcp))
		netif_wake_queue(netcp->ndev);
}

/* Release a free receive buffer */
static void netcp_rxpool_free(void *arg, unsigned q_num, unsigned bufsize,
		struct dma_async_tx_descriptor *desc)
{
	struct netcp_priv *netcp = arg;

	if (q_num == 0) {
		struct netcp_packet *p_info = desc->callback_param;
		struct sk_buff *skb = p_info->skb;

		dma_unmap_sg(netcp->dev, &p_info->sg[2], 1, DMA_FROM_DEVICE);
		dev_kfree_skb_any(skb);
		kfree(p_info);
	} else {
		void *bufptr = desc->callback_param;
		struct scatterlist sg[1];

		sg_init_table(sg, 1);
		sg_set_buf(&sg[0], bufptr, PAGE_SIZE);
		sg_dma_address(&sg[0]) = virt_to_phys(bufptr);
		dma_unmap_sg(netcp->dev, sg, 1, DMA_FROM_DEVICE);
		free_page((unsigned long)bufptr);
	}
}

static void netcp_rx_complete2nd(void *data)
{
	WARN(1, "Attempt to complete secondary receive buffer!\n");
}

/* Allocate a free receive buffer */
static struct dma_async_tx_descriptor *netcp_rxpool_alloc(void *arg,
		unsigned q_num, unsigned bufsize)
{
	struct netcp_priv *netcp = arg;
	struct dma_async_tx_descriptor *desc;
	struct dma_device *device;
	u32 err = 0;

	device = netcp->rx_channel->device;

	if (q_num == 0) {
		struct netcp_packet *p_info;
		struct sk_buff *skb;

		/* Allocate a primary receive queue entry */
		p_info = kzalloc(sizeof(*p_info), GFP_ATOMIC);
		if (!p_info) {
			dev_err(netcp->dev, "packet alloc failed\n");
			return NULL;
		}
		p_info->netcp = netcp;

		skb = netdev_alloc_skb(netcp->ndev, bufsize);
		if (!skb) {
			dev_err(netcp->dev, "skb alloc failed\n");
			kfree(p_info);
			return NULL;
		}
		skb->dev = netcp->ndev;
		p_info->skb = skb;

		sg_init_table(p_info->sg, NETCP_SGLIST_SIZE);
		sg_set_buf(&p_info->sg[0], p_info->epib, sizeof(p_info->epib));
		sg_set_buf(&p_info->sg[1], p_info->psdata, sizeof(p_info->psdata));
		sg_set_buf(&p_info->sg[2], skb_tail_pointer(skb), skb_tailroom(skb));

		p_info->sg_ents = 2 + dma_map_sg(netcp->dev, &p_info->sg[2],
						 1, DMA_FROM_DEVICE);
		if (p_info->sg_ents != 3) {
			dev_err(netcp->dev, "dma map failed\n");
			dev_kfree_skb_any(skb);
			kfree(p_info);
			return NULL;
		}

		desc = dmaengine_prep_slave_sg(netcp->rx_channel, p_info->sg,
					       3, DMA_DEV_TO_MEM,
					       DMA_HAS_EPIB | DMA_HAS_PSINFO);
		if (IS_ERR_OR_NULL(desc)) {
			dma_unmap_sg(netcp->dev, &p_info->sg[2], 1, DMA_FROM_DEVICE);
			dev_kfree_skb_any(skb);
			kfree(p_info);
			err = PTR_ERR(desc);
			if (err != -ENOMEM) {
				dev_err(netcp->dev,
					"dma prep failed, error %d\n", err);
			}
			return NULL;
		}

		desc->callback_param = p_info;
		desc->callback = netcp_rx_complete;
		p_info->cookie = desc->cookie;

	} else {

		/* Allocate a secondary receive queue entry */
		struct scatterlist sg[1];
		void *bufptr;
		
		bufptr = (void *)__get_free_page(GFP_ATOMIC);
		if (!bufptr) {
			dev_warn(netcp->dev, "page alloc failed for pool %d\n", q_num);
			return NULL;
		}
		
		sg_init_table(sg, 1);
		sg_set_buf(&sg[0], bufptr, PAGE_SIZE);

		err = dma_map_sg(netcp->dev, sg, 1, DMA_FROM_DEVICE);
		if (err != 1) {
			dev_warn(netcp->dev, "map error for pool %d\n", q_num);
			free_page((unsigned long)bufptr);
			return NULL;
		}

		desc = dmaengine_prep_slave_sg(netcp->rx_channel, sg, 1,
					       DMA_DEV_TO_MEM,
					       q_num << DMA_QNUM_SHIFT);
		if (IS_ERR_OR_NULL(desc)) {
			dma_unmap_sg(netcp->dev, sg, 1, DMA_FROM_DEVICE);
			free_page((unsigned long)bufptr);

			err = PTR_ERR(desc);
			if (err != -ENOMEM) {
				dev_err(netcp->dev,
					"dma prep failed, error %d\n", err);
			}
			return NULL;
		}

		desc->callback_param = bufptr;
		desc->callback = netcp_rx_complete2nd;
	}
	
	return desc;
}

/* NAPI poll */
static int netcp_poll(struct napi_struct *napi, int budget)
{
	struct netcp_priv *netcp = container_of(napi, struct netcp_priv, napi);
	unsigned long flags;
	unsigned packets;

	spin_lock_irqsave(&netcp->lock, flags);

	BUG_ON(netcp->rx_state != RX_STATE_SCHEDULED);
	netcp_set_rx_state(netcp, RX_STATE_POLL);

	spin_unlock_irqrestore(&netcp->lock, flags);

	packets = dma_poll(netcp->rx_channel, budget);

	if (packets < budget) {
		netcp_set_rx_state(netcp, RX_STATE_INTERRUPT);
		napi_complete(&netcp->napi);
		dmaengine_resume(netcp->rx_channel);
	} else {
		netcp_set_rx_state(netcp, RX_STATE_SCHEDULED);
	}

	dma_rxfree_refill(netcp->rx_channel);

	return packets;
}

/* Push an outcoming packet */
static int netcp_ndo_start_xmit(struct sk_buff *skb, struct net_device *ndev)
{
	struct netcp_priv *netcp = netdev_priv(ndev);
	struct dma_async_tx_descriptor *desc;
	struct netcp_tx_pipe *tx_pipe = NULL;
	struct netcp_hook_list *tx_hook;
	struct netcp_packet *p_info;
	bool need_poll = 0;
	int real_sg_ents;
	int ret = 0;

		ndev->stats.tx_packets++;
	ndev->stats.tx_bytes += skb->len;

	p_info = kzalloc(sizeof(*p_info), GFP_ATOMIC);
	if (!p_info) {
		ndev->stats.tx_dropped++;
		dev_kfree_skb_any(skb);
		dev_warn(netcp->dev, "failed to alloc packet info\n");
		return -ENOMEM;
	}

	p_info->netcp = netcp;
	p_info->skb = skb;
	p_info->tx_pipe = NULL;
	p_info->psdata_len = 0;

	if (unlikely(skb->len < NETCP_MIN_PACKET_SIZE)) {
		ret = skb_padto(skb, NETCP_MIN_PACKET_SIZE);
		if (ret < 0) {
			dev_warn(netcp->dev, "padding failed, ignoring\n");
			return ret;
		}
		skb->len = NETCP_MIN_PACKET_SIZE;
	}

	netcp_dump_packet(p_info, "txs");

	/* Find out where to inject the packet for transmission */
	list_for_each_entry(tx_hook, &netcp->txhook_list_head, list) {
		ret = tx_hook->hook_rtn(tx_hook->order, tx_hook->hook_data,
					p_info);
		if (ret) {
			dev_err(netcp->dev, "TX hook %d rejected"
				" the packet: %d\n", tx_hook->order, ret);
			dev_kfree_skb_any(skb);
			kfree(p_info);
			return ret;
		}
	}

	/* Make sure some TX hook claimed the packet */
	tx_pipe = p_info->tx_pipe;
	if (tx_pipe == NULL) {
		dev_err(netcp->dev, "No TX hook claimed the packet!\n");
		dev_kfree_skb_any(skb);
		kfree(p_info);
		return -ENXIO;
	}

	sg_init_table(p_info->sg, NETCP_SGLIST_SIZE);
	sg_set_buf(&p_info->sg[0], p_info->epib, sizeof(p_info->epib));
#if 1
	sg_set_buf(&p_info->sg[1], &p_info->psdata[NETCP_PSDATA_LEN - p_info->psdata_len],
			p_info->psdata_len * sizeof(u32));
#else
	sg_set_buf(&p_info->sg[1], &p_info->psdata[0],
			p_info->psdata_len * sizeof(u32));
#endif

	/* Map all the packet fragments	into the scatterlist */
	real_sg_ents = skb_to_sgvec(skb, &p_info->sg[2], 0, skb->len);
	p_info->sg_ents = 2 + dma_map_sg(netcp->dev, &p_info->sg[2],
					 real_sg_ents, DMA_TO_DEVICE);
	if (p_info->sg_ents != (real_sg_ents + 2)) {
		ndev->stats.tx_dropped++;
		dev_kfree_skb_any(skb);
		kfree(p_info);
		dev_warn(netcp->dev, "failed to map transmit packet\n");
		ret = -ENXIO;
		goto out;
	}

	desc = dmaengine_prep_slave_sg(tx_pipe->dma_channel, p_info->sg,
				       p_info->sg_ents, DMA_MEM_TO_DEV,
				       DMA_HAS_EPIB | DMA_HAS_PSINFO);

	if (IS_ERR_OR_NULL(desc)) {
		ndev->stats.tx_dropped++;
		dma_unmap_sg(netcp->dev, &p_info->sg[2], real_sg_ents,
			     DMA_TO_DEVICE);
		dev_kfree_skb_any(skb);
		kfree(p_info);
		dev_warn(netcp->dev, "failed to prep slave dma\n");
		netif_stop_queue(ndev);
		ret = -ENOBUFS;
		goto out;
	}

	desc->callback_param = p_info;
	desc->callback = netcp_tx_complete;
	p_info->cookie = dmaengine_submit(desc);

	ndev->trans_start = jiffies;

	ret = NETDEV_TX_OK;

out:
	if (atomic_dec_and_test(&tx_pipe->dma_poll_count)) {
		dev_dbg(netcp->dev, "transmit poll threshold reached\n");
		need_poll = true;
		atomic_add(tx_pipe->dma_poll_threshold,
			   &tx_pipe->dma_poll_count);
	}

	if (need_poll || ret < 0) {
		dev_dbg(netcp->dev, "polling transmit channel %d\n",
			tx_pipe->dma_queue);
		dma_poll(tx_pipe->dma_channel, -1);
	}

	return ret;
}

static void netcp_uc_list_set(struct net_device *ndev, int vid)
{
	struct netcp_priv *netcp = netdev_priv(ndev);
	struct netcp_module_data *module;
	struct netdev_hw_addr *uc_addr;
	int i = 0, err;
	int uc_count;
	u8 *uc_list;

	uc_count = netdev_uc_count(ndev);
	dev_dbg(netcp->dev, "uc count is = %d\n", uc_count);

	uc_list = kzalloc(uc_count * ETH_ALEN, GFP_ATOMIC);
	if (!uc_list)
		goto out;

	netdev_for_each_uc_addr(uc_addr, ndev) {
		memcpy(&uc_list[i * ETH_ALEN], &uc_addr->addr[0], ETH_ALEN);
		i++;

		dev_dbg(netcp->dev, "uc address %x:%x:%x:%x:%x:%x\n",
			uc_addr->addr[0], uc_addr->addr[1], uc_addr->addr[2],
			uc_addr->addr[3], uc_addr->addr[4], uc_addr->addr[5]);

	}

	for_each_module(netcp, module) {
		if (module->add_ucast != NULL) {
			err = module->add_ucast(module, (u8 *)uc_list,
						uc_count, vid);
			if (err != 0) {
				dev_err(netcp->dev, "Could not add "
					"unicast entries\n");
				return;
			}
		}
	}

	kfree(uc_list);
out:
	return;
}

static void netcp_mc_list_set(struct net_device *ndev, int vid)
{
	struct netcp_priv *netcp = netdev_priv(ndev);
	struct netcp_module_data *module;
	struct netdev_hw_addr *mc_addr;
	int i = 0, err;
	int mc_count;
	u8 *mc_list;

	mc_count = netdev_mc_count(ndev);
	dev_dbg(netcp->dev, "mc count is = %d\n", mc_count);

	mc_list = kzalloc(mc_count * ETH_ALEN, GFP_ATOMIC);
	if (!mc_list)
		goto out;

	netdev_for_each_mc_addr(mc_addr, ndev) {
		memcpy(&mc_list[i * ETH_ALEN], &mc_addr->addr[0], ETH_ALEN);
		i++;

		dev_dbg(netcp->dev, "mc address %x:%x:%x:%x:%x:%x\n",
			mc_addr->addr[0], mc_addr->addr[1], mc_addr->addr[2],
			mc_addr->addr[3], mc_addr->addr[4], mc_addr->addr[5]);

	}

	for_each_module(netcp, module) {
		if (module->add_mcast != NULL) {
			err = module->add_mcast(module, (u8 *)mc_list,
						mc_count, vid);
			if (err != 0) {
				dev_err(netcp->dev, "Could not add "
					"multicast entries\n");
				return;
			}
		}
	}

	kfree(mc_list);
out:
	return;
}

static void __netcp_set_rx_mode(struct net_device *ndev, int vid)
{
	if (!netdev_mc_empty(ndev))
		netcp_mc_list_set(ndev, vid);

	if (!netdev_uc_empty(ndev))
		netcp_uc_list_set(ndev, vid);
}

static void netcp_set_rx_mode(struct net_device *ndev)
{
	struct netcp_priv *netcp = netdev_priv(ndev);
	unsigned long flags;
	u16 vid;

	spin_lock_irqsave(&netcp->lock, flags);

	__netcp_set_rx_mode(ndev, -1);

	for_each_set_bit(vid, netcp->active_vlans, VLAN_N_VID)
		__netcp_set_rx_mode(ndev, vid);

	spin_unlock_irqrestore(&netcp->lock, flags);

	return;
}
struct dma_chan *netcp_get_rx_chan(struct netcp_priv *netcp)
{
	return netcp->rx_channel;
}

static void netcp_rx_notify(struct dma_chan *chan, void *arg)
{
	struct netcp_priv *netcp = arg;

	BUG_ON(netcp->rx_state != RX_STATE_INTERRUPT);
	dmaengine_pause(netcp->rx_channel);
	netcp_set_rx_state(netcp, RX_STATE_SCHEDULED);
	napi_schedule(&netcp->napi);
}

/* Open the device */
static int netcp_ndo_open(struct net_device *ndev)
{
	struct netcp_priv *netcp = netdev_priv(ndev);
	struct netcp_module_data *module;
	struct dma_keystone_info config;
	dma_cap_mask_t mask;
	int err = -ENODEV;
	const char *name;
	int i;

	netif_carrier_off(ndev);

	BUG_ON(netcp->rx_state != RX_STATE_INVALID);

	dma_cap_zero(mask);
	dma_cap_set(DMA_SLAVE, mask);

	name = netcp->rx_chan_name;
	netcp->rx_channel = dma_request_channel_by_name(mask, name);
	if (IS_ERR_OR_NULL(netcp->rx_channel))
		goto fail;

	memset(&config, 0, sizeof(config));
	config.direction		= DMA_DEV_TO_MEM;
	config.scatterlist_size		= NETCP_SGLIST_SIZE;
	config.rxpool_allocator		= netcp_rxpool_alloc;
	config.rxpool_destructor	= netcp_rxpool_free;
	config.rxpool_param		= netcp;
	config.rxpool_thresh_enable	= DMA_THRESH_NONE;

	for (i = 0; i < KEYSTONE_QUEUES_PER_CHAN &&
		    netcp->rx_queue_depths[i] &&
		    netcp->rx_buffer_sizes[i]; ++i) {
		config.rxpools[i].pool_depth  = netcp->rx_queue_depths[i];
		config.rxpools[i].buffer_size = netcp->rx_buffer_sizes[i];
		dev_dbg(netcp->dev, "rx_pool[%d] depth %d, size %d\n", i,
				config.rxpools[i].pool_depth,
				config.rxpools[i].buffer_size);
	}
	config.rxpool_count = i;

	err = dma_keystone_config(netcp->rx_channel, &config);
	if (err) {
		dev_err(netcp->dev, "%d error configuring RX channel\n",
				err);
		goto fail;
	}

	dma_set_notify(netcp->rx_channel, netcp_rx_notify, netcp);

	dev_dbg(netcp->dev, "opened RX channel: %p\n", netcp->rx_channel);

	netcp_set_rx_state(netcp, RX_STATE_INTERRUPT);

	for_each_module(netcp, module) {
		if (module->open != NULL) {
			err = module->open(module, ndev);
			if (err != 0) {
				dev_err(netcp->dev, "Open failed\n");
				goto fail;
			}
		}
	}

	napi_enable(&netcp->napi);
	dma_rxfree_refill(netcp->rx_channel);

	dev_info(netcp->dev, "netcp device %s opened\n", ndev->name);

	return 0;
fail:
	if (netcp->rx_channel) {
		dma_release_channel(netcp->rx_channel);
		netcp->rx_channel = NULL;
	}
	return err;
}

/* Close the device */
static int netcp_ndo_stop(struct net_device *ndev)
{
	struct netcp_priv *netcp = netdev_priv(ndev);
	struct netcp_module_data *module;
	unsigned long flags;
	int err = 0;

	spin_lock_irqsave(&netcp->lock, flags);

	netif_stop_queue(ndev);
	netif_carrier_off(ndev);

	BUG_ON(!netcp_is_alive(netcp));

	netcp_set_rx_state(netcp, RX_STATE_TEARDOWN);

	dmaengine_pause(netcp->rx_channel);

	spin_unlock_irqrestore(&netcp->lock, flags);

	napi_disable(&netcp->napi);

	if (netcp->rx_channel) {
		dma_release_channel(netcp->rx_channel);
		netcp->rx_channel = NULL;
	}

	netcp_set_rx_state(netcp, RX_STATE_INVALID);

	for_each_module(netcp, module) {
		if (module->close != NULL) {
			err = module->close(module);
			if (err != 0) {
				dev_err(netcp->dev, "Close failed\n");
				goto out;
			}
		}
	}
out:
	dev_dbg(netcp->dev, "netcp device %s stopped\n", ndev->name);

	return 0;
}

static int netcp_hwtstamp_ioctl(struct net_device *ndev,
				struct ifreq *ifr, int cmd)
{
	struct netcp_priv *netcp = netdev_priv(ndev);
	struct hwtstamp_config cfg;

	if (copy_from_user(&cfg, ifr->ifr_data, sizeof(cfg)))
		return -EFAULT;

	if (cfg.flags)
		return -EINVAL;

	switch (cfg.tx_type) {
	case HWTSTAMP_TX_OFF:
		netcp->hwts_tx_en = 0;
		break;
	case HWTSTAMP_TX_ON:
		netcp->hwts_tx_en = 1;
		break;
	default:
		return -ERANGE;
	}

	switch (cfg.rx_filter) {
	case HWTSTAMP_FILTER_NONE:
		netcp->hwts_rx_en = 0;
		break;
	default:
		netcp->hwts_rx_en = 1;
		cfg.rx_filter = HWTSTAMP_FILTER_ALL;
		break;
	}

	return copy_to_user(ifr->ifr_data, &cfg, sizeof(cfg)) ? -EFAULT : 0;
}

static int netcp_ndo_ioctl(struct net_device *ndev,
			   struct ifreq *req, int cmd)
{
	if (!netif_running(ndev))
		return -EINVAL;

	if (cmd == SIOCSHWTSTAMP)
		return netcp_hwtstamp_ioctl(ndev, req, cmd);

	return 0;
}

static int netcp_ndo_change_mtu(struct net_device *ndev, int new_mtu)
{
	struct netcp_priv *netcp = netdev_priv(ndev);
	int old_max_frame = ndev->mtu + ETH_HLEN + ETH_FCS_LEN;
	int max_frame = new_mtu + ETH_HLEN + ETH_FCS_LEN;
	unsigned long flags;
	int ret = 0;

	spin_lock_irqsave(&netcp->lock, flags);

	netif_stop_queue(ndev);
	netif_carrier_off(ndev);

	BUG_ON(!netcp_is_alive(netcp));

	netcp_set_rx_state(netcp, RX_STATE_TEARDOWN);

	dmaengine_pause(netcp->rx_channel);

	spin_unlock_irqrestore(&netcp->lock, flags);

	napi_disable(&netcp->napi);

	netcp_set_rx_state(netcp, RX_STATE_INVALID);

	/* MTU < 68 is an error for IPv4 traffic, just don't allow it */
	if ((new_mtu < 68) ||
	    (max_frame > NETCP_MAX_FRAME_SIZE)) {
		dev_err(netcp->dev, "Invalid mtu size = %d\n", new_mtu);
		ret = -EINVAL;
		goto out_change_mtu;
	}

	if (old_max_frame == max_frame) {
		ret = 0;
		goto out_change_mtu;
	}

	netcp->rx_packet_max = max_frame;

	ndev->mtu = new_mtu;

out_change_mtu:
	netcp_set_rx_state(netcp, RX_STATE_INTERRUPT);

	dmaengine_resume(netcp->rx_channel);

	napi_enable(&netcp->napi);

	netif_start_queue(ndev);
	netif_carrier_on(ndev);

	return ret;
}

static void netcp_ndo_tx_timeout(struct net_device *ndev)
{
	struct netcp_priv *netcp = netdev_priv(ndev);

	dev_err(netcp->dev, "transmit timed out\n");
	if (netif_queue_stopped(ndev))
		netif_wake_queue(ndev);
}

static int netcp_rx_add_vid(struct net_device *ndev, u16 vid)
{
	struct netcp_priv *netcp = netdev_priv(ndev);
	struct netcp_module_data *module;
	unsigned long flags;
	int err = 0;

	dev_info(netcp->dev, "adding rx vlan id: %d\n", vid);

	spin_lock_irqsave(&netcp->lock, flags);

	set_bit(vid, netcp->active_vlans);

	for_each_module(netcp, module) {
		if ((module->add_vid != NULL) && (vid != 0)) {
			err = module->add_vid(module, vid);
			if (err != 0) {
				dev_err(netcp->dev, "Could not add "
					"vlan id = %d\n", vid);
				return -ENODEV;
			}
		}
	}

	spin_unlock_irqrestore(&netcp->lock, flags);

	return 0;
}

static int netcp_rx_kill_vid(struct net_device *ndev, u16 vid)
{
	struct netcp_priv *netcp = netdev_priv(ndev);
	struct netcp_module_data *module;
	int err = 0;

	dev_info(netcp->dev, "removing rx vlan id: %d\n", vid);

	for_each_module(netcp, module) {
		if (module->del_vid != NULL) {
			err = module->del_vid(module, vid);
			if (err != 0) {
				dev_err(netcp->dev, "Could not delete "
					"vlan id = %d\n", vid);
				return -ENODEV;
			}
		}
	}

	clear_bit(vid, netcp->active_vlans);

	return 0;
}

static const struct net_device_ops netcp_netdev_ops = {
	.ndo_open		= netcp_ndo_open,
	.ndo_stop		= netcp_ndo_stop,
	.ndo_start_xmit		= netcp_ndo_start_xmit,
	.ndo_set_rx_mode	= netcp_set_rx_mode,
	.ndo_do_ioctl           = netcp_ndo_ioctl,
	.ndo_change_mtu		= netcp_ndo_change_mtu,
	.ndo_set_mac_address	= eth_mac_addr,
	.ndo_validate_addr	= eth_validate_addr,
	.ndo_vlan_rx_add_vid	= netcp_rx_add_vid,
	.ndo_vlan_rx_kill_vid	= netcp_rx_kill_vid,
	.ndo_tx_timeout		= netcp_ndo_tx_timeout,
};

static const char *netcp_node_name(struct device_node *node)
{
	const char *name;

	if (of_property_read_string(node, "label", &name) < 0)
		name = node->name;
	if (!name)
		name = "unknown";
	return name;
}

static int __devinit netcp_probe(struct platform_device *pdev)
{
	struct device_node *node = pdev->dev.of_node;
	struct device_node *child;
	struct netcp_module *module;
	struct netcp_module_data *module_data;
	struct netcp_priv *netcp;
	struct net_device *ndev;
	resource_size_t size;
	struct resource res;
	void __iomem *efuse = NULL;
	u32 efuse_mac = 0;
	const char *name;
	const void *mac_addr;
	u8 efuse_mac_addr[6];
	int ret = 0;

	if (!node) {
		dev_err(&pdev->dev, "could not find device info\n");
		return -EINVAL;
	}

	ndev = alloc_etherdev(sizeof(struct netcp_priv));
	if (!ndev) {
		dev_err(&pdev->dev, "Error allocating net_device\n");
		ret = -ENOMEM;
		goto probe_quit;
	}
	ndev->features |= NETIF_F_SG;
	ndev->features |= NETIF_F_FRAGLIST;

	ndev->features |= NETIF_F_HW_VLAN_FILTER;

	ndev->vlan_features |= NETIF_F_TSO |
				NETIF_F_TSO6 |
				NETIF_F_IP_CSUM |
				NETIF_F_IPV6_CSUM |
				NETIF_F_SG|
				NETIF_F_FRAGLIST;

	platform_set_drvdata(pdev, ndev);
	netcp = netdev_priv(ndev);
	spin_lock_init(&netcp->lock);
	INIT_LIST_HEAD(&netcp->modules);
	INIT_LIST_HEAD(&netcp->txhook_list_head);
	INIT_LIST_HEAD(&netcp->rxhook_list_head);
	netcp->pdev = pdev;
	netcp->ndev = ndev;
	netcp->dev  = &ndev->dev;
	netcp->msg_enable = netif_msg_init(netcp_debug_level, NETCP_DEBUG);
	netcp->rx_packet_max = netcp_rx_packet_max;
	netcp_set_rx_state(netcp, RX_STATE_INVALID);

	ret = of_property_read_u32(node, "efuse-mac", &efuse_mac);

	if (efuse_mac) {
		if (of_address_to_resource(node, 1, &res)) {
			dev_err(&pdev->dev, "could not find resource\n");
			ret = -ENODEV;
			goto probe_quit;
		}
		size = resource_size(&res);

		if (!devm_request_mem_region(&pdev->dev, res.start, size,
					     dev_name(netcp->dev))) {
			dev_err(&pdev->dev, "could not reserve resource\n");
			ret = -ENOMEM;
			goto probe_quit;
		}

		efuse = devm_ioremap_nocache(&pdev->dev, res.start, size);
		if (!efuse) {
			dev_err(&pdev->dev, "could not map resource\n");
			ret = -ENOMEM;
			goto probe_quit;
		}
	}

	ret = of_property_read_string(node, "rx-channel", &netcp->rx_chan_name);
	if (ret < 0)
		netcp->rx_chan_name = "netrx";

	ret = of_property_read_u32_array(node, "rx-queue-depth",
			netcp->rx_queue_depths, KEYSTONE_QUEUES_PER_CHAN);
	if (ret < 0) {
		dev_err(&pdev->dev, "missing \"rx-queue-depth\" parameter\n");
		netcp->rx_queue_depths[0] = 128;
	}

	ret = of_property_read_u32_array(node, "rx-buffer-size",
			netcp->rx_buffer_sizes, KEYSTONE_QUEUES_PER_CHAN);
	if (ret) {
		dev_err(&pdev->dev, "missing \"rx-buffer-size\" parameter\n");
		netcp->rx_buffer_sizes[0] = 1500;
	}


	for_each_child_of_node(node, child) {
		name = netcp_node_name(child);
		module = netcp_find_module(name);
		if (!module) {
			dev_err(&pdev->dev, "Could not find module %s\n", name);
			goto clean_ndev_ret;
		}
		module_data = module->probe(&pdev->dev, child);
		if (IS_ERR_OR_NULL(module_data)) {
			dev_err(&pdev->dev, "Probe of module %s failed\n", name);
			goto clean_ndev_ret;
		}
		list_add_tail(&module_data->list, &netcp->modules);
		module_data->priv = netcp;
	}

	if (efuse_mac == 1) {
		emac_arch_get_mac_addr(efuse_mac_addr, efuse);
		if (is_valid_ether_addr(efuse_mac_addr))
			memcpy(ndev->dev_addr, efuse_mac_addr, ETH_ALEN);
	} else {
		mac_addr = of_get_mac_address(node);
		if (mac_addr)
			memcpy(ndev->dev_addr, mac_addr, ETH_ALEN);
		else
			random_ether_addr(ndev->dev_addr);
	}

	ether_setup(ndev);

	/* NAPI register */
	netif_napi_add(ndev, &netcp->napi, netcp_poll, NETCP_NAPI_WEIGHT);

	/* Register the network device */
	ndev->dev_id		= 0;
	ndev->watchdog_timeo	= NETCP_TX_TIMEOUT;
	ndev->netdev_ops	= &netcp_netdev_ops;

	SET_NETDEV_DEV(ndev, &pdev->dev);
	keystone_set_ethtool_ops(ndev);

	ret = register_netdev(ndev);
	if (ret) {
		dev_err(netcp->dev, "Error registering net device\n");
		ret = -ENODEV;
		goto clean_ndev_ret;
	}

	return 0;

clean_ndev_ret:
	free_netdev(ndev);
probe_quit:
	return ret;
}

static int __devexit netcp_remove(struct platform_device *pdev)
{
	struct net_device *ndev = platform_get_drvdata(pdev);
	struct netcp_priv *netcp = netdev_priv(ndev);
	struct netcp_module_data *module, *tmp;

	for_each_module_safe(netcp, module, tmp)
		module->remove(module);
	free_netdev(ndev);
	platform_set_drvdata(pdev, NULL);

	return 0;
}

static struct of_device_id __devinitdata of_match[] = {
	{ .compatible = "ti,keystone-netcp", },
	{},
};

MODULE_DEVICE_TABLE(of, keystone_hwqueue_of_match);

static struct platform_driver netcp_driver = {
	.driver = {
		.name		= "keystone-netcp",
		.owner		= THIS_MODULE,
		.of_match_table	= of_match,
	},
	.probe = netcp_probe,
	.remove = __devexit_p(netcp_remove),
};

static int __init netcp_init(void)
{
	return platform_driver_register(&netcp_driver);
}
module_init(netcp_init);

static void __exit netcp_exit(void)
{
	platform_driver_unregister(&netcp_driver);
}
module_exit(netcp_exit);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("TI Keystone Ethernet driver");
