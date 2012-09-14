/*
 * Copyright (C) 2012 Texas Instruments Incorporated
 * Authors: Sandeep Nair <sandeep_n@ti.com>
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
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/types.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_address.h>
#include <linux/interrupt.h>
#include <linux/byteorder/generic.h>
#include <linux/platform_device.h>
#include <linux/keystone-dma.h>
#include <linux/errqueue.h>

#include "keystone_net.h"
#include "keystone_pasahost.h"

struct sa_device {
	struct device			*dev;
	struct netcp_module_data	 module;
	struct netcp_tx_pipe		 tx_pipe;
	u32				 tx_queue_depth;
};

#define sa_from_module(data)	container_of(data, struct sa_device, module)
#define sa_to_module(sa)	(&(sa)->module)

struct ipsecmgr_mod_sa_swinfo {
	u32 word0;
	u32 word1;
};

#define	SA_TXHOOK_ORDER	30

static int sa_tx_hook(int order, void *data, struct netcp_packet *p_info)
{
	u16 offset, len, ihl;
	u32 *psdata;
	const struct iphdr *iph;
	struct netcp_module_data *module = data;
	struct sa_device *sa_dev = sa_from_module(module);
	struct ipsecmgr_mod_sa_swinfo *swinfo =
			(struct ipsecmgr_mod_sa_swinfo *)p_info->skb->sp;

	if (!swinfo)
		return 0;

	psdata = netcp_push_psdata(p_info, (2 * sizeof(u32)));
	if (!psdata)
		return -ENOMEM;

	iph = ip_hdr(p_info->skb);
	ihl = iph->ihl * 4;
	offset = ((ulong)skb_network_header(p_info->skb) -
				(ulong)p_info->skb->data) + ihl;
	len = ntohs(iph->tot_len) - ihl;

	psdata[0] = PASAHO_SINFO_FORMAT_CMD(offset, len);
	psdata[1] = 0;
	p_info->epib[1] = swinfo->word0;
	p_info->epib[2] = swinfo->word1;

	p_info->tx_pipe = &sa_dev->tx_pipe;
	kfree(swinfo);
	p_info->skb->sp = NULL;
	return 0;
}

static int sa_close(struct netcp_module_data *data)
{
	struct sa_device *sa_dev = sa_from_module(data);

	netcp_unregister_txhook(data->priv, SA_TXHOOK_ORDER, sa_tx_hook, data);

	if (sa_dev->tx_pipe.dma_channel) {
		dmaengine_pause(sa_dev->tx_pipe.dma_channel);
		dma_release_channel(sa_dev->tx_pipe.dma_channel);
		sa_dev->tx_pipe.dma_channel = NULL;
	}
	return 0;
}

static int sa_open(struct netcp_module_data *data, struct net_device *ndev)
{
	struct sa_device *sa_dev = sa_from_module(data);
	struct dma_keystone_info config;
	dma_cap_mask_t mask;
	int ret, err;

	dma_cap_zero(mask);
	dma_cap_set(DMA_SLAVE, mask);

	/* Open the SA IPSec data transmit channel */
	sa_dev->tx_pipe.dma_chan_name = "satx";
	sa_dev->tx_pipe.dma_channel = dma_request_channel_by_name(mask,
					sa_dev->tx_pipe.dma_chan_name);
	if (IS_ERR_OR_NULL(sa_dev->tx_pipe.dma_channel)) {
		dev_err(sa_dev->dev, "Could not get SA TX IPSec data channel\n");
		sa_dev->tx_pipe.dma_channel = NULL;
		ret = -ENODEV;
		goto fail;
	}

	memset(&config, 0, sizeof(config));
	config.direction = DMA_MEM_TO_DEV;
	config.tx_queue_depth = sa_dev->tx_queue_depth;

	err = dma_keystone_config(sa_dev->tx_pipe.dma_channel, &config);
	if (err) {
		ret = -ENODEV;
		goto fail;
	}

	sa_dev->tx_pipe.dma_queue = dma_get_tx_queue(sa_dev->tx_pipe.dma_channel);
	sa_dev->tx_pipe.dma_poll_threshold = config.tx_queue_depth / 2;
	atomic_set(&sa_dev->tx_pipe.dma_poll_count,
			sa_dev->tx_pipe.dma_poll_threshold);

	netcp_register_txhook(data->priv, SA_TXHOOK_ORDER, sa_tx_hook, data);
	return 0;

fail:
	sa_close(data);
	return ret;
}

static int sa_remove(struct netcp_module_data *data)
{
	struct sa_device *sa_dev = sa_from_module(data);
	kfree(sa_dev);
	return 0;
}

static struct netcp_module_data *sa_probe(struct device *dev,
					  struct device_node *node)
{
	struct sa_device *sa_dev;
	int ret = 0;

	sa_dev = devm_kzalloc(dev, sizeof(struct sa_device), GFP_KERNEL);
	if (!sa_dev) {
		dev_err(dev, "memory allocation failed\n");
		ret = -ENOMEM;
		goto exit;
	}

	if (!node) {
		dev_err(dev, "device tree info unavailable\n");
		ret = -ENODEV;
		goto exit;
	}

	ret = of_property_read_u32(node, "tx_queue_depth",
				   &sa_dev->tx_queue_depth);
	if (ret < 0) {
		dev_err(dev, "missing tx_queue_depth parameter, err %d\n", ret);
		sa_dev->tx_queue_depth = 32;
	}
	dev_dbg(dev, "tx_queue_depth %u\n", sa_dev->tx_queue_depth);

	sa_dev->dev = dev;
	sa_dev->module.open		= sa_open;
	sa_dev->module.close		= sa_close;
	sa_dev->module.remove		= sa_remove;

	return sa_to_module(sa_dev);
exit:
	return NULL;
}

static struct netcp_module sa_module = {
	.name	= "keystone-sa",
	.owner	= THIS_MODULE,
	.probe	= sa_probe,
};

static int __init keysone_sa_init(void)
{
	return netcp_register_module(&sa_module);
}
subsys_initcall(keysone_sa_init);

static void __exit keysone_sa_exit(void)
{
	netcp_unregister_module(&sa_module);
}
module_exit(keysone_sa_exit);

MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Sandeep Nair <sandeep_n@ti.com>");
MODULE_DESCRIPTION("IPSec driver for Keystone devices");

