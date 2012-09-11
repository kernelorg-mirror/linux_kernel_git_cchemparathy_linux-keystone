/*
 * Keystone crypto accelerator driver
 *
 * Copyright (C) 2011 Texas Instruments Incorporated - http://www.ti.com
 * Contact: Sandeep Nair <sandeep_n@ti.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 */

#include <linux/clk.h>
#include <linux/err.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/module.h>
#include <linux/platform_device.h>

struct keystone_crypto_data {
	struct clk	*clk;
};

static int __devinit keystone_crypto_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct keystone_crypto_data *crypto;
	int ret;

	crypto = devm_kzalloc(dev, sizeof(*crypto), GFP_KERNEL);
	if (!crypto)
		return -ENOMEM;

	crypto->clk = clk_get(dev, NULL);
	if (IS_ERR_OR_NULL(crypto->clk))
		return -ENODEV;

	ret = clk_prepare_enable(crypto->clk);
	if (ret < 0) {
		clk_put(crypto->clk);
		return ret;
	}

	platform_set_drvdata(pdev, crypto);
	dev_info(dev, "crypto accelerator enabled\n");

	return 0;
}

static int __devexit keystone_crypto_remove(struct platform_device *pdev)
{
	struct keystone_crypto_data *crypto = platform_get_drvdata(pdev);

	clk_disable_unprepare(crypto->clk);
	clk_put(crypto->clk);
	kfree(crypto);
	return 0;
}

static struct of_device_id __devinitdata of_match[] = {
	{ .compatible = "ti,keystone-crypto", },
	{},
};
MODULE_DEVICE_TABLE(of, of_match);

static struct platform_driver keystone_crypto_driver = {
	.probe	= keystone_crypto_probe,
	.remove	= __devexit_p(keystone_crypto_remove),
	.driver	= {
		.name		= "keystone-crypto",
		.owner		= THIS_MODULE,
		.of_match_table	= of_match,
	},
};

static int __init keystone_crypto_mod_init(void)
{
	return  platform_driver_register(&keystone_crypto_driver);
}

static void __exit keystone_crypto_mod_exit(void)
{
	platform_driver_unregister(&keystone_crypto_driver);
}

module_init(keystone_crypto_mod_init);
module_exit(keystone_crypto_mod_exit);

MODULE_DESCRIPTION("Keystone crypto acceleration support.");
MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Sandeep Nair <sandeep_n@ti.com>");

