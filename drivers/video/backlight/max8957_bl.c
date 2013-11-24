/*
 *  Max8957 Backlight Driver
 *
 *  Copyright (c) 2011 Maxim Integrated Product
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2 as
 *  published by the Free Software Foundation.
 *
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/mutex.h>
#include <linux/fb.h>
#include <linux/backlight.h>
#include <linux/mfd/pmicmax8957.h>
#include <linux/mfd/max8957_bl.h>
#include <linux/delay.h>

struct max8957_bl_data {
	int bl_intensity;
	struct device *dev;
};

struct max8957_bl_data *max8957_bl_data;

/*
static int max8957_bl_send_intensity(struct backlight_device *bd)
{
	struct max8957_chip *max8957_chip = dev_get_drvdata(bd->dev.parent);
	u8 reg_data;
	u8 intensity = bd->props.brightness;
	int ret;

	if (bd->props.power != FB_BLANK_UNBLANK)
		intensity = 0;
	if (bd->props.state & BL_CORE_FBBLANK)
		intensity = 0;
	if (bd->props.state & BL_CORE_SUSPENDED)
		intensity = 0;

	ret = max8957_write(max8957_chip, MAX8957_REG_IWLED, &intensity, 1);
	if (unlikely(ret < 0))
		return ret;

	if (intensity == 0 && max8957_bl_data->max8957_bl_intensity != 0)
	{
		ret = max8957_read(max8957_chip, MAX8957_REG_WLEDBSTCNTL1, &reg_data, 1);
		if (unlikely(ret < 0))
			return ret;

		reg_data = reg_data & ~MAX8957_WLED1EN_EN;

		ret = max8957_write(max8957_chip, MAX8957_REG_WLEDBSTCNTL1, &reg_data, 1);
		if (unlikely(ret < 0))
			return ret;
	}
	else if (intensity != 0 && max8957_bl_data->max8957_bl_intensity == 0) // Turn on WLED if it's off
	{
		ret = max8957_read(max8957_chip, MAX8957_REG_WLEDBSTCNTL1, &reg_data, 1);
		if (unlikely(ret < 0))
			return ret;

		reg_data = reg_data | MAX8957_WLED1EN_EN;

		ret = max8957_write(max8957_chip, MAX8957_REG_WLEDBSTCNTL1, &reg_data, 1);
		if (unlikely(ret < 0))
			return ret;
	}

	max8957_bl_data->bl_intensity = intensity;
	return 0;
}

static int max8957_bl_get_intensity(struct backlight_device *bd)
{
	return max8957_bl_data->bl_intensity;
}

static const struct backlight_ops max8957_bl_ops =
{
	.options = BL_CORE_SUSPENDRESUME,
	.get_brightness = max8957_bl_get_intensity,
	.update_status  = max8957_bl_send_intensity,
};
*/

int max8957_set_bl_on_off(enum max8957_wled_ch wled_ch, enum max8957_wled_en wled_en)
{
	struct max8957_chip *max8957_chip = NULL;
	u8 reg_data, mask;
	int rc = 0;

#if MAXIM_BACKLIGHT_DEBUG
	pr_err("%s: wled_ch=%d, wled_en=%d\n", __func__, wled_ch, wled_en);
#endif

	if (!max8957_bl_data) {
		pr_err("%s: max8957_bl_data is not initialized\n", __func__);
		return -ENODEV;
	}

	max8957_chip = dev_get_drvdata(max8957_bl_data->dev);
	if (!max8957_chip) {
		pr_err("%s: failed to get max8957_chip\r\n", __func__);
		return -ENODEV;
	}

	if (wled_ch == MAX8957_WLED_1) {
		reg_data = (wled_en << MAX8957_WLED1EN_SHIFT) & MAX8957_WLED1EN_M;
		mask = MAX8957_WLED1EN_M;
	} else if (wled_ch == MAX8957_WLED_2) {
		reg_data = (wled_en << MAX8957_WLED2EN_SHIFT) & MAX8957_WLED2EN_M;
		mask = MAX8957_WLED2EN_M;
	} else {
		pr_err("%s: Invalid WLED channel\n", __func__);
		return -ENODEV;
	}

#if MAXIM_BACKLIGHT_DEBUG
	pr_err("%s: set MAX8957_REG_WLEDBSTCNTL1 to 0x%x, mask=0x%x\n", __func__, reg_data, mask);
#endif
    rc = max8957_set_bits(max8957_chip, MAX8957_REG_WLEDBSTCNTL1, mask, reg_data);
	if (rc)
		pr_err("%s: Failed to set MAX8957_REG_WLEDBSTCNTL1, rc=%d\n", __func__, rc);

	return rc;
}
EXPORT_SYMBOL(max8957_set_bl_on_off);

static int max8957_bl_probe(struct platform_device *pdev)
{
	struct max8957_backlight_platform_data *pdata = pdev->dev.platform_data;
	struct max8957_chip *max8957_chip = dev_get_drvdata(pdev->dev.parent);
	u8 reg_data;
	int rc = 0;

#if MAXIM_BACKLIGHT_DEBUG
	pr_err("%s:+\n", __func__);
#endif

	if (!max8957_chip) {
		pr_err("%s: failed to get max8957_chip\r\n", __func__);
		return -ENODEV;
	}

	max8957_bl_data = kzalloc(sizeof(struct max8957_bl_data), GFP_KERNEL);
	if (!max8957_bl_data) {
		printk(KERN_ERR "%s: kzalloc fail !!!\n", __func__);
		return -ENOMEM;
	}

	max8957_bl_data->dev = pdev->dev.parent;

	/*
	memset(&props, 0, sizeof(struct backlight_properties));
	props.max_brightness = 0xFF;
	bd = backlight_device_register(pdata->name, &pdev->dev, NULL, &max8957_bl_ops, &props);
	if (IS_ERR(bd))
		return PTR_ERR (bd);

	bd->dev.parent = pdev->dev.parent;
	platform_set_drvdata(pdev, bd);

	bd->props.power = FB_BLANK_UNBLANK;
	bd->props.brightness = 0x20;
	backlight_update_status(bd);
	*/

	/* Set WLEDxEN */
	reg_data = ((pdata->wled1_en << MAX8957_WLED1EN_SHIFT) & MAX8957_WLED1EN_M) |
				((pdata->wled2_en << MAX8957_WLED2EN_SHIFT) & MAX8957_WLED2EN_M);

	/* Set WLEDPWMxEN */
	reg_data |= ((pdata->wledpwm1_en << MAX8957_WLEDPWM1EN_SHIFT) & MAX8957_WLEDPWM1EN_M) |
				((pdata->wledpwm2_en << MAX8957_WLEDPWM2EN_SHIFT) & MAX8957_WLEDPWM2EN_M);

	/* Set WLEDFOSC (Converter Switching Frequency Select) */
	reg_data |= (pdata->wledfosc << MAX8957_WLEDFOSC_SHIFT) & MAX8957_WLEDFOSC_M;

	rc = max8957_write(max8957_chip, MAX8957_REG_WLEDBSTCNTL1, &reg_data, 1);
	if (rc) {
		pr_err("%s: Failed to set MAX8957_REG_WLEDBSTCNTL1, rc=%d\n", __func__, rc);
		goto err;
	}

	/* Set brightness */
	reg_data = pdata->iwled;
	rc = max8957_write(max8957_chip, MAX8957_REG_IWLED, &reg_data, 1);
	if (rc) {
		pr_err("%s: Failed to set MAX8957_REG_IWLED, rc=%d\n", __func__, rc);
		goto err;
	}

#if MAXIM_BACKLIGHT_DEBUG
	pr_err("%s:-\n", __func__);
#endif

	return rc;

err:
#if MAXIM_BACKLIGHT_DEBUG
	pr_err("%s:-, Failed, rc=%d\n", __func__, rc);
#endif

	return rc;
}

static int max8957_bl_remove(struct platform_device *pdev)
{
	/*
	struct backlight_device *bd = platform_get_drvdata(pdev);

	bd->props.power = 0;
	bd->props.brightness = 0;
	backlight_update_status(bd);

	backlight_device_unregister(bd);
	*/

	return 0;
}

static struct platform_driver max8957_bl_driver = {
	.probe		= max8957_bl_probe,
	.remove		= max8957_bl_remove,
	.driver		=
	{
		.name	= "max8957-bl",
		.owner = THIS_MODULE,
	},
};

static int __init max8957_bl_init(void)
{
	return platform_driver_register(&max8957_bl_driver);
}

static void __exit max8957_bl_exit(void)
{
	platform_driver_unregister(&max8957_bl_driver);
}

module_init(max8957_bl_init);
module_exit(max8957_bl_exit);

MODULE_DESCRIPTION("Max8957 Backlight Driver");
MODULE_VERSION("1.0");
MODULE_LICENSE("GPL v2");
