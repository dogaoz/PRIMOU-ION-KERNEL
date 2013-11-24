/* linux/arch/arm/mach-msm/board-primoc-keypad.c
 *
 * Copyright (C) 2010-2012 HTC Corporation.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/platform_device.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/gpio_event.h>
#include <linux/keyreset.h>
#include <asm/mach-types.h>
#include <linux/gpio.h>
#include <mach/gpio.h>

#include "board-primoc.h"
#include "proc_comm.h"
#include <linux/mfd/pmicmax8957.h>

static char *keycaps = "--qwerty";
#undef MODULE_PARAM_PREFIX
#define MODULE_PARAM_PREFIX "board_primoc."
module_param_named(keycaps, keycaps, charp, 0);

static struct gpio_event_direct_entry primoc_keypad_input_map[] = {
	{
		.gpio = PRIMOC_GPIO_KEYPAD_POWER_KEY,
		.code = KEY_POWER,
	},
	{
		.gpio = MAX8957_GPIO_PM_TO_SYS(PRIMOC_VOL_UP),
		.code = KEY_VOLUMEUP,
	},
	{
		.gpio = MAX8957_GPIO_PM_TO_SYS(PRIMOC_VOL_DN),
		.code = KEY_VOLUMEDOWN,
	},
};

uint32_t inputs_gpio_table[] = {
	PCOM_GPIO_CFG(PRIMOC_GPIO_KEYPAD_POWER_KEY, 0, GPIO_INPUT,
		      GPIO_PULL_UP, GPIO_4MA),
};

static void primoc_setup_input_gpio(void)
{
	gpio_tlmm_config(inputs_gpio_table[0], GPIO_CFG_ENABLE);
}

static struct gpio_event_input_info primoc_keypad_input_info = {
	.info.func = gpio_event_input_func,
	.flags = GPIOEDF_PRINT_KEYS | GPIOEDF_PRINT_KEY_UNSTABLE,
	.type = EV_KEY,
#if BITS_PER_LONG != 64 && !defined(CONFIG_KTIME_SCALAR)
	.debounce_time.tv.nsec = 20 * NSEC_PER_MSEC,
# else
	.debounce_time.tv64 = 20 * NSEC_PER_MSEC,
# endif
	.keymap = primoc_keypad_input_map,
	.keymap_size = ARRAY_SIZE(primoc_keypad_input_map),
	.setup_input_gpio = primoc_setup_input_gpio,
};

static struct gpio_event_info *primoc_keypad_info[] = {
	&primoc_keypad_input_info.info,
};

static struct gpio_event_platform_data primoc_keypad_data = {
	.names = {
		"primoc-keypad",
		NULL,
	},
	.info = primoc_keypad_info,
	.info_count = ARRAY_SIZE(primoc_keypad_info),
};

static struct platform_device primoc_keypad_input_device = {
	.name = GPIO_EVENT_DEV_NAME,
	.id = 0,
	.dev		= {
		.platform_data	= &primoc_keypad_data,
	},
};

static struct keyreset_platform_data primoc_reset_keys_pdata = {
	.keys_down = {
		KEY_POWER,
		KEY_VOLUMEDOWN,
		KEY_VOLUMEUP,
		0
	},
};

struct platform_device primoc_reset_keys_device = {
	.name = KEYRESET_NAME,
	.dev.platform_data = &primoc_reset_keys_pdata,
};

int __init primoc_init_keypad(void)
{
	printk(KERN_DEBUG "%s\n", __func__);

	if (platform_device_register(&primoc_reset_keys_device))
		printk(KERN_WARNING "%s: register reset key fail\n", __func__);

	return platform_device_register(&primoc_keypad_input_device);
}
