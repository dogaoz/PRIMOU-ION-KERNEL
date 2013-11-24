/* linux/arch/arm/mach-msm/board-primoc.h
 * Copyright (C) 2007-2009 HTC Corporation.
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

#ifndef __ARCH_ARM_MACH_MSM_BOARD_PRIMOC_H
#define __ARCH_ARM_MACH_MSM_BOARD_PRIMOC_H

#include <mach/board.h>

/* Macros assume PMIC GPIOs start at 0 */
#ifdef CONFIG_PMIC8058

/* QCT PMIC8058 */
#define PM8058_GPIO_PM_TO_SYS(pm_gpio)     (pm_gpio + NR_GPIO_IRQS)
#define PM8058_GPIO_SYS_TO_PM(sys_gpio)    (sys_gpio - NR_GPIO_IRQS)
#else

/* Maxim MAX8957 */
#define MAX8957_GPIO_BASE			NR_GPIO_IRQS
#define MAX8957_GPIO_PM_TO_SYS(pm_gpio)		(pm_gpio + MAX8957_GPIO_BASE)
#define MAX8957_GPIO_SYS_TO_PM(sys_gpio)	(sys_gpio - MAX8957_GPIO_BASE)
#endif

#define MSM_LINUX_BASE1		0x14300000
#define MSM_LINUX_SIZE1		0x0BD00000
#define MSM_LINUX_BASE2		0x20000000
#define MSM_LINUX_SIZE2		0x10000000
#define MSM_MEM_256MB_OFFSET	0x10000000

#define MSM_GPU_MEM_BASE	0x00100000
#define MSM_GPU_MEM_SIZE	0x003F0000

#define MSM_FB_SIZE             0x00500000
#define MSM_PMEM_ADSP_SIZE	0x02600000
#define MSM_PMEM_ADSP2_SIZE	0x002C0000
#define MSM_PMEM_AUDIO_SIZE	0x00200000
#define MSM_PMEM_SF_SIZE	0x01200000
#define PMEM_KERNEL_EBI1_SIZE   0x00700000

#define PRIMOC_GPIO_WIFI_IRQ		147
#define PRIMOC_GPIO_WIFI_SHUTDOWN_N	39

#define PRIMOC_GPIO_KEYPAD_POWER_KEY	46

#define PRIMOC_GPIO_TORCH_EN		98
#define PRIMOC_GPIO_PHANTOM		102
#ifdef CONFIG_MFD_MAX8957
#define PRIMOC_GPIO_FLASH_EN		23
#define PRIMOC_GPIO_VIBRTOR_EN		24
#endif

#define PRIMOC_GPIO_PS_INT_N		180


#define PRIMOC_LAYOUTS			{ \
		{ { 0,  1, 0}, { -1, 0,  0}, {0, 0,  1} }, \
		{ {-1,  0, 0}, { 0, -1,  0}, {0, 0,  1} }, \
		{ {-1,  0, 0}, { 0,  -1, 0}, {0, 0,  1} }, \
		{ { 0,  1, 0}, { 0,  0, -1}, {1, 0,  0} }  \
					}

#define PRIMOC_MDDI_TE		(30)
#define PRIMOC_LCD_RSTz		(126)
#define PRIMOC_LCD_ID1		(128)
#define PRIMOC_LCD_ID0		(129)

#define PRIMOC_AUD_CODEC_EN	(36)
#define PRIMOC_AUD_MICPATH_SEL	(127)
#define PRIMOC_AUD_SPK_SD	(108)
#define PRIMOC_AUD_AMP_EN	(109)

/* BT */
#define PRIMOC_GPIO_BT_UART1_RTS	(134)
#define PRIMOC_GPIO_BT_UART1_CTS	(135)
#define PRIMOC_GPIO_BT_UART1_RX		(136)
#define PRIMOC_GPIO_BT_UART1_TX		(137)
#define PRIMOC_GPIO_BT_WAKE		(31)
#define PRIMOC_GPIO_BT_SHUTDOWN_N	(38)
#define PRIMOC_GPIO_BT_RESET_N		(41)
#define PRIMOC_GPIO_BT_HOST_WAKE	(44)

#define PRIMOC_GPIO_BT_PCM_OUT		(138)
#define PRIMOC_GPIO_BT_PCM_IN		(139)
#define PRIMOC_GPIO_BT_PCM_SYNC		(140)
#define PRIMOC_GPIO_BT_PCM_CLK		(141)

/* Vibrator */
#define PRIMOC_GPIO_VIBRATOR_ON		(168)

/* USB */
#define PRIMOC_AUDIOz_UART_SW		(109)
#define PRIMOC_USB_UART_SW		(96)
#define PRIMOC_USBz_AUDIO_SW		(95)

#define PRIMOC_GPIO_USB_ID_PIN		(49)
#define PRIMOC_GPIO_USB_ID1_PIN		(145)

#define PRIMOC_GPIO_PS_HOLD		(29)

/* 35mm Headset */
#define PRIMOC_GPIO_35MM_HEADSET_DET	(26)

/* EMMC */
#define PRIMOC_GPIO_EMMC_RST		(88)

/* Touch */
#define PRIMOC_GPIO_TP_ATT_N		(20)
#define PRIMOC_GPIO_TP_RSTz		(21)

/* PMIC MAX8957 GPIO definition */
#define PMGPIO(x) 			(x)
#define PRIMOC_VOL_UP			PMGPIO(0)
#define PRIMOC_VOL_DN			PMGPIO(1)
#define PRIMOC_AUD_REMO_PRESz		PMGPIO(2)
#define PRIMOC_GPIO_GSENSOR_INT		PMGPIO(5)

/* Camera */
#define PRIMOC_S5K4E5YX_EVT2		(2)
#define PRIMOC_S5K4E5YX_EVT1		(1)
#define PRIMOC_GPIO_CAM_I2C_SCL		(16)
#define PRIMOC_GPIO_CAM_I2C_SDA		(17)
#define PRIMOC_GPIO_CAM1_PWD		(35)
#define PRIMOC_GPIO_CAM1_VCM_PWD	(34)
#define PRIMOC_GPIO_CAM_MCLK		(15)
#define PRIMOC_GPIO_CAM_ID		(87)

#ifdef CONFIG_RAWCHIP
/* Rawchip */
#define PRIMOC_GPIO_RAW_INTR0		(142)
#define PRIMOC_GPIO_RAW_INTR1		(18)
#define PRIMOC_GPIO_RAW_RSTN		(19)
/* Rawchip SPI */
#define PRIMOC_GPIO_MCAM_SPI_DO		(47)
#define PRIMOC_GPIO_MCAM_SPI_DI		(48)
#define PRIMOC_GPIO_MCAM_SPI_CLK	(45)
#define PRIMOC_GPIO_MCAM_SPI_CS		(89)
#endif

#define PRIMOC_SD_CDETz			(42)

/* Display */
extern struct platform_device msm_device_mdp;
extern struct platform_device msm_device_mddi0;
extern int panel_type;

int primoc_init_mmc(unsigned int sys_rev);
void __init primoc_audio_init(void);
int primoc_init_keypad(void);
int __init primoc_wifi_init(void);

#endif /* __ARCH_ARM_MACH_MSM_BOARD_PRIMOC_H */
