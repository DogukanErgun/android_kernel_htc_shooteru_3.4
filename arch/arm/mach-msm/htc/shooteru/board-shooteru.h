/* linux/arch/arm/mach-msm/board-spade.h
 *
 * Copyright (C) 2010-2011 HTC Corporation.
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

#ifndef __ARCH_ARM_MACH_MSM_BOARD_SHOOTERU_H
#define __ARCH_ARM_MACH_MSM_BOARD_SHOOTERU_H

#include <mach/board.h>
#include <mach/msm_memtypes.h>
#include <mach/rpm-regulator.h>
#include <linux/regulator/pmic8901-regulator.h>
#include <mach/board-msm8660.h>

#define SHOOTERU_PROJECT_NAME	"shooteru"

#define MSM_RAM_CONSOLE_BASE	MSM_HTC_RAM_CONSOLE_PHYS
#define MSM_RAM_CONSOLE_SIZE	MSM_HTC_RAM_CONSOLE_SIZE

/*** Memory map ***/
#define MSM_SMI_BASE         0x38000000
#define MSM_SMI_SIZE         0x4000000

#ifdef CONFIG_FB_MSM_TRIPLE_BUFFER
#define MSM_FB_PRIM_BUF_SIZE (960 * 544 * 4 * 3) /* 4 bpp x 3 pages */
#else
#define MSM_FB_PRIM_BUF_SIZE (960 * 544 * 4 * 2) /* 4 bpp x 2 pages */
#endif


#ifdef CONFIG_FB_MSM_HDMI_MSM_PANEL
#define MSM_FB_EXT_BUF_SIZE (1920 * 1088 * 2 * 1) /* 2 bpp x 1 page */
#elif defined(CONFIG_FB_MSM_TVOUT)
#define MSM_FB_EXT_BUF_SIZE (720 * 576 * 2 * 2) /* 2 bpp x 2 pages */
#else
#define MSM_FB_EXT_BUF_SIZE 0
#endif

/* Note: must be multiple of 4096 */
#define MSM_FB_SIZE roundup(MSM_FB_PRIM_BUF_SIZE + MSM_FB_EXT_BUF_SIZE, 4096)

#define PHY_BASE_ADDR1       0x48000000
#define SIZE_ADDR1           0x38000000

#define MSM_ION_MM_FW_SIZE   0x200000
#define MSM_ION_MM_SIZE      0x3D00000
#define MSM_ION_MFC_SIZE     0x100000
#define MSM_ION_SF_SIZE      0x2A00000
#define MSM_ION_AUDIO_SIZE   0x4CF000

#define MSM_ION_HEAP_NUM     6

#define MSM_ION_MM_FW_BASE   0x38000000
#define MSM_ION_MM_BASE      0x38200000
#define MSM_ION_MFC_BASE     0x3BF00000
#define MSM_ION_SF_BASE      0x40400000
/*** END Memory map ***/

/* GPIO definition */


/* Direct Keys */
#define SHOOTERU_GPIO_SW_LCM_3D		(64)
#define SHOOTERU_GPIO_SW_LCM_2D		(68)
#define SHOOTERU_GPIO_KEY_VOL_DOWN	(103)
#define SHOOTERU_GPIO_KEY_VOL_UP		(104)
#define SHOOTERU_GPIO_KEY_CAM_STEP2	(115)
#define SHOOTERU_GPIO_KEY_CAM_STEP1	(123)
#define SHOOTERU_GPIO_KEY_POWER		(125)

/* Battery */
#define SHOOTERU_GPIO_MBAT_IN            (61)
#define SHOOTERU_GPIO_CHG_INT		(126)

/* Wifi */
#define SHOOTERU_GPIO_WIFI_IRQ              (46)
#define SHOOTERU_GPIO_WIFI_SHUTDOWN_N       (96)
/* Sensors */
#define SHOOTERU_SENSOR_I2C_SDA		(72)
#define SHOOTERU_SENSOR_I2C_SCL		(73)
#define SHOOTERU_GYRO_INT               (127)
#define SHOOTERU_ECOMPASS_INT           (128)
#define SHOOTERU_GSENSOR_INT           (129)

/* Microp */

/* TP */

#define SHOOTERU_TP_ATT_N_XC   (57)
#define SHOOTERU_TP_I2C_SDA           (51)
#define SHOOTERU_TP_I2C_SCL           (52)
#define SHOOTERU_TP_ATT_N             (65)
#define SHOOTERU_TP_ATT_N_XB       (50)

/* LCD */
#define GPIO_LCD_TE	28
#define GPIO_LCM_RST_N			(66)
#define GPIO_LCM_ID			(50)

/* Audio */
#define SHOOTERU_AUD_CODEC_RST        (67)
#define SHOOTERU_AUD_MIC_SEL		PMGPIO(14)

/* BT */
#define SHOOTERU_GPIO_BT_HOST_WAKE      (45)
#define SHOOTERU_GPIO_BT_UART1_TX       (53)
#define SHOOTERU_GPIO_BT_UART1_RX       (54)
#define SHOOTERU_GPIO_BT_UART1_CTS      (55)
#define SHOOTERU_GPIO_BT_UART1_RTS      (56)
#define SHOOTERU_GPIO_BT_SHUTDOWN_N     (100)
#define SHOOTERU_GPIO_BT_CHIP_WAKE      (130)
#define SHOOTERU_GPIO_BT_RESET_N        (142)

/* USB */
#define SHOOTERU_GPIO_USB_ID        (63)
#define SHOOTERU_GPIO_MHL_RESET        (70)
#define SHOOTERU_GPIO_MHL_INT        (71)
#define SHOOTERU_GPIO_MHL_USB_SWITCH        (99)

/* Camera */
#define SHOOTERU_CAM_CAM1_ID           (10)
#define SHOOTERU_CAM_I2C_SDA           (47)
#define SHOOTERU_CAM_I2C_SCL           (48)
#define SHOOTERU_CAM_MCLK     	 (32)
#define SHOOTERU_CAM_VCM_PD      (58)
#define SHOOTERU_CAM1_RSTz       (137)
#define SHOOTERU_CAM2_RSTz       (138)
#define SHOOTERU_CAM2_PWDN       (140)
#define SHOOTERU_MCLK_SWITCH     (141)


/* General */
#define SHOOTERU_GENERAL_I2C_SDA		(59)
#define SHOOTERU_GENERAL_I2C_SCL		(60)

/* Flashlight */
#define SHOOTERU_FLASH_EN             (29)
#define SHOOTERU_TORCH_EN             (30)

/* Accessory */
#define SHOOTERU_GPIO_AUD_HP_DET        (31)

/* SPI */
#define SHOOTERU_SPI_DO                 (33)
#define SHOOTERU_SPI_DI                 (34)
#define SHOOTERU_SPI_CS                 (35)
#define SHOOTERU_SPI_CLK                (36)



/* LCM */
#define SHOOTERU_CTL_3D_1		(131)
#define SHOOTERU_CTL_3D_2		(132)
#define SHOOTERU_CTL_3D_3		(133)
#define SHOOTERU_CTL_3D_4		(134)
#define SHOOTERU_LCM_3D_PDz		(135)

/* PMIC */

/* PMIC GPIO definition */
#define PMGPIO(x) (x-1)
#define SHOOTERU_VOL_UP             PMGPIO(16)
#define SHOOTERU_VOL_DN             PMGPIO(17)
#define SHOOTERU_AUD_HP_EN          PMGPIO(18)
#define SHOOTERU_AUD_HANDSET_ENO		PMGPIO(18)
#define SHOOTERU_HAP_ENABLE         PMGPIO(19)
#define SHOOTERU_3DLCM_PD	    PMGPIO(20)
#define SHOOTERU_AUD_QTR_RESET      PMGPIO(21)
#define SHOOTERU_TP_RST             PMGPIO(23)
#define SHOOTERU_GREEN_LED          PMGPIO(24)
#define SHOOTERU_AMBER_LED          PMGPIO(25)
#define SHOOTERU_3DCLK		    PMGPIO(26)
#define SHOOTERU_CHG_STAT	   PMGPIO(33)
#define SHOOTERU_SDC3_DET           PMGPIO(34)
#define SHOOTERU_PLS_INT            PMGPIO(35)
#define SHOOTERU_AUD_REMO_PRES      PMGPIO(37)
#define SHOOTERU_WIFI_BT_SLEEP_CLK  PMGPIO(38)

extern struct regulator_init_data msm_saw_regulator_pdata_s0;
extern struct regulator_init_data msm_saw_regulator_pdata_s1;
extern struct rpm_regulator_platform_data shooteru_rpm_regulator_early_pdata __devinitdata;
extern struct rpm_regulator_platform_data shooteru_rpm_regulator_pdata __devinitdata;
extern struct platform_device msm8x60_8901_mpp_vreg __devinitdata;
extern struct pm8901_vreg_pdata pm8901_regulator_pdata[];
extern int pm8901_regulator_pdata_len;
extern struct platform_device msm_adc_device;

void __init shooteru_init_mmc(void);
int __init shooteru_init_wifi_mmc(void);
void __init shooteru_audio_init(void);
int __init shooteru_init_keypad(void);
void __init shooteru_init_fb(void);
void __init shooteru_init_pmic(void);
int __init shooteru_wifi_init(void);
void __init shooteru_gpio_mpp_init(void);
void shooteru_init_gpiomux(void);
void __init msm8x60_allocate_fb_region(void);
void __init shooteru_pm8901_gpio_mpp_init(void);
void msm8x60_mdp_writeback(struct memtype_reserve *reserve_table);
#ifdef CONFIG_FB_MSM_HDMI_MSM_PANEL
int hdmi_enable_5v(int on);
#endif
#define _GET_REGULATOR(var, name) do {					\
	if (var == NULL) {						\
		var = regulator_get(NULL, name);			\
		if (IS_ERR(var)) {					\
			pr_err("'%s' regulator not found, rc=%ld\n",	\
				name, PTR_ERR(var));			\
			var = NULL;					\
		}							\
	}								\
} while (0)

#endif /* __ARCH_ARM_MACH_MSM_BOARD_SHOOTERU_H */
