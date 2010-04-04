/*
 * drivers/video/samsung/s3cfb_picturetubes.c
 *
 * Copyright (c) 2009 Barnes and Noble, Inc
 * All rights reserved.
 *
 * Module Author: Intrinsyc Software, Inc
 *
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file COPYING in the main directory of this archive for
 * more details.
 *
 *	S3C Frame Buffer Driver
 *	based on skeletonfb.c, sa1100fb.h, s3c2410fb.c
 */

#include <linux/wait.h>
#include <linux/fb.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/leds.h>
#include <linux/workqueue.h>
#include <linux/mutex.h>
#include <linux/regulator/consumer.h>

#include <plat/regs-lcd.h>
#include <plat/gpio-cfg.h>
#include <plat/regs-clock.h>
#include <plat/regs-gpio.h>
#include <mach/bravo_gpio.h>

#include "s3cfb.h"

#define BACKLIGHT_LEVEL_MIN		0
#define BACKLIGHT_LEVEL_DEFAULT		64
#define BACKLIGHT_LEVEL_MAX		255

static struct regulator *backlight_reg = NULL;

int lcd_power = ON;
EXPORT_SYMBOL(lcd_power);

void lcd_power_ctrl(s32 value);
EXPORT_SYMBOL(lcd_power_ctrl);

int backlight_power = OFF;
EXPORT_SYMBOL(backlight_power);

void backlight_power_ctrl(s32 value);
EXPORT_SYMBOL(backlight_power_ctrl);

int backlight_level = BACKLIGHT_LEVEL_DEFAULT;
EXPORT_SYMBOL(backlight_level);

void backlight_level_ctrl(s32 value);
EXPORT_SYMBOL(backlight_level_ctrl);

int powering_down = 0;
EXPORT_SYMBOL(powering_down);

#define S3CFB_HFP		10		/* front porch */
#define S3CFB_HSW		4		/* hsync width */
#define S3CFB_HBP		10		/* back porch */

#define S3CFB_VFP		4		/* front porch */
#define S3CFB_VSW		2		/* vsync width */
#define S3CFB_VBP		4		/* back porch */

#define S3CFB_HRES		240	/* horizon pixel  x resolition */
#define S3CFB_VRES		480	/* line cnt       y resolution */

#define S3CFB_HRES_VIRTUAL	240		/* horizon pixel  x resolition */
#define S3CFB_VRES_VIRTUAL	960		/* line cnt       y resolution */

#define S3CFB_HRES_OSD		240	/* horizon pixel  x resolition */
#define S3CFB_VRES_OSD		480	/* line cnt       y resolution */

#define S3CFB_VFRAME_FREQ  	69	/* frame rate freq */

#define S3CFB_PIXEL_CLOCK 	9000000 /* 9 MHz pixel clock */

static void s3cfb_set_fimd_info(void)
{
	s3cfb_fimd.vidcon1 = S3C_VIDCON1_IHSYNC_INVERT | S3C_VIDCON1_IVSYNC_INVERT | S3C_VIDCON1_IVDEN_NORMAL | S3C_VIDCON1_IVCLK_RISE_EDGE;
	s3cfb_fimd.vidtcon0 = S3C_VIDTCON0_VBPD(S3CFB_VBP - 1) | S3C_VIDTCON0_VFPD(S3CFB_VFP - 1) | S3C_VIDTCON0_VSPW(S3CFB_VSW - 1);
	s3cfb_fimd.vidtcon1 = S3C_VIDTCON1_HBPD(S3CFB_HBP - 1) | S3C_VIDTCON1_HFPD(S3CFB_HFP - 1) | S3C_VIDTCON1_HSPW(S3CFB_HSW - 1);
	s3cfb_fimd.vidtcon2 = S3C_VIDTCON2_LINEVAL(S3CFB_VRES - 1) | S3C_VIDTCON2_HOZVAL(S3CFB_HRES - 1);

	s3cfb_fimd.vidosd0a = S3C_VIDOSDxA_OSD_LTX_F(0) | S3C_VIDOSDxA_OSD_LTY_F(0);
	s3cfb_fimd.vidosd0b = S3C_VIDOSDxB_OSD_RBX_F(S3CFB_HRES - 1) | S3C_VIDOSDxB_OSD_RBY_F(S3CFB_VRES - 1);

	s3cfb_fimd.vidosd1a = S3C_VIDOSDxA_OSD_LTX_F(0) | S3C_VIDOSDxA_OSD_LTY_F(0);
	s3cfb_fimd.vidosd1b = S3C_VIDOSDxB_OSD_RBX_F(S3CFB_HRES_OSD - 1) | S3C_VIDOSDxB_OSD_RBY_F(S3CFB_VRES_OSD - 1);

	s3cfb_fimd.width = S3CFB_HRES;
	s3cfb_fimd.height = S3CFB_VRES;
	s3cfb_fimd.xres = S3CFB_HRES;
	s3cfb_fimd.yres = S3CFB_VRES;

	// Offset should point to bottom of display area
	s3cfb_fimd.fixed_yoffset = 0;
	s3cfb_fimd.lcd_height = S3CFB_VRES;
	s3cfb_fimd.lcd_width = S3CFB_HRES;

#if defined(CONFIG_FB_S3C_VIRTUAL_SCREEN)
	s3cfb_fimd.xres_virtual = S3CFB_HRES_VIRTUAL;
	s3cfb_fimd.yres_virtual = S3CFB_VRES_VIRTUAL;
#else
	s3cfb_fimd.xres_virtual = S3CFB_HRES;
	s3cfb_fimd.yres_virtual = S3CFB_VRES;
#endif

	s3cfb_fimd.osd_width = S3CFB_HRES_OSD;
	s3cfb_fimd.osd_height = S3CFB_VRES_OSD;
	s3cfb_fimd.osd_xres = S3CFB_HRES_OSD;
	s3cfb_fimd.osd_yres = S3CFB_VRES_OSD;

	s3cfb_fimd.osd_xres_virtual = S3CFB_HRES_OSD;
	s3cfb_fimd.osd_yres_virtual = S3CFB_VRES_OSD;

	s3cfb_fimd.pixclock = S3CFB_PIXEL_CLOCK;

	s3cfb_fimd.hsync_len = S3CFB_HSW;
	s3cfb_fimd.vsync_len = S3CFB_VSW;
	s3cfb_fimd.left_margin = S3CFB_HFP;
	s3cfb_fimd.upper_margin = S3CFB_VFP;
	s3cfb_fimd.right_margin = S3CFB_HBP;
	s3cfb_fimd.lower_margin = S3CFB_VBP;

	s3cfb_fimd.set_lcd_power = lcd_power_ctrl;
	s3cfb_fimd.set_backlight_power = backlight_power_ctrl;
	s3cfb_fimd.set_brightness = backlight_level_ctrl;

	s3cfb_fimd.backlight_min = BACKLIGHT_LEVEL_MIN;
	s3cfb_fimd.backlight_max = BACKLIGHT_LEVEL_MAX;
}

#if  defined(CONFIG_S3C6410_PWM)
extern void s3cfb_set_brightness(int val);
#else
void s3cfb_set_brightness(int val){}
#endif

void lcd_power_ctrl(s32 value)
{
	lcd_power = value;

	//printk(KERN_ERR "%s lcd_power: %d backlight_power: %d\n", __FUNCTION__, lcd_power, backlight_power);

	if (lcd_power && backlight_power) {
		/* power on the backlight here as well in case it was deferred */
		backlight_power_ctrl(backlight_power);
	}
}

static void backlight_ctrl(s32 value)
{
	int ret = 0;
	
	if (backlight_reg == NULL) {
		printk(KERN_ERR "backlight regulator unavailable!\n");
		return;
	}

	if (value && (!regulator_is_enabled(backlight_reg))) {
		ret = regulator_enable(backlight_reg);
	}

	else if (regulator_is_enabled(backlight_reg)) {
		ret = regulator_disable(backlight_reg);
	}

	if (ret) {
		printk(KERN_ERR "failed to %s regulator: %d\n", value ? "enable" : "disable", ret);
	}
}

void backlight_level_ctrl(s32 value)
{
	int ret;

	value = (value / 7); /* Clamp to a range between 0 and 40 */

	if (value == 0) {
		/* If value is 0 just round it 'up' to 1 mA */
		value = 1;
	}

	if ((value < BACKLIGHT_LEVEL_MIN) ||	/* Invalid Value */
		(value > BACKLIGHT_LEVEL_MAX) ||
		(value == backlight_level))	/* Same Value */
		return;

	backlight_level = value;

	if (backlight_reg == NULL) {
		printk(KERN_ERR "backlight regulator unavailable!\n");		
	}

	ret = regulator_set_current_limit(backlight_reg, 0, backlight_level * 1000);

	if (ret) {
		printk(KERN_ERR "failed to set current limit: %d value: %d\n", ret, value);
	}
}

void backlight_power_ctrl(s32 value)
{
	if ((value < OFF) ||	/* Invalid Value */
		(value > ON))
		return;

	//printk(KERN_ERR "%s lcd_power: %d\n", __FUNCTION__, lcd_power);

	if (lcd_power) {
		/* Don't turn on the backlight until lcd is powered up. */
		backlight_ctrl((value ? ON : OFF));	
	}
	
	backlight_power = value;
}

void s3cfb_init_hw(struct device *dev)
{
	int spcon;
	printk(KERN_INFO "LCD TYPE :: Picture Toob will be initialized\n");

	backlight_reg = regulator_get(dev, "ISINKA");

	if (IS_ERR(backlight_reg)) {
		printk(KERN_ERR "failed to request lcd-backlight regulator\n");
	}

	s3cfb_set_fimd_info();
	s3cfb_set_gpio();

	spcon = get_s3c_reg_val(S3C64XX_SPCON);
	spcon &= ~(0x3 << 18);
	spcon |= (0x1 << 18);
	set_s3c_reg_val(S3C64XX_SPCON, spcon);
}

