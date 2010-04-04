/*
 * drivers/video/samsung/s3cfb_at272.c
 *
 * Copyright (C) 2008 Jinsung Yang <jsgood.yang@samsung.com>
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
#include <linux/gpio.h>
#include <linux/workqueue.h>
#include <linux/mutex.h>
#include <linux/mfd/wm8350/core.h>

#include <plat/regs-gpio.h>
#include <plat/regs-lcd.h>
#include <plat/gpio-cfg.h>
#include <plat/regs-clock.h>

#include "s3cfb.h"

#define BACKLIGHT_LEVEL_MIN		0
#define BACKLIGHT_LEVEL_DEFAULT		0x39
#define BACKLIGHT_LEVEL_MAX		0x39 /* Max is rougly 45 mA */

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

#define S3CFB_HFP		40//8	/* front porch */
#define S3CFB_HSW		1//3	/* hsync width */
#define S3CFB_HBP		40//13	/* back porch */

#define S3CFB_VFP		5//5	/* front porch */
#define S3CFB_VSW		1//1	/* vsync width */
#define S3CFB_VBP		5//7	/* back porch */

#define S3CFB_HRES		480	/* horizon pixel  x resolition */
#define S3CFB_VRES		272	/* line cnt       y resolution */

#define S3CFB_VRES_FB		480
#define S3CFB_HRES_FB		272

#define S3CFB_HRES_VIRTUAL	480	/* horizon pixel  x resolition */
#define S3CFB_VRES_VIRTUAL	544	/* line cnt       y resolution */

#define S3CFB_HRES_OSD		480	/* horizon pixel  x resolition */
#define S3CFB_VRES_OSD		272	/* line cnt       y resolution */

#define S3CFB_VFRAME_FREQ     	50	/* frame rate freq */

#define S3CFB_PIXEL_CLOCK 	9000000//(S3CFB_VFRAME_FREQ * (S3CFB_HFP + S3CFB_HSW + S3CFB_HBP + S3CFB_HRES) * (S3CFB_VFP + S3CFB_VSW + S3CFB_VBP + S3CFB_VRES))

static void s3cfb_set_fimd_info(void)
{
	s3cfb_fimd.vidcon1 = S3C_VIDCON1_IHSYNC_INVERT | S3C_VIDCON1_IVSYNC_INVERT | S3C_VIDCON1_IVDEN_NORMAL;
	s3cfb_fimd.vidtcon0 = S3C_VIDTCON0_VBPD(S3CFB_VBP - 1) | S3C_VIDTCON0_VFPD(S3CFB_VFP - 1) | S3C_VIDTCON0_VSPW(S3CFB_VSW - 1);
	s3cfb_fimd.vidtcon1 = S3C_VIDTCON1_HBPD(S3CFB_HBP - 1) | S3C_VIDTCON1_HFPD(S3CFB_HFP - 1) | S3C_VIDTCON1_HSPW(S3CFB_HSW - 1);
	s3cfb_fimd.vidtcon2 = S3C_VIDTCON2_LINEVAL(S3CFB_VRES - 1) | S3C_VIDTCON2_HOZVAL(S3CFB_HRES - 1);

	s3cfb_fimd.vidosd0a = S3C_VIDOSDxA_OSD_LTX_F(0) | S3C_VIDOSDxA_OSD_LTY_F(0);
	s3cfb_fimd.vidosd0b = S3C_VIDOSDxB_OSD_RBX_F(S3CFB_HRES - 1) | S3C_VIDOSDxB_OSD_RBY_F(S3CFB_VRES - 1);

	s3cfb_fimd.vidosd1a = S3C_VIDOSDxA_OSD_LTX_F(0) | S3C_VIDOSDxA_OSD_LTY_F(0);
	s3cfb_fimd.vidosd1b = S3C_VIDOSDxB_OSD_RBX_F(S3CFB_HRES_OSD - 1) | S3C_VIDOSDxB_OSD_RBY_F(S3CFB_VRES_OSD - 1);

	s3cfb_fimd.width = S3CFB_HRES_FB;
	s3cfb_fimd.height = S3CFB_VRES_FB;
	s3cfb_fimd.xres = S3CFB_HRES_FB;
	s3cfb_fimd.yres = S3CFB_VRES_FB;

	// Offset should point to bottom of display area
	s3cfb_fimd.fixed_yoffset = S3CFB_VRES_FB - S3CFB_VRES;
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

#define WM8350_DEFAULT_BACKLIGHT_BRIGHTNESS	255

struct _wm8350_led
{
	struct wm8350* wm8350;
	struct led_classdev led_dev;

	s32 backlight_off;
	s32 backlight_brightness;
	u8 backlist_last_level;

	struct work_struct work;
	struct mutex mutex;
	spinlock_t value_lock;	
};

static struct _wm8350_led lcd_backlight = {
	.backlight_off = 0,
	.backlight_brightness = WM8350_DEFAULT_BACKLIGHT_BRIGHTNESS
};

#if  defined(CONFIG_S3C6410_PWM)
extern void s3cfb_set_brightness(int val);
#else
void s3cfb_set_brightness(int val){}
#endif

void lcd_power_ctrl(s32 value)
{
	printk(KERN_ERR "lcd_power_ctrl: 0x%x\n", value);
	lcd_power = value;
}

static void backlight_ctrl(s32 value)
{
	printk(KERN_ERR "backlight_ctrl: 0x%x\n", value);
	// Make sure no one else is playing around with these settings
	mutex_lock(&lcd_backlight.mutex);

	if (value) {
		wm8350_set_bits(lcd_backlight.wm8350, WM8350_POWER_MGMT_7, WM8350_CS1_ENA);
		wm8350_set_bits(lcd_backlight.wm8350, WM8350_CSA_FLASH_CONTROL, WM8350_CS1_DRIVE);
		wm8350_set_bits(lcd_backlight.wm8350, WM8350_DCDC_LDO_REQUESTED, 1 << (WM8350_DCDC_5 - WM8350_DCDC_1));
		/* backlight ON */
		// power up DC5
		backlight_power = ON;
	}
	else {
		/* backlight OFF */
		// power down DC5
		wm8350_clear_bits(lcd_backlight.wm8350, WM8350_DCDC_LDO_REQUESTED, 1 << (WM8350_DCDC_5 - WM8350_DCDC_1));
		wm8350_clear_bits(lcd_backlight.wm8350, WM8350_POWER_MGMT_7, WM8350_CS1_ENA);
		backlight_power = OFF;
	}

	mutex_unlock(&lcd_backlight.mutex);
}

void backlight_level_ctrl(s32 value)
{
	u16 val;
	printk(KERN_ERR "backlight_level_ctrl: 0x%x\n", value);

	if ((value < BACKLIGHT_LEVEL_MIN) ||	/* Invalid Value */
		(value > BACKLIGHT_LEVEL_MAX) ||
		(value == backlight_level))	/* Same Value */
		return;

	if (backlight_power) {
		mutex_lock(&lcd_backlight.mutex);
		val = wm8350_reg_read(lcd_backlight.wm8350, WM8350_CURRENT_SINK_DRIVER_A) & ~WM8350_CS1_ISEL_MASK;
		wm8350_reg_write(lcd_backlight.wm8350, WM8350_CURRENT_SINK_DRIVER_A, val | value);
		mutex_unlock(&lcd_backlight.mutex);
	}
	
	backlight_level = value;	
}

void backlight_power_ctrl(s32 value)
{
	printk(KERN_ERR "backlight_power_ctrl: 0x%x\n", value);

	if ((value < OFF) ||	/* Invalid Value */
		(value > ON) ||
		(value == backlight_power))	/* Same Value */
		return;

	backlight_ctrl((value ? backlight_level : OFF));	
	
	backlight_power = value;	
}

static void wm8350_work(struct work_struct *work)
{
	struct _wm8350_led *led = container_of(work, struct _wm8350_led, work);
	unsigned long flags;
	int val;
	int level;

	mutex_lock(&led->mutex);

	spin_lock_irqsave(&led->value_lock, flags);
	switch(led->backlight_brightness) {
	case LED_OFF:
		level = 0;		
		break;
	case LED_HALF:
		// ~ 22mA
		level = 0x32;
		break;
	case LED_FULL:
		// ~ 45mA
		level = 0x39;
		break;
	default:
		level = 0x28 + ((0xc * led->backlight_brightness) / LED_FULL); 
		break;
	}

	spin_unlock_irqrestore(&led->value_lock, flags);
	
	val = wm8350_reg_read(led->wm8350, WM8350_CURRENT_SINK_DRIVER_A) & ~WM8350_CS1_ISEL_MASK;
	wm8350_reg_write(led->wm8350, WM8350_CURRENT_SINK_DRIVER_A, val | level);

	wm8350_set_bits(lcd_backlight.wm8350, WM8350_POWER_MGMT_7, WM8350_CS1_ENA);
	wm8350_set_bits(lcd_backlight.wm8350, WM8350_DCDC_LDO_REQUESTED, 1 << (WM8350_DCDC_5 - WM8350_DCDC_1));

	if (level <= 0x28) {
		powering_down = 1;
	} else {
		powering_down = 0;
	}

	mutex_unlock(&led->mutex);
}

static void wm8350_brightness_set(struct led_classdev *led_cdev, enum led_brightness value)
{
	struct _wm8350_led *led = container_of(led_cdev, struct _wm8350_led, led_dev);
	unsigned long flags;
	
	spin_lock_irqsave(&led->value_lock, flags);
	led->backlight_brightness = value;
	schedule_work(&led->work);
	spin_unlock_irqrestore(&led->value_lock, flags);
}

static enum led_brightness wm8350_brightness_get(struct led_classdev *led_cdev)
{
	unsigned long flags;
	enum led_brightness value;
	struct _wm8350_led* led = container_of(led_cdev, struct _wm8350_led, led_dev);	

	spin_lock_irqsave(&led->value_lock, flags);
	// A little bit ugly but no point in reading the WM8350 to get this value
	value = led->backlight_brightness;
	spin_unlock_irqrestore(&led->value_lock, flags);
	return value;
}

static int wm8350_bl_probe(struct platform_device *pdev)
{
	u16 val;
	struct wm8350 *wm8350 = platform_get_drvdata(pdev);

	if(!wm8350) {
		printk(KERN_ERR "No driver data supplied\n");
		return -ENODEV;
	} 
	
	lcd_backlight.wm8350 = wm8350;
	lcd_backlight.led_dev.name = "lcd-backlight";
	lcd_backlight.led_dev.brightness = WM8350_DEFAULT_BACKLIGHT_BRIGHTNESS;
	lcd_backlight.led_dev.brightness_set = wm8350_brightness_set;
	lcd_backlight.led_dev.brightness_get = wm8350_brightness_get;

	INIT_WORK(&lcd_backlight.work, wm8350_work);
	mutex_init(&lcd_backlight.mutex);
	spin_lock_init(&lcd_backlight.value_lock);

	wm8350_isink_set_flash(wm8350, WM8350_ISINK_A, 0, 0, 0, 0, 0, WM8350_CS1_DRIVE);
	wm8350_dcdc25_set_mode(wm8350, WM8350_DCDC_5, WM8350_DC5_MODE_BOOST, 
				WM8350_DC5_ILIM_HIGH, WM8350_DC5_RMP_20V, WM8350_DC5_FBSRC_FB2);
	wm8350_dcdc_set_slot(wm8350, WM8350_DCDC_5, 0xf, 0xf,  WM8350_DC5_ERRACT_NONE);
		
	val = wm8350_reg_read(wm8350, WM8350_CURRENT_SINK_DRIVER_A) & ~WM8350_CS1_ISEL_MASK;
	wm8350_reg_write(wm8350, WM8350_CURRENT_SINK_DRIVER_A, val | 0x39);
	wm8350_set_bits(wm8350, WM8350_POWER_MGMT_7, WM8350_CS1_ENA);
	wm8350_set_bits(wm8350, WM8350_CSA_FLASH_CONTROL, WM8350_CS1_DRIVE);
	wm8350_set_bits(wm8350, WM8350_DCDC_LDO_REQUESTED, 1 << (WM8350_DCDC_5 - WM8350_DCDC_1));

	return led_classdev_register(&pdev->dev, &lcd_backlight.led_dev);
}

static int wm8350_bl_remove(struct platform_device *pdev)
{
	led_classdev_unregister(&lcd_backlight.led_dev);
	return 0;
}

#ifdef CONFIG_PM
static int wm8350_bl_suspend(struct platform_device *pdev, pm_message_t state)
{
	printk("wm8350_bl_suspend\n");
	led_classdev_suspend(&lcd_backlight.led_dev);
	return 0;
}

static int wm8350_bl_resume(struct platform_device *dev)
{
	printk("wm8350_bl_resume\n");
	led_classdev_resume(&lcd_backlight.led_dev);
	return 0;
}
#else
#define wm8350_bl_suspend	NULL
#define wm8350_bl_resume	NULL
#endif

static struct platform_driver wm8350_bl_driver = {
	.probe		= wm8350_bl_probe,
	.remove		= wm8350_bl_remove,
	.suspend	= wm8350_bl_suspend,
	.resume		= wm8350_bl_resume,
	.driver		= {
		.name	= "wm8350-lcd",
	},
};

static int __init wm8350_bl_init(void)
{
	printk("Wolfson WM8350 LCD Backlight Device Driver (c) 2009 Intrinsyc Software \n");

	platform_driver_register(&wm8350_bl_driver);
	return 0;
}

static void __exit wm8350_bl_exit(void)
{
 	platform_driver_unregister(&wm8350_bl_driver);
}

module_init(wm8350_bl_init);
module_exit(wm8350_bl_exit);

MODULE_AUTHOR("David Bolcsfoldi <dbolcsfoldi@intrinsyc.com>");
MODULE_DESCRIPTION("Wolfson WM8350 LCD Backlight Driver");
MODULE_LICENSE("GPL");

void s3cfb_init_hw(void)
{
	printk(KERN_INFO "LCD TYPE :: AT272 will be initialized\n");

	s3cfb_set_fimd_info();
	s3cfb_set_gpio();
}

