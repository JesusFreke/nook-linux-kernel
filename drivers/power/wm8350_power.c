/*
 * Battery driver for wm8350 PMIC
 *
 * Copyright 2007, 2008 Wolfson Microelectronics PLC.
 *
 * Based on OLPC Battery Driver
 *
 * Copyright 2006  David Woodhouse <dwmw2@infradead.org>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * Modifications 2009 : Intrinsyc Software, Inc on behalf of Barnes and Noble
 * Portions of this code copyright (c) 2009 Barnes and Noble, Inc
 *
 */
#include <linux/module.h>
#include <linux/err.h>
#include <linux/kernel.h>
#include <plat/regs-clock.h>

#include <linux/platform_device.h>
#include <linux/power_supply.h>
#include <linux/mfd/wm8350/supply.h>
#include <linux/mfd/wm8350/core.h>
#include <linux/mfd/wm8350/comparator.h>
#include <mach/bravo_gpio.h>
#include <plat/regs-usb-otg-hs.h>
#include <asm/io.h>
#include <asm/atomic.h>
#include <linux/vmalloc.h>
#include <linux/fcntl.h>
#include <linux/file.h>
#include <linux/fs.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/workqueue.h>
#include <linux/pm.h>

#define GPIO_CHARGECHK  S3C64XX_GPM(0)
#define GPIO_CHARGE_EN  S3C64XX_GPN(2)

#define WM8350_CHARGE_REQUEST_ON   (true)
#define WM8350_CHARGE_REQUEST_OFF  (false)

#define WM8350_LED_BLINK (2)
#define WM8350_LED_ON    (1)
#define WM8350_LED_OFF   (0)

static void wm8350_led_ctrl(struct wm8350 *wm8350, int led_state);
static void wm8350_temperature_monitor_thread(struct work_struct *para);
static int wm8350_battery_charger_enabled(struct wm8350* wm8350);

extern void s3c_otg_soft_connect(int connect);
static int charger_dump=0;
static int blink_on=0;

static int read_sysfs_int(struct wm8350 *wm8350, const char* file, int *pval)
{
    struct file *fp=NULL;
    int ret = 0;
    long filesize;
    char *dp;
    loff_t pos;
    
    if (pval == NULL)
        return -1;
    
    fp = filp_open(file, O_RDONLY| O_LARGEFILE, 0);
    if (IS_ERR(fp))    {
        dev_err(wm8350->dev,"%s: filp_open(%s) failed\n", __FUNCTION__, file);
        return -1;
    }
    else
    {
        filesize = fp->f_mapping->host->i_size;
        if (filesize <= 0 ) {
            dev_err(wm8350->dev,"Invalid file: %s\n", file);
            filp_close(fp, NULL);
            return filesize;
        }
        dp = vmalloc(filesize);
        if (dp == NULL) {
            dev_err(wm8350->dev,"Out of memory.\n");
            filp_close(fp, NULL);            
            return -1;
        }
        pos = 0;
        ret = vfs_read(fp, dp, filesize, &pos);
        if (ret > 0)
        {
            int val = 0;

            dp[ret] = 0;
            val = simple_strtol(dp, NULL, 10);
            *pval = val;
        }
        vfree(dp);
        filp_close(fp, NULL);
    }
    return ret;
}

static int wm8350_get_battery_temperature(struct wm8350 *wm8350, int *pval)
{
    int ret = 0;

    ret = read_sysfs_int(wm8350, "/sys/class/power_supply/bq27200-0/temp", pval);

    return ret;
}

static int wm8350_get_battery_capacity(struct wm8350 *wm8350, int *pval)
{
    int ret = 0;
    
    ret = read_sysfs_int(wm8350, "/sys/class/power_supply/bq27200-0/capacity", pval);

    return ret;
}

static int wm8350_get_battery_current(struct wm8350 *wm8350, int *pval)
{
    int ret = 0;

    ret = read_sysfs_int(wm8350, "/sys/class/power_supply/bq27200-0/current_now", pval);

    return ret;
}

static int wm8350_get_battery_voltage(struct wm8350 *wm8350, int *pval)
{
    int ret = 0;

    ret = read_sysfs_int(wm8350, "/sys/class/power_supply/bq27200-0/voltage_now", pval);

    return ret;
}

#define BATTERY_UNKNOWN 0
#define BATTERY_INVALID 1
#define BATTERY_VALID   3

static int wm8350_battery_status(struct wm8350 *wm8350)
{
    struct file *fp=NULL;
    int ret = 0;
    long len;
    char *dp;
    loff_t pos;

    fp = filp_open("/sys/class/power_supply/bq27200-0/manufacturer", O_RDONLY| O_LARGEFILE, 0);
    if (IS_ERR(fp)) {
        dev_err(wm8350->dev,"%s: OPEN manufacturer file failed\n", __FUNCTION__);
        return BATTERY_UNKNOWN;
    }
    else
    {
        len = fp->f_mapping->host->i_size;
        if (len <= 0 ) {
            dev_err(wm8350->dev,"Invalid file \n");
            filp_close(fp, NULL);
            return len;
        }
        dp = vmalloc(len);
        if (dp == NULL) {
            dev_err(wm8350->dev,"Out of memory loading.\n");
        	filp_close(fp, NULL);            
            return -1;
        }
        pos = 0;
        if (vfs_read(fp, dp, len, &pos))
        {
            dev_dbg(wm8350->dev, "wm8350: manufacturer :%c%c%c%c\n",dp[0],dp[1],dp[2],dp[3]);
            if (!strncmp(dp, "LICO", 4))
                ret = BATTERY_VALID;
            else if (!strncmp(dp, "Unknown", 7))
                ret = BATTERY_INVALID;
            else
                ret = BATTERY_UNKNOWN;
        }
        vfree(dp);
        filp_close(fp, NULL);
    }
    return ret;
}

static void wm8350_request_charge_pause(struct wm8350 *wm8350, bool pause_on)
{
    wm8350_reg_unlock(wm8350);

    if (pause_on)
    {
        wm8350_set_bits(wm8350, WM8350_BATTERY_CHARGER_CONTROL_2, WM8350_CHG_PAUSE);
    }
    else
    {
        wm8350_clear_bits(wm8350, WM8350_BATTERY_CHARGER_CONTROL_2, WM8350_CHG_PAUSE);
    }

    wm8350_reg_lock(wm8350);
}

static void wm8350_charging_start(struct wm8350 *wm8350)
{
    u16 bcc1_reg_val= 0x0;
 
    wm8350_reg_unlock(wm8350);
    bcc1_reg_val = wm8350_reg_read(wm8350, WM8350_BATTERY_CHARGER_CONTROL_1);
    bcc1_reg_val |= WM8350_CHG_ENA_R168;
    wm8350_reg_write(wm8350, WM8350_BATTERY_CHARGER_CONTROL_1,  bcc1_reg_val);
    wm8350->charging = 1;
    wm8350_reg_lock(wm8350);
}

static void wm8350_charging_stop(struct wm8350 *wm8350)
{
    u16 bcc1_reg_val= 0x0;

    wm8350_reg_unlock(wm8350);
    bcc1_reg_val = wm8350_reg_read(wm8350, WM8350_BATTERY_CHARGER_CONTROL_1);
    bcc1_reg_val &= ~(WM8350_CHG_ENA_R168);
    wm8350_reg_write(wm8350, WM8350_BATTERY_CHARGER_CONTROL_1,  bcc1_reg_val);
    wm8350->charging = 0;
    wm8350_led_ctrl(wm8350,WM8350_LED_OFF);
    wm8350_reg_lock(wm8350);
}

static int wm8350_read_battery_uvolts(struct wm8350 *wm8350)
{
    return wm8350_read_auxadc(wm8350, WM8350_AUXADC_BATT, 0, 0) * WM8350_AUX_COEFF;
}

static int wm8350_read_line_uvolts(struct wm8350 *wm8350)
{
    return wm8350_read_auxadc(wm8350, WM8350_AUXADC_LINE, 0, 0) * WM8350_AUX_COEFF;
}

static int wm8350_read_usb_uvolts(struct wm8350 *wm8350)
{
    return wm8350_read_auxadc(wm8350, WM8350_AUXADC_USB, 0, 0) * WM8350_AUX_COEFF;
}

#ifndef CONFIG_USBHOST_CLOCK_EPLL
#define WM8350_BATT_SUPPLY  1
#define WM8350_USB_SUPPLY   2
#define WM8350_LINE_SUPPLY  4
#endif
static inline int wm8350_charge_time_min(struct wm8350 *wm8350, int min)
{
    if (!wm8350->power.rev_g_coeff)
    {
        return (((min - 30) / 15) & 0xf) << 8;
    }
    else
    {
        return (((min - 30) / 30) & 0xf) << 8;
    }
}

static int prev_detected_supply = -1;
#ifdef CONFIG_USBHOST_CLOCK_EPLL
int last_detected_supply = WM8350_INIT_UNKNOWN_SUPPLY;
EXPORT_SYMBOL_GPL(last_detected_supply);
#else
static int last_detected_supply = WM8350_BATT_SUPPLY;
#endif
static int wm8350_get_supplies(struct wm8350 *wm8350)
{
    return last_detected_supply;
}

static void wm8350_request_charge_status(struct wm8350 *wm8350, bool turn_charge_on)
{
    wm8350_reg_unlock(wm8350);

    if (turn_charge_on)
    {
        wm8350_set_bits(wm8350, WM8350_POWER_MGMT_5, WM8350_CHG_ENA);
    }
    else
    {
        wm8350_clear_bits(wm8350, WM8350_POWER_MGMT_5, WM8350_CHG_ENA);
    }

    wm8350_reg_lock(wm8350);
}

static int wm8350_charger_config(struct wm8350 *wm8350, struct wm8350_charger_policy *policy)
{
    u16 bcc1_reg_val      = 0x0000;
    u16 bcc2_reg_val      = 0x0000;
    u16 eoc_mA            = 0;
    u16 fast_limit_mA     = 0;
    u16 trickle_charge_mA = 0;
    if (NULL == policy)
    {
        dev_err(wm8350->dev, "wm8350_charger_config() - ERROR: No charger policy, charger not configured.\n");
        return -EINVAL;
    }

    /* make sure USB fast charge current is not > 500mA */
    if (policy->fast_limit_USB_mA > 500)
    {
        dev_err(wm8350->dev, "wm8350_charger_config() - ERROR: USB fast charge > 500mA\n");
        return -EINVAL;
    }

    eoc_mA = WM8350_CHG_EOC_mA(policy->eoc_mA);

    wm8350_reg_unlock(wm8350);

    /* Change the Battery Charger Control 1 Register fields as follows:
    *   - CHG_ENA:                Preserve original value
    *   - CHG_EOC_SEL:            Set to eoc_mA
    *   - CHG_TRICKLE_TEMP_CHOKE: Enable
    *   - CHG_TRICKLE_USB_CHOKE:  Enable
    *   - CHG_RECOVER_T:          180 us
    *   - CHG_END_ACT:            Set current to 0 when charging ends
    *   - CHG_FAST:               Disable
    *   - CHG_FAST_USB_THROTTLE:  Enable
    *   - CHG_NTC_MON:            Disable
    *   - CHG_BATT_HOT_MON:       Disable
    *   - CHG_BATT_COLD_MON:      Disable
    *   - CHG_CHIP_TEMP_MON:      Disable
    */
    bcc1_reg_val = wm8350_reg_read(wm8350, WM8350_BATTERY_CHARGER_CONTROL_1); // & WM8350_CHG_ENA_R168;

    if (wm8350_battery_status(wm8350) == BATTERY_VALID)
    {
        // Start charging
        bcc1_reg_val |= WM8350_CHG_ENA_R168;
    }
    else
    {
        // Stop charging
        bcc1_reg_val &= ~(WM8350_CHG_ENA_R168);
    }

    bcc1_reg_val |= (
            eoc_mA                        |
            WM8350_CHG_TRICKLE_3_9V       |
            WM8350_CHG_TRICKLE_TEMP_CHOKE |
            WM8350_CHG_TRICKLE_USB_CHOKE  |
            WM8350_CHG_FAST_USB_THROTTLE  |
            WM8350_CHG_CHIP_TEMP_MON);

    wm8350_reg_write(wm8350, WM8350_BATTERY_CHARGER_CONTROL_1,  bcc1_reg_val);

    if (wm8350_get_supplies(wm8350) & WM8350_LINE_SUPPLY)
    {
        dev_dbg(wm8350->dev, "wm8350_charger_config() - Configuring for Wall Charging\n");
        fast_limit_mA     = WM8350_CHG_FAST_LIMIT_mA(policy->fast_limit_mA);
        trickle_charge_mA = policy->trickle_charge_mA;
    }
    else
    {
        dev_dbg(wm8350->dev, "wm8350_charger_config() - Configuring for USB Charging\n");
        fast_limit_mA     = WM8350_CHG_FAST_LIMIT_mA(policy->fast_limit_USB_mA);
        trickle_charge_mA = policy->trickle_charge_USB_mA;
    }

    /* Change the Battery Charger Control 2 Register fields as follows:
    *   - CHG_ACTIVE:       Preserve original value
    *   - CHG_PAUS:         Set to DON'T PAUSE
    *   - CHG_STS:          Preserve original value
    *   - CHG_TIME:         Set to value from Charging Policy
    *   - CHG_MASK_WALL_FB: Set to DO NOT MASK
    *   - CHG_TRICKLE_SEL:  Set to value from Charging Policy
    *   - CHG_VSEL:         Set to value from Charging Policy
    *   - CHG_ISEL:         Set to value from Charging Policy
    */
    bcc2_reg_val = wm8350_reg_read(wm8350, WM8350_BATTERY_CHARGER_CONTROL_2);

    bcc2_reg_val &= (WM8350_CHG_ACTIVE | WM8350_CHG_STS_MASK);

    bcc2_reg_val = (bcc2_reg_val      |
            policy->charge_mV |
            trickle_charge_mA |
            fast_limit_mA     |
            wm8350_charge_time_min(wm8350, policy->charge_timeout));

    wm8350_reg_write(wm8350, WM8350_BATTERY_CHARGER_CONTROL_2, bcc2_reg_val);

    /* Do not change the Battery Charger Control 3 Register fields. */
    wm8350_reg_lock(wm8350);
    return 0;
}

/** 
Read the Battery charger control 1 register to determine if charging has 
been enabled or not.
@return 1 if charging enabled, 0 otherwise.
*/
static int wm8350_battery_charger_enabled(struct wm8350* wm8350)
{
    u16 charge_ctl1 = wm8350_reg_read(wm8350, WM8350_BATTERY_CHARGER_CONTROL_1);
    int charge_enabled = (charge_ctl1 & WM8350_CHG_ACTIVE);    
    printk("charger control 1 = %d\n", charge_enabled);
    return charge_enabled;
}

static void wm8350_led_ctrl(struct wm8350 *wm8350, int led_state)
{
    u16 reg;
    wm8350_reg_unlock(wm8350);
    if(led_state == WM8350_LED_ON)
    {
        wm8350->led_is_on = 1;
        reg = wm8350_reg_read(wm8350, WM8350_POWER_MGMT_7);
        wm8350_reg_write(wm8350, WM8350_POWER_MGMT_7, reg | WM8350_CS2_ENA);

    	// set current sink to 8 mA
        wm8350_reg_write(wm8350, WM8350_CURRENT_SINK_DRIVER_B, 0x802d);

        reg = wm8350_reg_read(wm8350, WM8350_CSB_FLASH_CONTROL);
        reg |= (0x1<<13);
        reg &= ~(0x1<<15); // set to LED mode
        wm8350_reg_write(wm8350, WM8350_CSB_FLASH_CONTROL,reg);
    }
    else if (led_state == WM8350_LED_OFF)
    {
        wm8350->led_is_on = 0;
        reg = wm8350_reg_read(wm8350, WM8350_POWER_MGMT_7);
        wm8350_reg_write(wm8350, WM8350_POWER_MGMT_7, reg & ~(WM8350_CS2_ENA));

        reg = wm8350_reg_read(wm8350, WM8350_CSB_FLASH_CONTROL);
        reg &= ~(0x1<<13);
        reg |= (0x1<<15); // set it to LED mode
        wm8350_reg_write(wm8350, WM8350_CSB_FLASH_CONTROL,reg);
    }
    else if (led_state == WM8350_LED_BLINK)
    {
        if (wm8350->led_is_on == 0)
        {
            reg = wm8350_reg_read(wm8350, WM8350_POWER_MGMT_7);
            wm8350_reg_write(wm8350, WM8350_POWER_MGMT_7, reg & ~( WM8350_CS2_ENA));

            // set current sink to 8 mA
            wm8350_reg_write(wm8350, WM8350_CURRENT_SINK_DRIVER_B, 0x802d);

            reg = wm8350_reg_read(wm8350, WM8350_CSB_FLASH_CONTROL);
            reg |= (0x1<<15); // set it to FLASH mode
            wm8350_reg_write(wm8350, WM8350_CSB_FLASH_CONTROL, reg);

            reg = wm8350_reg_read(wm8350, WM8350_POWER_MGMT_7);
            wm8350_reg_write(wm8350, WM8350_POWER_MGMT_7, reg | WM8350_CS2_ENA);

            reg = (1 << 15) | // flash mode
                  (0 << 14) | // CSn_DRIVE triggered
                  (1 << 13) | // CSn_DRIVE bit (pulse now)
                  (1 << 12) | // once per trigger
                  (0 << 8)  | // 32ms pulse 
                  (0 << 4)  | // no off ramp
                  (0 << 0);   // no on ramp 

            wm8350_reg_write(wm8350, WM8350_CSB_FLASH_CONTROL, reg);
         }
     }
     else
     {
        printk("invalid led_state %d for led_ctrl - do nothing\n", led_state);
     }

     wm8350_reg_lock(wm8350);
}

static int wm8350_batt_status(struct wm8350 *wm8350)
{
    u16 state;

    state = wm8350_reg_read(wm8350, WM8350_BATTERY_CHARGER_CONTROL_2);
    state &= WM8350_CHG_STS_MASK;

    switch (state)
    {
        case WM8350_CHG_STS_OFF:
            return POWER_SUPPLY_STATUS_DISCHARGING;

        case WM8350_CHG_STS_TRICKLE:
        case WM8350_CHG_STS_FAST:
            return POWER_SUPPLY_STATUS_CHARGING;

        default:
            return POWER_SUPPLY_STATUS_UNKNOWN;
    }
}

static ssize_t wm8350_charger_state_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    struct wm8350 *wm8350 = dev_get_drvdata(dev);
    char *charge;
    int state;

    state = wm8350_reg_read(wm8350, WM8350_BATTERY_CHARGER_CONTROL_2) & WM8350_CHG_STS_MASK;

    switch (state)
    {
        case WM8350_CHG_STS_OFF:
            charge = "Charger Off";
            break;

        case WM8350_CHG_STS_TRICKLE:
            charge = "Trickle Charging";
            break;

        case WM8350_CHG_STS_FAST:
            charge = "Fast Charging";
            break;

        default:
            return 0;
    }

    return sprintf(buf, "%s\n", charge);
}
static ssize_t wm8350_blink_state_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    return sprintf(buf, "%d\n", blink_on);
}

static ssize_t wm8350_charger_state_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
    charger_dump = simple_strtol(buf, NULL, 10);
    return count;
}

static ssize_t wm8350_blink_state_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
    if (1 == simple_strtol(buf, NULL, 10))
        blink_on=1;
    else
        blink_on=0;
    return count;
}

static DEVICE_ATTR(charger_state, 0666, wm8350_charger_state_show, wm8350_charger_state_store);
static DEVICE_ATTR(blink_state, 0666, wm8350_blink_state_show, wm8350_blink_state_store);

static void wm8350_configure_charging_ntc_override(struct wm8350 *wm8350)
{
    wm8350_reg_write(wm8350, WM8350_SECURITY, 0x1375);
    wm8350_reg_write(wm8350, 0xDE,            0x2864);
    wm8350_reg_write(wm8350, WM8350_SECURITY, 0x00A7);
    wm8350_reg_write(wm8350, 0xDE,            0x0013);
    wm8350_reg_write(wm8350, WM8350_SECURITY, 0x0013);
#if 0
    if (last_detected_supply == WM8350_USB_SUPPLY)
    {
        wm8350_reg_write(wm8350, 0xA8,            0x2041);
        wm8350_reg_write(wm8350, 0xA9,            0x0B3F);
    }
    else
    {
        wm8350_reg_write(wm8350, 0xA8,            0x2041);
        wm8350_reg_write(wm8350, 0xA9,            0x0F3F);
    }
#else
    wm8350_reg_write(wm8350, 0xA8,            0x2041);
    wm8350_reg_write(wm8350, 0xA9,            0x0B30);
    wm8350_reg_write(wm8350, 0xA8,            0x2061);
    wm8350_reg_write(wm8350, 0xA9,            0x0B3F);
#endif
}

#ifdef CONFIG_USBHOST_CLOCK_EPLL
extern void otg_phy_off(void);
extern int s3c_udc_resume(struct platform_device *pdev);
extern struct platform_device s3c_device_usbgadget;
extern int s3c_udc_suspend(struct platform_device *pdev, pm_message_t state);
#endif
extern void s3c_otg_show_soft_connect(void);

void wm8350_cable_detect(struct wm8350 *wm8350)
{
    struct wm8350_power          *power  = &(wm8350->power);

    int vbatt, vusb;
    volatile unsigned long regval = 0x0;

    vbatt = wm8350_read_battery_uvolts(wm8350);
    vusb = wm8350_read_usb_uvolts(wm8350);

    if (vusb < 4400000)
    {
    	mdelay(2);
    	printk("%s: first read vUSB=%d\n", __FUNCTION__,vusb);
    	vusb = wm8350_read_usb_uvolts(wm8350);	
    }
    
    s3c_otg_show_soft_connect();

    printk("%s: vUSB=%d, vBATT=%d, Prev Supply=%d\n", __FUNCTION__,vusb, vbatt, last_detected_supply);
    #ifndef CONFIG_USBHOST_CLOCK_EPLL
    // Disconnect OTG
    s3c_otg_soft_connect(0);
    last_detected_supply = WM8350_BATT_SUPPLY;
    #endif
    cancel_delayed_work(&wm8350->monitor_work);
    // Check if we have anyting plugged in
    if (vusb < 4400000)
    {
        printk(KERN_NOTICE "wm8350_charger_handler() - Power Supply = Battery\n");
        /* Disable CHARGECHK */
        s3c_gpdat_setval(GPIO_CHARGECHK, 1);
	    /* Configure Charge_EN for USB Charging */
	    s3c_gpio_setpull(GPIO_CHARGE_EN, S3C_GPIO_PULL_DOWN);
	    s3c_gpdat_setval(GPIO_CHARGE_EN, 0);

#ifdef CONFIG_USBHOST_CLOCK_EPLL
        regval = get_s3c_reg_val(S3C_HCLK_GATE);
        if (regval & S3C_CLKCON_HCLK_USB)//OTG has been powered up
        {
			printk(KERN_NOTICE "WM8350_CABLE_DETECT: unplug USB cable, turn off OTG block\n");
			s3c_otg_soft_connect(0);
			udelay(100);
			s3c_udc_suspend(&s3c_device_usbgadget, PMSG_SUSPEND);
			mdelay(1);
        }
#endif
		last_detected_supply = WM8350_BATT_SUPPLY;
		//make sure LED is off when no cable plugged in
		wm8350_led_ctrl(wm8350,WM8350_LED_OFF);
    }
    else
    {
        // It must be a USB or WALL charger
        volatile int usb_status = 0;
        int usb_dp     = 0;
        int usb_dm     = 0;

#ifdef CONFIG_USBHOST_CLOCK_EPLL
		if ( (last_detected_supply > WM8350_BATT_SUPPLY) && (last_detected_supply < WM8350_INIT_UNKNOWN_SUPPLY) )
		{
			printk(KERN_NOTICE "wm8350_cable_detect() -- nothing has been changed(%d)\n", last_detected_supply);
			return;
		}
	
		printk(KERN_NOTICE "poweron OTG\n");
	
		if ( (last_detected_supply != WM8350_INIT_UNKNOWN_SUPPLY) )
		{
			mdelay(1);
			s3c_udc_resume(&s3c_device_usbgadget);
		}
		else {
			s3c_otg_soft_connect(0);
		}
		
#else
		// Disconnect OTG
		s3c_otg_soft_connect(0);
		last_detected_supply = WM8350_BATT_SUPPLY;
#endif
	
		/* Enable CHARGECHK */
		s3c_gpdat_setval(GPIO_CHARGECHK, 0);
		udelay(500);
		
		usb_status = readl(S3C_UDC_OTG_HPRT);
		usb_dp = ((usb_status & (0x1 << 11)) >> 11);
		usb_dm = ((usb_status & (0x1 << 10)) >> 10);
		printk("%s: dp=%d, dm=%d, HPRT(0x%x)\n", __FUNCTION__, usb_dp, usb_dm, usb_status);
		
		/* Disable CHARGECHK */
		s3c_gpdat_setval(GPIO_CHARGECHK, 1);
		udelay(100);
	
		if ((1 == usb_dp) && (1 == usb_dm))
		{
			printk(KERN_NOTICE "wm8350_cable_detect() - Power Supply = Wall Charger\n");

#ifdef CONFIG_USBHOST_CLOCK_EPLL
			s3c_udc_suspend(&s3c_device_usbgadget, PMSG_SUSPEND);
			mdelay(1);
#endif
			/* Configure Charge_EN for Wall Charging */
			s3c_gpio_setpull(GPIO_CHARGE_EN, S3C_GPIO_PULL_UP);
			s3c_gpdat_setval(GPIO_CHARGE_EN, 1);
			last_detected_supply = WM8350_LINE_SUPPLY;
		}
		else if ((0 == usb_dp) && (0 == usb_dm))
		{
			printk(KERN_NOTICE "wm8350_cable_detect() - Power Supply = USB Host\n");

			// Connect OTG Again
			s3c_otg_soft_connect(1);
#ifdef CONFIG_USBHOST_CLOCK_EPLL
			udelay(100);
#endif
			/* Configure Charge_EN for USB Charging */
			s3c_gpio_setpull(GPIO_CHARGE_EN, S3C_GPIO_PULL_DOWN);
			s3c_gpdat_setval(GPIO_CHARGE_EN, 0);
			last_detected_supply = WM8350_USB_SUPPLY;
		}
		else
		{
#ifdef CONFIG_USBHOST_CLOCK_EPLL
			s3c_udc_suspend(&s3c_device_usbgadget, PMSG_SUSPEND);
			mdelay(1);
#endif
			/* Configure Charge_EN for USB Charging */
			s3c_gpio_setpull(GPIO_CHARGE_EN, S3C_GPIO_PULL_UP);
			s3c_gpdat_setval(GPIO_CHARGE_EN, 1);
			last_detected_supply = WM8350_LINE_SUPPLY;
			printk(KERN_NOTICE "wm8350_cable_detect() - Power Supply = USB Charger!\n");
			dev_dbg(wm8350->dev, "wm8350_charger_handler() - USB D+ is %s\n", (1 == usb_dp) ? "HIGH" : "LOW");
			dev_dbg(wm8350->dev, "wm8350_charger_handler() - USB D- is %s\n", (1 == usb_dm) ? "HIGH" : "LOW");
		}
#if 0
		/* Compensate for the lack of NTC Hardware */
		wm8350_configure_charging_ntc_override(wm8350);
	
		/* Configure the Battery Charger Status registers */
		wm8350_charger_config(wm8350, policy);
#endif
    }
    printk("%s: New Supply = %d\n", __FUNCTION__, last_detected_supply);

    schedule_delayed_work(&wm8350->monitor_work, 0);
    power_supply_changed(&power->battery);
    power_supply_changed(&power->usb);
    power_supply_changed(&power->ac);
}

void wm8350_cable_detect_thread(struct work_struct *para)
{
    struct wm8350 *wm8350 = container_of(para,struct wm8350, detect_work.work);

    wm8350_cable_detect(wm8350);
}

/*********************************************************************
 *      Handle Interrupts
 *********************************************************************/
#ifdef CONFIG_BRAVO_LOWBATT_WAKEUP
static int g_pccmp_off_thr_sleepmode = 0x0;  //In sleep mode, it is used to indicate that shutdown_threshold is 2.9V
int g_pccmp_off_thr = 0x4; //In active mode, shutdown_threshold is 3.3V
EXPORT_SYMBOL_GPL(g_pccmp_off_thr);

extern void battery_set_low_thr(int );
extern void battery_low_immediate_report(void);

static void wm8350_lowbattery_handler(struct wm8350 *wm8350, int irq, void *data)
{
#if (0) //If SYS_HYST_COMP_FAIL_EINT is  needed in system active mode
	//Clear SYS_HYST_COMP_FAIL_EINT bit from InterruptStatus2 register
	u16 regval = 0x0;
	int pccmp_off_thr = 0x0;

	if (g_pccmp_off_thr_sleepmode)
	{
		printk(KERN_NOTICE "wm8350_lowbattery_handler: PCCMP_OFF_THR(%d->%d)\n", g_pccmp_off_thr_sleepmode, g_pccmp_off_thr);
		battery_set_low_thr(g_pccmp_off_thr_sleepmode); //2900 ~ 3600
		g_pccmp_off_thr_sleepmode = 0x0;
	}
	else
	{
		regval = wm8350_reg_read(wm8350, WM8350_POWER_CHECK_COMPARATOR);
		pccmp_off_thr = (regval & WM8350_PCCMP_OFF_THR_MASK)>>4;
		battery_set_low_thr(pccmp_off_thr);
	}
#endif
		
	//Susan -- We are safe to trigger schedule_work for bq27200_battery_monitor_status(), because this work always gets done in keventd worker thread
	//wm8350_mask_irq(wm8350, WM8350_IRQ_SYS_HYST_COMP_FAIL);
	printk(KERN_NOTICE "wm8350_lowbattery_handler: battery_low_immediate_report\n");
	battery_low_immediate_report();
}

static void wm8350_resume_lowbattery_setting(struct wm8350 *wm8350)
{
	u16 regval = 0x0;

	//Susan -- g_pccmp_off_thr_sleepmode is only accessed at two places: 1. driver's resume kernel path 2.keventd worker thread
	//So accessing g_pccmp_off_thr_sleepmode is safe, shouldn't have race condition here
	regval = wm8350_reg_read(wm8350, WM8350_POWER_CHECK_COMPARATOR);
	g_pccmp_off_thr_sleepmode = (regval & WM8350_PCCMP_OFF_THR_MASK)>>4;

	regval &= ~WM8350_PCCMP_OFF_THR_MASK;
	regval |= g_pccmp_off_thr<<4;
	wm8350_reg_unlock(wm8350);
	wm8350_reg_write(wm8350, WM8350_POWER_CHECK_COMPARATOR, regval);
	wm8350_reg_read(wm8350, WM8350_POWER_CHECK_COMPARATOR);
	wm8350_reg_lock(wm8350);

	//Susan -- We are safe to trigger schedule_work for bq27200_battery_monitor_status(), because this work always gets done in keventd worker thread
	printk(KERN_NOTICE "WM8350 RESUME PCCMP_OFF_THR(%d)\n", g_pccmp_off_thr);
}
#endif

static void wm8350_charger_handler(struct wm8350 *wm8350, int irq, void *data)
{
    struct wm8350_power          *power  = &(wm8350->power);
    struct wm8350_charger_policy *policy = power->policy;

    int charge_ctl1 = wm8350_reg_read(wm8350, WM8350_BATTERY_CHARGER_CONTROL_1);

    printk("%s Start --> irq = %d\n", __FUNCTION__, irq);
 
    if (charger_dump > 1)
    {
        int battery_status = wm8350_battery_status(wm8350);

        if (battery_status == BATTERY_VALID)
        {
            int I = 0,V = 0;
            int vbatt, vusb;
            int charge_ctl2 = wm8350_reg_read(wm8350, WM8350_BATTERY_CHARGER_CONTROL_2);
            int sm   = wm8350_reg_read(wm8350, WM8350_STATE_MACHINE_STATUS);
    
            vbatt = wm8350_read_battery_uvolts(wm8350);
            vusb = wm8350_read_usb_uvolts(wm8350);
    
            
            wm8350_get_battery_current(wm8350, &I);
            wm8350_get_battery_voltage(wm8350, &V);
    
            printk("%s: C1=%04X C2=%04X SM=%04X BI=%8d, BV=%6d, PVB=%8d, PVU=%8d\n",
                   __FUNCTION__,
                   charge_ctl1,
                   charge_ctl2,
                   sm,
                   I,
                   V,
                   vbatt,
                   vusb);
        }
        else
        {
            printk("%s: Battery Status = %d\n",
                   __FUNCTION__,
                   battery_status);
        }
    }

    switch (irq)
    {
        case WM8350_IRQ_CHG_BAT_FAIL:
            dev_err(wm8350->dev, "wm8350_charger_handler() - ERROR: Battery failed\n");
            break;

        case WM8350_IRQ_CHG_TO:
            dev_warn(wm8350->dev, "wm8350_charger_handler() - WARNING: Charger Timeout\n");
            power_supply_changed(&power->battery);
            //Turn off the LED when charging is completed
            printk("IRQ_CHG_TO, turn off LED\n");
            //wm8350_led_ctrl(wm8350,WM8350_LED_OFF);
            wm8350_charging_stop(wm8350);
    	    printk("%s: WM8350_POWER_STOP_CHARGING due to IRQ_CHG_TO.\n", __FUNCTION__);
    	    wm8350_charging_start(wm8350); //be ready for next time charge condition is meet            
            break;

        case WM8350_IRQ_CHG_BAT_HOT:
        case WM8350_IRQ_CHG_BAT_COLD:
            dev_dbg(wm8350->dev, "wm8350_charger_handler() - Charger Battery is Too %s!\n", (WM8350_IRQ_CHG_BAT_HOT == irq) ? "Hot" : "Cold");
            power_supply_changed(&power->battery);
            break;

        case WM8350_IRQ_CHG_START:
            dev_dbg(wm8350->dev, "wm8350_charger_handler() - Charger Start\n");

            // This interrupt can be generated even though it isn't charging.
            // Ensure charging has been enabled before turning on the LED.
            if (wm8350_battery_charger_enabled(wm8350))
            {
                wm8350_led_ctrl(wm8350,WM8350_LED_ON);
            }
            power_supply_changed(&power->battery);
            break;

        case WM8350_IRQ_CHG_END:
            dev_dbg(wm8350->dev, "wm8350_charger_handler() - Charger End\n");
            //Turn off the LED when charging is completed
            wm8350_led_ctrl(wm8350,WM8350_LED_OFF);
            //wm8350_reg_unlock(wm8350);
            //wm8350_clear_bits(wm8350, WM8350_BATTERY_CHARGER_CONTROL_1, WM8350_CHG_FAST);
            //wm8350_reg_lock(wm8350);
            //wm8350_charging_stop(wm8350);
    	    printk("%s: WM8350_POWER_STOP_CHARGING due to IRQ_CHG_END, turn off LED.\n", __FUNCTION__);
    	    //wm8350_charging_start(wm8350); //be ready for next time charge condition is meet
            power_supply_changed(&power->battery);
            break;

        case WM8350_IRQ_CHG_FAST_RDY:
            dev_dbg(wm8350->dev, "wm8350_charger_handler() - Fast Charger ready\n");
            wm8350_charger_config(wm8350, policy);
            wm8350_reg_unlock(wm8350);
            wm8350_set_bits(wm8350, WM8350_BATTERY_CHARGER_CONTROL_1, WM8350_CHG_FAST);
            wm8350_reg_lock(wm8350);
            // Turn on the LED when fast charging starts as long as charging 
            // has been enabled.
            if (wm8350_battery_charger_enabled(wm8350))
            {
                wm8350_led_ctrl(wm8350,WM8350_LED_ON);
            }
            break;

        case WM8350_IRQ_CHG_VBATT_LT_3P9:
            dev_warn(wm8350->dev, "wm8350_charger_handler() - battery < 3.9V\n");
            break;

        case WM8350_IRQ_CHG_VBATT_LT_3P1:
            dev_warn(wm8350->dev, "wm8350_charger_handler() - battery < 3.1V\n");
            break;

        case WM8350_IRQ_CHG_VBATT_LT_2P85:
            dev_warn(wm8350->dev, "wm8350_charger_handler() - battery < 2.85V\n");
            break;

#if 0
        case WM8350_IRQ_EXT_WALL_FB:
        case WM8350_IRQ_EXT_BAT_FB:
#endif
        /* Supply change.  We will overnotify but it should do no harm. */
        case WM8350_IRQ_EXT_USB_FB:
            cancel_delayed_work(&wm8350->detect_work);
            wm8350_cable_detect(wm8350);
            break;

        default:
            dev_err(wm8350->dev, "wm8350_charger_handler() - ERROR: Unknown interrupt %d\n", irq);
    }
}

/*********************************************************************
 *      AC Power
 *********************************************************************/
static int wm8350_ac_get_prop(struct power_supply *psy,
                  enum power_supply_property psp,
                  union power_supply_propval *val)
{
    struct wm8350 *wm8350 = dev_get_drvdata(psy->dev->parent);
    int ret = 0;

    switch (psp)
    {
        case POWER_SUPPLY_PROP_ONLINE:
            val->intval = !!(wm8350_get_supplies(wm8350) & WM8350_LINE_SUPPLY);
            break;

        case POWER_SUPPLY_PROP_VOLTAGE_NOW:
            val->intval = wm8350_read_line_uvolts(wm8350);
            break;

        default:
            ret = -EINVAL;
            break;
    }

    return ret;
}

static enum power_supply_property wm8350_ac_props[] = {
    POWER_SUPPLY_PROP_ONLINE,
    POWER_SUPPLY_PROP_VOLTAGE_NOW,
};

/*********************************************************************
 *      USB Power
 *********************************************************************/
static int wm8350_usb_get_prop(struct power_supply *psy, enum power_supply_property psp, union power_supply_propval *val)
{
    struct wm8350 *wm8350 = dev_get_drvdata(psy->dev->parent);
    int ret = 0;

    switch (psp)
    {
        case POWER_SUPPLY_PROP_ONLINE:
            val->intval = !!(wm8350_get_supplies(wm8350) & WM8350_USB_SUPPLY);
            break;

        case POWER_SUPPLY_PROP_VOLTAGE_NOW:
            val->intval = wm8350_read_usb_uvolts(wm8350);
            break;

        default:
            ret = -EINVAL;
            break;
    }

    return ret;
}

static enum power_supply_property wm8350_usb_props[] = {
    POWER_SUPPLY_PROP_ONLINE,
    POWER_SUPPLY_PROP_VOLTAGE_NOW,
};

/*********************************************************************
 *      Battery properties
 *********************************************************************/
#ifndef CONFIG_MACH_BRAVO
static int wm8350_bat_check_health(struct wm8350 *wm8350)
{
    u16 reg;

    if (wm8350_read_battery_uvolts(wm8350) < 2850000)
        return POWER_SUPPLY_HEALTH_UNSPEC_FAILURE;

    reg = wm8350_reg_read(wm8350, WM8350_CHARGER_OVERRIDES);
    if (reg & WM8350_CHG_BATT_HOT_OVRDE)
        return POWER_SUPPLY_HEALTH_OVERHEAT;

    if (reg & WM8350_CHG_BATT_COLD_OVRDE)
        return POWER_SUPPLY_HEALTH_GOOD;

    return POWER_SUPPLY_HEALTH_GOOD;
}
#endif /* !CONFIG_MACH_BRAVO */

static int wm8350_bat_get_property(struct power_supply *psy,
                   enum power_supply_property psp,
                   union power_supply_propval *val)
{
#ifndef CONFIG_MACH_BRAVO
    struct wm8350 *wm8350 = dev_get_drvdata(psy->dev->parent);
#endif /* !CONFIG_MACH_BRAVO */
    int ret = 0;

    switch (psp)
    {
        case POWER_SUPPLY_PROP_STATUS:
#ifdef CONFIG_MACH_BRAVO
            val->intval = POWER_SUPPLY_STATUS_CHARGING;
#else  /* CONFIG_MACH_BRAVO */
            val->intval = wm8350_batt_status(wm8350);
#endif /* CONFIG_MACH_BRAVO */
            break;

        case POWER_SUPPLY_PROP_ONLINE:
#ifdef CONFIG_MACH_BRAVO
            val->intval = 1;
#else  /* CONFIG_MACH_BRAVO */
            val->intval = !!(wm8350_get_supplies(wm8350) & WM8350_BATT_SUPPLY);
#endif /* CONFIG_MACH_BRAVO */
            break;

        case POWER_SUPPLY_PROP_VOLTAGE_NOW:
#ifdef CONFIG_MACH_BRAVO
            val->intval = 420000;
#else  /* CONFIG_MACH_BRAVO */
            val->intval = wm8350_read_battery_uvolts(wm8350);
#endif /* CONFIG_MACH_BRAVO */
            break;

        case POWER_SUPPLY_PROP_HEALTH:
#ifdef CONFIG_MACH_BRAVO
            val->intval = POWER_SUPPLY_HEALTH_GOOD;
#else  /* CONFIG_MACH_BRAVO */
            val->intval = wm8350_bat_check_health(wm8350);
#endif /* CONFIG_MACH_BRAVO */
            break;

        default:
            ret = -EINVAL;
            break;
    }

    return ret;
}

static enum power_supply_property wm8350_bat_props[] = {
    POWER_SUPPLY_PROP_STATUS,
    POWER_SUPPLY_PROP_ONLINE,
    POWER_SUPPLY_PROP_VOLTAGE_NOW,
    POWER_SUPPLY_PROP_HEALTH,
};

static void wm8350_led_blink_thread(struct work_struct * para)
{
    struct wm8350 *wm8350 = container_of(para,struct wm8350, blink_work.work);

    schedule_delayed_work(&wm8350->blink_work, 4*HZ);
    if (blink_on==1)
    {
        if (wm8350->led_is_on == 0)
            wm8350_led_ctrl(wm8350,WM8350_LED_BLINK);
    }
}

static void wm8350_temperature_monitor_thread(struct work_struct *para)
{
    struct wm8350 *wm8350 = container_of(para,struct wm8350, monitor_work.work);
    struct wm8350_power          *power  = &(wm8350->power);
    struct wm8350_charger_policy *policy = power->policy;
    
    int degc = 0;
    int capacity = 0;

    int battery_status = wm8350_battery_status(wm8350);
    int charge_ctl1 = wm8350_reg_read(wm8350, WM8350_BATTERY_CHARGER_CONTROL_1);
    int charge_requested = (charge_ctl1 & WM8350_CHG_ACTIVE);
    int charge_ctl2 = wm8350_reg_read(wm8350, WM8350_BATTERY_CHARGER_CONTROL_2);
    int charge_state = (charge_ctl2 & WM8350_CHG_STS_MASK);
    int I = 0,V = 0;

    static bool temperature_limit_triggered = false;
    const int temperature_max = 45;
    const int temperature_max_threshold = temperature_max-2;
    const int temperature_min = 0;                // stop charging if we are <= this temp
    const int temperature_min_threshold = 3;      // re-start charging if > this temp

    dev_dbg(wm8350->dev, "wm8350_temperature_monitor_thread\n");
    schedule_delayed_work(&wm8350->monitor_work, 20*HZ);

    wm8350_get_battery_current(wm8350, &I);
    wm8350_get_battery_voltage(wm8350, &V);

    //if current into battery < 20mA and battery voltage > 4.1v, then stop charging.
    if ( (I > -20000) && ( I < 0 ) && (V > 4100) )
    {   
        if (charge_requested)
        {
            wm8350_charging_stop(wm8350);
    	    printk("%s: WM8350_POWER_STOP_CHARGING due to low charging current.\n", __FUNCTION__);
    	    wm8350_charging_start(wm8350); //be ready for next time charge condition is meet
        }
    }
    
    if (charger_dump)
    {
        if (battery_status == BATTERY_VALID)
        {
            //int I = 0,V = 0;
            int vbatt, vusb;
            int charge_ctl2 = wm8350_reg_read(wm8350, WM8350_BATTERY_CHARGER_CONTROL_2);
            int sm   = wm8350_reg_read(wm8350, WM8350_STATE_MACHINE_STATUS);
    
            vbatt = wm8350_read_battery_uvolts(wm8350);
            vusb = wm8350_read_usb_uvolts(wm8350);
    
            
            //wm8350_get_battery_current(wm8350, &I);
            //wm8350_get_battery_voltage(wm8350, &V);
    
            printk("%s: C1=%04X C2=%04X SM=%04X BI=%8d, BV=%6d, PVB=%8d, PVU=%8d\n",
            __FUNCTION__,
            charge_ctl1,
            charge_ctl2,
            sm,
            I,
            V,
            vbatt,
            vusb);
        }
        else
        {
            printk("%s: Battery Status = %d\n",
                __FUNCTION__,
                battery_status);
        }
    }
    
    if (last_detected_supply == WM8350_BATT_SUPPLY)
        {
        wm8350_charging_stop(wm8350);
        cancel_delayed_work(&wm8350->monitor_work);
        prev_detected_supply = last_detected_supply;
        printk("%s: WM8350_POWER_STOP_CHARGING\n", __FUNCTION__);
        return;
    }
    
    if (battery_status != BATTERY_VALID)
    {
        if (charge_requested)
        {
            wm8350_charging_stop(wm8350);
            printk("%s: WM8350_POWER_STOP_CHARGING\n", __FUNCTION__);
        }
        return;
    }
    
    if (prev_detected_supply != last_detected_supply)
    {
        wm8350_led_ctrl(wm8350,WM8350_LED_ON);

        /* Compensate for the lack of NTC Hardware */
        wm8350_configure_charging_ntc_override(wm8350);

        /* Configure the Battery Charger Status registers */
        wm8350_charger_config(wm8350, policy);

        prev_detected_supply = last_detected_supply;
    }

    //cable plugged in, check if vol<=3.9v, if so, make sure LED is on, charging may start soon
    if ( V < 4000 )
    {
    	printk("battery less than 4.0v, turn LED on\n");
        wm8350_led_ctrl(wm8350,WM8350_LED_ON);
    }

    wm8350_get_battery_capacity(wm8350, &capacity);
    if ( capacity == 100 )
    {
	printk("capacity = 100,  turn LED OFF\n");
        wm8350_led_ctrl(wm8350,WM8350_LED_OFF);
    }
    else
    {
	//printk("capacity = %d \n", capacity);
    }

    wm8350_get_battery_temperature(wm8350, &degc);
    if (charger_dump)
    {
        printk("%s: Temerature = %d\n", __FUNCTION__, degc);
    }

    if (temperature_limit_triggered)
    {
        if ( (degc >= temperature_max_threshold) || (degc <= temperature_min_threshold) )
        {
            // turn off charging
            printk("%s: Battery temperature still out of safe range = %d\n", __FUNCTION__, degc);
            return;
        }
        printk("%s: Temerature normal again = %d\n", __FUNCTION__, degc);
        wm8350_request_charge_pause(wm8350, false);
        temperature_limit_triggered = false;
    }
    if ((degc >= temperature_max) || (degc <= temperature_min))
    {
        // turn off charging
        temperature_limit_triggered = true;
        wm8350_request_charge_pause(wm8350, true);
        printk("%s: Battery temerature out of safe range = %d\n", __FUNCTION__, degc);
        return;
    }

    // We should be charging at this point
    if (!charge_requested)
    {
        wm8350_charging_start(wm8350);
        printk("%s: WM8350_POWER_START_CHARGING\n", __FUNCTION__);
    }
}

/*********************************************************************
 *      Initialisation
 *********************************************************************/
#ifdef CONFIG_BRAVO_LOWBATT_WAKEUP
static void wm8350_init_hyst_irq(struct wm8350 *wm8350) //WM8350_IRQ_SYS_HYST_COMP_FAIL
{
	u16 regval = wm8350_reg_read(wm8350, WM8350_POWER_CHECK_COMPARATOR);
	regval &= ~(WM8350_PCCMP_OFF_THR_MASK|WM8350_PCCMP_ON_THR_MASK);
	regval |= (g_pccmp_off_thr<<4)|(g_pccmp_off_thr+1);  //Make sure PCCMP_OFF_THR is lower than PCCMP_ON_THR
	wm8350_reg_unlock(wm8350);
	wm8350_reg_write(wm8350, WM8350_POWER_CHECK_COMPARATOR, regval);
	wm8350_reg_lock(wm8350);
	
	wm8350_register_irq(wm8350, WM8350_IRQ_SYS_HYST_COMP_FAIL, wm8350_lowbattery_handler, NULL);
	//At this moment, bq27200 is not initialized yet -- Clear IM_HYST bit in suspend path
    wm8350_unmask_irq(wm8350, WM8350_IRQ_SYS_HYST_COMP_FAIL);
}
#endif

static void wm8350_init_charger_irq(struct wm8350 *wm8350)
{
    /* register our interest in charger events... */

    wm8350_register_irq(wm8350, WM8350_IRQ_CHG_BAT_HOT, wm8350_charger_handler, NULL);
    wm8350_unmask_irq(wm8350, WM8350_IRQ_CHG_BAT_HOT);

    wm8350_register_irq(wm8350, WM8350_IRQ_CHG_BAT_COLD, wm8350_charger_handler, NULL);
    wm8350_unmask_irq(wm8350, WM8350_IRQ_CHG_BAT_COLD);

    wm8350_register_irq(wm8350, WM8350_IRQ_CHG_BAT_FAIL, wm8350_charger_handler, NULL);
    wm8350_unmask_irq(wm8350, WM8350_IRQ_CHG_BAT_FAIL);

    wm8350_register_irq(wm8350, WM8350_IRQ_CHG_TO, wm8350_charger_handler, NULL);
    wm8350_unmask_irq(wm8350, WM8350_IRQ_CHG_TO);

    wm8350_register_irq(wm8350, WM8350_IRQ_CHG_END, wm8350_charger_handler, NULL);
    wm8350_unmask_irq(wm8350, WM8350_IRQ_CHG_END);

    wm8350_register_irq(wm8350, WM8350_IRQ_CHG_START, wm8350_charger_handler, NULL);
    wm8350_unmask_irq(wm8350, WM8350_IRQ_CHG_START);

    wm8350_register_irq(wm8350, WM8350_IRQ_CHG_FAST_RDY, wm8350_charger_handler, NULL);
    wm8350_unmask_irq(wm8350, WM8350_IRQ_CHG_FAST_RDY);

    wm8350_register_irq(wm8350, WM8350_IRQ_CHG_VBATT_LT_3P9, wm8350_charger_handler, NULL);
    wm8350_unmask_irq(wm8350, WM8350_IRQ_CHG_VBATT_LT_3P9);

    wm8350_register_irq(wm8350, WM8350_IRQ_CHG_VBATT_LT_3P1, wm8350_charger_handler, NULL);
    wm8350_unmask_irq(wm8350, WM8350_IRQ_CHG_VBATT_LT_3P1);

    wm8350_register_irq(wm8350, WM8350_IRQ_CHG_VBATT_LT_2P85, wm8350_charger_handler, NULL);
    wm8350_unmask_irq(wm8350, WM8350_IRQ_CHG_VBATT_LT_2P85);

    /* ...and supply change events */

    wm8350_register_irq(wm8350, WM8350_IRQ_EXT_USB_FB, wm8350_charger_handler, NULL);
    wm8350_unmask_irq(wm8350, WM8350_IRQ_EXT_USB_FB);

#if 0
    wm8350_register_irq(wm8350, WM8350_IRQ_EXT_WALL_FB, wm8350_charger_handler, NULL);
    wm8350_unmask_irq(wm8350, WM8350_IRQ_EXT_WALL_FB);

    wm8350_register_irq(wm8350, WM8350_IRQ_EXT_BAT_FB, wm8350_charger_handler, NULL);
    wm8350_unmask_irq(wm8350, WM8350_IRQ_EXT_BAT_FB);
#endif
}
#ifdef CONFIG_BRAVO_LOWBATT_WAKEUP
static void wm8350_free_hyst_irq(struct wm8350 *wm8350) //WM8350_IRQ_SYS_HYST_COMP_FAIL
{
    wm8350_mask_irq(wm8350, WM8350_IRQ_SYS_HYST_COMP_FAIL);
    wm8350_free_irq(wm8350, WM8350_IRQ_SYS_HYST_COMP_FAIL);
}
#endif

static void wm8350_free_charger_irq(struct wm8350 *wm8350)
{
    wm8350_mask_irq(wm8350, WM8350_IRQ_CHG_BAT_HOT);
    wm8350_free_irq(wm8350, WM8350_IRQ_CHG_BAT_HOT);

    wm8350_mask_irq(wm8350, WM8350_IRQ_CHG_BAT_COLD);
    wm8350_free_irq(wm8350, WM8350_IRQ_CHG_BAT_COLD);

    wm8350_mask_irq(wm8350, WM8350_IRQ_CHG_BAT_FAIL);
    wm8350_free_irq(wm8350, WM8350_IRQ_CHG_BAT_FAIL);

    wm8350_mask_irq(wm8350, WM8350_IRQ_CHG_TO);
    wm8350_free_irq(wm8350, WM8350_IRQ_CHG_TO);

    wm8350_mask_irq(wm8350, WM8350_IRQ_CHG_END);
    wm8350_free_irq(wm8350, WM8350_IRQ_CHG_END);

    wm8350_mask_irq(wm8350, WM8350_IRQ_CHG_START);
    wm8350_free_irq(wm8350, WM8350_IRQ_CHG_START);

    wm8350_mask_irq(wm8350, WM8350_IRQ_CHG_VBATT_LT_3P9);
    wm8350_free_irq(wm8350, WM8350_IRQ_CHG_VBATT_LT_3P9);

    wm8350_mask_irq(wm8350, WM8350_IRQ_CHG_VBATT_LT_3P1);
    wm8350_free_irq(wm8350, WM8350_IRQ_CHG_VBATT_LT_3P1);

    wm8350_mask_irq(wm8350, WM8350_IRQ_CHG_VBATT_LT_2P85);
    wm8350_free_irq(wm8350, WM8350_IRQ_CHG_VBATT_LT_2P85);

    wm8350_mask_irq(wm8350, WM8350_IRQ_EXT_USB_FB);
    wm8350_free_irq(wm8350, WM8350_IRQ_EXT_USB_FB);

#if 0
    wm8350_mask_irq(wm8350, WM8350_IRQ_EXT_WALL_FB);
    wm8350_free_irq(wm8350, WM8350_IRQ_EXT_WALL_FB);

    wm8350_mask_irq(wm8350, WM8350_IRQ_EXT_BAT_FB);
    wm8350_free_irq(wm8350, WM8350_IRQ_EXT_BAT_FB);
#endif
}

static void wm8350_allocate_charger_policy(struct wm8350_charger_policy **policy_ptr)
{
    struct wm8350_charger_policy *policy = NULL;

    policy = kmalloc(sizeof(struct wm8350_charger_policy), GFP_KERNEL);

    if (NULL == policy)
    {
        printk(KERN_ERR "wm8350_allocate_charger_policy() - ERROR: Could not allocate memory for Charger Policy.\n");
    }
    else
    {
        /* Note: We expect policy->fast_limit_mA and policy->fast_limit_USB_mA
         *       to be set to a value between 0 and 750 mA.
         *       These values will be converted via macro before use.
         *       We also expect policy->eoc_mA to be set to a value between
         *       20 and 90 mA; it too will be converted via macro before use.
         *       The value for policy->charge_timeout should range between
         *       60 min and 510 min, and should also be a multiple of 30.
         *       This value will be converted via wm8350_charge_time_min()
         *       before use.
         */
        policy->eoc_mA                = 30;
        policy->charge_mV             = WM8350_CHG_4_20V;
        policy->fast_limit_mA         = 750;
        policy->fast_limit_USB_mA     = 400;
        policy->charge_timeout        = 480;
        policy->trickle_start_mV      = WM8350_CHG_TRICKLE_3_1V;
        policy->trickle_charge_mA     = WM8350_CHG_TRICKLE_50mA;
        policy->trickle_charge_USB_mA = WM8350_CHG_TRICKLE_50mA;

        *policy_ptr = policy;
    }
}

static void wm8350_free_charger_policy(struct wm8350_charger_policy **policy_ptr)
{
    if (NULL != *policy_ptr)
    {
        kfree(*policy_ptr);
        *policy_ptr = NULL;
    }
}

static __devinit int wm8350_power_probe(struct platform_device *pdev)
{
    struct wm8350 *wm8350 = platform_get_drvdata(pdev);
    struct wm8350_power *power = &(wm8350->power);
    struct power_supply *usb = &power->usb;
    struct power_supply *battery = &power->battery;
    struct power_supply *ac = &power->ac;
    int ret;
    printk("%s: Start\n", __FUNCTION__);

#ifdef CONFIG_MACH_BRAVO
    ac->name = "ac";
#else
    ac->name = "wm8350-ac";
#endif
    ac->type           = POWER_SUPPLY_TYPE_MAINS;
    ac->properties     = wm8350_ac_props;
    ac->num_properties = ARRAY_SIZE(wm8350_ac_props);
    ac->get_property   = wm8350_ac_get_prop;
    ret = power_supply_register(&pdev->dev, ac);
    if (ret)
    {
        dev_err(wm8350->dev, "wm8350_power_probe() - ERROR: Could not register %s AC Power Driver\n", ac->name);
        return ret;
    }

#ifdef CONFIG_MACH_BRAVO
    battery->name = "battery";
#else
    battery->name = "wm8350-battery";
#endif
    battery->properties     = wm8350_bat_props;
    battery->num_properties = ARRAY_SIZE(wm8350_bat_props);
    battery->get_property   = wm8350_bat_get_property;
    battery->use_for_apm    = 1;
    ret = power_supply_register(&pdev->dev, battery);
    if (ret)
    {
        dev_err(wm8350->dev, "wm8350_power_probe() - ERROR: Could not register %s Battery Power Driver\n", battery->name);
        goto battery_failed;
    }

#ifdef CONFIG_MACH_BRAVO
    usb->name = "usb";
#else
    usb->name = "wm8350-usb";
#endif
    usb->type           = POWER_SUPPLY_TYPE_USB;
    usb->properties     = wm8350_usb_props;
    usb->num_properties = ARRAY_SIZE(wm8350_usb_props);
    usb->get_property   = wm8350_usb_get_prop;
    ret = power_supply_register(&pdev->dev, usb);
    if (ret)
    {
        dev_err(wm8350->dev, "wm8350_power_probe() - ERROR: Could not register %s USB Power Driver\n", usb->name);
        goto usb_failed;
    }

    ret = device_create_file(&pdev->dev, &dev_attr_charger_state);
    if (ret < 0)
    {
        dev_warn(wm8350->dev, "wm8350_power_probe() - WARNING: failed to add charge sysfs: %d\n", ret);
    }

    ret = device_create_file(&pdev->dev, &dev_attr_blink_state);
    if (ret < 0)
    {
        dev_warn(wm8350->dev, "wm8350_power_probe() - WARNING: failed to add blink sysfs: %d\n", ret);
    }


    ret = 0;

#ifdef CONFIG_BRAVO_LOWBATT_WAKEUP
	wm8350_init_hyst_irq(wm8350);
#endif

    wm8350_allocate_charger_policy(&(wm8350->power.policy));
    wm8350_init_charger_irq(wm8350);

    /* Configure the CHARGECHK GPIO Signal */
    s3c_gpio_cfgpin(GPIO_CHARGECHK, S3C_GPIO_OUTPUT);
    s3c_gpio_setpull(GPIO_CHARGECHK, S3C_GPIO_PULL_NONE);
    s3c_gpdat_setval(GPIO_CHARGECHK, 1);

    /* Configure the Charge_EN GPIO Signal */
    s3c_gpio_cfgpin(GPIO_CHARGE_EN, S3C_GPIO_OUTPUT);
    s3c_gpio_setpull(GPIO_CHARGE_EN, S3C_GPIO_PULL_NONE);
    s3c_gpdat_setval(GPIO_CHARGE_EN, 0);

    wm8350_request_charge_status(wm8350, WM8350_CHARGE_REQUEST_OFF);
    //if (0 == wm8350_charger_config(wm8350, wm8350->power.policy))
    {
        /* Make sure the charging subsystem is enabled.
         * Note that we could have also activated this through the
         * Battery Charger Control 1 register.
         * Also note that this setting means that we *can* charge.
         * The CHG_ACTIVE bit in the Battery Charger Control 2
         * register indicates whether we *are* charging.
         */
        //wm8350_request_charge_status(wm8350, WM8350_CHARGE_REQUEST_ON);
    }
            
    wm8350_set_bits(wm8350, WM8350_INTERFACE_CONTROL, WM8350_CONFIG_DONE);

    INIT_DELAYED_WORK(&wm8350->detect_work, wm8350_cable_detect_thread);
    INIT_DELAYED_WORK(&wm8350->monitor_work, wm8350_temperature_monitor_thread);
    INIT_DELAYED_WORK(&wm8350->blink_work, wm8350_led_blink_thread);

    //schedule_delayed_work(&wm8350->detect_work, 0);
	wm8350_cable_detect(wm8350);
    schedule_delayed_work(&wm8350->blink_work, 5*HZ);

    printk("%s: Done\n", __FUNCTION__);

    return ret;

usb_failed:
    power_supply_unregister(battery);
battery_failed:
    power_supply_unregister(ac);

    return ret;
}

static __devexit int wm8350_power_remove(struct platform_device *pdev)
{
    struct wm8350 *wm8350 = platform_get_drvdata(pdev);
    struct wm8350_power *power = &wm8350->power;
    struct wm8350_charger_policy *policy = power->policy;

    wm8350_free_charger_policy(&policy);
    wm8350_free_charger_irq(wm8350);
#ifdef CONFIG_BRAVO_LOWBATT_WAKEUP
    wm8350_free_hyst_irq(wm8350);
#endif

    cancel_delayed_work(&wm8350->blink_work);
    device_remove_file(&pdev->dev, &dev_attr_charger_state);
    device_remove_file(&pdev->dev, &dev_attr_blink_state);

    power_supply_unregister(&power->battery);
    power_supply_unregister(&power->ac);
    power_supply_unregister(&power->usb);

    return 0;
}

#ifdef CONFIG_PM
#include <plat/pm.h>

static int wm8350_suspend(struct platform_device *pdev, pm_message_t state)
{
	struct wm8350 *wm8350 = platform_get_drvdata(pdev);
    printk("wm8350_suspend START\n");

    // Plugging in the USB cable is supposed to wake the device from suspend.
    // However, if it's preparing to suspend there is a race condition where
    // the cable could be in and the device goes to sleep. This is because the
    // IRQ handler schedules interrupt processing in a work queue and it's 
    // possible for the suspend handler to pre-empt IRQ processing. Aborting 
    // suspend if we're charging is a last attempt to detect the USB cable. If
    // it's plugged in after this then too bad.
    int status = wm8350_batt_status(wm8350);
    if (status != POWER_SUPPLY_STATUS_DISCHARGING || wm8350->charging)
    {
        printk("Device is charging - cancelling suspend\n");
        return -1;
    }

    // Prevent IRQ processing until resume - it's potentially unsafe.
    atomic_set(&wm8350->suspend_on, 1);

    // Ensure we aren't charging before suspend.
    wm8350_charging_stop(wm8350);

#ifdef CONFIG_BRAVO_LOWBATT_WAKEUP
	volatile unsigned short regval = 0x0;
	//Susan for Critical Power wakeup
	//Note that SYS_HYST_COMP_FAIL_EINT _ONLY_ gets triggered when line voltage falls _through_ PCCMP_OFF_THR
	//If line voltage is already lower than PCCMP_OFF_THR, SYS_HYST_COMP_FAIL_EINT won't gets generated again when
	//line voltage keeps lowering down
	//Re-configure PCCMP_OFF_THR as 3.3V for critical battery wakeup (need to go through application)
	regval = wm8350_reg_read(wm8350, WM8350_POWER_CHECK_COMPARATOR);
	regval &= ~WM8350_PCCMP_OFF_THR_MASK;
	regval |= g_pccmp_off_thr << 4; //3.3V as PCCMP_OFF_THR
	wm8350_reg_unlock(wm8350);
	wm8350_reg_write(wm8350, WM8350_POWER_CHECK_COMPARATOR, regval);
	wm8350_reg_lock(wm8350);

	//Enable SYS_HYST_COMP_FAIL_EINT
	regval = wm8350_reg_read(wm8350, WM8350_INT_STATUS_2_MASK);
	regval &= ~WM8350_IM_SYS_HYST_COMP_FAIL_EINT;
	wm8350_reg_write(wm8350, WM8350_INT_STATUS_2_MASK, regval);

	//Clear SYS_HYST_COMP_FAIL_EINT bit in 0x1Ah register
	regval = wm8350_reg_read(wm8350, WM8350_INT_STATUS_2);
	//Clear SYS_INT bit in 0x18h register
	regval = wm8350_reg_read(wm8350, WM8350_SYSTEM_INTERRUPTS);

#endif
	wm8350_save_wake_source(wm8350);
    printk("wm8350_suspend END\n");
    return 0;
}

static int wm8350_resume(struct platform_device *pdev)
{
    struct wm8350 *wm8350 = platform_get_drvdata(pdev);

    // Allow IRQ processing now that we're not suspended.
    atomic_set(&wm8350->suspend_on, 0);

    disable_irq(wm8350->chip_irq);
#ifdef CONFIG_BRAVO_LOWBATT_WAKEUP
	if (0)  //If SYS_HYST_COMP_FAIL_EINT is needed in system active mode, PCCMP_OFF_THR has different settings between system Active mode and system Sleep mode
		wm8350_resume_lowbattery_setting(wm8350);
	else
		; //wm8350_mask_irq(wm8350, WM8350_IRQ_SYS_HYST_COMP_FAIL);
#endif
    schedule_delayed_work(&wm8350->detect_work, 3*HZ);
    schedule_work(&(wm8350->irq_work));
    return 0;
}

#else
#define wm8350_suspend NULL
#define wm8350_resume  NULL
#endif /* CONFIG_PM */

static struct platform_driver wm8350_power_driver = {
    .probe = wm8350_power_probe,
    .remove = __devexit_p(wm8350_power_remove),
    .driver = {
        .name = "wm8350-power",
    },
    .suspend =  wm8350_suspend,
    .resume =   wm8350_resume,
};

static int __init wm8350_power_init(void)
{
    return platform_driver_register(&wm8350_power_driver);
}
module_init(wm8350_power_init);

static void __exit wm8350_power_exit(void)
{
    platform_driver_unregister(&wm8350_power_driver);
}
module_exit(wm8350_power_exit);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Power supply driver for WM8350");
MODULE_ALIAS("platform:wm8350-power");
