/* linux/drivers/input/touchscreen/s3c-ts.c
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
 *
 * Copyright (c) 2004 Arnaud Patard <arnaud.patard@rtp-net.org>
 * iPAQ H1940 touchscreen support
 *
 * ChangeLog
 *
 * 2004-09-05: Herbert Potzl <herbert@13thfloor.at>
 *  - added clock (de-)allocation code
 *
 * 2005-03-06: Arnaud Patard <arnaud.patard@rtp-net.org>
 *      - h1940_ -> s3c24xx (this driver is now also used on the n30
 *        machines :P)
 *      - Debug messages are now enabled with the config option
 *        TOUCHSCREEN_S3C_DEBUG
 *      - Changed the way the value are read
 *      - Input subsystem should now work
 *      - Use ioremap and readl/writel
 *
 * 2005-03-23: Arnaud Patard <arnaud.patard@rtp-net.org>
 *      - Make use of some undocumented features of the touchscreen
 *        controller
 *
 * 2006-09-05: Ryu Euiyoul <ryu.real@gmail.com>
 *      - added power management suspend and resume code
 *
 */

#include <linux/errno.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/input.h>
#include <linux/init.h>
#include <linux/serio.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/clk.h>

#include <asm/io.h>
#include <asm/irq.h>
#include <mach/hardware.h>

#include <plat/regs-adc.h>
#include <plat/ts.h>
#include <mach/irqs.h>

#include <asm/gpio.h>
#include <plat/gpio-cfg.h>
#include <plat/regs-gpio.h>
#include <plat/gpio-bank-h.h>
#include <plat/gpio-bank-c.h>

#include <mach/hardware.h>
#include <mach/audio.h>
#include <asm/io.h>

#include "bravo_buttons.h"

#define CONFIG_TOUCHSCREEN_S3C_DEBUG
#undef CONFIG_TOUCHSCREEN_S3C_DEBUG

/* For ts->dev.id.version */
#define S3C_TSVERSION   0x0101

#define WAIT4INT(x)  (((x)<<8) | \
             S3C_ADCTSC_YM_SEN | S3C_ADCTSC_YP_SEN | S3C_ADCTSC_XP_SEN | \
             S3C_ADCTSC_XY_PST(3))

#define AUTOPST      (S3C_ADCTSC_YM_SEN | S3C_ADCTSC_YP_SEN | S3C_ADCTSC_XP_SEN | \
             S3C_ADCTSC_AUTO_PST | S3C_ADCTSC_XY_PST(0))


#define DEBUG_LVL    KERN_DEBUG

#define LCD_ACTUAL_X  480
#define LCD_ACTUAL_Y  272
#define EINK_ACTUAL_Y 800

/* 
	Need this many samples in a row within the boundaries of a button to qualify as a button press
 */
#define NUM_CONTINUOUS_SAMPLES_FOR_BUTTON_PRESS 20

/* Touchscreen default configuration */
struct s3c_ts_mach_info s3c_ts_default_cfg __initdata = {
    .delay              = 10000,
    .presc              = 49,
    .oversampling_shift = 2,
    .resol_bit          = 10
};

/*
 * Definitions & global arrays.
 */
static char *s3c_ts_name = "S3C TouchScreen";
static void __iomem        *ts_base;
static struct resource     *ts_mem;
static struct resource     *ts_irq;
static struct clk          *ts_clock;
static struct s3c_ts_info  *ts;

static unsigned ts_offset_area = 0;
#ifdef CONFIG_FB_S3C
extern void s3cfb_set_offset(unsigned int offset);
#endif

#ifdef CONFIG_TOUCHSCREEN_BRAVO_HOME_SUPPORT
static int currentButtonID = BRAVO_BUTTON_ID_NONE;

static int getCurrentlyPressedButton(int xCoord, int yCoord)
{
    int buttonID = BRAVO_BUTTON_ID_NONE;

    if ((yCoord >= BRAVO_BUTTON_ROW_TOP) && (yCoord <= BRAVO_BUTTON_ROW_BOTTOM)) {
        /* The user touched home area */
#ifdef CONFIG_TOUCHSCREEN_S3C_DEBUG
        printk(KERN_INFO "Button Press (%d:%d)\n", xCoord, yCoord);
#endif /* CONFIG_TOUCHSCREEN_S3C_DEBUG */
		buttonID = BRAVO_BUTTON_ID_HOME;
    }

    return buttonID;
}

#endif /* CONFIG_TOUCHSCREEN_BRAVO_HOME_SUPPORT */ 

static void touch_timer_fire(unsigned long data)
{
    unsigned long data0;
    unsigned long data1;
    int updown;
#if 1 /* ANDROID */

    int a0,a1,a2,a3,a4,a5,a6;
    int x,y;

#ifdef CONFIG_SMDK6410_REV10

    /* Revised Numbers */
    a0 =    -22235;
    a1 =      -162;
    a2 = 130322459;
    a3 =     -1254;
    a4 =    -17424;
    a5 =  50081898;
    a6 =   -397946;

#if 0
    /* Pieter's Numbers */
    a0 =      -96;
    a1 =    -2243;
    a2 = 34021653;
    a3 =    -2132;
    a4 =       -8;
    a5 = 26521838;
    a6 =    65536;
#endif /* 0 */

#if 0
    /* Original Numbers */
    a0 =      5365;
    a1 =         1;
    a2 = -17910848;
    a3 =        13;
    a4 =      4380;
    a5 = -18677200;
    a6 =     65536;
#endif /* 0 */

#else  /* CONFIG_SMDK6410_REV10 */

    a0 =      5171;
    a1 =         9;
    a2 = -17497920;
    a3 =       -12;
    a4 =     -4659;
    a5 =  52871808;
    a6 =     65536;

#endif /* CONFIG_SMDK6410_REV10 */
#endif /* 1 // ANDROID */

    data0 = readl(ts_base + S3C_ADCDAT0);
    data1 = readl(ts_base + S3C_ADCDAT1);

    updown = (!(data0 & S3C_ADCDAT0_UPDOWN)) && (!(data1 & S3C_ADCDAT1_UPDOWN));

    if (updown)
    {
        if (ts->count)
        {
#if 1 /* ANDROID */
            x = (int) ts->xp;
            y = (int) ts->yp;

#ifdef CONFIG_TOUCHSCREEN_S3C_DEBUG
            {
                struct timeval tv;
                do_gettimeofday(&tv);
                printk(KERN_INFO "R: %06d, X: %03ld, Y: %03ld\n", (int)tv.tv_usec, ts->xp, ts->yp);
            }
#endif /* CONFIG_TOUCHSCREEN_S3C_DEBUG */

            /* Convert the hardware coordinates into screen coordinates.
             */
            ts->xp = (long) ((a2 + (a0 * x) + (a1 * y)) / a6);
            ts->yp = (long) ((a5 + (a3 * x) + (a4 * y)) / a6);

            /* The hardware swaps the x-axis and the y-axis.
             * The Origin is also placed at the bottom-right
             * instead of the top-left where we want it.
             * We thus adjust the data accordingly.
             */
            x = (int) ts->xp;
            y = (int) ts->yp;

            ts->xp = BRAVO_TOUCH_SCREEN_X_MAX - y;
            ts->yp = BRAVO_TOUCH_SCREEN_Y_MAX - x;

            /* Ensure the values we get fit in the specified screen size.
             */
            if (BRAVO_TOUCH_SCREEN_X_MIN > ts->xp) {
                ts->xp = BRAVO_TOUCH_SCREEN_X_MIN;
            }

            if (BRAVO_TOUCH_SCREEN_Y_MIN > ts->yp) {
                ts->yp = BRAVO_TOUCH_SCREEN_Y_MIN;
            }

            if (BRAVO_TOUCH_SCREEN_X_MAX < ts->xp) {
                ts->xp = BRAVO_TOUCH_SCREEN_X_MAX;
            }

            if (BRAVO_TOUCH_SCREEN_Y_MAX < ts->yp) {
                ts->yp = BRAVO_TOUCH_SCREEN_Y_MAX;
            }

#ifdef CONFIG_TOUCHSCREEN_S3C_DEBUG
            {
                struct timeval tv;
                do_gettimeofday(&tv);
                printk(KERN_INFO "T: %06d, X: %03ld, Y: %03ld\n", (int)tv.tv_usec, ts->xp, ts->yp);
            }
#endif /* CONFIG_TOUCHSCREEN_S3C_DEBUG */

            if ((ts->xp != ts->xp_old) || (ts->yp != ts->yp_old)) { 
#ifdef CONFIG_TOUCHSCREEN_BRAVO_HOME_SUPPORT
                /* Determine whether the user touched one of the Home button */

               if (ts_offset_area == 0)
                    currentButtonID = getCurrentlyPressedButton(ts->xp, ts->yp);
                else
                    currentButtonID = BRAVO_BUTTON_ID_NONE;

                if (BRAVO_BUTTON_ID_NONE == currentButtonID) {
                   /* The user is not touching one of the Home Row buttons.
                    * We thus send a BTN_TOUCH event for each sample.
                    */
					input_report_abs(ts->dev, ABS_X, ts->xp);
					input_report_abs(ts->dev, ABS_Y, ts->yp + EINK_ACTUAL_Y - ts_offset_area * LCD_ACTUAL_Y);
					input_report_abs(ts->dev, ABS_Z, 0);
					input_report_key(ts->dev, BTN_TOUCH, 1);
				}
                
				else {
                    input_report_key(ts->dev, currentButtonID, 1);
				}
#else
	            input_report_abs(ts->dev, ABS_X, ts->xp);
                input_report_abs(ts->dev, ABS_Y, ts->yp + EINK_ACTUAL_Y - ts_offset_area * LCD_ACTUAL_Y);
                input_report_abs(ts->dev, ABS_Z, 0);

				input_report_key(ts->dev, BTN_TOUCH, 1);
#endif /* CONFIG_TOUCHSCREEN_BRAVO_HOME_SUPPORT */

                /* input_report_abs(ts->dev, ABS_PRESSURE, 1); */
                input_sync(ts->dev);
            }

            ts->xp_old = ts->xp;
            ts->yp_old = ts->yp + EINK_ACTUAL_Y;

#else  /* 1 // ANDROID */

            input_report_abs(ts->dev, ABS_X, ts->xp);
            input_report_abs(ts->dev, ABS_Y, ts->yp);

            input_report_key(ts->dev, BTN_TOUCH, 1);
            input_report_abs(ts->dev, ABS_PRESSURE, 1);

            input_sync(ts->dev);
#endif /* 1 // ANDROID */
        }

        ts->xp    = 0;
        ts->yp    = 0;
        ts->count = 0;

        writel(S3C_ADCTSC_PULL_UP_DISABLE | AUTOPST, ts_base + S3C_ADCTSC);
        writel(readl(ts_base + S3C_ADCCON) | S3C_ADCCON_ENABLE_START, ts_base + S3C_ADCCON);
    }
    else
    {
        ts->count = 0;
#if 1 // ANDROID
#ifdef CONFIG_TOUCHSCREEN_BRAVO_HOME_SUPPORT
		if (currentButtonID != BRAVO_BUTTON_ID_NONE) {
			/* Button is pressed. Un-press it. */
			input_report_key(ts->dev, currentButtonID, 0);
			currentButtonID = BRAVO_BUTTON_ID_NONE;
		}

		else {
			input_report_abs(ts->dev, ABS_X, ts->xp_old);
			input_report_abs(ts->dev, ABS_Y, ts->yp_old);
	        input_report_abs(ts->dev, ABS_Z, 0);

			input_report_key(ts->dev, BTN_TOUCH, 0);
		}
#else
		input_report_abs(ts->dev, ABS_X, ts->xp_old);
        input_report_abs(ts->dev, ABS_Y, ts->yp_old);
        input_report_abs(ts->dev, ABS_Z, 0);
#endif
#endif // 1 // ANDROID

#ifndef CONFIG_TOUCHSCREEN_BRAVO_HOME_SUPPORT
        input_report_key(ts->dev, BTN_TOUCH, 0);
		printk("Reporting touch up.\n");
#endif

#if 0 // !ANDROID
        input_report_abs(ts->dev, ABS_PRESSURE, 0);
#endif // 0 // !ANDROID

        input_sync(ts->dev);

        writel(WAIT4INT(0), ts_base + S3C_ADCTSC);
    }
}

static struct timer_list touch_timer = TIMER_INITIALIZER(touch_timer_fire, 0, 0);

static irqreturn_t stylus_updown(int irqno, void *param)
{
    unsigned long data0;
    unsigned long data1;
    int updown;

    data0 = readl(ts_base + S3C_ADCDAT0);
    data1 = readl(ts_base + S3C_ADCDAT1);

    updown = (!(data0 & S3C_ADCDAT0_UPDOWN)) && (!(data1 & S3C_ADCDAT1_UPDOWN));

#ifdef CONFIG_TOUCHSCREEN_S3C_DEBUG
    printk(KERN_INFO "   %c\n", updown ? 'D' : 'U');
#endif // CONFIG_TOUCHSCREEN_S3C_DEBUG

    /* TODO we should never get an interrupt with updown set while
     * the timer is running, but maybe we ought to verify that the
     * timer isn't running anyways.
     */

    if (updown)
    {
        touch_timer_fire(0);
    }

    if (ts->s3c_adc_con==ADC_TYPE_2)
    {
        __raw_writel(0x0, ts_base+S3C_ADCCLRWK);
        __raw_writel(0x0, ts_base+S3C_ADCCLRINT);
    }

    return IRQ_HANDLED;
}

static irqreturn_t stylus_action(int irqno, void *param)
{
    unsigned long data0;
    unsigned long data1;

    data0 = readl(ts_base+S3C_ADCDAT0);
    data1 = readl(ts_base+S3C_ADCDAT1);

    if (ts->resol_bit==12)
    {
#ifdef CONFIG_SMDK6410_REV10
        ts->yp += S3C_ADCDAT0_XPDATA_MASK_12BIT - (data0 & S3C_ADCDAT0_XPDATA_MASK_12BIT);
        ts->xp += S3C_ADCDAT1_YPDATA_MASK_12BIT - (data1 & S3C_ADCDAT1_YPDATA_MASK_12BIT);
#else  /* CONFIG_SMDK6410_REV10 */
        ts->xp += data0 & S3C_ADCDAT0_XPDATA_MASK_12BIT;
        ts->yp += data1 & S3C_ADCDAT1_YPDATA_MASK_12BIT;
#endif /* CONFIG_SMDK6410_REV10 */
    }
    else
    {
#ifdef CONFIG_SMDK6410_REV10
        ts->yp += S3C_ADCDAT0_XPDATA_MASK - (data0 & S3C_ADCDAT0_XPDATA_MASK);
        ts->xp += S3C_ADCDAT1_YPDATA_MASK - (data1 & S3C_ADCDAT1_YPDATA_MASK);
#else  /* CONFIG_SMDK6410_REV10 */
        ts->xp += data0 & S3C_ADCDAT0_XPDATA_MASK;
        ts->yp += data1 & S3C_ADCDAT1_YPDATA_MASK;
#endif /* CONFIG_SMDK6410_REV10 */
    }

    ts->count++;

    if (ts->count < (1<<ts->shift))
    {
        writel(S3C_ADCTSC_PULL_UP_DISABLE | AUTOPST, ts_base+S3C_ADCTSC);
        writel(readl(ts_base+S3C_ADCCON) | S3C_ADCCON_ENABLE_START, ts_base+S3C_ADCCON);
    }
    else
    {
        mod_timer(&touch_timer, jiffies+1);
        writel(WAIT4INT(1), ts_base+S3C_ADCTSC);
    }

    if (ts->s3c_adc_con==ADC_TYPE_2)
    {
        __raw_writel(0x0, ts_base+S3C_ADCCLRWK);
        __raw_writel(0x0, ts_base+S3C_ADCCLRINT);
    }

    return IRQ_HANDLED;
}


static struct s3c_ts_mach_info *s3c_ts_get_platdata (struct device *dev)
{
    if (dev->platform_data != NULL)
        return (struct s3c_ts_mach_info *)dev->platform_data;

    return &s3c_ts_default_cfg;
}

static irqreturn_t offset_button_irq(int irq, void *data)
{
    ts_offset_area++;
    if ((ts_offset_area*LCD_ACTUAL_Y) > (EINK_ACTUAL_Y+LCD_ACTUAL_Y))
        ts_offset_area = 0;

#ifdef CONFIG_FB_S3C
    s3cfb_set_offset(ts_offset_area);
#endif /* CONFIG_FB_S3C */

    printk("%s: Touch Offset = %d\n", __FUNCTION__, ts_offset_area);
    return IRQ_HANDLED;
}

/*
 * The functions for inserting/removing us as a module.
 */
static int __init s3c_ts_probe(struct platform_device *pdev)
{
    struct resource *res;
    struct device *dev;
    struct input_dev *input_dev;
    struct s3c_ts_mach_info * s3c_ts_cfg;
    int ret, size;
    int err;

    dev = &pdev->dev;

    res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
    if (res == NULL) {
        dev_err(dev,"no memory resource specified\n");
        return -ENOENT;
    }

    size = (res->end - res->start) + 1;
    ts_mem = request_mem_region(res->start, size, pdev->name);
    if (ts_mem == NULL) {
        dev_err(dev, "failed to get memory region\n");
        ret = -ENOENT;
        goto err_req;
    }

    ts_base = ioremap(res->start, size);
    if (ts_base == NULL) {
        dev_err(dev, "failed to ioremap() region\n");
        ret = -EINVAL;
        goto err_map;
    }

    ts_clock = clk_get(&pdev->dev, "adc");
    if (IS_ERR(ts_clock)) {
        dev_err(dev, "failed to find watchdog clock source\n");
        ret = PTR_ERR(ts_clock);
        goto err_clk;
    }

    clk_enable(ts_clock);

    s3c_ts_cfg = s3c_ts_get_platdata(&pdev->dev);

    if ((s3c_ts_cfg->presc&0xff) > 0)
        writel(S3C_ADCCON_PRSCEN | S3C_ADCCON_PRSCVL(s3c_ts_cfg->presc&0xFF),\
                ts_base+S3C_ADCCON);
    else
        writel(0, ts_base+S3C_ADCCON);


    /* Initialise registers */
    if ((s3c_ts_cfg->delay&0xffff) > 0)
        writel(s3c_ts_cfg->delay & 0xffff, ts_base+S3C_ADCDLY);

    if (s3c_ts_cfg->resol_bit==12) {
        switch(s3c_ts_cfg->s3c_adc_con) {
        case ADC_TYPE_2:
            writel(readl(ts_base+S3C_ADCCON)|S3C_ADCCON_RESSEL_12BIT, ts_base+S3C_ADCCON);
            break;

        case ADC_TYPE_1:
            writel(readl(ts_base+S3C_ADCCON)|S3C_ADCCON_RESSEL_12BIT_1, ts_base+S3C_ADCCON);
            break;

        default:
            dev_err(dev, "Touchscreen over this type of AP isn't supported !\n");
            break;
        }
    }

    writel(WAIT4INT(0), ts_base+S3C_ADCTSC);

    ts = kzalloc(sizeof(struct s3c_ts_info), GFP_KERNEL);

    input_dev = input_allocate_device();

    if (!input_dev) {
        ret = -ENOMEM;
        goto fail;
    }

    ts->dev = input_dev;

    ts->dev->evbit[0] = ts->dev->evbit[0] = BIT_MASK(EV_SYN) | BIT_MASK(EV_KEY) | BIT_MASK(EV_ABS);
    ts->dev->keybit[BIT_WORD(BTN_TOUCH)] = BIT_MASK(BTN_TOUCH);

    if (s3c_ts_cfg->resol_bit==12)
    {
#if 1 /* ANDROID */
        input_set_abs_params(ts->dev, ABS_X, 0, 600, 0, 0);
        input_set_abs_params(ts->dev, ABS_Y, 0, 1070, 0, 0);
        /* input_set_abs_params(ts->dev, ABS_Z, 0, 0, 0, 0); */

        set_bit(0,ts->dev->evbit);
        set_bit(1,ts->dev->evbit);
        set_bit(2,ts->dev->evbit);
        set_bit(3,ts->dev->evbit);
        set_bit(5,ts->dev->evbit);

        set_bit(0,ts->dev->relbit);
        set_bit(1,ts->dev->relbit);

        set_bit(0,ts->dev->absbit);
        set_bit(1,ts->dev->absbit);
        set_bit(2,ts->dev->absbit);

        set_bit(0,ts->dev->swbit);

        for(err=0;err<512;err++) set_bit(err,ts->dev->keybit);

        input_event(ts->dev,5,0,1);
#else  /* 1 // ANDROID */
        input_set_abs_params(ts->dev, ABS_X, 0, 0xFFF, 0, 0);
        input_set_abs_params(ts->dev, ABS_Y, 0, 0xFFF, 0, 0);
#endif /* 1 // ANDROID */
    }
    else
    {
        input_set_abs_params(ts->dev, ABS_X, 0, 0x3FF, 0, 0);
        input_set_abs_params(ts->dev, ABS_Y, 0, 0x3FF, 0, 0);
    }

    input_set_abs_params(ts->dev, ABS_PRESSURE, 0, 1, 0, 0);

    sprintf(ts->phys, "input(ts)");

    ts->dev->name = s3c_ts_name;
    ts->dev->phys = ts->phys;
    ts->dev->id.bustype = BUS_RS232;
    ts->dev->id.vendor = 0xDEAD;
    ts->dev->id.product = 0xBEEF;
    ts->dev->id.version = S3C_TSVERSION;

    ts->shift = s3c_ts_cfg->oversampling_shift;
    ts->resol_bit = s3c_ts_cfg->resol_bit;
    ts->s3c_adc_con = s3c_ts_cfg->s3c_adc_con;

    /* For IRQ_PENDUP */
    ts_irq = platform_get_resource(pdev, IORESOURCE_IRQ, 0);
    if (ts_irq == NULL) {
        dev_err(dev, "no irq resource specified\n");
        ret = -ENOENT;
        goto err_clk;
    }

    ret = request_irq(ts_irq->start, stylus_updown, IRQF_SAMPLE_RANDOM, "s3c_updown", ts);
    if (ret != 0) {
        dev_err(dev,"s3c_ts.c: Could not allocate ts IRQ_PENDN !\n");
        ret = -EIO;
        goto err_clk;
    }

    /* For IRQ_ADC */
    ts_irq = platform_get_resource(pdev, IORESOURCE_IRQ, 1);
    if (ts_irq == NULL) {
        dev_err(dev, "no irq resource specified\n");
        ret = -ENOENT;
        goto err_clk;
    }

    ret = request_irq(ts_irq->start, stylus_action, IRQF_SAMPLE_RANDOM, "s3c_action", ts);
    if (ret != 0) {
        dev_err(dev, "s3c_ts.c: Could not allocate ts IRQ_ADC !\n");
        ret =  -EIO;
        goto err_irq;
    }

    printk(KERN_INFO "%s got loaded successfully : %d bits\n", s3c_ts_name, s3c_ts_cfg->resol_bit);

    /* All went ok, so register to the input system */
    ret = input_register_device(ts->dev);

    if(ret) {
        dev_err(dev, "s3c_ts.c: Could not register input device(touchscreen)!\n");
        ret = -EIO;
        goto fail;
    }

//#ifdef CONFIG_FB_S3C
    // Hook a button to offset the touch screen.
    s3c_gpio_setpull(S3C64XX_GPN(1), S3C_GPIO_PULL_UP);
    ret = request_irq(IRQ_EINT(1), offset_button_irq, IRQF_TRIGGER_FALLING, "ts_offset", (void *) 0x0);
    if (ret != 0) {
        printk("**** Failed to request IRQ 1 for offset button: %d\n", ret);
    }
//#endif /* CONFIG_FB_S3C */

    return 0;

fail:   input_free_device(input_dev);
    kfree(ts);

err_irq:
    free_irq(ts_irq->start, ts->dev);
    free_irq(ts_irq->end, ts->dev);

err_clk:
    clk_disable(ts_clock);
    clk_put(ts_clock);

err_map:
    iounmap(ts_base);

err_req:
    release_resource(ts_mem);
    kfree(ts_mem);

    return ret;
}

static int s3c_ts_remove(struct platform_device *dev)
{
    printk(KERN_INFO "s3c_ts_remove() of TS called !\n");

    disable_irq(IRQ_ADC);
    disable_irq(IRQ_PENDN);

    free_irq(IRQ_PENDN, ts->dev);
    free_irq(IRQ_ADC, ts->dev);

    if (ts_clock) {
        clk_disable(ts_clock);
        clk_put(ts_clock);
        ts_clock = NULL;
    }

    input_unregister_device(ts->dev);
    iounmap(ts_base);

    return 0;
}

#ifdef CONFIG_PM
static unsigned int adccon, adctsc, adcdly;

static int s3c_ts_suspend(struct platform_device *dev, pm_message_t state)
{
    adccon = readl(ts_base+S3C_ADCCON);
    adctsc = readl(ts_base+S3C_ADCTSC);
    adcdly = readl(ts_base+S3C_ADCDLY);

    disable_irq(IRQ_ADC);
    disable_irq(IRQ_PENDN);

    clk_disable(ts_clock);

    return 0;
}

static int s3c_ts_resume(struct platform_device *pdev)
{
    clk_enable(ts_clock);

    writel(adccon, ts_base+S3C_ADCCON);
    writel(adctsc, ts_base+S3C_ADCTSC);
    writel(adcdly, ts_base+S3C_ADCDLY);
    writel(WAIT4INT(0), ts_base+S3C_ADCTSC);

    enable_irq(IRQ_ADC);
    enable_irq(IRQ_PENDN);
    return 0;
}
#else  /* CONFIG_PM */
#define s3c_ts_suspend NULL
#define s3c_ts_resume  NULL
#endif /* CONFIG_PM */

static struct platform_driver s3c_ts_driver = {
    .probe   = s3c_ts_probe,
    .remove  = s3c_ts_remove,
    .suspend = s3c_ts_suspend,
    .resume  = s3c_ts_resume,
    .driver  = {
        .owner = THIS_MODULE,
        .name  = "s3c-ts",
    },
};

static char banner[] __initdata = KERN_INFO "S3C Touchscreen driver, (c) 2008 Samsung Electronics\n";

static int __init s3c_ts_init(void)
{
    printk(banner);
    return platform_driver_register(&s3c_ts_driver);
}

static void __exit s3c_ts_exit(void)
{
    platform_driver_unregister(&s3c_ts_driver);
}

module_init(s3c_ts_init);
module_exit(s3c_ts_exit);

MODULE_AUTHOR("Samsung AP");
MODULE_DESCRIPTION("S3C touchscreen driver");
MODULE_LICENSE("GPL");

