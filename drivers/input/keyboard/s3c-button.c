    /* drivers/input/keyboard/s3c-keypad.c
 *
 * Driver core for Samsung SoC onboard UARTs.
 *
 * Kim Kyoungil, Copyright (c) 2006-2009 Samsung Electronics
 *      http://www.samsungsemi.com/
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/input.h>
#include <linux/platform_device.h>
#include <linux/miscdevice.h>
#include <linux/clk.h>
#include <linux/ktime.h>

#include <linux/io.h>
#include <mach/hardware.h>
#include <asm/delay.h>
#include <asm/irq.h>

#include <asm/gpio.h>
#include <plat/gpio-cfg.h>
#include <plat/regs-gpio.h>
#include <mach/bravo_gpio.h>

#undef S3C_KEYPAD_DEBUG
// #define CONFIG_BRAVO_BUTTON_OFFSET

#ifdef S3C_KEYPAD_DEBUG
#define DPRINTK(x...) printk("s3c-button " x)
#else
#define DPRINTK(x...)       /* !!!! */
#endif

#define DEVICE_NAME "s3c-button"

//#define ENABLE_HALL_SENSOR

#ifdef CONFIG_MACH_BRAVO_EVT2
#    ifdef ENABLE_HALL_SENSOR
#        define BUTTON_NO 6
#    else
#        define BUTTON_NO 5
#    endif /* HALL_SENSOR */
#else
#    define BUTTON_NO 3
#endif /* CONFIG_MACH_BRAVO_EVT2 */

struct s3c_buttons;

struct s3c_button {
    int key_code;
    int irq;
    int up;
    int reported;

    struct work_struct work;
    ktime_t time_down;
    struct s3c_buttons *parent;
};

struct s3c_buttons {
    struct resource *res;
    struct resource *button_mem;

    struct input_dev *dev;
    struct s3c_button buttons[BUTTON_NO];
    struct hrtimer pwr_timer;
#ifdef CONFIG_PM
    struct work_struct resume_work;
#endif
};

#ifdef CONFIG_BRAVO_BUTTON_OFFSET
extern void step_touchscreen_offset(void);
#endif

static ssize_t s3c_hall_sensor_state(struct device *dev, struct device_attribute *attr, char *buf)
{
    int state = 1; // Default to open

#ifdef ENABLE_HALL_SENSOR
    state = (get_s3c_reg_val(S3C64XX_GPNDAT) & 0x80) >> 7;
#endif /* HALL_SENSOR */

    return sprintf(buf, "%d\n", state);
}
int enable_page_turn = 0;
static ssize_t s3c_page_enable_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    if (enable_page_turn == 0)
        return sprintf(buf, "0\n");
    else
        return sprintf(buf, "1\n");
}

static ssize_t s3c_page_turn_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    return sprintf(buf, "not applicable\n");
}

static ssize_t s3c_page_turn_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
    struct s3c_buttons *buttons = dev_get_drvdata(dev);
    int key_code = 0;
    if (1 == enable_page_turn)
    {
        if (1 == simple_strtol(buf, NULL, 10))
        {
            key_code = KEY_LEFT_NEXTPAGE;
            printk("next page\n");
        }
        else
        {
            key_code = KEY_LEFT_PREVPAGE;
            printk("prev page\n");
        }
        input_report_key(buttons->dev, key_code, 1);
        input_report_key(buttons->dev, key_code, 0);
        input_sync(buttons->dev);
        enable_page_turn = 0;
    }   
    return 0;
}
static ssize_t s3c_page_enable_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
    enable_page_turn=1;
    return 0;
}



static DEVICE_ATTR(hall_sensor_state, 0444, s3c_hall_sensor_state, NULL);
static DEVICE_ATTR(page_turn, 0666, s3c_page_turn_show, s3c_page_turn_store);
static DEVICE_ATTR(page_enable, 0666, s3c_page_enable_show, s3c_page_enable_store);

#ifdef ENABLE_HALL_SENSOR
static void s3c_hall_sensor_work(struct work_struct *work)
{
    struct s3c_button *button = container_of(work, struct s3c_button, work);

    input_report_key(button->parent->dev, button->key_code, 1);
    input_report_key(button->parent->dev, button->key_code, 0);
    input_sync(button->parent->dev);
}
#endif /* HALL_SENSOR */

static void s3c_pwr_button_work(struct work_struct *work)
{
    struct s3c_button *button = container_of(work, struct s3c_button, work);

    if (button->up) {
        input_report_key(button->parent->dev, button->key_code, 0);
    } else {
        input_report_key(button->parent->dev, button->key_code, 1);
    }

    input_sync(button->parent->dev);
}

static void s3c_button_work(struct work_struct *work)
{
    struct s3c_button *button = container_of(work, struct s3c_button, work);

    if (button->up) {

#ifdef CONFIG_BRAVO_BUTTON_OFFSET
        if (button->key_code == KEY_LEFT_NEXTPAGE && !(get_s3c_reg_val(S3C64XX_GPNDAT) & 0x100)) {
            step_touchscreen_offset();
            return;
        }
#endif /* CONFIG_BRAVO_BUTTON_OFFSET */

    } else {
        input_report_key(button->parent->dev, button->key_code, 1);
        input_report_key(button->parent->dev, button->key_code, 0);
    }

    input_sync(button->parent->dev);
}

static irqreturn_t s3c_button_isr(int irq, void *dev_id)
{
    struct s3c_buttons *buttons = dev_id;

    disable_irq(irq);

    if (irq == IRQ_EINT(6)) {
        buttons->buttons[0].up = get_s3c_reg_val(S3C64XX_GPNDAT) & 0x40;
        schedule_work(&(buttons->buttons[0].work));
    } else if (irq == IRQ_EINT(8)) {
        buttons->buttons[1].up = get_s3c_reg_val(S3C64XX_GPNDAT) & 0x100;
        schedule_work(&(buttons->buttons[1].work));
    } else if (irq == IRQ_EINT(1)) {
         buttons->buttons[2].up = get_s3c_reg_val(S3C64XX_GPNDAT) & 0x2;
         schedule_work(&(buttons->buttons[2].work));
    }
#ifdef CONFIG_MACH_BRAVO_EVT2
    else if (irq == IRQ_EINT(24)) {
        buttons->buttons[3].up = get_s3c_reg_val(S3C64XX_GPMDAT) & 0x2;
        schedule_work(&(buttons->buttons[3].work));
    } else if (irq == IRQ_EINT(26)) {
        buttons->buttons[4].up = get_s3c_reg_val(S3C64XX_GPMDAT) & 0x8;
        schedule_work(&(buttons->buttons[4].work));
#   ifdef ENABLE_HALL_SENSOR
    } else if (irq == IRQ_EINT(7)) {
        buttons->buttons[5].up = get_s3c_reg_val(S3C64XX_GPNDAT) & 0x80;
        schedule_work(&(buttons->buttons[5].work));
#   endif /* HALL_SENSOR */
    }
#endif /* CONFIG_MACH_BRAVO_EVT2 */
    else {
        printk(KERN_ERR "Unknown interrupt %d in s3c-button driver!\n", irq);
    }

    enable_irq(irq);
    return IRQ_HANDLED;
}

#ifdef CONFIG_PM

static void s3c_button_resume_work(struct work_struct *work)
{
    struct s3c_buttons *buttons = container_of(work, struct s3c_buttons, resume_work);
    int pwr_button_pressed = ((get_s3c_reg_val(S3C64XX_GPNDAT) & 0x2) == 0);

    if (!pwr_button_pressed) {
        input_report_key(buttons->dev, KEY_WAKEUP, 1);
        input_report_key(buttons->dev, KEY_WAKEUP, 0);
        input_sync(buttons->dev);
    } else {
        // Power button is still down (Start of long Press)
        printk("%s: POWER Button down during resume\n", __FUNCTION__);
        buttons->buttons[2].up = 0;
        schedule_work(&(buttons->buttons[2].work));
    }
}

#endif /* CONFIG_PM */

static int s3c_button_probe(struct platform_device *pdev)
{
    struct s3c_buttons *s3c_buttons = NULL;
    int ret, size;

    s3c_buttons = kzalloc(sizeof(struct s3c_buttons), GFP_KERNEL);

    if (!s3c_buttons) {
        ret = -ENOMEM;
        goto out_1;
    }

    s3c_buttons->res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
    if (s3c_buttons->res == NULL) {
        dev_err(&pdev->dev,"no memory resource specified\n");
        ret = -ENOENT;
        goto out_1;
    }

    size = (s3c_buttons->res->end - s3c_buttons->res->start) + 1;

    s3c_buttons->button_mem = request_mem_region(s3c_buttons->res->start, size, pdev->name);
    if (s3c_buttons->button_mem == NULL) {
        dev_err(&pdev->dev, "failed to get memory region\n");
        ret = -ENOENT;
        goto out_2;
    }

    s3c_buttons->dev = input_allocate_device();

    if (!s3c_buttons->dev) {
        ret = -ENOMEM;
        goto out_2;
    }

    platform_set_drvdata(pdev, s3c_buttons);

    /* create and register the input driver */
    set_bit(EV_KEY, s3c_buttons->dev->evbit);
    set_bit(KEY_LEFT_NEXTPAGE & KEY_MAX, s3c_buttons->dev->keybit);
    set_bit(KEY_LEFT_PREVPAGE & KEY_MAX, s3c_buttons->dev->keybit);
    set_bit(KEY_RIGHT_NEXTPAGE & KEY_MAX, s3c_buttons->dev->keybit);
    set_bit(KEY_RIGHT_PREVPAGE & KEY_MAX, s3c_buttons->dev->keybit);
    set_bit(KEY_POWER & KEY_MAX, s3c_buttons->dev->keybit);
    set_bit(KEY_WAKEUP & KEY_MAX, s3c_buttons->dev->keybit);

    s3c_buttons->dev->name = DEVICE_NAME;
    s3c_buttons->dev->phys = "s3c-button/input0";

    s3c_buttons->dev->id.bustype = BUS_HOST;
    s3c_buttons->dev->id.vendor = 0x0001;
    s3c_buttons->dev->id.product = 0x0001;
    s3c_buttons->dev->id.version = 0x0001;

#ifdef CONFIG_PM
    INIT_WORK(&(s3c_buttons->resume_work), s3c_button_resume_work);
#endif

    s3c_buttons->buttons[0].key_code = KEY_RIGHT_PREVPAGE;
    s3c_buttons->buttons[0].irq = IRQ_EINT(6);
    s3c_buttons->buttons[0].up = 1;
    s3c_buttons->buttons[0].parent = s3c_buttons;
    INIT_WORK(&(s3c_buttons->buttons[0].work), s3c_button_work);

    s3c_buttons->buttons[1].key_code = KEY_RIGHT_NEXTPAGE;
    s3c_buttons->buttons[1].irq = IRQ_EINT(8);
    s3c_buttons->buttons[1].up = 1;
    s3c_buttons->buttons[1].parent = s3c_buttons;
    INIT_WORK(&(s3c_buttons->buttons[1].work), s3c_button_work);

    s3c_buttons->buttons[2].key_code = KEY_POWER;
    s3c_buttons->buttons[2].irq = IRQ_EINT(1);
    s3c_buttons->buttons[2].up = 1;
    s3c_buttons->buttons[2].parent = s3c_buttons;
    INIT_WORK(&(s3c_buttons->buttons[2].work), s3c_pwr_button_work);

#ifdef CONFIG_MACH_BRAVO_EVT2
    s3c_buttons->buttons[3].key_code = KEY_LEFT_NEXTPAGE;
    s3c_buttons->buttons[3].irq = IRQ_EINT(26);
    s3c_buttons->buttons[3].up = 1;
    s3c_buttons->buttons[3].parent = s3c_buttons;
    INIT_WORK(&(s3c_buttons->buttons[3].work), s3c_button_work);

    s3c_buttons->buttons[4].key_code = KEY_LEFT_PREVPAGE;
    s3c_buttons->buttons[4].irq = IRQ_EINT(24);
    s3c_buttons->buttons[4].up = 1;
    s3c_buttons->buttons[4].parent = s3c_buttons;
    INIT_WORK(&(s3c_buttons->buttons[4].work), s3c_button_work);

#   ifdef ENABLE_HALL_SENSOR
    s3c_buttons->buttons[5].key_code = KEY_POWER;
    s3c_buttons->buttons[5].irq = IRQ_EINT(7);
    s3c_buttons->buttons[5].up = 1;
    s3c_buttons->buttons[5].parent = s3c_buttons;
    INIT_WORK(&(s3c_buttons->buttons[5].work), s3c_hall_sensor_work);
#   endif /* HALL_SENSOR */

    ret = device_create_file(&pdev->dev, &dev_attr_page_enable);
    if (ret < 0)
    {
        printk(KERN_ERR "s3c_button_probe() - WARNING: failed to add page_enable sysfs: %d\n", ret);
    }

    ret = device_create_file(&pdev->dev, &dev_attr_page_turn);
    if (ret < 0)
    {
        printk(KERN_ERR "s3c_button_probe() - WARNING: failed to add page_turn sysfs: %d\n", ret);
    }

    ret = device_create_file(&pdev->dev, &dev_attr_hall_sensor_state);
    if (ret < 0)
    {
        printk(KERN_ERR "s3c_button_probe() - WARNING: failed to add hall sensor sysfs: %d\n", ret);
    }

#endif /* CONFIG_MACH_BRAVO_EVT2 */

    ret = request_irq(IRQ_EINT(6), s3c_button_isr, IRQF_TRIGGER_FALLING | IRQF_TRIGGER_RISING, "s3c-button", s3c_buttons);
    s3c_gpio_setpull(S3C64XX_GPN(6), S3C_GPIO_PULL_NONE);

    if (ret) {
        printk(KERN_ERR "request_irq failed IRQ_EINT(6)!!!\n");
        ret = -EIO;
        goto out_3;
    }

    ret = request_irq(IRQ_EINT(8), s3c_button_isr, IRQF_TRIGGER_FALLING | IRQF_TRIGGER_RISING, "s3c-button", s3c_buttons);
    s3c_gpio_setpull(S3C64XX_GPN(8), S3C_GPIO_PULL_NONE);

    if (ret) {
        printk(KERN_ERR "request_irq failed IRQ_EINT(8) !!!\n");
        ret = -EIO;
        goto out_irq1;
    }

    ret = request_irq(IRQ_EINT(1), s3c_button_isr, IRQF_TRIGGER_FALLING | IRQF_TRIGGER_RISING, "s3c-button", s3c_buttons);
    s3c_gpio_setpull(S3C64XX_GPN(1), S3C_GPIO_PULL_NONE);

    if (ret) {
        printk(KERN_ERR "request_irq failed IRQ_EINT(1) !!!\n");
        ret = -EIO;
        goto out_irq2;
    }

#ifdef CONFIG_MACH_BRAVO_EVT2
    ret = request_irq(IRQ_EINT(26), s3c_button_isr, IRQF_TRIGGER_FALLING | IRQF_TRIGGER_RISING, "s3c-button", s3c_buttons);
    s3c_gpio_setpull(S3C64XX_GPM(3), S3C_GPIO_PULL_NONE);

    if (ret) {
        printk(KERN_ERR "request_irq failed IRQ_EINT(26)!!!\n");
        ret = -EIO;
        goto out_irq3;
    }

    ret = request_irq(IRQ_EINT(24), s3c_button_isr, IRQF_TRIGGER_FALLING | IRQF_TRIGGER_RISING, "s3c-button", s3c_buttons);
    s3c_gpio_setpull(S3C64XX_GPM(1), S3C_GPIO_PULL_NONE);

    if (ret) {
        printk(KERN_ERR "request_irq failed IRQ_EINT(24) !!!\n");
        ret = -EIO;
        goto out_irq4;
    }

#ifdef ENABLE_HALL_SENSOR
    ret = request_irq(IRQ_EINT(7), s3c_button_isr, IRQF_TRIGGER_FALLING | IRQF_TRIGGER_RISING, "s3c-button", s3c_buttons);
    s3c_gpio_setpull(S3C64XX_GPN(7), S3C_GPIO_PULL_NONE);

    if (ret) {
        printk(KERN_ERR "request_irq failed IRQ_EINT(7) !!!\n");
        ret = -EIO;
        goto out_irq5;
    }
#endif /* HALL_SENSOR */

#endif /* CONFIG_MACH_BRAVO_EVT2 */

    ret = input_register_device(s3c_buttons->dev);

    if (ret) {
        printk(KERN_ERR "Unable to register s3c-keypad input device!!!\n");
        goto out_irq3;
    }

    printk(DEVICE_NAME " Initialized\n");
    return 0;
#ifdef CONFIG_MACH_BRAVO_EVT2
out_irq5:
    free_irq(s3c_buttons->buttons[4].irq, s3c_buttons);
out_irq4:
    free_irq(s3c_buttons->buttons[3].irq, s3c_buttons);
#endif /* CONFIG_MACH_BRAVO_EVT2 */
out_irq3:
    free_irq(s3c_buttons->buttons[2].irq, s3c_buttons);
out_irq2:
    free_irq(s3c_buttons->buttons[1].irq, s3c_buttons);
out_irq1:
    free_irq(s3c_buttons->buttons[0].irq, s3c_buttons);
out_3:
    input_free_device(s3c_buttons->dev);
out_2:
    release_resource(s3c_buttons->button_mem);
    kfree(s3c_buttons->button_mem);
out_1:
    kfree(s3c_buttons);

    return ret;
}

static int s3c_button_remove(struct platform_device *pdev)
{
    struct s3c_buttons *dev = platform_get_drvdata(pdev);

    free_irq(dev->buttons[0].irq, dev);
    free_irq(dev->buttons[1].irq, dev);
    free_irq(dev->buttons[2].irq, dev);
#ifdef CONFIG_MACH_BRAVO_EVT2
    free_irq(dev->buttons[3].irq, dev);
    free_irq(dev->buttons[4].irq, dev);
#ifdef ENABLE_HALL_SENSOR
    free_irq(dev->buttons[5].irq, dev);
#endif /* HALL_SENSOR */

    device_remove_file(&pdev->dev, &dev_attr_hall_sensor_state);
    device_remove_file(&pdev->dev, &dev_attr_page_turn);
    device_remove_file(&pdev->dev, &dev_attr_page_enable);

#endif /* CONFIG_MACH_BRAVO_EVT2 */
    input_unregister_device(dev->dev);

    release_resource(dev->button_mem);
    kfree(dev->button_mem);
    kfree(dev);

    printk(DEVICE_NAME " Removed.\n");
    return 0;
}

#ifdef CONFIG_PM
#include <plat/pm.h>

static int s3c_button_suspend(struct platform_device *dev, pm_message_t state)
{
    return 0;
}

static int s3c_button_resume(struct platform_device *dev)
{
    struct s3c_buttons *buttons = platform_get_drvdata(dev);
    schedule_work(&(buttons->resume_work));
    return 0;
}
#else
#define s3c_button_suspend NULL
#define s3c_button_resume  NULL
#endif /* CONFIG_PM */

static struct platform_driver s3c_button_driver = {
    .probe      = s3c_button_probe,
    .remove     = s3c_button_remove,
    .suspend    = s3c_button_suspend,
    .resume     = s3c_button_resume,
    .driver     = {
        .owner  = THIS_MODULE,
        .name   = "s3c-button",
    },
};

static int __init s3c_button_init(void)
{
    int ret;

    ret = platform_driver_register(&s3c_button_driver);

    if(!ret)
       printk(KERN_INFO "S3C Button Driver\n");

    return ret;
}

static void __exit s3c_button_exit(void)
{
    platform_driver_unregister(&s3c_button_driver);
}

module_init(s3c_button_init);
module_exit(s3c_button_exit);

MODULE_AUTHOR("Samsung");
MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("KeyPad interface for Samsung S3C");

