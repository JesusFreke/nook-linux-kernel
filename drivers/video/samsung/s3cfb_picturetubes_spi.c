/*
 * drivers/video/samsung/s3cfb_picturetubes_spi.c
 *
 * Copyright (c) 2009 Barnes and Noble, Inc
 * All rights reserved.
 *
 * Module author : Intrinsyc Software, Inc
 *
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file COPYING in the main directory of this archive for
 * more details.
 *
 *  S3C panel SPI bitbang driver
 */

#include <linux/delay.h>
#include <linux/err.h>
#include <linux/fb.h>
#include <linux/init.h>
#include <linux/lcd.h>
#include <linux/module.h>
#include <linux/spi/spi.h>
#include <linux/gpio.h>

#include <plat/regs-gpio.h>
#include <plat/gpio-cfg.h>

struct pictube_data {
	int resumed;
};

static int pictube_write_reg(struct spi_device *spi, u8 reg, u8 val)
{
	struct spi_message msg;
	struct spi_transfer index_xfer = {
		.len = 2,
		.cs_change = 1,
	};

	struct spi_transfer value_xfer = {
		.len = 2,
		.cs_change = 0,
	};

	u8 index_buf[2];
	u8 value_buf[2];

	spi_message_init(&msg);

	index_buf[0] = 0x74;
	index_buf[1] = reg;
	index_xfer.tx_buf = index_buf;
	spi_message_add_tail(&index_xfer, &msg);

	value_buf[0] = 0x76;
	value_buf[1] = val;
	value_xfer.tx_buf = value_buf;
	spi_message_add_tail(&value_xfer, &msg);
	
	return spi_sync(spi, &msg);
}

static int pictube_init(struct spi_device *spi)
{
	printk("PICTURE TUBE INIT START!\n");
	/*
		WAIT 150ms
		HX8352 83h,02h
		HX8352 85h,03h
		HX8352 8Bh,00h
		HX8352 8Ch,93h
		HX8352 91h,01h
		HX8352 83h,00h
		HX8352 3Eh,A5h
		HX8352 3Fh,52h
		HX8352 40h,00h
		HX8352 41h,36h
		HX8352 42h,00h
		HX8352 43h,77h
		HX8352 44h,15h
		HX8352 45h,76h
		HX8352 46h,01h
		HX8352 47h,00h
		HX8352 48h,00h
		HX8352 49h,02h
		HX8352 1DH,08h
		HX8352 2Bh,F9h
	*/

	msleep(150);
	pictube_write_reg(spi, 0x83, 0x2);
	pictube_write_reg(spi, 0x85, 0x3);
	pictube_write_reg(spi, 0x8b, 0x0);
	pictube_write_reg(spi, 0x8c, 0x93);
	pictube_write_reg(spi, 0x91, 0x1);
	pictube_write_reg(spi, 0x83, 0x0);
	pictube_write_reg(spi, 0x3e, 0xa5);
	pictube_write_reg(spi, 0x3f, 0x52);
	pictube_write_reg(spi, 0x40, 0x0);
	pictube_write_reg(spi, 0x41, 0x36);
	pictube_write_reg(spi, 0x42, 0x0);
	pictube_write_reg(spi, 0x43, 0x77);
	pictube_write_reg(spi, 0x44, 0x15);
	pictube_write_reg(spi, 0x45, 0x76);
	pictube_write_reg(spi, 0x46, 0x1);
	pictube_write_reg(spi, 0x47, 0x0);
	pictube_write_reg(spi, 0x48, 0x0);
	pictube_write_reg(spi, 0x49, 0x2);
	//pictube_write_reg(spi, 0x1d, 0x8);
	pictube_write_reg(spi, 0x2b, 0xf9);

	/*
	WAIT 10ms
	HX8352 1Bh,23h
	HX8352 1Ah,11h
	HX8352 1Ch,0Dh
	HX8352 1Fh,68h
	*/
	msleep(10);
	pictube_write_reg(spi, 0x1b, 0x23);
	pictube_write_reg(spi, 0x1a, 0x11);
	pictube_write_reg(spi, 0x1c, 0xc);
	//pictube_write_reg(spi, 0x1c, 0xd);
	//pictube_write_reg(spi, 0x1f, 0x68);
	pictube_write_reg(spi, 0x1f, 0x6a);
	
	/* 
	WAIT 20ms
	HX8352 19h,0Ah
	HX8352 19h,1Ah
	*/

	msleep(20);
	pictube_write_reg(spi, 0x19, 0xa);
	pictube_write_reg(spi, 0x19, 0x1a);

	/*
	WAIT 40ms
	HX8352 19h,12h
	*/

	msleep(40);
	pictube_write_reg(spi, 0x19, 0x12);
	
	/*
	WAIT 40ms
	HX8352 1Eh,0eh
	*/

	msleep(40);
	//pictube_write_reg(spi, 0x1e, 0xe);
	pictube_write_reg(spi, 0x1e, 0x2d);

	/*
	WAIT 100ms now 40
	HX8352 3Ch,C0h
	HX8352 3Dh,C0h
	HX8352 34h,38h
	HX8352 35h,38h
	HX8352 24h,38h
	*/

	msleep(40);
	pictube_write_reg(spi, 0x3c, 0xc0);
	pictube_write_reg(spi, 0x3d, 0xc0);
	pictube_write_reg(spi, 0x34, 0x38);
	pictube_write_reg(spi, 0x35, 0x38);
	pictube_write_reg(spi, 0x24, 0x38);

	/*
	WAIT 40ms
	HX8352 24h,3Ch
	HX8352 16h,1Ch
	HX8352 3Ah,C0h
	HX8352 01h,06h
	HX8352 55h,00h
	HX8352 22h,00h
	*/

	msleep(40);
	pictube_write_reg(spi, 0x24, 0x3c);
	pictube_write_reg(spi, 0x16, 0x1c);
	pictube_write_reg(spi, 0x3a, 0xc0);
	pictube_write_reg(spi, 0x1, 0x6);
	pictube_write_reg(spi, 0x55, 0x0);
	pictube_write_reg(spi, 0x22, 0x0);

	printk("PICTURE TUBE INIT DONE\n");
	return 0;
}

static int pictube_do_suspend(struct spi_device *spi)
{
	s3c_gpdat_setval(S3C64XX_GPM(4), 1);
	s3c_gpio_cfgpin(S3C64XX_GPM(4), S3C_GPIO_OUTPUT);
	s3c_gpio_setpull(S3C64XX_GPM(4), S3C_GPIO_PULL_NONE);
	s3c_gpdat_setval(S3C64XX_GPN(9), 0);
	
	return 0;
}

static int pictube_do_resume(struct spi_device *spi)
{
	s3c_gpdat_setval(S3C64XX_GPN(9), 1);
	s3c_gpdat_setval(S3C64XX_GPM(4), 0);
	s3c_gpio_cfgpin(S3C64XX_GPM(4), S3C_GPIO_OUTPUT);
	s3c_gpio_setpull(S3C64XX_GPM(4), S3C_GPIO_PULL_NONE);
	msleep(10);

	s3c_gpdat_setval(S3C64XX_GPN(9), 1);
	udelay(500);
	s3c_gpdat_setval(S3C64XX_GPN(9), 0);
	udelay(20);
	s3c_gpdat_setval(S3C64XX_GPN(9), 1);
	msleep(5);

	pictube_init(spi);

	return 0;
}

static ssize_t lcd_power_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct spi_device *spi = to_spi_device(dev);
	struct pictube_data *pd = spi_get_drvdata(spi);

	return sprintf(buf, "%u\n", pd->resumed);
}

static ssize_t lcd_power_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	unsigned long power;
	struct spi_device *spi = to_spi_device(dev);
	struct pictube_data *pd = spi_get_drvdata(spi);

	int ret = strict_strtoul(buf, 10, &power);

	if (ret) {
		dev_err(dev, "invalid parameter for lcd_power: %s\n", buf);
		goto out;
	}

	if (power != pd->resumed) {
		if (power) {
			printk("Resuming picture tubes\n");
			pictube_do_resume(spi);
		} else {
			printk("Suspending picture tubes\n");
			pictube_do_suspend(spi);
		}

		pd->resumed = power;
	}

	count = 0;
out:
	return count;
}

DEVICE_ATTR(lcd_power, 0666, lcd_power_show, lcd_power_store);

static int __devinit pictube_probe(struct spi_device *spi)
{
	int ret;
	struct pictube_data *pd;

	pd = kzalloc(sizeof(*pd), GFP_KERNEL);

	if (pd == NULL) {
		ret = -ENOMEM;
		goto out;
	}	

	pictube_do_resume(spi);
	ret = device_create_file(&spi->dev, &dev_attr_lcd_power);

	if (ret) {
		dev_err(&spi->dev, "failed to create lcd_power entry: %d\n", ret);
		goto err_free;
	}

	pd->resumed = 1;
	spi_set_drvdata(spi, pd);

	return ret;

err_free:
	kfree(pd);
out:
	return ret;
}

static int __devexit pictube_remove(struct spi_device *spi)
{
	device_remove_file(&spi->dev, &dev_attr_lcd_power);
	kfree(spi_get_drvdata(spi));
	return 0;
}

#ifdef CONFIG_PM
static int pictube_suspend(struct spi_device *spi, pm_message_t state)
{
        struct pictube_data *pd = spi_get_drvdata(spi);
	pd->resumed = 0;
	return pictube_do_suspend(spi);
}

static int pictube_resume(struct spi_device *spi)
{
	return 0;
}

#else
#define pictube_suspend NULL
#define pictube_resume NULL
#endif

static void pictube_shutdown(struct spi_device *spi)
{
	/* TODO */
}

static struct spi_driver pictube_spi_driver = {
	.driver = {
		.name = "pictube-spi",
		.owner = THIS_MODULE,
	},

	.probe		= pictube_probe,
	.remove		= __devexit_p(pictube_remove),
	.shutdown	= pictube_shutdown,
	.suspend	= pictube_suspend,
	.resume		= pictube_resume,
};

static int __init pictube_spi_init(void)
{
	return spi_register_driver(&pictube_spi_driver);
}

static void __exit pictube_spi_exit(void)
{
	spi_unregister_driver(&pictube_spi_driver);
}

module_init(pictube_spi_init);
module_exit(pictube_spi_exit);

MODULE_AUTHOR("David Bolcsfoldi <dbolcsfoldi@intrinsyc.com>");
MODULE_DESCRIPTION("Bravo smart LCD panel driver");
MODULE_LICENSE("GPL");

