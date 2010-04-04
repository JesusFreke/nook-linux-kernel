/*
 * wm8350-i2c.c  --  Generic I2C driver for Wolfson WM8350 PMIC
 *
 * Copyright 2007, 2008 Wolfson Microelectronics PLC.
 *
 * Author: Liam Girdwood
 *         linux@wolfsonmicro.com
 *
 *  This program is free software; you can redistribute  it and/or modify it
 *  under  the terms of  the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the  License, or (at your
 *  option) any later version.
 *
 */

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/init.h>
#include <linux/i2c.h>
#include <plat/iic.h>
#include <linux/platform_device.h>

struct tm01176001_device_info {
	struct device *dev;
	int id;
	struct i2c_client *i2c_client;
	int (*read_dev)(struct tm01176001_device_info *di, char reg, int size, void *dest);
	int (*write_dev)(struct tm01176001_device_info *di, char reg, int size, void *src);
};
static int cap_ts_i2c_read_device(struct tm01176001_device_info *di, char reg, int bytes, void *dest);
static int cap_ts_i2c_write_device(struct tm01176001_device_info *di, char reg, int bytes, void *src);

#ifdef CONFIG_MACH_BRAVO
static uint32_t g_reg;

static ssize_t cap_ts_r_val(struct device *dev, struct device_attribute *attr, char *buf)
{
	unsigned char regval = 0xff;
	struct tm01176001_device_info *di = dev->driver_data;

	cap_ts_i2c_read_device(di, g_reg, 1, &regval);
		
	return sprintf(buf, "0x%x\n",  regval);
}

static ssize_t cap_ts_w_val(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	unsigned char val = simple_strtoul(buf, NULL, 16);
	struct tm01176001_device_info *di = dev->driver_data;

	if(val > 0xffff)
		return -EINVAL;

	cap_ts_i2c_write_device(di, g_reg, 1, &val);

	return count;
}

static DEVICE_ATTR(val, S_IWUSR | S_IRUGO, cap_ts_r_val, cap_ts_w_val);

static ssize_t cap_ts_r_reg(struct device *dev, struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "0x%.2x\n", g_reg);
}

static ssize_t cap_ts_w_reg(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	unsigned long val = simple_strtoul(buf, NULL, 16);

	if(val > 0xffff)
		return -EINVAL;

	g_reg = val;

	return count;
}

static DEVICE_ATTR(reg, S_IWUSR | S_IRUGO, cap_ts_r_reg, cap_ts_w_reg);

static struct device_attribute* const cap_ts_attr[] = {
	&dev_attr_reg,
	&dev_attr_val,
};
#endif

static int cap_ts_i2c_read_device(struct tm01176001_device_info *di, char reg, int bytes, void *dest)
{
	int ret;

	ret = i2c_master_send(di->i2c_client, &reg, 1);
	if (ret < 0)
		goto err_out;
	
	ret = i2c_master_recv(di->i2c_client, dest, bytes);
	if (ret < 0)
		goto err_out;
	if (ret != bytes)
	{
		ret = -EIO;
		goto err_out;
	}

	printk(KERN_NOTICE "cap_ts_i2c_read_device: READ succeed\n");
	return 0;

err_out:
	printk(KERN_NOTICE "cap_ts_i2c_read_device FAIL to read\n");
	return ret;
}

static int cap_ts_i2c_write_device(struct tm01176001_device_info *di, char reg, int bytes, void *src)
{
	int ret;
	u8 msg[2];
	
	msg[0] = reg;
	memcpy(&msg[1], src, bytes);
	ret = i2c_master_send(di->i2c_client, msg, bytes + 1);
	if (ret < 0)
		goto err_out_1;
	
	if (ret != bytes + 1)
	{
		ret = -EIO;
		goto err_out_1;
	}

	printk(KERN_NOTICE "cap_ts_i2c_write_device: WRITE succeed\n");
	return 0;

err_out_1:
	printk(KERN_NOTICE "cap_ts_i2c_write_device: FAIL to write\n");
	return ret;
}

static int cap_ts_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	int retval;
	int i;
	struct tm01176001_device_info *di;
#ifdef CONFIG_MACH_BRAVO_I2C_MULTICLK
	struct s3c_i2c_client_platdata *client_pdata = (struct s3c_i2c_client_platdata *)(client->dev.platform_data);
#endif
	di = kzalloc(sizeof(struct tm01176001_device_info), GFP_KERNEL);
	if (di == NULL) {
		kfree(client);
		return -ENOMEM;
	}

	i2c_set_clientdata(client, di);
	di->dev = &client->dev;
	di->i2c_client = client;
	di->read_dev = cap_ts_i2c_read_device;
	di->write_dev = cap_ts_i2c_write_device;

	//Susan -- the last step, as device_attr for bq27200 testing
#ifdef CONFIG_MACH_BRAVO
	for (i = 0; i < ARRAY_SIZE(cap_ts_attr); i++) 
	{
		if ( (retval = device_create_file(&client->dev, cap_ts_attr[i])) < 0) 
		{
			while (i--) 
			{
				 device_remove_file(&client->dev, cap_ts_attr[i]);
			}
			goto err_out;
		}
	}
#endif
	printk(KERN_NOTICE "cap_ts_i2c_probe: return 0\n");
	return 0;

err_out:
	printk(KERN_NOTICE "cap_ts_i2c_probe: FAIL.\n");
	return retval;
}

static int cap_ts_i2c_remove(struct i2c_client *i2c)
{
	struct tm01176001_device_info *di = i2c_get_clientdata(i2c);

	kfree(di);

	return 0;
}

static const struct i2c_device_id cap_ts_i2c_id[] = {
       { "tm01176001", 0 },
       { }
};
MODULE_DEVICE_TABLE(i2c, cap_ts_i2c_id);

static struct i2c_driver cap_ts_i2c_driver = {
	.driver = {
		   .name = "tm01176001",
		   .owner = THIS_MODULE,
	},
	.probe = cap_ts_i2c_probe,
	.remove = cap_ts_i2c_remove,
	.id_table = cap_ts_i2c_id,
};

static int __init cap_ts_i2c_init(void)
{
	return i2c_add_driver(&cap_ts_i2c_driver);
}
/* init early so consumer devices can complete system boot */
//subsys_initcall(wm8350_i2c_init);

static void __exit cap_ts_i2c_exit(void)
{
	i2c_del_driver(&cap_ts_i2c_driver);
}
module_init(cap_ts_i2c_init);
module_exit(cap_ts_i2c_exit);

MODULE_DESCRIPTION("Capacitive Touch I2C support for the TM01176-001(ICS)");
MODULE_LICENSE("GPL");
