/*
 *  isl29001.c - Linux kernel modules for ambient light sensor
 *
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <plat/iic.h>
#include <linux/mutex.h>
#include <linux/delay.h>
#include <linux/workqueue.h>

#include <plat/gpio-cfg.h>
#include <mach/gpio.h>
#include <linux/i2c/isl29001.h>

#define DRIVER_VERSION		"0.0.3"

/*
 * Defines
 */

#define ISL29001_POWER_DOWN		0x00
#define ISL29001_POWER_RESET		0x0C
#define ISL29001_CNV_D1_IT		0x00
#define ISL29001_CNV_D2_IT		0x04
#define ISL29001_CNV_D1D2_IT		0x08
#define ISL29001_CNV_D1_ET		0x30
#define ISL29001_CNV_D2_ET		0x34
#define ISL29001_CNV_D1D2_ET		0x38

extern void s3c6410_enable_lightsensor(void);

/*
 * Structs
 */

struct isl29001_data {
	struct i2c_client *client;
	struct mutex update_lock;

	unsigned int power_state : 1;
	unsigned int operating_mode : 3;
};

/*
 * Global data
 */

static const u8 ISL29001_MODE_RANGE[6] = {
	ISL29001_CNV_D1_IT, ISL29001_CNV_D2_IT, ISL29001_CNV_D1D2_IT,
 	ISL29001_CNV_D1_ET, ISL29001_CNV_D2_ET, ISL29001_CNV_D1D2_ET,
};

/*
 * Management functions
 */

static int isl29001_set_operating_mode(struct i2c_client *client, int mode)
{
	struct isl29001_data *data = i2c_get_clientdata(client);

	int ret = i2c_smbus_write_byte(client, ISL29001_MODE_RANGE[mode]);

	data->operating_mode = mode;

	return ret;
}

static int isl29001_set_power_state(struct i2c_client *client, int state)
{
	struct isl29001_data *data = i2c_get_clientdata(client);
	int ret = 0;

	if (state == 0) {
		i2c_smbus_write_byte(client, ISL29001_POWER_DOWN);

		s3c_gpio_setpull(S3C64XX_GPN(12), S3C_GPIO_PULL_UP);
		s3c_gpdat_setval(S3C64XX_GPN(12), 1);
	}
	else {
		s3c6410_enable_lightsensor();
		ret = i2c_smbus_write_byte(client, ISL29001_POWER_RESET);

		if (ret >= 0) {
			/* On power up we should reset operating mode also... */
			ret = isl29001_set_operating_mode(client, data->operating_mode);
		}
	}

	if (ret < 0) {
		printk(KERN_ERR "Failed to set power state (%d) for light sensor: %d\n", state, ret);
	}

	data->power_state = state;

	return ret;
}

/*
 * SysFS support
 */

static ssize_t isl29001_show_power_state(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct isl29001_data *data = i2c_get_clientdata(to_i2c_client(dev));

	return sprintf(buf, "%u\n", data->power_state);
}

static ssize_t isl29001_store_power_state(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct isl29001_data *data = i2c_get_clientdata(client);
	unsigned long val = simple_strtoul(buf, NULL, 10);
	int ret;

	if (val < 0 || val > 1)
		return -EINVAL;

	mutex_lock(&data->update_lock);
	ret = isl29001_set_power_state(client, val);
	mutex_unlock(&data->update_lock);

	if (ret < 0)
		return ret;

	return count;
}

static DEVICE_ATTR(power_state, S_IWUSR | S_IRUGO,
		   isl29001_show_power_state, isl29001_store_power_state);

static ssize_t isl29001_show_operating_mode(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct isl29001_data *data = i2c_get_clientdata(to_i2c_client(dev));

	return sprintf(buf, "%u\n", data->operating_mode);
}

static ssize_t isl29001_store_operating_mode(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct isl29001_data *data = i2c_get_clientdata(client);
	unsigned long val = simple_strtoul(buf, NULL, 10);
	int ret;

	if (val < 0 || val > 1)
		return -EINVAL;

	if (data->power_state == 0)
		return -EBUSY;

	mutex_lock(&data->update_lock);
	ret = isl29001_set_operating_mode(client, val);
	mutex_unlock(&data->update_lock);

	if (ret < 0)
		return ret;

	return count;
}

static DEVICE_ATTR(operating_mode, S_IWUSR | S_IRUGO,
		   isl29001_show_operating_mode, isl29001_store_operating_mode);

/**
 * Returns light value from sensor.
 * @client: I2C client
 * @value: pointer to raw light level, set on return if no error
 *
 * Returns < 0 on error.
 *
 * The I2C client must be locked on entry.
 */
static int isl29001_get_lux(struct i2c_client *client, int *value)
{
   struct i2c_msg msg[1];
   unsigned char data[4];
   int err;
   int lux = -1;

   if (!client->adapter)
      return -ENODEV;

   msg->addr = client->addr;
   msg->flags = 0;
   msg->len = 4;
   msg->buf = data;
   memset(data, 0x00, 4);

   msg->flags = I2C_M_RD;
   err = i2c_transfer(client->adapter, msg, 1);
   if (err >= 0) 
   {
	lux = data[0] + (data[1] << 8);
	if (value) {
	    *value = lux;
	}
   	//printk("isl29001_show_lux read OK: data[0](0x%x),data[1](0x%x),data[2](0x%x),data[3](0x%x)\n", data[0],data[1],data[2],data[3]);
   }
   else
   	printk(KERN_NOTICE "isl29001_show_lux read FAILED.\n");

   return err;
}

static ssize_t isl29001_show_lux(struct i2c_client *client, char *buf)
{
	int lux;
	int rc = isl29001_get_lux(client, &lux);
	if ((rc >= 0) && buf) {
		rc = snprintf(buf, PAGE_SIZE, "%d\n", lux);
	}
	return rc;
}

static ssize_t isl29001_show_lux1_input(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct isl29001_data *data = i2c_get_clientdata(client);
	int ret;

	/* No LUX data if not operational */
	if (!data->power_state)
		return -EBUSY;

	mutex_lock(&data->update_lock);
	ret = isl29001_show_lux(client, buf);
	mutex_unlock(&data->update_lock);

	return ret;
}

static DEVICE_ATTR(lux1_input, S_IRUGO,
		   isl29001_show_lux1_input, NULL);

static struct attribute *isl29001_attributes[] = {
	&dev_attr_power_state.attr,
	&dev_attr_operating_mode.attr,
	&dev_attr_lux1_input.attr,
	NULL
};

static const struct attribute_group isl29001_attr_group = {
	.attrs = isl29001_attributes,
};

/*
 * Command function
 */

int isl29001_command(struct i2c_client *client, unsigned int cmd, void *arg)
{
	struct isl29001_data *data = i2c_get_clientdata(client);
	if ((cmd == ISL29001_CMD_READ_LUX) && arg) {
		mutex_lock(&data->update_lock);
		isl29001_get_lux(client, (int*)arg);
		mutex_unlock(&data->update_lock);
		return 0;
	}
	return -EINVAL;
}

/*
 * Initialization function
 */

static int __devinit isl29001_init_client(struct i2c_client *client)
{
	struct isl29001_data *data = i2c_get_clientdata(client);
	int err;
	char buf;

	/*
	 * Probe the chip. To do so we try to power up the device and then to
	 * read back the 0x03 code
	 */
	buf = ISL29001_POWER_RESET;	//ADC reset 
	err = i2c_master_send(client,&buf,1);
	if (err < 0) {
		dev_dbg(&client->dev, "isl29001: Error in write POWER RESET! [0x%x]\n",err);
		return err;
	}
	dev_dbg(&client->dev, "isl29001 I2C interface OK!\n");
	mdelay(1);
	data->power_state = 1;

	/* Set the default operating mode */
	err = i2c_smbus_write_byte(client,
				   ISL29001_MODE_RANGE[data->operating_mode]);
	if (err < 0) {
		dev_dbg(&client->dev, "isl29001: Error in write operation mode! [0x%x]\n",err);
		return err;
	}

	return 0;
}

/*
 * I2C init/probing/exit functions
 */

static int __devinit isl29001_probe(struct i2c_client *client,
				   const struct i2c_device_id *id)
{
	struct i2c_adapter *adapter = to_i2c_adapter(client->dev.parent);
	struct isl29001_data *data;
	struct s3c_i2c_client_platdata *pdata = NULL;
	int *opmode, err = 0;

	printk(KERN_NOTICE "isl29001_probe (0)\n");
	if (!i2c_check_functionality(adapter, I2C_FUNC_SMBUS_WORD_DATA)) {
		err = -EIO;
		goto exit;
	}

	data = kzalloc(sizeof(struct isl29001_data), GFP_KERNEL);
	if (!data) {
		err = -ENOMEM;
		goto exit;
	}
	data->client = client;
	i2c_set_clientdata(client, data);

	printk(KERN_NOTICE "isl29001_probe (1)\n");
	/* Check platform data */
	pdata = (struct s3c_i2c_client_platdata *)(client->dev.platform_data);
	opmode = (int *)(pdata->client_owner_platdata);
	if (opmode) {
		if (*opmode < 0 || *opmode > 5) {
			dev_err(&client->dev, "invalid operating_mode (%d)\n",
					*opmode);
			err = -EINVAL;
			goto exit_kfree;
		}
		data->operating_mode = *opmode;
	} else
		data->operating_mode = 0;	/* default mode is standard */
	dev_info(&client->dev, "%s operating mode\n",
			data->operating_mode ? "extended" : "standard");

	printk(KERN_NOTICE "isl29001_probe (2)\n");
	/* Initialize the ISL29001 chip */
	err = isl29001_init_client(client);
	if (err) {
		dev_dbg(&client->dev, "_init_client failed!\n");
		goto exit_kfree;
	}

	mutex_init(&data->update_lock);
	printk(KERN_NOTICE "isl29001_probe (3)\n");
	/* Register sysfs hooks */
	err = sysfs_create_group(&client->dev.kobj, &isl29001_attr_group);
	if (err) {
		dev_dbg(&client->dev, "sysfs create group failed!\n");
		goto exit_kfree;
	}

	dev_info(&client->dev, "support ver. %s enabled\n", DRIVER_VERSION);

	printk(KERN_NOTICE "isl29001_probe return 0\n");
	return 0;

exit_kfree:
	kfree(data);
exit:
	return err;
}

static int __devexit isl29001_remove(struct i2c_client *client)
{
	sysfs_remove_group(&client->dev.kobj, &isl29001_attr_group);

	/* Power down the device */
	isl29001_set_power_state(client, 0);

	kfree(i2c_get_clientdata(client));

	return 0;
}

#ifdef CONFIG_PM

static int isl29001_suspend(struct i2c_client *client, pm_message_t mesg)
{
	return isl29001_set_power_state(client, 0);
}

static int isl29001_resume(struct i2c_client *client)
{
	isl29001_set_power_state(client, 1);
	return 0;
}

#else

#define isl29001_suspend	NULL
#define isl29001_resume		NULL

#endif /* CONFIG_PM */

static const struct i2c_device_id isl29001_id[] = {
	{ "isl29001", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, isl29001_id);

struct i2c_driver isl29001_driver = {
	.driver = {
		.name	= ISL29001_DRV_NAME,
		.owner	= THIS_MODULE,
	},
	.suspend = isl29001_suspend,
	.resume	= isl29001_resume,
	.probe	= isl29001_probe,
	.command = isl29001_command,
	.remove	= __devexit_p(isl29001_remove),
	.id_table = isl29001_id,
};

static int __init isl29001_init(void)
{
#ifdef CONFIG_MACH_BRAVO_EVT1
	s3c6410_enable_lightsensor();  //Power up Light Sensor
#endif //EVT1
	return i2c_add_driver(&isl29001_driver);
}

static void __exit isl29001_exit(void)
{
	i2c_del_driver(&isl29001_driver);
}

MODULE_AUTHOR("Benny Chuang <bchuang@intrinsyc.com>");
MODULE_DESCRIPTION("ISL29001 ambient light sensor driver");
MODULE_LICENSE("GPL");
MODULE_VERSION(DRIVER_VERSION);

module_init(isl29001_init);
module_exit(isl29001_exit);
