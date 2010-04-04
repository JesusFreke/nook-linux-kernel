/*
 * BQ27x00 battery driver
 *
 * Copyright (C) 2008 Rodolfo Giometti <giometti@linux.it>
 * Copyright (C) 2008 Eurotech S.p.A. <info@eurotech.it>
 *
 * Based on a previous work by Copyright (C) 2008 Texas Instruments, Inc.
 *
 * This package is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * THIS PACKAGE IS PROVIDED ``AS IS'' AND WITHOUT ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, WITHOUT LIMITATION, THE IMPLIED
 * WARRANTIES OF MERCHANTIBILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 *
 * Modifications 2009 : Intrinsyc Software, Inc on behalf of Barnes and Noble
 * Portions of this code copyright (c) 2009 Barnes and Noble, Inc
 *
 */
#include <linux/module.h>
#include <linux/param.h>
#include <linux/jiffies.h>
#include <linux/workqueue.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/power_supply.h>
#include <linux/pm.h>
#include <linux/idr.h>
#include <linux/i2c.h>
#include <asm/unaligned.h>

#define DRIVER_VERSION         "1.0.0"

#define BQ27x00_REG_TEMP      0x06
#define BQ27x00_REG_VOLT      0x08
#define BQ27x00_REG_RSOC      0x0B /* Relative State-of-Charge */
#define BQ27x00_REG_AI         0x14
#define BQ27x00_REG_FLAGS      0x0A
#define BQ27x00_VENDOR_MODEL	0x7E
#define BQ27x00_VALID_VENDOR1 0x02 // the LICO battery has 0x02 in register 0x7E
#define BQ27x00_VALID_VENDOR2 0x00 // the B&N battery has 0x00 in register 0x7E
#define BQ27x00_REG_TTE		0x16
#define BQ27x00_REG_TTF		0x18

#define BQ27X00_DEBUG  printk
/* If the system has several batteries we need a different name for each
 * of them...
 */

#define BQ27X00_STATUS_MONITOR_UR	1	//status monitor update routine
static DEFINE_IDR(battery_id);
static DEFINE_MUTEX(battery_mutex);

#define BQ27X00_MAX_ERROR_MESSAGES 30

#define CURRENT_CONV(I) (I * 178)  /* (3.57 / 0.02) = 178.5 */

void show_stack(struct task_struct *, unsigned long *);

struct bq27x00_device_info;
struct bq27x00_access_methods {
   int (*read)(u8 reg, int *rt_value, int b_single,
      struct bq27x00_device_info *di);
};

struct bq27x00_device_info {
   struct device       *dev;
   int         id;
   int         voltage_uV;
   int         current_uA;
   int         temp_C;
   int         charge_rsoc;
   int 		charge_status; //Benny 06-10
   int 		health;
   char 	*manufacturer;	//Benny 07-10
   struct bq27x00_access_methods   *bus;
   struct power_supply   bat;

   struct i2c_client   *client;
#if BQ27X00_STATUS_MONITOR_UR
   struct delayed_work monitor_work;
#endif
#ifdef CONFIG_BRAVO_LOWBATT_WAKEUP
   int  low_thr;
#endif
   //cached parameters to reduce I2C overhead
   int volt_cached;
   int curr_cached;
   int rsoc_cached;
   int temp_cached;
   int tte_cached;
   int ttf_cached;
   int vendor_cached;
   int turn;
   //end of cached parameters
};
#ifdef CONFIG_BRAVO_LOWBATT_WAKEUP
static struct bq27x00_device_info *bq27200_di = NULL;
#endif

static enum power_supply_property bq27x00_battery_props[] = {
   POWER_SUPPLY_PROP_STATUS,  
   POWER_SUPPLY_PROP_HEALTH,	//Benny 06-13 added, TBD
   POWER_SUPPLY_PROP_PRESENT,
   POWER_SUPPLY_PROP_TECHNOLOGY,
   POWER_SUPPLY_PROP_VOLTAGE_NOW,
   POWER_SUPPLY_PROP_CURRENT_NOW,
   POWER_SUPPLY_PROP_CAPACITY,
   POWER_SUPPLY_PROP_TEMP,
   POWER_SUPPLY_PROP_TIME_TO_EMPTY_NOW,
   POWER_SUPPLY_PROP_TIME_TO_FULL_NOW,
   POWER_SUPPLY_PROP_MANUFACTURER, 	//Benny 07-09 TBD
};

static enum	{
	BATMUF_UNKNOWN = 0,
	BATMUF_LICO,
};

 static int bq27200_read(u8 reg, int *rt_value, int b_single, struct bq27x00_device_info *di);

static int bq27x00_battery_voltage_compensated(int volt, struct bq27x00_device_info *di);

#ifdef CONFIG_MACH_BRAVO
static uint32_t g_bq27200_reg;

static int g_bqErrorMessageCount = 0;

#define bq_error_msg(dev, format, arg...)                             \
    if (g_bqErrorMessageCount++ <= BQ27X00_MAX_ERROR_MESSAGES)        \
        dev_err( dev , format , ## arg);                              \
    if (g_bqErrorMessageCount == BQ27X00_MAX_ERROR_MESSAGES)          \
        printk("Too many bq27200 errors - assuming no battery!\r\n");    
    
static ssize_t bq27200_r_val(struct device *dev, struct device_attribute *attr, char *buf)
{
	int rtval = 0x0;
	bq27200_read(g_bq27200_reg, &rtval, 1, dev->driver_data);
		
	return sprintf(buf, "0x%x\n",  rtval);
}

static ssize_t bq27200_w_val(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	unsigned long val = simple_strtoul(buf, NULL, 16);

	if(val > 0xffff)
		return -EINVAL;

	//bq27200_reg_write(dev->driver_data, g_bq27200_reg, val);

	return count;
}

static DEVICE_ATTR(val, S_IWUSR | S_IRUGO, bq27200_r_val, bq27200_w_val);

static ssize_t bq27200_r_reg(struct device *dev, struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "0x%.2x\n", g_bq27200_reg);
}

static ssize_t bq27200_w_reg(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	unsigned long val = simple_strtoul(buf, NULL, 16);

	if(val > 0xffff)
		return -EINVAL;

	g_bq27200_reg = val;

	return count;
}

static DEVICE_ATTR(reg, S_IWUSR | S_IRUGO, bq27200_r_reg, bq27200_w_reg);

static struct device_attribute* const bq27200_attr[] = {
	&dev_attr_reg, 
	&dev_attr_val,
};
#endif

/*
 * Common code for BQ27x00 devices
 */
static int bq27x00_read(u8 reg, int *rt_value, int b_single,
         struct bq27x00_device_info *di)
{
   int ret;

   // Do not hog the I2C bus
   //schedule_timeout_interruptible(msecs_to_jiffies(200));
   ret = di->bus->read(reg, rt_value, b_single, di);
   *rt_value = be16_to_cpu(*rt_value);

   return ret;
}

#ifdef CONFIG_BRAVO_LOWBATT_WAKEUP
extern int g_pccmp_off_thr;
void battery_set_low_thr(int low_vol_threshold)
{
	int flags;

	local_irq_save(flags);
	//Do we need to take mutex on accessing bq27x00_device_info structure?
	bq27200_di->low_thr = low_vol_threshold*100+2900;
	local_irq_restore(flags);
}
EXPORT_SYMBOL_GPL(battery_set_low_thr);

void battery_low_immediate_report(void)
{
	int flags;
	struct work_struct *work = NULL;

	if (bq27200_di)  //bq27200 driver has been initialized
		work = (struct work_struct *)(&(bq27200_di->monitor_work.work));

	if (work)
	{
		local_irq_save(flags);
		schedule_work(work);
		local_irq_restore(flags);
	}
}
EXPORT_SYMBOL_GPL(battery_low_immediate_report);
#endif
/*
 * Return the battery temperature in Celcius degrees
 * Or < 0 if something fails.
 */
static int bq27x00_battery_temperature(struct bq27x00_device_info *di)
{

   int ret;
   int temp = 0;

   ret = bq27x00_read(BQ27x00_REG_TEMP, &temp, 0, di);
   if (ret) {
      bq_error_msg(di->dev, "error reading temperature\n");
      return di->temp_cached;	//return the previous value since our bq27x00_read failed
   }

   //printk(KERN_NOTICE "bq27x00_battery_temperature: temperature(%d)\n", ((temp>>2) -273));
   //return (temp >> 2) - 273;
   //printk(KERN_NOTICE "bq27x00_battery_temperature: temperature(%d)\n", (temp>>2));
   return ((temp >> 2) - 273);
}

/*
 * Return the battery Voltage in milivolts
 * //out_of_date: "Or < 0 if something fails"
 */
static int bq27x00_battery_voltage(struct bq27x00_device_info *di)
{
   int ret;
   int volt = 0;

   ret = bq27x00_read(BQ27x00_REG_VOLT, &volt, 0, di);
   if (ret) {
      bq_error_msg(di->dev, "error reading voltage\n");
      //return ret;
      return di->volt_cached;	//returned value assumed to be volt other than true/false
   }
   
   //in case the I2C returns a invalid value
   if (volt < 2000) {
      bq_error_msg(di->dev, "I2C returned voltage is less than 2v!!\n");  
      volt = di->volt_cached;
   }
   	 
   //printk(KERN_NOTICE "bq27x00_battery_voltage: volt(%d)\n", volt);

   //volt = bq27x00_battery_voltage_compensated(volt, di);

   //printk(KERN_NOTICE "bq27x00_battery_voltage_compensated: volt(%d)\n", volt);
   
   return volt;
}

/*
 * Return the battery average current
 * Note that current can be negative signed as well
 * Or 0 if something fails.
 */
static int bq27x00_battery_current(struct bq27x00_device_info *di)
{
   int ret;
   int curr = 0;
   int flags = 0;

   ret = bq27x00_read(BQ27x00_REG_AI, &curr, 0, di);
   if (ret) {
      bq_error_msg(di->dev, "error reading current\n");
      return 0;
   }
   curr = CURRENT_CONV(curr);

   ret = bq27x00_read(BQ27x00_REG_FLAGS, &flags, 0, di);
   if (ret < 0) {
      bq_error_msg(di->dev, "error reading flags\n");
      return 0;
   }
   if ((flags & (1 << 7)) != 0) {
      return -curr;
   }

   //printk(KERN_NOTICE "bq27x00_battery_current: curr(%d)\n", curr);
   return curr;
}

/* 
 * Adjust voltage reading from bq27200 with formula U = U - I * 0.15
 */
static int bq27x00_battery_voltage_compensated(int volt, struct bq27x00_device_info *di)
{
   int curr = bq27x00_battery_current(di);

   //bypass compensation when battery is being charged	   
   return (curr > 0 ? (int)(volt-((curr/1000)*15)/100) : volt);
}

/*
 * Return the battery Relative State-of-Charge
 * //Or < 0 if something fails.
 */
static int bq27x00_battery_rsoc(struct bq27x00_device_info *di)
{
   int ret;
   int rsoc = 0;

   ret = bq27x00_read(BQ27x00_REG_RSOC, &rsoc, 1, di);
   if (ret) {
      bq_error_msg(di->dev, "error reading relative State-of-Charge\n");
      //return ret;
      return di->rsoc_cached;
   }

   rsoc >>= 8;
   
   //in case the I2C returns a invalid value
   if ( rsoc <= 0 ) {
      bq_error_msg(di->dev, "I2C returned battery capacity is less than 0%%!!\n");  
      rsoc = di->rsoc_cached;
   }
   	 
   return rsoc ;
}

static int bq27x00_battery_vendor(struct bq27x00_device_info *di)
{
   int ret;
   int vendor= 0;

   //the vendor ID is located in register BQ27x00_VENDOR_MODEL
   ret = bq27x00_read(BQ27x00_VENDOR_MODEL, &vendor, 1, di);
   if (ret) {
      bq_error_msg(di->dev, "error reading vendor ID\n");
      return ret;
   }
   vendor = vendor >>8;
//  printk(KERN_NOTICE  "vendor  = [%x]  \n", vendor);
   return vendor;
}

static int bq27x00_battery_ttf(struct bq27x00_device_info *di)
{
   int ret;
   int ttf = 0;

   ret = bq27x00_read(BQ27x00_REG_TTF, &ttf, 0, di);
   if (ret) {
      bq_error_msg(di->dev, "error reading ttf\n");
      return ret;
   }
   //printk(KERN_NOTICE "bq27x00_battery_ttf: ttf(%x)\n", ttf);

   return ttf;
}

static int bq27x00_battery_tte(struct bq27x00_device_info *di)
{
   int ret;
   int tte = 0;

   ret = bq27x00_read(BQ27x00_REG_TTE, &tte, 0, di);
   if (ret) {
      bq_error_msg(di->dev, "error reading tte\n");
      return ret;
   }
   //printk(KERN_NOTICE "bq27x00_battery_tte: tte(%x)\n", tte);

   return tte;
}

#if 1
static int bq27x00_battery_sflag(struct bq27x00_device_info *di)
{
   int ret;
   int flags = 0;

   ret = bq27x00_read(BQ27x00_REG_FLAGS, &flags, 1, di);
   if (ret < 0) {
      bq_error_msg(di->dev, "error reading Status flags\n");
      di->charge_status = POWER_SUPPLY_STATUS_UNKNOWN;
      return ret;
   }  
   dev_dbg(di->dev, "Status Flags read value [%x]\n",flags);
//	printk("Status Flags read value [%x]\n",(flags >> 8));

   return (flags >> 8);
}
#endif

#define to_bq27x00_device_info(x) container_of((x), \
            struct bq27x00_device_info, bat);

static int bq27x00_battery_get_property(struct power_supply *psy,
               enum power_supply_property psp,
               union power_supply_propval *val)
{
   int vendorID;
   
   //printk(KERN_NOTICE "bq27x00_battery_get_property()\n");        	        	

   struct bq27x00_device_info *di = to_bq27x00_device_info(psy);
   switch (psp) {
   case POWER_SUPPLY_PROP_STATUS:
	val->intval = di->charge_status;
	dev_dbg(di->dev, "Status of bq27x00 is %x\n",di->charge_status);
	break;
   case POWER_SUPPLY_PROP_HEALTH:
	val->intval = di->health;
	break;
   case POWER_SUPPLY_PROP_VOLTAGE_NOW:
   case POWER_SUPPLY_PROP_PRESENT:
      //val->intval = bq27x00_battery_voltage(di);
      val->intval = di->volt_cached;
      //printk(KERN_NOTICE "bq27x00_battery_get_property(), volt(%d)\n", val->intval);       
      if (psp == POWER_SUPPLY_PROP_PRESENT)
      //{
         val->intval = val->intval <= 0 ? 0 : 1;
         //printk(KERN_NOTICE "bq27x00_battery_get_property(), pres(%d)\n", val->intval);       
         //}
	break;
   case POWER_SUPPLY_PROP_TECHNOLOGY:
	val->intval = POWER_SUPPLY_TECHNOLOGY_LIPO;
	break;
   case POWER_SUPPLY_PROP_CURRENT_NOW:
      //val->intval = bq27x00_battery_current(di);
      val->intval = di->curr_cached;
      //printk(KERN_NOTICE "bq27x00_battery_get_property(), curr(%d)\n", val->intval);       
      break;
   case POWER_SUPPLY_PROP_CAPACITY:
      //val->intval = bq27x00_battery_rsoc(di);
      val->intval = di->rsoc_cached;
      //printk(KERN_NOTICE "bq27x00_battery_get_property(), rsoc(%d)\n", val->intval);       
      break;
   case POWER_SUPPLY_PROP_TEMP:
      //val->intval = bq27x00_battery_temperature(di);
      val->intval = di->temp_cached;
      //printk(KERN_NOTICE "bq27x00_battery_get_property(), temp(%d)\n", val->intval);       
      break;
   case POWER_SUPPLY_PROP_TIME_TO_EMPTY_NOW:
      //val->intval = bq27x00_battery_tte(di);
      val->intval = di->tte_cached;
      //printk(KERN_NOTICE "bq27x00_battery_get_property(), tte(%d)\n", val->intval);             
      break;
   case POWER_SUPPLY_PROP_TIME_TO_FULL_NOW:
      //val->intval = bq27x00_battery_ttf(di);
      val->intval = di->ttf_cached;
      //printk(KERN_NOTICE "bq27x00_battery_get_property(), ttf(%d)\n", val->intval);       
      break;
   case POWER_SUPPLY_PROP_MANUFACTURER:
   	//vendorID= bq27x00_battery_vendor(di) & 0xFE;  // we only care about bits 1-7
   	vendorID= di->vendor_cached; 
	if ((vendorID == BQ27x00_VALID_VENDOR2))
	{
		val->intval =  BATMUF_LICO;
      		di->charge_status =  di->charge_status;
	}
	else
	{
		val->intval = BATMUF_UNKNOWN;
		di->charge_status = POWER_SUPPLY_STATUS_UNKNOWN;
	}
	//printk(KERN_NOTICE "bq27x00_battery_get_property(), vendor(%d)\n", val->intval);       
      break;
   default:
	dev_dbg(di->dev, "Unknown psp number is %x\n",psp);
       return -EINVAL;
   }

   return 0;
}

static void bq27x00_powersupply_init(struct bq27x00_device_info *di)
{
   di->bat.type = POWER_SUPPLY_TYPE_BATTERY;
   di->bat.properties = bq27x00_battery_props;
   di->bat.num_properties = ARRAY_SIZE(bq27x00_battery_props);
   di->bat.get_property = bq27x00_battery_get_property;
   di->bat.external_power_changed = NULL;
}

/*
 * BQ27200 specific code
 */

static int bq27200_read(u8 reg, int *rt_value, int b_single,
         struct bq27x00_device_info *di)
{
   struct i2c_client *client = di->client;
   struct i2c_msg msg[1];
   unsigned char data[2];
   int err;

   if (!client->adapter)
      return -ENODEV;

   msg->addr = client->addr;
   msg->flags = 0;
   msg->len = 1;
   msg->buf = data;

   data[0] = reg;
   err = i2c_transfer(client->adapter, msg, 1);

   if (err >= 0) {
      if (!b_single)
         msg->len = 2;
      else
         msg->len = 1;

      msg->flags = I2C_M_RD;
      err = i2c_transfer(client->adapter, msg, 1);
      if (err >= 0) {
         if (!b_single)
            *rt_value = get_unaligned_be16(data);
         else
            *rt_value = data[0];

         return 0;
      }
   }
   return err;
}

#if BQ27X00_STATUS_MONITOR_UR
#define MONITOR_PERIOD	5*HZ
#define LOWER_THRESHOLD_VOL	3500
static void bq27200_battery_status_monitor(struct work_struct *work)
{
	struct bq27x00_device_info *di = container_of(work,struct bq27x00_device_info, monitor_work.work);
#ifdef CONFIG_BRAVO_LOWBATT_WAKEUP
	int volt;
#endif

	dev_dbg(di->dev, "Battery status monitor routine!\n");
	
	di->charge_status =  (bq27x00_battery_sflag(di) & 0x80 ) ? POWER_SUPPLY_STATUS_CHARGING:POWER_SUPPLY_STATUS_DISCHARGING;
	
#ifdef CONFIG_BRAVO_LOWBATT_WAKEUP
	//Susan -- voltage value reading from gas gadget is larger than PCCMP_OFF_THR when SYS_HYST_IRQ happens
	//So we use the delta of "500"mv to do comparison
	if( ( di->volt_cached = (volt = bq27x00_battery_voltage(di)) ) < (di->low_thr+100) )
	{
		//printk(KERN_NOTICE "Battery Status Monitor: volt(%d), low_thr(%d), POWER_SUPPLY_HEALTH_DEAD\n", volt, di->low_thr);
		di->health = POWER_SUPPLY_HEALTH_DEAD;
	}
	else
	{
		//printk(KERN_NOTICE "Battery Status Monitor: volt(%d), low_thr(%d), POWER_SUPPLY_HEALTH_GOOD\n", volt, di->low_thr);
		di->health = POWER_SUPPLY_HEALTH_GOOD;
	}
#else
	if( bq27x00_battery_voltage(di) < LOWER_THRESHOLD_VOL ) 
	{
		//printk(KERN_NOTICE "Battery Status Monitor: volt(%d), low_thr(%d), POWER_SUPPLY_HEALTH_DEAD\n", volt, di->low_thr);
		di->health = POWER_SUPPLY_HEALTH_DEAD;
	}
	else
	{
		//printk(KERN_NOTICE "Battery Status Monitor: volt(%d), low_thr(%d), POWER_SUPPLY_HEALTH_GOOD\n", volt, di->low_thr);
		di->health = POWER_SUPPLY_HEALTH_GOOD;
	}
#endif

	//printk(KERN_NOTICE "Battery Status Monitor: volt(%d), vlot_cached(%d)\n", volt, di->volt_cached);

	//slow updates of parameters
	switch ( di->turn ) {
	case 0:
        	di->vendor_cached = bq27x00_battery_vendor(di) & 0xFE;
        	//printk(KERN_NOTICE "Battery Status Monitor: vendor(%d), turn(%d)\n", di->vendor_cached, di->turn);
        	break;
        case 1:
        	di->temp_cached = bq27x00_battery_temperature(di);
        	//printk(KERN_NOTICE "Battery Status Monitor: temp(%d), turn(%d)\n", di->temp_cached, di->turn);
        	break;        
        case 2:
               	di->rsoc_cached = bq27x00_battery_rsoc(di);
        	//printk(KERN_NOTICE "Battery Status Monitor: rsoc(%d), turn(%d)\n", di->rsoc_cached, di->turn);        	        	
        	break;
        case 3:
                di->curr_cached = bq27x00_battery_current(di);
	       	//printk(KERN_NOTICE "Battery Status Monitor: curr(%d), turn(%d)\n", di->curr_cached, di->turn);        	
        	break;
        }	
        di->turn = (di->turn > 2) ? 0 : ++di->turn;        
        
	power_supply_changed(&di->bat);
	schedule_delayed_work(&di->monitor_work, MONITOR_PERIOD);
}
#endif
//

static int bq27200_battery_probe(struct i2c_client *client,
             const struct i2c_device_id *id)
{
   char *name;
   struct bq27x00_device_info *di;
   struct bq27x00_access_methods *bus;
   int num = 0;
   int retval = 0;
   int i;

   /* Get new ID for the new battery device */
   retval = idr_pre_get(&battery_id, GFP_KERNEL);
   if (retval == 0)
      return -ENOMEM;
   mutex_lock(&battery_mutex);
   retval = idr_get_new(&battery_id, client, &num);
   mutex_unlock(&battery_mutex);
   if (retval < 0)
      return retval;

   name = kasprintf(GFP_KERNEL, "bq27200-%d", num);
   if (!name) {
      dev_err(&client->dev, "failed to allocate device name\n");
      retval = -ENOMEM;
      goto batt_failed_1;
   }
   BQ27X00_DEBUG(KERN_NOTICE  "bq27200_battery_probe: bq27x00_device_info->bat.name(%s)\n", name);

   di = kzalloc(sizeof(*di), GFP_KERNEL);
   if (!di) {
      dev_err(&client->dev, "failed to allocate device info data\n");
      retval = -ENOMEM;
      goto batt_failed_2;
   }
   di->id = num;

   bus = kzalloc(sizeof(*bus), GFP_KERNEL);
   if (!bus) {
      dev_err(&client->dev, "failed to allocate access method "
               "data\n");
      retval = -ENOMEM;
      goto batt_failed_3;
   }

   i2c_set_clientdata(client, di);
   di->dev = &client->dev;
   di->bat.name = name;
   bus->read = &bq27200_read;
   di->bus = bus;
   di->client = client;
   di->health = POWER_SUPPLY_HEALTH_GOOD;
   //init using dummy values
   di->turn = 0;   
   di->tte_cached = 65535;
   di->ttf_cached = 65535;
   di->rsoc_cached = 100;
   di->volt_cached = 4000;
   di->curr_cached = 300000;
   di->temp_cached = 25;
   di->vendor_cached = 1;
#ifdef CONFIG_BRAVO_LOWBATT_WAKEUP
   di->low_thr = g_pccmp_off_thr * 100 + 2900;
   
   bq27200_di = di;
#endif
   
#if BQ27X00_STATUS_MONITOR_UR
   INIT_DELAYED_WORK(&di->monitor_work, bq27200_battery_status_monitor); //Benny 0612
   schedule_delayed_work(&di->monitor_work, 10*HZ);
#endif
   bq27x00_powersupply_init(di);

   retval = power_supply_register(&client->dev, &di->bat);
   if (retval) {
      dev_err(&client->dev, "failed to register battery\n");
      goto batt_failed_4;
   }

   dev_info(&client->dev, "support ver. %s enabled\n", DRIVER_VERSION);
	//Susan -- the last step, as device_attr for bq27200 testing
#ifdef CONFIG_MACH_BRAVO
	for (i = 0; i < ARRAY_SIZE(bq27200_attr); i++) 
	{
		if ( (retval = device_create_file(&client->dev, bq27200_attr[i])) < 0) 
		{
			while (i--) 
			{
				 device_remove_file(&client->dev, bq27200_attr[i]);
			}
			goto batt_failed_1;
		}
	}
#endif   

   return 0;

batt_failed_4:
   kfree(bus);
batt_failed_3:
   kfree(di);
batt_failed_2:
   kfree(name);
batt_failed_1:
   mutex_lock(&battery_mutex);
   idr_remove(&battery_id, num);
   mutex_unlock(&battery_mutex);

   return retval;
}

static int bq27200_battery_remove(struct i2c_client *client)
{
   struct bq27x00_device_info *di = i2c_get_clientdata(client);

   power_supply_unregister(&di->bat);
   di->charge_status = POWER_SUPPLY_STATUS_UNKNOWN;
#if BQ27X00_STATUS_MONITOR_UR
   cancel_delayed_work(&di->monitor_work);
#endif
   kfree(di->bat.name);

   mutex_lock(&battery_mutex);
   idr_remove(&battery_id, di->id);
   mutex_unlock(&battery_mutex);

   kfree(di);

   return 0;
}

/*
 * Module stuff
 */

static const struct i2c_device_id bq27200_id[] = {
   { "bq27200", 0 },
   {},
};

static struct i2c_driver bq27200_battery_driver = {
   .driver = {
      .name = "bq27200-battery",
   },
   .probe = bq27200_battery_probe,
   .remove = bq27200_battery_remove,
   .id_table = bq27200_id,
};

static int __init bq27x00_battery_init(void)
{
   int ret;

   ret = i2c_add_driver(&bq27200_battery_driver);
   if (ret)
      printk(KERN_ERR "Unable to register BQ27200 driver\n");

   return ret;
}
module_init(bq27x00_battery_init);

static void __exit bq27x00_battery_exit(void)
{
   i2c_del_driver(&bq27200_battery_driver);
}
module_exit(bq27x00_battery_exit);

MODULE_AUTHOR("Rodolfo Giometti <giometti@linux.it>");
MODULE_DESCRIPTION("BQ27x00 battery monitor driver");
MODULE_LICENSE("GPL");
