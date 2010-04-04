/* drivers/input/keyboard/synaptics_i2c_rmi.c
 *
 * Copyright (C) 2007 Google, Inc.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/module.h>
#include <linux/delay.h>
#include <linux/earlysuspend.h>
#include <linux/hrtimer.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/platform_device.h>
#include <linux/synaptics_i2c_rmi.h>
#include <mach/regs-irq.h>

#include <plat/iic.h>

#define swap(x, y) do { typeof(x) z = x; x = y; y = z; } while (0)

#define LCD_ACTUAL_X  480
#define LCD_ACTUAL_Y  (144)

#define BRAVO_Y_OFFSET_0 (BRAVO_Y_MAX - BRAVO_Y_OFFSET_DELTA) 
#define BRAVO_Y_OFFSET_DELTA (2082)
#define HOME_AREA_OFFSET (11436)
#define BRAVO_Y_MAX (13518)

#define EINK_ACTUAL_Y 800

#define BRAVO_X_FIXED_PRECISION (1000)
#define BRAVO_X_MULTIPLIER (800)

#define BUTTON_OFFSET_X (700)
#define HOME_BUTTON_X_START (3000-500) // double the width so you can use your thumb
#define HOME_BUTTON_X_END (3800+500)   // ditto

// #define CONFIG_BRAVO_BUTTON_OFFSET
#define CONFIG_BRAVO_HOME_SCREEN_EVENT

#include <asm/gpio.h>
#include <plat/gpio-cfg.h>
#include <plat/regs-gpio.h>
#include <mach/bravo_gpio.h>

static struct workqueue_struct *synaptics_wq;

struct synaptics_ts_data {
	uint16_t addr;
	struct i2c_client *client;
	struct input_dev *input_dev;
	int use_irq;
	struct hrtimer timer;
	struct work_struct  work;
	uint16_t max[2];
	int snap_state[2];
	int snap_down_on[2];
	int snap_down_off[2];
	int snap_up_on[2];
	int snap_up_off[2];
	int snap_down[2];
	int snap_up[2];
	uint32_t flags;
	int (*power)(int on);
	struct early_suspend early_suspend;
#ifdef CONFIG_MACH_BRAVO
	int int_src;
	int gesture_mask;
	int home_down;
	int home_x_start;
	int home_x_end;
	int home_height;
	int home_timeout;
	struct delayed_work home_work;
#endif /* CONFIG_MACH_BRAVO */
#ifdef CONFIG_BRAVO_BUTTON_OFFSET
	struct work_struct offset_work;
#endif /* CONFIG_BRAVO_BUTTON_OFFSET */
};

#ifdef CONFIG_HAS_EARLYSUSPEND
static void synaptics_ts_early_suspend(struct early_suspend *h);
static void synaptics_ts_late_resume(struct early_suspend *h);
#endif

#ifdef CONFIG_MACH_BRAVO
static uint8_t g_smbus_reg = 0;
#endif

#ifdef CONFIG_BRAVO_BUTTON_OFFSET
#ifdef CONFIG_FB_BRAVO
extern void bravo_vfb_set_offset(unsigned int offset);
#else
extern void s3cfb_set_offset(unsigned int offset);
#endif /* CONFIG_FB_BRAVO */

static unsigned ts_offset_area = 0;

void step_touchscreen_offset(void) 
{
	++ts_offset_area;

	if ((ts_offset_area*LCD_ACTUAL_Y) > (EINK_ACTUAL_Y+LCD_ACTUAL_Y))
		ts_offset_area = 0;

#ifdef CONFIG_FB_BRAVO
	bravo_vfb_set_offset(ts_offset_area);
#else
	s3cfb_set_offset(ts_offset_area);
#endif
	printk("%s: Touch Offset = %d\n", __FUNCTION__, ts_offset_area);
}

static void do_offset_work(struct work_struct *work)
{
	step_touchscreen_offset();
}

#endif

static int synaptics_init_panel(struct synaptics_ts_data *ts)
{
	int ret;

	ret = i2c_smbus_write_byte_data(ts->client, 0xff, 0x10); /* page select = 0x10 */
	if (ret < 0) {
		printk(KERN_ERR "i2c_smbus_write_byte_data failed for page select\n");
		goto err_page_select_failed;
	}
	ret = i2c_smbus_write_byte_data(ts->client, 0x41, 0x04); /* Set "No Clip Z" */
	if (ret < 0)
		printk(KERN_ERR "i2c_smbus_write_byte_data failed for No Clip Z\n");

	ret = i2c_smbus_write_byte_data(ts->client, 0x42, 0x10); /* Minimum press delay */

	if (ret < 0) {
		printk(KERN_ERR "i2c_smbus_write_byte_data failed for Press Time\n");
	}

err_page_select_failed:
	ret = i2c_smbus_write_byte_data(ts->client, 0xff, 0x04); /* page select = 0x04 */
	if (ret < 0)
		printk(KERN_ERR "i2c_smbus_write_byte_data failed for page select\n");
	ret = i2c_smbus_write_byte_data(ts->client, 0xf0, 0x81); /* normal operation, 80 reports per second */
	if (ret < 0)
		printk(KERN_ERR "synaptics_ts_resume: i2c_smbus_write_byte_data failed\n");
	return ret;
}

static void print_gesture(uint8_t gesture) 
{
	if (gesture) {
		printk("Gesture reg: 0x%x ", gesture); 
	}

	if (gesture & 0x20) {
		printk("Zoom - %s\n", (gesture & 0x80) ? "confirmed" : "unconfirmed");
	}

	if (gesture & 0x10) {
		printk("Flick - %s\n", (gesture & 0x80) ? "confirmed" : "unconfirmed");
	}

	if (gesture & 0x8) {
		printk("Long press - %s\n", (gesture & 0x80) ? "confirmed" : "unconfirmed");
	}

	if (gesture & 0x3) {
		printk("Double tap - %s\n", (gesture & 0x80) ? "confirmed" : "unconfirmed");
	}

	if (gesture & 0x2) {
		printk("Tap and hold - %s\n", (gesture & 0x80) ? "confirmed" : "unconfirmed");
	}

	if (gesture & 0x1) {
		printk("Tap - %s\n", (gesture & 0x80) ? "confirmed" : "unconfirmed");
	}
}

static int synaptics_handle_gesture(struct synaptics_ts_data *ts, uint8_t gesture, uint8_t magnitude)
{
	int report = 0;
	int code;
		
	if (ts->gesture_mask == SYNAPTICS_GESTURE_MASK_NONE ||
		ts->gesture_mask == SYNAPTICS_GESTURE_MASK_MOTION) {
		report = 1;
	}

	if (ts->gesture_mask & SYNAPTICS_GESTURE_MASK_TAP) {
         
	    if (gesture & 0x1) {
#ifdef GESTURE_DEBUG
		print_gesture(gesture);
#endif /* GESTURE_DEBUG */
                if (gesture & 0x80) 
                    printk("TAP confirmed wakeup LCD\n");
                else
                    printk("TAP unconfirmed wakeup LCD\n");
                input_report_key(ts->input_dev, KEY_HOME, 1);
		input_report_key(ts->input_dev, KEY_HOME, 0);
		report = 1;
            }
	}

	if ((ts->gesture_mask & SYNAPTICS_GESTURE_MASK_DOUBLE_TAP) &&
		(gesture & 0x3) &&
		(gesture & 0x80)) {
#ifdef GESTURE_DEBUG
		print_gesture(gesture);
#endif /* GESTURE_DEBUG */
		report = 1;
	}

	if ((ts->gesture_mask & SYNAPTICS_GESTURE_MASK_FLICK) &&
		(gesture & 0x10) &&
		(gesture & 0x80)) {
#ifdef GESTURE_DEBUG
		print_gesture(gesture);
#endif /* GESTURE_DEBUG */
		if (magnitude & 0xf) {
			if ((magnitude & 0xf) > 0x8) {
				code = KEY_LEFT_SWIPE;
			} else if((magnitude & 0xf) < 0x9) {
				code = KEY_RIGHT_SWIPE;
			}
	
			input_report_key(ts->input_dev, code, 1);
			input_report_key(ts->input_dev, code, 0);
			report = 0;
		} else {
			report = 1;
		}
	}

	if ((ts->gesture_mask & SYNAPTICS_GESTURE_MASK_PINCH) &&
		(gesture & 0x20) &&
		(gesture & 0x80)) {
#ifdef GESTURE_DEBUG
		print_gesture(gesture);
#endif /* GESTURE_DEBUG */
		report = 1;
	}

	return report;
}

static void synaptics_handle_abs_motion(struct synaptics_ts_data *ts, int x, int y, int z, int w, int f)
{
#ifdef CONFIG_MACH_BRAVO
	x = (x * BRAVO_X_MULTIPLIER) / BRAVO_X_FIXED_PRECISION;
#ifdef CONFIG_BRAVO_BUTTON_OFFSET
	y -= ts_offset_area * BRAVO_Y_OFFSET_DELTA;
#endif
#endif /* CONFIG_MACH_BRAVO */

	if (z) {
		input_report_abs(ts->input_dev, ABS_X, x);
		input_report_abs(ts->input_dev, ABS_Y, y);
	}

	input_report_abs(ts->input_dev, ABS_PRESSURE, z);
	input_report_abs(ts->input_dev, ABS_TOOL_WIDTH, w);
	input_report_key(ts->input_dev, BTN_TOUCH, f);
}

static void home_func(struct work_struct *work)
{
	struct delayed_work *dwork = container_of(work, struct delayed_work, work);
	struct synaptics_ts_data *ts = container_of(dwork, struct synaptics_ts_data, home_work);
	
	ts->home_down = 0;
	printk("HomeKey Up.\n\n");
}

static int synaptics_handle_home(struct synaptics_ts_data *ts, uint8_t gesture, uint8_t z)
{
	switch (gesture) {
		case 0x00: // No Gesture
		case 0x90: // Flick
		case 0xa0: // Zoom
		case 0x83: // Double Tap
		case 0x82: // Tap & Hold
			break;

		case 0x01: // Unconfirmed Tap
		case 0x81: // Tap
			if (!ts->home_down) {
				printk("Tap HomeKey Down detect: 0x%02x\n", gesture);
				input_report_key(ts->input_dev, KEY_HOME, 1);
				input_report_key(ts->input_dev, KEY_HOME, 0);
				input_sync(ts->input_dev);
				ts->home_down = 1;
				schedule_delayed_work(&ts->home_work, ts->home_timeout * 1000 * 2 / HZ);
			} else {
				// Kick timer
				int to = ts->home_timeout * 1000 * 2;
				
				if (gesture & 0x80)
					to = to / 2;
					
				cancel_delayed_work_sync(&ts->home_work);
				flush_scheduled_work();
				schedule_delayed_work(&ts->home_work, to / HZ);
			}
			return 1;
	
		case 0x88: // Press
			if (!ts->home_down) {
				printk("Press HomeKey Down detect: 0x%02x\n", gesture);
				input_report_key(ts->input_dev, KEY_HOME, 1);
				input_report_key(ts->input_dev, KEY_HOME, 0);
				input_sync(ts->input_dev);
				ts->home_down = 1;
				schedule_delayed_work(&ts->home_work, ts->home_timeout * 1000 / HZ);
			} else {
				// Kick timer
				cancel_delayed_work_sync(&ts->home_work);
				flush_scheduled_work();
				schedule_delayed_work(&ts->home_work, ts->home_timeout * 1000 / HZ);
			}
			return 1;
			
		default:
			;//printk("Missed HomeKey detect: 0x%02x\n\n", gesture);
	}
	return 0;
}

static void synaptics_ts_work_func(struct work_struct *work)
{
	int i, ret;
	struct i2c_msg msg[2];
	uint8_t start_reg;
	uint8_t buf[14];
	
	/* First read in the data source */
	struct synaptics_ts_data *ts = container_of(work, struct synaptics_ts_data, work);
	
	msg[0].addr = ts->client->addr;
	msg[0].flags = 0;
	msg[0].len = 1;
	msg[0].buf = &start_reg;
	start_reg = 0x00;
	msg[1].addr = ts->client->addr;
	msg[1].flags = I2C_M_RD;
	msg[1].len = sizeof(buf);
	msg[1].buf = buf;

	for (i = 0; i < (ts->use_irq ? 1 :10); ++i) {
		int pos[2];
		int a;
		int base = 2;
		int z;
		int w;
		int finger;
		int gesture_report;
		uint32_t flip_flag = SYNAPTICS_FLIP_X;

		ret = i2c_transfer(ts->client->adapter, msg, 2);

		if (ret < 0) {
			dev_err(&ts->client->dev, "%s i2c_tranfer_failed; %d", __FUNCTION__, ret);
			goto out;
		}

		z = buf[1];
		w = buf[0] >> 4;
		finger = buf[0] & 0x7;

		for (a = 0; a < 2; a++) {
			int p = buf[base + 1];
			p |= (uint16_t)(buf[base] & 0x1f) << 8;
			if (ts->flags & flip_flag)
				p = ts->max[a] - p;
			if (ts->flags & SYNAPTICS_SNAP_TO_INACTIVE_EDGE) {
				if (ts->snap_state[a]) {
					if (p <= ts->snap_down_off[a])
						p = ts->snap_down[a];
					else if (p >= ts->snap_up_off[a])
						p = ts->snap_up[a];
					else
						ts->snap_state[a] = 0;
				} else {
					if (p <= ts->snap_down_on[a]) {
						p = ts->snap_down[a];
						ts->snap_state[a] = 1;
					} else if (p >= ts->snap_up_on[a]) {
						p = ts->snap_up[a];
						ts->snap_state[a] = 1;
					}
				}
			}

			pos[a] = p;
			base += 2;
			flip_flag <<= 1;
		}
			
		if (ts->flags & SYNAPTICS_SWAP_XY)
			swap(pos[0], pos[1]);

#ifdef CONFIG_BRAVO_BUTTON_OFFSET
			if (z && pos[1] < HOME_AREA_OFFSET && 
				(buf[12] & 0x80) && (buf[12] & 0x1)
				&& pos[0] < BUTTON_OFFSET_X) {
				if (!work_pending(&(ts->offset_work))) {
					schedule_work(&(ts->offset_work));
				}
				goto out;
			} 
#endif /* CONFIG_BRAVO_BUTTON_OFFSET */

#ifdef CONFIG_BRAVO_HOME_SCREEN_EVENT
			
		if (pos[1] < ts->home_height &&
			  pos[0] > ts->home_x_start && 
			  pos[0] < ts->home_x_end) {
			//printk("Raw Gesture(%d,%d,%d) = 0x%02x\n", pos[0], pos[1], z, buf[12]);
			if (synaptics_handle_home(ts, buf[12], z)) {
				goto out;
			}
		}

#endif /* CONFIG_BRAVO_HOME_SCREEN_EVENT */

		gesture_report = synaptics_handle_gesture(ts, buf[12], buf[13]);
		
		if (gesture_report && 
		    ts->gesture_mask & SYNAPTICS_GESTURE_MASK_MOTION) {
		 
		  if( pos[1] > HOME_AREA_OFFSET ) {
		    synaptics_handle_abs_motion(ts, pos[0], pos[1], z, w, finger);
		  } else {
		    synaptics_handle_abs_motion(ts, pos[0], pos[1], 0, w, finger);
		  }
		}
		else if (gesture_report && (ts->gesture_mask & SYNAPTICS_GESTURE_MASK_TAP))
		{
			printk("wake up LCD with TAP\n");
		}
		input_sync(ts->input_dev);
	}

out:
	if (ts->use_irq)
		enable_irq(ts->client->irq);
}

static enum hrtimer_restart synaptics_ts_timer_func(struct hrtimer *timer)
{
	struct synaptics_ts_data *ts = container_of(timer, struct synaptics_ts_data, timer);

	queue_work(synaptics_wq, &ts->work);

	hrtimer_start(&ts->timer, ktime_set(0, 12500000), HRTIMER_MODE_REL);
	return HRTIMER_NORESTART;
}

static irqreturn_t synaptics_ts_irq_handler(int irq, void *dev_id)
{
	struct synaptics_ts_data *ts = dev_id;

	disable_irq(ts->client->irq);
	queue_work(synaptics_wq, &ts->work);
	return IRQ_HANDLED;
}

static int synaptics_set_int_source(struct i2c_client *client, int source) 
{
	return i2c_smbus_write_byte_data(client, 0xf1, source | 2);
}

static ssize_t synaptics_ts_show_interrupt_src(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct synaptics_ts_data *ts = i2c_get_clientdata(client);

	return sprintf(buf, "%u\n", ts->int_src);
}

static ssize_t synaptics_ts_store_interrupt_src(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	int ret;
	unsigned long int_src;
	struct i2c_client *client = to_i2c_client(dev);
	struct synaptics_ts_data *ts = i2c_get_clientdata(client);

	ret = strict_strtoul(buf, 10, &int_src);
	
	if (ret) {
		dev_err(dev, "Invalid parameter for interrupt source: %s\n", buf);
		goto out;
	}

	switch (int_src) {
		case SYNAPTICS_INT_SOURCE_GESTURE:
			ret = synaptics_set_int_source(client, 0x2);
			break;

		case SYNAPTICS_INT_SOURCE_ABS_MOTION:
			ret = synaptics_set_int_source(client, 0x3);
			break;

		case SYNAPTICS_INT_SOURCE_REL_MOTION:
			ret = synaptics_set_int_source(client, 0x0);
			break;

		default:
			dev_err(dev, "Invalid interrupt source: %s\n", buf);
			goto out;
	}

	if (ret) {
		goto out;
	}

	ts->int_src = int_src;
	count = 0;
out:
	return count;
}

static ssize_t synaptics_ts_show_gesture_mask(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct synaptics_ts_data *ts = i2c_get_clientdata(client);

	return sprintf(buf, "%u\n", ts->gesture_mask);
}

static ssize_t synaptics_ts_store_gesture_mask(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	unsigned long gesture_mask;
	struct i2c_client *client = to_i2c_client(dev);
	struct synaptics_ts_data *ts = i2c_get_clientdata(client);
	
	int ret = strict_strtoul(buf, 10, &gesture_mask);
	
	if (ret || gesture_mask > SYNAPTICS_GESTURE_MASK_ANY) {
		dev_err(dev, "Invalid parameter for gesture mask: %s\n", buf);
		goto out;
	}

	ts->gesture_mask = gesture_mask;
	count = 0;
out:
	return count;
}

static ssize_t synaptics_ts_store_home_width(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	struct synaptics_ts_data *ts = i2c_get_clientdata(to_i2c_client(dev));
	unsigned long width;

	int ret = strict_strtoul(buf, 10, &width);

	if (ret || width > ts->max[0] || width < 0) {
		dev_err(dev, "Invalid parameter for home area width: %s\n", buf);
		goto out;
	}

	ts->home_x_start = (ts->max[0] / 2) - (width / 2);
	ts->home_x_end = (ts->max[0] / 2) + (width / 2); 
	count = 0;
out:
	return count;
}

static ssize_t synaptics_ts_show_home_width(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct synaptics_ts_data *ts = i2c_get_clientdata(to_i2c_client(dev));
	return sprintf(buf, "%u\n", ts->home_x_end - ts->home_x_start);
}

static ssize_t synaptics_ts_store_home_height(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	struct synaptics_ts_data *ts = i2c_get_clientdata(to_i2c_client(dev));
	unsigned long height;

	int ret = strict_strtoul(buf, 10, &height);

	if (ret || height > ts->max[1] || height < 0) {
		dev_err(dev, "Invalid parameter for home area height: %s\n", buf);
		goto out;
	}

	ts->home_height = height;
	count = 0;
out:
	return count;
}

static ssize_t synaptics_ts_show_home_height(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct synaptics_ts_data *ts = i2c_get_clientdata(to_i2c_client(dev));
	return sprintf(buf, "%u\n", ts->home_height);
}

static ssize_t synaptics_ts_store_home_timeout(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	struct synaptics_ts_data *ts = i2c_get_clientdata(to_i2c_client(dev));
	unsigned long timeout;

	int ret = strict_strtoul(buf, 10, &timeout);

	if (ret || timeout > 0) {
		dev_err(dev, "Invalid parameter for home area timeout: %s\n", buf);
		goto out;
	}

	ts->home_timeout = timeout;
	count = 0;
out:
	return count;
}

#ifdef CONFIG_MACH_BRAVO
static ssize_t synaptics_ts_show_smbus_reg(struct device *dev, struct device_attribute *attr, char *buf)
{
	//struct synaptics_ts_data *ts = i2c_get_clientdata(to_i2c_client(dev));
	return sprintf(buf, "0x%x\n", g_smbus_reg);
}

static ssize_t synaptics_ts_store_smbus_reg(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	//struct synaptics_ts_data *ts = i2c_get_clientdata(to_i2c_client(dev));
	unsigned long smbus_reg;

	smbus_reg = simple_strtoul(buf, NULL, 16);

	if ( smbus_reg > 0xFF ) {
		dev_err(dev, "Invalid SMBus register #: %s\n", buf);
		goto out;
	}

	g_smbus_reg = smbus_reg;
	
	count = 0;
out:
	return count;
}

static ssize_t synaptics_ts_show_smbus_val(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct synaptics_ts_data *ts = i2c_get_clientdata(to_i2c_client(dev));
	int ret = 0;
	
	ret = i2c_smbus_read_byte_data(ts->client, g_smbus_reg);
	if (ret < 0) {
		printk(KERN_ERR "i2c_smbus_read_byte_data failed\n");
		return sprintf(buf, "0\n");
	}	
	return sprintf(buf, "0x%x\n", ret);
}

static ssize_t synaptics_ts_store_smbus_val(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	struct synaptics_ts_data *ts = i2c_get_clientdata(to_i2c_client(dev));
	unsigned long value;
	int ret;

	value = simple_strtoul(buf, NULL, 16);

	if ( value > 0xFF ) {
		dev_err(dev, "Invalid value: %s for reg: %u\n", buf, g_smbus_reg);
		goto out;
	}

	ret = i2c_smbus_write_byte_data(ts->client, g_smbus_reg, value); /* device command = reset */
	if (ret < 0) {
		printk(KERN_ERR "i2c_smbus_write_byte_data failed\n");
	}
	
	count = 0;
out:
	return count;
}
#endif

static ssize_t synaptics_ts_show_home_timeout(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct synaptics_ts_data *ts = i2c_get_clientdata(to_i2c_client(dev));
	return sprintf(buf, "%u\n", ts->home_timeout);
}

static DEVICE_ATTR(interrupt_src, 0666, synaptics_ts_show_interrupt_src, synaptics_ts_store_interrupt_src);
static DEVICE_ATTR(gesture_mask, 0666, synaptics_ts_show_gesture_mask, synaptics_ts_store_gesture_mask);
static DEVICE_ATTR(home_width, 0666, synaptics_ts_show_home_width, synaptics_ts_store_home_width);
static DEVICE_ATTR(home_height, 0666, synaptics_ts_show_home_height, synaptics_ts_store_home_height);
static DEVICE_ATTR(home_timeout, 0666, synaptics_ts_show_home_timeout, synaptics_ts_store_home_timeout);
#ifdef CONFIG_MACH_BRAVO
static DEVICE_ATTR(smbus_reg, 0666, synaptics_ts_show_smbus_reg, synaptics_ts_store_smbus_reg);
static DEVICE_ATTR(smbus_val, 0666, synaptics_ts_show_smbus_val, synaptics_ts_store_smbus_val);
#endif

static int register_device_attrs(struct device *dev) 
{
	int ret = device_create_file(dev, &dev_attr_interrupt_src);

	if (ret < 0) {
		goto err1;
	}

	ret = device_create_file(dev, &dev_attr_gesture_mask);

	if (ret < 0) {
		goto err2;
	}

	ret = device_create_file(dev, &dev_attr_home_width);

	if (ret < 0) {
		goto err3;
	}

	ret = device_create_file(dev, &dev_attr_home_height);

	if (ret < 0) {
		goto err4;
	}

	ret = device_create_file(dev, &dev_attr_home_timeout);

	if (ret < 0) {
		goto err5;
	}
#ifdef CONFIG_MACH_BRAVO
	ret = device_create_file(dev, &dev_attr_smbus_reg);

	if (ret < 0) {
		goto err6;
	}	

	ret = device_create_file(dev, &dev_attr_smbus_val);

	if (ret < 0) {
		goto err7;
	}	
#endif	
	return ret;

#ifdef CONFIG_MACH_BRAVO
err7:
	device_remove_file(dev, &dev_attr_smbus_val);
err6:
	device_remove_file(dev, &dev_attr_smbus_reg);	
#endif
err5:
	device_remove_file(dev, &dev_attr_home_height);
err4:
	device_remove_file(dev, &dev_attr_home_width);
err3:
	device_remove_file(dev, &dev_attr_gesture_mask);
err2:
	device_remove_file(dev, &dev_attr_interrupt_src);
err1:
	return ret;
}

static int synaptics_ts_probe(
	struct i2c_client *client, const struct i2c_device_id *id)
{
	struct synaptics_ts_data *ts;
	uint8_t buf0[4];
	uint8_t buf1[8];
	struct i2c_msg msg[2];
	int ret = 0;
	uint16_t max_x, max_y;
	int fuzz_x, fuzz_y, fuzz_p, fuzz_w;
	struct synaptics_i2c_rmi_platform_data *pdata;
	int inactive_area_left;
	int inactive_area_right;
	int inactive_area_top;
	int inactive_area_bottom;
	int snap_left_on;
	int snap_left_off;
	int snap_right_on;
	int snap_right_off;
	int snap_top_on;
	int snap_top_off;
	int snap_bottom_on;
	int snap_bottom_off;
	uint32_t panel_version;

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		printk(KERN_ERR "synaptics_ts_probe: need I2C_FUNC_I2C\n");
		ret = -ENODEV;
		goto err_check_functionality_failed;
	}

	ts = kzalloc(sizeof(*ts), GFP_KERNEL);
	if (ts == NULL) {
		ret = -ENOMEM;
		goto err_alloc_data_failed;
	}

	INIT_WORK(&ts->work, synaptics_ts_work_func);
	ts->client = client;
	i2c_set_clientdata(client, ts);
	pdata = ((struct s3c_i2c_client_platdata *) client->dev.platform_data)->client_owner_platdata;

	if (pdata)
		ts->power = pdata->power;
	if (ts->power) {
		ret = ts->power(1);
		if (ret < 0) {
			printk(KERN_ERR "synaptics_ts_probe power on failed\n");
			goto err_power_failed;
		}
	}

	ret = i2c_smbus_write_byte_data(ts->client, 0xf4, 0x01); /* device command = reset */
	if (ret < 0) {
		printk(KERN_ERR "i2c_smbus_write_byte_data failed\n");
		/* fail? */
	}
	{
		int retry = 10;
		while (retry-- > 0) {
			ret = i2c_smbus_read_byte_data(ts->client, 0xe4);
			if (ret >= 0)
				break;
			msleep(100);
		}
	}

	if (ret < 0) {
		printk(KERN_ERR "i2c_smbus_read_byte_data failed\n");
		goto err_detect_failed;
	}
	printk(KERN_INFO "synaptics_ts_probe: Product Major Version %x\n", ret);
	panel_version = ret << 8;
	ret = i2c_smbus_read_byte_data(ts->client, 0xe5);
	if (ret < 0) {
		printk(KERN_ERR "i2c_smbus_read_byte_data failed\n");
		goto err_detect_failed;
	}

	printk(KERN_INFO "synaptics_ts_probe: Product Minor Version %x\n", ret);
	panel_version |= ret;

	ret = i2c_smbus_read_byte_data(ts->client, 0xe3);
	if (ret < 0) {
		printk(KERN_ERR "i2c_smbus_read_byte_data failed\n");
		goto err_detect_failed;
	}
	printk(KERN_INFO "synaptics_ts_probe: product property %x\n", ret);

	if (pdata) {
		while (pdata->version > panel_version)
			pdata++;
		ts->flags = pdata->flags;
		inactive_area_left = pdata->inactive_left;
		inactive_area_right = pdata->inactive_right;
		inactive_area_top = pdata->inactive_top;
		inactive_area_bottom = pdata->inactive_bottom;
		snap_left_on = pdata->snap_left_on;
		snap_left_off = pdata->snap_left_off;
		snap_right_on = pdata->snap_right_on;
		snap_right_off = pdata->snap_right_off;
		snap_top_on = pdata->snap_top_on;
		snap_top_off = pdata->snap_top_off;
		snap_bottom_on = pdata->snap_bottom_on;
		snap_bottom_off = pdata->snap_bottom_off;
		fuzz_x = pdata->fuzz_x;
		fuzz_y = pdata->fuzz_y;
		fuzz_p = pdata->fuzz_p;
		fuzz_w = pdata->fuzz_w;
	} else {
		inactive_area_left = 0;
		inactive_area_right = 0;
		inactive_area_top = 0;
		inactive_area_bottom = 0;
		snap_left_on = 0;
		snap_left_off = 0;
		snap_right_on = 0;
		snap_right_off = 0;
		snap_top_on = 0;
		snap_top_off = 0;
		snap_bottom_on = 0;
		snap_bottom_off = 0;
		fuzz_x = 0;
		fuzz_y = 0;
		fuzz_p = 0;
		fuzz_w = 0;
	}

	ret = i2c_smbus_read_byte_data(ts->client, 0xf0);
	if (ret < 0) {
		printk(KERN_ERR "i2c_smbus_read_byte_data failed\n");
		goto err_detect_failed;
	}
	printk(KERN_INFO "synaptics_ts_probe: device control %x\n", ret);

	ret = i2c_smbus_read_byte_data(ts->client, 0xf1);
	if (ret < 0) {
		printk(KERN_ERR "i2c_smbus_read_byte_data failed\n");
		goto err_detect_failed;
	}
	printk(KERN_INFO "synaptics_ts_probe: interrupt enable %x\n", ret);

	ret = i2c_smbus_write_byte_data(ts->client, 0xf1, 0); /* disable interrupt */
	if (ret < 0) {
		printk(KERN_ERR "i2c_smbus_write_byte_data failed\n");
		goto err_detect_failed;
	}

	msg[0].addr = ts->client->addr;
	msg[0].flags = 0;
	msg[0].len = 1;
	msg[0].buf = buf0;
	buf0[0] = 0xe0;
	msg[1].addr = ts->client->addr;
	msg[1].flags = I2C_M_RD;
	msg[1].len = 8;
	msg[1].buf = buf1;
	ret = i2c_transfer(ts->client->adapter, msg, 2);
	if (ret < 0) {
		printk(KERN_ERR "i2c_transfer failed\n");
		goto err_detect_failed;
	}
	printk(KERN_INFO "synaptics_ts_probe: 0xe0: %x %x %x %x %x %x %x %x\n",
	       buf1[0], buf1[1], buf1[2], buf1[3],
	       buf1[4], buf1[5], buf1[6], buf1[7]);

	ret = i2c_smbus_write_byte_data(ts->client, 0xff, 0x10); /* page select = 0x10 */
	if (ret < 0) {
		printk(KERN_ERR "i2c_smbus_write_byte_data failed for page select\n");
		goto err_detect_failed;
	}

	ret = i2c_smbus_read_word_data(ts->client, 0x04);
	if (ret < 0) {
		printk(KERN_ERR "i2c_smbus_read_word_data failed\n");
		goto err_detect_failed;
	}
	ts->max[0] = max_x = (ret >> 8 & 0xff) | ((ret & 0x1f) << 8);
	ret = i2c_smbus_read_word_data(ts->client, 0x06);
	if (ret < 0) {
		printk(KERN_ERR "i2c_smbus_read_word_data failed\n");
		goto err_detect_failed;
	}
	ts->max[1] = max_y = (ret >> 8 & 0xff) | ((ret & 0x1f) << 8);
	if (ts->flags & SYNAPTICS_SWAP_XY)
		swap(max_x, max_y);

	ret = synaptics_init_panel(ts); /* will also switch back to page 0x04 */
	if (ret < 0) {
		printk(KERN_ERR "synaptics_init_panel failed\n");
		goto err_detect_failed;
	}

	ts->input_dev = input_allocate_device();
	if (ts->input_dev == NULL) {
		ret = -ENOMEM;
		printk(KERN_ERR "synaptics_ts_probe: Failed to allocate input device\n");
		goto err_input_dev_alloc_failed;
	}
	ts->input_dev->name = "synaptics-rmi-touchscreen";
	set_bit(EV_SYN, ts->input_dev->evbit);
	set_bit(EV_KEY, ts->input_dev->evbit);
	set_bit(BTN_TOUCH, ts->input_dev->keybit);
	set_bit(BTN_2, ts->input_dev->keybit);
#ifdef CONFIG_BRAVO_HOME_SCREEN_EVENT
	set_bit(KEY_HOME, ts->input_dev->keybit);
	set_bit(KEY_LEFT_SWIPE, ts->input_dev->keybit);
	set_bit(KEY_RIGHT_SWIPE, ts->input_dev->keybit);
#endif
#ifdef CONFIG_MACH_BRAVO_EVT15
	ts->max[1] = max_y = BRAVO_Y_MAX;
#endif
	set_bit(EV_ABS, ts->input_dev->evbit);
	inactive_area_left = inactive_area_left * max_x / 0x10000;
	inactive_area_right = inactive_area_right * max_x / 0x10000;
	inactive_area_top = inactive_area_top * max_y / 0x10000;
	inactive_area_bottom = inactive_area_bottom * max_y / 0x10000;
	snap_left_on = snap_left_on * max_x / 0x10000;
	snap_left_off = snap_left_off * max_x / 0x10000;
	snap_right_on = snap_right_on * max_x / 0x10000;
	snap_right_off = snap_right_off * max_x / 0x10000;
	snap_top_on = snap_top_on * max_y / 0x10000;
	snap_top_off = snap_top_off * max_y / 0x10000;
	snap_bottom_on = snap_bottom_on * max_y / 0x10000;
	snap_bottom_off = snap_bottom_off * max_y / 0x10000;
	fuzz_x = fuzz_x * max_x / 0x10000;
	fuzz_y = fuzz_y * max_y / 0x10000;
	ts->snap_down[!!(ts->flags & SYNAPTICS_SWAP_XY)] = -inactive_area_left;
	ts->snap_up[!!(ts->flags & SYNAPTICS_SWAP_XY)] = max_x + inactive_area_right;
	ts->snap_down[!(ts->flags & SYNAPTICS_SWAP_XY)] = -inactive_area_top;
	ts->snap_up[!(ts->flags & SYNAPTICS_SWAP_XY)] = max_y + inactive_area_bottom;
	ts->snap_down_on[!!(ts->flags & SYNAPTICS_SWAP_XY)] = snap_left_on;
	ts->snap_down_off[!!(ts->flags & SYNAPTICS_SWAP_XY)] = snap_left_off;
	ts->snap_up_on[!!(ts->flags & SYNAPTICS_SWAP_XY)] = max_x - snap_right_on;
	ts->snap_up_off[!!(ts->flags & SYNAPTICS_SWAP_XY)] = max_x - snap_right_off;
	ts->snap_down_on[!(ts->flags & SYNAPTICS_SWAP_XY)] = snap_top_on;
	ts->snap_down_off[!(ts->flags & SYNAPTICS_SWAP_XY)] = snap_top_off;
	ts->snap_up_on[!(ts->flags & SYNAPTICS_SWAP_XY)] = max_y - snap_bottom_on;
	ts->snap_up_off[!(ts->flags & SYNAPTICS_SWAP_XY)] = max_y - snap_bottom_off;
	printk(KERN_INFO "synaptics_ts_probe: max_x %d, max_y %d\n", max_x, max_y);
	printk(KERN_INFO "synaptics_ts_probe: inactive_x %d %d, inactive_y %d %d\n",
	       inactive_area_left, inactive_area_right,
	       inactive_area_top, inactive_area_bottom);
	printk(KERN_INFO "synaptics_ts_probe: snap_x %d-%d %d-%d, snap_y %d-%d %d-%d\n",
	       snap_left_on, snap_left_off, snap_right_on, snap_right_off,
	       snap_top_on, snap_top_off, snap_bottom_on, snap_bottom_off);

#ifdef CONFIG_MACH_BRAVO
	INIT_DELAYED_WORK(&ts->home_work, home_func);
#endif /* CONFIG_MACH_BRAVO */

	input_set_abs_params(ts->input_dev, ABS_X, -inactive_area_left, max_x + inactive_area_right, fuzz_x, 0);
	input_set_abs_params(ts->input_dev, ABS_Y, -inactive_area_top, max_y + inactive_area_bottom, fuzz_y, 0);
	input_set_abs_params(ts->input_dev, ABS_PRESSURE, 0, 255, fuzz_p, 0);
	input_set_abs_params(ts->input_dev, ABS_TOOL_WIDTH, 0, 15, fuzz_w, 0);
	input_set_abs_params(ts->input_dev, ABS_HAT0X, -inactive_area_left, max_x + inactive_area_right, fuzz_x, 0);
	input_set_abs_params(ts->input_dev, ABS_HAT0Y, -inactive_area_top, max_y + inactive_area_bottom, fuzz_y, 0);
	/* ts->input_dev->name = ts->keypad_info->name; */
	ret = input_register_device(ts->input_dev);
	if (ret) {
		printk(KERN_ERR "synaptics_ts_probe: Unable to register %s input device\n", ts->input_dev->name);
		goto err_input_register_device_failed;
	}

#ifdef CONFIG_MACH_BRAVO
	ts->home_x_start = HOME_BUTTON_X_START;
	ts->home_x_end = HOME_BUTTON_X_END;
	ts->home_height = HOME_AREA_OFFSET;
	ts->home_timeout = 10;

	ret = register_device_attrs(&ts->client->dev);
#endif /* CONFIG_MACH_BRAVO */

	if (ret) {
		dev_err(&ts->client->dev, "failed to register device attributes: %d\n", ret);
		goto err_device_create_file_failed;
	}

	if (client->irq) {
		ret = request_irq(client->irq, synaptics_ts_irq_handler, IRQF_TRIGGER_LOW, client->name, ts);

		if (ret == 0) {
			printk("Enabling irq: %d\n", client->irq);
			enable_irq_wake(client->irq);
			ret = i2c_smbus_write_byte_data(ts->client, 0xf1, 0x03); /* enable abs int */
			if (ret)
				free_irq(client->irq, ts);
		}
		if (ret == 0)
			ts->use_irq = 1;
		else
			dev_err(&client->dev, "request_irq failed\n");
	}

	if (!ts->use_irq) {
		hrtimer_init(&ts->timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
		ts->timer.function = synaptics_ts_timer_func;
		hrtimer_start(&ts->timer, ktime_set(1, 0), HRTIMER_MODE_REL);
	}
#ifdef CONFIG_HAS_EARLYSUSPEND
	ts->early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1;
	ts->early_suspend.suspend = synaptics_ts_early_suspend;
	ts->early_suspend.resume = synaptics_ts_late_resume;
	register_early_suspend(&ts->early_suspend);
#endif

#ifdef CONFIG_BRAVO_BUTTON_OFFSET
	ts->int_src = SYNAPTICS_INT_SOURCE_ABS_MOTION;
	ts->gesture_mask = SYNAPTICS_GESTURE_MASK_MOTION;
	INIT_WORK(&(ts->offset_work), do_offset_work);
#endif /* CONFIG_BRAVO_BUTTON_OFFSET */

	printk(KERN_INFO "synaptics_ts_probe: Start touchscreen %s in %s mode\n", ts->input_dev->name, ts->use_irq ? "interrupt" : "polling");
	return 0;
	
err_device_create_file_failed:
	input_unregister_device(ts->input_dev);
err_input_register_device_failed:
	input_free_device(ts->input_dev);
err_input_dev_alloc_failed:
err_detect_failed:
err_power_failed:
	kfree(ts);
err_alloc_data_failed:
err_check_functionality_failed:
	return ret;
}

static int synaptics_ts_remove(struct i2c_client *client)
{
	struct synaptics_ts_data *ts = i2c_get_clientdata(client);
	unregister_early_suspend(&ts->early_suspend);
	if (ts->use_irq || client->irq > 0)
		free_irq(client->irq, ts);
	else 
		hrtimer_cancel(&ts->timer);
	input_unregister_device(ts->input_dev);
	kfree(ts);
	return 0;
}


static int synaptics_ts_suspend(struct i2c_client *client, pm_message_t mesg)
{
	int ret;
	struct synaptics_ts_data *ts = i2c_get_clientdata(client);

	if (ts->use_irq) { 
		disable_irq(client->irq);
	}
	else 
		hrtimer_cancel(&ts->timer);

	ret = cancel_work_sync(&ts->work);
	if (ret && ts->use_irq) /* if work was pending disable-count is now 2 */
		enable_irq(client->irq);
	ret = i2c_smbus_write_byte_data(ts->client, 0xf1, 0); /* disable interrupt */
	if (ret < 0)
		printk(KERN_ERR "synaptics_ts_suspend: i2c_smbus_write_byte_data failed\n");

	ret = i2c_smbus_write_byte_data(client, 0xf0, 0x86); /* deep sleep */
	if (ret < 0)
		printk(KERN_ERR "synaptics_ts_suspend: i2c_smbus_write_byte_data failed\n");
	if (ts->power) {
		ret = ts->power(0);
		if (ret < 0)
			printk(KERN_ERR "synaptics_ts_resume power off failed\n");
	}
	return 0;
}

static int synaptics_ts_resume(struct i2c_client *client)
{
	int ret;
	struct synaptics_ts_data *ts = i2c_get_clientdata(client);

	if (ts->power) {
		ret = ts->power(1);
		if (ret < 0)
			printk(KERN_ERR "synaptics_ts_resume power on failed\n");
	}

	synaptics_init_panel(ts);

	if (ts->use_irq) {
		enable_irq(client->irq);
	}
	
	if (!ts->use_irq) { 
		hrtimer_start(&ts->timer, ktime_set(1, 0), HRTIMER_MODE_REL);
	} 

	else { 
		ret = i2c_smbus_write_byte_data(ts->client, 0xf1, 0x03); /* enable abs int */
		
		if (ret < 0) {
			printk(KERN_ERR "Failed to enable abs interrupt: %d\n", ret);
		}
	}
	
	return 0;
}

#ifdef CONFIG_HAS_EARLYSUSPEND
static void synaptics_ts_early_suspend(struct early_suspend *h)
{
	struct synaptics_ts_data *ts;
	ts = container_of(h, struct synaptics_ts_data, early_suspend);
	synaptics_ts_suspend(ts->client, PMSG_SUSPEND);
}

static void synaptics_ts_late_resume(struct early_suspend *h)
{
	struct synaptics_ts_data *ts;
	ts = container_of(h, struct synaptics_ts_data, early_suspend);
	synaptics_ts_resume(ts->client);
}
#endif

static const struct i2c_device_id synaptics_ts_id[] = {
	{ SYNAPTICS_I2C_RMI_NAME, 0 },
	{ }
};

static struct i2c_driver synaptics_ts_driver = {
	.probe		= synaptics_ts_probe,
	.remove		= synaptics_ts_remove,
#ifndef CONFIG_HAS_EARLYSUSPEND
	.suspend	= synaptics_ts_suspend,
	.resume		= synaptics_ts_resume,
#endif
	.id_table	= synaptics_ts_id,
	.driver = {
		.name	= SYNAPTICS_I2C_RMI_NAME,
	},
};

static int __devinit synaptics_ts_init(void)
{
	synaptics_wq = create_singlethread_workqueue("synaptics_wq");
	if (!synaptics_wq)
		return -ENOMEM;
	return i2c_add_driver(&synaptics_ts_driver);
}

static void __exit synaptics_ts_exit(void)
{
	i2c_del_driver(&synaptics_ts_driver);
	if (synaptics_wq)
		destroy_workqueue(synaptics_wq);
}

module_init(synaptics_ts_init);
module_exit(synaptics_ts_exit);

MODULE_DESCRIPTION("Synaptics Touchscreen Driver");
MODULE_LICENSE("GPL");
