/*
    peeknpoke.c - Peek-and-poke driver

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program; if not, write to the Free Software
    Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
*/

#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/fs.h>
#include <asm/uaccess.h>
#include <linux/cdev.h>
#include <linux/jiffies.h>
#include <linux/i2c.h>
#include <linux/err.h>

#include <asm/bitops.h>
#include <asm/io.h>
#include <plat/map-base.h>
#include <plat/regs-clock.h>
#include <plat/regs-gpio.h>
#include <plat/gpio-cfg.h>
#include <mach/hardware.h>
#include <mach/regs-mem.h>
#include <mach/regs-irq.h>
#include <asm/gpio.h>

#include <asm/mach/time.h>

#include <plat/pm.h>
#include <plat/s3c64xx-dvfs.h>
#include <plat/power-clock-domain.h>
#include <linux/platform_device.h>

#include "peeknpoke.h"

#define AIPI_BASE_ADDR 0x70000000
#define AIPI_SIZE 0x0f00ffff
#define VALID_REG(reg)  ((reg >= AIPI_BASE_ADDR) && (reg <= (AIPI_BASE_ADDR+AIPI_SIZE)))

unsigned int get_s3c_reg_val(void __iomem *reg_addr);
unsigned int set_s3c_reg_val(void __iomem *reg_addr, unsigned int regval);

//#define DEBUG_PEEKNPOKE

static const char peeknpoke_name[] = "peeknpoke";

static int peeknpoke_open(struct inode *inode, struct file *file);
static int peeknpoke_ioctl(struct inode *inode, struct file *file, unsigned int cmd, unsigned long arg);
static int peeknpoke_release(struct inode *inode, struct file *file);

static struct file_operations peeknpoke_fops =
{
    owner:      THIS_MODULE,
/* llseek:  */
/* read:  */
/* write: */
/*  readdir:    */
/*  poll:       */
    ioctl:      peeknpoke_ioctl,
/*  mmap:       */
    open:       peeknpoke_open,
/*  flush:      */
    release:    peeknpoke_release,
/*  fasync:     */
/*  lock:       */
/*  readv:      */
/*  writev:     */
};


struct peeknpoke_data {
    struct class_device *class_dev;
    struct semaphore    update_lock;
    char    valid;      /* !=0 if following fields are valid */
    u32     last_updated;   /* In jiffies */
    u16     temp_input; /* Register values */
    void __iomem *soc_base; /* Virtual addr of MPC8xxx Regs */
    void __iomem  *base;
    u32 addr;
};

static struct peeknpoke_data *data = NULL;

/***********************************************************************
 * support routines for the 'addr' file
 */
static ssize_t show_peeknpoke_addr(struct device_driver * ddp, char * buf)
{
    return snprintf(buf, PAGE_SIZE, "0x%08X\n", data->addr);
}

static ssize_t store_peeknpoke_addr(struct device_driver * ddp, const char *buf, size_t count)
{
    u32 reg;
    
    if (data->soc_base)
    {
        iounmap(data->soc_base);
	data->soc_base = NULL;
	data->base = NULL;
	data->addr = NULL;
    }

    reg = (u32)simple_strtoul(buf, NULL, 16);

    if (VALID_REG(reg))
    {
	data->addr = reg;
    	data->soc_base = ioremap(reg, PAGE_SIZE);
    	data->base = data->soc_base;
    }
    else
    {
        data->addr = NULL;
        printk("%s: Register out of range (0x%08X)\n", __FUNCTION__, reg);
    }

    return count;
}

static DRIVER_ATTR(addr, S_IRUGO | S_IWUGO, show_peeknpoke_addr, store_peeknpoke_addr);

/***********************************************************************
 * support routines for the 'reg' file
 */
static ssize_t show_peeknpoke_reg(struct device_driver *ddp, char *buf)
{
    u32 x;
    u32 reg;

    reg = data->addr;

    if (reg && data->base)
    {
         x = __raw_readl(data->base);
         return snprintf(buf, PAGE_SIZE, "[0x%08x] = 0x%08X\n", reg, x);
    }
    else
    {
         printk("%s: Register out of range (0x%08X)\n", __FUNCTION__, reg);
         return snprintf(buf, PAGE_SIZE, "[0x%08x] = 0x%08X\n", reg, 0);
    }
    return 0;
}

static ssize_t
store_peeknpoke_reg(struct device_driver *ddp, const char *buf, size_t count)
{
    u32 x;
    u32 reg = data->addr;

    if (reg && data->base)
    {
        x = (u32)simple_strtoul(buf, NULL, 16);
    	__raw_writel(x, data->base);
    }
    else
    {
        printk("%s: Register out of range (0x%08X)\n", __FUNCTION__, reg);
    }
    return count;
}

static DRIVER_ATTR(reg, S_IRUGO | S_IWUGO, show_peeknpoke_reg, store_peeknpoke_reg);

static int peeknpoke_open(struct inode *inode, struct file *file)
{
    int devnum = MINOR(inode->i_rdev);

#ifdef DEBUG_PEEKNPOKE
    printk("%s(, %d) .... \n", __FUNCTION__, devnum);
#endif
    if (devnum == 0)
    {
        return 0;
    }
    return -ENODEV;
}

static int peeknpoke_release(struct inode *inode, struct file *file)
{
    return 0;
}

static int peeknpoke_ioctl(struct inode *inode, struct file *file, unsigned int cmd, unsigned long arg)
{
        int rc = -EINVAL;
    int devnum = MINOR(inode->i_rdev);
    peeknpokeregs pnparg;


#ifdef DEBUG_PEEKNPOKE
    printk("%s: minor %d,  cmd 0x%08x\n", __FUNCTION__, devnum,  cmd);
#endif // DEBUG_PEEKNPOKE
    if (!(devnum == 0))
        return rc;

    if (copy_from_user(&pnparg, (void __user *) arg, sizeof(pnparg)))
    {
        rc = -EFAULT;
        goto exit;
    }

exit:
    return rc;
}

/* dummy function. Necessary while initializing the device_driver structure
 * for sysfs*/
static int __devinit peeknpoke_probe_dev(struct device *dev)
{
    return 0;
}

static struct device_driver peeknpoke_driver = {
    .name       = "peeknpoke",
    .bus        = NULL,
    .probe      = peeknpoke_probe_dev,
    .remove     = NULL,
    .suspend    = NULL,
    .resume     = NULL,
};



static int __init peeknpoke_init(void)
{
    int rc;

    if (!(data = kmalloc(sizeof(struct peeknpoke_data), GFP_KERNEL)))
    {
        return -ENOMEM;
    }
    memset(data, 0, sizeof(struct peeknpoke_data));
    data->valid = 0;
    init_MUTEX(&data->update_lock);

    //data->soc_base = ioremap(AIPI_BASE_ADDR, AIPI_SIZE);
    //data->base = data->soc_base;
    data->soc_base = NULL;
    data->base = NULL;

    printk("PeeknPoke: map SOC 0x%08x to 0x%08x\n", 0x44, (u32)data->soc_base);

    if ((rc = register_chrdev(DEV_PEEKNPOKE_MAJOR, peeknpoke_name, &peeknpoke_fops)) <0 )
    {
        printk(KERN_INFO "%s: unable to get major %d for Peek-n-Poke\n",
               peeknpoke_name, DEV_PEEKNPOKE_MAJOR);
        return rc;
    }

        peeknpoke_driver.bus = &platform_bus_type;
    if (driver_register(&peeknpoke_driver) < 0)
    {
        printk ("driver register to sysfs failed for Peek-n-Poke\n");
    }

    if (driver_create_file(&peeknpoke_driver, &driver_attr_addr))
        printk(KERN_INFO "%s: unable to setup sysfs addr\n", peeknpoke_name);

    if (driver_create_file(&peeknpoke_driver, &driver_attr_reg))
        printk(KERN_INFO "%s: unable to setup sysfs reg\n", peeknpoke_name);

    return 0;
}

static void __exit peeknpoke_exit(void)
{
    if (data != NULL)
    {
        if (data->soc_base)
	    iounmap(data->soc_base);
        kfree(data);
    }
    //device_remove_file(dev, &dev_attr_addr);
    unregister_chrdev(DEV_PEEKNPOKE_MAJOR, peeknpoke_name);

}

MODULE_DESCRIPTION("PeeknPoke driver");
MODULE_LICENSE("GPL");

module_init(peeknpoke_init);
module_exit(peeknpoke_exit);


