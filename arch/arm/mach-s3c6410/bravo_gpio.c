/* linux/arch/arm/mach-s3c6410/bravo_gpio.c
 *
 * Copyright (c) 2009 Barnes and Noble Inc.
 * All rights reserved.
 *
 * Module author: sgu@intrinsyc.com, Intrinsyc Software, Inc
 *
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
*/


#include <linux/kernel.h>
#include <linux/sched.h>
#include <linux/types.h>
#include <linux/mm.h>
#include <linux/vmalloc.h>
#include <linux/proc_fs.h>
#include <linux/delay.h>
#include <asm/io.h>
#include <mach/bravo_gpio.h>
#include <plat/irqs.h>

unsigned int get_s3c_reg_val(void __iomem *reg_addr)
{
	unsigned int regval;
	int flags;
	local_irq_save(flags);
	regval = __raw_readl(reg_addr);
	local_irq_restore(flags);
	return regval;
}
EXPORT_SYMBOL(get_s3c_reg_val);

unsigned int set_s3c_reg_val(void __iomem *reg_addr, unsigned int regval)
{
	unsigned int regval_0;
	int flags;
	local_irq_save(flags);
	__raw_writel(regval, reg_addr);
	regval_0 = __raw_readl(reg_addr);
	local_irq_restore(flags);
}
EXPORT_SYMBOL(set_s3c_reg_val);

//New WIFI RESET Sequence
void s3c6410_enable_wifi_pins(void)
{
	int flags;
	u32 regval = 0x0;

    // Configure as WF_SD_CMD and WF_SD_CLK as SD
    regval = get_s3c_reg_val(S3C64XX_GPCCON);
	regval &= ~(0xf << 16);
	regval &= ~(0xf << 20);
	regval |= (0x3 << 16);
	regval |= (0x3 << 20);
	set_s3c_reg_val(S3C64XX_GPCCON, regval);

	// Configure WF_SD_D0 to WF_SD_D3 as SD
	 
	// configure GPH6 and GPH7 as SD
	regval = get_s3c_reg_val(S3C64XX_GPHCON0);
	regval &= ~(0xf << 24);
	regval &= ~(0xf << 28);
	regval |= (0x3 << 24);
	regval |= (0x3 << 28);
	set_s3c_reg_val(S3C64XX_GPHCON0, regval);

	// configure GPH8 and GPH9 as SD
	regval = get_s3c_reg_val(S3C64XX_GPHCON1);
	regval &= ~(0xf);
	regval &= ~(0xf << 4);
	regval |= (0x3);
	regval |= (0x3 << 4);
	set_s3c_reg_val(S3C64XX_GPHCON1, regval);

	//Configure GPN3 (WF_PWR_DNn) as output/pulldown
	regval = get_s3c_reg_val(S3C64XX_GPNPUD);
	regval &= ~(0x3<<6); //GPN3
	regval |= (0x1<<6); //Pulldown GPN3 -- avoid floating configuration for INPUT GPIO pin
	set_s3c_reg_val(S3C64XX_GPNPUD, regval);

	regval = get_s3c_reg_val(S3C64XX_GPNDAT);
	regval &= ~(1<<3);  //LOW for GPN3
	set_s3c_reg_val(S3C64XX_GPNDAT, regval);

	regval = get_s3c_reg_val(S3C64XX_GPNCON);
	regval &= ~(0x3<<6);
	regval |= (0x1<<6); //OUTPUT
	set_s3c_reg_val(S3C64XX_GPNCON, regval);	

	mdelay(1);

	//Configure WF_MAC_WAKE(GPL8) and WF_HOST_WAKE(GPL9)
	regval = get_s3c_reg_val(S3C64XX_GPLPUD);
	regval &= ~(0x3 << 16);
	regval |= (2<<16); //Just pullup resistor on GPL8
	set_s3c_reg_val(S3C64XX_GPLPUD, regval);

	regval = get_s3c_reg_val(	S3C64XX_GPLDAT);
	regval |= (1<<8); //HIGH GPL8
	set_s3c_reg_val(S3C64XX_GPLDAT, regval);
	
	regval = get_s3c_reg_val(S3C64XX_GPLCON1);
	regval &= ~(0xf << 0); //GPL8
	regval |= (1<<0); //OUTPUT GPL8
	set_s3c_reg_val(S3C64XX_GPLCON1, regval);

	regval = get_s3c_reg_val(S3C64XX_GPLPUD);
	regval &= ~(3 << 18); //Disable pullup/pulldown resistor on GPL9
	set_s3c_reg_val(S3C64XX_GPLPUD, regval);
	
	regval = get_s3c_reg_val(S3C64XX_GPLCON1);
	regval &= ~(0xf << 4); //GPL9
	regval |= (0x3<<4); //GPL9 Ext Interrupt[17]
	set_s3c_reg_val(S3C64XX_GPLCON1, regval);
	//TODO -- Need to register WIFI wakeup Host interrupt ISR
	
	//s3c_gpio_setpull(WF_RSTn, S3C_GPIO_PULL_DOWN);  //RESETn low
	//s3c_gpio_cfgpin(WF_RSTn, WF_RSTn_AF); //output
	regval = get_s3c_reg_val(S3C64XX_GPNPUD);
	regval &= ~(0x3 << 8); //GPN4
	regval |= (0x1 << 8); //pulldown
	set_s3c_reg_val(S3C64XX_GPNPUD, regval);
	
	regval = get_s3c_reg_val(S3C64XX_GPNDAT);
	regval &= ~(0x1 <<4); //GPN4, Low
	set_s3c_reg_val(S3C64XX_GPNDAT, regval);

	regval = get_s3c_reg_val(S3C64XX_GPNCON);
	regval &= ~(0x3 << 8);  //GPN4
	regval |= (0x1 << 8);  //OUTPUT
	set_s3c_reg_val(S3C64XX_GPNCON, regval);
	
	//PullUp WIFI_PWR_EN (GPN5)
	regval = get_s3c_reg_val(S3C64XX_GPNPUD);
	regval &= ~(3 << 10); //GPN5
	regval |= (2 << 10); //Pullup
	set_s3c_reg_val(S3C64XX_GPNPUD, regval);

	regval = get_s3c_reg_val(S3C64XX_GPNDAT);
	regval |= (1 << 5); //GPN5 High
	set_s3c_reg_val(S3C64XX_GPNDAT, regval);

	regval = get_s3c_reg_val(S3C64XX_GPNCON);
	regval &= ~(3 << 10); //GPN5
	regval |= (1 << 10); //OUTPUT
	set_s3c_reg_val(S3C64XX_GPNCON, regval);

	//PullUp WF_PWR_DNn (GPN3)
	regval = get_s3c_reg_val(S3C64XX_GPNPUD);
	regval &= ~(0x3<<6); //GPN3
	regval |= (0x2<<6); //Pullup
	set_s3c_reg_val(S3C64XX_GPNPUD, regval);

	regval = get_s3c_reg_val(S3C64XX_GPNDAT);
	regval |= (0x1<<3); //High GPN3
	set_s3c_reg_val(S3C64XX_GPNDAT, regval);

	//Enable Pullup Resistor for WF_RSTn (GPN4)
	regval = get_s3c_reg_val(S3C64XX_GPNPUD);
	regval &= ~(0x3 << 8); //GPN4
	regval |= (0x2 << 8); //Pullup resistor on GPN3, PullUp resistor on GPN4
	set_s3c_reg_val(S3C64XX_GPNPUD, regval);

	regval = get_s3c_reg_val(S3C64XX_GPNDAT);
	regval |= (0x1<<4); //High GPN4
	set_s3c_reg_val(S3C64XX_GPNDAT, regval);

	//Make sure accurate WIFI RESET timing here
	udelay(600); //delay 1000us to satisfy ">500us"

	local_irq_save(flags);
	//Generate negative RESET pulse width (1us < T1 < 20us)
	regval = get_s3c_reg_val(S3C64XX_GPNPUD);
	regval &= ~(0x3<<8);
	regval |= (0x1<<8); //PullDown resistor on GPN4
	set_s3c_reg_val(S3C64XX_GPNPUD, regval);

	regval = get_s3c_reg_val(S3C64XX_GPNDAT);
	regval &= ~(0x1<<4); //GPN4 LOW
	set_s3c_reg_val(S3C64XX_GPNDAT, regval);
	
	udelay(10); //Satisfy "1us < T1 < 20us"

	regval = get_s3c_reg_val(S3C64XX_GPNPUD);
	regval &= ~(0x3<<8);
	regval |= (0x2<<8); //PullUp resistor on GPN3, PullUp resistor on GPN4
	set_s3c_reg_val(S3C64XX_GPNPUD, regval);

	regval = get_s3c_reg_val(S3C64XX_GPNDAT);
	regval |= (0x1<<4);  //High GPN3, High GPN4
	set_s3c_reg_val(S3C64XX_GPNDAT, regval);
	local_irq_restore(flags);
}
#ifdef CONFIG_BRAVO_WIFI_ONOFF
void bravo_on_wifi(void)
{
	s3c6410_enable_wifi_pins();
}
EXPORT_SYMBOL_GPL(bravo_on_wifi);

void s3c6410_turnoff_wifi(void)
{
	unsigned long regval = 0;

	// Drive WF_MAC_WAKE (GPL8) low
	regval = get_s3c_reg_val(S3C64XX_GPLDAT);
	regval &= ~(1<<8);
	set_s3c_reg_val(S3C64XX_GPLDAT, regval);

	// Drive WF_RSTn (GPN4) low
	regval = get_s3c_reg_val(S3C64XX_GPNDAT);
	regval &= ~(1<<4);
	set_s3c_reg_val(S3C64XX_GPNDAT, regval);

	// Drive WF_SD_CMD (GPC4) and WF_SD_CLK (GPC5) low

	// Configure as outputs
	regval = get_s3c_reg_val(S3C64XX_GPCCON);
	regval &= ~(0xf << 16);
	regval &= ~(0xf << 20);
	regval |= (0x1 << 16);
	regval |= (0x1 << 20);
	set_s3c_reg_val(S3C64XX_GPCCON, regval);

	// and drive low
	regval = get_s3c_reg_val(S3C64XX_GPCDAT);
	regval &= ~(0x1 << 4);
	regval &= ~(0x1 << 5);
	set_s3c_reg_val(S3C64XX_GPCDAT, regval);

	// Drive WF_SD_D0 to WF_SD_D3 low (GPH6-GPH9)
	
	// configure GPH6 and GPH7 as outputs
	regval = get_s3c_reg_val(S3C64XX_GPHCON0);
	regval &= ~(0xf << 24);
	regval &= ~(0xf << 28);
	regval |= (0x1 << 24);
	regval |= (0x1 << 28);
	set_s3c_reg_val(S3C64XX_GPHCON0, regval);

	// configure GPH8 and GPH9 as outputs
	regval = get_s3c_reg_val(S3C64XX_GPHCON1);
	regval &= ~(0xf);
	regval &= ~(0xf << 4);
	regval |= (0x1);
	regval |= (0x1 << 4);
	set_s3c_reg_val(S3C64XX_GPHCON1, regval);
	
	// and drive low
	regval = get_s3c_reg_val(S3C64XX_GPHDAT);
	regval &= ~(0x1 << 6);
	regval &= ~(0x1 << 7);
	regval &= ~(0x1 << 8);
	regval &= ~(0x1 << 9);
	set_s3c_reg_val(S3C64XX_GPHDAT, regval);

	//Pulldown WF_PWR_DNn
	regval = get_s3c_reg_val(S3C64XX_GPNPUD);
	regval &= ~(0x3<<6); //GPN3
	regval |= (0x1<<6); //Pulldown
	set_s3c_reg_val(S3C64XX_GPNPUD, regval);

	regval = get_s3c_reg_val(S3C64XX_GPNDAT);
	regval &= ~(0x1<<3);
	set_s3c_reg_val(S3C64XX_GPNDAT, regval);

	regval = get_s3c_reg_val(S3C64XX_GPNPUD);
	regval &= ~(3 << 10); //GPN5
	regval |= (1 << 10); //Pulldown
	set_s3c_reg_val(S3C64XX_GPNPUD, regval);

	regval = get_s3c_reg_val(S3C64XX_GPNDAT);
	regval &= ~(1<<5); //GPN5 LOW
	set_s3c_reg_val(S3C64XX_GPNDAT, regval);

	mdelay(1); //For sake of safe power up and reset
}
void bravo_off_wifi(void)
{
	s3c6410_turnoff_wifi();
}
EXPORT_SYMBOL_GPL(bravo_off_wifi);
#endif

void s3c6410_reset_wifi(void)
{
	mdelay(1); //For sake of safe power up and reset
}

#ifdef CONFIG_MACH_BRAVO_EVT1
void s3c6410_enable_lightsensor(void)
{
	unsigned long regval = 0x0;
	printk(KERN_NOTICE "Enable light sensor (Light_PD GPN12 LOW)\n");
	regval = get_s3c_reg_val(S3C64XX_GPNCON);
	regval &= ~(3 << (12*2)); //GPN12 --Light_PD pin
	regval |= (1 << (12*2)); //GPN12 OUTPUT
	set_s3c_reg_val(S3C64XX_GPNCON, regval);

	regval = get_s3c_reg_val(S3C64XX_GPNPUD);
	regval &= ~(3 << (12*2));
	regval |= (1 << (12*2)); //GPN12 PullDown
	set_s3c_reg_val(S3C64XX_GPNPUD, regval);

	regval = get_s3c_reg_val(S3C64XX_GPNDAT);
	regval &= ~(1 << 12); //GPN12 LOW
	set_s3c_reg_val(S3C64XX_GPNDAT, regval);
	regval = get_s3c_reg_val(S3C64XX_GPNDAT);
	printk(KERN_NOTICE "Enable light sensor (Light_PD GPN12 0x%x)\n", regval);
	
	mdelay(2);
}

//Susan -- currently this function only support GROUP0 EINT config
void s3c_set_irqflt_group0(int irq, int enable, int value)
{
	unsigned long flags;
	unsigned int regval;
	unsigned int regaddr = 0x0;
	int shift = 0x0;
	
	if ( (irq > IRQ_EINT(27)) || (irq < IRQ_EINT(0)) )
	{
		printk(KERN_NOTICE "s3c_set_irqflt_group0 only support EINT irq group0, (%d) is not of group0\n", irq);
		return;
	}
	regaddr = (unsigned long)(S3C64XX_EINT0FLTCON0) + (((irq - IRQ_EINT(0))>>3)<<2);
	printk(KERN_NOTICE "S3C_SET_IRQFLT_GROUP0: REGADDR(0x%x)\n", regaddr);
	shift = ((((irq - IRQ_EINT(0))%8)>>1)<<3);

	local_irq_save(flags);
	
	regval = get_s3c_reg_val((void __iomem *)regaddr);
	printk(KERN_NOTICE "s3c_set_irqflt_group0: regaddr(0x%x), shift(%d), regval(0x%x), base(0x%x)\n", (regaddr), shift, regval, (unsigned long)(S3C64XX_EINT0FLTCON0));
	if (enable)
	{
		regval |= ((0xC0 | value) & 0xFF)<<shift;
		set_s3c_reg_val((void __iomem *)regaddr, regval);
	}
	else  //Disable IRQ FLT
	{
		regval &= ~(0xFF<<shift);
		set_s3c_reg_val((void __iomem *)regaddr, regval);
	}
	regval = get_s3c_reg_val((void __iomem *)regaddr);
	printk(KERN_NOTICE "s3c_set_irqflt_group0: regval(0x%x)\n", regval);
	
	local_irq_restore(flags);
}
EXPORT_SYMBOL(s3c_set_irqflt_group0);

#endif
