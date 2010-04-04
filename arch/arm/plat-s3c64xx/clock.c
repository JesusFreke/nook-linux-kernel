/* linux/arch/arm/plat-s3c64xx/clock.c
 *
 * Copyright 2008 Openmoko, Inc.
 * Copyright 2008 Simtec Electronics
 *	Ben Dooks <ben@simtec.co.uk>
 *	http://armlinux.simtec.co.uk/
 *
 * S3C64XX Base clock support
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * Modifications 2009 : Intrinsyc Software, Inc on behalf of Barnes and Noble
 * Portions of this code copyright (c) 2009 Barnes and Noble, Inc
*/

#include <linux/init.h>
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/ioport.h>
#include <linux/delay.h>
#include <linux/io.h>

#include <mach/hardware.h>
#include <mach/map.h>

#include <plat/regs-clock.h>
#include <plat/cpu.h>
#include <plat/devs.h>
#include <plat/clock.h>

#include <plat/s3c64xx-dvfs.h>

extern void s3c64xx_change_divider(u32 divider);

/* definition for cpu freq */

#define ARM_PLL_CON 	S3C_APLL_CON
#define ARM_CLK_DIV	S3C_CLK_DIV0

#define ARM_DIV_RATIO_BIT		0
#define ARM_DIV_MASK			(0xf<<ARM_DIV_RATIO_BIT)
#define HCLK_DIV_RATIO_BIT		9
#define HCLK_DIV_MASK			(0x7<<HCLK_DIV_RATIO_BIT)
#define PCLK_DIV_RATIO_BIT		12
#define PCLK_DIV_MASK			(0xf<<PCLK_DIV_RATIO_BIT)

#define READ_ARM_DIV    		((__raw_readl(ARM_CLK_DIV)&ARM_DIV_MASK) + 1)
#define READ_HCLKX2_DIV                    ((__raw_readl(ARM_CLK_DIV)&HCLK_DIV_MASK) + 1)
#define PLL_CALC_VAL(MDIV,PDIV,SDIV)	((1<<31)|(MDIV)<<16 |(PDIV)<<8 |(SDIV))
#define GET_ARM_CLOCK(baseclk)		s3c6400_get_pll(__raw_readl(S3C_APLL_CON),baseclk)

#define INIT_XTAL			12 * MHZ

static const u32 s3c_cpu_clock_table[][6] = {
        {532*MHZ, 266, 3, 1, 0, 0},
        {266*MHZ, 266, 3, 1, 0, 1},
        {133*MHZ, 266, 3, 1, 0, 3},
};

unsigned int S3C64XX_FREQ_TAB = 0;

struct clk clk_27m = {
	.name		= "clk_27m",
	.id		= -1,
	.rate		= 27000000,
};

static int clk_48m_ctrl(struct clk *clk, int enable)
{
	unsigned long flags;
	u32 val;

	/* can't rely on clock lock, this register has other usages */
	local_irq_save(flags);

	val = __raw_readl(S3C_OTHERS);
	if (enable)
		val |= S3C_OTHERS_USB_SIG_MASK;
	else
		val &= ~S3C_OTHERS_USB_SIG_MASK;

	__raw_writel(val, S3C_OTHERS);
	local_irq_restore(flags);

	return 0;
}

struct clk clk_48m = {
	.name		= "clk_48m",
	.id		= -1,
	.rate		= 48000000,
	.enable		= clk_48m_ctrl,
};
/*
static u32 s3c_cpu_clk_tab_size(void)
{
	u32 size;
	if(S3C64XX_FREQ_TAB)
		size = ARRAY_SIZE(s3c_cpu_clk_tab_800MHz);
	else
		size = ARRAY_SIZE(s3c_cpu_clk_tab_533MHz);

	return size;
}
*/
unsigned long s3c64xx_get_clk_rate(int *armclk)
{
    u32 hclkx2_div_tmp;
    unsigned long apll_con;
    unsigned long clk_div0_tmp;
    unsigned long m = 0;
    unsigned long p = 0;
    unsigned long s = 0;
    unsigned long ret;

	apll_con = __raw_readl(S3C_APLL_CON);
	clk_div0_tmp = __raw_readl(S3C_CLK_DIV0);// & 0xf;
	hclkx2_div_tmp = clk_div0_tmp & (HCLK_DIV_MASK);
	hclkx2_div_tmp = hclkx2_div_tmp >> HCLK_DIV_RATIO_BIT;
	clk_div0_tmp = clk_div0_tmp & 0xf;
	m = (apll_con >> 16) & 0x3ff;
	p = (apll_con >> 8) & 0x3f;
	s = apll_con & 0x3;

	ret = (m * (INIT_XTAL / (p * (1 << s))));

	*armclk = (ret / (clk_div0_tmp + 1));

	ret = ret /(hclkx2_div_tmp + 1);

	return ret;
}

unsigned long s3c_fclk_get_rate(void)
{
	unsigned long apll_con;
	unsigned long clk_div0_tmp;
	unsigned long m = 0;
	unsigned long p = 0;
	unsigned long s = 0;
	unsigned long ret;

	apll_con = __raw_readl(S3C_APLL_CON);
	clk_div0_tmp = __raw_readl(S3C_CLK_DIV0) & 0xf;
	m = (apll_con >> 16) & 0x3ff;
	p = (apll_con >> 8) & 0x3f;
	s = apll_con & 0x3;
	ret = (m * (INIT_XTAL / (p * (1 << s))));
	
	return (ret / (clk_div0_tmp + 1));
}

unsigned long s3c_fclk_round_rate(struct clk *clk, unsigned long rate)
{
	u32 iter;

	for(iter = 1 ; iter < ARRAY_SIZE(s3c_cpu_clock_table) ; iter++) {
		if(rate > s3c_cpu_clock_table[iter][0])
			return s3c_cpu_clock_table[iter-1][0];
	}
	
	return s3c_cpu_clock_table[ARRAY_SIZE(s3c_cpu_clock_table) - 1][0];
}

int s3c_fclk_set_rate(struct clk *clk, unsigned long rate)
{
	u32 round_tmp;
	u32 iter;
	u32 clk_div0_tmp;

	round_tmp = s3c_fclk_round_rate(clk,rate);
	
	if(round_tmp == (int)s3c_fclk_get_rate())
		return 0;

	for (iter = 0 ; iter < ARRAY_SIZE(s3c_cpu_clock_table) ; iter++){
		if(round_tmp == s3c_cpu_clock_table[iter][0])
			break;
	}

	if(iter >= ARRAY_SIZE(s3c_cpu_clock_table))
		iter = ARRAY_SIZE(s3c_cpu_clock_table) - 1;
     
	clk_div0_tmp = __raw_readl(ARM_CLK_DIV) & ~(ARM_DIV_MASK);
	clk_div0_tmp |= s3c_cpu_clock_table[iter][5];
	s3c64xx_change_divider(clk_div0_tmp);
	//__raw_writel(clk_div0_tmp, ARM_CLK_DIV);

	clk->rate = s3c_cpu_clock_table[iter][0];
	return 0;
}

// andyEdit - 2.6.28 Camera driver
extern unsigned long clk_get_rate(struct clk *clk);
static int s3c64xx_setrate_sclk_cam(struct clk *clk, unsigned long rate)
{
	u32 shift = 20;
	u32 cam_div, cfg;
	unsigned long src_clk = clk_get_rate(clk->parent);

	cam_div = src_clk / rate;

	if (cam_div > 32)
		cam_div = 32;

	cfg = __raw_readl(S3C_CLK_DIV0);
	cfg &= ~(0xf << shift);
	cfg |= ((cam_div - 1) << shift);
	__raw_writel(cfg, S3C_CLK_DIV0);

	printk("parent clock for camera: %ld.%03ld MHz, divisor: %d\n", \
		print_mhz(src_clk), cam_div);

	return 0;
}


struct clk clk_cpu = {
	.name		= "clk_cpu",
	.id		= -1,
	.rate		= 0,
	.parent		= &clk_mpll,
	.ctrlbit	= 0,
	.set_rate	= s3c_fclk_set_rate,
	.round_rate	= s3c_fclk_round_rate,
};

static int inline s3c64xx_gate(void __iomem *reg,
				struct clk *clk,
				int enable)
{
	unsigned int ctrlbit = clk->ctrlbit;
	u32 con;

	con = __raw_readl(reg);

	if (enable)
		con |= ctrlbit;
	else
		con &= ~ctrlbit;

	__raw_writel(con, reg);
	return 0;
}

static int s3c64xx_pclk_ctrl(struct clk *clk, int enable)
{
	return s3c64xx_gate(S3C_PCLK_GATE, clk, enable);
}

static int s3c64xx_hclk_ctrl(struct clk *clk, int enable)
{
	return s3c64xx_gate(S3C_HCLK_GATE, clk, enable);
}

int s3c64xx_sclk_ctrl(struct clk *clk, int enable)
{
	return s3c64xx_gate(S3C_SCLK_GATE, clk, enable);
}

static struct clk init_clocks_disable[] = {
	{
		.name		= "nand",
		.id		= -1,
		.parent		= &clk_h,
	}, {
		.name		= "adc",
		.id		= -1,
		.parent		= &clk_p,
		.enable		= s3c64xx_pclk_ctrl,
		.ctrlbit	= S3C_CLKCON_PCLK_TSADC,
	}, {
		.name		= "keypad",
		.id		= -1,
		.parent		= &clk_p,
		.enable		= s3c64xx_pclk_ctrl,
		.ctrlbit	= S3C_CLKCON_PCLK_KEYPAD,
	}, {
		.name		= "i2c",
		.id		= -1,
		.parent		= &clk_p,
		.enable		= s3c64xx_pclk_ctrl,
		.ctrlbit	= S3C_CLKCON_PCLK_IIC,
	}, {
		.name		= "iis",
		.id		= 0,
		.parent		= &clk_p,
		.enable		= s3c64xx_pclk_ctrl,
		.ctrlbit	= S3C_CLKCON_PCLK_IIS0,
	}, {
		.name		= "iis",
		.id		= 1,
		.parent		= &clk_p,
		.enable		= s3c64xx_pclk_ctrl,
		.ctrlbit	= S3C_CLKCON_PCLK_IIS1,
	}, {
		.name		= "spi",
		.id		= 0,
		.parent		= &clk_p,
		.enable		= s3c64xx_pclk_ctrl,
		.ctrlbit	= S3C_CLKCON_PCLK_SPI0,
	}, {
		.name		= "spi",
		.id		= 1,
		.parent		= &clk_p,
		.enable		= s3c64xx_pclk_ctrl,
		.ctrlbit	= S3C_CLKCON_PCLK_SPI1,
	}, {
		.name		= "48m",
		.id		= 0,
		.parent		= &clk_48m,
		.enable		= s3c64xx_sclk_ctrl,
		.ctrlbit	= S3C_CLKCON_SCLK_MMC0_48,
	}, {
		.name		= "48m",
		.id		= 1,
		.parent		= &clk_48m,
		.enable		= s3c64xx_sclk_ctrl,
		.ctrlbit	= S3C_CLKCON_SCLK_MMC1_48,
	}, {
		.name		= "48m",
		.id		= 2,
		.parent		= &clk_48m,
		.enable		= s3c64xx_sclk_ctrl,
		.ctrlbit	= S3C_CLKCON_SCLK_MMC2_48,
	}, {
		.name    	= "otg",
		.id	   	= -1,
		.parent  	= &clk_h,
		.enable  	= s3c64xx_hclk_ctrl,
		.ctrlbit 	= S3C_CLKCON_HCLK_USB
	},
};

static struct clk init_clocks[] = {
	{
		.name		= "lcd",
		.id		= -1,
		.parent		= &clk_h,
		.enable		= s3c64xx_hclk_ctrl,
		.ctrlbit	= S3C_CLKCON_HCLK_LCD,
	}, {
		.name		= "gpio",
		.id		= -1,
		.parent		= &clk_p,
		.enable		= s3c64xx_pclk_ctrl,
		.ctrlbit	= S3C_CLKCON_PCLK_GPIO,
	}, {
		.name		= "usb-host",
		.id		= -1,
		.parent		= &clk_h,
		.enable		= s3c64xx_hclk_ctrl,
		.ctrlbit	= S3C_CLKCON_SCLK_UHOST,
	}, {
		.name		= "hsmmc",
		.id		= 0,
		.parent		= &clk_h,
		.enable		= s3c64xx_hclk_ctrl,
		.ctrlbit	= S3C_CLKCON_HCLK_HSMMC0,
	}, {
		.name		= "hsmmc",
		.id		= 1,
		.parent		= &clk_h,
		.enable		= s3c64xx_hclk_ctrl,
		.ctrlbit	= S3C_CLKCON_HCLK_HSMMC1,
	}, {
		.name		= "hsmmc",
		.id		= 2,
		.parent		= &clk_h,
		.enable		= s3c64xx_hclk_ctrl,
		.ctrlbit	= S3C_CLKCON_HCLK_HSMMC2,
	}, {
		.name    	= "hclk_mfc",
		.id		   	= -1,
		.parent  	= &clk_h,
		.enable  	= s3c64xx_hclk_ctrl,
		.ctrlbit 	= S3C_CLKCON_HCLK_MFC
	},{ 
		.name    	= "hclk_post0",
		.id		   	= -1,
		.parent  	= &clk_h,
		.enable  	= s3c64xx_hclk_ctrl,
		.ctrlbit 	= S3C_CLKCON_HCLK_POST0
	},{
		.name    	= "hclk_jpeg",
		.id	   	= -1,
		.parent  	= &clk_h,
		.enable  	= s3c64xx_hclk_ctrl,
	  .ctrlbit 	= S3C_CLKCON_HCLK_JPEG
	},{ 
		.name   	= "tv_encoder",
		.id	   	= -1,
		.parent  	= &clk_h,
		.enable  	= s3c64xx_hclk_ctrl,
		.ctrlbit 	= S3C_CLKCON_HCLK_TV
	},{ 
		.name    	= "tv_scaler",
		.id	   	= -1,
		.parent  	= &clk_h,
		.enable  	= s3c64xx_hclk_ctrl,
		.ctrlbit 	= S3C_CLKCON_HCLK_SCALER
	},{
		.name		= "timers",
		.id		= -1,
		.parent		= &clk_p,
		.enable		= s3c64xx_pclk_ctrl,
		.ctrlbit	= S3C_CLKCON_PCLK_PWM,
	}, {
		.name		= "uart",
		.id		= 0,
		.parent		= &clk_p,
		.enable		= s3c64xx_pclk_ctrl,
		.ctrlbit	= S3C_CLKCON_PCLK_UART0,
	}, {
		.name		= "uart",
		.id		= 1,
		.parent		= &clk_p,
		.enable		= s3c64xx_pclk_ctrl,
		.ctrlbit	= S3C_CLKCON_PCLK_UART1,
	}, {
		.name		= "uart",
		.id		= 2,
		.parent		= &clk_p,
		.enable		= s3c64xx_pclk_ctrl,
		.ctrlbit	= S3C_CLKCON_PCLK_UART2,
	}, {
		.name		= "uart",
		.id		= 3,
		.parent		= &clk_p,
		.enable		= s3c64xx_pclk_ctrl,
		.ctrlbit	= S3C_CLKCON_PCLK_UART3,
	}, {
		.name		= "rtc",
		.id		= -1,
		.parent		= &clk_p,
		.enable		= s3c64xx_pclk_ctrl,
		.ctrlbit	= S3C_CLKCON_PCLK_RTC,
	}, {
		.name		= "watchdog",
		.id		= -1,
		.parent		= &clk_p,
		.ctrlbit	= S3C_CLKCON_PCLK_WDT,
	}, {
		.name		= "ac97",
		.id		= -1,
		.parent		= &clk_p,
		.ctrlbit	= S3C_CLKCON_PCLK_AC97,
	},{ 
		.name    	= "sclk_mfc",
		.id	   	= -1,
		.parent  	= &clk_s,
		.enable  	= s3c64xx_sclk_ctrl,
		.ctrlbit 	= S3C_CLKCON_SCLK_MFC,
		.usage   	= 0,
		.rate    	= 48*1000*1000,
	},{ 
		.name    	= "sclk_jpeg",
		.id	   	= -1,
		.parent  	= &clk_s,
		.enable  	= s3c64xx_sclk_ctrl,
		.ctrlbit 	= S3C_CLKCON_SCLK_JPEG,
		.usage   	= 0,
		.rate    	= 48*1000*1000,
	},{ 
		.name     = "pclk_mfc",
		.id       = -1,
		.parent   = &clk_p,
		.enable   = s3c64xx_pclk_ctrl,
		.ctrlbit  = S3C_CLKCON_PCLK_MFC,
	// andyEdit - 2.6.28 Camera driver
	}, {
		.name		= "fimc",
		.id		= -1,
		.parent		= &clk_h,
		.ctrlbit	= S3C_CLKCON_HCLK_CAMIF,
	}, {
		.name		= "sclk_cam",
		.id		= -1,
		.parent		= &clk_hx2,
		.enable		= s3c64xx_sclk_ctrl,
		.ctrlbit	= S3C_CLKCON_SCLK_CAM,
		.set_rate	= s3c64xx_setrate_sclk_cam,
	},
#if 0
	{
		.name        = "camera",
		.id      = -1,
		.parent      = &clk_hx2,
		.enable      = s3c_clkcon_enable,
		.ctrlbit     = S3C_CLKCON_SCLK_CAM,
		.set_rate    = s3c6410_setrate_camera,
	},
#endif
};

static struct clk *clks[] __initdata = {
	&clk_ext,
	&clk_epll,
	&clk_27m,
	&clk_48m,
	&clk_cpu,
};

void __init s3c64xx_register_clocks(void)
{
	struct clk *clkp;
	int ret;
	int ptr;

	s3c24xx_register_clocks(clks, ARRAY_SIZE(clks));

	clkp = init_clocks;
	for (ptr = 0; ptr < ARRAY_SIZE(init_clocks); ptr++, clkp++) {
		ret = s3c24xx_register_clock(clkp);
		if (ret < 0) {
			printk(KERN_ERR "Failed to register clock %s (%d)\n",
			       clkp->name, ret);
		}
	}

	clkp = init_clocks_disable;
	for (ptr = 0; ptr < ARRAY_SIZE(init_clocks_disable); ptr++, clkp++) {

		ret = s3c24xx_register_clock(clkp);
		if (ret < 0) {
			printk(KERN_ERR "Failed to register clock %s (%d)\n",
			       clkp->name, ret);
		}

		(clkp->enable)(clkp, 0);
	}
}
