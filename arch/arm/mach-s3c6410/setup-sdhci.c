/* linux/arch/arm/mach-s3c6410/setup-sdhci.c
 *
 * Copyright 2008 Simtec Electronics
 * Copyright 2008 Simtec Electronics
 *	Ben Dooks <ben@simtec.co.uk>
 *	http://armlinux.simtec.co.uk/
 *
 * S3C6410 - Helper functions for settign up SDHCI device(s) (HSMMC)
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
*/

#include <linux/kernel.h>
#include <linux/types.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/io.h>

#include <linux/mmc/card.h>
#include <linux/mmc/host.h>

#include <mach/gpio.h>
#include <plat/gpio-cfg.h>
#include <plat/regs-sdhci.h>
#include <plat/sdhci.h>

#include <mach/map.h>
#include <plat/gpio-cfg.h>
#include <plat/regs-gpio.h>
#include <mach/bravo_gpio.h>

#ifdef CONFIG_MACH_CAPELA
#include <mach/capela.h>
#endif


/* clock sources for the mmc bus clock, order as for the ctrl2[5..4] */

char *s3c6410_hsmmc_clksrcs[] = {
	[0] = "hsmmc",
	[1] = "hsmmc",
	[2] = "hsmmc",
	[3] = NULL,
	/* [3] = "48m", - note not successfully used yet */
};

char *s3c6410_hsmmc2_clksrcs[] = {
	[0] = "hsmmc",
	[1] = "hsmmc",
	[2] = "epll",
	[3] = NULL,
};

void s3c6410_setup_sdhci0_cfg_gpio(struct platform_device *dev, int width)
{
	unsigned int gpio;
	unsigned int end;
	volatile unsigned long regval = 0x0;

#ifdef CONFIG_MACH_BRAVO_EVT1
	s3c_gpio_cfgpin(S3C64XX_GPE(0), S3C_GPIO_SFN(1)); //GPE0 OUTPUT
	s3c_gpio_setpull(S3C64XX_GPE(0), S3C_GPIO_PULL_DOWN); //GPE0 PullDown
#endif

	regval = get_s3c_reg_val(S3C64XX_GPGCON);
	pr_debug("s3c6410_setup_sdhci0_cfg_gpio:(0) GPGCON(0x%x)\n", (unsigned)regval);
	
	
	end = S3C64XX_GPG(2 + width);

	/* Set all the necessary GPG pins to special-function 0 */
	for (gpio = S3C64XX_GPG(0); gpio < end; gpio++) {
		s3c_gpio_cfgpin(gpio, S3C_GPIO_SFN(2));
		s3c_gpio_setpull(gpio, S3C_GPIO_PULL_NONE);
	}

#ifdef CONFIG_MACH_CAPELA
	gpio_set_value(GPIO_TF_DETECT, GPIO_TF_DETECT_AF);
	s3c_gpio_setpull(GPIO_TF_DETECT, S3C_GPIO_PULL_UP);
#else
#ifdef CONFIG_MACH_BRAVO
	s3c_gpio_setpull(S3C64XX_GPG(6), S3C_GPIO_PULL_UP);
	s3c_gpio_cfgpin(S3C64XX_GPG(6), S3C_GPIO_SFN(2)); //Susan -- otherwise, CH1 on EVT1 willl fail at initialization (CMD8)
#endif
#endif
	regval = get_s3c_reg_val(S3C64XX_GPGCON);
	pr_debug("s3c6410_setup_sdhci0_cfg_gpio:(1) GPGCON(0x%x)\n", (unsigned)regval);
}

void s3c6410_setup_sdhci1_cfg_gpio(struct platform_device *dev, int width)
{
	unsigned int gpio;
	unsigned int end;
	volatile unsigned long regval = 0x0;

	#ifdef CONFIG_MACH_BRAVO_EVT1
	s3c_gpio_cfgpin(S3C64XX_GPE(1), S3C_GPIO_SFN(1)); //GPE1 OUTPUT
	s3c_gpio_setpull(S3C64XX_GPE(1), S3C_GPIO_PULL_DOWN); //GPE1 PullDown
	#endif
	
	regval = get_s3c_reg_val(S3C64XX_GPGCON);
	pr_debug("s3c6410_setup_sdhci1_cfg_gpio:(0) GPGCON(0x%x)\n", (unsigned)regval);
	
	end = S3C64XX_GPH(2 + width);

	/* Set all the necessary GPG pins to special-function 0 */
	for (gpio = S3C64XX_GPH(0); gpio < end; gpio++) {
		s3c_gpio_cfgpin(gpio, S3C_GPIO_SFN(2));
		s3c_gpio_setpull(gpio, S3C_GPIO_PULL_UP);
	}
	#ifdef CONFIG_MACH_BRAVO_PROTOTYPE
	#ifdef BRAVO_SDIO_1_BIT
	pr_debug("s3c6410_setup_sdhci1_cfg_gpio: config special SDIO 1 bit mode for CH1\n");
	s3c_gpio_setpull(S3C64XX_GPH(2), S3C_GPIO_PULL_UP); //SDIO_DAT0

	s3c_gpio_cfgpin(S3C64XX_GPH(3), S3C_GPIO_SFN(2)); //SDIO_DAT1
	s3c_gpio_setpull(S3C64XX_GPH(3), S3C_GPIO_PULL_UP);
	#else
	pr_debug("s3c6410_setup_sdhci1_cfg_gpio: WIDTH(%d)\n", width);
	#endif //BRAVO_SDIO_1_BIT
	#endif //CONFIG_MACH_BRAVO_PROTOTYPE

	#ifdef CONFIG_MACH_BRAVO_EVT1
	//s3c_gpio_setpull(S3C64XX_GPG(6), S3C_GPIO_PULL_UP);
	//s3c_gpio_cfgpin(S3C64XX_GPG(6), S3C_GPIO_SFN(3));
	#endif
	regval = get_s3c_reg_val(S3C64XX_GPGCON);
	pr_debug("s3c6410_setup_sdhci1_cfg_gpio:(1) GPGCON(0x%x)\n", (unsigned)regval);

}

void s3c6410_setup_sdhci2_cfg_gpio(struct platform_device *dev, int width)
{
	volatile unsigned long regval = 0x0;
	
	regval = get_s3c_reg_val(S3C64XX_GPGCON);
	pr_debug("s3c6410_setup_sdhci2_cfg_gpio:(0) GPGCON(0x%x)\n", (unsigned)regval);
	
	/* GPIO C : Command, Clock for the Bigbang */
	s3c_gpio_cfgpin(S3C64XX_GPC(5), S3C_GPIO_SFN(3));
	s3c_gpio_cfgpin(S3C64XX_GPC(4), S3C_GPIO_SFN(3));

	s3c_gpio_setpull(S3C64XX_GPC(5), S3C_GPIO_PULL_NONE);
	s3c_gpio_setpull(S3C64XX_GPC(4), S3C_GPIO_PULL_UP);//Susan -- CH2 CMD must be pull up

	pr_debug("s3c6410_setup_sdhci2_cfg_gpio:WIDTH(%d)\n", width);

	if (width == 1) {
		/* GPIO H : MMC Data1[0] */
		s3c_gpio_cfgpin(S3C64XX_GPH(6), S3C_GPIO_SFN(3));
		s3c_gpio_setpull(S3C64XX_GPH(6), S3C_GPIO_PULL_UP); //Susan -- CH2 DAT0 must be pull up
		s3c_gpio_cfgpin(S3C64XX_GPH(7), S3C_GPIO_SFN(3));
		s3c_gpio_setpull(S3C64XX_GPH(7), S3C_GPIO_PULL_UP);//Susan -- CH2 DAT1 must be pull up
	} else if (width == 4) {
		/* GPIO H : MMC DATA1[0:3] */
		s3c_gpio_cfgpin(S3C64XX_GPH(6), S3C_GPIO_SFN(3));
		s3c_gpio_cfgpin(S3C64XX_GPH(7), S3C_GPIO_SFN(3));
		s3c_gpio_cfgpin(S3C64XX_GPH(8), S3C_GPIO_SFN(3));
		s3c_gpio_cfgpin(S3C64XX_GPH(9), S3C_GPIO_SFN(3));

		s3c_gpio_setpull(S3C64XX_GPH(6), S3C_GPIO_PULL_UP);  //Susan -- CH2 DAT0 must be pullup
		s3c_gpio_setpull(S3C64XX_GPH(7), S3C_GPIO_PULL_UP);  //Susan -- CH2 DAT1 must be pullup
		s3c_gpio_setpull(S3C64XX_GPH(8), S3C_GPIO_PULL_UP);  //Susan -- CH2 DAT2 must be pullup
		s3c_gpio_setpull(S3C64XX_GPH(9), S3C_GPIO_PULL_UP);  //Susan -- CH2 DAT3 must be pullup
	}
	
	regval = get_s3c_reg_val(S3C64XX_GPGCON);
	pr_debug("s3c6410_setup_sdhci2_cfg_gpio:(1) GPGCON(0x%x)\n", (unsigned)regval);

}

void s3c6410_setup_sdhci0_cfg_card(struct platform_device *dev,
				    void __iomem *r,
				    struct mmc_ios *ios,
				    struct mmc_card *card)
{
	u32 ctrl2 = 0, ctrl3 = 0;

	/* don't need to alter anything acording to card-type */

	writel(S3C64XX_SDHCI_CONTROL4_DRIVE_9mA, r + S3C64XX_SDHCI_CONTROL4);

	ctrl2 = readl(r + S3C_SDHCI_CONTROL2);
	ctrl2 &= S3C_SDHCI_CTRL2_SELBASECLK_MASK;
	ctrl2 |= (S3C64XX_SDHCI_CTRL2_ENSTAASYNCCLR |
		  S3C64XX_SDHCI_CTRL2_ENCMDCNFMSK |
		  S3C_SDHCI_CTRL2_ENFBCLKRX |
		  S3C_SDHCI_CTRL2_DFCNT_NONE |
		  S3C_SDHCI_CTRL2_ENCLKOUTHOLD);

	if (ios->clock < 25 * 1000000)
	{
		//writel(S3C64XX_SDHCI_CONTROL4_DRIVE_9mA, r + S3C64XX_SDHCI_CONTROL4);
		//printk("*** KEVIN_DEBUG : clock => low speed (%d)***\n",ios->clock);
		
		ctrl3 = (S3C_SDHCI_CTRL3_FCSEL3 |
			 S3C_SDHCI_CTRL3_FCSEL2 |
			 S3C_SDHCI_CTRL3_FCSEL1 |
			 S3C_SDHCI_CTRL3_FCSEL0);
	}
	else
	{
		//writel(S3C64XX_SDHCI_CONTROL4_DRIVE_2mA, r + S3C64XX_SDHCI_CONTROL4);
		//printk("*** KEVIN_DEBUG : clock => high speed(%d)***\n",ios->clock);
		ctrl3 |= (S3C_SDHCI_CTRL3_FCSEL1 |
			 S3C_SDHCI_CTRL3_FCSEL0);
	}
	
	writel(ctrl2, r + S3C_SDHCI_CONTROL2);
	writel(ctrl3, r + S3C_SDHCI_CONTROL3);
}
