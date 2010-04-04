/* linux/arch/arm/plat-s3c/gpio-config.c
 *
 * Copyright 2008 Openmoko, Inc.
 * Copyright 2008 Simtec Electronics
 *	Ben Dooks <ben@simtec.co.uk>
 *	http://armlinux.simtec.co.uk/
 *
 * S3C series GPIO configuration core
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
*/

#include <linux/kernel.h>
#include <linux/gpio.h>
#include <linux/io.h>

#include <mach/gpio-core.h>
#include <plat/gpio-cfg.h>
#include <plat/gpio-cfg-helpers.h>

int s3c_gpio_cfgpin(unsigned int pin, unsigned int config)
{
	struct s3c_gpio_chip *chip = s3c_gpiolib_getchip(pin);
	unsigned long flags;
	int offset;
	int ret;

	if (!chip)
		return -EINVAL;

	offset = pin - chip->chip.base;

	local_irq_save(flags);
	ret = s3c_gpio_do_setcfg(chip, offset, config);
	local_irq_restore(flags);

	return 0;
}

int s3c_gpio_getcfg(unsigned int pin, unsigned int *config)
{
	struct s3c_gpio_chip *chip = s3c_gpiolib_getchip(pin);
	unsigned long flags;
	int offset, ret;

	if (!chip)
		return -EINVAL;

	offset = pin - chip->chip.base;

	local_irq_save(flags);
	ret = s3c_gpio_do_getcfg(chip, offset, config);
	local_irq_restore(flags);

	return ret;
}

int s3c_gpdat_getval(unsigned int pin, unsigned int *val)
{
	struct s3c_gpio_chip *chip = s3c_gpiolib_getchip(pin);
	void __iomem *reg = chip->base + 0x04;
	int offset;
	unsigned int regval = 0x0;
	unsigned long flags;

	if (!chip)
		return -EINVAL;

	offset = pin - chip->chip.base;

	local_irq_save(flags);
	regval = __raw_readl(reg);
	local_irq_restore(flags);

	*val = (regval) & (1<<offset);
	//if (pin != 126) //Don't printk loginfo for HRDY pin
	//	printk(KERN_NOTICE "s3c_gpdat_getval: chip.label(%s),base(0x%x),reg(0x%x),regval(0x%x),pin(%d),offset(%d)\n", chip->chip.label, chip->base, reg, regval,pin,offset);

	return 0;
}


int s3c_gpio_setpull(unsigned int pin, s3c_gpio_pull_t pull)
{
	struct s3c_gpio_chip *chip = s3c_gpiolib_getchip(pin);
	unsigned long flags;
	int offset, ret;

	if (!chip)
		return -EINVAL;

	offset = pin - chip->chip.base;

	local_irq_save(flags);
	ret = s3c_gpio_do_setpull(chip, offset, pull);
	local_irq_restore(flags);

	return ret;
}

int s3c_gpdat_setval(unsigned int pin, unsigned int level)
{
	struct s3c_gpio_chip *chip = s3c_gpiolib_getchip(pin);
	unsigned long flags;
	int offset, ret;
	volatile u32 regval, regval1;

	if (!chip)
		return -EINVAL;

	offset = pin - chip->chip.base;

	local_irq_save(flags);
	regval = __raw_readl(chip->base+0x4);  //Always GPPORT DAT register?
	regval &= ~(1<<offset);
	regval |= ((level&0x1)<<offset);
	__raw_writel(regval, (chip->base+0x4));
	regval1 = __raw_readl(chip->base+0x4);
	local_irq_restore(flags);

	//printk(KERN_NOTICE "s3c_gpio_setlevel: regval(0x%x), regval1(0x%x)\n", regval, regval1);

	return ret;
}

#ifdef CONFIG_S3C_GPIO_CFG_S3C24XX
int s3c_gpio_setcfg_s3c24xx_banka(struct s3c_gpio_chip *chip,
				  unsigned int off, unsigned int cfg)
{
	void __iomem *reg = chip->base;
	unsigned int shift = off;
	u32 con;

	if (s3c_gpio_is_cfg_special(cfg)) {
		cfg &= 0xf;

		/* Map output to 0, and SFN2 to 1 */
		cfg -= 1;
		if (cfg > 1)
			return -EINVAL;

		cfg <<= shift;
	}

	con = __raw_readl(reg);
	con &= ~(0x1 << shift);
	con |= cfg;
	__raw_writel(con, reg);

	return 0;
}

int s3c_gpio_setcfg_s3c24xx(struct s3c_gpio_chip *chip,
			    unsigned int off, unsigned int cfg)
{
	void __iomem *reg = chip->base;
	unsigned int shift = off * 2;
	u32 con;

	if (s3c_gpio_is_cfg_special(cfg)) {
		cfg &= 0xf;
		if (cfg > 3)
			return -EINVAL;

		cfg <<= shift;
	}

	con = __raw_readl(reg);
	con &= ~(0x3 << shift);
	con |= cfg;
	__raw_writel(con, reg);

	return 0;
}
#endif

#ifdef CONFIG_S3C_GPIO_CFG_S3C64XX
int s3c_gpio_setcfg_s3c64xx_4bit(struct s3c_gpio_chip *chip,
				 unsigned int off, unsigned int cfg)
{
	void __iomem *reg = chip->base;
	unsigned int shift = (off & 7) * 4;
	u32 con;

	if (off < 8 && chip->chip.ngpio > 8)
		reg -= 4;

	if (s3c_gpio_is_cfg_special(cfg)) {
		cfg &= 0xf;
		cfg <<= shift;
	}

	con = __raw_readl(reg);
	con &= ~(0xf << shift);
	con |= cfg;
	__raw_writel(con, reg);

	return 0;
}
//[ICS] Add getcfg function
int s3c_gpio_getcfg_s3c64xx_4bit(struct s3c_gpio_chip *chip,unsigned int off, unsigned int *cfg)
{
	void __iomem *reg = chip->base;
	unsigned int shift = (off & 7) * 4;
	u32 con;

	if (off < 8 && chip->chip.ngpio > 8)
		reg -= 4;

	con = __raw_readl(reg);
	*cfg = con;  //[ICS] we want the whole register value

	return 0;
}
//[ICS] Add getcfg function
int s3c_gpio_getcfg_s3c64xx_2bit(struct s3c_gpio_chip *chip,unsigned int off, unsigned int *cfg)
{
	void __iomem *reg = chip->base;
	unsigned int shift = off * 2;
	u32 con;

	con = __raw_readl(reg);
	*cfg = con;

	return 0;
}

#endif /* CONFIG_S3C_GPIO_CFG_S3C64XX */

#ifdef CONFIG_S3C_GPIO_CFG_S5PC1XX
int s3c_gpio_setcfg_s5pc1xx(struct s3c_gpio_chip *chip,
				 unsigned int off, unsigned int cfg)
{
	void __iomem *reg = chip->base;
	unsigned int shift = (off & 7) * 4;
	u32 con;

	if (off < 8 && chip->chip.ngpio > 8)
		reg -= 4;

	if (s3c_gpio_is_cfg_special(cfg)) {
		cfg &= 0xf;
		cfg <<= shift;
	}

	con = __raw_readl(reg);
	con &= ~(0xf << shift);
	con |= cfg;
	__raw_writel(con, reg);

	return 0;
}
#endif /* CONFIG_S3C_GPIO_CFG_S5PC1XX */

#ifdef CONFIG_S3C_GPIO_PULL_UPDOWN
int s3c_gpio_setpull_updown(struct s3c_gpio_chip *chip,
			    unsigned int off, s3c_gpio_pull_t pull)
{
	void __iomem *reg = chip->base + 0x08;
	int shift = off * 2;
	u32 pup;

	pup = __raw_readl(reg);
	pup &= ~(3 << shift);
	pup |= pull << shift;
	__raw_writel(pup, reg);
	pup = __raw_readl(reg);
	//printk(KERN_NOTICE "s3c_gpio_setpull_updown: chip.label(%s),off(%d),reg(0x%x),pup(0x%x)\n", chip->chip.label, off, reg, pup);
	return 0;
}

s3c_gpio_pull_t s3c_gpio_getpull_updown(struct s3c_gpio_chip *chip,
					unsigned int off)
{
	void __iomem *reg = chip->base + 0x08;
	int shift = off * 2;
	u32 pup = __raw_readl(reg);

	pup >>= shift;
	pup &= 0x3;
	return (__force s3c_gpio_pull_t)pup;
}
#endif
