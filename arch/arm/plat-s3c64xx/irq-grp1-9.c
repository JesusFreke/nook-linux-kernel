/* arch/arm/plat-s3c64xx/irq-eint.c
 *
 * Copyright 2008 Openmoko, Inc.
 * Copyright 2008 Simtec Electronics
 *      Ben Dooks <ben@simtec.co.uk>
 *      http://armlinux.simtec.co.uk/
 *
 * S3C64XX - Interrupt handling for IRQ_EINT(x)
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/kernel.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/io.h>

#include <asm/hardware/vic.h>

#include <plat/regs-irqtype.h>

#include <mach/map.h>
#include <plat/cpu.h>

#include <mach/gpio.h>
#include <plat/gpio-cfg.h>
#include <plat/gpio-bank-n.h>
#include <plat/regs-gpio.h>
#include <mach/bravo_gpio.h>

#define S3C_GRP1_9_DEBUG
#undef S3C_GRP1_9_DEBUG

#define eint_offset(group,irq)	((irq) - group_regs[group-1].base)
#define eint_irq_to_bit(group, irq)	((1 << eint_offset(group, irq)) << group_regs[group-1].shift)

typedef struct
{
    void *config;
    void *filter;
    void *mask;
    void *pending;
    u32 shift;
    u32 base;
    u32 count;
    void __iomem *gpio;
    void __iomem *gpio1;
    int irqs;
} group_registers;

static u16 group_lookup[NR_IRQS] = {0};
static group_registers group_regs[9] =
{
    {
	.config  = S3C64XX_EINT12CON,
	.filter  = S3C64XX_EINT12FLTCON,
	.mask    = S3C64XX_EINT12MASK,
	.pending = S3C64XX_EINT12PEND,
	.shift   = 0,
	.base    = IRQ_EINT_GROUP1_BASE,
	.count   = IRQ_EINT_GROUP1_NR,
	.gpio    = S3C64XX_GPACON,
	.gpio1   = S3C64XX_GPBCON,
	.irqs    = 4
    },
    {
	.config  = S3C64XX_EINT12CON,
	.filter  = S3C64XX_EINT12FLTCON,
	.mask    = S3C64XX_EINT12MASK,
	.pending = S3C64XX_EINT12PEND,
	.shift   = 16,
	.base    = IRQ_EINT_GROUP2_BASE,
	.count   = IRQ_EINT_GROUP2_NR,
	.gpio    = S3C64XX_GPCCON,
	.irqs    = 4

    },
    {
	.config  = S3C64XX_EINT34CON,
	.filter  = S3C64XX_EINT34FLTCON,
	.mask    = S3C64XX_EINT34MASK,
	.pending = S3C64XX_EINT34PEND,
	.shift   = 0,
	.base    = IRQ_EINT_GROUP3_BASE,
	.count   = IRQ_EINT_GROUP3_NR,
	.gpio    = S3C64XX_GPDCON,
	.irqs    = 4
    },
    {
	.config  = S3C64XX_EINT34CON,
	.filter  = S3C64XX_EINT34FLTCON,
	.mask    = S3C64XX_EINT34MASK,
	.pending = S3C64XX_EINT34PEND,
	.shift   = 16,
	.base    = IRQ_EINT_GROUP4_BASE,
	.count   = IRQ_EINT_GROUP4_NR,
	.gpio    = S3C64XX_GPFCON,
	.irqs    = 2
    },
    {
	.config  = S3C64XX_EINT56CON,
	.filter  = S3C64XX_EINT56FLTCON,
	.mask    = S3C64XX_EINT56MASK,
	.pending = S3C64XX_EINT56PEND,
	.shift   = 0,
	.base    = IRQ_EINT_GROUP5_BASE,
	.count   = IRQ_EINT_GROUP5_NR,
	.gpio    = S3C64XX_GPGCON,
	.irqs    = 4
    },
    {
	.config  = S3C64XX_EINT56CON,
	.filter  = S3C64XX_EINT56FLTCON,
	.mask    = S3C64XX_EINT56MASK,
	.pending = S3C64XX_EINT56PEND,
	.shift   = 16,
	.base    = IRQ_EINT_GROUP6_BASE,
	.count   = IRQ_EINT_GROUP6_NR,
	.gpio    = S3C64XX_GPHCON0,
	.irqs    = 4
    },
    {
	.config  = S3C64XX_EINT78CON,
	.filter  = S3C64XX_EINT78FLTCON,
	.mask    = S3C64XX_EINT78MASK,
	.pending = S3C64XX_EINT78PEND,
	.shift   = 0,
	.base    = IRQ_EINT_GROUP7_BASE,
	.count   = IRQ_EINT_GROUP7_NR,
	.gpio    = S3C64XX_GPOCON,
	.irqs    = 4
    },
    {
	.config  = S3C64XX_EINT78CON,
	.filter  = S3C64XX_EINT78FLTCON,
	.mask    = S3C64XX_EINT78MASK,
	.pending = S3C64XX_EINT78PEND,
	.shift   = 16,
	.base    = IRQ_EINT_GROUP8_BASE,
	.count   = IRQ_EINT_GROUP8_NR,
	.gpio    = S3C64XX_GPPCON,
	.irqs    = 4
    },
    {
	.config  = S3C64XX_EINT9CON,
	.filter  = S3C64XX_EINT9FLTCON,
	.mask    = S3C64XX_EINT9MASK,
	.pending = S3C64XX_EINT9PEND,
	.shift   = 0,
	.base    = IRQ_EINT_GROUP9_BASE,
	.count   = IRQ_EINT_GROUP9_NR,
	.gpio    = S3C64XX_GPQCON,
	.irqs    = 4
    }
};

static inline void s3c_irq_grp_mask(unsigned int irq)
{
	u32 mask;
	u32 group;

	if (irq > NR_IRQS)
		return;

	group = group_lookup[irq];
	if (group == 0)
		return;

#ifdef S3C_GRP1_9_DEBUG
	printk("%s: irq=%d, group=%d, mask=0x%08x\n", __func__, irq, group, eint_irq_to_bit(group, irq));
#endif
	mask = __raw_readl(group_regs[group-1].mask);
	mask |= eint_irq_to_bit(group, irq);
	__raw_writel(mask, group_regs[group-1].mask);
}

static void s3c_irq_grp_unmask(unsigned int irq)
{
	u32 mask;
	u32 group;

	if (irq > NR_IRQS)
		return;

	group = group_lookup[irq];
	if (group == 0)
		return;

#ifdef S3C_GRP1_9_DEBUG
	printk("%s: irq=%d, group=%d, mask=0x%08x\n", __func__, irq, group, eint_irq_to_bit(group, irq));
#endif
	mask = __raw_readl(group_regs[group-1].mask);
	mask &= ~(eint_irq_to_bit(group, irq));
	__raw_writel(mask, group_regs[group-1].mask);
}

static inline void s3c_irq_grp_ack(unsigned int irq)
{
	u32 group;

	if (irq > NR_IRQS)
		return;

	group = group_lookup[irq];
	if (group == 0)
		return;

#ifdef S3C_GRP1_9_DEBUG
	printk("%s: irq=%d, group=%d, mask=0x%08x\n", __func__, irq, group, eint_irq_to_bit(group, irq));
#endif
	__raw_writel(eint_irq_to_bit(group, irq), group_regs[group-1].pending);
}

static void s3c_irq_grp_maskack(unsigned int irq)
{
	/* compiler should in-line these */
	s3c_irq_grp_mask(irq);
	s3c_irq_grp_ack(irq);
}

static int s3c_irq_grp_set_type(unsigned int irq, unsigned int type)
{
	int offs = 0;
	int shift;
	u32 ctrl, mask;
	u32 newvalue = 0;
	void __iomem *reg;
	u32 group;
	volatile u32 regval;

	if (irq > NR_IRQS)
		return -EINVAL;

	group = group_lookup[irq];
	if (group == 0)
		return -EINVAL;

	reg = group_regs[group-1].config;
	offs = eint_offset(group, irq);

	switch (type) {
	case IRQ_TYPE_NONE:
#ifdef S3C_GRP1_9_DEBUG
		printk(KERN_WARNING "No edge setting!\n");
#endif
		break;

	case IRQ_TYPE_EDGE_RISING:
		newvalue = S3C2410_EXTINT_RISEEDGE;
#ifdef S3C_GRP1_9_DEBUG
		printk("%s: IRQ(%d), S3C2410_EXTINT_RISEEDGE\n", __func__, irq );
#endif
		break;

	case IRQ_TYPE_EDGE_FALLING:
		newvalue = S3C2410_EXTINT_FALLEDGE;
#ifdef S3C_GRP1_9_DEBUG
		printk("%s: IRQ(%d), S3C2410_EXTINT_FALLEDGE\n", __func__, irq );
#endif
		break;

	case IRQ_TYPE_EDGE_BOTH:
		newvalue = S3C2410_EXTINT_BOTHEDGE;
#ifdef S3C_GRP1_9_DEBUG
		printk("%s: IRQ(%d), S3C2410_EXTINT_BOTHEDGE\n", __func__, irq );
#endif
		break;

	case IRQ_TYPE_LEVEL_LOW:
		newvalue = S3C2410_EXTINT_LOWLEV;
#ifdef S3C_GRP1_9_DEBUG
		printk("%s: IRQ(%d), S3C2410_EXTINT_LOWLEV\n", __func__, irq );
#endif
		break;

	case IRQ_TYPE_LEVEL_HIGH:
		newvalue = S3C2410_EXTINT_HILEV;
#ifdef S3C_GRP1_9_DEBUG
		printk("%s: IRQ(%d), S3C2410_EXTINT_HILEV\n", __func__, irq );
#endif
		break;

	default:
#ifdef S3C_GRP1_9_DEBUG
		printk(KERN_ERR "No such irq type %d", type);
#endif
		return -1;
	}

	// TODO
	// Handle special case of A & B for group 1
	// Later...

	// How many IRQS per config item
	shift = (offs / group_regs[group-1].irqs) * group_regs[group-1].irqs;
	// Shift to bit position and the to upper/lower word
	mask = (0x7 << shift) << group_regs[group-1].shift;

	// Update config with signaling method.
	ctrl = __raw_readl(reg);
	ctrl &= ~mask;
	ctrl |= (newvalue << shift) << group_regs[group-1].shift;
	__raw_writel(ctrl, reg);
#ifdef S3C_GRP1_9_DEBUG
	printk("%s: shift=%d, mask=0x%08x, ctrl=0x%08x\n", __func__, shift, mask, ctrl);
#endif

	// Configure GPIO function
	regval = get_s3c_reg_val(group_regs[group-1].gpio);
	regval |= 0x3 << (offs * 2);
	set_s3c_reg_val(group_regs[group-1].gpio, regval);
#ifdef S3C_GRP1_9_DEBUG
	printk("%s: gpio=0x%08x, offs=%d, func=0x%08x\n", __func__, group_regs[group-1].gpio , offs, 0x3 << (offs * 2));
#endif

	return 0;
}

static struct irq_chip s3c_irq_grp_1_9 = {
	.name		= "s3c-grp129",
	.mask		= s3c_irq_grp_mask,
	.unmask		= s3c_irq_grp_unmask,
	.mask_ack	= s3c_irq_grp_maskack,
	.ack		= s3c_irq_grp_ack,
	.set_type	= s3c_irq_grp_set_type,
};

static void s3c_irq_demux_eint_group(unsigned int __irq, struct irq_desc *desc)
{
	u32 groups;
	u32 spur = 1;

	for (groups = 0; groups < 9; groups++)
	{
    		u32 status = __raw_readl(group_regs[groups].pending);
		u32 mask = __raw_readl(group_regs[groups].mask);
		unsigned int irq;

		status &= ~mask;
		status >>= group_regs[groups].shift;

		for (irq = 0; irq < group_regs[groups].count; irq++)
		{
			if (status & 1)
			{
				generic_handle_irq(group_regs[groups].base + irq);
				spur = 0;
			}
			status >>= 1;
		}
	}
	if (spur)
		printk("%s: Unresolved interrupt\n", __func__);
}

static inline void s3c_irq_init_group(unsigned int group, unsigned int start, unsigned int count)
{
	int irq;

	for (irq = start; irq < (start + count); irq++) {
		set_irq_chip(irq, &s3c_irq_grp_1_9);
		set_irq_handler(irq, handle_level_irq);
		set_irq_flags(irq, IRQF_VALID);
		group_lookup[irq] = group;
	}
}


int __init s3c64xx_init_irq_grp_1_9(void)
{
	printk("Registered interrupt handler for Group 1 - 9\n");
    
	s3c_irq_init_group(1, IRQ_EINT_GROUP(1,0) , IRQ_EINT_GROUP1_NR);
	s3c_irq_init_group(2, IRQ_EINT_GROUP(2,0) , IRQ_EINT_GROUP2_NR);
	s3c_irq_init_group(3, IRQ_EINT_GROUP(3,0) , IRQ_EINT_GROUP3_NR);
	s3c_irq_init_group(4, IRQ_EINT_GROUP(4,0) , IRQ_EINT_GROUP4_NR);
	s3c_irq_init_group(5, IRQ_EINT_GROUP(5,0) , IRQ_EINT_GROUP5_NR);
	s3c_irq_init_group(6, IRQ_EINT_GROUP(6,0) , IRQ_EINT_GROUP6_NR);
	s3c_irq_init_group(7, IRQ_EINT_GROUP(7,0) , IRQ_EINT_GROUP7_NR);
	s3c_irq_init_group(8, IRQ_EINT_GROUP(8,0) , IRQ_EINT_GROUP8_NR);
	s3c_irq_init_group(9, IRQ_EINT_GROUP(9,0) , IRQ_EINT_GROUP9_NR);

	set_irq_chained_handler(IRQ_GROUP1_9, s3c_irq_demux_eint_group);
	
	return 0;
}

arch_initcall(s3c64xx_init_irq_grp_1_9);
