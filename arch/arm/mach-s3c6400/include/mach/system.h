/* linux/arch/arm/mach-s3c6400/include/mach/system.h
 *
 * Copyright 2008 Openmoko, Inc.
 * Copyright 2008 Simtec Electronics
 *      Ben Dooks <ben@simtec.co.uk>
 *      http://armlinux.simtec.co.uk/
 *
 * S3C6400 - system implementation
 */

#ifndef __ASM_ARCH_SYSTEM_H
#define __ASM_ARCH_SYSTEM_H __FILE__

#include <linux/io.h>

#include <plat/map-base.h>
#include <plat/regs-clock.h>

void (*s3c64xx_idle)(void);
void (*s3c64xx_reset_hook)(void);

void s3c64xx_default_idle(void)
{
	/* nothing here yet */
}
	
static void arch_idle(void)
{
	if (s3c64xx_idle != NULL)
		(s3c64xx_idle)();
	else
		s3c64xx_default_idle();
}

static void arch_reset(char mode)
{
	if (mode == 's') {
		cpu_reset(0);
}

	if (s3c64xx_reset_hook)
		s3c64xx_reset_hook();

	printk("arch_reset: attempting watchdog reset\n");

	__raw_writel(0x6410, S3C_SW_RST);

	/* wait for reset to assert... */
	mdelay(5000);

	printk(KERN_ERR "Watchdog reset failed to assert reset\n");

	/* we'll take a jump through zero as a poor second */
	cpu_reset(0);
}
#endif /* __ASM_ARCH_IRQ_H */
