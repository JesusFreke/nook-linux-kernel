/* arch/arm/mach-s3c6410/include/mach/usb-control.h
 *
 * Copyright (c) 2004 Simtec Electronics
 *	Ben Dooks <ben@simtec.co.uk>
 *
 * S3C6410 - usb port information
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
*/

#ifndef __ASM_ARCH_USBCONTROL_H
#define __ASM_ARCH_USBCONTROL_H

struct regulator;

#define S3C_HCDFLG_USED	(1)

struct s3c6410_hcd_port {
    unsigned char flags;
    unsigned char power;
    unsigned char oc_status;
    unsigned char oc_changed;
};

struct s3c6410_hcd_info {
    struct usb_hcd          *hcd;
    struct s3c6410_hcd_port  port[2];
#ifdef CONFIG_MACH_BRAVO
	/* Turn off these regulators in host part since it suspends
	   after the OTG part */
	struct regulator *otg33v;
	struct regulator *otgi12v;
#endif

    void (*power_control)(int port, int to);
    void (*enable_oc)(struct s3c6410_hcd_info *, int on);
    void (*report_oc)(struct s3c6410_hcd_info *, int ports);
};

static void inline s3c6410_usb_report_oc(struct s3c6410_hcd_info *info, int ports)
{
    if (info->report_oc != NULL)
    {
        (info->report_oc)(info, ports);
    }
}

#endif /*__ASM_ARCH_USBCONTROL_H */
