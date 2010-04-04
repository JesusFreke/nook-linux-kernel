/* linux/arch/arm/plat-s3c/include/plat/regs-iis.h
 *
 * Copyright (c) 2003 Simtec Electronics <linux@simtec.co.uk>
 *		      http://www.simtec.co.uk/products/SWLINUX/
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * S3C64XX IIS register definition
*/

#ifndef __ASM_ARCH_REGS_IIS_H
#define __ASM_ARCH_REGS_IIS_H

#define S3C64XX_IIS0CON		(0x00)
#define S3C64XX_IIS0MOD		(0x04)
#define S3C64XX_IIS0FIC		(0x08)
#define S3C64XX_IIS0PSR		(0x0C)
#define S3C64XX_IIS0TXD		(0x10)
#define S3C64XX_IIS0RXD		(0x14)

#define S3C64XX_IIS0MOD_DCE_MASK	(0x3<<16)
#define S3C64XX_IIS0MOD_DCE_SD2		(0x1<<17)
#define S3C64XX_IIS0MOD_DCE_SD1		(0x1<<16)
#define S3C64XX_IIS0MOD_BLC_MASK	(0x3<<13)
#define S3C64XX_IIS0MOD_BLC_16BIT	(0x0<<13)
#define S3C64XX_IIS0MOD_BLC_08BIT	(0x1<<13)
#define S3C64XX_IIS0MOD_BLC_24BIT	(0x2<<13)
#define S3C64XX_IIS0MOD_CLK_MASK	(0x7<<10)
#define S3C64XX_IIS0MOD_INTERNAL_CLK	(0x0<<12)
#define S3C64XX_IIS0MOD_EXTERNAL_CLK	(0x1<<12)
#define S3C64XX_IIS0MOD_IMS_INTERNAL_MASTER	(0x0<<10)
#define S3C64XX_IIS0MOD_IMS_EXTERNAL_MASTER	(0x1<<10)
#define S3C64XX_IIS0MOD_IMS_SLAVE	(0x2<<10)
#define S3C64XX_IIS0MOD_MODE_MASK	(0x3<<8)
#define S3C64XX_IIS0MOD_TXMODE		(0x0<<8)
#define S3C64XX_IIS0MOD_RXMODE		(0x1<<8)
#define S3C64XX_IIS0MOD_TXRXMODE	(0x2<<8)
#define S3C64XX_IIS0MOD_FM_MASK		(0x3<<5)
#define S3C64XX_IIS0MOD_IIS		(0x0<<5)
#define S3C64XX_IIS0MOD_MSB		(0x1<<5)
#define S3C64XX_IIS0MOD_LSB		(0x2<<5)
#define S3C64XX_IIS0MOD_FS_MASK		(0x3<<3)
#define S3C64XX_IIS0MOD_768FS		(0x3<<3)
#define S3C64XX_IIS0MOD_384FS		(0x2<<3)
#define S3C64XX_IIS0MOD_512FS		(0x1<<3)
#define S3C64XX_IIS0MOD_256FS		(0x0<<3)
#define S3C64XX_IIS0MOD_BFS_MASK	(0x3<<1)
#define S3C64XX_IIS0MOD_48FS		(0x1<<1)
#define S3C64XX_IIS0MOD_32FS		(0x0<<1)

#define S3C64XX_IIS0CON_I2SACTIVE	(0x1<<0)
#define S3C64XX_IIS0CON_RXDMACTIVE	(0x1<<1)
#define S3C64XX_IIS0CON_I2SACTIVE	(0x1<<0)
#define S3C64XX_IIS0CON_TXDMACTIVE	(0x1<<2)

#define S3C64XX_IIS_TX_FLUSH	(0x1<<15)
#define S3C64XX_IIS_RX_FLUSH	(0x1<<7)

#define S3C64XX_IISCON_FTXURINTEN 	(0x1<<16)

#define S3C64XX_IIS0MOD_24BIT		(0x2<<13)
#define S3C64XX_IIS0MOD_8BIT		(0x1<<13)
#define S3C64XX_IIS0MOD_16BIT		(0x0<<13)
#endif /* __ASM_ARCH_REGS_IIS_H */
