/* linux/arch/arm/mach-s3c6410/mach-smdk6410.c
 *
 * Copyright 2008 Openmoko, Inc.
 * Copyright 2008 Simtec Electronics
 *  Ben Dooks <ben@simtec.co.uk>
 *  http://armlinux.simtec.co.uk/
 *
 * Modifications 2009 : Intrinsyc Software, Inc on behalf of Barnes and Noble
 * Portions of this code copyright (c) 2009 Barnes and Noble, Inc
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
*/

#include <linux/kernel.h>
#include <linux/types.h>
#include <linux/interrupt.h>
#include <linux/list.h>
#include <linux/timer.h>
#include <linux/init.h>
#include <linux/serial_core.h>
#include <linux/platform_device.h>
#include <linux/io.h>
#include <linux/i2c.h>
#include <linux/mtd/nand.h>
#include <linux/mtd/partitions.h>
#include <linux/synaptics_i2c_rmi.h>

#include <asm/mach/arch.h>
#include <asm/mach/map.h>
#include <asm/mach/irq.h>

#include <mach/gpio.h>
#include <plat/gpio-cfg.h>

#include <mach/hardware.h>
#include <mach/map.h>
#include <mach/regs-mem.h>
#include <mach/gpio-core.h>

#include <asm/setup.h>
#include <asm/irq.h>
#include <asm/mach-types.h>

#include <plat/regs-serial.h>
#include <plat/iic.h>

#include <plat/regs-rtc.h>
#include <plat/regs-clock.h>
#include <plat/regs-gpio.h>

#include <plat/nand.h>
#include <plat/partition.h>
#include <plat/s3c6410.h>
#include <plat/clock.h>
#include <plat/regs-clock.h>
#include <plat/devs.h>
#include <plat/cpu.h>
#include <plat/ts.h>
#include <plat/adc.h>
#include <plat/reserved_mem.h>
#include <plat/pm.h>

#include <linux/android_pmem.h>

#include <mach/gpio.h>
#include <plat/gpio-cfg.h>
#include <mach/bravo_gpio.h>
#include <linux/delay.h>
#include <plat/regs-usb-otg-hs.h>
#include <linux/mfd/wm8350/audio.h>
#include <linux/mfd/wm8350/core.h>

#include <plat/spi-gpio.h>
#include <linux/spi/spi.h>
#include <linux/spi/spi_bitbang.h>

#define UCON S3C2410_UCON_DEFAULT | S3C2410_UCON_UCLK
#define ULCON S3C2410_LCON_CS8 | S3C2410_LCON_PNONE | S3C2410_LCON_STOPB
#define UFCON S3C2410_UFCON_RXTRIG8 | S3C2410_UFCON_FIFOMODE
#define S3C_CLKDIVN_UHOST_MASK    (0xF<<20)
extern struct sys_timer s3c_timer;

// Initialize all GPIO's to their reset (default) state
struct s3c_gpio_reg_default gpio_default_table[] =
{
    /*============REG====CON/CON1=======DAT========PUD========CONSLP======PUDSLP==*/
    S3C64XX_INIT_0(GPA, 0x00220000, 0x00000000, 0x00005555, 0x00000000, 0x00000000), // UART 1
    S3C64XX_INIT_0(GPB, 0x00040000, 0x00000000, 0x00001555, 0x00000000, 0x00000000),
    S3C64XX_INIT_0(GPC, 0x00000000, 0x00000000, 0x00005555, 0x00000000, 0x00000000),
    S3C64XX_INIT_0(GPD, 0x00000000, 0x00000000, 0x00000155, 0x00000000, 0x00000000),
    S3C64XX_INIT_0(GPE, 0x00000000, 0x00000000, 0x00000155, 0x00000000, 0x00000000),
    S3C64XX_INIT_0(GPF, 0x00000000, 0x00000000, 0x55555555, 0x00000000, 0x00000000),
    S3C64XX_INIT_0(GPG, 0x00000000, 0x00000000, 0x00001555, 0x00000000, 0x00000000),
    S3C64XX_INIT_1(GPH, 0x00000000,
                        0x00000000, 0x00000000, 0x00055555, 0x00000000, 0x00000000),
    S3C64XX_INIT_0(GPI, 0x00000000, 0x00000000, 0x55555555, 0x00000000, 0x00000000),
    S3C64XX_INIT_0(GPJ, 0x00000000, 0x00000000, 0x00555555, 0x00000000, 0x00000000),
    S3C64XX_INIT_3(GPK, 0x22222222,
                        0x22222222, 0x00000000, 0x00005555),
    S3C64XX_INIT_3(GPL, 0x22222222,
                        0x02222222, 0x00000000, 0x15555555),
    S3C64XX_INIT_2(GPM, 0x00222222, 0x00000000, 0x000002aa),
    S3C64XX_INIT_2(GPN, 0x00000000, 0x00000000, 0x55555555),
    S3C64XX_INIT_0(GPO, 0xaaaaaaaa, 0x00000000, 0x00000000, 0x00000000, 0x00000000),
    S3C64XX_INIT_0(GPP, 0x2aaaaaaa, 0x00000000, 0x1011aaa0, 0x00000000, 0x00000000),
    S3C64XX_INIT_0(GPQ, 0x0002aaaa, 0x00000000, 0x00555555, 0x00000000, 0x00000000),
};

// Process the GPIO default table.
static void s3c6410_default_gpio(struct s3c_gpio_reg_default *ptr, int count)
{
    printk("[%s]++\n", __FUNCTION__);
    for (; count > 0; count--, ptr++)
    {
       __raw_writel(ptr->val_con, ptr->con);
        if (ptr->con1)
            __raw_writel(ptr->val_con1, ptr->con1);
        if (ptr->pud)
            __raw_writel(ptr->val_pud, ptr->pud);
        if (ptr->conslp)
            __raw_writel(ptr->val_conslp, ptr->conslp);
        if (ptr->pudslp)
            __raw_writel(ptr->val_pudslp, ptr->pudslp);
    }
    printk("[%s]--\n", __FUNCTION__);
}

// Initialize Bravo GPIO's that are used.  Leave unused pins at chip default
struct s3c_gpio_reg gpio_init_table[] =
{
    /*======PIN=========FUNCTION====INIT========PUD ==============SLPCON============SLPPUD=======*/
    {S3C64XX_GPA(4), S3C_GPIO_SFN(0x2), 0, S3C_GPIO_PULL_UP, S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_UP},     
    {S3C64XX_GPA(5), S3C_GPIO_INPUT,    0, S3C_GPIO_PULL_DOWN, S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_DOWN},

    {S3C64XX_GPB(5), S3C_GPIO_SFN(0x2), 0, S3C_GPIO_PULL_NONE, S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, /* I2C_CLK */
    {S3C64XX_GPB(6), S3C_GPIO_SFN(0x2), 0, S3C_GPIO_PULL_NONE, S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, /* I2C_DATA */

    {S3C64XX_GPC(1), S3C_GPIO_SFN(0x2), 0, S3C_GPIO_PULL_NONE, S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE},  /* LCD_SCL ???*/
    {S3C64XX_GPC(2), S3C_GPIO_SFN(0x2), 0, S3C_GPIO_PULL_NONE, S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE},  /* LCD_SDA ???*/
    {S3C64XX_GPC(3), S3C_GPIO_SFN(0x2), 0, S3C_GPIO_PULL_NONE, S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE},  /* LCD_CS ???*/
    {S3C64XX_GPC(4), S3C_GPIO_SFN(0x3), 0, S3C_GPIO_PULL_NONE, S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE},  /* WF_SD_CMD */
    {S3C64XX_GPC(5), S3C_GPIO_SFN(0x3), 0, S3C_GPIO_PULL_NONE, S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE},  /* WF_SD_CLK */
#if 0
    /* These are not currently used */
    {S3C64XX_GPC(6), S3C_GPIO_SFN(0x2), 0, S3C_GPIO_PULL_NONE, S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, /* WF_SPI_SDI */
    {S3C64XX_GPC(7), S3C_GPIO_SFN(0x2), 0, S3C_GPIO_PULL_NONE, S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, /* WF_SPI_CSN */
#endif

    {S3C64XX_GPD(0), S3C_GPIO_SFN(0x3), 0, S3C_GPIO_PULL_NONE, S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, /* I2S_BCLK */
    {S3C64XX_GPD(1), S3C_GPIO_SFN(0x3), 0, S3C_GPIO_PULL_NONE, S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, /* I2S_MCLK */
    {S3C64XX_GPD(2), S3C_GPIO_SFN(0x3), 0, S3C_GPIO_PULL_NONE, S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, /* I2S_LRCK */
    {S3C64XX_GPD(3), S3C_GPIO_SFN(0x3), 0, S3C_GPIO_PULL_NONE, S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, /* I2S_DI */
    {S3C64XX_GPD(4), S3C_GPIO_SFN(0x3), 0, S3C_GPIO_PULL_NONE, S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE},  /* I2S_DO */

    {S3C64XX_GPE(0), S3C_GPIO_INPUT, 0, S3C_GPIO_PULL_NONE, S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE},    /* 2ND_CARD_EN */
    {S3C64XX_GPE(1), S3C_GPIO_OUTPUT, 0, S3C_GPIO_PULL_NONE, S3C_GPIO_SLP_OUT1, S3C_GPIO_PULL_NONE},    /* CARD_EN */
    {S3C64XX_GPE(3), S3C_GPIO_OUTPUT, 0, S3C_GPIO_PULL_NONE, S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE},    /* MUTE */
    {S3C64XX_GPE(4), S3C_GPIO_OUTPUT, 0, S3C_GPIO_PULL_NONE, S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE},    /* HSDPA_PWREN */

    {S3C64XX_GPG(0), S3C_GPIO_SFN(0x2), 0, S3C_GPIO_PULL_NONE, S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE},  /* 2SD_CLK */
    {S3C64XX_GPG(1), S3C_GPIO_SFN(0x2), 0, S3C_GPIO_PULL_NONE, S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE},  /* 2SD_CMD */
    {S3C64XX_GPG(2), S3C_GPIO_SFN(0x2), 0, S3C_GPIO_PULL_NONE, S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE},  /* 2SD_D0 */
    {S3C64XX_GPG(3), S3C_GPIO_SFN(0x2), 0, S3C_GPIO_PULL_NONE, S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE},  /* 2SD_D1 */
    {S3C64XX_GPG(4), S3C_GPIO_SFN(0x2), 0, S3C_GPIO_PULL_NONE, S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE},  /* 2SD_D2 */
    {S3C64XX_GPG(5), S3C_GPIO_SFN(0x2), 0, S3C_GPIO_PULL_NONE, S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE},  /* 2SD_D3 */
    {S3C64XX_GPG(6), S3C_GPIO_INPUT, 0, S3C_GPIO_PULL_NONE, S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE},    /* tied to GND */

    {S3C64XX_GPH(0), S3C_GPIO_SFN(0x2), 0, S3C_GPIO_PULL_NONE, S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE},  /* SD_CLK ???*/
    {S3C64XX_GPH(1), S3C_GPIO_SFN(0x2), 0, S3C_GPIO_PULL_NONE, S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, /* SD_CMD ???*/
    {S3C64XX_GPH(2), S3C_GPIO_SFN(0x2), 0, S3C_GPIO_PULL_NONE, S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, /* SD_D0 ???*/
    {S3C64XX_GPH(3), S3C_GPIO_SFN(0x2), 0, S3C_GPIO_PULL_NONE, S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, /* SD_D1 ???*/
    {S3C64XX_GPH(4), S3C_GPIO_SFN(0x2), 0, S3C_GPIO_PULL_NONE, S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, /* SD_D2 ???*/
    {S3C64XX_GPH(5), S3C_GPIO_SFN(0x2), 0, S3C_GPIO_PULL_NONE, S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, /* SD_D3 ???*/
    {S3C64XX_GPH(6), S3C_GPIO_SFN(0x3), 0, S3C_GPIO_PULL_NONE, S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE},  /* WF_SD_D0 */
    {S3C64XX_GPH(7), S3C_GPIO_SFN(0x3), 0, S3C_GPIO_PULL_NONE, S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE},  /* WF_SD_D1 */
    {S3C64XX_GPH(8), S3C_GPIO_SFN(0x3), 0, S3C_GPIO_PULL_NONE, S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE},  /* WF_SD_D2 */
    {S3C64XX_GPH(9), S3C_GPIO_SFN(0x3), 0, S3C_GPIO_PULL_NONE, S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE},  /* WF_SD_D3 */

    {S3C64XX_GPI(2), S3C_GPIO_SFN(0x2), 0, S3C_GPIO_PULL_NONE, S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE},  /* RGB_B2 */
    {S3C64XX_GPI(3), S3C_GPIO_SFN(0x2), 0, S3C_GPIO_PULL_NONE, S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE},  /* RGB_B3 */
    {S3C64XX_GPI(4), S3C_GPIO_SFN(0x2), 0, S3C_GPIO_PULL_NONE, S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE},  /* RGB_B4 */
    {S3C64XX_GPI(5), S3C_GPIO_SFN(0x2), 0, S3C_GPIO_PULL_NONE, S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE},  /* RGB_B5 */
    {S3C64XX_GPI(6), S3C_GPIO_SFN(0x2), 0, S3C_GPIO_PULL_NONE, S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE},  /* RGB_B6 */
    {S3C64XX_GPI(7), S3C_GPIO_SFN(0x2), 0, S3C_GPIO_PULL_NONE, S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE},  /* RGB_B7 */
    {S3C64XX_GPI(10), S3C_GPIO_SFN(0x2), 0, S3C_GPIO_PULL_NONE, S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, /* RGB_G2 */
    {S3C64XX_GPI(11), S3C_GPIO_SFN(0x2), 0, S3C_GPIO_PULL_NONE, S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, /* RGB_G3 */
    {S3C64XX_GPI(12), S3C_GPIO_SFN(0x2), 0, S3C_GPIO_PULL_NONE, S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, /* RGB_G4 */
    {S3C64XX_GPI(13), S3C_GPIO_SFN(0x2), 0, S3C_GPIO_PULL_NONE, S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, /* RGB_G5 */
    {S3C64XX_GPI(14), S3C_GPIO_SFN(0x2), 0, S3C_GPIO_PULL_NONE, S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, /* RGB_G6 */
    {S3C64XX_GPI(15), S3C_GPIO_SFN(0x2), 0, S3C_GPIO_PULL_NONE, S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, /* RGB_G7 */

    {S3C64XX_GPJ(2), S3C_GPIO_SFN(0x2), 0, S3C_GPIO_PULL_NONE, S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE},  /* RGB_R2 */
    {S3C64XX_GPJ(3), S3C_GPIO_SFN(0x2), 0, S3C_GPIO_PULL_NONE, S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE},  /* RGB_R3 */
    {S3C64XX_GPJ(4), S3C_GPIO_SFN(0x2), 0, S3C_GPIO_PULL_NONE, S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE},  /* RGB_R4 */
    {S3C64XX_GPJ(5), S3C_GPIO_SFN(0x2), 0, S3C_GPIO_PULL_NONE, S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE},  /* RGB_R5 */
    {S3C64XX_GPJ(6), S3C_GPIO_SFN(0x2), 0, S3C_GPIO_PULL_NONE, S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE},  /* RGB_R6 */
    {S3C64XX_GPJ(7), S3C_GPIO_SFN(0x2), 0, S3C_GPIO_PULL_NONE, S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE},  /* RGB_R7 */
    {S3C64XX_GPJ(8), S3C_GPIO_SFN(0x2), 0, S3C_GPIO_PULL_NONE, S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE},  /* HSYNC */
    {S3C64XX_GPJ(9), S3C_GPIO_SFN(0x2), 0, S3C_GPIO_PULL_NONE, S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE},  /* VSYNC */
    {S3C64XX_GPJ(10), S3C_GPIO_SFN(0x2), 0, S3C_GPIO_PULL_NONE, S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, /* LCD_DE */
    {S3C64XX_GPJ(11), S3C_GPIO_SFN(0x2), 0, S3C_GPIO_PULL_NONE, S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, /* LCD_CLK */

    /* Ports K, L, M and N are alive in sleep so have no sleep registers */
    {S3C64XX_GPK(8), S3C_GPIO_INPUT, 0, S3C_GPIO_PULL_NONE, 0, 0},  /* CON3 */
    {S3C64XX_GPK(9),  S3C_GPIO_INPUT, 0, S3C_GPIO_PULL_NONE, 0, 0}, /* CON2 */
    {S3C64XX_GPK(10), S3C_GPIO_INPUT, 0, S3C_GPIO_PULL_NONE, 0, 0}, /* CON1 */
    {S3C64XX_GPK(11),  S3C_GPIO_INPUT, 0, S3C_GPIO_PULL_NONE, 0, 0},    /* CON0 */
    {S3C64XX_GPK(12),  S3C_GPIO_OUTPUT, 0, S3C_GPIO_PULL_NONE, 0, 0}, /* TFT LED enable */
    {S3C64XX_GPK(13),  S3C_GPIO_OUTPUT, 0, S3C_GPIO_PULL_NONE, 0, 0}, /* TFT LED enable */
    {S3C64XX_GPK(14), S3C_GPIO_INPUT, 0, S3C_GPIO_PULL_NONE, 0, 0}, /* Panel_detect2 */
    {S3C64XX_GPK(15),  S3C_GPIO_INPUT, 0, S3C_GPIO_PULL_NONE, 0, 0},    /* Panel_detect1 */

    {S3C64XX_GPL(5),  S3C_GPIO_OUTPUT, 0, S3C_GPIO_PULL_NONE, 0, 0},    /* 3G_RESET */
    {S3C64XX_GPL(6),  S3C_GPIO_INPUT, 0, S3C_GPIO_PULL_NONE, 0, 0}, /* PWRGD */
#if 0
    /* not currently used */
    {S3C64XX_GPL(7),  S3C_GPIO_INPUT, 0, S3C_GPIO_PULL_NONE, 0, 0}, /* WF_SPI_INTN */
#endif
    {S3C64XX_GPL(8),  S3C_GPIO_OUTPUT, 0, S3C_GPIO_PULL_NONE, 0, 0},    /* WF_MAC_WAKE */
    {S3C64XX_GPL(9),  S3C_GPIO_INPUT, 0, S3C_GPIO_PULL_NONE, 0, 0}, /* WF_HOST_WAKE */
    {S3C64XX_GPL(11),  S3C_GPIO_SFN(0x3), 0, S3C_GPIO_PULL_NONE, 0, 0}, /* TOUCH_INT */
    {S3C64XX_GPL(12),  S3C_GPIO_OUTPUT, 0, S3C_GPIO_PULL_NONE, 0, 0},   /* W_DISABLEn */
#if 0
    /* not currently used */
    {S3C64XX_GPL(13),  S3C_GPIO_OUTPUT, 0, S3C_GPIO_PULL_NONE, 0, 0},   /* PWR_HOLD */
#endif
    {S3C64XX_GPL(14),  S3C_GPIO_INPUT, 0, S3C_GPIO_PULL_NONE, 0, 0},    /* CHARGE_FAIL */

    {S3C64XX_GPM(0),  S3C_GPIO_OUTPUT, 0, S3C_GPIO_PULL_NONE, 0, 0},    /* ChargeCHK */
    {S3C64XX_GPM(1),  S3C_GPIO_INPUT, 0, S3C_GPIO_PULL_NONE, 0, 0}, /* KEY4 */
    {S3C64XX_GPM(3),  S3C_GPIO_INPUT, 0, S3C_GPIO_PULL_NONE, 0, 0}, /* KEY3 */
    {S3C64XX_GPM(4),  S3C_GPIO_OUTPUT, 1, S3C_GPIO_PULL_NONE, 0, 0}, /* LCD_EN */
    {S3C64XX_GPM(5),  S3C_GPIO_INPUT, 0, S3C_GPIO_PULL_NONE, 0, 0}, /* USB5V_SENSE */

    {S3C64XX_GPN(0),  S3C_GPIO_SFN(0x2), 0, S3C_GPIO_PULL_NONE, 0, 0},  /* 8350 IRQ */
    {S3C64XX_GPN(1),  S3C_GPIO_SFN(0x2), 0, S3C_GPIO_PULL_NONE, 0, 0},  /* SYS_ON */
    {S3C64XX_GPN(2),  S3C_GPIO_OUTPUT, 0, S3C_GPIO_PULL_NONE, 0, 0},    /* Charge_en */
#if 0
    /* not currently used */
    {S3C64XX_GPN(3),  S3C_GPIO_OUTPUT, 0, S3C_GPIO_PULL_NONE, 0, 0},    /* WF_PWR_DNn */
#endif
    {S3C64XX_GPN(4),  S3C_GPIO_OUTPUT, 0, S3C_GPIO_PULL_NONE, 0, 0},    /* WF_RSTN */
    {S3C64XX_GPN(5),  S3C_GPIO_OUTPUT, 0, S3C_GPIO_PULL_NONE, 0, 0},    /* WF_PWR_EN */
    {S3C64XX_GPN(6),  S3C_GPIO_INPUT, 0, S3C_GPIO_PULL_NONE, 0, 0}, /* KEY1 */
    {S3C64XX_GPN(7),  S3C_GPIO_INPUT, 0, S3C_GPIO_PULL_NONE, 0, 0}, /* HALL_DET */
    {S3C64XX_GPN(8),  S3C_GPIO_INPUT, 0, S3C_GPIO_PULL_NONE, 0, 0}, /* KEY2 */
    {S3C64XX_GPN(9),  S3C_GPIO_OUTPUT, 0, S3C_GPIO_PULL_NONE, 0, 0},    /* LCD_RST */
    {S3C64XX_GPN(10), S3C_GPIO_INPUT, 0, S3C_GPIO_PULL_NONE, 0, 0}, /* HP_IN */
    {S3C64XX_GPN(11), S3C_GPIO_INPUT, 0, S3C_GPIO_PULL_NONE, 0, 0}, /* WAKEUP_AP */
    {S3C64XX_GPN(12), S3C_GPIO_OUTPUT, 1, S3C_GPIO_PULL_NONE, 0, 0},    /* LIGHT_PD */

    {S3C64XX_GPO(4), S3C_GPIO_OUTPUT, 1, S3C_GPIO_PULL_NONE, S3C_GPIO_SLP_OUT1, S3C_GPIO_PULL_NONE},    /* HnRST */
    {S3C64XX_GPO(5), S3C_GPIO_OUTPUT, 1, S3C_GPIO_PULL_NONE, S3C_GPIO_SLP_OUT1, S3C_GPIO_PULL_NONE},    /* HDC */
    {S3C64XX_GPO(9), S3C_GPIO_OUTPUT, 1, S3C_GPIO_PULL_NONE, S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE},    /* OSC_EN */

    {S3C64XX_GPP(2), S3C_GPIO_INPUT, 0, S3C_GPIO_PULL_NONE, S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE},    /* HRDY */
};

// Process the Bravo GPIO table.
static void s3c6410_init_gpio(struct s3c_gpio_reg *ptr, int count)
{
    printk("[%s]++\n", __FUNCTION__);
    for (; count > 0; count--, ptr++)
    {
        s3c_gpio_cfgpin(ptr->reg_addr, ptr->normal_function);
        s3c_gpio_setpull(ptr->reg_addr, ptr->normal_pull);
        s3c_gpio_slp_cfgpin(ptr->reg_addr, ptr->sleep_function);
        s3c_gpio_slp_setpull_updown(ptr->reg_addr, ptr->sleep_pull);
        s3c_gpdat_setval(ptr->reg_addr, ptr->init_val);
    }
    printk("[%s]--\n", __FUNCTION__);
}

void s3c_otg_soft_connect(int connect);

static struct s3c2410_uartcfg smdk6410_uartcfgs[] __initdata = {
    [0] = {
        .hwport      = 0,
        .flags       = 0,
        .ucon        = S3C64XX_UCON_DEFAULT,
        .ulcon       = S3C64XX_ULCON_DEFAULT,
        .ufcon       = S3C64XX_UFCON_DEFAULT,
    },
    [1] = {
        .hwport      = 1,
        .flags       = 0,
        .ucon        = S3C64XX_UCON_DEFAULT,
        .ulcon       = S3C64XX_ULCON_DEFAULT,
        .ufcon       = S3C64XX_UFCON_DEFAULT,
    },
};

struct map_desc smdk6410_iodesc[] = {};

struct platform_device sec_device_backlight = {
    .name   = "smdk-backlight",
    .id     = -1,
};

struct platform_device bravo_modem_mgr_device = {
    .name   = "bravo_modem_mgr",
    .id     = -1,
};

static struct s3c6410_pmem_setting pmem_setting = {
    .pmem_start = RESERVED_PMEM_START,
    .pmem_size = RESERVED_PMEM,
#ifdef CONFIG_VIDEO_G3D
    .pmem_g3d_start = G3D_RESERVED_PMEM_START,
    .pmem_g3d_size = RESERVED_PMEM_G3D,
    .pmem_gpu1_start = GPU1_RESERVED_PMEM_START,
    .pmem_gpu1_size = RESERVED_PMEM_GPU1,
#endif /* CONFIG_VIDEO_G3D */
    .pmem_render_start = RENDER_RESERVED_PMEM_START,
    .pmem_render_size = RESERVED_PMEM_RENDER,
    .pmem_stream_start = STREAM_RESERVED_PMEM_START,
    .pmem_stream_size = RESERVED_PMEM_STREAM,
    .pmem_preview_start = PREVIEW_RESERVED_PMEM_START,
    .pmem_preview_size = RESERVED_PMEM_PREVIEW,
    .pmem_picture_start = PICTURE_RESERVED_PMEM_START,
    .pmem_picture_size = RESERVED_PMEM_PICTURE,
    .pmem_jpeg_start = JPEG_RESERVED_PMEM_START,
    .pmem_jpeg_size = RESERVED_PMEM_JPEG,
};

#ifdef CONFIG_MACH_BRAVO_EVT15

static struct s3c64xx_spigpio_info spi_gpio_cfg = {
    .pin_clk = S3C64XX_GPC(1),
    .pin_mosi = S3C64XX_GPC(2),
    .pin_miso = S3C64XX_GPC(0),
    .pin_cs = S3C64XX_GPC(3),
    .bus_num = 0,
};

static struct platform_device s3c64xx_spi = {
    .name = "spi-s3c64xx-gpio",
    .id = -1,
    .dev = {
        .platform_data = &spi_gpio_cfg,
    },
};

#endif /* CONFIG_MACH_BRAVO_EVT15 */

#ifdef CONFIG_FB_BRAVO
static u64 bravo_vfb_dmamask = 0xffffffffUL;

struct platform_device bravo_vfb = {
    .name = "bravo-vfb",
    .id = -1,
    .dev = {
        .dma_mask = &bravo_vfb_dmamask,
        .coherent_dma_mask = 0xffffffffUL
    }
};

#endif /* CONFIG_FB_BRAVO */

static struct platform_device *smdk6410_devices[] __initdata = {
#ifdef CONFIG_SMDK6410_SD_CH0
    &s3c_device_hsmmc0,
#endif
#ifdef CONFIG_SMDK6410_SD_CH1
    &s3c_device_hsmmc1,
#endif
#ifdef CONFIG_SMDK6410_SD_CH2
    &s3c_device_hsmmc2,
#endif
    &s3c_device_i2c0,
#ifdef CONFIG_S3C_DEV_I2C1
    &s3c_device_i2c1,
#endif
#ifdef CONFIG_FB_BRAVO
    &bravo_vfb,
#endif
#ifdef CONFIG_TOUCHSCREEN_S3C
    &s3c_device_ts,
#endif
#ifndef CONFIG_MACH_BRAVO
    &s3c_device_smc911x,
    &s3c_device_nand,
#endif
    &s3c_device_lcd,
    &s3c_device_keypad,
    &s3c_device_usb,
    &s3c_device_usbgadget,
#ifdef CONFIG_S3C64XX_ADC
    &s3c_device_adc,
#endif
#ifdef CONFIG_RTC_DRV_S3C
    &s3c_device_rtc,
#endif
#ifdef CONFIG_VIDEO_G2D
    &s3c_device_2d,
#endif
#ifdef CONFIG_VIDEO_FIMC
    &s3c_device_fimc0,
    &s3c_device_fimc1,
#endif
#if CONFIG_VIDEO_CAM
    &s3c_device_camif,
#endif
#ifdef CONFIG_VIDEO_MFC
    &s3c_device_mfc,
#endif
#ifdef CONFIG_VIDEO_G3D
    &s3c_device_g3d,
#endif
#ifdef CONFIG_VIDEO_ROTATOR
    &s3c_device_rotator,
#endif
#ifdef CONFIG_VIDEO_JPEG
    &s3c_device_jpeg,
#endif
    &s3c_device_vpp,
#ifdef CONFIG_VIDEO_TVOUT
    &s3c_device_tvenc,
#endif
#ifdef CONFIG_VIDEO_SCALER
    &s3c_device_tvscaler,
#endif
    &sec_device_backlight,
    &s3c_eink_device,
    &s3c_device_button,
#ifdef CONFIG_BRAVO_MODEM_MGR
    &bravo_modem_mgr_device,
#endif /* CONFIG_BRAVO_MODEM_MGR */
#ifdef CONFIG_MACH_BRAVO_EVT15
    &s3c64xx_spi,
#endif /* CONFIG_MACH_BRAVO_EVT15 */
#ifdef CONFIG_SND_S3C64XX_SOC_I2S
    &s3c_device_iis,
#endif
};

static struct wm8350_audio_platform_data smdk6410_wm8350_setup = {
        .vmid_discharge_msecs = 1000,
        .drain_msecs = 30,
        .cap_discharge_msecs = 700,
        .vmid_charge_msecs = 700,
        .vmid_s_curve = WM8350_S_CURVE_SLOW,
        .dis_out4 = WM8350_DISCHARGE_SLOW,
        .dis_out3 = WM8350_DISCHARGE_SLOW,
        .dis_out2 = WM8350_DISCHARGE_SLOW,
        .dis_out1 = WM8350_DISCHARGE_SLOW,
        .vroi_out4 = WM8350_TIE_OFF_500R,
        .vroi_out3 = WM8350_TIE_OFF_500R,
        .vroi_out2 = WM8350_TIE_OFF_500R,
        .vroi_out1 = WM8350_TIE_OFF_500R,
        .vroi_enable = 0,
        .codec_current_on = WM8350_CODEC_ISEL_1_0,
        .codec_current_standby = WM8350_CODEC_ISEL_0_5,
        .codec_current_charge = WM8350_CODEC_ISEL_1_5,
};

static struct regulator_init_data ldo2_initdata = {
    .supply_regulator_dev = NULL,
    .constraints = {
        .name = "OTG_3.3V",
        .min_uV = 3300000,
        .max_uV = 3300000,
        .apply_uV = 1,
        .boot_on = 1,
        .valid_modes_mask = REGULATOR_MODE_NORMAL,
        .valid_ops_mask = REGULATOR_CHANGE_VOLTAGE | REGULATOR_CHANGE_STATUS,
        .state_mem = { .uV = 0, .mode = REGULATOR_MODE_NORMAL, .enabled = 0, },
        .initial_state = PM_SUSPEND_ON,
    },
    .num_consumer_supplies = 0,
    .consumer_supplies = NULL,
};

static struct regulator_init_data ldo4_initdata = {
    .supply_regulator_dev = NULL,
    .constraints = {
        .name = "OTGI_1.2V",
        .min_uV = 1200000,
        .max_uV = 1200000,
        .apply_uV = 1,
        .boot_on = 1,
        .valid_modes_mask = REGULATOR_MODE_NORMAL,
        .valid_ops_mask = REGULATOR_CHANGE_VOLTAGE | REGULATOR_CHANGE_STATUS,
        .state_mem = { .uV = 0, .mode = REGULATOR_MODE_NORMAL, .enabled = 0, },
        .initial_state = PM_SUSPEND_ON,
    },
    .num_consumer_supplies = 0,
    .consumer_supplies = NULL,
};

static struct regulator_init_data dcdc1_initdata = {
	.supply_regulator_dev = NULL,
	.constraints = {
		.name = "VCC_1.2V",
		.min_uV = 1000000,
		.max_uV = 1200000,
		.apply_uV = 1,
		.boot_on = 1,
		.valid_modes_mask = REGULATOR_MODE_NORMAL,
		.valid_ops_mask = REGULATOR_CHANGE_VOLTAGE,
		.state_mem = { .uV = 1050000, .mode = REGULATOR_MODE_NORMAL, .enabled = 1, },
		.initial_state = PM_SUSPEND_ON,
	},
	.num_consumer_supplies = 0,
	.consumer_supplies = NULL,
};

static struct regulator_init_data dcdc6_initdata = {
	.supply_regulator_dev = NULL,
	.constraints = {
		.name = "VCC_ARM_1.1V",
		.min_uV = 1000000,
		.max_uV = 1100000,
		.apply_uV = 1,
		.boot_on = 1,
		.valid_modes_mask = REGULATOR_MODE_NORMAL,
		.valid_ops_mask = REGULATOR_CHANGE_VOLTAGE,
		.state_mem = { .uV = 1050000, .mode = REGULATOR_MODE_NORMAL, .enabled = 1, },
		.initial_state = PM_SUSPEND_ON,
	},
	.num_consumer_supplies = 0,
	.consumer_supplies = NULL,
};

static int __init smdk6410_wm8350_init(struct wm8350 *wm8350)
{
    printk(KERN_ERR "smdk6410_wm8350_init: Setting platform data **********************************\n");
    wm8350->codec.platform_data = &smdk6410_wm8350_setup;

	wm8350_register_regulator(wm8350, WM8350_DCDC_1, &dcdc1_initdata);
	wm8350_dcdc_set_slot(wm8350, WM8350_DCDC_1, 0x1, 0xe, 0xe);
	wm8350_register_regulator(wm8350, WM8350_DCDC_6, &dcdc6_initdata);
	wm8350_dcdc_set_slot(wm8350, WM8350_DCDC_6, 0x1, 0xe, 0xe);

    wm8350_register_regulator(wm8350, WM8350_LDO_2, &ldo2_initdata);
    wm8350_ldo_set_slot(wm8350, WM8350_LDO_2, 0xf, 0xf);
    wm8350_register_regulator(wm8350, WM8350_LDO_4, &ldo4_initdata);
    wm8350_ldo_set_slot(wm8350, WM8350_LDO_4, 0xf, 0xf);

	if (get_s3c_reg_val(S3C64XX_GPKDAT) & 0x800) {
		wm8350_gpio_config(wm8350, 6, WM8350_GPIO_DIR_IN, 0x1, 0, WM8350_GPIO_PULL_NONE, WM8350_GPIO_INVERT_ON, WM8350_GPIO_DEBOUNCE_OFF);

		wm8350_set_bits(wm8350, WM8350_DCDC1_LOW_POWER, 0x7200);
		wm8350_set_bits(wm8350, WM8350_DCDC3_LOW_POWER, 0x2200);
		wm8350_set_bits(wm8350, WM8350_DCDC4_LOW_POWER, 0x2200);
		wm8350_set_bits(wm8350, WM8350_DCDC6_LOW_POWER, 0x7200);
		wm8350_set_bits(wm8350, WM8350_DCDC2_CONTROL, 0x1200);
		wm8350_set_bits(wm8350, WM8350_DCDC5_CONTROL, 0x1200);
		wm8350_set_bits(wm8350, WM8350_LDO2_LOW_POWER, 0x1200); 
		wm8350_set_bits(wm8350, WM8350_LDO3_LOW_POWER, 0x1200); 
		wm8350_set_bits(wm8350, WM8350_LDO4_LOW_POWER, 0x1200);
	}

    return 0;
}

static struct wm8350_platform_data __initdata wm8350_pdata = {
        .init = smdk6410_wm8350_init,
};

#ifdef CONFIG_MACH_BRAVO_I2C_MULTICLK

static struct synaptics_i2c_rmi_platform_data synaptics_pdata = {
    .version = 0,
    .power = NULL,
    .flags = SYNAPTICS_FLIP_Y,
    .inactive_left = 0x0,
    .inactive_right = 0x0,
    .inactive_top = 0x0,
    .inactive_bottom = 0x0,
    .snap_left_on = 0x0,
    .snap_left_off = 0x0,
    .snap_right_on = 0x0,
    .snap_right_off = 0x0,
    .snap_top_on = 0x0,
    .snap_top_off = 0x0,
    .snap_bottom_on = 0x0,
    .snap_bottom_off = 0x0,
    .fuzz_x = 0x0,
    .fuzz_y = 0x0,
    .fuzz_p = 0,
    .fuzz_w = 0,
};

//Note:  s3c_i2c_client_platdata is i2c client's platform_data
static struct s3c_i2c_client_platdata i2c_devs0_platdata[] __initdata = {
    { 0x1a, 50, 50, 50, 50, 0x0a, &wm8350_pdata},
    { 0x55, 260, 4000, 50, 4000, 0x41, NULL},
    { 0x44, 50, 50, 50, 50, 0x0a, NULL},
    { 0x20, 50, 50, 50, 50, 0x0a, &synaptics_pdata},
    { 0x50, 50, 50, 50, 50, 0x0a, NULL},
};

// The .platform_data entries of this array are set at run time to the array above
struct i2c_board_info i2c_devs0[] __initdata = {
    { I2C_BOARD_INFO("wm8350", 0x1a), .irq = IRQ_EINT(0), },
    { I2C_BOARD_INFO("bq27200",  0x55), },
#ifdef CONFIG_MACH_BRAVO_EVT1
    { I2C_BOARD_INFO("isl29001",  0x44), },
    { I2C_BOARD_INFO(SYNAPTICS_I2C_RMI_NAME, 0x20), .platform_data = &synaptics_pdata, .irq = IRQ_EINT(19), },
#endif
    { I2C_BOARD_INFO("24c08",  0x50), },
};
#else
static struct i2c_board_info i2c_devs0[] __initdata = {
    { I2C_BOARD_INFO("wm8350", 0x1a), .irq = IRQ_EINT(0), },
    { I2C_BOARD_INFO("bq27200",  0x55), },
    { I2C_BOARD_INFO("24c08",  0x50), },
};
#endif

static struct i2c_board_info i2c_devs1[] __initdata = {
    { I2C_BOARD_INFO("24c128", 0x57), },    /* Samsung S524AD0XD1 */
    { I2C_BOARD_INFO("WM8580", 0x1b), },
};

#ifdef CONFIG_MACH_BRAVO_EVT15
static struct spi_board_info spi_board_info[] __initdata = {
    {
    .modalias       = "pictube-spi",
    .platform_data  = NULL,
    .mode           = SPI_MODE_0,
    .irq            = 0,
    .max_speed_hz   = 10000000, /* 10 MHz bus */
    .bus_num        = 0,
    .chip_select    = 0,
    },
};
#endif

#ifdef CONFIG_TOUCHSCREEN_S3C
static struct s3c_ts_mach_info s3c_ts_platform __initdata = {
    .delay          = 10000,
    .presc          = 49,
    .oversampling_shift = 2,
    .resol_bit      = 12,
    .s3c_adc_con        = ADC_TYPE_2,
};
#endif

static struct s3c_adc_mach_info s3c_adc_platform = {
    /* s3c6410 support 12-bit resolution */
    .delay  =   10000,
    .presc  =   49,
    .resolution =   12,
};

static void __init smdk6410_map_io(void)
{
    s3c_device_nand.name = "s3c6410-nand";

    s3c64xx_init_io(smdk6410_iodesc, ARRAY_SIZE(smdk6410_iodesc));
    s3c64xx_gpiolib_init();
    s3c24xx_init_clocks(12000000);
    s3c24xx_init_uarts(smdk6410_uartcfgs, ARRAY_SIZE(smdk6410_uartcfgs));
}

static void smdk6410_set_qos(void)
{
    u32 reg;                                 /* AXI sfr */

    reg = (u32) ioremap((unsigned long) S3C6410_PA_AXI_SYS, SZ_4K); /* QoS override: FIMD min. latency */
    writel(0x2, S3C_VA_SYS + 0x128);                    /* AXI QoS */
    writel(0x7, reg + 0x460);                       /* (8 - MFC ch.) */
    writel(0x7ff7, reg + 0x464);                    /* Bus cacheable */
    writel(0x8ff, S3C_VA_SYS + 0x838);

    __raw_writel(0x0, S3C_AHB_CON0);
}

static void __init smdk6410_smc911x_set(void)
{
    unsigned int tmp;

    tmp = __raw_readl(S3C64XX_SROM_BW);
    tmp &= ~(S3C64XX_SROM_BW_WAIT_ENABLE1_MASK | S3C64XX_SROM_BW_WAIT_ENABLE1_MASK |
        S3C64XX_SROM_BW_DATA_WIDTH1_MASK);
    tmp |= S3C64XX_SROM_BW_BYTE_ENABLE1_ENABLE | S3C64XX_SROM_BW_WAIT_ENABLE1_ENABLE |
        S3C64XX_SROM_BW_DATA_WIDTH1_16BIT;

    __raw_writel(tmp, S3C64XX_SROM_BW);

    __raw_writel(S3C64XX_SROM_BCn_TACS(0) | S3C64XX_SROM_BCn_TCOS(4) |
            S3C64XX_SROM_BCn_TACC(13) | S3C64XX_SROM_BCn_TCOH(1) |
            S3C64XX_SROM_BCn_TCAH(4) | S3C64XX_SROM_BCn_TACP(6) |
            S3C64XX_SROM_BCn_PMC_NORMAL, S3C64XX_SROM_BC1);
}

static void __init smdk6410_fixup (struct machine_desc *desc, struct tag *tags,
          char **cmdline, struct meminfo *mi)
{
    /*
     * Bank start addresses are not present in the information
     * passed in from the boot loader.  We could potentially
     * detect them, but instead we hard-code them.
     */
    mi->nr_banks = 1;
    mi->bank[0].start = PHYS_OFFSET;
    mi->bank[0].size = PHYS_UNRESERVED_SIZE;
    mi->bank[0].node = 0;
}

void __init s3c_i2cdev0_set_platdata(void)
{
    int i,j;

    for (i = 0; i < ARRAY_SIZE(i2c_devs0); i++ )
    {
        for (j = 0; j < ARRAY_SIZE(i2c_devs0_platdata); j++)
            if (i2c_devs0_platdata[j].slave_addr == i2c_devs0[i].addr)
                i2c_devs0[i].platform_data = &(i2c_devs0_platdata[j]);
    }
}
extern unsigned int get_s3c_reg_val(void __iomem *reg_addr);

static void __init smdk6410_machine_init(void)
{
    // Process the GPIO default table.
    s3c6410_default_gpio(gpio_default_table, ARRAY_SIZE(gpio_default_table));
    // Process the Bravo GPIO table.
    s3c6410_init_gpio(gpio_init_table, ARRAY_SIZE(gpio_init_table));

    // The WM8580 will tell USB when to connect.
    s3c_otg_soft_connect(0);

    s3c_device_nand.dev.platform_data = &s3c_nand_mtd_part_info;

    smdk6410_smc911x_set();

    s3c_i2c0_set_platdata(NULL);
#ifdef CONFIG_S3C_DEV_I2C1
    s3c_i2c1_set_platdata(NULL);
#endif

#ifdef CONFIG_MACH_BRAVO_I2C_MULTICLK
    //TODO -- If I2C channel 1 has multiple clients attached with different CLK requirement,
    //We should also use s3c_i2c_client_platdata to configure CLK requirement for different i2c clients on the same bus
    s3c_i2cdev0_set_platdata();
#endif

#ifdef CONFIG_TOUCHSCREEN_S3C
    s3c_ts_set_platdata(&s3c_ts_platform);
#endif
    s3c_adc_set_platdata(&s3c_adc_platform);

    s3c_gpio_setpull(S3C64XX_GPN(0), S3C_GPIO_PULL_UP);
    i2c_register_board_info(0, i2c_devs0, ARRAY_SIZE(i2c_devs0));
    i2c_register_board_info(1, i2c_devs1, ARRAY_SIZE(i2c_devs1));

#ifdef CONFIG_MACH_BRAVO_EVT15
    spi_register_board_info(spi_board_info, ARRAY_SIZE(spi_board_info));
#endif

#ifdef CONFIG_MACH_BRAVO
    s3c6410_enable_wifi_pins();
    printk(KERN_NOTICE "smdk6410_machine_init: enable wifi pins(jiffies %lu)\n", jiffies);
    s3c6410_reset_wifi();
    {
        volatile u32 regval = 0;
        regval = get_s3c_reg_val(S3C64XX_GPN_BASE);
        printk(KERN_NOTICE "GPNCON(0x%x)\n", regval);
        regval = get_s3c_reg_val(S3C64XX_GPN_BASE+0x4);
        printk(KERN_NOTICE "GPNDAT(0x%x)\n", regval);
        regval = get_s3c_reg_val(S3C64XX_GPN_BASE+0x8);
        printk(KERN_NOTICE "GPNPUD(0x%x)\n", regval);

        regval = get_s3c_reg_val(S3C64XX_GPE_BASE);
        printk(KERN_NOTICE "GPECON(0x%x)\n", regval);
        regval = get_s3c_reg_val(S3C64XX_GPE_BASE+0x4);
        printk(KERN_NOTICE "GPEDAT(0x%x)\n", regval);
        regval = get_s3c_reg_val(S3C64XX_GPE_BASE+0x8);
        printk(KERN_NOTICE "GPEPUD(0x%x)\n", regval);
    }
#ifdef CONFIG_MACH_BRAVO_EVT1
    s3c6410_enable_lightsensor();  //Power up Light Sensor
#endif //EVT1
#endif //BRAVO

    platform_add_devices(smdk6410_devices, ARRAY_SIZE(smdk6410_devices));
    s3c6410_add_mem_devices (&pmem_setting);
    s3c6410_pm_init();

    smdk6410_set_qos();
}

MACHINE_START(SMDK6410, "SMDK6410")
    /* Maintainer: Ben Dooks <ben@fluff.org> */
    .phys_io    = S3C_PA_UART & 0xfff00000,
    .io_pg_offst    = (((u32)S3C_VA_UART) >> 18) & 0xfffc,
    .boot_params    = S3C64XX_PA_SDRAM + 0x100,
    .fixup      = smdk6410_fixup,
    .init_irq   = s3c6410_init_irq,
    .map_io     = smdk6410_map_io,
    .init_machine   = smdk6410_machine_init,
    .timer      = &s3c64xx_timer,
MACHINE_END

#if defined(CONFIG_RTC_DRV_S3C)
/* RTC common Function for samsung APs*/
unsigned int s3c_rtc_set_bit_byte(void __iomem *base, uint offset, uint val)
{
    writeb(val, base + offset);

    return 0;
}

unsigned int s3c_rtc_read_alarm_status(void __iomem *base)
{
    return 1;
}

void s3c_rtc_set_pie(void __iomem *base, uint to)
{
    unsigned int tmp;

    tmp = readw(base + S3C2410_RTCCON) & ~S3C_RTCCON_TICEN;

        if (to)
                tmp |= S3C_RTCCON_TICEN;

        writew(tmp, base + S3C2410_RTCCON);
}

void s3c_rtc_set_freq_regs(void __iomem *base, uint freq, uint s3c_freq)
{
    unsigned int tmp;

        tmp = readw(base + S3C2410_RTCCON) & (S3C_RTCCON_TICEN | S3C2410_RTCCON_RTCEN );
        writew(tmp, base + S3C2410_RTCCON);
        s3c_freq = freq;
        tmp = (32768 / freq)-1;
        writel(tmp, base + S3C2410_TICNT);
}

void s3c_rtc_enable_set(struct platform_device *pdev,void __iomem *base, int en)
{
    unsigned int tmp;

    if (!en) {
        tmp = readw(base + S3C2410_RTCCON);
        writew(tmp & ~ (S3C2410_RTCCON_RTCEN | S3C_RTCCON_TICEN), base + S3C2410_RTCCON);
    } else {
        /* re-enable the device, and check it is ok */
        if ((readw(base+S3C2410_RTCCON) & S3C2410_RTCCON_RTCEN) == 0){
            dev_info(&pdev->dev, "rtc disabled, re-enabling\n");

            tmp = readw(base + S3C2410_RTCCON);
            writew(tmp|S3C2410_RTCCON_RTCEN, base+S3C2410_RTCCON);
        }

        if ((readw(base + S3C2410_RTCCON) & S3C2410_RTCCON_CNTSEL)){
            dev_info(&pdev->dev, "removing RTCCON_CNTSEL\n");

            tmp = readw(base + S3C2410_RTCCON);
            writew(tmp& ~S3C2410_RTCCON_CNTSEL, base+S3C2410_RTCCON);
        }

        if ((readw(base + S3C2410_RTCCON) & S3C2410_RTCCON_CLKRST)){
            dev_info(&pdev->dev, "removing RTCCON_CLKRST\n");

            tmp = readw(base + S3C2410_RTCCON);
            writew(tmp & ~S3C2410_RTCCON_CLKRST, base+S3C2410_RTCCON);
        }
    }
}
#endif

#if defined(CONFIG_KEYPAD_S3C) || defined (CONFIG_KEYPAD_S3C_MODULE)
void s3c_setup_keypad_cfg_gpio(int rows, int columns)
{
    unsigned int gpio;
    unsigned int end;

    end = S3C64XX_GPK(8 + rows);

    /* Set all the necessary GPK pins to special-function 0 */
    for (gpio = S3C64XX_GPK(8); gpio < end; gpio++) {
        s3c_gpio_cfgpin(gpio, S3C_GPIO_SFN(3));
        s3c_gpio_setpull(gpio, S3C_GPIO_PULL_NONE);
    }

    end = S3C64XX_GPL(0 + columns);

    /* Set all the necessary GPK pins to special-function 0 */
    for (gpio = S3C64XX_GPL(0); gpio < end; gpio++) {
        s3c_gpio_cfgpin(gpio, S3C_GPIO_SFN(3));
        s3c_gpio_setpull(gpio, S3C_GPIO_PULL_NONE);
    }
}

EXPORT_SYMBOL(s3c_setup_keypad_cfg_gpio);
#endif
//added by Nas
extern void otg_dump_regs(void);
void otg_phy_init(u32 otg_phy_clk)
{
	int flags;

#ifdef CONFIG_USBHOST_CLOCK_EPLL
	otg_dump_regs();
#endif

	local_irq_save(flags);

	writel(readl(S3C_OTHERS) | S3C_OTHERS_USB_SIG_MASK, S3C_OTHERS);
	writel(readl(S3C_HCLK_GATE) | (S3C_CLKCON_HCLK_USB), S3C_HCLK_GATE);
#ifdef CONFIG_USBHOST_CLOCK_EPLL
	writel((0x0), S3C_USBOTG_PHYPWR); //power up OTG block, analog block, force_suspend disable
	writel(0x2, S3C_USBOTG_PHYCLK); //OPHYCLK[xo_ext_clk_enb] -- 0b1 (external 12MHz oscillator), OPHYCLK[clk_sel] -- 0b10 (12MHz) 
#else
	writel(0x0, S3C_USBOTG_PHYPWR);     /* Power up OTG block, analog block, force_suspend disable */
	writel(otg_phy_clk, S3C_USBOTG_PHYCLK);
#endif

	local_irq_restore(flags);

	printk(KERN_NOTICE "OTG_PHY_INIT: RESET OTG\n");
	writel(0x1, S3C_USBOTG_RSTCON);
	mdelay(80);
	writel(0x0, S3C_USBOTG_RSTCON);
	mdelay(80);
}
EXPORT_SYMBOL_GPL(otg_phy_init);
void otg_phy_off(void) {
    int flags;

    printk("otg_phy_off\n");

    local_irq_save(flags);
	
#ifdef CONFIG_USBHOST_CLOCK_EPLL
    writel(readl(S3C_USBOTG_PHYPWR)|(0x1b), S3C_USBOTG_PHYPWR); //Powerdown OTG block (otg_disable), Analog block(analog_powerdown), force_suspend
    writel(readl(S3C_HCLK_GATE) & ~(S3C_CLKCON_HCLK_USB), S3C_HCLK_GATE);
#else
    writel(readl(S3C_USBOTG_PHYPWR)|(0x11), S3C_USBOTG_PHYPWR);
#endif
    writel(readl(S3C_OTHERS) & ~S3C_OTHERS_USB_SIG_MASK, S3C_OTHERS);

    local_irq_restore(flags);
}
EXPORT_SYMBOL_GPL(otg_phy_off);

// Control D+ on OTG
void s3c_otg_soft_connect(int connect)
{
    volatile u32 uTemp;

    printk("[%s] connect = %d\n", __FUNCTION__, connect);

    if (connect)
    {
        uTemp = readl(S3C_UDC_OTG_DCTL);
        uTemp = uTemp & ~SOFT_DISCONNECT;
        writel(uTemp, S3C_UDC_OTG_DCTL);
    }
    else
    {
        uTemp = readl(S3C_UDC_OTG_DCTL);
        uTemp |= SOFT_DISCONNECT;
        writel(uTemp, S3C_UDC_OTG_DCTL);
    }

	uTemp = readl(S3C_UDC_OTG_DCTL);
	printk(KERN_NOTICE "s3c_otg_soft_connect: DCTL(0x%x)\n", uTemp);
}
EXPORT_SYMBOL(s3c_otg_soft_connect);

void s3c_otg_show_soft_connect(void)
{
	volatile u32 uTemp;

	uTemp = readl(S3C_UDC_OTG_DCTL);
	printk(KERN_NOTICE "s3c_otg_SHOW_soft_connect: DCTL(0x%x)\n", uTemp);
}
EXPORT_SYMBOL(s3c_otg_show_soft_connect);

void usb_host_clk_en(int usb_host_clksrc, u32 otg_phy_clk) {
    switch (usb_host_clksrc) {
    case 0: /* epll clk */
        writel((readl(S3C_CLK_SRC)& ~S3C_CLKSRC_UHOST_MASK)
            |S3C_CLKSRC_EPLL_CLKSEL|S3C_CLKSRC_UHOST_EPLL,
            S3C_CLK_SRC);
        writel(readl(S3C_CLK_DIV1)& ~S3C_CLKDIVN_UHOST_MASK, S3C_CLK_DIV1);  //CLK_DIV1[UHOST_RATIO] --0

        /* USB host colock divider ratio is 2 */
        //writel((readl(S3C_CLK_DIV1)& ~S3C_CLKDIVN_UHOST_MASK)
        //  |S3C_CLKDIV1_USBDIV2, S3C_CLK_DIV1);
        break;
    case 1: /* oscillator 48M clk */
        writel(readl(S3C_CLK_SRC)& ~(0x3 << 5), S3C_CLK_SRC);
        writel(readl(S3C_HCLK_GATE) | (S3C_CLKCON_HCLK_USB), S3C_HCLK_GATE);
        otg_phy_init(otg_phy_clk);

        /* USB host colock divider ratio is 1 */
        writel(readl(S3C_CLK_DIV1)& ~S3C_CLKDIVN_UHOST_MASK, S3C_CLK_DIV1);
        break;
    default:
        printk(KERN_INFO "Unknown USB Host Clock Source\n");
        BUG();
        break;
    }

    //Susan -- HCLK_UHOST is not bit 26, HCLK_SECUR is not bit 29
    //writel(readl(S3C_HCLK_GATE)|S3C_CLKCON_HCLK_UHOST|S3C_CLKCON_HCLK_SECUR,S3C_HCLK_GATE);
    
	writel(readl(S3C_HCLK_GATE) |(1<<29)|(1<<28), S3C_HCLK_GATE); //in 6410, HCLK_UHOST is bit 29, HCLK_SECUR is bit 28
    writel(readl(S3C_SCLK_GATE)|S3C_CLKCON_SCLK_UHOST, S3C_SCLK_GATE);
}

void usb_host_clk_disable(void) {

#ifdef CONFIG_USBHOST_CLOCK_EPLL
	//Nothing to do with USB OTG clock control
#else
    otg_phy_off();
    writel(readl(S3C_HCLK_GATE) & ~(S3C_CLKCON_HCLK_USB), S3C_HCLK_GATE);
#endif
    writel(readl(S3C_SCLK_GATE) & ~(S3C_CLKCON_SCLK_UHOST), S3C_SCLK_GATE);
    //Susan -- HCLK_GATE[26] is not UHOST HCLK_GATE bit, HCLK_GATE[29] is for UHOST
    //writel(readl(S3C_HCLK_GATE) & ~(S3C_CLKCON_HCLK_UHOST), S3C_HCLK_GATE);
    writel(readl(S3C_HCLK_GATE) & ~(1<<29), S3C_HCLK_GATE);

#ifdef CONFIG_USBHOST_CLOCK_EPLL
	//Nothing to do with USB OTG clock control
#else
    writel(readl(S3C_USBOTG_PHYPWR) | (0x19), S3C_USBOTG_PHYPWR);
#endif
}

void s3c_config_sleep_gpio(void)
{
}
EXPORT_SYMBOL(s3c_config_sleep_gpio);

void s3c_config_wakeup_gpio(void)
{
}
EXPORT_SYMBOL(s3c_config_wakeup_gpio);

void s3c_config_wakeup_source(void)
{
    u32 interrupt_mask = 0x0fffffff;
    u32 cover_open = get_s3c_reg_val(S3C64XX_GPNDAT) & 0x80;

    /*In Bravo, GPN10 is assigned for Headphone detection, it is not wakeup source.
    Below code is integrated from reference board's code base, so comment it out. */
    /* TODOTODO:  External wakeup via EXT INT Group0 need to be implemented here by using other GPIO pin */

#ifndef CONFIG_MACH_BRAVO
    /* EINT10 */
    s3c_gpio_cfgpin(S3C64XX_GPN(10), S3C64XX_GPN10_EINT10);
    s3c_gpio_setpull(S3C64XX_GPN(10), S3C_GPIO_PULL_UP);

    udelay(50);

    __raw_writel((__raw_readl(S3C64XX_EINT0CON0) & ~(0x7 << 20)) |
             (S3C64XX_EXTINT_BOTHEDGE << 20), S3C64XX_EINT0CON0);

    __raw_writel(1UL << (IRQ_EINT(10) - IRQ_EINT(0)), S3C64XX_EINT0PEND);
    __raw_writel(__raw_readl(S3C64XX_EINT0MASK)&~(1UL << (IRQ_EINT(10) - IRQ_EINT(0))), S3C64XX_EINT0MASK);
#endif

    // Allow the HeadPhone to wake the system
    interrupt_mask &= ~(1<<10);

    // Allow the HALL sensor to wakeup the device only when it is closed
    interrupt_mask &= ~(1<<7);

    // Allow the Power Button to wake the system
    interrupt_mask &= ~(1<<1);

    // Allow the PIMIC to wake the system
    interrupt_mask &= ~(1<<0);

    __raw_writel(interrupt_mask, S3C_EINT_MASK);

    /* Alarm Wakeup Enable */
    __raw_writel((__raw_readl(S3C_PWR_CFG) & ~(0x1 << 10)), S3C_PWR_CFG);
}
EXPORT_SYMBOL(s3c_config_wakeup_source);
