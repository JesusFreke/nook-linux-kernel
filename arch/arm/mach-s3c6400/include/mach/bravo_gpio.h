/*
 *  linux/include/asm-arm/arch-pxa/zylonite.h
 *
 * Copyright(C) 2006 Marvell Internaltional Ltd
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef __ASM_MACH_BRAVO_H__
#define __ASM_MACH_BRAVO_H__
#include <mach/gpio.h>
#include <mach/map.h>
#include <plat/gpio-cfg.h>
#include <plat/regs-gpio.h>
#include <linux/firmware.h>

// Bravo GPIO setup structure
struct s3c_gpio_reg {
    u32 reg_addr;
    u32 normal_function;
    u32 init_val;
    u32 normal_pull;
    u32 sleep_function;
    u32 sleep_pull;
};

// Default GPIO setup structure
struct s3c_gpio_reg_default {
    void __iomem *con;
    void __iomem *con1;
    void __iomem *pud;
    void __iomem *conslp;
    void __iomem *pudslp;
    u32 val_con;
    u32 val_con1;
    u32 val_dat;
    u32 val_pud;
    u32 val_conslp;
    u32 val_pudslp;
};

#define S3C64XX_INIT_0(reg, _con, _dat, _pud, _conslp, _pudslp) { \
      con: S3C64XX_##reg##CON, con1: 0, \
      pud: S3C64XX_##reg##PUD, \
      conslp: S3C64XX_##reg##CONSLP, \
      pudslp: S3C64XX_##reg##PUDSLP, \
      val_con: _con, \
      val_dat: _dat, \
      val_pud: _pud, \
      val_conslp: _conslp, \
      val_pudslp: _pudslp \
      }

#define S3C64XX_INIT_1(reg, _con, _con1, _dat, _pud, _conslp, _pudslp) { \
      con: S3C64XX_##reg##CON0, con1: S3C64XX_##reg##CON1, \
      pud: S3C64XX_##reg##PUD, \
      conslp: S3C64XX_##reg##CONSLP, \
      pudslp: S3C64XX_##reg##PUDSLP, \
      val_con: _con, \
      val_con1: _con1, \
      val_dat: _dat, \
      val_pud: _pud, \
      val_conslp: _conslp, \
      val_pudslp: _pudslp \
      }

#define S3C64XX_INIT_2(reg, _con, _dat, _pud) { \
      con: S3C64XX_##reg##CON, con1: 0, \
      pud: S3C64XX_##reg##PUD, \
      conslp: 0, \
      pudslp: 0, \
      val_con: _con, \
      val_dat: _dat, \
      val_pud: _pud, \
      }

#define S3C64XX_INIT_3(reg, _con, _con1, _dat, _pud) { \
      con: S3C64XX_##reg##CON, con1: S3C64XX_##reg##CON1, \
      pud: S3C64XX_##reg##PUD, \
      conslp: 0, \
      pudslp: 0, \
      val_con: _con, \
      val_con1: _con1, \
      val_dat: _dat, \
      val_pud: _pud, \
      }

/* Define gpio pins for Bravo platform (HW1.0) */
#define Snd_CARD_EN     S3C64XX_GPE(0)
#define WF_PWR_EN       S3C64XX_GPN(5)  //TODO -- GPN is alive-part GPIO, is it able to keep pin-state in sleep mode?
#define WF_RSTn         S3C64XX_GPN(4)  //TODO -- GPN is alive-part GPIO
#define WF_PWR_DNn      S3C64XX_GPN(3)  //TODO -- GPN is alive-part GPIO
#define WF_WAKEUP_S3C6410       S3C64XX_GPL(10)
#define S3C6410_WAKEUP_WF       S3C64XX_GPL(9)

#define Snd_CARD_EN_AF      S3C_GPIO_SFN(1)
#define WF_PWR_EN_AF        S3C_GPIO_SFN(1)
#define WF_RSTn_AF          S3C_GPIO_SFN(1)
#define WF_PWR_DNn_AF       S3C_GPIO_SFN(1)
#define WF_WAKEUP_S3C6410_AF        S3C_GPIO_SFN(3)  //EXT INTERRUPT 18
#define S3C6410_WAKEUP_WF_AF        S3C_GPIO_SFN(1)  //OUTPUT

extern unsigned int get_s3c_reg_val(void __iomem *reg_addr);
extern unsigned int set_s3c_reg_val(void __iomem *reg_addr, unsigned int regval);
extern void s3c6410_enable_wifi_pins(void);
extern void s3c6410_reset_wifi(void);
#ifdef CONFIG_MACH_BRAVO_EVT1
extern void s3c6410_enable_lightsensor(void);
#endif  //EVT1
extern void s3c_set_irqflt_group0(int irq, int enable, int value);
#endif
