/* /arch/arm/plat-s3c64xx/include/plat/s3c64xx-dvfs.h
 *
 * Copyright (c) 2009 Samsung Electronics
  *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
*
 * Modifications 2009 : Intrinsyc Software, Inc on behalf of Barnes and Noble
 * Portions of this code copyright (c) 2009 Barnes and Noble, Inc
*/

#ifndef __PLAT_S3C64XX_DVFS_H
#define __PLAT_S3C64XX_DVFS_H __FILE__

#define MAXIMUM_FREQ 800000
#define USE_FREQ_TABLE
//#undef USE_DVS
#define USE_DVS
//#define USE_DVFS_AL1_LEVEL
#undef USE_DVFS_AL1_LEVEL
#define VERY_HI_RATE  532*1000*1000
#define APLL_GEN_CLK  532*1000
#define KHZ_T		1000

#define MPU_CLK		"clk_cpu"
#define INDX_ERROR  65535

#if defined(CONFIG_MACH_SMDK6410)
#define VCC_ARM		0
#define VCC_INT		1
extern void ltc3714_init(unsigned int, unsigned int);
#endif
extern unsigned int s3c64xx_cpufreq_index;
extern unsigned int S3C64XX_FREQ_TAB;
extern unsigned int S3C64XX_MAXFREQLEVEL;
extern int set_voltage(unsigned int);
extern unsigned int s3c64xx_target_frq(unsigned int pred_freq, int flag);
extern int s3c6410_pm_target(unsigned int target_freq);
extern int is_userspace_gov(void);
extern void set_dvfs_level(int flag);
extern void set_dvfs_perf_level(void);
#ifdef CONFIG_MACH_BRAVO
extern int lock_voltage(unsigned int);
extern int unlock_voltage(void);
#endif /* CONFIG_MACH_BRAVO */
#endif /* __PLAT_S3C64XX_DVFS_H */
