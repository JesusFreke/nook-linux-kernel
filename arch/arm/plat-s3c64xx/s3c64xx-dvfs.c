/*
 *  linux/arch/arm/plat-s3c64xx/s3c64xx-cpufreq.c
 *
 *  CPU frequency scaling for S3C64XX
 *
 *  Copyright (C) 2008 Samsung Electronics
 *
 *  Based on cpu-sa1110.c, Copyright (C) 2001 Russell King
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * Modifications 2009 : Intrinsyc Software, Inc on behalf of Barnes and Noble
 * Portions of this code copyright (c) 2009 Barnes and Noble, Inc
 */
#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/sched.h>
#include <linux/cpufreq.h>
#include <linux/delay.h>
#include <linux/init.h>
#include <linux/err.h>
#include <linux/clk.h>

#ifdef CONFIG_ANDROID_POWER
#include <linux/android_power.h>
#endif

#ifdef CONFIG_MACH_BRAVO
#include <linux/regulator/consumer.h>
#endif /*CONFIG_MACH_BRAVO*/

#include <asm/system.h>
#include <plat/s3c64xx-dvfs.h>

#define CPU_FREQ_EARLY_FLAG   0x100

#if defined(CONFIG_MACH_SMDK6410)
//extern int set_pmic(unsigned int pwr, unsigned int voltage);
#endif
#if CONFIG_MACH_BRAVO
static int set_pmic(unsigned int pwer, unsigned int voltage);
static int voltage_locked = 0;
#endif 

unsigned int S3C64XX_MAXFREQLEVEL = 3;
static unsigned int s3c64xx_cpufreq_level = 3;
static char cpufreq_governor_name[CPUFREQ_NAME_LEN] = "userspace";
static char userspace_governor[CPUFREQ_NAME_LEN] = "userspace";
unsigned int s3c64xx_cpufreq_index = 0;
static DEFINE_MUTEX(dvfs_lock);
#define CLIP_LEVEL(a, b) (a > b ? b : a)

static struct cpufreq_frequency_table freq_table_532MHz[] = {
	{0, 532*KHZ_T},
	{1, 266*KHZ_T},
	{2, 133*KHZ_T},
#ifdef USE_DVFS_AL1_LEVEL
	{3, 133*KHZ_T},
	{4, 66*KHZ_T},
	{5, CPUFREQ_TABLE_END},
#else
//	{3, 66*KHZ_T},
	{3, CPUFREQ_TABLE_END},		
#endif

};

static struct cpufreq_frequency_table freq_table_800MHz[] = {
	{0, 800*KHZ_T},
	{1, 400*KHZ_T},
	{2, 266*KHZ_T},	
	{3, 133*KHZ_T},
#ifdef USE_DVFS_AL1_LEVEL
	{4, 133*KHZ_T},
	{5, (66)*KHZ_T},
	{6, CPUFREQ_TABLE_END},
#else
	{4, (66)*KHZ_T},
	{5, CPUFREQ_TABLE_END},
#endif
};

static unsigned char transition_state_800MHz[][2] = {
	{1, 0},
	{2, 0},
	{3, 1},
	{4, 2},
#ifdef USE_DVFS_AL1_LEVEL
	{5, 3},
	{5, 4},
#else
	{4, 3},
#endif
};

static unsigned char transition_state_532MHz[][2] = {
	{1, 0},
	{2, 0},
	{3, 1},
#ifdef USE_DVFS_AL1_LEVEL
	{4, 2},
	{4, 3},
#else
//	{3, 2},
#endif
};

/* frequency voltage matching table */
static const unsigned int frequency_match_532MHz[][4] = {
/* frequency, Mathced VDD ARM voltage , Matched VDD INT*/
	{532000, 1100, 1200, 0},
	{266000, 1050, 1200, 1},
	{133000, 1000, 1200, 2},
#ifdef USE_DVFS_AL1_LEVEL
	{133000, 1000, 1050, 3},
	{66000, 1000, 1050, 4},
#else
//	{66000, 1000, 1050, 3},
#endif
};

/* frequency voltage matching table */
static const unsigned int frequency_match_800MHz[][4] = {
/* frequency, Mathced VDD ARM voltage , Matched VDD INT*/
	{800000, 1300, 1200, 0},
	{400000, 1100, 1200, 1},
	{266000, 1050, 1200, 2},
	{133000, 1000, 1200, 3},
#ifdef USE_DVFS_AL1_LEVEL
	{133000, 1000, 1050, 4},
	{66000, 1000, 1050, 5},
#else
	{66000, 1000, 1050, 4},
#endif
};

static const unsigned int (*frequency_match[2])[4] = {
	frequency_match_532MHz,
	frequency_match_800MHz,
};

static unsigned char (*transition_state[2])[2] = {
	transition_state_532MHz,
	transition_state_800MHz,
};

static struct cpufreq_frequency_table *s3c6410_freq_table[] = {
	freq_table_532MHz,
	freq_table_800MHz,
};

#ifdef CONFIG_MACH_BRAVO
static int set_pmic(unsigned int pwer, unsigned int voltage)
{
	int ret = 0;
	struct regulator* reg;

	switch(pwer) {
	case VCC_ARM:
		reg = regulator_get(NULL, "DCDC6");
		break;

	case VCC_INT:
		reg = regulator_get(NULL, "DCDC1");
		break;

	default:
		printk(KERN_ERR "Unsupported power source: %u\n", pwer);
		ret = -ENODEV;
		goto out;
	}

	if (IS_ERR(reg)) {
		printk(KERN_ERR "Failed to get regulator for %u\n", pwer);
		goto out;
	}

	//printk("Regulator voltage for %s from %d to %u uV\n", pwer == VCC_INT ? "VCC_INT" : "VCC_ARM", regulator_get_voltage(reg), voltage*1000);
	ret = regulator_set_voltage(reg, voltage * 1000, voltage * 1000);

	if (ret) {
		printk(KERN_ERR "Setting voltage for regulator %u failed: %d\n", pwer, ret);
	}

	//udelay(100);
	mdelay(1);

out:
	return ret;
}
#endif /* CONFIG_MACH_BRAVO */

static int dvfs_perf_lock = 0;
int dvfs_change_quick = 0;
void static sdvfs_lock(unsigned int *lock)
{
	while(*lock) {
		msleep(1);
	}
	*lock = 1;
}

void static sdvfs_unlock(unsigned int *lock)
{
	*lock = 0;
}

void set_dvfs_perf_level(void)
{
	sdvfs_lock(&dvfs_perf_lock);

	/* if user input (keypad, touchscreen) occur, raise up 800MHz */
	/* maximum frequency :800MHz(0), 400MHz(1) */
	s3c64xx_cpufreq_index = 0;
	dvfs_change_quick = 1;

	sdvfs_unlock(&dvfs_perf_lock);
}
EXPORT_SYMBOL(set_dvfs_perf_level);

void set_dvfs_level(int flag)
{
	mutex_lock(&dvfs_lock);
	if(flag == 0)
#ifdef USE_DVFS_AL1_LEVEL
		s3c64xx_cpufreq_level = S3C64XX_MAXFREQLEVEL - 2;
#else
		s3c64xx_cpufreq_level = S3C64XX_MAXFREQLEVEL - 1;
#endif
	else
		s3c64xx_cpufreq_level = S3C64XX_MAXFREQLEVEL;
	mutex_unlock(&dvfs_lock);

}
EXPORT_SYMBOL(set_dvfs_level);

#ifdef USE_DVS
static unsigned int s_arm_voltage, s_int_voltage;
int set_voltage(unsigned int freq_index)
{
	static int index = 0;
	unsigned int arm_voltage, int_voltage;
	
	if(index == freq_index || voltage_locked)
		return 0;
		
	index = freq_index;
	
	arm_voltage = frequency_match[S3C64XX_FREQ_TAB][index][1];
	int_voltage = frequency_match[S3C64XX_FREQ_TAB][index][2];
	
	if(arm_voltage != s_arm_voltage) {
		set_pmic(VCC_ARM, arm_voltage);
		s_arm_voltage = arm_voltage;
	}
	if(int_voltage != s_int_voltage) {
		set_pmic(VCC_INT, int_voltage);
		s_int_voltage = int_voltage;
	}

	return 0;
}

#ifdef CONFIG_MACH_BRAVO
int lock_voltage(unsigned int freq_index)
{
	int ret;
	mutex_lock(&dvfs_lock);

	printk(KERN_NOTICE "lock_voltage: set_voltage(%d)\n", freq_index);
	ret = set_voltage(freq_index);

	if (!ret) {
		voltage_locked = 1;
	}

	mutex_unlock(&dvfs_lock);
	return ret;
}

int unlock_voltage(void) 
{
	mutex_lock(&dvfs_lock);
	voltage_locked = 0;
	mutex_unlock(&dvfs_lock);
	return 0;
}

#endif /* CONFIG_MACH_BRAVO */
#endif	/* USE_DVS */

unsigned int s3c64xx_target_frq(unsigned int pred_freq, 
				int flag)
{
	int index; 
	unsigned int freq;
	struct cpufreq_frequency_table *freq_tab = s3c6410_freq_table[S3C64XX_FREQ_TAB];

	if(freq_tab[0].frequency < pred_freq) {
	   index = 0;	
	   goto s3c64xx_target_frq_end;
	}

	if((flag != 1)&&(flag != -1)) {
		printk("s3c64xx_target_frq: flag error!!!!!!!!!!!!!");
	}

	sdvfs_lock(&dvfs_perf_lock);
	index = s3c64xx_cpufreq_index;

	if(freq_tab[index].frequency == pred_freq) {	
		if(flag == 1)
			index = transition_state[S3C64XX_FREQ_TAB][index][1];
		else
			index = transition_state[S3C64XX_FREQ_TAB][index][0];
	}
	else if(flag == -1) {
		index = 1;
	}
	else {
		index = 0; 
	}
s3c64xx_target_frq_end:
	mutex_lock(&dvfs_lock);
	index = CLIP_LEVEL(index, s3c64xx_cpufreq_level);
	mutex_unlock(&dvfs_lock);
	s3c64xx_cpufreq_index = index;

	freq = freq_tab[index].frequency;
	sdvfs_unlock(&dvfs_perf_lock);
	return freq;
}

int s3c64xx_target_freq_index(unsigned int freq)
{
	int index = 0;
	
	struct cpufreq_frequency_table *freq_tab = s3c6410_freq_table[S3C64XX_FREQ_TAB];

	if(freq >= freq_tab[index].frequency) {
		goto s3c64xx_target_freq_index_end;
	}

	/*Index might have been calculated before calling this function.
	check and early return if it is already calculated*/
	if(freq_tab[s3c64xx_cpufreq_index].frequency == freq) {		
		return s3c64xx_cpufreq_index;
	}

	while((freq < freq_tab[index].frequency) &&
			(freq_tab[index].frequency != CPUFREQ_TABLE_END)) {
		index++;
	}

	if(index > 0) {
		if(freq != freq_tab[index].frequency) {
			index--;
		}
	}

	if(freq_tab[index].frequency == CPUFREQ_TABLE_END) {
		index--;
	}

s3c64xx_target_freq_index_end:
	mutex_lock(&dvfs_lock);
	index = CLIP_LEVEL(index, s3c64xx_cpufreq_level);
	mutex_unlock(&dvfs_lock);
	s3c64xx_cpufreq_index = index;

	return index; 
} 

int is_userspace_gov(void)
{
	int ret = 0;

	if(!strnicmp(cpufreq_governor_name, userspace_governor, CPUFREQ_NAME_LEN)) {
		ret = 1;
	}

	return ret;
}

int s3c6410_verify_speed(struct cpufreq_policy *policy)
{
#ifndef USE_FREQ_TABLE
	struct clk *mpu_clk;
#endif	/* USE_FREQ_TABLE */
	if(policy->cpu)
		return -EINVAL;
#ifdef USE_FREQ_TABLE
	return cpufreq_frequency_table_verify(policy, s3c6410_freq_table[S3C64XX_FREQ_TAB]);
#else
	cpufreq_verify_within_limits(policy, policy->cpuinfo.min_freq,
				     policy->cpuinfo.max_freq);
	mpu_clk = clk_get(NULL, MPU_CLK);

	if(IS_ERR(mpu_clk))
		return PTR_ERR(mpu_clk);

	policy->min = clk_round_rate(mpu_clk, policy->min * KHZ_T) / KHZ_T;
	policy->max = clk_round_rate(mpu_clk, policy->max * KHZ_T) / KHZ_T;

	cpufreq_verify_within_limits(policy, policy->cpuinfo.min_freq,
				     policy->cpuinfo.max_freq);

	clk_put(mpu_clk);

	return 0;
#endif	/* USE_FREQ_TABLE */
}
extern unsigned long s3c_fclk_get_rate(void);
unsigned int s3c6410_getspeed(unsigned int cpu)
{
	struct clk * mpu_clk;
	unsigned long rate;

	if(cpu)
		return 0;

	mpu_clk = clk_get(NULL, MPU_CLK);
	if (IS_ERR(mpu_clk))
		return 0;

	rate = s3c_fclk_get_rate() / KHZ_T;
	clk_put(mpu_clk);

	return rate;
}

static int s3c6410_target(struct cpufreq_policy *policy,
		       unsigned int target_freq,
		       unsigned int relation)
{
	struct clk * mpu_clk;
	struct cpufreq_freqs freqs;
	static int prevIndex = 0;
	int ret = 0;
	unsigned long arm_clk;
	unsigned int index;

	mpu_clk = clk_get(NULL, MPU_CLK);
	if(IS_ERR(mpu_clk))
		return PTR_ERR(mpu_clk);

	if(policy != NULL) {
		if(policy->governor) {
			if (strnicmp(cpufreq_governor_name, policy->governor->name, CPUFREQ_NAME_LEN)) {
				strcpy(cpufreq_governor_name, policy->governor->name);
			}
		}
	}

	freqs.old = s3c6410_getspeed(0);

	if(freqs.old == s3c6410_freq_table[S3C64XX_FREQ_TAB][0].frequency) {
		prevIndex = 0;
	}
	
	index = s3c64xx_target_freq_index(target_freq);
	if(index == INDX_ERROR) {
		printk("s3c6410_target: INDX_ERROR \n");
		return -EINVAL;
	}
	
	if(prevIndex == index)
		return ret;

	arm_clk = s3c6410_freq_table[S3C64XX_FREQ_TAB][index].frequency;
	freqs.new = arm_clk;
	freqs.cpu = 0;
	freqs.new_hclk = 133000;
  
	if(index > S3C64XX_MAXFREQLEVEL) {
		freqs.new_hclk = 66000;         
	} 

	target_freq = arm_clk;

	cpufreq_notify_transition(&freqs, CPUFREQ_PRECHANGE);

#ifdef USE_DVS
	if(prevIndex < index) {
		/* frequency scaling */
		ret = clk_set_rate(mpu_clk, target_freq * KHZ_T);
		if(ret != 0) {
			printk("frequency scaling error\n");
			ret = -EINVAL;
			goto s3c6410_target_end;
		}
		/* voltage scaling */
		mutex_lock(&dvfs_lock);
		set_voltage(index);
		mutex_unlock(&dvfs_lock);
	}
	else {
		/* voltage scaling */
		mutex_lock(&dvfs_lock);
		set_voltage(index);
		mutex_unlock(&dvfs_lock);
		/* frequency scaling */
		ret = clk_set_rate(mpu_clk, target_freq * KHZ_T);
		if(ret != 0) {
			printk("frequency scaling error\n");
			ret = -EINVAL;
			goto s3c6410_target_end;
		}
	}
#else
	ret = clk_set_rate(mpu_clk, target_freq * KHZ_T);
	if(ret != 0) {
		printk("frequency scaling error\n");
		ret = -EINVAL;
		goto s3c6410_target_end;
	}
#endif	/* USE_DVS */
	cpufreq_notify_transition(&freqs, CPUFREQ_POSTCHANGE);
	prevIndex = index;
	clk_put(mpu_clk);
s3c6410_target_end:
	return ret;
}

int s3c6410_pm_target(unsigned int target_freq)
{
	struct clk * mpu_clk;
	int ret = 0;
	unsigned long arm_clk;
	unsigned int index;

	mpu_clk = clk_get(NULL, MPU_CLK);
	if(IS_ERR(mpu_clk))
		return PTR_ERR(mpu_clk);

	index = s3c64xx_target_freq_index(target_freq);
	if(index == INDX_ERROR) {
	   printk("s3c6410_target: INDX_ERROR \n");
	   return -EINVAL;
	}
	
	arm_clk = s3c6410_freq_table[S3C64XX_FREQ_TAB][index].frequency;
	target_freq = arm_clk;

#ifdef USE_DVS
	mutex_lock(&dvfs_lock);
	set_voltage(index);
	mutex_unlock(&dvfs_lock);
#endif	/* USE_DVS */
	/* frequency scaling */
	ret = clk_set_rate(mpu_clk, target_freq * KHZ_T);
	if(ret != 0) {
		printk("frequency scaling error\n");
		return -EINVAL;
	}
	
	clk_put(mpu_clk);
	return ret;
}

#ifdef CONFIG_ANDROID_POWER
void s3c64xx_cpufreq_powersave(android_early_suspend_t *h)
{
	unsigned long irqflags;
	spin_lock_irqsave(&g_cpufreq_lock, irqflags);
	s3c64xx_cpufreq_level = S3C64XX_MAXFREQLEVEL + 2;
	spin_unlock_irqrestore(&g_cpufreq_lock, irqflags);
	return;
}

void s3c64xx_cpufreq_performance(android_early_suspend_t *h)
{
	unsigned long irqflags;
	if(!is_userspace_gov()) {
		spin_lock_irqsave(&g_cpufreq_lock, irqflags);
		s3c64xx_cpufreq_level = S3C64XX_MAXFREQLEVEL;
		s3c64xx_cpufreq_index = CLIP_LEVEL(s3c64xx_cpufreq_index, S3C64XX_MAXFREQLEVEL);
		spin_unlock_irqrestore(&g_cpufreq_lock, irqflags);
		s3c6410_target(NULL, s3c6410_freq_table[S3C64XX_FREQ_TAB][s3c64xx_cpufreq_index].frequency, 1);
	}
	else {
		spin_lock_irqsave(&g_cpufreq_lock, irqflags);
		s3c64xx_cpufreq_level = S3C64XX_MAXFREQLEVEL;
		spin_unlock_irqrestore(&g_cpufreq_lock, irqflags);
#ifdef USE_DVS
		mutex_lock(&dvfs_lock);
		set_voltage(s3c64xx_cpufreq_index);
		mutex_unlock(&dvfs_lock);
#endif
	}

	return;
}

static struct android_early_suspend s3c64xx_freq_suspend = {
	.suspend = s3c64xx_cpufreq_powersave,
	.resume = s3c64xx_cpufreq_performance,
	.level = ANDROID_EARLY_SUSPEND_LEVEL_DISABLE_FB + 1,
};
#endif

unsigned int get_min_cpufreq(void)
{
	return (s3c6410_freq_table[S3C64XX_FREQ_TAB][S3C64XX_MAXFREQLEVEL].frequency);
}

static int __init s3c6410_cpu_init(struct cpufreq_policy *policy)
{
	struct clk * mpu_clk;

	mpu_clk = clk_get(NULL, MPU_CLK);
	if(IS_ERR(mpu_clk))
		return PTR_ERR(mpu_clk);

	if(policy->cpu != 0)
		return -EINVAL;
	policy->cur = policy->min = policy->max = s3c6410_getspeed(0);

	if(policy->max == MAXIMUM_FREQ) {
		S3C64XX_FREQ_TAB = 1;
#ifdef USE_DVFS_AL1_LEVEL
		S3C64XX_MAXFREQLEVEL = 5;
#else
		S3C64XX_MAXFREQLEVEL = 4;
#endif
	}
	else {
		S3C64XX_FREQ_TAB = 0;
#ifdef USE_DVFS_AL1_LEVEL
		S3C64XX_MAXFREQLEVEL = 4;
#else
		S3C64XX_MAXFREQLEVEL = 3;
#endif
	}
	s3c64xx_cpufreq_level = S3C64XX_MAXFREQLEVEL;

#ifdef USE_FREQ_TABLE
	cpufreq_frequency_table_get_attr(s3c6410_freq_table[S3C64XX_FREQ_TAB], policy->cpu);
#else
	policy->cpuinfo.min_freq = clk_round_rate(mpu_clk, 0) / KHZ_T;
	policy->cpuinfo.max_freq = clk_round_rate(mpu_clk, VERY_HI_RATE) / KHZ_T;
#endif	/* USE_FREQ_TABLE */
	policy->cpuinfo.transition_latency = 30000;

	clk_put(mpu_clk);

#ifdef USE_DVS
	s_arm_voltage = frequency_match[S3C64XX_FREQ_TAB][0][1];
	s_int_voltage = frequency_match[S3C64XX_FREQ_TAB][0][2];
#if defined(CONFIG_MACH_SMDK6410)
#ifndef CONFIG_MACH_BRAVO
	ltc3714_init(s_arm_voltage, s_int_voltage);
#endif
#endif
#endif

#ifdef CONFIG_ANDROID_POWER
	android_register_early_suspend(&s3c64xx_freq_suspend);
#endif
#ifdef USE_FREQ_TABLE
	return cpufreq_frequency_table_cpuinfo(policy, s3c6410_freq_table[S3C64XX_FREQ_TAB]);
#else
	return 0;
#endif	/* USE_FREQ_TABLE */
}

static struct cpufreq_driver s3c6410_driver = {
	.flags		= CPUFREQ_STICKY,
	.verify		= s3c6410_verify_speed,
	.target		= s3c6410_target,
	.get			= s3c6410_getspeed,
	.init			= s3c6410_cpu_init,
	.name			= "s3c6410",
};

static int __init s3c6410_cpufreq_init(void)
{
	return cpufreq_register_driver(&s3c6410_driver);
}

device_initcall(s3c6410_cpufreq_init);
