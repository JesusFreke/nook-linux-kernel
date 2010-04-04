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

#include <asm/system.h>
#include <plat/s3c64xx-dvfs.h>

#if (FREQ_TAB == ARM_CLOCK_800MHz)
#define CPUFREQ_MAXLEVEL 3
#else
#define CPUFREQ_MAXLEVEL 2
#endif

#define CPU_FREQ_EARLY_FLAG   0x100

static unsigned int s3c64xx_cpufreq_level = CPUFREQ_MAXLEVEL;

#define CLIP_LEVEL(a, b) (a > b ? b : a)

static struct cpufreq_frequency_table freq_table_532MHz[]={
	{0, 532*KHZ_T},
	{1, 266*KHZ_T},
	{2, 133*KHZ_T},
	{3, 133*KHZ_T},
	{4, 66*KHZ_T},
	{5, CPUFREQ_TABLE_END},
};

static struct cpufreq_frequency_table freq_table_800MHz[]={
	{0, 800*KHZ_T},
	{1, 400*KHZ_T},
	{2, 266*KHZ_T},	
	{3, 133*KHZ_T},
	{4, 133*KHZ_T},
	{5, (66)*KHZ_T},
	{6, CPUFREQ_TABLE_END},
};

static unsigned char transition_state_800MHz[][2] = {
	{1, 0},
	{2, 0},
	{3, 1},
	{4, 2},
	{5, 3},
	{5, 4},
};

static unsigned char transition_state_532MHz[][2] = {
	{1, 0},
	{2, 0},
	{3, 1},
	{4, 2},
	{4, 3},
};

static unsigned char (*transition_state[2])[2] = {
	transition_state_532MHz,
	transition_state_800MHz,
} ;

static struct cpufreq_frequency_table *s3c6410_freq_table[] = {
	freq_table_532MHz,
	freq_table_800MHz,
};

unsigned int s3c64xx_target_frq(unsigned int pred_freq, 
				int flag)
{
	static int index = 0;
	unsigned int freq;
	struct cpufreq_frequency_table *freq_tab = s3c6410_freq_table[FREQ_TAB];
	
	if(flag == 0) {
		if(freq_tab[index].frequency != pred_freq) {
//			printk("s3c64xx_target_freq:index error!!!!!!\n");
			return INDX_ERROR;
		}
	   return index;
	}

	if(flag == CPU_FREQ_EARLY_FLAG)
	{
		index = CPUFREQ_MAXLEVEL;
		return (freq_tab[index].frequency);
	}
	
	if(freq_tab[0].frequency < pred_freq) {
	  index = 0;	
	  freq = freq_tab[index].frequency;
          return freq;
	}

	if((flag != 1)&&(flag != -1)) {
		printk("s3c64xx_target_frq: flag error!!!!!!!!!!!!!");
	}
	// printk("==================== pred_freq = %d index = %d flag = %d\n", pred_freq, index, flag);
	if(freq_tab[index].frequency == pred_freq) {	
		if(flag == 1)
			index = transition_state[FREQ_TAB][index][1];
		else
			index = transition_state[FREQ_TAB][index][0];
	}
	else if(flag == -1) {
		index = 1;
	}
	else	{
		index = 0; 
	}
	
	index = CLIP_LEVEL(index, s3c64xx_cpufreq_level);
	
	freq = freq_tab[index].frequency;

//	printk("\n #================ Current Frequency = %d    Previous Frequency = %d \n", freq, pred_freq);

	return freq;
}

int s3c64xx_target_freq_index(unsigned int freq)
{
	int index = 0;
	int index2;
	
	struct cpufreq_frequency_table *freq_tab = s3c6410_freq_table[FREQ_TAB];

	if(freq >= freq_tab[index].frequency) {
		return index; 
	}
	/*Index might have been calculated before calling this function. check and early return if it is already calculated*/
	index2 = s3c64xx_target_frq(freq, 0);  	
	
	if(index2 != INDX_ERROR) {		
		return index2;	
	} 
	// later it will be deleted
	else	{
		printk("Index yet to be calculated !!!!! %d %d\n", index2, freq);
	}

	while((freq < freq_tab[index].frequency) &&
			(freq_tab[index].frequency != CPUFREQ_TABLE_END))
	{
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
	
	index = CLIP_LEVEL(index, s3c64xx_cpufreq_level);
	
	return index; 
} 

int s3c6410_verify_speed(struct cpufreq_policy *policy)
{
#ifndef USE_FREQ_TABLE
	struct clk *mpu_clk;
#endif
	if (policy->cpu)
		return -EINVAL;
#ifdef USE_FREQ_TABLE
	return cpufreq_frequency_table_verify(policy, s3c6410_freq_table[FREQ_TAB]);
#else
	cpufreq_verify_within_limits(policy, policy->cpuinfo.min_freq,
				     policy->cpuinfo.max_freq);
	mpu_clk = clk_get(NULL, MPU_CLK);
	if (IS_ERR(mpu_clk))
		return PTR_ERR(mpu_clk);

	policy->min = clk_round_rate(mpu_clk, policy->min * KHZ_T) / KHZ_T;
	policy->max = clk_round_rate(mpu_clk, policy->max * KHZ_T) / KHZ_T;

	cpufreq_verify_within_limits(policy, policy->cpuinfo.min_freq,
				     policy->cpuinfo.max_freq);

	clk_put(mpu_clk);

	return 0;
#endif
}
extern unsigned long s3c_fclk_get_rate(void);
unsigned int s3c6410_getspeed(unsigned int cpu)
{
	struct clk * mpu_clk;
	unsigned long rate;

	if (cpu)
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
	int ret = 0;
	unsigned long arm_clk;
	unsigned int index;

	mpu_clk = clk_get(NULL, MPU_CLK);
	if (IS_ERR(mpu_clk))
		return PTR_ERR(mpu_clk);

	freqs.old = s3c6410_getspeed(0);
	
	index = s3c64xx_target_freq_index(target_freq);
	if(index == INDX_ERROR)
	{
	   printk("s3c6410_target: INDX_ERROR \n");
	   return -EINVAL;
	}

	arm_clk = s3c6410_freq_table[FREQ_TAB][index].frequency;

	freqs.new = arm_clk;
	freqs.cpu = 0;

	target_freq = arm_clk;
//	printk("================= old freq = %d, new freq = %d\n", freqs.old, freqs.new);
	cpufreq_notify_transition(&freqs, CPUFREQ_PRECHANGE);
#ifdef USE_DVS
	if(freqs.new < freqs.old){
		/* frequency scaling */
		ret = clk_set_rate(mpu_clk, target_freq * KHZ_T);
		if(ret != 0)
			printk("frequency scaling error\n");
		/* voltage scaling */
		set_power(freqs.new, index);
	}
	else{
		/* voltage scaling */
		set_power(freqs.new, index);
		/* frequency scaling */
		ret = clk_set_rate(mpu_clk, target_freq * KHZ_T);
		if(ret != 0)
			printk("frequency scaling error\n");
	}

#else
	ret = clk_set_rate(mpu_clk, target_freq * KHZ_T);
	if(ret != 0)
		printk("frequency scaling error\n");

#endif
	cpufreq_notify_transition(&freqs, CPUFREQ_POSTCHANGE);
	clk_put(mpu_clk);
	return ret;
}

int s3c6410_pm_target(unsigned int target_freq)
{
	struct clk * mpu_clk;
	struct cpufreq_freqs freqs;
	int ret = 0;
	unsigned long arm_clk;
	unsigned int index;

	mpu_clk = clk_get(NULL, MPU_CLK);
	if (IS_ERR(mpu_clk))
		return PTR_ERR(mpu_clk);

	freqs.old = s3c6410_getspeed(0);
	
	index = s3c64xx_target_freq_index(target_freq);
	if(index == INDX_ERROR) {
	   printk("s3c6410_target: INDX_ERROR \n");
	   return -EINVAL;
	}
	
	arm_clk = s3c6410_freq_table[FREQ_TAB][index].frequency;

	freqs.new = arm_clk;
	freqs.cpu = 0;
	target_freq = arm_clk;
//	printk("================= old freq = %d, new freq = %d\n", freqs.old, freqs.new);
#ifdef USE_DVS
	if(freqs.new < freqs.old) {
		/* frequency scaling */
		ret = clk_set_rate(mpu_clk, target_freq * KHZ_T);
		if(ret != 0)
			printk("frequency scaling error\n");
		/* voltage scaling */
		set_power(freqs.new, index);
	}
	else {
		/* voltage scaling */
		set_power(freqs.new, index);
		/* frequency scaling */
		ret = clk_set_rate(mpu_clk, target_freq * KHZ_T);
		if(ret != 0)
			printk("frequency scaling error\n");
	}
#else
	ret = clk_set_rate(mpu_clk, target_freq * KHZ_T);
	if(ret != 0)
		printk("frequency scaling error\n");
#endif
	clk_put(mpu_clk);
	return ret;
//	s3c6410_target(NULL, target_freq, 1);
}

void s3c64xx_cpufreq_powersave(android_early_suspend_t *h)
{
	s3c64xx_cpufreq_level = CPUFREQ_MAXLEVEL + 2;

	return;
}

void s3c64xx_cpufreq_performance(android_early_suspend_t *h)
{
	unsigned int tmp;
	s3c64xx_cpufreq_level = CPUFREQ_MAXLEVEL;
	tmp = s3c64xx_target_frq(s3c6410_freq_table[FREQ_TAB][CPUFREQ_MAXLEVEL + 1].frequency, CPU_FREQ_EARLY_FLAG);
	s3c6410_target(NULL, tmp, 1);

	return;
}

static struct android_early_suspend s3c64xx_freq_suspend = {
	.suspend = s3c64xx_cpufreq_powersave,
	.resume = s3c64xx_cpufreq_performance,
};

static int __init s3c6410_cpu_init(struct cpufreq_policy *policy)
{
	struct clk * mpu_clk;
//#ifdef CONFIG_ANDROID_POWER
//	android_early_suspend_t	cpufreq_early_suspend;
//#endif
	
#ifdef USE_DVS
	ltc3714_init();
#endif
	mpu_clk = clk_get(NULL, MPU_CLK);
	if (IS_ERR(mpu_clk))
		return PTR_ERR(mpu_clk);

	if (policy->cpu != 0)
		return -EINVAL;
	policy->cur = policy->min = policy->max = s3c6410_getspeed(0);
#ifdef USE_FREQ_TABLE
	cpufreq_frequency_table_get_attr(s3c6410_freq_table[FREQ_TAB], policy->cpu);
#else
	policy->cpuinfo.min_freq = clk_round_rate(mpu_clk, 0) / KHZ_T;
	policy->cpuinfo.max_freq = clk_round_rate(mpu_clk, VERY_HI_RATE) / KHZ_T;
#endif
	policy->cpuinfo.transition_latency = 40000;	//1us

	clk_put(mpu_clk);
#ifdef CONFIG_ANDROID_POWER
//	cpufreq_early_suspend.suspend = s3c64xx_cpufreq_powersave;
//	cpufreq_early_suspend.resume = s3c64xx_cpufreq_performance;
//	cpufreq_early_suspend.level = ANDROID_EARLY_SUSPEND_LEVEL_DISABLE_FB;
	android_register_early_suspend(&s3c64xx_freq_suspend);
#endif
#ifdef USE_FREQ_TABLE
	return cpufreq_frequency_table_cpuinfo(policy, s3c6410_freq_table[FREQ_TAB]);
#else
	return 0;
#endif
}

static struct cpufreq_driver s3c6410_driver = {
	.flags		= CPUFREQ_STICKY,
	.verify		= s3c6410_verify_speed,
	.target		= s3c6410_target,
	.get		= s3c6410_getspeed,
	.init		= s3c6410_cpu_init,
	.name		= "s3c6410",
};

static int __init s3c6410_cpufreq_init(void)
{
	return cpufreq_register_driver(&s3c6410_driver);
}

arch_initcall(s3c6410_cpufreq_init);
