/*
* SoC audio for Bravo (S3C6410) with WM8350 (Follow smdk6410_wm8990.c)
 *
 * Copyright 2007, 2008 Wolfson Microelectronics PLC.
 * Author: Liam Girdwood
 *         lg@opensource.wolfsonmicro.com or linux@wolfsonmicro.com
 *
 *  Copyright (C) 2007, Ryu Euiyoul <ryu.real@gmail.com>
 *
 *  This program is free software; you can redistribute  it and/or modify it
 *  under  the terms of  the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the  License, or (at your
 *  option) any later version.
 *
 *  Revision history
 *    28th Feb 2008   Initial version.
 *
 */

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/timer.h>
#include <linux/interrupt.h>
#include <linux/workqueue.h>
#include <linux/platform_device.h>
#include <linux/i2c.h>
#include <linux/switch.h>
#include <linux/delay.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>

#include <asm/mach-types.h>
#include <plat/regs-s3c64xx-iis.h>
#include <plat/regs-s3c2412-iis.h>
#include <mach/map.h>
#include <mach/regs-irq.h>

#include <plat/regs-gpio.h>
#include <linux/gpio.h>
#include <plat/gpio-bank-e.h>
#include <plat/gpio-cfg.h>
#include <mach/hardware.h>
#include <mach/audio.h>
#include <asm/io.h>
#include <plat/regs-clock.h>
#include <mach/bravo_gpio.h>

#include <linux/mfd/wm8350/audio.h>
#include <linux/mfd/wm8350/core.h>
#include "../codecs/wm8350.h"
#include "s3c-pcm.h"
#include "s3c64xx-i2s.h"

/* define the scenarios */
#define BRAVO_AUDIO_OFF			0
#define BRAVO_STEREO_TO_SPEAKERS		1
#define BRAVO_STEREO_TO_HEADPHONES	2

#define HEADPHONE_SWITCH_NAME "h2w"

#ifdef CONFIG_SND_DEBUG
#define s3cdbg(x...) printk(x)
#else
#define s3cdbg(x...) //
#endif

void Mute();
void UnMute();

struct platform_playback
{
	struct delayed_work  playback_sw;
	struct switch_dev  sdev;
	struct mutex  ctrl_lock;
};
static struct platform_playback bravo_playback;

struct platform_pcm_apm bravo_speaker_apm;
EXPORT_SYMBOL_GPL(bravo_speaker_apm);
/*
 * TODO: - We need to work out PLL values for 256FS for every rate.
 */
static int bravo_hifi_hw_params(struct snd_pcm_substream *substream,
	struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *codec_dai = rtd->dai->codec_dai;
	struct snd_soc_dai *cpu_dai = rtd->dai->cpu_dai;
	unsigned int pll_out = 0;
	unsigned int audio0_ratio = 0;
	unsigned int epll_or_mpll = 1; //0 -- MOUTepll, 1 -- DOUTmpll, 2 -- FINepll
	int ret = 0;
	unsigned int iispsr, iismod;
	unsigned int prescaler = 0;

	u32*	regs;

	regs = ioremap(S3C64XX_PA_IIS0, 0x100);

	s3cdbg("Entered %s, rate = %d\n", __FUNCTION__, params_rate(params));

	/*PCLK & SCLK gating enable*/
	writel(readl(S3C_PCLK_GATE)|S3C_CLKCON_PCLK_IIS0, S3C_PCLK_GATE);
	//writel(readl(S3C_SCLK_GATE)|S3C_CLKCON_SCLK_AUDIO0, S3C_SCLK_GATE);

	iismod = readl(regs + S3C64XX_IIS0MOD);
	iismod &= ~S3C64XX_IIS0MOD_FS_MASK;
	iismod &= ~S3C64XX_IIS0MOD_BFS_MASK;

	/*Clear I2S prescaler value [13:8] and disable prescaler*/
	iispsr = readl(regs + S3C64XX_IIS0PSR);	
	iispsr &=~((0x3f<<8)|(1<<15)); 
	writel(iispsr, regs + S3C64XX_IIS0PSR);

	//Configure DOUTmpll according to different fs (sample frequency)
	switch (params_rate(params)) {
	case 8000:
		pll_out = 2048000;
		break;
	case 11025:
		pll_out = 2822400;
		break;
	case 16000:
		pll_out = 4096000;
		break;
	case 22050:
		pll_out = 5644800;
		break;
	case 32000:
		pll_out = 8192000;
		break;
	case 44100:
		pll_out = 11289600;
		break;
	case 48000:
		pll_out = 12288000;
		break;
	case 88200:
		pll_out = 22579200;
		break;
	case 96000:
		pll_out = 24576000;
		break;
	default:
		printk(KERN_ERR "bravo_hifi_hw_params: S3C6410 doesn't support %d SampleRate\n", params_rate(params));
		return -EINVAL;
		/* somtimes 32000 rate comes to 96000 
		   default values are same as 32000 */
		//iismod |= S3C64XX_IIS0MOD_384FS;
		//pll_out = 12288000;
		//break;
	}

	/* set MCLK division for sample rate */
	s3cdbg("bravo_hifi_hw_params:params_format(%d)\n", params_format(params));
	
	audio0_ratio = 0;
	epll_or_mpll = 1; //DOUTmpll(133MHz)
	
	switch (params_format(params)) {
	case SNDRV_PCM_FORMAT_S8:
	case SNDRV_PCM_FORMAT_S16_LE:
		iismod |= S3C64XX_IIS0MOD_256FS | S3C64XX_IIS0MOD_32FS;
		switch(params_rate(params))
		{
		case 8000:
			audio0_ratio = 4; //division factor (4+1)
			prescaler = 13; //IISPSR[PSVALA]=25
			break;
		case 11025:
			audio0_ratio = 0; //CLKAUDIO0in/(4+1)
			prescaler = 47;
			break;		
		case 16000:
			audio0_ratio = 0; //CLKAUDIO0in/(4+1)
			prescaler = 33;
		case 22050:
			audio0_ratio = 0; //CLKAUDIO0in/(0+1)
			prescaler = 24;
			break;
		case 32000:
			audio0_ratio = 0; //CLKAUDIO0in/(0+1)
			prescaler = 16;
			break;
		case 44100:
			audio0_ratio = 0; //CLKAUDIO0in/(0+1)
			prescaler = 12;
			break;
		case 48000:
			epll_or_mpll = 2; //FINepll
			audio0_ratio = 0; //CLKAUDIO0in/(0+1)
			prescaler = 1;
			break;
		case 64000:
			audio0_ratio = 0; //CLKAUDIO0in/(0+1)
			prescaler = 8;
			break;
		case 88200:
			audio0_ratio = 0; //CLKAUDIO0in/(0+1)
			prescaler = 6;
			break;
		case 96000:
#ifdef CONFIG_USBHOST_CLOCK_EPLL
			epll_or_mpll = 0; //MOUTepll(48MHz)
			audio0_ratio = 1; //CLKAUDIO0in/(1+1)
			prescaler = 1;
#else
			epll_or_mpll = 0; //MOUTepll(24MHz)
			audio0_ratio = 0; //CLKAUDIO0in/(0+1)
			prescaler = 1;
#endif
			break;
		default:
			printk(KERN_ERR "bravo_hifi_hw_params: S3C6410 doesn't support %d SampleRate\n", params_rate(params));
			return -EINVAL;
		}
		break;
	case SNDRV_PCM_FORMAT_S24_LE:
		iismod |= S3C64XX_IIS0MOD_512FS | S3C64XX_IIS0MOD_48FS;
		switch(params_rate(params))
		{
		case 8000:
			audio0_ratio = 0; //CLKAUDIO0in/(4+1)
			prescaler = 33;
			break;
		case 11025:
			audio0_ratio = 0; //CLKAUDIO0in/(0+1)
			prescaler = 24;
			break;
		case 16000:
			audio0_ratio = 0; //CLKAUDIO0in/(0+1)
			prescaler = 16;
			break;
		case 22050:
			audio0_ratio = 0; //CLKAUDIO0in/(0+1)
			prescaler = 12;
			break;
		case 32000:
			audio0_ratio = 0; //CLKAUDIO0in/(0+1)
			prescaler = 8;
			break;
		case 44100:
			audio0_ratio = 0; //CLKAUDIO0in/(0+1)
			prescaler = 6;
			break;
		case 48000:
#ifdef CONFIG_USBHOST_CLOCK_EPLL
			epll_or_mpll = 0; //MOUTepll(48MHz)
			audio0_ratio = 1; //CLKAUDIO0in/(1+1)
			prescaler = 1;
#else
			epll_or_mpll = 0; //MOUTepll(24MHz)
			audio0_ratio = 0; //CLKAUDIO0in/(0+1)
			prescaler = 1;
#endif
			break;
		case 64000:
			audio0_ratio = 0; //CLKAUDIO0in/(0+1)
			prescaler = 4;
			break;
		case 88200:
			audio0_ratio = 0; //CLKAUDIO0in/(0+1)
			prescaler = 3;
			break;
		case 96000:
			//Bravo doesn't support 24BLC, 96KHz Fs -- couldn't generate IISLRCLK from MOUTepll/DOUTmpll/FINepll
		default:
			printk(KERN_ERR "bravo_hifi_hw_params: S3C6410 doesn't support %d SampleRate\n", params_rate(params));
			return -EINVAL;
		}
		break;
	default:
		printk(KERN_ERR "bravo_hifi_hw_params: S3C6410 doesn't support %d PCM format\n", params_format(params));
		return -EINVAL;
	}

	//Set AUDIO0_RATIO in CLK_DIV2[11:8]
	writel( (readl(S3C_CLK_DIV2)&(~(0xf<<8)))|(audio0_ratio<<8), S3C_CLK_DIV2);
	
	/* Susan -- since MMC controller is using MOUTepll, change IIS to use DOUTmpll for _all_ SampleFrequences(IISLRCK) */
	/* AUDIO0 sel : DOUTmpll */
	writel((readl(S3C_CLK_SRC)&(~(0x7<<7)))|(epll_or_mpll<<7), S3C_CLK_SRC);

	/*SCLK gating enable*/
	writel(readl(S3C_SCLK_GATE)|S3C_CLKCON_SCLK_AUDIO0, S3C_SCLK_GATE);
	s3cdbg("bravo_hifi_hw_params: CLK_SRC(0x%x),CLK_DIV0(0x%x), CLK_DIV1(0x%x), CLK_DIV2(0x%x), MPLL_CON(0x%x)\n", readl(S3C_CLK_SRC), readl(S3C_CLK_DIV0), readl(S3C_CLK_DIV1), readl(S3C_CLK_DIV2), readl(S3C_MPLL_CON));
	s3cdbg("bravo_hifi_hw_params: PCLK_GATE(0x%x),SCLK_GATE(0x%x)\n",readl(S3C_PCLK_GATE),readl(S3C_SCLK_GATE));

	writel(iismod , regs + S3C64XX_IIS0MOD);

	/* set prescaler division for sample rate */
	prescaler = prescaler - 1; 
	ret = cpu_dai->ops.set_clkdiv(cpu_dai, S3C64XX_DIV_PRESCALER, (prescaler << 0x8));
	if (ret < 0)
		return ret;

	/* set cpu DAI configuration */
	ret = cpu_dai->ops.set_fmt(cpu_dai,
		SND_SOC_DAIFMT_I2S | SND_SOC_DAIFMT_NB_NF |
		SND_SOC_DAIFMT_CBS_CFS ); 
	if (ret < 0)
		return ret;

	/* set BLC format in codec */
	ret = codec_dai->ops.hw_params(substream, params, codec_dai); //SNDRV_PCM_FORMAT_S16_LE or SNDRV_PCM_FORMAT_S24_LE
	if (ret<0)
		return ret;
	/* set codec DAI configuration */
	ret = codec_dai->ops.set_fmt(codec_dai,
		SND_SOC_DAIFMT_I2S | SND_SOC_DAIFMT_NB_NF |
		SND_SOC_DAIFMT_CBS_CFS ); 
	if (ret < 0)
		return ret;

	/* set the codec system clock for DAC and ADC */
	ret = codec_dai->ops.set_sysclk(codec_dai, WM8350_MCLK_SEL_MCLK, pll_out, SND_SOC_CLOCK_OUT);
	if (ret < 0)
		return ret;

	/* set SYSCLK(MCLK_DIV), codec ADCCLK_DIV and DACCLK_DIV*/
	ret = snd_soc_dai_set_clkdiv(codec_dai, WM8350_SYS_CLKDIV, 0);  //MCLK_DIV=1
	if (ret < 0)
		return ret;
	ret = snd_soc_dai_set_clkdiv(codec_dai, WM8350_ADC_CLKDIV, 0);  //ADC_CLKDIV=1
	if (ret < 0)
		return ret;
	ret = snd_soc_dai_set_clkdiv(codec_dai, WM8350_DAC_CLKDIV, 0);  //DAC_CLKDIV=1
	if (ret < 0)
		return ret;

	return 0;
}

/*
 * Neo1973 WM8990 HiFi DAI opserations.
 */
static struct snd_soc_ops bravo_hifi_ops = {
	.hw_params = bravo_hifi_hw_params,
};

static int bravo_scenario = 0;

static int bravo_get_scenario(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	ucontrol->value.integer.value[0] = bravo_scenario;
	return 0;
}

static int set_scenario_endpoints(struct snd_soc_codec *codec, int scenario)
{
	bravo_scenario = scenario;
	switch (bravo_scenario) {
	case BRAVO_AUDIO_OFF:
		snd_soc_dapm_disable_pin(codec, "Headphone Jack");
		snd_soc_dapm_disable_pin(codec, "Speaker");
		break;
	case BRAVO_STEREO_TO_SPEAKERS:
		snd_soc_dapm_disable_pin(codec, "Headphone Jack");
		snd_soc_dapm_enable_pin(codec, "Speaker");
		break;
	case BRAVO_STEREO_TO_HEADPHONES:
		snd_soc_dapm_enable_pin(codec, "Headphone Jack");
		snd_soc_dapm_disable_pin(codec, "Speaker");
		break;
	default:
		snd_soc_dapm_enable_pin(codec, "Headphone Jack");
		snd_soc_dapm_enable_pin(codec, "Speaker");
		break;
	}

	snd_soc_dapm_sync(codec);
	return 0;
}

static int bravo_set_scenario(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);

	if (bravo_scenario == ucontrol->value.integer.value[0])
		return 0;

	set_scenario_endpoints(codec, ucontrol->value.integer.value[0]);
	return 1;
}

//static const struct snd_soc_dapm_widget wm8990_dapm_widgets[] = {
static const struct snd_soc_dapm_widget bravo_dapm_widgets[] = {
	SND_SOC_DAPM_HP("Headphone Jack", NULL),
	SND_SOC_DAPM_SPK("Speaker", NULL),
};

/* example machine audio_mapnections */
static const struct snd_soc_dapm_route audio_map[] = {
	/* Headphone L/R */
	{"Headphone Jack", NULL, "OUT1L"},
	{"Headphone Jack", NULL, "OUT1R"},

	/* Speaker */
	{"Speaker", NULL, "Out4 Mixer"},
};

static const char *bravo_scenarios[] = {
	"Off",
	"Speaker",
	"Headphone Jack"
};

static const struct soc_enum bravo_scenario_enum[] = {
	SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(bravo_scenarios), bravo_scenarios),
};

static const struct snd_kcontrol_new wm8350_bravo_controls[] = {
	SOC_ENUM_EXT("BRAVO Mode", bravo_scenario_enum[0],
		bravo_get_scenario, bravo_set_scenario),
	SOC_SINGLE("Bravo Speaker", WM8350_OUT4_MIXER_CONTROL, 15, 1, 0),
};

/*
 * This is an example machine initialisation for a wm8990 connected to a
 * smdk6410. It is missing logic to detect hp/mic insertions and logic
 * to re-route the audio in such an event.
 */
static int bravo_wm8350_init(struct snd_soc_codec *codec)
{
	int i, err;
    unsigned regval;

	/* Add smdk6410 specific widgets */
	for (i = 0; i < ARRAY_SIZE(bravo_dapm_widgets); i++)
		snd_soc_dapm_new_control(codec, &bravo_dapm_widgets[i]);

	/* add smdk6410 specific controls */
	for (i = 0; i < ARRAY_SIZE(wm8350_bravo_controls); i++) {
		err = snd_ctl_add(codec->card,
				snd_soc_cnew(&wm8350_bravo_controls[i],
				codec, NULL));
		if (err < 0)
			return err;
	}

	/* set up smdk6410 specific audio paths */
	snd_soc_dapm_add_routes(codec, audio_map,ARRAY_SIZE(audio_map));

	/* not connected */
	snd_soc_dapm_disable_pin(codec, "IN3L PGA");
	snd_soc_dapm_disable_pin(codec, "IN3R PGA");
	snd_soc_dapm_disable_pin(codec, "IN2L");
	snd_soc_dapm_disable_pin(codec, "IN2R");
	snd_soc_dapm_disable_pin(codec, "IN1LN");
	snd_soc_dapm_disable_pin(codec, "IN1LP");
	snd_soc_dapm_disable_pin(codec, "IN1RN");
	snd_soc_dapm_disable_pin(codec, "IN1RP");
	snd_soc_dapm_disable_pin(codec, "Left Mic Mixer");
	snd_soc_dapm_disable_pin(codec, "Right Mic Mixer");
	snd_soc_dapm_disable_pin(codec, "Right Capture Mixer");
	snd_soc_dapm_disable_pin(codec, "Left Capture Mixer");
	snd_soc_dapm_disable_pin(codec, "Out3 Mixer");
	snd_soc_dapm_disable_pin(codec, "Left Out2 PGA");
	snd_soc_dapm_disable_pin(codec, "Right Out2 PGA");
	//snd_soc_dapm_disable_pin(codec, "Out4 Capture Channel");

	/* set endpoints to default mode & sync with DAPM */
	set_scenario_endpoints(codec, BRAVO_STEREO_TO_HEADPHONES);

    // Configure GPE3 (MUTE) as an output.
 	regval = get_s3c_reg_val(S3C64XX_GPECON);
	regval &= ~(0xf << 12);
	regval |= (0x1 << 12);  // Set GPE3 as output.
	set_s3c_reg_val(S3C64XX_GPECON, regval);

	return 0;
}

static struct snd_soc_dai_link bravo_dai[] = {
{ /* Hifi Playback - for similatious use with voice below */
	.name = "wm8350",
	.stream_name = "wm8350 HiFi",
	.cpu_dai = &s3c64xx_i2s_dai,
	.codec_dai = &wm8350_dai,
	.init = bravo_wm8350_init,
	.ops = &bravo_hifi_ops,
},
};

static struct snd_soc_card bravo_snd_card = {
	.name = "bravo_snd_card",
	.platform = &s3c24xx_soc_platform,
	.dai_link = bravo_dai,
	.num_links = ARRAY_SIZE(bravo_dai),
};

static struct snd_soc_device bravo_snd_devdata = {
	.card = &bravo_snd_card,
	.codec_dev = &soc_codec_dev_wm8350,
};

static struct platform_device *bravo_snd_device;

void bravo_speaker_pm(struct work_struct *w)
{
	volatile unsigned long regval;
	struct platform_pcm_apm *pcm_apm = container_of(w, struct platform_pcm_apm, pm_work);
	struct wm8350 *wm8350_control = (struct wm8350 *)(bravo_snd_devdata.codec->control_data);

	mutex_lock(&(pcm_apm->ctrl_lock));
	//Note -- we are on SUSPEND/RESUME path, don't need to down/up bravo_playback.ctrl_lock
	//GPIO port should have been resumed
	regval = get_s3c_reg_val(S3C64XX_GPNDAT);
        wm8350_reg_write(wm8350_control, WM8350_DAC_DIGITAL_VOLUME_L, 0x81B6);
        wm8350_reg_write(wm8350_control, WM8350_DAC_DIGITAL_VOLUME_R, 0x81B6);

	if (regval & (1<<10))
	{
		//Headphone is not present
		if (pcm_apm->action == 1) //turnon
		{
			pcm_apm->pm = 1;
			//For Speaker<-->OUT4 output path, reduce 6dB by OUT4_ATTN
		   	wm8350_set_bits(wm8350_control, WM8350_OUT4_MIXER_CONTROL, WM8350_OUT4_ATTN);
			wm8350_set_bits(wm8350_control, WM8350_POWER_MGMT_6, WM8350_DC2_ENA);
			printk(KERN_NOTICE "bravo_speaker_pm: TURNON SYS_5V\n");
            
            UnMute();
		}
		if ( (pcm_apm->action == 0) && (pcm_apm->pm == 1) ) //turnoff
		{
            // There is a lot of noise if you don't mute prior to cutting
            // the power.
            Mute();

			pcm_apm->pm = 0;
			printk(KERN_NOTICE "bravo_speaker_pm: TURNOFF SYS_5V\n");
		    wm8350_clear_bits(wm8350_control, WM8350_POWER_MGMT_6, WM8350_DC2_ENA);
		}
		mdelay(1);
	}
	else
	{
		if ((pcm_apm->action == 0) && (pcm_apm->pm == 1)) //Headphone has been plugged-in
		{
			pcm_apm->pm = 0;
			printk(KERN_NOTICE "bravo_speaker_pm: TURNOFF SYS_5V\n");
			wm8350_clear_bits(wm8350_control, WM8350_POWER_MGMT_6, WM8350_DC2_ENA);
			mdelay(1);
		}
	}
	mutex_unlock(&(pcm_apm->ctrl_lock));
}

EXPORT_SYMBOL_GPL(bravo_speaker_pm);

void bravo_speaker_check(void)
{
	volatile unsigned long regval;
	
	//Note -- we are on SUSPEND/RESUME path, don't need to down/up bravo_playback.ctrl_lock
	//GPIO port should have been resumed
	regval = get_s3c_reg_val(S3C64XX_GPNDAT);
	if (regval & (1<<10))
	{
		//Headphone is not present
		switch_set_state(&(bravo_playback.sdev), 0);
		printk(KERN_NOTICE "BRAVO_PLAYBACK_SWITCH: state(0)\n");
	}
	else
	{
		//Headphone is present
		switch_set_state(&(bravo_playback.sdev), 1);
		printk(KERN_NOTICE "BRAVO_PLAYBACK_SWITCH: state(1)\n");
	}
}
EXPORT_SYMBOL_GPL(bravo_speaker_check);

extern int bravo_hp_status(void);
static void bravo_playback_switch(struct work_struct *w)
{
#if (0)
	struct wm8350_data *wm8350_data = NULL;
	struct wm8350 *wm8350 = NULL;
	int hpstatus = 0xF;
	volatile unsigned short regval_pmic;
	wm8350_data = container_of(w, struct wm8350_data, playback_sw);
	wm8350 = (struct wm8350 *)(wm8350_data->codec.control_data);
	if ( (hpstatus = bravo_hp_status()) )  //GPN10 is HIGH -- Headphone is not present
	{
		printk(KERN_NOTICE "wm8350_playback_switch(%s): HIGH\n", current->comm);
		mutex_lock(&(wm8350_data->codec.mutex)); //We can only protect exclusive access of codec control registers via codec.mutex
		//Enable speaker path...		
		regval_pmic = wm8350_reg_read(wm8350, WM8350_OUT4_MIXER_CONTROL);
		regval_pmic |= 0x1800;
		wm8350_reg_write(wm8350, WM8350_OUT4_MIXER_CONTROL, regval_pmic);
		regval_pmic = wm8350_reg_read(wm8350, WM8350_POWER_MGMT_2);
		regval_pmic |= (1<<5);
		wm8350_reg_write(wm8350, WM8350_POWER_MGMT_2, regval_pmic);
		//Disable headphone path...
		regval_pmic = wm8350_reg_read( wm8350, WM8350_POWER_MGMT_3);
		regval_pmic &= ~(0x3);
		wm8350_reg_write(wm8350, WM8350_POWER_MGMT_3, regval_pmic);
		regval_pmic = wm8350_reg_read(wm8350, WM8350_LOUT1_VOLUME);
		regval_pmic &= ~(0x1FF);
		wm8350_reg_write(wm8350, WM8350_LOUT1_VOLUME, regval_pmic);
		regval_pmic = wm8350_reg_read(wm8350, WM8350_ROUT1_VOLUME);
		regval_pmic &= ~(0x1FF);
		regval_pmic |= (0x100);
		wm8350_reg_write(wm8350, WM8350_ROUT1_VOLUME, regval_pmic);
		mutex_unlock(&(wm8350_data->codec.mutex));
	}
	else //GPN10 is LOW (Headphone is present)
	{
		printk(KERN_NOTICE "wm8350_playback_switch(%s): LOW\n", current->comm);		
		mutex_lock(&(wm8350_data->codec.mutex));				
		//Enable headphone path...		
		regval_pmic = wm8350_reg_read(wm8350, WM8350_LOUT1_VOLUME);		
		regval_pmic &= ~(0x1FF);		
		regval_pmic |= (0xFC);		
		wm8350_reg_write(wm8350, WM8350_LOUT1_VOLUME, regval_pmic);		
		regval_pmic = wm8350_reg_read(wm8350, WM8350_ROUT1_VOLUME);		
		regval_pmic &= ~(0x1FF);		
		regval_pmic |= (0x1FC);		
		wm8350_reg_write(wm8350, WM8350_ROUT1_VOLUME, regval_pmic);		
		regval_pmic = wm8350_reg_read( wm8350, WM8350_POWER_MGMT_3);		
		regval_pmic |= (0x3);		
		wm8350_reg_write(wm8350, WM8350_POWER_MGMT_3, regval_pmic);		
		//Disable speaker path...		
		regval_pmic = wm8350_reg_read(wm8350, WM8350_POWER_MGMT_2);		
		regval_pmic &= ~(1<<5);		
		wm8350_reg_write(wm8350, WM8350_POWER_MGMT_2, regval_pmic);				
		regval_pmic = wm8350_reg_read(wm8350, WM8350_OUT4_MIXER_CONTROL);		
		regval_pmic &= ~(0x1800);		
		wm8350_reg_write(wm8350, WM8350_OUT4_MIXER_CONTROL, regval_pmic);		
		mutex_unlock(&(wm8350_data->codec.mutex));	
	}
#endif
	struct delayed_work *dw = (struct delayed_work *)w;
	struct platform_playback *this_playback = container_of(dw, struct platform_playback, playback_sw);

	mutex_lock(&(this_playback->ctrl_lock));

	bravo_speaker_check();

	mutex_unlock(&(this_playback->ctrl_lock));
}

int bravo_hp_status(void)
{
 	volatile unsigned long regval = get_s3c_reg_val(S3C64XX_GPNDAT);
	if ( !(regval & (1<<10)) )
		return 0;
	else
		return 1;
}

void bravo_hp_initstate_check(void)
{
	int state = 0xf;

	state = bravo_hp_status();
	if (!state) //Headphone is present
	{
		mutex_lock(&(bravo_playback.ctrl_lock));
		switch_set_state(&(bravo_playback.sdev), 1);
		mutex_unlock(&(bravo_playback.ctrl_lock));
		printk(KERN_NOTICE "BRAVO_PLAYBACK_SWITCH: state(1)\n");
	}
	else
		printk(KERN_NOTICE "Headphone is not present, use default speaker settings\n");
}

// Sets GPE3 low - mute. 
void Mute()
{
    unsigned regval = get_s3c_reg_val(S3C64XX_GPEDAT);
    regval &= ~(1 << 3);  
	set_s3c_reg_val(S3C64XX_GPEDAT, regval);
}

// Set GPE3 high - don't mute.
void UnMute()
{
	unsigned regval = get_s3c_reg_val(S3C64XX_GPEDAT);
    regval |= (1 << 3);  
    set_s3c_reg_val(S3C64XX_GPEDAT, regval);
}

static irqreturn_t bravo_hp_isr(int irq, void *dev_id)
{
	unsigned long flags;
	struct platform_playback *this_playback = (struct platform_playback *)(dev_id);
	
	
	//printk(KERN_NOTICE "HP_ISR(0)\n");
	disable_irq(irq);
	
	local_irq_save(flags);  //Don't use mutex_lock, that will cause unpredictable result
	schedule_delayed_work(&(this_playback->playback_sw), 1*HZ);
	local_irq_restore(flags);
	
	enable_irq(irq);
	//printk(KERN_NOTICE "HP_ISR(1)\n");
	
	return IRQ_HANDLED;
}

static ssize_t hp_print_switch_name(struct switch_dev *sdev, char *buf)
{
        return sprintf(buf, "h2w\n");
}
 
static ssize_t hp_print_switch_state(struct switch_dev *sdev, char *buf)
{
	volatile unsigned long regval = 0x0;
	regval = get_s3c_reg_val(S3C64XX_GPNDAT);

	if ( !(regval & (1<<10)) ) //LOW--Headphone is represent
		return sprintf(buf, "online\n");
	else //HIGH -- Headphone is not present
		return sprintf(buf, "offline\n");
}

static int __init bravo_snd_init(void)
{
	int ret;

	memset(&bravo_playback, 0x00, sizeof(struct platform_playback));

	bravo_snd_device = platform_device_alloc("soc-audio", 0);
	if (!bravo_snd_device)
		return -ENOMEM;

	platform_set_drvdata(bravo_snd_device, &bravo_snd_devdata);
	bravo_snd_devdata.dev = &bravo_snd_device->dev;
	ret = platform_device_add(bravo_snd_device);
	if (ret)
		goto err_platform_device;
	
	mutex_init(&(bravo_playback.ctrl_lock));
	INIT_DELAYED_WORK(&(bravo_playback.playback_sw), bravo_playback_switch);
	bravo_playback.sdev.name = HEADPHONE_SWITCH_NAME;
	bravo_playback.sdev.print_name = hp_print_switch_name;
	bravo_playback.sdev.print_state = NULL; //HeadsetObserver expects a digital number -- hp_print_switch_state;
	ret = switch_dev_register(&(bravo_playback.sdev));
	if (ret)
		goto err_switch_device;

	ret = request_irq(IRQ_EINT(10), bravo_hp_isr, IRQF_TRIGGER_FALLING|IRQF_TRIGGER_RISING, "soc-audio", &bravo_playback);
	s3c_gpio_setpull(S3C64XX_GPN(10), S3C_GPIO_PULL_NONE);
	s3c_set_irqflt_group0(IRQ_EINT(10),  1, 0x20); //filter count is 32 (FIN clock)

	bravo_hp_initstate_check();
	
	if (!ret)
		return 0;

	ret = -EIO;
	
err_request_irq:
	free_irq(IRQ_EINT(10), &bravo_playback);
	printk(KERN_ERR "request_irq failed on IRQ_EINT(10)\n");
err_switch_device:
	switch_dev_unregister(&(bravo_playback.sdev));
	memset(&bravo_playback, 0x0, sizeof(struct platform_playback));
err_platform_device:
	platform_device_put(bravo_snd_device);
	
	return ret;
}

static void __exit bravo_snd_exit(void)
{
	s3c_set_irqflt_group0(IRQ_EINT(10), 0, 0); //Disable IRQ FLT ENABLE/SELECT
	free_irq(IRQ_EINT(10), &bravo_snd_devdata);
	platform_device_unregister(bravo_snd_device);
}

module_init(bravo_snd_init);
module_exit(bravo_snd_exit);

/* Module information */
MODULE_AUTHOR("Intrinsyc.com");
MODULE_DESCRIPTION("ALSA SoC WM8350 S3C6410");
MODULE_LICENSE("GPL");
