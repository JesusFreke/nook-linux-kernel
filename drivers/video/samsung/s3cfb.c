/*
 * drivers/video/samsung/s3cfb.c
 *
 * $Id: s3cfb.c,v 1.1 2008/11/17 11:12:08 jsgood Exp $
 *
 * Copyright (C) 2008 Jinsung Yang <jsgood.yang@samsung.com>
 *
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file COPYING in the main directory of this archive for
 * more details.
 *
 *	S3C Frame Buffer Driver
 *	based on skeletonfb.c, sa1100fb.h, s3c2410fb.c
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/string.h>
#include <linux/mm.h>
#include <linux/tty.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/fb.h>
#include <linux/init.h>
#include <linux/dma-mapping.h>
#include <linux/interrupt.h>
#include <linux/workqueue.h>
#include <linux/wait.h>
#include <linux/platform_device.h>
#include <linux/clk.h>
#include <linux/vmalloc.h>
#ifndef CONFIG_PM
#undef CONFIG_ANDROID_POWER
#endif
#ifndef CONFIG_HAS_WAKELOCK
#ifdef CONFIG_ANDROID_POWER
#include <linux/android_power.h>
#endif
#else
#include <linux/wakelock.h>
#include <linux/earlysuspend.h>
#endif
#include <asm/io.h>
#include <asm/uaccess.h>
#include <asm/div64.h>
#include <asm/atomic.h>

#include <plat/regs-gpio.h>
#include <plat/regs-lcd.h>
#include <plat/power-clock-domain.h>
#include <linux/i2c/isl29001.h>

#include "s3cfb.h"

#define BRAVO_ABL_ROUT	5*HZ

/*
 *  Globals
 */

s3cfb_info_t s3cfb_info[S3CFB_NUM];

#ifdef CONFIG_FB_BRAVO
s3cfb_info_t *s3c_lcd_fb = NULL;
#endif

struct bravo_auto_bl_ctrl
{
	unsigned int fn_enable;
	atomic_t requested_lvl;
	atomic_t gain;
	atomic_t offset;
	atomic_t lux;
	struct delayed_work auto_bl_control;
};

struct bravo_auto_bl_ctrl global_autobl = {
	.fn_enable = 0,
	.gain = ATOMIC_INIT(20),
	.offset = ATOMIC_INIT(5)
};
EXPORT_SYMBOL(global_autobl);

struct s3cfb_watchdog_control global_lcd_watchdog = {
    .s3cfb_info = NULL,
    .active = ATOMIC_INIT(0) 
};

EXPORT_SYMBOL(global_lcd_watchdog);

extern void s3c_set_normal_cfg(unsigned int config, unsigned int flag, unsigned int deviceID);

void s3cfb_set_lcd_power(int to)
{
	s3cfb_fimd.lcd_power = to;

	if (s3cfb_fimd.set_lcd_power)
		(s3cfb_fimd.set_lcd_power)(to);
}
EXPORT_SYMBOL(s3cfb_set_lcd_power);

void s3cfb_set_backlight_power(int to)
{
	s3cfb_fimd.backlight_power = to;

	if (s3cfb_fimd.set_backlight_power)
		(s3cfb_fimd.set_backlight_power)(to);
}
EXPORT_SYMBOL(s3cfb_set_backlight_power);

void s3cfb_set_backlight_level(int to)
{
	s3cfb_fimd.backlight_level = to;

	if (s3cfb_fimd.set_brightness)
		(s3cfb_fimd.set_brightness)(to);
}
EXPORT_SYMBOL(s3cfb_set_backlight_level);

static int __init s3cfb_map_video_memory(s3cfb_info_t *fbi)
{
	DPRINTK("map_video_memory(fbi=%p)\n", fbi);
	
	fbi->map_size_f1 = PAGE_ALIGN(fbi->fb.fix.smem_len);
	fbi->map_cpu_f1 = dma_alloc_writecombine(fbi->dev, fbi->map_size_f1, &fbi->map_dma_f1, GFP_KERNEL);
	fbi->map_size_f1 = fbi->fb.fix.smem_len;

	if (fbi->map_cpu_f1) {
		/* prevent initial garbage on screen */
		printk("Window[%d] - FB1: map_video_memory: clear %p:%08x\n",
			fbi->win_id, fbi->map_cpu_f1, fbi->map_size_f1);
		memset(fbi->map_cpu_f1, 0x00, fbi->map_size_f1);

		fbi->screen_dma_f1 = fbi->map_dma_f1;
		fbi->fb.screen_base = fbi->map_cpu_f1;
		fbi->fb.fix.smem_start = fbi->screen_dma_f1;

		//void* e_ink_ptr = fbi->fb.screen_base + (EINK_OFFSET_Y * fbi->fb.fix.line_length * s3cfb_fimd.bytes_per_pixel);

		printk("            FB1: map_video_memory: dma=%08x cpu=%p size=%08x\n",
			fbi->map_dma_f1, fbi->map_cpu_f1, fbi->fb.fix.smem_len);
	}

	if (!fbi->map_cpu_f1)
		return -ENOMEM;

#if defined(CONFIG_FB_S3C_DOUBLE_BUFFERING)
	if (fbi->win_id < 2 && fbi->map_cpu_f1) {
		fbi->map_size_f2 = (fbi->fb.fix.smem_len / 2);
		fbi->map_cpu_f2 = fbi->map_cpu_f1 + fbi->map_size_f2;
		fbi->map_dma_f2 = fbi->map_dma_f1 + fbi->map_size_f2;

		/* prevent initial garbage on screen */
		printk("Window[%d] - FB2: map_video_memory: clear %p:%08x\n",
			fbi->win_id, fbi->map_cpu_f2, fbi->map_size_f2);

		fbi->screen_dma_f2 = fbi->map_dma_f2;

		printk("            FB2: map_video_memory: dma=%08x cpu=%p size=%08x\n",
			fbi->map_dma_f2, fbi->map_cpu_f2, fbi->map_size_f2);
	}
#endif

	if (s3cfb_fimd.map_video_memory)
		(s3cfb_fimd.map_video_memory)(fbi);

	return 0;
}

static void s3cfb_unmap_video_memory(s3cfb_info_t *fbi)
{
	dma_free_writecombine(fbi->dev, fbi->map_size_f1, fbi->map_cpu_f1,  fbi->map_dma_f1);

#if defined(CONFIG_FB_S3C_DOUBLE_BUFFERING)
	dma_free_writecombine(fbi->dev, fbi->map_size_f2, fbi->map_cpu_f2,  fbi->map_dma_f2);
#endif

	if (s3cfb_fimd.unmap_video_memory)
		(s3cfb_fimd.unmap_video_memory)(fbi);
}

/*
 *	s3cfb_check_var():
 *	Get the video params out of 'var'. If a value doesn't fit, round it up,
 *	if it's too big, return -EINVAL.
 *
 */
static int s3cfb_check_var(struct fb_var_screeninfo *var, struct fb_info *info)
{
	DPRINTK("check_var(var=%p, info=%p)\n", var, info);

	switch (var->bits_per_pixel) {
		case 8:
			var->red = s3cfb_rgb_8.red;
			var->green = s3cfb_rgb_8.green;
			var->blue = s3cfb_rgb_8.blue;
			var->transp = s3cfb_rgb_8.transp;
			s3cfb_fimd.bytes_per_pixel = 1;
			break;

		case 16:
			var->red = s3cfb_rgb_16.red;
			var->green = s3cfb_rgb_16.green;
			var->blue = s3cfb_rgb_16.blue;
			var->transp = s3cfb_rgb_16.transp;
			s3cfb_fimd.bytes_per_pixel = 2;
			break;

		case 24:
			var->red = s3cfb_rgb_24.red;
			var->green = s3cfb_rgb_24.green;
			var->blue = s3cfb_rgb_24.blue;
			var->transp = s3cfb_rgb_24.transp;
			s3cfb_fimd.bytes_per_pixel = 4;
			break;

		case 32:
			var->red = s3cfb_rgb_32.red;
			var->green = s3cfb_rgb_32.green;
			var->blue = s3cfb_rgb_32.blue;
			var->transp = s3cfb_rgb_32.transp;
			s3cfb_fimd.bytes_per_pixel = 4;
			break;
	}

	return 0;
}

/*
 *      s3cfb_set_par - Optional function. Alters the hardware state.
 *      @info: frame buffer structure that represents a single frame buffer
 *
 */
static int s3cfb_set_par(struct fb_info *info)
{
	struct fb_var_screeninfo *var = &info->var;
	s3cfb_info_t *fbi = (s3cfb_info_t *) info;

        if (var->bits_per_pixel == 16 || var->bits_per_pixel == 24)
		fbi->fb.fix.visual = FB_VISUAL_TRUECOLOR;
	else
		fbi->fb.fix.visual = FB_VISUAL_PSEUDOCOLOR;

	fbi->fb.fix.line_length = var->width * s3cfb_fimd.bytes_per_pixel;

	/* activate this new configuration */
	s3cfb_activate_var(fbi, var);

	return 0;
}

/**
 *	s3cfb_pan_display
 *	@var: frame buffer variable screen structure
 *	@info: frame buffer structure that represents a single frame buffer
 *
 *	Pan (or wrap, depending on the `vmode' field) the display using the
 *	`xoffset' and `yoffset' fields of the `var' structure.
 *	If the values don't fit, return -EINVAL.
 *
 *	Returns negative errno on error, or zero on success.
 */
#ifdef CONFIG_FB_BRAVO
int s3cfb_pan_display(struct fb_var_screeninfo *var, struct fb_info *info)
#else
static int s3cfb_pan_display(struct fb_var_screeninfo *var, struct fb_info *info)
#endif
{
	s3cfb_info_t *fbi = (s3cfb_info_t *)info;

	DPRINTK("s3c_fb_pan_display(var=%p, info=%p)\n", var, info);

	if (var->xoffset != 0)
		return -EINVAL;

	if (var->yoffset + info->var.yres > info->var.yres_virtual)
		return -EINVAL;

	fbi->fb.var.xoffset = var->xoffset;
	fbi->fb.var.yoffset = var->yoffset; 

	s3cfb_set_fb_addr(fbi);
	return 0;
}

/**
 *      s3cfb_blank
 *	@blank_mode: the blank mode we want.
 *	@info: frame buffer structure that represents a single frame buffer
 *
 *	Blank the screen if blank_mode != 0, else unblank. Return 0 if
 *	blanking succeeded, != 0 if un-/blanking failed due to e.g. a
 *	video mode which doesn't support it. Implements VESA suspend
 *	and powerdown modes on hardware that supports disabling hsync/vsync:
 *	blank_mode == 2: suspend vsync
 *	blank_mode == 3: suspend hsync
 *	blank_mode == 4: powerdown
 *
 *	Returns negative errno on error, or zero on success.
 *
 */
static int s3cfb_blank(int blank_mode, struct fb_info *info)
{
	DPRINTK("blank(mode=%d, info=%p)\n", blank_mode, info);

	switch (blank_mode) {
	case VESA_NO_BLANKING:	/* lcd on, backlight on */
		s3cfb_set_lcd_power(1);
		s3cfb_set_backlight_power(1);
		break;

	case VESA_VSYNC_SUSPEND: /* lcd on, backlight off */
	case VESA_HSYNC_SUSPEND:
		s3cfb_set_lcd_power(1);
		s3cfb_set_backlight_power(0);
		break;

	case VESA_POWERDOWN: /* lcd and backlight off */
		s3cfb_set_lcd_power(0);
		s3cfb_set_backlight_power(0);
		break;

	default:
		return -EINVAL;
	}

	return 0;
}

#if defined(CONFIG_FB_S3C_VIRTUAL_SCREEN)
int s3cfb_set_vs_info(s3cfb_vs_info_t vs_info)
{
	/* check invalid value */
	if (vs_info.width != s3cfb_fimd.width || vs_info.height != s3cfb_fimd.height)
		return 1;

	if (!(vs_info.bpp == 8 || vs_info.bpp == 16 || vs_info.bpp == 24 || vs_info.bpp == 32))
		return 1;

	if (vs_info.offset < 0)
		return 1;

	if (vs_info.v_width != s3cfb_fimd.xres_virtual || vs_info.v_height != s3cfb_fimd.yres_virtual)
		return 1;

	/* save virtual screen information */
	s3cfb_fimd.vs_info = vs_info;

	if (s3cfb_fimd.vs_info.offset < 1)
		s3cfb_fimd.vs_info.offset = 1;

	if (s3cfb_fimd.vs_info.offset > S3CFB_MAX_DISPLAY_OFFSET)
		s3cfb_fimd.vs_info.offset = S3CFB_MAX_DISPLAY_OFFSET;

	s3cfb_fimd.vs_offset = s3cfb_fimd.vs_info.offset;

	return 0;
}
#endif

int s3cfb_onoff_win(s3cfb_info_t *fbi, int onoff)
{
	int win_num =  fbi->win_id;

	if (onoff)
		writel(readl(S3C_WINCON0 + (0x04 * win_num)) | S3C_WINCONx_ENWIN_F_ENABLE, S3C_WINCON0 + (0x04 * win_num));
	else
		writel(readl(S3C_WINCON0 + (0x04 * win_num)) &~ (S3C_WINCONx_ENWIN_F_ENABLE), S3C_WINCON0 + (0x04 * win_num));

	return 0;
}

int s3cfb_onoff_color_key_alpha(s3cfb_info_t *fbi, int onoff)
{
	int win_num =  fbi->win_id - 1;

	if (onoff)
		writel(readl(S3C_W1KEYCON0 + (0x08 * win_num)) | S3C_WxKEYCON0_KEYBLEN_ENABLE, S3C_W1KEYCON0 + (0x08 * win_num));
	else
		writel(readl(S3C_W1KEYCON0 + (0x08 * win_num)) &~ (S3C_WxKEYCON0_KEYBLEN_ENABLE), S3C_W1KEYCON0 + (0x08 * win_num));

	return 0;
}

int s3cfb_onoff_color_key(s3cfb_info_t *fbi, int onoff)
{
	int win_num =  fbi->win_id - 1;

	if (onoff)
		writel(readl(S3C_W1KEYCON0 + (0x08 * win_num)) | S3C_WxKEYCON0_KEYEN_F_ENABLE, S3C_W1KEYCON0 + (0x08 * win_num));
	else
		writel(readl(S3C_W1KEYCON0 + (0x08 * win_num)) &~ (S3C_WxKEYCON0_KEYEN_F_ENABLE), S3C_W1KEYCON0 + (0x08 * win_num));

	return 0;
}

int s3cfb_set_color_key_registers(s3cfb_info_t *fbi, s3cfb_color_key_info_t colkey_info)
{
	unsigned int compkey = 0;
	int win_num =  fbi->win_id;

	if (win_num == 0) {
		printk("WIN0 do not support color key\n");
		return -1;
	}

	win_num--;

	if (fbi->fb.var.bits_per_pixel == S3CFB_PIXEL_BPP_16) {
		/* RGB 5-6-5 mode */
		compkey  = (((colkey_info.compkey_red & 0x1f) << 19) | 0x70000);
		compkey |= (((colkey_info.compkey_green & 0x3f) << 10) | 0x300);
		compkey |= (((colkey_info.compkey_blue  & 0x1f)  << 3 )| 0x7);
	} else if (fbi->fb.var.bits_per_pixel == S3CFB_PIXEL_BPP_24) {
		/* currently RGB 8-8-8 mode  */
		compkey  = ((colkey_info.compkey_red & 0xff) << 16);
		compkey |= ((colkey_info.compkey_green & 0xff) << 8);
		compkey |= ((colkey_info.compkey_blue & 0xff) << 0);
	} else
		printk("Invalid BPP has been given!\n");

	if (colkey_info.direction == S3CFB_COLOR_KEY_DIR_BG)
		writel(S3C_WxKEYCON0_COMPKEY(compkey) | S3C_WxKEYCON0_DIRCON_MATCH_FG_IMAGE, S3C_W1KEYCON0 + (0x08 * win_num));

	else if (colkey_info.direction == S3CFB_COLOR_KEY_DIR_FG)
		writel(S3C_WxKEYCON0_COMPKEY(compkey) | S3C_WxKEYCON0_DIRCON_MATCH_BG_IMAGE, S3C_W1KEYCON0 + (0x08 * win_num));

	else
		printk("Color key direction is not correct :: %d!\n", colkey_info.direction);

	return 0;
}

int s3cfb_set_color_value(s3cfb_info_t *fbi, s3cfb_color_val_info_t colval_info)
{
	unsigned int colval = 0;

	int win_num =  fbi->win_id;

	if (win_num == 0) {
		printk("WIN0 do not support color key value\n");
		return -1;
	}

	win_num--;

	if (fbi->fb.var.bits_per_pixel == S3CFB_PIXEL_BPP_16) {
		/* RGB 5-6-5 mode */
		colval  = (((colval_info.colval_red   & 0x1f) << 19) | 0x70000);
		colval |= (((colval_info.colval_green & 0x3f) << 10) | 0x300);
		colval |= (((colval_info.colval_blue  & 0x1f)  << 3 )| 0x7);
	} else if (fbi->fb.var.bits_per_pixel == S3CFB_PIXEL_BPP_24) {
		/* currently RGB 8-8-8 mode  */
		colval  = ((colval_info.colval_red  & 0xff) << 16);
		colval |= ((colval_info.colval_green & 0xff) << 8);
		colval |= ((colval_info.colval_blue  & 0xff) << 0);
	} else
		printk("Invalid BPP has been given!\n");

	writel(S3C_WxKEYCON1_COLVAL(colval), S3C_W1KEYCON1 + (0x08 * win_num));

	return 0;
}

static int s3cfb_set_bpp(s3cfb_info_t *fbi, int bpp)
{
	struct fb_var_screeninfo *var= &fbi->fb.var;
	int win_num =  fbi->win_id;
	unsigned int val;

	val = readl(S3C_WINCON0 + (0x04 * win_num));
	val &= ~(S3C_WINCONx_BPPMODE_F_MASK | S3C_WINCONx_BLD_PIX_MASK);
	val |= S3C_WINCONx_ALPHA_SEL_1;

	switch (bpp) {
	case 1:
	case 2:
	case 4:
	case 8:
		s3cfb_fimd.bytes_per_pixel = 1;
		break;

	case 16:
		writel(val | S3C_WINCONx_BPPMODE_F_16BPP_565 | S3C_WINCONx_BLD_PIX_PLANE, S3C_WINCON0 + (0x04 * win_num));
		var->bits_per_pixel = bpp;
		s3cfb_fimd.bytes_per_pixel = 2;
		break;

	case 24:
		writel(val | S3C_WINCONx_BPPMODE_F_24BPP_888 | S3C_WINCONx_BLD_PIX_PLANE, S3C_WINCON0 + (0x04 * win_num));
		var->bits_per_pixel = bpp;
		s3cfb_fimd.bytes_per_pixel = 4;
		break;

	case 25:
		writel(val | S3C_WINCONx_BPPMODE_F_25BPP_A888 | S3C_WINCONx_BLD_PIX_PLANE, S3C_WINCON0 + (0x04 * win_num));
		var->bits_per_pixel = bpp;
		s3cfb_fimd.bytes_per_pixel = 4;
		break;

	case 28:
		writel(val | S3C_WINCONx_BPPMODE_F_28BPP_A888 | S3C_WINCONx_BLD_PIX_PIXEL, S3C_WINCON0 + (0x04 * win_num));
		var->bits_per_pixel = bpp;
		s3cfb_fimd.bytes_per_pixel = 4;
		break;

	case 32:
		var->bits_per_pixel = bpp;
		s3cfb_fimd.bytes_per_pixel = 4;
		break;
	}

	return 0;
}

void s3cfb_stop_lcd(void)
{
	unsigned long flags;
	unsigned long tmp;

	local_irq_save(flags);

	tmp = readl(S3C_VIDCON0);
	writel(tmp & ~(S3C_VIDCON0_ENVID_ENABLE | S3C_VIDCON0_ENVID_F_ENABLE), S3C_VIDCON0);

	local_irq_restore(flags);
}

EXPORT_SYMBOL(s3cfb_stop_lcd);

void s3cfb_start_lcd(void)
{
	unsigned long flags;
	unsigned long tmp;

	local_irq_save(flags);

	tmp = readl(S3C_VIDCON0);
	writel(tmp | S3C_VIDCON0_ENVID_ENABLE | S3C_VIDCON0_ENVID_F_ENABLE, S3C_VIDCON0);

	local_irq_restore(flags);
}

EXPORT_SYMBOL(s3cfb_start_lcd);

void s3cfb_set_clock(unsigned int clkval)
{
	unsigned int tmp;

	tmp = readl(S3C_VIDCON0);

	tmp &= ~(0x1 << 4);
	tmp &= ~(0xff << 6);

	writel(tmp | (clkval << 6) | (1 << 4), S3C_VIDCON0);
}

EXPORT_SYMBOL(s3cfb_set_clock);

// Move LCD window over virtual screen
void s3cfb_set_offset(unsigned int offset)
{
    s3cfb_fimd.fixed_yoffset = (s3cfb_fimd.height -  s3cfb_fimd.lcd_height) - (offset * s3cfb_fimd.lcd_height);
    if (s3cfb_fimd.fixed_yoffset < 0)
        s3cfb_fimd.fixed_yoffset = 0;
    s3cfb_set_fb_addr(&s3cfb_info[0]);
}

EXPORT_SYMBOL(s3cfb_set_offset);


int s3cfb_init_win(s3cfb_info_t *fbi, int bpp, int left_x, int top_y, int width, int height, int onoff)
{
	s3cfb_onoff_win(fbi, OFF);
	s3cfb_set_bpp(fbi, bpp);
	s3cfb_set_win_position(fbi, left_x, top_y, width, height);
	s3cfb_set_win_size(fbi, width, height);
	s3cfb_set_fb_size(fbi);
	s3cfb_onoff_win(fbi, onoff);

	return 0;
}

int s3cfb_wait_for_vsync(void)
{
	int cnt;

	cnt = s3cfb_fimd.vsync_info.count;
	wait_event_interruptible_timeout(s3cfb_fimd.vsync_info.wait_queue, cnt != s3cfb_fimd.vsync_info.count, HZ / 10);

  	return cnt;
}

static void s3cfb_update_palette(s3cfb_info_t *fbi, unsigned int regno, unsigned int val)
{
	unsigned long flags;

	local_irq_save(flags);

	fbi->palette_buffer[regno] = val;

	if (!fbi->palette_ready) {
		fbi->palette_ready = 1;
		s3cfb_fimd.palette_win = fbi->win_id;
	}

	local_irq_restore(flags);
}

static inline unsigned int s3cfb_chan_to_field(unsigned int chan, struct fb_bitfield bf)
{
	chan &= 0xffff;
	chan >>= 16 - bf.length;

	return chan << bf.offset;
}

static int s3cfb_setcolreg(unsigned int regno, unsigned int red, unsigned int green, unsigned int blue, unsigned int transp, struct fb_info *info)
{
	s3cfb_info_t *fbi = (s3cfb_info_t *)info;
	unsigned int val = 0;

	switch (fbi->fb.fix.visual) {
	case FB_VISUAL_TRUECOLOR:
		if (regno < 16) {
			unsigned int *pal = fbi->fb.pseudo_palette;

			val = s3cfb_chan_to_field(red, fbi->fb.var.red);
			val |= s3cfb_chan_to_field(green, fbi->fb.var.green);
			val |= s3cfb_chan_to_field(blue, fbi->fb.var.blue);

			pal[regno] = val;
		}

		break;

	case FB_VISUAL_PSEUDOCOLOR:
		if (regno < 256) {
//			if (info->var.bits_per_pixel == 16) {
				val = ((red >> 0) & 0xf800);
				val |= ((green >> 5) & 0x07e0);
				val |= ((blue >> 11) & 0x001f);
//			} else if (info->var.bits_per_pixel == 24) {
//				val = ((red << 8) & 0xff0000);
//				val |= ((green >> 0) & 0xff00);
//				val |= ((blue >> 8) & 0xff);
//			}

			DPRINTK("index = %d, val = 0x%08x\n", regno, val);
			s3cfb_update_palette(fbi, regno, val);
		}

		break;

	default:
		return 1;	/* unknown type */
	}

	return 0;
}

static int match_any(struct device *dev, void *data)
{
	return 1;
}

/**
 * Read the light level using the sensor driver
 * Returns -1 in case of error, or else the raw value of the light sensor
 */
static int read_lux_value(void)
{
	int lux = -1;
	int rc = -1;
	// Find the first I2C device for the ISL29001 driver, a.k.a. "0-0044"
	struct device *dev = driver_find_device(&isl29001_driver.driver, NULL, NULL, match_any);
	if (dev) {
		struct i2c_client *client = to_i2c_client(dev);
		if (i2c_use_client(client)) {
			rc = client->driver->command(client, ISL29001_CMD_READ_LUX, &lux);
			i2c_release_client(client);
		}
		put_device(dev);
	}
	if (rc)
		printk("Error %d in read_lux_value\n", rc);
	return lux;
}

/**
 * Calculate the effective backlight power level to use based on ambient light
 * @requested: requested value, range 0..255
 * @lux: ambient light range 0..32767
 *
 * Returns a value from approx. 0..255, but possibly outside that range.
 */
static int calc_bl_power(int requested, int lux)
{
	return (requested * (lux + atomic_read(&global_autobl.offset))) /
		atomic_read(&global_autobl.gain);
}

/** Read the current light level and store it for use until the next update */
static void cache_light_level(void)
{
	atomic_set(&global_autobl.lux, read_lux_value());
}

/** auto backlight control */
static void set_auto_backlight(void)
{
	int lux = atomic_read(&global_autobl.lux);
	int level = atomic_read(&global_autobl.requested_lvl);

	//get light sensor value!
	if ( lux < 0 ) {
		printk("Auto light control not working, invalid light sensor reading value!\n");
	} else {
		level = calc_bl_power(level, lux);
		level = max(s3cfb_fimd.backlight_min, min(s3cfb_fimd.backlight_max, level));
		DPRINTK("Auto backlight requested %d set %d lux %d\n", atomic_read(&global_autobl.requested_lvl), level, lux);
	}
	s3cfb_fimd.backlight_level = level;
	if (s3cfb_fimd.set_brightness)
		(s3cfb_fimd.set_brightness)(level);
}

/** Entry point for work task */
static void bravo_auto_brightness_control(struct work_struct *work)
{
	/* Only bother reading the current light level if the backlight is turned on */
	if (atomic_read(&global_autobl.requested_lvl) > 0)
		cache_light_level();
	set_auto_backlight();
	schedule_delayed_work(&global_autobl.auto_bl_control, BRAVO_ABL_ROUT);
}

/* sysfs export of backlight control */
static int s3cfb_sysfs_show_lcd_power(struct device *dev, struct device_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%d\n", s3cfb_fimd.lcd_power);
}

static int s3cfb_sysfs_store_lcd_power(struct device *dev, struct device_attribute *attr, const char *buf, size_t len)
{
	s3cfb_info_t *info;

	if (len < 1)
		return -EINVAL;

	info = container_of(dev, s3cfb_info_t, dev);

	if (strnicmp(buf, "on", 2) == 0 || strnicmp(buf, "1", 1) == 0) {
/*		s3cfb_set_lcd_power(1);
        s3c_set_normal_cfg(S3C64XX_DOMAIN_F, S3C64XX_ACTIVE_MODE, S3C64XX_ROT);
        s3c_set_normal_cfg(S3C64XX_DOMAIN_F, S3C64XX_ACTIVE_MODE, S3C64XX_LCD);
        s3c_set_normal_cfg(S3C64XX_DOMAIN_F, S3C64XX_ACTIVE_MODE, S3C64XX_POST);
		s3cfb_late_resume(&info->early_suspend);*/
	}	
	else if (strnicmp(buf, "off", 3) == 0 || strnicmp(buf, "0", 1) == 0) {
/*		s3cfb_set_lcd_power(0);
		s3cfb_early_suspend(&info->early_suspend); 
        s3c_set_normal_cfg(S3C64XX_DOMAIN_F, S3C64XX_LP_MODE, S3C64XX_ROT);
        s3c_set_normal_cfg(S3C64XX_DOMAIN_F, S3C64XX_LP_MODE, S3C64XX_LCD);
        s3c_set_normal_cfg(S3C64XX_DOMAIN_F, S3C64XX_LP_MODE, S3C64XX_POST);*/

	}
	else {
		return -EINVAL;
	}
	return len;
}

static int s3cfb_sysfs_show_backlight_power(struct device *dev, struct device_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%d\n", s3cfb_fimd.backlight_power);
}

static int s3cfb_sysfs_store_backlight_power(struct device *dev, struct device_attribute *attr, const char *buf, size_t len)
{
	if (len < 1)
		return -EINVAL;

	if (strnicmp(buf, "on", 2) == 0 || strnicmp(buf, "1", 1) == 0)
		s3cfb_set_backlight_power(1);
	else if (strnicmp(buf, "off", 3) == 0 || strnicmp(buf, "0", 1) == 0)
		s3cfb_set_backlight_power(0);
	else
		return -EINVAL;

	return len;
}

static int s3cfb_sysfs_show_backlight_level(struct device *dev, struct device_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%d\n", s3cfb_fimd.backlight_level);
}

static int s3cfb_sysfs_store_backlight_level(struct device *dev, struct device_attribute *attr, const char *buf, size_t len)
{
	unsigned long value = simple_strtoul(buf, NULL, 10);
	int need_new_lux;

	if (value < s3cfb_fimd.backlight_min || value > s3cfb_fimd.backlight_max)
		return -ERANGE;

	/* Since the light level is not polled if the backlight is off,
	 * when it's turned on we need to read the latest value.
	 */
	need_new_lux = global_autobl.fn_enable && (atomic_read(&global_autobl.requested_lvl) == 0) && (value > 0);

	atomic_set(&global_autobl.requested_lvl, value);
	if ( global_autobl.fn_enable ) {
		if (need_new_lux)
			cache_light_level();
		set_auto_backlight();
	} else {
		s3cfb_set_backlight_level(value);
	}

	return len;
}

static int s3cfb_sysfs_show_auto_bl(struct device *dev, struct device_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%d\n", global_autobl.fn_enable);
}

static int s3cfb_sysfs_store_auto_bl(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	if ( strnicmp(buf, "1", 1) == 0 )
		global_autobl.fn_enable = 1;
	else if ( strnicmp(buf, "0", 1) == 0 )
		global_autobl.fn_enable = 0;
	else 
		return -EINVAL;
 
	if ( global_autobl.fn_enable ) {
		cache_light_level();
		set_auto_backlight();
		schedule_delayed_work(&global_autobl.auto_bl_control, BRAVO_ABL_ROUT); 
	} else {
		cancel_delayed_work(&global_autobl.auto_bl_control);
		s3cfb_set_backlight_level(atomic_read(&global_autobl.requested_lvl));
	}
	return count;
}

static int s3cfb_sysfs_show_auto_bl_gain(struct device *dev, struct device_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%d\n", atomic_read(&global_autobl.gain));
}

static int s3cfb_sysfs_store_auto_bl_gain(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	atomic_set(&global_autobl.gain, (int)simple_strtol(buf, NULL, 10));
	return count;
}

static int s3cfb_sysfs_show_auto_bl_ofs(struct device *dev, struct device_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%d\n", atomic_read(&global_autobl.offset));
}

static int s3cfb_sysfs_store_auto_bl_ofs(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	atomic_set(&global_autobl.offset, (int)simple_strtol(buf, NULL, 10));
	return count;
}

// 0 or 1 to turn on lcd power
static DEVICE_ATTR(lcd_power, 0666,
			s3cfb_sysfs_show_lcd_power,
			s3cfb_sysfs_store_lcd_power);

// 0 or 1 to turn on backlight power
static DEVICE_ATTR(backlight_power, 0666,
			s3cfb_sysfs_show_backlight_power,
			s3cfb_sysfs_store_backlight_power);

// BACKLIGHT_LEVEL_MIN to BACKLIGHT_LEVEL_MAX to set backlight level
static DEVICE_ATTR(backlight_level, 0666,
			s3cfb_sysfs_show_backlight_level,
			s3cfb_sysfs_store_backlight_level);
			
// 0 or 1 to enable automatic backlight control
static DEVICE_ATTR(auto_bl_ctrl, 0666,
			s3cfb_sysfs_show_auto_bl,
			s3cfb_sysfs_store_auto_bl);

// integer setting relative amount of effect of the ambient light on the backlight
// (larger numbers have less effect)
static DEVICE_ATTR(auto_bl_gain, 0666,
			s3cfb_sysfs_show_auto_bl_gain,
			s3cfb_sysfs_store_auto_bl_gain);

// integer setting how close to black the ambient light can take the backlight
// (larger numbers are farther from black)
static DEVICE_ATTR(auto_bl_ofs, 0666,
			s3cfb_sysfs_show_auto_bl_ofs,
			s3cfb_sysfs_store_auto_bl_ofs);

struct fb_ops s3cfb_ops = {
	.owner		= THIS_MODULE,
	.fb_check_var	= s3cfb_check_var,
	.fb_set_par	= s3cfb_set_par,
	.fb_blank	= s3cfb_blank,
	.fb_pan_display	= s3cfb_pan_display,
	.fb_setcolreg	= s3cfb_setcolreg,
	.fb_fillrect	= cfb_fillrect,
	.fb_copyarea	= cfb_copyarea,
	.fb_imageblit	= cfb_imageblit,
#if defined(CONFIG_FRAMEBUFFER_CONSOLE)	
	.fb_cursor	= soft_cursor,
#endif
	.fb_ioctl	= s3cfb_ioctl,
};

static void s3cfb_init_fbinfo(s3cfb_info_t *finfo, char *drv_name, int index)
{
	int i = 0;

	if (index == 0) {
#ifdef CONFIG_FB_S3C_PICTURETUBES
		s3cfb_init_hw(finfo->dev);
#else
		s3cfb_init_hw();
#endif /* CONFIG_FB_S3C_PICTURE_TUBES */
	}

	strcpy(finfo->fb.fix.id, drv_name);

	finfo->win_id = index;
	finfo->fb.fix.type = FB_TYPE_PACKED_PIXELS;
	finfo->fb.fix.type_aux = 0;
	finfo->fb.fix.xpanstep = 0;
	finfo->fb.fix.ypanstep = 1;
	finfo->fb.fix.ywrapstep = 0;
	finfo->fb.fix.accel = FB_ACCEL_NONE;

	finfo->fb.fbops = &s3cfb_ops;
	finfo->fb.flags	= FBINFO_FLAG_DEFAULT;

	finfo->fb.pseudo_palette = &finfo->pseudo_pal;

	finfo->fb.var.nonstd = 0;
	finfo->fb.var.activate = FB_ACTIVATE_NOW;
	finfo->fb.var.accel_flags = 0;
	finfo->fb.var.vmode = FB_VMODE_NONINTERLACED;

	finfo->fb.var.xoffset = s3cfb_fimd.xoffset;
	finfo->fb.var.yoffset = s3cfb_fimd.yoffset;

	if (index == 0) {
		finfo->fb.var.height = s3cfb_fimd.height;
		finfo->fb.var.width = s3cfb_fimd.width;

		finfo->fb.var.xres = s3cfb_fimd.xres;
		finfo->fb.var.yres = s3cfb_fimd.yres;

		finfo->fb.var.xres_virtual = s3cfb_fimd.xres_virtual;
		finfo->fb.var.yres_virtual = s3cfb_fimd.yres_virtual;
	} else {
		finfo->fb.var.height = s3cfb_fimd.osd_height;
		finfo->fb.var.width = s3cfb_fimd.osd_width;

		finfo->fb.var.xres = s3cfb_fimd.osd_xres;
		finfo->fb.var.yres = s3cfb_fimd.osd_yres;

		finfo->fb.var.xres_virtual = s3cfb_fimd.osd_xres_virtual;
		finfo->fb.var.yres_virtual = s3cfb_fimd.osd_yres_virtual;
	}

	finfo->fb.var.bits_per_pixel = s3cfb_fimd.bpp;
    finfo->fb.var.pixclock = s3cfb_fimd.pixclock;
	finfo->fb.var.hsync_len = s3cfb_fimd.hsync_len;
	finfo->fb.var.left_margin = s3cfb_fimd.left_margin;
	finfo->fb.var.right_margin = s3cfb_fimd.right_margin;
	finfo->fb.var.vsync_len = s3cfb_fimd.vsync_len;
	finfo->fb.var.upper_margin = s3cfb_fimd.upper_margin;
	finfo->fb.var.lower_margin = s3cfb_fimd.lower_margin;
	finfo->fb.var.sync = s3cfb_fimd.sync;
	finfo->fb.var.grayscale = s3cfb_fimd.cmap_grayscale;

	finfo->fb.fix.smem_len = finfo->fb.var.xres_virtual * finfo->fb.var.yres_virtual * s3cfb_fimd.bytes_per_pixel;

	finfo->fb.fix.line_length = finfo->fb.var.width * s3cfb_fimd.bytes_per_pixel;

#if !defined(CONFIG_FB_S3C_VIRTUAL_SCREEN) && defined(CONFIG_FB_S3C_DOUBLE_BUFFERING)
	if (index < 2)
		finfo->fb.fix.smem_len *= 2;
#endif

	for (i = 0; i < 256; i++)
		finfo->palette_buffer[i] = S3CFB_PALETTE_BUFF_CLEAR;
}

static void s3cfb_watchdog(struct work_struct *work)
{
    s3cfb_info_t *info = global_lcd_watchdog.s3cfb_info;

    if ((info != NULL) && atomic_read(&global_lcd_watchdog.active))
    {
        printk(KERN_NOTICE "s3cfb_watchdog: Timedout\n");
        // Recover LCD
        clk_enable(info->clk);
        s3cfb_start_lcd();
        s3cfb_set_lcd_power(ON);
    } 
}

/*
 *  Probe
 */
static int __init s3cfb_probe(struct platform_device *pdev)
{
	struct resource *res;
	struct fb_info *fbinfo;
	s3cfb_info_t *info;

	char driver_name[] = "s3cfb";
	int index = 0, ret, size;

	fbinfo = framebuffer_alloc(sizeof(s3cfb_info_t), &pdev->dev);

	if (!fbinfo)
		return -ENOMEM;

	platform_set_drvdata(pdev, fbinfo);

	info = fbinfo->par;
	info->dev = &pdev->dev;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);

	if (res == NULL) {
		dev_err(&pdev->dev, "failed to get memory registers\n");
		ret = -ENXIO;
		goto dealloc_fb;
	}

	size = (res->end - res->start) + 1;
	info->mem = request_mem_region(res->start, size, pdev->name);

	if (info->mem == NULL) {
		dev_err(&pdev->dev, "failed to get memory region\n");
		ret = -ENOENT;
		goto dealloc_fb;
	}

	info->io = ioremap(res->start, size);

	if (info->io == NULL) {
		dev_err(&pdev->dev, "ioremap() of registers failed\n");
		ret = -ENXIO;
		goto release_mem;
	}

	s3cfb_pre_init();

	info->clk = clk_get(NULL, "clk_27m");

	if (!info->clk || IS_ERR(info->clk)) {
		printk(KERN_INFO "failed to get lcd clock source\n");
		ret =  -ENOENT;
		goto release_io;
	}

	clk_enable(info->clk);
	printk("S3C_LCD clock got enabled :: %ld.%03ld Mhz\n", PRINT_MHZ(clk_get_rate(info->clk)));

	s3cfb_fimd.vsync_info.count = 0;
	init_waitqueue_head(&s3cfb_fimd.vsync_info.wait_queue);

	res = platform_get_resource(pdev, IORESOURCE_IRQ, 0);

	if (res == NULL) {
		dev_err(&pdev->dev, "failed to get irq\n");
		ret = -ENXIO;
		goto release_clock;
	}

	ret = request_irq(res->start, s3cfb_irq, 0, "s3c-lcd", pdev);

	if (ret != 0) {
		printk("Failed to install irq (%d)\n", ret);
		goto release_clock;
	}

	msleep(5);

	for (index = 0; index < CONFIG_FB_S3C_NUM; index++) {
		s3cfb_info[index].mem = info->mem;
		s3cfb_info[index].io = info->io;
		s3cfb_info[index].clk = info->clk;

		s3cfb_init_fbinfo(&s3cfb_info[index], driver_name, index);

		/* Initialize video memory */
		ret = s3cfb_map_video_memory(&s3cfb_info[index]);

		if (ret) {
			printk("Failed to allocate video RAM: %d\n", ret);
			ret = -ENOMEM;
			goto release_irq;
		}

		ret = s3cfb_init_registers(&s3cfb_info[index]);
		ret = s3cfb_check_var(&s3cfb_info[index].fb.var, &s3cfb_info[index].fb);
		
		s3cfb_set_fb_addr(&s3cfb_info[index]);
		
		if (index < 2){
			if (fb_alloc_cmap(&s3cfb_info[index].fb.cmap, 256, 0) < 0)
				goto dealloc_fb;
		} else {
			if (fb_alloc_cmap(&s3cfb_info[index].fb.cmap, 16, 0) < 0)
				goto dealloc_fb;
		}

#ifdef CONFIG_FB_BRAVO
		if (index == 0) {
			s3c_lcd_fb = (s3cfb_info_t *) &s3cfb_info[index].fb;
		}
#endif
		ret = register_framebuffer(&s3cfb_info[index].fb);

		if (ret < 0) {
			printk(KERN_ERR "Failed to register framebuffer device: %d\n", ret);
			goto free_video_memory;
		}

		printk(KERN_INFO "fb%d: %s frame buffer device\n", s3cfb_info[index].fb.node, s3cfb_info[index].fb.fix.id);
	}
#ifndef CONFIG_HAS_WAKELOCK
#ifdef CONFIG_ANDROID_POWER
	info->idle_lock.name = "s3cfb_idle_lock";
	android_init_suspend_lock(&info->idle_lock);
	
	info->early_suspend.suspend = s3cfb_early_suspend;
	info->early_suspend.resume = s3cfb_late_resume;
	info->early_suspend.level = ANDROID_EARLY_SUSPEND_LEVEL_DISABLE_FB;
	android_register_early_suspend(&info->early_suspend);
#endif
#else
	wake_lock_init(&info->idle_lock, WAKE_LOCK_IDLE, "s3cfb_idle_lock");
#ifdef CONFIG_HAS_EARLYSUSPEND
	info->early_suspend.suspend = s3cfb_early_suspend;
	info->early_suspend.resume = s3cfb_late_resume;
	info->early_suspend.level = EARLY_SUSPEND_LEVEL_DISABLE_FB;
	register_early_suspend(&info->early_suspend);
#endif
#endif

	/* create device files */
	ret = device_create_file(&(pdev->dev), &dev_attr_backlight_power);
	if (ret < 0)
		printk(KERN_WARNING "s3cfb: failed to add entries\n");

	ret = device_create_file(&(pdev->dev), &dev_attr_backlight_level);
	if (ret < 0)
		printk(KERN_WARNING "s3cfb: failed to add entries\n");

	ret = device_create_file(&(pdev->dev), &dev_attr_lcd_power);
	if (ret < 0)
		printk(KERN_WARNING "s3cfb: failed to add entries\n");
	
	ret = device_create_file(&(pdev->dev), &dev_attr_auto_bl_ctrl);
	if (ret < 0)
		printk(KERN_WARNING "s3cfb: failed to add entries\n");

	ret = device_create_file(&(pdev->dev), &dev_attr_auto_bl_gain);
	if (ret < 0)
		printk(KERN_WARNING "s3cfb: failed to add entries\n");

	ret = device_create_file(&(pdev->dev), &dev_attr_auto_bl_ofs);
	if (ret < 0)
		printk(KERN_WARNING "s3cfb: failed to add entries\n");

	// Initialize the LCD watchdog
	global_lcd_watchdog.s3cfb_info = info;
	INIT_DELAYED_WORK(&global_lcd_watchdog.watchdog_control, s3cfb_watchdog);

	s3cfb_set_lcd_power(1);
	s3cfb_set_backlight_power(1);
	//s3cfb_set_backlight_level(S3CFB_DEFAULT_BACKLIGHT_LEVEL);
#if 1	//Benny 0720 auto backlight control
	INIT_DELAYED_WORK(&global_autobl.auto_bl_control, bravo_auto_brightness_control);
	if ( global_autobl.fn_enable )
		schedule_delayed_work(&global_autobl.auto_bl_control, BRAVO_ABL_ROUT);
#endif		
	
	// Activate the LCD watchdog   
	atomic_set(&global_lcd_watchdog.active, 1);

	return 0;

free_video_memory:
	s3cfb_unmap_video_memory(&s3cfb_info[index]);

release_irq:
	free_irq(res->start, &info);

release_clock:
	clk_disable(info->clk);
	clk_put(info->clk);

release_io:
	iounmap(info->io);

release_mem:
	release_resource(info->mem);
	kfree(info->mem);

dealloc_fb:
	framebuffer_release(fbinfo);
	return ret;
}

/*
 *  Remove
 */
static int s3cfb_remove(struct platform_device *pdev)
{
	struct fb_info *fbinfo = platform_get_drvdata(pdev);
	s3cfb_info_t *info = fbinfo->par;
	int index = 0, irq;

	cancel_delayed_work_sync(&global_autobl.auto_bl_control);

	s3cfb_stop_lcd();
	msleep(1);

	if (info->clk) {
		clk_disable(info->clk);
		clk_put(info->clk);
	 	info->clk = NULL;
	}

	irq = platform_get_irq(pdev, 0);
	release_resource(info->mem);

	for (index = 0; index < S3CFB_NUM; index++) {
		s3cfb_unmap_video_memory((s3cfb_info_t *) &s3cfb_info[index]);
		free_irq(irq, &s3cfb_info[index]);
		unregister_framebuffer(&info[index].fb);
	}

	return 0;
}

static struct platform_driver s3cfb_driver = {
	.probe		= s3cfb_probe,
	.remove		= s3cfb_remove,
#ifdef CONFIG_HAS_WAKELOCK
#ifndef CONFIG_HAS_EARLYSUSPEND
	.suspend	= s3cfb_suspend,
	.resume		= s3cfb_resume,
#endif
#endif
        .driver		= {
		.name	= "s3c-lcd",
		.owner	= THIS_MODULE,
	},
};

int __devinit s3cfb_init(void)
{
	return platform_driver_register(&s3cfb_driver);
}
static void __exit s3cfb_cleanup(void)
{
	platform_driver_unregister(&s3cfb_driver);
}

module_init(s3cfb_init);
module_exit(s3cfb_cleanup);

MODULE_AUTHOR("Jinsung Yang");
MODULE_DESCRIPTION("S3C Framebuffer Driver");
MODULE_LICENSE("GPL");

