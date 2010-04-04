#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/string.h>
#include <linux/fb.h>
#include <linux/mm.h>
#include <linux/dma-mapping.h>
#include <linux/platform_device.h>

struct inode;
struct file;
struct vm_area_struct;
#include <linux/poll.h>
#include <media/s3c_g2d_driver.h>

#include "samsung/s3cfb.h"

#define BRAVO_VFB_NAME "bravo-vfb"
/* 'F''B' */
#define BRAVO_VFB_UPDATE 0x4642

struct bravo_vfb_info {
    struct fb_info fb;
    struct device *dev;

    __u32 vfb_map_size;
    u_char* vfb_cpu;
    dma_addr_t vfb_dma;
};

static unsigned int screen_offset = 0;
static int updated;

/* TODO rename once driver is up and running */
u32 S3C_FrameBuffer_for_eink = 0;
u32 S3C_FrameBuffer_for_eink_dma = 0;

extern void s1d13521fb_trigger(void);
extern void s1d13521if_dirty(int update);

extern int powering_down;
extern s3cfb_info_t *s3c_lcd_fb;
extern int s3cfb_pan_display(struct fb_var_screeninfo *var, struct fb_info *info);

/*#define TIME_ROTATION*/

#define EINK_OFFSET_Y 0
#define EINK_YRES 800
#define EINK_XRES 600

#define SRC_WIDTH 600

#ifdef CONFIG_FB_S3C_PICTURETUBES

#define SRC_HEIGHT 144
#define SRC_WORK_WIDTH 480
#define SRC_WORK_HEIGHT 144
#define DST_FULL_WIDTH 240
#define DST_FULL_HEIGHT 480
#define DST_START_X 96
#define DST_WORK_WIDTH 480
#define DST_WORK_HEIGHT 144
#define DST_CLIP_X 240
#define DST_CLIP_Y 480

#define LCD_YRES 144
#define LCD_FB_XRES 240
#define LCD_FB_YRES 480
#define LCD_BPP 2

#endif

#ifdef CONFIG_FB_S3C_AT272

#define SRC_HEIGHT 272
#define SRC_WORK_WIDTH 480
#define SRC_WORK_HEIGHT 272
#define DST_FULL_WIDTH 480
#define DST_FULL_HEIGHT 272
#define DST_START_X 0
#define DST_WORK_WIDTH 480
#define DST_WORK_HEIGHT 272
#define DST_CLIP_X 480
#define DST_CLIP_Y 272

#define LCD_YRES 272
#define LCD_FB_XRES 480
#define LCD_FB_YRES 272
#define LCD_BPP 2

#endif

static int __init bravo_vfb_map_video_memory(struct bravo_vfb_info *fbi)
{
    fbi->vfb_map_size = PAGE_ALIGN(fbi->fb.fix.smem_len);
    fbi->vfb_cpu = dma_alloc_writecombine(fbi->dev, fbi->vfb_map_size, &fbi->vfb_dma, GFP_KERNEL);
    fbi->vfb_map_size = fbi->fb.fix.smem_len;

    if (!fbi->vfb_cpu) {
        return -ENOMEM;
    }
    memset(fbi->vfb_cpu, 0x00, fbi->vfb_map_size);
    fbi->fb.screen_base = fbi->vfb_cpu;
    fbi->fb.fix.smem_start = fbi->vfb_dma;
    return 0;
}

static void bravo_vfb_unmap_video_memory(struct bravo_vfb_info *fbi)
{
    dma_free_writecombine(fbi->dev, fbi->vfb_map_size, fbi->vfb_cpu, fbi->vfb_dma);
}

static int bravo_vfb_check_var(struct fb_var_screeninfo *var, struct fb_info *info)
{
    if (var->bits_per_pixel != 16) {
        return -EINVAL;
    }

    var->red.offset = 11;
    var->red.length = 5;
    var->green.offset = 5;
    var->green.length = 6;
    var->blue.offset = 0;
    var->blue.length = 5;
    var->transp.offset = 0;
    var->transp.length = 0;
    return 0;
}

static int bravo_vfb_set_par(struct fb_info *info)
{
    struct fb_var_screeninfo *var = &info->var;
    struct bravo_vfb_info *fbi = (struct bravo_vfb_info *) info;

    if (info->var.bits_per_pixel != 16 ) {
        return -EINVAL;
    }

    s3cfb_activate_var(s3c_lcd_fb, var);
    fbi->fb.fix.visual = FB_VISUAL_TRUECOLOR;
    return 0;
}

static int bravo_vfb_blank(int blank_mode, struct fb_info *info)
{
    switch (blank_mode) {
    case VESA_NO_BLANKING:  /* lcd on, backlight on */
        break;

    case VESA_VSYNC_SUSPEND: /* lcd on, backlight off */
    case VESA_HSYNC_SUSPEND:
        break;

    case VESA_POWERDOWN: /* lcd and backlight off */
        break;

    default:
        return -EINVAL;
    }

    return 0;
}

static int bravo_vfb_rotate_bitblt_lcd(dma_addr_t src, dma_addr_t lcd_addr)
{
    int ret;
    s3c_g2d_params params;

    ret = s3c_g2d_is_ready();

    if (ret == -EAGAIN) {
        /* Rotation hardware isn't initialized yet */
        goto out;
    }

    params.src_base_addr = src;
    params.src_full_width = SRC_WIDTH;
    params.src_full_height = SRC_HEIGHT;
    params.src_start_x = 0;
    params.src_start_y = 0;
    params.src_work_width = SRC_WORK_WIDTH;
    params.src_work_height = SRC_WORK_HEIGHT;

    params.dst_base_addr = lcd_addr;
    params.dst_full_width = DST_FULL_WIDTH;
    params.dst_full_height = DST_FULL_HEIGHT;
    params.dst_start_x = DST_START_X;
    params.dst_start_y = 0;

    params.dst_work_width = DST_WORK_WIDTH;
    params.dst_work_height = DST_WORK_HEIGHT;

    params.cw_x1 = 0;
    params.cw_y1 = 0;
    params.cw_x2 = DST_CLIP_X;
    params.cw_y2 = DST_CLIP_Y;

    params.bpp_dst = G2D_RGB16;
    params.alpha_mode = false;
    params.color_key_mode = false;
    params.bpp_src = G2D_RGB16;

#ifdef CONFIG_FB_S3C_PICTURETUBES
    ret = s3c_g2d_rotate_internal(&params, ROT_90);
#else
    ret = s3c_g2d_rotate_internal(&params, ROT_0);
#endif
out:
    return ret;
}

static int bravo_vfb_pan(struct fb_var_screeninfo *var, struct fb_info *info)
{
    int ret;
    struct fb_var_screeninfo lcd_var;
    dma_addr_t lcd_dma;
    struct bravo_vfb_info *fbi = (struct bravo_vfb_info *) info;

    if (var->xoffset != 0)
        return -EINVAL;

    if (var->yoffset + info->var.yres > info->var.yres_virtual)
        return -EINVAL;

    fbi->fb.var.xoffset = var->xoffset;
    fbi->fb.var.yoffset = var->yoffset;

    S3C_FrameBuffer_for_eink = (u32) fbi->fb.screen_base + (fbi->fb.fix.line_length * (fbi->fb.var.yoffset + EINK_OFFSET_Y));
    S3C_FrameBuffer_for_eink_dma = fbi->vfb_dma + (fbi->fb.fix.line_length * (fbi->fb.var.yoffset + EINK_OFFSET_Y));

    lcd_dma = s3c_lcd_fb->screen_dma_f1;
    lcd_var.xoffset = 0;
    lcd_var.yoffset = 0;

    if (var->yoffset) {
        lcd_dma += LCD_FB_YRES * LCD_FB_XRES * LCD_BPP;
        lcd_var.yoffset = LCD_FB_YRES;
    }

    if (!powering_down) {
        s1d13521fb_trigger();
        s1d13521if_dirty(0);
    }

#ifdef TIME_ROTATION
    u64 trot = ktime_to_us(ktime_get());
#endif
    if (s3c_g2d_is_ready() == -EBUSY) {
        s3c_g2d_wait_until_ready();
    }

    ret = bravo_vfb_rotate_bitblt_lcd(S3C_FrameBuffer_for_eink_dma + ((EINK_YRES - (screen_offset * LCD_YRES)) * fbi->fb.fix.line_length), lcd_dma);

#ifdef TIME_ROTATION
    s3c_g2d_wait_until_ready();
    printk("Rotation took %dms\n", (int) (ktime_to_us(ktime_get()) - trot) / 1000);
#endif
    s3cfb_pan_display(&lcd_var, (struct fb_info *) s3c_lcd_fb);

    updated = 1;
    return ret;
}

void bravo_vfb_set_offset(unsigned int offset)
{
    dma_addr_t lcd_dma;
    struct fb_var_screeninfo lcd_var;
    int mem_offset;
    screen_offset = offset;

    lcd_var.yoffset = s3c_lcd_fb->fb.var.yoffset;
    lcd_var.xoffset = s3c_lcd_fb->fb.var.xoffset;

    mem_offset = EINK_YRES - (screen_offset * LCD_YRES);

    if (mem_offset < 0) {
        mem_offset = 0;
    }

    lcd_dma = s3c_lcd_fb->screen_dma_f1;

    if (s3c_lcd_fb->fb.var.yoffset) {
        lcd_dma += LCD_FB_YRES * LCD_FB_XRES * LCD_BPP;
    }

    if (updated) {
        if (s3c_g2d_is_ready() < 0) {
            printk(KERN_ERR "Rotator busy, not adjusting offset.\n");
        } else {
            bravo_vfb_rotate_bitblt_lcd(S3C_FrameBuffer_for_eink_dma + (mem_offset * EINK_XRES * LCD_BPP), lcd_dma);
            s3c_g2d_wait_until_ready();
            s3cfb_pan_display(&lcd_var, (struct fb_info *) s3c_lcd_fb);
        }
    }
}

static int bravo_vfb_setcolreg(unsigned int regno, unsigned int red, unsigned int green, unsigned int blue, unsigned int transp, struct fb_info *info)
{
    return 0;
}

static int bravo_vfb_ioctl(struct fb_info *info, unsigned int cmd, unsigned long arg)
{
    struct fb_var_screeninfo var;
    struct bravo_vfb_info *fbi = (struct bravo_vfb_info *) info;

    /* Once we get copybit optimized when can do clip to the LCD for fast interactivity */
    switch(cmd) {
    case BRAVO_VFB_UPDATE:
        var.xoffset = fbi->fb.var.xoffset;
        var.yoffset = fbi->fb.var.yoffset;
        return bravo_vfb_pan(&var, info);
    default:
        break;
    }

    return -EINVAL;
}

static struct fb_ops bravo_vfb_ops = {
    .owner = THIS_MODULE,
    .fb_check_var = bravo_vfb_check_var,
    .fb_set_par = bravo_vfb_set_par,
    .fb_blank = bravo_vfb_blank,
    .fb_pan_display = bravo_vfb_pan,
    .fb_setcolreg = bravo_vfb_setcolreg,
    .fb_fillrect = cfb_fillrect,
    .fb_copyarea = cfb_copyarea,
    .fb_imageblit = cfb_imageblit,
    .fb_ioctl = bravo_vfb_ioctl,
};

static int __init bravo_vfb_probe(struct platform_device *pdev)
{
    struct fb_info *fbi;
    struct bravo_vfb_info *info;
    int ret;

    fbi = framebuffer_alloc(sizeof(struct bravo_vfb_info), &pdev->dev);

    if (!fbi) {
        return -ENOMEM;
    }

    platform_set_drvdata(pdev, fbi);
    info = fbi->par;
    info->dev = &pdev->dev;

    strcpy(info->fb.fix.id, BRAVO_VFB_NAME);

    info->fb.fix.type = FB_TYPE_PACKED_PIXELS;
    info->fb.fix.type_aux = 0;
    info->fb.fix.xpanstep = 0;
    info->fb.fix.ypanstep = 1;
    info->fb.fix.ywrapstep = 0;
    info->fb.fix.accel = FB_ACCEL_NONE;

    info->fb.fbops = &bravo_vfb_ops;
    info->fb.flags = FBINFO_FLAG_DEFAULT;
    info->fb.pseudo_palette = NULL;

    info->fb.var.nonstd = 0;
    info->fb.var.activate = FB_ACTIVATE_NOW;
    info->fb.var.accel_flags = 0;
    info->fb.var.vmode = FB_VMODE_NONINTERLACED;

    info->fb.var.xoffset = 0;
    info->fb.var.yoffset = 0;

    /* E Ink height is 122.4mm
       LCD (picturetubes) is 25.92mm */
    info->fb.var.height = 148;
    /* E Ink width is 90.6mm
       LCD (picturetubes) is 86.4mm*/
    info->fb.var.width = 91;

    info->fb.var.xres = EINK_XRES;
    info->fb.var.yres = EINK_YRES + LCD_YRES;

    info->fb.var.xres_virtual = EINK_XRES;
    info->fb.var.yres_virtual = (EINK_YRES + LCD_YRES) * 2; /* Double buffered */

    info->fb.var.bits_per_pixel = 16; /* RGB 565 */
    info->fb.var.pixclock = EINK_XRES * (EINK_YRES + LCD_YRES) * 60; /* ~33MHz of fake pixel clock */
    info->fb.var.hsync_len = 0;
    info->fb.var.left_margin = 0;
    info->fb.var.right_margin = 0;
    info->fb.var.vsync_len = 0;
    info->fb.var.upper_margin = 0;
    info->fb.var.lower_margin = 0;
    info->fb.var.sync = 0;
    info->fb.var.grayscale = 0;

    bravo_vfb_check_var(&(info->fb.var), &(info->fb));

    info->fb.fix.smem_len = info->fb.var.xres_virtual * info->fb.var.yres_virtual * 2; /* bytes per pixel and double buffered */
    info->fb.fix.line_length = info->fb.var.xres * 2; /* bytes per pixel */

    ret = bravo_vfb_map_video_memory(info);

    if (ret) {
        dev_err(&pdev->dev, "Failed to map video memory: %d\n", ret);
        goto dealloc_fb;
    }

    ret = register_framebuffer(&(info->fb));

    if (ret < 0) {
        dev_err(&pdev->dev, "Failed to register frame buffer device: %d\n", ret);
        goto free_video_memory;
    }

    printk(KERN_INFO "fb%d: %s frame buffer device\n", info->fb.node, info->fb.fix.id);
    return 0;

free_video_memory:
    bravo_vfb_unmap_video_memory(info);

dealloc_fb:
    framebuffer_release(fbi);
    return ret;
}

static int bravo_vfb_remove(struct platform_device *pdev)
{
    struct fb_info *fbinfo = platform_get_drvdata(pdev);
    struct bravo_vfb_info *info = fbinfo->par;

    /* stop lcd */
    /* stop e-ink */

    bravo_vfb_unmap_video_memory(info);
    unregister_framebuffer(&info->fb);

    return 0;
}

static struct platform_driver bravo_vfb_driver = {
    .probe = bravo_vfb_probe,
    .remove = bravo_vfb_remove,
#ifdef CONFIG_PM
    .suspend = NULL,
    .resume = NULL,
#endif
#ifdef CONFIG_HAS_EARLYSUSPEND
    .suspend_late = NULL,
    .resume_early = NULL,
#endif
    .driver = {
        .name = "bravo-vfb",
        .owner = THIS_MODULE,
    },
};

int __init bravo_vfb_init(void)
{
    printk("Enabling Bravo virtual frame buffer\n");
    return platform_driver_register(&bravo_vfb_driver);
}

void __exit bravo_vfb_exit(void)
{
    platform_driver_unregister(&bravo_vfb_driver);
}

subsys_initcall(bravo_vfb_init);
__exitcall(bravo_vfb_exit);

MODULE_AUTHOR("David Bolcsfoldi");
MODULE_DESCRIPTION("Bravo virtual frame buffer driver");
MODULE_LICENSE("GPL");

