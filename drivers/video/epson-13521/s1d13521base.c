//-----------------------------------------------------------------------------
//
// linux/drivers/video/epson/s1d13521fb.c -- frame buffer driver for Epson
// S1D13521 series of LCD controllers.
//
// Copyright(c) Seiko Epson Corporation 2000-2009.
// All rights reserved.
//
// Modifications 2009 : Intrinsyc Software, Inc on behalf of Barnes and Noble
// Portions of this code copyright (c) 2009 Barnes and Noble, Inc
//
// This file is subject to the terms and conditions of the GNU General Public
// License. See the file COPYING in the main directory of this archive for
// more details.
//
//----------------------------------------------------------------------------

#define S1D13521FB_VERSION              "S1D13521FB: $Revision: 1 $"
#include <linux/version.h>
#include <linux/vmalloc.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/kernel.h>
#include <linux/types.h>
#include <linux/errno.h>
#include <linux/string.h>
#include <linux/mm.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/fb.h>
#include <linux/kthread.h>
#include <linux/freezer.h>

#ifdef CONFIG_FB_EPSON_PCI
    #include <linux/pci.h>
#endif

#include <linux/proc_fs.h>
#include <linux/interrupt.h>
#include <asm/uaccess.h>
#include <linux/timer.h>

#ifndef CONFIG_HAS_WAKELOCK
#ifdef CONFIG_ANDROID_POWER
#include <linux/android_power.h>
#endif // CONFIG_ANDROID_POWER
#else  // CONFIG_HAS_WAKELOCK
#include <linux/wakelock.h>
#include <linux/earlysuspend.h>
#endif // CONFIG_HAS_WAKELOCK

#include "s1d13521fb.h"
// Cleanup
#define command(cmd)  s1d13521if_command(cmd)
#define data_get()    s1d13521if_data_get()
#define data(val)     s1d13521if_data(val)

#ifdef CONFIG_MACH_BRAVO_RDU
#include "logo_rdu.h"
#endif

#ifdef CONFIG_FB_EPSON_GPIO_SMDK6410
    #ifdef CONFIG_FB_EPSON_PCI
    #undef CONFIG_FP_EPSON_PCI
    #endif
#endif

#ifdef VIDEO_REFRESH_PERIOD
    #undef VIDEO_REFRESH_PERIOD
#endif

#ifdef CONFIG_HAS_EARLYSUSPEND
#define SCREEN_SAVER "/data/screensaver/screensaver.bin"
#define SCREEN_SAVER_DEFAULT_PIXELS (800*600)
#define SCREEN_SAVER_DEFAULT "/data/screensaver/default_screensaver.bin"
#endif

#define S1D13521_FBID           "S1D13521"
#define S1D13521_DEVICENAME     "s1d13521fb"

// This is the refresh period for updating the display for RAM based LCDs
// and/or indirect interfaces.  It is set to (1 second / X).  This value can
// modified by the end-user.  Be aware that decreasing the refresh period
// increases CPU usage as well.
// There are HZ (typically 100) jiffies per second
#if CONFIG_FB_EPSON_VIRTUAL_FRAMEBUFFER_FREQ != 0
    #define VIDEO_REFRESH_PERIOD   (HZ/CONFIG_FB_EPSON_VIRTUAL_FRAMEBUFFER_FREQ)*1
#endif

// Force screen update even if the dirty flag is not set
#ifdef CONFIG_FB_EPSON_IDLE_UPDATE
    #define VFB_IDLE_COUNT  120  // Number of frames between force refresh
    static int gIdleCount = 0;
#endif

// This section only reports the configuration settings
#ifdef CONFIG_FB_EPSON_SHOW_SETTINGS
        #warning ########################################################################
        #warning Epson S1D13521 Frame Buffer Configuration:

        #ifdef CONFIG_FB_EPSON_PCI
                #warning Using PCI interface
        #else
                #warning Not using PCI interface
        #endif

        #ifdef CONFIG_FB_EPSON_GPIO_SMDK6410
                #warning SMDK6410/Broadsheet port using GPIO pins
        #else
                #warning Not a Gumstix/Broadsheet port
        #endif

        #ifdef CONFIG_FB_EPSON_PROC
                #warning Adding /proc functions
        #else
                #warning No /proc functions
        #endif

        #ifdef CONFIG_FB_EPSON_DEBUG_PRINTK
                #warning Enable debugging printk() calls
        #else
                #warning No debugging printk() calls
        #endif

        #ifdef CONFIG_FB_EPSON_BLACK_AND_WHITE
                #warning Virtual Framebuffer Black And White
        #else
                #warning Virtual Framebuffer 16 Shades of Gray
        #endif

        #ifdef VIDEO_REFRESH_PERIOD
                #warning Timer video refresh ENABLED.
        #else
                #warning Timer video refresh DISABLED.
        #endif

        #ifdef CONFIG_FB_EPSON_HRDY_OK
                #warning Assuming HRDY signal present
        #else
                #warning Assuming HRDY signal NOT present.
        #endif
        #warning ########################################################################
#endif

//-----------------------------------------------------------------------------
//
// Local Definitions
//
//---------------------------------------------------------------------------

//-----------------------------------------------------------------------------
//
// Function Prototypes
//
//-----------------------------------------------------------------------------
static int  s1d13521fb_set_par(struct fb_info *info);
static void s1d13521fb_fillrect(struct fb_info *p, const struct fb_fillrect *rect);
static void s1d13521fb_copyarea(struct fb_info *p, const struct fb_copyarea *area);
static void s1d13521fb_imageblit(struct fb_info *p, const struct fb_image *image);
static int  s1d13521fb_check_var(struct fb_var_screeninfo *var, struct fb_info *info);
static int  s1d13521fb_setcolreg(unsigned regno, unsigned red, unsigned green, unsigned blue, unsigned transp, struct fb_info *info);
static int  s1d13521fb_ioctl(struct fb_info *info, unsigned int cmd, unsigned long arg);
static int  s1d13521fb_blank(int blank_mode, struct fb_info *info);
static int  s1d13521fb_set_virtual_framebuffer(void);

static int  s1d13521fb_mmap(struct fb_info *info, struct vm_area_struct *vma);
static int s1d13521fb_sync(struct fb_info *info);
static int s1d13521fb_release(struct fb_info *info, int user);
static void s1d13521_image_display(struct s1d13521fb_eink_data *eink, char *filename);

#ifdef CONFIG_HAS_EARLYSUSPEND
void s1d13521_early_suspend(struct early_suspend *h);
void s1d13521_late_resume(struct early_suspend *h);
#endif


extern void __iomem *pEink;  //SROM Bank3 memory range(XM0CSn3)
extern void set_gpio_val(int pin, int val);

#define FRAME_BUF_SIZE 600 * 800

//-----------------------------------------------------------------------------
//
// Globals
//
//-----------------------------------------------------------------------------

char *s1d13521fb_version  = S1D13521FB_VERSION;
int gDisplayChange = 0;          // Display update needed flag
u32 s1d13521fb_init_done = 0;    // Hardware initialization done?

static struct workqueue_struct *eink_wq;

static struct fb_ops s1d13521fb_ops =
{
        .owner          = THIS_MODULE,
        .fb_set_par     = s1d13521fb_set_par,
        .fb_check_var   = s1d13521fb_check_var,
        .fb_setcolreg   = s1d13521fb_setcolreg,
        .fb_fillrect    = s1d13521fb_fillrect,
        .fb_copyarea    = s1d13521fb_copyarea,
        .fb_imageblit   = s1d13521fb_imageblit,
        .fb_mmap        = s1d13521fb_mmap,
        .fb_ioctl       = s1d13521fb_ioctl,
        .fb_sync    = s1d13521fb_sync,
        .fb_release = s1d13521fb_release,
};

struct fb_fix_screeninfo s1d13521fb_fix =
{
        .id             = "S1D13521",
        .type           = FB_TYPE_PACKED_PIXELS,
        .type_aux       = 0,
        .xpanstep       = 0,
        .ypanstep       = 0,
        .ywrapstep      = 0,
        .smem_len       = (S1D_DISPLAY_WIDTH*S1D_DISPLAY_HEIGHT*S1D_DISPLAY_BPP/sizeof(u8)) +
                  (S1D_DISPLAY_WIDTH*S1D_DISPLAY_HEIGHT*8/sizeof(u8)) +
                  (S1D_DISPLAY_WIDTH*S1D_DISPLAY_HEIGHT*8/sizeof(u8)),
        .line_length    = S1D_DISPLAY_WIDTH*S1D_DISPLAY_BPP/8,
        .accel          = FB_ACCEL_NONE,
};

struct fb_info   s1d13521_fb;
FB_INFO_S1D13521 s1d13521fb_info;
void *s1d13521fb_current = NULL;
void *s1d13521fb_previous = NULL;
extern int s1d13521fb_process_collisions(void);

extern u32 S3C_FrameBuffer_for_eink;
struct task_struct *s1d13521_task;

static wait_queue_head_t trg_waitq;
static wait_queue_head_t frend_waitq;
wait_queue_head_t lut_waitq;
struct s1d13521fb_eink_data *s1d13521fb_data = NULL;

//-----------------------------------------------------------------------------
// Parse user specified options (`video=s1d13521:')
// Example:
// video=s1d13521:noaccel
//-----------------------------------------------------------------------------
int __init s1d13521fb_setup(char *options, int *ints)
{
        return 0;
}

//-----------------------------------------------------------------------------
//
// Fill in the 'var' and 'fix' structure.
//
//-----------------------------------------------------------------------------
static int s1d13521fb_check_var(struct fb_var_screeninfo *var, struct fb_info *info)
{
        var->xres               = S1D_DISPLAY_WIDTH;
        var->yres               = S1D_DISPLAY_HEIGHT;
        var->xres_virtual       = var->xres;
        var->yres_virtual       = var->yres;
        var->xoffset            = var->yoffset = 0;
        var->bits_per_pixel     = S1D_DISPLAY_BPP;
        var->grayscale          = 1;
        var->nonstd             = 0;                    /* != 0 Non standard pixel format */
        var->activate           = FB_ACTIVATE_NOW;      /* see FB_ACTIVATE_*             */
        var->height             = -1;                   /* height of picture in mm       */
        var->width              = -1;                   /* width of picture in mm        */
        var->accel_flags        = 0;                    /* acceleration flags (hints     */
        var->pixclock           = S1D_DISPLAY_PCLK;
        var->right_margin       = 0;
        var->lower_margin       = 0;
        var->hsync_len          = 0;
        var->vsync_len          = 0;
        var->left_margin        = 0;
        var->upper_margin       = 0;
        var->sync               = 0;
        var->vmode              = FB_VMODE_NONINTERLACED;
        var->red.msb_right      = var->green.msb_right = var->blue.msb_right = 0;
        var->transp.offset      = var->transp.length = var->transp.msb_right = 0;

        s1d13521fb_fix.visual = FB_VISUAL_TRUECOLOR;

        switch (info->var.bits_per_pixel)
        {
                case 1:
                case 2:
                case 4:
                case 5:
                case 8:
                        var->red.offset  = var->green.offset = var->blue.offset = 0;
                        var->red.length  = var->green.length = var->blue.length = S1D_DISPLAY_BPP;
                        break;

        case 16:
            /* 16 bpp, 565 format */
            var->red.offset     = 11;
            var->green.offset   = 5;
            var->blue.offset    = 0;
            var->red.length     = 5;
            var->green.length   = 6;
            var->blue.length    = 5;
            break;

                default:
                        printk(KERN_WARNING "%dbpp is not supported.\n",
                                        info->var.bits_per_pixel);
                        return -EINVAL;
        }

        return 0;
}


//----------------------------------------------------------------------------
// Set a single color register. The values supplied have a 16 bit
// magnitude.
// Return != 0 for invalid regno.
//
// We get called even if we specified that we don't have a programmable palette
// or in direct/true color modes!
//----------------------------------------------------------------------------
static int s1d13521fb_setcolreg(unsigned regno, unsigned red, unsigned green,
                                unsigned blue, unsigned transp, struct fb_info *info)
{
        // Make the first 16 LUT entries available to the console
        if (info->var.bits_per_pixel == 8 && s1d13521fb_fix.visual == FB_VISUAL_TRUECOLOR)
        {
                if (regno < 16)
                {
                        // G= 30%R + 59%G + 11%B
                        unsigned gray = (red*30 + green*59 + blue*11)/100;
                        gray = (gray>>8) & 0xFF;

#ifdef CONFIG_FB_EPSON_BLACK_AND_WHITE
                        if (gray != 0)
                                gray = 0xFF;
#endif

                        // invert: black on white
                        gray = 0xFF-gray;
                        ((u32*)info->pseudo_palette)[regno] = gray;

                        dbg_info("%s(): regno=%02Xh red=%04Xh green=%04Xh blue=%04Xh transp=%04Xh ->gray=%02Xh\n",
                                __FUNCTION__, regno, red, green, blue, transp,gray);
                }
        }
        else
                return 1;

        return 0;
}

//-----------------------------------------------------------------------------
//
// Set the hardware.
//
//-----------------------------------------------------------------------------
static int s1d13521fb_set_par(struct fb_info *info)
{
        dbg_info("%s():\n",__FUNCTION__);

        info->fix = s1d13521fb_fix;
        //info->fix.mmio_start = (unsigned long)virt_to_phys((void*)s1d13521fb_info.RegAddr);
        //info->fix.mmio_len   = s1d13521fb_info.RegAddrMappedSize; //S1D_MMAP_PHYSICAL_REG_SIZE;
        info->fix.mmio_start = 0;  //[ICS] we don't support EINK HWI/F mapping
        info->fix.mmio_len   = 0; //[ICS] we don't support EINK HWI/F mapping
        info->fix.smem_start = virt_to_phys((void*)s1d13521fb_info.VirtualFramebufferAddr);
        info->screen_base    = (unsigned char*)s1d13521fb_info.VirtualFramebufferAddr;
        info->screen_size    = (S1D_DISPLAY_WIDTH*S1D_DISPLAY_HEIGHT*S1D_DISPLAY_BPP)/8;
        return 0;
}

//----------------------------------------------------------------------------
//
// PRIVATE FUNCTION:
// Remaps virtual framebuffer from virtual memory space to a physical space.
//
//      If no virtual framebuffer is used, the default fb_mmap routine should
//      be fine, unless there is a concern over its use of
//              io_remap_pfn_range versus remap_pfn_range
//
//----------------------------------------------------------------------------
static int s1d13521fb_sync(struct fb_info *info)
{
    //printk(KERN_NOTICE "s1d13521fb_sync: gDisplayChange = 1\n");
    //gDisplayChange = 1;
    return 0;
}

static int s1d13521fb_release(struct fb_info *info, int user)
{
    printk(KERN_NOTICE "s1d13521fb_release: gDisplayChange = 1\n");
    s1d13521fb_trigger();
    return 0;
}

static int s1d13521fb_mmap(struct fb_info *info, struct vm_area_struct *vma)
{
        unsigned long off;
        unsigned long start;
        u32 len;

        dbg_info("%s(): \n", __FUNCTION__);

        off = vma->vm_pgoff << PAGE_SHIFT;

        // frame buffer memory
        start = info->fix.smem_start;
        len = PAGE_ALIGN((start & ~PAGE_MASK) + info->fix.smem_len);

        if (off >= len)
        {
#ifdef CONFIG_MACH_SMDK6410
            //[ICS] We don't support EINK<->S3C6410 HWI/F mapping
            printk(KERN_NOTICE "s1d13521fb driver doesn't support mapping of area outside of framebuffer\n");
            return -1;
#else
            // memory mapped io
            off -= len;

            if (info->var.accel_flags)
                    return -EINVAL;

            start = info->fix.mmio_start;
            len = PAGE_ALIGN((start & ~PAGE_MASK) + info->fix.mmio_len);
#endif
        }

        start &= PAGE_MASK;

        if ((vma->vm_end - vma->vm_start + off) > len)
                return -EINVAL;

        off += start;
        vma->vm_pgoff = off >> PAGE_SHIFT;

        // This is an IO map - tell maydump to skip this VMA
        vma->vm_flags |= VM_RESERVED;  //VM_RESERVED is OK here

        //[ICS] We only support _one_ contiguous memory region for EINK framebuffer
        if (remap_pfn_range(vma, vma->vm_start, off >> PAGE_SHIFT, vma->vm_end - vma->vm_start, vma->vm_page_prot))
                return -EAGAIN;

        printk(KERN_NOTICE "s1d13521fb_mmap: gDisplayChange = 1\n");
        s1d13521fb_trigger();

        return 0;
}

//----------------------------------------------------------------------------
// PRIVATE FUNCTION:
// Allocates virtual framebuffer.
//----------------------------------------------------------------------------
static int s1d13521fb_set_virtual_framebuffer(void)
{
    u32 order = 0;
    u32 size = VFB_SIZE_BYTES * 3;
    u32 addr;
    //u32 size_pfn_count = (size>>12);

    while (size > (PAGE_SIZE << order))
        order++;

    s1d13521fb_info.page_order = order;

    s1d13521fb_info.VirtualFramebufferAddr = __get_free_pages(GFP_KERNEL, order);

    if (s1d13521fb_info.VirtualFramebufferAddr == 0)
    {
        printk(KERN_WARNING "%s(): Could not allocate memory for virtual display buffer.\n", __FUNCTION__);
        return 1;
    }

    for (addr = s1d13521fb_info.VirtualFramebufferAddr; addr < (s1d13521fb_info.VirtualFramebufferAddr+size); addr += PAGE_SIZE)
        SetPageReserved(virt_to_page(addr));

    dbg_info("%s(): VirtualFramebufferAddr=%08X\n", __FUNCTION__,s1d13521fb_info.VirtualFramebufferAddr);

    s1d13521fb_current  = (void*) s1d13521fb_info.VirtualFramebufferAddr;
    s1d13521fb_previous = (void*) s1d13521fb_info.VirtualFramebufferAddr + PREV_FRAMEBUFFER;

    return 0;
}

//----------------------------------------------------------------------------
// PRIVATE FUNCTION:
// One time display initialization.
//----------------------------------------------------------------------------

void s1d13521fb_init_display(void)
{
    //bs60_flash
    printk(KERN_NOTICE  "++[%s]\n", __FUNCTION__ );

    bs_cmd_wait_for_bit( 0x338, 0, 0 );
    bs_cmd_wait_for_bit( 0x338, 3, 0 );

    bs_cmd_upd_full( 0, 0, 0 );
    bs_cmd_wait_for_bit( 0x338, 0, 0 );
    bs_cmd_wait_for_bit( 0x338, 3, 0 );

    bs_cmd_wr_value_fb( 0xFF );
    bs_cmd_upd_init();
    bs_cmd_wait_dspe_trg();
    printk(KERN_NOTICE  "--[%s]\n", __FUNCTION__ );
}

#if 0
void s1d13521fb_test_area(void)
{
    int i;
    unsigned mode = WF_MODE_GU;
    unsigned cmd = UPD_PART;

    printk(KERN_NOTICE  "[%s] ... displaying BARS\n", __FUNCTION__ );
    for (i = 0; i < 8; i++)
    {
        memset(s1d13521fb_current + (i*100*600), ((i&1) ? 0x00 : 0xff), 600*100);
        s1d13521fb_do_area(cmd, mode,  0, i*100, 600, 100);
    }
}
#endif

#define BAR_SIZE (800/16)

void s1d13521fb_gray_bars(unsigned cmd, unsigned mode)
{
    int i,color = 0x00;

    printk(KERN_NOTICE  "[%s] ... displaying Gray BARS\n", __FUNCTION__ );
    for (i = 0; i < 16; i++)
    {
        memset(s1d13521fb_current + (i*BAR_SIZE*600), (color | (color >> 4)), 600*BAR_SIZE);
        s1d13521fb_do_area(cmd, mode,  0, i*BAR_SIZE, 600, BAR_SIZE);
        color += 16;
    }
}

void s1d13521fb_black( void )
{
    printk(KERN_NOTICE  "[%s] ... displaying black\n", __FUNCTION__ );
    bs_cmd_wait_for_bit( 0x338, 0, 0 );
    bs_cmd_wait_for_bit( 0x338, 3, 0 );
    bs_cmd_wr_value_fb( 0x00 );
    bs_cmd_upd_full( 3, 0, 0 );
    bs_cmd_wait_for_bit( 0x338, 0, 0 );
    bs_cmd_wait_for_bit( 0x338, 3, 0 );
}

void s1d13521fb_white( void )
{
    printk(KERN_NOTICE  "[%s] ... displaying white\n", __FUNCTION__ );
    bs_cmd_wait_for_bit( 0x338, 0, 0 );
    bs_cmd_wait_for_bit( 0x338, 3, 0 );
    bs_cmd_wr_value_fb( 0xFF );
    bs_cmd_upd_full( 3, 0, 0 );
    bs_cmd_wait_for_bit( 0x338, 0, 0 );
    bs_cmd_wait_for_bit( 0x338, 4, 0 );
}

int s1d13521fb_init_hw(void)
{
    int wfm_fvsn = 0;

    // Initialize the chip to use Indirect Interface
    if (s1d13521if_InterfaceInit(&s1d13521fb_info) != 0)
    {
        printk(KERN_WARNING "s1d13521fb_init: InterfaceInit error\n");
        return -EINVAL;
    }

    //bs_update_wfm(wfm_data, wfm_size);

    //-------------------------------------------------------------------------
    // Set the controller registers and initialize the display
    //-------------------------------------------------------------------------
    init_pll();

    s1d13521fb_InitRegisters();

    //wfm_fvsn = bs_cmd_print_wfm_version();
    wfm_fvsn = s1d13521fb_print_wfm_info();
    //s1d13521fb_init_display();
    bs_cmd_set_lut_auto_sel_mode(0);
    bs_cmd_set_wfm_auto_sel_mode(1);
    bs_cmd_set_rotmode(3);
#if defined(CONFIG_LOGO)
#ifdef CONFIG_MACH_BRAVO_RDU
    memcpy(s1d13521fb_current, rdu_logo_data, rdu_logo_size);
#else
    // Display while screen
    memset(s1d13521fb_current, 0xff, FRAME_BUF_SIZE);
#endif
    bs_cmd_ld_img(DFMT);
    bs_cmd_wr_data_fb();
    bs_cmd_ld_img_end();
    bs_cmd_upd_full( 3, 0, 0 );
    bs_cmd_wait_for_bit( 0x338, 0, 0 );
    bs_cmd_wait_for_bit( 0x338, 3, 0 );
#else
    s1d13521fb_gray_bars(UPD_PART, WF_MODE_GU);
#endif
    printk(KERN_NOTICE "Completion of s1d13521_init_hw\n");
    return 0;
}

//----------------------------------------------------------------------------
// PRIVATE FUNCTION:
// Set registers to initial values
//----------------------------------------------------------------------------
void s1d13521fb_print_disp_timings( void )
{
    int vsize, vsync, vblen, velen, hsize, hsync, hblen, helen;
    vsize = s1d13521if_ReadReg16( 0x300 );
    vsync = s1d13521if_ReadReg16( 0x302 );
    vblen = s1d13521if_ReadReg16( 0x304 );
    velen = ( vblen >> 8 ) & 0xFF;
    vblen &= 0xFF;
    hsize = s1d13521if_ReadReg16( 0x306 );
    hsync = s1d13521if_ReadReg16( 0x308 );
    hblen = s1d13521if_ReadReg16( 0x30A );
    helen = ( hblen >> 8 ) & 0xFF;
    hblen &= 0xFF;
    printk(KERN_NOTICE  "[%s] disp_timings: vsize=%d vsync=%d vblen=%d velen=%d\n", __FUNCTION__, vsize, vsync, vblen, velen );
    printk(KERN_NOTICE  "[%s] disp_timings: hsize=%d hsync=%d hblen=%d helen=%d\n", __FUNCTION__, hsize, hsync, hblen, helen );
}

int s1d13521fb_print_wfm_info( void )
{
    u16 a,b,c,d,e;
    int wfm_fvsn,wfm_luts,wfm_trc, wfm_mc, wfm_sb, wfm_eb, wfm_wmta;
    a = s1d13521if_ReadReg16( 0x354 );
    b = s1d13521if_ReadReg16( 0x356 );
    c = s1d13521if_ReadReg16( 0x358 );
    d = s1d13521if_ReadReg16( 0x35C );
    e = s1d13521if_ReadReg16( 0x35E );
    wfm_fvsn = a & 0xFF;
    wfm_luts = ( a >> 8 ) & 0xFF;
    wfm_trc = ( b >> 8 ) & 0xFF;
    wfm_mc = b & 0xFF;
    wfm_sb = ( c >> 8 ) & 0xFF;
    wfm_eb = c & 0xFF;
    wfm_wmta = d | ( e << 16 );

    printk(KERN_NOTICE  "[%s] wfm: fvsn=%d luts=%d mc=%d trc=%d eb=0x%02x sb=0x%02x wmta=%d\n",
      __FUNCTION__, wfm_fvsn, wfm_luts, wfm_mc, wfm_trc, wfm_eb, wfm_sb, wfm_wmta );

    return wfm_fvsn;
}

static  u16 InitCmdArray_1[] =
{
        INIT_DSPE_CFG,  5,      BS60_INIT_HSIZE,
                                BS60_INIT_VSIZE,
                                BS60_INIT_SDRV_CFG,
                                BS60_INIT_GDRV_CFG,
                                BS60_INIT_LUTIDXFMT,
        INIT_DSPE_TMG,  5,      BS60_INIT_FSLEN,
                                (BS60_INIT_FELEN<<8)|BS60_INIT_FBLEN,
                                BS60_INIT_LSLEN,
                                (BS60_INIT_LELEN<<8)|BS60_INIT_LBLEN,
                                BS60_INIT_PIXCLKDIV,
};
static  u16 InitCmdArray_2[] =
{
        RD_WFM_INFO,    2,      0x0886, 0,
};

void s1d13521fb_ExecCmdArray(u16 *cmd_array, unsigned int array_size)
{
    unsigned i, cmd,j,numparams;
    s1d13521_ioctl_cmd_params cmd_params;
    i = 0;
    while (i < array_size)
    {
        cmd = cmd_array[i++];
        numparams = cmd_array[i++];

        for (j = 0; j < numparams; j++)
            cmd_params.param[j] = cmd_array[i++];

        s1d13521if_cmd(cmd,&cmd_params,numparams);
    }
}

void s1d13521fb_InitRegisters(void)
{
    unsigned i;
    unsigned int regval;

    s1d13521if_WriteReg16( 0x006, 0x0000 );  //According to Demo
    s1d13521if_cmd(0x06, NULL, 0);  //bs_cmd_init_sys_run

    s1d13521if_WriteReg16( 0x0106, 0x0203); //0x01a8 );

    schedule_timeout(HZ);  //sleep 1 second
    printk(KERN_NOTICE "[%s] ... INIT_SYS_RUN --- done\n", __FUNCTION__ );

    regval = s1d13521if_ReadReg16( 0x000 );
    printk(KERN_NOTICE "[%s] ... REG[0x000] = 0x%04x\n", __FUNCTION__, regval);
    regval = s1d13521if_ReadReg16( 0x0002 );
    printk(KERN_NOTICE "[%s] ... REG[0x002] = 0x%04x\n", __FUNCTION__, regval );
    regval = s1d13521if_ReadReg16( 0x0004 );
    printk(KERN_NOTICE "[%s] ... REG[0x004] = 0x%04x\n", __FUNCTION__, regval );

    s1d13521if_WriteReg16( 0x0304, 0x0123 ); // frame begin/end length reg
    s1d13521if_WriteReg16( 0x030A, 0x4567 ); // line begin/end length reg
    regval = s1d13521if_ReadReg16( 0x304 );
    printk(KERN_NOTICE "s1d13521if_ReadReg16( 0x304 ); return (0x%x)\n", regval);
    if ( regval != 0x0123 )
        printk(KERN_NOTICE  "[%s] !!! ERROR: 0x304 read/write error: 0x%04x != 0x0123\n",__FUNCTION__, regval);

    regval = s1d13521if_ReadReg16( 0x030A );
    printk(KERN_NOTICE "s1d13521if_ReadReg16( 0x030A ); return (0x%x)\n", regval);
    if ( regval != 0x4567 )
        printk(KERN_NOTICE "[%s] !!! ERROR: 0x30A read/write error: 0x%04x != 0x4567\n", __FUNCTION__, regval );

    s1d13521if_WriteReg16( 0x0304, 0xFEDC );
    s1d13521if_WriteReg16( 0x030A, 0xBA98 );

    regval = s1d13521if_ReadReg16( 0x304 );
    printk(KERN_NOTICE "s1d13521if_ReadReg16( 0x304 ); return (0x%x)\n", regval);
    if ( regval != 0xFEDC )
        printk(KERN_NOTICE "[%s] !!! ERROR: 0x304 read/write error: 0x%04x != 0xFEDC\n",__FUNCTION__, regval );
    regval = s1d13521if_ReadReg16( 0x030A );
    printk(KERN_NOTICE "s1d13521if_ReadReg16( 0x030A ); return (0x%x)\n", regval);
    if ( regval != 0xBA98 )
        printk(KERN_NOTICE "[%s] !!! ERROR: 0x30A read/write error: 0x%04x != 0xBA98\n",__FUNCTION__, regval );
    printk(KERN_NOTICE "[%s] ... register read/write ok\n", __FUNCTION__ );
    i = 0;
#if 1
    s1d13521fb_ExecCmdArray(InitCmdArray_1, (sizeof(InitCmdArray_1)/sizeof(InitCmdArray_1[0])) );
    s1d13521fb_print_disp_timings();

    s1d13521fb_ExecCmdArray(InitCmdArray_2, (sizeof(InitCmdArray_2)/sizeof(InitCmdArray_2[0])) );
    s1d13521if_wait_for_bit( 0x338, 0, 0 );
    //s1d13521fb_print_wfm_info();

    printk(KERN_NOTICE  "[%s] ... display engine initialized with waveform\n", __FUNCTION__ );

    s1d13521if_cmd(UPD_GDRV_CLR, NULL, 0);
    s1d13521if_wait_for_bit( 0x338, 0, 0 );
    s1d13521if_wait_for_bit( 0x338, 3, 0 );
#else
    bs_cmd_init_dspe_cfg(BS60_INIT_HSIZE,
                         BS60_INIT_VSIZE,
                         BS60_INIT_SDRV_CFG,
                         BS60_INIT_GDRV_CFG,
                         BS60_INIT_LUTIDXFMT);

    bs_cmd_init_dspe_tmg(BS60_INIT_FSLEN,
                        (BS60_INIT_FELEN<<8)|BS60_INIT_FBLEN,
                        BS60_INIT_LSLEN,
                        (BS60_INIT_LELEN<<8)|BS60_INIT_LBLEN,
                        BS60_INIT_PIXCLKDIV);
    bs_cmd_print_disp_timings();
    //s1d13521fb_ExecCmdArray(InitCmdArray_1, (sizeof(InitCmdArray_1)/sizeof(InitCmdArray_1[0])) );
    //s1d13521fb_print_disp_timings();

    bs_cmd_rd_wfm_info(0x0886);
    bs_cmd_get_wfm_info();
    bs_cmd_print_wfm_info();

    //s1d13521fb_ExecCmdArray(InitCmdArray_2, (sizeof(InitCmdArray_2)/sizeof(InitCmdArray_2[0])) );
    //s1d13521if_wait_for_bit( 0x338, 0, 0 );
    //s1d13521fb_print_wfm_info();

    printk(KERN_NOTICE  "[%s] ... display engine initialized with waveform\n", __FUNCTION__ );

    bs_cmd_init_cmd_set(0, 0, 0);
    //s1d13521if_cmd(UPD_GDRV_CLR, NULL, 0);
    //s1d13521if_wait_for_bit( 0x338, 0, 0 );
    //s1d13521if_wait_for_bit( 0x338, 3, 0 );
#endif
    s1d13521if_WriteReg16( 0x01A, 4 ); // i2c clock divider
    s1d13521if_WriteReg16( 0x320, 0 ); // temp auto read on

    // Read controller setup
    DEBUGP_L1(2, "*** Dumping s1d13521if controller registers\n");
    DEBUGP_L1(2, "s1d13521if_ReadReg16(0x0A)=0x%04x\n", s1d13521if_ReadReg16(0x0A));
    DEBUGP_L1(2, "s1d13521if_ReadReg16(0x10)=0x%04x\n", s1d13521if_ReadReg16(0x10));
    DEBUGP_L1(2, "s1d13521if_ReadReg16(0x11)=0x%04x\n", s1d13521if_ReadReg16(0x11));
    DEBUGP_L1(2, "s1d13521if_ReadReg16(0x12)=0x%04x\n", s1d13521if_ReadReg16(0x12));
    DEBUGP_L1(2, "s1d13521if_ReadReg16(0x13)=0x%04x\n", s1d13521if_ReadReg16(0x13));
    DEBUGP_L1(2, "s1d13521if_ReadReg16(0x14)=0x%04x\n", s1d13521if_ReadReg16(0x14));
    DEBUGP_L1(2, "s1d13521if_ReadReg16(0x15)=0x%04x\n", s1d13521if_ReadReg16(0x15));
    DEBUGP_L1(2, "s1d13521if_ReadReg16(0x16)=0x%04x\n", s1d13521if_ReadReg16(0x16));
    DEBUGP_L1(2, "s1d13521if_ReadReg16(0x17)=0x%04x\n", s1d13521if_ReadReg16(0x17));
    DEBUGP_L1(2, "s1d13521if_ReadReg16(0x18)=0x%04x\n", s1d13521if_ReadReg16(0x18));

    s1d13521fb_init_done = 1;
}


void s1d13521fb_InitRegisters_Resume(void)
{
    unsigned int regval;

    s1d13521if_WriteReg16( 0x006, 0x0000 );
    s1d13521if_cmd(0x06, NULL, 0);  // init_sys_run

    s1d13521if_WriteReg16( 0x0106, 0x0203);

    regval = s1d13521if_ReadReg16( 0x0000 );
    regval = s1d13521if_ReadReg16( 0x0002 );
    regval = s1d13521if_ReadReg16( 0x0004 );

    s1d13521fb_ExecCmdArray(InitCmdArray_1, (sizeof(InitCmdArray_1)/sizeof(InitCmdArray_1[0])) );
    s1d13521fb_ExecCmdArray(InitCmdArray_2, (sizeof(InitCmdArray_2)/sizeof(InitCmdArray_2[0])) );
    s1d13521if_wait_for_bit( 0x338, 0, 0 );

    s1d13521if_cmd(UPD_GDRV_CLR, NULL, 0);
    s1d13521if_wait_for_bit( 0x338, 0, 0 );
    s1d13521if_wait_for_bit( 0x338, 3, 0 );

    s1d13521if_WriteReg16( 0x01A, 4 ); // i2c clock divider
    s1d13521if_WriteReg16( 0x320, 0 ); // temp auto read on
}

int s1d13521fb_resume_hw(void)
{
    s1d13521fb_InitRegisters_Resume();

    bs_cmd_set_lut_auto_sel_mode(0);
    bs_cmd_set_wfm_auto_sel_mode(1);
    bs_cmd_set_rotmode(3);

    // Restore the framebuffer
    bs_cmd_wait_for_bit( 0x338, 0, 0 );
    bs_cmd_wait_for_bit( 0x338, 3, 0 );
    bs_cmd_upd_full( 3, 0, 0 );
    bs_cmd_wait_for_bit( 0x338, 0, 0 );
    bs_cmd_wait_for_bit( 0x338, 3, 0 );

    return 0;
}

//---------------------------------------------------------------------------
//
//---------------------------------------------------------------------------
int s1d13521if_cmd(unsigned ioctlcmd,s1d13521_ioctl_cmd_params *params,int numparams)
{
    int i;
    unsigned cmd = ioctlcmd & 0xFF;

    lock_kernel();

    if (s1d13521if_WaitForHRDY() != 0)
    {
        unlock_kernel();
        return -1;
    }

    command(cmd);

    for (i = 0; i < numparams; i++)
        data(params->param[i]);

    unlock_kernel();
    return 0;
}

//----------------------------------------------------------------------------
int s1d13521fb_ioctl(struct fb_info *info, unsigned int cmd, unsigned long arg)
{
        void __user *argp = (void __user *)arg;
        struct s1d13521_ioctl_hwc ioctl_hwc;
        s1d13521_ioctl_cmd_params cmd_params;

        dbg_info("%s(): cmd=%04Xh\n", __FUNCTION__,cmd);

        switch (cmd)
        {
          case S1D13521_VBUF_REFRESH:
             s1d13521fb_do_refresh_display(UPD_FULL,WF_MODE_GC, NULL);
             break;

          // zero parameter commands:

          case S1D13521_RUN_SYS:
          case S1D13521_STBY:
          case S1D13521_SLP:
          case S1D13521_INIT_SYS_RUN:
          case S1D13521_INIT_SYS_STBY:
          case S1D13521_RD_SFM:
          case S1D13521_END_SFM:
          case S1D13521_BST_END_SDR:
          case S1D13521_LD_IMG_END:
          case S1D13521_LD_IMG_WAIT:
          case S1D13521_LD_IMG_DSPEADR:
          case S1D13521_WAIT_DSPE_TRG:
          case S1D13521_WAIT_DSPE_FREND:
          case S1D13521_WAIT_DSPE_LUTFREE:
          case S1D13521_UPD_INIT:
          case S1D13521_UPD_GDRV_CLR:
            s1d13521if_cmd(cmd,&cmd_params,0);
            break;

          // one parameter commands
          case S1D13521_INIT_ROTMODE:
          case S1D13521_WR_SFM:
          case S1D13521_LD_IMG:
          case S1D13521_WAIT_DSPE_MLUTFREE:
          case S1D13521_UPD_FULL:
          case S1D13521_UPD_PART:
            if (copy_from_user(&cmd_params, argp, 1*sizeof(u16)))
                return -EFAULT;

            s1d13521if_cmd(cmd,&cmd_params,1);
            break;

          // two parameter commandss
          case S1D13521_WR_REG:
          case S1D13521_LD_IMG_SETADR:
          case S1D13521_RD_WFM_INFO:
          case S1D13521_UPD_SET_IMGADR:
            if (copy_from_user(&cmd_params, argp, 2*sizeof(u16)))
                return -EFAULT;

            if (cmd == S1D13521_WR_REG && cmd_params.param[0] == 0x154)
                s1d13521if_cmd(cmd,&cmd_params,1);
            else
                s1d13521if_cmd(cmd,&cmd_params,2);

            break;

          // three parameter commands
          case S1D13521_INIT_CMD_SET:
          case S1D13521_INIT_PLL_STANDBY:
            if (copy_from_user(&cmd_params, argp, 3*sizeof(u16)))
                return -EFAULT;

            s1d13521if_cmd(cmd,&cmd_params,3);
            break;

          // four parameter commands
          case S1D13521_INIT_SDRAM:
          case S1D13521_BST_RD_SDR:
          case S1D13521_BST_WR_SDR:
            if (copy_from_user(&cmd_params, argp, 4*sizeof(u16)))
                return -EFAULT;

            s1d13521if_cmd(cmd,&cmd_params,4);
            break;

          // five parameter commands
          case S1D13521_INIT_DSPE_CFG:
          case S1D13521_INIT_DSPE_TMG:
          case S1D13521_LD_IMG_AREA:
          case S1D13521_UPD_FULL_AREA:
          case S1D13521_UPD_PART_AREA:
            if (copy_from_user(&cmd_params, argp, 5*sizeof(u16)))
                return -EFAULT;

            s1d13521if_cmd(cmd,&cmd_params,5);
            break;

          case S1D13521_RD_REG:
            if (copy_from_user(&cmd_params, argp, 2*sizeof(u16)))
                return -EFAULT;

            cmd_params.param[1] = s1d13521if_ReadReg16(cmd_params.param[1]);
            return copy_to_user(argp, &cmd_params, 2*sizeof(u16)) ? -EFAULT : 0;

          case S1D13521_REGREAD:
            if (copy_from_user(&ioctl_hwc, argp, sizeof(ioctl_hwc)))
                return -EFAULT;

            ioctl_hwc.value = s1d13521if_ReadReg16(ioctl_hwc.addr);
            return copy_to_user(argp, &ioctl_hwc, sizeof(ioctl_hwc)) ? -EFAULT : 0;

          case S1D13521_REGWRITE:
            if (copy_from_user(&ioctl_hwc, argp, sizeof(ioctl_hwc)))
                return -EFAULT;

            s1d13521if_WriteReg16(ioctl_hwc.addr, ioctl_hwc.value);
            return 0;

          case S1D13521_MEMBURSTWRITE:
            {
            u8 buffer[2048];
            unsigned user_buflen,copysize,copysize16;
            u16 *ptr16;
            u8* user_buffer;

            if (copy_from_user(&ioctl_hwc, argp, sizeof(ioctl_hwc)))
                return -EFAULT;

            // ioctl_hwc.value = number of bytes in the user buffer
            // ioctl_hwc.buffer = pointer to user buffer

            user_buflen = ioctl_hwc.value;
            user_buffer = (u8*) ioctl_hwc.buffer;

            while (user_buflen != 0)
            {
                copysize = user_buflen;

                if (user_buflen > sizeof(buffer))
                        copysize = sizeof(buffer);

                if (copy_from_user(buffer,user_buffer,copysize))
                        return -EFAULT;

                copysize16 = (copysize + 1)/2;
                ptr16 = (u16*) buffer;
                s1d13521if_BurstWrite16(ptr16,copysize16);
                user_buflen -= copysize;
                user_buffer += copysize;
            }

            return 0;
            }

          case S1D13521_MEMBURSTREAD:
            {
            u8 buffer[2048];
            unsigned user_buflen,copysize,copysize16;
            u16 *ptr16;
            u8* user_buffer;

            if (copy_from_user(&ioctl_hwc, argp, sizeof(ioctl_hwc)))
                return -EFAULT;

            // ioctl_hwc.value = size in bytes of the user buffer, number of bytes to copy
            // ioctl_hwc.buffer = pointer to user buffer

            user_buflen = ioctl_hwc.value;
            user_buffer = (u8*) ioctl_hwc.buffer;

            while (user_buflen != 0)
            {
                copysize = user_buflen;

                if (user_buflen > sizeof(buffer))
                        copysize = sizeof(buffer);

                copysize16 = (copysize + 1)/2;
                ptr16 = (u16*) buffer;
                s1d13521if_BurstRead16(ptr16,copysize16);

                if (copy_to_user(user_buffer,buffer,copysize))
                        return -EFAULT;

                user_buflen -= copysize;
                user_buffer += copysize;
            }

            return 0;
            }

          default:
            return -EINVAL;
        }

        return 0;
}

//----------------------------------------------------------------------------
// Blank the display
//
// >>> Modify powerup and powerdown sequences as required.          <<<
// >>> This routine will need to be extensively modified to support <<<
// >>> any powerdown modes required.                                <<<
//
//----------------------------------------------------------------------------
int s1d13521fb_blank(int blank_mode, struct fb_info *info)
{
        dbg_info("%s(): blank_mode=%d\n", __FUNCTION__, blank_mode);

        // If nothing has changed, just return
        if (s1d13521fb_info.blank_mode == blank_mode)
        {
                dbg_info("%s(): blank_mode=%d (not changed)\n", __FUNCTION__, blank_mode);
                return 0;
        }

        // Older versions of Linux Framebuffers used VESA modes, these are now remapped to FB_ modes
        switch (blank_mode)
        {
                // Disable display blanking and get out of powerdown mode.
                case FB_BLANK_UNBLANK:
                        // If the last mode was powerdown, then it is necessary to power up.
                        if (s1d13521fb_info.blank_mode == FB_BLANK_POWERDOWN)
                        {
                        }

                        break;

                // Panels don't use VSYNC or HSYNC, but the intent here is just to blank the display
                case FB_BLANK_NORMAL:
                case FB_BLANK_VSYNC_SUSPEND:
                case FB_BLANK_HSYNC_SUSPEND:
                        break;

                case FB_BLANK_POWERDOWN:
                        // When powering down, the controller needs to get into an idle state
                        // then the power save register bits 0/1 are the key values to play with
                        // if sleep mode is used, it disables the PLL and will require 10 msec to
                        // come back on.
                        // Also, when powering down, the linux refresh timer should be stopped and
                        // restarted when coming out of powerdown.

                        // If the last mode wasn't powerdown, then powerdown here.
                        if (s1d13521fb_info.blank_mode != FB_BLANK_POWERDOWN)
                        {
                        }

                        break;

                default:
                        dbg_info("%s() dropped to default on arg %d\n", __FUNCTION__, blank_mode);
                        return -EINVAL;         // Invalid argument
        }

        s1d13521fb_info.blank_mode = blank_mode;
        return 0;
}

//----------------------------------------------------------------------------
//
//----------------------------------------------------------------------------
#if 0
struct fb_fillrect {
        __u32 dx;       /* screen-relative */
        __u32 dy;
        __u32 width;
        __u32 height;
        __u32 color;
        __u32 rop;
};
#endif

void s1d13521fb_fillrect(struct fb_info *p, const struct fb_fillrect *rect)
{
        cfb_fillrect(p,rect);
        s1d13521fb_trigger();
}

//----------------------------------------------------------------------------
//
//----------------------------------------------------------------------------
#if 0
struct fb_copyarea {
        __u32 dx;
        __u32 dy;
        __u32 width;
        __u32 height;
        __u32 sx;
        __u32 sy;
};
#endif

void s1d13521fb_copyarea(struct fb_info *p, const struct fb_copyarea *area)
{
        cfb_copyarea(p,area);
        s1d13521fb_trigger();
}

//----------------------------------------------------------------------------
//
//----------------------------------------------------------------------------
#if 0
struct fb_image {
        __u32 dx;               /* Where to place image */
        __u32 dy;
        __u32 width;            /* Size of image */
        __u32 height;
        __u32 fg_color;         /* Only used when a mono bitmap */
        __u32 bg_color;
        __u8  depth;            /* Depth of the image */
        const char *data;       /* Pointer to image data */
        struct fb_cmap cmap;    /* color map info */
#endif

void s1d13521fb_imageblit(struct fb_info *p, const struct fb_image *image)
{
        cfb_imageblit(p, image);
        s1d13521fb_trigger();
}

static int s1d13521fb_remove(struct platform_device *pdev)
{
#ifdef CONFIG_FB_EPSON_PROC
    s1d13521proc_terminate();
    kthread_stop(s1d13521_task);
    s1d13521_task = NULL;
#endif
    unregister_framebuffer(&s1d13521_fb);

    if (s1d13521fb_info.RegAddr)
        iounmap(s1d13521fb_info.RegAddr);

    platform_set_drvdata(pdev, NULL);

    if (s1d13521fb_info.VirtualFramebufferAddr)
        free_pages(s1d13521fb_info.VirtualFramebufferAddr, s1d13521fb_info.page_order);

    //[ICS] s1d13521_fb and s1d13521fb_info are both in global DATA segment, shouldn't call below function
    //framebuffer_release(info);
    return 0;
}

void s1d13521_lock(struct s1d13521fb_eink_data *eink)
{
    mutex_lock(&eink->inuse_lock);
    DEBUGP_L0(4, "*** Locked eInk\n");

    mutex_lock(&eink->lock);
        DEBUGP_L0(4, "*** Exit LowPower State\n");
        bs_cmd_run_sys();
        eink->flags = 0;
        bs_cmd_wr_reg(0x033A, 0xffff);
        DEBUGP_L0(4, "*** Exit LowPower State Done\n");
    mutex_unlock(&eink->lock);
}

void s1d13521_unlock(struct s1d13521fb_eink_data *eink)
{
    mutex_lock(&eink->lock);
        DEBUGP_L0(4, "*** Enter LowPower State\n");
        eink->flags = 1;
        bs_cmd_slp();
        DEBUGP_L0(4, "*** Enter LowPower State Done\n");
    mutex_unlock(&eink->lock);
    mutex_unlock(&eink->inuse_lock);
    DEBUGP_L0(4, "*** UnLocked eInk\n");
}

void s1d13521fb_wait_for_trigger(void)
{
#if 0
    //bs_cmd_wait_dspe_trg();
    // Wait for opperation to comp[lete
    s1d13521if_wait_for_bit( 0x338, 0, 0 );

    DEBUGP_L1(4, "*** Got Trigger Done\n");
#else
    long timeout = msecs_to_jiffies(60); // We will wait until it is done. 60ms should be plenty
    u16 status = bs_cmd_rd_reg(0x0338);
    if (status & 1)
    {
        int ret = wait_event_interruptible_timeout(trg_waitq, !(bs_cmd_rd_reg(0x0338) & (1 << 0)), timeout);
        if (ret == 0)
        {
            DEBUGP_L1(2, "*** Waiting for trigger done interrupt timedout\n");
            return;
        }
        else
        {
            DEBUGP_L1(4, "*** Got trigger done in %ld\n", timeout - ret);
            return;
        }
    }
    DEBUGP_L1(4, "*** Trigger Already Done\n");
#endif
}

void s1d13521if_wakeup(void)
{
    set_gpio_val(GPIO_HDC, 0);
    *((u16 *)pEink) = (u16)(0x06);
    set_gpio_val(GPIO_HDC, 1);
}

int s1d13521fb_wait_for_frend(void)
{
#if 0
    //bs_cmd_wait_dspe_trg();
    // Wait for opperation to comp[lete
    s1d13521if_wait_for_bit( 0x338, 3, 0 );
#else
    long timeout = msecs_to_jiffies(1000);
    u16 status = bs_cmd_rd_reg(0x0338);
    if (status & (1 << 3))
    {
        // Wait 1 second (Frame should be done in 850ms)
        int ret = wait_event_interruptible_timeout(
                    frend_waitq,
                    (gDisplayChange || !(bs_cmd_rd_reg(0x0338) & (1 << 3))),
                    timeout);
        if (ret == 0)
        {
            DEBUGP_L1(2, "*** Waiting for frame end interrupt timedout\n");
            return 0;
        }
        else
        {
            DEBUGP_L1(4, "*** Got frame end in %ld\n", timeout - ret);
        }
    }
    else
    {
        DEBUGP_L1(4, "*** Frame end Already Set\n");
    }
#endif
    return 1;
}


static void s1d13521if_eink_work_func(struct work_struct *work)
{
    u16 status = 0;
    struct s1d13521fb_eink_data *eink = container_of(work, struct s1d13521fb_eink_data, work);

    mutex_lock(&eink->lock);

    if (eink->flags)
    {
        DEBUGP_L1(5, "*** eInk Interrupt while sleeping\n");
        mutex_unlock(&eink->lock);
        enable_irq(eink->irq);
        return;
    }

    status = bs_cmd_rd_reg(0x033C);
    do
    {
        DEBUGP_L1(5, "*** eInk = Interrupt = 0x%08x\n", status);

        if (status & (1 << 0))
        {
            DEBUGP_L2(5, "*** eInk = Trigger Done\n");
            wake_up_interruptible(&trg_waitq);
        }
        if (status & (1 << 1))
        {
            DEBUGP_L2(5, "*** eInk = Update Buffer refresh Done\n");
        }
        if (status & (1 << 2))
        {
            DEBUGP_L2(5, "*** eInk = Frame Complete\n");
        }
        if (status & (1 << 3))
        {
            DEBUGP_L2(5, "*** eInk = One LUT N-Frame Complete\n");
            wake_up_interruptible(&lut_waitq);
        }
        if (status & (1 << 4))
        {
            DEBUGP_L2(5, "*** eInk = Update Buffer Changed\n");
        }
        if (status & (1 << 5))
        {
            DEBUGP_L2(5, "*** eInk = All Frames Completed\n");
            wake_up_interruptible(&frend_waitq);
        }
        if (status & (1 << 6))
        {
            DEBUGP_L2(5, "*** eInk = Display FIFO Underflow\n");
        }
        if (status & (1 << 7))
        {
            DEBUGP_L2(5, "*** eInk = LUT Busy Conflict\n");
        }

        // Clear interrupts
        bs_cmd_wr_reg(0x033A, status);

        status = bs_cmd_rd_reg(0x033C);
    } while (status);

    // Clear interrupts
    bs_cmd_wr_reg(0x033A, 0xffff);
    mutex_unlock(&eink->lock);

    enable_irq(eink->irq);
}

static irqreturn_t s1d13521fb_eink_irq_handler(int irq, void *dev_id)
{
    struct s1d13521fb_eink_data *eink = dev_id;

    disable_irq(eink->irq);
    queue_work(eink_wq, &eink->work);
    return IRQ_HANDLED;
}

// eInk enable/disable sysfs entry
static ssize_t s1d13521fb_enable_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    return sprintf(buf, "%d\n", s1d13521if_enabled());
}

static ssize_t s1d13521fb_enable_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
    int val = simple_strtol(buf, NULL, 10);

    s1d13521if_enable(val);

    return count;
}
static DEVICE_ATTR(enable, 0666, s1d13521fb_enable_show, s1d13521fb_enable_store);

// eInk redraw sysfs entry
static ssize_t s1d13521fb_redraw_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    return sprintf(buf, "%d\n", 1);
}

int s1d13521fb_do_convert_display(s3c_rect *changed);

static ssize_t s1d13521fb_redraw_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
    int val = simple_strtol(buf, NULL, 10);

    printk(KERN_NOTICE "PROC_UPD_FULL\n");
#ifdef CONFIG_MACH_BRAVO_RDU
    bs_cmd_wait_for_bit( 0x338, 0, 0 );
    bs_cmd_wait_for_bit( 0x338, 3, 0 );
    bs_cmd_wr_value_fb( 0x00 );
    bs_cmd_upd_full( 3, 0, 0 );
    bs_cmd_wait_for_bit( 0x338, 0, 0 );
    bs_cmd_wait_for_bit( 0x338, 3, 0 );
#else
    s1d13521_lock(s1d13521fb_data);
    // Take a snapshot of the current image
    s1d13521fb_do_convert_display(NULL);

    bs_cmd_wait_dspe_trg();          /* wait for op trigger to be free */
    bs_cmd_wait_dspe_frend();        /* wait for any in progress update to finish*/
    bs_cmd_upd_full(3, GC_LUT, 0);   /* request update */
    s1d13521_unlock(s1d13521fb_data);
#endif
    return count;
}
static DEVICE_ATTR(redraw, 0666, s1d13521fb_redraw_show, s1d13521fb_redraw_store);

// eInk image sysfs entry
static ssize_t s1d13521fb_image_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    return sprintf(buf, "%d\n", 1);
}

static ssize_t s1d13521fb_image_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
    char filename[1024];
    int len = count;

    memcpy(filename, buf, count);
    filename[len] = 0;
    while (len > 0)
    {
        if (filename[len] <= ' ')
            filename[len] = 0;
        else
            break;
        len--;
    }

    s1d13521_image_display(s1d13521fb_data, filename);

    return count;
}
static DEVICE_ATTR(image, 0666, s1d13521fb_image_show, s1d13521fb_image_store);

static u16 s1d13521fb_enabled_interrupts = 0;

static int __devinit s1d13521fb_probe(struct platform_device *pdev)
{
    int ret = 0;
    struct s1d13521fb_eink_data *eink;

    dbg_info("s1d13521fb_probe()\n");

    printk("Epson S1D13521 FB Driver\n");

    // Allocate the virtual framebuffer
    if (s1d13521fb_set_virtual_framebuffer())
    {
        printk(KERN_WARNING "s1d13521fb_init: _get_free_pages() failed.\n");
        ret =  -EINVAL;
        goto err_out;
    }

    s1d13521fb_check_var(&s1d13521_fb.var, &s1d13521_fb);
    s1d13521fb_set_par(&s1d13521_fb);

    eink = kzalloc(sizeof(*eink), GFP_KERNEL);
    if (eink == NULL)
    {
        ret = -ENOMEM;
        goto err_out;
    }

    INIT_WORK(&eink->work, s1d13521if_eink_work_func);
    mutex_init(&eink->lock);
    mutex_init(&eink->inuse_lock);

#ifdef CONFIG_HAS_EARLYSUSPEND
    eink->early_suspend.suspend = s1d13521_early_suspend;
    eink->early_suspend.resume = s1d13521_late_resume;
    eink->early_suspend.level = EARLY_SUSPEND_LEVEL_DISABLE_FB;
    register_early_suspend(eink);
#endif


    //-------------------------------------------------------------------------
    // Initialize Hardware Display Blank
    //-------------------------------------------------------------------------
    s1d13521fb_info.blank_mode = VESA_NO_BLANKING;
    s1d13521fb_ops.fb_blank = s1d13521fb_blank;

    // Set flags for controller supported features
    s1d13521_fb.flags = FBINFO_FLAG_DEFAULT;

    // Set the pseudo palette
    s1d13521_fb.pseudo_palette = s1d13521fb_info.pseudo_palette;

    s1d13521_fb.fbops = &s1d13521fb_ops;
    s1d13521_fb.node = -1;

    /* enable platform-dependent hardware glue, if any */
    s1d13521_fb.device = &(pdev->dev);
    platform_set_drvdata(pdev, eink);
    s1d13521fb_data = eink;

    s1d13521fb_init_hw();
    printk(KERN_NOTICE "S1D13521fb_probe: VIDEO_REFRESH_PERIOD(%d)\n", VIDEO_REFRESH_PERIOD);

#ifndef CONFIG_MACH_BRAVO_RDU
    s1d13521_task = kthread_run(s1d13521fb_sync_thread, eink, S1D13521_DEVICENAME);
#endif

    if (register_framebuffer(&s1d13521_fb) < 0)
    {
        printk(KERN_ERR "s1d13521fb_init(): Could not register frame buffer with kernel.\n");
        ret = -EINVAL;
        goto err_out;
    }

    printk("fb%d: %s frame buffer device\n", s1d13521_fb.node, s1d13521_fb.fix.id);

#ifdef CONFIG_FB_EPSON_PROC
    s1d13521proc_init();
#endif

    init_waitqueue_head(&trg_waitq);
    init_waitqueue_head(&frend_waitq);
    init_waitqueue_head(&lut_waitq);
    // Hook the eInk engine interrupt.
#ifdef CONFIG_MACH_BRAVO_EVT1
    eink->irq = IRQ_EINT_GROUP(8, 5);
#else
    eink->irq = IRQ_EINT_GROUP(7, 3);
#endif
    ret = request_irq(eink->irq, s1d13521fb_eink_irq_handler, IRQF_TRIGGER_RISING, "eInk", eink);
    if (ret != 0)
    {
        printk("**** Failed to get IRQ %d for eInk: %d\n", eink->irq, ret);
    }
    else
    {
        s1d13521fb_enabled_interrupts = 0;
        printk("**** Hooked IRQ %d for eInk.\n", eink->irq);
        // Enable Display Engine only
        bs_cmd_wr_reg(0x033A, 0xffff);  // Clear interrupts
        bs_cmd_wr_reg(0x244, 0x0002);   // Only Display Engine interrupts
        //bs_cmd_wr_reg(0x242, 0x0000);   // Unmask All
        s1d13521fb_enabled_interrupts |= (1 << 0);  // Trigger Done
        //s1d13521fb_enabled_interrupts |= (1 << 1); // Update Buffer Resfresh Done
        //s1d13521fb_enabled_interrupts |= (1 << 2); // One Frame Complete
        s1d13521fb_enabled_interrupts |= (1 << 3); // One LUT N-Frame Complete
        //s1d13521fb_enabled_interrupts |= (1 << 4); // Update Buffer Changed
        s1d13521fb_enabled_interrupts |= (1 << 5); // All Frames Complete
        //enable_interrupts |= (1 << 6); // Display FIFO Unferflow
        s1d13521fb_enabled_interrupts |= (1 << 7); // LUT Busy Conflict Detected
        bs_cmd_wr_reg(0x33E, s1d13521fb_enabled_interrupts); // Enable Selected Display Engine interrupts
    }
    // Clear interrupts
    bs_cmd_wr_reg(0x033A, 0xffff);
    eink->flags = 0;

    ret = device_create_file(&pdev->dev, &dev_attr_enable);
    if (ret < 0)
    {
        printk(KERN_ERR "s1d13521fb_probe() - WARNING: failed to add enable/disable sysfs: %d\n", ret);;
    }
    ret = device_create_file(&pdev->dev, &dev_attr_redraw);
    if (ret < 0)
    {
        printk(KERN_ERR "s1d13521fb_probe() - WARNING: failed to add redraw sysfs: %d\n", ret);;
    }
    ret = device_create_file(&pdev->dev, &dev_attr_image);
    if (ret < 0)
    {
        printk(KERN_ERR "s1d13521fb_probe() - WARNING: failed to add image sysfs: %d\n", ret);;
    }
    return 0;

err_out:
    s1d13521fb_remove(pdev);
    return ret;
}


static void s1d13521_image_display(struct s1d13521fb_eink_data *eink, char *filename)
{
    int ret = 0;
    s1d13521_lock(eink);

    do
    {
        u16 *buffer = NULL;
        u8 *image = NULL;
        long  size = 0;
        struct file* filp;
        int i;
        loff_t pos;

        filp = filp_open(filename, O_RDWR | O_LARGEFILE, 0);
        if (IS_ERR(filp))
        {
            printk(KERN_INFO "Unable to open '%s'.\n", filename);
            ret = -ENOENT;
            break;
        }
	
        size = filp->f_path.dentry->d_inode->i_size;
        if (size < (SCREEN_SAVER_DEFAULT_PIXELS*sizeof(u16)) )
        {
            //screensaver file "/data/screensaver/screensaver.bin" is being updated by application, don't touch it
            //Let's display the default screensaver file "/data/screensaver/default_screensaver.bin"
            printk(KERN_INFO "Invalid file '%s', size %d.\n", filename, size);
            filp_close(filp, current->files);

            filp = filp_open(SCREEN_SAVER_DEFAULT, O_RDWR | O_LARGEFILE, 0);
            if (IS_ERR(filp))
            {
	            printk(KERN_INFO "Unable to open '%s'.\n", SCREEN_SAVER_DEFAULT);
	            ret = -ENOENT;
	            break;
            }
            size = filp->f_path.dentry->d_inode->i_size;
            if (size < (SCREEN_SAVER_DEFAULT_PIXELS*sizeof(u16)))
            {
			printk(KERN_INFO "Invalid default screensaver file '%s', size %d.\n", SCREEN_SAVER_DEFAULT, size);
			filp_close(filp, current->files);
			break;
            }
        }

        //printk(KERN_WARNING "EINK:filp_open(%s), file_size(%d)\n", filename, size);
        buffer = vmalloc(size);
        if (buffer == NULL)
        {
            printk(KERN_INFO "Out of memory loading '%s'.\n", filename);
            filp_close(filp, current->files);
            break;
        }

        pos = 0;
        ret = kernel_read(filp, pos, buffer, size);
        if (ret != size)
        {
            printk(KERN_INFO "Error %d reading '%s'.\n", ret, filename);
            vfree(buffer);
            filp_close(filp, current->files);
            ret = 0;
            break;
        }
        filp_close(filp, current->files);

        image = (u8 *)s1d13521fb_current;
        for (i = 0; i < SCREEN_SAVER_DEFAULT_PIXELS; i++)
        {
            unsigned red, green, blue, gray;
            u16 color = buffer[i];

            red   = (color >> 11) & 0x1f;
            green = (color >> 5)  & 0x3f;
            blue  = (color & 0x1f);
            gray = (red*30 + green*59 + blue*11)/20;
            image[i] = gray;
        }

        bs_cmd_ld_img(DFMT);
        bs_cmd_wr_data_fb();
        bs_cmd_ld_img_end();
        bs_cmd_upd_full( 3, 0, 0 );
        s1d13521fb_wait_for_trigger();
        s1d13521fb_wait_for_frend();

        vfree(buffer);
        ret = 0;
    } while (0);

    if ( ret == (-ENOENT) )
    {
            printk(KERN_NOTICE "-ENOENT branch\n");
            bs_cmd_wr_value_fb( 0xFF );
            bs_cmd_upd_full( 3, 0, 0 );
            s1d13521fb_wait_for_trigger();
            s1d13521fb_wait_for_frend();
    }
	
    s1d13521_unlock(eink);
}

#ifdef CONFIG_PM

static void resume_reset( void )
{
    set_gpio_val( GPIO_RESET_L, 0 );
    mdelay(100);
    set_gpio_val( GPIO_RESET_L, 1 );
}

#ifdef CONFIG_HAS_EARLYSUSPEND

void s1d13521_early_suspend(struct early_suspend *h)
{
    struct s1d13521fb_eink_data *eink  = container_of(h, struct s1d13521fb_eink_data, early_suspend);
    int enabled = s1d13521if_enabled();

    if (enabled > 1)
	return;

    s1d13521if_enable(0);
    s1d13521_image_display(eink, SCREEN_SAVER);
}

void s1d13521_late_resume(struct early_suspend *h)
{
    struct s1d13521fb_eink_data *eink  = container_of(h, struct s1d13521fb_eink_data, early_suspend);

    s1d13521if_enable(1);
}
#endif

static int s1d13521fb_suspend(struct platform_device *dev, pm_message_t pm)
{
    struct s1d13521fb_eink_data *eink = (struct s1d13521fb_eink_data *) platform_get_drvdata(dev);

    DEBUGP_L1(4, "*** eInk Suspending\n");
    return 0;
}

static int s1d13521fb_resume(struct platform_device *dev)
{
    struct s1d13521fb_eink_data *eink = (struct s1d13521fb_eink_data *) platform_get_drvdata(dev);
    DEBUGP_L1(4, "*** eInk Resuming\n");

    resume_reset();
    s1d13521fb_resume_hw();
    {
        // Enable Display Engine only
        DEBUGP_L0(3, "*** ReEnable Interrupts (%d) - Flags = %d\n", eink->irq, eink->flags);
        bs_cmd_wr_reg(0x033A, 0xffff);  // Clear interrupts
        bs_cmd_wr_reg(0x244, 0x0002);   // Only Display Engine interrupts
        bs_cmd_wr_reg(0x33E, s1d13521fb_enabled_interrupts);
        // Clear interrupts
        bs_cmd_wr_reg(0x033A, 0xffff);

        DEBUGP_L0(4, "*** ReEnter LowPower State\n");
        mutex_lock(&eink->lock);
        eink->flags = 1;
        bs_cmd_slp();
        mutex_unlock(&eink->lock);
        DEBUGP_L0(4, "*** ReEnter LowPower State Done\n");
    }
    return 0;
}
#else
#define s1d13521fb_suspend NULL
#define s1d13521fb_resume NULL
#endif

static struct platform_driver s1d13521fb_driver = {
    .probe      = s1d13521fb_probe,
    .remove     = s1d13521fb_remove,
    .suspend    = s1d13521fb_suspend,
    .resume     = s1d13521fb_resume,
    .driver     = {
        .name   = S1D13521_DEVICENAME,
    },
};

static int __init s1d13521fb_init(void)
{
    eink_wq = create_singlethread_workqueue("eink_wq");
    if (!eink_wq)
        return -ENOMEM;
    return platform_driver_register(&s1d13521fb_driver);
}


static void __exit s1d13521fb_exit(void)
{
    platform_driver_unregister(&s1d13521fb_driver);
}

module_init(s1d13521fb_init);
module_exit(s1d13521fb_exit);

MODULE_AUTHOR("Epson Research and Development");
MODULE_DESCRIPTION("framebuffer driver for Epson s1d13521 controller");
MODULE_LICENSE("GPL");
