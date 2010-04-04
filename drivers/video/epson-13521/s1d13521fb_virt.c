/*
 * drivers/video/epson-13521/s1d13521fb_virt.c
 *
 * Copyright (c) 2009 Barnes and Noble, Inc
 * All rights reserved.
 *
 * Module author: Intrinsyc Software, Inc
 *
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file COPYING in the main directory of this archive for
 * more details.
 *
 * Virtual screen management routines.
 */

#define S1D13521FB_VERSION              "S1D13521FB: $Revision: 1 $"
#include <linux/version.h>
#include <linux/module.h>
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

#include "s1d13521fb.h"

#ifndef CONFIG_MACH_BRAVO_RDU
#include "startup1.h"
#include "startup2.h"
#else
const char startup1_data[] = "Dummy";
const char startup2_data[] = "Dummy";
#endif

extern u32 S3C_FrameBuffer_for_eink;
extern u32 S3C_FrameBuffer_for_eink_dma;

extern struct task_struct *s1d13521_task;

#define TIME_UPDATE 1

#define PREV_FRAMEBUFFER (VFB_SIZE_BYTES*2)

extern u32 s1d13521fb_init_done; // Hardware initialization done?
static u32 s1d13521fb_enabled = 0;   // Ignore all updates
static u32 s1d13521fb_changed = 0;   // Do we need to update the eInk display

static volatile int gDisplayChange = 0;          // Display update needed flag
static int bdr_val = 0;

extern int s1d13521fb_update_regions(s3c_rect *changed);
extern int s1d13521fb_schedule_rect(unsigned cmd, unsigned mode, s3c_rect *real_area, u16 x, u16 y, u16 w, u16 h);
extern void s1d13521fb_schedule_clear(void);
extern void s1d13521fb_track_work(u32 lut, s3c_rect *real_area);
extern u16 s1d13521fb_schedule_backlog(void);
extern int s1d13521fb_process_collisions(void);
extern u16 s1d13521fb_get_free_lut(void);
extern void s1d13521fb_black( void );
void DisplayBootAnimation(void);
void DrawPartial(void);

extern wait_queue_head_t lut_waitq;

void s1d13521if_dirty(int update)
{
    s1d13521fb_changed = update;
}

int s1d13521if_dirtied(void)
{
    return s1d13521fb_changed;
}

int s1d13521if_enabled(void)
{
    return s1d13521fb_init_done ? s1d13521fb_enabled : 0;
}

void s1d13521if_enable(int enable)
{
    if (s1d13521fb_enabled > 1)
    {
    	// if the value has previously been set to > 1, do NOT
    	// allow the value to be overwritten
        return;
	}

    s1d13521fb_enabled = enable;
}

#include "../../media/video/samsung/scaler/s3c-tvscaler.h"

static scaler_params_t conv_param = {0,};

extern void s3c_tvscaler_init(void);
extern void s3c_tvscaler_config(scaler_params_t *sp);
extern void s3c_tvscaler_start(void);
extern void s3c_tvscaler_int_enable(u32 int_type);
extern void s1d13521if_wakeup(void);

static int s1d13521fb_find_changed_region(s3c_rect *changed)
{
    int first, last;
    int start, end;

    u8 *pNew = s1d13521fb_current;
    u8 *pOld = s1d13521fb_previous;

    changed->left = 0;
    changed->right = S1D_DISPLAY_WIDTH-1;
    changed->top = 0;
    changed->bottom = S1D_DISPLAY_HEIGHT-1;

    // Search for difference from top of image
    first = 0;
    for (start = 0; start < S1D_DISPLAY_HEIGHT; start++)
    {
        if (!memcmp(pNew+start*S1D_DISPLAY_WIDTH,pOld+start*S1D_DISPLAY_WIDTH, S1D_DISPLAY_WIDTH))
            first = start;
        else
            break;
    }

    // Search for difference from bottom of image
    last = 0;
    for (end = S1D_DISPLAY_HEIGHT-1; end >= 0; end--)
    {
        if (!memcmp(pNew+end*S1D_DISPLAY_WIDTH,pOld+end*S1D_DISPLAY_WIDTH, S1D_DISPLAY_WIDTH))
            last = end;
        else
            break;
    }

    // Check if we found a difference
    if ((first == S1D_DISPLAY_HEIGHT-1) && (last == 0))
    {
        // No change, Do Nothing
        return 0;
    }

    if (first < last)
    {
        changed->top = first;
        changed->bottom = last;
    }
    return 1;
}

static void s1d13521fb_copy_to_eink(const s3c_rect *changed)
{
    //DISABLE_EINK_INTERRUPTS();
    if (changed)
    {
        int h = changed->bottom - changed->top + 1;
        if (h < (S1D_DISPLAY_HEIGHT/2))
        {
            // Update the eInk inernal buffer (use full width) for partial
            bs_cmd_ld_img_area(DFMT, 0, changed->top, S1D_DISPLAY_WIDTH, h);
            bs_cmd_wr_data_area_fb(0, changed->top, S1D_DISPLAY_WIDTH, h);
            bs_cmd_ld_img_end();
        }
        else
        {
            // Copy the whole buffer (full update)
            bs_cmd_ld_img(DFMT);
            bs_cmd_wr_data_fb();
            bs_cmd_ld_img_end();
        }
    }
    else
    {
        // Copy the whole buffer (full update)
        bs_cmd_ld_img(DFMT);
        bs_cmd_wr_data_fb();
        bs_cmd_ld_img_end();
    }
    //ENABLE_EINK_INTERRUPTS();
    return;
}

int s1d13521fb_do_convert_display(s3c_rect *changed)
{
    int retval = 0;
    void *temp;
    u64 tcalc = TIME_STAMP();
    
    // Save the old image.
    temp = s1d13521fb_current;
    s1d13521fb_current = s1d13521fb_previous;
    s1d13521fb_previous = temp;
    
    tcalc = TIME_STAMP();

    // Source
    conv_param.SrcFrmSt = (u32) S3C_FrameBuffer_for_eink_dma;
    conv_param.SrcFullWidth = S1D_DISPLAY_WIDTH;
    conv_param.SrcFullHeight= S1D_DISPLAY_HEIGHT;
    conv_param.SrcCSpace = RGB16;
    conv_param.SrcStartX = conv_param.SrcStartY = 0;
    conv_param.SrcWidth = conv_param.SrcFullWidth;
    conv_param.SrcHeight = conv_param.SrcFullHeight;

    // Dest
    conv_param.DstFrmSt = virt_to_phys(s1d13521fb_current);
    conv_param.DstFullWidth = S1D_DISPLAY_WIDTH;
    conv_param.DstFullHeight= S1D_DISPLAY_HEIGHT;
    conv_param.DstCSpace = YC420;
    conv_param.DstStartX = conv_param.DstStartY = 0;
    conv_param.DstWidth = conv_param.DstFullWidth;
    conv_param.DstHeight = conv_param.DstFullHeight;


    conv_param.InPath = POST_DMA;
    conv_param.OutPath = POST_DMA;
    conv_param.Mode = ONE_SHOT;

    s3c_tvscaler_init();
    s3c_tvscaler_config(&conv_param);
    s3c_tvscaler_int_enable(1);
    s3c_tvscaler_start();

    DEBUGP_L1(3, "Color Convert done in %dms\n", TIME_DELTA_MS(tcalc));

    if (changed)
    {
        tcalc = TIME_STAMP();
        retval = s1d13521fb_find_changed_region(changed);
        DEBUGP_L1(3, "Find Page Change (x=%d, y=%d, w=%d, h=%d) done in %dms\n",
                 changed->left, changed->top,
                 changed->right-changed->left+1, changed->bottom-changed->top+1,
                 TIME_DELTA_MS(tcalc));
    }
    else
        retval = 1;

    if (retval)
    {
        tcalc = TIME_STAMP();
        s1d13521fb_copy_to_eink(changed);
        DEBUGP_L1(3, "Copy to eInk done in %dms\n", TIME_DELTA_MS(tcalc));
    }

    return retval;
}

void s1d13521fb_do_area(unsigned cmd, unsigned mode, u16 x, u16 y, u16 w, u16 h)
{
    u16 lut = 0;
    u64 tcalc = 0;
    DEBUGP_L1(3, "cmd=%d, mode=%d, x=%d, y=%d, w=%d, h=%d\n", cmd, mode, x, y, w, h);

    tcalc = TIME_STAMP();

    lut = s1d13521fb_get_free_lut();

    bs_cmd_ld_img_area(DFMT, x, y, w, h);
    bs_cmd_wr_data_area_fb(x, y, w, h);
    bs_cmd_ld_img_end();

    if (cmd == UPD_FULL)
    {
        bs_cmd_upd_full_area(mode, lut, bdr_val, x, y, w, h);
    }
    else
    {
        bs_cmd_upd_part_area(mode, lut, bdr_val, x, y, w, h);
    }

    bs_cmd_wait_dspe_trg();              /* wait for op trigger to be free */

    DEBUGP_L1(3, "done in %dms\n", TIME_DELTA_MS(tcalc));
}

//----------------------------------------------------------------------------
// s1d13521fb_do_refresh_display(void): unconditionally refresh display:
// This function updates the display
//----------------------------------------------------------------------------
void s1d13521fb_do_refresh_display(unsigned cmd,unsigned mode, s3c_rect *changed)
{
    u64 tcalc = TIME_STAMP();
    s3c_rect area;
    DEBUGP_L1(3, "\n");

    if (!s1d13521fb_init_done)
        return;

    if (changed)
    {
        // Check if we need to do a full update (more than 1/2 of the screen changed)
        // if ((h < (S1D_DISPLAY_HEIGHT/2)) && (s1d13521fb_update_regions(changed)))
        if (s1d13521fb_update_regions(changed))
        {
            DEBUGP_L1(3, "Partial update done in %dms\n", TIME_DELTA_MS(tcalc));
            return;
        }
    }

    // Do a full update
    s1d13521fb_wait_for_trigger();	
    s1d13521fb_wait_for_frend();
    s1d13521fb_schedule_clear();

    bs_cmd_ld_img(DFMT);
    bs_cmd_wr_data_fb();
    bs_cmd_ld_img_end();

    bs_cmd_wait_dspe_lutfree();
    area.top = 0;
    area.left = 0;
    area.bottom = S1D_DISPLAY_HEIGHT-1;
    area.right = S1D_DISPLAY_WIDTH-1;
    s1d13521fb_track_work(0, &area);

    //bs_cmd_set_wfm_auto_sel_mode(0);
    if (cmd == UPD_FULL)
    {
        bs_cmd_upd_full(mode, 0, bdr_val);    /* request update */
    }
    else
    {
        bs_cmd_upd_part(mode, 0, bdr_val);
    }
    s1d13521fb_wait_for_trigger();
    //bs_cmd_set_wfm_auto_sel_mode(1);

    DEBUGP_L1(3, "Full update done in %dms\n", TIME_DELTA_MS(tcalc));
}

//----------------------------------------------------------------------------
// PRIVATE FUNCTION:
// This function updates the display
//----------------------------------------------------------------------------
void s1d13521fb_refresh_display(s3c_rect *changed)
{
    if (!s1d13521fb_init_done)
        return;

    if (s1d13521fb_info.blank_mode == VESA_NO_BLANKING)
    {
        unsigned mode,cmd;

        mode = WF_MODE_GC;
        cmd = UPD_FULL;

        s1d13521fb_do_refresh_display(cmd, mode, changed);
    }
}

void s1d13521fb_trigger(void)
{
    // Disabled by user or still not initialized
    if (s1d13521if_enabled() != 1)
        return;

    // Did the eInk portion of the display change?
    if (!s1d13521if_dirtied())
        return;

    gDisplayChange = 1;          // Display update needed flag
    wake_up_process(s1d13521_task);
    DEBUGP_L0(3, "*** LCD Trigger\n");
}

int s1d13521fb_sync_thread(void *arg)
{
    struct s1d13521fb_eink_data *eink = arg;

    set_freezable();
    s1d13521_lock(eink);

// Do not display boot animation for RDU.
#ifndef CONFIG_MACH_BRAVO_RDU
    DisplayBootAnimation();
#endif    
    
    while (!kthread_should_stop())
    {
        if (!gDisplayChange)
        {
            set_current_state(TASK_INTERRUPTIBLE);
            schedule();
            if (bs_cmd_rd_reg(0x0006) & 1)
            {
                s1d13521_lock(eink);
            }
        }

        while (gDisplayChange)
        {
            if (gDisplayChange)
            {
                s3c_rect changed;
                u64 tcalc = TIME_STAMP();
                DEBUGP_L0(3, "Triggered\n");

                gDisplayChange = 0;
                // Take a snapshot of the current image
                if (s1d13521fb_do_convert_display(&changed))
                {
                    // Process
                    s1d13521fb_refresh_display(&changed);
                    DEBUGP_L0(3, "Work done in %dms\n", TIME_DELTA_MS(tcalc));
                }
            }

            while (s1d13521fb_process_collisions())
            {
                if (interruptible_sleep_on_timeout(&lut_waitq, 1*HZ) == 0)
                {
                    DEBUGP_L0(3, "*** lut_waitq timedout\n");
                }

                if (gDisplayChange)
                    break;
            }
        }

        if (!s1d13521fb_schedule_backlog())
        {
            // Save Power
            schedule_timeout_interruptible(2*HZ);
            {
                if (!gDisplayChange)
                {
                    s1d13521_unlock(eink);
                    DEBUGP_L0(4, "*** Try to Freeze\n");
                    try_to_freeze();
                    DEBUGP_L0(4, "*** UnFrozen\n");
                }
            }
        }
        else
        {
            DEBUGP_L0(3, "*** LowPower State Conditions gDisplay=%d collisions=%d\n",
                    gDisplayChange,
                    s1d13521fb_schedule_backlog());
        }
    }
    return 0;
}                           

#ifndef CONFIG_MACH_BRAVO_RDU
void DisplayBootAnimation()
{
    int count = 0;
    int updateRate = 2;  // Boot animation updated every 5 seconds.
    int timeRemaining;

    // Clear screen.
    s1d13521fb_black();

    while (!s1d13521fb_enabled)
    {
        // Alternate drawing between the two startup images.
        if (count % 2 == 0)
        {
            memcpy(s1d13521fb_current, startup1_data, startup1_size);
        }
        else
        {
            memcpy(s1d13521fb_current, startup2_data, startup2_size);
        }
        DrawPartial();

        // When Android is ready to run, 's1d13521fb_enabled' is set and the 
        // thread is woken up by s1d13521fb_trigger() being called. This timeout
        // will then be interrupted and the boot animation will complete.
        set_current_state(TASK_INTERRUPTIBLE);
        timeRemaining = schedule_timeout(HZ * updateRate);
        if (timeRemaining != 0)
        {
            break;
        }
        count++;
    }
}
#endif

// Does a partial update but doesn't wait for the drawing to be complete. Used
// when there is no chance that the frame buffer will be written to again before
// it finishes drawing.
void DrawPartial(void)
{
    bs_cmd_ld_img(DFMT);
    bs_cmd_wr_data_fb();
    bs_cmd_ld_img_end();
    bs_cmd_upd_part(UPD_PART, 0, 0);
}

