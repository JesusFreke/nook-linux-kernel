//-----------------------------------------------------------------------------
//
// s1d13521fb_collide.c -- frame buffer update region tracking.
// S1D13521 series of LCD controllers.
//
// Copyright(c) 2009 Barnes and Noble, Inc.
// All rights reserved.
//
// Module author : Intrinsyc Software, Inc.
//
// This file is subject to the terms and conditions of the GNU General Public
// License. See the file COPYING in the main directory of this archive for
// more details.
//
//----------------------------------------------------------------------------

#include "s1d13521fb.h"

#define REGION_THRESHOLD 8
#define ACTUAL_STRIDE 800
#define EINK_STRIDE 600

static u8 region_map[S1D_DISPLAY_HEIGHT] = {0};
static s3c_rect changed_regions[S1D_DISPLAY_HEIGHT];
static s3c_rect lut_work[32];
static u16 collision_status = 0;
static s3c_rect lut_waiting[32];
static s3c_rect lut_waiting_align[32];
static u16 eink_mode = WF_MODE_GC;
static u16 eink_border = 0;

void s1d13521fb_schedule_clear(void)
{
    collision_status = 0;
}

u16 s1d13521fb_schedule_backlog(void)
{
    return collision_status;
}

static int rect_intersect(const s3c_rect *r1, const s3c_rect *r2)
{
    return !(r2->left > r1->right
        || r2->right < r1->left
        || r2->top > r1->bottom
        || r2->bottom < r1->top);
}

static void rect_merge(const s3c_rect *r1, const s3c_rect *r2, s3c_rect *r3)
{
    r3->left = min(r1->left, r2->left);
    r3->top = min(r1->top, r2->top);
    r3->right = max(r1->right, r2->right);
    r3->bottom = max(r1->bottom, r2->bottom);
}

void s1d13521fb_dump_rect(const s3c_rect *area, int width, int height, int data)
{
    u8 *pSrc = (u8*) s1d13521fb_current;

    int i,j;
    int w = area->right-area->left+1;
    int h = area->bottom-area->top+1;
    if (h > height)
        return;
    if (w > width)
        return;

    printk("\n");
    for (i = area->top; i < area->bottom+1; i++)
    {
        for (j = area->left; j < min(area->left+40,area->right); j++)
        {
            u8 val = pSrc[i*EINK_STRIDE+j];
            if (data)
                printk("%02x", val);
            else
            {
                if (val > 0x80)
                {
                    printk(".");
                }
                else
                {
                    printk("@");
                }
            }
        }
        printk("\n");
    }
}

int s1d13521fb_check_bw_rect(s3c_rect *area)
{
    u8 *pSrc = (u8*) s1d13521fb_current;

    int i,j;

    for (i = area->top; i < area->bottom; i++)
    {
        for (j = area->left; j < area->right; j++)
        {
            u8 val = pSrc[i*EINK_STRIDE+j];

            if ((val != 0xff) && (val != 0x00))
            {
                return val;
            }
        }
    }
    return 0;
}

u16 s1d13521fb_get_lut_status(void)
{
    return bs_cmd_rd_reg(0x0336);
}

u16 s1d13521fb_get_free_lut(void)
{
    u16 lut_status = 0;
    u16 free_lut = 0;
    int i = 0;
    static int next_lut = 0;

    // Wait for at least one lut to be free
    bs_cmd_wait_dspe_lutfree();

    lut_status = bs_cmd_rd_reg(0x0336);

    for (i = 0; i < 16; i++)
    {
        if ((lut_status & (1 << next_lut)) == 0)
        {
            free_lut = next_lut;
            break;
        }
        next_lut += 1;
        next_lut &= 0xf;
    }
    next_lut += 1;
    next_lut &= 0xf;

    return free_lut;
}

void s1d13521fb_track_work(u32 lut, s3c_rect *real_area)
{
    lut_work[lut] = *real_area;
}

int s1d13521fb_schedule_work(unsigned mode, u16 collision , u16 collision_number, s3c_rect *real_area, s3c_rect *align_area)
{
    int i;
    u16 lut = 0;
    u16 lut_status = 0;
    s3c_rect merge;

    lut_status = s1d13521fb_get_lut_status();

    // Check for Collision s on Collision s
    for (i = 0; i < 16; i++)
    {
        if ((collision_status & (1 << i)) == 0)
        {
            continue;
        }

        if (rect_intersect(real_area, &lut_waiting[i]))
        {
            rect_merge(real_area, &lut_waiting[i], &merge);
            lut_waiting[i] = merge;
            rect_merge(align_area, &lut_waiting_align[i], &merge);
            lut_waiting_align[i] = merge;
            DEBUGP_L2(2,"Collision <%2d>     x=%d, y=%d, w=%d, h=%d\n",
                            i,
                            merge.left, merge.top,
                            merge.right-merge.left+1, merge.bottom-merge.top+1);
            return 1;
        }
    }

    for (i = 0; i < 16; i++)
    {
        if ((lut_status & (1 << i)) == 0)
        {
            continue;
        }

        if (rect_intersect(real_area, &lut_work[i]))
        {
            // We will have to wait
            if (collision_status & (1 << i))
            {
                rect_merge(real_area, &lut_waiting[i], &merge);
                lut_waiting[i] = merge;
                rect_merge(align_area, &lut_waiting_align[i], &merge);
                lut_waiting_align[i] = merge;

                DEBUGP_L2(2,"Collision [%2d]     x=%d, y=%d, w=%d, h=%d\n",
                                i,
                                real_area->left, real_area->top,
                                real_area->right-real_area->left+1, real_area->bottom-real_area->top+1);
            }
            else
            {
                lut_waiting[i] = *real_area;
                lut_waiting_align[i] = *align_area;
                collision_status |= (1 << i);

                DEBUGP_L2(2,"Collision (%2d)     x=%d, y=%d, w=%d, h=%d\n",
                                i,
                                real_area->left, real_area->top,
                                real_area->right-real_area->left+1, real_area->bottom-real_area->top+1);
            }

            return 1;
        }
    }

    // No Collision s detected, do the work
    lut = s1d13521fb_get_free_lut();
    s1d13521fb_track_work(lut, real_area);


    // Tell eInk to do it
    {
        //DISABLE_EINK_INTERRUPTS();
        bs_cmd_upd_part_area(eink_mode, lut, eink_border,
            real_area->left, real_area->top,
            real_area->right-real_area->left+1, real_area->bottom-real_area->top+1);

        //ENABLE_EINK_INTERRUPTS();
    }

    if (collision)
        DEBUGP_L2(2, "Processing(%2d,%2d)  x=%d, y=%d, w=%d, h=%d\n",
                    lut,
                    collision_number,
                    real_area->left, real_area->top,
                    real_area->right-real_area->left+1, real_area->bottom-real_area->top+1);
    else
        DEBUGP_L2(2, "Processing(%2d)     x=%d, y=%d, w=%d, h=%d\n",
                    lut,
                    real_area->left, real_area->top,
                    real_area->right-real_area->left+1, real_area->bottom-real_area->top+1);

#if 0
    if (s1d13521fb_check_bw_rect(real_area))
        s1d13521fb_dump_rect(real_area, 180, 15, 1);
#endif

    // wait for op trigger to be free
    s1d13521fb_wait_for_trigger();
    return 0;
}

int s1d13521fb_process_collisions(void)
{
    int i = 0;

    if (!collision_status)
        return 0;

    for (i = 0; i < 16; i++)
    {
        u16 mask = (1 << i);
        u16 lut_status = s1d13521fb_get_lut_status();

        if (((lut_status & mask) == 0) && (collision_status & mask))
        {
            // Mark the Collision  as resolved
            collision_status &= ~mask;
            s1d13521fb_schedule_work(eink_mode, 1, i, &lut_waiting[i], &lut_waiting_align[i]);
        }
    }
    return collision_status;
}

int s1d13521fb_update_region_map(s3c_rect *changed)
{
    int line, region;
    int x_min, x_max;
    int new_region;

    region = 0;
    new_region = true;

    // Search for difference from top of image
    for (line = changed->top; line <= changed->bottom; line++)
    {
        int left_x;
        u8 *pnew = (u8*) (s1d13521fb_current+line*S1D_DISPLAY_WIDTH);
        u8 *pold = (u8*) (s1d13521fb_previous+line*S1D_DISPLAY_WIDTH);

        // Compare from line begining to end to find difference for region
        x_min = S1D_DISPLAY_WIDTH;
        for (left_x = 0; left_x < S1D_DISPLAY_WIDTH; left_x++)
        {
            if (pnew[left_x] != pold[left_x])
            {
                x_min = left_x;
                break;
            }
        }

        if (x_min < (S1D_DISPLAY_WIDTH-1))
        {
            int right_x;

            // Compare from line end to x_min to find max X for region
            x_max = 0;
            for (right_x = S1D_DISPLAY_WIDTH-1; right_x >= x_min; right_x--)
            {
                if (pnew[right_x] != pold[right_x])
                {
                    x_max = right_x;
                    break;
                }
            }
            if (new_region)
            {
                new_region = false;
                region += 1;
                changed_regions[region-1].top = line;
                changed_regions[region-1].bottom = line;
                changed_regions[region-1].right = 0;
                changed_regions[region-1].left = S1D_DISPLAY_WIDTH-1;
            }
            changed_regions[region-1].bottom = line;
            changed_regions[region-1].right = max(changed_regions[region-1].right,x_max);
            changed_regions[region-1].left = min(changed_regions[region-1].left,x_min);
        }
        else
        {
            if (!new_region)
            {
                new_region = true;
            }
        }
    }
    return region;
}

int s1d13521fb_schedule_rect(unsigned cmd, unsigned mode, s3c_rect *real_area, u16 x, u16 y, u16 w, u16 h)
{
    s3c_rect area_align;

    area_align.top = y;
    area_align.left = x;
    area_align.bottom = y+h-1;
    area_align.right = x+w-1;

    s1d13521fb_process_collisions();

    // Try to do the work.
    s1d13521fb_schedule_work(mode, 0,0, real_area, &area_align);

    // Check Area Overlap (not needed any more)
    if (bs_cmd_rd_reg(BS_INTERRUPT_STATUS_MASKED) & 1<<7)
    {
        s3c_rect area;
        u16 lut;

        DEBUGP(1,"**  eInk detected Overlap\n");
        bs_cmd_wr_reg(BS_INTERRUPT_STATUS_MASKED, 1<<7); // Clear the overlap error

        bs_cmd_wait_dspe_trg();              /* wait for op trigger to be free */
        bs_cmd_wait_dspe_frend();            /* wait for any in progress update to finish*/
        s1d13521fb_schedule_clear();

        lut = s1d13521fb_get_free_lut();
        area.top = 0;
        area.left = 0;
        area.bottom = S1D_DISPLAY_HEIGHT-1;
        area.right = S1D_DISPLAY_WIDTH-1;
        s1d13521fb_track_work(0, &area);

        bs_cmd_upd_part(UPD_PART, lut, eink_border);
        bs_cmd_wait_dspe_trg();
   }
    return 1;
}

int s1d13521fb_update_regions(s3c_rect *changed)
{
    unsigned cmd = UPD_PART;
    int i;
    u64 tcalc = TIME_STAMP();
    int regions = s1d13521fb_update_region_map(changed);

    DEBUGP_L1(2,"Regions Found = %d in %dms\n", regions, TIME_DELTA_MS(tcalc));

    tcalc = TIME_STAMP();

    // Check if it is worthwhile to do this work as regions.
    if (regions < REGION_THRESHOLD)
    {
        int total_height = 0;
        for (i = 0; i < regions; i++)
        {
            int h = changed_regions[i].bottom - changed_regions[i].top + 1;

            total_height += h; 
            // Just do a full update if the area is too big.
            if (total_height > (S1D_DISPLAY_HEIGHT/2))
            {
                DEBUGP_L1(2, "Full Update: Combined Height %d > %d\n", total_height, (S1D_DISPLAY_HEIGHT/2));
                return 0;
            }
        }
    }
    else
    {
        // Just do a full update if the area is too complicated.
        return 0;
    }

    for (i = 0; i < regions; i++)
    {
        int min_x, max_x;
        int h = changed_regions[i].bottom - changed_regions[i].top + 1;
        int w = 0;
        int y = changed_regions[i].top;

        // Align for 16Bit transfer
        min_x = changed_regions[i].left;
        max_x = changed_regions[i].right;
        if ((min_x > 0) && (min_x & 1))
            min_x -= 1;
        if ((max_x > 0) && (max_x & 1))
            max_x += 1;
        w = max_x - min_x + 1;

        s1d13521fb_schedule_rect(cmd, WF_MODE_GC, &changed_regions[i], min_x, y, w, h);
    }
    DEBUGP_L2(3, "done in %dms\n", TIME_DELTA_MS(tcalc));

    return 1;
}
