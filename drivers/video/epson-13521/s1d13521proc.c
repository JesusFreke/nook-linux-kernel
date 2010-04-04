//-----------------------------------------------------------------------------
//
// linux/drivers/video/epson/s1d13521proc.c -- proc handling for frame buffer
// driver for Epson S1D13521 LCD controller.
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

#ifdef CONFIG_FB_EPSON_PROC

#include <linux/version.h>
#include <linux/proc_fs.h>

#include <linux/syscalls.h>
#include <linux/mm.h>
#include <linux/vmalloc.h>

#include <mach/gpio.h>
#include <plat/gpio-cfg.h>

#include <mach/map.h>
#include <plat/gpio-cfg.h>
#include <plat/regs-gpio.h>


#include "s1d13521fb.h"

// Definitions for "type" in proc_read_fb() and proc_write_fb()
#define PROC_INFO       0
#define PROC_REG        1
#define PROC_FRAME      2
#define PROC_INIT       3
#define PROC_S3C_REG 4

#define PROC_SET_LUT_AUTO_SELECT 5
#define PROC_SET_ROT_MODE 6
#define PROC_LD_IMG_PGM 7
#define PROC_LD_IMG_AREA_PGM 8
#define PROC_UPD_FULL 9
#define PROC_DRAW_SCREEN 0xa
#define PROC_UPDATE_BY_TIMER 0xb
#define PROC_ENABLE_UPDATE 0xc
#define PROC_UPDATE_WAVEFORM 0xd

struct s3c_gpio_reg {
    const char *reg_name;
    void __iomem *addr;
};

static const struct s3c_gpio_reg  s3c_regs[] = {
{"GPACON",             S3C64XX_GPACON},
{"GPADAT",             S3C64XX_GPADAT},
{"GPAPUD",             S3C64XX_GPAPUD},
{"GPACONSLP",          S3C64XX_GPACONSLP},
{"GPAPUDSLP",          S3C64XX_GPAPUDSLP},
{"GPBCON",             S3C64XX_GPBCON},
{"GPBDAT",             S3C64XX_GPBDAT},
{"GPBPUD",             S3C64XX_GPBPUD},
{"GPBCONSLP",          S3C64XX_GPBCONSLP},
{"GPBPUDSLP",          S3C64XX_GPBPUDSLP},
{"GPCCON",             S3C64XX_GPCCON},
{"GPCDAT",             S3C64XX_GPCDAT},
{"GPCPUD",             S3C64XX_GPCPUD},
{"GPCCONSLP",          S3C64XX_GPCCONSLP},
{"GPCPUDSLP",          S3C64XX_GPCPUDSLP},
{"GPDCON",             S3C64XX_GPDCON},
{"GPDDAT",             S3C64XX_GPDDAT},
{"GPDPUD",             S3C64XX_GPDPUD},
{"GPDCONSLP",          S3C64XX_GPDCONSLP},
{"GPDPUDSLP",          S3C64XX_GPDPUDSLP},
{"GPECON",             S3C64XX_GPECON},
{"GPEDAT",             S3C64XX_GPEDAT},
{"GPEPUD",             S3C64XX_GPEPUD},
{"GPECONSLP",          S3C64XX_GPECONSLP},
{"GPEPUDSLP",          S3C64XX_GPEPUDSLP},
{"GPFCON",             S3C64XX_GPFCON},
{"GPFDAT",             S3C64XX_GPFDAT},
{"GPFPUD",             S3C64XX_GPFPUD},
{"GPFCONSLP",          S3C64XX_GPFCONSLP},
{"GPFPUDSLP",          S3C64XX_GPFPUDSLP},
{"GPGCON",             S3C64XX_GPGCON},
{"GPGDAT",             S3C64XX_GPGDAT},
{"GPGPUD",             S3C64XX_GPGPUD},
{"GPGCONSLP",          S3C64XX_GPGCONSLP},
{"GPGPUDSLP",          S3C64XX_GPGPUDSLP},
{"GPHCON0",            S3C64XX_GPHCON0},
{"GPHCON1",            S3C64XX_GPHCON1},
{"GPHDAT",             S3C64XX_GPHDAT},
{"GPHPUD",             S3C64XX_GPHPUD},
{"GPHCONSLP",          S3C64XX_GPHCONSLP},
{"GPHPUDSLP",          S3C64XX_GPHPUDSLP},
{"GPICON",             S3C64XX_GPICON},
{"GPIDAT",             S3C64XX_GPIDAT},
{"GPIPUD",             S3C64XX_GPIPUD},
{"GPICONSLP",          S3C64XX_GPICONSLP},
{"GPIPUDSLP",          S3C64XX_GPIPUDSLP},
{"GPJCON",             S3C64XX_GPJCON},
{"GPJDAT",             S3C64XX_GPJDAT},
{"GPJPUD",             S3C64XX_GPJPUD},
{"GPJCONSLP",          S3C64XX_GPJCONSLP},
{"GPJPUDSLP",          S3C64XX_GPJPUDSLP},
{"GPKCON0",            S3C64XX_GPKCON},
{"GPKCON1",            S3C64XX_GPKCON1},
{"GPKDAT",             S3C64XX_GPKDAT},
{"GPKPUD",             S3C64XX_GPKPUD},
{"GPLCON0",            S3C64XX_GPLCON},
{"GPLCON1",            S3C64XX_GPLCON1},
{"GPLDAT",             S3C64XX_GPLDAT},
{"GPLPUD",             S3C64XX_GPLPUD},
{"GPMCON",             S3C64XX_GPMCON},
{"GPMDAT",             S3C64XX_GPMDAT},
{"GPMPUD",             S3C64XX_GPMPUD},
{"GPNCON",             S3C64XX_GPNCON},
{"GPNDAT",             S3C64XX_GPNDAT},
{"GPNPUD",             S3C64XX_GPNPUD},
{"GPOCON",             S3C64XX_GPOCON},
{"GPODAT",             S3C64XX_GPODAT},
{"GPOPUD",             S3C64XX_GPOPUD},
{"GPOCONSLP",          S3C64XX_GPOCONSLP},
{"GPOPUDSLP",          S3C64XX_GPOPUDSLP},
{"GPPCON",             S3C64XX_GPPCON},
{"GPPDAT",             S3C64XX_GPPDAT},
{"GPPPUD",             S3C64XX_GPPPUD},
{"GPPCONSLP",          S3C64XX_GPPCONSLP},
{"GPPPUDSLP",          S3C64XX_GPPPUDSLP},
{"GPQCON",             S3C64XX_GPQCON},
{"GPQDAT",             S3C64XX_GPQDAT},
{"GPQPUD",             S3C64XX_GPQPUD},
{"GPQCONSLP",          S3C64XX_GPQCONSLP},
{"GPQPUDSLP",          S3C64XX_GPQPUDSLP},
};

//-----------------------------------------------------------------------------
// Local Function Prototypes
//-----------------------------------------------------------------------------
static int  proc_read_fb(char *page, char **start, off_t off, int count, int *eof, void *data);
static int  proc_write_fb(struct file *file, const char *buffer, int count, void *data);
static long s1d13521proc_htol(const char *szAscii);
static int  s1d13521proc_iswhitechar(int c);

//-----------------------------------------------------------------------------
// Local Globals
//-----------------------------------------------------------------------------
static struct proc_dir_entry *s1d13521fb_dir = NULL;
static struct proc_dir_entry *info_file = NULL;
static struct proc_dir_entry *reg_file = NULL;
static struct proc_dir_entry *s3c_reg_file = NULL;
static struct proc_dir_entry *ft_file = NULL;
static struct proc_dir_entry *init_file = NULL;

static struct proc_dir_entry *lut_auto = NULL;
static struct proc_dir_entry *rot_mode = NULL;
static struct proc_dir_entry *ld_pgm = NULL;
static struct proc_dir_entry *ld_area_pgm = NULL;
static struct proc_dir_entry *update_full = NULL;
static struct proc_dir_entry *draw_screen = NULL;
static struct proc_dir_entry *update_by_timer = NULL;
static struct proc_dir_entry *enable_update = NULL;
static struct proc_dir_entry *waveform_update = NULL;

static unsigned long ProcRegIndex = 0;
static unsigned long ProcRegVal = 0;
static unsigned char ProcS3cReg[16] = {0x0};
static unsigned long ProcS3cRegVal = 0x0;

extern int gDisplayChange;
extern struct s1d13521fb_eink_data *s1d13521fb_data;

#ifdef CONFIG_FB_EPSON_GPIO_SMDK6410
extern void init_gpio(void);
extern int dump_gpio(char *buf);
#endif
extern unsigned int get_s3c_reg_val(void __iomem *reg_addr);
extern unsigned int set_s3c_reg_val(void __iomem *reg_addr, unsigned int regval);
extern struct proc_dir_entry proc_root;
//-----------------------------------------------------------------------------
// /proc setup
//-----------------------------------------------------------------------------
static int local_atoi(const char *name, u32 *pos)
{
    int val = 0;

    for (;; name++) {
    switch (*name) {
        case '0' ... '9':
        val = 10*val+(*name-'0');
        break;
        default:
        *pos = (u32)name;
        return val;
    }
    }
}

int get_matching_s3c(const char *name, u32 *p_index)
{
    u32 i;
    struct s3c_gpio_reg *p_reg;

    for (i = 0; i < (sizeof(s3c_regs)/sizeof(struct s3c_gpio_reg)); i++)
    {
        if (!strcmp(s3c_regs[i].reg_name, name))
        {
            //printk(KERN_NOTICE "get_matching_s3c:s3c_regs[i].reg_name(%d), name(%d)\n", strlen(s3c_regs[i].reg_name), strlen(name));
            *p_index = i;
            return 0;
        }
    }

    *p_index = 0xffffffff;
    return -1;
}


int s1d13521proc_get_s3cregval(unsigned char *ProcS3cReg, u32 *p_regval)
{
    u32 s3c_reg_index = 0;
    int ret = 0;
    u32 regval;

    ret = get_matching_s3c(ProcS3cReg, &s3c_reg_index);
    if (ret)
    {
        printk(KERN_NOTICE "S3C REG(%s) doesn't exists\n", ProcS3cReg);
        return -1;
    }
    else
    {
        regval = get_s3c_reg_val(s3c_regs[s3c_reg_index].addr);
        printk(KERN_NOTICE "S3C REG(%s)  0x%x[0x%x]\n", ProcS3cReg, s3c_regs[s3c_reg_index].addr, regval);
        *p_regval = regval;
        return 0;
    }
}

void s1d13521proc_set_s3cregval(unsigned char *ProcS3cReg, u32 ProcS3cRegVal)
{
    u32 s3c_reg_index = 0;
    int ret = 0;
    u32 regval0, regval1;

    ret = get_matching_s3c(ProcS3cReg, &s3c_reg_index);
    if (ret)
    {
        printk(KERN_NOTICE "S3C REG(%s) doesn't exists\n", ProcS3cReg);
    }
    else
    {
        regval0 = get_s3c_reg_val(s3c_regs[s3c_reg_index].addr);
        set_s3c_reg_val(s3c_regs[s3c_reg_index].addr, ProcS3cRegVal);
        regval1 = get_s3c_reg_val(s3c_regs[s3c_reg_index].addr);
        printk(KERN_NOTICE "S3C REG(%s)  0x%x[0x%x --> 0x%x]\n", ProcS3cReg, s3c_regs[s3c_reg_index].addr, regval0, regval1);
    }
}

int s1d13521proc_init(void)
{
        // First setup a subdirectory for s1d13521fb
        s1d13521fb_dir = proc_mkdir("s1d13521fb", NULL);

        if (!s1d13521fb_dir)
                return -ENOMEM;

        s1d13521fb_dir->owner = THIS_MODULE;
        info_file = create_proc_read_entry("info", 0444, s1d13521fb_dir, proc_read_fb, (void*)PROC_INFO);

        if (info_file == NULL)
                return -ENOMEM;

        ft_file = create_proc_entry("ft", 0644, s1d13521fb_dir);

        if (ft_file == NULL)
                return -ENOMEM;

        ft_file->data = (void *)PROC_FRAME;
        ft_file->read_proc = proc_read_fb;
        ft_file->write_proc = proc_write_fb;
        ft_file->owner = THIS_MODULE;

        init_file = create_proc_entry("init", 0644, s1d13521fb_dir);

        if (init_file == NULL)
                return -ENOMEM;

        init_file->data = (void *)PROC_INIT;
        init_file->read_proc = proc_read_fb;
        init_file->write_proc = proc_write_fb;
        init_file->owner = THIS_MODULE;

        reg_file = create_proc_entry("regio", 0644, s1d13521fb_dir);

        if (reg_file == NULL)
                return -ENOMEM;

        reg_file->data = (void *)PROC_REG;
        reg_file->read_proc = proc_read_fb;
        reg_file->write_proc = proc_write_fb;
        reg_file->owner = THIS_MODULE;

        s3c_reg_file = create_proc_entry("s3creg", 0644, s1d13521fb_dir);

        if (s3c_reg_file == NULL)
                return -ENOMEM;

        s3c_reg_file->data = (void *)PROC_S3C_REG;
        s3c_reg_file->read_proc = proc_read_fb;
        s3c_reg_file->write_proc = proc_write_fb;
        s3c_reg_file->owner = THIS_MODULE;

        lut_auto = create_proc_entry("lut_auto", 0644, s1d13521fb_dir);
        if (lut_auto == NULL)
                return -ENOMEM;
        lut_auto->data = (void *)PROC_SET_LUT_AUTO_SELECT;
        lut_auto->read_proc = proc_read_fb;
        lut_auto->write_proc = proc_write_fb;
        lut_auto->owner = THIS_MODULE;

        rot_mode = create_proc_entry("rot_mode", 0644, s1d13521fb_dir);
        if (rot_mode == NULL)
                return -ENOMEM;
        rot_mode->data = (void *)PROC_SET_ROT_MODE;
        rot_mode->read_proc = proc_read_fb;
        rot_mode->write_proc = proc_write_fb;
        rot_mode->owner = THIS_MODULE;

        ld_pgm = create_proc_entry("ld_pgm", 0644, s1d13521fb_dir);
        if (ld_pgm == NULL)
                return -ENOMEM;
        ld_pgm->data = (void *)PROC_LD_IMG_PGM;
        ld_pgm->read_proc = proc_read_fb;
        ld_pgm->write_proc = proc_write_fb;
        ld_pgm->owner = THIS_MODULE;

        ld_area_pgm = create_proc_entry("ld_area_pgm", 0644, s1d13521fb_dir);
        if (ld_area_pgm == NULL)
                return -ENOMEM;
        ld_area_pgm->data = (void *)PROC_LD_IMG_AREA_PGM;
        ld_area_pgm->read_proc = proc_read_fb;
        ld_area_pgm->write_proc = proc_write_fb;
        ld_area_pgm->owner = THIS_MODULE;

        update_full = create_proc_entry("update_full", 0666, s1d13521fb_dir);
        if (update_full == NULL)
                return -ENOMEM;
        update_full->data = (void *)PROC_UPD_FULL;
        update_full->read_proc = proc_read_fb;
        update_full->write_proc = proc_write_fb;
        update_full->owner = THIS_MODULE;

        draw_screen = create_proc_entry("draw_screen", 0666, s1d13521fb_dir);
        if (draw_screen == NULL)
                return -ENOMEM;
        draw_screen->data = (void *)PROC_DRAW_SCREEN;
        draw_screen->read_proc = proc_read_fb;
        draw_screen->write_proc = proc_write_fb;
        draw_screen->owner = THIS_MODULE;

        update_by_timer = create_proc_entry("update_by_timer", 0644, s1d13521fb_dir);
        if (update_by_timer == NULL)
                return -ENOMEM;
        update_by_timer->data = (void *)PROC_UPDATE_BY_TIMER;
        update_by_timer->read_proc = proc_read_fb;
        update_by_timer->write_proc = proc_write_fb;
        update_by_timer->owner = THIS_MODULE;

        enable_update = create_proc_entry("enable_update", 0666, s1d13521fb_dir);
        if(enable_update == NULL)
            return -ENOMEM;
        enable_update->data = (void *)PROC_ENABLE_UPDATE;
        enable_update->read_proc = proc_read_fb;
        enable_update->write_proc = proc_write_fb;
        enable_update->owner = THIS_MODULE;

        waveform_update = create_proc_entry("waveform_update", 0644, s1d13521fb_dir);
        if(waveform_update == NULL)
            return -ENOMEM;
        waveform_update->data = (void *)PROC_UPDATE_WAVEFORM;
        waveform_update->read_proc = proc_read_fb;
        waveform_update->write_proc = proc_write_fb;
        waveform_update->owner = THIS_MODULE;

        return 0;
}

//-----------------------------------------------------------------------------
//
//-----------------------------------------------------------------------------
void s1d13521proc_terminate(void)
{
    if (ft_file)
        remove_proc_entry("ft",s1d13521fb_dir);
    if (reg_file)
        remove_proc_entry("regio", s1d13521fb_dir);
    if (info_file)
        remove_proc_entry("info", s1d13521fb_dir);
    if (init_file)
        remove_proc_entry("init", s1d13521fb_dir);
    if (s3c_reg_file)
        remove_proc_entry("s3creg", s1d13521fb_dir);
}

//-----------------------------------------------------------------------------
// /proc read function
//-----------------------------------------------------------------------------
static int proc_read_fb(char *page, char **start, off_t off, int count, int *eof, void *data)
{
       int len;
    u32 regval = 0x0;
    int ret = 0;

        switch ((u32)data)
        {
          default:
                len = sprintf(page, "s1d13521fb driver\n");
                break;

          case PROC_INFO:
                len = sprintf(page, "%s\n"
                        "Virtual Framebuffer Frequency: %dHz\n\n"
                        "Syntax when writing to reg:  [index=hex] [val=hex]\n"
                        "To read a register, only set index=hex and then read from reg.\n"
                        "For example, to read register 0xAB:\n"
                        "   echo index=AB > /proc/s1d13521fb/regio\n"
                        "   cat /proc/s1d13521fb/regio\n\n",
                        s1d13521fb_version
                        ,CONFIG_FB_EPSON_VIRTUAL_FRAMEBUFFER_FREQ
                        );
#ifdef CONFIG_FB_EPSON_GPIO_SMDK6410
                page += len;
                len += dump_gpio(page);
#endif
                break;

          case PROC_REG:
                len = sprintf(page, "Register I/O: REG[%0lXh]=%0lXh\n\n",
                        ProcRegIndex, (unsigned long)s1d13521if_ReadReg16(ProcRegIndex));
                break;

          case PROC_S3C_REG:
          ret = s1d13521proc_get_s3cregval(ProcS3cReg, &regval);
          if (ret)
          {
                    len = sprintf(page, "S3C_REG(%s) doesn't exist.\n", ProcS3cReg);
          }
          else
          {
                    len = sprintf(page, "S3C_REG(%s) 0x%x[0x%x]\n\n", ProcS3cReg, ProcRegIndex, regval);
          }
                break;

          case PROC_FRAME:
                len = sprintf(page,"s1d13521fb_do_refresh_display(UPD_FULL,WF_MODE_GC)\n\n");
                s1d13521fb_do_refresh_display(UPD_FULL,WF_MODE_GC, NULL);
                break;

          case PROC_INIT:
                len = 0; // sprintf(page,"init the hw...\n\n");
#ifdef CONFIG_FB_EPSON_GPIO_SMDK6410
                init_gpio();
#endif
//              s1d13521fb_InitRegisters();
//              s1d13521fb_init_display();
                break;

       case PROC_ENABLE_UPDATE:
        len = sprintf(page, "%d\n", s1d13521if_enabled());
        break;
    }
    return len;
}

//-----------------------------------------------------------------------------
// proc write function
//
// echo index=AB > /proc/s1d13521fb/regio
//-----------------------------------------------------------------------------
static int proc_write_fb(struct file *file, const char *buffer, int count, void *data)
{
    int GotRegVal;
    int len = count;
    char *buf = (char *)buffer;
    u32 pos;
    int rot_mode;
    int auto_sel;
    u16 w, h, x, y;

        #define SKIP_OVER_WHITESPACE(str,count)                         \
        {                                                               \
                while ((count > 0) && !s1d13521proc_iswhitechar(*(str)))\
                {                                                       \
                        (str)++;                                        \
                        count--;                                        \
                }                                                       \
                                                                        \
                while ((count > 0) && s1d13521proc_iswhitechar(*(str))) \
                {                                                       \
                        (str)++;                                        \
                        count--;                                        \
                }                                                       \
        }

        switch ((int)data)
        {
          case PROC_FRAME:
                dbg_info("s1d13521proc: %s(): PROC_FRAME\n",__FUNCTION__);
                s1d13521fb_do_refresh_display(UPD_FULL,WF_MODE_GC, NULL);
                break;

          case PROC_REG:
                GotRegVal = FALSE;

                while (count > 0)
                {
                        if (!strncmp(buf,"index=",6))
                                ProcRegIndex = s1d13521proc_htol(buf+6);
                        else if (!strncmp(buf,"val=",4))
                        {
                                ProcRegVal = s1d13521proc_htol(buf+4);
                                GotRegVal = TRUE;
                        }
                        else
                        {
                                count = 0;
                                break;
                        }

                        SKIP_OVER_WHITESPACE(buf,count);
                }

                if (GotRegVal)
                        s1d13521if_WriteReg16(ProcRegIndex,ProcRegVal);
                break;

          case PROC_S3C_REG:
          printk(KERN_NOTICE "PROC_S3C_REG: count(%d)\n", count);
                GotRegVal = FALSE;

                if (!strncmp(buf,"s3creg=",7))
                {
                    int cursor;

            buf += 7; count -= 7;

            memset(ProcS3cReg, 0x0, sizeof(ProcS3cReg));
            cursor = 0;
            while (cursor < count)
            {
                if ( s1d13521proc_iswhitechar(buf[cursor]) )
                {
                    ProcS3cReg[cursor] = 0x0;
                    break;
                }
                ProcS3cReg[cursor] = buf[cursor];
                if (ProcS3cReg[cursor] == 0x0)
                    break;

                cursor++;
            }
            len = strlen(ProcS3cReg);
            buf += len; count -= len;
            //printk(KERN_NOTICE "PROC_S3C_REG:strlen(buf) = %d\n", len);
                }

                while ( (count > 0) && (s1d13521proc_iswhitechar(*buf)) )
                {
            buf++;
            count--;
                }

                if (!strncmp(buf, "val=0x", 6))
                {
            ProcS3cRegVal = s1d13521proc_htol(buf+6);
            //printk(KERN_NOTICE "ProcS3cRegVal(0x%x)\n", ProcS3cRegVal);
            GotRegVal = TRUE;
                }

                if (GotRegVal)
                    s1d13521proc_set_s3cregval(ProcS3cReg, ProcS3cRegVal);

                break;

          case PROC_SET_LUT_AUTO_SELECT:
          auto_sel = local_atoi(buf, &pos);

          printk(KERN_NOTICE "PROC_SET_LUT_AUTO_SELECT(%d)\n", auto_sel);
          if (!auto_sel)
            bs_cmd_set_lut_auto_sel_mode(0);
          else
            bs_cmd_set_lut_auto_sel_mode(1);

          break;

          case PROC_SET_ROT_MODE:
          rot_mode = local_atoi(buf, &pos);

          printk(KERN_NOTICE "PROC_SET_ROT_MODE(%d)\n", rot_mode);
          bs_cmd_set_rotmode(rot_mode);    // set rotation mode

          break;

          case PROC_LD_IMG_PGM:
          printk(KERN_NOTICE "PROC_LD_IMG_PGM\n");
          bs_cmd_ld_img( DFMT );
          bs_cmd_wr_data_fb();
          bs_cmd_ld_img_end( );

          break;

          case PROC_UPD_FULL:
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
          bs_cmd_wait_dspe_trg();              /* wait for op trigger to be free */
          bs_cmd_wait_dspe_frend();            /* wait for any in progress update to finish*/
          bs_cmd_upd_full(3, GC_LUT, 0);   /* request update */
          s1d13521_unlock(s1d13521fb_data);
#endif
          break;

          case PROC_LD_IMG_AREA_PGM:
        w = S1D_DISPLAY_WIDTH;
        h = S1D_DISPLAY_HEIGHT;
        x = 0;
        y = 0;
        if (!strncmp(buf,"x=",2))
        {
            buf += 2;
            count -= 2;
            pos = buf;
            x = local_atoi(buf, &pos);
            count -= (pos - (u32)buf);

            while ( (count > 0) && (s1d13521proc_iswhitechar(*buf)) )
            {
                buf++;
                count--;
            }

            if ( (count > 2) && (!strncmp(buf,"y=",2)) )
            {
                buf += 2;
                count -= 2;
                y = local_atoi(buf,&pos);
            }
        }

        printk(KERN_NOTICE "PROC_LD_IMG_AREA_PGM x(%d), y(%d), w(%d), h(%d)\n", x, y, w, h);
        bs_cmd_ld_img_area( DFMT, x, y, w, h );
        bs_cmd_wr_data_fb();
        bs_cmd_ld_img_end( );

        break;

         case PROC_DRAW_SCREEN:
          printk(KERN_NOTICE "PROC_DRAW_SCREEN\n");
          bs_cmd_ld_img( DFMT );
          bs_cmd_wr_data_fb();
          bs_cmd_ld_img_end( );

          bs_cmd_wait_dspe_trg();              /* wait for op trigger to be free */
          bs_cmd_wait_dspe_frend();            /* wait for any in progress update to finish*/

          bs_cmd_upd_full(3, GC_LUT, 0);   /* request update */

          break;
      case PROC_UPDATE_BY_TIMER:
        printk(KERN_NOTICE "PROC_UPDATE_BY_TIMER\n");
        gDisplayChange = 1;

      case PROC_ENABLE_UPDATE:
        s1d13521if_enable(s1d13521proc_htol(buf));
        break;


      case PROC_UPDATE_WAVEFORM:
        printk(KERN_NOTICE "PROC_UPDATE_WAVEFORM\n");
        {
            char filename[1024];
            char *buffer = NULL;
            long  size = 0;
            struct file* filp;
            int ret;
            struct page *pages = NULL;
            u32 order = 0;

            memcpy(filename, buf, len);
            filename[len] = 0;
            while (len > 0)
            {
                if (filename[len] <= ' ')
                    filename[len] = 0;
                else
                    break;
                len--;
            }

            if (len <= 0)
            {
                printk(KERN_INFO "Invalid waveform file name '%s'.\n", buf);
                return;
            }
            filp = filp_open(filename, O_RDWR | O_LARGEFILE, 0);
            if (IS_ERR(filp))
            {
                printk(KERN_INFO "Unable to open '%s'.\n", filename);
                return;
            }
            size = filp->f_path.dentry->d_inode->i_size;
            if (size <= 0)
            {
                printk(KERN_INFO "Invalid wavefile '%s'\n", filename);
                filp_close(filp, current->files);
                return;
            }
            while (size > (PAGE_SIZE << order))
                order++;
            printk(KERN_INFO "Wavefile size=%d order=%d\n", size, order);

            pages = alloc_pages(GFP_KERNEL, order);
            buffer = page_address(pages);

            ret = kernel_read(filp, 0, buffer, size);
            filp_close(filp, current->files);

            if (ret != size)
            {
                printk(KERN_INFO "Failed to read '%s' size=0x%x ret=0x%x.\n", filename, size, ret);
                free_pages(buffer, order);
                return;
            }

            bs_update_wfm(buffer, size);
            free_pages(buffer, order);
        }
        break;

    }

    return len;
}

//-----------------------------------------------------------------------------
//
//-----------------------------------------------------------------------------
static int s1d13521proc_iswhitechar(int c)
{
        return ((c == ' ') || (c >= 0x09 && c <= 0x0d));
}

//-----------------------------------------------------------------------------
//
//-----------------------------------------------------------------------------
static int s1d13521proc_toupper(int c)
{
        if (c >= 'a' && c <= 'z')
                c += 'A'-'a';

        return c;
}

//-----------------------------------------------------------------------------
//
//-----------------------------------------------------------------------------
static int s1d13521proc_isxdigit(int c)
{
        if ((c >= '0' && c <= '9') ||
            (c >= 'a' && c <= 'f') ||
            (c >= 'A' && c <= 'F'))
                return 1;

        return 0;
}

//-----------------------------------------------------------------------------
//
//-----------------------------------------------------------------------------
static long s1d13521proc_htol(const char *szAscii)
{
        long lTmp;
        char ch;

        lTmp = 0;

        // skip whitespace
        while (s1d13521proc_iswhitechar(*szAscii))
                szAscii++;

        while (!s1d13521proc_iswhitechar((int) *szAscii) && ('\0' != *szAscii))
        {
                ch = (char)s1d13521proc_toupper(*szAscii);

                if (!s1d13521proc_isxdigit((int) ch))
                        return 0;

                if ((ch >= 'A') && (ch <= 'F'))
                        lTmp = lTmp * 16 + 10 + (ch - 'A');
                else
                        lTmp = lTmp * 16 + (ch - '0');

                szAscii++;
        }

        return lTmp;
}


#endif // CONFIG_FB_EPSON_PROC
