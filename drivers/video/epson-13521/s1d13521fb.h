//-----------------------------------------------------------------------------
//
// linux/drivers/video/epson/s1d13521fb.h --
// Function header for Epson S1D13521 controller frame buffer drivers.
//
// Copyright(c) Seiko Epson Corporation 2000-2008.
// All rights reserved.
//
// This file is subject to the terms and conditions of the GNU General Public
// License. See the file COPYING in the main directory of this archive for
// more details.
//
//----------------------------------------------------------------------------

#ifndef __S1D13521FB_H__
#define __S1D13521FB_H__

#include <linux/earlysuspend.h>
#include <plat/regs-gpio.h>
#include <asm/gpio.h>
#include <plat/gpio-cfg.h>
#include <linux/kernel.h>
#include <linux/fb.h>

#include "s1d13521ioctl.h"
#include S1D13xxxFB_INCLUDE

#include "bs_cmd.h"

#ifndef FALSE
  #define FALSE 0
#endif

#ifndef TRUE
  #define TRUE  (!FALSE)
#endif

//S1D13521 HW interface definition


#ifdef CONFIG_MACH_BRAVO_PROTOTYPE
//[ICS] There is not CNFX pin in Epson13521 Demo interface
//#define GPIO_CNFX       16

//Susan -- on Bravo prototype board, CNF1 is connected V3.3, always high
//#define GPIO_CNF1       S3C64XX_GPL(4)
#define GPIO_XCLKOUT  S3C64XX_GPF(14)
//#ifdef CONFIG_FB_EPSON_HRDY_OK
#define GPIO_HRDY       S3C64XX_GPO(4)
//#endif
#define GPIO_HIRQ       S3C64XX_GPO(3)

#define GPIO_HDC        S3C64XX_GPO(2)
#define GPIO_RESET_L    S3C64XX_GPP(9)

#define GPIO_HIRQ_WRONG  S3C64XX_GPN(9)
#define GPIO_HRDY_WRONG  S3C64XX_GPN(10)
#define GPIO_HRD_L_WRONG  S3C64XX_GPP(6)
#define GPIO_HWE_L_WRONG  S3C64XX_GPP(5)
#define GPIO_HCS_L      S3C64XX_GPO(1)
#endif

#ifdef  CONFIG_MACH_BRAVO_EVT1
#define GPIO_HRDY       S3C64XX_GPP(2)
#define GPIO_HIRQ       S3C64XX_GPP(5)

#define GPIO_HDC        S3C64XX_GPO(5)
#define GPIO_RESET_L    S3C64XX_GPO(4)
#endif

#define HRDY_TIMEOUT_MS 5000
#define S1D_MMAP_PHYSICAL_REG_SIZE      sizeof(S1D13XXXFB_IND_STRUCT)
#define VFB_SIZE_BYTES      ((S1D_DISPLAY_WIDTH*S1D_DISPLAY_HEIGHT*S1D_DISPLAY_BPP)/8)
#define PREV_FRAMEBUFFER (VFB_SIZE_BYTES*2)

//EINK S1D13521 specific cofig data
#define GC_LUT 15    // use lut number 15 for grayscale transitions
#define ROT_MODE       3    // rotation mode = 270 degrees
#define DFMT           3    // data format = 8 bit
#define BDR_VAL        0    // border update val

// In Indirect Mode, a copy of the framebuffer is kept in system memory.
// A timer periodically writes this copy to the "real" framebuffer in
// hardware. This copy is called a virtual framebuffer.

//----------------------------------------------------------------------------
// Global structures used by s1d13521fb frame buffer code
//----------------------------------------------------------------------------
typedef struct
{
    volatile unsigned char *RegAddr;
    unsigned RegAddrMappedSize;
    u32 VirtualFramebufferAddr;
    u32 page_order;
    int blank_mode;
    u32 pseudo_palette[16];
}FB_INFO_S1D13521;

typedef struct {
    uint32_t left;
    uint32_t top;
    uint32_t right;
    uint32_t bottom;
} s3c_rect;

struct s1d13521fb_eink_data {
    struct early_suspend early_suspend;
    int irq;
    struct work_struct work;
    uint32_t flags;
    struct mutex lock;
    struct mutex inuse_lock;
};

void s1d13521_lock(struct s1d13521fb_eink_data *eink);
void s1d13521_unlock(struct s1d13521fb_eink_data *eink);

extern struct fb_fix_screeninfo s1d13521fb_fix;
extern struct fb_info s1d13521_fb;
extern FB_INFO_S1D13521 s1d13521fb_info;
extern char *s1d13521fb_version;
extern void *s1d13521fb_current;
extern void *s1d13521fb_previous;

//-----------------------------------------------------------------------------
// Global Function Prototypes
//-----------------------------------------------------------------------------

#ifdef CONFIG_FB_EPSON_DEBUG_PRINTK
#define dbg_info(fmt, args...) do { printk(KERN_NOTICE fmt, ## args); } while (0)
#else
#define dbg_info(fmt, args...) do { } while (0)
#endif

#ifdef CONFIG_FB_EPSON_DEBUG_PRINTK
#define assert(expr) \
        if(!(expr)) { \
        printk( "Assertion failed! %s,%s,%s,line=%d\n",\
        #expr,__FILE__,__FUNCTION__,__LINE__); \
        BUG(); \
        }
#else
#define assert(expr)
#endif

static inline u32 ktime_to_ms(void)
{
    ktime_t tm = ktime_get();

    return tm.tv.sec * 1000 + tm.tv.nsec/1000000;
}

#ifdef CONFIG_FB_EPSON_DEBUG_PRINTK
static int pc_debug = CONFIG_FB_EPSON_DEBUG_PRINTK_LEVEL;
#define DEBUGP(n, x, args...) do {      \
    if (pc_debug >= (n))                \
        printk("%10u: %s:" x, ktime_to_ms(), __func__ , ## args);     \
    } while (0)
#define DEBUGP_L0(n, x, args...) do {      \
    if (pc_debug >= (n))                \
        printk("%10u: %s:" x, ktime_to_ms(), __func__ , ## args);     \
    } while (0)
#define DEBUGP_L1(n, x, args...) do {      \
    if (pc_debug >= (n))                \
        printk("%10u:   %s:" x, ktime_to_ms(), __func__ , ## args);     \
    } while (0)
#define DEBUGP_L2(n, x, args...) do {      \
    if (pc_debug >= (n))                \
        printk("%10u:     %s:" x, ktime_to_ms(), __func__ , ## args);     \
    } while (0)
#define DEBUGP_L3(n, x, args...) do {      \
    if (pc_debug >= (n))                \
        printk("%10u:        %s:" x, ktime_to_ms(), __func__ , ## args);     \
    } while (0)
#else
#define DEBUGP(n, x, args...)
#define DEBUGP_L0(n, x, args...)
#define DEBUGP_L1(n, x, args...)
#define DEBUGP_L2(n, x, args...)
#define DEBUGP_L3(n, x, args...)
#endif


#ifdef CONFIG_FB_EPSON_PROC
int  __devinit s1d13521proc_init(void);
void __devexit s1d13521proc_terminate(void);
#endif

#ifdef CONFIG_FB_EPSON_PCI
int  __devinit s1d13521pci_init(long *physicalAddress);
void __devexit s1d13521pci_terminate(void);
#endif

int  __devinit s1d13521if_InterfaceInit(FB_INFO_S1D13521 *info);
void __devexit s1d13521if_InterfaceTerminate(FB_INFO_S1D13521 *info);
void s1d13521if_ifmode( enum bs_ifm m );
int  s1d13521if_cmd(unsigned ioctlcmd,s1d13521_ioctl_cmd_params *params,int numparams);
void s1d13521if_command(int v);
void s1d13521if_data(int v);
int  s1d13521if_data_get(void);
void s1d13521if_BurstWrite16(u16 *ptr16, unsigned copysize16);
void s1d13521if_BurstRead16(u16 *ptr16, unsigned copysize16);
u16  s1d13521if_ReadReg16(u16 Index);
void s1d13521if_WriteReg16(u16 Index, u16 Value);
void s1d13521if_WriteReg16_nodebug(u16 Index, u16 Value);
void s1d13521if_wait_for_bit( int reg, int bitpos, int bitval );
int  s1d13521if_WaitForHRDY(void);


void s1d13521fb_do_refresh_display(unsigned cmd,unsigned mode, s3c_rect *changed);
void s1d13521fb_init_display(void);
void s1d13521fb_InitRegisters(void);

int s1d13521if_enabled(void);
void s1d13521if_enable(int enable);

void init_pll( void );
int s1d13521fb_print_wfm_info( void );

// Virtual eInk management
void s1d13521fb_do_area(unsigned cmd, unsigned mode, u16 x, u16 y, u16 w, u16 h);
void s1d13521fb_do_rect_area(unsigned cmd, unsigned mode, s3c_rect *real_area, u16 x, u16 y, u16 w, u16 h);

void s1d13521fb_trigger(void);
int s1d13521fb_sync_thread(void *arg);

extern void s1d13521fb_wait_for_trigger(void);
extern int s1d13521fb_wait_for_frend(void);

#endif  //__S1D13521FB_H__
