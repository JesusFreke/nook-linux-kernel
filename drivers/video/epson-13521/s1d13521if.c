//-----------------------------------------------------------------------------
//
// linux/drivers/video/epson/s1d13521if.c -- frame buffer driver for Epson
// S1D13521 LCD controller.
//
// Copyright(c) Seiko Epson Corporation 2000-2008.
// All rights reserved.
//
// This file is subject to the terms and conditions of the GNU General Public
// License. See the file COPYING in the main directory of this archive for
// more details.
//
//----------------------------------------------------------------------------
//----------------------------------------------------------------------------
//
// NOTE: This file provides all indirect interface functionality
//
//-----------------------------------------------------------------------------
#include <linux/version.h>
#include <linux/kernel.h>
#include <plat/map-base.h>
#include <plat/regs-clock.h>
#include <mach/regs-mem.h>
#include <mach/regs-irq.h>
#include <asm/gpio.h>
#include <linux/gpio.h>
#include <linux/io.h>
#include <linux/delay.h>
#include "s1d13521fb.h"
// Cleanup
#define command(cmd)  s1d13521if_command(cmd)
#define data_get()    s1d13521if_data_get()
#define data(val)     s1d13521if_data(val)

//-----------------------------------------------------------------------------
//
// Local Definitions
//
//---------------------------------------------------------------------------
#define MAP_SIZE        4096
#define EPSON13521_IF_GPIO_ADDR      0x70000000  //SROM_BW

static void * _map;

//-----------------------------------------------------------------------------
//
// Function Prototypes
//
//-----------------------------------------------------------------------------
void init_gpio(void);
void set_cmd_mode(void);
unsigned int get_gpio_val(int pin);
void set_gpio_val(int pin, int val);

//-----------------------------------------------------------------------------
//
// Globals
//
//-----------------------------------------------------------------------------

extern FB_INFO_S1D13521 s1d13521fb_info;
extern void __iomem *pEink;  //SROM Bank3 memory range(XM0CSn3)

//---------------------------------------------------------------------------
//
//---------------------------------------------------------------------------
int s1d13521if_InterfaceInit(FB_INFO_S1D13521 *info)
{
        u8* RegAddr;
        RegAddr  = (unsigned char*) ioremap_nocache(EPSON13521_IF_GPIO_ADDR,MAP_SIZE);

        if (RegAddr == NULL)
        {
                printk(KERN_ERR "s1d13521if_InterfaceInit (Gumstix): ioremap_nocache failed\n");
                return -EINVAL;
        }

        dbg_info("s1d13521ifgpio: %s():: RegAddr %x\n", __FUNCTION__,(unsigned int)RegAddr);

        info->RegAddr = RegAddr;
        info->RegAddrMappedSize = MAP_SIZE;
        _map = (void*)info->RegAddr;

        init_gpio();
        s1d13521if_ifmode(BS_IFM_CMD); // command mode, reset the chip
        //set_cmd_mode(); // command mode, reset the chip
        return 0;
}

//---------------------------------------------------------------------------
//
//---------------------------------------------------------------------------
void s1d13521if_InterfaceTerminate(FB_INFO_S1D13521 *info)
{
}

//---------------------------------------------------------------------------
//
//---------------------------------------------------------------------------
#ifdef CONFIG_FB_EPSON_HRDY_OK

static int wait_for_ready(void)
{
    int cnt = HRDY_TIMEOUT_MS;
    unsigned int d = get_gpio_val(GPIO_HRDY);

    while ( (d == 0) && (--cnt > 0) )
    {
        if (cnt > (HRDY_TIMEOUT_MS - 5))
            mdelay(1);
        else
            mdelay(10);

        d = get_gpio_val(GPIO_HRDY);
    }

    if (cnt)
        ; //printk(KERN_NOTICE  "s1d13521if_cmd: wait_for_ready: GPIO_HRDY(ready %d)\n",cnt);
    else
    {
        printk(KERN_NOTICE "%s(): GPIO_HRDY not ready!!!\n", __FUNCTION__);
        return -1;
    }
    return 0;
}

#endif

//---------------------------------------------------------------------------
//
//---------------------------------------------------------------------------
int s1d13521if_WaitForHRDY(void)
{
#ifndef CONFIG_FB_EPSON_HRDY_OK
        {
        int cnt = HRDY_TIMEOUT_MS;
        u16 reg;

        // The host must not issue any new command until HRDY is asserted.
        // If HRDY is not used, the host can poll the Sequence Controller Busy Status
        // bit in REG[000Ah] using the RD_REG command

        // If the HRDY is not used, poll Sequence Controller Busy Status
        set_gpio_val(GPIO_HDC, 0);
        *((u16 *)pEink) = (u16)(RD_REG);
        set_gpio_val(GPIO_HDC, 1);
        //command(RD_REG);
        data(0x0A);                     // Register address

        // Loop while host interface busy bit is set...
        reg = data_get();
        while (reg & 0x20)
        {
                if (--cnt <= 0)         // Avoid endless loop
                        break;

                mdelay(1);
                reg = data_get();
        }

        if (cnt <= 0)
        {
                printk(KERN_ERR "%s(): I/F busy bit stuck, reg = 0x%04x\n", __FUNCTION__, reg);
                return -1;
        }

        return 0;
        }
#else
        return wait_for_ready();
#endif
}

//---------------------------------------------------------------------------
//
//
//---------------------------------------------------------------------------
u16 s1d13521if_ReadReg16(u16 Index)
{
        u16 Value;

        lock_kernel();
        command(RD_REG);
        data(Index);
        Value = data_get();
        unlock_kernel();

        //dbg_info("s1d13521ifgpio: %s(): Reg[%02xh]=%02xh\n",__FUNCTION__, Index,Value);
        return Value;
}

//---------------------------------------------------------------------------
//
//
//---------------------------------------------------------------------------
void s1d13521if_WriteReg16(u16 Index, u16 Value)
{
        //dbg_info("%s(): %02x,%02x\n",__FUNCTION__,Index,Value);

        lock_kernel();
        command(WR_REG);
        data(Index);                    // Register address
        data(Value);                    // Register value
        unlock_kernel();
}

void s1d13521if_WriteReg16_nodebug(u16 Index, u16 Value)
{
        lock_kernel();
        command(WR_REG);
        data(Index);                    // Register address
        data(Value);                    // Register value
        unlock_kernel();
}

void s1d13521if_BurstWrite16(u16 *ptr16, unsigned copysize16)
{
    while (copysize16-- > 0)
    {
        *(u16 *)pEink = *ptr16++;
    }
}

//---------------------------------------------------------------------------
//
//---------------------------------------------------------------------------
void s1d13521if_BurstRead16(u16 *ptr16, unsigned copysize16)
{
    while (copysize16-- > 0)
    {
        *ptr16++ = *((u16 *)pEink);
    }
}

//---------------------------------------------------------------------------
//
//---------------------------------------------------------------------------

void s1d13521if_command(int v)
{
    s1d13521if_WaitForHRDY();
    set_gpio_val(GPIO_HDC, 0);
    *((u16 *)pEink) = (u16)(v);
    set_gpio_val(GPIO_HDC, 1);
}

//---------------------------------------------------------------------------
//
//---------------------------------------------------------------------------
void s1d13521if_data(int v)
{
    s1d13521if_WaitForHRDY();
    *((u16 *)pEink) = (u16)(v);
}

//---------------------------------------------------------------------------
// The data bus direction is already set to input
//---------------------------------------------------------------------------
int s1d13521if_data_get(void)
{
    int d;

    s1d13521if_WaitForHRDY();
    d = (int)(*((u16 *)pEink));

    return d;
}

void s1d13521if_reset( void )
{
    set_gpio_val( GPIO_RESET_L, 0 );
    mdelay(1000);
    set_gpio_val( GPIO_RESET_L, 1 );
    printk(KERN_NOTICE "[%s] Reset done...\n", __FUNCTION__);
}

void s1d13521if_ifmode( enum bs_ifm m )
{
#if 0
    if ( m == BS_IFM_REG )
        set_gpio_val( GPIO_CNF1, 0 );
    else
        set_gpio_val( GPIO_CNF1, 1 );
#endif
    int ready = 1;
    while(ready != 0) {
        s1d13521if_reset();
        ready = s1d13521if_WaitForHRDY();
    }
    mdelay(1000);
    printk(KERN_NOTICE "Command mode set...\n");
}

#if 0
void set_cmd_mode(void)
{
    dbg_info("s1d13521ifgpio: %s():\n",__FUNCTION__);

    // reset pulse
    set_gpio_val(GPIO_RESET_L, 0);
    mdelay(1000);  //demo is 1s
    set_gpio_val(GPIO_RESET_L, 1);
    printk(KERN_NOTICE "[%s] Reset done...\n", __FUNCTION__);

    s1d13521if_WaitForHRDY();

    mdelay(500); //demo is 1s
    printk(KERN_NOTICE "return from set_cmd_mode\n");
    printk(KERN_NOTICE "return from set_cmd_mode\n");
    printk(KERN_NOTICE "return from set_cmd_mode\n");
    printk(KERN_NOTICE "return from set_cmd_mode\n");
    printk(KERN_NOTICE "return from set_cmd_mode\n");
    printk(KERN_NOTICE "return from set_cmd_mode\n");
}
#endif
