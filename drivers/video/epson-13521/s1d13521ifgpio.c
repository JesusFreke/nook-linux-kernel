//-----------------------------------------------------------------------------
//
// linux/drivers/video/epson/s1d13521ifgpio.c -- Gumstix specific GPIO
// interface code for Epson S1D13521 LCD controllerframe buffer driver.
//
//
// Copyright(c) Seiko Epson Corporation 2008.
// All rights reserved.
//
// Code based on E-Ink demo source code.
//
// Modifications 2009 : Intrinsyc Software, Inc on behalf of Barnes and Noble
// Portions of this code copyright (c) 2009 Barnes and Noble, Inc
//
// This file is subject to the terms and conditions of the GNU General Public
// License. See the file COPYING in the main directory of this archive for
// more details.
//
//----------------------------------------------------------------------------
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
#include <mach/bravo_gpio.h>
#include "s1d13521fb.h"

// Cleanup
#define command(cmd)  s1d13521if_command(cmd)
#define data_get()    s1d13521if_data_get()
#define data(val)     s1d13521if_data(val)

//---------------------------------------------------------------------------
//
// Local Definitions
//
//---------------------------------------------------------------------------
void __iomem *pEink;  //SROM Bank3 memory range(XM0CSn3)

#define MAP_SIZE        4096

// gpio registers

#define REG(r) (*(volatile unsigned int *)((char*)_map+(r)))

#define EPSON13521_IF_GPIO_ADDR      0x70000000  //SROM_BW

#define GPFCON      S3C64XX_GPF_BASE
#define GPFDAT      (S3C64XX_GPF_BASE + 0x4)
#define GPFPUD      (S3C64XX_GPF_BASE + 0x8)

#define GPNCON      S3C64XX_GPN_BASE
#define GPNDAT      (S3C64XX_GPN_BASE + 0x4)
#define GPNPUD      (S3C64XX_GPN_BASE + 0x8)

#define GPOCON      S3C64XX_GPO_BASE
#define GPODAT      (S3C64XX_GPO_BASE + 0x4)
#define GPOPUD      (S3C64XX_GPO_BASE + 0x8)
#define GPOSLP      (S3C64XX_GPO_BASE + 0xC)
#define GPOPUDSLP   (S3C64XX_GPO_BASE + 0x10)

#define GPPCON      S3C64XX_GPP_BASE
#define GPPDAT      (S3C64XX_GPP_BASE + 0x4)
#define GPPPUD      (S3C64XX_GPP_BASE + 0x8)
#define GPPSLP      (S3C64XX_GPP_BASE + 0xC)
#define GPPPUDSLP   (S3C64XX_GPP_BASE + 0x10)

#define ASSERT_13521 //
//---------------------------------------------------------------------------
//
// Local Globals
//
//---------------------------------------------------------------------------
static int HDB_DIR = 0;

//---------------------------------------------------------------------------
//
// Local Function Prototypes
//
//---------------------------------------------------------------------------
void init_gpio(void);
void set_cmd_mode(void);
unsigned int  get_gpio_val(int pin);
void set_gpio_val(int pin, int val);

static void set_gpio_mode(int pin);
static void set_gpio_dir(int pin, int val);
static void gpio_hdb_dir(int v);

#if defined(CONFIG_FB_EPSON_DEBUG_PRINTK) || defined (CONFIG_FB_EPSON_PROC)
static unsigned long get_gpio_mode(int pin);
static int get_gpio_dir(int pin);
#endif

void s1d13521if_wait_for_bit( int reg, int bitpos, int bitval )
{
    int d,v;
    while ( 1 )
    {
        d = s1d13521if_ReadReg16( reg );
        v = ( d >> bitpos ) & 0x1;

        if ( v == ( bitval & 0x1 ) )
                break;
        schedule_timeout_interruptible(msecs_to_jiffies(5));
    } // while
}

const int INIT_PWR_SAVE_MODE = 0x0000;

const int INIT_PLL_CFG_0 = 0x0004;
const int INIT_PLL_CFG_1 = 0x2948;
const int INIT_PLL_CFG_2 = 0x0040;
const int INIT_CLK_CFG   = 0x0000;

#define INIT_SPI_FLASH_ACC_MODE 0 // access mode select
#define INIT_SPI_FLASH_RDC_MODE 0 // read command select
#define INIT_SPI_FLASH_CLK_DIV  3  // clock divider
#define INIT_SPI_FLASH_CLK_PHS  0 // clock phase select
#define INIT_SPI_FLASH_CLK_POL  0 // clock polarity select
#define INIT_SPI_FLASH_ENB      1 // enable
const int INIT_SPI_FLASH_CTL =
  ( INIT_SPI_FLASH_ACC_MODE << 7 ) |
  ( INIT_SPI_FLASH_RDC_MODE << 6 ) |
  ( INIT_SPI_FLASH_CLK_DIV << 3 ) |
  ( INIT_SPI_FLASH_CLK_PHS << 2 ) |
  ( INIT_SPI_FLASH_CLK_POL << 1 ) | INIT_SPI_FLASH_ENB;

//const int INIT_SPI_FLASH_CS_ENB = 1;
//const int INIT_SPI_FLASH_CSC = INIT_SPI_FLASH_CS_ENB;


void init_pll( void )
{
    int v;
    s1d13521if_WriteReg16( 0x006, 0x0000 );
    s1d13521if_WriteReg16( 0x010, INIT_PLL_CFG_0 );
    s1d13521if_WriteReg16( 0x012, INIT_PLL_CFG_1 );
    s1d13521if_WriteReg16( 0x014, INIT_PLL_CFG_2 );
    s1d13521if_WriteReg16( 0x016, INIT_CLK_CFG );
    s1d13521if_WriteReg16( 0x018, 7 );
#if 0
    v = s1d13521if_ReadReg16( 0x00A );
    while ( ( v & 0x1 ) == 0 )
        v = s1d13521if_ReadReg16( 0x00A );
#endif
    mdelay(200);
    v = s1d13521if_ReadReg16( 0x006 );
    s1d13521if_WriteReg16( 0x006, v & ~0x1 );
}

void init_HDB_SROM_DBUS(void)
{
    unsigned int tmp;

    tmp = __raw_readl(S3C64XX_SROM_BW);
    tmp &= ~(S3C64XX_SROM_BW_BYTE_ENABLE0_MASK | S3C64XX_SROM_BW_WAIT_ENABLE0_MASK |
        S3C64XX_SROM_BW_DATA_WIDTH0_MASK);
    tmp |= S3C64XX_SROM_BW_DATA_WIDTH0_16BIT;  //Disable XM0 WaitEnable0

    __raw_writel(tmp, S3C64XX_SROM_BW);

    __raw_writel(S3C64XX_SROM_BCn_TACS(1) | S3C64XX_SROM_BCn_TCOS(2) |
            S3C64XX_SROM_BCn_TACC(10) | S3C64XX_SROM_BCn_TCOH(2) |
            S3C64XX_SROM_BCn_TCAH(1) |S3C64XX_SROM_BCn_PMC_NORMAL, S3C64XX_SROM_BC0);

    pEink = ioremap(0x10000000, 0x10000);

    // Output 1 on CS and 0 on XM0ADDR and High Z everyting else.
    __raw_writel(0x00cfd0f4, S3C64XX_MEM0CONSLP0);
}

void init_gpio(void)
{
    volatile u32 regval;

    dbg_info("s1d13521ifgpio: %s():\n",__FUNCTION__);

#ifdef CONFIG_MACH_BRAVO_EVT1
    //Configure OSC_EN and CLKI pin
    //CLKI(GPP1 -- INPUT/non-PUD?)
    //OSC_EN (GPO9 OUTPUT, HIGH, PullUp)

    s3c_gpio_cfgpin(S3C64XX_GPO(9), S3C_GPIO_OUTPUT);
    s3c_gpio_setpull(S3C64XX_GPO(9), S3C_GPIO_PULL_UP);
    s3c_gpio_slp_cfgpin(S3C64XX_GPO(9), S3C_GPIO_SLP_OUT0);
    s3c_gpio_slp_setpull_updown(S3C64XX_GPO(9), S3C_GPIO_PULL_DOWN);
    s3c_gpdat_setval(S3C64XX_GPO(9), 1); // Enable it

    mdelay(1000);

    // GPO5 Configure HDC
    // ==================
    s3c_gpio_cfgpin(GPIO_HDC, S3C_GPIO_OUTPUT);
    s3c_gpio_setpull(GPIO_HDC, S3C_GPIO_PULL_NONE);
    s3c_gpio_slp_cfgpin(GPIO_HDC, S3C_GPIO_SLP_OUT1);
    s3c_gpio_slp_setpull_updown(GPIO_HDC, S3C_GPIO_PULL_UP);

    // GPO4 Configure RESET_L
    // =======================
    s3c_gpio_cfgpin(GPIO_RESET_L, S3C_GPIO_OUTPUT);
    s3c_gpio_setpull(GPIO_RESET_L, S3C_GPIO_PULL_NONE);
    s3c_gpio_slp_cfgpin(GPIO_RESET_L, S3C_GPIO_SLP_OUT1);
    s3c_gpio_slp_setpull_updown(GPIO_RESET_L, S3C_GPIO_PULL_UP);

    // GPP2 Configure HRDY
    // ===================
    s3c_gpio_cfgpin(GPIO_HRDY, S3C_GPIO_INPUT);
    s3c_gpio_setpull(GPIO_HRDY, S3C_GPIO_PULL_NONE);
    s3c_gpio_slp_cfgpin(GPIO_HRDY, S3C_GPIO_SLP_INPUT);
    s3c_gpio_slp_setpull_updown(GPIO_HRDY, S3C_GPIO_PULL_NONE);

#endif

    //Configure SROM BANK3 control registers
    init_HDB_SROM_DBUS();

    s3c_gpio_cfgpin(GPIO_HIRQ, S3C_GPIO_INPUT);
    s3c_gpio_setpull(GPIO_HIRQ, S3C_GPIO_PULL_NONE);

    ASSERT_13521( get_gpio_dir( GPIO_HDC ) == 1 );
    ASSERT_13521( get_gpio_dir( GPIO_RESET_L ) == 1 );
    ASSERT_13521( get_gpio_dir( GPIO_HRDY ) == 0 );
    ASSERT_13521( get_gpio_dir( GPIO_HIRQ ) == 0 );

    s3c_gpdat_setval(GPIO_RESET_L, 0);
    mdelay(1000);
    s3c_gpdat_setval(GPIO_HDC, 0); //CMD mode
    s3c_gpdat_setval(GPIO_RESET_L, 1 );
    printk(KERN_NOTICE "[%s] Reset done...\n", __FUNCTION__);

    get_gpio_val(GPIO_HDC);
    get_gpio_val(GPIO_RESET_L);

    schedule_timeout(HZ);  //Sleep for 1 second
}

//---------------------------------------------------------------------------
//
//---------------------------------------------------------------------------
void set_gpio_val(int pin, int val)
{
    if (val & 0x1)
    {
        s3c_gpdat_setval(pin, 1);
    }
    else
    {
        s3c_gpdat_setval(pin, 0);
    }
}

//---------------------------------------------------------------------------
//
//---------------------------------------------------------------------------
void set_gpio_dir(int pin, int val)
{
    if (val)
        s3c_gpio_cfgpin(pin, S3C_GPIO_OUTPUT);
    else
        s3c_gpio_cfgpin(pin, S3C_GPIO_INPUT);
}

//---------------------------------------------------------------------------
//
//---------------------------------------------------------------------------

void set_gpio_mode(int pin)
{
    s3c_gpio_cfgpin(pin,S3C_GPIO_OUTPUT);
}

#if defined(CONFIG_FB_EPSON_DEBUG_PRINTK) || defined (CONFIG_FB_EPSON_PROC)
//---------------------------------------------------------------------------
//
//---------------------------------------------------------------------------
unsigned long get_gpio_mode(int pin)
{
    unsigned int regval = 0;

    s3c_gpio_getcfg(pin, &regval);
    printk(KERN_NOTICE "pin(%d) -- config(0x%x)\n", pin, regval);

    return regval;
}

//---------------------------------------------------------------------------
//
//---------------------------------------------------------------------------
int get_gpio_dir(int pin)
{
    unsigned int regval = 0;

    s3c_gpio_getcfg(pin, &regval);
    printk(KERN_NOTICE "pin(%d) -- config(0x%x)\n", pin, regval);

    return regval;
}
#endif

//---------------------------------------------------------------------------
// Set the data bus direction:
//      val = 1 -> output
//      val = 0 -> input
//---------------------------------------------------------------------------
void gpio_hdb_dir(int val)  //ICS version
{
    int flags;
    unsigned long GPKCON_val = 0x0;
    void __iomem *reg0 = S3C64XX_GPKCON;
    void __iomem *reg1 = S3C64XX_GPKCON1;
    unsigned long regval = 0x0;

    if (val & 0x1)  //output
        GPKCON_val = 0x11111111;
    else
        GPKCON_val = 0x00000000;

    local_irq_save(flags);

    __raw_writel(GPKCON_val, reg0);
    __raw_writel(GPKCON_val, reg1);
    regval = __raw_readl(reg0);
    //printk(KERN_NOTICE "gpio_hdb_dir:GPKCON(0x%x)\n", regval);
    regval = __raw_readl(reg1);
    //printk(KERN_NOTICE "gpio_hdb_dir:GPKCON1(0x%x)\n", regval);

    HDB_DIR = (val&0x1);

    local_irq_restore(flags);
}

unsigned int get_gpio_val(int pin)
{
    unsigned int val = 0x0;

    s3c_gpdat_getval(pin, &val);

    return val;
}

#ifdef CONFIG_FB_EPSON_PROC
typedef struct
{
        const int gpio;
        const char *gpiostr;
} GPIONAMEST;

int dump_gpio(char *buf)
{
static GPIONAMEST aGpio[] =
        {
        //{GPIO_CNFX,     "GPIO_CNFX   "},
        //{GPIO_CNF1,     "GPIO_CNF1   "},
#ifdef CONFIG_FB_EPSON_HRDY_OK
        {GPIO_HRDY,     "GPIO_HRDY   "},
#endif
        {GPIO_HDC,      "GPIO_HDC    "},
        {GPIO_RESET_L,  "GPIO_RESET_L"},
        //{GPIO_HRD_L,    "GPIO_HRD_L  "},
        //{GPIO_HWE_L,    "GPIO_HWE_L  "},
        //{GPIO_HCS_L,    "GPIO_HCS_L  "},
        {GPIO_HIRQ,     "GPIO_HIRQ   "},
        //{GPIO_HDB+0,    "GPIO_HDB_0  "},
        //{GPIO_HDB+1,    "GPIO_HDB_1  "},
        //{GPIO_HDB+2,    "GPIO_HDB_2  "},
        //{GPIO_HDB+3,    "GPIO_HDB_3  "},
        //{GPIO_HDB+4,    "GPIO_HDB_4  "},
        //{GPIO_HDB+5,    "GPIO_HDB_5  "},
        //{GPIO_HDB+6,    "GPIO_HDB_6  "},
        //{GPIO_HDB+7,    "GPIO_HDB_7  "},
        //{GPIO_HDB+8,    "GPIO_HDB_8  "},
        //{GPIO_HDB+9,    "GPIO_HDB_9  "},
        //{GPIO_HDB+10,   "GPIO_HDB_A  "},
        //{GPIO_HDB+11,   "GPIO_HDB_B  "},
        //{GPIO_HDB+12,   "GPIO_HDB_C  "},
        //{GPIO_HDB+13,   "GPIO_HDB_D  "},
        //{GPIO_HDB+14,   "GPIO_HDB_E  "},
        //{GPIO_HDB+15,   "GPIO_HDB_F  "}
        };

        int i;
        char *bufin = buf;

        buf +=sprintf(buf,"GPIO        MODE  DIR  VAL\n");
        buf +=sprintf(buf,"--------------------------\n");

        for (i = 0; i < sizeof(aGpio)/sizeof(aGpio[0]); i++)
        {
                buf +=sprintf(buf,"%s  %d    %d    %d\n",aGpio[i].gpiostr,
                        get_gpio_mode(aGpio[i].gpio),
                        get_gpio_dir(aGpio[i].gpio),
                        get_gpio_val(aGpio[i].gpio));
        }

        return strlen(bufin);
}

#endif //CONFIG_FB_EPSON_PROC

