// $Id: bs_cmd.cpp,v 1.2 2008/04/11 06:28:10 hgates Exp $

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

extern u16  s1d13521if_ReadReg16(u16 Index);
extern void s1d13521if_WriteReg16(u16 Index, u16 Value);
extern int  s1d13521if_WaitForHRDY(void);
extern void s1d13521if_command(int v);
extern void s1d13521if_data(int v);
extern int  s1d13521if_data_get(void);
extern void s1d13521if_reset(void);
extern void s1d13521if_ifmode(enum bs_ifm m); // Does nothing


extern void __iomem *pEink;  //SROM Bank3 memory range(XM0CSn3)

bs_chip bsc =
{
    reset:          s1d13521if_reset,
    command:        s1d13521if_command,
    data:           s1d13521if_data,
    data_get:       s1d13521if_data_get,
    wait_for_ready: s1d13521if_WaitForHRDY,
    wr_reg:         s1d13521if_WriteReg16,
    rd_reg:         s1d13521if_ReadReg16,
    ifmode:         s1d13521if_ifmode
};

bool bs_hif_mode_cmd = true;

int bs_hsize = 0;
int bs_vsize = 0;

int wfm_fvsn = 0;
int wfm_luts = 0;
int wfm_mc = 0;
int wfm_trc = 0;
int wfm_eb = 0;
int wfm_sb = 0;
int wfm_wmta = 0;


//-----------------------------------------

void bs_cmd_init_cmd_set( u16 arg0, u16 arg1, u16 arg2 )
{
  if ( !bs_hif_mode_cmd ) printk(KERN_NOTICE "[%s] !!! ERROR: not in hif_mode_cmd", __FUNCTION__ );

  bsc.command( 0x00 );
  bsc.data( arg0 );
  bsc.data( arg1 );
  bsc.data( arg2 );
}


void bs_cmd_init_pll_stby( u16 cfg0, u16 cfg1, u16 cfg2 )
{
  if ( !bs_hif_mode_cmd ) printk(KERN_NOTICE "[%s] !!! ERROR: not in hif_mode_cmd", __FUNCTION__ );

  bsc.command( 0x01 );
  bsc.data( cfg0 );
  bsc.data( cfg1 );
  bsc.data( cfg2 );
}

void bs_cmd_run_sys( void )
{
  if ( !bs_hif_mode_cmd ) printk(KERN_NOTICE "[%s] !!! ERROR: not in hif_mode_cmd", __FUNCTION__ );

  bsc.command( 0x02 );
}


void bs_cmd_stby( void )
{
  if ( !bs_hif_mode_cmd ) printk(KERN_NOTICE "[%s] !!! ERROR: not in hif_mode_cmd", __FUNCTION__ );

  bsc.command( 0x04 );
}

void bs_cmd_slp( void )
{
  if ( !bs_hif_mode_cmd ) printk(KERN_NOTICE "[%s] !!! ERROR: not in hif_mode_cmd", __FUNCTION__ );

  bsc.command( 0x05 );
}

void bs_cmd_init_sys_run( void )
{
  if ( !bs_hif_mode_cmd ) printk(KERN_NOTICE "[%s] !!! ERROR: not in hif_mode_cmd", __FUNCTION__ );

  bsc.command( 0x06 );
}

void bs_cmd_init_sys_stby( void )
{
  if ( !bs_hif_mode_cmd ) printk(KERN_NOTICE "[%s] !!! ERROR: not in hif_mode_cmd", __FUNCTION__ );

  bsc.command( 0x07 );
}

void bs_cmd_init_sdram( u16 cfg0, u16 cfg1, u16 cfg2, u16 cfg3 )
{
  if ( !bs_hif_mode_cmd ) printk(KERN_NOTICE "[%s] !!! ERROR: not in hif_mode_cmd", __FUNCTION__ );

  bsc.command( 0x08 );
  bsc.data( cfg0 );
  bsc.data( cfg1 );
  bsc.data( cfg2 );
  bsc.data( cfg3 );
}

void bs_cmd_init_dspe_cfg( u16 hsize, u16 vsize, u16 sdcfg, u16 gdcfg, u16 lutidxfmt )
{
  if ( !bs_hif_mode_cmd ) printk(KERN_NOTICE "[%s] !!! ERROR: not in hif_mode_cmd", __FUNCTION__ );

  bsc.command( 0x09 );
  bsc.data( hsize );
  bsc.data( vsize );
  bsc.data( sdcfg );
  bsc.data( gdcfg );
  bsc.data( lutidxfmt );
  bs_hsize = hsize;
  bs_vsize = vsize;
  printk(KERN_NOTICE "[%s] ... hsize=%d vsize=%d\n", __FUNCTION__, bs_hsize, bs_vsize );
}

void bs_cmd_init_dspe_tmg( u16 fs, u16 fbe, u16 ls, u16 lbe, u16 pixclkcfg )
{
  if ( !bs_hif_mode_cmd ) printk(KERN_NOTICE "[%s] !!! ERROR: not in hif_mode_cmd", __FUNCTION__ );

  bsc.command( 0x0A );
  bsc.data( fs );
  bsc.data( fbe );
  bsc.data( ls );
  bsc.data( lbe );
  bsc.data( pixclkcfg );
}

void bs_cmd_set_rotmode( u16 rotmode )
{
  u16 arg = ( rotmode & 0x3 ) << 8;

  if ( !bs_hif_mode_cmd ) printk(KERN_NOTICE "[%s] !!! ERROR: not in hif_mode_cmd", __FUNCTION__ );

  bsc.command( 0x0B );
  bsc.data( arg );
}


u16 bs_cmd_rd_reg( u16 ra )
{
  if ( !bs_hif_mode_cmd ) return bsc.rd_reg( ra );

  bsc.command( 0x10 );
  bsc.data( ra );
  return bsc.data_get( );
}

void bs_cmd_wr_reg( u16 ra, u16 wd )
{
  if ( !bs_hif_mode_cmd ) { bsc.wr_reg( ra, wd ); return; }

  bsc.command( 0x11 );
  bsc.data( ra );
  bsc.data( wd );
}

#if 0
int bs_cmd_rd_irq( void )
{
  return bsc.gpio_hirq( );
}
#endif

void bs_cmd_rd_sfm( void )
{
  if ( !bs_hif_mode_cmd ) printk(KERN_NOTICE "[%s] !!! ERROR: not in hif_mode_cmd", __FUNCTION__ );

  bsc.command( 0x012 );
}

void bs_cmd_wr_sfm( u8 wd )
{
  u16 data = wd;

  if ( !bs_hif_mode_cmd ) printk(KERN_NOTICE "[%s] !!! ERROR: not in hif_mode_cmd", __FUNCTION__ );

  bsc.command( 0x13 );
  bsc.data( data );
}


void bs_cmd_end_sfm( void )
{
  if ( !bs_hif_mode_cmd ) printk(KERN_NOTICE "[%s] !!! ERROR: not in hif_mode_cmd", __FUNCTION__ );

  bsc.command( 0x14 );
}

void bs_cmd_bst_rd_sdr( u32 ma, u32 bc )
{
  if ( !bs_hif_mode_cmd ) printk(KERN_NOTICE "[%s] !!! ERROR: not in hif_mode_cmd", __FUNCTION__ );

  bsc.command( 0x1C );
  bsc.data( ma & 0xFFFF );
  bsc.data( ( ma >> 16 ) & 0xFFFF );
  bsc.data( bc & 0xFFFF );
  bsc.data( ( bc >> 16 ) & 0xFFFF );
  bsc.command( 0x11 );
  bsc.data( 0x154 );
}


void bs_cmd_bst_wr_sdr( u32 ma, u32 bc )
{
  if ( !bs_hif_mode_cmd ) printk(KERN_NOTICE "[%s] !!! ERROR: not in hif_mode_cmd", __FUNCTION__ );

  bsc.command( 0x1D );
  bsc.data( ma & 0xFFFF );
  bsc.data( ( ma >> 16 ) & 0xFFFF );
  bsc.data( bc & 0xFFFF );
  bsc.data( ( bc >> 16 ) & 0xFFFF );
  bsc.command( 0x11 );
  bsc.data( 0x154 );
}


void bs_cmd_bst_end( void )
{
  if ( !bs_hif_mode_cmd ) {
    bsc.wr_reg( 0x0142, 2 );
    return;
  }

  bsc.command( 0x1E );
}

void bs_cmd_ld_img( u16 dfmt )
{
  u16 arg = ( dfmt & 0x3 ) << 4;

  if ( !bs_hif_mode_cmd ) printk(KERN_NOTICE "[%s] !!! ERROR: not in hif_mode_cmd", __FUNCTION__ );

  bsc.command( 0x20 );
  bsc.data( arg );
  bsc.command( 0x11 );
  bsc.data( 0x154 );
}

void bs_cmd_ld_img_area( u16 dfmt, u16 x, u16 y, u16 w, u16 h )
{
  u16 arg = ( dfmt & 0x3 ) << 4;

  if ( !bs_hif_mode_cmd ) printk(KERN_NOTICE "[%s] !!! ERROR: not in hif_mode_cmd", __FUNCTION__ );

  bsc.command( 0x22 );
  bsc.data( arg );
  bsc.data( x );
  bsc.data( y );
  bsc.data( w );
  bsc.data( h );
  bsc.command( 0x11 );
  bsc.data( 0x0154 );
}

void bs_cmd_ld_img_end( void )
{
  if ( !bs_hif_mode_cmd ) { bsc.wr_reg( 0x0142, 2 ); return; }

  bsc.command( 0x23 );
}

void bs_cmd_ld_img_wait( void )
{
  if ( !bs_hif_mode_cmd ) printk(KERN_NOTICE "[%s] !!! ERROR: not in hif_mode_cmd", __FUNCTION__ );

  bsc.command( 0x24 );
}

void bs_cmd_wait_dspe_trg( void )
{
  if ( !bs_hif_mode_cmd ) printk(KERN_NOTICE "[%s] !!! ERROR: not in hif_mode_cmd", __FUNCTION__ );

  bsc.command( 0x28 );
}

void bs_cmd_wait_dspe_frend( void )
{
  if ( !bs_hif_mode_cmd ) printk(KERN_NOTICE "[%s] !!! ERROR: not in hif_mode_cmd", __FUNCTION__ );

  bsc.command( 0x29 );
}

void bs_cmd_wait_dspe_lutfree( void )
{
  if ( !bs_hif_mode_cmd ) printk(KERN_NOTICE "[%s] !!! ERROR: not in hif_mode_cmd", __FUNCTION__ );

  bsc.command( 0x2A );
}

void bs_cmd_wait_dspe_mlutfree( u16 lutmsk )
{
  if ( !bs_hif_mode_cmd ) printk(KERN_NOTICE "[%s] !!! ERROR: not in hif_mode_cmd", __FUNCTION__ );

  bsc.command( 0x2B );
  bsc.data( lutmsk );
}

void bs_cmd_rd_wfm_info( u32 ma )
{
  if ( !bs_hif_mode_cmd ) printk(KERN_NOTICE "[%s] !!! ERROR: not in hif_mode_cmd", __FUNCTION__ );

  bsc.command( 0x30 );
  bsc.data( ma & 0xFFFF );
  bsc.data( ( ma >> 16 ) & 0xFFFF );
}

void bs_cmd_upd_init( void )
{
  if ( !bs_hif_mode_cmd ) printk(KERN_NOTICE "[%s] !!! ERROR: not in hif_mode_cmd", __FUNCTION__ );

  bsc.command( 0x32 );
}

void bs_cmd_upd_full( u16 mode, u16 lutn, u16 bdrupd )
{
  u16 arg = ( ( mode & 0xF ) << 8 ) | ( ( lutn & 0xF ) << 4 ) | ( ( bdrupd & 0x1 ) << 14 );

  if ( !bs_hif_mode_cmd ) printk(KERN_NOTICE "[%s] !!! ERROR: not in hif_mode_cmd", __FUNCTION__ );

  bsc.command( 0x33 );
  bsc.data( arg );
}

void bs_cmd_upd_full_area( u16 mode, u16 lutn, u16 bdrupd, u16 x, u16 y, u16 w, u16 h )
{
  u16 arg = ( ( mode & 0xF ) << 8 ) | ( ( lutn & 0xF ) << 4 ) | ( ( bdrupd & 0x1 ) << 14 );

  if ( !bs_hif_mode_cmd ) printk(KERN_NOTICE "[%s] !!! ERROR: not in hif_mode_cmd", __FUNCTION__ );

  bsc.command( 0x34 );
  bsc.data( arg );
  bsc.data( x );
  bsc.data( y );
  bsc.data( w );
  bsc.data( h );
  return;
}

void bs_cmd_upd_part( u16 mode, u16 lutn, u16 bdrupd )
{
  u16 arg = ( ( mode & 0xF ) << 8 ) | ( ( lutn & 0xF ) << 4 ) | ( ( bdrupd & 0x1 ) << 14 );

  if ( !bs_hif_mode_cmd ) printk(KERN_NOTICE "[%s] !!! ERROR: not in hif_mode_cmd", __FUNCTION__ );

  bsc.command( 0x35 );
  bsc.data( arg );
}

void bs_cmd_upd_part_area( u16 mode, u16 lutn, u16 bdrupd, u16 x, u16 y, u16 w, u16 h )
{
  u16 arg = ( ( mode & 0xF ) << 8 ) | ( ( lutn & 0xF ) << 4 ) | ( ( bdrupd & 0x1 ) << 14 );

  if ( !bs_hif_mode_cmd ) printk(KERN_NOTICE "[%s] !!! ERROR: not in hif_mode_cmd", __FUNCTION__ );

  bsc.command( 0x36 );
  bsc.data( arg );
  bsc.data( x );
  bsc.data( y );
  bsc.data( w );
  bsc.data( h );
  return;
}

void bs_cmd_gdrv_clr( void )
{
  if ( !bs_hif_mode_cmd ) {
    int v = ( 5 << 1 ) | 1;
    bsc.wr_reg( 0x0334, v );
    return;
  }

  bsc.command( 0x37 );
}

void bs_cmd_upd_set_imgadr( u32 ma )
{
  if ( !bs_hif_mode_cmd ) printk(KERN_NOTICE "[%s] !!! ERROR: not in hif_mode_cmd", __FUNCTION__ );

  bsc.command( 0x38 );
  bsc.data( ma & 0xFFFF );
  bsc.data( ( ma >> 16 ) & 0xFFFF );
}


//-----------------------------------------


void bs_cmd_set_hif_mode_cmd( void )
{
  printk(KERN_NOTICE "[%s] ... setting hif mode to cmd\n", __FUNCTION__ );
  bsc.ifmode( BS_IFM_CMD );
  bs_hif_mode_cmd = true;
  mdelay(1000);
}

void bs_cmd_set_hif_mode_reg( void )
{
  printk(KERN_NOTICE "[%s] ... setting hif mode to reg\n", __FUNCTION__ );
  bsc.ifmode( BS_IFM_REG );
  bs_hif_mode_cmd = false;
  mdelay(1000);
}

void bs_cmd_flag_hif_mode_cmd( void )
{
  printk(KERN_NOTICE "[%s] ... flagging cmd hif mode\n", __FUNCTION__ );
  bs_hif_mode_cmd = true;
}

void bs_cmd_flag_hif_mode_reg( void )
{
  printk(KERN_NOTICE "[%s] ... flagging reg hif mode\n", __FUNCTION__ );
  bs_hif_mode_cmd = false;
}

void bs_cmd_get_disp_sizes( void )
{
  bs_hsize = bs_cmd_rd_reg( 0x306 ); // line data length
  bs_vsize = bs_cmd_rd_reg( 0x300 ); // frame data length
  printk(KERN_NOTICE "[%s] ... hsize=%d vsize=%d\n", __FUNCTION__, bs_hsize, bs_vsize );
}

void bs_cmd_print_disp_timings( void )
{
  int vsize = bs_cmd_rd_reg( 0x300 );
  int vsync = bs_cmd_rd_reg( 0x302 );
  int vblen = bs_cmd_rd_reg( 0x304 );
  int velen = ( vblen >> 8 ) & 0xFF;
  int hsize = bs_cmd_rd_reg( 0x306 );
  int hsync = bs_cmd_rd_reg( 0x308 );
  int hblen = bs_cmd_rd_reg( 0x30A );
  int helen = ( hblen >> 8 ) & 0xFF;
  vblen &= 0xFF;
  hblen &= 0xFF;
  printk(KERN_NOTICE "[%s] disp_timings: vsize=%d vsync=%d vblen=%d velen=%d\n", __FUNCTION__, vsize, vsync, vblen, velen );
  printk(KERN_NOTICE "[%s] disp_timings: hsize=%d hsync=%d hblen=%d helen=%d\n", __FUNCTION__, hsize, hsync, hblen, helen );
}

void bs_cmd_wait_for_bit( int reg, int bitpos, int bitval )
{
  while ( true ) {
    int d = bs_cmd_rd_reg( reg );
    int v = ( d >> bitpos ) & 0x1;
    if ( v == ( bitval & 0x1 ) )
        break;
    schedule_timeout_interruptible(msecs_to_jiffies(5)); //mdelay(5);
  } // while
}

void bs_cmd_set_wfm( int addr )
{
  bs_cmd_rd_wfm_info( addr );
  bs_cmd_wait_for_bit( 0x338, 0, 0 );
}

void bs_cmd_get_wfm_info( void )
{
  u16 a = bs_cmd_rd_reg( 0x354 );
  u16 b = bs_cmd_rd_reg( 0x356 );
  u16 c = bs_cmd_rd_reg( 0x358 );
  u16 d = bs_cmd_rd_reg( 0x35C );
  u16 e = bs_cmd_rd_reg( 0x35E );
  wfm_fvsn = a & 0xFF;
  wfm_luts = ( a >> 8 ) & 0xFF;
  wfm_trc = ( b >> 8 ) & 0xFF;
  wfm_mc = b & 0xFF;
  wfm_sb = ( c >> 8 ) & 0xFF;
  wfm_eb = c & 0xFF;
  wfm_wmta = d | ( e << 16 );
}

void bs_cmd_print_wfm_info( void )
{
  bs_cmd_get_wfm_info( );
  printk(KERN_NOTICE "[%s] wfm: fvsn=%d luts=%d mc=%d trc=%d eb=0x%02x sb=0x%02x wmta=%d\n",
      __FUNCTION__, wfm_fvsn, wfm_luts, wfm_mc, wfm_trc, wfm_eb, wfm_sb, wfm_wmta );
}

// Get the waveform version
int bs_cmd_print_wfm_version( void )
{
    return wfm_fvsn;
}

void bs_cmd_clear_gd( void )
{
  bs_cmd_gdrv_clr( );
  bs_cmd_wait_for_bit( 0x338, 0, 0 );
  bs_cmd_wait_for_bit( 0x338, 3, 0 );
}

int bs_cmd_get_lut_auto_sel_mode( void )
{
  int v = bs_cmd_rd_reg( 0x330 );
  return ( ( v >> 7 ) & 0x1 );
}

void bs_cmd_set_lut_auto_sel_mode( int v )
{
  int d = bs_cmd_rd_reg( 0x330 );
  if ( v & 0x1 )
    d |= 0x80;
  else
    d &= ~0x80;
  bs_cmd_wr_reg( 0x330, d );
}

void bs_auto_wfm_0(u8 threshold, u8 c_enable, u8 curr, u8 n_enable, u8 next, u8 wfm)
{
  int v = ( threshold << 8 ) |
    ( (c_enable & 1) << 7 ) |
    ( (n_enable & 1) << 6 ) |
    ( (wfm & 0xf)         );
  bs_cmd_wr_reg( 0x360, v );

  v = curr << 8 | (next & 0xff);
  bs_cmd_wr_reg( 0x362, v );
}
void bs_auto_wfm_1(u8 threshold, u8 c_enable, u8 curr, u8 n_enable, u8 next, u8 wfm)
{
  int v = ( threshold << 8 ) |
    ( (c_enable & 1) << 7 ) |
    ( (n_enable & 1) << 6 ) |
    ( (wfm & 0xf)         );
  bs_cmd_wr_reg( 0x364, v );

  v = curr << 8 | (next & 0xff);
  bs_cmd_wr_reg( 0x366, v );
}
void bs_auto_wfm_2(u8 threshold, u8 c_enable, u8 curr, u8 n_enable, u8 next, u8 wfm)
{
  int v = ( threshold << 8 ) |
    ( (c_enable & 1) << 7 ) |
    ( (n_enable & 1) << 6 ) |
    ( (wfm & 0xf)         );
  bs_cmd_wr_reg( 0x368, v );

  v = curr << 8 | (next & 0xff);
  bs_cmd_wr_reg( 0x36a, v );
}
void bs_auto_wfm_3(u8 threshold, u8 c_enable, u8 curr, u8 n_enable, u8 next, u8 wfm)
{
  int v = ( threshold << 8 ) |
    ( (c_enable & 1) << 7 ) |
    ( (n_enable & 1) << 6 ) |
    ( (wfm & 0xf)         );
  bs_cmd_wr_reg( 0x36c, v );
}

int bs_cmd_get_wfm_auto_sel_mode( void )
{
  int v = bs_cmd_rd_reg( 0x330 );
  return ( ( v >> 6 ) & 0x1 );
}

void bs_cmd_set_wfm_auto_sel_mode( int v )
{
  int d = bs_cmd_rd_reg( 0x330 );
  if ( v & 0x1 )
  {
    bs_auto_wfm_0(0x10, 0, 0, 1, 0xf0, WF_MODE_DU);
    bs_auto_wfm_1(0x10, 0, 0, 1, 0x00, WF_MODE_DU);
    bs_auto_wfm_2(0x10, 0, 0, 0, 0x00, WF_MODE_GU);
    bs_auto_wfm_3(0x00, 0, 0, 0, 0x00, WF_MODE_GC);

    d |= 0x40;
    d |= 0x04;
  }
  else
  {
    d &= ~0x40;
    d |= 0x04;
  }
  bs_cmd_wr_reg( 0x330, d );
}

void bs_cmd_wr_data( int n, u8 * wd )
{
  u16 v = 0;
  int x = 0;
  int i = 0;
  for ( i = 0; i < n; i++ ) {
    if ( x == 0 ) v = (u16) wd[i];
    else {
      v |= ( (u16) wd[i] ) << 8;
      bsc.data( v );
    }
    x = 1 - x;
  } // for i
  if ( n & 0x1 ) bsc.data( v );
}

void bs_cmd_wr_data_fb(void)
{
    int size16 = VFB_SIZE_BYTES / 2;
    u16 *pSource = (u16*) s1d13521fb_current;

    s1d13521if_BurstWrite16(pSource, size16);
}

void bs_cmd_wr_value_fb(u8 val)
{
    int size16 = VFB_SIZE_BYTES / 2;
    u16 *pSource = (u16*) s1d13521fb_current;

    memset(pSource, val, VFB_SIZE_BYTES);
    bs_cmd_ld_img(DFMT);
    s1d13521if_BurstWrite16(pSource, size16);
    bs_cmd_ld_img_end();
}

void bs_cmd_wr_data_area_fb(u16 x, u16 y, u16 w, u16 h)
{
    int i;
    int offset = y * S1D_DISPLAY_WIDTH;
    int size16 = (h * S1D_DISPLAY_WIDTH) / 2;
    u16 *pSource = (u16*) (s1d13521fb_current + offset);

    if (w == 600)
    {
        s1d13521if_BurstWrite16(pSource, size16);
    }
    else
    {
#if 0
       for (i = y; i < (y+h); i++)
        {
            u8 *pSrc = (u8*) (s1d13521fb_current + (i * S1D_DISPLAY_WIDTH));
            u16 v = 0;

            // For every column
            for (j = x; j < (x + w -1); j++)
            {
                if (x == 0)
                {
                    v = (u16) pSrc[j];
                }
                else
                {
                    v |= ((u16) pSrc[j]) << 8;
                    bsc.data(v);
                    v = 0;
                }
            }
        }
#endif
        // For every Row
        for (i = y; i < (y+h); i++)
        {
            u16 *pSrc = (u16*) (s1d13521fb_current + (i * S1D_DISPLAY_WIDTH));

            // For every column
            pSrc += x/2;
            s1d13521if_BurstWrite16(pSrc, w/2);
        }
    }
}

void bs_cmd_rd_data( int n, u8 * rd )
{
  int n2 = n / 2;
  int x = 0;
  int i = 0;
  for ( i = 0; i < n2; i++ ) {
    u16 v = bsc.data_get( );
    rd[x++] = v & 0xFF;
    rd[x++] = ( v >> 8 ) & 0xFF;
  } // for i
  if ( n & 0x1 ) rd[x++] = bsc.data_get( );
}

void bs_cmd_ld_img_data( u16 dfmt, u16 w, u16 h, u8 * img )
{
  bs_cmd_ld_img( dfmt );
  bs_cmd_wr_data( w*h, img );
  bs_cmd_ld_img_end( );
}

void bs_cmd_ld_img_area_data( u16 dfmt, u16 x, u16 y, u16 w, u16 h, u8 * img )
{
  bs_cmd_ld_img_area( dfmt, x, y, w, h );
  bs_cmd_wr_data( w*h, img );
  bs_cmd_ld_img_end( );
}

#if 0
void bs_cmd_wr_pgm_data( pgm & p )
{
  int bc = p.width( ) * p.height( );
  u16 v = 0;
  int x = 0;
  int i = 0;
  for ( i = 0; i < bc; i++ ) {
    if ( x == 0 ) v = p.data( i );
    else {
      v |= ( (u16) p.data( i ) ) << 8;
      bsc.data( v );
    }
    x = 1 - x;
  } // for i
  if ( bc & 0x1 ) bsc.data( v );
}

void bs_cmd_ld_img_pgm( u16 dfmt, const char * pgmf )
{
  pgm a_pgm;
  a_pgm.read_file( pgmf );
  bs_cmd_ld_img( dfmt );
  bs_cmd_wr_pgm_data( a_pgm );
  bs_cmd_ld_img_end( );
}

void bs_cmd_ld_img_area_pgm( u16 dfmt, u16 x, u16 y, const char * pgmf )
{
  pgm a_pgm;
  a_pgm.read_file( pgmf );
  int w = a_pgm.width( );
  int h = a_pgm.height( );
  bs_cmd_ld_img_area( dfmt, x, y, w, h );
  bs_cmd_wr_pgm_data( a_pgm );
  bs_cmd_ld_img_end( );
}

void bs_cmd_wr_ptm_data( ptm & p )
{
  int bc = p.width( ) * p.height( );
  u16 v = 0;
  int x = 0;
  int i = 0;
  for ( i = 0; i < bc; i++ ) {
    if ( x == 0 ) v = p.data( i );
    else {
      v |= ( (u16) p.data( i ) ) << 8;
      bsc.data( v );
    }
    x = 1 - x;
  } // for i
  if ( bc & 0x1 ) bsc.data( v );
}

void bs_cmd_ld_img_ptm( u16 dfmt, const char * ptmf )
{
  ptm a_ptm;
  a_ptm.read_file( ptmf );
  bs_cmd_ld_img( dfmt );
  bs_cmd_wr_ptm_data( a_ptm );
  bs_cmd_ld_img_end( );
}

void bs_cmd_ld_img_area_ptm( u16 dfmt, u16 x, u16 y, const char * ptmf )
{
  ptm a_ptm;
  a_ptm.read_file( ptmf );
  int w = a_ptm.width( );
  int h = a_ptm.height( );
  bs_cmd_ld_img_area( dfmt, x, y, w, h );
  bs_cmd_wr_ptm_data( a_ptm );
  bs_cmd_ld_img_end( );
}
#endif

void bs_cmd_rd_sdr( u32 ma, u32 bc, u8 * data )
{
  int n = 0;
  int i = 0;

  bs_cmd_bst_rd_sdr( ma, bc );

  for ( i = 0; i < (int)(bc)/2; i++ ) {
    u16 d = bsc.data_get( );
    data[n++] = d & 0xFF;
    data[n++] = ( d >> 8 ) & 0xFF;
  } // for i
  bs_cmd_bst_end( );
}

void bs_cmd_wr_sdr( u32 ma, u32 bc, u8 * data )
{
  int n = 0;
  int i = 0;

  bs_cmd_bst_wr_sdr( ma, bc );

  for ( i = 0; i < (int)(bc)/2; i++ ) {
    u16 d = data[n++];
    d |= ( data[n++] << 8 );
    bsc.data( d );
  } // for i
  bs_cmd_bst_end( );
}



#define BSC_RD_REG(r) s1d13521if_ReadReg16(r)
#define BSC_WR_REG(r, v) s1d13521if_WriteReg16_nodebug(r, v)

#define INIT_SPI_FLASH_ACC_MODE 0 // access mode select
#define INIT_SPI_FLASH_RDC_MODE 0 // read command select
#define INIT_SPI_FLASH_CLK_DIV  3  // clock divider
#define INIT_SPI_FLASH_CLK_PHS  0 // clock phase select
#define INIT_SPI_FLASH_CLK_POL  0 // clock polarity select
#define INIT_SPI_FLASH_ENB      1 // enable
#define INIT_SPI_FLASH_CTL ( \
  ( INIT_SPI_FLASH_ACC_MODE << 7 ) | \
  ( INIT_SPI_FLASH_RDC_MODE << 6 ) | \
  ( INIT_SPI_FLASH_CLK_DIV << 3 ) | \
  ( INIT_SPI_FLASH_CLK_PHS << 2 ) | \
  ( INIT_SPI_FLASH_CLK_POL << 1 ) | \
  INIT_SPI_FLASH_ENB )

#define INIT_SPI_FLASH_CS_ENB 1
#define INIT_SPI_FLASH_CSC INIT_SPI_FLASH_CS_ENB

static int sfm_cd = 0;

void test_regs( void )
{
  int d = BSC_RD_REG( 0x000 );
  printk( "[%s] ... REG[0x000] = 0x%04x\n", __FUNCTION__, d );
  d = BSC_RD_REG( 0x0002 );
  printk( "[%s] ... REG[0x002] = 0x%04x\n", __FUNCTION__, d );
  d = BSC_RD_REG( 0x0004 );
  printk( "[%s] ... REG[0x004] = 0x%04x\n", __FUNCTION__, d );
  BSC_WR_REG( 0x304, 0x0123 );
  BSC_WR_REG( 0x30A, 0x4567 );
  d = BSC_RD_REG( 0x304 );
  assert( d == 0x0123 );
  d = BSC_RD_REG( 0x30A );
  assert( d == 0x4567 );
}

void init_spi( void )
{
  BSC_WR_REG( 0x204, INIT_SPI_FLASH_CTL );
  BSC_WR_REG( 0x208, INIT_SPI_FLASH_CSC );
}

void bs_update_wfm(const char* wfm_data, int wfm_size)
{
    // Disable display updates from LCD
    s1d13521if_enable(0);
    msleep(2000);

    printk(KERN_NOTICE  "[%s] ... Before reset\n", __FUNCTION__);
    //BSC_WR_REG( 0x08, 0x1 );
    //msleep(2000);
    init_pll();

    printk(KERN_NOTICE  "[%s] ... Before BSC_WR_REG(0x106, 0x0203)\n", __FUNCTION__);
    BSC_WR_REG( 0x106, 0x0203 );
    printk(KERN_NOTICE  "[%s] ... Before init_spi\n", __FUNCTION__);
    init_spi();
    printk(KERN_NOTICE  "[%s] ... Before test_regs\n", __FUNCTION__);
    test_regs();
    printk(KERN_NOTICE  "[%s] ... Before bs_sfm_start\n", __FUNCTION__);
    bs_sfm_start();
    printk(KERN_NOTICE  "[%s] ... Before bs_sfm_write\n", __FUNCTION__);
    bs_sfm_write( 0x0886, wfm_size, wfm_data );


    printk(KERN_NOTICE  "[%s] ... Before reset\n", __FUNCTION__);
    BSC_WR_REG( 0x08, 0x1 );
    msleep(2000);
    //init_pll();
    //s1d13521fb_InitRegisters();
    //s1d13521fb_init_display();
    //s1d13521if_set_lut_auto_sel_mode(1);
    //s1d13521if_set_rotmode(3);
    //s1d13521if_enable(1);
}

void bs_sfm_start( void )
{
  int access_mode= 0;
  int enable = 1;
  int v = 0;

  sfm_cd = BSC_RD_REG( 0x0204 ); // spi flash control reg
  BSC_WR_REG( 0x0208, 0 );
  BSC_WR_REG( 0x0204, 0 ); // disable

  v = ( access_mode << 7 ) |
    ( SFM_READ_COMMAND << 6 ) |
    ( SFM_CLOCK_DIVIDE << 3 ) |
    ( SFM_CLOCK_PHASE << 2 ) |
    ( SFM_CLOCK_POLARITY << 1 ) |
    enable;
  BSC_WR_REG( 0x0204, v );
}

void bs_sfm_end( void )
{
  BSC_WR_REG( 0x0204, sfm_cd );
}

void bs_sfm_wr_byte( int data )
{
    int v = ( data & 0xFF ) | 0x100;
    BSC_WR_REG( 0x202, v );
    s1d13521if_wait_for_bit( 0x206, 3, 0 );
}

int bs_sfm_rd_byte( void )
{
    int v = 0;
    BSC_WR_REG( 0x202, 0 );
    s1d13521if_wait_for_bit( 0x206, 3, 0 );
    v = BSC_RD_REG( 0x200 );
    return ( v & 0xFF );
}

int bs_sfm_esig( void )
{
    int es = 0;
#if 0
    BSC_WR_REG( 0x208, 1 );
    bs_sfm_wr_byte( BS_SFM_DP );
    BSC_WR_REG( 0x208, 0 );
#endif
    BSC_WR_REG( 0x208, 1 );
    bs_sfm_wr_byte( BS_SFM_RES );
    bs_sfm_wr_byte( 0 );
    bs_sfm_wr_byte( 0 );
    bs_sfm_wr_byte( 0 );
    es = bs_sfm_rd_byte( );
    BSC_WR_REG( 0x208, 0 );
    return es;
}

int bs_sfm_read_status( void )
{
    int s = 0;
    BSC_WR_REG( 0x0208, 1 );
    bs_sfm_wr_byte( BS_SFM_RDSR );
    s = bs_sfm_rd_byte( );
    BSC_WR_REG( 0x0208, 0 );
    return s;
}


void bs_sfm_write_enable( void )
{
    BSC_WR_REG( 0x0208, 1 );
    bs_sfm_wr_byte( BS_SFM_WREN );
    BSC_WR_REG( 0x0208, 0 );
}

void bs_sfm_write_disable( void )
{
    BSC_WR_REG( 0x0208, 1 );
    bs_sfm_wr_byte( BS_SFM_WRDI );
    BSC_WR_REG( 0x0208, 0 );
}

void bs_sfm_erase( int addr )
{
    printk( "... erasing sector (0x%06x)\n", addr );
    bs_sfm_write_enable( );
    BSC_WR_REG( 0x0208, 1 );
    bs_sfm_wr_byte( BS_SFM_SE );
    bs_sfm_wr_byte( ( addr >> 16 ) & 0xFF );
    bs_sfm_wr_byte( ( addr >> 8 ) & 0xFF );
    bs_sfm_wr_byte( addr & 0xFF );
    BSC_WR_REG( 0x0208, 0 );
    while ( true )
    {
        int s = bs_sfm_read_status( );
        if ( ( s & 0x1 ) == 0 )
            break;
    } // while
}



void bs_sfm_read( int addr, int size, char * data )
{
    int i;
    memset(data, 0, size);
    printk( "... reading the serial flash memory (address=0x%06x, size=%d)\n", addr, size );
    BSC_WR_REG( 0x0208, 1 );
    bs_sfm_wr_byte( BS_SFM_READ );
    bs_sfm_wr_byte( ( addr >> 16 ) & 0xFF );
    bs_sfm_wr_byte( ( addr >> 8 ) & 0xFF );
    bs_sfm_wr_byte( addr & 0xFF );
    for ( i = 0; i < size; i++ )
        data[i] = bs_sfm_rd_byte( );
    BSC_WR_REG( 0x0208, 0 );
    printk( "... reading the serial flash memory --- done\n" );
}

void bs_sfm_program_page( int pa, int size, char * data )
{
    int d;
    bs_sfm_write_enable( );
    BSC_WR_REG( 0x0208, 1 );
    bs_sfm_wr_byte( BS_SFM_PP );
    bs_sfm_wr_byte( ( pa >> 16 ) & 0xFF );
    bs_sfm_wr_byte( ( pa >> 8 ) & 0xFF );
    bs_sfm_wr_byte( pa & 0xFF );
    for ( d = 0; d < BS_SFM_PAGE_SIZE; d++ ) {
        bs_sfm_wr_byte( data[d] );
    } // for d
    BSC_WR_REG( 0x0208, 0 );
    while ( true ) {
        int s = bs_sfm_read_status( );
        if ( ( s & 0x1 ) == 0 )
            break;
    } // while
}

void bs_sfm_program_sector( int sa, int size, char * data )
{
    int p;
    int pa = sa;
    printk( "... programming sector (0x%06x)\n", sa );
    for ( p = 0; p < BS_SFM_PAGE_COUNT; p++ ) {
        int y = p * BS_SFM_PAGE_SIZE;
        bs_sfm_program_page( pa, BS_SFM_PAGE_SIZE, &data[y] );
        pa += BS_SFM_PAGE_SIZE;
    } // for p
}

void bs_sfm_write( int addr, int size, const char * data )
{
    int s, i;
    int x = 0;
    char * sd = NULL;
    char * rd = NULL;
    int s1 = addr / BS_SFM_SECTOR_SIZE;
    int s2 = ( addr + size - 1 ) / BS_SFM_SECTOR_SIZE;
    sd = kmalloc(BS_SFM_SECTOR_SIZE, GFP_KERNEL);

    printk( "... writing the serial flash memory (address=0x%06x, size=%d)\n", addr, size );

    for ( s = s1; s <= s2; s++ )
    {
        int sa = s * BS_SFM_SECTOR_SIZE;
        int start = 0;
        int count = BS_SFM_SECTOR_SIZE;
        if ( s == s1 )
        {
            if ( addr > sa )
            {
                start = addr - sa;
                bs_sfm_read( sa, start, sd );
            }
        }
        if ( s == s2 )
        {
            int limit = addr + size;
            if ( ( sa + BS_SFM_SECTOR_SIZE ) > limit )
            {
                count = limit - sa;
                bs_sfm_read( limit, ( sa + BS_SFM_SECTOR_SIZE - limit ), &sd[count] );
            }
        }
        bs_sfm_erase( sa );
        for ( i = start; i < count; i++ )
        {
            assert( x < size );
            sd[i] = data[x++];
        }
        bs_sfm_program_sector( sa, BS_SFM_SECTOR_SIZE, sd );
    } // for s

    bs_sfm_write_disable( );

    rd = kmalloc(size, GFP_KERNEL);
    assert( rd != NULL );

    printk( "... verifying the serial flash memory write\n" );

    bs_sfm_read( addr, size, rd );

    for ( i = 0; i < size; i++ ) {
        if ( rd[i] != data[i] ) {
            printk( "+++++++++++++++ rd[%d]=0x%02x  data[%d]=0x%02x\n", i, rd[i], i, data[i] );
            printk( "[%s] !!! ERROR: failed to verify the flash memory write data", __FUNCTION__ );
        }
    } // for i

    kfree(rd);
    kfree(sd);

    printk( "... writing the serial flash memory --- done\n" );
}

// end of file
