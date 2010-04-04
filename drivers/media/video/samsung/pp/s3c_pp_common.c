
/*
 * linux/drivers/video/s3c_pp_common.c
 *
 * Revision 1.0  
 *
 *
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file COPYING in the main directory of this archive for
 * more details.
 *
 *	    S3C PostProcessor driver 
 *
 */
#include <linux/init.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/types.h>
#include <linux/timer.h>
#include <linux/kernel.h>
#include <linux/fs.h>
#include <linux/platform_device.h>
#include <linux/interrupt.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <asm/uaccess.h>
#include <linux/errno.h> /* error codes */
#include <asm/div64.h>
#include <linux/mm.h>
#include <linux/tty.h>
#include <asm/io.h>
#include <asm/irq.h>
#include <mach/hardware.h>
#include <asm/uaccess.h>
#include <mach/map.h>
#include <linux/miscdevice.h>

#include <linux/version.h>
#if LINUX_VERSION_CODE == KERNEL_VERSION(2,6,16)
#include <linux/config.h>
#include <asm/arch/registers.h>
#else
#include <plat/regs-pp.h>
#endif

#include "s3c_pp_common.h"

#define PFX "s3c_pp"

// setting the source/destination color space
void set_data_format(s3c_pp_instance_context_t *pp_instance)
{
	// set the source color space
	switch(pp_instance->src_color_space)
	{
		case YC420:
			pp_instance->in_pixel_size	= 1;
			break;
		case YCBYCR:
			pp_instance->in_pixel_size	= 2;
			break;
		case YCRYCB:
			pp_instance->in_pixel_size	= 2;
			break;
		case CBYCRY:
			pp_instance->in_pixel_size	= 2;
			break;
		case CRYCBY:
			pp_instance->in_pixel_size	= 2;
			break;
		case RGB24:
			pp_instance->in_pixel_size	= 4;
			break;
		case RGB16:
			pp_instance->in_pixel_size	= 2;
			break;
		default:
			break;
	}

	// set the destination color space
	if ( DMA_ONESHOT == pp_instance->out_path ) 
	{
		switch(pp_instance->dst_color_space)
		{
			case YC420:
				pp_instance->out_pixel_size	= 1;
				break;
			case YCBYCR:
				pp_instance->out_pixel_size	= 2;
				break;
			case YCRYCB:
				pp_instance->out_pixel_size	= 2;
				break;
			case CBYCRY:
				pp_instance->out_pixel_size	= 2;
				break;
			case CRYCBY:
				pp_instance->out_pixel_size	= 2;
				break;
			case RGB24:
				pp_instance->out_pixel_size	= 4;
				break;
			case RGB16:
				pp_instance->out_pixel_size	= 2;
				break;
			default:
				break;
		}
	}
	else if ( FIFO_FREERUN == pp_instance->out_path ) 
	{
		if(pp_instance->dst_color_space == RGB30) 
		{
			pp_instance->out_pixel_size	= 4;
		} 
		else if(pp_instance->dst_color_space == YUV444) 
		{
			pp_instance->out_pixel_size	= 4;
		} 
	}

	// setting the register about src/dst data format
	set_data_format_register(pp_instance);
}


void set_src_addr(s3c_pp_instance_context_t *pp_instance)
{
	s3c_pp_buf_addr_t	buf_addr;
	unsigned int        start_pos_cb_cr;
	unsigned int        end_pos_cb_cr;

	buf_addr.offset_y		= (pp_instance->src_full_width - pp_instance->src_width) * pp_instance->in_pixel_size;
	buf_addr.start_pos_y		= (pp_instance->src_full_width*pp_instance->src_start_y+pp_instance->src_start_x)*pp_instance->in_pixel_size;
	buf_addr.end_pos_y		= pp_instance->src_width*pp_instance->src_height*pp_instance->in_pixel_size + buf_addr.offset_y*(pp_instance->src_height-1);
	buf_addr.src_frm_start_addr	= pp_instance->src_buf_addr_phy_rgb_y;
	buf_addr.src_start_y		= pp_instance->src_buf_addr_phy_rgb_y + buf_addr.start_pos_y;
	buf_addr.src_end_y		= buf_addr.src_start_y + buf_addr.end_pos_y;

	if (pp_instance->src_color_space == YC420) 
	{		
		buf_addr.offset_cb    = buf_addr.offset_cr = ((pp_instance->src_full_width - pp_instance->src_width) >> 1) * pp_instance->in_pixel_size;

		start_pos_cb_cr =   ((pp_instance->src_full_width >> 1) * (pp_instance->src_start_y >> 1) + pp_instance->src_start_x >> 1)
		                    * pp_instance->in_pixel_size;
		end_pos_cb_cr   =   ((pp_instance->src_width  >> 1) * (pp_instance->src_height >> 1)) * pp_instance->in_pixel_size
		                  + ((pp_instance->src_height >> 1) - 1) * buf_addr.offset_cb;

		buf_addr.start_pos_cb = start_pos_cb_cr;
		buf_addr.end_pos_cb   = end_pos_cb_cr;

		buf_addr.start_pos_cr = start_pos_cb_cr;
		buf_addr.end_pos_cr   = end_pos_cb_cr;

		buf_addr.src_start_cb	= pp_instance->src_buf_addr_phy_cb + buf_addr.start_pos_cb;	
		buf_addr.src_end_cb	= buf_addr.src_start_cb + buf_addr.end_pos_cb;

		buf_addr.src_start_cr	= pp_instance->src_buf_addr_phy_cr + buf_addr.start_pos_cr;
		buf_addr.src_end_cr	= buf_addr.src_start_cr + buf_addr.end_pos_cr;
	}

	set_src_addr_register(&buf_addr, pp_instance);
}

void set_dest_addr(s3c_pp_instance_context_t *pp_instance)
{
	s3c_pp_buf_addr_t	buf_addr;
	unsigned int        out_start_pos_cb_cr;
	unsigned int        out_end_pos_cb_cr;

	if ( DMA_ONESHOT == pp_instance->out_path ) 
	{
		buf_addr.offset_rgb	= (pp_instance->dst_full_width - pp_instance->dst_width)*pp_instance->out_pixel_size;
		buf_addr.start_pos_rgb	= (pp_instance->dst_full_width*pp_instance->dst_start_y + pp_instance->dst_start_x)*pp_instance->out_pixel_size;
		buf_addr.end_pos_rgb	= pp_instance->dst_width*pp_instance->dst_height*pp_instance->out_pixel_size 	\
					  + buf_addr.offset_rgb*(pp_instance->dst_height - 1);
		buf_addr.dst_start_rgb 	= pp_instance->dst_buf_addr_phy_rgb_y + buf_addr.start_pos_rgb;
		buf_addr.dst_end_rgb 	= buf_addr.dst_start_rgb + buf_addr.end_pos_rgb;

		if (pp_instance->dst_color_space == YC420) 
		{
			buf_addr.out_offset_cb		= buf_addr.out_offset_cr = ((pp_instance->dst_full_width - pp_instance->dst_width) >> 1)*pp_instance->out_pixel_size;

			out_start_pos_cb_cr = ((pp_instance->dst_full_width >> 1) * (pp_instance->dst_start_y >> 1) + pp_instance->dst_start_x >> 1)
			                      * pp_instance->out_pixel_size;

			out_end_pos_cb_cr =  ((pp_instance->dst_width  >> 1) * (pp_instance->dst_height >> 1)) * pp_instance->out_pixel_size
			                   + ((pp_instance->dst_height >> 1) - 1) * buf_addr.out_offset_cb;

			buf_addr.out_start_pos_cb	= out_start_pos_cb_cr;
			buf_addr.out_end_pos_cb		= out_end_pos_cb_cr;

			buf_addr.out_start_pos_cr	= out_start_pos_cb_cr;
			buf_addr.out_end_pos_cr		= out_end_pos_cb_cr;
		
			buf_addr.out_src_start_cb 	= pp_instance->dst_buf_addr_phy_cb + buf_addr.out_start_pos_cb;
			buf_addr.out_src_end_cb 	= buf_addr.out_src_start_cb + buf_addr.out_end_pos_cb;
			buf_addr.out_src_start_cr 	= pp_instance->dst_buf_addr_phy_cr + buf_addr.out_start_pos_cr;
			buf_addr.out_src_end_cr 	= buf_addr.out_src_start_cr + buf_addr.out_end_pos_cr;
		}

		set_dest_addr_register (&buf_addr, pp_instance);
	}
}

void set_src_next_buf_addr(s3c_pp_instance_context_t *pp_instance)
{
	s3c_pp_buf_addr_t	buf_addr;
	unsigned int        start_pos_cb_cr;
	unsigned int        end_pos_cb_cr;


	buf_addr.offset_y		= (pp_instance->src_full_width - pp_instance->src_width) * pp_instance->in_pixel_size;
	buf_addr.start_pos_y		= (pp_instance->src_full_width*pp_instance->src_start_y+pp_instance->src_start_x)*pp_instance->in_pixel_size;
	buf_addr.end_pos_y		= pp_instance->src_width*pp_instance->src_height*pp_instance->in_pixel_size + buf_addr.offset_y*(pp_instance->src_height-1);
	buf_addr.src_frm_start_addr	= pp_instance->src_next_buf_addr_phy_rgb_y;
	buf_addr.src_start_y		= pp_instance->src_next_buf_addr_phy_rgb_y + buf_addr.start_pos_y;
	buf_addr.src_end_y		= buf_addr.src_start_y + buf_addr.end_pos_y;


	if(pp_instance->src_color_space == YC420)
	{
		buf_addr.offset_cb	= buf_addr.offset_cr = ((pp_instance->src_full_width - pp_instance->src_width) >> 1) * pp_instance->in_pixel_size;

		start_pos_cb_cr =   ((pp_instance->src_full_width >> 1) * (pp_instance->src_start_y >> 1) + pp_instance->src_start_x >> 1)
			                * pp_instance->in_pixel_size;
		end_pos_cb_cr   =  ((pp_instance->src_width  >> 1) * (pp_instance->src_height >> 1)) * pp_instance->in_pixel_size
		                 + ((pp_instance->src_height >> 1) - 1) * buf_addr.offset_cb;

		buf_addr.start_pos_cb = start_pos_cb_cr;
		buf_addr.end_pos_cb   = end_pos_cb_cr;

		buf_addr.start_pos_cr = start_pos_cb_cr;
		buf_addr.end_pos_cr   = end_pos_cb_cr;

		buf_addr.src_start_cb	= pp_instance->src_next_buf_addr_phy_cb + buf_addr.start_pos_cb;	
		buf_addr.src_end_cb	= buf_addr.src_start_cb + buf_addr.end_pos_cb;

		buf_addr.src_start_cr	= pp_instance->src_next_buf_addr_phy_cr + buf_addr.start_pos_cr;
		buf_addr.src_end_cr	= buf_addr.src_start_cr + buf_addr.end_pos_cr;
	}

	set_src_next_addr_register(&buf_addr, pp_instance);	
}

static void get_pre_h_ratio(s3c_pp_instance_context_t *pp_instance, unsigned int *pre_h_ratio, unsigned int *h_shift)
{
	if(pp_instance->src_width >= (pp_instance->dst_width<<5)) {
		*pre_h_ratio = 32;
		*h_shift = 5;		
	} else if(pp_instance->src_width >= (pp_instance->dst_width<<4)) {
		*pre_h_ratio = 16;
		*h_shift = 4;		
	} else if(pp_instance->src_width >= (pp_instance->dst_width<<3)) {
		*pre_h_ratio = 8;
		*h_shift = 3;		
	} else if(pp_instance->src_width >= (pp_instance->dst_width<<2)) {
		*pre_h_ratio = 4;
		*h_shift = 2;		
	} else if(pp_instance->src_width >= (pp_instance->dst_width<<1)) {
		*pre_h_ratio = 2;
		*h_shift = 1;		
	} else {
		*pre_h_ratio = 1;
		*h_shift = 0;		
	}
}

static void get_pre_v_ratio(s3c_pp_instance_context_t *pp_instance, unsigned int *pre_v_ratio, unsigned int *v_shift)
{
	if(pp_instance->src_height >= (pp_instance->dst_height<<5)) {
		*pre_v_ratio = 32;
		*v_shift = 5;		
	} else if(pp_instance->src_height >= (pp_instance->dst_height<<4)) {
		*pre_v_ratio = 16;
		*v_shift = 4;		
	} else if(pp_instance->src_height >= (pp_instance->dst_height<<3)) {
		*pre_v_ratio = 8;
		*v_shift = 3;		
	} else if(pp_instance->src_height >= (pp_instance->dst_height<<2)) {
		*pre_v_ratio = 4;
		*v_shift = 2;		
	} else if(pp_instance->src_height >= (pp_instance->dst_height<<1)) {
		*pre_v_ratio = 2;
		*v_shift = 1;		
	} else {
		*pre_v_ratio = 1;
		*v_shift = 0;		
	}	
}


int parameters_calibration(s3c_pp_instance_context_t *pp_instance)
{
	unsigned int	pre_h_ratio, h_shift;
	unsigned int	pre_v_ratio, v_shift;
	unsigned int	calibration_src_width, calibration_src_height;
	
	
	if (pp_instance->src_width >= (pp_instance->dst_width<<6)) {
		printk(KERN_ERR "out of horizontal scale range\n");
		return -EINVAL;
	}

	if (pp_instance->src_height >= (pp_instance->dst_height<<6)) {
		printk(KERN_ERR "out of vertical scale range\n");
		return -EINVAL;
	}


	get_pre_h_ratio(pp_instance, &pre_h_ratio, &h_shift);
	get_pre_v_ratio(pp_instance, &pre_v_ratio, &v_shift);

	// Source width must be 4's multiple of PreScale_H_Ratio and source height must be 2's multiple of PreScale_V_Ratio
	calibration_src_width  = pp_instance->src_width - (pp_instance->src_width % (4 * pre_h_ratio));
	calibration_src_height = pp_instance->src_height - (pp_instance->src_height % (2 * pre_v_ratio));
	pp_instance->src_width  = calibration_src_width;
	pp_instance->src_height = calibration_src_height;

#ifdef hroh_debug
	printk("calibration_src_width  = %d\n", pp_instance->src_width);
	printk("calibration_src_height = %d\n", pp_instance->src_height);
#endif

	return 0;
}

// setting the scaling information(source/destination size)
void set_scaler(s3c_pp_instance_context_t *pp_instance)
{
	unsigned int		pre_h_ratio, h_shift;
	unsigned int		pre_v_ratio, v_shift;
	s3c_pp_scaler_info_t	scaler_info;


	get_pre_h_ratio(pp_instance, &pre_h_ratio, &h_shift);
	scaler_info.pre_h_ratio = pre_h_ratio;
	scaler_info.h_shift = h_shift;

	scaler_info.pre_dst_width = pp_instance->src_width / scaler_info.pre_h_ratio;
	scaler_info.dx = (pp_instance->src_width<<8) / (pp_instance->dst_width<<scaler_info.h_shift);


	get_pre_v_ratio(pp_instance, &pre_v_ratio, &v_shift);
	scaler_info.pre_v_ratio = pre_v_ratio;
	scaler_info.v_shift = v_shift;
	
	scaler_info.pre_dst_height = pp_instance->src_height / scaler_info.pre_v_ratio;
	scaler_info.dy = (pp_instance->src_height<<8) / (pp_instance->dst_height<<scaler_info.v_shift);
	scaler_info.sh_factor = 10 - (scaler_info.h_shift + scaler_info.v_shift);


	// setting the register about scaling information
	set_scaler_register(&scaler_info, pp_instance);
}

int cal_data_size(s3c_color_space_t color_space, unsigned int width, unsigned int height)
{
	switch(color_space) {
		case YC420:
			return (width * height * 3) >> 1;
		case YCBYCR:
			return (width * height << 1);
		case YCRYCB:
			return (width * height << 1);
		case CBYCRY:
			return (width * height << 1);
		case CRYCBY:
			return (width * height << 1);
		case RGB24:
			return (width * height << 2);
		case RGB16:
			return (width * height << 1);
		default:
			printk(KERN_ERR "Input parameter is wrong\n");
			return -EINVAL;
	}
}

int get_src_data_size(s3c_pp_instance_context_t *pp_instance)
{
	return cal_data_size ( pp_instance->src_color_space, pp_instance->src_full_width, pp_instance->src_full_height );
}

int get_dest_data_size(s3c_pp_instance_context_t *pp_instance)
{
	return cal_data_size ( pp_instance->dst_color_space, pp_instance->dst_full_width, pp_instance->dst_full_height );
}


