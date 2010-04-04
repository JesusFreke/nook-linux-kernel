/* linux/arch/arm/plat-s3c64xx/include/plat/reserved_mem.h
 *
 * Copyright (c) 2007 Samsung Electronics
 *		      http://www.samsungsemi.com/
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * Modifications 2009 : Intrinsyc Software, Inc on behalf of Barnes and Noble
 * Portions of this code copyright (c) 2009 Barnes and Noble, Inc
*/


#ifndef _ASM_ARM_ARCH_RESERVED_MEM_H
#define _ASM_ARM_ARCH_RESERVED_MEM_H

#include <linux/types.h>
#include <linux/list.h>
#include <asm/setup.h>

#define DRAM_END_ADDR (PHYS_OFFSET+PHYS_SIZE)


#ifdef CONFIG_VIDEO_G3D
#define RESERVED_MEM_MFC	(4 * 1024 * 1024)
#define RESERVED_PMEM_PICTURE	(RESERVED_MEM_MFC)
#define RESERVED_PMEM_STREAM	(1 * 1024 * 1024)
#define RESERVED_PMEM_PREVIEW	(3 * 1024 * 1024)
#define RESERVED_PMEM_JPEG	(RESERVED_PMEM_PREVIEW)
#define RESERVED_PMEM_RENDER	(8 * 1024 * 1024)
#define RESERVED_PMEM_GPU1	(10 * 1024 * 1024)
#define RESERVED_PMEM_G3D	(12 * 1024 * 1024)
#define RESERVED_PMEM		(0 * 1024 * 1024)
#else
#define RESERVED_MEM_MFC	(4 * 1024 * 1024)
#define RESERVED_PMEM_PICTURE	(RESERVED_MEM_MFC)
#define RESERVED_PMEM_STREAM	(1 * 1024 * 1024)
#define RESERVED_PMEM_PREVIEW	(3 * 1024 * 1024)
#define RESERVED_PMEM_JPEG	(RESERVED_PMEM_PREVIEW)
#define RESERVED_PMEM_RENDER	(8 * 1024 * 1024)
#define RESERVED_PMEM_GPU1	(0 * 1024 * 1024)
#define RESERVED_PMEM_G3D	(0 * 1024 * 1024)
#define RESERVED_PMEM		(0 * 1024 * 1024)
#endif /*  CONFIG_VIDEO_G3D */

#if defined(CONFIG_RESERVED_MEM_CMM_JPEG_MFC_POST_CAMERA)
#define MFC_RESERVED_MEM_START		(DRAM_END_ADDR - RESERVED_MEM_MFC)
#define PICTURE_RESERVED_PMEM_START	(MFC_RESERVED_MEM_START)
#define STREAM_RESERVED_PMEM_START	(PICTURE_RESERVED_PMEM_START - RESERVED_PMEM_STREAM)
#define PREVIEW_RESERVED_PMEM_START	(STREAM_RESERVED_PMEM_START - RESERVED_PMEM_STREAM)
#define JPEG_RESERVED_PMEM_START	(PREVIEW_RESERVED_PMEM_START)
#define RENDER_RESERVED_PMEM_START	(PREVIEW_RESERVED_PMEM_START - RESERVED_PMEM_RENDER)
#define GPU1_RESERVED_PMEM_START	(RENDER_RESERVED_PMEM_START - RESERVED_PMEM_GPU1)
#define G3D_RESERVED_PMEM_START		(GPU1_RESERVED_PMEM_START - RESERVED_PMEM_G3D)
#define RESERVED_PMEM_START		(G3D_RESERVED_PMEM_START - RESERVED_PMEM)
#define PHYS_UNRESERVED_SIZE		(RESERVED_PMEM_START - PHYS_OFFSET)
#else
#define PHYS_UNRESERVED_SIZE		(DRAM_END_ADDR - PHYS_OFFSET)

#endif

struct s3c6410_pmem_setting{
        resource_size_t pmem_start;
        resource_size_t pmem_size;
	resource_size_t pmem_g3d_start;
	resource_size_t pmem_g3d_size;
        resource_size_t pmem_gpu1_start;
        resource_size_t pmem_gpu1_size;
        resource_size_t pmem_render_start;
        resource_size_t pmem_render_size;
        resource_size_t pmem_stream_start;
        resource_size_t pmem_stream_size;
        resource_size_t pmem_preview_start;
        resource_size_t pmem_preview_size;
        resource_size_t pmem_picture_start;
        resource_size_t pmem_picture_size;
        resource_size_t pmem_jpeg_start;
        resource_size_t pmem_jpeg_size;
};
 
void s3c6410_add_mem_devices (struct s3c6410_pmem_setting *setting);

#endif /* _ASM_ARM_ARCH_RESERVED_MEM_H */

