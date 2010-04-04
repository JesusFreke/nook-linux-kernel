/*
 * Project Name JPEG DRIVER IN Linux
 * Copyright  2007 Samsung Electronics Co, Ltd. All Rights Reserved. 
 *
 * This software is the confidential and proprietary information
 * of Samsung Electronics  ("Confidential Information").   
 * you shall not disclose such Confidential Information and shall use
 * it only in accordance with the terms of the license agreement
 * you entered into with Samsung Electronics 
 *
 * This file implements JPEG driver.
 *
 * @name JPEG DRIVER MODULE Module (JPGDriver.h)
 * @author Jiun Yu (jiun.yu@samsung.com)
 * @date 05-07-07
 */

#ifndef __JPEG_DRIVER_H__
#define __JPEG_DRIVER_H__

#define MAX_INSTANCE_NUM	2

#define IOCTL_JPG_DECODE				0x00000002
#define IOCTL_JPG_ENCODE				0x00000003
//#define IOCTL_JPG_GET_STRBUF			0x00000004
#define IOCTL_JPG_SET_STRBUF			0x00000004
//#define IOCTL_JPG_GET_FRMBUF			0x00000005
#define IOCTL_JPG_SET_FRMBUF			0x00000005
//#define IOCTL_JPG_GET_THUMB_STRBUF		0x0000000A
#define IOCTL_JPG_SET_THUMB_STRBUF		0x0000000A
//#define IOCTL_JPG_GET_THUMB_FRMBUF		0x0000000B
#define IOCTL_JPG_SET_THUMB_FRMBUF		0x0000000B
#define IOCTL_JPG_GET_PHY_FRMBUF		0x0000000C
#define IOCTL_JPG_GET_PHY_THUMB_FRMBUF	0x0000000D

#endif /*__JPEG_DRIVER_H__*/
