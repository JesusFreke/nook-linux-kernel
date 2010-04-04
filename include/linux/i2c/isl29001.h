/*
 *  isl29001.h - Linux kernel modules for ambient light sensor
 *
 * Copyright (c) 2009 Barnes & Noble
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */

#ifndef ISL29001_H_
#define ISL29001_H_

#define ISL29001_DRV_NAME	"isl29001"

/* Command to read lux value into int* argument */
#define ISL29001_CMD_READ_LUX 1

extern struct i2c_driver isl29001_driver;

#endif
