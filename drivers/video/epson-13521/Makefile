#
# Makefile for the Epson s1d13521 framebuffer device driver
#


obj-$(CONFIG_FB_EPSON_S1D13521)			+= s1d13521fb.o
s1d13521fb-objs					+= s1d13521base.o s1d13521if.o bs_cmd.o s1d13521fb_virt.o s1d13521fb_collide.o
s1d13521fb-$(CONFIG_FB_EPSON_PCI) 		+= s1d13521pci.o
s1d13521fb-$(CONFIG_FB_EPSON_PROC) 		+= s1d13521proc.o
s1d13521fb-$(CONFIG_FB_EPSON_GPIO_SMDK6410)	+= s1d13521ifgpio.o
EXTRA_CFLAGS					+= -D'S1D13xxxFB_INCLUDE="s1d13521.h"'




