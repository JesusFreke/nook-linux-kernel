#
# Makefile for regulator drivers.
#


obj-$(CONFIG_REGULATOR) += core.o
obj-$(CONFIG_REGULATOR_FIXED_VOLTAGE) += fixed.o
obj-$(CONFIG_REGULATOR_VIRTUAL_CONSUMER) += virtual.o

obj-$(CONFIG_REGULATOR_BQ24022) += bq24022.o
obj-$(CONFIG_REGULATOR_WM8350) += wm8350-regulator.o

ccflags-$(CONFIG_REGULATOR_DEBUG) += -DDEBUG
