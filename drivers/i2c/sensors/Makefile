#
# Makefile for miscellaneous I2C chip drivers.
#
#

obj-$(CONFIG_SENSORS_ONLY_BMA255)	+= bma255_without_Ecompass.o

ifeq ($(CONFIG_I2C_DEBUG_CHIP),y)
EXTRA_CFLAGS += -DDEBUG
endif

