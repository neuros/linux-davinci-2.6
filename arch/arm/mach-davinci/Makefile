#
# Makefile for the linux kernel.
#
#


# Common objects
obj-y 				:= time.o irq.o dma.o clock.o serial.o io.o devices.o

# Board specific
obj-$(CONFIG_MACH_DAVINCI_EVM)  	+= board-evm.o i2c-emac.o
obj-$(CONFIG_DAVINCI_MCBSP)		+= mcbsp.o
obj-$(CONFIG_DAVINCI_I2C_EXPANDER)	+= i2c-client.o

ifeq ($(CONFIG_LEDS),y)
obj-$(CONFIG_MACH_DAVINCI_EVM)	+= leds-evm.o
endif
