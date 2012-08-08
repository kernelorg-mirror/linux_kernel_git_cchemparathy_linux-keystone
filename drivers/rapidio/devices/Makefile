#
# Makefile for RapidIO devices
#

obj-$(CONFIG_RAPIDIO_TSI721)	+= tsi721.o
ifeq ($(CONFIG_RAPIDIO_DMA_ENGINE),y)
obj-$(CONFIG_RAPIDIO_TSI721)	+= tsi721_dma.o
endif
obj-$(CONFIG_TI_KEYSTONE_RAPIDIO)      += keystone_rio.o
