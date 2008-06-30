obj-$(CONFIG_RT73) += rt73.o

rt73-objs := assoc.o auth.o auth_rsp.o connect.o \
	md5.o mlme.o rt2x00debug.o rtmp_info.o rtmp_init.o \
	rtmp_main.o rtmp_tkip.o rtmp_wep.o rtusb_bulk.o \
	rtusb_data.o rtusb_io.o sanity.o sync.o wpa.o

EXTRA_CFLAGS += -DAGGREGATION_SUPPORT -DWMM_SUPPORT -DRTMP_EMBEDDED

ifeq ($(CONFIG_RT73_DEBUG),y)
EXTRA_CFLAGS += -DDBG
endif

