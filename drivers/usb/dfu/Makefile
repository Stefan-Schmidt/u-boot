include $(TOPDIR)/config.mk

LIB	:= $(obj)libusb_dfu.o

# new USB gadget layer dependencies
COBJS-y += dfu.o

ifdef CONFIG_MMC
COBJS-y += mmc.o
endif

ifdef CONFIG_CMD_FAT
COBJS-y += fat.o
endif

COBJS	:= $(COBJS-y)
SRCS	:= $(COBJS:.o=.c)
OBJS	:= $(addprefix $(obj),$(COBJS))

all:	$(LIB)

$(LIB):	$(obj).depend $(OBJS)
	$(call cmd_link_o_target, $(OBJS))

#########################################################################

# defines $(obj).depend target
include $(SRCTREE)/rules.mk

sinclude $(obj).depend

#########################################################################

