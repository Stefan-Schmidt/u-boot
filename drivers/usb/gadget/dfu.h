/*
 * dfu.h -- Device Firmware Update gadget
 *
 * Copyright (C) 2011 Samsung Electronics
 * author: Andrzej Pietrasiewicz <andrzej.p@samsung.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */

#ifndef DFU_H_
#define DFU_H_

#include <linux/compiler.h>

/*
 * Linux kernel compatibility layer
 */
#define GFP_ATOMIC				((gfp_t) 0)
#define GFP_KERNEL				((gfp_t) 0)
#define true					1
#define false					0
#define dev_dbg(...)				do {} while (0)
#define dev_vdbg(...)				do {} while (0)
#define dev_err(...)				do {} while (0)
#define dev_warn(...)				do {} while (0)
#define dev_info(...)				do {} while (0)
#define pr_warning(...)				do {} while (0)
#define spin_lock_init(lock)			do {} while (0)
#define spin_lock(lock)				do {} while (0)
#define spin_unlock(lock)			do {} while (0)
#define spin_lock_irqsave(lock,flags)		do {flags = 1;} while (0)
#define spin_unlock_irqrestore(lock,flags)	do {flags = 0;} while (0)
#define kmalloc(x,y)				malloc(x)
#define kfree(x)				free(x)
#define kzalloc(size,flags)			calloc((size), 1)
#define __init
#define __exit
#define __exit_p(x)				x
#define module_init(...)
#define module_exit(...)
#define min_t					min
#define spinlock_t				int
#define bool					int
/*
 * end compatibility layer
 */

#define DBG(d, fmt, args...) \
	dev_dbg(&(d)->gadget->dev , fmt , ## args)
#define VDBG(d, fmt, args...) \
	dev_vdbg(&(d)->gadget->dev , fmt , ## args)
#define ERROR(d, fmt, args...) \
	dev_err(&(d)->gadget->dev , fmt , ## args)
#define WARN(d, fmt, args...) \
	dev_warn(&(d)->gadget->dev , fmt , ## args)
#define INFO(d, fmt, args...) \
	dev_info(&(d)->gadget->dev , fmt , ## args)

#define DRIVER_VERSION			"Msciwoj"

/* Thanks to NetChip Technologies for donating this product ID.  */
#define DRIVER_VENDOR_NUM		0x0525		/* NetChip */
#define DRIVER_PRODUCT_NUM		0xffff		/* DFU */

#define STRING_MANUFACTURER		0
#define STRING_PRODUCT			1
#define STRING_SERIAL			2
#define STRING_DFU_NAME			49
#define DFU_STR_BASE			50

#define	DFU_CONFIG_VAL			1
#define DFU_DT_FUNC			0x21

#define DFU_BIT_WILL_DETACH		(0x1 << 3)
#define DFU_BIT_MANIFESTATION_TOLERANT	(0x1 << 2)
#define DFU_BIT_CAN_UPLOAD		(0x1 << 1)
#define DFU_BIT_CAN_DNLOAD		0x1

/* big enough to hold our biggest descriptor */
#define USB_BUFSIZ			4096

#define USB_REQ_DFU_DETACH		0x00
#define USB_REQ_DFU_DNLOAD		0x01
#define USB_REQ_DFU_UPLOAD		0x02
#define USB_REQ_DFU_GETSTATUS		0x03
#define USB_REQ_DFU_CLRSTATUS		0x04
#define USB_REQ_DFU_GETSTATE		0x05
#define USB_REQ_DFU_ABORT		0x06

#define DFU_STATUS_OK			0x00
#define DFU_STATUS_errTARGET		0x01
#define DFU_STATUS_errFILE		0x02
#define DFU_STATUS_errWRITE		0x03
#define DFU_STATUS_errERASE		0x04
#define DFU_STATUS_errCHECK_ERASED	0x05
#define DFU_STATUS_errPROG		0x06
#define DFU_STATUS_errVERIFY		0x07
#define DFU_STATUS_errADDRESS		0x08
#define DFU_STATUS_errNOTDONE		0x09
#define DFU_STATUS_errFIRMWARE		0x0a
#define DFU_STATUS_errVENDOR		0x0b
#define DFU_STATUS_errUSBR		0x0c
#define DFU_STATUS_errPOR		0x0d
#define DFU_STATUS_errUNKNOWN		0x0e
#define DFU_STATUS_errSTALLEDPKT	0x0f

#define RET_STALL			-1
#define RET_ZLP				0
#define RET_STAT_LEN			6

#define ALTSETTING_BASE			2
#define STRING_ALTSETTING_BASE		4

enum dfu_state {
	DFU_STATE_appIDLE		= 0,
	DFU_STATE_appDETACH		= 1,
	DFU_STATE_dfuIDLE		= 2,
	DFU_STATE_dfuDNLOAD_SYNC	= 3,
	DFU_STATE_dfuDNBUSY		= 4,
	DFU_STATE_dfuDNLOAD_IDLE	= 5,
	DFU_STATE_dfuMANIFEST_SYNC	= 6,
	DFU_STATE_dfuMANIFEST		= 7,
	DFU_STATE_dfuMANIFEST_WAIT_RST	= 8,
	DFU_STATE_dfuUPLOAD_IDLE	= 9,
	DFU_STATE_dfuERROR		= 10,
};

struct dfu_status {
	__u8				bStatus;
	__u8				bwPollTimeout[3];
	__u8				bState;
	__u8				iString;
} __packed;

struct dfu_function_descriptor {
	__u8				bLength;
	__u8				bDescriptorType;
	__u8				bmAttributes;
	__le16				wDetachTimeOut;
	__le16				wTransferSize;
	__le16				bcdDFUVersion;
} __packed;

struct dfu_dev {
	spinlock_t			lock;
	struct usb_gadget		*gadget;
	struct usb_request		*req;	/* for control responses */

	/* when configured, we have one config */
	u8				config;
	u8				altsetting;
	enum dfu_state			dfu_state;
	unsigned int			dfu_status;
	struct usb_descriptor_header	**function;
	struct usb_string		*strings;
	bool				not_prepared;
};

#endif /* DFU_H_ */
