/*
 * dfu.c -- Device Firmware Update gadget
 *
 * Copyright (C) 2011 Samsung Electronics
 * author: Andrzej Pietrasiewicz <andrzej.p@samsung.com>
 *
 * Copyright (C) 2011-2012 Stefan Schmidt <stefan@datenfreihafen.org>
 *
 * Based on gadget zero:
 * Copyright (C) 2003-2007 David Brownell
 * All rights reserved.
 *
 * (C) 2007 by OpenMoko, Inc.
 * Author: Harald Welte <laforge@openmoko.org>
 *
 * based on existing SAM7DFU code from OpenPCD:
 * (C) Copyright 2006 by Harald Welte <hwelte@hmw-consulting.de>
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

#define VERBOSE_DEBUG
#define DEBUG

/*
#include <linux/kernel.h>
#include <linux/utsname.h>
#include <linux/device.h>
*/

#include <common.h>
#include <asm-generic/errno.h>
#include <linux/usb/ch9.h>
#include <linux/usb/gadget.h>

#include <flash_entity.h>

#include "gadget_chips.h"
/* #include "epautoconf.c" */
/* #include "config.c" */
/* #include "usbstring.c" */

#include <malloc.h>
#include "dfu.h"

#define POLL_TIMEOUT_MILLISECONDS 5

static struct flash_entity *flash_ents;
static int num_flash_ents;

static struct usb_device_descriptor device_desc = {
	.bLength =		sizeof device_desc,
	.bDescriptorType =	USB_DT_DEVICE,
	.bcdUSB =		__constant_cpu_to_le16(0x0100),
	.bDeviceClass =		USB_CLASS_VENDOR_SPEC,
	.idVendor =		__constant_cpu_to_le16(DRIVER_VENDOR_NUM),
	.idProduct =		__constant_cpu_to_le16(DRIVER_PRODUCT_NUM),
	.iManufacturer =	STRING_MANUFACTURER,
	.iProduct =		STRING_PRODUCT,
	.iSerialNumber =	STRING_SERIAL,
	.bNumConfigurations =	1,
};

static struct usb_config_descriptor dfu_config = {
	.bLength =		sizeof dfu_config,
	.bDescriptorType =	USB_DT_CONFIG,
	/* compute wTotalLength on the fly */
	.bNumInterfaces =	1,
	.bConfigurationValue =	DFU_CONFIG_VAL,
	.iConfiguration =	STRING_DFU_NAME,
	.bmAttributes =		USB_CONFIG_ATT_ONE | USB_CONFIG_ATT_SELFPOWER,
	.bMaxPower =		1,	/* self-powered */
};

static struct usb_otg_descriptor otg_descriptor = {
	.bLength =		sizeof otg_descriptor,
	.bDescriptorType =	USB_DT_OTG,
	.bmAttributes =		USB_OTG_SRP,
};

static const struct dfu_function_descriptor dfu_func = {
	.bLength =		sizeof dfu_func,
	.bDescriptorType =	DFU_DT_FUNC,
	.bmAttributes =		DFU_BIT_WILL_DETACH | /*FIXME */
				DFU_BIT_MANIFESTATION_TOLERANT |
				DFU_BIT_CAN_UPLOAD |
				DFU_BIT_CAN_DNLOAD,
	.wDetachTimeOut =	0,
	.wTransferSize =	USB_BUFSIZ,
	.bcdDFUVersion =	__constant_cpu_to_le16(0x0110),
};

static const struct usb_interface_descriptor dfu_intf_runtime = {
	.bLength =		sizeof dfu_intf_runtime,
	.bDescriptorType =	USB_DT_INTERFACE,
	.bNumEndpoints =	0,
	.bInterfaceClass =	USB_CLASS_APP_SPEC,
	.bInterfaceSubClass =	1,
	.bInterfaceProtocol =	1,
	.iInterface =		STRING_DFU_NAME,
};

static const struct usb_descriptor_header *dfu_function_runtime[] = {
	(struct usb_descriptor_header *) &otg_descriptor,
	(struct usb_descriptor_header *) &dfu_func,
	(struct usb_descriptor_header *) &dfu_intf_runtime,
	NULL,
};

static struct usb_qualifier_descriptor dev_qualifier = {
	.bLength =		sizeof dev_qualifier,
	.bDescriptorType =	USB_DT_DEVICE_QUALIFIER,
	.bcdUSB =		__constant_cpu_to_le16(0x0200),
	.bDeviceClass =		USB_CLASS_VENDOR_SPEC,
	.bNumConfigurations =	1,
};

static char manufacturer[50];
static const char longname[] = "DFU Gadget";
/* default serial number takes at least two packets */
static const char serial[] = "0123456789.0123456789.012345678";
static const char dfu_name[] = "Device Firmware Upgrade";
static const char shortname[] = "DFU";

/* static strings, in UTF-8 */
static struct usb_string strings_runtime[] = {
	{ STRING_MANUFACTURER, manufacturer, },
	{ STRING_PRODUCT, longname, },
	{ STRING_SERIAL, serial, },
	{ STRING_DFU_NAME, dfu_name, },
	{  }			/* end of list */
};

static struct usb_gadget_strings stringtab_runtime = {
	.language	= 0x0409,	/* en-us */
	.strings	= strings_runtime,
};

static struct usb_gadget_strings stringtab_dfu = {
	.language	= 0x0409,	/* en-us */
};

static bool is_runtime(struct dfu_dev *dev)
{
	return dev->dfu_state == DFU_STATE_appIDLE ||
		dev->dfu_state == DFU_STATE_appDETACH;
}

static int config_buf(struct usb_gadget *gadget,
		u8 *buf, u8 type, unsigned index)
{
	int hs = 0;
	struct dfu_dev *dev = get_gadget_data(gadget);
	const struct usb_descriptor_header **function;
	int len;

	if (index > 0)
		return -EINVAL;

	if (gadget_is_dualspeed(gadget)) {
		hs = (gadget->speed == USB_SPEED_HIGH);
		if (type == USB_DT_OTHER_SPEED_CONFIG)
			hs = !hs;
	}
	if (is_runtime(dev))
		function = dfu_function_runtime;
	else
		function = (const struct usb_descriptor_header **)dev->function;

	/* for now, don't advertise srp-only devices */
	if (!gadget_is_otg(gadget))
		function++;
#ifdef CONFIG_USB_ETHER
	len = usb_gadget_config_buf(&dfu_config,
			buf, USB_BUFSIZ, function);
#endif
	if (len < 0)
		return len;
	((struct usb_config_descriptor *) buf)->bDescriptorType = type;
	return len;
}

static void dfu_reset_config(struct dfu_dev *dev)
{
	if (dev->config == 0)
		return;

	DBG(dev, "reset config\n");

	dev->config = 0;
}

static int dfu_set_config(struct dfu_dev *dev, unsigned number)
{
	int result = 0;
	struct usb_gadget *gadget = dev->gadget;
	char *speed;

	if (number == dev->config)
		return 0;

	dfu_reset_config(dev);

	if (DFU_CONFIG_VAL != number) {
		result = -EINVAL;
		return result;
	}

	switch (gadget->speed) {
	case USB_SPEED_LOW:
		speed = "low";
		break;
	case USB_SPEED_FULL:
		speed = "full";
		break;
	case USB_SPEED_HIGH:
		speed = "high";
		break;
	default:
		speed = "?";
		break;
	}

	dev->config = number;
	INFO(dev, "%s speed config #%d: %s\n", speed, number, dfu_name);
	return result;
}

/*-------------------------------------------------------------------------*/

static void empty_complete(struct usb_ep *ep, struct usb_request *req)
{
	/* intentionally empty */
}

static void dnload_request_complete(struct usb_ep *ep, struct usb_request *req)
{
	struct dfu_dev *dev = req->context;
	struct flash_entity *fe = &flash_ents[dev->altsetting];

	if (dev->not_prepared) {
		printf("DOWNLOAD %s\n", fe->name);
		fe->prepare(fe->ctx, FLASH_WRITE);
		dev->not_prepared = false;
	}

	if (req->length > 0)
		fe->write_block(fe->ctx, req->length, req->buf);
	else {
		fe->finish(fe->ctx, FLASH_WRITE);
		dev->not_prepared = true;
	}
}

static void handle_getstatus(struct usb_request *req)
{
	struct dfu_status *dstat = (struct dfu_status *)req->buf;
	struct dfu_dev *dev = req->context;

	switch (dev->dfu_state) {
	case DFU_STATE_dfuDNLOAD_SYNC:
	case DFU_STATE_dfuDNBUSY:
		dev->dfu_state = DFU_STATE_dfuDNLOAD_IDLE;
		break;
	case DFU_STATE_dfuMANIFEST_SYNC:
		break;
	default:
		break;
	}

	/* send status response */
	dstat->bStatus = dev->dfu_status;
	dstat->bState = dev->dfu_state;
	dstat->iString = 0;
	/* FIXME: Use real values from flash subsystem here instead a hardcoded
	 * value */
	dstat->bwPollTimeout[0] = POLL_TIMEOUT_MILLISECONDS & 0xff;
	dstat->bwPollTimeout[1] = (POLL_TIMEOUT_MILLISECONDS >> 8) & 0xff;
	dstat->bwPollTimeout[2] = (POLL_TIMEOUT_MILLISECONDS >> 16) & 0xff;
	/* Without the following line getstatus breaks with dfu-util. Maybe this
	 * value gets set correctly by the new gadget layer nowadays, check */
	//req->actual = MIN(sizeof(*dstat), max);
}

static void handle_getstate(struct usb_request *req)
{
	struct dfu_dev *dev = req->context;

	((u8 *)req->buf)[0] = dev->dfu_state & 0xff;
	req->actual = sizeof(u8);
}

static int handle_upload(struct usb_request *req, u16 len)
{
	struct dfu_dev *dev = req->context;
	struct flash_entity *fe = &flash_ents[dev->altsetting];
	int n;

	if (dev->not_prepared) {
		printf("UPLOAD %s\n", fe->name);
		fe->prepare(fe->ctx, FLASH_READ);
		dev->not_prepared = false;
	}
	n = fe->read_block(fe->ctx, len, req->buf);

	/* no more data to read from this entity */
	if (n < len) {
		fe->finish(fe->ctx, FLASH_READ);
		dev->not_prepared = true;
	}

	return n;
}

static int handle_dnload(struct usb_gadget *gadget, u16 len)
{
	struct dfu_dev *dev = get_gadget_data(gadget);
	struct usb_request *req = dev->req;

	if (len == 0)
		dev->dfu_state = DFU_STATE_dfuMANIFEST_SYNC;

	req->complete = dnload_request_complete;

	return len;
}

static int
dfu_handle(struct usb_gadget *gadget, const struct usb_ctrlrequest *ctrl)
{
	struct dfu_dev *dev = get_gadget_data(gadget);
	struct usb_request *req = dev->req;
	u16 len = le16_to_cpu(ctrl->wLength);
	int rc = 0;

// u_int16_t len = urb->device_request.wLength;
// handle_getstatus(urb, len);

	switch (dev->dfu_state) {
	case DFU_STATE_appIDLE:
		switch (ctrl->bRequest) {
		case USB_REQ_DFU_GETSTATUS:
			handle_getstatus(req);
			//break;
			return RET_STAT_LEN;
		case USB_REQ_DFU_GETSTATE:
			handle_getstate(req);
			break;
		case USB_REQ_DFU_DETACH:
			dev->dfu_state = DFU_STATE_appDETACH;
			dev->dfu_state = DFU_STATE_dfuIDLE; // Why?
			return RET_ZLP;
		default:
			return RET_STALL;
		}
		break;
	case DFU_STATE_appDETACH:
		switch (ctrl->bRequest) {
		case USB_REQ_DFU_GETSTATUS:
			handle_getstatus(req);
			// break;
			return RET_STAT_LEN;
		case USB_REQ_DFU_GETSTATE:
			handle_getstate(req);
			break;
		default:
			dev->dfu_state = DFU_STATE_appIDLE;
			return RET_STALL;
		}
		/* FIXME: implement timer to return to appIDLE */
		break;
	case DFU_STATE_dfuIDLE:
		switch (ctrl->bRequest) {
		case USB_REQ_DFU_DNLOAD:
			if (len == 0) {
				dev->dfu_state = DFU_STATE_dfuERROR;
				return RET_STALL;
			}
			dev->dfu_state = DFU_STATE_dfuDNLOAD_SYNC;
			// ret = handle_dnload(urb, val, len, 1);
			return handle_dnload(gadget, len);
		case USB_REQ_DFU_UPLOAD:
			dev->dfu_state = DFU_STATE_dfuUPLOAD_IDLE;
			// handle_upload(urb, val, len, 1);
			return handle_upload(req, len);
		case USB_REQ_DFU_ABORT:
			/* no zlp? */
			return RET_ZLP;
		case USB_REQ_DFU_GETSTATUS:
			handle_getstatus(req);
			return RET_STAT_LEN; // Why?
		case USB_REQ_DFU_GETSTATE:
			handle_getstate(req);
			break;
		case USB_REQ_DFU_DETACH:
			/* Proprietary extension: 'detach' from idle mode and
			 * get back to runtime mode in case of USB Reset.  As
			 * much as I dislike this, we just can't use every USB
			 * bus reset to switch back to runtime mode, since at
			 * least the Linux USB stack likes to send a number of
			 * resets in a row :(
			 */
			dev->dfu_state = DFU_STATE_dfuMANIFEST_WAIT_RST;
			dev->dfu_state = DFU_STATE_appIDLE; // Why?
			break;
		default:
			dev->dfu_state = DFU_STATE_dfuERROR;
			return RET_STALL;
		}
		break;
	case DFU_STATE_dfuDNLOAD_SYNC:
		switch (ctrl->bRequest) {
		case USB_REQ_DFU_GETSTATUS:
			handle_getstatus(req);
			return RET_STAT_LEN; // Why?
			/* FIXME: state transition depending
			 * on block completeness */
		case USB_REQ_DFU_GETSTATE:
			handle_getstate(req);
			break;
		default:
			dev->dfu_state = DFU_STATE_dfuERROR;
			return RET_STALL;
		}
		break;
	case DFU_STATE_dfuDNBUSY:
		switch (ctrl->bRequest) {
		case USB_REQ_DFU_GETSTATUS:
			/* FIXME: only accept getstatus if bwPollTimeout
			 * has elapsed */
			handle_getstatus(req);
			return RET_STAT_LEN; // Why?
		default:
			dev->dfu_state = DFU_STATE_dfuERROR;
			return RET_STALL;
		}
		break;
	case DFU_STATE_dfuDNLOAD_IDLE:
		switch (ctrl->bRequest) {
		case USB_REQ_DFU_DNLOAD:
			dev->dfu_state = DFU_STATE_dfuDNLOAD_SYNC;
			//ret = handle_dnload(urb, len, 0);
			return handle_dnload(gadget, len);
		case USB_REQ_DFU_ABORT:
			dev->dfu_state = DFU_STATE_dfuIDLE;
			return RET_ZLP;
		case USB_REQ_DFU_GETSTATUS:
			handle_getstatus(req);
			return RET_STAT_LEN;
		case USB_REQ_DFU_GETSTATE:
			handle_getstate(req);
			break;
		default:
			dev->dfu_state = DFU_STATE_dfuERROR;
			return RET_STALL;
		}
		break;
	case DFU_STATE_dfuMANIFEST_SYNC:
		switch (ctrl->bRequest) {
		case USB_REQ_DFU_GETSTATUS:
			/* We're MainfestationTolerant */
			dev->dfu_state = DFU_STATE_dfuIDLE;
			handle_getstatus(req);
			return RET_STAT_LEN;
		case USB_REQ_DFU_GETSTATE:
			handle_getstate(req);
			break;
		default:
			dev->dfu_state = DFU_STATE_dfuERROR;
			return RET_STALL;
		}
		break;
	case DFU_STATE_dfuMANIFEST:
		/* we should never go here */
		dev->dfu_state = DFU_STATE_dfuERROR;
		return RET_STALL;
	case DFU_STATE_dfuMANIFEST_WAIT_RST:
		/* we should never go here */
		break;
	case DFU_STATE_dfuUPLOAD_IDLE:
		switch (ctrl->bRequest) {
		case USB_REQ_DFU_UPLOAD:
			/* state transition if less data then requested */
			//rc = handle_upload(urb, len, 0);
			rc = handle_upload(req, len);
			if (rc >= 0 && rc < len)
				dev->dfu_state = DFU_STATE_dfuIDLE;
			return rc;
		case USB_REQ_DFU_ABORT:
			dev->dfu_state = DFU_STATE_dfuIDLE;
			/* no zlp? */
			return RET_ZLP;
		case USB_REQ_DFU_GETSTATUS:
			handle_getstatus(req);
			return RET_STAT_LEN;
		case USB_REQ_DFU_GETSTATE:
			handle_getstate(req);
			break;
		default:
			dev->dfu_state = DFU_STATE_dfuERROR;
			return RET_STALL;
		}
		break;
	case DFU_STATE_dfuERROR:
		switch (ctrl->bRequest) {
		case USB_REQ_DFU_GETSTATUS:
			handle_getstatus(req);
			return RET_STAT_LEN;
		case USB_REQ_DFU_GETSTATE:
			handle_getstate(req);
			break;
		case USB_REQ_DFU_CLRSTATUS:
			dev->dfu_state = DFU_STATE_dfuIDLE;
			dev->dfu_status = DFU_STATUS_OK;
			/* no zlp? */
			return RET_ZLP;
		default:
			dev->dfu_state = DFU_STATE_dfuERROR;
			return RET_STALL;
		}
		break;
	default:
		return -1;
	}

	return 0;
#if 0
out:
	debug("new_state = %u, ret = %u\n", dev->dfu_state, ret);

	switch (ret) {
	case RET_ZLP:
		urb->actual_length = 0;
		return DFU_EP0_ZLP;
		break;
	case RET_STALL:
		return DFU_EP0_STALL;
		break;
	case RET_NOTHING:
		break;
	}

	return DFU_EP0_DATA;
#endif
}

static int
dfu_setup(struct usb_gadget *gadget, const struct usb_ctrlrequest *ctrl)
{
	struct dfu_dev *dev = get_gadget_data(gadget);
	struct usb_request *req = dev->req;
	int value = -EOPNOTSUPP;
	u16 w_index = le16_to_cpu(ctrl->wIndex);
	u16 w_value = le16_to_cpu(ctrl->wValue);
	u16 w_length = le16_to_cpu(ctrl->wLength);

	req->zero = 0;
	req->complete = empty_complete;
	if (!(ctrl->bRequestType & USB_TYPE_CLASS))
		switch (ctrl->bRequest) {

		case USB_REQ_GET_DESCRIPTOR:
			if (ctrl->bRequestType != USB_DIR_IN)
				goto unknown;
			switch (w_value >> 8) {

			case USB_DT_DEVICE:
				value = min(w_length, (u16) sizeof device_desc);
				memcpy(req->buf, &device_desc, value);
				break;
			case USB_DT_DEVICE_QUALIFIER:
				if (!gadget_is_dualspeed(gadget))
					break;
				value = min(w_length,
					    (u16) sizeof dev_qualifier);
				memcpy(req->buf, &dev_qualifier, value);
				break;

			case USB_DT_OTHER_SPEED_CONFIG:
				if (!gadget_is_dualspeed(gadget))
					break;
				/* FALLTHROUGH */
			case USB_DT_CONFIG:
				value = config_buf(gadget, req->buf,
						w_value >> 8,
						w_value & 0xff);
				if (value >= 0)
					value = min_t(w_length, (u16) value);
				break;

			case USB_DT_STRING:
				/* wIndex == language code. */
				value = usb_gadget_get_string(
					is_runtime(dev) ? &stringtab_runtime :
					&stringtab_dfu,
					w_value & 0xff, req->buf);
				if (value >= 0)
					value = min_t(w_length, (u16) value);
				break;
			case DFU_DT_FUNC:
				value = min(w_length, (u16) sizeof dfu_func);
				memcpy(req->buf, &dfu_func, value);
				break;
			}
			break;

		case USB_REQ_SET_CONFIGURATION:
			if (ctrl->bRequestType != 0)
				goto unknown;
			if (gadget->a_hnp_support)
				DBG(dev, "HNP available\n");
			else if (gadget->a_alt_hnp_support)
				DBG(dev, "HNP needs a different root port\n");
			else
				VDBG(dev, "HNP inactive\n");
			spin_lock(&dev->lock);
			value = dfu_set_config(dev, w_value);
			spin_unlock(&dev->lock);
			break;
		case USB_REQ_GET_CONFIGURATION:
			if (ctrl->bRequestType != USB_DIR_IN)
				goto unknown;
			*(u8 *)req->buf = dev->config;
			value = min(w_length, (u16) 1);
			break;

		/* until we add altsetting support, or other interfaces,
		 * only 0/0 are possible.  pxa2xx only supports 0/0 (poorly)
		 * and already killed pending endpoint I/O.
		 */
		case USB_REQ_SET_INTERFACE:
			if (ctrl->bRequestType != USB_RECIP_INTERFACE)
				goto unknown;
			spin_lock(&dev->lock);
			if (dev->config && w_index == 0) {
				u8		config = dev->config;

				/* resets interface configuration, forgets about
				 * previous transaction state (queued bufs, etc)
				 * and re-inits endpoint state (toggle etc)
				 * no response queued, just zero
				 * status == success.
				 * if we had more than one interface we couldn't
				 * use this "reset the config" shortcut.
				 */
				dfu_reset_config(dev);
				dfu_set_config(dev, config);
				dev->altsetting = w_value;
				value = 0;
			}
			spin_unlock(&dev->lock);
			break;
		case USB_REQ_GET_INTERFACE:
			if (ctrl->bRequestType !=
			    (USB_DIR_IN|USB_RECIP_INTERFACE))
				goto unknown;
			if (!dev->config)
				break;
			if (w_index != 0) {
				value = -EDOM;
				break;
			}
			*(u8 *)req->buf = 0;
			value = min(w_length, (u16) 1);
			break;

		default:
unknown:
			VDBG(dev,
				"unknown control req%02x.%02x v%04x i%04x l%d\n",
				ctrl->bRequestType, ctrl->bRequest,
				w_value, w_index, w_length);
		}
	else
		value = dfu_handle(gadget, ctrl);

	/* respond with data transfer before status phase? */
	if (value >= 0) {
		req->length = value;
		req->zero = value < w_length;
		value = usb_ep_queue(gadget->ep0, req, GFP_ATOMIC);
		if (value < 0) {
			DBG(dev, "ep_queue --> %d\n", value);
			req->status = 0;
		}
	}

	/* device either stalls (value < 0) or reports success */
	return value;
}

/*-------------------------------------------------------------------------*/

static void dfu_disconnect(struct usb_gadget *gadget)
{
	struct dfu_dev *dev = get_gadget_data(gadget);
	unsigned long flags;

	spin_lock_irqsave(&dev->lock, flags);
	dfu_reset_config(dev);
	spin_unlock_irqrestore(&dev->lock, flags);
}

static int dfu_prepare_function(struct dfu_dev *dev, int n)
{
	struct usb_interface_descriptor *d;
	int i = 0;

	dev->function = kzalloc((ALTSETTING_BASE + n + 1) *
				sizeof(struct usb_descriptor_header *),
				GFP_KERNEL);
	if (!dev->function)
		goto enomem;

	dev->function[0] = (struct usb_descriptor_header *)&otg_descriptor;
	dev->function[1] = (struct usb_descriptor_header *)&dfu_func;

	for (i = 0; i < n; ++i) {
		d = kzalloc(sizeof(*d), GFP_KERNEL);
		if (!d)
			goto enomem;

		d->bLength =		sizeof(*d);
		d->bDescriptorType =	USB_DT_INTERFACE;
		d->bAlternateSetting =	i;
		d->bNumEndpoints =	0;
		d->bInterfaceClass =	USB_CLASS_APP_SPEC;
		d->bInterfaceSubClass =	1;
		d->bInterfaceProtocol =	2;
		d->iInterface =		DFU_STR_BASE + i;

		dev->function[ALTSETTING_BASE + i] =
			(struct usb_descriptor_header *)d;
	}
	dev->function[ALTSETTING_BASE + i] = NULL;

	return 1;

enomem:
	while (i) {
		kfree(dev->function[--i + ALTSETTING_BASE]);
		dev->function[i + ALTSETTING_BASE] = NULL;
	}
	kfree(dev->function);

	return 0;
}

static int
dfu_prepare_strings(struct dfu_dev *dev, struct flash_entity *f, int n)
{
	int i = 0;

	dev->strings = kzalloc((STRING_ALTSETTING_BASE + n + 1) *
				sizeof(struct usb_string),
				GFP_KERNEL);
	if (!dev->strings)
		goto enomem;

	dev->strings[0].id = STRING_MANUFACTURER;
	dev->strings[0].s = manufacturer;
	dev->strings[1].id = STRING_PRODUCT;
	dev->strings[1].s = longname;
	dev->strings[2].id = STRING_SERIAL;
	dev->strings[2].s = serial;
	dev->strings[3].id = STRING_DFU_NAME;
	dev->strings[3].s = dfu_name;

	for (i = 0; i < n; ++i) {
		char *s;

		dev->strings[STRING_ALTSETTING_BASE + i].id = DFU_STR_BASE + i;
		s = kzalloc(strlen(f[i].name) + 1, GFP_KERNEL);
		if (!s)
			goto enomem;

		strcpy(s, f[i].name);
		dev->strings[STRING_ALTSETTING_BASE + i].s = s;
	}
	dev->strings[STRING_ALTSETTING_BASE + i].id = 0;
	dev->strings[STRING_ALTSETTING_BASE + i].s = NULL;

	return 1;

enomem:
	while (i) {
		kfree((void *)dev->strings[--i + STRING_ALTSETTING_BASE].s);
		dev->strings[i + STRING_ALTSETTING_BASE].s = NULL;
	}
	kfree(dev->strings);

	return 0;
}

static void dfu_unbind(struct usb_gadget *gadget)
{
	struct dfu_dev *dev = get_gadget_data(gadget);
	int i;

	DBG(dev, "unbind\n");

	if (dev->strings) {
		i = num_flash_ents;
		while (i) {
			kfree((void *)
			      dev->strings[--i + STRING_ALTSETTING_BASE].s);
			dev->strings[i + STRING_ALTSETTING_BASE].s = NULL;
		}
		kfree(dev->strings);
	}
	if (dev->function) {
		i = num_flash_ents;
		while (i) {
			kfree(dev->function[--i + ALTSETTING_BASE]);
			dev->function[i + ALTSETTING_BASE] = NULL;
		}
		kfree(dev->function);
	}
	/* we've already been disconnected ... no i/o is active */
	if (dev->req) {
		dev->req->length = USB_BUFSIZ;
		kfree(dev->req->buf);
		usb_ep_free_request(gadget->ep0, dev->req);
	}
	kfree(dev);
	set_gadget_data(gadget, NULL);
}

static int __init dfu_bind(struct usb_gadget *gadget)
{
	struct dfu_dev *dev;
	int gcnum;

	usb_ep_autoconfig_reset(gadget);

	gcnum = usb_gadget_controller_number(gadget);
	if (gcnum >= 0)
		device_desc.bcdDevice = cpu_to_le16(0x0200 + gcnum);
	else {
		pr_warning("%s: controller '%s' not recognized\n",
			shortname, gadget->name);
		device_desc.bcdDevice = __constant_cpu_to_le16(0x9999);
	}

	dev = kzalloc(sizeof(*dev), GFP_KERNEL);
	if (!dev)
		return -ENOMEM;
	spin_lock_init(&dev->lock);
	dev->gadget = gadget;
	set_gadget_data(gadget, dev);

	dev->req = usb_ep_alloc_request(gadget->ep0, GFP_KERNEL);
	if (!dev->req)
		goto enomem;
	dev->req->buf = kmalloc(USB_BUFSIZ, GFP_KERNEL);
	if (!dev->req->buf)
		goto enomem;

	dev->req->complete = empty_complete;

	device_desc.bMaxPacketSize0 = gadget->ep0->maxpacket;

	if (gadget_is_dualspeed(gadget))
		dev_qualifier.bMaxPacketSize0 = device_desc.bMaxPacketSize0;

	if (gadget_is_otg(gadget)) {
		otg_descriptor.bmAttributes |= USB_OTG_HNP,
		dfu_config.bmAttributes |= USB_CONFIG_ATT_WAKEUP;
	}

	usb_gadget_set_selfpowered(gadget);

	dev->dfu_state = DFU_STATE_appIDLE;
	dev->dfu_status = DFU_STATUS_OK;
	dev->not_prepared = true;

	if (!dfu_prepare_function(dev, num_flash_ents))
		goto enomem;

	if (!dfu_prepare_strings(dev, flash_ents, num_flash_ents))
		goto enomem;
	stringtab_dfu.strings = dev->strings;

	gadget->ep0->driver_data = dev;
	dev->req->context = dev;

	INFO(dev, "%s, version: " DRIVER_VERSION "\n", longname);

	/* snprintf(manufacturer, sizeof manufacturer, "%s %s with %s",
		init_utsname()->sysname, init_utsname()->release,
		gadget->name); */

	return 0;

enomem:
	dfu_unbind(gadget);
	return -ENOMEM;
}

static void dfu_suspend(struct usb_gadget *gadget)
{
	if (gadget->speed == USB_SPEED_UNKNOWN)
		return;

	DBG(dev, "suspend\n");
}

static void dfu_resume(struct usb_gadget *gadget)
{
	DBG(dev, "resume\n");
}

static struct usb_gadget_driver dfu_driver = {
#ifdef CONFIG_USB_GADGET_DUALSPEED
	.speed		= USB_SPEED_HIGH,
#else
	.speed		= USB_SPEED_FULL,
#endif
	/*.function	= (char *) longname,*/
	.bind		= dfu_bind,
	.unbind		= __exit_p(dfu_unbind),

	.setup		= dfu_setup,
	.disconnect	= dfu_disconnect,

	.suspend	= dfu_suspend,
	.resume		= dfu_resume,

	/*
	.driver		= {
		.name		= (char *) shortname,
		.owner		= THIS_MODULE,
	},*/
};

void register_flash_entities(struct flash_entity *flents, int n)
{
	flash_ents = flents;
	num_flash_ents = n;
}

#ifdef CONFIG_USB_ETHER
int __init dfu_init(void)
{
	return usb_gadget_register_driver(&dfu_driver);
}
module_init(dfu_init);

void __exit dfu_cleanup(void)
{
	usb_gadget_unregister_driver(&dfu_driver);
}
module_exit(cleanup);
#endif
