/*
 * (C) 2007 by OpenMoko, Inc.
 * Author: Harald Welte <laforge@openmoko.org>
 *
 * based on existing SAM7DFU code from OpenPCD:
 * (C) Copyright 2006 by Harald Welte <hwelte@hmw-consulting.de>
 *
 * See file CREDITS for list of people who contributed to this
 * project.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston,
 * MA 02111-1307 USA
 */

#include <config.h>
#if defined(CONFIG_USBD_DFU)

/* FIXME disbale debug */
#define DEBUG

#include <common.h>
DECLARE_GLOBAL_DATA_PTR; /* FIXME needed? */

#include <malloc.h>
#include <linux/types.h>
#include <linux/list.h>
#include <asm/errno.h>
#include <usbdevice.h>
#include <usb_dfu.h>
#include <usb_dfu_descriptors.h>

#include <flash_entity.h>

#include "../../serial/usbtty.h"			/* for STR_* defs */

#define POLL_TIMEOUT_MILLISECONDS 5

static struct flash_entity *flash_ents;
static int num_flash_ents;

/* HACK to include nand backend code for now */
#include "../dfu/nand.c"

void register_flash_entities2(struct flash_entity *flents, int n)
{
	flash_ents = flents;
	num_flash_ents = n;
}

static void handle_getstatus(struct urb *urb, int max)
{
	struct usb_device_instance *dev = urb->device;
	struct dfu_status *dstat = (struct dfu_status *) urb->buffer;

	if (!urb->buffer || urb->buffer_length < sizeof(*dstat)) {
		debug("invalid urb! ");
		return;
	}

	switch (dev->dfu_state) {
	case DFU_STATE_dfuDNLOAD_SYNC:
	case DFU_STATE_dfuDNBUSY:
		debug("DNLOAD_IDLE ");
		dev->dfu_state = DFU_STATE_dfuDNLOAD_IDLE;
		break;
	case DFU_STATE_dfuMANIFEST_SYNC:
		break;
	default:
		/* return; */
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
	urb->actual_length = MIN(sizeof(*dstat), max);

	/* we don't need to explicitly send data here, will
	 * be done by the original caller! */
}

static void handle_getstate(struct urb *urb)
{
	if (!urb->buffer || urb->buffer_length < sizeof(u_int8_t)) {
		debug("invalid urb! ");
		return;
	}

	urb->buffer[0] = urb->device->dfu_state & 0xff;
	urb->actual_length = sizeof(u_int8_t);
}

static int handle_dnload(struct urb *urb, u_int16_t len, int first)
{
	struct usb_device_instance *dev = urb->device;

	if (len == 0)
		dev->dfu_state = DFU_STATE_dfuMANIFEST_SYNC;

	return handle_nand_dnload(urb, len, first);
}

static int handle_upload(struct urb *urb, u_int16_t len, int first)
{
	return handle_nand_upload(urb, len, first);
}

#ifndef CONFIG_USBD_PRODUCTID_DFU
#define CONFIG_USBD_PRODUCTID_DFU CONFIG_USBD_PRODUCTID_CDCACM
#endif

static const struct usb_device_descriptor dfu_dev_descriptor = {
	.bLength		= USB_DT_DEVICE_SIZE,
	.bDescriptorType	= USB_DT_DEVICE,
	.bcdUSB			= 0x0100,
	.bDeviceClass		= 0x00,
	.bDeviceSubClass	= 0x00,
	.bDeviceProtocol	= 0x00,
	.bMaxPacketSize0	= EP0_MAX_PACKET_SIZE,
	.idVendor		= CONFIG_USBD_VENDORID,
	.idProduct		= CONFIG_USBD_PRODUCTID_DFU,
	.bcdDevice		= 0x0000,
	.iManufacturer		= DFU_STR_MANUFACTURER,
	.iProduct		= DFU_STR_PRODUCT,
	.iSerialNumber		= DFU_STR_SERIAL,
	.bNumConfigurations	= 0x01,
};

static struct _dfu_desc dfu_cfg_descriptor = {
	.ucfg = {
		.bLength		= USB_DT_CONFIG_SIZE,
		.bDescriptorType	= USB_DT_CONFIG,
		.wTotalLength		= USB_DT_CONFIG_SIZE +
					  DFU_NUM_ALTERNATES *
					  USB_DT_INTERFACE_SIZE +
					  USB_DT_DFU_SIZE,
		.bNumInterfaces		= 1,
		.bConfigurationValue	= 1,
		.iConfiguration		= DFU_STR_CONFIG,
		.bmAttributes		= BMATTRIBUTE_RESERVED,
		.bMaxPower		= 50,
	},
	.func_dfu = DFU_FUNC_DESC,
};

int dfu_ep0_handler(struct urb *urb)
{
	int rc, ret = RET_NOTHING;
	u_int8_t req = urb->device_request.bRequest;
	u_int16_t len = urb->device_request.wLength;
	struct usb_device_instance *dev = urb->device;

	debug("dfu_ep0(req=0x%x, len=%u) old_state = %u ",
		req, len, dev->dfu_state);

	switch (dev->dfu_state) {
	case DFU_STATE_appIDLE:
		switch (req) {
		case USB_REQ_DFU_GETSTATUS:
			handle_getstatus(urb, len);
			break;
		case USB_REQ_DFU_GETSTATE:
			handle_getstate(urb);
			break;
		case USB_REQ_DFU_DETACH:
			dev->dfu_state = DFU_STATE_appDETACH;
			ret = RET_ZLP;
			goto out;
			break;
		default:
			ret = RET_STALL;
		}
		break;
	case DFU_STATE_appDETACH:
		switch (req) {
		case USB_REQ_DFU_GETSTATUS:
			handle_getstatus(urb, len);
			break;
		case USB_REQ_DFU_GETSTATE:
			handle_getstate(urb);
			break;
		default:
			dev->dfu_state = DFU_STATE_appIDLE;
			ret = RET_STALL;
			goto out;
			break;
		}
		/* FIXME: implement timer to return to appIDLE */
		break;
	case DFU_STATE_dfuIDLE:
		switch (req) {
		case USB_REQ_DFU_DNLOAD:
			printf("Got DNLOAD req in dfuIDLE, len = %d\n", len);
			if (len == 0) {
				dev->dfu_state = DFU_STATE_dfuERROR;
				ret = RET_STALL;
				goto out;
			}
			dev->dfu_state = DFU_STATE_dfuDNLOAD_SYNC;
			ret = handle_dnload(urb, len, 1);
			break;
		case USB_REQ_DFU_UPLOAD:
			dev->dfu_state = DFU_STATE_dfuUPLOAD_IDLE;
			handle_upload(urb, len, 1);
			break;
		case USB_REQ_DFU_ABORT:
			/* no zlp? */
			ret = RET_ZLP;
			break;
		case USB_REQ_DFU_GETSTATUS:
			handle_getstatus(urb, len);
			break;
		case USB_REQ_DFU_GETSTATE:
			handle_getstate(urb);
			break;
		case USB_REQ_DFU_DETACH:
			/* Proprietary extension: 'detach' from idle mode and
			 * get back to runtime mode in case of USB Reset.  As
			 * much as I dislike this, we just can't use every USB
			 * bus reset to switch back to runtime mode, since at
			 * least the Linux USB stack likes to send a number of
			 * resets in a row :( */
			dev->dfu_state = DFU_STATE_dfuMANIFEST_WAIT_RST;
			break;
		default:
			dev->dfu_state = DFU_STATE_dfuERROR;
			ret = RET_STALL;
			goto out;
			break;
		}
		break;
	case DFU_STATE_dfuDNLOAD_SYNC:
		switch (req) {
		case USB_REQ_DFU_GETSTATUS:
			handle_getstatus(urb, len);
			/* FIXME: state transition depending on block
			 * completeness */
			break;
		case USB_REQ_DFU_GETSTATE:
			handle_getstate(urb);
			break;
		default:
			dev->dfu_state = DFU_STATE_dfuERROR;
			ret = RET_STALL;
			goto out;
		}
		break;
	case DFU_STATE_dfuDNBUSY:
		switch (req) {
		case USB_REQ_DFU_GETSTATUS:
			/* FIXME: only accept getstatus if bwPollTimeout
			 * has elapsed */
			handle_getstatus(urb, len);
			break;
		default:
			dev->dfu_state = DFU_STATE_dfuERROR;
			ret = RET_STALL;
			goto out;
		}
		break;
	case DFU_STATE_dfuDNLOAD_IDLE:
		switch (req) {
		case USB_REQ_DFU_DNLOAD:
			dev->dfu_state = DFU_STATE_dfuDNLOAD_SYNC;
			ret = handle_dnload(urb, len, 0);
			break;
		case USB_REQ_DFU_ABORT:
			dev->dfu_state = DFU_STATE_dfuIDLE;
			ret = RET_ZLP;
			break;
		case USB_REQ_DFU_GETSTATUS:
			handle_getstatus(urb, len);
			break;
		case USB_REQ_DFU_GETSTATE:
			handle_getstate(urb);
			break;
		default:
			dev->dfu_state = DFU_STATE_dfuERROR;
			ret = RET_STALL;
			break;
		}
		break;
	case DFU_STATE_dfuMANIFEST_SYNC:
		switch (req) {
		case USB_REQ_DFU_GETSTATUS:
			/* We're MainfestationTolerant */
			dev->dfu_state = DFU_STATE_dfuIDLE;
			handle_getstatus(urb, len);
			break;
		case USB_REQ_DFU_GETSTATE:
			handle_getstate(urb);
			break;
		default:
			dev->dfu_state = DFU_STATE_dfuERROR;
			ret = RET_STALL;
			break;
		}
		break;
	case DFU_STATE_dfuMANIFEST:
		/* we should never go here */
		dev->dfu_state = DFU_STATE_dfuERROR;
		ret = RET_STALL;
		break;
	case DFU_STATE_dfuMANIFEST_WAIT_RST:
		/* we should never go here */
		break;
	case DFU_STATE_dfuUPLOAD_IDLE:
		switch (req) {
		case USB_REQ_DFU_UPLOAD:
			/* state transition if less data then requested */
			rc = handle_upload(urb, len, 0);
			if (rc >= 0 && rc < len)
				dev->dfu_state = DFU_STATE_dfuIDLE;
			break;
		case USB_REQ_DFU_ABORT:
			dev->dfu_state = DFU_STATE_dfuIDLE;
			/* no zlp? */
			ret = RET_ZLP;
			break;
		case USB_REQ_DFU_GETSTATUS:
			handle_getstatus(urb, len);
			break;
		case USB_REQ_DFU_GETSTATE:
			handle_getstate(urb);
			break;
		default:
			dev->dfu_state = DFU_STATE_dfuERROR;
			ret = RET_STALL;
			break;
		}
		break;
	case DFU_STATE_dfuERROR:
		switch (req) {
		case USB_REQ_DFU_GETSTATUS:
			handle_getstatus(urb, len);
			break;
		case USB_REQ_DFU_GETSTATE:
			handle_getstate(urb);
			break;
		case USB_REQ_DFU_CLRSTATUS:
			dev->dfu_state = DFU_STATE_dfuIDLE;
			dev->dfu_status = DFU_STATUS_OK;
			/* no zlp? */
			ret = RET_ZLP;
			break;
		default:
			dev->dfu_state = DFU_STATE_dfuERROR;
			ret = RET_STALL;
			break;
		}
		break;
	default:
		return DFU_EP0_UNHANDLED;
		break;
	}

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
}

void str2wide(char *str, u16 * wide);
static struct usb_string_descriptor *create_usbstring(char *string)
{
	struct usb_string_descriptor *strdesc;
	int size = sizeof(*strdesc) + strlen(string)*2;

	if (size > 255)
		return NULL;

	strdesc = malloc(size);
	if (!strdesc)
		return NULL;

	strdesc->bLength = size;
	strdesc->bDescriptorType = USB_DT_STRING;
	str2wide(string, strdesc->wData);

	return strdesc;
}


static void dfu_init_strings(struct usb_device_instance *dev)
{
	int i;
	struct usb_string_descriptor *strdesc;

	strdesc = create_usbstring(CONFIG_DFU_CFG_STR);
	usb_strings[DFU_STR_CONFIG] = strdesc;

	for (i = 0; i < DFU_NUM_ALTERNATES; i++) {
		if (i == 0) {
			strdesc = create_usbstring(CONFIG_DFU_ALT0_STR);
		} else {
			struct part_info *part = get_partition_nand(i-1);

			if (part)
				strdesc = create_usbstring(part->name);
			else
				strdesc =
				    create_usbstring("undefined partition");
		}
		if (!strdesc)
			continue;
		usb_strings[STR_COUNT+i+1] = strdesc;
	}
}

int dfu_init_instance(struct usb_device_instance *dev)
{
	int i;

	for (i = 0; i != DFU_NUM_ALTERNATES; i++) {
		struct usb_interface_descriptor *uif =
		    dfu_cfg_descriptor.uif+i;

		uif->bLength		= USB_DT_INTERFACE_SIZE;
		uif->bDescriptorType	= USB_DT_INTERFACE;
		uif->bAlternateSetting	= i;
		uif->bInterfaceClass	= 0xfe;
		uif->bInterfaceSubClass	= 1;
		uif->bInterfaceProtocol	= 2; /* FIXME: Corect number? */
		uif->iInterface		= DFU_STR_ALT(i);
	}

	dev->dfu_dev_desc = &dfu_dev_descriptor;
	dev->dfu_cfg_desc = &dfu_cfg_descriptor;
	dev->dfu_state = DFU_STATE_appIDLE;
	dev->dfu_status = DFU_STATUS_OK;

	dfu_init_strings(dev);

	return 0;
}

/* event handler for usb device state events */
void dfu_event(struct usb_device_instance *device,
	       usb_device_event_t event, int data)
{
	switch (event) {
	case DEVICE_RESET:
		switch (device->dfu_state) {
		case DFU_STATE_appDETACH:
			device->dfu_state = DFU_STATE_dfuIDLE;
			printf("DFU: Switching to DFU Mode\n");
			break;
		case DFU_STATE_dfuMANIFEST_WAIT_RST:
			device->dfu_state = DFU_STATE_appIDLE;
			printf("DFU: Switching back to Runtime mode\n");
			break;
		default:
			break;
		}
		break;
	case DEVICE_CONFIGURED:
	case DEVICE_DE_CONFIGURED:
		debug("SET_CONFIGURATION(%u) ", device->configuration);
		/* fallthrough */
	case DEVICE_SET_INTERFACE:
		debug("SET_INTERFACE(%u,%u) old_state = %u ",
			device->interface, device->alternate,
			device->dfu_state);
		switch (device->dfu_state) {
		case DFU_STATE_appIDLE:
		case DFU_STATE_appDETACH:
		case DFU_STATE_dfuIDLE:
		case DFU_STATE_dfuMANIFEST_WAIT_RST:
			/* do nothing, we're fine */
			break;
		case DFU_STATE_dfuDNLOAD_SYNC:
		case DFU_STATE_dfuDNBUSY:
		case DFU_STATE_dfuDNLOAD_IDLE:
		case DFU_STATE_dfuMANIFEST:
			device->dfu_state = DFU_STATE_dfuERROR;
			device->dfu_status = DFU_STATUS_errNOTDONE;
			/* FIXME: free malloc()ed buffer! */
			break;
		case DFU_STATE_dfuMANIFEST_SYNC:
		case DFU_STATE_dfuUPLOAD_IDLE:
		case DFU_STATE_dfuERROR:
			device->dfu_state = DFU_STATE_dfuERROR;
			device->dfu_status = DFU_STATUS_errUNKNOWN;
			break;
		}
		debug("new_state = %u\n", device->dfu_state);
		break;
	default:
		break;
	}
}
#endif /* CONFIG_USBD_DFU */
