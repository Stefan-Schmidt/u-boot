/*
 * (C) Copyright 2009 Texas Instruments Incorporated.
 *
 * Based on
 * u-boot OMAP1510 USB drivers (drivers/usbdcore_omap1510.c)
 * twl4030 init based on linux (drivers/i2c/chips/twl4030_usb.c)
 *
 * Author:	Diego Dompe (diego.dompe@ridgerun.com)
 *		Atin Malaviya (atin.malaviya@gmail.com)
 *		Matthew Warton (mwarton@ok-labs.com)
 *		Bernard Blackham (b-musb@largestprime.net)
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.	See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
 */

#include <common.h>

#include <asm/io.h>
#include <asm/arch/clocks.h>
#include <asm/arch/clocks_omap3.h>
#include <asm/arch/sys_proto.h>
#include <usb/musb_udc.h>
#include "../gadget/ep0.h"
#if defined(CONFIG_USB_OMAP3)
#include "omap3.h"
#elif defined(CONFIG_USB_DAVINCI)
#include "davinci.h"
#endif

#define UDC_BASE MENTOR_USB0_BASE

/* Private definitions */
enum ep0_status { IDLE, DATA_STAGE, DATA_COMPLETE };

/* Private variables */
static struct usb_device_instance *udc_device;
static enum ep0_status ep0status = IDLE;
static unsigned char do_set_address = 0;
static struct urb *ep0_urb = NULL;

/* Helper functions */
static void insl(u32 reg, u32 *data, u32 size)
{
	u32 t;

	for (t = 0; t < size; t++, data++)
		*data = readl(reg);
}

static void outsl(u32 reg, u32 *data, u32 size)
{
	u32 t;

	for (t = 0; t < size; t++, data++)
		writel(*data, reg);
}

static void outsb(u32 reg, u8 *data, u32 size)
{
	u32 t;

	for (t = 0; t < size; t++, data++)
		writeb(*data, reg);
}

static void musb_fifo_read(int epnumber, u8 *data, u32 size)
{
	if ((u32)data & 0x3) {		/* Not aligned data */
		insb((UDC_FIFO0 + (epnumber << 2)), data, size);
	} else {			/* 32 bits aligned data */
		int i;

		insl(UDC_FIFO0 + (epnumber << 2), (u32 *)data, size >> 2);
		data += size & ~0x3;
		i = size & 0x3;
		while (i--) {
			*data = readb(UDC_FIFO0 + (epnumber << 2));
			data++;
		}
	}
}

static void musb_fifo_write(int epnumber, u8 *data, u32 size)
{
	if ((u32)data & 0x3) {		/* Not aligned data */
		outsb(UDC_FIFO0 + (epnumber << 2), data, size);
	} else {			/* 32 bits aligned data */
		int i;

		outsl(UDC_FIFO0 + (epnumber << 2), (u32 *)data, size >> 2);
		data += size & ~0x3;
		i = size & 0x3;
		while (i--) {
			writeb(*data, UDC_FIFO0 + (epnumber << 2));
			data++;
		}
	}
}

static void musb_fifos_configure(struct usb_device_instance *device)
{
	int ep;
	struct usb_bus_instance *bus;
	struct usb_endpoint_instance *endpoint;
	unsigned short ep_ptr, ep_size, ep_doublebuffer;
	int ep_addr, packet_size, buffer_size, attributes;

	bus = device->bus;

	ep_ptr = 0;

	for (ep = 0; ep < bus->max_endpoints; ep++) {
		endpoint = bus->endpoint_array + ep;
		ep_addr = endpoint->endpoint_address;
		if ((ep_addr & USB_ENDPOINT_DIR_MASK) == USB_DIR_IN) {
			/* IN endpoint */
			packet_size = endpoint->tx_packetSize;
			attributes = endpoint->tx_attributes;
		} else {
			/* OUT endpoint */
			packet_size = endpoint->rcv_packetSize;
			attributes = endpoint->rcv_attributes;
		}

		switch (packet_size) {
		case 0:
			ep_size = 0;
			break;
		case 8:
			ep_size = 0;
			break;
		case 16:
			ep_size = 1;
			break;
		case 32:
			ep_size = 2;
			break;
		case 64:
			ep_size = 3;
			break;
		case 128:
			ep_size = 4;
			break;
		case 256:
			ep_size = 5;
			break;
		case 512:
			ep_size = 6;
			break;
		default:
			printf("ep 0x%02x has bad packet size %d",
				ep_addr, packet_size);
			packet_size = 0;
			ep_size = 0;
			break;
		}

		switch (attributes & USB_ENDPOINT_XFERTYPE_MASK) {
		case USB_ENDPOINT_XFER_CONTROL:
		case USB_ENDPOINT_XFER_BULK:
		case USB_ENDPOINT_XFER_INT:
		default:
			/* A non-isochronous endpoint may optionally be
			 * double-buffered. For now we disable
			 * double-buffering.
			 */
			ep_doublebuffer = 0;
			if (packet_size > 64)
				packet_size = 0;
			if (!ep || !ep_doublebuffer)
				buffer_size = packet_size;
			else
				buffer_size = packet_size * 2;
			break;
		case USB_ENDPOINT_XFER_ISOC:
			/* Isochronous endpoints are always double-
			 * buffered
			 */
			ep_doublebuffer = 1;
			buffer_size = packet_size * 2;
			break;
		}

		/* check to see if our packet buffer RAM is exhausted */
		if ((ep_ptr + buffer_size) > UDC_MAX_FIFO_SIZE) {
			printf("out of packet RAM for ep 0x%02x buf size %d",
				ep_addr, buffer_size);
			buffer_size = packet_size = 0;
		}

		/* force a default configuration for endpoint 0 since it is
		 * always enabled
		 */
		if (!ep && ((packet_size < 8) || (packet_size > 64))) {
			buffer_size = packet_size = 64;
			ep_size = 3;
		}

		writeb(ep & 0xF, UDC_INDEX);
		if ((ep_addr & USB_ENDPOINT_DIR_MASK) == USB_DIR_IN) {
			/* IN endpoint */
			writeb((ep_doublebuffer << 4) | (ep_size & 0xf),
				UDC_TXFIFOSZ);
			writew(ep_ptr >> 3, UDC_TXFIFOADDR);
			if (!ep) {	/* This only apply for ep != 0 */
				writew(packet_size & 0x3FF, UDC_TXMAXP);
			}
		} else {
			/* OUT endpoint */
			writeb((ep_doublebuffer << 4) | (ep_size & 0xf),
				UDC_RXFIFOSZ);
			writew(ep_ptr >> 3, UDC_RXFIFOADDR);
			if (!ep) {	/* This only apply for ep != 0 */
				writew(packet_size & 0x3FF, UDC_RXMAXP);
			}
		}
		ep_ptr += buffer_size;
	}
}

static void musb_ep0_tx(struct usb_endpoint_instance *endpoint)
{
	unsigned int size = 0;
	struct urb *urb = endpoint->tx_urb;

	writeb(0, UDC_INDEX);

	if (urb) {

		if ((size =
			MIN(urb->actual_length - endpoint->sent,
				endpoint->tx_packetSize))) {

			musb_fifo_write(0, urb->buffer + endpoint->sent, size);
		}
		endpoint->last = size;

		if (((endpoint->sent + size) == ep0_urb->device_request.wLength)
			|| (size != endpoint->tx_packetSize)) {
			ep0status = DATA_COMPLETE;
			/* Transmit packet and set data end */
			writew(0xA, UDC_CSR0);
		} else {
			writew(0x2, UDC_CSR0);	/* Transmit packet */
		}
	}
}

static void musb_ep0_handler(struct usb_endpoint_instance *endpoint)
{
	u16 csr0;

	writeb(0, UDC_INDEX);

	/* Check errors */
	csr0 = readw(UDC_CSR0);

	if (csr0 & 0x4) {			/* Sent stall */
		writew(csr0 & ~0x4, UDC_CSR0);	/* Clear stall */
		printf("%s: stall received on EP0!\n", __FUNCTION__);
	}

	if (csr0 & 0x10) {		/* Setup end */
		writew(0x80, UDC_CSR0);	/* Clear setup end */
		printf("%s: setup END early happened! status is %d\n",
			__FUNCTION__, ep0status);
		ep0status = IDLE;
		return;
	}

	switch (ep0status) {
	case DATA_COMPLETE:
		if (do_set_address) {
			/*
			 * We need to set the address only after
			 * the status stage is complete
			 */
			writeb(udc_device->address, UDC_FADDR);
			do_set_address = 0;
		}
		ep0status = IDLE;
		/* Fallthrough */
	case IDLE:		/* Receiving a setup packet */
		if (csr0 & 0x1) {
			insl(UDC_FIFO0,
				(unsigned int *) &ep0_urb->device_request, 2);

			/* If we have data, then don't go to IDLE state */
			if (ep0_urb->device_request.wLength) {
				ep0status = DATA_STAGE;

				writew(0x40, UDC_CSR0);	/* Clear RXPKTRDY */
				if ((ep0_urb->device_request.
					bmRequestType & USB_REQ_DIRECTION_MASK)
					== USB_REQ_DEVICE2HOST) {

					/* Try to process setup packet */
					if (ep0_recv_setup(ep0_urb)) {
						/*
						 * Not a setup packet, stall
						 * next EP0 transaction
						 */
						writew(0x20, UDC_CSR0);
						ep0status = IDLE;
						return;
					}
					/*
					 * If we are sending data, do it now, as
					 * ep0_recv_setup should have prepare
					 * them
					 */
					endpoint->tx_urb = ep0_urb;
					endpoint->sent = 0;

					musb_ep0_tx(endpoint);
				} else {
					endpoint->rcv_urb = ep0_urb;
					ep0_urb->actual_length = 0;
				}
			} else {	/* Processing zero-length packet */
				/*
				 * The www.linux-usb.org/usbtest 'test 14'
				 * fails with error for zero length request.
				 * If the SETUP packet requests ZERO length data
				 * from device-to-host, the TXPKTRDY bit needs
				 * to be set in TXCSR otherwise the STATUS stage
				 * of control transfer will never complete.
				 */
				if ((ep0_urb->device_request.
					bmRequestType & USB_REQ_DIRECTION_MASK)
					== USB_REQ_DEVICE2HOST) {
					/*
					 * Clear RXPKTRDY and DATAEND and
					 * TXPKTRDY
					 */
					writew(0x4A, UDC_CSR0);
				} else {
					/* Clear RXPKTRDY and DATAEND */
					writew(0x48, UDC_CSR0);
				}

				/* Try to process setup packet */
				if (ep0_recv_setup(ep0_urb)) {
					/*
					 * Not a setup packet, stall next EP0
					 * transaction
					 */
					writew(0x20, UDC_CSR0);
					ep0status = IDLE;
					return;
				}

				switch (ep0_urb->device_request.bRequest) {
				case USB_REQ_SET_ADDRESS:
					usbd_device_event_irq(udc_device,
						DEVICE_ADDRESS_ASSIGNED, 0);
					do_set_address = 1;
					break;
				case USB_REQ_SET_CONFIGURATION:
					usbd_device_event_irq(udc_device,
						DEVICE_CONFIGURED, 0);
					break;
				}

				ep0status = DATA_COMPLETE;
			}
		}
		break;
	case DATA_STAGE:
		if ((ep0_urb->device_request.
			bmRequestType & USB_REQ_DIRECTION_MASK)
			== USB_REQ_DEVICE2HOST) {
			if (!(csr0 & 0x2)) {	/* There packet was send? */
				endpoint->sent += endpoint->last;
				/*
				 * If we finished sending data we would not
				 * be on the DATA_STAGE
				 */
				musb_ep0_tx(endpoint);
			}
		} else {
			/* Receiving data */
			u16 length = readw(UDC_COUNT0);

			if (length) {
				if (ep0_urb->actual_length + length >
					ep0_urb->device_request.wLength)
					length =
					ep0_urb->device_request.wLength -
					ep0_urb->actual_length;

				endpoint->last = length;

				musb_fifo_read(0, &ep0_urb->
					buffer[ep0_urb->actual_length], length);
				ep0_urb->actual_length += length;
			}

			/*
			 * We finish if we received the amount of data expected,
			 * or less of the packet size
			 */
			if ((ep0_urb->actual_length ==
				ep0_urb->device_request.wLength) ||
				(endpoint->last != endpoint->tx_packetSize)) {
				ep0status = DATA_COMPLETE;
				/* Clear RXPKTRDY and DATAEND */
				writew(0x48, UDC_CSR0);
				/* This will process the incoming data */
                                //printf("musb %08x %08x\n", ((int *)ep0_urb->buffer)[0], ((int *)ep0_urb->buffer)[1]);
				if (ep0_recv_setup(ep0_urb)) {
					/*
					 * Not a setup packet, stall next EP0
					 * transaction
					 */
					writew(0x20, UDC_CSR0);
					return;
				}
			} else
				writew(0x40, UDC_CSR0);	/* Clear RXPKTRDY */
		}
		break;
	}
}

static void musb_ep_tx(struct usb_endpoint_instance *endpoint)
{
	unsigned int size = 0, epnumber =
		endpoint->endpoint_address & USB_ENDPOINT_NUMBER_MASK;
	struct urb *urb = endpoint->tx_urb;

	writeb(epnumber, UDC_INDEX);
	if (urb) {
		if ((size =
			MIN(urb->actual_length - endpoint->sent,
			endpoint->tx_packetSize))) {
			musb_fifo_write(epnumber, urb->buffer + endpoint->sent,
					size);
		}
		endpoint->last = size;
		endpoint->state = 1; /* Transmit hardware is busy */

		writew(readw(UDC_TXCSR) | 0x1, UDC_TXCSR); /* Transmit packet */
	}
}


static void musb_tx_handler(struct usb_endpoint_instance *endpoint)
{
	unsigned int epnumber =
		endpoint->endpoint_address & USB_ENDPOINT_NUMBER_MASK;
	u16 txcsr;

	writeb(epnumber, UDC_INDEX);

	/* Check errors */
	txcsr = readw(UDC_TXCSR);

	if (txcsr & 0x4) {		/* Clear underrun */
		txcsr &= ~0x4;
	}
	if (txcsr & 0x20) {		/* SENTSTALL */
		writew(txcsr & ~0x20, UDC_TXCSR);	/* Clear stall */
		return;
	}

	if (endpoint->tx_urb && !(txcsr & 0x1)) { /* The packet was send? */
		if ((endpoint->sent + endpoint->last == endpoint->tx_urb->
			actual_length)	/* Send a zero length packet? */
			&& (endpoint->last == endpoint->tx_packetSize)) {
			/* Prepare to transmit a zero-length packet. */
			endpoint->sent += endpoint->last;
			musb_ep_tx(endpoint);
		} else if (endpoint->tx_urb->actual_length) {
			/* retire the data that was just sent */
			usbd_tx_complete(endpoint);
			endpoint->state = 0; /* Transmit hardware is free */

			/*
			 * Check to see if we have more data ready to transmit
			 * now.
			 */
			if (endpoint->tx_urb && endpoint->tx_urb->
				actual_length) {
				musb_ep_tx(endpoint);
			}
		}
	}
}

static void musb_rx_handler(struct usb_endpoint_instance *endpoint)
{
	unsigned int epnumber =
		endpoint->endpoint_address & USB_ENDPOINT_NUMBER_MASK;
	u16 rxcsr;
	u16 length;

	writeb(epnumber, UDC_INDEX);

	/* Check errors */
	rxcsr = readw(UDC_RXCSR);

	if (!(rxcsr & 0x1))		/* There is a package received? */
		return;

	if (rxcsr & 0x40)		/* SENTSTALL */
		writew(rxcsr & ~0x40, UDC_RXCSR);	/* Clear stall */

	length = readw(UDC_RXCOUNT);

	if (endpoint->rcv_urb) {
		/* Receiving data */
		if (length) {
			musb_fifo_read(epnumber, &endpoint->rcv_urb->
				buffer[endpoint->rcv_urb->actual_length],
				length);
			writew(rxcsr & ~0x1, UDC_RXCSR);	/* Clear RXPKTRDY */
			usbd_rcv_complete(endpoint, length, 0);
		}
	} else {
		printf("%s: no receive URB!\n", __FUNCTION__);
	}
}

static void musb_reset(void)
{
	usbd_device_event_irq(udc_device, DEVICE_HUB_CONFIGURED, 0);
	usbd_device_event_irq(udc_device, DEVICE_RESET, 0);
	ep0status = IDLE;
	do_set_address = 0;
}

/* Public functions - called by usbdcore, usbtty, etc. */
void udc_irq(void)
{
	unsigned char int_usb = readb(UDC_INTRUSB);
	unsigned short int_tx = readw(UDC_INTRTX);
	unsigned short int_rx = readw(UDC_INTRRX);
	int ep;

	if (int_usb) {
		if (int_usb & 0x4) {	/* Reset */
			/* The controller clears FADDR, INDEX, and FIFOs */
			musb_reset();
		}
		if (int_usb & 0x20) {	/* Disconnected */
			usbd_device_event_irq(udc_device, DEVICE_HUB_RESET, 0);
		}
		if (int_usb & 0x1)
			usbd_device_event_irq(udc_device,
					DEVICE_BUS_INACTIVE, 0);
		if (int_usb & 0x2)
			usbd_device_event_irq(udc_device,
					DEVICE_BUS_ACTIVITY, 0);
	}

	/* Note: IRQ values auto clear so read just before processing */
	if (int_rx) {		/* OUT endpoints */
		ep = 1;
		int_rx >>= 1;
		while (int_rx) {
			if (int_rx & 1)
				musb_rx_handler(udc_device->bus->endpoint_array
						+ ep);
			int_rx >>= 1;
			ep++;
		}
	}
	if (int_tx) {		/* IN endpoints */
		if (int_tx & 1)
			musb_ep0_handler(udc_device->bus->endpoint_array);

		ep = 1;
		int_tx >>= 1;
		while (int_tx) {
			if (int_tx & 1)
				musb_tx_handler(udc_device->bus->endpoint_array
						+ ep);
			int_tx >>= 1;
			ep++;
		}
	}
}

/* Turn on the USB connection */
void udc_connect(void)
{
	writeb(0x1, UDC_DEVCTL);

	if (!(readb(UDC_DEVCTL) & 0x80)) {
		printf("Error, the USB hardware is not on B mode\n");
		writeb(0x0, UDC_DEVCTL);
		return;
	}
}

/* Turn off the USB connection */
void udc_disconnect(void)
{
	if (!(readb(UDC_DEVCTL) & 0x80)) {
		printf("Error, the USB hardware is not on B mode");
		return;
	}

	writeb(0x0, UDC_DEVCTL);
}

int udc_endpoint_write(struct usb_endpoint_instance *endpoint)
{
	/* Transmit only if the hardware is available */
	if (endpoint->tx_urb && endpoint->state == 0)
		musb_ep_tx(endpoint);

	return 0;
}

/*
 * udc_setup_ep - setup endpoint
 *
 * Associate a physical endpoint with endpoint_instance
 */
void udc_setup_ep(struct usb_device_instance *device, unsigned int ep,
			struct usb_endpoint_instance *endpoint)
{
	int ep_addr;
	int attributes;

	/*
	 * We dont' have a way to identify if the endpoint definitions changed,
	 * so we have to always reconfigure the FIFOs to avoid problems
	 */
	musb_fifos_configure(device);

	ep_addr = endpoint->endpoint_address;
	if ((ep_addr & USB_ENDPOINT_DIR_MASK) == USB_DIR_IN) {
		/* IN endpoint */
		attributes = endpoint->tx_attributes;
	} else {
		/* OUT endpoint */
		attributes = endpoint->rcv_attributes;
	}

	writeb(ep & 0xF, UDC_INDEX);
	if ((ep_addr & USB_ENDPOINT_DIR_MASK) == USB_DIR_IN) {
		/* IN endpoint */
		if (!ep) {		/* This only apply for ep != 0 */
			/* Empty fifo twice on case of previous double buffer */
			writew(1<<3, UDC_TXCSR);
			writew(1<<3, UDC_TXCSR);

			if (attributes & USB_ENDPOINT_XFER_ISOC)
				writew(readw(UDC_TXCSR) | (1 << 13) | (1 << 14) |
					0x4, UDC_TXCSR);
			else
				writew((readw(UDC_TXCSR) | (1 << 13) | 0x4) &
					~(1 << 14), UDC_TXCSR);
		}
		/* Enable interrupt */
		writew(readw(UDC_INTRTXE) | (1 << ep), UDC_INTRTXE);
	} else {
		/* OUT endpoint */
		if (!ep) {		/* This only apply for ep != 0 */
			if (attributes & USB_ENDPOINT_XFER_ISOC)
				writew((readw(UDC_RXCSR) | (1 << 14)) & ~(1 << 13),
					UDC_RXCSR);
			else
				writew(readw(UDC_RXCSR) & ~(1 << 14) & ~(1 << 13),
					UDC_RXCSR);
		}
		/* Enable interrupt */
		writew(readw(UDC_INTRRXE) | (1 << ep), UDC_INTRRXE);
	}
}

/*
 * udc_startup_events - allow udc code to do any additional startup
 */
void udc_startup_events(struct usb_device_instance *device)
{
	/* The DEVICE_INIT event puts the USB device in the state STATE_INIT. */
	usbd_device_event_irq(device, DEVICE_INIT, 0);

	/*
	 * The DEVICE_CREATE event puts the USB device in the state
	 * STATE_ATTACHED.
	 */
	usbd_device_event_irq(device, DEVICE_CREATE, 0);

	/*
	 * Some USB controller driver implementations signal
	 * DEVICE_HUB_CONFIGURED and DEVICE_RESET events here.
	 * DEVICE_HUB_CONFIGURED causes a transition to the state STATE_POWERED,
	 * and DEVICE_RESET causes a transition to the state STATE_DEFAULT.
	 * The MUSB client controller has the capability to detect when the
	 * USB cable is connected to a powered USB bus, so we will defer the
	 * DEVICE_HUB_CONFIGURED and DEVICE_RESET events until later.
	 */

	/* Save the device structure pointer */
	udc_device = device;

	/* Setup ep0 urb */
	if (!ep0_urb) {
		ep0_urb =
			usbd_alloc_urb(udc_device, udc_device->bus->
					endpoint_array);
	} else {
		printf("udc_enable: ep0_urb already allocated %p\n", ep0_urb);
	}

	/* Enable control interrupts */
	writeb(0xf7, UDC_INTRUSBE);
}

void udc_set_nak(int epid)
{
/*
 * On MUSB the NAKing is controlled by the USB controller buffers,
 * so as long as we don't read data from the FIFO, the controller will NAK.
 * Nothing to see here, move along...
 */
}

void udc_unset_nak(int epid)
{
/*
 * On MUSB the NAKing is controlled by the USB controller buffers,
 * so as long as we don't read data from the FIFO, the controller will NAK.
 * Nothing to see here, move along...
 */
}

/* Start to initialize h/w stuff */
int udc_init(void)
{
	/* Clock is initialized on the board code */

	if (musb_platform_init() == -1)
		return -1;

	/* MUSB soft-reset */
	writel(2, UDC_SYSCONFIG);
	/* MUSB end any previous session */
	writeb(0x0, UDC_DEVCTL);

	writel(readl(UDC_FORCESTDBY) & ~1, UDC_FORCESTDBY); /* disable MSTANDBY */
	writel(readl(UDC_SYSCONFIG) | (2<<12), UDC_SYSCONFIG); /* ena SMARTSTDBY */
	writel(readl(UDC_SYSCONFIG) & ~1, UDC_SYSCONFIG); /* disable AUTOIDLE */
	writel(readl(UDC_SYSCONFIG) | (2<<3), UDC_SYSCONFIG); /* enable SMARTIDLE */
	writel(readl(UDC_SYSCONFIG) | 1, UDC_SYSCONFIG); /* enable AUTOIDLE */

	/* Configure the PHY as PHY interface is 12-pin, 8-bit SDR ULPI */
	sr32((void *)UDC_INTERFSEL, 0, 1, 1);

	/* Turn off interrupts */
	writew(0x00, UDC_INTRTXE);
	writew(0x00, UDC_INTRRXE);

#if CONFIG_MUSB_FULL_SPEED
	/*
	 * Use Full speed for debugging proposes, useful so most USB
	 * analyzers can catch the transactions
	 */
	writeb(0, UDC_POWER);
	printf("MUSB: using full speed\n");
#else
	printf("MUSB: using high speed\n");
#endif

	return 0;
}
