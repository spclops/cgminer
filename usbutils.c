/*
 * Copyright 2012 Andrew Smith
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the Free
 * Software Foundation; either version 3 of the License, or (at your option)
 * any later version.  See COPYING for more details.
 */

#include "config.h"

#include <stdint.h>
#include <stdbool.h>

#include "logging.h"
#include "miner.h"
#include "usbutils.h"

#ifdef USE_ICARUS
#define DRV_ICARUS 1
#endif

#ifdef USE_BITFORCE
#define DRV_BITFORCE 2
#endif

#ifdef USE_MODMINER
#define DRV_MODMINER 3
#endif

#define DRV_LAST -1

#define USB_CONFIG 1

#define EPI(x) (LIBUSB_ENDPOINT_IN | (unsigned char)(x))
#define EPO(x) (LIBUSB_ENDPOINT_OUT | (unsigned char)(x))

#ifdef USE_MODMINER
static struct usb_endpoints mmq_eps[] = {
	{ LIBUSB_TRANSFER_TYPE_BULK,	64,	EPI(3), 0 },
	{ LIBUSB_TRANSFER_TYPE_BULK,	64,	EPO(3), 0 }
};
#endif

// TODO: Add support for (at least) Interrupt endpoints
static struct usb_find_devices find_dev[] = {
/*
#ifdef USE_ICARUS
	{ DRV_ICARUS, 	"ICA",	0x067b,	0x0230,	true,	EPI(3),	EPO(2), 1 },
	{ DRV_ICARUS, 	"LOT",	0x0403,	0x6001,	false,	EPI(0),	EPO(0), 1 },
	{ DRV_ICARUS, 	"CM1",	0x067b,	0x0230,	false,	EPI(0),	EPO(0), 1 },
#endif
#ifdef USE_BITFORCE
	{ DRV_BITFORCE,	"BFL",	0x0403,	0x6014,	true,	EPI(1),	EPO(2), 1 },
#endif
*/
#ifdef USE_MODMINER
	{
		.drv = DRV_MODMINER,
		.name = "MMQ",
		.idVendor = 0x1fc9,
		.idProduct = 0x0003,
		.config = 1,
		.interface = 1,
		.timeout = 100,
		.epcount = 2,
		.eps = mmq_eps },
#endif
	{ DRV_LAST, NULL, 0, 0, 0, 0, 0, 0, NULL }
};

#ifdef USE_BITFORCE
extern struct device_api bitforce_api;
#endif

#ifdef USE_ICARUS
extern struct device_api icarus_api;
#endif

#ifdef USE_MODMINER
extern struct device_api modminer_api;
#endif

/*
 * Our own internal list of used USB devices
 * So two drivers or a single driver searching
 * can't touch the same device during detection
 */
struct usb_list {
	uint8_t bus_number;
	uint8_t device_address;
	uint8_t filler[2];
	struct usb_list *prev;
	struct usb_list *next;
};

#define STRBUFLEN 256
static const char *BLANK = "";

static pthread_mutex_t *list_lock = NULL;
static struct usb_list *usb_head = NULL;

#define EOL "\n"

static const char *DESDEV = "Device";
static const char *DESCON = "Config";
static const char *DESSTR = "String";
static const char *DESINT = "Interface";
static const char *DESEP = "Endpoint";
static const char *DESHID = "HID";
static const char *DESRPT = "Report";
static const char *DESPHY = "Physical";
static const char *DESHUB = "Hub";

static const char *EPIN = "In: ";
static const char *EPOUT = "Out: ";
static const char *EPX = "?: ";

static const char *CONTROL = "Control";
static const char *ISOCHRONOUS_X = "Isochronous+?";
static const char *ISOCHRONOUS_N_X = "Isochronous+None+?";
static const char *ISOCHRONOUS_N_D = "Isochronous+None+Data";
static const char *ISOCHRONOUS_N_F = "Isochronous+None+Feedback";
static const char *ISOCHRONOUS_N_I = "Isochronous+None+Implicit";
static const char *ISOCHRONOUS_A_X = "Isochronous+Async+?";
static const char *ISOCHRONOUS_A_D = "Isochronous+Async+Data";
static const char *ISOCHRONOUS_A_F = "Isochronous+Async+Feedback";
static const char *ISOCHRONOUS_A_I = "Isochronous+Async+Implicit";
static const char *ISOCHRONOUS_D_X = "Isochronous+Adaptive+?";
static const char *ISOCHRONOUS_D_D = "Isochronous+Adaptive+Data";
static const char *ISOCHRONOUS_D_F = "Isochronous+Adaptive+Feedback";
static const char *ISOCHRONOUS_D_I = "Isochronous+Adaptive+Implicit";
static const char *ISOCHRONOUS_S_X = "Isochronous+Sync+?";
static const char *ISOCHRONOUS_S_D = "Isochronous+Sync+Data";
static const char *ISOCHRONOUS_S_F = "Isochronous+Sync+Feedback";
static const char *ISOCHRONOUS_S_I = "Isochronous+Sync+Implicit";
static const char *BULK = "Bulk";
static const char *INTERRUPT = "Interrupt";
static const char *UNKNOWN = "Unknown";

static const char *destype(uint8_t bDescriptorType)
{
	switch (bDescriptorType) {
	case LIBUSB_DT_DEVICE:
		return DESDEV;
	case LIBUSB_DT_CONFIG:
		return DESCON;
	case LIBUSB_DT_STRING:
		return DESSTR;
	case LIBUSB_DT_INTERFACE:
		return DESINT;
	case LIBUSB_DT_ENDPOINT:
		return DESEP;
	case LIBUSB_DT_HID:
		return DESHID;
	case LIBUSB_DT_REPORT:
		return DESRPT;
	case LIBUSB_DT_PHYSICAL:
		return DESPHY;
	case LIBUSB_DT_HUB:
		return DESHUB;
	}
	return UNKNOWN;
}

static const char *epdir(uint8_t bEndpointAddress)
{
	switch (bEndpointAddress & LIBUSB_ENDPOINT_DIR_MASK) {
	case LIBUSB_ENDPOINT_IN:
		return EPIN;
	case LIBUSB_ENDPOINT_OUT:
		return EPOUT;
	}
	return EPX;
}

static const char *epatt(uint8_t bmAttributes)
{
	switch(bmAttributes & LIBUSB_TRANSFER_TYPE_MASK) {
	case LIBUSB_TRANSFER_TYPE_CONTROL:
		return CONTROL;
	case LIBUSB_TRANSFER_TYPE_ISOCHRONOUS:
		switch(bmAttributes & LIBUSB_ISO_SYNC_TYPE_MASK) {
		case LIBUSB_ISO_SYNC_TYPE_NONE:
			switch(bmAttributes & LIBUSB_ISO_USAGE_TYPE_MASK) {
			case LIBUSB_ISO_USAGE_TYPE_DATA:
				return ISOCHRONOUS_N_D;
			case LIBUSB_ISO_USAGE_TYPE_FEEDBACK:
				return ISOCHRONOUS_N_F;
			case LIBUSB_ISO_USAGE_TYPE_IMPLICIT:
				return ISOCHRONOUS_N_I;
			}
			return ISOCHRONOUS_N_X;
		case LIBUSB_ISO_SYNC_TYPE_ASYNC:
			switch(bmAttributes & LIBUSB_ISO_USAGE_TYPE_MASK) {
			case LIBUSB_ISO_USAGE_TYPE_DATA:
				return ISOCHRONOUS_A_D;
			case LIBUSB_ISO_USAGE_TYPE_FEEDBACK:
				return ISOCHRONOUS_A_F;
			case LIBUSB_ISO_USAGE_TYPE_IMPLICIT:
				return ISOCHRONOUS_A_I;
			}
			return ISOCHRONOUS_A_X;
		case LIBUSB_ISO_SYNC_TYPE_ADAPTIVE:
			switch(bmAttributes & LIBUSB_ISO_USAGE_TYPE_MASK) {
			case LIBUSB_ISO_USAGE_TYPE_DATA:
				return ISOCHRONOUS_D_D;
			case LIBUSB_ISO_USAGE_TYPE_FEEDBACK:
				return ISOCHRONOUS_D_F;
			case LIBUSB_ISO_USAGE_TYPE_IMPLICIT:
				return ISOCHRONOUS_D_I;
			}
			return ISOCHRONOUS_D_X;
		case LIBUSB_ISO_SYNC_TYPE_SYNC:
			switch(bmAttributes & LIBUSB_ISO_USAGE_TYPE_MASK) {
			case LIBUSB_ISO_USAGE_TYPE_DATA:
				return ISOCHRONOUS_S_D;
			case LIBUSB_ISO_USAGE_TYPE_FEEDBACK:
				return ISOCHRONOUS_S_F;
			case LIBUSB_ISO_USAGE_TYPE_IMPLICIT:
				return ISOCHRONOUS_S_I;
			}
			return ISOCHRONOUS_S_X;
		}
		return ISOCHRONOUS_X;
	case LIBUSB_TRANSFER_TYPE_BULK:
		return BULK;
	case LIBUSB_TRANSFER_TYPE_INTERRUPT:
		return INTERRUPT;
	}

	return UNKNOWN;
}

static void append(char **buf, char *append, size_t *off, size_t *len)
{
	int new = strlen(append);
	if ((new + *off) >= *len)
	{
		*len *= 2;
		*buf = realloc(*buf, *len);
	}

	strcpy(*buf + *off, append);
	*off += new;
}

static bool setgetdes(ssize_t count, libusb_device *dev, struct libusb_device_handle *handle, struct libusb_config_descriptor **config, int cd, char **buf, size_t *off, size_t *len)
{
	char tmp[512];
	int err;

	err = libusb_set_configuration(handle, cd);
	if (err) {
		sprintf(tmp, EOL "  ** dev %d: Failed to set config descriptor to %d, err %d",
				(int)count, cd, err);
		append(buf, tmp, off, len);
		return false;
	}

	err = libusb_get_active_config_descriptor(dev, config);
	if (err) {
		sprintf(tmp, EOL "  ** dev %d: Failed to get active config descriptor set to %d, err %d",
				(int)count, cd, err);
		append(buf, tmp, off, len);
		return false;
	}

	sprintf(tmp, EOL "  ** dev %d: Set & Got active config descriptor to %d, err %d",
			(int)count, cd, err);
	append(buf, tmp, off, len);
	return true;
}

static void usb_full(ssize_t count, libusb_device *dev, char **buf, size_t *off, size_t *len)
{
	struct libusb_device_descriptor desc;
	struct libusb_device_handle *handle;
	struct libusb_config_descriptor *config;
	const struct libusb_interface_descriptor *idesc;
	const struct libusb_endpoint_descriptor *epdesc;
	unsigned char man[STRBUFLEN+1];
	unsigned char prod[STRBUFLEN+1];
	unsigned char ser[STRBUFLEN+1];
	char tmp[512];
	int err, i, j, k;

	err = libusb_get_device_descriptor(dev, &desc);
	if (err) {
		sprintf(tmp, EOL ".USB dev %d: Failed to get descriptor, err %d",
					(int)count, err);
		append(buf, tmp, off, len);
		return;
	}

	sprintf(tmp, EOL ".USB dev %d: Device Descriptor:" EOL "\tLength: %d" EOL
			"\tDescriptor Type: %s" EOL "\tUSB: %04x" EOL "\tDeviceClass: %d" EOL
			"\tDeviceSubClass: %d" EOL "\tDeviceProtocol: %d" EOL "\tMaxPacketSize0: %d" EOL
			"\tidVendor: %04x" EOL "\tidProduct: %04x" EOL "\tDeviceRelease: %x" EOL
			"\tNumConfigurations: %d",
				(int)count, (int)(desc.bLength), destype(desc.bDescriptorType),
				desc.bcdUSB, (int)(desc.bDeviceClass), (int)(desc.bDeviceSubClass),
				(int)(desc.bDeviceProtocol), (int)(desc.bMaxPacketSize0),
				desc.idVendor, desc.idProduct, desc.bcdDevice,
				(int)(desc.bNumConfigurations));
	append(buf, tmp, off, len);

	err = libusb_open(dev, &handle);
	if (err) {
		sprintf(tmp, EOL "  ** dev %d: Failed to open, err %d", (int)count, err);
		append(buf, tmp, off, len);
		return;
	}

	if (libusb_kernel_driver_active(handle, 0) == 1) {
		sprintf(tmp, EOL "   * dev %d: kernel attached", (int)count);
		append(buf, tmp, off, len);
	}

	err = libusb_get_active_config_descriptor(dev, &config);
	if (err) {
		if (!setgetdes(count, dev, handle, &config, 1, buf, off, len)
		&&  !setgetdes(count, dev, handle, &config, 0, buf, off, len)) {
			libusb_close(handle);
			sprintf(tmp, EOL "  ** dev %d: Failed to set config descriptor to %d or %d",
					(int)count, 1, 0);
			append(buf, tmp, off, len);
			return;
		}
	}

	sprintf(tmp, EOL "     dev %d: Active Config:" EOL "\tDescriptorType: %s" EOL
			"\tNumInterfaces: %d" EOL "\tConfigurationValue: %d" EOL
			"\tAttributes: %d" EOL "\tMaxPower: %d",
				(int)count, destype(config->bDescriptorType),
				(int)(config->bNumInterfaces), (int)(config->iConfiguration),
				(int)(config->bmAttributes), (int)(config->MaxPower));
	append(buf, tmp, off, len);

	for (i = 0; i < (int)(config->bNumInterfaces); i++) {
		for (j = 0; j < config->interface[i].num_altsetting; j++) {
			idesc = &(config->interface[i].altsetting[j]);

			sprintf(tmp, EOL "     _dev %d: Interface Descriptor %d:" EOL
					"\tDescriptorType: %s" EOL "\tInterfaceNumber: %d" EOL
					"\tNumEndpoints: %d" EOL "\tInterfaceClass: %d" EOL
					"\tInterfaceSubClass: %d" EOL "\tInterfaceProtocol: %d",
						(int)count, j, destype(idesc->bDescriptorType),
						(int)(idesc->bInterfaceNumber),
						(int)(idesc->bNumEndpoints),
						(int)(idesc->bInterfaceClass),
						(int)(idesc->bInterfaceSubClass),
						(int)(idesc->bInterfaceProtocol));
			append(buf, tmp, off, len);

			for (k = 0; k < (int)(idesc->bNumEndpoints); k++) {
				epdesc = &(idesc->endpoint[k]);

				sprintf(tmp, EOL "     __dev %d: Interface %d Endpoint %d:" EOL
						"\tDescriptorType: %s" EOL
						"\tEndpointAddress: %s0x%x" EOL
						"\tAttributes: %s" EOL "\tMaxPacketSize: %d" EOL
						"\tInterval: %d" EOL "\tRefresh: %d",
							(int)count, (int)(idesc->bInterfaceNumber), k,
							destype(epdesc->bDescriptorType),
							epdir(epdesc->bEndpointAddress),
							(int)(epdesc->bEndpointAddress),
							epatt(epdesc->bmAttributes),
							epdesc->wMaxPacketSize,
							(int)(epdesc->bInterval),
							(int)(epdesc->bRefresh));
				append(buf, tmp, off, len);
			}
		}
	}

	libusb_free_config_descriptor(config);
	config = NULL;

	err = libusb_get_string_descriptor_ascii(handle, desc.iManufacturer, man, STRBUFLEN);
	if (err < 0)
		sprintf((char *)man, "** err(%d)", err);

	err = libusb_get_string_descriptor_ascii(handle, desc.iProduct, prod, STRBUFLEN);
	if (err < 0)
		sprintf((char *)prod, "** err(%d)", err);

	err = libusb_get_string_descriptor_ascii(handle, desc.iSerialNumber, ser, STRBUFLEN);
	if (err < 0)
		sprintf((char *)ser, "** err(%d)", err);

	sprintf(tmp, EOL "     dev %d: More Info:" EOL "\tManufacturer: '%s'" EOL
			"\tProduct: '%s'" EOL "\tSerial '%s'",
				(int)count, man, prod, ser);
	append(buf, tmp, off, len);

	libusb_close(handle);
}

// Function to dump all USB devices
static void usb_all()
{
	libusb_device **list;
	ssize_t count, i;
	char *buf;
	size_t len, off;

	count = libusb_get_device_list(NULL, &list);
	if (count < 0) {
		applog(LOG_ERR, "USB all: failed, err %d", (int)count);
		return;
	}

	if (count == 0)
		applog(LOG_WARNING, "USB all: found no devices");
	else
	{
		len = 10000;
		buf = malloc(len+1);

		sprintf(buf, "USB all: found %d devices", (int)count);

		off = strlen(buf);

		for (i = 0; i < count; i++)
			usb_full(i, list[i], &buf, &off, &len);

		applog(LOG_WARNING, "%s", buf);

		free(buf);
	}

	libusb_free_device_list(list, 1);
}

static void cgusb_check_init()
{
	mutex_lock(&cgusb_lock);

	if (list_lock == NULL) {
		list_lock = calloc(1, sizeof(*list_lock));
		mutex_init(list_lock);

		if (opt_usbdump >= 0) {
			libusb_set_debug(NULL, opt_usbdump);
			usb_all();
		}
	}

	mutex_unlock(&cgusb_lock);
}

static bool in_use(libusb_device *dev, bool lock)
{
	struct usb_list *usb_tmp;
	bool used = false;
	uint8_t bus_number;
	uint8_t device_address;

	bus_number = libusb_get_bus_number(dev);
	device_address = libusb_get_device_address(dev);

	if (lock)
		mutex_lock(list_lock);

	if ((usb_tmp = usb_head))
		do {
			if (bus_number == usb_tmp->bus_number
			&&  device_address == usb_tmp->device_address) {
				used = true;
				break;
			}

			usb_tmp = usb_tmp->next;

		} while (usb_tmp != usb_head);

	if (lock)
		mutex_unlock(list_lock);

	return used;
}

static void add_used(libusb_device *dev, bool lock)
{
	struct usb_list *usb_tmp;
	char buf[128];
	uint8_t bus_number;
	uint8_t device_address;

	bus_number = libusb_get_bus_number(dev);
	device_address = libusb_get_device_address(dev);

	if (lock)
		mutex_lock(list_lock);

	if (in_use(dev, false)) {
		if (lock)
			mutex_unlock(list_lock);

		sprintf(buf, "add_used() duplicate bus_number %d device_address %d",
				bus_number, device_address);
		quit(1, buf);
	}

	usb_tmp = malloc(sizeof(*usb_tmp));

	usb_tmp->bus_number = bus_number;
	usb_tmp->device_address = device_address;

	if (usb_head) {
		// add to end
		usb_tmp->prev = usb_head->prev;
		usb_tmp->next = usb_head;
		usb_head->prev = usb_tmp;
		usb_tmp->prev->next = usb_tmp;
	} else {
		usb_tmp->prev = usb_tmp;
		usb_tmp->next = usb_tmp;
		usb_head = usb_tmp;
	}

	if (lock)
		mutex_unlock(list_lock);
}

static void release(uint8_t bus_number, uint8_t device_address, bool lock)
{
	struct usb_list *usb_tmp;
	bool found = false;
	char buf[128];

	if (lock)
		mutex_lock(list_lock);

	usb_tmp = usb_head;
	if (usb_tmp)
		do {
			if (bus_number == usb_tmp->bus_number
			&&  device_address == usb_tmp->device_address) {
				found = true;
				break;
			}

			usb_tmp = usb_tmp->next;

		} while (usb_tmp != usb_head);

	if (!found) {
		if (lock)
			mutex_unlock(list_lock);

		sprintf(buf, "release() unknown: bus_number %d device_address %d",
				bus_number, device_address);
		quit(1, buf);
	}

	if (usb_tmp->next == usb_tmp) {
		usb_head = NULL;
	} else {
		usb_tmp->next->prev = usb_tmp->prev;
		usb_tmp->prev->next = usb_tmp->next;
	}

	if (lock)
		mutex_unlock(list_lock);

	free(usb_tmp);
}

static void release_dev(libusb_device *dev, bool lock)
{
	uint8_t bus_number;
	uint8_t device_address;

	bus_number = libusb_get_bus_number(dev);
	device_address = libusb_get_device_address(dev);

	release(bus_number, device_address, lock);
}

#if 0
static void release_cgusb(struct cg_usb_device *cgusb, bool lock)
{
	release(cgusb->bus_number, cgusb->device_address, lock);
}
#endif

static struct cg_usb_device *free_cgusb(struct cg_usb_device *cgusb)
{
	if (cgusb->serial_string && cgusb->serial_string != BLANK)
		free(cgusb->serial_string);

	if (cgusb->manuf_string && cgusb->manuf_string != BLANK)
		free(cgusb->manuf_string);

	if (cgusb->prod_string && cgusb->prod_string != BLANK)
		free(cgusb->prod_string);

	free(cgusb->descriptor);

	free(cgusb);

	return NULL;
}

void usb_uninit(struct cgpu_info *cgpu)
{
	libusb_release_interface(cgpu->usbdev->handle, cgpu->usbdev->found->interface);
	libusb_close(cgpu->usbdev->handle);
	cgpu->usbdev = free_cgusb(cgpu->usbdev);
}

bool usb_init(struct cgpu_info *cgpu, struct libusb_device *dev, struct usb_find_devices *found)
{
	struct cg_usb_device *cgusb = NULL;
	struct libusb_config_descriptor *config;
	const struct libusb_interface_descriptor *idesc;
	const struct libusb_endpoint_descriptor *epdesc;
	unsigned char strbuf[STRBUFLEN+1];
	int err, i, j, k;

	cgusb = calloc(1, sizeof(*cgusb));
	cgusb->found = found;
	cgusb->descriptor = calloc(1, sizeof(*(cgusb->descriptor)));

	err = libusb_get_device_descriptor(dev, cgusb->descriptor);
	if (err) {
		applog(LOG_ERR, "USB init failed to get descriptor, err %d", err);
		goto dame;
	}

	err = libusb_open(dev, &(cgusb->handle));
	if (err) {
		switch (err)
		{
		case LIBUSB_ERROR_ACCESS:
			applog(LOG_ERR, "USB init open device failed, err %d, you dont have priviledge to access the device", err);
			break;
#ifdef WIN32
		// Windows specific message
		case LIBUSB_ERROR_NOT_SUPPORTED:
			applog(LOG_ERR, "USB init, open device failed, err %d, you need to install a Windows USB driver for the device", err);
			break;
#endif
		default:
			applog(LOG_ERR, "USB init, open device failed, err %d", err);
		}

		goto dame;
	}

	if (libusb_kernel_driver_active(cgusb->handle, 0) == 1) {
		applog(LOG_WARNING, "USB init, kernel attached ...");
		if (libusb_detach_kernel_driver(cgusb->handle, 0) == 0)
			applog(LOG_WARNING, "USB init, kernel detached successfully");
		else
			applog(LOG_WARNING, "USB init, kernel detach failed :(");
		
	}

	err = libusb_set_configuration(cgusb->handle, found->config);
	if (err) {
		applog(LOG_DEBUG, "USB init, failed to set config to %d, err %d",
			found->config, err);
		goto cldame;
	}

	err = libusb_get_active_config_descriptor(dev, &config);
	if (err) {
		applog(LOG_DEBUG, "USB init, failed to get config descriptor %d, err %d",
			found->config, err);
		goto cldame;
	}

	if ((int)(config->bNumInterfaces) < found->interface)
		goto cldame;

	for (i = 0; i < found->epcount; i++)
		found->eps[i].found = false;

	for (i = 0; i < config->interface[found->interface].num_altsetting; i++) {
		idesc = &(config->interface[found->interface].altsetting[i]);
		for (j = 0; j < (int)(idesc->bNumEndpoints); j++) {
			epdesc = &(idesc->endpoint[j]);
			for (k = 0; k < found->epcount; k++) {
				if (!found->eps[k].found) {
					if (epdesc->bmAttributes == found->eps[k].att
					&&  epdesc->wMaxPacketSize >= found->eps[k].size
					&&  epdesc->bEndpointAddress == found->eps[k].ep) {
						found->eps[k].found = true;
						break;
					}
				}
			}
		}
	}

	for (i = 0; i < found->epcount; i++)
		if (found->eps[i].found == false)
			goto cldame;

	err = libusb_claim_interface(cgusb->handle, found->interface);
	if (err) {
		applog(LOG_DEBUG, "USB init, claim interface %d failed, err %d",
			found->interface, err);
		goto cldame;
	}

	cgusb->bus_number = libusb_get_bus_number(dev);
	cgusb->device_address = libusb_get_device_address(dev);
	cgusb->usbver = cgusb->descriptor->bcdUSB;

// TODO: allow this with the right version of the libusb include and running library
//	cgusb->speed = libusb_get_device_speed(dev);

	err = libusb_get_string_descriptor_ascii(cgusb->handle,
				cgusb->descriptor->iProduct, strbuf, STRBUFLEN);
	if (err > 0)
		cgusb->prod_string = strdup((char *)strbuf);
	else
		cgusb->prod_string = (char *)BLANK;

	err = libusb_get_string_descriptor_ascii(cgusb->handle,
				cgusb->descriptor->iManufacturer, strbuf, STRBUFLEN);
	if (err > 0)
		cgusb->manuf_string = strdup((char *)strbuf);
	else
		cgusb->manuf_string = (char *)BLANK;

	err = libusb_get_string_descriptor_ascii(cgusb->handle,
				cgusb->descriptor->iSerialNumber, strbuf, STRBUFLEN);
	if (err > 0)
		cgusb->serial_string = strdup((char *)strbuf);
	else
		cgusb->serial_string = (char *)BLANK;

// TODO: ?
//	cgusb->fwVersion <- for temp1/temp2 decision? or serial? (driver-modminer.c)
//	cgusb->interfaceVersion

	applog(LOG_DEBUG, "USB init device bus_number=%d device_address=%d usbver=%04x prod='%s' manuf='%s' serial='%s'", (int)(cgusb->bus_number), (int)(cgusb->device_address), cgusb->usbver, cgusb->prod_string, cgusb->manuf_string, cgusb->serial_string);

	cgpu->usbdev = cgusb;

	libusb_free_config_descriptor(config);

	return true;

cldame:

	libusb_close(cgusb->handle);

dame:

	if (config)
		libusb_free_config_descriptor(config);

	cgusb = free_cgusb(cgusb);

	return false;
}

static bool usb_check_device(struct device_api *api, struct libusb_device *dev, struct usb_find_devices *look)
{
	struct libusb_device_descriptor desc;
	int err;

	err = libusb_get_device_descriptor(dev, &desc);
	if (err) {
		applog(LOG_DEBUG, "USB check device: Failed to get descriptor, err %d", err);
		return false;
	}

	if (desc.idVendor != look->idVendor || desc.idProduct != look->idProduct) {
		applog(LOG_DEBUG, "%s looking for %04x:%04x but found %04x:%04x instead",
			api->name, look->idVendor, look->idProduct, desc.idVendor, desc.idProduct);

		return false;
	}

	applog(LOG_DEBUG, "%s looking for and found %04x:%04x",
		api->name, look->idVendor, look->idProduct);

	return true;
}

static struct usb_find_devices *usb_check_each(int drv, struct device_api *api, struct libusb_device *dev)
{
	struct usb_find_devices *found;
	int i;

	for (i = 0; find_dev[i].drv != DRV_LAST; i++)
		if (find_dev[i].drv == drv) {
			if (usb_check_device(api, dev, &(find_dev[i]))) {
				found = malloc(sizeof(*found));
				memcpy(found, &(find_dev[i]), sizeof(*found));
				return found;
			}
		}

	return NULL;
}

static struct usb_find_devices *usb_check(__maybe_unused struct device_api *api, __maybe_unused struct libusb_device *dev)
{
#ifdef USE_BITFORCE
	if (api == &bitforce_api)
		return usb_check_each(DRV_BITFORCE, api, dev);
#endif

#ifdef USE_ICARUS
	if (api == &icarus_api)
		return usb_check_each(DRV_ICARUS, api, dev);
#endif

#ifdef USE_MODMINER
	if (api == &modminer_api)
		return usb_check_each(DRV_MODMINER, api, dev);
#endif

	return NULL;
}

void usb_detect(struct device_api *api, bool (*device_detect)(struct libusb_device *, struct usb_find_devices *))
{
	libusb_device **list;
	ssize_t count, i;
	struct usb_find_devices *found;

	cgusb_check_init();
	
	count = libusb_get_device_list(NULL, &list);
	if (count < 0) {
		applog(LOG_DEBUG, "USB scan devices: failed, err %d", count);
		return;
	}

	if (count == 0)
		applog(LOG_DEBUG, "USB scan devices: found no devices");

	for (i = 0; i < count; i++) {
		mutex_lock(list_lock);

		if (in_use(list[i], false))
			mutex_unlock(list_lock);
		else {
			add_used(list[i], false);

			mutex_unlock(list_lock);

			found = usb_check(api, list[i]);
			if (!found)
				release_dev(list[i], true);
			else
				if (!device_detect(list[i], found))
					release_dev(list[i], true);
		}
	}

	libusb_free_device_list(list, 1);
}

int _usb_read(struct cgpu_info *cgpu, int ep, char *buf, size_t bufsiz, int *processed, unsigned int timeout, int eol)
{
	struct cg_usb_device *usbdev = cgpu->usbdev;
	int err, got, tot;

	if (eol == -1) {
		got = 0;
		err = libusb_bulk_transfer(usbdev->handle,
				usbdev->found->eps[ep].ep,
				(unsigned char *)buf,
				bufsiz, &got,
				timeout == DEVTIMEOUT ? usbdev->found->timeout : timeout);

		*processed = got;

		return err;
	}

	tot = 0;
	while (bufsiz) {
		got = 0;
		err = libusb_bulk_transfer(usbdev->handle,
				usbdev->found->eps[ep].ep,
				(unsigned char *)buf,
				1, &got,
				timeout == DEVTIMEOUT ? usbdev->found->timeout : timeout);

		tot += got;

		if (err)
			break;

		if (eol == buf[0])
			break;

		buf += got;
		bufsiz -= got;
	}

	*processed = tot;

	return err;
}

int _usb_write(struct cgpu_info *cgpu, int ep, char *buf, size_t bufsiz, int *processed, unsigned int timeout)
{
	struct cg_usb_device *usbdev = cgpu->usbdev;
	int err, sent;

	sent = 0;
	err = libusb_bulk_transfer(usbdev->handle,
			usbdev->found->eps[ep].ep,
			(unsigned char *)buf,
			bufsiz, &sent,
			timeout == DEVTIMEOUT ? usbdev->found->timeout : timeout);

	*processed = sent;

	return err;
}

void usb_cleanup()
{
	// TODO:
}
