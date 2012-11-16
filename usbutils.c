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

#define EPI(x) (LIBUSB_ENDPOINT_IN | (unsigned char)(x))
#define EPO(x) (LIBUSB_ENDPOINT_OUT | (unsigned char)(x))

struct find_devices {
	int drv;
	const char *name;
	uint16_t idVendor;
	uint16_t idProduct;
	bool fixed_endpoint;	// TODO: search when false
	unsigned char ep_in;
	unsigned char ep_out;
};

// Currently they are all bulk endpoints (since the code is currently only bulk)
// TODO?: add variable timeouts
static struct find_devices find_dev[] = {
#ifdef USE_ICARUS
	{ DRV_ICARUS, 	"ICA",	0x067b,	0x0230,	true,	EPI(3),	EPO(2) },
	{ DRV_ICARUS, 	"LLT",	0x0403,	0x6001,	false,	EPI(0),	EPO(0) },
	{ DRV_ICARUS, 	"CM1",	0x067b,	0x0230,	false,	EPI(0),	EPO(0) },
#endif
#ifdef USE_BITFORCE
	{ DRV_BITFORCE,	"BFL",	0x0403,	0x6014,	true,	EPI(1),	EPO(2) },
#endif
#ifdef USE_MODMINER
	{ DRV_MODMINER,	"MMQ",	0x1fc9,	0x0003,	true,	EPI(3),	EPO(3) },
#endif
	{ DRV_LAST,	NULL,	0,	0,	true,	EPI(0),	EPO(0) }
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

static void cgusb_check_init()
{
	mutex_lock(&cgusb_lock);

	if (list_lock == NULL) {
		list_lock = calloc(sizeof(*list_lock), 1);
		mutex_init(list_lock);
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

// TODO: need a version that will release based on a cg_usb_device
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

static void release_cgusb(struct cg_usb_device *cgusb, bool lock)
{
	release(cgusb->bus_number, cgusb->device_address, lock);
}

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
	libusb_release_interface(cgpu->usbdev->handle, 0);
	libusb_close(cgpu->usbdev->handle);
	cgpu->usbdev = free_cgusb(cgpu->usbdev);
}

bool usb_init(struct cgpu_info *cgpu, struct libusb_device *dev, unsigned char ep_in, unsigned char ep_out)
{
	struct cg_usb_device *cgusb = NULL;
	unsigned char strbuf[STRBUFLEN+1];
	int err;

	cgusb = calloc(sizeof(*cgusb), 1);
	cgusb->descriptor = calloc(sizeof(*(cgusb->descriptor)), 1);

	err = libusb_get_device_descriptor(dev, cgusb->descriptor);
	if (err) {
		applog(LOG_ERR, "USB open device failed to get descriptor, err %d", err);
		goto dame;
	}

	err = libusb_open(dev, &(cgusb->handle));
	if (err) {
		if (err == -3)
			applog(LOG_ERR, "USB open device failed, err %d, you dont have priviledge to access the device", err);
		else
			applog(LOG_ERR, "USB open device failed, err %d", err);

		goto dame;
	}

	if (libusb_kernel_driver_active(cgusb->handle, 0) == 1) {
		applog(LOG_WARNING, "USB open, kernel attached ...");
		if (libusb_detach_kernel_driver(cgusb->handle, 0) == 0)
			applog(LOG_WARNING, "USB open, kernel detached successfully");
		else
			applog(LOG_WARNING, "USB open, kernel detach failed :(");
		
	}

	cgusb->ep_in = ep_in;
	cgusb->ep_out = ep_out;
	cgusb->bus_number = libusb_get_bus_number(dev);
	cgusb->device_address = libusb_get_device_address(dev);

// TODO: allow this with the right version of the libusb include and running library
//	cgusb->speed = libusb_get_device_speed(dev);

	cgusb->max_packet_size = libusb_get_max_packet_size(dev, ep_out);

	// TODO: these 3 get garbage replies
	err = libusb_get_descriptor(cgusb->handle, LIBUSB_DT_STRING,
					cgusb->descriptor->iProduct, strbuf, STRBUFLEN);
	if (err > 0)
		cgusb->prod_string = strdup((char *)strbuf);
	else
		cgusb->prod_string = (char *)BLANK;

	err = libusb_get_descriptor(cgusb->handle, LIBUSB_DT_STRING,
					cgusb->descriptor->iManufacturer, strbuf, STRBUFLEN);
	if (err > 0)
		cgusb->manuf_string = strdup((char *)strbuf);
	else
		cgusb->manuf_string = (char *)BLANK;

	err = libusb_get_descriptor(cgusb->handle, LIBUSB_DT_STRING,
					cgusb->descriptor->iSerialNumber, strbuf, STRBUFLEN);
	if (err > 0)
		cgusb->serial_string = strdup((char *)strbuf);
	else
		cgusb->serial_string = (char *)BLANK;

// TODO: ?
//	cgusb->fwVersion
//	cgusb->interfaceVersion

applog(LOG_ERR, "USB open device ep_in=%d ep_out=%d bus_number=%d device_address=%d max_packet_size=%d prod='%s' manuf='%s' serial='%s'", cgusb->ep_in, cgusb->ep_out, (int)(cgusb->bus_number), (int)(cgusb->device_address), (int)(cgusb->max_packet_size), cgusb->prod_string, cgusb->manuf_string, cgusb->serial_string);

	err = libusb_claim_interface(cgusb->handle, 0);
	if (err) {
		applog(LOG_ERR, "USB open, claim device failed, err %d", err);

		libusb_close(cgusb->handle);

		goto dame;
	}

	cgpu->usbdev = cgusb;

	return true;
dame:
	cgusb = free_cgusb(cgusb);

	return false;
}

static bool usb_check_device(struct device_api *api, struct libusb_device *dev, uint16_t idVendor, uint16_t idProduct)
{
	struct libusb_device_descriptor desc;
	int err;

	err = libusb_get_device_descriptor(dev, &desc);
	if (err) {
		applog(LOG_DEBUG, "USB check device: Failed to get descriptor, err %d", err);
		return false;
	}
	if (desc.idVendor != idVendor || desc.idProduct != idProduct) {
		applog(LOG_DEBUG, "%s looking for %04x:%04x but found %04x:%04x instead",
			api->name, idVendor, idProduct, desc.idVendor, desc.idProduct);

		return false;
	}

	applog(LOG_DEBUG, "%s looking for and found %04x:%04x", api->name, idVendor, idProduct);

	return true;
}

static struct find_devices *usb_check_each(int drv, struct device_api *api, struct libusb_device *dev)
{
	int i;

	for (i = 0; find_dev[i].drv != DRV_LAST; i++)
		if (find_dev[i].drv == drv) {
			// TODO: handle find 64 byte Bulk endpont
			// This code currently assumes a fixed_endpoint
			if (usb_check_device(api, dev, find_dev[i].idVendor, find_dev[i].idProduct))
				return &(find_dev[i]);
		}

	return NULL;
}

static struct find_devices *usb_check(__maybe_unused struct device_api *api, __maybe_unused struct libusb_device *dev)
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

void usb_detect(struct device_api *api, bool (*device_detect)(struct libusb_device *, unsigned char, unsigned char))
{
	libusb_device **list;
	ssize_t count, i;
	struct find_devices *found;

	cgusb_check_init();
	
	count = libusb_get_device_list(NULL, &list);
	if (count < 0) {
		applog(LOG_DEBUG, "USB scan devices: Failed to find any USB devices, err %d", count);
		return;
	}


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
				if (!device_detect(list[i], found->ep_in, found->ep_out))
					release_dev(list[i], true);
		}
	}

	libusb_free_device_list(list, 1);
}

int _usb_read(struct cgpu_info *cgpu, char *buf, size_t bufsiz, int *processed, unsigned int timeout, int eol)
{
	struct cg_usb_device *usbdev = cgpu->usbdev;
	int err, got, tot;

	if (eol == -1) {
		got = 0;
		err = libusb_bulk_transfer(usbdev->handle,
						usbdev->ep_in,
						(unsigned char *)buf,
						bufsiz, &got,
						timeout);

		*processed = got;

		return err;
	}

	tot = 0;
	while (bufsiz) {
		got = 0;
		err = libusb_bulk_transfer(usbdev->handle,
						usbdev->ep_in,
						(unsigned char *)buf,
						1, &got,
						timeout);

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

int _usb_write(struct cgpu_info *cgpu, char *buf, size_t bufsiz, int *processed, unsigned int timeout)
{
	struct cg_usb_device *usbdev = cgpu->usbdev;
	int err, sent;

	sent = 0;
	err = libusb_bulk_transfer(usbdev->handle,
					usbdev->ep_out,
					(unsigned char *)buf,
					bufsiz, &sent,
					timeout);

	*processed = sent;

	return err;
}

void usb_cleanup()
{
	// TODO:
}
