/*
 * Copyright 2012 Andrew Smith
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the Free
 * Software Foundation; either version 3 of the License, or (at your option)
 * any later version.  See COPYING for more details.
 */

#ifndef USBUTILS_H
#define USBUTILS_H

#include <libusb-1.0/libusb.h>

// TODO: windows may need a longer default?
// or conversely - linux could have an even shorter default?
#define DEFAULT_USB_TIMEOUT 100 // ms

struct cg_usb_device {
	libusb_device_handle *handle;
	pthread_mutex_t *mutex;
	struct libusb_device_descriptor *descriptor;
	uint8_t bus_number;
	uint8_t device_address;
	int speed;
	int max_packet_size;
	unsigned char ep_in;
	unsigned char ep_out;
	char *prod_string;
	char *manuf_string;
	char *serial_string;
	unsigned char fwVersion;	// ??
	unsigned char interfaceVersion;	// ??
};

struct usb_devices {
	libusb_device *dev;
	unsigned char ep_in;
	unsigned char ep_out;
	unsigned char filler[2];
	struct usb_devices *prev;
	struct usb_devices *next;
};

struct device_api;
struct cgpu_info;

void usb_uninit(struct cgpu_info *cgpu);
bool usb_init(struct cgpu_info *cgpu, struct libusb_device *dev, unsigned char ep_in, unsigned char ep_out);
void usb_detect(struct device_api *api, bool (*device_detect)(struct libusb_device *, unsigned char, unsigned char));
int _usb_read(struct cgpu_info *cgpu, char *buf, size_t bufsiz, int *processed, unsigned int timeout, int eol);
int _usb_write(struct cgpu_info *cgpu, char *buf, size_t bufsiz, int *processed, unsigned int timeout);

#define usb_read(cgpu, buf, bufsiz, read) \
	_usb_read(cgpu, buf, bufsiz, read, DEFAULT_USB_TIMEOUT, -1)

#define usb_read_timeout(cgpu, buf, bufsiz, read, timeout) \
	_usb_read(cgpu, buf, bufsiz, read, timeout, -1)

#define usb_write(cgpu, buf, bufsiz, wrote) \
	_usb_write(cgpu, buf, bufsiz, wrote, DEFAULT_USB_TIMEOUT)

#define usb_write_timeout(cgpu, buf, bufsiz, wrote, timeout) \
	_usb_write(cgpu, buf, bufsiz, wrote, timeout)

#endif
