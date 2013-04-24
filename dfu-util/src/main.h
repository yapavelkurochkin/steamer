/*
 * dfu-util
 *
 * (C) 2007-2008 by OpenMoko, Inc., (C) 2010 STC Metrotek
 * Written by Harald Welte <laforge@openmoko.org>,
 * Modified by Nikolay Zamotaev <fhunter@metrotek.spb.ru>
 *
 * Based on existing code of dfu-programmer-0.4
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

#ifndef _MAIN_H_
#define _MAIN_H_

#include <stdio.h>
#include <string.h>
#include <getopt.h>
#include <usb.h>
#include <errno.h>

#include "dfu.h"
#include "usb_dfu.h"
#include "stm32dfu.h"
#include "dfu-version.h"
#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#define DFU_IFF_DFU		0x0001	/* DFU Mode, (not Runtime) */
#define DFU_IFF_VENDOR		0x0100
#define DFU_IFF_PRODUCT		0x0200
#define DFU_IFF_CONFIG		0x0400
#define DFU_IFF_IFACE		0x0800
#define DFU_IFF_ALT		0x1000
#define DFU_IFF_DEVNUM		0x2000
#define DFU_IFF_PATH		0x4000

struct usb_vendprod {
	u_int16_t vendor;
	u_int16_t product;
};

struct dfu_if {
	u_int16_t vendor;
	u_int16_t product;
	u_int8_t configuration;
	u_int8_t interface;
	u_int8_t altsetting;
	int bus;
	u_int8_t devnum;
	const char *path;
	unsigned int flags;
	struct usb_device *dev;

	struct usb_dev_handle *dev_handle;
};

#define MAX_STR_LEN 64

static struct option opts[] = {
	{ "help", 0, 0, 'h' },
	{ "version", 0, 0, 'V' },
	{ "list", 0, 0, 'l' },
	{ "device", 1, 0, 'd' },
	{ "offset", 1, 0, 'o' },
	{ "download", 1, 0, 'D' },
	{ "execute", 0, 0, 'X' },
};


/* FLAGS */
#define NO_FLAGS			0x00
#define DO_EXECUTE			0x01
#define DO_ERASE			0x02

#endif
