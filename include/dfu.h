/*
 * dfu.h - Device Firmware Upgrade
 *
 * copyright (c) 2011 samsung electronics
 * author: andrzej pietrasiewicz <andrzej.p@samsung.com>
 *
 * this program is free software; you can redistribute it and/or modify
 * it under the terms of the gnu general public license as published by
 * the free software foundation; either version 2 of the license, or
 * (at your option) any later version.
 *
 * this program is distributed in the hope that it will be useful,
 * but without any warranty; without even the implied warranty of
 * merchantability or fitness for a particular purpose.  see the
 * gnu general public license for more details.
 *
 * you should have received a copy of the gnu general public license
 * along with this program; if not, write to the free software
 * foundation, inc., 59 temple place, suite 330, boston, ma  02111-1307  usa
 */

#ifndef __DFU_H__
#define __DFU_H__

extern int dfu_init(void);
extern int dfu_cleanup(void);

#endif
