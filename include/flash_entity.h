/*
 * flash_entity.h - flashable area description
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

#ifndef FLASH_ENTITY_H_
#define FLASH_ENTITY_H_

#define FLASH_READ			0
#define FLASH_WRITE			1

struct flash_entity {
	char				*name;
	void				*ctx;
	int (*prepare)(void *ctx, u8 mode);
	int (*read_block)(void *ctx, unsigned int n, void *buf);
	int (*write_block)(void *ctx, unsigned int n, void *buf);
	int (*finish)(void *ctx, u8 mode);
};

void register_flash_entities(struct flash_entity *flents, int n);

#endif /* FLASH_ENTITY_H_ */
