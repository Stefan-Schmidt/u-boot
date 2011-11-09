/*
 * Copyright (C) 2010 Samsung Electrnoics
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

#include <linux/compiler.h>

#define MBR_RESERVED_SIZE	0x8000
#define MBR_START_OFS_BLK	(0x500000 / 512)

struct mbr_partition {
	char status;
	char f_chs[3];
	char partition_type;
	char l_chs[3];
	int lba;
	int nsectors;
} __packed;

struct mbr {
	char code_area[440];
	char disk_signature[4];
	char nulls[2];
	struct mbr_partition parts[4];
	unsigned short signature;
};

extern unsigned int mbr_offset[16];
extern unsigned int mbr_parts;

void set_mbr_dev(int dev);
void set_mbr_info(char *ramaddr, unsigned int len, unsigned int reserved);
void set_mbr_table(unsigned int start_addr, int parts,
		unsigned int *blocks, unsigned int *part_offset);
int get_mbr_table(unsigned int *part_offset);
