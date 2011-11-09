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

#ifndef __DFU_BACKEND_H__
#define __DFU_BACKEND_H__

struct mbr_part_data {
	unsigned long		offset; /* #sectors from mmc begin */
	unsigned long		length; /* #sectors in this partition*/
	u8			primary; /* != 0 if primary, 0 if extended */
};

typedef int (*rw_op)(void *buf, unsigned int len, unsigned long from);
typedef int (*erase_op)(unsigned int len, unsigned long from);

struct flash_entity_ctx {
	unsigned long		offset;	/* offset into the device */
	unsigned long		length; /* size of the entity */
	u8			*buf; /* buffer for one chunk */
	unsigned long		buf_len; /* one chunk length */
	unsigned int		buffered; /* # available bytes in buf */
	unsigned int		num_done; /* total bytes handled */
	rw_op			read; /* chunk read op for this medium */
	rw_op			write; /* chunk write op for this medium */
	erase_op		erase; /* erase op for this medium or NULL */
	struct flash_entity	*this_entity; /* the containing entity */
	void			*associated; /* related entity, if any */
};

extern void board_dfu_init(void);
extern int board_dfu_cleanup(void);
extern int usb_gadget_handle_interrupts(void);

extern int read_mmc(void *buf, unsigned int len, unsigned long from);
extern int write_mmc(void *buf, unsigned int len, unsigned long from);

extern block_dev_desc_t *set_fat_dev(block_dev_desc_t *d);
extern int set_fat_part_num(int pnum);
extern const char *set_fat_filename(const char *fn);
extern int read_fat(void *buf, unsigned int len, unsigned long from);
extern int write_fat(void *buf, unsigned int len, unsigned long from);

extern int read_mbr(struct mmc *mmc, struct mbr_part_data *pd, int *extended_lba, int mmc_mbr_dev);

extern int read_block(void *ctx, unsigned int n, void *buf);
extern int write_block(void *ctx, unsigned int n, void *buf);
extern int generic_prepare(void *ctx, u8 mode);
extern int generic_finish(void *ctx, u8 mode);

#define MMC_SECTOR_SZ		512
#define MMC_FAT_BLOCK_SZ	(4 * 1024 * 1024)

#endif
