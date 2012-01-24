#include <common.h>
#include <nand.h>
#include <jffs2/load_kernel.h>
#include <dfu_backend.h>
#include <flash_entity.h>
int mtdparts_init(void);
extern struct list_head devices;

struct dnload_state {
	nand_info_t *nand;
	struct part_info *part;
	unsigned int part_net_size;	/* net size (no bad blocks) of part */

	nand_erase_options_t erase_opts;

	unsigned char *ptr;	/* pointer to next empty byte in buffer */
	unsigned int off;	/* offset of current erase page in flash chip */
	unsigned char *buf;	/* pointer to allocated erase page buffer */

	/* unless doing an atomic transfer, we use the static buffer below.
	 * This saves us from having to clean up dynamic allications in the
	 * various error paths of the code.  Also, it will always work, no
	 * matter what the memory situation is. */
	unsigned char _buf[0x20000];	/* FIXME: depends flash page size */
};

static struct dnload_state _dnstate;

/* Return the 'net size' of the partition (i.e. excluding any bad blocks) */
unsigned int nand_net_part_size(struct part_info *part)
{
	struct mtd_info *mtd;
	unsigned int offs;
	unsigned int bb_delta = 0;

	if (!part || !part->dev || !part->dev->id ||
		part->dev->id->num >= CONFIG_SYS_MAX_NAND_DEVICE)
		return 0;

	mtd = &nand_info[part->dev->id->num];

	for (offs = part->offset; offs < part->offset + part->size;
		offs += mtd->erasesize) {
		if (nand_isbad_bbt(mtd, offs, 0))
			bb_delta += mtd->erasesize;
	}

	return part->size - bb_delta;
}

static struct part_info *get_partition_nand(int idx)
{
	struct mtd_device *dev;
	struct part_info *part;
	struct list_head *pentry;
	int i;

	if (mtdparts_init())
		return NULL;
	if (list_empty(&devices))
		return NULL;

	dev = list_entry(devices.next, struct mtd_device, link);
	i = 0;
	list_for_each(pentry, &dev->parts) {
		if (i == idx)  {
			part = list_entry(pentry, struct part_info, link);
			return part;
		}
		i++;
	}

	return NULL;
}

static int initialize_ds_nand(struct usb_device_instance *dev,
			      struct dnload_state *ds)
{
	ds->part = get_partition_nand(dev->alternate - 1);
	if (!ds->part) {
		printf("DFU: unable to find partition %u\b", dev->alternate-1);
			dev->dfu_state = DFU_STATE_dfuERROR;
		dev->dfu_status = DFU_STATUS_errADDRESS;
		return RET_STALL;
	}
	ds->nand = &nand_info[ds->part->dev->id->num];
	ds->off = ds->part->offset;
	ds->part_net_size = nand_net_part_size(ds->part);

	if (ds->nand->erasesize > sizeof(ds->_buf)) {
		printf("Warning - NAND ERASESIZE bigger than static buffer\n");
		ds->buf = malloc(ds->nand->erasesize);
		if (!ds->buf) {
			printf("DFU: can't allocate %u bytes\n",
				ds->nand->erasesize);
			dev->dfu_state = DFU_STATE_dfuERROR;
			dev->dfu_status = DFU_STATUS_errADDRESS;
			return RET_STALL;
		}
	} else
		ds->buf = ds->_buf;

	ds->ptr = ds->buf;

	memset(&ds->erase_opts, 0, sizeof(ds->erase_opts));
	ds->erase_opts.quiet = 1;
	/* FIXME: do this more dynamic */
	if ((!strcmp(ds->part->name, "rootfs")) ||
	    (!strcmp(ds->part->name, "fs")))
		ds->erase_opts.jffs2 = 1;

	/* FIXME: How to set these options without write_opts?
	 * ds->write_opts.pad = 1;
	 * ds->write_opts.blockalign = 1;
	 * ds->write_opts.quiet = 1;
	*/

	debug("initialize_ds_nand(dev=%p, ds=%p): ", dev, ds);
	debug("nand=%p, ptr=%p, buf=%p, off=0x%x\n", ds->nand, ds->ptr,
						     ds->buf, ds->off);

	return RET_NOTHING;
}

static int erase_flash_verify_nand(struct urb *urb, struct dnload_state *ds,
				   unsigned long erasesize, size_t size)
{
	struct usb_device_instance *dev = urb->device;
	int rc;

	debug("erase_flash_verify_nand(urb=%p, ds=%p, erase=0x%lx size=0x%x)\n",
		urb, ds, erasesize, size);

	if (erasesize == ds->nand->erasesize) {
		/* we're only writing a single block and need to
		 * do bad block skipping / offset adjustments our own */
		while (ds->nand->block_isbad(ds->nand, ds->off)) {
			debug("SKIP_ONE_BLOCK(0x%08x)!!\n", ds->off);
			ds->off += ds->nand->erasesize;
		}
	}

	/* we have finished one eraseblock, flash it */
	ds->erase_opts.offset = ds->off;
	ds->erase_opts.length = erasesize;
	debug("Erasing 0x%x bytes @ offset 0x%x (jffs=%u)\n",
		(unsigned int)ds->erase_opts.length,
		(unsigned int)ds->erase_opts.offset,
		ds->erase_opts.jffs2);
	rc = nand_erase_opts(ds->nand, &ds->erase_opts);
	if (rc) {
		debug("Error erasing\n");
		dev->dfu_state = DFU_STATE_dfuERROR;
		dev->dfu_status = DFU_STATUS_errERASE;
		return RET_STALL;
	}

	debug("Writing 0x%x bytes @ offset 0x%x\n", size, ds->off);
	/* FIXME handle oob */
	rc = nand_write_skip_bad(ds->nand, ds->off, &size, ds->buf, 0);
	if (rc) {
		debug("Error writing\n");
		dev->dfu_state = DFU_STATE_dfuERROR;
		dev->dfu_status = DFU_STATUS_errWRITE;
		return RET_STALL;
	}

	ds->off += size;
	ds->ptr = ds->buf;

	/* FIXME: implement verify! */
	return RET_NOTHING;
}

static int erase_tail_clean_nand(struct urb *urb, struct dnload_state *ds)
{
	struct usb_device_instance *dev = urb->device;
	int rc;

	ds->erase_opts.offset = ds->off;
	ds->erase_opts.length = ds->part->size - (ds->off - ds->part->offset);
	debug("Erasing tail of 0x%x bytes @ offset 0x%x (jffs=%u)\n",
		(unsigned int)ds->erase_opts.length,
		(unsigned int)ds->erase_opts.offset,
		ds->erase_opts.jffs2);
	rc = nand_erase_opts(ds->nand, &ds->erase_opts);
	if (rc) {
		printf("Error erasing tail\n");
		dev->dfu_state = DFU_STATE_dfuERROR;
		dev->dfu_status = DFU_STATUS_errERASE;
		return RET_STALL;
	}

	ds->off += ds->erase_opts.length; /* for consistency */

	return RET_NOTHING;
}

/* Read the next erase block from NAND into buffer */
static int read_next_nand(struct urb *urb, struct dnload_state *ds, size_t len)
{
	struct usb_device_instance *dev = urb->device;
	int rc;

	debug("Reading 0x%x@0x%x to 0x%p\n", len, ds->off, ds->buf);
	rc = nand_read_skip_bad(ds->nand, ds->off, &len, ds->buf);
	if (rc) {
		debug("Error reading\n");
		dev->dfu_state = DFU_STATE_dfuERROR;
		dev->dfu_status = DFU_STATUS_errWRITE;
		return RET_STALL;
	}
	ds->off += len;
	ds->ptr = ds->buf;

	return RET_NOTHING;
}

static int get_dfu_loadaddr(uint8_t **loadaddr)
{
	const char *s;
	s = getenv("loadaddr");
	if (s != NULL) {
		*loadaddr = (uint8_t *)simple_strtoul(s, NULL, 16);
		return 1;
	} else
		return 0;
}

static int get_dfu_filesize(unsigned long *filesize)
{
	const char *s;
	s = getenv("filesize");
	if (s != NULL) {
		*filesize = simple_strtoul(s, NULL, 16);
		return 1;
	} else
		return 0;
}

static int handle_nand_dnload(struct urb *urb, u_int16_t val, u_int16_t len,
			 int first)
{
	struct usb_device_instance *dev = urb->device;
	struct dnload_state *ds = &_dnstate;
	unsigned int actual_len = len;
	unsigned int remain_len;
	unsigned long size;
	uint8_t *loadaddr;
	int rc;

	debug("download(len=%u, first=%u) ", len, first);

	if (!get_dfu_loadaddr(&loadaddr)) {
		printf("Error: DFU Download requires loadaddr to be set.\n");
		dev->dfu_state = DFU_STATE_dfuERROR;
		dev->dfu_status = DFU_STATUS_errADDRESS;
		return RET_STALL;
	}

	if (len > CONFIG_USBD_DFU_XFER_SIZE) {
		/* Too big. Not that we'd really care, but it's a
		 * DFU protocol violation */
		debug("length exceeds flash page size ");
		dev->dfu_state = DFU_STATE_dfuERROR;
		dev->dfu_status = DFU_STATUS_errADDRESS;
		return RET_STALL;
	}

	if (first && dev->alternate != 0) {
		/* Make sure that we have a valid mtd partition table */
		char *mtdp = getenv("mtdparts");
		if (!mtdp)
			run_command("dynpart", 0);
	}

	if (len == 0) {
		debug("zero-size write -> MANIFEST_SYNC ");
		dev->dfu_state = DFU_STATE_dfuMANIFEST_SYNC;

		/* cleanup */
		switch (dev->alternate) {
			char buf[12];
		case 0:
			sprintf(buf, "%x", ds->ptr - ds->buf);
			setenv("filesize", buf);
			ds->ptr = ds->buf;
			break;
		default:
			rc = 0;
			if (ds->ptr > ds->buf)
				rc = erase_flash_verify_nand(urb, ds,
							ds->nand->erasesize,
							ds->nand->erasesize);
			/* rootfs partition */
			if (!rc && ((!strcmp(ds->part->name, "rootfs"))
				|| (!strcmp(ds->part->name, "fs"))))
				rc = erase_tail_clean_nand(urb, ds);

			ds->nand = NULL;
			break;
		}

		return RET_ZLP;
	}

	if (urb->actual_length != len) {
		debug("urb->actual_length(%u) != len(%u) ?!? ",
			urb->actual_length, len);
		dev->dfu_state = DFU_STATE_dfuERROR;
		dev->dfu_status = DFU_STATUS_errADDRESS;
		return RET_STALL;
	}

	if (first && ds->buf && ds->buf != ds->_buf && ds->buf != loadaddr) {
		free(ds->buf);
		ds->buf = ds->_buf;
	}

	switch (dev->alternate) {
	case 0:
		if (first) {
			printf("Starting DFU DOWNLOAD to RAM (0x%p)\n",
				loadaddr);
			ds->buf = loadaddr;
			ds->ptr = ds->buf;
		}

		memcpy(ds->ptr, urb->buffer, len);
		ds->ptr += len;
		break;
	default:
		if (first) {
			rc = initialize_ds_nand(dev, ds);
			if (rc)
				return rc;
			printf("Starting DFU DOWNLOAD to partition '%s'\n",
				ds->part->name);
		}

		size = ds->nand->erasesize;
		remain_len = ds->buf + size - ds->ptr;
		if (remain_len < len)
			actual_len = remain_len;

		memcpy(ds->ptr, urb->buffer, actual_len);
		ds->ptr += actual_len;

		/* check partition end */
		if (ds->off + (ds->ptr - ds->buf) > ds->part->offset +
		    ds->part->size) {
			printf("End of write exceeds partition end\n");
			dev->dfu_state = DFU_STATE_dfuERROR;
			dev->dfu_status = DFU_STATUS_errADDRESS;
			return RET_STALL;
		}

		if (ds->ptr >= ds->buf + size) {
			rc = erase_flash_verify_nand(urb, ds,
						     ds->nand->erasesize,
						     ds->nand->erasesize);
			if (rc)
				return rc;
			/* copy remainder of data into buffer */
			memcpy(ds->ptr, urb->buffer + actual_len, len -
			       actual_len);
			ds->ptr += (len - actual_len);
		}
		break;
	}

	return RET_ZLP;
}

static int handle_nand_upload(struct urb *urb, u_int16_t val, u_int16_t len,
			 int first)
{
	struct usb_device_instance *dev = urb->device;
	struct dnload_state *ds = &_dnstate;
	unsigned int remain;
	uint8_t *loadaddr;
	unsigned long filesize;
	int rc;

	debug("upload(val=0x%02x, len=%u, first=%u) ", val, len, first);

	if (!get_dfu_loadaddr(&loadaddr) || !get_dfu_filesize(&filesize)) {
		printf("Error: DFU Upload requires loadaddr and filesize to be "
			"set.\n");
		dev->dfu_state = DFU_STATE_dfuERROR;
		dev->dfu_status = DFU_STATUS_errADDRESS;
		return -EINVAL;
	}

	if (len > CONFIG_USBD_DFU_XFER_SIZE) {
		/* Too big */
		dev->dfu_state = DFU_STATE_dfuERROR;
		dev->dfu_status = DFU_STATUS_errADDRESS;
		debug("Error: Transfer size > CONFIG_USBD_DFU_XFER_SIZE ");
		return -EINVAL;
	}

	switch (dev->alternate) {
	case 0:
		if (first) {
			printf("Starting DFU Upload of RAM (0x%p)\n", loadaddr);
			ds->buf = loadaddr;
			ds->ptr = ds->buf;
		}

		if (ds->ptr + len > loadaddr + filesize)
			len = (loadaddr + filesize) - ds->ptr;

		memcpy(urb->buffer, ds->ptr, len);
		urb->actual_length = len;
		ds->ptr += len;
		break;
	default:
		if (first) {
			rc = initialize_ds_nand(dev, ds);
			if (rc)
				return -EINVAL;
			printf("Starting DFU Upload of partition '%s'\n",
				ds->part->name);
		}

		if (len > ds->nand->erasesize) {
			printf("We don't support transfers bigger than %u\n",
				ds->nand->erasesize);
			len = ds->nand->erasesize;
		}

		/* limit length to whatever number of bytes is left in
		 * this partition */
		remain = (ds->part->offset + ds->part->size) - ds->off;
		if (len > remain)
			len = remain;

		rc = read_next_nand(urb, ds, len);
		if (rc)
			return -EINVAL;

		debug("uploading %u bytes ", len);
		urb->buffer = ds->buf;
		urb->actual_length = len;
		break;
	}

	debug("returning len=%u\n", len);
	return len;
}
