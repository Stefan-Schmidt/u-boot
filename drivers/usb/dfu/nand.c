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
