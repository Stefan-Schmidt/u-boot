#include <common.h>
#include <mbr.h>
#include <mmc.h>
#include <dfu_backend.h>
#include <flash_entity.h>

#define SIGNATURE		((unsigned short) 0xAA55)
#define PARAM_LEN		12

static int read_ebr(struct mmc *mmc, struct mbr_partition *mp,
		int ebr_next, struct mbr_part_data *pd, int parts,
		int *extended_lba, int mmc_mbr_dev)
{
	struct mbr *ebr;
	struct mbr_partition *p;
	char buf[512];
	int ret, i;
	int lba = 0;

	if (ebr_next)
		lba = *extended_lba;

	lba += mp->lba;
	ret = mmc->block_dev.block_read(mmc_mbr_dev, lba, 1, buf);
	if (ret != 1)
		return 0;

	ebr = (struct mbr *) buf;

	if (ebr->signature != SIGNATURE) {
		printf("Signature error 0x%x\n", ebr->signature);
		return 0;
	}

	for (i = 0; i < 2; i++) {
		p = (struct mbr_partition *) &ebr->parts[i];

		if (i == 0) {
			lba += p->lba;
			if (p->partition_type == 0x83) {
				if (pd) {
					pd[parts].offset = lba;
					pd[parts].length = p->nsectors;
					pd[parts].primary = 0;
				}
				parts++;
			}
		}
	}

	if (p->lba && p->partition_type == 0x5)
		parts = read_ebr(mmc, p, 1, pd, parts, extended_lba, mmc_mbr_dev);

	return parts;
}

int read_mbr(struct mmc *mmc, struct mbr_part_data *pd, int *extended_lba,
	     int mmc_mbr_dev)
{
	struct mbr_partition *mp;
	struct mbr *mbr;
	char buf[512];
	int ret, i;
	int parts = 0;

	ret = mmc->block_dev.block_read(mmc_mbr_dev, 0, 1, buf);
	if (ret != 1)
		return 0;

	mbr = (struct mbr *) buf;

	if (mbr->signature != SIGNATURE) {
		printf("Signature error 0x%x\n", mbr->signature);
		return 0;
	}

	for (i = 0; i < 4; i++) {
		mp = (struct mbr_partition *) &mbr->parts[i];

		if (!mp->partition_type)
			continue;

		if (mp->partition_type == 0x83) {
			if (pd) {
				pd[parts].offset = mp->lba;
				pd[parts].length = mp->nsectors;
				pd[parts].primary = 1;
			}
			parts++;
		}

		if (mp->lba && mp->partition_type == 0x5) {
			*extended_lba = mp->lba;
			parts = read_ebr(mmc, mp, 0, pd, parts, extended_lba, mmc_mbr_dev);
		}
	}

	return parts;
}

static int rw_mmc(void *buf, char *op, unsigned int len, unsigned long from)
{
	char ram_addr[PARAM_LEN];
	char offset[PARAM_LEN];
	char length[PARAM_LEN];
	char *argv[] = {"mmc", op, ram_addr, offset, length};

	sprintf(ram_addr, "0x%lx", (unsigned long)buf);
	sprintf(offset, "0x%lx", from / MMC_SECTOR_SZ); /* guaranteed integer */
	sprintf(length, "0x%x", (len + MMC_SECTOR_SZ - 1) / MMC_SECTOR_SZ);

	return do_mmcops(NULL, 0, 6, argv);
}

inline int read_mmc(void *buf, unsigned int len, unsigned long from)
{
	return rw_mmc(buf, "read", len, from);
}

inline int write_mmc(void *buf, unsigned int len, unsigned long from)
{
	return rw_mmc(buf, "write", len, from);
}

