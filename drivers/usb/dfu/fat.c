#include <common.h>
#include <fat.h>
#include <dfu_backend.h>

static const char *fat_filename;
static int fat_part_num;
static block_dev_desc_t *fat_dev;

inline int set_fat_part_num(int pnum)
{
	fat_part_num = pnum;
	
	return pnum;
}

inline const char *set_fat_filename(const char *fn)
{
	fat_filename = fn;

	return fn;
}

inline block_dev_desc_t *set_fat_dev(block_dev_desc_t *d)
{
	fat_dev = d;

	return d;
}

int read_fat(void *buf, unsigned int len, unsigned long from)
{
	int ret;

	ret = fat_register_device(fat_dev, fat_part_num);
	if (ret < 0) {
		printf("error : fat_register_device\n");
		return 0;
	}
	printf("read up to %d B ", MMC_FAT_BLOCK_SZ);
	return file_fat_read(fat_filename, buf, len);
}

int write_fat(void *buf, unsigned int len, unsigned long from)
{
#ifdef CONFIG_FAT_WRITE
	int ret;

	ret = fat_register_device(fat_dev, fat_part_num);
	if (ret < 0) {
		printf("error : fat_register_divce\n");
		return 0;
	}

	printf("write up to %d B ", MMC_FAT_BLOCK_SZ);
	ret = file_fat_write(fat_filename, buf, len);

	/* format and write again */
	if (ret == 1) {
		printf("formatting\n");
		if (mkfs_vfat(fat_dev, fat_part_num)) {
			printf("error formatting device\n");
			return 0;
		}
		ret = file_fat_write(fat_filename, buf, len);
	}

	if (ret < 0) {
		printf("error : writing %s\n", fat_filename);
		return 0;
	}
#else
	printf("error : FAT write not supported\n");
	return 0;
#endif
	return len;
}

