#include <common.h>
#include <dfu_backend.h>
#include <flash_entity.h>

/*
 * Adapt transport layer buffer size to storage chunk size
 *
 * return < n to indicate no more data to read
 */
int read_block(void *ctx, unsigned int n, void *buf)
{
	struct flash_entity_ctx *ct = ctx;
	unsigned int nread = 0;

	if (n == 0)
		return n;

	while (nread < n) {
		unsigned int copy;

		if (ct->num_done >= ct->length)
			break;
		if (ct->buffered == 0) {
			ct->read(ct->buf, ct->buf_len,
				 ct->offset + ct->num_done);
			ct->buffered = ct->buf_len;
		}
		copy = min(n - nread, ct->buffered);

		memcpy(buf + nread, ct->buf + ct->buf_len - ct->buffered, copy);
		nread += copy;
		ct->buffered -= copy;
		ct->num_done += copy;
	}

	return nread;
}

/*
 * Adapt transport layer buffer size to storage chunk size
 */
int write_block(void *ctx, unsigned int n, void *buf)
{
	struct flash_entity_ctx *ct = ctx;
	unsigned int nwritten = 0;

	if (n == 0)
		return n;

	while (nwritten < n) {
		unsigned int copy;

		if (ct->num_done >= ct->length)
			break;
		if (ct->buffered >= ct->buf_len) {
			ct->write(ct->buf, ct->buf_len,
				  ct->offset + ct->num_done);
			ct->buffered = 0;
			ct->num_done += ct->buf_len;
			if (ct->num_done >= ct->length)
				break;
		}
		copy = min(n - nwritten, ct->buf_len - ct->buffered);

		memcpy(ct->buf + ct->buffered, buf + nwritten, copy);
		nwritten += copy;
		ct->buffered += copy;
	}

	return nwritten;
}

/*
 * Entity-specific prepare and finish
 */
static void reset_ctx(struct flash_entity_ctx *ctx)
{
	ctx->buffered = 0;
	ctx->num_done = 0;
}

int generic_prepare(void *ctx, u8 mode)
{
	struct flash_entity_ctx *ct = ctx;

	reset_ctx(ct);
	memset(ct->buf, 0, ct->buf_len);
	if (mode == FLASH_WRITE) {
		if (ct->erase) {
			printf("Erase entity: %s ", ct->this_entity->name);
			ct->erase(ct->length, ct->offset);
		}
		printf("Write entity: %s ", ct->this_entity->name);
	} else if (mode == FLASH_READ) {
		printf("Read entity: %s ", ct->this_entity->name);
	}
	return 0;
}

int generic_finish(void *ctx, u8 mode)
{
	struct flash_entity_ctx *ct = ctx;

	if (mode == FLASH_WRITE && ct->buffered > 0)
		ct->write(ct->buf, ct->buffered, ct->offset + ct->num_done);

	return 0;
}

