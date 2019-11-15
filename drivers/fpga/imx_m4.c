// SPDX-License-Identifier: GPL-2.0+
/*
 * (C) Copyright 2019, Foundries.IO
 *
 */
#include <asm/arch/sys_proto.h>
#include <asm/mach-imx/boot_mode.h>
#include <fpga.h>
#include <image.h>
#include <mtd.h>
#include <spi_flash.h>

extern enum bt_mode get_boot_mode(void);

void __weak board_m4_restart(void)
{
	printf("stub: M4 restart\n");
}

enum m4_fw_state {m4_fw_boot, m4_fw_abort, m4_fw_upgrade};

struct hash {
	uint8_t value[FIT_MAX_HASH_LEN];
	int len;
};

/*
 * TEE needs to store the payload length so we can recalculate the sha
 * before booting from flash
 */
struct sec_hash {
	struct hash hash;
	unsigned long payload_len;
};

#ifdef M4_HASH_DEBUG
void print_hash(char *msg, struct hash *hash)
{
	int i;
	printf("%s\n", msg);
	printf("\t hash       => ");
	for (i = 0; i < hash->len; i++)
		printf("%02x ", hash->value[i]);
	printf("\n");
	printf("\t hash_len    : 0x%x\n", hash->len);

}

void print_secure_hash(char *msg, struct sec_hash *sec)
{
	print_hash(msg, &sec->hash);
	printf("\t payload_len : 0x%lx\n", sec->payload_len);
}
#else
void print_hash(char *msg, struct hash *hash){}
void print_secure_hash(char *msg, struct sec_hash *sec){}
#endif

#if !defined(CONFIG_FIOVB)
/* When FIOV is not available, we initialize the secure hash using the currently
   stored image in flash to determine its value */
static struct sec_hash t_hash;

static int init_secure_hash(struct spi_flash *flash, size_t size)
{
	struct mtd_info *mtd = &flash->mtd;
	size_t len;

	len = round_up(size, mtd->writesize);
	if (spi_flash_read(flash, 0, len, (void *) M4_BASE)) {
		printf("M4: Failed to read from flash, can't boot\n");
		return -EIO;
	}

	if (calculate_hash((void *) M4_BASE, size, "sha256",
			   t_hash.hash.value, &t_hash.hash.len)) {
		printf("M4: unsupported hash algorithm to decode bistream\n");
		return -EINVAL;
	}

	t_hash.payload_len = size;
	print_secure_hash("M4: current secure hash init:", &t_hash);

	return 0;
}

static int get_secure_hash(struct sec_hash *sec)
{
	memcpy(sec->hash.value, t_hash.hash.value, t_hash.hash.len);
	sec->hash.len = t_hash.hash.len;
	sec->payload_len = t_hash.payload_len;
	print_secure_hash("M4: Retrieved Secure hash", sec);

	return 0;
}

static int update_secure_hash(struct hash *hash, unsigned long payload_len)
{
	memcpy(t_hash.hash.value, hash->value, hash->len);
	t_hash.hash.len = hash->len;
	t_hash.payload_len = payload_len;

	return 0;
}

#else
/* Use Foundries.IO verified boot TA: this application stores the hash of the
   currently installed firmware */
#include <fiovb.h>
static struct fiovb_ops *sec;

static inline int secure_read_char(char *name, char *value,
				   size_t *len, size_t buf_len)
{
	int ret;

	ret = sec->read_persistent_value(sec, name, buf_len,
					 (uint8_t *) value, len);
	if (ret != FIOVB_IO_RESULT_OK)
		return -EIO;

	return 0;
}

static inline int secure_read_long(char *name, unsigned long *value)
{
	char len_str[32] = { '\0' };
	size_t len;
	int ret;

	ret = sec->read_persistent_value(sec, name, sizeof(len_str),
					 (uint8_t *) len_str, &len);
	if (ret != FIOVB_IO_RESULT_OK)
		return -EIO;

	*value = simple_strtoul(len_str, NULL, 10);

	return 0;
}

static inline int secure_write_char(char *name, char *value, size_t len)
{
	int ret;

	ret = sec->write_persistent_value(sec, name, len, (uint8_t *) value);
	if (ret != FIOVB_IO_RESULT_OK)
		return -EIO;

	return 0;
}

static inline int secure_write_long(char *name, unsigned long value)
{
	char len_str[32] = { '\0' };
	int ret;

	snprintf(len_str, sizeof(len_str), "%ld", value);
	ret = sec->write_persistent_value(sec, name, strlen(len_str) + 1,
					  (uint8_t *) len_str);
	if (ret != FIOVB_IO_RESULT_OK)
		return -EIO;

	return 0;
}

static int get_secure_hash(struct sec_hash *sec_hash)
{
	int ret;

	ret = secure_read_char("m4hash", (char *) sec_hash->hash.value,
			       (size_t *) &sec_hash->hash.len,
			       sizeof(sec_hash->hash.value));
	if (ret)
		return ret;

	ret = secure_read_long("m4size", &sec_hash->payload_len);
	if (ret)
		return ret;

	print_secure_hash("M4: Retrieved Secure hash", sec_hash);

	return 0;
}

static int update_secure_hash(struct hash *hash, unsigned long len)
{
	int ret;

	ret = secure_write_char("m4hash", (char *) hash->value, hash->len);
	if (ret)
		return ret;

	ret = secure_write_long("m4size", len);
	if (ret)
		return ret;

	return ret;
}
#endif

static int m4_do_upgrade(struct spi_flash *flash, const void *data,
			 const size_t size, struct hash *hash)
{
	struct mtd_info *mtd = &flash->mtd;
	size_t len;

	len = round_down(M4_SIZE, mtd->erasesize);
	if (spi_flash_erase(flash, 0, len)) {
		printf("M4: Failed to erase flash\n");
		return -EIO;
	}

	len = round_up(size, mtd->writesize);
	if (spi_flash_write(flash, 0, len, data)) {
		printf("M4: Failed to write image to flash\n");
		return -EIO;
	}

	if (update_secure_hash(hash, size)) {
		printf("M4: Failed to update the secure hash\n");
		return -EIO;
	}

	printf("M4: Firmware upgraded from FIT...\n");

	/* in dual boot mode, the M4 is probably already running the old
	   software so let's restart it */
	if (get_boot_mode() == DUAL_BOOT)
		board_m4_restart();

	return 0;
}

static int m4_get_state(const void *data, size_t size,
		        struct hash *hash, enum m4_fw_state *action)
{
	struct sec_hash sec_hash = {
		.hash.len = 1,
		.payload_len = 0,
	};
	int retry = 1;

	/* assume validation error */
	*action = m4_fw_abort;

	if (size < 0x2000) {
		printf("M4: Image size too small (%d < 0x2000)!\n", size);
		return -EINVAL;
	}

	/* 1) check firmware for valid tags: assume the M4 image has IVT head
	 * and padding which should be the same as the one programmed into
	 * QSPI flash */
	if (!M4_FW_VALID(M4_WATERMARK(data))) {
		printf("M4: Invalid image: tag=0x%x\n", M4_WATERMARK(data));
		return -EINVAL;
	}

	/* 2) determine the FIT image hash  */
	if (calculate_hash(data, size, "sha256",
			   hash->value, &hash->len)) {
		printf("M4: unsupported hash algorithm to decode bistream\n");
		return -EINVAL;
	}

retry_hash:
	/* Get the installed firmware hash (optee->fiovb->rpmb) */
	if (get_secure_hash(&sec_hash)) {
		if (!retry--) {
			printf("M4: TA not accessible, abort\n");
			return -EIO;
		}

		/* first boot or keys corrupted, clean and retry  */
		update_secure_hash(&sec_hash.hash, sec_hash.payload_len);
		goto retry_hash;
	}

	if (sec_hash.hash.len != hash->len ||
	    memcmp(sec_hash.hash.value, hash->value, hash->len) ||
	    sec_hash.payload_len != size) {
		*action = m4_fw_upgrade;
	} else {
		*action = m4_fw_boot;
	}

	print_hash("M4: Firmware hash", hash);
	print_hash("M4: Secure hash", &sec_hash.hash);

	return 0;
}

static int m4_boot(struct spi_flash *flash, size_t size, struct hash *p)
{
	struct mtd_info *mtd = 	&flash->mtd;
	struct sec_hash sec_hash;
	struct hash hash;
	int ret = 0;
	size_t len;

	/* 0) handle boot modes */
	if (get_boot_mode() == DUAL_BOOT) {
		printf("M4: dual boot: M4 already running, continue\n");
		return 0;
	}

	if (readl(M4_BOOT_REG)) {
		printf("M4: single boot: M4 already running, continue\n");
		return 0;
	}

	/* 1) read the secure hash */
	if (get_secure_hash(&sec_hash)) {
		printf("M4: cant boot, TEE not accessible\n");
		ret = -EIO;
		goto error;
	}

	/* 2) validate it against the known value */
	if (p->len != sec_hash.hash.len  ||
	    size != sec_hash.payload_len ||
	    memcmp(p->value, sec_hash.hash.value, p->len)) {
		printf("M4: secure hash corrupted\n");

		print_hash("M4: Firmware hash", p);
		debug("M4: Firmware load_len (%d)\n", size);
		print_secure_hash("M4: Secure hash", &sec_hash);

		ret = -EIO;
		goto error;
	}

	/* 3) validate it against the installed image  */
	len = round_up(sec_hash.payload_len, mtd->writesize);
	if (spi_flash_read(flash, 0, len, (void *) M4_BASE)) {
		printf("M4: Failed to read from flash, can't boot\n");
		ret = -EIO;
		goto error;
	}

	debug("\tM4 Entry: 0x%x\n", M4_ENTRY(M4_BASE));
	debug("\tM4 Sram : 0x%x\n", M4_BASE);
	debug("\tM4 Size : 0x%lx\n", sec_hash.payload_len);

	/* calculate the installed image hash */
	if (calculate_hash((const void *) M4_BASE, sec_hash.payload_len,
			   "sha256", hash.value, &hash.len)) {
		printf("M4: cant boot, unsupported hash algorithm (sha256)\n");
		ret = -EINVAL;
		goto error;
	}

	/* do not boot if the installed image is corrupted */
	if (hash.len != sec_hash.hash.len ||
	    memcmp(hash.value, sec_hash.hash.value, sec_hash.hash.len)) {
		printf("M4: cant boot, invalid hash %s in firmware or tee\n",
			hash.len != sec_hash.hash.len ? "length" : "value");
		ret = -ENOEXEC;
		goto error;
	}

	printf("M4: Hash OK, booting\n");
	writel(M4_ENTRY(M4_BASE), M4_BOOT_REG);

	return 0;
error:
	printf("M4: boot failed (%d), erasing flash\n", ret);
	/* clear the data in SRAM and flash when we cant boot it */
	memset((void *) M4_BASE, 0x00, M4_SIZE);
	spi_flash_erase(flash, 0, M4_SIZE);

	/* invalidate the RPMB information so we trigger an upgrade on the
	   next boot avoiding an infinite loop */
	memset(hash.value, 0x00, sizeof(hash.value));
	hash.len = 0;
	update_secure_hash(&hash, 0);

	return ret;
}

/* As part of the boot sequence, SPL checks all images in the FIT; we chose
 * to register the M4 firmware as an FPGA image to benefit from this loading
 * process: ie, before booting the kernel, fpga_loadbistream will get executed
 * allowing the M4 firmware to be loaded and then run.
 * If upgrade fails due to hardware or security problems, do not boot a previous
 * firmware already resident in QSPI.
 */
int fpga_loadbitstream(int d, char *bitstream, size_t size, bitstream_type t)
{
	const void *data = (const void *) bitstream;
	struct spi_flash *flash = NULL;
	enum m4_fw_state state;
	struct hash hash;
	int ret;

	flash = spi_flash_probe(CONFIG_ENV_SPI_BUS, CONFIG_ENV_SPI_CS,
				 CONFIG_ENV_SPI_MAX_HZ, CONFIG_ENV_SPI_MODE);
	if (!flash) {
		printf("M4: Failed to probe QSPI\n");
		return -EIO;
	}

#if defined(CONFIG_FIOVB)
	sec = fiovb_ops_alloc(0);
	if (!sec) {
		printf("M4: cant allocate secure operations, rollback\n");
		return -EIO;
	}
#else
	ret = init_secure_hash(flash, size);
	if (ret) {
		printf("M4: cant validate the installed firmware\n");
		return ret;
	}
#endif
	ret = m4_get_state(data, size, &hash, &state);

	if (state == m4_fw_abort) {
		printf("M4: cant boot or upgrade the M4, rollback\n");
		goto done;
	}

	if (state == m4_fw_upgrade) {
		printf("M4: begin software upgrade\n");
		ret = m4_do_upgrade(flash, data, size, &hash);
		if (ret) {
			printf("M4: upgrade failed, rollback\n");
			return ret;
		}
		goto boot;
	}

	printf("M4: already installed\n");
boot:
	ret = m4_boot(flash, size, &hash);
done:
#if defined(CONFIG_FIOVB)
	fiovb_ops_free(sec);
#endif
	return ret;
}

