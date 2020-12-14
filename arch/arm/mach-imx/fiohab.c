// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (C) 2019 Foundries.IO
 */

#include <common.h>
#include <config.h>
#include <fuse.h>
#include <mapmem.h>
#include <image.h>
#include <asm/io.h>
#include <asm/system.h>
#include <asm/arch/clock.h>
#include <asm/arch/sys_proto.h>
#include <asm/mach-imx/hab.h>

#if defined(CONFIG_FIOVB) && !defined(CONFIG_SPL_BUILD)
#include <fiovb.h>
#include <mmc.h>

static struct mmc *init_mmc_device(int dev, bool force_init)
{
	struct mmc *mmc;

	mmc = find_mmc_device(dev);
	if (!mmc) {
		printf("no mmc device at slot %x\n", dev);
		return NULL;
	}

	if (mmc_init(mmc)) {
		printf("cant initialize mmc at slot %x\n", dev);
		return NULL;
	}

	return mmc;
}

static int fiovb_provisioned(void)
{
	char len_str[32] = { '\0' };
	struct fiovb_ops *sec;
	int ret;
	unsigned int fiohab_dev = env_get_ulong("fiohab_dev", 10, 0xFFUL);

	if (fiohab_dev == 0xFFUL) {
		printf("fiohav_dev var is not defined!\n");
		return -EINVAL;
	}

	if (!init_mmc_device(fiohab_dev, false)) {
		printf("Cant init MMC - RPMB not available\n");
		return -1;
	}

	sec = fiovb_ops_alloc(fiohab_dev);
	if (!sec) {
		printf("Not enough memory to allocate ops\n");
		return -ENOMEM;
	}

	snprintf(len_str, sizeof(len_str), "%ld", (unsigned long) 0);
	ret = sec->write_persistent_value(sec, "m4size", strlen(len_str) + 1,
					 (uint8_t *) len_str);
	fiovb_ops_free(sec);

	/* if the RPMB is accessible, then we can't close the device */
	if (ret == FIOVB_IO_RESULT_OK) {
		printf("Error, rpmb provisioned with test keys\n");
		return -1;
	}

	return 0;
}
#else
static int fiovb_provisioned(void)
{
	printf("RPMB provisioned check stubbed out !!\n");
	return 0;
}
#endif

#ifdef CONFIG_MX7ULP
#define SRK_FUSE_LIST								\
{ 5, 0 }, { 5, 1 }, { 5, 2}, { 5, 3 }, { 5, 4 }, { 5, 5}, { 5, 6 }, { 5 ,7 },	\
{ 6, 0 }, { 6, 1 }, { 6, 2}, { 6, 3 }, { 6, 4 }, { 6, 5}, { 6, 6 }, { 6 ,7 },
#define SECURE_FUSE_BANK	(29)
#define SECURE_FUSE_WORD	(6)
#define SECURE_FUSE_VALUE	(0x80000000)
#elif CONFIG_ARCH_MX6
#define SRK_FUSE_LIST								\
{ 3, 0 }, { 3, 1 }, { 3, 2}, { 3, 3 }, { 3, 4 }, { 3, 5}, { 3, 6 }, { 3 ,7 },
#define SECURE_FUSE_BANK	(0)
#define SECURE_FUSE_WORD	(6)
#define SECURE_FUSE_VALUE	(0x00000002)
#else
#error "SoC not supported"
#endif

static hab_rvt_report_status_t *hab_check;

static int hab_status(void)
{
	hab_check = (hab_rvt_report_status_t *) HAB_RVT_REPORT_STATUS;
	enum hab_config config = 0;
	enum hab_state state = 0;

	if (hab_check(&config, &state) != HAB_SUCCESS) {
		printf("HAB events active error\n");
		return 1;
	}

	return 0;
}

/* The fuses must have been programmed and their values set in the environment.
 * The fuse read operation returns a shadow value so a board reset is required
 * after the SRK fuses have been written.
 *
 * On CAAM enabled boards (imx7, imx6 and others), the board should not be closed
 * if RPMB keys have been provisioned as it would render it unavailable
 * afterwards
 */
static int do_fiohab_close(cmd_tbl_t *cmdtp, int flag, int argc,
			   char *const argv[])
{
	struct srk_fuse {
		u32 bank;
		u32 word;
	} const srk_fuses[] = { SRK_FUSE_LIST };
	char fuse_name[20] = { '\0' };
	uint32_t fuse, fuse_env;
	int i, ret;

	if (argc != 1) {
		cmd_usage(cmdtp);
		return 1;
	}

	/* if secure boot is already enabled, there is nothing to do */
	if (imx_hab_is_enabled()) {
		printf("secure boot already enabled\n");
		return 0;
	}

	/* if RPMB can be accessed, we cant close the board */
	ret = fiovb_provisioned();
	if (ret)
		return 1;

	/* if there are pending HAB errors, we cant close the board */
	if (hab_status())
		return 1;

	for (i = 0; i < ARRAY_SIZE(srk_fuses); i++) {
		ret = fuse_read(srk_fuses[i].bank, srk_fuses[i].word, &fuse);
		if (ret) {
			printf("Secure boot fuse read error\n");
			return 1;
		}

		/**
		 * if the fuses are not in in the environemnt or hold the wrong
		 * values, then we cant close the board
		 */
		sprintf(fuse_name, "srk_%d", i);
		fuse_env = (uint32_t) env_get_hex(fuse_name, 0);
		if (!fuse_env) {
			printf("%s not in environment\n", fuse_name);
			return 1;
		}

		if (fuse_env != fuse) {
			printf("%s - programmed: 0x%x != expected: 0x%x \n",
				fuse_name, fuse, fuse_env);
			return 1;
		}
	}

	ret = fuse_prog(SECURE_FUSE_BANK, SECURE_FUSE_WORD, SECURE_FUSE_VALUE);
	if (ret) {
		printf("Error writing the Secure Fuse\n");
		return 1;
	}

	return 0;
}

U_BOOT_CMD(fiohab_close, CONFIG_SYS_MAXARGS, 1, do_fiohab_close,
	   "Close the board for HAB","");

