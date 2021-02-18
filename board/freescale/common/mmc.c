/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * Copyright (C) 2016 Freescale Semiconductor, Inc.
 * Copyright 2018 NXP
 */
#include <common.h>
#include <command.h>
#include <asm/arch/sys_proto.h>
#include <linux/errno.h>
#include <asm/io.h>
#include <stdbool.h>
#include <mmc.h>
#include <env.h>

static int check_mmc_autodetect(void)
{
	char *autodetect_str = env_get("mmcautodetect");

	if ((autodetect_str != NULL) &&
		(strcmp(autodetect_str, "yes") == 0)) {
		return 1;
	}

	return 0;
}

/* This should be defined for each board */
__weak int mmc_map_to_kernel_blk(int dev_no)
{
	return dev_no;
}

void board_late_mmc_env_init(void)
{
	char cmd[32];
	char mmcblk[32];
	u32 dev_no = mmc_get_env_dev();

	if (!check_mmc_autodetect())
		return;

	env_set_ulong("mmcdev", dev_no);

	/* Set mmcblk env */
	sprintf(mmcblk, "/dev/mmcblk%dp2 rootwait rw",
		mmc_map_to_kernel_blk(dev_no));
	env_set("mmcroot", mmcblk);

	sprintf(cmd, "mmc dev %d", dev_no);
	run_command(cmd, 0);
}

#if defined(CONFIG_SECONDARY_BOOT_RUNTIME_DETECTION) && \
    defined(CONFIG_SYS_MMCSD_RAW_MODE_U_BOOT_USE_SECTOR)
unsigned long spl_mmc_get_uboot_raw_sector(struct mmc *mmc,
					   unsigned long raw_sect)
{
	int boot_secondary = boot_mode_getprisec();
	unsigned long offset = CONFIG_SYS_MMCSD_RAW_MODE_U_BOOT_SECTOR;

	if (boot_secondary) {
		offset += CONFIG_SECONDARY_BOOT_SECTOR_OFFSET;
		printf("SPL: Booting secondary boot path: using 0x%lx offset "
		       "for next boot image\n", offset);
	} else {
		printf("SPL: Booting primary boot path: using 0x%lx offset "
		       "for next boot image\n", offset);
	}

	return offset;
}
#endif
