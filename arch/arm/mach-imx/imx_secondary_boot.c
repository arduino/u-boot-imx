// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (C) 2021 Foundries.io Ltd
 */

#include <common.h>
#include <asm/io.h>
#include <asm/mach-imx/sys_proto.h>
#include <command.h>
#include <elf.h>
#include <imx_sip.h>
#include <linux/compiler.h>
#include <cpu_func.h>

static int do_secondary_boot(cmd_tbl_t *cmdtp, int flag,
			     int argc, char * const argv[])
{
	u32 persist_secondary = 0;

	if (argc < 2)
		return CMD_RET_USAGE;

	persist_secondary = simple_strtoul(argv[1], NULL, 10);

	if (!(persist_secondary == 0 || persist_secondary == 1))
		return CMD_RET_USAGE;

	call_imx_sip(IMX_SIP_SRC, IMX_SIP_SRC_SET_SECONDARY_BOOT,
		     persist_secondary, 0, 0);

	printf("Set PERSIST_SECONDARY_BOOT = %d\n", persist_secondary);

	return CMD_RET_SUCCESS;
}

U_BOOT_CMD(
	secondary_boot, CONFIG_SYS_MAXARGS, 1, do_secondary_boot,
	"Set PERSIST_SECONDARY_BOOT bit",
	"[0|1]\n"
	"   0 - boot primary image\n"
	"   1 - boot secondary image\n"
);
