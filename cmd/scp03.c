// SPDX-License-Identifier: GPL-2.0+
/*
 * (C) Copyright 2021, Foundries.IO
 *
 */

#include <common.h>
#include <command.h>
#include <env.h>
#include <scp03.h>

int do_scp03_enable(cmd_tbl_t *cmdtp, int flag, int argc, char *const argv[])
{
	if (argc != 1)
		return CMD_RET_USAGE;

	if (tee_enable_scp03())
		return CMD_RET_FAILURE;

	return CMD_RET_SUCCESS;
}

int do_scp03_provision(cmd_tbl_t *cmdtp, int flag, int argc, char *const argv[])
{
	if (argc != 1)
		return CMD_RET_USAGE;

	if (tee_provision_scp03())
		return CMD_RET_FAILURE;

	return CMD_RET_SUCCESS;
}

static cmd_tbl_t cmd_scp03[] = {
	U_BOOT_CMD_MKENT(enable, 1, 0, do_scp03_enable, "", ""),
	U_BOOT_CMD_MKENT(provision, 1, 0, do_scp03_provision, "", ""),
};

static int do_scp03(cmd_tbl_t *cmdtp, int flag, int argc, char * const argv[])
{
	cmd_tbl_t *cp;

	cp = find_cmd_tbl(argv[1], cmd_scp03, ARRAY_SIZE(cmd_scp03));

	argc--;
	argv++;

	if (!cp || argc > cp->maxargs)
		return CMD_RET_USAGE;

	if (flag == CMD_FLAG_REPEAT)
		return CMD_RET_FAILURE;

	return cp->cmd(cmdtp, flag, argc, argv);
}

U_BOOT_CMD(
	scp03, 2, 0, do_scp03,
	"Provides a command to enable SCP03 and provision the SCP03 keys",
	"enable    - enable SCP03 if not already enabled\n"
	"provision - provision SCP03 (and enable SCP03 if not already enabled)\n"
	);
