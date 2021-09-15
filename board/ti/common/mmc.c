// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (c) 2021 Foundries.io Ltd
 */

#include <common.h>
#include <mmc.h>

/* default implementation */
__weak int mmc_map_to_kernel_blk(int dev_no)
{
	return dev_no;
}
