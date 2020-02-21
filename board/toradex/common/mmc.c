// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (c) 2016-2019 Toradex, Inc.
 */

#include <common.h>
#include <mmc.h>

/* provide a default implementation */
__weak int mmc_map_to_kernel_blk(int dev_no)
{
	return dev_no;
}
