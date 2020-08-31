/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * SPL definitions for the i.MX7ULP SPL
 *
 * (C) Copyright 2019 Foundries.io
 */

#ifndef __IMX7ULP_SPL_CONFIG_H
#define __IMX7ULP_SPL_CONFIG_H

#ifdef CONFIG_SPL
/*
 * see figure 35-5 in i.MX 7ULP Reference manual:
 *  - IMX7ULP A7 OCRAM free area RAM is from 0x2F010000 to 0x2F03FF00.
 *  - Set the stack at the end of the free area section, at 0x2003FEB8.
 *  - The BOOT ROM loads what they consider the firmware image
 *    which consists of a 4K header in front of us that contains the IVT, DCD
 *    and some padding thus 'our' max size is really 0x2F03FF00 - 0x2F011000.
 *    187KB is more then enough for the SPL.
 */
#define CONFIG_SPL_MAX_SIZE		0x2EC00
#define CONFIG_SPL_STACK		0x2F03FEB8
/*
 * Pad SPL to 191KB (4KB header + 187KB max size). This allows to write the
 * SPL/U-Boot combination generated with u-boot-with-spl.imx directly to a
 * boot media (given that boot media specific offset is configured properly).
 */
#define CONFIG_SPL_PAD_TO		0x2FC00

/* MMC support */
#if defined(CONFIG_SPL_MMC_SUPPORT)
#define CONFIG_SYS_MMCSD_FS_BOOT_PARTITION	1
#define CONFIG_SYS_MONITOR_LEN			409600	/* 400 KB */
#endif

/* Define the payload for FAT/EXT support */
#if defined(CONFIG_SPL_FS_FAT) || defined(CONFIG_SPL_FS_EXT4)
# ifdef CONFIG_OF_CONTROL
#  define CONFIG_SPL_FS_LOAD_PAYLOAD_NAME	"u-boot-dtb.img"
# else
#  define CONFIG_SPL_FS_LOAD_PAYLOAD_NAME	"u-boot.img"
# endif
#endif

#define CONFIG_SPL_BSS_START_ADDR      0x68200000
#define CONFIG_SPL_BSS_MAX_SIZE        0x100000		/* 1 MB */
#define CONFIG_SYS_SPL_MALLOC_START    0x68300000
#define CONFIG_SYS_SPL_MALLOC_SIZE     0x100000		/* 1 MB */

#endif /* CONFIG_SPL */

#endif /* __IMX7ULP_SPL_CONFIG_H */
