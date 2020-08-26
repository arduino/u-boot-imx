/* Copyright 2018 NXP
 *
 * SPDX-License-Identifier:     GPL-2.0+
 */

#ifndef __IMX_COMMON_CONFIG_H
#define __IMX_COMMON_CONFIG_H

#ifdef CONFIG_ARM64
    #define MFG_BOOT_CMD "booti "
#else
    #define MFG_BOOT_CMD "bootz "
#endif

#ifdef CONFIG_USB_PORT_AUTO
    #define FASTBOOT_CMD "echo \"Run fastboot ...\"; fastboot auto; "
#else
    #define FASTBOOT_CMD "echo \"Run fastboot ...\"; fastboot 0; "
#endif

/* define the nandfit partiton environment for uuu */
#if defined(CONFIG_IMX8MM) || defined(CONFIG_IMX8MQ) || \
	defined(CONFIG_IMX8QM) || defined(CONFIG_IMX8QXP) || \
	defined(CONFIG_IMX8DXL) || defined(CONFIG_IMX8MN) || \
	defined(CONFIG_IMX8MP)
#define MFG_NAND_FIT_PARTITION "nandfit_part=yes\0"
#else
#define MFG_NAND_FIT_PARTITION ""
#endif

#define CONFIG_MFG_ENV_SETTINGS_DEFAULT \
	"mfgtool_args=setenv bootargs console=${console},${baudrate} " \
		"rdinit=/linuxrc " \
		"clk_ignore_unused "\
		"\0" \
	MFG_NAND_FIT_PARTITION \
	FASTBOOT_CMD \

#endif
