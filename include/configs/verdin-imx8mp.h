/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * Copyright 2020 Toradex
 */

#ifndef __VERDIN_IMX8MP_H
#define __VERDIN_IMX8MP_H

#include <linux/sizes.h>
#include <asm/arch/imx-regs.h>

#include "imx_env.h"

#define CONFIG_SPL_MAX_SIZE				(152 * 1024)
#define CONFIG_SYS_MONITOR_LEN				(512 * 1024)
#define CONFIG_SYS_MMCSD_RAW_MODE_U_BOOT_USE_SECTOR
#define CONFIG_SYS_MMCSD_RAW_MODE_U_BOOT_SECTOR	0x300
#define CONFIG_SYS_MMCSD_FS_BOOT_PARTITION		1
#define CONFIG_SYS_UBOOT_BASE				(QSPI0_AMBA_BASE + CONFIG_SYS_MMCSD_RAW_MODE_U_BOOT_SECTOR * 512)

#ifdef CONFIG_SPL_BUILD
#define CONFIG_SPL_STACK				0x187FF0
#define CONFIG_SPL_BSS_START_ADDR			0x0095e000
#define CONFIG_SPL_BSS_MAX_SIZE				0x2000 /* 8 KB */
#define CONFIG_SYS_SPL_MALLOC_START			0x42200000
#define CONFIG_SYS_SPL_MALLOC_SIZE			SZ_512K	/* 512 KB */

#define CONFIG_MALLOC_F_ADDR				0x184000 /* malloc f used before GD_FLG_FULL_MALLOC_INIT set */

#define CONFIG_SPL_ABORT_ON_RAW_IMAGE

#define CONFIG_POWER
#define CONFIG_POWER_I2C
#define CONFIG_POWER_PCA9450

#define CONFIG_SYS_I2C
#define CONFIG_SYS_I2C_SPEED		100000

#endif /* CONFIG_SPL_BUILD */

#define CONFIG_FASTBOOT_USB_DEV		0

#define CONFIG_REMAKE_ELF
/* ENET Config */
/* ENET1 */
#if defined(CONFIG_CMD_NET)
#define CONFIG_ETHPRIME			"eth1" /* Set eqos to primary since we use its MDIO */

#define CONFIG_FEC_XCV_TYPE		RGMII
#define CONFIG_FEC_MXC_PHYADDR		3
#define FEC_QUIRK_ENET_MAC

#define DWC_NET_PHYADDR			7
#ifdef CONFIG_DWC_ETH_QOS
#define CONFIG_SYS_NONCACHED_MEMORY	(1 * SZ_1M) /* 1M */
#endif

#define PHY_ANEG_TIMEOUT		20000

#endif /* CONFIG_CMD_NET */

#define MEM_LAYOUT_ENV_SETTINGS \
	"fdt_addr_r=0x43000000\0" \
	"kernel_addr_r=0x40000000\0" \
	"ramdisk_addr_r=0x46400000\0" \
	"scriptaddr=0x46000000\0"

/* Link Definitions */
#define CONFIG_LOADADDR			0x43500000
#define CONFIG_SYS_LOAD_ADDR		CONFIG_LOADADDR

/* Enable Distro Boot */
#ifndef CONFIG_SPL_BUILD
#define BOOT_TARGET_DEVICES(func) \
	func(MMC, mmc, 1) \
	func(MMC, mmc, 2) \
	func(USB, usb, 0) \
	func(DHCP, dhcp, na)
#include <config_distro_bootcmd.h>
#undef CONFIG_ISO_PARTITION
#else
#define BOOTENV
#endif

/* Initial environment variables */
#define CONFIG_EXTRA_ENV_SETTINGS \
	BOOTENV \
	MEM_LAYOUT_ENV_SETTINGS \
	"bootcmd_mfg=fastboot 0\0" \
	"console=ttymxc2\0" \
	"fdt_board=dev\0" \
	"initrd_addr=0x43800000\0" \
	"initrd_high=0xffffffffffffffff\0" \
	"boot_script_dhcp=boot.scr\0" \
	"boot_file=Image\0" \
	"setup=setenv setupargs console=${console},${baudrate} console=tty1 consoleblank=0 earlycon\0"

#define CONFIG_SYS_INIT_RAM_ADDR	0x40000000
#define CONFIG_SYS_INIT_RAM_SIZE	0x80000
#define CONFIG_SYS_INIT_SP_OFFSET \
	(CONFIG_SYS_INIT_RAM_SIZE - GENERATED_GBL_DATA_SIZE)
#define CONFIG_SYS_INIT_SP_ADDR \
	(CONFIG_SYS_INIT_RAM_ADDR + CONFIG_SYS_INIT_SP_OFFSET)

#define CONFIG_ENV_OVERWRITE
#if defined(CONFIG_ENV_IS_IN_MMC)
/* Environment in eMMC, before config block at the end of 1st "boot sector" */
#undef CONFIG_ENV_SIZE
#undef CONFIG_ENV_OFFSET

#define CONFIG_ENV_SIZE		0x2000
#define CONFIG_ENV_OFFSET		(-CONFIG_ENV_SIZE + \
					 CONFIG_TDX_CFG_BLOCK_OFFSET)
#define CONFIG_SYS_MMC_ENV_DEV		2
#define CONFIG_SYS_MMC_ENV_PART		1
#endif

#define CONFIG_SYS_BOOTM_LEN		SZ_64M /* Increase max gunzip size */

/* Size of malloc() pool */
#define CONFIG_SYS_MALLOC_LEN		SZ_32M

/* i.MX 8M Plus supports max. 8GB memory in two albeit concecutive banks */
#define CONFIG_SYS_SDRAM_BASE		0x40000000
#define PHYS_SDRAM			0x40000000
#define PHYS_SDRAM_SIZE			0xC0000000	/* 3 GB */
#define PHYS_SDRAM_2			0x100000000
#define PHYS_SDRAM_2_SIZE		0x140000000	/* 5 GB */

#define CONFIG_SYS_MEMTEST_START	PHYS_SDRAM
#define CONFIG_SYS_MEMTEST_END		(CONFIG_SYS_MEMTEST_START + \
					(PHYS_SDRAM_SIZE >> 1))

#define CONFIG_MXC_UART_BASE		UART3_BASE_ADDR

/* Monitor Command Prompt */
#define CONFIG_SYS_CBSIZE		2048
#define CONFIG_SYS_MAXARGS		64
#define CONFIG_SYS_BARGSIZE		CONFIG_SYS_CBSIZE
#define CONFIG_SYS_PBSIZE		(CONFIG_SYS_CBSIZE + \
					sizeof(CONFIG_SYS_PROMPT) + 16)

#define CONFIG_FSL_USDHC

#define CONFIG_SYS_FSL_USDHC_NUM	2
#define CONFIG_SYS_FSL_ESDHC_ADDR	0

#define CONFIG_SYS_MMC_IMG_LOAD_PART	1

/* USB configs */
#ifndef CONFIG_SPL_BUILD
#define CONFIG_USB_GADGET_MASS_STORAGE
#endif

#define CONFIG_USB_MAX_CONTROLLER_COUNT		2
#define CONFIG_USBD_HS
#define CONFIG_USB_GADGET_VBUS_DRAW		2

#endif /* __VERDIN_IMX8MP_H */
