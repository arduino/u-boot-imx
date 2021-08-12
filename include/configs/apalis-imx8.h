/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * Copyright 2019 Toradex
 */

#ifndef __APALIS_IMX8_H
#define __APALIS_IMX8_H

#include <asm/arch/imx-regs.h>
#include <linux/sizes.h>

#ifdef CONFIG_SPL_BUILD
#define CONFIG_SPL_MAX_SIZE				(192 * 1024)
#define CONFIG_SYS_MONITOR_LEN				(1024 * 1024)
#ifndef CONFIG_SYS_MMCSD_RAW_MODE_U_BOOT_USE_SECTOR
#define CONFIG_SYS_MMCSD_RAW_MODE_U_BOOT_USE_SECTOR
#define CONFIG_SYS_MMCSD_RAW_MODE_U_BOOT_SECTOR		0x1040 /* (flash.bin_offset + 2Mb)/sector_size */
#endif

/*
 * 0x08081000 - 0x08180FFF is for m4_0 xip image,
 * 0x08181000 - 0x008280FFF is for m4_1 xip image
  * So 3rd container image may start from 0x8281000
 */
#define CONFIG_SYS_UBOOT_BASE 0x08281000
#define CONFIG_SYS_MMCSD_FS_BOOT_PARTITION             0

#define CONFIG_SPL_LDSCRIPT		"arch/arm/cpu/armv8/u-boot-spl.lds"
#define CONFIG_SPL_STACK		0x013fff0
#define CONFIG_SPL_BSS_START_ADDR	0x00130000
#define CONFIG_SPL_BSS_MAX_SIZE		0x1000	/* 4 KB */
#define CONFIG_SYS_SPL_MALLOC_START	0x82200000
#define CONFIG_SYS_SPL_MALLOC_SIZE	0x80000	/* 512 KB */
#define CONFIG_SERIAL_LPUART_BASE	0x5a070000
#define CONFIG_MALLOC_F_ADDR		0x00138000

#define CONFIG_SPL_RAW_IMAGE_ARM_TRUSTED_FIRMWARE

#define CONFIG_SPL_ABORT_ON_RAW_IMAGE

#endif

#define CONFIG_REMAKE_ELF

#define CONFIG_DISPLAY_BOARDINFO_LATE

#define CONFIG_SYS_FSL_ESDHC_ADDR	0
#define USDHC1_BASE_ADDR		0x5b010000
#define USDHC2_BASE_ADDR		0x5b020000

#define CONFIG_ENV_OVERWRITE

#define CONFIG_ENV_VARS_UBOOT_RUNTIME_CONFIG

#define CONFIG_SKIP_RESOURCE_CHECKING

/* Networking */
#define FEC_QUIRK_ENET_MAC

/* We have a slow phy... */
#define PHY_ANEG_TIMEOUT		15000

#define CONFIG_TFTP_TSIZE

#define CONFIG_IPADDR			192.168.10.2
#define CONFIG_NETMASK			255.255.255.0
#define CONFIG_SERVERIP			192.168.10.1

#define FEC_ENET_ENABLE_TXC_DELAY

/**
 * SYS_SDRAM_BASE	0x80000000	0.125MiB
 * SYS_TEXT_BASE	0x80020000	2.375MiB
 * kernel_addr_r	0x80280000	45.5MiB
 * fdt_addr_r		0x83000000	1MiB
 * scriptaddr		0x83100000	15MiB
 * __RESERVED__		0x84000000	48MiB
 * loadaddr		0x87000000	48MiB
 * ramdisk_addr_r	0x8a000000	288MiB (to hdp_addr)
 * SYS_MEMTEST_START	0x90000000
 * hdp_addr		0x9c000000
 * SYS_MEMTEST_END	0xC0000000
 */
#define MEM_LAYOUT_ENV_SETTINGS \
	"fdt_addr_r=0x83000000\0" \
	"hdp_addr=0x9c000000\0" \
	"kernel_addr_r=0x80280000\0" \
	"ramdisk_addr_r=0x8a000000\0" \
	"scriptaddr=0x83100000\0"

/* Boot M4 */
#define M4_BOOT_ENV \
	"m4_0_image=m4_0.bin\0" \
	"m4_1_image=m4_1.bin\0" \
	"loadm4image_0=${load_cmd} ${loadaddr} ${m4_0_image}\0" \
	"loadm4image_1=${load_cmd} ${loadaddr} ${m4_1_image}\0" \
	"m4boot_0=run loadm4image_0; dcache flush; bootaux ${loadaddr} 0\0" \
	"m4boot_1=run loadm4image_1; dcache flush; bootaux ${loadaddr} 1\0" \

#define BOOT_TARGET_DEVICES(func) \
	func(MMC, mmc, 1) \
	func(MMC, mmc, 2) \
	func(MMC, mmc, 0) \
	func(USB, usb, 0)
#include <config_distro_bootcmd.h>

#ifdef CONFIG_AHAB_BOOT
#define AHAB_ENV "sec_boot=yes\0"
#else
#define AHAB_ENV "sec_boot=no\0"
#endif

#define FDT_FILE			"imx8qm-apalis-v1.1-eval.dtb"
#define FDT_FILE_V1_0			"imx8qm-apalis-eval.dtb"

/* Initial environment variables */
#define CONFIG_EXTRA_ENV_SETTINGS \
	AHAB_ENV \
	BOOTENV \
	M4_BOOT_ENV \
	MEM_LAYOUT_ENV_SETTINGS \
	"boot_script_dhcp=boot.scr\0" \
	"bootcmd_mfg=select_dt_from_module_version && fastboot 0\0" \
	"script=boot.scr\0" \
	"boot_file=Image\0" \
	"console=ttyLP1 earlycon\0" \
	"fdt_high=0xffffffffffffffff\0" \
	"boot_fdt=try\0" \
	"fdtfile=" FDT_FILE "\0" \
	"finduuid=part uuid mmc ${mmcdev}:2 uuid\0" \
	"hdp_file=hdmitxfw.bin\0" \
	"loadhdp=${load_cmd} ${hdp_addr} ${hdp_file}\0" \
	"mmcautodetect=yes\0" \
	"mmcargs=setenv bootargs console=${console},${baudrate} " \
		"root=PARTUUID=${uuid} rootwait " \
	"mmcdev=" __stringify(CONFIG_SYS_MMC_ENV_DEV) "\0" \
	"mmcpart=" __stringify(CONFIG_SYS_MMC_IMG_LOAD_PART) "\0" \
	"panel=NULL\0" \
	"update_uboot=askenv confirm Did you load u-boot-dtb.imx (y/N)?; " \
		"if test \"$confirm\" = \"y\"; then " \
		"setexpr blkcnt ${filesize} + 0x1ff && setexpr blkcnt " \
		"${blkcnt} / 0x200; mmc dev 0 1; mmc write ${loadaddr} 0x0 " \
		"${blkcnt}; fi\0" \
	"video=imxdpufb5:off video=imxdpufb6:off video=imxdpufb7:off\0" \
	"setup=run loadhdp; hdp load ${hdp_addr}; run mmcargs\0" \
	"defargs=pci=nomsi"

/* Link Definitions */
#define CONFIG_LOADADDR			0x87000000

#define CONFIG_SYS_LOAD_ADDR		CONFIG_LOADADDR

#define CONFIG_SYS_INIT_SP_ADDR		0x80200000

#define CONFIG_SYS_MEMTEST_START	0x90000000
#define CONFIG_SYS_MEMTEST_END		0xc0000000

/* Environment in eMMC, before config block at the end of 1st "boot sector" */
#define CONFIG_SYS_MMC_ENV_DEV		0	/* USDHC1 eMMC */
#define CONFIG_SYS_MMC_ENV_PART		1

#define CONFIG_SYS_MMC_IMG_LOAD_PART	1

/* On Apalis iMX8 USDHC1 is eMMC, USDHC2 is 8-bit and USDHC3 is 4-bit MMC/SD */
#define CONFIG_SYS_FSL_USDHC_NUM	3

#define CONFIG_SYS_BOOTM_LEN		SZ_64M /* Increase max gunzip size */

/* Size of malloc() pool */
#define CONFIG_SYS_MALLOC_LEN		((CONFIG_ENV_SIZE + (32 * 1024)) * 1024)

#define CONFIG_SYS_SDRAM_BASE		0x80000000
#define PHYS_SDRAM_1			0x80000000
#define PHYS_SDRAM_2			0x880000000
#define PHYS_SDRAM_1_SIZE		SZ_2G		/* 2 GB */
#define PHYS_SDRAM_2_SIZE		SZ_2G		/* 2 GB */

/* Serial */
#define CONFIG_BAUDRATE			115200

/* Monitor Command Prompt */
#define CONFIG_SYS_PROMPT_HUSH_PS2	"> "
#define CONFIG_SYS_CBSIZE		SZ_2K
#define CONFIG_SYS_MAXARGS		64
#define CONFIG_SYS_BARGSIZE		CONFIG_SYS_CBSIZE
#define CONFIG_SYS_PBSIZE		(CONFIG_SYS_CBSIZE + \
					sizeof(CONFIG_SYS_PROMPT) + 16)

/* Generic Timer Definitions */
#define COUNTER_FREQUENCY		8000000	/* 8MHz */

/* USB Config */
#ifndef CONFIG_SPL_BUILD
#define CONFIG_USBD_HS

#define CONFIG_CMD_USB_MASS_STORAGE
#define CONFIG_USB_GADGET_MASS_STORAGE
#define CONFIG_USB_FUNCTION_MASS_STORAGE

#endif

#define CONFIG_USB_MAX_CONTROLLER_COUNT 2

/* USB OTG controller configs */
#ifdef CONFIG_USB_EHCI_HCD
#define CONFIG_USB_HOST_ETHER
#define CONFIG_USB_ETHER_ASIX
#define CONFIG_MXC_USB_PORTSC		(PORT_PTS_UTMI | PORT_PTS_PTW)
#endif

#ifdef CONFIG_DM_VIDEO
#define CONFIG_VIDEO_LOGO
#define CONFIG_SPLASH_SCREEN
#define CONFIG_SPLASH_SCREEN_ALIGN
#define CONFIG_CMD_BMP
#define CONFIG_BMP_16BPP
#define CONFIG_BMP_24BPP
#define CONFIG_BMP_32BPP
#define CONFIG_VIDEO_BMP_RLE8
#define CONFIG_VIDEO_BMP_LOGO
#endif

#endif /* __APALIS_IMX8_H */
