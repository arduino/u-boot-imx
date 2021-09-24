// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (C) 2016 Freescale Semiconductor, Inc.
 * Copyright (C) 2020 Foundries.IO
 */

#include <common.h>
#include <init.h>
#include <asm/global_data.h>
#include <asm/io.h>
#include <asm/arch/sys_proto.h>
#include <asm/arch/mx7ulp-pins.h>
#include <asm/arch/iomux.h>
#include <asm/mach-imx/boot_mode.h>
#include <asm/gpio.h>
#include <fdt_support.h>
#include <linux/delay.h>
#include <usb.h>
#include <dm.h>

DECLARE_GLOBAL_DATA_PTR;

#define UART_PAD_CTRL		(PAD_CTL_PUS_UP)
#define QSPI_PAD_CTRL1		(PAD_CTL_PUS_UP | PAD_CTL_DSE)
#define OTG_ID_GPIO_PAD_CTRL	(PAD_CTL_IBE_ENABLE)
#define OTG_PWR_GPIO_PAD_CTRL   (PAD_CTL_OBE_ENABLE)

int dram_init(void)
{
	gd->ram_size = imx_ddr_size();

#ifdef CONFIG_OPTEE_TZDRAM_SIZE
	gd->ram_size -= CONFIG_OPTEE_TZDRAM_SIZE;
#endif

	return 0;
}

static iomux_cfg_t const lpuart4_pads[] = {
	MX7ULP_PAD_PTC3__LPUART4_RX | MUX_PAD_CTRL(UART_PAD_CTRL),
	MX7ULP_PAD_PTC2__LPUART4_TX | MUX_PAD_CTRL(UART_PAD_CTRL),
};

static void setup_iomux_uart(void)
{
	mx7ulp_iomux_setup_multiple_pads(lpuart4_pads,
					 ARRAY_SIZE(lpuart4_pads));
}

#ifdef CONFIG_FSL_QSPI
#ifndef CONFIG_DM_SPI
static iomux_cfg_t const quadspi_pads[] = {
	MX7ULP_PAD_PTB8__QSPIA_SS0_B | MUX_PAD_CTRL(QSPI_PAD_CTRL1),
	MX7ULP_PAD_PTB15__QSPIA_SCLK  | MUX_PAD_CTRL(QSPI_PAD_CTRL1),
	MX7ULP_PAD_PTB16__QSPIA_DATA3 | MUX_PAD_CTRL(QSPI_PAD_CTRL1),
	MX7ULP_PAD_PTB17__QSPIA_DATA2 | MUX_PAD_CTRL(QSPI_PAD_CTRL1),
	MX7ULP_PAD_PTB18__QSPIA_DATA1 | MUX_PAD_CTRL(QSPI_PAD_CTRL1),
	MX7ULP_PAD_PTB19__QSPIA_DATA0 | MUX_PAD_CTRL(QSPI_PAD_CTRL1),
};
#endif

int board_qspi_init(void)
{
	u32 val;
#ifndef CONFIG_DM_SPI
	mx7ulp_iomux_setup_multiple_pads(quadspi_pads, ARRAY_SIZE(quadspi_pads));
#endif

	/* enable clock */
	val = readl(PCC1_RBASE + 0x94);

	if (!(val & 0x20000000)) {
		writel(0x03000003, (PCC1_RBASE + 0x94));
		writel(0x43000003, (PCC1_RBASE + 0x94));
	}

	/* Enable QSPI as a wakeup source on B0 */
	if (soc_rev() >= CHIP_REV_2_0)
		setbits_le32(SIM0_RBASE + WKPU_WAKEUP_EN, WKPU_QSPI_CHANNEL);
	return 0;
}
#endif

#ifdef CONFIG_USB_EHCI_MX6
static iomux_cfg_t const usb_otg1_pads[] = {
	/* gpio for otgid */
	MX7ULP_PAD_PTC13__PTC13 | MUX_PAD_CTRL(OTG_ID_GPIO_PAD_CTRL),
	/* gpio for power en */
	MX7ULP_PAD_PTE15__PTE15 | MUX_PAD_CTRL(OTG_PWR_GPIO_PAD_CTRL),
};

#define OTG0_ID_GPIO	IMX_GPIO_NR(3, 13)
#define OTG0_PWR_EN	IMX_GPIO_NR(5, 15)

static void setup_usb(void)
{
	mx7ulp_iomux_setup_multiple_pads(usb_otg1_pads,
						 ARRAY_SIZE(usb_otg1_pads));

	gpio_request(OTG0_ID_GPIO, "otg_id");
	gpio_direction_input(OTG0_ID_GPIO);
}

/* Needs to override the ehci power if controlled by GPIO */
int board_ehci_power(int port, int on)
{
	switch (port) {
	case 0:
		if (on)
			gpio_direction_output(OTG0_PWR_EN, 1);
		else
			gpio_direction_output(OTG0_PWR_EN, 0);
		break;
	default:
		printf("MXC USB port %d not yet supported\n", port);
		return -EINVAL;
	}

	return 0;
}

int board_usb_phy_mode(int port)
{
	/* The device mode is valid for port 0 only */
	if (port == 0 && gpio_get_value(OTG0_ID_GPIO))
		return USB_INIT_DEVICE;

	return USB_INIT_HOST;
}
#endif

int board_early_init_f(void)
{
	setup_iomux_uart();

	return 0;
}

/* will reset the M4 and the A7 */
void board_m4_restart(void)
{
	unsigned reset = IMX_GPIO_NR(3, 10); /* PTC10 */

	printf("ea board: system reset\n");
	gpio_request(reset, "system_reset");
	gpio_direction_output(reset, 0);
	udelay(100);
	gpio_set_value(reset, 1);
}

int board_init(void)
{
	/* address of boot parameters */
	gd->bd->bi_boot_params = PHYS_SDRAM + 0x100;

#ifdef CONFIG_USB_EHCI_MX6
	setup_usb();
#endif

#ifdef CONFIG_FSL_QSPI
	board_qspi_init();
#endif

	return 0;
}

#if IS_ENABLED(CONFIG_OF_BOARD_SETUP)
int ft_board_setup(void *blob, bd_t *bd)
{
	const char *path;
	int rc, nodeoff;

	if (get_boot_device() == USB_BOOT) {
		path = fdt_get_alias(blob, "mmc0");
		if (!path) {
			puts("Not found mmc0\n");
			return 0;
		}

		nodeoff = fdt_path_offset(blob, path);
		if (nodeoff < 0)
			return 0;

		printf("Found usdhc0 node\n");
		if (fdt_get_property(blob, nodeoff, "vqmmc-supply",
		    NULL) != NULL) {
			rc = fdt_delprop(blob, nodeoff, "vqmmc-supply");
			if (!rc) {
				puts("Removed vqmmc-supply property\n");
add:
				rc = fdt_setprop(blob, nodeoff,
						 "no-1-8-v", NULL, 0);
				if (rc == -FDT_ERR_NOSPACE) {
					rc = fdt_increase_size(blob, 32);
					if (!rc)
						goto add;
				} else if (rc) {
					printf("Failed to add no-1-8-v property, %d\n", rc);
				} else {
					puts("Added no-1-8-v property\n");
				}
			} else {
				printf("Failed to remove vqmmc-supply property, %d\n", rc);
			}
		}
	}

	return 0;
}
#endif

#ifdef CONFIG_SPL_BUILD
#include <spl.h>

#ifdef CONFIG_SPL_LOAD_FIT
int board_fit_config_name_match(const char *name)
{
	if (!strcmp(name, "imx7ulp-com"))
		return 0;

	return -1;
}
#endif

void spl_board_init(void)
{
	preloader_console_init();
}

void board_init_f(ulong dummy)
{
	arch_cpu_init();

	board_early_init_f();
}
#endif
