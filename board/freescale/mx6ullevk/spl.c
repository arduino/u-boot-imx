// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright 2020 Foundries.io
 */

#include <common.h>
#include <spl.h>
#include <asm/io.h>
#include <asm/arch/clock.h>
#include <asm/arch/crm_regs.h>
#include <asm/arch/imx-regs.h>
#include <asm/arch/mx6ull_pins.h>
#include <asm/mach-imx/iomux-v3.h>

DECLARE_GLOBAL_DATA_PTR;

#include <asm/arch/mx6-ddr.h>

#ifdef CONFIG_SPL_LOAD_FIT
int board_fit_config_name_match(const char *name)
{
	if (!strcmp(name, "imx6ull-14x14-evk"))
		return 0;

	return -1;
}
#endif

#ifdef CONFIG_FSL_ESDHC_IMX

/* Values copied from imx6ul-14x14-evk.dtsi */
static iomux_v3_cfg_t const usdhc2_pads[] = {
	MX6_PAD_NAND_RE_B__USDHC2_CLK | MUX_PAD_CTRL(0x17059),
	MX6_PAD_NAND_WE_B__USDHC2_CMD | MUX_PAD_CTRL(0x17059),
	MX6_PAD_NAND_DATA00__USDHC2_DATA0 | MUX_PAD_CTRL(0x17059),
	MX6_PAD_NAND_DATA01__USDHC2_DATA1 | MUX_PAD_CTRL(0x17059),
	MX6_PAD_NAND_DATA02__USDHC2_DATA2 | MUX_PAD_CTRL(0x17059),
	MX6_PAD_NAND_DATA03__USDHC2_DATA3 | MUX_PAD_CTRL(0x17059),
};

static void setup_iomux_mmc(void)
{
	imx_iomux_v3_setup_multiple_pads(
		usdhc2_pads, ARRAY_SIZE(usdhc2_pads));
}
#endif

static void ccgr_init(void)
{
	struct mxc_ccm_reg *ccm = (struct mxc_ccm_reg *)CCM_BASE_ADDR;

	writel(0xFFFFFFFF, &ccm->CCGR0);
	writel(0xFFFFFFFF, &ccm->CCGR1);
	writel(0xFFFFFFFF, &ccm->CCGR2);
	writel(0xFFFFFFFF, &ccm->CCGR3);
	writel(0xFFFFFFFF, &ccm->CCGR4);
	writel(0xFFFFFFFF, &ccm->CCGR5);
	writel(0xFFFFFFFF, &ccm->CCGR6);
	writel(0xFFFFFFFF, &ccm->CCGR7);
}

void board_init_f(ulong dummy)
{
	/* DDR initialization done via DCD */

	ccgr_init();
	enable_usdhc_clk(1, 0);
	enable_usdhc_clk(1, 1);

	/* setup AIPS and disable watchdog */
	arch_cpu_init();

	/* iomux and setup of i2c */
	board_early_init_f();

	/* setup GP timer */
	timer_init();

	/* UART clocks enabled and gd valid - init serial console */
	preloader_console_init();

	/* Clear the BSS. */
	memset(__bss_start, 0, __bss_end - __bss_start);

	get_clocks();

#ifdef CONFIG_FSL_ESDHC_IMX
	/* setup MMC pins */
	setup_iomux_mmc();
#endif

	/* load/boot image from boot device */
	board_init_r(NULL, 0);
}

void reset_cpu(ulong addr)
{
}
