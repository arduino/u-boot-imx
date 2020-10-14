// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright 2020 Toradex
 */

#include <common.h>
#include <asm/io.h>
#include <asm/arch/clock.h>
#include <asm/arch/imx8mm_pins.h>
#include <asm/arch/sys_proto.h>
#include <asm-generic/gpio.h>
#include <asm/mach-imx/dma.h>
#include <asm/mach-imx/iomux-v3.h>
#include <asm/mach-imx/gpio.h>
#include <asm/mach-imx/mxc_i2c.h>
#include <asm/mach-imx/video.h>
#include <dm.h>
#include <errno.h>
#include <fsl_esdhc.h>
#include <i2c.h>
#include <malloc.h>
#include <micrel.h>
#include <miiphy.h>
#include <mmc.h>
#include <netdev.h>
#include <power/pmic.h>
#include <power/bd71837.h>
#include <spl.h>
#include <usb.h>

#include "../common/tdx-cfg-block.h"

DECLARE_GLOBAL_DATA_PTR;

#define I2C_PMIC	0

#define UART_PAD_CTRL	(PAD_CTL_PUE | PAD_CTL_PE | PAD_CTL_DSE4)
#define WDOG_PAD_CTRL	(PAD_CTL_DSE6 | PAD_CTL_ODE | PAD_CTL_PUE | PAD_CTL_PE)

typedef enum {
	PCB_VERSION_1_0,
	PCB_VERSION_1_1
} pcb_rev_t;

/* Verdin UART_3, Console/Debug UART */
static iomux_v3_cfg_t const uart_pads[] = {
	IMX8MM_PAD_SAI2_RXFS_UART1_TX | MUX_PAD_CTRL(UART_PAD_CTRL),
	IMX8MM_PAD_SAI2_RXC_UART1_RX | MUX_PAD_CTRL(UART_PAD_CTRL),
};

static iomux_v3_cfg_t const wdog_pads[] = {
	IMX8MM_PAD_GPIO1_IO02_WDOG1_WDOG_B  | MUX_PAD_CTRL(WDOG_PAD_CTRL),
};

int board_early_init_f(void)
{
	struct wdog_regs *wdog = (struct wdog_regs *)WDOG1_BASE_ADDR;

	imx_iomux_v3_setup_multiple_pads(wdog_pads, ARRAY_SIZE(wdog_pads));

	set_wdog_reset(wdog);

	imx_iomux_v3_setup_multiple_pads(uart_pads, ARRAY_SIZE(uart_pads));

	return 0;
}

#ifdef CONFIG_BOARD_POSTCLK_INIT
int board_postclk_init(void)
{
	/* TODO */
	return 0;
}
#endif

#ifdef CONFIG_OF_BOARD_SETUP
int ft_board_setup(void *blob, bd_t *bd)
{
	return ft_common_board_setup(blob, bd);
}
#endif

#ifdef CONFIG_FEC_MXC
static int setup_fec(void)
{
	struct iomuxc_gpr_base_regs *const iomuxc_gpr_regs
		= (struct iomuxc_gpr_base_regs *) IOMUXC_GPR_BASE_ADDR;

	/* Use 125M anatop REF_CLK1 for ENET1, not from external */
	clrsetbits_le32(&iomuxc_gpr_regs->gpr[1],
			IOMUXC_GPR_GPR1_GPR_ENET1_TX_CLK_SEL_MASK, 0);
	return set_clk_enet(ENET_125MHZ);
}

int board_phy_config(struct phy_device *phydev)
{
	int tmp;

	switch(ksz9xx1_phy_get_id(phydev) & MII_KSZ9x31_SILICON_REV_MASK) {
	case PHY_ID_KSZ9031:
		/*
		* The PHY adds 1.2ns for the RXC and 0ns for TXC clock by default. The MAC
		* and the layout don't add a skew between clock and data.
		* Add 0.3ns for the RXC path and 0.96 + 0.42 ns (1.38 ns) for the TXC path
		* to get the required clock skews.
		*/
		/* control data pad skew - devaddr = 0x02, register = 0x04 */
		ksz9031_phy_extended_write(phydev, 0x02,
					MII_KSZ9031_EXT_RGMII_CTRL_SIG_SKEW,
					MII_KSZ9031_MOD_DATA_NO_POST_INC, 0x0070);
		/* rx data pad skew - devaddr = 0x02, register = 0x05 */
		ksz9031_phy_extended_write(phydev, 0x02,
					MII_KSZ9031_EXT_RGMII_RX_DATA_SKEW,
					MII_KSZ9031_MOD_DATA_NO_POST_INC, 0x7777);
		/* tx data pad skew - devaddr = 0x02, register = 0x06 */
		ksz9031_phy_extended_write(phydev, 0x02,
					MII_KSZ9031_EXT_RGMII_TX_DATA_SKEW,
					MII_KSZ9031_MOD_DATA_NO_POST_INC, 0x0000);
		/* gtx and rx clock pad skew - devaddr = 0x02, register = 0x08 */
		ksz9031_phy_extended_write(phydev, 0x02,
					MII_KSZ9031_EXT_RGMII_CLOCK_SKEW,
					MII_KSZ9031_MOD_DATA_NO_POST_INC, 0x03f4);
		break;
	case PHY_ID_KSZ9131:
	default:
		/* read rxc dll control - devaddr = 0x2, register = 0x4c */
		tmp = ksz9031_phy_extended_read(phydev, 0x02,
					MII_KSZ9131_EXT_RGMII_2NS_SKEW_RXDLL,
					MII_KSZ9031_MOD_DATA_NO_POST_INC);
		/* disable rxdll bypass (enable 2ns skew delay on RXC) */
		tmp &= ~MII_KSZ9131_RXTXDLL_BYPASS;
		/* rxc data pad skew 2ns - devaddr = 0x02, register = 0x4c */
		tmp = ksz9031_phy_extended_write(phydev, 0x02,
					MII_KSZ9131_EXT_RGMII_2NS_SKEW_RXDLL,
					MII_KSZ9031_MOD_DATA_NO_POST_INC, tmp);
		/* read txc dll control - devaddr = 0x02, register = 0x4d */
		tmp = ksz9031_phy_extended_read(phydev, 0x02,
					MII_KSZ9131_EXT_RGMII_2NS_SKEW_TXDLL,
					MII_KSZ9031_MOD_DATA_NO_POST_INC);
		/* disable txdll bypass (enable 2ns skew delay on TXC) */
		tmp &= ~MII_KSZ9131_RXTXDLL_BYPASS;
		/* rxc data pad skew 2ns - devaddr = 0x02, register = 0x4d */
		tmp = ksz9031_phy_extended_write(phydev, 0x02,
					MII_KSZ9131_EXT_RGMII_2NS_SKEW_TXDLL,
					MII_KSZ9031_MOD_DATA_NO_POST_INC, tmp);
		break;
	}

	if (phydev->drv->config)
		phydev->drv->config(phydev);
	return 0;
}
#endif

#ifdef CONFIG_USB_TCPC
struct tcpc_port port1;
struct tcpc_port port2;

static int setup_pd_switch(uint8_t i2c_bus, uint8_t addr)
{
	struct udevice *bus;
	struct udevice *i2c_dev = NULL;
	int ret;
	uint8_t valb;

	ret = uclass_get_device_by_seq(UCLASS_I2C, i2c_bus, &bus);
	if (ret) {
		printf("%s: Can't find bus\n", __func__);
		return -EINVAL;
	}

	ret = dm_i2c_probe(bus, addr, 0, &i2c_dev);
	if (ret) {
		printf("%s: Can't find device id=0x%x\n",
			__func__, addr);
		return -ENODEV;
	}

	ret = dm_i2c_read(i2c_dev, 0xB, &valb, 1);
	if (ret) {
		printf("%s dm_i2c_read failed, err %d\n", __func__, ret);
		return -EIO;
	}
	valb |= 0x4; /* Set DB_EXIT to exit dead battery mode */
	ret = dm_i2c_write(i2c_dev, 0xB, (const uint8_t *)&valb, 1);
	if (ret) {
		printf("%s dm_i2c_write failed, err %d\n", __func__, ret);
		return -EIO;
	}

	/* Set OVP threshold to 23V */
	valb = 0x6;
	ret = dm_i2c_write(i2c_dev, 0x8, (const uint8_t *)&valb, 1);
	if (ret) {
		printf("%s dm_i2c_write failed, err %d\n", __func__, ret);
		return -EIO;
	}

	return 0;
}

int pd_switch_snk_enable(struct tcpc_port *port)
{
	if (port == &port1) {
		debug("Setup pd switch on port 1\n");
		return setup_pd_switch(1, 0x72);
	} else if (port == &port2) {
		debug("Setup pd switch on port 2\n");
		return setup_pd_switch(1, 0x73);
	} else
		return -EINVAL;
}

struct tcpc_port_config port1_config = {
	.i2c_bus = 1, /*i2c2*/
	.addr = 0x50,
	.port_type = TYPEC_PORT_UFP,
	.max_snk_mv = 5000,
	.max_snk_ma = 3000,
	.max_snk_mw = 40000,
	.op_snk_mv = 9000,
	.switch_setup_func = &pd_switch_snk_enable,
};

struct tcpc_port_config port2_config = {
	.i2c_bus = 1, /*i2c2*/
	.addr = 0x52,
	.port_type = TYPEC_PORT_UFP,
	.max_snk_mv = 5000,
	.max_snk_ma = 3000,
	.max_snk_mw = 40000,
	.op_snk_mv = 9000,
	.switch_setup_func = &pd_switch_snk_enable,
};

static int setup_typec(void)
{
	int ret;

	debug("tcpc_init port 2\n");
	ret = tcpc_init(&port2, port2_config, NULL);
	if (ret) {
		printf("%s: tcpc port2 init failed, err=%d\n",
		       __func__, ret);
	} else if (tcpc_pd_sink_check_charging(&port2)) {
		/* Disable PD for USB1, since USB2 has priority */
		port1_config.disable_pd = true;
		printf("Power supply on USB2\n");
	}

	debug("tcpc_init port 1\n");
	ret = tcpc_init(&port1, port1_config, NULL);
	if (ret) {
		printf("%s: tcpc port1 init failed, err=%d\n",
		       __func__, ret);
	} else {
		if (!port1_config.disable_pd)
			printf("Power supply on USB1\n");
		return ret;
	}

	return ret;
}
#endif /* CONFIG_USB_TCPC */

#ifdef CONFIG_USB_EHCI_HCD
int board_usb_init(int index, enum usb_init_type init)
{
	int ret = 0;
#ifdef CONFIG_USB_TCPC
	struct tcpc_port *port_ptr;
#endif

	debug("board_usb_init %d, type %d\n", index, init);

	imx8m_usb_power(index, true);

#ifdef CONFIG_USB_TCPC
	if (index == 0)
		port_ptr = &port1;
	else
		port_ptr = &port2;

	if (init == USB_INIT_HOST)
		tcpc_setup_dfp_mode(port_ptr);
	else
		tcpc_setup_ufp_mode(port_ptr);
#endif

	return ret;
}

int board_usb_cleanup(int index, enum usb_init_type init)
{
	int ret = 0;

	debug("board_usb_cleanup %d, type %d\n", index, init);

#ifdef CONFIG_USB_TCPC
	if (init == USB_INIT_HOST) {
		if (index == 0)
			ret = tcpc_disable_src_vbus(&port1);
		else
			ret = tcpc_disable_src_vbus(&port2);
	}
#endif

	imx8m_usb_power(index, false);
	return ret;
}

#ifdef CONFIG_SPL_BUILD
int board_usb_phy_mode(struct udevice *dev)
#else
int board_ehci_usb_phy_mode(struct udevice *dev)
#endif
{
#ifdef CONFIG_USB_TCPC
	int ret = 0;
	enum typec_cc_polarity pol;
	enum typec_cc_state state;
	struct tcpc_port *port_ptr;

	if (dev->seq == 0)
		port_ptr = &port1;
	else
		port_ptr = &port2;

	tcpc_setup_ufp_mode(port_ptr);

	ret = tcpc_get_cc_status(port_ptr, &pol, &state);
	if (!ret) {
		if (state == TYPEC_STATE_SRC_RD_RA || state == TYPEC_STATE_SRC_RD)
			return USB_INIT_HOST;
	}
#endif /* CONFIG_USB_TCPC */

	return USB_INIT_DEVICE;
}
#endif

int board_init(void)
{
#ifdef CONFIG_USB_TCPC
	setup_typec();
#endif

#ifdef CONFIG_FEC_MXC
	setup_fec();
#endif

	return 0;
}

#ifdef CONFIG_VIDEO_MXS

/* TODO: video integration */
#define ADV7535_MAIN 0x3d
#define ADV7535_DSI_CEC 0x3c

static const struct sec_mipi_dsim_plat_data imx8mm_mipi_dsim_plat_data = {
	.version	= 0x1060200,
	.max_data_lanes = 4,
	.max_data_rate  = 1500000000ULL,
	.reg_base = MIPI_DSI_BASE_ADDR,
	.gpr_base = CSI_BASE_ADDR + 0x8000,
};

#define DISPLAY_MIX_SFT_RSTN_CSR		0x00
#define DISPLAY_MIX_CLK_EN_CSR		0x04

   /* 'DISP_MIX_SFT_RSTN_CSR' bit fields */
#define BUS_RSTN_BLK_SYNC_SFT_EN	BIT(6)

   /* 'DISP_MIX_CLK_EN_CSR' bit fields */
#define LCDIF_PIXEL_CLK_SFT_EN		BIT(7)
#define LCDIF_APB_CLK_SFT_EN		BIT(6)

void disp_mix_bus_rstn_reset(ulong gpr_base, bool reset)
{
	if (!reset)
		/* release reset */
		setbits_le32(gpr_base + DISPLAY_MIX_SFT_RSTN_CSR, BUS_RSTN_BLK_SYNC_SFT_EN);
	else
		/* hold reset */
		clrbits_le32(gpr_base + DISPLAY_MIX_SFT_RSTN_CSR, BUS_RSTN_BLK_SYNC_SFT_EN);
}

void disp_mix_lcdif_clks_enable(ulong gpr_base, bool enable)
{
	if (enable)
		/* enable lcdif clks */
		setbits_le32(gpr_base + DISPLAY_MIX_CLK_EN_CSR, LCDIF_PIXEL_CLK_SFT_EN | LCDIF_APB_CLK_SFT_EN);
	else
		/* disable lcdif clks */
		clrbits_le32(gpr_base + DISPLAY_MIX_CLK_EN_CSR, LCDIF_PIXEL_CLK_SFT_EN | LCDIF_APB_CLK_SFT_EN);
}

struct mipi_dsi_client_dev adv7535_dev = {
	.channel	= 0,
	.lanes = 4,
	.format  = MIPI_DSI_FMT_RGB888,
	.mode_flags = MIPI_DSI_MODE_VIDEO | MIPI_DSI_MODE_VIDEO_SYNC_PULSE |
			  MIPI_DSI_MODE_EOT_PACKET | MIPI_DSI_MODE_VIDEO_HSE,
	.name = "ADV7535",
};

struct mipi_dsi_client_dev rm67191_dev = {
	.channel	= 0,
	.lanes = 4,
	.format  = MIPI_DSI_FMT_RGB888,
	.mode_flags = MIPI_DSI_MODE_VIDEO | MIPI_DSI_MODE_VIDEO_SYNC_PULSE |
			  MIPI_DSI_MODE_EOT_PACKET | MIPI_DSI_MODE_VIDEO_HSE,
};

#define FSL_SIP_GPC			0xC2000000
#define FSL_SIP_CONFIG_GPC_PM_DOMAIN	0x3
#define DISPMIX				9
#define MIPI				10

void do_enable_mipi2hdmi(struct display_info_t const *dev)
{
	gpio_request(IMX_GPIO_NR(1, 8), "DSI EN");
	gpio_direction_output(IMX_GPIO_NR(1, 8), 1);

	/* ADV7353 initialization */
/* TODO: disable for now
	adv7535_init(); */

	/* enable the dispmix & mipi phy power domain */
	call_imx_sip(FSL_SIP_GPC, FSL_SIP_CONFIG_GPC_PM_DOMAIN, DISPMIX, true, 0);
	call_imx_sip(FSL_SIP_GPC, FSL_SIP_CONFIG_GPC_PM_DOMAIN, MIPI, true, 0);

	/* Put lcdif out of reset */
	disp_mix_bus_rstn_reset(imx8mm_mipi_dsim_plat_data.gpr_base, false);
	disp_mix_lcdif_clks_enable(imx8mm_mipi_dsim_plat_data.gpr_base, true);

	/* Setup mipi dsim */
	sec_mipi_dsim_setup(&imx8mm_mipi_dsim_plat_data);
	imx_mipi_dsi_bridge_attach(&adv7535_dev); /* attach adv7535 device */
}

void do_enable_mipi_led(struct display_info_t const *dev)
{
	gpio_request(IMX_GPIO_NR(1, 8), "DSI EN");
	gpio_direction_output(IMX_GPIO_NR(1, 8), 0);
	mdelay(100);
	gpio_direction_output(IMX_GPIO_NR(1, 8), 1);

	/* enable the dispmix & mipi phy power domain */
	call_imx_sip(FSL_SIP_GPC, FSL_SIP_CONFIG_GPC_PM_DOMAIN, DISPMIX, true, 0);
	call_imx_sip(FSL_SIP_GPC, FSL_SIP_CONFIG_GPC_PM_DOMAIN, MIPI, true, 0);

	/* Put lcdif out of reset */
	disp_mix_bus_rstn_reset(imx8mm_mipi_dsim_plat_data.gpr_base, false);
	disp_mix_lcdif_clks_enable(imx8mm_mipi_dsim_plat_data.gpr_base, true);

	/* Setup mipi dsim */
	sec_mipi_dsim_setup(&imx8mm_mipi_dsim_plat_data);

	rm67191_init();
	rm67191_dev.name = displays[1].mode.name;
	imx_mipi_dsi_bridge_attach(&rm67191_dev); /* attach rm67191 device */
}

void board_quiesce_devices(void)
{
	gpio_request(IMX_GPIO_NR(1, 8), "DSI EN");
	gpio_direction_output(IMX_GPIO_NR(1, 8), 0);
}

struct display_info_t const displays[] = {{
	.bus = LCDIF_BASE_ADDR,
	.addr = 0,
	.pixfmt = 24,
	.detect = NULL,
	.enable	= do_enable_mipi2hdmi,
	.mode	= {
		.name			= "MIPI2HDMI",
		.refresh		= 60,
		.xres			= 1920,
		.yres			= 1080,
		.pixclock		= 6734, /* 148500000 */
		.left_margin	= 148,
		.right_margin	= 88,
		.upper_margin	= 36,
		.lower_margin	= 4,
		.hsync_len		= 44,
		.vsync_len		= 5,
		.sync			= FB_SYNC_EXT,
		.vmode			= FB_VMODE_NONINTERLACED

} }, {
	.bus = LCDIF_BASE_ADDR,
	.addr = 0,
	.pixfmt = 24,
	.detect = NULL,
	.enable	= do_enable_mipi_led,
	.mode	= {
		.name			= "RM67191_OLED",
		.refresh		= 60,
		.xres			= 1080,
		.yres			= 1920,
		.pixclock		= 7575, /* 132000000 */
		.left_margin	= 34,
		.right_margin	= 20,
		.upper_margin	= 4,
		.lower_margin	= 10,
		.hsync_len		= 2,
		.vsync_len		= 2,
		.sync			= FB_SYNC_EXT,
		.vmode			= FB_VMODE_NONINTERLACED

} } };
size_t display_count = ARRAY_SIZE(displays);
#endif /* CONFIG_VIDEO_MXS */

static pcb_rev_t get_pcb_revision(void)
{
	struct udevice *bus;
	struct udevice *i2c_dev = NULL;
	int ret;
	uint8_t is_bd71837 = 0;

	ret = uclass_get_device_by_seq(UCLASS_I2C, I2C_PMIC, &bus);
	if (!ret)
		ret = dm_i2c_probe(bus, 0x4b, 0, &i2c_dev);
	if (!ret)
		ret = dm_i2c_read(i2c_dev, 0x0, &is_bd71837, 1);

	/* BD71837_REV, High Nibble is major version, fix 1010 */
	is_bd71837 = !ret && ((is_bd71837 & 0xf0) == 0xa0);
	return is_bd71837 ? PCB_VERSION_1_0 : PCB_VERSION_1_1;
}

static void select_dt_from_module_version(void)
{
	char variant[32];
	int is_wifi = 0;

#ifdef CONFIG_TDX_CFG_BLOCK
	/*
	 * If we have a valid config block and it says we are a module with
	 * Wi-Fi/Bluetooth make sure we use the -wifi device tree.
	 */
	is_wifi = tdx_hw_tag.prodid == VERDIN_IMX8MMQ_WIFI_BT_IT;
#endif

	switch(get_pcb_revision()) {
	case PCB_VERSION_1_0:
		printf("Detected a V1.0 module\n");
		if (is_wifi)
			strncpy(&variant[0], "wifi", sizeof(variant));
		else
			strncpy(&variant[0], "nonwifi", sizeof(variant));
		break;
	default:
		if (is_wifi)
			strncpy(&variant[0], "wifi-v1.1", sizeof(variant));
		else
			strncpy(&variant[0], "nonwifi-v1.1", sizeof(variant));
		break;
	}
	printf("Setting variant to %s\n", variant);
	env_set("variant", variant);
	env_set("variant1", "bla");
#ifndef CONFIG_ENV_IS_NOWHERE
	env_save();
#endif
}

int board_late_init(void)
{
	select_dt_from_module_version();

	return 0;
}

#ifdef CONFIG_FSL_FASTBOOT
#ifdef CONFIG_ANDROID_RECOVERY
int is_recovery_key_pressing(void)
{
	return 0; /* TODO */
}
#endif /*CONFIG_ANDROID_RECOVERY*/
#endif /*CONFIG_FSL_FASTBOOT*/
