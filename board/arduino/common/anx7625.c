// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright 2021 Arduino
 * Copyright 2021 Foundries.io
 */

#define DEBUG 1
#include <common.h>
#include <asm-generic/gpio.h>
#include <asm/mach-imx/gpio.h>
#include <dm.h>
#include <i2c.h>
#include <dm/uclass.h>
#include <linux/bitops.h>
#include <linux/delay.h>

#define ANX7625_DRV_VERSION       "0.1.04-uboot"

#define USE_PD_2_0_ONLY

/* @TODO: obtain gpios from dts */
#define I2C_ADDR_ANX_58           0x2c /* 0x58 */
#define I2C_ADDR_ANX_7E           0x3f /* 0x7e */

/* addr 58 registers */
#define TCPC_ROLE_CONTROL         0x1a
#define DRP_CONTROL                 (uint8_t)BIT(6)
#define TCPC_INTR_ALERT_1         0xcc
#define INTR_BIT2                   (uint8_t)BIT(2)
#define INTR_SOFTWARE_INT           (uint8_t)BIT(3)
#define INTR_BIT4                   (uint8_t)BIT(4)
#define INTR_RECEIVED_MSG           (uint8_t)BIT(5)

/* addr 7e registers */
#define FLASH_LOAD_STA            0x05
#define FLASH_LOAD_STA_CHK          (uint8_t)BIT(7)

/* Setting is in increments of 100mV */
#define MAX_VOLTAGE_SETTING_REG   0x29
/* 5V */
#define MAX_VOLTAGE_VALUE_V         5
#define MAX_VOLTAGE_VALUE_MV        (MAX_VOLTAGE_VALUE_V * 1000)
#define MAX_VOLTAGE_VALUE           (MAX_VOLTAGE_VALUE_V * 10)
/* Setting is in increments of 500mW */
#define MAX_POWER_SETTING_REG     0x2a
/* 5V @ 3000mA = 15000mW */
#define MAX_POWER_VALUE_MA          3000
#define MAX_POWER_VALUE_MW          (MAX_POWER_VALUE_MA * MAX_VOLTAGE_VALUE_V)
#define MAX_POWER_VALUE             (MAX_POWER_VALUE_MW / 500)

/* Setting is in increments of 500mW */
#define MIN_POWER_SETTING_REG     0x2b
/* 5V @ 900mA = 4500mW */
#define MIN_POWER_VALUE_MA          900
#define MIN_POWER_VALUE_MW          (MIN_POWER_VALUE_MA * MAX_VOLTAGE_VALUE_V)
#define MIN_POWER_VALUE             (MIN_POWER_VALUE_MW / 500)

#define MAX_RDO_VOLTAGE_SETTING   0x2c
#define RDO_VOLTAGE_UNIT_TO_V(x)    (x / 10)

#define MAX_RDO_POWER_SETTING     0x2d
#define RDO_POWER_UNIT_TO_MA(x)     ((x * 500) / MAX_VOLTAGE_VALUE_V)

#define AUTO_PD_MODE              0x2f
#define AUTO_PD_ENABLE_BIT              (uint8_t)BIT(1)
#define TRYSRC_EN                   (uint8_t)BIT(2)
#define TRYSNK_EN                   (uint8_t)BIT(3)
#define GOTO_SAFE5V_EN              (uint8_t)BIT(4)
#define OCM_FW_VERSION            0x31
#define OCM_FW_REVERSION          0x32
#define XTAL_FRQ_SEL              0x3f
#define INTERFACE_CHANGE_INT_MASK 0x43
#define INT_MASK_OFF                0
#define INTERFACE_CHANGE_INT      0x44
#define RECVD_MSG_INT               (uint8_t)BIT(0)
#define VCONN_CHG_INT               (uint8_t)BIT(2)
#define VBUS_CHG_INT                (uint8_t)BIT(3)
#define CC_STATUS_CHG_INT           (uint8_t)BIT(4)
#define DATA_ROLE_CHG_INT           (uint8_t)BIT(5)
#define PR_C_GOT_POWER              (uint8_t)BIT(6)
#define DP_HPD_CHANGE               (uint8_t)BIT(7)
#define INTERFACE_STATUS          0x45
#define CC_STATUS                 0x46

#define INTERFACE_PD_MSG_SEND_BUF 0xc0
#define INTERFACE_PD_MSG_RECV_BUF 0xe0
#define INTERFACE_PD_MSG_TIMEOUT_MS 3
#define INTERFACE_PD_MSG_RETRIES    3

enum anx7625_pd_msg_type {
	TYPE_PWR_SRC_CAP = 0x00,
	TYPE_PWR_SNK_CAP = 0x01,
	TYPE_DP_SNK_IDENTITY = 0x02,
	TYPE_SVID = 0x03,
	TYPE_GET_DP_SNK_CAP = 0x04,
	TYPE_ACCEPT = 0x05,
	TYPE_REJECT = 0x06,
	TYPE_PSWAP_REQ = 0x10,
	TYPE_DSWAP_REQ = 0x11,
	TYPE_GOTO_MIN_REQ = 0x12,
	TYPE_VCONN_SWAP_REQ = 0x13,
	TYPE_VDM = 0x14,
	TYPE_DP_SNK_CFG = 0x15,
	TYPE_PWR_OBJ_REQ = 0x16,
	TYPE_PD_STATUS_REQ = 0x17,
	TYPE_DP_ALT_ENTER = 0x19,
	TYPE_DP_ALT_EXIT = 0x1a,
	TYPE_GET_SNK_CAP = 0x1b,
	TYPE_SOP_PRIME = 0x1c,
	TYPE_SOP_DOUBLE_PRIME = 0x1d,
	TYPE_RESPONSE_TO_REQ = 0xf0,
	TYPE_SOFT_RST = 0xf1,
	TYPE_HARD_RST = 0xf2,
	TYPE_RESTART = 0xf3
};

/* PDO Fixed Flags */
#define PDO_FIXED_DATA_SWAP  ((uint32_t)BIT(25)) /* Data role swap command */
#define PDO_FIXED_COMM_CAP   ((uint32_t)BIT(26)) /* USB Communications Capable */
#define PDO_FIXED_DUAL_ROLE  ((uint32_t)BIT(29)) /* Dual role device */

/* Voltage in 50mV units */
#define PDO_FIXED_VOLT(mv)   (uint32_t)((((uint32_t)mv) / 50) << 10)
/* Max current in 10mA units */
#define PDO_FIXED_CURR(ma)   (uint32_t)((((uint32_t)ma) / 10))

/* Build a fixed PDO packet */
#define PDO_FIXED(mv, ma, flags) (PDO_FIXED_VOLT(mv) | PDO_FIXED_CURR(ma) | (flags))

#define POWER_DELIVERY_TIMEOUT    50 // [x10 = 500ms]
#define OCM_LOADING_TIME          10

#if defined(CONFIG_TARGET_PORTENTA_M8)
#define ANX7625_POWER_EN_PAD      IMX_GPIO_NR(4, 21)
#define ANX7625_RESET_N_PAD       IMX_GPIO_NR(4, 27)
#define LEDR_PAD                  IMX_GPIO_NR(5, 2)
#define LEDG_PAD                  IMX_GPIO_NR(4, 28)
#define LEDB_PAD                  IMX_GPIO_NR(4, 29)
#elif defined(CONFIG_TARGET_PORTENTA_X8)
#define ANX7625_POWER_EN_PAD      IMX_GPIO_NR(4, 25)
#define ANX7625_RESET_N_PAD       IMX_GPIO_NR(4, 26)
#define LEDR_PAD                  IMX_GPIO_NR(5, 2)
#define LEDG_PAD                  IMX_GPIO_NR(4, 28)
#define LEDB_PAD                  IMX_GPIO_NR(5, 1)
#else
#error ANX/LED GPIOs not configured for this board
#endif

#define ANX7625_DUMP_MSG              1
/* #define ANX7625_VERBOSE              1 */
#define ANX7625_NEGOTIATE_PD          1
#define ANX7625_NEGOTIATE_ERROR       1 /* If defined when negotiation fails blink red led repeatedly to warn the user and doesn't boot into kernel */
#define ANX7625_NEGOTIATE_ENDS_SETUP  1 /* If defined exits immediately after power negotiation is finished, otherwise waits for the timeout to occours */

static uint8_t last_chip_addr;

static void request_gpios(void)
{
	gpio_request(LEDR_PAD, "ledr");
	gpio_request(LEDG_PAD, "ledg");
	gpio_request(LEDB_PAD, "ledb");

	gpio_request(ANX7625_POWER_EN_PAD, "anx_power_en");
	gpio_request(ANX7625_RESET_N_PAD, "anx_reset_n");
}

static void free_gpios(void)
{
	gpio_free(LEDR_PAD);
	gpio_free(LEDG_PAD);
	gpio_free(LEDB_PAD);

	gpio_free(ANX7625_POWER_EN_PAD);
	gpio_free(ANX7625_RESET_N_PAD);
}

static void leds_off(void)
{
	gpio_direction_output(LEDR_PAD, 1);
	gpio_direction_output(LEDG_PAD, 1);
	gpio_direction_output(LEDB_PAD, 1);
}

#if defined(ANX7625_NEGOTIATE_PD) && defined(ANX7625_NEGOTIATE_ERROR)
static void led_red_on(void)
{
	gpio_direction_output(LEDR_PAD, 0);
}

static void led_red_off(void)
{
	gpio_direction_output(LEDR_PAD, 1);
}
#endif

/*
 * There is a sync issue while access I2C register between AP(CPU) and
 * internal firmware(OCM), to avoid the race condition, AP should access
 * the reserved slave address before slave address occurs changes.
 */
static int i2c_access_workaround(struct udevice *dev)
{
	struct dm_i2c_chip *chip;
	uint offset;
	uint8_t valb = 0x0;

	chip = dev_get_parent_plat(dev);
	if (!chip) {
		printf("%s: can't find chip!\n", __func__);
		return -ENODEV;
	}

	if (chip->chip_addr == last_chip_addr) {
		return 0;
	}

	last_chip_addr = chip->chip_addr;
	if (last_chip_addr == I2C_ADDR_ANX_58)
		offset = 0x0;
	else
		offset = 0x39;

	return dm_i2c_write(dev, offset, &valb, 1);
}

/* helper function to avoid needing a local valb everywhere */
static int dm_i2c_writeb(struct udevice *dev, uint offset, const uint8_t buffer)
{
	uint8_t valb = buffer;

	i2c_access_workaround(dev);
	return dm_i2c_write(dev, offset, &valb, 1);
}

static int anx7625_power_on(struct udevice *dev)
{
	int ret = 0;

	debug("%s\n", __func__);
	gpio_direction_output(ANX7625_POWER_EN_PAD, 1);
	udelay(10000); /* 10ms */
	gpio_direction_output(ANX7625_RESET_N_PAD, 1);
#ifndef USE_PD_2_0_ONLY
	udelay(10000); /* 10ms */
#else
	udelay(1000); /* 1ms */
	ret = dm_i2c_writeb(dev, 0xDF, 0x01); // Set PD2.0 version
	ret |= dm_i2c_writeb(dev, 0x2F, 0x02); // Force OCM to check version specified in previous reg
	if (ret) {
		printf("%s %d dm_i2c_write failed, err %d\n", __func__, __LINE__, ret);
		return -EIO;
	}
	udelay(9000); /* 1ms */
#endif
	return ret;
}

static void anx7625_standby(void)
{
	debug("%s\n", __func__);
	gpio_direction_output(ANX7625_RESET_N_PAD, 0);
	udelay(10000); /* 10ms */
	gpio_direction_output(ANX7625_POWER_EN_PAD, 0);
	udelay(10000); /* 10ms */
}

static uint8_t anx7625_msg_calc_checksum(uint8_t *msg_buf, uint8_t msg_size)
{
	uint8_t checksum = 0;

	for (int i = 0; i < msg_size; i++)
		checksum += msg_buf[i];

	return (uint8_t)(0 - checksum);
}

#ifdef ANX7625_DUMP_MSG
static char *anx7625_msg_type_str(enum anx7625_pd_msg_type msg_type)
{
	return  (msg_type == TYPE_PWR_SRC_CAP) ? "SRC CAP" :
		(msg_type == TYPE_PWR_SNK_CAP) ? "SNK CAP" :
		(msg_type == TYPE_PWR_OBJ_REQ) ? "RDO" :
		(msg_type == TYPE_DP_SNK_IDENTITY) ? "SNK identity" :
		(msg_type == TYPE_SVID) ? "SVID" :
		(msg_type == TYPE_PSWAP_REQ) ? "PR_SWAP" :
		(msg_type == TYPE_DSWAP_REQ) ? "DR_SWAP" :
		(msg_type == TYPE_GOTO_MIN_REQ) ? "GOTO_MIN" :
		(msg_type == TYPE_DP_ALT_ENTER) ? "DPALT_ENTER" :
		(msg_type == TYPE_DP_ALT_EXIT) ? "DPALT_EXIT" :
		(msg_type == TYPE_VCONN_SWAP_REQ) ? "VCONN_SWAP" :
		(msg_type == TYPE_GET_DP_SNK_CAP) ? "GET_SINK_DP_CAP" :
		(msg_type == TYPE_DP_SNK_CFG) ? "dp cap" :
		(msg_type == TYPE_SOFT_RST) ? "Soft Reset" :
		(msg_type == TYPE_HARD_RST) ? "Hard Reset" :
		(msg_type == TYPE_RESTART) ? "Restart" :
		(msg_type == TYPE_PD_STATUS_REQ) ? "PD Status" :
		(msg_type == TYPE_ACCEPT) ? "ACCEPT" :
		(msg_type == TYPE_REJECT) ? "REJECT" :
		(msg_type == TYPE_VDM) ? "VDM" :
		(msg_type == TYPE_RESPONSE_TO_REQ) ? "Response to Request" :
		(msg_type == TYPE_SOP_PRIME) ? "SOP'" :
		(msg_type == TYPE_SOP_DOUBLE_PRIME) ? "SOP\"" :
		"Unknown";
}

static void anx7625_msg_dump(const uint8_t *msg_buf, uint8_t msg_size)
{
        debug("%s:", __func__);
	for (int i = 0; i < msg_size; i++) {
		if (i == 0)
	                debug(" (%d)", msg_buf[i]);
		else if (i == 1)
	                debug(" [0x%02x/%s]", msg_buf[i], anx7625_msg_type_str(msg_buf[i]));
		else if (i == msg_size - 1)
	                debug(" {0x%02x}", msg_buf[i]);
		else
	                debug(" 0x%02x", msg_buf[i]);
	}
	debug("\n");
}
#endif

static int anx7625_msg_wait_for_ready(struct udevice *dev, int timeout_ms)
{
	int ret, count = 0;
	uint8_t valb = 0;

	if (timeout_ms) {
		while (count <= (timeout_ms * 10)) {
			ret = dm_i2c_read(dev, INTERFACE_PD_MSG_SEND_BUF, &valb, 1);
			if (ret)
				return ret;
			else if (valb == 0)
				break;

			count++;
			udelay(100); /* 100us */
		}

		if (valb != 0)
			return -ETIMEDOUT;
	}

	return 0;
}

static int anx7625_msg_send(struct udevice *dev, enum anx7625_pd_msg_type msg_type,
			    const uint8_t *msg_buf, uint8_t msg_size)
{
	int ret;
	uint8_t pd_msg_buffer[32];

	pd_msg_buffer[0] = msg_size + 1;
	pd_msg_buffer[1] = (uint8_t)msg_type;
	if (msg_size > 0)
		memcpy(&pd_msg_buffer[2], msg_buf, msg_size);
	pd_msg_buffer[msg_size + 2] = anx7625_msg_calc_checksum(pd_msg_buffer, msg_size + 2);

	i2c_access_workaround(dev);

	ret = anx7625_msg_wait_for_ready(dev, INTERFACE_PD_MSG_TIMEOUT_MS);
	if (ret)
		return ret;

#ifdef ANX7625_DUMP_MSG
	anx7625_msg_dump(pd_msg_buffer, pd_msg_buffer[0] + 2);
#endif
	ret = dm_i2c_write(dev, INTERFACE_PD_MSG_SEND_BUF + 1, &pd_msg_buffer[1], msg_size + 2);
	if (ret) {
		printf("%s dm_i2c_write failed, err %d\n", __func__, ret);
		return -EIO;
	}

	ret = dm_i2c_writeb(dev, INTERFACE_PD_MSG_SEND_BUF, pd_msg_buffer[0]);
	if (ret) {
		printf("%s dm_i2c_write failed, err %d\n", __func__, ret);
		return -EIO;
	}

	return 0;
}

/* Remember to clear the ALERT register. Set 0xCC bit5 as 1 to receive the next message */
static int anx7625_msg_recv(struct udevice *dev, enum anx7625_pd_msg_type *msg_type,
			    uint8_t *msg_buf, uint8_t *msg_size, uint8_t retries)
{
	static uint8_t pd_msg_recv_buffer[32];
	int ret, count = 0;
	uint8_t checksum = 0;

	while (count < retries) {
		ret = dm_i2c_read(dev, INTERFACE_PD_MSG_RECV_BUF, pd_msg_recv_buffer, 32);
		if (ret) {
			printf("%s: dm_i2c_read failed, err:%d\n", __func__, ret);
			return ret;
		}

		if (pd_msg_recv_buffer[0] == 0) {
			count++;
			continue;
		}

		/* length check */
		if (pd_msg_recv_buffer[0] >= 32) {
			printf("%s: bad length: %d\n", __func__, pd_msg_recv_buffer[0]);
			return -EINVAL;
		}

#ifdef ANX7625_DUMP_MSG
		anx7625_msg_dump(pd_msg_recv_buffer, pd_msg_recv_buffer[0] + 2);
#endif

		/* checksum */
		checksum = anx7625_msg_calc_checksum(pd_msg_recv_buffer, pd_msg_recv_buffer[0] + 1);
		if (pd_msg_recv_buffer[pd_msg_recv_buffer[0] + 1] != checksum) {
			printf("%s: checksum error [0x%x vs. 0x%x]\n", __func__,
			       pd_msg_recv_buffer[pd_msg_recv_buffer[0] + 1],
			       checksum);
			return -EINVAL;
		}

		/* copy data */
		*msg_size = pd_msg_recv_buffer[0] - 1;
		*msg_type = (enum anx7625_pd_msg_type) pd_msg_recv_buffer[1];
		if (*msg_size > 0)
			memcpy(msg_buf, &pd_msg_recv_buffer[2], *msg_size);

		/* clear recv packet */
		dm_i2c_writeb(dev, INTERFACE_PD_MSG_RECV_BUF, 0);
		return 0;
	}

	return -ETIMEDOUT;
}

/* @TODO: @MS please consider below what I know from PD
 * DFP only mode: we always start as UFP(Rd), the other device is always DFP(Rp)
 * DRP mode: after connection we switch between Rp(DFP) and Rd(UFP) until we detect a valid connection. When
 * connection is detected, start either in UFP or DFP mode.
 * The DFP sends SOURCE_CAPS in loop immediately after connection and waits for ack.
 * In case DFP becames SINK (or SINK becames DFP like in our case) then SINK needs to send SNK_CAPS that need
 * to be accepted by other device. This is because current sink may be different between DFP and UFP modes.
 * In DFP only mode we can switch data role (became DFP) after POWER_RDY message. The Rp/Rd connection
 * remains fixed, only data role is changed.
 * We currently setup anx to work in DFP only mode (it has no limitations for the features we want to implement)
 * so below messages are likely to be ignored by hub or psu.
 * Identity should be provided under request from discover identity message. DFP sends discovery identity message.
 * Android driver is implementing anx in DRP mode.
 */
static int anx7625_setup_pd_cap(struct udevice *dev)
{
	int ret;

	/* TODO: locate data in dts */

	static uint32_t init_src_caps[1] = {
		/*5000mV, 3A, Fixed */
		PDO_FIXED(MAX_VOLTAGE_VALUE_MV, MAX_POWER_VALUE_MA, PDO_FIXED_DUAL_ROLE | PDO_FIXED_COMM_CAP | PDO_FIXED_DATA_SWAP),
	};
	static uint32_t init_snk_caps[3] = {
		/*5000mV, 3A, Fixed */
		PDO_FIXED(MAX_VOLTAGE_VALUE_MV, MAX_POWER_VALUE_MA, PDO_FIXED_DUAL_ROLE | PDO_FIXED_COMM_CAP | PDO_FIXED_DATA_SWAP),
	};
	static uint8_t init_dp_snk_ident[16] = {
		0x00, 0x00, 0x00, 0xec, /* snk_id_hdr */
		0x00, 0x00, 0x00, 0x00, /* snk_cert */
		0x00, 0x00, 0x00, 0x00, /* snk_prd */
		0x39, 0x00, 0x00, 0x51  /* 5snk_ama */
	};
	static uint8_t init_svid[4] = { 0x00, 0x00, 0x01, 0xff };

	/* SRC capabilities */
	ret = anx7625_msg_send(dev, TYPE_PWR_SRC_CAP,
			       (const uint8_t *)init_src_caps, sizeof(init_src_caps));
	if (ret) {
		debug("%s: failed to send SRC cap\n", __func__);
		return -EIO;
	}

	/* SNK capabilities */
	ret = anx7625_msg_send(dev, TYPE_PWR_SNK_CAP,
			       (const uint8_t *)init_snk_caps, sizeof(init_snk_caps));
	if (ret) {
		debug("%s: failed to send SNK cap\n", __func__);
		return -EIO;
	}

	/* DP SNK ident */
	ret = anx7625_msg_send(dev, TYPE_DP_SNK_IDENTITY,
			       init_dp_snk_ident, sizeof(init_dp_snk_ident));
	if (ret) {
		debug("%s: failed to send DP SNK ident\n", __func__);
		return -EIO;
	}

	/* SVID ident */
	ret = anx7625_msg_send(dev, TYPE_SVID,
			       init_svid, sizeof(init_svid));
	if (ret) {
		debug("%s: failed to send SVID\n", __func__);
		return -EIO;
	}

	debug("%s: succeeded\n", __func__);
	return 0;
}

static int anx7625_config(struct udevice *dev)
{
	int ret = 0;

	debug("%s\n", __func__);
	i2c_access_workaround(dev);

	/* Set XTAL 27M */
	ret = dm_i2c_writeb(dev, XTAL_FRQ_SEL, 0x80);
	if (ret) {
		printf("%s dm_i2c_write failed, err %d\n", __func__, ret);
		return -EIO;
	}

	return 0;
}

static int anx7625_ocm_loading_check(struct udevice *dev)
{
	uint8_t valb = 0;

	debug("%s\n", __func__);
	i2c_access_workaround(dev);

	/* Check interface workable */
	dm_i2c_read(dev, FLASH_LOAD_STA, &valb, 1);
	if ((valb & FLASH_LOAD_STA_CHK) != FLASH_LOAD_STA_CHK)
		return -ENODEV;

	return 0;
}

/* Auto rdo means auto pd negotiation */
static int anx7625_disable_auto_rdo(struct udevice *dev)
{
	int ret = 0;
	uint8_t valb = 0;

	debug("%s\n", __func__);
	i2c_access_workaround(dev);

	ret = dm_i2c_read(dev, AUTO_PD_MODE, &valb, 1);
	if (ret) {
		printf("%s failed, err %d\n", __func__, ret);
		return -EIO;
	}

	return dm_i2c_writeb(dev, AUTO_PD_MODE, valb & ~AUTO_PD_ENABLE_BIT);
}

/* Auto rdo means auto pd negotiation */
static int anx7625_enable_auto_rdo(struct udevice *dev)
{
	int ret = 0;
	uint8_t valb = 0;

	debug("%s\n", __func__);
	i2c_access_workaround(dev);

	ret = dm_i2c_read(dev, AUTO_PD_MODE, &valb, 1);
	if (ret) {
		printf("%s failed, err %d\n", __func__, ret);
		return -EIO;
	}

	return dm_i2c_writeb(dev, AUTO_PD_MODE, valb | AUTO_PD_ENABLE_BIT);
}

static int anx7625_disable_safe_5v_during_auto_rdo(struct udevice *dev)
{
	int ret = 0;
	uint8_t valb = 0;

	debug("%s\n", __func__);
	i2c_access_workaround(dev);

	ret = dm_i2c_read(dev, AUTO_PD_MODE, &valb, 1);
	if (ret) {
		printf("%s failed, err %d\n", __func__, ret);
		return -EIO;
	}

	return dm_i2c_writeb(dev, AUTO_PD_MODE, valb & ~GOTO_SAFE5V_EN);
}

/* We should set role control (hw) before sending anything on CC bus */
static int anx7625_setup_role_control(struct udevice *dev)
{
	int ret = 0;
	uint8_t valb = 0;

	i2c_access_workaround(dev);

	ret = dm_i2c_read(dev, TCPC_ROLE_CONTROL, &valb, 1);
	if (ret) {
		printf("%s failed dm_i2c_read TCPC_ROLE_CONTROL, err %d\n", __func__, ret);
		return -EIO;
	}

	return dm_i2c_writeb(dev, TCPC_ROLE_CONTROL, valb & ~DRP_CONTROL);
}

static int anx7625_chip_register_init(struct udevice *dev_p0, struct udevice *dev_typec)
{
	int ret;
	uint8_t valb = 0;

	i2c_access_workaround(dev_p0);

	/* Clearing whole interrupt vector mask */
	ret = dm_i2c_writeb(dev_p0, INTERFACE_CHANGE_INT_MASK, INT_MASK_OFF);
	if (ret) {
		printf("%s %d dm_i2c_write failed, err %d\n", __func__, __LINE__, ret);
		return -EIO;
	}

	anx7625_disable_auto_rdo(dev_p0);

	//anx7625_disable_safe_5v_during_auto_rdo(dev_p0); /* @TODO: @MS not needed, safe 5V is 5V@500mA, we don't need to disable this choice just elaborate obtained power later on */

	ret = dm_i2c_writeb(dev_p0, MAX_VOLTAGE_SETTING_REG, MAX_VOLTAGE_VALUE);
	if (ret) {
		printf("%s %d dm_i2c_write failed, err %d\n", __func__, __LINE__, ret);
		return -EIO;
	}

	ret = dm_i2c_writeb(dev_p0, MAX_POWER_SETTING_REG, MAX_POWER_VALUE);
	if (ret) {
		printf("%s %d dm_i2c_write failed, err %d\n", __func__, __LINE__, ret);
		return -EIO;
	}

	ret = dm_i2c_writeb(dev_p0, MIN_POWER_SETTING_REG, MIN_POWER_VALUE);
	if (ret) {
		printf("%s %d dm_i2c_write failed, err %d\n", __func__, __LINE__, ret);
		return -EIO;
	}

	/* Try sink or source @TODO: need to read default policy from dts */
	ret = dm_i2c_read(dev_p0, AUTO_PD_MODE, &valb, 1);
	if (ret) {
		printf("%s %d dm_i2c_read failed, err %d\n", __func__, __LINE__, ret);
		return -EIO;
	}

	/* Try sink or source @TODO: need to read default policy from dts */
	ret = dm_i2c_writeb(dev_p0, AUTO_PD_MODE, (valb | TRYSNK_EN) & ~TRYSRC_EN);
	//ret = dm_i2c_writeb(dev_p0, AUTO_PD_MODE, (valb | TRYSRC_EN) & ~TRYSNK_EN);
	if (ret) {
		printf("%s %d dm_i2c_write failed, err %d\n", __func__, __LINE__, ret);
		return -EIO;
	}

	/* HW setup (DFP/DRP) needs to be done before sending any pd message */
	anx7625_setup_role_control(dev_typec);

	anx7625_enable_auto_rdo(dev_p0); /* This should triggers OCM to handle negotiation with configured values */

	return 0;
}

static int anx7625_setup(struct udevice *dev_typec, struct udevice *dev_p0)
{
	int ret = -ENODEV, retry_count, i;
	uint8_t ver, rver;

	debug("%s\n", __func__);
	for (retry_count = 0; retry_count < 5; retry_count++) {
		if(anx7625_power_on(dev_p0)) {
			return -EIO;
		}

		if (anx7625_config(dev_p0)) {
			return -EIO;
		}

		for (i = 0; i < OCM_LOADING_TIME; i++) {
			if (anx7625_ocm_loading_check(dev_p0) == 0) {
				printf("%s: OCM started\n", __func__);

				ret = anx7625_chip_register_init(dev_p0, dev_typec);
	                        if (ret)
					printf("%s: chip init registers failed!\n", __func__);
				else
					printf("%s: chip init registers succeeded.\n", __func__);

				/*  @TODO: @MS Cannot see messages below with analyzer, seems not working */
				/* Any message need to be sent after hw register configuration (anx7625_chip_register_init) */
				/*ret = anx7625_setup_pd_cap(dev_p0);
				if (ret) {
					printf("%s: setup pd cap failed!\n", __func__);
					return -EIO;
				}*/

				dm_i2c_read(dev_p0, OCM_FW_VERSION, &ver, 1);
				dm_i2c_read(dev_p0, OCM_FW_REVERSION, &rver, 1);

				printf("Driver ver %s\n", ANX7625_DRV_VERSION);
				printf("Firmware ver %02x%02x\n", ver, rver);
				return 0;
			}
			udelay(1000); /* 1ms */
		}

		anx7625_standby();
	}

	return ret;
}

#ifdef ANX7625_NEGOTIATE_PD
static void anx7625_print_cc_status(const char *cc, uint8_t cc_status) {
	switch (cc_status) {
		case 0x0:
			debug("%s: SRC.Open\n", cc);
			break;
		case 0x1:
			debug("%s: SRC.Rd\n", cc);
			break;
		case 0x2:
			debug("%s: SRC.Ra\n", cc);
			break;
		case 0x4:
			debug("%s: SNK.Default\n", cc);
			break;
		case 0x8:
			debug("%s: SNK.Power1.5\n", cc);
			break;
		case 0xc:
			debug("%s: SNK.Power3.0\n", cc);
			break;
		default:
			debug("%s: Unknown status (0x%x)\n", cc, cc_status);
			break;
	}
}

/* This function waits until PR_C_GOT_POWER is asserted or timeouts. This bit
 * is set when anx7625 receive PS_RDY from the hub.
 * In USB_PD_R2_0 V1.3 - 20170112.pdf p. 280 we find the parameter
 * tSrcReady_max = 285ms. We rounded up to 300ms.
 * This time shall be used as timeout value for this function. If no power delivery
 * negotiation happened in this time span, then the device is considered no power delivery
 * capable.
 * */
static int anx7625_negotiate_pd(struct udevice *dev_typec, struct udevice *dev_p0)
{
	int ret, min_power_setting_ma;
	unsigned int count = 0;
	enum anx7625_pd_msg_type msg_type;
	uint8_t msg_buffer[32];
	uint8_t p0_isr, last_cc1 = 0, last_cc2 = 0, valb = 0;

	printf("Request:\n");
	ret = dm_i2c_read(dev_p0, MAX_VOLTAGE_SETTING_REG, &valb, 1);
	if (ret) {
			printf("%s dm_i2c_read MAX_VOLTAGE_SETTING_REG failed, err %d\n", __func__, ret);
			return -EIO;
	}
	printf("  Voltage Max %d [V]\n", RDO_VOLTAGE_UNIT_TO_V(valb));

	ret = dm_i2c_read(dev_p0, MIN_POWER_SETTING_REG, &valb, 1);
	if (ret) {
			printf("%s dm_i2c_read MIN_POWER_SETTING_REG failed, err %d\n", __func__, ret);
			return -EIO;
	}
	min_power_setting_ma = RDO_POWER_UNIT_TO_MA(valb);
	printf("  Power Min 5V @ %d [mA]\n", min_power_setting_ma);

	ret = dm_i2c_read(dev_p0, MAX_POWER_SETTING_REG, &valb, 1);
	if (ret) {
			printf("%s dm_i2c_read MAX_POWER_SETTING_REG failed, err %d\n", __func__, ret);
			return -EIO;
	}
	printf("  Power Max 5V @ %d [mA]\n", RDO_POWER_UNIT_TO_MA(valb));

	printf("Begin Negotiation:\n");
	while (count < POWER_DELIVERY_TIMEOUT) { /* Begin negotiation loop. Every loop lasts 10ms. */
		if (count % 10 == 0) debug("%s: count:%u\n", __func__, count / 10); /* Print count every 100ms */

		/*  Interface and Status INT */
		ret = dm_i2c_read(dev_p0, INTERFACE_CHANGE_INT, &p0_isr, 1);
		if (ret) {
			printf("%s %d dm_i2c_read INTERFACE_CHANGE_INT failed, err %d\n", __func__, __LINE__, ret);
			return -EIO;
		}

#ifdef ANX7625_VERBOSE
		debug("Interface and Status INT: 0x%02x\n", p0_isr);
#endif

		/*  Interface Status */
		ret = dm_i2c_read(dev_p0, INTERFACE_STATUS, &valb, 1);
		if (ret) {
			printf("%s %d dm_i2c_read INTERFACE_STATUS failed, err %d\n", __func__, __LINE__, ret);
			return -EIO;
		}

#ifdef ANX7625_VERBOSE
		debug("  Interface Status: 0x%02x\n", valb);
#endif

		/* Begin Parse Interface and Status INT */
		if (p0_isr & RECVD_MSG_INT) {
			ret = anx7625_msg_recv(dev_p0, &msg_type, msg_buffer, &valb, INTERFACE_PD_MSG_RETRIES);
			if (ret) {
				printf("%s anx7625_msg_recv failed, err %d\n", __func__, ret);
			}
			p0_isr &= ~RECVD_MSG_INT;
			dm_i2c_writeb(dev_p0, INTERFACE_CHANGE_INT, p0_isr);
		}

		if (p0_isr & VCONN_CHG_INT) {
			debug("  INT: VCONN CHANGE\n");
			p0_isr &= ~VCONN_CHG_INT;
			dm_i2c_writeb(dev_p0, INTERFACE_CHANGE_INT, p0_isr);
		}

		if (p0_isr & VBUS_CHG_INT) {
			debug("  INT: VBUS CHANGE\n");
			p0_isr &= ~VBUS_CHG_INT;
			dm_i2c_writeb(dev_p0, INTERFACE_CHANGE_INT, p0_isr);
		}

		if (p0_isr & CC_STATUS_CHG_INT) {
			ret = dm_i2c_read(dev_p0, CC_STATUS, &valb, 1);
			if (ret) {
				printf("%s %d dm_i2c_read failed, CC_STATUS err %d\n", __func__, __LINE__, ret);
				return -EIO;
			}

			if ((last_cc1 != (valb & 0xf)) || (last_cc2 != (valb >> 4))) {
				last_cc1 = valb & 0xf;
				last_cc2 = valb >> 4;
				debug("  INT: CC STATUS CHANGE\n");
				anx7625_print_cc_status("  CC1", last_cc1);
				anx7625_print_cc_status("  CC2", last_cc2);
			}

			p0_isr &= ~CC_STATUS_CHG_INT;
			dm_i2c_writeb(dev_p0, INTERFACE_CHANGE_INT, p0_isr);
		}

		if (p0_isr & DATA_ROLE_CHG_INT) {
			debug("  INT: DATA ROLE CHANGE\n");
			p0_isr &= ~DATA_ROLE_CHG_INT;
			dm_i2c_writeb(dev_p0, INTERFACE_CHANGE_INT, p0_isr);
		}

		if (p0_isr & PR_C_GOT_POWER) {
			debug("  INT: POWER NEGOTIATION FINISHED\n");
			ret = dm_i2c_read(dev_p0, MAX_RDO_VOLTAGE_SETTING, &valb, 1);
			if(ret) {
				printf("%s %d dm_i2c_read failed, err %d\n", __func__, __LINE__, ret);
				return -EIO;
			}
			debug("  Voltage: %d [V]\n", RDO_VOLTAGE_UNIT_TO_V(valb));

			ret = dm_i2c_read(dev_p0, MAX_RDO_POWER_SETTING, &valb, 1);
			if(ret) {
				printf("%s %d dm_i2c_read failed, err %d\n", __func__, __LINE__, ret);
				return -EIO;
			}
			debug("  Power: 5V @ %d [mA]\n", (valb * 500) / 5);

#ifdef ANX7625_NEGOTIATE_ERROR
			if (valb < MIN_POWER_VALUE) {
				printf("Power negotiation failure (5V @ %d [mA] below min setting 5V @ %d [mA])\n",
					   RDO_POWER_UNIT_TO_MA(valb), MIN_POWER_VALUE_MA);;
				printf("Please change power adapter and/or USB-C hub and retry.\n");

				while (1) {
					led_red_on();
					udelay(200000);
					led_red_off();
					udelay(800000);
				}
			}
#endif
			printf("Power negotiation COMPLETE\n");
			p0_isr &= ~PR_C_GOT_POWER;
			dm_i2c_writeb(dev_p0, INTERFACE_CHANGE_INT, p0_isr);
#ifdef ANX7625_NEGOTIATE_ENDS_SETUP
			break;
#endif
		}

		if (p0_isr & DP_HPD_CHANGE) {
			debug("  INT: DP HDP CHANGE\n");
			p0_isr &= ~DP_HPD_CHANGE;
			dm_i2c_writeb(dev_p0, INTERFACE_CHANGE_INT, p0_isr);
		}
		/* End Parse Interface and Status INT */

		//dm_i2c_writeb(dev_typec, TCPC_INTR_ALERT_1, 0xff); /* Among other things clears the ALERT register. Set 0xCC bit5 as 1 to receive the next message @TODO: necessary? */
		udelay(10000); /* 10ms */
		count++;
	} /* End negotiation loop */

	return 0;
}
#endif

int anx7625_probe(uint8_t i2c_bus_num)
{
	struct udevice *bus_i2c1;
	struct udevice *dev_typec;
	struct udevice *dev_p0;

	debug("%s: start\n", __func__);

	request_gpios();
	leds_off();

	if (uclass_get_device_by_seq(UCLASS_I2C, i2c_bus_num, &bus_i2c1)) {
		printf("%s: Can't find bus\n", __func__);
		return -EINVAL;
	}

	if (i2c_get_chip(bus_i2c1, I2C_ADDR_ANX_58, 1, &dev_typec)) {
		printf("%s: Can't get i2c chip id=0x%x\n",
		       __func__, I2C_ADDR_ANX_58);
		return -ENODEV;
	}

	if (i2c_get_chip(bus_i2c1, I2C_ADDR_ANX_7E, 1, &dev_p0)) {
		printf("%s: Can't get i2c chip id=0x%x\n",
			__func__, I2C_ADDR_ANX_7E);
		return -ENODEV;
	}

	if (anx7625_setup(dev_typec, dev_p0)) {
		printf("%s: Failed anx7625 setup\n", __func__);
		return -EIO;
	}

#ifdef ANX7625_NEGOTIATE_PD
	anx7625_negotiate_pd(dev_typec, dev_p0);
#endif
	free_gpios();

	debug("%s: done\n", __func__);
	return 0;
}
