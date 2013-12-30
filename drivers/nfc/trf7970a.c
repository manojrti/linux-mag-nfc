/**
 * trf7970a.c - TI's TRF7970a RFID/NFC Transceiver Driver
 *
 * Copyright (C) 2013 Texas Instruments Incorporated - http://www.ti.com
 *
 * Author: Erick Macias <emacias@ti.com>
 * Author: Felipe Balbi <balbi@ti.com>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2  of
 * the License as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/device.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/bitops.h>
#include <linux/mutex.h>
#include <linux/regmap.h>
#include <linux/spi/spi.h>
#include <linux/interrupt.h>
#include <linux/nfc.h>
#include <linux/skbuff.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/of.h>
#include <linux/of_gpio.h>

#include <net/nfc/nfc.h>
#include <net/nfc/digital.h>

/* REVISIT This device supports NFC, ISO14443A, ISO14443B,
 * FELICA, ISO15693 and ISO18000-3.
 *
 * Currently, NFC framework doesn't support ISO15693
 * or ISO18000-3.
 *
 * For now, enable only those which framework supports
 */
#if 0 /* XXX */
#define TRF7970A_PROTOCOLS	(NFC_PROTO_NFC_DEP_MASK |\
				NFC_PROTO_MIFARE_MASK | \
				NFC_PROTO_FELICA_MASK | \
				NFC_PROTO_ISO14443_MASK | \
				NFC_PROTO_ISO14443_B_MASK | \
				NFC_PROTO_ISO15693_MASK)
#else
#define TRF7970A_PROTOCOLS	(NFC_PROTO_MIFARE_MASK | \
				NFC_PROTO_ISO15693_MASK)
#endif

/* Direct Commands */
#define TRF7970A_CMD_IDLE			0x00
#define TRF7970A_CMD_SOFT_INIT			0x03
#define TRF7970A_CMD_RF_COLLISION		0x04
#define TRF7970A_CMD_RF_COLLISION_RESPONSE_N	0x05
#define TRF7970A_CMD_RF_COLLISION_RESPONSE_0	0x06
#define TRF7970A_CMD_RESET			0x0f
#define TRF7970A_CMD_TRANSMIT_NO_CRC		0x10
#define TRF7970A_CMD_TRANSMIT			0x11
#define TRF7970A_CMD_DELAY_TRANSMIT_NO_CRC	0x12
#define TRF7970A_CMD_DELAY_TRANSMIT		0x13
#define TRF7970A_CMD_EOF			0x14
#define TRF7970A_CMD_CLOSE_SLOT			0x15
#define TRF7970A_CMD_BLOCK_RX			0x16
#define TRF7970A_CMD_ENABLE_RX			0x17
#define TRF7970A_CMD_TEST_EXT_RF		0x18
#define TRF7970A_CMD_TEST_INT_RF		0x19
#define TRF7970A_CMD_RX_GAIN_ADJUST		0x1a

/* Must be set when issuing direct commands */
#define TRF7970A_CMD_CTRL			BIT(7)
#define TRF7970A_CMD_RW				BIT(6)
#define TRF7970A_CMD_CONTINUOUS			BIT(5)

#define TRF7970A_CMD_OPCODE(opcode)		((opcode) & 0x1f)

/* Registers */
#define TRF7970A_CHIP_STATUS_CTRL		0x00
#define TRF7970A_ISO_CTRL			0x01
#define TRF7970A_ISO14443B_TX_OPTIONS		0x02
#define TRF7970A_ISO14443A_HIGH_BITRATE_OPTIONS	0x03
#define TRF7970A_TX_TIMER_SETTING_H_BYTE	0x04
#define TRF7970A_TX_TIMER_SETTING_L_BYTE	0x05
#define TRF7970A_TX_PULSE_LENGTH_CTRL		0x06
#define TRF7970A_RX_NO_RESPONSE_WAIT		0x07
#define TRF7970A_RX_WAIT_TIME			0x08
#define TRF7970A_MODULATOR_SYS_CLK_CTRL		0x09
#define TRF7970A_RX_SPECIAL_SETTINGS		0x0a
#define TRF7970A_REG_IO_CTRL			0x0b
#define TRF7970A_IRQ_STATUS			0x0c
#define TRF7970A_COLLISION_IRQ_MASK		0x0d
#define TRF7970A_COLLISION_POSITION		0x0e
#define TRF7970A_RSSI_OSC_STATUS		0x0f
#define TRF7970A_SPECIAL_FUNCTION_REGISTER1	0x10
#define TRF7970A_SPECIAL_FUNCTION_REGISTER2	0x11
#define TRF7970A_RAM1				0x12
#define TRF7970A_RAM2				0x13
#define TRF7970A_ADJUTABLE_FIFO_IRQ_LEVELS	0x14
#define TRF7970A_NFC_LOW_FIELD_LEVEL		0x16
#define TRF7970A_NFCID1				0x17
#define TRF7970A_NFC_TARGET_LEVEL		0x18
#define TRF79070A_NFC_TARGET_PROTOCOL		0x19
#define TRF7970A_TEST_REGISTER1			0x1a
#define TRF7970A_TEST_REGISTER2			0x1b
#define TRF7970A_FIFO_STATUS			0x1c
#define TRF7970A_TX_LENGTH_BYTE1		0x1d
#define TRF7970A_TX_LENGTH_BYTE2		0x1e
#define TRF7970A_FIFO_IO_REGISTER		0x1f

/* 5 command bytes + FIFO size */
#define TRF7970A_MAX_PAYLOAD			(5 + 256)

/* Chip Status Control Register Bits */
#define TRF7970A_CHIP_STATUS_VRS5_3		BIT(0)
#define TRF7970A_CHIP_STATUS_REC_ON		BIT(1)
#define TRF7970A_CHIP_STATUS_AGC_ON		BIT(2)
#define TRF7970A_CHIP_STATUS_PM_ON		BIT(3)
#define TRF7970A_CHIP_STATUS_RF_PWR		BIT(4)
#define TRF7970A_CHIP_STATUS_RF_ON		BIT(5)
#define TRF7970A_CHIP_STATUS_DIRECT		BIT(6)
#define TRF7970A_CHIP_STATUS_STBY		BIT(7)

/* IRQ Status Register Bits */
#define TRF7970A_IRQ_STATUS_NORESP		BIT(0) /* ISO15693 only */
#define TRF7970A_IRQ_STATUS_COL			BIT(1)
#define TRF7970A_IRQ_STATUS_FRAMING_EOF_ERROR	BIT(2)
#define TRF7970A_IRQ_STATUS_PARITY_ERROR	BIT(3)
#define TRF7970A_IRQ_STATUS_CRC_ERROR		BIT(4)
#define TRF7970A_IRQ_STATUS_FIFO		BIT(5)
#define TRF7970A_IRQ_STATUS_SRX			BIT(6)
#define TRF7970A_IRQ_STATUS_TX			BIT(7)

/* Modulator and SYS_CLK Control Register Bits */
#define TRF7970A_MODULATOR_DEPTH(n)		((n) & 0x7)

#define TRF7970A_MODULATOR_DEPTH_ASK10		(TRF7970A_MODULATOR_DEPTH(0))
#define TRF7970A_MODULATOR_DEPTH_OOK		(TRF7970A_MODULATOR_DEPTH(1))
#define TRF7970A_MODULATOR_DEPTH_ASK7		(TRF7970A_MODULATOR_DEPTH(2))
#define TRF7970A_MODULATOR_DEPTH_ASK8_5		(TRF7970A_MODULATOR_DEPTH(3))
#define TRF7970A_MODULATOR_DEPTH_ASK13		(TRF7970A_MODULATOR_DEPTH(4))
#define TRF7970A_MODULATOR_DEPTH_ASK16		(TRF7970A_MODULATOR_DEPTH(5))
#define TRF7970A_MODULATOR_DEPTH_ASK22		(TRF7970A_MODULATOR_DEPTH(6))
#define TRF7970A_MODULATOR_DEPTH_ASK30		(TRF7970A_MODULATOR_DEPTH(7))

#define TRF7970A_MODULATOR_EN_ANA		BIT(3)
#define TRF7970A_MODULATOR_CLK(n)		(((n) & 0x3) << 4)

#define TRF7970A_MODULATOR_CLK_DISABLED		(TRF7970A_MODULATOR_CLK(0))
#define TRF7970A_MODULATOR_CLK_3_6		(TRF7970A_MODULATOR_CLK(1))
#define TRF7970A_MODULATOR_CLK_6_13		(TRF7970A_MODULATOR_CLK(2))
#define TRF7970A_MODULATOR_CLK_13_27		(TRF7970A_MODULATOR_CLK(3))

#define TRF7970A_MODULATOR_EN_OOK		BIT(6)
#define TRF7970A_MODULATOR_27MHZ		BIT(7)

/* ISO Control Register Bits */
#define TRF7970A_ISO_CTRL_NFC_BITRATE(n)	((n) & 0x3)
#define TRF7970A_ISO_CTRL_NFC_BITRATE_MASK	(TRF7970A_ISO_CTRL_NFC_BITRATE(3))
#define TRF7970A_ISO_CTRL_NFC_BITRATE_106KBPS	(TRF7970A_ISO_CTRL_NFC_BITRATE(1))
#define TRF7970A_ISO_CTRL_NFC_BITRATE_212KBPS	(TRF7970A_ISO_CTRL_NFC_BITRATE(2))
#define TRF7970A_ISO_CTRL_NFC_BITRATE_424KBPS	(TRF7970A_ISO_CTRL_NFC_BITRATE(3))

#define TRF7970A_ISO_CTRL_ACTIVE		BIT(3)
#define TRF7970A_ISO_CTRL_INITATOR		BIT(4)
#define TRF7970A_ISO_CTRL_NFC_MODE		BIT(5)
#define TRF7970A_ISO_CTRL_DIRECT_MODE		BIT(6)
#define TRF7970A_ISO_CTRL_RX_CRC_N		BIT(7)	/* true = No CRC */

#define TRF7970A_ISO_CTRL_NFC_106KBPS		1
#define TRF7970A_ISO_CTRL_NFC_212KBPS		2
#define TRF7970A_ISO_CTRL_NFC_424KBPS		3

#define TRF7970A_ISO_CTRL_NFC_106		(TRF7970A_ISO_CTRL_NFC_MODE | \
						TRF7970A_ISO_CTRL_NFC_106KBPS)
#define TRF7970A_ISO_CTRL_NFC_212		(TRF7970A_ISO_CTRL_NFC_MODE | \
						TRF7970A_ISO_CTRL_NFC_212KBPS)
#define TRF7970A_ISO_CTRL_NFC_424		(TRF7970A_ISO_CTRL_NFC_MODE | \
						TRF7970A_ISO_CTRL_NFC_424KBPS)
#define TRF7970A_ISO_CTRL_4A_106		(0x08)
#define TRF7970A_ISO_CTRL_4A_212		(0x09)
#define TRF7970A_ISO_CTRL_4A_424		(0x0a)
#define TRF7970A_ISO_CTRL_4A_848		(0x0b)
#define TRF7970A_ISO_CTRL_4B_106		(0x0c)
#define TRF7970A_ISO_CTRL_4B_212		(0x0d)
#define TRF7970A_ISO_CTRL_4B_424		(0x0e)
#define TRF7970A_ISO_CTRL_4B_848		(0x0f)
#define TRF7970A_ISO_CTRL_FELICA_212		(0x1a)
#define TRF7970A_ISO_CTRL_FELICA_424		(0x1b)

/* Regulator Control Register Bits */
#define TRF7970A_REG_IO_CTRL_VOLT(n)		((n) & 0x7)
#define TRF7970A_REG_IO_CTRL_VOLT_MASK		(TRF7970A_REG_IO_CTRL_VOLT(3))
#define TRF7970A_REG_IO_CTRL_VOLT_400mV		(TRF7970A_REG_IO_CTRL_VOLT(0))
#define TRF7970A_REG_IO_CTRL_VOLT_350mV		(TRF7970A_REG_IO_CTRL_VOLT(2))
#define TRF7970A_REG_IO_CTRL_VOLT_250mV		(TRF7970A_REG_IO_CTRL_VOLT(3))

#define TRF7970A_REG_IO_CTRL_VOLT_AUTO		BIT(7)

struct trf7970a {
	struct device *dev;
	struct spi_device *spi;
	struct nfc_digital_dev *nfc;
	struct mutex lock;

	struct sk_buff_head response_queue;
	struct regmap *regmap;

	unsigned long flags;
#define TRF7970A_RADIO_ON			BIT(0)

	struct sk_buff *resp;

	unsigned long quirks;

/* WORKAROUND: TRF7970A has a bug when reading IRQ Status register:
 *
 * When reading IRQ Status register on TRF7970a, we *must* issue a read
 * continuous command for IRQ Status and Collision Position registers,
 * because single reads will fail.
 */
#define TRF7970A_IRQ_STATUS_READ_ERRATA		BIT(0)

	u32 cur_technology;
	u32 framing;
	u32 tx_cmd;
	u32 enable_gpio2;
	u32 enable_gpio;
	u32 alloc_skb;
	struct sk_buff *skb;

	u32 baud_rate;

	struct timer_list res_timer;

	struct nfc_digital_dev *ndev;
	nfc_digital_cmd_complete_t cb;
	void *arg;
};

enum trf7970a_response_type {
	TRF7970A_RESPONSE_UNKNOWN = 0,
	TRF7970A_RESPONSE_N,
	TRF7970A_RESPONSE_0,
};

static int trf7970a_read(struct trf7970a *trf, u8 reg,
		u8 *val)
{
	u8 addr = reg | TRF7970A_CMD_RW;
	int ret;

	ret = spi_write_then_read(trf->spi, &addr, 1, val, 1);

#if 1 /* XXX */
printk("a_read(0x%02x): 0x%02x\n", addr, *val);
#endif

	return ret;
}

static void dump_regs(struct trf7970a *trf)
{
	int rc;
	u8 i, v;

	for (i = 0; i < 0xc; i++) {
		rc = trf7970a_read(trf, i, &v);
		if (rc) {
			printk("XXX read failed: %d\n", rc);
			break;
		} else {
			printk("-- 0x%02x: 0x%02x\n", i, v);
		}
	}
}

static int trf7970a_read_cont(struct trf7970a *trf, u8 reg,
		u8 *buf, size_t len)
{
	u8 addr = reg | TRF7970A_CMD_RW | TRF7970A_CMD_CONTINUOUS;
	int ret;

	ret = spi_write_then_read(trf->spi, &addr, 1, buf, len);

#if 1 /* XXX */
{
	int i;

	printk("a_read_cont(0x%02x):", addr);
	for (i = 0; i< len; i++)
		printk(" 0x%02x", buf[i]);
	printk("\n");
}
#endif

	return ret; 
}

static int trf7970a_write(struct trf7970a *trf, u8 reg,
		u8 val)
{
	u8 buf[2] = { reg, val };

#if 1 /* XXX */
printk("a_write(0x%02x, 0x%02x)\n", reg, val);
#endif

	return spi_write(trf->spi, buf, 2);
}

/* WORKAROUND for silicon bug. We must use a read continuous for IRQ Status and
 * collision position registers.
 */
static int trf7970a_read_irqstatus(struct trf7970a *trf, u8 *status)
{
	int ret;
	u8 buf[2];
	u8 addr;

	addr = TRF7970A_IRQ_STATUS | TRF7970A_CMD_RW;

	if (trf->quirks & TRF7970A_IRQ_STATUS_READ_ERRATA) {
		addr |= TRF7970A_CMD_CONTINUOUS;
		ret = spi_write_then_read(trf->spi, &addr, 1, buf, 2);
	} else {
		ret = spi_write_then_read(trf->spi, &addr, 1, buf, 1);
	}

	if (ret) {
printk("irq ERR: 0x%d\n", ret);
		return ret;
	}

	*status = buf[0];

printk("irq: 0x%x\n", buf[0]);

	return 0;
}

/* ------------------------------------------------------------- */

static int trf7970a_cmd(struct trf7970a *trf, u8 opcode)
{
	u8 cmd = TRF7970A_CMD_CTRL;

	/* lower five bits of the command, contain opcode */
	cmd |= TRF7970A_CMD_OPCODE(opcode);

#if 1 /* XXX */
printk("a_cmd(0x%02x)\n", cmd);
#endif

	return spi_write(trf->spi, &cmd, 1);
}

static inline int trf7970a_reset(struct trf7970a *trf)
{
	return trf7970a_cmd(trf, TRF7970A_CMD_RESET);
}

static inline int trf7970a_idle(struct trf7970a *trf)
{
	return trf7970a_cmd(trf, TRF7970A_CMD_IDLE);
}

static inline int trf7970a_soft_init(struct trf7970a *trf)
{
	return trf7970a_cmd(trf, TRF7970A_CMD_SOFT_INIT);
}

static inline int trf7970a_rf_collision(struct trf7970a *trf)
{
	return trf7970a_cmd(trf, TRF7970A_CMD_RF_COLLISION);
}

static inline int trf7970a_rf_collision_response(struct trf7970a *trf,
		unsigned resp)
{
	switch (resp) {
	case TRF7970A_RESPONSE_N:
		return trf7970a_cmd(trf, TRF7970A_CMD_RF_COLLISION_RESPONSE_N);
	case TRF7970A_RESPONSE_0:
		return trf7970a_cmd(trf, TRF7970A_CMD_RF_COLLISION_RESPONSE_0);
	default:
		return -EINVAL;
	}
}

static int trf7970a_transmit(struct trf7970a *trf, struct sk_buff *skb,
		u16 timeout)
{
	int ret;
	char buf[TRF7970A_MAX_PAYLOAD];
	unsigned int len = 2;

	buf[0] = TRF7970A_CMD_CTRL | TRF7970A_CMD_OPCODE(TRF7970A_CMD_RESET);
	buf[1] = TRF7970A_CMD_CTRL | TRF7970A_CMD_OPCODE(trf->tx_cmd);

	if (!skb->len)
		goto skip;

len = skb->len + 5;

	buf[2] = TRF7970A_CMD_CONTINUOUS | TRF7970A_TX_LENGTH_BYTE1;

	if (trf->framing == NFC_DIGITAL_FRAMING_NFCA_SHORT) {
		buf[3] = 0x00;
		buf[4] = 0x0f;
	} else {
		buf[3] = ((skb->len & 0xf0) >> 4);
		buf[4] = ((skb->len & 0x0f) << 4);
	}

	if (trf->framing == NFC_DIGITAL_FRAMING_NFCA_T2T) {
		if (skb->data[0] == 0x30) { /* Type 2 READ */
			ret = trf7970a_write(trf, TRF7970A_ISO_CTRL, 0x88);
			if (ret)
				dev_err(trf->dev,
					"ISO_CTRL reg write failed\n");

			ret = trf7970a_write(trf, TRF7970A_CHIP_STATUS_CTRL,
					0x21);
			if (ret)
				dev_err(trf->dev,
					"STAT CTRL reg write failed\n");

			ret = trf7970a_write(trf,
					TRF7970A_SPECIAL_FUNCTION_REGISTER1,
					0x20);
			if (ret)
				dev_err(trf->dev,
					"SPEC FUNC reg can't be written\n");
#if 0 /* MIFARE WRITE */
		} else if (skb->data[0] == 0xa2) { /* Type 2 WRITE */
			/*
			 * Need to drop into MODE 1 and feed cmd data
			 * via SPI and not thru FIFO (I think).
			 */
			ret = trf7970a_write(trf, TRF7970A_ISO_CTRL, 0x08);
			if (ret)
				dev_err(trf->dev,
					"ISO_CTRL reg write failed\n");

			ret = trf7970a_write(trf, TRF7970A_CHIP_STATUS_CTRL,
					0x21);
			if (ret)
				dev_err(trf->dev,
					"STAT CTRL reg write failed\n");

			ret = trf7970a_cmd(trf, TRF7970A_CMD_BLOCK_RX);
			if (ret)
				dev_err(trf->dev,
						"CMD_BLOCK_RX cmd failed\n");

			ret = trf7970a_write(trf,
					TRF7970A_SPECIAL_FUNCTION_REGISTER1,
					0x08);
			if (ret)
				dev_err(trf->dev,
					"SPEC FUNC reg can't be written\n");
#endif
		} else {
			printk("**************** Don't know what to do here\n");
		}
	}

	skb_copy_bits(skb, 0, buf + 5, skb->len);

skip:

#if 1 /* XXX */
	{
	int i;

	printk("-->");
	for (i = 0; i < len; i++)
		printk(" 0x%02x", buf[i]);
	printk("\n");
	}
#endif

	mod_timer(&trf->res_timer, jiffies + msecs_to_jiffies(timeout));

	return spi_write(trf->spi, buf, len);
}

static inline int trf7970a_eof(struct trf7970a *trf)
{
	return trf7970a_cmd(trf, TRF7970A_CMD_EOF);
}

static inline int trf7970a_close_slot(struct trf7970a *trf)
{
	return trf7970a_cmd(trf, TRF7970A_CMD_CLOSE_SLOT);
}

static inline int trf7970a_block_rx(struct trf7970a *trf)
{
	return trf7970a_cmd(trf, TRF7970A_CMD_BLOCK_RX);
}

static inline int trf7970a_enable_rx(struct trf7970a *trf)
{
	return trf7970a_cmd(trf, TRF7970A_CMD_ENABLE_RX);
}

static inline int trf7970a_cmd_test_ext_rf(struct trf7970a *trf)
{
	return trf7970a_cmd(trf, TRF7970A_CMD_TEST_EXT_RF);
}

static inline int trf7970a_cmd_test_int_rf(struct trf7970a *trf)
{
	return trf7970a_cmd(trf, TRF7970A_CMD_TEST_INT_RF);
}

static inline int trf7970a_rx_gain_adjust(struct trf7970a *trf)
{
	return trf7970a_cmd(trf, TRF7970A_CMD_RX_GAIN_ADJUST);
}

/* ----------------------------------------------------------------- */

static int __trf7970a_switch_rf_on(struct trf7970a *trf)
{
	int ret;
	u8 reg;

	/* switch power gpio on first */
	if (gpio_is_valid(trf->enable_gpio))
		gpio_set_value(trf->enable_gpio, 1);

	/* WARNING: if we pull both enable lines high too quickly, device will
	 * start misbehaving. We better add some delay here just to make sure
	 */
	usleep_range(5000, 25000);

	if (gpio_is_valid(trf->enable_gpio2))
		gpio_set_value(trf->enable_gpio2, 1);

	/* let device power up */
	usleep_range(5000, 250000);

	ret = trf7970a_soft_init(trf);
	if (ret) {
		dev_err(trf->dev, "Failed to initialize chip --> %d\n", ret);
		return ret;
	}

	ret = trf7970a_idle(trf);
	if (ret) {
		dev_err(trf->dev, "Couldn't idle device\n");
		return ret;
	}

	/* asuming SYS_CLK of 27.12MHz for now */
	reg = TRF7970A_MODULATOR_DEPTH_OOK;

	ret = trf7970a_write(trf, TRF7970A_MODULATOR_SYS_CLK_CTRL, reg);
	if (ret) {
		dev_err(trf->dev, "Failed to write to Modulator register\n");
		return ret;
	}

	reg = TRF7970A_CHIP_STATUS_RF_ON
		| TRF7970A_CHIP_STATUS_VRS5_3;

	ret = trf7970a_write(trf, TRF7970A_CHIP_STATUS_CTRL, reg);
	if (ret) {
		dev_err(trf->dev, "Failed to write to Chip Status register\n");
		return ret;
	}

	set_bit(TRF7970A_RADIO_ON, &trf->flags);

	return 0;
}

static int __trf7970a_switch_rf_off(struct trf7970a *trf)
{
	/* switch off power GPIO, this brings the whole device down */
	if (gpio_is_valid(trf->enable_gpio))
		gpio_set_value(trf->enable_gpio, 0);

	if (gpio_is_valid(trf->enable_gpio2))
		gpio_set_value(trf->enable_gpio2, 0);

	clear_bit(TRF7970A_RADIO_ON, &trf->flags);

	return 0;
}

static int __trf7970a_config_rf_tech(struct trf7970a *trf, int tech)
{
	int ret = -EINVAL;

	switch (tech) {
	case NFC_DIGITAL_RF_TECH_106A:
		ret = 0;
		break;
#if 0 /* XXX */
	case NFC_DIGITAL_RF_TECH_212F:
		break;
	case NFC_DIGITAL_RF_TECH_424F:
		break;
#endif
	case NFC_DIGITAL_RF_TECH_ISO15693:
		ret = 0;
		break;
	default:
		dev_err(trf->dev, "Unsupported RF Technology: %d\n", tech);
	}

	if (!ret)
		trf->cur_technology = tech;

	return ret;
}

static int __trf7970a_config_framing(struct trf7970a *trf, int framing)
{
	int reg = 0;
	int ret = 0;

	/*
	 * Due to errata, we can't use NFC mode on the trf7970a.
	 * Instead, we have to use RFID mode and hardcode the
	 * ISO CTRL register to 0x08 until the target is being
	 * selected (i.e., NFC_DIGITAL_FRAMING_NFCA_STANDARD_WITH_CRC_A).
	 */
	switch (framing) {
	case NFC_DIGITAL_FRAMING_NFCA_SHORT:
		trf->tx_cmd = TRF7970A_CMD_TRANSMIT_NO_CRC;
		reg = 0x88;
		break;
	case NFC_DIGITAL_FRAMING_NFCA_STANDARD:
		trf->tx_cmd = TRF7970A_CMD_TRANSMIT_NO_CRC;
		reg = 0x88;
		break;
	case NFC_DIGITAL_FRAMING_NFCA_STANDARD_WITH_CRC_A:
		trf->tx_cmd = TRF7970A_CMD_TRANSMIT;
		reg = 0x08;
		break;
	case NFC_DIGITAL_FRAMING_NFCA_T2T:
		trf->tx_cmd = TRF7970A_CMD_TRANSMIT;
		break;
	case NFC_DIGITAL_FRAMING_NFCA_NFC_DEP:
		/* trf->tx_cmd = TRF7970A_CMD_TRANSMIT; */
		/* reg = 0x08; */
		break;
	case NFC_DIGITAL_FRAMING_ISO15693_INVENTORY:
#if 0 /* XXX */
		reg = 0x02;
#else
		ret = trf7970a_write(trf, TRF7970A_ISO_CTRL, 0x02);
		if (ret)
			dev_err(trf->dev, "ISO Control reg write failed\n");
#endif

		ret = trf7970a_write(trf, TRF7970A_TX_TIMER_SETTING_H_BYTE,
				0xc1);
		if (ret)
			dev_err(trf->dev, "TX Timer high byte write failed\n");

		ret = trf7970a_write(trf, TRF7970A_TX_TIMER_SETTING_L_BYTE,
				0xbb);
		if (ret)
			dev_err(trf->dev, "TX Timer low byte write failed\n");

		ret = trf7970a_write(trf, TRF7970A_TX_PULSE_LENGTH_CTRL, 0x00);
		if (ret)
			dev_err(trf->dev, "Pulse length ctrl write failed\n");

		ret = trf7970a_write(trf, TRF7970A_RX_NO_RESPONSE_WAIT, 0x30);
		if (ret)
			dev_err(trf->dev, "RX no response wait write failed\n");

		ret = trf7970a_write(trf, TRF7970A_RX_WAIT_TIME, 0x1f);
		if (ret)
			dev_err(trf->dev, "RX wait time write failed\n");

		ret = trf7970a_write(trf, TRF7970A_MODULATOR_SYS_CLK_CTRL,
				0x21);
		if (ret)
			dev_err(trf->dev, "Modulator sys clk write failed\n");

		ret = trf7970a_write(trf, TRF7970A_RX_SPECIAL_SETTINGS, 0x40);
		if (ret)
			dev_err(trf->dev, "RX special write failed\n");

		trf->tx_cmd = TRF7970A_CMD_TRANSMIT;

		break;
	case NFC_DIGITAL_FRAMING_ISO15693_EOF:
		trf->tx_cmd = TRF7970A_CMD_EOF;
		break;
	case NFC_DIGITAL_FRAMING_ISO15693_TVT:
		trf->tx_cmd = TRF7970A_CMD_TRANSMIT;
		break;
	default:
		dev_dbg(trf->dev, "Unsupported Framing Option\n");
		return -EINVAL;
	}

	if (!ret) {
		trf->framing = framing;

		if (reg) {
			ret = trf7970a_write(trf, TRF7970A_ISO_CTRL, reg);
			if (ret)
				dev_err(trf->dev,
					"ISO Control reg write failed\n");
		}
	}

#if 0 /* XXX */
{
	static int first_time = 1;
	int i, rc;
	u8 v;

	if (first_time) {
		for (i = 0; i < 0xc; i++) {
			rc = trf7970a_read(trf, i, &v);
			if (rc) {
				printk("XXX read failed: %d\n", rc);
				break;
			} else {
				printk("-- 0x%02x: 0x%02x\n", i, v);
			}
		}
	}
}
#endif

	return ret;
}

/* ----------------------------------------------------------------- */

static int trf7970a_switch_rf(struct nfc_digital_dev *ndev, bool on)
{
	struct trf7970a *trf = nfc_digital_get_drvdata(ndev);
	int ret = 0;

	dev_vdbg(trf->dev, "%s\n", __func__);

	if (on)
		ret = __trf7970a_switch_rf_on(trf);
	else
		ret = __trf7970a_switch_rf_off(trf);

	if (on)
		dump_regs(trf);

	return ret;
}

static int trf7970a_in_configure_hw(struct nfc_digital_dev *ndev,
		int type, int param)
{
	struct trf7970a *trf = nfc_digital_get_drvdata(ndev);
	int ret = 0;

	if (!test_bit(TRF7970A_RADIO_ON, &trf->flags)) {
		ret = __trf7970a_switch_rf_on(trf);
		if (ret)
			return ret;
	}

	switch (type) {
	case NFC_DIGITAL_CONFIG_RF_TECH:
		ret = __trf7970a_config_rf_tech(trf, param);
		break;
	case NFC_DIGITAL_CONFIG_FRAMING:
		ret = __trf7970a_config_framing(trf, param);
		break;
	default:
		dev_err(trf->dev, "Unknown configuration type\n");
		ret = -EINVAL;
	}

	return ret;
}

static int trf7970a_in_send_cmd(struct nfc_digital_dev *ndev,
		struct sk_buff *skb, u16 timeout,
		nfc_digital_cmd_complete_t cb, void *arg)
{
	struct trf7970a *trf = nfc_digital_get_drvdata(ndev);
	struct sk_buff *resp, *eof_skb;
	int ret = 0;

	if (skb->len > 256) {
		resp = ERR_PTR(-EINVAL);
		ret = -EINVAL;
		goto err;
	}

	trf->cb = cb;
	trf->arg = arg;
	trf->ndev = ndev;

	ret = trf7970a_transmit(trf, skb, timeout);
	if (ret) {
		dev_err(trf->dev, "Failed to send TX command\n");
		return ret;
	}

#if 1 /* XXX */
	/*
	 * TI Tags must have option bit set on writes whch requires an EOF
	 * to be sent to prompt the tag to send a response.
	 */
	if ((trf->framing == NFC_DIGITAL_FRAMING_ISO15693_TVT) &&
			(skb->data[1] == 0x21) &&/* Type V WRITE_SINGLE_BLOCK */
			(skb->data[0] & 0x40)) { /* Option bit set */

		usleep_range(5000, 250000);

		eof_skb = alloc_skb(1, GFP_KERNEL);

		trf->tx_cmd = TRF7970A_CMD_EOF;

		ret = trf7970a_transmit(trf, eof_skb, timeout);

		trf->tx_cmd = TRF7970A_CMD_TRANSMIT;

		if (ret) {
			dev_err(trf->dev, "Failed to send EOF command\n");
			return ret;
		}
	}
#endif

err:
	return ret < 0 ? ret : 0;
}

static int trf7970a_tg_configure_hw(struct nfc_digital_dev *ndev,
		int type, int param)
{
	struct trf7970a *trf = nfc_digital_get_drvdata(ndev);
	dev_info(trf->dev, "%s\n", __func__);
	return 0;
}

static int trf7970a_tg_send_cmd(struct nfc_digital_dev *ndev,
		struct sk_buff *skb, u16 timeout,
		nfc_digital_cmd_complete_t cb, void *arg)
{
	struct trf7970a *trf = nfc_digital_get_drvdata(ndev);
	dev_info(trf->dev, "%s\n", __func__);
	return 0;
}

static int trf7970a_tg_listen(struct nfc_digital_dev *ndev,
		u16 timeout, nfc_digital_cmd_complete_t cb, void *arg)
{
	struct trf7970a *trf = nfc_digital_get_drvdata(ndev);
	dev_info(trf->dev, "%s\n", __func__);
	return 0;
}

static int trf7970a_tg_listen_mdaa(struct nfc_digital_dev *ndev,
		struct digital_tg_mdaa_params *mdaa_params,
		u16 timeout, nfc_digital_cmd_complete_t cb, void *arg)
{
	struct trf7970a *trf = nfc_digital_get_drvdata(ndev);
	dev_info(trf->dev, "%s\n", __func__);
	return 0;
}

static void trf7970a_abort_cmd(struct nfc_digital_dev *ndev)
{
	struct trf7970a *trf = nfc_digital_get_drvdata(ndev);
	dev_info(trf->dev, "%s\n", __func__);
}

static struct nfc_digital_ops trf7970a_nfc_ops = {
	.switch_rf		= trf7970a_switch_rf,

	.in_configure_hw	= trf7970a_in_configure_hw,
	.in_send_cmd		= trf7970a_in_send_cmd,

	.tg_configure_hw	= trf7970a_tg_configure_hw,
	.tg_send_cmd		= trf7970a_tg_send_cmd,
	.tg_listen		= trf7970a_tg_listen,
	.tg_listen_mdaa		= trf7970a_tg_listen_mdaa,

	.abort_cmd		= trf7970a_abort_cmd,
};

/* ----------------------------------------------------------------- */

static void trf7970a_res_timer_handler(unsigned long data)
{
	struct trf7970a *trf = (struct trf7970a *)data;

#if 0 /* XXX */
printk("------------- TIMEOUT ----------------\n"); /* XXX */
#endif

	trf->cb(trf->ndev, trf->arg, ERR_PTR(-ETIMEDOUT));
}

static void trf7970a_handle_collision(struct trf7970a *trf)
{

	if ((trf->cur_technology == NFC_DIGITAL_RF_TECH_ISO15693) &&
	    (trf->framing == NFC_DIGITAL_FRAMING_ISO15693_INVENTORY)) {
		trf->cb(trf->ndev, trf->arg, ERR_PTR(-ENODATA));
	} else {
		dev_info(trf->dev, "Collision Error\n");
		trf7970a_block_rx(trf);
		trf7970a_reset(trf);
	}
}

static int trf7970a_rx_irq(struct trf7970a *trf, u8 status)
{
	struct sk_buff *skb;
	int ret;
	char *dp;
	u8 fifo;

	dev_dbg(trf->dev, "RX Finished\n");

	del_timer(&trf->res_timer);

	if (status & 0x02) {
		if (!trf->alloc_skb) {
			kfree_skb(trf->skb);
			trf->alloc_skb = 1;
		}

		trf->resp = ERR_PTR(-EIO);
		return 0;
	}

	ret = trf7970a_read(trf, TRF7970A_FIFO_STATUS, &fifo);
	if (ret)
		return -EINVAL;

	if (fifo & 0x80) { /* XXX */
		printk("************** FIFO Overflow\n");
		fifo = 0;
	}

	if (trf->alloc_skb) {
#if 0 /* XXX */
		skb = alloc_skb(fifo + NFC_HEADER_SIZE, GFP_KERNEL);
#else
		skb = alloc_skb(500 + NFC_HEADER_SIZE, GFP_KERNEL);
#endif
		if (!skb)
			return -ENOMEM;

		trf->skb = skb;

		skb_reserve(skb, NFC_HEADER_SIZE);

		trf->alloc_skb = 0;
	} else {
		skb = trf->skb;
	}

	if (!(fifo & 0x7f))
		goto out;

	dp = skb_put(skb, fifo);

	if (fifo == 1) {
		ret = trf7970a_read(trf, TRF7970A_FIFO_IO_REGISTER, dp);
	} else {
		ret = trf7970a_read_cont(trf, TRF7970A_FIFO_IO_REGISTER,
				dp, fifo);
	}

#if 1 /* XXX */
{
	int i;

	printk("fifo data:");
	for (i = 0; i < fifo; i++)
		printk(" 0x%02x", skb->data[i]);
	printk("\n");
}
#endif

out:
	if (((status & 0xc0) == 0x40) || (status == 0xe0)) {
		trf->resp = skb;
		trf->alloc_skb = 1;
		return 0;
	}

	return 1;
}

static irqreturn_t trf7970a_irq(int irq, void *_trf)
{
	struct trf7970a *trf = _trf;
	unsigned int done = false;
	int ret;
	u8 status;

	ret = trf7970a_read_irqstatus(trf, &status);
	if (ret) {
		dev_err(trf->dev, "Failed to read IRQ STATUS register\n");
		return IRQ_NONE;
	}

	usleep_range(1, 5); /* XXX Driver doesn't work without this delay */

	if (status & TRF7970A_IRQ_STATUS_FIFO)
		dev_dbg(trf->dev, "FIFO Status\n");

	if (status & TRF7970A_IRQ_STATUS_COL) {
		trf7970a_handle_collision(trf);
		return IRQ_HANDLED;
	}

	/* RX status only means we have received all data
	 *
	 * Note that ideally we would test for equality with
	 * 'status == TRF7970A_IRQ_STATUS_SRX', but IRQ latencies
	 * are quite high as of now, so we need this change meanwhile.
	 */
	if (status & TRF7970A_IRQ_STATUS_SRX) {
		ret = trf7970a_rx_irq(trf, status);
		if (ret < 0)
			return IRQ_HANDLED;

		if (ret == 0)
			done = true;
	}

	/* TX status only means transmission is REALLY completed
	 *
	 * Note that ideally we would test for equality with
	 * 'status == TRF7970A_IRQ_STATUS_TX', but IRQ latencies
	 * are quite high as of now, so we need this change meanwhile.
	 */
	if (status & TRF7970A_IRQ_STATUS_TX) {
		dev_dbg(trf->dev, "TX Finished\n");
#if 0 /* XXX */
		ret = trf7970a_reset(trf);
		WARN_ON(ret);
#endif
	}

	if (status & TRF7970A_IRQ_STATUS_FRAMING_EOF_ERROR)
		dev_err(trf->dev, "Framing Error\n");

	if (status & TRF7970A_IRQ_STATUS_PARITY_ERROR)
		dev_err(trf->dev, "Parity Error\n");

	if (status & TRF7970A_IRQ_STATUS_CRC_ERROR)
		dev_err(trf->dev, "CRC Error\n");

	if (done)
		trf->cb(trf->ndev, trf->arg, trf->resp);

	return IRQ_HANDLED;
}

static int trf7970a_probe(struct spi_device *spi)
{
	struct device_node *np = spi->dev.of_node;
	const struct spi_device_id *id = spi_get_device_id(spi);
	struct trf7970a *trf;
	int gpio;
	int ret;

	trf = devm_kzalloc(&spi->dev, sizeof(*trf), GFP_KERNEL);
	if (!trf)
		return -ENOMEM;

	trf->dev = &spi->dev;
	trf->spi = spi;
	trf->quirks = id->driver_data;

	/* invalid by default */
	trf->enable_gpio = -EINVAL;
	trf->enable_gpio = -EINVAL;

	trf->alloc_skb = 1;

	spi->bits_per_word = 8;
	spi->mode = SPI_MODE_1;

	mutex_init(&trf->lock);
	skb_queue_head_init(&trf->response_queue);

	if (!np) {
		dev_err(&spi->dev, "Missing OF handle\n");
		return -EINVAL;
	}

	trf->enable_gpio = of_get_named_gpio(np, "ti,enable-gpios", 0);
	trf->enable_gpio2 = of_get_named_gpio(np, "ti,enable-gpios", 1);

	gpio = of_get_named_gpio(np, "ti,cs0-sel", 0);

	ret = gpio_request(gpio, "cs0 sel");
	if (ret) {
		dev_err(&spi->dev, "Failed to request CS0 select gpio\n");
		return ret;
	}
	gpio_direction_output(gpio, 0);

	gpio_set_value(gpio, 1);

	ret = gpio_request(trf->enable_gpio, "enable gpio");
	if (ret) {
		dev_err(&spi->dev, "Failed to request Enable GPIO\n");
		return ret;
	}
	gpio_direction_output(trf->enable_gpio, 0);

	ret = gpio_request(trf->enable_gpio2, "enable gpio2");
	if (ret) {
		dev_err(&spi->dev, "Failed to request Enable GPIO 2\n");
		return ret;
	}
	gpio_direction_output(trf->enable_gpio2, 0);

	/* make sure we're starting with device off */
	ret = __trf7970a_switch_rf_off(trf);
	if (ret) {
		dev_err(&spi->dev, "Failed to switch RF off\n");
		return ret;
	}

	ret = devm_request_threaded_irq(&spi->dev, spi->irq, NULL,
			trf7970a_irq, IRQF_TRIGGER_RISING | IRQF_ONESHOT,
			"trf7970a", trf);
	if (ret) {
		dev_err(&spi->dev, "Failed to request IRQ#%d\n", spi->irq);
		return ret;
	}

	trf->nfc = nfc_digital_allocate_device(&trf7970a_nfc_ops,
			TRF7970A_PROTOCOLS,
			NFC_DIGITAL_DRV_CAPS_IN_CRC |
				NFC_DIGITAL_DRV_CAPS_TG_CRC,
			0, 0);
	if (!trf->nfc) {
		dev_err(trf->dev, "Failed to allocate NFC Digital device\n");
		ret = -ENOMEM;
		goto err0;
	}

	nfc_digital_set_parent_dev(trf->nfc, &spi->dev);
	nfc_digital_set_drvdata(trf->nfc, trf);
	spi_set_drvdata(spi, trf);

	ret = nfc_digital_register_device(trf->nfc);
	if (ret) {
		dev_dbg(&spi->dev, "couldn't register NFC device\n");
		goto err1;
	}

	init_timer(&trf->res_timer);
	trf->res_timer.function = trf7970a_res_timer_handler;
	trf->res_timer.data = (unsigned long)trf;

	return 0;

err1:
	nfc_digital_free_device(trf->nfc);

err0:
	return ret;
}

static int trf7970a_remove(struct spi_device *spi)
{
	struct trf7970a *trf = spi_get_drvdata(spi);

	gpio_set_value(trf->enable_gpio, 0);
	gpio_set_value(trf->enable_gpio2, 0);
	nfc_digital_unregister_device(trf->nfc);
	nfc_digital_free_device(trf->nfc);

	return 0;
}

static const struct spi_device_id trf7970a_id_table[] = {
	{ "trf7970a", TRF7970A_IRQ_STATUS_READ_ERRATA },
	{  }
};
MODULE_DEVICE_TABLE(spi, trf7970a_id_table);

static struct spi_driver trf7970a_spi_driver = {
	.probe		= trf7970a_probe,
	.remove		= trf7970a_remove,
	.id_table	= trf7970a_id_table,
	.driver		= {
		.name	= "trf7970a",
		.owner	= THIS_MODULE,
	},
};

static int __init trf7970a_spi_init(void)
{
	return spi_register_driver(&trf7970a_spi_driver);
}

static void __exit trf7970a_spi_exit(void)
{
	spi_unregister_driver(&trf7970a_spi_driver);
}

module_init(trf7970a_spi_init);
module_exit(trf7970a_spi_exit);

MODULE_AUTHOR("Felipe Balbi <balbi@ti.com>");
MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("TI's TRF7970a RFID/NFC Transceiver Driver");
