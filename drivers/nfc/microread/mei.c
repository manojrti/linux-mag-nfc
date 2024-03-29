/*
 * HCI based Driver for Inside Secure microread NFC Chip
 *
 * Copyright (C) 2013  Intel Corporation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the
 * Free Software Foundation, Inc.,
 * 59 Temple Place - Suite 330, Boston, MA 02111-1307, USA.
 */

#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt

#include <linux/module.h>
#include <linux/mod_devicetable.h>
#include <linux/nfc.h>
#include <net/nfc/hci.h>
#include <net/nfc/llc.h>

#include "../mei_phy.h"
#include "microread.h"

#define MICROREAD_DRIVER_NAME "microread"

struct mei_nfc_hdr {
	u8 cmd;
	u8 status;
	u16 req_id;
	u32 reserved;
	u16 data_size;
} __attribute__((packed));

#define MEI_NFC_HEADER_SIZE 10
#define MEI_NFC_MAX_HCI_PAYLOAD 300
#define MEI_NFC_MAX_READ (MEI_NFC_HEADER_SIZE + MEI_NFC_MAX_HCI_PAYLOAD)

struct microread_mei_phy {
	struct mei_device *mei_device;
	struct nfc_hci_dev *hdev;

	int powered;

	int hard_fault;		/*
				 * < 0 if hardware error occured (e.g. i2c err)
				 * and prevents normal operation.
				 */
};

#define MEI_DUMP_SKB_IN(info, skb)					\
do {								\
	pr_debug("%s:\n", info);				\
	print_hex_dump(KERN_DEBUG, "mei in : ", DUMP_PREFIX_OFFSET,	\
		       16, 1, (skb)->data, (skb)->len, 0);	\
} while (0)

#define MEI_DUMP_SKB_OUT(info, skb)					\
do {								\
	pr_debug("%s:\n", info);				\
	print_hex_dump(KERN_DEBUG, "mei out: ", DUMP_PREFIX_OFFSET,	\
		       16, 1, (skb)->data, (skb)->len, 0);	\
} while (0)

static int microread_mei_enable(void *phy_id)
{
	struct microread_mei_phy *phy = phy_id;

	pr_info(DRIVER_DESC ": %s\n", __func__);

	phy->powered = 1;

	return 0;
}

static void microread_mei_disable(void *phy_id)
{
	struct microread_mei_phy *phy = phy_id;

	pr_info(DRIVER_DESC ": %s\n", __func__);

	phy->powered = 0;
}

/*
 * Writing a frame must not return the number of written bytes.
 * It must return either zero for success, or <0 for error.
 * In addition, it must not alter the skb
 */
static int microread_mei_write(void *phy_id, struct sk_buff *skb)
{
	struct microread_mei_phy *phy = phy_id;
	int r;

	MEI_DUMP_SKB_OUT("mei frame sent", skb);

	r = mei_cl_send(phy->device, skb->data, skb->len);
	if (r > 0)
		r = 0;

	return r;
}

static void microread_event_cb(struct mei_cl_device *device, u32 events,
			       void *context)
{
	struct microread_mei_phy *phy = context;

	if (phy->hard_fault != 0)
		return;

	if (events & BIT(MEI_CL_EVENT_RX)) {
		struct sk_buff *skb;
		int reply_size;

		skb = alloc_skb(MEI_NFC_MAX_READ, GFP_KERNEL);
		if (!skb)
			return;

		reply_size = mei_cl_recv(device, skb->data, MEI_NFC_MAX_READ);
		if (reply_size < MEI_NFC_HEADER_SIZE) {
			kfree(skb);
			return;
		}

		skb_put(skb, reply_size);
		skb_pull(skb, MEI_NFC_HEADER_SIZE);

		MEI_DUMP_SKB_IN("mei frame read", skb);

		nfc_hci_recv_frame(phy->hdev, skb);
	}
}

static struct nfc_phy_ops mei_phy_ops = {
	.write = microread_mei_write,
	.enable = microread_mei_enable,
	.disable = microread_mei_disable,
};

static int microread_mei_probe(struct mei_cl_device *device,
			       const struct mei_cl_device_id *id)
{
	struct nfc_mei_phy *phy;
	int r;

	pr_info("Probing NFC microread\n");

	phy = nfc_mei_phy_alloc(device);
	if (!phy) {
		pr_err("Cannot allocate memory for microread mei phy.\n");
		return -ENOMEM;
	}

	r = microread_probe(phy, &mei_phy_ops, LLC_NOP_NAME,
			    MEI_NFC_HEADER_SIZE, 0, MEI_NFC_MAX_HCI_PAYLOAD,
			    &phy->hdev);
	if (r < 0) {
		nfc_mei_phy_free(phy);

		return r;
	}

	return 0;
}

static int microread_mei_remove(struct mei_cl_device *device)
{
	struct nfc_mei_phy *phy = mei_cl_get_drvdata(device);

	microread_remove(phy->hdev);

	nfc_mei_phy_free(phy);

	return 0;
}

static struct mei_cl_device_id microread_mei_tbl[] = {
	{ MICROREAD_DRIVER_NAME },

	/* required last entry */
	{ }
};
MODULE_DEVICE_TABLE(mei, microread_mei_tbl);

static struct mei_cl_driver microread_driver = {
	.id_table = microread_mei_tbl,
	.name = MICROREAD_DRIVER_NAME,

	.probe = microread_mei_probe,
	.remove = microread_mei_remove,
};

static int microread_mei_init(void)
{
	int r;

	pr_debug(DRIVER_DESC ": %s\n", __func__);

	r = mei_cl_driver_register(&microread_driver);
	if (r) {
		pr_err(MICROREAD_DRIVER_NAME ": driver registration failed\n");
		return r;
	}

	return 0;
}

static void microread_mei_exit(void)
{
	mei_cl_driver_unregister(&microread_driver);
}

module_init(microread_mei_init);
module_exit(microread_mei_exit);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION(DRIVER_DESC);
