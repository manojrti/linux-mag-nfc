/*
 * NFC Digital Protocol stack
 * Copyright (c) 2013, Intel Corporation.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 */

#include "digital.h"

#define DIGITAL_CMD_SENS_REQ    0x26
#define DIGITAL_CMD_ALL_REQ     0x52
#define DIGITAL_CMD_SEL_REQ_CL1 0x93
#define DIGITAL_CMD_SEL_REQ_CL2 0x95
#define DIGITAL_CMD_SEL_REQ_CL3 0x97

#define DIGITAL_SDD_REQ_SEL_PAR 0x20

#define DIGITAL_SDD_RES_CT  0x88
#define DIGITAL_SDD_RES_LEN 5

#define DIGITAL_SEL_RES_NFCID1_COMPLETE(sel_res) (!((sel_res) & 0x04))
#define DIGITAL_SEL_RES_IS_T2T(sel_res) (!((sel_res) & 0x60))

#define DIGITAL_SENS_RES_IS_T1T(sens_res) (((sens_res) & 0x000C) == 0x000C)
#define DIGITAL_SENS_RES_IS_VALID(sens_res) \
	((!((sens_res) & 0x1F00) && (((sens_res) & 0x000C) == 0x000C)) || \
	(((sens_res) & 0x1F00) && ((sens_res) & 0x000C) != 0x000C))

#define DIGITAL_MIFARE_READ_RES_LEN 16
#define DIGITAL_MIFARE_ACK_RES	0x0A

struct digital_sdd_res {
	u8 nfcid1[4];
	u8 bcc;
} __packed;

struct digital_sel_req {
	u8 sel_cmd;
	u8 b2;
	u8 nfcid1[4];
	u8 bcc;
} __packed;

static int digital_in_send_sdd_req(struct nfc_digital_dev *ddev,
				   struct nfc_target *target);

static void digital_in_recv_sel_res(struct nfc_digital_dev *ddev, void *arg,
				    struct sk_buff *resp)
{
	struct nfc_target *target = arg;
	int rc;
	u8 sel_res;
	u8 nfc_proto;

	if (IS_ERR(resp)) {
		rc = PTR_ERR(resp);
		resp = NULL;
		goto exit;
	}

	if (!DIGITAL_DRV_CAPS_IN_CRC(ddev)) {
		rc = digital_skb_check_crc_a(resp);
		if (rc) {
			PROTOCOL_ERR("4.4.1.3");
			goto exit;
		}
	}

	if (!resp->len) {
		rc = -EIO;
		goto exit;
	}

	sel_res = resp->data[0];

	if (!DIGITAL_SEL_RES_NFCID1_COMPLETE(sel_res)) {
		rc = digital_in_send_sdd_req(ddev, target);
		if (rc)
			goto exit;

		goto exit_free_skb;
	}

	if (DIGITAL_SEL_RES_IS_T2T(sel_res)) {
		nfc_proto = NFC_PROTO_MIFARE;
	} else {
		rc = -EOPNOTSUPP;
		goto exit;
	}

	target->sel_res = sel_res;

	rc = digital_target_found(ddev, target, nfc_proto);

exit:
	kfree(target);

exit_free_skb:
	dev_kfree_skb(resp);

	if (rc)
		digital_poll_next_tech(ddev);
}

static int digital_in_send_sel_req(struct nfc_digital_dev *ddev,
				   struct nfc_target *target,
				   struct digital_sdd_res *sdd_res)
{
	struct sk_buff *skb;
	struct digital_sel_req *sel_req;
	u8 sel_cmd;
	int rc;

	skb = digital_skb_alloc(ddev, sizeof(struct digital_sel_req));
	if (!skb)
		return -ENOMEM;

	skb_put(skb, sizeof(struct digital_sel_req));
	sel_req = (struct digital_sel_req *)skb->data;

	if (target->nfcid1_len <= 4)
		sel_cmd = DIGITAL_CMD_SEL_REQ_CL1;
	else if (target->nfcid1_len < 10)
		sel_cmd = DIGITAL_CMD_SEL_REQ_CL2;
	else
		sel_cmd = DIGITAL_CMD_SEL_REQ_CL3;

	sel_req->sel_cmd = sel_cmd;
	sel_req->b2 = 0x70;
	memcpy(sel_req->nfcid1, sdd_res->nfcid1, 4);
	sel_req->bcc = sdd_res->bcc;

	if (DIGITAL_DRV_CAPS_IN_CRC(ddev)) {
		rc = digital_in_configure_hw(ddev, NFC_DIGITAL_CONFIG_FRAMING,
				NFC_DIGITAL_FRAMING_NFCA_STANDARD_WITH_CRC_A);
		if (rc)
			goto exit;
	} else {
		digital_skb_add_crc_a(skb);
	}

	rc = digital_in_send_cmd(ddev, skb, 30, digital_in_recv_sel_res,
				 target);
exit:
	if (rc)
		kfree_skb(skb);

	return rc;
}

static void digital_in_recv_sdd_res(struct nfc_digital_dev *ddev, void *arg,
				    struct sk_buff *resp)
{
	struct nfc_target *target = arg;
	struct digital_sdd_res *sdd_res;
	int rc;
	u8 offset, size;
	u8 i, bcc;

	if (IS_ERR(resp)) {
		rc = PTR_ERR(resp);
		resp = NULL;
		goto exit;
	}

	if (resp->len < DIGITAL_SDD_RES_LEN) {
		PROTOCOL_ERR("4.7.2.8");
		rc = -EINVAL;
		goto exit;
	}

	sdd_res = (struct digital_sdd_res *)resp->data;

	for (i = 0, bcc = 0; i < 4; i++)
		bcc ^= sdd_res->nfcid1[i];

	if (bcc != sdd_res->bcc) {
		PROTOCOL_ERR("4.7.2.6");
		rc = -EINVAL;
		goto exit;
	}

	if (sdd_res->nfcid1[0] == DIGITAL_SDD_RES_CT) {
		offset = 1;
		size = 3;
	} else {
		offset = 0;
		size = 4;
	}

	memcpy(target->nfcid1 + target->nfcid1_len, sdd_res->nfcid1 + offset,
	       size);
	target->nfcid1_len += size;

	rc = digital_in_send_sel_req(ddev, target, sdd_res);

exit:
	dev_kfree_skb(resp);

	if (rc) {
		kfree(target);
		digital_poll_next_tech(ddev);
	}
}

static int digital_in_send_sdd_req(struct nfc_digital_dev *ddev,
				   struct nfc_target *target)
{
	int rc;
	struct sk_buff *skb;
	u8 sel_cmd;

	rc = digital_in_configure_hw(ddev, NFC_DIGITAL_CONFIG_FRAMING,
				     NFC_DIGITAL_FRAMING_NFCA_STANDARD);
	if (rc)
		return rc;

	skb = digital_skb_alloc(ddev, 2);
	if (!skb) {
		PR_ERR("alloc_skb failed");
		return -ENOMEM;
	}

	if (target->nfcid1_len == 0)
		sel_cmd = DIGITAL_CMD_SEL_REQ_CL1;
	else if (target->nfcid1_len == 3)
		sel_cmd = DIGITAL_CMD_SEL_REQ_CL2;
	else
		sel_cmd = DIGITAL_CMD_SEL_REQ_CL3;

	*skb_put(skb, sizeof(u8)) = sel_cmd;
	*skb_put(skb, sizeof(u8)) = DIGITAL_SDD_REQ_SEL_PAR;

	return digital_in_send_cmd(ddev, skb, 30, digital_in_recv_sdd_res,
				   target);
}

static void digital_in_recv_sens_res(struct nfc_digital_dev *ddev, void *arg,
				     struct sk_buff *resp)
{
	struct nfc_target *target = NULL;
	u16 sens_res;
	int rc;

	if (IS_ERR(resp)) {
		rc = PTR_ERR(resp);
		resp = NULL;
		goto exit;
	}

	if (resp->len < sizeof(u16)) {
		rc = -EIO;
		goto exit;
	}

	target = kzalloc(sizeof(struct nfc_target), GFP_KERNEL);
	if (!target) {
		rc = -ENOMEM;
		goto exit;
	}

	memcpy(&target->sens_res, resp->data, sizeof(u16));

	sens_res = be16_to_cpu(target->sens_res);

	if (!DIGITAL_SENS_RES_IS_VALID(sens_res)) {
		PROTOCOL_ERR("4.6.3.3");
		rc = -EINVAL;
		goto exit;
	}

	if (DIGITAL_SENS_RES_IS_T1T(sens_res))
		rc = digital_target_found(ddev, target, NFC_PROTO_JEWEL);
	else
		rc = digital_in_send_sdd_req(ddev, target);

exit:
	dev_kfree_skb(resp);

	if (rc) {
		kfree(target);
		digital_poll_next_tech(ddev);
	}
}

int digital_in_send_sens_req(struct nfc_digital_dev *ddev, u8 rf_tech)
{
	struct sk_buff *skb;
	int rc;

	rc = digital_in_configure_hw(ddev, NFC_DIGITAL_CONFIG_RF_TECH,
				     NFC_DIGITAL_RF_TECH_106A);
	if (rc)
		return rc;

	rc = digital_in_configure_hw(ddev, NFC_DIGITAL_CONFIG_FRAMING,
				     NFC_DIGITAL_FRAMING_NFCA_SHORT);
	if (rc)
		return rc;

	skb = digital_skb_alloc(ddev, 1);
	if (!skb)
		return -ENOMEM;

	*skb_put(skb, sizeof(u8)) = DIGITAL_CMD_SENS_REQ;

	rc = digital_in_send_cmd(ddev, skb, 30, digital_in_recv_sens_res, NULL);
	if (rc)
		kfree_skb(skb);

	return rc;
}

int digital_in_recv_mifare_res(struct sk_buff *resp)
{
	/* Successful READ command response is 16 data bytes + 2 CRC bytes long.
	 * Since the driver can't differentiate a ACK/NACK response from a valid
	 * READ response, the CRC calculation must be handled at digital level
	 * even if the driver supports it for this technology.
	 */
	if (resp->len == DIGITAL_MIFARE_READ_RES_LEN + DIGITAL_CRC_LEN) {
		if (digital_skb_check_crc_a(resp)) {
			PROTOCOL_ERR("9.4.1.2");
			return -EIO;
		}

		return 0;
	}

	/* ACK response (i.e. successful WRITE). */
	if (resp->len == 1 && resp->data[0] == DIGITAL_MIFARE_ACK_RES) {
		resp->data[0] = 0;
		return 0;
	}

	/* NACK and any other responses are treated as error. */
	return -EIO;
}
