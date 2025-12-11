/*
 *
 *  AicSemi Bluetooth USB driver
 *
 *
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/types.h>
#include <linux/sched.h>
#include <linux/errno.h>
#include <linux/skbuff.h>
#include <linux/usb.h>

#include <linux/ioctl.h>
#include <linux/io.h>
#include <linux/firmware.h>
#include <linux/vmalloc.h>
#include <linux/fs.h>
#include <linux/uaccess.h>
#include <linux/reboot.h>
#include <linux/compat.h>

#include "aic_btusb.h"

#include <linux/firmware.h>

#define AICBT_RELEASE_NAME "202012_ANDROID"
#define VERSION "2.1.0"

#define SUSPNED_DW_FW 0

static spinlock_t queue_lock;
static spinlock_t dlfw_lock;
static volatile uint16_t dlfw_dis_state = 0;

/* USB Device ID */
#define USB_VENDOR_ID_AIC 0xA69C
#define USB_VENDOR_ID_AIC_V2 0x368B
#define USB_PRODUCT_ID_AIC8801 0x8801
#define USB_PRODUCT_ID_AIC8800DC 0x88dc
#define USB_PRODUCT_ID_AIC8800D80 0x8d81
#define USB_PRODUCT_ID_AIC8800D80X2 0x8d91

enum AICWF_IC {
	PRODUCT_ID_AIC8801 = 0,
	PRODUCT_ID_AIC8800DC,
	PRODUCT_ID_AIC8800DW,
	PRODUCT_ID_AIC8800D80,
	PRODUCT_ID_AIC8800D80X2
};

u16 g_chipid = PRODUCT_ID_AIC8801;
u8 chip_id = 0;
u8 sub_chip_id = 0;

int btdual = 0;
int bt_support = 0;

module_param(btdual, int, 0660);
module_param(bt_support, int, 0660);

struct btusb_data {
	struct hci_dev *hdev;
	struct usb_device *udev;
	struct usb_interface *intf;
	struct usb_interface *isoc;

	spinlock_t lock;

	unsigned long flags;

	struct work_struct work;
	struct work_struct waker;

	struct usb_anchor tx_anchor;
	struct usb_anchor intr_anchor;
	struct usb_anchor bulk_anchor;
	struct usb_anchor isoc_anchor;
	struct usb_anchor deferred;
	int tx_in_flight;
	spinlock_t txlock;

	spinlock_t rxlock;
	struct sk_buff *evt_skb;
	struct sk_buff *acl_skb;
	struct sk_buff *sco_skb;

	struct usb_endpoint_descriptor *intr_ep;
	struct usb_endpoint_descriptor *bulk_tx_ep;
	struct usb_endpoint_descriptor *bulk_rx_ep;
	struct usb_endpoint_descriptor *isoc_tx_ep;
	struct usb_endpoint_descriptor *isoc_rx_ep;

	__u8 cmdreq_type;

	unsigned int sco_num;
	int isoc_altsetting;
	int suspend_count;
	uint16_t sco_handle;

	int (*recv_bulk)(struct btusb_data *data, void *buffer, int count);

	struct notifier_block pm_notifier;
	struct notifier_block reboot_notifier;

	firmware_info *fw_info;
};

static bool reset_on_close = 0;

const struct aicbt_firmware *aicbt_fw;

static inline int check_set_dlfw_state_value(uint16_t change_value)
{
	spin_lock(&dlfw_lock);
	if (!dlfw_dis_state) {
		dlfw_dis_state = change_value;
	}
	spin_unlock(&dlfw_lock);
	return dlfw_dis_state;
}

static inline void set_dlfw_state_value(uint16_t change_value)
{
	spin_lock(&dlfw_lock);
	dlfw_dis_state = change_value;
	spin_unlock(&dlfw_lock);
}

static void aic_free(struct btusb_data *data)
{
	return;
}

static struct btusb_data *aic_alloc(struct usb_interface *intf)
{
	struct btusb_data *data;
	data = devm_kzalloc(&intf->dev, sizeof(*data), GFP_KERNEL);
	return data;
}

static void print_acl(struct sk_buff *skb, int direction)
{
#if PRINT_ACL_DATA
	//uint wlength = skb->len;
	u16 *handle = (u16 *)(skb->data);
	u16 len = *(handle + 1);
	//u8 *acl_data = (u8 *)(skb->data);

	AICBT_INFO("aic %s: direction %d, handle %04x, len %d", __func__,
		   direction, *handle, len);
#endif
}

static void print_sco(struct sk_buff *skb, int direction)
{
#if PRINT_SCO_DATA
	uint wlength = skb->len;
	u16 *handle = (u16 *)(skb->data);
	u8 len = *(u8 *)(handle + 1);
	//u8 *sco_data =(u8 *)(skb->data);

	AICBT_INFO("aic %s: direction %d, handle %04x, len %d,wlength %d",
		   __func__, direction, *handle, len, wlength);
#endif
}

static void print_error_command(struct sk_buff *skb)
{
	u16 *opcode = (u16 *)(skb->data);
	u8 *cmd_data = (u8 *)(skb->data);
	u8 len = *(cmd_data + 2);

	printk(" 0x%04x,len:%d,", *opcode, len);
}

static void print_command(struct sk_buff *skb)
{
#if PRINT_CMD_EVENT
	print_error_command(skb);
#endif
}

enum CODEC_TYPE {
	CODEC_CVSD,
	CODEC_MSBC,
};

static enum CODEC_TYPE codec_type = CODEC_CVSD;
static void set_select_msbc(enum CODEC_TYPE type);
static enum CODEC_TYPE check_select_msbc(void);

int send_hci_cmd(firmware_info *fw_info)
{
	int ret_val = -1;
	int i = 0;

	while ((ret_val < 0) && (i++ < 3)) {
		ret_val = usb_control_msg(fw_info->udev, fw_info->pipe_out, 0,
					  USB_TYPE_CLASS, 0, 0,
					  (void *)(fw_info->send_pkt),
					  fw_info->pkt_len, MSG_TO);

		if (ret_val > 0) {
			AICBT_DBG("Right in send hci cmd = %d,"
				  "size = %d",
				  ret_val, fw_info->pkt_len);
		} else {
			AICBT_ERR("Error in send hci cmd = %d,"
				  "size = %d",
				  ret_val, fw_info->pkt_len);
		}
	}
	return ret_val;
}

int rcv_hci_evt(firmware_info *fw_info)
{
	int ret_len = 0, ret_val = 0;
	int i;

	while (1) {
		for (i = 0; i < 5; i++) {
			ret_val = usb_interrupt_msg(fw_info->udev,
						    fw_info->pipe_in,
						    (void *)(fw_info->rcv_pkt),
						    RCV_PKT_LEN, &ret_len,
						    MSG_TO);
			if (ret_val >= 0)
				break;
		}

		if (ret_val < 0)
			return ret_val;

		if (CMD_CMP_EVT == fw_info->evt_hdr->evt) {
			if (fw_info->cmd_hdr->opcode ==
			    fw_info->cmd_cmp->opcode)
				return ret_len;
		}
	}
}

//for 8800DC start
u32 fwcfg_tbl[][2] = {
	{ 0x40200028, 0x0021047e },
	{ 0x40200024, 0x0000011d },
};

//Crystal provided by CPU (start)
int hci_send_dbg_rd_mem_cmd(firmware_info *fw_info, u32 addr)
{
	struct hci_dbg_rd_mem_cmd *rd_cmd;
	struct hci_dbg_rd_mem_cmd_evt *evt_para;

	int ret_val = -1;

	rd_cmd = (struct hci_dbg_rd_mem_cmd *)(fw_info->req_para);
	if (!rd_cmd)
		return -ENOMEM;

	rd_cmd->start_addr = addr;
	rd_cmd->type = 32;
	rd_cmd->length = 4;
	fw_info->cmd_hdr->opcode = cpu_to_le16(HCI_VSC_DBG_RD_MEM_CMD);
	fw_info->cmd_hdr->plen = sizeof(struct hci_dbg_rd_mem_cmd);
	fw_info->pkt_len = CMD_HDR_LEN + sizeof(struct hci_dbg_rd_mem_cmd);

	ret_val = send_hci_cmd(fw_info);
	if (ret_val < 0) {
		printk("%s: Failed to send hci cmd 0x%04x, errno %d", __func__,
		       fw_info->cmd_hdr->opcode, ret_val);
		return ret_val;
	}

	ret_val = rcv_hci_evt(fw_info);
	if (ret_val < 0) {
		printk("%s: Failed to receive hci event, errno %d", __func__,
		       ret_val);
		return ret_val;
	}

	evt_para = (struct hci_dbg_rd_mem_cmd_evt *)(fw_info->rsp_para);

	printk("%s: fw status = 0x%04x, length %d, %x %x %x %x", __func__,
	       evt_para->status, evt_para->length, evt_para->data[0],
	       evt_para->data[1], evt_para->data[2], evt_para->data[3]);

	return 0;
}

int set_bbpll_config(firmware_info *fw_info)
{
	int ret_val = -1;
	struct fw_status *evt_status;
	struct hci_dbg_rd_mem_cmd_evt *evt_para;
	struct aicbt_patch_table_cmd *patch_table_cmd;

	//Read crystal provided by CPU or not.
	ret_val = hci_send_dbg_rd_mem_cmd(fw_info, 0x40500148);
	if (ret_val < 0) {
		printk("%s error ret_val:%d\r\n", __func__, ret_val);
		return ret_val;
	}
	evt_para = (struct hci_dbg_rd_mem_cmd_evt *)(fw_info->rsp_para);

	if (!(evt_para->data[0] & 0x01)) {
		printk("%s Crystal not provided by CPU \r\n", __func__);
		return 0;
	} else {
		printk("%s Crystal provided by CPU \r\n", __func__);

		//Read 0x40505010 value to check bbpll set or not.
		ret_val = hci_send_dbg_rd_mem_cmd(fw_info, 0x40505010);
		if (ret_val < 0) {
			printk("%s error ret_val:%d\r\n", __func__, ret_val);
			return ret_val;
		}
		evt_para = (struct hci_dbg_rd_mem_cmd_evt *)(fw_info->rsp_para);
		if ((evt_para->data[3] >> 5) == 3) {
			printk("%s Not need to set \r\n", __func__);
			return 0;
		} else {
			patch_table_cmd = (struct aicbt_patch_table_cmd
						   *)(fw_info->req_para);
			patch_table_cmd->patch_num = 1;
			patch_table_cmd->patch_table_addr[0] = 0x40505010;
			evt_para->data[3] |= ((0x1 << 5) | (0x1 << 6));
			evt_para->data[3] &= (~(0x1 << 7));
			patch_table_cmd->patch_table_data[0] = 0;
			patch_table_cmd->patch_table_data[0] = evt_para->data[3]
							       << 24;
			patch_table_cmd->patch_table_data[0] |=
				evt_para->data[2] << 16;
			patch_table_cmd->patch_table_data[0] |=
				evt_para->data[1] << 8;
			patch_table_cmd->patch_table_data[0] |=
				evt_para->data[0];
			//printk("%s patch_table_cmd->patch_table_data[0]:%x \r\n", __func__,
			//patch_table_cmd->patch_table_data[0]);
		}
		//		patch_table_cmd->patch_table_data[0] = 0x7C301010;

		fw_info->cmd_hdr->opcode = cpu_to_le16(HCI_VSC_UPDATE_PT_CMD);
		fw_info->cmd_hdr->plen = HCI_VSC_UPDATE_PT_SIZE;
		fw_info->pkt_len = fw_info->cmd_hdr->plen + 3;
		ret_val = send_hci_cmd(fw_info);
		if (ret_val < 0) {
			AICBT_ERR("%s: rcv_hci_evt err %d", __func__, ret_val);
			return ret_val;
		}
		ret_val = rcv_hci_evt(fw_info);
		if (ret_val < 0) {
			printk("%s: Failed to receive hci event, errno %d",
			       __func__, ret_val);
			return ret_val;
		}
		evt_status = (struct fw_status *)fw_info->rsp_para;
		ret_val = evt_status->status;
		if (0 != evt_status->status) {
			ret_val = -1;
		} else {
			ret_val = 0;
		}
	}

	return ret_val;
}
//Crystal provided by CPU (end)

int fw_config(firmware_info *fw_info)
{
	int ret_val = -1;
	struct hci_dbg_rd_mem_cmd *rd_cmd;
	struct hci_dbg_rd_mem_cmd_evt *evt_para;
	int len = 0, i = 0;
	struct fw_status *evt_status;

	rd_cmd = (struct hci_dbg_rd_mem_cmd *)(fw_info->req_para);
	if (!rd_cmd)
		return -ENOMEM;

	rd_cmd->start_addr = 0x40200024;
	rd_cmd->type = 32;
	rd_cmd->length = 4;
	fw_info->cmd_hdr->opcode = cpu_to_le16(HCI_VSC_DBG_RD_MEM_CMD);
	fw_info->cmd_hdr->plen = sizeof(struct hci_dbg_rd_mem_cmd);
	fw_info->pkt_len = CMD_HDR_LEN + sizeof(struct hci_dbg_rd_mem_cmd);

	ret_val = send_hci_cmd(fw_info);
	if (ret_val < 0) {
		printk("%s: Failed to send hci cmd 0x%04x, errno %d", __func__,
		       fw_info->cmd_hdr->opcode, ret_val);
		return ret_val;
	}

	ret_val = rcv_hci_evt(fw_info);
	if (ret_val < 0) {
		printk("%s: Failed to receive hci event, errno %d", __func__,
		       ret_val);
		return ret_val;
	}

	evt_para = (struct hci_dbg_rd_mem_cmd_evt *)(fw_info->rsp_para);

	printk("%s: fw status = 0x%04x, length %d, %x %x %x %x", __func__,
	       evt_para->status, evt_para->length, evt_para->data[0],
	       evt_para->data[1], evt_para->data[2], evt_para->data[3]);

	ret_val = evt_para->status;
	if (evt_para->status == 0) {
		uint16_t rd_data =
			(evt_para->data[0] | (evt_para->data[1] << 8));
		printk("%s rd_data is %x\n", __func__, rd_data);
		if (rd_data == 0x119) {
			struct aicbt_patch_table_cmd *patch_table_cmd =
				(struct aicbt_patch_table_cmd
					 *)(fw_info->req_para);
			len = sizeof(fwcfg_tbl) / sizeof(u32) / 2;
			patch_table_cmd->patch_num = len;
			for (i = 0; i < len; i++) {
				memcpy(&patch_table_cmd->patch_table_addr[i],
				       &fwcfg_tbl[i][0], sizeof(uint32_t));
				memcpy(&patch_table_cmd->patch_table_data[i],
				       &fwcfg_tbl[i][1], sizeof(uint32_t));
				//printk("%s [%d] data: %08x %08x\n", __func__, i, patch_table_cmd->patch_table_addr[i],patch_table_cmd->patch_table_data[i]);
			}
			fw_info->cmd_hdr->opcode =
				cpu_to_le16(HCI_VSC_UPDATE_PT_CMD);
			fw_info->cmd_hdr->plen = HCI_VSC_UPDATE_PT_SIZE;
			fw_info->pkt_len = fw_info->cmd_hdr->plen + 3;
			ret_val = send_hci_cmd(fw_info);
			if (ret_val < 0) {
				AICBT_ERR("%s: rcv_hci_evt err %d", __func__,
					  ret_val);
				return ret_val;
			}
			ret_val = rcv_hci_evt(fw_info);
			if (ret_val < 0) {
				printk("%s: Failed to receive hci event, errno %d",
				       __func__, ret_val);
				return ret_val;
			}
			evt_status = (struct fw_status *)fw_info->rsp_para;
			ret_val = evt_status->status;
			if (0 != evt_status->status) {
				ret_val = -1;
			} else {
				ret_val = 0;
			}
		}

		//Crystal provided by CPU (start)
		ret_val = set_bbpll_config(fw_info);
		//Crystal provided by CPU (end)
	}
	return ret_val;
}

int system_config(firmware_info *fw_info)
{
	int ret_val = -1;
	struct hci_dbg_rd_mem_cmd *rd_cmd;
	struct hci_dbg_rd_mem_cmd_evt *evt_para;
	//int len = 0, i = 0;
	//struct fw_status *evt_status;

	rd_cmd = (struct hci_dbg_rd_mem_cmd *)(fw_info->req_para);
	if (!rd_cmd)
		return -ENOMEM;

	rd_cmd->start_addr = 0x40500000;
	rd_cmd->type = 32;
	rd_cmd->length = 4;
	fw_info->cmd_hdr->opcode = cpu_to_le16(HCI_VSC_DBG_RD_MEM_CMD);
	fw_info->cmd_hdr->plen = sizeof(struct hci_dbg_rd_mem_cmd);
	fw_info->pkt_len = CMD_HDR_LEN + sizeof(struct hci_dbg_rd_mem_cmd);

	ret_val = send_hci_cmd(fw_info);
	if (ret_val < 0) {
		printk("%s: Failed to send hci cmd 0x%04x, errno %d", __func__,
		       fw_info->cmd_hdr->opcode, ret_val);
		return ret_val;
	}

	ret_val = rcv_hci_evt(fw_info);
	if (ret_val < 0) {
		printk("%s: Failed to receive hci event, errno %d", __func__,
		       ret_val);
		return ret_val;
	}

	evt_para = (struct hci_dbg_rd_mem_cmd_evt *)(fw_info->rsp_para);

	printk("%s: fw status = 0x%04x, length %d, %x %x %x %x", __func__,
	       evt_para->status, evt_para->length, evt_para->data[0],
	       evt_para->data[1], evt_para->data[2], evt_para->data[3]);

	ret_val = evt_para->status;
	if (evt_para->status == 0) {
		uint32_t rd_data =
			(evt_para->data[0] | (evt_para->data[1] << 8) |
			 (evt_para->data[2] << 16) | (evt_para->data[3] << 24));
		//printk("%s 0x40500000 rd_data is %x\n", __func__, rd_data);
		chip_id = (u8)(rd_data >> 16);
		//btenable = (u8) ((rd_data >> 26) && 0x01);
		btdual = (u8)((rd_data >> 27) && 0x01);
	}

	rd_cmd->start_addr = 0x20;
	rd_cmd->type = 32;
	rd_cmd->length = 4;
	fw_info->cmd_hdr->opcode = cpu_to_le16(HCI_VSC_DBG_RD_MEM_CMD);
	fw_info->cmd_hdr->plen = sizeof(struct hci_dbg_rd_mem_cmd);
	fw_info->pkt_len = CMD_HDR_LEN + sizeof(struct hci_dbg_rd_mem_cmd);

	ret_val = send_hci_cmd(fw_info);
	if (ret_val < 0) {
		printk("%s: Failed to send hci cmd 0x%04x, errno %d", __func__,
		       fw_info->cmd_hdr->opcode, ret_val);
		return ret_val;
	}

	ret_val = rcv_hci_evt(fw_info);
	if (ret_val < 0) {
		printk("%s: Failed to receive hci event, errno %d", __func__,
		       ret_val);
		return ret_val;
	}

	evt_para = (struct hci_dbg_rd_mem_cmd_evt *)(fw_info->rsp_para);

	printk("%s: fw status = 0x%04x, length %d, %x %x %x %x", __func__,
	       evt_para->status, evt_para->length, evt_para->data[0],
	       evt_para->data[1], evt_para->data[2], evt_para->data[3]);

	ret_val = evt_para->status;
	if (evt_para->status == 0) {
		uint32_t rd_data =
			(evt_para->data[0] | (evt_para->data[1] << 8) |
			 (evt_para->data[2] << 16) | (evt_para->data[3] << 24));
		//printk("%s 0x02 rd_data is %x\n", __func__, rd_data);
		sub_chip_id = (u8)(rd_data);
	}
	printk("chip_id = %x, sub_chip_id = %x\n", chip_id, sub_chip_id);
	return ret_val;
}

int check_fw_status(firmware_info *fw_info)
{
	struct fw_status *read_ver_rsp;
	int ret_val = -1;

	fw_info->cmd_hdr->opcode = cpu_to_le16(HCI_VSC_FW_STATUS_GET_CMD);
	fw_info->cmd_hdr->plen = 0;
	fw_info->pkt_len = CMD_HDR_LEN;

	ret_val = send_hci_cmd(fw_info);
	if (ret_val < 0) {
		printk("%s: Failed to send hci cmd 0x%04x, errno %d", __func__,
		       fw_info->cmd_hdr->opcode, ret_val);
		return ret_val;
	}

	ret_val = rcv_hci_evt(fw_info);
	if (ret_val < 0) {
		printk("%s: Failed to receive hci event, errno %d", __func__,
		       ret_val);
		return ret_val;
	}

	read_ver_rsp = (struct fw_status *)(fw_info->rsp_para);

	printk("%s: fw status = 0x%04x", __func__, read_ver_rsp->status);
	return read_ver_rsp->status;
}

int download_data(firmware_info *fw_info, u32 fw_addr, const char *filename)
{
	unsigned int i = 0;
	int size;
	u8 *dst = NULL;
	int err = 0;
	struct hci_dbg_wr_mem_cmd *dl_cmd;
	int hdr_len = sizeof(__le32) + sizeof(__u8) + sizeof(__u8);
	int data_len = HCI_VSC_MEM_WR_SIZE;
	int frag_len = data_len + hdr_len;
	int ret_val;
	int ncmd = 1;
	struct fw_status *evt_para;

	/* load aic firmware */
	size = aic_load_firmware(&dst, filename, NULL);
	if (size <= 0) {
		printk("wrong size of firmware file\n");
		vfree(dst);
		dst = NULL;
		return -1;
	}

	dl_cmd = (struct hci_dbg_wr_mem_cmd *)(fw_info->req_para);
	if (!dl_cmd)
		return -ENOMEM;
	evt_para = (struct fw_status *)fw_info->rsp_para;

	/* Copy the file on the Embedded side */
	printk("### Upload %s firmware, @ = %x  size=%d\n", filename, fw_addr,
	       size);

	if (size > HCI_VSC_MEM_WR_SIZE) { // > 1KB data
		for (i = 0; i < (size - HCI_VSC_MEM_WR_SIZE);
		     i += HCI_VSC_MEM_WR_SIZE) { //each time write 240 bytes
			data_len = HCI_VSC_MEM_WR_SIZE;
			frag_len = data_len + hdr_len;
			memcpy(dl_cmd->data, dst + i, data_len);
			dl_cmd->length = data_len;
			dl_cmd->type = 32;
			dl_cmd->start_addr = fw_addr + i;
			fw_info->cmd_hdr->opcode = cpu_to_le16(DOWNLOAD_OPCODE);
			fw_info->cmd_hdr->plen = frag_len;
			fw_info->pkt_len = frag_len + 3;
			ret_val = send_hci_cmd(fw_info);

			while (ncmd > 0) {
				ret_val = rcv_hci_evt(fw_info);
				//print_hex_dump(KERN_ERR,"evt data:",DUMP_PREFIX_NONE,16,1,fw_info->rcv_pkt,ret_val,false);
				//printk("rcv_hci_evt %d\n", ret_val);
				if (ret_val < 0) {
					AICBT_ERR("%s: rcv_hci_evt err %d",
						  __func__, ret_val);
					goto out;
				} else {
					AICBT_DBG(
						"%s: Receive acked frag num %d",
						__func__, evt_para->status);
					ncmd--;
				}
				if (0 != evt_para->status) {
					AICBT_ERR(
						"%s: Receive acked frag num %d, err status %d",
						__func__, ret_val,
						evt_para->status);
					ret_val = -1;
					goto out;
				}
			}
			ncmd = 1;
		}
	}

	if (!err && (i < size)) { // <1KB data
		data_len = size - i;
		frag_len = data_len + hdr_len;
		memcpy(dl_cmd->data, dst + i, data_len);
		dl_cmd->length = data_len;
		dl_cmd->type = 32;
		dl_cmd->start_addr = fw_addr + i;
		fw_info->cmd_hdr->opcode = cpu_to_le16(DOWNLOAD_OPCODE);
		fw_info->cmd_hdr->plen = frag_len;
		fw_info->pkt_len = frag_len + 3;
		ret_val = send_hci_cmd(fw_info);
		//printk("(%d) data_len %d, src %x, dst %x\n", i, data_len, (dst + i), fw_addr + i);
		//printk("%p , %d\n", dl_cmd, fw_info->pkt_len);
		/* Send download command */
		//print_hex_dump(KERN_ERR,"cmd data:",DUMP_PREFIX_NONE,16,1,fw_info->send_pkt,32,false);
		while (ncmd > 0) {
			ret_val = rcv_hci_evt(fw_info);
			//print_hex_dump(KERN_ERR,"evt data:",DUMP_PREFIX_NONE,16,1,fw_info->rcv_pkt,ret_val,false);
			if (ret_val < 0) {
				AICBT_ERR("%s: rcv_hci_evt err %d", __func__,
					  ret_val);
				goto out;
			} else {
				AICBT_DBG("%s: Receive acked frag num %d",
					  __func__, evt_para->status);
				ncmd--;
			}
			if (0 != evt_para->status) {
				AICBT_ERR(
					"%s: Receive acked frag num %d, err status %d",
					__func__, ret_val, evt_para->status);
				ret_val = -1;
				goto out;
			}
		}
		ncmd = 0;
	}

out:
	if (dst) {
		vfree(dst);
		dst = NULL;
	}

	if (ret_val != -1) {
		ret_val = 0;
	}

	printk("fw download complete\n\n");
	return ret_val;
}

struct aicbt_info_t {
	uint32_t btmode;
	uint32_t btport;
	uint32_t uart_baud;
	uint32_t uart_flowctrl;
	uint32_t lpm_enable;
	uint32_t txpwr_lvl;
};

struct aicbsp_info_t {
	int hwinfo;
	uint32_t cpmode;
};

enum aicbsp_cpmode_type {
	AICBSP_CPMODE_WORK,
	AICBSP_CPMODE_TEST,
};

/*  btmode
 * used for force bt mode,if not AICBSP_MODE_NULL
 * efuse valid and vendor_info will be invalid, even has beed set valid
*/
enum aicbt_btmode_type {
	AICBT_BTMODE_BT_ONLY_SW = 0x0, // bt only mode with switch
	AICBT_BTMODE_BT_WIFI_COMBO, // wifi/bt combo mode
	AICBT_BTMODE_BT_ONLY, // bt only mode without switch
	AICBT_BTMODE_BT_ONLY_TEST, // bt only test mode
	AICBT_BTMODE_BT_WIFI_COMBO_TEST, // wifi/bt combo test mode
	AICBT_MODE_NULL = 0xFF, // invalid value
};

enum aicbt_btport_type {
	AICBT_BTPORT_NULL,
	AICBT_BTPORT_MB,
	AICBT_BTPORT_UART,
};

enum aicbt_uart_baud_type {
	AICBT_UART_BAUD_115200 = 115200,
	AICBT_UART_BAUD_921600 = 921600,
	AICBT_UART_BAUD_1_5M = 1500000,
	AICBT_UART_BAUD_3_25M = 3250000,
};

enum aicbt_uart_flowctrl_type {
	AICBT_UART_FLOWCTRL_DISABLE = 0x0, // uart without flow ctrl
	AICBT_UART_FLOWCTRL_ENABLE, // uart with flow ctrl
};

#define AICBSP_HWINFO_DEFAULT (-1)
#define AICBSP_CPMODE_DEFAULT AICBSP_CPMODE_WORK
#define AICBT_TXPWR_DFT 0x6F2F

#define AICBT_BTMODE_DEFAULT AICBT_BTMODE_BT_WIFI_COMBO
#define AICBT_BTPORT_DEFAULT AICBT_BTPORT_MB
#define AICBT_UART_BAUD_DEFAULT AICBT_UART_BAUD_1_5M
#define AICBT_UART_FC_DEFAULT AICBT_UART_FLOWCTRL_ENABLE
#define AICBT_LPM_ENABLE_DEFAULT 0
#define AICBT_TXPWR_LVL_DEFAULT AICBT_TXPWR_DFT

struct aicbsp_info_t aicbsp_info = {
	.hwinfo = AICBSP_HWINFO_DEFAULT,
	.cpmode = AICBSP_CPMODE_DEFAULT,
};

static struct aicbt_info_t aicbt_info = {
	.btmode = AICBT_BTMODE_DEFAULT,
	.btport = AICBT_BTPORT_DEFAULT,
	.uart_baud = AICBT_UART_BAUD_DEFAULT,
	.uart_flowctrl = AICBT_UART_FC_DEFAULT,
	.lpm_enable = AICBT_LPM_ENABLE_DEFAULT,
	.txpwr_lvl = AICBT_TXPWR_LVL_DEFAULT,
};

int patch_table_load(firmware_info *fw_info, struct aicbt_patch_table *_head)
{
	struct aicbt_patch_table *head, *p;
	int i;
	uint32_t *data = NULL;
	struct aicbt_patch_table_cmd *patch_table_cmd =
		(struct aicbt_patch_table_cmd *)(fw_info->req_para);
	struct fw_status *evt_para;
	int ret_val = 0;
	int ncmd = 1;
	uint32_t len = 0;
	uint32_t tot_len = 0;
	head = _head;
	for (p = head; p != NULL; p = p->next) {
		data = p->data;
		if (AICBT_PT_BTMODE == p->type) {
			*(data + 1) = aicbsp_info.hwinfo < 0;
			*(data + 3) = aicbsp_info.hwinfo;
			*(data + 5) = aicbsp_info.cpmode;

			*(data + 7) = aicbt_info.btmode;
			*(data + 9) = aicbt_info.btport;
			*(data + 11) = aicbt_info.uart_baud;
			*(data + 13) = aicbt_info.uart_flowctrl;
			*(data + 15) = aicbt_info.lpm_enable;
			*(data + 17) = aicbt_info.txpwr_lvl;

			printk("%s bt btmode:%d \r\n", __func__,
			       aicbt_info.btmode);
			printk("%s bt uart_baud:%d \r\n", __func__,
			       aicbt_info.uart_baud);
			printk("%s bt uart_flowctrl:%d \r\n", __func__,
			       aicbt_info.uart_flowctrl);
			printk("%s bt lpm_enable:%d \r\n", __func__,
			       aicbt_info.lpm_enable);
			printk("%s bt tx_pwr:%d \r\n", __func__,
			       aicbt_info.txpwr_lvl);
		}
		if (p->type == AICBT_PT_INF || p->type == AICBT_PT_PWRON) {
			continue;
		}
		if (p->type == AICBT_PT_VER) {
			char *data_s = (char *)p->data;
			printk("patch version %s\n", data_s);
			continue;
		}
		if (p->len == 0) {
			printk("len is 0\n");
			continue;
		}
		tot_len = p->len;
		while (tot_len) {
			if (tot_len > HCI_PT_MAX_LEN) {
				len = HCI_PT_MAX_LEN;
			} else {
				len = tot_len;
			}
			for (i = 0; i < len; i++) {
				patch_table_cmd->patch_num = len;
				memcpy(&patch_table_cmd->patch_table_addr[i],
				       data, sizeof(uint32_t));
				memcpy(&patch_table_cmd->patch_table_data[i],
				       data + 1, sizeof(uint32_t));
				//printk("[%d] data: %08x %08x\n", i, patch_table_cmd->patch_table_addr[i],patch_table_cmd->patch_table_data[i]);
				data += 2;
			}
			tot_len -= len;
			evt_para = (struct fw_status *)fw_info->rsp_para;
			//print_hex_dump(KERN_ERR,"data0:",DUMP_PREFIX_NONE,16,1,patch_table_cmd,sizeof(struct aicbt_patch_table_cmd),false);

			//printk("patch num %x %d\n", patch_table_cmd->patch_num, (int)sizeof(struct aicbt_patch_table_cmd));
			fw_info->cmd_hdr->opcode =
				cpu_to_le16(HCI_VSC_UPDATE_PT_CMD);
			fw_info->cmd_hdr->plen = HCI_VSC_UPDATE_PT_SIZE;
			fw_info->pkt_len = fw_info->cmd_hdr->plen + 3;
			AICBT_DBG("patch num 0x%x, plen 0x%x\n",
				  patch_table_cmd->patch_num,
				  fw_info->cmd_hdr->plen);
			//print_hex_dump(KERN_ERR,"patch table:",DUMP_PREFIX_NONE,16,1,fw_info->send_pkt,32,false);
			ret_val = send_hci_cmd(fw_info);
			while (ncmd > 0) {
				ret_val = rcv_hci_evt(fw_info);
				//printk("%s ret_val:%d \r\n", __func__, ret_val);
				if (ret_val < 0) {
					AICBT_ERR("%s: rcv_hci_evt err %d",
						  __func__, ret_val);
					goto out;
				} else {
					AICBT_DBG(
						"%s: Receive acked frag num %d",
						__func__, evt_para->status);
					ncmd--;
				}
				if (0 != evt_para->status) {
					AICBT_ERR(
						"%s: Receive acked frag num %d, err status %d",
						__func__, ret_val,
						evt_para->status);
					ret_val = -1;
					goto out;
				} else {
					ret_val = 0;
				}
			}
			ncmd = 1;
		}
	}
out:
	//aicbt_patch_table_free(&head);

	return ret_val;
}

int aic_load_firmware(u8 **fw_buf, const char *name, struct device *device)
{
	const struct firmware *fw = NULL;
	u32 *dst = NULL;
	void *buffer = NULL;
	int size = 0;
	int ret = 0;

	printk("%s: request firmware = %s \n", __func__, name);

	ret = request_firmware(&fw, name, NULL);

	if (ret < 0) {
		printk("Load %s fail\n", name);
		release_firmware(fw);
		return -1;
	}

	size = fw->size;
	dst = (u32 *)fw->data;

	if (size <= 0) {
		printk("wrong size of firmware file\n");
		release_firmware(fw);
		return -1;
	}

	buffer = vmalloc(size);
	memset(buffer, 0, size);
	memcpy(buffer, dst, size);

	*fw_buf = buffer;

	release_firmware(fw);

	return size;
}

int aicbt_patch_table_free(struct aicbt_patch_table **head)
{
	struct aicbt_patch_table *p = *head, *n = NULL;
	while (p) {
		n = p->next;
		kfree(p->name);
		kfree(p->data);
		kfree(p);
		p = n;
	}
	*head = NULL;
	return 0;
}

struct aicbt_patch_table *aicbt_patch_table_alloc(const char *filename)
{
	uint8_t *rawdata = NULL, *p;
	int size;
	struct aicbt_patch_table *head = NULL, *new = NULL, *cur = NULL;

	/* load aic firmware */
	size = aic_load_firmware((u8 **)&rawdata, filename, NULL);
	if (size <= 0) {
		printk("wrong size of firmware file\n");
		goto err;
	}

	p = rawdata;
	if (memcmp(p, AICBT_PT_TAG,
		   sizeof(AICBT_PT_TAG) < 16 ? sizeof(AICBT_PT_TAG) : 16)) {
		printk("TAG err\n");
		goto err;
	}
	p += 16;

	while (p - rawdata < size) {
		printk("size = %d  p - rawdata = 0x%0lx \r\n", size,
		       p - rawdata);
		new = (struct aicbt_patch_table *)vmalloc(
			sizeof(struct aicbt_patch_table));
		memset(new, 0, sizeof(struct aicbt_patch_table));
		if (head == NULL) {
			head = new;
			cur = new;
		} else {
			cur->next = new;
			cur = cur->next;
		}

		cur->name = (char *)vmalloc(sizeof(char) * 16);
		memset(cur->name, 0, sizeof(char) * 16);
		memcpy(cur->name, p, 16);
		p += 16;

		cur->type = *(uint32_t *)p;
		p += 4;

		cur->len = *(uint32_t *)p;
		p += 4;

		printk("cur->type %x, len %d\n", cur->type, cur->len);

		if ((cur->type) >= 1000) { //Temp Workaround
			cur->len = 0;
		} else {
			if (cur->len > 0) {
				cur->data = (uint32_t *)vmalloc(
					sizeof(uint8_t) * cur->len * 8);
				memset(cur->data, 0,
				       sizeof(uint8_t) * cur->len * 8);
				memcpy(cur->data, p, cur->len * 8);
				p += cur->len * 8;
			}
		}
	}

	if (rawdata)
		vfree(rawdata);

	return head;

err:
	aicbt_patch_table_free(&head);
	if (rawdata)
		vfree(rawdata);
	return NULL;
}

int aicbt_patch_info_unpack(struct aicbt_patch_info_t *patch_info,
			    struct aicbt_patch_table *head_t)
{
	uint8_t *patch_info_array = (uint8_t *)patch_info;
	int base_len = 0;
	int memcpy_len = 0;

	if (AICBT_PT_INF == head_t->type) {
		base_len =
			((offsetof(struct aicbt_patch_info_t,
				   ext_patch_nb_addr) -
			  offsetof(struct aicbt_patch_info_t, adid_addrinf)) /
			 sizeof(uint32_t)) /
			2;
		printk("%s head_t->len:%d base_len:%d \r\n", __func__,
		       head_t->len, base_len);

		if (head_t->len > base_len) {
			patch_info->info_len = base_len;
			memcpy_len =
				patch_info->info_len + 1; //include ext patch nb
		} else {
			patch_info->info_len = head_t->len;
			memcpy_len = patch_info->info_len;
		}
		printk("%s memcpy_len:%d \r\n", __func__, memcpy_len);

		if (patch_info->info_len == 0)
			return 0;

		memcpy(((patch_info_array) + sizeof(patch_info->info_len)),
		       head_t->data, memcpy_len * sizeof(uint32_t) * 2);
		printk("%s adid_addrinf:%x addr_adid:%x \r\n", __func__,
		       ((struct aicbt_patch_info_t *)patch_info_array)
			       ->adid_addrinf,
		       ((struct aicbt_patch_info_t *)patch_info_array)
			       ->addr_adid);

		if (patch_info->ext_patch_nb > 0) {
			int index = 0;
			patch_info->ext_patch_param =
				(uint32_t *)(head_t->data + ((memcpy_len) * 2));

			for (index = 0; index < patch_info->ext_patch_nb;
			     index++) {
				printk("%s id:%x addr:%x \r\n", __func__,
				       *(patch_info->ext_patch_param +
					 (index * 2)),
				       *(patch_info->ext_patch_param +
					 (index * 2) + 1));
			}
		}
	}
	return 0;
}

int rwnx_send_dbg_mem_write_req(firmware_info *fw_info, u32 mem_addr,
				u32 mem_data)
{
	int ret_val = -1;
	struct hci_dbg_wr_mem_cmd *dl_cmd;
	int data_len = 4;
	int frag_len = 0;
	int hdr_len = sizeof(__le32) + sizeof(__u8) + sizeof(__u8);

	frag_len = data_len + hdr_len;
	dl_cmd = (struct hci_dbg_wr_mem_cmd *)(fw_info->req_para);
	if (!dl_cmd)
		return -ENOMEM;

	dl_cmd->start_addr = mem_addr;
	dl_cmd->type = 32;
	dl_cmd->length = data_len;
	memcpy(dl_cmd->data, (__u8 *)(&mem_data), data_len);
	frag_len = data_len + hdr_len;
	fw_info->cmd_hdr->opcode = cpu_to_le16(DOWNLOAD_OPCODE);
	fw_info->cmd_hdr->plen = frag_len;
	fw_info->pkt_len = frag_len + sizeof(struct hci_command_hdr);
	;

	//print_hex_dump(KERN_ERR,"data:",DUMP_PREFIX_NONE,16,1,fw_info->send_pkt,fw_info->pkt_len,false);

	ret_val = send_hci_cmd(fw_info);
	if (ret_val < 0) {
		printk("%s: Failed to send hci cmd 0x%04x, errno %d", __func__,
		       fw_info->cmd_hdr->opcode, ret_val);
		return ret_val;
	}

	ret_val = rcv_hci_evt(fw_info);
	//print_hex_dump(KERN_ERR,"evt data:",DUMP_PREFIX_NONE,16,1,fw_info->rcv_pkt,ret_val,false);
	if (ret_val < 0) {
		printk("%s: Failed to receive hci event, errno %d", __func__,
		       ret_val);
		return ret_val;
	} else {
		ret_val = 0;
	}

	return ret_val;
}

int aicbt_ext_patch_data_load(firmware_info *fw_info,
			      struct aicbt_patch_info_t *patch_info)
{
	int ret = 0;
	uint32_t ext_patch_nb = patch_info->ext_patch_nb;
	char ext_patch_file_name[100];
	int index = 0;
	uint32_t id = 0;
	uint32_t addr = 0;

	if (ext_patch_nb > 0) {
		printk("%s [0x40506004]: 0x04318000\r\n", __func__);
		ret = rwnx_send_dbg_mem_write_req(fw_info, 0x40506004,
						  0x04318000);
		printk("%s [0x40506004]: 0x04338000\r\n", __func__);
		ret = rwnx_send_dbg_mem_write_req(fw_info, 0x40506004,
						  0x04338000);

		if (ret < 0) {
			printk("%s set dc enable fail\r\n", __func__);
			return ret;
		}

		for (index = 0; index < patch_info->ext_patch_nb; index++) {
			id = *(patch_info->ext_patch_param + (index * 2));
			addr = *(patch_info->ext_patch_param + (index * 2) + 1);
			memset(ext_patch_file_name, 0,
			       sizeof(ext_patch_file_name));
			sprintf(ext_patch_file_name, "%s%d.bin",
				aicbt_fw->bt_ext_patch, id);
			printk("%s ext_patch_file_name:%s ext_patch_id:%x ext_patch_addr:%x \r\n",
			       __func__, ext_patch_file_name, id, addr);

			if (download_data(fw_info, addr, ext_patch_file_name)) {
				ret = -1;
				break;
			}
		}
	} else {
		printk("%s nothing ext patch need to load \r\n", __func__);
	}
	return ret;
}

int aicbt_patch_trap_data_load(firmware_info *fw_info,
			       struct aicbt_patch_table *head)
{
	int ret_val;

	struct aicbt_patch_info_t patch_info = {
		.info_len = 0,
		.adid_addrinf = 0,
		.addr_adid = 0,
		.patch_addrinf = 0,
		.addr_patch = 0,
		.reset_addr = 0,
		.reset_val = 0,
		.adid_flag_addr = 0,
		.adid_flag = 0,
		.ext_patch_nb_addr = 0,
		.ext_patch_nb = 0,
	};
	if (head == NULL) {
		return -1;
	}

	patch_info.addr_patch =
		FW_RAM_PATCH_BASE_ADDR; //set default patch base addr
	aicbt_patch_info_unpack(&patch_info, head);

	if (download_data(fw_info, patch_info.addr_patch, aicbt_fw->bt_patch)) {
		printk("aic load patch fail %d\n", ret_val);
		return -1;
	}

	if (aicbt_ext_patch_data_load(fw_info, &patch_info)) {
		printk("aic load ext patch fail %d\n", ret_val);
		return -1;
	}

	return 0;
}

int download_patch(firmware_info *fw_info, int cached)
{
	int ret_val = 0;
	struct aicbt_patch_table *head;

	printk("%s: Download fw patch start, cached %d", __func__, cached);

	if (!fw_info) {
		printk("%s: No patch entry exists(fw_info %p)", __func__,
		       fw_info);
		ret_val = -1;
		goto end;
	}

	ret_val = fw_config(fw_info);
	if (ret_val) {
		printk("%s: fw config failed %d", __func__, ret_val);
		goto free;
	}

	ret_val = system_config(fw_info);
	if (ret_val) {
		printk("%s: system config failed %d", __func__, ret_val);
		goto free;
	}

	/*
     * step1: check firmware statis
     * step2: download firmware if updated
     */

	ret_val = check_fw_status(fw_info);

	if (ret_val) {
		switch (sub_chip_id) {
		case DC_U01:
		case DC_U02:
		case DC_U02H:
			aicbt_fw = &fw_8800dc[sub_chip_id];
			printk("%s aicbt_fw desc:%s \r\n", __func__,
			       aicbt_fw->desc);
			break;
		default:
			printk("%s unsupported sub_chip_id %x\n", __func__,
			       sub_chip_id);
			goto free;
			break;
		}

		//step 1 get patch table data
		head = aicbt_patch_table_alloc(aicbt_fw->bt_table);
		if (head == NULL) {
			printk("aicbt_patch_table_alloc fail\n");
			goto free;
		}
		//step 2 unpack table data and download patch , ext patch
		ret_val = aicbt_patch_trap_data_load(fw_info, head);
		if (ret_val) {
			printk("aicbt_patch_trap_data_load fail\n");
			goto free;
		}

		//step 3 download patch table
		ret_val = patch_table_load(fw_info, head);
		if (ret_val) {
			printk("aic load patch table fail\n");
			goto free;
		}
		printk("%s download complete\r\n", __func__);
	}

free:
	/* Free fw data after download finished */
	kfree(fw_info->fw_data);
	fw_info->fw_data = NULL;

end:
	return ret_val;
}

//for 8800dc end

firmware_info *firmware_info_init(struct usb_interface *intf)
{
	struct usb_device *udev = interface_to_usbdev(intf);
	firmware_info *fw_info;

	AICBT_DBG("%s: start", __func__);

	fw_info = kzalloc(sizeof(*fw_info), GFP_KERNEL);
	if (!fw_info)
		return NULL;

	fw_info->send_pkt = kzalloc(SEND_PKT_LEN, GFP_KERNEL);
	if (!fw_info->send_pkt) {
		kfree(fw_info);
		return NULL;
	}

	fw_info->rcv_pkt = kzalloc(RCV_PKT_LEN, GFP_KERNEL);
	if (!fw_info->rcv_pkt) {
		kfree(fw_info->send_pkt);
		kfree(fw_info);
		return NULL;
	}

	fw_info->intf = intf;
	fw_info->udev = udev;
	fw_info->pipe_in = usb_rcvintpipe(fw_info->udev, INTR_EP);
	fw_info->pipe_out = usb_sndctrlpipe(fw_info->udev, CTRL_EP);
	fw_info->cmd_hdr = (struct hci_command_hdr *)(fw_info->send_pkt);
	fw_info->evt_hdr = (struct hci_event_hdr *)(fw_info->rcv_pkt);
	fw_info->cmd_cmp =
		(struct hci_ev_cmd_complete *)(fw_info->rcv_pkt + EVT_HDR_LEN);
	fw_info->req_para = fw_info->send_pkt + CMD_HDR_LEN;
	fw_info->rsp_para = fw_info->rcv_pkt + EVT_HDR_LEN + CMD_CMP_LEN;

#if BTUSB_RPM
	AICBT_INFO("%s: Auto suspend is enabled", __func__);
	usb_enable_autosuspend(udev);
	pm_runtime_set_autosuspend_delay(&(udev->dev), 2000);
#else
	AICBT_INFO("%s: Auto suspend is disabled", __func__);
	usb_disable_autosuspend(udev);
#endif

#if BTUSB_WAKEUP_HOST
	device_wakeup_enable(&udev->dev);
#endif

	return fw_info;
}

void firmware_info_destroy(struct usb_interface *intf)
{
	firmware_info *fw_info;
	struct usb_device *udev;
	struct btusb_data *data;

	udev = interface_to_usbdev(intf);
	data = usb_get_intfdata(intf);

	fw_info = data->fw_info;
	if (!fw_info)
		return;

#if BTUSB_RPM
	usb_disable_autosuspend(udev);
#endif

	/*
     * In order to reclaim fw data mem, we free fw_data immediately
     * after download patch finished instead of here.
     */
	kfree(fw_info->rcv_pkt);
	kfree(fw_info->send_pkt);
	kfree(fw_info);
}

static struct usb_driver btusb_driver;

static struct usb_device_id btusb_table[] = {
	{ USB_DEVICE_AND_INTERFACE_INFO(
		USB_VENDOR_ID_AIC, USB_PRODUCT_ID_AIC8801, 0xe0, 0x01, 0x01) },
	{ USB_DEVICE_AND_INTERFACE_INFO(USB_VENDOR_ID_AIC,
					USB_PRODUCT_ID_AIC8800D80, 0xe0, 0x01,
					0x01) },
	{ USB_DEVICE_AND_INTERFACE_INFO(USB_VENDOR_ID_AIC,
					USB_PRODUCT_ID_AIC8800DC, 0xe0, 0x01,
					0x01) },
	{ USB_DEVICE_AND_INTERFACE_INFO(USB_VENDOR_ID_AIC_V2,
					USB_PRODUCT_ID_AIC8800D80X2, 0xe0, 0x01,
					0x01) },
	{}
};

MODULE_DEVICE_TABLE(usb, btusb_table);

static int inc_tx(struct btusb_data *data)
{
	unsigned long flags;
	int rv;

	spin_lock_irqsave(&data->txlock, flags);
	rv = test_bit(BTUSB_SUSPENDING, &data->flags);
	if (!rv)
		data->tx_in_flight++;
	spin_unlock_irqrestore(&data->txlock, flags);

	return rv;
}

void check_sco_event(struct urb *urb)
{
	u8 *opcode = (u8 *)(urb->transfer_buffer);
	u8 status;
	static uint16_t sco_handle = 0;
	uint16_t handle;
	u8 air_mode = 0;
	struct hci_dev *hdev = urb->context;

	switch (*opcode) {
	case HCI_EV_SYNC_CONN_COMPLETE:
		AICBT_INFO("%s: HCI_EV_SYNC_CONN_COMPLETE(0x%02x)", __func__,
			   *opcode);
		status = *(opcode + 2);
		sco_handle = *(opcode + 3) | *(opcode + 4) << 8;
		air_mode = *(opcode + 18);
		printk("%s status:%d,air_mode:%d \r\n", __func__, status,
		       air_mode);
		if (status == 0) {
			SCO_NUM++;
			hdev->notify(hdev, 0);
			//schedule_work(&data->work);
			if (air_mode == 0x03) {
				set_select_msbc(CODEC_MSBC);
			}
		}
		break;
	case HCI_EV_DISCONN_COMPLETE:
		AICBT_INFO("%s: HCI_EV_DISCONN_COMPLETE(0x%02x)", __func__,
			   *opcode);
		status = *(opcode + 2);
		handle = *(opcode + 3) | *(opcode + 4) << 8;
		if (status == 0 && sco_handle == handle) {
			SCO_NUM--;
			hdev->notify(hdev, 0);
			set_select_msbc(CODEC_CVSD);
			//schedule_work(&data->work);
		}
		break;
	default:
		AICBT_DBG("%s: event 0x%02x", __func__, *opcode);
		break;
	}
}

static inline void btusb_free_frags(struct btusb_data *data)
{
	unsigned long flags;

	spin_lock_irqsave(&data->rxlock, flags);

	kfree_skb(data->evt_skb);
	data->evt_skb = NULL;

	kfree_skb(data->acl_skb);
	data->acl_skb = NULL;

	kfree_skb(data->sco_skb);
	data->sco_skb = NULL;

	spin_unlock_irqrestore(&data->rxlock, flags);
}

static int btusb_recv_intr(struct btusb_data *data, void *buffer, int count)
{
	struct sk_buff *skb;
	int err = 0;

	spin_lock(&data->rxlock);
	skb = data->evt_skb;
	//printk("%s count %d\n", __func__, count);

	while (count) {
		int len;

		if (!skb) {
			skb = bt_skb_alloc(HCI_MAX_EVENT_SIZE, GFP_ATOMIC);
			if (!skb) {
				err = -ENOMEM;
				break;
			}

			bt_cb(skb)->pkt_type = HCI_EVENT_PKT;
			bt_cb(skb)->expect = HCI_EVENT_HDR_SIZE;
		}

		len = min_t(uint, bt_cb(skb)->expect, count);
		memcpy(skb_put(skb, len), buffer, len);

		count -= len;
		buffer += len;
		bt_cb(skb)->expect -= len;

		if (skb->len == HCI_EVENT_HDR_SIZE) {
			/* Complete event header */
			bt_cb(skb)->expect = hci_event_hdr(skb)->plen;

			if (skb_tailroom(skb) < bt_cb(skb)->expect) {
				kfree_skb(skb);
				skb = NULL;

				err = -EILSEQ;
				break;
			}
		}

		if (bt_cb(skb)->expect == 0) {
			/* Complete frame */
			hci_recv_frame(data->hdev, skb);
			skb = NULL;
		}
	}

	data->evt_skb = skb;
	spin_unlock(&data->rxlock);

	return err;
}

static int btusb_recv_bulk(struct btusb_data *data, void *buffer, int count)
{
	struct sk_buff *skb;
	int err = 0;

	spin_lock(&data->rxlock);
	skb = data->acl_skb;

	while (count) {
		int len;

		if (!skb) {
			skb = bt_skb_alloc(HCI_MAX_FRAME_SIZE, GFP_ATOMIC);
			if (!skb) {
				err = -ENOMEM;
				break;
			}

			bt_cb(skb)->pkt_type = HCI_ACLDATA_PKT;
			bt_cb(skb)->expect = HCI_ACL_HDR_SIZE;
		}

		len = min_t(uint, bt_cb(skb)->expect, count);
		memcpy(skb_put(skb, len), buffer, len);

		count -= len;
		buffer += len;
		bt_cb(skb)->expect -= len;

		if (skb->len == HCI_ACL_HDR_SIZE) {
			__le16 dlen = hci_acl_hdr(skb)->dlen;

			/* Complete ACL header */
			bt_cb(skb)->expect = __le16_to_cpu(dlen);

			if (skb_tailroom(skb) < bt_cb(skb)->expect) {
				kfree_skb(skb);
				skb = NULL;

				err = -EILSEQ;
				break;
			}
		}

		if (bt_cb(skb)->expect == 0) {
			/* Complete frame */
			hci_recv_frame(data->hdev, skb);
			skb = NULL;
		}
	}

	data->acl_skb = skb;
	spin_unlock(&data->rxlock);

	return err;
}

static int btusb_recv_isoc(struct btusb_data *data, void *buffer, int count)
{
	struct sk_buff *skb;
	int err = 0;

	spin_lock(&data->rxlock);
	skb = data->sco_skb;

	while (count) {
		int len;

		if (!skb) {
			skb = bt_skb_alloc(HCI_MAX_SCO_SIZE, GFP_ATOMIC);
			if (!skb) {
				err = -ENOMEM;
				break;
			}

			bt_cb(skb)->pkt_type = HCI_SCODATA_PKT;
			bt_cb(skb)->expect = HCI_SCO_HDR_SIZE;
		}

		len = min_t(uint, bt_cb(skb)->expect, count);
		memcpy(skb_put(skb, len), buffer, len);

		count -= len;
		buffer += len;
		bt_cb(skb)->expect -= len;

		if (skb->len == HCI_SCO_HDR_SIZE) {
			/* Complete SCO header */
			bt_cb(skb)->expect = hci_sco_hdr(skb)->dlen;

			if (skb_tailroom(skb) < bt_cb(skb)->expect) {
				kfree_skb(skb);
				skb = NULL;

				err = -EILSEQ;
				break;
			}
		}

		if (bt_cb(skb)->expect == 0) {
			/* Complete frame */
			hci_recv_frame(data->hdev, skb);
			skb = NULL;
		}
	}

	data->sco_skb = skb;
	spin_unlock(&data->rxlock);

	return err;
}

static void btusb_intr_complete(struct urb *urb)
{
	struct hci_dev *hdev = urb->context;
	struct btusb_data *data = GET_DRV_DATA(hdev);
	int err;

	AICBT_DBG("%s: urb %p status %d count %d ", __func__, urb, urb->status,
		  urb->actual_length);

	if (!test_bit(HCI_RUNNING, &hdev->flags)) {
		printk("%s return \n", __func__);
		return;
	}
	if (urb->status == 0) {
		hdev->stat.byte_rx += urb->actual_length;
		if (btusb_recv_intr(data, urb->transfer_buffer,
				    urb->actual_length) < 0) {
			AICBT_ERR("%s corrupted event packet", hdev->name);
			hdev->stat.err_rx++;
		}
		check_sco_event(urb);

	}
	/* Avoid suspend failed when usb_kill_urb */
	else if (urb->status == -ENOENT) {
		return;
	}

	if (!test_bit(BTUSB_INTR_RUNNING, &data->flags))
		return;

	usb_mark_last_busy(data->udev);
	usb_anchor_urb(urb, &data->intr_anchor);

	err = usb_submit_urb(urb, GFP_ATOMIC);
	if (err < 0) {
		if (err != -EPERM && err != -ENODEV)
			AICBT_ERR("%s: Failed to re-submit urb %p, err %d",
				  __func__, urb, err);
		usb_unanchor_urb(urb);
	}
}

static int btusb_submit_intr_urb(struct hci_dev *hdev, gfp_t mem_flags)
{
	struct btusb_data *data = GET_DRV_DATA(hdev);
	struct urb *urb;
	unsigned char *buf;
	unsigned int pipe;
	int err, size;

	if (!data->intr_ep)
		return -ENODEV;

	urb = usb_alloc_urb(0, mem_flags);
	if (!urb)
		return -ENOMEM;

	size = le16_to_cpu(data->intr_ep->wMaxPacketSize);

	buf = kmalloc(size, mem_flags);
	if (!buf) {
		usb_free_urb(urb);
		return -ENOMEM;
	}

	AICBT_DBG("%s: mMaxPacketSize %d, bEndpointAddress 0x%02x", __func__,
		  size, data->intr_ep->bEndpointAddress);

	pipe = usb_rcvintpipe(data->udev, data->intr_ep->bEndpointAddress);

	usb_fill_int_urb(urb, data->udev, pipe, buf, size, btusb_intr_complete,
			 hdev, data->intr_ep->bInterval);

	urb->transfer_flags |= URB_FREE_BUFFER;

	usb_anchor_urb(urb, &data->intr_anchor);

	err = usb_submit_urb(urb, mem_flags);
	if (err < 0) {
		AICBT_ERR("%s: Failed to submit urb %p, err %d", __func__, urb,
			  err);
		usb_unanchor_urb(urb);
	}

	usb_free_urb(urb);

	return err;
}

static void btusb_bulk_complete(struct urb *urb)
{
	struct hci_dev *hdev = urb->context;
	struct btusb_data *data = GET_DRV_DATA(hdev);
	int err;

	AICBT_DBG("%s: urb %p status %d count %d", __func__, urb, urb->status,
		  urb->actual_length);

	if (!test_bit(HCI_RUNNING, &hdev->flags)) {
		printk("%s HCI_RUNNING\n", __func__);
		return;
	}
	if (urb->status == 0) {
		hdev->stat.byte_rx += urb->actual_length;
		if (data->recv_bulk(data, urb->transfer_buffer,
				    urb->actual_length) < 0) {
			AICBT_ERR("%s Corrupted ACL packet", hdev->name);
			hdev->stat.err_rx++;
		}

	}
	/* Avoid suspend failed when usb_kill_urb */
	else if (urb->status == -ENOENT) {
		printk("%s ENOENT\n", __func__);
		return;
	}
	AICBT_DBG("%s: OUT", __func__);

	if (!test_bit(BTUSB_BULK_RUNNING, &data->flags)) {
		printk("%s BTUSB_BULK_RUNNING\n", __func__);
		return;
	}
	usb_anchor_urb(urb, &data->bulk_anchor);
	usb_mark_last_busy(data->udev);

	//printk("LIULI bulk submit\n");
	err = usb_submit_urb(urb, GFP_ATOMIC);
	if (err < 0) {
		/* -EPERM: urb is being killed;
         * -ENODEV: device got disconnected */
		if (err != -EPERM && err != -ENODEV)
			AICBT_ERR(
				"btusb_bulk_complete %s urb %p failed to resubmit (%d)",
				hdev->name, urb, -err);
		usb_unanchor_urb(urb);
	}
}

static int btusb_submit_bulk_urb(struct hci_dev *hdev, gfp_t mem_flags)
{
	struct btusb_data *data = GET_DRV_DATA(hdev);
	struct urb *urb;
	unsigned char *buf;
	unsigned int pipe;
	int err, size = HCI_MAX_FRAME_SIZE;

	AICBT_DBG("%s: hdev name %s", __func__, hdev->name);
	AICBT_DBG("%s: mMaxPacketSize %d, bEndpointAddress 0x%02x", __func__,
		  size, data->bulk_rx_ep->bEndpointAddress);

	if (!data->bulk_rx_ep)
		return -ENODEV;

	urb = usb_alloc_urb(0, mem_flags);
	if (!urb)
		return -ENOMEM;

	buf = kmalloc(size, mem_flags);
	if (!buf) {
		usb_free_urb(urb);
		return -ENOMEM;
	}

	pipe = usb_rcvbulkpipe(data->udev, data->bulk_rx_ep->bEndpointAddress);

	usb_fill_bulk_urb(urb, data->udev, pipe, buf, size, btusb_bulk_complete,
			  hdev);

	urb->transfer_flags |= URB_FREE_BUFFER;

	usb_mark_last_busy(data->udev);
	usb_anchor_urb(urb, &data->bulk_anchor);

	err = usb_submit_urb(urb, mem_flags);
	if (err < 0) {
		AICBT_ERR("%s: Failed to submit urb %p, err %d", __func__, urb,
			  err);
		usb_unanchor_urb(urb);
	}

	usb_free_urb(urb);

	return err;
}

static void btusb_isoc_complete(struct urb *urb)
{
	struct hci_dev *hdev = urb->context;
	struct btusb_data *data = GET_DRV_DATA(hdev);
	int i, err;
	unsigned int total_length = 0;

	AICBT_DBG("%s: urb %p status %d count %d", __func__, urb, urb->status,
		  urb->actual_length);

	if (!test_bit(HCI_RUNNING, &hdev->flags))
		return;

	if (urb->status == 0) {
		for (i = 0; i < urb->number_of_packets; i++) {
			unsigned int offset = urb->iso_frame_desc[i].offset;
			unsigned int length =
				urb->iso_frame_desc[i].actual_length;
			//u8 *data = (u8 *)(urb->transfer_buffer + offset);
			//AICBT_DBG("%d,%d ,%x,%x,%x  s %d.",
			//offset, length, data[0], data[1],data[2],urb->iso_frame_desc[i].status);

			if (total_length >= urb->actual_length) {
				AICBT_ERR("total_len >= actual_length ,return");
				break;
			}
			total_length += length;

			if (urb->iso_frame_desc[i].status)
				continue;

			hdev->stat.byte_rx += length;
			if (length) {
				if (btusb_recv_isoc(
					    data, urb->transfer_buffer + offset,
					    length) < 0) {
					AICBT_ERR("%s corrupted SCO packet",
						  hdev->name);
					hdev->stat.err_rx++;
				}
			}
		}
	}
	/* Avoid suspend failed when usb_kill_urb */
	else if (urb->status == -ENOENT) {
		return;
	}

	if (!test_bit(BTUSB_ISOC_RUNNING, &data->flags))
		return;

	usb_anchor_urb(urb, &data->isoc_anchor);
	i = 0;
retry:
	err = usb_submit_urb(urb, GFP_ATOMIC);
	if (err < 0) {
		/* -EPERM: urb is being killed;
         * -ENODEV: device got disconnected */
		if (err != -EPERM && err != -ENODEV)
			AICBT_ERR(
				"%s: Failed to re-sumbit urb %p, retry %d, err %d",
				__func__, urb, i, err);
		if (i < 10) {
			i++;
			mdelay(1);
			goto retry;
		}

		usb_unanchor_urb(urb);
	}
}

static inline void fill_isoc_descriptor(struct urb *urb, int len, int mtu)
{
	int i, offset = 0;

	AICBT_DBG("%s: len %d mtu %d", __func__, len, mtu);

	for (i = 0; i < BTUSB_MAX_ISOC_FRAMES && len >= mtu;
	     i++, offset += mtu, len -= mtu) {
		urb->iso_frame_desc[i].offset = offset;
		urb->iso_frame_desc[i].length = mtu;
	}

	if (len && i < BTUSB_MAX_ISOC_FRAMES) {
		urb->iso_frame_desc[i].offset = offset;
		urb->iso_frame_desc[i].length = len;
		i++;
	}

	urb->number_of_packets = i;
}

static int btusb_submit_isoc_urb(struct hci_dev *hdev, gfp_t mem_flags)
{
	struct btusb_data *data = GET_DRV_DATA(hdev);
	struct urb *urb;
	unsigned char *buf;
	unsigned int pipe;
	int err, size;
	int interval;

	if (!data->isoc_rx_ep)
		return -ENODEV;
	AICBT_DBG("%s: mMaxPacketSize %d, bEndpointAddress 0x%02x", __func__,
		  size, data->isoc_rx_ep->bEndpointAddress);

	urb = usb_alloc_urb(BTUSB_MAX_ISOC_FRAMES, mem_flags);
	if (!urb)
		return -ENOMEM;

	size = le16_to_cpu(data->isoc_rx_ep->wMaxPacketSize) *
	       BTUSB_MAX_ISOC_FRAMES;

	buf = kmalloc(size, mem_flags);
	if (!buf) {
		usb_free_urb(urb);
		return -ENOMEM;
	}

	pipe = usb_rcvisocpipe(data->udev, data->isoc_rx_ep->bEndpointAddress);

	urb->dev = data->udev;
	urb->pipe = pipe;
	urb->context = hdev;
	urb->complete = btusb_isoc_complete;
	if (urb->dev->speed == USB_SPEED_HIGH ||
	    urb->dev->speed >= USB_SPEED_SUPER) {
		/* make sure interval is within allowed range */
		interval = clamp((int)data->isoc_rx_ep->bInterval, 1, 16);
		urb->interval = 1 << (interval - 1);
	} else {
		urb->interval = data->isoc_rx_ep->bInterval;
	}

	AICBT_INFO("urb->interval %d \r\n", urb->interval);

	urb->transfer_flags = URB_FREE_BUFFER | URB_ISO_ASAP;
	urb->transfer_buffer = buf;
	urb->transfer_buffer_length = size;

	fill_isoc_descriptor(urb, size,
			     le16_to_cpu(data->isoc_rx_ep->wMaxPacketSize));

	usb_anchor_urb(urb, &data->isoc_anchor);

	err = usb_submit_urb(urb, mem_flags);
	if (err < 0) {
		AICBT_ERR("%s: Failed to submit urb %p, err %d", __func__, urb,
			  err);
		usb_unanchor_urb(urb);
	}

	usb_free_urb(urb);

	return err;
}

static void btusb_tx_complete(struct urb *urb)
{
	struct sk_buff *skb = urb->context;
	struct hci_dev *hdev = (struct hci_dev *)skb->dev;
	struct btusb_data *data = GET_DRV_DATA(hdev);

	if (!test_bit(HCI_RUNNING, &hdev->flags))
		goto done;

	if (!urb->status)
		hdev->stat.byte_tx += urb->transfer_buffer_length;
	else
		hdev->stat.err_tx++;

done:
	spin_lock(&data->txlock);
	data->tx_in_flight--;
	spin_unlock(&data->txlock);

	kfree(urb->setup_packet);

	kfree_skb(skb);
}

static void btusb_isoc_tx_complete(struct urb *urb)
{
	struct sk_buff *skb = urb->context;
	struct hci_dev *hdev = (struct hci_dev *)skb->dev;

	AICBT_DBG("%s: urb %p status %d count %d", __func__, urb, urb->status,
		  urb->actual_length);

	if (skb && hdev) {
		if (!test_bit(HCI_RUNNING, &hdev->flags))
			goto done;

		if (!urb->status)
			hdev->stat.byte_tx += urb->transfer_buffer_length;
		else
			hdev->stat.err_tx++;
	} else
		AICBT_ERR("%s: skb 0x%p hdev 0x%p", __func__, skb, hdev);

done:
	kfree(urb->setup_packet);

	kfree_skb(skb);
}

static int btusb_shutdown(struct hci_dev *hdev)
{
	struct sk_buff *skb;
	printk("aic %s\n", __func__);

	skb = __hci_cmd_sync(hdev, HCI_OP_RESET, 0, NULL, HCI_INIT_TIMEOUT);
	if (IS_ERR(skb)) {
		printk("HCI reset during shutdown failed\n");
		return PTR_ERR(skb);
	}
	kfree_skb(skb);

	return 0;
}

static int btusb_open(struct hci_dev *hdev)
{
	struct btusb_data *data = GET_DRV_DATA(hdev);
	int err = 0;

	AICBT_INFO("%s: Start", __func__);

	err = usb_autopm_get_interface(data->intf);
	if (err < 0)
		return err;

	data->intf->needs_remote_wakeup = 1;

	//err = download_patch(data->fw_info,1);
	printk(" download_patch %d", err);
	if (err < 0) {
		goto failed;
	}

	if (test_and_set_bit(HCI_RUNNING, &hdev->flags)) {
		goto done;
	}

	if (test_and_set_bit(BTUSB_INTR_RUNNING, &data->flags)) {
		goto done;
	}

	err = btusb_submit_intr_urb(hdev, GFP_KERNEL);
	if (err < 0)
		goto failed;

	err = btusb_submit_bulk_urb(hdev, GFP_KERNEL);
	if (err < 0) {
		mdelay(URB_CANCELING_DELAY_MS);
		usb_kill_anchored_urbs(&data->intr_anchor);
		goto failed;
	}

	set_bit(BTUSB_BULK_RUNNING, &data->flags);
	btusb_submit_bulk_urb(hdev, GFP_KERNEL);

done:
	usb_autopm_put_interface(data->intf);
	AICBT_INFO("%s: End", __func__);
	return 0;

failed:
	clear_bit(BTUSB_INTR_RUNNING, &data->flags);
	clear_bit(HCI_RUNNING, &hdev->flags);
	usb_autopm_put_interface(data->intf);
	AICBT_ERR("%s: Failed", __func__);
	return err;
}

static void btusb_stop_traffic(struct btusb_data *data)
{
	mdelay(URB_CANCELING_DELAY_MS);
	usb_kill_anchored_urbs(&data->intr_anchor);
	usb_kill_anchored_urbs(&data->bulk_anchor);
	usb_kill_anchored_urbs(&data->isoc_anchor);
}

static int btusb_close(struct hci_dev *hdev)
{
	struct btusb_data *data = GET_DRV_DATA(hdev);
	int err;

	AICBT_INFO("%s: hci running %lu", __func__, hdev->flags & HCI_RUNNING);

	if (!test_and_clear_bit(HCI_RUNNING, &hdev->flags)) {
		return 0;
	}

	if (!test_and_clear_bit(BTUSB_INTR_RUNNING, &data->flags)) {
		return 0;
	}

	cancel_work_sync(&data->work);
	cancel_work_sync(&data->waker);

	clear_bit(BTUSB_ISOC_RUNNING, &data->flags);
	clear_bit(BTUSB_BULK_RUNNING, &data->flags);
	clear_bit(BTUSB_INTR_RUNNING, &data->flags);

	btusb_stop_traffic(data);
	err = usb_autopm_get_interface(data->intf);
	if (err < 0)
		goto failed;

	data->intf->needs_remote_wakeup = 0;
	usb_autopm_put_interface(data->intf);

failed:
	mdelay(URB_CANCELING_DELAY_MS);
	usb_scuttle_anchored_urbs(&data->deferred);
	return 0;
}

static int btusb_flush(struct hci_dev *hdev)
{
	struct btusb_data *data = GET_DRV_DATA(hdev);

	AICBT_DBG("%s", __func__);

	mdelay(URB_CANCELING_DELAY_MS);
	usb_kill_anchored_urbs(&data->tx_anchor);

	return 0;
}

int btusb_send_frame(struct hci_dev *hdev, struct sk_buff *skb)
{
	//struct hci_dev *hdev = (struct hci_dev *) skb->dev;

	struct btusb_data *data = GET_DRV_DATA(hdev);
	struct usb_ctrlrequest *dr;
	struct urb *urb;
	unsigned int pipe;
	int err = 0;
	int retries = 0;
	u16 *opcode = NULL;

	AICBT_DBG("%s: hdev %p, btusb data %p, pkt type %d", __func__, hdev,
		  data, bt_cb(skb)->pkt_type);

	//printk("aic %d %d\r\n", bt_cb(skb)->pkt_type, skb->len);
	if (!test_bit(HCI_RUNNING, &hdev->flags))
		return -EBUSY;

	skb->dev = (void *)hdev;

	switch (bt_cb(skb)->pkt_type) {
	case HCI_COMMAND_PKT:
		print_command(skb);
		urb = usb_alloc_urb(0, GFP_ATOMIC);
		if (!urb)
			return -ENOMEM;

		dr = kmalloc(sizeof(*dr), GFP_ATOMIC);
		if (!dr) {
			usb_free_urb(urb);
			return -ENOMEM;
		}

		dr->bRequestType = data->cmdreq_type;
		dr->bRequest = 0;
		dr->wIndex = 0;
		dr->wValue = 0;
		dr->wLength = __cpu_to_le16(skb->len);

		pipe = usb_sndctrlpipe(data->udev, 0x00);

		usb_fill_control_urb(urb, data->udev, pipe, (void *)dr,
				     skb->data, skb->len, btusb_tx_complete,
				     skb);

		hdev->stat.cmd_tx++;
		break;

	case HCI_ACLDATA_PKT:
		if (bt_cb(skb)->pkt_type == HCI_COMMAND_PKT) {
			print_command(skb);
			opcode = (u16 *)(skb->data);
			printk("aic cmd:0x%04x", *opcode);
		} else {
			print_acl(skb, 1);
		}
		if (!data->bulk_tx_ep)
			return -ENODEV;

		urb = usb_alloc_urb(0, GFP_ATOMIC);
		if (!urb)
			return -ENOMEM;

		pipe = usb_sndbulkpipe(data->udev,
				       data->bulk_tx_ep->bEndpointAddress);

		usb_fill_bulk_urb(urb, data->udev, pipe, skb->data, skb->len,
				  btusb_tx_complete, skb);
		urb->transfer_flags |= URB_ZERO_PACKET;

		hdev->stat.acl_tx++;
		break;

	case HCI_SCODATA_PKT:
		print_sco(skb, 1);
		if (!data->isoc_tx_ep || !SCO_NUM) {
			kfree(skb);
			return -ENODEV;
		}

		urb = usb_alloc_urb(BTUSB_MAX_ISOC_FRAMES, GFP_ATOMIC);
		if (!urb) {
			AICBT_ERR("%s: Failed to allocate mem for sco pkts",
				  __func__);
			kfree(skb);
			return -ENOMEM;
		}

		pipe = usb_sndisocpipe(data->udev,
				       data->isoc_tx_ep->bEndpointAddress);

		usb_fill_int_urb(urb, data->udev, pipe, skb->data, skb->len,
				 btusb_isoc_tx_complete, skb,
				 data->isoc_tx_ep->bInterval);

		urb->transfer_flags = URB_ISO_ASAP;

		fill_isoc_descriptor(
			urb, skb->len,
			le16_to_cpu(data->isoc_tx_ep->wMaxPacketSize));

		hdev->stat.sco_tx++;
		goto skip_waking;

	default:
		return -EILSEQ;
	}

	err = inc_tx(data);
	if (err) {
		usb_anchor_urb(urb, &data->deferred);
		schedule_work(&data->waker);
		err = 0;
		goto done;
	}

skip_waking:
	usb_anchor_urb(urb, &data->tx_anchor);
retry:
	err = usb_submit_urb(urb, GFP_ATOMIC);
	if (err < 0) {
		AICBT_ERR(
			"%s: Failed to submit urb %p, pkt type %d, err %d, retries %d",
			__func__, urb, bt_cb(skb)->pkt_type, err, retries);
		if ((bt_cb(skb)->pkt_type != HCI_SCODATA_PKT) &&
		    (retries < 10)) {
			mdelay(1);

			if (bt_cb(skb)->pkt_type == HCI_COMMAND_PKT)
				print_error_command(skb);
			retries++;
			goto retry;
		}
		kfree(urb->setup_packet);
		usb_unanchor_urb(urb);
	} else {
		usb_mark_last_busy(data->udev);
	}
	usb_free_urb(urb);

done:
	return err;
}

static void btusb_notify(struct hci_dev *hdev, unsigned int evt)
{
	struct btusb_data *data = GET_DRV_DATA(hdev);

	AICBT_DBG("%s: name %s, evt %d", __func__, hdev->name, evt);

	if (SCO_NUM != data->sco_num) {
		data->sco_num = SCO_NUM;
		schedule_work(&data->work);
	}
}

static inline int set_isoc_interface(struct hci_dev *hdev, int altsetting)
{
	struct btusb_data *data = GET_DRV_DATA(hdev);
	struct usb_interface *intf = data->isoc;
	struct usb_endpoint_descriptor *ep_desc;
	int i, err;

	if (!data->isoc)
		return -ENODEV;

	err = usb_set_interface(data->udev, 1, altsetting);
	if (err < 0) {
		AICBT_ERR("%s: Failed to set interface, altsetting %d, err %d",
			  __func__, altsetting, err);
		return err;
	}

	data->isoc_altsetting = altsetting;

	data->isoc_tx_ep = NULL;
	data->isoc_rx_ep = NULL;

	for (i = 0; i < intf->cur_altsetting->desc.bNumEndpoints; i++) {
		ep_desc = &intf->cur_altsetting->endpoint[i].desc;

		if (!data->isoc_tx_ep && usb_endpoint_is_isoc_out(ep_desc)) {
			data->isoc_tx_ep = ep_desc;
			continue;
		}

		if (!data->isoc_rx_ep && usb_endpoint_is_isoc_in(ep_desc)) {
			data->isoc_rx_ep = ep_desc;
			continue;
		}
	}

	if (!data->isoc_tx_ep || !data->isoc_rx_ep) {
		AICBT_ERR("%s: Invalid SCO descriptors", __func__);
		return -ENODEV;
	}

	AICBT_ERR("%s: hdev->reassembly implemant\r\n", __func__);

	return 0;
}

static void set_select_msbc(enum CODEC_TYPE type)
{
	printk("%s codec type = %d", __func__, (int)type);
	codec_type = type;
}

static enum CODEC_TYPE check_select_msbc(void)
{
	return codec_type;
}

static void btusb_work(struct work_struct *work)
{
	struct btusb_data *data = container_of(work, struct btusb_data, work);
	struct hci_dev *hdev = data->hdev;
	int err;
	int new_alts;
	printk("%s data->sco_num:%d \r\n", __func__, data->sco_num);

	if (data->sco_num > 0) {
		if (!test_bit(BTUSB_DID_ISO_RESUME, &data->flags)) {
			err = usb_autopm_get_interface(data->isoc ? data->isoc :
								    data->intf);
			if (err < 0) {
				clear_bit(BTUSB_ISOC_RUNNING, &data->flags);
				mdelay(URB_CANCELING_DELAY_MS);
				usb_kill_anchored_urbs(&data->isoc_anchor);
				printk("%s usb_kill_anchored_urbs after \r\n",
				       __func__);
				return;
			}

			set_bit(BTUSB_DID_ISO_RESUME, &data->flags);
		}

		AICBT_INFO("%s voice settings = 0x%04x", __func__,
			   hdev->voice_setting);
		if (!(hdev->voice_setting & 0x0003)) {
			if (data->sco_num == 1)
				if (check_select_msbc()) {
					new_alts = 1;
				} else {
					new_alts = 2;
				}
			else {
				AICBT_INFO(
					"%s: we don't support mutiple sco link for cvsd",
					__func__);
				return;
			}
		} else {
			if (check_select_msbc()) {
				if (data->sco_num == 1)
					new_alts = 1;
				else {
					AICBT_INFO(
						"%s: we don't support mutiple sco link for msbc",
						__func__);
					return;
				}
			} else {
				new_alts = 2;
			}
		}
		if (data->isoc_altsetting != new_alts) {
			clear_bit(BTUSB_ISOC_RUNNING, &data->flags);
			mdelay(URB_CANCELING_DELAY_MS);
			usb_kill_anchored_urbs(&data->isoc_anchor);

			printk("%s set_isoc_interface in \r\n", __func__);
			if (set_isoc_interface(hdev, new_alts) < 0)
				return;
		}

		printk("%s set_isoc_interface out \r\n", __func__);

		if (!test_and_set_bit(BTUSB_ISOC_RUNNING, &data->flags)) {
			printk("%s btusb_submit_isoc_urb\r\n", __func__);
			if (btusb_submit_isoc_urb(hdev, GFP_KERNEL) < 0)
				clear_bit(BTUSB_ISOC_RUNNING, &data->flags);
			else
				btusb_submit_isoc_urb(hdev, GFP_KERNEL);
		}
	} else {
		AICBT_INFO("%s: data->sco_num :%d", __func__, data->sco_num);
		clear_bit(BTUSB_ISOC_RUNNING, &data->flags);
		mdelay(URB_CANCELING_DELAY_MS);
		usb_kill_anchored_urbs(&data->isoc_anchor);

		set_isoc_interface(hdev, 0);
		if (test_and_clear_bit(BTUSB_DID_ISO_RESUME, &data->flags))
			usb_autopm_put_interface(data->isoc ? data->isoc :
							      data->intf);
	}
}

static void btusb_waker(struct work_struct *work)
{
	struct btusb_data *data = container_of(work, struct btusb_data, waker);
	int err;

	AICBT_DBG("%s", __func__);

	err = usb_autopm_get_interface(data->intf);
	if (err < 0)
		return;

	usb_autopm_put_interface(data->intf);
}

int bt_pm_notify(struct notifier_block *notifier, ulong pm_event, void *unused)
{
	struct btusb_data *data;
	firmware_info *fw_info;
	struct usb_device *udev;

	AICBT_INFO("%s: pm event %ld", __func__, pm_event);

	data = container_of(notifier, struct btusb_data, pm_notifier);
	fw_info = data->fw_info;
	udev = fw_info->udev;

	switch (pm_event) {
	case PM_SUSPEND_PREPARE:
	case PM_HIBERNATION_PREPARE:
		if (!device_may_wakeup(&udev->dev)) {
			fw_info->intf->needs_binding = 1;
			AICBT_INFO(
				"%s:remote wakeup not supported, binding needed",
				__func__);
		}
		break;

	case PM_POST_SUSPEND:
	case PM_POST_HIBERNATION:
	case PM_POST_RESTORE:

#if BTUSB_RPM
		usb_disable_autosuspend(udev);
		usb_enable_autosuspend(udev);
		pm_runtime_set_autosuspend_delay(&(udev->dev), 2000);
#endif
		break;

	default:
		break;
	}

	return NOTIFY_DONE;
}

int bt_reboot_notify(struct notifier_block *notifier, ulong pm_event,
		     void *unused)
{
	struct btusb_data *data;
	firmware_info *fw_info;
	struct usb_device *udev;

	AICBT_INFO("%s: pm event %ld", __func__, pm_event);

	data = container_of(notifier, struct btusb_data, reboot_notifier);
	fw_info = data->fw_info;
	udev = fw_info->udev;

	switch (pm_event) {
	case SYS_DOWN:
		AICBT_DBG("%s:system down or restart", __func__);
		break;

	case SYS_HALT:
	case SYS_POWER_OFF:
#if SUSPNED_DW_FW
		cancel_work_sync(&data->work);

		btusb_stop_traffic(data);
		mdelay(URB_CANCELING_DELAY_MS);
		usb_kill_anchored_urbs(&data->tx_anchor);

		if (fw_info_4_suspend) {
			download_suspend_patch(fw_info_4_suspend, 1);
		} else
			AICBT_ERR("%s: Failed to download suspend fw",
				  __func__);
#endif

#ifdef SET_WAKEUP_DEVICE
		set_wakeup_device_from_conf(fw_info_4_suspend);
#endif
		AICBT_DBG("%s:system halt or power off", __func__);
		break;

	default:
		break;
	}

	return NOTIFY_DONE;
}

static int aicwf_usb_chipmatch(u16 vid, u16 pid)
{
	if (pid == USB_PRODUCT_ID_AIC8801) {
		g_chipid = PRODUCT_ID_AIC8801;
		printk("%s USE AIC8801\r\n", __func__);
		return 0;
	} else if (pid == USB_PRODUCT_ID_AIC8800DC) {
		g_chipid = PRODUCT_ID_AIC8800DC;
		printk("%s USE AIC8800DC\r\n", __func__);
		return 0;
	} else if (pid == USB_PRODUCT_ID_AIC8800D80) {
		g_chipid = PRODUCT_ID_AIC8800D80;
		printk("%s USE AIC8800D80\r\n", __func__);
		return 0;
	} else if (pid == USB_PRODUCT_ID_AIC8800D80X2) {
		g_chipid = PRODUCT_ID_AIC8800D80X2;
		printk("%s USE AIC8800D80X2\r\n", __func__);
		return 0;
	} else {
		return -1;
	}
}

static int btusb_probe(struct usb_interface *intf,
		       const struct usb_device_id *id)
{
	struct usb_device *udev = interface_to_usbdev(intf);
	struct usb_endpoint_descriptor *ep_desc;
	u8 endpoint_num;
	struct btusb_data *data;
	struct hci_dev *hdev;
	firmware_info *fw_info;
	int i, err = 0;

	bt_support = 1;

	AICBT_INFO(
		"%s: usb_interface %p, bInterfaceNumber %d, idVendor 0x%04x, "
		"idProduct 0x%04x",
		__func__, intf, intf->cur_altsetting->desc.bInterfaceNumber,
		id->idVendor, id->idProduct);

	aicwf_usb_chipmatch(id->idVendor, id->idProduct);

	/* interface numbers are hardcoded in the spec */
	if (intf->cur_altsetting->desc.bInterfaceNumber != 0)
		return -ENODEV;

	AICBT_DBG("%s: can wakeup = %x, may wakeup = %x", __func__,
		  device_can_wakeup(&udev->dev), device_may_wakeup(&udev->dev));

	data = aic_alloc(intf);
	if (!data)
		return -ENOMEM;

	for (i = 0; i < intf->cur_altsetting->desc.bNumEndpoints; i++) {
		ep_desc = &intf->cur_altsetting->endpoint[i].desc;

		endpoint_num = usb_endpoint_num(ep_desc);
		printk("endpoint num %d\n", endpoint_num);

		if (!data->intr_ep && usb_endpoint_is_int_in(ep_desc)) {
			data->intr_ep = ep_desc;
			continue;
		}

		if (!data->bulk_tx_ep && usb_endpoint_is_bulk_out(ep_desc)) {
			data->bulk_tx_ep = ep_desc;
			continue;
		}

		if (!data->bulk_rx_ep && usb_endpoint_is_bulk_in(ep_desc)) {
			data->bulk_rx_ep = ep_desc;
			continue;
		}
	}

	if (!data->intr_ep || !data->bulk_tx_ep || !data->bulk_rx_ep) {
		aic_free(data);
		return -ENODEV;
	}

	data->cmdreq_type = USB_TYPE_CLASS;

	data->udev = udev;
	data->intf = intf;

	dlfw_dis_state = 0;
	spin_lock_init(&queue_lock);
	spin_lock_init(&dlfw_lock);
	spin_lock_init(&data->lock);

	INIT_WORK(&data->work, btusb_work);
	INIT_WORK(&data->waker, btusb_waker);
	spin_lock_init(&data->txlock);

	init_usb_anchor(&data->tx_anchor);
	init_usb_anchor(&data->intr_anchor);
	init_usb_anchor(&data->bulk_anchor);
	init_usb_anchor(&data->isoc_anchor);
	init_usb_anchor(&data->deferred);

	spin_lock_init(&data->rxlock);
	data->recv_bulk = btusb_recv_bulk;

	fw_info = firmware_info_init(intf);
	if (fw_info)
		data->fw_info = fw_info;
	else {
		AICBT_WARN("%s: Failed to initialize fw info", __func__);
		/* Skip download patch */
		goto end;
	}

	AICBT_INFO("%s: download begining...", __func__);

	if (g_chipid == PRODUCT_ID_AIC8800DC) {
		err = download_patch(data->fw_info, 1);
	}

	AICBT_INFO("%s: download ending...", __func__);
	if (err < 0) {
		return err;
	}

	hdev = hci_alloc_dev();
	if (!hdev) {
		aic_free(data);
		data = NULL;
		return -ENOMEM;
	}

	HDEV_BUS = HCI_USB;

	data->hdev = hdev;

	SET_HCIDEV_DEV(hdev, &intf->dev);

	hdev->open = btusb_open;
	hdev->close = btusb_close;
	hdev->flush = btusb_flush;
	hdev->send = btusb_send_frame;
	hdev->notify = btusb_notify;
	hdev->shutdown = btusb_shutdown;

	hci_set_drvdata(hdev, data);

	if (!reset_on_close) {
		/* set_bit(HCI_QUIRK_RESET_ON_CLOSE, &hdev->quirks); */
		AICBT_DBG("%s: Set HCI_QUIRK_RESET_ON_CLOSE", __func__);
	}

	/* Interface numbers are hardcoded in the specification */
	data->isoc = usb_ifnum_to_if(data->udev, 1);
	if (data->isoc) {
		err = usb_driver_claim_interface(&btusb_driver, data->isoc,
						 data);
		if (err < 0) {
			hci_free_dev(hdev);
			hdev = NULL;
			aic_free(data);
			data = NULL;
			return err;
		}
	}

	err = hci_register_dev(hdev);
	if (err < 0) {
		hci_free_dev(hdev);
		hdev = NULL;
		aic_free(data);
		data = NULL;
		return err;
	}

	usb_set_intfdata(intf, data);

	data->pm_notifier.notifier_call = bt_pm_notify;
	data->reboot_notifier.notifier_call = bt_reboot_notify;
	register_pm_notifier(&data->pm_notifier);
	register_reboot_notifier(&data->reboot_notifier);

end:
	return 0;
}

static void btusb_disconnect(struct usb_interface *intf)
{
	struct btusb_data *data;
	struct hci_dev *hdev = NULL;

	bt_support = 0;

	AICBT_INFO("%s: usb_interface %p, bInterfaceNumber %d", __func__, intf,
		   intf->cur_altsetting->desc.bInterfaceNumber);

	data = usb_get_intfdata(intf);

	if (intf->cur_altsetting->desc.bInterfaceNumber != 0)
		return;

	if (data)
		hdev = data->hdev;
	else {
		AICBT_WARN("%s: Failed to get bt usb data[Null]", __func__);
		return;
	}

	unregister_pm_notifier(&data->pm_notifier);
	unregister_reboot_notifier(&data->reboot_notifier);

	firmware_info_destroy(intf);

	usb_set_intfdata(data->intf, NULL);

	if (data->isoc)
		usb_set_intfdata(data->isoc, NULL);

	hci_unregister_dev(hdev);

	if (intf == data->isoc)
		usb_driver_release_interface(&btusb_driver, data->intf);
	else if (data->isoc)
		usb_driver_release_interface(&btusb_driver, data->isoc);

	hci_free_dev(hdev);
	aic_free(data);
	data = NULL;
	set_dlfw_state_value(0);
}

#ifdef CONFIG_PM
static int btusb_suspend(struct usb_interface *intf, pm_message_t message)
{
	struct btusb_data *data = usb_get_intfdata(intf);
	//firmware_info *fw_info = data->fw_info;

	AICBT_INFO("%s: event 0x%x, suspend count %d", __func__, message.event,
		   data->suspend_count);

	if (intf->cur_altsetting->desc.bInterfaceNumber != 0)
		return 0;

#ifdef CONFIG_BT_WAKEUP_IN_PM
	set_bt_wakeup_param(fw_info);
#endif

	if (data->suspend_count++)
		return 0;

	spin_lock_irq(&data->txlock);
	if (!((message.event & PM_EVENT_AUTO) && data->tx_in_flight)) {
		set_bit(BTUSB_SUSPENDING, &data->flags);
		spin_unlock_irq(&data->txlock);
	} else {
		spin_unlock_irq(&data->txlock);
		data->suspend_count--;
		AICBT_ERR("%s: Failed to enter suspend", __func__);
		return -EBUSY;
	}

	cancel_work_sync(&data->work);

	btusb_stop_traffic(data);
	mdelay(URB_CANCELING_DELAY_MS);
	usb_kill_anchored_urbs(&data->tx_anchor);

	return 0;
}

static void play_deferred(struct btusb_data *data)
{
	struct urb *urb;
	int err;

	while ((urb = usb_get_from_anchor(&data->deferred))) {
		usb_anchor_urb(urb, &data->tx_anchor);
		err = usb_submit_urb(urb, GFP_ATOMIC);
		if (err < 0) {
			AICBT_ERR("%s: Failed to submit urb %p, err %d",
				  __func__, urb, err);
			kfree(urb->setup_packet);
			usb_unanchor_urb(urb);
		} else {
			usb_mark_last_busy(data->udev);
		}
		usb_free_urb(urb);

		data->tx_in_flight++;
	}
	mdelay(URB_CANCELING_DELAY_MS);
	usb_scuttle_anchored_urbs(&data->deferred);
}

static int btusb_resume(struct usb_interface *intf)
{
	struct btusb_data *data = usb_get_intfdata(intf);
	struct hci_dev *hdev = data->hdev;
	int err = 0;

	AICBT_INFO("%s: Suspend count %d", __func__, data->suspend_count);

	if (intf->cur_altsetting->desc.bInterfaceNumber != 0)
		return 0;

	if (--data->suspend_count)
		return 0;

#if 0
    /*check_fw_version to check the status of the BT Controller after USB Resume*/
    err = check_fw_version(fw_info);
    if (err !=0)
    {
        AICBT_INFO("%s: BT Controller Power OFF And Return hci_hardware_error:%d", __func__, err);
        hci_hardware_error();
    }
#endif

	AICBT_INFO("%s g_chipid %x\n", __func__, g_chipid);
	if (g_chipid == PRODUCT_ID_AIC8800DC) {
		if (data->fw_info) {
			err = download_patch(data->fw_info, 1);
		} else {
			AICBT_WARN("%s: Failed to initialize fw info",
				   __func__);
		}
	}
#ifdef CONFIG_BT_WAKEUP_IN_PM
	reset_bt_from_wakeup_process(data->fw_info);
	hci_resume_hardware_error();
#endif

#if 1
	if (test_bit(BTUSB_INTR_RUNNING, &data->flags)) {
		err = btusb_submit_intr_urb(hdev, GFP_NOIO);
		if (err < 0) {
			clear_bit(BTUSB_INTR_RUNNING, &data->flags);
			goto failed;
		}
	}
#endif

	if (test_bit(BTUSB_BULK_RUNNING, &data->flags)) {
		err = btusb_submit_bulk_urb(hdev, GFP_NOIO);
		if (err < 0) {
			clear_bit(BTUSB_BULK_RUNNING, &data->flags);
			goto failed;
		}

		btusb_submit_bulk_urb(hdev, GFP_NOIO);
	}

	if (test_bit(BTUSB_ISOC_RUNNING, &data->flags)) {
		if (btusb_submit_isoc_urb(hdev, GFP_NOIO) < 0)
			clear_bit(BTUSB_ISOC_RUNNING, &data->flags);
		else
			btusb_submit_isoc_urb(hdev, GFP_NOIO);
	}

	spin_lock_irq(&data->txlock);
	play_deferred(data);
	clear_bit(BTUSB_SUSPENDING, &data->flags);
	spin_unlock_irq(&data->txlock);
	schedule_work(&data->work);

	return 0;

failed:
	mdelay(URB_CANCELING_DELAY_MS);
	usb_scuttle_anchored_urbs(&data->deferred);
	spin_lock_irq(&data->txlock);
	clear_bit(BTUSB_SUSPENDING, &data->flags);
	spin_unlock_irq(&data->txlock);

	return err;
}
#endif

static struct usb_driver btusb_driver = {
	.name = "aic_btusb",
	.probe = btusb_probe,
	.disconnect = btusb_disconnect,
#ifdef CONFIG_PM
	.suspend = btusb_suspend,
	.resume = btusb_resume,
#if CONFIG_RESET_RESUME
	.reset_resume = btusb_resume,
#endif
#endif
	.id_table = btusb_table,
	.supports_autosuspend = 1,
	.disable_hub_initiated_lpm = 1,
};

static int __init btusb_init(void)
{
	int err;

	AICBT_INFO("AICBT_RELEASE_NAME: %s", AICBT_RELEASE_NAME);
	AICBT_INFO("AicSemi Bluetooth USB driver module init, version %s",
		   VERSION);
	AICBT_INFO("RELEASE DATE: 2023_1211_1958 \r\n");
	err = usb_register(&btusb_driver);
	if (err < 0)
		AICBT_ERR("Failed to register aic bluetooth USB driver");
	return err;
}

static void __exit btusb_exit(void)
{
	AICBT_INFO("AicSemi Bluetooth USB driver module exit");
	usb_deregister(&btusb_driver);
}

module_init(btusb_init);
module_exit(btusb_exit);

module_param(mp_drv_mode, int, 0644);
MODULE_PARM_DESC(mp_drv_mode, "0: NORMAL; 1: MP MODE");

#if LINUX_VERSION_CODE >= KERNEL_VERSION(5, 4, 0)
#if LINUX_VERSION_CODE >= KERNEL_VERSION(6, 13, 0)
MODULE_IMPORT_NS("VFS_internal_I_am_really_a_filesystem_and_am_NOT_a_driver");
#else
MODULE_IMPORT_NS(VFS_internal_I_am_really_a_filesystem_and_am_NOT_a_driver);
#endif
#endif

MODULE_AUTHOR("AicSemi Corporation");
MODULE_DESCRIPTION("AicSemi Bluetooth USB driver version");
MODULE_VERSION(VERSION);
MODULE_LICENSE("GPL");
