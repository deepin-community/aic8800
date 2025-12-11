/*
 *
 *  Aic Bluetooth USB driver
 *
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
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
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/types.h>
#include <linux/sched.h>
#include <linux/skbuff.h>
#include <linux/errno.h>
#include <linux/usb.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/poll.h>

#include <linux/version.h>
#include <linux/pm_runtime.h>
#include <linux/firmware.h>
#include <linux/suspend.h>

#include <net/bluetooth/bluetooth.h>
#include <net/bluetooth/hci_core.h>
#include <net/bluetooth/hci.h>

/***********************************
** AicSemi - For aic_btusb driver **
***********************************/
#define URB_CANCELING_DELAY_MS 10
/* when OS suspended, module is still powered,usb is not powered,
 * this may set to 1, and must comply with special patch code.
 */
#define CONFIG_RESET_RESUME 1
#define PRINT_CMD_EVENT 0
#define PRINT_ACL_DATA 0
#define PRINT_SCO_DATA 0

#define AICBT_DBG_FLAG 0

#if AICBT_DBG_FLAG
#define AICBT_DBG(fmt, arg...) printk("aic_btusb: " fmt "\n", ##arg)
#else
#define AICBT_DBG(fmt, arg...)
#endif

#define AICBT_INFO(fmt, arg...) printk("aic_btusb: " fmt "\n", ##arg)
#define AICBT_WARN(fmt, arg...) printk("aic_btusb: " fmt "\n", ##arg)
#define AICBT_ERR(fmt, arg...) printk("aic_btusb: " fmt "\n", ##arg)

#define HDEV_BUS hdev->bus
#define USB_RPM 1

#define GET_DRV_DATA(x) hci_get_drvdata(x)

#define SCO_NUM hdev->conn_hash.sco_num

#define BTUSB_RPM (0 * USB_RPM) /* 1 SS enable; 0 SS disable */
#define BTUSB_WAKEUP_HOST 0 /* 1  enable; 0  disable */
#define BTUSB_MAX_ISOC_FRAMES 48
#define BTUSB_INTR_RUNNING 0
#define BTUSB_BULK_RUNNING 1
#define BTUSB_ISOC_RUNNING 2
#define BTUSB_SUSPENDING 3
#define BTUSB_DID_ISO_RESUME 4

#define HCI_VENDOR_USB_DISC_HARDWARE_ERROR 0xFF
#define HCI_VENDOR_USB_RESUME_HARDWARE_ERROR 0xFE

#define HCI_CMD_READ_BD_ADDR 0x1009
#define HCI_VENDOR_READ_LMP_VERISION 0x1001
#define HCI_VENDOR_RESET 0x0C03

#define DRV_NORMAL_MODE 0
#define DRV_MP_MODE 1
int mp_drv_mode = 0; /* 1 Mptool Fw; 0 Normal Fw */

#define HCI_OP_Write_Extended_Inquiry_Response 0x0c52
#define HCI_OP_Write_Simple_Pairing_Mode 0x0c56
#define HCI_OP_Read_Buffer_Size 0x1005
#define HCI_OP_Host_Buffer_Size 0x0c33
#define HCI_OP_Read_Local_Version_Information 0x1001
#define HCI_OP_Read_BD_ADDR 0x1009
#define HCI_OP_Read_Local_Supported_Commands 0x1002
#define HCI_OP_Write_Scan_Enable 0x0c1a
#define HCI_OP_Write_Current_IAC_LAP 0x0c3a
#define HCI_OP_Write_Inquiry_Scan_Activity 0x0c1e
#define HCI_OP_Write_Class_of_Device 0x0c24
#define HCI_OP_LE_Rand 0x2018
#define HCI_OP_LE_Set_Random_Address 0x2005
#define HCI_OP_LE_Set_Extended_Scan_Enable 0x2042
#define HCI_OP_LE_Set_Extended_Scan_Parameters 0x2041
#define HCI_OP_Set_Event_Filter 0x0c05
#define HCI_OP_Write_Voice_Setting 0x0c26
#define HCI_OP_Change_Local_Name 0x0c13
#define HCI_OP_Read_Local_Name 0x0c14
#define HCI_OP_Wirte_Page_Timeout 0x0c18
#define HCI_OP_LE_Clear_Resolving_List 0x0c29
#define HCI_OP_LE_Set_Addres_Resolution_Enable_Command 0x0c2e
#define HCI_OP_Write_Inquiry_mode 0x0c45
#define HCI_OP_Write_Page_Scan_Type 0x0c47
#define HCI_OP_Write_Inquiry_Scan_Type 0x0c43

#define HCI_OP_Delete_Stored_Link_Key 0x0c12
#define HCI_OP_LE_Read_Local_Resolvable_Address 0x202d
#define HCI_OP_LE_Extended_Create_Connection 0x2043
#define HCI_OP_Read_Remote_Version_Information 0x041d
#define HCI_OP_LE_Start_Encryption 0x2019
#define HCI_OP_LE_Add_Device_to_Resolving_List 0x2027
#define HCI_OP_LE_Set_Privacy_Mode 0x204e
#define HCI_OP_LE_Connection_Update 0x2013

#define HCI_OP_LE_Read_Local_Support_Featrues 0x2003
#define HCI_OP_LE_Get_Vendor_Capabilities_Command 0xfd53
#define HCI_OP_LE_Set_Le_Scan_Enable 0x200C
#define HCI_OP_LE_Create_Connection 0x200d

typedef struct {
	struct usb_interface *intf;
	struct usb_device *udev;
	int pipe_in, pipe_out;
	uint8_t *send_pkt;
	uint8_t *rcv_pkt;
	struct hci_command_hdr *cmd_hdr;
	struct hci_event_hdr *evt_hdr;
	struct hci_ev_cmd_complete *cmd_cmp;
	uint8_t *req_para, *rsp_para;
	uint8_t *fw_data;
	int pkt_len;
	int fw_len;
} firmware_info;

/*******************************
**    Reasil patch code
********************************/
#define CMD_CMP_EVT 0x0e
#define RCV_PKT_LEN 64
#define SEND_PKT_LEN 300
#define MSG_TO 1000
#define PATCH_SEG_MAX 252
#define DATA_END 0x80
#define DOWNLOAD_OPCODE 0xfc02
#define HCI_VSC_UPDATE_PT_CMD 0xFC75
#define HCI_VSC_SET_ADFILTER_PT_CMD 0xFDAB
#define HCI_VSC_RESET_ADFILTER_PROCESS_PT_CMD 0xFDAC
#define TRUE 1
#define FALSE 0
#define CMD_HDR_LEN sizeof(struct hci_command_hdr)
#define EVT_HDR_LEN sizeof(struct hci_event_hdr)
#define CMD_CMP_LEN sizeof(struct hci_ev_cmd_complete)
#define MAX_PATCH_SIZE_24K (1024 * 24)
#define MAX_PATCH_SIZE_40K (1024 * 40)

#define FW_RAM_ADID_BASE_ADDR 0x101788
#define FW_RAM_PATCH_BASE_ADDR 0x184000
#define AICBT_PT_TAG "AICBT_PT_TAG"

enum aicbt_patch_table_type {
	AICBT_PT_INF = 0x00,
	AICBT_PT_TRAP = 0x1,
	AICBT_PT_B4,
	AICBT_PT_BTMODE,
	AICBT_PT_PWRON,
	AICBT_PT_AF,
	AICBT_PT_VER,
	AICBT_PT_MAX,
};

enum AIC_DC_SUBID {
	DC_U01 = 0,
	DC_U02,
	DC_U02H,
};

struct aicbt_firmware {
	const char *desc;
	const char *bt_adid;
	const char *bt_patch;
	const char *bt_table;
	const char *bt_ext_patch;
};

const struct aicbt_firmware fw_8800dc[] = {
	[DC_U01] = { .desc = "aic8800dc u01 bt patch",
		     .bt_adid = "fw_adid_8800dc.bin",
		     .bt_patch = "fw_patch_8800dc.bin",
		     .bt_table = "fw_patch_table_8800dc.bin",
		     .bt_ext_patch = "fw_patch_8800dc_ext" },
	[DC_U02] = { .desc = "aic8800dc u02 bt patch",
		     .bt_adid = "fw_adid_8800dc_u02.bin",
		     .bt_patch = "fw_patch_8800dc_u02.bin",
		     .bt_table = "fw_patch_table_8800dc_u02.bin",
		     .bt_ext_patch = "fw_patch_8800dc_u02_ext" },
	[DC_U02H] = { .desc = "aic8800dch u02 bt patch",
		      .bt_adid = "fw_adid_8800dc_u02h.bin",
		      .bt_patch = "fw_patch_8800dc_u02h.bin",
		      .bt_table = "fw_patch_table_8800dc_u02h.bin",
		      .bt_ext_patch = "fw_patch_8800dc_u02h_ext" },
};

#define HCI_VSC_FW_STATUS_GET_CMD 0xFC78

struct fw_status {
	u8 status;
} __packed;

#define HCI_PATCH_DATA_MAX_LEN 240
#define HCI_VSC_MEM_WR_SIZE 240
#define HCI_VSC_MEM_RD_SIZE 128
#define HCI_VSC_UPDATE_PT_SIZE 249
#define HCI_PT_MAX_LEN 31

#define HCI_VSC_DBG_RD_MEM_CMD 0xFC01

#define MAX_AD_FILTER_NUM 4 // Max AD Filter num
#define MAX_GPIO_TRIGGER_NUM 2 // Max user config num of gpio
#define MAX_ROLE_COMNO_IDX_NUM \
	2 // Max num of ad role type combo,form( enum gpio_combo_idx)

#define AD_ROLE_FLAG 0x0f
#define ROLE_COMBO_IDX_FLAG 0xf0

enum ad_role_type {
	ROLE_ONLY, // ROLE_ONLY will trigger wake up immediately.
	ROLE_COMBO, //ROLE_COMBO will trigger When all the conditions (ad_role == ROLE_COMBO,and ad_filter is matching)are met.
};

enum gpio_combo_idx {
	COMBO_0,
	COMBO_1,
};

enum gpio_trigger_bit {
	TG_IDX_0 = (1 << 0),
	TG_IDX_1 = (1 << 1),
};

struct wakeup_ad_data_filter {
	uint32_t ad_data_mask;
	uint8_t gpio_trigger_idx;
	uint8_t ad_role; //from enum ad_role_type
	uint8_t ad_len;
	uint8_t ad_type;
	uint8_t ad_data[31];
	bdaddr_t wl_addr;
};

struct ble_wakeup_param_t {
	uint32_t magic_num; // "BLES" = 0x53454C42
	uint32_t delay_scan_to; // timeout for start scan in ms
	uint32_t reboot_to; // timeout for reboot in ms
	uint32_t gpio_num[MAX_GPIO_TRIGGER_NUM];
	uint32_t gpio_dft_lvl[MAX_GPIO_TRIGGER_NUM];
	struct wakeup_ad_data_filter ad_filter[MAX_AD_FILTER_NUM];
};

struct hci_dbg_rd_mem_cmd {
	__le32 start_addr;
	__u8 type;
	__u8 length;
} __attribute__((packed));

struct hci_dbg_rd_mem_cmd_evt {
	__u8 status;
	__u8 length;
	__u8 data[HCI_VSC_MEM_RD_SIZE];
} __attribute__((packed));

struct long_buffer_tag {
	__u8 length;
	__u8 data[HCI_VSC_MEM_WR_SIZE];
};

struct hci_dbg_wr_mem_cmd {
	__le32 start_addr;
	__u8 type;
	__u8 length;
	__u8 data[HCI_VSC_MEM_WR_SIZE];
};

struct aicbt_patch_table {
	char *name;
	uint32_t type;
	uint32_t *data;
	uint32_t len;
	struct aicbt_patch_table *next;
};

struct aicbt_patch_info_t {
	uint32_t info_len;
	//base len start
	uint32_t adid_addrinf;
	uint32_t addr_adid;
	uint32_t patch_addrinf;
	uint32_t addr_patch;
	uint32_t reset_addr;
	uint32_t reset_val;
	uint32_t adid_flag_addr;
	uint32_t adid_flag;
	//base len end
	//ext patch nb
	uint32_t ext_patch_nb_addr;
	uint32_t ext_patch_nb;
	uint32_t *ext_patch_param;
};

struct aicbt_patch_table_cmd {
	uint8_t patch_num;
	uint32_t patch_table_addr[31];
	uint32_t patch_table_data[31];
} __attribute__((packed));

enum aic_endpoit { CTRL_EP = 0, INTR_EP = 3, BULK_EP = 1, ISOC_EP = 4 };

/* #define HCI_VERSION_CODE KERNEL_VERSION(3, 14, 41) */
#define HCI_VERSION_CODE LINUX_VERSION_CODE

int aic_load_firmware(u8 **fw_buf, const char *name, struct device *device);
int aicbt_patch_table_free(struct aicbt_patch_table **head);
int download_patch(firmware_info *fw_info, int cached);

#define NUM_REASSEMBLY 4
