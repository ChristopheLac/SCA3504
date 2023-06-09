/*
 * Copyright (c) 2016-2018 Intel Corporation.
 * Copyright (c) 2018-2021 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/init.h>

#include <zephyr/usb/usb_device.h>
#include <zephyr/usb/class/usb_hid.h>
#include "commandUsb.h"

#define LOG_LEVEL LOG_LEVEL_INF
LOG_MODULE_REGISTER(usb_main);

static bool configured;
static const struct device *hdev;
static ATOMIC_DEFINE(hid_ep_in_busy, 1);
static struct k_work report_send;

#define HID_EP_BUSY_FLAG	0

/*
 * Simple HID Report Descriptor
 * Report ID is present for completeness, although it can be omitted.
 * Output of "usbhid-dump -d 2fe3:0006 -e descriptor":
 *  05 01 09 00 A1 01 15 00    26 FF 00 85 01 75 08 95
 *  01 09 00 81 02 C0
 */
static const uint8_t hid_report_desc[] = {
//	HID_USAGE_PAGE(0xFF00),
	0x06, 0x00, 0xFF, // USAGE_PAGE(Vendor-Defined) => 06 00 FF n'existe pas dans ZEPYR
	HID_USAGE(HID_USAGE_GEN_DESKTOP_POINTER),
	HID_COLLECTION(HID_COLLECTION_APPLICATION),
		HID_USAGE_MIN8(1),
		HID_USAGE_MAX8(32),
		HID_LOGICAL_MIN8(1),
		HID_LOGICAL_MAX16(0xFF, 0x00),
		HID_REPORT_SIZE(8),
		HID_REPORT_COUNT(32),
		HID_OUTPUT(0x02),

		HID_USAGE_MIN8(1),
		HID_USAGE_MAX8(32),
		HID_LOGICAL_MIN8(0),
		HID_LOGICAL_MAX16(0xFF, 0x00),
		HID_REPORT_SIZE(8),
		HID_REPORT_COUNT(32),
		HID_INPUT(0x02),

	HID_END_COLLECTION,
};
volatile uint8_t bufferIn[32];
volatile uint32_t byteRead;

static void send_report(struct k_work *work)
{
	int ret, wrote;
		// ret = hid_int_ep_read(hdev, (uint8_t *)&bufferIn,
		// 		       sizeof(bufferIn), &byteRead);
		// if (ret != 0) {
		// 	/*
		// 	 * Do nothing and wait until host has reset the device
		// 	 * and hid_ep_in_busy is cleared.
		// 	 */
		// 	LOG_ERR("OUT Failed to read report");
		// } else {
		// 	LOG_DBG("OUT Report read %d %d %d", bufferIn[0], bufferIn[1], bufferIn[2]);
		// }


	if (!atomic_test_and_set_bit(hid_ep_in_busy, HID_EP_BUSY_FLAG)) {
//		ret = hid_int_ep_write(hdev, (uint8_t *)&report_1,
//				       sizeof(report_1), &wrote);
		if (ret != 0) {
			/*
			 * Do nothing and wait until host has reset the device
			 * and hid_ep_in_busy is cleared.
			 */
			LOG_ERR("Failed to submit report");
		} else {
			LOG_DBG("Report submitted");
		}
	} else {
		LOG_DBG("HID IN endpoint busy");
	}
}

static void int_in_ready_cb(const struct device *dev)
{
	ARG_UNUSED(dev);
	if (!atomic_test_and_clear_bit(hid_ep_in_busy, HID_EP_BUSY_FLAG)) {
		LOG_WRN("IN endpoint callback without preceding buffer write");
	} else {
		LOG_WRN("IN endpoint callback without preceding buffer write");
	}
}

static void int_out_ready_cb(const struct device *dev)
{
	int ret;
	uint8_t bufferIn[32];
	uint8_t bufferOut[32];
	uint32_t byteRead, byteToSend;
	ret = hid_int_ep_read(hdev, (uint8_t *)&bufferIn, sizeof(bufferIn), &byteRead);
	if (ret != 0) {
		/*
			* Do nothing and wait until host has reset the device
			* and hid_ep_in_busy is cleared.
			*/
		LOG_ERR("int_OUT Failed to read report");
	} else {
		LOG_DBG("int_OUT Report read:%d [%d %d %d]", byteRead, bufferIn[0], bufferIn[1], bufferIn[2]);
		byteToSend = traiteCommande(bufferIn, byteRead, bufferOut, sizeof(bufferOut));
		if (byteToSend > 0) {
			byteToSend = 32;
			ret = hid_int_ep_write(hdev, bufferOut, byteToSend, NULL);
			if (ret != 0) {
				/*
				* Do nothing and wait until host has reset the device
				* and hid_ep_in_busy is cleared.
				*/
				LOG_ERR("int_OUT Failed to submit report %d", ret);
			} else {
				LOG_DBG("int_OUT Report submitted");
				k_sleep(K_MSEC(10));
			}
		}
	}
}

/*
 * On Idle callback is available here as an example even if actual use is
 * very limited. In contrast to report_event_handler(),
 * report value is not incremented here.
 */
static void on_idle_cb(const struct device *dev, uint16_t report_id)
{
	LOG_DBG("On idle callback");
	k_work_submit(&report_send);
}

static void protocol_cb(const struct device *dev, uint8_t protocol)
{
	LOG_INF("New protocol: %s", protocol == HID_PROTOCOL_BOOT ?
		"boot" : "report");
}

static const struct hid_ops ops = {
//	.int_in_ready = int_in_ready_cb,
#ifdef CONFIG_ENABLE_HID_INT_OUT_EP
	.int_out_ready = int_out_ready_cb,
#endif
	.on_idle = on_idle_cb,
	.protocol_change = protocol_cb,
};

static void status_cb(enum usb_dc_status_code status, const uint8_t *param)
{
	switch (status) {
	case USB_DC_RESET:
		configured = false;
		break;
	case USB_DC_CONFIGURED:
		if (!configured) {
			int_in_ready_cb(hdev);
			configured = true;
		}
		break;
	case USB_DC_SOF:
//		LOG_DBG("new trame receive");
		break;
	default:
		LOG_DBG("status %u unhandled", status);
		break;
	}
}

void commandUsb_init(void);
void usb_main(void)
{
	int ret;
    commandUsb_init();

	LOG_INF("Starting application");

	ret = usb_enable(status_cb);
	if (ret != 0) {
		LOG_ERR("Failed to enable USB");
		return;
	}

	k_work_init(&report_send, send_report);
}

static int composite_pre_init(const struct device *dev)
{
	hdev = device_get_binding("HID_0");
	if (hdev == NULL) {
		LOG_ERR("Cannot get USB HID Device");
		return -ENODEV;
	}

	LOG_INF("HID Device: dev %p", hdev);

	usb_hid_register_device(hdev, hid_report_desc, sizeof(hid_report_desc),
				&ops);

	atomic_set_bit(hid_ep_in_busy, HID_EP_BUSY_FLAG);
//	k_timer_start(&event_timer, REPORT_PERIOD, REPORT_PERIOD);

	if (usb_hid_set_proto_code(hdev, HID_BOOT_IFACE_CODE_NONE)) {
		LOG_WRN("Failed to set Protocol Code");
	}

	return usb_hid_init(hdev);
}

SYS_INIT(composite_pre_init, APPLICATION, CONFIG_KERNEL_INIT_PRIORITY_DEVICE);


/* size of stack area used by each thread */
#define STACKSIZE 1024
/* scheduling priority used by each thread */
#define PRIORITY 7
	
K_THREAD_DEFINE(usb_main_id, STACKSIZE, usb_main, NULL, NULL, NULL, PRIORITY, 0, 0);
