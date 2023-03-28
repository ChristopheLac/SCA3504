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
LOG_MODULE_REGISTER(main_usb);

static bool configured;
static const struct device *hdev;
static struct k_work report_send;
static ATOMIC_DEFINE(hid_ep_in_busy, 1);

#define HID_EP_BUSY_FLAG	0
#define REPORT_ID_1		0x01
#define REPORT_PERIOD		K_SECONDS(2)

static struct report {
	uint8_t id;
	uint8_t value;
} __packed report_1 = {
	.id = REPORT_ID_1,
	.value = 0,
};

static void report_event_handler(struct k_timer *dummy);
static K_TIMER_DEFINE(event_timer, report_event_handler, NULL);

//usb_register_descriptors

/*
 * Simple HID Report Descriptor
 * Report ID is present for completeness, although it can be omitted.
 * Output of "usbhid-dump -d 2fe3:0006 -e descriptor":
 *  05 01 09 00 A1 01 15 00    26 FF 00 85 01 75 08 95
 *  01 09 00 81 02 C0
 */
static const uint8_t hid_report_desc[] = HID_MOUSE_REPORT_DESC(2);

static const uint8_t hid_report_desc2[] = {
	HID_USAGE_PAGE(HID_USAGE_GEN_DESKTOP),
	HID_USAGE(HID_USAGE_GEN_DESKTOP_UNDEFINED),
	HID_COLLECTION(HID_COLLECTION_APPLICATION),
	HID_USAGE_MIN8(1),
	HID_USAGE_MAX8(10),
	HID_LOGICAL_MIN8(0),
	HID_LOGICAL_MAX8(255),
	HID_REPORT_ID(2),
	HID_REPORT_SIZE(8),
	HID_REPORT_COUNT(10),

	HID_OUTPUT(0x02),
	HID_REPORT_SIZE(5),
	HID_REPORT_COUNT(1),
	HID_USAGE_MIN8(1),
	HID_USAGE_MAX8(10),
	HID_LOGICAL_MIN8(0),
	HID_LOGICAL_MAX8(255),
	HID_REPORT_SIZE(8),
	HID_REPORT_COUNT(10),
	HID_REPORT_ID(3),

	HID_INPUT(0x02),
	HID_END_COLLECTION,
};

uint8_t bufferIn[100];
uint8_t bufferOut[100];


//static void send_report(struct k_work *work)
static void send_report(void)
{
	int ret, wrote;

	if (!atomic_test_and_set_bit(hid_ep_in_busy, HID_EP_BUSY_FLAG)) {
		bufferIn[0] = CMD_VERSION;
		traiteCommande(bufferIn, sizeof(bufferIn), bufferOut, 32);
		ret = hid_int_ep_write(hdev, (uint8_t *)bufferOut,
				       32, &wrote);
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

static void report_event_handler(struct k_timer *dummy)
{
	/* Increment reported data */
	report_1.value++;
	k_work_submit(&report_send);
}

static void protocol_cb(const struct device *dev, uint8_t protocol)
{
	LOG_INF("New protocol: %s", protocol == HID_PROTOCOL_BOOT ?
		"boot" : "report");
}
uint8_t test = 0;
int get_report_cb(const struct device *dev, struct usb_setup_packet *setup, int32_t *len, uint8_t **data) {
	if (USB_REQTYPE_DIR_TO_DEVICE == setup->RequestType.direction) {
		test++;
	} else {
		test--;
	}
	send_report();
	return 0;
}


int set_report_cb(const struct device *dev, struct usb_setup_packet *setup, int32_t *len, uint8_t **data) {
	if (USB_REQTYPE_DIR_TO_DEVICE == setup->RequestType.direction) {
		test++;
	} else {
		test--;
	}
	send_report();
	return 0;
}

static const struct hid_ops ops = {
	.int_in_ready = int_in_ready_cb,
	.on_idle = on_idle_cb,
	.protocol_change = protocol_cb,
	.get_report = get_report_cb,
	.set_report = set_report_cb,
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
		break;
	default:
		LOG_DBG("status %u unhandled", status);
		break;
	}
}

void main_usb(void)
{
	int ret;

	LOG_INF("Starting application");

	ret = usb_enable(status_cb);
	if (ret != 0) {
		LOG_ERR("Failed to enable USB");
		return;
	}

//	k_work_init(&report_send, send_report);
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
