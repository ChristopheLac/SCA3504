/*
 * Copyright (c) 2018 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

/** @file
 *  @brief Nordic UART Bridge Service (NUS) sample
 */

#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/gatt.h>
#include <bluetooth/scan.h>

#include <zephyr/logging/log.h>

#define LOG_MODULE_NAME bluetooth_main
LOG_MODULE_REGISTER(LOG_MODULE_NAME);

#define STACKSIZE 1024
#define PRIORITY 7

extern struct bt_conn *conn_connected;
extern void (*start_scan_func)(void);
extern int write_cmd(struct bt_conn *conn);

static void scan_filter_match(struct bt_scan_device_info *device_info,
			      struct bt_scan_filter_match *filter_match,
			      bool connectable)
{
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(device_info->recv_info->addr, addr, sizeof(addr));
	LOG_INF("Filters matched. Connected %s", addr);
}

static void scan_connecting_error(struct bt_scan_device_info *device_info)
{
	LOG_ERR("Connecting failed");
}
static struct bt_conn *default_conn;
static void scan_connecting(struct bt_scan_device_info *device_info,
			    struct bt_conn *conn)
{
	default_conn = bt_conn_ref(conn);
}


BT_SCAN_CB_INIT(scan_cb, scan_filter_match, NULL,
		scan_connecting_error, scan_connecting);

const char bleDeviceName[]="SCANCUBE-SCA3512";
static void start_scan(void)
{
	int err;
	
	struct bt_scan_init_param scan_init = {
		.connect_if_match = 1,
	};

	bt_scan_init(&scan_init);
	bt_scan_cb_register(&scan_cb);

	err = bt_scan_filter_add(BT_SCAN_FILTER_TYPE_NAME, bleDeviceName);
    if (err) {
        LOG_ERR("Scanning filters cannot be add (%d)", err);
        return;
    }
	err = bt_scan_filter_enable(BT_SCAN_NAME_FILTER, false);
    if (err) {
        LOG_ERR("Scanning filters cannot be enable (%d)", err);
        return;
    }

	err = bt_scan_start(BT_SCAN_TYPE_SCAN_ACTIVE);
	if (err) {
		LOG_ERR("%s: Scanning failed to start (err %d)", __func__,
		       err);
		return;
	}

	LOG_INF("%s: Scanning successfully started\n", __func__);
}

void mtu_updated(struct bt_conn *conn, uint16_t tx, uint16_t rx)
{
	LOG_INF("Updated MTU: TX: %d RX: %d bytes", tx, rx);
}

static struct bt_gatt_cb gatt_callbacks = {
	.att_mtu_updated = mtu_updated
};

void bluetooth_main(void)
{
	int err;
	uint32_t count = 10;

	err = bt_enable(NULL);
	if (err) {
		LOG_ERR("Bluetooth init failed (err %d)\n", err);
		while (1)
		{
			k_sleep(K_SECONDS(1));
		}
		
	}
	LOG_INF("Bluetooth initialized\n");

	bt_gatt_cb_register(&gatt_callbacks);

	conn_connected = NULL;

	start_scan_func = start_scan;
	start_scan_func();

	struct bt_conn *conn;
	k_sleep(K_SECONDS(10));


	while (true) {
///		conn = NULL;

		if (conn_connected) {
			/* Get a connection reference to ensure that a
			 * reference is maintained in case disconnected
			 * callback is called while we perform GATT Write
			 * command.
			 */
			conn = bt_conn_ref(conn_connected);
		}

		if (conn) {
			(void)write_cmd(conn);
			bt_conn_unref(conn);

			if (count) {
				count--;
				if (!count) {
					break;
				}
			}

			k_yield();
		} else {
			k_sleep(K_SECONDS(1));
		}
	}
}

K_THREAD_DEFINE(bluetooth_main_id, STACKSIZE, bluetooth_main, NULL, NULL, NULL, PRIORITY, 0, 0);

