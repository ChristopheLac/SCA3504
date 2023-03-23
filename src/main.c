/*
 * Copyright (c) 2022 Libre Solar Technologies GmbH
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>

#include <string.h>

#include <zephyr/sys/util.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/modbus/modbus.h>

#include <zephyr/devicetree.h> // PWM
#include <errno.h>
#include <zephyr/drivers/led.h>
#include <zephyr/sys/util.h>


#include <zephyr/logging/log.h>

/*//#include "uart_async_adapter.h"// bluetooth

#include <zephyr/types.h>
//#include <zephyr/kernel.h>
//#include <zephyr/drivers/uart.h>
#include <zephyr/usb/usb_device.h>

//#include <zephyr/device.h>
//#include <zephyr/devicetree.h>
#include <soc.h>

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/hci.h>

#include <bluetooth/services/nus.h>

//#include <dk_buttons_and_leds.h>

//#include <zephyr/settings/settings.h>

//#include <stdio.h>

//#include <zephyr/logging/log.h>*/
#include <zephyr/types.h>
#include <zephyr/usb/usb_device.h>
#include "uart_async_adapter.h"
#include <soc.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/hci.h>

#include <bluetooth/services/nus.h>

#include <dk_buttons_and_leds.h>

#include <zephyr/settings/settings.h>

#include <stdio.h>// BLuetooth

//

// debut essais LED DEBUG
/* The devicetree node identifier for the "led0" alias. */
#define LED0_NODE DT_ALIAS(led0)

/*
 * A build error on this line means your board is unsupported.
 * See the sample documentation for information on how to fix this.
 */
static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(LED0_NODE, gpios);


// fin essais LED DEBUG


LOG_MODULE_REGISTER(mbc_sample, LOG_LEVEL_INF);

static int client_iface;

const static struct modbus_iface_param client_param = {
	.mode = MODBUS_MODE_RTU,
	.rx_timeout = 50000,
	.serial = {
		.baud = 115200,//19200,
		.parity = UART_CFG_PARITY_EVEN,// UART_CFG_PARITY_NONE,
		.stop_bits_client = UART_CFG_STOP_BITS_1, // UART_CFG_STOP_BITS_2,
	},
};

#define MODBUS_NODE DT_COMPAT_GET_ANY_STATUS_OKAY(zephyr_modbus_serial)

static int init_modbus_client(void)
{
	const char iface_name[] = {DEVICE_DT_NAME(MODBUS_NODE)};

	client_iface = modbus_iface_get_by_name(iface_name);

	return modbus_init_client(client_iface, client_param);
}

#if (BLUETOOTH > 0)
/* change this to any other UART peripheral if desired */
#define UART_DEVICE_NODE DT_CHOSEN(zephyr_shell_uart)

#define MSG_SIZE 80

/* queue to store up to 10 messages (aligned to 4-byte boundary) */
K_MSGQ_DEFINE(uart_msgq, MSG_SIZE, 10, 4);

static const struct device *uart_dev = DEVICE_DT_GET(UART_DEVICE_NODE);//static const struct device *const uart_dev = DEVICE_DT_GET(UART_DEVICE_NODE);

/* receive buffer used in UART ISR callback */
static char rx_buf[MSG_SIZE];
static int rx_buf_pos;

/*
 * Read characters from UART until line end is detected. Afterwards push the
 * data to the message queue.
 */
void serial_cb(const struct device *dev, void *user_data)
{
	uint8_t c;

	if (!uart_irq_update(uart_dev)) {
		return;
	}

	while (uart_irq_rx_ready(uart_dev)) {

		uart_fifo_read(uart_dev, &c, 1);

		if ((c == '\n' || c == '\r') && rx_buf_pos > 0) {
			/* terminate string */
			rx_buf[rx_buf_pos] = '\0';

			/* if queue is full, message is silently dropped */
			k_msgq_put(&uart_msgq, &rx_buf, K_NO_WAIT);

			/* reset the buffer (it was copied to the msgq) */
			rx_buf_pos = 0;
		} else if (rx_buf_pos < (sizeof(rx_buf) - 1)) {
			rx_buf[rx_buf_pos++] = c;
		}
		/* else: characters beyond buffer size are dropped */
	}
}

/*
 * Print a null-terminated string character by character to the UART interface
 */
void print_uart(char *buf)
{
	int msg_len = strlen(buf);

	for (int i = 0; i < msg_len; i++) {
		uart_poll_out(uart_dev, buf[i]);
	}
}
#endif


//code pour le Bluetooth
#if (BLUETOOTH > 0)
/*#define LOG_MODULE_NAME peripheral_uart
LOG_MODULE_REGISTER(LOG_MODULE_NAME);*/

#define STACKSIZE 1024//  revisar CONFIG_BT_NUS_THREAD_STACK_SIZE
#define PRIORITY 7

#define DEVICE_NAME CONFIG_BT_DEVICE_NAME
#define DEVICE_NAME_LEN	(sizeof(DEVICE_NAME) - 1)

#define RUN_STATUS_LED DK_LED1//cambiar
#define RUN_LED_BLINK_INTERVAL 1000

#define CON_STATUS_LED DK_LED2//cabiar

#define KEY_PASSKEY_ACCEPT DK_BTN1_MSK
#define KEY_PASSKEY_REJECT DK_BTN2_MSK

#define UART_BUF_SIZE 40 //CONFIG_BT_NUS_UART_BUFFER_SIZE
#define UART_WAIT_FOR_BUF_DELAY K_MSEC(50)
#define UART_WAIT_FOR_RX 50 //CONFIG_BT_NUS_UART_RX_WAIT_TIME

#define BT_ADDR_LE_STR_LEN 30// revisar

static K_SEM_DEFINE(ble_init_ok, 0, 1);

static struct bt_conn *current_conn;
static struct bt_conn *auth_conn;

//static const struct device *uart = DEVICE_DT_GET(DT_CHOSEN(nordic_nus_uart));//Borrar
static struct k_work_delayable uart_work;//Borrar

struct uart_data_t {
	void *fifo_reserved;
	uint8_t data[UART_BUF_SIZE];
	uint16_t len;
};

static K_FIFO_DEFINE(fifo_uart_tx_data);
static K_FIFO_DEFINE(fifo_uart_rx_data);

static const struct bt_data ad[] = {
	BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
	BT_DATA(BT_DATA_NAME_COMPLETE, DEVICE_NAME, DEVICE_NAME_LEN),
};

static const struct bt_data sd[] = {
	BT_DATA_BYTES(BT_DATA_UUID128_ALL, BT_UUID_NUS_VAL),
};

#if CONFIG_BT_NUS_UART_ASYNC_ADAPTER
UART_ASYNC_ADAPTER_INST_DEFINE(async_adapter);
#else
static const struct device *const async_adapter;
#endif


static void uart_cb(const struct device *dev, struct uart_event *evt, void *user_data)
{
	ARG_UNUSED(dev);

	static size_t aborted_len;
	struct uart_data_t *buf;
	static uint8_t *aborted_buf;
	static bool disable_req;

	switch (evt->type) {
	case UART_TX_DONE:
		LOG_DBG("UART_TX_DONE");
		if ((evt->data.tx.len == 0) ||
		    (!evt->data.tx.buf)) {
			return;
		}

		if (aborted_buf) {
			buf = CONTAINER_OF(aborted_buf, struct uart_data_t,
					   data);
			aborted_buf = NULL;
			aborted_len = 0;
		} else {
			buf = CONTAINER_OF(evt->data.tx.buf, struct uart_data_t,
					   data);
		}

		k_free(buf);

		buf = k_fifo_get(&fifo_uart_tx_data, K_NO_WAIT);
		if (!buf) {
			return;
		}

		if (uart_tx(uart_dev, buf->data, buf->len, SYS_FOREVER_MS)) {
			LOG_WRN("Failed to send data over UART");
		}

		break;

	case UART_RX_RDY:
		LOG_DBG("UART_RX_RDY");
		buf = CONTAINER_OF(evt->data.rx.buf, struct uart_data_t, data);
		buf->len += evt->data.rx.len;

		if (disable_req) {
			return;
		}

		if ((evt->data.rx.buf[buf->len - 1] == '\n') ||
		    (evt->data.rx.buf[buf->len - 1] == '\r')) {
			disable_req = true;
			uart_rx_disable(uart_dev);
		}

		break;

	case UART_RX_DISABLED:
		LOG_DBG("UART_RX_DISABLED");
		disable_req = false;

		buf = k_malloc(sizeof(*buf));
		if (buf) {
			buf->len = 0;
		} else {
			LOG_WRN("Not able to allocate UART receive buffer");
			k_work_reschedule(&uart_work, UART_WAIT_FOR_BUF_DELAY);
			return;
		}

		uart_rx_enable(uart_dev, buf->data, sizeof(buf->data),
			       UART_WAIT_FOR_RX);

		break;

	case UART_RX_BUF_REQUEST:
		LOG_DBG("UART_RX_BUF_REQUEST");
		buf = k_malloc(sizeof(*buf));
		if (buf) {
			buf->len = 0;
			uart_rx_buf_rsp(uart_dev, buf->data, sizeof(buf->data));
		} else {
			LOG_WRN("Not able to allocate UART receive buffer");
		}

		break;

	case UART_RX_BUF_RELEASED:
		LOG_DBG("UART_RX_BUF_RELEASED");
		buf = CONTAINER_OF(evt->data.rx_buf.buf, struct uart_data_t,
				   data);

		if (buf->len > 0) {
			k_fifo_put(&fifo_uart_rx_data, buf);
		} else {
			k_free(buf);
		}

		break;

	case UART_TX_ABORTED:
		LOG_DBG("UART_TX_ABORTED");
		if (!aborted_buf) {
			aborted_buf = (uint8_t *)evt->data.tx.buf;
		}

		aborted_len += evt->data.tx.len;
		buf = CONTAINER_OF(aborted_buf, struct uart_data_t,
				   data);

		uart_tx(uart_dev, &buf->data[aborted_len],
			buf->len - aborted_len, SYS_FOREVER_MS);

		break;

	default:
		break;
	}
}

static void uart_work_handler(struct k_work *item)
{
	struct uart_data_t *buf;

	buf = k_malloc(sizeof(*buf));
	if (buf) {
		buf->len = 0;
	} else {
		LOG_WRN("Not able to allocate UART receive buffer");
		k_work_reschedule(&uart_work, UART_WAIT_FOR_BUF_DELAY);
		return;
	}

	uart_rx_enable(uart_dev, buf->data, sizeof(buf->data), UART_WAIT_FOR_RX);
}

static bool uart_test_async_api(const struct device *dev)
{
	const struct uart_driver_api *api =
			(const struct uart_driver_api *)dev->api;

	return (api->callback_set != NULL);
}

static int uart_init(void)
{
	int err;
	int pos;
	struct uart_data_t *rx;
	struct uart_data_t *tx;

	if (!device_is_ready(uart_dev)) {
		return -ENODEV;
	}

	if (IS_ENABLED(CONFIG_USB_DEVICE_STACK)) {
		err = usb_enable(NULL);
		if (err && (err != -EALREADY)) {
			LOG_ERR("Failed to enable USB");
			return err;
		}
	}

	rx = k_malloc(sizeof(*rx));
	if (rx) {
		rx->len = 0;
	} else {
		return -ENOMEM;
	}

	k_work_init_delayable(&uart_work, uart_work_handler);


	if (IS_ENABLED(CONFIG_BT_NUS_UART_ASYNC_ADAPTER) && !uart_test_async_api(uart_dev)) {
		/* Implement API adapter */
		uart_async_adapter_init(async_adapter, uart_dev);
		uart_dev = async_adapter;
	}

	err = uart_callback_set(uart_dev, uart_cb, NULL);
	if (err) {
		LOG_ERR("Cannot initialize UART callback");
		return err;
	}

	if (IS_ENABLED(CONFIG_UART_LINE_CTRL)) {
		LOG_INF("Wait for DTR");
		while (true) {
			uint32_t dtr = 0;

			uart_line_ctrl_get(uart_dev, UART_LINE_CTRL_DTR, &dtr);
			if (dtr) {
				break;
			}
			/* Give CPU resources to low priority threads. */
			k_sleep(K_MSEC(100));
			break;
		}
		LOG_INF("DTR set");
		err = uart_line_ctrl_set(uart_dev, UART_LINE_CTRL_DCD, 1);
		if (err) {
			LOG_WRN("Failed to set DCD, ret code %d", err);
		}
		err = uart_line_ctrl_set(uart_dev, UART_LINE_CTRL_DSR, 1);
		if (err) {
			LOG_WRN("Failed to set DSR, ret code %d", err);
		}
	}

	tx = k_malloc(sizeof(*tx));

	if (tx) {
		pos = snprintf(tx->data, sizeof(tx->data),
			       "Starting Nordic UART service example\r\n");

		if ((pos < 0) || (pos >= sizeof(tx->data))) {
			k_free(tx);
			LOG_ERR("snprintf returned %d", pos);
			return -ENOMEM;
		}

		tx->len = pos;
	} else {
		return -ENOMEM;
	}

	err = uart_tx(uart_dev, tx->data, tx->len, SYS_FOREVER_MS);
	if (err) {
		LOG_ERR("Cannot display welcome message (err: %d)", err);
		return err;
	}

	return uart_rx_enable(uart_dev, rx->data, sizeof(rx->data), 50);
}

static void connected(struct bt_conn *conn, uint8_t err)
{
	char addr[BT_ADDR_LE_STR_LEN];

	if (err) {
		LOG_ERR("Connection failed (err %u)", err);
		return;
	}

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));
	LOG_INF("Connected %s", addr);

	current_conn = bt_conn_ref(conn);

	dk_set_led_on(CON_STATUS_LED);// reparar el led reviara CON_STATUS_LED
}

static void disconnected(struct bt_conn *conn, uint8_t reason)
{
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	LOG_INF("Disconnected: %s (reason %u)", addr, reason);

	if (auth_conn) {
		bt_conn_unref(auth_conn);
		auth_conn = NULL;
	}

	if (current_conn) {
		bt_conn_unref(current_conn);
		current_conn = NULL;
		dk_set_led_off(CON_STATUS_LED); //reparar el led
	}
}

#ifdef CONFIG_BT_NUS_SECURITY_ENABLED
static void security_changed(struct bt_conn *conn, bt_security_t level,
			     enum bt_security_err err)
{
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	if (!err) {
		LOG_INF("Security changed: %s level %u", addr, level);
	} else {
		LOG_WRN("Security failed: %s level %u err %d", addr,
			level, err);
	}
}
#endif

BT_CONN_CB_DEFINE(conn_callbacks) = {
	.connected    = connected,
	.disconnected = disconnected,
#ifdef CONFIG_BT_NUS_SECURITY_ENABLED
	.security_changed = security_changed,
#endif
};

#if defined(CONFIG_BT_NUS_SECURITY_ENABLED)
static void auth_passkey_display(struct bt_conn *conn, unsigned int passkey)
{
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	LOG_INF("Passkey for %s: %06u", addr, passkey);
}

static void auth_passkey_confirm(struct bt_conn *conn, unsigned int passkey)
{
	char addr[BT_ADDR_LE_STR_LEN];

	auth_conn = bt_conn_ref(conn);

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	LOG_INF("Passkey for %s: %06u", addr, passkey);
	LOG_INF("Press Button 1 to confirm, Button 2 to reject.");
}


static void auth_cancel(struct bt_conn *conn)
{
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	LOG_INF("Pairing cancelled: %s", addr);
}


static void pairing_complete(struct bt_conn *conn, bool bonded)
{
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	LOG_INF("Pairing completed: %s, bonded: %d", addr, bonded);
}


static void pairing_failed(struct bt_conn *conn, enum bt_security_err reason)
{
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	LOG_INF("Pairing failed conn: %s, reason %d", addr, reason);
}


static struct bt_conn_auth_cb conn_auth_callbacks = {
	.passkey_display = auth_passkey_display,
	.passkey_confirm = auth_passkey_confirm,
	.cancel = auth_cancel,
};

static struct bt_conn_auth_info_cb conn_auth_info_callbacks = {
	.pairing_complete = pairing_complete,
	.pairing_failed = pairing_failed
};
#else
static struct bt_conn_auth_cb conn_auth_callbacks;
static struct bt_conn_auth_info_cb conn_auth_info_callbacks;
#endif

static void bt_receive_cb(struct bt_conn *conn, const uint8_t *const data,
			  uint16_t len)
{
	int err;
	char addr[BT_ADDR_LE_STR_LEN] = {0};

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, ARRAY_SIZE(addr));

	LOG_INF("Received data from: %s", addr);

	for (uint16_t pos = 0; pos != len;) {
		struct uart_data_t *tx = k_malloc(sizeof(*tx));

		if (!tx) {
			LOG_WRN("Not able to allocate UART send data buffer");
			return;
		}

		/* Keep the last byte of TX buffer for potential LF char. */
		size_t tx_data_size = sizeof(tx->data) - 1;

		if ((len - pos) > tx_data_size) {
			tx->len = tx_data_size;
		} else {
			tx->len = (len - pos);
		}

		memcpy(tx->data, &data[pos], tx->len);

		pos += tx->len;

		/* Append the LF character when the CR character triggered
		 * transmission from the peer.
		 */
		if ((pos == len) && (data[len - 1] == '\r')) {
			tx->data[tx->len] = '\n';
			tx->len++;
		}

		err = uart_tx(uart_dev, tx->data, tx->len, SYS_FOREVER_MS); //uart = uart_dev
		if (err) {
			k_fifo_put(&fifo_uart_tx_data, tx);
		}
	}
}

static struct bt_nus_cb nus_cb = {
	.received = bt_receive_cb,
};

void error(void)
{
	dk_set_leds_state(DK_ALL_LEDS_MSK, DK_NO_LEDS_MSK);//TODO//

	while (true) {
		/* Spin for ever */
		k_sleep(K_MSEC(1000));
	}
}

#ifdef CONFIG_BT_NUS_SECURITY_ENABLED
static void num_comp_reply(bool accept)
{
	if (accept) {
		bt_conn_auth_passkey_confirm(auth_conn);
		LOG_INF("Numeric Match, conn %p", (void *)auth_conn);
	} else {
		bt_conn_auth_cancel(auth_conn);
		LOG_INF("Numeric Reject, conn %p", (void *)auth_conn);
	}

	bt_conn_unref(auth_conn);
	auth_conn = NULL;
}
#endif /* CONFIG_BT_NUS_SECURITY_ENABLED */
#endif //  (BLUETOOTH > 0)

uint32_t button[4] = {0};
void button_changed(uint32_t button_state, uint32_t has_changed)
{
	uint8_t idx = 0;
	switch(has_changed) {
		case DK_BTN1_MSK:
			idx = 0;
			break;
		case DK_BTN2_MSK:
			idx = 1;
			break;
		case DK_BTN3_MSK:
			idx = 2;
			break;
		case DK_BTN4_MSK:
			idx = 3;
			break;
		break;
	}
	if (0 == (button_state & has_changed)) {
		button[idx] = 0;
		dk_set_led_off(idx);
	} else {
		button[idx] = 1;
		dk_set_led_on(idx);
	}


}
#if (BLUETOOTH > 0)

static void configure_gpio(void)
{
	int err;

#ifdef CONFIG_BT_NUS_SECURITY_ENABLED
#endif /* CONFIG_BT_NUS_SECURITY_ENABLED */

}
#endif // BLUETOOTH > 0

void main(void)
{
	uint16_t holding_reg[] = {0x0000,0x03e8,0x0000,0x1388};//{0x0000,0x0000,0x0000,0x0002,0x0006,0x7C28,0x0000,0x07D0,0x0000,0x05DC,0x0000,0x05DC,0x0000,0x038E,0x0000,0x0001};
	const uint8_t coil_qty = 3;
	uint8_t coil[1] = {0};
	const int32_t sleep = 250; //250
	int16_t start_address = 0x1802; //F10
	int16_t start_address1 = 0x007D;//{0x0000,0x1388,0x0000,0x0000,0x0000,0x0000,0x0000,0x0000}; commandez le moteur F06
	int16_t start_address2 = 0x1840;// Lirez les registers, addres du register et # de registers. F03
	int16_t reg_val = 0x0008;
	int16_t num_regs = 0x0006;
	static uint8_t node = 1;
	int err;
	bool comande;

#if (BLUETOOTH > 0)
	uint8_t tx_buf[MSG_SIZE];

	//pour Bluetooth
	int blink_status = 0;
	int errble = 0;

	configure_gpio();

	errble = uart_init();
	if (errble) {
		error();
	}

	if (IS_ENABLED(CONFIG_BT_NUS_SECURITY_ENABLED)) {
		errble = bt_conn_auth_cb_register(&conn_auth_callbacks);
		if (errble) {
			printk("Failed to register authorization callbacks.\n");
			return;
		}

		errble = bt_conn_auth_info_cb_register(&conn_auth_info_callbacks);
		if (errble) {
			printk("Failed to register authorization info callbacks.\n");
			return;
		}
	}

	errble = bt_enable(NULL);
	if (errble) {
		error();
	}

	LOG_INF("Bluetooth initialized");

	k_sem_give(&ble_init_ok);

	if (IS_ENABLED(CONFIG_SETTINGS)) {
		settings_load();
	}

	errble = bt_nus_init(&nus_cb);
	if (errble) {
		LOG_ERR("Failed to initialize UART service (err: %d)", errble);
		return;
	}

	errble = bt_le_adv_start(BT_LE_ADV_CONN, ad, ARRAY_SIZE(ad), sd,
			      ARRAY_SIZE(sd));
	if (errble) {
		LOG_ERR("Advertising failed to start (err %d)", errble);
		return;
	}

	// end Bluetooth
#endif // BLUETOOTH > 0
	//pour PWD


	if (!device_is_ready(led.port)) {
		return;
	}
int ret;

	ret = dk_leds_init(); //Corregir
	if (ret) {
		LOG_ERR("Cannot init LEDs (err: %d)", ret);
	}
	ret = dk_buttons_init(button_changed);
	if (ret) {
		LOG_ERR("Cannot init buttons (err: %d)", ret);
	}

//	ret = gpio_pin_configure_dt(&led, GPIO_OUTPUT_ACTIVE);
//	if (ret < 0) {
//		return;
//	}


#if 0
    // test clignotement led debug
	while (1) {
		ret = gpio_pin_toggle_dt(&led);
		if (err < 0) {
			LOG_ERR("err=%d", err);
			return;
		}
		k_msleep(1000);
	}
#endif

	//end PWD
#if (USB_VCP > 0)
	if (!device_is_ready(uart_dev)) {
		printk("UART device not found!");
//		return;
	}

	/* configure interrupt and callback to receive data */
	uart_irq_callback_user_data_set(uart_dev, serial_cb, NULL);
	uart_irq_rx_enable(uart_dev);

	//print_uart("Hello! I'm your echo bot.\r\n");
	//print_uart("Tell me something and press enter:\r\n");
#endif // BLUETOOTH > 0

// modbus

	if (init_modbus_client()) {
		LOG_ERR("Modbus RTU client initialization failed");
//		return;
	}

//	print_uart("Hello! I'm your echo bot.\r\n");

	/*err = modbus_write_holding_regs(client_iface, node, start_address, holding_reg, ARRAY_SIZE(holding_reg));
	if (err != 0) {
		LOG_ERR("FC16 failed");
//		return;
	}

	k_msleep(sleep);*/

	/*err = modbus_write_holding_reg(client_iface, node, start_address1, reg_val);
	if (err != 0) {
		LOG_ERR("FC06 failed");
//		return;
	}

	k_msleep(sleep);*/

	
	/*err = modbus_read_holding_regs(client_iface, node, start_address2, holding_reg, num_regs);
	if (err != 0) {
		LOG_ERR("FC03 failed with %d", err);
//		return;
	}*/

	LOG_HEXDUMP_INF(holding_reg, sizeof(holding_reg),"WR|RD 1holding register:");
	comande = 1;

	while (true) {
		uint16_t addr = 0;

		err = modbus_read_coils(client_iface, node, 0, coil, coil_qty);
		if (err != 0) {
			LOG_ERR("FC01 failed with %d", err);
//			return;
		}

		//LOG_INF("Coils state 0x%02x", coil[0]);

		err = modbus_write_coil(client_iface, node, addr++, true);
		if (err != 0) {
			LOG_ERR("FC05 failed with %d", err);
//			return;
		}

		k_msleep(sleep);
		err = modbus_write_coil(client_iface, node, addr++, true);
		if (err != 0) {
			LOG_ERR("FC05 failed with %d", err);
//			return;
		}

		k_msleep(sleep);
		err = modbus_write_coil(client_iface, node, addr++, true);
		if (err != 0) {
			LOG_ERR("FC05 failed with %d", err);
//			return;
		}

		k_msleep(sleep);
		err = modbus_read_coils(client_iface, node, 0, coil, coil_qty);
		if (err != 0) {
			LOG_ERR("FC01 failed with %d", err);
//			return;
		}

		//LOG_INF("Coils state 0x%02x", coil[0]);

		coil[0] = 0;
		err = modbus_write_coils(client_iface, node, 0, coil, coil_qty);
		if (err != 0) {
			LOG_ERR("FC15 failed with %d", err);
//			return;
		}

		k_msleep(sleep);


		if(comande){

			/*err = modbus_write_holding_regs(client_iface, node, start_address, holding_reg, ARRAY_SIZE(holding_reg));
			if (err != 0) {
				LOG_ERR("FC16 failed");
//				return;
			}

			k_msleep(sleep);*/

			/*err = modbus_write_holding_reg(client_iface, node, start_address1,reg_val);
			if (err != 0) {
				LOG_ERR("FC06 failed");
//				return;
			}

			k_msleep(sleep);*/

			/*err = modbus_read_holding_regs(client_iface, node, start_address2, holding_reg, ARRAY_SIZE(holding_reg));
			if (err != 0) {
				LOG_ERR("FC03 failed with %d", err);
//				return;
			}*/
			
			//LOG_HEXDUMP_INF(holding_reg, sizeof(holding_reg),"WR|RD 4holding register:");

			//k_msleep(sleep);
			comande = 0;
			//pour PWM
//			for (int num_led = 0; num_led < num_leds; num_led++) {
//			run_led_test(led_pwm, num_led);
//			}
			//the end PWM.

		}

#if (USB_VCP > 0)
		while (k_msgq_get(&uart_msgq, &tx_buf, K_SECONDS(1)) == 0) {
		print_uart("Echo: ");
		print_uart(tx_buf);
		print_uart("\r\n");
		//holding_reg[0] = strtoul(rx_buf[0],NULL,16);// + strtoul(rx_buf[1],NULL,16);


		/*start_address2 = ((int)rx_buf[0]-48)*4096 + ((int)rx_buf[1]-48)*256 + ((int)rx_buf[2]-48)*16 + ((int)rx_buf[3]-48);
		num_regs = ((int)rx_buf[4]-48)*4096 + ((int)rx_buf[5]-48)*256 + ((int)rx_buf[6]-48)*16 + ((int)rx_buf[7]-48);//code F03*/

		reg_val = ((int)rx_buf[0]-48)*4096 + ((int)rx_buf[1]-48)*256 + ((int)rx_buf[2]-48)*16 + ((int)rx_buf[3]-48);//F06

		/*start_address = ((int)rx_buf[0]-48)*4096 + ((int)rx_buf[1]-48)*256 + ((int)rx_buf[2]-48)*16 + ((int)rx_buf[3]-48);//Adrress.

		holding_reg[15]= ((int)rx_buf[64]-48)*4096 + ((int)rx_buf[65]-48)*256 + ((int)rx_buf[66]-48)*16 + ((int)rx_buf[67]-48);
		holding_reg[14] = ((int)rx_buf[60]-48)*4096 + ((int)rx_buf[61]-48)*256 + ((int)rx_buf[62]-48)*16 + ((int)rx_buf[63]-48);
		holding_reg[13]= ((int)rx_buf[56]-48)*4096 + ((int)rx_buf[57]-48)*256 + ((int)rx_buf[58]-48)*16 + ((int)rx_buf[59]-48);
		holding_reg[12] = ((int)rx_buf[52]-48)*4096 + ((int)rx_buf[53]-48)*256 + ((int)rx_buf[54]-48)*16 + ((int)rx_buf[55]-48);
		holding_reg[11]= ((int)rx_buf[48]-48)*4096 + ((int)rx_buf[49]-48)*256 + ((int)rx_buf[50]-48)*16 + ((int)rx_buf[51]-48);
		holding_reg[10] = ((int)rx_buf[44]-48)*4096 + ((int)rx_buf[45]-48)*256 + ((int)rx_buf[46]-48)*16 + ((int)rx_buf[47]-48);
		holding_reg[9]= ((int)rx_buf[40]-48)*4096 + ((int)rx_buf[41]-48)*256 + ((int)rx_buf[42]-48)*16 + ((int)rx_buf[43]-48);
		holding_reg[8] = ((int)rx_buf[36]-48)*4096 + ((int)rx_buf[37]-48)*256 + ((int)rx_buf[38]-48)*16 + ((int)rx_buf[39]-48);
		holding_reg[7]= ((int)rx_buf[32]-48)*4096 + ((int)rx_buf[33]-48)*256 + ((int)rx_buf[34]-48)*16 + ((int)rx_buf[35]-48);
		holding_reg[6] = ((int)rx_buf[28]-48)*4096 + ((int)rx_buf[29]-48)*256 + ((int)rx_buf[30]-48)*16 + ((int)rx_buf[31]-48);
		holding_reg[5]= ((int)rx_buf[24]-48)*4096 + ((int)rx_buf[25]-48)*256 + ((int)rx_buf[26]-48)*16 + ((int)rx_buf[27]-48);
		holding_reg[4] = ((int)rx_buf[20]-48)*4096 + ((int)rx_buf[21]-48)*256 + ((int)rx_buf[22]-48)*16 + ((int)rx_buf[23]-48);
		holding_reg[3]= ((int)rx_buf[16]-48)*4096 + ((int)rx_buf[17]-48)*256 + ((int)rx_buf[18]-48)*16 + ((int)rx_buf[19]-48);
		holding_reg[2] = ((int)rx_buf[12]-48)*4096 + ((int)rx_buf[13]-48)*256 + ((int)rx_buf[14]-48)*16 + ((int)rx_buf[15]-48);
		holding_reg[1]= ((int)rx_buf[8]-48)*4096 + ((int)rx_buf[9]-48)*256 + ((int)rx_buf[10]-48)*16 + ((int)rx_buf[11]-48);
		holding_reg[0] = ((int)rx_buf[4]-48)*4096 + ((int)rx_buf[5]-48)*256 + ((int)rx_buf[6]-48)*16 + ((int)rx_buf[7]-48);//F10*/
		
		//holding_reg[0] = ((int)rx_buf[0]-48)*1000 + ((int)rx_buf[1]-48)*100 + ((int)rx_buf[2]-48)*10 + ((int)rx_buf[3]-48);
		//holding_reg[0] = strtoul(rx_buf,NULL,16);
		//holding_reg[0] = ((uint16_t)tx_buf[0] << 8) | ((uint8_t)tx_buf[1]); //
		//holding_reg[1] = ((uint16_t)tx_buf[2] << 8) | ((uint8_t)tx_buf[3]); //
		//strtoul(rx_buf,2,10);// >> strtoul(rx_buf,3,10);
		comande = 1;
		LOG_HEXDUMP_INF(holding_reg, sizeof(holding_reg),"WR|RD 3holding register:");
		k_msleep(sleep);
		}
#endif (USB_VCP > 0)

	}

	//for (;;) {
		//dk_set_led(RUN_STATUS_LED, (++blink_status) % 2);
		//k_sleep(K_MSEC(RUN_LED_BLINK_INTERVAL));
	//}
}

#if (BLUETOOTH > 0)
void ble_write_thread(void)
{
	/* Don't go any further until BLE is initialized */
	k_sem_take(&ble_init_ok, K_FOREVER);
	//int16_t reg_val = 0x0008;
	//int err = 0;

	for (;;) {
		/* Wait indefinitely for data to be sent over bluetooth */
		struct uart_data_t *buf = k_fifo_get(&fifo_uart_rx_data,
						     K_FOREVER);

		if (bt_nus_send(NULL, buf->data, buf->len)) {
			LOG_WRN("Failed to send data over BLE connection");
		}
		//reg_val = ((int)rx_buf[0]-48)*4096 + ((int)rx_buf[1]-48)*256 + ((int)rx_buf[2]-48)*16 + ((int)rx_buf[3]-48);

		//err = modbus_write_holding_reg(client_iface, node, start_address1,reg_val);
		//if (err != 0) {
		//	LOG_ERR("FC06 failed");
		//	return;
		//}


		k_free(buf);
	}
}

K_THREAD_DEFINE(ble_write_thread_id, STACKSIZE, ble_write_thread, NULL, NULL,
		NULL, PRIORITY, 0, 0);

#endif // BLUETOOTH > 0