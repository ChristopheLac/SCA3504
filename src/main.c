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

#include <stdio.h> // BLuetooth
#include "commandUsb.h"

str_exchange_table exchange_table;

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

LOG_MODULE_REGISTER(main, LOG_LEVEL_INF);

uint32_t button[4] = {0};
void button_changed(uint32_t button_state, uint32_t has_changed)
{
	uint8_t idx = 0;
	switch (has_changed)
	{
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
	if (0 == (button_state & has_changed))
	{
		button[idx] = 0;
		dk_set_led_off(idx);
	}
	else
	{
		button[idx] = 1;
		dk_set_led_on(idx);
	}
}
#include "moteur.h"
// void main_usb(void);
void main(void)
{
	int ret;

	if (!device_is_ready(led.port))
	{
		return;
	}

	ret = dk_leds_init(); // Corregir
	if (ret)
	{
		LOG_ERR("Cannot init LEDs (err: %d)", ret);
	}
	ret = dk_buttons_init(button_changed);
	if (ret)
	{
		LOG_ERR("Cannot init buttons (err: %d)", ret);
	}
	// main_usb();

	uint8_t rampe = 10;
	while (1)
	{
		// ret = gpio_pin_toggle_dt(&led);
		// k_sleep(K_MSEC(1000));
		gpio_pin_toggle_dt(&led);
#if 0
k_sleep(K_MSEC(2000));
// 			moteur_setPositionSens(0, false, eSensHoraire, 0, 0, 0);
// 			gpio_pin_toggle_dt(&led);
// k_sleep(K_MSEC(10000));
			moteur_setPositionSens(9000, true, eSensHoraire, 0, 0, 0);
			gpio_pin_toggle_dt(&led);
 k_sleep(K_MSEC(2000));
  			moteur_setPositionSens(18000, true, eSensAntiHoraire, 0, 0, 0);
 			gpio_pin_toggle_dt(&led);
k_sleep(K_MSEC(2000));
 			moteur_setPositionSens(9000, true, eSensAntiHoraire, 0, 0, 0);
 			gpio_pin_toggle_dt(&led);
k_sleep(K_MSEC(2000));
//			moteur_setPositionSens(0, false, eSensHoraire, 0, 0, 0);
// 			gpio_pin_toggle_dt(&led);
// k_sleep(K_MSEC(10000));
			// gpio_pin_toggle_dt(&led);
#endif
		k_sleep(K_MSEC(1000));
		while (true != moteur_getArrivePositionFin())
		{
			k_sleep(K_MSEC(1000));
		}

		k_sleep(K_MSEC(2000));
		moteur_setPositionSens(36000 * 1, true, eSensHoraire, 1, rampe, 0);
		gpio_pin_toggle_dt(&led);

		k_sleep(K_MSEC(1000));
		while (true != moteur_getArrivePositionFin())
		{
			k_sleep(K_MSEC(1000));
		}

		k_sleep(K_MSEC(2000));
		moteur_setPositionSens(36000 * 2, true, eSensHoraire, 5, rampe, 0);
		gpio_pin_toggle_dt(&led);

		k_sleep(K_MSEC(1000));
		while (true != moteur_getArrivePositionFin())
		{
			k_sleep(K_MSEC(1000));
		}

		k_sleep(K_MSEC(2000));
		moteur_setPositionSens(36000 * 3, true, eSensHoraire, 10, rampe, 0);
		gpio_pin_toggle_dt(&led);

		k_sleep(K_MSEC(1000));
		while (true != moteur_getArrivePositionFin())
		{
			k_sleep(K_MSEC(1000));
		}

		k_sleep(K_MSEC(2000));
		moteur_setPositionSens(36000 * 4, true, eSensHoraire, 50, rampe, 0);
		gpio_pin_toggle_dt(&led);

		k_sleep(K_MSEC(1000));
		while (true != moteur_getArrivePositionFin())
		{
			k_sleep(K_MSEC(1000));
		}

		k_sleep(K_MSEC(2000));
		moteur_setPositionSens(36000 * 5, true, eSensHoraire, 100, rampe, 0);
		gpio_pin_toggle_dt(&led);
	}

	rampe += 10;
	if (rampe > 100)
	{
		rampe = 10;
	}
}
