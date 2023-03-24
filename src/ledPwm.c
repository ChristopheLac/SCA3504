/*
 * Copyright (c) 2022 Libre Solar Technologies GmbH
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/sys/util.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/devicetree.h> // PWM
#include <errno.h>
#include <zephyr/drivers/led.h>
#include <zephyr/sys/util.h>

#include <zephyr/logging/log.h>

#include <zephyr/types.h>
#include <soc.h>
#include <dk_buttons_and_leds.h>
#include <zephyr/settings/settings.h>

#include <zephyr/drivers/pwm.h>


LOG_MODULE_REGISTER(log_ledPwm, LOG_LEVEL_INF);

// Code pour pwm

#define LED_PWM_NODE_ID DT_COMPAT_GET_ANY_STATUS_OKAY(pwm_leds)

const char *led_label[] = {
	DT_FOREACH_CHILD_SEP_VARGS(LED_PWM_NODE_ID, DT_PROP_OR, (, ), label, NULL)};

const int num_leds = ARRAY_SIZE(led_label);

#define MAX_BRIGHTNESS 100

#define FADE_DELAY_MS 10
#define FADE_DELAY K_MSEC(FADE_DELAY_MS)

#include "commandUsb.h"
/**
 * @brief Run tests on a single LED using the LED API syscalls.
 *
 */
static void ledPuissance(void)
{
	int err;
	uint16_t level = 0;
	uint16_t levelEnd = 0;

	const struct device *led_pwm;
	uint8_t led = 0;
	exchange_table.Light = 20;

	led_pwm = DEVICE_DT_GET(LED_PWM_NODE_ID);

	if (!device_is_ready(led_pwm))
	{
		LOG_ERR("Device %s is not ready", led_pwm->name);
		return;
	}

	if (!num_leds)
	{
		LOG_ERR("No LEDs found for %s", led_pwm->name);
		return;
	}

	while (1)
	{
		if ((exchange_table.Light) != levelEnd)
		{
			//	LOG_INF("change LED %d - %s", led, led_label[led] ? : "no label");
			levelEnd = exchange_table.Light;
			/* Increase LED brightness gradually up to the level. */
			while (levelEnd != level)
			{
				err = led_set_brightness(led_pwm, led, 100 - level);
				if (err < 0)
				{
					LOG_ERR("err=%d brightness=%d\n", err, level);
					return;
				}
				k_sleep(FADE_DELAY);
				if (levelEnd > level)
				{
					level++;
				}
				else
				{
					level--;
				}
			}
			k_sleep(K_MSEC(1000));
		} else {
			k_sleep(K_MSEC(1000));
		}
	}
}
/* size of stack area used by each thread */
#define STACKSIZE 1024
/* scheduling priority used by each thread */
#define PRIORITY 5

	K_THREAD_DEFINE(ledPuissance_id, STACKSIZE, ledPuissance, NULL, NULL, NULL, PRIORITY, 0, 0);
