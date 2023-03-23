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

LOG_MODULE_REGISTER(log_ledPwm, LOG_LEVEL_INF);

//Code pour pwm

#define LED_PWM_NODE_ID	 DT_COMPAT_GET_ANY_STATUS_OKAY(pwm_leds)

const char *led_label[] = {
	DT_FOREACH_CHILD_SEP_VARGS(LED_PWM_NODE_ID, DT_PROP_OR, (,), label, NULL)
};

const int num_leds = ARRAY_SIZE(led_label);

#define MAX_BRIGHTNESS	100

#define FADE_DELAY_MS	10
#define FADE_DELAY	K_MSEC(FADE_DELAY_MS)

/**
 * @brief Run tests on a single LED using the LED API syscalls.
 *
 */
static void run_led_test(void)
{
	int err;
	uint16_t level;

	const struct device *led_pwm;
	uint8_t led = 0;

	led_pwm = DEVICE_DT_GET(LED_PWM_NODE_ID);

	if (!device_is_ready(led_pwm)) {
		LOG_ERR("Device %s is not ready", led_pwm->name);
		return;
	}

	if (!num_leds) {
		LOG_ERR("No LEDs found for %s", led_pwm->name);
		return;
	}

while(1) {
	LOG_INF("Testing LED %d - %s", led, led_label[led] ? : "no label");

	/* Turn LED on. */
	err = led_on(led_pwm, led);
	if (err < 0) {
		LOG_ERR("err=%d", err);
		return;
	}
	LOG_INF("  Turned on");
	k_sleep(K_MSEC(1000));

	/* Turn LED off. */
	err = led_off(led_pwm, led);
	if (err < 0) {
		LOG_ERR("err=%d", err);
		return;
	}
	LOG_INF("  Turned off");
	k_sleep(K_MSEC(1000));

	/* Increase LED brightness gradually up to the maximum level. */
	LOG_INF("  Increasing brightness gradually");
	for (level = 0; level <= MAX_BRIGHTNESS; level++) {
		err = led_set_brightness(led_pwm, led, level);
		if (err < 0) {
			LOG_ERR("err=%d brightness=%d\n", err, level);
			return;
		}
		k_sleep(FADE_DELAY);
	}
	k_sleep(K_MSEC(1000));

	/* Set LED blinking (on: 0.1 sec, off: 0.1 sec) */
	err = led_blink(led_pwm, led, 100, 100);
	if (err < 0) {
		LOG_ERR("err=%d", err);
		return;
	}
	LOG_INF("  Blinking on: 0.1 sec, off: 0.1 sec");
	k_sleep(K_MSEC(5000));

	/* Enable LED blinking (on: 1 sec, off: 1 sec) */
	/*err = led_blink(led_pwm, led, 65, 131);
	if (err < 0) {
		LOG_ERR("err=%d", err);
		LOG_INF("  Cycle period not supported - on: 1 sec, off: 1 sec");
	} else {
		LOG_INF("  Blinking on: 1 sec, off: 1 sec");
	}
	k_sleep(K_MSEC(5000));*/

	/* Turn LED off. */
	err = led_off(led_pwm, led);
	if (err < 0) {
		LOG_ERR("err=%d", err);
		return;
	}
	LOG_INF("  Turned off, loop end");

	k_sleep(K_SECONDS(5));
}
}
/* size of stack area used by each thread */
#define STACKSIZE 1024
/* scheduling priority used by each thread */
#define PRIORITY 5

K_THREAD_DEFINE(run_led_test_id, STACKSIZE, run_led_test, NULL, NULL, NULL, PRIORITY, 0, 0);
