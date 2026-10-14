/*
 * status_led.c
 *
 *  Author: Thomas Bourgeois
 *
 * Status LED scheme for the ROV. See status_led.h for the full table.
 */

#include "status_led.h"
#include "main.h" // LED pin names (GREEN_LED_Pin etc.) and HAL

/* Power-up timing */
#define LAMP_TEST_ON_MS     1000U // all three on
#define LAMP_TEST_GAP_MS    250U  // all off before the green blink starts
#define BOOT_BLINK_MS       250U  // green on / off time of the power-up blink
#define BOOT_DONE_TOGGLES   12U   // fast flicker at end of boot (even = ends on)
#define BOOT_DONE_STEP_MS   50U

/* Writes all three LEDs from a bit mask. */
static void set_leds(uint8_t mask) {
	HAL_GPIO_WritePin(GREEN_LED_GPIO_Port, GREEN_LED_Pin,
			(mask & LED_GREEN_BIT) ? GPIO_PIN_SET : GPIO_PIN_RESET);
	HAL_GPIO_WritePin(YELLOW_LED_GPIO_Port, YELLOW_LED_Pin,
			(mask & LED_YELLOW_BIT) ? GPIO_PIN_SET : GPIO_PIN_RESET);
	HAL_GPIO_WritePin(RED_LED_GPIO_Port, RED_LED_Pin,
			(mask & LED_RED_BIT) ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

void status_led_lamp_test(void) {
	set_leds(LED_ALL_BITS);
	HAL_Delay(LAMP_TEST_ON_MS);
	set_leds(0U);
	HAL_Delay(LAMP_TEST_GAP_MS);
}

void status_led_boot_start(void) {
	set_leds(LED_GREEN_BIT);
	HAL_Delay(BOOT_BLINK_MS);
	set_leds(0U);
	HAL_Delay(BOOT_BLINK_MS);
	set_leds(LED_GREEN_BIT); // stays on while the sensors initialize
}

void status_led_boot_done(void) {
	set_leds(LED_GREEN_BIT);
	for (uint8_t i = 0; i < BOOT_DONE_TOGGLES; i++) {
		HAL_GPIO_TogglePin(GREEN_LED_GPIO_Port, GREEN_LED_Pin);
		HAL_Delay(BOOT_DONE_STEP_MS);
	}
}

void status_led_show_leak(void) {
	set_leds(LED_ALL_BITS);
}

uint8_t status_led_pattern(bool leak, led_link_state_t link, bool i2c_fault) {
	if (leak) {
		return LED_ALL_BITS; // LEAK overrides everything
	}

	uint8_t mask;
	switch (link) {
	case LED_LINK_CONNECTED:
		mask = LED_GREEN_BIT;
		break;
	case LED_LINK_LOST:
		mask = LED_RED_BIT;
		break;
	case LED_LINK_WAITING:
	default:
		mask = LED_GREEN_BIT | LED_RED_BIT;
		break;
	}

	if (i2c_fault) {
		mask |= LED_YELLOW_BIT;
	}

	/* Exception: waiting + I2C fault would be all three on, which is the
	 * LEAK pattern. Show yellow alone instead. */
	if (mask == LED_ALL_BITS) {
		mask = LED_YELLOW_BIT;
	}

	return mask;
}

void status_led_update(bool leak, led_link_state_t link, bool i2c_fault) {
	set_leds(status_led_pattern(leak, link, i2c_fault));
}
