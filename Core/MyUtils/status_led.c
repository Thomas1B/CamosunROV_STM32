/*
 * status_led.c
 *
 *  Author: Thomas Bourgeois
 *
 * Status LED scheme for the ROV. See status_led.h for the full table.
 *
 * How it works:
 *   - Every LED state is stored as a 3-bit "mask" (one bit per LED):
 *       bit 0 = green, bit 1 = yellow, bit 2 = red   (1 = on, 0 = off)
 *     e.g. mask = 0b101 -> green on, yellow off, red on.
 *   - status_led_pattern() decides WHICH LEDs should be on (no hardware).
 *   - set_leds() takes that mask and actually drives the GPIO pins.
 */

#include "status_led.h"
#include "main.h" // LED pin names (GREEN_LED_Pin etc.) and HAL

/* ---------------------------------------------------------------------------
 * Power-up timing
 * ------------------------------------------------------------------------- */
#define LAMP_TEST_ON_MS     1000U // all three on
#define LAMP_TEST_GAP_MS    250U  // all off before green comes on for sensor init
#define BOOT_DONE_TOGGLES   12U   // fast flicker at end of boot (even = ends on)
#define BOOT_DONE_STEP_MS   50U   // time between toggles (12 x 50 ms = 0.6 s)

/**
 * @brief Writes all three LEDs from a bit mask.
 *
 * For each LED, (mask & LED_x_BIT) is non-zero if that LED's bit is set.
 * The "? :" picks GPIO_PIN_SET (pin high, LED on) when the bit is set,
 * otherwise GPIO_PIN_RESET (pin low, LED off). Assumes active-high LEDs.
 *
 * @param mask  combination of LED_GREEN_BIT / LED_YELLOW_BIT / LED_RED_BIT
 */
static void set_leds(uint8_t mask) {
	HAL_GPIO_WritePin(GREEN_LED_GPIO_Port, GREEN_LED_Pin,
			(mask & LED_GREEN_BIT) ? GPIO_PIN_SET : GPIO_PIN_RESET);
	HAL_GPIO_WritePin(YELLOW_LED_GPIO_Port, YELLOW_LED_Pin,
			(mask & LED_YELLOW_BIT) ? GPIO_PIN_SET : GPIO_PIN_RESET);
	HAL_GPIO_WritePin(RED_LED_GPIO_Port, RED_LED_Pin,
			(mask & LED_RED_BIT) ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

/* ---------------------------------------------------------------------------
 * Power-up sequence (blocking, called once from main() before the main loop)
 * ------------------------------------------------------------------------- */

/**
 * @brief Lamp test: all three LEDs on for 1 s so you can see every LED works,
 *        then all off for 250 ms so the next step is clearly separate.
 */
void status_led_lamp_test(void) {
	set_leds(LED_ALL_BITS);       // green + yellow + red on
	HAL_Delay(LAMP_TEST_ON_MS);
	set_leds(0U);                 // all off
	HAL_Delay(LAMP_TEST_GAP_MS);
}

/**
 * @brief Green on (solid) while the sensors initialize. Returns right away;
 *        green stays on until status_led_boot_done() is called.
 */
void status_led_boot_start(void) {
	set_leds(LED_GREEN_BIT); // stays on while the sensors initialize
}

/**
 * @brief Fast green flicker to show boot is finished.
 *
 * Starts with green on, then toggles it BOOT_DONE_TOGGLES times. An even
 * number of toggles means green ends up back on. Takes 12 x 50 ms = 0.6 s.
 */
void status_led_boot_done(void) {
	set_leds(LED_GREEN_BIT);                                 // start from green on
	for (uint8_t i = 0; i < BOOT_DONE_TOGGLES; i++) {
		HAL_GPIO_TogglePin(GREEN_LED_GPIO_Port, GREEN_LED_Pin); // on -> off -> on ...
		HAL_Delay(BOOT_DONE_STEP_MS);
	}
}

/* ---------------------------------------------------------------------------
 * Leak
 * ------------------------------------------------------------------------- */

/**
 * @brief Shows the LEAK pattern: red only, solid. Called directly by the
 *        boot leak check and emergency_shutdown() so it shows immediately.
 */
void status_led_show_leak(void) {
	set_leds(LED_RED_BIT);
}

/* ---------------------------------------------------------------------------
 * Normal operation (after boot)
 * ------------------------------------------------------------------------- */

/**
 * @brief Decides which LEDs should be on for the current state.
 *
 * Pure function: no hardware access, so the table can be tested on a PC.
 * Each LED has one job, and their results are OR-ed (|=) into the mask so
 * statuses combine (e.g. green blinking + yellow = waiting, sensor fault).
 *
 *   Green  solid    = Pi connected
 *   Green  blinking = waiting for the Pi
 *   Red    blinking = Pi link lost
 *   Yellow solid    = I2C sensor fault
 *   Red    solid    = LEAK (only red, overrides everything)
 *
 * @param leak       true if a leak is latched
 * @param link       state of the Pi link (waiting / connected / lost)
 * @param i2c_fault  true if any I2C sensor (IMU / depth / temp) is faulted now
 * @param blink_on   true during the "on" half of the 1 Hz blink
 * @return mask of LED_GREEN_BIT / LED_YELLOW_BIT / LED_RED_BIT
 */
uint8_t status_led_pattern(bool leak, led_link_state_t link, bool i2c_fault,
		bool blink_on) {
	// LEAK wins over everything else: return red only and skip the rest.
	if (leak) {
		return LED_RED_BIT; // LEAK: only red, solid. Overrides everything.
	}

	uint8_t mask = 0U; // start with all LEDs off, then add bits below

	/* Green and red LEDs: show the state of the UART link to the Pi */
	switch (link) {
	case LED_LINK_CONNECTED:
		mask |= LED_GREEN_BIT;                  // green solid
		break;
	case LED_LINK_LOST:
		// Red only during the "on" half of the blink -> red blinks at 1 Hz
		if (blink_on) {
			mask |= LED_RED_BIT;                // red blinking
		}
		break;
	case LED_LINK_WAITING:
	default:
		// Green only during the "on" half of the blink -> green blinks at 1 Hz
		if (blink_on) {
			mask |= LED_GREEN_BIT;              // green blinking
		}
		break;
	}

	/* Yellow: I2C sensor fault (independent of the link state) */
	if (i2c_fault) {
		mask |= LED_YELLOW_BIT;                 // yellow solid
	}

	return mask;
}

/**
 * @brief Updates the LEDs for the current state. Call every main-loop pass.
 *
 * Blink timing (non-blocking, no timer or delay needed):
 *   HAL_GetTick() counts milliseconds since boot. Dividing by 500 gives the
 *   number of 500 ms half-periods so far. "& 1U" keeps only the lowest bit,
 *   which alternates 0, 1, 0, 1 ... every 500 ms:
 *     0 -> "on" half  (blink_on = true)
 *     1 -> "off" half (blink_on = false)
 *   So any blinking LED is on 500 ms, off 500 ms = 1 Hz.
 *
 * Because the phase comes from the system clock, green and red always blink
 * in step, and the blink keeps going even if this is called at uneven times.
 */
void status_led_update(bool leak, led_link_state_t link, bool i2c_fault) {
	bool blink_on = ((HAL_GetTick() / STATUS_BLINK_HALF_PERIOD_MS) & 1U) == 0U;
	set_leds(status_led_pattern(leak, link, i2c_fault, blink_on));
}