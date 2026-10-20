/*
 * status_led.h
 *
 *  Author: Thomas Bourgeois
 *
 * Drives the three status LEDs on the custom PCB (GREEN PC1, YELLOW PC2,
 * RED PC0) according to the ROV status LED scheme.
 *
 * Power-up:
 *   1. status_led_lamp_test()  - all three on ~1 s, proves every LED works
 *   2. status_led_boot_start() - green on (solid) while the sensors initialize
 *   3. status_led_boot_done()  - fast green flicker, boot finished
 *
 * After boot, call status_led_update() every main-loop pass (non-blocking).
 * Each LED shows one thing, so statuses combine:
 *
 *   Green   solid           Pi connected
 *   Green   blinking 1 Hz   Waiting for Pi (no valid command received yet)
 *   Red     blinking 1 Hz   Pi link lost (commands received, then timed out)
 *   Yellow  solid           I2C sensor fault (IMU / depth / temp)
 *   Red     solid           LEAK (only red is lit, overrides everything)
 *   all off                 STM32 not running
 *
 * e.g. green blinking + yellow solid = waiting for Pi with a sensor fault.
 * "Connected" = valid commands from the Pi over UART4 within the last
 * 150 ms (CMD_TIMEOUT_MS in main.c).
 *
 * Note: if the firmware hangs, the LEDs freeze on their last pattern (a
 * blinking LED stops blinking, either on or off).
 */

#ifndef MYUTILS_STATUS_LED_H_
#define MYUTILS_STATUS_LED_H_

#include <stdbool.h>
#include <stdint.h>

/** State of the UART link to the Pi, as shown on the green/red LEDs. */
typedef enum {
	LED_LINK_WAITING = 0, // booted, no valid command received from the Pi yet
	LED_LINK_CONNECTED,   // commands arriving normally
	LED_LINK_LOST,        // commands were received, then timed out
} led_link_state_t;

/* Bit mask of which LEDs are on (returned by status_led_pattern()). */
#define LED_GREEN_BIT  (1U << 0)
#define LED_YELLOW_BIT (1U << 1)
#define LED_RED_BIT    (1U << 2)
#define LED_ALL_BITS   (LED_GREEN_BIT | LED_YELLOW_BIT | LED_RED_BIT)

/* Status blink: 1 Hz, 50 % duty (500 ms on, 500 ms off). */
#define STATUS_BLINK_HALF_PERIOD_MS 500U

/**
 * @brief Lamp test: all three LEDs on for ~1 s, then all off briefly.
 *        Blocking (~1.25 s). Call once at power-up.
 */
void status_led_lamp_test(void);

/**
 * @brief Green on (solid) while the sensors initialize. Non-blocking.
 *        Call after the lamp test.
 */
void status_led_boot_start(void);

/**
 * @brief Fast green flicker to show boot is finished. Blocking (~0.6 s).
 */
void status_led_boot_done(void);

/**
 * @brief Shows the LEAK pattern (red only, solid) immediately.
 *        Safe to call from the boot leak check and emergency_shutdown().
 */
void status_led_show_leak(void);

/**
 * @brief Works out which LEDs should be on for a given system state.
 *        Pure function (no hardware access), so the table can be checked
 *        on its own.
 *
 * @param leak       true if a leak is latched (overrides everything)
 * @param link       state of the Pi link
 * @param i2c_fault  true if any I2C sensor (IMU / depth / temp) is faulted now
 * @param blink_on   current phase of the 1 Hz blink (true = "on" half)
 * @return mask of LED_GREEN_BIT / LED_YELLOW_BIT / LED_RED_BIT
 */
uint8_t status_led_pattern(bool leak, led_link_state_t link, bool i2c_fault,
		bool blink_on);

/**
 * @brief Sets the LEDs for the current state, including the 1 Hz blink
 *        (timed from HAL_GetTick()). Non-blocking; call every main-loop
 *        pass after boot.
 */
void status_led_update(bool leak, led_link_state_t link, bool i2c_fault);

#endif /* MYUTILS_STATUS_LED_H_ */