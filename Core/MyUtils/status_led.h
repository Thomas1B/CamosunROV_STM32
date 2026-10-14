/*
 * status_led.h
 *
 *  Author: Thomas Bourgeois
 *
 * Drives the three status LEDs on the custom PCB (GREEN PC1, YELLOW PC2,
 * RED PC0) according to the ROV status LED scheme.
 *
 * Power-up (the only time any LED blinks):
 *   1. status_led_lamp_test()  - all three on ~1 s, proves every LED works
 *   2. status_led_boot_start() - green blinks, then stays on during sensor init
 *   3. status_led_boot_done()  - fast green flicker, boot finished
 *
 * After boot, call status_led_update() every main-loop pass. All patterns
 * are solid (no blinking):
 *
 *   State                         Green  Yellow  Red
 *   Connected, sensors OK           on    off    off
 *   Connected, I2C fault            on    on     off
 *   Waiting for Pi, sensors OK      on    off    on
 *   Waiting for Pi, I2C fault       off   on     off   (exception, see below)
 *   Link lost, sensors OK           off   off    on
 *   Link lost, I2C fault            off   on     on
 *   LEAK                            on    on     on    (overrides everything)
 *   No power                        off   off    off
 *
 * Rules: yellow = I2C sensor fault (IMU / depth / temp); green = Pi
 * connected; red = Pi link lost; green + red = waiting for the Pi.
 * Exception: "waiting + I2C fault" would be all three on, but that pattern
 * belongs to LEAK, so it shows yellow alone.
 *
 * Note: if the firmware hangs, the LEDs freeze on their last pattern.
 * All off only means no power.
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

/**
 * @brief Lamp test: all three LEDs on for ~1 s, then all off briefly.
 *        Blocking (~1.25 s). Call once at power-up.
 */
void status_led_lamp_test(void);

/**
 * @brief Power-up blink: green on, off, then on and left on while the
 *        sensors initialize. Blocking (~0.5 s). Call after the lamp test.
 */
void status_led_boot_start(void);

/**
 * @brief Fast green flicker to show boot is finished. Blocking (~0.6 s).
 */
void status_led_boot_done(void);

/**
 * @brief Shows the LEAK pattern (all three on) immediately.
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
 * @return mask of LED_GREEN_BIT / LED_YELLOW_BIT / LED_RED_BIT
 */
uint8_t status_led_pattern(bool leak, led_link_state_t link, bool i2c_fault);

/**
 * @brief Sets the LEDs to the solid pattern for the current state.
 *        Non-blocking; call every main-loop pass after boot.
 */
void status_led_update(bool leak, led_link_state_t link, bool i2c_fault);

#endif /* MYUTILS_STATUS_LED_H_ */
