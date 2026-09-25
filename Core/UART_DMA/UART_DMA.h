/*
 * UART_DMA.h
 *
 *  Created on: September 23, 2026
 *      Author: Thomas Bourgeois
 *
 *  Public interface for the serial protocol between the STM32 and the
 *  Raspberry Pi (UART4, 115200 8N1, DMA in both directions).
 *
 *  Declares:
 *    - Frame constants: PROTO_START_BYTE, CMD_FRAME_SIZE (10),
 *      TELEM_FRAME_SIZE (18), PROTO_RX_DMA_SIZE (64)
 *    - cmd_data_t   : decoded command frame (Pi -> STM32)
 *    - telem_data_t : telemetry values to send (STM32 -> Pi)
 *    - RX functions : proto_start_rx, proto_rx_event_handler,
 *                     proto_rx_error_handler, proto_rx_keepalive
 *    - TX functions : proto_build_telem_frame, proto_send_telem
 *    - proto_checksum8, shared by both directions
 *
 *  See UART_DMA.c for implementation details and the call order.
 *  Keep this file in sync with UART_DMA.py on the Pi side.
 *
 * ==========================================================================
 *  Design
 *
 *  Fixed-length frames, FULLY DMA-based reception using idle-line
 *  detection (HAL_UARTEx_ReceiveToIdle_DMA). Manual byte packing, no
 *  __attribute__((packed)).
 *
 *  How reception works:
 *    - DMA receives into a 64-byte buffer; the CPU never touches individual
 *      bytes while they arrive.
 *    - The Pi sends one 10-byte command frame every 20 ms (50 Hz). A frame
 *      takes ~0.87 ms on the wire at 115200 bit/s, so there is ~19 ms of
 *      silence after each one.
 *    - The UART hardware raises an IDLE event once the line has been quiet
 *      for one character time (~87 us). HAL then stops the DMA and calls
 *      HAL_UARTEx_RxEventCallback() with the number of bytes received.
 *    - That chunk is scanned for complete frames (START + valid checksum),
 *      then DMA is re-armed. Parsing takes microseconds, well inside the
 *      ~19 ms gap, so no bytes are lost.
 *    - Each chunk is handled ON ITS OWN. A broken/incomplete frame is simply
 *      ignored -- nothing is carried over or combined with the next chunk;
 *      reception just waits for the next full message.
 *    - If several complete frames arrive back-to-back in one chunk, the LAST
 *      valid one wins (freshest command).
 *
 * ==========================================================================
 *  Wire formats
 *
 *  Pi -> STM32 (command), 10 bytes:
 *    [ START ][ motor1..motor6 ][ cam_tilt_dc ][ CHECKSUM ]
 *       1 B        6x 1 B         2 B (i16)       1 B
 *
 *  STM32 -> Pi (telemetry), 18 bytes:
 *    [ START ][ depth_dm ][ water_temp_dc ][ battery_dv ][ inside_temp_dc ]
 *       1 B      2 B (u16)     2 B (i16)       2 B (u16)      2 B (i16)
 *    [ heading_dc ][ roll_dc ][ pitch_dc ][ leak ][ fault_flags ][ CHECKSUM ]
 *       2 B (u16)    2 B (i16)   2 B (i16)  1 B (u8)   1 B (u8)       1 B
 *
 *  cam_tilt_dc: camera tilt x10 (deci-degrees), 0 = level, + = up.
 *               No range clamping is done in this protocol layer -- the
 *               motor and tilt control functions clamp their own inputs.
 *  leak:        live leak-sensor state, 0 = dry, 1 = water detected NOW.
 *  fault_flags: latched SystemState_t bit flags (SYS_STATE_FAULT_LEAK,
 *               _IMU, _DEPTH, _TEMP, _UART), OR'ed together into one byte.
 *               Stored as uint8_t, since a combination of flags (e.g.
 *               LEAK | TEMP = 0x09) is not itself one of the enum's values.
 *
 *  All multi-byte values little-endian. Tilt and telemetry measurements
 *  are scaled x10 ("deci-units"); motors are plain percent (-100..100).
 *  CHECKSUM = sum of all bytes between START and CHECKSUM, mod 256.
 *
 * ==========================================================================
 *  CubeMX requirements (UART4)
 *
 *    - Mode Asynchronous, 115200 bit/s, 8 bits, no parity, 1 stop bit
 *    - DMA: UART4_RX (DMA1 Stream 2) and UART4_TX (DMA1 Stream 4), both NORMAL
 *    - NVIC: UART4 global interrupt ENABLED (the IDLE event arrives here)
 *    - Needs STM32CubeF4 firmware package v1.26.0 or newer (ReceiveToIdle API)
 */

#ifndef UART_DMA_H
#define UART_DMA_H

#include <stdint.h>
#include "stm32f4xx_hal.h"   /* for UART_HandleTypeDef, HAL_StatusTypeDef */

#define PROTO_START_BYTE   0xAA

#define CMD_FRAME_SIZE     10   /* START + 6 motors + tilt(2) + checksum */
#define TELEM_FRAME_SIZE   18   /* START + 7x 2-byte fields + leak + faults + checksum */

#define PROTO_RX_DMA_SIZE  64   /* DMA chunk buffer; > several frames */

typedef struct {
	int8_t motor[6]; /* -100..100 (%), one per thruster */
	int16_t tilt_dc; /* camera tilt x10, deci-degrees */
} cmd_data_t;

typedef struct {
	uint16_t depth_dm; /* depth x10, 0-3000 (0-300.0 m) */
	int16_t water_temp_dc; /* water temperature x10, deci-degrees C */
	uint16_t battery_dv; /* battery voltage x10, deci-volts */
	int16_t inside_temp_dc; /* enclosure temperature x10, deci-degrees C */
	uint16_t heading_dc; /* BNO055 heading x10, 0-3600 */
	int16_t roll_dc; /* BNO055 roll x10, ~-900..900 */
	int16_t pitch_dc; /* BNO055 pitch x10, ~-1800..1800 */
	uint8_t leak; /* 0 = dry, 1 = leak detected right now */
	uint8_t fault_flags; /* latched SystemState_t flags, OR'ed */
} telem_data_t;

/* Additive checksum: sum of `len` bytes at `data`, mod 256. */
uint8_t proto_checksum8(const uint8_t *data, uint16_t len);

/*
 * proto_start_rx()
 * Call once at startup. Starts idle-line DMA reception and disables the
 * DMA half-transfer interrupt (otherwise HAL would report a bogus "event"
 * at 32 bytes).
 */
HAL_StatusTypeDef proto_start_rx(UART_HandleTypeDef *huart);

/*
 * proto_rx_event_handler()
 * Call from HAL_UARTEx_RxEventCallback(huart, Size) for this UART.
 * Parses the received chunk (complete frames only), then re-arms DMA.
 * Returns 1 if `out` now holds a valid decoded command, 0 otherwise.
 */
uint8_t proto_rx_event_handler(UART_HandleTypeDef *huart, uint16_t size,
		cmd_data_t *out);

/*
 * proto_rx_error_handler()
 * Call from HAL_UART_ErrorCallback() for this UART. Restarts reception
 * (after an overrun, noise or framing error HAL stops reception and would
 * otherwise never resume).
 */
void proto_rx_error_handler(UART_HandleTypeDef *huart);

/*
 * proto_rx_keepalive()
 * Safety net, call on EVERY pass of the main loop. If reception is not
 * running and nothing is mid-abort, restarts it -- e.g. when a re-arm in
 * the RX interrupt returned HAL_BUSY because a telemetry transmit held
 * the UART handle's lock at that instant. Cheap, and does nothing while
 * reception is running normally.
 */
void proto_rx_keepalive(UART_HandleTypeDef *huart);

/* Packs telemetry into a ready-to-send 18-byte frame. */
void proto_build_telem_frame(const telem_data_t *data,
		uint8_t txBuf[TELEM_FRAME_SIZE]);

/*
 * proto_send_telem()
 * Builds the frame into an internal static buffer (safe for DMA) and
 * starts a DMA transmit. Returns HAL_BUSY without sending if the previous
 * frame is still going out.
 */
HAL_StatusTypeDef proto_send_telem(UART_HandleTypeDef *huart,
		const telem_data_t *data);

#endif /* UART_DMA_H */
