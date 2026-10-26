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
 *    - Frame constants: PROTO_START_BYTE, CMD_FRAME_SIZE (11),
 *      TELEM_FRAME_SIZE (20), PROTO_REF_START_BYTE, REF_FRAME_SIZE (10),
 *      PROTO_RX_DMA_SIZE (64)
 *    - cmd_data_t   : decoded command frame (Pi -> STM32)
 *    - ref_data_t   : decoded reference frame (Pi -> STM32, Bar30 settings)
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
 *    - The Pi sends one 11-byte command frame every 20 ms (50 Hz). A frame
 *      takes ~0.95 ms on the wire at 115200 bit/s, so there is ~19 ms of
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
 *  Pi -> STM32 (command), 11 bytes:
 *    [ START ][ motor1..motor6 ][ cam_tilt_dc ][ recover ][ CHECKSUM ]
 *       1 B        6x 1 B         2 B (i16)      1 B (u8)     1 B
 *
 *  Pi -> STM32 (reference), 10 bytes -- Bar30 depth-calculation settings
 *  from the command centre, sent only when they change (3 times each) and
 *  whenever the STM32 (re)connects:
 *    [ START_REF ][ surface_hpa_x100 ][ density_x10 ][ flags ][ ref_id ][ CHECKSUM ]
 *        1 B          4 B (u32)          2 B (u16)     1 B      1 B        1 B
 *
 *  START_REF:   0xAB (commands use 0xAA). Sent on its own, ~10 ms after a
 *               command frame, so it normally arrives as its own chunk.
 *  surface_hpa_x100: surface pressure reference, hPa (= mbar) x100,
 *               e.g. 101325 = 1013.25 mbar -> MS5837_SetSurfaceReference().
 *  density_x10: water density, kg/m^3 x10, e.g. 10000 = 1000.0 kg/m^3
 *               -> MS5837_SetFluidDensity().
 *  flags:       bit0 = second-order pressure calibration on (water below
 *               20 degC) -> bar30.secondOrderCalculation.
 *  ref_id:      +1 (wrapping) each time the values change; the 3 repeats of
 *               one set share the same ref_id.
 *  Frames whose values are out of range (surface 800.00..1100.00 mbar,
 *  density 900.0..1100.0 kg/m^3) are rejected even if the checksum is
 *  good, so a stray 0xAB inside a corrupted chunk can never change them.
 *  Until a reference frame arrives, main.c uses its SURFACE_PRESSURE,
 *  WATER_DENSITY and second_order_calculation #defines (boot defaults).
 *
 *  STM32 -> Pi (telemetry), 20 bytes:
 *    [ START ][ pressure_x10 ][ depth_x100 ][ water_temp_x100 ][ battery_x100 ]
 *       1 B      2 B (u16)       2 B (i16)       2 B (i16)          2 B (u16)
 *    [ inside_temp_x100 ]
 *       2 B (i16)
 *    [ heading_x100 ][ roll_x100 ][ pitch_x100 ][ leak ][ fault_flags ][ CHECKSUM ]
 *       2 B (u16)      2 B (i16)    2 B (i16)    1 B (u8)   1 B (u8)       1 B
 *
 *  cam_tilt_dc: camera tilt x10 (deci-degrees), 0 = level, + = up.
 *               No range clamping is done in this protocol layer -- the
 *               motor and tilt control functions clamp their own inputs.
 *  recover:     0 = normal, 1 = clear a leak shutdown. Acted on only when it
 *               changes 0 -> 1, and only if the leak sensor is dry; otherwise
 *               ignored. The Pi should send 0 the rest of the time.
 *  pressure_x10: absolute Bar30 pressure in kPa x10 (0.1 kPa steps, which
 *               is numerically the same as hPa / mbar). Raw sensor reading,
 *               NOT minus the surface reference, so ~1013 at the surface.
 *               0.1 kPa = 100 Pa ~= 1 cm of water.
 *  leak:        latched leak state, 0 = no leak, 1 = leak detected. Stays 1
 *               (even after the sensor dries) until a recover command
 *               succeeds, which also needs the sensor to be dry.
 *  fault_flags: SystemState_t bit flags active RIGHT NOW (SYS_STATE_FAULT_LEAK,
 *               _IMU, _DEPTH, _TEMP, _UART), OR'ed together into one byte.
 *               A bit clears when its fault recovers (except LEAK, which
 *               stays set until a recover command). While a sensor's bit is
 *               set, its telemetry field holds a stale value; ignore it.
 *               Stored as uint8_t, since a combination of flags (e.g.
 *               LEAK | TEMP = 0x09) is not itself one of the enum's values.
 *
 *  All multi-byte values little-endian. Telemetry measurements are scaled
 *  x100 (2 decimal places, e.g. 12.34 m -> 1234), EXCEPT pressure, which is
 *  kPa x10 (e.g. 101.3 kPa -> 1013). Camera tilt is scaled x10
 *  (deci-degrees); motors are plain percent (-100..100).
 *
 *  Telemetry ranges that fit the 16-bit fields (values outside are clamped
 *  to the limit by the sender, see to_i16_x100()/to_u16_x100()/to_u16_x10()):
 *    i16 fields x100: -327.68 .. 327.67   (depth m, temps degC, roll/pitch deg)
 *    u16 fields x100:    0.00 .. 655.35   (battery V, heading deg)
 *    u16 field  x10 :     0.0 .. 6553.5   (pressure kPa; Bar30 max ~3000 kPa)
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

#ifndef UART_DMA_UART_DMA_H_
#define UART_DMA_UART_DMA_H_

#include <stdint.h>
#include "stm32f4xx_hal.h"   /* for UART_HandleTypeDef, HAL_StatusTypeDef */

#define PROTO_START_BYTE   0xAA

#define CMD_FRAME_SIZE     11   /* START + 6 motors + tilt(2) + recover + checksum */
#define TELEM_FRAME_SIZE   20   /* START + 8x 2-byte fields + leak + faults + checksum */

#define PROTO_REF_START_BYTE 0xAB /* reference frame start (commands use 0xAA) */
#define REF_FRAME_SIZE     10   /* START + surface(4) + density(2) + flags + ref_id + checksum */
#define REF_FLAG_SECOND_ORDER 0x01U /* flags bit0: second-order pressure calibration on */

/* Accepted ranges for reference values (anything else is rejected). */
#define REF_SURFACE_MIN_X100  80000UL  /*  800.00 mbar */
#define REF_SURFACE_MAX_X100 110000UL  /* 1100.00 mbar */
#define REF_DENSITY_MIN_X10    9000U   /*  900.0 kg/m^3 */
#define REF_DENSITY_MAX_X10   11000U   /* 1100.0 kg/m^3 */

#define PROTO_RX_DMA_SIZE  64   /* DMA chunk buffer; > several frames */

/* proto_rx_event_handler() return bits: what the chunk contained */
#define PROTO_RX_CMD  0x01U     /* `out` holds a new valid command */
#define PROTO_RX_REF  0x02U     /* `ref_out` holds a new valid reference frame */

typedef struct {
	int8_t motor[6]; /* -100..100 (%), one per thruster */
	int16_t tilt_dc; /* camera tilt x10, deci-degrees */
	uint8_t recover; /* 0 = normal, 1 = clear leak shutdown (on 0 -> 1 change) */
} cmd_data_t;

typedef struct {
	uint32_t surface_hpa_x100; /* surface pressure reference, hPa (= mbar) x100 */
	uint16_t density_x10; /* water density, kg/m^3 x10 */
	uint8_t flags; /* bit0 = second-order calibration on (REF_FLAG_SECOND_ORDER) */
	uint8_t ref_id; /* +1 each time the values change */
} ref_data_t;

typedef struct {
	uint16_t pressure_x10; /* absolute pressure, kPa x10 (= hPa), ~1013 at surface */
	int16_t depth_x100; /* depth x100, centimetres, 0-30000 (0-300.00 m) */
	int16_t water_temp_x100; /* water temperature x100, 0.01 degC */
	uint16_t battery_x100; /* battery voltage x100, 0.01 V */
	int16_t inside_temp_x100; /* enclosure temperature x100, 0.01 degC */
	uint16_t heading_x100; /* BNO055 heading x100, 0-36000 (0.01 deg) */
	int16_t roll_x100; /* BNO055 roll x100, ~-9000..9000 */
	int16_t pitch_x100; /* BNO055 pitch x100, ~-18000..18000 */
	uint8_t leak; /* 0 = no leak, 1 = leak latched (until recover) */
	uint8_t fault_flags; /* live SystemState_t flags, OR'ed (LEAK stays set) */
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
 * Returns a bit mask: PROTO_RX_CMD if `out` now holds a valid decoded
 * command, PROTO_RX_REF if `ref_out` now holds a valid reference frame,
 * 0 if the chunk held neither.
 */
uint8_t proto_rx_event_handler(UART_HandleTypeDef *huart, uint16_t size,
		cmd_data_t *out, ref_data_t *ref_out);

/*
 * proto_rx_error_handler()
 * Call from HAL_UART_ErrorCallback() for this UART. Restarts reception
 * (after an overrun, noise or framing error HAL stops reception and would
 * otherwise never resume).
 */
void proto_rx_error_handler(UART_HandleTypeDef *huart);

/*
 * proto_get_cmd_checksum_errors()
 * Number of command frames lost to a checksum failure since boot: a START
 * byte followed by a full CMD_FRAME_SIZE bytes whose checksum didn't match.
 * Not counted until the first valid frame arrives (ignores start-up noise).
 * Frames that never arrive at all, or chunks dropped by a UART error, are
 * NOT counted -- this is a checksum-failure count only.
 */
uint32_t proto_get_cmd_checksum_errors(void);

/*
 * proto_rx_keepalive()
 * Safety net, call on EVERY pass of the main loop. If reception is not
 * running and nothing is mid-abort, restarts it -- e.g. when a re-arm in
 * the RX interrupt returned HAL_BUSY because a telemetry transmit held
 * the UART handle's lock at that instant. Cheap, and does nothing while
 * reception is running normally.
 */
void proto_rx_keepalive(UART_HandleTypeDef *huart);

/* Packs telemetry into a ready-to-send 20-byte frame. */
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

#endif /* UART_DMA_UART_DMA_H_ */