/*
 * UART_DMA.c
 *
 *  Created on: September 23, 2026
 *      Author: Thomas Bourgeois
 *
 *  Serial protocol between the STM32 and the Raspberry Pi, using the HAL
 *  UART driver with DMA in both directions.
 *
 *  RX (Pi -> STM32): command frames, received with ReceiveToIdle DMA.
 *    Each IDLE or buffer-full event delivers one chunk, which is scanned
 *    for complete, checksum-valid frames. The last valid frame in the chunk
 *    wins. Partial frames are discarded; nothing is carried between chunks.
 *
 *    Command frame (CMD_FRAME_SIZE = 10 bytes):
 *      [0]      START          0xAA
 *      [1..6]   motor[0..5]    int8
 *      [7..8]   camera tilt    int16, little-endian
 *      [9]      checksum       8-bit sum of bytes [1..8]
 *
 *  TX (STM32 -> Pi): telemetry frames, sent with HAL_UART_Transmit_DMA.
 *
 *    Telemetry frame (TELEM_FRAME_SIZE = 18 bytes), 16-bit fields little-endian:
 *      [0]      START          0xAA
 *      [1..2]   depth          uint16, decimetres
 *      [3..4]   water temp     int16,  0.1 degC
 *      [5..6]   battery        uint16, 0.1 V
 *      [7..8]   inside temp    int16,  0.1 degC
 *      [9..10]  heading        uint16, 0.1 deg
 *      [11..12] roll           int16,  0.1 deg
 *      [13..14] pitch          int16,  0.1 deg
 *      [15]     leak           uint8
 *      [16]     fault flags    uint8
 *      [17]     checksum       8-bit sum of bytes [1..16]
 *
 *  Usage:
 *    - Call proto_start_rx() once after the UART is initialised.
 *    - Call proto_rx_event_handler() from HAL_UARTEx_RxEventCallback().
 *    - Call proto_rx_error_handler() from HAL_UART_ErrorCallback().
 *    - Call proto_rx_keepalive() periodically from the main loop to restart
 *      reception if it ever stops.
 *    - The UART4 global interrupt must be enabled (IDLE detection and
 *      TX complete both depend on it).
 */


#include "UART_DMA.h"

/* =========================================================================
 * Buffers
 *
 * dmaBuf : where the DMA writes each received chunk. It is parsed in place,
 *          then the DMA is re-armed. Nothing is kept between chunks.
 * txBuf  : telemetry frame being sent. static, because the DMA transmit is
 *          still reading it after proto_send_telem() returns.
 * ========================================================================= */
static uint8_t dmaBuf[PROTO_RX_DMA_SIZE];
static uint8_t txBuf[TELEM_FRAME_SIZE];

/* ========================================================================= */
uint8_t proto_checksum8(const uint8_t *data, uint16_t len) {
    uint8_t sum = 0;
    for (uint16_t i = 0; i < len; i++) {
        sum = (uint8_t)(sum + data[i]);
    }
    return sum;
}

/* =========================================================================
 * Unpack one command frame (START and checksum already validated).
 * frame layout: [0]=START [1..6]=motor[0..5]
 *               [7..8]=cam tilt (i16, little-endian) [9]=checksum
 * No range clamping here: the motor/tilt control functions do their own.
 * ========================================================================= */
static void proto_unpack_cmd_frame(const uint8_t *frame, cmd_data_t *out) {
    for (int i = 0; i < 6; i++) {
        /* Same 8 bits reinterpreted as signed: 0x9C -> -100 */
        out->motor[i] = (int8_t)frame[1 + i];
    }

    /* Camera tilt: rebuild the 16-bit pattern (low | high << 8), then
     * reinterpret it as signed. */
    out->tilt_dc = (int16_t)((uint16_t)frame[7] | ((uint16_t)frame[8] << 8));
}

/* =========================================================================
 * proto_frame_valid()
 *
 * Checks whether the CMD_FRAME_SIZE (10) bytes starting at `frame` form a
 * valid command frame. Returns 1 if valid, 0 if not.
 *
 * Two checks, both must pass (&&):
 *   1. frame[0] must be the START byte (0xAA).
 *      If not, the checksum is skipped entirely (&& short-circuits), which
 *      keeps scanning past junk bytes fast.
 *   2. The checksum is recomputed over the 8 body bytes between START and
 *      the checksum byte (6 motors + 2 tilt bytes):
 *        &frame[1]           -> start right after START
 *        CMD_FRAME_SIZE - 2  -> 10 - START - CHECKSUM = 8 bytes
 *      and compared with frame[CMD_FRAME_SIZE - 1] (frame[9]), the checksum
 *      the Pi sent. A byte corrupted or lost in transit makes them differ.
 *
 * Example: aa 32 00 9c 1e 00 aa 65 ff fa
 *          START ok; 0x32+0x00+0x9C+0x1E+0x00+0xAA+0x65+0xFF = 762,
 *          762 mod 256 = 250 = 0xFA = frame[9]  -> valid, returns 1
 *
 * A motor value of -86 is also 0xAA, so a false START can appear inside a
 * frame body -- check 2 rejects it, since the checksum won't match there.
 * ========================================================================= */
static uint8_t proto_frame_valid(const uint8_t *frame) {
    return (frame[0] == PROTO_START_BYTE) &&
           (proto_checksum8(&frame[1], CMD_FRAME_SIZE - 2) ==
            frame[CMD_FRAME_SIZE - 1]);
}

/* ========================================================================= */
HAL_StatusTypeDef proto_start_rx(UART_HandleTypeDef *huart) {
    HAL_StatusTypeDef st = HAL_UARTEx_ReceiveToIdle_DMA(huart, dmaBuf,
                                                        PROTO_RX_DMA_SIZE);
    if (st == HAL_OK) {
        /* HAL enables the half-transfer interrupt by default, which would
         * fire HAL_UARTEx_RxEventCallback at 32 bytes. We only want IDLE
         * and buffer-full events. */
        __HAL_DMA_DISABLE_IT(huart->hdmarx, DMA_IT_HT);
    }
    return st;
}

/* =========================================================================
 * Each chunk is handled on its own. Only COMPLETE, checksum-valid frames
 * inside this chunk are accepted; any fragment (a frame cut short, or the
 * tail of a frame caught mid-way at power-up) is ignored and thrown away
 * with the rest of the chunk. Nothing is carried over to the next chunk.
 * ========================================================================= */
uint8_t proto_rx_event_handler(UART_HandleTypeDef *huart, uint16_t size,
                               cmd_data_t *out) {
    uint8_t found = 0;

    if (size > PROTO_RX_DMA_SIZE) {
        size = PROTO_RX_DMA_SIZE;
    }

    /* Scan for complete frames. On a match, jump past the whole frame; on a
     * false START (e.g. motor value -86 = 0xAA) advance one byte, so a real
     * frame right after it in the same chunk is still found. */
    uint16_t i = 0;
    while ((uint16_t)(i + CMD_FRAME_SIZE) <= size) {
        if (proto_frame_valid(&dmaBuf[i])) {
            proto_unpack_cmd_frame(&dmaBuf[i], out);   /* last valid one wins */
            found = 1;
            i += CMD_FRAME_SIZE;
        } else {
            i++;
        }
    }

    /* Chunk fully processed (or discarded) -- re-arm for the next one.
     * HAL has already set RxState back to READY before calling the
     * callback, so restarting from here is allowed. */
    proto_start_rx(huart);

    return found;
}

/* ========================================================================= */
void proto_rx_error_handler(UART_HandleTypeDef *huart) {
    proto_start_rx(huart);   /* HAL has already aborted the old transfer */
}

/* ========================================================================= */
void proto_rx_keepalive(UART_HandleTypeDef *huart) {
    __disable_irq();
    uint8_t idle = (huart->RxState == HAL_UART_STATE_READY) &&
                   (huart->hdmarx->State == HAL_DMA_STATE_READY);
    __enable_irq();

    if (idle) {
        proto_start_rx(huart);
    }
}

/* =========================================================================
 * Telemetry packing: little-endian,
 * low byte = value & 0xFF, high byte = (value >> 8) & 0xFF.
 * ========================================================================= */
static void put_u16(uint8_t *dst, uint16_t v) {
    dst[0] = (uint8_t)(v & 0xFF);
    dst[1] = (uint8_t)((v >> 8) & 0xFF);
}

void proto_build_telem_frame(const telem_data_t *data,
                             uint8_t buf[TELEM_FRAME_SIZE]) {
    buf[0] = PROTO_START_BYTE;
    put_u16(&buf[1],  data->depth_dm);
    put_u16(&buf[3],  (uint16_t)data->water_temp_dc);   /* two's complement bits */
    put_u16(&buf[5],  data->battery_dv);
    put_u16(&buf[7],  (uint16_t)data->inside_temp_dc);
    put_u16(&buf[9],  data->heading_dc);
    put_u16(&buf[11], (uint16_t)data->roll_dc);
    put_u16(&buf[13], (uint16_t)data->pitch_dc);
    buf[15] = data->leak;
    buf[16] = data->fault_flags;
    buf[17] = proto_checksum8(&buf[1], TELEM_FRAME_SIZE - 2);
}

/* ========================================================================= */
HAL_StatusTypeDef proto_send_telem(UART_HandleTypeDef *huart,
                                   const telem_data_t *data) {
    /* gState returns to READY only after the UART4 interrupt reports
     * transmission complete -- another reason that IRQ must be enabled. */
    if (huart->gState != HAL_UART_STATE_READY) {
        return HAL_BUSY;
    }
    proto_build_telem_frame(data, txBuf);
    return HAL_UART_Transmit_DMA(huart, txBuf, TELEM_FRAME_SIZE);
}
