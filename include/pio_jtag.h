/*
 * Copyright (c) 2026 Dan Maio & Antigravity AI
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * RP2040 PIO Hardware Acceleration for JTAG (CMSIS-DAP)
 */

#ifndef _PIO_JTAG_H_
#define _PIO_JTAG_H_

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include "hardware/pio.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    PIO pio;
    uint sm;
    uint offset;
    bool enabled;
    uint32_t clock_hz;
    uint pin_tck;
    uint pin_tdi;
    uint pin_tdo;
    uint pin_tms;
    uint pin_trst;
    uint pin_reset;
} pio_jtag_t;

extern pio_jtag_t g_pio_jtag;

/**
 * Initialize the PIO state machine for JTAG pin operations.
 *
 * @param clock_hz Initial TCK clock frequency in Hz.
 * @return true on success, false if state machine could not be claimed.
 */
bool pio_jtag_init(uint32_t clock_hz);

/**
 * Deinitialize PIO JTAG, releasing the state machine and restoring GPIOs.
 */
void pio_jtag_deinit(void);

/**
 * Check if PIO JTAG acceleration is currently active.
 */
bool pio_jtag_is_enabled(void);

/**
 * Configure the JTAG TCK frequency.
 *
 * @param clock_hz Desired TCK frequency in Hz (e.g. 1000000 for 1 MHz).
 */
void pio_jtag_set_clock(uint32_t clock_hz);

/**
 * Execute a CMSIS-DAP DAP_JTAG_Sequence command using PIO hardware acceleration.
 *
 * @param info Sequence info byte (TCK count, TMS level, TDO capture flag).
 * @param tdi Input TDI data buffer (LSB first).
 * @param tdo Output TDO data buffer (LSB first, if capture requested).
 */
void pio_jtag_sequence(uint32_t info, const uint8_t *tdi, uint8_t *tdo);

/**
 * Transmit TDI bits and capture TDO bits at hardware speed.
 *
 * @param tdi Buffer containing bits to shift out on TDI (LSB first).
 * @param tdo Buffer to receive sampled bits from TDO (LSB first), or NULL to discard.
 * @param bit_len Number of clock pulses / bits to transfer.
 */
void pio_jtag_write_read(const uint8_t *tdi, uint8_t *tdo, size_t bit_len);

/**
 * Transmit TDI bits without capturing TDO.
 *
 * @param tdi Buffer containing bits to shift out on TDI (LSB first).
 * @param bit_len Number of clock pulses / bits to transfer.
 */
void pio_jtag_write(const uint8_t *tdi, size_t bit_len);

/**
 * Clock TCK for a specified number of cycles with TDI idle high.
 *
 * @param bit_len Number of TCK cycles.
 */
void pio_jtag_clock(size_t bit_len);

/**
 * Perform a single-bit JTAG cycle with TDI output and TDO sampling.
 *
 * @param tdi Bit value to output on TDI (0 or 1).
 * @return Sampled bit value from TDO (0 or 1).
 */
uint8_t pio_jtag_cycle_tdio(uint8_t tdi);

#ifdef __cplusplus
}
#endif

#endif /* _PIO_JTAG_H_ */
