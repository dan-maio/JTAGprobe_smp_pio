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
    bool pins_pio;      // TCK and TDI driven by the PIO (else by SIO)
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
 * Hand TCK, TDI and TMS from the PIO to SIO, keeping their levels, for the
 * CPU bit-banging paths (JTAG_IR, JTAG_Transfer, JTAG_ReadIDCode,
 * JTAG_WriteAbort, DAP_SWJ_Sequence, DAP_SWJ_Pins), through
 * PIN_TCK_TDI_TO_SIO() in DAP_config.h.
 */
void pio_jtag_pins_to_sio(void);

/**
 * Hand TCK, TDI and TMS back to the PIO, before a sequence is clocked.
 */
void pio_jtag_pins_to_pio(void);

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
 * Run the entries of one DAP_JTAG_Sequence command through the PIO, streamed
 * without waiting between entries (the PIO sets TMS from each entry header).
 *
 * @param count Number of entries.
 * @param request The entries: info byte, then the TDI bytes (LSB first).
 * @param response Where the captured TDO bytes go (entries with the TDO flag).
 * @return Request bytes consumed (upper 16 bits), response bytes written
 *         (lower 16 bits).
 */
uint32_t pio_jtag_sequences(uint32_t count, const uint8_t *request, uint8_t *response);

/**
 * Run one DAP_JTAG_Sequence entry through the PIO (JTAG_Sequence()).
 *
 * @param info Sequence info byte (TCK count, TMS level, TDO capture flag).
 * @param tdi Input TDI data buffer (LSB first).
 * @param tdo Output TDO data buffer (LSB first, if capture requested).
 */
void pio_jtag_sequence(uint32_t info, const uint8_t *tdi, uint8_t *tdo);

#ifdef __cplusplus
}
#endif

#endif /* _PIO_JTAG_H_ */
