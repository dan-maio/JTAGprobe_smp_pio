/*
 * Copyright (c) 2026 Dan Maio & Antigravity AI
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * RP2040 PIO Hardware Acceleration for JTAG (CMSIS-DAP)
 */

#include "pio_jtag.h"
#include "DAP_config.h"
#include "hardware/clocks.h"
#include "hardware/gpio.h"
#include "jtag.pio.h"

pio_jtag_t g_pio_jtag = {
    .pio = NULL,
    .sm = 0,
    .offset = 0,
    .enabled = false,
    .clock_hz = DAP_DEFAULT_SWJ_CLOCK,
    .pin_tck = DAP_SWJ_SWCLK_TCK,
    .pin_tdi = DAP_SWJ_TDI,
    .pin_tdo = DAP_SWJ_TDO,
    .pin_tms = DAP_SWJ_SWDIO_TMS,
    .pin_trst = DAP_SWJ_nTRST,
    .pin_reset = DAP_SWJ_nRESET
};

bool pio_jtag_init(uint32_t clock_hz) {
    if (g_pio_jtag.enabled) {
        pio_jtag_set_clock(clock_hz);
        return true;
    }

    PIO pio = pio0;
    int sm = pio_claim_unused_sm(pio, false);
    if (sm < 0) {
        pio = pio1;
        sm = pio_claim_unused_sm(pio, false);
        if (sm < 0) {
            return false;
        }
    }

    if (!pio_can_add_program(pio, &jtag_tdo_program)) {
        pio_sm_unclaim(pio, (uint)sm);
        return false;
    }

    uint offset = pio_add_program(pio, &jtag_tdo_program);

    pio_sm_config c = jtag_tdo_program_get_default_config(offset);
    sm_config_set_out_pins(&c, g_pio_jtag.pin_tdi, 1);
    sm_config_set_in_pins(&c, g_pio_jtag.pin_tdo);
    sm_config_set_sideset_pins(&c, g_pio_jtag.pin_tck);

    // LSB first for CMSIS-DAP, 8-bit autopush and autopull thresholds
    sm_config_set_out_shift(&c, true, true, 8);
    sm_config_set_in_shift(&c, true, true, 8);

    // Initial frequency divider (4 PIO instructions per JTAG TCK period)
    if (clock_hz == 0) {
        clock_hz = DAP_DEFAULT_SWJ_CLOCK;
    }
    float div = (float)clock_get_hz(clk_sys) / (4.0f * (float)clock_hz);
    if (div < 1.0f) div = 1.0f;
    if (div > 65535.0f) div = 65535.0f;
    sm_config_set_clkdiv(&c, div);

    // Initial pin levels: TCK high, TDI high
    pio_sm_set_pins_with_mask(pio, (uint)sm,
        (1u << g_pio_jtag.pin_tck) | (1u << g_pio_jtag.pin_tdi),
        (1u << g_pio_jtag.pin_tck) | (1u << g_pio_jtag.pin_tdi));

    // Direction mask: TCK and TDI outputs, TDO input
    pio_sm_set_pindirs_with_mask(pio, (uint)sm,
        (1u << g_pio_jtag.pin_tck) | (1u << g_pio_jtag.pin_tdi),
        (1u << g_pio_jtag.pin_tck) | (1u << g_pio_jtag.pin_tdi) | (1u << g_pio_jtag.pin_tdo));

    pio_gpio_init(pio, g_pio_jtag.pin_tdi);
    pio_gpio_init(pio, g_pio_jtag.pin_tck);

    // TDO input with pull-up
    gpio_init(g_pio_jtag.pin_tdo);
    gpio_set_dir(g_pio_jtag.pin_tdo, GPIO_IN);
    gpio_pull_up(g_pio_jtag.pin_tdo);

    // Synchronous JTAG bypass to reduce input delay
    hw_set_bits(&pio->input_sync_bypass, 1u << g_pio_jtag.pin_tdo);

    // SIO pins: TMS output high
    gpio_init(g_pio_jtag.pin_tms);
    gpio_set_dir(g_pio_jtag.pin_tms, GPIO_OUT);
    gpio_put(g_pio_jtag.pin_tms, 1);

    // nTRST output high
    gpio_init(g_pio_jtag.pin_trst);
    gpio_set_dir(g_pio_jtag.pin_trst, GPIO_OUT);
    gpio_put(g_pio_jtag.pin_trst, 1);

    // nRESET open-drain high (input with pull-up)
    gpio_init(g_pio_jtag.pin_reset);
    gpio_set_dir(g_pio_jtag.pin_reset, GPIO_IN);
    gpio_pull_up(g_pio_jtag.pin_reset);
    gpio_put(g_pio_jtag.pin_reset, 0);

    pio_sm_init(pio, (uint)sm, offset, &c);
    pio_sm_set_enabled(pio, (uint)sm, true);

    g_pio_jtag.pio = pio;
    g_pio_jtag.sm = (uint)sm;
    g_pio_jtag.offset = offset;
    g_pio_jtag.enabled = true;
    g_pio_jtag.clock_hz = clock_hz;

    return true;
}

void pio_jtag_deinit(void) {
    if (!g_pio_jtag.enabled) return;

    pio_sm_set_enabled(g_pio_jtag.pio, g_pio_jtag.sm, false);
    pio_remove_program(g_pio_jtag.pio, &jtag_tdo_program, g_pio_jtag.offset);
    pio_sm_unclaim(g_pio_jtag.pio, g_pio_jtag.sm);

    // Restore pins to SIO High-Z
    gpio_init(g_pio_jtag.pin_tck);
    gpio_set_dir(g_pio_jtag.pin_tck, GPIO_IN);
    gpio_disable_pulls(g_pio_jtag.pin_tck);

    gpio_init(g_pio_jtag.pin_tdi);
    gpio_set_dir(g_pio_jtag.pin_tdi, GPIO_IN);
    gpio_disable_pulls(g_pio_jtag.pin_tdi);

    gpio_init(g_pio_jtag.pin_tdo);
    gpio_set_dir(g_pio_jtag.pin_tdo, GPIO_IN);
    gpio_disable_pulls(g_pio_jtag.pin_tdo);

    gpio_init(g_pio_jtag.pin_tms);
    gpio_set_dir(g_pio_jtag.pin_tms, GPIO_IN);
    gpio_disable_pulls(g_pio_jtag.pin_tms);

    g_pio_jtag.enabled = false;
}

bool pio_jtag_is_enabled(void) {
    return g_pio_jtag.enabled;
}

void pio_jtag_set_clock(uint32_t clock_hz) {
    if (clock_hz == 0) {
        clock_hz = DAP_DEFAULT_SWJ_CLOCK;
    }
    g_pio_jtag.clock_hz = clock_hz;
    if (!g_pio_jtag.enabled) return;

    float div = (float)clock_get_hz(clk_sys) / (4.0f * (float)clock_hz);
    if (div < 1.0f) div = 1.0f;
    if (div > 65535.0f) div = 65535.0f;

    pio_sm_set_clkdiv(g_pio_jtag.pio, g_pio_jtag.sm, div);
}

void pio_jtag_write_read(const uint8_t *tdi, uint8_t *tdo, size_t bit_len) {
    if (bit_len == 0 || !g_pio_jtag.enabled) return;

    size_t byte_count = (bit_len + 7U) / 8U;
    size_t remainder = bit_len % 8U;
    size_t rx_count = (remainder != 0U) ? byte_count : (byte_count + 1U);

    size_t tx_remain = byte_count;
    size_t rx_remain = rx_count;

    // Send length - 1 into TX FIFO to kick off the PIO state machine
    pio_sm_put_blocking(g_pio_jtag.pio, g_pio_jtag.sm, (uint32_t)(bit_len - 1U));

    while (tx_remain > 0 || rx_remain > 0) {
        if (tx_remain > 0 && !pio_sm_is_tx_fifo_full(g_pio_jtag.pio, g_pio_jtag.sm)) {
            uint8_t byte_val = tdi ? *tdi++ : 0xFFU;
            g_pio_jtag.pio->txf[g_pio_jtag.sm] = (uint32_t)byte_val;
            tx_remain--;
        }

        if (rx_remain > 0 && !pio_sm_is_rx_fifo_empty(g_pio_jtag.pio, g_pio_jtag.sm)) {
            uint32_t val = g_pio_jtag.pio->rxf[g_pio_jtag.sm];
            if (tdo != NULL) {
                if (rx_remain > 1) {
                    *tdo++ = (uint8_t)val;
                } else if (remainder != 0U) {
                    *tdo++ = (uint8_t)(val >> (8U - remainder));
                }
                // If rx_remain == 1 and remainder == 0: dummy byte discarded
            }
            rx_remain--;
        }
    }
}

void pio_jtag_write(const uint8_t *tdi, size_t bit_len) {
    pio_jtag_write_read(tdi, NULL, bit_len);
}

void pio_jtag_clock(size_t bit_len) {
    pio_jtag_write_read(NULL, NULL, bit_len);
}

uint8_t pio_jtag_cycle_tdio(uint8_t tdi) {
    uint8_t tdi_byte = tdi & 1U;
    uint8_t tdo_byte = 0;
    pio_jtag_write_read(&tdi_byte, &tdo_byte, 1);
    return tdo_byte & 1U;
}

void pio_jtag_sequence(uint32_t info, const uint8_t *tdi, uint8_t *tdo) {
    uint32_t n = info & JTAG_SEQUENCE_TCK;
    if (n == 0U) {
        n = 64U;
    }

    // Set TMS level for this sequence block
    if (info & JTAG_SEQUENCE_TMS) {
        gpio_set_mask(1u << g_pio_jtag.pin_tms);
    } else {
        gpio_clr_mask(1u << g_pio_jtag.pin_tms);
    }

    bool capture = (info & JTAG_SEQUENCE_TDO) != 0U;
    if (capture) {
        pio_jtag_write_read(tdi, tdo, n);
    } else {
        pio_jtag_write(tdi, n);
    }
}
