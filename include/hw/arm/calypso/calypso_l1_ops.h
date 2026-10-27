/* SPDX-License-Identifier: GPL-2.0-or-later */
#ifndef HW_ARM_CALYPSO_L1_OPS_H
#define HW_ARM_CALYPSO_L1_OPS_H

#include <stdint.h>
#include <stdbool.h>

/* ═════════════════════════════════════════════════════════════════════════
 * THE SEAM BETWEEN THE PLATFORM AND LAYER 1
 * ═════════════════════════════════════════════════════════════════════════
 *
 * The platform (qosmo) models the Calypso and its board: ARM, SoC, TPU, TSP,
 * SIM, ULPD, UART, timers, SPI/I²C, INTH, TRF6151, IOTA, RF3166, ASM4532,
 * XIO, plus the TDMA clock and the API RAM window. It carries NO layer 1 and
 * builds and boots on its own: the osmocom-bb firmware starts and never sees
 * anything on the air. A layer 1 is a separate implementation plugged into
 * this vtable:
 *
 *     l1-grgsm      calypso_l1_grgsm.c   (demodulation by gr-gsm, on the host)
 *     l1-dsp        calypso_c54x.c ...   (real TI mask ROM on an emulated C54x)
 *
 * The layer 1 is otherwise reached only through direct calls into the
 * symbols the platform exports; `api_read`/`api_write` on the 0xFFD00000
 * window are the two places where layer 1 behaviour hides behind a memory
 * access, hence the hooks below.
 *
 * WHAT GOES THROUGH HERE, AND WHAT DOES NOT
 *
 * This vtable only carries the platform -> L1 direction: the 9 places where
 * the hardware has to tell layer 1 that something happened. The opposite
 * direction (the L1 reading the API RAM or the current frame, resynchronising)
 * stays as direct calls into the symbols the platform exports
 * (`calypso_api_ram()`, `calypso_trx_get_fn()`, `calypso_trx_autosync_fn()`,
 * `calypso_tsp_move()`, `calypso_tpu_run_scenario()`). This is deliberate:
 * those symbols are hardware, they have a single implementation by
 * construction, and indirecting them would buy nothing.
 *
 * The `dcch_set`, `dcch_active`, `dcch_is_tch` and `rach_conf` entries are
 * NOT in the vtable either: their only callers are in calypso_l1ctl_tap.c,
 * i.e. inside the grgsm L1. They are internal to that layer 1, not part of
 * the seam; exposing them here would force the l1-dsp layer to implement
 * functions nothing on its execution path calls.
 *
 * IF NO L1 IS REGISTERED
 *
 * Everything is silently inert (see calypso_l1_dispatch.c) and
 * `api_read_override` returns false, so the API RAM behaves as bare RAM.
 * This is the nominal state of the platform alone, not an error: it is what
 * makes `qosmo` testable without embedding either layer 1.
 * ═════════════════════════════════════════════════════════════════════════ */

typedef struct CalypsoL1Ops {
    /* Short name of the layer 1, for traces: "grgsm", "c54x", ... */
    const char *name;

    /* End of calypso_machine_init(). `firmware_elf` is the -kernel of the
     * command line (osmocom-bb's layer1.highram.elf), NULL if absent. */
    void (*init)(const char *firmware_elf);

    /* On every lowering of the TPU frame IRQ, i.e. once per TDMA frame, from
     * frame_irq_lower(). This is the heartbeat of the L1. */
    void (*frame_tick)(void);

    /* 16-bit read in the API RAM window, BEFORE the value goes to the ARM.
     * Return true and fill `*out` to impose a value; returning false lets
     * the real RAM content through.
     * `off` is the offset in the window, not an absolute address. */
    bool (*api_read_override)(uint32_t off, uint16_t *out);

    /* Does the L1 have a block to serve right now? Consulted when the
     * firmware reads d_task_d and finds 0: if true, the platform answers
     * ALLC_DSP_TASK instead of "nothing to do". */
    bool (*si_valid)(void);

    /* Frame number as seen by the L1 (l1s). Used by the platform to detect
     * a continuity break in the burst ring. */
    uint32_t (*l1s_fn)(void);

    /* The firmware just wrote d_burst_d / d_rach / d_dsp_page. These are the
     * three offsets the gr-gsm L1 needs. */
    void (*burst_written)(uint16_t d_burst_d);
    void (*rach_written)(uint16_t d_rach, uint32_t fn);
    void (*page_written)(uint16_t d_dsp_page);

    /* EVERY 16-bit write in the API RAM window, after the platform has stored
     * it. The C54x L1 decodes the whole window (its calypso_dsp_write()),
     * whereas gr-gsm only cares about three named offsets.
     *
     * The three hooks above ARE redundant with this one and stay anyway: an
     * L1 that only wants d_burst_d does not have to filter the whole window
     * to get it. An L1 may fill either or both; the order is
     * api_write_observed() FIRST, then the named hook if there is one. */
    void (*api_write_observed)(uint32_t off, uint16_t val, unsigned size);

    /* A byte went out on the modem UART (sercomm). Optional: this is the hook
     * point of the grgsm L1CTL tap. NULL if the layer 1 does not want it, in
     * which case the UART behaves as a UART. */
    void (*uart_tx_byte)(uint8_t ch);
} CalypsoL1Ops;

/* To be called by the layer 1 from a constructor or its own type_init, hence
 * BEFORE calypso_machine_init(). A second registration overwrites the first
 * and reports it on stderr: two layer 1s in the same binary is a meson.build
 * error, not an operating mode. */
void calypso_l1_register(const CalypsoL1Ops *ops);

/* Name of the active L1, or "aucune" (none). For banners and traces. */
const char *calypso_l1_name(void);

/* The accessors the platform uses. They wrap the "is an L1 registered?" test
 * so that the call sites stay readable. */
void calypso_l1_do_init(const char *firmware_elf);
void calypso_l1_do_frame_tick(void);
bool calypso_l1_do_api_read_override(uint32_t off, uint16_t *out);
bool calypso_l1_do_si_valid(void);
uint32_t calypso_l1_do_l1s_fn(void);
void calypso_l1_do_burst_written(uint16_t d_burst_d);
void calypso_l1_do_rach_written(uint16_t d_rach, uint32_t fn);
void calypso_l1_do_page_written(uint16_t d_dsp_page);
void calypso_l1_do_api_write_observed(uint32_t off, uint16_t val, unsigned size);
void calypso_l1_do_uart_tx_byte(uint8_t ch);

void calypso_l1_disable(const char *why);

#endif
