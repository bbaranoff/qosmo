/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Platform -> layer 1 dispatch.
 *
 * Exists so that calypso_trx.c, calypso_mb.c and calypso_uart.c never name
 * the layer 1 implementation they are linked against. See calypso_l1_ops.h
 * for the reasoning behind the split.
 *
 * THE DEFAULTS ARE NOT ERRORS. With no layer 1 registered, qosmo alone is a
 * Calypso whose modem receives nothing: the osmocom-bb firmware boots, scans,
 * finds no cell, and that is the expected behaviour. It is what lets the
 * platform - boot, IRQ, TDMA, UART, SIM - be tested without dragging in
 * gr-gsm or the C54x.
 */
#include "qemu/osdep.h"
#include <stdlib.h>
#include "hw/arm/calypso/calypso_api.h"
#include "hw/arm/calypso/calypso_l1_ops.h"

static const CalypsoL1Ops *l1;

void calypso_l1_register(const CalypsoL1Ops *ops)
{
    if (l1 && ops) {
        /* Two layer 1 implementations linked into the same binary: the
         * fork's meson.build picked up the other one's sources. Keep the last
         * registered so the result stays deterministic, but say so loudly. */
        fprintf(stderr, "calypso: DEUX couches 1 enregistrees (%s puis %s) - "
                        "verifier hw/arm/calypso/meson.build\n",
                l1->name ? l1->name : "?", ops->name ? ops->name : "?");
    }
    l1 = ops;
}

/* External DSP (CALYPSO_DSP_EXTERN): the C54x in c54x_exe holds layer 1, so
 * the gr-gsm shunt must neither initialise itself (its UDP listeners would
 * steal the bench's stream) nor substitute anything in the API RAM. Every
 * calypso_l1_do_* then becomes a no-op. */
void calypso_l1_disable(const char *why)
{
    if (l1) {
        fprintf(stderr, "calypso: couche 1 « %s » desactivee (%s)\n",
                l1->name ? l1->name : "?", why ? why : "");
    }
    l1 = NULL;
}

const char *calypso_l1_name(void)
{
    return (l1 && l1->name) ? l1->name : "aucune";
}

void calypso_l1_do_init(const char *firmware_elf)
{
    /* First of all: with an external DSP the shunt must not even open its
     * ports, hence the check here - calypso_mb.c calls this before
     * calypso_trx_init(). */
    const char *ext = getenv("CALYPSO_DSP_EXTERN");
    if (ext && *ext) {
        calypso_l1_disable("DSP externe, CALYPSO_DSP_EXTERN");
        return;
    }
    if (l1 && l1->init) {
        l1->init(firmware_elf);
    }
}

void calypso_l1_do_frame_tick(void)
{
    if (l1 && l1->frame_tick) {
        l1->frame_tick();
    }
}

bool calypso_l1_do_api_read_override(uint32_t off, uint16_t *out)
{
    /* false = the base serves the real API RAM contents. */
    return (l1 && l1->api_read_override) ? l1->api_read_override(off, out)
                                         : false;
}

bool calypso_l1_do_si_valid(void)
{
    return (l1 && l1->si_valid) ? l1->si_valid() : false;
}

uint32_t calypso_l1_do_l1s_fn(void)
{
    /* With no layer 1, the only frame number that exists is the platform's
     * TDMA clock. Returning 0 here breaks the burst ring continuity check in
     * api_write(). */
    return (l1 && l1->l1s_fn) ? l1->l1s_fn() : calypso_trx_get_fn();
}

void calypso_l1_do_burst_written(uint16_t d_burst_d)
{
    if (l1 && l1->burst_written) {
        l1->burst_written(d_burst_d);
    }
}

void calypso_l1_do_rach_written(uint16_t d_rach, uint32_t fn)
{
    if (l1 && l1->rach_written) {
        l1->rach_written(d_rach, fn);
    }
}

void calypso_l1_do_page_written(uint16_t d_dsp_page)
{
    if (l1 && l1->page_written) {
        l1->page_written(d_dsp_page);
    }
}

void calypso_l1_do_uart_tx_byte(uint8_t ch)
{
    if (l1 && l1->uart_tx_byte) {
        l1->uart_tx_byte(ch);
    }
}

void calypso_l1_do_api_write_observed(uint32_t off, uint16_t val, unsigned size)
{
    if (l1 && l1->api_write_observed) {
        l1->api_write_observed(off, val, size);
    }
}
