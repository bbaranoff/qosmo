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
#include <string.h>
#include "hw/arm/calypso/calypso_api.h"
#include "hw/arm/calypso/calypso_l1_ops.h"
#include "hw/arm/calypso/calypso_dcch_tap.h"

static const CalypsoL1Ops *l1;

/* ── The loaded firmware ─────────────────────────────────────────────────
 *
 * calypso_mb.c passes -kernel here; it is kept for the users that need a
 * firmware SYMBOL rather than a hard-coded address. The gr-gsm layer 1 has
 * its own lookup (elf_symbol/arm_read32) but is disabled with an external
 * DSP, and calypso_trx.c needs it for "last_rach". */
static const char *g_firmware_elf;

const char *calypso_firmware_elf(void)
{
    return g_firmware_elf;
}

/* Address of a symbol of the loaded ELF, 0 if not found. ELF32 little-endian
 * symbol table, as in l1-grgsm/calypso_l1_grgsm.c:elf_symbol(). */
uint32_t calypso_firmware_symbol(const char *want)
{
    if (!g_firmware_elf || !want) {
        return 0;
    }
    FILE *f = fopen(g_firmware_elf, "rb");
    if (!f) {
        return 0;
    }
    fseek(f, 0, SEEK_END);
    long sz = ftell(f);
    fseek(f, 0, SEEK_SET);
    if (sz < 52 || sz > (64L << 20)) {
        fclose(f);
        return 0;
    }
    uint8_t *b = g_malloc((size_t)sz);
    size_t got = fread(b, 1, (size_t)sz, f);
    fclose(f);
    uint32_t ret = 0;
    if (got == (size_t)sz && b[0] == 0x7f && b[1] == 'E' && b[2] == 'L' &&
        b[3] == 'F' && b[4] == 1) {
#define R16(o) ((uint32_t)b[o] | ((uint32_t)b[(o) + 1] << 8))
#define R32(o) (R16(o) | (R16((o) + 2) << 16))
        uint32_t shoff = R32(0x20), shent = R16(0x2e), shnum = R16(0x30);
        for (uint32_t si = 0; si < shnum; si++) {
            uint32_t sh = shoff + si * shent;
            if ((long)(sh + 40) > sz) {
                break;
            }
            if (R32(sh + 4) != 2) {          /* SHT_SYMTAB */
                continue;
            }
            uint32_t symoff = R32(sh + 0x10), symsz = R32(sh + 0x14);
            uint32_t link = R32(sh + 0x18), entsz = R32(sh + 0x24);
            uint32_t strsh = shoff + link * shent;
            if ((long)(strsh + 40) > sz || entsz < 16) {
                break;
            }
            uint32_t stroff = R32(strsh + 0x10), strsz = R32(strsh + 0x14);
            for (uint32_t o = 0; o + 16 <= symsz && (long)(symoff + o + 16) <= sz;
                 o += entsz) {
                uint32_t ni = R32(symoff + o);
                if (ni < strsz && !strcmp((const char *)(b + stroff + ni), want)) {
                    ret = R32(symoff + o + 4);
                    break;
                }
            }
            break;
        }
#undef R16
#undef R32
    }
    g_free(b);
    return ret;
}

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
    g_firmware_elf = firmware_elf;   /* before any return: see calypso_firmware_symbol() */
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
    /* Do NOT release the dedicated channel on d_dsp_page == 0: the firmware
     * does an l1s_dsp_abort() when switching TO the dedicated channel. The
     * release is inferred from the return to a common channel, in the tap. */
}

void calypso_l1_do_uart_tx_byte(uint8_t ch)
{
    if (l1 && l1->uart_tx_byte) {
        l1->uart_tx_byte(ch);
    } else {
        /* No layer 1 registered (external DSP): the bridge tap still publishes
         * the dedicated channel, otherwise pont.py drops the uplink. */
        calypso_dcch_tap_tx_byte(ch);
    }
}

void calypso_l1_do_api_write_observed(uint32_t off, uint16_t val, unsigned size)
{
    if (l1 && l1->api_write_observed) {
        l1->api_write_observed(off, val, size);
    }
}
