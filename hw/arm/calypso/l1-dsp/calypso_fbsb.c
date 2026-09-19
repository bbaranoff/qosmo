/*
 * calypso_fbsb.c - QEMU-side FBSB state tracking (logs only)
 *
 * No host-side synthesis: FB/SB detection is driven entirely by the DSP
 * writing the NDB cells, and the ARM reads them directly. This file only
 * follows d_task_md transitions, counts dispatches and dumps the cells.
 *
 * The dump reads each cell in BOTH views, because they can disagree:
 *
 *   data[] : DSP view - what the correlator writes
 *   api[]  : ARM view - what the firmware actually reads (prim_fbsb.c:306
 *                       read_fb_result() reads api_ram, not data[])
 *
 * A divergence is not a probe artefact, it is the diagnostic: "data[] full,
 * api[] empty" means the result is computed and never reaches the firmware.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */
#include "calypso_fbsb.h"
#include "calypso_full_pcb.h"   /* DARAM lock helpers */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

void calypso_fbsb_init(CalypsoFbsb *s, uint16_t *ndb_word_base,
                       uint16_t api_base, uint16_t *api_ram)
{
    if (!s) return;
    s->ndb       = ndb_word_base;
    s->api       = api_ram;
    s->api_base  = api_base;
    calypso_fbsb_reset(s);
}

void calypso_fbsb_reset(CalypsoFbsb *s)
{
    if (!s) return;
    /* Deliberately does NOT clear ndb / api / api_base: those are bindings,
     * not session state. */
    s->state       = FBSB_IDLE;
    s->fb0_attempt = 0;
    s->fb1_attempt = 0;
    s->sb_attempt  = 0;
    s->fn_started  = 0;
}

void calypso_fbsb_on_dsp_task_change(CalypsoFbsb *s, uint16_t d_task_md,
                                     uint64_t fn)
{
    /* Dedupe: task and state rarely change between frames, and printing one
     * line per call produced 2554 lines out of 20000, each with its own
     * fflush. Print transitions only, plus a periodic repeat count. */
    {
        static uint32_t l_key = 0xFFFFFFFFu; static unsigned long long rep = 0;
        uint32_t key = ((uint32_t)d_task_md << 8) ^ (uint32_t)(s ? s->state : 0xFF);
        if (key != l_key) {
            if (rep)
                fprintf(stderr, "[calypso-fbsb] on_dsp_task_change × %llu "
                        "(identique, non repete)\n", rep);
            l_key = key; rep = 0;
            fprintf(stderr, "[calypso-fbsb] on_dsp_task_change task=%u fn=%lu state=%d\n",
                    d_task_md, (unsigned long)fn, s ? (int)s->state : -1);
            fflush(stderr);
        } else if (++rep % 2000 == 0) {
            fprintf(stderr, "[calypso-fbsb] on_dsp_task_change × %llu "
                    "(task=%u state=%d, fn=%lu)\n",
                    rep, d_task_md, s ? (int)s->state : -1, (unsigned long)fn);
            fflush(stderr);
        }
    }
    if (!s) return;
    switch (d_task_md) {
    case DSP_TASK_FB:
        s->fb0_attempt++;   /* real count of FB task dispatches */
        s->state       = FBSB_FB0_SEARCH;
        s->fn_started  = fn;
        calypso_fbsb_dump(s, "FB0_SEARCH (real DSP path)");
        break;
    case DSP_TASK_SB:
        s->sb_attempt++;    /* real count of SB task dispatches */
        s->state      = FBSB_SB_SEARCH;
        s->fn_started = fn;
        calypso_fbsb_dump(s, "SB_SEARCH (real DSP path)");
        break;
    case DSP_TASK_ALLC: {
        static int log_once;
        if (!log_once++) {
            fprintf(stderr,
                    "[fbsb] ALLC task=24 fn=%lu — real DSP CCCH demod\n",
                    (unsigned long)fn);
            fflush(stderr);
        }
        break;
    }
    case DSP_TASK_NONE:
    default:
        break;
    }
}

/* Read one NDB cell in either view. `base` is s->api_base (0x0800) for both:
 * data[] is indexed from &data[0x0800] and api_ram from C54X_API_BASE, which
 * is the same value. Returns -1 when the view is absent (null pointer), so
 * that "not bound" stays distinguishable from "read as zero". */
static int fbsb_cell(const uint16_t *view, uint16_t base, uint16_t cell)
{
    return view ? (int)view[cell - base] : -1;
}

void calypso_fbsb_dump(const CalypsoFbsb *s, const char *tag)
{
    if (!s) return;
    static const char *names[] = {
        "IDLE", "FB0_SEARCH", "FB0_FOUND",
        "FB1_SEARCH", "FB1_FOUND",
        "SB_SEARCH",  "SB_FOUND",
        "DONE", "FAIL",
    };
    const uint16_t b = s->api_base;

    int d_det = fbsb_cell(s->ndb, b, NDB_D_FB_DET);
    int d_toa = fbsb_cell(s->ndb, b, NDB_A_SYNC_DEMOD_TOA);
    int d_pm  = fbsb_cell(s->ndb, b, NDB_A_SYNC_DEMOD_PM);
    int d_ang = fbsb_cell(s->ndb, b, NDB_A_SYNC_DEMOD_ANG);
    int d_snr = fbsb_cell(s->ndb, b, NDB_A_SYNC_DEMOD_SNR);

    int a_det = fbsb_cell(s->api, b, NDB_D_FB_DET);
    int a_toa = fbsb_cell(s->api, b, NDB_A_SYNC_DEMOD_TOA);
    int a_pm  = fbsb_cell(s->api, b, NDB_A_SYNC_DEMOD_PM);
    int a_ang = fbsb_cell(s->api, b, NDB_A_SYNC_DEMOD_ANG);
    int a_snr = fbsb_cell(s->api, b, NDB_A_SYNC_DEMOD_SNR);

    /* TOA and ANGLE are signed on the firmware side. */
    fprintf(stderr,
            "[fbsb] %s state=%s fb0_att=%u fb1_att=%u sb_att=%u "
            "data[](det=%d toa=%d pm=%d ang=%d snr=0x%04x) "
            "api[](det=%d toa=%d pm=%d ang=%d snr=0x%04x)%s\n",
            tag ? tag : "", names[s->state],
            s->fb0_attempt, s->fb1_attempt, s->sb_attempt,
            d_det, (int)(int16_t)d_toa, d_pm, (int)(int16_t)d_ang, d_snr & 0xFFFF,
            a_det, (int)(int16_t)a_toa, a_pm, (int)(int16_t)a_ang, a_snr & 0xFFFF,
            (d_det > 0 && a_det <= 0) ? "  <<<< DIVERGENCE data/api" : "");
    fflush(stderr);
}
