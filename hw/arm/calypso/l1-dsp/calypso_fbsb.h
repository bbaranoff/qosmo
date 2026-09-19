/*
 * calypso_fbsb.h - QEMU-side FBSB (FCCH/SCH burst search) tracking
 *
 * Follows the FBSB sequence by watching d_task_md transitions and reading
 * back the NDB cells. Detection itself belongs to the DSP; nothing here
 * synthesises a result (see calypso_fbsb.c).
 *
 * The state machine being mirrored is the firmware's, in osmocom-bb
 * src/target/firmware/layer1/prim_fbsb.c:
 *
 *   IDLE
 *     │
 *     │  ARM writes d_task_md = FB_DSP_TASK (mode 0)
 *     ▼
 *   FB0_SEARCH ── correlator finds burst ──► FB0_FOUND
 *     │                                         │
 *     │ 12 attempts no FB                       │ ferr small enough
 *     │                                         ▼
 *     ▼                                       FB1_SEARCH ──► FB1_FOUND
 *   FAIL (result=255)                            │              │
 *                                                ▼              ▼
 *                                              FAIL          SB_SEARCH ──► SB_FOUND
 *                                                                │            │
 *                                                                ▼            ▼
 *                                                              FAIL        SUCCESS
 *
 * NDB cells we read/write (offsets in DSP data words from API base 0x0800).
 * Layout from osmocom-bb include/calypso/dsp_api.h:202-204 (d_fb_det,
 * d_fb_mode, then a_sync_demod[4]); the #defines below are authoritative:
 *   d_dsp_page         0x08D4    (page toggle from ARM)
 *   d_fb_det           0x08F8    (DSP -> ARM: non-zero = FB found)
 *   d_fb_mode          0x08F9    (ARM -> DSP: 0 = wideband search, 1 = narrow)
 *   a_sync_demod[0]    0x08FA    D_TOA   - time of arrival
 *   a_sync_demod[1]    0x08FB    D_PM    - power measurement
 *   a_sync_demod[2]    0x08FC    D_ANGLE - frequency phase angle
 *   a_sync_demod[3]    0x08FD    D_SNR   - signal-to-noise ratio
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */
#ifndef CALYPSO_FBSB_H
#define CALYPSO_FBSB_H

#include <stdint.h>
#include <stdbool.h>

/* NDB cell offsets - DSP data word addresses.
 *
 * Anchored on osmocom-bb dsp_api.h:18, BASE_API_NDB = 0xFFD001A8: NDB starts
 * at DSP word 0x0800 + 0x1A8/2 = 0x08D4, which is d_dsp_page (field 0 of
 * T_NDB_MCU_DSP); d_dsp_state is field 14, hence 0x08E2. Cross-checked at
 * runtime (the only ARM write to 0x08E2 is dsp.c:215, ndb->d_dsp_state = 3 =
 * C_DSP_IDLE3) and against the DSP ROM, which reads 0x08D4 as the page
 * selector and never references 0x08E2 in any of the 5 program ROMs.
 * An off-by-one-word here is read as a real address: keep both constants and
 * the comment above in agreement. */
#define NDB_D_DSP_PAGE       0x08D4
#define NDB_D_DSP_STATE      0x08E2
#define NDB_D_FB_DET         0x08F8
#define NDB_D_FB_MODE        0x08F9
#define NDB_A_SYNC_DEMOD_TOA 0x08FA
#define NDB_A_SYNC_DEMOD_PM  0x08FB
#define NDB_A_SYNC_DEMOD_ANG 0x08FC
#define NDB_A_SYNC_DEMOD_SNR 0x08FD

/* d_task_md values used by the firmware (subset).
 * From osmocom-bb dsp_api.h, verified against l1s_pm_cmd / l1s_fbdet_cmd. */
#define DSP_TASK_NONE       0
#define DSP_TASK_PM         1   /* PM_DSP_TASK (power measurement, NOT FB) */
#define DSP_TASK_FB         5   /* FB_DSP_TASK (frequency burst, idle) */
#define DSP_TASK_SB         6   /* SB_DSP_TASK (sync burst, idle)      */
#define DSP_TASK_TCH_FB     8   /* TCH_FB_DSP_TASK (dedicated) */
#define DSP_TASK_TCH_SB     9   /* TCH_SB_DSP_TASK (dedicated) */
#define DSP_TASK_ALLC      24   /* ALLC_DSP_TASK (CCCH read while FULL BCCH/CCCH) */

/* FBSB orchestration state. One instance per Calypso. */
typedef enum {
    FBSB_IDLE = 0,
    FBSB_FB0_SEARCH,
    FBSB_FB0_FOUND,
    FBSB_FB1_SEARCH,
    FBSB_FB1_FOUND,
    FBSB_SB_SEARCH,
    FBSB_SB_FOUND,
    FBSB_DONE,
    FBSB_FAIL,
} CalypsoFbsbState;

typedef struct CalypsoFbsb {
    CalypsoFbsbState state;
    uint16_t        *ndb;          /* points into ARM dsp_ram[] (word-addressed) */
    uint16_t         api_base;     /* DSP-side word base (0x0800) */

    /* Per-attempt counters mirroring prim_fbsb.c, incremented for real in
     * calypso_fbsb_on_dsp_task_change(). */
    uint8_t          fb0_attempt;
    uint8_t          fb1_attempt;
    uint8_t          sb_attempt;

    /* ARM view of the same cells: `api` points at api_ram, i.e. what the
     * firmware actually reads (prim_fbsb.c read_fb_result), while `ndb` points
     * at the DSP-side data[]. The two views can diverge, and that divergence
     * is the diagnostic. */
    uint16_t        *api;

    /* Bookkeeping. */
    uint64_t         fn_started;
} CalypsoFbsb;

/* Lifecycle. */
void calypso_fbsb_init(CalypsoFbsb *s, uint16_t *ndb_word_base,
                       uint16_t api_base, uint16_t *api_ram);
void calypso_fbsb_reset(CalypsoFbsb *s);

/* Hooks. */
void calypso_fbsb_on_dsp_task_change(CalypsoFbsb *s, uint16_t d_task_md,
                                     uint64_t fn);

/* Trace helper: single-line dump of current state. */
void calypso_fbsb_dump(const CalypsoFbsb *s, const char *tag);

#endif /* CALYPSO_FBSB_H */
