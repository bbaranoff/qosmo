/*
 * Calypso BSP/RIF DMA module — implementation.
 *
 * On real hardware the BSP (Baseband Serial Port) is a synchronous serial
 * link that DMA-feeds I/Q samples from the IOTA RF frontend into C54x
 * DSP DARAM. The DSP code (FB/SB/burst detection in PROM0) reads them
 * from a fixed DARAM buffer and posts results into the NDB.
 *
 * In QEMU, DL bursts arrive via UDP (TRXDv0 from calypso-ipc-device on port 5702).
 * This module owns that socket, decodes the TRXDv0 header, converts hard
 * bits to I/Q samples, and DMA-writes them into DSP DARAM.
 *
 * L1CTL control (DLCI 5) goes through the UART — bursts never touch UART.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */
#include "qemu/osdep.h"
#include <math.h>
#include "qemu/main-loop.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <limits.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>   /* inet_aton */
#include <unistd.h>
#include <errno.h>
#include <fcntl.h>
#include <poll.h>
#include "qemu/timer.h"
#include "calypso_bsp.h"
#include "calypso_rhea_dma.h"
#include "calypso_rif.h"
#include "calypso_c54x.h"
#include "hw/arm/calypso/calypso_iota.h"
#include "hw/arm/calypso/calypso_invariants.h"
#include "calypso_twl3025.h"
#include "hw/arm/calypso/calypso_trx.h"
#include "calypso_tint0.h"  /* GSM_HYPERFRAME */
#include "calypso_full_pcb.h"  /* DARAM lock helpers */
#include "hw/arm/calypso/calypso_trf6151.h"   /* RF calibration behind the a_pm model (calypso_bsp_rssi_apm) */

int calypso_rxfb_fired = 0;   /* set to 1 once RX-FBFLAGS sets data[0x3fad] bit15 */

/* Forward decls for env-gated helpers pre-warmed in calypso_bsp_init(). */
static uint32_t d_rach_word_offset(void);
static int rach_force_bsic(void);

#include "hw/arm/calypso/calypso_debug.h"
#include "calypso_gmsk.h"

/* calypso_api.h le declare, mais tire tout l'etat QEMU : prototype seul. */
extern uint32_t calypso_trx_get_fn(void);

/* DARAM write stamp, published for the c54x memory dump. */
unsigned calypso_daram_last_fn;
/* [2026-09-19] fn du dernier burst effectivement remis au BSP, pour mesurer
 * l'ecart entre le DEPOT et l'EVALUATION par la tache DSP (sonde DECALAGE). */
unsigned g_depot_fn, g_depot_seq;
unsigned calypso_daram_wr_count;
#define BSP_LOG(fmt, ...) \
    do { if (calypso_debug_enabled("BSP")) \
        fprintf(stderr, "[BSP] " fmt "\n", ##__VA_ARGS__); } while (0)

#define BSP_TRXD_PORT  6702   /* bridge forwards DL bursts here (5702 is bridge's own) */

/* ==========================================================================
 * DSP I/Q CHAIN
 *
 * Both mechanisms below derive from the REAL DL signal the BSP receives.
 *
 *  1. FB-STREAM — ring of decimated FCCH I/Q samples, served to the native
 *     correlator by the data[0x9213]/[0x9215] read intercept in calypso_c54x.c.
 *     This is the correlator INPUT in native mode (CALYPSO_FB_STREAM=1). It
 *     stays a crutch as long as the BSP -> DARAM chain does not feed the
 *     buffer on its own.
 *
 *  2. MAV / a_pm — mean magnitude of the DL burst, from which a_pm derives.
 *     On real Calypso the PM task does not compute a_pm from samples: the DSP
 *     zero-fills the result page and an integrator reads an ABB/RF power
 *     register. That register is not part of the modelled ADC, so we model it
 *     from the true DL magnitude. The MAV_REF -> RF_REF anchor is frontend
 *     calibration (like the trf6151 gain), not a decreed constant: two
 *     different signals give two different a_pm.
 * ========================================================================== */
#define BSP_FBS_RING 16384              /* power of two */
static int16_t  bsp_fbs[BSP_FBS_RING];
static uint32_t bsp_fbs_wr, bsp_fbs_rd;
static uint16_t bsp_last_mav;           /* MAV(|I|+|Q|) of the last DL burst */

/* Feed the FB-STREAM ring and the magnitude measurement from a raw DL burst
 * (cs16, `n` interleaved I,Q int16). Called on burst reception. */
static void bsp_iq_publish(const int16_t *iq, int n)
{
    if (!iq || n <= 0) {
        return;
    }

    /* Burst MAV -> a_pm (header §2). No sqrt: mean of |I|+|Q|. */
    {
        uint64_t acc = 0;
        for (int i = 0; i < n; i++) {
            int v = iq[i];
            acc += (v < 0) ? (uint32_t)(-v) : (uint32_t)v;
        }
        uint32_t mav = (uint32_t)(acc / (uint32_t)n);
        /* [2026-09-23] UN BURST SILENCIEUX NE MESURE PAS LA CELLULE.
         * Les effacements (bsp_ts0_service, trame sans burst) et les
         * remplissages a zero des intervalles vides passent aussi par ici : le
         * MAV tombait a ~0 et le a_pm suivant au plancher (-100 dBm, que le
         * firmware rend « rxlev <=-110 (0) »), soit C1 = 0 et la cellule jugee
         * inutilisable -- « Found signal rxlev <=-110 (0) », « Channel sync
         * error », LOST_COVERAGE toutes les 5 a 7 s (run du 2026-09-23 15:36).
         * On garde donc la derniere mesure d'un vrai burst.
         * CALYPSO_BSP_MAV_MIN=0 retablit la mise a jour inconditionnelle. */
        static int mav_min = -1;
        if (mav_min < 0) {
            const char *e = calypso_getenv("CALYPSO_BSP_MAV_MIN");
            mav_min = e ? atoi(e) : 64;
        }
        if (mav >= (uint32_t)mav_min) {
            bsp_last_mav = (mav > 0xffff) ? 0xffff : (uint16_t)mav;
        }
    }

    /* FB-STREAM ring (header §1). */
    {
        static int on = -1, decim = 4;
        if (on < 0) {
            const char *e = calypso_getenv("CALYPSO_FB_STREAM");
            on = (e && atoi(e) > 0) ? 1 : 0;
            const char *d = calypso_getenv("CALYPSO_FB_STREAM_DECIM");
            if (d && *d) {
                decim = atoi(d);
            }
            if (decim < 1) {
                decim = 1;
            }
        }
        if (!on) {
            return;
        }
        /* Skip all-zero frames (fn 0..4 at startup): they pollute the ring the
         * demodulator reads at the leading edge, which would then hit zeros
         * instead of the real FCCH pushed right after. */
        int nonzero = 0;
        for (int i = 0; i < n && i < 64; i++) {
            if (iq[i]) {
                nonzero = 1;
                break;
            }
        }
        if (!nonzero) {
            return;
        }
        for (int k = 0; 2 * (k * decim) + 1 < n; k++) {
            bsp_fbs[bsp_fbs_wr++ & (BSP_FBS_RING - 1)] = iq[2 * (k * decim)];
            bsp_fbs[bsp_fbs_wr++ & (BSP_FBS_RING - 1)] = iq[2 * (k * decim) + 1];
        }
    }
}

bool calypso_bsp_fb_stream_next(uint16_t *outI, uint16_t *outQ)
{
    if (bsp_fbs_rd + 1 >= bsp_fbs_wr) {
        return false;
    }
    *outI = (uint16_t)bsp_fbs[bsp_fbs_rd++ & (BSP_FBS_RING - 1)];
    *outQ = (uint16_t)bsp_fbs[bsp_fbs_rd++ & (BSP_FBS_RING - 1)];
    return true;
}

uint16_t calypso_bsp_rssi_apm(void)
{
    static double mav_ref = 20929.0, rf_ref = -60.0;
    static int init;
    if (!init) {
        init = 1;
        const char *mr = calypso_getenv("CALYPSO_DECAN_PM_MAV_REF");
        if (mr && *mr) {
            mav_ref = atof(mr);
        }
        const char *rr = calypso_getenv("CALYPSO_DECAN_PM_RF_REF");
        if (rr && *rr) {
            rf_ref = atof(rr);
        }
        if (mav_ref < 1.0) {
            mav_ref = 1.0;
        }
    }
    double mav = (double)bsp_last_mav;
    if (mav < 1.0) {
        mav = 1.0;
    }
    double rf = 20.0 * log10(mav / mav_ref) + rf_ref;
    if (rf < -100.0) {
        rf = -100.0;   /* floor: track a weak signal without rejecting the cell */
    }
    if (rf > -30.0) {
        rf = -30.0;
    }
    int rfi = (int)(rf >= 0 ? rf + 0.5 : rf - 0.5);
    return calypso_trf6151_apm_for_rf(rfi);
}

/* Per-TN burst queue: FN-indexed ring so lookahead bursts from the BTS
 * (osmo-bts-trx schedules up to ~92 frames ahead) are preserved until the
 * QEMU virtual FN catches up to each burst's scheduled FN. On real hardware
 * BSP DMA is synchronous within the TDMA frame; in QEMU bursts arrive over
 * UDP from the bridge with their scheduled FN embedded in the TRXD header,
 * and delivery must happen at the exact virtual FN or the DSP correlates
 * samples against a frame boundary that does not match the modulator phase
 * (→ d_fb_det stays 0 indefinitely). */
#define BSP_NUM_TN     8                 /* one queue per timeslot */
/* [2026-09-20] 128 -> 8192. With a real BTS the bursts arrive at 217 frames/s
 * while the emulated DSP consumes ~100/s; at 128 slots the queue overflowed
 * every second and dropped the OLDEST bursts, so the stream the ROM saw had
 * holes: FCCH spacings of 4, 26, 18 frames instead of 10 and the SCH never 1
 * frame after the FCCH it predicted (measured: 88 SB attempts, 0 decoded).
 * 8192 per timeslot holds ~70 s of the deficit; the stream stays coherent,
 * only late, which is what acquisition needs (the SB sets the firmware time
 * in stream time). 8 x 8192 x 780 B = 51 MB of BSS. */
#define BSP_QUEUE_LEN  8192              /* lookahead depth per TN */
/* Match window: the real BSP captures samples around BDLENA, so an exact FN
 * match is a QEMU artefact. 64 frames covers the BTS scheduler lookahead
 * (measured delta 1..139, mean ~50; a window of 4 left 99 % of bursts stale
 * before the virtual FN caught up) and stays narrow enough not to swap an FCCH
 * for a non-FCCH in the 51-multiframe pattern (GSM 45.002: FCCH every 10
 * frames on the BCCH slot). */
#define BSP_FN_MATCH_WINDOW  64

/* === DARAM write-by-range instrumentation ===
 *
 * Buckets:
 *   low    : DARAM zone read by AR3 with stride +19 in the FB-det correlator
 *   target : the runtime daram_addr window
 *   wrap   : circular wrap zone AR2/AR7, BK=176, stride -19
 *   other  : anywhere else, including the daram_len=296 overflow
 *
 * Emitted as `[BSP] DARAM-WR-STATS ...` every BSP_DARAM_WR_LOG_EVERY writes.
 * low=target=wrap=0 means the BSP DMA was never armed; target>>0 with low=0
 * means the BSP writes but the env override was ignored. */
#define BSP_BUCKET_LOW_LO     0x0000
#define BSP_BUCKET_LOW_HI     0x03A3
#define BSP_BUCKET_TARGET_LO  0x3FB0
#define BSP_BUCKET_TARGET_HI  0x3FFF
#define BSP_BUCKET_WRAP_LO    0xFC5D
#define BSP_BUCKET_WRAP_HI    0xFFED
#define BSP_DARAM_WR_LOG_EVERY 1000


/* Size of the I/Q buffer handed to the DSP, in int16 (2 per sample).
 *
 * 148 is a GSM burst in BITS (3+57+1+26+1+57+3), i.e. exactly one burst at
 * 1 SPS with no search margin. What the DSP actually CONSUMES (Osmocom wiki
 * HardwareCalypsoDSP, task by task):
 *   RX NB : 150 I/Q samples, 10-bit TSC window from r68, correlation over
 *           16 bits (TSC[10..25])                  -> 300 int16
 *   SB    : 190 I/Q samples, 50-bit window from r39, correlation over the
 *           full 64 bits                            -> 380 int16
 * So the DSP reads PAST a 296-int16 deposit: 2 samples too far for an NB, 42
 * for an SB, landing on stale neighbouring DARAM right inside the SB
 * correlation window.
 *
 * Sized for the worst case (SB, 190) plus margin. The runtime default stays
 * 296, so behaviour is unchanged; this only makes CALYPSO_BSP_DARAM_LEN=380
 * reachable past the old `n > 296` guard. */
#define BSP_IQ_MAX_I16   384   /* 192 I/Q samples; SB asks for 190 */

typedef struct {
    int16_t  iq[BSP_IQ_MAX_I16];
    int      n;        /* number of int16 values */
    uint32_t fn;
    bool     valid;
} BspBurstSlot;

typedef struct {
    BspBurstSlot  slot[BSP_QUEUE_LEN];
} BspBurstQueue;

static struct {
    C54xState *dsp;
    uint16_t   daram_addr;
    uint16_t   daram_len;
    uint64_t   bursts_seen;
    uint64_t   bursts_written;
    uint64_t   bursts_dropped_no_window;
    uint64_t   bursts_dropped_queue_full;
    uint64_t   bursts_dropped_stale;
    uint8_t    inject_canary;     /* CALYPSO_BSP_INJECT_CANARY=1: overwrite
                                      samples with 0xCAFE to identify the
                                      target buffer through the read trace */
    uint8_t    bypass_bdlena;      /* CALYPSO_BSP_BYPASS_BDLENA=1: deliver
                                      every burst without waiting for the
                                      BDLENA window — debug-only env-gated
                                      hack to probe the target DARAM addr */
    int        trxd_fd;            /* UDP socket for TRXDv0 DL bursts */
    struct sockaddr_in trxd_peer;  /* BTS address (for UL replies) */
    bool       trxd_peer_valid;
    uint8_t    last_att;           /* last DL attenuation byte */

    /* FN-indexed queue per TN */
    BspBurstQueue  q[BSP_NUM_TN];

    /* DARAM write-by-range counters (see BSP_BUCKET_*). */
    uint64_t   wr_low;
    uint64_t   wr_target;
    uint64_t   wr_wrap;
    uint64_t   wr_other;
    uint64_t   wr_total;
    uint64_t   wr_last_logged;

    /* Drain timer: decouples BSP->DSP DMA delivery from tdma_tick, which can
     * be slow under icount=auto. Armed on QEMU_CLOCK_REALTIME; see
     * bsp_drain_cb for why that clock and not VIRTUAL. */
    QEMUTimer *drain_timer;
} bsp;

#define BSP_DRAIN_PERIOD_MS  5

/* === Deterministic replay ===
 * With CALYPSO_BSP_REPLAY_FILE set, the BSP loads a burst dump
 * (BSP_DUMP_RX_FILE format) and injects it on QEMU_CLOCK_VIRTUAL at a fixed
 * rate INSTEAD of listening on the UDP socket, making the source fully
 * deterministic. Capture with BSP_DUMP_RX_FILE, then replay the same file.
 *
 * Rate: one burst every 576 us of virtual time = one GSM TN slot
 * (8 slots x 217 frames/s = ~1736 bursts/s), close enough to the TDMA rhythm. */
typedef struct ReplayBurst {
    uint32_t fn;
    uint8_t  tn;
    uint16_t n;
    int16_t  iq[BSP_IQ_MAX_I16];
} ReplayBurst;

static ReplayBurst *replay_bursts = NULL;
static size_t       replay_count  = 0;
static size_t       replay_idx    = 0;
static QEMUTimer   *replay_timer  = NULL;
#define BSP_REPLAY_PERIOD_NS  (576ULL * 1000ULL)  /* 576us per TN slot */
/* Nanosecond period, to pair with timer_new_ns / qemu_clock_get_ns. */
#define BSP_DRAIN_PERIOD_NS  (BSP_DRAIN_PERIOD_MS * 1000000ULL)

/* Bump the bucket matching `addr`, then emit a stats line periodically.
 * Called on every BSP-side DARAM write (rx_burst and deliver_buffered). */
static inline void bsp_daram_wr_bucket(uint16_t addr)
{
    bsp.wr_total++;
    /* The target zone follows the runtime daram_addr, i.e. what the BSP really
     * writes. daram_addr == 0 is discovery mode: no target zone, so every
     * write counts as "other". */
    uint16_t tgt_lo = bsp.daram_addr;
    uint16_t tgt_hi = bsp.daram_addr ? (uint16_t)(bsp.daram_addr + bsp.daram_len - 1) : 0;
    if (addr <= BSP_BUCKET_LOW_HI) {
        bsp.wr_low++;
    } else if (tgt_lo && addr >= tgt_lo && addr <= tgt_hi) {
        bsp.wr_target++;
    } else if (addr >= BSP_BUCKET_WRAP_LO && addr <= BSP_BUCKET_WRAP_HI) {
        bsp.wr_wrap++;
    } else {
        bsp.wr_other++;
    }
    if (bsp.wr_total - bsp.wr_last_logged >= BSP_DARAM_WR_LOG_EVERY) {
        bsp.wr_last_logged = bsp.wr_total;
        BSP_LOG("DARAM-WR-STATS low=%llu target=%llu wrap=%llu other=%llu total=%llu",
                (unsigned long long)bsp.wr_low,
                (unsigned long long)bsp.wr_target,
                (unsigned long long)bsp.wr_wrap,
                (unsigned long long)bsp.wr_other,
                (unsigned long long)bsp.wr_total);
    }
}

/* Signed hyperframe distance (entry_fn - reference_fn) in (-H/2, H/2]. */
static int32_t bsp_fn_delta(uint32_t entry_fn, uint32_t ref_fn)
{
    int32_t d = (int32_t)entry_fn - (int32_t)ref_fn;
    if (d > (int32_t)(GSM_HYPERFRAME / 2))       d -= GSM_HYPERFRAME;
    else if (d <= -(int32_t)(GSM_HYPERFRAME / 2)) d += GSM_HYPERFRAME;
    return d;
}

/* Enqueue a burst into queue[tn]. If a slot already carries the same FN,
 * overwrite it (duplicate retransmission from BTS). If the queue is full,
 * drop the oldest entry (smallest fn_delta relative to enqueue). */
static void bsp_enqueue(uint8_t tn, uint32_t fn, const int16_t *iq, int n)
{
    if (tn >= BSP_NUM_TN) return;
    BspBurstQueue *qq = &bsp.q[tn];

    int free_idx = -1;
    int oldest_idx = 0;
    int32_t oldest_delta = INT32_MAX;

    for (int i = 0; i < BSP_QUEUE_LEN; i++) {
        BspBurstSlot *s = &qq->slot[i];
        if (s->valid && s->fn == fn) {
            memcpy(s->iq, iq, n * sizeof(int16_t));
            s->n = n;
            return;
        }
        if (!s->valid) {
            if (free_idx < 0) free_idx = i;
        } else {
            int32_t d = bsp_fn_delta(s->fn, fn);
            if (d < oldest_delta) { oldest_delta = d; oldest_idx = i; }
        }
    }

    int idx;
    if (free_idx >= 0) {
        idx = free_idx;
    } else {
        idx = oldest_idx;
        bsp.bursts_dropped_queue_full++;
        if (bsp.bursts_dropped_queue_full == 1 || (bsp.bursts_dropped_queue_full % 10000) == 0)
            BSP_LOG("FILE PLEINE tn=%u : %llu bursts jetes (le DSP consomme moins vite que le BTS n'emet)",
                    tn, (unsigned long long)bsp.bursts_dropped_queue_full);
    }
    BspBurstSlot *s = &qq->slot[idx];
    memcpy(s->iq, iq, n * sizeof(int16_t));
    s->n = n;
    s->fn = fn;
    s->valid = true;
}

/* Purge entries older than the match window and return the slot whose FN
 * is closest to current_fn (within ±BSP_FN_MATCH_WINDOW) for this TN, or
 * NULL if none. Future bursts beyond the window stay queued. */
static BspBurstSlot *bsp_take_for_fn(uint8_t tn, uint32_t current_fn)
{
    if (tn >= BSP_NUM_TN) return NULL;
    BspBurstQueue *qq = &bsp.q[tn];
    BspBurstSlot *match = NULL;
    int32_t best_abs = INT32_MAX;
    /* FN-PROBE: track the NEAREST burst, ignoring the window, to expose the
     * burst_fn vs dispatcher_fn offset and drift even when everything is stale. */
    uint32_t near_fn = 0; int32_t near_d = 0; int32_t near_ad = INT32_MAX; int n_valid = 0;

    for (int i = 0; i < BSP_QUEUE_LEN; i++) {
        BspBurstSlot *s = &qq->slot[i];
        if (!s->valid) continue;
        int32_t d = bsp_fn_delta(s->fn, current_fn);
        int32_t ad = d < 0 ? -d : d;
        n_valid++;
        if (ad < near_ad) { near_ad = ad; near_d = d; near_fn = s->fn; }
        if (d < -BSP_FN_MATCH_WINDOW) {
            s->valid = false;
            bsp.bursts_dropped_stale++;
        } else if (ad <= BSP_FN_MATCH_WINDOW && ad < best_abs) {
            match = s;
            best_abs = ad;
        }
    }
    /* FN-PROBE (CALYPSO_BSP_FN_PROBE): the FN carried by the nearest burst (set
     * in bsp_enqueue) side by side with the FN the dispatcher tests here
     * (current_fn = calypso_trx_get_fn). A CONSTANT delta is an offset; a
     * DRIFTING delta is a clock problem. Capped at 300 then 1 in 500. */
    {
        static int fp = -1;
        if (fp < 0) fp = calypso_gate("CALYPSO_BSP_FN_PROBE", 0);
        if (fp && n_valid > 0) {
            static unsigned fpn = 0;
            if (fpn < 300 || (fpn % 500) == 0)
                BSP_LOG("FN-PROBE tn=%u dispatcher_fn=%u burst_fn=%u delta=%d "
                        "n_valid=%d verdict=%s window=%d",
                        tn, current_fn, near_fn, near_d, n_valid,
                        match ? "MATCH" : (near_d > BSP_FN_MATCH_WINDOW ? "FUTURE" : "STALE"),
                        BSP_FN_MATCH_WINDOW);
            fpn++;
        }
    }
    /* Periodic stale ratio summary: a runaway ratio (e.g. 7000:1) is the
     * symptom of a stalled DSP — virtual fn isn't catching up to queued
     * burst FNs before the match window expires. Report every 5000 stales
     * so the spiral is visible without flooding the log. */
    {
        static uint64_t last_logged_stale;
        if (bsp.bursts_dropped_stale - last_logged_stale >= 5000) {
            last_logged_stale = bsp.bursts_dropped_stale;
            BSP_LOG("STALE ratio: stale=%llu written=%llu (cur_fn=%u)",
                    (unsigned long long)bsp.bursts_dropped_stale,
                    (unsigned long long)bsp.bursts_written,
                    current_fn);
        }
    }
    return match;
}

/* TPU-RX-WIRE (RANK2): take the NEAREST valid buffered burst for a TN, ignoring
 * the FN-match window. Used when the TPU RX window fired (a BDLENA pulse was
 * consumed) : the hardware RX window IS the timing signal, so we deliver the
 * closest burst we have rather than waiting for a virtual-FN match that never
 * lands in full mode (device_fn >> virtual cur_fn). Marks the slot consumed. */
static BspBurstSlot *bsp_take_nearest(uint8_t tn, uint32_t current_fn)
{
    if (tn >= BSP_NUM_TN) return NULL;
    BspBurstQueue *qq = &bsp.q[tn];
    BspBurstSlot *best = NULL;
    int32_t best_abs = INT32_MAX;
    for (int i = 0; i < BSP_QUEUE_LEN; i++) {
        BspBurstSlot *s = &qq->slot[i];
        if (!s->valid) continue;
        int32_t d = bsp_fn_delta(s->fn, current_fn);
        int32_t ad = d < 0 ? -d : d;
        if (ad < best_abs) { best = s; best_abs = ad; }
    }
    return best;
}

static uint16_t parse_uint_env(const char *name, uint16_t def)
{
    const char *v = calypso_getenv(name);
    if (!v || !*v) return def;
    /* Auto-detect hex even without a 0x prefix: any non-decimal hex digit
     * (a-f / A-F) forces base 16. strtoul with base 0 parses "2a00" as
     * decimal and yields 2. */
    int base = 0;
    for (const char *p = v; *p; p++) {
        if ((*p >= 'a' && *p <= 'f') || (*p >= 'A' && *p <= 'F')) {
            base = 16;
            break;
        }
    }
    return (uint16_t)strtoul(v, NULL, base);
}

uint16_t calypso_bsp_get_daram_addr(void) { return bsp.daram_addr; }
uint32_t calypso_bsp_get_last_fn(void) { return calypso_daram_last_fn; }
uint16_t calypso_bsp_get_daram_len(void)  { return bsp.daram_len; }

/* Reference probe state: the last burst rx_burst received, and where it put it. */
/* Reference probe: a short history of the bursts handed to the BSP. The DSP
 * reads DARAM whenever its own DMA has drained the RIF, which is not
 * necessarily the frame the burst arrived in, so comparing against the LAST
 * burst alone cannot tell "nothing arrived" from "the previous one is still
 * there". Keeping four lets the probe name the AGE of what the DSP is
 * looking at, which is the question the SB turns on. */
#define BSP_VERIF_HIST 4
static struct {
    int16_t  iq[BSP_IQ_MAX_I16];
    int      n;
    uint16_t addr;
    uint32_t fn;
    uint16_t page;   /* d_dsp_page as it stood when the burst was handed over */
} bsp_verif_h[BSP_VERIF_HIST];
static unsigned bsp_verif_pos;   /* next slot to write */
static int      bsp_verif_plein; /* how many slots hold a burst */

/* d_dsp_page of the burst the last compare() matched. Sampled per burst, so a
 * [verif] line can state the page EXACTLY instead of interpolating it from the
 * every-37th [bsp-page] trace — which is what made the first page/address
 * correlation unreadable. */
static uint16_t bsp_verif_last_page;
uint16_t calypso_bsp_verif_last_page(void) { return bsp_verif_last_page; }

/* Compare DARAM against each recorded burst and return the best match. *age is
 * 0 for the burst handed over on this frame, 1 for the previous one, and so on;
 * the address compared is the one recorded with that burst. Returns the number
 * of identical words, or -1 when nothing has been recorded yet. */
int calypso_bsp_verif_compare(uint32_t *fn, uint16_t *addr, int *n, int *age)
{
    if (!bsp_verif_plein || !bsp.dsp) return -1;
    int best = -1, best_age = -1;
    for (int a = 0; a < bsp_verif_plein; a++) {
        unsigned i = (bsp_verif_pos + BSP_VERIF_HIST - 1 - (unsigned)a) % BSP_VERIF_HIST;
        int ident = 0;
        for (int k = 0; k < bsp_verif_h[i].n; k++)
            if ((int16_t)bsp.dsp->data[(bsp_verif_h[i].addr + k) & 0x3fff]
                == bsp_verif_h[i].iq[k]) ident++;
        if (ident > best) { best = ident; best_age = a; }
    }
    unsigned b = (bsp_verif_pos + BSP_VERIF_HIST - 1 - (unsigned)best_age) % BSP_VERIF_HIST;
    bsp_verif_last_page = bsp_verif_h[b].page;
    if (fn)   *fn   = bsp_verif_h[b].fn;
    if (addr) *addr = bsp_verif_h[b].addr;
    if (n)    *n    = bsp_verif_h[b].n;
    if (age)  *age  = best_age;
    return best;
}
uint8_t  calypso_bsp_get_last_att(void)   { return bsp.last_att; }

/* ---- UDP TRXDv0 DL receive callback ---- */

/* [2026-09-21] FN-KEYED TS0 STORE (real chain, CALYPSO_BSP_STREAM=1).
 * The BTS runs in real time, the emulated DSP does not: delivering each UDP
 * burst the moment it arrives hands the ROM bursts of arbitrary frames, and
 * once the ARM has synchronised (its frame counter = the BTS's, learnt from
 * the SB) the four bursts of a BCCH block are not the four it asked for -
 * every block failed the Fire code with perfect bursts. So every TS0 burst
 * is kept by BTS frame number (ring of 2^18 frames, ~20 min at half speed):
 *  - before synchronisation (FB/SB search) the bursts are played in arrival
 *    order, one per DSP tick;
 *  - an SB delivered into an SB window (one-shot RIF DMA of 191 samples)
 *    fixes offset = SB fn - tick fn, exactly the relation of the synthetic
 *    cell (whose SB carries the tick fn);
 *  - afterwards tick T gets the burst of BTS frame T + offset, modulated
 *    (GMSK, sampling instant 0.5, amplitude 30000, gmsk_elargir on the normal
 *    bursts) and framed for the armed window (3 or 21 leading samples). */
#define BSP_TS0_RING (1u << 18)
#define BSP_FN_MAX   2715648u
static struct { uint32_t fn; uint8_t bits[148]; uint8_t valid, joue; } *g_ts0;
static uint32_t g_ts0_next; static int g_ts0_any;
static int64_t g_ts0_offset = INT64_MIN;
static uint32_t g_ts0_fn_max;   /* trame BTS la plus recente stockee */
static unsigned long g_ts0_recales;      /* nb de recalages d'offset (voir bsp_ts0_service) */
static unsigned long g_ts0_silences;     /* effacements deposes (voir bsp_ts0_service) */
static long long     g_ts0_recul_total;  /* somme des reculs, en trames */
static const uint8_t bsp_train_sb[64] = { 1,0,1,1,1,0,0,1,0,1,1,0,0,0,1,0,0,0,0,0,0,1,0,0,0,0,0,0,1,1,1,1, 0,0,1,0,1,1,0,1,0,1,0,0,0,1,0,1,0,1,1,1,0,1,1,0,0,0,0,1,1,0,1,1, };
static int bsp_ts0_est_sb(const uint8_t *b) { for (int k = 0; k < 64; k++) if ((b[42 + k] & 1) != bsp_train_sb[k]) return 0; return 1; }
static int bsp_ts0_est_fcch(const uint8_t *b) { for (int k = 0; k < 148; k++) if (b[k] & 1) return 0; return 1; }
static int g_toa_bias = 0;   /* biais de placement, en echantillons ; voir calypso_bsp_toa_feedback() */
static void bsp_ts0_stocker(uint32_t fn, const uint8_t *bits)
{
    if (!g_ts0) g_ts0 = calloc(BSP_TS0_RING, sizeof *g_ts0);
    if (!g_ts0) return;
    unsigned i = fn % BSP_TS0_RING;
    g_ts0[i].fn = fn; memcpy(g_ts0[i].bits, bits, 148); g_ts0[i].valid = 1; g_ts0[i].joue = 0;
    /* [2026-09-22] La trame la plus recente recue du BTS. Sert a dire, sur un
     * manque, DE QUEL COTE vient l'ecart : si la trame reclamee depasse
     * celle-ci, le DSP court devant le BTS (il faut plus d'avance) ; si elle
     * est en-dessous, le burst a bien existe et c'est autre chose. Sans ce
     * signe, « pas de burst BTS » n'oriente vers rien. */
    if (!g_ts0_any || (int32_t)(fn - g_ts0_fn_max) > 0) { g_ts0_fn_max = fn; }
    bsp.bursts_seen++;   /* the drain loop of calypso_bsp_service() stops when this does not move */
    if (!g_ts0_any) { g_ts0_any = 1; g_ts0_next = fn; }
}
/* [2026-09-21] L'INTERVALLE DE LA FENETRE, DIT PAR LE FIRMWARE.
 *
 * tpu_window.c:89 : tpu_enq_offset((5000 + l1s.tpu_offset + 625*tn) % 5000).
 * Le registre TPU_OFFSET, que QEMU relaie dans chaque TICK, porte donc le
 * decalage de synchro ET la position de l'intervalle sur lequel la fenetre RX
 * est armee. Il etait stocke ici depuis le 2026-09-17 sans jamais etre lu.
 *
 * On ne connait pas l1s.tpu_offset, mais on n'en a pas besoin : hors mode
 * dedie l'ARM ecoute TS0, donc la valeur observee a ce moment-la EST la
 * reference. L'ecart a la reference, divise par 625, donne l'intervalle.
 * La reference se reapprend d'elle-meme a chaque resynchro, puisque le mobile
 * n'est alors pas en dedie. */
static int g_bsp_tpu_offset = 0;
static int g_tpu_ref = -1;

/* Intervalle de la fenetre courante, 0 si on ne sait pas. */
static int bsp_fenetre_tn(void)
{
    if (g_tpu_ref < 0) {
        return 0;
    }
    int d = ((g_bsp_tpu_offset - g_tpu_ref) % 5000 + 5000) % 5000;
    return ((d + 312) / 625) % 8;   /* 625 qbits par intervalle, arrondi */
}

/* [2026-09-21] L'INTERVALLE DEDIE.
 *
 * Le pont envoie les huit intervalles de chaque trame, le magasin ci-dessus
 * n'en garde qu'un (TS0) et bsp_ts0_livrer() complete la trame avec du
 * bourrage a zero. Tant que le mobile lit la BCCH et la CCCH, c'est exact :
 * tout ce qui l'interesse est sur TS0. Des qu'il passe en mode dedie, non :
 * le BSC lui alloue un SDCCH/8 sur TS1 (mesure du 2026-09-21 :
 * « [dcch] canal dedie arme : chan_nr=0x51 SDCCH/8 SS=2 TN=1 », suivi d'une
 * liberation immediate), et il n'entendait donc ni le UA ni le LOCATION
 * UPDATING ACCEPT.
 *
 * QEMU apprend cet intervalle du flux L1CTL du firmware et l'annonce par
 * PONT_DCCH ; on garde alors les bursts de CET intervalle-la, par numero de
 * trame du BTS, et bsp_ts0_livrer() les joue A LA PLACE de TS0. La trame
 * reste d'un seul burst par tick : le cadencement en 1250 symboles, si
 * durement regle, n'est pas touche.
 *
 * Un seul intervalle a la fois, comme le mobile : le magasin est alloue a
 * l'armement et repart a zero si l'intervalle change. */
#define BSP_DEDIE_RING (1u << 16)   /* ~5 min de trames BTS : le DSP en pas-a-pas derive de plusieurs secondes */
static struct { uint32_t fn; uint8_t bits[148]; uint8_t valid; } *g_dedie;

/* [2026-09-21] LA TRAME LIVREE AU DSP EST UNE VRAIE TRAME.
 *
 * bsp_ts0_livrer() assemblait « TS0 + sept intervalles de bourrage a zero ».
 * Le cadencement (un burst par tick, indexe par numero de trame BTS) n'est pas
 * une bequille : sans lui la ROM voit ~1526 symboles par trame au lieu de 1250
 * et son TOA ne se stabilise jamais -- mesure du 2026-09-21 en mode sans hack,
 * TOA a 2967, 3735, 4215, 48, 13536 et « BURST ID 3!=2 », « EMPTY ».
 * Le bourrage a zero, lui, EN EST une : le mobile n'entend rien de ce que la
 * BTS emet ailleurs que sur TS0. On garde donc les huit intervalles, et la
 * trame remise au DSP porte ce que la BTS a reellement emis. */
#define BSP_AUTRES_RING (1u << 16)
static struct { uint32_t fn; uint8_t bits[7][148]; uint8_t presents; } *g_autres;
static unsigned long g_autres_stockes, g_autres_joues;

static void bsp_autres_stocker(uint32_t fn, unsigned tn, const uint8_t *bits)
{
    if (tn < 1 || tn > 7) {
        return;
    }
    if (!g_autres) {
        g_autres = calloc(BSP_AUTRES_RING, sizeof *g_autres);
        if (!g_autres) {
            return;
        }
    }
    unsigned i = fn % BSP_AUTRES_RING;
    if (g_autres[i].fn != fn) {
        g_autres[i].fn = fn;
        g_autres[i].presents = 0;
    }
    memcpy(g_autres[i].bits[tn - 1], bits, 148);
    g_autres[i].presents |= (uint8_t)(1u << (tn - 1));
    g_autres_stockes++;
}

static const uint8_t *bsp_autres_bits(uint32_t fn, unsigned tn)
{
    if (!g_autres || tn < 1 || tn > 7) {
        return NULL;
    }
    unsigned i = fn % BSP_AUTRES_RING;
    if (g_autres[i].fn != fn || !(g_autres[i].presents & (1u << (tn - 1)))) {
        return NULL;
    }
    return g_autres[i].bits[tn - 1];
}
static int g_dedie_tn = -1, g_dedie_ss;
/* [2026-09-22] genre du canal dedie : 0 = SDCCH/4, 1 = SDCCH/8,
 * BSP_DEDIE_TCH = TCH -- sur un TCH TOUTES les trames de l'intervalle sont
 * a la connexion (multitrame de 26 : 24 trafic + 1 SACCH + 1 libre), pas
 * seulement les blocs d'un sous-canal. */
#define BSP_DEDIE_TCH 2
static int g_dedie_genre = 1;
static unsigned long g_dedie_stockes, g_dedie_joues, g_dedie_manques;
/* [2026-09-22] g_dedie_perdues : les trames dediees que bsp_ts0_service() saute
 * ENTIEREMENT, faute de burst du BTS pour la trame reclamee. Elle repart par un
 * `return` avant bsp_ts0_livrer(), donc ni `joues` ni `manques` ne les voient :
 * les deux comptaient 0 pendant que 79 trames dediees sur 172 n'etaient jamais
 * livrees. C'est ce zero qui m'a fait conclure « le pont livre tout, le defaut
 * est dans la demodulation du DSP » -- conclusion fausse, batie sur un
 * instrument aveugle a la perte majoritaire. Ne jamais lire manques=0 comme
 * « rien ne se perd » sans lire perdues en meme temps. */
static unsigned long g_dedie_perdues;

/* Cette trame du BTS appartient-elle au canal dedie du mobile ?
 *
 * [2026-09-21, mesure] Premiere version : une fois arme, l'intervalle dedie
 * remplacait TS0 a CHAQUE trame. Le BTS emet sur TS1 en permanence (bursts de
 * bourrage compris), donc le mobile n'avait plus de TS0 du tout : « FBSB RESP:
 * result=255 », « MON: no cell info », « LOS during RACH request » des qu'il
 * tentait une mise a jour, sans meme emettre un RACH. Il faut donc ne prendre
 * QUE les trames du canal, celles ou sa couche 1 arme justement sa fenetre sur
 * TS1 -- ailleurs il lit la BCCH et la CCCH sur TS0, et il en a besoin.
 *
 * 45.002, SDCCH/8 + SACCH/8, multitrame de 51 comptee par paires :
 *   SDCCH descendant, sous-voie i : trames 4i..4i+3 de chaque multitrame ;
 *   SACCH descendante, i = 0..3   : trames 32+4i..35+4i, multitrame paire ;
 *                      i = 4..7   : memes trames, multitrame impaire. */
static bool bsp_dedie_trame(uint32_t fn)
{
    if (g_dedie_genre == BSP_DEDIE_TCH) {
        return true;    /* TCH : tout l'intervalle est a la connexion */
    }
    unsigned p51 = fn % 51u;
    unsigned ss = (unsigned)g_dedie_ss & 7u;
    if (p51 >= 4u * ss && p51 <= 4u * ss + 3u) {
        return true;
    }
    unsigned base = 32u + 4u * (ss & 3u);
    if (p51 >= base && p51 <= base + 3u) {
        return ((fn / 51u) % 2u) == (ss < 4u ? 0u : 1u);
    }
    return false;
}

/* Etat du canal dedie, lisible sans le terminal du DSP :
 * /dev/shm/calypso_bsp_dedie, une ligne reecrite a chaque changement notable.
 * C'est la seule facon de savoir, de l'exterieur, si les bursts de
 * l'intervalle dedie arrivent jusqu'ici et s'ils sont joues. */
static unsigned long g_dedie_replis;   /* voir bsp_dedie_bits() */
static void bsp_dedie_etat(const char *quoi)
{
    static unsigned long dernier_joues, dernier_stockes, dernier_perdues;
    if (quoi == NULL && g_dedie_joues == dernier_joues &&
        g_dedie_stockes == dernier_stockes && g_dedie_perdues == dernier_perdues) {
        return;
    }
    dernier_joues = g_dedie_joues;
    dernier_stockes = g_dedie_stockes;
    dernier_perdues = g_dedie_perdues;
    FILE *f = fopen("/dev/shm/calypso_bsp_dedie", "w");
    if (!f) {
        return;
    }
    fprintf(f, "tn=%d ss=%d stockes=%lu joues=%lu manques=%lu replis=%lu perdues=%lu recales=%lu recul=%lld silences=%lu %s\n",
            g_dedie_tn, g_dedie_ss, g_dedie_stockes, g_dedie_joues,
            g_dedie_manques, g_dedie_replis, g_dedie_perdues,
            g_ts0_recales, g_ts0_recul_total, g_ts0_silences, quoi ? quoi : "");
    fclose(f);
}

void calypso_bsp_set_dedie(int tn, int genre, int ss)
{
    if (tn <= 0 || tn > 7 || genre == 0xFF) {
        if (g_dedie_tn > 0) {
            BSP_LOG("canal dedie libere (TS%d) : %lu bursts stockes, %lu joues",
                    g_dedie_tn, g_dedie_stockes, g_dedie_joues);
        }
        g_dedie_tn = -1;
        bsp_dedie_etat("libere");
        return;
    }
    if (g_dedie_tn != tn) {
        if (!g_dedie) {
            g_dedie = calloc(BSP_DEDIE_RING, sizeof *g_dedie);
        }
        if (g_dedie) {
            memset(g_dedie, 0, BSP_DEDIE_RING * sizeof *g_dedie);
        }
        g_dedie_stockes = g_dedie_joues = g_dedie_manques = g_dedie_replis = g_dedie_perdues = 0;
    }
    g_dedie_tn = tn;
    g_dedie_ss = ss;
    g_dedie_genre = genre;
    bsp_dedie_etat("arme");
    if (genre == BSP_DEDIE_TCH) {
        BSP_LOG("canal dedie arme : TCH TS%d - TOUTES ses trames remplacent TS0", tn);
        printf("  [ts0] canal dedie arme : TCH TS%d (toutes les trames)\n", tn);
        return;
    }
    BSP_LOG("canal dedie arme : SDCCH/%d SS=%d TS%d - ses bursts remplacent TS0 sur les "
            "trames du canal (fn%%51 = %u-%u et SACCH %u-%u), TS0 ailleurs",
            genre ? 8 : 4, ss, tn, 4u * ((unsigned)ss & 7u), 4u * ((unsigned)ss & 7u) + 3u,
            32u + 4u * ((unsigned)ss & 3u), 32u + 4u * ((unsigned)ss & 3u) + 3u);
}

static void bsp_dedie_stocker(uint32_t fn, const uint8_t *bits)
{
    if (!g_dedie) {
        return;
    }
    unsigned i = fn % BSP_DEDIE_RING;
    g_dedie[i].fn = fn;
    memcpy(g_dedie[i].bits, bits, 148);
    g_dedie[i].valid = 1;
    g_dedie_stockes++;
}

/* [2026-09-22] LE MAGASIN DEDIE COMMENCE TROP TARD : ON RETOMBE SUR g_autres.
 *
 * `g_dedie` n'est alloue et rempli qu'a l'ARMEMENT du canal
 * (`calypso_bsp_set_dedie`), et l'armement vient du tap L1CTL de QEMU, donc du
 * PREMIER bloc dedie recu -- pas de l'IMMEDIATE ASSIGNMENT. Or le BSP joue la
 * trame BTS `tick + g_ts0_offset`, en retard sur celle qui arrive : toutes les
 * trames du canal anterieures a l'armement etaient jouees VIDES.
 *
 * Mesure du 2026-09-22, `/dev/shm/calypso_bsp_dedie` a la liberation :
 *     stockes=4311 joues=404 manques=138
 * 138 sur 542 trames du canal, soit une sur quatre sans burst -- exactement la
 * signature des « Dropping frame with 110 bit errors » (110 sur 456 = un burst
 * sur quatre), des « MON: lev=<=-110 snr=0 » sur l'intervalle dedie, et des
 * « Received frame for unsupported SAPI 2 » / « MDL-ERROR-IND cause 3 » que
 * LAPDm sort d'un bloc reconstitue a partir de trois bursts sur quatre.
 * Cote TS0 au meme moment : 11 manques en tout. Ce n'est donc pas la BTS qui
 * est en retard, c'est ce magasin-ci qui ne couvre pas assez loin.
 *
 * Il n'y avait rien a stocker de plus : `g_autres` garde DEJA les sept
 * intervalles de chaque trame, sans condition et des le premier burst (il sert
 * a completer la trame continue). `g_dedie` en est un doublon partiel. On le
 * garde -- c'est lui qui compte les bursts du canal -- mais quand il n'a pas la
 * trame, on prend celle de `g_autres` au lieu de rendre NULL.
 *
 * MONTANT_DEDIE_STRICT=1 retablit l'ancien comportement (sans repli), pour
 * pouvoir remesurer l'ecart. */
static const uint8_t *bsp_dedie_bits(uint32_t fn)
{
    if (g_dedie_tn <= 0) {
        return NULL;
    }
    if (g_dedie) {
        unsigned i = fn % BSP_DEDIE_RING;
        if (g_dedie[i].valid && g_dedie[i].fn == fn) {
            return g_dedie[i].bits;
        }
    }
    static int strict = -1;
    if (strict < 0) {
        const char *e = calypso_getenv("MONTANT_DEDIE_STRICT");
        strict = (e && *e == '1');
    }
    if (strict) {
        return NULL;
    }
    const uint8_t *b = bsp_autres_bits(fn, (unsigned)g_dedie_tn);
    if (b) {
        g_dedie_replis++;
    }
    return b;
}

static void bsp_ts0_livrer(uint32_t tick_fn, unsigned i)
{
    static int16_t iq[2 * 256];
    const uint8_t *bits = g_ts0[i].bits;
    /* [2026-09-22] Ce que la trace [ts0] ne disait pas : si la trame livree
     * appartient au canal dedie, et si son burst vient bien de cet
     * intervalle-la. Sans quoi on ne peut pas comparer la geometrie de fenetre
     * d'un bloc SDCCH a celle d'un bloc BCCH -- la question ouverte apres
     * « manques=0 » et des blocs a 96 erreurs sur 456. */
    bool trame_dediee = false, burst_dedie = false;
    {   /* Sur les trames du canal dedie, c'est son intervalle qui compte. */
        /* Deux verdicts : la table 45.002 (trames du canal) et ce que dit le
         * firmware (intervalle de la fenetre). MONTANT_TPU_TN=1 fait foi au
         * second ; par defaut on garde le premier et on JOURNALISE les
         * desaccords, parce que la valeur du TICK peut avoir une trame de
         * retard sur la programmation reelle de la fenetre -- c'est justement
         * ce qu'il faut mesurer avant de s'y fier. */
        /* [2026-09-22] SANS CANAL ARME, AUCUNE TRAME N'EST « A NOUS ».
         * bsp_dedie_trame() lit g_dedie_ss, qui vaut 0 tant que rien n'est
         * arme : hors connexion, une trame sur huit etait donc declaree du
         * canal, bsp_dedie_bits() rendait NULL sur sa garde g_dedie_tn <= 0 et
         * g_dedie_manques montait. Le compteur melangeait ainsi le temps de
         * campement (ou jouer TS0 est la bonne chose) avec les vrais trous du
         * canal -- releve du 2026-09-22 : « joues=72 manques=162 », plus de
         * manques que de trames jouees, ce qui n'a aucun sens pour un canal
         * ouvert quelques secondes. Un compteur qu'on ne peut pas lire est
         * pire qu'absent : il a servi de preuve a un diagnostic faux. */
        bool a_nous = g_dedie_tn > 0 && bsp_dedie_trame(g_ts0[i].fn);
        {
            static int suit_tpu = -1;
            if (suit_tpu < 0) { const char *e = calypso_getenv("MONTANT_TPU_TN"); suit_tpu = (e && *e == '1'); }
            bool selon_tpu = (bsp_fenetre_tn() == g_dedie_tn);
            if (g_dedie_tn > 0 && selon_tpu != a_nous) {
                static unsigned long n;
                if (++n <= 40 || (n % 200) == 0) {
                    BSP_LOG("dedie : DESACCORD fn=%u p51=%u p102=%u : table=%d "
                            "tpu=%d (offset=%d ref=%d, tn_fenetre=%d) [%lu fois]",
                            g_ts0[i].fn, g_ts0[i].fn % 51u, g_ts0[i].fn % 102u,
                            (int)a_nous, (int)selon_tpu, g_bsp_tpu_offset,
                            g_tpu_ref, bsp_fenetre_tn(), n);
                }
            }
            if (suit_tpu) {
                a_nous = selon_tpu;
            }
        }
        trame_dediee = a_nous;
        const uint8_t *d = a_nous ? bsp_dedie_bits(g_ts0[i].fn) : NULL;
        /* [2026-09-23] SONDE SACCH/TF : sur TCH/F la parole et la FACCH suivent
         * la 26-multitrame, la SACCH la 104 (TS pair : fn%26 == 12, bloc de TS2
         * aux fn%104 = 38, 64, 90, 12). Appel de 15:42 : a_dd et a_fd bons,
         * a_cd FIRE KO a chaque bloc avec deux contenus constants, LOS au bout
         * de 32 blocs. On journalise ce que le BSP joue sur ces trames-la. */
        if (g_dedie_genre == BSP_DEDIE_TCH && a_nous && (g_ts0[i].fn % 26u) == 12u) {
            static unsigned long n_sacch;
            if (n_sacch++ < 24) {
                int uns = -1;
                if (d) { uns = 0; for (int k = 0; k < 148; k++) uns += (d[k] & 1); }
                printf("  [sacch_tf] tick=%u fn=%u fn%%104=%u TS%d burst=%s uns=%d one_shot=%d nwin=%d\n",
                       tick_fn, g_ts0[i].fn, g_ts0[i].fn % 104u, g_dedie_tn,
                       d ? "oui" : "NON", uns, (int)calypso_rhea_dma_one_shot(),
                       calypso_rhea_dma_one_shot() ? calypso_rhea_dma_get_len_words() / 2 : 0);
            }
            /* [2026-09-23] Les bits eux-memes, pour decoder la SACCH hors DSP
             * (A5 + gsm0503_xcch_decode) : si elle decode la, le defaut est
             * dans le chemin BSP/ROM, sinon dans ce que le BSP recoit.
             * Enregistrements de 156 octets : tick BE32, fn BE32, 148 bits 0/1.
             * 256 bursts au plus (64 blocs), fichier remis a zero au lancement. */
            static FILE *f_sacch;
            static unsigned n_sacch_bits;
            if (!f_sacch && n_sacch_bits == 0)
                f_sacch = fopen("/dev/shm/calypso_sacch_tf.bin", "wb");
            if (f_sacch && d && n_sacch_bits < 256) {
                uint8_t h[8] = { tick_fn >> 24, tick_fn >> 16, tick_fn >> 8, tick_fn,
                                 g_ts0[i].fn >> 24, g_ts0[i].fn >> 16, g_ts0[i].fn >> 8, g_ts0[i].fn };
                fwrite(h, 1, 8, f_sacch);
                fwrite(d, 1, 148, f_sacch);
                fflush(f_sacch);
                if (++n_sacch_bits == 256) { fclose(f_sacch); f_sacch = NULL; }
            }
        }
        if (d) {
            bits = d;
            burst_dedie = true;
            g_dedie_joues++;
            if (g_dedie_joues <= 40 || g_dedie_joues % 1000 == 0) {
                /* Combien de 1 dans les 148 bits : un burst de bourrage en a
                 * toujours le meme nombre, un vrai bloc varie. De quoi voir
                 * d'un coup d'oeil si la BTS emet vraiment le SDCCH. */
                int uns = 0;
                for (int k = 0; k < 148; k++) uns += (d[k] & 1);
                BSP_LOG("dedie : TS%d joue (fn=%u p51=%u uns=%d, %lu fois)",
                        g_dedie_tn, g_ts0[i].fn, g_ts0[i].fn % 51u, uns, g_dedie_joues);
            }
        } else if (a_nous) {
            /* La trame est au canal, mais aucun burst de cet intervalle n'est
             * arrive : le bloc sera incomplet et echouera au code de Fire. */
            g_dedie_manques++;
            if (g_dedie_manques <= 40 || g_dedie_manques % 1000 == 0) {
                BSP_LOG("dedie : TS%d MANQUANT pour fn=%u p51=%u (%lu fois) - "
                        "le bloc partira incomplet",
                        g_dedie_tn, g_ts0[i].fn, g_ts0[i].fn % 51u, g_dedie_manques);
            }
        }
        if (a_nous) {
            bsp_dedie_etat(NULL);
        }
    }
    int sb = bsp_ts0_est_sb(bits), fcch = bsp_ts0_est_fcch(bits);
    memset(iq, 0, sizeof iq);
    const bool one_shot = calypso_rhea_dma_one_shot();
    int nwin = one_shot ? calypso_rhea_dma_get_len_words() / 2 : 0;
    int marge = nwin >= 190 ? 21 : nwin >= 150 ? 3 : 0;
    gmsk_moduler(bits, 148, 30000, 0.0, 0.5, iq + 2 * marge);
    if (!sb && !fcch) {
        static double sym = -2;
        if (sym == -2) { const char *e = calypso_getenv("CALYPSO_BSP_NB_SYM"); sym = (e && *e) ? atof(e) : 0.3; }
        gmsk_elargir(iq + 2 * marge, 148, sym);
    }
    int total = marge > 0 ? (nwin > marge + 148 ? nwin : marge + 148) : 148;
    if (total > 256) total = 256;
    if (sb) {
        /* [2026-09-22] SONDE : la SB arrive-t-elle dans une VRAIE fenetre SB ?
         * Le calage de g_ts0_offset et la marge de 21 echantillons exigent
         * nwin >= 190 (fenetre de 382 mots). Mesure prealable : les fenetres
         * demandees font 151 et 64, jamais 382. Si c'est le cas, la SB est
         * cadree comme un burst normal (marge 3) et le calage ne se fait pas
         * par ce chemin. On compte les deux cas. */
        static unsigned n_sb, n_sb_fenetre;
        n_sb++;
        if (one_shot && nwin >= 190) n_sb_fenetre++;
        if (n_sb <= 20 || n_sb % 50 == 0) {
            printf("  [sbwin] SB #%u : one_shot=%d nwin=%d marge=%d -> %s (dans une vraie fenetre SB : %u/%u)\n",
                   n_sb, one_shot, nwin, marge,
                   (one_shot && nwin >= 190) ? "FENETRE SB" : "cadree comme un burst normal",
                   n_sb_fenetre, n_sb);
        }
    }
    if (sb && one_shot && nwin >= 190) {
        int64_t off = (int64_t)g_ts0[i].fn - (int64_t)tick_fn;
        if (off != g_ts0_offset) printf("  [ts0] SB fn=%u livree au tick %u dans une fenetre SB : offset ARM-tick = %lld%s\n",
                                        g_ts0[i].fn, tick_fn, (long long)off, g_ts0_offset == INT64_MIN ? " (premier calage)" : " (recalage)");
        g_ts0_offset = off;
    }
    /* [2026-09-21] Une ligne par trame livree : c'etait la sonde de calage des
     * fenetres (nwin > 0 est vrai pour TOUT burst normal depuis la DMA one-shot,
     * donc elle imprimait en continu). Repliee derriere
     * CALYPSO_BSP_TS0_DEBUG=1 ; par defaut, les 20 premieres livraisons puis
     * une sur 5000, de quoi voir que le flux tourne sans noyer la console. */
    { static int ts0_dbg = -1;
      if (ts0_dbg < 0) { const char *e = calypso_getenv("CALYPSO_BSP_TS0_DEBUG"); ts0_dbg = (e && *e && *e != '0'); }
      static unsigned nl, nd;
      /* Les trames du canal dedie sont rares (4 sur 51, plus la SACCH) et ne
       * durent que le temps d'une connexion : on les trace TOUTES, jusqu'a 300,
       * sans dependre de CALYPSO_BSP_TS0_DEBUG. C'est la seule facon de
       * comparer leur fenetre et leur marge a celles d'un bloc BCCH. */
      bool trace = ts0_dbg ? (nl < 400 || nl % 5000 == 0 || fcch || sb || nwin > 0)
                           : (nl < 20 || nl % 5000 == 0);
      nl++;
      if (trame_dediee && nd < 300) { trace = true; nd++; }
      if (trace) printf("  [ts0] tick=%u fn=%u p51=%u %s fenetre=%d marge=%d rif_avant=%d%s\n",
                        tick_fn, g_ts0[i].fn, g_ts0[i].fn % 51u,
                        sb ? "SB" : fcch ? "FCCH" : "NB", nwin, marge, calypso_rif_level(),
                        trame_dediee ? (burst_dedie ? "  <- canal dedie" : "  <- canal dedie, BURST MANQUANT") : ""); }
    g_ts0[i].joue = 1;
    calypso_bsp_rx_burst(0, g_ts0[i].fn, iq, 2 * total);
    bsp.bursts_written++;
    /* Continuous DMA (FB search): the ROM counts 1250 samples per frame and
     * the ARM turns its FB TOA into frames (ntdma) and bits with that
     * constant, so every tick must carry a WHOLE frame: TS0 (padded to 156
     * by calypso_bsp_rx_burst) then seven filler timeslots. With TS0 alone
     * (156 samples per tick) the SB window was armed eight ticks too late
     * and no SB ever decoded. One-shot windows take TS0 only. */
    if (!one_shot) {
        extern uint16_t g_remplissage_ts0[];   /* forward: defined below (filler, zeros by default) */
        static int16_t iq_autre[2 * 148];
        for (int tn = 1; tn < 8; tn++) {
            const uint8_t *b = bsp_autres_bits(g_ts0[i].fn, (unsigned)tn);
            if (!b) {
                calypso_bsp_rx_burst((uint8_t)tn, g_ts0[i].fn,
                                     (const int16_t *)g_remplissage_ts0, 2 * 148);
                continue;
            }
            memset(iq_autre, 0, sizeof iq_autre);
            gmsk_moduler(b, 148, 30000, 0.0, 0.5, iq_autre);
            {   /* meme elargissement que les bursts normaux de TS0 */
                static double sym2 = -2;
                if (sym2 == -2) { const char *e = calypso_getenv("CALYPSO_BSP_NB_SYM"); sym2 = (e && *e) ? atof(e) : 0.3; }
                gmsk_elargir(iq_autre, 148, sym2);
            }
            calypso_bsp_rx_burst((uint8_t)tn, g_ts0[i].fn, iq_autre, 2 * 148);
            g_autres_joues++;
        }
        if ((g_autres_joues % 5000) == 1) {
            BSP_LOG("trame complete : %lu bursts hors TS0 stockes, %lu joues "
                    "(le bourrage a zero ne sert plus que pour les manquants)",
                    g_autres_stockes, g_autres_joues);
        }
    }
}
/* [2026-09-22] L'HORLOGE DU BANC EST CELLE DU DSP, PAS CELLE DU MUR.
 *
 * bsp_ts0_service() joue la trame BTS `tick + g_ts0_offset`. L'offset est fixe
 * une fois pour toutes sur la premiere SB, et le tick de l'ARM avance au
 * rythme du C54x emule (~6,7 ms par trame) alors que la BTS, elle, tourne en
 * temps reel (4,615 ms). Le DSP consomme donc l'anneau 20 % moins vite qu'il
 * ne se remplit, et le retard n'est borne par rien.
 *
 * Mesure du 2026-09-22 (run de 10:01) : a 10:09, pont.py annoncait fn=99674
 * et le BSP jouait la trame BTS 80410 -- 19000 trames, 87 secondes de retard.
 * A 10 s de banc la mise a jour de localisation passait encore (retard ~2 s) ;
 * a 10:04 l'IMMEDIATE ASSIGNMENT arrivait 40 s apres le RACH (T3126 expire,
 * « lchan allocation failed : Timeout » cote BSC) ; a 10:06 plus rien ne
 * passait. Meme cause pour les « Dropping frame with 110 bit errors » du canal
 * dedie : l'anneau TS1 ne commence a se remplir qu'a l'armement du canal, or le
 * DSP lit des trames d'AVANT cet instant, d'ou les bursts « MANQUANT ».
 *
 * Le rattrapage cote DSP est impossible (il tourne deja a fond). C'est donc la
 * BTS qui doit ralentir : on publie ici la trame que le DSP reclame, et
 * pont.py (pont/trx.py, classe Clock) freine son horloge -- donc les IND CLOCK
 * de la BTS -- pour ne jamais la devancer de plus de quelques trames.
 *
 * Format, 16 octets : seq (non nul, incremente a chaque ecriture), cale
 * (0 avant le premier calage sur SB : l'horloge reste libre), trame BTS
 * reclamee, tick de l'ARM. CALYPSO_BSP_HORLOGE=0 coupe la publication. */
#define SHM_HORLOGE "/dev/shm/calypso_horloge"
static void bsp_horloge_publier(uint32_t fn_bts, uint32_t tick_fn, int cale)
{
    static int actif = -1, fd = -1;
    static uint32_t seq;
    if (actif < 0) {
        const char *e = calypso_getenv("CALYPSO_BSP_HORLOGE");
        actif = (e && *e == '0') ? 0 : 1;
    }
    if (!actif) {
        return;
    }
    if (fd < 0) {
        fd = open(SHM_HORLOGE, O_CREAT | O_WRONLY, 0644);
        if (fd < 0) {
            actif = 0;
            return;
        }
    }
    if (++seq == 0) {
        seq = 1;            /* 0 veut dire « rien de publie » cote lecteur */
    }
    uint32_t rec[4] = { seq, (uint32_t)cale, fn_bts, tick_fn };
    if (pwrite(fd, rec, sizeof rec, 0) != (ssize_t)sizeof rec) {
        /* rien a faire : l'horloge libre reprend la main cote pont */
    }
}

static void bsp_ts0_service(uint32_t tick_fn)
{
    if (!g_ts0 || !g_ts0_any) return;
    static unsigned manques, manques_log;
    if (g_ts0_offset != INT64_MIN) {
        int64_t w = ((int64_t)tick_fn + g_ts0_offset) % (int64_t)BSP_FN_MAX; if (w < 0) w += BSP_FN_MAX;
        unsigned i = (unsigned)w % BSP_TS0_RING;
        bsp_horloge_publier((uint32_t)w, tick_fn, 1);
        if (g_ts0[i].valid && g_ts0[i].fn == (uint32_t)w) { bsp_ts0_livrer(tick_fn, i); return; }
        /* [2026-09-22] LE DSP COURT DEVANT LE BTS : RECULER, NE PAS TROUER.
         *
         * L'offset tick->trame BTS etait pose UNE SEULE FOIS, au premier calage,
         * et jamais revu. Or le BTS s'arrete par a-coups : osmo-bts-trx bat ses
         * trames sur un timerfd cale en dur a 4615 us et ne sait pas suivre une
         * horloge plus lente -- son filtre de derive est un TODO vide
         * (scheduler_trx.c:571). Notre DSP tourne a 195,7 trames/s au lieu de
         * 216,7 (deficit 9,7 %, reproduit sur ~15 demarrages), donc le BTS se
         * croit en permanence « plus rapide que le TRX », compense, puis
         * resynchronise -- et pendant ce temps il ne produit RIEN. Le DSP, lui,
         * continue de ticker : il reclame alors des trames qui n'existent pas
         * encore. Mesure : ecart TOUJOURS positif, +1 a +40, 0 occurrence de
         * signe negatif sur 27200 manques ; par fenetres de ~200 trames a 100 %
         * de perte, ~43 fois par run.
         *
         * Trouer le flux est le pire choix possible : un bloc LAPDm, ce sont
         * QUATRE bursts consecutifs. Avec 42 % de trames dediees manquantes
         * (mesure du 2026-09-22) presque aucun bloc ne se forme, le
         * desentrelaceur sort du bruit, et le mobile lit 69 a 99 bits faux sur
         * 184 : ni SABM/UA, ni LU ACCEPT, ni CP-DATA du SMS.
         *
         * On recule donc l'offset pour repartir de la trame la plus recente
         * recue. Le DSP prend du retard sur le temps mural, mais son flux reste
         * CONTIGU -- et c'est tout ce que le montage demande : le commentaire du
         * mode STREAM le dit, « Decouples the absolute FN (BTS) from the tick FN
         * (QEMU): only the ORDER matters, and the SCH carries the real FN for
         * synchronisation. » Les trous deviennent de la latence.
         *
         * MESURE A/B DU 2026-09-22 (2 x 6 min, meme banc, meme protocole) :
         *
         *   |                      | temoin | recalage |
         *   | perdues / joues      | 20/132 |   0/161  |  <- fait ce qu'il annonce
         *   | MDL-ERROR            |   18   |     4    |  <- mieux
         *   | LU ACCEPT / REQUEST  |  1/3   |    1/2   |  <- egal
         *   | TRAMES JETEES        |  686   |  *1409*  |  <- DEUX FOIS PIRE
         *   | bits faux (mediane)  |   95   |    96    |  <- inchange
         *
         * CONCLUSION : ca ne marche pas. En remplacant une trame absente par la
         * plus recente disponible, on ne livre pas un trou mais LE MAUVAIS BURST
         * A LA BONNE PLACE. Pour le desentrelaceur une donnee fausse mais
         * plausible est pire qu'une absence : il ne peut plus la traiter comme
         * un effacement. On a converti des effacements en erreurs.
         * Corollaire : « perdues » n'est PAS un predicteur du succes. La
         * correlation vue sur trois runs (7 %, 0 %, 42 %) ne survit pas au test
         * controle.
         *
         * DESACTIVE PAR DEFAUT. CALYPSO_BSP_RECALE=1 pour le reessayer -- par
         * exemple en ne recalant que hors du canal dedie, ou en marquant le
         * burst rejoue comme peu fiable pour que le desentrelaceur l'efface.
         * Attention aussi : ce chemin sort avant `manques++`, donc il aveugle
         * le compteur « manques » -- les deux bras n'etaient pas comparables
         * sur cette metrique-la. */
        if ((int32_t)((uint32_t)w - g_ts0_fn_max) > 0) {
            static int recale = -1;
            if (recale < 0) { const char *e = calypso_getenv("CALYPSO_BSP_RECALE");
                              recale = (e && *e == '1') ? 1 : 0;   /* OFF par defaut : voir la mesure ci-dessus */
                              if (recale) BSP_LOG("recalage actif : le flux TS0 reste contigu quand le BTS prend du retard"); }
            if (recale) {
                unsigned j = g_ts0_fn_max % BSP_TS0_RING;
                if (g_ts0[j].valid && g_ts0[j].fn == g_ts0_fn_max) {
                    int64_t recul = (int64_t)(uint32_t)w - (int64_t)g_ts0_fn_max;
                    g_ts0_offset -= recul;
                    if (g_ts0_offset < 0) g_ts0_offset += BSP_FN_MAX;
                    g_ts0_recales++;
                    g_ts0_recul_total += recul;
                    if (g_ts0_recales <= 20 || g_ts0_recales % 200 == 0) {
                        printf("  [ts0] recalage %lu : le BTS avait %lld trames de retard (fn=%u -> %u), flux garde contigu\n",
                               g_ts0_recales, (long long)recul, (uint32_t)w, g_ts0_fn_max);
                    }
                    bsp_ts0_livrer(tick_fn, j);
                    return;
                }
            }
        }
        manques++;
        if (g_dedie_tn > 0 && bsp_dedie_trame((uint32_t)w)) {
            /* Trame du canal dedie sautee en entier : hors de portee de
             * g_dedie_manques, qui n'est touche que dans bsp_ts0_livrer(). */
            g_dedie_perdues++;
            if (g_dedie_perdues <= 40 || g_dedie_perdues % 200 == 0) {
                BSP_LOG("canal dedie (TS%d) : trame fn=%u perdue, aucun burst du BTS (perdues=%lu, joues=%lu)",
                        g_dedie_tn, (uint32_t)w, g_dedie_perdues, g_dedie_joues);
            }
            bsp_dedie_etat(NULL);
        }
        if (manques_log++ < 10 || manques % 200 == 0) {
            int32_t devant = (int32_t)((uint32_t)w - g_ts0_fn_max);
            printf("  [ts0] tick=%u : pas de burst BTS pour fn=%u (manques=%u/%u) ; derniere trame recue fn=%u, soit %+d : %s\n",
                   tick_fn, (uint32_t)w, manques, tick_fn, g_ts0_fn_max, devant,
                   devant > 0 ? "le DSP COURT DEVANT le BTS (avance insuffisante)"
                              : "le burst a existe puis a disparu (autre cause)");
        }
        /* [2026-09-22] UNE TRAME MANQUANTE DOIT ETRE UN EFFACEMENT, PAS LA
         * TRAME PRECEDENTE REJOUEE.
         *
         * Sortir ici sans rien deposer ne « troue » pas le flux : la page API
         * GARDE le burst du tick precedent, et la tache de decodage du DSP
         * tourne quand meme dessus. Chaine verifiee :
         *   - calypso_bsp_rx_burst() est le seul ecrivain de la DARAM des
         *     bursts, et n'est appelee que depuis bsp_ts0_livrer() -- que ce
         *     chemin-ci saute ;
         *   - calypso_rif_drain() rend 0 sur FIFO vide (calypso_rif.c) ;
         *   - le transfert sort alors par `if (got <= 0) break`
         *     (calypso_rhea_dma.c) SANS rien ecrire dans la page ;
         *   - sur le chemin DRR c'est pire encore, le dernier mot est repete :
         *     « On an empty FIFO, DRR keeps its last value [...] returning 0
         *     would fabricate a sample » (calypso_rif.c).
         *
         * SIGNATURE MESUREE, 2026-09-22 : les blocs rejetes par le mobile ont
         * un nombre d'erreurs IDENTIQUE -- 17 rejets = 96 neuf fois, 105 cinq
         * fois, puis 95, 73, 62. Un canal bruite ne rend pas neuf fois le meme
         * compte ; un decodeur qui relit deux fois la meme page, si.
         *
         * C'est aussi ce que l'A/B du recalage (voir plus haut) ne pouvait pas
         * voir : ses deux bras substituaient un burst FAUX -- le plus recent
         * d'un cote, le precedent de l'autre -- jamais un effacement. Des
         * echantillons nuls donnent des bits souples proches de zero, la seule
         * entree que le desentrelaceur puisse traiter comme une incertitude
         * plutot que comme une donnee.
         *
         * Ne touche a aucune trame livree : ce bloc ne s'execute QUE sur les
         * ticks ou, aujourd'hui, le DSP relit des echantillons perimes. Et il
         * ne fait pas baisser `perdues` -- c'est voulu, `perdues` sert ici de
         * temoin de non-effet.
         * CALYPSO_BSP_SILENCE=0 retablit exactement le comportement d'avant. */
        {
            static int silence = -1;
            if (silence < 0) {
                const char *e = calypso_getenv("CALYPSO_BSP_SILENCE");
                silence = (e && *e == '0') ? 0 : 1;
                if (silence) BSP_LOG("effacement actif : une trame sans burst depose des echantillons nuls");
            }
            if (silence) {
                static int16_t vide[2 * 256];
                const bool one_shot = calypso_rhea_dma_one_shot();
                int nwin  = one_shot ? calypso_rhea_dma_get_len_words() / 2 : 0;
                int marge = nwin >= 190 ? 21 : nwin >= 150 ? 3 : 0;
                int total = marge > 0 ? (nwin > marge + 148 ? nwin : marge + 148) : 148;
                if (total > 256) total = 256;
                memset(vide, 0, (size_t)(2 * total) * sizeof *vide);
                calypso_bsp_rx_burst(0, (uint32_t)w, vide, 2 * total);
                bsp.bursts_written++;
                /* En mode continu la trame doit faire 1250 echantillons, sinon
                 * le compteur de symboles de la ROM derive : memes remplissages
                 * que bsp_ts0_livrer(). */
                if (!one_shot) {
                    extern uint16_t g_remplissage_ts0[];
                    for (int tn = 1; tn < 8; tn++) {
                        calypso_bsp_rx_burst((uint8_t)tn, (uint32_t)w,
                                             (const int16_t *)g_remplissage_ts0, 2 * 148);
                    }
                }
                g_ts0_silences++;
                if (g_ts0_silences <= 20 || g_ts0_silences % 500 == 0) {
                    printf("  [ts0] silence %lu : fn=%u sans burst, effacement depose "
                           "(au lieu de relire la page du tick precedent)\n",
                           g_ts0_silences, (uint32_t)w);
                }
            }
        }
        return;
    }
    bsp_horloge_publier(0, tick_fn, 0);
    for (unsigned k = 0; k < BSP_TS0_RING; k++) {
        unsigned i = (g_ts0_next + k) % BSP_TS0_RING;
        if (g_ts0[i].valid && !g_ts0[i].joue) { bsp_ts0_livrer(tick_fn, i); g_ts0_next = g_ts0[i].fn + 1; return; }
    }
}
static void bsp_trxd_readable(void *opaque)
{
    /* The bridge sends 8 header bytes + 4*148 I/Q = 600 bytes, and a burst at
     * 4 SPS reaches 592 I/Q. Smaller buffers truncated silently: the BSP then
     * saw unconverted soft bits, the IQ_PASSTHROUGH branch was never taken,
     * the hard cos_tab fallback ran and AFC rotation was ineffective. */
    uint8_t buf[4096];
    struct sockaddr_in addr;
    socklen_t alen = sizeof(addr);

    ssize_t n = recvfrom(bsp.trxd_fd, buf, sizeof(buf), MSG_DONTWAIT,
                         (struct sockaddr *)&addr, &alen);
    if (n < 8) return;

    /* FEED-FP, leg 1/2 — INPUT. Read-only capped probe, CALYPSO_BSP_FINGERPRINT
     * (default 0).
     *
     * corr_iq.py measured 400/400 IDENTICAL FCCH bursts (rms=32533, coh=0.998,
     * both constant) at the DSP input, while the calypso-ipc-device source
     * served 10111 FCCH out of 103244 bursts = 9.8 %, exactly the GSM rate:
     * the freeze is INSIDE QEMU. This leg fingerprints what ARRIVES over UDP;
     * leg 2/2 (c54x_bsp_load) fingerprints what LEAVES towards the RIF. Varied
     * fingerprints here plus constant ones there bracket the freeze.
     *
     * ⚠️ Hash the WHOLE burst, not 8 words: two different bursts can share a
     * prefix, and a prefix comparison reads as a false freeze. */
    {
        static int fp_on = -1;
        if (fp_on < 0) fp_on = calypso_gate("CALYPSO_BSP_FINGERPRINT", 0);
        if (fp_on && n > 8) {
            uint32_t h = 2166136261u;      /* FNV-1a 32 bits */
            unsigned nz = 0;
            for (ssize_t i = 8; i < n; i++) {
                h = (h ^ buf[i]) * 16777619u;
                if (buf[i]) nz++;
            }
            static uint32_t prev_h;
            static unsigned long long n_tot, n_same;
            n_tot++;
            if (n_tot > 1 && h == prev_h) n_same++;
            prev_h = h;
            if (n_tot <= 20 || (n_tot % 500) == 0)
                fprintf(stderr, "[bsp] FEED-FP IN  #%llu fp=%08x nz=%u/%lld "
                        "identiques=%llu/%llu\n",
                        n_tot, h, nz, (long long)(n - 8), n_same, n_tot);
        }
    }

    /* Publish the DL I/Q: feeds the FB-STREAM ring (native correlator input)
     * and the magnitude measurement a_pm derives from. buf[8..] holds
     * interleaved int16 I/Q (cs16, passthrough mode). */
    if (n > 8) {
        bsp_iq_publish((const int16_t *)(buf + 8), (int)((n - 8) / 2));
    }

    /* Out-of-band I/Q tee (observability): copy of the raw burst to
     * CALYPSO_IQ_TEE_HOST:CALYPSO_IQ_TEE_PORT, read by the osmo-operator live
     * FFT. Opt-in: nothing is sent unless one of the two variables is set. */
    {
        const char *tee_p = calypso_getenv("CALYPSO_IQ_TEE_PORT");
        const char *tee_h = calypso_getenv("CALYPSO_IQ_TEE_HOST");
        if ((tee_p && *tee_p) || (tee_h && *tee_h)) {
            static int tee_fd = -1;
            static struct sockaddr_in tee_dst;
            if (tee_fd == -1) {
                tee_fd = socket(AF_INET, SOCK_DGRAM, 0);
                int port = (tee_p && *tee_p) ? atoi(tee_p) : 6703;
                memset(&tee_dst, 0, sizeof(tee_dst));
                tee_dst.sin_family = AF_INET;
                tee_dst.sin_port = htons(port);
                tee_dst.sin_addr.s_addr = (tee_h && *tee_h) ? inet_addr(tee_h)
                                                            : htonl(INADDR_LOOPBACK);
                BSP_LOG("IQ-TEE -> %s:%d (FFT live)",
                        (tee_h && *tee_h) ? tee_h : "127.0.0.1", port);
            }
            if (tee_fd >= 0) {
                sendto(tee_fd, buf, n, MSG_DONTWAIT,
                       (struct sockaddr *)&tee_dst, sizeof(tee_dst));
            }
        }
    }

    /* Diag: log the first 10 receive sizes to check that the bridge really
     * sends 600 bytes (I/Q mode) and that the BSP buffer does not truncate. */
    {
        static int rxsz_log = 0;
        if (rxsz_log++ < 10) {
            BSP_LOG("RXSZ #%d recv=%zd from %s:%d", rxsz_log,
                    n, inet_ntoa(addr.sin_addr), ntohs(addr.sin_port));
        }
    }

    /* Refine UL peer to actual DL sender (init-time default is bridge
     * 127.0.0.1:5702 — DL source confirms it or replaces it). */
    if (addr.sin_addr.s_addr != bsp.trxd_peer.sin_addr.s_addr ||
        addr.sin_port != bsp.trxd_peer.sin_port) {
        bsp.trxd_peer = addr;
        BSP_LOG("TRXD peer learned: %s:%d",
                inet_ntoa(addr.sin_addr), ntohs(addr.sin_port));
    }

    /* TRXDv0 DL: tn(1) fn(4) rssi(1) toa(2) bits(148) = 156 bytes. The RX
     * header is 8 bytes, like TX. Even when the BTS emits 154-byte packets,
     * the n-8 skip plus the 148-bit clamp keeps the DSP demod aligned; n-6
     * broke mobile L1 sync, leaving the mobile in the cell-selection loop
     * without ever reaching LU. */
    uint8_t  tn  = buf[0] & 0x07;
    uint32_t fn  = ((uint32_t)buf[1]<<24)|((uint32_t)buf[2]<<16)|
                   ((uint32_t)buf[3]<<8)|buf[4];
    bsp.last_att = (n > 5) ? buf[5] : 0;

    int nbits = (int)n - 8;  /* TRXDv0 header is 8 bytes (TS+FN+RSSI+ToA) */
    if (nbits > 148) nbits = 148;
    if (nbits <= 0) return;

    const uint8_t *bits = buf + 8;
    {   /* FN-keyed store of the TS0 bursts (see bsp_ts0_stocker); delivery
         * happens per DSP tick in calypso_bsp_service(). */
        static int stream = -1;
        if (stream < 0) { const char *e = calypso_getenv("CALYPSO_BSP_STREAM"); stream = (e && *e == '1') ? 1 : 0; }
        if (stream) {
            if (tn == 0 && nbits == 148) { bsp_ts0_stocker(fn, bits); return; }
            if (tn == g_dedie_tn && nbits == 148) {
                /* Le canal dedie : garde, sans rien changer au cadencement -
                 * bsp_ts0_livrer() le jouera a la place de TS0 quand la
                 * fenetre est en tir unique. */
                bsp_dedie_stocker(fn, bits);
            }
            if (tn >= 1 && tn <= 7 && nbits == 148) {
                /* Les autres intervalles de la MEME trame : ils prendront la
                 * place du bourrage a zero dans la trame continue. */
                bsp_autres_stocker(fn, tn, bits);
                bsp.bursts_seen++;
                return;
            }
            /* [2026-09-21] TS1..TS7 STOP HERE. The frame handed to the ROM is
             * assembled by bsp_ts0_livrer(), which already appends its own
             * seven filler timeslots after the stored TS0 burst - exactly 1250
             * symbols per tick. Letting the bridge's other timeslots carry on
             * to calypso_bsp_rx_burst() at the end of this function fed the RIF
             * a SECOND, unpaced stream on top of that one: measured 157
             * non-TS0 bursts/s against ~89 DSP ticks/s, i.e. ~276 extra
             * samples per ARM frame. The ROM, which counts its frame sample by
             * sample, then saw ~1526 symbols per frame instead of 1250: its FB
             * ToA never settled on 23 + n*1250 (observed residues 280/652/810/
             * 1152 modulo 1250), the RIF transit stage saturated (rif_avant=
             * 8196, oldest samples destroyed) and the narrow FB1 search never
             * found the tone again - FB0 hit, FB1 never, FBSB result=255.
             * bursts_seen must still move: the drain loop of
             * calypso_bsp_service() stops as soon as it does not. */
            bsp.bursts_seen++;
            { static unsigned long n_hors; if (n_hors++ == 0 || n_hors % 20000 == 0)
                  BSP_LOG("STREAM : %lu bursts hors TS0 ignores (la trame est assemblee par bsp_ts0_livrer)", n_hors); }
            return;
        }
    }

    /* Log burst type: check if all-zero (FB) or mixed (NB/SB) */
    {
        int zeros = 0, ones = 0;
        for (int i = 0; i < nbits; i++) {
            if (bits[i] == 0) zeros++;
            else ones++;
        }
        static int burst_log = 0;
        if (burst_log < 20 || (burst_log % 10000) == 0) {
            BSP_LOG("BURST fn=%u tn=%u zeros=%d ones=%d %s",
                    fn, tn, zeros, ones,
                    zeros == nbits ? "*** FB ***" :
                    ones > 100 ? "DUMMY/NB" : "SB/OTHER");
        }
        burst_log++;
    }

    /* FN-alignment instrumentation: measure burst arrival FN vs QEMU
     * virtual FN. A persistent negative delta means BTS is lagging
     * (bursts arrive for FNs that have already passed); a positive
     * delta is normal lookahead. */
    {
        uint32_t cur_fn = calypso_trx_get_fn();
        int32_t  delta  = bsp_fn_delta(fn, cur_fn);

        static int rx_log = 0;
        if (rx_log < 100 || (rx_log % 1000) == 0) {
            BSP_LOG("RX tn=%u fn=%u cur_fn=%u delta=%d",
                    tn, fn, cur_fn, delta);
        }
        rx_log++;

        /* Rolling summary over last 500 samples: min/max/mean */
        static int32_t  hist[500];
        static unsigned hist_pos = 0;
        static unsigned hist_seen = 0;
        hist[hist_pos] = delta;
        hist_pos = (hist_pos + 1) % 500;
        hist_seen++;
        if ((hist_seen % 500) == 0) {
            unsigned nh = hist_seen < 500 ? hist_seen : 500;
            int32_t mn = INT32_MAX, mx = INT32_MIN;
            int64_t sum = 0;
            for (unsigned i = 0; i < nh; i++) {
                int32_t d = hist[i];
                if (d < mn) mn = d;
                if (d > mx) mx = d;
                sum += d;
            }
            BSP_LOG("RX delta stats (last %u): min=%d max=%d mean=%lld",
                    nh, mn, mx, (long long)(sum / (int64_t)nh));
        }
    }

    /* GMSK modulation: convert TRXDv0 hard bits to I/Q samples.
     * GMSK with h=0.5: each bit adds ±π/2 to the phase.
     * NRZ encoding: bit 0 → phase += π/2, bit 1 → phase -= π/2.
     *
     * The Calypso IOTA chip delivers complex I/Q pairs to the BSP.
     * Phase increments are exactly ±π/2, so I=cos(φ) and Q=sin(φ)
     * cycle through {±1, 0}. We produce interleaved I,Q pairs.
     *
     * For FB (all-zero bits): phase advances π/2 per bit → pure tone.
     * I/Q sequence: (1,0),(0,1),(-1,0),(0,-1),(1,0),...
     *
     * IQ PASSTHROUGH: when the UDP payload is >= 296 bytes and
     * CALYPSO_BSP_IQ_PASSTHROUGH is on, buf[8..] is read as LE int16 I/Q pairs
     * (the format calypso-ipc-device sends: scipy GMSK at BT=0.3, more
     * realistic than our hard +/-pi/2 modulation). Otherwise the internal
     * modulator runs (148 hard bits -> 296 int16). */
    int16_t iq[BSP_IQ_MAX_I16];  /* see BSP_IQ_MAX_I16 */
    int iq_count = 0;

    static int iq_pt_mode = -1;
    if (iq_pt_mode < 0) {
        const char *e = calypso_getenv("CALYPSO_BSP_IQ_PASSTHROUGH");
        /* Passthrough is the DEFAULT: cos/sin synthesis yields an incoherent
         * tone. Opt out explicitly with CALYPSO_BSP_IQ_PASSTHROUGH=0. */
        iq_pt_mode = (e && *e == '0') ? 0 : 1;
        BSP_LOG("IQ_PASSTHROUGH=%d (defaut ON ; synthese=opt-out =0)", iq_pt_mode);
    }
    int iq_bytes = (int)n - 8;  /* payload bytes after 8-byte hdr */
    /* The bridge sends 2 int16 per bit (I,Q interleaved) = 4 bytes/bit, so
     * 146-148 bits is 584-592 payload bytes. Auto-detect: >= 4*146 bytes means
     * I/Q mode, otherwise soft bits at 1 byte per bit. */
    int iq_min_bits = 146;
    if (iq_pt_mode && iq_bytes >= 4 * iq_min_bits) {
        /* The device sends 592 I/Q at 4 SPS (OSR=4) while the DSP correlator
         * wants 148 samples at 1 SPS (FCCH = +pi/2 per sample). Decimate by
         * CALYPSO_BSP_IQ_DECIM (default 4): the +0.393 rad/sample tone at 4 SPS
         * becomes +0.393*decim = +pi/2. decim=1 keeps the first 148 samples at
         * 4 SPS = 37 symbols, which never correlates. */
        static int decim = -1;
        if (decim < 0) {
            const char *d = calypso_getenv("CALYPSO_BSP_IQ_DECIM");
            decim = (d && *d) ? atoi(d) : 4;
            if (decim < 1) decim = 1;
            BSP_LOG("IQ_DECIM=%d (STEP3 decimation ->1SPS)", decim);
        }
        const int16_t *isrc = (const int16_t *)(buf + 8);
        int total_cplx = iq_bytes / 4;   /* up to 592 (buf[4096]) */
        iq_count = 0;
        for (int k = 0; k * decim < total_cplx && iq_count <= BSP_IQ_MAX_I16 - 2; k++) {
            iq[iq_count++] = isrc[2 * (k * decim)];      /* I */
            iq[iq_count++] = isrc[2 * (k * decim) + 1];  /* Q */
        }
        nbits = iq_count / 2;

        /* WINDOW WIDENING (passthrough branch). Same reason as the synthesis
         * branch: the DSP consumes 150 samples for an NB and 190 for an SB,
         * and we delivered 148. The UDP payload is 592 complex at 4 SPS =
         * exactly 148 symbols, so the missing 42 symbol periods are not in it
         * either.
         *
         * Rather than arbitrary guard bits, EXTRAPOLATE the phase rotation
         * measured on the last two real samples and continue it. For an FCCH
         * (pure tone) this is exact; for a normal burst it is a phase-coherent
         * extrapolation, which avoids punching a hole right inside the SB
         * correlation window (50 bits from r39).
         *
         * ⚠️ THESE SAMPLES ARE SYNTHETIC. A detection that appears ONLY
         *    thanks to them is suspect and must be reported as such.
         * Default 148 leaves behaviour unchanged. */
        {
            static int win = -1;
            if (win < 0) {
                const char *e = calypso_getenv("CALYPSO_BSP_RX_WINDOW");
                win = (e && *e) ? atoi(e) : 148;
                if (win > BSP_IQ_MAX_I16 / 2) win = BSP_IQ_MAX_I16 / 2;
                if (win != 148)
                    BSP_LOG("RX_WINDOW=%d echantillons (passthrough) — "
                            "prolongement par extrapolation de phase, SYNTHETIQUE",
                            win);
            }
            if (win > nbits && iq_count >= 4) {
                double i1 = iq[iq_count - 2], q1 = iq[iq_count - 1];
                double i0 = iq[iq_count - 4], q0 = iq[iq_count - 3];
                double n0 = i0 * i0 + q0 * q0;
                double pr = 1.0, pi_ = 0.0;      /* rotation per sample */
                if (n0 > 0.0) {
                    pr  = (i1 * i0 + q1 * q0) / n0;
                    pi_ = (q1 * i0 - i1 * q0) / n0;
                }
                double cr = i1, ci = q1;
                while (nbits < win && iq_count + 1 < BSP_IQ_MAX_I16) {
                    double nr = cr * pr - ci * pi_;
                    double ni = cr * pi_ + ci * pr;
                    cr = nr; ci = ni;
                    if (cr >  32767.0) cr =  32767.0;
                    if (cr < -32768.0) cr = -32768.0;
                    if (ci >  32767.0) ci =  32767.0;
                    if (ci < -32768.0) ci = -32768.0;
                    iq[iq_count++] = (int16_t)cr;
                    iq[iq_count++] = (int16_t)ci;
                    nbits++;
                }
            }
        }

        /* AFC rotation (TWL3025 VCXO offset) is NOT applied here: it runs at
         * delivery time, where the DAC value is current instead of stale by
         * the lookahead depth. The AFC loop closes as firmware delta -> DSP
         * TSP -> TWL3025 DAC -> rotated samples -> DSP correlator. */
        /* calypso_twl3025_apply_phase(iq, copy_count / 2, fn, tn); */
        static int pt_log = 0;
        if (pt_log < 10 || (pt_log % 5000) == 0) {
            BSP_LOG("IQ passthrough #%d fn=%u tn=%u bytes=%d nbits=%d "
                    "iq[0..3]=%d,%d,%d,%d",
                    pt_log, fn, tn, iq_bytes, nbits,
                    iq[0], iq[1], iq[2], iq[3]);
        }
        pt_log++;
    } else {
        /* Q15 full-scale amplitude: real BSP/IOTA delivers near-full-range Q15
         * samples. ±0x7FFE keeps one bit of headroom below INT16_MIN. */
        /* [2026-09-20] GMSK (BT = 0.3, one sample per symbol at the symbol
         * centre), the same modulator as the synthetic cell of c54x_exe. The
         * hard +-90-degree MSK used before (I/Q on the phase points, full scale)
         * let the ROM find the FCCH tone but not decode the SCH: replayed on the
         * c54x_exe bench with that modulation, 1 CRC OK on 81 SB attempts
         * against 27 on 76 with GMSK, and the FB frequency estimate wandered to
         * +1050 Hz. Amplitude 30000 as the synthetic cell. */
        gmsk_moduler(bits, nbits, 30000, 0.0, 0.5, iq + iq_count);
        /* [2026-09-21] Same widening as the synthetic cell (gmsk_elargir,
         * CALYPSO_BSP_NB_SYM, default 0.3) on the normal bursts: the ROM zeroes
         * the +-1 channel taps of a plain 1-sps GMSK on about 60 percent of the
         * bursts and the block fails. FCCH (all zeros) and SB (training
         * sequence at bits 42..105) are left as they are: FB/SB lock natively
         * on them. */
        if (nbits == 148) {
            static const uint8_t train_sb[64] = { 1,0,1,1,1,0,0,1,0,1,1,0,0,0,1,0,0,0,0,0,0,1,0,0,0,0,0,0,1,1,1,1, 0,0,1,0,1,1,0,1,0,1,0,0,0,1,0,1,0,1,1,1,0,1,1,0,0,0,0,1,1,0,1,1, };
            static double sym = -2;
            if (sym == -2) { const char *e = calypso_getenv("CALYPSO_BSP_NB_SYM"); sym = (e && *e) ? atof(e) : 0.3; }
            int zeros = 1, sb = 1;
            for (int k = 0; k < 148 && zeros; k++) if (bits[k]) zeros = 0;
            for (int k = 0; k < 64 && sb; k++) if ((bits[42 + k] & 1) != train_sb[k]) sb = 0;
            if (!zeros && !sb) gmsk_elargir(iq + iq_count, nbits, sym);
        }
        iq_count += 2 * nbits;

        /* WINDOW WIDENING — CALYPSO_BSP_RX_WINDOW.
         *
         * We delivered exactly `nbits` samples, i.e. 148 = a GSM burst in BITS
         * (3+57+1+26+1+57+3), with no margin. What the DSP consumes (Osmocom
         * wiki HardwareCalypsoDSP, task by task):
         *   RX NB : 150 samples, 10-bit TSC window from r68, correlation over
         *           16 bits (TSC[10..25])
         *   SB    : 190 samples, 50-bit window from r39, correlation over the
         *           full 64 bits
         *
         * The capture does start AT THE BEGINNING of the burst: in an SB the
         * 64-bit training sequence occupies bits 42..105 (3 tail + 39 data +
         * 64 TSC) and the DSP searches 50 bits from r39 -> 39..89, which
         * contains 42. The missing samples are therefore AFTER the burst: the
         * guard period (8.25 bits) then the start of the neighbouring slot.
         * Hardware agrees: the TRF6151 analog window measures 914.6 us against
         * 577 us of useful burst, about 1.58x.
         *
         * ⚠️ ASSUMPTION, and the limit of this fix: we do NOT have the
         *    neighbouring slot signal — the source supplies one burst only. So
         *    we extend the modulator with GUARD bits (bit=1, like the firmware
         *    TX padding), which gives a defined, phase-continuous fill instead
         *    of zeros that would punch a hole right in the middle of the SB
         *    correlation window.
         *    => A detection that appears ONLY thanks to those samples is
         *    suspect: they are synthetic. Report it as such.
         *
         * Default 148 leaves behaviour unchanged. Use 190 to test SB.
         */
        {
            static int win = -1;
            if (win < 0) {
                const char *e = calypso_getenv("CALYPSO_BSP_RX_WINDOW");
                win = (e && *e) ? atoi(e) : 148;
                if (win < nbits) win = nbits;               /* never truncate */
                if (win > BSP_IQ_MAX_I16 / 2) win = BSP_IQ_MAX_I16 / 2;
                if (win != 148)
                    fprintf(stderr, "[bsp] RX_WINDOW = %d echantillons "
                            "(burst=%d + %d de garde SYNTHETIQUE) — "
                            "NB en demande 150, SB 190\n",
                            win, nbits, win - nbits);
            }
            for (int g = nbits; g < win && iq_count + 1 < BSP_IQ_MAX_I16; g++) {
                iq[iq_count++] = 0;                 /* guard: silence */
                iq[iq_count++] = 0;
            }
        }
    }

    /* Hand the burst straight to the DSP: the samples land in DARAM now, as
     * the RIF delivers them on silicon. There is no FN rendez-vous to keep —
     * the burst carries its own frame number for the SCH, and the correlator
     * only ever sees the window it is given. */
    /* [2026-09-21] Frame the TS0 burst like the synthetic cell when the ARM
     * has armed a task window (one-shot RIF DMA of 151 samples for a normal
     * burst, 191 for the SB; tpu_window.c): 3 (NB) or 21 (SB) silent samples
     * ahead, the window length in all. Measured on the real chain before: the
     * bare 148-sample burst put the ROM's NB TOA at 2 instead of 5, the
     * equaliser's pre-cursor tap fell outside its 3-tap window and every BCCH
     * block failed the Fire code; the SB read TOA 3 instead of 23. */
    if (tn == 0 && calypso_rhea_dma_one_shot()) {
        int nwin = calypso_rhea_dma_get_len_words() / 2;
        /* [2026-09-22] LE BIAIS DE LA BOUCLE TOA EST APPLIQUE ICI.
         * calypso_bsp_toa_feedback() (plus bas) mesure l'ecart entre le TOA vu
         * par la ROM et la cible 23, et l'integre dans g_toa_bias. Mais
         * g_toa_bias n'etait LU nulle part : la boucle calculait sa correction
         * et la jetait, malgre le commentaire « applied to the DARAM
         * placement » de sa declaration. CALYPSO_BSP_TOA_LOCK=1 n'avait donc
         * aucun effet observable.
         * Mesure du 2026-09-22 : la SB atterrit a TOA 7 (71 fois) ou 11 (17),
         * jamais 23 -- 16 echantillons trop tot -- et son CRC ne passe que
         * 31 fois sur 118 (26 %). En aval : la resynchro FB+SB echoue une fois
         * sur deux, d'ou « LOS during RACH request » et l'echec du SMS.
         * On ne corrige QUE la fenetre SB (nwin >= 190). Le burst normal garde
         * sa marge de 3, qui lui donne le TOA 5 attendu et mesure -- 7353
         * bursts sur 8046 -- et auquel il ne faut pas toucher. */
        int marge = nwin >= 190 ? 21 : nwin >= 150 ? 3 : 0;
        { static unsigned n; if (n++ < 40 || (n % 500) == 0)
            printf("  [cadre] nwin=%d marge=%d biais=%d iq=%d %s\n",
                   nwin, marge, g_toa_bias, iq_count, nwin >= 190 ? "(fenetre SB)" : "(NB)"); }
        if (nwin >= 190) {
            marge += g_toa_bias;
            if (marge < 0)  marge = 0;
            if (marge > 96) marge = 96;   /* la fenetre SB fait ~191 echantillons */
        }
        if (marge > 0 && iq_count >= 2 * 148) {
            static int16_t cadre[2 * 256];
            int total = nwin > marge + 148 ? nwin : marge + 148;
            if (total > 256) total = 256;
            int n = iq_count < 2 * 148 ? iq_count : 2 * 148;   /* the burst alone, guard dropped */
            memset(cadre, 0, sizeof cadre);
            memcpy(cadre + 2 * marge, iq, (size_t)n * sizeof(int16_t));
            { static unsigned nl; if (nl++ < 24 || nl % 5000 == 0) printf("  [livre] fn=%u p51=%u fenetre=%d marge=%d\n", fn, fn % 51u, nwin, marge); }
            calypso_bsp_rx_burst(tn, fn, cadre, 2 * total);
            return;
        }
    }
    calypso_bsp_rx_burst(tn, fn, iq, iq_count);

    /* Delivery is handled exclusively by calypso_bsp_deliver_buffered()
     * called from the TDMA tick. No immediate delivery — it would
     * double-consume BDLENA pulses and race with the buffered path. */
}

/* ---- Init ---- */

/* REALTIME drain callback: pulls the BSP UDP queue into DSP DMA at a 5 ms
 * wall-clock rate (200/s). Monotonic anti-drift rearm on `last_target +
 * period`, so dispatcher jitter does not accumulate.
 *
 * On QEMU_CLOCK_VIRTUAL under heavy DSP load the drain ran slower than wall
 * time, the BSP queue overflowed and 95 % of bursts were dropped. Since
 * tdma_tick is REALTIME-monotonic with a clk_master pthread, virtual and wall
 * are aligned and REALTIME matches the ARM frame_irq/tdma rate. */

/* Track the firmware tpu_offset (relayed by QEMU in the TICK). When the
 * firmware shifts its RX window (synchronize_tdma) the burst must follow so
 * the measured ToA converges to 23, i.e. native acquisition without a canned
 * ToA. Units: 4 qbits = 1 bit = 1 sample at 1 SPS. */
static int  g_bsp_tpu_ref = 0x7fffffff;   /* premier offset observe = origine */
void calypso_bsp_set_tpu_offset(int qbits)
{
    g_bsp_tpu_offset = qbits;
    if (g_dedie_tn <= 0) {
        g_tpu_ref = qbits;          /* pas de canal dedie : l'ARM est sur TS0 */
    }
}

/* Native ToA lock (CALYPSO_BSP_TOA_LOCK=1): slow closed loop driving the burst
 * placement bias so the ToA the DSP measures reaches 23 ("on time") WITHOUT
 * canning the output. The firmware then sees the alignment and stops
 * correcting. One-sample-per-frame integrator, for stability. */
/* defini plus haut, avant le cadrage qui l'applique */
void calypso_bsp_toa_feedback(int toa)
{
    static int en = -1;
    if (en < 0) {
        const char *e = calypso_getenv("CALYPSO_BSP_TOA_LOCK"); en = (e && *e=='1') ? 1 : 0;
        if (en) BSP_LOG("TOA_LOCK on : verrouillage natif du TOA sur 23 (biais placement)");
    }
    { static unsigned n; if (n++ < 40 || (n % 500) == 0)
        printf("  [toaloop] appel n=%u en=%d toa=%d biais=%d\n", n, en, toa, g_toa_bias); }
    if (!en || toa <= 0) return;
    int within = toa % 156;              /* intra-frame position (ntdma removed) */
    int err = within - 23;               /* target: 23 */
    if (err > 80) err -= 156;            /* take the shorter path across the wrap */
    if (err < -80) err += 156;
    if (err == 0) return;
    g_toa_bias -= (err > 0) ? 1 : -1;    /* slow integrator: burst earlier when ToA is too large */
}


/* Explicit drain for the non-QEMU host (c54x_exe --arm): the bsp_trxd_readable
 * iohandler and the bsp_drain_cb timer only exist inside the QEMU event loop.
 * Standalone, nobody empties UDP socket 6702 and bridge/BTS bursts pile up in
 * Recv-Q without ever reaching the DSP. Called once per frame from pont.c.
 * Returns the number of bursts read. */
/* [2026-09-20] STREAM frame assembler. One TDMA frame of the BTS stream is
 * handed to the DSP per tick, in timeslot order, ALWAYS 1250 symbols long:
 *   - continuous DMA (FB search): TS0..TS7 of that frame, each padded to
 *     156 (157 on TS3/TS7) symbols by calypso_bsp_rx_burst(); a timeslot the
 *     BTS did not send (idle TS carry nothing over TRXD) gets the dummy burst
 *     instead. The old path loaded the raw 148-symbol bursts that had arrived,
 *     so a frame was 1184 symbols at best and shorter with idle timeslots: the
 *     ROM's symbol counter drifted (FB TOA intra-frame offsets of 264..632
 *     instead of a constant), the FB1 narrow window and the SB frame missed.
 *   - one-shot DMA (the SB window): TS0 alone, framed like the synthetic cell
 *     (21 silent samples before and after), which is what puts the ROM's
 *     SB TOA near 23.
 * Slots older than the delivered frame are purged (their TS0 was lost). */
uint16_t g_remplissage_ts0[2 * 157];
#define g_remplissage g_remplissage_ts0
static bool bsp_source_toutes_ts;   /* the source delivers TS1..7 itself: no automatic fillers */
static BspBurstSlot *bsp_slot_exact(uint8_t tn, uint32_t fn)
{
    BspBurstQueue *qq = &bsp.q[tn];
    for (int i = 0; i < BSP_QUEUE_LEN; i++)
        if (qq->slot[i].valid && qq->slot[i].fn == fn) return &qq->slot[i];
    return NULL;
}
static void bsp_livrer_trame(uint32_t fn)
{
    static int16_t iq[2 * 256];
    const bool one_shot = calypso_rhea_dma_one_shot();
    bsp_source_toutes_ts = true;
    for (int tn = 0; tn < BSP_NUM_TN; tn++) {
        BspBurstSlot *sl = bsp_slot_exact((uint8_t)tn, fn);
        if (one_shot) {
            if (tn == 0 && sl) {
                /* The window geometry of tpu_window.c: the burst sits 23 symbols
                 * into the SB window (191 samples taken by the DSP, ALGTH 764)
                 * and 3 symbols into the NB window (151 samples, ALGTH 604);
                 * the 64-sample PM window takes the head of the burst. A 21
                 * margin in a 151-sample window cut the last 18 symbols of
                 * every normal burst. */
                int nwin = calypso_rhea_dma_get_len_words() / 2;
                int marge = nwin >= 190 ? 21 : nwin >= 150 ? 3 : 0;
                int total = nwin > marge + 148 ? nwin : marge + 148;
                if (total > 256) total = 256;
                int n = sl->n < 296 ? sl->n : 296;
                memset(iq, 0, sizeof iq);
                memcpy(iq + 2 * marge, sl->iq, (size_t)n * sizeof(int16_t));
                calypso_bsp_rx_burst(0, fn, iq, 2 * total);
                bsp.bursts_written++;
                { static unsigned nl; if (nl++ < 40 || nl % 2000 == 0) printf("  [livre] fn=%u p51=%u one_shot=1 nwin=%d marge=%d n=%d total=%d\n", fn, fn % 51u, nwin, marge, n / 2, total); }
            }
        } else if (sl) {
            int n = sl->n < 296 ? sl->n : 296;
            /* [2026-09-21] TS0 in STREAM mode: frame it like the one-shot
             * branch whenever the RIF DMA length is a task window (151 = NB,
             * 191 = SB), even if the one-shot flag reads false at delivery
             * time. Measured on the real chain: the NB bursts reached the ROM
             * at TOA 2 (no margin) instead of 5 as on the synthetic cell, the
             * equaliser's pre-cursor tap fell outside its window and every
             * block failed the Fire code. */
            int nwin = tn == 0 ? calypso_rhea_dma_get_len_words() / 2 : 0;
            int marge = nwin >= 190 ? 21 : nwin >= 150 ? 3 : 0;
            if (tn == 0 && marge > 0) {
                int total = nwin > marge + 148 ? nwin : marge + 148;
                if (total > 256) total = 256;
                memset(iq, 0, sizeof iq);
                memcpy(iq + 2 * marge, sl->iq, (size_t)n * sizeof(int16_t));
                calypso_bsp_rx_burst(0, fn, iq, 2 * total);
            } else
                calypso_bsp_rx_burst((uint8_t)tn, fn, sl->iq, n);
            bsp.bursts_written++;
            { static unsigned nl; if (tn == 0 && (nl++ < 40 || nl % 2000 == 0)) printf("  [livre] fn=%u p51=%u one_shot=0 nwin=%d marge=%d n=%d\n", fn, fn % 51u, nwin, marge, n / 2); }
        } else {
            calypso_bsp_rx_burst((uint8_t)tn, fn, (const int16_t *)g_remplissage, 2 * 148);
        }
        if (sl) sl->valid = false;
    }
    /* purge what is older than this frame */
    for (int tn = 0; tn < BSP_NUM_TN; tn++)
        for (int i = 0; i < BSP_QUEUE_LEN; i++) {
            BspBurstSlot *s2 = &bsp.q[tn].slot[i];
            if (s2->valid && bsp_fn_delta(s2->fn, fn) < 0) { s2->valid = false; bsp.bursts_dropped_stale++; }
        }
    { static unsigned nl; if (nl++ < 3) BSP_LOG("STREAM trame fn=%u livree (%s)", fn, one_shot ? "fenetre SB, TS0 + marges" : "8 TS, 1250 symboles"); }
}

/* [2026-09-23] ATTENDRE LA TRAME EN RETARD PLUTOT QUE LA TROUER.
 *
 * osmo-bts-trx bat ses trames sur son propre timer et se recale sur chaque
 * IND CLOCK du pont (toutes les 51 trames) : il cale, puis rattrape d'un bloc.
 * Les trames d'une meme phase de la multitrame arrivent donc APRES que le DSP
 * les reclame, et bsp_ts0_service() les remplace par un effacement. Releve du
 * 2026-09-23 14:50, SDCCH/8 SS=4 : « canal dedie (TS1) : trame fn=99109..99112
 * perdue, aucun burst du BTS », puis 99211..99214, 99313..99316 -- le bloc
 * descendant du mobile (fn%51 = 16..19) un sur deux ; l'UA ne passait jamais
 * (« I frame ignored in state SABM_SENT », T200).
 *
 * La BTS est asservie au DSP (pont/dsp/clock.py) : attendre quelques
 * millisecondes qu'elle livre ne cree pas de derive, ca cale le DSP sur elle.
 * On n'attend que si la trame reclamee est DEVANT la derniere recue (la BTS est
 * en retard, la trame va arriver) et de moins de 200 trames ; sinon (burst
 * disparu, ou calage perdu) on garde le comportement d'avant.
 * CALYPSO_BSP_ATTENTE_MS : 0 (defaut, QEMU inchange) = pas d'attente ;
 * c54x_exe/run.sh pose 40 en MODE=dsp. */
static void bsp_attendre_trame(uint32_t tick_fn)
{
    static int max_ms = -1;
    static unsigned long attentes, servies, echues;
    static double total_ms;
    if (max_ms < 0) {
        const char *e = calypso_getenv("CALYPSO_BSP_ATTENTE_MS");
        max_ms = e ? atoi(e) : 0;
        if (max_ms > 0) {
            printf("  [ts0] attente des trames en retard de la BTS : jusqu'a %d ms\n", max_ms);
        }
    }
    if (max_ms <= 0 || !g_ts0 || !g_ts0_any || g_ts0_offset == INT64_MIN || bsp.trxd_fd < 0) {
        return;
    }
    int64_t w = ((int64_t)tick_fn + g_ts0_offset) % (int64_t)BSP_FN_MAX;
    if (w < 0) w += BSP_FN_MAX;
    unsigned i = (unsigned)w % BSP_TS0_RING;
    if (g_ts0[i].valid && g_ts0[i].fn == (uint32_t)w) {
        return;
    }
    int32_t devant = (int32_t)((uint32_t)w - g_ts0_fn_max);
    if (devant <= 0 || devant > 200) {
        return;
    }
    attentes++;
    int64_t t0 = qemu_clock_get_ns(QEMU_CLOCK_REALTIME);
    int64_t fin = t0 + (int64_t)max_ms * 1000000;
    bool servie = false;
    for (;;) {
        int64_t reste = fin - qemu_clock_get_ns(QEMU_CLOCK_REALTIME);
        if (reste <= 0) {
            break;
        }
        struct pollfd p = { .fd = bsp.trxd_fd, .events = POLLIN };
        if (poll(&p, 1, (int)((reste + 999999) / 1000000)) <= 0) {
            break;
        }
        for (int k = 0; k < 256; k++) {
            unsigned long long avant = bsp.bursts_seen;
            bsp_trxd_readable(NULL);
            if (bsp.bursts_seen == avant) break;
        }
        if (g_ts0[i].valid && g_ts0[i].fn == (uint32_t)w) {
            servie = true;
            break;
        }
    }
    double ms = (qemu_clock_get_ns(QEMU_CLOCK_REALTIME) - t0) / 1e6;
    total_ms += ms;
    if (servie) servies++; else echues++;
    if (attentes <= 10 || attentes % 1000 == 0) {
        printf("  [ts0] attente %lu : fn=%u (%+d devant la BTS) %s en %.1f ms "
               "(servies=%lu echues=%lu, %.0f ms au total)\n",
               attentes, (uint32_t)w, devant, servie ? "arrivee" : "ECHUE", ms,
               servies, echues, total_ms);
    }
}

int calypso_bsp_service(uint32_t current_fn)
{
    int n = 0;
    /* 1) empty the UDP socket into the internal queue (bsp_trxd_readable) */
    while (bsp.trxd_fd >= 0 && n < 256) {
        unsigned long long before = bsp.bursts_seen;
        bsp_trxd_readable(NULL);
        if (bsp.bursts_seen == before) break;   /* recvfrom < 8: nothing left */
        n++;
    }
    /* STREAM (CALYPSO_BSP_STREAM=1): deliver ONE TS0 burst per frame, the
     * oldest in FN order, instead of everything. The DSP then sees a stream of
     * CONSECUTIVE frames as on silicon: the firmware FB search finds the FCCH
     * at a CONSTANT position, the ToA stabilises and acquisition locks without
     * stalling the output. Decouples the absolute FN (BTS) from the tick FN
     * (QEMU): only the ORDER matters, and the SCH carries the real FN for
     * synchronisation. */
    static int stream = -1;
    if (stream < 0) { const char *e = calypso_getenv("CALYPSO_BSP_STREAM"); stream = (e && *e=='1') ? 1 : 0;
                      if (stream) BSP_LOG("STREAM on : 1 burst TS0/trame en ordre FN (cohérence horloge)"); }
    if (stream) {
        /* [2026-09-21] FN-keyed TS0 store (bsp_ts0_service): arrival order
         * until an SB fixes the ARM/tick offset, then the burst of the ARM's
         * frame at every tick. */
        bsp_attendre_trame(current_fn);
        bsp_ts0_service(current_fn);
        return n;
    }
    /* 2) otherwise: deliver this frame's bursts (DARAM + interrupt). */
    calypso_bsp_deliver_buffered(current_fn);
    return n;
}

static void bsp_drain_cb(void *opaque)
{
    static int64_t last_target = 0;
    /* Drain the DL UDP socket HERE, off the reliable REALTIME timer. Under
     * icount=auto the DSP (c54x_run) monopolises the mainloop thread, so the
     * bsp_trxd_readable iohandler is never served, device packets pile up
     * unread (Recv-Q grows) and delivery stops (BSP-DELIVER=0, D_BURST_D
     * empty, snr=0). 64 iterations is margin (~1-2 bursts per 5 ms). */
    {
        /* Direct PEEK on bsp.trxd_fd: is the data on THIS fd?
         * (errno=EAGAIN means nothing here; >0 means data is present.) */
        uint8_t tb[16]; struct sockaddr_in sa; socklen_t sl = sizeof(sa);
        errno = 0;
        ssize_t pk = (bsp.trxd_fd >= 0)
            ? recvfrom(bsp.trxd_fd, tb, sizeof(tb), MSG_DONTWAIT | MSG_PEEK,
                       (struct sockaddr *)&sa, &sl)
            : -99;
        int e = errno;
        for (int i = 0; i < 64 && bsp.trxd_fd >= 0; i++)
            bsp_trxd_readable(NULL);
        static uint64_t dc = 0;
        if (dc < 30 || (dc % 2000) == 0)
            /* Report `bursts_written` (incremented in deliver_buffered when a
             * burst really lands in DARAM), not `bursts_seen`: bursts_seen only
             * moves in calypso_bsp_rx_burst, which the inline-write delivery
             * path bypasses, so it reads 0 forever. */
            fprintf(stderr, "[BSP] DRAIN-CB #%llu fd=%d PEEK=%zd errno=%d "
                    "delivered=%llu enq_drops(stale=%llu,full=%llu) seen_DEAD=%llu\n",
                    (unsigned long long)dc, bsp.trxd_fd, pk, e,
                    (unsigned long long)bsp.bursts_written,
                    (unsigned long long)bsp.bursts_dropped_stale,
                    (unsigned long long)bsp.bursts_dropped_queue_full,
                    (unsigned long long)bsp.bursts_seen);
        dc++;
    }
    if (bsp.dsp) {
        uint32_t cur_fn = calypso_trx_get_fn();
        calypso_bsp_deliver_buffered(cur_fn);
    }
    int64_t now = qemu_clock_get_ns(QEMU_CLOCK_REALTIME);
    if (last_target == 0) last_target = now;
    int64_t target = last_target + BSP_DRAIN_PERIOD_NS;
    while (target <= now) {
        target += BSP_DRAIN_PERIOD_NS;
    }
    last_target = target;
    timer_mod(bsp.drain_timer, target);
}

/* Replay callback: enqueue one burst per virtual TN slot. */
static void bsp_replay_cb(void *opaque)
{
    if (replay_idx < replay_count) {
        ReplayBurst *r = &replay_bursts[replay_idx++];
        bsp_enqueue(r->tn, r->fn, r->iq, r->n);
        if (replay_idx <= 5 || (replay_idx % 1000) == 0) {
            BSP_LOG("REPLAY inject #%zu fn=%u tn=%u n=%u",
                    replay_idx, r->fn, (unsigned)r->tn, (unsigned)r->n);
        }
    } else if (replay_idx == replay_count && replay_count > 0) {
        BSP_LOG("REPLAY exhausted after %zu bursts (idle from now)",
                replay_count);
        replay_idx++;  /* prevent log spam */
    }
    timer_mod(replay_timer,
              qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + BSP_REPLAY_PERIOD_NS);
}

/* Load all bursts from a BSP_DUMP_RX_FILE-format dump into memory.
 * Returns number loaded, 0 on failure. */

/* RX delivery: end-of-transfer interrupt of the RIF-RX DMA channel.
 *
 * CAL000 §3.7.1 allows only two vectors for the RIF, one per exchange mode, and
 * both are unmasked in the measured IMR (0x50ef):
 *   vec16 / bit 0  = INT0n  "RIF receive"  — XIO mode, one word at a time
 *   vec30 / bit 14 = INT10n "DMA interrupt" — buffered mode, dedicated RIF-RX
 *                    channel (CAL000 §6: "The RIF-RX and RIF-TX have a
 *                    dedicated channel each"; "an end-DMA request is sent")
 * The BSP deposits a BUFFER in DARAM, so this is buffered mode: vec30, whose
 * trampoline is at 0x0158. CAL000 §5.1 has the vector map.
 *
 * Two earlier choices were wrong. vec21 (XINT = SPI transmit) and vec19 (TINT =
 * DSP timer) are RETE stubs in the PDROM table, unrelated to the RIF; a
 * native_twl run announced 24644 bursts on vec21 and 24857 on vec19, so the
 * correlator was never told a burst had arrived. vec28/bit12 is the TPU FRAME
 * interrupt, which announces "new frame", not "a burst arrived": the DSP arms
 * its RX window then waits for the end-of-reception interrupt.
 *
 * Measured on the deterministic replay (c54x_exe --rejouer, synthetic cell),
 * FB correlator (0x770a) executions:
 *   vec28/12 : 0      vec16/0 : 0      vec30/14 : 110 */
static void calypso_bsp_deliver(C54xState *dsp, int vec, int bit)
{
    (void)vec; (void)bit;
    c54x_interrupt_ex(dsp, C54X_IT_DMA_VEC, C54X_IT_DMA_BIT);
}

static size_t bsp_replay_load(const char *path)
{
    FILE *f = fopen(path, "rb");
    if (!f) {
        BSP_LOG("REPLAY open '%s' failed: %s", path, strerror(errno));
        return 0;
    }
    size_t loaded = 0;
    size_t cap = 256;
    replay_bursts = calloc(cap, sizeof(ReplayBurst));
    if (!replay_bursts) { fclose(f); return 0; }
    while (1) {
        uint8_t hdr[12];
        if (fread(hdr, 1, 12, f) != 12) break;
        if (memcmp(hdr, "IQ16", 4) != 0) {
            BSP_LOG("REPLAY bad magic at burst %zu, stop", loaded);
            break;
        }
        uint32_t fn = (uint32_t)hdr[4]
                    | ((uint32_t)hdr[5] << 8)
                    | ((uint32_t)hdr[6] << 16)
                    | ((uint32_t)hdr[7] << 24);
        uint8_t  tn = hdr[8];
        uint16_t n  = (uint16_t)hdr[9] | ((uint16_t)hdr[10] << 8);
        if (n == 0 || n > BSP_IQ_MAX_I16) {
            BSP_LOG("REPLAY out-of-range n=%u at burst %zu, stop", n, loaded);
            break;
        }
        if (loaded >= cap) {
            cap *= 2;
            ReplayBurst *grown = realloc(replay_bursts,
                                         cap * sizeof(ReplayBurst));
            if (!grown) {
                BSP_LOG("REPLAY OOM at %zu bursts, stop", loaded);
                break;
            }
            replay_bursts = grown;
        }
        ReplayBurst *r = &replay_bursts[loaded];
        r->fn = fn;
        r->tn = tn;
        r->n  = n;
        if (fread(r->iq, sizeof(int16_t), n, f) != (size_t)n) {
            BSP_LOG("REPLAY truncated at burst %zu, stop", loaded);
            break;
        }
        loaded++;
    }
    fclose(f);
    return loaded;
}

void calypso_bsp_init(C54xState *dsp)
{
    bsp.dsp = dsp;
    calypso_manifest_once();   /* dump active overrides (CALYPSO_INVARIANTS gate, default off) */
    /* DARAM target the BSP DMAs DL samples into. Default 0x2a00, identified
     * by canary injection:
     *   1. static PROM0 scan: 0x2a00 is the top STM #imm,ARx init (50 sites,
     *      AR1..AR6; companion BK=0x015e=350 = GSM burst size)
     *   2. runtime canary (CALYPSO_BSP_INJECT_CANARY=1): the DSP READS 0xCAFE
     *      at 0x2a00..0x2a13 from PC=0x93a5, the real consumer routine, with
     *      AR3 post-incrementing. End-to-end proof.
     * Override via env if needed. */
    bsp.daram_addr     = parse_uint_env("CALYPSO_BSP_DARAM_ADDR", 0x2a00);
    bsp.daram_len      = parse_uint_env("CALYPSO_BSP_DARAM_LEN",  296);
    bsp.bursts_seen = 0;
    bsp.bursts_written = 0;
    bsp.bursts_dropped_no_window = 0;
    /* HACK — CALYPSO_BSP_BYPASS_BDLENA=1 bypasses the IOTA BDLENA window. On
     * silicon the BSP only delivers samples to the DSP during the BDLENA window
     * IOTA asserts; in emulation this flag delivers EVERY burst, which is how
     * the DARAM address the correlator really reads gets probed. Default OFF.
     * Removal criterion: DARAM target identified and a_pm / a_sync_demod
     * published nonzero by the DSP. */
    bsp.bypass_bdlena = (uint8_t)parse_uint_env("CALYPSO_BSP_BYPASS_BDLENA", 0);
    if (bsp.bypass_bdlena) {
        BSP_LOG("HACK: CALYPSO_BSP_BYPASS_BDLENA=1 — IOTA BDLENA gate DISABLED");
    }
    /* Canary injection (debug): with CALYPSO_BSP_INJECT_CANARY=1 the BSP
     * overwrites every sample with the 0xCAFE marker before the DARAM write.
     * Combined with the data_read_locked canary watch in calypso_c54x.c, this
     * pinpoints WHERE the DSP reads the BSP buffer at runtime. Disable in
     * normal runs. */
    bsp.inject_canary = (uint8_t)parse_uint_env("CALYPSO_BSP_INJECT_CANARY", 0);
    if (bsp.inject_canary) {
        BSP_LOG("HACK: CALYPSO_BSP_INJECT_CANARY=1 — samples overwritten with 0xCAFE for buffer discovery");
    }
    bsp.bursts_dropped_queue_full = 0;
    bsp.bursts_dropped_stale = 0;
    memset(bsp.q, 0, sizeof(bsp.q));
    bsp.trxd_fd = -1;
    /* Pre-set UL peer to bridge default (TRXDv0 listener on 127.0.0.1:5702).
     * Eliminates the race where ARM/DSP fires the first UL burst before any
     * DL has arrived to learn the peer addr. The peer is refined to the
     * actual sender on first DL receive (bsp_trxd_readable). */
    memset(&bsp.trxd_peer, 0, sizeof(bsp.trxd_peer));
    bsp.trxd_peer.sin_family = AF_INET;
    bsp.trxd_peer.sin_port   = htons(5702);
    bsp.trxd_peer.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    bsp.trxd_peer_valid = true;

    /* Deterministic-replay short-circuit. If CALYPSO_BSP_REPLAY_FILE is
     * set, load it now and skip the UDP listener. Replay timer takes over
     * the supply role. */
    const char *replay_path = calypso_getenv("CALYPSO_BSP_REPLAY_FILE");
    if (replay_path && *replay_path) {
        replay_count = bsp_replay_load(replay_path);
        BSP_LOG("REPLAY mode: loaded %zu bursts from %s (UDP socket bypassed)",
                replay_count, replay_path);
        if (replay_count > 0) {
            replay_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, bsp_replay_cb,
                                        NULL);
            timer_mod(replay_timer,
                      qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL)
                      + BSP_REPLAY_PERIOD_NS);
        }
        goto skip_udp_listener;
    }

    /* Bind UDP socket for TRXDv0 DL bursts from bridge/BTS.
     *
     * Default bind = 0.0.0.0 (was 127.0.0.1 hard-coded) so external
     * sources can inject bursts — bridge in the same netns still works,
     * and the host or other containers can reach BSP via the container
     * IP or via Docker port mapping (-p 6702:6702/udp).
     *
     * Override via env :
     *   CALYPSO_BSP_BIND_ADDR=<ip>  bind explicit IPv4 (e.g. 127.0.0.1,
     *                                172.20.0.11). Default: 0.0.0.0
     *   CALYPSO_BSP_BIND_LOOPBACK=1 legacy alias = 127.0.0.1 */
    int fd = socket(AF_INET, SOCK_DGRAM, 0);
    if (fd >= 0) {
        int one = 1;
        setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, &one, sizeof(one));

        const char *bind_addr_env = calypso_getenv("CALYPSO_BSP_BIND_ADDR");
        const char *bind_lo_env   = calypso_getenv("CALYPSO_BSP_BIND_LOOPBACK");
        const char *bind_addr     = NULL;
        if (bind_addr_env && *bind_addr_env)
            bind_addr = bind_addr_env;
        else if (bind_lo_env && *bind_lo_env == '1')
            bind_addr = "127.0.0.1";
        else
            bind_addr = "0.0.0.0";

        /* Port override: lets a Python proxy (iq_proxy.py) sit between the
         * source and QEMU. The source keeps sending to 6702 while QEMU listens
         * on CALYPSO_BSP_PORT and the proxy applies e.g. a Doppler shift. */
        const char *port_env = calypso_getenv("CALYPSO_BSP_PORT");
        int bsp_port = BSP_TRXD_PORT;
        if (port_env && *port_env) {
            int p = atoi(port_env);
            if (p > 0 && p < 65536) bsp_port = p;
        }
        struct sockaddr_in sa = {
            .sin_family = AF_INET,
            .sin_port = htons(bsp_port),
        };
        if (inet_aton(bind_addr, &sa.sin_addr) == 0) {
            BSP_LOG("CALYPSO_BSP_BIND_ADDR=%s invalid, falling back to 0.0.0.0",
                    bind_addr);
            sa.sin_addr.s_addr = htonl(INADDR_ANY);
            bind_addr = "0.0.0.0";
        }
        if (bind(fd, (struct sockaddr *)&sa, sizeof(sa)) == 0) {
            fcntl(fd, F_SETFL, fcntl(fd, F_GETFL) | O_NONBLOCK);
            qemu_set_fd_handler(fd, bsp_trxd_readable, NULL, NULL);
            bsp.trxd_fd = fd;
            BSP_LOG("TRXD UDP listening on %s:%d", bind_addr, bsp_port);
        } else {
            BSP_LOG("TRXD bind %s:%d failed: %s",
                    bind_addr, bsp_port, strerror(errno));
            close(fd);
        }
    }

skip_udp_listener:
    /* Pre-init env-gated state so the first RACH burst does not pay for
     * getenv/strtoul mid-run. */
    (void)d_rach_word_offset();
    (void)rach_force_bsic();

    /* Arm the REALTIME drain timer: wall-paced 5 ms, monotonic anti-drift in
     * bsp_drain_cb, on the same CLOCK_MONOTONIC as the clk_master pthread
     * (calypso_trx.c). On VIRTUAL the drain lagged under DSP load and 95 % of
     * bursts were dropped. */
    bsp.drain_timer = timer_new_ns(QEMU_CLOCK_REALTIME, bsp_drain_cb, NULL);
    timer_mod(bsp.drain_timer,
              qemu_clock_get_ns(QEMU_CLOCK_REALTIME) + BSP_DRAIN_PERIOD_NS);
    BSP_LOG("BSP drain timer armed: %dms REALTIME wall-paced, monotonic",
            BSP_DRAIN_PERIOD_MS);

    BSP_LOG("init dsp=%p daram_addr=0x%04x len=%u%s%s",
            (void *)dsp, bsp.daram_addr, bsp.daram_len,
            bsp.daram_addr ? "" : "  (DISCOVERY mode — no DMA)",
            "");
}

/* ---- DL burst → DSP DARAM ---- */

/* TS1..TS7 filler (see calypso_bsp_set_remplissage): a dummy burst padded to a
 * 157-symbol timeslot with guard silence; zeros until the bench registers one. */
/* g_remplissage and bsp_source_toutes_ts: defined before bsp_livrer_trame() */
void calypso_bsp_set_remplissage(const int16_t *iq, int n_int16)
{
    memset(g_remplissage, 0, sizeof g_remplissage);
    if (!iq || n_int16 <= 0) return;
    if (n_int16 > 2 * 157) n_int16 = 2 * 157;
    for (int i = 0; i < n_int16; i++) g_remplissage[i] = (uint16_t)iq[i];
}

/* [2026-09-23] ENREGISTREUR DU TCH, pour rejouer hors banc EXACTEMENT ce que
 * le DSP a recu (tools/rejeu_banc.c de c54x_exe). Chaque livraison d I/Q
 * pendant un TCH : 'B', tick BE32, tn, fn BE32, one_shot, nwin BE16,
 * n_int16 BE16, puis n_int16 echantillons int16 natifs. Les ecritures de
 * l ARM dans l API RAM sont enregistrees a cote par pont.c ('A').
 * 60000 livraisons au plus, des l'armement du canal dedie (SDCCH compris) ;
 * CALYPSO_REJEU_ENREG=0 coupe. */
uint32_t calypso_trx_get_fn(void);
static FILE *g_enreg_f;
static unsigned long g_enreg_n;
FILE *calypso_bsp_enreg_fichier(void)
{
    static int on = -1;
    if (on < 0) { const char *e = calypso_getenv("CALYPSO_REJEU_ENREG"); on = !(e && *e == '0'); }
    /* tout canal dedie (SDCCH compris) : ce qui corrompt l'etat du DSP peut preceder le TCH */
    if (!on || g_dedie_tn <= 0 || g_enreg_n >= 60000)
        return NULL;
    if (!g_enreg_f)
        g_enreg_f = fopen("/dev/shm/calypso_rejeu_tch.bin", "wb");
    return g_enreg_f;
}
static void bsp_enreg_burst(uint8_t tn, uint32_t fn, const int16_t *iq, int n)
{
    FILE *f = calypso_bsp_enreg_fichier();
    if (!f) return;
    uint32_t t = calypso_trx_get_fn();
    int nwin = calypso_rhea_dma_one_shot() ? calypso_rhea_dma_get_len_words() : 0;
    uint8_t h[15] = { 'B', t >> 24, t >> 16, t >> 8, t, tn, fn >> 24, fn >> 16, fn >> 8, fn,
                      (uint8_t)calypso_rhea_dma_one_shot(), (uint8_t)(nwin >> 8), (uint8_t)nwin,
                      (uint8_t)(n >> 8), (uint8_t)n };
    fwrite(h, 1, sizeof h, f);
    fwrite(iq, sizeof(int16_t), (size_t)n, f);
    fflush(f);
    g_enreg_n++;
}

void calypso_bsp_rx_burst(uint8_t tn, uint32_t fn,
                          const int16_t *iq, int n_int16)
{
    bsp.bursts_seen++;
    bsp_enreg_burst(tn, fn, iq, n_int16);
    /* [2026-09-19] Publish the frame of the burst being handed over. The global
     * existed (c54x_internal.h) but nothing ever wrote it, so every consumer
     * read 0 — including the rhea-dma transfer trace, which could not be lined
     * up with the firmware's "=>FB @ FNR X" for want of a frame number. */
    calypso_daram_last_fn = (unsigned)fn;
    g_depot_fn = (unsigned)fn; g_depot_seq++;
    /* Liveness marker for the rx_burst path (CALYPSO_DEBUG=BSP-RXBURST). */
    {
        static unsigned rxb_n;
        if (calypso_debug_enabled("BSP-RXBURST") &&
            (rxb_n <= 20 || rxb_n % 2000 == 0))
            fprintf(stderr, "[BSP] BSP-RXBURST #%u fn=%u tn=%u n=%d\n",
                    rxb_n, (unsigned)fn, (unsigned)tn, n_int16);
        rxb_n++;
    }

    if (!bsp.dsp) {
        if (bsp.bursts_seen <= 3)
            BSP_LOG("rx_burst: no DSP attached, dropping fn=%u tn=%u", fn, tn);
        return;
    }
    if (n_int16 <= 0 || iq == NULL) return;

    /* [2026-09-20] FULL TIMESLOT. A bare 148-symbol burst (296 words) is padded
     * with the 8.25 guard symbols of its timeslot (156 symbols, 157 on
     * timeslots 3 and 7 so a frame is exactly 1250) whenever DMA2 runs in
     * continuous mode, i.e. during the FB search. There the RIF is a continuous
     * stream over the WHOLE frame (the firmware divides the ROM's ToA by
     * BITS_PER_TDMA = 1250, tpu.h) and the ROM counts what it consumes. The
     * one-shot SB window keeps the block it is given (190 samples with the
     * margins). CALYPSO_BSP_TRAME_PLEINE=0 disables. */
    static int pleine = -1;
    if (pleine < 0) { const char *e = calypso_getenv("CALYPSO_BSP_TRAME_PLEINE"); pleine = (e && *e == '0') ? 0 : 1; }
    const bool continu = !calypso_rhea_dma_one_shot();
    {
        static int16_t plein[2 * 157];
        if (pleine && n_int16 <= 296 && continu) {
            int cible = 2 * (156 + ((tn == 3 || tn == 7) ? 1 : 0));
            memcpy(plein, iq, (size_t)n_int16 * sizeof(int16_t));
            memset(plein + n_int16, 0, (size_t)(cible - n_int16) * sizeof(int16_t));
            iq = plein; n_int16 = cible;
        }
    }

    /* FOLLOW THE DMA AAD (CALYPSO_BSP_AAD_FOLLOW, default 1). bsp.daram_addr is
     * an env constant (0x2a00) that was never programmed from the address the
     * ROM hands its DMA, so the burst landed somewhere other than where the
     * running task reads it (0x0cce for both FB and SB). We now deposit at the
     * address actually programmed, whenever it is known. */
    {
        static int follow = -1;
        if (follow < 0) { const char *e = calypso_getenv("CALYPSO_BSP_AAD_FOLLOW"); follow = (e && *e=='0') ? 0 : 1; }
        if (follow) {
            uint16_t aad = calypso_rhea_dma_get_daram();
            if (aad && aad != bsp.daram_addr) {
                static uint16_t prev; static unsigned nl;
                if (aad != prev && nl < 8) { BSP_LOG("AAD_FOLLOW : depot du burst en 0x%04x (etait 0x%04x)", aad, bsp.daram_addr); prev = aad; nl++; }
                bsp.daram_addr = aad;
            }
            /* AND THE LENGTH. bsp.daram_len was hardcoded to 296, which
             * TRUNCATED the SB window: that window is 380 words (190 complex),
             * so the second 39-bit data block of the SCH was never transferred.
             * Measured: the DSP buffer matched the delivered samples over
             * exactly 296 words, then diverged. We now follow the page length
             * the DSP programs itself (ALGTH). CALYPSO_BSP_LEN_FOLLOW=0
             * restores the fixed cap. */
            static int lfollow = -1;
            if (lfollow < 0) { const char *e = calypso_getenv("CALYPSO_BSP_LEN_FOLLOW"); lfollow = (e && *e=='0') ? 0 : 1; }
            if (lfollow) {
                uint16_t lw = calypso_rhea_dma_get_len_words();
                if (lw && lw != bsp.daram_len && lw <= 2048) {
                    static uint16_t prevl; static unsigned nll;
                    if (lw != prevl && nll < 8) {
                        BSP_LOG("LEN_FOLLOW : fenetre de %u mots (etait %u) — "
                                "296 tronquait la SB qui en demande 380", lw, bsp.daram_len);
                        prevl = lw; nll++;
                    }
                    bsp.daram_len = lw;
                }
            }
        }
    }
    /* [2026-09-19] FOLLOW THE PAGE (CALYPSO_BSP_PAGE_FOLLOW, default 0 — an
     * experiment, not a behaviour).
     *
     * Measured with CALYPSO_BSP_VERIF over 4000 bursts: the address the burst
     * is deposited at is statistically INDEPENDENT of the page the DSP task is
     * about to read. Crossing each comparison with d_dsp_page gave
     *
     *     w_page=0 -> 0x0cce 2340x, 0x0e4e 272x, 0x2a00 222x
     *     w_page=1 -> 0x0cce 1024x, 0x0e4e 142x
     *
     * the same ~89% share to 0x0cce either way, while the DSP does alternate
     * its pages properly (d_dsp_page cycles 0x0002/0x0003, never 0x0001). The
     * burst was found intact in DARAM in 0.4% of cases, and not at all in 46%.
     * On silicon the API buffer ping-pongs with that bit; here it does not, so
     * the task reads a page the samples never reached.
     *
     * This deposits at base + w_page*stride while the GSM task is active. The
     * base is the LOWEST address the AAD has shown (0x0cce on this bench, the
     * page-0 buffer) and the stride defaults to the observed 0x180 = 384 words
     * separating the two buffers. CALYPSO_BSP_PAGE_STRIDE overrides it and
     * accepts a NEGATIVE value, which tests the inverted page mapping. */
    {
        static int pfollow = -1;
        static long stride;
        static uint16_t base0;
        if (pfollow < 0) {
            const char *e = calypso_getenv("CALYPSO_BSP_PAGE_FOLLOW");
            pfollow = (e && *e && *e != '0') ? 1 : 0;
            const char *st = calypso_getenv("CALYPSO_BSP_PAGE_STRIDE");
            stride = (st && *st) ? strtol(st, NULL, 0) : 0x180;
        }
        if (pfollow && bsp.dsp && bsp.dsp->api_ram && bsp.daram_addr) {
            uint16_t pg = bsp.dsp->api_ram[0x08D4 - 0x0800];
            if (!base0 || bsp.daram_addr < base0) base0 = bsp.daram_addr;
            if (pg & 2) {                       /* B_GSM_TASK: a task is armed */
                uint16_t cible = (uint16_t)(base0 + ((pg & 1) ? stride : 0));
                if (cible != bsp.daram_addr) {
                    static unsigned nl;
                    if (nl < 8) {
                        BSP_LOG("PAGE_FOLLOW : w_page=%d -> depot en 0x%04x "
                                "(AAD disait 0x%04x, base 0x%04x, pas %+ld)",
                                (int)(pg & 1), cible, bsp.daram_addr, base0, stride);
                        nl++;
                    }
                    bsp.daram_addr = cible;
                }
            }
        }
    }

    if (bsp.daram_addr == 0) {
        if (bsp.bursts_seen <= 5) {
            BSP_LOG("rx_burst fn=%u tn=%u n=%d (target unset)",
                    fn, tn, n_int16);
        }
        return;
    }

    /* The BDLENA gate is gone: the burst is delivered unconditionally. NO
     * d_dsp_page write happens here — the firmware drives the page flip through
     * dsp_end_scenario (MMIO write at 0x01A8). We only signal sample arrival to
     * the DSP; d_dsp_page is observed read-only below.
     *
     * FCCH probe (CALYPSO_IQDUMP_FCCH=1): coherence and dphi of the DECIMATED
     * burst written to DARAM. A real decimated FCCH gives dphi ~ +1.571 (pi/2)
     * and coh ~ 1, which checks the content layer of the feed. */
    if (calypso_getenv("CALYPSO_IQDUMP_FCCH")) {
        int ns = n_int16 / 2;
        double accr = 0, acci = 0, den = 0;
        for (int k = 1; k < ns; k++) {
            double i0 = iq[2*(k-1)], q0 = iq[2*(k-1)+1];
            double i1 = iq[2*k],     q1 = iq[2*k+1];
            accr += i1*i0 + q1*q0;
            acci += q1*i0 - i1*q0;
            den  += sqrt((i0*i0+q0*q0)*(i1*i1+q1*q1));
        }
        double mag = sqrt(accr*accr + acci*acci);
        double coh = den > 0 ? mag/den : 0;
        double dphi = atan2(acci, accr);
        if (coh > 0.85) {
            static int fcp = 0;
            if (fcp++ < 30)
                fprintf(stderr, "[BSP] FCCH-PROBE fn=%u ns=%d coh=%.3f dphi=%.3f (vise +1.571 @1SPS)\n",
                        (unsigned)fn, ns, coh, dphi);
        }
    }

    if (bsp.dsp && bsp.dsp->api_ram) {
        static uint32_t obs_n = 0;
        /* 0x08E2 is d_dsp_state; d_dsp_page is 0x08D4 (calypso_fbsb.h). */
        uint16_t cur = bsp.dsp->api_ram[0x08D4 - 0x0800];
        obs_n++;
        if (calypso_debug_enabled("PUMP") &&
            (obs_n <= 20 || obs_n % 37 == 0)) {
            fprintf(stderr, "[bsp-page] #%u rx_burst fn=%u tn=%u "
                    "d_dsp_page=0x%04x (B_GSM_TASK=%d w_page=%d)\n",
                    obs_n, fn, tn, cur, !!(cur & 2), !!(cur & 1));
            fflush(stderr);
        }
    }
    /* This wire announces an RX BURST: end of RIF-RX DMA transfer, hence
     * vec30/bit14 (INT10n). The IFR test skips the assert while the DSP has not
     * served the previous one, so interrupts do not stack when the DSP is
     * slower than the BSP delivery rate. */
    if (bsp.dsp && bsp.dsp->running &&
        !(bsp.dsp->ifr & (1 << C54X_IT_DMA_BIT))) {
        calypso_bsp_deliver(bsp.dsp, C54X_IT_DMA_VEC, C54X_IT_DMA_BIT);
        if (bsp.dsp->idle) bsp.dsp->idle = false;
    }

    /* @BEQUILLE — BSP_DIRECT_BRINT0  (CALYPSO_BSP_DIRECT_BRINT0, EXISTS, default OFF ; calypso_wire.env:=1)
     *   masque  : on silicon, BSP DMA completion (the BDLENA window) raises BRINT0
     *             on vec21/bit5 — the "buffer received" interrupt that wakes the
     *             FB-det correlator handler (ISR PROM1[0xFFD4] -> CALL 0xf310).
     *             The direct-feed path writes DARAM but never raises it, because
     *             the TPU->TSP->IOTA->BSP chain that produces the pulse is not
     *             wired; without it the correlator at 0x8d00 is NEVER dispatched
     *             (0 hits measured). We raise it here, with the same IFR bit5
     *             anti-stacking test as deliver_buffered.
     *   retirer : once calypso_iota_take_bdl_pulse() is fed by the TPU RX window
     *             and consumed on the live path (see TPU_RX_WIRE).
     */
    { static int _db = -1; if (_db < 0) _db = calypso_gate("CALYPSO_BSP_DIRECT_BRINT0", 0);
      /* MISSION-GATE: raise BRINT0 only while the DSP is actually on the FB/SB
       * mission (d_task_md), so the correlator wakeup lands at the right moment,
       * like the real "buffer received" on silicon.
       * FB=5 SB=6 TCH_FB=8 TCH_SB=9 (osmocom l1_environment.h). */
      uint16_t _md = c54x_task_md(bsp.dsp);
      int _fbsb = (_md == 5 || _md == 6 || _md == 8 || _md == 9);
      if (_db && _fbsb && bsp.dsp && bsp.dsp->running && !(bsp.dsp->ifr & (1 << 5))) {
        calypso_bsp_deliver(bsp.dsp, 21, 5);
        if (bsp.dsp->idle) bsp.dsp->idle = false;
      } }

    /* RX-FBFLAGS on the LIVE path (rx_burst). The dispatcher at
     * 0x8d00 -> 0xa076 is PURE POLLING (BITF/BC on RAM), so no interrupt or INTM
     * change is needed; it only needs the handshake bits.
     *
     * @BEQUILLE — RX_FBFLAGS (live rx_burst path)  (CALYPSO_RX_FBFLAGS, EXISTS, default OFF)
     *   masque  : the BRINT0 ISR (PROM1[0xFFD4] -> CALL 0xf310) is never taken
     *             (INTM=1), so the FB-det handshake bits it should set stay clear:
     *             data[0x3fad] bit15 (the kernel master gate, CC 0xa0a0 @0x8754 ->
     *             kernel 0xa076), 0x3faa bit2+bit8, 0x3fab bit8, 0x3fae bit8. We
     *             set them from burst delivery instead, gated on the FB/SB mission.
     *   retirer : once BRINT0 is really served and its ISR writes these bits; the
     *             twin block in deliver_buffered is dead code and omits 0x3fad,
     *             so remove that one first.
     *   NB      : encloses POKE_TASK_MD and POKE_DISPATCH below.
     */
    { static int _fbf = -1; if (_fbf < 0) _fbf = calypso_gate("CALYPSO_RX_FBFLAGS", 0);
      uint16_t _mdf = c54x_task_md(bsp.dsp);
      int _fbsbf = (_mdf == 5 || _mdf == 6 || _mdf == 8 || _mdf == 9);
      if (_fbf && _fbsbf && bsp.dsp) {
          { static unsigned _n; if (_n++ == 0) fprintf(stderr, "[bequille] RX_FBFLAGS ACTIF\n"); }
          bsp.dsp->data[0x3fad] |= 0x8000;   /* master kernel gate  @0x8754 */
          calypso_rxfb_fired = 1;   /* arms the 0x8753 probe on the c54x side */
          bsp.dsp->data[0x3faa] |= 0x0104;   /* bit2+bit8           @0x886b/85/98 */
          bsp.dsp->data[0x3fab] |= 0x0100;   /* bit8 (FBEN)         @0x888d */
          bsp.dsp->data[0x3fae] |= 0x0100;   /* bit8                @0x90c8/ed/28 */
          /* @BEQUILLE — POKE_TASK_MD (+ POKE_DISPATCH below)  (CALYPSO_POKE_TASK_MD,
           *              atoi>0, but DEFAULT 1 WHEN THE VARIABLE IS ABSENT)
           *   masque  : the task descriptor (d_task_md, API-RAM pages 0x0804/0x0818)
           *             is never published to the DSP, so the CALA enters the
           *             correlator with task_md=0, i.e. NO mission, and spins on
           *             nothing. We write the FB/SB mission by hand into the two
           *             API-RAM pages the DSP dispatcher reads; d_dsp_page already
           *             has bit1 (task-ready) set, only task_md was missing.
           *             POKE_DISPATCH goes further and replicates dsp_end_scenario.
           *   retirer : once the write-page DMA reaches the DSP (task_md read from
           *             0x0586+DB_W_D_TASK_MD); both pokes then become no-ops.
           *   NB      : default ON, but nested inside the RX_FBFLAGS block, so it is
           *             never reached without CALYPSO_RX_FBFLAGS.
           */
          { static int _pt = -1; if (_pt < 0) { const char *_pe = calypso_getenv("CALYPSO_POKE_TASK_MD"); _pt = _pe ? (atoi(_pe) > 0) : 1; }  /* default ON, =0 to disable */
            if (_pt) { static unsigned _n;
                       if (_n++ == 0) fprintf(stderr, "[bequille] POKE_TASK_MD ACTIF (ecrit d_task_md=%u pages 0/1)\n", _mdf);
                       bsp.dsp->data[0x0804] = _mdf;   /* task_md page0 = mission (5=FB 6=SB) */
                       bsp.dsp->data[0x0818] = _mdf; } /* task_md page1 */ }
          /* POKE_DISPATCH replicates osmocom dsp_end_scenario (dsp.c:480):
           * d_task_md on the current WRITE page plus d_dsp_page =
           * B_GSM_TASK(0x0002)|w_page, ALTERNATING 2<->3 (native stays at 2, so
           * w_page never flips). */
          { static int _pd = -1; static uint16_t _wp = 0;
            if (_pd < 0) { const char *_de = calypso_getenv("CALYPSO_POKE_DISPATCH"); _pd = _de ? (atoi(_de) > 0) : 0; }
            if (_pd) {
                bsp.dsp->data[_wp ? 0x0818 : 0x0804] = _mdf;      /* d_task_md on the write page */
                /* d_dsp_page is 0x08D4, not 0x08E2 (that is d_dsp_state), and the
                 * ROM reads the API RAM, not data[]: calypso_c54x.c serves the
                 * 0x0800+ range from api_ram. */
                if (bsp.dsp->api_ram)
                    bsp.dsp->api_ram[0x08D4 - 0x0800] = (uint16_t)(0x0002 | _wp);
                else
                    bsp.dsp->data[0x08D4] = (uint16_t)(0x0002 | _wp);
                _wp ^= 1;                                         /* flip w_page (2<->3) */
            } }
          static unsigned _fbfn = 0;
          if (_fbfn++ < 8)
              fprintf(stderr, "[c54x] RX-FBFLAGS(live): 3fad|=0x8000 3faa|=0x104 "
                      "3fab|=0x100 3fae|=0x100 (task_md=%u fn=%u)\n",
                      _mdf, (unsigned)fn);
      } }

    /* @BEQUILLE — BSP_DISPATCH_FB (+ _TGT, _NOIMR, _ONESHOT)  (CALYPSO_BSP_DISPATCH_FB,
     *              EXISTS, default OFF ; calypso_wire.env:=1)
     *   masque  : the native LUT at 0x8341 that installs the FB-det handler 0x8d00
     *             into the dispatch slots is never reached (0x7234 derails into the
     *             0x013b overlay through a garbage d_dsp_page), so the computed
     *             BACC/CALA always resolves to the stub 0xab38. We write slots
     *             0x43c0/0x4387/0x43d8 by hand — per burst on the FB/SB mission,
     *             not a static pin — and open IMR bit9 in the scheduler's place, so
     *             the next BACC(0xb40f)/CALA(0xb01e) lands in the correlator.
     *   retirer : once 0x7234 reaches 0x8341 and populates these slots by itself;
     *             the IMR unmask is separable (CALYPSO_BSP_DISPATCH_NOIMR=1).
     */
    { static int _di = -1; static uint16_t _tgt = 0; static int _os = -1; static int _done = 0;
      if (_di < 0) { _di = calypso_gate("CALYPSO_BSP_DISPATCH_FB", 0);
        const char *_t = calypso_getenv("CALYPSO_BSP_DISPATCH_FB_TGT");
        _tgt = (_t && *_t) ? (uint16_t)strtoul(_t, NULL, 0) : 0x8d00;
        _os = calypso_gate("CALYPSO_BSP_DISPATCH_ONESHOT", 0); } /* default target 0x8d00.
        * ONESHOT installs and raises BRINT0 only ONCE instead of re-dispatching
        * every frame; re-dispatching congests the correlator, which re-enters in
        * a loop without finishing. One pulse lets it run a full pass. */
      uint16_t _md2 = c54x_task_md(bsp.dsp);
      int _fb2 = (_md2 == 5 || _md2 == 6 || _md2 == 8 || _md2 == 9);
      if (_di && _fb2 && bsp.dsp && bsp.dsp->running && !(_os && _done)) {
        _done = 1;
        bsp.dsp->data[0x43c0] = _tgt;   /* terminal BACC slot 0xb40f (was go-live 0xa4c7) */
        bsp.dsp->data[0x4387] = _tgt;   /* idle/CALA slot 0xb01e (was stub 0xab38) */
        bsp.dsp->data[0x43d8] = _tgt;   /* reseed slot (was stub 0xab38) */
        { /* NOIMR: unmasking the IMR is separable from installing the handler.
           * The unmask preempts the FB routine 6 instructions after entry
           * (interrupt to vec21/0x00d4), so CALYPSO_BSP_DISPATCH_NOIMR=1
           * installs the handler WITHOUT touching the IMR. */
          static int _noimr = -1;
          if (_noimr < 0) _noimr = calypso_gate("CALYPSO_BSP_DISPATCH_NOIMR", 0);
          if (!_noimr) bsp.dsp->imr |= 0x0200;   /* bit9: route the frame scheduler */
        }
        static unsigned _dl = 0;
        if (_dl++ < 8)
            fprintf(stderr, "[c54x] BSP-DISPATCH-FB : install 0x%04x (LUT setup FB) "
                    "-> slots 0x43c0/0x4387/0x43d8 + IMR|=bit9 (task_md=%u fn=%u) insn=%u\n",
                    _tgt, _md2, (unsigned)fn, bsp.dsp->insn_count);
      } }

    /* [2026-09-19] Deposit the WHOLE burst. The DARAM write used to be clamped to
     * bsp.daram_len, a length read back from the DMA page register, which is 96 on
     * most frames: a 296-word burst then lost two thirds of its samples and what
     * landed in memory matched the burst at no offset at all. Measured before the
     * fix: 0/296 words identical to the injected burst right after rx_burst, on
     * every SCH frame. The only real bound is the buffer itself. The serial-port
     * path above was already fixed this way; the DARAM write was not. */
    int n = n_int16 < BSP_IQ_MAX_I16 ? n_int16 : BSP_IQ_MAX_I16;

    /* Load samples into the BSP serial port buffer (PORTR PA=0x0034). The DSP
     * reads one sample per PORTR instruction from this buffer. Deliver the
     * WHOLE burst (iq[] = interleaved I/Q, 2*nbits int16): truncating to 148
     * int16 gave the correlator only 74 complex symbols = HALF the burst, so
     * the FCCH tone could never correlate and FBSB_CONF was never emitted.
     * Bound on n_int16, the real burst size, not on n, which is clamped to
     * daram_len for the DARAM write. */
    {
        /* Same cap as the DARAM buffer (BSP_IQ_MAX_I16). This path feeds
         * c54x_bsp_load, the BSP port, which measures `BSP LOAD=0` in every
         * run and so is not the active path — but the two caps must not
         * diverge. */
        uint16_t samples[BSP_IQ_MAX_I16];
        int ns = n_int16 > BSP_IQ_MAX_I16 ? BSP_IQ_MAX_I16 : n_int16;
        for (int i = 0; i < ns; i++)
            samples[i] = (uint16_t)iq[i];
        c54x_bsp_load(bsp.dsp, samples, ns);
    }
    /* [2026-09-20] THE OTHER SEVEN TIMESLOTS. The injectors only carry TS0
     * (the cell's FCCH/SCH/BCCH bursts); on silicon the FB search receives the
     * whole frame, 1250 symbols, and its ToA counts them. When a TS0 burst
     * arrives in continuous mode, the idle remainder of the frame follows it
     * here (no signal on TS1..TS7), so the stream keeps the frame's length.
     * A caller that delivers all eight timeslots itself passes tn != 0 for the
     * others and is left alone. */
    /* [2026-09-20] A source that delivers the other timeslots itself (pont.py
     * --dsp-port forwards all eight TRXD bursts of every frame) must not get
     * the seven fillers on top: the stream then carried 15 timeslots per
     * frame, the ROM's frame counting and TOA were off by 2x and the DSP did
     * twice the DMA/ISR work per frame. Remembered from the first tn != 0
     * burst seen. */
    if (tn != 0) bsp_source_toutes_ts = true;
    if (pleine && continu && tn == 0 && n_int16 <= 2 * 157 && !bsp_source_toutes_ts) {
        for (int ts = 1; ts < 8; ts++)
            c54x_bsp_load(bsp.dsp, g_remplissage, 2 * (156 + ((ts == 3 || ts == 7) ? 1 : 0)));
    }

    /* [2026-09-19] REFERENCE PROBE. Keeps a copy of the burst rx_burst was handed,
     * together with the destination address, length and frame. Everything it needs
     * is in scope here, so no caller has to guess which burst went where. The probe
     * The comparison itself runs LATER, from the frame loop, once the DSP has
     * executed and its DMA has drained the RIF: nothing is in DARAM yet at the
     * moment the burst is handed over. Read it with calypso_bsp_verif_compare().
     * Gate CALYPSO_BSP_VERIF. */
    if (calypso_getenv("CALYPSO_BSP_VERIF")) {
        unsigned i = bsp_verif_pos;
        int nv = n_int16 < BSP_IQ_MAX_I16 ? n_int16 : BSP_IQ_MAX_I16;
        memcpy(bsp_verif_h[i].iq, iq, (size_t)nv * sizeof(int16_t));
        bsp_verif_h[i].n = nv;
        bsp_verif_h[i].addr = bsp.daram_addr;
        bsp_verif_h[i].fn = fn;
        bsp_verif_h[i].page = (bsp.dsp && bsp.dsp->api_ram)
                            ? bsp.dsp->api_ram[0x08D4 - 0x0800] : 0xffffu;
        bsp_verif_pos = (i + 1) % BSP_VERIF_HIST;
        if (bsp_verif_plein < BSP_VERIF_HIST) bsp_verif_plein++;
    }

    /* [2026-09-19] The burst is NOT written into DARAM from here any more.
     * c54x_bsp_load above hands it to the RIF receive FIFO, and the DSP's own
     * DMA2 drains the FIFO into the API page it has programmed
     * (calypso_rhea_dma_rx_request). That is the silicon path and it works:
     * measured 17500 transfers of 296 words in 4 chained pages to the address
     * the firmware itself armed.
     *
     * Writing DARAM here as well made rx_burst a SECOND writer of the same
     * buffer, and the later writer wins: the DMA deposited the SCH burst at the
     * address the SB task had armed, then the next burst to arrive — a frame
     * that is not the SCH — overwrote it before the DSP read it. Hence an SB
     * that is attempted and comes back empty. One buffer, one writer. */
    bsp.bursts_written++;

    /* I/Q dumps for offline analysis (e.g. locating the FCCH peak at
     * +67.7 kHz = 1625/24). Captures COHERENT bursts (= FCCH) rather than the
     * first 24, which are non-FCCH startup traffic; coh uses the same math as
     * FCCH-PROBE. Outputs: /tmp/iq_rx_*.bin (CALYPSO_IQDUMP, raw int16) and
     * BSP_DUMP_RX_FILE (IQ16: 12-byte header [magic|fn LE|tn|n_int16 LE|pad]
     * then int16). */
    if (calypso_getenv("CALYPSO_IQDUMP") || calypso_getenv("BSP_DUMP_RX_FILE")) {
        int nsx = n / 2;
        double ar = 0, ai = 0, dn = 0;
        for (int k = 1; k < nsx; k++) {
            double i0 = iq[2*(k-1)], q0 = iq[2*(k-1)+1];
            double i1 = iq[2*k],     q1 = iq[2*k+1];
            ar += i1*i0 + q1*q0; ai += q1*i0 - i1*q0;
            dn += sqrt((i0*i0+q0*q0)*(i1*i1+q1*q1));
        }
        double bcoh = dn > 0 ? sqrt(ar*ar+ai*ai)/dn : 0;
        if (bcoh > 0.85) {   /* coherent burst = FCCH */
            if (calypso_getenv("CALYPSO_IQDUMP")) {
                static unsigned rx_dump_n;
                if (rx_dump_n < 24) {
                    char path[80];
                    snprintf(path, sizeof(path), "/tmp/iq_rx_%03u.bin", rx_dump_n);
                    FILE *f = fopen(path, "wb");
                    if (f) { fwrite(iq, sizeof(int16_t), n, f); fclose(f); }
                    rx_dump_n++;
                }
            }
            const char *bp = calypso_getenv("BSP_DUMP_RX_FILE");
            if (bp && *bp) {
                static FILE *bf; static int binit;
                if (!binit) { bf = fopen(bp, "wb"); binit = 1; }
                if (bf) {
                    uint8_t hdr[12] = { 0x49,0x51,0x31,0x36,
                        (uint8_t)fn, (uint8_t)(fn>>8), (uint8_t)(fn>>16), (uint8_t)(fn>>24),
                        (uint8_t)tn, (uint8_t)n, (uint8_t)(n>>8), 0 };
                    fwrite(hdr, 1, 12, bf);
                    fwrite(iq, sizeof(int16_t), n, bf);
                    fflush(bf);
                }
            }
        }
    }

    /* Log DARAM content after write for FB bursts (inside lock so values
     * read are consistent with what we just wrote). */
    if (bsp.bursts_written <= 3) {
        BSP_LOG("DARAM after write [0x%04x]: %d %d %d %d %d %d %d %d",
                bsp.daram_addr,
                n>0?(int16_t)bsp.dsp->data[bsp.daram_addr]:0,
                n>1?(int16_t)bsp.dsp->data[bsp.daram_addr+1]:0,
                n>2?(int16_t)bsp.dsp->data[bsp.daram_addr+2]:0,
                n>3?(int16_t)bsp.dsp->data[bsp.daram_addr+3]:0,
                n>4?(int16_t)bsp.dsp->data[bsp.daram_addr+4]:0,
                n>5?(int16_t)bsp.dsp->data[bsp.daram_addr+5]:0,
                n>6?(int16_t)bsp.dsp->data[bsp.daram_addr+6]:0,
                n>7?(int16_t)bsp.dsp->data[bsp.daram_addr+7]:0);
    }
    calypso_pcb_daram_lock_release();
    if (bsp.bursts_written <= 5 || (bsp.bursts_written % 1000) == 0) {
        BSP_LOG("DMA fn=%u tn=%u n=%d → DARAM[0x%04x..0x%04x] total=%llu "
                "iq[0..3]=%d,%d,%d,%d",
                fn, tn, n, bsp.daram_addr,
                (unsigned)(bsp.daram_addr + n - 1),
                (unsigned long long)bsp.bursts_written,
                n>0 ? iq[0] : 0, n>1 ? iq[1] : 0,
                n>2 ? iq[2] : 0, n>3 ? iq[3] : 0);
    }

    /* Fire BRINT0. On silicon the firmware opens the RX window through a TPU
     * scenario -> TSP write -> IOTA BDLENA, and BRINT0 fires once per window;
     * here the IFR bit rate-limits it instead. */
    if (bsp.dsp && !(bsp.dsp->ifr & (1 << 5))) {
        calypso_bsp_deliver(bsp.dsp, 21, 5);
        if (bsp.dsp->idle) bsp.dsp->idle = false;
    }
}

/* ---- Deliver buffered burst when BDLENA fires ---- */
/* Called from calypso_tdma_tick (calypso_trx.c) each frame.
 * For each TN: purge stale entries, then if a queued burst matches the
 * current QEMU virtual FN and a BDLENA pulse is pending, deliver it. */
void calypso_bsp_deliver_buffered(uint32_t current_fn)
{
    if (!bsp.dsp || bsp.daram_addr == 0) return;

    for (int tn = 0; tn < BSP_NUM_TN; tn++) {
        /* Drain ALL matchable bursts per call. One burst per call dropped the
         * effective drain rate below the IPC arrival rate under BQL contention:
         * the queue filled and bursts more than 64 FN behind cur_fn were marked
         * stale (87 % drop measured). BSP_FN_MATCH_WINDOW in bsp_take_for_fn
         * bounds the catch-up, so there is no runaway. */
        int rxwin = 0;
        {
            /* @BEQUILLE — TPU_RX_WIRE (BDLENA pulse consumption)  (CALYPSO_TPU_RX_WIRE,
             *              EXISTS, default OFF)
             *   masque  : the RX-window -> BSP transfer is not wired.
             *             calypso_iota_take_bdl_pulse() has exactly one caller, this
             *             one, so DARAM 0x2a00 stayed empty and the FB correlator ran
             *             on garbage. On a pulse for this TN the wire (a) consumes the
             *             pulse, (b) sets the FB task in scheduler word d[0x3f92] bit11
             *             in place of the native ORM at 0xa539, which never runs because
             *             d[0x5a00]==0x88, and (c) delivers the NEAREST buffered burst,
             *             bypassing the FN-match window that never lands in full mode.
             *   retirer : once d[0x3f92] is set by the native ORM and the FN match
             *             succeeds without the bypass.
             */
            static int en = -1;
            if (en < 0) en = calypso_gate("CALYPSO_TPU_RX_WIRE", 0);
            if (en && calypso_iota_take_bdl_pulse((uint8_t)tn)) {
                rxwin = 1;
                if (bsp.dsp) bsp.dsp->data[0x3f92] |= 0x0800;
                static unsigned rxw_log;
                if (rxw_log++ < 12)
                    BSP_LOG("TPU-RX-WIRE tn=%u fn=%u : BDLENA pulse consumed -> "
                            "d[0x3f92]|=0x0800 (FB task queued) + deliver nearest",
                            tn, current_fn);
            }
        }
        BspBurstSlot *sl;
        while ((sl = rxwin ? bsp_take_nearest((uint8_t)tn, current_fn)
                           : bsp_take_for_fn(tn, current_fn)) != NULL) {

        /* No d_dsp_page write here; the probe below is read-only (see
         * calypso_bsp_rx_burst). */
        if (bsp.dsp && bsp.dsp->api_ram) {
            static uint32_t obs_n = 0;
            /* 0x08E2 is d_dsp_state; d_dsp_page is 0x08D4 (calypso_fbsb.h). */
        uint16_t cur = bsp.dsp->api_ram[0x08D4 - 0x0800];
            obs_n++;
            if (calypso_debug_enabled("PUMP") &&
                (obs_n <= 20 || obs_n % 37 == 0)) {
                fprintf(stderr, "[bsp-page] #%u drain fn=%u tn=%u "
                        "d_dsp_page=0x%04x (B_GSM_TASK=%d w_page=%d)\n",
                        obs_n, current_fn, tn, cur,
                        !!(cur & 2), !!(cur & 1));
                fflush(stderr);
            }
        }
        /* Anti-stacking gate: skip while the IFR bit is still set, i.e. the DSP
         * has not served the previous interrupt. */
        if (bsp.dsp && bsp.dsp->running &&
            !(bsp.dsp->ifr & (1 << C54X_IT_DMA_BIT))) {
            calypso_bsp_deliver(bsp.dsp, C54X_IT_DMA_VEC, C54X_IT_DMA_BIT);
            if (bsp.dsp->idle) bsp.dsp->idle = false;
        }

        /* Same rule as rx_burst: bound on the burst size and the buffer, never on
         * the DMA-derived page length. */
        int n = sl->n < BSP_IQ_MAX_I16 ? sl->n : BSP_IQ_MAX_I16;

        /* === SB-INPUT discriminator (phase based) ===
         * GMSK has a constant envelope, so magnitude(I,Q) is constant for FCCH
         * AND SCH. The only separating discriminant is the phase trajectory:
         *   FCCH   = pure tone -> constant dphase -> cross[k] = I[k]*Q[k-1] -
         *            Q[k]*I[k-1] keeps the same sign for every k
         *   SCH/NB = GMSK data -> dphase varies +/-90 deg per sample -> cross
         *            alternates
         * Count cross products sharing cross[0]'s sign over 10 pairs:
         *   >= 9 same sign -> TONAL_FB, <= 8 -> MODULATED.
         * nmax is kept as well, to detect SILENT. Capped at 600. */
        {
            static unsigned db_log;
            const unsigned LIMIT = 600;
            if (db_log < LIMIT) {
                const int N = 22 < n / 2 ? 22 : n / 2;  /* N pairs ⇒ 2N samples */
                int nmax = 0;
                for (int i = 0; i < 2 * N && i < n; i++) {
                    int s = (int)sl->iq[i];
                    if (s < 0) s = -s;
                    if (s > nmax) nmax = s;
                }
                int same_sign = 0;
                int cross0 = 0;
                int cross_logged[8] = {0};
                int cross_log_cnt = 0;
                int n_cross = 0;
                for (int k = 1; k < N && n_cross < 11; k++) {
                    int I  = (int)sl->iq[2*k];
                    int Q  = (int)sl->iq[2*k + 1];
                    int Ip = (int)sl->iq[2*(k-1)];
                    int Qp = (int)sl->iq[2*(k-1) + 1];
                    /* Wide type to avoid overflow: I*Q reaches 1G, the difference 2G. */
                    long cross_l = (long)I * (long)Qp - (long)Q * (long)Ip;
                    int cross = cross_l > 0 ? 1 : (cross_l < 0 ? -1 : 0);
                    if (n_cross == 0) cross0 = cross;
                    else if (cross != 0 && cross == cross0) same_sign++;
                    if (cross_log_cnt < 8) cross_logged[cross_log_cnt++] = cross;
                    n_cross++;
                }
                const char *cat;
                if (nmax < 64) cat = "SILENT";
                else if (same_sign >= 8) cat = "TONAL_FB";
                else cat = "MODULATED";
                BSP_LOG("BURST-IN fn=%u tn=%u %s nmax=%d cross0=%d same=%d/10 "
                        "signs=%d,%d,%d,%d,%d,%d,%d,%d",
                        (unsigned)sl->fn, (unsigned)tn, cat,
                        nmax, cross0, same_sign,
                        cross_logged[0], cross_logged[1], cross_logged[2],
                        cross_logged[3], cross_logged[4], cross_logged[5],
                        cross_logged[6], cross_logged[7]);
                db_log++;
                if (db_log == LIMIT)
                    BSP_LOG("BURST-IN log capped at %u", LIMIT);
            }
        }

        /* Liveness marker for the buffered delivery path
         * (CALYPSO_DEBUG=BSP-DELIVER). */
        {
            static unsigned dlv_n;
            if (calypso_debug_enabled("BSP-DELIVER") &&
                (dlv_n <= 20 || dlv_n % 2000 == 0))
                fprintf(stderr, "[BSP] BSP-DELIVER #%u fn=%u tn=%u n=%d (apply AFC)\n",
                        dlv_n, (unsigned)sl->fn, (unsigned)tn, n);
            dlv_n++;
        }
        /* apply_phase lives in c54x_bsp_load, where ALL feeds converge towards
         * the RIF. Applying it here as well would rotate twice, since this path
         * also goes through c54x_bsp_load. */

        uint16_t samples[296];
        for (int i = 0; i < n && i < 296; i++)
            samples[i] = (uint16_t)sl->iq[i];
        c54x_bsp_load(bsp.dsp, samples, n > 296 ? 296 : n);

        /* woff is LOCAL: a static offset rolled across bursts. */
        unsigned woff = 0;
        calypso_pcb_daram_lock_acquire();
        for (int i = 0; i < n; i++) {
            uint16_t a = (uint16_t)(bsp.daram_addr + woff);
            /* CALYPSO_BSP_INJECT_CANARY: overwrite with the 0xCAFE marker to
             * identify the real target buffer on the DSP side, through the
             * canary-read hook in c54x. */
            uint16_t v = bsp.inject_canary ? 0xCAFE : (uint16_t)sl->iq[i];
            bsp.dsp->data[a] = v;
            bsp_daram_wr_bucket(a);
            woff++;
            if (woff >= bsp.daram_len) woff = 0;
        }
        calypso_pcb_daram_lock_release();
        bsp.bursts_written++;

        /* I/Q dump for the deliver_buffered path: the post-AFC samples handed
         * to the correlator. Gated by CALYPSO_IQDUMP, independent counter,
         * iq_dlv prefix. */
        if (calypso_getenv("CALYPSO_IQDUMP")) {
            static unsigned dlv_dump_n;
            if (dlv_dump_n < 24) {
                char path[80];
                snprintf(path, sizeof(path), "/tmp/iq_dlv_%03u.bin", dlv_dump_n);
                FILE *f = fopen(path, "wb");
                if (f) {
                    for (int i = 0; i < n; i++) {
                        int16_t s = (int16_t)sl->iq[i];
                        fwrite(&s, sizeof(int16_t), 1, f);
                    }
                    fclose(f);
                    BSP_LOG("IQDUMP dlv #%u fn=%u tn=%u → %s (%d int16)",
                            dlv_dump_n, (unsigned)sl->fn, (unsigned)tn, path, n);
                }
                dlv_dump_n++;
            }
        }
        sl->valid = false;  /* consumed */

        /* === BRINT0 assert ===
         * Fire the BRINT0 IRQ (vec 21, IMR bit 5) after the DARAM write. On
         * silicon, BSP DMA completion raises it, waking the DSP into the ISR at
         * PROM1[0xFFD4] -> CALL 0xf310. Without it the DSP never learns a burst
         * is available and stays in its dispatcher loop polling data[0x3fab]
         * forever (59M reads observed). The chain:
         *   1. the 0xCAFE canary proves the end-to-end BSP->DSP read at 0x2a00
         *      (PC=0x93a5)
         *   2. the DSP polls data[0x3fab] bits through the dispatcher table at
         *      data[0x16b3]
         *   3. those bits are ORed by the ISR that BRINT0 triggers
         * The IFR test skips the assert while the previous BRINT0 is unserved,
         * so the IOTA pending queue cannot overflow when the DSP handles ISRs
         * more slowly than the 217 Hz burst rate. */
        if (bsp.dsp && !(bsp.dsp->ifr & (1 << 5))) {
            calypso_bsp_deliver(bsp.dsp, 21, 5);
        }

        /* RX-FBFLAGS on the buffered path. On silicon the BRINT0 ISR (0xf310)
         * ORs the FB-det handshake bits the correlator handler polls:
         *   data[0x3faa] bit2 (0x0004) + bit8 (0x0100)   @0x886b/0x8885/0x8898
         *   data[0x3fab] bit8 (0x0100)                   @0x888d
         *   data[0x3fae] bit8 (0x0100)                   @0x90c8/0x90ed/0x9128
         * The emulated ISR does not set them, so the handler loops over
         * 0x90b0-0x9130 without ever reaching the kernel. We set them at burst
         * delivery to DARAM 0x2a00, i.e. "burst ready".
         */
        {
            /* @BEQUILLE — RX_FBFLAGS (buffered path)  (CALYPSO_RX_FBFLAGS, EXISTS, default OFF)
             *   masque  : the same FB-det handshake bits the BRINT0 ISR should set, but
             *             WITHOUT data[0x3fad] bit15, so it cannot unblock the kernel.
             *   retirer : FIRST — this block is dead code: rx_burst delivers
             *             directly, so the buffered queue stays empty.
             */
            static int _fbf = -1;
            if (_fbf < 0) _fbf = calypso_gate("CALYPSO_RX_FBFLAGS", 0);
            if (_fbf && bsp.dsp) {
                bsp.dsp->data[0x3faa] |= 0x0104;   /* bit2 + bit8 */
                bsp.dsp->data[0x3fab] |= 0x0100;   /* bit8 (FBEN target) */
                bsp.dsp->data[0x3fae] |= 0x0100;   /* bit8 (confirmed gate) */
                static unsigned _fbfn = 0;
                if (_fbfn++ < 8)
                    BSP_LOG("RX-FBFLAGS: pose 0x3faa|=0x104 0x3fab|=0x100 0x3fae|=0x100 "
                            "(handshake FB-det depuis livraison burst)");
            }
        }

        /* RX I/Q tap: when BSP_DUMP_RX_FILE is set, append the raw burst
         * (n int16_t LE, interleaved I/Q) to the file. 12-byte header per burst:
         *   magic 'IQ16' (4B) | fn (4B LE) | tn (1B) | n_int16 (2B LE) | pad (1B)
         * Readable with fcch_ref.py <dump> --fmt int16 --burst N. */
        {
            static FILE *rx_dump_f = NULL;
            static int   rx_dump_init = 0;
            if (!rx_dump_init) {
                rx_dump_init = 1;
                const char *p = calypso_getenv("BSP_DUMP_RX_FILE");
                if (p && *p) {
                    rx_dump_f = fopen(p, "ab");
                    BSP_LOG("BSP_DUMP_RX_FILE='%s' fopen=%s",
                            p, rx_dump_f ? "ok" : strerror(errno));
                } else {
                    BSP_LOG("BSP_DUMP_RX_FILE not set (p=%p p[0]=%c)",
                            (void *)p, p ? p[0] : '?');
                }
            }
            if (rx_dump_f) {
                uint8_t hdr[12] = {
                    'I','Q','1','6',
                    (uint8_t)(sl->fn      ), (uint8_t)(sl->fn >>  8),
                    (uint8_t)(sl->fn >> 16), (uint8_t)(sl->fn >> 24),
                    tn,
                    (uint8_t)(n      ), (uint8_t)(n >> 8),
                    0
                };
                fwrite(hdr, 1, 12, rx_dump_f);
                fwrite(sl->iq, sizeof(int16_t), n, rx_dump_f);
                fflush(rx_dump_f);
            }
        }

        if (bsp.bursts_written <= 10 || (bsp.bursts_written % 1000) == 0) {
            BSP_LOG("DMA tn=%u fn=%u n=%d total=%llu stale=%llu qfull=%llu",
                    tn, sl->fn, n,
                    (unsigned long long)bsp.bursts_written,
                    (unsigned long long)bsp.bursts_dropped_stale,
                    (unsigned long long)bsp.bursts_dropped_queue_full);

            /* Dump first 8 words written so we can verify the I/Q
             * constellation actually landed in the DSP data memory at
             * daram_addr — independent of any ARM-side mapping. */
            calypso_pcb_daram_lock_acquire();
            BSP_LOG("DMA @0x%04x: %04x %04x %04x %04x %04x %04x %04x %04x",
                    bsp.daram_addr,
                    bsp.dsp->data[bsp.daram_addr + 0],
                    bsp.dsp->data[bsp.daram_addr + 1],
                    bsp.dsp->data[bsp.daram_addr + 2],
                    bsp.dsp->data[bsp.daram_addr + 3],
                    bsp.dsp->data[bsp.daram_addr + 4],
                    bsp.dsp->data[bsp.daram_addr + 5],
                    bsp.dsp->data[bsp.daram_addr + 6],
                    bsp.dsp->data[bsp.daram_addr + 7]);
            calypso_pcb_daram_lock_release();
        }

        /* Fire BRINT0 */
        if (bsp.dsp && !(bsp.dsp->ifr & (1 << 5))) {
            calypso_bsp_deliver(bsp.dsp, 21, 5);
            if (bsp.dsp->idle) bsp.dsp->idle = false;
        }
        }  /* end while drain */
    }
}

/* ---- UL burst → UDP to BTS ---- */

void calypso_bsp_send_ul(uint8_t tn, uint32_t fn, const uint8_t bits[148])
{
    if (bsp.trxd_fd < 0 || !bsp.trxd_peer_valid) return;

    /* TRXDv0 UL (TRX → BTS): tn(1) fn(4) rssi(1) toa(2) bits(148) = 156 bytes.
     *
     * The osmo-bts-trx TRXD protocol is *asymmetric* :
     *   - DL (BTS → TRX) : 6-byte header, 154 bytes total. No ToA.
     *   - UL (TRX → BTS) : 8-byte header WITH ToA, 156 bytes total. The
     *     ToA is needed by BTS RACH/SACCH timing-advance estimation.
     *
     * Sending 154-byte UL caused osmo-bts-trx::trx_if.c:821 to log
     *   "Rx TRXD PDU with odd burst length 146"
     * (BTS subtracts its 8-byte header from msg len, expects 148 body).
     * Always send 156 bytes for UL. */
    uint8_t pkt[8 + 148];
    pkt[0] = tn & 0x07;
    pkt[1] = (fn >> 24) & 0xff;
    pkt[2] = (fn >> 16) & 0xff;
    pkt[3] = (fn >>  8) & 0xff;
    pkt[4] =  fn        & 0xff;
    pkt[5] = 60;            /* RSSI → -60 dBm at the BTS */
    pkt[6] = 0; pkt[7] = 0; /* ToA256 = 0 (centered, no timing advance request) */
    for (int i = 0; i < 148; i++)
        pkt[8 + i] = bits[i] ? 127 : (uint8_t)(-127);

    /* Hex dump of every UL burst as it is sent, symmetric with the
     * calypso-ipc-device UL print, so L1 -> bridge -> BTS can be correlated at
     * the byte level when chasing TRXD framing or RACH parity issues. Capped at
     * 200 to keep the log finite. */
    {
        static unsigned ul_log_count = 0;
        if (ul_log_count++ < 200 || (ul_log_count % 1000) == 0) {
            BSP_LOG("UL #%u TN=%u fn=%u rssi=-60 toa=0 len=%zu "
                    "hdr=%02x%02x%02x%02x%02x%02x%02x%02x "
                    "bits[0:16]=[%+d %+d %+d %+d %+d %+d %+d %+d "
                    "%+d %+d %+d %+d %+d %+d %+d %+d]",
                    ul_log_count, tn, fn, sizeof(pkt),
                    pkt[0], pkt[1], pkt[2], pkt[3],
                    pkt[4], pkt[5], pkt[6], pkt[7],
                    (int8_t)pkt[8], (int8_t)pkt[9], (int8_t)pkt[10],
                    (int8_t)pkt[11], (int8_t)pkt[12], (int8_t)pkt[13],
                    (int8_t)pkt[14], (int8_t)pkt[15],
                    (int8_t)pkt[16], (int8_t)pkt[17], (int8_t)pkt[18],
                    (int8_t)pkt[19], (int8_t)pkt[20], (int8_t)pkt[21],
                    (int8_t)pkt[22], (int8_t)pkt[23]);
        }
    }

    sendto(bsp.trxd_fd, pkt, sizeof(pkt), 0,
           (struct sockaddr *)&bsp.trxd_peer, sizeof(bsp.trxd_peer));
}

bool calypso_bsp_tx_burst(uint8_t tn, uint32_t fn, uint8_t bits[148])
{
    if (!bsp.dsp || !bits) return false;

    /* On real Calypso, the DSP encodes the UL burst (channel coding +
     * interleaving + burst formation) and writes the 148 hard bits to a
     * DARAM buffer that the BSP TX DMA reads. The exact destination is
     * configured per task by TPU scenarios. We currently read from a
     * candidate location; if it's all-zero, the DSP encoder did not run
     * for this frame (timing miss or wrong addr) and we drop the burst. */
    bool any = false;
    calypso_pcb_daram_lock_acquire();
    for (int i = 0; i < 148; i++) {
        uint16_t w = bsp.dsp->data[0x0900 + i];
        bits[i] = (uint8_t)(w & 1);
        if (bits[i]) any = true;
    }
    calypso_pcb_daram_lock_release();

    return any;
}

/* ---- RACH access burst encoding ---- */
#include <osmocom/coding/gsm0503_coding.h>

/* d_rach lives in NDB at a struct offset that depends on the DSP version.
 * The firmware writes (uic|bsic)<<2 | (ra<<8) to ndb->d_rach right before
 * setting db_w->d_task_ra.
 *
 * Default 0x023A — confirmed empirically 2026-05-07 via D_RACH-FINDER ring
 * trace : ARM-side write at API byte 0x0474 (= DSP word 0x0A3A = word 0x23A
 * from API base) carries values 0x0300, 0x0f00, ... matching mobile L3
 * `RANDOM ACCESS ra 0xRR` log lines exactly.
 *
 * Read from the environment once and cached, so nothing is parsed mid-run. */
#define D_RACH_DEFAULT_OFFSET 0x023A
static uint32_t d_rach_word_offset(void)
{
    static uint32_t cached = 0;
    static bool     done = false;
    if (done) return cached;
    const char *e = calypso_getenv("CALYPSO_NDB_D_RACH_OFFSET");
    if (e && *e) {
        cached = (uint32_t)strtoul(e, NULL, 0);
        BSP_LOG("d_rach offset: 0x%04x (env=%s)", cached, e);
    } else {
        cached = D_RACH_DEFAULT_OFFSET;
        BSP_LOG("d_rach offset: 0x%04x (default macro — pinned 2026-05-07)", cached);
    }
    done = true;
    return cached;
}

/* CALYPSO_RACH_FORCE_BSIC=N forces the BSIC used by the RACH encoder to a
 * fixed value, overriding whatever is read from d_rach. Useful when the
 * d_rach offset is uncertain : if the BTS responds with IMM_ASS_CMD as
 * soon as we encode with the BSC's `base_station_id_code`, the chain is
 * proven and we know the only remaining bug is the d_rach offset.
 *
 * Returns -1 if unset, otherwise the forced BSIC value (0..63).
 *
 * @BEQUILLE — RACH_FORCE_BSIC  (CALYPSO_RACH_FORCE_BSIC, VALUE, default unset = inert)
 *   masque  : the uncertainty on the NDB offset of d_rach. Instead of reading the
 *             BSIC the firmware wrote, we impose the BSC's own, to prove the RACH
 *             encoding chain independently of that offset.
 *   retirer : once CALYPSO_NDB_D_RACH_OFFSET is confirmed, i.e. IMM_ASS_CMD is
 *             received with the BSIC read from d_rach and no forcing.
 */
static int rach_force_bsic(void)
{
    static int cached = -2;
    if (cached != -2) return cached;
    const char *e = calypso_getenv("CALYPSO_RACH_FORCE_BSIC");
    /* Same empty-string-as-unset handling as d_rach_word_offset(). */
    if (!e || !*e) {
        cached = -1;
        BSP_LOG("CALYPSO_RACH_FORCE_BSIC unset → BSIC read from d_rach");
        return cached;
    }
    long v = strtol(e, NULL, 0);
    if (v < 0 || v > 63) {
        BSP_LOG("CALYPSO_RACH_FORCE_BSIC=%s out of range [0..63] — ignored", e);
        cached = -1;
        return cached;
    }
    cached = (int)v;
    BSP_LOG("CALYPSO_RACH_FORCE_BSIC=%d (forcing all RACH bursts with this BSIC)", cached);
    return cached;
}

bool calypso_bsp_tx_rach_burst(uint32_t fn, uint8_t bits[148])
{
    if (!bsp.dsp || !bits) return false;

    /* Read d_rach from NDB. dsp->data[] is the DSP-side word view; the
     * API RAM at DSP word 0x0800.. is shared with the ARM-visible page
     * at 0xFFD00000. We address via dsp->data[0x0800 + offset]. */
    uint32_t off = d_rach_word_offset();
    uint16_t d_rach = calypso_dsp_daram_read(bsp.dsp, 0x0800 + off);
    if (d_rach == 0) {
        /* Pre-LU : firmware hasn't written d_rach yet. Normal during cell
         * selection / SI decode phase. Don't alarm — just skip silently
         * (cap log to first 5 to keep it visible if there's a real issue). */
        static unsigned zero_log = 0;
        if (zero_log++ < 5) {
            BSP_LOG("RACH: d_rach@0x%04x is zero — skipping #%u "
                    "(normal pre-LU, mobile not yet in RR_EST_REQ)",
                    off, zero_log);
        }
        return false;
    }

    /* prim_rach.c:73 packs as:
     *   d_rach[7:0]  = uic<<2 (or bsic<<2)
     *   d_rach[15:8] = ra (8-bit RACH info) */
    uint8_t uic_or_bsic = (uint8_t)((d_rach & 0xFF) >> 2);
    uint8_t ra          = (uint8_t)((d_rach >> 8) & 0xFF);

    /* Optional BSIC override (probes whether wrong BSIC is the only blocker). */
    int forced = rach_force_bsic();
    if (forced >= 0) {
        uic_or_bsic = (uint8_t)forced;
    }

    /* gsm0503_rach_ext_encode writes 148 unpacked bits (ubit_t=uint8_t 0/1)
     * into burst[]. is_11bit=false → use 8-bit RACH (legacy GSM). */
    int rc = gsm0503_rach_ext_encode(bits, ra, uic_or_bsic, false);
    if (rc < 0) {
        BSP_LOG("RACH encode failed rc=%d ra=0x%02x bsic=0x%02x", rc, ra, uic_or_bsic);
        return false;
    }

    static int rach_log = 0;
    if (++rach_log <= 20) {
        BSP_LOG("RACH encode #%d fn=%u ra=0x%02x bsic=0x%02x d_rach=0x%04x",
                rach_log, fn, ra, uic_or_bsic, d_rach);
    }
    return true;
}

/* Emit an UL RACH access burst from an EXPLICIT ra/bsic pair, bypassing the
 * d_rach DARAM read. Called from the write-d_rach hook in calypso_trx.c for the
 * case where the DSP d_task_ra task is swallowed and calypso_bsp_tx_rach_burst
 * never fires. One call is one real RACH attempt. */
bool calypso_bsp_send_rach_ra(uint8_t ra, uint8_t bsic, uint32_t fn, uint8_t tn)
{
    int forced = rach_force_bsic();
    if (forced >= 0) bsic = (uint8_t)forced;
    uint8_t bits[148] = {0};
    int rc = gsm0503_rach_ext_encode(bits, ra, bsic, false);
    if (rc < 0) {
        BSP_LOG("RACH-RA encode fail rc=%d ra=0x%02x bsic=0x%02x", rc, ra, bsic);
        return false;
    }
    static int lg = 0;
    if (++lg <= 20)
        BSP_LOG("RACH-RA encode #%d fn=%u ra=0x%02x bsic=0x%02x (hook d_rach)", lg, fn, ra, bsic);
    calypso_bsp_send_ul(tn, fn, bits);   /* -> 127.0.0.1:5702 -> bridge g_bsp_fd */
    return true;
}
