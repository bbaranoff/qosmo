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
#include <sys/stat.h>   /* calypso_reglage */
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

/* calypso_api.h declares it but pulls in all the QEMU state: bare prototype. */
extern uint32_t calypso_trx_get_fn(void);

/* DARAM write stamp, published for the c54x memory dump. */
unsigned calypso_daram_last_fn;
/* fn of the last burst actually handed to the BSP, to measure the gap between
 * the DEPOSIT and the EVALUATION by the DSP task (DECALAGE probe). */
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
        /* A silent burst does not measure the cell. Clears (bsp_ts0_service,
         * frame without burst) and the zero fills of empty slots also come
         * through here; letting them drive the MAV to ~0 would push the next
         * a_pm to the floor (-100 dBm, which the firmware reports as
         * "rxlev <=-110 (0)"), i.e. C1 = 0 and the cell deemed unusable
         * (LOST_COVERAGE every 5 to 7 s). The last measurement of a real
         * burst is therefore kept. CALYPSO_BSP_MAV_MIN=0 restores the
         * unconditional update. */
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

/* One measurement per window. In idle mode the TI L1 makes several measurements in the
 * same frame (d_task_md = 0x200 + n: the cell ARFCN 1 and the neighbour 277 = DCS 614); if
 * apm_jamais_nul (pont.c) filled them all with the MAV of the LAST delivered packet, the
 * neighbour window (fake-air noise) would come last and drag the cell to the floor with it.
 * The MAV of each measurement window (one-shot, < 150 samples) is therefore kept in frame
 * order; the k-th PM takes the k-th one. */
#define BSP_PM_MAV_N 8
static int faux_air_gain(uint32_t tick, int limite);   /* below, with faux_air_arfcn */
static uint32_t g_tick_courant;        /* (early definition, see bsp_ts0_service) */
static uint16_t g_pm_mav[BSP_PM_MAV_N], g_pm_mav_der[BSP_PM_MAV_N];
static int g_pm_mav_n, g_pm_mav_der_n;
static uint32_t g_pm_mav_tick;
#define BSP_PM_FIFO_N 16
static uint16_t g_pm_mav_cellule = 14000;
static unsigned g_pm_dette;
/* The PM the DSP writes. c54x_mem.c replaces every DSP write into a_pm (page R,
 * 0x834-836 / 0x848-84A) with calypso_bsp_rssi_apm(). bsp_last_mav is only fed by the UDP
 * path (bsp_iq_publish); with pont.c it would stay on noise, so the value comes from the
 * LAST delivered measurement window (the DSP writes its PM right after processing it),
 * with the gain programmed for that window. */
static uint16_t g_pm_der_mav; static int g_pm_der_gain = -999, g_pm_der_ok;                   /* predicted PMs: their windows, delivered later, do not go in the queue */     /* MAV of a cell burst (see calypso_bsp_rssi_apm_suivante) */
static int faux_air_arfcn(uint32_t tick, int limite);
static int faux_air_cellule(void);
static uint16_t g_pm_fifo[BSP_PM_FIFO_N];
static int16_t g_pm_fifo_gain[BSP_PM_FIFO_N];
static uint32_t g_pm_fifo_tick[BSP_PM_FIFO_N];  /* delivery tick: past 3 frames the entry is stale */   /* RX gain of the window (faux_air_gain), -999 unknown */
static uint32_t g_pm_fifo_w, g_pm_fifo_r;
/* Live trace: `touch /dev/shm/apm_trace` -> every frame (pont.c [apmT]) and every delivery
 * in a measurement window ([apmL]) are traced, 4000 lines at most; `rm` to stop. Shows WHEN
 * the DSP writes a_pm / d_task_md and when the MAV queue serves them. */
int calypso_apm_trace_on(void)
{
    static unsigned n_appels, n_lignes; static int on;
    if (n_appels++ % 108 == 0) on = (access("/dev/shm/apm_trace", F_OK) == 0);
    if (!on || n_lignes >= 4000) return 0;
    n_lignes++;
    return 1;
}
/* Rank of the measurement window in the frame. In idle mode the TI L1 measures two
 * frequencies per frame (IBA_I pairs (1, 277)): the 1st window uses the synthesizer
 * programming done at the END of the previous frame (qbit 4851..4983), the 2nd the one at
 * the START of the frame (qbit 323/671), e.g. frame N: 323 -> 1, 4861 -> 614. Looking up
 * "last programming before qbit 5000" would return the same for both, so the lookup is
 * bounded by the window rank. */
static int pm_rang_fenetre(void)
{
    return g_pm_mav_tick == g_tick_courant ? g_pm_mav_n : 0;
}
static int pm_limite_qbit(int rang)
{
    return rang == 0 ? -1 : 2500;    /* -1: previous frames only */
}
static void bsp_pm_mav_noter(const int16_t *iq, int n, double k_air)
{
    if (g_pm_mav_tick != g_tick_courant) {
        if (g_pm_mav_n > 0) { memcpy(g_pm_mav_der, g_pm_mav, sizeof g_pm_mav); g_pm_mav_der_n = g_pm_mav_n; }
        g_pm_mav_tick = g_tick_courant; g_pm_mav_n = 0;
    }
    if (n <= 0) return;
    int lim = pm_limite_qbit(g_pm_mav_n);      /* rank of this window in the frame (before adding) */
    uint64_t acc = 0;
    for (int i = 0; i < n; i++) { int v = iq[i]; acc += (v < 0) ? (uint32_t)(-v) : (uint32_t)v; }
    uint32_t mav = (uint32_t)(acc / (uint32_t)n * k_air);
    uint16_t m16 = mav > 0xffff ? 0xffff : (uint16_t)mav;
    if (m16 >= 5000) g_pm_mav_cellule = m16;   /* last real burst seen in a measurement window */
    g_pm_der_mav = m16; g_pm_der_gain = faux_air_gain(g_tick_courant, lim); g_pm_der_ok = 1;
    if (calypso_apm_trace_on())
        printf("  [apmL] tick=%u fenetre=%d rang=%d mav=%u arfcn=%d gain=%d file=%u\n", g_tick_courant,
               calypso_rhea_dma_get_len_words() / 2, g_pm_mav_n - 1, m16, faux_air_arfcn(g_tick_courant, lim),
               faux_air_gain(g_tick_courant, lim), g_pm_fifo_w - g_pm_fifo_r);
    if (g_pm_mav_n < BSP_PM_MAV_N) g_pm_mav[g_pm_mav_n++] = m16;
    /* Event queue. In idle mode the pump serves the measurement windows AFTER the DONE
     * (PONT_DONE_TOT): the 0 the DSP writes for this PM is only seen by apm_jamais_nul at
     * the next DONE, so a per-frame association would hand it the MAV of the windows of
     * THAT frame (neighbour noise). Each delivered window pushes its MAV; each null PM
     * found in an R page pops the oldest. */
    g_pm_fifo_gain[g_pm_fifo_w % BSP_PM_FIFO_N] = (int16_t)faux_air_gain(g_tick_courant, lim);
    g_pm_fifo_tick[g_pm_fifo_w % BSP_PM_FIFO_N] = g_tick_courant;
    g_pm_fifo[g_pm_fifo_w++ % BSP_PM_FIFO_N] = m16;
    if (g_pm_fifo_w - g_pm_fifo_r > BSP_PM_FIFO_N) g_pm_fifo_r = g_pm_fifo_w - BSP_PM_FIFO_N;
}
static uint16_t bsp_apm_de_mav(double mav, int gain_db);
static int air_niveaux_on(void);
/* AGC of the realistic air (CALYPSO_AIR_AGC=1, default): the level of a cell burst (dBm)
 * does not change its I/Q amplitude -- a real receiver brings it back to full scale through
 * its AGC, and at amplitude 3000 the DSP does not decode the BCCH. The level only affects
 * the power MEASUREMENT: air_choisir() stores the amplitude factor by (fn, TN) and
 * calypso_bsp_rx_burst() applies it to the MAV of the measurement window
 * (bsp_pm_mav_noter). */
#define AIR_PM_RING 4096u
static struct { uint32_t fn; float k; } g_air_pm[AIR_PM_RING][8];
static double air_pm_k(uint32_t fn, unsigned tn)
{
    const typeof(g_air_pm[0][0]) *e = &g_air_pm[fn % AIR_PM_RING][tn & 7];
    return (e->fn == fn && e->k > 0) ? e->k : 1.0;
}
static double g_air_k_der = 1.0;   /* factor of the last burst delivered to the DSP (calypso_bsp_rx_burst) */
static int g_air_dpm = -1;          /* model D_PM for this burst (same scale as a_pm), -1 unknown */
static int faux_air_gain(uint32_t tick, int limite);
/* D_PM (a_serv_demod[1], 1/64 dB at baseband, layer1/prim_rx_nb.c:
 * agc_inp_dbm8_by_pm(D_PM >> 3)): the DSP computes it on full-scale I/Q; the rssi app
 * derives the level of the sync view (rx_level) from it. */
uint16_t calypso_bsp_air_dpm(uint16_t val)
{
    /* The DSP value is not used: on full-scale I/Q it saturates (rxlev 63, -47 dBm shown)
     * and a plain 20 log k offset is off by more than 10 dB. The a_pm model
     * (bsp_apm_de_mav, programmed gain of the window) gives the same scale as the
     * spectrum view. */
    if (!air_niveaux_on() || g_air_dpm < 0 || val == 0) return val;
    return (uint16_t)g_air_dpm;
}
static double air_env_dbm(const char *nom, double defaut);
/* [apm] probe (pont.c): the per-window MAVs of the current and previous frame */
int calypso_bsp_pm_mav_texte(char *buf, size_t n)
{
    int o = snprintf(buf, n, "tick %u:", g_pm_mav_tick);
    for (int k = 0; k < g_pm_mav_n && o < (int)n; k++) o += snprintf(buf + o, n - o, " %u", g_pm_mav[k]);
    if (o < (int)n) o += snprintf(buf + o, n - o, " | avant:");
    for (int k = 0; k < g_pm_mav_der_n && o < (int)n; k++) o += snprintf(buf + o, n - o, " %u", g_pm_mav_der[k]);
    if (o < (int)n) o += snprintf(buf + o, n - o, " | dernier=%u courant=%u", bsp_last_mav, g_tick_courant);
    return o;
}
/* did the current frame deliver a real burst (MAV >= 5000) in a measurement window? ([apm] probe) */
int calypso_bsp_pm_mav_fort(void)
{
    if (g_pm_mav_tick != g_tick_courant) return 0;
    for (int k = 0; k < g_pm_mav_n; k++) if (g_pm_mav[k] >= 5000) return 1;
    return 0;
}
/* the next null PM (pont.c, apm_jamais_nul): the MAV of the oldest unconsumed measurement window */
uint16_t calypso_bsp_rssi_apm_suivante(int *depile_mav)
{
    while (g_pm_fifo_r != g_pm_fifo_w && (uint32_t)(g_tick_courant - g_pm_fifo_tick[g_pm_fifo_r % BSP_PM_FIFO_N]) > 3)
        g_pm_fifo_r++;                       /* window never measured: not kept for the next one */
    if (g_pm_fifo_r != g_pm_fifo_w) {
        int g = g_pm_fifo_gain[g_pm_fifo_r % BSP_PM_FIFO_N];
        uint16_t m = g_pm_fifo[g_pm_fifo_r++ % BSP_PM_FIFO_N];
        if (depile_mav) *depile_mav = m;
        if (calypso_apm_trace_on())
            printf("  [apmS] tick=%u PM servie : mav=%u gain=%d (livree au tick %u), reste %u en file\n", g_tick_courant,
                   m, g, g_pm_fifo_tick[(g_pm_fifo_r - 1) % BSP_PM_FIFO_N], g_pm_fifo_w - g_pm_fifo_r);
        return bsp_apm_de_mav(m, g);
    }
    if (depile_mav) *depile_mav = -1;
    return calypso_bsp_rssi_apm();
}
uint16_t calypso_bsp_rssi_apm_k(int k)
{
    if (g_pm_mav_tick == g_tick_courant && k < g_pm_mav_n) return bsp_apm_de_mav(g_pm_mav[k], -999);
    if (g_pm_mav_tick != g_tick_courant || g_pm_mav_n == 0) {
        if (k < g_pm_mav_der_n) return bsp_apm_de_mav(g_pm_mav_der[k], -999);
    }
    return calypso_bsp_rssi_apm();
}

uint16_t calypso_bsp_rssi_apm(void)
{
    if (g_pm_der_ok) return bsp_apm_de_mav((double)g_pm_der_mav, g_pm_der_gain);
    return bsp_apm_de_mav((double)bsp_last_mav, -999);
}

static uint16_t bsp_apm_de_mav(double mav, int gain_db)
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
        } else if (air_niveaux_on()) {
            /* Realistic air: a single scale. mav_ref is the MAV of a burst modulated at 30000,
             * which the realistic air places at CALYPSO_AIR_REF_DBM (-40); the reference level
             * must be the same here, otherwise a cell played at -60 dBm comes out at a
             * different level in the model and in the rssi app. */
            rf_ref = air_env_dbm("CALYPSO_AIR_REF_DBM", -40.0);
            /* Measured MAV of a GMSK burst modulated at 30000 (mean of |I|,|Q|: 1330..1400 at
             * amplitude 3000); 20929 would leave the cell 4 dB under its setpoint. */
            if (!(mr && *mr)) mav_ref = 13700.0;
        }
        if (mav_ref < 1.0) {
            mav_ref = 1.0;
        }
    }
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
    if (gain_db > -999) {
        /* same formula as calypso_trf6151_apm_for_rf, with the gain the L1 programmed for the window */
        int bb = rfi + gain_db, apm;
        if (bb < 0) bb = 0;
        apm = bb * 64;
        if (apm < 64) apm = 64;
        if (apm > 0xFFFF) apm = 0xFFFF;
        return (uint16_t)apm;
    }
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

/* Live settings (calypso_bsp.h). Tuning values such as CALYPSO_BSP_TCH_DECAL,
 * PONT_TCH_DEPOT_IDLE or CALYPSO_TI_SYNC_DL can be changed on a running c54x_exe: write
 * "NAME=value" into /dev/shm/calypso_reglages; the file is re-read at most every 50 calls
 * when its mtime changes. Falls back to the environment variable. */
int calypso_reglage(const char *nom, int defaut)
{
    static char texte[4096]; static struct timespec mtime; static unsigned n_appels;
    if (n_appels++ % 50 == 0) {
        struct stat st;
        if (stat("/dev/shm/calypso_reglages", &st) == 0) {
            if (st.st_mtim.tv_sec != mtime.tv_sec || st.st_mtim.tv_nsec != mtime.tv_nsec) {
                FILE *f = fopen("/dev/shm/calypso_reglages", "r");
                if (f) {
                    size_t n = fread(texte, 1, sizeof texte - 1, f); texte[n] = 0; fclose(f);
                    mtime = st.st_mtim;
                    printf("  [reglages] /dev/shm/calypso_reglages relu : %s\n", texte);
                    fflush(stdout);
                }
            }
        } else texte[0] = 0;
    }
    size_t l = strlen(nom);
    for (const char *p = texte; (p = strstr(p, nom)); p += l)
        if ((p == texte || p[-1] == '\n' || p[-1] == ' ') && p[l] == '=') return atoi(p + l + 1);
    const char *e = calypso_getenv(nom);
    return (e && *e) ? atoi(e) : defaut;
}
static int64_t g_ts0_offset = INT64_MIN;
/* Mobile FN: fn - tick of the last SCH delivered in a REAL SB window, i.e. what the TI L1
 * has just learnt; it then counts one frame per tick until the next SB. The BSP offset
 * (g_ts0_offset) is instead re-aligned by the BCCH reads and drifts (the DSP tick is slightly
 * faster than the BTS), so a RACH dated by it can land at fn-1 or fn+1 of what the mobile
 * expects and the IMM ASS is ignored. calypso_bsp_fn_mobile_du_tick() dates the RACH
 * (pont.c tx_tsp_fn). */
static int64_t g_off_mobile = INT64_MIN;
static int g_off_rac_vu;   /* the offset comes from an RA_C: delivered SBs no longer replace it */
int calypso_bsp_fn_mobile_du_tick(uint32_t tick, uint32_t *fn)
{
    if (g_off_mobile == INT64_MIN) return 0;
    int64_t w = ((int64_t)tick + g_off_mobile) % (int64_t)BSP_FN_MAX;
    if (w < 0) w += BSP_FN_MAX;
    *fn = (uint32_t)w;
    return 1;
}
/* ... and what the TI L1 WRITES in dedicated mode: a_a5fn (pont.c) gives its exact fn.
 * Dedicated mode (TCH above all) slips the tick by one frame; without this update the first
 * RACH after the call would be dated fn-1 by the offset of the SB from BEFORE the call and
 * its IMM ASS ignored. CALYPSO_BSP_OFF_A5FN=0 disables it. */
static uint32_t g_a5fn_tick; static int g_a5fn_vu;   /* last frame where a_a5fn gave the mobile fn */
void calypso_bsp_off_mobile_a5fn(uint32_t tick, uint32_t fn)
{
    /* By default a_a5fn does NOT set the offset: it oscillates by +-1 depending on which W
     * page is read at the tick, which would date the first RACH of the next access one
     * frame off. RA_C (calypso_bsp_off_mobile_rac) is authoritative; a_a5fn only keeps the
     * dedicated alignment alive. CALYPSO_BSP_OFF_A5FN=1: a_a5fn also sets the offset. */
    static int on = -1;
    if (on < 0) { const char *e = calypso_getenv("CALYPSO_BSP_OFF_A5FN"); on = (e && *e == '1'); }
    g_a5fn_tick = tick; g_a5fn_vu = 1;
    if (!on && g_off_mobile != INT64_MIN) return;
    int64_t off = (int64_t)fn - (int64_t)tick;
    if (off != g_off_mobile) {
        static unsigned n;
        if (n++ < 30)
            printf("  [off-mobile] a_a5fn fn=%u tick=%u : offset %lld -> %lld\n", fn, tick,
                   g_off_mobile == INT64_MIN ? 0LL : (long long)g_off_mobile, (long long)off);
        g_off_mobile = off;
    }
}
/* ... and the mobile's own truth: RA_C (TI L1 trace in rvinterf.log) gives the fn at which
 * IT sent its RACH. The trace only carries fn % 42432, so the multiple of 42432 of the
 * current offset is kept. The mobile offset drifts upwards over time while the BTS one (SB,
 * BCCH) drifts down, so estimates coming from the BTS would date the RACHs 1..3 frames early
 * and deliver the wrong frames in dedicated mode. */
/* Mobile FN through d_fn. The mobile fn slips against the tick during dedicated mode
 * (load) while the BSP offset stays, which breaks every BCCH/paging read (BS_I/NP_I 1). The
 * W page carries d_fn = fn_report | (fn%104) << 8: the frame the mobile believes it reads at
 * the next tick. (tick + 1 + offset) % 104 must equal it; a gap d (|d| <= 6) seen three
 * times in a row -> offset + d (and the mobile offset used to date the RACHs). Called by
 * montant.c on every page carrying a task. CALYPSO_BSP_DFN=0 disables, CALYPSO_BSP_DFN_K
 * shifts the reference. */
void calypso_bsp_dfn_observe(uint32_t tick, uint16_t dfn, uint16_t task_d)
{
    static int on = -1, k = 0, d_prec = 99, n_meme; static unsigned n_log;
    if (on < 0) { const char *e = calypso_getenv("CALYPSO_BSP_DFN"); on = !(e && *e == '0');
                  e = calypso_getenv("CALYPSO_BSP_DFN_K"); k = (e && *e) ? atoi(e) : 0; }
    if (!on || dfn == 0 || !(task_d & 0x7fff) || g_ts0_offset == INT64_MIN) return;
    unsigned f104 = (unsigned)(dfn >> 8);
    if (f104 >= 104) return;
    int64_t w = ((int64_t)tick + 1 + g_ts0_offset) % (int64_t)BSP_FN_MAX; if (w < 0) w += BSP_FN_MAX;
    int d = ((int)f104 + k - (int)(w % 104) + 2 * 104) % 104;
    if (d >= 52) d -= 104;
    if (n_log < 60 || (d && n_log % 50 == 0)) {
        n_log++;
        printf("  [dfn] tick=%u tache=%u d_fn%%104=%u livre%%104=%u ecart %+d (offset %lld)\n",
               tick, task_d & 0x7fff, f104, (unsigned)(w % 104), d, (long long)g_ts0_offset);
    } else n_log++;
    if (d == 0 || d < -6 || d > 6) { d_prec = 99; n_meme = 0; return; }
    n_meme = (d == d_prec) ? n_meme + 1 : 1; d_prec = d;
    if (n_meme < 3) return;
    g_ts0_offset = (g_ts0_offset + d + BSP_FN_MAX) % BSP_FN_MAX;
    if (g_off_mobile != INT64_MIN) g_off_mobile += d;
    printf("  [dfn] tick=%u : fn du mobile %+d trame(s) -> offset %lld\n", tick, d, (long long)g_ts0_offset);
    d_prec = 99; n_meme = 0;
}
void calypso_bsp_off_mobile_rac(uint32_t tick, uint32_t fn42432)
{
    int64_t ref = g_off_mobile != INT64_MIN ? g_off_mobile : (g_ts0_offset != INT64_MIN ? g_ts0_offset : 0);
    int64_t m = ((int64_t)fn42432 - (int64_t)(tick % 42432u)) % 42432; if (m < 0) m += 42432;
    int64_t d = (m - (ref % 42432 + 42432) % 42432) % 42432; if (d < 0) d += 42432; if (d > 21216) d -= 42432;
    int64_t off = ref + d;
    g_a5fn_tick = tick; g_a5fn_vu = 1; g_off_rac_vu = 1;
    if (off != g_off_mobile) {
        printf("  [off-mobile] RA_C fn%%42432=%u tick=%u : offset du mobile %lld -> %lld\n", fn42432, tick,
               g_off_mobile == INT64_MIN ? 0LL : (long long)g_off_mobile, (long long)off);
        g_off_mobile = off;
    }
    /* Optionally let the delivery offset follow RA_C (gap <= 6 only). Off by default:
     * RA_C - tick equals the delivery offset OR +1 depending on the access, so following it
     * can shift the delivery by one frame and break the NP_I reads.
     * CALYPSO_BSP_RAC_LIVRAISON=1 enables it. */
    static int rac_liv = -1;
    if (rac_liv < 0) { const char *e = calypso_getenv("CALYPSO_BSP_RAC_LIVRAISON"); rac_liv = (e && *e == '1'); }
    if (rac_liv && g_ts0_offset != INT64_MIN && off != g_ts0_offset) {
        int64_t e = off - g_ts0_offset;
        if (e >= -6 && e <= 6) {
            printf("  [off-mobile] offset de livraison %lld -> %lld (RA_C)\n", (long long)g_ts0_offset, (long long)off);
            g_ts0_offset = (off + BSP_FN_MAX) % BSP_FN_MAX;
        }
    }
}
/* TI firmware in dedicated mode (ALLC tasks seen by montant.c): see CALYPSO_BSP_DEDIE_DECAL */
static int g_dedie_ti;
extern unsigned long calypso_bsp_routeur_bilan(char *buf, size_t n);
static unsigned g_dedie_entrees;   /* entries into TI dedicated mode (pont.c: did a RACH succeed?) */
unsigned calypso_bsp_dedie_entrees(void) { return g_dedie_entrees; }
void calypso_bsp_set_dedie_ti(int actif)
{
    if (actif && !g_dedie_ti) g_dedie_entrees++;
    char b[160] = "";
    if (actif != g_dedie_ti) {
        if (!actif) calypso_bsp_routeur_bilan(b, sizeof b);
        printf("  [ts0] firmware TI %s le mode dedie%s\n", actif ? "entre dans" : "quitte", b);
    }
    g_dedie_ti = actif;
}
/* SB window at the end of the frame (CALYPSO_BSP_SB_FIN_TRAME=1, Magnetite / TI L1 TCS211).
 * The TI L1 opens its SB window SB_MARGIN (92 qbits) before the start of the burst: when
 * the SCH is aligned at the start of the frame, the window opens AT THE END OF THE PREVIOUS
 * FRAME (SB_I time_alignmt=4928), and the L1 infers "BTS frame N = local frame k+1"
 * (l1a_add_time_for_nb: 4928+80 -> 8, fn_offset-1). Delivering the burst of tick k at
 * tick k would arm every NB window one tick AFTER its burst. In this mode the offset is
 * set to fn - tick - 1 at the SCH delivered in an SB window, then VERIFIED on the BCCH read
 * itself (bsp_bcch_calage): the L1 arms its NB windows on the frames where it believes
 * the block is, which directly tells by how much the stream is shifted. The SCH must not
 * be delivered early: on resync the L1 arms TWO SB windows in a row and sets its clock
 * on the second. */
static int sb_fin_trame(void)
{
    static int on = -1;
    if (on < 0) { const char *e = calypso_getenv("CALYPSO_BSP_SB_FIN_TRAME"); on = (e && *e == '1'); }
    return on;
}
/* Fake air (CALYPSO_FAUX_AIR=<cell ARFCN>, Magnetite). Without it the BTS bursts are
 * delivered whatever frequency the phone is tuned to, and the TI L1 sees the cell
 * (BSIC 7) on 65, 81, 265... QEMU publishes every synthesizer programming
 * (calypso_rf_arfcn.h); here a window tuned elsewhere than the cell receives noise
 * (CALYPSO_FAUX_AIR_BRUIT, standard deviation, 200 by default) instead of the burst.
 * Unknown ARFCN (no shm, not yet programmed): burst delivered as is. */
#include "hw/arm/calypso/calypso_rf_arfcn.h"
#include <sys/mman.h>
#include <fcntl.h>
static int faux_air_cellule(void)
{
    static int a = -2;
    if (a == -2) { const char *e = calypso_getenv("CALYPSO_FAUX_AIR"); a = (e && *e) ? atoi(e) : -1;
                   if (a >= 0) printf("  [faux-air] cellule sur l'ARFCN %d : bruit sur les autres frequences\n", a); }
    return a;
}
/* ARFCN of the window of the tick: last programming done before `limite` qbits in frame tick. */
static int faux_air_arfcn(uint32_t tick, int limite)
{
    static const CalypsoRfArfcnShm *m;
    static uint32_t essai;
    if (!m) {
        if (essai++ % 216) return -1;              /* QEMU creates the file on its first write */
        int fd = open(CALYPSO_RF_ARFCN_SHM, O_RDONLY);
        if (fd < 0) return -1;
        void *p = mmap(NULL, sizeof *m, PROT_READ, MAP_SHARED, fd, 0);
        close(fd);
        if (p == MAP_FAILED) return -1;
        m = p;
    }
    if (m->magic != CALYPSO_RF_ARFCN_MAGIC) return -1;
    uint32_t w = __atomic_load_n(&m->widx, __ATOMIC_ACQUIRE);
    uint32_t n = w < CALYPSO_RF_ARFCN_N ? w : CALYPSO_RF_ARFCN_N - 4;   /* the oldest may be overwritten */
    for (uint32_t k = 1; k <= n; k++) {
        const CalypsoRfProg *e = &m->e[(w - k) % CALYPSO_RF_ARFCN_N];
        /* arfcn -1 = PWR write at the start of the frame (qbit 323/671, 0x003a: mode change, not
         * a tuning) decoded with a PLL of another band. The synthesizer is only tuned at the
         * END of the previous frame (PLL 4851, band 4861/4983): walk back to that tuning,
         * otherwise "unknown" would count as "cell" and the neighbour windows would receive
         * the cell burst. */
        if (e->arfcn < 0) continue;
        int32_t d = (int32_t)(e->fn - tick);
        if (d < 0 || (d == 0 && e->qbit <= limite)) return e->arfcn;
    }
    return -1;
}
/* RX gain programmed for the window of the tick (same lookup as faux_air_arfcn), -999 if unknown */
static int faux_air_gain(uint32_t tick, int limite)
{
    static const CalypsoRfArfcnShm *m;
    static uint32_t essai;
    if (!m) {
        if (essai++ % 216) return -999;
        int fd = open(CALYPSO_RF_ARFCN_SHM, O_RDONLY);
        if (fd < 0) return -999;
        void *p = mmap(NULL, sizeof *m, PROT_READ, MAP_SHARED, fd, 0);
        close(fd);
        if (p == MAP_FAILED) return -999;
        m = p;
    }
    if (m->magic != CALYPSO_RF_ARFCN_MAGIC) return -999;
    uint32_t w = __atomic_load_n(&m->widx, __ATOMIC_ACQUIRE);
    uint32_t n = w < CALYPSO_RF_ARFCN_N ? w : CALYPSO_RF_ARFCN_N - 4;
    int recent = -999;
    for (uint32_t k = 1; k <= n; k++) {
        const CalypsoRfProg *e = &m->e[(w - k) % CALYPSO_RF_ARFCN_N];
        /* Realistic air: the last RECEIVE tuning (not the transmit one) is the fallback. Outside
         * step-by-step mode (banc-rssi, tick 0 or ~130 frames behind QEMU) no tuning is "before"
         * the tick, and -999 would convert the PM with the RESET gain of c54x_exe
         * (calypso_trf6151_apm_for_rf). Same fallback as air_accord. */
        if (recent == -999 && !((uint16_t)e->reserve & (1u << 14))) recent = e->gain_db;
        int32_t d = (int32_t)(e->fn - tick);
        if (d < 0 || (d == 0 && e->qbit <= limite)) return e->gain_db;
    }
    return air_niveaux_on() ? recent : -999;
}
static void faux_air_bruit(int16_t *iq, int n_ech)
{
    static double sigma = -1; static uint32_t x = 0x12345678u;
    if (sigma < 0) { const char *e = calypso_getenv("CALYPSO_FAUX_AIR_BRUIT"); sigma = (e && *e) ? atof(e) : 200.0; }
    for (int k = 0; k < 2 * n_ech; k++) {
        double u = 0;
        for (int j = 0; j < 4; j++) { x = x * 1664525u + 1013904223u; u += (double)(x >> 8) / 16777216.0; }
        iq[k] = (int16_t)lrint((u - 2.0) * 1.732 * sigma);   /* sum of 4 uniforms ~ gaussian */
    }
}
/* Realistic air (CALYPSO_AIR_NIVEAUX=1, banc-rssi). Without it, every received burst is
 * played at amplitude 30000 whatever the frequency or direction the receiver is tuned to:
 * the rssi app sees the cell on any ARFCN with all eight bars (rxlev 63), including in the
 * UPLINK view where only the mobile is. Here:
 *   - QEMU publishes the mode and the uplink frequency (calypso_trf6151_rf_flags, field
 *     `reserve`);
 *   - the bridge also copies the uplink (byte 6 = 0xA5, pont/trx.py send_ul), stored
 *     separately (g_air_ul);
 *   - a window tuned to the downlink frequency of the cell receives the downlink at level
 *     CALYPSO_AIR_DL_DBM (-60) minus the BTS attenuation, a window tuned to its uplink
 *     frequency receives the mobile burst of that slot (uplink TN = position - 3) at level
 *     CALYPSO_AIR_UL_DBM (-75), everything else noise. Amplitude 30000 corresponds to
 *     CALYPSO_AIR_REF_DBM (-40): calibrate against the display.
 * Cell: CALYPSO_FAUX_AIR (otherwise every frequency is the cell). Without the QEMU flags:
 * nothing changes. */
static int air_niveaux_on(void)
{
    static int on = -1;
    if (on < 0) { const char *e = calypso_getenv("CALYPSO_AIR_NIVEAUX"); on = (e && *e == '1');
                  if (on) printf("  [air] niveaux realistes : DL/UL separes, amplitude selon le niveau\n"); }
    return on;
}
static double air_env_dbm(const char *nom, double defaut)
{
    const char *e = calypso_getenv(nom);
    return (e && *e) ? atof(e) : defaut;
}
/* tuning of the window: like faux_air_arfcn, but skipping the TRANSMIT tunings and keeping
 * the receptions on an uplink frequency. Returns 1 if known (*arfcn, *ul, *qbit), 0 otherwise. */
static int air_accord(uint32_t tick, int limite, int *arfcn, int *ul, int *qbit)
{
    static const CalypsoRfArfcnShm *m;
    static uint32_t essai;
    if (!m) {
        if (essai++ % 216) return 0;
        int fd = open(CALYPSO_RF_ARFCN_SHM, O_RDONLY);
        if (fd < 0) return 0;
        void *p = mmap(NULL, sizeof *m, PROT_READ, MAP_SHARED, fd, 0);
        close(fd);
        if (p == MAP_FAILED) return 0;
        m = p;
    }
    if (m->magic != CALYPSO_RF_ARFCN_MAGIC) return 0;
    uint32_t w = __atomic_load_n(&m->widx, __ATOMIC_ACQUIRE);
    uint32_t n = w < CALYPSO_RF_ARFCN_N ? w : CALYPSO_RF_ARFCN_N - 4;
    /* Fallback on the most recent tuning. Outside step-by-step mode (banc-rssi), the DSP runs
     * ~130 frames behind QEMU and the ring only keeps ~10 frames: no tuning is "before" the
     * tick. The synthesizer of an app like rssi stays on the same frequency for a long time,
     * so the last receive tuning is the right one. */
    int r_arfcn = -1, r_ul = 0, r_qbit = -1;
    for (uint32_t k = 1; k <= n; k++) {
        const CalypsoRfProg *e = &m->e[(w - k) % CALYPSO_RF_ARFCN_N];
        uint16_t f = (uint16_t)e->reserve;
        if (f & (1u << 14)) continue;                              /* transmit tuning */
        int a_ul = (f & (1u << 15)) && (f & (1u << 13));
        int a = a_ul ? (int)(f & 0x7ff) : e->arfcn;
        if (a < 0) continue;
        if (r_arfcn < 0) { r_arfcn = a; r_ul = a_ul; r_qbit = e->qbit; }
        int32_t d = (int32_t)(e->fn - tick);
        if (!(d < 0 || (d == 0 && e->qbit <= limite))) continue;
        *arfcn = a; *ul = a_ul; *qbit = e->qbit; return 1;
    }
    if (r_arfcn >= 0) { *arfcn = r_arfcn; *ul = r_ul; *qbit = r_qbit; return 1; }
    return 0;
}
/* the uplink copied by the bridge, per frame and per TN (BTS) */
#define AIR_UL_RING (1u << 12)
static struct { uint32_t fn; uint8_t bits[8][148]; uint8_t presents; } *g_air_ul;
static unsigned long g_air_ul_recus;
static struct { uint32_t fn; int valide; } g_air_ul_der[8];   /* last uplink fn received per TN */
static void air_ul_stocker(uint32_t fn, unsigned tn, const uint8_t *bits)
{
    if (!g_air_ul && !(g_air_ul = calloc(AIR_UL_RING, sizeof *g_air_ul))) return;
    unsigned i = fn % AIR_UL_RING;
    if (g_air_ul[i].fn != fn) { g_air_ul[i].fn = fn; g_air_ul[i].presents = 0; }
    memcpy(g_air_ul[i].bits[tn & 7], bits, 148);
    g_air_ul[i].presents |= (uint8_t)(1u << (tn & 7));
    g_air_ul_der[tn & 7].fn = fn; g_air_ul_der[tn & 7].valide = 1;
    g_air_ul_recus++;
}
/* The LAST uplink burst of the slot, if it is less than two multiframes (102 frames) away
 * from the played frame. The exact frame cannot be looked up: on the direct path the window
 * is served when the downlink arrives, which the BTS sends ahead of time, while the uplink
 * of the same fn does not exist yet. */
static const uint8_t *air_ul_bits(uint32_t fn, unsigned tn)
{
    if (!g_air_ul || !g_air_ul_der[tn & 7].valide) return NULL;
    uint32_t f = g_air_ul_der[tn & 7].fn;
    int32_t d = (int32_t)(fn - f);
    if (d < -102 || d > 102) return NULL;
    unsigned i = f % AIR_UL_RING;
    return (g_air_ul[i].fn == f && (g_air_ul[i].presents & (1u << (tn & 7)))) ? g_air_ul[i].bits[tn & 7] : NULL;
}
/* noise of the realistic air: floor CALYPSO_AIR_BRUIT_DBM (-110), on the same scale as the
 * bursts (30000 = CALYPSO_AIR_REF_DBM). The fake-air noise (standard deviation 200, ~-83 dBm
 * on this scale) would drown the mobile uplink (-75 dBm = 533). */
static void air_bruit(int16_t *iq, int n_ech)
{
    static double sigma = -1; static uint32_t x = 0x9e3779b9u;
    if (sigma < 0) sigma = 30000.0 * pow(10.0, (air_env_dbm("CALYPSO_AIR_BRUIT_DBM", -110.0)
                                               - air_env_dbm("CALYPSO_AIR_REF_DBM", -40.0)) / 20.0);
    for (int k = 0; k < 2 * n_ech; k++) {
        double u = 0;
        for (int j = 0; j < 4; j++) { x = x * 1664525u + 1013904223u; u += (double)(x >> 8) / 16777216.0; }
        iq[k] = (int16_t)lrint((u - 2.0) * 1.732 * sigma);
    }
}
/* scales samples modulated at 30000 (sb_moduler) to the amplitude chosen by the realistic air */
static void air_echelle(int16_t *iq, int n_ech, int amp)
{
    if (amp == 30000) return;
    double k = amp / 30000.0;
    for (int i = 0; i < 2 * n_ech; i++) iq[i] = (int16_t)lrint(iq[i] * k);
}
/* slot of a measurement window, from the qbit of the synthesizer tuning (~200 qbits before the window) */
static unsigned air_pos_fenetre(uint32_t tick)
{
    int a, ul, q;
    if (!air_accord(tick, 5000, &a, &ul, &q) || q < 0) return 0;
    return (unsigned)(((q + 300) / 625) & 7);
}
/* The burst to play in the window (slot `pos` of frame fn) and its amplitude (*amp, already
 * computed for the reference level); NULL = noise. Without CALYPSO_AIR_NIVEAUX, or with an
 * unknown tuning: dl as is. */
static const uint8_t *air_choisir(uint32_t tick, int limite, uint32_t fn, unsigned pos, const uint8_t *dl, int *amp)
{
    if (!air_niveaux_on()) return dl;
    int a, ul, q;
    if (!air_accord(tick, limite, &a, &ul, &q)) return dl;
    int cell = faux_air_cellule();
    static unsigned long n_dl, n_ul, n_bruit;
    const uint8_t *b = NULL;
    double dbm = 0;
    if (cell < 0 || a == cell) {
        if (!ul) { b = dl; dbm = air_env_dbm("CALYPSO_AIR_DL_DBM", -60.0) - calypso_bsp_get_last_att(); }
        else     { b = air_ul_bits(fn, (pos + 5) & 7); dbm = air_env_dbm("CALYPSO_AIR_UL_DBM", -75.0); }
    }
    static int agc = -1;
    if (agc < 0) { const char *e = calypso_getenv("CALYPSO_AIR_AGC"); agc = !(e && *e == '0'); }
    typeof(g_air_pm[0][0]) *pm = &g_air_pm[fn % AIR_PM_RING][pos & 7];
    pm->fn = fn; pm->k = 1.0f;
    if (b) {
        double r = *amp * pow(10.0, (dbm - air_env_dbm("CALYPSO_AIR_REF_DBM", -40.0)) / 20.0);
        if (r > 32000.0) r = 32000.0;
        if (r < 1.0) r = 1.0;
        if (agc) pm->k = (float)(r / *amp);    /* full-scale I/Q, the level goes to the measurement */
        else *amp = (int)r;
        if (ul) n_ul++; else n_dl++;
    } else n_bruit++;
    unsigned long t = n_dl + n_ul + n_bruit;
    if (t <= 30 || t % 5000 == 0)
        printf("  [air] tick=%u fn=%u pos=%u accord %s %d (cellule %d) -> %s amp=%d k_pm=%.3f [%lu DL, %lu UL, %lu bruit ; %lu UL recus]\n",
               tick, fn, pos, ul ? "UL" : "DL", a, cell, b ? (ul ? "burst montant" : "burst descendant") : "bruit",
               b ? *amp : 0, (double)pm->k, n_dl, n_ul, n_bruit, g_air_ul_recus);
    return b;
}
/* Background noise (CALYPSO_BSP_BRUIT_FOND=1, Magnetite). Slots without a burst are filled
 * with noise rather than zeros: the TI L1 takes its power measurements there, and 100 PMs
 * at 0 (l1_check_pm_error) make it believe the DSP is dead (l1_initialize_for_recovery,
 * DSP restarted every ~173 frames). A real receiver always sees noise. */
extern uint16_t g_remplissage_ts0[];
static const int16_t *bsp_remplissage(void)
{
    static int on = -1;
    static int16_t bruit[2 * 148];
    if (on < 0) { const char *e = calypso_getenv("CALYPSO_BSP_BRUIT_FOND"); on = (e && *e == '1'); }
    if (!on) return (const int16_t *)g_remplissage_ts0;
    faux_air_bruit(bruit, 148);
    return bruit;
}
static int g_bcch_verrou;        /* offset confirmed by the BCCH read: SCHs no longer re-align it */
static uint32_t g_bcch_vu_tick;  /* last tick where a BCCH NB window was seen armed */
/* TRX mode (CALYPSO_BSP_TRX=1, c54x_exe/run.sh OSMO_TRX=1): this instance is the radio of
 * the BTS (trx firmware), it never receives an SB; its uplink bursts come from the air
 * (pont/air.py) tagged with the GSM fn of the mobile. The fn <-> tick offset is then SET by
 * pont.c, which decodes the sch of the SB the firmware transmits (fn F at tick T):
 * calypso_bsp_trx_calage(). Uplink burst fn X is expected in the DSP frame where downlink
 * burst fn X+1 leaves (l1s_bts_cmd programs TX next_time.fn and RX "in the next frame",
 * l1s_bts_resp tags current_fn - 2): offset = F - T + delta, delta = -1 with the original
 * firmware, -2 with BTS_RX_LAG 3 (patched prim_bts.c: one frame of margin for the air)
 * (CALYPSO_BSP_TRX_DELTA). Once set, the mobile-mode re-alignments (delivered SB, second
 * chance) no longer touch the offset. */
static int g_trx_cale = 0;
/* Uplink prosthesis (TRX mode). The NB demodulation of the trx firmware extcode
 * (dsp_extcode.c) does not produce correct bits on this core (its result words d[5..33]
 * come out as 0000/8080/0007...), nor does its SCH encoder. Since the air carries the
 * exact bits, they are written directly into the result page the ARM will read
 * (l1s_bts_resp: d[0] toa, d[1] pm, d[3] energy > 0x1000, d[5+i/4] nibble (3 - i%4),
 * bit 3 = value; order of the 116 bits: burst[3..59], burst[88..144], burst[87],
 * burst[60]). The RACH (rx cmd AB) goes through the I/Q and is not touched. The
 * "bits already correct" counter shows whether the core emulates the extcode correctly
 * (CALYPSO_BSP_TRX_UL=0 to disable). */
static struct { uint32_t fn, tick; uint8_t bits[148]; int valide; } g_trx_ul;
#define BSP_AUTRES_RING (1u << 16)
struct autres_s { uint32_t fn; uint8_t bits[7][148]; uint8_t presents; };
static struct autres_s *g_autres;   /* filled by bsp_autres_stocker (mobile TS1..7), read by the TS1 prosthesis */
static unsigned long g_autres_stockes, g_autres_joues;
/* access burst (05.02 5.2.7): 8 extended tail bits then the 41-bit synchronisation sequence */
static int bsp_ts0_est_rach(const uint8_t *bits)
{
    static const char sync[] = "01001011011111111001100110101010001111000";
    for (int i = 0; i < 41; i++) if ((bits[8 + i] & 1) != (uint8_t)(sync[i] - '0')) return 0;
    return 1;
}
/* how many of the 116 bits of the page already match the exact burst (without writing):
 * ~85-116 in the page where the extcode just demodulated THIS burst, ~58 (chance) in the other. */
static int trx_ul_compter(const uint16_t *d, const uint8_t *bits)
{
    int j = 0;
    for (int i = 0; i < 116; i++) {
        int pos = i < 57 ? 3 + i : i < 114 ? 88 + (i - 57) : i == 114 ? 87 : 60;
        int sh = (3 - (i & 3)) << 2;
        if (((d[5 + (i >> 2)] >> sh) & 8) == ((bits[pos] & 1) ? 8 : 0)) j++;
    }
    return j;
}
static void trx_ul_ecrire(uint16_t *d, const uint8_t *bits, int *justes)
{
    /* d[] of l1s_bts_resp: toa, pm, -, energy, -, 116 bits as nibbles (bit 3); bit order:
     * burst[3..59], burst[88..144], burst[87], burst[60] */
    int j = 0;
    for (int i = 0; i < 116; i++) {
        int pos = i < 57 ? 3 + i : i < 114 ? 88 + (i - 57) : i == 114 ? 87 : 60;
        int bit = bits[pos] & 1;
        int sh = (3 - (i & 3)) << 2;
        if (((d[5 + (i >> 2)] >> sh) & 8) == (bit ? 8 : 0)) j++;
        d[5 + (i >> 2)] = (uint16_t)((d[5 + (i >> 2)] & ~(0xf << sh)) | ((bit ? 0xf : 0x0) << sh));
    }
    d[0] = 3;                                      /* toa = d[0] - 3 = 0 */
    if (d[3] <= 0x1000) { d[3] = 0x4000; d[1] = 0x1530; d[2] = 0; }
    if (justes) *justes = j;
}
int calypso_bsp_trx_ul_prothese(uint16_t *dspdata, uint32_t tick, int w_page)
{
    static int actif = -1; static unsigned n_ok, n_pages, n_justes, n_ts1;
    if (actif < 0) { const char *e = calypso_getenv("CALYPSO_BSP_TRX_UL"); actif = !(e && *e == '0'); }
    if (!actif || !g_trx_cale) return 0;
    (void)w_page;
    /* The page is selected by the parity of fn, not by d_dsp_page. The ARM W page (index
     * 0/1, toggles every frame) and the DSP page bit each start from 0 at their own reset:
     * their relation changes from one boot to the next, and a skipped DSP tick is enough
     * to invert it. The result page of frame fn is the one of index (fn + parity) & 1.
     * Default: write BOTH pages on every burst (the exact burst of fn is in the page the
     * ARM will read, whichever it is; the other page is rewritten at the next burst before
     * the ARM reads it). CALYPSO_BSP_TRX_PARITE=auto: the parity is VOTED, mainly by the
     * bits (the DSP page is the one whose demod resembles the exact burst, trx_ul_compter,
     * gap >= 20 bits), with the extcode energy detections (d3 > 0x1000 in a single page)
     * as a fallback; if, once calibrated, the written page stays random over 64 bursts,
     * the parity is flipped once (parite_flip). CALYPSO_BSP_TRX_PARITE=0|1: forced. */
    static int parite = -1; static unsigned votes[2], votes_nrj[2];
    static int parite_flip, mode_pages = -1; static unsigned n_post, j_post;
    if (mode_pages < 0) {
        const char *e = calypso_getenv("CALYPSO_BSP_TRX_PARITE");
        if (e && (*e == '0' || *e == '1')) { parite = *e - '0'; parite_flip = 1; mode_pages = 1;
            printf("  [ts0] prothese : parite de page FORCEE = %d (CALYPSO_BSP_TRX_PARITE)\n", parite); }
        else if (e && !strcmp(e, "auto")) { mode_pages = 0; printf("  [ts0] prothese : parite de page VOTEE (CALYPSO_BSP_TRX_PARITE=auto)\n"); }
        else { mode_pages = 2; parite_flip = 1;
            printf("  [ts0] prothese : burst exact ecrit dans LES DEUX pages (defaut depuis le 21/10 ; CALYPSO_BSP_TRX_PARITE=auto|0|1 pour une seule)\n"); }
    }
    int fait = 0;
    static unsigned n_pas_valide, n_diff, n_appels;
    n_appels++;
    if (!g_trx_ul.valide) n_pas_valide++;
    else if ((int32_t)(tick - g_trx_ul.tick) != 1) n_diff++;
    if (n_appels % 20000 == 0)
        printf("  [ts0] prothese TS0 : %u appels, %u sans burst, %u mauvais tick, parite %d (votes %u/%u)\n", n_appels, n_pas_valide, n_diff, parite, votes[0], votes[1]);
    /* TS0: the burst played by bsp_ts0_livrer at tick T = tick - 1 */
    if (g_trx_ul.valide && (int32_t)(tick - g_trx_ul.tick) == 1) {
        if (parite < 0) {
            int det = -1, j[2] = { -1, -1 };
            for (int pg = 0; pg < 2; pg++) {
                uint16_t *db = &dspdata[pg ? 0x2200 : 0x2000];
                if (db[16] != 2) continue;
                j[pg] = trx_ul_compter(&db[34 + 32], g_trx_ul.bits);
                if (db[34 + 32 + 3] > 0x1000) det = (det < 0) ? pg : -2;
            }
            if (j[0] >= 0 && j[1] >= 0 && (j[0] - j[1] >= 20 || j[1] - j[0] >= 20))
                votes[(unsigned)((j[1] > j[0]) ^ (g_trx_ul.fn & 1))]++;
            if (det >= 0) votes_nrj[(unsigned)(det ^ (g_trx_ul.fn & 1))]++;
            if (mode_pages != 0) { /* both pages: no decision, only the measurement is kept */ }
            else if (votes[0] + votes[1] >= 4 && votes[0] != votes[1] && 3 * (votes[0] > votes[1] ? votes[1] : votes[0]) <= votes[0] + votes[1]) {
                parite = votes[1] > votes[0];
                printf("  [ts0] prothese : parite de page calibree = %d (votes bits %u/%u, energie %u/%u) : la trame fn ecrit dans la page (fn + %d) & 1\n",
                       parite, votes[0], votes[1], votes_nrj[0], votes_nrj[1], parite);
            } else if (n_ok >= 40 && votes_nrj[0] + votes_nrj[1] >= 6 && votes_nrj[0] != votes_nrj[1]) {
                parite = votes_nrj[1] > votes_nrj[0];
                printf("  [ts0] prothese : parite de page calibree EN SECOURS par l'energie = %d (votes bits %u/%u, energie %u/%u) : les bits ne tranchent pas (demod au hasard dans les deux pages ?)\n",
                       parite, votes[0], votes[1], votes_nrj[0], votes_nrj[1]);
            }
            if (n_ok < 5 || (n_ok % 10 == 0 && n_ok < 100) || n_ok % 500 == 0)
                printf("  [ts0] prothese : bits justes page0=%d page1=%d (fn=%u, energie dans la page %d) votes bits %u/%u energie %u/%u\n",
                       j[0], j[1], g_trx_ul.fn, det, votes[0], votes[1], votes_nrj[0], votes_nrj[1]);
        }
        for (int pg = 0; pg < 2; pg++) {
            if (parite >= 0 && pg != (int)((g_trx_ul.fn + parite) & 1)) continue;
            uint16_t *db = &dspdata[pg ? 0x2200 : 0x2000];
            if (db[16] != 2) continue;                 /* rx[0].cmd: 2 = NB (AB window: left alone) */
            int justes = 0;
            trx_ul_ecrire(&db[34 + 32], g_trx_ul.bits, &justes);
            n_justes += (unsigned)justes; n_pages++; fait = 1;
            if (parite >= 0) { n_post++; j_post += (unsigned)justes; }
        }
        /* guard: written page still random (< 70/116 over 64 bursts) -> the parity was wrong */
        if (parite >= 0 && !parite_flip && n_post >= 64) {
            if (j_post < 70u * n_post) {
                parite ^= 1; parite_flip = 1;
                printf("  [ts0] prothese : la page ecrite restait au hasard (%.1f/116 sur %u bursts) -> parite basculee a %d\n", (double)j_post / n_post, n_post, parite);
            } else if (n_post == 64)
                printf("  [ts0] prothese : parite %d confirmee (%.1f/116 justes sur %u bursts)\n", parite, (double)j_post / n_post, n_post);
            if (n_post >= 64 && j_post >= 70u * n_post) parite_flip = 1;   /* confirmed: no more flipping */
        }
        if (fait) {
            n_ok++;
            if (n_ok <= 5 || n_ok % 500 == 0)
                printf("  [ts0] prothese montante : burst fn=%u (tick %u) ecrit a la phase A du tick %u ; bits deja justes avant ecriture : %.1f/116 en moyenne (%u bursts, parite %d)\n",
                       g_trx_ul.fn, g_trx_ul.tick, tick, n_pages ? (double)n_justes / n_pages : 0.0, n_ok, parite);
        }
        g_trx_ul.valide = 0;
    } else if (g_trx_ul.valide && (int32_t)(tick - g_trx_ul.tick) > 1) g_trx_ul.valide = 0;
    /* TS1 (the TCH): the mobile uplink burst stored for frame w = T + offset, written into
     * rx[2] / data[72..] of the page of w, which the patched firmware reports with
     * tn = rx_start + 1 (prim_bts.c). */
    if (g_autres && g_ts0_offset != INT64_MIN) {
        int64_t w = ((int64_t)(tick - 1) + g_ts0_offset) % (int64_t)BSP_FN_MAX; if (w < 0) w += BSP_FN_MAX;
        unsigned i = (unsigned)w % BSP_AUTRES_RING;
        static unsigned n_look, n_trouve;
        n_look++;
        if (g_autres[i].fn == (uint32_t)w && (g_autres[i].presents & 1)) {
            n_trouve++;
            for (int pg = 0; pg < 2; pg++) {
                if (parite >= 0 && pg != (int)(((uint32_t)w + parite) & 1)) continue;
                uint16_t *db = &dspdata[pg ? 0x2200 : 0x2000];
                uint16_t *d = &db[34 + 72];
                memset(d, 0, 40 * sizeof *d);
                trx_ul_ecrire(d, g_autres[i].bits[0], NULL);
                db[16 + 2 * 2] = 2;                    /* rx[2].cmd = NB: the marker read by l1s_bts_resp */
                fait = 1;
            }
            g_autres[i].presents &= (uint8_t)~1u;
            n_ts1++;
            if (n_ts1 <= 5 || n_ts1 % 500 == 0)
                printf("  [ts0] prothese montante TS1 : burst fn=%u ecrit (rx[2], data[72]) a la phase A du tick %u (%u bursts TS1, parite %d)\n", (uint32_t)w, tick, n_ts1, parite);
        }
        if (n_look % 20000 == 0)
            printf("  [ts0] prothese TS1 : %u recherches, %u trouvees, %lu bursts TS1..7 stockes au total\n", n_look, n_trouve, g_autres_stockes);
    }
    return fait;
}
int calypso_bsp_trx_calage(uint32_t fn_sb, uint32_t tick)
{
    static int actif = -1, delta = -3;   /* -3: BTS_RX_LAG 4 of the patched trx firmware (prim_bts.c); -1 for the original firmware */
    if (actif < 0) {
        const char *e = calypso_getenv("CALYPSO_BSP_TRX"); actif = (e && *e == '1') ? 1 : 0;
        e = calypso_getenv("CALYPSO_BSP_TRX_DELTA"); if (e && *e) delta = atoi(e);
        if (actif) BSP_LOG("mode TRX : offset fn/tick pose par le SB emis (delta %d), bursts montants joues a la trame du firmware", delta);
    }
    if (!actif) return 0;
    int64_t off = ((int64_t)fn_sb + delta - (int64_t)tick) % (int64_t)BSP_FN_MAX; if (off < 0) off += BSP_FN_MAX;
    if (g_trx_cale && off == g_ts0_offset) return 1;
    static unsigned n;
    if (n++ < 10 || n % 500 == 0)
        printf("  [ts0] mode TRX : offset fn-tick %s %lld (SB fn=%u au tick %u, delta %d)\n",
               g_trx_cale ? "recale a" : "pose a", (long long)off, fn_sb, tick, delta);
    g_ts0_offset = off; g_trx_cale = 1;
    return 1;
}
static uint32_t g_ts0_fn_max;   /* most recent BTS frame stored */
static unsigned long g_ts0_recales;      /* number of offset re-alignments (see bsp_ts0_service) */
static unsigned long g_ts0_silences;     /* erasures deposited (see bsp_ts0_service) */
static long long     g_ts0_recul_total;  /* sum of the step-backs, in frames */
static const uint8_t bsp_train_sb[64] = { 1,0,1,1,1,0,0,1,0,1,1,0,0,0,1,0,0,0,0,0,0,1,0,0,0,0,0,0,1,1,1,1, 0,0,1,0,1,1,0,1,0,1,0,0,0,1,0,1,0,1,1,1,0,1,1,0,0,0,0,1,1,0,1,1, };
static int bsp_ts0_est_sb(const uint8_t *b) { for (int k = 0; k < 64; k++) if ((b[42 + k] & 1) != bsp_train_sb[k]) return 0; return 1; }
static int bsp_ts0_est_fcch(const uint8_t *b) { for (int k = 0; k < 148; k++) if (b[k] & 1) return 0; return 1; }
static int g_toa_bias = 0;   /* placement bias, in samples; see calypso_bsp_toa_feedback() */
static uint32_t g_tick_courant;        /* (forward definition: the current tick, set by bsp_ts0_service) */
static void bsp_ts0_stocker(uint32_t fn, const uint8_t *bits)
{
    if (!g_ts0) g_ts0 = calloc(BSP_TS0_RING, sizeof *g_ts0);
    if (!g_ts0) return;
    unsigned i = fn % BSP_TS0_RING;
    /* TRX mode: arrival margin of the uplink burst = (tick where it will be played) - (current tick);
     * <= 0: arrived too late, lost. Statistics over the first 2000, then every 2000. */
    if (g_trx_cale) {
        static long n, n_tard, somme; static int mini = 999;
        int64_t tj = ((int64_t)fn - g_ts0_offset) % (int64_t)BSP_FN_MAX; if (tj < 0) tj += BSP_FN_MAX;
        int64_t dm = tj - (int64_t)g_tick_courant; if (dm > BSP_FN_MAX / 2) dm -= BSP_FN_MAX; if (dm < -(int64_t)BSP_FN_MAX / 2) dm += BSP_FN_MAX;
        int m = (int)dm;
        n++; somme += m; if (m < mini) mini = m; if (m <= 0) n_tard++;
        if (n == 50 || n == 500 || n % 2000 == 0)
            printf("  [ts0] mode TRX : marge d'arrivee des bursts montants : min %d, moyenne %.1f trame(s), %ld/%ld en retard\n", mini, (double)somme / n, n_tard, n);
    }
    g_ts0[i].fn = fn; memcpy(g_ts0[i].bits, bits, 148); g_ts0[i].valid = 1; g_ts0[i].joue = 0;
    /* The most recent frame received from the BTS. On a miss it tells ON WHICH
     * SIDE the gap is: if the requested frame is beyond it, the DSP runs ahead
     * of the BTS (more lead is needed); if it is below, the burst did exist and
     * the cause is elsewhere. */
    if (!g_ts0_any || (int32_t)(fn - g_ts0_fn_max) > 0) { g_ts0_fn_max = fn; }
    bsp.bursts_seen++;   /* the drain loop of calypso_bsp_service() stops when this does not move */
    if (!g_ts0_any) { g_ts0_any = 1; g_ts0_next = fn; }
}
/* THE WINDOW'S TIMESLOT, AS TOLD BY THE FIRMWARE.
 *
 * tpu_window.c:89: tpu_enq_offset((5000 + l1s.tpu_offset + 625*tn) % 5000).
 * The TPU_OFFSET register, which QEMU relays in every TICK, therefore carries
 * the sync offset AND the position of the timeslot the RX window is armed on.
 *
 * l1s.tpu_offset is unknown but not needed: outside dedicated mode the ARM
 * listens to TS0, so the value observed then IS the reference. The distance to
 * the reference, divided by 625, gives the timeslot. The reference relearns
 * itself on every resync, since the mobile is then not in dedicated mode. */
static int g_bsp_tpu_offset = 0;
static int g_tpu_ref = -1;

/* Timeslot of the current window, 0 if unknown. */
static int bsp_fenetre_tn(void)
{
    if (g_tpu_ref < 0) {
        return 0;
    }
    int d = ((g_bsp_tpu_offset - g_tpu_ref) % 5000 + 5000) % 5000;
    return ((d + 312) / 625) % 8;   /* 625 qbits per timeslot, rounded */
}
/* For montant.c: the timeslot of the TPU's current RX window (-1 if unknown). The mobile uses
 * it to find the TS of its TCH ON ITS OWN when nobody announced the ASSIGNMENT COMMAND to it
 * (A5/1-ciphered on the SDCCH: neither the bridge nor the air can read it). */
int calypso_bsp_fenetre_tn(void) { return bsp_fenetre_tn(); }

/* THE DEDICATED TIMESLOT.
 *
 * The bridge sends the eight timeslots of every frame, the store above only
 * keeps one (TS0) and bsp_ts0_livrer() completes the frame. As long as the
 * mobile reads the BCCH and the CCCH this is exact: everything it needs is on
 * TS0. Once it enters dedicated mode it is not: the BSC allocates it e.g. an
 * SDCCH/8 on TS1, where the UA and the LOCATION UPDATING ACCEPT arrive.
 *
 * QEMU learns this timeslot from the firmware's L1CTL stream and announces it
 * through PONT_DCCH; the bursts of THAT timeslot are then kept, by BTS frame
 * number, and bsp_ts0_livrer() plays them INSTEAD of TS0. The frame stays one
 * burst per tick: the 1250-symbol pacing is not touched.
 *
 * One timeslot at a time, like the mobile: the store is allocated when the
 * channel is armed and restarts from scratch if the timeslot changes. */
#define BSP_DEDIE_RING (1u << 16)   /* ~5 min of BTS frames: the lock-step DSP drifts by several seconds */
static struct { uint32_t fn; uint8_t bits[148]; uint8_t valid; } *g_dedie;

/* THE FRAME DELIVERED TO THE DSP IS A REAL FRAME.
 *
 * The pacing (one burst per tick, indexed by BTS frame number) is not a
 * crutch: without it the ROM sees ~1526 symbols per frame instead of 1250 and
 * its TOA never settles (TOA at 2967, 3735, 4215, 48, 13536 and "BURST ID
 * 3!=2", "EMPTY"). Zero filler for the other timeslots WOULD be one: the mobile
 * would hear nothing of what the BTS emits outside TS0. All eight timeslots are
 * therefore kept, and the frame handed to the DSP carries what the BTS really
 * emitted. */

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
/* Kind of the dedicated channel: 0 = SDCCH/4, 1 = SDCCH/8,
 * BSP_DEDIE_TCH = TCH -- on a TCH ALL the frames of the timeslot belong to
 * the connection (26-multiframe: 24 traffic + 1 SACCH + 1 idle), not only
 * the blocks of a sub-channel. */
#define BSP_DEDIE_TCH 2
/* PDCH (GPRS): like the TCH, all the frames of the timeslot belong to the
 * connection (blocks B0..B11 of the 52-multiframe, PTCCH, idle). */
#define BSP_DEDIE_PDCH 4
static int g_dedie_genre = 1;
static unsigned long g_dedie_stockes, g_dedie_joues, g_dedie_manques;

/* LAST TCH BURSTS DELIVERED TO THE DSP, for the downlink FACCH prosthesis of montant.c: under the
 * TI L1 (Magnetite) the ROM decodes the FACCH/F with its last two bursts missing ("01 8b 01 2b"
 * instead of the UA "01 73 01 2b"), while the delivered bursts carry the UA intact. */
#define TCH_ANNEAU 64u
static struct { uint32_t fn; uint8_t bits[148]; int ok; } g_tch_anneau[TCH_ANNEAU];
static uint32_t g_tch_dernier_fn; static int g_tch_dernier_ok;
int calypso_bsp_tch_dernier_fn(uint32_t *fn)
{
    if (!g_tch_dernier_ok || g_dedie_genre != BSP_DEDIE_TCH || g_dedie_tn <= 0) return 0;
    *fn = g_tch_dernier_fn;
    return 1;
}
int calypso_bsp_tch_burst(uint32_t fn, uint8_t bits[148])
{
    unsigned k = fn % TCH_ANNEAU;
    if (!g_tch_anneau[k].ok || g_tch_anneau[k].fn != fn) return 0;
    memcpy(bits, g_tch_anneau[k].bits, 148);
    return 1;
}
/* g_dedie_perdues: the dedicated frames that bsp_ts0_service() skips
 * ENTIRELY, for lack of a BTS burst for the requested frame. It leaves through
 * a `return` before bsp_ts0_livrer(), so neither `joues` nor `manques` see
 * them. Never read manques=0 as "nothing is lost" without reading perdues at
 * the same time. */
static unsigned long g_dedie_perdues;

/* Does this BTS frame belong to the mobile's dedicated channel?
 *
 * Only the frames of the channel are taken, those where its layer 1 arms its
 * window on the dedicated timeslot -- elsewhere it reads the BCCH and the
 * CCCH on TS0, and needs them. (The BTS emits on TS1 permanently, filler
 * bursts included: replacing TS0 on EVERY frame would leave the mobile with no
 * TS0 at all.)
 *
 * 45.002, SDCCH/8 + SACCH/8, 51-multiframe counted in pairs:
 *   downlink SDCCH, sub-channel i : frames 4i..4i+3 of every multiframe;
 *   downlink SACCH, i = 0..3      : frames 32+4i..35+4i, even multiframe;
 *                   i = 4..7      : same frames, odd multiframe. */
static bool bsp_dedie_trame(uint32_t fn)
{
    if (g_dedie_genre == BSP_DEDIE_TCH || g_dedie_genre == BSP_DEDIE_PDCH) {
        return true;    /* TCH, PDCH: the whole timeslot belongs to the connection */
    }
    /* TI FIRMWARE IN DEDICATED MODE ON AN SDCCH/8: ALL THE FRAMES OF THE TIMESLOT. The bench does not
     * know which sub-channel the mobile accepted: the BSC answers every RACH with a different SDCCH/8
     * (SS1, SS2...) and the bridge announces the last IMM ASS, while the mobile takes the one of ITS
     * reference. Wrong sub-channel -> SABM without UA, "lchan allocation failed ...
     * WAIT_RLL_RTP_ESTABLISH: Timeout". Once the firmware is in dedicated mode (g_dedie_ti: uplink
     * tasks posted, IMM ASS accepted) it no longer reads TS0: it gets the whole TS1 and picks its
     * sub-channel there. BEFORE (waiting for the IMM ASS), the SS alone: it needs TS0.
     * CALYPSO_BSP_SDCCH8_TOUT=0 falls back to the announced sub-channel. */
    if (g_dedie_genre == 1 && g_dedie_ti) {
        static int tout = -1;
        if (tout < 0) { const char *e = calypso_getenv("CALYPSO_BSP_SDCCH8_TOUT"); tout = !(e && *e == '0'); }
        if (tout) return true;
    }
    /* AND BEFORE THAT, NOTHING. Replacing TS0 with the SS frames as soon as the bridge announces an
     * IMM ASS (SS0, p51 0..3), while the mobile has accepted nothing yet, breaks the BCCH (2..5) and
     * overwrites the CCCH blocks 6..9 / 12..15 with SS1 / SS3: the IMM ASS of its own RACH is never
     * read, new RACH, SDCCH Timeout at the BSC. TI firmware (MS_MAGNETITE): TS0 intact until the entry
     * in dedicated mode (g_dedie_ti). CALYPSO_BSP_SDCCH_TS0_ATTENTE=0: SS frames replace TS0 at once. */
    if (g_dedie_genre == 1 && !g_dedie_ti) {
        static int attente = -1;
        if (attente < 0) { const char *e = calypso_getenv("CALYPSO_BSP_SDCCH_TS0_ATTENTE");
                           attente = e && *e ? (*e != '0') : (calypso_getenv("MS_MAGNETITE") != NULL); }
        if (attente) return false;
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

/* State of the dedicated channel, readable without the DSP terminal:
 * /dev/shm/calypso_bsp_dedie, one line rewritten on every notable change.
 * It is the only way to know, from outside, whether the bursts of the
 * dedicated timeslot reach this point and whether they are played. */
static unsigned long g_dedie_replis;   /* see bsp_dedie_bits() */
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
    if (genre == BSP_DEDIE_PDCH) {
        BSP_LOG("canal dedie arme : PDCH TS%d (GPRS) - TOUTES ses trames remplacent TS0", tn);
        printf("  [ts0] canal dedie arme : PDCH TS%d (GPRS, toutes les trames)\n", tn);
        return;
    }
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

/* THE DEDICATED STORE STARTS TOO LATE: FALL BACK ON g_autres.
 *
 * `g_dedie` is only allocated and filled when the channel is ARMED
 * (`calypso_bsp_set_dedie`), and the arming comes from QEMU's L1CTL tap, i.e.
 * from the FIRST dedicated block received -- not from the IMMEDIATE
 * ASSIGNMENT. The BSP plays BTS frame `tick + g_ts0_offset`, behind the one
 * arriving: the channel frames older than the arming would be played EMPTY
 * (one frame in four without burst, the signature of "Dropping frame with 110
 * bit errors" and of blocks rebuilt from three bursts out of four).
 *
 * `g_autres` ALREADY keeps the seven other timeslots of every frame,
 * unconditionally and from the first burst (it serves to complete the
 * continuous frame). `g_dedie` is a partial duplicate of it, kept because it
 * counts the channel bursts; when it does not have the frame, the one of
 * `g_autres` is taken instead of returning NULL.
 *
 * MONTANT_DEDIE_STRICT=1 disables the fallback, to measure the gap. */
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

/* THE SCH OUTSIDE A ONE-SHOT WINDOW: SB block waiting for an armed DMA.
 *
 * The ROM's SB task does not work on the continuous stream of the FB search:
 * it programs a task page (ALGTH >= 302 words, i.e. >= 151 samples), double
 * page, and expects the burst 23 symbols from the start of its window
 * (tpu_window.c: L1_SB_MARGIN_Q = 23 * 4). What it receives depends on the
 * RIF residue (4 words per FB search frame, never empty) and on the instant it
 * arms its DMA relative to the deposit: if the DMA is not armed when
 * bsp_ts0_livrer() deposits the SCH, calypso_rif_rx_burst() drops the burst
 * ("n_muets") and the page only holds stale data.
 *
 * Behind CALYPSO_BSP_SB_FENETRE (1 by default, 0 = no retention):
 *   1. an SCH deposited while the DMA is not armed is not dropped: it is kept
 *      ONE tick and delivered, framed at 21 (TOA 23), in the one-shot SB window
 *      (>= 190 samples) the ROM arms right after; the ARM-tick offset is
 *      realigned on the tick of the actual delivery;
 *   2. it is NEVER delivered in another window (NB 151, continuous): an
 *      unclaimed block expires;
 *   3. the [sbwin] trace tells armed/one_shot/programmed page/RIF residue and
 *      the action, and pont.c adds TOA/PM/SNR to the [a_sch] traces: a well
 *      framed SB reads TOA=23 or 24.
 * The FB search (48-sample page, DMA armed continuously) and the one-shot
 * windows armed before the deposit do not change. */
static int16_t  g_sb_attente_iq[2 * 512];
/* THE LAST DEPOSITED SCH, whatever the DMA state at the deposit. At the
 * deposit of the SCH the DMA is most often ARMED for something else (64-sample
 * PM window, one_shot=1: the SCH is consumed there), then the SB window arms
 * and finds nothing. The last SCH (fn, tick, bits) is therefore ALWAYS kept,
 * with a note of whether it was delivered in an SB window; if it was not and
 * an SB window arms in this tick or the next, it is delivered there. On
 * silicon the SB window covers the instant of the burst whatever the DSP did
 * before: this is what it reconstructs. */
static struct { uint32_t fn; uint32_t tick; uint8_t bits[148]; int livre_sb; int valide; } g_dernier_sch;
static uint32_t g_sb_trace_jusqua;     /* tick up to which EVERY SCH is traced ([sbwin]) */
static unsigned long g_sb_relivres, g_sb_expires;
static uint32_t g_tick_courant;        /* current tick (bsp_ts0_service), to date the redelivery */
static int sb_fenetre_on(void)
{
    static int on = -1;
    if (on < 0) { const char *e = calypso_getenv("CALYPSO_BSP_SB_FENETRE"); on = (e && *e == '0') ? 0 : 1;
                  if (on) BSP_LOG("SB_FENETRE on : un SCH depose alors que la DMA n'est pas armee est garde un tick et livre dans la fenetre SB one-shot qui suit"); }
    return on;
}
/* Frames the SB burst in `iq` for the one-shot window as programmed at this
 * instant (191 samples, burst at 21: TOA 23); returns the number of samples
 * of the block, 0 if it is not an SB window. */
/* SCH MODULATION, TUNED IN REPLAY (c54x_exe --rejouer, REJEU_CONTINUER=1,
 * 8000 frames, deterministic). On the failures, mostly only bits 0-2 of the
 * SB word are wrong: the "razor's edge" profile of the NBs before
 * gmsk_elargir. Sweep (SCH decoded / SCH present, 8000 frames):
 *   widening a: 0 -> 34 %, 0.15 -> 50 %, 0.3 -> 60 %, 0.5-1.5 -> 63-66 %
 *   + sampling instant 0.35 (instead of 0.5)                     -> 76 %
 *   + gaussian noise sigma 3000 (10 % of the amplitude)           -> 82 %
 *   carrier phase, head margin: no effect; amplitude < 22000: no more FB.
 * Those three settings are the defaults here (CALYPSO_BSP_SB_SYM, _SB_DEC,
 * _SB_NOISE; 0 / 0.5 / 0 = none of them). The noise is seeded by the fn of
 * the frame: deterministic, replayable. */
/* TWO WAVEFORMS FOR THE SCH. In replay (8000 frames, 209 SCH): waveform A
 * (below) misses 20 % of the SCHs; waveform D (A + causal tail 0.8, 0.4 and
 * another noise seed) misses 17 %; both together only miss 7 % in common
 * (A+D = 15/209). g_sb_variante = 1 selects D, used by the second chance
 * (see bsp_ts0_service). */
static int g_sb_variante = 0;
static void sb_moduler(const uint8_t *bits, int16_t *dst, uint32_t fn)
{
    static double sym = -2, dec = -2, sigma = -2;
    if (sym == -2) {
        const char *e;
        e = calypso_getenv("CALYPSO_BSP_SB_SYM");   sym   = (e && *e) ? atof(e) : 1.0;
        e = calypso_getenv("CALYPSO_BSP_SB_DEC");   dec   = (e && *e) ? atof(e) : 0.35;
        e = calypso_getenv("CALYPSO_BSP_SB_NOISE"); sigma = (e && *e) ? atof(e) : 3000.0;
        if (dec < 0 || dec >= 1.0) dec = 0.5;
        BSP_LOG("SCH : elargissement %.2f, instant %.2f symbole, bruit sigma %.0f (mesure rejeu du 2026-09-30 : 82 %% de SCH decodes)", sym, dec, sigma);
    }
    gmsk_moduler(bits, 148, 30000, 0.0, dec, dst);
    if (sym != 0.0) gmsk_elargir(dst, 148, sym);
    if (g_sb_variante == 1) {                       /* forme D : traine causale 0.8, 0.4, normalisee */
        double xi[148], xq[148];
        for (int k = 0; k < 148; k++) { xi[k] = dst[2*k]; xq[k] = dst[2*k+1]; }
        for (int k = 0; k < 148; k++) {
            double yi = xi[k], yq = xq[k];
            if (k >= 1) { yi += 0.8 * xi[k-1]; yq += 0.8 * xq[k-1]; }
            if (k >= 2) { yi += 0.4 * xi[k-2]; yq += 0.4 * xq[k-2]; }
            dst[2*k] = (int16_t)lrint(yi / 2.2); dst[2*k+1] = (int16_t)lrint(yq / 2.2);
        }
    }
    if (sigma > 0) {
        uint32_t seed = fn * 2654435761u + 777u + (g_sb_variante ? 7919u : 0u);
        for (int k = 0; k < 296; k++) {
            seed = seed * 1103515245u + 12345u; double u1 = ((seed >> 8) & 0xffff) / 65536.0 + 1e-6;
            seed = seed * 1103515245u + 12345u; double u2 = ((seed >> 8) & 0xffff) / 65536.0;
            double g = sqrt(-2.0 * log(u1)) * cos(2.0 * M_PI * u2);
            double v = dst[k] + sigma * g;
            if (v > 32767) v = 32767;
            if (v < -32768) v = -32768;
            dst[k] = (int16_t)lrint(v);
        }
    }
}
static int sb_cadrer(const uint8_t *bits, int16_t *iq, int cap_ech, int *marge_out)
{
    if (!calypso_rhea_dma_one_shot()) return 0;
    int total = calypso_rhea_dma_get_len_words() / 2;           /* echantillons de la page one-shot */
    if (total < 190) return 0;                                  /* 151 = fenetre NB : pas pour un SCH */
    if (total > cap_ech) total = cap_ech;
    int marge = 21;
    if (marge > total - 148) marge = total - 148;
    memset(iq, 0, (size_t)(2 * total) * sizeof *iq);
    sb_moduler(bits, iq + 2 * marge, g_dernier_sch.fn);
    *marge_out = marge;
    return total;
}
/* The ROM arms a one-shot SB window of 191 samples (one_shot=1
 * page_prog=191, TOA=24 CRC_OK), often AFTER the deposit of the SCH in the
 * tick; calypso_rif_rx_burst() then drops the burst (empty window on page 0:
 * TOA=0 SNR=12) and page 1 receives the burst of the NEXT frame at TOA 24,
 * hence a wrong CRC. The kept block is redelivered ONLY in a one-shot SB
 * window (>= 190), at most one tick after the deposit -- never in the NB
 * windows of the BCCH reading (one_shot=0 page=151), where it would replace
 * the BCCH burst -- and the ARM-tick offset is realigned on the tick of the
 * actual delivery, as the native delivery of an SCH in an SB window does. An
 * unclaimed block expires. */
int calypso_bsp_sb_retenter(void)
{
    if (!g_dernier_sch.valide || g_dernier_sch.livre_sb) return 0;
    if ((int32_t)(g_tick_courant - g_dernier_sch.tick) > 1) {   /* trop vieux : la ROM ne l'attend plus */
        g_dernier_sch.valide = 0; g_sb_expires++;
        return 0;
    }
    if (!calypso_rhea_dma_rx_armed() || !calypso_rhea_dma_one_shot()) return 0;
    int marge = 0;
    int total = sb_cadrer(g_dernier_sch.bits, g_sb_attente_iq, 512, &marge);
    if (!total) return 0;                                       /* armee, mais pas une fenetre SB (PM 64, NB 151) : on attend */
    calypso_rif_flush();
    calypso_bsp_rx_burst(0, g_dernier_sch.fn, g_sb_attente_iq, 2 * total);
    bsp.bursts_written++;
    g_sb_relivres++;
    g_dernier_sch.livre_sb = 1;
    /* The SCH is delivered at this tick: it is the one that sets the ARM-tick
     * offset, exactly like the native delivery in an SB window (bsp_ts0_livrer). */
    int64_t off = (int64_t)g_dernier_sch.fn - (int64_t)g_tick_courant - sb_fin_trame();
    /* Not after a RA_C: the mobile does not realign its fn on those SBs (RA_C offset 752, SB 758 at
     * the same moment); the SDCCHs would be delivered six frames off (a_cd FIRE KO). */
    if (!g_off_rac_vu) g_off_mobile = (int64_t)g_dernier_sch.fn - (int64_t)g_tick_courant;   /* what the mobile just learnt */
    int recale = (off != g_ts0_offset);
    if (!g_trx_cale && !g_bcch_verrou) g_ts0_offset = off;
    if (g_sb_relivres <= 100 || g_sb_relivres % 100 == 0)
        printf("  [sbwin] SB fn=%u RELIVRE au tick %u (depose au tick %u, fenetre SB armee apres) : page=%d marge=%d%s [%lu relivres, %lu expires]\n",
               g_dernier_sch.fn, g_tick_courant, g_dernier_sch.tick, total, marge,
               recale ? ", offset ARM-tick recale" : "", g_sb_relivres, g_sb_expires);
    return 1;
}
/* WINDOW ROUTING IN DEDICATED MODE (TI firmware). The bench deposits ONE burst (148 samples, TS0)
 * per tick; in dedicated mode the ROM arms TWO one-shot windows per frame: the power measurement
 * (64) then the NB reception (151), 215 samples. On silicon the measurement falls on another
 * timeslot and consumes emptiness. Here: measurement window (< 150) <- filler (bsp_remplissage),
 * NB window (150..189) <- the real burst, kept in a ring if the NB is not armed at the deposit;
 * pont.c runs the DSP between two deliveries within the same tick (pump loop). Only in TI
 * dedicated mode (calypso_bsp_set_dedie_ti); CALYPSO_BSP_NB_RETENTER=0 disables. */
#define NB_GARDE_N 4
static struct { int valide; uint32_t fn, tick; uint8_t bits[148]; } g_nb_garde[NB_GARDE_N];
static unsigned g_nb_garde_w;
static unsigned long g_nb_relivres, g_nb_expires, g_pm_servies;
static int nb_retenter_on(void)
{
    /* IN IDLE MODE TOO (not only in TI dedicated mode): the TI L1 measures neighbours on the paging
     * and BCCH frames, so at the deposit a measurement window (64) is often the armed one; without
     * the retention the paging blocks come out in error (NP_I error_flag=1), the DSC counter runs
     * out, the cell is reselected every ~8 s and no paging is received (no MT SMS/call).
     * CALYPSO_BSP_NB_RETENTER=0 disables, =2: dedicated mode only. */
    /* DEFAULT ACCORDING TO THE FIRMWARE. The idle-mode retention serves the TI L1 (MS_MAGNETITE):
     * without it, every NP_I in error, no incoming call. The osmocom firmware does not need it and
     * it empties its FB search (FBSB result=255, see continue_fb in bsp_ts0_livrer): 2 (dedicated
     * only) by default without MS_MAGNETITE. CALYPSO_BSP_NB_RETENTER forces the value in both cases. */
    static int on = -1;
    if (on < 0) {
        const char *e = calypso_getenv("CALYPSO_BSP_NB_RETENTER");
        on = (e && *e) ? atoi(e) : (calypso_getenv("MS_MAGNETITE") ? 1 : 2);
        BSP_LOG("NB_RETENTER=%d (%s) : garde NB %s", on, (e && *e) ? "force" : "defaut du firmware",
                on == 1 ? "en veille et en dedie" : on == 2 ? "en dedie seulement" : "coupee");
    }
    return on == 1 || (on == 2 && g_dedie_ti);
}
static int pm_sur_cellule(void);   /* does the measurement target the cell frequency? (fake air) */
static void nb_garde_poser(uint32_t fn, uint32_t tick, const uint8_t *bits)
{
    unsigned k = g_nb_garde_w++ % NB_GARDE_N;
    if (g_nb_garde[k].valide) g_nb_expires++;
    g_nb_garde[k].fn = fn; g_nb_garde[k].tick = tick; g_nb_garde[k].valide = 1;
    memcpy(g_nb_garde[k].bits, bits, 148);
}
/* AGC ON THE NORMAL BURSTS. The TI L1 derives the cell's IL from a_serv_demod[D_PM], which the DSP
 * measures on the samples of the NB window (l1ctl_pagc: IL = -(pm - programmed agc ...)). A fixed
 * amplitude of 30000 whatever the programmed TRF6151 gain would freeze pm at 169, let the PAGC loop
 * run idle (gain 112/124/136, IL 59/19, s_rxlev 69/101 > 63) and hide the antenna icon
 * (rxlev*5/64 >= 5, iconsShowSignal). The amplitude follows the gain of the window: 30000 at the
 * reference gain CALYPSO_BSP_NB_GAIN_REF (dB, anchor of the level seen by the mobile: +1 dB ->
 * rxlev -1; 124: floor gain 92, IL 83, s_rxlev 63/69; 148: gain 112..124, IL 131, s_rxlev 45 = the
 * neighbour ARFCN 1 measured by a_pm). CALYPSO_BSP_NB_AGC=0 goes back to a fixed 30000. Only the
 * NB windows: the measurement windows go through the a_pm model, which already adds the gain
 * (bsp_apm_de_mav). */
static int nb_amp(uint32_t tick)
{
    static int on = -1; static double ref;
    static unsigned long n, n_ecrete; static int g_min = 999, g_max = -999;
    if (on < 0) { const char *e = calypso_getenv("CALYPSO_BSP_NB_AGC"); on = !(e && *e == '0');
                  e = calypso_getenv("CALYPSO_BSP_NB_GAIN_REF"); ref = (e && *e) ? atof(e) : 148.0; }
    if (!on) return 30000;
    int g = faux_air_gain(tick, 1250);
    if (g == -999) return 30000;
    double a = 30000.0 * pow(10.0, (g - ref) / 20.0);
    if (a > 32000.0) { a = 32000.0; n_ecrete++; }
    if (a < 30.0) a = 30.0;
    n++; if (g < g_min) g_min = g; if (g > g_max) g_max = g;
    if (n <= 10 || n % 2000 == 0)
        printf("  [nb-agc] tick=%u gain=%d ref=%.0f -> amplitude %d [%lu NB, gain %d..%d, %lu ecretees]\n",
               tick, g, ref, (int)a, n, g_min, g_max, n_ecrete);
    return (int)a;
}
static void nb_moduler_amp(const uint8_t *bits, int16_t *iq, int marge, int amp)
{
    static double sym = -2;
    if (sym == -2) { const char *e = calypso_getenv("CALYPSO_BSP_NB_SYM"); sym = (e && *e) ? atof(e) : 0.3; }
    gmsk_moduler(bits, 148, amp, 0.0, 0.5, iq + 2 * marge);
    gmsk_elargir(iq + 2 * marge, 148, sym);
}
static void nb_moduler(const uint8_t *bits, int16_t *iq, int marge)
{
    nb_moduler_amp(bits, iq, marge, 30000);
}
static int nb_marge_ti(void)
{
    static int nb_marge = -1;
    if (nb_marge < 0) { const char *e = calypso_getenv("CALYPSO_BSP_NB_MARGE"); nb_marge = (e && *e) ? atoi(e) : 3;
                        if (nb_marge < 0 || nb_marge > 3) nb_marge = 3; }
    return nb_marge;
}
/* the armed measurement window receives enough to complete (the ROM then arms the next one): the head of
 * the last kept burst if the measurement targets the cell (its power is the right one), filler otherwise */
static int pm_servir(int nwin)
{
    static int16_t iq[2 * 256];
    const int16_t *src = bsp_remplissage();
    int k_rec = -1;
    for (int k = 0; k < NB_GARDE_N; k++)
        if (g_nb_garde[k].valide && (k_rec < 0 || (int32_t)(g_nb_garde[k].fn - g_nb_garde[k_rec].fn) > 0)) k_rec = k;
    int sur_cell = pm_sur_cellule();
    if (k_rec >= 0 && sur_cell) {
        memset(iq, 0, sizeof iq);
        nb_moduler(g_nb_garde[k_rec].bits, iq, 0);
        src = iq;
    }
    /* PROBE: in paging idle mode (Magnetite, IBA_I), when the cell measurement (ARFCN 1) falls to the
     * noise floor, tells whether the measurement window receives filler for lack of a kept burst
     * (k_rec<0) or because the fake air sees it tuned elsewhere. First 40 then 1 in 500. */
    {
        static unsigned long n, n_sans_garde, n_hors;
        n++; if (k_rec < 0) n_sans_garde++; if (!sur_cell) n_hors++;
        if (n <= 40 || n % 500 == 0)
            printf("  [pm] tick=%u nwin=%d garde=%s (fn %u, %d ticks) faux-air=%d cellule=%d -> %s "
                   "[%lu servies, %lu sans garde, %lu hors cellule]\n",
                   g_tick_courant, nwin, k_rec >= 0 ? "oui" : "non",
                   k_rec >= 0 ? g_nb_garde[k_rec].fn : 0u,
                   k_rec >= 0 ? (int)(g_tick_courant - g_nb_garde[k_rec].tick) : -1,
                   faux_air_arfcn(g_tick_courant, 5000), faux_air_cellule(),
                   src == iq ? "burst" : "remplissage", n, n_sans_garde, n_hors);
    }
    calypso_rif_flush();
    calypso_bsp_rx_burst(0, g_depot_fn, src, 2 * (nwin > 148 ? 148 : nwin));
    g_pm_servies++;
    return 1;
}
/* DMA PROBE (CALYPSO_BSP_DMA_TRACE=1): every state change of the RX channel (armed, one-shot, length)
 * with the tick, during TI dedicated mode. The rhea-dma model traces its registers on stderr, but
 * c54x_exe (verbosite.c) swallows them. Shows what the ROM arms on the TCH (TS1). 3000 lines. */
static int pm_sur_cellule(void)
{
    int cell = faux_air_cellule();
    if (cell < 0) return 1;
    int a = faux_air_arfcn(g_tick_courant, pm_limite_qbit(pm_rang_fenetre()));   /* see pm_rang_fenetre() */
    return a < 0 || a == cell;
}
static void dma_trace(const char *ou)
{
    static int on = -1; static unsigned n; static int armee_p = -1, os_p = -1, len_p = -1;
    if (on < 0) { const char *e = calypso_getenv("CALYPSO_BSP_DMA_TRACE"); on = (e && *e == '1'); }
    if (!on || !g_dedie_ti || n >= 3000) return;
    int armee = calypso_rhea_dma_rx_armed(), os = calypso_rhea_dma_one_shot(), len = calypso_rhea_dma_get_len_words() / 2;
    if (armee == armee_p && os == os_p && len == len_p) return;
    armee_p = armee; os_p = os; len_p = len; n++;
    printf("  [dma] tick=%u %s : armee=%d one_shot=%d fenetre=%d rif=%d\n", g_tick_courant, ou, armee, os, len, calypso_rif_level());
}
void calypso_bsp_dma_trace(const char *ou) { dma_trace(ou); }
/* trace of the armings and deposits in TI dedicated mode (calypso_rhea_dma.c [dma-arm], rx_burst [dma-dep]) */
int calypso_bsp_dma_trace_on(void)
{
    static int on = -1; static unsigned n;
    if (on < 0) { const char *e = calypso_getenv("CALYPSO_BSP_DMA_TRACE"); on = (e && *e == '1'); }
    return on && g_dedie_ti && n++ < 20000;
}
unsigned long calypso_bsp_routeur_bilan(char *buf, size_t n)
{
    snprintf(buf, n, " (routeur : %lu NB relivres, %lu expires, %lu mesures servies)", g_nb_relivres, g_nb_expires, g_pm_servies);
    return g_nb_relivres;
}
int calypso_bsp_nb_retenter(void)
{
    dma_trace("pump");
    if (!nb_retenter_on()) return 0;
    /* NOT ON THE TCH. pont.c already deposits there after the IDLE, window armed (PONT_TCH_DEPOT_IDLE).
     * Two kept NBs redelivered in the same tick, with a DMA interrupt pending (IFR=4000) and bit 14
     * removed from the IMR (3000), send the ROM to pc=0 (SP-CORRUPT); the TI L1 then reinitialises
     * the DSP ~1400 frames later and the mobile loses the cell on every call.
     * CALYPSO_BSP_NB_RETENTER_TCH=1: redeliver on the TCH too. */
    {
        static int sur_tch = -1;
        if (sur_tch < 0) { const char *e = calypso_getenv("CALYPSO_BSP_NB_RETENTER_TCH"); sur_tch = (e && *e == '1'); }
        /* BUT THE MEASUREMENT WINDOW, YES. After the NB of the TCH, the ROM arms in the same frame a
         * 64-sample PM window (neighbours); if nobody serves it, it stays armed at the next tick and the
         * TS2 burst of that frame falls into it (64 samples, never delivered as NB): one NB in two lost,
         * FACCH UA and SACCH lost -> Assignment Failure. The measurement is served (no NB redelivered),
         * one per tick, never on a pending DMA interrupt. CALYPSO_BSP_PM_TCH=0 disables. */
        if (!sur_tch && g_dedie_tn > 0 && g_dedie_genre == BSP_DEDIE_TCH) {
            static int pm_tch = -1; static uint32_t t_pm = 0xffffffffu;
            if (pm_tch < 0) { const char *e = calypso_getenv("CALYPSO_BSP_PM_TCH"); pm_tch = !(e && *e == '0'); }
            if (!pm_tch || t_pm == g_tick_courant) return 0;
            if (!calypso_rhea_dma_rx_armed() || !calypso_rhea_dma_one_shot()) return 0;
            int nw = calypso_rhea_dma_get_len_words() / 2;
            if (nw <= 0 || nw >= 150) return 0;
            if (bsp.dsp && (bsp.dsp->ifr & (1u << 14))) return 0;
            t_pm = g_tick_courant;
            return pm_servir(nw);
        }
    }
    /* [nb-etat] the DMA state as seen here (redelivered vs expired NBs). */
    if (g_dedie_ti) {
        static unsigned n_et; static uint32_t t_et;
        int ng = 0; for (int k = 0; k < NB_GARDE_N; k++) ng += g_nb_garde[k].valide;
        if (ng && (n_et < 300 || g_tick_courant - t_et > 500)) {
            n_et++; t_et = g_tick_courant;
            printf("  [nb-etat] tick=%u armee=%d one_shot=%d nwin=%d garde=%d genre=%d tn=%d\n", g_tick_courant,
                   (int)calypso_rhea_dma_rx_armed(), (int)calypso_rhea_dma_one_shot(),
                   calypso_rhea_dma_get_len_words() / 2, ng, (int)g_dedie_genre, g_dedie_tn);
        }
    }
    if (!calypso_rhea_dma_rx_armed() || !calypso_rhea_dma_one_shot()) return 0;
    int nwin = calypso_rhea_dma_get_len_words() / 2;
    if (nwin <= 0) return 0;
    /* ON THE TCH: ONE DELIVERY PER TICK, NEVER ON A PENDING DMA INTERRUPT (two kept NBs redelivered
     * in the same tick with IFR bit 14 pending send the ROM to pc=0, jump outside the ROM from
     * 1:e9bd). Without redelivery, 70 % of the TCH NB windows stay empty (TOA=0, SNR=0): downlink
     * FACCH (UA) never decoded. */
    if (g_dedie_tn > 0 && g_dedie_genre == BSP_DEDIE_TCH) {
        static uint32_t t_livre = 0xffffffffu;
        if (t_livre == g_tick_courant) return 0;
        if (bsp.dsp && (bsp.dsp->ifr & (1u << 14))) return 0;
        if (nwin >= 150 && nwin < 190) t_livre = g_tick_courant;
    }
    if (nwin < 150) return pm_servir(nwin);
    if (nwin >= 190) return 0;                                  /* SB window: calypso_bsp_sb_retenter */
    /* the NB is armed: the oldest kept burst still fresh (2 ticks) */
    int k_min = -1;
    for (int k = 0; k < NB_GARDE_N; k++) {
        if (!g_nb_garde[k].valide) continue;
        if ((int32_t)(g_tick_courant - g_nb_garde[k].tick) > 2) { g_nb_garde[k].valide = 0; g_nb_expires++; continue; }
        if (k_min < 0 || (int32_t)(g_nb_garde[k].fn - g_nb_garde[k_min].fn) < 0) k_min = k;
    }
    if (k_min < 0) return 0;
    int marge = nb_marge_ti();
    static int16_t iq[2 * 256];
    int total = nwin > marge + 148 ? nwin : marge + 148;
    if (total > 256) total = 256;
    memset(iq, 0, sizeof iq);
    nb_moduler_amp(g_nb_garde[k_min].bits, iq, marge, nb_amp(g_tick_courant));
    calypso_rif_flush();
    calypso_bsp_rx_burst(0, g_nb_garde[k_min].fn, iq, 2 * total);
    bsp.bursts_written++;
    g_nb_garde[k_min].valide = 0;
    g_nb_relivres++;
    if (g_nb_relivres <= 40 || g_nb_relivres % 500 == 0)
        printf("  [nbwin] NB fn=%u p51=%u RELIVRE au tick %u (depose au tick %u) : fenetre=%d [%lu relivres, %lu expires, %lu mesures servies]\n",
               g_nb_garde[k_min].fn, g_nb_garde[k_min].fn % 51u, g_tick_courant, g_nb_garde[k_min].tick, nwin,
               g_nb_relivres, g_nb_expires, g_pm_servies);
    return 1;
}
/* pont.c: trace EVERY SCH for `ticks` ticks (after an FB detection, or when
 * the ARM posts an SB task), to see the exact deposit around the attempts. */
void calypso_bsp_sb_trace(unsigned ticks)
{
    uint32_t j = g_tick_courant + ticks;
    if ((int32_t)(j - g_sb_trace_jusqua) > 0) g_sb_trace_jusqua = j;
}
/* SB_FIN_TRAME: the BCCH reading (p51 2..5) tells where the L1 believes the block is. NB windows
 * armed on 3..6 = stream one frame ahead (offset - 1); on 1..4 = behind (offset + 1). Two blocks
 * in a row before correcting; a well aligned block locks the offset (re-read SCHs no longer
 * realign it), unlocked when no BCCH window is seen any more (the L1 has lost the cell). */
static void bsp_bcch_calage(uint32_t tick, unsigned p, bool nb)
{
    static uint8_t arme[7];
    static int tard, tot;
    if (p >= 1 && p <= 6) {
        arme[p] = nb;
        if (nb) g_bcch_vu_tick = tick;
    }
    if (p == 6) {
        bool bon = arme[2] && arme[3] && arme[4] && arme[5] && !arme[6];
        bool retard = !arme[2] && arme[3] && arme[4] && arme[5] && arme[6];
        bool avance = arme[1] && arme[2] && arme[3] && arme[4] && !arme[5];
        if (retard) { tard++; tot = 0; }
        else if (avance) { tot++; tard = 0; }
        else if (bon) {
            tard = tot = 0;
            if (!g_bcch_verrou) printf("  [bcch] tick=%u bloc BCCH dans ses fenetres (2..5) : offset %lld verrouille\n",
                                       tick, (long long)g_ts0_offset);
            g_bcch_verrou = 1;
        }
        if (tard >= 2 || tot >= 2) {
            int d = tard >= 2 ? -1 : +1;
            g_ts0_offset = (g_ts0_offset + d + BSP_FN_MAX) % BSP_FN_MAX;
            if (g_off_mobile != INT64_MIN) g_off_mobile += d;   /* [2026-10-22 14:20] le datage des RACH suit */
            printf("  [bcch] tick=%u fenetres NB sur p51 %s : offset %+d -> %lld\n",
                   tick, d < 0 ? "3..6 (flux en avance)" : "1..4 (flux en retard)", d, (long long)g_ts0_offset);
            tard = tot = 0;
        }
        memset(arme, 0, sizeof arme);
    }
    if (g_bcch_verrou && (uint32_t)(tick - g_bcch_vu_tick) > 160u) {
        g_bcch_verrou = 0;
        printf("  [bcch] tick=%u plus de lecture BCCH depuis %u trames : offset deverrouille\n", tick, tick - g_bcch_vu_tick);
    }
}
/* THE SAME ALIGNMENT ON THE CCCH BLOCKS (paging). In idle mode the L1 hardly re-reads the BCCH:
 * after a dedicated phase (LU, call) the offset could stay wrong by a few frames, every NP_I in
 * error, no page seen (MT SMS, incoming call) until the cell is lost. A series of NB windows must
 * start on a CCCH block start (6, 12, 16, 22, 26, 32, 36, 42, 46); shifted by d (|d| <= 3) twice
 * in a row -> offset - d. The BCCH (2..5) stays with bsp_bcch_calage(). Outside dedicated mode,
 * offset not locked by the BCCH. CALYPSO_BSP_CCCH_CALAGE=0 disables. */
static void bsp_ccch_calage(uint32_t tick, unsigned p, bool nb)
{
    static const unsigned debuts[] = { 6, 12, 16, 22, 26, 32, 36, 42, 46 };
    static int on = -1, prec_p = -1, debut = -1, d_prec = 99, d_n;
    if (on < 0) { const char *e = calypso_getenv("CALYPSO_BSP_CCCH_CALAGE"); on = !(e && *e == '0'); }
    if (!on) return;
    if (nb && debut >= 0 && (int)p == prec_p + 1) { prec_p = (int)p; return; }   /* la serie continue */
    if (debut >= 0 && prec_p - debut + 1 >= 4 && !g_bcch_verrou) {               /* une serie vient de finir */
        int d = 99;
        for (unsigned k = 0; k < sizeof debuts / sizeof debuts[0]; k++) {
            int e = debut - (int)debuts[k];
            if (e >= -3 && e <= 3 && (d == 99 || abs(e) < abs(d))) d = e;
        }
        if (d == 0) d_n = 0, d_prec = 0;
        else if (d != 99) {
            d_n = (d == d_prec) ? d_n + 1 : 1; d_prec = d;
            if (d_n >= 2) {
                g_ts0_offset = (g_ts0_offset - d + BSP_FN_MAX) % BSP_FN_MAX;
                if (g_off_mobile != INT64_MIN) g_off_mobile -= d;   /* le datage des RACH suit */
                printf("  [ccch] tick=%u fenetres NB sur p51 %d.. au lieu de %d.. : offset %+d -> %lld\n",
                       tick, debut, debut - d, -d, (long long)g_ts0_offset);
                d_n = 0; d_prec = 99;
            }
        }
    }
    debut = nb ? (int)p : -1;
    prec_p = nb ? (int)p : -1;
}
static void bsp_ts0_livrer(uint32_t tick_fn, unsigned i)
{
    static int16_t iq[2 * 512];
    const uint8_t *bits = g_ts0[i].bits;
    /* For the [ts0] trace: whether the delivered frame belongs to the
     * dedicated channel, and whether its burst does come from that timeslot.
     * Needed to compare the window geometry of an SDCCH block with that of a
     * BCCH block. */
    bool trame_dediee = false, burst_dedie = false;
    {   /* On the frames of the dedicated channel, its timeslot is what counts. */
        /* Two verdicts: the 45.002 table (channel frames) and what the firmware
         * says (window timeslot). MONTANT_TPU_TN=1 trusts the second; by default
         * the first is kept and the disagreements are LOGGED, because the TICK
         * value may be one frame behind the real programming of the window. */
        /* WITHOUT AN ARMED CHANNEL, NO FRAME IS "OURS". bsp_dedie_trame() reads
         * g_dedie_ss, which is 0 while nothing is armed: outside a connection
         * one frame in eight would otherwise be declared part of the channel,
         * bsp_dedie_bits() would return NULL on its g_dedie_tn <= 0 guard and
         * g_dedie_manques would mix camping time (where playing TS0 is right)
         * with the real holes of the channel. */
        bool a_nous = g_dedie_tn > 0 && bsp_dedie_trame(g_ts0[i].fn);
        {
            static int suit_tpu = -1;
            if (suit_tpu < 0) { const char *e = calypso_getenv("MONTANT_TPU_TN"); suit_tpu = (e && *e == '1'); }
            bool selon_tpu = (bsp_fenetre_tn() == g_dedie_tn);
            if (g_dedie_tn > 0 && selon_tpu != a_nous) {
                static unsigned long n;
                if (++n <= 40 || (n % 200) == 0) {
                    /* printf and not BSP_LOG: CALYPSO_DEBUG=BSP turns on all
                     * the BSP traces and makes the bench lose real time; this
                     * one is already capped (40, then 1 in 200) and serves to
                     * judge the SACCH of the SDCCH/8. */
                    printf("  [BSP] dedie : DESACCORD fn=%u p51=%u p102=%u : table=%d "
                            "tpu=%d (offset=%d ref=%d, tn_fenetre=%d) [%lu fois]\n",
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
        /* SACCH/TF PROBE: on TCH/F the speech and the FACCH follow the
         * 26-multiframe, the SACCH the 104 (even TS: fn%26 == 12, TS2 block at
         * fn%104 = 38, 64, 90, 12). Logs what the BSP plays on those frames. */
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
            /* The bits themselves, to decode the SACCH outside the DSP (A5 +
             * gsm0503_xcch_decode): if it decodes there, the defect is in the
             * BSP/ROM path, otherwise in what the BSP receives.
             * 156-byte records: tick BE32, fn BE32, 148 bits 0/1.
             * 256 bursts at most (64 blocks), file reset at startup. */
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
        /* ALL THE TCH BURSTS, TO DECODE THEM OUTSIDE THE DSP. The SACCH/TF
         * alone (calypso_sacch_tf.bin) says nothing about the speech nor the
         * FACCH. Same format as calypso_sacch_tf.bin (tick BE32, fn BE32,
         * 148 bits 0/1), file reset on every TCH, 20000 bursts at most
         * (~92 s). tools/comparer_parole.py decodes them (A5 + FR/FACCH) and
         * compares them with /dev/shm/calypso_add_dl.bin. */
        if (g_dedie_genre == BSP_DEDIE_TCH && a_nous && d) {
            static FILE *f_tch;
            static unsigned n_tch;
            static uint32_t tick_prec;
            if (!f_tch || (uint32_t)(tick_fn - tick_prec) > 500u) {   /* nouvel appel */
                if (f_tch) fclose(f_tch);
                f_tch = fopen("/dev/shm/calypso_tch_dl.bin", "wb");
                n_tch = 0;
            }
            tick_prec = tick_fn;
            {   /* for the downlink FACCH prosthesis (montant.c) */
                unsigned k = g_ts0[i].fn % TCH_ANNEAU;
                g_tch_anneau[k].fn = g_ts0[i].fn; memcpy(g_tch_anneau[k].bits, d, 148); g_tch_anneau[k].ok = 1;
                g_tch_dernier_fn = g_ts0[i].fn; g_tch_dernier_ok = 1;
            }
            if (f_tch && n_tch < 20000) {
                uint8_t h[8] = { tick_fn >> 24, tick_fn >> 16, tick_fn >> 8, tick_fn,
                                 g_ts0[i].fn >> 24, g_ts0[i].fn >> 16, g_ts0[i].fn >> 8, g_ts0[i].fn };
                fwrite(h, 1, 8, f_tch);
                fwrite(d, 1, 148, f_tch);
                if ((++n_tch % 64) == 0) fflush(f_tch);
            }
        }
        if (d) {
            bits = d;
            burst_dedie = true;
            g_dedie_joues++;
            if (g_dedie_joues <= 40 || g_dedie_joues % 1000 == 0) {
                /* How many 1s in the 148 bits: a filler burst always has the
                 * same number, a real block varies. Enough to see at a glance
                 * whether the BTS really emits the SDCCH. */
                int uns = 0;
                for (int k = 0; k < 148; k++) uns += (d[k] & 1);
                BSP_LOG("dedie : TS%d joue (fn=%u p51=%u uns=%d, %lu fois)",
                        g_dedie_tn, g_ts0[i].fn, g_ts0[i].fn % 51u, uns, g_dedie_joues);
            }
        } else if (a_nous) {
            /* The frame belongs to the channel, but no burst of that timeslot
             * arrived: the block will be incomplete and fail the Fire code. */
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
    const bool armee = calypso_rhea_dma_rx_armed();
    const int page_prog = calypso_rhea_dma_get_len_words() / 2;   /* ALGTH/2 words = samples per page, ONE_SHOT or not */
    const int rif_avant = calypso_rif_level();
    int nwin = one_shot ? page_prog : 0;
    /* CALYPSO_BSP_NB_MARGE: the TI L1 aims at an NB TOA of 3 (l1ctl_toa_update: ISH =
     * estimated_TOA - 16 qbits); with margin 3 the DSP measures 5, hence +8 qbits of tpu_offset
     * every period without end (the bench does not move the burst). Magnetite: 1. */
    static int nb_marge = -1;
    if (nb_marge < 0) { const char *e = calypso_getenv("CALYPSO_BSP_NB_MARGE"); nb_marge = (e && *e) ? atoi(e) : 3;
                        if (nb_marge < 0 || nb_marge > 3) nb_marge = 3; }
    int marge = nwin >= 190 ? 21 : nwin >= 150 ? nb_marge : 0;
    /* fake air: NB window (start of frame) -> synthesizer programmed before 1250 qbits;
     * SB window (end of frame), PM or continuous -> last programming of the frame. */
    bool hors_freq = false;
    if (faux_air_cellule() >= 0) {
        int a = faux_air_arfcn(tick_fn, (nwin >= 150 && nwin < 190) ? 1250 : 5000);
        hors_freq = (a >= 0 && a != faux_air_cellule());
        static unsigned n_hors, n_dans;
        if (hors_freq) n_hors++; else if (a >= 0) n_dans++;
        if (hors_freq && (n_hors <= 20 || n_hors % 5000 == 0))
            printf("  [faux-air] tick=%u fn=%u fenetre=%d accorde sur %d : bruit [%u hors, %u sur la cellule]\n",
                   tick_fn, g_ts0[i].fn, nwin, a, n_hors, n_dans);
        if (hors_freq) sb = fcch = 0;
    }
    /* realistic air: which burst (downlink of this timeslot, uplink of the mobile, or noise) and at which
     * level. A measurement window (< 150) targets the timeslot derived from the synth setting; NB/SB: TS0. */
    int amp_ts0 = (one_shot && nwin >= 150 && nwin < 190) ? nb_amp(tick_fn) : 30000;
    const uint8_t *bits_mod = bits;
    if (air_niveaux_on() && !hors_freq) {
        unsigned pos = (one_shot && nwin > 0 && nwin < 150) ? air_pos_fenetre(tick_fn) : 0;
        const uint8_t *dl = pos ? bsp_autres_bits(g_ts0[i].fn, pos) : bits;
        bits_mod = air_choisir(tick_fn, (nwin >= 150 && nwin < 190) ? 1250 : 5000, g_ts0[i].fn, pos, dl, &amp_ts0);
        if (!bits_mod) { hors_freq = true; bits_mod = bits; }
        if (bits_mod != bits || hors_freq) sb = fcch = 0;
    }
    /* An SCH is memorised here (g_dernier_sch); if the SB window only arms
     * after this deposit, calypso_bsp_sb_retenter() will deliver it there. */
    const char *sb_action = "cadree comme un burst normal";
    if (sb && sb_fenetre_on()) {
        g_dernier_sch.fn = g_ts0[i].fn; g_dernier_sch.tick = tick_fn;
        memcpy(g_dernier_sch.bits, bits, 148);
        g_dernier_sch.livre_sb = (one_shot && nwin >= 190);
        g_dernier_sch.valide = 1;
        if (!armee) sb_action = "DMA NON ARMEE (le RIF jette) : SCH memorise pour la fenetre SB";
        else if (!(one_shot && nwin >= 190)) sb_action = "autre fenetre (PM/NB/continue) : SCH memorise pour la fenetre SB";
    }
    /* TRX mode: the uplink NB burst played here is memorised; at phase A of the next frame,
     * calypso_bsp_trx_ul_prothese() writes its exact bits into the result page of the extcode
     * (see the function). */
    if (g_trx_cale && !sb && !fcch && !bsp_ts0_est_rach(bits)) {
        g_trx_ul.fn = g_ts0[i].fn; g_trx_ul.tick = tick_fn; memcpy(g_trx_ul.bits, bits, 148); g_trx_ul.valide = 1;
    }
    /* TI dedicated mode (routing, see calypso_bsp_nb_retenter): the NB is not armed at the deposit ->
     * the burst goes into the keep ring, and an armed measurement window receives filler instead */
    dma_trace("depot");
    bool devie = false;
    /* NEVER IN CONTINUOUS DMA (FB search). An NB of the FB search sent to the ring without delivering
     * anything (neither TS0 nor the seven filler timeslots) would make the ROM count only the FCCH/SCH
     * ticks (FB TOA ~1255 instead of ~8 frames) and the osmocom firmware would place its SB window 7
     * frames too early (FBSB result=255 in a loop). See the full frame below. */
    const bool continue_fb = armee && !one_shot;
    if (!sb && !fcch && !hors_freq && !continue_fb && nb_retenter_on() && !(one_shot && nwin >= 150 && nwin < 190)) {
        nb_garde_poser(g_ts0[i].fn, tick_fn, bits);
        if (one_shot && nwin > 0 && nwin < 150) pm_servir(nwin);
        devie = true;
    }
    if (hors_freq) { if (air_niveaux_on()) air_bruit(iq + 2 * marge, 148); else faux_air_bruit(iq + 2 * marge, 148); }
    else if (sb) {
        sb_moduler(bits, iq + 2 * marge, g_ts0[i].fn);   /* see sb_moduler() */
        if (air_niveaux_on()) air_echelle(iq + 2 * marge, 148, amp_ts0);   /* same level as the cell */
    }
    else    gmsk_moduler(bits_mod, 148, amp_ts0, 0.0, 0.5, iq + 2 * marge);
    if (!sb && !fcch && !hors_freq) {
        static double sym = -2;
        if (sym == -2) { const char *e = calypso_getenv("CALYPSO_BSP_NB_SYM"); sym = (e && *e) ? atof(e) : 0.3; }
        gmsk_elargir(iq + 2 * marge, 148, sym);
    }
    int total = marge > 0 ? (nwin > marge + 148 ? nwin : marge + 148) : 148;
    if (total > 256) total = 256;
    if (sb) {
        /* PROBE: does the SB land in a REAL SB window? The alignment of
         * g_ts0_offset and the 21-sample margin require nwin >= 190 (382-word
         * window). Otherwise the SB is framed like a normal burst (margin 3)
         * and the alignment does not happen through this path. Both cases are
         * counted. */
        static unsigned n_sb, n_sb_fenetre;
        n_sb++;
        if (one_shot && nwin >= 190) n_sb_fenetre++;
        if (one_shot && nwin >= 190) sb_action = "FENETRE SB (one-shot)";
        /* All the SCHs of the first 200, then 1 in 10: it must be visible on
         * every resync. */
        if (n_sb <= 200 || n_sb % 10 == 0 || (int32_t)(g_sb_trace_jusqua - tick_fn) > 0) {
            printf("  [sbwin] SB #%u fn=%u p51=%u tick=%u : armee=%d one_shot=%d page_prog=%d rif_avant=%d nwin=%d marge=%d -> %s (dans une vraie fenetre SB : %u/%u)\n",
                   n_sb, g_ts0[i].fn, g_ts0[i].fn % 51u, tick_fn, (int)armee, (int)one_shot, page_prog, rif_avant, nwin, marge,
                   sb_action, n_sb_fenetre, n_sb);
        }
    }
    if (sb && one_shot && nwin >= 190) {
        int64_t off = (int64_t)g_ts0[i].fn - (int64_t)tick_fn - sb_fin_trame();
        if (!g_off_rac_vu) g_off_mobile = (int64_t)g_ts0[i].fn - (int64_t)tick_fn;   /* [2026-10-22] voir sb_retenter */
        if (off != g_ts0_offset) printf("  [ts0] SB fn=%u livree au tick %u dans une fenetre SB : offset ARM-tick = %lld%s\n",
                                        g_ts0[i].fn, tick_fn, (long long)off, g_ts0_offset == INT64_MIN ? " (premier calage)" : " (recalage)");
        if (!g_trx_cale && !g_bcch_verrou) g_ts0_offset = off;
    }
    /* One line per delivered frame: the window alignment probe (nwin > 0 is
     * true for EVERY normal burst with the one-shot DMA, so it would print
     * continuously). Behind CALYPSO_BSP_TS0_DEBUG=1; by default, the first 20
     * deliveries then one in 5000, enough to see that the stream runs without
     * flooding the console. */
    { static int ts0_dbg = -1;
      if (ts0_dbg < 0) { const char *e = calypso_getenv("CALYPSO_BSP_TS0_DEBUG"); ts0_dbg = (e && *e && *e != '0'); }
      static unsigned nl, nd;
      /* The frames of the dedicated channel are rare (4 in 51, plus the SACCH)
       * and only last for a connection: ALL of them are traced, up to 300,
       * independently of CALYPSO_BSP_TS0_DEBUG. It is the only way to compare
       * their window and margin with those of a BCCH block. */
      bool trace = ts0_dbg ? (nl < 400 || nl % 5000 == 0 || fcch || sb || nwin > 0)
                           : (nl < 20 || nl % 5000 == 0);
      nl++;
      if (trame_dediee && nd < 300) { trace = true; nd++; }
      /* The first 400 normal BCCH frames (p51 2..5) tell which DMA window the ROM had
       * armed when the burst arrived (diagnoses an SB decoded with every BCCH block in
       * CRC KO, BS_I error_flag=1). */
      if (sb_fin_trame() && g_dedie_tn <= 0)
          bsp_bcch_calage(tick_fn, g_ts0[i].fn % 51u, nwin >= 150 && nwin < 190);
      if (sb_fin_trame() && g_dedie_tn <= 0 && !g_dedie_ti)
          bsp_ccch_calage(tick_fn, g_ts0[i].fn % 51u, nwin >= 150 && nwin < 190);
      { static unsigned nbcch;
        const unsigned p = g_ts0[i].fn % 51u;
        if (p >= 2 && p <= 5 && nbcch < 400) { trace = true; nbcch++; } }
      if (trace) printf("  [ts0] tick=%u fn=%u p51=%u %s fenetre=%d marge=%d rif_avant=%d%s\n",
                        tick_fn, g_ts0[i].fn, g_ts0[i].fn % 51u,
                        sb ? "SB" : fcch ? "FCCH" : "NB", nwin, marge, calypso_rif_level(),
                        trame_dediee ? (burst_dedie ? "  <- canal dedie" : "  <- canal dedie, BURST MANQUANT") : ""); }
    g_ts0[i].joue = 1;
    if (devie) return;                       /* garde / remplissage faits ci-dessus */
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
            int amp_autre = 30000;
            if (air_niveaux_on()) {   /* [2026-10-23] air realiste, voir air_choisir() */
                b = air_choisir(tick_fn, 5000, g_ts0[i].fn, (unsigned)tn, b, &amp_autre);
                if (!b) {
                    static int16_t bruit_autre[2 * 148];
                    air_bruit(bruit_autre, 148);
                    calypso_bsp_rx_burst((uint8_t)tn, g_ts0[i].fn, bruit_autre, 2 * 148);
                    continue;
                }
            }
            if (!b) {
                calypso_bsp_rx_burst((uint8_t)tn, g_ts0[i].fn, bsp_remplissage(), 2 * 148);
                continue;
            }
            memset(iq_autre, 0, sizeof iq_autre);
            gmsk_moduler(b, 148, amp_autre, 0.0, 0.5, iq_autre);
            {   /* same widening as the normal bursts of TS0 */
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
/* THE BENCH CLOCK IS THE DSP'S, NOT THE WALL'S.
 *
 * bsp_ts0_service() plays BTS frame `tick + g_ts0_offset`. The ARM tick
 * advances at the pace of the emulated C54x (~6.7 ms per frame) while the BTS
 * runs in real time (4.615 ms). The DSP therefore consumes the ring 20 %
 * slower than it fills, and the lag is bounded by nothing. Catching up on the
 * DSP side is impossible (it already runs flat out), so the BTS must slow
 * down: the frame the DSP requests is published here, and pont.py
 * (pont/trx.py, class Clock) brakes its clock -- hence the BTS's IND CLOCK --
 * so as never to run ahead by more than a few frames.
 *
 * Format, 16 bytes: seq (non-zero, incremented on every write), cale (0 before
 * the first alignment on SB: the clock stays free), requested BTS frame, ARM
 * tick. CALYPSO_BSP_HORLOGE=0 disables the publication. */
#define SHM_HORLOGE "/dev/shm/calypso_horloge"
/* BTS frame of a tick, from the alignment of the TS0 stream (SB, then BCCH lock): the frame the
 * phone's L1 believes it lives in at this tick. pont.c (tx_tsp_fn) dates the uplink bursts with it
 * when the firmware does not set a_a5fn (TI L1: the RACH would otherwise leave as a "bare TICK" and
 * the air would not place it). 0 if not aligned yet. */
int calypso_bsp_fn_bts_du_tick(uint32_t tick, uint32_t *fn_bts)
{
    if (g_ts0_offset == INT64_MIN) return 0;
    int64_t w = ((int64_t)tick + g_ts0_offset) % (int64_t)BSP_FN_MAX;
    if (w < 0) w += BSP_FN_MAX;
    *fn_bts = (uint32_t)w;
    return 1;
}
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
        /* nothing to do: the free clock takes over on the bridge side */
    }
}

/* THE SECOND CHANCE. The firmware posts two SB tasks on two consecutive
 * frames; the first falls on the FCCH frame (null burst), empty window, chance
 * lost; the second on the SCH. The BSP already has the SCH of the next frame
 * in its ring (the BTS is ahead). It is therefore delivered in the window of
 * attempt 1, in waveform D, then the ROM's result is read in the API RAM at
 * the next tick (a_sch R page: B_BLUD without B_SCH_CRC, T2 <= 25, T3' <= 4):
 *   - decoded: the firmware set its clock on fn=SCH at that tick; the ARM-tick
 *     offset is realigned by one frame (fn_sch - tick), the frame of the next
 *     tick is fn_sch+1, the stream stays contiguous (the SCH itself is not
 *     replayed: no need);
 *   - missed: nothing changes, the native SCH goes out in attempt 2
 *     (waveform A).
 * Replay: A alone misses 20 %, A+D in common 7 %. CALYPSO_BSP_SB_DOUBLE=0
 * disables. */
static struct { int actif; uint32_t fn_sch; uint32_t tick; uint16_t a0[2]; unsigned n, ok, ko; } g_sc;
static int sb_double_on(void)
{
    static int on = -1;
    /* DISABLED BY DEFAULT. Every decode in attempt 1 advances by ONE frame the
     * index the DSP requests from the BTS; the bridge's DL margin cannot
     * rebuild itself (the BTS does not go faster than real time), and long
     * calls then end in "LOS during dedicated mode" with a downlink TCH that
     * degrades progressively (a_dd err, FIRE). CALYPSO_BSP_SB_DOUBLE=1 enables. */
    if (on < 0) { const char *e = calypso_getenv("CALYPSO_BSP_SB_DOUBLE"); on = (e && *e == '1') ? 1 : 0;
                  if (on) BSP_LOG("SB_DOUBLE on : le SCH suivant est aussi livre (forme D) dans la fenetre de la tentative 1"); }
    return on;
}
/* a_sch[0..4] of R page pg: words (0x50|0x78 + 0x1E)/2 of the API RAM (calypso_api.h) */
static const uint16_t *sc_a_sch(int pg)
{
    if (!bsp.dsp || !bsp.dsp->api_ram) return NULL;
    return &bsp.dsp->api_ram[((pg ? 0x78u : 0x50u) + 0x1Eu) / 2];
}
static int sc_rom_a_decode(void)
{
    for (int pg = 0; pg < 2; pg++) {
        const uint16_t *a = sc_a_sch(pg);
        if (!a || a[0] == g_sc.a0[pg]) continue;
        if ((a[0] & 0x8100) != 0x8000) continue;               /* B_BLUD sans B_SCH_CRC */
        uint32_t sb = (uint32_t)a[3] | ((uint32_t)a[4] << 16);
        unsigned t2 = (sb >> 18) & 0x1f, t3p = ((sb >> 24) & 1) | ((sb >> 15) & 6);
        if (t2 <= 25 && t3p <= 4) return 1;
    }
    return 0;
}
static void bsp_ts0_service(uint32_t tick_fn)
{
    /* phone 2 of the TRX (TS1): it emits no SB, the fn/tick offset is given
     * (CALYPSO_BSP_TRX_OFFSET = firmware fn - tick, -1 measured on phone 1) + delta */
    static int fixe = -1;
    if (fixe < 0) {
        const char *e = calypso_getenv("CALYPSO_BSP_TRX_OFFSET");
        fixe = (e && *e) ? 1 : 0;
        if (fixe) {
            const char *dd = calypso_getenv("CALYPSO_BSP_TRX_DELTA"); int delta = (dd && *dd) ? atoi(dd) : -3;
            int64_t off = ((int64_t)atoi(e) + delta) % (int64_t)BSP_FN_MAX; if (off < 0) off += BSP_FN_MAX;
            g_ts0_offset = off; g_trx_cale = 1;
            printf("  [ts0] mode TRX : offset fn-tick FIXE %lld (CALYPSO_BSP_TRX_OFFSET=%s, delta %d)\n", (long long)off, e, delta);
        }
    }
    if (!g_ts0 || !g_ts0_any) return;
    g_tick_courant = tick_fn;
    /* second chance: at the next tick, the ROM's verdict decides the clock */
    if (g_sc.actif) {
        if (tick_fn == g_sc.tick + 1) {
            if (sc_rom_a_decode()) {
                int64_t off = (int64_t)g_sc.fn_sch - (int64_t)g_sc.tick - sb_fin_trame();
                if (off < 0) off += BSP_FN_MAX;
                g_sc.ok++; if (!g_trx_cale && !g_bcch_verrou) g_ts0_offset = off;
                if (g_sc.ok <= 300 || g_sc.ok % 50 == 0)
                    printf("  [sbwin] SECONDE CHANCE : SB fn=%u DECODE a la tentative 1 (tick %u), offset ARM-tick recale [%u/%u]\n",
                           g_sc.fn_sch, g_sc.tick, g_sc.ok, g_sc.n);
            } else {
                g_sc.ko++;
                if (g_sc.ko <= 300 || g_sc.ko % 50 == 0)
                    printf("  [sbwin] SECONDE CHANCE : SB fn=%u rate a la tentative 1, SCH natif en tentative 2 [%u/%u]\n",
                           g_sc.fn_sch, g_sc.ok, g_sc.n);
            }
        }
        g_sc.actif = 0;
    }
    /* An SCH kept at the previous tick is delivered in the SB window that has
     * just armed; it realigns the ARM-tick offset, so the frame this tick would
     * have played will be played at the next tick: the stream stays contiguous. */
    if (calypso_bsp_sb_retenter()) return;
    static unsigned manques, manques_log;
    if (g_ts0_offset != INT64_MIN) {
        int64_t w = ((int64_t)tick_fn + g_ts0_offset) % (int64_t)BSP_FN_MAX; if (w < 0) w += BSP_FN_MAX;
        /* CATCHING UP WITH THE LIVE STREAM (CALYPSO_BSP_RATTRAPE=1, Magnetite). At boot the instance
         * falls behind the BTS (DSP restarts by the firmware, ARM lock waiting up to 5 s per frame)
         * and never catches up on its own: uplink RACHs dated in the past are never played. Beyond
         * the threshold of lateness relative to the last received frame, the offset jumps to the live
         * stream (margin, see below) and the BCCH lock drops: the L1 sees a discontinuity and
         * resynchronises on the live stream. */
        {
            /* margin 2 and threshold 30: the uplink only has ~2 frames of play at the TRX
             * (BTS_RX_LAG); with 20 to 200 frames of lateness the RACHs arrive after their frame
             * and are never played. */
            static int rattrape = -1, marge_r = 2, seuil_r = 30;
            if (rattrape < 0) { const char *e = calypso_getenv("CALYPSO_BSP_RATTRAPE"); rattrape = (e && *e == '1');
                                e = calypso_getenv("CALYPSO_BSP_RATTRAPE_MARGE"); if (e && *e) marge_r = atoi(e);
                                e = calypso_getenv("CALYPSO_BSP_RATTRAPE_SEUIL"); if (e && *e) seuil_r = atoi(e);
                                if (seuil_r <= marge_r) seuil_r = marge_r + 1; }
            int32_t retard = (int32_t)(g_ts0_fn_max - (uint32_t)w);
            if (rattrape && g_ts0_any && !g_trx_cale && retard > seuil_r) {
                int64_t saut = retard - marge_r;
                g_ts0_offset = (g_ts0_offset + saut) % (int64_t)BSP_FN_MAX;
                g_bcch_verrou = 0;
                w = ((int64_t)tick_fn + g_ts0_offset) % (int64_t)BSP_FN_MAX; if (w < 0) w += BSP_FN_MAX;
                printf("  [ts0] RATTRAPAGE : %d trames de retard sur le BTS (derniere recue fn=%u) -> offset %+lld, livre fn=%lld\n",
                       retard, g_ts0_fn_max, (long long)saut, (long long)w);
            }
        }
        /* SHIFT IN DEDICATED MODE (CALYPSO_BSP_DEDIE_DECAL=<n>, Magnetite). The offset is locked on
         * the BCCH reading (which can differ by 1-2 frames from the SBs); the shift lets the TI L1
         * decode the SACCH (SA_I) and the SDCCH (UA) in dedicated mode. Applied only during the
         * dedicated tasks signalled by montant.c (calypso_bsp_set_dedie_ti). */
        {
            static int decal = -999;
            if (decal == -999) { const char *e = calypso_getenv("CALYPSO_BSP_DEDIE_DECAL"); decal = (e && *e) ? atoi(e) : 0;
                                 if (decal) printf("  [ts0] decalage en dedie TI : %+d trame(s)\n", decal); }
            if (decal && g_dedie_ti) { w = (w + decal) % (int64_t)BSP_FN_MAX; if (w < 0) w += BSP_FN_MAX; }
        }
        /* TCH ONLY: CALYPSO_BSP_TCH_DECAL=<n> frames on the BCCH alignment. On SDCCH the DSP's A5 falls
         * on the delivered burst (+1 structural, [a5] "gap +1"); on TCH "gap +2": one frame too many,
         * downlink FACCH (UA) in Fire KO, RR Assignment Failure. */
        {
            int tch_decal = calypso_reglage("CALYPSO_BSP_TCH_DECAL", 0);   /* [2026-10-22] a chaud */
            if (tch_decal && g_dedie_tn > 0 && g_dedie_genre == BSP_DEDIE_TCH) {
                w = (w + tch_decal) % (int64_t)BSP_FN_MAX; if (w < 0) w += BSP_FN_MAX;
            }
        }
        /* IN TI DEDICATED MODE, THE MOBILE'S FRAME. The TI L1 writes a_a5fn = the fn it processes
         * (pont.c -> g_off_mobile); the BSP offset, aligned on the BCCH, can be 1 to 2 frames behind it
         * on TCH (shifted FACCH/TCH deinterleaving, A5 at the wrong fn -> every a_fd in Fire KO,
         * ciphered or not, and Assignment Failure). While a_a5fn has spoken less than 200 frames ago,
         * tick + g_off_mobile is played. CALYPSO_BSP_DEDIE_A5FN=0 falls back to the BCCH offset. */
        {
            static int a5fn_on = -1;
            /* OFF BY DEFAULT: the BCCH alignment already gives the mobile's frame in dedicated mode
             * (SDCCH 100 %, TCH d_fn aligned); this alignment can jump by +7 frames (noisy RA_C offset)
             * and leave idle mode misaligned after the dedicated phase (every NP_I in error, incoming
             * call never paged). CALYPSO_BSP_DEDIE_A5FN=1 enables. */
            if (a5fn_on < 0) { const char *e = calypso_getenv("CALYPSO_BSP_DEDIE_A5FN"); a5fn_on = (e && *e == '1'); }
            if (a5fn_on && g_dedie_ti && g_a5fn_vu && g_off_mobile != INT64_MIN &&
                (uint32_t)(tick_fn - g_a5fn_tick) < 3000) {
                /* RA_C reports the frame FOLLOWING the transmission: in dedicated mode the mobile opens
                 * its SDCCH/8 SS3 windows at (tick + RA_C offset) % 51 = 13..16 / 45..48 instead of
                 * 12..15 / 44..47 -> one frame less for reception (CALYPSO_BSP_DEDIE_DL_DECAL, default -1). */
                static int dl_decal = 99;
                if (dl_decal == 99) { const char *e = calypso_getenv("CALYPSO_BSP_DEDIE_DL_DECAL"); dl_decal = (e && *e) ? atoi(e) : -1; }
                /* the TCH has its own setting (CALYPSO_BSP_TCH_DL_DECAL, default = the SDCCH one):
                 * TCH [a5]: deciphered fn = last deposit + 2 with -1 */
                static int tch_decal = 99;
                if (tch_decal == 99) { const char *e = calypso_getenv("CALYPSO_BSP_TCH_DL_DECAL"); tch_decal = (e && *e) ? atoi(e) : dl_decal; }
                int dec = g_dedie_genre == BSP_DEDIE_TCH ? tch_decal : dl_decal;
                int64_t wm = ((int64_t)tick_fn + g_off_mobile + (g_off_rac_vu ? dec : 0)) % (int64_t)BSP_FN_MAX; if (wm < 0) wm += BSP_FN_MAX;
                static unsigned n_dm;
                if (wm != w && (n_dm++ < 20 || n_dm % 2000 == 0))
                    printf("  [dedie-a5fn] tick=%u trame BCCH %lld -> trame du mobile %lld (%+lld) [%u]\n", tick_fn,
                           (long long)w, (long long)wm, (long long)(wm - w), n_dm);
                w = wm;
            }
        }
        unsigned i = (unsigned)w % BSP_TS0_RING;
        bsp_horloge_publier((uint32_t)w, tick_fn, 1);
        /* SB_FIN_TRAME, offset LOCKED by the BCCH reading: the TI L1 opens its SB window at the
         * end of the frame preceding the SB; with the stream aligned on the NBs, the SCH would
         * arrive at the next tick, window closed (every camping SB re-read missed: SB_I flag 0,
         * NET_I 0 -> cell abandoned). An SB window armed during frame w therefore receives SCH
         * w+1, without realignment (lock). Outside the lock (resync), reading ahead would skew
         * the alignment. */
        if (sb_fin_trame() && g_bcch_verrou && g_dedie_tn <= 0 && calypso_rhea_dma_rx_armed() &&
            calypso_rhea_dma_one_shot() && calypso_rhea_dma_get_len_words() / 2 >= 190) {
            uint32_t fn1 = (uint32_t)((w + 1) % BSP_FN_MAX);
            unsigned j = fn1 % BSP_TS0_RING;
            if (g_ts0[j].valid && g_ts0[j].fn == fn1 && bsp_ts0_est_sb(g_ts0[j].bits)) {
                static unsigned n_avance;
                if (++n_avance <= 20 || n_avance % 500 == 0)
                    printf("  [sbwin] SB_FIN_TRAME (verrou) : fenetre SB au tick %u (trame %u) -> SCH fn=%u [%u]\n",
                           tick_fn, (uint32_t)w, fn1, n_avance);
                bsp_ts0_livrer(tick_fn, j);
                g_dernier_sch.valide = 0;
                g_ts0[i].joue = 1;
                return;
            }
        }
        if (g_ts0[i].valid && g_ts0[i].fn == (uint32_t)w && sb_double_on() && g_dedie_tn <= 0 &&
            bsp_ts0_est_fcch(g_ts0[i].bits) && calypso_rhea_dma_rx_armed() && calypso_rhea_dma_one_shot() &&
            calypso_rhea_dma_get_len_words() / 2 >= 190) {
            unsigned j = (unsigned)((w + 1) % BSP_FN_MAX) % BSP_TS0_RING;
            if (g_ts0[j].valid && g_ts0[j].fn == (uint32_t)((w + 1) % BSP_FN_MAX) && bsp_ts0_est_sb(g_ts0[j].bits)) {
                for (int pg = 0; pg < 2; pg++) { const uint16_t *a = sc_a_sch(pg); g_sc.a0[pg] = a ? a[0] : 0; }
                /* bsp_ts0_livrer() realigns the offset as soon as an SCH goes out in
                 * an SB window; here it must NOT: if the ROM misses, attempt 2 must see
                 * the native SCH at the next tick (otherwise the next frame would go out
                 * in its place and D would replace A instead of adding to it). The
                 * realignment only happens at the next tick, on CRC OK read in the API
                 * RAM. */
                int64_t off_avant = g_ts0_offset;
                g_sb_variante = 1;
                bsp_ts0_livrer(tick_fn, j);           /* the next SCH, waveform D, in the window of attempt 1 */
                g_sb_variante = 0;
                if (!g_trx_cale) g_ts0_offset = off_avant;
                g_ts0[i].joue = 1;                    /* the FCCH frame is not played: the window was for the SB */
                g_sc.actif = 1; g_sc.fn_sch = g_ts0[j].fn; g_sc.tick = tick_fn; g_sc.n++;
                if (g_sc.n <= 300 || g_sc.n % 50 == 0)
                    printf("  [sbwin] SECONDE CHANCE : trame FCCH fn=%u au tick %u, fenetre SB armee -> SCH fn=%u livre (forme D)\n",
                           (uint32_t)w, tick_fn, g_ts0[j].fn);
                return;
            }
        }
        if (g_ts0[i].valid && g_ts0[i].fn == (uint32_t)w) { bsp_ts0_livrer(tick_fn, i); return; }
        /* THE DSP RUNS AHEAD OF THE BTS: STEP BACK (optional).
         *
         * osmo-bts-trx beats its frames on a timerfd hard-set to 4615 us and
         * cannot follow a slower clock -- its drift filter is an empty TODO
         * (scheduler_trx.c:571). When the DSP runs slower than real time the
         * BTS believes it is "faster than the TRX", compensates, then
         * resynchronises -- and produces NOTHING meanwhile. The DSP keeps
         * ticking and requests frames that do not exist yet.
         *
         * This path steps the offset back to restart from the most recent
         * received frame: the DSP falls behind wall time but its stream stays
         * CONTIGUOUS (STREAM mode: only the ORDER matters, the SCH carries the
         * real FN for synchronisation). Holes become latency.
         *
         * DISABLED BY DEFAULT: replacing a missing frame with the most recent
         * available one does not deliver a hole but THE WRONG BURST AT THE
         * RIGHT PLACE. For the deinterleaver a wrong but plausible input is
         * worse than an absence: it can no longer treat it as an erasure
         * (A/B: twice as many frames dropped by the mobile). CALYPSO_BSP_RECALE=1
         * enables it. This path exits before `manques++`, so it blinds the
         * "manques" counter. */
        if ((int32_t)((uint32_t)w - g_ts0_fn_max) > 0) {
            static int recale = -1;
            if (recale < 0) { const char *e = calypso_getenv("CALYPSO_BSP_RECALE");
                              recale = (e && *e == '1') ? 1 : 0;   /* OFF by default: see above */
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
            /* Dedicated channel frame skipped entirely: out of reach of
             * g_dedie_manques, which is only touched in bsp_ts0_livrer(). */
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
        /* A MISSING FRAME MUST BE AN ERASURE, NOT THE PREVIOUS FRAME REPLAYED.
         *
         * Returning here without depositing anything does not "hole" the
         * stream: the API page KEEPS the burst of the previous tick, and the
         * DSP decoding task runs on it anyway. Verified chain:
         *   - calypso_bsp_rx_burst() is the only writer of the burst DARAM, and
         *     is only called from bsp_ts0_livrer() -- which this path skips;
         *   - calypso_rif_drain() returns 0 on an empty FIFO (calypso_rif.c);
         *   - the transfer then exits through `if (got <= 0) break`
         *     (calypso_rhea_dma.c) WITHOUT writing anything into the page;
         *   - on the DRR path it is worse, the last word is repeated: "On an
         *     empty FIFO, DRR keeps its last value [...] returning 0 would
         *     fabricate a sample" (calypso_rif.c).
         * A decoder re-reading the same page twice yields blocks with an
         * IDENTICAL error count; null samples instead give soft bits close to
         * zero, the only input the deinterleaver can treat as uncertainty
         * rather than data.
         *
         * Touches no delivered frame: this block ONLY runs on the ticks where
         * the DSP would otherwise re-read stale samples. It does not lower
         * `perdues`, which serves as a no-effect witness.
         * CALYPSO_BSP_SILENCE=0 disables the erasure. */
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
                if (bsp_remplissage() != (const int16_t *)g_remplissage_ts0)
                    faux_air_bruit(vide, total);              /* background noise, not zeros (PM = 0) */
                calypso_bsp_rx_burst(0, (uint32_t)w, vide, 2 * total);
                bsp.bursts_written++;
                /* In continuous mode the frame must be 1250 samples, otherwise
                 * the ROM's symbol counter drifts: same fillers as
                 * bsp_ts0_livrer(). */
                if (!one_shot) {
                    for (int tn = 1; tn < 8; tn++) {
                        calypso_bsp_rx_burst((uint8_t)tn, (uint32_t)w, bsp_remplissage(), 2 * 148);
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
    /* uplink copied by the bridge (byte 6 = 0xA5): stored apart for the realistic air, never played
     * as a downlink burst (it would break the sync), whether CALYPSO_AIR_NIVEAUX is set or not. */
    if (n >= 8 + 148 && buf[6] == 0xA5) {
        air_ul_stocker(fn, tn, buf + 8);
        bsp.bursts_seen++;   /* the drain loop of calypso_bsp_service() stops when this no longer moves */
        return;
    }
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
                /* The dedicated channel: kept, without changing the pacing -
                 * bsp_ts0_livrer() will play it instead of TS0 when the window
                 * is one-shot. */
                bsp_dedie_stocker(fn, bits);
            }
            if (tn >= 1 && tn <= 7 && nbits == 148) {
                /* The other timeslots of the SAME frame: they take the place of
                 * the zero filler in the continuous frame. */
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
        /* the SCH goes through sb_moduler() (instant, widening, noise tuned in
         * replay); FCCH as is; NB widened (CALYPSO_BSP_NB_SYM). */
        int est_sb = 0;
        if (nbits == 148) {
            static const uint8_t train_sb[64] = { 1,0,1,1,1,0,0,1,0,1,1,0,0,0,1,0,0,0,0,0,0,1,0,0,0,0,0,0,1,1,1,1, 0,0,1,0,1,1,0,1,0,1,0,0,0,1,0,1,0,1,1,1,0,1,1,0,0,0,0,1,1,0,1,1, };
            est_sb = 1;
            for (int k = 0; k < 64 && est_sb; k++) if ((bits[42 + k] & 1) != train_sb[k]) est_sb = 0;
        }
        /* realistic air also on this direct path (without CALYPSO_BSP_STREAM: banc-rssi) */
        int amp_dir = 30000, bruit_dir = 0;
        if (air_niveaux_on() && nbits == 148) {
            const uint8_t *b2 = air_choisir(g_tick_courant, 5000, fn, tn, bits, &amp_dir);
            if (!b2) bruit_dir = 1;
            else if (b2 != bits) { bits = b2; est_sb = 0; }
        }
        if (bruit_dir) air_bruit(iq + iq_count, nbits);
        else if (est_sb) {
            /* the SCH at the same level as the rest of the cell: at 30000 among bursts at 3000, it
             * would saturate after the gain setting on the FB (FB1 -71 dBm then "Sync error") */
            sb_moduler(bits, iq + iq_count, fn);
            air_echelle(iq + iq_count, nbits, amp_dir);
        }
        else {
            gmsk_moduler(bits, nbits, amp_dir, 0.0, 0.5, iq + iq_count);
            if (nbits == 148) {
                static double sym = -2;
                if (sym == -2) { const char *e = calypso_getenv("CALYPSO_BSP_NB_SYM"); sym = (e && *e) ? atof(e) : 0.3; }
                int zeros = 1;
                for (int k = 0; k < 148 && zeros; k++) if (bits[k]) zeros = 0;
                if (!zeros) gmsk_elargir(iq + iq_count, nbits, sym);
            }
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
        /* THE BIAS OF THE TOA LOOP IS APPLIED HERE.
         * calypso_bsp_toa_feedback() (below) measures the gap between the TOA
         * seen by the ROM and the target 23, and integrates it into
         * g_toa_bias. ONLY the SB window (nwin >= 190) is corrected. The
         * normal burst keeps its margin of 3, which gives it the expected and
         * measured TOA 5, and must not be touched. */
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
        g_tpu_ref = qbits;          /* no dedicated channel: the ARM is on TS0 */
    }
}

/* Native ToA lock (CALYPSO_BSP_TOA_LOCK=1): slow closed loop driving the burst
 * placement bias so the ToA the DSP measures reaches 23 ("on time") WITHOUT
 * canning the output. The firmware then sees the alignment and stops
 * correcting. One-sample-per-frame integrator, for stability. */
/* defined above, before the framing that applies it */
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

/* WAIT FOR THE LATE FRAME RATHER THAN HOLE IT.
 *
 * osmo-bts-trx beats its frames on its own timer and realigns on every
 * IND CLOCK of the bridge (every 51 frames): it stalls, then catches up by a
 * block. The frames of a given multiframe phase can thus arrive AFTER the DSP
 * requests them, and bsp_ts0_service() would replace them by an erasure (the
 * mobile's downlink block, fn%51 = 16..19, lost one time in two; the UA never
 * gets through).
 *
 * The BTS is slaved to the DSP (pont/dsp/clock.py): waiting a few
 * milliseconds for it to deliver creates no drift, it aligns the DSP on it.
 * The wait only happens if the requested frame is AHEAD of the last received
 * one (the BTS is late, the frame is about to arrive) and by less than 200
 * frames; otherwise (burst gone, or alignment lost) no wait.
 * CALYPSO_BSP_ATTENTE_MS: 0 (default, QEMU unchanged) = no wait;
 * c54x_exe/run.sh sets 40 in MODE=dsp. */
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
    /* TRX mode (g_trx_cale): the uplink bursts are RARE (RACH, then 4 bursts per SDCCH block);
     * waiting 40 ms on every frame without burst would choke the lock-step DSP/ARM pace from the
     * first received burst (the trx firmware would program nothing more: 121 insn/frame, clock lost).
     * A frame without burst is a silence. */
    if (max_ms <= 0 || !g_ts0 || !g_ts0_any || g_ts0_offset == INT64_MIN || bsp.trxd_fd < 0 || g_trx_cale) {
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

/* TCH RECORDER, to replay off the bench EXACTLY what the DSP received
 * (tools/rejeu_banc.c of c54x_exe). Every I/Q delivery during a TCH: 'B',
 * tick BE32, tn, fn BE32, one_shot, nwin BE16, n_int16 BE16, then n_int16
 * native int16 samples. The ARM writes into the API RAM are recorded next to
 * it by pont.c ('A'). 60000 deliveries at most, from the arming of the
 * dedicated channel (SDCCH included); CALYPSO_REJEU_ENREG=0 disables. */
uint32_t calypso_trx_get_fn(void);
static FILE *g_enreg_f;
static unsigned long g_enreg_n;
FILE *calypso_bsp_enreg_fichier(void)
{
    static int on = -1;
    /* CALYPSO_REJEU_ENREG=tch: record only the TCH (otherwise the cap can be
     * reached by the SDCCHs at the start of the run, before the call);
     * CALYPSO_REJEU_ENREG_MAX=<n>: delivery cap (60000). */
    static int tch_seul = 0; static unsigned long plafond = 60000;
    if (on < 0) { const char *e = calypso_getenv("CALYPSO_REJEU_ENREG"); on = !(e && *e == '0');
                  tch_seul = (e && !strcmp(e, "tch")) ? 1 : 0;
                  const char *m = calypso_getenv("CALYPSO_REJEU_ENREG_MAX"); if (m && *m) plafond = strtoul(m, NULL, 0); }
    if (tch_seul && g_dedie_genre != BSP_DEDIE_TCH) return NULL;
    /* any dedicated channel (SDCCH included): what corrupts the DSP state may precede the TCH */
    if (!on || g_dedie_tn <= 0 || g_enreg_n >= plafond)
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
    g_air_k_der = air_niveaux_on() ? air_pm_k(fn, tn) : 1.0;
    if (air_niveaux_on()) {
        uint64_t acc = 0;
        for (int i = 0; i < n_int16; i++) { int v = iq[i]; acc += (v < 0) ? (uint32_t)(-v) : (uint32_t)v; }
        g_air_dpm = bsp_apm_de_mav((double)acc / n_int16 * g_air_k_der, faux_air_gain(g_tick_courant, 5000));
    }
    if (calypso_bsp_dma_trace_on())
        printf("  [dma-dep] tick=%u insn=%u fn=%u tn=%u n=%d armee=%d one_shot=%d fenetre=%d\n", g_tick_courant,
               bsp.dsp->insn_count, (unsigned)fn, (unsigned)tn, n_int16 / 2, (int)calypso_rhea_dma_rx_armed(),
               (int)calypso_rhea_dma_one_shot(), calypso_rhea_dma_get_len_words() / 2);
    if (calypso_rhea_dma_one_shot() && calypso_rhea_dma_get_len_words() / 2 > 0 &&
        calypso_rhea_dma_get_len_words() / 2 < 150)
        bsp_pm_mav_noter(iq, n_int16, g_air_k_der);   /* measurement window: see calypso_bsp_rssi_apm_k() */

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
