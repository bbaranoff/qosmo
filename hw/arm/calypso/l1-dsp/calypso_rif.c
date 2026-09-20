/*
 * calypso_rif.c - Calypso Radio InterFace (RIF), DSP side (XIO space).
 *
 * Every PORTR instruction of the DSP firmware reads XIO:0003 (SPCR): the
 * PORTR-ANY probe reports `PA=0x0003 addr=0x000e PC=0xa675` on 30 samples out
 * of 30. Without a RIF model PORTR is a no-op (outside PA=0xF430/0x0034 under
 * gate CALYPSO_FIX_PORTR, default 0, which this firmware never uses), so the
 * receive handler tests a stale word and returns: the DSP does take its RX
 * interrupt (PC=0x00c0, 16330 entries) but produces nothing (fb0_att=37,
 * A_CD-WR=0).
 *
 * SILICON (CAL207 §12.1, Table 19):
 *
 *   Register  XIO      ARM         Access     Reset
 *   DXR       0x0000   FFFF:7000   16b R/W    undefined
 *   DRR       0x0001   FFFF:7002   16b R      undefined
 *   SPCX      0x0002   n.a.        15b        000 0101 1001 1110 = 0x059E
 *   SPCR      0x0003   n.a.        15b        011 1100 1010 0010 = 0x3CA2
 *
 * SPCR (§12.6), field by field; the fields add up to exactly the documented
 * reset value 0x3CA2, which confirms the reading:
 *
 *   0  DLB          0   digital local loopback
 *   1  RRST         1   receiver reset (0 -> clears RSRFULL and RRDY)
 *   2  RRDY         0   "Unused" per §12.6, yet §12.3 says "DRR is updated
 *                       when RSR is ready to be read and RRDY=1". The document
 *                       contradicts itself; see CALYPSO_RIF_RRDY_UNUSED below.
 *   3  RSRFULL      0   a word sits in RSR and DRR has not been read yet. Per
 *                       the doc it is NOT cleared by a DRR read: "This allows
 *                       keeping trace of a reception problem".
 *   4  ALMOST_FULL  0   1 = receive FIFO non empty (at THRESHOLD)
 *   5  FIFO_EMPTY   1   FIFO empty. The prose says "0 = receive FIFO is
 *                       empty", but the reset value is 1 AND the FIFO is empty
 *                       at reset: the prose is inverted, follow the reset.
 *   6  FIFO_FULL    0   FIFO full
 *   9:7 THRESHOLD 001   threshold of the "FIFO non empty" flag, maximum 4
 *   10 XINT_MASK    1   1 = transmit interrupt masked
 *   11 RINT_MASK    1   1 = RECEIVE interrupt masked
 *   12 XDMA_MASK    1   1 = transmit DMA request masked
 *   13 RDMA_MASK    1   1 = RECEIVE DMA request masked
 *   14 CLKLB        0   TX/RX clocks from the same internal source
 *
 * §3.7.1 has the DSP exchange with the RIF "either through XIO (word by word,
 * an interrupt is sent to the DSP), or through the API in DMA mode (a DMA
 * request and an end-DMA request are sent to the ARM)". Which of the two is
 * not this model's call: RINT_MASK (bit 11) and RDMA_MASK (bit 13) decide it,
 * and the FIRMWARE programs them. Both are masked at reset, so something must
 * open them; that choice is obeyed here instead of being forced by a gate.
 *
 * RX interrupt vector: INT0n = IMR bit 0 = vector 16 (CAL000 §5.1). The IMR
 * measured on this firmware (0x52ef) does have bit 0 open.
 *
 * NOT modelled here, and assumed:
 *   - the ARM aliases FFFF:7000 / FFFF:7002 (DXR/DRR seen from the MCU): the
 *     osmocom firmware does not touch them, so they are not wired.
 *   - transmission (DXR/SPCX): writes are accepted and counted, nothing is
 *     sent. TX goes through the TPU/TSP today.
 *   - XSR/RSR are not accessible (§12.4) and are not exposed.
 *   - the exact FIFO depth, which the doc does not give; THRESHOLD is capped
 *     at 4, so 4 is used. A full burst waits in a staging buffer and feeds the
 *     FIFO as DRR is read.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */
#include "qemu/osdep.h"
#include "hw/arm/calypso/calypso_debug.h"
#include "calypso_c54x.h"
#include "calypso_rif.h"
#include "calypso_rhea_dma.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* --- SPCR fields (§12.6) --- */
#define SPCR_DLB          (1u << 0)
#define SPCR_RRST         (1u << 1)
#define SPCR_RRDY         (1u << 2)
#define SPCR_RSRFULL      (1u << 3)
#define SPCR_ALMOST_FULL  (1u << 4)
#define SPCR_FIFO_EMPTY   (1u << 5)
#define SPCR_FIFO_FULL    (1u << 6)
#define SPCR_THRESHOLD_SH 7
#define SPCR_THRESHOLD_M  (7u << SPCR_THRESHOLD_SH)
#define SPCR_XINT_MASK    (1u << 10)
#define SPCR_RINT_MASK    (1u << 11)
#define SPCR_XDMA_MASK    (1u << 12)
#define SPCR_RDMA_MASK    (1u << 13)
#define SPCR_CLKLB        (1u << 14)

/* R/W bits are kept verbatim on write; the rest is hardware state recomputed
 * on read. */
#define SPCR_RW_MASK  (SPCR_DLB | SPCR_RRST | SPCR_THRESHOLD_M | SPCR_XINT_MASK \
                       | SPCR_RINT_MASK | SPCR_XDMA_MASK | SPCR_RDMA_MASK | SPCR_CLKLB)

#define SPCR_RESET   0x3CA2
#define SPCX_RESET   0x059E

#define RIF_FIFO_DEPTH  4        /* THRESHOLD is capped at 4 (§12.6) */
#define RIF_STAGE_MAX   8192     /* [2026-09-20] a full 1250-symbol frame is 2500 words */

static struct {
    bool     init;
    uint16_t spcr;               /* R/W fields only */
    uint16_t spcx;
    uint16_t dxr;

    uint16_t fifo[RIF_FIFO_DEPTH];
    int      fifo_n;

    uint16_t stage[RIF_STAGE_MAX];   /* burst waiting to enter the FIFO */
    int      stage_n;
    int      stage_pos;

    bool     rsrfull;            /* word shifted in while DRR was unread */
    uint16_t drr;                /* last word popped from the FIFO */
    bool     drr_valid;

    unsigned n_burst, n_drr_rd, n_spcr_rd, n_spcr_wr, n_int, n_dma, n_overrun;
    unsigned n_wr_arme;      /* SPCR writes that CLEAR a receive mask */
    unsigned n_wr_rrst_on;   /* SPCR writes that release RRST */

    /* [2026-08-04] @BEQUILLE RIF_BACKPRESSURE: bursts refused for lack of
     * room, and bursts forced through by the anti-stall valve. `bp_run` =
     * consecutive refusals in progress. */
    unsigned n_bp_skip, n_bp_force, bp_run;
    unsigned n_muets;        /* bursts arriving with no receive window open */
} rif;

/* Discard whatever the receiver still holds (a one-shot window just closed:
 * the radio stops, the rest of the frame is never received). */
void calypso_rif_flush(void)
{
    rif.fifo_n = 0;
    rif.stage_pos = rif.stage_n = 0;
}

bool calypso_rif_on(void)
{
    static int on = -1;
    if (on < 0) {
        on = calypso_gate("CALYPSO_RIF_XIO", 1);
        if (!on)
            fprintf(stderr, "[rif] CALYPSO_RIF_XIO=0 : RIF non modelise, PORTR/PORTW "
                    "redeviennent des no-op (comportement d'avant le 03/08)\n");
    }
    return on != 0;
}

/* ═══════════════════════════════════════════════════════════════════════════
 * @BEQUILLE — RIF_BACKPRESSURE  (CALYPSO_RIF_BACKPRESSURE, default 0)
 *
 *   (1) THIS IS A CRUTCH. The silicon has no back-pressure: when a burst
 *       arrives while the previous one is unread, the receiver OVERWRITES it
 *       and sets RSRFULL (§12.6). Refusing delivery is a DEVIATION from the
 *       hardware, not a model fix.
 *
 *   (2) WHAT IT MASKS: the c54x interpreter is ~96x too slow. Measured
 *       [2026-08-04] on a native_twl run: 73500 bursts pushed for 54492
 *       overruns — 74% of the bursts overwrite an unconsumed predecessor, and
 *       the DSP drains only ~4 words (RIF_FIFO_DEPTH) out of 296 before the
 *       next overwrite. The DMA therefore carries zeros, `a_cd` is published
 *       empty, and an SI only comes out when fill/drain/RRST happen to line
 *       up. While this gate is 1 the figure measured is NOT the real DSP
 *       throughput, only that of a bench slowed down to suit it.
 *
 *   (3) WHEN TO REMOVE IT: as soon as the DSP consumes a whole burst within
 *       one frame, that is, once the interpreter is fast enough (JIT, or a
 *       revised per-frame budget). Criterion: `overrun` must fall close to 0
 *       with the gate at 0. A non-zero `n_bp_force` means the DSP still does
 *       not consume and the valve had to force: the crutch is not enough, and
 *       that is not progress.
 *
 * RESTRICTED SCOPE: back-pressure applies ONLY when a drain path is armed
 * (RDMA or RINT unmasked). With an unarmed receiver nothing will ever empty
 * the staging buffer, so refusing would discard fresh bursts to protect a dead
 * one; in that case overwrite, like the hardware.
 *
 * ANTI-STALL VALVE: without it, a DSP that stops reading would freeze
 * reception forever. After CALYPSO_RIF_BP_MAX_SKIP consecutive refusals
 * (default 200) the next burst is forced through and logged.
 * ═══════════════════════════════════════════════════════════════════════════ */
static bool rif_backpressure_on(void)
{
    static int on = -1;
    if (on < 0) {
        on = calypso_gate("CALYPSO_RIF_BACKPRESSURE", 0);
        fprintf(stderr, "[rif] BEQUILLE contre-pression %s "
                "(CALYPSO_RIF_BACKPRESSURE=%d) — %s\n",
                on ? "ACTIVE" : "inactive", on,
                on ? "un burst n'ecrase plus un predecesseur non lu ; "
                     "DEVIATION du §12.6, le debit mesure n'est plus celui du DSP"
                   : "comportement materiel conserve : ecrasement + RSRFULL (defaut)");
    }
    return on != 0;
}

/* NUMERIC value: `calypso_gate` only returns 0/1, so it does not fit here.
 * getenv+atoi idiom, like the other thresholds of this model. */
static unsigned rif_bp_max_skip(void)
{
    static int v = -1;
    if (v < 0) {
        const char *e = calypso_getenv("CALYPSO_RIF_BP_MAX_SKIP");
        v = e ? atoi(e) : 200;
        if (v <= 0)
            v = 200;   /* a valve at 0 would be a guaranteed stall */
    }
    return (unsigned)v;
}

/* §12.6 marks bit 2 "Unused" while §12.3 relies on it. RRDY is exposed as "data
 * is available", the only reading that makes §12.3 consistent; the gate restores
 * the letter of §12.6 if a run needs it. */
static bool rif_rrdy_unused(void)
{
    static int u = -1;
    if (u < 0) u = calypso_gate("CALYPSO_RIF_RRDY_UNUSED", 0);
    return u != 0;
}

static void rif_init(void)
{
    if (rif.init)
        return;
    rif.init = true;
    rif.spcr = SPCR_RESET & SPCR_RW_MASK;
    rif.spcx = SPCX_RESET;
    fprintf(stderr, "[rif] RIF XIO arme (CAL207 §12) : DXR@0x0000 DRR@0x0001 "
            "SPCX@0x0002=0x%04x SPCR@0x0003=0x%04x ; RX IT = INT0n vec16/bit0, "
            "choisie par SPCR.RINT_MASK (bit11) / RDMA_MASK (bit13), tous deux "
            "MASQUES au reset — c'est au firmware de les ouvrir.\n",
            SPCX_RESET, SPCR_RESET);
}

static int rif_threshold(void)
{
    int t = (rif.spcr & SPCR_THRESHOLD_M) >> SPCR_THRESHOLD_SH;
    if (t < 1) t = 1;
    if (t > RIF_FIFO_DEPTH) t = RIF_FIFO_DEPTH;
    return t;
}

/* Pour the staging buffer into the FIFO while there is room. */
static void rif_refill(void)
{
    while (rif.fifo_n < RIF_FIFO_DEPTH && rif.stage_pos < rif.stage_n)
        rif.fifo[rif.fifo_n++] = rif.stage[rif.stage_pos++];
}

/* Hardware state recomposed on every SPCR read. */
static uint16_t rif_spcr_read(void)
{
    uint16_t v = rif.spcr & SPCR_RW_MASK;
    if (rif.fifo_n == 0)
        v |= SPCR_FIFO_EMPTY;
    if (rif.fifo_n >= rif_threshold())
        v |= SPCR_ALMOST_FULL;
    if (rif.fifo_n >= RIF_FIFO_DEPTH)
        v |= SPCR_FIFO_FULL;
    if (rif.rsrfull)
        v |= SPCR_RSRFULL;
    if (!rif_rrdy_unused() && rif.fifo_n > 0)
        v |= SPCR_RRDY;
    return v;
}

bool calypso_rif_portr(C54xState *s, uint16_t pa, uint16_t *out)
{
    (void)s;
    if (!calypso_rif_on())
        return false;
    rif_init();

    switch (pa) {
    case RIF_XIO_DRR:
        rif_refill();
        if (rif.fifo_n > 0) {
            rif.drr = rif.fifo[0];
            memmove(&rif.fifo[0], &rif.fifo[1],
                    (size_t)(rif.fifo_n - 1) * sizeof(uint16_t));
            rif.fifo_n--;
            rif.drr_valid = true;
            rif_refill();
        }
        /* On an empty FIFO, DRR keeps its last value: the doc defines no
         * read-empty value ("Undefined" at reset), and returning 0 would
         * fabricate a sample. */
        *out = rif.drr;
        if (rif.n_drr_rd++ < 20)
            fprintf(stderr, "[rif] DRR read #%u = 0x%04x (fifo=%d etage=%d/%d) "
                    "PC=0x%04x\n", rif.n_drr_rd, *out, rif.fifo_n,
                    rif.stage_pos, rif.stage_n, s ? s->pc : 0);
        return true;

    case RIF_XIO_SPCR:
        *out = rif_spcr_read();
        if (rif.n_spcr_rd++ < 20)
            fprintf(stderr, "[rif] SPCR read #%u = 0x%04x (fifo=%d empty=%d "
                    "almost=%d rsrfull=%d RINT_MASK=%d RDMA_MASK=%d) PC=0x%04x\n",
                    rif.n_spcr_rd, *out, rif.fifo_n,
                    !!(*out & SPCR_FIFO_EMPTY), !!(*out & SPCR_ALMOST_FULL),
                    !!(*out & SPCR_RSRFULL), !!(*out & SPCR_RINT_MASK),
                    !!(*out & SPCR_RDMA_MASK), s ? s->pc : 0);
        return true;

    case RIF_XIO_SPCX:
        *out = rif.spcx;
        return true;

    case RIF_XIO_DXR:
        *out = rif.dxr;   /* R/W per §12.2 */
        return true;

    default:
        return false;
    }
}

bool calypso_rif_portw(C54xState *s, uint16_t pa, uint16_t val)
{
    if (!calypso_rif_on())
        return false;
    rif_init();

    switch (pa) {
    case RIF_XIO_SPCR: {
        uint16_t before = rif.spcr;
        rif.spcr = val & SPCR_RW_MASK;
        /* §12.6: "Writing a zero to RRST clears the RSRFULL bit and RRDY bit".
         * The FIFO is flushed too: the receiver is in reset. */
        if ((before & SPCR_RRST) && !(val & SPCR_RRST)) {
            rif.rsrfull = false;
            rif.fifo_n = 0;
            rif.stage_pos = rif.stage_n;
            rif.drr_valid = false;
        }
        rif.n_spcr_wr++;
        if (!(val & SPCR_RINT_MASK) || !(val & SPCR_RDMA_MASK)) rif.n_wr_arme++;
        if (val & SPCR_RRST) rif.n_wr_rrst_on++;
        if (rif.n_spcr_wr <= 20 || (rif.n_spcr_wr % 200) == 0)
            fprintf(stderr, "[rif] SPCR write #%u 0x%04x -> RRST=%d THRESHOLD=%d "
                    "RINT_MASK=%d RDMA_MASK=%d XINT_MASK=%d PC=0x%04x\n",
                    rif.n_spcr_wr, val, !!(val & SPCR_RRST), rif_threshold(),
                    !!(val & SPCR_RINT_MASK), !!(val & SPCR_RDMA_MASK),
                    !!(val & SPCR_XINT_MASK), s ? s->pc : 0);
        return true;
    }
    case RIF_XIO_SPCX:
        rif.spcx = val;
        return true;

    case RIF_XIO_DXR:
        /* TX is not modelled (see header): accept and count. */
        rif.dxr = val;
        return true;

    case RIF_XIO_DRR:
        /* §12.3: "DRR cannot be written via the RHEA interface." */
        return true;

    default:
        return false;
    }
}

/* [2026-09-19] How many words the receiver still holds: the FIFO plus the part
 * of the staging buffer not yet poured into it. A transfer that starts with a
 * non-zero level begins on LEFTOVERS from an earlier burst, which pushes the
 * real burst one page further into DARAM — the mechanism behind the TOA ladder
 * in steps of 48 samples (= one 96-word DMA page) measured on this bench. */
int calypso_rif_level(void)
{
    if (!calypso_rif_on()) return 0;
    rif_init();
    int reste = rif.stage_n - rif.stage_pos;
    if (reste < 0) reste = 0;
    return rif.fifo_n + reste;
}

int calypso_rif_drain(uint16_t *dst, int max)
{
    if (!calypso_rif_on() || !dst || max <= 0)
        return 0;
    rif_init();

    int got = 0;
    while (got < max) {
        if (rif.fifo_n == 0) {
            rif_refill();          /* same refill as a DRR read */
            if (rif.fifo_n == 0)
                break;             /* receiver empty: return what we have */
        }
        dst[got++] = rif.fifo[0];
        memmove(&rif.fifo[0], &rif.fifo[1],
                (size_t)(rif.fifo_n - 1) * sizeof(uint16_t));
        rif.fifo_n--;
    }
    /* Receiver drained: RSRFULL no longer applies (§12.6). */
    if (got && rif.fifo_n == 0 && rif.stage_pos >= rif.stage_n)
        rif.rsrfull = false;
    return got;
}

void calypso_rif_rx_burst(C54xState *s, const uint16_t *w, int n)
{
    if (!calypso_rif_on() || !w || n <= 0)
        return;
    rif_init();

    if (n > RIF_STAGE_MAX)
        n = RIF_STAGE_MAX;

    /* [2026-09-20] The receiver is a STREAM: words not yet drained stay ahead of
     * the new ones (the hardware shift register does not forget them because a
     * new burst arrived). The DMA now transfers full pages only and leaves the
     * tail of a frame in here, so this append is what carries the 156.25
     * symbols per frame across frame boundaries. Overrun is a real overflow of
     * the staging area. CALYPSO_RIF_REPLACE=1 restores the old
     * replace-and-backpressure behaviour for A/B. */
    static int remplace = -1;
    if (remplace < 0) remplace = calypso_getenv("CALYPSO_RIF_REPLACE") ? 1 : 0;
    if (!remplace && !calypso_rhea_dma_rx_armed()) {
        /* [2026-09-20] No receive window open (DMA2 disabled, or its one-shot
         * window already filled): on silicon the radio is off between windows
         * and those samples never exist. Queueing them would put a stale
         * backlog in front of the next window and shift every ToA. */
        rif.n_muets++;
        return;
    }
    if (!remplace) {
        int reste = rif.stage_n - rif.stage_pos;
        if (reste < 0) reste = 0;
        if (reste && rif.stage_pos)
            memmove(rif.stage, rif.stage + rif.stage_pos, (size_t)reste * sizeof(uint16_t));
        rif.stage_n = reste; rif.stage_pos = 0;
        if (rif.stage_n + n > RIF_STAGE_MAX) {
            rif.rsrfull = true;
            if (rif.n_overrun++ < 10)
                fprintf(stderr, "[rif] OVERRUN #%u : %d mots en attente + %d nouveaux > %d, "
                        "les plus anciens sont perdus\n", rif.n_overrun, reste, n, RIF_STAGE_MAX);
            int perte = rif.stage_n + n - RIF_STAGE_MAX;
            memmove(rif.stage, rif.stage + perte, (size_t)(rif.stage_n - perte) * sizeof(uint16_t));
            rif.stage_n -= perte;
        }
        memcpy(rif.stage + rif.stage_n, w, (size_t)n * sizeof(uint16_t));
        rif.stage_n += n;
        rif_refill();
        rif.n_burst++;
    } else {
    /* The previous burst was not fully consumed: a real receiver overrun,
     * counted rather than hidden. */
    if (rif.stage_pos < rif.stage_n) {
        rif.rsrfull = true;
        if (rif.n_overrun++ < 10)
            fprintf(stderr, "[rif] OVERRUN #%u : %d mots du burst precedent non lus "
                    "(RSRFULL pose, cf. §12.6 : le bit garde la trace du probleme)\n",
                    rif.n_overrun, rif.stage_n - rif.stage_pos);

        /* @BEQUILLE RIF_BACKPRESSURE — see rif_backpressure_on() above.
         * REFUSE the new burst instead of overwriting, so the DSP has time to
         * consume the 296 words instead of the ~4 it otherwise drains. RSRFULL
         * stays set: the firmware keeps the trace of the problem.
         * [2026-08-04] Apply back-pressure ONLY when a drain path is armed.
         * Measured: `RDMA_MASK=1` on 36 of the 52 sampled `[rif] burst #`
         * lines, so the receiver is unarmed half of the time. Refusing a burst
         * then is HARMFUL: the stale staging buffer is never drained, so fresh
         * bursts are dropped until the valve forces one through (measured: 38
         * BP-FORCE, with `292/296 mots encore a lire` unchanged after 200
         * refusals). With nothing armed, the hardware behaviour — overwrite —
         * is the right one. */
        bool drain_arme = !(rif.spcr & SPCR_RDMA_MASK) ||
                          !(rif.spcr & SPCR_RINT_MASK);
        if (rif_backpressure_on() && drain_arme) {
            if (++rif.bp_run <= rif_bp_max_skip()) {
                rif.n_bp_skip++;
                if (rif.n_bp_skip <= 10 || (rif.n_bp_skip % 2000) == 0)
                    fprintf(stderr, "[rif] BP-SKIP #%u : burst refuse, %d/%d mots "
                            "du precedent encore a lire (refus consecutifs %u/%u)\n",
                            rif.n_bp_skip, rif.stage_n - rif.stage_pos,
                            rif.stage_n, rif.bp_run, rif_bp_max_skip());
                return;          /* no delivery, hence no notification */
            }
            /* Valve: the DSP has stopped consuming, so force the burst
             * through rather than freeze reception. A non-zero counter here
             * CONTRADICTS any claim of progress. */
            rif.n_bp_force++;
            fprintf(stderr, "[rif] BP-FORCE #%u : %u refus consecutifs atteints "
                    "(CALYPSO_RIF_BP_MAX_SKIP), burst impose — le DSP ne draine "
                    "plus, la contre-pression ne suffit pas\n",
                    rif.n_bp_force, rif.bp_run);
        }
    }
    rif.bp_run = 0;

    memcpy(rif.stage, w, (size_t)n * sizeof(uint16_t));
    rif.stage_n = n;
    rif.stage_pos = 0;
    rif.fifo_n = 0;
    rif_refill();
    rif.n_burst++;
    }   /* CALYPSO_RIF_REPLACE */

    /* §12.6 bits 11/13: the FIRMWARE picked the mode, not this model. */
    bool rint = !(rif.spcr & SPCR_RINT_MASK);
    bool rdma = !(rif.spcr & SPCR_RDMA_MASK);

    if (rint && s) {
        rif.n_int++;
        c54x_interrupt_ex(s, C54X_IT_RIF_RX_VEC, C54X_IT_RIF_RX_BIT);
    }
    if (rdma) {
        rif.n_dma++;
        /* The end-of-DMA request of §3.7.1 is served, not just counted. */
        calypso_rhea_dma_rx_request(s);
    }

    if (rif.n_burst <= 10 || (rif.n_burst % 500) == 0)
        fprintf(stderr, "[rif] BILAN bursts=%u notifies=%u (IT=%u DMA=%u) muets=%u ; "
                "ecritures SPCR=%u dont %u liberent un masque de reception, "
                "%u relachent RRST\n",
                rif.n_burst, rif.n_int + rif.n_dma, rif.n_int, rif.n_dma,
                rif.n_burst - rif.n_int - rif.n_dma,
                rif.n_spcr_wr, rif.n_wr_arme, rif.n_wr_rrst_on);
    if (rif.n_burst <= 10 || (rif.n_burst % 500) == 0)
        fprintf(stderr, "[rif] burst #%u n=%d -> FIFO %d/%d ; RINT_MASK=%d "
                "RDMA_MASK=%d => %s (IT=%u DMA=%u overrun=%u)\n",
                rif.n_burst, n, rif.fifo_n, RIF_FIFO_DEPTH,
                !rint, !rdma,
                rint ? "INT0n vec16/bit0"
                     : (rdma ? "requete DMA (end-DMA vers l'ARM, §3.7.1)"
                             : "AUCUNE notification — les deux masques sont poses, "
                               "le firmware n'a pas arme le recepteur"),
                rif.n_int, rif.n_dma, rif.n_overrun);

    if (rif_backpressure_on() && (rif.n_burst <= 10 || (rif.n_burst % 500) == 0))
        fprintf(stderr, "[rif] BP-BILAN burst=%u overrun=%u refuses=%u imposes=%u\n",
                rif.n_burst, rif.n_overrun, rif.n_bp_skip, rif.n_bp_force);
}
