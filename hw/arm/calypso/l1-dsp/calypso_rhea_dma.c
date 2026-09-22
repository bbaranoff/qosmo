/*
 * calypso_rhea_dma.c - Calypso RHEA DMA controller, MCU side (FFFF:FC00).
 *
 * WHY THIS FILE EXISTS
 * --------------------
 * Measured: the DSP firmware drives the RIF correctly (the two-write RRST
 * sequence of §12.6, SPCR read-back, reaction on a non-empty FIFO) but keeps
 * both RINT_MASK and RDMA_MASK at 1 for the whole run and never reads DRR. It
 * sees that data is there and does not take it through the serial port, so the
 * data must arrive in memory - §3.7.1, "API interface for radio data in DMA
 * mode (buffered mode with data block transfer)".
 *
 * [2026-08-03, run native_twl] The ARM writes exactly one thing in this window:
 *     ALLOC_CONFIG <- 0x000C  ->  channel1=DSP channel2=DSP channel3/4=ARM
 * It hands both RIF channels to the DSP and keeps the two UART channels.
 * Crossed with §6 Table 2 (channels 0..3: 0=RIF_DMA_REQ_X, 1=RIF_DMA_REQ_R,
 * 2/3=UARTs) and §11 (registers DMA1..DMA4), the consistent reading is
 * DMA1=RIF TX, DMA2=RIF RX, DMA3/DMA4=UARTs - §6 numbers from 0, §11 from 1.
 * The alternative (DMA1=RIF RX, DMA2=UART) would hand a UART channel to the
 * DSP, which makes no sense for this firmware; it stays possible and is settled
 * by the RAD/AAD values actually written.
 *
 * Consequence (§11.3): "The DMA transfer configuration registers are connected
 * to either the ARM Rhea bus OR TO THE DSP RHEA BUS regarding the corresponding
 * DMA_ALLOC flag." The ARM therefore never writes DMA1_AAD: the registers of
 * the handed-over channels live at XIO:FC10 / XIO:FC20, on the DSP side. Hence
 * the calypso_rhea_dma_xio() bridge below - same register bank, second bus.
 *
 * WHAT THIS MODULE DOES
 * Stores and returns the §11 registers with their reset values, and logs every
 * decoded write. DMA_START itself runs no transfer: the RX transfer happens in
 * calypso_rhea_dma_rx_request() when a burst reaches the RIF (see the block
 * above that function).
 *
 * MAP (§11.1, Table 18) - 0x100 window from FFFF:FC00:
 *   +0x00 CONTROLLER_CONFIG   6b R/W   DMA_BURST(4:2)=1, PRIORITY_ENABLE(5)=1
 *   +0x02 ALLOC_CONFIG        4b R/W   reset 1111 = all 4 channels owned by ARM
 *   +0x10 DMA1_RAD           16b R/W   RHEA_START(10:0) + RHEA_CS(15:11)
 *   +0x12 DMA1_RDPTH         11b R/W   Rhea buffer depth, in BYTES
 *   +0x14 DMA1_AAD           12b R/W   start of the RX buffer in the API,
 *                                      "The address is always expressed in Bytes"
 *   +0x16 DMA1_ALGTH         12b R/W   API page length, in bytes
 *   +0x18 DMA1_CTRL          13b       reset 0x04A2 (see below)
 *   +0x1A DMA1_CUR_OFFSET_API 12b R    current offset
 *   ... channels 2/3/4 at +0x20 / +0x30 / +0x40.
 *
 * DMAn_CTRL (§11.3.5) - the documented reset string "0 0100 1?10 0010" resolves
 * field by field to 0x04A2, which validates the field list:
 *   0 ENABLE=0 - 1 IDLE=1 (R) - 2 ONE_SHOT=0 - 3 FIFO_MODE=0 - 4 CURRENT_PAGE=0
 *   5 MAS=1 (16-bit transfers) - 6 DMA_START (W, always reads back 0)
 *   7 IRQ_MODE=1 - 8 IRQ_STATE=0 (R) - 9 RHEA_ERROR=0 (R)
 *   10 DIRECTION=1 -> "Transactions are done on Rhea -> API" - 12:11 PRIORITY=00
 * The "?" in the doc is DMA_START, which always reads back zero. DIRECTION=1 at
 * reset confirms the expected direction for radio reception: peripheral -> API.
 *
 * Doc inconsistency, flagged rather than hidden: Table 18 gives
 * CONTROLLER_CONFIG reset as "11 111?" while the §11.2.1 field list gives
 * DMA_BURST=0x1 and PRIORITY_ENABLE=1, i.e. 0b100100 = 0x24. The field list is
 * the more precise of the two and is what CTRLCFG_RESET follows.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */
#include "qemu/osdep.h"
#include "hw/arm/calypso/calypso_debug.h"
#include "calypso_rhea_dma.h"
#include "calypso_rif.h"
#include "calypso_c54x.h"
#include "calypso_bsp.h"

#include <stdio.h>
#include <stdlib.h>

#define RD_CTRL_CFG   0x00
#define RD_ALLOC_CFG  0x02
#define RD_CH_BASE(n) (0x10 + 0x10 * (n))   /* n = 0..3 -> 0x10 0x20 0x30 0x40 */
#define RD_RAD        0x0
#define RD_RDPTH      0x2
#define RD_AAD        0x4
#define RD_ALGTH      0x6
#define RD_CTRL       0x8
#define RD_CUR_OFF    0xA

#define CTRL_RESET    0x04A2
#define CTRLCFG_RESET 0x0024
#define ALLOC_RESET   0x000F

/* DMAn_CTRL fields */
#define CTRL_ENABLE       (1u << 0)
#define CTRL_IDLE         (1u << 1)
#define CTRL_ONE_SHOT     (1u << 2)
#define CTRL_FIFO_MODE    (1u << 3)
#define CTRL_CURRENT_PAGE (1u << 4)
#define CTRL_MAS          (1u << 5)
#define CTRL_DMA_START    (1u << 6)
#define CTRL_IRQ_MODE     (1u << 7)
#define CTRL_IRQ_STATE    (1u << 8)
#define CTRL_RHEA_ERROR   (1u << 9)
#define CTRL_DIRECTION    (1u << 10)

/* Read-only bits: kept by the model, never overwritten by the ARM. */
#define CTRL_RO_MASK  (CTRL_IDLE | CTRL_IRQ_STATE | CTRL_RHEA_ERROR)

/* [2026-09-22] COMBIEN DE TRANSFERTS SE TERMINENT, COMBIEN LA ROM EN ACQUITTE.
 * La ROM ferme un transfert en LISANT DMAn_CTRL (IRQ_STATE s'efface a la
 * lecture, CAL207 §11.3.5). Si elle lit moins souvent que la DMA ne termine,
 * ses files de requetes -- 14 entrees circulaires, 0xaa83/0xaaad/0xaad1 --
 * debordent : c'est le `DSP Error Status: 24` (DMA_PROG|DMA_TASK) permanent
 * depuis le boot, 18492 occurrences sur un run. La trace existante etait
 * plafonnee a 20 lignes et ne comptait donc rien. */
static unsigned long long g_rd_fini, g_rd_acquitte;

static struct {
    bool     init;
    uint16_t ctrl_cfg, alloc_cfg;
    struct { uint16_t rad, rdpth, aad, algth, ctrl, cur_off; } ch[4];
    unsigned n_wr, n_rd, n_start;
} rd;
static C54xState *rd_dsp;   /* the DSP served by the last request (for pumps) */
/* [2026-09-20] TOA ORIGIN. On silicon the FB search starts consuming the sample
 * stream at a TPU-timed instant inside the frame, and the firmware's constant
 * 23 (prim_fbsb.c "toa -= 23") encodes where a burst aligned on the frame then
 * lands. The bench has no TPU: the DSP arms DMA2 at the frame interrupt and the
 * frame's samples follow, so a burst at frame position 0 read TOA = 144. The
 * gap is closed HERE, where the timing lives: when the channel is armed in
 * continuous mode, the first CALYPSO_RHEA_DMA_ARM_SKIP words of the stream
 * (default 242 = 121 symbols = 144 - 23) are consumed without being stored. */
static int rd_skip_pending;
static int arm_skip_words(void)
{
    static int v = -1;
    /* Default 0 since the stream carries whole 1250-symbol frames: measured
     * TOA = 23 + 9 x 1250 + 7 for an FCCH nine frames after the command, i.e.
     * the ROM's own origin already matches the firmware's 23. (242 was the
     * calibration for the earlier 156-symbol frames.) */
    if (v < 0) { const char *e = calypso_getenv("CALYPSO_RHEA_DMA_ARM_SKIP"); v = (e && *e) ? atoi(e) : 0; if (v < 0) v = 0; }
    return v;
}

static bool rhea_dma_on(void)
{
    static int on = -1;
    if (on < 0) {
        on = calypso_gate("CALYPSO_RHEA_DMA", 1);
        if (!on)
            fprintf(stderr, "[rhea-dma] CALYPSO_RHEA_DMA=0 : retour au stub muet "
                    "(lectures a 0, ecritures jetees) — comportement d'avant le 03/08\n");
    }
    return on != 0;
}

static void rhea_dma_init(void)
{
    if (rd.init)
        return;
    rd.init = true;
    rd.ctrl_cfg  = CTRLCFG_RESET;
    rd.alloc_cfg = ALLOC_RESET;
    for (int i = 0; i < 4; i++)
        rd.ch[i].ctrl = CTRL_RESET;
    fprintf(stderr, "[rhea-dma] controleur DMA RHEA arme (CAL207 §11) @0xFFFFFC00 : "
            "4 canaux, reset CTRL=0x%04x (DIRECTION=1 Rhea->API, MAS=1 16 bits, "
            "IRQ_MODE=1, ENABLE=0). Canaux RIF : DMA1=TX DMA2=RX (§6 Table 2). "
            "ENREGISTREMENT SEUL : aucun transfert n'est execute.\n", CTRL_RESET);
}

/* Translates the API address (in BYTES, §11.3.3) into both useful frames. */
static void log_aad(int n, uint16_t aad)
{
    uint32_t byte = aad & 0x0FFF;
    fprintf(stderr, "[rhea-dma] *** DMA%d_AAD = 0x%03x octets  ->  mot DSP 0x%04x  "
            "(= ARM 0x%08x). C'est la DESTINATION du tampon de reception dans la "
            "memoire API.\n",
            n + 1, byte, (unsigned)(0x0800 + byte / 2), 0xFFD00000u + byte);
}

static void log_ctrl(int n, uint16_t v)
{
    fprintf(stderr, "[rhea-dma] DMA%d_CTRL <- 0x%04x : ENABLE=%d ONE_SHOT=%d "
            "FIFO_MODE=%d PAGE=%d MAS=%d(%s) DMA_START=%d IRQ_MODE=%d DIRECTION=%d(%s) "
            "PRIORITY=%d PC=0x%04x\n",
            n + 1, v,
            !!(v & CTRL_ENABLE), !!(v & CTRL_ONE_SHOT), !!(v & CTRL_FIFO_MODE),
            !!(v & CTRL_CURRENT_PAGE), !!(v & CTRL_MAS),
            (v & CTRL_MAS) ? "16b" : "8b",
            !!(v & CTRL_DMA_START), !!(v & CTRL_IRQ_MODE),
            !!(v & CTRL_DIRECTION),
            (v & CTRL_DIRECTION) ? "Rhea->API" : "API->Rhea",
            (v >> 11) & 3, rd_dsp ? (rd_dsp->pc & 0xffff) : 0);
}

uint64_t calypso_rhea_dma_read(void *opaque, hwaddr off, unsigned size)
{
    (void)opaque; (void)size;
    if (!rhea_dma_on())
        return 0;
    rhea_dma_init();

    uint16_t v = 0;
    if (off == RD_CTRL_CFG)       v = rd.ctrl_cfg;
    else if (off == RD_ALLOC_CFG) v = rd.alloc_cfg;
    else {
        for (int n = 0; n < 4; n++) {
            hwaddr b = RD_CH_BASE(n);
            if (off < b || off > b + RD_CUR_OFF)
                continue;
            switch (off - b) {
            case RD_RAD:     v = rd.ch[n].rad;     break;
            case RD_RDPTH:   v = rd.ch[n].rdpth;   break;
            case RD_AAD:     v = rd.ch[n].aad;     break;
            case RD_ALGTH:   v = rd.ch[n].algth;   break;
            /* DMA_START: "Reading of this bit is always equal to zero" (§11.3.5) */
            case RD_CTRL:
                v = rd.ch[n].ctrl & (uint16_t)~CTRL_DMA_START;
                /* ═════════════════════════════════════════════════════════════
                 * IRQ_STATE and RHEA_ERROR are cleared on read, CAL207 §11.3.5
                 * for both bits: "0 = cleared after being read". Leaving them
                 * sticky makes the firmware re-read them forever as if the event
                 * kept happening.
                 * ═════════════════════════════════════════════════════════════ */
                if (rd.ch[n].ctrl & (CTRL_IRQ_STATE | CTRL_RHEA_ERROR)) {
                    static unsigned long long n_clr;
                    if (rd.ch[n].ctrl & CTRL_IRQ_STATE) {
                        g_rd_acquitte++;
                        if ((g_rd_acquitte % 200) == 1) {
                            /* [2026-09-22] NIVEAU DE VERBOSITE. verbosite.c
                             * classe par mots-cles : cette ligne ne contient que
                             * « DMA », donc niveau 3, invisible au -v par
                             * defaut. C'est pour ca qu'elle avait « disparu » --
                             * pas parce que la ROM avait cesse d'acquitter.
                             * Plutot que de forcer un « ERR » sur une ligne saine
                             * (elle sortirait en niveau 0 a chaque run normal),
                             * on ne promeut QUE l'anomalie : retard
                             * d'acquittement = WARN, donc niveau 1. */
                            double pc = g_rd_fini ? 100.0 * g_rd_acquitte / g_rd_fini : 0.0;
                            bool mauvais = g_rd_fini > 100 && pc < 90.0;
                            fprintf(stderr, "[rhea-dma] %sbilan : %llu transferts finis, "
                                    "%llu acquittes par la ROM (%.0f%%)\n",
                                    mauvais ? "WARN " : "", g_rd_fini, g_rd_acquitte, pc);
                        }
                    }
                    /* [2026-09-22] Plafond leve de 20 a 4000 et compteurs
                     * joints : c'est le rapport « transferts finis / acquittes »
                     * qui dit si les files de la ROM debordent. */
                    if (n_clr++ < 4000)
                        fprintf(stderr, "[rhea-dma] DMA%d_CTRL lu = 0x%04x : acquitte "
                                "(finis=%llu acquittes=%llu)\n",
                                n + 1, v, g_rd_fini, g_rd_acquitte);
                    rd.ch[n].ctrl &= (uint16_t)~(CTRL_IRQ_STATE | CTRL_RHEA_ERROR);
                }
                break;
            case RD_CUR_OFF: v = rd.ch[n].cur_off; break;
            default: break;
            }
            break;
        }
    }
    if (rd.n_rd++ < 40)
        fprintf(stderr, "[rhea-dma] RD  +0x%02x = 0x%04x\n", (unsigned)off, v);
    return v;
}

void calypso_rhea_dma_write(void *opaque, hwaddr off, uint64_t val, unsigned size)
{
    (void)opaque; (void)size;
    if (!rhea_dma_on())
        return;
    rhea_dma_init();

    uint16_t v = (uint16_t)val;
    rd.n_wr++;

    if (off == RD_CTRL_CFG) {
        rd.ctrl_cfg = v & 0x003F;
        fprintf(stderr, "[rhea-dma] CONTROLLER_CONFIG <- 0x%04x (DMA_BURST=%d "
                "PRIORITY_ENABLE=%d)\n", v, (v >> 2) & 7, !!(v & 0x20));
        return;
    }
    if (off == RD_ALLOC_CFG) {
        rd.alloc_cfg = v & 0x000F;
        fprintf(stderr, "[rhea-dma] ALLOC_CONFIG <- 0x%04x : canal1=%s canal2=%s "
                "canal3=%s canal4=%s\n", v,
                (v & 1) ? "ARM" : "DSP", (v & 2) ? "ARM" : "DSP",
                (v & 4) ? "ARM" : "DSP", (v & 8) ? "ARM" : "DSP");
        return;
    }

    for (int n = 0; n < 4; n++) {
        hwaddr b = RD_CH_BASE(n);
        if (off < b || off > b + RD_CUR_OFF)
            continue;
        switch (off - b) {
        case RD_RAD:
            rd.ch[n].rad = v;
            fprintf(stderr, "[rhea-dma] DMA%d_RAD   <- 0x%04x (RHEA_START=0x%03x "
                    "RHEA_CS=%d)\n", n + 1, v, v & 0x7FF, (v >> 11) & 0x1F);
            break;
        case RD_RDPTH:
            rd.ch[n].rdpth = v & 0x07FF;
            fprintf(stderr, "[rhea-dma] DMA%d_RDPTH <- %u octets\n", n + 1, v & 0x7FF);
            break;
        case RD_AAD:
            /* Logged unmasked: AAD is the answer to "where must the burst land". */
            rd.ch[n].aad = v & 0x0FFF;
            log_aad(n, v);
            break;
        case RD_ALGTH:
            rd.ch[n].algth = v & 0x0FFF;
            fprintf(stderr, "[rhea-dma] DMA%d_ALGTH <- %u octets (= %u mots de page API) PC=0x%04x\n",
                    n + 1, v & 0xFFF, (v & 0xFFF) / 2, rd_dsp ? (rd_dsp->pc & 0xffff) : 0);
            break;
        case RD_CTRL: {
            uint16_t keep = rd.ch[n].ctrl & CTRL_RO_MASK;
            uint16_t before = rd.ch[n].ctrl;
            rd.ch[n].ctrl = (uint16_t)((v & ~CTRL_RO_MASK & 0x1FFF) | keep);
            /* PROM0 0xa640 is an explicit channel disarm, not polling:
             *     portr *(0x4356), 0xfc28    ; read DMA2_CTRL
             *     andm  *(0x4356), #0xfffe   ; clear bit 0 = ENABLE
             *     portw *(0x4356), 0xfc28    ; write back
             * The write is inert here only because ENABLE is already 0. Count
             * those passes instead of repeating them, and log everything that
             * CHANGES the state or sets ENABLE/DMA_START. */
            if (n == 1 && (v & CTRL_ENABLE) && !(before & CTRL_ENABLE) && !(v & CTRL_ONE_SHOT))
                rd_skip_pending = arm_skip_words();      /* FB search armed: set the TOA origin */
            bool inerte = (rd.ch[n].ctrl == before) && !(v & CTRL_DMA_START);
            if (inerte) {
                static unsigned long long n_rmw[4];
                if (n_rmw[n]++ == 0 || (n_rmw[n] % 20000) == 0)
                    fprintf(stderr, "[rhea-dma] DMA%d_CTRL DESARMEMENT x%llu "
                            "(0xa643 andm #0xfffe = efface ENABLE ; sans effet ici, "
                            "ENABLE valait deja 0 — relu/reecrit 0x%04x)\n",
                            n + 1, n_rmw[n], v);
                break;
            }
            log_ctrl(n, v);
            if (v & CTRL_DMA_START) {
                rd.n_start++;
                fprintf(stderr, "[rhea-dma] *** DMA%d DMA_START #%u — l'ARM DEMANDE un "
                        "transfert. Ce module NE L'EXECUTE PAS (instrument de lecture). "
                        "Parametres courants : RAD=0x%04x RDPTH=%u AAD=0x%03x "
                        "(mot DSP 0x%04x) ALGTH=%u\n",
                        n + 1, rd.n_start, rd.ch[n].rad, rd.ch[n].rdpth,
                        rd.ch[n].aad, (unsigned)(0x0800 + rd.ch[n].aad / 2),
                        rd.ch[n].algth);
            }
            break;
        }
        case RD_CUR_OFF:
            /* Read-only (§11.3.6): the write is ignored. */
            break;
        default:
            break;
        }
        return;
    }

    fprintf(stderr, "[rhea-dma] WR  +0x%02x = 0x%04x (hors carte §11)\n",
            (unsigned)off, v);
}

/* ═══════════════════════════════════════════════════════════════════════════
 * THE TRANSFER. On a burst, once the firmware has selected DMA mode (RDMA_MASK=0,
 * its own decision in SPCR - nothing is forced here):
 *   1. drain the RIF receiver into API memory at DMA2_AAD;
 *   2. bound the transfer by DMA2_ALGTH (page length, in BYTES, §11.3.3);
 *   3. set IRQ_STATE, clear DMA_START, raise INT10n when IRQ_MODE=1.
 *
 * Deliberately not done here:
 *   - nothing is armed by this code: with ENABLE=0 there is no transfer. The
 *     point is to serve a firmware request, not to manufacture one;
 *   - SPCR and the masks are left alone: the IT/DMA choice belongs to the
 *     firmware.
 *
 * The firmware itself reports the missing transfer by raising
 * DSP_ERR_DMA_PROG | DSP_ERR_DMA_TASK (d_error_status = 24) on every frame.
 *
 * [2026-09-18] Running the transfer is unconditional. It used to sit behind a
 * gate defaulting to off, as a non-regression precaution; without the transfer
 * the DSP correlates over a buffer nobody fills. Deterministic replay, this
 * behaviour the only change (1200 frames, synthetic cell):
 *     off : FB accepted   0, SB attempted   0, CRC OK  0
 *     on  : FB accepted 119, SB attempted 115, CRC OK  2
 * ═══════════════════════════════════════════════════════════════════════════ */

static void rhea_abort_impl(const char *why)
{
    rd.ch[1].ctrl |= CTRL_RHEA_ERROR;
    static unsigned long long n_ab;
    if (n_ab++ < 20)
        fprintf(stderr, "[rhea-dma] RHEA_ERROR pose : %s (CAL207 §11.3.5, efface a la lecture)\n", why);
}
#define rhea_abort(w) rhea_abort_impl(w)

/* DARAM address the ROM actually programs into DMA2 (the RX channel). AAD is in
 * BYTES from the base of the API window; the DSP word is 0x800 + (AAD&0xFFF)/2,
 * the same conversion as the transfer itself. Returns 0 when the channel is not
 * armed. The BSP uses it to drop the burst WHERE the DSP will read it, instead
 * of at a frozen constant. */
uint16_t calypso_rhea_dma_get_daram(void)
{
    if (!(rd.ch[1].ctrl & CTRL_ENABLE) && rd.ch[1].aad == 0) return 0;
    if (rd.ch[1].aad == 0) return 0;
    return (uint16_t)(0x800u + ((rd.ch[1].aad & 0x0FFFu) / 2u));
}

/* Page length actually programmed by the DSP, in words. ALGTH is in bytes (see
 * max_words below: algth/2). The BSP sizes its burst drop with this instead of a
 * frozen 296-word cap, which truncated the 380-word SB window and cut off the
 * second data block. */
uint16_t calypso_rhea_dma_get_len_words(void)
{
    if (rd.ch[1].algth == 0) return 0;
    return (uint16_t)(rd.ch[1].algth / 2u);
}

/* INT10n is a LEVEL line (CAL000 §5.1): asserted while any channel holds
 * IRQ_STATE. */
bool calypso_rhea_dma_irq_level(void)
{
    if (!rhea_dma_on() || !rd.init)
        return false;
    for (int i = 0; i < 4; i++)
        if (rd.ch[i].ctrl & CTRL_IRQ_STATE)
            return true;
    return false;
}

void calypso_rhea_dma_rx_request(C54xState *s)
{
    if (!rhea_dma_on())
        return;
    rhea_dma_init();

    const int n = 1;                 /* DMA2 = RIF RX channel (§6 Table 2) */
    uint16_t ctrl = rd.ch[n].ctrl;
    if (s) rd_dsp = s;

    /* ═══════════════════════════════════════════════════════════════════════
     * The IDLE bit is driven by the HARDWARE, CAL207 §11.3.5: "1 IDLE: 0 = DMA
     * transfer is running / 1 = DMA channel is idle", access R, reset 1. Never
     * clearing it leaves the DSP unable to tell "transfer running" from
     * "transfer done".
     *
     * What that costs, traced back into the ROM: 0xaa83 / 0xaaad / 0xaad1 set
     * DMA_PROG / DMA_TASK / DMA_PEND. All three have the same shape, a 14-entry
     * CIRCULAR queue (`stm #0x000e`; `stl *AR0+%`; `banz`), and the bit is
     * raised when it OVERFLOWS: the DSP piles up requests nothing ever closes.
     * Counted over one run: DMA_PROG 948 times, DMA_PEND 541. On the osmocom
     * side the L1 only prints and clears d_error_status, so the witness is a
     * DSP report, not something the ARM corrects.
     *
     * ⚠️ The bit is R in the doc: the firmware must never write it. CTRL_RO_MASK
     * already protects it from software writes; only internal state is touched
     * here, which is the hardware's role.
     * ORDERING: it is cleared further down, AFTER every validation, so that an
     * early return never leaves the channel marked "running" with no transfer
     * having taken place.
     * ═══════════════════════════════════════════════════════════════════════ */

    /* Only serve what the firmware has actually armed. */
    if (!(ctrl & CTRL_ENABLE)) {
        static unsigned long long n_off;
        if (n_off++ == 0 || (n_off % 5000) == 0)
            fprintf(stderr, "[rhea-dma] requete RX x%llu ignoree : DMA2 ENABLE=0 "
                    "(le firmware n'a pas arme ce canal — on ne l'arme pas a sa "
                    "place)\n", n_off);
        return;
    }
    if (!(ctrl & CTRL_DIRECTION)) {
        static unsigned n_dir;
        if (n_dir++ < 5)
            fprintf(stderr, "[rhea-dma] requete RX ignoree : DMA2 DIRECTION=0 "
                    "(API->Rhea), ce n'est pas une reception\n");
        return;
    }

    /* ALGTH is a length in BYTES (§11.3.3); API memory is addressed in words. */
    int max_words = rd.ch[n].algth ? (int)(rd.ch[n].algth / 2) : 0;
    if (max_words <= 0) {
        rhea_abort("ALGTH nulle");   /* BEFORE the if: the hardware state must not
                                      * depend on a log cap. */
        static unsigned n_len;
        if (n_len++ < 5)
            fprintf(stderr, "[rhea-dma] requete RX ignoree : DMA2_ALGTH=%u — aucune "
                    "longueur de page programmee\n", rd.ch[n].algth);
        return;
    }

    uint16_t *api = s ? s->api_ram : NULL;
    if (!api) {
        rhea_abort("memoire API absente");
        static unsigned n_api;
        if (n_api++ < 5)
            fprintf(stderr, "[rhea-dma] requete RX ignoree : memoire API absente\n");
        return;
    }

    /* AAD is a BYTE offset from the API base -> word index into api_ram: the DSP
     * word is 0x0800 + aad/2 and the api_ram index is that word minus
     * C54X_API_BASE, so exactly aad/2. */
    unsigned dst_idx = (rd.ch[n].aad & 0x0FFF) / 2;
    if (dst_idx + (unsigned)max_words > C54X_API_SIZE) {
        max_words = (int)(C54X_API_SIZE - dst_idx);
        static unsigned n_clip;
        if (max_words <= 0) {
            rhea_abort("AAD hors fenetre API");
            if (n_clip++ < 5)
                fprintf(stderr, "[rhea-dma] requete RX ignoree : AAD=0x%03x hors "
                        "memoire API\n", rd.ch[n].aad);
            return;
        }
        if (n_clip++ < 5)
            fprintf(stderr, "[rhea-dma] transfert TRONQUE a %d mots : AAD=0x%03x + "
                    "ALGTH depasse la fenetre API\n", max_words, rd.ch[n].aad);
    }

    static uint16_t buf[4096];
    if (max_words > (int)(sizeof(buf) / sizeof(buf[0])))
        max_words = (int)(sizeof(buf) / sizeof(buf[0]));

    /* ─────────────────────────────────────────────────────────────────────────
     * PAGE CHAINING (§11.3.5 CURRENT_PAGE). One page per request is not enough:
     * with a 96-word page (ALGTH=192) against a 296-word burst, the leftover
     * 200 words wait in the RIF staging area and the next burst finds them
     * there -> RSRFULL -> overrun, measured at 11392 of 13000 bursts (84%). At
     * that loss rate, assembling 4 consecutive bursts (one CCCH block) is
     * impossible, which is enough to explain SI=0 without blaming demodulation.
     * A burst therefore needs about 3 pages, drained in one pass. */
    rd.ch[n].ctrl &= (uint16_t)~CTRL_IDLE;      /* 0 = transfer running */
    /* [2026-09-19] What the receiver already held BEFORE this transfer, and how
     * each page filled. A transfer that opens on a non-zero level starts on the
     * tail of an earlier burst, so the real burst begins one page further in —
     * which is what a TOA quantised to multiples of 48 samples (one 96-word
     * page) looks like from the correlator. */
    int niveau_avant = calypso_rif_level();
    int got_page[8] = {0};
    int premier_nz = -1;
    int total = 0, pages = 0;
    /* Non-zero words are accumulated page by page: `buf` holds ONLY the last
     * page (max_words words), so scanning buf[0..total[ would read leftovers. */
    int nz_total = 0;
    uint16_t head8[8] = {0};
    /* [2026-09-20] STREAM SEMANTICS. The RIF is a continuous sample stream and
     * the DMA fires when a PAGE is full, not when a burst was handed over:
     *   - double-buffer mode (ONE_SHOT=0, the FB search: ALGTH=192 -> 96-word
     *     pages at 0x0cce and 0x0d2e): only FULL pages are transferred, a partial
     *     tail stays in the receiver for the next frame; at most the two pages
     *     are filled per request, then ONE interrupt, and the ROM's ISR consumes
     *     both halves (0xb2c1 selects the half by the parity of 0x3fb4, which it
     *     advances by 2 per interrupt = 2 x 48 samples). Refilling page 0 before
     *     the ISR ran would corrupt the half being processed, so the caller
     *     pumps the next pages once the DSP is idle (calypso_rhea_dma_pump).
     *   - one-shot mode (the SB window, ALGTH=764): the single page takes what is
     *     there, partial or not, and interrupts.
     * Measured before: a 312-word frame drained in one pass raised one interrupt,
     * the ROM counted 2 blocks per frame instead of 3.25 and its TOA advanced 96
     * per frame where the firmware divides by 156. */
    const bool one_shot = (ctrl & CTRL_ONE_SHOT) != 0;
    /* [2026-09-20] The double buffer is not refilled while its completion is
     * still pending: IRQ_STATE is cleared when the ISR reads DMA2_CTRL, and
     * until then both pages belong to the DSP. Transferring anyway raised a
     * second INT10n on top of the first (one interrupt lost) and overwrote the
     * pages under the ISR. Measured: 13 pairs handed over per frame, the ROM
     * counted 6 (0x3fb4 += 12 instead of 26). The words wait in the receiver;
     * calypso_rhea_dma_pump() moves them once the DSP is idle. */
    if (!one_shot && (ctrl & CTRL_IRQ_STATE)) {
        /* [2026-09-22] Rendre IDLE avant de sortir. La ligne 517 l'a efface
         * (« transfert en cours ») et TOUTES les autres sorties le reposent :
         * skip_pending, rien-a-transferer, fin de transfert. Celle-ci seule ne
         * le faisait pas, laissant le canal annoncer indefiniment un transfert
         * qui n'aura pas lieu -- la ROM qui scrute IDLE pour savoir si elle peut
         * reprogrammer le canal attend alors sans fin. */
        rd.ch[n].ctrl |= CTRL_IDLE;
        return;
    }
    if (!one_shot && rd_skip_pending > 0) {
        static uint16_t poubelle[256];
        while (rd_skip_pending > 0) {
            int m = rd_skip_pending < 256 ? rd_skip_pending : 256;
            int g = calypso_rif_drain(poubelle, m);
            if (g <= 0) break;
            rd_skip_pending -= g;
        }
        if (rd_skip_pending > 0) {              /* origin not reached yet: nothing to store */
            rd.ch[n].ctrl |= CTRL_IDLE;
            return;
        }
    }
    for (;;) {
        if (!one_shot && calypso_rif_level() < max_words)
            break;                                /* wait for a full page */
        int got = calypso_rif_drain(buf, max_words);
        if (got <= 0)
            break;

        /* Pages are written CONTIGUOUSLY, page k -> dst_idx + k*96, not
         * ping-ponged. CAL207 §11.3.5 DMA_CTRL bit4 says the destination follows
         * CURRENT_PAGE ("0 = first API page, 1 = second API page, automatically
         * updated during the transfer"), i.e. page 0 = AAD (dst_idx), page 1 =
         * AAD+ALGTH. But the FB correlator reads a CONTIGUOUS 296-word buffer
         * (=148 IQ at 1 SPS, disasm PROM 0xb2c4=0x0cce / 0xb2c9=0x0d2e), and
         * two 96-word pages fold a 296-word burst back on itself: page 2 over
         * page 0, page 3 over page 1 -> corrupted FCCH, correlator peaking at
         * the edge (TOA=39). The hardware double buffer is a REAL-TIME artifact,
         * the DSP consuming page 0 while the DMA fills page 1; it has no purpose
         * when the whole burst is drained in one synchronous pass. Contiguous
         * pages give a flat [0x0cce..0x0df6) the correlator can read whole.
         * A/B: CALYPSO_RHEA_DMA_PINGPONG=1 restores the 2-page ping-pong. */
        /* [2026-09-20] Ping-pong is the default again: page 0 at AAD, page 1 at
         * AAD+ALGTH, following CURRENT_PAGE (§11.3.5). The FB ROM reads exactly
         * those two 96-word halves (0x0cce / 0x0d2e); contiguous pages 2 and 3
         * landed past them. CALYPSO_RHEA_DMA_CONTIGU=1 restores the old layout. */
        unsigned pdst = dst_idx;
        if (rd.ch[n].ctrl & CTRL_CURRENT_PAGE)
            pdst = dst_idx + (unsigned)max_words;   /* 2nd API page = AAD+ALGTH */
        {
            static int contigu = -1;
            if (contigu < 0) contigu = calypso_getenv("CALYPSO_RHEA_DMA_CONTIGU") ? 1 : 0;
            if (contigu) pdst = dst_idx + (unsigned)(pages * max_words);
        }
        /* [2026-09-19] Does the burst DMA ever land ON a_sch? a_sch[0..4] sit at
         * API words 0x37..0x3b (R page 0) and 0x4b..0x4f (R page 1). The cell
         * carries plain numbers (0x1111, 0x1388) that no firmware and no DSP
         * probe accounts for -- and an I/Q sample is exactly a plain number.
         * If a burst is deposited over the SB result, that is the carnage. */
        if ((pdst <= 0x3b && pdst + (unsigned)got > 0x37) ||
            (pdst <= 0x4f && pdst + (unsigned)got > 0x4b)) {
            static unsigned nov;
            if (nov++ < 20)
                fprintf(stderr, "[rhea-dma] *** ECRASEMENT a_sch : depot %u mots en "
                        "0x%04x..0x%04x recouvre a_sch (0x37..0x3b / 0x4b..0x4f) ***\n",
                        got, pdst, pdst + got - 1);
        }
        for (int i = 0; i < got; i++)
            if (pdst + (unsigned)i < C54X_API_SIZE)
                api[pdst + i] = buf[i];

        for (int i = 0; i < got; i++)
            if (buf[i]) nz_total++;
        if (pages == 0)
            for (int i = 0; i < 8 && i < got; i++) head8[i] = buf[i];
        /* [2026-09-19] Where the burst actually STARTS inside the window. The
         * cell injector leaves marge=21 samples of silence on each side of a
         * 148-symbol burst in a 190-sample window, so the first non-zero word
         * is expected at index 42 (21 samples x 2 words). Anything else means
         * the SB task is handed a burst that does not sit where it looks. */
        if (premier_nz < 0)
            for (int i = 0; i < got; i++)
                if (buf[i]) { premier_nz = total + i; break; }

        if (pages < 8) got_page[pages] = got;
        total += got;
        pages++;
        rd.ch[n].cur_off = (uint16_t)(got * 2);
        rd.ch[n].ctrl ^= CTRL_CURRENT_PAGE;      /* §11.3.5: next page */

        if (one_shot)
            break;                                /* a single page, then the interrupt */
        if (!(rd.ch[n].ctrl & CTRL_CURRENT_PAGE))
            break;                                /* both pages full: interrupt, let the ISR consume */
        if (got < max_words)
            break;                                /* receiver emptied before the page filled */
        if (pages >= 64) {                        /* guard: never loop forever */
            static unsigned n_cap;
            if (n_cap++ < 5)
                fprintf(stderr, "[rhea-dma] ⚠ plafond de 64 pages atteint sur une "
                        "requete (burst anormalement long ?) — reste non draine\n");
            break;
        }
    }

    if (total <= 0) {
        rd.ch[n].ctrl |= CTRL_IDLE;            /* nothing transferred -> idle */
        static unsigned long long n_empty;
        if (n_empty++ == 0 || (n_empty % 5000) == 0)
            fprintf(stderr, "[rhea-dma] requete RX x%llu : recepteur RIF vide, "
                    "rien a transferer\n", n_empty);
        return;
    }
    int got = total;

    /* §11.3.5: transfer complete -> IRQ_STATE set, DMA_START drops. */
    if (!(rd.ch[n].ctrl & CTRL_IRQ_STATE)) {
        g_rd_fini++;   /* un transfert de plus que la ROM devra acquitter */
    }
    rd.ch[n].ctrl = (uint16_t)((rd.ch[n].ctrl | CTRL_IRQ_STATE | CTRL_IDLE)
                               & ~CTRL_DMA_START);   /* done -> IDLE=1 */
    if (rd.ch[n].ctrl & CTRL_ONE_SHOT) {
        rd.ch[n].ctrl &= (uint16_t)~CTRL_ENABLE;
        calypso_rif_flush();             /* window closed: the radio stops receiving */
    }

    {
        static unsigned long long n_ok;
        n_ok++;
        /* [2026-09-19] The frame number was missing, and without it a transfer
         * cannot be put side by side with the firmware's own "=>FB @ FNR X" in
         * osmocon.log — which is the only way to tell whether the SB window of
         * frame X+1 really receives the SCH burst. calypso_daram_last_fn is set
         * by calypso_bsp_rx_burst() for exactly this.
         *
         * A 296-word transfer is an ordinary burst; anything else is a window
         * the firmware armed on purpose (380 words = the SB window), so those
         * are traced one by one up to 2000 instead of one in 500. */
        static unsigned long long n_sb;
        bool fenetre = (got != 296);
        if (n_ok <= 20 || (n_ok % 500) == 0 || (fenetre && n_sb++ < 2000))
            fprintf(stderr, "[rhea-dma] *** TRANSFERT RX #%llu fn=%u : %d mots RIF -> "
                    "api_ram[0x%04x..0x%04x] en %d page(s) (mot DSP 0x%04x, ARM 0x%08x) "
                    "ALGTH=%u IRQ_MODE=%d ONE_SHOT=%d\n",
                    n_ok, calypso_bsp_get_last_fn(),
                    got, dst_idx, dst_idx + (got > max_words ? max_words : got) - 1, pages,
                    (unsigned)(C54X_API_BASE + dst_idx),
                    0xFFD00000u + (rd.ch[n].aad & 0x0FFF),
                    rd.ch[n].algth, !!(rd.ch[n].ctrl & CTRL_IRQ_MODE),
                    !!(rd.ch[n].ctrl & CTRL_ONE_SHOT));
        /* Sample the payload, not just the plumbing, on the SAME cadence as the
         * TRANSFERT line above, so every sample matches a logged transfer.
         * Capping this at the first 10 transfers only shows startup, where the
         * input really is empty (measured: bursts #1..#5 at nz=0/2368, then #6
         * at nz=2211/2368) and invites the wrong conclusion that the DMA carries
         * only zeros. */
        if (n_ok <= 20 || (n_ok % 500) == 0 || (fenetre && n_sb <= 2000)) {
            fprintf(stderr, "[rhea-dma]     RIF avant = %d mots ; pages = "
                    "%d+%d+%d+%d+%d+%d+%d+%d ; 1er mot non nul a l'index %d "
                    "(attendu 42)\n", niveau_avant,
                    got_page[0], got_page[1], got_page[2], got_page[3],
                    got_page[4], got_page[5], got_page[6], got_page[7], premier_nz);
            fprintf(stderr, "[rhea-dma]     contenu : %d/%d mots non nuls ; "
                    "8 premiers = %04x %04x %04x %04x %04x %04x %04x %04x\n",
                    nz_total, total,
                    head8[0], head8[1], head8[2], head8[3],
                    head8[4], head8[5], head8[6], head8[7]);
        }
    }

    /* §11.3.5 IRQ_MODE: the firmware decides whether it wants the interrupt. */
    if ((rd.ch[n].ctrl & CTRL_IRQ_MODE) && s) {
        static unsigned long long n_it;
        n_it++;
        if (n_it <= 10 || (n_it % 500) == 0)
            fprintf(stderr, "[rhea-dma] end-DMA -> INT10n (vec%d/bit%d) #%llu\n",
                    C54X_IT_DMA_VEC, C54X_IT_DMA_BIT, n_it);
        c54x_interrupt_ex(s, C54X_IT_DMA_VEC, C54X_IT_DMA_BIT);
    }
}

bool calypso_rhea_dma_rx_armed(void)
{
    if (!rhea_dma_on()) return true;      /* no DMA model: legacy path, keep everything */
    rhea_dma_init();
    uint16_t ctrl = rd.ch[1].ctrl;
    return (ctrl & CTRL_ENABLE) && (ctrl & CTRL_DIRECTION);
}

bool calypso_rhea_dma_one_shot(void)
{
    if (!rhea_dma_on()) return false;
    rhea_dma_init();
    uint16_t ctrl = rd.ch[1].ctrl;
    return (ctrl & CTRL_ENABLE) && (ctrl & CTRL_ONE_SHOT);
}

/* Stream pump: transfer the next full page(s) of the receiver into the
 * double buffer, if the channel is armed in continuous mode and enough words
 * are waiting. Called by the frame loop once the DSP has gone idle, i.e. once
 * its ISR has consumed the previous pair. Returns 1 when a transfer happened. */
int calypso_rhea_dma_pump(C54xState *s)
{
    if (!rhea_dma_on()) return 0;
    rhea_dma_init();
    if (s) rd_dsp = s; else s = rd_dsp;
    uint16_t ctrl = rd.ch[1].ctrl;
    if (!(ctrl & CTRL_ENABLE) || !(ctrl & CTRL_DIRECTION) || (ctrl & CTRL_ONE_SHOT))
        return 0;
    int page = rd.ch[1].algth / 2;
    if (page <= 0) return 0;
    int besoin = (ctrl & CTRL_CURRENT_PAGE) ? page : 2 * page;
    if (calypso_rif_level() < besoin) return 0;
    int avant = calypso_rif_level();
    calypso_rhea_dma_rx_request(s);
    return calypso_rif_level() < avant;
}

/* XIO bridge: the SAME register bank, seen from the DSP Rhea bus. §11.1 gives
 * both addresses for every register (FFFF:FCxx and XIO:FCxx); which bus reaches
 * them depends on DMA_ALLOC. Since the ARM hands the RIF channels to the DSP
 * (measured ALLOC_CONFIG=0x000C), the receive transfer is programmed through
 * here. */
bool calypso_rhea_dma_xio(bool write, uint16_t pa, uint16_t *val, uint16_t pc)
{
    if (pa < 0xFC00 || pa > 0xFCFF)
        return false;
    if (!rhea_dma_on())
        return false;
    hwaddr off = pa - 0xFC00;
    /* Dedupe, do not cap. The CTRL polling loop (a640/a646/a652) saturates a
     * fixed line cap within a few frames, and a journal truncated by its own
     * instrument makes "AAD is never written" unprovable. Each (direction, PA,
     * PC) triple is logged ONCE and then summarised periodically, so a first AAD
     * write always shows up however late it comes. */
    {
        struct seen { uint16_t pa, pc; uint8_t wr; unsigned long long n; };
        static struct seen tab[64];
        static int ntab = 0;
        int i, found = -1;
        for (i = 0; i < ntab; i++)
            if (tab[i].pa == pa && tab[i].pc == pc && tab[i].wr == (write ? 1 : 0))
                { found = i; break; }
        if (found < 0) {
            if (ntab < 64) { tab[ntab].pa = pa; tab[ntab].pc = pc;
                             tab[ntab].wr = write ? 1 : 0; tab[ntab].n = 1;
                             found = ntab++; }
            fprintf(stderr, "[rhea-dma] XIO %s PA=0x%04x (+0x%02x) %s0x%04x PC=0x%04x "
                    "— NOUVEAU site (acces DSP, canal cede par ALLOC_CONFIG)\n",
                    write ? "PORTW" : "PORTR", pa, (unsigned)off,
                    write ? "<- " : "-> ", val ? *val : 0, pc);
        } else {
            tab[found].n++;
            if ((tab[found].n % 20000) == 0)
                fprintf(stderr, "[rhea-dma] XIO %s PA=0x%04x PC=0x%04x : %llu passages "
                        "(valeur courante 0x%04x)\n", write ? "PORTW" : "PORTR",
                        pa, pc, tab[found].n, val ? *val : 0);
        }
    }
    if (write) {
        calypso_rhea_dma_write(NULL, off, *val, 2);
    } else {
        *val = (uint16_t)calypso_rhea_dma_read(NULL, off, 2);
    }
    return true;
}
