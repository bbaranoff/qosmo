/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * c54x_decode.c - operand decode: resolve_smem/lmem/xmem, condition codes.
 *
 * Split out of calypso_c54x.c on 2026-09-18. File map in c54x_internal.h.
 */
#include "c54x_internal.h"
#include "hw/arm/calypso/calypso_debug.h"

uint16_t resolve_smem(C54xState *s, uint16_t opcode, bool *indirect)
{
    if (opcode & 0x80) {
        /* Indirect addressing. Per SPRU131G 5.4.1 Table 5-5, bits 2:0 (ARF)
         * select the AR used by THIS instruction; ARP in ST0 is only then
         * updated to ARF, for the NEXT direct-Smem reference. Driving the
         * access from ARP instead of ARF is an off-by-one that makes every
         * indirect instruction modify the PREVIOUS one's AR. */
        *indirect = true;
        int mod = (opcode >> 3) & 0x0F;
        int nar = opcode & 0x07;
        int cur_arp = nar;
        uint16_t addr = s->ar[cur_arp];
        uint16_t ar_before = s->ar[cur_arp];  /* MOD-MISMATCH probe: base before post-modify */

        /* Probe: first use of each AR as an address base. A value still equal
         * to its reset content means read-before-write, i.e. the reset value
         * is load-bearing. Diagnostic only, to be removed. */
        {
            static uint8_t ar_used = 0;
            if (!(ar_used & (1 << cur_arp))) {
                ar_used |= (1 << cur_arp);
                fprintf(stderr, "[c54x] AR-FIRSTUSE AR%d=0x%04x PC=0x%04x insn=%u\n",
                        cur_arp, addr, s->pc, s->insn_count);
            }
        }

        if (cur_arp == 2 && addr < 0x0820) {
            /* @BEQUILLE - AR2_FLOOR_DROP  (CALYPSO_AR2_FLOOR_DROP, EQ1, default OFF)
             *   masks  : the AR2 address computation in the correlator, which underflows
             *            the 0x0800 DARAM buffer down into MMR space (0x00=IMR, 0x1E=XPC)
             *            and clobbers it. The drop redirects the access to 0xFFFF instead
             *            of fixing the pointer. Note the LOG is gated by the AR2-FLOOR
             *            debug token, the DROP is NOT.
             *   remove : once AR2 stays inside [0x0800,0x2b28) over the whole FB kernel
             *            (AR2-FLOOR token counter at 0 on a full run).
             */
            static int ar2_drop = -1;
            if (ar2_drop < 0) {
                const char *e = calypso_getenv("CALYPSO_AR2_FLOOR_DROP");
                ar2_drop = (e && *e == '1') ? 1 : 0;
            }
            if (calypso_debug_enabled("AR2-FLOOR"))
                C54_DBG("AR2-FLOOR",
                    "AR2=0x%04x < floor PC=0x%04x op=0x%04x BK=0x%04x A=%010llx insn=%u",
                    addr, s->pc, prog_fetch(s, s->pc), s->bk,
                    (unsigned long long)(s->a & 0xFFFFFFFFFFULL), s->insn_count);
            if (ar2_drop && addr < 0x0800)
                addr = 0xFFFF;  /* scratch address: keeps the write out of MMR space */
        }

        /* Post-modify */
        switch (mod) {
        case 0x0: /* *ARn */
            break;
        case 0x1: /* *ARn- */
            s->ar[cur_arp]--;
            break;
        case 0x2: /* *ARn+ */
            s->ar[cur_arp]++;
            break;
        case 0x3: /* *+ARn */
            addr = ++s->ar[cur_arp];
            break;
        /* MOD 4-11, canonical C54x order per binutils tic54x-dis.c:506-518:
         * 4=-0B 5=-0 6=+0 7=+0B 8=-% 9=-0% 10=+% 11=+0%.
         * Modes 4/7 are reverse-carry (bit-reversed) adds, used by the FB/SB
         * correlator; CALYPSO_ISA_BITREV=0 reverts them to a plain +/-AR0.
         * Circular modes 8-11 go through c54x_circ_ref, where BK=0 stays
         * LINEAR: the firmware issues STM #0,BK deliberately. */
        case 0x4: /* *ARn-0B - reverse carry */
            s->ar[cur_arp] = c54x_revcarry(s, s->ar[cur_arp], s->ar[0], 1);
            break;
        case 0x5: /* *ARn-0 */
            s->ar[cur_arp] -= s->ar[0];
            break;
        case 0x6: /* *ARn+0 */
            s->ar[cur_arp] += s->ar[0];
            break;
        case 0x7: /* *ARn+0B - reverse carry */
            s->ar[cur_arp] = c54x_revcarry(s, s->ar[cur_arp], s->ar[0], 0);
            break;
        case 0x8: /* *ARn-% (circular -1) */
            s->ar[cur_arp] = c54x_circ_ref(s->ar[cur_arp], -1, s->bk);
            break;
        case 0x9: /* *ARn-0% (circular -AR0) */
            s->ar[cur_arp] = c54x_circ_ref(s->ar[cur_arp], -(int16_t)s->ar[0], s->bk);
            break;
        case 0xA: /* *ARn+% (circular +1) */
            s->ar[cur_arp] = c54x_circ_ref(s->ar[cur_arp], +1, s->bk);
            break;
        case 0xB: /* *ARn+0% (circular +AR0) */
            s->ar[cur_arp] = c54x_circ_ref(s->ar[cur_arp], +(int16_t)s->ar[0], s->bk);
            break;
        /* Indirect modes 12..15 use a long-immediate operand from the next
         * program word. Encoding per tic54x-dis.c (MOD field = bits 6:3 of
         * the smem byte) and SPRU131G Table 5-9:
         *   12 : *AR(x)(lk)        - addr = AR(x) + lk, NO modify
         *   13 : *+AR(x)(lk)       - premod: AR(x) += lk; addr = AR(x)
         *   14 : *+AR(x)(lk)%      - premod circular: AR(x) = circ(AR(x)+lk)
         *   15 : *(lk)             - ABSOLUTE long address (lk itself)
         *
         * MOD=15 is an ABSOLUTE address, not AR(x)+lk: the PROM0 bootloader
         * at 0xb429 reads BL_ADDR_LO with `LDU *(0x0ffe), A`, and decoding it
         * as AR-relative yields AR(x)+0x0ffe and loads garbage. */
        case 0xC: /* *AR(x)(lk) */
            addr = s->ar[cur_arp] + prog_fetch(s, s->pc + 1);
            s->lk_used = true;
            break;
        case 0xD: /* *+AR(x)(lk) */
            s->ar[cur_arp] += prog_fetch(s, s->pc + 1);
            addr = s->ar[cur_arp];
            s->lk_used = true;
            break;
        case 0xE: { /* *+AR(x)(lk)% - circular */
            uint16_t lk = prog_fetch(s, s->pc + 1);
            uint16_t v  = s->ar[cur_arp] + lk;
            if (s->bk) {
                uint16_t base = s->ar[cur_arp] - (s->ar[cur_arp] % s->bk);
                if (v >= base + s->bk) v -= s->bk;
            }
            s->ar[cur_arp] = v;
            addr = v;
            s->lk_used = true;
            break;
        }
        case 0xF: /* *(lk) - absolute address */
            addr = prog_fetch(s, s->pc + 1);
            s->lk_used = true;
            break;
        }

        /* MOD-MISMATCH probe: recompute the silicon-correct AR in parallel and
         * compare. Pure observation, it must NOT alter execution. Reference:
         * tic54x-dis.c:506-518 for the canonical MOD order. Diagnostic only,
         * to be removed. */
        {
            int16_t  a0  = (int16_t)s->ar[0];
            uint16_t bk  = s->bk;
            uint16_t sil;               /* AR value expected on silicon */
            switch (mod) {
            case 0x0: sil = ar_before;                       break; /* *ar      */
            case 0x1: sil = (uint16_t)(ar_before - 1);       break; /* *ar-     */
            case 0x2: sil = (uint16_t)(ar_before + 1);       break; /* *ar+     */
            case 0x3: sil = (uint16_t)(ar_before + 1);       break; /* *+ar     */
            case 0x4: sil = (uint16_t)(ar_before - a0);      break; /* *ar-0B   */
            case 0x5: sil = (uint16_t)(ar_before - a0);      break; /* *ar-0    */
            case 0x6: sil = (uint16_t)(ar_before + a0);      break; /* *ar+0    */
            case 0x7: sil = (uint16_t)(ar_before + a0);      break; /* *ar+0B   */
            case 0x8: sil = c54x_circ_ref(ar_before, -1,  bk); break; /* *ar-%  */
            case 0x9: sil = c54x_circ_ref(ar_before, -a0, bk); break; /* *ar-0% */
            case 0xA: sil = c54x_circ_ref(ar_before, +1,  bk); break; /* *ar+%  */
            case 0xB: sil = c54x_circ_ref(ar_before, +a0, bk); break; /* *ar+0% */
            default:  sil = s->ar[cur_arp];                  break; /* 12-15 lk : skip */
            }
            /* which MOD encodings the firmware actually uses (first hit each) */
            static uint16_t mod_seen = 0;
            if (!(mod_seen & (1u << mod))) {
                mod_seen |= (1u << mod);
                fprintf(stderr, "[c54x] MOD-FIRSTHIT mod=%2d AR%d PC=0x%04x op=0x%04x insn=%u\n",
                        mod, cur_arp, s->pc, opcode, s->insn_count);
            }
            /* silicon vs model divergence (modes 0..11 only) */
            if (mod <= 0xB && sil != s->ar[cur_arp]) {
                static uint32_t mm_n[16] = {0};
                if (mm_n[mod] < 8)
                    fprintf(stderr, "[c54x] MOD-MISMATCH mod=%2d AR%d ar0=0x%04x bk=0x%04x "
                            "base=0x%04x qemu=0x%04x silicon=0x%04x PC=0x%04x op=0x%04x insn=%u\n",
                            mod, cur_arp, (uint16_t)a0, bk, ar_before,
                            s->ar[cur_arp], sil, s->pc, opcode, s->insn_count);
                mm_n[mod]++;
            }
        }

        /* Update ARP - only in compatibility mode (ST1.CMPT = 1, SPRU131G 5.4.1);
         * with CMPT = 0 the ARP is left alone (manual example MAR *AR3+, ARP stays 0). */
        if (s->st1 & ST1_CMPT)
            s->st0 = (s->st0 & ~ST0_ARP_MASK) | (nar << ST0_ARP_SHIFT);

        return addr;
    } else {
        /* Direct addressing. SPRU131G 5.3: CPL=0 -> dma = DP:offset (DP<<7 | 7 bits);
         * CPL=1 -> dma = SP + offset. [2026-09-20] CPL was ignored: the ROM runs its
         * C tasks with CPL=1 (0x71d5 `stm #0x6900, ST1` before `cala A`) and clears
         * it for the hand-written DSP kernels (0x7c20 `rsbx CPL`), so every
         * SP-relative local in the C parts was read from page DP instead. */
        *indirect = false;
        uint16_t offset = opcode & 0x7F;
        if (s->st1 & ST1_CPL) return (uint16_t)(s->sp + offset);
        return (dp(s) << 7) | offset;
    }
}

/* Resolve an Lmem (long-word, 32-bit) operand for the dual long-word family
 * (DADD/DSUB/DLD/DRSUB/DADST/DSUBT/DSADT, 0x50-0x5F). Returns the even-aligned
 * base address (data[addr]=high, data[addr+1]=low) and applies the LONG-operand
 * post-modify: the implicit unit step is 2 words, NOT 1 (SPRU172C, e.g. the
 * DADST example: "long-operand instruction, AR incremented/decremented by 2").
 * AR0-indexed and long-offset (lk) steps use their value as-is. Mirrors
 * resolve_smem's MOD field decode (bits 6:3). */
uint16_t resolve_lmem(C54xState *s, uint16_t opcode)
{
    /* [2026-09-20] The address is returned AS IS: the C54x reads the high word
     * at the given address and the low word at address ^ 1 (SPRU172C DADD
     * example, AR3 = 0101h: hi = data[0101h], lo = data[0100h]). Forcing the
     * address even swapped the halves of every odd-addressed long operand. */
    if (!(opcode & 0x80)) {
        /* Direct - dmad pair, no post-mod. CPL selects SP- or DP-relative. */
        uint16_t off = opcode & 0x7F;
        if (s->st1 & ST1_CPL) return (uint16_t)(s->sp + off);
        return (uint16_t)(((s->st0 & ST0_DP_MASK) << 7) | off);
    }
    int mod = (opcode >> 3) & 0x0F;
    int nar = opcode & 0x07;
    uint16_t addr = s->ar[nar];
    switch (mod) {
    case 0x0: break;                                              /* *ARn      */
    case 0x1: s->ar[nar] -= 2; break;                             /* *ARn-     */
    case 0x2: s->ar[nar] += 2; break;                             /* *ARn+     */
    case 0x3: s->ar[nar] += 2; addr = s->ar[nar]; break;          /* *+ARn     */
    case 0x4: s->ar[nar] = c54x_revcarry(s, s->ar[nar], s->ar[0], 1); break; /* *ARn-0B */
    case 0x5: s->ar[nar] -= s->ar[0]; break;                                 /* *ARn-0  */
    case 0x6: s->ar[nar] += s->ar[0]; break;                                 /* *ARn+0  */
    case 0x7: s->ar[nar] = c54x_revcarry(s, s->ar[nar], s->ar[0], 0); break; /* *ARn+0B */
    case 0x8: s->ar[nar] = c54x_circ_ref(s->ar[nar], -2, s->bk); break;                 /* *ARn-%  */
    case 0x9: s->ar[nar] = c54x_circ_ref(s->ar[nar], -(int16_t)s->ar[0], s->bk); break; /* *ARn-0% */
    case 0xA: s->ar[nar] = c54x_circ_ref(s->ar[nar], +2, s->bk); break;                 /* *ARn+%  */
    case 0xB: s->ar[nar] = c54x_circ_ref(s->ar[nar], +(int16_t)s->ar[0], s->bk); break; /* *ARn+0% */
    case 0xC: addr = (uint16_t)(s->ar[nar] + prog_fetch(s, s->pc + 1)); s->lk_used = true; break;
    case 0xD: s->ar[nar] += prog_fetch(s, s->pc + 1); addr = s->ar[nar]; s->lk_used = true; break;
    case 0xE: { uint16_t lk = prog_fetch(s, s->pc + 1);
                s->ar[nar] = c54x_circ_ref(s->ar[nar], (int16_t)lk, s->bk);
                addr = s->ar[nar]; s->lk_used = true; break; }
    case 0xF: addr = prog_fetch(s, s->pc + 1); s->lk_used = true; break;
    }
    return addr;
}

/* SP ledger for the IRQ push/pop asymmetry diagnostic. c54x_run counts
 * pushes and pops from the sign of the SP delta; c54x_interrupt_ex counts
 * IRQ entries with their word count. A net_words that drifts instead of
 * staying near 0 means an asymmetry, e.g. IRQ entry pushing one word while
 * FRET pops two, which walks SP until it wraps. */
struct c54x_g_sp_ledger_s g_sp_ledger;

/* Xmem operand decode, per the XMEM/XMOD/XARX macros of binutils tic54x.h:
 *   XMEM(OP) = opcode bits [7:4], the 4-bit Xmem nibble
 *   XMOD     = nibble bits [3:2] : 0=*AR, 1=*AR-, 2=*AR+, 3=*AR+0%
 *   XARX     = nibble bits [1:0] + 2, so AR2..AR5 only
 *
 * Xmem is INDIRECT-ONLY: unlike Smem it has no DP-relative direct mode.
 * Decoding an Xmem operand with resolve_smem reads the low byte as Smem
 * direct addressing whenever bit 7 is clear and lands the write in MMR space
 * (0x00-0x1F); one such write (PC=0x8a46, op=0x9918, STL B,*AR2) took
 * SP from 0x4800 to 0x0000 and then IMR to 0, idling the DSP for good.
 *
 * xmod=3 (*AR+0%) is CIRCULAR modulo BK via c54x_circ_ref, with BK=0 staying
 * linear; a plain `addr + AR0` here drifts over 16 bits and runs AR2 away.
 * The same rule applies to the other dual-operand handlers (MVDD, MAC D0-D9,
 * MASA DB, SQDST DC, ST||LD C8-CB).
 *
 * Beware: the 1/2 sign convention is NOT uniform across those handlers
 * (MVDD has 1=- 2=+, MAC has 1=+ 2=-). That inconsistency is unresolved. */
uint16_t resolve_xmem(C54xState *s, uint16_t op)
{
    uint8_t xmem  = (op >> 4) & 0xF;
    int     xar   = (xmem & 0x3) + 2;
    int     xmod  = (xmem & 0xC) >> 2;
    uint16_t addr = s->ar[xar];
    switch (xmod) {
    case 0: break;
    case 1: s->ar[xar] = addr - 1; break;
    case 2: s->ar[xar] = addr + 1; break;
    case 3: s->ar[xar] = c54x_circ_ref(addr, +(int16_t)s->ar[0], s->bk); break; /* *AR+0%, circular modulo BK (BK=0 -> linear) */
    }
    return addr;
}

/* PC ring buffer for the pre-IDLE trace. */
uint16_t pc_ring[256];
int pc_ring_idx = 0;

/* Evaluate a C54x condition from the LOW byte of the opcode, per the
 * condition_codes[] table of binutils opcodes/tic54x-opc.c:
 *   CC1=0x40 selects an accumulator test, CCB=0x08 selects B instead of A,
 *   test bits[2:0] = EQ 5, NEQ 4, LT 3, LEQ 7, GT 6, GEQ 2;
 *   AOV=0x70 ANOV=0x60; TC=0x30 NTC=0x20; C=0x0C NC=0x08; UNC=0x00.
 * The condition lives in the low byte, not in (op>>4)&0xF: that field only
 * agrees for UNC and AEQ, and getting it wrong flips conditional calls, so
 * pushes go missing and SP is over-popped into a runaway return address. */
bool c54x_cond_true(C54xState *s, uint8_t cc)
{
    if (cc == 0x00) return true;                       /* UNC */
    if (cc & 0x40) {                                   /* CC1 : test accu */
        int64_t acc = (cc & 0x08) ? sext40(s->b) : sext40(s->a);
        bool ov = (cc & 0x08) ? !!(s->st0 & (1 << 9))  /* OVB */
                              : !!(s->st0 & (1 << 8));  /* OVA */
        /* [2026-09-21] OVA/OVB are cleared once a conditional tests them */
        if ((cc & 0x70) == 0x70) { s->st0 &= (cc & 0x08) ? ~(1 << 9) : ~(1 << 8); return ov; }    /* AOV/BOV  */
        if ((cc & 0x70) == 0x60) { s->st0 &= (cc & 0x08) ? ~(1 << 9) : ~(1 << 8); return !ov; }   /* ANOV/BNOV */
        switch (cc & 0x07) {
        case 0x05: return acc == 0;                    /* EQ  */
        case 0x04: return acc != 0;                    /* NEQ */
        case 0x03: return acc <  0;                    /* LT  */
        case 0x07: return acc <= 0;                    /* LEQ */
        case 0x06: return acc >  0;                    /* GT  */
        case 0x02: return acc >= 0;                    /* GEQ */
        default:   return true;
        }
    }
    if ((cc & 0x30) == 0x30) return !!(s->st0 & ST0_TC);
    if ((cc & 0x30) == 0x20) return  !(s->st0 & ST0_TC);
    if ((cc & 0x0C) == 0x0C) return !!(s->st0 & ST0_C);
    if ((cc & 0x0C) == 0x08) return  !(s->st0 & ST0_C);
    return true;
}

/* Per-instruction interrupt LEVEL check, gated by CALYPSO_C54X_IRQ_LEVEL.
 * The base model only services interrupts at the c54x_interrupt_ex call edge,
 * so an IFR bit latched while INTM=1 is never taken afterwards. Real silicon
 * re-checks pending unmasked interrupts at every instruction boundary.
 *
 * This matters for the frame IT (bit12 / vector 28), which c54x_interrupt_ex
 * raises as an EDGE once per frame. The firmware keeps INTM=1 nearly all the
 * time (wait loop at 0xdde6 plus the critical sections around 0xb52x), and its
 * 5-instruction INTM=0 window rarely coincides with bit12 being pending:
 * vector 28 was dispatched 80 times over ~36000 frames, starving the FB
 * kernel. Two gated workarounds follow from that measurement:
 *  - LEVEL: hold bit12 asserted in the IFR, re-asserting it every instruction,
 *    until vector 28 actually vectors, so the next INTM 1->0 transition cannot
 *    miss it.
 *  - PRIO: when bit12 and a lower-priority bit (e.g. bit5/BRINT0) are pending
 *    in the same window, take bit12 first instead of the raw ctz, otherwise
 *    BRINT0 steals the rare window and re-masks with INTM=1. */
bool g_frame_it_level = false;

/* [2026-07-30] IFR-CLEAR-WHO: name every site that clears an IFR bit.
 *
 * All IFR clear sites funnel through here and pass their own name, because a
 * pending interrupt can vanish before the INTM=0 window opens: the window is
 * open 99% of the time, yet `intm0+pend` measures 0.
 *
 * Only UNMASKED bits (IMR set) are logged; clearing a masked bit is harmless.
 * A clear from a "vector-*" site is legitimate, the interrupt was serviced.
 * A clear from "mmio-write" on an unmasked, unserviced bit is a lost
 * interrupt. Gated by CALYPSO_IFR_CLEAR_WHO (default on), log rate capped.
 */
void c54x_ifr_clear(C54xState *s, uint16_t mask, const char *site)
{
    uint16_t before = s->ifr;
    s->ifr &= (uint16_t)~mask;
    uint16_t perdus = (uint16_t)(before & ~s->ifr & s->imr);   /* unmasked bits only */
    if (!perdus) {
        return;
    }
    static int en = -1;
    if (en < 0) {
        en = calypso_gate("CALYPSO_IFR_CLEAR_WHO", 1);
        if (en)
            fprintf(stderr, "[c54x] IFR-CLEAR-WHO arme : tout effacement d'un bit "
                    "DEMASQUE de l'IFR est nomme par son site\n");
    }
    if (!en) {
        return;
    }
    static unsigned long long par_bit[16];
    static unsigned nlog = 0;
    for (int b = 0; b < 16; b++) {
        if (!(perdus & (1u << b))) {
            continue;
        }
        par_bit[b]++;
        /* bit 5 = BRINT0 (I/Q delivery by the BSP) and bit 12 = frame are the
         * two bits of interest: log the first 20 of each, then periodically. */
        bool cible = (b == 5 || b == 12);
        if ((cible && par_bit[b] <= 20) || (par_bit[b] % 5000) == 0) {
            nlog++;
            fprintf(stderr, "[c54x] IFR-CLEAR-WHO bit=%d%s #%llu site=%s "
                    "IFR 0x%04x->0x%04x IMR=0x%04x INTM=%d PC=0x%04x insn=%u\n",
                    b, (b == 5) ? "(BRINT0)" : (b == 12) ? "(frame)" : "",
                    (unsigned long long)par_bit[b], site, before, s->ifr, s->imr,
                    (s->st1 & ST1_INTM) ? 1 : 0, s->pc, s->insn_count);
        }
    }
    if ((nlog % 200) == 199)
        fprintf(stderr, "[c54x] IFR-CLEAR-WHO resume : bit5=%llu bit12=%llu insn=%u\n",
                (unsigned long long)par_bit[5], (unsigned long long)par_bit[12],
                s->insn_count);
}


