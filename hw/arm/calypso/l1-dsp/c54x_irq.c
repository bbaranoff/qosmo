/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * c54x_irq.c - interrupts: IFR/IMR, frame IT, level hold.
 *
 * Split out of calypso_c54x.c on 2026-09-18 (one file per role).
 * File map in c54x_internal.h.
 */
#include "c54x_internal.h"
#include "hw/arm/calypso/calypso_debug.h"

void calypso_inth_arm_ack(void);
/* @BEQUILLE - FRAME_IT_LEVEL  (CALYPSO_FRAME_IT_LEVEL, EQ1, default OFF)
 *   masque  : the firmware's INTM=0 window, too rare to catch the frame IT.
 *             Re-asserts IFR bit 12 on EVERY instruction until vector 28 is
 *             taken, because the c54x IFR latches events and has no level mode.
 *   retirer : once the firmware's INTM duty cycle is wide enough to catch the
 *             IT as it comes.
 */
bool frame_it_level_on(void)
{
    static int c = -1;
    if (c < 0) { const char *e = calypso_getenv("CALYPSO_FRAME_IT_LEVEL"); c = (e && *e == '1') ? 1 : 0; }
    return c;
}
/* @BEQUILLE - FRAME_IT_PRIO  (CALYPSO_FRAME_IT_PRIO, EQ1, default OFF)
 *   masque  : interrupt priority. Forces b=12 instead of ctz(pend) so the frame
 *             IT wins over BRINT0/bit 5; on the c54x, priority is fixed by the
 *             vector number and is not configurable.
 *   retirer : once BRINT0 and the frame IT no longer compete for the same
 *             window (BSP delivery at the right rate).
 */
bool frame_it_prio_on(void)
{
    static int c = -1;
    if (c < 0) { const char *e = calypso_getenv("CALYPSO_FRAME_IT_PRIO"); c = (e && *e == '1') ? 1 : 0; }
    return c;
}

bool c54x_irq_level_check(C54xState *s)
{
    static int en = -1;
    if (en < 0) { const char *_d = calypso_getenv("CALYPSO_DSP"); en = (calypso_getenv("CALYPSO_C54X_IRQ_LEVEL") || (_d && !strcmp(_d, "c54x"))) ? 1 : 0; }
    if (!en) return false;
    /* Level hold: keep bit 12 pending in the IFR until the frame IT has been
     * vectored (vec 28), so the next INTM=0 window picks it up. */
    if (g_frame_it_level && frame_it_level_on()) {
        s->ifr |= (1u << 12);
    }
    /* [2026-08-04] INT10n (bit 14) is a LEVEL line: the DMA line.
     *
     * CAL000 5.1 gives the sense of each line:
     *     INT0n  (level) -> RIF receive        INT8n  (edge) -> TPU frame
     *     INT1n  (level) -> RIF transmit       INT9n  (edge) -> TPU programmable
     *     INT10n (level) -> DMA interrupt      INT7n  (edge) -> CYPHER
     * Posting bit 14 once on the edge loses the event whenever INTM is 1 at
     * that instant: over the 15 logged vec=30 requests, IMR=0x52ed (bit 14
     * unmasked) and IFR=0x4000 (bit 14 pending) with INTM=1 all 15 times, and
     * "IRQ-LEVEL take" never fires. The DSP then never services DMA
     * completion, its internal queues overflow (116 writes at 0x434e/0x434f)
     * and it raises DSP_ERR_DMA_PEND.
     *
     * The level source is the channel IRQ_STATE, which CAL207 11.3.5 describes
     * as "cleared after being read": the line drops when the firmware reads the
     * register, as on silicon. */
    if (calypso_rhea_dma_irq_level()) {
        s->ifr |= (1u << C54X_IT_DMA_BIT);
    }
    /* [2026-07-22] LEVELCHK-DBG (gated CALYPSO_AR0_DEBUG): with IMR != 0 (window
     * armed), report which gate keeps the frame IT from being taken - INTM vs
     * IPTR vs pend=0. */
    {
        static int lcdbg = -1;
        if (lcdbg < 0) lcdbg = calypso_gate("CALYPSO_AR0_DEBUG", 0);
        if (lcdbg && s->imr && s->insn_count > 4000) {   /* skip boot-reset noise */
            static unsigned lc = 0;
            uint16_t iptr = (s->pmst >> PMST_IPTR_SHIFT) & 0x1FF;
            uint16_t pend = (uint16_t)(s->ifr & s->imr);
            if (lc++ < 150)
                fprintf(stderr, "[c54x] LEVELCHK-DBG INTM=%d delay=%d IPTR=0x%03x "
                        "IFR=0x%04x IMR=0x%04x pend=0x%04x PC=0x%04x insn=%u -> %s\n",
                        !!(s->st1 & ST1_INTM), s->delay_slots, iptr,
                        s->ifr, s->imr, pend, s->pc, s->insn_count,
                        (s->st1 & ST1_INTM) ? "BLOCK:INTM" :
                        s->delay_slots ? "BLOCK:delay" :
                        (iptr == 0x1FF) ? "BLOCK:IPTR=0x1FF" :
                        (!pend) ? "BLOCK:pend=0(IFR&IMR)" : "WOULD-TAKE!");
        }
    }
    /* [2026-07-23] LEVELCHK-EMPIRICAL (unconditional, capped): "IRQ-LEVEL take"
     * never fires in native runs although INTM-TRANS shows IFR=0x1020/0x1030
     * (bit 5 BRINT0 + bit 12 frame pending) right at the INTM 1->0 (RETE)
     * moments. Traces every early-return path of this function so the blocking
     * gate is observed rather than inferred. */
    {
        static unsigned _lcn = 0;
        static uint32_t _last_insn = 0xFFFFFFFFu;
        bool _intm = !!(s->st1 & ST1_INTM);
        bool _delay = s->delay_slots != 0;
        uint16_t _iptr = (s->pmst >> PMST_IPTR_SHIFT) & 0x1FF;
        uint16_t _pend = (uint16_t)(s->ifr & s->imr);
        /* [2026-07-23] DEDUP: while RPT is active the same instruction is
         * re-executed hundreds of times at the same PC WITHOUT insn_count
         * advancing, which burns the whole cap on one repeat loop. Log only on
         * a new insn_count so the cap covers distinct instructions. */
        if (_pend && _iptr != 0x1FF && _lcn < 5000 && s->insn_count != _last_insn) {
            _last_insn = s->insn_count;
            _lcn++;
            fprintf(stderr, "[c54x] LEVELCHK-EMPIRICAL #%u PC=0x%04x INTM=%d delay=%d "
                    "IPTR=0x%03x IFR=0x%04x IMR=0x%04x pend=0x%04x insn=%u idle=%d "
                    "rpt_active=%d rpt_count=%u -> %s\n",
                    _lcn, s->pc, _intm, _delay, _iptr, s->ifr, s->imr, _pend, s->insn_count, s->idle,
                    s->rpt_active, s->rpt_count,
                    _intm ? "BLOCKED:INTM=1" : _delay ? "BLOCKED:delay_slots" :
                    (_iptr == 0x1FF) ? "BLOCKED:IPTR=0x1FF" : "WOULD-DISPATCH");
        }
    }
    /* [2026-07-30] LEVELCHK-WINDOW - count the window, not the boring case.
     *
     * LEVELCHK-EMPIRICAL is capped at 5000 lines and burns them all on
     * BLOCKED:INTM=1 before insn=14M, while the question is the opposite one:
     * when INTM drops, is the pending IT seen, and is it taken? Measured
     * 2026-07-30: "IRQ-LEVEL take" = 0 over 52M instructions for 739 traced
     * INTM 1->0 transitions, with pend=0x1020 (bit 5 BRINT0 + bit 12 frame)
     * permanently set; the window lasts about one instruction (1->0 at
     * insn 187431877, 0->1 at 187431878).
     *
     * Counters, not lines: the summary shows at a glance whether the window is
     * even SEEN here. w_intm0 staying at 0 while INTM-TRANS counts 1->0
     * transitions means this function is not called inside the window - an
     * ordering problem in the execution loop, not a guard. */
    {
        static unsigned long long w_calls = 0, w_intm0 = 0, w_pend = 0, w_ready = 0;
        static unsigned w_log = 0;
        uint16_t _p = (uint16_t)(s->ifr & s->imr);
        uint16_t _ip = (uint16_t)((s->pmst >> PMST_IPTR_SHIFT) & 0x1FF);
        w_calls++;
        if (!(s->st1 & ST1_INTM)) {
            w_intm0++;
            if (_p) {
                w_pend++;
                if (s->delay_slots == 0 && _ip != 0x1FF) w_ready++;
                if (w_log < 20) {
                    w_log++;
                    fprintf(stderr, "[c54x] LEVELCHK-WINDOW #%u FENETRE OUVERTE "
                            "PC=0x%04x pend=0x%04x IFR=0x%04x IMR=0x%04x delay=%d "
                            "IPTR=0x%03x insn=%u -> %s\n",
                            w_log, s->pc, _p, s->ifr, s->imr, s->delay_slots, _ip,
                            s->insn_count,
                            (s->delay_slots == 0 && _ip != 0x1FF) ? "PRISE"
                              : (s->delay_slots ? "BLOQUE:delay" : "BLOQUE:IPTR=0x1FF"));
                }
            }
        }
        if ((w_calls % 2000000ULL) == 0)
            fprintf(stderr, "[c54x] LEVELCHK-WINDOW resume : appels=%llu intm0=%llu "
                    "intm0+pend=%llu prises=%llu insn=%u\n",
                    (unsigned long long)w_calls, (unsigned long long)w_intm0,
                    (unsigned long long)w_pend, (unsigned long long)w_ready,
                    s->insn_count);
    }

    if ((s->st1 & ST1_INTM) || s->delay_slots != 0) return false;
    /* Do not vector until the ROM has relocated IPTR: reset value 0x1ff puts the
     * vector table at 0xff80, which is garbage. Wait for a relocated IPTR
     * (typically 0x001). */
    if (((s->pmst >> PMST_IPTR_SHIFT) & 0x1FF) == 0x1FF) return false;
    uint16_t pend = (uint16_t)(s->ifr & s->imr);
    if (!pend) return false;
    int b = __builtin_ctz(pend);          /* lowest set bit = highest priority */
    /* Priority: the frame IT (bit 12 / vec 28) outranks lower bits such as
     * BRINT0 (bit 5), which would steal the rare window and re-mask INTM. */
    if (frame_it_prio_on() && (pend & (1u << 12))) {
        b = 12;
    }
    int vec = b + 16;                     /* C54x: maskable IMR bit b -> vector b+16 */
    /* Bit 3 is the DSP timer: the frame IT is raised on bit 12 / vector 28 at
     * the source, so vec = b + 16 holds with no exception. */
    c54x_ifr_clear(s, (uint16_t)(1u << b), "vector-level");
    if (b == 12) g_frame_it_level = false;   /* frame IT vectored: release the level hold */
    s->sp--; data_write(s, s->sp, (uint16_t)s->pc);
    /* [2026-07-22] Push XPC ONLY in extended mode (xpc != 0), as a standard c54x
     * does. The firmware leaves the ISR through POPM ST1 + RCD (pops 1 word =
     * PC), not RETE (pops 2). With xpc == 0 (no paging), pushing XPC leaves an
     * orphan word that is never popped: SP drifts +1 per IT until it wraps, the
     * bootstub RET at 0xab38 pops mem[0x5ac8]=0 instead of mem[0x5ac7], and the
     * PC=0 storm follows. Legacy behaviour: CALYPSO_IT_PUSH_XPC_ALWAYS=1. */
    {
        static int always = -1;
        if (always < 0) { const char *e = calypso_getenv("CALYPSO_IT_PUSH_XPC_ALWAYS");
                          always = (e && *e != 0) ? 1 : 0; }
        if (always || s->xpc != 0) { s->sp--; data_write(s, s->sp, s->xpc); }
    }
    s->st1 |= ST1_INTM;
    s->xpc = 0;
    uint16_t iptr = (s->pmst >> PMST_IPTR_SHIFT) & 0x1FF;
    s->pc = (uint16_t)((iptr * 0x80) + vec * 4);
    static unsigned lvln = 0;
    if (lvln++ < 60)
        fprintf(stderr, "[c54x] IRQ-LEVEL take bit=%d vec=%d -> PC=0x%04x "
                "IPTR=0x%03x IMR=0x%04x IFR=0x%04x insn=%u\n",
                b, vec, s->pc, iptr, s->imr, s->ifr, s->insn_count);
    return true;
}


/* [2026-07-28] Unified gate for emulation fixes.
 *   CALYPSO_FIXES=FIX_ONE,FIX_TWO   enables the named fixes
 *   CALYPSO_FIXES=all               enables them all
 *   (unset)                         none; original behaviour strictly unchanged
 *
 * Protocol: land all safe fixes behind this gate at once, test under full load
 * (camp + LU + SMS, not a bare boot), and once a fix is confirmed delete the
 * CONDITION, not the fix - drop the `if (calypso_fix_enabled("FIX_..."))` and
 * its braces so the code becomes unconditional and the name leaves the gate
 * list. This is a temporary airlock, never a configuration option: an airlock
 * empties. Never let a validated fix grow old in it. */
/* [2026-08-04] FIX_F4XX_SRCDST - the F4xx/F5xx family had src and dst swapped.
 *
 * TI SPRU172C (Mnemonic Instruction Set, March 2001), instruction pages:
 *     ADD form 9 : 15..10 = 111101   bit9 = S   bit8 = D   7..5 = 000  SHIFT
 *     LD  form   : 15..10 = 111101   bit9 = S   bit8 = D   7..5 = 010  SHIFT
 *     (same field layout for SUB, SFTA, SFTL, NEG, ABS, MACA, *,ASM,*)
 * So bit 9 = SRC and bit 8 = DST, as in the F0-F3 family (OR form 4), where the
 * one-word block already had it right. 22 handlers had the opposite assignment
 * (`src = (op>>8)&1, dst = (op>>9)&1`).
 *
 * The blast radius is why the escape gate exists: CALYPSO_FIX_F4XX_SRCDST=0
 * restores the pre-2026-08-04 behaviour exactly, to isolate a regression
 * without touching code. Default 1: the fix matches the vendor documentation.
 *
 * WARNING: not validated under load. No bench currently combines "c54x active"
 * with "full LU" (shunt_legit forces CALYPSO_DSP_RUN_C54X=0). Remove the gate
 * only once a camp->LU run has exercised it. */
