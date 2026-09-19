/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * c54x_probes.c - probes, traces and instrumentation (diagnostics only)
 *
 * Split out of calypso_c54x.c on 2026-09-18 (per-role file split).
 * File map in c54x_internal.h.
 */
#include "c54x_internal.h"

/* ================================================================
 * Memory access
 * ================================================================ */

/* Forward decl: used by data_write() VECDUMP at MMR_PMST. */
uint16_t prog_read(C54xState *s, uint32_t addr);
uint16_t prog_fetch(C54xState *s, uint16_t pc);

/* Propagated by D_BURST_D probe, consumed by A_CD-BY-BURST correlation. */
uint16_t g_last_d_burst_d;

/* === Generic watch-write zone helper ===
 *
 * One monitored memory range: per-PC counter, throttled log, periodic summary.
 * Callers in c54x_mem.c: COEFFS, A_CD, BLK-SRC, DISP-TBL, TRAMPO.
 *
 * Cost: 512 KB of static storage per zone (per_pc[0x10000] * uint64_t). */

bool watch_write_zone_check(C54xState *s, uint16_t addr, uint16_t val,
                                   const char *name,
                                   uint16_t lo, uint16_t hi,
                                   WatchWriteState *st)
{
    if (addr < lo || addr > hi) return false;
    uint16_t exec_pc = s->last_exec_pc;
    st->per_pc[exec_pc]++;
    st->total++;
    bool should_log =
        st->total <= 500
        || exec_pc != st->last_exec_pc
        || (s->insn_count - st->last_log_insn) > 100000;
    if (should_log) {
        const char *wk_name[] = {
            "UNK", "F3", "8x", "77", "76", "PSHM",
            "RET", "IRQ_ACK", "ARM_MMIO", "RES_AR", "OTHER"
        };
        uint8_t wk = s->writer_kind;
        const char *wkn = (wk < sizeof(wk_name)/sizeof(wk_name[0]))
                          ? wk_name[wk] : "??";
        fprintf(stderr,
                "[c54x] %s-WR #%llu addr=0x%04x val=0x%04x "
                "exec_pc=0x%04x cur_pc=0x%04x cur_op=0x%04x wk=%s "
                "AR3=%04x AR4=%04x AR5=%04x insn=%u\n",
                name, (unsigned long long)st->total, addr, val,
                exec_pc, s->pc, s->prog[s->pc], wkn,
                s->ar[3], s->ar[4], s->ar[5], s->insn_count);
        st->last_exec_pc = exec_pc;
        st->last_log_insn = s->insn_count;
    }
    if (s->insn_count - st->last_summary_insn >= 5000000) {
        st->last_summary_insn = s->insn_count;
        /* Keep the -WR- infix: `<NAME>-WR-SUMMARY` must stay consistent with
         * the per-hit `<NAME>-WR #N` lines and with the regex tests. */
        fprintf(stderr, "[c54x] %s-WR-SUMMARY insn=%u total=%llu",
                name, s->insn_count, (unsigned long long)st->total);
        for (int p = 0; p < 0x10000; p++) {
            if (st->per_pc[p]) {
                fprintf(stderr, " pc[0x%04x]=%llu",
                        p, (unsigned long long)st->per_pc[p]);
            }
        }
        fprintf(stderr, "\n");
    }
    return true;
}

/* === FB-det timing/content stats ===
 *
 * Filled by c54x_mem.c over the ~928 fires of PC 0x8f51:
 *   - AR4 inside/outside [0x2bc0..0x2bff] -> addressing vs timing
 *   - insn delta since the last write, per cluster (compute/clear/pattern)
 *   - val[AR4] histogram: zero / 0xfffe sentinel / other
 *
 * Reading the counters:
 *   ar4_in_zone < 100%            addressing bug, AR4 points outside the zone
 *   delta_clear < delta_compute   timing race, the clear wins
 *   delta_compute >> 1M           compute never lands in the fire window
 *   other > 0                     some sweeps do see data
 */
struct c54x_g_fb_det_timing_s g_fb_det_timing;

/* === Generic ARn write tracer with provenance (AR0..AR7) ===
 * Gate: AR-TRACE. Env `CALYPSO_AR_TRACE` is a hex bitmask of the AR indices
 * to trace, default 0xFF (all); 0x14 = AR2 + AR4, 0x04 = AR2 only.
 *
 * Hooked from `case MMR_AR0..AR7` in data_write_locked, so it observes MMR
 * loads only, never auto-modify; deltas in [-3, 3] are skipped as increment
 * noise. A write of 0 is flagged separately (suspect MMR clobber via
 * STL A,*AR-).
 *
 * Opcode classification names where the value comes from:
 *   STM-#lk    ROM immediate, a deliberate firmware update
 *   MVDM-mem   load from memory
 *   MVMM       copy of another AR
 *   STM Smem   indirect memory load
 *   STLM-A     from accumulator A */
ArEntry  g_ar_hist[8][AR_HIST_MAX];
unsigned g_ar_used[8]    = {0};
unsigned g_ar_total[8]   = {0};
unsigned g_ar_mask       = 0;
int      g_ar_enabled    = -1;
unsigned g_ar_log_cap    = 50;

void ar_write_track(C54xState *s, unsigned idx, uint16_t new_val)
{
    /* AR3-PRELOAD: read-only, always on, capped at 120 lines. Logs AR3 loads
     * landing in or around the I/Q buffer [0x2a00..0x2c00), whose end is
     * 0x2b28. Only MMR loads (STM/STLM/MVDM) reach ar_write_track, never
     * auto-increment, so an OUT-OF-BUF load (>= 0x2b28) points at the loading
     * instruction, while the absence of one means an out-of-range AR3 comes
     * from increments instead (loop too long / buffer too short). */
    if (idx == 3 && new_val >= 0x2a00 && new_val < 0x2c00) {
        static unsigned ar3p_n = 0;
        if (ar3p_n < 120) {
            uint16_t op = prog_fetch(s, s->pc);
            fprintf(stderr, "[c54x] AR3-PRELOAD PC=0x%04x op=0x%04x AR3 %04x->%04x "
                    "%s insn=%u\n", s->pc, op, s->ar[3], new_val,
                    new_val >= 0x2b28 ? "*** OUT-OF-BUF ***" : "(in-buf)",
                    s->insn_count);
            ar3p_n++;
        }
    }
    if (g_ar_enabled < 0) {
        const char *e = getenv("CALYPSO_AR_TRACE");
        g_ar_mask = (e && *e) ? (unsigned)strtoul(e, NULL, 0) : 0xFFu;
        g_ar_enabled = calypso_debug_enabled("AR-TRACE") ? 1 : 0;
        if (g_ar_enabled) {
            fprintf(stderr,
                "[c54x] AR-TRACE enabled, mask=0x%02x (AR0..AR7), "
                "log_cap=%u hist_max=%u\n",
                g_ar_mask, g_ar_log_cap, AR_HIST_MAX);
        }
    }
    if (g_ar_enabled <= 0) return;
    if (idx >= 8) return;
    if (!(g_ar_mask & (1u << idx))) return;

    uint16_t old_val = s->ar[idx];
    int32_t delta = (int32_t)new_val - (int32_t)old_val;
    if (delta >= -3 && delta <= 3) return;  /* skip auto-modify noise */
    g_ar_total[idx]++;

    uint16_t op = prog_fetch(s, s->pc);
    const char *kind = "MISC";
    if ((op & 0xFF80) == 0x7700) kind = "STM-#lk";
    else if ((op & 0xFF00) == 0x8400) kind = "STLM-A";
    else if ((op & 0xFF00) == 0x8600) kind = "MVDM-mem";
    else if ((op & 0xFF00) == 0x8800) kind = "MVMM";
    else if ((op & 0xF800) == 0x8000) kind = "STL-A";

    unsigned i;
    for (i = 0; i < g_ar_used[idx]; i++) {
        if (g_ar_hist[idx][i].pc == s->pc) {
            g_ar_hist[idx][i].count++;
            g_ar_hist[idx][i].val_last = new_val;
            g_ar_hist[idx][i].op_last = op;
            break;
        }
    }
    if (i == g_ar_used[idx] && g_ar_used[idx] < AR_HIST_MAX) {
        g_ar_hist[idx][i].pc = s->pc;
        g_ar_hist[idx][i].op_last = op;
        g_ar_hist[idx][i].val_last = new_val;
        g_ar_hist[idx][i].count = 1;
        g_ar_used[idx]++;
    }
    if (g_ar_total[idx] <= g_ar_log_cap) {
        fprintf(stderr,
            "[c54x] AR%u-W #%u %s @insn=%u PC=0x%04x op=0x%04x  "
            "AR%u %04x → %04x (Δ=%+d)  A=%010llx SP=%04x\n",
            idx, g_ar_total[idx], kind, s->insn_count, s->pc, op,
            idx, old_val, new_val, delta,
            (unsigned long long)(s->a & 0xFFFFFFFFFFULL), s->sp);
    }
    /* Semantic distinction, kept explicit in the log:
     * - STM-#lk / LD-#k  deliberate AR update from a ROM immediate, the AR
     *                    changes by firmware design
     * - anything else    side effect of an MMR write where the AR happens to
     *                    point at its own MMR slot (self-alias). A coincidence,
     *                    not an explicit AR update. */
    if (new_val == 0) {
        int deliberate = ((op & 0xFF80) == 0x7700);  /* STM-#lk only */
        fprintf(stderr,
            "[c54x] AR%u-W ZERO %s @insn=%u PC=0x%04x op=0x%04x AR%u←0 "
            "(kind=%s)\n",
            idx,
            deliberate ? "DELIBERATE" : "SIDE-EFFECT",
            s->insn_count, s->pc, op, idx, kind);
    }
}

/* === A accumulator provenance tracer ===
 * Records A's last writer at the top-of-loop chokepoint: A is compared with
 * the previous iteration and, on a change, the writer's PC and opcode are
 * kept. When the trigger PC fires, A and its last writer are dumped, which
 * tells a deliberate A=0 (mask-all by design) from a diverged one.
 *
 * Gate: A-TRACE. Env CALYPSO_A_TRACE_PC sets the trigger PC (hex, 0 when the
 * gate is on but the variable is unset). Zero cost when off. */
int64_t  g_a_last_value      = 0;
uint16_t g_a_last_writer_pc  = 0;
uint16_t g_a_last_writer_op  = 0;
unsigned g_a_last_writer_insn = 0;
int      g_a_trace_enabled   = -1;
uint16_t g_a_trace_pc        = 0xFFFF;
unsigned g_a_trace_hits      = 0;
unsigned g_a_trace_log_cap   = 50;

void a_track_init_lazy(void)
{
    if (g_a_trace_enabled >= 0) return;
    const char *e = getenv("CALYPSO_A_TRACE_PC");
    if (calypso_debug_enabled("A-TRACE")) {
        g_a_trace_pc = (e && *e) ? (uint16_t)strtoul(e, NULL, 0) : 0;
        g_a_trace_enabled = 1;
        fprintf(stderr,
            "[c54x] A-TRACE enabled, trigger PC=0x%04x log_cap=%u\n",
            g_a_trace_pc, g_a_trace_log_cap);
    } else {
        g_a_trace_enabled = 0;
    }
}

void a_track_iter(C54xState *s, uint16_t prev_pc, uint16_t prev_op)
{
    if (g_a_trace_enabled <= 0) return;
    /* A changed: the instruction that just ran is its writer. */
    if (s->a != g_a_last_value) {
        g_a_last_writer_pc   = prev_pc;
        g_a_last_writer_op   = prev_op;
        g_a_last_writer_insn = s->insn_count;
        g_a_last_value       = s->a;
    }
    /* Trigger : PC about to execute matches target */
    if (s->pc == g_a_trace_pc) {
        g_a_trace_hits++;
        int a_zero = ((s->a & 0xFFFF) == 0);
        /* Log the first N hits for context, plus every A_low=0 hit whatever
         * the count: a silent cap hides late critical events, as it did for
         * the IMR clobber at insn=253328. */
        if (g_a_trace_hits <= g_a_trace_log_cap || a_zero) {
            fprintf(stderr,
                "[c54x] A-AT-PC #%u @insn=%u PC=0x%04x  A=%010llx (low=0x%04x, %s) "
                "last_writer: PC=0x%04x op=0x%04x @insn=%u\n",
                g_a_trace_hits, s->insn_count, s->pc,
                (unsigned long long)(s->a & 0xFFFFFFFFFFULL),
                (unsigned)(s->a & 0xFFFF),
                a_zero ? "A_low=0 → STL clobber zone" : "A_low≠0",
                g_a_last_writer_pc, g_a_last_writer_op,
                g_a_last_writer_insn);
        }
    }
}

/* === AR6 windowed snapshot at a trigger PC ===
 * Captures AR6, B and AR6's last writer on each fire of the trigger PC,
 * restricted to [insn_lo, insn_hi] because such a PC fires 10M+ times
 * (0x821a does). At a fire, AR6=0 means the buffer base diverged and the
 * write lands on IMR, AR6=0x16 means AR6 aliases its own MMR slot.
 *
 * The last writer is tracked by top-of-loop comparison, like the A tracer.
 *
 * Gate: AR6-AT. Env:
 *   CALYPSO_AR6_AT_PC=0x821a    trigger PC
 *   CALYPSO_AR6_WIN_LO=3619500  insn window start
 *   CALYPSO_AR6_WIN_HI=3619810  insn window end (one outer-loop iteration)
 *   CALYPSO_AR6_AT_LOG_CAP=200  max log lines (default 200)
 */
uint16_t g_ar6_last_value     = 0;
uint16_t g_ar6_last_writer_pc = 0;
uint16_t g_ar6_last_writer_op = 0;
unsigned g_ar6_last_writer_insn = 0;
int      g_ar6_at_enabled     = -1;
uint16_t g_ar6_at_pc          = 0xFFFF;
unsigned g_ar6_at_win_lo      = 0;
unsigned g_ar6_at_win_hi      = 0;
unsigned g_ar6_at_hits        = 0;
unsigned g_ar6_at_log_cap     = 200;

void ar6_at_init_lazy(void)
{
    if (g_ar6_at_enabled >= 0) return;
    const char *e = getenv("CALYPSO_AR6_AT_PC");
    if (calypso_debug_enabled("AR6-AT")) {
        g_ar6_at_pc = (e && *e) ? (uint16_t)strtoul(e, NULL, 0) : 0;
        g_ar6_at_enabled = 1;
        const char *lo = getenv("CALYPSO_AR6_WIN_LO");
        const char *hi = getenv("CALYPSO_AR6_WIN_HI");
        const char *cap = getenv("CALYPSO_AR6_AT_LOG_CAP");
        g_ar6_at_win_lo = (lo && *lo) ? (unsigned)strtoul(lo, NULL, 0) : 0;
        g_ar6_at_win_hi = (hi && *hi) ? (unsigned)strtoul(hi, NULL, 0) : 0xFFFFFFFFu;
        g_ar6_at_log_cap = (cap && *cap) ? (unsigned)strtoul(cap, NULL, 0) : 200;
        fprintf(stderr,
            "[c54x] AR6-AT-PC enabled, trigger PC=0x%04x window=[%u..%u] cap=%u\n",
            g_ar6_at_pc, g_ar6_at_win_lo, g_ar6_at_win_hi, g_ar6_at_log_cap);
    } else {
        g_ar6_at_enabled = 0;
    }
}

void ar6_at_iter(C54xState *s, uint16_t prev_pc, uint16_t prev_op)
{
    if (g_ar6_at_enabled <= 0) return;
    /* Track AR6 last writer */
    if (s->ar[6] != g_ar6_last_value) {
        g_ar6_last_writer_pc   = prev_pc;
        g_ar6_last_writer_op   = prev_op;
        g_ar6_last_writer_insn = s->insn_count;
        g_ar6_last_value       = s->ar[6];
    }
    /* Trigger: PC about to execute matches AND we are inside the window. */
    if (s->pc == g_ar6_at_pc &&
        s->insn_count >= g_ar6_at_win_lo &&
        s->insn_count <= g_ar6_at_win_hi) {
        g_ar6_at_hits++;
        if (g_ar6_at_hits <= g_ar6_at_log_cap) {
            uint16_t ar6 = s->ar[6];
            const char *regime;
            if (ar6 == 0)              regime = "AR6=0 → addr=IMR (BUFFER BASE DIVERGENCE)";
            else if (ar6 == 0x16)      regime = "AR6=0x16 → addr=MMR_AR6 (SELF-ALIAS)";
            else if (ar6 < 0x20)       regime = "AR6 in MMR zone";
            else                        regime = "AR6 normal";
            fprintf(stderr,
                "[c54x] AR6-AT-PC #%u @insn=%u PC=0x%04x  AR6=0x%04x (%s) "
                "B=%010llx (high=0x%04x)  last_writer: PC=0x%04x op=0x%04x @insn=%u\n",
                g_ar6_at_hits, s->insn_count, s->pc,
                ar6, regime,
                (unsigned long long)(s->b & 0xFFFFFFFFFFULL),
                (unsigned)((s->b >> 16) & 0xFFFF),
                g_ar6_last_writer_pc, g_ar6_last_writer_op,
                g_ar6_last_writer_insn);
        }
    }
}


/* RSBX INTM hit counter (cheap probe). */
uint64_t g_rsbx_intm_hits = 0;
int      g_rsbx_intm_enabled = -1;

/* DISP-ENTRY discriminator: captures the interrupt-preemption context. The
 * C54x does not push ST0/DP on interrupt entry, so the ISR inherits the DP of
 * the code it preempted. Bad dispatcher entries (DP != 0x124) correlating with
 * a recent interrupt therefore mean preemption, not a clobbered DP. */
uint64_t g_last_intr_insn  = 0;      /* insn_count of the last interrupt serviced */;
int      g_last_intr_vec   = -1;     /* vector of the last interrupt */;
uint16_t g_last_intr_fg_pc = 0;      /* preempted foreground PC */;
uint16_t g_last_intr_fg_dp = 0;      /* preempted foreground DP */;
/* Last LDP (the instruction that set DP) and the predecessor PC. */
uint16_t g_last_ldp_pc  = 0;         /* PC of the instruction that set DP */;
uint16_t g_last_ldp_val = 0;         /* DP value written */;
int      g_last_ldp_kind = 0;        /* 1=LDP #k  2=LDP #k9  3=LD Smem,DP */;
uint16_t g_prev_pc = 0;  /* PC of the instruction executed just before */
uint16_t g_prev_op = 0;  /* its opcode; makes TC reliable for BC after CMPM/BITF */;
uint16_t g_last_st0w_pc  = 0;        /* PC of the last full ST0 write (POPM ST0 / STLM) */;
uint16_t g_last_st0w_val = 0;        /* ST0 value restored */;
uint16_t g_last_st0w_op  = 0;        /* opcode of the instruction writing ST0 */;
uint16_t g_last_st0w_xpc = 0;        /* XPC at the write (PC 0xf48b is XPC-dependent) */;
uint16_t g_last_st0w_prev = 0;       /* predecessor PC of the write */;

/* === ST0 push/pop ring (gate: DISP-ENTRY) ===============================
 * Records PSHM ST0 (push) and POPM/STLM ST0 (write) in a ring, dumped on a bad
 * dispatcher entry (LUT read != 0xff72). It splits a stale DP three ways:
 *   - last PUSH val=0x3124 but POP=0x3125: the stack slot was clobbered
 *     between push and pop (SP / circular-addressing family)
 *   - last PUSH val=0x3125: DP was already wrong at push time (LDP skipped
 *     upstream)
 *   - no PUSH ST0 paired with the POP: SP misalignment, the pop reads another
 *     slot */
St0Ev    g_st0_ring[ST0_RING_N];
unsigned g_st0_ring_idx = 0;
int      g_st0_ring_on  = -1;
void st0_ring_rec(C54xState *s, uint16_t val, char kind)
{
    if (g_st0_ring_on < 0) g_st0_ring_on = calypso_debug_enabled("DISP-ENTRY") ? 1 : 0;
    if (!g_st0_ring_on) return;
    St0Ev *e = &g_st0_ring[g_st0_ring_idx % ST0_RING_N];
    e->pc = s->pc; e->op = prog_fetch(s, s->pc); e->val = val; e->sp = s->sp; e->kind = kind;
    g_st0_ring_idx++;
}
/* LUT slot read at dispatcher entry 0x834d. Captured silently on purpose:
 * DISP-TRACE's logging shifts the timing and hides the bug it looks for. */
uint16_t g_disp_lut_ea  = 0;
uint16_t g_disp_lut_val = 0;
/* Ring of SP events (push/pop), fed from the single chokepoint of the run
 * loop: every SP change records {pc, op, delta}. Dumped by BLACKHOLE-CALA to
 * name the recurring drain source, a push that is never popped. Array writes
 * only, so the cost is negligible. */
struct sp_evt g_spring[64];
uint32_t g_spring_idx = 0;

/* === SHADOW STACK (push/pop pairing) ===
 * Logical mirror of the DSP stack: every PUSH (CALL/CALLD/PSHM/IRQ, SP-)
 * pushes {pc,op,kind}, every POP (RET/RETD/RETE/FRET/RETED/POPM, SP+) pops and
 * checks the pairing. A POP against an empty shadow is a return with no
 * matching call: the over-pop source, reading pristine stack above SP_base.
 * It names that orphan return (PC/op/SP) instead of its victims.
 * Gate: env CALYPSO_ORPHAN, deliberately outside CALYPSO_DEBUG so the master
 * switch stays off (anti-Heisenbug). kind: 'C'=call 'P'=pshm/pshd 'I'=irq
 * 'R'=reti. Array writes only when off. */
struct shadow_ent g_shadow[SHADOW_N];
int  g_shadow_depth = 0;     /* words currently pushed (logical) */;
int  g_shadow_on   = -1;     /* -1 = gate not resolved yet */;
uint64_t g_orphan_hits = 0;  /* orphan POPs detected */;
uint64_t g_mismatch_hits = 0;/* POPs whose kind does not match */;

/* Tracks direct stores into the stack zone [0x1100..0x1140], ABOVE SP_base.
 * A push never writes these slots (the stack grows below 0x1100), so only a
 * direct ST does. Tells a legitimate vector init (slot written by firmware)
 * from a pristine slot (never written = genuine over-pop garbage). */
uint16_t g_stkslot_wpc[STKSLOT_N];
uint16_t g_stkslot_wop[STKSLOT_N];
uint8_t  g_stkslot_written[STKSLOT_N];

void rsbx_intm_check(C54xState *s, uint16_t op)
{
    if (g_rsbx_intm_enabled < 0) {
        const char *e = cdbg_env("RSBX-INTM");
        g_rsbx_intm_enabled = (e && *e == '1') ? 1 : 0;
        if (g_rsbx_intm_enabled) {
            fprintf(stderr, "[c54x] RSBX-INTM-TRACE enabled (op=0xF6BB)\n");
        }
    }
    if (g_rsbx_intm_enabled <= 0) return;
    if (op == 0xF6BB) {
        g_rsbx_intm_hits++;
        if (g_rsbx_intm_hits <= 20 || (g_rsbx_intm_hits % 1000) == 0) {
            fprintf(stderr,
                "[c54x] RSBX-INTM #%llu @insn=%u PC=0x%04x  ST1 INTM 0x%04x → "
                "(cleared) — IRQ enable path atteint !\n",
                (unsigned long long)g_rsbx_intm_hits, s->insn_count, s->pc,
                s->st1);
        }
    }
}

/* === SP absolute-write tracer ===
 * Logs every SP write that *teleports* SP to an arbitrary value - STL/STM/STLM
 * absolute, FRAME #imm, MVMM register transfer - as opposed to PUSH/POP/
 * CALL/RET, which move SP by one. A site that teleports SP=0x3fbe is the exact
 * corrupter of the bootstub entry observed at insn=3995013.
 *
 * Hooked at the three sites that can do it:
 *   site 0  data_write_locked case MMR_SP (STL/STM/STLM to MMR_SP)
 *   site 1  F7Dx case 0xD, LD #k8u, SP
 *   site 2  MVMM register transfer with dst==8 (SP in the 3-bit MMR encoding)
 *
 * Gate: CALYPSO_DEBUG token SP-ABS, zero cost when off. First N writes are
 * logged verbatim, the rest go to a per-PC histogram (cap SP_ABS_HIST_MAX). */
SpAbsEntry g_sp_abs_hist[SP_ABS_HIST_MAX];
unsigned   g_sp_abs_used     = 0;
unsigned   g_sp_abs_total    = 0;
int        g_sp_abs_enabled  = -1;
unsigned   g_sp_abs_log_cap  = 50;

void sp_abs_track(C54xState *s, uint16_t new_val, uint8_t site)
{
    if (g_sp_abs_enabled < 0) {
        const char *e = cdbg_env("SP-ABS");
        g_sp_abs_enabled = (e && *e == '1') ? 1 : 0;
        if (g_sp_abs_enabled) {
            fprintf(stderr, "[c54x] SP-ABS-TRACE enabled, log_cap=%u hist_max=%u\n",
                    g_sp_abs_log_cap, SP_ABS_HIST_MAX);
        }
    }
    if (g_sp_abs_enabled <= 0) return;
    g_sp_abs_total++;
    /* Per-PC histogram */
    unsigned i;
    for (i = 0; i < g_sp_abs_used; i++) {
        if (g_sp_abs_hist[i].pc == s->pc && g_sp_abs_hist[i].site == site) {
            g_sp_abs_hist[i].count++;
            g_sp_abs_hist[i].value_last = new_val;
            break;
        }
    }
    if (i == g_sp_abs_used && g_sp_abs_used < SP_ABS_HIST_MAX) {
        g_sp_abs_hist[i].pc         = s->pc;
        g_sp_abs_hist[i].value_last = new_val;
        g_sp_abs_hist[i].count      = 1;
        g_sp_abs_hist[i].site       = site;
        g_sp_abs_used++;
    }
    /* Verbatim log first N */
    if (g_sp_abs_total <= g_sp_abs_log_cap) {
        const char *site_name = site == 0 ? "MMR_SP-W" :
                                site == 1 ? "LD-#k8-SP" : "MVMM-SP";
        int32_t delta = (int32_t)new_val - (int32_t)s->sp;
        fprintf(stderr,
            "[c54x] SP-ABS #%u %s @insn=%u PC=0x%04x SP %04x → %04x (Δ=%+d) "
            "A=%010llx AR4=%04x\n",
            g_sp_abs_total, site_name, s->insn_count, s->pc,
            s->sp, new_val, delta,
            (unsigned long long)(s->a & 0xFFFFFFFFFFULL), s->ar[4]);
    }
    /* Flag an SP landing in a suspect zone: 0x3fb0..0x3fbf is the BSP read
     * region, 0x2b80..0x2c00 the I/Q buffer tail. */
    if ((new_val >= 0x3fb0 && new_val <= 0x3fbf) ||
        (new_val >= 0x2b80 && new_val <= 0x2c00)) {
        fprintf(stderr,
            "[c54x] SP-ABS SUSPECT! @insn=%u PC=0x%04x SP←0x%04x (corrupter ?)\n",
            s->insn_count, s->pc, new_val);
    }
}

/* === MVPD overlay occupancy trace ===
 * Buckets writes to data[0x0080..0x27FF] in 0x80-word buckets and dumps the
 * occupancy at the end of the boot phase. It shows which sub-ranges MVPD
 * loads at boot (= overlay code), which decides whether the BSP buffer can
 * live in the read region [0x0000..0x03A3] without overwriting code that is
 * executing.
 * Gates:
 *   CALYPSO_DEBUG token MVPD   enables the trace (default off)
 *   CALYPSO_MVPD_BOOT_LIMIT=N  insn cap for the dump (default 500000) */
uint32_t g_mvpd_buckets[MVPD_BUCKETS_N];  /* 80 buckets */;
int      g_mvpd_trace_enabled = -1;
unsigned g_mvpd_boot_limit    = 0;
int      g_mvpd_dumped        = 0;

void mvpd_trace_init_lazy(void)
{
    if (g_mvpd_trace_enabled >= 0) return;
    const char *e = cdbg_env("MVPD");
    g_mvpd_trace_enabled = (e && *e == '1') ? 1 : 0;
    const char *l = getenv("CALYPSO_MVPD_BOOT_LIMIT");
    g_mvpd_boot_limit = (l && *l) ? (unsigned)strtoul(l, NULL, 0) : 500000u;
    if (g_mvpd_trace_enabled) {
        fprintf(stderr,
            "[c54x] MVPD-TRACE enabled, range=[0x%04x..0x%04x] bucket_sz=%u "
            "buckets=%u boot_limit=%u\n",
            MVPD_RANGE_LO, MVPD_RANGE_HI, MVPD_BUCKET_SZ, MVPD_BUCKETS_N,
            g_mvpd_boot_limit);
    }
}

void mvpd_trace_record(uint16_t addr)
{
    if (g_mvpd_trace_enabled <= 0) return;
    if (addr < MVPD_RANGE_LO || addr >= MVPD_RANGE_HI) return;
    unsigned b = (addr - MVPD_RANGE_LO) >> MVPD_BUCKET_BITS;
    if (b < MVPD_BUCKETS_N) g_mvpd_buckets[b]++;
}

void mvpd_trace_dump_if_due(unsigned insn)
{
    if (g_mvpd_trace_enabled <= 0) return;
    if (g_mvpd_dumped) return;
    if (insn < g_mvpd_boot_limit) return;
    g_mvpd_dumped = 1;
    fprintf(stderr, "[c54x] MVPD-OCCUPANCY DUMP @insn=%u (boot phase end)\n", insn);
    for (unsigned b = 0; b < MVPD_BUCKETS_N; b++) {
        if (g_mvpd_buckets[b] == 0) continue;
        uint16_t lo = MVPD_RANGE_LO + b * MVPD_BUCKET_SZ;
        uint16_t hi = lo + MVPD_BUCKET_SZ - 1;
        fprintf(stderr,
            "[c54x] MVPD-BUCKET [0x%04x..0x%04x] writes=%u\n",
            lo, hi, g_mvpd_buckets[b]);
    }
    /* Buffer placement verdict: few or no writes in [0x0080..0x03A3]
     * (correlator read region, buckets 0..6) means the range is safe for BSP
     * DMA; otherwise the buffer needs another home. */
    unsigned bucket_0_to_6_total = 0;
    for (unsigned b = 0; b < 7 && b < MVPD_BUCKETS_N; b++)
        bucket_0_to_6_total += g_mvpd_buckets[b];
    fprintf(stderr,
        "[c54x] MVPD-VERDICT correlator_read_region [0x0080..0x03A3] "
        "writes=%u → %s\n",
        bucket_0_to_6_total,
        bucket_0_to_6_total == 0
            ? "EMPTY (safe pour BSP buffer placement ici)"
            : bucket_0_to_6_total < 100
            ? "lightly used (probably safe, audit specifics)"
            : "HEAVILY USED (code overlay, NE PAS placer BSP buffer ici)");
}

/* === Correlator trace ===
 * Captures AR0..AR7, SP and ST0/ST1 on entry into the FB-det correlator, and
 * the data reads performed while inside it, to check empirically that the
 * firmware reads its I/Q input from [0x0000..0x03A3].
 * Gate: CALYPSO_DEBUG token CORRELATOR, zero cost when off.
 *
 * Correlator range: [0x8d00..0x9000) (FB-det handler in PROM0). The upper
 * bound was raised from 0x8F80 to 0x9000 because d_fb_det WATCH-READ showed
 * reads at PC=0x8FAC and 0x8FB5 falling outside the old filter, which made
 * CORR-ENTRY report 0 while the firmware was in fact working in the zone.
 *
 * Reads are kept as a capped list of unique addresses so long runs do not
 * flood the log. */
CorrReadEntry g_corr_read_hist[CORR_READ_HIST_MAX];
unsigned    g_corr_read_used    = 0;
int         g_corr_trace_enabled = -1; /* -1 uninit, 0 off, 1 on */;
unsigned    g_corr_entry_count  = 0;
unsigned    g_corr_entry_log_cap = 100000;  /* uncap : voir le par-frame post-+3s */;

/* Set by calypso_trx.c when the ARM writes d_task_md=5 (FB command). The
 * D_TASK_MD-RD probe timestamps the DSP reads against this write, comparing
 * the ARM write EA with the DSP read EA and their order. */
uint32_t g_arm_taskmd5_insn = 0;
uint16_t g_arm_taskmd5_ea   = 0;
uint64_t    g_corr_read_total   = 0;
uint16_t    g_corr_last_pc      = 0xFFFF; /* track PC transitions */;

void corr_trace_init_lazy(void)
{
    if (g_corr_trace_enabled >= 0) return;
    const char *e = cdbg_env("CORRELATOR");
    g_corr_trace_enabled = (e && *e == '1') ? 1 : 0;
    if (g_corr_trace_enabled) {
        fprintf(stderr, "[c54x] CORRELATOR-TRACE enabled, range=[0x%04x..0x%04x) "
                        "hist_max=%u entry_log_cap=%u\n",
                CORR_PC_LO, CORR_PC_HI,
                CORR_READ_HIST_MAX, g_corr_entry_log_cap);
    }
}

/* CORR-ENTRY tracker: called at the top of the loop for every dispatched
 * instruction. Detects a PC transition from outside into the FB-det range and
 * logs the first N entries with the AR/SP/ST context. */
void corr_entry_track(uint16_t pc, void *s_void)
{
    if (g_corr_trace_enabled <= 0) return;
    bool was_in  = (g_corr_last_pc >= CORR_PC_LO && g_corr_last_pc < CORR_PC_HI);
    bool is_in   = (pc >= CORR_PC_LO && pc < CORR_PC_HI);
    g_corr_last_pc = pc;
    if (!was_in && is_in) {
        g_corr_entry_count++;
        if (g_corr_entry_count <= g_corr_entry_log_cap
            || (g_corr_entry_count % 100) == 0) {
            C54xState *s = (C54xState *)s_void;
            fprintf(stderr,
                "[c54x] CORR-ENTRY #%u @PC=0x%04x from=0x%04x SP=0x%04x "
                "ST0=0x%04x ST1=0x%04x AR=[%04x %04x %04x %04x %04x %04x %04x %04x] "
                "A=%010llx B=%010llx T=%04x\n",
                g_corr_entry_count, pc, g_corr_last_pc, s->sp,
                s->st0, s->st1,
                s->ar[0], s->ar[1], s->ar[2], s->ar[3],
                s->ar[4], s->ar[5], s->ar[6], s->ar[7],
                (unsigned long long)(s->a & 0xFFFFFFFFFFULL),
                (unsigned long long)(s->b & 0xFFFFFFFFFFULL),
                s->t);
        }
    }
}

/* === FBDB/FBF3 + 0x3DC0 probes ========================================
 *
 * Read-only probes around the fc50-fc6f kernel, three observations:
 *   A) B at PC=0xfbd9, before `SUB #8, B, A`: a wrong B upstream yields a
 *      wrong A downstream whatever the F2xx handler does.
 *   B) A at PC=0xfbdb (just after the SUB) and at PC=0xfbf3 (just before
 *      STLM A,AR4): A_low=0 there gives AR4=0, hence AR5=1=MMR_IFR.
 *   C) Whether bit 4 of the flag at 0x3DC0, tested by the BITF at fc63, is
 *      ever set. If no DSP routine sets it, the BCD NTC at fc66 always
 *      branches and fc50-fc6f loops forever without entering its body.
 *
 * Gate: CALYPSO_DEBUG token FBDB. Cost when off: one compare and one branch
 * per opcode. */
int g_fbdb_probe_enabled = -1;
unsigned g_fbdb_probe_log_cap = 100;
unsigned g_fbdb_probe_count_b = 0;
unsigned g_fbdb_probe_count_a = 0;
unsigned g_fbdb_probe_count_fbf3 = 0;
unsigned g_addr3dc0_wr_count = 0;
unsigned g_addr3dc0_rd_count = 0;

static void fbdb_probe_init_lazy(void)
{
    if (g_fbdb_probe_enabled >= 0) return;
    const char *e = cdbg_env("FBDB");
    g_fbdb_probe_enabled = (e && *e == '1') ? 1 : 0;
    if (g_fbdb_probe_enabled) {
        fprintf(stderr,
            "[c54x] FBDB-PROBE enabled : track B@0xfbd9 + A@0xfbdb + A@0xfbf3 "
            "+ ALL r/w to 0x3DC0 (= SARAM flag polled by fc63 BITF)\n");
    }
}

/* Hook called from c54x_run top-of-loop, before c54x_exec_one. */
void fbdb_probe_check_pc(uint16_t pc, void *s_void)
{
    if (g_fbdb_probe_enabled < 0) fbdb_probe_init_lazy();
    if (g_fbdb_probe_enabled <= 0) return;
    C54xState *s = (C54xState *)s_void;
    if (pc == 0xfbd9 && g_fbdb_probe_count_b < g_fbdb_probe_log_cap) {
        g_fbdb_probe_count_b++;
        int64_t b = s->b & 0xFFFFFFFFFFLL;
        fprintf(stderr,
            "[c54x] FBDB-PROBE B@fbd9 #%u: B=0x%010llx (low=0x%04x sign=%d) "
            "A=0x%010llx AR2=0x%04x AR3=0x%04x AR4=0x%04x insn=%u\n",
            g_fbdb_probe_count_b,
            (unsigned long long)b, (unsigned)(b & 0xFFFF),
            (b & 0x8000000000LL) ? -1 : 1,
            (unsigned long long)(s->a & 0xFFFFFFFFFFLL),
            s->ar[2], s->ar[3], s->ar[4], s->insn_count);
    }
    else if (pc == 0xfbdb && g_fbdb_probe_count_a < g_fbdb_probe_log_cap) {
        g_fbdb_probe_count_a++;
        int64_t a = s->a & 0xFFFFFFFFFFLL;
        fprintf(stderr,
            "[c54x] FBDB-PROBE A@fbdb #%u (= after SUB #8,B,A): A=0x%010llx "
            "(low=0x%04x), TC=%d insn=%u\n",
            g_fbdb_probe_count_a, (unsigned long long)a,
            (unsigned)(a & 0xFFFF),
            !!(s->st0 & (1 << 13)), s->insn_count);  /* TC = ST0 bit 13 per SPRU131G */
    }
    else if (pc == 0xfbf3 && g_fbdb_probe_count_fbf3 < g_fbdb_probe_log_cap) {
        g_fbdb_probe_count_fbf3++;
        int64_t a = s->a & 0xFFFFFFFFFFLL;
        fprintf(stderr,
            "[c54x] FBDB-PROBE A@fbf3 #%u (= just before STLM A,AR4): "
            "A=0x%010llx (low=0x%04x) — if low=0 → AR4=0 → AR5=1=MMR_IFR corruption "
            "insn=%u\n",
            g_fbdb_probe_count_fbf3, (unsigned long long)a,
            (unsigned)(a & 0xFFFF), s->insn_count);
    }
}

/* === STUCK-STATE PC+XPC histogram =====================================
 *
 * While the DSP is stuck (INTM=1 AND BRINT0 pending in IFR), record PC+XPC.
 * The plain PC histogram cannot tell XPC pages apart - a bare "fc50" may be
 * the page 0x1F mirror, or page 0x28, 0x38... - so the XPC qualification is
 * what names the real blocking loop.
 *
 * Stuck entry and exit are logged to delimit the window, and the top 20
 * PC+XPC pairs are dumped periodically while it lasts.
 *
 * Gate: CALYPSO_DEBUG token STUCK. Cost when off: one bit test and one
 * branch. */
StuckHistEntry g_stuck_hist[STUCK_HIST_SIZE];
unsigned g_stuck_hist_used = 0;
int g_stuck_probe_enabled = -1;
int g_stuck_active = 0;
uint32_t g_stuck_duration = 0;
uint64_t g_stuck_start_insn = 0;
unsigned g_stuck_dump_count = 0;

static void stuck_probe_init_lazy(void)
{
    if (g_stuck_probe_enabled >= 0) return;
    const char *e = cdbg_env("STUCK");
    g_stuck_probe_enabled = (e && *e == '1') ? 1 : 0;
    if (g_stuck_probe_enabled) {
        fprintf(stderr,
            "[c54x] STUCK-PROBE enabled : capture PC+XPC histogramme quand "
            "INTM=1 + IFR bit5 (BRINT0) pending\n");
    }
}

static void stuck_probe_record(uint16_t pc, uint8_t xpc)
{
    /* Linear scan small hist (cap 64). Insert or increment. */
    for (unsigned i = 0; i < g_stuck_hist_used; i++) {
        if (g_stuck_hist[i].pc == pc && g_stuck_hist[i].xpc == xpc) {
            g_stuck_hist[i].count++;
            return;
        }
    }
    if (g_stuck_hist_used < STUCK_HIST_SIZE) {
        g_stuck_hist[g_stuck_hist_used].pc = pc;
        g_stuck_hist[g_stuck_hist_used].xpc = xpc;
        g_stuck_hist[g_stuck_hist_used].count = 1;
        g_stuck_hist_used++;
    }
    /* If hist full, silently drop new PCs — top hot ones already captured. */
}

static void stuck_probe_dump(uint64_t cur_insn, const char *trig)
{
    /* Bubble sort by count desc (n<=64). */
    for (unsigned k = 0; k < g_stuck_hist_used; k++) {
        unsigned best = k;
        for (unsigned i = k + 1; i < g_stuck_hist_used; i++) {
            if (g_stuck_hist[i].count > g_stuck_hist[best].count) best = i;
        }
        if (best != k) {
            StuckHistEntry tmp = g_stuck_hist[k];
            g_stuck_hist[k] = g_stuck_hist[best];
            g_stuck_hist[best] = tmp;
        }
    }
    if (calypso_debug_enabled("STUCK-HIST")) fprintf(stderr,
        "[c54x] STUCK-HIST [%s] duration=%u insn since insn=%llu (now=%llu) top:\n",
        trig, g_stuck_duration,
        (unsigned long long)g_stuck_start_insn,
        (unsigned long long)cur_insn);
    unsigned n_show = g_stuck_hist_used > 20 ? 20 : g_stuck_hist_used;
    for (unsigned i = 0; i < n_show; i++) {
        fprintf(stderr,
            "[c54x]   #%2u PC=0x%04x XPC=%u  count=%u\n",
            i + 1, g_stuck_hist[i].pc, g_stuck_hist[i].xpc,
            g_stuck_hist[i].count);
    }
}

/* === FORCE-INTM-ONESHOT ================================================
 *
 * Arbitration probe: when INTM=1 AND BRINT0 (IFR bit 5) is pending, clear
 * INTM ONCE to let the dispatch happen, then watch the existing tracers
 * (CORR-ENTRY, a_sync_demod writes, RETE log, INTM-TRANS). Real snr/toa after
 * the dispatch means INTM was the only blocker; garbage or nothing means the
 * downstream path is broken too, or the ISR state is corrupt.
 *
 * This is a probe, NOT a fix: the one-shot observes without masking steady
 * behaviour. If INTM turns out to be the only blocker, the fix belongs in the
 * ISR (why no RETE), not in a systematic INTM clear.
 *
 * Env: CALYPSO_FORCE_INTM_ONESHOT=1. */
int g_force_intm_oneshot_enabled = -1;
int g_force_intm_oneshot_done = 0;
uint64_t g_force_intm_oneshot_insn = 0;
/* CALYPSO_FORCE_INTM_AT_PC=0xfc6f restricts the force to that PC, so it can
 * fire at a safe point (the RET of the fc50 compute kernel, say) instead of
 * mid-compute. The 0xFFFF sentinel means no PC restriction: fire at the first
 * opportunity. */
uint16_t g_force_intm_at_pc = 0xFFFF;

/* @BEQUILLE - FORCE_INTM_ONESHOT (+ FORCE_INTM_AT_PC)  (CALYPSO_FORCE_INTM_ONESHOT=1,
 *              CALYPSO_FORCE_INTM_AT_PC=0xXXXX ; default OFF ; calypso_wire.env:=1)
 *   masks   : the RSBX INTM at 0xa51b that the firmware never executes. With the
 *             PC gate the block ALSO raises the interrupt (s->ifr |= s->imr &
 *             0x3000): it manufactures the event, it does not merely open the
 *             window.
 *   remove  : once INTM reaches 0 through the ROM path (see the INTM-TRANS trace).
 *   NB      : run.sh already reports it as "NON-nominal".
 */
void force_intm_oneshot_check(C54xState *s)
{
    if (g_force_intm_oneshot_enabled < 0) {
        const char *e = getenv("CALYPSO_FORCE_INTM_ONESHOT");
        /* Strict gate: ON only for =1; =0 or unset means OFF. The one-shot
         * masks the vec28 livelock by clearing INTM once, which is useful
         * until the frame-IT over-fire is fixed at its BSP root. */
        g_force_intm_oneshot_enabled = (e && *e == '1') ? 1 : 0;
        /* Optional PC gate: with CALYPSO_FORCE_INTM_AT_PC=0xXXXX set, fire
         * only when the PC matches, so the force lands at a safe point (the
         * RET at fc6f, the idle dispatcher) instead of mid-compute at fc57.
         * That separates state corruption from a broken downstream path. */
        const char *pc_e = getenv("CALYPSO_FORCE_INTM_AT_PC");
        if (pc_e && *pc_e) {
            unsigned long pc_val = strtoul(pc_e, NULL, 0);
            if (pc_val <= 0xFFFF) {
                g_force_intm_at_pc = (uint16_t)pc_val;
            }
        }
        if (g_force_intm_oneshot_enabled) {
            if (g_force_intm_at_pc != 0xFFFF) {
                fprintf(stderr,
                    "[c54x] FORCE-INTM-ONESHOT enabled : will clear INTM ONCE "
                    "when INTM=1 + BRINT0 pending + PC=0x%04x (= safe-PC gate, "
                    "départage state-corruption vs aval-cassé)\n",
                    g_force_intm_at_pc);
            } else {
                fprintf(stderr,
                    "[c54x] FORCE-INTM-ONESHOT enabled : will clear INTM ONCE "
                    "when INTM=1 + BRINT0 pending (= sonde aval-sain, no PC gate)\n");
            }
        }
    }
    if (g_force_intm_oneshot_enabled <= 0) return;
    if (g_force_intm_oneshot_done) return;
    int intm_set = !!(s->st1 & ST1_INTM);
    if (!intm_set) return;
    /* [2026-07-22] Raise IFR before clearing INTM. At go-live (0xa4e1) IMR is
     * 0x3000 (frame bit unmasked) but IFR is 0, so no interrupt is pending and
     * clearing INTM alone does nothing: the frame interrupt is never latched
     * (the c54x interrupt model is incomplete). At the target PC, force the
     * frame interrupt pending (IFR |= unmasked bits) and only then clear INTM.
     * Without the PC gate, keep the old behaviour: fire on an already pending
     * interrupt. */
    if (g_force_intm_at_pc != 0xFFFF) {
        if (s->pc != g_force_intm_at_pc) return;
        uint16_t unmasked = s->imr & 0x3000;   /* vec28/frame (bit12) + bit13 */
        if (!unmasked) return;                 /* nothing unmasked to force */
        s->ifr |= unmasked;                    /* raise the frame interrupt */
    } else {
        int it_pending = !!(s->ifr & s->imr);
        if (!it_pending) return;
        if (s->insn_count < 1000000) return;
    }
    /* FIRE one-shot : clear INTM, log context. */
    g_force_intm_oneshot_done = 1;
    g_force_intm_oneshot_insn = s->insn_count;
    fprintf(stderr,
        "[c54x] FORCE-INTM-ONESHOT FIRED @insn=%llu PC=0x%04x XPC=%u SP=0x%04x "
        "ST1=0x%04x IMR=0x%04x IFR=0x%04x%s — clearing INTM to allow dispatch\n",
        (unsigned long long)s->insn_count, s->pc, s->xpc & 0xFF, s->sp,
        s->st1, s->imr, s->ifr,
        (g_force_intm_at_pc != 0xFFFF) ? " (safe-PC gate)" : "");
    s->st1 &= ~ST1_INTM;  /* clear INTM bit 11 */
    fprintf(stderr,
        "[c54x] FORCE-INTM-ONESHOT post-clear : ST1=0x%04x — watch next IRQ "
        "dispatch + CORR-ENTRY + a_sync_demod writes\n", s->st1);
}

/* === INT3 cycle tracer + control-flow signature ========================
 *
 * Answers why an INT3 ISR does not reach its RETE.
 *
 * Per INT3 cycle:
 *   START   INT3 dispatched (vec=19): reset the trace, log cycle_id and entry PC
 *   DURING  every conditional branch executed: (PC, op, target, taken)
 *   END ok  RETE fires: dump the trace tagged GOOD plus the insn count
 *   END bad a new INT3 dispatch before the RETE: dump the previous trace
 *           tagged ORPHAN-NEXT-INT3
 *
 * Diffing a good cycle against an orphan one offline gives the first branch
 * that diverges; the state tested at that branch is the cause.
 *
 * Capped at INT3_BRANCH_TRACE_MAX branches per cycle, overflow is tagged.
 * Gate: CALYPSO_DEBUG token INT3-CYCLE. */
Int3BranchEvent g_int3_trace[INT3_BRANCH_TRACE_MAX];
unsigned g_int3_trace_count = 0;
int g_int3_trace_overflow = 0;
int g_int3_cycle_active = 0;
uint64_t g_int3_cycle_id = 0;
uint16_t g_int3_cycle_entry_pc = 0;
uint64_t g_int3_cycle_entry_insn = 0;
int g_int3_trace_enabled = -1;

static void int3_trace_init_lazy(void)
{
    if (g_int3_trace_enabled >= 0) return;
    const char *e = cdbg_env("INT3-CYCLE");
    g_int3_trace_enabled = (e && *e == '1') ? 1 : 0;
    if (g_int3_trace_enabled) {
        fprintf(stderr,
            "[c54x] INT3-CYCLE-TRACE enabled : par cycle vec=19, log toutes "
            "branches conditionnelles + RETE/orphan tag. Cap=%u branches/cycle.\n",
            INT3_BRANCH_TRACE_MAX);
    }
}

/* Detect conditional branch / call / return family. Returns 1 if op
 * is in a tracked branch family, else 0. */
static int is_int3_traced_branch(uint16_t op)
{
    uint16_t hi = op & 0xFF00;
    if (hi == 0x6C00) return 1;  /* BANZ pmad,Sind */
    if (hi == 0x6E00) return 1;  /* BANZD pmad,Sind */
    if (hi == 0xF800) return 1;  /* BC pmad,cond */
    if (hi == 0xF900) return 1;  /* CC pmad,cond */
    if (hi == 0xFA00) return 1;  /* BCD pmad,cond */
    if (hi == 0xFB00) return 1;  /* CCD pmad,cond */
    if (hi == 0xFC00) {
        /* FC00 unconditional = RET ; FCxx where xx is cond = RC */
        if (op != 0xFC00) return 1;
        return 0;
    }
    if (hi == 0xFE00) {
        if (op != 0xFE00) return 1; /* RCD cond */
        return 0;
    }
    return 0;
}

/* Called from c54x_interrupt_ex when vec=19 (INT3 FRAME) dispatched. */
void int3_cycle_start(C54xState *s, uint16_t target_pc)
{
    if (g_int3_trace_enabled < 0) int3_trace_init_lazy();
    if (g_int3_trace_enabled <= 0) return;
    /* If previous cycle still active = orphan (= didn't RETE before re-entry) */
    if (g_int3_cycle_active) {
        fprintf(stderr,
            "[c54x] INT3-CYCLE #%llu ORPHAN-NEXT-INT3 — previous cycle didn't "
            "RETE, new entry @insn=%llu PC=0x%04x. Trace below (%u branches%s) :\n",
            (unsigned long long)g_int3_cycle_id,
            (unsigned long long)s->insn_count, target_pc,
            g_int3_trace_count,
            g_int3_trace_overflow ? "+ OVERFLOW" : "");
        for (unsigned i = 0; i < g_int3_trace_count; i++) {
            Int3BranchEvent *e = &g_int3_trace[i];
            /* 2-word branches (no lk_used here for simplicity) : BC/CC/BCD/CCD
             * (0xF8-0xFB) and BANZ/BANZD (0x6C/0x6E). 1-word : RET/RC/RCD. */
            uint16_t hi = e->op & 0xFF00;
            bool two_word = (hi >= 0xF800 && hi <= 0xFB00)
                            || hi == 0x6C00 || hi == 0x6E00;
            uint16_t fallthrough = e->pc + (two_word ? 2 : 1);
            fprintf(stderr,
                "[c54x]   #%3u Δ%u PC=0x%04x op=0x%04x → next=0x%04x %s ×%u\n",
                i + 1, e->insn_offset, e->pc, e->op, e->next_pc,
                (e->next_pc == fallthrough) ? "(NOT_TAKEN)" : "(TAKEN)",
                e->repeat);
        }
    }
    g_int3_cycle_id++;
    g_int3_cycle_active = 1;
    g_int3_cycle_entry_pc = target_pc;
    g_int3_cycle_entry_insn = s->insn_count;
    g_int3_trace_count = 0;
    g_int3_trace_overflow = 0;
    if (calypso_debug_enabled("INT3-CYCLE")) fprintf(stderr,
        "[c54x] INT3-CYCLE #%llu START @insn=%llu PC→0x%04x SP=0x%04x "
        "PMST=0x%04x IFR=0x%04x\n",
        (unsigned long long)g_int3_cycle_id,
        (unsigned long long)s->insn_count, target_pc, s->sp,
        s->pmst, s->ifr);
}

/* Called from c54x_run after c54x_exec_one. exec_pc/exec_op are the
 * instruction that just executed; s->pc is the resulting PC. */
void int3_cycle_track_branch(C54xState *s, uint16_t exec_pc,
                                    uint16_t exec_op, int consumed)
{
    if (g_int3_trace_enabled <= 0) return;
    if (!g_int3_cycle_active) return;
    if (!is_int3_traced_branch(exec_op)) return;
    /* Compute next_pc correctly across all branch-handler patterns :
     * 1. Non-delayed branch TAKEN  → handler set s->pc=target, returned consumed=0
     *    → s->pc already = target.
     * 2. Delayed branch TAKEN      → handler armed delay_slots=2 + delayed_pc,
     *    returned consumed>0; main loop hasn't run +=consumed yet
     *    → eventual target = s->delayed_pc.
     * 3. Branch FALL-THROUGH (any) → handler returned consumed>0, s->pc unchanged,
     *    delay_slots not set; main loop will += consumed → next insn
     *    → next = exec_pc + consumed. */
    uint16_t actual_next;
    if (consumed == 0) {
        actual_next = s->pc;
    } else if (s->delay_slots == 2) {
        actual_next = s->delayed_pc;
    } else {
        actual_next = (uint16_t)(exec_pc + consumed);
    }

    /* Dedup-pattern : look up to 4 slots back. Catches consecutive
     * identical (distance 1, AAA), strict alternation (distance 2,
     * ABAB), and short cycles up to length 4 (ABCDABCD). Each iteration
     * of the repeating pattern bumps the matched slot's repeat — total
     * iterations = max(repeat) across slots forming the cycle. */
    for (unsigned back = 1; back <= 4 && back <= g_int3_trace_count; back++) {
        Int3BranchEvent *cand = &g_int3_trace[g_int3_trace_count - back];
        if (cand->pc == exec_pc && cand->op == exec_op && cand->next_pc == actual_next) {
            cand->repeat++;
            return;
        }
    }
    if (g_int3_trace_count >= INT3_BRANCH_TRACE_MAX) {
        g_int3_trace_overflow = 1;
        return;
    }
    Int3BranchEvent *e = &g_int3_trace[g_int3_trace_count++];
    e->pc = exec_pc;
    e->op = exec_op;
    e->next_pc = actual_next;
    e->insn_offset = (uint32_t)(s->insn_count - g_int3_cycle_entry_insn);
    e->repeat = 1;
}

/* Called from RETE handler (L3300 area) BEFORE INTM is cleared. */
void int3_cycle_end_good(C54xState *s, uint16_t return_addr)
{
    if (g_int3_trace_enabled <= 0) return;
    if (!g_int3_cycle_active) return;
    uint64_t duration = s->insn_count - g_int3_cycle_entry_insn;
    fprintf(stderr,
        "[c54x] INT3-CYCLE #%llu RETE-GOOD @insn=%llu duration=%llu PC→0x%04x "
        "branches=%u%s\n",
        (unsigned long long)g_int3_cycle_id,
        (unsigned long long)s->insn_count, (unsigned long long)duration,
        return_addr, g_int3_trace_count,
        g_int3_trace_overflow ? "+ OVERFLOW" : "");
    for (unsigned i = 0; i < g_int3_trace_count; i++) {
        Int3BranchEvent *e = &g_int3_trace[i];
        uint16_t hi = e->op & 0xFF00;
        bool two_word = (hi >= 0xF800 && hi <= 0xFB00)
                        || hi == 0x6C00 || hi == 0x6E00;
        uint16_t fallthrough = e->pc + (two_word ? 2 : 1);
        fprintf(stderr,
            "[c54x]   #%3u Δ%u PC=0x%04x op=0x%04x → next=0x%04x %s ×%u\n",
            i + 1, e->insn_offset, e->pc, e->op, e->next_pc,
            (e->next_pc == fallthrough) ? "(NOT_TAKEN)" : "(TAKEN)",
            e->repeat);
    }
    g_int3_cycle_active = 0;
}

/* Called from c54x_run top-of-loop. */
void stuck_probe_check(C54xState *s)
{
    if (g_stuck_probe_enabled < 0) stuck_probe_init_lazy();
    if (g_stuck_probe_enabled <= 0) return;
    int intm_set = !!(s->st1 & ST1_INTM);
    int brint0_pending = !!(s->ifr & (1 << 5));
    int now_stuck = (intm_set && brint0_pending);
    if (now_stuck && !g_stuck_active) {
        g_stuck_active = 1;
        g_stuck_start_insn = s->insn_count;
        g_stuck_duration = 0;
        g_stuck_hist_used = 0;  /* fresh hist per stuck window */
        fprintf(stderr,
            "[c54x] STUCK-ENTER insn=%llu PC=0x%04x XPC=%u IFR=0x%04x IMR=0x%04x\n",
            (unsigned long long)s->insn_count, s->pc, s->xpc & 0xFF,
            s->ifr, s->imr);
    }
    if (now_stuck) {
        g_stuck_duration++;
        /* Sample every 100 insns to bound hist diversity */
        if ((g_stuck_duration % 100) == 0) {
            stuck_probe_record(s->pc, s->xpc & 0xFF);
        }
        /* Dump periodically while stuck */
        if ((g_stuck_duration % 5000000) == 0 && g_stuck_dump_count < 5) {
            g_stuck_dump_count++;
            stuck_probe_dump(s->insn_count, "periodic-5M");
        }
    } else if (g_stuck_active) {
        g_stuck_active = 0;
        fprintf(stderr,
            "[c54x] STUCK-EXIT insn=%llu duration=%u PC=0x%04x XPC=%u IFR=0x%04x\n",
            (unsigned long long)s->insn_count, g_stuck_duration,
            s->pc, s->xpc & 0xFF, s->ifr);
        if (g_stuck_duration >= 10000) {  /* only dump if long-ish stuck */
            stuck_probe_dump(s->insn_count, "on-exit");
        }
    }
}

/* Hook called from data_write_locked when an absolute write hits 0x3DC0. */
void fbdb_probe_write_3dc0(uint16_t addr, uint16_t old_val,
                                  uint16_t new_val, uint16_t pc, unsigned insn)
{
    if (g_fbdb_probe_enabled <= 0) return;
    g_addr3dc0_wr_count++;
    if (g_addr3dc0_wr_count <= 50) {
        uint16_t set_mask = new_val & ~old_val;  /* bits set by this write */
        fprintf(stderr,
            "[c54x] FBDB-PROBE WR 0x%04x : 0x%04x → 0x%04x (set=0x%04x) "
            "PC=0x%04x insn=%u %s\n",
            addr, old_val, new_val, set_mask, pc, insn,
            (set_mask & 0x0010) ? "*** BIT 4 SET ***" : "");
    }
}

void fbdb_probe_read_3dc0(uint16_t addr, uint16_t val,
                                 uint16_t pc, unsigned insn)
{
    if (g_fbdb_probe_enabled <= 0) return;
    g_addr3dc0_rd_count++;
    if (g_addr3dc0_rd_count <= 30
        || (g_addr3dc0_rd_count % 10000) == 0) {
        fprintf(stderr,
            "[c54x] FBDB-PROBE RD 0x%04x = 0x%04x (bit4=%d) PC=0x%04x insn=%u\n",
            addr, val, !!(val & 0x0010), pc, insn);
    }
}

void corr_read_record(uint16_t addr)
{
    if (g_corr_trace_enabled <= 0) return;
    g_corr_read_total++;
    unsigned i;
    for (i = 0; i < g_corr_read_used; i++) {
        if (g_corr_read_hist[i].addr == addr) {
            g_corr_read_hist[i].count++;
            return;
        }
    }
    if (g_corr_read_used < CORR_READ_HIST_MAX) {
        g_corr_read_hist[g_corr_read_used].addr  = addr;
        g_corr_read_hist[g_corr_read_used].count = 1;
        g_corr_read_used++;
    }
}

void corr_read_dump(const char *trig)
{
    if (g_corr_trace_enabled <= 0) return;
    fprintf(stderr, "[c54x] CORR-READ DUMP[%s] total=%llu uniq=%u\n",
            trig, (unsigned long long)g_corr_read_total, g_corr_read_used);
    /* Sort by count descending (simple selection sort, n<=128). */
    for (unsigned k = 0; k < g_corr_read_used; k++) {
        unsigned best = k;
        for (unsigned i = k + 1; i < g_corr_read_used; i++) {
            if (g_corr_read_hist[i].count > g_corr_read_hist[best].count)
                best = i;
        }
        if (best != k) {
            CorrReadEntry tmp = g_corr_read_hist[k];
            g_corr_read_hist[k] = g_corr_read_hist[best];
            g_corr_read_hist[best] = tmp;
        }
        fprintf(stderr, "[c54x] CORR-READ #%u addr=0x%04x count=%u\n",
                k + 1, g_corr_read_hist[k].addr, g_corr_read_hist[k].count);
    }
}

/* === DSP throughput emission ===
 *
 * Emits `[c54x] INSN-COUNT-STATS total=N delta=N elapsed_ms=N rate=N/s` every
 * 1M instructions. Two tests read it:
 *   - test_dsp_throughput_5x (milestones, static)
 *   - test_dsp_throughput_above_threshold (observability, runtime)
 * The pytest threshold is 50M/s, a x2 margin below the 100M/s measured here. */
#include <time.h>
struct c54x_g_throughput_s g_throughput;

inline void throughput_tick(uint64_t insn_count)
{
    if (insn_count - g_throughput.last_logged_insn < 1000000) return;
    struct timespec now;
    clock_gettime(CLOCK_MONOTONIC, &now);
    if (g_throughput.last_logged_ts.tv_sec == 0 &&
        g_throughput.last_logged_ts.tv_nsec == 0) {
        g_throughput.last_logged_ts = now;
        g_throughput.last_logged_insn = insn_count;
        return;
    }
    int64_t delta_ns =
        (int64_t)(now.tv_sec - g_throughput.last_logged_ts.tv_sec) * 1000000000LL +
        (int64_t)(now.tv_nsec - g_throughput.last_logged_ts.tv_nsec);
    uint64_t delta_insn = insn_count - g_throughput.last_logged_insn;
    uint64_t rate = (delta_ns > 0)
        ? (delta_insn * 1000000000ULL / (uint64_t)delta_ns) : 0;
    if (calypso_debug_enabled("INSN-COUNT-STATS")) fprintf(stderr,
            "[c54x] INSN-COUNT-STATS total=%llu delta=%llu elapsed_ms=%lld rate=%llu/s\n",
            (unsigned long long)insn_count,
            (unsigned long long)delta_insn,
            (long long)(delta_ns / 1000000),
            (unsigned long long)rate);
    g_throughput.last_logged_ts = now;
    g_throughput.last_logged_insn = insn_count;
}

/* === Read-by-range tracking for FB-det path analysis ===
 *
 * Identifies which DARAM zone the FB-det routine reads. Cumulative counters
 * per range, plus a snapshot/delta at each trigger PC, so the delta between
 * two consecutive triggers is the reads accumulated in the window before it.
 *
 * Mutually exclusive ranges:
 *   RR_MMRS   [0x0000..0x005F]  C54x MMR registers
 *   RR_LOW    [0x0060..0x03A3]  linear correlator zone
 *   RR_APIRAM [0x0800..0x27FF]  ARM/DSP shared API RAM
 *   RR_TARGET [0x3FB0..0x3FFF]  where the BSP DMA writes by default
 *   RR_WRAP   [0xFC5D..0xFFED]  correlator wrap zone, BK=176 (AR2/AR7)
 *   RR_OTHER  everything else (overlay 0x80..0x7FF, above 0x4000, ...) */

struct c54x_g_read_stats_s g_read_stats;

inline void read_stats_record(uint16_t addr)
{
    int r;
    if      (addr <= 0x005F)                    r = RR_MMRS;
    else if (addr <= 0x03A3)                    r = RR_LOW;
    else if (addr >= 0x0800 && addr <= 0x27FF)  r = RR_APIRAM;
    else if (addr >= 0x3FB0 && addr <= 0x3FFF)  r = RR_TARGET;
    else if (addr >= 0xFC5D && addr <= 0xFFED)  r = RR_WRAP;
    else                                        r = RR_OTHER;
    g_read_stats.cumulative[r]++;
}

void read_stats_trigger_check(C54xState *s)
{
    /* [2026-05-14] Single trigger PC 0x8f51 (FB-det compute loop, 50 hits per
     * sweep). The wider list {0x8f51, 0x778a, 0x9ac0, 0x9ad0, 0x9b00, 0x821a}
     * did not work: 0x821a sits in the boot mailbox poll loop, 14 insns apart,
     * and ate the 200-line cap before 0x8f51 ever fired; the other PCs were
     * init/reset code, 1-3 hits each over a whole run. The cap is now 5000, to
     * cover several FB-det sweeps. */
    if (s->pc != 0x8f51) return;
    g_read_stats.trigger_count++;
    if (g_read_stats.trigger_count > 5000) return;
    uint64_t delta[RR_NUM];
    for (int r = 0; r < RR_NUM; r++) {
        delta[r] = g_read_stats.cumulative[r] - g_read_stats.snapshot[r];
        g_read_stats.snapshot[r] = g_read_stats.cumulative[r];
    }
    if (calypso_debug_enabled("READ-AMONT")) fprintf(stderr,
            "[c54x] READ-AMONT #%llu PC=0x%04x insn=%u "
            "mmrs=%llu low=%llu apiram=%llu target=%llu wrap=%llu other=%llu\n",
            (unsigned long long)g_read_stats.trigger_count, s->pc, s->insn_count,
            (unsigned long long)delta[RR_MMRS],
            (unsigned long long)delta[RR_LOW],
            (unsigned long long)delta[RR_APIRAM],
            (unsigned long long)delta[RR_TARGET],
            (unsigned long long)delta[RR_WRAP],
            (unsigned long long)delta[RR_OTHER]);
}

/* === NOP-region guard + transfer ring + A-write ring (2026-05-27 Plan B) ===
 * Trip ONCE on first entry into the unmapped prog zone (= PC < 0x7000 in
 * bank 0, outside OVLY DARAM 0x80-0x27FF). At trip, dump :
 *   (a) trigger transfer (the call/branch that landed in NOP zone)
 *   (b) N last control-flow transfers (most recent → oldest)
 *   (c) N last A-writes
 * Together they name the root cause without speculation. */



XferLog   g_xfer_ring[NOP_RING_N];
unsigned  g_xfer_idx;
AWriteLog g_awrite_ring[NOP_RING_N];
unsigned  g_awrite_idx;
int       g_nop_tripped;

const char *classify_xfer_op(uint16_t op)
{
    if ((op & 0xFF80) == 0xF880) return "FB";
    if ((op & 0xFF80) == 0xF980) return "FCALL";
    if ((op & 0xFF80) == 0xFA80) return "FBD";
    if ((op & 0xFF80) == 0xFB80) return "FCALLD";
    if (op == 0xF4E2 || op == 0xF5E2) return "BACC";
    if (op == 0xF4E3 || op == 0xF5E3) return "CALA";
    if (op == 0xF4E6 || op == 0xF5E6) return "FBACC";
    if (op == 0xF4E7 || op == 0xF5E7) return "FCALA";
    if (op == 0xF6E6) return "FBACCD";
    if (op == 0xF6E7) return "FCALAD";
    if (op == 0xF4E4) return "FRET";
    if (op == 0xF4EB) return "RETE";
    if (op == 0xF6E4 || op == 0xF6E5) return "FRETD";
    if (op == 0xF073) return "B";
    if (op == 0xF273) return "BD";
    if (op == 0xF074) return "CALL";
    if (op == 0xF274) return "CALLD";
    return "OTHER";
}

void xfer_log_push(uint16_t src_pc, uint8_t src_xpc, uint16_t op,
                          uint16_t tgt_pc, uint8_t tgt_xpc, int64_t a_val,
                          uint64_t insn)
{
    XferLog *e = &g_xfer_ring[g_xfer_idx % NOP_RING_N];
    e->src_pc = src_pc;
    e->src_xpc = src_xpc;
    e->op = op;
    e->tgt_pc = tgt_pc;
    e->tgt_xpc = tgt_xpc;
    e->a_val = a_val;
    e->insn = insn;
    const char *t = classify_xfer_op(op);
    /* strncpy without padding */
    int k = 0;
    while (k < 7 && t[k]) { e->type[k] = t[k]; k++; }
    e->type[k] = '\0';
    g_xfer_idx++;
}

void awrite_log_push(uint16_t pc, uint8_t xpc, uint16_t op,
                            int64_t old_a, int64_t new_a, uint64_t insn)
{
    AWriteLog *e = &g_awrite_ring[g_awrite_idx % NOP_RING_N];
    e->pc = pc;
    e->xpc = xpc;
    e->op = op;
    e->old_a = old_a;
    e->new_a = new_a;
    e->insn = insn;
    g_awrite_idx++;
}

/* NOP-region predicate :
 *   xpc == 0 && pc < 0x7000 && !(OVLY && pc in [0x80, 0x2800])
 * Anything that lands here is in the unmapped prog area = NOP slide. */
/* Forward declaration: pc_in_nop_region below needs the OVLY alias floor,
 * defined in another translation unit. */
uint16_t c54x_ovly_bas(void);

inline int pc_in_nop_region(const C54xState *s, uint16_t pc, uint8_t xpc)
{
    if (xpc != 0) return 0;                 /* upper bank: handled elsewhere */
    if (pc >= 0x7000) return 0;             /* PROM0 + PROM1 mirror = valid */
    if ((s->pmst & PMST_OVLY) && pc >= c54x_ovly_bas() && pc < 0x2800)
        return 0;                           /* OVLY DARAM mapping = valid */
    return 1;
}

void nop_guard_dump(C54xState *s, uint16_t pc, uint8_t xpc)
{
    if (g_nop_tripped) return;
    g_nop_tripped = 1;
    C54_LOG("================================================");
    C54_LOG("NOP-REGION GUARD TRIPPED");
    C54_LOG("  trigger PC=0x%04x XPC=%u  prog[lin]=0x%04x  insn=%u",
            pc, xpc, s->prog[((uint32_t)xpc << 16) | pc], s->insn_count);
    C54_LOG("  state : A=%010llx B=%010llx SP=0x%04x ST1=0x%04x INTM=%d "
            "AR0..7: %04x %04x %04x %04x %04x %04x %04x %04x",
            (unsigned long long)(s->a & 0xFFFFFFFFFFULL),
            (unsigned long long)(s->b & 0xFFFFFFFFFFULL),
            s->sp, s->st1, !!(s->st1 & ST1_INTM),
            s->ar[0], s->ar[1], s->ar[2], s->ar[3],
            s->ar[4], s->ar[5], s->ar[6], s->ar[7]);

    C54_LOG("--- last %d control-flow transfers (oldest → newest) ---", NOP_RING_N);
    unsigned start = g_xfer_idx > NOP_RING_N ? (g_xfer_idx - NOP_RING_N) : 0;
    for (unsigned i = start; i < g_xfer_idx; i++) {
        const XferLog *t = &g_xfer_ring[i % NOP_RING_N];
        C54_LOG("  [%u] %-7s src=(xpc=%u,pc=0x%04x) op=0x%04x → tgt=(xpc=%u,pc=0x%04x) "
                "A=%010llx insn=%llu",
                i, t->type, t->src_xpc, t->src_pc, t->op,
                t->tgt_xpc, t->tgt_pc,
                (unsigned long long)(t->a_val & 0xFFFFFFFFFFULL),
                (unsigned long long)t->insn);
    }

    C54_LOG("--- last %d A-writes (oldest → newest) ---", NOP_RING_N);
    unsigned astart = g_awrite_idx > NOP_RING_N ? (g_awrite_idx - NOP_RING_N) : 0;
    for (unsigned i = astart; i < g_awrite_idx; i++) {
        const AWriteLog *a = &g_awrite_ring[i % NOP_RING_N];
        int64_t do_old = a->old_a & 0xFFFFFFFFFFLL;
        int64_t do_new = a->new_a & 0xFFFFFFFFFFLL;
        C54_LOG("  [%u] PC=0x%04x xpc=%u op=0x%04x  A: %010llx → %010llx "
                "(Δ=%+lld) insn=%llu",
                i, a->pc, a->xpc, a->op,
                (unsigned long long)do_old, (unsigned long long)do_new,
                (long long)(do_new - do_old),
                (unsigned long long)a->insn);
    }
    C54_LOG("================================================");
}

