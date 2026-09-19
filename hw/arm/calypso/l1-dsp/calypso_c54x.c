/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * calypso_c54x.c - TMS320C54x core: init, reset, the c54x_run loop, public API.
 *
 * [2026-09-18] Split of a single 21475-line file, by role:
 *   calypso_c54x.c  init / reset / c54x_run loop / public API
 *   c54x_exec.c     c54x_exec_one and the instruction families
 *   c54x_decode.c   operand resolution (Smem/Lmem/Xmem), conditions
 *   c54x_mem.c      data and program memory, overlay, locks
 *   c54x_irq.c      IFR/IMR, frame interrupt, interrupts
 *   c54x_probes.c   probes and traces (diagnostics only)
 */
#include "c54x_internal.h"

static bool dsp_idle_fast_forward(C54xState *s, int *consumed_out)
{
    static int     ff_enabled = -1;
    static int     ff_n_ranges = 0;
    static uint16_t ff_lo[DSP_IDLE_FF_MAX_RANGES];
    static uint16_t ff_hi[DSP_IDLE_FF_MAX_RANGES];
    static uint64_t ff_hits = 0;

    if (ff_enabled < 0) {
        const char *e = getenv("CALYPSO_DSP_IDLE_FF");
        ff_enabled = (!e || *e != '0') ? 1 : 0;
        /* Defaults: two empirically observed dispatcher loops in the
         * stock layer1.highram.elf firmware:
         *   1) 0xe9ac..0xe9b7 — PROM1 mirror, init/SP-aware path
         *   2) 0xcc62..0xcc6f — PROM0 page 0, runtime mailbox poll loop
         * Override via CALYPSO_DSP_IDLE_RANGE="lo1:hi1,lo2:hi2,..."
         * (max 4 ranges). Each range is hex. Empty = use defaults. */
        const char *r = getenv("CALYPSO_DSP_IDLE_RANGE");
        if (r && *r) {
            const char *p = r;
            while (*p && ff_n_ranges < DSP_IDLE_FF_MAX_RANGES) {
                unsigned lo, hi;
                if (sscanf(p, "%x:%x", &lo, &hi) == 2 && lo <= hi &&
                    lo <= 0xFFFF && hi <= 0xFFFF) {
                    ff_lo[ff_n_ranges] = (uint16_t)lo;
                    ff_hi[ff_n_ranges] = (uint16_t)hi;
                    ff_n_ranges++;
                }
                while (*p && *p != ',') p++;
                if (*p == ',') p++;
            }
        }
        if (ff_n_ranges == 0) {
            ff_lo[0] = 0xe9ac; ff_hi[0] = 0xe9b7;
            ff_lo[1] = 0xcc62; ff_hi[1] = 0xcc6f;
            ff_n_ranges = 2;
        }
        char buf[160] = ""; int blen = 0;
        for (int i = 0; i < ff_n_ranges; i++) {
            blen += snprintf(buf + blen, sizeof(buf) - blen,
                             "%s0x%04x..0x%04x",
                             i ? "," : "", ff_lo[i], ff_hi[i]);
        }
        C54_LOG("DSP IDLE FF: %s, ranges=[%s]",
                ff_enabled ? "enabled" : "disabled", buf);
    }
    if (!ff_enabled) return false;
    bool in_range = false;
    for (int i = 0; i < ff_n_ranges; i++) {
        if (s->pc >= ff_lo[i] && s->pc <= ff_hi[i]) {
            in_range = true;
            break;
        }
    }
    if (!in_range) return false;

    /* Task slots in both write pages - DSP word addresses:
     *   page 0 : 0x0800 (d_task_d), 0x0802 (d_task_u),
     *            0x0804 (d_task_md), 0x0807 (d_task_ra)
     *   page 1 : 0x0814, 0x0816, 0x0818, 0x081B (offsets +0x14)
     */
    if (s->data[0x0800] | s->data[0x0802] | s->data[0x0804] | s->data[0x0807] |
        s->data[0x0814] | s->data[0x0816] | s->data[0x0818] | s->data[0x081B]) {
        return false;  /* something pending → exec normally */
    }
    /* Pending IRQ would also break us out of the dispatcher next iter. */
    if (!(s->st1 & ST1_INTM) && (s->ifr & s->imr)) {
        return false;
    }

    /* Fast-forward this dispatcher iteration.
     *
     * Cycle budget: a real C54x at 65 MHz gives 1 cycle = 15 ns. The
     * dispatcher body is ~8 instructions per pass (matches the 8 hot PCs
     * observed), so one pass = 8 cycles = 120 ns of DSP time. The caller caps
     * the fast-forward run length per c54x_run invocation so the skips never
     * overshoot the n_insns budget.
     *
     * Wall-clock alignment (CLK IND cadence) belongs to the TDMA timer in
     * calypso_trx.c, not to this function. */
    *consumed_out = 8;
    ff_hits++;
    if ((ff_hits & 0xFFFFFFu) == 0) {
        C54_LOG("DSP IDLE FF: %llu skips so far (PC=0x%04x SP=0x%04x)",
                (unsigned long long)ff_hits, s->pc, s->sp);
    }
    return true;
}

/* SP observability state for the CALYPSO_TRAP_OOR probe. Pure observation,
 * no PC whitelist: the SP clobber lives in legitimate code.
 *   - g_sp_trail[256] : SP changes with |delta| > 32 (scheduler reloads, large
 *     allocations); push/pop +-1 noise is skipped.
 *   - sp_low watermark : every new low, coalesced per PC on powers of ten.
 *     Catches both absolute reloads and push-drain runaway.
 *   - Each event records A_low, the candidate STL A,Smem source.
 *   - Halt at a fixed checkpoint (CALYPSO_TRAP_CHECKPOINT, default 4200000). */

struct c54x_g_sp_trail_s g_sp_trail[256];
unsigned g_sp_trail_idx = 0;

/* sp_low watermark, coalesced per PC */
uint16_t g_sp_low = 0xFFFF;
uint16_t g_sp_low_pc = 0xFFFF;
unsigned g_sp_low_hits_at_pc = 0;
unsigned g_sp_low_distinct_pcs = 0;

/* Per-PC SP-decrement histogram.
 *
 * Gated on the SP VALUE, not on insn_count. insn_count is inflated by the
 * idle fast-forward (which credits cycles without executing opcodes) and
 * jittered by external I/O (bridge/osmocon/BTS over UDP+PTY move the guest
 * instant at which a burst arrives), so an insn window is not reproducible
 * run to run.
 *
 * The histogram arms when SP falls below the plateau (default < 0x2000) and
 * stays armed until SP < 0x0100 (near underflow); inside that window every
 * SP decrement is counted per PC. The window auto-aligns on the descent
 * whatever the fast-forward and the external jitter do, and it excludes the
 * balanced PSHM/POP churn of the plateau, so a real leaker dominates
 * mechanically.
 *
 * Env overrides:
 *   CALYPSO_SP_HIST_ARM   (default 0x2000) - arm threshold
 *   CALYPSO_SP_HIST_DUMP  (default 0x0100) - dump threshold
 *
 * Covers every SP-write path (direct s->sp--, MMR_SP through the data_write
 * callback, IRQ push): they all go through the s->sp variable. */
SpDecEntry g_sp_dec_hist[SP_HIST_MAX];
unsigned   g_sp_dec_used = 0;
unsigned   g_sp_dec_total_events = 0;
uint16_t   g_sp_dec_arm_threshold = 0;
uint16_t   g_sp_dec_dump_threshold = 0;
int        g_sp_dec_enabled = -1;
int        g_sp_dec_armed = 0;
int        g_sp_dec_dumped = 0;
unsigned   g_sp_dec_arm_insn = 0;   /* insn at which we armed */;
uint16_t   g_sp_dec_arm_sp = 0;     /* SP value at arm */;

/* Raw SP ring buffer: per-iteration (insn, PC, SP, op) recorded at the top of
 * the run loop, unfiltered and unclassified. Two trigger modes.
 *
 * "bootstub" fires on the edge prev_pc outside [0x00,0x7F] -> s->pc inside it,
 * which captures the offending RET together with its SP and the popped word
 * mem[topgate_last_sp]. That discriminates two unrelated bugs:
 *   - valid SP (~0x3fbb) with mem[SP] == 0 : the return slot was overwritten
 *     by a stray write; fd28-fd2a is not involved.
 *   - SP outside the stack (~0x2bc0) : the 0xfd2a A=AR4 family, fixed in
 *     fd28-fd2a.
 * "floor" fires on the SP floor crossing, 600k insns later, already inside the
 * boot-stub spiral where pops wrap SP forward - too late to name the culprit.
 *
 * Env gates:
 *   CALYPSO_SP_RING=1          enable (default OFF, zero cost otherwise)
 *   CALYPSO_SP_RING_MAX=N      dumps per run (default 4)
 *   CALYPSO_SP_RING_TRIG=mode  floor|bootstub|both (default bootstub)
 *   CALYPSO_SP_RING_INSN_MIN=N skip the first N insns (default 1000000: the
 *                              firmware makes legitimate CALLs into the boot
 *                              stub 0x0000/0x0001 during init, and the first
 *                              one would burn the one-shot on a false
 *                              positive) */
SpRingEntry g_sp_ring[SP_RING_SZ];
unsigned    g_sp_ring_head = 0;
uint64_t    g_sp_ring_total = 0;
int         g_sp_ring_enabled = -1;
unsigned    g_sp_ring_dump_count = 0;
unsigned    g_sp_ring_dump_max = 0;
/* Trigger mode: 1 = floor-cross, 2 = bootstub-entry, 3 = both */
int         g_sp_ring_trig_mode = 0;
unsigned    g_sp_ring_insn_min  = 0;  /* skip first N insns (boot phase) */;

static void sp_ring_record(unsigned insn, uint16_t pc, uint16_t sp, uint16_t op)
{
    if (g_sp_ring_enabled <= 0) return;
    SpRingEntry *e = &g_sp_ring[g_sp_ring_head];
    e->insn = insn; e->pc = pc; e->sp = sp; e->op = op;
    g_sp_ring_head = (g_sp_ring_head + 1) & (SP_RING_SZ - 1);
    g_sp_ring_total++;
}

static void sp_ring_dump(const char *trig, unsigned insn_now, uint16_t sp_now)
{
    if (g_sp_ring_enabled <= 0) return;
    if (g_sp_ring_dump_max && g_sp_ring_dump_count >= g_sp_ring_dump_max) return;
    g_sp_ring_dump_count++;
    fprintf(stderr,
        "[c54x] SP-RING DUMP[%s] @insn=%u sp_now=0x%04x total_recorded=%llu "
        "dump#%u\n",
        trig, insn_now, sp_now,
        (unsigned long long)g_sp_ring_total, g_sp_ring_dump_count);
    unsigned n = (g_sp_ring_total < SP_RING_SZ)
                 ? (unsigned)g_sp_ring_total : SP_RING_SZ;
    unsigned start = (g_sp_ring_total < SP_RING_SZ) ? 0 : g_sp_ring_head;
    for (unsigned k = 0; k < n; k++) {
        unsigned idx = (start + k) & (SP_RING_SZ - 1);
        SpRingEntry *e = &g_sp_ring[idx];
        fprintf(stderr,
            "[c54x] SP-RING[%u] insn=%u PC=0x%04x SP=0x%04x op=0x%04x\n",
            k, e->insn, e->pc, e->sp, e->op);
    }
}

static void sp_ring_init_lazy(void)
{
    if (g_sp_ring_enabled >= 0) return;
    const char *e = cdbg_env("SP-RING");
    g_sp_ring_enabled = (e && *e == '1') ? 1 : 0;
    const char *m = getenv("CALYPSO_SP_RING_MAX");
    g_sp_ring_dump_max = (m && *m) ? (unsigned)strtoul(m, NULL, 0) : 4u;
    /* Default trigger is bootstub; floor-cross fires inside the spiral, too
     * late to name the offending RET. */
    const char *t = getenv("CALYPSO_SP_RING_TRIG");
    if (!t || !*t || !strcmp(t, "bootstub")) g_sp_ring_trig_mode = 2;
    else if (!strcmp(t, "floor"))            g_sp_ring_trig_mode = 1;
    else if (!strcmp(t, "both"))             g_sp_ring_trig_mode = 3;
    else                                     g_sp_ring_trig_mode = 2;
    const char *im = getenv("CALYPSO_SP_RING_INSN_MIN");
    g_sp_ring_insn_min = (im && *im) ? (unsigned)strtoul(im, NULL, 0) : 1000000u;
    if (g_sp_ring_enabled) {
        fprintf(stderr,
            "[c54x] SP-RING enabled, sz=%u, max_dumps=%u, trig=%s, "
            "insn_min=%u\n",
            SP_RING_SZ, g_sp_ring_dump_max,
            g_sp_ring_trig_mode == 1 ? "floor" :
            g_sp_ring_trig_mode == 2 ? "bootstub" :
            g_sp_ring_trig_mode == 3 ? "both" : "?",
            g_sp_ring_insn_min);
    }
}

/* Detect the edge where PC enters the boot-stub area [0x0000, 0x007F].
 * topgate_last_pc = PC of the insn just executed (the RET that branched)
 * cur_pc          = destination = popped return address
 * topgate_last_sp = SP before the RET pop
 * cur_sp          = SP after it (= topgate_last_sp + 1 for a 1-word return)
 * Dumps the verbose state plus the ring, to identify the corrupting RET.
 * Capped at one detailed dump per run: the first one holds the caller, later
 * fires are re-entries of the same spiral. */
int g_bootstub_dumped = 0;
static void sp_ring_check_bootstub_entry(C54xState *s,
                                         uint16_t prev_pc, uint16_t prev_op,
                                         uint16_t prev_sp, uint16_t cur_pc,
                                         uint16_t cur_sp, unsigned insn)
{
    if (g_sp_ring_enabled <= 0) return;
    if (!(g_sp_ring_trig_mode & 2)) return;
    if (g_bootstub_dumped) return;
    /* Skip the boot phase: the firmware makes legitimate CALLs into the boot
     * stub 0x0000-0x0001 during init (LDMM SP,B is documented). Without this
     * gate the first trigger fires at insn=145 and burns the one-shot. */
    if (insn < g_sp_ring_insn_min) return;
    int was_inside = (prev_pc <= 0x007F);
    int now_inside = (cur_pc  <= 0x007F);
    if (was_inside || !now_inside) return;

    g_bootstub_dumped = 1;
    uint16_t popped = s->data[prev_sp & 0xFFFF];
    uint16_t neighborhood[8];
    for (int k = 0; k < 8; k++) {
        neighborhood[k] = s->data[(uint16_t)(prev_sp + k) & 0xFFFF];
    }
    fprintf(stderr,
        "[c54x] BOOTSTUB-ENTRY caught @insn=%u\n"
        "[c54x]   prev_pc=0x%04x prev_op=0x%04x  (the RET site = corrupter)\n"
        "[c54x]   cur_pc=0x%04x  (destination, in bootstub)\n"
        "[c54x]   prev_sp=0x%04x  cur_sp=0x%04x  (delta=%+d)\n"
        "[c54x]   mem[prev_sp]=0x%04x  (= popped return addr, must equal cur_pc)\n"
        "[c54x]   AR0..7: %04x %04x %04x %04x %04x %04x %04x %04x  ARP=%d DP=%d\n"
        "[c54x]   ST0=0x%04x ST1=0x%04x INTM=%d XPC=%d\n"
        "[c54x]   stack neighborhood mem[prev_sp..+7]: %04x %04x %04x %04x %04x %04x %04x %04x\n",
        insn, prev_pc, prev_op, cur_pc,
        prev_sp, cur_sp, (int)cur_sp - (int)prev_sp,
        popped,
        s->ar[0], s->ar[1], s->ar[2], s->ar[3],
        s->ar[4], s->ar[5], s->ar[6], s->ar[7],
        arp(s), dp(s),
        s->st0, s->st1, !!(s->st1 & ST1_INTM), s->xpc,
        neighborhood[0], neighborhood[1], neighborhood[2], neighborhood[3],
        neighborhood[4], neighborhood[5], neighborhood[6], neighborhood[7]);

    /* Discriminator:
     *   valid SP (~0x3fbb observed) with popped == 0 -> the return slot was
     *     overwritten by a stray write; 0xfd2a is innocent.
     *   SP in a non-stack buffer (~0x2bc0 or similar) -> the 0xfd2a A=AR4
     *     family; fd28-fd2a is the fix. */
    int sp_in_valid_stack = (prev_sp >= 0x3000 && prev_sp <= 0x5FFF);
    int sp_in_buffer_area = (prev_sp >= 0x2000 && prev_sp <= 0x2FFF);
    if (calypso_debug_enabled("BOOTSTUB-ENTRY")) fprintf(stderr,
        "[c54x] BOOTSTUB-ENTRY VERDICT: sp_in_valid_stack=%d "
        "sp_in_buffer_area=%d popped_is_zero=%d\n"
        "[c54x]   → %s\n",
        sp_in_valid_stack, sp_in_buffer_area, (popped == 0),
        sp_in_valid_stack && popped == 0
            ? "RETURN SLOT OVERWRITTEN (sauvage write near SP), audit ailleurs"
            : sp_in_buffer_area
            ? "SP IN NON-STACK BUFFER (likely 0xfd2a family), audit fd28-fd2a"
            : "INCONCLUSIVE — inspect ring + state above");

    sp_ring_dump("bootstub-entry", insn, cur_sp);
}

static void sp_hist_dump(const char *trig, unsigned insn_now, uint16_t sp_now)
{
    if (calypso_debug_enabled("SP-HIST")) fprintf(stderr,
        "[c54x] SP-HIST DUMP[%s] arm@(insn=%u,sp=0x%04x) now@(insn=%u,sp=0x%04x) "
        "events=%u distinct_pcs=%u\n",
        trig, g_sp_dec_arm_insn, g_sp_dec_arm_sp, insn_now, sp_now,
        g_sp_dec_total_events, g_sp_dec_used);

    /* Top-K by dec_count (trickle leak). */
    if (calypso_debug_enabled("SP-HIST")) fprintf(stderr, "[c54x] SP-HIST TOP BY COUNT (corrupteur trickle):\n");
    for (unsigned k = 0; k < 20 && k < g_sp_dec_used; k++) {
        unsigned best = k;
        for (unsigned i = k + 1; i < g_sp_dec_used; i++) {
            if (g_sp_dec_hist[i].dec_count > g_sp_dec_hist[best].dec_count)
                best = i;
        }
        if (best != k) {
            SpDecEntry tmp = g_sp_dec_hist[k];
            g_sp_dec_hist[k] = g_sp_dec_hist[best];
            g_sp_dec_hist[best] = tmp;
        }
        if (calypso_debug_enabled("SP-HIST")) fprintf(stderr,
            "[c54x] SP-HIST #%u pc=0x%04x op_last=0x%04x dec_count=%u "
            "delta_sum=%d\n",
            k + 1, g_sp_dec_hist[k].pc, g_sp_dec_hist[k].op_last,
            g_sp_dec_hist[k].dec_count, g_sp_dec_hist[k].delta_sum);
    }

    /* Top-K by |delta_sum| (single-event corrupter: one huge event). */
    if (calypso_debug_enabled("SP-HIST")) fprintf(stderr, "[c54x] SP-HIST TOP BY |delta_sum| (corrupteur single-jump):\n");
    for (unsigned k = 0; k < 10 && k < g_sp_dec_used; k++) {
        unsigned best = k;
        int32_t best_abs = g_sp_dec_hist[k].delta_sum < 0
                           ? -g_sp_dec_hist[k].delta_sum
                           :  g_sp_dec_hist[k].delta_sum;
        for (unsigned i = k + 1; i < g_sp_dec_used; i++) {
            int32_t a = g_sp_dec_hist[i].delta_sum < 0
                        ? -g_sp_dec_hist[i].delta_sum
                        :  g_sp_dec_hist[i].delta_sum;
            if (a > best_abs) { best = i; best_abs = a; }
        }
        if (best != k) {
            SpDecEntry tmp = g_sp_dec_hist[k];
            g_sp_dec_hist[k] = g_sp_dec_hist[best];
            g_sp_dec_hist[best] = tmp;
        }
        if (calypso_debug_enabled("SP-HIST")) fprintf(stderr,
            "[c54x] SP-HIST D#%u pc=0x%04x op_last=0x%04x dec_count=%u "
            "delta_sum=%d\n",
            k + 1, g_sp_dec_hist[k].pc, g_sp_dec_hist[k].op_last,
            g_sp_dec_hist[k].dec_count, g_sp_dec_hist[k].delta_sum);
    }
    g_sp_dec_dumped = 1;
}

static void sp_hist_account(uint16_t exec_pc, uint16_t exec_op,
                            uint16_t sp_before, uint16_t sp_now,
                            unsigned insn)
{
    if (g_sp_dec_enabled < 0) {
        const char *e_arm = getenv("CALYPSO_SP_HIST_ARM");
        const char *e_dump = getenv("CALYPSO_SP_HIST_DUMP");
        unsigned arm  = (e_arm  && *e_arm)  ? (unsigned)strtoul(e_arm,  NULL, 0) : 0x2000u;
        unsigned dump = (e_dump && *e_dump) ? (unsigned)strtoul(e_dump, NULL, 0) : 0x0100u;
        if (arm > 0xFFFF) arm = 0xFFFF;
        if (dump > 0xFFFF) dump = 0xFFFF;
        if (dump >= arm) dump = (arm > 0x100) ? (arm - 0x100) : 0;
        g_sp_dec_arm_threshold  = (uint16_t)arm;
        g_sp_dec_dump_threshold = (uint16_t)dump;
        g_sp_dec_enabled = calypso_debug_enabled("SP-HIST") ? 1 : 0;
        fprintf(stderr,
            "[c54x] SP-HIST gating SP-value : ARM<0x%04x DUMP<0x%04x\n",
            g_sp_dec_arm_threshold, g_sp_dec_dump_threshold);
    }
    if (!g_sp_dec_enabled) return;
    /* No one-shot freeze here: returning early once dumped would drop every
     * event after the first dump. Repeat dumps while SP stays under the
     * threshold are bounded by sp_ring_dump_max and by the top-of-loop edge
     * detection. */

    /* No (int16_t) cast on the delta: the signed wrap misclassified high->low
     * falls as pops. Plain int32 subtraction instead:
     *   0x9006 -> 0x0000 : delta = -36870 (descent captured)
     *   0xC000 -> 0x0000 : delta = -49152 (descent captured)
     * This does break the underflow wrap (0x2bc0 -> 0xfff8 = +52280, seen as
     * a pop), but the ring buffer, not this histogram, decides the kill; the
     * histogram only tracks trickle drift. */
    if (!g_sp_dec_armed) {
        int32_t first_check = (int32_t)sp_now - (int32_t)sp_before;
        if (first_check < 0) {
            g_sp_dec_armed = 1;
            g_sp_dec_arm_insn = insn;
            g_sp_dec_arm_sp = sp_now;
            fprintf(stderr,
                "[c54x] SP-HIST ARMED @insn=%u SP=0x%04x (sp_before=0x%04x delta=%d) "
                "pc=0x%04x op=0x%04x\n",
                insn, sp_now, sp_before, first_check, exec_pc, exec_op);
        } else {
            return;  /* no negative delta yet — wait */
        }
    }

    /* Record the event BEFORE the dump check, otherwise a single-event jump
     * that crosses the dump threshold in one instruction is lost. */
    int32_t delta = (int32_t)sp_now - (int32_t)sp_before;
    if (delta < 0) {
        g_sp_dec_total_events++;
        unsigned i;
        for (i = 0; i < g_sp_dec_used; i++) {
            if (g_sp_dec_hist[i].pc == exec_pc) break;
        }
        if (i == g_sp_dec_used) {
            if (g_sp_dec_used >= SP_HIST_MAX) {
                static int sat_log = 0;
                if (!sat_log) {
                    fprintf(stderr,
                        "[c54x] SP-HIST saturated (>%u distinct PCs) — "
                        "broaden if needed\n", SP_HIST_MAX);
                    sat_log = 1;
                }
            } else {
                g_sp_dec_hist[i].pc = exec_pc;
                g_sp_dec_hist[i].op_last = exec_op;
                g_sp_dec_hist[i].dec_count = 0;
                g_sp_dec_hist[i].delta_sum = 0;
                g_sp_dec_used++;
            }
        }
        if (i < g_sp_dec_used) {
            g_sp_dec_hist[i].op_last = exec_op;
            g_sp_dec_hist[i].dec_count++;
            g_sp_dec_hist[i].delta_sum += delta;
        }
        /* Log the first 10 events verbatim: for a single-event jump the
         * corrupter is among them (often the only entry in the histogram). */
        if (g_sp_dec_total_events <= 10) {
            if (calypso_debug_enabled("SP-HIST")) fprintf(stderr,
                "[c54x] SP-HIST EVENT #%u pc=0x%04x op=0x%04x "
                "sp_before=0x%04x sp_now=0x%04x delta=%d insn=%u\n",
                g_sp_dec_total_events, exec_pc, exec_op,
                sp_before, sp_now, delta, insn);
        }
    }

    /* Dump AFTER the accounting. Edge-triggered only: fire when SP crosses
     * below the floor (sp_before >= threshold && sp_now < threshold). Gated by
     * bit 0 of g_sp_ring_trig_mode; bootstub is the default because
     * floor-cross fires inside the spiral, too late. */
    if ((g_sp_ring_trig_mode & 1) &&
        sp_before >= g_sp_dec_dump_threshold &&
        sp_now    <  g_sp_dec_dump_threshold) {
        sp_ring_dump("sp-floor-cross", insn, sp_now);
        sp_hist_dump("sp-floor-cross", insn, sp_now);
    }
}

static void dsp_trap_dump(C54xState *s, uint16_t exec_pc, uint16_t exec_op,
                          uint16_t sp_before, const char *trig)
{
    if (calypso_debug_enabled("TRAP")) fprintf(stderr,
        "[c54x] TRAP[%s] insn=%u exec_pc=0x%04x exec_op=0x%04x "
        "next_pc=0x%04x sp_before=0x%04x sp_now=0x%04x INTM=%d\n",
        trig, s->insn_count, exec_pc, exec_op, s->pc,
        sp_before, s->sp, !!(s->st1 & ST1_INTM));
    if (calypso_debug_enabled("TRAP")) fprintf(stderr, "[c54x] TRAP pc_ring[-16..-1]:");
    for (int i = 16; i >= 1; i--)
        fprintf(stderr, " %04x", pc_ring[(pc_ring_idx - i) & 255]);
    fprintf(stderr, "\n[c54x] TRAP sp_low=0x%04x at last_pc=0x%04x hits_at_pc=%u distinct_pcs=%u\n",
            g_sp_low, g_sp_low_pc, g_sp_low_hits_at_pc, g_sp_low_distinct_pcs);
    /* SP-HIST dump (SP-windowed, independent of insn_count). */
    if (g_sp_dec_used > 0 && !g_sp_dec_dumped)
        sp_hist_dump("trap", s->insn_count, s->sp);
    fprintf(stderr, "[c54x] TRAP sp_trail[-256..-1] (|Δ|>32 only; insn old->new @pc op A_low):\n");
    for (int i = 256; i >= 1; i--) {
        unsigned k = (g_sp_trail_idx - i) & 255;
        if (g_sp_trail[k].insn == 0 && g_sp_trail[k].exec_pc == 0) continue;
        fprintf(stderr, "  %u  %04x->%04x  pc=%04x op=%04x A_low=%04x\n",
                g_sp_trail[k].insn, g_sp_trail[k].old_sp,
                g_sp_trail[k].new_sp, g_sp_trail[k].exec_pc,
                g_sp_trail[k].exec_op, g_sp_trail[k].a_low);
    }
    fprintf(stderr,
        "[c54x] TRAP regs A=%010llx B=%010llx T=%04x  "
        "AR0..7: %04x %04x %04x %04x %04x %04x %04x %04x  "
        "BK=%04x ARP=%d DP=%d  ST0=%04x ST1=%04x PMST=%04x  "
        "RSA=%04x REA=%04x BRC=%d  IFR=%04x IMR=%04x XPC=%d\n",
        (unsigned long long)(s->a & 0xFFFFFFFFFFULL),
        (unsigned long long)(s->b & 0xFFFFFFFFFFULL),
        s->t,
        s->ar[0], s->ar[1], s->ar[2], s->ar[3],
        s->ar[4], s->ar[5], s->ar[6], s->ar[7],
        s->bk, (s->st0 >> 13) & 7, s->st0 & 0x1FF,
        s->st0, s->st1, s->pmst,
        s->rsa, s->rea, s->brc, s->ifr, s->imr, s->xpc);
    fprintf(stderr,
        "[c54x] TRAP prog[exec_pc..+3]=%04x %04x %04x %04x  "
        "prog[next_pc..+3]=%04x %04x %04x %04x\n",
        s->prog[exec_pc], s->prog[(uint16_t)(exec_pc+1)],
        s->prog[(uint16_t)(exec_pc+2)], s->prog[(uint16_t)(exec_pc+3)],
        s->prog[s->pc], s->prog[(uint16_t)(s->pc+1)],
        s->prog[(uint16_t)(s->pc+2)], s->prog[(uint16_t)(s->pc+3)]);
}

int c54x_run(C54xState *s, int n_insns)
{
    int executed = 0;

    /* Run counter, used by the BOOT trace further down. */
    static int run_num = 0;
    run_num++;

    /* SP history ring buffer (64 entries × insn/PC/SP). Sampled every
     * 1M insns at top of run-loop. Dumped on STATE-DUMP. Reveals whether
     * SP descends monotonically (cumulative leak — each ISR entry leaks
     * one stack frame) or oscillates around a value (one big initial
     * drop then steady-state). Different fixes. */
    static struct { unsigned insn; uint16_t pc; uint16_t sp; } sp_ring[64];
    static unsigned sp_ring_idx = 0;
    static unsigned next_sp_sample = 1000000u;
    if (s->insn_count >= next_sp_sample) {
        next_sp_sample += 1000000u;
        sp_ring[sp_ring_idx & 63].insn = s->insn_count;
        sp_ring[sp_ring_idx & 63].pc   = s->pc;
        sp_ring[sp_ring_idx & 63].sp   = s->sp;
        sp_ring_idx++;
    }

    /* XPC tracking probe. Tells whether the CCCH demod path ever reaches
     * PROM1 (XPC=1) through the B 0x9ab1 at 0x19aac, and where it lands:
     *   - insn count per XPC (0..3)
     *   - last PC visited per XPC
     *   - first-visit insn per XPC
     *   - ring of the last 16 PCs visited under XPC=1
     */
    {
        static uint64_t xpc_insn_count[4] = {0};
        static uint16_t xpc_last_pc[4]    = {0};
        static uint64_t xpc_first_insn[4] = {0,0,0,0};
        static uint16_t xpc1_pc_ring[16];
        static unsigned xpc1_pc_ring_idx = 0;
        static unsigned xpc1_pc_ring_count = 0;
        static unsigned next_xpc_dump = 100000000u;  /* 100M */
        uint8_t cur_xpc = s->xpc & 0x3;
        xpc_insn_count[cur_xpc]++;
        xpc_last_pc[cur_xpc] = s->pc;
        if (xpc_first_insn[cur_xpc] == 0)
            xpc_first_insn[cur_xpc] = s->insn_count;
        if (cur_xpc == 1) {
            xpc1_pc_ring[xpc1_pc_ring_idx & 15] = s->pc;
            xpc1_pc_ring_idx++;
            xpc1_pc_ring_count++;
        }
        if (s->insn_count >= next_xpc_dump) {
            next_xpc_dump += 100000000u;
            if (calypso_debug_enabled("XPC-STATS")) fprintf(stderr,
                    "[c54x] XPC-STATS insn=%u counts: 0=%llu 1=%llu 2=%llu 3=%llu | "
                    "first_insn: 0=%llu 1=%llu 2=%llu 3=%llu | last_pc: 0=0x%04x 1=0x%04x 2=0x%04x 3=0x%04x\n",
                    s->insn_count,
                    (unsigned long long)xpc_insn_count[0],
                    (unsigned long long)xpc_insn_count[1],
                    (unsigned long long)xpc_insn_count[2],
                    (unsigned long long)xpc_insn_count[3],
                    (unsigned long long)xpc_first_insn[0],
                    (unsigned long long)xpc_first_insn[1],
                    (unsigned long long)xpc_first_insn[2],
                    (unsigned long long)xpc_first_insn[3],
                    xpc_last_pc[0], xpc_last_pc[1], xpc_last_pc[2], xpc_last_pc[3]);
            if (xpc1_pc_ring_count > 0) {
                /* Last 16 PCs visited under XPC=1 */
                if (calypso_debug_enabled("XPC1-PC-RING")) fprintf(stderr,
                        "[c54x] XPC1-PC-RING count=%u last16: "
                        "%04x %04x %04x %04x %04x %04x %04x %04x "
                        "%04x %04x %04x %04x %04x %04x %04x %04x\n",
                        xpc1_pc_ring_count,
                        xpc1_pc_ring[(xpc1_pc_ring_idx-16)&15],
                        xpc1_pc_ring[(xpc1_pc_ring_idx-15)&15],
                        xpc1_pc_ring[(xpc1_pc_ring_idx-14)&15],
                        xpc1_pc_ring[(xpc1_pc_ring_idx-13)&15],
                        xpc1_pc_ring[(xpc1_pc_ring_idx-12)&15],
                        xpc1_pc_ring[(xpc1_pc_ring_idx-11)&15],
                        xpc1_pc_ring[(xpc1_pc_ring_idx-10)&15],
                        xpc1_pc_ring[(xpc1_pc_ring_idx-9)&15],
                        xpc1_pc_ring[(xpc1_pc_ring_idx-8)&15],
                        xpc1_pc_ring[(xpc1_pc_ring_idx-7)&15],
                        xpc1_pc_ring[(xpc1_pc_ring_idx-6)&15],
                        xpc1_pc_ring[(xpc1_pc_ring_idx-5)&15],
                        xpc1_pc_ring[(xpc1_pc_ring_idx-4)&15],
                        xpc1_pc_ring[(xpc1_pc_ring_idx-3)&15],
                        xpc1_pc_ring[(xpc1_pc_ring_idx-2)&15],
                        xpc1_pc_ring[(xpc1_pc_ring_idx-1)&15]);
            }
        }
    }

    /* DISPATCH-CALLER probe. The three callers of 0x9aaf found by a PROM
     * scan:
     *   PC=0x8815 : f074 9aaf  (B 0x9aaf from the table at 0x8810)
     *   PC=0x9296 : f274 9aaf  (BD 0x9aaf)
     *   PC=0x9418 : f274 9aaf  (BD 0x9aaf)
     * Logs A, AR0..2 and data[0x0828/9] on each hit. */
    if (s->pc == 0x8815 || s->pc == 0x9296 || s->pc == 0x9418) {
        static unsigned hit_counts[3] = {0, 0, 0};
        int idx = (s->pc == 0x8815) ? 0 : (s->pc == 0x9296) ? 1 : 2;
        hit_counts[idx]++;
        if (hit_counts[idx] <= 20 || hit_counts[idx] % 100 == 0) {
            fprintf(stderr,
                    "[c54x] DISPATCH-CALLER hit=%u pc=0x%04x "
                    "A=0x%010llx AR0=0x%04x AR1=0x%04x AR2=0x%04x "
                    "data[0x0828]=0x%04x data[0x0829]=0x%04x "
                    "data[0x083c]=0x%04x data[0x083d]=0x%04x insn=%u\n",
                    hit_counts[idx], s->pc,
                    (unsigned long long)(s->a & 0xFFFFFFFFFFULL),
                    s->ar[0], s->ar[1], s->ar[2],
                    s->data[0x0828], s->data[0x0829],
                    s->data[0x083c], s->data[0x083d],
                    s->insn_count);
        }
    }

    /* AR7-INIT-CHAIN / MVMD-AR7-BRC / RPTB-ARMED probe: value of AR7 at the
     * MVMD AR7,BRC in PC=0x8208, the last 16 writes that produced it, and the
     * BRC state after the RPTBD setup. */
    {
        static uint16_t prev_ar7 = 0xFFFF;
        static struct {
            uint16_t pc;
            uint16_t old_val;
            uint16_t new_val;
            uint64_t insn;
            uint8_t  xpc;
        } ar7_history[16] = {{0}};
        static unsigned ar7_hist_idx = 0;

        if (s->ar[7] != prev_ar7) {
            ar7_history[ar7_hist_idx & 15].pc = s->pc;
            ar7_history[ar7_hist_idx & 15].old_val = prev_ar7;
            ar7_history[ar7_hist_idx & 15].new_val = s->ar[7];
            ar7_history[ar7_hist_idx & 15].insn = s->insn_count;
            ar7_history[ar7_hist_idx & 15].xpc = s->xpc & 0x3;
            ar7_hist_idx++;
            prev_ar7 = s->ar[7];
        }

        /* (b) Full snapshot on each hit of PC=0x8208 (MVMD AR7, BRC) */
        if (s->pc == 0x8208) {
            static unsigned mvmd_hits = 0;
            mvmd_hits++;
            if (mvmd_hits <= 20 || (mvmd_hits % 100) == 0) {
                if (calypso_debug_enabled("MVMD-AR7-BRC")) fprintf(stderr,
                        "[c54x] MVMD-AR7-BRC #%u AR7=0x%04x BRC_before=0x%04x "
                        "AR0=0x%04x AR1=0x%04x AR2=0x%04x AR6=0x%04x DP=%d insn=%u\n",
                        mvmd_hits, s->ar[7], s->brc,
                        s->ar[0], s->ar[1], s->ar[2], s->ar[6], dp(s),
                        s->insn_count);
                int n_hist = ar7_hist_idx < 16 ? ar7_hist_idx : 16;
                for (int i = 0; i < n_hist; i++) {
                    int slot = (ar7_hist_idx - n_hist + i) & 15;
                    if (calypso_debug_enabled("AR7-HIST")) fprintf(stderr,
                            "[c54x]   AR7-HIST[%d] pc=XPC%u:0x%04x 0x%04x->0x%04x insn=%llu\n",
                            i, ar7_history[slot].xpc, ar7_history[slot].pc,
                            ar7_history[slot].old_val, ar7_history[slot].new_val,
                            (unsigned long long)ar7_history[slot].insn);
                }
            }
        }

        /* (c) RPTB state after setup (PC=0x820c = delay slot after RPTBD) */
        if (s->pc == 0x820c) {
            static unsigned rptb_hits = 0;
            rptb_hits++;
            if (rptb_hits <= 20 || (rptb_hits % 100) == 0) {
                if (calypso_debug_enabled("RPTB-ARMED")) fprintf(stderr,
                        "[c54x] RPTB-ARMED #%u BRC=0x%04x RSA=0x%04x REA=0x%04x "
                        "ST1=0x%04x (INTM=%d) insn=%u\n",
                        rptb_hits, s->brc, s->rsa, s->rea, s->st1,
                        (s->st1 >> 11) & 1, s->insn_count);
            }
        }
    }

    /* INT3-BLOCKED probe. Samples 1/1000 of the context (PC/ST1/BRC/XPC)
     * while INT3 is pending and INTM=1. Discriminates (a) an opcode that sets
     * INTM without clearing it (POPM variant), (b) a long non-interruptible
     * RPTB (BRC > 0 throughout), (c) a raw STM ST1 / MVDM ST1. */
    {
        static uint64_t blocked_count = 0;
        static uint16_t sample_pcs[32] = {0};
        static uint16_t sample_st1s[32] = {0};
        static uint16_t sample_brcs[32] = {0};
        static uint8_t  sample_xpcs[32] = {0};
        static unsigned sample_idx = 0;
        static unsigned next_blocked_dump = 100000000u;

        bool int3_pending_now = (s->ifr & 0x08) != 0;
        bool intm_set_now = ((s->st1 >> 11) & 1) != 0;

        if (int3_pending_now && intm_set_now) {
            blocked_count++;
            if ((blocked_count % 1000) == 0) {
                sample_pcs[sample_idx & 31] = s->pc;
                sample_st1s[sample_idx & 31] = s->st1;
                sample_brcs[sample_idx & 31] = s->brc;
                sample_xpcs[sample_idx & 31] = s->xpc & 0x3;
                sample_idx++;
            }
        }

        if (s->insn_count >= next_blocked_dump) {
            next_blocked_dump += 100000000u;
            if (calypso_debug_enabled("INT3-BLOCKED")) fprintf(stderr,
                    "[c54x] INT3-BLOCKED insn=%u blocked_total=%llu blocked_samples=%u\n",
                    s->insn_count,
                    (unsigned long long)blocked_count,
                    sample_idx);
            int n = sample_idx < 32 ? sample_idx : 32;
            for (int i = 0; i < n; i++) {
                int slot = (sample_idx - n + i) & 31;
                if (calypso_debug_enabled("INT3-BLOCKED-SAMPLE")) fprintf(stderr,
                        "[c54x] INT3-BLOCKED-SAMPLE pc=XPC%u:0x%04x st1=0x%04x brc=0x%04x\n",
                        sample_xpcs[slot], sample_pcs[slot],
                        sample_st1s[slot], sample_brcs[slot]);
            }
        }
    }

    /* IRQ-FRAME-HEALTH probe. INT3 is the frame interrupt (IMR bit 3,
     * vector 19, address 0xFFCC); this counts fire/serviced/missed and the
     * service latency. Discriminates a mis-vectored ISR (serviced < fire), a
     * dead TPU/TSP source (fire == 0) and compute that is too slow
     * (missed > 0). */
    {
        static uint64_t int3_fire_count = 0;
        static uint64_t int3_serviced_count = 0;
        static uint64_t int3_missed_count = 0;
        static uint64_t last_int3_fire_insn = 0;
        static uint64_t last_int3_service_insn = 0;
        static uint64_t total_service_latency_insn = 0;
        static bool int3_pending_prev = false;
        static unsigned next_irq_dump = 200000000u;

        bool int3_now_pending = (s->ifr & 0x08) != 0;
        bool int3_just_fired = int3_now_pending && !int3_pending_prev;
        /* ISR enter approximation : INT3 cleared from IFR while INTM=0 */
        bool int3_just_serviced = !int3_now_pending && int3_pending_prev &&
                                  ((s->st1 >> 11) & 1) == 0;

        if (int3_just_fired) {
            int3_fire_count++;
            if (int3_pending_prev) {
                int3_missed_count++;
            }
            last_int3_fire_insn = s->insn_count;
        }
        if (int3_just_serviced) {
            int3_serviced_count++;
            if (last_int3_fire_insn > last_int3_service_insn) {
                total_service_latency_insn += (s->insn_count - last_int3_fire_insn);
            }
            last_int3_service_insn = s->insn_count;
        }
        int3_pending_prev = int3_now_pending;

        if (s->insn_count >= next_irq_dump) {
            next_irq_dump += 200000000u;
            uint64_t avg_latency = int3_serviced_count > 0
                ? total_service_latency_insn / int3_serviced_count : 0;
            double service_ratio = int3_fire_count > 0
                ? (double)int3_serviced_count / int3_fire_count : 0.0;
            if (calypso_debug_enabled("IRQ-FRAME-HEALTH")) fprintf(stderr,
                    "[c54x] IRQ-FRAME-HEALTH insn=%u int3_fire=%llu int3_serviced=%llu "
                    "int3_missed=%llu avg_latency_insn=%llu service_ratio=%.2f\n",
                    s->insn_count,
                    (unsigned long long)int3_fire_count,
                    (unsigned long long)int3_serviced_count,
                    (unsigned long long)int3_missed_count,
                    (unsigned long long)avg_latency,
                    service_ratio);
        }
    }

    /* EXIT-COMPUTE / IRQ-DURING-COMPUTE probe. The DSP spins under XPC=2 in
     * the hot zone 0xdf80..0xdfc0 (CCCH demod MAC loop). exits_count together
     * with irq_pending_in_compute discriminate three cases: compute never
     * exits, the IRQ never fires (missing TPU/TSP source), or the IRQ fires
     * but is never serviced (INTM stuck or mis-vectored ISR). */
    {
        static uint16_t last_pc_sample = 0;
        static uint8_t  last_xpc_sample = 0;
        static unsigned exits_count = 0;
        static unsigned irqs_pending_during_compute = 0;
        static unsigned int3_pending_during_compute = 0;
        static uint64_t insns_in_compute = 0;
        static unsigned next_compute_dump = 200000000u;

        bool in_compute_now = ((s->xpc & 0x3) == 2 &&
                               s->pc >= 0xdf80 && s->pc <= 0xdfc0);
        bool was_in_compute = (last_xpc_sample == 2 &&
                               last_pc_sample >= 0xdf80 && last_pc_sample <= 0xdfc0);

        if (was_in_compute && !in_compute_now) {
            exits_count++;
            if (exits_count <= 30 || exits_count % 200 == 0) {
                fprintf(stderr,
                        "[c54x] EXIT-COMPUTE #%u from=XPC%u:0x%04x to=XPC%u:0x%04x "
                        "A=0x%010llx IFR=0x%04x INTM=%d insn=%u\n",
                        exits_count,
                        last_xpc_sample, last_pc_sample,
                        s->xpc & 0x3, s->pc,
                        (unsigned long long)(s->a & 0xFFFFFFFFFFULL),
                        s->ifr, (s->st1 >> 11) & 1, s->insn_count);
            }
        }

        if (in_compute_now) {
            insns_in_compute++;
            if (s->ifr != 0) {
                irqs_pending_during_compute++;
                if (s->ifr & 0x08) int3_pending_during_compute++;
            }
        }

        if (s->insn_count >= next_compute_dump) {
            next_compute_dump += 200000000u;
            fprintf(stderr,
                    "[c54x] COMPUTE-STATS insn=%u in_compute=%llu exits=%u "
                    "irq_pending_in_compute=%u int3_pending_in_compute=%u\n",
                    s->insn_count,
                    (unsigned long long)insns_in_compute,
                    exits_count,
                    irqs_pending_during_compute,
                    int3_pending_during_compute);
        }

        last_pc_sample = s->pc;
        last_xpc_sample = s->xpc & 0x3;
    }

    /* DISPATCH-ENTRY probe. The dispatcher caller branches to
     * 0x8810 + task_id*3, where each entry is { 0xf4e4 (FRET or padding),
     * 0xf074 (B opcode), <target> }. Probes the PCs that start an entry
     * (0x8810 + N*3), so task_id = (PC - 0x8810) / 3, and reads the target at
     * PC+2. */
    if (s->pc >= 0x8810 && s->pc < 0x8900 && ((s->pc - 0x8810) % 3) == 0) {
        static unsigned entry_hits = 0;
        entry_hits++;
        if (entry_hits <= 50 || entry_hits % 200 == 0) {
            uint16_t entry_idx = (s->pc - 0x8810) / 3;
            uint16_t header = s->prog[s->pc];     /* normally 0xf4e4 */
            uint16_t branch = s->prog[s->pc + 1]; /* normally 0xf074 */
            uint16_t target = s->prog[s->pc + 2];
            if (calypso_debug_enabled("DISPATCH-ENTRY")) fprintf(stderr,
                    "[c54x] DISPATCH-ENTRY #%u pc=0x%04x entry_idx=%u "
                    "header=0x%04x branch=0x%04x target=0x%04x "
                    "A=0x%010llx insn=%u\n",
                    entry_hits, s->pc, entry_idx,
                    header, branch, target,
                    (unsigned long long)(s->a & 0xFFFFFFFFFFULL),
                    s->insn_count);
        }
    }

    /* Periodic DSP state dump (every 500M insns, starting at 500M).
     * Captures: state regs, hot-zone disasm (0xa2c0..0xa2d0 + 0xb8e0..0xb910),
     * vector table at current PMST IPTR base, hot-PC opcodes, SP history. */
    {
        static unsigned next_dump = 500000000u;
        if (s->insn_count >= next_dump) {
            next_dump += 500000000u;
            uint16_t iptr = (s->pmst >> PMST_IPTR_SHIFT) & 0x1FF;
            uint16_t vbase = iptr * 0x80;
            C54_LOG("STATE-DUMP insn=%u PC=0x%04x ST0=0x%04x ST1=0x%04x INTM=%d IMR=0x%04x IFR=0x%04x XPC=%d PMST=0x%04x SP=0x%04x AR1=0x%04x AR2=0x%04x BRC=%d",
                    s->insn_count, s->pc, s->st0, s->st1,
                    !!(s->st1 & ST1_INTM),
                    s->imr, s->ifr, s->xpc, s->pmst, s->sp,
                    s->ar[1], s->ar[2], s->brc);
            C54_LOG("STATE-DUMP prog[0xa2c0..0xa2d0]: %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x",
                    s->prog[0xa2c0], s->prog[0xa2c1], s->prog[0xa2c2], s->prog[0xa2c3],
                    s->prog[0xa2c4], s->prog[0xa2c5], s->prog[0xa2c6], s->prog[0xa2c7],
                    s->prog[0xa2c8], s->prog[0xa2c9], s->prog[0xa2ca], s->prog[0xa2cb],
                    s->prog[0xa2cc], s->prog[0xa2cd], s->prog[0xa2ce], s->prog[0xa2cf],
                    s->prog[0xa2d0]);
            /* Hot zone b8e9..b906 (vec1 handler). */
            C54_LOG("STATE-DUMP prog[0xb8e0..0xb910]: %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x",
                    s->prog[0xb8e0], s->prog[0xb8e1], s->prog[0xb8e2], s->prog[0xb8e3],
                    s->prog[0xb8e4], s->prog[0xb8e5], s->prog[0xb8e6], s->prog[0xb8e7],
                    s->prog[0xb8e8], s->prog[0xb8e9], s->prog[0xb8ea], s->prog[0xb8eb],
                    s->prog[0xb8ec], s->prog[0xb8ed], s->prog[0xb8ee], s->prog[0xb8ef],
                    s->prog[0xb8f0], s->prog[0xb8f1], s->prog[0xb8f2], s->prog[0xb8f3],
                    s->prog[0xb8f4], s->prog[0xb8f5], s->prog[0xb8f6], s->prog[0xb8f7],
                    s->prog[0xb8f8], s->prog[0xb8f9], s->prog[0xb8fa], s->prog[0xb8fb],
                    s->prog[0xb8fc], s->prog[0xb8fd], s->prog[0xb8fe], s->prog[0xb8ff],
                    s->prog[0xb900]);
            C54_LOG("STATE-DUMP prog[0xb900..0xb920]: %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x",
                    s->prog[0xb900], s->prog[0xb901], s->prog[0xb902], s->prog[0xb903],
                    s->prog[0xb904], s->prog[0xb905], s->prog[0xb906], s->prog[0xb907],
                    s->prog[0xb908], s->prog[0xb909], s->prog[0xb90a], s->prog[0xb90b],
                    s->prog[0xb90c], s->prog[0xb90d], s->prog[0xb90e], s->prog[0xb90f],
                    s->prog[0xb910], s->prog[0xb911], s->prog[0xb912], s->prog[0xb913],
                    s->prog[0xb914], s->prog[0xb915], s->prog[0xb916], s->prog[0xb917],
                    s->prog[0xb918], s->prog[0xb919], s->prog[0xb91a], s->prog[0xb91b],
                    s->prog[0xb91c], s->prog[0xb91d], s->prog[0xb91e], s->prog[0xb91f],
                    s->prog[0xb920]);
            C54_LOG("STATE-DUMP vbase=0x%04x prog[vbase..vbase+0x18]: %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x",
                    vbase,
                    s->prog[vbase+0x00], s->prog[vbase+0x01], s->prog[vbase+0x02], s->prog[vbase+0x03],
                    s->prog[vbase+0x04], s->prog[vbase+0x05], s->prog[vbase+0x06], s->prog[vbase+0x07],
                    s->prog[vbase+0x08], s->prog[vbase+0x09], s->prog[vbase+0x0a], s->prog[vbase+0x0b],
                    s->prog[vbase+0x0c], s->prog[vbase+0x0d], s->prog[vbase+0x0e], s->prog[vbase+0x0f],
                    s->prog[vbase+0x10], s->prog[vbase+0x11], s->prog[vbase+0x12], s->prog[vbase+0x13],
                    s->prog[vbase+0x14], s->prog[vbase+0x15], s->prog[vbase+0x16], s->prog[vbase+0x17],
                    s->prog[vbase+0x18]);
            /* Hot-PC opcode dump for known correlator/handler sites */
            C54_LOG("STATE-DUMP HOT-OPS: 0x8d33=%04x 0x8eb9=%04x 0x8f51=%04x 0xa2c7=%04x 0xa2c8=%04x 0xb8e9=%04x 0xb8eb=%04x 0xb8f4=%04x 0xb8f5=%04x 0xb906=%04x",
                    s->prog[0x8d33], s->prog[0x8eb9], s->prog[0x8f51],
                    s->prog[0xa2c7], s->prog[0xa2c8],
                    s->prog[0xb8e9], s->prog[0xb8eb], s->prog[0xb8f4],
                    s->prog[0xb8f5], s->prog[0xb906]);
            /* DARAM 0x066F..0x0682 wait-loop disassembly: tells a B-self
             * (f073 066f) from IDLE n (f7e1/f7e2/f7e3) or a poll-and-branch.
             * An IDLE here would put the bug in the emulator's IDLE
             * handler. */
            C54_LOG("STATE-DUMP prog[0x0660..0x0690]: %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x",
                    s->prog[0x0660], s->prog[0x0661], s->prog[0x0662], s->prog[0x0663],
                    s->prog[0x0664], s->prog[0x0665], s->prog[0x0666], s->prog[0x0667],
                    s->prog[0x0668], s->prog[0x0669], s->prog[0x066a], s->prog[0x066b],
                    s->prog[0x066c], s->prog[0x066d], s->prog[0x066e], s->prog[0x066f],
                    s->prog[0x0670], s->prog[0x0671], s->prog[0x0672], s->prog[0x0673],
                    s->prog[0x0674], s->prog[0x0675], s->prog[0x0676], s->prog[0x0677],
                    s->prog[0x0678], s->prog[0x0679], s->prog[0x067a], s->prog[0x067b],
                    s->prog[0x067c], s->prog[0x067d], s->prog[0x067e], s->prog[0x067f],
                    s->prog[0x0680]);
            /* Same range but data[] view in case OVLY=1 routes fetches
             * to data array (different memory than prog). */
            C54_LOG("STATE-DUMP data[0x0660..0x0680]: %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x",
                    s->data[0x0660], s->data[0x0661], s->data[0x0662], s->data[0x0663],
                    s->data[0x0664], s->data[0x0665], s->data[0x0666], s->data[0x0667],
                    s->data[0x0668], s->data[0x0669], s->data[0x066a], s->data[0x066b],
                    s->data[0x066c], s->data[0x066d], s->data[0x066e], s->data[0x066f],
                    s->data[0x0670], s->data[0x0671], s->data[0x0672], s->data[0x0673],
                    s->data[0x0674], s->data[0x0675], s->data[0x0676], s->data[0x0677],
                    s->data[0x0678], s->data[0x0679], s->data[0x067a], s->data[0x067b],
                    s->data[0x067c], s->data[0x067d], s->data[0x067e], s->data[0x067f],
                    s->data[0x0680]);
            /* IRQ entry handler at PC=0x1854 (last 0→1 transition) */
            C54_LOG("STATE-DUMP prog[0x1850..0x1860]: %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x",
                    s->prog[0x1850], s->prog[0x1851], s->prog[0x1852], s->prog[0x1853],
                    s->prog[0x1854], s->prog[0x1855], s->prog[0x1856], s->prog[0x1857],
                    s->prog[0x1858], s->prog[0x1859], s->prog[0x185a], s->prog[0x185b],
                    s->prog[0x185c], s->prog[0x185d], s->prog[0x185e], s->prog[0x185f],
                    s->prog[0x1860]);
            /* SP history ring (last 32 sampled at 1M-insn intervals) */
            {
                char buf[2048]; int o = 0;
                int start = (sp_ring_idx >= 32) ? (sp_ring_idx - 32) : 0;
                for (unsigned i = start; i < sp_ring_idx; i++) {
                    int idx = i & 63;
                    o += snprintf(buf+o, sizeof(buf)-o,
                                  "[%u:PC=%04x SP=%04x] ",
                                  sp_ring[idx].insn,
                                  sp_ring[idx].pc,
                                  sp_ring[idx].sp);
                    if (o > (int)sizeof(buf) - 64) break;
                }
                C54_LOG("STATE-DUMP SP-RING (last %d): %s",
                        (int)(sp_ring_idx >= 32 ? 32 : sp_ring_idx), buf);
            }
        }
    }

    while (executed < n_insns && s->running && !s->idle) {
        /* Cold-reset redirect to the firmware entry point, off by default.
         * The PROM dump does not contain the Calypso silicon mask-ROM, which
         * on real hardware runs at reset, sets SP=0x5AC8 plus the MMRs and
         * jumps to the firmware entry PROM0[0x7120] (prog[0x7120]=0x7718
         * STM #lk,SP, prog[0x7121]=0x5ac8). The SP==0x1100 gate restricts this
         * to the cold reset (the silicon reset value): once 0x7120 has set
         * SP=0x5AC8 the condition no longer holds, so the later sequential
         * firmware walks through 0xff80 are not hijacked (see SOFT-RESET-TRIG
         * below, insn > 100k).
         *
         * @BEQUILLE - REDIR_LEGACY  (CALYPSO_REDIR_LEGACY, EXISTS, default OFF)
         *   masks   : the real reset vector 0xff80 -> 0xb410, diverted to stand in
         *             for the TI mask-ROM missing from the dump.
         *   remove  : as soon as the real reset handler 0xb410 sets SP=0x5AC8 itself
         *             (STM #0x5AC8,SP decoded correctly) and the boot runs through
         *             without over-pop - that is the bug to trace, not to bypass.
         *   note    : owns INITTAB and REDIR7000; mutually exclusive with
         *             MASKROM_INIT.
         */
        static int redir_legacy = -1;
        if (redir_legacy < 0) redir_legacy = calypso_gate("CALYPSO_REDIR_LEGACY", 0);
        if (redir_legacy && s->pc == 0xFF80 && s->sp == 0x1100) {
            static int redirect_log;
            /* @BEQUILLE - REDIR7000  (CALYPSO_REDIR7000, EXISTS, default OFF)
             *   masks   : the init of the BACC-A tables (d[0x4c5b]/d[0x3fe1]) that entry
             *             point 0x7120 assumes is already done; redirects the reset to
             *             0x7000, which runs the full init (tables plus A), so BACC A
             *             reaches the real firmware entry instead of A=0 -> boot stub.
             *   remove  : same condition as INITTAB (table populated by the firmware path).
             *   note    : nested inside REDIR_LEGACY, and overridden by INITTAB (else if).
             */
            static int redir7000 = -1;
            if (redir7000 < 0) redir7000 = calypso_gate("CALYPSO_REDIR7000", 0);
            if (redirect_log < 3) {
                C54_LOG("SILICON-BOOT-REDIRECT PC=0xFF80 SP=0x1100 → 0x%04x%s",
                        redir7000 ? 0x7000 : 0x7120,
                        redir7000 ? " (REDIR7000: SP=0x5AC8 + init complète A-tables)" : "");
                redirect_log++;
            }
            /* @BEQUILLE - INITTAB  (CALYPSO_INITTAB, EXISTS, default OFF)
             *   masks   : the absence of the TI mask-ROM that populates the task handler
             *             table 0x4c24-0x4c5d at reset; without it 0x7120 does
             *             BACC d[0x4c5b] = null. Sets SP, pushes 0x7120 as return address
             *             and jumps to the table init 0xc704, which fills
             *             data[0x4c24-0x4c5d] and returns into the normal boot.
             *   remove  : as soon as the table is populated by a firmware path (0xc704
             *             reached natively after the 0x8869 clear) - INSTALL-TRACE probe
             *             shows d[4c5c] != 0.
             *   note    : never evaluated unless CALYPSO_REDIR_LEGACY is set.
             */
            static int inittab = -1;
            if (inittab < 0) inittab = calypso_gate("CALYPSO_INITTAB", 0);
            if (inittab) {
                s->sp = 0x5AC8;
                s->sp--; s->data[s->sp] = 0x7120;   /* return = normal boot */
                s->pc = 0xc704;                       /* run table init -> RET 0x7120 */
            } else if (redir7000) { s->sp = 0x5AC8; s->pc = 0x7000; }
            else s->pc = 0x7120;
        }
        /* Mask-ROM table init, mutually exclusive with redir_legacy (which
         * already handles 0xff80). */
        if (!redir_legacy && s->pc == 0xFF80 && s->sp == 0x1100) {
            /* @BEQUILLE - MASKROM_INIT  (CALYPSO_MASKROM_INIT, EXISTS, default OFF)
             *   masks   : same as INITTAB - the missing TI mask-ROM that sets SP=0x5AC8
             *             and populates the task handler table (0x4c04-0x4c5d) at cold
             *             reset. Without it the table is zero after the RPTB clear at
             *             0x8869, entry 0x7120 does BACC d[0x4c5b] = null, and
             *             0x7025/0xd247/0xc8e9/corr never run. Pushes 0xb410 as return
             *             address and jumps to 0xc704 (absolute addressing, so it works
             *             with AR/DP still uninitialised).
             *   remove  : same condition as INITTAB (table populated by a firmware path).
             */
            static int mrti = -1;
            if (mrti < 0) mrti = calypso_gate("CALYPSO_MASKROM_INIT", 0);   /* opt-in: forcing the boot op derails from a cold state; kept for A/B */
            if (mrti) {
                static int mrti_log = 0;
                if (mrti_log < 2) { mrti_log++;
                    fprintf(stderr, "[c54x] MASK-ROM-INIT: cold-reset SP=0x5AC8, run table-init 0xc704 (RET 0xb410) insn=%u\n", s->insn_count); }
                s->sp = 0x5AC8;
                s->sp--; s->data[s->sp] = 0x7120;   /* return = firmware entry (BACC d[0x4c5b] populated) */
                s->pc = 0xc704;                       /* fill handler table -> RET 0x7120 -> operational */
            }
        }
        if (s->pc == 0xb3e4) {
            /* @BEQUILLE - D247  (CALYPSO_D247, EXISTS, default OFF)
             *   masks   : the absence of the TI mask-ROM bootstrap that, on silicon,
             *             calls the operational subsystem 0xd247 (install handler table
             *             0xc704, TDMA slots 0xc867, vectors). In QEMU 0xd247 has one
             *             native caller, PROM0 0x7102, in a block never reached on a cold
             *             boot; without it d[0x4c5c]=0 and d[0x3f6b]=0xd294 (RET no-op),
             *             so FB acquisition is a no-op and d[3f70] never reaches 2. Pushes
             *             the return address and diverts the PC, once, at the boot-init
             *             terminal 0xb3e4 (SP=0x5AC8, cells seeded; RET @0xd25f comes
             *             back to 0xb3e4).
             *   remove  : as soon as the native calling block 0x70ce-0x7106 is reached
             *             (D247-TRACE shows site 0x7102 non-zero), or as soon as the table
             *             d[4c5c] is populated by the firmware path.
             */
            static int _d247 = -1;
            if (_d247 < 0) _d247 = calypso_gate("CALYPSO_D247", 0);   /* opt-in OFF: the bootstrap does reach 0xc6a5 (coefficient init) but loops on an empty source. Kept for A/B */
            static int _d247_done = 0;
            if (_d247 && !_d247_done) {
                _d247_done = 1;
                fprintf(stderr, "[c54x] BOOTSTRAP-D247 @0xb3e4 : run 0xd247 (install table+slots+vec) insn=%u SP=0x%04x\n", s->insn_count, s->sp);
                s->sp--; s->data[s->sp] = 0xb3e4;   /* return = boot-init terminal */
                s->pc = 0xd247;
            }
        }
        if (s->pc == 0x886a && s->data[0x4c5c] == 0) {
            /* @BEQUILLE - REPOPULATE  (CALYPSO_REPOPULATE, EXISTS, default OFF)
             *   masks   : the RPTB memset 0x8866-0x886a wipes the handler table after it
             *             was populated (insn ~19793, well after insn 92) and the firmware
             *             never calls 0xc704 again; the real firmware order is
             *             clear -> populate. At the clear's RET (0x886a), when the table is
             *             empty, the PC is diverted to 0xc704, whose own RET @0xc826 pops
             *             the same return address - so it self-heals on every clear.
             *   remove  : as soon as 0xc704 is reached AFTER the clear by the native flow
             *             (D247-TRACE: d[4c41]/d[4c46] non-zero at end of boot).
             */
            static int mrti2 = -1;
            if (mrti2 < 0) mrti2 = calypso_gate("CALYPSO_REPOPULATE", 0);   /* opt-in OFF: populating 0x4c5c does NOT unblock FB acquisition (measured: fb0_att stays 0). Kept for A/B */
            if (mrti2) {
                static int rlg = 0;
                if (rlg < 3) { rlg++;
                    fprintf(stderr, "[c54x] TABLE-REPOPULATE @0x886a (clear a wipe la table) -> run 0xc704 insn=%u\n", s->insn_count); }
                s->pc = 0xc704;
            }
        }
        /* D247-TRACE (read-only). 0xd247 has exactly one native caller, PROM0
         * 0x7102, inside the operational block 0x70ce-0x7106. These probes
         * force nothing:
         *   (a) is 0x7102 reached natively, i.e. does the calling block run?
         *   (b) does 0xd247 fire, and what is the table state before and after
         *       its RET (@0xd25f)?
         *   (c) does the clear at 0x87ff run (its callers are in PROM1 via
         *       FCALL, not PROM0), and before or after 0xd247 - does it wipe
         *       the work of 0xc704?
         * d[4c41] and d[4c46] are two table slots read by the dispatcher
         * 0xc8e9 (CALA), so they report install success directly. On by
         * default, 20 lines per site. CALYPSO_D247_TRACE_OFF=1 disables; the
         * gate tests the VALUE with atoi, not mere presence. */
        {
            static int _d247t = -1;
            if (_d247t < 0) { const char *_e = getenv("CALYPSO_D247_TRACE_OFF"); _d247t = (_e && atoi(_e)) ? 0 : 1; }
            if (_d247t) {
                static unsigned _n7102=0, _nd247=0, _nd25f=0, _n87ff=0;
                if (s->pc == 0x7102 && _n7102++ < 20)
                    fprintf(stderr, "[c54x] D247-TRACE #%u @0x7102 (caller reel de 0xd247) "
                            "d[3f70]=0x%04x SP=0x%04x insn=%u\n",
                            _n7102, s->data[0x3f70], s->sp, s->insn_count);
                if (s->pc == 0xd247 && _nd247++ < 20)
                    fprintf(stderr, "[c54x] D247-TRACE #%u ENTRY 0xd247 d[3f70]=0x%04x SP=0x%04x "
                            "d[4c41]=0x%04x d[4c46]=0x%04x d[4c5c]=0x%04x (avant install) insn=%u\n",
                            _nd247, s->data[0x3f70], s->sp, s->data[0x4c41], s->data[0x4c46],
                            s->data[0x4c5c], s->insn_count);
                if (s->pc == 0xd25f && _nd25f++ < 20)
                    fprintf(stderr, "[c54x] D247-TRACE #%u RET 0xd25f d[3f70]=0x%04x "
                            "d[4c41]=0x%04x d[4c46]=0x%04x d[4c5c]=0x%04x (apres install) insn=%u\n",
                            _nd25f, s->data[0x3f70], s->data[0x4c41], s->data[0x4c46],
                            s->data[0x4c5c], s->insn_count);
                if (s->pc == 0x87ff && _n87ff++ < 20)
                    fprintf(stderr, "[c54x] D247-TRACE #%u CLEAR-ENTRY 0x87ff d[4c5c]=0x%04x "
                            "(table AVANT clear) SP=0x%04x insn=%u\n",
                            _n87ff, s->data[0x4c5c], s->sp, s->insn_count);
            }
        }
        /* CYCLE-TRACE (read-only). Cycle 1 (bit 4 armed, CALYPSO_SEED_52FD)
         * completes cleanly through a51c -> a526 -> a529 -> a534 -> a537 ->
         * a53c -> a53f -> a541 -> a544 -> a549 -> a582 -> b522 -> 011e; from
         * cycle 2 on the flow falls into a 0x71d7 <-> 0x71db loop (146
         * iterations observed) instead of repeating that path. This traces
         * EVERY pass, not just the first, to show where the two cycles
         * diverge, and logs entry into the wrapper 0x71d3 (before the loop)
         * with d[3f92], d[5a00], d[435b] (IMR shadow) and the real IMR.
         * 80 lines per site. */
        {
            static int _cyc = -1;
            if (_cyc < 0) { const char *_e = getenv("CALYPSO_D247_TRACE_OFF"); _cyc = (_e && atoi(_e)) ? 0 : 1; }
            if (_cyc) {
                static unsigned n51c=0,n537=0,n53c=0,n53f=0,n544=0,n549=0,n71d3=0;
                unsigned _cap = 20000;
                if (s->pc==0xa51c && n51c++<80)
                    fprintf(stderr, "[c54x] CYCLE-TRACE #%u ENTRY-a51c d[3f92]=0x%04x d[5a00]=0x%04x "
                            "d[435b]=0x%04x IMR=0x%04x insn=%u\n", n51c, s->data[0x3f92], s->data[0x5a00],
                            s->data[0x435b], s->imr, s->insn_count);
                if (s->pc==0xa537 && n537++<80)
                    fprintf(stderr, "[c54x] CYCLE-TRACE #%u a537(CMPM d5a00,0x88) TC=%d d[5a00]=0x%04x insn=%u\n",
                            n537, !!(s->st0 & ST0_TC), s->data[0x5a00], s->insn_count);
                if (s->pc==0xa53c && n53c++<_cap) {
                    /* The effective address is computed the way the BITF does,
                     * and printed. Hard-coding data[0x0810] is wrong: the BITF
                     * reads *AR1(0x0010), and AR1 alternates between 0x0800 and
                     * 0x0814, so the cell actually tested is 0x0810 or 0x0824. */
                    uint16_t _cs = (uint16_t)(s->ar[1] + 0x0010);
                    fprintf(stderr, "[c54x] CYCLE-TRACE #%u a53c(BITF AR1+10,0x8000) AR1=0x%04x d[3f92]=0x%04x "
                            "data[0x%04x]=0x%04x(B_TASK_ABORT=%d) fn=%u insn=%u\n",
                            n53c, s->ar[1], s->data[0x3f92], _cs, s->data[_cs],
                            !!(s->data[_cs] & 0x8000), s->data[0x0585], s->insn_count);
                }
                if (s->pc==0xa53f && n53f++<_cap)
                    fprintf(stderr, "[c54x] CYCLE-TRACE #%u a53f(BC a575 if NTC) TC=%d insn=%u\n",
                            n53f, !!(s->st0 & ST0_TC), s->insn_count);
                if (s->pc==0xa544 && n544++<80)
                    fprintf(stderr, "[c54x] CYCLE-TRACE #%u a544(BC a549 if NTC, bit0 d09bc) TC=%d d[09bc]=0x%04x insn=%u\n",
                            n544, !!(s->st0 & ST0_TC), s->data[0x09bc], s->insn_count);
                if (s->pc==0xa549 && n549++<80)
                    fprintf(stderr, "[c54x] CYCLE-TRACE #%u CONVERGE-a549 insn=%u\n", n549, s->insn_count);
                if (s->pc==0x71d3 && n71d3++<80)
                    fprintf(stderr, "[c54x] CYCLE-TRACE #%u ENTRY-71d3(wrapper) d[3f92]=0x%04x d[5a00]=0x%04x "
                            "d[435b]=0x%04x IMR=0x%04x SP=0x%04x insn=%u\n",
                            n71d3, s->data[0x3f92], s->data[0x5a00], s->data[0x435b], s->imr, s->sp, s->insn_count);
            }
        }
        /* CLUSTERB-8D21 (read-only). CALLD target inside the correlator range
         * (0x8d00-0x9000), called only by the task-type 4/6 handlers
         * (Cluster B). Static disassembly shows two nested RPTB/RPTBD, T=0x18
         * (24, tap-count shaped) and circular indirect MAR addressing - a
         * genuine signal-processing signature, unlike the audio cluster
         * c1fa/c27b and the bitmask helpers 8f7f/8f9d. 30 lines. */
        {
            static int _c8d21 = -1;
            if (_c8d21 < 0) { const char *_e = getenv("CALYPSO_D247_TRACE_OFF"); _c8d21 = (_e && atoi(_e)) ? 0 : 1; }
            if (_c8d21 && s->pc == 0x8d21) {
                static unsigned _n8d21 = 0;
                if (_n8d21++ < 30)
                    fprintf(stderr, "[c54x] CLUSTERB-8D21 #%u AR2=0x%04x AR3=0x%04x AR5=0x%04x "
                            "BK=0x%04x A=0x%06llx B=0x%06llx insn=%u\n",
                            _n8d21, s->ar[2], s->ar[3], s->ar[5], s->bk,
                            (unsigned long long)(s->a & 0xFFFFFFULL),
                            (unsigned long long)(s->b & 0xFFFFFFULL), s->insn_count);
            }
        }
        /* BITF-000B-HIT (read-only). The two BITF on data[0x000b] in the
         * background dispatcher (0xdeb6: BITF *(0x000b),0x4000, bit 14;
         * 0xdec2: BITF *(0x000b),0x2000, bit 13). Tests whether the go-live
         * cycle that loops without ever reaching the correlator is waiting on
         * a counter or flag in 0x000b that only real TPU sequencer timing
         * would advance (the 11 tpu_enq_at(0) of l1s_rx_win_ctrl are not
         * modelled, see calypso_tpu.c). Logs data[0x000b] BEFORE execution,
         * i.e. what the BITF is about to test, at both PCs. Complements
         * WATCH-000B-WR, which says whether the cell is written at all.
         * 40 lines each. */
        {
            static unsigned _nb6 = 0, _nc2 = 0;
            if (s->pc == 0xdeb6 && _nb6++ < 40)
                fprintf(stderr, "[c54x] BITF-000B-HIT #%u PC=0xdeb6 mask=0x4000 "
                        "data[0x000b]=0x%04x TC-will-be=%d insn=%u\n",
                        _nb6, s->data[0x000b], (s->data[0x000b] & 0x4000) != 0, s->insn_count);
            if (s->pc == 0xdec2 && _nc2++ < 40)
                fprintf(stderr, "[c54x] BITF-000B-HIT #%u PC=0xdec2 mask=0x2000 "
                        "data[0x000b]=0x%04x TC-will-be=%d insn=%u\n",
                        _nc2, s->data[0x000b], (s->data[0x000b] & 0x2000) != 0, s->insn_count);
        }
        /* CLUSTERB-SITES (read-only). The three task-type dispatch sites:
         * 0x8ac4 = task 3 / site 1, 0x8b01 = task 4 / site 2 (the one that
         * fired once), 0x8b8c = task 6 / site 3 (most likely SB_DSP_TASK=6).
         * Logs the current task type d[0x4357] and AR3, expected to be 0x2bc0
         * (the I/Q pointer) for sites 2 and 3. 30 lines per site. */
        {
            static int _cbs = -1;
            if (_cbs < 0) { const char *_e = getenv("CALYPSO_D247_TRACE_OFF"); _cbs = (_e && atoi(_e)) ? 0 : 1; }
            if (_cbs) {
                static unsigned _ncbs[3] = {0};
                uint16_t sites[3] = {0x8ac4, 0x8b01, 0x8b8c};
                for (int _i = 0; _i < 3; _i++) {
                    if (s->pc == sites[_i] && _ncbs[_i]++ < 30) {
                        fprintf(stderr, "[c54x] CLUSTERB-SITE#%d #%u @0x%04x task_type(d[0x4357])=0x%04x "
                                "AR3=0x%04x insn=%u\n",
                                _i+1, _ncbs[_i], sites[_i], s->data[0x4357], s->ar[3], s->insn_count);
                    }
                }
            }
        }
        /* TASKTYPE-SRC (read-only). 0xa6e9 is STL A,*(0x4357), the source of
         * the internal task-type code that drives the whole Cluster B
         * dispatch. Logs A, to identify the upstream event behind each value.
         * 40 lines. */
        {
            static int _tts = -1;
            if (_tts < 0) { const char *_e = getenv("CALYPSO_D247_TRACE_OFF"); _tts = (_e && atoi(_e)) ? 0 : 1; }
            if (_tts && s->pc == 0xa6e9) {
                static unsigned _ntts = 0;
                if (_ntts++ < 40)
                    fprintf(stderr, "[c54x] TASKTYPE-SRC #%u A=0x%06llx (-> d[0x4357]) insn=%u\n",
                            _ntts, (unsigned long long)(s->a & 0xFFFFFFULL), s->insn_count);
            }
        }
        /* A546-HIT (read-only). The only known native BACC to the 0xd247
         * bootstrap goes through 0xa546 (LD d[0x3fe0],A ; BACC A), itself
         * gated by BITF d[0x09bc],1 at 0xa544 (see WATCH-09BC-WR). Confirms
         * whether that path fires. */
        {
            static int _a546on = -1;
            if (_a546on < 0) { const char *_e = getenv("CALYPSO_D247_TRACE_OFF"); _a546on = (_e && atoi(_e)) ? 0 : 1; }
            if (_a546on && s->pc == 0xa546) {
                static unsigned _na546 = 0;
                if (_na546++ < 20)
                    fprintf(stderr, "[c54x] A546-HIT #%u : BACC natif via d[0x3fe0]=0x%04x va tirer "
                            "d[0x09bc]=0x%04x insn=%u\n",
                            _na546, s->data[0x3fe0], s->data[0x09bc], s->insn_count);
            }
        }
        /* C1FA-ENTRY (read-only). 0xc1fa is the only CALA target of the
         * dispatch at 0xa57c (LD d[0x3fd4],A ; CALA A) that had never been
         * traced; CALA-TRACE shows the constant 0xc1fa on every hit. 20 lines,
         * plus a dump of prog[0xc1fa..+0x60] on the first hit so it can be
         * disassembled offline. */
        {
            static int _c1fa = -1;
            if (_c1fa < 0) { const char *_e = getenv("CALYPSO_D247_TRACE_OFF"); _c1fa = (_e && atoi(_e)) ? 0 : 1; }
            if (_c1fa && s->pc == 0xc1fa) {
                static unsigned _nc1fa = 0;
                if (_nc1fa++ < 20)
                    fprintf(stderr, "[c54x] C1FA-ENTRY #%u A=0x%06llx SP=0x%04x "
                            "AR0..7=%04x %04x %04x %04x %04x %04x %04x %04x insn=%u\n",
                            _nc1fa, (unsigned long long)(s->a & 0xFFFFFFULL), s->sp,
                            s->ar[0], s->ar[1], s->ar[2], s->ar[3],
                            s->ar[4], s->ar[5], s->ar[6], s->ar[7], s->insn_count);
                if (_nc1fa == 1) {
                    fprintf(stderr, "[c54x] C1FA-PROG-DUMP prog[0xc1fa..0xc25a]:\n");
                    for (uint16_t a = 0xc1fa; a <= 0xc25a; a++)
                        fprintf(stderr, "[c54x]   prog[0x%04x]=0x%04x\n", a, prog_fetch(s, a));
                }
            }
        }
        /* CLUSTER-B-PROBE (read-only). An xref scan found a PROM0 path that is
         * not polluted by the GPRS bootstrap (Cluster A / 0x87ff) and leads to
         * 0x8f7f/0x8f9d, inside the correlator range, through a per-item
         * dispatcher at 0x86d4-0x871c. Checks whether that path is ever
         * reached natively. 20 lines per site. */
        {
            static int _clb = -1;
            if (_clb < 0) { const char *_e = getenv("CALYPSO_D247_TRACE_OFF"); _clb = (_e && atoi(_e)) ? 0 : 1; }
            if (_clb) {
                static unsigned _nclb[8] = {0};
                uint16_t clb_pcs[8] = {0x86cc, 0x86d4, 0x8ac4, 0x8ad2, 0x8b01, 0x8b09, 0x8b8c, 0x8b94};
                for (int _i = 0; _i < 8; _i++) {
                    if (s->pc == clb_pcs[_i] && _nclb[_i]++ < 20) {
                        fprintf(stderr, "[c54x] CLUSTER-B-PROBE #%u @0x%04x A=0x%06llx SP=0x%04x "
                                "AR0..5=%04x %04x %04x %04x %04x %04x insn=%u\n",
                                _nclb[_i], clb_pcs[_i], (unsigned long long)(s->a & 0xFFFFFFULL), s->sp,
                                s->ar[0], s->ar[1], s->ar[2], s->ar[3], s->ar[4], s->ar[5], s->insn_count);
                    }
                }
            }
        }
        /* SOFT-RESET-TRIGGER probe. The SP-CATASTROPHE trace shows the boot
         * init at PC=0x7120 firing again at insn=190M, i.e. an internal
         * firmware soft reset. Logs every arrival at PC=0xFF80 or PC=0x7120
         * after insn > 100k (the initial silicon reset is long past), with the
         * pc_ring[-16..-1] trail and SP/AR/IMR/IFR/INTM, which names the
         * instruction that branched here. */
        if ((s->pc == 0xFF80 || s->pc == 0x7120) && s->insn_count > 100000) {
            /* Deeper trail, gated by CALYPSO_DEBUG=SOFT_RESET_TRAIL:
             * pc[-64..-1] walks back ~64 instructions before the soft reset to
             * identify the caller chain. */
            if (calypso_debug_enabled("SOFT_RESET_TRAIL")) {
                static unsigned deep_log;
                if (deep_log < 5) {
                    fprintf(stderr,
                        "[c54x] SOFT-RESET DEEP-TRAIL #%u (last 64 PCs):\n",
                        deep_log);
                    for (int row = 0; row < 8; row++) {
                        fprintf(stderr, "[c54x] SR-DEEP[%2d-%2d] :",
                                -64 + row*8, -57 + row*8);
                        for (int col = 0; col < 8; col++) {
                            int idx = -64 + row*8 + col;
                            fprintf(stderr, " %04x",
                                pc_ring[(pc_ring_idx + idx) & 255]);
                        }
                        fprintf(stderr, "\n");
                    }
                    deep_log++;
                }
            }
            static unsigned srt_log;
            if (srt_log < 30) {
                C54_LOG("SOFT-RESET-TRIG #%u PC=0x%04x insn=%u SP=0x%04x "
                        "IMR=0x%04x IFR=0x%04x INTM=%d B=0x%010llx "
                        "AR0=%04x AR1=%04x AR2=%04x AR3=%04x "
                        "AR4=%04x AR5=%04x AR6=%04x AR7=%04x",
                        srt_log, s->pc, s->insn_count, s->sp,
                        s->imr, s->ifr, !!(s->st1 & ST1_INTM),
                        (unsigned long long)(s->b & 0xFFFFFFFFFFULL),
                        s->ar[0], s->ar[1], s->ar[2], s->ar[3],
                        s->ar[4], s->ar[5], s->ar[6], s->ar[7]);
                C54_LOG("SOFT-RESET-TRIG #%u trail pc[-16..-1] = "
                        "%04x %04x %04x %04x %04x %04x %04x %04x "
                        "%04x %04x %04x %04x %04x %04x %04x %04x",
                        srt_log,
                        pc_ring[(pc_ring_idx-17)&255], pc_ring[(pc_ring_idx-16)&255],
                        pc_ring[(pc_ring_idx-15)&255], pc_ring[(pc_ring_idx-14)&255],
                        pc_ring[(pc_ring_idx-13)&255], pc_ring[(pc_ring_idx-12)&255],
                        pc_ring[(pc_ring_idx-11)&255], pc_ring[(pc_ring_idx-10)&255],
                        pc_ring[(pc_ring_idx-9)&255],  pc_ring[(pc_ring_idx-8)&255],
                        pc_ring[(pc_ring_idx-7)&255],  pc_ring[(pc_ring_idx-6)&255],
                        pc_ring[(pc_ring_idx-5)&255],  pc_ring[(pc_ring_idx-4)&255],
                        pc_ring[(pc_ring_idx-3)&255],  pc_ring[(pc_ring_idx-2)&255]);
                srt_log++;
            }
        }
        /* PROM3-VISIT probe. Counts DSP visits to the candidate SB-decode
         * entries 0x8167, 0x81ff and 0x82b8 (PROM3 SB dispatch candidates).
         * Logs the first visit only (insn_count plus caller from the ring),
         * then counts silently. Task 6 firing 30 times with no visit points at
         * the dispatch; visits with sb_att still 0 point deeper into the
         * demodulator. */
        {
            static uint64_t v8167, v81ff, v82b8;
            static uint32_t v8167_first_insn, v81ff_first_insn, v82b8_first_insn;
            uint16_t pc = s->pc;
            if (pc == 0x8167) {
                if (v8167 == 0) {
                    v8167_first_insn = s->insn_count;
                    C54_LOG("PROM3-VISIT 0x8167 FIRST-HIT insn=%u SP=%04x "
                            "AR2=%04x AR3=%04x AR4=%04x AR5=%04x",
                            v8167_first_insn, s->sp,
                            s->ar[2], s->ar[3], s->ar[4], s->ar[5]);
                }
                v8167++;
                if ((v8167 % 100000) == 0)
                    C54_LOG("PROM3-VISIT 0x8167 count=%llu insn=%u",
                            (unsigned long long)v8167, s->insn_count);
            }
            if (pc == 0x81ff) {
                if (v81ff == 0) {
                    v81ff_first_insn = s->insn_count;
                    C54_LOG("PROM3-VISIT 0x81ff FIRST-HIT insn=%u SP=%04x "
                            "AR2=%04x AR3=%04x AR4=%04x AR5=%04x",
                            v81ff_first_insn, s->sp,
                            s->ar[2], s->ar[3], s->ar[4], s->ar[5]);
                }
                v81ff++;
                if ((v81ff % 100000) == 0)
                    C54_LOG("PROM3-VISIT 0x81ff count=%llu insn=%u",
                            (unsigned long long)v81ff, s->insn_count);
            }
            if (pc == 0x82b8) {
                if (v82b8 == 0) {
                    v82b8_first_insn = s->insn_count;
                    C54_LOG("PROM3-VISIT 0x82b8 FIRST-HIT insn=%u SP=%04x "
                            "AR2=%04x AR3=%04x AR4=%04x AR5=%04x",
                            v82b8_first_insn, s->sp,
                            s->ar[2], s->ar[3], s->ar[4], s->ar[5]);
                }
                v82b8++;
                if ((v82b8 % 100000) == 0)
                    C54_LOG("PROM3-VISIT 0x82b8 count=%llu insn=%u",
                            (unsigned long long)v82b8, s->insn_count);
            }
        }
        /* Top-of-loop SP chokepoint. The end-of-loop SP hook is bypassed by
         * every instruction that exits early (goto unimpl, return, continue, a
         * handler that leaves the dispatch chain): the SP write happens but is
         * never accounted. Measured: 61 events captured against an expected
         * descent of 11k+ words.
         *
         * Reading s->sp here, at a mandatory pass-through point, and comparing
         * it with the previous iteration's value is bypass-proof by
         * construction - it watches the VALUE at a chokepoint, not the write
         * sites. The statics persist across c54x_run calls. */
        {
            static uint16_t topgate_last_sp = 0;
            static uint16_t topgate_last_pc = 0;
            static uint16_t topgate_last_op = 0;
            static int      topgate_valid   = 0;

            if (topgate_valid && s->sp != topgate_last_sp) {
                /* Account the PREVIOUS instruction that changed SP, whatever
                 * exit path it took (early exit, return, ...). */
                sp_hist_account(topgate_last_pc, topgate_last_op,
                                topgate_last_sp, s->sp, s->insn_count);
            }

            /* Bootstub-entry trigger: detects the edge prev_pc outside the
             * boot stub -> cur_pc inside it, i.e. the corrupted RET that
             * jumped to 0x00XX. Dumps the verbose state plus the ring, which
             * holds ~4096 iterations of run-up. */
            if (topgate_valid) {
                sp_ring_check_bootstub_entry(s,
                    topgate_last_pc, topgate_last_op, topgate_last_sp,
                    s->pc, s->sp, s->insn_count);
            }

            /* A provenance tracer: tracks A's last writer and dumps at the
             * trigger PC. Settles the NMI-versus-A-divergence fork without an
             * invasive implementation. */
            a_track_init_lazy();
            if (topgate_valid) {
                a_track_iter(s, topgate_last_pc, topgate_last_op);
            }

            /* AR6 windowed snapshot: tells AR6=0 (base divergence) from
             * AR6=0x16 (self-alias feedback) at the trigger PC. Env
             * CALYPSO_AR6_AT_PC=0x821a plus the window size. */
            ar6_at_init_lazy();
            if (topgate_valid) {
                ar6_at_iter(s, topgate_last_pc, topgate_last_op);
            }

            topgate_last_sp = s->sp;
            topgate_last_pc = s->pc;
            topgate_last_op = prog_fetch(s, s->pc);
            topgate_valid   = 1;

            sp_ring_init_lazy();
            sp_ring_record(s->insn_count, s->pc, s->sp, topgate_last_op);

            /* MVPD overlay occupancy: lazy init, then dump once the boot
             * phase has ended. */
            mvpd_trace_init_lazy();
            mvpd_trace_dump_if_due(s->insn_count);

            /* Correlator entry trace: detects the edge prev_pc outside
             * [CORR_PC_LO..CORR_PC_HI) -> cur_pc inside it, and logs the full
             * state on entry (AR3/4/5 are the likely buffer pointers). Dumps
             * the accumulated reads every 20 entries, to see whether the
             * pattern stabilises or varies between runs. */
            corr_trace_init_lazy();
            if (g_corr_trace_enabled > 0 && topgate_valid) {
                /* Use CORR_PC_HI (0x9000), not 0x8f80: the shorter range
                 * silently missed the Cluster B targets 0x8f9d and 0x8fb8. */
                int prev_in = (topgate_last_pc >= CORR_PC_LO && topgate_last_pc < CORR_PC_HI);
                int cur_in  = (s->pc >= CORR_PC_LO && s->pc < CORR_PC_HI);
                if (!prev_in && cur_in) {
                    g_corr_entry_count++;
                    if (g_corr_entry_count <= g_corr_entry_log_cap) {
                        fprintf(stderr,
                            "[c54x] CORR-ENTRY #%u @insn=%u prev_pc=0x%04x → cur_pc=0x%04x\n"
                            "[c54x]   AR0..7: %04x %04x %04x %04x %04x %04x %04x %04x  "
                            "ARP=%d DP=%d BK=0x%04x\n"
                            "[c54x]   SP=0x%04x ST0=0x%04x ST1=0x%04x INTM=%d XPC=%d\n",
                            g_corr_entry_count, s->insn_count,
                            topgate_last_pc, s->pc,
                            s->ar[0], s->ar[1], s->ar[2], s->ar[3],
                            s->ar[4], s->ar[5], s->ar[6], s->ar[7],
                            arp(s), dp(s), s->bk,
                            s->sp, s->st0, s->st1, !!(s->st1 & ST1_INTM), s->xpc);
                    }
                    /* Dump every 20 entries, to see whether the addresses
                     * read stabilise (correlator repeating) or vary. */
                    if ((g_corr_entry_count % 20) == 0) {
                        char tag[32];
                        snprintf(tag, sizeof(tag), "every20-entry%u", g_corr_entry_count);
                        corr_read_dump(tag);
                    }
                }
            }
        }

        /* DSP idle fast-forward — see dsp_idle_fast_forward() comment.
         * Skips MAC simulation when DSP is in its empty-task-slot
         * polling loop, returning host CPU to the rest of QEMU. */
        {
            int ff_cyc;
            if (dsp_idle_fast_forward(s, &ff_cyc)) {
                s->cycles    += ff_cyc;
                s->insn_count += ff_cyc;
                executed     += ff_cyc;
                continue;
            }
        }

        /* Replay any interrupt that fired while INTM=1.
         * c54x_interrupt_ex sets IFR but does nothing else when INTM=1;
         * the real C54x re-evaluates pending interrupts every cycle, so
         * as soon as INTM clears (via RETE or RSBX INTM) a pending
         * BRINT0/TINT0/... must dispatch. Without this, a BRINT0 that
         * arrived inside another ISR is lost and the FB correlator never
         * receives its I/Q samples (d_fb_det stays 0). */
        if (!(s->st1 & ST1_INTM)) {
            uint16_t pending = s->ifr & s->imr;
            if (pending) {
                int imr_bit = __builtin_ctz(pending);
                int vec = imr_bit + 16;
                c54x_ifr_clear(s, (uint16_t)(1 << imr_bit), "vector-ex");
                s->sp--;
                data_write(s, s->sp, s->pc);
                /* A C54x interrupt is a far transition: save XPC
                 * unconditionally and force page 0 for the vector fetch.
                 * Fetching the vector through the live XPC reads the wrong
                 * page. */
                s->sp--;
                data_write(s, s->sp, s->xpc);
                s->st1 |= ST1_INTM;
                /* Set g_last_intr_* here too, otherwise the HIGHVEC and DISP
                 * probes miss replayed interrupts. fg_pc is captured BEFORE
                 * s->pc is overwritten by the vector. */
                g_last_intr_insn = s->insn_count; g_last_intr_vec = vec;
                g_last_intr_fg_pc = (uint16_t)s->pc; g_last_intr_fg_dp = dp(s);
                s->xpc = 0;
                uint16_t iptr = (s->pmst >> PMST_IPTR_SHIFT) & 0x1FF;
                s->pc = (iptr * 0x80) + vec * 4;
                static int pending_log = 0;
                if (pending_log < 20) {
                    C54_LOG("PENDING IRQ replay vec=%d bit=%d PC->0x%04x SP=0x%04x insn=%u",
                            vec, imr_bit, s->pc, s->sp, s->insn_count);
                    pending_log++;
                }
            }
        }

        /* Record PC in ring buffer */
        pc_ring[pc_ring_idx & 255] = s->pc;
        pc_ring_idx++;

        /* Push counter at PC=0xb906. Logs at powers of ten to track the
         * cadence; SP is captured at the hit. */
        {
            static unsigned hit_b906 = 0;
            if (s->pc == 0xb906) {
                hit_b906++;
                if (hit_b906 == 1 || hit_b906 == 10 || hit_b906 == 100 ||
                    hit_b906 == 1000 || hit_b906 == 10000 ||
                    hit_b906 == 100000 || hit_b906 == 1000000) {
                    C54_LOG("HIT-b906 #%u op=0x%04x SP=0x%04x XPC=%d insn=%u",
                            hit_b906, s->prog[0xb906], s->sp, s->xpc,
                            s->insn_count);
                }
            }
        }

        /* INTM transition tracer: every change of ST1 bit 11 with
         * surrounding state. Identifies which IRQ entered the trap and
         * whether RETE / RSBX paths ever execute again. On each 0->1
         * (IRQ entry), also dump prog[PC..PC+8] and the 4 most-recently
         * pushed stack words (data[SP..SP+3]) so we can see what handler
         * we're entering and why it never RETEs.
         *
         * NOTE: this block runs BEFORE c54x_exec_one of the current
         * iteration. So when a transition is observed, the cause was
         * either (a) the previous iteration's exec_one (RETE, RSBX INTM
         * etc. — INTM 1→0), or (b) the pending-IRQ replay block above
         * (INTM 0→1, PC moved to vector). For (a), s->pc has already
         * advanced past the cause — log the previous iteration's
         * exec_pc/exec_op (captured at end of loop into last_exec_*) so
         * the cause is unambiguous. For (b), s->pc IS the vector entry
         * and is informative as-is. */
        {
            static int intm_log = 0;
            static uint16_t prev_intm = 0xFFFF;
            uint16_t cur_intm = !!(s->st1 & ST1_INTM);
            /* Dead site: TINT0 used to be forced on every INTM 1->0
             * transition here. It broke BRINT0, because vec 20 (bit 4) beats
             * vec 21 (bit 5) in priority and starved the I/Q delivery. TINT0
             * now comes from the faithful TIMER0 tick further down, which
             * honours the IMR. The statics are read once and discarded. */
            {
                static int _t0i = -1;
                if (_t0i < 0) _t0i = calypso_gate("CALYPSO_TINT0_MASTER", 0);
                static unsigned _t0period = 0;
                if (_t0period == 0) { const char *_p = getenv("CALYPSO_TINT0_PERIOD"); _t0period = _p ? (unsigned)atoi(_p) : 1500; if (_t0period < 1) _t0period = 1500; }
                static unsigned _t0last = 0;
                (void)_t0i; (void)_t0last; (void)_t0period;
            }
            if (prev_intm != 0xFFFF && cur_intm != prev_intm && intm_log < 200) {
                C54_LOG("INTM-TRANS %u->%u current PC=0x%04x op=0x%04x | "
                        "cause prev_exec PC=0x%04x op=0x%04x | "
                        "XPC=%d IFR=0x%04x SP=0x%04x insn=%u",
                        (unsigned)prev_intm, (unsigned)cur_intm,
                        s->pc, s->prog[s->pc],
                        s->last_exec_pc, s->last_exec_op,
                        s->xpc, s->ifr, s->sp,
                        s->insn_count);
                if (cur_intm == 1) {
                    C54_LOG("  HANDLER prog[PC..PC+8]: %04x %04x %04x %04x %04x %04x %04x %04x %04x",
                            s->prog[s->pc],
                            s->prog[(uint16_t)(s->pc + 1)],
                            s->prog[(uint16_t)(s->pc + 2)],
                            s->prog[(uint16_t)(s->pc + 3)],
                            s->prog[(uint16_t)(s->pc + 4)],
                            s->prog[(uint16_t)(s->pc + 5)],
                            s->prog[(uint16_t)(s->pc + 6)],
                            s->prog[(uint16_t)(s->pc + 7)],
                            s->prog[(uint16_t)(s->pc + 8)]);
                    C54_LOG("  STACK data[SP..SP+3]: %04x %04x %04x %04x",
                            s->data[s->sp],
                            s->data[(uint16_t)(s->sp + 1)],
                            s->data[(uint16_t)(s->sp + 2)],
                            s->data[(uint16_t)(s->sp + 3)]);
                }
                intm_log++;
            }
            /* INT3-CYCLE-TRACE: fire end-good on ANY INTM 1->0 transition,
             * not just RETE - the firmware uses a POPM ST1 + RCD pattern. The
             * callee is a no-op when the probe is disabled or no cycle is
             * active, so the unconditional call is safe. */
            if (prev_intm == 1 && cur_intm == 0) {
                int3_cycle_end_good(s, s->pc);
            }
            prev_intm = cur_intm;
        }

        /* SP-WATCH: log every transition where SP enters / leaves the
         * API mailbox region [0x0800..0x08FF]. This pinpoints the exact
         * instruction that corrupts the stack pointer so we don't have
         * to keep recoding to investigate. */
        {
            static uint16_t prev_sp = 0xFFFF;
            bool was_in = (prev_sp >= 0x0800 && prev_sp < 0x0900);
            bool is_in  = (s->sp  >= 0x0800 && s->sp  < 0x0900);
            if (was_in != is_in) {
                if (calypso_debug_enabled("SP-WATCH")) fprintf(stderr,
                        "[c54x] SP-WATCH %s SP=0x%04x (prev=0x%04x) "
                        "PC=0x%04x op=0x%04x insn=%u\n",
                        is_in ? "ENTER api" : "LEAVE api",
                        s->sp, prev_sp, s->pc, s->prog[s->pc], s->insn_count);
            }
            prev_sp = s->sp;
        }

        /* SP-DRAIN probe (CALYPSO_DEBUG=SP-DRAIN): attributes each net SP
         * decrement to the instruction that just executed (last_exec_pc/op,
         * captured at the end of the previous iteration). These blocks run
         * BEFORE exec_one of the current iteration, so s->sp reflects the
         * previous instruction. Isolates the unpaired instruction that drains
         * SP in the boot trampoline 0x0000 <-> 0xffcd. 8-slot histogram plus
         * the first 120 events. Silent by default. */
        if (calypso_debug_enabled("SP-DRAIN")) {
            static uint16_t sd_prev_sp = 0xFFFF;
            static unsigned  sd_log = 0;
            static uint32_t  sd_cnt[8];
            static uint16_t  sd_pc[8];
            static uint32_t  sd_events;
            if (sd_prev_sp != 0xFFFF) {
                int delta = (int)(uint16_t)(sd_prev_sp - s->sp); /* >0 = push */
                if (delta > 0 && delta < 0x100) {
                    uint16_t cpc = s->last_exec_pc;
                    int slot = -1, freeslot = -1;
                    for (int i = 0; i < 8; i++) {
                        if (sd_cnt[i] && sd_pc[i] == cpc) { slot = i; break; }
                        if (!sd_cnt[i] && freeslot < 0) freeslot = i;
                    }
                    if (slot < 0 && freeslot >= 0) { slot = freeslot; sd_pc[slot] = cpc; }
                    if (slot >= 0) sd_cnt[slot] += (uint32_t)delta;
                    if (sd_log < 120) {
                        sd_log++;
                        C54_DBG("SP-DRAIN",
                                "push -%d SP=0x%04x<-0x%04x by PC=0x%04x op=0x%04x insn=%u",
                                delta, s->sp, sd_prev_sp, cpc,
                                s->last_exec_op, s->insn_count);
                    }
                    if ((++sd_events % 1000) == 0) {
                        C54_DBG("SP-DRAIN",
                                "TOP pushers: %04x:%u %04x:%u %04x:%u %04x:%u "
                                "%04x:%u %04x:%u %04x:%u %04x:%u (events=%u SP=0x%04x)",
                                sd_pc[0], sd_cnt[0], sd_pc[1], sd_cnt[1],
                                sd_pc[2], sd_cnt[2], sd_pc[3], sd_cnt[3],
                                sd_pc[4], sd_cnt[4], sd_pc[5], sd_cnt[5],
                                sd_pc[6], sd_cnt[6], sd_pc[7], sd_cnt[7],
                                sd_events, s->sp);
                    }
                }
            }
            sd_prev_sp = s->sp;
        }

        /* CALLSITE probe (CALYPSO_DEBUG=CALLSITE): at the RCD epilogue
         * 0x7707, dumps the return address RCD is about to pop, the call-site
         * opcode (FCALL F9xx versus CALL F074) and the pre-RETD pc-ring, which
         * tells a park from a crash. */
        if (s->pc == 0x7707 && calypso_debug_enabled("CALLSITE")) {
            static int n7707 = 0;
            if (n7707 < 8) {
                n7707++;
                uint16_t ret = data_read(s, s->sp);
                C54_DBG("CALLSITE",
                    "RCD@7707 #%d SP=0x%04x ret=0x%04x caller[ret-2..ret-1]=0x%04x 0x%04x XPC=%d insn=%u",
                    n7707, s->sp, ret, prog_read(s, (uint16_t)(ret-2)),
                    prog_read(s, (uint16_t)(ret-1)), s->xpc, s->insn_count);
                char buf[300]; int o=0;
                for (int i=20;i>=1;i--)
                    o+=snprintf(buf+o,sizeof(buf)-o,"%04x ", pc_ring[(pc_ring_idx-i)&255]);
                C54_DBG("CALLSITE", "  pre-RETD pcring(20): %s", buf);
            }
        }

        /* XPC-WR tracer (CALYPSO_DEBUG=XPC-WR): every XPC transition with the
         * instruction that caused it, i.e. the origin of the XPC=3 garbage. */
        if (calypso_debug_enabled("XPC-WR")) {
            static uint8_t xprev = 0xFF;
            if (xprev != 0xFF && (uint8_t)s->xpc != xprev) {
                C54_DBG("XPC-WR",
                    "XPC %u->%u cause prev_exec PC=0x%04x op=0x%04x SP=0x%04x insn=%u",
                    xprev, (unsigned)(s->xpc & 0xFF), s->last_exec_pc,
                    s->last_exec_op, s->sp, s->insn_count);
            }
            xprev = (uint8_t)s->xpc;
        }

        /* AR2-WR tracer (CALYPSO_DEBUG=AR2-WR): tells a reset from a runaway.
         * delta == -1 is the normal post-decrement (logged every 200);
         * delta != -1 is a reset, jump or load, which is the discriminator
         * between "a reset exists" and "there is never a reset". Reports BK
         * and the reset target. */
        if (calypso_debug_enabled("AR2-WR")) {
            static int      ar2_first = 1;
            static uint16_t ar2_prev = 0;
            static uint32_t ar2_dec  = 0;
            uint16_t cur = s->ar[2];
            if (!ar2_first && cur != ar2_prev) {
                int delta = (int)(int16_t)(cur - ar2_prev);
                if (delta == -1) {
                    if ((++ar2_dec % 200) == 0)
                        C54_DBG("AR2-WR", "AR2 dec #%u ->0x%04x (linear -1) PC=0x%04x insn=%u",
                                ar2_dec, cur, s->last_exec_pc, s->insn_count);
                } else {
                    C54_DBG("AR2-WR",
                        "AR2 %s 0x%04x->0x%04x (delta=%+d) cause PC=0x%04x op=0x%04x BK=0x%04x insn=%u",
                        delta > 0 ? "RESET/UP" : "JUMP-DN", ar2_prev, cur, delta,
                        s->last_exec_pc, s->last_exec_op, s->bk, s->insn_count);
                }
            }
            ar2_first = 0; ar2_prev = cur;
        }

        /* Dump entry into the 0xe260 loop (first 5 hits). */
        if (s->pc == 0xe260 || s->pc == 0xe261) {
            static int e260_log = 0;
            if (e260_log < 5) {
                e260_log++;
                C54_LOG("E260-ENTRY #%d PC=0x%04x AR2=%04x AR5=%04x BRC=%d RSA=%04x REA=%04x rptb=%d IMR=%04x SP=%04x insn=%u",
                        e260_log, s->pc, s->ar[2], s->ar[5], s->brc, s->rsa, s->rea, s->rptb_active, s->imr, s->sp, s->insn_count);
                int idx = pc_ring_idx;
                char buf[1024]; int o = 0;
                for (int i = 50; i >= 1; i--) {
                    o += snprintf(buf+o, sizeof(buf)-o, "%04x ", pc_ring[(idx-i)&255]);
                }
                C54_LOG("E260-PCRING (last 50): %s", buf);
                /* Runtime opcodes 0xe255..0xe28f */
                char ob[1024]; int oo = 0;
                for (uint16_t a = 0xe255; a <= 0xe28f; a++) {
                    oo += snprintf(ob+oo, sizeof(ob)-oo, "%04x ", s->prog[a]);
                }
                C54_LOG("E260-PROG[e255..e28f]: %s", ob);
            }
        }

        /* CALA loop tracer: A and SP at PC=0xd24e and 0xd250 (first 40). */
        if (s->pc == 0xd24e || s->pc == 0xd250) {
            static int cala_log = 0;
            if (cala_log++ < 40) {
                C54_LOG("CALA-TRACE PC=0x%04x A=%08x SP=0x%04x BRC=%d AR2=%04x AR3=%04x AR4=%04x AR5=%04x insn=%u",
                        s->pc, (uint32_t)(s->a & 0xFFFFFFFF), s->sp, s->brc,
                        s->ar[2], s->ar[3], s->ar[4], s->ar[5], s->insn_count);
            }
        }

        /* PC histogram: visits per PC, top 20 dumped every 2M insns. */
        {
            static uint32_t pc_hist[0x10000];
            static uint64_t hist_last_dump = 0;
            pc_hist[s->pc]++;
            if (s->insn_count - hist_last_dump >= 2000000) {
                hist_last_dump = s->insn_count;
                /* find the top 20 */
                uint32_t top_cnt[20] = {0};
                uint16_t top_pc[20] = {0};
                for (int i = 0; i < 0x10000; i++) {
                    uint32_t c = pc_hist[i];
                    if (c == 0) continue;
                    for (int j = 0; j < 20; j++) {
                        if (c > top_cnt[j]) {
                            for (int k = 19; k > j; k--) {
                                top_cnt[k] = top_cnt[k-1];
                                top_pc[k] = top_pc[k-1];
                            }
                            top_cnt[j] = c;
                            top_pc[j] = (uint16_t)i;
                            break;
                        }
                    }
                }
                C54_LOG("PC HIST insn=%u top: %04x:%u %04x:%u %04x:%u %04x:%u %04x:%u %04x:%u %04x:%u %04x:%u %04x:%u %04x:%u",
                        s->insn_count,
                        top_pc[0], top_cnt[0], top_pc[1], top_cnt[1], top_pc[2], top_cnt[2],
                        top_pc[3], top_cnt[3], top_pc[4], top_cnt[4], top_pc[5], top_cnt[5],
                        top_pc[6], top_cnt[6], top_pc[7], top_cnt[7], top_pc[8], top_cnt[8],
                        top_pc[9], top_cnt[9]);
                C54_LOG("PC HIST cont:        %04x:%u %04x:%u %04x:%u %04x:%u %04x:%u %04x:%u %04x:%u %04x:%u %04x:%u %04x:%u",
                        top_pc[10], top_cnt[10], top_pc[11], top_cnt[11], top_pc[12], top_cnt[12],
                        top_pc[13], top_cnt[13], top_pc[14], top_cnt[14], top_pc[15], top_cnt[15],
                        top_pc[16], top_cnt[16], top_pc[17], top_cnt[17], top_pc[18], top_cnt[18],
                        top_pc[19], top_cnt[19]);
                memset(pc_hist, 0, sizeof(pc_hist));
            }
        }

        /* Rolling PC sampler: a histogram over a 100k-insn window, reset each
         * window, so the top-5 always answers "what is the DSP doing right
         * now".
         *
         * The cumulative-since-boot PC histogram is useless for that: it shows
         * 0xa218..0xa222 dominant because the init loop at 0xa222 (BANZD on
         * AR5, 60k iterations, ~984k insns = 0.06% of a 1.7B run) ran once
         * early and stays at the top forever. */
        {
            static uint32_t pc_recent[0x10000];
            static uint32_t recent_last_dump = 0;
            pc_recent[s->pc]++;
            if (s->insn_count - recent_last_dump >= 100000) {
                recent_last_dump = s->insn_count;
                uint32_t top_cnt[5] = {0};
                uint16_t top_pc[5]  = {0};
                for (int i = 0; i < 0x10000; i++) {
                    uint32_t c = pc_recent[i];
                    if (c <= top_cnt[4]) continue;
                    top_cnt[4] = c; top_pc[4] = (uint16_t)i;
                    for (int j = 4; j > 0 && top_cnt[j] > top_cnt[j-1]; j--) {
                        uint32_t tc = top_cnt[j]; top_cnt[j] = top_cnt[j-1]; top_cnt[j-1] = tc;
                        uint16_t tp = top_pc[j]; top_pc[j] = top_pc[j-1]; top_pc[j-1] = tp;
                    }
                }
                C54_LOG("PC RECENT (last 100k) top: %04x:%u %04x:%u %04x:%u %04x:%u %04x:%u",
                        top_pc[0], top_cnt[0], top_pc[1], top_cnt[1],
                        top_pc[2], top_cnt[2], top_pc[3], top_cnt[3],
                        top_pc[4], top_cnt[4]);
                memset(pc_recent, 0, sizeof(pc_recent));
            }
        }

        /* ENTER-RPTB-A218 probe. Measured over the first 20 events: BRC is 0
         * and AR1 is 0 on every visit, AR2 increments by 2, and visits are 16
         * insns apart. Logs the first 200 events, then samples every 100k
         * visits so the late run is covered too (a cap of 20 saturated at
         * insn=48M on a 2.4B-insn run). The !s->rpt_active guard avoids
         * spurious mid-RPTB hits. */
        if (s->pc == 0xa218 && !s->rpt_active) {
            static unsigned a218_total = 0;
            static int a218_log = 0;
            a218_total++;
            bool log_now = (a218_log < 200) ||
                           (a218_total % 100000 == 0);
            if (log_now) {
                C54_LOG("ENTER-RPTB-A218 #%d total=%u BRC=%u (0x%04x) "
                        "AR0=0x%04x AR1=0x%04x AR2=0x%04x AR3=0x%04x "
                        "AR4=0x%04x AR5=0x%04x A=%010llx T=0x%04x "
                        "ST0=0x%04x insn=%u",
                        a218_log + 1, a218_total, s->brc, s->brc,
                        s->ar[0], s->ar[1], s->ar[2], s->ar[3],
                        s->ar[4], s->ar[5],
                        (unsigned long long)(s->a & 0xFFFFFFFFFFLL),
                        s->t, s->st0, s->insn_count);
                a218_log++;
            }
        }
        /* BANZD-A222 probe. 0xa222 op=0x6e81 with operand 0x8208 is
         * `BANZD pmad, *Sind`. Captures every AR, so the one the *Sind operand
         * actually decodes to is observed rather than guessed: AR1 is 0 on
         * every visit, so the branch test uses a different AR (AR5). First 200
         * events, then every 100k. */
        if (s->pc == 0xa222 && !s->rpt_active) {
            static unsigned a222_total = 0;
            static int a222_log = 0;
            a222_total++;
            bool log_now = (a222_log < 200) ||
                           (a222_total % 100000 == 0);
            if (log_now) {
                C54_LOG("BANZD-A222 #%d total=%u op=0x%04x op2=0x%04x "
                        "AR0=0x%04x AR1=0x%04x AR2=0x%04x AR3=0x%04x "
                        "AR4=0x%04x AR5=0x%04x AR6=0x%04x AR7=0x%04x "
                        "BRC=%u insn=%u",
                        a222_log + 1, a222_total,
                        s->prog[s->pc], s->prog[(uint16_t)(s->pc + 1)],
                        s->ar[0], s->ar[1], s->ar[2], s->ar[3],
                        s->ar[4], s->ar[5], s->ar[6], s->ar[7],
                        s->brc, s->insn_count);
                a222_log++;
            }
        }
        /* Companion probe at 0xa215 (BRC setup) and 0xa217 (outer entry).
         * 0xa215 op=0x4492 with 0xa216 operand 0x0092 is `ADD/SUB Smem,16,dst`
         * per tic54x-opc.c (2 words, mask FE00, base 0x4400). Logs A before
         * and after plus the Smem read, to trace the value landing in dst,
         * which may eventually feed BRC. 30 events. */
        if (s->pc == 0xa215 || s->pc == 0xa217) {
            static int brc_setup_215 = 0;
            static int brc_setup_217 = 0;
            int *cnt = (s->pc == 0xa215) ? &brc_setup_215 : &brc_setup_217;
            if (*cnt < 30) {
                C54_LOG("ENTER-A%04x #%d AR0=%04x AR1=%04x AR2=%04x "
                        "A=%010llx B=%010llx T=%04x BRC=%u DP=0x%03x insn=%u",
                        s->pc, *cnt + 1,
                        s->ar[0], s->ar[1], s->ar[2],
                        (unsigned long long)(s->a & 0xFFFFFFFFFFLL),
                        (unsigned long long)(s->b & 0xFFFFFFFFFFLL),
                        s->t, s->brc, (s->st0 & 0x1FF), s->insn_count);
                (*cnt)++;
            }
        }

        /* XC-COND probe at PC=0xa0e0 / 0xa0e4. The routine 0xa0e0..0xa0e9 ends
         * at PC=0xa0e7 op=0xc8be with AR4 consistently 0x18 (= MMR_SP) before
         * the instruction, so the ST||LD writes to SP.
         *
         * The static dump shows two `XC 1, cond` before 0xc8be:
         *   0xa0e0 = 0xfd30  ; XC 1, cond=0x30 (TC)
         *   0xa0e4 = 0xfd43  ; XC 1, cond=0x43 (ALT, A<0)
         * When such a condition is false, the conditional STM #lk,AR4 (likely
         * at 0xa0e5) is skipped and AR4 keeps the stale 0x18 from an earlier
         * path. Logs the condition byte, the TC/A/B flags, AR4 and the next
         * opcode (the one skipped or executed) on every visit, 100 per PC. */
        if (s->pc == 0xa0e0 || s->pc == 0xa0e4) {
            static unsigned xc_log_e0;
            static unsigned xc_log_e4;
            unsigned *cnt = (s->pc == 0xa0e0) ? &xc_log_e0 : &xc_log_e4;
            if (*cnt < 100) {
                uint16_t op_xc = s->prog[s->pc];
                uint8_t  cond_byte = op_xc & 0xFF;
                uint16_t next_op   = s->prog[(uint16_t)(s->pc + 1)];
                /* Mirrors the condition decode of the XC handler in
                 * c54x_exec_one, common subset only. */
                bool cond = false;
                if      (cond_byte == 0x00) cond = true;
                else if (cond_byte == 0x0C) cond = (s->st0 & ST0_C) != 0;
                else if (cond_byte == 0x08) cond = !(s->st0 & ST0_C);
                else if (cond_byte == 0x30) cond = (s->st0 & ST0_TC) != 0;
                else if (cond_byte == 0x20) cond = !(s->st0 & ST0_TC);
                else if (cond_byte == 0x45) cond = (sext40(s->a) == 0);
                else if (cond_byte == 0x44) cond = (sext40(s->a) != 0);
                else if (cond_byte == 0x46) cond = (sext40(s->a) > 0);
                else if (cond_byte == 0x42) cond = (sext40(s->a) >= 0);
                else if (cond_byte == 0x43) cond = (sext40(s->a) < 0);
                else if (cond_byte == 0x47) cond = (sext40(s->a) <= 0);
                else if (cond_byte == 0x4D) cond = (sext40(s->b) == 0);
                else if (cond_byte == 0x4C) cond = (sext40(s->b) != 0);
                else if (cond_byte == 0x4E) cond = (sext40(s->b) > 0);
                else if (cond_byte == 0x4A) cond = (sext40(s->b) >= 0);
                else if (cond_byte == 0x4B) cond = (sext40(s->b) < 0);
                else if (cond_byte == 0x4F) cond = (sext40(s->b) <= 0);
                if (calypso_debug_enabled("XC-COND")) fprintf(stderr,
                        "[c54x] XC-COND #%u PC=0x%04x op=0x%04x cond=0x%02x "
                        "→ %s | TC=%d C=%d A=%010llx (sgn:%c) "
                        "B=%010llx (sgn:%c) AR4=0x%04x next_op=0x%04x insn=%u\n",
                        *cnt + 1, s->pc, op_xc, cond_byte,
                        cond ? "TAKEN " : "SKIPPED",
                        !!(s->st0 & ST0_TC),
                        !!(s->st0 & ST0_C),
                        (unsigned long long)(s->a & 0xFFFFFFFFFFULL),
                        sext40(s->a) < 0 ? '-' : (sext40(s->a) == 0 ? '0' : '+'),
                        (unsigned long long)(s->b & 0xFFFFFFFFFFULL),
                        sext40(s->b) < 0 ? '-' : (sext40(s->b) == 0 ? '0' : '+'),
                        s->ar[4], next_op, s->insn_count);
                (*cnt)++;
            }
        }

        /* MAC-8d33 trace: the FB-det inner correlator.
         * The DSP loops indefinitely in 0x8d2d..0x8d36. Static dump:
         *   8d2d 0x771a 0x0004      ; (2-word) — likely setup
         *   8d2f 0xf072 0x8d33      ; RPTB pmad, end=0x8d33 (per tic54x)
         *   8d31 0xf461             ; F46x = SFTA src,shift,dst (1-word)
         *   8d32 0xf591             ; F591 = ROL B (per our decoder)
         *   8d33 0xf3e2             ; F3E0-F3FF = SFTL src,SHIFT,DST  (writes a_sync_SNR)
         *   8d34 0x6e89 0x8d2d      ; BANZD pmad=0x8d2d, *AR - outer back-branch
         *   8d36 0xf3e1             ; SFTL B,1,B (exit path)
         * PC histogram counts (105k outer / 526k inner = 5x) confirm the RPTB
         * body (0x8d32, 0x8d33, 0x8d34) runs 5 times per outer iteration.
         *
         * Captures A before, T and AR2..AR5 at each PC in this zone, rate
         * limited to: the first 50 (init and early convergence), every 5000th
         * (steady state), and any |A_after - last_logged_A| > 0x100000, a
         * significant accumulator shift worth dumping. */
        if (s->pc >= 0x8d2c && s->pc <= 0x8d3a) {
            static uint64_t mac8d_count;
            static int64_t  last_logged_a;
            int64_t a_now = sext40(s->a);
            int64_t da = a_now - last_logged_a;
            if (da < 0) da = -da;
            mac8d_count++;
            bool log_now = (mac8d_count <= 50) ||
                           (mac8d_count % 5000) == 0 ||
                           da > 0x100000LL;
            if (log_now) {
                C54_LOG("MAC-8d33 #%llu PC=0x%04x op=0x%04x A_pre=%010llx B=%010llx "
                        "T=0x%04x ARs: %04x %04x %04x %04x %04x %04x BRC=%d insn=%u",
                        (unsigned long long)mac8d_count,
                        s->pc, s->prog[s->pc],
                        (unsigned long long)(s->a & 0xFFFFFFFFFFULL),
                        (unsigned long long)(s->b & 0xFFFFFFFFFFULL),
                        s->t,
                        s->ar[2], s->ar[3], s->ar[4], s->ar[5], s->ar[6], s->ar[7],
                        s->brc, s->insn_count);
                last_logged_a = a_now;
            }
        }
        /* Outer-entry tracer at PC=0x8d2d: always logs A on entry, 200 events.
         * A non-zero on outer entry means the accumulator was not reset
         * between FB-det attempts - the 21 consecutive 0x2fb0 SNR values
         * observed are consistent with an accumulator stuck across
         * attempts. */
        if (s->pc == 0x8d2d) {
            static uint64_t enter_8d2d;
            enter_8d2d++;
            if (enter_8d2d <= 200) {
                C54_LOG("ENTER-8d2d #%llu A_pre=%010llx B_pre=%010llx T=0x%04x "
                        "ARs: %04x %04x %04x %04x %04x %04x %04x %04x SP=0x%04x BRC=%d insn=%u",
                        (unsigned long long)enter_8d2d,
                        (unsigned long long)(s->a & 0xFFFFFFFFFFULL),
                        (unsigned long long)(s->b & 0xFFFFFFFFFFULL),
                        s->t,
                        s->ar[0], s->ar[1], s->ar[2], s->ar[3],
                        s->ar[4], s->ar[5], s->ar[6], s->ar[7],
                        s->sp, s->brc, s->insn_count);
            }
        }

        /* HOT-OPS probe for 0xe9ac..0xe9b7 plus 0xe981..0xe983. The DSP locks
         * in a deterministic 7-instruction loop at 0xe9ac..0xe9b7 (PROM1
         * mirror), with an outer 3-PC loop at 0xe981..0xe983 reloading a BRC
         * counter - the shape of `RPTB end_addr` plus an outer reset. One-shot
         * dump of the real opcodes on first entry into the body range, with
         * the words before it (the RPTB instruction itself) and the outer
         * loop. */
        {
            static bool e9ac_dumped = false;
            if (!e9ac_dumped && s->pc >= 0xe9ac && s->pc <= 0xe9b7) {
                e9ac_dumped = true;
                fprintf(stderr,
                        "[c54x] HOT-OPS-DUMP triggered at PC=0x%04x insn=%u\n",
                        s->pc, s->insn_count);
                fprintf(stderr,
                        "[c54x] HOT-OPS prog[0xe9a0..0xe9bf]:");
                for (uint16_t a = 0xe9a0; a <= 0xe9bf; a++)
                    fprintf(stderr, " %04x", s->prog[a]);
                fprintf(stderr, "\n");
                fprintf(stderr,
                        "[c54x] HOT-OPS prog[0xe97c..0xe98f] (outer):");
                for (uint16_t a = 0xe97c; a <= 0xe98f; a++)
                    fprintf(stderr, " %04x", s->prog[a]);
                fprintf(stderr, "\n");
                fprintf(stderr,
                        "[c54x] HOT-OPS state: BRC=%d RSA=0x%04x REA=0x%04x "
                        "rptb_active=%d ST1=0x%04x AR0..7: %04x %04x %04x %04x "
                        "%04x %04x %04x %04x\n",
                        s->brc, s->rsa, s->rea, s->rptb_active, s->st1,
                        s->ar[0], s->ar[1], s->ar[2], s->ar[3],
                        s->ar[4], s->ar[5], s->ar[6], s->ar[7]);
            }
        }

        /* Track SP changes inside RPTB loops */
        uint16_t sp_before = s->sp;
        /* Snapshot for the transfer ring, the A-write ring and the NOP-region
         * guard. */
        uint16_t pre_pc  = s->pc;
        uint8_t  pre_xpc = s->xpc & 0x3;
        uint16_t pre_op  = prog_fetch(s, s->pc);
        int64_t  pre_a   = s->a;

        /* EB04 loop: dump the first 20 iterations. */
        if (s->pc == 0xEB04) {
            static int eb04_log = 0;
            if (eb04_log < 20) {
                C54_LOG("EB04 op=%04x A=0x%010llx B=0x%010llx T=%04x "
                        "INTM=%d IMR=%04x IFR=%04x rptb=%d RSA=%04x REA=%04x BRC=%d "
                        "AR0=%04x AR1=%04x AR2=%04x AR3=%04x AR4=%04x AR5=%04x AR6=%04x AR7=%04x",
                        prog_fetch(s, s->pc),
                        (unsigned long long)(s->a & 0xFFFFFFFFFFLL),
                        (unsigned long long)(s->b & 0xFFFFFFFFFFLL),
                        s->t,
                        !!(s->st1 & ST1_INTM), s->imr, s->ifr,
                        s->rptb_active, s->rsa, s->rea, s->brc,
                        s->ar[0], s->ar[1], s->ar[2], s->ar[3],
                        s->ar[4], s->ar[5], s->ar[6], s->ar[7]);
                eb04_log++;
            }
        }

        /* Dump DSP state when stuck — triggers once after 500M instructions
         * if DSP hasn't reached IDLE yet */
        {
            static int dumped = 0;
            if (s->insn_count > 500000000 && !dumped && !s->idle) {
                dumped = 1;
                C54_LOG("DSP NO-IDLE dump at insn=%u PC=0x%04x:", s->insn_count, s->pc);
                C54_LOG("  ST0=0x%04x ST1=0x%04x PMST=0x%04x SP=0x%04x INTM=%d",
                        s->st0, s->st1, s->pmst, s->sp, !!(s->st1 & ST1_INTM));
                C54_LOG("  IMR=0x%04x IFR=0x%04x rptb=%d RSA=0x%04x REA=0x%04x BRC=%d",
                        s->imr, s->ifr, s->rptb_active, s->rsa, s->rea, s->brc);
                C54_LOG("  A=0x%010llx B=0x%010llx T=0x%04x XPC=%d",
                        (unsigned long long)(s->a & 0xFFFFFFFFFFLL),
                        (unsigned long long)(s->b & 0xFFFFFFFFFFLL), s->t, s->xpc);
                C54_LOG("  AR0=%04x AR1=%04x AR2=%04x AR3=%04x AR4=%04x AR5=%04x AR6=%04x AR7=%04x",
                        s->ar[0], s->ar[1], s->ar[2], s->ar[3],
                        s->ar[4], s->ar[5], s->ar[6], s->ar[7]);
                /* Code around the current PC; prog_fetch honours OVLY. */
                C54_LOG("  Code around PC:");
                for (int i = -4; i < 16; i++) {
                    uint16_t a = s->pc + i;
                    C54_LOG("  %c [0x%04x] = 0x%04x",
                            i == 0 ? '>' : ' ', a, prog_fetch(s, a));
                }
                C54_LOG("  ST0=0x%04x ST1=0x%04x PMST=0x%04x SP=0x%04x INTM=%d",
                        s->st0, s->st1, s->pmst, s->sp, !!(s->st1 & ST1_INTM));
                C54_LOG("  A=0x%010llx B=0x%010llx T=0x%04x",
                        (unsigned long long)(s->a & 0xFFFFFFFFFFLL),
                        (unsigned long long)(s->b & 0xFFFFFFFFFFLL), s->t);
                C54_LOG("  AR0=%04x AR1=%04x AR2=%04x AR3=%04x AR4=%04x AR5=%04x AR6=%04x AR7=%04x",
                        s->ar[0], s->ar[1], s->ar[2], s->ar[3],
                        s->ar[4], s->ar[5], s->ar[6], s->ar[7]);
            }
        }

        /* BSP read entry points: these routines contain PORTR PA=0xF430 (read
         * a BSP sample), so a DSP that never visits them has a dead FB-det
         * chain. Targets come from static analysis of the PROM0 callers of the
         * 64 PORTR PA=0xF430 sites at 0x9b80+. */
        if (!s->rpt_active &&
            (s->pc == 0x9a78 || s->pc == 0x9aaf || s->pc == 0x9ad3 ||
             s->pc == 0x9b4c || s->pc == 0x8811)) {
            static unsigned bsp_visits[5];
            int idx = (s->pc == 0x9a78) ? 0 :
                      (s->pc == 0x9aaf) ? 1 :
                      (s->pc == 0x9ad3) ? 2 :
                      (s->pc == 0x9b4c) ? 3 : 4;
            if (bsp_visits[idx] < 5) {
                bsp_visits[idx]++;
                C54_LOG("BSP-ENTRY PC=0x%04x  A=0x%010llx ar0=%04x ar1=%04x "
                        "ar2=%04x ar3=%04x ar4=%04x SP=0x%04x insn=%u",
                        s->pc,
                        (unsigned long long)(s->a & 0xFFFFFFFFFFLL),
                        s->ar[0], s->ar[1], s->ar[2], s->ar[3], s->ar[4],
                        s->sp, s->insn_count);
            }
        }

        /* Writes to the dispatcher poll addresses data[0x4359] / data[0x3fab]
         * are traced in data_write. */

        /* Dispatcher hot loop at PROM0 0xb968-0xb9a4: the state machine the
         * DSP spins in while waiting for ARM tasks. Logs the first 8 visits
         * per PC, enough to show the whole conditional structure - which
         * addresses it polls and which constants it compares against. */
        if (s->pc >= 0xb968 && s->pc <= 0xb9a4 && !s->rpt_active) {
            static uint8_t disp_visits[64];
            int idx = s->pc - 0xb968;
            if (idx >= 0 && idx < 64 && disp_visits[idx] < 8) {
                disp_visits[idx]++;
                C54_LOG("DISP-TRACE PC=0x%04x op=0x%04x A=0x%010llx "
                        "B=0x%010llx ar0=%04x ar1=%04x ar2=%04x ar3=%04x "
                        "ar4=%04x ar5=%04x TC=%d",
                        s->pc, prog_fetch(s, s->pc),
                        (unsigned long long)(s->a & 0xFFFFFFFFFFLL),
                        (unsigned long long)(s->b & 0xFFFFFFFFFFLL),
                        s->ar[0], s->ar[1], s->ar[2], s->ar[3],
                        s->ar[4], s->ar[5],
                        !!(s->st0 & ST0_TC));
            }
        }

        /* IRQ vector area 0xFFCC-0xFFE0 (INT3, TINT0 and BRINT0 slots):
         * captures the three 4-word handlers the INT3 dispatch lands on at
         * IPTR=0x1ff. First 4 visits per PC. */
        if (s->pc >= 0xFFCC && s->pc < 0xFFE0 && !s->rpt_active) {
            static uint8_t vec_visits[20];   /* index 0 = 0xffcc */
            int idx = s->pc - 0xFFCC;
            if (vec_visits[idx] < 4) {
                vec_visits[idx]++;
                C54_LOG("VEC-TRACE PC=0x%04x op=0x%04x SP=0x%04x A=0x%010llx "
                        "B=0x%010llx TC=%d INTM=%d ar7=%04x",
                        s->pc, prog_fetch(s, s->pc), s->sp,
                        (unsigned long long)(s->a & 0xFFFFFFFFFFLL),
                        (unsigned long long)(s->b & 0xFFFFFFFFFFLL),
                        !!(s->st0 & ST0_TC),
                        !!(s->st1 & ST1_INTM),
                        s->ar[7]);
            }
        }

        /* DSP init: one line per distinct PC in 0xE900-0xE960. */
        if (s->pc >= 0xE900 && s->pc < 0xE960 && !s->rpt_active) {
            static uint16_t seen_pcs[96];
            int idx = s->pc - 0xE900;
            if (!seen_pcs[idx]) {
                seen_pcs[idx] = 1;
                C54_LOG("INIT PC=0x%04x op=0x%04x SP=0x%04x BRC=%d rptb=%d RSA=0x%04x REA=0x%04x",
                        s->pc, prog_fetch(s, s->pc), s->sp, s->brc,
                        s->rptb_active, s->rsa, s->rea);
            }
        }

        /* SINT17 handler (0x8a00-0x8a5f). */
        if (s->pc >= 0x8a00 && s->pc < 0x8a60) {
            static int sint17_log = 0;
            if (sint17_log < 500) {
                C54_LOG("SINT17 PC=0x%04x op=0x%04x SP=0x%04x DP=0x%03x A=0x%010llx B=0x%010llx AR0=%04x",
                        s->pc, prog_fetch(s, s->pc), s->sp, dp(s),
                        (unsigned long long)(s->a & 0xFFFFFFFFFFLL),
                        (unsigned long long)(s->b & 0xFFFFFFFFFFLL), s->ar[0]);
                sint17_log++;
            }
        }

        /* Sample the PC every 1M instructions, to find stuck loops. */
        if (executed > 0 && (executed % 1000000) == 0) {
            static int sample_log = 0;
            if (sample_log < 20)
                C54_LOG("@%dM: PC=0x%04x op=0x%04x SP=0x%04x insn=%u",
                        executed/1000000, s->pc, prog_read(s, s->pc), s->sp, s->insn_count);
            sample_log++;
        }
        if (run_num <= 2 && executed < 2000) {
            C54_LOG("BOOT[%d.%d] PC=0x%04x op=0x%04x SP=0x%04x A=0x%010llx B=0x%010llx",
                    run_num, executed, s->pc, prog_fetch(s, s->pc), s->sp,
                    (unsigned long long)(s->a & 0xFFFFFFFFFFLL),
                    (unsigned long long)(s->b & 0xFFFFFFFFFFLL));
        }
        /* The RPTB end-of-body check lives below, after `s->pc += consumed`,
         * so the redirect to RSA is the last write to PC in this iteration.
         * Running it before the PC advance is off by one: the redirect sets
         * pc=RSA, then `s->pc += consumed` bumps it to RSA+1, and the first
         * body instruction is never re-executed (the PC histogram then shows
         * the body as [RSA+1..REA+1] instead of [RSA..REA]). */

        /* RPTB entry at 0x76FD: dump all AR values. */
        if (s->pc == 0x76FD) {
            static int rptb_entry_log = 0;
            if (rptb_entry_log < 30)
                C54_LOG("RPTB-ENTRY PC=0x76FD AR0=%04x AR1=%04x AR2=%04x AR3=%04x AR4=%04x AR5=%04x AR6=%04x AR7=%04x ARP=%d DP=%d BRC=%d SP=%04x",
                        s->ar[0], s->ar[1], s->ar[2], s->ar[3], s->ar[4], s->ar[5], s->ar[6], s->ar[7],
                        arp(s), dp(s), s->brc, s->sp);
            rptb_entry_log++;
        }
        if (s->pc == 0x03F0) {
            static int f3_log = 0;
            if (f3_log < 2) {
                C54_LOG("PC=0x03F0 op=0x%04x insn=%u SP=0x%04x IMR=0x%04x XPC=%d PMST=0x%04x",
                        prog_fetch(s, s->pc), s->insn_count, s->sp, s->imr, s->xpc, s->pmst);
                C54_LOG("  trail: %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x",
                        pc_ring[(pc_ring_idx-20)&255], pc_ring[(pc_ring_idx-19)&255],
                        pc_ring[(pc_ring_idx-18)&255], pc_ring[(pc_ring_idx-17)&255],
                        pc_ring[(pc_ring_idx-16)&255], pc_ring[(pc_ring_idx-15)&255],
                        pc_ring[(pc_ring_idx-14)&255], pc_ring[(pc_ring_idx-13)&255],
                        pc_ring[(pc_ring_idx-12)&255], pc_ring[(pc_ring_idx-11)&255],
                        pc_ring[(pc_ring_idx-10)&255], pc_ring[(pc_ring_idx-9)&255],
                        pc_ring[(pc_ring_idx-8)&255], pc_ring[(pc_ring_idx-7)&255],
                        pc_ring[(pc_ring_idx-6)&255], pc_ring[(pc_ring_idx-5)&255],
                        pc_ring[(pc_ring_idx-4)&255], pc_ring[(pc_ring_idx-3)&255],
                        pc_ring[(pc_ring_idx-2)&255], pc_ring[(pc_ring_idx-1)&255]);
                f3_log++;
            }
        }

        /* Boot trace. */
        if (g_boot_trace > 0) {
            C54_LOG("BOOT[%d] PC=0x%04x op=0x%04x SP=0x%04x PMST=0x%04x",
                    51 - g_boot_trace, s->pc, prog_fetch(s, s->pc), s->sp, s->pmst);
            g_boot_trace--;
        }

        /* Execute the instruction. */
        int consumed;
        uint16_t exec_pc = s->pc;
        /* FIRS-BANK (CALYPSO_FIRS_BANK, default OFF, read-only). Is the PROM0
         * polyphase filter bank entered through its prologue or in the middle?
         * Eight FIRS in four pairs (pmad 0x64, 0x63, 0x62, 0x61), preceded by
         * a prologue that loads AR2/AR5 <- 0x0060 and by a six-entry dispatch
         * table at 0x8359..0x8363. Measured over two runs: the first FIRS
         * executed is always 0x8478 (the last stage), and nothing writes the
         * buffer 0x0060..0x0066, which is the FIRS Xmem. Counting each site
         * separates "never reached" from "reached, then silent". */
        {
            static int _fb = -1;
            if (_fb < 0) {
                _fb = calypso_gate("CALYPSO_FIRS_BANK", 0);
                fprintf(stderr, "[c54x] FIRS-BANK %s : prologue (0x8336/0x834b/0x834f), "
                        "table de dispatch (0x8359..0x8363) et six etages du banc\n",
                        _fb ? "ACTIVE" : "INACTIVE (defaut)");
            }
            if (_fb) {
                static const uint16_t sites[] = {
                    0x8336, 0x834b, 0x834f,                       /* prologue */
                    0x8359, 0x835b, 0x835d, 0x835f, 0x8361, 0x8363, /* dispatch */
                    0x8365, 0x8394, 0x83c9, 0x83ff, 0x8435, 0x846b  /* stages   */
                };
                static const char *quoi[] = {
                    "PROLOGUE stm #0x60,AR2", "PROLOGUE stm #0x60,AR5", "PROLOGUE branche table",
                    "DISPATCH->0x8365", "DISPATCH->0x8394", "DISPATCH->0x83c9",
                    "DISPATCH->0x83ff", "DISPATCH->0x8435", "DISPATCH->0x846b",
                    "ETAGE 0x8365", "ETAGE 0x8394", "ETAGE 0x83c9",
                    "ETAGE 0x83ff", "ETAGE 0x8435", "ETAGE 0x846b"
                };
                static unsigned long long hits[15];
                static unsigned shown[15];
                for (int k = 0; k < 15; k++) {
                    if (exec_pc != sites[k]) continue;
                    hits[k]++;
                    if (shown[k] < 3) {
                        shown[k]++;
                        fprintf(stderr, "[c54x] FIRS-BANK 0x%04x %-24s #%llu "
                                "AR2=0x%04x AR3=0x%04x AR5=0x%04x AR6=0x%04x insn=%u\n",
                                sites[k], quoi[k], (unsigned long long)hits[k],
                                s->ar[2], s->ar[3], s->ar[5], s->ar[6], s->insn_count);
                    }
                    break;
                }
                {   /* periodic summary: which sites were reached, which never */
                    static unsigned long long tick = 0;
                    if ((++tick % 4000000) == 0) {
                        fprintf(stderr, "[c54x] FIRS-BANK bilan :");
                        for (int k = 0; k < 15; k++)
                            fprintf(stderr, " %04x=%llu", sites[k],
                                    (unsigned long long)hits[k]);
                        fprintf(stderr, "\n");
                    }
                }
            }
        }
        /* CORR-SLIDE (CALYPSO_CORR_SLIDE, default OFF, read-only). Does the
         * correlator window actually slide? Measured: only 3 to 4 distinct
         * outputs over 50 offsets while the burst buffer is live, i.e. the
         * outputs come in identical runs. Shows AR1..AR5 and the accumulators
         * at the three key points of each RPTB 0x84b0..0x84c6 iteration. */
        {
            static int _cs = -1;
            if (_cs < 0) {
                _cs = calypso_gate("CALYPSO_CORR_SLIDE", 0);
                fprintf(stderr, "[c54x] CORR-SLIDE %s : AR1..AR5 + accumulateurs en "
                        "0x84b4 (entree tour), 0x84bc (store A) et 0x84c5 (store B)\n",
                        _cs ? "ACTIVE" : "INACTIVE (defaut)");
            }
            if (_cs && (exec_pc == 0x84b4 || exec_pc == 0x84bc || exec_pc == 0x84c5)) {
                static unsigned _n = 0;
                if (_n < 24) {
                    _n++;
                    const char *ou = (exec_pc == 0x84b4) ? "entree tour" :
                                     (exec_pc == 0x84bc) ? "store A    " : "store B    ";
                    fprintf(stderr, "[c54x] CORR-SLIDE #%2u 0x%04x %s | AR0=%d "
                            "AR1=0x%04x AR2=0x%04x AR3=0x%04x AR4=0x%04x AR5=0x%04x "
                            "| A=0x%010llx B=0x%010llx BK=0x%04x insn=%u\n",
                            _n, exec_pc, ou, (int)(int16_t)s->ar[0],
                            s->ar[1], s->ar[2], s->ar[3], s->ar[4], s->ar[5],
                            (unsigned long long)(s->a & 0xFFFFFFFFFFULL),
                            (unsigned long long)(s->b & 0xFFFFFFFFFFULL),
                            s->bk, s->insn_count);
                }
            }
        }
        if (exec_pc == 0xb40e || exec_pc == 0xb40f) {
            static unsigned tn = 0;
            if (tn++ < 8)
                fprintf(stderr, "[c54x] TERMINAL-DISP PC=0x%04x AR7=0x%04x data[AR7]=0x%04x "
                        "data[0x4387]=0x%04x data[0x43c0]=0x%04x A=0x%06llx SP=0x%04x insn=%u\n",
                        exec_pc, s->ar[7], s->data[s->ar[7]],
                        s->data[0x4387], s->data[0x43c0],
                        (unsigned long long)(s->a & 0xFFFFFFULL), s->sp, s->insn_count);
        }
        /* OVLY-TRACE: the frame handler 0x013b..0x0160 (DARAM overlay) derails
         * at the RET in 0x0157 with an empty stack. Traces pc/op/sp of the
         * first pass and dumps the overlay contents, to locate the imbalance
         * (a PSHM with no matching POPM, or a branch wrongly taken before the
         * POPMs). One-shot, one frame. */
        if (exec_pc >= 0x0100 && exec_pc <= 0x0160) {
            static unsigned ot = 0; static int dumped = 0;
            if (!dumped) {
                dumped = 1;
                fprintf(stderr, "[c54x] OVLY-DUMP data[0x0100..0x0160]:");
                for (int a = 0x0100; a <= 0x0160; a++) fprintf(stderr, " %04x", s->data[a]);
                fprintf(stderr, "\n");
            }
            if (exec_pc == 0x0154) {
                /* @BEQUILLE - TEST_3FCD  (CALYPSO_TEST_3FCD, EXISTS, default OFF)
                 *   masks   : data[0x3fcd], the address the RET @0x0157 pops, is never
                 *             written; the firmware installs a handler in the neighbouring
                 *             cell data[0x3fce] (= 0xdf82). At the PSHD (0x0154) one is
                 *             derived from the other, which tests whether 0xdf82 is the
                 *             right target, i.e. whether the overlay vector table sits at
                 *             the wrong offset.
                 *   remove  : as soon as the overlay vector table is installed at the right
                 *             offset (data[0x3fcd] non-zero without forcing), or at once if
                 *             FIX_3FCD (same cell, PC 0x013b) is kept as the single
                 *             mechanism.
                 */
                static int t3 = -1;
                if (t3 < 0) t3 = calypso_gate("CALYPSO_TEST_3FCD", 0);
                if (t3 && s->data[0x3fcd] == 0 && s->data[0x3fce] != 0) {
                    static unsigned tn3 = 0;
                    if (tn3++ < 4)
                        fprintf(stderr, "[c54x] TEST-3FCD: data[0x3fcd] 0x0000 -> 0x%04x (=data[0x3fce]) insn=%u\n",
                                s->data[0x3fce], s->insn_count);
                    s->data[0x3fcd] = s->data[0x3fce];
                }
            }
            if (ot++ < 90)
                fprintf(stderr, "[c54x] OVLY-TRACE pc=0x%04x op=0x%04x sp=0x%04x A=0x%06llx insn=%u\n",
                        exec_pc, prog_fetch(s, exec_pc), s->sp,
                        (unsigned long long)(s->a & 0xFFFFFFULL), s->insn_count);
        }
        if (exec_pc == 0xa4e4) {
            /* @BEQUILLE - INIT_435B (+ SEED_52FD)  (CALYPSO_INIT_435B_OFF=0 => ACTIVE;
             *              CALYPSO_SEED_52FD picks the value; the four .env profiles set 0)
             *   masks   : the DSP boot's initialisation of the shadow IMR data[0x435b].
             *             Task handlers OR and AND bits into that cell (the correlator at
             *             0xbd3c does ORM 0x10, and so on) and the go-live state machine
             *             0xa501/0xa582 propagates it into IMR. Nothing ever writes it here
             *             (STATE435B-WR stays empty), so IMR ends up 0 and the machine
             *             deadlocks. Injects 0x52ed, or 0x52fd under SEED_52FD, at
             *             exec_pc == 0xa4e4.
             *   remove  : when a firmware write to 0x435b is observed before 0xa4e4.
             *   TRAP    : the name says _OFF, but "=0" ENABLES it.
             */
            static int i435 = -1;
            if (i435 < 0) { const char *_e435 = getenv("CALYPSO_INIT_435B_OFF"); i435 = (_e435 && atoi(_e435)) ? 0 : 1; }  /* gate tests the VALUE: OFF=0 means active */
            if (i435 && s->data[0x435b] == 0) {
                static unsigned in = 0;
                if (in++ < 4)
                    fprintf(stderr, "[c54x] INIT-435B: data[0x435b] 0x0000 -> 0x52ed (masque IMR reset SANS bit4/clobber) insn=%u\n", s->insn_count);
                s->data[0x435b] = getenv("CALYPSO_SEED_52FD") ? 0x52fd : 0x52ed;   /* default 0x52ed: without bit 4 (TINT), which avoids the firmware clobber at 0xa509 that strips bit 12 (frame). 0x52fd sets bit 4 and breaks the frame, which shows the firmware does not use TINT0 */
            }
        }
        /* SM-TRACE: full path of the go-live state machine 0xa4e4-0xa5b5 once
         * d_dsp_page is aligned - where does it branch or loop back? The
         * deciding values are d_dsp_page (0x3fb0), data[0x09bc] (ARM flag) and
         * A (the dispatch target). */
        if (exec_pc >= 0xa4e4 && exec_pc <= 0xa5b8) {
            static unsigned st = 0;
            if (st++ < 70)
                fprintf(stderr, "[c54x] SM-TRACE pc=0x%04x op=0x%04x A=0x%06llx TC=%d "
                        "d[3fb0]=%04x d[09bc]=%04x d[3fe0]=%04x d[435b]=%04x insn=%u\n",
                        exec_pc, prog_fetch(s, exec_pc), (unsigned long long)(s->a & 0xFFFFFFULL),
                        (s->st0 & ST0_TC) ? 1 : 0, s->data[0x3fb0], s->data[0x09bc],
                        s->data[0x3fe0], s->data[0x435b], s->insn_count);
        }
        /* TERM-TRACE: the AR7 index computation at the mask-ROM terminal
         * 0xb405-0xb412. Does AR7 end up 0x4387 (idle) instead of 0x43c0
         * (go-live), i.e. is the result of LD #0x39 (0xb408) plus ADD #0x4387
         * (0xb409) lost or mis-stored into AR7? Logs A and AR0..7 at every
         * instruction of the zone. Gate CALYPSO_TERM_TRACE_OFF. */
        {
            static int _tt = -1;
            if (_tt < 0) _tt = getenv("CALYPSO_TERM_TRACE_OFF") ? 0 : 1;
            if (_tt && exec_pc >= 0xb400 && exec_pc <= 0xb414) {
                static unsigned _ttn = 0;
                if (_ttn++ < 60)
                    fprintf(stderr, "[c54x] TERM-TRACE pc=0x%04x op=0x%04x A=0x%06llx "
                            "AR[0..7]=%04x %04x %04x %04x %04x %04x %04x %04x insn=%u\n",
                            exec_pc, prog_fetch(s, exec_pc), (unsigned long long)(s->a & 0xFFFFFFULL),
                            s->ar[0], s->ar[1], s->ar[2], s->ar[3], s->ar[4], s->ar[5], s->ar[6], s->ar[7],
                            s->insn_count);
            }
        }
        /* CALA-TRACE-WIDE: every computed transfer (CALA/CALAD/BACC/FBACC/
         * FCALA/FCALAD - f4e2/f4e3/f4e6/f4e7/f5e2/f5e3/f5e6/f5e7/f6e6/f6e7)
         * over the whole PROM0 range, not just CALA over fragments. The point
         * is to catch ANY computed jump landing in the correlator range
         * (CORR_PC_LO..CORR_PC_HI); three static-scan passes failed to find
         * the reference. Two counters: hits inside the range are never capped
         * (strong signal), general hits are capped at 200 for context. */
        {
            static int _ctw = -1;
            if (_ctw < 0) { const char *_e = getenv("CALYPSO_D247_TRACE_OFF"); _ctw = (_e && atoi(_e)) ? 0 : 1; }
            if (_ctw && exec_pc >= 0x7000 && exec_pc <= 0xdfff) {   /* all of PROM0, no arbitrary sub-range */
                uint16_t _cop = prog_fetch(s, exec_pc);
                bool _is_xfer = (_cop==0xf4e2||_cop==0xf4e3||_cop==0xf4e6||_cop==0xf4e7||
                                  _cop==0xf5e2||_cop==0xf5e3||_cop==0xf5e6||_cop==0xf5e7||
                                  _cop==0xf6e6||_cop==0xf6e7);
                if (_is_xfer) {
                    uint16_t _tgt = (uint16_t)(s->a & 0xFFFF);
                    bool _in_corr = (_tgt >= CORR_PC_LO && _tgt < CORR_PC_HI);
                    if (_in_corr) {
                        fprintf(stderr, "[c54x] CALA-WIDE *** DANS-CORRELATEUR *** pc=0x%04x op=0x%04x "
                                "-> target=0x%04x task_md p0(0804)=%04x p1(0818)=%04x d_dsp_page(08e2)=%04x "
                                "d[4357]=%04x insn=%u\n",
                                exec_pc, _cop, _tgt, s->data[0x0804], s->data[0x0818],
                                s->data[0x08e2], s->data[0x4357], s->insn_count);
                    } else {
                        static unsigned _ctwn = 0;
                        if (_ctwn++ < 200)
                            fprintf(stderr, "[c54x] CALA-WIDE pc=0x%04x op=0x%04x -> target=0x%04x insn=%u\n",
                                    exec_pc, _cop, _tgt, s->insn_count);
                    }
                }
            }
        }
        /* INSTALL-TRACE: the 0xc7xx block installs the task handler table
         * (STL A -> d[4c5c] at 0xc803), and d[4c5c] == 0 means the FB
         * correlator is never dispatched. Says whether the block is reached
         * and what A holds at 0xc803; the neighbouring literals d[4c5a] and
         * d[4c5d] confirm the block ran. Gate CALYPSO_INSTALL_TRACE_OFF. */
        {
            static int _it = -1;
            if (_it < 0) _it = getenv("CALYPSO_INSTALL_TRACE_OFF") ? 0 : 1;
            if (_it && (exec_pc==0xc7fa || exec_pc==0xc801 || exec_pc==0xc803 ||
                        exec_pc==0xc805 || exec_pc==0xc7e2 || exec_pc==0xc827)) {
                static unsigned _itn = 0;
                if (_itn++ < 30)
                    fprintf(stderr, "[c54x] INSTALL-TRACE pc=0x%04x A=0x%04x "
                            "d[4c5a]=%04x d[4c5c]=%04x d[4c5d]=%04x d[3f5e]=%04x insn=%u\n",
                            exec_pc, (uint16_t)(s->a & 0xFFFF),
                            s->data[0x4c5a], s->data[0x4c5c], s->data[0x4c5d],
                            s->data[0x3f5e], s->insn_count);
            }
        }
        /* BACC-C827-SRC: where does the jump to 0xc827 come from, the one that
         * skips the handler table install 0xc7a0-0xc825? Records prev_pc, its
         * opcode, A and the ARs whenever 0xc827 is entered without
         * fall-through (prev is neither 0xc825 nor 0xc826). Gate
         * CALYPSO_BACC_C827_OFF. */
        {
            static uint16_t _pp827 = 0;
            static int _bsc = -1;
            if (_bsc < 0) _bsc = getenv("CALYPSO_BACC_C827_OFF") ? 0 : 1;
            if (_bsc && exec_pc == 0xc827 && _pp827 != 0xc825 && _pp827 != 0xc826 && _pp827 != 0xc827) {
                static unsigned _bn = 0;
                if (_bn++ < 15)
                    fprintf(stderr, "[c54x] BACC-C827-SRC from=0x%04x op@from=0x%04x A=0x%04x "
                            "AR[0..7]=%04x %04x %04x %04x %04x %04x %04x %04x insn=%u\n",
                            _pp827, prog_fetch(s, _pp827), (uint16_t)(s->a & 0xFFFF),
                            s->ar[0], s->ar[1], s->ar[2], s->ar[3], s->ar[4], s->ar[5], s->ar[6], s->ar[7],
                            s->insn_count);
            }
            _pp827 = exec_pc;
        }

/* PHASE-SM: the d[3f70] state machine (go-live -> operational).
         * 0xddeb does LD d[0x098a] then BC on A==0, which resets phase to 0;
         * 0xde86 does LD d[0x098c]; 0xde9c does ST #2. d[0x098a] and d[0x098c]
         * are the ARM handshake cells: the ARM must set them non-zero for the
         * machine to reach d[3f70] = 2. Logs the decision points and the
         * values. Gate CALYPSO_PHASE_SM_OFF. */
        {
            static int _ps = -1;
            if (_ps < 0) _ps = getenv("CALYPSO_PHASE_SM_OFF") ? 0 : 1;
            if (_ps && (exec_pc==0xddeb || exec_pc==0xde86 || exec_pc==0xde97 ||
                        exec_pc==0xde9c || exec_pc==0xde8b || exec_pc==0xdea8 || exec_pc==0xdddb)) {
                static unsigned _psn = 0;
                if (_psn++ < 40)
                    fprintf(stderr, "[c54x] PHASE-SM pc=0x%04x A=0x%04x d[3f70]=%04x "
                            "d[098a]=%04x d[098b]=%04x d[098c]=%04x d[098d]=%04x d[0fff]=%04x insn=%u\n",
                            exec_pc, (uint16_t)(s->a & 0xFFFF), s->data[0x3f70],
                            s->data[0x098a], s->data[0x098b], s->data[0x098c], s->data[0x098d],
                            s->data[0x0fff], s->insn_count);
                  /* Effective address of the LD. Direct addressing on the
                   * C54x is DP-relative: dma = (DP << 7) | offset7. At 0xde86
                   * the cell d[0x098c] holds 1 (seeded by BGEN) and yet A
                   * comes out 0x0000, 38 times in a row. Printing DP, the
                   * opcode and the computed address decides between "DP points
                   * at another page" and "right page, empty cell". Read-only,
                   * no behaviour change. */
                  {
                      uint16_t _op  = prog_fetch(s, exec_pc);
                      uint16_t _lk  = prog_fetch(s, (uint16_t)(exec_pc + 1));
                      int      _ind = (_op & 0x80) ? 1 : 0;
                      int      _mod = (_op >> 3) & 0x0F;
                      /* mod 0xF = *(lk): the address IS the long word. lk is
                       * printed for every other mode too - it costs nothing and
                       * avoids guessing again. */
                      static unsigned _ean = 0;
                      /* Cap: without it this probe floods the log (480898
                       * lines measured) and the truncation wipes the others.
                       * 40 lines, then one every 100000. */
                      _ean++;
                      if (_ean <= 40 || (_ean % 100000) == 0)
                      fprintf(stderr, "[c54x] PHASE-SM-EA #%u pc=0x%04x op=0x%04x ind=%d mod=0x%x "
                              "lk=0x%04x d[lk]=0x%04x AR0=0x%04x A=0x%04x insn=%u\n",
                              _ean, exec_pc, _op, _ind, _mod, _lk, s->data[_lk],
                              s->ar[0], (uint16_t)(s->a & 0xFFFF), s->insn_count);
                  }
            }
        }
        if (exec_pc == 0xa51c) {   /* go-live SM reads d_dsp_page @0x08d4 */
            static unsigned dp=0;
            if (dp++ < 12)
                fprintf(stderr, "[c54x] SM-DPAGE @0xa51c data[0x08d4]=0x%04x data[0x08E2]=0x%04x "
                        "data[0x435b]=0x%04x A=0x%06llx insn=%u\n",
                        s->data[0x08d4], s->data[0x08E2], s->data[0x435b],
                        (unsigned long long)(s->a & 0xFFFFFFULL), s->insn_count);
        }
        if (exec_pc == 0xa4cd) {   /* BC AEQ (0xf845): why is A == 0, which skips the RSBX INTM at 0xa4d0?
                                    * 0xaad5 reads AR0=data[0x434e], AR1=data[0x434f] and computes A. */
            static unsigned an = 0;
            if (an++ < 16)
                fprintf(stderr, "[c54x] A4CD-BC A=0x%06llx (AEQ %s -> %s) "
                        "d[434e]=%04x d[434f]=%04x d[*434e]=%04x d[*434f]=%04x d[3f70]=%04x insn=%u\n",
                        (unsigned long long)(s->a & 0xFFFFFFULL),
                        (s->a & 0xFFFFFFFFFFULL) == 0 ? "vrai" : "faux",
                        (s->a & 0xFFFFFFFFFFULL) == 0 ? "BRANCHE(skip enable)" : "fall-through(RSBX INTM)",
                        s->data[0x434e], s->data[0x434f],
                        s->data[s->data[0x434e] & 0xFFFF], s->data[s->data[0x434f] & 0xFFFF],
                        s->data[0x3f70], s->insn_count);
        }
        if (exec_pc == 0xb40f) {
            /* @BEQUILLE - MASKROM_GOLIVE  (CALYPSO_MASKROM_GOLIVE, EXISTS, default OFF)
             *   masks   : the launch vector mem[0x5ac8] at the stack base, pre-loaded on
             *             silicon by a mask-ROM absent from the dump. At 0, the RET of the
             *             idle BACC jumps to PC=0 (storm). The value is derived, not
             *             invented: mem[0x5ac8] = data[0x43c0], the go-live pointer the
             *             firmware itself wrote (0xa4c7 = ORM #0x3000,IMR, which arms the
             *             IMR).
             *   remove  : ALREADY REDUNDANT - the LD #k8u decode fix kills the storm
             *             natively (the stray << 16 set AR7 to 0x4387, idle, instead of
             *             0x43c0, go-live). This is now only an A/B guard; drop it.
             */
            static int mrg = -1;
            if (mrg < 0) mrg = calypso_gate("CALYPSO_MASKROM_GOLIVE", 0);
            if (mrg && s->data[0x5ac8] == 0 && s->data[0x43c0] != 0) {
                static unsigned mgn = 0;
                if (mgn++ < 4)
                    fprintf(stderr, "[c54x] MASKROM-GOLIVE: mem[0x5ac8] 0x0000 -> 0x%04x "
                            "(=data[0x43c0], pointeur go-live firmware) insn=%u\n",
                            s->data[0x43c0], s->insn_count);
                s->data[0x5ac8] = s->data[0x43c0];
            }
        }
        uint16_t exec_op = prog_fetch(s, s->pc);
        /* Repeat state BEFORE execution: tells "the repeat was already
         * active" from "the instruction just executed IS the RPT that armed
         * it". Consumed by the RPT block at the end of the loop. */
        bool rpt_was_active = s->rpt_active;
        /* FBWATCH-ALIVE canary: proves the probe family is armed, and samples
         * the foreground PC. If this line appears, g_fbwatch_on is 1 and the
         * silence of the other FBWATCH probes is real; if it does not, they
         * were simply dead. Fires every ~20M insns (~10 lines per run). */
        if (g_fbwatch_on > 0 && (s->insn_count % 20000000u) == 0) {
            fprintf(stderr, "[c54x] FBWATCH-ALIVE insn=%u PC=0x%04x INTM=%d SP=0x%04x\n",
                    s->insn_count, exec_pc, !!(s->st1 & ST1_INTM), s->sp);
        }
        /* FBWATCH-POLL: the foreground polls a flag through BITF at 0xf7af /
         * 0xf7b7 (RC NTC loops while TC == 0). Captures the flag address
         * (AR0..AR2 plus the data they point at) and TC, to identify the bit
         * that is never set, i.e. the hardware still to be modelled. */
        if (g_fbwatch_on > 0 && (exec_pc == 0xf7af || exec_pc == 0xf7b7)) {
            static unsigned wpoll = 0;
            if (wpoll++ < 30) {
                uint16_t mask = prog_fetch(s, exec_pc + 1);
                fprintf(stderr, "[c54x] FBWATCH-POLL pc=0x%04x mask=0x%04x | "
                        "AR0=0x%04x d=0x%04x | AR1=0x%04x d=0x%04x | AR2=0x%04x d=0x%04x | TC=%d insn=%u\n",
                        exec_pc, mask,
                        s->ar[0], s->data[s->ar[0]], s->ar[1], s->data[s->ar[1]],
                        s->ar[2], s->data[s->ar[2]], !!(s->st0 & ST0_TC), s->insn_count);
            }
        }
        /* FBWATCH: does the FB handler at 0x9ac0 run at all? */
        if (g_fbwatch_on > 0 && exec_pc == 0x9ac0) {
            static unsigned w9 = 0;
            if (w9++ < 40)
                fprintf(stderr, "[c54x] FBWATCH-9AC0 #%u insn=%u SP=0x%04x DP=0x%03x\n",
                        w9, s->insn_count, s->sp, s->st0 & 0x1FF);
        }
        /* FBWATCH-INITTAB: does the dispatch table init routine run (0xc704,
         * which fills data[0x4c24-0x4c5d], the BACC-A/CALA targets)? Zero hits
         * means the boot skips the setup pass entirely. */
        if (g_fbwatch_on > 0 && (exec_pc == 0xc704 || exec_pc == 0xc472)) {
            static unsigned wit = 0;
            if (wit++ < 10)
                fprintf(stderr, "[c54x] FBWATCH-INITTAB pc=0x%04x insn=%u SP=0x%04x\n",
                        exec_pc, s->insn_count, s->sp);
        }
        /* FBWATCH-DISP, producer versus consumer: does the CALAD dispatch at
         * 0x833b run once per frame, and which handler address does it compute
         * into A? No line at all means a dead dispatcher (producer side);
         * A never equal to 0x9ac0 means the jump table or formula never yields
         * the FB handler; A == 0x9ac0 means FB is dispatched but does not
         * detect (handler bug). High cap, to see the distribution. */
        if (g_fbwatch_on > 0 && exec_pc == 0x833b) {
            static unsigned wdp = 0;
            if (wdp++ < 120)
                fprintf(stderr, "[c54x] FBWATCH-DISP #%u insn=%u A_handler=0x%04x DP=0x%03x SP=0x%04x\n",
                        wdp, s->insn_count, (uint16_t)(s->a & 0xffff), s->st0 & 0x1FF, s->sp);
        }
        /* CORR-ENTRY tracker (CALYPSO_CORRELATOR_TRACE=1): captures the
         * out -> in transition of the FB-det range [0x8d00..0x9000). */
        corr_entry_track(s->pc, s);
        /* FBDB-PROBE (CALYPSO_FBDB_PROBE=1): traces B at 0xfbd9, A at 0xfbdb
         * (after the F2xx SUB) and A at 0xfbf3 (before STLM A,AR4). */
        fbdb_probe_check_pc(s->pc, s);
        /* FORCE-INTM-ONESHOT (CALYPSO_FORCE_INTM_ONESHOT=1): arbitration
         * probe - clears INTM ONCE while INTM=1 and BRINT0 is pending, so the
         * existing tracers show whether everything downstream is healthy. */
        force_intm_oneshot_check(s);
        /* STUCK-PROBE (CALYPSO_STUCK_PROBE=1): PC and XPC histogram while
         * INTM=1 and BRINT0 is pending. */
        stuck_probe_check(s);

        /* CALA-70C3 forensic probes. The DSP loops for ever on CALA A at
         * PROM0[0x70c3] with A = 0x0001_70c3, a self-reference. A_H = 0x0001
         * cannot come from a sign-extended `LD Smem,A`, which yields A_H in
         * {0x0000, 0xFFFF}, so the writer is an upstream DLD or something that
         * composes H and L. Three probes:
         *   1. source of the jump to 0x70c3 (XPC:PC plus the opcode at
         *      prev_pc), first-hit only to escape post-runaway pollution (the
         *      XPC MMR is overwritten once SP rampages through
         *      data[0x18..0x1F]);
         *   2. a counter on LD@0x70c1 - zero confirms a direct jump that skips
         *      the LD;
         *   3. the last writer of A (the PC that put 0x0001_70c3 there).
         * On by default, about 3 branches per instruction. */
        static int      p70c3_first    = 0;
        static uint64_t p70c1_counter  = 0;
        static uint16_t p_last_a_pc    = 0xFFFF;
        static int64_t  p_last_a_val   = 0;
        int64_t a_before_exec = s->a;

        if (s->pc == 0x70c1) p70c1_counter++;

        if (s->pc == 0x70c3 && !p70c3_first) {
            p70c3_first = 1;
            uint16_t prev_pc = pc_ring[(pc_ring_idx - 2) & 255];
            uint16_t prev_op = prog_fetch(s, prev_pc);
            C54_LOG("PROBE-CALA70C3-FIRST insn=%u XPC=%u PC=0x%04x op=0x%04x "
                    "prev_pc=0x%04x prev_op=0x%04x "
                    "A=%010llx (A_G=0x%02x A_H=0x%04x A_L=0x%04x) "
                    "LD@70C1_count=%llu last_A_writer_pc=0x%04x last_A_val=%010llx "
                    "SP=0x%04x BK=0x%04x",
                    s->insn_count, s->xpc & 0x3, s->pc, prog_fetch(s, s->pc),
                    prev_pc, prev_op,
                    (unsigned long long)(s->a & 0xFFFFFFFFFFULL),
                    (uint8_t)((s->a >> 32) & 0xFF),
                    (uint16_t)((s->a >> 16) & 0xFFFF),
                    (uint16_t)(s->a & 0xFFFF),
                    (unsigned long long)p70c1_counter,
                    p_last_a_pc,
                    (unsigned long long)(p_last_a_val & 0xFFFFFFFFFFULL),
                    s->sp, s->bk);
            C54_LOG("PROBE-CALA70C3-TRAIL pc[-12..-1] = "
                    "%04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x",
                    pc_ring[(pc_ring_idx-13)&255], pc_ring[(pc_ring_idx-12)&255],
                    pc_ring[(pc_ring_idx-11)&255], pc_ring[(pc_ring_idx-10)&255],
                    pc_ring[(pc_ring_idx-9)&255],  pc_ring[(pc_ring_idx-8)&255],
                    pc_ring[(pc_ring_idx-7)&255],  pc_ring[(pc_ring_idx-6)&255],
                    pc_ring[(pc_ring_idx-5)&255],  pc_ring[(pc_ring_idx-4)&255],
                    pc_ring[(pc_ring_idx-3)&255],  pc_ring[(pc_ring_idx-2)&255]);
        }

        {
            /* DISP-ENTRY: predecessor = the PC executed on the previous
             * iteration. */
            static uint16_t s_last_run_pc = 0;
            static uint16_t s_last_run_op = 0;
            g_prev_pc = s_last_run_pc;
            g_prev_op = s_last_run_op;
            s_last_run_pc = s->pc;
            s_last_run_op = exec_op;
        }

        /* AR3-TRIP: catches the FIRST instruction that moves AR3 by a large
         * step. At this point s->ar[3] holds the result of the instruction
         * that just ran, i.e. g_prev_pc / g_prev_op. Legitimate
         * post-increments are +-1 or +-2, so a jump of 0x800 or more is a
         * suspect load or modification. delta == SP is flagged separately: it
         * is the "AR3 += SP" signature behind the derail. One subtract and one
         * compare per instruction, 80 lines. */
        {
            static uint16_t s_prev_ar3 = 0;
            static unsigned s_ar3trip = 0;
            uint16_t cur_ar3 = s->ar[3];
            uint16_t d = (uint16_t)(cur_ar3 - s_prev_ar3);
            uint16_t mag = (d & 0x8000) ? (uint16_t)(-d) : d;
            if (mag >= 0x0800 && s_ar3trip < 80) {
                s_ar3trip++;
                fprintf(stderr, "[c54x] AR3-TRIP #%u by PC=0x%04x op=0x%04x : "
                        "AR3 0x%04x -> 0x%04x (d=%+d) SP=0x%04x%s XPC=%u insn=%u\n",
                        s_ar3trip, g_prev_pc, g_prev_op,
                        s_prev_ar3, cur_ar3, (int16_t)d, s->sp,
                        (d == s->sp && s->sp != 0) ? "  <== delta==SP!" : "",
                        s->xpc, s->insn_count);
            }
            s_prev_ar3 = cur_ar3;
        }

        /* AR0-TRACE: AR0 is the index of the *AR3+0% at 0xb3d1 (AR3 += AR0),
         * and it is measured at about 0x5AC7, i.e. the value of SP - corrupt.
         * Logs EVERY change of AR0 during the init window (insn < 3000) with
         * the instruction that wrote it (g_prev_pc/op), which names the
         * corrupter. */
        {
            static uint16_t s_prev_ar0 = 0xFFFF;
            static unsigned s_ar0n = 0;
            if (s->ar[0] != s_prev_ar0 && s->insn_count < 3000 && s_ar0n < 60) {
                s_ar0n++;
                fprintf(stderr, "[c54x] AR0-TRACE #%u by PC=0x%04x op=0x%04x : "
                        "AR0 0x%04x -> 0x%04x BK=0x%04x SP=0x%04x DP=0x%03x insn=%u\n",
                        s_ar0n, g_prev_pc, g_prev_op, s_prev_ar0, s->ar[0],
                        s->bk, s->sp, (s->st0 & 0x1FF), s->insn_count);
            }
            s_prev_ar0 = s->ar[0];
        }

        /* One-shot dump of the full context at the first pass through 0xb3d1
         * (ADD *AR3+0%,A): AR0, AR3, BK, DP, ST1, A and data[AR3]. */
        if (s->pc == 0x3d1 + 0xb000) {
            static unsigned s_b3d1 = 0;
            if (s_b3d1 < 6) {
                s_b3d1++;
                uint16_t ea = s->ar[3];
                fprintf(stderr, "[c54x] B3D1-CTX #%u AR0=0x%04x AR3=0x%04x BK=0x%04x "
                        "DP=0x%03x ST1=0x%04x A=0x%010llx data[AR3]=0x%04x SP=0x%04x insn=%u\n",
                        s_b3d1, s->ar[0], s->ar[3], s->bk, (s->st0 & 0x1FF),
                        s->st1, (unsigned long long)(s->a & 0xFFFFFFFFFFULL),
                        s->data[ea], s->sp, s->insn_count);
            }
        }

        /* DETECTOR-TRACE: instruction-by-instruction trace of the FB detector
         * / correlator in [0xf074..0xf0c0]. In the ROM dump 0xf070-0xf0b0 is a
         * sigmoid table under XPC=0; the real execution is in another XPC
         * bank, and exec_op = prog_fetch(s->pc) honours the bank, so the real
         * opcode is shown. Logs PC, XPC (which bank), the opcode and A/B/T,
         * which carry the correlation arithmetic (accumulated sample*coeff).
         * A and B staying 0 means the arithmetic produces nothing (opcode
         * mis-emulated, or the sample is never read); A and B rising while
         * d_fb_det stays 0 means a threshold or decision bug - cross-check
         * with CALYPSO_FBDET_SENTINEL=2, which watches the writes to 0x08f8.
         * CALYPSO_DETTRACE=1, 800 lines. */
        {
            static int dettr_on = -1;
            if (dettr_on < 0) dettr_on = calypso_gate("CALYPSO_DETTRACE", 0);
            if (dettr_on && exec_pc >= 0xf074 && exec_pc <= 0xf0c0) {
                static unsigned dettr_n = 0;
                if (dettr_n < 800) {
                    dettr_n++;
                    fprintf(stderr, "[c54x] DETTRACE #%u PC=0x%04x XPC=%u op=0x%04x "
                            "A=0x%010llx B=0x%010llx T=0x%04x AR2=%04x AR3=%04x "
                            "AR5=%04x insn=%u\n",
                            dettr_n, exec_pc, s->xpc, exec_op,
                            (unsigned long long)(s->a & 0xFFFFFFFFFFULL),
                            (unsigned long long)(s->b & 0xFFFFFFFFFFULL),
                            s->t, s->ar[2], s->ar[3], s->ar[5], s->insn_count);
                    if (dettr_n == 800)
                        fprintf(stderr, "[c54x] DETTRACE capped at 800\n");
                }
            }
        }

        /* BACC-DISP: at the BACC dispatch (0xb40f) and the LD *AR7,A (0xb40e)
         * before it, captures A, AR7 and data[AR7], the source of the 0xf074
         * pointer. Tells whether the slot read (data[AR7]) already holds
         * 0xf074 (a wrong pointer) or A is corrupt some other way. */
        if (exec_pc == 0xb40f) {   /* BACC A = dispatch handler */
            uint16_t handler = (uint16_t)(s->a & 0xFFFF);
            static uint16_t seen[96]; static unsigned nseen = 0;
            bool dup = false;
            for (unsigned i = 0; i < nseen; i++)
                if (seen[i] == handler) { dup = true; break; }
            if (!dup && nseen < 96) {
                seen[nseen++] = handler;
                fprintf(stderr, "[c54x] BACC-DISP #%u handler=0x%04x AR7=0x%04x %sXPC=%u insn=%u\n",
                        nseen, handler, s->ar[7],
                        (handler == 0xab38) ? "(RET/noop) " : "<<REAL>> ",
                        s->xpc, s->insn_count);
            }
        }

        /* LOOPTRACE: the WHOLE main loop [0xb3a0..0xb410] - the prologue that
         * reads d_dsp_page at 0xb3cc plus the dispatcher at 0xb400 - together
         * with the caller (g_prev_pc). Shows how 0xb401 is reached the first
         * time, whether word[0] = 0x2900 at 0xb400 is executed or skipped, and
         * why data[SP] (the seeded return address) is empty when the idle RET
         * pops it and lands on PC=0. Measured: POST-BOOTSTUB-RET runs 640M
         * times (the PC=0 storm) from the very first dispatch, at insn 4398.
         * AR1 is included because the READA copy arms AR1=0x4387. 300 lines. */
        if (exec_pc >= 0xb3a0 && exec_pc <= 0xb410) {
            static unsigned de = 0;
            if (de < 300) {
                de++;
                fprintf(stderr, "[c54x] LOOPTRACE #%u from=0x%04x PC=0x%04x op=0x%04x "
                        "A=0x%06llx AR1=%04x AR7=%04x SP=%04x "
                        "stk[SP-1,SP,SP+1]=%04x,%04x,%04x insn=%u\n",
                        de, g_prev_pc, exec_pc, exec_op,
                        (unsigned long long)(s->a & 0xFFFFFF),
                        s->ar[1], s->ar[7], s->sp,
                        s->data[(uint16_t)(s->sp-1)], s->data[s->sp],
                        s->data[(uint16_t)(s->sp+1)], s->insn_count);
            }
        }

        {
            /* @BEQUILLE - SEED_5AC8 (+ SEED5AC8_VAL)  (CALYPSO_SEED5AC8, atoi>0, default
             *              OFF; _VAL defaults to 0x71f4, calypso_wire.env sets 0xa4c7)
             *   masks   : the population of mem[0x5ac8], the word the terminal RET (0xab38)
             *             pops to choose the go-live entry. Nothing writes it in this model,
             *             so the RET lands on PC=0 and storms.
             *   remove  : when it is known WHO writes mem[0x5ac8] on silicon. This is not a
             *             fix: in the ROM (PROM0.bin, little-endian) the boot at 0xb405 does
             *             ST #0xa4df,data[0x3f6d], so the go-live soft-vector is 0xa4df,
             *             which SKIPS both the IMR arming (0xa4c7 ORM #0x3000,IMR) and the
             *             enable (0xa4d0 RSBX INTM). Even a "correct" mem[0x5ac8] = 0x71f4
             *             therefore never arms the IMR; the real cause of the storm and of
             *             the missing enable is still open.
             */
            static int seed_on = -1;
            if (seed_on < 0) { const char *e = getenv("CALYPSO_SEED5AC8"); seed_on = (e && atoi(e) > 0) ? 1 : 0; }
            /* Wired to the DSP's own STM #0x5ac8,SP (PC=0xb382, op=0x7718),
             * so the seed follows the real stack init rather than an arbitrary
             * poke at the terminal BACC. Timing is safe: nothing writes
             * 0x5ac8 between 0xb382 and the RET at 0xab38. */
            if (seed_on && exec_pc == 0xb382) {
                /* Seed value, CALYPSO_SEED5AC8_VAL. The terminal RET pops
                 * mem[0x5ac8] to choose the go-live entry: 0x71f4 (default)
                 * goes through the trampoline to 0xa4df, which skips the
                 * RSBX INTM enable at 0xa4d0; 0xa4c7 enters at the ORM IMR, so
                 * the native RSBX INTM runs and the frame interrupt (delivered
                 * on bit 12) is actually taken. */
                static int sval = -1;
                if (sval < 0) { const char *e = getenv("CALYPSO_SEED5AC8_VAL");
                                sval = (e && *e) ? (int)strtoul(e, NULL, 0) : 0x71f4; }
                static unsigned sd = 0;
                if (sd < 8)
                    fprintf(stderr, "[c54x] SEED-5AC8 (cable@0xb382 STM SP) #%u : "
                            "mem[0x5ac8] 0x%04x->0x%04x SP=0x%04x op=0x%04x insn=%u\n",
                            ++sd, s->data[0x5ac8], (unsigned)sval, s->sp, exec_op, s->insn_count);
                s->data[0x5ac8] = (uint16_t)sval;
            }
        }
        /* GOLIVE-WATCH (ungated): does the firmware ever reach the go-live
         * routine (0xa4c9..0xa520) or the trampoline 0x71f4? Logs PC, opcode,
         * INTM, IMR and the soft-vector data[0x3f6d]. 120 lines. */
        if ((exec_pc >= 0xa4c9 && exec_pc <= 0xa520) || exec_pc == 0x71f4 || exec_pc == 0x71f6) {
            static unsigned gw = 0;
            if (gw++ < 120)
                fprintf(stderr, "[c54x] GOLIVE-WATCH #%u PC=0x%04x op=0x%04x INTM=%d "
                        "IMR=0x%04x data[0x3f6d]=0x%04x A=0x%06llx insn=%u\n",
                        gw, exec_pc, exec_op, (s->st1 & ST1_INTM) ? 1 : 0, s->imr,
                        s->data[0x3f6d], (unsigned long long)(s->a & 0xFFFFFF), s->insn_count);
        }
        /* AR0-DELTA (CALYPSO_AR0_DEBUG, read-only): every change of AR0 in the
         * boot window, which locates the instruction that corrupts it. AR0 is
         * expected near 0x5ac8, to write the go-live vector
         * mem[0x5ac8] = 0x71f4. */
        {
            static int ad_en = -1;
            static uint16_t ar0_prev = 0xFFFF;
            if (ad_en < 0) ad_en = calypso_gate("CALYPSO_AR0_DEBUG", 0);
            if (ad_en && s->insn_count < 12000 && s->ar[0] != ar0_prev
                && exec_pc != 0xb387) {   /* skip the fill loop, which floods the cap */
                static unsigned adn = 0;
                if (adn++ < 200)
                    fprintf(stderr, "[c54x] AR0-DELTA 0x%04x->0x%04x by PC=0x%04x "
                            "op=0x%04x AR3=0x%04x insn=%u\n",
                            ar0_prev, s->ar[0], exec_pc, exec_op, s->ar[3], s->insn_count);
                ar0_prev = s->ar[0];
            }
        }
        /* PROG-DUMP-B3D0 (CALYPSO_AR0_DEBUG, read-only, one-shot): dumps the
         * region that seeds data[0x3f6d] = 0xa4df (at 0xb405) and the terminal
         * BACC 0xb40f, to find the companion setup of mem[0x5ac8]. */
        if (getenv("CALYPSO_AR0_DEBUG") && exec_pc == 0xb405) {
            static int done = 0;
            if (!done) {
                done = 1;
                fprintf(stderr, "[c54x] PROG-DUMP @0xb405 SP=0x%04x AR0=0x%04x AR1=0x%04x "
                        "AR3=0x%04x data[0x5ac8]=0x%04x data[0x3f6d]=0x%04x data[0x4387]=0x%04x\n",
                        s->sp, s->ar[0], s->ar[1], s->ar[3],
                        s->data[0x5ac8], s->data[0x3f6d], s->data[0x4387]);
                for (uint16_t a = 0xb3d0; a <= 0xb414; a += 4)
                    fprintf(stderr, "[c54x] PROG[0x%04x..]= %04x %04x %04x %04x\n",
                            a, s->prog[a], s->prog[(uint16_t)(a+1)],
                            s->prog[(uint16_t)(a+2)], s->prog[(uint16_t)(a+3)]);
                /* the RET at 0xab38, target of the BACC */
                fprintf(stderr, "[c54x] PROG[0xab36..]= %04x %04x %04x %04x\n",
                        s->prog[0xab36], s->prog[0xab37], s->prog[0xab38], s->prog[0xab39]);
                /* Call site 0x71f2 (transfers to 0xb3a3 without pushing) plus
                 * the trampoline 0x71f4. It should be a CALL that pushes
                 * 0x71f4. */
                for (uint16_t a = 0x71ec; a <= 0x71f8; a += 4)
                    fprintf(stderr, "[c54x] PROG[0x%04x..]= %04x %04x %04x %04x\n",
                            a, s->prog[a], s->prog[(uint16_t)(a+1)],
                            s->prog[(uint16_t)(a+2)], s->prog[(uint16_t)(a+3)]);
                /* start of the routine at 0xb3a3, target of the transfer */
                fprintf(stderr, "[c54x] PROG[0xb3a0..]= %04x %04x %04x %04x %04x %04x\n",
                        s->prog[0xb3a0], s->prog[0xb3a1], s->prog[0xb3a2],
                        s->prog[0xb3a3], s->prog[0xb3a4], s->prog[0xb3a5]);
                /* the fill: setup 0xb384-0xb38c (STM AR0, RPT #k, ST #imm *AR0+) */
                fprintf(stderr, "[c54x] PROG[0xb384..]= %04x %04x %04x %04x %04x %04x %04x %04x %04x\n",
                        s->prog[0xb384], s->prog[0xb385], s->prog[0xb386], s->prog[0xb387],
                        s->prog[0xb388], s->prog[0xb389], s->prog[0xb38a], s->prog[0xb38b], s->prog[0xb38c]);
                /* result of the fill in memory: the constant and where it stops */
                fprintf(stderr, "[c54x] FILL-MEM data[0x5a00]=0x%04x [0x5ac5]=0x%04x [0x5ac6]=0x%04x "
                        "[0x5ac7]=0x%04x [0x5ac8]=0x%04x [0x5ac9]=0x%04x\n",
                        s->data[0x5a00], s->data[0x5ac5], s->data[0x5ac6],
                        s->data[0x5ac7], s->data[0x5ac8], s->data[0x5ac9]);
                /* vector store loop (0xb4c8-0xb4e0) and its AR setup */
                /* routine 0xa9ea (CALL at 0xb3f3, pushes 0xb3f5): locate its
                 * over-pop (PSHM/POPM imbalance) that drifts SP into the
                 * storm */
                for (uint16_t a = 0xa9ea; a <= 0xaa1a; a += 4)
                    fprintf(stderr, "[c54x] A9EA-PROG[0x%04x..]= %04x %04x %04x %04x\n",
                            a, s->prog[a], s->prog[(uint16_t)(a+1)],
                            s->prog[(uint16_t)(a+2)], s->prog[(uint16_t)(a+3)]);
                for (uint16_t a = 0xb4c8; a <= 0xb4e0; a += 4)
                    fprintf(stderr, "[c54x] VECLOOP-PROG[0x%04x..]= %04x %04x %04x %04x\n",
                            a, s->prog[a], s->prog[(uint16_t)(a+1)],
                            s->prog[(uint16_t)(a+2)], s->prog[(uint16_t)(a+3)]);
                /* go-live tail after 0xa582: does it install vec28, i.e. write
                 * 0x00f0? */
                for (uint16_t a = 0xa582; a <= 0xa5a2; a += 4)
                    fprintf(stderr, "[c54x] GOTAIL-PROG[0x%04x..]= %04x %04x %04x %04x\n",
                            a, s->prog[a], s->prog[(uint16_t)(a+1)],
                            s->prog[(uint16_t)(a+2)], s->prog[(uint16_t)(a+3)]);
                fprintf(stderr, "[c54x] VEC28-NOW data[0x00f0..0x00f3]= %04x %04x %04x %04x (doit brancher vers 0x7234)\n",
                        s->data[0x00f0], s->data[0x00f1], s->data[0x00f2], s->data[0x00f3]);
                for (uint16_t a = 0xb360; a <= 0xb384; a += 4)
                    fprintf(stderr, "[c54x] HANDLER-B360[0x%04x..]= %04x %04x %04x %04x\n",
                            a, s->prog[a], s->prog[(uint16_t)(a+1)],
                            s->prog[(uint16_t)(a+2)], s->prog[(uint16_t)(a+3)]);
                for (uint16_t a = 0x701c; a <= 0x7024; a += 4)
                    fprintf(stderr, "[c54x] RET701F-PROG[0x%04x..]= %04x %04x %04x %04x\n",
                            a, s->prog[a], s->prog[(uint16_t)(a+1)],
                            s->prog[(uint16_t)(a+2)], s->prog[(uint16_t)(a+3)]);
                for (uint16_t a = 0xa670; a <= 0xa680; a += 4)
                    fprintf(stderr, "[c54x] CALA671-PROG[0x%04x..]= %04x %04x %04x %04x\n",
                            a, s->prog[a], s->prog[(uint16_t)(a+1)],
                            s->prog[(uint16_t)(a+2)], s->prog[(uint16_t)(a+3)]);
                for (uint16_t a = 0xa4c0; a <= 0xa4e4; a += 4)
                    fprintf(stderr, "[c54x] GOLIVE-PROG[0x%04x..]= %04x %04x %04x %04x\n",
                            a, s->prog[a], s->prog[(uint16_t)(a+1)],
                            s->prog[(uint16_t)(a+2)], s->prog[(uint16_t)(a+3)]);
                fprintf(stderr, "[c54x] CTRL-CELLS data[0x098a]=0x%04x data[0x098c]=0x%04x "
                        "data[0x3f70]=0x%04x api[0x098a]=0x%04x\n",
                        s->data[0x098a], s->data[0x098c], s->data[0x3f70],
                        s->api_ram ? s->api_ram[0x098a-0x0800] : 0xffff);
            }
        }
        /* RUNTIME-DYN (read-only): dynamics of the state machine guarding the
         * scheduler at 0xa51c, which never runs (data[0x3fb0] == 0). Three
         * questions:
         *   (1) IMR-DYN   : does the DSP execute its IMR arming sites, and who
         *                   clears the IMR afterwards?
         *   (2) WAIT-TEST : is bit 1 of 0x3f70 (the wait-loop exit) ever set at
         *                   the test?
         *   (3) DE-BR     : does the super-loop reach the set-bit-1 branch at
         *                   0xde9c? */
        if (exec_pc == 0x76fc || exec_pc == 0xa509 || exec_pc == 0xb37e) {
            static unsigned id = 0;
            if (id++ < 40)
                fprintf(stderr, "[c54x] IMR-DYN PC=0x%04x IMR_before=0x%04x writes=0x%04x "
                        "INTM=%d insn=%u\n", exec_pc, s->imr,
                        s->prog[(uint16_t)(exec_pc + 1)],
                        (s->st1 & ST1_INTM) ? 1 : 0, s->insn_count);
        }
        {
            /* @BEQUILLE - KEEP_IMR (+ KEEP_IMR_VAL)  (CALYPSO_KEEP_IMR, EXISTS, default 1
             *              in hack/native/native_helped/wire; fallback value 0x52fd)
             *   masks   : the IMR clobber by 0xb37e (STM #0,IMR), which fires ~47 insns
             *             after go-live armed it, and by 0xa509 (which strips bit 12).
             *             s->imr is rewritten from the shadow data[0x435b] as soon as
             *             bit 5 drops, over the whole region [0xa4ca..0xdea0].
             *             Bit 5 is BRINT0 / vec 21, the "BSP buffer received" interrupt
             *             that wakes the correlator. Measured: go-live arms IMR=0x52fd
             *             178 times and 0xb37e wipes it each time; restoring only
             *             0x3050/0x0050 (bit 5 clear) masks BRINT0 for good, IFR bit 5
             *             stays pending for ever and the correlator is never dispatched.
             *             Hence the restore uses the real image, not a constant.
             *   remove  : when the firmware stops losing bit 5, that is when the go-live
             *             sequence 0xa4c7 / 0xa51b / 0xa582 runs in the right order.
             */
            static int ki = -1; static uint16_t kiv = 0;
            if (ki < 0) { ki = calypso_gate("CALYPSO_KEEP_IMR", 0);
                const char *e = getenv("CALYPSO_KEEP_IMR_VAL");
                kiv = (e && *e) ? (uint16_t)strtoul(e, NULL, 0) : 0x52fd; }
            if (ki && exec_pc >= 0xa4ca && exec_pc <= 0xdea0 && !(s->imr & 0x0020)) {
                uint16_t img = s->data[0x435b];            /* shadow IMR (= 0x52fd) */
                if (!(img & 0x0020)) img = kiv;            /* shadow without bit 5 -> fallback */
                s->imr = img;
                static unsigned kil = 0;
                if (kil++ < 8)
                    fprintf(stderr, "[c54x] KEEP-IMR re-arme IMR=0x%04x (bit5/BRINT0 preserve, "
                            "shadow d[435b]=0x%04x) @PC=0x%04x insn=%u\n",
                            s->imr, s->data[0x435b], exec_pc, s->insn_count);
            }
        }
        if (exec_pc == 0xa4d4) {
            static unsigned wt = 0; static uint16_t last = 0xffff;
            uint16_t fl = s->data[0x3f70];
            /* @BEQUILLE - FORCE_GOLIVE  (CALYPSO_FORCE_GOLIVE, atoi>0, default OFF;
             *              empty in hack.env)
             *   masks   : the go-live wait loop tests bit 1 of data[0x3f70], which only the
             *             setter at 0xde9c writes, and that setter is itself conditioned on
             *             the control cells 0x098a/0x098c that the ARM leaves at 0. The bit
             *             is set here, at the 0xa4d4 test.
             *   remove  : as soon as the ARM handshake (ARM2DSP_BGEN) gets the flow past
             *             0xddf5 and the native setter 0xde9c runs.
             */
            { static int fg = -1; if (fg < 0) { const char *e = getenv("CALYPSO_FORCE_GOLIVE"); fg = (e && atoi(e) > 0) ? 1 : 0; }
              if (fg && !(fl & 0x0002)) { s->data[0x3f70] = (uint16_t)(fl | 0x0002); fl = s->data[0x3f70];
                static unsigned fgc = 0; if (fgc++ < 8) fprintf(stderr, "[c54x] FORCE-GOLIVE 0x3f70 |= bit1 -> 0x%04x insn=%u\n", fl, s->insn_count); } }
            if ((fl & 0x0002) || fl != last) {
                if (wt++ < 60)
                    fprintf(stderr, "[c54x] WAIT-TEST PC=0xa4d4 data[0x3f70]=0x%04x bit1=%d "
                            "insn=%u\n", fl, !!(fl & 2), s->insn_count);
                last = fl;
            }
        }

        if (exec_pc == 0x013b) {
            /* @BEQUILLE - ISR_TO_8341  (CALYPSO_ISR_TO_8341, EXISTS, default OFF)
             *   masks   : the overlay ISR prologue 0x013b derails and never reaches the FB
             *             LUT at 0x8341, which does the complete correlator setup (BRC, BK,
             *             data pointers) that the forced entry at 0x8d00 short-circuits into
             *             a dead MAC loop. The frame interrupt vectors 0x00f0 -> 0x7234
             *             (B 0x013b) -> the prologue, so s->pc is forced to 0x8341 when the
             *             interrupt comes from the frame scheduler (g_prev_pc == 0x7234).
             *   remove  : as soon as the 0x013b prologue ends at 0x8341 through its own flow
             *             (same condition as a successful FIX_3FCD).
             *   note    : calypso_wire.env unsets this EXPLICITLY - an empty ":=" under
             *             set -a would turn this EXISTS gate on. Never convert it to ":=".
             */
            static int r8 = -1;
            if (r8 < 0) r8 = calypso_gate("CALYPSO_ISR_TO_8341", 0);
            if (r8 && g_prev_pc == 0x7234) {
                static unsigned r8n = 0;
                if (r8n++ < 8)
                    fprintf(stderr, "[c54x] ISR-TO-8341 : frame ISR 0x013b -> 0x8341 "
                            "(LUT FB setup complet) prev=0x%04x insn=%u\n",
                            g_prev_pc, s->insn_count);
                s->pc = 0x8341;
                return 0;
            }
        }

        if (exec_pc == 0x8d00) {
            /* @BEQUILLE - CORR_SETUP (+ CORR_AR1/_AR4/_AR5)  (CALYPSO_CORR_SETUP, EXISTS,
             *              default OFF)
             *   masks   : the pointer setup the native LUT at 0x8341 performs before entering
             *             0x8d00 (STM #0x2f22,AR1 / #0x2be4,AR4 / #0x0060,AR5, disassembled at
             *             PROM0 0x8347/0x8349/0x834b). Without those pointers the MAC loops at
             *             0x8e8b/0x8e8c without concluding, because 0x8d00 is reached through
             *             BSP_DISPATCH_FB instead. The constants are injected at the
             *             correlator entry.
             *   remove  : when the native path goes through 0x8341 before 0x8d00.
             *   note    : EXISTS idiom - an empty ":=" would TURN IT ON, hence the explicit
             *             unset in calypso_wire.env. Measured ineffective: the ARs are
             *             rewritten before 0x8e8b.
             */
            static int cs = -1; static uint16_t a1 = 0, a4 = 0, a5 = 0;
            if (cs < 0) {
                cs = calypso_gate("CALYPSO_CORR_SETUP", 0);
                const char *e;
                a1 = (e = getenv("CALYPSO_CORR_AR1")) && *e ? (uint16_t)strtoul(e,0,0) : 0x2f22;
                a4 = (e = getenv("CALYPSO_CORR_AR4")) && *e ? (uint16_t)strtoul(e,0,0) : 0x2be4;
                a5 = (e = getenv("CALYPSO_CORR_AR5")) && *e ? (uint16_t)strtoul(e,0,0) : 0x0060;
            }
            if (cs) {
                s->ar[1] = a1; s->ar[4] = a4; s->ar[5] = a5;
                static unsigned csn = 0;
                if (csn++ < 8)
                    fprintf(stderr, "[c54x] CORR-SETUP @0x8d00 : AR1=0x%04x AR4=0x%04x "
                            "AR5=0x%04x (setup LUT 0x8341 injecte) insn=%u\n",
                            a1, a4, a5, s->insn_count);
            }
        }
        calypso_arm2dsp_on_dsp_step(s, exec_pc);

        if (exec_pc == 0xa4ca) {
            /* @BEQUILLE - POKE_A4C7_ONCE  (CALYPSO_POKE_A4C7_ONCE, atoi>0, default OFF)
             *   masks   : 0xa4c7 (ORM #0x3000,IMR, the ROM's own IMR arming) is never
             *             reached: the flow always enters the go-live wait loop directly at
             *             0xa4ca, three words later, and 0xa4c7 has zero hits. The FIRST
             *             arrival at 0xa4ca is redirected to 0xa4c7, so the CPU executes the
             *             real ROM instruction and falls through into 0xa4ca normally. No
             *             register or memory value is poked - only the entry PC, once.
             *             Answers: does IMR arm to 0x3000, does the frame interrupt then
             *             vector to vec 28, does d_fb_det become non-zero, and do
             *             data[0x3f70] and data[0x435b] populate as well (single-root-cause
             *             test)?
             *   remove  : as soon as the upstream path (0xa4cd BC AEQ, or the setter of
             *             d[434e]/d[434f]) falls through into 0xa4c7.
             *   note    : calypso_hack.env itself calls this "falsification, not a fix".
             */
            static int poke_en = -1;
            if (poke_en < 0) { const char *e = getenv("CALYPSO_POKE_A4C7_ONCE"); poke_en = (e && atoi(e) > 0) ? 1 : 0; }
            static int poke_done = 0;
            if (poke_en && !poke_done) {
                poke_done = 1;
                fprintf(stderr, "[c54x] POKE-A4C7-ONCE: redirecting PC 0xa4ca -> "
                        "0xa4c7 (let ROM's own ORM #0x3000,IMR run) insn=%u\n",
                        s->insn_count);
                s->pc = 0xa4c7;
                return 0;
            }
        }

        /* CALA-71DA (CALYPSO_CALA_71DA, read-only): the generic
         * save/dispatch/restore wrapper at 0x71c0-0x71f2 (19 PSHM; ST0=0,
         * ST1=0x6900; CALA at 0x71da; 19 POPM; RET) dispatches to the address
         * in A. Logs A just before the CALA, to tell whether this dispatcher
         * ever calls anything other than a no-op stub - the same closed
         * self-referential loop as data[0x4387] -> 0xab38. */
        if (exec_pc == 0x71da) {
            static int cala_en = -1;
            if (cala_en < 0) cala_en = calypso_gate("CALYPSO_CALA_71DA", 0);
            if (cala_en) {
                static unsigned cala_n = 0;
                static uint16_t last_target = 0xFFFF;
                static unsigned same_target_count = 0;
                uint16_t target = (uint16_t)(s->a & 0xFFFF);
                if (target == last_target) same_target_count++;
                else { same_target_count = 0; last_target = target; }
                if (cala_n < 100 || same_target_count == 0 || (cala_n % 2000) == 0) {
                    fprintf(stderr, "[c54x] CALA-71DA #%u target=0x%04x SP=0x%04x "
                            "repeat_run=%u insn=%u\n", cala_n, target, s->sp,
                            same_target_count, s->insn_count);
                }
                cala_n++;
            }
        }

        {
            /* @BEQUILLE - C54X_FORCE_IMR  (CALYPSO_C54X_FORCE_IMR=<hex>, default OFF)
             *   masks   : the IMR re-arming after the ROM's STM #0,IMR at 0xb37e (insn
             *             ~1047), which is never undone before the scheduler at 0xb41c, so
             *             the frame interrupt stays masked and the DSP spins; and the
             *             RSBX INTM the ROM only plays after go-live. The requested bits are
             *             ORed into the IMR on every step outside an ISR, and INTM is cleared
             *             inside [0xb380..0xb440]. A typical value is 0x52fd (bit 3 INT3,
             *             bit 5 BRINT0, ...). Falsifiable test of the chain
             *             IMR -> interrupt -> 0x0fff -> go-live.
             *   remove  : when the go-live state machine reaches 0xa582 and sets the IMR
             *             itself.
             */
            static int fimr = -1; static uint16_t fimrv = 0;
            if (fimr < 0) { const char *e = getenv("CALYPSO_C54X_FORCE_IMR");
                fimrv = (e && *e) ? (uint16_t)strtoul(e, NULL, 0) : 0; fimr = fimrv ? 1 : 0; }
            if (fimr && (s->imr & fimrv) != fimrv) {
                static unsigned fic = 0;
                if (fic++ < 8)
                    fprintf(stderr, "[c54x] FORCE-IMR 0x%04x |= 0x%04x @PC=0x%04x insn=%u\n",
                            s->imr, fimrv, exec_pc, s->insn_count);
                s->imr |= fimrv;
            }
            /* The idle scheduler loop b380-b440 must run with INTM=0 (waiting
             * for an interrupt), but the ROM only clears INTM (RSBX at 0xa51b)
             * after go-live - a chicken and egg. INTM is therefore cleared
             * outside ISRs only, inside the idle loop, so IRQ-LEVEL can serve
             * the latched frame interrupt. ISRs (low PCs, 0x7234) are left
             * alone. */
            if (fimr && exec_pc >= 0xb380 && exec_pc <= 0xb440 && (s->st1 & ST1_INTM) &&
                ((s->pmst >> PMST_IPTR_SHIFT) & 0x1FF) != 0x1FF) {
                static unsigned ftc = 0;
                if (ftc++ < 8)
                    fprintf(stderr, "[c54x] FORCE-INTM clr @PC=0x%04x IFR=0x%04x IMR=0x%04x insn=%u\n",
                            exec_pc, s->ifr, s->imr, s->insn_count);
                s->st1 &= ~ST1_INTM;
            }
        }

        {
            /* @BEQUILLE - FORCE_098  (CALYPSO_FORCE_098=<hexval>, default empty/OFF;
             *              hack.env)
             *   masks   : the ARM never sets the d_background handshake cells
             *             0x098a/0x098c that the phase state machine (0xddeb, 0xde86)
             *             reads back. They are written non-zero on the DSP side just BEFORE
             *             the LD *(0x098c) / *(0x098a) of the setters (0xde86 on the GO
             *             path, 0xde94, 0xb3e4), which tests whether the DSP then takes the
             *             GO branch at 0xddf5, sets bit 1 and goes live.
             *   remove  : as soon as CALYPSO_ARM2DSP_BGEN sets those cells through the ARM
             *             bridge, with the right causality; calypso_hack.env already
             *             declares the replacement.
             */
            static int f98 = -1; static uint16_t f98v = 0;
            if (f98 < 0) { const char *e = getenv("CALYPSO_FORCE_098");
                f98v = (e && *e) ? (uint16_t)strtoul(e, NULL, 0) : 0; f98 = f98v ? 1 : 0; }
            if (f98 && (exec_pc == 0xde86 || exec_pc == 0xde94 || exec_pc == 0xb3e4 ||
                        exec_pc == 0xa5bd)) {
                s->data[0x098a] = f98v; s->data[0x098c] = f98v;
                static unsigned f9c = 0;
                if (f9c++ < 12)
                    fprintf(stderr, "[c54x] FORCE-098 @0x%04x data[098a/c]=0x%04x insn=%u\n",
                            exec_pc, f98v, s->insn_count);
            }
        }
        if (exec_pc >= 0xa4ca && exec_pc <= 0xa575) {
            /* @BEQUILLE - GOLIVE_TASKW  (CALYPSO_GOLIVE_TASKW, EQ1, default OFF)
             *   masks   : the firmware clears d[0x3f92] with ST #0 at 0xa4c4 and should
             *             re-arm it with ORM #0x0800 at 0xa539, but that native setter is
             *             skipped (d[5a00] == 0x88), so the FB-task bit stays 0 for ever and
             *             the DSP scheduler never dispatches the correlator. The ORM is
             *             replayed here, and only once the ARM has actually ordered go-live
             *             (bit 15 of d[0x0810], set by the CTRLSYS wire): right ARM -> DSP
             *             causality, right bit (0x0800), and an OR so the other scheduler
             *             bits are not overwritten.
             *   remove  : as soon as 0xa539 really executes (its d[5a00] predicate holds the
             *             right value), which makes the replay redundant.
             *   note    : inert without ARM2DSP_CTRLSYS, which sets data[0x0810] bit 15.
             */
            static int gt = -1;
            if (gt < 0) { const char *e = getenv("CALYPSO_GOLIVE_TASKW");
                          gt = (e && *e == '1') ? 1 : 0; }
            if (gt && (s->data[0x0810] & 0x8000)) {
                s->data[0x3f92] |= 0x0800;   /* replay of ORM #0x0800 @0xa539 */
                static unsigned glg = 0;
                if (glg++ < 8)
                    fprintf(stderr, "[c54x] GO-LIVE-TASKW @0x%04x d[3f92]=0x%04x "
                            "(ORM 0xa539 rejoue, ARM 0810 bit15 set) insn=%u\n",
                            exec_pc, s->data[0x3f92], s->insn_count);
            }
        }
        /* SM-TRACE (CALYPSO_SM_TRACE): instruction-by-instruction trace of the
         * handshake state machine 0xdde0-0xde9f (the re-clear route at 0xde8b
         * versus the setter at 0xde9c). Shows PC, opcode, A, TC and the five
         * cells 0x098a..0x098e at each step, to see where the flow leaves the
         * 0xde9c path and which values would route it there. 400 lines. */
        if (exec_pc >= 0xdde0 && exec_pc <= 0xde9f) {
            static int smt = -1; static unsigned smn = 0;
            if (smt < 0) smt = calypso_gate("CALYPSO_SM_TRACE", 0);
            if (smt && smn < 400) {
                smn++;
                fprintf(stderr, "[c54x] SM-TRACE PC=0x%04x op=0x%04x A=0x%04x TC=%d 098[a=%04x b=%04x c=%04x d=%04x e=%04x] insn=%u\n",
                        exec_pc, exec_op, (unsigned)(s->a & 0xFFFF),
                        (s->st0 & ST0_TC) ? 1 : 0,
                        s->data[0x098a], s->data[0x098b], s->data[0x098c],
                        s->data[0x098d], s->data[0x098e], s->insn_count);
            }
        }
        /* B3-TRACE (CALYPSO_B3_TRACE): the idle scheduler 0xb380-0xb440 polls
         * the flag word data[0x0fff] (0xb424 BITF 0x0fff,#2; 0xb427 BC NTC
         * 0xb41c) and should route into the go-live block 0xb3db-0xb3ef
         * (0xb3ef = ST #2,0x3f70). Traces PC plus data[0x0fff], d_dsp_page and
         * d[0x3f70], to show why the ARM command (d_dsp_page bit 1) never
         * reaches 0xb3db. 500 lines. */
        if (exec_pc >= 0xb380 && exec_pc <= 0xb440) {
            static int b3t = -1; static unsigned b3n = 0;
            if (b3t < 0) b3t = calypso_gate("CALYPSO_B3_TRACE", 0);
            if (b3t && b3n < 500) {
                b3n++;
                fprintf(stderr, "[c54x] B3-TRACE PC=0x%04x op=0x%04x fff=0x%04x "
                        "dsp_page=0x%04x 3f70=0x%04x TC=%d insn=%u\n",
                        exec_pc, exec_op, s->data[0x0fff], s->data[0x08E2],
                        s->data[0x3f70], (s->st0 & ST0_TC) ? 1 : 0, s->insn_count);
            }
        }
        if (exec_pc == 0xde97 || exec_pc == 0xde9c || exec_pc == 0xdddb || exec_pc == 0xde8b) {
            static unsigned db = 0;
            if (db++ < 50)
                fprintf(stderr, "[c54x] DE-BR PC=0x%04x data[0x3f70]=0x%04x A=0x%06llx TC=%d "
                        "insn=%u\n", exec_pc, s->data[0x3f70],
                        (unsigned long long)(s->a & 0xFFFFFF),
                        (s->st0 & ST0_TC) ? 1 : 0, s->insn_count);
        }
        /* HANDLER-PATH (read-only): the vec 28 handler runs but does not reach
         * the dispatch at 0xa51c. Traces the exact chain 0x00f0 (vector) ->
         * 0x7234 (handler) -> 0x013b (prologue) -> 0xa4e4 (scheduler) ->
         * 0xa4ff (CALL 0xb522) -> 0xa501/0xa507 -> 0xa51c (dispatch) or
         * 0xa509 (arm IMR), to see where it leaves the path. 120 lines. */
        switch (exec_pc) {
        case 0x00f0: case 0x7234: case 0x013b: case 0xa4e4: case 0xa4ff:
        case 0xb522: case 0xa501: case 0xa507: case 0xa51c: case 0xa509:
        case 0xa582: case 0x011e: case 0xa4c7: case 0xa4ca:
        case 0x703d: case 0xa9ea: case 0xb3ec: {
            static unsigned hp = 0;
            if (hp++ < 120)
                fprintf(stderr, "[c54x] HANDLER-PATH PC=0x%04x A=0x%06llx SP=0x%04x "
                        "INTM=%d d[0x435b]=0x%04x d[0x3f70]=0x%04x insn=%u\n",
                        exec_pc, (unsigned long long)(s->a & 0xFFFFFF), s->sp,
                        (s->st1 & ST1_INTM) ? 1 : 0, s->data[0x435b],
                        s->data[0x3f70], s->insn_count);
            break;
        }
        default: break;
        }
        /* ENTRY-A4CA (read-only): HOW the DSP enters the wait loop at 0xa4ca -
         * the exact caller (the previous exec_pc outside 0xa4ca..0xa4e2) and
         * the state. Tells a normal entry through go-live 0xa500 (INTM already
         * cleared) from a direct branch or soft-vector, which bypasses go-live
         * and leaves INTM set. 20 lines. */
        {
            static uint16_t prev_pc = 0;
            if (exec_pc == 0xa4ca && (prev_pc < 0xa4ca || prev_pc > 0xa4e2)) {
                static unsigned ea = 0;
                if (ea++ < 20)
                    fprintf(stderr, "[c54x] ENTRY-A4CA #%u FROM PC=0x%04x INTM=%d "
                            "IMR=0x%04x data[0x3f6d]=0x%04x d[434e]=%04x d[434f]=%04x "
                            "insn=%u\n",
                            ea, prev_pc, (s->st1 & ST1_INTM) ? 1 : 0, s->imr,
                            s->data[0x3f6d], s->data[0x434e], s->data[0x434f],
                            s->insn_count);
            }
            prev_pc = exec_pc;
        }
        /* B19D-WATCH (read-only): is the ARM-command dispatch 0xb19d -> go-live
         * 0xa500 ever reached? GOLIVE-WATCH already covers 0xa500 and never
         * sees anything past 0xa4e2, so this watches the upstream side. No hit
         * means the DSP never processes d_dsp_page and is stuck in the wrong
         * loop before go-live. 30 lines. */
        if (exec_pc >= 0xb19d && exec_pc <= 0xb1b0) {
            static unsigned bw = 0;
            if (bw++ < 30)
                fprintf(stderr, "[c54x] B19D-WATCH #%u PC=0x%04x op=0x%04x INTM=%d "
                        "A=0x%06llx data[0x08E2]=0x%04x insn=%u\n",
                        bw, exec_pc, exec_op, (s->st1 & ST1_INTM) ? 1 : 0,
                        (unsigned long long)(s->a & 0xFFFFFF),
                        s->data[0x08E2], s->insn_count);
        }
        /* AAD5-TRACE: the go-live/AFC loop at 0xa4ca never releases (the BC at
         * 0xa4cd is AEQ on A == 0), and A comes from CALL 0xaad5. Traces
         * 0xaad5-0xaae6, the code that sets A, instruction by instruction with
         * AR0/AR1/A/TC, the ring counters data[0x434e]/data[0x434f] and the
         * candidate flags data[0x3f70]/data[0x3f92]/data[0x435b]: a counter
         * that never advances points at a decode or data bug, an idle wait at
         * a missing interrupt (IMR == 0). The verdict at the 0xa4cd gate is
         * logged too. 100 lines. */
        if (exec_pc >= 0xaad5 && exec_pc <= 0xaae6) {
            static unsigned at = 0;
            if (at++ < 100)
                fprintf(stderr, "[c54x] AAD5 #%u PC=0x%04x op=0x%04x A=0x%06llx "
                        "AR0=%04x AR1=%04x BK=%04x TC=%d d[434e]=%04x d[434f]=%04x "
                        "insn=%u\n",
                        at, exec_pc, exec_op, (unsigned long long)(s->a & 0xFFFFFF),
                        s->ar[0], s->ar[1], s->bk, (s->st0 & ST0_TC) ? 1 : 0,
                        s->data[0x434e], s->data[0x434f], s->insn_count);
        }
        if (exec_pc == 0xa4cd) {     /* the BC AEQ gate: why is A == 0? */
            static unsigned gt = 0;
            if (gt++ < 30)
                fprintf(stderr, "[c54x] AFC-GATE #%u @0xa4cd A=0x%06llx (==0?%d) TC=%d "
                        "d[3f70]=%04x d[3f92]=%04x d[435b]=%04x d[3fde]=%04x insn=%u\n",
                        gt, (unsigned long long)(s->a & 0xFFFFFF),
                        ((s->a & 0xFFFFFFFFFFULL) == 0), (s->st0 & ST0_TC) ? 1 : 0,
                        s->data[0x3f70], s->data[0x3f92], s->data[0x435b],
                        s->data[0x3fde], s->insn_count);
        }

        /* INTM-CLEAR (ungated): the first INTM 1->0 transition of the run means
         * RSBX INTM finally executed, i.e. interrupts are globally enabled. */
        {
            static int prev_intm = -1;
            int now_intm = (s->st1 & ST1_INTM) ? 1 : 0;
            if (prev_intm == 1 && now_intm == 0) {
                static unsigned ic = 0;
                if (ic++ < 10)
                    fprintf(stderr, "[c54x] *** INTM-CLEAR #%u : INTM 1->0 @PC=0x%04x "
                            "IMR=0x%04x insn=%u — interruptions ARMEES ! ***\n",
                            ic, exec_pc, s->imr, s->insn_count);
            }
            prev_intm = now_intm;
        }

        /* MVDESYNC: operand words must never be executed as instructions.
         * Fires when the PC lands on an MVKD operand word (0xb3ce, 0xb3d1,
         * 0xb3d4) or an MVDK one (0xb3dd, 0xb3e0, 0xb3e3), which means a
         * residual mis-decode. Silence is the expected result. 40 lines. */
        if (exec_pc==0xb3ce||exec_pc==0xb3d1||exec_pc==0xb3d4||
            exec_pc==0xb3dd||exec_pc==0xb3e0||exec_pc==0xb3e3) {
            static unsigned md=0;
            if (md++<40)
                fprintf(stderr, "[c54x] MVDESYNC #%u PC=0x%04x op=0x%04x "
                        "(mot-operande execute = mis-decode!) from=0x%04x insn=%u\n",
                        md, exec_pc, exec_op, g_prev_pc, s->insn_count);
        }
        /* Values of the two CALA slots; the third is the BACC at 0xb40f,
         * already covered by BACC-DISP. */
        if (exec_pc==0xb3a5) {
            static unsigned p5=0;
            if (p5++<30)
                fprintf(stderr, "[c54x] CALA-SLOT1 @0xb3a5 data[0x0c36]=0x%04x "
                        "A=0x%06llx SP=0x%04x insn=%u\n",
                        s->data[0x0c36], (unsigned long long)(s->a & 0xFFFFFF),
                        s->sp, s->insn_count);
        }
        if (exec_pc==0xb3e6) {
            static unsigned p6=0;
            if (p6++<30)
                fprintf(stderr, "[c54x] CALA-SLOT2 @0xb3e6 A=0x%06llx "
                        "data[0x3f6b]=0x%04x SP=0x%04x insn=%u\n",
                        (unsigned long long)(s->a & 0xFFFFFF), s->data[0x3f6b],
                        s->sp, s->insn_count);
        }
        /* Does the init ever program IPTR=0x140 (vector base 0xa000, so INT3
         * lands at 0xa04c, the real frame handler)? One-shot. */
        {
            static int seen140=0;
            uint16_t iptr_now=(s->pmst >> PMST_IPTR_SHIFT) & 0x1FF;
            if (iptr_now==0x140 && !seen140) {
                seen140=1;
                fprintf(stderr, "[c54x] *** IPTR=0x140 REACHED *** PMST=0x%04x "
                        "PC=0x%04x insn=%u\n", s->pmst, exec_pc, s->insn_count);
            }
        }
        /* Post-bootstub-RET probe: prologue at 0x7013 (PSHM of the context) and
         * epilogue at 0x7020 (POPM ar1/st0/st1/pmst, then RET). An epilogue
         * that pops a stale context or return PC, with no matching prologue or
         * interrupt, means the stack desynchronised after the BACC.
         * retPC = data[SP+4], after the four POPMs. */
        if (exec_pc == 0x7013) {
            static unsigned pbp = 0;
            if (pbp++ < 40)
                fprintf(stderr, "[c54x] PBPRO #%u from=0x%04x SP=0x%04x insn=%u\n",
                        pbp, g_prev_pc, s->sp, s->insn_count);
        }
        if (exec_pc == 0x7020) {
            static unsigned pbr = 0;
            if (pbr++ < 40)
                fprintf(stderr, "[c54x] PBRET #%u from=0x%04x SP=0x%04x pop[ar1=0x%04x "
                        "st0=0x%04x st1=0x%04x pmst=0x%04x retPC=0x%04x] insn=%u\n",
                        pbr, g_prev_pc, s->sp,
                        s->data[s->sp], s->data[(uint16_t)(s->sp+1)],
                        s->data[(uint16_t)(s->sp+2)], s->data[(uint16_t)(s->sp+3)],
                        s->data[(uint16_t)(s->sp+4)], s->insn_count);
        }

        {
            /* @BEQUILLE - GOLIVE_REDIRECT  (CALYPSO_DSP_GOLIVE_BOOT, EXISTS, default OFF)
             *   masks   : writes s->pc = 0xb3ec when the DSP reaches 0xb3ff, i.e. the
             *             go-live soft-vector choice the boot ROM does not make in this
             *             model. The DSP then runs the go-live path in the FOREGROUND,
             *             setting up its own context - this is a redirect, not a
             *             vectorisation onto an unset context. Side effect: inhibits
             *             VEC28-FORCE in c54x_interrupt_ex.
             *   remove  : when data[0x3f6d] is populated by the ROM path and points at
             *             0xa4c7.
             */
            static int g_golive = -1;
            if (g_golive < 0) g_golive = calypso_gate("CALYPSO_DSP_GOLIVE_BOOT", 0);
            if (g_golive) {
                static int gdone = 0;
                if (!gdone && s->pc == 0xb3ff) {
                    gdone = 1;
                    fprintf(stderr, "[c54x] GOLIVE-REDIRECT pc 0xb3ff(wait) -> 0xb3ec(go-live "
                            "path: set flags + CALL 0xa9ea + BACC 0x703d -> contexte + IMR) "
                            "IMR=0x%04x SP=0x%04x insn=%u\n",
                            s->imr, s->sp, s->insn_count);
                    s->pc = 0xb3ec;
                }
            }
        }
        /* DERAIL-ZERO: entering 0x0000-0x0008 means a CALA through a null
         * function pointer (an empty SARAM dispatcher slot). Logs the CALA site
         * (g_prev_pc), the accumulators and the four ROM words BEFORE the CALA,
         * i.e. the `ld *(slot),b` whose slot address is the one to decode. */
        if (exec_pc <= 0x0008 && g_prev_pc > 0x0008) {
            static unsigned dz = 0;
            if (dz++ < 60) {
                uint16_t p4 = prog_fetch(s, (uint16_t)(g_prev_pc - 4));
                uint16_t p3 = prog_fetch(s, (uint16_t)(g_prev_pc - 3));
                uint16_t p2 = prog_fetch(s, (uint16_t)(g_prev_pc - 2));
                uint16_t p1 = prog_fetch(s, (uint16_t)(g_prev_pc - 1));
                fprintf(stderr, "[c54x] DERAIL-ZERO #%u PC=0x%04x from=0x%04x op=0x%04x "
                        "A=0x%010llx B=0x%010llx SP=0x%04x prevwords=[%04x %04x %04x %04x] "
                        "insn=%u\n",
                        dz, exec_pc, g_prev_pc, g_prev_op,
                        (unsigned long long)(s->a & 0xFFFFFFFFFFULL),
                        (unsigned long long)(s->b & 0xFFFFFFFFFFULL), s->sp,
                        p4, p3, p2, p1, s->insn_count);
                if (dz == 1) {
                    /* SP ring at the first storm: the last 64 push/pop events
                     * (pc, op, delta), which name the instruction that
                     * over-pops and drifts SP. */
                    fprintf(stderr, "[c54x] SP-RING-AT-STORM (64 derniers, ancien->recent ; MISMATCH = op qui touche SP a tort):\n");
                    for (int k = 63; k >= 0; k--) {
                        struct sp_evt *e = &g_spring[(g_spring_idx - 1 - k) & 63];
                        const char *mn = (e->op == 0xFC00) ? "RET" : classify_xfer_op(e->op);
                        int exp = 99;  /* 99 = not a transfer op (legitimate PSHM/POPM/FRAME/STM-SP, or unknown) */
                        if (e->op == 0xFC00) exp = +1;                                  /* RET */
                        else if (!strcmp(mn,"CALL")||!strcmp(mn,"CALLD")||!strcmp(mn,"CALA")) exp = -1;
                        else if (!strcmp(mn,"FCALL")||!strcmp(mn,"FCALLD")||!strcmp(mn,"FCALA")||!strcmp(mn,"FCALAD")) exp = -2;
                        else if (!strcmp(mn,"RETE")) exp = +1;
                        else if (!strcmp(mn,"FRET")||!strcmp(mn,"FRETD")) exp = +2;
                        else if (!strcmp(mn,"B")||!strcmp(mn,"BD")||!strcmp(mn,"BACC")||!strcmp(mn,"FB")||!strcmp(mn,"FBD")||!strcmp(mn,"FBACC")||!strcmp(mn,"FBACCD")) exp = 0;
                        const char *flag = "";
                        if (exp != 99 && e->delta != exp) flag = "  <<< MISMATCH (delta != mnemo)";
                        else if (exp == 99 && e->delta != 0) flag = "  <<< NON-XFER touche SP (PSHM/POPM/FRAME? ou parasite)";
                        fprintf(stderr, "  pc=0x%04x op=0x%04x %-7s delta=%+d(exp%+d) sp=0x%04x%s\n",
                                e->pc, e->op, mn, e->delta, exp, e->sp, flag);
                    }
                }
            }
        }

        /* DERAIL-ORIGIN: first entry into the garbage table area 0xf000-0xf0ff
         * from outside it, i.e. the jump or fall-through that derails. Logs
         * where it came from (g_prev_pc), the opcode, the ARs (already garbage
         * or not) and SP. */
        if (exec_pc >= 0xf000 && exec_pc <= 0xf0ff
            && (g_prev_pc < 0xf000 || g_prev_pc > 0xf0ff)) {
            static unsigned dor = 0;
            if (dor++ < 60) {
                uint16_t q2 = prog_fetch(s, (uint16_t)(g_prev_pc - 2));
                uint16_t q1 = prog_fetch(s, (uint16_t)(g_prev_pc - 1));
                fprintf(stderr, "[c54x] DERAIL-ORIGIN #%u into=0x%04x from=0x%04x op=0x%04x "
                        "prev=[%04x %04x] AR[2,3,4,5]=%04x,%04x,%04x,%04x SP=0x%04x XPC=%u "
                        "insn=%u\n",
                        dor, exec_pc, g_prev_pc, g_prev_op, q2, q1,
                        s->ar[2], s->ar[3], s->ar[4], s->ar[5], s->sp, s->xpc,
                        s->insn_count);
            }
        }
        /* Silent capture of the LUT slot read at 0x834d
         * (LD (DP<<7|0x07)<<1,A). One compare per instruction and no log, so
         * no timing impact. Feeds the BLACKHOLE-CALA probe (self-CALA at
         * 0x70c3). */
        if (s->pc == 0x834d) {
            g_disp_lut_ea  = (uint16_t)(((s->st0 & 0x1FF) << 7) | 0x07);
            g_disp_lut_val = s->data[g_disp_lut_ea];
        }
        uint16_t sp_before_exec = s->sp;
        uint16_t ds_before = s->delay_slots;  /* delay slots are counted in WORDS */
        /* SHADOW-DADST pre-capture (read-only). Gated on the DADST/DSADT
         * opcode (0x5a/0x5b/0x5e/0x5f) everywhere, not on PC 0x9a80, which
         * varies between runs (narrow versus wide). This family falls into
         * SFTL (case 0x5). Captures the state BEFORE, for the A and B deltas,
         * the AR5 walk and the shadow-correct result; the logged PC shows
         * where it actually runs. */
        int64_t  sd_a0 = s->a, sd_b0 = s->b;
        uint16_t sd_ar5_0 = s->ar[5], sd_lhi = 0, sd_llo = 0;
        uint8_t  sd_sub = (exec_op >> 8) & 0xFF;
        int sd_armed = (sd_sub == 0x5a || sd_sub == 0x5b
                        || sd_sub == 0x5e || sd_sub == 0x5f);
        if (sd_armed) {
            sd_lhi = s->data[s->ar[5]];
            sd_llo = s->data[(uint16_t)(s->ar[5] + 1)];
        }
        /* PISTE-PC: step-by-step trace of a PC RANGE, with the real opcode as
         * the core sees it and the registers BEFORE execution. Env:
         * CALYPSO_PISTE_LO / CALYPSO_PISTE_HI / CALYPSO_PISTE_N (default 400).
         * Motivation: the stage that rewrites the soft-bit buffer at 0x2a00
         * emits 142 zeros on 7 of 12 SB jobs and sane values on the other 5.
         * Unlike the probes in c54x_mem.c, the PC printed here is the one
         * BEFORE the advance, so it is the instruction's real PC. */
        {
            static int pi_init = 0; static long pi_lo = -1, pi_hi = -1, pi_max = 400, pi_n = 0;
            if (!pi_init) { pi_init = 1;
                const char *l = getenv("CALYPSO_PISTE_LO"), *h = getenv("CALYPSO_PISTE_HI"),
                           *n = getenv("CALYPSO_PISTE_N");
                if (l && *l) pi_lo = strtol(l, NULL, 0);
                if (h && *h) pi_hi = strtol(h, NULL, 0); else pi_hi = pi_lo;
                if (n && *n) pi_max = strtol(n, NULL, 0);
            }
            if (pi_lo >= 0 && s->pc >= (uint16_t)pi_lo && s->pc <= (uint16_t)pi_hi
                && pi_n < pi_max) {
                pi_n++;
                fprintf(stderr, "[c54x] PISTE pc=0x%04x op=0x%04x A=0x%010llx B=0x%010llx "
                        "T=0x%04x AR0=%04x AR2=%04x AR3=%04x AR4=%04x AR5=%04x AR6=%04x AR7=%04x "
                        "BRC=%u rpt=%u insn=%u mot_suivant=0x%04x\n",
                        s->pc, exec_op, (unsigned long long)(s->a & 0xFFFFFFFFFFULL),
                        (unsigned long long)(s->b & 0xFFFFFFFFFFULL), s->t,
                        s->ar[0], s->ar[2], s->ar[3], s->ar[4], s->ar[5], s->ar[6], s->ar[7],
                        (unsigned)s->brc, (unsigned)s->rpt_count, (unsigned)s->insn_count,
                        prog_fetch(s, (uint16_t)(s->pc + 1)));
                /* Words POINTED AT by AR2..AR5, read straight from data[]
                 * (these addresses are internal RAM, outside the OVLY/MMR
                 * aliases), to see what a dual-operand mpy/mac really
                 * consumes. */
                { /* Direct addressing: DP, CPL, SP and both candidate
                   * addresses with their contents. A `ld Smem,T` that loads 0
                   * may be reading the right (empty) cell or the wrong one;
                   * only the resolved address decides. */
                    uint16_t dpv = s->st0 & 0x1FF;
                    uint16_t adp = (uint16_t)((dpv << 7) | (exec_op & 0x7F));
                    uint16_t asp = (uint16_t)(s->sp + (exec_op & 0x7F));
                    fprintf(stderr, "[c54x] PISTE   DP=0x%03x CPL=%d SP=%04x | direct@DP=0x%04x:%04x "
                            "direct@SP=0x%04x:%04x | indirect=%s\n",
                            dpv, (s->st1 & 0x4000) ? 1 : 0, s->sp,
                            adp, s->data[adp], asp, s->data[asp],
                            (exec_op & 0x80) ? "oui" : "non");
                }
                { /* Optional dump of a data range on each pass, to see a
                   * healthy job diverge from one that emits zeros. */
                    static int pd_init = 0; static long pd_lo = -1, pd_hi = -1;
                    if (!pd_init) { pd_init = 1;
                        const char *l = getenv("CALYPSO_PISTE_DUMP_LO");
                        const char *h = getenv("CALYPSO_PISTE_DUMP_HI");
                        if (l && *l) pd_lo = strtol(l, NULL, 0);
                        if (h && *h) pd_hi = strtol(h, NULL, 0); else pd_hi = pd_lo;
                    }
                    if (pd_lo >= 0) {
                        for (long a = pd_lo; a <= pd_hi; a += 8) {
                            fprintf(stderr, "[c54x] PISTE-RAM 0x%04lx:", a);
                            for (long k = a; k < a + 8 && k <= pd_hi; k++)
                                fprintf(stderr, " %04x", s->data[(uint16_t)k]);
                            fprintf(stderr, "\n");
                        }
                    }
                }
                fprintf(stderr, "[c54x] PISTE   *AR2[%04x]=%04x *AR3[%04x]=%04x "
                        "*AR4[%04x]=%04x *AR5[%04x]=%04x\n",
                        s->ar[2], s->data[s->ar[2]], s->ar[3], s->data[s->ar[3]],
                        s->ar[4], s->data[s->ar[4]], s->ar[5], s->data[s->ar[5]]);
            }
        }
        uint16_t t_avant_piste = s->t; uint16_t pc_avant_piste = s->pc;
        int64_t a_avant_piste = s->a, b_avant_piste = s->b;
        consumed = c54x_exec_one(s);
        /* PISTE-T: logs EVERY change of the T register inside an instruction
         * window (CALYPSO_T_LO / CALYPSO_T_HI, in insns). Motivation: every
         * mpy of the SB chain computes T*Smem with T == 0 while the memory
         * operands are sane, so who loads T and when has to be measured, not
         * guessed. */
        {
            static int ti_init = 0; static long ti_lo = -1, ti_hi = -1;
            if (!ti_init) { ti_init = 1;
                const char *l = getenv("CALYPSO_T_LO"), *h = getenv("CALYPSO_T_HI");
                if (l && *l) ti_lo = strtol(l, NULL, 0);
                if (h && *h) ti_hi = strtol(h, NULL, 0);
            }
            int dans_fenetre = (ti_lo >= 0 && s->insn_count >= (unsigned)ti_lo
                                && s->insn_count <= (unsigned)ti_hi);
            if (dans_fenetre && s->t != t_avant_piste)
                fprintf(stderr, "[c54x] PISTE-T pc=0x%04x op=0x%04x T: 0x%04x -> 0x%04x insn=%u\n",
                        pc_avant_piste, exec_op, t_avant_piste, s->t, (unsigned)s->insn_count);
            /* Who zeroes: non-zero -> zero transitions of an accumulator. The
             * fault propagates step by step ("zero because its input is
             * zero"), so what matters is the FIRST zeroing. */
            if (dans_fenetre && a_avant_piste != 0 && s->a == 0)
                fprintf(stderr, "[c54x] PISTE-NUL pc=0x%04x op=0x%04x A: 0x%010llx -> 0 insn=%u\n",
                        pc_avant_piste, exec_op,
                        (unsigned long long)(a_avant_piste & 0xFFFFFFFFFFULL),
                        (unsigned)s->insn_count);
            if (dans_fenetre && b_avant_piste != 0 && s->b == 0)
                fprintf(stderr, "[c54x] PISTE-NUL pc=0x%04x op=0x%04x B: 0x%010llx -> 0 insn=%u\n",
                        pc_avant_piste, exec_op,
                        (unsigned long long)(b_avant_piste & 0xFFFFFFFFFFULL),
                        (unsigned)s->insn_count);
        }
        /* SP-COLLAPSE probe (read-only): catches the exact instruction that
         * collapses SP below 0x0800 (the first plus 30 more), which seeds the
         * whole cascade (boot stub, spin at 0xc6ac). */
        {
            static uint16_t sp_prev = 0xffff;
            static unsigned spc_n = 0;
            if (sp_prev >= 0x0800 && s->sp < 0x0800 && spc_n < 30) {
                spc_n++;
                fprintf(stderr, "[c54x] SP-COLLAPSE #%u exec_pc=0x%04x exec_op=0x%04x "
                        "prev_PC=0x%04x prev_op=0x%04x SP 0x%04x->0x%04x B=0x%010llx insn=%u\n",
                        spc_n, exec_pc, exec_op, s->last_exec_pc, s->last_exec_op,
                        sp_prev, s->sp,
                        (unsigned long long)(s->b & 0xFFFFFFFFFFULL), s->insn_count);
            }
            sp_prev = s->sp;
        }
        /* FEXX-ENTRY probe (read-only): captures the jump or vector that
         * transfers control INTO the ROM range 0xfe00-0xff7f (data table plus
         * the init at 0xff00), the real entry of the derail. The static CALL
         * 0xfe00 at 0xfdd5 never executes, so it is a COMPUTED branch;
         * prev_PC/op is the culprit and A is the target when it is a CALA.
         * First plus 20 more. */
        {
            static int fe_was_in = 0;
            static unsigned fe_n = 0;
            int fe_in = (exec_pc >= 0xfe00 && exec_pc < 0xff80);
            if (fe_in && !fe_was_in && fe_n < 20) {
                fe_n++;
                fprintf(stderr, "[c54x] FEXX-ENTRY #%u entered 0x%04x from prev_PC=0x%04x "
                        "prev_op=0x%04x A=0x%010llx SP=0x%04x insn=%u\n",
                        fe_n, exec_pc, s->last_exec_pc, s->last_exec_op,
                        (unsigned long long)(s->a & 0xFFFFFFFFFFULL), s->sp, s->insn_count);
            }
            fe_was_in = fe_in;
        }
        /* HIGHVEC-ENTRY probe (read-only): entry into the IPTR=0x1FF vector
         * page [0xFF80..0xFFFF], where interrupts land (vec 19 = 0xffcc,
         * vec 21 = 0xffd4). The decisive question is whether control arrives
         * through an INTERRUPT DISPATCH (prev = preempted foreground, d_irq
         * near 0) or through a firmware BRANCH/CALL (prev = a branch opcode
         * targeting here, d_irq large). Also pins the register regime (ARs, A,
         * B) at the entry point. */
        {
            static int hv_was_in = 0;
            static unsigned hv_n = 0;
            int hv_in = (exec_pc >= 0xff80);
            if (hv_in && !hv_was_in && hv_n < 24) {
                hv_n++;
                uint16_t hv_iptr = (s->pmst >> PMST_IPTR_SHIFT) & 0x1FF;
                uint64_t d_irq = (uint64_t)s->insn_count - g_last_intr_insn;
                int irq_driven = (d_irq <= 2 && g_last_intr_vec >= 0);
                fprintf(stderr, "[c54x] HIGHVEC-ENTRY #%u entered 0x%04x prev_PC=0x%04x "
                        "prev_op=0x%04x | %s d_irq=%llu lastvec=%d fg_pc=0x%04x | "
                        "IPTR=0x%03x INTM=%d SP=0x%04x | AR3=0x%04x AR4=0x%04x AR5=0x%04x "
                        "A=0x%010llx B=0x%010llx insn=%u\n",
                        hv_n, exec_pc, s->last_exec_pc, s->last_exec_op,
                        irq_driven ? "IRQ-DISPATCH" : "FW-BRANCH",
                        (unsigned long long)d_irq, g_last_intr_vec, g_last_intr_fg_pc,
                        hv_iptr, !!(s->st1 & ST1_INTM), s->sp,
                        s->ar[3], s->ar[4], s->ar[5],
                        (unsigned long long)(s->a & 0xFFFFFFFFFFULL),
                        (unsigned long long)(s->b & 0xFFFFFFFFFFULL), s->insn_count);
            }
            hv_was_in = hv_in;
        }
        /* REGIME-PIN probe (read-only): pins the register regime of the
         * CURRENT run without ambiguity - two different regimes have been
         * observed (AR5=0x80 with AR3=0x000b, and AR3=AR4=0x2ace frozen in the
         * I/Q buffer), and only one can be debugged at a time. Full register
         * file dump on the 1st, 10000th and 1000000th visit of the foreground
         * spin [0x82c0..0x82f0], the frozen IQ-READ loop. */
        if (exec_pc >= 0x82c0 && exec_pc <= 0x82f0) {
            static unsigned rp_n = 0;
            rp_n++;
            if (rp_n == 1 || rp_n == 10000 || rp_n == 1000000) {
                uint16_t rp_iptr = (s->pmst >> PMST_IPTR_SHIFT) & 0x1FF;
                fprintf(stderr, "[c54x] REGIME-PIN #%u @0x%04x | "
                        "AR0=%04x AR1=%04x AR2=%04x AR3=%04x AR4=%04x AR5=%04x AR6=%04x AR7=%04x | "
                        "A=0x%010llx B=0x%010llx T=0x%04x | SP=0x%04x ST0=0x%04x ST1=0x%04x "
                        "PMST=0x%04x IPTR=0x%03x INTM=%d insn=%u\n",
                        rp_n, exec_pc,
                        s->ar[0], s->ar[1], s->ar[2], s->ar[3], s->ar[4], s->ar[5], s->ar[6], s->ar[7],
                        (unsigned long long)(s->a & 0xFFFFFFFFFFULL),
                        (unsigned long long)(s->b & 0xFFFFFFFFFFULL), s->t,
                        s->sp, s->st0, s->st1, s->pmst, rp_iptr,
                        !!(s->st1 & ST1_INTM), s->insn_count);
            }
        }
        /* SQURA-A probe (read-only): watches A become positive. Logs A at the
         * two SQURA (0x76ff, 0x7700) and at the RCD LEQ decision in 0x75e8,
         * which is taken when A <= 0. With the blind MAC, A stayed <= 0, the
         * RCD was taken and the stack over-popped; with SQURA decoded, A must
         * become positive and the RCD must not be taken. */
        if (exec_pc == 0x76ff || exec_pc == 0x7700 || exec_pc == 0x75e8) {
            static unsigned sqa_n = 0;
            if (sqa_n < 30) {
                sqa_n++;
                int64_t av = (s->a & 0x8000000000ULL) ? (s->a | ~0xFFFFFFFFFFLL) : (s->a & 0xFFFFFFFFFFLL);
                fprintf(stderr, "[c54x] SQURA-A pc=0x%04x op=0x%04x A=0x%010llx (signed=%lld) "
                        "T=0x%04x AR2=0x%04x %sinsn=%u\n",
                        exec_pc, exec_op, (unsigned long long)(s->a & 0xFFFFFFFFFFULL),
                        (long long)av, s->t, s->ar[2],
                        (exec_pc == 0x75e8) ? (av <= 0 ? "RCD-PREND(A<=0) " : "RCD-passe(A>0) ") : "",
                        s->insn_count);
            }
        }
        /* DISP-A probe (read-only): how A evolves through the dispatcher
         * 0x80b0-0x80c9 - which LD sets A = 0xfe36 (the garbage handler of the
         * FCALAD at 0x80c8) and from which data cell. */
        if (exec_pc >= 0x80b0 && exec_pc < 0x80ca) {
            static unsigned da_n = 0;
            uint16_t alo = (uint16_t)(s->a & 0xFFFF);
            int garbage = (exec_pc == 0x80c2 && alo >= 0xfe00); /* the LD that derails */
            if (da_n < 40 || garbage) {
                da_n++;
                uint16_t dp = s->st0 & 0x1FF;
                fprintf(stderr, "[c54x] DISP-A pc=0x%04x op=0x%04x A.lo=0x%04x DP=0x%03x "
                        "AR1=0x%04x AR2=0x%04x AR3=0x%04x d[DP:9]=0x%04x insn=%u%s\n",
                        exec_pc, exec_op, alo, dp, s->ar[1], s->ar[2], s->ar[3],
                        s->data[(uint16_t)((dp << 7) | 0x09)], s->insn_count,
                        garbage ? "  <<< A=GARBAGE handler = LE DERAIL" : "");
            }
        }
        /* DP-AT-DISP probe (read-only): tracks the LAST instruction that
         * changes DP (ST0[8:0]) and, when the dispatcher is entered (0x80b0),
         * logs DP and its writer. Finds who sets DP = 0x124, the wrong page:
         * the coefficient table at 0x9200 instead of a handler. */
        {
            static uint16_t dp_prev = 0xFFFF, dp_set_pc = 0, dp_set_op = 0;
            uint16_t dp_now = s->st0 & 0x1FF;
            if (dp_now != dp_prev) { dp_set_pc = exec_pc; dp_set_op = exec_op; dp_prev = dp_now; }
            if (exec_pc == 0x80b0) {
                static unsigned dpw_n = 0;
                if (dpw_n < 20 || dp_now == 0x124) {
                    dpw_n++;
                    fprintf(stderr, "[c54x] DP-AT-DISP DP=0x%03x set_by_PC=0x%04x set_op=0x%04x insn=%u%s\n",
                            dp_now, dp_set_pc, dp_set_op, s->insn_count,
                            (dp_now == 0x124) ? "  <<< BAD DP (coefficient page = LE DERAIL)" : "");
                }
            }
        }
        /* SHADOW-DADST post-compute and oracle (read-only, logs only: the
         * first 40 during boot, then one in 2000 in steady state). */
        if (sd_armed) {
            static unsigned sd_n = 0;
            if (sd_n < 40 || (sd_n % 2000) == 0) {
                int64_t a1 = s->a, b1 = s->b;
                int asm5 = s->st1 & ST1_ASM_MASK; if (asm5 & 0x10) asm5 -= 32; /* signed 5-bit ASM */
                uint64_t a0u = sd_a0 & 0xFFFFFFFFFFULL;
                int64_t pure_sh = (asm5 >= 0) ? (int64_t)(a0u << asm5)
                                              : (int64_t)(a0u >> (-asm5));
                int shiftLike = ((a1 & 0xFFFFFFFFFFULL) == (pure_sh & 0xFFFFFFFFFFULL)); /* OBS1 */
                int16_t T = (int16_t)s->t, lhi = (int16_t)sd_lhi, llo = (int16_t)sd_llo;
                int is_dadst = (sd_sub == 0x5a || sd_sub == 0x5b);
                int32_t sh_hi = is_dadst ? (lhi + T) : (lhi - T); /* SPRU131 dual-16; sign to confirm */
                int32_t sh_lo = is_dadst ? (llo - T) : (llo + T);
                int64_t shadow = (((int64_t)sh_hi & 0xFFFF) << 16) | (uint16_t)sh_lo; /* OBS3 */
                fprintf(stderr,
                  "[c54x] SHADOW-DADST #%u pc=0x%04x op=0x%04x xpc=%u C16=%d FRCT=%d ASM=%d BK=0x%04x\n"
                  "[c54x]   A %010llx->%010llx dA=%lld shiftLike=%d  B %010llx->%010llx dB=%lld\n"
                  "[c54x]   AR5 0x%04x->0x%04x walked=%d  T=0x%04x  Lmem@AR5 hi=0x%04x lo=0x%04x\n"
                  "[c54x]   shadow=%s hi%+d lo%+d packed=%010llx insn=%u\n",
                  sd_n, exec_pc, exec_op, s->xpc & 0xFF,
                  !!(s->st1 & ST1_C16), !!(s->st1 & ST1_FRCT), asm5, s->bk,
                  (unsigned long long)a0u, (unsigned long long)(a1 & 0xFFFFFFFFFFULL),
                  (long long)(a1 - sd_a0), shiftLike,
                  (unsigned long long)(sd_b0 & 0xFFFFFFFFFFULL),
                  (unsigned long long)(b1 & 0xFFFFFFFFFFULL), (long long)(b1 - sd_b0),
                  sd_ar5_0, s->ar[5], (s->ar[5] != sd_ar5_0),                       /* OBS2 */
                  s->t, sd_lhi, sd_llo, is_dadst ? "DADST" : "DSADT", sh_hi, sh_lo,
                  (unsigned long long)(shadow & 0xFFFFFFFFFFULL), s->insn_count);
            }
            sd_n++;
        }

        /* DECODE-AUDIT: decoder inventory over the correlator overlay
         * 0x8000-0x9FFF. Logs once per distinct (PC, op) the raw opcode, the
         * next word and the LENGTH consumed (including lk_used), to diff
         * against doc/opcodes/tic54x_hi8_map.md. Check the length column
         * first: a wrong length desynchronises the whole following stream, as
         * the 0x86/0x87 bug did before it turned into an SP runaway. The point
         * is to clear the whole decode/length/mode class at once instead of
         * peeling off one bug at a time. Deduplicated by (PC, op), so the XPC
         * variants of one page are captured too. Gate
         * CALYPSO_DEBUG=DECODE-AUDIT; cost is negligible once covered. */
        static int da_lo = -1, da_hi = -1;
        static long long da_insn = -1;
        if (da_lo < 0) {
            const char *l = getenv("CALYPSO_DA_LO"); const char *h = getenv("CALYPSO_DA_HI");
            const char *n = getenv("CALYPSO_DA_INSN");
            da_lo = l ? (int)strtol(l, NULL, 0) : 0x8000;   /* correlator overlay by default */
            da_hi = h ? (int)strtol(h, NULL, 0) : 0x9FFF;   /* CALYPSO_DA_LO/HI widen it (e.g. 0x7000..0xFFFF) */
            da_insn = n ? strtoll(n, NULL, 0) : 0;          /* CALYPSO_DA_INSN: skip the boot, aim at the detection window (e.g. 250000000) */
        }
        if (exec_pc >= (uint16_t)da_lo && exec_pc <= (uint16_t)da_hi
            && s->insn_count >= (uint64_t)da_insn
            && calypso_debug_enabled("DECODE-AUDIT")) {
            static uint16_t da_op_seen[0x10000];
            static uint8_t  da_has[0x10000];
            static unsigned da_n = 0;
            uint16_t da_op = prog_fetch(s, exec_pc);
            /* Skip op == 0x0000: empty PROM holes, a control-flow runaway
             * rather than a DECODE bug. They used to consume the cap during
             * boot on the 0xCB00-0xD3FF sweep. */
            if (da_op != 0x0000 && da_n < 4000
                && (!da_has[exec_pc] || da_op_seen[exec_pc] != da_op)) {
                da_has[exec_pc] = 1;
                da_op_seen[exec_pc] = da_op;
                da_n++;
                fprintf(stderr, "[c54x] DECODE-AUDIT PC=0x%04x op=%04x op2=%04x "
                        "hi8=%02x len=%d XPC=%u insn=%u\n",
                        exec_pc, da_op, prog_fetch(s, exec_pc + 1),
                        (da_op >> 8) & 0xFF, consumed, s->xpc & 0xFF,
                        s->insn_count);
            }
        }
        /* INTM-TRANS: every toggle of the INTM bit (ST1 bit 11) with the PC
         * and opcode that caused it. Answers "does INTM ever reach 0, and if
         * so who re-arms it". Silent unless CALYPSO_INTM_TRANS is set. */
        {
            static int g_intm_prev_tr = -1, g_intm_tr_en = -1;
            if (g_intm_tr_en < 0) { const char *e = getenv("CALYPSO_INTM_TRANS");
                                    g_intm_tr_en = (e && *e != 0) ? 1 : 0; }
            int intm_now_tr = !!(s->st1 & ST1_INTM);
            if (intm_now_tr != g_intm_prev_tr && g_intm_tr_en) {
                fprintf(stderr, "[c54x] INTM-TRANS %d->%d PC=0x%04x op=0x%04x "
                        "IFR=0x%04x IMR=0x%04x insn=%u\n",
                        g_intm_prev_tr, intm_now_tr, exec_pc, exec_op,
                        s->ifr, s->imr, s->insn_count);
            }

            /* @BEQUILLE - INTM_ACK  (CALYPSO_INTM_ACK, default 0)
             *   masks   : three gestures that belong to the other side and never
             *             arrive. (a) The ARM firmware's "new IRQ agreement" epilogue
             *             does not reach our INTH on the native path - real hardware
             *             writes IRQ_CTRL bit 0 there. (b) The ARM write of
             *             d_dsp_page is swallowed by the 2-byte overlay registered at
             *             priority 10 on 0xFFD001A8, so the cell keeps its boot value
             *             and B_GSM_TASK never reaches the DSP - real firmware sets
             *             d_dsp_page = B_GSM_TASK|w_page in dsp_end_scenario().
             *             (c) No time base drives TINT0, because
             *             calypso_tint0_start() is never called. All three are
             *             triggered from the DSP here instead.
             *   remove  : once the d_dsp_page overlay is replaced by a real
             *             pass-through (the ARM writes, the DSP reads) and the ROM's
             *             TIMER0 is actually programmed (TCR TSS=0). Each gesture then
             *             goes back to its owner.
             *
             * Transition direction:
             *   0->1 (interrupt taken) -> ACK ARM: the DSP accepts the request raised
             *                       by the ARM/TPU; close the agreement on the INTH
             *                       side and set d_dsp_page = B_GSM_TASK|w_page.
             *   1->0 (RETE)       -> ACK DSP: the DSP has finished its ISR; release
             *                       the frame interrupt LEVEL hold and flip the read
             *                       page (mirror of r_page ^= 1).
             *   either            -> TINT0 tick (vec 20 / IMR bit 4).
             *
             * CALYPSO_INTM_ACK_SWAP=1 swaps the two directions.
             * CALYPSO_INTM_ACK_NO_TINT0=1 keeps the acks without the tick.
             */
            if (intm_now_tr != g_intm_prev_tr && g_intm_prev_tr >= 0) {
                static int _ia = -1, _iswap = -1, _int0 = -1;
                static int _iarm = -1, _idsp = -1, _idpage = -1;
                static unsigned _idpage_every = 0;
                static int _busy = 0;
                if (_ia < 0) {
                    _ia    = calypso_gate("CALYPSO_INTM_ACK", 0);
                    _iswap = calypso_gate("CALYPSO_INTM_ACK_SWAP", 0);
                    _int0  = getenv("CALYPSO_INTM_ACK_NO_TINT0") ? 0 : 1;
                    /* Each gesture is separately switchable, for A/B. Default
                     * follows INTM_ACK, except the d_dsp_page write, which is
                     * explicitly opt-in.
                     *   CALYPSO_INTM_ACK_NO_ARM=1    disable the ARM ack (INTH)
                     *   CALYPSO_INTM_ACK_NO_DSP=1    disable the DSP ack (level hold)
                     *   CALYPSO_INTM_ACK_DPAGE=1     enable the d_dsp_page write
                     *   CALYPSO_INTM_ACK_DPAGE_EVERY default 65536 insns (= the
                     *                                measured TIMER0 period = frame
                     *                                cadence)
                     *
                     * The period matters: writing d_dsp_page on every interrupt taken
                     * fires ~1000 times, whereas dsp_end_scenario() writes it ONCE PER
                     * FRAME (osmocom-bb layer1/sync.c -> calypso/dsp.c:471). That
                     * hammered the DSP synchronisation cell at ~50x the frame
                     * cadence. */
                    _iarm   = getenv("CALYPSO_INTM_ACK_NO_ARM") ? 0 : 1;
                    _idsp   = getenv("CALYPSO_INTM_ACK_NO_DSP") ? 0 : 1;
                    _idpage = calypso_gate("CALYPSO_INTM_ACK_DPAGE", 0);
                    { const char *_e = getenv("CALYPSO_INTM_ACK_DPAGE_EVERY");
                      _idpage_every = (_e && *_e) ? (unsigned)strtoul(_e, NULL, 0) : 65536u;
                      if (_idpage_every < 1) _idpage_every = 65536u; }
                    if (_ia)
                        fprintf(stderr, "[c54x] INTM_ACK=1 (BEQUILLE) : ack ARM sur "
                                "%s (INTH=%d dpage=%d/%u insn), ack DSP sur %s (=%d), "
                                "tick TINT0=%d\n",
                                _iswap ? "1->0 (RETE)" : "0->1 (IT prise)",
                                _iarm, _idpage, _idpage_every,
                                _iswap ? "0->1 (IT prise)" : "1->0 (RETE)", _idsp, _int0);
                }
                if (_ia && !_busy) {
                    _busy = 1;
                    int _entree = _iswap ? (intm_now_tr == 0) : (intm_now_tr == 1);

                    if (_entree) {
                        /* --- ACK ARM ------------------------------------- */
                        if (_iarm) calypso_inth_arm_ack();
                        /* d_dsp_page = B_GSM_TASK | w_page (dsp.c:471), at most
                         * once per frame - see the note above. */
                        if (_idpage) {
                            static unsigned _last = 0;
                            if (_last == 0 || (s->insn_count - _last) >= _idpage_every) {
                                static uint16_t _wpage = 0;
                                uint16_t _v = (uint16_t)(0x0002 | _wpage);
                                s->data[0x08D4] = _v;
                                if (s->api_ram) s->api_ram[0x08D4 - C54X_API_BASE] = _v;
                                _wpage ^= 1;                  /* w_page ^= 1 */
                                _last = s->insn_count;
                                static unsigned _n = 0;
                                if (_n++ < 40)
                                    fprintf(stderr, "[c54x] INTM_ACK ARM : IRQ agreement + "
                                            "d_dsp_page=0x%04x (B_GSM_TASK|w_page) PC=0x%04x "
                                            "insn=%u\n", _v, exec_pc, s->insn_count);
                            }
                        }
                    } else if (_idsp) {
                        /* --- ACK DSP ------------------------------------- */
                        g_frame_it_level = false;   /* release the LEVEL hold */
                        if (g_last_intr_vec >= 0) {
                            /* end of service: the source that was served drops */
                            if (g_last_intr_vec == 28)
                                c54x_ifr_clear(s, (uint16_t)(1u << 12), "intm-ack-dsp");
                            g_last_intr_vec = -1;
                        }
                        /* mirror of sync.c: r_page ^= 1 after consumption */
                        {
                            static uint16_t _rpage = 0;
                            _rpage ^= 1;
                            static unsigned _n = 0;
                            if (_n++ < 40)
                                fprintf(stderr, "[c54x] INTM_ACK DSP : level hold relache, "
                                        "r_page=%u PC=0x%04x insn=%u\n",
                                        _rpage, exec_pc, s->insn_count);
                        }
                    }

                    /* --- TINT0 tick (vec 20 / IMR bit 4) ------------------ */
                    if (_int0) {
                        static unsigned _n = 0;
                        if (_n++ < 40)
                            fprintf(stderr, "[c54x] INTM_ACK TINT tick "
                                    "IMR=0x%04x IFR=0x%04x insn=%u\n",
                                    s->imr, s->ifr, s->insn_count);
                        c54x_fire_tint(s);   /* honours the IMR; SPRU131 5.1: bit 3 / vec 19 */
                    }
                    _busy = 0;
                }
            }

            g_intm_prev_tr = intm_now_tr;
        }

        /* SP event ring: records every SP change (push or pop) with the PC and
         * opcode responsible. Feeds the BLACKHOLE-CALA dump. */
        if (s->sp != sp_before_exec) {
            struct sp_evt *e = &g_spring[g_spring_idx++ & 63];
            e->pc = exec_pc;
            e->op = prog_fetch(s, exec_pc);
            e->delta = (int16_t)(s->sp - sp_before_exec);
            e->sp = s->sp;
            g_sp_ledger.net_words += (int16_t)(sp_before_exec - s->sp);
            if ((int16_t)(s->sp - sp_before_exec) < 0) g_sp_ledger.sp_pushes++;
            else g_sp_ledger.sp_pops++;

            /* SP diving into the API RAM zone (0x0700-0x0a00) is corruption: a
             * CALLD push then clobbers d_fb_det / d_fb_mode (0x08f8, 0x08f9).
             * Logs the instruction that brings SP in (the transition from
             * outside the zone), the delta and the stack neighbourhood, which
             * names the offending SP setter. */
            {
                static uint32_t spdz_n = 0;
                int in_zone   = (s->sp >= 0x0700 && s->sp <= 0x0a00);
                int was_out   = (sp_before_exec < 0x0700 || sp_before_exec > 0x0a00);
                if (in_zone && was_out && spdz_n < 30) {
                    fprintf(stderr, "[c54x] SP-DANGER SP 0x%04x→0x%04x (delta=%+d) "
                            "PC=0x%04x op=0x%04x XPC=%u insn=%u\n",
                            sp_before_exec, s->sp,
                            (int)(int16_t)(s->sp - sp_before_exec),
                            exec_pc, prog_fetch(s, exec_pc), s->xpc, s->insn_count);
                    spdz_n++;
                }
            }

            /* Shadow stack: pairs pushes with pops (CALYPSO_ORPHAN). Names THE
             * orphan return (the over-pop), not the 15 victims at 0xc8be. */
            if (g_shadow_on < 0) {
                const char *eo = getenv("CALYPSO_ORPHAN");  /* its own env, outside CALYPSO_DEBUG */
                g_shadow_on = (eo && *eo) ? 1 : 0;
            }
            if (g_shadow_on) {
                uint16_t op = e->op;
                int16_t  d  = e->delta;
                int is_call = (op==0xF074||op==0xF274||op==0xF4E3||op==0xF4E7||op==0xF6E3);
                int is_ret  = (op==0xFC00||op==0xFE00||op==0xF4EB||op==0xF4E4||op==0xF6EB
                               ||(op&0xFF00)==0xFC00);   /* RET/RETD/RETE/FRET/RETED plus conditional RC */
                int is_pshm = ((op&0xFF00)==0x4A00||(op&0xFF00)==0x4B00);
                (void)is_call;
                if (d < 0) {                 /* PUSH: SP went down */
                    int words = -d, w;
                    char kind = is_pshm ? 'P' : 'C';   /* PSHM pushes data; anything else a return address */
                    for (w = 0; w < words; w++) {
                        if (g_shadow_depth >= 0 && g_shadow_depth < SHADOW_N) {
                            g_shadow[g_shadow_depth].pc   = exec_pc;
                            g_shadow[g_shadow_depth].op   = op;
                            g_shadow[g_shadow_depth].sp   = s->sp;
                            g_shadow[g_shadow_depth].kind = kind;
                        }
                        g_shadow_depth++;
                    }
                } else if (d > 0) {          /* POP: SP went up */
                    int words = d, w;
                    for (w = 0; w < words; w++) {
                        g_shadow_depth--;
                        if (is_ret) {
                            if (g_shadow_depth < 0) {
                                g_orphan_hits++;
                                if (g_orphan_hits <= 40) {
                                    /* Return target: RETD/RETED arm delayed_pc
                                     * (deferred commit); RET/FRET commit
                                     * immediately into s->pc. */
                                    uint16_t ret_tgt = (s->delay_slots ? s->delayed_pc : s->pc);
                                    /* Last real PUSH in g_spring (the missing
                                     * matching CALL): scan backwards for
                                     * delta < 0. */
                                    uint16_t lp_pc = 0, lp_op = 0; int lp_found = 0, scan;
                                    for (scan = 1; scan <= 64; scan++) {
                                        struct sp_evt *pe = &g_spring[(g_spring_idx - scan) & 63];
                                        if (pe->delta < 0) { lp_pc = pe->pc; lp_op = pe->op;
                                                             lp_found = 1; break; }
                                    }
                                    fprintf(stderr,
                                        "[c54x] ORPHAN-RETURN #%llu insn=%u pc=0x%04x op=0x%04x "
                                        "SP=0x%04x → ret_tgt=0x%04x  lastPUSH=%s(pc=0x%04x op=0x%04x) "
                                        "net_words=%lld — over-pop (pile vierge au-dessus de SP_base)\n",
                                        (unsigned long long)g_orphan_hits, s->insn_count,
                                        exec_pc, op, s->sp, ret_tgt,
                                        lp_found ? "" : "AUCUN", lp_pc, lp_op,
                                        (long long)g_sp_ledger.net_words);
                                    /* Slot this return reads: written (a
                                     * legitimate vector) or untouched (real
                                     * garbage)? */
                                    {
                                        uint16_t rs = (uint16_t)(s->sp - 1);
                                        if (rs >= STKSLOT_LO && rs <= STKSLOT_HI) {
                                            int si = rs - STKSLOT_LO;
                                            if (g_stkslot_written[si])
                                                fprintf(stderr, "[c54x]     slot 0x%04x ÉCRIT par "
                                                    "ST@pc=0x%04x op=0x%04x → VECTEUR LÉGIT (pas un bug)\n",
                                                    rs, g_stkslot_wpc[si], g_stkslot_wop[si]);
                                            else
                                                fprintf(stderr, "[c54x]     slot 0x%04x JAMAIS écrit "
                                                    "→ VIERGE = vrai over-pop garbage\n", rs);
                                        }
                                    }
                                    /* On the very first orphan, dump the whole
                                     * g_spring ring (reset to over-pop) so
                                     * pushes and pops can be counted directly:
                                     * structural imbalance versus a single
                                     * bug. */
                                    if (g_orphan_hits == 1) {
                                        int k;
                                        fprintf(stderr, "[c54x]   g_spring (anciens→récents, reset→#1):\n");
                                        for (k = 64; k >= 1; k--) {
                                            struct sp_evt *pe = &g_spring[(g_spring_idx - k) & 63];
                                            if (pe->pc == 0 && pe->op == 0 && pe->delta == 0) continue;
                                            fprintf(stderr, "[c54x]     pc=0x%04x op=0x%04x %s%d SP→0x%04x\n",
                                                    pe->pc, pe->op, pe->delta < 0 ? "PUSH" : "POP ",
                                                    pe->delta < 0 ? -pe->delta : pe->delta, pe->sp);
                                        }
                                    }
                                }
                            } else if (g_shadow[g_shadow_depth].kind != 'C') {
                                g_mismatch_hits++;
                                if (g_mismatch_hits <= 40)
                                    fprintf(stderr,
                                        "[c54x] MISMATCH-RETURN #%llu insn=%u pc=0x%04x op=0x%04x "
                                        "SP=0x%04x dépile kind='%c' poussé par pc=0x%04x op=0x%04x — "
                                        "return lit une valeur non-retour (PSHM)\n",
                                        (unsigned long long)g_mismatch_hits, s->insn_count,
                                        exec_pc, op, s->sp, g_shadow[g_shadow_depth].kind,
                                        g_shadow[g_shadow_depth].pc, g_shadow[g_shadow_depth].op);
                            }
                        }
                    }
                }
                if (g_shadow_depth < 0) g_shadow_depth = 0;  /* re-anchor after an orphan */
            }
        }

        /* B4B (CALYPSO_B4B): traces the flow AFTER the detector, from 0x9ac0
         * to 0xec07 (the frequency decision plus the write to 0x08f8) or into
         * a loop, plus a one-shot opcode dump from 0x9ac0 for disassembly. */
        {
            /* SCAN-08F8 (CALYPSO_SCAN_08F8): one-shot scan of the current bank
             * for instructions holding the word 0x08f8, the address of
             * d_fb_det. Says whether a writer of d_fb_det exists at all, and
             * at which PC. */
            static int _sc = -1; static int _scdone = 0;
            if (_sc < 0) _sc = calypso_gate("CALYPSO_SCAN_08F8", 0);
            if (_sc && !_scdone && exec_pc == 0x9ac0) {
                _scdone = 1;
                unsigned _hits = 0;
                for (uint32_t _p = 0x7000; _p <= 0xfffe; _p++) {
                    if (prog_fetch(s, (uint16_t)_p) == 0x08f8) {
                        fprintf(stderr, "[c54x] SCAN-08F8 word@0x%04x=0x08f8  prev=0x%04x prev2=0x%04x\n",
                                _p, prog_fetch(s, (uint16_t)(_p-1)), prog_fetch(s, (uint16_t)(_p-2)));
                        if (++_hits > 40) break;
                    }
                }
                fprintf(stderr, "[c54x] SCAN-08F8 total hits(bank%u)=%u\n", s->xpc, _hits);
            }
            static int _b4b = -1; static int _armed = 0; static unsigned _b4bn = 0; static int _opd = 0;
            if (_b4b < 0) _b4b = calypso_gate("CALYPSO_B4B", 0);
            if (_b4b && exec_pc == 0x9ac0) {
                _armed = 1;
                if (!_opd) { _opd = 1;
                    fprintf(stderr, "[c54x] B4B-OPDUMP 0x9ac0..0x9adf:");
                    for (int _k = 0; _k < 32; _k++) fprintf(stderr, " %04x", prog_fetch(s, (uint16_t)(0x9ac0 + _k)));
                    fprintf(stderr, "\n");
                }
            }
            if (_b4b && _armed && _b4bn < 600) {
                _b4bn++;
                fprintf(stderr, "[c54x] B4B-FLOW pc=0x%04x xpc=%u op=0x%04x A=%lld insn=%u\n",
                        s->pc, s->xpc, prog_fetch(s, s->pc), (long long)(s->a & 0xFFFFFFFFFFULL), s->insn_count);
                if ((s->pc == 0xec07) || (s->pc >= 0x8d00 && s->pc <= 0x8d10)) _armed = 0;
            }
        }
        if (exec_pc == 0xa076) {   /* MAC kernel: reads the operands (I/Q plus coefficients) */
            g_flow_armed = 1;   /* FLOWTRACE: arm the window around the detector */
            static int _b2k = -1; static unsigned _b2kn = 0;
            if (_b2k < 0) _b2k = calypso_gate("CALYPSO_B2AR", 0);
            /* The counter lives outside the gate: min/max must cover the
             * WHOLE run, not stop at the first iteration. */
            static uint16_t _ar5min = 0xffff, _ar5max = 0;
            static unsigned _ar5seen = 0, _ar5in = 0;
            if (s->ar[5] < _ar5min) _ar5min = s->ar[5];
            if (s->ar[5] > _ar5max) _ar5max = s->ar[5];
            _ar5seen++;
            if (s->ar[5] >= 0x2a00 && s->ar[5] < 0x2b28) _ar5in++;
            if (_b2k && (_ar5seen % 20000) == 0)
                fprintf(stderr, "[c54x] B2AR5-RANGE n=%u min=0x%04x max=0x%04x IN_BUF=%u (buf=0x2a00..0x2b27)\n",
                        _ar5seen, _ar5min, _ar5max, _ar5in);
            if (_b2k && _b2kn < 16) {
                _b2kn++;
                #define _INB(a) (((a) >= 0x2a00 && (a) < 0x2b28) ? "IN" : "oob")
                fprintf(stderr, "[c54x] B2KERN @0xa076 AR2=%04x[%d]%s AR3=%04x[%d]%s AR4=%04x[%d]%s AR5=%04x[%d]%s\n",
                        s->ar[2],(int)(int16_t)s->data[s->ar[2]],_INB(s->ar[2]),
                        s->ar[3],(int)(int16_t)s->data[s->ar[3]],_INB(s->ar[3]),
                        s->ar[4],(int)(int16_t)s->data[s->ar[4]],_INB(s->ar[4]),
                        s->ar[5],(int)(int16_t)s->data[s->ar[5]],_INB(s->ar[5]));
                #undef _INB
            }
        }
        {   /* ARWATCH (CALYPSO_ARWATCH): at five correlator PCs, shows which
             * AR point inside the burst buffer [0x2a00..0x2b28) - '*' in, '.'
             * out. */
            static int _aw = -1; static unsigned _awn = 0;
            if (_aw < 0) _aw = calypso_gate("CALYPSO_ARWATCH", 0);
            if (_aw && _awn < 60 &&
                (exec_pc == 0x8d00 || exec_pc == 0x8d1a || exec_pc == 0x8e5f ||
                 exec_pc == 0x8e8c || exec_pc == 0x8e97)) {
                _awn++;
                char _in[9]; int _k;
                for (_k = 0; _k < 8; _k++)
                    _in[_k] = (s->ar[_k] >= 0x2a00 && s->ar[_k] < 0x2b28) ? '*' : '.';
                _in[8] = 0;
                fprintf(stderr, "[c54x] ARWATCH pc=0x%04x AR0=%04x AR1=%04x AR2=%04x AR3=%04x "
                        "AR4=%04x AR5=%04x AR6=%04x AR7=%04x [%s] A=0x%06llx insn=%u\n",
                        exec_pc, s->ar[0], s->ar[1], s->ar[2], s->ar[3],
                        s->ar[4], s->ar[5], s->ar[6], s->ar[7], _in,
                        (unsigned long long)(s->a & 0xFFFFFFULL), s->insn_count);
            }
        }
        {   /* DISPATCH-PROBE (CALYPSO_DISPATCH_PROBE=1, default 0, read-only).
             *
             * The ROM's RX arming routine, 0xa5cd, installs the trampoline
             * data[0x0158] and programs DMA2_AAD/ALGTH/CTRL(ENABLE=1). It
             * exists, it is complete, and it is NEVER executed. It is an entry
             * of the dispatch table, copied at init from PROGRAM memory into
             * DATA:
             *     0xb4b6 : program[0xaae7..0xab34] -> data[0x4387..0x43d4]
             * so program[0xab10] = 0xa5cd lands in data[0x43b0], index 41.
             *
             * The dispatcher repeats the same pattern in several places:
             *     sub #k1,A ; bc ...,AGT ; sub #k2,A ; add #0x4387,A
             *     stlm A,AR3 ; ld *AR3,A ; cala A
             * i.e. a range-based switch. The question is which value reaches A,
             * and static analysis cannot answer it. The switch sites are the
             * `add #0x4387`: A is read BEFORE the addition, giving the raw
             * index, and the handler it designates is resolved. */
            static int _dp = -1;
            if (_dp < 0) {
                _dp = calypso_gate("CALYPSO_DISPATCH_PROBE", 0);
                if (_dp)
                    fprintf(stderr, "[dispatch] sonde armee : 7 sites d'aiguillage "
                            "(add #0x4387), l'armement RX 0xa5cd, l'arm/desarm DMA "
                            "0xa620/0xa62e, le publieur d_fb_det 0x79e4, l'init de "
                            "table 0xb4b6, et les acces au handler courant "
                            "data[0x43d8] (0xb01d lit, 0xbb01 ecrit). "
                            "LECTURE SEULE.\n");
            }
            if (_dp) {
                /* --- the switch sites: A holds the index before the `add` --- */
                /* Five sites, not seven: at 0xb0f1 and 0xb0ff the ROM scan
                 * shows the operand 0x4387 preceded by opcode 0xf000
                 * (`ld #k16,A`), not 0xf200 (`add #k16,A`) as at the other
                 * five. Those two load the table base rather than switch, and
                 * A there already holds a resolved table address (0x43ac =
                 * base + 37), which is where the absurd index=17324 came
                 * from. */
                {   /* CHAIN-B05F: the missing link, traced step by step.
                     *
                     * Measured state: the ARM commands d_task_d = 0x0018 (24,
                     * ALLC); the DSP reads the cell and does see 0x0018 (35 of
                     * 41 samples); the table holds the right handler at index
                     * 41 (data[0x43b0] = 0xa5cd). Yet the five _sw[] switch
                     * sites below - the index resolution - are NEVER reached,
                     * so RX arming is never requested.
                     *
                     * In between, the block that reads d_task_d at 0xb05f runs
                     * a comparison chain (ROM dump):
                     *     0xb062: f130 7fff
                     *     0xb064: f210 000c   0xb066: f843 b077   ; tests 12
                     *     0xb068: f210 0022   0xb06a: f846 b077   ; tests 34
                     *     0xb06c: f210 001e   0xb06e: f842 b070   ; tests 30
                     *     0xb070: f200 4387                       ; table base
                     * The value read is 24 - neither 12, nor 34, nor 30.
                     *
                     * Decoding the branch conditions f843/f846/f842 by hand is
                     * exactly what produced three false leads out of three, so
                     * the path actually taken and the accumulator at each step
                     * are measured instead.
                     *
                     * The whole segment 0xb05f..0xb078 is traced, so a block
                     * that never executes leaves the probe silent for a
                     * different reason - and the periodic counter says so.
                     * 200 steps, plus one summary every 20 passes. */
                    if (exec_pc >= 0xb05f && exec_pc <= 0xb078) {
                        static unsigned long long _cn = 0, _pass = 0;
                        if (exec_pc == 0xb05f) _pass++;
                        if (_cn < 200) {
                            _cn++;
                            /* The ARs are printed because 0xb060 is
                             * `10e1 0000`, an INDIRECT LD: the block reads its
                             * task code THROUGH a pointer, and the measurement
                             * gives A = 0x5294, outside the range of the
                             * comparisons (12/30/34), hence the systematic
                             * bailout to 0xb077. The question is therefore what
                             * the register points at, not what the comparison
                             * yields. The cell pointed at by AR[ARP] is printed
                             * too. */
                            unsigned _arp = (s->st0 >> 13) & 7;
                            uint16_t _ea  = s->ar[_arp];
                            fprintf(stderr,
                                    "[dispatch] CHAIN-B05F pc=0x%04x op=0x%04x "
                                    "A=0x%06llx (bas=0x%04x) TC=%d ARP=%u "
                                    "AR[ARP]=0x%04x *AR[ARP]=0x%04x "
                                    "AR0=%04x AR1=%04x AR2=%04x AR3=%04x "
                                    /* data[0x7fff] tests one hypothesis. At
                                     * 0xb062, `f130 7fff` overwrites A with
                                     * 0x5294, a CONSTANT whatever the input
                                     * (checked before and after the api_ram
                                     * fix). A result independent of the operand
                                     * means the opcode is not computing, it is
                                     * READING. If data[0x7fff] holds 0x5294,
                                     * the long immediate is being treated as an
                                     * ADDRESS instead of a value - the same
                                     * class as the LDU *(0x0ffe) bug documented
                                     * at the head of the F1xx handler. If it
                                     * does not match, the hypothesis falls and
                                     * the handler itself must be
                                     * instrumented. */
                                    "data[0x7fff]=0x%04x "
                                    "passage=%llu insn=%u\n",
                                    exec_pc, prog_fetch(s, exec_pc),
                                    (unsigned long long)(s->a & 0xFFFFFFULL),
                                    (unsigned)(s->a & 0xFFFF),
                                    (s->st0 >> 12) & 1, _arp, _ea, s->data[_ea],
                                    s->ar[0], s->ar[1], s->ar[2], s->ar[3],
                                    s->data[0x7fff],
                                    _pass, s->insn_count);
                            fflush(stderr);
                        } else if (exec_pc == 0xb05f && (_pass % 20) == 0) {
                            fprintf(stderr,
                                    "[dispatch] CHAIN-B05F resume : %llu passages en "
                                    "0xb05f, %llu pas traces (plafond)\n", _pass, _cn);
                            fflush(stderr);
                        }
                    }
                }
                static const uint16_t _sw[] = {0xb070,0xb08d,0xb0aa,0xb0be,0xb0d1};
                for (unsigned _i = 0; _i < sizeof(_sw)/sizeof(_sw[0]); _i++) {
                    if (exec_pc != _sw[_i]) continue;
                    uint16_t idx  = (uint16_t)(s->a & 0xFFFF);
                    uint16_t addr = (uint16_t)(0x4387 + idx);
                    uint16_t hnd  = s->data[addr];
                    /* Deduplicate by (site, index): a stable switch must not
                     * flood the log. */
                    static struct { uint16_t pc, idx; unsigned long long n; } _seen[64];
                    static int _n = 0;
                    int _k = -1;
                    for (int _j = 0; _j < _n; _j++)
                        if (_seen[_j].pc == exec_pc && _seen[_j].idx == idx) { _k = _j; break; }
                    if (_k >= 0) {
                        if (++_seen[_k].n % 20000 == 0)
                            fprintf(stderr, "[dispatch] site 0x%04x index=%u × %llu\n",
                                    exec_pc, idx, _seen[_k].n);
                    } else {
                        if (_n < 64) { _seen[_n].pc = exec_pc; _seen[_n].idx = idx;
                                       _seen[_n].n = 1; _n++; }
                        fprintf(stderr, "[dispatch] *** site 0x%04x  index=%u  "
                                "-> data[0x%04x] = 0x%04x%s  A=0x%06llx insn=%u\n",
                                exec_pc, idx, addr, hnd,
                                (hnd == 0xa5cd) ? "  <<< ARMEMENT RX !" :
                                (hnd == 0xab38) ? "  (slot vide, RET partage)" : "",
                                (unsigned long long)(s->a & 0xFFFFFFULL), s->insn_count);
                    }
                }
                /* --- every computed call or branch, with its target -------
                 * Measured: none of the five switch sites fires, and yet the
                 * table handler 0xa62e EXECUTES (A = 0xffa62e, the address
                 * itself, the signature of a cala). So another call path
                 * exists. 0xa62e is reachable neither by a direct CALL/B (no
                 * reference in the image) nor by fall-through (0xa62d is a
                 * ret), which leaves a computed branch.
                 * tic54x-opc.c gives four forms, mask 0xFEFF, bit 8 selecting
                 * the source accumulator:
                 *     bacc  0xF4E2   baccd 0xF6E2   cala 0xF4E3   calad 0xF6E3
                 * All four are covered, on A as well as B - the whole class,
                 * not just the expected case. Deduplicated by (calling PC,
                 * target). */
                {
                  uint16_t _op = prog_fetch(s, exec_pc);
                  uint16_t _base = (uint16_t)(_op & 0xFEFF);
                  const char *_kind = (_base==0xF4E2) ? "bacc"  :
                                      (_base==0xF6E2) ? "baccd" :
                                      (_base==0xF4E3) ? "cala"  :
                                      (_base==0xF6E3) ? "calad" : NULL;
                  if (_kind) {
                    int64_t _src = (_op & 0x0100) ? s->b : s->a;
                    uint16_t tgt = (uint16_t)(_src & 0xFFFF);
                    static struct { uint16_t pc, tgt; unsigned long long n; } _c[96];
                    static int _cn = 0;
                    int _k = -1;
                    for (int _j = 0; _j < _cn; _j++)
                        if (_c[_j].pc == exec_pc && _c[_j].tgt == tgt) { _k = _j; break; }
                    if (_k >= 0) {
                        if (++_c[_k].n % 20000 == 0)
                            fprintf(stderr, "[dispatch] %s 0x%04x -> 0x%04x × %llu\n",
                                    _kind, exec_pc, tgt, _c[_k].n);
                    } else {
                        if (_cn < 96) { _c[_cn].pc = exec_pc; _c[_cn].tgt = tgt;
                                        _c[_cn].n = 1; _cn++; }
                        const char *q = (tgt==0xa5cd) ? "  <<< ARMEMENT RX !"
                                      : (tgt==0xa62e) ? "  (desarmement DMA)"
                                      : (tgt==0xab38) ? "  (RET partage, slot vide)"
                                      : (tgt==0xb5a1) ? "  (greffe #2)"
                                      : (tgt==0xa4c7) ? "  (graine go-live)" : "";
                        fprintf(stderr, "[dispatch] *** %-5s depuis 0x%04x -> 0x%04x%s"
                                "  (%s)  data[0x43d8]=0x%04x insn=%u\n",
                                _kind, exec_pc, tgt, q,
                                (_op & 0x0100) ? "src=B" : "src=A",
                                s->data[0x43d8], s->insn_count);
                    }
                  }
                }

                /* --- the notable sites, simple pass counting --------------- */
                {
                    static const struct { uint16_t pc; const char *quoi; } _pt[] = {
                      {0xa5cd, "ARMEMENT RX (tremplin 0x0158 + DMA2_AAD)"},
                      {0xa5e8, "  ecriture DMA2_AAD"},
                      {0xa5f6, "  ecriture DMA2_CTRL (ENABLE=1)"},
                      {0xa620, "SPCR : RDMA_MASK=0 (mode DMA arme)"},
                      {0xa62e, "SPCR : RDMA_MASK=1 (mode DMA coupe)"},
                      {0xa601, "test data[0x435e] bit13 (verrou config DMA)"},
                      {0x79e4, "publication d_fb_det (data[0x08f8] |= 1)"},
                      {0x79e3, "  gate : data[@0x7e] doit valoir 4"},
                      {0xb4b6, "init : recopie de la table -> data[0x4387]"},
                      {0xb01d, "lecture du handler courant data[0x43d8]"},
                      {0xbb00, "ECRITURE du handler courant data[0x43d8]"},

                      /* The RX arming chain. Index 41
                       * (`ld #0x29,A ; call 0xa9ea`) leads to 0xa5cd. Four
                       * sites request it, none is reached. The sites
                       * themselves, the ENTRY of the block containing them and
                       * the LOADER that puts that entry into A upstream are all
                       * probed - otherwise it is impossible to say where the
                       * chain breaks. */
                      {0xb220, "site 1 : ld #0x29 -> ARMEMENT RX (AR3=0x00bf)"},
                      {0xb216, "  entree du bloc du site 1"},
                      {0xb2b4, "site 2 : ld #0x29 -> ARMEMENT RX (AR3=0x0097)"},
                      {0xb2aa, "  entree du bloc du site 2"},
                      {0xb330, "site 3 : ld #0x29 -> ARMEMENT RX (AR3=0x0030)"},
                      {0xb35d, "site 4 : ld #0x29 -> ARMEMENT RX (AR3=0x0040)"},
                      /* the `ld #k16,A` that load the entry of block 1, then 2 */
                      {0xaba9, "  chargeur -> 0xb216"},
                      {0xabc9, "  chargeur -> 0xb216"},
                      {0xabf1, "  chargeur -> 0xb2aa"},
                      {0xabf5, "  chargeur -> 0xb2aa"},
                      {0xabf9, "  chargeur -> 0xb2aa"},
                      {0xabfd, "  chargeur -> 0xb2aa"},
                      {0xb758, "  chargeur -> 0xb2aa"},
                      /* the dispatcher itself: how many times, with which index */
                      {0xa9ea, "helper index->handler (A = base+index apres le add)"},

                      /* The task queue. The main loop at 0xa4ca does
                       * `call 0xaad5` (dequeue) and, when A != 0, `calad A`.
                       * 0xaad5 dequeues from a ring:
                       *     AR1 = data[0x434f] (write)  AR0 = data[0x434e] (read)
                       *     empty -> A = 0 -> the loop spins doing nothing
                       * Measured: 31000 iterations with an empty queue. The
                       * question is therefore not why a given handler does not
                       * run, but WHO ENQUEUES. The enqueue is 0xaac3 (39
                       * callers), a ring of 14, the handler arrives in A, and
                       * overflow sets bit 5 of data[0x3f92]. */
                      {0xaac3, "EMPILEMENT d'une tache (A = handler empile)"},
                      {0xaaca, "  ecriture effective dans l'anneau"},
                      {0xaad1, "  ⚠ FILE PLEINE (data[0x3f92] bit5)"},
                      {0xaae3, "DEPILEMENT effectif (file non vide)"},
                      {0xaadd, "  file VIDE -> A=0, la boucle tourne a blanc"},
                      {0xa4cf, "boucle principale : calad du handler depile"},
                    };
                    for (unsigned _i = 0; _i < sizeof(_pt)/sizeof(_pt[0]); _i++) {
                        if (exec_pc != _pt[_i].pc) continue;
                        static unsigned long long _cnt[40];
                        unsigned long long c = ++_cnt[_i];
                        if (c == 1 || c % 20000 == 0 ||
                            exec_pc == 0xaac3 || exec_pc == 0xaad1)
                            fprintf(stderr, "[dispatch] PC=0x%04x ×%llu — %s "
                                    "(A=0x%06llx AR1=0x%04x AR2=0x%04x AR3=0x%04x "
                                    "d[0x3f92]=0x%04x file[r=0x%04x w=0x%04x]) insn=%u\n",
                                    exec_pc, c, _pt[_i].quoi,
                                    (unsigned long long)(s->a & 0xFFFFFFULL),
                                    s->ar[1], s->ar[2], s->ar[3],
                                    s->data[0x3f92], s->data[0x434e], s->data[0x434f],
                                    s->insn_count);
                    }
                }
            }
        }
        {   /* XPCWATCH (CALYPSO_XPCWATCH, default 0): does the DSP ever switch
             * program bank?
             *
             * The FB chain is traced end to end and stops in one place: the
             * task arms data[0x0158..0159] = "call 0x728a", that trampoline is
             * only reachable through interrupt slot 30 (data 0x00F8-0x00FB =
             * "fb 0x0158", copied from PDROM 0xe399 by the reada at 0xb4c9),
             * and that slot is never taken - IMR bit 14 (= vec 30 - 16, the
             * formula in calypso_dma.c:185) is permanently unmasked, yet the
             * IFR only ever takes 0x0020 and 0x0008. Both missing pieces live
             * in PROM1, loaded into PROGRAM at 0x18000, i.e. in a BANK: the
             * only reference in the whole silicon to 0xaae8 (base of the
             * handler table, table[5] = 0xab77) is at PROM1@0x1ab31, and a
             * fifth writer of 0x0158 at PROM1@0x19fe1. If XPC stays 0 for ever,
             * PROM1/2/3 are dead code here, which would explain the missing
             * handler installer, the publisher 0x79e4 never reached, and the
             * missing caller of slot 30 all at once.
             *
             * Measures (a) every XPC change with the PC and opcode that causes
             * it, 40 lines; (b) a periodic summary of the MASK of pages seen,
             * so that absence is a readable result rather than an inference:
             * "pages XPC vues = 0x00000001" means page 0 only, so no bank was
             * ever entered. Without that summary, zero lines from (a) would be
             * ambiguous between "no switching" and "probe not armed".
             * The far-branch family IS modelled (fb/fcall/fbacc) and the first
             * instruction of the run is an `fb` from the reset vector 0xff80,
             * so a motionless XPC is not a missing instruction. */
            static int _xw = -1; static uint16_t _xprev = 0xFFFF;
            static unsigned _xn = 0, _xbil = 0; static uint32_t _xvus = 0;
            if (_xw < 0) {
                _xw = calypso_gate("CALYPSO_XPCWATCH", 0);
                if (_xw)
                    fprintf(stderr, "[c54x] XPCWATCH arme (changements de banque "
                            "programme + bilan des pages vues)\n");
            }
            if (_xw) {
                uint16_t _x = (uint16_t)(s->xpc & 0xFF);
                if (_x < 32) _xvus |= (1u << _x);
                if (_x != _xprev) {
                    if (_xn++ < 40)
                        fprintf(stderr, "[c54x] XPCWATCH XPC 0x%02x -> 0x%02x PC=0x%04x "
                                "op=0x%04x insn=%u\n",
                                (unsigned)(_xprev == 0xFFFF ? 0u : _xprev), (unsigned)_x,
                                exec_pc, prog_fetch(s, exec_pc), s->insn_count);
                    _xprev = _x;
                }
                if ((s->insn_count % 5000000u) == 0 && _xbil < 12) {
                    _xbil++;
                    fprintf(stderr, "[c54x] XPCWATCH bilan insn=%u : pages XPC vues = "
                            "0x%08x (bit n = page n ; 0x00000001 = page 0 SEULE = aucune "
                            "banque jamais entree)\n",
                            s->insn_count, (unsigned)_xvus);
                }
            }
        }
        {   /* @BEQUILLE - FORCE_VEC  (CALYPSO_FORCE_VEC=<n>, VALUE, inert by default)
             *   masks   : the HARDWARE SOURCE of a DSP interrupt this model does not
             *             implement. Measured: the FB task arms
             *             data[0x0158..0159] = "call 0x728a" (13 times), that trampoline
             *             is only reachable through interrupt slot 30
             *             (data 0x00F8-0x00FB = "fb 0x0158", copied from PDROM 0xe399 by
             *             the reada at 0xb4c9), IMR bit 14 (= vec - 16) is permanently
             *             unmasked in every IMR measured, and the IFR NEVER takes
             *             anything but 0x0020/0x0008: nothing ever raises that bit. Four
             *             leads were eliminated by measurement (task/loop race, masked
             *             IMR, end-of-DMA interrupt, program bank never switched - XPC
             *             does see pages 0 AND 1).
             *   remove  : as soon as it is known WHICH hardware line carries this vector
             *             on the Calypso. This is a DOCUMENTATION gap (the DSP interrupt
             *             table is nowhere in the repository), not a bug to grep for.
             *   note    : this gate is not a fix, it is a decisive test - it says whether
             *             everything downstream of the vector is healthy. If it is, the
             *             problem reduces to identifying and wiring the source.
             *   WARNING : together with SEED5AC8 and DISPATCH_INSTALL that makes THREE
             *             simultaneous crutches: what is measured is no longer the native
             *             behaviour but what it would do with three holes plugged.
             *             Acceptable for a test, not as a configuration.
             *
             * Trigger: injection happens only while the FB handler is actually armed
             * (data[0x0159] == 0x728a), never in the window where the main loop has put
             * back its light handler 0x7242 - otherwise the test would measure the other
             * path. Cadence CALYPSO_FORCE_VEC_PERIOD insns (default 65536 = one per
             * frame), hard limit of 200 injections, 20 log lines. */
            static int _fv = -2, _fvbit = -1; static unsigned _fvn = 0, _fvper = 65536;
            static uint32_t _fvlast = 0;
            if (_fv == -2) {
                const char *e = getenv("CALYPSO_FORCE_VEC");
                _fv = (e && *e) ? (int)strtol(e, NULL, 0) : -1;
                const char *p = getenv("CALYPSO_FORCE_VEC_PERIOD");
                if (p && *p) _fvper = (unsigned)strtoul(p, NULL, 0);
                if (_fv >= 0) {
                    _fvbit = _fv - 16;
                    fprintf(stderr, "[c54x] FORCE-VEC arme : vecteur %d (IMR bit %d), "
                            "1 injection / %u insn quand data[0x0159]==0x728a, "
                            "plafond 200 (BEQUILLE, voir calypso_c54x.c)\n",
                            _fv, _fvbit, _fvper);
                }
            }
            if (_fv >= 16 && _fvbit >= 0 && _fvn < 200 &&
                (s->insn_count - _fvlast) >= _fvper &&
                s->data[0x0159] == 0x728a) {
                _fvlast = s->insn_count;
                if (++_fvn <= 20)
                    fprintf(stderr, "[c54x] FORCE-VEC #%u injection vec=%d bit=%d "
                            "IMR=0x%04x IFR=0x%04x INTM=%d PC=0x%04x insn=%u\n",
                            _fvn, _fv, _fvbit, s->imr, s->ifr,
                            !!(s->st1 & ST1_INTM), exec_pc, s->insn_count);
                c54x_interrupt_ex(s, _fv, _fvbit);
            }
        }
        {   /* TRACEFROM (CALYPSO_TRACEFROM=<pc>): dumps the opcodes at that PC,
             * then follows the control flow from it, printing each
             * discontinuity, until one of the landmarks 0xa076 (MAC kernel),
             * 0x79e4 (d_fb_det publisher) or 0x9ac0 (detector) is reached, or
             * 4000 instructions have passed. Three windows per run. */
            static int _tf = -1; static uint16_t _tfpc = 0; static int _tfd = 0;
            static int _tfn2 = 24;   /* CALYPSO_TRACEFROM_N: dump length */
            static int _tfarm = 0; static unsigned _tfn = 0, _tfr = 0; static uint16_t _tfp = 0;
            if (_tf < 0) { const char *e = getenv("CALYPSO_TRACEFROM");
                _tf = (e && *e) ? 1 : 0;
                if (_tf) _tfpc = (uint16_t)strtol(e, NULL, 0);
                const char *n = getenv("CALYPSO_TRACEFROM_N");
                if (n && *n) _tfn2 = atoi(n); }
            if (_tf) {
                if (exec_pc == _tfpc) {
                    if (!_tfd) { _tfd = 1;
                        fprintf(stderr, "[c54x] TRACEFROM-OPDUMP 0x%04x..+23:", _tfpc);
                        for (int _k = 0; _k < _tfn2; _k++) {
                            if ((_k % 8) == 0) fprintf(stderr, "\n  0x%04x:", _tfpc + _k);
                            fprintf(stderr, " %04x", prog_fetch(s, (uint16_t)(_tfpc + _k)));
                        }
                        fprintf(stderr, "\n"); }
                    if (_tfr < 3) { _tfr++; _tfarm = 1; _tfn = 0;
                        fprintf(stderr, "[c54x] TRACEFROM === entree 0x%04x #%u (task_md=%u) ===\n",
                                _tfpc, _tfr, (unsigned)(s->data[0x0804] ? s->data[0x0804] : s->data[0x0818])); }
                }
                if (_tfarm) {
                    if (++_tfn > 4000) { _tfarm = 0;
                        fprintf(stderr, "[c54x] TRACEFROM fin (4000 insn)\n"); }
                    else {
                        int _d = (int)s->pc - (int)_tfp;
                        if ((_d > 3 || _d < 0) && _tfn < 3000)
                            fprintf(stderr, "[c54x] TRACEFROM 0x%04x -> 0x%04x op=0x%04x A=0x%06llx\n",
                                    _tfp, s->pc, prog_fetch(s, s->pc),
                                    (unsigned long long)(s->a & 0xFFFFFFULL));
                        if (s->pc == 0xa076 || s->pc == 0x79e4 || s->pc == 0x9ac0) {
                            fprintf(stderr, "[c54x] TRACEFROM *** ATTEINT 0x%04x %s ***\n", s->pc,
                                    s->pc == 0xa076 ? "(kernel MAC)" :
                                    s->pc == 0x79e4 ? "(publisher d_fb_det)" : "(detecteur)");
                            _tfarm = 0; }
                    }
                }
                _tfp = s->pc;
            }
        }
        {   /* CORROUT (CALYPSO_CORROUT): on leaving the MAC kernel
             * (0xa070-0xa0a0), dumps the accumulators, T, the pointer ARs and
             * the workspace at 0x2c00 - what the correlator produced. */
            static int _co = -1; static int _in_k = 0; static unsigned _con = 0;
            if (_co < 0) _co = calypso_gate("CALYPSO_CORROUT", 0);
            if (_co) {
                if (exec_pc >= 0xa070 && exec_pc <= 0xa0a0) { _in_k = 1; }
                else if (_in_k) {   /* just LEFT the MAC kernel */
                    _in_k = 0;
                    if (_con < 40) {
                        _con++;
                        fprintf(stderr, "[c54x] CORROUT sortie noyau -> PC=0x%04x "
                                "A=0x%010llx B=0x%010llx T=%04x "
                                "AR3=%04x AR4=%04x AR5=%04x AR6=%04x insn=%u\n",
                                exec_pc,
                                (unsigned long long)(s->a & 0xFFFFFFFFFFULL),
                                (unsigned long long)(s->b & 0xFFFFFFFFFFULL),
                                s->t, s->ar[3], s->ar[4], s->ar[5], s->ar[6],
                                s->insn_count);
                        fprintf(stderr, "[c54x] CORROUT   wz[2c00..0f]=");
                        for (int _k = 0; _k < 16; _k++)
                            fprintf(stderr, " %04x", s->data[0x2c00 + _k]);
                        fprintf(stderr, "\n");
                    }
                }
            }
        }
        {   /* VECTAB (CALYPSO_VECTAB): one-shot dump of the interrupt vector
             * table 0x0080..0x00FF, classifying each slot and flagging those
             * whose target falls in the FB routine range. */
            static int _vt = -1; static int _vtdone = 0; static int _vtseen = 0;
            if (_vt < 0) _vt = calypso_gate("CALYPSO_VECTAB", 0);
            if (_vt && !_vtdone && exec_pc == 0xb01c && ++_vtseen >= 2) {
                _vtdone = 1;
                fprintf(stderr, "[c54x] VECTAB table de vecteurs 0x0080..0x00FF "
                        "(f880=B long, f4eb=RETE stub, f495=NOP)\n");
                for (int _v = 0; _v < 32; _v++) {
                    uint16_t _a = (uint16_t)(0x0080 + _v * 4);
                    uint16_t _w0 = s->data[_a], _w1 = s->data[_a + 1];
                    const char *_kind = (_w0 == 0xf4eb) ? "RETE (STUB)" :
                                        ((_w0 & 0xFF80) == 0xF880) ? "B long" :
                                        (_w0 == 0xf495) ? "NOP" : "?";
                    int _fb = ((_w0 & 0xFF80) == 0xF880) &&
                              (_w1 >= 0x7600 && _w1 <= 0x7a00);
                    fprintf(stderr, "[c54x] VECTAB   vec%-2d @0x%04x  w0=0x%04x w1=0x%04x  %-12s"
                            "%s%s\n", _v, _a, _w0, _w1, _kind,
                            _v == 19 ? "  [FRAME]" : _v == 21 ? "  [BRINT0]" : "",
                            _fb ? "   <<<< CIBLE DANS LA ZONE FB" : "");
                }
                fprintf(stderr, "[c54x] VECTAB fin ; code juste AVANT 0x76f8 "
                        "(0x76e8..0x76f7):");
                for (int _k = 0; _k < 16; _k++)
                    fprintf(stderr, " %04x", prog_fetch(s, (uint16_t)(0x76e8 + _k)));
                fprintf(stderr, "\n");
            }
        }
        {   /* SCANDATA (CALYPSO_SCANDATA, with _LO/_HI): one-shot scan of
             * data[] for cells pointing into the FB routine range. A total of 0
             * means the FB routine is unreachable by computed dispatch in this
             * image. */
            static int _sd2 = -1; static int _sddone = 0; static unsigned _sdhit = 0;
            static uint16_t _lo = 0x76f8, _hi = 0x79f0;
            if (_sd2 < 0) {
                _sd2 = calypso_gate("CALYPSO_SCANDATA", 0);
                const char *a = getenv("CALYPSO_SCANDATA_LO");
                const char *b = getenv("CALYPSO_SCANDATA_HI");
                if (a && *a) _lo = (uint16_t)strtol(a, NULL, 0);
                if (b && *b) _hi = (uint16_t)strtol(b, NULL, 0);
            }
            if (_sd2 && !_sddone && exec_pc == 0xb01c) {
                static int _seen = 0;
                if (++_seen >= 2) {   /* after the table init */
                    _sddone = 1;
                    fprintf(stderr, "[c54x] SCANDATA cellules data[] pointant dans "
                            "[0x%04x..0x%04x] (routine FB) :\n", _lo, _hi);
                    for (uint32_t _a = 0; _a < C54X_DATA_SIZE; _a++) {
                        uint16_t _v = s->data[_a];
                        if (_v < _lo || _v > _hi) continue;
                        fprintf(stderr, "[c54x] SCANDATA   data[0x%04x] = 0x%04x%s\n",
                                (unsigned)_a, _v,
                                (_a >= 0x4380 && _a <= 0x43ff) ? "   <<<< DANS LA TABLE DE DISPATCH" :
                                (_a >= 0x0800 && _a <  0x2800) ? "   (API RAM)" : "");
                        if (++_sdhit > 60) break;
                    }
                    fprintf(stderr, "[c54x] SCANDATA total=%u  (0 = routine FB inatteignable "
                            "par dispatch calcule dans cette image)\n", _sdhit);
                }
            }
        }
        {   /* DISPWATCH (CALYPSO_DISPWATCH): at the dispatch sites, shows A,
             * the three candidate table slots and the current d_task_md, with
             * FB tasks (5, 6, 8, 9) given their own, larger budget. */
            static int _dw = -1; static unsigned _dwn = 0, _dwfb = 0;
            if (_dw < 0) _dw = calypso_gate("CALYPSO_DISPWATCH", 0);
            if (_dw && (exec_pc == 0xb40f || exec_pc == 0xb01c || exec_pc == 0xb01e ||
                        exec_pc == 0xb0f0 || exec_pc == 0xb0f6)) {
                unsigned _md = s->data[0x0804] ? s->data[0x0804] : s->data[0x0818];
                int _isfb = (_md == 5 || _md == 6 || _md == 8 || _md == 9);
                int _emit = 0;
                if (_isfb) { if (_dwfb < 60) { _dwfb++; _emit = 1; } }
                else       { if (_dwn  < 20) { _dwn++;  _emit = 1; } }
                if (_emit) {
                    const char *_site = (exec_pc == 0xb40f) ? "BACC-terminal" :
                                        (exec_pc == 0xb01c) ? "LD-slot" :
                                        (exec_pc == 0xb01e) ? "CALA" :
                                        (exec_pc == 0xb0f0) ? "idx-calc" : "idx-LD";
                    fprintf(stderr, "[c54x] DISPWATCH %s pc=0x%04x A=0x%06llx "
                            "slot43c0=0x%04x slot4387=0x%04x slot43d8=0x%04x "
                            "task_md=%u%s insn=%u\n", _site, exec_pc,
                            (unsigned long long)(s->a & 0xFFFFFFULL),
                            s->data[0x43c0], s->data[0x4387], s->data[0x43d8],
                            _md, _isfb ? "  <<<< TACHE FB" : "", s->insn_count);
                }
            }
        }
        {   /* SCANREF (CALYPSO_SCANREF=<target>): one-shot scan of all four
             * program banks for words equal to the target, printing the two
             * preceding words so the referencing instruction can be
             * identified. */
            static int _sr = -1; static uint16_t _srt = 0; static int _srd = 0;
            if (_sr < 0) { const char *e = getenv("CALYPSO_SCANREF");
                _sr = (e && *e) ? 1 : 0;
                if (_sr) _srt = (uint16_t)strtol(e, NULL, 0); }
            if (_sr && !_srd && exec_pc == 0xb01c) {
                _srd = 1;
                uint16_t _sx = s->xpc; unsigned _tot = 0;
                fprintf(stderr, "[c54x] SCANREF cible=0x%04x (f074=CALL f272=RPTBD f820/f880=B "
                        "76f8=ST#imm 10f8=LD)\n", _srt);
                for (unsigned _bk = 0; _bk < 4; _bk++) {
                    s->xpc = (uint16_t)_bk; unsigned _h = 0;
                    for (uint32_t _p = 0x7000; _p <= 0xfffe; _p++) {
                        if (prog_fetch(s, (uint16_t)_p) != _srt) continue;
                        if (_p >= (uint32_t)_srt - 2 && _p <= (uint32_t)_srt + 2) continue;
                        fprintf(stderr, "[c54x] SCANREF bank%u @0x%04x prev2=0x%04x prev1=0x%04x\n",
                                _bk, _p, prog_fetch(s, (uint16_t)(_p-2)), prog_fetch(s, (uint16_t)(_p-1)));
                        _tot++;
                        if (++_h > 15) break;
                    }
                }
                s->xpc = _sx;
                fprintf(stderr, "[c54x] SCANREF total=%u sur 4 banks ; code @0x%04x..+15:", _tot, _srt);
                for (int _k = 0; _k < 16; _k++)
                    fprintf(stderr, " %04x", prog_fetch(s, (uint16_t)(_srt + _k)));
                fprintf(stderr, "\n");
            }
        }
        {   /* SCANFB (CALYPSO_SCANFB): same scan as SCANREF, over the four
             * fixed FB addresses (body, entry stub, subroutine, correlation). */
            static int _sf = -1; static int _sfd = 0;
            if (_sf < 0) _sf = calypso_gate("CALYPSO_SCANFB", 0);
            if (_sf && !_sfd && exec_pc == 0xb01c) {
                _sfd = 1;
                const uint16_t _tg[4] = { 0x7708, 0x76fb, 0x770d, 0x795f };
                const char *_nm[4] = { "corps FB", "stub entree", "sous-prog", "correlation" };
                uint16_t _sx = s->xpc;
                for (unsigned _bk = 0; _bk < 4; _bk++) {
                    s->xpc = (uint16_t)_bk;
                    for (int _t = 0; _t < 4; _t++) {
                        unsigned _h = 0;
                        for (uint32_t _p = 0x7000; _p <= 0xfffe; _p++) {
                            if (prog_fetch(s, (uint16_t)_p) == _tg[_t]) {
                                if (_p >= _tg[_t] - 2 && _p <= _tg[_t] + 2) continue;
                                fprintf(stderr, "[c54x] SCANFB bank%u ref 0x%04x (%s) @0x%04x "
                                        "prev2=0x%04x prev1=0x%04x\n", _bk, _tg[_t], _nm[_t], _p,
                                        prog_fetch(s, (uint16_t)(_p-2)), prog_fetch(s, (uint16_t)(_p-1)));
                                if (++_h > 12) break;
                            }
                        }
                    }
                }
                s->xpc = _sx;
                fprintf(stderr, "[c54x] SCANFB fin (f074=CALL, f272=BD, fc00=RET, 76f8=ST #imm)\n");
            }
        }
        {   /* FBENTRY (CALYPSO_FBENTRY): traces the FB routine range
             * 0x75e0-0x79f0 instruction by instruction, then the exit (with the
             * stack top, flagged when it lands in the vector page) and the ten
             * instructions after it. */
            static int _fe = -1; static int _dmp = 0; static int _in = 0; static int _after = 0;
            static unsigned _n = 0;
            if (_fe < 0) _fe = calypso_gate("CALYPSO_FBENTRY", 0);
            if (_fe) {
                if (exec_pc >= 0x75e0 && exec_pc <= 0x79f0) {   /* includes the subroutine at 0x75e8 */
                    if (!_dmp) { _dmp = 1;
                        fprintf(stderr, "[c54x] FBENTRY-OPDUMP 0x76f8..0x7730:");
                        for (int _k = 0; _k < 57; _k++)
                            fprintf(stderr, " %04x", prog_fetch(s, (uint16_t)(0x76f8 + _k)));
                        fprintf(stderr, "\n"); }
                    _in = 1;
                    if (_n < 250) { _n++;
                        fprintf(stderr, "[c54x] FBENTRY pc=0x%04x op=0x%04x op2=0x%04x A=0x%06llx "
                                "TC=%u AR1=%04x AR2=%04x ST0=0x%04x ST1=0x%04x\n",
                                exec_pc, prog_fetch(s, exec_pc), prog_fetch(s, (uint16_t)(exec_pc+1)),
                                (unsigned long long)(s->a & 0xFFFFFFULL), (unsigned)((s->st0 >> 12) & 1),
                                s->ar[1], s->ar[2], s->st0, s->st1); }
                } else if (_in) { _in = 0; _after = 10;
                    if (_n < 250) { _n++;
                        fprintf(stderr, "[c54x] FBENTRY *** SORTIE vers 0x%04x *** (op=0x%04x) "
                                "SP=0x%04x pile[SP]=0x%04x pile[SP+1]=0x%04x%s\n",
                                exec_pc, prog_fetch(s, exec_pc), s->sp,
                                s->data[s->sp], s->data[(uint16_t)(s->sp + 1)],
                                (exec_pc >= 0x0080 && exec_pc <= 0x00ff)
                                    ? "  [VECTEUR IT]" : ""); }
                } else if (_after > 0) { _after--;
                    if (_n < 250) { _n++;
                        fprintf(stderr, "[c54x] FBENTRY   apres-sortie pc=0x%04x op=0x%04x SP=0x%04x%s\n",
                                exec_pc, prog_fetch(s, exec_pc), s->sp,
                                (exec_pc == 0x7707 || exec_pc == 0x7708)
                                    ? "  <<<< RETOUR DANS LA ROUTINE FB" : ""); }
                }
            }
        }
        {   /* SCAN43D8 (CALYPSO_SCAN43D8): one-shot scan of the four program
             * banks for the word 0x43d8, classifying each reference as an
             * install (ST #imm) or a read (LD). */
            static int _s4 = -1; static int _s4done = 0;
            if (_s4 < 0) _s4 = calypso_gate("CALYPSO_SCAN43D8", 0);
            if (_s4 && !_s4done && exec_pc == 0xb01c) {
                _s4done = 1;
                unsigned _h = 0;
                uint16_t _savedxpc = s->xpc;
                for (unsigned _bk = 0; _bk < 4; _bk++) {
                s->xpc = (uint16_t)_bk;
                fprintf(stderr, "[c54x] SCAN43D8 --- bank %u ---\n", _bk);
                for (uint32_t _p = 0x7000; _p <= 0xfffe; _p++) {
                    if (prog_fetch(s, (uint16_t)_p) == 0x43d8) {
                        uint16_t _o2 = prog_fetch(s, (uint16_t)(_p-2));
                        uint16_t _o1 = prog_fetch(s, (uint16_t)(_p-1));
                        const char *_k = (_o2 == 0x76f8) ? "ST #imm -> INSTALLE"
                                       : (_o2 == 0x10f8) ? "LD -> LIT"
                                       : (_o1 == 0x76f8) ? "ST #imm (dec) -> INSTALLE" : "?";
                        fprintf(stderr, "[c54x] SCAN43D8 @0x%04x prev2=0x%04x prev1=0x%04x  %s\n",
                                _p, _o2, _o1, _k);
                        if (++_h > 60) break;
                    }
                }
                }
                s->xpc = _savedxpc;
                fprintf(stderr, "[c54x] SCAN43D8 total=%u sur 4 banks\n", _h);
                fprintf(stderr, "[c54x] SCAN43D8 code @0xbaf8..0xbb10:");
                for (int _k2 = 0; _k2 < 25; _k2++)
                    fprintf(stderr, " %04x", prog_fetch(s, (uint16_t)(0xbaf8 + _k2)));
                fprintf(stderr, "\n");
            }
        }
        {   /* SLOTSRC (CALYPSO_SLOTSRC): instruction trace of 0xaff0-0xb01d,
             * the code that computes the dispatch slot. */
            static int _ss = -1; static unsigned _ssn = 0;
            if (_ss < 0) _ss = calypso_gate("CALYPSO_SLOTSRC", 0);
            if (_ss && _ssn < 120 && exec_pc >= 0xaff0 && exec_pc <= 0xb01d) {
                _ssn++;
                fprintf(stderr, "[c54x] SLOTSRC pc=0x%04x op=0x%04x op2=0x%04x "
                        "A=0x%06llx AR1=%04x AR2=%04x AR3=%04x ST0=0x%04x insn=%u\n",
                        exec_pc, prog_fetch(s, exec_pc), prog_fetch(s, (uint16_t)(exec_pc+1)),
                        (unsigned long long)(s->a & 0xFFFFFFULL),
                        s->ar[1], s->ar[2], s->ar[3], (unsigned)s->st0, s->insn_count);
            }
        }
        {   /* FBCALL (CALYPSO_FBCALL): on each rising edge of d_task_md to 5
             * (FB), follows the control flow for up to 20000 instructions and
             * reports whether it reaches the FB routine. Three rounds. */
            static int _fc = -1;
            if (_fc < 0) _fc = calypso_gate("CALYPSO_FBCALL", 0);
            if (_fc) {
                static uint16_t _pmd = 0xffff, _ppc = 0; static int _arm = 0;
                static unsigned _steps = 0, _logged = 0, _rounds = 0;
                uint16_t _md = s->data[0x0804] ? s->data[0x0804] : s->data[0x0818];
                if (_md != _pmd) {
                    if (_md == 5 && _rounds < 3) {
                        _rounds++; _arm = 1; _steps = 0;
                        fprintf(stderr, "[c54x] FBCALL === tache FB #%u (d_task_md=5) ===\n", _rounds);
                    }
                    _pmd = _md;
                }
                if (_arm) {
                    if (++_steps > 20000) { _arm = 0;
                        fprintf(stderr, "[c54x] FBCALL fin (20000 insn) sans 0x7700\n"); }
                    else {
                        int _d = (int)s->pc - (int)_ppc;
                        if ((_d > 3 || _d < 0) && _logged < 300) {
                            _logged++;
                            fprintf(stderr, "[c54x] FBCALL 0x%04x -> 0x%04x op=0x%04x A=0x%06llx\n",
                                    _ppc, s->pc, prog_fetch(s, s->pc),
                                    (unsigned long long)(s->a & 0xFFFFFFULL));
                        }
                        if (s->pc >= 0x76f0 && s->pc <= 0x79f0) {
                            fprintf(stderr, "[c54x] FBCALL *** ROUTINE FB ATTEINTE 0x%04x ***\n", s->pc);
                            _arm = 0; }
                    }
                }
                _ppc = s->pc;
            }
        }
        {   /* DISPIDX (CALYPSO_DISPIDX): captures the dispatch index computed
             * at 0xb0f0 and, at 0xb0f6, resolves the table slot it designates,
             * flagging the FB task. */
            static int _di = -1; static unsigned _din = 0; static unsigned _idx = 0;
            if (_di < 0) _di = calypso_gate("CALYPSO_DISPIDX", 0);
            if (_di) {
                if (exec_pc == 0xb0f0) _idx = (unsigned)(s->a & 0xFFFF);
                if (exec_pc == 0xb0f6 && _din < 80) {
                    _din++;
                    unsigned _slotaddr = 0x4387 + _idx;
                    unsigned _slot = (_slotaddr < 0x10000) ? s->data[_slotaddr] : 0;
                    unsigned _md = s->data[0x0804] ? s->data[0x0804] : s->data[0x0818];
                    fprintf(stderr, "[c54x] DISPIDX idx=%u slot=data[0x%04x]=0x%04x "
                            "d_task_md=%u%s insn=%u\n", _idx, _slotaddr, _slot, _md,
                            (_md == 5) ? "  <<<< TACHE FB" : "", s->insn_count);
                }
            }
        }
        {   /* DISPTAB (CALYPSO_DISPTAB): dumps the dispatch table
             * data[0x4380..0x43cf] at the dispatcher; 0xab38 is the default
             * handler, so the FB slot is whatever differs. */
            static int _dd2 = -1; static unsigned _ddn2 = 0;
            if (_dd2 < 0) _dd2 = calypso_gate("CALYPSO_DISPTAB", 0);
            if (_dd2 && exec_pc == 0xb0f1 && _ddn2 < 4) {
                _ddn2++;
                fprintf(stderr, "[c54x] DISPTAB-DUMP #%u data[0x4380..0x43cf] :", _ddn2);
                for (int _k = 0; _k < 80; _k++) {
                    if ((_k % 16) == 0) fprintf(stderr, "\n  0x%04x:", 0x4380 + _k);
                    fprintf(stderr, " %04x", s->data[0x4380 + _k]);
                }
                fprintf(stderr, "\n  (0xab38 = handler par defaut ; on cherche le slot FB)\n");
            }
        }
        {   /* TASKGO (CALYPSO_TASKGO): on each rising edge of d_task_md to 5
             * (FB), traces the next 250 instructions and reports whether the FB
             * routine is reached, plus the highest PC seen. */
            static int _tg = -1;
            if (_tg < 0) _tg = calypso_gate("CALYPSO_TASKGO", 0);
            if (_tg) {
                static uint16_t _prev = 0xffff; static int _armed = 0;
                static unsigned _n = 0, _rounds = 0, _hi = 0;
                uint16_t _md = s->data[0x0804] ? s->data[0x0804] : s->data[0x0818];
                if (_md != _prev) {
                    if (_md == 5 && _rounds < 4) {
                        _rounds++; _armed = 1; _n = 0; _hi = 0;
                        fprintf(stderr, "[c54x] TASKGO front d_task_md -> 5 (FB) "
                                "md0804=%u md0818=%u PC=0x%04x insn=%u\n",
                                (unsigned)s->data[0x0804], (unsigned)s->data[0x0818],
                                s->pc, s->insn_count);
                    }
                    _prev = _md;
                }
                if (_armed && _n < 250) { _n++;
                    if (s->pc > _hi) _hi = s->pc;
                    fprintf(stderr, "[c54x] TASKGO-FLOW pc=0x%04x xpc=%u op=0x%04x A=0x%06llx\n",
                            s->pc, s->xpc, prog_fetch(s, s->pc),
                            (unsigned long long)(s->a & 0xFFFFFFULL));
                    if (s->pc >= 0x7700 && s->pc <= 0x79f0) {
                        fprintf(stderr, "[c54x] TASKGO *** ATTEINT LA ROUTINE FB 0x%04x ***\n", s->pc);
                        _armed = 0; }
                } else if (_armed) { _armed = 0;
                    fprintf(stderr, "[c54x] TASKGO fin (250 pas) sans 0x7700 ; PC max vu=0x%04x\n", _hi); }
            }
        }
        {   /* @BEQUILLE - DISPATCH_INSTALL  (CALYPSO_DISPATCH_INSTALL=0xNNNN,
             *              VALUE, unset by default = inert)
             *   masks   : the routine that ought to install a task handler in
             *             data[0x43d8]. Measured: that cell only ever receives the
             *             0xab38 plug (a RET), written once by 0xbb00, and nothing
             *             writes it again - confirmed statically (2 references across
             *             the 5 ROMs) and at runtime. 0xb01c re-reads it on EVERY
             *             frame and 0xb01e calls it, so the DSP acknowledges each task
             *             and does nothing.
             *   remove  : as soon as it is known WHICH routine should populate
             *             0x43d8. This gate is not a fix, it is a decisive test: it
             *             says whether the rest of the chain lights up once the
             *             handler is right. If it does, the question reduces to the
             *             installer.
             *
             * Measured candidates: 0xab77 = table[5], which reads d_task_md
             * (*AR1(0x0004)) and tests its bit 15 - the most plausible for the FB
             * task; 0xb284 = fcall 0x770a, the entry of the publisher's bank, never
             * reached otherwise.
             *
             * Only the plug is overwritten: if a real handler is ever installed, this
             * gate will not mask it, it will fall silent. */
            static int _di2 = -2, _ditask = -1; static unsigned _din2 = 0;
            if (_di2 == -2) {
                const char *e = getenv("CALYPSO_DISPATCH_INSTALL");
                _di2 = (e && *e) ? (int)strtoul(e, NULL, 0) : -1;
                const char *t = getenv("CALYPSO_DISPATCH_INSTALL_TASK");
                _ditask = (t && *t) ? atoi(t) : -1;   /* -1 = any task */
                if (_di2 >= 0)
                    fprintf(stderr, "[c54x] DISPATCH-INSTALL arme : data[0x43d8] "
                            "<- 0x%04x pour d_task_md=%s (BEQUILLE, voir "
                            "calypso_c54x.c)\n", (unsigned)_di2,
                            _ditask < 0 ? "toute tache non nulle" : "la tache demandee");
            }
            /* @BEQUILLE - FORCE_TASK  (CALYPSO_FORCE_TASK=<n>, default OFF)
             *   masks   : the L1's inability to SUSTAIN its CCCH command. It orders a
             *             task to the DSP in the ARM's place. Measured: task 24 is
             *             indeed commanded (7 to 34 times depending on the run) and
             *             burst reports are accepted since the burst-id fix, but the
             *             mobile does not camp, so it reselects about every 10 s, and
             *             each reselection does L1-RESET: d_dsp_page = 0 (17 times in
             *             a 65 s run), which wipes the engagement. The DSP never gets
             *             a long window.
             *             What it does NOT fabricate is the result: it sets the
             *             COMMAND, not a_cd, so WATCH-ACD remains an honest judge - if
             *             the DSP writes a_cd under this crutch, the DSP wrote it.
             *   remove  : when the mobile camps and commands the CCCH by itself.
             *
             * It does not fight the ARM: it only fills passes where both write pages
             * are empty (no command in flight), so a real ARM command always wins.
             * 20 log lines.
             */
            if (exec_pc == 0xb01c) {
                static int _ft = -2; static unsigned _ftn = 0;
                if (_ft == -2) {
                    const char *e = getenv("CALYPSO_FORCE_TASK");
                    _ft = (e && *e) ? (int)strtol(e, NULL, 0) : -1;
                    if (_ft >= 0)
                        fprintf(stderr, "[c54x] FORCE-TASK arme : d_task_md <- %d "
                                "quand l'ARM ne commande rien (BEQUILLE — la commande "
                                "est fabriquee, le RESULTAT ne l'est pas)\n", _ft);
                }
                /* A reception command is never a single word. osmocom-bb
                 * calypso/dsp.c:479 has three fields:
                 *     void dsp_load_rx_task(task, burst_id, tsc) {
                 *         db_w->d_task_d      = task;          // word 0
                 *         db_w->d_burst_d     = burst_id;      // word 1
                 *         db_w->d_ctrl_system |= tsc & 0x7;    // word 16
                 *     }
                 * d_task_md (word 4) is only used by FB/SB (prim_fbsb.c:279,379),
                 * where it comes with an NDB parameter (d_fb_mode).
                 *
                 * Layout (T_DB_MCU_TO_DSP, include/calypso/dsp_api.h:82):
                 *     word 0 d_task_d | 1 d_burst_d | 4 d_task_md | 16 d_ctrl_system
                 * Bases: page 0 = 0x0800, page 1 = 0x0814, hence
                 *     d_task_d      0x0800 / 0x0814
                 *     d_burst_d     0x0801 / 0x0815
                 *     d_ctrl_system 0x0810 / 0x0824
                 *
                 * The PAGE is the one the ARM designates: bit 0 of
                 * d_dsp_page = B_GSM_TASK | w_page (dsp.c:471).
                 *
                 * burst_id CYCLES 0,1,2,3: the firmware calls RX NB four times and
                 * only collects the data on the fourth answer (wiki
                 * HardwareCalypsoDSP). TSC: CALYPSO_FORCE_TASK_TSC, default 7 (the
                 * BCC measured on this network).
                 */
                if (_ft >= 0 && s->data[0x0800] == 0 && s->data[0x0814] == 0 &&
                    s->data[0x0804] == 0 && s->data[0x0818] == 0) {
                    static unsigned _fbid = 0;
                    static int _ftsc = -1;
                    if (_ftsc < 0) {
                        const char *t = getenv("CALYPSO_FORCE_TASK_TSC");
                        _ftsc = (t && *t) ? (int)strtol(t, NULL, 0) : 7;
                    }
                    /* Keep our OWN page counter: d_dsp_page is reset to 0 by
                     * every L1-RESET (30 times per minute, measured), so
                     * reading it back would always yield page 0. The firmware
                     * keeps w_page in ARM RAM and flips it at the end of each
                     * scenario; do the same. */
                    static unsigned _fpg = 0;
                    unsigned pg   = _fpg & 1u;
                    uint16_t base = pg ? 0x0814 : 0x0800;
                    uint16_t ctrl = pg ? 0x0824 : 0x0810;
                    uint16_t bid  = (uint16_t)(_fbid++ & 3u);

                    /* These three writes must hit BOTH data[] and api_ram[]:
                     * for any address inside the API window the DSP reads
                     * api_ram[], not data[]:
                     *     data_read_locked():
                     *       if (addr >= C54X_API_BASE && addr < ...+C54X_API_SIZE)
                     *           v = s->api_ram[addr - C54X_API_BASE];
                     * Writing data[] alone lands the forced task in an array
                     * nobody reads. Measured (LD-TRACE probe):
                     *     pc=0xb05f addr=0x0814 data[addr]=0x0018 val_lue=0x0000
                     * The resolved address is right and data[] does hold 24
                     * (ALLC), yet the read returns 0. The whole measured chain
                     * follows: the dispatcher compares 0 against the constants
                     * 12/30/34, bails out at the first branch to 0xb077, the
                     * index resolution is never reached, index 41 (RX arming,
                     * 0xa5cd) is never requested, and A_CD-WR stays 0.
                     *
                     * This makes the FORCE_TASK crutch above EFFECTIVE; it does
                     * not make it legitimate. */
                    s->data[base + 0] = (uint16_t)_ft;         /* d_task_d      */
                    s->data[base + 1] = bid;                   /* d_burst_d     */
                    s->data[ctrl]    |= (uint16_t)(_ftsc & 7); /* d_ctrl_system */
                    if (s->api_ram) {
                        s->api_ram[base + 0 - C54X_API_BASE] = (uint16_t)_ft;
                        s->api_ram[base + 1 - C54X_API_BASE] = bid;
                        s->api_ram[ctrl - C54X_API_BASE]    |= (uint16_t)(_ftsc & 7);
                    }

                    /* End of scenario. Without it, a page is filled that
                     * nobody opens. Exact port of dsp_end_scenario()
                     * (osmocom-bb calypso/dsp.c:466):
                     *     ndb->d_dsp_page = B_GSM_TASK | w_page;   // announce
                     *     w_page ^= 1;                             // flip
                     *     tpu_dsp_frameirq_enable(); tpu_frame_irq_en(1,1);
                     * B_GSM_TASK = (1 << 1) = 0x0002 (l1_environment.h:249) and
                     * the page is bit 0, so d_dsp_page is 0x0002 or 0x0003.
                     * The third part, the frame interrupt, is already faithful
                     * here: calypso_tpu.c tests TPU_CTRL_DSP_EN then
                     * ICTRL_DSP_FRAME active-low, exactly what the firmware
                     * sets. */
                    s->data[0x08D4] = (uint16_t)(0x0002u | pg);
                    if (s->api_ram) {
                        s->api_ram[0x08D4 - 0x0800] = (uint16_t)(0x0002u | pg);
                    }
                    _fpg ^= 1u;                                /* w_page ^= 1 */

                    if (_ftn++ < 20)
                        fprintf(stderr, "[c54x] FORCE-TASK #%u page=%u d_task_d<-%d "
                                "d_burst_d<-%u tsc=%d d_dsp_page<-0x%04x (fin de "
                                "scenario) insn=%u\n", _ftn, pg, _ft, bid, _ftsc,
                                (unsigned)(0x0002u | pg), s->insn_count);
                }
            }

            /* Mutual exclusion: in "init" mode the graft in
             * data_write_locked() does the work and this block must stay
             * quiet, otherwise the two fight each other. */
            static int _at_init = -1;
            if (_at_init < 0) { const char *m = getenv("CALYPSO_DISPATCH_INSTALL_AT");
                                _at_init = (m && strcmp(m, "init") == 0) ? 1 : 0; }
            if (_di2 >= 0 && !_at_init && exec_pc == 0xb01c) {
                uint16_t _md = s->data[0x0804] ? s->data[0x0804] : s->data[0x0818];
                /* Reinstall on EVERY dispatch of the targeted task, and only
                 * while the slot still holds the plug or our own value: if a
                 * real routine ever installs something else, fall silent
                 * instead of overwriting it. Installing once only would freeze
                 * the handler after the first pass, and firing on any task
                 * would measure the wrong one - task 1 (PM) arrives before task
                 * 5 (FB) - hence _TASK. */
                int _cible = (_ditask < 0) ? (_md != 0) : (_md == (uint16_t)_ditask);
                uint16_t _cur = s->data[0x43d8];
                if (_cible && (_cur == 0xab38 || _cur == (uint16_t)_di2)) {
                    if (_cur != (uint16_t)_di2) {
                        s->data[0x43d8] = (uint16_t)_di2;
                        if (_din2++ < 20)
                            fprintf(stderr, "[c54x] DISPATCH-INSTALL #%u data[0x43d8] "
                                    "0x%04x -> 0x%04x (d_task_md=%u) insn=%u\n",
                                    _din2, _cur, (unsigned)_di2, (unsigned)_md,
                                    s->insn_count);
                    } else {
                        s->data[0x43d8] = (uint16_t)_di2;
                    }
                } else if (_cur != 0xab38 && _cur != (uint16_t)_di2) {
                    static int _dit = 0;
                    if (!_dit++)
                        fprintf(stderr, "[c54x] DISPATCH-INSTALL : un VRAI handler "
                                "0x%04x est apparu dans data[0x43d8] — la bequille "
                                "se retire, cherchez qui l'a pose\n", _cur);
                }
            }
        }
        {   /* DISPCALL: does the task dispatcher REALLY call the slot
             * data[0x43d8], and where does it land?
             *
             * The trigger is a condition, not an address to trace:
             * exec_pc == 0xb01c, the slot re-reader, which is known to execute
             * because the DISPATCH_INSTALL crutch already fires there. The PCs
             * ACTUALLY executed over the next CALYPSO_DISPCALL_N (default 10)
             * instructions are then recorded - no assumption about the ISA, nor
             * that 0xb01e is the CALA; this is the measured trajectory.
             *
             * What it decides:
             *   - the sequence passes through the slot value -> the handler IS
             *     called, and the question becomes what it does (follow up with
             *     CALYPSO_TRACEFROM=<slot value>);
             *   - it never enters -> that cell is not the one being called and
             *     the whole 0x43d8 lead falls;
             *   - complete silence -> 0xb01c is not reached, which is different
             *     again, and checkable because the DISPATCH_INSTALL arming line
             *     always prints.
             *
             * The trajectory repeats every frame, so identical runs are folded:
             * one line only when the sequence, the slot or d_task_md CHANGES,
             * carrying the number of repeats swallowed; 40 lines in total.
             * Gate CALYPSO_DISPCALL (default 0). */
            enum { DC_MAX = 24 };
            static int _dc = -1, _dcn2 = 10, _dc_armed = 0, _dc_k = 0, _dc_prevlen = -1;
            static uint16_t _dc_seq[DC_MAX], _dc_prev[DC_MAX];
            static uint16_t _dc_slot = 0, _dc_md = 0, _dc_pslot = 0xffff, _dc_pmd = 0xffff;
            static unsigned _dc_rep = 0, _dc_lines = 0;
            if (_dc < 0) {
                const char *_e = getenv("CALYPSO_DISPCALL_N");
                _dc = calypso_gate("CALYPSO_DISPCALL", 0);
                if (_e && *_e) _dcn2 = atoi(_e);
                if (_dcn2 < 2) _dcn2 = 2;
                if (_dcn2 > DC_MAX) _dcn2 = DC_MAX;
                if (_dc)
                    fprintf(stderr, "[c54x] DISPCALL arme : %d PC apres chaque passage "
                            "a 0xb01c, replie, 40 lignes max\n", _dcn2);
            }
            if (_dc) {
                if (exec_pc == 0xb01c) {          /* (re)arm */
                    _dc_armed = 1; _dc_k = 0;
                    _dc_slot = s->data[0x43d8];
                    _dc_md = s->data[0x0804] ? s->data[0x0804] : s->data[0x0818];
                }
                if (_dc_armed) {
                    _dc_seq[_dc_k++] = s->pc;     /* PC AFTER the executed instruction */
                    if (_dc_k >= _dcn2) {
                        int _i, _same;
                        _dc_armed = 0;
                        _same = (_dc_prevlen == _dc_k && _dc_slot == _dc_pslot
                                 && _dc_md == _dc_pmd);
                        if (_same)
                            for (_i = 0; _i < _dc_k; _i++)
                                if (_dc_seq[_i] != _dc_prev[_i]) { _same = 0; break; }
                        if (_same) {
                            _dc_rep++;
                        } else {
                            if (_dc_lines < 40) {
                                int _entre = 0, _fb = 0;
                                _dc_lines++;
                                for (_i = 0; _i < _dc_k; _i++) {
                                    if (_dc_seq[_i] == _dc_slot) _entre = 1;
                                    if (_dc_seq[_i] >= 0x7700 && _dc_seq[_i] <= 0x79f0) _fb = 1;
                                }
                                fprintf(stderr, "[c54x] DISPCALL slot=data[0x43d8]=0x%04x "
                                        "d_task_md=%u insn=%u (x%u identiques avant) :",
                                        _dc_slot, (unsigned)_dc_md, s->insn_count, _dc_rep);
                                for (_i = 0; _i < _dc_k; _i++)
                                    fprintf(stderr, " 0x%04x", _dc_seq[_i]);
                                fprintf(stderr, "  -> %s%s\n",
                                        _entre ? "*** ENTRE DANS LE SLOT ***"
                                               : "slot JAMAIS atteint",
                                        _fb ? " + banque FB 0x7700-0x79f0" : "");
                            }
                            _dc_rep = 0;
                            for (_i = 0; _i < _dc_k; _i++) _dc_prev[_i] = _dc_seq[_i];
                            _dc_prevlen = _dc_k; _dc_pslot = _dc_slot; _dc_pmd = _dc_md;
                        }
                    }
                }
            }
        }
        {   /* DMAQ: the firmware's DMA request queue, at the exact moment of
             * the test. Every indirect inference about it proved wrong ("0xaa83
             * is never reached" - no, the PC traces were gated; "the consumer is
             * never called" - it is; "6 reads at 0xaa87" - no, the mailbox
             * monitor folds identical repeated reads). This measures the
             * registers and cells directly at the PC of the `banz`, which is
             * what decides. Gate CALYPSO_DMAQ. */
            static int _dq = -1; static unsigned _dqn = 0;
            if (_dq < 0) _dq = calypso_gate("CALYPSO_DMAQ", 0);
            /* Trigger on a CONDITION, not on the address. Logging the three
             * PCs unconditionally burns the budget at startup (insn ~13M),
             * before any enqueue, and only ever measures "queue empty". Only
             * the moments with something to see are logged:
             *   - the queue is NOT empty at the test (d[433f] != d[433e]), or
             *   - the write pointer GOES BACKWARDS (a reset wiped the queue),
             * plus the very first pass, for a time anchor. */
            if (_dq && _dqn < 60) {
                static uint16_t _dqprev = 0;
                uint16_t _w = s->data[0x433f], _r = s->data[0x433e];
                int _interessant = (_w != _r) || (_dqprev && _w < _dqprev);
                if (exec_pc == 0xaa87 || exec_pc == 0xaa8c || exec_pc == 0xaa8d)
                    _dqprev = _w;
                if ((_interessant || _dqn == 0)
                    && (exec_pc == 0xaa87 || exec_pc == 0xaa8c || exec_pc == 0xaa8d)) {
                    _dqn++;
                    fprintf(stderr, "[c54x] DMAQ PC=0x%04x AR0=%04x AR1=%04x | "
                            "d[433e]=%04x d[433f]=%04x d[3fde]=%04x | %s insn=%u\n",
                            exec_pc, s->ar[0], s->ar[1],
                            s->data[0x433e], s->data[0x433f], s->data[0x3fde],
                            exec_pc == 0xaa8d
                                ? (s->ar[1] ? "banz -> DEPILE" : "banz -> VIDE")
                                : "",
                            s->insn_count);
                }
            }
        }
        {   /* AB38 (CALYPSO_AB38): dumps the opcodes at 0xab38, the shared RET
             * plug, and then follows the flow for 120 instructions to see
             * whether it ever reaches the FB routine. Three rounds. */
            static int _ab = -1;
            if (_ab < 0) _ab = calypso_gate("CALYPSO_AB38", 0);
            if (_ab) {
                static int _dumped = 0, _armed = 0; static unsigned _n = 0, _rounds = 0;
                if (exec_pc == 0xab38) {
                    if (!_dumped) { _dumped = 1;
                        fprintf(stderr, "[c54x] AB38-OPDUMP 0xab38..0xab4f:");
                        for (int _k = 0; _k < 24; _k++)
                            fprintf(stderr, " %04x", prog_fetch(s, (uint16_t)(0xab38 + _k)));
                        fprintf(stderr, "\n"); }
                    if (_rounds < 3) { _rounds++; _armed = 1; _n = 0; }
                }
                if (_armed && _n < 120) { _n++;
                    fprintf(stderr, "[c54x] AB38-FLOW pc=0x%04x xpc=%u op=0x%04x A=0x%06llx insn=%u\n",
                            s->pc, s->xpc, prog_fetch(s, s->pc),
                            (unsigned long long)(s->a & 0xFFFFFFULL), s->insn_count);
                    if (s->pc >= 0x7700 && s->pc <= 0x79f0) {
                        fprintf(stderr, "[c54x] AB38-FLOW *** ATTEINT LA ROUTINE FB 0x%04x ***\n", s->pc);
                        _armed = 0; }
                } else if (_armed) { _armed = 0;
                    fprintf(stderr, "[c54x] AB38-FLOW fin de fenetre (120 pas) sans atteindre 0x7700\n"); }
            }
        }
        {   /* FBROUTE (CALYPSO_FBROUTE): high-water mark of the PC inside the
             * FB routine range, plus the milestones 0x7725 (CALL correlator),
             * 0x798c (unconditional SNR) and 0x79e3/0x79e4 (the guard and the
             * d_fb_det publication), with the DP-relative cell dma(0x7e) the
             * guard wants at 4. */
            static int _fr = -1;
            if (_fr < 0) _fr = calypso_gate("CALYPSO_FBROUTE", 0);
            if (_fr) {
                {   /* FBGATE-TRACE: the last lock, step by step.
                     *
                     * Since the DMA delivers samples the FB routine executes
                     * (it was never entered before) and the state machine in
                     * the DP-relative cell @0x7e advances 0 -> 1 -> 2 -> 3. The
                     * ROM scan gives the only five writers of that cell:
                     *     0x77a8 ST #1   0x77ae ST #2   0x77b4 ST #3
                     *     0x795d ST #4   <- the only one satisfying the guard
                     *                       at 0x79e3
                     * The first four are measured (FBCNT-WR); 0x795d never is,
                     * because the high-water mark stops at 0x794e, fifteen
                     * words earlier.
                     *
                     * What sits at 0x794e (ROM dump):
                     *     0x7949 f310 0005   SUB #5
                     *     0x794b 6ff8 0c3d   absolute access to data[0x0c3d]
                     *     0x794e fc47        CONDITIONAL RETURN - always taken
                     * The routine is not short of data: it DECIDES to leave.
                     *
                     * Reading 0xfc47 as a conditional return (by analogy with
                     * 0xfc00 RET and the 0xfc45 in the dispatcher just before)
                     * and 6ff8 0c3d as an absolute access are both hypotheses,
                     * so the real path and the flags are measured instead of
                     * decoded by hand. If 0x794e is passed even once the
                     * condition is not constant; if it exits every time, the
                     * flags (TC, C) and data[0x0c3d] say which one decides.
                     * data[0x0c3d] is NOT in the range the DMA fills
                     * (0x0cce..0x0cce+296) - 145 words before it. */
                    if (exec_pc >= 0x7944 && exec_pc <= 0x7962) {
                        static unsigned _n = 0;
                        if (_n < 120) {
                            _n++;
                            fprintf(stderr,
                                    /* AR4 is what decides. Cross-disassembly
                                     * (binutils table plus tic54x-dis):
                                     *     0x7944 add *AR4, A ; 0x7945 sub #2, A
                                     *     0x794e rc ALEQ      (return if A <= 0)
                                     * Measured: *AR4 yields 1 then 2 (high
                                     * word), so A is -1 then 0 and the guard
                                     * exits every time; it needs *AR4 > 2. That
                                     * looks like a correlation counter that
                                     * must pass a threshold of 2 - the DSP does
                                     * detect something, just not enough. The
                                     * POINTER and the CELL it points at are
                                     * printed, because the question is now who
                                     * fills that buffer, not what the condition
                                     * evaluates to. */
                                    "[c54x] FBGATE pc=0x%04x op=0x%04x A=0x%06llx "
                                    "B=0x%06llx TC=%d C=%d AR4=0x%04x *AR4=0x%04x "
                                    "(data[]=0x%04x) data[0x0c3d]=0x%04x "
                                    "data[0x41fe]=0x%04x insn=%u\n",
                                    exec_pc, prog_fetch(s, exec_pc),
                                    (unsigned long long)(s->a & 0xFFFFFFULL),
                                    (unsigned long long)(s->b & 0xFFFFFFULL),
                                    (s->st0 >> 12) & 1, (s->st0 >> 11) & 1,
                                    /* Reading s->data[AR4] alone is wrong for
                                     * any address in the API window: the DSP
                                     * reads api_ram[], a DIFFERENT array (see
                                     * data_read_locked). AR4 holds 0x0cd0, well
                                     * inside both the API window and the DMA
                                     * buffer, so data[] alone shows 0x0000
                                     * while the cell is fed. BOTH views are
                                     * printed, so any divergence stays visible
                                     * instead of being assumed. */
                                    s->ar[4],
                                    (s->api_ram && s->ar[4] >= C54X_API_BASE &&
                                     s->ar[4] < C54X_API_BASE + C54X_API_SIZE)
                                        ? s->api_ram[s->ar[4] - C54X_API_BASE]
                                        : s->data[s->ar[4]],
                                    s->data[s->ar[4]],
                                    s->data[0x0c3d], s->data[0x41fe],
                                    s->insn_count);
                            fflush(stderr);
                        }
                    }
                }
                static unsigned _in = 0, _hi = 0, _n76fb = 0;
                if (exec_pc >= 0x7700 && exec_pc <= 0x79f0) {
                    _in++;
                    if (exec_pc > _hi) {
                        _hi = exec_pc;
                        fprintf(stderr, "[c54x] FBROUTE high-water PC=0x%04x (hits=%u) "
                                "[0x7725=CALL corr, 0x798c=SNR incond, 0x79e4=ORM d_fb_det]\n",
                                exec_pc, _in);
                    }
                }
                if (exec_pc == 0x76fb && _n76fb < 10) {
                    _n76fb++;
                    fprintf(stderr, "[c54x] FBROUTE ENTER @0x76fb (BD 0x7700) #%u insn=%u\n",
                            _n76fb, s->insn_count);
                }
                static unsigned _ms[5] = {0,0,0,0,0};
                const uint16_t _mpc[5] = {0x7720, 0x7725, 0x798c, 0x79e3, 0x79e4};
                for (int _k = 0; _k < 5; _k++) {
                    if (exec_pc == _mpc[_k] && _ms[_k]++ < 6) {
                        unsigned _dp = (unsigned)(s->st0 & 0x1FF);
                        unsigned _ea = _dp * 0x80 + 0x7e;   /* dma(0x7e), DP-relative */
                        fprintf(stderr, "[c54x] FBROUTE jalon PC=0x%04x #%u A=0x%06llx "
                                "DP=0x%03x dma(0x7e)=data[0x%04x]=0x%04x (garde veut 4) insn=%u\n",
                                exec_pc, _ms[_k], (unsigned long long)(s->a & 0xFFFFFFULL),
                                _dp, _ea, (_ea < 0x10000) ? s->data[_ea] : 0, s->insn_count);
                    }
                }
            }
        }
        if (exec_pc == 0x93a5) {   /* DARAM 0x2a00 consumer (AR3 post-increment) = the real correlator input */
            static int _b2c = -1; static unsigned _b2cn = 0;
            if (_b2c < 0) _b2c = calypso_gate("CALYPSO_B2SEQ", 0);
            if (_b2c && _b2cn < 8) { _b2cn++;
                fprintf(stderr, "[c54x] B2SEQ-IN 0x2a00@0x93a5 (I,Q)x16:");
                for (int _i = 0; _i < 16; _i++)
                    fprintf(stderr, " (%d,%d)", (int)(int16_t)s->data[0x2a00 + 2*_i], (int)(int16_t)s->data[0x2a00 + 2*_i + 1]);
                fprintf(stderr, "\n"); }
        }
        {
            /* DARAM-DUMP (CALYPSO_DARAM_DUMP): writes the correlator input
             * buffer as binary IQ16, so tools/corr_iq.py --src bursts can
             * measure coherence and dphi instead of judging by eye. */
            static int _dd = -1; static FILE *_ddf = NULL; static unsigned _ddn = 0;
            static uint16_t _ddpc = 0x9ac0; static unsigned _ddmax = 200;
            static uint16_t _ddaddr = 0x2a00;   /* recorded base, see CALYPSO_DARAM_DUMP_ADDR */
            if (_dd < 0) {
                const char *e = getenv("CALYPSO_DARAM_DUMP");
                _dd = (e && *e && strcmp(e, "0")) ? 1 : 0;
                if (_dd) {
                    const char *path = (strcmp(e, "1") == 0) ? "/dev/shm/daram_2a00.cfile" : e;
                    const char *p = getenv("CALYPSO_DARAM_DUMP_PC");
                    if (p && *p) _ddpc = (uint16_t)strtol(p, NULL, 0);
                    const char *m = getenv("CALYPSO_DARAM_DUMP_MAX");
                    if (m && *m) _ddmax = (unsigned)atoi(m);
                    /* The recorded base must be configurable: the native
                     * profiles deliver at CALYPSO_BSP_DARAM_ADDR=0x4c00, and a
                     * hard-coded 0x2a00 records a buffer nobody feeds. This
                     * probe writes from the CPU thread, so it is coherent by
                     * construction - unlike the QEMU monitor, which reads
                     * outside that thread while the BSP rewrites the 296 words
                     * during the `xp`, giving coherence around 0.5 and a
                     * drifting FFT peak (two bursts spliced together). */
                    const char *ad = getenv("CALYPSO_DARAM_DUMP_ADDR");
                    if (ad && *ad) _ddaddr = (uint16_t)strtol(ad, NULL, 0);
                    fprintf(stderr, "[c54x] DARAM-DUMP base=0x%04x "
                            "(CALYPSO_DARAM_DUMP_ADDR)\n", _ddaddr);
                    _ddf = fopen(path, "wb");
                    fprintf(stderr, "[c54x] DARAM-DUMP armed pc=0x%04x max=%u -> %s (%s)\n",
                            _ddpc, _ddmax, path, _ddf ? "ok" : "FOPEN FAILED");
                }
            }
            /* Record only the passes that are searching for FCCH. Without
             * this guard the _ddmax budget is spent during boot, while
             * d_fb_mode (0x08f9) is 0, so the dump never shows the FB phase and
             * reads as "the buffer never contains FCCH".
             * CALYPSO_DARAM_DUMP_ANYMODE=1 records everything. */
            static int _ddany = -1;
            if (_ddany < 0) { const char *e = getenv("CALYPSO_DARAM_DUMP_ANYMODE");
                              _ddany = (e && atoi(e) > 0) ? 1 : 0; }
            if (_dd && _ddf && exec_pc == _ddpc && _ddn < _ddmax &&
                (_ddany || s->data[0x08f9] != 0)) {
                unsigned char hdr[12];
                unsigned nw = 296;   /* 296 words = 148 I/Q pairs */
                hdr[0]='I'; hdr[1]='Q'; hdr[2]='1'; hdr[3]='6';
                unsigned _fnv = calypso_daram_last_fn;   /* real GSM fn of the last deposit */
                hdr[4]=(unsigned char)(_fnv & 0xff); hdr[5]=(unsigned char)((_fnv >> 8) & 0xff);
                hdr[6]=(unsigned char)((_fnv >> 16) & 0xff); hdr[7]=(unsigned char)((_fnv >> 24) & 0xff);
                hdr[8]=0;
                hdr[9]=(unsigned char)(nw & 0xff); hdr[10]=(unsigned char)((nw >> 8) & 0xff);
                hdr[11]=0;
                fwrite(hdr, 1, 12, _ddf);
                for (unsigned _k = 0; _k < nw; _k++) {
                    uint16_t _v = s->data[_ddaddr + _k];
                    unsigned char _b[2]; _b[0]=(unsigned char)(_v & 0xff); _b[1]=(unsigned char)(_v >> 8);
                    fwrite(_b, 1, 2, _ddf);
                }
                /* DARAM-SANITY: in-run verdict on what was just dumped.
                 * coh = |Sum z[k+1].conj(z[k])| / Sum |z[k+1]||z[k]|;
                 * dphi = arg(Sum ...). The FB kernel wants FCCH at 1 SPS, so
                 * dphi = +pi/2 (+1.571); +0.393 means 4 SPS undecimated, and
                 * the verdict string names the remedy. */
                if (_ddn == 0 || (_ddn % 50) == 0) {
                    double ar = 0, ai = 0, dn = 0, en = 0;
                    unsigned np = nw / 2;
                    for (unsigned _k = 1; _k < np; _k++) {
                        double i0 = (int16_t)s->data[_ddaddr + 2*(_k-1)];
                        double q0 = (int16_t)s->data[_ddaddr + 2*(_k-1) + 1];
                        double i1 = (int16_t)s->data[_ddaddr + 2*_k];
                        double q1 = (int16_t)s->data[_ddaddr + 2*_k + 1];
                        ar += i1*i0 + q1*q0; ai += q1*i0 - i1*q0;
                        dn += sqrt((i0*i0 + q0*q0) * (i1*i1 + q1*q1));
                        en += i1*i1 + q1*q1;
                    }
                    double coh  = dn > 0 ? sqrt(ar*ar + ai*ai) / dn : 0.0;
                    double dphi = atan2(ai, ar);
                    double rms  = np > 1 ? sqrt(en / (double)(np - 1)) : 0.0;
                    double ad   = fabs(dphi);
                    const char *v;
                    if (rms < 1.0)                        v = "VIDE (buffer non alimente)";
                    else if (coh > 0.90 && ad > 1.37 && ad < 1.77)
                        v = (dphi > 0) ? "FCCH @1SPS OK -- c est ce que le kernel cherche"
                                       : "FCCH @1SPS MIROIR -> CALYPSO_DL_IQ_CONJ=1";
                    else if (coh > 0.90 && ad > 0.29 && ad < 0.49)
                        v = "4 SPS NON DECIME -> CALYPSO_BSP_IQ_DECIM=4 (et FB_IQ_OWNS=0)";
                    else if (coh > 0.90 && ad < 0.29)
                        v = "sur-echantillonne (>4 SPS) ou pas un ton -> verifier la source";
                    else                                  v = "BRUIT/DATA -- pas un ton FCCH";
                    static unsigned _prevwr = 0;
                    unsigned _dwr = calypso_daram_wr_count - _prevwr;
                    _prevwr = calypso_daram_wr_count;
                    fprintf(stderr, "[c54x] DARAM-SANITY rec=%u fn=%u depots_depuis=%u coh=%.3f "
                            "dphi=%+.3f (%+.2fxpi/2) rms=%.0f : %s%s\n", _ddn,
                            calypso_daram_last_fn, _dwr, coh, dphi, dphi / (M_PI/2), rms, v,
                            _dwr == 0 ? "  [BUFFER FIGE : aucun depot BSP depuis le dump precedent]" : "");
                }
                if (++_ddn >= _ddmax) {
                    fflush(_ddf); fclose(_ddf); _ddf = NULL;
                    fprintf(stderr, "[c54x] DARAM-DUMP done : %u records de %u mots\n", _ddn, nw);
                } else if ((_ddn % 20) == 0) {
                    fflush(_ddf);
                    fprintf(stderr, "[c54x] DARAM-DUMP rec=%u\n", _ddn);
                }
            }
        }
        if (exec_pc == 0x9ac0) {
            /* B2SEQ (CALYPSO_B2SEQ): dumps 16 (I,Q) pairs from 0x2a00, the
             * real correlator input fed by the BSP/ADC. An Fs/4 pattern is
             * FCCH: (a,0) (0,a) (-a,0) (0,-a) ...; a near-constant sequence is
             * DC with no tone, i.e. an empty input. */
            { static int _b2s = -1; static unsigned _b2sn = 0;
              if (_b2s < 0) _b2s = calypso_gate("CALYPSO_B2SEQ", 0);
              if (_b2s && _b2sn < 8) { _b2sn++;
                  fprintf(stderr, "[c54x] B2SEQ 0x2a00 (I,Q)x16:");
                  for (int _i = 0; _i < 16; _i++)
                      fprintf(stderr, " (%d,%d)", (int)(int16_t)s->data[0x2a00 + 2*_i], (int)(int16_t)s->data[0x2a00 + 2*_i + 1]);
                  fprintf(stderr, "\n"); } }
            static unsigned dr = 0;
            if (dr < 30 || (dr % 200) == 0)
                fprintf(stderr, "[c54x] DETECTOR-RUN #%u @0x9ac0 d_fb_mode[08f9]=0x%04x "
                        "d_fb_det[08f8]=0x%04x insn=%u\n",
                        dr, s->data[0x08f9], s->data[0x08f8], s->insn_count);
            dr++;
            /* B2AR (CALYPSO_B2AR): where the correlator ARs point and what
             * they read. AR5 and AR3 inside [0x2a00..0x2b27] means it reads the
             * FCCH; outside means it reads next to it, i.e. a badly
             * initialised pointer. */
            { static int _b2a = -1; static unsigned _b2an = 0;
              if (_b2a < 0) _b2a = calypso_gate("CALYPSO_B2AR", 0);
              if (_b2a && _b2an < 12) { _b2an++;
                  int _a3in = (s->ar[3] >= 0x2a00 && s->ar[3] < 0x2b28);
                  int _a5in = (s->ar[5] >= 0x2a00 && s->ar[5] < 0x2b28);
                  fprintf(stderr, "[c54x] B2AR @0x9ac0 AR2=%04x AR3=%04x[%d]%s AR4=%04x AR5=%04x[%d]%s\n",
                          s->ar[2], s->ar[3], (int)(int16_t)s->data[s->ar[3]], _a3in?"IN":"OOB",
                          s->ar[4], s->ar[5], (int)(int16_t)s->data[s->ar[5]], _a5in?"IN":"OOB"); } }
            /* B2 (CALYPSO_B2): magnitude of accumulators A and B plus the max
             * and its index over the 296 words of the input (0x2a00) and the
             * workspace (0x2c00). Tells "all zero" from "flat, no peak". */
            {
                static int _b2 = -1; static unsigned _b2n = 0;
                if (_b2 < 0) _b2 = calypso_gate("CALYPSO_B2", 0);
                if (_b2 && _b2n < 24) {
                    _b2n++;
                    int64_t A = ((int64_t)(s->a & 0xFFFFFFFFFFULL) << 24) >> 24;
                    int64_t B = ((int64_t)(s->b & 0xFFFFFFFFFFULL) << 24) >> 24;
                    uint16_t mi = 0, mw = 0; int xi = 0, xw = 0;
                    for (int i = 0; i < 296; i++) { int16_t v = (int16_t)s->data[0x2a00 + i]; uint16_t a = v < 0 ? (uint16_t)(-v) : (uint16_t)v; if (a > mi) { mi = a; xi = i; } }
                    for (int i = 0; i < 296; i++) { int16_t v = (int16_t)s->data[0x2c00 + i]; uint16_t a = v < 0 ? (uint16_t)(-v) : (uint16_t)v; if (a > mw) { mw = a; xw = i; } }
                    fprintf(stderr, "[c54x] B2 @0x9ac0 |A|=%lld |B|=%lld  in(2a00):max=%u@%d  ws(2c00):max=%u@%d\n",
                            (long long)(A < 0 ? -A : A), (long long)(B < 0 ? -B : B), mi, xi, mw, xw);
                }
            }
        }
        if (exec_pc == 0xec07) {
            static unsigned cr = 0;
            if (cr < 30) {
                int64_t a = (s->a & 0x8000000000LL) ? (int64_t)(s->a | ~0xFFFFFFFFFFLL) : (int64_t)s->a;
                int64_t b = (s->b & 0x8000000000LL) ? (int64_t)(s->b | ~0xFFFFFFFFFFLL) : (int64_t)s->b;
                /* angle = atan2(B,A) is computed at analysis time. */
                fprintf(stderr, "[c54x] CORR-ABG #%u A=%lld B=%lld | "
                        "AR2=%04x[%04x] AR3=%04x[%04x] AR4=%04x[%04x] AR5=%04x[%04x] insn=%u\n",
                        cr, (long long)a, (long long)b,
                        s->ar[2], s->data[s->ar[2]], s->ar[3], s->data[s->ar[3]],
                        s->ar[4], s->data[s->ar[4]], s->ar[5], s->data[s->ar[5]],
                        s->insn_count);
                cr++;
            }
        }

        /* CORR-PEAK probe: at the TOA store (PC=0x9ac0, STL A into
         * a_sync_demod) dumps A and B in full, the ARs, T and the input window
         * read (BSP buffer at 0x2a00), to see how the correlator derives the
         * TOA - peak offset, wrap, reference - from an otherwise correct FCCH.
         * 40 lines, negligible cost off-site. */
        if (exec_pc == 0xa0e7 || exec_pc == 0x9ac0) {
            static unsigned cp_log = 0;
            /* Fire only when real I/Q is present, otherwise the budget is spent
             * on the boot with an empty buffer. */
            if (cp_log < 40 && (s->data[0x2a00] || s->data[0x2a02] ||
                                s->data[0x2a04] || s->data[0x2a08] ||
                                s->data[0x2a10] || s->data[0x2a20])) {
                uint16_t a3 = s->ar[3];
                fprintf(stderr,
                    "[c54x] CORR-PEAK #%u PC=0x%04x A=%010llx B=%010llx T=%04x "
                    "AR2=%04x AR3=%04x AR4=%04x AR5=%04x\n"
                    "[c54x]   in@0x2a00: %04x %04x %04x %04x %04x %04x %04x %04x\n"
                    "[c54x]   in@0x2c00: %04x %04x %04x %04x %04x %04x %04x %04x\n"
                    "[c54x]   *AR3@0x%04x: %04x %04x %04x %04x %04x %04x %04x %04x insn=%u\n",
                    cp_log, exec_pc,
                    (unsigned long long)(s->a & 0xFFFFFFFFFFULL),
                    (unsigned long long)(s->b & 0xFFFFFFFFFFULL),
                    s->t, s->ar[2], a3, s->ar[4], s->ar[5],
                    s->data[0x2a00], s->data[0x2a01], s->data[0x2a02], s->data[0x2a03],
                    s->data[0x2a04], s->data[0x2a05], s->data[0x2a06], s->data[0x2a07],
                    s->data[0x2c00], s->data[0x2c01], s->data[0x2c02], s->data[0x2c03],
                    s->data[0x2c04], s->data[0x2c05], s->data[0x2c06], s->data[0x2c07],
                    a3,
                    s->data[a3], s->data[(uint16_t)(a3+1)], s->data[(uint16_t)(a3+2)], s->data[(uint16_t)(a3+3)],
                    s->data[(uint16_t)(a3+4)], s->data[(uint16_t)(a3+5)], s->data[(uint16_t)(a3+6)], s->data[(uint16_t)(a3+7)],
                    s->insn_count);
                fflush(stderr);
                cp_log++;
            }
        }

        /* Track A writes (probe 3, post-exec): exec_pc is the PC that just
         * executed, so if A changed, that opcode wrote it. */
        if (s->a != a_before_exec) {
            p_last_a_pc  = exec_pc;
            p_last_a_val = s->a;
        }
        /* End of the CALA-70C3 forensic probes. */

        /* BOOT-BRANCH probe: control-flow skeleton of the boot phase. Hunts
         * the opcode that branches OVER the DSP init code, which should set
         * SP=0x5AC8, AR4/AR5 and IMR=0xFFFF. The IMR-W trace localised the
         * collateral damage at PC=0xf03a (insn 262501) and PC=0x8ebc (insn
         * 5247868), through a stale ARx=0 turning STH B,*ARx+ into a write to
         * MMR_IMR; the bad branch happens earlier. Logs every PC discontinuity
         * (branch, call, return, interrupt entry) while insn_count <= 300000,
         * with SP/IMR/AR4/AR5, which shows when each register was armed, or
         * that it never was. */
        if (s->insn_count <= 300000) {
            /* c54x_exec_one does NOT advance s->pc for a sequential
             * instruction; `s->pc += consumed` further down does. So during
             * the exec itself only a real branch, CALL, RET or interrupt entry
             * modifies s->pc. */
            if (s->pc != exec_pc) {
                static unsigned boot_br_log;
                const unsigned LIMIT = 8000;
                if (boot_br_log < LIMIT) {
                    if (calypso_debug_enabled("BOOT-BRANCH")) fprintf(stderr,
                            "[c54x] BOOT-BRANCH #%u insn=%u %04x(op=%04x,c=%d) → %04x "
                            "SP=%04x IMR=%04x AR4=%04x AR5=%04x INTM=%d\n",
                            boot_br_log, s->insn_count,
                            exec_pc, exec_op, consumed, s->pc,
                            s->sp, s->imr, s->ar[4], s->ar[5],
                            !!(s->st1 & ST1_INTM));
                    boot_br_log++;
                    if (boot_br_log == LIMIT) {
                        if (calypso_debug_enabled("BOOT-BRANCH")) fprintf(stderr,
                                "[c54x] BOOT-BRANCH log capped at %u\n", LIMIT);
                    }
                }
            }
        }

        /* INT3-CYCLE-TRACE (CALYPSO_INT3_CYCLE_TRACE=1): records the branch
         * decisions taken during the INT3 ISR cycle. */
        int3_cycle_track_branch(s, exec_pc, exec_op, consumed);

        /* SP changes, logged only after init (insn > 490M). */
        if (s->sp != sp_before && s->insn_count > 490000000) {
            static int sp_leak_log = 0;
            if (sp_leak_log < 100) {
                C54_LOG("SP %+d PC=0x%04x op=0x%04x SP 0x%04x→0x%04x insn=%u",
                        (int16_t)(s->sp - sp_before), exec_pc, exec_op, sp_before, s->sp, s->insn_count);
                sp_leak_log++;
            }
        }

        /* SP-FLOOR guard plus delta histogram. Trips on the FIRST descent of
         * SP below SP_FLOOR: that snapshot is taken BEFORE the MMR
         * self-corruption (SP lives at MMR data[0x18]), so it captures the
         * cause, not the crash. The running delta histogram identifies leaking
         * call/return pairs (a far push of 2 words against a near pop of 1
         * loses one word per pair). On by default, a few branches per
         * instruction. */
        #define SP_FLOOR 0x0080
        {
            static int sp_floor_tripped = 0;
            static uint64_t sp_delta_pushf = 0;  /* delta == -2 */
            static uint64_t sp_delta_pushn = 0;  /* delta == -1 */
            static uint64_t sp_delta_popn  = 0;  /* delta == +1 */
            static uint64_t sp_delta_popf  = 0;  /* delta == +2 */
            static uint64_t sp_delta_other = 0;  /* anything else (jumps, LD#k,SP) */
            static uint64_t sp_delta_log_n = 0;

            int delta = (int)(int16_t)(s->sp - sp_before);
            if (delta != 0) {
                switch (delta) {
                    case -2: sp_delta_pushf++; break;
                    case -1: sp_delta_pushn++; break;
                    case  1: sp_delta_popn++;  break;
                    case  2: sp_delta_popf++;  break;
                    default: sp_delta_other++; break;
                }
                /* First 80 SP changes, then every 5000: enough to
                 * characterise the leaking call/return pair without drowning
                 * the log. */
                sp_delta_log_n++;
                if (sp_delta_log_n <= 80 || (sp_delta_log_n % 5000) == 0) {
                    C54_LOG("SP-Δ #%llu PC=0x%04x op=0x%04x XPC=%u Δ%+d  SP 0x%04x→0x%04x insn=%u",
                            (unsigned long long)sp_delta_log_n,
                            exec_pc, exec_op, s->xpc & 0x3,
                            delta, sp_before, s->sp, s->insn_count);
                }
            }

            /* Detectors, once per instruction AFTER exec_one. */
            /* (1) A-write ring: track each modification of s->a */
            if (s->a != pre_a) {
                awrite_log_push(pre_pc, pre_xpc, pre_op, pre_a, s->a, s->insn_count);
            }
            /* (2) Transfer ring : detect non-sequential PC change.
             *   exec_one returns `consumed` = instruction size in words. If
             *   new PC != pre_pc + consumed (and no delay-slot pending), it's
             *   a transfer. We also catch XPC changes (FAR transfers). */
            uint16_t expected_pc = (uint16_t)(pre_pc + consumed);
            if ((s->pc != expected_pc || (s->xpc & 0x3) != pre_xpc)
                && s->delay_slots == 0) {
                xfer_log_push(pre_pc, pre_xpc, pre_op,
                              s->pc, s->xpc & 0x3, pre_a, s->insn_count);
            }
            /* (3) NOP-region guard : trip ONCE at first entry into the
             * unmapped prog zone (PC <0x7000 in bank 0, outside OVLY DARAM
             * 0x80-0x27FF). Dumps trigger + transfer ring + A-write ring. */
            if (!g_nop_tripped && pc_in_nop_region(s, s->pc, s->xpc & 0x3)) {
                nop_guard_dump(s, s->pc, s->xpc & 0x3);
            }

            if (!sp_floor_tripped && s->sp < SP_FLOOR) {
                sp_floor_tripped = 1;
                long long net_push = (long long)(sp_delta_pushf*2 + sp_delta_pushn);
                long long net_pop  = (long long)(sp_delta_popf *2 + sp_delta_popn);
                C54_LOG("================================================");
                C54_LOG("SP-FLOOR TRIPPED  SP=0x%04x < 0x%04x  insn=%u",
                        s->sp, (unsigned)SP_FLOOR, s->insn_count);
                C54_LOG("  trigger PC=0x%04x op=0x%04x XPC=%u Δ%+d (SP 0x%04x→0x%04x)",
                        exec_pc, exec_op, s->xpc & 0x3, delta, sp_before, s->sp);
                C54_LOG("  ST1 INTM=%d  IFR=0x%04x  IMR=0x%04x",
                        !!(s->st1 & ST1_INTM), s->ifr, s->imr);
                C54_LOG("SP delta histogram :");
                C54_LOG("  push far  (Δ=-2) : %llu", (unsigned long long)sp_delta_pushf);
                C54_LOG("  push near (Δ=-1) : %llu", (unsigned long long)sp_delta_pushn);
                C54_LOG("  pop  near (Δ=+1) : %llu", (unsigned long long)sp_delta_popn);
                C54_LOG("  pop  far  (Δ=+2) : %llu", (unsigned long long)sp_delta_popf);
                C54_LOG("  other            : %llu", (unsigned long long)sp_delta_other);
                C54_LOG("  net push - pop   : %lld words (positive = SP leaked downward)",
                        net_push - net_pop);
                C54_LOG("================================================");
            }
        }
        #undef SP_FLOOR

        /* SP observability:
         *   (a) sp_trail[256]  : |delta| > 32 events (scheduler reloads, large
         *                        allocations)
         *   (b) sp_low watermark: every new low, coalesced per PC on powers of
         *                        ten
         * Gated to insn > 33754, i.e. after the normal init stack move
         * 0x9022 -> 0x5ac8. */
        {
            static int trap_armed = -1;
            if (trap_armed < 0) {
                const char *e = cdbg_env("TRAP-OOR"); (void)e;
                /* Forced off: this was the SP descent / SP-CATASTROPHE
                 * analysis, now resolved (0x70c3 self-CALA, DROM LUT, stub),
                 * and the companion site below set s->running = 0 at the 4.2M
                 * checkpoint, which halted the DSP under CALYPSO_DEBUG=ALL. */
                trap_armed = 0;
            }
            if (trap_armed && s->sp != sp_before && s->insn_count > 33754) {
                int16_t delta = (int16_t)(s->sp - sp_before);
                uint16_t a_low = (uint16_t)(s->a & 0xFFFF);

                /* Per-PC SP-HIST accounting lives at the top-of-loop
                 * chokepoint, which is bypass-proof. Calling it here too would
                 * double-count. */

                /* (a) trail: big jumps only, skipping push/pop noise of 1..32 */
                if (delta > 32 || delta < -32) {
                    unsigned k = g_sp_trail_idx & 255;
                    g_sp_trail[k].insn    = s->insn_count;
                    g_sp_trail[k].old_sp  = sp_before;
                    g_sp_trail[k].new_sp  = s->sp;
                    g_sp_trail[k].exec_pc = exec_pc;
                    g_sp_trail[k].exec_op = exec_op;
                    g_sp_trail[k].a_low   = a_low;
                    g_sp_trail_idx++;
                }

                /* (b) sp_low watermark: fires on any new low, including
                 * delta = -1. */
                if (s->sp < g_sp_low) {
                    g_sp_low = s->sp;
                    if (exec_pc == g_sp_low_pc) {
                        g_sp_low_hits_at_pc++;
                        unsigned n = g_sp_low_hits_at_pc;
                        bool milestone = (n == 1 || n == 10 || n == 100 ||
                                          n == 1000 || n == 10000 || n == 100000);
                        if (milestone) {
                            if (calypso_debug_enabled("SP-LOW")) fprintf(stderr,
                                "[c54x] SP-LOW #%u @pc=0x%04x op=0x%04x "
                                "sp 0x%04x->0x%04x A_low=0x%04x insn=%u\n",
                                n, exec_pc, exec_op,
                                sp_before, s->sp, a_low, s->insn_count);
                        }
                    } else {
                        g_sp_low_pc = exec_pc;
                        g_sp_low_hits_at_pc = 1;
                        g_sp_low_distinct_pcs++;
                        if (calypso_debug_enabled("SP-LOW")) fprintf(stderr,
                            "[c54x] SP-LOW NEW (#%u distinct) @pc=0x%04x op=0x%04x "
                            "sp 0x%04x->0x%04x A_low=0x%04x insn=%u\n",
                            g_sp_low_distinct_pcs, exec_pc, exec_op,
                            sp_before, s->sp, a_low, s->insn_count);
                    }
                }
            }
        }

        /* SP catastrophic delta tracer. SP has been seen going from 0x9c1e to
         * 0x0001 inside one window, losing ~40k stack words; the progressive
         * leak log above caps at 100 small deltas and misses that single
         * event. This block flags any |delta| > 256 in one instruction, never
         * capped, so the offending STM / PSHM / POPM / RETE-on-corrupt-stack /
         * FRAME-with-huge-offset is named the FIRST time it happens. The ARs
         * are included, to see whether the ST/LD destination resolved to an MMR
         * slot (for instance *AR = 0x18 = MMR_SP).
         *
         * The threshold is 256, not 100, to filter legitimate FRAME #imm8s,
         * which is a signed 8-bit value and can reach +-127. Real
         * catastrophes, from a dual-operand write into MMR_SP, are always
         * thousands of words. */
        {
            int32_t dsp = (int32_t)(int16_t)(s->sp - sp_before);
            if (dsp > 256 || dsp < -256) {
                if (calypso_debug_enabled("SP-CATASTROPHE")) fprintf(stderr,
                        "[c54x] SP-CATASTROPHE Δ=%+d PC=0x%04x op=0x%04x "
                        "SP 0x%04x → 0x%04x INTM=%d "
                        "AR0..7: %04x %04x %04x %04x %04x %04x %04x %04x "
                        "BK=%04x A=%010llx insn=%u\n",
                        (int)dsp, exec_pc, exec_op, sp_before, s->sp,
                        !!(s->st1 & ST1_INTM),
                        s->ar[0], s->ar[1], s->ar[2], s->ar[3],
                        s->ar[4], s->ar[5], s->ar[6], s->ar[7],
                        s->bk,
                        (unsigned long long)(s->a & 0xFFFFFFFFFFULL),
                        s->insn_count);
            }
        }
        /* TRAP-OOR firing point: halt at a fixed insn checkpoint and dump the
         * trail plus sp_low, for offline analysis of the whole descent. No PC
         * whitelist and no SP edge can catch the clobber, because it lives in
         * legitimate code. Checkpoint set by CALYPSO_TRAP_CHECKPOINT (default
         * 4200000, just after the SP recovery 0x0008 -> 0x2900 at insn
         * 4.09M). */
        {
            static int trap_armed = -1;
            static int tripped = 0;
            static unsigned checkpoint = 0;
            if (trap_armed < 0) {
                const char *e = cdbg_env("TRAP-OOR"); (void)e;
                /* Forced off: setting s->running = 0 at the checkpoint halted
                 * the DSP under CALYPSO_DEBUG=ALL, and the analysis it served
                 * is resolved. */
                trap_armed = 0;
                const char *c = getenv("CALYPSO_TRAP_CHECKPOINT");
                checkpoint = (c && *c) ? (unsigned)strtoul(c, NULL, 0) : 4200000u;
            }
            if (trap_armed && !tripped && s->insn_count >= checkpoint) {
                tripped = 1;
                dsp_trap_dump(s, exec_pc, exec_op, sp_before, "CHECKPOINT");
                s->running = 0;
            }
        }

        /* DUAL-OP-INTERPRET diagnostic. Compares the decoder's AR field
         * interpretation (3-bit fields) with the SPRU172C dual-operand
         * encoding (2-bit AR fields plus an offset of 2, AR2..AR5 only). When
         * the two disagree on which AR is used and SP-CATASTROPHE has just
         * fired, the encoding is the suspect. 100 entries. */
        if ((exec_op & 0xFC00) == 0xC800 && (
             (int32_t)(int16_t)(s->sp - sp_before) > 100 ||
             (int32_t)(int16_t)(s->sp - sp_before) < -100)) {
            static unsigned dop_log;
            if (dop_log++ < 100) {
                int xar_cur = (exec_op >> 4) & 0x07;
                int yar_cur = exec_op & 0x07;
                int xar_spru = ((exec_op >> 4) & 0x03) + 2;
                int yar_spru = (exec_op & 0x03) + 2;
                int xmod_spru = (exec_op >> 6) & 0x03;
                int ymod_spru = (exec_op >> 2) & 0x03;
                fprintf(stderr,
                        "[c54x] DUAL-OP-INTERPRET op=0x%04x PC=0x%04x : "
                        "current_dec X=AR%d Y=AR%d (3bit) | "
                        "SPRU172C    X=AR%d Y=AR%d xmod=%d ymod=%d (2bit+2) | "
                        "AR%d_cur=%04x AR%d_spru=%04x | "
                        "AR%d_cur=%04x AR%d_spru=%04x\n",
                        exec_op, exec_pc,
                        xar_cur, yar_cur,
                        xar_spru, yar_spru, xmod_spru, ymod_spru,
                        xar_cur, s->ar[xar_cur],
                        xar_spru, s->ar[xar_spru],
                        yar_cur, s->ar[yar_cur],
                        yar_spru, s->ar[yar_spru]);
            }
        }

        /* Snapshot the just-executed PC/op into C54xState so other
         * tracers (in particular INTM-TRANS at top of next iteration)
         * can attribute post-instruction state changes to the cause. */
        s->last_exec_pc = exec_pc;
        s->last_exec_op = exec_op;


        /* RPT: after executing an instruction while a repeat is active,
         * re-execute the SAME instruction (do not advance PC) until the count
         * reaches 0. */
        if (s->rpt_active && !s->idle) {
            /* RPT #k repeats the next instruction k+1 times (TI SPRU172C).
             * The RPT handler sets rpt_active = 1, advances the PC and returns;
             * this block runs immediately afterwards with rpt_active already
             * true, so without the guard below the RPT instruction itself would
             * consume one repetition and the repeated instruction would run
             * only k times.
             *
             * Measured (READA-ITER probe on the table loader at 0xb4c4):
             *     rpt #0x02 ; reada *AR1+
             *     #1 addr=0x43d5 rpt_count=1   <- should be 2
             *     #2 addr=0x43d6 rpt_count=0
             *     (no #3)      -> data[0x43d7] NEVER initialised
             * The first loop of the same loader, `rpt #0x4d`, writes
             * 0x4387..0x43d3 = 77 words = k, whereas k+1 = 78 would reach
             * 0x43d4. Both loops are short by exactly one copy.
             *
             * Scope: EVERY RPT loop of the firmware, including the block copy
             * `rpt #0x0b ; mvdd` that feeds a_cd[3..14], which copied only 11
             * of the 12 L2 payload words. Handling it here, once, covers every
             * form (RPT #k8, #lk, Smem, RPTZ).
             *
             * This is not an error in the computation of rpt_count - the eight
             * setters are correct - it is the loop consuming one iteration too
             * many. */
            if (!rpt_was_active) {
                /* The instruction just executed IS the RPT, so it must
                 * not consume a repetition. The handler already advanced
                 * the PC, so loop again without decrementing. */
                s->cycles++;
                executed++;
                continue;
            }
            if (s->rpt_count > 0) {
                s->rpt_count--;
                /* Don't advance PC — re-execute same instruction next cycle */
                s->cycles++;
                executed++;
                if (s->rpt_count == 0) {
                    static int rpt_done_log = 0;
                    if (rpt_done_log < 10)
                        C54_LOG("RPT DONE PC=0x%04x op=0x%04x count_was=%d", s->pc, prog_fetch(s, s->pc), 0);
                    rpt_done_log++;
                }
                continue;
            } else {
                s->rpt_active = false;
                s->par_set = false;
            }
        }

        if (consumed > 0)
            s->pc += consumed;
        s->pc &= 0xFFFF;  /* the C54x PC is 16 bits (23 with XPC, but it wraps at 16) */
        /* consumed == 0 means PC was set by branch */

        /* SP-CORRUPT watchpoint: which instruction takes SP out of the valid
         * stack range [0x5900, 0x5c00]? One observed derail is 0xa58d leaving
         * SP = 0xc905 after a POPD. Logs the first inside -> outside
         * transition, whose pc/op is the culprit. */
        {
            static uint16_t g_wp_prev_sp = 0x5ac8;
            if ((s->sp < 0x5900 || s->sp > 0x5c00) &&
                (g_wp_prev_sp >= 0x5900 && g_wp_prev_sp <= 0x5c00)) {
                static unsigned wpn = 0;
                if (wpn++ < 20)
                    fprintf(stderr, "[c54x] SP-CORRUPT pc=0x%04x op=0x%04x sp 0x%04x -> 0x%04x insn=%u\n",
                            exec_pc, exec_op, g_wp_prev_sp, s->sp, s->insn_count);
            }
            g_wp_prev_sp = s->sp;
        }

        /* On-chip TIMER0 tick: the missing clock. Go-live arms IMR bit 4
         * (TINT, vec 20) and waits for the timer, but the TIM/PRD/TCR
         * registers alone never decrement, so TINT never fires. TIM is ticked
         * here (through the TDDR prescaler) once per instruction; on underflow
         * it reloads from PRD and fires TINT. Gate CALYPSO_DSP_TIMER_OFF for
         * A/B, on by default. The firmware starts the timer (clearing TSS) and
         * configures PRD/TDDR. */
        {
            {
                /* @BEQUILLE - TINT0_PERINSN  (CALYPSO_TINT0_PERINSN, EXISTS, default OFF)
                 *   masks   : the absence of a DSP time base. Fires TINT every 2000
                 *             insns, unrelated to the TDMA cadence.
                 *   remove  : superseded by the faithful TIMER0 tick just below.
                 *   TRAP    : this code IS executed; only the absence of the environment
                 *             variable keeps it quiet.
                 */
                if (getenv("CALYPSO_TINT0_PERINSN")) {
                    static unsigned _t0c = 0;
                    if (++_t0c >= 2000) { _t0c = 0; c54x_fire_tint(s); }
                }
            }
            static int _tmr = -1;
            if (_tmr < 0) _tmr = getenv("CALYPSO_DSP_TIMER_OFF") ? 0 : 1;
            /* @BEQUILLE - TINT0_MASTER  (CALYPSO_TINT0_MASTER, EXISTS, default OFF
             *              outside the WIRE profile - calypso.env/wire.env only set it
             *              under CALYPSO_WIRE=1)
             *   masks   : the ROM's TIMER0 configuration (TCR/PRD). The firmware stops the
             *             timer (TSS=1) in an init that never runs, so PRD is forced to
             *             0xFFFF and TIM is ticked DESPITE TSS. On real hardware TINT0 is
             *             the TDMA master clock.
             *   remove  : when the ROM's TIMER0 init sequence executes (TCR programmed,
             *             TSS=0).
             */
            static int _t0master = -1;
            if (_t0master < 0) _t0master = calypso_gate("CALYPSO_TINT0_MASTER", 0);
            /* Under TINT0_MASTER the ROM is modelled as having configured and
             * started the timer, so TIM ticks despite TSS. With PRD left at its
             * reset value the underflow comes every ~65536 insns, which is
             * about one TDMA frame at 13 MHz. The underflow fires TINT through
             * c54x_fire_tint(), which HONOURS the IMR - nothing is forced. */
            if (_t0master && s->data[PRD_ADDR] == 0) s->data[PRD_ADDR] = 0xFFFF;
            if (_tmr && (_t0master || !(s->data[TCR_ADDR] & TCR_TSS))) {
                if (s->timer_psc == 0) {
                    s->timer_psc = s->data[TCR_ADDR] & TCR_TDDR_MASK;
                    if (s->data[TIM_ADDR] == 0) {
                        s->data[TIM_ADDR] = s->data[PRD_ADDR];
                        static unsigned _tn = 0;
                        if (_tn++ < 8)
                            fprintf(stderr, "[c54x] DSP-TIMER TINT fire "
                                    "PRD=0x%04x TDDR=%u IMR=0x%04x INTM=%d insn=%u\n",
                                    s->data[PRD_ADDR], (unsigned)(s->data[TCR_ADDR] & TCR_TDDR_MASK),
                                    s->imr, (s->st1 & ST1_INTM) ? 1 : 0, s->insn_count);
                        c54x_fire_tint(s);   /* SPRU131 5.1: TINT = bit 3 / vec 19 */
                    } else {
                        s->data[TIM_ADDR]--;
                    }
                } else {
                    s->timer_psc--;
                }
            }
        }

        /* BRANCH-TRACE: consumed == 0 means the PC was set by a TAKEN branch,
         * call or return, since a sequential instruction goes through
         * `s->pc += consumed` above. Logs EVERY taken control transfer in the
         * boot/init window (insn < 6000), filtering out the PC=0 storm, with
         * the site, the branch opcode, the target and the condition state (TC,
         * A == 0, A, ARx, IMR, INTM). The point is to see WHICH conditional
         * branch diverts the firmware from installing vectors and enabling
         * interrupts into the idle guard. It discriminates (a) a taken BC/BANZ
         * whose target short-circuits the IPTR write, which is then never
         * reached, from (b) the code reaching that write with no effect, which
         * would be an MMR bug. 700 lines. */
        if (consumed == 0 && exec_pc != 0 && s->pc != 0 && s->insn_count < 6000) {
            static unsigned bt = 0;
            if (bt++ < 700) {
                uint64_t aa = s->a & 0xFFFFFFFFFFULL;
                fprintf(stderr, "[c54x] BRANCH-TRACE #%u site=0x%04x op=0x%04x -> "
                        "tgt=0x%04x TC=%d Az=%d A=0x%010llx AR1=%04x AR3=%04x "
                        "AR5=%04x SP=%04x IMR=%04x INTM=%d insn=%u\n",
                        bt, exec_pc, exec_op, s->pc,
                        (s->st0 & ST0_TC) ? 1 : 0, (aa == 0) ? 1 : 0,
                        (unsigned long long)aa, s->ar[1], s->ar[3], s->ar[5],
                        s->sp, s->imr, (s->st1 & ST1_INTM) ? 1 : 0, s->insn_count);
            }
        }

        /* Delayed-branch slot countdown, counted in WORDS, not instructions.
         * A delayed branch (RCD, CALLD, RETD, BD, CCD) sets delayed_pc and
         * delay_slots = 2; the following instructions execute as pipeline
         * slots, and the branch commits by forcing PC to delayed_pc once the
         * two words are done.
         *
         * Per SPRU172C the C54x ALWAYS has 2 words of delay: one 2-word
         * instruction OR two 1-word instructions. Decrementing once per
         * instruction executes only ONE delay-slot instruction, which is right
         * for a single 2-word slot (STM #k) but SKIPS the second of two 1-word
         * slots. When that second instruction is a PSHM/PSHD (PROM0 sites
         * 0xb53a, 0xc9a2, 0xcaab, the power-scan code the mobile runs during
         * cell search), the push is lost: ~58 words of cumulative over-pop,
         * POPM ST0 at 0x94f3 picks up the orphan 0x80fd, DP becomes 0x0fd, the
         * dispatcher at 0x8341 reads a garbage LUT, CALAD 0x70c3 is a
         * self-CALA, 0x70c4 (28868) lands in d_fb_det/a_pm, and rxlev/TOA are
         * poisoned into NO_CELL_FOUND. See doc/SP_CATASTROPHE_70c4_SEQUENCE.
         *
         * So decrement by the number of WORDS executed (consumed), and do NOT
         * count the arming iteration (ds_before == 0 is the branch itself; the
         * delay starts with the next instruction). */
        if (s->delay_slots > 0) {
            if (ds_before == 0) {
                /* The delayed branch's own iteration: decrement nothing;
                 * delay_slots (= 2) counts WORDS. */
            } else {
                int wexec = (consumed > 0) ? consumed : 1;
                if (s->delay_slots > wexec) s->delay_slots -= wexec;
                else                        s->delay_slots = 0;
                if (s->delay_slots == 0)
                    s->pc = s->delayed_pc;
            }
        }


        /* === RPTB (block repeat) end-of-body check ===
         * Must run AFTER PC advance and delayed-branch settle so the
         * redirect to RSA is the final word on s->pc for this iteration.
         * Triggers when PC has overshot REA (= reached REA+1 or beyond,
         * accounting for 2-word instructions at the body's tail). Skip
         * during RPT (single-instruction repeat has priority). */
        if (s->rptb_active && !s->rpt_active && s->pc >= s->rea + 1) {
            static int rptb_log = 0;
            if (rptb_log < 20) {
                C54_LOG("RPTB redirect PC=0x%04x→RSA=0x%04x REA=0x%04x BRC=%d",
                        s->pc, s->rsa, s->rea, s->brc);
                rptb_log++;
            }
            if (s->brc > 0) {
                s->brc--;
                s->pc = s->rsa;
            } else {
                s->rptb_active = false;
                { static int _re=0;
                  if (_re<50) {
                    C54_LOG("RPTB EXIT PC=0x%04x RSA=0x%04x REA=0x%04x insn=%u SP=0x%04x",
                            s->pc, s->rsa, s->rea, s->insn_count, s->sp);
                    _re++;
                  }
                }
                s->st1 &= ~ST1_BRAF;
            }
        }

        s->cycles++;
        s->insn_count++;

        executed++;

        /* SP-LEDGER: periodic dump to check that net_words tends to 0 over a
         * long run - the push/pop balance metric. About one compare per
         * instruction. */
        if (s->insn_count - g_sp_ledger.last_dump_insn >= 20000000u) {
            g_sp_ledger.last_dump_insn = s->insn_count;
            fprintf(stderr,
                "[c54x] SP-LEDGER insn=%u PC=0x%04x SP=0x%04x net_words=%lld pushes=%llu pops=%llu irq=%llu\n",
                s->insn_count, s->pc, s->sp, (long long)g_sp_ledger.net_words,
                (unsigned long long)g_sp_ledger.sp_pushes,
                (unsigned long long)g_sp_ledger.sp_pops,
                (unsigned long long)g_sp_ledger.irq_entries);
            fflush(stderr);
        }

        /* Yield throttle (CALYPSO_DSP_YIELD=N). The c54x runs synchronously
         * inside tdma_tick on the main thread; every N instructions c54x_run
         * returns so the main loop can pump I/O (osmocon) and then deliver
         * interrupts to the DSP.
         *   small N = frequent yields = fast osmocon, slower DSP
         *   large N or 0 = rare yields or off, the DSP keeps the whole budget
         *
         * The break MUST happen at a clean boundary: after `s->pc += consumed`
         * AND after the delay slots have committed (delay_slots == 0).
         * Otherwise (a) the current instruction is re-executed on re-entry,
         * double-popping, and (b) an interrupt delivered by the main loop would
         * land in the middle of a RETD/RCD delay slot and corrupt the deferred
         * return. Both lead to SP over-pop, garbage DP and the self-CALA at
         * 0x70c3. */
        {
            static int dsp_yield = -1;
            if (dsp_yield < 0) {
                const char *e = getenv("CALYPSO_DSP_YIELD");
                /* Default 32768 (2^15): the empirically tuned DSP / osmocon
                 * interrupt cadence, on by default. Only an explicit
                 * CALYPSO_DSP_YIELD=0 turns it off. */
                dsp_yield = (e && *e) ? atoi(e) : 32768;
                if (dsp_yield < 0) dsp_yield = 0;
                fprintf(stderr, "[c54x] CALYPSO_DSP_YIELD = %d insn/yield %s\n",
                        dsp_yield, dsp_yield ? "(variateur ON)" : "(OFF, legacy)");
            }
            /* Yield only at an INTERRUPTIBLE point. The yield hands control to
             * the main loop, which delivers the interrupt to the DSP, and on a
             * real C54x an interrupt is only taken with INTM=0, outside a
             * critical section. Breaking on a plain instruction counter lands
             * in the middle of a dispatch sequence (INTM=1, DP inherited before
             * the LDP), and the interrupt on resume then corrupts DP/ST0,
             * giving a CALAD into the LUT (the 0x9207 wedge) or a self-CALA.
             *
             * The guards are the four non-interruptible states of the C54x:
             *   delay_slots == 0 : not inside a delayed branch (RETD/RCD/CALLD/BD)
             *   !rpt_active      : not inside an RPT - a single repeat is NOT
             *                      interruptible on hardware until RC is
             *                      exhausted, whereas RPTB is
             *   INTM == 0        : interruptible, outside a critical section
             *   plus the break happening after the PC and delay commit, so
             *   never mid-instruction
             * The hard cap at 4x forces a yield when INTM stays 1 abnormally
             * long, to avoid starving the main loop; that case is traced
             * below. */
            if (dsp_yield > 0 && s->delay_slots == 0 && !s->rpt_active &&
                ((executed >= (unsigned)dsp_yield && !(s->st1 & ST1_INTM)) ||
                 executed >= (unsigned)dsp_yield * 4u)) {
                /* Evidence for the gate: log the first breaks and every
                 * cap-forced break (INTM=1, tolerated but illegal). Never
                 * seeing a cap-forced break over N runs while 0x9207
                 * disappears proves the gate rather than merely observing
                 * it. */
                if (calypso_debug_enabled("YIELD-BREAK")) {
                    static unsigned yb = 0;
                    int forced = (s->st1 & ST1_INTM) ? 1 : 0;
                    if (yb < 40 || forced)
                        fprintf(stderr, "[c54x] YIELD-BREAK #%u INTM=%d delay=%d rpt=%d pc=0x%04x exec=%u %s\n",
                                yb, forced, s->delay_slots, s->rpt_active, s->pc, executed,
                                forced ? "*** CAP-FORCED (INTM=1 illegal) ***" : "(safe)");
                    yb++;
                }
                break;   /* clean, interruptible boundary: the main loop serves I/O and interrupts */
            }
        }
    }
    return executed;
}

/* ================================================================
 * ROM loader
 * ================================================================ */



/* ================================================================
 * Init / Reset / Interrupts
 * ================================================================ */

C54xState *c54x_init(void)
{
    C54xState *s = calloc(1, sizeof(C54xState));
    if (!s) return NULL;
    return s;
}

void c54x_set_api_ram(C54xState *s, uint16_t *api_ram)
{
    s->api_ram = api_ram;
}

void c54x_set_initial_pc(C54xState *s, uint32_t pc)
{
    s->pc = pc;
    s->blob_loaded = true;
    C54_LOG("set_initial_pc: PC=0x%05x (blob_loaded=1)", pc);
}

/* ==========================================================================
 * c54x early boot. Only active under CALYPSO_DSP_RUN_C54X; it forces no
 * mailbox value, only WHEN the DSP boots.
 *
 * Ordering matters. The ARM posts its bootloader command (data[0x0fff] = cmd
 * 2 or 4, data[0x0ffe] = entry) as early as fn=0, +0.073 s. If the DSP boots
 * after that, its IDLE init at 0xb419 (`ST #1, *0xfff`) OVERWRITES that
 * 0x0002 and it spins for ever in 0xb41c. Booting here, at machine init and
 * therefore BEFORE the ARM vCPU runs, the DSP sets its IDLE and parks in
 * 0xb41c BEFORE the ARM write: 0xb419 does not run again (the PC persists
 * across wake-ups), the command survives, and the first wake-up consumes it.
 * ========================================================================== */
bool g_c54x_early_booted;

void c54x_early_boot(C54xState *s)
{
    static int run_c54x = -1;
    if (run_c54x < 0) {
        const char *e = getenv("CALYPSO_DSP_RUN_C54X");
        run_c54x = (e && *e == '1') ? 1 : 0;
    }
    if (!s || !run_c54x) {
        return;
    }

    uint16_t pc0 = s->pc;
    s->running = true;
    c54x_run(s, 2000);   /* reset (0xff80) -> 0xb419 (sets IDLE) -> park at 0xb41c */
    if (s->pc >= 0xb41c && s->pc <= 0xb428) {
        g_c54x_early_booted = true;   /* suppresses the re-reset in calypso_trx.c */
        fprintf(stderr, "[c54x-earlyboot] PARK pc=0x%04x (de 0x%04x) insn=%u "
                "data[0x0fff]=0x%04x data[0x0ffe]=0x%04x (attendu IDLE 0x0001)\n",
                s->pc, pc0, s->insn_count, s->data[0x0fff], s->data[0x0ffe]);
    } else {
        fprintf(stderr, "[c54x-earlyboot] WARN pas parque pc=0x%04x insn=%u "
                "-> execution continue\n", s->pc, s->insn_count);
    }
}

bool c54x_early_booted(void)
{
    return g_c54x_early_booted;
}

/* Current DSP mission, read from the API RAM:
 *   d_task_md, page 0 = data[0x0804], page 1 = data[0x0818]. */
uint16_t c54x_task_md(C54xState *s)
{
    if (!s || !s->data) {
        return 0;
    }
    uint16_t md = s->data[0x0804];
    return md ? md : s->data[0x0818];
}

int c54x_load_blob_daram(C54xState *s, const char *path, uint16_t daram_addr)
{
    FILE *f = fopen(path, "rb");
    if (!f) {
        C54_LOG("load_blob_daram: cannot open '%s'", path);
        return -1;
    }
    int words = 0;
    uint32_t addr = daram_addr;
    uint8_t buf[2];
    while (addr < C54X_DATA_SIZE && fread(buf, 1, 2, f) == 2) {
        uint16_t w = buf[0] | ((uint16_t)buf[1] << 8);
        s->data[addr] = w;
        if (s->api_ram &&
            addr >= C54X_API_BASE && addr < C54X_API_BASE + C54X_API_SIZE)
            s->api_ram[addr - C54X_API_BASE] = w;
        addr++;
        words++;
    }
    fclose(f);
    C54_LOG("load_blob_daram: %d words at DARAM[0x%04x..0x%04x] from %s",
            words, daram_addr, addr - 1, path);
    return words;
}

int c54x_load_section(C54xState *s, const char *path,
                      uint32_t start_addr, bool is_program)
{
    FILE *f = fopen(path, "rb");
    if (!f) {
        C54_LOG("load_section: cannot open '%s'", path);
        return -1;
    }
    uint16_t *mem = is_program ? s->prog : s->data;
    uint32_t limit = is_program ? C54X_PROG_SIZE : C54X_DATA_SIZE;
    int words = 0;
    uint32_t addr = start_addr;
    uint8_t buf[2];
    while (addr < limit && fread(buf, 1, 2, f) == 2) {
        uint16_t w = buf[0] | ((uint16_t)buf[1] << 8);
        mem[addr] = w;
        if (!is_program && s->api_ram &&
            addr >= C54X_API_BASE && addr < C54X_API_BASE + C54X_API_SIZE)
            s->api_ram[addr - C54X_API_BASE] = w;
        addr++;
        words++;
    }
    fclose(f);
    C54_LOG("load_section: %d words at %s[0x%05x..0x%05x] from %s",
            words, is_program ? "prog" : "data",
            start_addr, addr - 1, path);
    return words;
}

int c54x_load_registers(C54xState *s, const char *path)
{
    FILE *f = fopen(path, "rb");
    if (!f) {
        C54_LOG("load_registers: cannot open '%s'", path);
        return -1;
    }
    int words = 0;
    uint8_t buf[2];
    /* First 0x20 words = MMR page → reset-override buffer (applied in
     * c54x_reset). Remaining words (0x20..0x5F = low scratch DARAM) →
     * data[] directly, like a section load. */
    while (words < C54X_DATA_SIZE && fread(buf, 1, 2, f) == 2) {
        uint16_t w = buf[0] | ((uint16_t)buf[1] << 8);
        if (words < 0x20)
            s->reg_init[words] = w;
        else
            s->data[words] = w;
        words++;
    }
    fclose(f);
    if (words >= 0x20)
        s->reg_init_valid = true;
    C54_LOG("load_registers: %d words from %s (MMR reset override %s)",
            words, path, s->reg_init_valid ? "ON" : "OFF (file too short)");
    return words;
}

void c54x_reset(C54xState *s)
{
    g_boot_trace = 50;
    s->blob_loaded = false;  /* explicit reset exits dsp-blob fixture mode */
    s->a = 0; s->b = 0;   /* mode c54x is a clean datasheet reset: A = B = 0.
                           * The snapshot has BL=0x60, applied in bin mode only. */
    /* Register reset state. The hard-coded values below are mode "c54x": a
     * clean datasheet reset, with the critical fields aligned on the silicon
     * snapshot and the benign or garbage ones at 0. The three modes selected
     * further down are orthogonal:
     *   c54x   = these hard-coded values only, independent of any file
     *   bin    = calypso_dsp.Registers.bin overrides everything, verbatim
     *            (the default)
     *   hybrid = bin for the operational registers, clean values forced for
     *            the critical ones (IFR=0, BRC/RSA/REA=0)
     *
     * The .bin is a mid-execution snapshot, taken after the bootloader
     * handshake. Field by field, judged on whether the value makes sense AT
     * RESET:
     *
     *   MMR        value                 class       note
     *   IMR  0x00  0x52FD                critical    IRQ mask, identical in 3 dumps
     *   IFR  0x01  0x0008                benign      INT3 pending; masked by INTM=1
     *   ST0  0x06  0x181F                critical    DP=0x1F
     *   ST1  0x07  0x2900                critical    INTM/SXM/XF
     *   A    08-0a 0x000000              ok
     *   B    0b-0d 0x000060              benign      BL=0x60, reloaded before use
     *   T    0x0E  0x0000                ok
     *   TRN  0x0F  0xFF75                benign      Viterbi, neutral at reset
     *   AR0  0x10  0x5AAD                load-bearing, see below
     *   AR1-5 11-15 invariant            critical    identical in 3 dumps (API RAM...)
     *   AR6  0x16  0xBAE6                benign      reloaded before use
     *   AR7  0x17  0x1E44                benign      likewise
     *   SP   0x18  0x1100                critical    post-handshake stack
     *   BK   0x19  0xFFF6                critical    circular buffer
     *   BRC  0x1A  0x8FD7                garbage     leftover of an RPTB in flight
     *   RSA  0x1B  0xD9EC                garbage     rptb_active is false at reset
     *   REA  0x1C  0xBBEF                garbage
     *   PMST 0x1D  0xFFA8                critical    IPTR=0x1FF, MP_MC, OVLY, DROM
     *   XPC  0x1E  0x0000                never overridden (runtime page, vector fetch)
     *
     * The critical fields (SP/ST0/ST1/PMST/IMR/AR1-5/BK) are identical between
     * the snapshot and the hard-coded values and are what drives the reset.
     *
     * AR values follow the silicon spec (doc/datasheets/README.md section 3),
     * cross-checked against 3 ROM dumps (3311/3416/3606) plus the local
     * osmocom dump:
     *   AR1=0x005F, AR2=0x0813, AR3=0x0014, AR4=0x0003, AR5=0x0014 (invariant)
     *   BK=0xFFF6
     *   AR6, AR7: no documented invariant, left at 0
     * A blanket memset to 0 is wrong: with AR2=0, the STL A,*AR2 at PC=0x9ac0
     * writes mem[0x00] = IMR, clearing it, which masks every FRAME/BRINT0
     * interrupt and leaves the DSP stuck in df9x. */
    memset(s->ar, 0, sizeof(s->ar));
    s->ar[0] = 0x5AAD;  /* silicon value. AR0 is read before being written at
                         * insn=1 (PC=0xb410, ORM data[*AR0]), shown by the
                         * AR-FIRSTUSE probe, so the reset value is
                         * load-bearing: the "neutral" 0xFF75 from the local
                         * dump made c54x and bin modes diverge on the very
                         * first boot instruction. */
    s->ar[1] = 0x005F;
    s->ar[2] = 0x0813;  /* API RAM pointer; 0 here clobbers the IMR */
    s->ar[3] = 0x0014;
    s->ar[4] = 0x0003;
    s->ar[5] = 0x0014;
    s->t = 0; s->trn = 0;   /* TRN=0; the snapshot has 0xff75, neutral at reset */
    s->sp = 0x1100; s->bk = 0xFFF6;  /* SP and BK aligned with silicon: 3 ROM dumps
                                 * (3311/3416/3606) plus the local one all show
                                 * SP=0x1100 after the bootloader handshake. The
                                 * firmware repoints it to its own stack (0x5AC8
                                 * observed) through its init sequence, as on real
                                 * silicon. Starting at 0x5AC8 short-circuits that
                                 * re-init and is the suspected root of the AR5/SP
                                 * overlap clobber at mem[0x3fbe].
                                 * See doc/datasheets/README.md sections 3-4. */
    /* BRC/RSA/REA = 0, the clean datasheet reset. The snapshot captures
     * leftovers of an RPTB in flight (BRC=0x8fd7 RSA=0xd9ec REA=0xbbef), which
     * are meaningless at reset and applied in bin mode only. rptb_active is
     * false (set below), so these registers are not consulted until an RPTB
     * reloads them. */
    s->brc = 0; s->rsa = 0; s->rea = 0;
    /* MMR reset values aligned with Calypso silicon (3 FreeCalypso ROM dumps
     * plus the local one). See doc/datasheets/README.md section 3. */
    s->st0  = 0x181F;                              /* DP=0x01F per silicon */
    s->st1  = ST1_INTM | ST1_SXM | ST1_XF;         /* 0x2900: INTM=1, SXM=1, XF=1 */
    s->pmst = 0xFFA8;                              /* IPTR=0x1FF, MP_MC=1, OVLY=1, DROM=1 */
    s->imr = 0x52FD;                               /* aligned with the local osmocom dump,
                                                    * post bootloader handshake. IMR=0 masks
                                                    * every interrupt: IRQ #2..#10 were seen
                                                    * with INTM=1 IMR=0x0000 IFR=0x28, so no
                                                    * handler ran, no dispatcher flag was
                                                    * written, and the DSP looped in df9x. */
    s->ifr = 0;        /* clean datasheet reset; the snapshot has 0x0008 (INT3
                        * pending), applied in bin mode only, and neutral
                        * anyway since INTM=1 masks it. */
    s->xpc = 0;
    /* Register reset mode selector, env CALYPSO_DSP_REG_MODE:
     *   "c54x"   -> the hard-coded values above ONLY; a loaded .bin is ignored
     *   "bin"    -> calypso_dsp.Registers.bin overrides everything, verbatim
     *   "hybrid" -> the .bin for the validated operational registers, but the
     *               clean values for the fields the .bin gets wrong at reset:
     *                 IFR         : leftover pending interrupt -> 0
     *                 BRC/RSA/REA : garbage from an RPTB in flight -> 0
     * The default is "bin" when a .bin is loaded, otherwise necessarily the
     * hard-coded values. Read once; reset is called twice (boot and
     * DSP_DL_STATUS_READY). */
    {
        static int reg_mode = -1;  /* 0=c54x 1=bin 2=hybrid */
        if (reg_mode < 0) {
            const char *e = getenv("CALYPSO_DSP_REG_MODE");
            if      (e && !strcasecmp(e, "c54x"))   reg_mode = 0;
            else if (e && !strcasecmp(e, "hybrid")) reg_mode = 2;
            else                                    reg_mode = 1; /* "bin"/default */
            C54_LOG("reset: CALYPSO_DSP_REG_MODE=%s → mode=%s",
                    e ? e : "(unset)",
                    reg_mode == 0 ? "c54x(hardcode)" :
                    reg_mode == 2 ? "hybrid" : "bin");
        }
        if (s->reg_init_valid && reg_mode != 0) {
            const uint16_t *r = s->reg_init;
            /* Operational registers, common to bin and hybrid */
            s->imr  = r[0x00];
            s->st0  = r[0x06];
            s->st1  = r[0x07];
            s->a    = ((int64_t)(r[0x0a] & 0xFF) << 32) |
                      ((uint32_t)r[0x09] << 16) | r[0x08];
            s->b    = ((int64_t)(r[0x0d] & 0xFF) << 32) |
                      ((uint32_t)r[0x0c] << 16) | r[0x0b];
            s->t    = r[0x0e];
            s->trn  = r[0x0f];
            for (int i = 1; i < 8; i++)   /* AR1..AR7; AR0 is handled below */
                s->ar[i] = r[0x10 + i];
            s->sp   = r[0x18];
            s->bk   = r[0x19];
            s->pmst = r[0x1d];
            if (reg_mode == 1) {
                /* Pure bin: the .bin verbatim, nothing forced. The snapshot
                 * IS the silicon state, so it is taken whole - forcing
                 * IFR/BRC/RSA/REA to 0 here would throw away real silicon
                 * state, INT3 pending in particular. rptb_active is false (set
                 * below), so BRC/RSA/REA are not consulted until an RPTB
                 * reloads them. */
                s->ifr   = r[0x01];
                s->ar[0] = r[0x10];
                s->brc   = r[0x1a];
                s->rsa   = r[0x1b];
                s->rea   = r[0x1c];
            } else { /* hybrid: operational registers from the .bin, critical
                      * fields forced to the clean datasheet values. */
                s->ifr   = 0x0000;   /* no interrupt pending at reset */
                s->ar[0] = r[0x10];  /* AR0 from the silicon snapshot, not a
                                      * hard-coded value: it is read before
                                      * being written at insn=1. */
                s->brc   = 0x0000;
                s->rsa   = 0x0000;
                s->rea   = 0x0000;
            }
            /* XPC is never overridden: it is a runtime page register, and the
             * reset vector fetch at IPTR*0x80 must come from page 0. */
            C54_LOG("reset: dsp-registers %s applied (SP=0x%04x PMST=0x%04x "
                    "ST0=0x%04x ST1=0x%04x IMR=0x%04x IFR=0x%04x AR0=0x%04x "
                    "BRC=0x%04x RSA=0x%04x REA=0x%04x)",
                    reg_mode == 2 ? "HYBRID" : "BIN",
                    s->sp, s->pmst, s->st0, s->st1, s->imr, s->ifr, s->ar[0],
                    s->brc, s->rsa, s->rea);
        } else {
            C54_LOG("reset: registres = hardcode C (mode c54x ou pas de .bin) "
                    "SP=0x%04x PMST=0x%04x IMR=0x%04x IFR=0x%04x AR0=0x%04x",
                    s->sp, s->pmst, s->imr, s->ifr, s->ar[0]);
        }
    }
    s->timer_psc = 0;
    s->data[TCR_ADDR] = TCR_TSS;  /* Timer stopped at reset (TSS=1) per HW spec */
    s->data[TIM_ADDR] = 0xFFFF;   /* TIM = max at reset */
    s->data[PRD_ADDR] = 0xFFFF;   /* PRD = max at reset */
    s->rpt_active = false;
    s->rptb_active = false; { static int _re=0; if (_re<50) { C54_LOG("RPTB EXIT PC=0x%04x RSA=0x%04x REA=0x%04x insn=%u SP=0x%04x", s->pc, s->rsa, s->rea, s->insn_count, s->sp); _re++; } }
    s->idle = false;
    s->running = true;
    s->cycles = 0;
    s->insn_count = 0;
    s->unimpl_count = 0;

    /* Boot ROM MVPD: copy PROM0 code to DARAM overlay.
     * On real Calypso, the internal boot ROM copies PROM0[0x7080..0x9FFF]
     * to DARAM data[0x0080..0x27FF] before jumping to user code.
     * This populates the DARAM code overlay that the DSP executes with OVLY=1.
     *
     * On real silicon, DARAM and API RAM share one physical memory in the
     * range 0x0800-0x27FF (DSP-words). Mirror the copy into api_ram so the
     * ARM-side view matches the DSP-side view from boot — without this
     * mirror, every ARM read into the overlay zone returns 0 while the
     * DSP executes the copied code, which silently splits the two views. */
    for (int i = 0; i < 0x2780; i++) {
        uint16_t addr = 0x0080 + i;
        uint16_t val = s->prog[0x7080 + i];
        s->data[addr] = val;
        if (s->api_ram &&
            addr >= C54X_API_BASE && addr < C54X_API_BASE + C54X_API_SIZE)
            s->api_ram[addr - C54X_API_BASE] = val;
    }

    /* prog[0xFF80..0xFF83] is left as the legitimate PROM1 mirror (0x56d0,
     * 0x9631, 0xf820, 0xff89 from the dump). The boot-init redirect is applied
     * at runtime and ONLY while SP still holds the silicon reset value 0x1100
     * - see the c54x_run main loop.
     *
     * A static override of prog[0xFF80] would intercept the silicon reset AND
     * every sequential firmware walk into 0xff80: the ROM has an ordinary
     * subroutine whose RET at 0xff79 pops 0xff7a, and the POPM/STM epilogue
     * walks through 0xff80. Each such walk would become a soft reset, SP would
     * be set to 0x5AC8, the state would derail and the DSP would sit in a
     * boot-reset cycle with FB never stabilising past the first 4 s. */

    /* Boot ROM stubs at 0x0000-0x007F. They prevent a stack runaway when a
     * computed call lands in the stub area, at no cost: reverting them to NOPs
     * gives an identical PC histogram and the same IMR behaviour.
     *
     * Per slot:
     *   - 0x0000/0x0001   : RET (0xFC00), pops ret_pc, BALANCED return
     *   - 0x02..0x7F      : FRET (0xF4E4), return-from-far
     *
     * RET at 0x0000 rather than IDLE or LDMM, because the firmware does near
     * CALAs to 0x0000 with A=0 (the null/default handler path; for instance
     * the LDU at 0xfa7e reads a table pointer of 0, then CALA A=0). A near
     * CALA pushes one word, RET pops one, so the return is balanced and the
     * boot continues.
     *   - IDLE does not return: it halts, slides 0x0000 -> 0x0002 into the
     *     FRET loop, and leaks SP (0x1106 -> 0x4d75).
     *   - LDMM SP,B + RET returns but sets SP = B, so B=0 gives SP=0 and pops
     *     garbage.
     * The rest stays FRET, so far calls (FCALA pushes 2) that land in the zone
     * are balanced by a 2-word pop. The firmware idles at its real point, the
     * IDLE of the TDMA slot table (doc/DSP_ROM_MAP.md). */
    for (int i = 0; i < 0x80; i++)
        s->prog[i] = 0xF4E4;  /* FRET: return-from-far (far call into a stub) */
    s->prog[0x0000] = 0xFC00;  /* RET: pops ret_pc, balances the near CALA to 0 */
    s->prog[0x0001] = 0xFC00;  /* RET: likewise */

    /* Reset vector: IPTR * 0x80 */
    uint16_t iptr = (s->pmst >> PMST_IPTR_SHIFT) & 0x1FF;
    s->pc = iptr * 0x80;  /* 0xFF80 for default PMST */

    C54_LOG("Reset: PC=0x%04x PMST=0x%04x SP=0x%04x prog[PC]=0x%04x",
            s->pc, s->pmst, s->sp, s->prog[s->pc]);

    /* Build identity dump: the silicon-aligned reset values this binary
     * actually uses. If they change, firmware behaviour changes, so a report
     * can be attributed to a code state by reading qemu.log. */
    C54_LOG("BUILD-IDENT silicon-reset: SP=0x%04x BK=0x%04x IMR=0x%04x "
            "ST0=0x%04x ST1=0x%04x PMST=0x%04x",
            s->sp, s->bk, s->imr, s->st0, s->st1, s->pmst);
    C54_LOG("BUILD-IDENT silicon-AR: AR0=0x%04x AR1=0x%04x AR2=0x%04x AR3=0x%04x "
            "AR4=0x%04x AR5=0x%04x AR6=0x%04x AR7=0x%04x",
            s->ar[0], s->ar[1], s->ar[2], s->ar[3],
            s->ar[4], s->ar[5], s->ar[6], s->ar[7]);
    /* Decoder fix flags: removing these fixes from the source changes or
     * removes this line, which identifies the binary that produced a run. */
    C54_LOG("BUILD-IDENT decoder-fixes: F1xx-FIRS-catch=REMOVED "
            "L3609-src-dst=FIXED F-AUDIT-v5=max-min-cmpl-rnd-roltc-fixed "
            "F2xx-ALU-block=ADDED-2026-05-25-night "
            "F3xx-INTR-mis-REMOVED-ADD-SUB-LD-ADDED "
            "PROBE-HIGHVEC-REGIME=2026-06-23 "
            "SQURA-0x38-FIX=2026-06-23 "
            "2026-05-25");

}

int g_c54x_int3_src = 0;   /* 1=trx 2=bsp 3=shunt: INT3 source, diagnostics only */


void c54x_interrupt_ex(C54xState *s, int vec, int imr_bit)
{
    if (vec < 0 || vec >= 32) return;
    if (imr_bit < 0 || imr_bit >= 16) return;
    s->ifr |= (1 << imr_bit);
    if (imr_bit == 12 && frame_it_level_on()) g_frame_it_level = true;  /* arm the frame LEVEL hold */

    /* Frame interrupt rate probe: each dispatch of the frame interrupt with
     * the insn delta since the previous one. A delta near 130 means
     * over-delivery (one per BSP burst), which drowns the DSP; near 256000 is
     * one per frame, which is correct. The frame interrupt is vec 28, not
     * vec 19 (19 is TINT). */
    if (vec == C54X_IT_TPU_FRAME_VEC) {
        static uint64_t last_i3 = 0;
        static unsigned i3n = 0;
        bool post_fb = s->insn_count > 160000u;   /* ~fn 1206: the FB order has been delivered */
        if (i3n < 40 || (post_fb && i3n < 100)) {
            i3n++;
            uint16_t real_page = s->api_ram ? s->api_ram[0x08E2 - 0x0800]
                                            : s->data[0x08E2];
            fprintf(stderr, "[c54x] INT3-RATE #%u src=%d insn=%u delta=%lld idle=%d "
                    "REAL_page(08D4)=0x%04x daram584=0x%04x task_md(058a)=0x%04x\n",
                    i3n, g_c54x_int3_src, s->insn_count,
                    (long long)((uint64_t)s->insn_count - last_i3), s->idle,
                    real_page, s->data[0x0584], s->data[0x058a]);
        }
        last_i3 = s->insn_count;
    }

    bool unmasked = (s->imr & (1 << imr_bit)) != 0;
    /* @BEQUILLE - FIX_BRINT0_UNMASK  (CALYPSO_FIXES=FIX_BRINT0_UNMASK, default OFF)
     *   masks   : the absence of any native arming of IMR bit 5 (BRINT0 / vec 21).
     *   remove  : as soon as the real arming branch is implemented, OR at once if
     *             the test shows the root cause is upstream (vector 21 installed as
     *             zero).
     *   WARNING : DIAGNOSTIC, to be removed, never to be confirmed. It does not
     *             count as a fix.
     * It answers one question: is BRINT0 the LAST lock or only the NEXT one? If
     * unmasking it artificially gets the DSP into the demodulator
     * (CALYPSO_WATCH_9F00_RD goes from 0 to non-zero), it was the last; otherwise
     * the root cause is upstream, the candidate being vector 21 installed as
     * zero. */
    if (imr_bit == 5 && !unmasked && calypso_fix_enabled("FIX_BRINT0_UNMASK")) {
        static unsigned _bu = 0;
        if (_bu++ < 5)
            fprintf(stderr, "[c54x] FIX_BRINT0_UNMASK : bit 5 demasque ARTIFICIELLEMENT "
                    "(IMR=0x%04x, IFR=0x%04x, PC=0x%04x) — diagnostic, pas un correctif\n",
                    s->imr, s->ifr, s->pc);
        unmasked = true;
    }

    /* SYNC-DISPATCH-PROBE (unconditional, capped). c54x_interrupt_ex dispatches
     * SYNCHRONOUSLY here, at the moment the interrupt is raised, when INTM=0,
     * and unlike c54x_irq_level_check's "IRQ-LEVEL take" it has no log of its
     * own. This says whether BRINT0 (vec 21) is in fact served silently by this
     * path on every raise, which would explain "never seen pending by the
     * poller" without any bug: the two mechanisms simply never overlap in the
     * window LEVELCHK-EMPIRICAL observes. */
    if (vec == 21) {
        static unsigned _sd = 0;
        if (_sd < 100) {
            _sd++;
            fprintf(stderr, "[c54x] SYNC-DISPATCH-PROBE #%u vec=21(BRINT0) imr_bit=%d "
                    "INTM=%d unmasked=%d idle=%d ifr_before=0x%04x PC=0x%04x insn=%u -> %s\n",
                    _sd, imr_bit, !!(s->st1 & ST1_INTM), unmasked, s->idle, s->ifr, s->pc,
                    s->insn_count,
                    s->idle ? (unmasked ? "DISPATCH(idle-wake)" : "STAYS-PENDING(idle,masked)") :
                    (!(s->st1 & ST1_INTM) && unmasked && s->delay_slots == 0) ? "DISPATCH(normal)" :
                    !unmasked ? "STAYS-PENDING(masked)" : "STAYS-PENDING(INTM=1-or-delay)");
        }
    }

    /* Per SPRU131: IDLE exits on ANY interrupt (masked or unmasked).
     * - Unmasked: branch to vector, set INTM=1
     * - Masked: just resume after IDLE, IFR bit stays set */
    if (s->idle) {
        s->idle = false;
        if (unmasked) {
            /* Service the interrupt: branch to vector */
            c54x_ifr_clear(s, (uint16_t)(1 << imr_bit), "vector-ex");
            s->sp--;
            data_write(s, s->sp, (uint16_t)(s->pc + 1));
            g_sp_ledger.irq_words_pushed++;
            s->sp--;
            data_write(s, s->sp, s->xpc);          /* XPC is always saved */
            g_sp_ledger.irq_words_pushed++;
            g_sp_ledger.irq_entries++;
            g_sp_ledger.net_words += 2;  /* PC and XPC pushed here, outside the exec ring */
            s->st1 |= ST1_INTM;
            /* Capture the preempted context; the foreground DP is unchanged. */
            g_last_intr_insn = s->insn_count; g_last_intr_vec = vec;
            g_last_intr_fg_pc = (uint16_t)(s->pc + 1); g_last_intr_fg_dp = dp(s);
            s->xpc = 0;                            /* fetch the vector from page 0 */
            uint16_t iptr = (s->pmst >> PMST_IPTR_SHIFT) & 0x1FF;
            s->pc = (iptr * 0x80) + vec * 4;
            if (vec == 28) {
                if (g_vec28_trace_en < 0)
                    g_vec28_trace_en = calypso_gate("CALYPSO_TRACE_VEC28_STACK", 0);
                if (g_vec28_trace_en && !g_vec28_tracing) {
                    g_vec28_tracing = true;
                    g_vec28_sp_entry = s->sp;
                    g_vec28_trace_pops = 0;
                    fprintf(stderr, "[c54x] VEC28-STACK-TRACE ARMED (idle-wake) "
                            "SP_entry=0x%04x data[SP]=0x%04x(expect XPC) "
                            "data[SP+1]=0x%04x(expect return PC) PC=0x%04x insn=%u\n",
                            s->sp, data_read(s, s->sp), data_read(s, (uint16_t)(s->sp + 1)),
                            s->pc, s->insn_count);
                }
            }
        }
        /* If masked: just wake, advance PC past IDLE */
        if (!unmasked) {
            s->pc++;  /* resume at instruction after IDLE */
        }
    } else if (!(s->st1 & ST1_INTM) && unmasked && s->delay_slots == 0) {
        /* Normal (non-IDLE) interrupt servicing.
         * The delay_slots == 0 guard is faithful to the C54x: an interrupt is
         * NOT recognised between a delayed branch (RETD/RCD/CALLD/BD) and its
         * two delay slots. Vectoring mid-delay would leave delay_slots armed,
         * so on return (RETE) the delayed_pc commit would happen in the wrong
         * context, over-popping SP, corrupting DP and reaching the self-CALA at
         * 0x70c3. The IFR bit stays set, so the interrupt is served on the next
         * call, at most ~2 instructions later, once delay_slots is back to 0. */
        /* FRAME-IT-PRIO (CALYPSO_FRAME_IT_PRIO): when the frame interrupt
         * (bit 12 / vec 28) is latched AND unmasked but a lower-priority
         * interrupt is about to be served - typically BRINT0, vec 21, which the
         * BSP over-delivers once per burst and which drowns the frame - serve
         * the FRAME first in this INTM=0 window. The requested interrupt's bit
         * stays pending and is served on the next edge. This drains a starved
         * frame interrupt (measured 5908 times pending against 1 taken) into
         * vec 28 -> scheduler 0x7234 -> dispatcher -> FB kernel. */
        if (vec != 28 && frame_it_prio_on() &&
            (s->ifr & (1u << 12)) && (s->imr & (1u << 12))) {
            static unsigned _fp = 0;
            if (_fp++ < 30)
                fprintf(stderr, "[c54x] FRAME-IT-PRIO override vec=%d->28 (frame latchee) "
                        "IFR=0x%04x IMR=0x%04x PC=0x%04x insn=%u\n",
                        vec, s->ifr, s->imr, s->pc, s->insn_count);
            vec = 28; imr_bit = 12;
            g_frame_it_level = false;   /* release the LEVEL hold, otherwise bit 12 re-asserts and vec 28 over-fires 7x per frame (139k INTM transitions) */
        }
        c54x_ifr_clear(s, (uint16_t)(1 << imr_bit), "vector-ex");
        s->sp--;
        data_write(s, s->sp, (uint16_t)s->pc);
        g_sp_ledger.irq_words_pushed++;
        s->sp--;
        data_write(s, s->sp, s->xpc);              /* XPC is always saved */
        g_sp_ledger.irq_words_pushed++;
        g_sp_ledger.irq_entries++;
        g_sp_ledger.net_words += 2;  /* PC and XPC pushed here, outside the exec ring */
        s->st1 |= ST1_INTM;
        /* Capture the preempted context; the foreground DP is unchanged. */
        g_last_intr_insn = s->insn_count; g_last_intr_vec = vec;
        g_last_intr_fg_pc = (uint16_t)s->pc; g_last_intr_fg_dp = dp(s);
        s->xpc = 0;                                /* fetch the vector from page 0 */
        uint16_t iptr = (s->pmst >> PMST_IPTR_SHIFT) & 0x1FF;
        s->pc = (iptr * 0x80) + vec * 4;
        if (vec == 28) {
            if (g_vec28_trace_en < 0)
                g_vec28_trace_en = calypso_gate("CALYPSO_TRACE_VEC28_STACK", 0);
            if (g_vec28_trace_en && !g_vec28_tracing) {
                g_vec28_tracing = true;
                g_vec28_sp_entry = s->sp;
                g_vec28_trace_pops = 0;
                fprintf(stderr, "[c54x] VEC28-STACK-TRACE ARMED (normal) "
                        "SP_entry=0x%04x data[SP]=0x%04x(expect XPC) "
                        "data[SP+1]=0x%04x(expect return PC) PC=0x%04x insn=%u\n",
                        s->sp, data_read(s, s->sp), data_read(s, (uint16_t)(s->sp + 1)),
                        s->pc, s->insn_count);
            }
        }
        /* Start the cycle trace on the frame interrupt (vec 28). */
        if (vec == C54X_IT_TPU_FRAME_VEC) {
            int3_cycle_start(s, s->pc);
        }
    }

    /* Log interrupts: first 20 + every 100th, so we can count them.
     * PMST/IPTR included so we can correlate which vector base the IRQ
     * lands at — INT3 at IPTR=0x1ff (vec=0xffcc) hits a garbage ROM stub,
     * INT3 at IPTR=0x140 (vec=0xa04c) hits the firmware's real handler. */
    static uint64_t int_log_count;
    int_log_count++;
    if (int_log_count <= 20 || (int_log_count % 100) == 0) {
        uint16_t iptr_now = (s->pmst >> PMST_IPTR_SHIFT) & 0x1FF;
        C54_LOG("IRQ #%llu vec=%d bit=%d: INTM=%d IMR=0x%04x IFR=0x%04x "
                "idle=%d PC=0x%04x PMST=0x%04x IPTR=0x%03x",
                (unsigned long long)int_log_count,
                vec, imr_bit, !!(s->st1 & ST1_INTM), s->imr, s->ifr,
                s->idle, s->pc, s->pmst, iptr_now);
    }
}

void c54x_wake(C54xState *s)
{
    s->idle = false;
}

/* Declared at FILE scope, to avoid -Wnested-externs. */
extern void calypso_twl3025_apply_phase(int16_t *iq_samples, int n_samples,
                                        uint32_t fn, uint8_t tn);
extern uint32_t calypso_trx_get_fn(void);
/* c54x_task_md(): declared in calypso_c54x.h, defined earlier in this file. */

void c54x_bsp_load(C54xState *s, const uint16_t *samples, int n)
{
    if (n > 2048) n = 2048;

    /* FEED-FP, leg 2 of 2 - OUTPUT. Leg 1 of 2 is in calypso_bsp.c
     * (bsp_trxd_readable): same gate CALYPSO_BSP_FINGERPRINT, same FNV-1a
     * hash, same cap. Here we fingerprint what leaves towards the RIF.
     *
     * Reading it: `identiques` close to the total = FROZEN burst. If the IN
     * leg varies and this one does not, the freeze is between the two. If
     * BOTH are frozen, look upstream of the UDP link (calypso-ipc-device).
     *
     * Caveat: this probe counts CONSECUTIVE repetitions, not distinct ones -
     * an A,B,A,B alternation would report identiques=0 while still being
     * pathological. Acceptable because the observed symptom is a CONSTANT,
     * but no conclusion beyond that case. */
    {
        static int fp_on = -1;
        if (fp_on < 0) fp_on = calypso_gate("CALYPSO_BSP_FINGERPRINT", 0);
        if (fp_on && n > 0) {
            uint32_t h = 2166136261u;
            unsigned nz = 0;
            for (int i = 0; i < n; i++) {
                h = (h ^ (samples[i] & 0xFF)) * 16777619u;
                h = (h ^ (samples[i] >> 8))   * 16777619u;
                if (samples[i]) nz++;
            }
            static uint32_t prev_h;
            static unsigned long long n_tot, n_same;
            n_tot++;
            if (n_tot > 1 && h == prev_h) n_same++;
            prev_h = h;
            if (n_tot <= 20 || (n_tot % 500) == 0)
                fprintf(stderr, "[c54x] FEED-FP OUT #%llu fp=%08x nz=%u/%d "
                        "identiques=%llu/%llu\n",
                        n_tot, h, nz, n, n_same, n_tot);
        }
    }

    memcpy(s->bsp_buf, samples, n * sizeof(uint16_t));
    s->bsp_len = n;
    s->bsp_pos = 0;

    /* RATE NORMALISATION - a SINGLE decimation point. The native feeds arrive
     * at INCONSISTENT rates: g_shunt.last_iq (shunt route, the SAME feed as
     * shunt_legit) is RAW @4SPS (1083333 Hz = 4x270833, undecimated, cf.
     * osmo-trx rx-sps=4), whereas the BSP deliver_buffered path already
     * decimates /4 to 1SPS. Depending on the active path c54x_bsp_load was
     * therefore handed 4SPS OR 1SPS, so the correlator and apply_phase (both
     * of which expect 1SPS, FCCH = +pi/2 per sample) mangled the 4SPS bursts
     * -> inconsistent dphi (tone folded onto DC). Decimation by DECIM happens
     * HERE, but ONLY if the input is oversampled (n indicates 4SPS: 148 sym
     * x4x2 = 1184 words, versus 296 @1SPS) -> never a double decimation,
     * output ALWAYS 1SPS, every path uniform (like shunt_legit).
     * DECIM = CALYPSO_BSP_IQ_DECIM (default 4). */
    {
        static int decim = -1;
        if (decim < 0) {
            const char *d = getenv("CALYPSO_BSP_IQ_DECIM");
            decim = (d && *d) ? atoi(d) : 4;
            if (decim < 1) decim = 1;
        }
        if (decim > 1 && n > 2 * 296) {          /* 4SPS detected (n ~1184) */
            int oc = 0;
            for (int k = 0; (k * decim) * 2 + 1 < n; k++) {
                s->bsp_buf[2 * oc]     = s->bsp_buf[2 * (k * decim)];
                s->bsp_buf[2 * oc + 1] = s->bsp_buf[2 * (k * decim) + 1];
                oc++;
            }
            n = oc * 2;                          /* n becomes the 1SPS count */
            s->bsp_len = n;
        }
    }

    /* AFC AT THE CONVERGENCE POINT. The VCXO rotation (apply_phase) used to be
     * applied only in calypso_bsp_deliver_buffered (calypso_bsp.c:1888), which
     * is NOT the live native path (measured: b-bsp-load-ok=0, the feed goes
     * elsewhere). The native DSP therefore received UNROTATED samples -> OPEN
     * AFC loop -> the firmware integrates the error with no effect on the
     * samples -> DAC runaway (-700 -> 4095, +34 kHz) -> FCCH rotation outside
     * the DSP capture range (+-20 kHz) -> nonsensical TOA. Here ALL feeds
     * converge (c54x_bsp_load -> RIF), so the rotation is applied ONCE, on
     * bsp_buf (mutable), and bsp_buf is what is fed to the RIF. See
     * CHAINE_RF_MATERIELLE.md:106: "apply_phase just before c54x_bsp_load".
     * tn=0 (FB/SB = TS0; a constant phase offset does not change the corrected
     * FREQUENCY). Inert when CALYPSO_TWL3025_AFC=0 (apply_phase returns
     * early). */
    calypso_twl3025_apply_phase((int16_t *)s->bsp_buf, n / 2,
                                calypso_trx_get_fn(), 0);

    /* The same burst feeds the RIF receive FIFO, which is the route the DSP
     * firmware actually reads it through (PORTR DRR after seeing SPCR).
     * bsp_buf stays in place for the probes and for the older PORTR PA=0xF430
     * path (CALYPSO_FIX_PORTR). What is fed is bsp_buf (= the samples rotated
     * by the AFC above), not `samples` (const, unrotated). */
    /* FN GATING - push to the RIF/correlator ONLY the bursts of an FCCH frame
     * during FB acquisition. Root cause: this feed had NO FN gate at all
     * (DIRECT_FEED bypasses the match), so the correlator buffer at 0x0cce
     * received EVERY burst (mostly non-FCCH), mixed and overwritten ->
     * VARIABLE magnitude (~100..28000, versus a constant one for FCCH) ->
     * flat correlation surface -> argmax at the window edge r39 -> TOA=39
     * (never 23). Feeding only the FCCH frames {1,11,21,31,41} (emulated
     * offset, cf. calypso_dsp_shunt.c:3710) leaves the correlator seeing only
     * the real tone. Gate CALYPSO_RIF_FCCH_ONLY (default OFF, reversible).
     * The gating applies ONLY in FB mode (d_task_md 5/8) so the other tasks
     * (SB/NB) are not starved of their burst. */
    {
        static int rif_fcch = -1;
        if (rif_fcch < 0) rif_fcch = calypso_gate("CALYPSO_RIF_FCCH_ONLY", 0);
        int _push = 1;
        if (rif_fcch) {
            /* Filter in FB (task_md 5/8) AND in IDLE (0): the RIF stage keeps
             * only the LAST burst pushed, so letting the non-FCCH idle bursts
             * through (87% of the time) makes the FB DMA drain a polluted
             * burst. NOT filtered in SB (6) / NB: those tasks need THEIR own
             * frame. */
            uint16_t _md = c54x_task_md(s);   /* FB=5, TCH_FB=8, SB=6 */
            if (_md == 5 || _md == 8 || _md == 0) {
                int _p = (int)(calypso_trx_get_fn() % 51u);
                /* Natively the mission does not switch to 6 on the SCH frame
                 * (the get_task_md fallback stays 5/0), so an FCCH-only filter
                 * also smothered the SCH -> 0x0e4e (the SB DMA destination)
                 * drained on a stale FCCH/idle burst -> garbage sb_toa. GSM
                 * 05.02: on the BCCH during acquisition, only FCCH (tone) and
                 * SCH (sync) carry meaning on TS0. So push FCCH AND SCH
                 * (= FCCH+1). The DMA routes by AAD (FCCH -> 0x0cce, SCH ->
                 * 0x0e4e) on DIFFERENT frames, so 0x0cce stays clean (FCCH
                 * only) and 0x0e4e finally receives a real SCH. */
                int _is_fcch = ((_p % 10) == 1) && (_p <= 41); /* FCCH in {1,11,21,31,41} */
                int _is_sch  = ((_p % 10) == 2) && (_p <= 42); /* SCH  in {2,12,22,32,42} */
                _push = _is_fcch || _is_sch;
            }
        }
        if (_push)
            calypso_rif_rx_burst(s, s->bsp_buf, n);
    }

    /* Confirm what the PORTR PA=0x0034 serving path will hand the DSP,
     * and also flag if the DSP consumed less than half of the previous
     * batch before a new one arrived (would indicate correlator starvation
     * or DSP never reading via PORTR at all). */
    static uint64_t load_count;
    load_count++;
    if (load_count <= 10 || (load_count % 1000) == 0) {
        C54_LOG("BSP LOAD #%llu n=%d: %04x %04x %04x %04x %04x %04x %04x %04x",
                (unsigned long long)load_count, n,
                n > 0 ? samples[0] : 0, n > 1 ? samples[1] : 0,
                n > 2 ? samples[2] : 0, n > 3 ? samples[3] : 0,
                n > 4 ? samples[4] : 0, n > 5 ? samples[5] : 0,
                n > 6 ? samples[6] : 0, n > 7 ? samples[7] : 0);
    }
}

