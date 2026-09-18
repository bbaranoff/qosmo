/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * calypso_c54x.c — coeur TMS320C54x : init, reset, boucle c54x_run, API publique.
 *
 * [2026-09-18] Ce fichier faisait 21475 lignes. Decoupe par role :
 *   calypso_c54x.c  init / reset / boucle c54x_run / API publique
 *   c54x_exec.c     c54x_exec_one et les familles d'instructions
 *   c54x_decode.c   resolution des operandes (Smem/Lmem/Xmem), conditions
 *   c54x_mem.c      memoire donnee et programme, overlay, verrous
 *   c54x_irq.c      IFR/IMR, IT trame, interruptions
 *   c54x_probes.c   sondes et traces (diagnostic seul)
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

    /* Task slots in both write pages — DSP word addresses :
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
     * Cycle-budget calibration: real C54x at 65 MHz means 1 cycle ≈ 15 ns.
     * The dispatcher body is ~8 instructions per pass (matches the 8 hot
     * PCs observed). One pass ≈ 8 cycles ≈ 120 ns of *DSP* time.
     *
     * Per Claude web 2026-05-07 review: previously this returned a
     * fixed 8-cycle skip per call regardless of host wall time. Combined
     * with c54x_run(256000) that meant a single tick callback could
     * burn through 32k FF iterations in microseconds host time but
     * accumulate the full 256k cycles credit on the DSP — the net
     * effect on QEMU virtual time was minimal (DSP cycles aren't a
     * QEMU clock anyway), so this isn't itself the cause of the BTS
     * timing skew. But to match wall-clock more honestly we now cap
     * the FF run length per c54x_run invocation: at most enough skips
     * to consume the budget (n_insns) without overshooting.
     *
     * The actual wall-clock alignment (CLK IND cadence) is owned by
     * the TDMA timer in calypso_trx.c, not by this function. */
    *consumed_out = 8;
    ff_hits++;
    if ((ff_hits & 0xFFFFFFu) == 0) {
        C54_LOG("DSP IDLE FF: %llu skips so far (PC=0x%04x SP=0x%04x)",
                (unsigned long long)ff_hits, s->pc, s->sp);
    }
    return true;
}

/* === CALYPSO_TRAP_OOR hook v2 (root-cause probe SP descent 0..checkpoint) ===
 * v2 redesign: T1/T2 dropped (scheduler exonerated → SP clobber lives in
 * legit code, whitelist can't see it). Pure observability:
 *   - g_sp_trail[256] : SP changes with |Δ|>32 (scheduler reloads, large
 *     allocations) — skip push/pop ±1 noise.
 *   - sp_low watermark : every new low logged (PC-coalesced power-of-10)
 *     — catches BOTH absolute reloads AND push-drain runaway.
 *   - Per-event A_low captured (= candidate STL A,Smem source).
 *   - Halt at fixed checkpoint (env CALYPSO_TRAP_CHECKPOINT, default 4.2M
 *     = just after the insn=4.09M SP recovery 0x0008→0x2900). */

struct c54x_g_sp_trail_s g_sp_trail[256];
unsigned g_sp_trail_idx = 0;

/* sp_low watermark — coalesced by PC */
uint16_t g_sp_low = 0xFFFF;
uint16_t g_sp_low_pc = 0xFFFF;
unsigned g_sp_low_hits_at_pc = 0;
unsigned g_sp_low_distinct_pcs = 0;

/* SP-decrement histogram per-PC (fix 2026-05-24 v3 — Claude web correction).
 *
 * Gating par VALEUR SP (pas insn_count) — robuste à :
 *   - DSP idle fast-forward (dsp_idle_fast_forward L5937 inflate insn_count
 *     sans exécuter d'opcodes)
 *   - jitter externe wall-clock (bridge/osmocon/BTS sur UDP+PTY → instant
 *     guest où arrive un burst varie run-à-run, insn_count des events
 *     déclenchés par bursts pas stable)
 *
 * Logique : armé quand SP descend SOUS le plateau (default < 0x2000, sous
 * 0x3fb0 où SP stationne 632k→3.5M insns du run jackpot). Reste armé
 * jusqu'au dump quand SP < 0x0100 (proche underflow). Pendant cette fenêtre,
 * compte chaque SP-décrement par PC.
 *
 * 3 bénéfices :
 *   1. Auto-aligne sur la descente quels que soient FF et jitter externe
 *   2. Rend la question FF caduque (descente = real insns, pas idle-poll)
 *   3. Exclut churn équilibré du plateau (PSHM/POP matched à PCs distincts
 *      pollue insn-window mais pas SP-window — leaker domine mécaniquement)
 *
 * Override env :
 *   CALYPSO_SP_HIST_ARM   (default 0x2000) — threshold pour armer
 *   CALYPSO_SP_HIST_DUMP  (default 0x0100) — threshold pour dumper
 *
 * Capture toutes voies SP-write : direct s->sp--, MMR_SP via data_write
 * callback, IRQ push (audit couverture 2026-05-24 : tous paths passent
 * par s->sp variable). */
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

/* === Raw SP ring buffer (Patch 3 — 2026-05-25, rev 2) ===
 * Per-iteration record of (insn, PC, SP, op) at top-of-loop, no filter,
 * no sign classification. Plusieurs triggers configurables.
 *
 * Rev 1 (floor-cross) : a montré que le « plongeon SP→0 » est en réalité
 * un wrap-forward par pops dans un boot-stub spiral (PC=0x0000/0x0001
 * en boucle, SP++ par RET). Le kill réel = un RET corrompu qui saute
 * à 0x0000 BIEN AVANT le wrap, dans [3.5M, 4.09M] insns. Floor-cross
 * arrive 600k insns trop tard, déjà dans le spiral.
 *
 * Rev 2 (bootstub-entry) : trigger sur l'EDGE prev_pc ∉ [0x00,0x7F] →
 * s->pc ∈ [0x00,0x7F]. Capture la transition exacte = le RET fauteur
 * + son SP + le mot poppé (mem[topgate_last_sp]). Discrimine 2 bugs
 * radicalement différents :
 *   - SP valide (~0x3fbb) + mem[SP]=0 → return slot écrasé par un
 *     write sauvage. fd28-fd2a n'y change rien. À chasser autrement.
 *   - SP en non-stack (~0x2bc0) → famille 0xfd2a A=AR4. fd28-fd2a
 *     devient le fix.
 *
 * Env gates :
 *   CALYPSO_SP_RING=1          active (default OFF, zéro coût sinon)
 *   CALYPSO_SP_RING_MAX=N      cap dumps par run (default 4)
 *   CALYPSO_SP_RING_TRIG=mode  floor|bootstub|both (default bootstub)
 *   CALYPSO_SP_RING_INSN_MIN=N skip first N insns (default 1000000 — le
 *                              firmware Calypso fait des CALL légitimes
 *                              au boot stub 0x0000/0x0001 en phase init,
 *                              le 1er CALL captérait un faux positif et
 *                              consommerait le one-shot. Le vrai bug
 *                              observé est dans [3.5M, 4.09M] insns) */
SpRingEntry g_sp_ring[SP_RING_SZ];
unsigned    g_sp_ring_head = 0;
uint64_t    g_sp_ring_total = 0;
int         g_sp_ring_enabled = -1;
unsigned    g_sp_ring_dump_count = 0;
unsigned    g_sp_ring_dump_max = 0;
/* Trigger mode (rev 2) : 1 = floor-cross, 2 = bootstub-entry, 3 = both */
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
    /* Trigger mode parse (rev 2). Default = bootstub (le seul utile post
     * rev-1 — floor-cross firait dans le spiral, trop tard). */
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

/* Rev 2 : detect edge PC entry into boot stub area [0x0000, 0x007F].
 * topgate_last_pc = PC of insn just executed (the RET that branched).
 * cur_pc          = destination = popped return address.
 * topgate_last_sp = SP before the RET pop.
 * cur_sp          = SP after the RET pop (= topgate_last_sp + 1 if 1-word).
 * Capture verbose state + dump ring to identify the corrupting RET.
 * Static cap : one detailed dump per run (le 1er, qui contient le
 * caller; subsequent fires sont des re-entries du même spiral). */
int g_bootstub_dumped = 0;
static void sp_ring_check_bootstub_entry(C54xState *s,
                                         uint16_t prev_pc, uint16_t prev_op,
                                         uint16_t prev_sp, uint16_t cur_pc,
                                         uint16_t cur_sp, unsigned insn)
{
    if (g_sp_ring_enabled <= 0) return;
    if (!(g_sp_ring_trig_mode & 2)) return;
    if (g_bootstub_dumped) return;
    /* Skip boot phase : firmware fait des CALL légitimes au boot stub
     * 0x0000-0x0001 pendant l'init (LDMM SP,B est documenté). Le 1er
     * trigger sans gate fire à insn=145 et consomme le one-shot. */
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

    /* Diagnostic discriminator — match user's discrimination criteria :
     *   SP valide (~0x3fbb plage observée) + popped==0 → return slot
     *     écrasé par write sauvage. 0xfd2a est innocent.
     *   SP en zone non-stack (~0x2bc0 ou similar buffer) → famille
     *     0xfd2a A=AR4. fd28-fd2a est le fix. */
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

    /* Top-K par dec_count (trickle leak). */
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

    /* Top-K par |delta_sum| (single-event jump corrupteur — 1 event huge). */
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
    /* Patch 1 (2026-05-25) : freeze RETIRÉ. L'ancien `if (g_sp_dec_dumped)
     * return;` faisait du one-shot, donc tous les events post-1er-dump
     * étaient perdus. Sans freeze, plusieurs dumps consécutifs si SP
     * reste sous threshold — c'est borné en pratique par le rate-limit
     * du sp_ring_dump_max et par le edge-detect dans le top-of-loop. */

    /* Patch 2 (2026-05-25) : drop le cast (int16_t). Le wrap signé
     * mis-classifiait les chutes high→low en pop. Pure int32 sub :
     *   0x9006→0x0000 : delta = -36870 (correct, descent capturé)
     *   0xC000→0x0000 : delta = -49152 (correct, descent capturé)
     * Note : casse l'underflow wrap (0x2bc0→0xfff8 = +52280, vu comme
     * pop), mais l'histo n'est plus la source de vérité pour le kill
     * — c'est le ring buffer qui tranche. Histo = drift trickle uniquement. */
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

    /* Record event AVANT le dump check (fix 2026-05-24 v4 — sinon un
     * single-event jump qui franchit DUMP en une instruction est perdu). */
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
        /* Log first 10 events verbatim — for single-event jumps the corrupteur
         * est dans les premiers events (souvent un seul mot dans le histo). */
        if (g_sp_dec_total_events <= 10) {
            if (calypso_debug_enabled("SP-HIST")) fprintf(stderr,
                "[c54x] SP-HIST EVENT #%u pc=0x%04x op=0x%04x "
                "sp_before=0x%04x sp_now=0x%04x delta=%d insn=%u\n",
                g_sp_dec_total_events, exec_pc, exec_op,
                sp_before, sp_now, delta, insn);
        }
    }

    /* DUMP : APRÈS l'accounting, vérifier seuil dump.
     * Patch 1 (2026-05-25) : edge-trigger only — dump quand SP croise
     * sous le floor (sp_before >= threshold && sp_now < threshold).
     * Rev 2 : gaté par g_sp_ring_trig_mode (bit 0 = floor). Par défaut
     * bootstub seulement, parce que floor-cross fire dans le spiral
     * (trop tard) — cf rev 1 finding. */
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
    /* SP-HIST dump (fix v3 2026-05-24 — SP-windowed, no-insn-dep). */
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

    /* Log first 10 instructions of each run (for 2nd cycle debug) */
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

    /* XPC tracking probe (2026-05-15 nuit, per Claude web Q1).
     * Hypothèse à valider : le path completion CCCH demod passe par PROM1
     * (XPC=1) via le B 0x9ab1 à 0x19aac. Si XPC=1 jamais atteint → bug
     * dans le route initial. Si atteint mais PC pas dans 0x9aac+ → entrée
     * OK mais pas cette zone. Tracking :
     *   - insn count par XPC (0..3)
     *   - dernier PC visité par XPC
     *   - first_visit_insn par XPC (= quand on entre en XPC=N pour la 1ère fois)
     *   - ring buffer 16 derniers PCs visités sous XPC=1 (zone d'intérêt)
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
                /* Dernier 16 PCs visités sous XPC=1 (ring buffer) */
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

    /* DISPATCH-CALLER probe (2026-05-15 nuit, per Claude web).
     * Les 3 callers de 0x9aaf identifiés par scan PROM :
     *   PC=0x8815 : f074 9aaf  (B 0x9aaf depuis table @0x8810)
     *   PC=0x9296 : f274 9aaf  (BD 0x9aaf depuis routine spécifique)
     *   PC=0x9418 : f274 9aaf  (BD 0x9aaf depuis autre routine)
     * Log A, AR0..2, data[0x0828/9] à chaque hit. */
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

    /* AR7-INIT-CHAIN + MVMD-AR7-BRC + RPTB-ARMED probe (Claude web 2026-05-15
     * nuit étape 3). Diagnostic : valeur AR7 au moment du MVMD AR7,BRC à
     * PC=0x8208, sa chaîne causale (16 derniers writes AR7), et l'état BRC
     * post-RPTBD setup. */
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

        /* (b) Snapshot complet à chaque hit de PC=0x8208 (MVMD AR7, BRC) */
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

        /* (c) État RPTB après setup (PC=0x820c = delay slot post-RPTBD) */
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

    /* INT3-BLOCKED probe (Claude web 2026-05-15 nuit, étape 2).
     * Sample 1/1000 du context (PC/ST1/BRC/XPC) quand INT3 pending + INTM=1.
     * Discrimine : (a) opcode set INTM=1 sans clear (variante POPM),
     * (b) RPTB long non-interruptible (BRC > 0 partout),
     * (c) STM ST1 / MVDM ST1 brut. Cf matrice Claude web. */
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

    /* IRQ-FRAME-HEALTH probe (Claude web 2026-05-15 nuit, étape 1).
     * Diagnostic timing TDMA vs wall-clock : INT3 = frame interrupt
     * (IMR bit 3, vec 19, addr 0xFFCC). Mesure fire/serviced/missed/latency.
     * Discrimine : ISR mal vectorisée (service<fire), TPU/TSP fail (fire=0),
     * compute trop lent (missed>0). Cause root LOST 3468 + variance XPC. */
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

    /* EXIT-COMPUTE + IRQ-DURING-COMPUTE probe (Claude web 2026-05-15 nuit).
     * Le DSP tourne en XPC=2 dans zone hot 0xdf80..0xdfc0 (CCCH demod MAC loop).
     * Discrimine entre 3 hypothèses :
     *   (1) compute jamais exit (threshold non franchi)
     *   (2) IRQ jamais fire (TPU/TSP source manquante)
     *   (3) IRQ fire mais pas serviced (INTM stuck ou ISR mal vectorisée)
     * Matrice de décision basée sur exits_count + irq_pending_in_compute. */
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

    /* DISPATCH-ENTRY probe (per Claude web option 3 hybride).
     * Le dispatcher caller saute vers 0x8810 + task_id*3, où chaque entry =
     * { 0xf4e4 (FRET ou padding), 0xf074 (B opcode), <target> }.
     * On probe le PC qui correspond au début d'un entry (PC = 0x8810 + N*3).
     * task_id estimé = (PC - 0x8810) / 3.
     * Si entry exec OK → on lit data[PC+2] qui est le target. */
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
            /* Hot zone after ARP fix: b8e9..b906 (run 2, vec1 handler). */
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
            /* DARAM 0x066F..0x0682 wait-loop disasm (run 3 stuck zone).
             * Looking for B-self (f073 066f) vs IDLE n (f7e1/f7e2/f7e3)
             * vs poll-and-branch. If IDLE found → emulator IDLE handler
             * is the real bug (3 runs all hit the same opcode, terminate
             * in different bassins because PMST/IPTR varies). */
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
        /* === SILICON-BOOT-ROM REDIRECT (réactivé 2026-05-30) ===========
         * Le dump PROM ne contient PAS le mask-ROM silicon du Calypso. Sur
         * vrai HW, ce ROM masqué tourne au reset, pose SP=0x5AC8 + MMR, puis
         * saute à l'entrée firmware PROM0[0x7120] (= STM #0x5AC8,SP vérifié :
         * prog[0x7120]=0x7718 STM #lk,SP, prog[0x7121]=0x5ac8). On MODÉLISE
         * ce hardware manquant — ce n'est PAS un override d'instruction
         * firmware : on route vers l'entrée firmware propre, qui fait elle-
         * même son init SP.
         *
         * RÉGRESSION corrigée : retiré le 29/05 (c3ec660 « relancer via
         * 0xFF80 réel »). Sans lui, reset → 0xff80(FB) → 0xb410 → CC → 0x76f8
         * SANS jamais exécuter STM #0x5AC8,SP → SP coincé à 0x1100 (invalide,
         * = aire MMR/AR0) → over-pop boot (net→-57) → return corrompu →
         * self-CALA 0x70c3 → spirale (16M pushes) → PMST 0x70C4 fuit en
         * TOA=28868 côté osmocon → FB jamais locké.
         *
         * Gate SP==0x1100 = cold-reset uniquement (valeur silicon-reset). Une
         * fois SP=0x5AC8 posé par 0x7120, la condition retombe → les walks
         * séquentiels firmware passant par 0xff80 plus tard NE sont PAS
         * hijackés (cf SOFT-RESET-TRIG ci-dessous, insn>100k). */
        /* EXPÉRIENCE 2026-05-30 (CC-web) testée et CONCLUE : poser SP=0x5AC8 sans
         * rediriger le PC (laisser 0xff80→0xb410 tourner) → le reset-handler
         * 0xb410 s'exécute MAIS n'appelle PAS le boot-init 0x7000-0x7025 (reste
         * 1 hit incident) ; FB-dispatch échoue identiquement. Donc FB-dispatch
         * n'est PAS la queue du boot-init = issue steady-state SÉPARÉE. Acquis :
         * l'over-pop était 100% un artefact de SP=0x1100 (F@0x76f8 tourne propre
         * depuis 0x5AC8, no wedge). On revient au redirect 0x7120 committé. */
        /* === REDIRECT NEUTRALISÉ PAR DÉFAUT (2026-05-31) ===================
         * Ce bloc MODÉLISE un mask-ROM TI absent du dump = un HACK (simulation).
         * Méthode user : NE RIEN simuler. On laisse le vrai reset vector jouer
         * (0xff80 = FB 0xb410 = vrai reset handler firmware) et on débugge
         * CHAQUE bug réel de la chaîne de boot avec les valeurs qu'elle produit.
         * Le redirect est conservé derrière CALYPSO_REDIR_LEGACY=1 UNIQUEMENT
         * pour comparaison A/B ; OFF par défaut. Premier bug réel attendu sans
         * lui (cf ancien commentaire) : à 0xb410 le CC saute le STM #0x5AC8,SP
         * → SP reste 0x1100 → over-pop. C'est CE bug qu'on trace, pas qu'on
         * contourne. */
        /* @BEQUILLE — REDIR_LEGACY  (CALYPSO_REDIR_LEGACY, EXISTS, defaut OFF)
         *   masque  : le reset vector reel 0xff80 -> 0xb410 est detourne pour simuler le
         *             mask-ROM TI absent du dump (le commentaire ci-dessus l'assume).
         *   retirer : des que le vrai reset handler 0xb410 pose SP=0x5AC8 lui-meme
         *             (STM #0x5AC8,SP correctement decode) et que le boot deroule sans
         *             over-pop — c'est le bug a tracer, pas a contourner.
         *   NB      : maitre de INITTAB et REDIR7000 ; exclut MASKROM_INIT.
         */
        static int redir_legacy = -1;
        if (redir_legacy < 0) redir_legacy = calypso_gate("CALYPSO_REDIR_LEGACY", 0);
        if (redir_legacy && s->pc == 0xFF80 && s->sp == 0x1100) {
            static int redirect_log;
            /* EXPÉRIENCE CALYPSO_REDIR7000 (2026-05-30) : le redirect→0x7120 saute
             * l'init qui peuple les tables BACC-A (data[0x4c5b]/0x3fe1) → A=0 →
             * boot stub → dispatch dormant. Test : poser SP=0x5AC8 (mask-ROM) +
             * rediriger vers 0x7000 (init COMPLÈTE : tables + A) pour que BACC A
             * atteigne la vraie entrée firmware. cf SESSION_2026-05-29 fix#2. */
            /* @BEQUILLE — REDIR7000  (CALYPSO_REDIR7000, EXISTS, defaut OFF)
             *   masque  : l'init des tables BACC-A (d[0x4c5b]/d[0x3fe1]) que le point d'entree
             *             0x7120 suppose deja faite : on redirige le reset vers 0x7000.
             *   retirer : identique a INITTAB (table peuplee par le chemin firmware).
             *   NB      : imbriquee dans REDIR_LEGACY, et ecrasee par INITTAB (else if).
             */
            static int redir7000 = -1;
            if (redir7000 < 0) redir7000 = calypso_gate("CALYPSO_REDIR7000", 0);
            if (redirect_log < 3) {
                C54_LOG("SILICON-BOOT-REDIRECT PC=0xFF80 SP=0x1100 → 0x%04x%s",
                        redir7000 ? 0x7000 : 0x7120,
                        redir7000 ? " (REDIR7000: SP=0x5AC8 + init complète A-tables)" : "");
                redirect_log++;
            }
            /* VALIDATION CALYPSO_INITTAB (env, réversible) : prouve que peupler la
             * table de dispatch débloque FB. Pose SP, PUSH retour=0x7120, saute à
             * 0xc704 (table-init) → peuple data[0x4c24-0x4c5d] → RET vers 0x7120 →
             * boot normal continue AVEC table peuplée → BACC A atteint les vrais
             * handlers. Débloque FB → root+fix prouvés ; sinon → table pas le seul. */
            /* @BEQUILLE — INITTAB  (CALYPSO_INITTAB, EXISTS, defaut OFF)
             *   masque  : l'absence du mask-ROM TI qui peuple la table de handlers de tache
             *             0x4c24-0x4c5d au reset ; sans elle 0x7120 fait BACC d[0x4c5b]=null.
             *   retirer : des que la table est peuplee par un chemin firmware (0xc704 atteint
             *             nativement apres le clear 0x8869) — sonde INSTALL-TRACE d[4c5c]!=0.
             *   NB      : sans CALYPSO_REDIR_LEGACY, ce gate n'est jamais evalue.
             */
            static int inittab = -1;
            if (inittab < 0) inittab = calypso_gate("CALYPSO_INITTAB", 0);
            if (inittab) {
                s->sp = 0x5AC8;
                s->sp--; s->data[s->sp] = 0x7120;   /* retour = boot normal */
                s->pc = 0xc704;                       /* run table-init → RET 0x7120 */
            } else if (redir7000) { s->sp = 0x5AC8; s->pc = 0x7000; }
            else s->pc = 0x7120;
        }
        /* [2026-07-23] MASK-ROM TABLE-INIT (default ON) : le dump PROM ne contient
         * pas le mask-ROM TI qui, au reset, PEUPLE la table de handlers de tache
         * (0x4c04-0x4c5d) -- sinon 0 apres le clear RPTB 0x8869 -> l'entree firmware
         * 0x7120 BACC d[0x4c5b]=null et 0x7025/0xd247/0xc8e9/corr ne tournent JAMAIS.
         * On MODELISE ce HW absent : au cold-reset (PC=0xff80,SP=0x1100) pose SP=0x5AC8,
         * PUSH retour=0xb410 (reset handler normal -> park b41c PRESERVE), saute 0xc704
         * (fill table, RET @0xc826, adressage ABSOLU -> OK meme AR/DP non-init). INITTAB
         * a prouve que peupler la table debloque FB. OFF via CALYPSO_MASKROM_INIT_OFF=1.
         * Exclusif avec redir_legacy (qui gere deja 0xff80). */
        if (!redir_legacy && s->pc == 0xFF80 && s->sp == 0x1100) {
            /* @BEQUILLE — MASKROM_INIT  (CALYPSO_MASKROM_INIT, EXISTS, defaut OFF)
             *   masque  : identique a INITTAB — mask-ROM TI absent qui pose SP=0x5AC8 et
             *             peuple la table de handlers au cold-reset.
             *   retirer : meme condition qu'INITTAB (table peuplee par chemin firmware).
             *   NB      : le commentaire ci-dessus renvoie a CALYPSO_MASKROM_INIT_OFF, variable
             *             qui N'EXISTE PAS — le gate reel est opt-in CALYPSO_MASKROM_INIT.
             */
            static int mrti = -1;
            if (mrti < 0) mrti = calypso_gate("CALYPSO_MASKROM_INIT", 0);   /* [2026-07-23] OPT-IN (default OFF) : le forcing boot-op derail (etat froid) ; garde pour A/B */
            if (mrti) {
                static int mrti_log = 0;
                if (mrti_log < 2) { mrti_log++;
                    fprintf(stderr, "[c54x] MASK-ROM-INIT: cold-reset SP=0x5AC8, run table-init 0xc704 (RET 0xb410) insn=%u\n", s->insn_count); }
                s->sp = 0x5AC8;
                s->sp--; s->data[s->sp] = 0x7120;   /* retour = entree firmware (BACC d[0x4c5b] peuple) */
                s->pc = 0xc704;                       /* peuple table handlers -> RET 0x7120 -> operationnel */
            }
        }
        /* [2026-07-23] TABLE RE-POPULATE apres le clear boot : la routine 0x8866-0x886a
         * (RPTB memset 64 mots) WIPE la table handlers APRES le populate mask-rom (insn
         * ~19793 > insn 92). Le firmware normal ferait clear->populate mais 0xc704 n'est
         * jamais atteint apres le clear. Fix : au RET du clear (0x886a), si la table est
         * vide, rediriger vers 0xc704 (populate ; son RET @0xc826 depile le meme retour
         * = caller du clear). Self-heal a chaque clear. OFF via CALYPSO_MASKROM_INIT_OFF. */
        /* [2026-07-23] BOOTSTRAP OPÉRATIONNEL 0xd247 : le sous-système op (install
         * table handlers 0xc704 + slots TDMA 0xc867 + vecteurs) est AUTO-RÉFÉRENTIEL
         * (appelé seulement depuis 0x7025, jamais bootstrappé -> mask-ROM absent). Sans
         * lui : d[0x4c5c]=0 + d[0x3f6b]=0xd294(RET no-op) -> acquisition FB no-op ->
         * d[3f70] jamais 2 -> corr jamais. On MODÉLISE le bootstrap mask-ROM : au terminal
         * boot-init 0xb3e4 (état prêt : SP=0x5AC8, cellules seedées), one-shot run 0xd247
         * (RET @0xd25f -> revient à 0xb3e4). OFF via CALYPSO_D247_OFF=1. */
        if (s->pc == 0xb3e4) {
            /* @BEQUILLE — D247  (CALYPSO_D247, EXISTS, defaut OFF)
             *   masque  : l'absence du bootstrap mask-ROM TI qui, sur silicium, appelle le
             *             sous-systeme operationnel 0xd247 (install table handlers 0xc704 +
             *             slots TDMA 0xc867 + vecteurs) ; en QEMU 0xd247 n'a d'appelant natif
             *             qu'a PROM0 0x7102, bloc jamais atteint au boot froid. On PUSH le
             *             retour et on detourne le PC.
             *   retirer : des que le bloc appelant natif 0x70ce-0x7106 est atteint (sonde
             *             D247-TRACE site 0x7102 non nulle), OU des que la table d[4c5c] est
             *             peuplee par le chemin firmware.
             *   NB      : le commentaire ci-dessus annonce CALYPSO_D247_OFF=1 — cette variable
             *             n'existe pas, le gate reel est opt-in CALYPSO_D247.
             */
            static int _d247 = -1;
            if (_d247 < 0) _d247 = calypso_gate("CALYPSO_D247", 0);   /* [2026-07-23] OPT-IN OFF : bootstrap pousse dans 0xc6a5 (init coeffs) mais boucle sur source vide. Garde A/B */
            static int _d247_done = 0;
            if (_d247 && !_d247_done) {
                _d247_done = 1;
                fprintf(stderr, "[c54x] BOOTSTRAP-D247 @0xb3e4 : run 0xd247 (install table+slots+vec) insn=%u SP=0x%04x\n", s->insn_count, s->sp);
                s->sp--; s->data[s->sp] = 0xb3e4;   /* retour = terminal boot-init */
                s->pc = 0xd247;
            }
        }
        if (s->pc == 0x886a && s->data[0x4c5c] == 0) {
            /* @BEQUILLE — REPOPULATE  (CALYPSO_REPOPULATE, EXISTS, defaut OFF)
             *   masque  : le memset RPTB 0x8866-0x886a wipe la table de handlers apres son
             *             peuplement, sans que le firmware rappelle 0xc704 ; la branche reelle
             *             = l'ordre firmware clear -> populate. On detourne le PC vers 0xc704.
             *   retirer : des que 0xc704 est atteint APRES le clear par le flot natif
             *             (D247-TRACE : d[4c41]/d[4c46] non nuls en fin de boot).
             */
            static int mrti2 = -1;
            if (mrti2 < 0) mrti2 = calypso_gate("CALYPSO_REPOPULATE", 0);   /* [2026-07-23] OPT-IN OFF : peupler 0x4c5c ne debloque PAS l acquisition FB (teste : fb0_att reste 0). Garde pour A/B */
            if (mrti2) {
                static int rlg = 0;
                if (rlg < 3) { rlg++;
                    fprintf(stderr, "[c54x] TABLE-REPOPULATE @0x886a (clear a wipe la table) -> run 0xc704 insn=%u\n", s->insn_count); }
                s->pc = 0xc704;
            }
        }
        /* [2026-07-23] D247-TRACE (READ-ONLY, no state mutation) : le workflow de
         * recon a montre que 0xd247 A un vrai appelant natif unique -- PROM0 0x7102,
         * dans le bloc operationnel 0x70ce-0x7106 (PAS un stub mask-ROM orphelin comme
         * suppose par BOOTSTRAP-D247 ci-dessus, qui l'appelait a tort au cold-reset
         * ou SP est invalide -> derail 0x3350). Ces sondes verifient SANS RIEN FORCER :
         * (a) exec_pc atteint-il 0x7102 nativement (le bloc appelant tourne-t-il) ?
         * (b) 0xd247 fire-t-il, avec quel etat table avant/apres son RET (@0xd25f) ?
         * (c) le clear 0x87ff (callers trouves dans PROM1 via FCALL, PAS PROM0) tourne-t-il,
         *     et AVANT ou APRES 0xd247 -- wipe-t-il le travail de 0xc704 ? d[4c41]/d[4c46]
         *     = 2 slots de la table lus par le dispatcher 0xc8e9 (CALA), indicateurs directs
         *     de succes d'install. Defaut ON, cap 20/site. OFF via CALYPSO_D247_TRACE_OFF=1
         *     (atoi, pas presence -- cf bug de gating INIT_435B_OFF corrige plus tot). */
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
        /* [2026-07-23] CYCLE-TRACE (READ-ONLY) : cycle 1 (bit4 arme, CALYPSO_SEED_52FD)
         * complete PROPREMENT a51c->a526->a529->a534->a537->a53c->a53f->a541->a544->a549
         * ->a582->b522->011e (confirme HANDLER-PATH). Puis cycle 2+ tombe dans une boucle
         * 0x71d7<->0x71db (146x observe) au lieu de refaire ce chemin. Cette sonde trace
         * CHAQUE passage (pas cappe a 1) pour voir EXACTEMENT ou/quand ca diverge entre
         * cycle 1 et cycle 2, + logge l entree dans le wrapper 0x71d3 (avant la boucle)
         * avec l etat cle (d[3f92], d[5a00], d[435b]=IMR-shadow, IMR reel). Cap 80/site. */
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
                    /* [2026-07-30] CORRECTIF DE SONDE : cette ligne imprimait
                     * `data[0x0810]` EN DUR alors que le BITF lit *AR1(0x0010).
                     * Tant que AR1 restait bloque a 0x0800 (page 0 latchee sur le
                     * dechet 0xf600 de d_dsp_page) les deux coincidaient. Depuis que
                     * le handshake de page est repare, AR1 alterne 0x0800/0x0814 et la
                     * cellule REELLEMENT testee est 0x0810 ou 0x0824 — la ligne
                     * affichait donc la mauvaise page une fois sur deux. Ca m'a fait
                     * lire deux fois « d_ctrl_system = 0 » alors que la page active
                     * portait autre chose. On calcule desormais l'adresse comme le
                     * fait l'instruction, et on l'imprime. */
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
        /* [2026-07-23] CLUSTERB-8D21 (READ-ONLY) : cible CALLD jamais tracee avant, a
         * l'INTERIEUR du range correlateur (0x8d00-0x9000), appelee UNIQUEMENT par les
         * handlers task-type 4/6 (Cluster B). Desassemblage statique montre 2 RPTB/RPTBD
         * imbriques + T=0x18(24, tap-count-shaped) + adressage MAR indirect circulaire --
         * signature DSP signal-processing authentique (contraste net avec le cluster audio
         * c1fa/c27b et les utilitaires bitmask 8f7f/8f9d, tous deux ecartes). Cap 30. */
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
        /* [2026-07-23] BITF-000B-HIT (READ-ONLY) : les 2 BITF sur data[0x000b] trouves
         * dans le dispatcher background (0xdeb6: BITF *(0x000b),0x4000 bit14 ;
         * 0xdec2: BITF *(0x000b),0x2000 bit13). Hypothese user (screenshot
         * INTM-TRANS + "l'histoire des 11") : est-ce que ce cycle go-live qui
         * boucle sans jamais atteindre le correlateur attend un compteur/flag
         * en 0x000b que seul un vrai timing sequenceur TPU (les 11 tpu_enq_at(0)
         * de l1s_rx_win_ctrl, non modelise -- cf calypso_tpu.c) ferait progresser ?
         * Logge data[0x000b] AVANT execution (= ce que BITF va tester) aux deux
         * PC. Complement de WATCH-000B-WR (qui confirme si la cellule est meme
         * ecrite). Cap 40 chacun. */
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
        /* [2026-07-23] CLUSTERB-SITES (READ-ONLY) : les 3 sites de dispatch task-type
         * (0x8b01=task4/site2 = celui qui a tire une fois ; 0x8ac4=task3/site1 ;
         * 0x8b8c=task6/site3, tres probablement SB_DSP_TASK=6). Logge task-type courant
         * (d[0x4357]) + AR3 (attendu 0x2bc0 pour sites 2/3, pointeur I/Q). Cap 30/site. */
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
        /* [2026-07-23] TASKTYPE-SRC (READ-ONLY) : 0xa6e9 = STL A,*(0x4357), source du code
         * task-type interne qui pilote tout le dispatch Cluster B. Logge A pour identifier
         * l'evenement amont qui produit chaque valeur. Cap 40. */
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
        /* [2026-07-23] A546-HIT (READ-ONLY) : le seul BACC natif connu vers le bootstrap
         * 0xd247 passe par 0xa546 (LD d[0x3fe0],A ; BACC A), lui-meme gate par
         * BITF d[0x09bc],1 a 0xa544 (cf WATCH-09BC-WR). Confirme si ce chemin tire. */
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
        /* [2026-07-23] C1FA-ENTRY (READ-ONLY) : 0xc1fa est la SEULE cible CALA
         * jamais tracee du dispatch 0xa57c (LD d[0x3fd4],A ; CALA A), constante=0xc1fa
         * a chaque hit (confirme statique par CALA-TRACE). Jamais disassemble ni
         * instrumente jusqu'ici -- premiere sonde. Cap 20, dump prog[0xc1fa..+0x60]
         * au 1er hit pour desassembler offline sans dependre d'un futur pass statique. */
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
        /* [2026-07-23] CLUSTER-B-PROBE (READ-ONLY) : le workflow xref-scan a trouve un
         * chemin dans PROM0 NON pollue par le bootstrap GPRS (Cluster A/0x87ff) qui mene
         * vers 0x8f7f/0x8f9d (dans le range correlateur !) via un dispatcher per-item
         * 0x86d4-0x871c. Cap 20/site, verifie si ce chemin est jamais atteint nativement. */
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
        /* === SOFT-RESET-TRIGGER probe (2026-05-28) ===
         * SP-CATASTROPHE trace montre PC=0x7120 (boot init via notre override
         * 0xFF80) re-firing à insn=190M. C'est un soft-reset interne firmware.
         * Pour pinpointer le déclencheur : log toute arrivée à PC=0xFF80 ou
         * PC=0x7120 APRÈS insn > 100k (= silicon reset initial déjà passé).
         * Trail pc_ring[-16..-1] + SP/AR/IMR/IFR/INTM → on voit l'instr qui
         * a sauté ici. */
        if ((s->pc == 0xFF80 || s->pc == 0x7120) && s->insn_count > 100000) {
            /* Deeper trail probe — gated par CALYPSO_DEBUG=SOFT_RESET_TRAIL.
             * pc[-64..-1] permet de remonter ~64 instructions avant la
             * réception du soft-reset pour identifier le caller chain. */
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
        /* === PROM3-VISIT probe (2026-05-28) ===
         * Compte les visites du DSP aux entries SB-decode candidats :
         *   0x8167, 0x81ff, 0x82b8 (PROM3 dispatch SB candidates per session).
         * Log à la première visite uniquement (insn_count + caller via ring),
         * puis compteur silencieux. Si à la fin du run task=6 a fire 30×
         * mais aucune visite → bug dispatch (item 5). Si visites OK mais
         * sb_att=0 → bug demod plus profond. */
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
        /* === TOP-OF-LOOP SP CHOKEPOINT (fix 2026-05-24 v6 Claude web) ===
         * Le hook SP existant est en BAS de boucle (L7471). Toute
         * instruction qui sort tôt (goto unimpl, return, continue, handler
         * qui sort de la dispatch chain) bypasse le hook → l'écriture SP
         * a lieu mais n'est pas comptabilisée. Hier l'audit "tout passe
         * par s->sp" était correct sur les SITES d'écriture mais ne
         * vérifiait pas si le hook tourne pour ces instructions.
         *
         * Symptôme : 61 events captés vs descente attendue de 11k+ mots
         * → la descente passe par bypass(es). Fix : observer s->sp à un
         * CHOKEPOINT obligé (top de boucle), comparer avec la valeur de
         * l'itération précédente. Bypass-proof par construction : on
         * regarde la VALEUR à un point de passage, pas le SITE.
         *
         * Implementation : statics (persistent inter-c54x_run-calls). */
        {
            static uint16_t topgate_last_sp = 0;
            static uint16_t topgate_last_pc = 0;
            static uint16_t topgate_last_op = 0;
            static int      topgate_valid   = 0;

            if (topgate_valid && s->sp != topgate_last_sp) {
                /* Compte l'instruction PRÉCÉDENTE qui a changé SP, quelle
                 * que soit sa voie de sortie (early-exit, return, etc.) */
                sp_hist_account(topgate_last_pc, topgate_last_op,
                                topgate_last_sp, s->sp, s->insn_count);
            }

            /* Patch 3 rev 2 : bootstub-entry trigger (le bon signal post
             * rev 1). Détecte l'edge prev_pc ∉ bootstub → cur_pc ∈ bootstub
             * = le RET corrompu qui a sauté à 0x00XX. Capture verbose +
             * dump ring contenant ~4096 iters d'approche. */
            if (topgate_valid) {
                sp_ring_check_bootstub_entry(s,
                    topgate_last_pc, topgate_last_op, topgate_last_sp,
                    s->pc, s->sp, s->insn_count);
            }

            /* A provenance tracer (2026-05-25 v3, Claude web review).
             * Track A's last writer + dump at trigger PC. Resout fork
             * NMI-vs-A-divergence avant impl invasive. */
            a_track_init_lazy();
            if (topgate_valid) {
                a_track_iter(s, topgate_last_pc, topgate_last_op);
            }

            /* AR6 windowed snapshot (2026-05-25 v4) — disambigue AR6=0
             * (base divergence) vs AR6=0x16 (self-alias feedback) au PC
             * trigger. Env CALYPSO_AR6_AT_PC=0x821a + window. */
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

            /* MVPD overlay occupancy : lazy-init + dump-if-boot-phase-ended. */
            mvpd_trace_init_lazy();
            mvpd_trace_dump_if_due(s->insn_count);

            /* Correlator entry trace : detect edge prev_pc ∉ [0x8d00..0x8f80]
             * → cur_pc ∈ same range. Log full state (AR3/4/5 = buffer pointers
             * probables) au moment de l'entrée. Dump des reads accumulés
             * périodiquement (toutes 20 entrées) pour observer si pattern
             * se stabilise vs varie entre runs. */
            corr_trace_init_lazy();
            if (g_corr_trace_enabled > 0 && topgate_valid) {
                /* [2026-07-23] FIX : range obsolete 0x8f80 remplace par CORR_PC_HI
                 * (0x9000) -- ce duplicate ratait silencieusement les cibles Cluster B
                 * (0x8f9d/0x8fb8) trouvees par le workflow xref-scan. */
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
                    /* Dump tous les 20 entrées pour observer si addr lues
                     * stabilisent (correlator répète) ou varient. */
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
                /* IT C54x = transition far : save XPC inconditionnel (APTS
                 * == AVIS, zéro sémantique pile) + force page 0 pour le fetch
                 * du vecteur (sinon vecteur lu via XPC vivant = bug racine). */
                s->sp--;
                data_write(s, s->sp, s->xpc);
                s->st1 |= ST1_INTM;
                /* corrélation IRQ (revival dsp 2026-06-23) : ce site de replay
                 * in-loop posait g_last_intr_* nulle part → les sondes
                 * HIGHVEC/DISP rataient les IT rejouées. On les pose ICI aussi,
                 * fg_pc capturé AVANT que s->pc soit écrasé par le vecteur. */
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

        /* Push counter at PC=0xb906 (and other suspected push sites).
         * Logs at powers of 10 to track cadence. SP captured at hit. */
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
            /* [2026-07-23] TINT0 tick SYNC transitions INTM (intuition user) : a chaque
             * RSBX INTM (1->0, re-enable), le go-live/handler attend le prochain TINT0.
             * On rend TINT0 (vec20/bit4) pending -> pris immediatement quand INTM=0.
             * Gate CALYPSO_TINT0_MASTER. C'est la vraie cadence (par slot, pas par frame). */
            {
                static int _t0i = -1;
                if (_t0i < 0) _t0i = calypso_gate("CALYPSO_TINT0_MASTER", 0);
                static unsigned _t0period = 0;
                if (_t0period == 0) { const char *_p = getenv("CALYPSO_TINT0_PERIOD"); _t0period = _p ? (unsigned)atoi(_p) : 1500; if (_t0period < 1) _t0period = 1500; }
                static unsigned _t0last = 0;
                /* [2026-07-23] THROTTLE : firer TINT0 a INTM 1->0 (prise propre) mais
                 * max 1x par _t0period insns (~cadence frame TDMA), sinon flood overlay
                 * a chaque micro-RSBX (63k/run) -> 200x lent. Sync transition + cadence. */
                /* [2026-07-23] TINT0 CEDE A BRINT0 : vec20(bit4) < vec21(bit5) en priorite
                 * -> si on fire TINT0 quand BRINT0 est pending, TINT0 gagne toujours la
                 * fenetre INTM=0 et AFFAME BRINT0 (livraison I/Q). On ne fire/arme TINT0
                 * QUE si BRINT0 (IFR bit5) n'est PAS pending -> BRINT0 sert l'I/Q d'abord.
                 * Sur vrai HW TINT0=cadence frame (rare), s'interleave avec BRINT0/burst. */
                /* [2026-07-23] FORCING RETIRE (hacky, cassait BRINT0). TINT0 vient
                 * maintenant du timer0 fidele (bloc TIMER0 tick) qui respecte l'IMR. */
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
            /* INT3-CYCLE-TRACE : fire end-good on ANY INTM 1→0 transition,
             * not just RETE — firmware uses POPM ST1 + RCD pattern. The
             * function itself is a no-op when probe disabled or no cycle
             * active, so unconditional call is safe. */
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

        /* SP-DRAIN probe (CALYPSO_DEBUG=SP-DRAIN) : attribue chaque
         * décrément net de SP à l'instruction qui vient de s'exécuter
         * (last_exec_pc/op — capturés en fin de boucle précédente).
         * Ces blocs tournent AVANT exec_one de l'itération courante, donc
         * s->sp reflète le résultat de l'insn précédente = last_exec_pc.
         * Isole l'instruction non-appariée qui draine SP dans le trampoline
         * boot 0x0000↔0xffcd. Histogramme 8-slots + log des 120 premiers
         * events. Silent par défaut. */
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

        /* CALLSITE probe (CALYPSO_DEBUG=CALLSITE) : à l'épilogue RCD 0x7707,
         * dump l'adresse de retour que RCD va popper + l'opcode du call-site
         * (FCALL F9xx vs CALL F074) + pc-ring pré-RETD = park-vs-crash. */
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

        /* XPC-WR tracer (CALYPSO_DEBUG=XPC-WR) : toute transition de XPC avec
         * l'instruction qui l'a causée (= origine du XPC=3 garbage). */
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

        /* AR2-WR tracer (CALYPSO_DEBUG=AR2-WR) : discrimine reset vs runaway.
         * delta==-1 = post-décrément normal (progression, log tous les 200).
         * delta!=-1 = reset/jump/load = LE discriminateur (#1 reset existe
         * vs #2 jamais de reset). Reporte BK + la cible du reset. */
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

        /* TRACE: dump entry into 0xe260 loop (first 5 hits) */
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
                /* Dump runtime opcodes 0xe255..0xe28f */
                char ob[1024]; int oo = 0;
                for (uint16_t a = 0xe255; a <= 0xe28f; a++) {
                    oo += snprintf(ob+oo, sizeof(ob)-oo, "%04x ", s->prog[a]);
                }
                C54_LOG("E260-PROG[e255..e28f]: %s", ob);
            }
        }

        /* CALA loop tracer: dump A and SP at PC=0xd24e and 0xd250 (first 40) */
        if (s->pc == 0xd24e || s->pc == 0xd250) {
            static int cala_log = 0;
            if (cala_log++ < 40) {
                C54_LOG("CALA-TRACE PC=0x%04x A=%08x SP=0x%04x BRC=%d AR2=%04x AR3=%04x AR4=%04x AR5=%04x insn=%u",
                        s->pc, (uint32_t)(s->a & 0xFFFFFFFF), s->sp, s->brc,
                        s->ar[2], s->ar[3], s->ar[4], s->ar[5], s->insn_count);
            }
        }

        /* PC histogram: count visits per PC, dump top 20 every 2M insns */
        {
            static uint32_t pc_hist[0x10000];
            static uint64_t hist_last_dump = 0;
            pc_hist[s->pc]++;
            if (s->insn_count - hist_last_dump >= 2000000) {
                hist_last_dump = s->insn_count;
                /* find top 20 */
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

        /* === Rolling PC sampler (v6 — find the REAL stuck zone) ===
         * The cumulative-since-boot PC HIST shows 0xa218..0xa222 dominant
         * because the init loop at 0xa222 (BANZD AR5, 60k iters) ran once
         * early. After that, the DSP moved on but the cumulative histogram
         * still shows those PCs at the top.
         *
         * BANZD-A222 traces (2026-05-08) confirmed AR5 was the actual loop
         * counter (61523→61499 in 25 iter), not AR1. Loop finishes in
         * ~984k insns (= 0.06% of a 1.7B run). Whatever IS currently
         * burning DSP cycles is in a different zone, invisible to the
         * cumulative top-N.
         *
         * Solution : rolling histogram per 100k-insn window. Resets each
         * window so we always see "what is the DSP doing RIGHT NOW".
         * Logs top-5 PCs of the most recent window. */
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

        /* === ENTER-RPTB-A218 probe (Q-BRC investigation 2026-05-08 v5+v6) ===
         * v5 hypothesis (BRC≈30770) was REFUTED by first 20 events :
         *   BRC=0 systematic, AR1=0 systematic, AR2 increments by 2,
         *   16 insns between visits.
         * v6 expands to capture the late-run behaviour : the cap=20 saturated
         * at insn=48M while the run reached 2.4B. We now have :
         *   (a) cap=200 for early events
         *   (b) periodic sampler at 100k-visits intervals (late-run)
         *   (c) BANZD-A222 probe to capture the actual AR used by the
         *       branch-back instruction at 0xa222 op=0x6e81.
         * The !s->rpt_active guard avoids spurious mid-RPTB hits. */
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
        /* === BANZD-A222 probe (v6) ===
         * 0xa222 op=0x6e81 + opnd 0x8208 = `BANZD pmad, *Sind`.
         * The *Sind operand decodes some AR but my v5 guess (AR1) was
         * unverified — capture all ARs so we see which one is non-zero
         * and how it evolves. If AR1=0 systematically, the branch test
         * uses a different AR. Cap=200, plus periodic 100k. */
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
         * 0xa215 op=0x4492 + 0xa216 opnd 0x0092 = `ADD/SUB Smem,16,dst` per
         * tic54x (2-word, mask FE00 base 0x4400). Logs A_pre / A_post and
         * the Smem read so we can trace what value lands in dst (may feed
         * BRC eventually). 30-event cap. */
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

        /* === XC-COND probe at PC=0xa0e0 / 0xa0e4 (Q1 hypothesis test) ===
         * Per Claude web v3 diag (2026-05-08) : routine 0xa0e0..0xa0e9 ends
         * at PC=0xa0e7 op=0xc8be where AR4 is consistently 0x18 (=MMR_SP)
         * pre-instruction → ST||LD writes to SP, catastrophe.
         *
         * Static dump shows two `XC 1, cond` instructions before 0xc8be :
         *   0xa0e0 = 0xfd30  ; XC 1, cond=0x30 (TC)
         *   0xa0e4 = 0xfd43  ; XC 1, cond=0x43 (ALT, A<0)
         *
         * Hypothesis : if XC condition evaluates to FALSE (TC bit not set, or
         * A not negative), the conditional STM #lk, AR4 (likely at 0xa0e5) is
         * SKIPPED → AR4 keeps stale value of 0x18 from earlier code path.
         *
         * Log every visit with : cond byte, TC/A/B flag values, AR4 value,
         * and the next opcode (which would be skipped or executed). If the
         * "taken" decision is consistently false at one of these XCs, that's
         * the bug. Cap to 100 events per PC. */
        if (s->pc == 0xa0e0 || s->pc == 0xa0e4) {
            static unsigned xc_log_e0;
            static unsigned xc_log_e4;
            unsigned *cnt = (s->pc == 0xa0e0) ? &xc_log_e0 : &xc_log_e4;
            if (*cnt < 100) {
                uint16_t op_xc = s->prog[s->pc];
                uint8_t  cond_byte = op_xc & 0xFF;
                uint16_t next_op   = s->prog[(uint16_t)(s->pc + 1)];
                /* Mirror the condition decode from c54x_exec_one (case 0xF
                 * XC handler around line 1108+) — only the common subset. */
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

        /* === MAC-8d33 trace — FB-det inner correlator ===
         * The DSP loops indefinitely in 0x8d2d..0x8d36. Static dump shows :
         *   8d2d 0x771a 0x0004      ; (2-word) — likely setup
         *   8d2f 0xf072 0x8d33      ; RPTB pmad, end=0x8d33 (per tic54x)
         *   8d31 0xf461             ; F46x = SFTA src,shift,dst (1-word)
         *   8d32 0xf591             ; F591 = ROL B (per our decoder)
         *   8d33 0xf3e2             ; F3E0-F3FF = SFTL src,SHIFT,DST  ← writes a_sync_SNR
         *   8d34 0x6e89 0x8d2d      ; BANZD pmad=0x8d2d, *AR — outer back-branch
         *   8d36 0xf3e1             ; SFTL B,1,B (exit path)
         * PC HIST counts (105k outer / 526k inner = 5×) confirm the 5-iter
         * RPTB body is (0x8d32, 0x8d33, 0x8d34) repeated 5 times.
         *
         * Capture A_pre, T, AR2..AR5 at each PC inside this zone. Rate-limit :
         *   first 50 always (init + early convergence)
         *   every 5000th (steady-state cadence)
         *   when |A_after - last_logged_A| > 0x100000 (significant accumulator
         *   shift = convergence event worth dumping)
         * Plus a dedicated "ENTER 0x8d2d" outer-iter counter that always logs
         * A_pre at the OUTER entry, so we can tell whether the accumulator
         * is reset between FB-det attempts (Observation 1 from session diag). */
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
        /* Dedicated outer-entry tracer at PC=0x8d2d : ALWAYS log A_pre on
         * entry (cap to 200 events). If A is non-zero on outer entry,
         * the accumulator wasn't reset between attempts — observation 1
         * from 2026-05-08 session : 21× 0x2fb0 SNR could mean stuck
         * accumulator across attempts. */
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

        /* === HOT-OPS PROBE for 0xe9ac..0xe9b7 + 0xe981..0xe983 ===
         * Diag v2 2026-05-08 : DSP locked in deterministic 7-instruction
         * loop at 0xe9ac..0xe9b7 (PROM1 mirror), with outer 3-PC loop
         * 0xe981..0xe983 reloading a BRC counter — pattern consistent
         * with `RPTB end_addr` + outer reset. We need the actual opcodes
         * to confirm/refute the RPTB hypothesis. One-shot dump on first
         * entry into the body range, with surrounding context (a few
         * words before for the RPTB instruction itself, and the outer). */
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
        /* === Plan B captures (c web review) : snapshot for transfer ring,
         * A-write ring, NOP-region guard. */
        uint16_t pre_pc  = s->pc;
        uint8_t  pre_xpc = s->xpc & 0x3;
        uint16_t pre_op  = prog_fetch(s, s->pc);
        int64_t  pre_a   = s->a;

        /* Trace EB04 loop — dump first 20 iterations */
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
                /* Dump code around current PC (using prog_fetch for correct OVLY) */
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

        /* BSP read entry points — these functions contain PORTR PA=0xF430
         * (read BSP sample). If DSP never visits them, the FB-det chain is
         * dead. Targets identified by static analysis of PROM0 callers of
         * the 64 PORTR PA=0xF430 sites at 0x9b80+. */
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

        /* Trace any write touching the dispatcher poll addresses
         * data[0x4359] / data[0x3fab]. We never see them go non-zero;
         * confirm whether ANY code path writes them. */
        /* (handled in data_write — see below) */

        /* Dispatcher hot loop trace at PROM0 0xb968-0xb9a4 — the state
         * machine the DSP spins in when waiting for ARM tasks. Logs the
         * first 8 visits per PC so we see the full conditional structure
         * (which addresses it polls, which constants it compares to). */
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

        /* IRQ vec area trace: log every PC visit in 0xFFCC-0xFFE0
         * (INT3 + TINT0 + BRINT0 vec slots). Captures the 3 actual
         * 4-word handlers our IRQ INT3 dispatch lands on at IPTR=0x1ff.
         * 80 unique PCs max, log first 4 visits each. */
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

        /* Trace DSP init - log once per unique PC in E900-E960 */
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

        /* Trace SINT17 handler (0x8a00-0x8a5f) */
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

        /* Sample PC every 1M instructions to find stuck loops */
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
        /* RPTB check moved below — must run AFTER `s->pc += consumed` so
         * that when the body's last instruction has executed and PC has
         * advanced to REA+1, the redirect to RSA is the FINAL operation
         * on PC for this iteration. The previous placement (before PC
         * advance) caused a 1-instruction off-by-one : redirect set
         * pc=RSA, then `s->pc += consumed` bumped it to RSA+1, so the
         * first body instruction was never re-executed across iterations
         * (PC HIST showed body=[RSA+1..REA+1] instead of [RSA..REA]). */

        /* Trace the IMR loop: how does the DSP reach 0x03F0? */
        /* Trace RPTB entry at 0x76FD: dump all AR values */
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

        /* Boot trace */
        if (g_boot_trace > 0) {
            C54_LOG("BOOT[%d] PC=0x%04x op=0x%04x SP=0x%04x PMST=0x%04x",
                    51 - g_boot_trace, s->pc, prog_fetch(s, s->pc), s->sp, s->pmst);
            g_boot_trace--;
        }

        /* Execute instruction */
        int consumed;
        uint16_t exec_pc = s->pc;
        /* [2026-07-22] TERMINAL-DISP : au tremplin 0xb40e (LD *AR7,A) / 0xb40f (BACC A),
         * quel slot est lu ? AR7 = l index de dispatch. data[AR7] = handler choisi
         * (0xab38 idle = storm). data[0x43c0] = le pointeur go-live (0xa4c7) VOISIN.
         * Montre si le terminal lit le mauvais slot (0x4387 idle au lieu de 0x43c0). */
        /* [2026-08-22] FIRS-BANK (CALYPSO_FIRS_BANK, defaut OFF) — le banc de
         * filtres polyphase de PROM0 est-il entre par son PROLOGUE ou au MILIEU ?
         * Huit FIRS en quatre paires (pmad 0x64,0x63,0x62,0x61), precedes d un
         * prologue qui charge AR2/AR5 <- 0x0060 puis d une table de dispatch de
         * six entrees en 0x8359..0x8363. Sur deux runs le premier FIRS execute
         * est TOUJOURS 0x8478 (dernier etage), et rien n ecrit le tampon
         * 0x0060..0x0066 qui est pourtant le Xmem du FIRS.
         * On compte le passage par chaque site : « jamais atteint » se distingue
         * ainsi de « atteint puis silencieux ». LECTURE SEULE. */
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
                    0x8336, 0x834b, 0x834f,                       /* prologue   */
                    0x8359, 0x835b, 0x835d, 0x835f, 0x8361, 0x8363, /* dispatch */
                    0x8365, 0x8394, 0x83c9, 0x83ff, 0x8435, 0x846b  /* etages   */
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
                {   /* bilan periodique : qui a ete atteint, qui jamais */
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
        /* [2026-08-22] CORR-SLIDE (CALYPSO_CORR_SLIDE, defaut OFF) — la fenetre
         * du correlateur glisse-t-elle ? distinct=3..4 sur 50 decalages alors que
         * le tampon de burst est vivant : les sorties viennent par plages
         * identiques. On regarde AR1..AR5 et les accumulateurs aux trois points
         * cles de chaque tour du RPTB 0x84b0..0x84c6. LECTURE SEULE. */
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
        /* [2026-07-22] FIX mask-ROM launch-vector (racine du storm, verifiee) :
         * le scheduler boot fait `BACC A(=0xab38 idle=RET)` @0xb40f ; le RET depile
         * mem[0x5ac8] = le VECTEUR DE LANCEMENT a la base de pile. Sur vrai HW ce mot
         * est pre-charge (mask-ROM absent du dump) ; en QEMU il vaut 0 -> RET->PC=0
         * -> storm. La bonne valeur = le pointeur go-live que le FIRMWARE LUI-MEME
         * a ecrit a data[0x43c0] (=0xa4c7 = `ORM #0x3000,IMR` = arm IMR). On la
         * derive (pas de constante magique) : mem[0x5ac8]=data[0x43c0] quand vide.
         * => RET idle saute a l'arm IMR -> storm mort ET IMR arme. Ni seed 0x71f4
         * (qui routait vers 0xa4df en SAUTANT l'arm IMR), ni poke arbitraire.
         * Gate CALYPSO_MASKROM_GOLIVE_OFF=1 pour reproduire le storm brut (A/B). */
        /* [2026-07-23] OVLY-TRACE : le handler frame 0x013b..0x0160 (overlay DARAM)
         * derail au RET 0x0157 (pile vide). Trace pc/op/sp du 1er passage + dump du
         * contenu overlay pour decoder ou est le desequilibre (PSHM non d-POPM /
         * branche prise a tort avant les POPM). One-shot (1 frame). */
        if (exec_pc >= 0x0100 && exec_pc <= 0x0160) {
            static unsigned ot = 0; static int dumped = 0;
            if (!dumped) {
                dumped = 1;
                fprintf(stderr, "[c54x] OVLY-DUMP data[0x0100..0x0160]:");
                for (int a = 0x0100; a <= 0x0160; a++) fprintf(stderr, " %04x", s->data[a]);
                fprintf(stderr, "\n");
            }
            /* [2026-07-23] TEST gated CALYPSO_TEST_3FCD : le RET@0x0157 saute a
             * data[0x3fcd]=0 (jamais ecrit). Le firmware installe un handler a
             * data[0x3fce]=0xdf82 (voisin +1). Test : au PSHD (0x0154), si
             * data[0x3fcd]==0, le derive de data[0x3fce] -> RET saute au handler.
             * Prouve/refute que 0xdf82 est la bonne cible (table vecteurs decalee). */
            if (exec_pc == 0x0154) {
                /* @BEQUILLE — TEST_3FCD  (CALYPSO_TEST_3FCD, EXISTS, defaut OFF)
                 *   masque  : data[0x3fcd] (adresse depilee par le RET @0x0157) n'est jamais
                 *             ecrite ; le firmware installe un handler au voisin data[0x3fce].
                 *             On derive l'un de l'autre.
                 *   retirer : des que la table de vecteurs overlay est installee au bon offset
                 *             (data[0x3fcd] non nul sans forcage) — ou immediatement si FIX_3FCD
                 *             (meme cellule, PC 0x013b) est retenu comme mecanisme unique.
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
        /* [2026-07-29] La bequille FIX_DPAGE_OFF a vecu ici (du 23 au 29/07). Elle
         * recopiait data[0x08E2] -> data[0x08d4] juste avant les deux lectures ROM
         * (0xa51c, 0xc8ea) pour compenser « l'offset +0x0E ». Sa condition de
         * retrait, ecrite dans son propre en-tete, etait : « des que l'offset est
         * corrige a la source ». C'est fait — voir calypso_fbsb.h :
         * d_dsp_page = 0x08D4, 0x08E2 = d_dsp_state.
         *
         * Deux choses valent d'etre retenues de sa disparition :
         *   - elle n'a JAMAIS pu marcher : elle ecrivait data[], alors que la ROM
         *     lit l'API RAM pour toute la plage 0x0800+ (cf le read path). Aucune
         *     ligne FIX-DPAGE n'a d'ailleurs ete emise au run du 29/07 19:26 ;
         *   - le chemin ARM principal, lui, etait deja correct (api_ram==dsp_ram,
         *     ARM 0x01A8 -> mot 0x08D4). Le desaccord ne venait que des ecrivains
         *     secondaires (shunt, bsp, arm2dsp) et des sondes.
         */
        /* [2026-07-23] TEST INIT-435B (gate CALYPSO_INIT_435B defaut ON) : data[0x435b]
         * = shadow IMR (les handlers tache OR/AND-ent des bits dedans : corr 0xbd3c
         * ORM 0x10, etc.), et la SM go-live 0xa501/0xa582 le propage dans IMR. Il n est
         * JAMAIS initialise en QEMU (STATE435B-WR vide) -> IMR=0 -> deadlock. On l amorce
         * au masque IMR reset 0x52fd (comme le boot DSP reel devrait) une fois. Si ca
         * arme IMR=0x52fd -> frame IT prise -> corr tourne -> self-sustain -> PROUVE. */
        if (exec_pc == 0xa4e4) {
            /* @BEQUILLE — INIT_435B (+ SEED_52FD)  (CALYPSO_INIT_435B_OFF=0 => ACTIVE ;
             *              CALYPSO_SEED_52FD choisit la valeur ; les 4 profils .env posent 0)
             *   masque  : l'initialisation du shadow IMR data[0x435b] par le boot DSP. Jamais
             *             ecrit en QEMU -> la SM 0xa582 propage IMR=0 -> deadlock. On injecte
             *             0x52ed (ou 0x52fd avec SEED_52FD) a exec_pc==0xa4e4.
             *   retirer : quand une ecriture firmware sur 0x435b est observee avant 0xa4e4.
             *   PIEGE   : le nom dit _OFF mais "=0" ACTIVE.
             */
            static int i435 = -1;
            if (i435 < 0) { const char *_e435 = getenv("CALYPSO_INIT_435B_OFF"); i435 = (_e435 && atoi(_e435)) ? 0 : 1; }  /* [2026-07-23] fix gate: teste VALEUR (OFF=0 => actif) */
            if (i435 && s->data[0x435b] == 0) {
                static unsigned in = 0;
                if (in++ < 4)
                    fprintf(stderr, "[c54x] INIT-435B: data[0x435b] 0x0000 -> 0x52ed (masque IMR reset SANS bit4/clobber) insn=%u\n", s->insn_count);
                s->data[0x435b] = getenv("CALYPSO_SEED_52FD") ? 0x52fd : 0x52ed;   /* [2026-07-23] defaut 0x52ed (SANS bit4/TINT -> evite le clobber firmware 0xa509 qui strippe bit12/frame). 0x52fd=bit4 opt-in (casse le frame, prouve : firmware n utilise PAS TINT0) */
            }
        }
        /* [2026-07-23] SM-TRACE : chemin complet de la SM go-live 0xa4e4-0xa5b5
         * avec d_dsp_page aligne -> ou branche/reboucle-t-elle ? flags decisifs :
         * d_dsp_page(0x3fb0), data[0x09bc](flag ARM), A(target dispatch). */
        if (exec_pc >= 0xa4e4 && exec_pc <= 0xa5b8) {
            static unsigned st = 0;
            if (st++ < 70)
                fprintf(stderr, "[c54x] SM-TRACE pc=0x%04x op=0x%04x A=0x%06llx TC=%d "
                        "d[3fb0]=%04x d[09bc]=%04x d[3fe0]=%04x d[435b]=%04x insn=%u\n",
                        exec_pc, prog_fetch(s, exec_pc), (unsigned long long)(s->a & 0xFFFFFFULL),
                        (s->st0 & ST0_TC) ? 1 : 0, s->data[0x3fb0], s->data[0x09bc],
                        s->data[0x3fe0], s->data[0x435b], s->insn_count);
        }
        /* [2026-07-23] TERM-TRACE : calcul d'index AR7 au terminal mask-ROM 0xb405-0xb412.
         * Question : AR7 devient 0x4387 (idle) au lieu de 0x43c0 (go-live) ? le calcul
         * LD#0x39 (0xb408) + ADD#0x4387 (0xb409) -> A=0x43c0 est-il perdu / mal-range dans AR7 ?
         * Logge A + AR0-7 a CHAQUE insn de la zone. Gate CALYPSO_TERM_TRACE_OFF. */
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
        /* [2026-07-23] CALA-TRACE-WIDE : ELARGI (recommande workflow xref-scan) sur
         * TOUTE la plage 0xa575-0xc300 (au lieu de fragments) et TOUS les transferts
         * calcules (CALA/CALAD/BACC/FBACC/FCALA/FCALAD -- f4e2/f4e3/f4e6/f4e7/f5e2/f5e3/
         * f5e6/f5e7/f6e6/f6e7), pas seulement CALA. Strategie empirique : capter N'IMPORTE
         * QUEL saut calcule qui atterrit dans le range correlateur (0x8d00-0x9000), plutot
         * que continuer le tracage statique exhaustif (3 workflows n'ont pas trouve la
         * reference statique). Deux compteurs separes : hits "dans le range" (JAMAIS
         * cappes, signal fort) et hits generaux (cap 200, pour contexte/pattern). */
        {
            static int _ctw = -1;
            if (_ctw < 0) { const char *_e = getenv("CALYPSO_D247_TRACE_OFF"); _ctw = (_e && atoi(_e)) ? 0 : 1; }
            if (_ctw && exec_pc >= 0x7000 && exec_pc <= 0xdfff) {   /* [2026-07-23] ELARGI a tout PROM0 (plus de limite arbitraire) */
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
        /* [2026-07-23] INSTALL-TRACE : le bloc 0xc7xx installe la table de handlers de tache
         * (STL A -> d[4c5c] a 0xc803). d[4c5c]=0 -> corr FB jamais dispatche. Ce bloc est-il
         * atteint, et A vaut quoi a 0xc803 ? Litteraux voisins (d[4c5a]/d[4c5d]) = bloc atteint ?
         * Gate CALYPSO_INSTALL_TRACE_OFF. */
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
        /* [2026-07-23] BACC-C827-SRC : d'OU vient le saut vers 0xc827 (qui skippe l'install
         * de la table de handlers 0xc7a0-0xc825) ? Traque prev_pc + op + A + AR quand on entre
         * a 0xc827 sans fall-through (prev != 0xc825/0xc826). Gate CALYPSO_BACC_C827_OFF. */
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

/* [2026-07-23] PHASE-SM : la state-machine d[3f70] (phase go-live->operationnel).
         * 0xddeb LD d[0x098a];BC si A==0 -> reset phase=0. 0xde86 LD d[0x098c]. 0xde9c ST#2.
         * d[0x098a]/d[0x098c] = handshake ARM (l'ARM DOIT les poser !=0 pour avancer -> d[3f70]=2).
         * Logge les points de decision + valeurs. Gate CALYPSO_PHASE_SM_OFF. */
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
                  /* SONDE [2026-07-29] — adresse EFFECTIVE du LD.
                   * Sur c54x l'adressage direct est DP-relatif : dma = (DP<<7)|offset7.
                   * À 0xde86 le commentaire annonce « LD d[0x098c] », la cellule vaut 1
                   * (semée par BGEN) et pourtant A ressort à 0x0000, 38 fois de suite.
                   * On imprime DP, l'opcode et l'adresse calculée pour trancher entre
                   * « DP pointe une autre page » et « bonne page, cellule nulle ».
                   * Diagnostic pur : aucune écriture, aucun changement de comportement. */
                  {
                      uint16_t _op  = prog_fetch(s, exec_pc);
                      uint16_t _lk  = prog_fetch(s, (uint16_t)(exec_pc + 1));
                      int      _ind = (_op & 0x80) ? 1 : 0;
                      int      _mod = (_op >> 3) & 0x0F;
                      /* mod 0xF = *(lk) : l'adresse EST le mot long. Pour tout
                       * autre mode on imprime quand même lk, ça ne coûte rien et
                       * ça évite de re-supposer. */
                      static unsigned _ean = 0;
                      /* PLAFOND : sans lui cette sonde noie le journal (480 898
                       * lignes mesurées) et provoque la troncature qui efface
                       * les autres. 40 lignes puis une toutes les 100 000. */
                      _ean++;
                      if (_ean <= 40 || (_ean % 100000) == 0)
                      fprintf(stderr, "[c54x] PHASE-SM-EA #%u pc=0x%04x op=0x%04x ind=%d mod=0x%x "
                              "lk=0x%04x d[lk]=0x%04x AR0=0x%04x A=0x%04x insn=%u\n",
                              _ean, exec_pc, _op, _ind, _mod, _lk, s->data[_lk],
                              s->ar[0], (uint16_t)(s->a & 0xFFFF), s->insn_count);
                  }
            }
        }
        if (exec_pc == 0xa51c) {   /* SM go-live lit d_dsp_page @0x08d4 (faux ?) */
            static unsigned dp=0;
            if (dp++ < 12)
                fprintf(stderr, "[c54x] SM-DPAGE @0xa51c data[0x08d4]=0x%04x data[0x08E2]=0x%04x "
                        "data[0x435b]=0x%04x A=0x%06llx insn=%u\n",
                        s->data[0x08d4], s->data[0x08E2], s->data[0x435b],
                        (unsigned long long)(s->a & 0xFFFFFFULL), s->insn_count);
        }
        if (exec_pc == 0xa4cd) {   /* [2026-07-23] BC AEQ (0xf845) : pourquoi A==0 -> skip RSBX INTM (0xa4d0) ?
                                    * 0xaad5 lit AR0=data[0x434e], AR1=data[0x434f] (ptrs) et calcule A. */
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
            /* [2026-07-23] DÉFAUT OFF : le storm est tué NATIVEMENT par le fix ISA LD #k8u
             * (calypso_c54x.c:7702, le <<16 mettait AR7=0x4387 idle au lieu de 0x43c0 go-live).
             * Ce hack (mem[0x5ac8]=data[0x43c0]) ne faisait que masquer ce bug -> plus nécessaire.
             * Opt-in CALYPSO_MASKROM_GOLIVE=1 pour le réactiver (A/B). */
            /* @BEQUILLE — MASKROM_GOLIVE  (CALYPSO_MASKROM_GOLIVE, EXISTS, defaut OFF)
             *   masque  : le vecteur de lancement mem[0x5ac8] a la base de pile, pre-charge
             *             par un mask-ROM absent du dump ; a 0, le RET du BACC idle saute a
             *             PC=0 (storm).
             *   retirer : DEJA INUTILE selon le commentaire ci-dessus — le fix ISA LD #k8u
             *             tue le storm nativement. A supprimer au prochain passage, ce n'est
             *             plus qu'une garde A/B.
             *   NB      : le commentaire amont annonce CALYPSO_MASKROM_GOLIVE_OFF — inexistant.
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
        /* [2026-08-04] FIX_RPT_COUNT, patte 1/2 : etat du repeat AVANT execution.
         * Sert a distinguer « le repeat etait deja actif » de « l'instruction
         * qu'on vient d'executer EST le RPT qui vient de l'armer ». Voir la
         * patte 2/2 dans le bloc RPT en fin de boucle. */
        bool rpt_was_active = s->rpt_active;
        /* === FBWATCH-ALIVE (canary) : PROUVE que la sonde est armée + sample PC
         * foreground. Si CE log sort, g_fbwatch_on=1 et le silence des autres
         * FBWATCH est RÉEL. S'il ne sort PAS, les probes étaient mortes. Fire
         * garanti tous les ~20M insns (≈10 lignes sur le run). === */
        if (g_fbwatch_on > 0 && (s->insn_count % 20000000u) == 0) {
            fprintf(stderr, "[c54x] FBWATCH-ALIVE insn=%u PC=0x%04x INTM=%d SP=0x%04x\n",
                    s->insn_count, exec_pc, !!(s->st1 & ST1_INTM), s->sp);
        }
        /* === FBWATCH-POLL : le foreground polle un flag via BITF @0xf7af/0xf7b7
         * (RC NTC = boucle tant que TC=0). Capture l'adresse du flag (AR0..AR2 +
         * data) + TC pour ID le bit jamais posé = ce qu'il faut câbler (modèle HW). */
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
        /* === FBWATCH (2) : le handler FB 0x9ac0 tourne-t-il ? (env one-shot) === */
        if (g_fbwatch_on > 0 && exec_pc == 0x9ac0) {
            static unsigned w9 = 0;
            if (w9++ < 40)
                fprintf(stderr, "[c54x] FBWATCH-9AC0 #%u insn=%u SP=0x%04x DP=0x%03x\n",
                        w9, s->insn_count, s->sp, s->st0 & 0x1FF);
        }
        /* === FBWATCH-INITTAB : la routine d'init de la table de dispatch
         * (0xc704, peuple data[0x4c24-0x4c5d] = cibles BACC-A/CALA) tourne-t-elle ?
         * 0 hit = jamais atteinte = root confirmé (boot saute le setup-pass). */
        if (g_fbwatch_on > 0 && (exec_pc == 0xc704 || exec_pc == 0xc472)) {
            static unsigned wit = 0;
            if (wit++ < 10)
                fprintf(stderr, "[c54x] FBWATCH-INITTAB pc=0x%04x insn=%u SP=0x%04x\n",
                        exec_pc, s->insn_count, s->sp);
        }
        /* === FBWATCH (4) PRODUCTEUR/CONSOMMATEUR : le dispatch CALAD @0x833b
         * tourne-t-il par-frame, et quelle adresse handler calcule-t-il dans A ?
         * 0 ligne = dispatcher mort (producteur). A jamais 0x9ac0 = la jump-table/
         * formule ne produit jamais le handler FB. A=0x9ac0 = FB dispatché mais
         * ne détecte pas (bug handler). Cap haut pour voir la distribution. */
        if (g_fbwatch_on > 0 && exec_pc == 0x833b) {
            static unsigned wdp = 0;
            if (wdp++ < 120)
                fprintf(stderr, "[c54x] FBWATCH-DISP #%u insn=%u A_handler=0x%04x DP=0x%03x SP=0x%04x\n",
                        wdp, s->insn_count, (uint16_t)(s->a & 0xffff), s->st0 & 0x1FF, s->sp);
        }
        /* CORR-ENTRY tracker (env CALYPSO_CORRELATOR_TRACE=1) : capture
         * transition out→in du range FB-det [0x8d00..0x9000). Cf top of
         * file pour la lazy-init + l'évidence runtime 2026-05-25 night. */
        corr_entry_track(s->pc, s);
        /* FBDB-PROBE (env CALYPSO_FBDB_PROBE=1, c web reframe 2026-05-25 night2) :
         * trace B@fbd9, A@fbdb (= post F2xx SUB), A@fbf3 (= before STLM A,AR4). */
        fbdb_probe_check_pc(s->pc, s);
        /* FORCE-INTM-ONESHOT (env CALYPSO_FORCE_INTM_ONESHOT=1, c web reframe
         * 2026-05-25 night4) : sonde arbitrage — clear INTM UNE FOIS quand
         * INTM=1 + BRINT0 pending. Observe via tracers existants si aval sain. */
        force_intm_oneshot_check(s);
        /* STUCK-PROBE (env CALYPSO_STUCK_PROBE=1, c web reframe 2026-05-25 night3) :
         * capture PC+XPC histogramme quand INTM=1 + BRINT0 pending. */
        stuck_probe_check(s);

        /* === CALA-70C3 FORENSIC PROBES (2026-05-27, c web review) ===
         * Pourquoi : DSP boucle infiniment sur CALA A à PROM0[0x70c3] avec
         * A=0x0001_70c3 (auto-référence). A_H=0x0001 ne peut PAS venir d'un
         * `LD Smem,A` sext40-é (qui donne A_H ∈ {0x0000, 0xFFFF}), donc
         * writer = DLD upstream ou compose H+L. Probes pour identifier :
         *   1. Source du jump vers 0x70c3 (XPC:PC + opcode@prev_pc), gated
         *      FIRST-HIT pour échapper à la pollution post-runaway (MMR XPC
         *      écrasé quand SP rampage à travers data[0x18..0x1F]).
         *   2. Compteur LD@0x70c1 — si 0, confirme le jump direct (skip LD).
         *   3. Dernier writer de A (PC qui a posé 0x0001_70c3 dans A).
         * Active par défaut, coût ~3 branches/insn. */
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
            /* DISP-ENTRY : prédécesseur = PC exécuté à l'itération précédente */
            static uint16_t s_last_run_pc = 0;
            static uint16_t s_last_run_op = 0;
            g_prev_pc = s_last_run_pc;
            g_prev_op = s_last_run_op;
            s_last_run_pc = s->pc;
            s_last_run_op = exec_op;
        }

        /* SONDE GAP-1 AR3-TRIP (2026-06-23, approche structurée non-decode) :
         * pince la PREMIERE instruction qui fait sauter AR3 d'un GRAND pas.
         * A ce point s->ar[3] reflete le resultat de l'instruction qui vient
         * de tourner = g_prev_pc/g_prev_op. Les post-incr legitimes valent +-1/2 ;
         * un saut >= 0x800 = chargement/modif suspecte. Flag special si delta==SP
         * (la signature "AR3 += SP" qu'on a identifiee comme cause du derail).
         * Cout : 1 sub + 1 cmp / insn, log cape a 80. */
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

        /* SONDE GAP-1 AR0-TRACE (2026-06-23) : AR0 est l'INDEX du *AR3+0% a
         * 0xb3d1 (AR3 += AR0). On a etabli qu'AR0 ~= 0x5AC7 (~SP) = corrompu.
         * Logge CHAQUE changement d'AR0 dans la fenetre init (insn<3000) avec
         * l'instruction qui l'a pose (g_prev_pc/op) -> nomme le corrupteur. */
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

        /* Dump one-shot du contexte complet au 1er passage a 0xb3d1
         * (ADD *AR3+0%,A) : AR0/AR3/BK/DP/ST1/A + data[AR3]. */
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

        /* === SONDE DETECTOR-TRACE (Phase B, 2026-06-23) ===================
         * Trace instruction-par-instruction du detecteur FB / correlateur dans
         * [0xf074..0xf0c0]. La region 0xf070-0xf0b0 EST une table sigmoide en
         * XPC=0 (dump ROM) ; l'execution reelle est dans une AUTRE banque XPC.
         * exec_op = prog_fetch(s->pc) respecte la banque -> on voit le VRAI
         * opcode. On logge PC + XPC (= quelle banque) + op + A/B/T (le math de
         * correlation : sample*coeff accumule). Si A/B restent 0 -> le math ne
         * produit rien (opcode mal emule / sample non lu). Si A/B montent mais
         * d_fb_det reste 0 -> bug de seuil/decision (croiser avec
         * CALYPSO_FBDET_SENTINEL=2 qui monitore les writes a 0x08f8).
         * Env CALYPSO_DETTRACE=1, cap 800. */
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

        /* SONDE Phase B BACC-IN : au bacc dispatch (0xb40f) et au LD *AR7,A
         * (0xb40e) qui le precede, capture A + AR7 + data[AR7] = la source du
         * pointeur 0xf074. Dit si le slot lu (data[AR7]) vaut deja 0xf074
         * (pointeur faux) ou si A est corrompu autrement. Cap 40. */
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

        /* SONDE Phase B DISP-ENTRY (trisection 2026-06-24) : trace one-shot
         * l'entree du dispatcher [0xb400..0xb40f] -> comment AR7 obtient sa
         * valeur (immediat litteral vs registre d'index/evenement gele), le
         * reset SP delibere (0xb403), et la pile au bacc. AR3/AR4 inclus (test
         * "AR7 derive d'un registre gele"). Cap 64. */
        /* SONDE Phase B LOOPTRACE (2026-06-24, élargie depuis DISP-ENTRY) :
         * la maladie n'est PAS l'idle bénin -> POST-BOOTSTUB-RET tourne 640M de
         * fois (storm PC=0) dès le 1er dispatch (insn 4398). On veut la boucle
         * principale ENTIÈRE [0xb3c0..0xb410] (= prologue lecture d_dsp_page @0xb3cc
         * + dispatcher @0xb400) AVEC le caller (g_prev_pc) -> voir comment 0xb401
         * est atteint la 1ère fois, si word[0]=0x2900 @0xb400 est exécuté ou sauté,
         * et pourquoi data[SP] (seed de retour) est vide quand l'idle-RET dépile
         * -> PC=0. AR1 inclus (la copie READA arme AR1=0x4387). Cap 300. */
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

        /* === EXPERIENCE SEED-5AC8 (2026-06-24) : le chainon go-live ===========
         * MECANISME VERIFIE sur le vrai dump : la frame body seme le soft-vector
         * data[0x3f6d]=0xa4df @0xb405 (continuation GO-LIVE). Le trampoline
         * 0x71f4 = `LD *(0x3f6d),A ; BACC A` honore ce vecteur -> 0xa4df ->
         * 0xa4ca/0xa500 -> 0xa51b RSBX INTM (1er enable IT du run) -> 0xa51c lit
         * d_dsp_page. MAIS le terminal 0xb40f BACC data[0x4387]=0xab38=RET depile
         * mem[0x5ac8]=0 -> PC=0 -> storm, au lieu d'atteindre 0x71f4.
         * ATTENTION (2026-07-22) : LE SEED N'EST PAS UN FIX. C'est un band-aid GATE
         * (OFF par defaut, CALYPSO_SEED5AC8=1 pour l'activer). Poker mem[0x5ac8]
         * MASQUE le vrai bug : verifie sur ROM (PROM0.bin, LE) le boot @0xb405 fait
         * `ST #0xa4df, data[0x3f6d]` -> le soft-vector go-live vaut 0xa4df, qui SAUTE
         * l'arm IMR (0xa4c7 `ORM #0x3000,IMR`) ET l'enable (0xa4d0 `RSBX INTM`). Donc
         * meme mem[0x5ac8]=0x71f4 "correct" n'arme jamais l'IMR. La vraie cause du
         * storm ET du no-enable reste a trouver (pourquoi 0xa4c7/0xa4d0 jamais
         * atteints ; qui doit peupler mem[0x5ac8]). NE PAS traiter le seed en fix. */
        {
            /* @BEQUILLE — SEED_5AC8 (+ SEED5AC8_VAL)  (CALYPSO_SEED5AC8, atoi>0, defaut OFF ;
             *              _VAL defaut 0x71f4, calypso_wire.env:=0xa4c7)
             *   masque  : le peuplement de mem[0x5ac8] (mot depile par le RET terminal 0xab38,
             *             qui choisit l'entree go-live). Personne ne l'ecrit dans notre modele.
             *   retirer : quand on sait QUI ecrit mem[0x5ac8] sur silicium — le commentaire du
             *             bloc dit deja "NE PAS traiter le seed en fix".
             */
            static int seed_on = -1;
            /* GATE : seed OFF par defaut (band-aid), ON seulement si CALYPSO_SEED5AC8=1. */
            if (seed_on < 0) { const char *e = getenv("CALYPSO_SEED5AC8"); seed_on = (e && atoi(e) > 0) ? 1 : 0; }
            /* [2026-07-22] CABLE sur le STM #0x5ac8,SP du DSP (PC=0xb382) : le
             * seed est desormais SOURCE du stack-init REEL du DSP (op=0x7718),
             * pas d'un poke arbitraire au BACC terminal. Timing sur : aucune
             * ecriture ne touche 0x5ac8 entre 0xb382 et le RET (0xab38). */
            if (seed_on && exec_pc == 0xb382) {
                /* [2026-07-22] valeur du seed configurable : le RET terminal depile
                 * mem[0x5ac8] pour choisir l entree go-live. 0x71f4 (defaut) ->
                 * trampoline -> 0xa4df (SAUTE l enable RSBX INTM 0xa4d0). 0xa4c7 ->
                 * entree par l ORM IMR -> RSBX INTM 0xa4d0 = enable natif -> la frame
                 * IT (proprement livree bit12) est alors PRISE. CALYPSO_SEED5AC8_VAL. */
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
        /* GOLIVE-WATCH (ungated) : le firmware atteint-il enfin la routine go-live
         * (0xa4c9..0xa520) ou le trampoline 0x71f4 ? logge PC/op/INTM/IMR +
         * soft-vector data[0x3f6d]. Cap 120. */
        if ((exec_pc >= 0xa4c9 && exec_pc <= 0xa520) || exec_pc == 0x71f4 || exec_pc == 0x71f6) {
            static unsigned gw = 0;
            if (gw++ < 120)
                fprintf(stderr, "[c54x] GOLIVE-WATCH #%u PC=0x%04x op=0x%04x INTM=%d "
                        "IMR=0x%04x data[0x3f6d]=0x%04x A=0x%06llx insn=%u\n",
                        gw, exec_pc, exec_op, (s->st1 & ST1_INTM) ? 1 : 0, s->imr,
                        s->data[0x3f6d], (unsigned long long)(s->a & 0xFFFFFF), s->insn_count);
        }
        /* [2026-07-22] AR0-DELTA (gated CALYPSO_AR0_DEBUG, RO) : chaque changement
         * d'AR0 dans la fenetre boot -> localise l'instruction qui corrompt AR0
         * (attendu ~0x5ac8 pour ecrire le vecteur go-live mem[0x5ac8]=0x71f4). */
        {
            static int ad_en = -1;
            static uint16_t ar0_prev = 0xFFFF;
            if (ad_en < 0) ad_en = calypso_gate("CALYPSO_AR0_DEBUG", 0);
            if (ad_en && s->insn_count < 12000 && s->ar[0] != ar0_prev
                && exec_pc != 0xb387) {   /* skip le fill-loop qui noie le cap */
                static unsigned adn = 0;
                if (adn++ < 200)
                    fprintf(stderr, "[c54x] AR0-DELTA 0x%04x->0x%04x by PC=0x%04x "
                            "op=0x%04x AR3=0x%04x insn=%u\n",
                            ar0_prev, s->ar[0], exec_pc, exec_op, s->ar[3], s->insn_count);
                ar0_prev = s->ar[0];
            }
        }
        /* [2026-07-22] PROG-DUMP-B3D0 (gated CALYPSO_AR0_DEBUG, RO, one-shot) :
         * desassemble la region qui seed data[0x3f6d]=0xa4df (@0xb405) et le
         * BACC terminal 0xb40f, pour trouver le setup companion de mem[0x5ac8]. */
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
                /* et le RET 0xab38 (cible du BACC) */
                fprintf(stderr, "[c54x] PROG[0xab36..]= %04x %04x %04x %04x\n",
                        s->prog[0xab36], s->prog[0xab37], s->prog[0xab38], s->prog[0xab39]);
                /* CALL-site 0x71f2 (transfert -> 0xb3a3 sans push = LE bug) +
                 * trampoline 0x71f4. Doit etre un CALL empilant 0x71f4. */
                for (uint16_t a = 0x71ec; a <= 0x71f8; a += 4)
                    fprintf(stderr, "[c54x] PROG[0x%04x..]= %04x %04x %04x %04x\n",
                            a, s->prog[a], s->prog[(uint16_t)(a+1)],
                            s->prog[(uint16_t)(a+2)], s->prog[(uint16_t)(a+3)]);
                /* debut de routine cote 0xb3a3 (cible du transfert) */
                fprintf(stderr, "[c54x] PROG[0xb3a0..]= %04x %04x %04x %04x %04x %04x\n",
                        s->prog[0xb3a0], s->prog[0xb3a1], s->prog[0xb3a2],
                        s->prog[0xb3a3], s->prog[0xb3a4], s->prog[0xb3a5]);
                /* LE FILL : setup 0xb384-0xb38c (STM AR0, RPT #k, ST #imm *AR0+) */
                fprintf(stderr, "[c54x] PROG[0xb384..]= %04x %04x %04x %04x %04x %04x %04x %04x %04x\n",
                        s->prog[0xb384], s->prog[0xb385], s->prog[0xb386], s->prog[0xb387],
                        s->prog[0xb388], s->prog[0xb389], s->prog[0xb38a], s->prog[0xb38b], s->prog[0xb38c]);
                /* resultat du fill en memoire : constante + ou il s'arrete */
                fprintf(stderr, "[c54x] FILL-MEM data[0x5a00]=0x%04x [0x5ac5]=0x%04x [0x5ac6]=0x%04x "
                        "[0x5ac7]=0x%04x [0x5ac8]=0x%04x [0x5ac9]=0x%04x\n",
                        s->data[0x5a00], s->data[0x5ac5], s->data[0x5ac6],
                        s->data[0x5ac7], s->data[0x5ac8], s->data[0x5ac9]);
                /* [2026-07-22] routine go-live 0xa4c0-0xa4e4 : ORM 0xa4c7, test
                 * wait-loop 0xa4d4 (cellules 0x098a/0x098c), pour porter vers ARM. */
                /* store loop des vecteurs (0xb4c8-0xb4e0) + sa source (AR-setup) */
                /* routine 0xa9ea (CALL @0xb3f3, push retour 0xb3f5) : ou est
                 * son over-pop (PSHM/POPM desequilibre) qui derive SP -> storm. */
                for (uint16_t a = 0xa9ea; a <= 0xaa1a; a += 4)
                    fprintf(stderr, "[c54x] A9EA-PROG[0x%04x..]= %04x %04x %04x %04x\n",
                            a, s->prog[a], s->prog[(uint16_t)(a+1)],
                            s->prog[(uint16_t)(a+2)], s->prog[(uint16_t)(a+3)]);
                for (uint16_t a = 0xb4c8; a <= 0xb4e0; a += 4)
                    fprintf(stderr, "[c54x] VECLOOP-PROG[0x%04x..]= %04x %04x %04x %04x\n",
                            a, s->prog[a], s->prog[(uint16_t)(a+1)],
                            s->prog[(uint16_t)(a+2)], s->prog[(uint16_t)(a+3)]);
                /* go-live tail post-0xa582 : installe-t-il vec28 (write 0x00f0) ? */
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
        /* RUNTIME-DYN (2026-06-24, RO) : dynamique de l'automate qui garde le
         * scheduler 0xa51c (qui ne tourne jamais, data[0x3fb0]=0). Trois faits :
         *  (1) IMR-DYN : le DSP execute-t-il ses sites d'armement IMR, et qui efface ?
         *  (2) WAIT-TEST : le flag 0x3f70 bit1 (sortie wait-loop) est-il jamais set au test ?
         *  (3) DE97-BR : le super-loop atteint-il la branche set-bit1 (0xde9c) ? */
        if (exec_pc == 0x76fc || exec_pc == 0xa509 || exec_pc == 0xb37e) {
            static unsigned id = 0;
            if (id++ < 40)
                fprintf(stderr, "[c54x] IMR-DYN PC=0x%04x IMR_before=0x%04x writes=0x%04x "
                        "INTM=%d insn=%u\n", exec_pc, s->imr,
                        s->prog[(uint16_t)(exec_pc + 1)],
                        (s->st1 & ST1_INTM) ? 1 : 0, s->insn_count);
        }
        /* [2026-07-22] KEEP-IMR (gated CALYPSO_KEEP_IMR) : 0xb37e (STM #0,IMR)
         * efface IMR ~47 insns apres que le go-live l'a arme -> la wait-loop
         * tourne avec IMR sans les bits d'IT -> l'IT jamais prise.
         * [2026-07-25 FIX BIT5 — diag video user] : l'ancienne version re-armait
         * IMR=0x3000 (bits 12/13, vec28/29) mais SANS bit5 (BRINT0/vec21). Or
         * BRINT0 = l'IT "buffer BSP recu" qui reveille le correlateur. Resultat
         * mesure : IMR=0x52fd (bit5=1) arme 178x par le go-live puis ECRASE par
         * 0xb37e, KEEP_IMR restaurait 0x3050/0x0050 (bit5=0) -> BRINT0 masque a
         * vie -> IFR bit5 pending eternel -> correlateur jamais dispatche.
         * FIX : re-armer la VRAIE image = le shadow d[0x435b] (=0x52fd, bit5
         * inclus), et le faire des que bit5 TOMBE (pas seulement quand imr==0),
         * sur toute la region go-live+background [0xa4ca..0xdea0]. Valeur de
         * repli / override : CALYPSO_KEEP_IMR_VAL (defaut 0x52fd). */
        {
            /* @BEQUILLE — KEEP_IMR (+ KEEP_IMR_VAL)  (CALYPSO_KEEP_IMR, EXISTS, defaut 1 en
             *              hack/native/native_helped/wire ; valeur de repli 0x52fd)
             *   masque  : le clobber de l'IMR par 0xb37e (STM #0,IMR) et 0xa509 (strip bit12).
             *             On re-ecrit s->imr = data[0x435b] des que bit5/BRINT0 tombe, sur
             *             toute la region [0xa4ca..0xdea0].
             *   retirer : quand le firmware ne perd plus bit5 — c'est-a-dire quand la sequence
             *             go-live 0xa4c7/0xa51b/0xa582 se deroule dans le bon ordre.
             */
            static int ki = -1; static uint16_t kiv = 0;
            if (ki < 0) { ki = calypso_gate("CALYPSO_KEEP_IMR", 0);
                const char *e = getenv("CALYPSO_KEEP_IMR_VAL");
                kiv = (e && *e) ? (uint16_t)strtoul(e, NULL, 0) : 0x52fd; }
            if (ki && exec_pc >= 0xa4ca && exec_pc <= 0xdea0 && !(s->imr & 0x0020)) {
                uint16_t img = s->data[0x435b];            /* shadow IMR (=0x52fd) */
                if (!(img & 0x0020)) img = kiv;            /* shadow sans bit5 -> repli */
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
            /* FORCE-GOLIVE (etape1, gated CALYPSO_FORCE_GOLIVE) : release the
             * go-live wait-loop by setting data[0x3f70] bit1 at the 0xa4d4 test
             * (normally gated on control cells 0x098a/0x098c which the ARM leaves 0). */
            /* @BEQUILLE — FORCE_GOLIVE  (CALYPSO_FORCE_GOLIVE, atoi>0, defaut OFF ; hack.env vide)
             *   masque  : la wait-loop go-live teste data[0x3f70] bit1, pose seulement par le
             *             setter 0xde9c, lui-meme conditionne aux cellules 0x098a/0x098c que
             *             l'ARM laisse a 0.
             *   retirer : des que le handshake ARM (ARM2DSP_BGEN) fait franchir 0xddf5 et que
             *             le setter natif 0xde9c s'execute.
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

        /* [2026-07-25] RANK1 : route frame ISR 0x013b -> 0x8341 (LUT FB native,
         * setup COMPLET du correlateur : BRC/BK/data-ptr, que l'entree forcee
         * 0x8d00 court-circuite -> boucle morte MAC diag). Le frame IT vectorise
         * 0x00f0 -> 0x7234 (B 0x013b) -> prologue 0x013b qui deraille et n'atteint
         * jamais 0x8341. On redirige le prologue vers 0x8341 quand l'IT vient du
         * frame scheduler (g_prev_pc==0x7234). Gate CALYPSO_ISR_TO_8341 (def off). */
        if (exec_pc == 0x013b) {
            /* @BEQUILLE — ISR_TO_8341  (CALYPSO_ISR_TO_8341, EXISTS, defaut OFF)
             *   masque  : le prologue ISR overlay 0x013b deraille et n'atteint jamais la LUT
             *             FB 0x8341 (setup complet BRC/BK/data-ptr du correlateur). On force
             *             s->pc = 0x8341.
             *   retirer : des que le prologue 0x013b se termine sur 0x8341 par son propre flot
             *             (meme condition que FIX_3FCD reussi).
             *   NB      : calypso_wire.env fait un unset EXPLICITE — un ":=vide" sous set -a
             *             rallumerait ce gate EXISTS. Ne jamais le convertir en ":=".
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

        /* [2026-07-25] CORR-SETUP (diag user "setup AR/BK a l'entree 0x8d00,
         * meme patch avec les constantes") : le correlateur 0x8d00 est atteint
         * (via BSP-DISPATCH-FB) mais SANS le setup que la LUT native 0x8341 pose
         * juste avant : STM #0x2f22,AR1 / #0x2be4,AR4 / #0x0060,AR5 (desassemble
         * PROM0 0x8347/0x8349/0x834b). Sans ces pointeurs le MAC boucle a
         * 0x8e8b/0x8e8c sans conclure. On INJECTE ces constantes a l'entree
         * 0x8d00. Gate CALYPSO_CORR_SETUP ; override _AR1/_AR4/_AR5. */
        if (exec_pc == 0x8d00) {
            /* @BEQUILLE — CORR_SETUP (+ CORR_AR1/_AR4/_AR5)  (CALYPSO_CORR_SETUP, EXISTS,
             *              defaut OFF)
             *   masque  : le setup de pointeurs que la LUT native 0x8341 pose avant d'entrer
             *             en 0x8d00 (STM #0x2f22,AR1 / #0x2be4,AR4 / #0x0060,AR5). On INJECTE
             *             ces constantes a l'entree du correlateur.
             *   retirer : quand le chemin natif passe par 0x8341 avant 0x8d00 (au lieu d'y
             *             entrer par BSP_DISPATCH_FB).
             *   NB      : idiome EXISTS — un ":=" vide l'ALLUMERAIT, d'ou le unset explicite
             *             de calypso_wire.env. Mesure : inefficace (AR reecrits avant 0x8e8b).
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

        /* POKE-A4C7-ONCE (2026-07-03, gated CALYPSO_POKE_A4C7_ONCE, DIAGNOSTIC
         * ONLY -- falsification test, not a fix, revert after use). Addendum 20 :
         * the go-live wait-loop entry at 0xa4ca is ALWAYS reached directly,
         * skipping 0xa4c7 (ORM #0x3000,IMR -- arms bit12/vec28) 3 words earlier,
         * which is 0-hit all session. This redirects the FIRST arrival at 0xa4ca
         * to 0xa4c7 instead -- the CPU then naturally executes the real ROM ORM
         * instruction and falls through back into 0xa4ca normally. No register/
         * memory value is poked directly -- only the entry PC, once, to let the
         * ROM's OWN arming instruction run. Tests: does IMR arm (0x3000), does
         * the frame IT then vector to vec28, does d_fb_det become nonzero, and
         * do data[0x3f70]/data[0x435b] populate too (single-root-cause test). */
        if (exec_pc == 0xa4ca) {
            /* @BEQUILLE — POKE_A4C7_ONCE  (CALYPSO_POKE_A4C7_ONCE, atoi>0, defaut OFF)
             *   masque  : 0xa4c7 (ORM #0x3000,IMR = armement IMR par la ROM) n'est jamais
             *             atteint : le flot entre a 0xa4ca en sautant l'instruction d'armement.
             *             On detourne le PC une fois.
             *   retirer : des que le chemin amont (0xa4cd BC AEQ, ou le setter de d[434e]/
             *             d[434f]) laisse tomber dans 0xa4c7.
             *   NB      : calypso_hack.env le qualifie lui-meme de "falsification, pas un fix".
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

        /* CALA-71DA (2026-07-03, gated CALYPSO_CALA_71DA, RO) : le wrapper
         * generique save/dispatch/restore a 0x71c0-0x71f2 (PSHM x19 ; ST0=0,
         * ST1=0x6900 ; CALA @0x71da ; POPM x19 ; RET) dispatche vers l adresse
         * dans A. Log A juste avant le CALA -- determine si ce dispatcher
         * appelle jamais autre chose qu un stub no-op (meme famille de boucle
         * fermee auto-referentielle que data[0x4387]->0xab38, addendum 15). */
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

        /* FORCE-IMR (2026-07-02, gate CALYPSO_C54X_FORCE_IMR=<hex>) : le ROM efface
         * IMR (STM #0,IMR @0xb37e insn~1047) et ne le re-arme jamais avant le
         * scheduler b41c -> la frame IT (INT3, IFR bit3) reste masquee -> spin.
         * On OR les bits demandes dans IMR a chaque pas HORS ISR (INTM=0 window de
         * IRQ-LEVEL le sert). Test falsifiable de la chaine IMR->IT->0x0fff->golive.
         * Defaut OFF. Typique : 0x52fd (bit3 INT3 + bit5 BRINT0 + ...). */
        {
            /* @BEQUILLE — C54X_FORCE_IMR  (CALYPSO_C54X_FORCE_IMR=<hex>, defaut OFF)
             *   masque  : le re-armement de l'IMR apres le STM #0,IMR du mask-ROM @0xb37e, et
             *             le RSBX INTM que le ROM ne joue qu'apres go-live. On OR les bits dans
             *             l'IMR a chaque pas hors ISR et on clear INTM dans [0xb380..0xb440].
             *   retirer : quand la SM go-live atteint 0xa582 et pose l'IMR elle-meme.
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
            /* La boucle idle scheduler b380-b440 doit tourner INTM=0 (attente IT).
             * Le ROM ne clear INTM (RSBX @0xa51b) qu apres go-live (chicken-egg) ->
             * on clear INTM HORS ISR uniquement dans l idle loop, pour que IRQ-LEVEL
             * serve la frame IT latchee. Ne touche pas les ISR (PC bas / 7234). */
            if (fimr && exec_pc >= 0xb380 && exec_pc <= 0xb440 && (s->st1 & ST1_INTM) &&
                ((s->pmst >> PMST_IPTR_SHIFT) & 0x1FF) != 0x1FF) {
                static unsigned ftc = 0;
                if (ftc++ < 8)
                    fprintf(stderr, "[c54x] FORCE-INTM clr @PC=0x%04x IFR=0x%04x IMR=0x%04x insn=%u\n",
                            exec_pc, s->ifr, s->imr, s->insn_count);
                s->st1 &= ~ST1_INTM;
            }
        }

        /* FORCE-098 (etape A faithful, gated CALYPSO_FORCE_098=<hexval>) : juste AVANT
         * les lectures LD *(0x098c)/(0x098a) des setters (0xde86 chemin GO ; 0xde94 ;
         * 0xb3e4), pose data[0x098a]/[0x098c] = valeur non nulle cote DSP. Teste si le
         * DSP prend alors la branche GO (0xddf5) puis setter bit1 puis go-live. */
        {
            /* @BEQUILLE — FORCE_098  (CALYPSO_FORCE_098=<hexval>, defaut vide/OFF ; hack.env)
             *   masque  : l'ARM ne pose jamais les cellules de handshake d_background
             *             0x098a/0x098c que la phase-SM 0xddeb/0xde86 relit.
             *   retirer : des que CALYPSO_ARM2DSP_BGEN pose ces cellules par le pont ARM
             *             (causalite correcte) — calypso_hack.env declare deja le remplacement.
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
        /* GO-LIVE FB-task hold (2026-07-25) — INTEGRATION NATIVE, remplace l'ancien
         * FORCE poke (=0xC000, overwrite, mauvais bit). Le firmware efface d[0x3f92]
         * par ST #0 @0xa4c4 puis DEVRAIT le re-armer par ORM #0x0800 @0xa539 — mais
         * ce setter natif est skippe (d[5a00]==0x88), donc le bit tache-FB (0x0800)
         * reste 0 a vie et le scheduler DSP ne dispatche jamais le correlateur. On
         * REJOUE ici exactement ce que ferait l'ORM 0xa539, mais SEULEMENT quand l'ARM
         * a effectivement commande le go-live (d[0x0810] bit15, pose par le wire
         * CTRLSYS) : causalite correcte ARM->DSP, bon bit (0x0800, PAS 0xC000), OR (pas
         * d'overwrite des autres bits scheduler). Gate CALYPSO_GOLIVE_TASKW, defaut OFF.
         * 0x0810 est deja gere par le wire CTRLSYS (arm2dsp) -> plus de FORCE_0810. */
        if (exec_pc >= 0xa4ca && exec_pc <= 0xa575) {
            /* @BEQUILLE — GOLIVE_TASKW  (CALYPSO_GOLIVE_TASKW, EQ1, defaut OFF)
             *   masque  : le setter natif ORM #0x0800 @0xa539 est skippe (d[5a00]==0x88), donc
             *             le bit tache-FB de d[0x3f92] reste 0 et le scheduler ne dispatche
             *             jamais le correlateur. On rejoue l'instruction.
             *   retirer : des que 0xa539 est reellement execute (le predicat d[5a00] tient la
             *             bonne valeur), ce qui rend le rejeu redondant.
             *   NB      : inerte sans ARM2DSP_CTRLSYS (exige data[0x0810] bit15).
             */
            static int gt = -1;
            if (gt < 0) { const char *e = getenv("CALYPSO_GOLIVE_TASKW");
                          gt = (e && *e == '1') ? 1 : 0; }
            if (gt && (s->data[0x0810] & 0x8000)) {
                s->data[0x3f92] |= 0x0800;   /* rejoue ORM #0x0800 @0xa539 */
                static unsigned glg = 0;
                if (glg++ < 8)
                    fprintf(stderr, "[c54x] GO-LIVE-TASKW @0x%04x d[3f92]=0x%04x "
                            "(ORM 0xa539 rejoue, ARM 0810 bit15 set) insn=%u\n",
                            exec_pc, s->data[0x3f92], s->insn_count);
            }
        }
        /* SM-TRACE (gated CALYPSO_SM_TRACE) : trace instruction-par-instruction
         * l'etat-machine handshake 0xdde0-0xde9f (route reclear 0xde8b vs setter
         * 0xde9c). Montre PC/op/A/TC + les 5 cellules 0x098a..0x098e a chaque
         * pas, pour voir OU le flot devie du chemin de9c et quelles valeurs le
         * routeraient. Cap 400. */
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
        /* B3-TRACE (gated CALYPSO_B3_TRACE) : le scheduler idle 0xb380-0xb440 poll
         * le mot de flags data[0x0fff] (b424 BITF 0x0fff,#2 ; b427 BC NTC b41c) et
         * doit router vers le bloc go-live b3db-b3ef (b3ef = ST #2,0x3f70). Trace PC +
         * data[0x0fff]/[0x08E2=d_dsp_page]/[0x3f70=golive]. Montre pourquoi la commande
         * ARM (d_dsp_page bit1) n aboutit pas au bloc b3db. Cap 500. */
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
        /* HANDLER-PATH (2026-06-25, RO) : apres VEC28-FORCE, le handler vec28
         * tourne mais n'atteint pas le dispatch 0xa51c. Trace le chemin exact :
         * 0xf0(vecteur) -> 0x7234(handler) -> 0x013b(prologue) -> 0xa4e4(sched) ->
         * 0xa4ff(CALL 0xb522) -> 0xa501/0xa507 -> 0xa51c(dispatch) / 0xa509(arm IMR).
         * Voir OU la chaine devie. Cap 120. */
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
        /* ENTRY-A4CA (2026-06-24, RO) : COMMENT le DSP entre dans la boucle
         * d'attente 0xa4ca — le caller exact (exec_pc precedent hors region
         * 0xa4ca..0xa4e2) + l'etat. Tranche : entree via go-live 0xa500 (normal,
         * INTM deja cleared) vs branche/soft-vector directe (anormal = pas passe
         * par le go-live, INTM encore set). Cap 20. */
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
        /* B19D-WATCH (2026-06-24, RO) : la dispatch commande ARM 0xb19d -> go-live
         * 0xa500 est-elle JAMAIS atteinte ? 0xa500 est deja sous GOLIVE-WATCH
         * (jamais vu > 0xa4e2) ; ici on regarde l'amont 0xb19d. Si jamais hit ->
         * le DSP ne traite pas d_dsp_page, coince dans la mauvaise boucle en
         * amont du go-live. Cap 30. */
        if (exec_pc >= 0xb19d && exec_pc <= 0xb1b0) {
            static unsigned bw = 0;
            if (bw++ < 30)
                fprintf(stderr, "[c54x] B19D-WATCH #%u PC=0x%04x op=0x%04x INTM=%d "
                        "A=0x%06llx data[0x08E2]=0x%04x insn=%u\n",
                        bw, exec_pc, exec_op, (s->st1 & ST1_INTM) ? 1 : 0,
                        (unsigned long long)(s->a & 0xFFFFFF),
                        s->data[0x08E2], s->insn_count);
        }
        /* AAD5-TRACE (2026-06-24) : la boucle go-live/AFC 0xa4ca ne relache jamais
         * (BC 0xa4cd = AEQ A==0). A vient de CALL 0xaad5. On trace 0xaad5-0xaae6
         * (le poseur de A) instruction par instruction + AR0/AR1/A/TC + les mots
         * compteurs data[0x434e]/data[0x434f] et les flags candidats
         * data[0x3f70]/data[0x3f92]/data[0x435b] : voit-on un compteur qui
         * n'avance pas (bug decode/data) ou une attente d'IT (IMR=0 -> jamais) ?
         * Logge aussi le verdict au gate 0xa4cd. Cap 100. */
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
        if (exec_pc == 0xa4cd) {     /* le gate BC AEQ : pourquoi A==0 ? */
            static unsigned gt = 0;
            if (gt++ < 30)
                fprintf(stderr, "[c54x] AFC-GATE #%u @0xa4cd A=0x%06llx (==0?%d) TC=%d "
                        "d[3f70]=%04x d[3f92]=%04x d[435b]=%04x d[3fde]=%04x insn=%u\n",
                        gt, (unsigned long long)(s->a & 0xFFFFFF),
                        ((s->a & 0xFFFFFFFFFFULL) == 0), (s->st0 & ST0_TC) ? 1 : 0,
                        s->data[0x3f70], s->data[0x3f92], s->data[0x435b],
                        s->data[0x3fde], s->insn_count);
        }

        /* INTM-CLEAR (ungated) : 1ere transition INTM 1->0 du run = RSBX INTM
         * enfin execute = interruptions globalement armees. LE signal de victoire. */
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

        /* === SONDES MVDK-FIX VALIDATION + OBSERVATION (2026-06-24) ===
         * MVDESYNC : apres le fix MVDK 0x71, les mots-operandes ne doivent PLUS
         * etre executes comme instructions. Fire si PC atterrit sur un mot
         * operande MVKD (0xb3ce/d1/d4) ou MVDK (0xb3dd/e0/e3) = mis-decode
         * residuel. SILENCE attendu = fix OK. Cap 40. */
        if (exec_pc==0xb3ce||exec_pc==0xb3d1||exec_pc==0xb3d4||
            exec_pc==0xb3dd||exec_pc==0xb3e0||exec_pc==0xb3e3) {
            static unsigned md=0;
            if (md++<40)
                fprintf(stderr, "[c54x] MVDESYNC #%u PC=0x%04x op=0x%04x "
                        "(mot-operande execute = mis-decode!) from=0x%04x insn=%u\n",
                        md, exec_pc, exec_op, g_prev_pc, s->insn_count);
        }
        /* Valeurs des 2 slots CALA (le 3e = BACC 0xb40f deja a BACC-DISP). */
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
        /* FINDING-2 unblock : l'init programme-t-il enfin IPTR=0x140 (base
         * vecteurs 0xa000 -> INT3 @0xa04c, le vrai handler trame) ? one-shot. */
        {
            static int seen140=0;
            uint16_t iptr_now=(s->pmst >> PMST_IPTR_SHIFT) & 0x1FF;
            if (iptr_now==0x140 && !seen140) {
                seen140=1;
                fprintf(stderr, "[c54x] *** IPTR=0x140 REACHED *** PMST=0x%04x "
                        "PC=0x%04x insn=%u\n", s->pmst, exec_pc, s->insn_count);
            }
        }
        /* SONDE post-bootstub-ret (GAP-1) : prologue @0x7013 (pshm contexte) et
         * epilogue @0x7020 (popm ar1/st0/st1/pmst; ret). Si l'epilogue depile un
         * contexte/retPC stale (pas de prologue/IT correspondant) -> desync pile
         * post-bacc. retPC = data[SP+4] (apres les 4 popm). */
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

        /* GOLIVE-REDIRECT (2026-06-25, gated CALYPSO_DSP_GOLIVE_BOOT) : EXPÉRIENCE (B)
         * preuve-de-racine. Au point où le DSP exécuterait la wait-loop à 0xa4df,
         * redirige le FLUX (PC) vers 0xa4c7 (l'arme IMR go-live), UNE fois. Le DSP
         * exécute ensuite 0xa4c7(ORM #0x3000,IMR)->0xa4ca->... EN FOREGROUND (il pose
         * son propre contexte). PAS une vectorisation (pas de saut ISR sur contexte
         * non posé) = équivalent « et si le soft-vector pointait 0xa4c7 ». TEST, pas fix. */
        {
            /* @BEQUILLE — GOLIVE_REDIRECT  (CALYPSO_DSP_GOLIVE_BOOT, EXISTS, defaut OFF)
             *   masque  : ecrit s->pc = 0xb3ec quand le DSP atteint 0xb3ff, c'est-a-dire le
             *             choix de soft-vector go-live que le boot ROM ne fait pas dans notre
             *             modele. Second effet : inhibe VEC28-FORCE (bloc c54x_interrupt_ex).
             *   retirer : quand data[0x3f6d] est peuple par le chemin ROM et pointe 0xa4c7.
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
        /* SONDE GAP-1 DERAIL-ZERO : entrée dans la zone 0x0000-0x0008 = le CALA
         * vers un pointeur de fonction NUL (slot dispatcher SARAM = 0). Logge le
         * site du CALA (g_prev_pc), les accumulateurs (le 0), et les 4 mots ROM
         * AVANT le CALA (= le `ld *(slot),b` -> l'adresse du slot nul a decoder). */
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
                    /* [2026-07-22] SP-RING au 1er storm : les 24 derniers push/pop
                     * (pc:op delta) -> l instruction qui over-pop et derive SP. */
                    fprintf(stderr, "[c54x] SP-RING-AT-STORM (64 derniers, ancien->recent ; MISMATCH = op qui touche SP a tort):\n");
                    for (int k = 63; k >= 0; k--) {
                        struct sp_evt *e = &g_spring[(g_spring_idx - 1 - k) & 63];
                        const char *mn = (e->op == 0xFC00) ? "RET" : classify_xfer_op(e->op);
                        int exp = 99;  /* 99 = op non-transfert (PSHM/POPM/FRAME/STM-SP legit, ou inconnu) */
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

        /* SONDE GAP-1 DERAIL-ORIGIN : premiere entree dans la zone table garbage
         * 0xf090-0xf0a0 depuis l'EXTERIEUR = le saut/chute qui deraille. Logge
         * d'ou (g_prev_pc), l'opcode, les ARs (deja garbage ou non), le SP. */
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
        /* SURGICAL : capture silencieuse du slot LUT lu au 0x834d (LD
         * (DP<<7|0x07)<<1,A). 1 compare/insn, pas de log → ~zéro impact
         * timing. Sert le probe BLACKHOLE-CALA (self-CALA 0x70c3). */
        if (s->pc == 0x834d) {
            g_disp_lut_ea  = (uint16_t)(((s->st0 & 0x1FF) << 7) | 0x07);
            g_disp_lut_val = s->data[g_disp_lut_ea];
        }
        uint16_t sp_before_exec = s->sp;
        uint16_t ds_before = s->delay_slots;  /* delay-slot word-count fix 2026-05-31 */
        /* ===== SHADOW-DADST pre-capture (RO, revival dsp 2026-06-22) —
         * GATÉ SUR L'OPCODE DADST/DSADT (0x5a/0x5b/0x5e/0x5f) PARTOUT (pas le PC
         * 0x9a80, qui est run-variant : narrow vs wide). Cette famille tombe en
         * SFTL (case 0x5). Capture l'état AVANT pour ΔA/ΔB, marche AR5, et le
         * résultat shadow-correct. Strictement RO. Le PC loggé révèle où elle tourne. */
        int64_t  sd_a0 = s->a, sd_b0 = s->b;
        uint16_t sd_ar5_0 = s->ar[5], sd_lhi = 0, sd_llo = 0;
        uint8_t  sd_sub = (exec_op >> 8) & 0xFF;
        int sd_armed = (sd_sub == 0x5a || sd_sub == 0x5b
                        || sd_sub == 0x5e || sd_sub == 0x5f);
        if (sd_armed) {
            sd_lhi = s->data[s->ar[5]];
            sd_llo = s->data[(uint16_t)(s->ar[5] + 1)];
        }
        /* [2026-09-18] PISTE-PC : trace pas-a-pas d une PLAGE de PC, avec
         * l opcode REEL vu par le coeur et les registres AVANT execution.
         * Env : CALYPSO_PISTE_LO / CALYPSO_PISTE_HI / CALYPSO_PISTE_N (def 400).
         * Motif : l etage qui reecrit le tampon de bits souples 0x2a00 sort
         * 142 zeros sur 7 jobs SB sur 12 et des valeurs saines sur les 5 autres.
         * Il faut voir l instruction et son etat, pas le deduire. Contrairement
         * aux sondes de c54x_mem.c, le PC imprime ici est celui AVANT
         * avancement : c est le vrai PC de l instruction. */
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
                /* mots POINTES par AR2..AR5 : lecture directe de data[] (ces
                 * adresses sont en RAM interne, hors alias OVLY/MMR), pour voir
                 * ce qu un mpy/mac dual-operand consomme reellement. */
                { /* adressage DIRECT : DP, CPL, SP et les deux adresses
                   * candidates avec leur contenu. Un `ld Smem,T` qui charge 0
                   * peut lire la bonne case (vide) ou la mauvaise case : seule
                   * l adresse resolue tranche. */
                    uint16_t dpv = s->st0 & 0x1FF;
                    uint16_t adp = (uint16_t)((dpv << 7) | (exec_op & 0x7F));
                    uint16_t asp = (uint16_t)(s->sp + (exec_op & 0x7F));
                    fprintf(stderr, "[c54x] PISTE   DP=0x%03x CPL=%d SP=%04x | direct@DP=0x%04x:%04x "
                            "direct@SP=0x%04x:%04x | indirect=%s\n",
                            dpv, (s->st1 & 0x4000) ? 1 : 0, s->sp,
                            adp, s->data[adp], asp, s->data[asp],
                            (exec_op & 0x80) ? "oui" : "non");
                }
                { /* dump optionnel d une plage data a chaque passage : voir
                   * DIVERGER un job sain d un job qui sort des zeros. */
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
        /* [2026-09-18] PISTE-T : journalise CHAQUE changement du registre T dans
         * une fenetre d instructions (CALYPSO_T_LO / CALYPSO_T_HI, en insn).
         * Motif : tous les mpy de la chaine SB calculent T*Smem avec T=0 alors
         * que les operandes memoire sont sains ; il faut savoir qui a charge T
         * et quand, pas le supposer. */
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
            /* QUI ANNULE : transitions non-nul -> nul d un accumulateur. Le
             * defaut se propage de proche en proche (« nul parce que son entree
             * est nulle ») ; ce qui compte est la PREMIERE annulation. */
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
        /* [2026-09-18] OVM (ST1 bit 9, mode saturation) : la ROM du DSP l'ACTIVE, et le
         * coeur l'ignorait totalement — `sat32()` etait defini et utilise ZERO fois dans
         * tout le fichier. Consequence : les resultats qui devraient etre ecretes a
         * 32 bits croissent librement dans les 8 bits de garde. Dans un MLSE en virgule
         * fixe, l'ecretage est ce qui BORNE les metriques de chemin ; sans lui elles
         * divergent et la contribution relative des symboles de bord s'effondre. Mesure
         * a l'appui : le profil d'influence couvre trois ordres de grandeur entre le
         * milieu du burst (13452, 19534) et ses bords (2 a 30).
         * Ecretage POST-INSTRUCTION, donc approximatif : le silicium sature par
         * operation. C'est une sonde de decision, pas l'implementation finale.
         * Gate CALYPSO_ISA_OVM, defaut 0 pour ne rien changer par surprise. */
        {
            static int ovm_gate = -1;
            if (ovm_gate < 0) {
                ovm_gate = calypso_gate("CALYPSO_ISA_OVM", 0);
                if (ovm_gate)
                    fprintf(stderr, "[c54x] ISA-OVM ACTIF : ecretage des accumulateurs a "
                            "32 bits quand ST1.OVM est pose (approximation post-instruction)\n");
            }
            if (ovm_gate && (s->st1 & ST1_OVM)) {
                s->a = sat32(s->a);
                s->b = sat32(s->b);
            }
        }
        /* SP-COLLAPSE probe (RO, revival dsp 2026-06-22) : attrape l'instruction
         * EXACTE qui effondre SP sous 0x0800 (1ère + 30 suivantes) — le seed de
         * toute la cascade (boot-stub / spin 0xc6ac). */
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
        /* FEXX-ENTRY probe (RO, revival dsp 2026-06-22) : capture le saut/vecteur
         * qui transfère le contrôle DANS la ROM 0xfe00-0xff7f (data table + init
         * 0xff00) = la VRAIE entrée du déraillement (le CALL statique 0xfe00 @0xfdd5
         * ne s'exécute jamais → c'est un branchement CALCULÉ). prev_PC/op = coupable,
         * A = la cible si c'est un CALA. 1ère + 20 suivantes. */
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
        /* HIGHVEC-ENTRY probe (RO, revival dsp 2026-06-23) : entrée dans la PAGE
         * VECTEUR IPTR=0x1FF [0xFF80..0xFFFF] = là où l'IRQ atterrit (vec19=0xffcc,
         * vec21=0xffd4). LA question décisive (fusion §2 vs bugs séparés) :
         * le contrôle entre-t-il via DISPATCH D'INTERRUPTION (prev = foreground
         * préempté, d_irq~0) ou via un BRANCH/CALL firmware (prev = un opcode de
         * branche ciblant ici, d_irq grand) ? Un dump, deux mondes. Épingle aussi
         * le RÉGIME (ARs, A, B) au point d'entrée. Strictement RO. */
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
        /* REGIME-PIN probe (RO, revival dsp 2026-06-23) : épingle SANS AMBIGUÏTÉ le
         * régime du run COURANT. Les vieux logs disent AR5=0x80/AR3=0x000b ; ce soir
         * AR3=AR4=0x2ace figés dans le buffer I/Q. Deux régimes = deux fantômes ;
         * on en debugge UN. Dump complet du fichier registre à la 1ère + 10000e +
         * 1Me visite du spin foreground [0x82c0..0x82f0] (la boucle IQ-READ gelée). */
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
        /* SQURA-A probe (RO, revival dsp 2026-06-23) : voir A basculer >0 avec le
         * fix SQURA. Logue A aux 2 SQURA (0x76ff/0x7700) + à la décision RCD
         * LEQ@0x75e8 (prend si A<=0). Avec le blind-MAC A restait <=0 (RCD prend,
         * over-pop) ; avec SQURA A doit devenir >0 (RCD ne prend pas). */
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
        /* DISP-A probe (RO, revival dsp 2026-06-22) : évolution de A à travers le
         * dispatcher 0x80b0-0x80c9 → quel LD pose A=0xfe36 (le handler garbage du
         * FCALAD @0x80c8) et depuis quelle case data. */
        if (exec_pc >= 0x80b0 && exec_pc < 0x80ca) {
            static unsigned da_n = 0;
            uint16_t alo = (uint16_t)(s->a & 0xFFFF);
            int garbage = (exec_pc == 0x80c2 && alo >= 0xfe00); /* le LD du derail */
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
        /* DP-AT-DISP probe (RO, revival dsp 2026-06-22) : tracke la DERNIÈRE
         * instruction qui change DP (ST0[8:0]), et au moment où le dispatcher
         * entre (0x80b0) logue DP + qui l'a posé → trouve qui met DP=0x124 (la
         * mauvaise page = table coefficient 0x9200 au lieu d'un handler). */
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
        /* ===== SHADOW-DADST post-compute + oracle (RO, logs only,
         * cap = 40 premiers (boot) + 1/2000 (régime permanent) ) ===== */
        if (sd_armed) {
            static unsigned sd_n = 0;
            if (sd_n < 40 || (sd_n % 2000) == 0) {
                int64_t a1 = s->a, b1 = s->b;
                int asm5 = s->st1 & ST1_ASM_MASK; if (asm5 & 0x10) asm5 -= 32; /* ASM signé 5b */
                uint64_t a0u = sd_a0 & 0xFFFFFFFFFFULL;
                int64_t pure_sh = (asm5 >= 0) ? (int64_t)(a0u << asm5)
                                              : (int64_t)(a0u >> (-asm5));
                int shiftLike = ((a1 & 0xFFFFFFFFFFULL) == (pure_sh & 0xFFFFFFFFFFULL)); /* OBS1 */
                int16_t T = (int16_t)s->t, lhi = (int16_t)sd_lhi, llo = (int16_t)sd_llo;
                int is_dadst = (sd_sub == 0x5a || sd_sub == 0x5b);
                int32_t sh_hi = is_dadst ? (lhi + T) : (lhi - T); /* SPRU131 dual-16, signe TBC */
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

        /* === DECODE-AUDIT (2026-06-02, brief CC-web : audit différentiel) ===
         * Inventaire du décodeur sur l'overlay corrélateur 0x8000-0x9FFF :
         * logge UNE fois par (PC,op) distinct l'opcode brut, le mot suivant, et
         * la LONGUEUR consommée (= consumed, inclut lk_used). À diff contre
         * doc/opcodes/tic54x_hi8_map.md — colonne longueur d'abord (un mauvais
         * len désync tout le flux suivant, cf bug 0x86/0x87→SP runaway). Vise à
         * vider la classe « décode/longueur/mode » d'un coup au lieu de peler
         * bug par bug. dedup par (PC,op) → capture aussi les variantes XPC d'un
         * même Pag. Gate CALYPSO_DEBUG=DECODE-AUDIT, coût ~nul après couverture. */
        static int da_lo = -1, da_hi = -1;
        static long long da_insn = -1;
        if (da_lo < 0) {
            const char *l = getenv("CALYPSO_DA_LO"); const char *h = getenv("CALYPSO_DA_HI");
            const char *n = getenv("CALYPSO_DA_INSN");
            da_lo = l ? (int)strtol(l, NULL, 0) : 0x8000;   /* overlay corrélateur par défaut */
            da_hi = h ? (int)strtol(h, NULL, 0) : 0x9FFF;   /* CALYPSO_DA_LO/HI pour élargir (ex. 0x7000..0xFFFF) */
            da_insn = n ? strtoll(n, NULL, 0) : 0;          /* CALYPSO_DA_INSN : skip le boot, viser la fenêtre détection (ex. 250000000) */
        }
        if (exec_pc >= (uint16_t)da_lo && exec_pc <= (uint16_t)da_hi
            && s->insn_count >= (uint64_t)da_insn
            && calypso_debug_enabled("DECODE-AUDIT")) {
            static uint16_t da_op_seen[0x10000];
            static uint8_t  da_has[0x10000];
            static unsigned da_n = 0;
            uint16_t da_op = prog_fetch(s, exec_pc);
            /* skip op=0x0000 : trous PROM vide (runaway control-flow, pas un bug
             * de DÉCODE) — ils empoisonnaient le cap au boot (sweep 0xCB00-0xD3FF). */
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
        /* [2026-07-22] INTM-TRANS : trace toute bascule du bit INTM (ST1 b11)
         * avec le PC/opcode qui l'a causee. Repond a "INTM passe-t-il jamais a
         * 0, et si oui qui le re-arme". Silent sauf CALYPSO_DEBUG=INTM-TRANS. */
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

            /* ============================================================= *
             * [2026-07-30] ACQUITTEMENTS + TICK A CHAQUE TRANSITION INTM
             *   CALYPSO_INTM_ACK (defaut 0)
             *
             * @BEQUILLE — INTM_ACK
             *   (1) C'EST UNE BEQUILLE. Le vrai materiel n'acquitte rien « sur
             *       transition INTM » : l'ARM ecrit IRQ_CTRL bit0 dans l'epilogue
             *       de son irq(), et pose d_dsp_page = B_GSM_TASK|w_page dans
             *       dsp_end_scenario(). Ici on declenche ces gestes depuis le DSP,
             *       parce que les deux cotes ne se les envoient pas.
             *   (2) CE QU'ELLE MASQUE : (a) l'epilogue « new IRQ agreement » du
             *       firmware ARM n'atteint pas notre INTH sur le chemin natif ;
             *       (b) l'ecriture ARM de d_dsp_page est avalee par l'overlay
             *       2 octets pose en priorite 10 sur 0xFFD001A8, si bien que la
             *       cellule garde sa valeur de boot et que B_GSM_TASK ne parvient
             *       jamais au DSP ; (c) aucune base de temps ne fait battre TINT0,
             *       calypso_tint0_start() n'etant jamais appele.
             *   (3) QUAND LA RETIRER : des que l'overlay d_dsp_page est retire au
             *       profit d'un vrai pass-through (l'ARM ecrit, le DSP lit), et
             *       que le TIMER0 du ROM est reellement programme (TCR TSS=0).
             *       Alors les trois gestes reviennent a leurs proprietaires.
             *
             * SENS DE LA TRANSITION :
             *   0->1 (IT prise)   -> ACK ARM  : le DSP accepte la requete levee
             *                       par l'ARM/TPU ; on ferme l'accord cote INTH
             *                       et on pose d_dsp_page = B_GSM_TASK|w_page.
             *   1->0 (RETE)       -> ACK DSP  : le DSP a fini son ISR ; on relache
             *                       le LEVEL hold de la frame-IT et on bascule la
             *                       page de lecture (miroir de r_page ^= 1).
             *   les deux          -> TICK TINT0 (vec20 / IMR bit4).
             *
             * CALYPSO_INTM_ACK_SWAP=1 echange les deux sens.
             * CALYPSO_INTM_ACK_NO_TINT0=1 garde les acks sans le tick.
             * ============================================================= */
            if (intm_now_tr != g_intm_prev_tr && g_intm_prev_tr >= 0) {
                static int _ia = -1, _iswap = -1, _int0 = -1;
                static int _iarm = -1, _idsp = -1, _idpage = -1;
                static unsigned _idpage_every = 0;
                static int _busy = 0;
                if (_ia < 0) {
                    _ia    = calypso_gate("CALYPSO_INTM_ACK", 0);
                    _iswap = calypso_gate("CALYPSO_INTM_ACK_SWAP", 0);
                    _int0  = getenv("CALYPSO_INTM_ACK_NO_TINT0") ? 0 : 1;
                    /* [2026-07-30] A/B : chaque geste isolable. Defaut = suit
                     * INTM_ACK, sauf la pose de d_dsp_page (opt-in explicite).
                     *   CALYPSO_INTM_ACK_NO_ARM=1    coupe l'ack ARM (INTH)
                     *   CALYPSO_INTM_ACK_NO_DSP=1    coupe l'ack DSP (level hold)
                     *   CALYPSO_INTM_ACK_DPAGE=1     active la pose de d_dsp_page
                     *   CALYPSO_INTM_ACK_DPAGE_EVERY defaut 65536 insn (= periode
                     *                                TIMER0 mesuree = cadence trame)
                     *
                     * MOTIF (mesure du 30/07) : la pose de d_dsp_page s'executait a
                     * CHAQUE prise d'IT, soit ~1000 fois la ou dsp_end_scenario() ne
                     * l'ecrit qu'UNE FOIS PAR TRAME (osmocom-bb layer1/sync.c ->
                     * calypso/dsp.c:471). On martelait la cellule de synchro du DSP a
                     * ~50x la cadence trame. */
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
                        /* d_dsp_page = B_GSM_TASK | w_page  (dsp.c:471), au plus
                         * une fois par trame — voir le MOTIF plus haut. */
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
                        g_frame_it_level = false;   /* relache le LEVEL hold */
                        if (g_last_intr_vec >= 0) {
                            /* fin de service : la source servie retombe */
                            if (g_last_intr_vec == 28)
                                c54x_ifr_clear(s, (uint16_t)(1u << 12), "intm-ack-dsp");
                            g_last_intr_vec = -1;
                        }
                        /* miroir de sync.c : r_page ^= 1 apres consommation */
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

                    /* --- TICK TINT0 (vec20 / IMR bit4) -------------------- */
                    if (_int0) {
                        static unsigned _n = 0;
                        if (_n++ < 40)
                            fprintf(stderr, "[c54x] INTM_ACK TINT tick "
                                    "IMR=0x%04x IFR=0x%04x insn=%u\n",
                                    s->imr, s->ifr, s->insn_count);
                        c54x_fire_tint(s);   /* respecte l'IMR ; §5.1 : bit3/vec19 */
                    }
                    _busy = 0;
                }
            }

            g_intm_prev_tr = intm_now_tr;
        }

        /* SP-event ring : enregistre tout changement de SP (push/pop) avec
         * le PC/op responsable. Sert le dump BLACKHOLE-CALA. */
        if (s->sp != sp_before_exec) {
            struct sp_evt *e = &g_spring[g_spring_idx++ & 63];
            e->pc = exec_pc;
            e->op = prog_fetch(s, exec_pc);
            e->delta = (int16_t)(s->sp - sp_before_exec);
            e->sp = s->sp;
            g_sp_ledger.net_words += (int16_t)(sp_before_exec - s->sp);
            if ((int16_t)(s->sp - sp_before_exec) < 0) g_sp_ledger.sp_pushes++;
            else g_sp_ledger.sp_pops++;

            /* PROBE 2026-05-31 : SP qui PLONGE dans la zone API RAM (0x0700-0x0a00)
             * = corruption → CALLD push clobber d_fb_det/d_fb_mode (0x08f8/9).
             * Loggue l'instruction qui y fait entrer SP (transition depuis hors
             * zone) + delta + voisinage pile. Nomme le setter de SP fautif. */
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

            /* === SHADOW STACK : appariement push/pop (gate ORPHAN) ===
             * Nomme LE return orphelin (over-pop), pas les 15 victimes 0xc8be. */
            if (g_shadow_on < 0) {
                const char *eo = getenv("CALYPSO_ORPHAN");  /* env dédiée (hors CALYPSO_DEBUG) */
                g_shadow_on = (eo && *eo) ? 1 : 0;
            }
            if (g_shadow_on) {
                uint16_t op = e->op;
                int16_t  d  = e->delta;
                int is_call = (op==0xF074||op==0xF274||op==0xF4E3||op==0xF4E7||op==0xF6E3);
                int is_ret  = (op==0xFC00||op==0xFE00||op==0xF4EB||op==0xF4E4||op==0xF6EB
                               ||(op&0xFF00)==0xFC00);   /* RET/RETD/RETE/FRET/RETED + RC cond */
                int is_pshm = ((op&0xFF00)==0x4A00||(op&0xFF00)==0x4B00);
                (void)is_call;
                if (d < 0) {                 /* PUSH : SP a baissé */
                    int words = -d, w;
                    char kind = is_pshm ? 'P' : 'C';   /* PSHM=data ; reste=adresse retour */
                    for (w = 0; w < words; w++) {
                        if (g_shadow_depth >= 0 && g_shadow_depth < SHADOW_N) {
                            g_shadow[g_shadow_depth].pc   = exec_pc;
                            g_shadow[g_shadow_depth].op   = op;
                            g_shadow[g_shadow_depth].sp   = s->sp;
                            g_shadow[g_shadow_depth].kind = kind;
                        }
                        g_shadow_depth++;
                    }
                } else if (d > 0) {          /* POP : SP a monté */
                    int words = d, w;
                    for (w = 0; w < words; w++) {
                        g_shadow_depth--;
                        if (is_ret) {
                            if (g_shadow_depth < 0) {
                                g_orphan_hits++;
                                if (g_orphan_hits <= 40) {
                                    /* cible du return : RETD/RETED arment delayed_pc
                                     * (commit différé), RET/FRET immédiat = s->pc. */
                                    uint16_t ret_tgt = (s->delay_slots ? s->delayed_pc : s->pc);
                                    /* dernier PUSH réel de g_spring (le CALL apparié
                                     * manquant) : scan arrière sur delta<0. */
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
                                    /* slot lu par ce return : écrit (vecteur
                                     * légit) ou VIERGE (vrai garbage) ? */
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
                                    /* Au TOUT premier orphan : dump complet du ring
                                     * g_spring (reset→over-pop) pour compter push vs
                                     * pop directement = racine structurelle vs bug. */
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
                if (g_shadow_depth < 0) g_shadow_depth = 0;  /* re-ancre après orphan */
            }
        }

        /* === CORR-ABG probe (2026-05-30, c-web) : la FB-det est FRÉQUENTIELLE
         * (FCCH = ton pur), pas un pic d'amplitude. Au site corrélateur 0xec07,
         * capture A & B SÉPARÉS (= I/Q de la corr complexe), l'angle atan2(B,A)
         * (= la fréquence vue par le détecteur), et les valeurs aux 4 pointeurs
         * AR (data-I/Q vs table de réf cos/sin — vérifie que la réf est un vrai
         * sinus, pas du garbage/zéro). Cap 30. */
        /* DETECTOR-RUN (2026-05-30) : compteur d'exécutions du VRAI détecteur
         * freq FCCH (0x9ac0). Pourquoi ne tourne-t-il qu'1× au boot ? Loggue
         * insn + d_fb_mode (0x08f9, large vs étroit) + d_task_md (0x0804/0x0818)
         * à chaque passage. */
        /* [2026-07-27] B4-bis (gated CALYPSO_B4B) : trace du flux APRES le
         * detecteur 0x9ac0 -> 0xec07 (decision freq + ecriture 0x08f8) ou boucle.
         * + dump one-shot des opcodes 0x9ac0.. pour desassemblage. */
        {
            /* [2026-07-27] SCAN-08F8 (gated CALYPSO_SCAN_08F8) : one-shot, cherche
             * les instructions dont un mot == 0x08f8 (adr d_fb_det) dans tout le
             * bank courant -> writer de d_fb_det existe-t-il, et a quel PC ? */
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
        if (exec_pc == 0xa076) {   /* kernel MAC = LECTURE des operandes (I/Q + coeffs) */
            g_flow_armed = 1;   /* FLOWTRACE : arme la fenetre autour du detecteur */
            static int _b2k = -1; static unsigned _b2kn = 0;
            if (_b2k < 0) _b2k = calypso_gate("CALYPSO_B2AR", 0);
            /* [fix] compteur HORS gate : le min/max doit couvrir TOUT le run
             * (la version precedente se bloquait a la 1ere iteration). */
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
        {   /* [2026-07-27] ARWATCH : voir en-tete du patch. */
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
        {   /* ═══════════════════════════════════════════════════════════════
             * [2026-08-03] DISPATCH-PROBE — CALYPSO_DISPATCH_PROBE=1, defaut 0.
             *
             * Sonde de LECTURE SEULE posee sur tous les points remarquables du
             * desassemblage du 03/08. Elle ne modifie RIEN : elle observe.
             *
             * CE QU'ON CHERCHE. La routine d'armement RX du ROM, `0xa5cd`, installe
             * le tremplin data[0x0158] ET programme DMA2_AAD/ALGTH/CTRL(ENABLE=1).
             * Elle existe, elle est complete, et elle n'est JAMAIS exécutée. C'est
             * une entree de la table de dispatch, recopiee a l'init de la memoire
             * PROGRAMME vers les DONNEES :
             *     0xb4b6 : program[0xaae7..0xab34] -> data[0x4387..0x43d4]
             * donc program[0xab10]=0xa5cd atterrit en data[0x43b0], index 41.
             *
             * Le dispatcher fait, en SEPT endroits, le meme motif :
             *     sub #k1,A ; bc ...,AGT ; sub #k2,A ; add #0x4387,A
             *     stlm A,AR3 ; ld *AR3,A ; cala A
             * soit un aiguillage par PLAGES. La question est donc : quelle valeur
             * arrive dans A ? Le statique ne peut pas y repondre — d'ou cette sonde.
             *
             * Les sept sites sont les `add #0x4387` : on y lit A AVANT l'addition,
             * donc l'index brut, et on resout le handler que ca designe.
             * ═══════════════════════════════════════════════════════════════ */
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
                /* --- les sept aiguillages : A contient l'index avant `add` ------ */
                /* [2026-08-03, CORRIGE] la v1 listait aussi 0xb0f1 et 0xb0ff :
                 * FAUX. Le scan du ROM montre l'operande 0x4387 precedee de
                 * l'opcode 0xf000 (`ld #k16,A`) a ces deux endroits, et de 0xf200
                 * (`add #k16,A`) aux cinq autres. Ce ne sont pas des aiguillages
                 * mais des chargements de la base ; A y contenait deja une adresse
                 * de table resolue (0x43ac = base+37), d'ou l'`index=17324` absurde
                 * du premier run. Cinq vrais sites, donc. */
                {   /* ──────────────────────────────────────────────────────────
                     * [2026-08-03] CHAIN-B05F — le chainon manquant, tracé pas à pas.
                     *
                     * ETAT AVANT. Mesuré : l'ARM commande `d_task_d = 0x0018` (24,
                     * ALLC) ; le DSP LIT la cellule et voit bien 0x0018 (patte 4/4,
                     * 35 échantillons sur 41) ; la table contient le bon handler en
                     * index 41 (`data[0x43b0] = 0xa5cd`). Et pourtant les cinq sites
                     * `_sw[]` ci-dessous — la résolution d'index — ne sont JAMAIS
                     * atteints, donc l'armement RX n'est jamais demandé.
                     *
                     * CE QUI SE TROUVE ENTRE LES DEUX. Le bloc qui lit d_task_d en
                     * `0xb05f` enchaîne sur une chaîne de comparaisons (dump ROM) :
                     *     0xb062: f130 7fff
                     *     0xb064: f210 000c   0xb066: f843 b077   ; teste 12
                     *     0xb068: f210 0022   0xb06a: f846 b077   ; teste 34
                     *     0xb06c: f210 001e   0xb06e: f842 b070   ; teste 30
                     *     0xb070: f200 4387                       ; base de table
                     * La valeur lue est 24 — ni 12, ni 34, ni 30.
                     *
                     * POURQUOI UNE SONDE ET PAS UN DESASSEMBLAGE. Décoder f843/f846/
                     * f842 (conditions de branchement) à la main est exactement le
                     * geste qui a produit 3 fausses pistes sur 3 le 30/07 (§0 du
                     * TODO). On MESURE le chemin réellement pris et l'accumulateur à
                     * chaque pas ; la conclusion sortira du run, pas de ma lecture.
                     *
                     * L'ABSENCE DOIT RESTER LISIBLE : on trace TOUT le segment
                     * 0xb05f..0xb078, donc si le bloc n'est pas exécuté du tout la
                     * sonde est muette pour une raison différente — et le compteur
                     * périodique le dit. Plafonnée à 200 pas + 1 résumé/20 passages. */
                    if (exec_pc >= 0xb05f && exec_pc <= 0xb078) {
                        static unsigned long long _cn = 0, _pass = 0;
                        if (exec_pc == 0xb05f) _pass++;
                        if (_cn < 200) {
                            _cn++;
                            /* [2026-08-03] AR ajoutes : `0xb060` est `10e1 0000`,
                             * un LD INDIRECT. Le bloc lit son code de tache A
                             * TRAVERS un pointeur, et la mesure donne A=0x5294 —
                             * hors de portee des comparaisons (12/30/34), d'ou le
                             * bailout systematique vers 0xb077. La question est donc
                             * « sur quoi pointe le registre », pas « que vaut la
                             * comparaison ». On imprime les AR et la cellule pointee
                             * par AR1 (candidat le plus probable, ARP le dira). */
                            unsigned _arp = (s->st0 >> 13) & 7;
                            uint16_t _ea  = s->ar[_arp];
                            fprintf(stderr,
                                    "[dispatch] CHAIN-B05F pc=0x%04x op=0x%04x "
                                    "A=0x%06llx (bas=0x%04x) TC=%d ARP=%u "
                                    "AR[ARP]=0x%04x *AR[ARP]=0x%04x "
                                    "AR0=%04x AR1=%04x AR2=%04x AR3=%04x "
                                    /* [2026-08-03] data[0x7fff] : test d'une
                                     * hypothese precise. En 0xb062, `f130 7fff`
                                     * ecrase A avec 0x5294, valeur CONSTANTE quelle
                                     * que soit l'entree (verifie avant et apres le
                                     * correctif api_ram). Un resultat independant de
                                     * l'operande = l'opcode ne calcule pas, il LIT.
                                     * Si data[0x7fff] vaut 0x5294, l'immediat long
                                     * est traite comme une ADRESSE au lieu d'une
                                     * valeur — meme classe que le bug LDU *(0x0ffe)
                                     * documente en tete du handler F1xx. Si ca ne
                                     * correspond pas, l'hypothese tombe et il faudra
                                     * instrumenter le handler lui-meme. */
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
                    /* dedupe par (site, index) : un aiguillage stable ne doit pas
                     * noyer le journal — c'est la LEÇON du 03/08 sur DISPATCH SB. */
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
                /* --- TOUT appel indirect `cala A` (opcode 0xf4e3) --------------
                 * Mesure du 03/08 : aucun des cinq aiguillages ne tire, et pourtant
                 * le handler 0xa62e de la table S'EXECUTE (A=0xffa62e = l'adresse
                 * elle-meme, signature d'un cala). Il existe donc un AUTRE chemin
                 * d'appel. Plutot que de le deviner site par site, on trace chaque
                 * appel indirect avec sa cible, et on nomme celles qui nous
                 * interessent. Dedupe par (PC appelant, cible). */
                /* [2026-08-03, ELARGI] la v1 ne tracait que `cala` sur A (0xf4e3).
                 * Or 0xa62e s'execute SANS qu'aucun cala ne le vise, et il n'est
                 * atteignable ni par CALL/B direct (aucune reference dans l'image)
                 * ni par continuation (0xa62d est un ret). Il reste donc un
                 * BRANCHEMENT calcule. tic54x-opc.c donne quatre formes, masque
                 * 0xFEFF, le bit 8 choisissant l'accumulateur source :
                 *     bacc  0xF4E2   baccd 0xF6E2   cala 0xF4E3   calad 0xF6E3
                 * On les couvre toutes, sur A comme sur B. Meme erreur que mes deux
                 * precedentes : j'avais instrumente le cas attendu, pas la classe. */
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

                /* --- les points remarquables, simple comptage de passage -------- */
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

                      /* ── [2026-08-03] LA CHAINE DE L'ARMEMENT RX ──────────────
                       * Index 41 (`ld #0x29,A ; call 0xa9ea`) -> 0xa5cd. Quatre
                       * sites le demandent, aucun n'est atteint. On sonde les
                       * sites EUX-MEMES, l'ENTREE du bloc qui les contient, et le
                       * CHARGEUR qui met cette entree dans A en amont — sinon on
                       * ne saurait dire ou la chaine se rompt. */
                      {0xb220, "site 1 : ld #0x29 -> ARMEMENT RX (AR3=0x00bf)"},
                      {0xb216, "  entree du bloc du site 1"},
                      {0xb2b4, "site 2 : ld #0x29 -> ARMEMENT RX (AR3=0x0097)"},
                      {0xb2aa, "  entree du bloc du site 2"},
                      {0xb330, "site 3 : ld #0x29 -> ARMEMENT RX (AR3=0x0030)"},
                      {0xb35d, "site 4 : ld #0x29 -> ARMEMENT RX (AR3=0x0040)"},
                      /* les `ld #k16,A` qui chargent l'entree du bloc 1 puis 2 */
                      {0xaba9, "  chargeur -> 0xb216"},
                      {0xabc9, "  chargeur -> 0xb216"},
                      {0xabf1, "  chargeur -> 0xb2aa"},
                      {0xabf5, "  chargeur -> 0xb2aa"},
                      {0xabf9, "  chargeur -> 0xb2aa"},
                      {0xabfd, "  chargeur -> 0xb2aa"},
                      {0xb758, "  chargeur -> 0xb2aa"},
                      /* le dispatcher lui-meme : combien de fois, quel index */
                      {0xa9ea, "helper index->handler (A = base+index apres le add)"},

                      /* ── [2026-08-03] LA FILE DE TACHES — le vrai coeur ────────
                       * La boucle principale 0xa4ca fait `call 0xaad5` (depile) et,
                       * si A != 0, `calad A`. 0xaad5 est un DEPILEMENT sur anneau :
                       *     AR1 = data[0x434f] (ecriture)  AR0 = data[0x434e] (lecture)
                       *     vide -> A = 0 -> la boucle tourne a blanc
                       * Mesure : 31 000 tours, file vide. Donc la question n'est plus
                       * « pourquoi tel handler ne tourne pas » mais « QUI EMPILE ».
                       * L'empilement est 0xaac3 (39 appelants), anneau de 14, le
                       * handler arrive dans A, debordement -> data[0x3f92] bit 5. */
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
        {   /* [2026-07-30] XPCWATCH (CALYPSO_XPCWATCH, defaut 0) — le DSP change-t-il
             * JAMAIS de banque programme ?
             *
             * POURQUOI. La chaine FB est tracee de bout en bout et s'arrete a un seul
             * endroit : la tache arme data[0x0158..0159] = « call 0x728a », ce tremplin
             * n'est atteignable que par le slot d'IT 30 (data 0x00F8-0x00FB = « fb 0x0158 »,
             * recopie de PDROM 0xe399 par le reada de 0xb4c9), et ce slot n'est jamais
             * pris : IMR bit 14 (= vec 30 - 16, formule de calypso_dma.c:185) est
             * demasque en permanence mais l'IFR ne prend jamais que 0x0020/0x0008.
             * Or les deux pieces manquantes du puzzle vivent dans PROM1, chargee en
             * PROGRAMME 0x18000, donc dans une BANQUE : l'unique reference de tout le
             * silicium a 0xaae8 (base du tableau de handlers, table[5]=0xab77) est en
             * PROM1@0x1ab31, et un cinquieme ecrivain de 0x0158 en PROM1@0x19fe1.
             * Si XPC reste a 0 pour toujours, PROM1/2/3 sont du code MORT chez nous, et
             * ca expliquerait d'un coup l'installateur de handler absent, le publieur
             * 0x79e4 jamais atteint, et l'invocateur du slot 30.
             *
             * CE QUE CA MESURE. (a) chaque changement de XPC, avec le PC et l'opcode qui
             * le provoque, plafonne a 40 lignes ; (b) un bilan periodique du MASQUE des
             * pages vues, pour que l'ABSENCE soit un resultat lisible et pas une deduction :
             * « pages XPC vues = 0x00000001 » veut dire page 0 uniquement, donc aucune
             * banque n'est jamais entree. Sans ce bilan, zero ligne (a) serait ambigu
             * entre « aucune commutation » et « sonde non armee ».
             * NB la famille de branchements lointains EST modelisee (fb/fcall/fbacc,
             * cf. le classifieur ~l.1532) et la premiere instruction du run est un
             * `fb` depuis le vecteur de reset 0xff80 — donc si XPC ne bouge pas, ce
             * n'est pas parce que l'instruction manque. */
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
        {   /* @BEQUILLE — FORCE_VEC  (CALYPSO_FORCE_VEC=<n>, VALEUR, defaut inerte)
             *   masque  : la SOURCE MATERIELLE d'une interruption DSP que le modele
             *             n'implemente pas. Mesure du 30/07 : la tache FB arme
             *             data[0x0158..0159] = « call 0x728a » (13x), ce tremplin n'est
             *             atteignable que par le slot d'IT 30 (data 0x00F8-0x00FB =
             *             « fb 0x0158 », recopie de PDROM 0xe399 par le reada de 0xb4c9),
             *             IMR bit 14 (= vec-16) est demasque en permanence dans tous les
             *             IMR mesures, et l'IFR ne prend JAMAIS que 0x0020/0x0008 : rien
             *             ne leve jamais ce bit. Quatre pistes eliminees par la mesure
             *             (course tache/boucle, IMR masque, IT de fin de DMA, banque
             *             programme jamais commutee — XPC voit bien les pages 0 ET 1).
             *   retirer : des qu'on sait QUELLE ligne materielle porte ce vecteur sur le
             *             Calypso. C'est une lacune DOCUMENTAIRE (la table des
             *             interruptions du DSP n'est nulle part dans le depot), pas un
             *             bug a chercher au grep.
             *   NB      : ce gate n'est PAS un correctif, c'est un test decisif — il dit
             *             si tout l'aval du vecteur est sain. Si oui, le probleme se
             *             reduit a identifier et cabler la source.
             *   ⚠ Avec SEED5AC8 et DISPATCH_INSTALL, ca fait TROIS bequilles simultanees :
             *             on ne mesure plus le natif, on mesure ce qu'il ferait si trois
             *             trous etaient bouches. Acceptable pour un test, pas comme
             *             configuration.
             *
             * Declencheur : on n'injecte que quand le handler FB est EFFECTIVEMENT arme
             * (data[0x0159] == 0x728a), donc jamais dans la fenetre ou la boucle
             * principale a remis son handler leger 0x7242 — sinon le test mesurerait
             * l'autre chemin. Cadence CALYPSO_FORCE_VEC_PERIOD insn (defaut 65536 = une
             * par trame), plafond dur 200 injections, 20 lignes de journal. */
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
        {   /* [2026-07-27] TRACEFROM : voir en-tete du patch. */
            static int _tf = -1; static uint16_t _tfpc = 0; static int _tfd = 0;
            static int _tfn2 = 24;   /* CALYPSO_TRACEFROM_N : longueur du dump */
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
        {   /* [2026-07-28] CORROUT : voir en-tete du patch. */
            static int _co = -1; static int _in_k = 0; static unsigned _con = 0;
            if (_co < 0) _co = calypso_gate("CALYPSO_CORROUT", 0);
            if (_co) {
                if (exec_pc >= 0xa070 && exec_pc <= 0xa0a0) { _in_k = 1; }
                else if (_in_k) {   /* on vient de QUITTER le noyau MAC */
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
        {   /* [2026-07-28] VECTAB : voir en-tete du patch. */
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
        {   /* [2026-07-28] SCANDATA : voir en-tete du patch. */
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
                if (++_seen >= 2) {   /* apres l init des tables */
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
        {   /* [2026-07-28] DISPWATCH : voir en-tete du patch. */
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
        {   /* [2026-07-28] SCANREF : voir en-tete du patch. */
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
        {   /* [2026-07-27] SCANFB : voir en-tete du patch. */
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
        {   /* [2026-07-27] FBENTRY : voir en-tete du patch. */
            static int _fe = -1; static int _dmp = 0; static int _in = 0; static int _after = 0;
            static unsigned _n = 0;
            if (_fe < 0) _fe = calypso_gate("CALYPSO_FBENTRY", 0);
            if (_fe) {
                if (exec_pc >= 0x75e0 && exec_pc <= 0x79f0) {   /* inclut le sous-prog 0x75e8 */
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
        {   /* [2026-07-27] SCAN43D8 : voir en-tete du patch. */
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
        {   /* [2026-07-27] SLOTSRC : voir en-tete du patch. */
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
        {   /* [2026-07-27] FBCALL : voir en-tete du patch. */
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
        {   /* [2026-07-27] DISPIDX : voir en-tete du patch. */
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
        {   /* [2026-07-27] DISPTAB-DUMP : contenu de la table au dispatcher. */
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
        {   /* [2026-07-27] TASKGO : voir en-tete du patch. */
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
        /* [2026-07-29] La sonde TASKMD a vécu une demi-journée ici. Elle
         * traçait les fronts de d_task_md avec les deux vues (data[]/api_ram) —
         * l'idée est bonne, elle est reprise et généralisée par le moniteur
         * mailbox (calypso_mailbox.c), qui couvre TOUTES les cellules dans les
         * QUATRE sens au lieu d'une paire dans un seul. Voir mailbox.log :
         *   grep '0x0804' mailbox.log
         */
        {   /* @BEQUILLE — DISPATCH_INSTALL  (CALYPSO_DISPATCH_INSTALL=0xNNNN,
             *              VALEUR, defaut unset = inerte)
             *   masque  : la routine qui doit installer un handler de tache dans
             *             data[0x43d8]. Mesure du 2026-07-29 : cette cellule ne
             *             recoit QUE le bouchon 0xab38 (=RET), pose une fois par
             *             0xbb00, et rien ne l'ecrit plus jamais — verifie
             *             statiquement (2 references dans les 5 ROM) et au runtime.
             *             0xb01c la relit a CHAQUE trame et 0xb01e l'appelle : le
             *             DSP acquitte donc chaque tache sans rien faire.
             *   retirer : des qu'on sait QUELLE routine doit peupler 0x43d8. Ce
             *             gate n'est PAS un correctif, c'est un test decisif : il
             *             dit si le reste de la chaine s'allume quand le handler
             *             est bon. Si oui, la question se reduit a l'installateur.
             *
             * Candidats mesures : 0xab77 = table[5], lit d_task_md (*AR1(0x0004))
             * et teste son bit 15 — le plus plausible pour la tache FB.
             *                     0xb284 = fcall 0x770a, l'entree de la banque du
             * publieur, jamais atteinte autrement.
             *
             * On n'ecrase QUE le bouchon : si un jour quelqu'un installe un vrai
             * handler, ce gate ne le masquera pas — il se taira. */
            static int _di2 = -2, _ditask = -1; static unsigned _din2 = 0;
            if (_di2 == -2) {
                const char *e = getenv("CALYPSO_DISPATCH_INSTALL");
                _di2 = (e && *e) ? (int)strtoul(e, NULL, 0) : -1;
                const char *t = getenv("CALYPSO_DISPATCH_INSTALL_TASK");
                _ditask = (t && *t) ? atoi(t) : -1;   /* -1 = n'importe quelle tache */
                if (_di2 >= 0)
                    fprintf(stderr, "[c54x] DISPATCH-INSTALL arme : data[0x43d8] "
                            "<- 0x%04x pour d_task_md=%s (BEQUILLE, voir "
                            "calypso_c54x.c)\n", (unsigned)_di2,
                            _ditask < 0 ? "toute tache non nulle" : "la tache demandee");
            }
            /* [2026-07-30] @BEQUILLE — FORCE_TASK  (CALYPSO_FORCE_TASK=<n>, defaut OFF)
             *
             *   c'en est une : on commande une tache au DSP a la place de l'ARM.
             *   masque  : l'incapacite de la L1 a MAINTENIR sa commande CCCH. Mesure
             *             du 30/07 : `task=24` est bien commandee (7 a 34 fois selon
             *             les runs), les rapports de burst sont acceptes depuis le
             *             correctif du burst-id — mais le mobile ne campe pas, donc il
             *             reselectionne toutes les ~10 s, et chaque reselection fait
             *             `L1-RESET: d_dsp_page=0` (17 fois sur un run de 65 s), ce qui
             *             balaie l'enclenchement. Le DSP n'a jamais une fenetre longue.
             *   ce qu'elle NE fabrique PAS : le resultat. On pose la COMMANDE, pas
             *             `a_cd`. `WATCH-ACD` reste donc un juge honnete — si le DSP
             *             ecrit a_cd sous cette bequille, c'est bien LUI qui l'a fait.
             *   retirer : quand le mobile campe et commande le CCCH tout seul.
             *
             * On ne LUTTE PAS contre l'ARM : on ne remplit que les passages ou les
             * deux pages d'ecriture sont vides (aucune commande en cours). Une vraie
             * commande de l'ARM a donc toujours la priorite.
             * Plafond de journal : 20 lignes.
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
                /* [2026-07-30, v2 — corrige par la lecture du firmware]
                 *
                 * v1 ecrivait `d_task_md` (mot 4 de la page d'ecriture). FAUX pour
                 * une tache de RECEPTION : osmocom-bb `calypso/dsp.c:479` montre la
                 * vraie commande, et elle a TROIS champs :
                 *     void dsp_load_rx_task(task, burst_id, tsc) {
                 *         db_w->d_task_d      = task;          // mot 0
                 *         db_w->d_burst_d     = burst_id;      // mot 1
                 *         db_w->d_ctrl_system |= tsc & 0x7;    // mot 16
                 *     }
                 * `d_task_md` (mot 4) ne sert qu'au FB/SB (prim_fbsb.c:279,379), et
                 * il s'accompagne la-bas d'un parametre NDB (`d_fb_mode`). Bref : une
                 * commande n'est jamais un seul mot — il y a une INIT.
                 *
                 * Disposition (T_DB_MCU_TO_DSP, include/calypso/dsp_api.h:82) :
                 *     mot 0 d_task_d | 1 d_burst_d | 4 d_task_md | 16 d_ctrl_system
                 * Bases : page 0 = 0x0800, page 1 = 0x0814. D'ou :
                 *     d_task_d      0x0800 / 0x0814
                 *     d_burst_d     0x0801 / 0x0815
                 *     d_ctrl_system 0x0810 / 0x0824   <- les 2 cellules vues en
                 *                                        CYCLE-TRACE sans etre nommees
                 *
                 * La PAGE est celle que l'ARM designe : `d_dsp_page = B_GSM_TASK |
                 * w_page` (dsp.c:471), donc son bit 0.
                 *
                 * Le burst_id CYCLE 0,1,2,3 : le firmware appelle RX NB quatre fois
                 * et ne recupere les donnees qu'a la 4e reponse (wiki HardwareCalypsoDSP).
                 * TSC : CALYPSO_FORCE_TASK_TSC, defaut 7 (= BCC mesure sur ce reseau).
                 */
                if (_ft >= 0 && s->data[0x0800] == 0 && s->data[0x0814] == 0 &&
                    s->data[0x0804] == 0 && s->data[0x0818] == 0) {
                    static unsigned _fbid = 0;
                    static int _ftsc = -1;
                    if (_ftsc < 0) {
                        const char *t = getenv("CALYPSO_FORCE_TASK_TSC");
                        _ftsc = (t && *t) ? (int)strtol(t, NULL, 0) : 7;
                    }
                    /* [2026-07-30, v3] On tient NOTRE page : `d_dsp_page` est
                     * remis a 0 par chaque `L1-RESET` (30 fois par minute mesure),
                     * donc le lire donnerait page 0 en permanence — c'est ce qu'on
                     * observait. Le firmware, lui, tient `w_page` en RAM ARM et le
                     * bascule a chaque fin de scenario. On fait pareil. */
                    static unsigned _fpg = 0;
                    unsigned pg   = _fpg & 1u;
                    uint16_t base = pg ? 0x0814 : 0x0800;
                    uint16_t ctrl = pg ? 0x0824 : 0x0810;
                    uint16_t bid  = (uint16_t)(_fbid++ & 3u);

                    /* ─────────────────────────────────────────────────────────
                     * [2026-08-03] CORRECTIF — ces trois ecritures n'atteignaient
                     * PAS le DSP. Elles ne touchaient que `s->data[]`, or pour
                     * toute adresse de la fenetre API le DSP lit `s->api_ram[]` :
                     *     data_read_locked() :
                     *       if (addr >= C54X_API_BASE && addr < ...+C54X_API_SIZE)
                     *           v = s->api_ram[addr - C54X_API_BASE];
                     * La tache forcee atterrissait donc dans un tableau que
                     * personne ne lit. Le miroir etait DEJA fait quinze lignes plus
                     * bas pour `d_dsp_page` (data[] ET api_ram[]) — la contrainte
                     * etait connue, elle n'avait simplement pas ete appliquee ici.
                     *
                     * MESURE QUI L'ETABLIT (sonde LD-TRACE, run du 03/08 19:52) :
                     *     pc=0xb05f addr=0x0814 data[addr]=0x0018 val_lue=0x0000
                     * L'adresse resolue est bonne, la cellule `data[]` contient
                     * bien 24 (ALLC) — et la lecture rend 0. Consequence en chaine,
                     * toute mesuree : le dispatcher compare 0 aux constantes
                     * 12/30/34, sort au premier branchement vers 0xb077, la
                     * resolution d'index n'est jamais atteinte, l'index 41
                     * (armement RX, `0xa5cd`) n'est jamais demande, `A_CD-WR = 0`.
                     *
                     * ⚠️ CECI RESTE UNE BEQUILLE (marqueur BEQUILLE, cf. le bloc
                     * FORCE_TASK ci-dessus) :
                     *   1. c'en est une : on commande la tache a la place de l'ARM ;
                     *   2. ce qu'elle masque : que la L1 ne commande pas le CCCH
                     *      assez souvent par elle-meme ;
                     *   3. quand la retirer : quand le mobile campe et commande le
                     *      CCCH seul. Ce correctif ne fait que la rendre EFFECTIVE —
                     *      il ne la legitime pas.
                     * ⚠️ Ce correctif change le comportement (la bequille agit
                     *    enfin). A valider sous charge avant d'effacer la condition. */
                    s->data[base + 0] = (uint16_t)_ft;         /* d_task_d      */
                    s->data[base + 1] = bid;                   /* d_burst_d     */
                    s->data[ctrl]    |= (uint16_t)(_ftsc & 7); /* d_ctrl_system */
                    if (s->api_ram) {
                        s->api_ram[base + 0 - C54X_API_BASE] = (uint16_t)_ft;
                        s->api_ram[base + 1 - C54X_API_BASE] = bid;
                        s->api_ram[ctrl - C54X_API_BASE]    |= (uint16_t)(_ftsc & 7);
                    }

                    /* FIN DE SCENARIO — sans elle, on remplit une page que personne
                     * n'ouvre. Port exact de `dsp_end_scenario()` (osmocom-bb
                     * calypso/dsp.c:466) :
                     *     ndb->d_dsp_page = B_GSM_TASK | w_page;   // annonce
                     *     w_page ^= 1;                             // bascule
                     *     tpu_dsp_frameirq_enable(); tpu_frame_irq_en(1,1);
                     * B_GSM_TASK = (1<<1) = 0x0002 (l1_environment.h:249), la page
                     * est le bit 0 -> d_dsp_page vaut 0x0002 ou 0x0003.
                     * La 3e partie (l'IT de trame) est deja fidele chez nous :
                     * calypso_tpu.c teste TPU_CTRL_DSP_EN puis ICTRL_DSP_FRAME en
                     * actif-bas, exactement ce que le firmware pose. */
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

            /* [2026-07-30] Exclusion mutuelle : en mode « init » la greffe de
             * data_write_locked() fait le travail, ce bloc-ci doit se taire —
             * sinon on retrouve la lutte qu'on cherche justement a supprimer. */
            static int _at_init = -1;
            if (_at_init < 0) { const char *m = getenv("CALYPSO_DISPATCH_INSTALL_AT");
                                _at_init = (m && strcmp(m, "init") == 0) ? 1 : 0; }
            if (_di2 >= 0 && !_at_init && exec_pc == 0xb01c) {
                uint16_t _md = s->data[0x0804] ? s->data[0x0804] : s->data[0x0818];
                /* [2026-07-29, v2] Deux corrections du premier jet :
                 *  - il ne s'installait QU'UNE FOIS (condition « == 0xab38 »), donc
                 *    le handler restait fige apres le premier passage ;
                 *  - il tirait sur n'importe quelle tache — en pratique la 1 (PM)
                 *    est arrivee avant la 5 (FB), donc le test ne mesurait pas ce
                 *    qu'on voulait. D'ou _TASK.
                 * On reinstalle donc a CHAQUE dispatch de la tache visee, mais
                 * seulement si le slot porte encore le bouchon ou notre propre
                 * valeur : si un jour une vraie routine installe autre chose, on
                 * se tait au lieu de l'ecraser. */
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
        {   /* [2026-07-29] DISPCALL — le dispatcher de tache appelle-t-il
             * REELLEMENT le slot data[0x43d8], et ou atterrit-on ?
             *
             * CONDITION DE DECLENCHEMENT (pas une adresse a tracer) :
             * exec_pc == 0xb01c, le relecteur du slot, dont on SAIT qu'il
             * s'execute — la bequille DISPATCH_INSTALL y tire deja. On note
             * ensuite les PC REELLEMENT executes pendant les CALYPSO_DISPCALL_N
             * (defaut 10) instructions suivantes : aucune hypothese sur l'ISA
             * ni sur « 0xb01e est le CALA », c'est la trajectoire mesuree.
             *
             * CE QUE CA TRANCHE :
             *   - la sequence passe par la valeur du slot -> le handler EST
             *     appele, la question devient « que fait-il » (enchainer
             *     CALYPSO_TRACEFROM=<valeur du slot>) ;
             *   - elle n'y entre jamais -> ce n'est pas cette cellule qui est
             *     appelee, toute la piste 0x43d8 tombe ;
             *   - silence complet -> 0xb01c n'est pas atteint, information
             *     encore differente (et verifiable : la ligne d'armement de
             *     DISPATCH_INSTALL, elle, s'imprime toujours).
             *
             * PLAFOND : la trajectoire se repete a chaque trame, donc on
             * REPLIE — une ligne uniquement quand la sequence, le slot ou
             * d_task_md CHANGENT, avec le nombre de repetitions avalees ;
             * 40 lignes au total. Gate CALYPSO_DISPCALL (defaut 0). */
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
                if (exec_pc == 0xb01c) {          /* (re)armement */
                    _dc_armed = 1; _dc_k = 0;
                    _dc_slot = s->data[0x43d8];
                    _dc_md = s->data[0x0804] ? s->data[0x0804] : s->data[0x0818];
                }
                if (_dc_armed) {
                    _dc_seq[_dc_k++] = s->pc;     /* PC APRES l'instruction executee */
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
        {   /* [2026-07-29] DMAQ — la file de requetes DMA du firmware, au moment
             * exact du test. Toutes les deductions indirectes ont echoue :
             *   - « pc=0xaa83 jamais atteint » : faux, les traces de PC sont gatees ;
             *   - « le consommateur n'est jamais appele » : faux, il l'est ;
             *   - « 6 lectures a 0xaa87 » : faux, le moniteur mailbox REPLIE les
             *     lectures repetees a l'identique.
             * On mesure donc directement les registres et les cellules au PC du
             * `banz`, la seule chose qui tranche. Plafonnee a 40 lignes.
             * Gate CALYPSO_DMAQ. */
            static int _dq = -1; static unsigned _dqn = 0;
            if (_dq < 0) _dq = calypso_gate("CALYPSO_DMAQ", 0);
            /* [2026-07-29, v2] CONDITION DE DECLENCHEMENT, pas adresse. La v1
             * logguait les 3 PC sans condition : ses 40 lignes ont ete consommees
             * au demarrage (insn ~13 M), AVANT le moindre empilement — elle a
             * donc mesure « file vide » et ne prouvait rien. On ne journalise
             * desormais que les instants OU IL Y A QUELQUE CHOSE A VOIR :
             *   - la file est NON vide au test (d[433f] != d[433e]), ou
             *   - le pointeur d'ecriture RECULE (un reset efface la file).
             * Plus le tout premier passage, pour l'ancrage temporel. */
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
        {   /* [2026-07-27] AB38 : voir en-tete du patch. */
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
        {   /* [2026-07-27] FBROUTE : voir en-tete du patch. */
            static int _fr = -1;
            if (_fr < 0) _fr = calypso_gate("CALYPSO_FBROUTE", 0);
            if (_fr) {
                {   /* ─────────────────────────────────────────────────────────
                     * [2026-08-03] FBGATE-TRACE — le dernier verrou, pas a pas.
                     *
                     * OU ON EN EST. Depuis que le DMA livre les echantillons, la
                     * routine FB s'execute (elle n'etait JAMAIS entree avant) et la
                     * machine a etats `@0x7e` avance 0 -> 1 -> 2 -> 3. Le scan du
                     * ROM donne les CINQ seuls ecrivains de cette cellule :
                     *     0x77a8 ST #1   0x77ae ST #2   0x77b4 ST #3
                     *     0x795d ST #4   <- le seul qui satisferait la garde 0x79e3
                     * Les quatre premiers sont MESURES (FBCNT-WR). `0x795d` ne l'est
                     * jamais : le high-water plafonne a `0x794e`, quinze mots avant.
                     *
                     * CE QU'IL Y A EN 0x794e (dump ROM) :
                     *     0x7949 f310 0005   SUB #5
                     *     0x794b 6ff8 0c3d   acces absolu a data[0x0c3d]
                     *     0x794e fc47        RETOUR CONDITIONNEL — toujours pris
                     * La routine ne manque pas de donnees : elle DECIDE de sortir.
                     *
                     * ⚠️ POURQUOI UNE SONDE ET PAS UN DESASSEMBLAGE. Je lis `0xfc47`
                     * comme un retour conditionnel par analogie avec `0xfc00` (RET) et
                     * `0xfc45` vu dans le dispatcher juste avant ; et `6ff8 0c3d` comme
                     * un acces absolu. Ces DEUX lectures sont des hypotheses. Decoder
                     * des conditions a la main est exactement ce qui a produit 3
                     * fausses pistes sur 3 le 30/07. On mesure le chemin REEL et les
                     * drapeaux ; la conclusion sortira du run.
                     *
                     * A LIRE : si l'execution passe 0x794e une seule fois, la
                     * condition n'est pas constante et il faudra correler. Si elle
                     * sort a chaque passage, les drapeaux (TC/C) et data[0x0c3d]
                     * diront laquelle. `data[0x0c3d]` n'est PAS dans la plage que le
                     * DMA remplit (0x0cce..0x0cce+296) — 145 mots avant. */
                    if (exec_pc >= 0x7944 && exec_pc <= 0x7962) {
                        static unsigned _n = 0;
                        if (_n < 120) {
                            _n++;
                            fprintf(stderr,
                                    /* [2026-08-03] AR4 ajoute — c'est LUI qui decide.
                                     * Desassemblage croise (table binutils + tic54x-dis) :
                                     *     0x7944 add *AR4, A ; 0x7945 sub #2, A
                                     *     0x794e rc ALEQ      (retour si A <= 0)
                                     * Mesure : *AR4 donne 1 puis 2 (mot haut), donc
                                     * A = -1 puis 0 -> la garde sort a chaque fois.
                                     * Il faut *AR4 > 2. Ressemble a un compteur de
                                     * correlations qui doit depasser un seuil de 2 :
                                     * le DSP detecte quelque chose, pas assez.
                                     * On imprime le POINTEUR et la CELLULE pointee —
                                     * la question est desormais « qui remplit ce
                                     * tampon », pas « que vaut la condition ».
                                     * ⚠️ J'ai d'abord ecrit que la condition ne
                                     * dependait pas du signal : FAUX, *AR4 varie. */
                                    "[c54x] FBGATE pc=0x%04x op=0x%04x A=0x%06llx "
                                    "B=0x%06llx TC=%d C=%d AR4=0x%04x *AR4=0x%04x "
                                    "(data[]=0x%04x) data[0x0c3d]=0x%04x "
                                    "data[0x41fe]=0x%04x insn=%u\n",
                                    exec_pc, prog_fetch(s, exec_pc),
                                    (unsigned long long)(s->a & 0xFFFFFFULL),
                                    (unsigned long long)(s->b & 0xFFFFFFULL),
                                    (s->st0 >> 12) & 1, (s->st0 >> 11) & 1,
                                    /* [2026-08-03] CORRIGE : la v1 lisait
                                     * `s->data[AR4]`. FAUX pour toute adresse de la
                                     * fenetre API — le DSP y lit `api_ram[]`, un
                                     * AUTRE tableau (cf. data_read_locked). AR4 vaut
                                     * 0x0cd0, donc en pleine fenetre API et en plein
                                     * tampon DMA : la v1 affichait 0x0000 alors que
                                     * la cellule est alimentee. C'est EXACTEMENT le
                                     * defaut corrige le matin meme pour FORCE_TASK,
                                     * reproduit dans mon propre instrument. On
                                     * imprime les DEUX vues pour que la divergence
                                     * reste visible au lieu d'etre supposee. */
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
                        unsigned _ea = _dp * 0x80 + 0x7e;   /* dma(0x7e) DP-relatif */
                        fprintf(stderr, "[c54x] FBROUTE jalon PC=0x%04x #%u A=0x%06llx "
                                "DP=0x%03x dma(0x7e)=data[0x%04x]=0x%04x (garde veut 4) insn=%u\n",
                                exec_pc, _ms[_k], (unsigned long long)(s->a & 0xFFFFFFULL),
                                _dp, _ea, (_ea < 0x10000) ? s->data[_ea] : 0, s->insn_count);
                    }
                }
            }
        }
        if (exec_pc == 0x93a5) {   /* consommateur DARAM 0x2a00 (AR3 post-inc) = VRAIE entree corr */
            static int _b2c = -1; static unsigned _b2cn = 0;
            if (_b2c < 0) _b2c = calypso_gate("CALYPSO_B2SEQ", 0);
            if (_b2c && _b2cn < 8) { _b2cn++;
                fprintf(stderr, "[c54x] B2SEQ-IN 0x2a00@0x93a5 (I,Q)x16:");
                for (int _i = 0; _i < 16; _i++)
                    fprintf(stderr, " (%d,%d)", (int)(int16_t)s->data[0x2a00 + 2*_i], (int)(int16_t)s->data[0x2a00 + 2*_i + 1]);
                fprintf(stderr, "\n"); }
        }
        {
            /* [2026-07-27] DARAM-DUMP (gated CALYPSO_DARAM_DUMP) : voir en-tete.
             * Ecrit le buffer d entree corr en binaire IQ16 -> mesurable par
             * tools/corr_iq.py --src bursts (coh/dphi), pas juge a l oeil. */
            static int _dd = -1; static FILE *_ddf = NULL; static unsigned _ddn = 0;
            static uint16_t _ddpc = 0x9ac0; static unsigned _ddmax = 200;
            static uint16_t _ddaddr = 0x2a00;   /* base filmee, cf. CALYPSO_DARAM_DUMP_ADDR */
            if (_dd < 0) {
                const char *e = getenv("CALYPSO_DARAM_DUMP");
                _dd = (e && *e && strcmp(e, "0")) ? 1 : 0;
                if (_dd) {
                    const char *path = (strcmp(e, "1") == 0) ? "/dev/shm/daram_2a00.cfile" : e;
                    const char *p = getenv("CALYPSO_DARAM_DUMP_PC");
                    if (p && *p) _ddpc = (uint16_t)strtol(p, NULL, 0);
                    const char *m = getenv("CALYPSO_DARAM_DUMP_MAX");
                    if (m && *m) _ddmax = (unsigned)atoi(m);
                    /* [2026-07-30] La base filmee etait CODEE EN DUR a 0x2a00.
                     * Consequence : sur tout banc qui livre ailleurs (les profils
                     * natifs livrent en CALYPSO_BSP_DARAM_ADDR=0x4c00), la sonde
                     * filmait un tampon que personne n'alimente -- et le seul
                     * instrument NON RACY du projet etait inutilisable. Le
                     * monitor QEMU, lui, lit hors du thread CPU : le BSP reecrit
                     * les 296 mots pendant le `xp`, d'ou des coh ~0.5 et un pic
                     * FFT qui derive (raccord de deux bursts). Cette sonde-ci
                     * ecrit depuis le thread CPU : elle est coherente par
                     * construction. Il ne lui manquait que son adresse. */
                    const char *ad = getenv("CALYPSO_DARAM_DUMP_ADDR");
                    if (ad && *ad) _ddaddr = (uint16_t)strtol(ad, NULL, 0);
                    fprintf(stderr, "[c54x] DARAM-DUMP base=0x%04x "
                            "(CALYPSO_DARAM_DUMP_ADDR)\n", _ddaddr);
                    _ddf = fopen(path, "wb");
                    fprintf(stderr, "[c54x] DARAM-DUMP armed pc=0x%04x max=%u -> %s (%s)\n",
                            _ddpc, _ddmax, path, _ddf ? "ok" : "FOPEN FAILED");
                }
            }
            /* [2026-07-27] C2 : ne filmer QUE les passages en recherche FCCH.
             * Sans ce garde, le cap _ddmax etait consomme des le boot, pendant
             * que d_fb_mode[0x08f9]==0 -> le dump ne montrait pas la phase FB et
             * on en concluait a tort « le buffer ne contient jamais de FCCH ».
             * Override CALYPSO_DARAM_DUMP_ANYMODE=1 pour revenir a l ancien. */
            static int _ddany = -1;
            if (_ddany < 0) { const char *e = getenv("CALYPSO_DARAM_DUMP_ANYMODE");
                              _ddany = (e && atoi(e) > 0) ? 1 : 0; }
            if (_dd && _ddf && exec_pc == _ddpc && _ddn < _ddmax &&
                (_ddany || s->data[0x08f9] != 0)) {
                unsigned char hdr[12];
                unsigned nw = 296;   /* 0x2a00..0x2b27 = 296 mots = 148 paires I/Q */
                hdr[0]='I'; hdr[1]='Q'; hdr[2]='1'; hdr[3]='6';
                unsigned _fnv = calypso_daram_last_fn;   /* fn GSM reel du dernier depot */
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
                /* [2026-07-27] DARAM-SANITY : verdict en run sur ce qu'on vient
                 * de dumper. coh = |Sum z[k+1].conj(z[k])| / Sum|z[k+1]||z[k]| ;
                 * dphi = arg(Sum ...). Le kernel FB veut du FCCH @1SPS => dphi
                 * = +pi/2 (+1.571). +0.393 = 4 SPS non decime (remede nomme). */
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
            /* [2026-07-27] B2SEQ (gated CALYPSO_B2SEQ) : dump 16 paires (I,Q) de
             * 0x2a00 (VRAIE entree corr, depot BSP/ADC). Pattern Fs/4 = FCCH :
             * (a,0)(0,a)(-a,0)(0,-a).. ; quasi-constant = DC sans ton (entree vide). */
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
            /* [2026-07-27] B2AR (gated CALYPSO_B2AR) : ou pointent les AR du corr
             * + valeur lue. AR5/AR3 dans [0x2a00..0x2b27] => lit la FCCH ; hors =>
             * lit a cote (RANK3, pointeur mal initialise). */
            { static int _b2a = -1; static unsigned _b2an = 0;
              if (_b2a < 0) _b2a = calypso_gate("CALYPSO_B2AR", 0);
              if (_b2a && _b2an < 12) { _b2an++;
                  int _a3in = (s->ar[3] >= 0x2a00 && s->ar[3] < 0x2b28);
                  int _a5in = (s->ar[5] >= 0x2a00 && s->ar[5] < 0x2b28);
                  fprintf(stderr, "[c54x] B2AR @0x9ac0 AR2=%04x AR3=%04x[%d]%s AR4=%04x AR5=%04x[%d]%s\n",
                          s->ar[2], s->ar[3], (int)(int16_t)s->data[s->ar[3]], _a3in?"IN":"OOB",
                          s->ar[4], s->ar[5], (int)(int16_t)s->data[s->ar[5]], _a5in?"IN":"OOB"); } }
            /* [2026-07-27] B2 (gated CALYPSO_B2) : module accu A/B + max/indice
             * sur les 296 mots entree(0x2a00) & workspace(0x2c00). Tranche nul vs
             * plat-sans-pic. */
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
                /* angle=atan2(B,A) calculé à l'analyse (pas de math.h ici). */
                fprintf(stderr, "[c54x] CORR-ABG #%u A=%lld B=%lld | "
                        "AR2=%04x[%04x] AR3=%04x[%04x] AR4=%04x[%04x] AR5=%04x[%04x] insn=%u\n",
                        cr, (long long)a, (long long)b,
                        s->ar[2], s->data[s->ar[2]], s->ar[3], s->data[s->ar[3]],
                        s->ar[4], s->data[s->ar[4]], s->ar[5], s->data[s->ar[5]],
                        s->insn_count);
                cr++;
            }
        }

        /* === CORR-PEAK probe (2026-05-30) : au store du TOA (PC=0x9ac0, STL A
         * dans a_sync_demod) dumper A/B complets + AR + T + la fenêtre d'entrée
         * lue (buffer BSP 0x2a00) → voir comment le corrélateur dérive le TOA
         * (offset peak, wrap, référence) à partir d'une FCCH pourtant correcte.
         * Cap 40, ~zéro coût hors site. */
        if (exec_pc == 0xa0e7 || exec_pc == 0x9ac0) {
            static unsigned cp_log = 0;
            /* Ne fire QUE quand une vraie I/Q est présente (input non-nul),
             * sinon le cap est gaspillé sur le boot (buffer vide). On teste
             * 0x2a00 (BSP write) ET 0x2c00 (où AR3 pointe = lecture corr). */
            /* Gate sur RX buffer 0x2a00 NON-NUL : ne fire que quand une vraie
             * I/Q est présente (sinon gaspillé au boot/vide). Capture la vraie
             * corrélation FCCH dès que le BSP livre. */
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

        /* Track A writes (probe 3, post-exec). exec_pc = PC qui vient
         * d'exécuter. Si A a changé, c'est cet opcode qui a écrit A. */
        if (s->a != a_before_exec) {
            p_last_a_pc  = exec_pc;
            p_last_a_val = s->a;
        }
        /* === END CALA-70C3 FORENSIC PROBES === */

        /* === BOOT-BRANCH probe : control-flow skeleton for boot phase ===
         * Hunt the opcode that branches OVER the DSP init code (which should
         * set SP=0x5AC8 + AR4/AR5 + IMR=0xFFFF). IMR-W trace localised the
         * collateral damage to PC=0xf03a (insn=262501) and PC=0x8ebc
         * (insn=5247868) via stale ARx=0 → STH B,*ARx+ → MMR_IMR=0. The bad
         * branch happens earlier. Log every PC discontinuity (branch, call,
         * return, IRQ entry) while insn_count <= 300000 + snapshot SP/IMR/
         * AR4/AR5 — visualizes when each register got armed (or never did). */
        if (s->insn_count <= 300000) {
            /* c54x_exec_one does NOT advance s->pc for sequential insn — that
             * happens in `s->pc += consumed` further down. So a real branch
             * (or CALL/RET/IRQ entry) is the only thing that modifies s->pc
             * during the exec itself. */
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

        /* INT3-CYCLE-TRACE (env CALYPSO_INT3_CYCLE_TRACE=1, c web reframe night5) :
         * record branch decisions during INT3 ISR cycle. */
        int3_cycle_track_branch(s, exec_pc, exec_op, consumed);

        /* Detect SP changes — only log after init (insn > 490M) */
        if (s->sp != sp_before && s->insn_count > 490000000) {
            static int sp_leak_log = 0;
            if (sp_leak_log < 100) {
                C54_LOG("SP %+d PC=0x%04x op=0x%04x SP 0x%04x→0x%04x insn=%u",
                        (int16_t)(s->sp - sp_before), exec_pc, exec_op, sp_before, s->sp, s->insn_count);
                sp_leak_log++;
            }
        }

        /* === SP-FLOOR guard + delta histogram (2026-05-27, c web review) ===
         * Trip on FIRST SP descent below SP_FLOOR — that snapshot is BEFORE
         * the MMR auto-corruption (SP at MMR data[0x18]), so it captures the
         * cause not the crash. Plus running delta histogram to identify
         * leaking call/ret pairs (FAR push 2 vs near pop 1 = -1 per pair).
         * Active by default — minimal cost (a few branches per insn). */
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
                /* Log first 80 SP changes + every 5000 — enough to characterize
                 * the leaking call/ret pair WITHOUT drowning the 1.3 GB log. */
                sp_delta_log_n++;
                if (sp_delta_log_n <= 80 || (sp_delta_log_n % 5000) == 0) {
                    C54_LOG("SP-Δ #%llu PC=0x%04x op=0x%04x XPC=%u Δ%+d  SP 0x%04x→0x%04x insn=%u",
                            (unsigned long long)sp_delta_log_n,
                            exec_pc, exec_op, s->xpc & 0x3,
                            delta, sp_before, s->sp, s->insn_count);
                }
            }

            /* === Plan B detectors (run once per insn AFTER exec_one) === */
            /* (1) A-write ring : track each modification of s->a */
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

        /* v2 SP observability — only when CALYPSO_TRAP_OOR=1.
         * (a) sp_trail[256] : |Δ|>32 events (scheduler reloads + big allocs)
         * (b) sp_low watermark : every new low, PC-coalesced power-of-10
         * Gated to insn>33754 (after init stack 0x9022→0x5ac8 normal). */
        {
            static int trap_armed = -1;
            if (trap_armed < 0) {
                const char *e = cdbg_env("TRAP-OOR"); (void)e;
                /* TRAP-OOR RETIRED 2026-05-29 : c'était l'analyse de descente
                 * SP / SP-CATASTROPHE, résolue (0x70c3 self-CALA / DROM-LUT /
                 * stub). Le site 2 faisait s->running=0 au checkpoint 4.2M =
                 * LE bottleneck (CALYPSO_DEBUG=ALL haltait le DSP). Forcé OFF. */
                trap_armed = 0;
            }
            if (trap_armed && s->sp != sp_before && s->insn_count > 33754) {
                int16_t delta = (int16_t)(s->sp - sp_before);
                uint16_t a_low = (uint16_t)(s->a & 0xFFFF);

                /* SP-HIST per-PC accounting déplacé en TOP-of-loop chokepoint
                 * (fix v6 2026-05-24) — bypass-proof. Voir L6773.
                 * Pas d'appel ici sinon double-count. */

                /* (a) trail — only big jumps (skip push/pop ±1..32 noise) */
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

                /* (b) sp_low watermark — fires on any new low (incl Δ=-1). */
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

        /* SP-LEDGER + SP-INTO-MMR probes RETIRÉS 2026-05-23 :
         * info diagnostic déjà extraite (irq_entries=1 sur 144s, SP wrap
         * via stack-relative writes en MMR). Ces probes fire à CHAQUE
         * instruction → overhead non-négligeable sur DSP throughput
         * (mesuré 9.1M insn/s vs 10M required = 9% slow). Reviendront en
         * cas de régression. SP-CATASTROPHE garde la haut |Δ|>256. */

        /* === SP catastrophic delta tracer ===
         * Diag v2 2026-05-08 : SP went from 0x9c1e → 0x0001 in one window
         * (lost ~40k stack words). The progressive-leak log above caps at
         * 100 small deltas and misses the single catastrophic event.
         * This block flags any |Δ| > 100 in one instruction — never
         * capped — so the buggy STM/PSHM/POPM/RETE-corrupted-stack /
         * FRAME-with-huge-offset is unambiguously identified the FIRST
         * time it happens. ARs included so we can see if the ST/LD
         * destination resolved to an MMR slot (e.g. *AR=0x18 → MMR_SP).
         *
         * Threshold raised from 100→256 on 2026-05-08 to filter legitimate
         * FRAME #imm8s (signed 8-bit can be ±127). Real catastrophes from
         * dual-op writing to MMR_SP are always thousands of words. */
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
        /* === v2 TRAP-OOR firing point — fixed checkpoint halt ===
         * T1/T2 dropped (v1 v2 redesign: scheduler at 0xfd2a exonerated,
         * SP clobber lives in legit code → no PC whitelist nor SP edge
         * can catch it). Halt at fixed insn checkpoint, dump trail+sp_low
         * for offline analysis of full descent.
         * Checkpoint configurable via CALYPSO_TRAP_CHECKPOINT (default
         * 4200000 = just after the insn=4.09M SP recovery 0x0008→0x2900). */
        {
            static int trap_armed = -1;
            static int tripped = 0;
            static unsigned checkpoint = 0;
            if (trap_armed < 0) {
                const char *e = cdbg_env("TRAP-OOR"); (void)e;
                /* TRAP-OOR RETIRED 2026-05-29 : c'était l'analyse de descente
                 * SP / SP-CATASTROPHE, résolue (0x70c3 self-CALA / DROM-LUT /
                 * stub). Le site 2 faisait s->running=0 au checkpoint 4.2M =
                 * LE bottleneck (CALYPSO_DEBUG=ALL haltait le DSP). Forcé OFF. */
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

        /* === VARIATEUR DE VITESSE osmocon : le break est MAINTENANT en
         * FIN d'itération (après s->pc += consumed + commit delay-slots),
         * cf bloc plus bas. Casser ici (avant l'avance du pc) ré-exécutait
         * l'instruction courante à la ré-entrée → double-pop des RET/RETD/RCD
         * → over-pop SP → garbage DP → self-CALA 0x70c3. Fix 2026-05-30. */

        /* === DUAL-OP-INTERPRET diagnostic ===
         * Compare current decoder's AR field interpretation (3-bit fields)
         * with SPRU172C's dual-operand encoding (2-bit AR fields + offset 2,
         * AR2..AR5 only). If the two disagree on which AR is used and the
         * SP-CATASTROPHE just fired, we have evidence the encoding is
         * wrong. Cap to 100 entries to avoid log explosion. */
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


        /* RPT: after executing an instruction while repeat is active,
         * re-execute the SAME instruction (don't advance PC) until count=0. */
        if (s->rpt_active && !s->idle) {
            /* ═════════════════════════════════════════════════════════════════
             * [2026-08-04] FIX_RPT_COUNT, patte 2/2 — `RPT #k` faisait k
             * repetitions au lieu de k+1.
             *
             * MECANISME. Le handler `RPT` pose `rpt_active=1`, avance le PC et
             * rend la main. Ce bloc, execute juste apres, voyait `rpt_active`
             * deja vrai et decrementait `rpt_count` — donc **l'instruction RPT
             * elle-meme consommait une repetition**. L'instruction repetee ne
             * s'executait plus que k fois.
             *
             * TI SPRU172C : « RPT #k : repeat next instruction k+1 times ».
             *
             * MESURE (sonde READA-ITER, chargeur de table 0xb4c4) :
             *     rpt #0x02 ; reada *AR1+
             *     #1 addr=0x43d5 rpt_count=1   <- devrait etre 2
             *     #2 addr=0x43d6 rpt_count=0
             *     (pas de #3)  -> data[0x43d7] JAMAIS initialise
             * Et la 1ere boucle du meme chargeur, `rpt #0x4d`, ecrit
             * 0x4387..0x43d3 = 77 mots = k, alors que k+1 = 78 irait a 0x43d4.
             * Les deux boucles sont donc courtes d'exactement UNE copie.
             *
             * PORTEE : TOUTES les boucles `RPT` du firmware, y compris le
             * bloc-copie `rpt #0x0b ; mvdd` qui alimente a_cd[3..14] — il ne
             * copiait que 11 des 12 mots de charge utile L2.
             *
             * ⚠️ Ce n'est PAS une erreur dans le calcul de `rpt_count` (les 8
             * poseurs sont corrects) : c'est la boucle qui consomme une fois de
             * trop. On corrige donc ICI, une seule fois, plutot que sur chaque
             * poseur — et ca couvre toutes les formes (RPT #k8, #lk, Smem, RPTZ).
             *
             * Gate `CALYPSO_FIX_RPT_COUNT=0` : restaure le comportement d'avant.
             * ═════════════════════════════════════════════════════════════════ */
            {
                static int frc = -1;
                if (frc < 0) {
                    frc = calypso_gate("CALYPSO_FIX_RPT_COUNT", 1);
                    fprintf(stderr, "[c54x] FIX_RPT_COUNT %s "
                            "(CALYPSO_FIX_RPT_COUNT=%d) — RPT #k fait %s "
                            "(TI SPRU172C)\n", frc ? "ACTIF" : "inactif", frc,
                            frc ? "k+1 repetitions" : "k (comportement d'avant)");
                }
                if (frc && !rpt_was_active) {
                    /* L'instruction qu'on vient d'executer EST le RPT : elle ne
                     * doit pas consommer de repetition. PC deja avance par le
                     * handler, on relance sans decrementer. */
                    s->cycles++;
                    executed++;
                    continue;
                }
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
        s->pc &= 0xFFFF;  /* C54x has 16-bit PC (23-bit with XPC, but wrap at 16-bit) */
        /* consumed == 0 means PC was set by branch */

        /* [2026-07-23] SP-CORRUPT watchpoint : quelle instruction sort SP de la
         * plage pile valide [0x5900,0x5c00] ? (derail 0xa58d SP=0xc905 post-POPD).
         * Logge la 1ere transition dedans->dehors avec pc/op = le coupable. */
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

        /* [2026-07-23] c54x on-chip TIMER0 tick — HORLOGE MANQUANTE (diag horloges +
         * intuition user "tick TINT"). Le go-live arme IMR bit4 (TINT vec20) et ATTEND
         * le timer, mais c etait une facade morte (registres TIM/PRD/TCR OK, mais AUCUN
         * decrement -> jamais de TINT). On tick TIM (avec prescaler TDDR) par instruction ;
         * a l underflow -> reload TIM=PRD + fire TINT vec20/bit4. Gate CALYPSO_DSP_TIMER_OFF
         * (A/B), defaut ON. Le firmware demarre le timer (clear TSS) + configure PRD/TDDR. */
        {
            /* [2026-07-23] TINT0 MASTER CLOCK (modele du gap : le firmware arrete le
             * timer DSP (TCR TSS=1) mais sur HW reel TINT0 = master clock TDMA. On fire
             * TINT0 vec20/bit4 a cadence ~frame (fixe) independamment de TSS. Gate
             * CALYPSO_TINT0_MASTER defaut ON, OFF via CALYPSO_TINT0_MASTER_OFF=1. */
            {
                /* [2026-07-23] fire crude per-2000-insn REMPLACE par sync frame-tick
                 * (dsp_shunt.c:430). Ce bloc desactive (garde pour A/B legacy). */
                /* @BEQUILLE — TINT0_PERINSN  (CALYPSO_TINT0_PERINSN, EXISTS, defaut OFF)
                 *   masque  : l'absence de base de temps DSP. Fire TINT toutes les
                 *             2000 insns, sans aucun rapport avec la cadence TDMA.
                 *   retirer : remplace par le tick TIMER0 fidele juste en dessous.
                 *   ATTENTION : le commentaire "Ce bloc desactive" est FAUX — le code est execute,
                 *               seule l'absence de la variable l'eteint.
                 */
                if (getenv("CALYPSO_TINT0_PERINSN")) {
                    static unsigned _t0c = 0;
                    if (++_t0c >= 2000) { _t0c = 0; c54x_fire_tint(s); }
                }
            }
            static int _tmr = -1;
            if (_tmr < 0) _tmr = getenv("CALYPSO_DSP_TIMER_OFF") ? 0 : 1;
            /* @BEQUILLE — TINT0_MASTER  (CALYPSO_TINT0_MASTER, EXISTS, defaut OFF hors profil
             *              WIRE — calypso.env/wire.env ne le posent que sous CALYPSO_WIRE=1)
             *   masque  : la configuration du TIMER0 par le ROM (TCR/PRD). Le firmware arrete
             *             le timer (TSS=1) dans une init non-tournee ; on force PRD=0xFFFF et
             *             on tick MALGRE TSS, plus un fire TINT0 vec20/bit4 au frame-tick du
             *             shunt (calypso_dsp_shunt.c).
             *   retirer : quand la sequence d'init TIMER0 du ROM s'execute (TCR programme,
             *             TSS=0).
             *   NB      : le 3e site historique est mort — neutralise par (void)_t0i;.
             */
            static int _t0master = -1;
            if (_t0master < 0) _t0master = calypso_gate("CALYPSO_TINT0_MASTER", 0);
            /* [2026-07-23] TIMER0 FIDELE : le firmware arrete le timer (TCR TSS=1) dans
             * l'init op non-tournee. En mode TINT0_MASTER on modelise le ROM ayant
             * configure+demarre le timer : on tick malgre TSS. PRD non configure (0/0xFFFF
             * reset) -> underflow ~65536 insns ~= frame TDMA (13MHz). Fire TINT a
             * l'underflow via c54x_fire_tint(), qui RESPECTE l'IMR (pas de forcing). */
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
                        c54x_fire_tint(s);   /* §5.1 : TINT = bit3/vec19 */
                    } else {
                        s->data[TIM_ADDR]--;
                    }
                } else {
                    s->timer_psc--;
                }
            }
        }

        /* === BRANCH-TRACE (2026-06-24, sonde amont event-starvation) ==========
         * consumed==0 <=> PC pose par une branche/call/ret PRISE (sinon s->pc +=
         * consumed sequentiel, ligne ci-dessus). On logge CHAQUE transfert de
         * controle PRIS dans la fenetre boot/init [insn<6000], filtre du storm
         * PC=0 (exec_pc!=0 && tgt!=0), avec site + opcode de branche + CIBLE +
         * etat des conditions (TC, ACC=0?, A, ARx, IMR, INTM). BUT : voir QUEL
         * branchement conditionnel detourne le firmware de l'install-vecteurs/
         * enable-IRQ vers la garde idle = le mur amont. Discrimine les 2 cas du
         * pari : (a) un BC/BANZ PRIS dont la cible court-circuite la pose IPTR
         * (=jamais atteint) vs (b) le code atteint la pose sans effet (=bug MMR).
         * Cap 700. */
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

        /* Delayed-branch slot countdown.
         * RCD (and later CALLD/RETD/BD/CCD if extended) sets delayed_pc and
         * delay_slots = 2. The two instructions following the RCD execute
         * as normal pipeline slots; once both have completed the branch
         * commits by forcing PC to delayed_pc. */
        /* Delay-slot countdown — compté en MOTS, pas en instructions.
         * BUG FIX 2026-05-31 : l'ancien code décrémentait dès l'itération qui
         * arme delay_slots=2, PUIS d'1 par instruction → une SEULE instruction
         * de delay-slot exécutée. OK pour un slot = 1 insn 2-mots (STM #k), mais
         * pour un slot = DEUX insns 1-mot, la 2e était SAUTÉE. Quand cette 2e
         * insn est un PSHM/PSHD (sites PROM0 0xb53a/0xc9a2/0xcaab = code
         * power-scan que le mobile exécute en cell-search), le push est perdu →
         * over-pop cumulé (~58 mots) → POPM ST0 @0x94f3 ramasse l'orphelin
         * 0x80fd → DP=0x0fd → dispatcher 0x8341 lit la LUT garbage → CALAD
         * 0x70c3 = self-CALA → écrit 0x70c4 (=28868) dans d_fb_det/a_pm →
         * rxlev/TOA poison → NO_CELL_FOUND. cf doc/SP_CATASTROPHE_70c4_SEQUENCE.
         * Le C54x a TOUJOURS 2 mots de delay (SPRU172C) : 1 insn 2-mots OU 2
         * insns 1-mot. On décrémente du nombre de MOTS exécutés (= consumed), et
         * on NE compte PAS l'itération qui arme (ds_before==0 = la branche
         * elle-même ; le delay commence à l'instruction suivante). */
        if (s->delay_slots > 0) {
            if (ds_before == 0) {
                /* itération de la branche différée elle-même : ne rien
                 * décrémenter ; delay_slots (=2) est un compteur de MOTS. */
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

        /* SP-LEDGER : dump périodique pour valider net_words→0 sur run long
         * (métrique de balance push/pop post-yield-fix). ~1 compare/insn. */
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

        /* === VARIATEUR DE VITESSE osmocon (gated, CALYPSO_DSP_YIELD=N) ===
         * Le DSP c54x tourne SYNCHRONE dans tdma_tick sur le thread principal.
         * Tous les N insns on sort de c54x_run → la mainloop pompe l'I/O
         * (osmocon) puis délivre les IT au DSP.
         *   N PETIT = yield fréquent = osmocon rapide / DSP ralenti
         *   N GRAND = yield rare / 0 = OFF (legacy, DSP garde tout le budget)
         * IMPÉRATIF : ne casser qu'à un BOUNDARY PROPRE — ici, après
         * `s->pc += consumed` ET le commit des delay-slots (delay_slots==0).
         * Sinon (a) l'instruction courante est ré-exécutée à la ré-entrée
         * (double-pop) et (b) un IT délivré par la mainloop tomberait au
         * milieu des delay-slots d'un RETD/RCD → retour différé corrompu.
         * Les deux mènent à l'over-pop SP → DP garbage → self-CALA 0x70c3.
         * (Valeur idéale = statique à déterminer ; gardée en env pour l'instant.) */
        {
            static int dsp_yield = -1;
            if (dsp_yield < 0) {
                const char *e = getenv("CALYPSO_DSP_YIELD");
                /* Défaut statique 32768 (2^15) : cadence DSP↔osmocon/IT calée
                 * (valeur trouvée empiriquement, ON par défaut). 0 = OFF legacy
                 * seulement si CALYPSO_DSP_YIELD=0 explicite. */
                dsp_yield = (e && *e) ? atoi(e) : 32768;
                if (dsp_yield < 0) dsp_yield = 0;
                fprintf(stderr, "[c54x] CALYPSO_DSP_YIELD = %d insn/yield %s\n",
                        dsp_yield, dsp_yield ? "(variateur ON)" : "(OFF, legacy)");
            }
            /* Bien implémenté (fix 2026-05-30) : ne yielder qu'à un point
             * INTERRUPTIBLE. Le yield rend la main à la mainloop qui délivre
             * l'IT (INT3) au DSP ; sur vrai C54x une IT n'est prise qu'à INTM=0
             * (hors section critique). Couper sur un simple compteur d'insns
             * tombait en pleine séquence de dispatch (INTM=1, DP hérité avant
             * LDP) → l'IT au resume corrompait DP/ST0 → CALAD vers la LUT
             * (wedge 0x9207) ou self-CALA. On exige donc :
             *   - executed >= dsp_yield ET INTM=0 (point sûr), OU
             *   - executed >= 4×dsp_yield (cap dur : évite la famine mainloop
             *     si le firmware reste en INTM=1 anormalement longtemps).
             * delay_slots==0 garde inchangée (jamais mid-branche-différée). */
            /* 4 gardes = les 4 états non-interruptibles du C54x :
             *   delay_slots==0  : pas mid-branche-différée (RETD/RCD/CALLD/BD)
             *   !rpt_active     : pas mid-RPT (single-repeat = NON interruptible
             *                     sur HW jusqu'à RC épuisé ; RPTB l'est, lui)
             *   INTM==0         : interruptible (hors section critique/dispatch)
             *   (+ break après commit pc/delay = pas mid-instruction)
             * Cap dur 4× : si INTM reste 1 anormalement, force le yield pour
             * éviter la famine mainloop (cas "illégal" tracé ci-dessous). */
            if (dsp_yield > 0 && s->delay_slots == 0 && !s->rpt_active &&
                ((executed >= (unsigned)dsp_yield && !(s->st1 & ST1_INTM)) ||
                 executed >= (unsigned)dsp_yield * 4u)) {
                /* Preuve du gate : log les premiers breaks + tout break "cap-forcé"
                 * (INTM=1 = illégal toléré). Si on ne voit JAMAIS de cap-forcé sur
                 * N runs ET 0x9207 disparaît → le gate est prouvé, pas juste constaté. */
                if (calypso_debug_enabled("YIELD-BREAK")) {
                    static unsigned yb = 0;
                    int forced = (s->st1 & ST1_INTM) ? 1 : 0;
                    if (yb < 40 || forced)
                        fprintf(stderr, "[c54x] YIELD-BREAK #%u INTM=%d delay=%d rpt=%d pc=0x%04x exec=%u %s\n",
                                yb, forced, s->delay_slots, s->rpt_active, s->pc, executed,
                                forced ? "*** CAP-FORCED (INTM=1 illegal) ***" : "(safe)");
                    yb++;
                }
                break;   /* boundary propre + interruptible → mainloop sert I/O + IT */
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
 * [c54x-earlyboot] — rapatrie de calypso_dsp_shunt.c le 2026-09-03.
 *
 * Ce code n'avait RIEN de shunt : il ne vit que sous CALYPSO_DSP_RUN_C54X et ne
 * force aucune valeur de mailbox, seulement le QUAND du boot. Il vivait dans le
 * shunt par accident d'historique (le shunt tenait le handle du c54x).
 *
 * FIX race d'ordre golive (2026-07-20). L'ARM poste sa commande bootloader
 * (data[0x0fff] = cmd 2/4, data[0x0ffe] = entry) des fn=0 / +0,073 s. Si le DSP
 * ne boote qu'ensuite, son init-IDLE en 0xb419 (`ST #1, *0xfff`) ECRASE ce
 * 0x0002 -> spin eternel en 0xb41c. En bootant ici, a machine-init et donc AVANT
 * que le vCPU ARM tourne, le DSP pose son IDLE et se parke en 0xb41c AVANT
 * l'ecriture ARM : 0xb419 ne re-tourne plus (le PC persiste entre les reveils),
 * la commande survit, et le premier reveil la consomme -> go-live natif.
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
    c54x_run(s, 2000);   /* reset(0xff80) -> 0xb419 (pose IDLE) -> park 0xb41c */
    if (s->pc >= 0xb41c && s->pc <= 0xb428) {
        g_c54x_early_booted = true;   /* gate le re-reset cote calypso_trx.c */
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

/* Mission courante du DSP, lue dans l'API RAM. Rapatrie du shunt le 2026-09-03 :
 * l'accesseur y renvoyait d'abord un latch alimente par le shunt, avec un
 * fallback natif sur ces deux cellules. Le latch parti, seul le fallback reste —
 * et c'est de la simple lecture d'API RAM, rien de shunt.
 *   d_task_md page 0 = data[0x0804], page 1 = data[0x0818]. */
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
    s->a = 0; s->b = 0;   /* mode c54x = reset datasheet propre : A=B=0
                           * (le snapshot a BL=0x60, appliqué seulement en mode bin) */
    /* ── REVIEW registres : hardcode C ↔ calypso_dsp.Registers.bin (2026-05-31) ──
     * Le hardcode ci-dessous = mode "c54x" = RESET DATASHEET PROPRE (champs
     * critiques alignés au snapshot, champs bénins/garbage à 0). Les 3 modes
     * sont ainsi orthogonaux (sélecteur plus bas) :
     *   c54x   = ce hardcode propre, indépendant du fichier
     *   bin    = override depuis .Registers.bin VERBATIM (snapshot exact, défaut)
     *   hybrid = bin pour l'opérationnel + champs critiques forcés propres
     *            (IFR=0, AR0=0xFF75, BRC/RSA/REA=0) → ≈ c54x sur ces champs
     *
     * Le .bin est un SNAPSHOT mi-exécution (post-handshake bootloader). Review
     * champ par champ (verdict = pertinence de la valeur AU RESET) :
     *
     *   MMR        valeur(bin=hardcode)  classe        commentaire
     *   IMR  0x00  0x52FD                CRITIQUE-OK   masque IRQ, identique 3 dumps
     *   IFR  0x01  0x0008                BÉNIN         bit3 INT3 pending ; INTM=1 masque
     *                                                  → hybrid le met à 0 (datasheet pur)
     *   ST0  0x06  0x181F                CRITIQUE-OK   DP=0x1F
     *   ST1  0x07  0x2900                CRITIQUE-OK   INTM/SXM/XF
     *   A    08-0a  0x000000             OK            accumulateur A = 0
     *   B    0b-0d  0x000060             BÉNIN         BL=0x60 (rechargé avant usage)
     *   T    0x0E  0x0000                OK
     *   TRN  0x0F  0xFF75                BÉNIN         Viterbi, neutre au reset
     *   AR0  0x10  0x5AAD                BÉNIN         rechargé (LD @0x7120) ; hybrid=0xFF75
     *   AR1-5 11-15 invariants           CRITIQUE-OK   identiques 3 dumps (API_RAM, etc.)
     *   AR6  0x16  0xBAE6                BÉNIN         rechargé avant usage
     *   AR7  0x17  0x1E44                BÉNIN         idem
     *   SP   0x18  0x1100                CRITIQUE-OK   pile post-handshake
     *   BK   0x19  0xFFF6                CRITIQUE-OK   circular buffer
     *   BRC  0x1A  0x8FD7                GARBAGE       reste RPTB mi-vol ; neutre
     *   RSA  0x1B  0xD9EC                GARBAGE       (rptb_active=false au reset)
     *   REA  0x1C  0xBBEF                GARBAGE       → hybrid les met à 0
     *   PMST 0x1D  0xFFA8                CRITIQUE-OK   IPTR=0x1FF, MP_MC, OVLY, DROM
     *   XPC  0x1E  0x0000                — jamais overridé (page runtime, fetch vec)
     *
     * CONCLUSION review : les champs CRITIQUE-OK (SP/ST0/ST1/PMST/IMR/AR1-5/BK)
     * sont identiques bin↔hardcode et pilotent le reset. Les divergences (IFR,
     * AR0/6/7, TRN, B, BRC/RSA/REA) sont toutes BÉNIGNES ou GARBAGE et neutres
     * au reset (IT masquée par INTM=1 ; AR rechargés ; rptb_active=false). Donc
     * aucune n'explique le stuck FB (boucle BITF @0xf2cd sur data[0x585f]).
     */
    /* AR registers aligned with silicon spec (doc/datasheets/README.md §3,
     * 2026-05-25). Cross-checked 3 ROM dumps (3311/3416/3606) + local osmocom :
     *   AR1=0x005F, AR2=0x0813, AR3=0x0014, AR4=0x0003, AR5=0x0014  (invariant)
     *   AR0=0xFF75, BK=0xFFF6  (local osmocom dump values)
     *   AR6, AR7 : non documenté invariant, garde 0
     *
     * Précédent : memset 0 = init shortcut, même problème que SP/IMR.
     * Symptôme : STL A,*AR2 à PC=0x9ac0 avec AR2=0 écrivait à mem[0x00]=IMR
     * → IMR cleared → toutes IRQ FRAME/BRINT0 masquées → DSP bloqué en df9x. */
    /* AR registers — mode c54x = reset datasheet propre.
     * AR1-5 = invariants cross-dump (3311/3416/3606) = identiques au snapshot,
     * gardés ici car CRITIQUES. AR0/AR6/AR7 = 0 (neutres : le firmware les
     * recharge avant usage, ex. LD @0x7120). Le snapshot a AR0=0x5aad,
     * AR6=0xbae6, AR7=0x1e44 → appliqués seulement en mode bin. */
    memset(s->ar, 0, sizeof(s->ar));
    s->ar[0] = 0x5AAD;  /* FIX 2026-05-31 : AR0 = valeur silicium (snapshot bin).
                         * PROUVÉ read-before-write à insn=1 (PC=0xb410 ORM
                         * data[*AR0]) via sonde AR-FIRSTUSE → AR0 reset est
                         * load-bearing, l'ancien 0xFF75 (dump local, "neutre")
                         * faisait diverger c54x vs bin dès la 1ʳᵉ instruction du
                         * boot. Aligné sur le silicium → convergence des modes. */
    s->ar[1] = 0x005F;
    s->ar[2] = 0x0813;  /* API_RAM-related — clobber IMR si =0 (cf 2026-05-25) */
    s->ar[3] = 0x0014;
    s->ar[4] = 0x0003;
    s->ar[5] = 0x0014;
    s->t = 0; s->trn = 0;   /* TRN=0 (snapshot 0xff75, neutre au reset) */
    s->sp = 0x1100; s->bk = 0xFFF6;  /* SP+BK init aligned with silicon (2026-05-25).
                                 * 3 ROM dumps (3311/3416/3606) + local : SP=0x1100
                                 * post-bootloader-handshake. Let firmware repoint
                                 * to its own stack (0x5AC8 historically observed)
                                 * via init sequence, comme sur silicon réel.
                                 * Précédent : SP=0x5AC8 = shortcut anticipant
                                 * la re-init firmware. Suspect d'être la racine
                                 * du clobber AR5↔SP overlap à mem[0x3fbe].
                                 * Voir doc/datasheets/README.md §3-4. */
    /* BRC/RSA/REA = 0 (reset datasheet propre). Le snapshot capture des restes
     * de RPTB mi-vol (BRC=0x8fd7 RSA=0xd9ec REA=0xbbef) = GARBAGE sans sens au
     * reset ; appliqués seulement en mode bin. rptb_active=false (posé plus bas)
     * → ces registres ne sont consultés qu'après qu'un RPTB les recharge. */
    s->brc = 0; s->rsa = 0; s->rea = 0;
    /* MMR reset values aligned with Calypso silicon (3 FreeCalypso ROM dumps + local).
     * Empirically validated 2026-04-28. See doc/datasheets/README.md §3.
     * Previous QEMU values (st0=0, st1=ST1_INTM, pmst=0xFFE0) were partial. */
    s->st0  = 0x181F;                              /* DP=0x01F per silicon */
    s->st1  = ST1_INTM | ST1_SXM | ST1_XF;         /* 0x2900: INTM=1, SXM=1, XF=1 */
    s->pmst = 0xFFA8;                              /* IPTR=0x1FF, MP_MC=1, OVLY=1, DROM=1 */
    s->imr = 0x52FD;                               /* IMR aligned avec local osmocom dump
                                                    * (doc/datasheets/README.md §3, post-
                                                    * bootloader-handshake). 0 était un autre
                                                    * shortcut comme SP. IRQ #2..#10 vus
                                                    * INTM=1 IMR=0x0000 IFR=0x28 → IRQs
                                                    * masquées toutes → handlers jamais run
                                                    * → flags dispatcher pas écrits → DSP
                                                    * boucle indéfiniment en df9x (= bloqueur
                                                    * #2 chain FBSB). Fix 2026-05-25. */
    s->ifr = 0;        /* IFR=0 (reset datasheet propre). Le snapshot a 0x0008
                        * (bit3 INT3 pending) ; appliqué seulement en mode bin.
                        * Neutre de toute façon : INTM=1 (ST1=0x2900) masque l'IT. */
    s->xpc = 0;
    /* ===================== Sélecteur d'état reset registres =====================
     * Trois modes, choisis par env CALYPSO_DSP_REG_MODE :
     *   "c54x"   → hardcode C ci-dessus UNIQUEMENT (le .bin chargé est ignoré).
     *   "bin"    → snapshot calypso_dsp.Registers.bin override TOUT (verbatim).
     *   "hybrid" → snapshot bin POUR les registres opérationnels validés, MAIS
     *              garde le hardcode pour les champs où le .bin est jugé faux
     *              par l'audit anti-drift (cf table plus haut) :
     *                IFR  : bin=0x0008 (IRQ pending résiduel) → hardcode 0
     *                AR0  : bin=0x5aad (non validé)           → hardcode 0xFF75
     *                BRC/RSA/REA : bin=garbage RPTB mi-vol     → hardcode 0
     * Défaut : "bin" si un .bin est chargé (continuité avec le comportement
     * câblé par run.sh), sinon forcément le hardcode (rien à overrider).
     * Tous lus une fois (reset appelé 2× : boot + DSP_DL_STATUS_READY). */
    {
        static int reg_mode = -1;  /* 0=c54x 1=bin 2=hybrid */
        if (reg_mode < 0) {
            const char *e = getenv("CALYPSO_DSP_REG_MODE");
            if      (e && !strcasecmp(e, "c54x"))   reg_mode = 0;
            else if (e && !strcasecmp(e, "hybrid")) reg_mode = 2;
            else                                    reg_mode = 1; /* "bin"/défaut */
            C54_LOG("reset: CALYPSO_DSP_REG_MODE=%s → mode=%s",
                    e ? e : "(unset)",
                    reg_mode == 0 ? "c54x(hardcode)" :
                    reg_mode == 2 ? "hybrid" : "bin");
        }
        if (s->reg_init_valid && reg_mode != 0) {
            const uint16_t *r = s->reg_init;
            /* Registres opérationnels — communs bin + hybrid */
            s->imr  = r[0x00];
            s->st0  = r[0x06];
            s->st1  = r[0x07];
            s->a    = ((int64_t)(r[0x0a] & 0xFF) << 32) |
                      ((uint32_t)r[0x09] << 16) | r[0x08];
            s->b    = ((int64_t)(r[0x0d] & 0xFF) << 32) |
                      ((uint32_t)r[0x0c] << 16) | r[0x0b];
            s->t    = r[0x0e];
            s->trn  = r[0x0f];
            for (int i = 1; i < 8; i++)   /* AR1..AR7 ; AR0 traité plus bas */
                s->ar[i] = r[0x10 + i];
            s->sp   = r[0x18];
            s->bk   = r[0x19];
            s->pmst = r[0x1d];
            if (reg_mode == 1) {
                /* BIN PUR (2026-06-25) : .bin VERBATIM, AUCUN hardcode forcé.
                 * Avant : IFR/BRC/RSA/REA forcés 0 (anti-drift) = hardcodes
                 * résiduels qui jetaient l'état silicium réel (IFR=0x0008 INT3
                 * pending notamment). Le snapshot EST l'état silicium → on le
                 * respecte intégralement. rptb_active=false (posé plus bas) =>
                 * BRC/RSA/REA non consultés tant qu'un RPTB ne les recharge pas. */
                s->ifr   = r[0x01];
                s->ar[0] = r[0x10];
                s->brc   = r[0x1a];
                s->rsa   = r[0x1b];
                s->rea   = r[0x1c];
            } else { /* reg_mode == 2 : hybrid → registres opérationnels du bin,
                      * MAIS champs critiques forcés aux valeurs datasheet pures
                      * (cf audit anti-drift) pour un reset propre. */
                s->ifr   = 0x0000;   /* pas d'IRQ pending au reset */
                s->ar[0] = r[0x10];  /* FIX 2026-05-31 : AR0 = snapshot silicium
                                      * (0x5aad), pas le hardcode 0xFF75 : prouvé
                                      * read-before-write insn=1 → load-bearing. */
                s->brc   = 0x0000;
                s->rsa   = 0x0000;
                s->rea   = 0x0000;
            }
            /* XPC jamais overridé (registre de page runtime ; le fetch vecteur
             * reset à IPTR*0x80 doit venir de la page 0 → XPC=0 conservé). */
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

    /* 2026-05-28 v3 : runtime conditional override (option A).
     *
     * v1 (static override prog[0xFF80] = B 0x7120) intercepted both the
     * silicon reset AND every sequential firmware walk into address 0xff80.
     * The firmware ROM contains a normal subroutine that includes a RET at
     * 0xff79 popping 0xff7a + POPM/STM sequence walking through 0xff80
     * during routine epilogues. The static override turned each such walk
     * into a soft-reset → SP=0x5AC8 → state derailed → DSP stuck in
     * boot-reset cycle, FB never stabilising past the first 4s.
     *
     * v3 fix : leave prog[0xFF80..0xFF83] as legitimate PROM1 mirror
     * (= 0x56d0, 0x9631, 0xf820, 0xff89 from the dump). Apply the boot-init
     * redirect at runtime ONLY when SP still holds the silicon-reset value
     * 0x1100 (i.e., the very first reach of 0xff80 after silicon-reset,
     * before any STM #imm,SP has run). See c54x_run main loop for the
     * runtime check. */

    /* Boot ROM stubs at 0x0000-0x007F.
     * Discriminant test 2026-04-26 confirmed FRET stub did NOT block the
     * firmware path to 0x0810 (reverting to NOPs gave identical PC HIST
     * + same IMR change=0). FRET stub kept: prevents stack runaway when
     * CALAA targets the stub area, with no downside.
     *
     * Fallback per slot (2026-05-29 v3 — RET@0x0000 équilibre le near-CALA) :
     *   - 0x0000/0x0001: RET (0xFC00) — pop ret_pc, retour ÉQUILIBRÉ.
     *   - rest (0x02..0x7F): FRET (0xF4E4) — retour-from-far (far-call→stub).
     *
     * Pourquoi RET@0x0000 (et pas IDLE/LDMM) : le firmware fait des near-CALA
     * `CALA → 0x0000` avec A=0 (chemin handler-nul/défaut ; ex. LDU@0xfa7e lit
     * un ptr de table = 0 → CALA A=0). Un near-CALA push 1 mot (ret_pc) ; RET
     * pop 1 mot → ÉQUILIBRÉ → retour propre au caller → le boot continue.
     *   - IDLE (v2) ne retournait pas → halt+slide 0x0000→0x0002 → FRET-loop
     *     → fuite SP (0x1106→0x4d75).
     *   - LDMM SP,B+RET (old/good 2026-05-28) retournait MAIS posait SP=B :
     *     si B=0 → SP=0 → pop garbage ("wake-on-IRQ" loop de v1).
     * Le rest reste FRET : les far-calls (FCALA, push 2) qui tombent dans la
     * zone sont équilibrés par FRET (pop 2). Le firmware idle à son vrai
     * point (IDLE de la table TDMA slots, cf doc/DSP_ROM_MAP.md). */
    for (int i = 0; i < 0x80; i++)
        s->prog[i] = 0xF4E4;  /* FRET — retour-from-far (far-call-into-stub) */
    s->prog[0x0000] = 0xFC00;  /* RET — pop ret_pc, équilibre le near-CALA→0 */
    s->prog[0x0001] = 0xFC00;  /* RET — idem */

    /* Reset vector: IPTR * 0x80 */
    uint16_t iptr = (s->pmst >> PMST_IPTR_SHIFT) & 0x1FF;
    s->pc = iptr * 0x80;  /* 0xFF80 for default PMST */

    C54_LOG("Reset: PC=0x%04x PMST=0x%04x SP=0x%04x prog[PC]=0x%04x",
            s->pc, s->pmst, s->sp, s->prog[s->pc]);

    /* Build identity dump (2026-05-25) — permet attribution causale dans
     * les rapports/bundles. Cf review Claude web : "Le rapport ne peut pas
     * s'attribuer à un état de code". On dump ici les valeurs reset
     * silicon-aligned utilisées par CE binaire — si elles changent, le
     * comportement firmware change. Lecture de qemu.log = identité du build. */
    C54_LOG("BUILD-IDENT silicon-reset: SP=0x%04x BK=0x%04x IMR=0x%04x "
            "ST0=0x%04x ST1=0x%04x PMST=0x%04x",
            s->sp, s->bk, s->imr, s->st0, s->st1, s->pmst);
    C54_LOG("BUILD-IDENT silicon-AR: AR0=0x%04x AR1=0x%04x AR2=0x%04x AR3=0x%04x "
            "AR4=0x%04x AR5=0x%04x AR6=0x%04x AR7=0x%04x",
            s->ar[0], s->ar[1], s->ar[2], s->ar[3],
            s->ar[4], s->ar[5], s->ar[6], s->ar[7]);
    /* Decoder fix flags : si ces fixes sont retirés du source, ce log
     * n'apparaîtra plus ou aura un format différent — preuve immédiate
     * de quel binaire produit le run. */
    C54_LOG("BUILD-IDENT decoder-fixes: F1xx-FIRS-catch=REMOVED "
            "L3609-src-dst=FIXED F-AUDIT-v5=max-min-cmpl-rnd-roltc-fixed "
            "F2xx-ALU-block=ADDED-2026-05-25-night "
            "F3xx-INTR-mis-REMOVED-ADD-SUB-LD-ADDED "
            "PROBE-HIGHVEC-REGIME=2026-06-23 "
            "SQURA-0x38-FIX=2026-06-23 "
            "2026-05-25");

}

int g_c54x_int3_src = 0;   /* 1=trx 2=bsp 3=shunt — diag source INT3 (RO) */


void c54x_interrupt_ex(C54xState *s, int vec, int imr_bit)
{
    if (vec < 0 || vec >= 32) return;
    if (imr_bit < 0 || imr_bit >= 16) return;
    /* [2026-09-03] REMAP VEC28 SUPPRIME. Ce bloc remappait a l'execution
     * vec19/bit3 -> vec28/bit12 sous les gates CALYPSO_DSP_FRAME_VEC28 /
     * CALYPSO_FRAME_IT_NATIVE (defaut OFF tous les deux), et allait jusqu'a
     * FORCER la vectorisation (frame_force) quand une tache GSM etait postee.
     * Son annotation @BEQUILLE demandait exactement ce qui est fait maintenant :
     * « retirer quand la ligne frame est cablee sur le bon vecteur a la source ».
     * calypso_trx.c emet desormais C54X_IT_TPU_FRAME_VEC/BIT (28/12) directement,
     * donc plus rien a remapper — et plus de force : la fenetre INTM du firmware
     * suffit, l'IMR du ROM (0x52ed) arme le bit 12 elle-meme. */
    s->ifr |= (1 << imr_bit);
    if (imr_bit == 12 && frame_it_level_on()) g_frame_it_level = true;  /* arme le LEVEL hold frame */

    /* SONDE FRAME-IT-RATE (2026-06-24 diag sur-delivrance) : chaque dispatch de
     * l'IT trame avec le delta insn depuis le precedent. delta ~130 =
     * sur-delivrance (BSP per-rafale) qui noie le DSP ; ~256000 = per trame
     * (correct). Cap 80.
     * [2026-09-03] Recablee sur vec 28 : la sonde disait « vec 19 = FRAME », ce
     * qui etait faux (19 = TINT). Sur vec 19 elle ne mesurait plus rien depuis
     * que l'IT trame est emise sur 28/12. */
    if (vec == C54X_IT_TPU_FRAME_VEC) {
        static uint64_t last_i3 = 0;
        static unsigned i3n = 0;
        bool post_fb = s->insn_count > 160000u;   /* ~fn 1206 : ordre FB livre */
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

    /* EXPÉRIENCE WIRE585F RETIRÉE 2026-05-30 : forcer data[0x585f] bit7 au frame-IRQ
     * a prouvé (mem=0x0180 TC=1) que le mécanisme est sain MAIS que 0x585f n'est
     * qu'UNE porte d'une chaîne (→ pose 0x3fd3 puis reboucle) — pas le verrou.
     * Whack-a-mole démontré, pas supposé. cf [[feedback_debug_gate_heisenbug]]. */

    bool unmasked = (s->imr & (1 << imr_bit)) != 0;
    /* @BEQUILLE — FIX_BRINT0_UNMASK  (CALYPSO_FIXES=FIX_BRINT0_UNMASK, defaut OFF)
     *   masque  : l absence d armement natif de l IMR bit 5 (BRINT0 / vec 21).
     *   retirer : des que la vraie branche d armement est implementee, OU
     *             immediatement si le test montre que la racine est en amont
     *             (vecteur 21 installe a zero).
     *   ⚠️ DIAGNOSTIC, a retirer, JAMAIS a confirmer. Ne compte pas comme un correctif.
     * Repond a UNE question : BRINT0 est-elle le DERNIER verrou ou seulement le
     * PROCHAIN ? Si le demasquage artificiel fait entrer le DSP dans le demod
     * (CALYPSO_WATCH_9F00_RD passe de 0 a non-nul), c est le dernier ; sinon la
     * racine est en amont — candidat : le vecteur 21 installe a zero. */
    if (imr_bit == 5 && !unmasked && calypso_fix_enabled("FIX_BRINT0_UNMASK")) {
        static unsigned _bu = 0;
        if (_bu++ < 5)
            fprintf(stderr, "[c54x] FIX_BRINT0_UNMASK : bit 5 demasque ARTIFICIELLEMENT "
                    "(IMR=0x%04x, IFR=0x%04x, PC=0x%04x) — diagnostic, pas un correctif\n",
                    s->imr, s->ifr, s->pc);
        unmasked = true;
    }

    /* [2026-07-23] SYNC-DISPATCH-PROBE (unconditional, capped) : c54x_interrupt_ex
     * fait un dispatch SYNCHRONE ici (au moment de la levee) si INTM=0 -- sans
     * log dedie contrairement a c54x_irq_level_check's "IRQ-LEVEL take". On veut
     * savoir si BRINT0 (vec21) est en fait servi PAR CE CHEMIN, silencieusement,
     * a chaque levee -- ce qui expliquerait "pend jamais vu par le poller" sans
     * bug : les deux mecanismes ne se chevauchent simplement jamais dans le temps
     * observe par LEVELCHK-EMPIRICAL. */
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
            data_write(s, s->sp, s->xpc);          /* save XPC inconditionnel */
            g_sp_ledger.irq_words_pushed++;
            g_sp_ledger.irq_entries++;
            g_sp_ledger.net_words += 2;  /* PC+XPC poussés ici (hors ring exec) */
            s->st1 |= ST1_INTM;
            /* DISP-ENTRY : capture contexte préempté (DP foreground inchangé) */
            g_last_intr_insn = s->insn_count; g_last_intr_vec = vec;
            g_last_intr_fg_pc = (uint16_t)(s->pc + 1); g_last_intr_fg_dp = dp(s);
            s->xpc = 0;                            /* fetch vecteur sur page 0 */
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
         * Garde delay_slots==0 (fix 2026-05-30) : faithful C54x — une IT
         * n'est PAS reconnue entre une branche différée (RETD/RCD/CALLD/BD)
         * et ses 2 delay-slots. Vectoriser mid-delay laisserait delay_slots
         * armé → au retour (RETE) le commit delayed_pc se ferait dans le
         * mauvais contexte → over-pop SP → DP garbage → self-CALA 0x70c3.
         * IFR reste set (non clearé) → l'IT est servie au prochain appel,
         * delay_slots étant retombé à 0 (max ~2 insns plus tard). */
        /* FRAME-IT PRIO (gated CALYPSO_FRAME_IT_PRIO) : si la frame-IT (bit12/vec28)
         * est latchee ET demasquee mais qu on s apprete a servir une IT de priorite
         * plus basse (ex BRINT0 vec21, sur-livree par le BSP a chaque burst -> noie
         * la frame), servir la FRAME d abord sur cette fenetre INTM=0. Le bit de l IT
         * demandee reste pendant (non cleare, imr_bit ecrase) -> servie au prochain
         * edge. Draine la frame-IT affamee (5908x pendante, 1x prise) -> vec28 ->
         * scheduler 0x7234 -> dispatcher -> kernel FB. */
        if (vec != 28 && frame_it_prio_on() &&
            (s->ifr & (1u << 12)) && (s->imr & (1u << 12))) {
            static unsigned _fp = 0;
            if (_fp++ < 30)
                fprintf(stderr, "[c54x] FRAME-IT-PRIO override vec=%d->28 (frame latchee) "
                        "IFR=0x%04x IMR=0x%04x PC=0x%04x insn=%u\n",
                        vec, s->ifr, s->imr, s->pc, s->insn_count);
            vec = 28; imr_bit = 12;
            g_frame_it_level = false;   /* FIX livelock : relache le LEVEL hold (sinon bit12 re-asserte -> vec28 sur-fire 7x/trame -> 139k INTM-TRANS) */
        }
        c54x_ifr_clear(s, (uint16_t)(1 << imr_bit), "vector-ex");
        s->sp--;
        data_write(s, s->sp, (uint16_t)s->pc);
        g_sp_ledger.irq_words_pushed++;
        s->sp--;
        data_write(s, s->sp, s->xpc);              /* save XPC inconditionnel */
        g_sp_ledger.irq_words_pushed++;
        g_sp_ledger.irq_entries++;
        g_sp_ledger.net_words += 2;  /* PC+XPC poussés ici (hors ring exec) */
        s->st1 |= ST1_INTM;
        /* DISP-ENTRY : capture contexte préempté (DP foreground inchangé) */
        g_last_intr_insn = s->insn_count; g_last_intr_vec = vec;
        g_last_intr_fg_pc = (uint16_t)s->pc; g_last_intr_fg_dp = dp(s);
        s->xpc = 0;                                /* fetch vecteur sur page 0 */
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
        /* FRAME-IT-CYCLE-TRACE : hook cycle start sur l'IT trame (vec 28) */
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

/* [2026-08-22] déclarations au scope FICHIER (évite -Wnested-externs). */
extern void calypso_twl3025_apply_phase(int16_t *iq_samples, int n_samples,
                                        uint32_t fn, uint8_t tn);
extern uint32_t calypso_trx_get_fn(void);
/* c54x_task_md() : declare dans calypso_c54x.h, defini plus haut dans ce fichier. */

void c54x_bsp_load(C54xState *s, const uint16_t *samples, int n)
{
    if (n > 2048) n = 2048;

    /* ─────────────────────────────────────────────────────────────────────
     * [2026-08-04] FEED-FP, patte 2/2 — SORTIE. Voir la patte 1/2 dans
     * calypso_bsp.c (bsp_trxd_readable). Meme gate CALYPSO_BSP_FINGERPRINT,
     * meme hash FNV-1a, meme plafond. Ici on empreinte ce qui part vers le RIF.
     *
     * LECTURE : `identiques` proche de `#total` = burst FIGE. Si la patte IN
     * varie et que celle-ci ne varie pas, le gel est entre les deux. Si les
     * DEUX sont figees, remonter en amont de l'UDP (calypso-ipc-device).
     *
     * ⚠️ Cette sonde compte les repetitions CONSECUTIVES, pas les distinctes :
     * une alternance A,B,A,B donnerait identiques=0 tout en etant pathologique.
     * On l'accepte parce que le symptome observe est une CONSTANTE, mais ne pas
     * en tirer de conclusion au-dela de ce cas. */
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

    /* [2026-08-22] NORMALISATION DU RATE — UN SEUL point de décimation. Les feeds
     * natifs arrivent à des rates INCOHERENTS : g_shunt.last_iq (route shunt, = le
     * MEME feed que shunt_legit) est @4SPS BRUT (1083333 Hz = 4×270833, non décimé,
     * cf. osmo-trx rx-sps=4) ; le chemin BSP deliver_buffered décime déjà ÷4 @1SPS.
     * Selon le chemin actif, c54x_bsp_load recevait donc du 4SPS OU du 1SPS -> le
     * corrélateur + apply_phase (qui veulent 1SPS, FCCH=+pi/2/samp) manglaient les
     * bursts 4SPS -> dphi incohérent (tonalité repliée à DC). On décime ÷DECIM ICI,
     * mais UNIQUEMENT si l'entrée est sur-échantillonnée (n indique 4SPS : 148 sym
     * ×4×2 = 1184 mots, vs 296 @1SPS) -> jamais de double-décim, sortie TOUJOURS
     * 1SPS, tous les chemins uniformes (comme shunt_legit).
     * DECIM = CALYPSO_BSP_IQ_DECIM (défaut 4). */
    {
        static int decim = -1;
        if (decim < 0) {
            const char *d = getenv("CALYPSO_BSP_IQ_DECIM");
            decim = (d && *d) ? atoi(d) : 4;
            if (decim < 1) decim = 1;
        }
        if (decim > 1 && n > 2 * 296) {          /* 4SPS détecté (n ~1184) */
            int oc = 0;
            for (int k = 0; (k * decim) * 2 + 1 < n; k++) {
                s->bsp_buf[2 * oc]     = s->bsp_buf[2 * (k * decim)];
                s->bsp_buf[2 * oc + 1] = s->bsp_buf[2 * (k * decim) + 1];
                oc++;
            }
            n = oc * 2;                          /* n devient le compte 1SPS */
            s->bsp_len = n;
        }
    }

    /* [2026-08-22] AFC AU POINT DE CONVERGENCE. La rotation VCXO (apply_phase)
     * n'etait appliquee que dans calypso_bsp_deliver_buffered (calypso_bsp.c:1888),
     * qui N'EST PAS le chemin natif vivant (mesure : b-bsp-load-ok=0 ; le feed passe
     * ailleurs). Le DSP natif recevait donc des samples NON tournes -> boucle AFC
     * OUVERTE -> le firmware integre l'erreur sans effet sur les samples -> DAC
     * runaway (-700 -> 4095, +34 kHz) -> rotation FCCH hors capture DSP (+-20 kHz)
     * -> TOA delirant. Ici TOUS les feeds convergent (c54x_bsp_load -> RIF), donc on
     * applique la rotation UNE fois, sur bsp_buf (mutable), et on feede bsp_buf a la
     * RIF. Doc CHAINE_RF_MATERIELLE.md:106 : "apply_phase juste avant c54x_bsp_load".
     * tn=0 (FB/SB = TS0 ; un offset de phase constant ne change pas la FREQUENCE
     * corrigee). Inerte si CALYPSO_TWL3025_AFC=0 (apply_phase retourne tot). */
    calypso_twl3025_apply_phase((int16_t *)s->bsp_buf, n / 2,
                                calypso_trx_get_fn(), 0);

    /* [2026-08-03] Le meme burst alimente la FIFO de reception du RIF, qui est
     * la voie par laquelle le firmware DSP le lit reellement (PORTR DRR apres
     * avoir vu SPCR). bsp_buf reste en place pour les sondes et pour l'ancien
     * chemin PORTR PA=0xF430 (CALYPSO_FIX_PORTR). On feede bsp_buf (= samples
     * tournes par l'AFC ci-dessus), pas `samples` (const, non tourne). */
    /* [2026-08-22] GATING FN — ne pousser au RIF/corrélateur QUE les bursts de
     * trame FCCH pendant l'acquisition FB. RACINE (workflow): ce feed n'avait AUCUN
     * gate FN (DIRECT_FEED bypasse le match), donc le buffer corrélateur 0x0cce
     * recevait TOUS les bursts (surtout non-FCCH), mélangés/écrasés -> magnitude
     * VARIABLE (~100..28000, vs FCCH constant) -> surface de corrélation plate ->
     * argmax = bord de fenêtre r39 -> TOA=39 (jamais 23). En ne feedant que les
     * trames FCCH {1,11,21,31,41} (offset ému, cf. calypso_dsp_shunt.c:3710), le
     * corrélateur ne voit que la vraie tonalité. Gate CALYPSO_RIF_FCCH_ONLY (défaut
     * OFF, réversible). On ne gate QU'en mode FB (d_task_md 5/8) pour ne pas priver
     * les autres tâches (SB/NB) de leur burst. */
    {
        static int rif_fcch = -1;
        if (rif_fcch < 0) rif_fcch = calypso_gate("CALYPSO_RIF_FCCH_ONLY", 0);
        int _push = 1;
        if (rif_fcch) {
            /* On filtre en FB (task_md 5/8) ET en IDLE (0) : le stage RIF ne garde
             * que le DERNIER burst poussé ; si on laisse passer les bursts non-FCCH
             * de l'idle (87% du temps), le DMA FB draine un burst pollué. On ne
             * filtre PAS en SB (6) / NB : ces tâches ont besoin de LEUR trame. */
            uint16_t _md = c54x_task_md(s);   /* FB=5, TCH_FB=8, SB=6 */
            if (_md == 5 || _md == 8 || _md == 0) {
                int _p = (int)(calypso_trx_get_fn() % 51u);
                /* [2026-08-22] verrou 2 (workflow) : en natif la mission ne bascule
                 * pas à 6 sur la trame SCH (get_task_md fallback reste 5/0), donc le
                 * filtre FCCH-only étouffait AUSSI le SCH → 0x0e4e (SB DMA dest) drainé
                 * sur un burst FCCH/idle périmé → sb_toa garbage. GSM 05.02 : sur la
                 * BCCH en acquisition, seuls FCCH (tonalité) et SCH (synchro) portent
                 * du sens à TS0. On pousse donc FCCH **et** SCH (=FCCH+1). Le DMA route
                 * par AAD (FCCH→0x0cce, SCH→0x0e4e) sur des trames DIFFÉRENTES, donc
                 * 0x0cce reste propre (que du FCCH), 0x0e4e reçoit enfin un vrai SCH. */
                int _is_fcch = ((_p % 10) == 1) && (_p <= 41); /* FCCH ∈ {1,11,21,31,41} */
                int _is_sch  = ((_p % 10) == 2) && (_p <= 42); /* SCH  ∈ {2,12,22,32,42} */
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

