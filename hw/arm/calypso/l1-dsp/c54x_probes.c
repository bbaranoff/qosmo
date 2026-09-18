/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * c54x_probes.c — Sondes, traces et instrumentation (diagnostic seul)
 *
 * Extrait de calypso_c54x.c le 2026-09-18 (decoupage par role).
 * Carte des fichiers dans c54x_internal.h.
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

/* PROBE 2026-05-31 (review c web) : dernier PC/op ayant écrit chaque slot de
 * pile (zone 0x1000-0x1FFF). Nomme le PSHM qui a posé l'orphelin 0x80fd lu par
 * POPM ST0 @0x94f3 → matching-frame vs frame étranger. À RETIRER. */

/* === Generic watch-write zone helper (2026-05-15 matin) ===
 *
 * Factorisation du pattern COEFFS-WR / A_CD-WR / ... : pour chaque zone
 * mémoire surveillée, maintient per-PC counter + log throttled + summary
 * périodique. La 4e+ instrumentation devient triviale au call site.
 *
 * Cost : 512 KB de statique par zone (per_pc[0x10000] × uint64_t). Acceptable
 * pour debug. */

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
        /* Format `<NAME>-WR-SUMMARY` (avec -WR- infix) pour cohérence avec
         * les per-hit lines `<NAME>-WR #N` et backward-compat regex tests. */
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

/* === FB-det timing/content stats (2026-05-14 night) ===
 *
 * Captures sur les ~928 fires de 0x8f51 (au lieu du cap 50 actuel) :
 *   - AR4 dans/hors zone [0x2bc0..0x2bff] → addressing vs timing
 *   - delta insn depuis dernier write par cluster (compute/clear/pattern)
 *   - histogramme val[AR4] : zero / 0xfffe sentinel / other
 *
 * Verdict :
 *   ar4_in_zone < 100% → bug d'addressing (AR4 pointe hors zone)
 *   delta_clear < delta_compute systématique → timing race (clear gagne)
 *   delta_compute >> 1M → compute jamais dans la fenêtre du fire
 *   other > 0 → certains sweeps voient des données — creuser ces fires
 */
struct c54x_g_fb_det_timing_s g_fb_det_timing;

/* === Generic ARn write tracer with provenance (2026-05-25 v3 unified) ===
 * Tracer paramétré pour AR0..AR7. Remplace les ad-hoc ar2/ar4. Env mask
 * `CALYPSO_AR_TRACE` (hex, default 0) :
 *   =0xFF  → trace tous les AR0..AR7
 *   =0x14  → trace AR2 + AR4 seulement (bits 2 et 4 set)
 *   =0x04  → AR2 only
 *   =0     → désactivé (zéro coût)
 *
 * Hook : `case MMR_AR0..AR7` dans data_write_locked. Skip auto-modify
 * noise (Δ ∈ [-3, 3]). Classification opcode via decode + flag ZERO
 * automatique (= suspect clobber MMR via STL A,*AR-).
 *
 * Question résolue par cette sonde : qui pose ARn = mauvaise valeur ?
 *   STM-#lk    → immediate hardcoded ROM (silicon-intentional, le fix
 *                est ailleurs : étape ultérieure qui ré-set manque)
 *   MVDM-mem   → load depuis mem (slot uninit divergence QEMU vs silicon)
 *   MVMM       → copie d'un autre AR (remonter le tracer sur source)
 *   STM Smem   → load mem indirect (idem MVDM)
 *   STLM-A     → from accumulator A (vérifier d'où A vient) */
ArEntry  g_ar_hist[8][AR_HIST_MAX];
unsigned g_ar_used[8]    = {0};
unsigned g_ar_total[8]   = {0};
unsigned g_ar_mask       = 0;
int      g_ar_enabled    = -1;
unsigned g_ar_log_cap    = 50;

void ar_write_track(C54xState *s, unsigned idx, uint16_t new_val)
{
    /* AR3-PRELOAD (revival dsp 2026-06-22, read-only, toujours actif, cape 120) :
     * capture les LOADS d'AR3 dans/autour du buffer I/Q [0x2a00..0x2c00). Le
     * correlateur PC=0xee38 lit AR3=0x2b97 (HORS buffer, fin=0x2b28). ar_write_track
     * n'est appele QUE sur load MMR (STM/STLM/MVDM), PAS sur auto-increment.
     * VERDICT :
     *  - AR3 loade a ~0x2a00 (debut buffer) et AUCUN load >=0x2b28 ici -> le 0x2b97
     *    vient d'INCREMENTS = boucle trop longue / buffer trop court = FIX B (BSP).
     *  - AR3 loade directement >=0x2b28 (flag OUT-OF-BUF) -> instruction mal emulee
     *    = FIX A (decodage c54x), avec le PC/op coupable. */
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
    /* Distinction sémantique critique (cf review Claude web 2026-05-25 v2) :
     * - STM-#lk    = deliberate AR update via immediate hardcoded ROM
     *                (silicon-intentional, l'AR change par design firmware)
     * - LD-#k      = idem (small immediate)
     * - STL-A / autres = side-effect d'un MMR write where AR happens to
     *                    self-alias (= AR pointing at its own MMR slot).
     *                    NOT an explicit AR update — coincidence pointer.
     * Label clairement pour ne pas confondre les 2 dans le hunt. */
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

/* === A accumulator provenance tracer (2026-05-25 v3) ===
 * Capture le LAST WRITER de A via chokepoint top-of-loop : compare A
 * iter-à-iter, si change → mémorise PC + op du writer. Quand un trigger
 * PC fire (default = 0x9ac0 = STL A,*AR2- clobber IMR), dump A + last
 * writer. Réponse à la question Claude web : A=0 délibéré (= mask-all
 * design firmware) ou A=0 divergence (= A devait porter mask valide) ?
 *
 * Env : CALYPSO_A_TRACE_PC=0x9ac0 (hex PC trigger, default 0xFFFF=off)
 * Zéro coût si env non set. */
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
    /* Detect A change → mémorise dernier writer */
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
        /* Log if (a) parmi les N premiers (contexte) OR (b) A_low=0 (= cas
         * suspect STL clobber zone). Évite cap log silencieux qui masque
         * les events critiques tardifs (cf cas insn=253328 IMR clobber). */
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

/* === AR6 windowed snapshot at trigger PC (2026-05-25 v4) ===
 * Capture AR6 + B + source provenance à chaque fire d'un PC trigger,
 * fenêtré sur [insn_lo, insn_hi] pour éviter explosion log (le PC 0x821a
 * fire 10M+ fois). Réponse à la question Claude web : aux fires qui
 * clobber IMR à PC=0x821a, AR6 vaut 0 (= base divergence) ou 0x16
 * (= self-alias feedback) ?
 *
 * Tracking AR6's last writer (= what set AR6 to its current value)
 * via top-of-loop comparison (même pattern que A tracer).
 *
 * Env :
 *   CALYPSO_AR6_AT_PC=0x821a    PC trigger
 *   CALYPSO_AR6_WIN_LO=3619500  insn window start
 *   CALYPSO_AR6_WIN_HI=3619810  insn window end (one outer-loop iter)
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
    /* Trigger : PC about to execute matches AND within window */
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


/* RSBX INTM hits counter (cheap probe, candidat 1 du doc §7). */
uint64_t g_rsbx_intm_hits = 0;
int      g_rsbx_intm_enabled = -1;

/* DISP-ENTRY discriminateur (c web 2026-05-29) : capture du contexte de
 * préemption d'IT, pour trancher "dispatcher atteint via vecteur IT avec DP
 * foreground sale" (b) vs "DP clobbé" (a). C54x n'empile PAS ST0/DP à l'IT →
 * l'ISR hérite du DP du code interrompu. Si les entrées dispatcher KO
 * (DP≠0x124) corrèlent avec une IT récente → préemption confirmée. */
uint64_t g_last_intr_insn  = 0;      /* insn_count de la dernière IT servie */;
int      g_last_intr_vec   = -1;     /* vecteur de la dernière IT */;
uint16_t g_last_intr_fg_pc = 0;      /* PC foreground préempté */;
uint16_t g_last_intr_fg_dp = 0;      /* DP foreground préempté */;
/* dernier LDP (qui a posé DP) + PC prédécesseur (comment on arrive à 0x8341) */
uint16_t g_last_ldp_pc  = 0;         /* PC de l'instruction qui a posé DP */;
uint16_t g_last_ldp_val = 0;         /* valeur DP posée */;
int      g_last_ldp_kind = 0;        /* 1=LDP#k(5902) 2=LDP#k9(6262) 3=LD Smem,DP(7049) */;
uint16_t g_prev_pc = 0;
uint16_t g_prev_op = 0;  /* opcode insn precedente - bc TC-fiable apres cmpm/bitf (2026-06-23) */              /* PC de l'instruction exécutée juste avant */;
uint16_t g_last_st0w_pc  = 0;        /* PC du dernier write ST0 entier (POPM ST0/STLM) */;
uint16_t g_last_st0w_val = 0;        /* valeur ST0 restaurée */;
uint16_t g_last_st0w_op  = 0;        /* opcode de l'instruction qui écrit ST0 */;
uint16_t g_last_st0w_xpc = 0;        /* XPC au moment du write (0xf48b dépend de XPC) */;
uint16_t g_last_st0w_prev = 0;       /* PC prédécesseur du write (comment on y arrive) */;

/* === ST0 push/pop ring (C-sweep 2026-05-30, gated DISP-ENTRY) ============
 * Capture PSHM ST0 (push) + POPM/STLM ST0 (write) dans un ring. Dumpé au
 * dispatcher BAD (lut != 0xff72) pour discriminer le DP périmé en 3 branches :
 *   - dernier PUSH val=0x3124 mais POP=0x3125 → CLOBBE pile entre push/pop
 *     (famille SP/circulaire)
 *   - dernier PUSH val=0x3125 → DP déjà faux au push (LDP sauté en amont)
 *   - pas de PUSH ST0 apparié au POP → désalignement SP (pop lit autre slot) */
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
/* SURGICAL 2026-05-30 : slot LUT lu à l'entrée dispatcher 0x834d (capture
 * silencieuse, pour épingler le DP coupable du self-CALA 0x70c3 sans le
 * spam de DISP-TRACE qui décale le timing et masque le bug). */
uint16_t g_disp_lut_ea  = 0;
uint16_t g_disp_lut_val = 0;
/* SURGICAL 2026-05-30 : ring des évènements SP (push/pop) au chokepoint
 * unique de la boucle run. Sur tout changement de SP on enregistre
 * {pc, op, delta}. Dumpé par BLACKHOLE-CALA → nomme la source récurrente
 * de drain (push jamais dépoppé). Écritures array only = ~zéro coût. */
struct sp_evt g_spring[64];
uint32_t g_spring_idx = 0;

/* === SHADOW STACK (pairing push/pop, 2026-05-30, c-web) ===
 * Miroir logique de la pile DSP : à chaque PUSH (CALL/CALLD/PSHM/IRQ → SP-)
 * on empile {pc,op,kind} ; à chaque POP (RET/RETD/RETE/FRET/RETED/POPM → SP+)
 * on dépile et on VÉRIFIE l'appariement. Un POP sur shadow VIDE = return SANS
 * call apparié = LA source de l'over-pop (lit la pile vierge au-dessus de SP_base).
 * On nomme ce return orphelin (PC/op/SP), ce que les 15 victimes 0xc8be ne disent
 * pas. Gated CALYPSO_DEBUG=ORPHAN. kind: 'C'=call 'P'=pshm/pshd 'I'=irq 'R'=reti.
 * Array-only quand off → ~zéro coût. */
struct shadow_ent g_shadow[SHADOW_N];
int  g_shadow_depth = 0;     /* nb de mots actuellement empilés (logique) */;
int  g_shadow_on   = -1;     /* -1 = pas encore résolu le gate */;
uint64_t g_orphan_hits = 0;  /* nb de POP-orphelins détectés */;
uint64_t g_mismatch_hits = 0;/* nb de POP avec kind mismatché */;

/* Tracker stores directs zone pile [0x1100..0x1140] (AU-DESSUS de SP_base) :
 * ces slots ne sont JAMAIS écrits par un push (la pile descend SOUS 0x1100),
 * donc uniquement par un ST direct. Discrimine vecteur-init LÉGIT (slot écrit
 * par le firmware) vs slot VIERGE (jamais écrit = vrai over-pop garbage). */
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

/* === AR4 write tracer with provenance (2026-05-25) ===
 * Hooke chaque write vers MMR_AR4 (= 0x14) via data_write_locked.
 * Logge (insn, PC, opcode courant, val écrite, ancien AR4) + tente une
 * classification de provenance via decode opcode :
 *   STM #lk, ARn  (0x77yx)  → immediate value depuis next prog word
 *   STLM src,ARn  (0x84yx)  → from accumulator A/B low
 *   MVDM dmad,ARn (0x86yx)  → from data memory absolute
 *   MVDD/MVMM     (autres)  → from another register
 * Flag SUSPECT si nouvelle valeur AR4 ∈ [0x2b80..0x2c00] (observed bug
 * zone) ou [0x3fb0..0x3fbf] (BSP buffer area). Env CALYPSO_AR4_TRACE=1.
 *
 * Question critique (cf Claude web) : provenance = const/mem/register ?
 * Si AR4 vient d'un mem load (LDM/MVDM), le corrupter remonte au TCB
 * en mémoire — pas l'instruction qui charge AR4, mais le TCB lui-même
 * (potentiellement uninitialized faute de re-init firmware sautée). */
/* === SP absolute-write tracer (2026-05-25 — nohack hunt) ===
 * Logge chaque write SP via STL/STM/STLM absolute, FRAME #imm, MVMM
 * register transfer — c'est-à-dire les sites où SP est *téléporté* à une
 * valeur arbitraire, par opposition aux PUSH/POP/CALL/RET qui sont des
 * inc/dec de 1. Si un site téléporte SP=0x3fbe, on tient le corrupter
 * exact du bootstub-entry observé à insn=3995013.
 *
 * Hooké aux 3 sites identifiés :
 *   L1218 : data_write_locked case MMR_SP — STL/STM/STLM to MMR_SP
 *   L3875 : F7Dx case 0xD — LD #k8u, SP
 *   L4285 : MVMM register transfer — dst==8 (SP via MMR enc 3-bit)
 *
 * Env-gated CALYPSO_SP_ABS_TRACE=1, zéro coût si OFF.
 * Limite N premiers writes verbatim + histo per-PC (cap SP_ABS_HIST_MAX). */
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
    /* Per-PC histo */
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
    /* Flag if SP lands in suspect zone (0x3fb0..0x3fbf = BSP read region
     * OR 0x2b80..0x2c00 = 0xfd2a A=AR4 historical) */
    if ((new_val >= 0x3fb0 && new_val <= 0x3fbf) ||
        (new_val >= 0x2b80 && new_val <= 0x2c00)) {
        fprintf(stderr,
            "[c54x] SP-ABS SUSPECT! @insn=%u PC=0x%04x SP←0x%04x (corrupter ?)\n",
            s->insn_count, s->pc, new_val);
    }
}

/* === MVPD overlay occupancy trace (2026-05-25) ===
 * Bucket writes à data[0x0080..0x27FF] en buckets de 0x80 words.
 * Dump occupancy à la fin de la boot phase (insn cap) ou périodiquement.
 * Objectif : identifier quelles sub-ranges de [0x0080..0x27FF] sont
 * chargées par MVPD au boot (= code overlay). Critique pour décider si
 * BSP buffer peut vivre dans la read-region [0x0000..0x03A3] sans
 * écraser du code en cours d'exécution.
 * Env gates :
 *   CALYPSO_MVPD_TRACE=1       active (default OFF)
 *   CALYPSO_MVPD_BOOT_LIMIT=N  cap insn pour dump (default 500000) */
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
    /* Verdict pour decision buffer placement : si [0x0080..0x03A3]
     * (correlator read region, bucket 0..6) a peu/zéro writes → safe
     * pour BSP DMA. Sinon il faut une autre zone. */
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

/* === Correlator trace (2026-05-25 — pour run 0x6000 dual-purpose) ===
 * Capture AR3/AR4/AR5 à l'entrée du correlator FB-det + les data reads
 * pendant son exécution. Objectif : valider empiriquement que le firmware
 * lit son input I/Q dans [0x0000..0x03A3] (assertion TODO.md:13 jamais
 * exercée avec real data — l'A/B précédent mesurait WR-SITE pré-BSP).
 * Env-gated CALYPSO_CORRELATOR_TRACE=1, zéro coût si OFF.
 *
 * Correlator range : [0x8d00..0x9000] (FB-det handler PROM0).
 *
 * 2026-05-25 night : range étendu de 0x8F80 → 0x9000. Évidence runtime
 * (d_fb_det WATCH-READ) montrait des reads à PC=0x8FAC et 0x8FB5 qui
 * étaient HORS l'ancien filtre → CORR-ENTRY=0 alors que firmware FAIT
 * des accès dans la zone FB-det. Range élargi pour capturer ces hits.
 *
 * À l'entrée from-outside : log AR0..7, SP, ST0/1.
 * Pendant exec : log les data_read addr (top N uniques, capped pour
 * éviter explosion log sur runs longs). */
CorrReadEntry g_corr_read_hist[CORR_READ_HIST_MAX];
unsigned    g_corr_read_used    = 0;
int         g_corr_trace_enabled = -1; /* -1 uninit, 0 off, 1 on */;
unsigned    g_corr_entry_count  = 0;
unsigned    g_corr_entry_log_cap = 100000;  /* uncap : voir le par-frame post-+3s */;

/* Posés par calypso_trx.c quand l'ARM écrit d_task_md=5 (commande FB).
 * La sonde D_TASK_MD-RD timestampe les reads DSP par rapport à ce write
 * (test H1 : EA write ARM vs EA read DSP + ordre). */
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

/* CORR-ENTRY tracker : appelé au top-of-loop pour chaque insn dispatch.
 * Détecte transition PC out→in du range FB-det. Log les premières N
 * entrées avec contexte AR/SP/ST. */
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

/* === FBDB/FBF3 + 0x3DC0 probes (c web reframe 2026-05-25 night2) ========
 *
 * Sondes diagnostiques POST-désassemblage fc50-fc6f, sans aucun fix.
 * Trois questions à trancher :
 *   A) B @ PC=0xfbd9 vaut-il une valeur cohérente avant `SUB #8, B, A` ?
 *      (= si B faux upstream, F2xx fix donne A faux downstream)
 *   B) A @ PC=0xfbdb juste après SUB → AR4 setup à PC=0xfbf3 → corruption ?
 *   C) Le bit 4 du flag 0x3DC0 (= testé par BITF à fc63) est-il jamais set ?
 *      Si jamais set par aucune routine DSP → BCD NTC à fc66 toujours branche →
 *      fc50-fc6f loop forever sans toucher au body.
 *
 * Env-gated : CALYPSO_FBDB_PROBE=1 active toutes les sondes.
 * Coût off : 1 compare + 1 branch par opcode (négligeable). */
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

/* === STUCK-STATE PC+XPC histogram (c web reframe 2026-05-25 night3) =====
 *
 * Sonde diagnostique : quand le DSP est en "stuck state" (= INTM=1 ET
 * BRINT0 pending dans IFR), on enregistre PC+XPC. Permet d'identifier
 * la VRAIE boucle de blocage XPC-qualifiée — sans présumer que c'est
 * fc50 ou autre PC particulier.
 *
 * Le PC HIST classique ne distingue pas les pages XPC (= ambiguous "fc50"
 * peut être page 0x1F mirror ou 0x28/0x38 etc.).
 *
 * Entry/exit du stuck state logué (= delimit la fenêtre).
 * Top-20 PC+XPC dump périodique (= quand stuck dure).
 *
 * Env-gated CALYPSO_STUCK_PROBE=1. Coût off : 1 bit-check + 1 branch. */
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

/* === FORCE-INTM-ONESHOT (c web reframe 2026-05-25 night4) ===============
 *
 * Sonde d'arbitrage : quand INTM=1 ET BRINT0 (IFR bit 5) pending,
 * forcer UNE SEULE FOIS INTM=0 pour permettre dispatch. Observer ce
 * qui se passe ensuite via les tracers existants (CORR-ENTRY, a_sync_demod
 * writes, RETE log, INTM-TRANS).
 *
 * Partitionne l'arbre :
 *   - snr/toa réels après dispatch → INTM est le SEUL blocker
 *   - garbage / rien → aval cassé aussi (≥2 bugs) OU corruption ISR state
 *
 * IMPORTANT : c'est une SONDE diagnostique, PAS un fix. Le one-shot
 * permet d'observer sans masquer un comportement régulier. Si on confirme
 * "INTM est le seul blocker", le vrai fix sera côté ISR (= pourquoi pas
 * de RETE), pas un INTM-clear systématique.
 *
 * Env-gated CALYPSO_FORCE_INTM_ONESHOT=1. */
int g_force_intm_oneshot_enabled = -1;
int g_force_intm_oneshot_done = 0;
uint64_t g_force_intm_oneshot_insn = 0;
/* CALYPSO_FORCE_INTM_AT_PC=0xfc6f : restreindre le force au PC donné (=
 * point sûr = RET du compute kernel fc50 par exemple). 0xFFFF sentinel
 * = pas de restriction PC (= comportement v1 = première opportunité). */
uint16_t g_force_intm_at_pc = 0xFFFF;

/* @BEQUILLE — FORCE_INTM_ONESHOT (+ FORCE_INTM_AT_PC)  (CALYPSO_FORCE_INTM_ONESHOT=1,
 *              CALYPSO_FORCE_INTM_AT_PC=0xXXXX ; defaut OFF ; calypso_wire.env:=1)
 *   masque  : le RSBX INTM 0xa51b que le firmware ne joue pas. Avec le PC-gate le
 *             bloc POSE EN PLUS l'IT (s->ifr |= s->imr & 0x3000) : il fabrique
 *             l'evenement, il n'ouvre pas seulement la fenetre.
 *   retirer : quand INTM passe a 0 par le chemin ROM (trace INTM-TRANS).
 *   NB      : run.sh le signale deja comme "NON-nominal".
 */
void force_intm_oneshot_check(C54xState *s)
{
    if (g_force_intm_oneshot_enabled < 0) {
        const char *e = getenv("CALYPSO_FORCE_INTM_ONESHOT");
        /* Gate PROPRE : ON seulement si =1 ; =0 ou unset -> OFF. (NB : ce oneshot
         * masque le livelock vec28 en clearant INTM 1x ; utile tant que le sur-fire
         * frame-IT n est pas corrige a la racine BSP.) */
        g_force_intm_oneshot_enabled = (e && *e == '1') ? 1 : 0;
        /* Optional PC gate : si CALYPSO_FORCE_INTM_AT_PC=0xXXXX présent,
         * fire seulement quand PC matche. Permet de départager state-
         * corruption vs aval-cassé per c web : force à un PC sûr (= RET
         * fc6f, idle dispatcher, etc.) au lieu de mid-compute fc57. */
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
    /* [2026-07-22] FAIRE FIRE L IFR. Au go-live (0xa4e1) IMR=0x3000 (bit frame
     * demasque) mais IFR=0 -> aucune IT pending, clearer INTM ne fait rien. Le
     * frame-IT n est jamais latche (modele IT c54x incomplet). Donc AU PC cible :
     * on FORCE l IFR frame pending (IFR |= bits demasques) PUIS on clear INTM ->
     * l IT part. Sans PC-gate : ancien comportement (fire sur IT deja pending). */
    if (g_force_intm_at_pc != 0xFFFF) {
        if (s->pc != g_force_intm_at_pc) return;
        uint16_t unmasked = s->imr & 0x3000;   /* vec28/frame (bit12) + bit13 */
        if (!unmasked) return;                 /* rien de demasque a forcer */
        s->ifr |= unmasked;                    /* <-- pose l IT frame pending */
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

/* === INT3 cycle tracer + control-flow signature (c web reframe 2026-05-25 night5)
 *
 * Sonde décisive pour départager F1 (= pourquoi ISR INT3 ne RETE pas).
 *
 * Per cycle INT3 :
 *   - START : INT3 dispatched (vec=19) → reset trace, log cycle_id + entry PC
 *   - DURING : chaque branch conditionnelle exécutée → (PC, op, target, taken)
 *   - END (good) : RETE fire → dump trace tagged GOOD + insn count
 *   - END (orphan) : nouveau INT3 dispatch avant RETE → dump previous tagged
 *                   ORPHAN-NEXT-INT3 + reason
 *
 * Diff offline good_cycle vs orphan_cycle → 1ère branche qui diverge
 * = trigger du bug. À cette branche, lire l'état testé = la vraie cause.
 *
 * Cappé 256 branches/cycle (= overflow tagué pour borne).
 * Env-gated CALYPSO_INT3_CYCLE_TRACE=1. */
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

/* === DSP throughput emission (2026-05-14 evening) ===
 *
 * Émet `[c54x] INSN-COUNT-STATS total=N delta=N elapsed_ms=N rate=N/s` toutes
 * les 1M insn. Lu en stéréo par :
 *   - test_dsp_throughput_5x (milestones, static)
 *   - test_dsp_throughput_above_threshold (observability, runtime)
 * Seuil pytest : 50M/s (marge ×2 sous les 100M/s historiques). */
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

/* === Read-by-range tracking for FB-det path analysis (2026-05-14 evening) ===
 *
 * Cible : identifier la zone DARAM lue par la routine FB-det sans préjuger.
 * Compteurs cumulatifs par plage + snapshot/delta à chaque "trigger PC"
 * (sites qui écrivent d_fb_det, identifiés par grep ZERO-WR + WR-SITE).
 *
 * Plages mutuellement exclusives :
 *   RR_MMRS   [0x0000..0x005F]  registres MMR C54x
 *   RR_LOW    [0x0060..0x03A3]  zone correlator linéaire (hypothèse 05-14)
 *   RR_APIRAM [0x0800..0x27FF]  API RAM partagée ARM/DSP (hypothèse β)
 *   RR_TARGET [0x3FB0..0x3FFF]  où BSP DMA écrit par défaut
 *   RR_WRAP   [0xFC5D..0xFFED]  zone correlator wrap BK=176 (AR2/AR7)
 *   RR_OTHER  tout le reste (incluant overlay 0x80..7FF, debord 0x4000+, etc.)
 *
 * Trigger PCs : 5 sites observés écrivant d_fb_det (4 ZERO-WR rares + 0x8f51
 * en boucle 50 fois). Le delta entre 2 triggers consécutifs = reads
 * cumulés dans la fenêtre amont. Cap à 200 triggers loggés pour ne pas
 * flooder. */

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
    /* Trigger PC réduit à 0x8f51 uniquement (FB-det compute loop, 50 hits/sweep).
     * 2026-05-14 — Run précédent : trigger list large {0x8f51, 0x778a, 0x9ac0,
     * 0x9ad0, 0x9b00, 0x821a} → 0x821a en boot mailbox poll loop (14 insns
     * entre hits) a dévoré les 200 lignes de cap avant que 0x8f51 ne fire.
     * Les autres PCs étaient init/reset (1-3 hits chacun sur tout le run).
     * Cap remonté à 5000 pour couvrir plusieurs sweeps FB-det. */
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
 * Together they name the racine without spéculation. */



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
/* [2026-08-22] Prototype : le plancher de l alias OVLY est utilise des
 * pc_in_nop_region (ligne ~1595), bien avant sa definition. */
uint16_t c54x_ovly_bas(void);
int c54x_dual_sett(void);
int c54x_mpy_fam(void);
int c54x_ld_par(void);
int c54x_mas_dual(void);

inline int pc_in_nop_region(const C54xState *s, uint16_t pc, uint8_t xpc)
{
    if (xpc != 0) return 0;                 /* banque sup : géré ailleurs */
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

