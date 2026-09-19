/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * c54x_internal.h - internal header of the C54x core.
 *
 * [2026-09-18] calypso_c54x.c had grown to 21475 lines and nobody could find
 * anything in it. The core is now split by role:
 *   calypso_c54x.c  init / reset / c54x_run loop / public API
 *   c54x_exec.c     c54x_exec_one and the instruction families
 *   c54x_decode.c   operand resolution (Smem/Lmem/Xmem), conditions
 *   c54x_mem.c      data and program memory, overlay, locks
 *   c54x_irq.c      IFR/IMR, frame interrupt, interrupt dispatch
 *   c54x_probes.c   probes and traces (diagnostics only)
 *
 * This header carries what those files share: includes, constants, types and
 * core global state. It is STRICTLY internal.
 *
 * File-scope anonymous structs were given names (c54x_<var>_s): two anonymous
 * definitions are DIFFERENT types from one translation unit to the next, which
 * makes them impossible to share.
 */
#ifndef CALYPSO_C54X_INTERNAL_H
#define CALYPSO_C54X_INTERNAL_H


#include "calypso_c54x.h"
#include "calypso_rif.h"
#include "calypso_rhea_dma.h"
#include "hw/arm/calypso/calypso_xio.h"   /* SAM/HOM arbitration */
#include "calypso_mailbox.h"
#include "calypso_dma.h"
#include "calypso_arm2dsp.h"
#include "hw/arm/calypso/calypso_invariants.h"
#include "calypso_bsp.h"
#include "hw/arm/calypso/calypso_trf6151.h"
#include "calypso_full_pcb.h"  /* daram_lock, api_ram_lock */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>   /* DARAM-SANITY: coherence/dphi of the corr buffer */
/* DARAM-FNSTAMP counters, defined in calypso_bsp.c. */
extern unsigned calypso_daram_last_fn;
extern unsigned calypso_daram_wr_count;

extern int calypso_rxfb_fired;   /* [probe golive] defined in calypso_bsp.c */

static int g_boot_trace = 0;
/* VEC28-STACK-TRACE, gated by CALYPSO_TRACE_VEC28_STACK. READ-ONLY diagnostic:
 * traces the software stack from a vec28 interrupt entry (2-word PC+XPC push by
 * c54x_interrupt_ex) through every RET-family instruction until SP returns to
 * its pre-interrupt level, to show whether a 1-word RET/RCD/RETED pop consumes
 * the orphaned XPC word (expected 0x0000) as if it were a return PC. Observes
 * only; no DSP state is modified. */
static int g_vec28_trace_en = -1;
static bool g_vec28_tracing = false;
static uint16_t g_vec28_sp_entry = 0;
static unsigned g_vec28_trace_pops = 0;

#include "hw/arm/calypso/calypso_debug.h"

/* Legacy C54_LOG: gated by CALYPSO_DEBUG containing "C54X" or "ALL".
 * For per-probe gating use C54_DBG("PROBE_NAME", fmt, ...). */
#define C54_LOG(fmt, ...) \
    do { if (calypso_debug_enabled("C54X")) \
        fprintf(stderr, "[c54x] " fmt "\n", ##__VA_ARGS__); } while (0)

/* ================================================================
 * Helpers
 * ================================================================ */

/* Sign-extend 40-bit accumulator */
static inline int64_t sext40(int64_t v)
{
    if (v & ((int64_t)1 << 39))
        v |= ~(((int64_t)1 << 40) - 1);
    else
        v &= ((int64_t)1 << 40) - 1;
    return v;
}

/* Saturate 40-bit to 32-bit (OVM mode) */
static inline int64_t sat32(int64_t v)
{
    if (v > 0x7FFFFFFF) return 0x7FFFFFFF;
    if (v < (int64_t)(int32_t)0x80000000) return (int64_t)(int32_t)0x80000000;
    return v;
}

/* Get ARP from ST0 */
static inline int arp(C54xState *s)
{
    return (s->st0 >> ST0_ARP_SHIFT) & 7;
}

/* Get DP from ST0 */
static inline uint16_t dp(C54xState *s)
{
    return s->st0 & ST0_DP_MASK;
}

/* Get ASM from ST1 (5-bit signed) */
static inline int asm_shift(C54xState *s)
{
    int v = s->st1 & ST1_ASM_MASK;
    if (v & 0x10) v |= ~0x1F;  /* sign extend */
    return v;
}

/* ---- shared constants and types (original order preserved) ---- */
#define C54_LOG(fmt, ...) \
    do { if (calypso_debug_enabled("C54X")) \
        fprintf(stderr, "[c54x] " fmt "\n", ##__VA_ARGS__); } while (0)
typedef struct {
    uint64_t per_pc[0x10000];
    uint64_t total;
    uint64_t last_log_insn;
    uint64_t last_summary_insn;
    uint16_t last_exec_pc;
} WatchWriteState;
#define AR_HIST_MAX 64
typedef struct {
    uint16_t pc;
    uint16_t op_last;
    uint16_t val_last;
    uint32_t count;
} ArEntry;
typedef struct { uint16_t pc, op, val, sp; char kind; } St0Ev; /* kind P=push p=pop L=ldp */
#define ST0_RING_N 24
struct sp_evt { uint16_t pc; uint16_t op; int16_t delta; uint16_t sp; };
struct shadow_ent { uint16_t pc; uint16_t op; uint16_t sp; char kind; };
#define SHADOW_N 512
#define STKSLOT_LO 0x1100
#define STKSLOT_HI 0x1140
#define STKSLOT_N  (STKSLOT_HI - STKSLOT_LO + 1)
#define SP_ABS_HIST_MAX 128
typedef struct {
    uint16_t pc;
    uint16_t value_last;
    uint32_t count;
    uint8_t  site;   /* 0=MMR_SP, 1=LDK8, 2=MVMM */
} SpAbsEntry;
#define MVPD_BUCKET_BITS 7      /* 0x80 = 128 words per bucket */
#define MVPD_BUCKET_SZ   (1u << MVPD_BUCKET_BITS)
#define MVPD_RANGE_LO    0x0080
#define MVPD_RANGE_HI    0x2800
#define MVPD_BUCKETS_N   (((MVPD_RANGE_HI - MVPD_RANGE_LO) + MVPD_BUCKET_SZ - 1) / MVPD_BUCKET_SZ)
#define CORR_PC_LO 0x8d00
#define CORR_PC_HI 0x9000   /* exclusive */
#define CORR_READ_HIST_MAX 128
typedef struct { uint16_t addr; uint32_t count; } CorrReadEntry;
#define STUCK_HIST_SIZE 64
typedef struct {
    uint16_t pc;
    uint8_t  xpc;
    uint32_t count;
} StuckHistEntry;
#define INT3_BRANCH_TRACE_MAX 1024
typedef struct {
    uint16_t pc;        /* PC of branch insn */
    uint16_t op;        /* opcode word 0 */
    uint16_t next_pc;   /* PC after exec (= branch taken target OR fall-through) */
    uint32_t insn_offset; /* delta from cycle start (first occurrence) */
    uint32_t repeat;    /* consecutive identical (pc,op,next_pc) collapsed count */
} Int3BranchEvent;
enum { RR_MMRS, RR_LOW, RR_APIRAM, RR_TARGET, RR_WRAP, RR_OTHER, RR_NUM };
#define NOP_RING_N 32
typedef struct {
    uint16_t src_pc;
    uint8_t  src_xpc;
    uint16_t op;
    uint16_t tgt_pc;
    uint8_t  tgt_xpc;
    int64_t  a_val;
    uint64_t insn;
    char     type[8];   /* "B", "BACC", "CALA", "FB", "FCALL", "FBACC", "FCALA", "RET", "FRET", "OTHER" */
} XferLog;
typedef struct {
    uint16_t pc;
    uint8_t  xpc;
    uint16_t op;
    int64_t  old_a;
    int64_t  new_a;
    uint64_t insn;
} AWriteLog;
#define RMAP_PCS 48
typedef struct { uint16_t addr, val, pc, op; } StkwEv;
#define STKW_RING_N 48
#define WMAP_PCS 64
#define WMAP_TICK 2000
#define DSP_IDLE_FF_MAX_RANGES 4
typedef struct {
    uint16_t pc;
    uint16_t op_last;
    uint32_t dec_count;
    int32_t  delta_sum;   /* negative = net drain */
} SpDecEntry;
#define SP_HIST_MAX 512
#define SP_RING_SZ 4096  /* must be a power of two; 4096 covers the thousands
                          * of instructions preceding the SP spiral */
typedef struct {
    unsigned insn;
    uint16_t pc;
    uint16_t sp;
    uint16_t op;
    uint16_t _pad;
} SpRingEntry;

/* ---- named structures for the core global state ---- */
struct c54x_g_fb_det_timing_s {
    /* Timing trackers, updated in data_write over 0x2bc0..0x2bff */
    uint64_t last_compute_insn;
    uint16_t last_compute_addr;
    uint64_t last_clear_insn;
    uint16_t last_clear_addr;
    uint64_t last_pattern_insn;
    uint16_t last_pattern_addr;
    /* Stats captured when 0x8f51 fires */
    uint64_t fb_det_total;
    uint64_t fb_det_ar4_in_zone;
    uint64_t fb_det_ar4_outside;
    uint64_t fb_det_dar4_zero;
    uint64_t fb_det_dar4_sentinel;
    uint64_t fb_det_dar4_other;
    /* Sweep tracking: one sweep = 50 consecutive 0x8f51 fires with AR3 walking
     * 0..0x3A3 at stride +19. A wrap (ar3 < last_ar3) starts a new sweep. */
    uint16_t last_ar3_at_fire;
    uint64_t sweep_id;
    uint64_t sweep_nonzero_count;
};
struct c54x_g_throughput_s {
    uint64_t last_logged_insn;
    struct timespec last_logged_ts;
};
struct c54x_g_read_stats_s {
    uint64_t cumulative[RR_NUM];
    uint64_t snapshot[RR_NUM];
    uint64_t trigger_count;
};
struct c54x_g_rmap_s { uint16_t pc; uint32_t n; uint16_t a[8]; uint8_t na;
                uint16_t amn, amx; };
struct c54x_g_wmap_s { uint16_t pc; uint32_t n; uint16_t v[8]; uint8_t nv;
                uint16_t mn, mx; uint16_t addr0; uint32_t n0; };
struct c54x_g_sp_ledger_s {
    uint64_t sp_pushes;        /* SP delta < 0 events */
    uint64_t sp_pops;          /* SP delta > 0 events */
    int64_t  net_words;        /* total words pushed - total words popped */
    uint64_t irq_entries;      /* count of c54x_interrupt_ex actual dispatches */
    uint64_t irq_words_pushed; /* words written by IRQ entry path (1 or 2 per APTS) */
    uint64_t last_dump_insn;
};
struct c54x_g_sp_trail_s {
    unsigned insn;
    uint16_t old_sp, new_sp, exec_pc, exec_op, a_low;
};

/* ---- etats globaux du coeur, definis dans l'un des .c ---- */
extern uint16_t g_last_d_burst_d;
extern struct c54x_g_fb_det_timing_s g_fb_det_timing;
extern ArEntry  g_ar_hist[8][AR_HIST_MAX];
extern unsigned g_ar_used[8];
extern unsigned g_ar_total[8];
extern unsigned g_ar_mask;
extern int      g_ar_enabled;
extern unsigned g_ar_log_cap;
extern int64_t  g_a_last_value;
extern uint16_t g_a_last_writer_pc;
extern uint16_t g_a_last_writer_op;
extern unsigned g_a_last_writer_insn;
extern int      g_a_trace_enabled;
extern uint16_t g_a_trace_pc;
extern unsigned g_a_trace_hits;
extern unsigned g_a_trace_log_cap;
extern uint16_t g_ar6_last_value;
extern uint16_t g_ar6_last_writer_pc;
extern uint16_t g_ar6_last_writer_op;
extern unsigned g_ar6_last_writer_insn;
extern int      g_ar6_at_enabled;
extern uint16_t g_ar6_at_pc;
extern unsigned g_ar6_at_win_lo;
extern unsigned g_ar6_at_win_hi;
extern unsigned g_ar6_at_hits;
extern unsigned g_ar6_at_log_cap;
extern uint64_t g_rsbx_intm_hits;
extern int      g_rsbx_intm_enabled;
extern uint64_t g_last_intr_insn;
extern int      g_last_intr_vec;
extern uint16_t g_last_intr_fg_pc;
extern uint16_t g_last_intr_fg_dp;
extern uint16_t g_last_ldp_pc;
extern uint16_t g_last_ldp_val;
extern int      g_last_ldp_kind;
extern uint16_t g_prev_pc;
extern uint16_t g_prev_op;
extern uint16_t g_last_st0w_pc;
extern uint16_t g_last_st0w_val;
extern uint16_t g_last_st0w_op;
extern uint16_t g_last_st0w_xpc;
extern uint16_t g_last_st0w_prev;
extern St0Ev    g_st0_ring[ST0_RING_N];
extern unsigned g_st0_ring_idx;
extern int      g_st0_ring_on;
extern uint16_t g_disp_lut_ea;
extern uint16_t g_disp_lut_val;
extern struct sp_evt g_spring[64];
extern uint32_t g_spring_idx;
extern struct shadow_ent g_shadow[SHADOW_N];
extern int  g_shadow_depth;
extern int  g_shadow_on;
extern uint64_t g_orphan_hits;
extern uint64_t g_mismatch_hits;
extern uint16_t g_stkslot_wpc[STKSLOT_N];
extern uint16_t g_stkslot_wop[STKSLOT_N];
extern uint8_t  g_stkslot_written[STKSLOT_N];
extern SpAbsEntry g_sp_abs_hist[SP_ABS_HIST_MAX];
extern unsigned   g_sp_abs_used;
extern unsigned   g_sp_abs_total;
extern int        g_sp_abs_enabled;
extern unsigned   g_sp_abs_log_cap;
extern uint32_t g_mvpd_buckets[MVPD_BUCKETS_N];  /* 128 words each */;
extern int      g_mvpd_trace_enabled;
extern unsigned g_mvpd_boot_limit;
extern int      g_mvpd_dumped;
extern CorrReadEntry g_corr_read_hist[CORR_READ_HIST_MAX];
extern unsigned    g_corr_read_used;
extern int         g_corr_trace_enabled;
extern unsigned    g_corr_entry_count;
extern unsigned    g_corr_entry_log_cap;
extern uint64_t    g_corr_read_total;
extern uint16_t    g_corr_last_pc;
extern int g_fbdb_probe_enabled;
extern unsigned g_fbdb_probe_log_cap;
extern unsigned g_fbdb_probe_count_b;
extern unsigned g_fbdb_probe_count_a;
extern unsigned g_fbdb_probe_count_fbf3;
extern unsigned g_addr3dc0_wr_count;
extern unsigned g_addr3dc0_rd_count;
extern StuckHistEntry g_stuck_hist[STUCK_HIST_SIZE];
extern unsigned g_stuck_hist_used;
extern int g_stuck_probe_enabled;
extern int g_stuck_active;
extern uint32_t g_stuck_duration;
extern uint64_t g_stuck_start_insn;
extern unsigned g_stuck_dump_count;
extern int g_force_intm_oneshot_enabled;
extern int g_force_intm_oneshot_done;
extern uint64_t g_force_intm_oneshot_insn;
extern uint16_t g_force_intm_at_pc;
extern Int3BranchEvent g_int3_trace[INT3_BRANCH_TRACE_MAX];
extern unsigned g_int3_trace_count;
extern int g_int3_trace_overflow;
extern int g_int3_cycle_active;
extern uint64_t g_int3_cycle_id;
extern uint16_t g_int3_cycle_entry_pc;
extern uint64_t g_int3_cycle_entry_insn;
extern int g_int3_trace_enabled;
extern struct c54x_g_throughput_s g_throughput;
extern struct c54x_g_read_stats_s g_read_stats;
extern XferLog   g_xfer_ring[NOP_RING_N];
extern unsigned  g_xfer_idx;
extern AWriteLog g_awrite_ring[NOP_RING_N];
extern unsigned  g_awrite_idx;
extern int       g_nop_tripped;
extern int      g_fbwatch_on;
extern struct c54x_g_rmap_s g_rmap[RMAP_PCS];
extern int      g_rmap_n;
extern uint32_t g_rmap_tot;
extern int      g_rmap_on;
extern uint16_t g_rmap_pclo, g_rmap_pchi;
extern StkwEv   g_stkw_ring[STKW_RING_N];
extern unsigned g_stkw_idx;
extern int      g_orphan_on;
extern FILE *g_flow_f;
extern long  g_flow_budget;
extern int   g_flow_armed;
extern struct c54x_g_wmap_s g_wmap[WMAP_PCS];
extern int      g_wmap_n;
extern uint32_t g_wmap_tot;
extern int      g_wmap_on;
extern uint16_t g_wmap_lo, g_wmap_hi, g_wmap_lo2, g_wmap_hi2;
extern int      g_dio_on;
extern uint64_t g_dio_after;
extern unsigned g_dio_n;
extern uint16_t g_dio_pclo, g_dio_pchi;
extern struct c54x_g_sp_ledger_s g_sp_ledger;
extern uint16_t pc_ring[256];
extern int pc_ring_idx;
extern bool g_frame_it_level;
extern struct c54x_g_sp_trail_s g_sp_trail[256];
extern unsigned g_sp_trail_idx;
extern uint16_t g_sp_low;
extern uint16_t g_sp_low_pc;
extern unsigned g_sp_low_hits_at_pc;
extern unsigned g_sp_low_distinct_pcs;
extern SpDecEntry g_sp_dec_hist[SP_HIST_MAX];
extern unsigned   g_sp_dec_used;
extern unsigned   g_sp_dec_total_events;
extern uint16_t   g_sp_dec_arm_threshold;
extern uint16_t   g_sp_dec_dump_threshold;
extern int        g_sp_dec_enabled;
extern int        g_sp_dec_armed;
extern int        g_sp_dec_dumped;
extern unsigned   g_sp_dec_arm_insn;
extern uint16_t   g_sp_dec_arm_sp;
extern SpRingEntry g_sp_ring[SP_RING_SZ];
extern unsigned    g_sp_ring_head;
extern uint64_t    g_sp_ring_total;
extern int         g_sp_ring_enabled;
extern unsigned    g_sp_ring_dump_count;
extern unsigned    g_sp_ring_dump_max;
extern int         g_sp_ring_trig_mode;
extern unsigned    g_sp_ring_insn_min;
extern int g_bootstub_dumped;
extern bool g_c54x_early_booted;
extern uint32_t g_arm_taskmd5_insn;
extern uint16_t g_arm_taskmd5_ea;

/* ---- functions shared between the core files ---- */
uint16_t prog_fetch(C54xState *s, uint16_t pc);
uint16_t prog_read(C54xState *s, uint32_t addr);
uint16_t c54x_ovly_bas(void);

void a_track_init_lazy(void);
void a_track_iter(C54xState *s, uint16_t prev_pc, uint16_t prev_op);
void ar6_at_init_lazy(void);
void ar6_at_iter(C54xState *s, uint16_t prev_pc, uint16_t prev_op);
void awrite_log_push(uint16_t pc, uint8_t xpc, uint16_t op, int64_t old_a, int64_t new_a, uint64_t insn);
int c54x_exec_one(C54xState *s);
void c54x_fire_tint(C54xState *s);
void c54x_ifr_clear(C54xState *s, uint16_t mask, const char *site);
bool calypso_fix_enabled(const char *name);
const char *classify_xfer_op(uint16_t op);
void corr_entry_track(uint16_t pc, void *s_void);
void corr_read_dump(const char *trig);
void corr_trace_init_lazy(void);
uint16_t data_read(C54xState *s, uint16_t addr);
void data_write(C54xState *s, uint16_t addr, uint16_t val);
void fbdb_probe_check_pc(uint16_t pc, void *s_void);
void force_intm_oneshot_check(C54xState *s);
bool frame_it_level_on(void);
bool frame_it_prio_on(void);
void int3_cycle_end_good(C54xState *s, uint16_t return_addr);
void int3_cycle_start(C54xState *s, uint16_t target_pc);
void int3_cycle_track_branch(C54xState *s, uint16_t exec_pc, uint16_t exec_op, int consumed);
void mvpd_trace_dump_if_due(unsigned insn);
void mvpd_trace_init_lazy(void);
void nop_guard_dump(C54xState *s, uint16_t pc, uint8_t xpc);
int pc_in_nop_region(const C54xState *s, uint16_t pc, uint8_t xpc);
void stuck_probe_check(C54xState *s);
void xfer_log_push(uint16_t src_pc, uint8_t src_xpc, uint16_t op, uint16_t tgt_pc, uint8_t tgt_xpc, int64_t a_val, uint64_t insn);

void ar_write_track(C54xState *s, unsigned idx, uint16_t new_val);
uint16_t c54x_circ_ref(uint16_t ar, int step, uint16_t bk);
bool c54x_cond_true(C54xState *s, uint8_t cc);
bool c54x_irq_level_check(C54xState *s);
void c54x_par_postmod(C54xState *s, int ar, int mod);
uint32_t c54x_prog_xlate(const C54xState *s, uint16_t addr16);
void corr_read_record(uint16_t addr);
void fbdb_probe_read_3dc0(uint16_t addr, uint16_t val, uint16_t pc, unsigned insn);
void fbdb_probe_write_3dc0(uint16_t addr, uint16_t old_val, uint16_t new_val, uint16_t pc, unsigned insn);
void mvpd_trace_record(uint16_t addr);
void read_stats_record(uint16_t addr);
void read_stats_trigger_check(C54xState *s);
uint16_t resolve_lmem(C54xState *s, uint16_t opcode);
uint16_t resolve_smem(C54xState *s, uint16_t opcode, bool *indirect);
uint16_t resolve_xmem(C54xState *s, uint16_t op);
void rsbx_intm_check(C54xState *s, uint16_t op);
void scratchwr_note(C54xState *s, uint16_t a, uint16_t v, const char *espace);
void sp_abs_track(C54xState *s, uint16_t new_val, uint8_t site);
void st0_ring_rec(C54xState *s, uint16_t val, char kind);
void throughput_tick(uint64_t insn_count);
bool watch_write_zone_check(C54xState *s, uint16_t addr, uint16_t val, const char *name, uint16_t lo, uint16_t hi, WatchWriteState *st);

void prog_write(C54xState *s, uint32_t addr, uint16_t val);

uint16_t c54x_revcarry(C54xState *s, uint16_t ar, uint16_t ar0, int sub);

#endif /* CALYPSO_C54X_INTERNAL_H */
