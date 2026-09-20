/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * c54x_mem.c - data and program memory: data_read/write, prog_fetch, overlay.
 *
 * Split out of calypso_c54x.c on 2026-09-18; file map in c54x_internal.h.
 */
#include "c54x_internal.h"
#include "hw/arm/calypso/calypso_debug.h"

static uint16_t data_read_locked(C54xState *s, uint16_t addr);
/* DEMODIO: forward declaration - the helper is defined further down, but its
 * caller data_read comes first. */
static void dio_note(C54xState *s, const char *rw, uint16_t addr, uint16_t val);

/* FBWATCH: FB-dispatch probe behind its own env (CALYPSO_FBWATCH=1), resolved
 * once into a static int so the hot path costs one int test. Deliberately NOT
 * routed through calypso_debug_enabled: the master gate must stay 0 to keep the
 * 127 debug gates short-circuited and QEMU real-time. Declared here because
 * data_read_locked uses it. Watches page reads, d_fb_det and the canary. */
int      g_fbwatch_on = -1;

uint16_t data_read(C54xState *s, uint16_t addr)
{
    if (c54x_rapide)                       /* see calypso_c54x.h */
        return data_read_locked(s, addr);
    /* Mailbox monitor: logs the value as found in memory on entry. The few cells
     * synthesized further down (FB-STREAM) show up on the write side anyway. */
    calypso_mbx(MBX_DSP_RD, addr, s->data[addr], 0, s->pc, 0, s->insn_count);

    /* FEED-DST - which buffer does the demod actually read? Gate
     * CALYPSO_FEED_DST (default 0), read-only, capped.
     *
     * [2026-08-04] The DMA delivers real IQ (295/296 non-zero words in steady
     * state) yet the DSP still publishes 89 % zeros in a_cd, so check whether the
     * demod reads where the DMA writes before blaming the opcode decoder.
     *
     *   zone A = 0x0cce..0x0df5: DMA destination (AAD=0x99c -> api_ram[0x4ce],
     *            DSP word 0x0800+0x4ce), where the real IQ lands.
     *   zone B = 0x4c00..0x4d27: correlator buffer (CALYPSO_BSP_DARAM_ADDR).
     *
     * A>0 and B=0 -> the demod reads the DMA, the fault is downstream
     * (decoder/ALU). A=0 and B>0 -> it reads a buffer the DMA never feeds, which
     * explains the zeros with no ALU bug. Both 0 -> neither, and the question
     * changes.
     *
     * Counts READS, not frames: an A/B ratio says how many words were read
     * where, not which buffer the demod prefers. The listed PCs are the first 8
     * distinct ones seen, not the most frequent. */
    {
        static int fd_on = -1;
        if (fd_on < 0) {
            fd_on = calypso_gate("CALYPSO_FEED_DST", 0);
            /* Arming trace: without it a silent probe is indistinguishable
             * from a disabled one. ARMEE with no A/B line after it means the
             * demod reads neither 0x0cce nor 0x4c00 - a result, not a failure. */
            fprintf(stderr, "[c54x] FEED-DST %s (CALYPSO_FEED_DST=%d) — "
                    "A=0x0cce..0x0df5 (dest DMA), B=0x4c00..0x4d27 (correlateur)\n",
                    fd_on ? "ARMEE" : "eteinte", fd_on);
        }
        if (fd_on) {
            int zone = (addr >= 0x0cce && addr <= 0x0df5) ? 0
                     : (addr >= 0x4c00 && addr <= 0x4d27) ? 1 : -1;
            if (zone >= 0) {
                static unsigned long long n_rd[2];
                static uint16_t pcs[2][8];
                static unsigned  pcn[2][8];
                static int       npc[2];
                n_rd[zone]++;
                int k;
                for (k = 0; k < npc[zone]; k++)
                    if (pcs[zone][k] == s->pc) { pcn[zone][k]++; break; }
                if (k == npc[zone] && npc[zone] < 8) {
                    pcs[zone][k] = s->pc; pcn[zone][k] = 1; npc[zone]++;
                }
                unsigned long long tot = n_rd[0] + n_rd[1];
                static unsigned n_print;
                if ((tot == 1 || (tot % 20000) == 0) && n_print < 20) {
                    n_print++;
                    fprintf(stderr, "[c54x] FEED-DST A(DMA 0x0cce)=%llu "
                            "B(corr 0x4c00)=%llu\n", n_rd[0], n_rd[1]);
                    for (int z = 0; z < 2; z++) {
                        fprintf(stderr, "[c54x] FEED-DST   %s PC:",
                                z == 0 ? "A" : "B");
                        for (int j = 0; j < npc[z]; j++)
                            fprintf(stderr, " 0x%04x(%u)", pcs[z][j], pcn[z][j]);
                        fprintf(stderr, "%s\n", npc[z] ? "" : " (aucune lecture)");
                    }
                    fflush(stderr);
                }
            }
        }
    }

    /* FB-STREAM (gates CALYPSO_FB_STREAM, CALYPSO_FB_STREAM_CELL / _CELLQ):
     * injects a fresh FCCH sample on every read of the demod sample cell, which
     * gives 0x2a00 a real window independent of cadence; models the on-chip DMA.
     * The I/Q cells are configurable because WATCH-9F00-RD measured the demod
     * reading 0x9260/0x9261, not the 0x9213/0x9215 default. */
    static uint16_t _fscI = 0, _fscQ = 0;
    if (_fscI == 0) {
        const char *c = calypso_getenv("CALYPSO_FB_STREAM_CELL");  _fscI = c ? (uint16_t)strtol(c, NULL, 0) : 0x9213;
        const char *q = calypso_getenv("CALYPSO_FB_STREAM_CELLQ"); _fscQ = q ? (uint16_t)strtol(q, NULL, 0) : 0x9215;
    }
    if (s->pc >= 0x9f00 && s->pc <= 0x9fb8 && (addr == _fscI || addr == _fscQ)) {
        static int _fs = -1;
        /* @BEQUILLE - FB_STREAM (read side)  (CALYPSO_FB_STREAM, default OFF)
         *   masque  : the missing on-chip DMA. Injects a fresh sample on every
         *             read of the cell instead of a peripheral filling the
         *             buffer.
         *   retirer : once the buffer is fed by the real path (BSP -> DARAM).
         *   Note: inert with CORR_ENTRY=0x94f5 - cells 0x9260/61 are never read
         *   there (measured 2026-07-28 by WATCH-9F00-RD). */
        if (_fs < 0) _fs = calypso_gate("CALYPSO_FB_STREAM", 0);
        if (_fs) {
            static uint16_t _si, _sq; static int _hv = 0; uint16_t _rv;
            if (addr == _fscI) { _hv = calypso_bsp_fb_stream_next(&_si, &_sq) ? 1 : 0; _rv = _hv ? _si : s->data[addr]; }
            else { _rv = _hv ? _sq : s->data[addr]; }
            static unsigned _sl = 0;
            if (_sl++ < 24)
                fprintf(stderr, "[c54x] FB-STREAM addr=0x%04x -> 0x%04x (cell 0x%04x) hv=%d PC=0x%04x\n",
                        addr, _rv, s->data[addr], _hv, s->pc);
            return _rv;
        }
    }
    if (s->pc >= 0x9f00 && s->pc <= 0x9fb8) {
        static int _r9 = -1;
        if (_r9 < 0) _r9 = calypso_gate("CALYPSO_WATCH_9F00_RD", 0);
        if (_r9) {
            /* WATCH-9F00-RD (gate CALYPSO_WATCH_9F00_RD): the demod stage at
             * 0x9f00..0x9fb8 writes its result into the 0x2a00 workzone, so its
             * input is whatever it reads. Logs every read on that path, with no
             * exclusion, to locate the real source I/Q buffer. */
            static unsigned _n9 = 0;
            if (_n9++ < 200)
                fprintf(stderr, "[c54x] WATCH-9F00-RD PC=0x%04x reads addr=0x%04x val=0x%04x insn=%u\n",
                        s->pc, addr, s->data[addr], s->insn_count);
        }
    }
    /* Correlator read tracer (env-gated CALYPSO_CORRELATOR_TRACE=1): records
     * addr only while PC is in [CORR_PC_LO..CORR_PC_HI), the FB-det range.
     * Cost when OFF: one compare plus one branch. */
    if (g_corr_trace_enabled > 0 && s->pc >= CORR_PC_LO && s->pc < CORR_PC_HI) {
        corr_read_record(addr);
    }
    /* IQ-READ tracer: who reads the BSP DMA buffer [0x2a00..0x2b27]? Confirms
     * that the FB correlator consumes the real I/Q the BSP wrote, and at which
     * PC (the actual correlator site). Cap 60, ~zero cost outside the zone. */
    if (addr >= 0x2a00 && addr < 0x2b28 && s->data[addr] != 0) {
        static unsigned iqr = 0, iqseen = 0;
        iqseen++;
        /* Boot (first 60) plus 1-in-8000 past insn>50M, so the sample covers
         * what the correlator reads at FB-det time (insn ~71M) and not just the
         * stale boot buffer. */
        if (iqr < 60 || (s->insn_count > 50000000u && (iqseen % 8000) == 0)) {
            uint16_t val = s->data[addr];
            /* A/B (complex correlation accumulators, 40-bit sign-extended)
             * plus the values under the other pointers during the I/Q read. */
            int64_t a = (s->a & 0x8000000000LL) ? (int64_t)(s->a | ~0xFFFFFFFFFFLL) : (int64_t)s->a;
            int64_t b = (s->b & 0x8000000000LL) ? (int64_t)(s->b | ~0xFFFFFFFFFFLL) : (int64_t)s->b;
            fprintf(stderr, "[c54x] IQ-READ #%u addr=0x%04x val=0x%04x PC=0x%04x A=%lld B=%lld "
                    "T=%04x s=%p | AR3=%04x[%04x] AR4=%04x[%04x] AR5=%04x[%04x] insn=%u\n",
                    iqr, addr, val, s->pc, (long long)a, (long long)b, s->t, (void*)s,
                    s->ar[3], s->data[s->ar[3]], s->ar[4], s->data[s->ar[4]],
                    s->ar[5], s->data[s->ar[5]], s->insn_count);
            iqr++;
        }
        /* SPAN write-vs-read: at detection time, dump once the contiguous span
         * data[0x2a00..0x2a1f] (what the correlator can read) and bsp_buf[0..31]
         * (what the BSP wrote).
         *   both waveform    -> buffer fine, bug is AR3/correlator (reads [0] only)
         *   span [0] then DC -> PORTR delivery copies only [0] (stride/len)
         *   bsp_buf DC       -> the BSP cs16 conversion truncates. */
        static int span_done = 0;
        if (!span_done && s->insn_count > 60000000u) {
            span_done = 1;
            fprintf(stderr, "[c54x] SPAN-READ  data[0x2a00..0x2a1f] insn=%u:", s->insn_count);
            for (int _i = 0; _i < 32; _i++) fprintf(stderr, " %04x", s->data[0x2a00 + _i]);
            fprintf(stderr, "\n[c54x] SPAN-WRITE bsp_buf[0..31] (bsp_len=%d):", s->bsp_len);
            for (int _i = 0; _i < 32 && _i < s->bsp_len; _i++) fprintf(stderr, " %04x", s->bsp_buf[_i]);
            fprintf(stderr, "\n");
        }
    }
    /* MTTCG: guards DARAM access, since the DSP and ARM-OVLY can race.
     * Without MTTCG the mutex is uncontended and costs almost nothing. */
    qemu_mutex_lock(&calypso_pcb_daram_lock);
    uint16_t v = data_read_locked(s, addr);
    qemu_mutex_unlock(&calypso_pcb_daram_lock);
    dio_note(s, "R", addr, v);
    return v;
}

static void flow_log(const char *rw, uint16_t addr, uint16_t val, uint16_t pc, unsigned insn);

/* RMAP: aggregated map of the addresses READ by the PCs of a range. Mirror of
 * WMAP. */
struct c54x_g_rmap_s g_rmap[RMAP_PCS];
int      g_rmap_n;
uint32_t g_rmap_tot;
int      g_rmap_on = -1;
uint16_t g_rmap_pclo, g_rmap_pchi;

static void rmap_dump(void)
{
    fprintf(stderr, "[c54x] RMAP PC 0x%04x..0x%04x  lectures=%u  PCs=%d%s\n",
            g_rmap_pclo, g_rmap_pchi, g_rmap_tot, g_rmap_n,
            g_rmap_n >= RMAP_PCS ? "  *** SATUREE ***" : "");
    for (int i = 0; i < g_rmap_n; i++) {
        fprintf(stderr, "[c54x] RMAP   PC=0x%04x n=%-7u lit 0x%04x..0x%04x  ex:",
                g_rmap[i].pc, g_rmap[i].n, g_rmap[i].amn, g_rmap[i].amx);
        for (int k = 0; k < g_rmap[i].na && k < 8; k++)
            fprintf(stderr, " %04x", g_rmap[i].a[k]);
        fprintf(stderr, "\n");
    }
}

static void rmap_note(uint16_t addr, uint16_t pc)
{
    if (g_rmap_on < 0) {
        const char *e = calypso_getenv("CALYPSO_RMAP");
        g_rmap_on = (e && atoi(e) > 0) ? 1 : 0;
        const char *lo = calypso_getenv("CALYPSO_RMAP_PCLO"), *hi = calypso_getenv("CALYPSO_RMAP_PCHI");
        g_rmap_pclo = lo ? (uint16_t)strtoul(lo, NULL, 0) : 0x9f00;
        g_rmap_pchi = hi ? (uint16_t)strtoul(hi, NULL, 0) : 0x9fff;
        if (g_rmap_on)
            fprintf(stderr, "[c54x] RMAP armed PC 0x%04x..0x%04x\n", g_rmap_pclo, g_rmap_pchi);
    }
    if (!g_rmap_on || pc < g_rmap_pclo || pc > g_rmap_pchi) return;

    int i;
    for (i = 0; i < g_rmap_n; i++) if (g_rmap[i].pc == pc) break;
    if (i == g_rmap_n) {
        if (g_rmap_n >= RMAP_PCS) return;
        g_rmap_n++;
        g_rmap[i].pc = pc; g_rmap[i].n = 0; g_rmap[i].na = 0;
        g_rmap[i].amn = 0xffff; g_rmap[i].amx = 0;
    }
    g_rmap[i].n++;
    if (addr < g_rmap[i].amn) g_rmap[i].amn = addr;
    if (addr > g_rmap[i].amx) g_rmap[i].amx = addr;
    if (g_rmap[i].na < 8) {
        int seen = 0;
        for (int k = 0; k < g_rmap[i].na; k++) if (g_rmap[i].a[k] == addr) { seen = 1; break; }
        if (!seen) g_rmap[i].a[g_rmap[i].na++] = addr;
    }
    if (++g_rmap_tot % 5000 == 0) rmap_dump();
}

static uint16_t data_read_locked(C54xState *s, uint16_t addr)
{
    {   /* DTASKD-WATCH, leg 4/4 - CALYPSO_DTASKD_WATCH=1, default 0. Read-only,
         * capped. Legs 1-2 are in calypso_trx.c, leg 3 in data_write_locked.
         *
         * [2026-08-03] Measured with CALYPSO_DISPATCH_PROBE=1: the DSP task
         * queue only ever receives one handler, 0xb5a1, 32559 times; the
         * index->handler helper 0xa9ea runs exactly once over the whole run,
         * with index 42 (disarm). Index 41 (RX arm, 0xa5cd) is never requested
         * and none of its call sites are reached, so the NB/CCCH path is never
         * queued. The question becomes whether the DSP even READS the cell where
         * the ARM drops the task: if the ROM never reads d_task_d it never learns
         * an NB task exists, and everything else follows.
         *
         * Cells (W pages, MCU->DSP side; dsp_api.h:22-23):
         *     W p0 = data[0x0800]     W p1 = data[0x0814]
         * d_task_md (data[0x0804]/[0x0818]) is watched too: it is the task the
         * DSP does consume today, hence the control witness. Without it a silent
         * leg 4 is ambiguous between "does not read d_task_d" and "reads nothing
         * in page W at all".
         *
         * Result:
         *   md read, d read     -> the DSP sees the NB task, the block is later.
         *   md read, d NOT read -> root cause: the ROM ignores d_task_d here.
         *   neither             -> page W is not read at all; go back up to the
         *                          selector 0xaad5 and whatever feeds 0xaac3. */
        static int _dw = -1;
        if (_dw < 0) {
            _dw = calypso_gate("CALYPSO_DTASKD_WATCH", 0);
            if (_dw) {
                fprintf(stderr, "[dtaskd] patte 4/4 armee (lectures DSP page W) : "
                        "d_task_d=data[0x0800]/[0x0814]  d_task_md=data[0x0804]/[0x0818]\n");
                fflush(stderr);
            }
        }
        if (_dw && (addr == 0x0800 || addr == 0x0814 ||
                    addr == 0x0804 || addr == 0x0818)) {
            static unsigned long long _nd = 0, _nmd = 0;
            int is_d = (addr == 0x0800 || addr == 0x0814);
            unsigned long long _n = is_d ? ++_nd : ++_nmd;
            if (_n <= 30 || (_n % 5000) == 0) {
                fprintf(stderr,
                        "[dtaskd] DSP<RD  %-9s data[0x%04x] = 0x%04x  "
                        "(d_task_d lus=%llu  d_task_md lus=%llu)  PC=0x%04x insn=%u\n",
                        is_d ? "d_task_d" : "d_task_md", addr, s->data[addr],
                        _nd, _nmd, s->last_exec_pc, s->insn_count);
                fflush(stderr);
            }
        }
    }
    rmap_note(addr, s->pc);
    {   /* WZREAD: reads of the MAC kernel input cell (gate CALYPSO_WZWRITE). */
        static int _wr = -1; static unsigned _wrn = 0;
        if (_wr < 0) _wr = calypso_gate("CALYPSO_WZWRITE", 0);
        if (_wr && addr == 0x2c00 && s->pc == 0xa07c && _wrn < 40) {
            /* Only the MAC kernel read matters here; 0x9aba is the
             * normalization loop, noisy and already characterized. */
            _wrn++;
            fprintf(stderr, "[c54x] WZREAD  data[0x%04x] = 0x%04x PC=0x%04x op=0x%04x "
                    "AR2=%04x AR3=%04x AR4=%04x AR5=%04x insn=%u\n",
                    addr, s->data[addr], s->pc, prog_fetch(s, s->pc),
                    s->ar[2], s->ar[3], s->ar[4], s->ar[5], s->insn_count);
        }
    }
    {   /* DMAWATCH (read side): DMA sub-register bank 0x0054..0x0057. */
        static int _dw2 = -1; static unsigned _dwr = 0;
        if (_dw2 < 0) _dw2 = calypso_gate("CALYPSO_DMAWATCH", 0);
        if (_dw2 && addr >= 0x0054 && addr <= 0x0057 && _dwr < 40) {
            _dwr++;
            const char *_nm = (addr==0x0054) ? "DMPREC?(modele:DMSA)" :
                              (addr==0x0055) ? "DMSA?(modele:DMSDI)" :
                              (addr==0x0056) ? "DMSDI?" : "DMSDN";
            fprintf(stderr, "[c54x] DMAWATCH RD 0x%04x %-20s = 0x%04x PC=0x%04x insn=%u\n",
                    addr, _nm, s->data[addr], s->pc, s->insn_count);
        }
    }
    {   /* DEMODRD: the demod sample reads at PC 0x9fb5. */
        static int _dr = -1; static unsigned _drn = 0;
        if (_dr < 0) _dr = calypso_gate("CALYPSO_DEMODRD", 0);
        if (_dr && _drn < 60 && s->pc == 0x9fb5) {   /* sample reads only */
            _drn++;
            fprintf(stderr, "[c54x] DEMODRD PC=0x%04x XPC=%u op=0x%04x lit data[0x%04x]=0x%04x "
                    "AR0=%04x AR1=%04x AR2=%04x AR3=%04x AR4=%04x AR5=%04x AR6=%04x AR7=%04x "
                    "BK=%04x | 5 mots: %04x %04x %04x %04x %04x | insn=%u\n",
                    s->pc, (unsigned)s->xpc, prog_fetch(s, s->pc), addr, s->data[addr],
                    s->ar[0], s->ar[1], s->ar[2], s->ar[3], s->ar[4], s->ar[5],
                    s->ar[6], s->ar[7], s->bk,
                    s->data[(uint16_t)(addr+0)], s->data[(uint16_t)(addr+1)],
                    s->data[(uint16_t)(addr+2)], s->data[(uint16_t)(addr+3)],
                    s->data[(uint16_t)(addr+4)], s->insn_count);
        }
    }
    {   /* SLOTSRC-RD: which address holds the shared-RET stub 0xab38? */
        static int _sr = -1; static unsigned _srn = 0;
        if (_sr < 0) _sr = calypso_gate("CALYPSO_SLOTSRC", 0);
        if (_sr && _srn < 40 && s->data[addr] == 0xab38) {
            _srn++;
            fprintf(stderr, "[c54x] SLOTSRC-RD data[0x%04x] = 0xab38 (STUB) lu PC=0x%04x insn=%u\n",
                    addr, s->pc, s->insn_count);
        }
    }
    flow_log("R", addr, s->data[addr], s->pc, s->insn_count);
    read_stats_record(addr);
    /* MEM-WATCH-2B80 (gate CALYPSO_MEM_WATCH_2B80): the FB correlator at
     * PC=0xee38 reads data[0x2b97] through AR3 (STM hardcoded in the ROM), in
     * [0x2b80,0x2c00) - a region distinct from the BSP DMA buffer
     * [0x2a00,0x2b28). Logs every read in that window (value, PC), cap 200, to
     * tell whether the region is ever populated at all. */
    if (addr >= 0x2b80 && addr < 0x2c00) {
        static int mw2b80_en = -1;
        if (mw2b80_en < 0) mw2b80_en = calypso_gate("CALYPSO_MEM_WATCH_2B80", 0);
        if (mw2b80_en) {
            static unsigned mw2b80_n = 0;
            if (mw2b80_n < 200) {
                mw2b80_n++;
                fprintf(stderr, "[c54x] MEM-WATCH-2B80-RD data[0x%04x]=0x%04x "
                        "PC=0x%04x insn=%u\n", addr, s->data[addr], s->pc,
                        s->insn_count);
            }
        }
    }
    /* Frame-IT probe: the frozen value of the polled flags and who polls them.
     * The value read (never changing) is what the BSP must produce or toggle. */
    if (addr == 0x006e || addr == 0x585f || addr == 0x8a44) {
        static uint32_t fr_n = 0;
        if (fr_n < 24) {
            fprintf(stderr, "[c54x] FLAGRD data[0x%04x]=0x%04x PC=0x%04x A=0x%04x "
                    "TC=%d insn=%u\n", addr, s->data[addr], s->pc,
                    (uint16_t)(s->a & 0xFFFF), !!(s->st0 & ST0_TC), s->insn_count);
            fr_n++;
        }
        /* One-shot: dump the overlay of the 0x010b loop plus the interrupt
         * state (IFR/IMR/INTM) during the spin, which separates a dead source
         * from an IMR-masked one from a stuck INTM. */
        static int dumped_010b = 0;
        if (addr == 0x006e && s->pc == 0x010b && !dumped_010b) {
            dumped_010b = 1;
            fprintf(stderr, "[c54x] SPIN-IT @0x010b IFR=0x%04x IMR=0x%04x INTM=%d "
                    "(INT3 bit3 IFR=%d IMR=%d ; BRINT0 bit5 IFR=%d IMR=%d) insn=%u\n",
                    s->ifr, s->imr, !!(s->st1 & ST1_INTM),
                    !!(s->ifr&(1<<3)), !!(s->imr&(1<<3)),
                    !!(s->ifr&(1<<5)), !!(s->imr&(1<<5)), s->insn_count);
            fprintf(stderr, "[c54x] OVERLAY-DUMP prog[0x0100..0x0118] (loop poll 0x006e):\n");
            for (uint16_t a = 0x0100; a <= 0x0118; a++)
                fprintf(stderr, "[c54x]   prog[0x%04x]=0x%04x\n", a, prog_fetch(s, a));
        }
    }
    /* D_TASK_MD-RD probe : trace DSP reads of d_task_md (write page 0
     * @ data[0x0804], write page 1 @ data[0x0818]). The DSP dispatcher
     * reads task_md then branches to FB / SB / ALLC / etc. routines.
     * If only one PC reads it, that's the single dispatcher. Capped 30. */
    if ((addr == 0x0804 || addr == 0x0818) && calypso_debug_enabled("D_TASK_MD-RD")) {
        /* Prints EA, page, value and the delta since the ARM wrote 5. If the
         * value read after that write (delta>0) is not 5, the 5 does not reach
         * this EA: compare ARM5_EA (written by the ARM) with EA (read by the DSP).
         *  - same EA, value != 5 -> ordering (read before write in the frame)
         *  - EA != ARM5_EA (page stride) -> w_page/r_page flip parity. */
        fprintf(stderr,
                "[c54x] D_TASK_MD-RD EA=0x%04x page=%d val=0x%04x "
                "ARM5_EA=0x%04x dArm5=%lld PC=0x%04x insn=%u\n",
                addr, (addr == 0x0804) ? 0 : 1, s->data[addr],
                g_arm_taskmd5_ea,
                (long long)((int64_t)s->insn_count - (int64_t)g_arm_taskmd5_insn),
                s->pc, s->insn_count);
    }
    /* WATCH-RD-ADDR (generic; CALYPSO_DEBUG=WATCH-RD plus
     * CALYPSO_WATCH_RD_ADDR=0xNNNN): logs every READ of an arbitrary data
     * address with PC, value and insn - who reads a dispatcher-pointer cell and
     * with which value (0 before it is populated vs valid after). Mirror of
     * WATCH-WR. */
    {
        static int watch_rd_addr = -1;
        if (watch_rd_addr < 0) {
            const char *e = calypso_getenv("CALYPSO_WATCH_RD_ADDR");
            watch_rd_addr = (e && *e) ? (int)strtol(e, NULL, 0) : 0;
        }
        if (watch_rd_addr && addr == (uint16_t)watch_rd_addr) {
            C54_DBG("WATCH-RD",
                "WATCH-RD data[0x%04x] = 0x%04x PC=0x%04x DP=0x%03x insn=%u",
                addr, s->data[addr], s->pc, (s->st0 & 0x1FF),
                (unsigned)s->insn_count);
        }
    }
    /* DISP-POLL (CALYPSO_DEBUG=DISP-POLL): the dispatcher busy-loop
     * (d1xx <-> da0d) polls the flag area DARAM[0x60..0x70]. Shows, in steady
     * state, which slot it reads and whether the flag ever goes non-zero.
     * Throttled to 1 in 40000 so the busy-loop cannot flood the log, plus every
     * non-zero value - that is the event that matters. */
    if (addr >= 0x0060 && addr <= 0x0070 && calypso_debug_enabled("DISP-POLL")) {
        static uint64_t poll_n = 0;
        uint16_t v = s->data[addr];
        if (v != 0 || (poll_n++ % 40000) == 0)
            fprintf(stderr,
                "[c54x] DISP-POLL-RD data[0x%04x]=0x%04x PC=0x%04x INTM=%d insn=%u%s\n",
                addr, v, s->pc, !!(s->st1 & ST1_INTM), s->insn_count,
                v ? "  <-- NON-ZERO (flag posé !)" : "");
    }
    /* FBDB-PROBE read 0x3DC0 (= SARAM flag polled by fc63 BITF).
     * Env CALYPSO_FBDB_PROBE=1. Logs first 30 reads + each 10000th. */
    if (addr == 0x3DC0 && g_fbdb_probe_enabled > 0) {
        fbdb_probe_read_3dc0(addr, s->data[addr], s->pc, s->insn_count);
    }
    /* D_BURST_D_W probe: does the DSP read db_w->d_burst_d?
     * 0x0801 (W page 0 + offset 1), 0x0815 (W page 1 + offset 1).
     *   reads see 0,1,2,3 in sequence -> the ARM fills db_w correctly
     *   reads always see 0            -> the ARM never sets burst_id
     *   never read                    -> the DSP does not use db_w for the
     *                                    burst sequence. */
    if (addr == 0x0801 || addr == 0x0815) {
        static uint64_t dbw_total[2];
        static uint64_t dbw_per_val[2][16];
        static uint64_t dbw_last_log[2];
        static uint16_t dbw_last_val[2];
        int page = (addr == 0x0815) ? 1 : 0;
        uint16_t cur_val = s->data[addr] & 0xF;
        dbw_total[page]++;
        if (cur_val < 16) dbw_per_val[page][cur_val]++;
        bool changed = (cur_val != dbw_last_val[page]);
        dbw_last_val[page] = cur_val;
        if (dbw_total[page] <= 100 || changed
            || (s->insn_count - dbw_last_log[page]) > 1000000) {
            fprintf(stderr,
                    "[c54x] D_BURST_D_W-RD page=%d #%llu addr=0x%04x "
                    "val=0x%04x exec_pc=0x%04x insn=%u\n",
                    page, (unsigned long long)dbw_total[page], addr,
                    s->data[addr], s->last_exec_pc, s->insn_count);
            dbw_last_log[page] = s->insn_count;
        }
        /* Summary every 50000 reads: histogram of the values read */
        if ((dbw_total[page] % 50000) == 0) {
            if (calypso_debug_enabled("D_BURST_D_W-SUMMARY")) fprintf(stderr,
                    "[c54x] D_BURST_D_W-SUMMARY page=%d total=%llu "
                    "val[0]=%llu [1]=%llu [2]=%llu [3]=%llu other=%llu\n",
                    page, (unsigned long long)dbw_total[page],
                    (unsigned long long)dbw_per_val[page][0],
                    (unsigned long long)dbw_per_val[page][1],
                    (unsigned long long)dbw_per_val[page][2],
                    (unsigned long long)dbw_per_val[page][3],
                    (unsigned long long)(dbw_total[page]
                        - dbw_per_val[page][0] - dbw_per_val[page][1]
                        - dbw_per_val[page][2] - dbw_per_val[page][3]));
        }
    }
    /* PC histogram to identify the PM routine. Two ranges:
     *   [0x3fb0..0x3fbf] = BSP buffer (I/Q samples)
     *   [0x3dcf..0x3dd5] = dominant scratch buffer (78k + 52k reads measured)
     * Counts per PC and dumps the top 10 every 50k reads in each range, which
     * separates "PM broken" from "PM never called". */
    if (addr >= 0x3fb0 && addr <= 0x3fbf) {
        static uint32_t pc_hist_3fb[65536];
        static uint32_t total_3fb;
        pc_hist_3fb[s->pc]++;
        total_3fb++;
        if ((total_3fb % 50000) == 0) {
            uint32_t top_pc[10] = {0};
            uint32_t top_cnt[10] = {0};
            for (uint32_t p = 0; p < 65536; p++) {
                uint32_t c = pc_hist_3fb[p];
                if (c == 0) continue;
                for (int i = 0; i < 10; i++) {
                    if (c > top_cnt[i]) {
                        for (int j = 9; j > i; j--) {
                            top_pc[j]  = top_pc[j-1];
                            top_cnt[j] = top_cnt[j-1];
                        }
                        top_pc[i]  = p;
                        top_cnt[i] = c;
                        break;
                    }
                }
            }
            if (calypso_debug_enabled("PC-HIST-3FB")) fprintf(stderr, "[c54x] PC-HIST-3FB total=%u :", total_3fb);
            for (int i = 0; i < 10 && top_cnt[i]; i++) {
                fprintf(stderr, " %04x:%u", top_pc[i], top_cnt[i]);
            }
            fprintf(stderr, "\n");
        }
    }
    if (addr >= 0x3dcf && addr <= 0x3dd5) {
        static uint32_t pc_hist_3dd[65536];
        static uint32_t total_3dd;
        pc_hist_3dd[s->pc]++;
        total_3dd++;
        if ((total_3dd % 50000) == 0) {
            uint32_t top_pc[10] = {0};
            uint32_t top_cnt[10] = {0};
            for (uint32_t p = 0; p < 65536; p++) {
                uint32_t c = pc_hist_3dd[p];
                if (c == 0) continue;
                for (int i = 0; i < 10; i++) {
                    if (c > top_cnt[i]) {
                        for (int j = 9; j > i; j--) {
                            top_pc[j]  = top_pc[j-1];
                            top_cnt[j] = top_cnt[j-1];
                        }
                        top_pc[i]  = p;
                        top_cnt[i] = c;
                        break;
                    }
                }
            }
            if (calypso_debug_enabled("PC-HIST-3DD")) fprintf(stderr, "[c54x] PC-HIST-3DD total=%u :", total_3dd);
            for (int i = 0; i < 10 && top_cnt[i]; i++) {
                fprintf(stderr, " %04x:%u", top_pc[i], top_cnt[i]);
            }
            fprintf(stderr, "\n");
        }
    }
    /* CANARY-READ probe: with CALYPSO_BSP_INJECT_CANARY=1 the BSP overwrites
     * every sample with 0xCAFE. Any address from which the DSP reads 0xCAFE is
     * on its sample-read path, i.e. the right target for CALYPSO_BSP_DARAM_ADDR.
     * Capped at 100 lines. */
    if (addr < 0x4000) {
        uint16_t v = s->data[addr];
        if (v == 0xCAFE) {
            static unsigned canary_log;
            const unsigned LIMIT = 100;
            if (canary_log < LIMIT) {
                fprintf(stderr,
                        "[c54x] CANARY-READ #%u addr=0x%04x PC=0x%04x "
                        "AR2=%04x AR3=%04x AR4=%04x AR5=%04x insn=%u\n",
                        canary_log, addr, s->pc,
                        s->ar[2], s->ar[3], s->ar[4], s->ar[5],
                        s->insn_count);
                canary_log++;
                if (canary_log == LIMIT)
                    fprintf(stderr, "[c54x] CANARY-READ capped at %u\n", LIMIT);
            }
        }
    }

    /* Watch the mailbox slots that the firmware polls at PROM0 0xb41a
     * (LDU *(0x0ffe), A then BACC A) and 0xb41c (CMPM *(0x0fff), 4).
     * If these stay zero / 0x10 forever, ARM never wrote them. */
    if (addr == 0x0ffe || addr == 0x0fff || addr == 0x0ffc || addr == 0x0ffd) {
        static unsigned watch_count;
        static uint16_t last_vd[4] = { 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF };
        int widx = (addr == 0x0fff) ? 0 : (addr == 0x0ffe) ? 1 : (addr == 0x0ffd) ? 2 : 3;
        uint16_t vd = s->data[addr];
        watch_count++;
        /* Log on change: first 60 lines plus every value CHANGE. The bootloader
         * spin at 0xb424 reads data[0x0fff]=0x0001 1.36 billion times, so a
         * 1-in-10000 sample emitted 135k lines (20 MB), stalled QEMU and lost
         * osmocon. This keeps the exact signal - the IDLE 0x0001 -> command
         * 0x0002/0x0004 transition - without the noise. */
        if (watch_count <= 60 || vd != last_vd[widx]) {
            uint16_t va = s->api_ram ? s->api_ram[addr - C54X_API_BASE] : 0xDEAD;
            fprintf(stderr,
                    "[c54x] WATCH-READ #%u data[0x%04x] data=0x%04x api_ram=0x%04x api_set=%d PC=0x%04x insn=%u\n",
                    watch_count, addr, vd, va, s->api_ram ? 1 : 0, s->pc, s->insn_count);
        }
        last_vd[widx] = vd;
    }
    /* Wait-loop diagnostic: 0x3dd0 was found to absorb ~99.5 % of DARAM
     * reads after the first ~500k reads - the DSP is stuck polling it.
     * Log the first PCs and then sample once per million reads so we can
     * trace the loop without flooding the log. */
    if (addr == 0x3dd0) {
        static unsigned wait_log;
        static unsigned wait_seen;
        wait_seen++;
        if (wait_log < 20 || (wait_seen % 1000000) == 0) {
            wait_log++;
            fprintf(stderr,
                    "[c54x] WAIT-3DD0 #%u data[0x3dd0]=0x%04x PC=0x%04x AR2=%04x AR3=%04x insn=%u\n",
                    wait_seen, s->data[0x3dd0], s->pc,
                    s->ar[2], s->ar[3], s->insn_count);
        }
    }
    /* d_fb_det watch. The DSP word address is 0x08F8:
     * ARM 0xFFD001F0 (BASE_API_NDB 0xFFD001A8 + 36 words * 2)
     *   = DSP word 0x0800 + 0x1F0/2 = 0x08F8.
     * 0x01F0 is the ARM byte offset, not a DSP word address. */
    if (addr == 0x08F8) {
        static unsigned fb_read;
        if (fb_read++ < 30) {
            fprintf(stderr,
                    "[c54x] WATCH-READ d_fb_det[0x08F8]=0x%04x PC=0x%04x insn=%u\n",
                    s->data[0x08F8], s->pc, s->insn_count);
        }
    }
    /* BOOT-POLL-RD: traces data reads in the boot polling loop (DARAM
     * 0x00ed..0x00ff = OVLY mirror of PROM0[0x70ed..0x70ff]; DSP_ROM_MAP puts
     * the boot polling loop at 0x7026-0x71FF, where it writes the API RAM
     * tables). Identifies which address the firmware polls to decide to leave,
     * hence which signal the modelled peripherals must emit. Cap 300. */
    if (s->pc >= 0x00ed && s->pc <= 0x010f) {
        static unsigned bpr_log;
        const unsigned LIMIT = 300;
        if (bpr_log < LIMIT) {
            uint16_t v;
            const char *region;
            if (addr >= C54X_API_BASE && addr < C54X_API_BASE + C54X_API_SIZE) {
                v = s->api_ram ? s->api_ram[addr - C54X_API_BASE] : 0;
                region = "api";
            } else if (addr < 0x4000) {
                v = s->data[addr];
                region = "daram";
            } else {
                v = s->data[addr];
                region = "mmr/oth";
            }
            fprintf(stderr,
                    "[c54x] BOOT-POLL-RD #%u PC=0x%04x op=0x%04x [%s 0x%04x]=0x%04x "
                    "AR1=%04x AR2=%04x AR3=%04x SP=%04x insn=%u\n",
                    bpr_log, s->pc, s->prog[s->pc], region, addr, v,
                    s->ar[1], s->ar[2], s->ar[3], s->sp, s->insn_count);
            bpr_log++;
            if (bpr_log == LIMIT) {
                fprintf(stderr, "[c54x] BOOT-POLL-RD log capped at %u\n", LIMIT);
            }
        }
    }

    /* CORR-RD: traces data reads in the FB-det correlator inner body
     * (PROM0[0x9aba..0x9abf] = RPTBD body of the publish routine at 0x9aaf+).
     * addr in [0x3fb0..0x3fbf] -> right zone (BSP RX), the value is a sample.
     * addr anywhere else -> addressing bug: the correlator reads an empty
     * buffer, A stays 0, it publishes 0 and never locks. Cap 200. */
    if (s->pc >= 0x9aba && s->pc <= 0x9abf) {
        static unsigned cr_log;
        const unsigned LIMIT = 200;
        if (cr_log < LIMIT) {
            uint16_t v;
            const char *region;
            if (addr >= C54X_API_BASE && addr < C54X_API_BASE + C54X_API_SIZE) {
                v = s->api_ram ? s->api_ram[addr - C54X_API_BASE] : 0;
                region = "api";
            } else if (addr < 0x4000) {
                v = s->data[addr];
                region = "daram";
            } else {
                v = s->data[addr];
                region = "mmr/oth";
            }
            fprintf(stderr,
                    "[c54x] CORR-RD #%u PC=0x%04x [%s 0x%04x]=0x%04x "
                    "AR2=%04x AR3=%04x AR4=%04x AR5=%04x A_lo=%04x B_lo=%04x insn=%u\n",
                    cr_log, s->pc, region, addr, v,
                    s->ar[2], s->ar[3], s->ar[4], s->ar[5],
                    (uint16_t)(s->a & 0xFFFF),
                    (uint16_t)(s->b & 0xFFFF),
                    s->insn_count);
            cr_log++;
            if (cr_log == LIMIT) {
                fprintf(stderr, "[c54x] CORR-RD log capped at %u\n", LIMIT);
            }
        }
    }
    /* IDLE-DISP: the DSP gets stuck in the PROM0 loop at 0xCC62..0xCC6F polling
     * task slots. Dumps (PC, addr, value, AR2..AR5) for the first N reads to see
     * which location the dispatcher inspects before deciding to branch out.
     * Captures DARAM, API RAM and MMR reads so the poll address cannot be
     * missed. Capped. */
    if (s->pc >= 0xCC62 && s->pc <= 0xCC6F) {
        static unsigned idle_rd_log;
        const unsigned LIMIT = 200;
        if (idle_rd_log < LIMIT) {
            uint16_t v;
            const char *region;
            if (addr >= C54X_API_BASE && addr < C54X_API_BASE + C54X_API_SIZE) {
                v = s->api_ram ? s->api_ram[addr - C54X_API_BASE] : 0;
                region = "api";
            } else if (addr < 0x4000) {
                v = s->data[addr];
                region = "daram";
            } else {
                v = s->data[addr];
                region = "mmr/other";
            }
            fprintf(stderr,
                    "[c54x] IDLE-DISP RD #%u PC=0x%04x [%s 0x%04x]=0x%04x "
                    "AR2=%04x AR3=%04x AR4=%04x AR5=%04x insn=%u\n",
                    idle_rd_log, s->pc, region, addr, v,
                    s->ar[2], s->ar[3], s->ar[4], s->ar[5], s->insn_count);
            idle_rd_log++;
            if (idle_rd_log == LIMIT) {
                fprintf(stderr,
                        "[c54x] IDLE-DISP RD log capped at %u — pattern should be visible above\n",
                        LIMIT);
            }
        }
    }

    /* UPPER-DARAM RD HIST: histogram of reads in [0x4000..0xFFFF], the
     * companion of the low-DARAM histogram below. Shows which upper addresses
     * the DSP polls for its samples, hence the right target for
     * CALYPSO_BSP_DARAM_ADDR. Top 16 every 100k reads; skips the first 1M insn
     * to drop the boot noise. */
    if (addr >= 0x4000 && s->insn_count > 1000000) {
        static unsigned uhist[0xC000]; /* 0x4000..0xFFFF = 48K words */
        static unsigned ureads;
        uhist[addr - 0x4000]++;
        ureads++;
        if ((ureads % 100000) == 0) {
            unsigned best[16] = {0}; uint16_t baddr[16] = {0};
            for (unsigned a = 0; a < 0xC000; a++) {
                unsigned c = uhist[a];
                if (c <= best[15]) continue;
                int p = 15;
                while (p > 0 && best[p-1] < c) {
                    best[p] = best[p-1]; baddr[p] = baddr[p-1]; p--;
                }
                best[p] = c; baddr[p] = (uint16_t)(0x4000 + a);
            }
            if (calypso_debug_enabled("UPPER-DARAM")) fprintf(stderr,
                    "[c54x] UPPER-DARAM RD HIST (reads=%u): ", ureads);
            for (int i = 0; i < 16 && best[i]; i++)
                fprintf(stderr, "%04x:%u ", baddr[i], best[i]);
            fprintf(stderr, "\n");
        }
    }
    /* === DARAM discovery histogram ===
     * Track ALL data reads from DARAM (addr < 0x4000) regardless of PC.
     * The FB handler runs from both PROM0 (0xBD47) and DARAM overlay,
     * so filtering by PC misses critical reads. */
    if (addr < 0x4000 && addr >= 0x20) {  /* skip MMRs 0x00-0x1F */
        static unsigned hist[0x4000]; /* 16 KW DARAM */
        static unsigned reads;
        if (addr < 0x4000) {
            hist[addr]++;
            reads++;
            if ((reads % 50000) == 0) {
                /* find top-16 */
                unsigned best[16] = {0}; uint16_t baddr[16] = {0};
                for (uint16_t a = 0; a < 0x4000; a++) {
                    unsigned c = hist[a];
                    if (c <= best[15]) continue;
                    int p = 15;
                    while (p > 0 && best[p-1] < c) {
                        best[p] = best[p-1]; baddr[p] = baddr[p-1]; p--;
                    }
                    best[p] = c; baddr[p] = a;
                }
                if (calypso_debug_enabled("DARAM")) fprintf(stderr,
                        "[c54x] DARAM RD HIST (FB-det, reads=%u): ",
                        reads);
                for (int i = 0; i < 16 && best[i]; i++)
                    fprintf(stderr, "%04x:%u ", baddr[i], best[i]);
                fprintf(stderr, "\n");
            }
        }
    }
    /* FB-det / dispatcher subroutine trace. The 0x7e80..0x7eb8 wrapper calls
     * into 0x81a5/0x81c8 with AR5=0x0e4c (the FB sample buffer); both ranges are
     * covered so wrapper polls and inner correlator reads are caught. Skips the
     * boot init phase. */
    if ((s->pc >= 0x7e80 && s->pc <= 0x7ec0) ||
        (s->pc >= 0x81a0 && s->pc <= 0x82ff)) {
        static int fbdet_rd_log = 0;
        if (s->insn_count > 50000000 && fbdet_rd_log < 2000) {
            uint16_t v;
            if (addr >= C54X_API_BASE && addr < C54X_API_BASE + C54X_API_SIZE)
                v = s->api_ram ? s->api_ram[addr - C54X_API_BASE] : 0;
            else
                v = s->data[addr];
            C54_LOG("FBDET RD [0x%04x]=0x%04x PC=0x%04x AR2=%04x AR3=%04x AR4=%04x AR5=%04x insn=%u",
                    addr, v, s->pc, s->ar[2], s->ar[3], s->ar[4], s->ar[5], s->insn_count);
            fbdet_rd_log++;
        }
    }
    /* Log AR0..AR7 when entering FB-det subroutines to understand
     * what each AR points at (sample buffer? coeffs? status?). */
    if ((s->pc == 0x81a5 || s->pc == 0x81c8) && s->insn_count > 50000000) {
        static int ar_log = 0;
        if (ar_log < 10) {
            C54_LOG("FB-CALL PC=0x%04x AR0=%04x AR1=%04x AR2=%04x AR3=%04x "
                    "AR4=%04x AR5=%04x AR6=%04x AR7=%04x SP=%04x BK=%04x",
                    s->pc, s->ar[0], s->ar[1], s->ar[2], s->ar[3],
                    s->ar[4], s->ar[5], s->ar[6], s->ar[7], s->sp, s->bk);
            ar_log++;
        }
    }
    /* d_spcx_rif (NDB word 2 = api 0xD6 = DSP data 0x08D6) */
    if (addr == 0x08D6) {
        static int spcx_rd = 0;
        if (spcx_rd < 32) {
            C54_LOG("d_spcx_rif RD = 0x%04x PC=0x%04x insn=%u",
                    s->api_ram ? s->api_ram[0xD6] : s->data[addr],
                    s->pc, s->insn_count);
            spcx_rd++;
        }
    }
    /* Log reads from API RAM at 0x08D4 (d_dsp_page). */
    if (addr == 0x08D4) {
        static int dsp_page_log = 0;
        if (dsp_page_log < 50) {
            C54_LOG("d_dsp_page RD = 0x%04x PC=0x%04x insn=%u SP=0x%04x",
                    s->api_ram ? s->api_ram[addr - 0x0800] : s->data[addr],
                    s->pc, s->insn_count, s->sp);
            dsp_page_log++;
        }
        /* FBWATCH producer side: does the DSP re-read d_dsp_page every frame? */
        if (g_fbwatch_on < 0) g_fbwatch_on = calypso_gate("CALYPSO_FBWATCH", 0);
        if (g_fbwatch_on) {
            static unsigned wpg = 0;
            if (wpg++ < 60)
                fprintf(stderr, "[c54x] FBWATCH-PAGE-RD #%u val=0x%04x PC=0x%04x insn=%u\n",
                        wpg, s->api_ram ? s->api_ram[addr - 0x0800] : s->data[addr],
                        s->pc, s->insn_count);
        }
    }
    /* Timer registers (0x0024-0x0026) - read returns current value */
    if (addr == TIM_ADDR) return s->data[TIM_ADDR];
    if (addr == PRD_ADDR) return s->data[PRD_ADDR];
    if (addr == TCR_ADDR) {
        /* TCR: PSC is read from bits 9:6, rest from stored value */
        uint16_t tcr = s->data[TCR_ADDR] & ~TCR_PSC_MASK;
        tcr |= (s->timer_psc & 0xF) << TCR_PSC_SHIFT;
        return tcr;
    }

    /* MMR region */
    if (addr < 0x20) {
        switch (addr) {
        case MMR_IMR:  return s->imr;
        case MMR_IFR:
        {
            static int ifr_log = 0;
            if ((s->ifr & 0x0020) && ifr_log < 10) {
                /* bit 5 = BRINT0 per C54X header (vec 21). */
                C54_LOG("IFR READ=0x%04x (BRINT0 pending) PC=0x%04x", s->ifr, s->pc);
                ifr_log++;
            }
            return s->ifr;
        }
        case MMR_ST0:  return s->st0;
        case MMR_ST1:  return s->st1;
        case MMR_AL:   return (uint16_t)(s->a & 0xFFFF);
        case MMR_AH:   return (uint16_t)((s->a >> 16) & 0xFFFF);
        case MMR_AG:   return (uint16_t)((s->a >> 32) & 0xFF);
        case MMR_BL:   return (uint16_t)(s->b & 0xFFFF);
        case MMR_BH:   return (uint16_t)((s->b >> 16) & 0xFFFF);
        case MMR_BG:   return (uint16_t)((s->b >> 32) & 0xFF);
        case MMR_T:    return s->t;
        case MMR_TRN:  return s->trn;
        case MMR_AR0: case MMR_AR1: case MMR_AR2: case MMR_AR3:
        case MMR_AR4: case MMR_AR5: case MMR_AR6: case MMR_AR7:
            return s->ar[addr - MMR_AR0];
        case MMR_SP:   return s->sp;
        case MMR_BK:   return s->bk;
        case MMR_BRC:  return s->brc;
        case MMR_RSA:  return s->rsa;
        case MMR_REA:  return s->rea;
        case MMR_PMST: return s->pmst;
        case MMR_XPC:  return s->xpc;
        default: return 0;
        }
    }

    /* API RAM (shared with ARM) */
    if (addr >= C54X_API_BASE && addr < C54X_API_BASE + C54X_API_SIZE) {
        if (s->api_ram) {
            uint16_t val = s->api_ram[addr - C54X_API_BASE];
            /* Log ALL API reads during interrupt handler (first 100) */
            static int api_rd_log = 0;
            if (api_rd_log < 100 && s->insn_count > 66000) {
                C54_LOG("API RD [0x%04x] = 0x%04x PC=0x%04x insn=%u",
                        addr, val, s->pc, s->insn_count);
                api_rd_log++;
            }
            return val;
        }
    }

    /* Log data reads during SINT17 handler (PC in 0xFFC0-0xFFFF) */
    if (s->pc >= 0xFFC0 && s->insn_count > 66090) {
        static int handler_rd_log = 0;
        if (handler_rd_log < 30) {
            C54_LOG("H_RD [0x%04x]=0x%04x PC=0x%04x", addr, s->data[addr], s->pc);
            handler_rd_log++;
        }
    }

    /* PROBE-3FAD-GATE (gate CALYPSO_PROBE_3FAD_GATE, cap 200, no cost when OFF):
     * captures the exact value read by BITF *(0x3fad),#0x8000 at 0x8753, the only
     * lock on the FB dispatcher (CC 0xa0a0 -> kernel 0xa076). Nothing else blocks
     * the sweep, so bit15 of 0x3fad alone decides.
     *   val & 0x8000      -> TC=1, the kernel runs.
     *   val & 0x8000 == 0 while RX-FBFLAGS set it -> a per-frame clearer
     *   (76f8 3fad 0000, XPC=0 at 0xace8/0xad04/0xad24 or XPC=2 overlay at
     *   0x28040/0x282d0) wiped it between the BSP write and the DSP sweep. */
    if (addr == 0x3fad && s->pc == 0x8753 && calypso_rxfb_fired) {  /* gated on RX-FBFLAGS having actually set 3fad bit15 */
        static int p3_en = -1; static unsigned p3_n = 0;
        if (p3_en < 0) p3_en = calypso_gate("CALYPSO_PROBE_3FAD_GATE", 0);
        if (p3_en && p3_n < 200) {
            p3_n++;
            fprintf(stderr, "[c54x] PROBE-3FAD-GATE @0x8753 val=0x%04x bit15=%d "
                    "(=>TC/kernel) task_md0=0x%04x xpc=%d insn=%u\n",
                    s->data[0x3fad], !!(s->data[0x3fad] & 0x8000),
                    s->data[0x0804], s->xpc, s->insn_count);
        }
        /* @BEQUILLE - FORCE_3FAD_KERNEL  (CALYPSO_FORCE_3FAD_KERNEL, default OFF)
         *   masque  : the producer of the "burst ready" flag data[0x3fad] bit15.
         *             A per-frame clearer (76f8 3fad 0000) wipes it between the
         *             BSP write and the DSP sweep; this re-sets it on the read
         *             path at 0x8753 so the BITF sees TC=1 -> CC 0xa0a0 ->
         *             kernel 0xa076.
         *   retirer : once the native setter (RX/BRINT0 chain) holds bit15 until
         *             the BITF at 0x8753, i.e. when PROBE_3FAD_GATE sees bit15=1
         *             without this gate.
         *   NB      : conditioned on calypso_rxfb_fired, set by CALYPSO_RX_FBFLAGS.
         */
        { static int _fk = -1; if (_fk < 0) _fk = calypso_gate("CALYPSO_FORCE_3FAD_KERNEL", 0);
          if (_fk && calypso_rxfb_fired) s->data[0x3fad] |= 0x8000; }
    }

    return s->data[addr];
}

static void data_write_locked(C54xState *s, uint16_t addr, uint16_t val);

/* Stack-write ring (ORPHAN trace): records data writes into the stack area so
 * that, at the POPM ST0 at 0xf48b, the writer of the popped slot can be named
 * (clobber below SP, circular / dual-operand family). No writer at dump time
 * means nobody wrote the slot in this frame: the slot is stale and SP is
 * misaligned (a POP with no matching PUSH). */
StkwEv   g_stkw_ring[STKW_RING_N];
unsigned g_stkw_idx  = 0;
int      g_orphan_on = -1;
static void stkw_rec(C54xState *s, uint16_t addr, uint16_t val)
{
    if (g_orphan_on < 0) g_orphan_on = calypso_gate("CALYPSO_ORPHAN", 0);  /* own env, outside CALYPSO_DEBUG: the master gate stays 0 */
    if (!g_orphan_on) return;
    if (addr < 0x1000 || addr > 0x6000) return;   /* stack area (SP drifts 0x1100 -> 0x56xx) */
    StkwEv *e = &g_stkw_ring[g_stkw_idx % STKW_RING_N];
    e->addr = addr; e->val = val; e->pc = s->pc; e->op = prog_fetch(s, s->pc);
    g_stkw_idx++;
    /* Track-value: names the CALL/push that lays down the orphan value (default
     * 0x3125, override with CALYPSO_TRACK_STKVAL=0xNNNN), to be correlated with
     * RCD at 0x765c and POPM at 0xf48b in insn order. */
    {
        static int tval = -2;
        if (tval == -2) {
            const char *te = calypso_getenv("CALYPSO_TRACK_STKVAL");
            tval = (te && *te) ? (int)strtol(te, NULL, 0) : 0x3125;
        }
        if (tval >= 0 && val == (uint16_t)tval)
            fprintf(stderr, "[c54x] STK-PUSH val=0x%04x addr=0x%04x PC=0x%04x op=0x%04x SP=0x%04x insn=%u\n",
                    val, addr, s->pc, prog_fetch(s, s->pc), s->sp, s->insn_count);
    }
}

/* SCRATCH-WR (CALYPSO_SCRATCH_WR, default OFF, read-only) - is the FIRS
 * coefficient table ever written?
 * Measured: prog[0x61..0x66]=0xF4E4 (filler) and data[0x61..0x66]=0 at execution
 * time. Program space 0x0000-0x6FFF is covered by no ROM file (PROM0 starts at
 * 0x7000), and that is exactly where pmad=0x61 falls.
 *   no write   -> a ROM segment is missing
 *   late write -> an ordering problem
 * Every write in BOTH spaces is stamped with insn and PC, with a periodic
 * summary so that "nothing written" cannot be read as "probe silent". */
static int scratchwr_on(void)
{
    static int g = -1;
    if (g < 0) {
        g = calypso_gate("CALYPSO_SCRATCH_WR", 0);
        fprintf(stderr, "[c54x] SCRATCH-WR %s : ecritures dans 0x0060-0x007F "
                "(scratch-pad DARAM + espace PROGRAMME via MVDP) — zone des "
                "coefficients du FIRS (pmad=0x0061)\n",
                g ? "ACTIVE" : "INACTIVE (defaut)");
    }
    return g;
}

void scratchwr_note(C54xState *s, uint16_t a, uint16_t v, const char *espace)
{
    if (!scratchwr_on()) return;
    /* The cap is PER ADDRESS, never global: a global 40-line cap is consumed by
     * data[0x74] around insn 5000-10000, so the coefficient writes (MVDD at insn
     * ~1376674) never print and the cap reads as an absence. */
    int zone = -1;
    if (a >= 0x0060 && a <= 0x007F)      zone = 0;   /* destination */
    else if (a >= 0x2CB0 && a <= 0x2CBF) zone = 1;   /* table SOURCE */
    if (zone < 0) return;

    static unsigned long long cum[2] = {0, 0};
    static unsigned long long cum_coef = 0;   /* 0x0061..0x0066 only */
    static unsigned long long cum_src  = 0;   /* 0x2CB9..0x2CBF only */
    static unsigned char shown[2][0x20];

    cum[zone]++;
    if (a >= 0x0061 && a <= 0x0066) cum_coef++;
    if (a >= 0x2CB9 && a <= 0x2CBF) cum_src++;

    unsigned idx = (zone == 0) ? (unsigned)(a - 0x0060) : (unsigned)(a - 0x2CB0);
    if (idx < 0x20 && shown[zone][idx] < 3) {
        shown[zone][idx]++;
        fprintf(stderr, "[c54x] SCRATCH-WR %s[0x%04x] <- 0x%04x PC=0x%04x insn=%u%s\n",
                espace, a, v, s->pc, s->insn_count,
                (a >= 0x0061 && a <= 0x0066) ? "  <<<< COEFFICIENTS (destination)" :
                (a >= 0x2CB9 && a <= 0x2CBF) ? "  <<<< SOURCE de la table" : "");
    }
    if (((cum[0] + cum[1]) % 500) == 0)
        fprintf(stderr, "[c54x] SCRATCH-WR bilan : scratch(0x60-0x7F)=%llu "
                "source(0x2CB0-0x2CBF)=%llu | dont coefficients(0x61-0x66)=%llu "
                "source utile(0x2CB9-0x2CBF)=%llu\n",
                (unsigned long long)cum[0], (unsigned long long)cum[1],
                (unsigned long long)cum_coef, (unsigned long long)cum_src);
}

/* provenance du dernier TOA ecrit par la ROM (voir la sonde sur 0x08fa) */
int g_toa_grille = -1, g_toa_valeur = 0;
unsigned g_toa_seq = 0;

void data_write(C54xState *s, uint16_t addr, uint16_t val)
{
    if (c54x_rapide) {                     /* see calypso_c54x.h */
        /* the one functional substitution of the slow path: a_pm from the
         * measured downlink magnitude (CALYPSO_PM_RSSI, default 1) */
        static int rssi_on = -1;
        if (rssi_on < 0) rssi_on = calypso_gate("CALYPSO_PM_RSSI", 1);
        if (rssi_on && ((addr >= 0x0834 && addr <= 0x0836) || (addr >= 0x0848 && addr <= 0x084A)))
            val = calypso_bsp_rssi_apm();
        data_write_locked(s, addr, val);
        return;
    }
    scratchwr_note(s, addr, val, "data");
    {   /* HANDLER-WATCH - CALYPSO_DISPATCH_PROBE=1, read-only.
         *
         * data[0x43d8] is the CURRENT handler slot; the dispatcher does
         *     0xb01c  ld   *(0x43d8), A
         *     0xb01e  cala A
         * and [2026-08-03] it always calls 0xab38, the shared RET of empty slots.
         * The whole RX chain therefore reduces to who writes this cell, and why
         * never 0xa5cd.
         *
         * Watching the WRITE rather than a PC is deliberate: a PC watch is easy
         * to aim at the operand instead of the instruction, and watching the cell
         * also catches writers that have not been identified yet. */
        static int _hw = -1;
        if (_hw < 0) _hw = calypso_gate("CALYPSO_DISPATCH_PROBE", 0);
        if (_hw && addr == 0x43d8) {
            static unsigned long long _n = 0;
            uint16_t old = s->data[addr];
            if (old != val || _n < 8) {
                _n++;
                fprintf(stderr, "[dispatch] *** data[0x43d8] : 0x%04x -> 0x%04x "
                        "par PC=0x%04x%s  (data[0x43b0]=0x%04x) insn=%u\n",
                        old, val, s->pc,
                        (val == 0xa5cd) ? "  <<< ARMEMENT RX INSTALLE !" :
                        (val == 0xab38) ? "  (RET partage = slot vide)" : "",
                        s->data[0x43b0], s->insn_count);
            }
        }
    }
    /* Mailbox monitor (calypso_mailbox.h). Must stay FIRST: s->data[addr] still
     * holds the old value here. Costs one inline int test when off, and
     * data_write is a hot path. */
    calypso_mbx(MBX_DSP_WR, addr, val, s->data[addr], s->pc, 0, s->insn_count);

    /* WATCH-ACD (gate CALYPSO_WATCH_ACD): does the DSP write a_cd by opcode?
     * A_CD-WR (watch_write_zone_check, 0x09d2..0x09e0) already covers the zone
     * ungated and logs the first 500 writes; it is the reference, WATCH-ACD only
     * a targeted duplicate. */
    if (addr >= 0x09D0 && addr <= 0x09DE) {
        static int _wac = -1;
        if (_wac < 0) _wac = calypso_gate("CALYPSO_WATCH_ACD", 0);
        if (_wac) { static unsigned _nac = 0;
            if (_nac++ < 60)
                fprintf(stderr, "[c54x] WATCH-ACD DSP-opcode-write data[0x%04x]=0x%04x (was 0x%04x) PC=0x%04x insn=%u\n",
                        addr, val, s->data[addr], s->pc, s->insn_count); }
    }
    /* WATCH-2A00 (gate CALYPSO_WATCH_2A00): traces every OPCODE write to the IQ
     * buffer 0x2a00..0x2a07. feed_iq writes s->data[] directly, bypassing
     * data_write, so anything seen here is a DSP opcode writer; s=%p lets the
     * pointer be compared with feed_iq's. */
    if (addr >= 0x2a00 && addr <= 0x2a07) {
        static int _w2a = -1;
        if (_w2a < 0) _w2a = calypso_gate("CALYPSO_WATCH_2A00", 0);
        if (_w2a) {
            static unsigned _n2a = 0;
            if (_n2a++ < 80)
                fprintf(stderr, "[c54x] WATCH-2A00 opcode-write data[0x%04x]=0x%04x "
                        "(was 0x%04x) PC=0x%04x s=%p insn=%u\n",
                        addr, val, s->data[addr], s->pc, (void*)s, s->insn_count);
        }
    }
    /* WATCH-9200 (gate CALYPSO_WATCH_9200): the demod (0x9fab-0x9fb5) reads
     * 0x9210-0x9218 / 0x9260-0x9261 as its IQ source, but they stay constant
     * (0xff06/0x04a3), leaving workzone 0x2a00 flat. Traces every opcode write
     * to the region to find whether anything feeds it per frame, and from where. */
    if ((addr >= 0x9210 && addr <= 0x9220) || (addr >= 0x9260 && addr <= 0x9262)) {
        static int _w92 = -1;
        if (_w92 < 0) _w92 = calypso_gate("CALYPSO_WATCH_9200", 0);
        if (_w92) {
            static unsigned _n92 = 0;
            if (_n92++ < 80)
                fprintf(stderr, "[c54x] WATCH-9200 opcode-write data[0x%04x]=0x%04x (was 0x%04x) PC=0x%04x insn=%u\n",
                        addr, val, s->data[addr], s->pc, s->insn_count);
        }
    }
    /* WATCH-RESULT (gate CALYPSO_WATCH_RESULT): traces OPCODE writes to the
     * native FB result cells, confirmed against the osmocom NDB layout -
     * d_fb_det 0x08F8, a_sync_demod TOA/PM/ANGLE/SNR 0x08FA..0x08FD. Shows what
     * the native correlator produces even while d_fb_det stays 0. */
    if (addr >= 0x08F8 && addr <= 0x08FD) {
        static int _wr = -1;
        if (_wr < 0) _wr = calypso_gate("CALYPSO_WATCH_RESULT", 0);
        if (_wr) {
            static unsigned _nr = 0;
            const char *_nm = (addr==0x08F8)?"d_fb_det":(addr==0x08F9)?"d_fb_mode":
                              (addr==0x08FA)?"TOA":(addr==0x08FB)?"PM":
                              (addr==0x08FC)?"ANGLE":"SNR";
            if (_nr++ < 120)
                fprintf(stderr, "[c54x] WATCH-RESULT data[0x%04x]=%-8s 0x%04x (was 0x%04x) PC=0x%04x insn=%u\n",
                        addr, _nm, val, s->data[addr], s->pc, s->insn_count);
        }
    }
    /* WATCH-0810 (gate CALYPSO_WATCH_0810): traces every DSP-side write to
     * data[0x0810] (d_ctrl_system, B_TASK_ABORT is bit15) with the writing PC.
     * Companion of ARM-WRITE-0810 on the trx side: together they say who re-sets
     * bit15 after the ARM clears it. The CTRLSYS wire writes s->data[] directly,
     * bypassing data_write, so its absence here identifies it as the re-setter.
     * Cap 200. */
    if (addr == 0x0810) {
        static int w810 = -1;
        if (w810 < 0) w810 = calypso_gate("CALYPSO_WATCH_0810", 0);
        if (w810) {
            static unsigned n810 = 0;
            if (n810++ < 200)
                fprintf(stderr, "[c54x] WATCH-0810 DSP-write data[0x0810]=0x%04x "
                        "(was 0x%04x) PC=0x%04x insn=%u\n",
                        val, s->data[0x0810], s->pc, s->insn_count);
        }
    }
    /* WRITE-WATCH (read-only, cap 80): who writes 0x434f (FIFO write pointer),
     * 0x434e (FIFO read pointer) and 0x3f6d (go-live soft vector), and from which
     * PC. Separates an active write from a leftover reset value: whether the FIFO
     * is really initialized, and whether the soft vector is actively pointed at
     * 0xa4df. */
    if (addr == 0x434f || addr == 0x434e || addr == 0x3f6d) {
        static unsigned ww = 0;
        if (ww++ < 80)
            fprintf(stderr, "[c54x] WRITE-WATCH data[0x%04x] <- 0x%04x PC=0x%04x insn=%u\n",
                    addr, val, s->pc, s->insn_count);
    }
    /* F70-SETBIT1 (read-only, cap 40): bit1 of data[0x3f70] is the flag that
     * leaves the wait loop (tested at 0xa4d4, RET if TC). Logs only when bit1
     * (0x0002) is written; never firing means the DSP never raises "frame ready".
     * Known writers of 0x0002: 0x710c, 0xa5bd, 0xb3ef, 0xde9c. */
    if (addr == 0x3f70 && (val & 0x0002)) {
        static unsigned f70 = 0;
        if (f70++ < 40)
            fprintf(stderr, "[c54x] F70-SETBIT1 data[0x3f70] 0x%04x->0x%04x PC=0x%04x insn=%u\n",
                    s->data[0x3f70], val, s->pc, s->insn_count);
    }
    /* VEC-INSTALL (read-only, cap 200): the runtime vector table lives in DARAM
     * at 0x0080 (IPTR=1, OVLY). Measured cold: vec19 (FRAME) at 0xcc and vec21
     * (BRINT0) at 0xd4 are RETE (0xf4eb) / NOP stubs, while vec17/20/22/24..
     * carry real handlers (FB 0xf8xx). Watches the 8 words of those two slots
     * plus any word 0 of the table [0x80..0xFC] receiving an FB branch (0xf8xx),
     * to say whether the firmware ever installs a real branch there, and when. */
    if ((addr >= 0x00cc && addr <= 0x00cf) || (addr >= 0x00d4 && addr <= 0x00d7) ||
        ((addr >= 0x0080 && addr <= 0x00ff) && ((addr & 3) == 0) &&
         (val & 0xFF80) == 0xF880)) {
        static unsigned vi = 0;
        if (vi++ < 200) {
            int vec = (addr - 0x0080) / 4;
            int word = (int)((addr - 0x0080) & 3);
            fprintf(stderr, "[c54x] VEC-INSTALL vec%d@0x%04x w%d <- 0x%04x %s PC=0x%04x insn=%u\n",
                    vec, addr, word, val,
                    (vec==19?"<FRAME":(vec==21?"<BRINT0":"")),
                    s->pc, s->insn_count);
        }
    }
    /* TASKTAB-WR (read-only): who seeds data[0x4c5b/0x4c5c], the task table read
     * by LD *(0x4c5c),A followed by CALA, expected to come from the FRAME handler
     * at 0xA04C. No write means never seeded: CALA with A=0 lands in the boot
     * stub. */
    if (addr >= 0x4c5b && addr <= 0x4c5d) {
        static unsigned ttab_n = 0;
        if (ttab_n++ < 80)
            fprintf(stderr, "[c54x] TASKTAB-WR data[0x%04x] <- 0x%04x PC=0x%04x insn=%u\n",
                    addr, val, s->pc, s->insn_count);
    }
    /* ARM<->DSP coherence sentinel (CALYPSO_FBDET_SENTINEL=1): forces every DSP
     * write of d_fb_det (0x08f8) to 0xDEAD. The ARM reads that word through
     * arm=0x01f0 -> s->dsp->data[0x08f8] (calypso_trx.c), so the "ARM RD
     * d_fb_det" probe (TRX token) then shows:
     *   ARM reads 0xDEAD -> same cell, memory is coherent; look at PM/threshold.
     *   ARM reads 0x0000 -> ARM<->DSP desync. */
    {
        static int sent = -1;
        if (sent < 0) { const char *e = calypso_getenv("CALYPSO_FBDET_SENTINEL"); sent = e ? atoi(e) : 0;
            if (sent==1) fprintf(stderr, "[c54x] FBDET-SENTINEL=1 FORCE : data[0x08f8] forcé à 0xDEAD\n");
            else if (sent==2) fprintf(stderr, "[c54x] FBDET-SENTINEL=2 MONITOR : logge la vraie valeur écrite à 0x08f8 (pas de force)\n"); }
        {
            /* Force a_pm (rxlev) on the array the ARM actually reads:
             * calypso_trx.c reads s->dsp->data[off/2+0x800], not api_ram. The
             * DSP writes 0 there (no real measurement), so the calibrated
             * trf6151 value is substituted to keep rxlev stable. */
            {
                /* Hardware RSSI integrator model - the native pm_meas. On
                 * real Calypso the PM task (md=1) does NOT compute a_pm from
                 * the samples: the DSP zero-fills the result page (PROM0
                 * 0xb446, `stl *AR1+,A` in rptb 80), then an integrator reads a
                 * power register on the ABB/RF side and posts a_pm. That
                 * register is not in the modelled ADC, so it is derived from
                 * the real downlink magnitude the BSP measures. The
                 * MAV_REF -> RF_REF anchor is frontend calibration, like the
                 * trf6151 gain, not a decreed value: two different signals give
                 * two different a_pm. */
                static int rssi_on = -1;
                if (rssi_on < 0) {
                    rssi_on = calypso_gate("CALYPSO_PM_RSSI", 1);
                }
                /* db_r layout: d_task_ra(7), a_serv_demod[4](8..11),
                 * a_pm[3](12..14), a_sch[5](15..19). a_pm read page 0 = word 12
                 * = data[0x28+12+0x800] = data[0x834..0x836]; page 1 =
                 * data[0x3C+12+0x800] = data[0x848..0x84A]. 0x830/0x844 are
                 * a_serv_demod, not a_pm. */
                if (rssi_on && ((addr >= 0x0834 && addr <= 0x0836) ||
                            (addr >= 0x0848 && addr <= 0x084A))) {
                    val = calypso_bsp_rssi_apm();
                }
            }
        }
        if (sent && addr == 0x08f8) {
            uint16_t orig = val;
            if (sent == 1) val = 0xDEAD;   /* FORCE mode (coherence test) */
            static unsigned sn = 0;
            if (sn++ < 40 || (sn % 2000) == 0)
                fprintf(stderr, "[c54x] FBDET-SENTINEL #%u DSP write d_fb_det[0x08f8] orig=0x%04x%s PC=0x%04x insn=%u\n",
                        sn, orig, (sent==1) ? " ->0xDEAD" : " (monitor)", s->pc, s->insn_count);
        }
    }
    stkw_rec(s, addr, val);   /* ORPHAN: stack-write ring */
    /* ORPHAN: tracks direct stores in [0x1100..0x1140] (legitimate vector vs
     * untouched). Slots above SP_base are never reached by a push. */
    if (addr >= STKSLOT_LO && addr <= STKSLOT_HI) {
        int _si = addr - STKSLOT_LO;
        g_stkslot_wpc[_si] = s->pc;
        g_stkslot_wop[_si] = prog_fetch(s, s->pc);
        g_stkslot_written[_si] = 1;
    }
    /* MVPD overlay occupancy : count writes to [0x0080..0x27FF] during
     * boot phase. Env-gated CALYPSO_MVPD_TRACE=1. */
    if (g_mvpd_trace_enabled > 0) {
        mvpd_trace_record(addr);
    }
    /* FBMODE-WR: who writes d_fb_mode (0x08f9) and with which value (wide vs
     * narrow search)? Switching to narrow after the boot reject means no more
     * cold acquisition. */
    if (addr == 0x08f9) {
        static unsigned fm = 0;
        if (fm < 40) {
            fprintf(stderr, "[c54x] FBMODE-WR #%u d_fb_mode <- 0x%04x PC=0x%04x insn=%u\n",
                    fm, val, s->pc, s->insn_count);
            fm++;
        }
    }
    /* FBWATCH (env CALYPSO_FBWATCH, resolved once, outside the master gate) */
    if (g_fbwatch_on < 0) g_fbwatch_on = calypso_gate("CALYPSO_FBWATCH", 0);
    if (g_fbwatch_on) {
        /* (1) who writes the FB dispatch flag (data[0x60..0x70] / 0x3dc0..2)? */
        if ((addr >= 0x0060 && addr <= 0x0070) || (addr >= 0x3dc0 && addr <= 0x3dc2)) {
            static unsigned wf = 0;
            if (wf++ < 80)
                fprintf(stderr, "[c54x] FBWATCH-FLAG data[0x%04x] <- 0x%04x PC=0x%04x insn=%u%s\n",
                        addr, val, s->pc, s->insn_count, val ? "  *** NON-ZERO ***" : "");
        }
        /* (3) does the DSP ever write an FB detection (d_fb_det 0x08f8 non-zero)? */
        if (addr == 0x08f8 && val) {
            static unsigned wd = 0;
            if (wd++ < 40)
                fprintf(stderr, "[c54x] FBWATCH-DET d_fb_det <- 0x%04x PC=0x%04x insn=%u\n",
                        val, s->pc, s->insn_count);
        }
        /* (5) the producer flag: who writes data[0x585f], the foreground/ISR
         * status word? bit7 (0x0080) is polled by the foreground at 0xf7af,
         * bit8 (0x0100) by the ISRs. Never written, or bit7 never set, means the
         * flag has no producer. */
        if (addr == 0x585f) {
            static unsigned w5 = 0;
            if (w5++ < 80)
                fprintf(stderr, "[c54x] FBWATCH-585F data[0x585f] <- 0x%04x PC=0x%04x insn=%u%s\n",
                        val, s->pc, s->insn_count, (val & 0x0080) ? "  *** BIT7 SET ***" : "");
        }
        /* is the dispatch table populated? data[0x4c5b] = BACC A target at 0x7127 */
        if (addr == 0x4c5b) {
            static unsigned wt = 0;
            if (wt++ < 20)
                fprintf(stderr, "[c54x] FBWATCH-INITTAB-WR data[0x4c5b] <- 0x%04x PC=0x%04x insn=%u\n",
                        val, s->pc, s->insn_count);
        }
    }
    /* ANGLE-WR: who writes a_sync_demod ANGLE (0x08fc), TOA (0x08fa) and SNR
     * (0x08fd), the output of the real FCCH frequency detector. Captures A/B
     * (complex correlation) and the reference pointers AR3/4/5 at the store, to
     * name the actual frequency-correlation site. Cap 40. */
    /* [2026-09-19] A_SCH-WR: which R page does the ROM write its SB result to,
     * and what does d_dsp_page say at that instant? a_sch[0,1,3,4] sit at
     * 0x0837..0x083b (R page 0) and 0x084b..0x084f (R page 1) per calypso_api.h
     * (API_R_PAGE + RP_A_SCH). If the ROM writes the page the firmware is not
     * reading, the firmware decodes a stale cell -- which is what an SB result
     * that is wrong yet repeatable looks like. */
    if ((addr >= 0x0837 && addr <= 0x083b) || (addr >= 0x084b && addr <= 0x084f)) {
        static unsigned sw = 0;
        if (val != 0 && sw < 120) {
            int pg = (addr >= 0x084b);
            fprintf(stderr, "[c54x] A_SCH-WR #%u page=%d [0x%04x]<-0x%04x "
                    "d_dsp_page=0x%04x PC=0x%04x insn=%u\n",
                    sw, pg, addr, val, s->data[0x08d4], s->pc, s->insn_count);
            sw++;
        }
    }
    if (addr >= 0x08fa && addr <= 0x08fd) {
        static unsigned aw = 0, awz = 0;
        /* [2026-09-19] Only the NON-ZERO stores are of interest, and the cap of
         * 40 never reached them: measured, the first 40 writes are all 0x0000
         * from a single clearing site (PC 0xb2cf/b2d2/b2d5/b2d8, the four cells
         * in four consecutive instructions), so the probe filled up on the
         * memset and never showed the detector. Zeros are counted and reported
         * once every 2000 instead. */
        if (val == 0) {
            if (++awz % 2000 == 1)
                fprintf(stderr, "[c54x] ANGLE-WR (mise a zero x%u, PC=0x%04x)\n",
                        awz, s->pc);
        } else if (aw < 200) {
            int64_t a = (s->a & 0x8000000000LL) ? (int64_t)(s->a | ~0xFFFFFFFFFFLL) : (int64_t)s->a;
            int64_t b = (s->b & 0x8000000000LL) ? (int64_t)(s->b | ~0xFFFFFFFFFFLL) : (int64_t)s->b;
            const char *nm = addr==0x08fa?"TOA":addr==0x08fb?"PM":addr==0x08fc?"ANGLE":"SNR";
            fprintf(stderr, "[c54x] ANGLE-WR #%u %s[0x%04x]<-0x%04x PC=0x%04x A=%lld B=%lld T=%04x | "
                    "AR2=%04x[%04x] AR3=%04x[%04x] AR4=%04x[%04x] AR5=%04x[%04x] insn=%u\n",
                    aw, nm, addr, val, s->pc, (long long)a, (long long)b, s->t,
                    s->ar[2], s->data[s->ar[2]], s->ar[3], s->data[s->ar[3]],
                    s->ar[4], s->data[s->ar[4]], s->ar[5], s->data[s->ar[5]], s->insn_count);
            aw++;
        }
    }
    /* MTTCG lock: see data_read above. */
    qemu_mutex_lock(&calypso_pcb_daram_lock);
    data_write_locked(s, addr, val);
    qemu_mutex_unlock(&calypso_pcb_daram_lock);
}

/* FLOWTRACE: chronological R/W log of [0x2800,0x3000) into
 * /tmp/calypso_flow.txt, with a line budget in CALYPSO_FLOWTRACE. */
FILE *g_flow_f = NULL;
long  g_flow_budget = -2;
int   g_flow_armed = 0;
static void flow_log(const char *rw, uint16_t addr, uint16_t val, uint16_t pc, unsigned insn)
{
    if (g_flow_budget == -2) {
        const char *e = calypso_getenv("CALYPSO_FLOWTRACE");
        g_flow_budget = (e && *e) ? atol(e) : -1;
        if (g_flow_budget > 0) {
            g_flow_f = fopen("/tmp/calypso_flow.txt", "w");
            fprintf(stderr, "[c54x] FLOWTRACE armed budget=%ld -> /tmp/calypso_flow.txt\n", g_flow_budget);
        }
    }
    if (g_flow_budget <= 0 || !g_flow_f || !g_flow_armed) return;
    if (addr < 0x2800 || addr >= 0x3000) return;
    fprintf(g_flow_f, "%s %04x %04x pc=%04x insn=%u\n", rw, addr, val, pc, insn);
    if (--g_flow_budget == 0) { fflush(g_flow_f); fclose(g_flow_f); g_flow_f = NULL;
        fprintf(stderr, "[c54x] FLOWTRACE done -> /tmp/calypso_flow.txt\n"); }
}


/* WMAP: aggregated map of the writers of a data[] range. An aggregate, not a
 * stream: no line cap, so no window can be missed by truncation. Periodic output
 * gives PC, count, distinct values, min/max. */
struct c54x_g_wmap_s g_wmap[WMAP_PCS];
int      g_wmap_n;
uint32_t g_wmap_tot;
int      g_wmap_on = -1;
uint16_t g_wmap_lo, g_wmap_hi, g_wmap_lo2, g_wmap_hi2;

static void wmap_dump(void)
{
    fprintf(stderr, "[c54x] WMAP plages 0x%04x..0x%04x + 0x%04x..0x%04x  ecritures=%u  ecrivains=%d%s\n",
            g_wmap_lo, g_wmap_hi, g_wmap_lo2, g_wmap_hi2, g_wmap_tot, g_wmap_n,
            g_wmap_n >= WMAP_PCS ? "  *** TABLE SATUREE, ecrivains manquants ***" : "");
    for (int i = 0; i < g_wmap_n; i++) {
        fprintf(stderr, "[c54x] WMAP   PC=0x%04x n=%-7u @0x%04x(n=%u) distinct=%s%d  min=0x%04x max=0x%04x  ex:",
                g_wmap[i].pc, g_wmap[i].n, g_wmap[i].addr0, g_wmap[i].n0,
                g_wmap[i].nv >= 8 ? ">=" : "", g_wmap[i].nv,
                g_wmap[i].mn, g_wmap[i].mx);
        for (int k = 0; k < g_wmap[i].nv && k < 8; k++)
            fprintf(stderr, " %04x", g_wmap[i].v[k]);
        fprintf(stderr, "%s\n", g_wmap[i].n0 < 2 ? "   <= (trop peu d echantillons)"
                : g_wmap[i].nv == 1 ? "   <= CONSTANTE dans le temps"
                                    : "   <= VARIE dans le temps = PORTE DE LA DONNEE");
    }
}

/* Heartbeat: proves the probe is alive even at zero writes. Called on every
 * write outside the range; prints once per 5e6 global writes, repeating the
 * total INSIDE the range (0 there is a real zero). */
static void wmap_heartbeat(void)
{
    static uint64_t k;
    if (!g_wmap_on) return;
    if (++k % 5000000ULL) return;
    fprintf(stderr, "[c54x] WMAP heartbeat: writes DSP=%llu, dans plages=%u"
            " (0 = la plage n est jamais ecrite par une instruction DSP)\n",
            (unsigned long long)k, g_wmap_tot);
}

static void wmap_note(uint16_t addr, uint16_t val, uint16_t pc)
{
    if (g_wmap_on < 0) {
        const char *e = calypso_getenv("CALYPSO_WMAP");
        g_wmap_on = (e && atoi(e) > 0) ? 1 : 0;
        const char *lo = calypso_getenv("CALYPSO_WMAP_LO"), *hi = calypso_getenv("CALYPSO_WMAP_HI");
        g_wmap_lo = lo ? (uint16_t)strtoul(lo, NULL, 0) : 0x2c00;
        g_wmap_hi = hi ? (uint16_t)strtoul(hi, NULL, 0) : 0x2c1f;
        const char *lo2 = calypso_getenv("CALYPSO_WMAP_LO2"), *hi2 = calypso_getenv("CALYPSO_WMAP_HI2");
        g_wmap_lo2 = lo2 ? (uint16_t)strtoul(lo2, NULL, 0) : 0xffff;
        g_wmap_hi2 = hi2 ? (uint16_t)strtoul(hi2, NULL, 0) : 0x0000;
        if (g_wmap_on)
            fprintf(stderr, "[c54x] WMAP armed 0x%04x..0x%04x\n", g_wmap_lo, g_wmap_hi);
    }
    if (!g_wmap_on) return;
    if (!((addr >= g_wmap_lo  && addr <= g_wmap_hi) ||
          (addr >= g_wmap_lo2 && addr <= g_wmap_hi2))) { wmap_heartbeat(); return; }

    int i;
    for (i = 0; i < g_wmap_n; i++) if (g_wmap[i].pc == pc) break;
    if (i == g_wmap_n) {
        if (g_wmap_n >= WMAP_PCS) return;
        g_wmap_n++;
        g_wmap[i].pc = pc; g_wmap[i].n = 0; g_wmap[i].nv = 0;
        g_wmap[i].mn = 0xffff; g_wmap[i].mx = 0;
        g_wmap[i].addr0 = addr; g_wmap[i].n0 = 0;   /* fixed witness cell */
    }
    g_wmap[i].n++;
    if (val < g_wmap[i].mn) g_wmap[i].mn = val;
    if (val > g_wmap[i].mx) g_wmap[i].mx = val;
    /* The signal criterion is variation over TIME at a FIXED address. */
    if (addr != g_wmap[i].addr0) return;
    g_wmap[i].n0++;
    if (g_wmap[i].nv < 8) {
        int seen = 0;
        for (int k = 0; k < g_wmap[i].nv; k++) if (g_wmap[i].v[k] == val) { seen = 1; break; }
        if (!seen) g_wmap[i].v[g_wmap[i].nv++] = val;
    }
    /* Low threshold plus a periodic tick, so that ABSENCE is measurable. */
    if (++g_wmap_tot % WMAP_TICK == 0) wmap_dump();
}


/* DEMODIO: per-access trace of the demod loop (PC range + insn threshold). */
int      g_dio_on = -1;
uint64_t g_dio_after;
unsigned g_dio_n;
uint16_t g_dio_pclo, g_dio_pchi;

static void dio_init(void)
{
    const char *e = calypso_getenv("CALYPSO_DEMODIO");
    g_dio_on = (e && atoi(e) > 0) ? 1 : 0;
    const char *a = calypso_getenv("CALYPSO_DEMODIO_AFTER");
    g_dio_after = a && *a ? strtoull(a, NULL, 0) : 40000000ULL;
    const char *lo = calypso_getenv("CALYPSO_DEMODIO_PCLO"), *hi = calypso_getenv("CALYPSO_DEMODIO_PCHI");
    g_dio_pclo = lo ? (uint16_t)strtoul(lo, NULL, 0) : 0x9f95;
    g_dio_pchi = hi ? (uint16_t)strtoul(hi, NULL, 0) : 0x9fe2;
    if (g_dio_on)
        fprintf(stderr, "[c54x] DEMODIO armed PC 0x%04x..0x%04x apres insn=%llu\n",
                g_dio_pclo, g_dio_pchi, (unsigned long long)g_dio_after);
}

static void dio_note(C54xState *s, const char *rw, uint16_t addr, uint16_t val)
{
    if (g_dio_on < 0) dio_init();
    if (!g_dio_on) return;
    if (s->pc < g_dio_pclo || s->pc > g_dio_pchi) return;
    if (s->insn_count < g_dio_after) return;
    if (g_dio_n >= 160) return;
    int64_t a = (s->a & 0x8000000000LL) ? (int64_t)(s->a | ~0xFFFFFFFFFFLL) : (int64_t)s->a;
    int64_t b = (s->b & 0x8000000000LL) ? (int64_t)(s->b | ~0xFFFFFFFFFFLL) : (int64_t)s->b;
    /* ASM and raw ST1 are printed because the loop at 0x9fab-0x9fb8 builds the
     * correlator reference into 0x2a00+ with
     *     0x9fb5  ld  *AR6-0%, TS, A     (A = table(+-1) << T, T in [16,31])
     *     0x9fb6  sfta A, +8
     *     0x9fb7  sfta A, -8             (the two cancel out)
     *     0x9fb8  sth *AR4+, A, ASM      (0x8694 = the ASM variant)
     * and it only ever writes 0x0000 / 0xffff, i.e. {0, -1} instead of {+K, -K}:
     * a reference with |DC|/rms ~ 0.7, matching the 0.60 measured on
     * daram_2a00.cfile, which is why no correlation peaks. The high word of
     * (A >> |ASM|) is 0 for any magnitude below 2^16 but 0xffff for any negative
     * value (sign extension), so the asymmetry depends entirely on ASM and T. T
     * was already printed; ASM must be measured HERE, since an ASM read at
     * another PC belongs to another window. */
    int _asm = s->st1 & 0x1F; if (_asm & 0x10) _asm |= ~0x1F;
    fprintf(stderr, "[c54x] DEMODIO %s PC=0x%04x op=0x%04x addr=0x%04x val=0x%04x "
            "A=%lld B=%lld T=0x%04x ASM=%d ST1=0x%04x AR2=%04x AR3=%04x AR4=%04x AR5=%04x insn=%u\n",
            rw, s->pc, prog_fetch(s, s->pc), addr, val, (long long)a, (long long)b,
            s->t, _asm, s->st1, s->ar[2], s->ar[3], s->ar[4], s->ar[5], s->insn_count);
    g_dio_n++;
}

static void data_write_locked(C54xState *s, uint16_t addr, uint16_t val)
{
    /* [2026-09-19] A_SCH-WR2: the twin probe in data_write() never fired on
     * a_sch while its ANGLE-WR neighbour did, so these writes take the OTHER
     * path. Here the ADDRESS is the function's own argument, so the attribution
     * is exact -- unlike a before/after snapshot per instruction, which the ARM
     * writing the same shared mapping from another process can fool. */
    /* [2026-09-19] TOA-LAW: the firmware wants a ToA of 23 and never sees it;
     * the measured values form a ladder of 48 (48, 96, 144, 192...). The ROM
     * stores a_sync[TOA] at 0x795a and the stored word equals B, with T holding
     * the constant 48 loaded at 0x7953. Capture B, T and the pointers on every
     * such store to get the law instead of guessing it. */
    /* [2026-09-19] ANGLE-LAW : meme traitement que le TOA, qui s'etait revele
     * avoir DEUX producteurs separes par AR2 (0x0cce = une grille, autre = une
     * vraie mesure). L'angle est ecrit par la ROM en 0x798b et rend ~0 quel que
     * soit l'offset injecte ; s'il a lui aussi deux populations, l'une des deux
     * mesure peut-etre correctement et c'est la mauvaise qui est consommee. */
    /* [2026-09-19] BUF-0cce : le TOA (grille de 48) et l'angle (bruit non
     * correle) sont TOUS DEUX degrades quand AR2=0x0cce. Hypothese unificatrice :
     * ce que le correlateur lit la ne porte pas le signal. Test direct -- la
     * FCCH est une tonalite pure, donc a 1 ech/symbole elle tourne de +pi/2 par
     * echantillon avec une coherence ~1. On mesure coherence et dphi SUR LE
     * CONTENU DU TAMPON au moment ou la tache FB rend son resultat. */
    if ((addr == 0x08fc || addr == 0x08fa) && val != 0 && s->ar[2] == 0x0cce) {
        static unsigned nb;
        if (nb < 20) {
            double ar=0, ai=0, den=0; int ns=148;
            const uint16_t BUF = 0x0cce;
            for (int k = 1; k < ns; k++) {
                double i0=(int16_t)s->data[BUF + 2*(k-1)], q0=(int16_t)s->data[BUF + 2*(k-1) + 1];
                double i1=(int16_t)s->data[BUF + 2*k],     q1=(int16_t)s->data[BUF + 2*k + 1];
                ar += i1*i0 + q1*q0;  ai += q1*i0 - i1*q0;
                den += sqrt((i0*i0+q0*q0)*(i1*i1+q1*q1));
            }
            double coh = den > 0 ? sqrt(ar*ar+ai*ai)/den : 0;
            double dphi = atan2(ai, ar);
            fprintf(stderr, "[c54x] BUF-0cce %s=%d : coherence=%.3f dphi=%+.3f "
                    "(FCCH pure vise coh~1, dphi=+1.571)\n",
                    addr == 0x08fa ? "TOA" : "ANGLE", (int)(int16_t)val, coh, dphi);
            nb++;
        }
    }
    if (addr == 0x08fc && val != 0) {
        static unsigned na;
        if (na < 300)
            fprintf(stderr, "[c54x] ANGLE-LAW <-%6d  AR2=%04x AR3=%04x AR4=%04x "
                    "AR5=%04x A=%lld B=%lld T=%04x PC=0x%04x\n",
                    (int)(int16_t)val, s->ar[2], s->ar[3], s->ar[4], s->ar[5],
                    (long long)s->a, (long long)s->b, s->t, s->pc);
        na++;
    }
    if (addr == 0x08fa) {
        static unsigned nt;
        /* [2026-09-19] Deux producteurs de TOA sortent de la MEME instruction
         * 0x795a, separes par AR2 : 0x0cce donne 100% de multiples de 48 (une
         * grille, pas une mesure), tout autre pointeur donne des valeurs fines.
         * On marque la provenance pour savoir laquelle la tache SB consomme. */
        if (s->pc == 0x795a) {
            g_toa_grille = (s->ar[2] == 0x0cce);
            g_toa_valeur = (int)(int16_t)val;
            g_toa_seq++;
        }
        /* [2026-09-19] La couture. Le TOA tombe a 100% sur un multiple de 48
         * echantillons = 96 mots = une page DMA, donc le tampon doit porter une
         * discontinuite a chaque frontiere de page. On imprime les mots de part
         * et d'autre des frontieres (96 et 192) du tampon 0x0cce : sur un burst
         * GMSK continu, |IQ| est quasi constant et rien ne doit sauter la. */
        if (val != 0 && nt < 12 && s->pc == 0x795a) {
            const uint16_t B0 = 0x0cce;
            fprintf(stderr, "[c54x] COUTURE TOA=%d | p1 94,95 -> 96,97 : "
                    "%04x %04x | %04x %04x || p2 190,191 -> 192,193 : "
                    "%04x %04x | %04x %04x\n", (int)(int16_t)val,
                    s->data[B0 + 94], s->data[B0 + 95],
                    s->data[B0 + 96], s->data[B0 + 97],
                    s->data[B0 + 190], s->data[B0 + 191],
                    s->data[B0 + 192], s->data[B0 + 193]);
        }
        if (nt < 400)
            fprintf(stderr, "[c54x] TOA-LAW <-%5d  B=%lld T=%d A=%lld "
                    "AR2=%04x AR3=%04x AR4=%04x AR5=%04x PC=0x%04x\n",
                    (int)(int16_t)val, (long long)s->b, (int)(int16_t)s->t,
                    (long long)s->a, s->ar[2], s->ar[3], s->ar[4], s->ar[5], s->pc);
        nt++;
    }
    /* [2026-09-19] A_SERV-WR : read_sb_result (prim_fbsb.c:148) lit
     * dsp_api.db_r->a_serv_demod[], donc la PAGE R aux mots 8..11 =
     * 0x0830..0x0833 (page 0) et 0x0844..0x0847 (page 1) -- PAS le NDB
     * a_sync_demod 0x08fa..0x08fd que lit read_fb_result. Si personne n'ecrit
     * ces cellules, le SB lit du mort : st->snr reste nul, le test
     * `snr > AFC_SNR_THRESHOLD` echoue toujours et TOUTES les mesures AFC sont
     * marquees invalides -- la moyenne glissante n'emet alors jamais. */
    /* [2026-09-19] SRC-3fa4 : les quatre resultats du SB sortent de quatre
     * cellules de travail, lues par la ROM en 0xb1e7..0xb1f5 :
     *   0x3fa4 -> TOA    0x3fa5 -> PM    0x3fa7 -> ANGLE    0x3fa6 -> SNR
     * Le SNR ne depasse jamais AFC_SNR_THRESHOLD=2560 (max observe 864), donc
     * 100% des mesures AFC sont marquees invalides. On remonte d'un cran : qui
     * ecrit 0x3fa6, et avec quoi ? */
    /* [2026-09-19] SNR-BINAIRE : le SNR publie vaut soit 16384 (0x4000, la
     * constante "bon signal" que la ROM pose aussi en 0x798d) soit une valeur a
     * un ou deux chiffres -- rien entre les deux, 15,3% de 16384 sur 111
     * mesures. Il en faudrait 20% pour atteindre les 8 mesures valides par
     * fenetre de 40 qu'exige l'AFC. Qu'est-ce qui distingue un bon burst d'un
     * mauvais ? On mesure la coherence du tampon 0x0cce AU MOMENT de l'ecriture,
     * et la phase de multitrame : une FCCH (fn%51 dans {0,10,20,30,40}) est une
     * tonalite pure, coherence ~1 et dphi=+pi/2 a 1 ech/symbole. */
    /* [2026-09-19] DECALAGE : le verdict SNR n'est PAS correle au contenu du
     * tampon -- 16384 sur du factice (coherence 0.488), deux chiffres sur de la
     * FCCH. Quand une vraie FCCH est la, le DSP la voit parfaitement
     * (coherence 0.999, dphi +1.565 pour +1.571 theorique). Donc le signal et
     * l'estimateur sont bons : c'est l'INSTANT de l'evaluation qui est decale
     * par rapport au depot. On apparie chaque evaluation avec le dernier depot
     * DMA et on compte les trames qui les separent. */
    /* [2026-09-19] CRC-CELL : le statut SB est forme en 0x98b3/0x98b4 --
     * A prend 0x8000 (B_BLUD seul) puis 0x8100, l'instruction 0x98b4 ayant
     * pour operandes 0x2bf8 et 0x0c08. Si data[0x0c08] porte 0x0100, c'est la
     * cellule du verdict CRC, et son ecrivain est le decodeur lui-meme. */
    if (addr == 0x0c08) {
        static unsigned n7;
        if (n7 < 60) {
            fprintf(stderr, "[c54x] CRC-CELL [0x0c08] <- 0x%04x  PC=0x%04x "
                    "A=%lld B=%lld T=%04x AR2=%04x AR3=%04x insn=%u\n",
                    val, s->pc, (long long)s->a, (long long)s->b, s->t,
                    s->ar[2], s->ar[3], s->insn_count);
            n7++;
        }
    }
    if (addr == 0x3fa6) {
        extern unsigned g_depot_fn, g_depot_seq;
        static unsigned n6;
        if (n6 < 60) {
            unsigned fn_eval = calypso_daram_last_fn;
            fprintf(stderr, "[c54x] DECALAGE snr=%-6d eval_fn=%-7u p51=%-2u | "
                    "dernier depot fn=%-7u p51=%-2u | ecart=%d trames\n",
                    (int)(int16_t)val, fn_eval, fn_eval % 51u,
                    g_depot_fn, g_depot_fn % 51u, (int)(fn_eval - g_depot_fn));
            n6++;
        }
    }
    if (addr == 0x3fa6) {
        static unsigned n5;
        if (n5 < 400) {
            double ar=0, ai=0, den=0; const uint16_t BUF = 0x0cce; int ns=148;
            for (int k = 1; k < ns; k++) {
                double i0=(int16_t)s->data[BUF + 2*(k-1)], q0=(int16_t)s->data[BUF + 2*(k-1) + 1];
                double i1=(int16_t)s->data[BUF + 2*k],     q1=(int16_t)s->data[BUF + 2*k + 1];
                ar += i1*i0 + q1*q0;  ai += q1*i0 - i1*q0;
                den += sqrt((i0*i0+q0*q0)*(i1*i1+q1*q1));
            }
            double coh = den > 0 ? sqrt(ar*ar+ai*ai)/den : 0;
            unsigned fn = calypso_daram_last_fn;
            fprintf(stderr, "[c54x] SNR-BINAIRE snr=%-6d fn=%-7u p51=%-2u "
                    "coherence=%.3f dphi=%+.3f %s\n",
                    (int)(int16_t)val, fn, fn % 51u, coh, atan2(ai, ar),
                    (fn % 51u) % 10 == 0 && (fn % 51u) <= 40 ? "<- trame FCCH" : "");
            n5++;
        }
    }
    if (addr >= 0x3fa4 && addr <= 0x3fa7) {
        static unsigned n4;
        if (n4 < 80) {
            static const char *q[4] = { "TOA(3fa4)", "PM(3fa5)", "SNR(3fa6)", "ANGLE(3fa7)" };
            fprintf(stderr, "[c54x] SRC-3fa4 %-11s <- %6d  PC=0x%04x A=%lld B=%lld "
                    "T=%04x AR2=%04x AR3=%04x insn=%u\n",
                    q[addr - 0x3fa4], (int)(int16_t)val, s->pc,
                    (long long)s->a, (long long)s->b, s->t, s->ar[2], s->ar[3],
                    s->insn_count);
            n4++;
        }
    }
    if ((addr >= 0x0830 && addr <= 0x0833) || (addr >= 0x0844 && addr <= 0x0847)) {
        static unsigned n3;
        if (n3 < 600) {
            static const char *nm[4] = { "TOA", "PM", "ANGLE", "SNR" };
            int pg = (addr >= 0x0844);
            int idx = addr - (pg ? 0x0844 : 0x0830);
            fprintf(stderr, "[c54x] A_SERV-WR page=%d %-5s [0x%04x] <- %6d "
                    "PC=0x%04x AR2=%04x insn=%u\n",
                    pg, nm[idx & 3], addr, (int)(int16_t)val, s->pc, s->ar[2],
                    s->insn_count);
            n3++;
        }
    }
    if ((addr >= 0x0837 && addr <= 0x083b) || (addr >= 0x084b && addr <= 0x084f)) {
        static unsigned n2;
        if (n2 < 60) {
            fprintf(stderr, "[c54x] A_SCH-WR2 [0x%04x] <- 0x%04x PC=0x%04x "
                    "A=%lld B=%lld T=%04x AR2=%04x AR3=%04x insn=%u\n",
                    addr, val, s->pc, (long long)s->a, (long long)s->b, s->t,
                    s->ar[2], s->ar[3], s->insn_count);
            n2++;
        }
    }
    { static long _dwl_n = 0; static int _dwl_on = -1; if (_dwl_on < 0) _dwl_on = calypso_getenv("CALYPSO_DWL_PROVE") ? 1 : 0;
      if (_dwl_on && (_dwl_n++ % 100000) == 0)
        fprintf(stderr, "[c54x] DWL-PROVE appel #%ld addr=0x%04x pc=0x%04x\n", _dwl_n, addr, s->pc); }
    {   /* FBCNT-WATCH - CALYPSO_FBROUTE=1 (same gate as FBROUTE, of which this
         * is the direct sequel). Read-only, capped.
         *
         * [2026-08-03] Since the DMA delivers samples the FB routine finally
         * runs (FBROUTE: ENTER twice, high-water 0x794e - the zone was never
         * entered before), which makes the guard at 0x79e3 measurable:
         *     at 0x7720, DP=0x083 -> dma(0x7e) = data[0x41fe] = 1, 2, 3, 3, 3, 3
         *     the guard requires == 4.
         * The counter tops out at 3, exactly one short. A candidate explanation
         * is that the DMA delivers the burst in 4 pages (ALGTH=192 -> 96 words,
         * burst=296) while the counter counts 3; this watch says who writes the
         * cell and when, which confirms or kills it.
         *
         * Known limit: 0x41fe is the address resolved at 0x7720 (DP=0x083). The
         * guard is at 0x79e3, which is never reached, so there is no proof it
         * reads the same cell. The inference is reasonable (same direct
         * addressing at 0x7e, same routine, DP constant over 6 samples) but
         * UNPROVEN. An incoherent writer in this watch indicts that inference
         * first, not the counter. */
        static int _fc = -1;
        if (_fc < 0) _fc = calypso_gate("CALYPSO_FBROUTE", 0);
        if (_fc && addr == 0x41fe) {
            static unsigned _n = 0;
            if (_n < 60) {
                _n++;
                fprintf(stderr,
                        "[c54x] FBCNT-WR #%u data[0x41fe] 0x%04x -> 0x%04x "
                        "PC=0x%04x A=0x%06llx insn=%u\n",
                        _n, s->data[0x41fe], val, s->last_exec_pc,
                        (unsigned long long)(s->a & 0xFFFFFFULL), s->insn_count);
                fflush(stderr);
            }
        }
    }
    {   /* WATCH-3FB4 (gate CALYPSO_WATCH_3FB4, read-only, capped): state/slot
         * index of the FB search. The coarse TOA (mode 0, branch A at 0x791c) is
         * (data[0x3fb4]-3)*T, so this says whether the index advances
         * (0->1->2...) or stays stuck at 0. DSP writers: 0x7744, 0x7776. */
        static int _w4 = -1;
        if (_w4 < 0) _w4 = calypso_gate("CALYPSO_WATCH_3FB4", 0);
        if (_w4 && addr == 0x3fb4) {
            static unsigned _n4 = 0;
            if (_n4 < 80) {
                _n4++;
                fprintf(stderr, "[c54x] WATCH-3FB4 data[0x3fb4] 0x%04x -> 0x%04x PC=0x%04x "
                        "T=0x%04x A=0x%06llx insn=%u\n", s->data[0x3fb4], val, s->pc,
                        s->t, (unsigned long long)(s->a & 0xFFFFFFULL), s->insn_count);
                fflush(stderr);
            }
        }
    }
    {   /* DTASKD-WATCH, leg 3/3 - CALYPSO_DTASKD_WATCH=1, default 0. Read-only,
         * capped. Legs 1 and 2 are in calypso_trx.c.
         *
         * The firmware console shows, every cycle, l1s_nb_cmd() requesting a
         * reception and l1s_nb_resp() printing EMPTY (prim_rx_nb.c:74), whose
         * test is
         *     if (dsp_api.db_r->d_task_d == 0) { puts("EMPTY\n"); return 0; }
         *
         * Trap: db_w->d_task_d and db_r->d_task_d are NOT the same cell. They are
         * two distinct firmware structures (osmocom-bb dsp_api.h:20-23, d_task_d
         * at offset 0 in both):
         *     db_w = T_DB_MCU_TO_DSP @ 0xFFD00000 (p0) / 0xFFD00028 (p1)
         *     db_r = T_DB_DSP_TO_MCU @ 0xFFD00050 (p0) / 0xFFD00078 (p1)
         * So EMPTY does not mean the ARM write was lost; it means the DSP never
         * copied the task into its RESPONSE page. This leg settles that.
         *
         * Matching DSP words (api = data[0x0800 + ARM_offset/2]):
         *     W p0 = data[0x0800]   W p1 = data[0x0814]   <- written by the ARM
         *     R p0 = data[0x0828]   R p1 = data[0x083C]   <- must be written here
         *
         * DISPATCH-PROBE already shows AR2 alternating between 0x0828 and 0x083c,
         * exactly the two R page bases: the DSP holds the right pointer, the
         * question is whether it writes the cell.
         *
         * The periodic summary prints the counters even at zero, so no line is
         * ambiguous between "the DSP does not write" and "the probe is off". */
        static int _dw = -1;
        if (_dw < 0) {
            _dw = calypso_gate("CALYPSO_DTASKD_WATCH", 0);
            if (_dw) {
                fprintf(stderr, "[dtaskd] patte 3/3 armee (ecritures DSP) : "
                        "R p0=data[0x0828] R p1=data[0x083C]\n");
                fflush(stderr);
            }
        }
        if (_dw && (addr == 0x0828 || addr == 0x083C)) {
            static unsigned long long _n0 = 0, _n1 = 0, _nz = 0;
            unsigned long long _n = (addr == 0x0828) ? ++_n0 : ++_n1;
            if (val) _nz++;
            if (_n <= 40 || (_n % 5000) == 0) {
                fprintf(stderr,
                        "[dtaskd] DSP>WR  R p%d  data[0x%04x] <- 0x%04x  "
                        "(non_nuls=%llu  p0=%llu p1=%llu)  PC=0x%04x insn=%u\n",
                        (addr == 0x0828) ? 0 : 1, addr, val,
                        _nz, _n0, _n1, s->last_exec_pc, s->insn_count);
                fflush(stderr);
            }
        }
    }
    dio_note(s, "W", addr, val);
    wmap_note(addr, val, s->pc);
    flow_log("W", addr, val, s->pc, s->insn_count);
    {   /* WZWRITE: who fills the MAC kernel input cell 0x2c00? */
        static int _wz = -1; static unsigned _wzn = 0;
        if (_wz < 0) _wz = calypso_gate("CALYPSO_WZWRITE", 0);
        if (_wz && addr == 0x2c00 && (s->pc == 0x9fd5 || s->pc == 0x9ab1) && _wzn < 40) {
            /* Filtered on the producing PC: the MAC init sites
             * 0xa03d/0xa042/0xa079 are excluded, they used to saturate the cap
             * and hide the bursts that follow. */
            _wzn++;
            fprintf(stderr, "[c54x] WZWRITE data[0x%04x] <- 0x%04x PC=0x%04x op=0x%04x "
                    "A=0x%010llx AR2=%04x AR3=%04x AR4=%04x AR5=%04x insn=%u\n",
                    addr, val, s->pc, prog_fetch(s, s->pc),
                    (unsigned long long)(s->a & 0xFFFFFFFFFFULL),
                    s->ar[2], s->ar[3], s->ar[4], s->ar[5], s->insn_count);
        }
    }
    {   /* DMAQ-RESET (gate CALYPSO_DMAQ, cap 30): does the DMA queue write
         * pointer return to its base without having been consumed - producer
         * pushes, a reset wipes, consumer finds the queue empty? */
        static int _dqr = -1; static unsigned _dqrn = 0;
        if (_dqr < 0) _dqr = calypso_gate("CALYPSO_DMAQ", 0);
        if (_dqr && _dqrn < 30 && (addr == 0x433f || addr == 0x433e)
            && val == 0x4330 && s->data[addr] != 0x4330) {
            _dqrn++;
            fprintf(stderr, "[c54x] DMAQ-RESET data[0x%04x] 0x%04x -> 0x4330 "
                    "PC=0x%04x (la file contenait %u element(s)) insn=%u\n",
                    addr, s->data[addr], s->pc,
                    (unsigned)((s->data[0x433f] - s->data[0x433e]) & 0xFFFF),
                    s->insn_count);
        }
    }
    {   /* ERRWATCH: who sets d_error_status (0x08D5), and with which bit? */
        static int _ew = -1; static unsigned _ewn = 0;
        if (_ew < 0) _ew = calypso_gate("CALYPSO_ERRWATCH", 0);
        /* Writes of 0 are ignored: they are the boot cleanup and used to
         * consume the whole cap. */
        if (_ew && addr == 0x08D5 && val != 0 && _ewn < 60) {
            _ewn++;
            const char *_b = (val & 0x0800) ? "STACK_OV" : (val & 0x0400) ? "DMA_UL_PEND" :
                             (val & 0x0200) ? "DMA_UL_PROG" : (val & 0x0100) ? "DMA_UL_TASK" :
                             (val & 0x0080) ? "VM" : (val & 0x0020) ? "DMA_PEND" :
                             (val & 0x0010) ? "DMA_TASK" : (val & 0x0008) ? "DMA_PROG" :
                             (val & 0x0004) ? "IQ_SAMPLES" : (val & 0x0001) ? "RHEA" : "(clear)";
            fprintf(stderr, "[c54x] ERRWATCH data[0x%04x] 0x%04x -> 0x%04x [%s] "
                    "PC=0x%04x op=0x%04x A=0x%06llx AR1=%04x AR2=%04x AR6=%04x insn=%u\n",
                    addr, s->data[addr], val, _b, s->pc, prog_fetch(s, s->pc),
                    (unsigned long long)(s->a & 0xFFFFFFULL),
                    s->ar[1], s->ar[2], s->ar[6], s->insn_count);
        }
    }
    {   /* DMAWATCH (write side). */
        static int _dw3 = -1; static unsigned _dww = 0;
        if (_dw3 < 0) _dw3 = calypso_gate("CALYPSO_DMAWATCH", 0);
        if (_dw3 && addr >= 0x0054 && addr <= 0x0057 && _dww < 40) {
            _dww++;
            fprintf(stderr, "[c54x] DMAWATCH WR 0x%04x <- 0x%04x (etait 0x%04x) PC=0x%04x insn=%u\n",
                    addr, val, s->data[addr], s->pc, s->insn_count);
        }
    }
    {   /* BOOTCMD, DSP side: who overwrites the boot command cell? */
        static int _bc2 = -1; static unsigned _bc2n = 0;
        if (_bc2 < 0) _bc2 = calypso_gate("CALYPSO_BOOTCMD", 0);
        if (_bc2 && addr >= 0x0FFC && addr <= 0x0FFF && _bc2n < 40) {
            _bc2n++;
            fprintf(stderr, "[c54x] BOOTCMD DSP data[0x%04x] 0x%04x -> 0x%04x PC=0x%04x insn=%u%s\n",
                    addr, s->data[addr], val, s->pc, s->insn_count,
                    (addr == 0x0FFF) ? "   <<<< CELLULE DE COMMANDE" : "");
        }
    }
    {   /* DISPTAB-WR: who fills the dispatch table 0x4380..0x43cf? */
        static int _dt = -1; static unsigned _dtn = 0;
        if (_dt < 0) _dt = calypso_gate("CALYPSO_DISPTAB", 0);
        if (_dt && addr >= 0x4380 && addr <= 0x43cf && _dtn < 60) {
            _dtn++;
            fprintf(stderr, "[c54x] DISPTAB-WR data[0x%04x] <- 0x%04x PC=0x%04x insn=%u\n",
                    addr, val, s->pc, s->insn_count);
        }
    }
    {
        /* @BEQUILLE - DEMOD_NOCLOBBER  (CALYPSO_DEMOD_NOCLOBBER, default OFF)
         *   masque  : the emulated demod stage (PC 0x9fb8 = I, 0x9fe2 = Q) fills
         *             the correlator input buffer [0x2a00,0x2b28) with constant
         *             pairs (0000,52ed) and overwrites the real FCCH laid down
         *             by feed_iq; the real branch is a demod that consumes the
         *             RX I/Q instead of producing constants. Here the write is
         *             DROPPED (return) instead of the producer being fixed.
         *   retirer : as soon as the demod stage 0x9f95-0x9fe2 reads a real I/Q
         *             source, or FB_IQ_OWNS=1 makes feed_iq the sole owner of
         *             0x2a00.
         */
        static int _nc = -1;
        if (_nc < 0) { const char *e = calypso_getenv("CALYPSO_DEMOD_NOCLOBBER"); _nc = (e && atoi(e) > 0) ? 1 : 0; }
        if (_nc && addr >= 0x2a00 && addr < 0x2b28 &&
            (s->pc == 0x9fb8 || s->pc == 0x9fe2)) {
            static unsigned _ncn = 0;
            if (_ncn++ < 8)
                fprintf(stderr, "[c54x] DEMOD-NOCLOBBER skip PC=0x%04x data[0x%04x] <- 0x%04x "
                        "(ecriture demod ignoree ; l alimentation vient de rx_burst sauf si FB_IQ_OWNS=1)\n", s->pc, addr, val);
            return;
        }
    }
    /* B4 (gate CALYPSO_B4): watchpoint on d_fb_det (0x08f8) - separates "the DSP
     * writes 0" (correlator concludes negative) from "never written" (path not
     * reached). Two different bugs. */
    if (addr == 0x08f8) {
        static int _b4 = -1; static unsigned _b4n = 0;
        if (_b4 < 0) _b4 = calypso_gate("CALYPSO_B4", 0);
        if (_b4 && _b4n < 64) {
            _b4n++;
            fprintf(stderr, "[c54x] B4-DFBDET-WR data[0x08f8] 0x%04x -> 0x%04x PC=0x%04x xpc=%u insn=%u\n",
                    s->data[0x08f8], val, s->pc, s->xpc, s->insn_count);
        }
    }
    /* B1 boot-copy watch (gate CALYPSO_B1): writes into the reference table
     * [0x2c00,0x2c10) with the source PC, to confirm the 0x76f8 -> 0x2c00 boot
     * copy and name its writer. */
    if (addr >= 0x2c00 && addr < 0x2c10) {
        static int _b1w = -1; static unsigned _b1wn = 0;
        if (_b1w < 0) _b1w = calypso_gate("CALYPSO_B1", 0);
        if (_b1w && _b1wn < 64 && val != 0) {
            _b1wn++;
            fprintf(stderr, "[c54x] B1-BOOTCOPY-WR data[0x%04x] 0x%04x -> 0x%04x PC=0x%04x xpc=%u insn=%u\n",
                    addr, s->data[addr], val, s->pc, s->xpc, s->insn_count);
        }
    }
    /* MEM-WATCH-2B80 (gate CALYPSO_MEM_WATCH_2B80): twin of the read-side probe
     * in data_read_locked. Logs every write in [0x2b80,0x2c00), cap 200, to say
     * whether a boot copy ever populates the region before the correlator reads
     * it. */
    if (addr >= 0x2b80 && addr < 0x2c00) {
        static int mw2b80_wen = -1;
        if (mw2b80_wen < 0) mw2b80_wen = calypso_gate("CALYPSO_MEM_WATCH_2B80", 0);
        if (mw2b80_wen) {
            static unsigned mw2b80_wn = 0;
            if (mw2b80_wn < 200) {
                mw2b80_wn++;
                fprintf(stderr, "[c54x] MEM-WATCH-2B80-WR data[0x%04x] 0x%04x -> 0x%04x "
                        "PC=0x%04x insn=%u\n", addr, s->data[addr], val, s->pc,
                        s->insn_count);
            }
        }
    }
    /* STATE-WR: transitions of the handler pointer data[0x3f5e] (L1 state
     * machine) together with the API control cells 0x0908/0x0909 at the same
     * instant. Shows whether the scheduler advances the state or stays pinned on
     * 0x7013 (FB), and from which PC. */
    if (addr == 0x3f5e) {
        static unsigned stwr = 0;
        if (stwr++ < 80)
            fprintf(stderr, "[c54x] STATE-WR data[0x3f5e] 0x%04x -> 0x%04x PC=0x%04x "
                    "api[0x908]=0x%04x api[0x909]=0x%04x api[0x945]=0x%04x insn=%u\n",
                    s->data[0x3f5e], val, s->pc, s->data[0x0908], s->data[0x0909],
                    s->data[0x0945], s->insn_count);
    }

    /* TASKPTR-WR: who writes data[0x0c36], the task pointer CALA'd at 0xb3a5
     * (null -> derail)? Catches DSP writes including indirect ones through AR. */
    if (addr == 0x0c36) {
        static unsigned tw = 0;
        if (tw++ < 60)
            fprintf(stderr, "[c54x] TASKPTR-WR data[0x0c36] 0x%04x -> 0x%04x PC=0x%04x "
                    "XPC=%u AR[0..7]=%04x,%04x,%04x,%04x,%04x,%04x,%04x,%04x insn=%u\n",
                    s->data[0x0c36], val, s->pc, s->xpc,
                    s->ar[0], s->ar[1], s->ar[2], s->ar[3],
                    s->ar[4], s->ar[5], s->ar[6], s->ar[7], s->insn_count);
    }

    /* STATE435B-WR (cap 40): who writes data[0x435b], the state word of the
     * go-live state machine at 0xa4e4, whose bits 0x10/0x40/0x100 gate both its
     * progress and the INTM enable. 0 keeps the state machine blocked. */
    if (addr == 0x435b) {
        static unsigned sw=0;
        if (sw++ < 40)
            fprintf(stderr, "[c54x] STATE435B-WR data[0x435b] 0x%04x -> 0x%04x PC=0x%04x A=0x%06llx insn=%u\n",
                    s->data[0x435b], val, s->pc,
                    (unsigned long long)(s->a & 0xFFFFFFULL), s->insn_count);
    }
    /* WATCH-09BC-WR (cap 40, read-only): the go-live gate at 0xa544
     * (BITF *(0x09bc),1 ; BC 0xa549 if NTC) skips the only native BACC
     * (0xa546-0xa548, through d[0x3fe0]=0x70ce) towards the operational bootstrap
     * 0x7102 CALL 0xd247 unless bit0 of data[0x09bc] is already set. A full scan
     * of PROM0-3 found no clear ORM/SET of that bit (only two ANDM clears at
     * 0x70fc/0x712d, plus an ambiguous site at 0xcf53). This confirms at runtime
     * who writes 0x09bc and with what, and whether bit0 is ever set natively. */
    if (addr == 0x09bc) {
        static unsigned w9 = 0;
        if (w9++ < 40)
            fprintf(stderr, "[c54x] WATCH-09BC-WR data[0x09bc] 0x%04x -> 0x%04x (bit0 %d->%d) "
                    "PC=0x%04x A=0x%06llx insn=%u\n",
                    s->data[0x09bc], val, s->data[0x09bc] & 1, val & 1, s->pc,
                    (unsigned long long)(s->a & 0xFFFFFFULL), s->insn_count);
    }
    /* WATCH-000B-WR (cap 40, read-only): data[0x000b] is tested twice by the
     * background dispatcher (0xdeb6: BITF *(0x000b),0x4000 ; 0xdec2: BITF
     * *(0x000b),0x2000 - bits 14 and 13). Confirms at runtime whether anything
     * writes it natively. If it is never written, BITF always reads 0, TC is
     * always false and that path never unblocks the loop. */
    if (addr == 0x000b) {
        static unsigned wb = 0;
        if (wb++ < 40)
            fprintf(stderr, "[c54x] WATCH-000B-WR data[0x000b] 0x%04x -> 0x%04x "
                    "PC=0x%04x A=0x%06llx insn=%u\n",
                    s->data[0x000b], val, s->pc,
                    (unsigned long long)(s->a & 0xFFFFFFULL), s->insn_count);
    }
    /* WATCH-0810-WR (cap 40): data[0x0810] = db_w->d_ctrl_system (MCU->DSP write
     * page, offset 16), named through osmocom-bb dsp_api.h:75 "Control Register
     * for RESET/RESUME". Bit15 = B_TASK_ABORT (l1_environment.h:365), set by the
     * ARM in l1s_reset(). The DSP tests it at 0xa53c/0xa53f
     * (BITF *(AR1+0x10),0x8000 with AR1=0x0800): bit15 SET falls through
     * a541-a544-a546-0x09bc-0xd247, the operational bootstrap; bit15 CLEAR jumps
     * to 0xa575 and never bootstraps. Says if and when the ARM writes the bit,
     * and its value when the DSP reads it. */
    if (addr == 0x0810) {
        static unsigned w810 = 0;
        if (w810++ < 40)
            fprintf(stderr, "[c54x] WATCH-0810-WR data[0x0810] 0x%04x -> 0x%04x "
                    "(bit15/B_TASK_ABORT %d->%d) PC=0x%04x insn=%u\n",
                    s->data[0x0810], val, !!(s->data[0x0810] & 0x8000), !!(val & 0x8000),
                    s->pc, s->insn_count);
    }
    /* DISPATCH-CELL-RESEED (read-only): data[0x43d8]/[0x3fd4]/[0x4368] measured
     * constant (0xab38/0xc1fa/0xaff9) over the whole observed runtime. Watches
     * writes to prove or refute that they are ever reseeded to anything else. */
    if (addr == 0x43d8 || addr == 0x3fd4 || addr == 0x4368) {
        static unsigned _ndcr = 0;
        if (_ndcr++ < 30)
            fprintf(stderr, "[c54x] DISPATCH-CELL-RESEED data[0x%04x] 0x%04x -> 0x%04x "
                    "PC=0x%04x insn=%u\n", addr, s->data[addr], val, s->pc, s->insn_count);
    }
    /* FBDET-WR (read-only, uncapped): data[0x08F8] = d_fb_det, the NDB field the
     * ARM firmware reads to learn whether the correlator found an FCCH. Critical
     * field, so every write is logged. */
    if (addr == 0x08F8) {
        fprintf(stderr, "[c54x] FBDET-WR data[0x08F8] 0x%04x -> 0x%04x PC=0x%04x insn=%u\n",
                s->data[0x08F8], val, s->pc, s->insn_count);
    }
    /* READY-WR (cap 40): who populates the go-live readiness. 0xaad5 reads
     * *data[0x434e] = *0x4340 and *data[0x434f] = *0x7f75; A=0 makes 0xa4cd skip
     * the enable. Watches both the pointers (0x434e/0x434f) and the targets
     * (0x4340/0x7f75). */
    if (addr == 0x434e || addr == 0x434f || addr == 0x4340 || addr == 0x7f75) {
        static unsigned rw=0;
        if (rw++ < 40)
            fprintf(stderr, "[c54x] READY-WR data[0x%04x] 0x%04x -> 0x%04x PC=0x%04x A=0x%06llx insn=%u\n",
                    addr, s->data[addr], val, s->pc,
                    (unsigned long long)(s->a & 0xFFFFFFULL), s->insn_count);
    }
    /* OVLD-WR (cap 30): writes to the overlay ISR cells 0x0154/0x0155/0x013b. */
    if (addr == 0x0155 || addr == 0x013b || addr == 0x0154) {
        static unsigned ov=0;
        if (ov++ < 30)
            fprintf(stderr, "[c54x] OVLD-WR data[0x%04x] 0x%04x -> 0x%04x PC=0x%04x A=0x%06llx insn=%u\n",
                    addr, s->data[addr], val, s->pc,
                    (unsigned long long)(s->a & 0xFFFFFFULL), s->insn_count);
    }
    /* 3FCD-WR (cap 40): who writes data[0x3fcd], the target of the RET at
     * 0x0157 - the frame handler 0x013b does PSHD *(0x3fcd) then RET, so it jumps
     * to data[0x3fcd]. 0 here derails. */
    if (addr == 0x3fcd || addr == 0x3fce || addr == 0x3fcf) {
        static unsigned f3=0;
        if (f3++ < 40)
            fprintf(stderr, "[c54x] 3FCD-WR data[0x%04x] 0x%04x -> 0x%04x PC=0x%04x A=0x%06llx XPC=%u insn=%u\n",
                    addr, s->data[addr], val, s->pc,
                    (unsigned long long)(s->a & 0xFFFFFFULL), s->xpc, s->insn_count);
    }
    /* DISPPTR-WR: who writes data[0x43c0], the dispatch pointer read by the
     * 0xb40e/0xb40f BACC trampoline, measured at 0xf074 - a table address, hence
     * a derail. */
    if (addr == 0x43c0) {
        static unsigned dw = 0;
        if (dw++ < 60)
            fprintf(stderr, "[c54x] DISPPTR-WR data[0x43c0] 0x%04x -> 0x%04x PC=0x%04x "
                    "A=0x%010llx XPC=%u insn=%u\n",
                    s->data[0x43c0], val, s->pc,
                    (unsigned long long)(s->a & 0xFFFFFFFFFFULL), s->xpc, s->insn_count);
    }

    /* DISPVAL-WR (cap 60): who writes the value 0xf074 (the LUT base, the faulty
     * BACC target) into the dispatch table 0x4300-0x43ff? Names whoever plants
     * the corrupt handler pointer. */
    if (val == 0xf074 && addr >= 0x4300 && addr < 0x4400) {
        static unsigned dvw = 0;
        if (dvw++ < 60)
            fprintf(stderr, "[c54x] DISPVAL-WR data[0x%04x] <- 0xf074 PC=0x%04x "
                    "A=0x%010llx XPC=%u insn=%u\n",
                    addr, s->pc,
                    (unsigned long long)(s->a & 0xFFFFFFFFFFULL), s->xpc, s->insn_count);
    }

    /* SLOT4387-WR (cap 40): does anything ever rewrite the terminal dispatch slot
     * data[0x4387] with a real handler rather than the 0xab38 idle RET? */
    if (addr == 0x4387) {
        static unsigned s4=0;
        if (s4++<12) {
            /* Full context: dump AR/A/B/DP/ST0 and the 6 program words
             * around the store, to reconstruct the index computation of the
             * 0xab10 table. */
            fprintf(stderr, "[c54x] SLOT4387-WR data[0x4387] <- 0x%04x PC=0x%04x insn=%u\n"
                    "         A=0x%06llx B=0x%06llx T=0x%04x DP=0x%03x ST0=0x%04x\n"
                    "         AR0=%04x AR1=%04x AR2=%04x AR3=%04x AR4=%04x AR5=%04x AR6=%04x AR7=%04x\n"
                    "         prog[pc-3..pc+2]=%04x %04x %04x %04x %04x %04x\n",
                    val, s->pc, s->insn_count,
                    (unsigned long long)(s->a & 0xFFFFFFULL), (unsigned long long)(s->b & 0xFFFFFFULL),
                    s->t, (s->st0 & ST0_DP_MASK), s->st0,
                    s->ar[0],s->ar[1],s->ar[2],s->ar[3],s->ar[4],s->ar[5],s->ar[6],s->ar[7],
                    prog_fetch(s,(uint16_t)(s->pc-3)), prog_fetch(s,(uint16_t)(s->pc-2)),
                    prog_fetch(s,(uint16_t)(s->pc-1)), prog_fetch(s,s->pc),
                    prog_fetch(s,(uint16_t)(s->pc+1)), prog_fetch(s,(uint16_t)(s->pc+2)));
        }
    }
    /* SEED-WR (cap 40): who writes the stack base data[0x5ac8..0x5acc], the
     * return address popped by the RET of the no-op handler 0xab38? Never written
     * means the scheduler-return seed is never laid down: the idle path is
     * correct but its init is missing. */
    if (addr >= 0x5ac8 && addr <= 0x5acc) {
        static unsigned sw = 0;
        if (sw++ < 40)
            fprintf(stderr, "[c54x] SEED-WR data[0x%04x] <- 0x%04x PC=0x%04x insn=%u\n",
                    addr, val, s->pc, s->insn_count);
    }
    /* VECWATCH (gate CALYPSO_AR0_DEBUG, cap 40): catches the value 0x71f4 (the
     * go-live trampoline vector) written ANYWHERE, from insn 0.
     *   written at an address other than 0x5ac8 -> stray write (SP/addressing)
     *   never written                           -> reset init is not modelled. */
    if (val == 0x71f4) {
        static int vw_en = -1;
        if (vw_en < 0) vw_en = calypso_gate("CALYPSO_AR0_DEBUG", 0);
        if (vw_en) {
            static unsigned vw = 0;
            if (vw++ < 40)
                fprintf(stderr, "[c54x] VECWATCH 0x71f4 -> data[0x%04x] PC=0x%04x "
                        "SP=0x%04x AR0=0x%04x AR1=0x%04x insn=%u\n",
                        addr, s->pc, s->sp, s->ar[0], s->ar[1], s->insn_count);
        }
    }

    /* === NDB-CTL-WR : trace ARM-side writes to NDB control flags in
     * [data[0x08F8]..data[0x0900]] = d_fb_det, d_fb_mode, a_sync_demod[],
     * d_sb_ext, etc. The firmware writes mode flags before scheduling
     * SB task - finding which flag toggles SB vs FB tells the DSP
     * dispatcher selector. Capped 50. */
    {
        bool ndb_ctl = (addr >= 0x08F8 && addr <= 0x0900);
        /* Filter on s->pc to identify ARM-side writes : ARM has no PC
         * concept here (it writes via MMIO callback), so s->pc reflects
         * the DSP PC at the moment. ARM-side calls land via calypso_dsp_write
         * which does direct s->data[] write, NOT data_write_locked -> so
         * this probe sees only DSP-side writes to NDB. Both paths are
         * useful to discriminate. */
        if (ndb_ctl) {
            static unsigned ndb_log = 0;
            if (ndb_log++ < 50) {
                if (calypso_debug_enabled("NDB-CTL-WR")) fprintf(stderr,
                        "[c54x] NDB-CTL-WR data[0x%04x] <- 0x%04x "
                        "(was 0x%04x) PC=0x%04x insn=%u\n",
                        addr, val, s->data[addr], s->pc, s->insn_count);
            }
        }
    }

    /* SYNC-DEMOD-WR: traces writes to the a_sync_demod cells [0x08FA..0x08FD]
     * (TOA/PM/ANGLE/SNR per the NDB layout, D_TOA=0 D_PM=1 D_ANGLE=2 D_SNR=3) and
     * to a_serv_demod on both read pages (0x0830..0x0833, 0x0844..0x0847).
     * Cap 200 entries. */
    if ((addr >= 0x08FA && addr <= 0x08FD) ||
        (addr >= 0x0830 && addr <= 0x0833) ||
        (addr >= 0x0844 && addr <= 0x0847)) {
        {   /* No PC filter: the writers 0x821a/0x8213/0x8217, once dismissed
             * as stale-AR garbage, may be the real TOA writers the firmware
             * reads. */
            static unsigned sd_log;
            const unsigned LIMIT = 200;
            if (sd_log < LIMIT) {
                const char *name = (addr==0x08FA||addr==0x0830||addr==0x0844) ? "TOA"
                                 : (addr==0x08FB||addr==0x0831||addr==0x0845) ? "PM"
                                 : (addr==0x08FC||addr==0x0832||addr==0x0846) ? "ANGLE"
                                 : "SNR";
                fprintf(stderr,
                        "[c54x] SYNC-DEMOD-WR #%u %s[0x%04x] <- 0x%04x "
                        "(was 0x%04x) PC=0x%04x op=0x%04x "
                        "A=0x%010llx B=0x%010llx "
                        "AR2=%04x AR3=%04x AR4=%04x AR5=%04x insn=%u\n",
                        sd_log, name, addr, val, s->data[addr],
                        s->pc, s->prog[s->pc],
                        (unsigned long long)(s->a & 0xFFFFFFFFFFULL),
                        (unsigned long long)(s->b & 0xFFFFFFFFFFULL),
                        s->ar[2], s->ar[3], s->ar[4], s->ar[5],
                        s->insn_count);
                sd_log++;
                if (sd_log == LIMIT)
                    fprintf(stderr,
                            "[c54x] SYNC-DEMOD-WR log capped at %u\n", LIMIT);
            }
        }
    }

    /* FB-DET-WR: traces writes to d_fb_det (DSP word 0x08F8) by PC. Measured
     * d_fb_det stuck at 0x1255 (96 times) with no varied lock signature, so the
     * question is whether one site always writes 0x1255 or several sites write
     * and the ARM only consumes that one. Each event snapshots the PC, the B
     * accumulator (source of the STH/STL) and the previous opcode (addressing
     * context). Cap 300. */
    if (addr == 0x08F8) {
        static unsigned fbdet_log;
        const unsigned LIMIT = 300;
        if (fbdet_log < LIMIT) {
            fprintf(stderr,
                    "[c54x] FB-DET-WR #%u data[0x08F8] <- 0x%04x "
                    "PC=0x%04x op=0x%04x prev=0x%04x "
                    "B=0x%010llx A=0x%010llx SP=0x%04x "
                    "AR2=%04x AR3=%04x AR4=%04x AR5=%04x insn=%u\n",
                    fbdet_log, val,
                    s->pc, s->prog[s->pc],
                    s->prog[(uint16_t)(s->pc - 1)],
                    (unsigned long long)(s->b & 0xFFFFFFFFFFULL),
                    (unsigned long long)(s->a & 0xFFFFFFFFFFULL),
                    s->sp,
                    s->ar[2], s->ar[3], s->ar[4], s->ar[5],
                    s->insn_count);
            fbdet_log++;
            if (fbdet_log == LIMIT)
                fprintf(stderr, "[c54x] FB-DET-WR log capped at %u\n", LIMIT);
        }
    }

    /* === A_SCH-WR probe : trace DSP writes to a_sch[0..4] in both db_r
     * pages. If DSP never writes these cells, firmware reads stale RAM
     * -> random BSIC in FBSB_CONF. If DSP writes garbage, the SCH demod
     * path is broken upstream. Helps discriminate the SB sync root cause.
     * Capped at 50 logged hits to avoid spam. */
    {
        bool a_sch_p0 = (addr >= 0x0837 && addr <= 0x083B);  /* page 0 a_sch[0..4] */
        bool a_sch_p1 = (addr >= 0x084B && addr <= 0x084F);  /* page 1 a_sch[0..4] */
        if (a_sch_p0 || a_sch_p1) {
            static unsigned a_sch_log = 0;
            if (a_sch_log++ < 50) {
                if (calypso_debug_enabled("A_SCH-WR")) fprintf(stderr,
                        "[c54x] A_SCH-WR data[0x%04x] <- 0x%04x page=%d "
                        "idx=%d PC=0x%04x insn=%u\n",
                        addr, val,
                        a_sch_p0 ? 0 : 1,
                        (int)(addr - (a_sch_p0 ? 0x0837 : 0x084B)),
                        s->pc, s->insn_count);
            }
        }
    }

    /* === BLOB-WR diagnostic for dsp_blobs/ test harness ===
     * Logs writes either targeting scratch [0x2000..0x200F] (dsp-deadbeef
     * etc.) or carrying a known blob signature value. Self-throttled to
     * 5000 hits per process. Zero impact when no blob test is running. */
    {
        static unsigned blob_wr_count = 0;
        bool is_scratch = (addr >= 0x2000 && addr <= 0x200F);
        bool is_magic = (val == 0xCAFE || val == 0xBEEF || val == 0xDEAD ||
                         val == 0x2B2B || val == 0x4906 || val == 0x1B00 ||
                         val == 0x7080 || val == 0x4000);
        if ((is_scratch || is_magic) && blob_wr_count < 5000) {
            blob_wr_count++;
            fprintf(stderr,
                    "[c54x] BLOB-WR data[0x%04x] <- 0x%04x PC=0x%04x insn=%u\n",
                    addr, val, s->pc, s->insn_count);
        }
    }

    /* WATCH-VEC (env CALYPSO_WATCH_VEC): writes to the interrupt vector table
     * (0x0080-0x00FF) and to the 0x013b dispatch (0x0138-0x013c). Separates never
     * written, written then dropped, and corrupted. */
    {
        static int wv = -1;
        if (wv < 0) { const char *e = calypso_getenv("CALYPSO_WATCH_VEC"); wv = (e && *e != 0) ? 1 : 0; }
        if (wv && ((addr >= 0x0080 && addr <= 0x00FF) || (addr >= 0x0138 && addr <= 0x013C))) {
            static unsigned wvn = 0;
            if (wvn++ < 100)
                fprintf(stderr, "[c54x] WATCH-VEC data[0x%04x] <- 0x%04x (was 0x%04x) "
                        "PC=0x%04x op=0x%04x insn=%u\n",
                        addr, val, s->data[addr], s->pc, prog_fetch(s, s->pc), s->insn_count);
        }
    }

    /* The DROM LUT column is read-only (SPRU172C: with PMST.DROM=1 the DSP ROM
     * in data space is read-only).
     *
     * The dispatcher reads a per-task LUT at data[(DP<<7)|0x07] = 0x9187, 0x9207,
     * 0x9287... (always offset 0x07, DP = task number). A walking `STH B,*AR2+`
     * in the RPTB body [0x815E..0x8176] corrupted 0x9207 (0xff72 -> 0xf6b7), so
     * CALAD-A became 0x70c3 - the CALA-A opcode itself in PROM0 - instead of the
     * MAC routine 0x8239. Each self-call pushed one word until SP reached MMR_SP
     * (0x0018) and the push aliased SP itself.
     *
     * Only the offset-0x07 column is protected. A whole-DROM guard breaks the
     * firmware scratch at other offsets (0x00/0x42/0x60...), which then reads
     * back stale and wedges the DSP at 0xebf0.
     *
     * The guard is deliberately NOT conditioned on PMST.DROM: on a 459M-insn run
     * PMST=0x70c4 appeared 149 times - PMST (MMR 0x1D) itself clobbered by the
     * self-CALA at 0x70c3 - which cleared DROM (bit 0x08) and disabled the guard,
     * re-corrupting the LUT: a self-sustaining loop (148838 self-CALAs). The
     * firmware never legitimately runs with DROM=0 (legitimate PMST is
     * 0xffa8/0xffb8, DROM=1 in both). Drops silently, like silicon. */
    if (addr >= 0x9000 && addr <= 0xDFFF && (addr & 0x7F) == 0x07) {
        static unsigned drom_w_attempts = 0;
        if (drom_w_attempts++ < 40) {
            if (calypso_debug_enabled("DROM-W-DROP")) fprintf(stderr,
                    "[c54x] DROM-W-DROP data[0x%04x] <- 0x%04x (was 0x%04x) "
                    "PC=0x%04x insn=%u pmst=0x%04x (LUT col, read-only)\n",
                    addr, val, s->data[addr], s->pc, s->insn_count, s->pmst);
        }
        return;
    }

    /* FBDB-PROBE write to 0x3DC0 (= SARAM flag polled by fc63 BITF).
     * Env CALYPSO_FBDB_PROBE=1. Logs old->new + which bits set, with focus
     * on bit 4 (= 0x0010) since that's the bit fc63 tests via BITF. */
    if (addr == 0x3DC0 && g_fbdb_probe_enabled > 0) {
        fbdb_probe_write_3dc0(addr, s->data[addr], val, s->pc, s->insn_count);
    }
    /* COEFFS-WR: watch-writes over [0x2bc0..0x2bff] (64 words). Measured
     * sequence init -> clear -> use, by cluster:
     *   0x8216 (23 hits)  real coefficients (f320, a660, ...)
     *   0x9ace (64 hits)  partial clear
     *   0x9abf (113 hits) uniform 0x0001 pattern
     * then all-zero from insn ~4M on, still all-zero when the correlator reads at
     * 0x8f51. The per-cluster timing trackers stay out of the helper because they
     * are cluster-specific; watch_write_zone_check factors out the per-PC counter,
     * the throttled log and the summary. */
    if (addr >= 0x2bc0 && addr <= 0x2bff) {
        uint16_t exec_pc = s->last_exec_pc;
        if (exec_pc == 0x8216 || exec_pc == 0x8217 || exec_pc == 0x8218) {
            g_fb_det_timing.last_compute_insn = s->insn_count;
            g_fb_det_timing.last_compute_addr = addr;
        } else if (exec_pc == 0x9ace) {
            g_fb_det_timing.last_clear_insn = s->insn_count;
            g_fb_det_timing.last_clear_addr = addr;
        } else if (exec_pc == 0x9abf) {
            g_fb_det_timing.last_pattern_insn = s->insn_count;
            g_fb_det_timing.last_pattern_addr = addr;
        }
    }
    {
        static WatchWriteState wws_coeffs;
        watch_write_zone_check(s, addr, val, "COEFFS", 0x2bc0, 0x2bff, &wws_coeffs);
    }
    /* INVARIANT (gate CALYPSO_INVARIANTS, default off): correlator pointers.
     * AR4 (write pointer) must take more than 2 distinct values, otherwise it is
     * a 2-word loop. AR5 (I/Q read pointer) must stay inside the buffer
     * [0x2a00..0x2b27], otherwise the correlator reads outside it and the kernel
     * at 0xa076 is never reached. */
    if (addr >= 0x2bc0 && addr <= 0x2bff) {
        static uint32_t pw;
        static uint16_t ar4_seen[8];
        static int ar4_n;
        uint16_t a4 = s->ar[4], a5 = s->ar[5];
        if (ar4_n < 8) {
            int f = 0;
            for (int i = 0; i < ar4_n; i++) { if (ar4_seen[i] == a4) { f = 1; break; } }
            if (!f) ar4_seen[ar4_n++] = a4;
        }
        if (++pw == 2000) {
            calypso_invariant("correlator_ar4_sweeps", ar4_n > 2,
                              "AR4 (write ptr) : %d valeur(s) distincte(s) sur %u writes",
                              ar4_n, pw);
        }
        calypso_invariant("correlator_ar5_in_iq_buffer",
                          a5 >= 0x2a00 && a5 <= 0x2b27,
                          "AR5 (read ptr I/Q) = 0x%04x HORS buffer [0x2a00..0x2b27]", a5);
    }
    /* A_CD-WR: tracks whether the DSP CCCH demod (DSP_TASK_ALLC) writes its
     * results into a_cd[15]. */
    {
        static WatchWriteState wws_a_cd;
        /* Bounds are 0x09D2..0x09E0. Four independent anchors put the base at
         * 0x09D2: (1) dsp_api.h preprocessed with the real l1_environment.h
         * macros gives offset 254 words = 0xFE; (2) ndb.h, reversed from the
         * loaded DSP binary, gives the same 0xFE; (3) B_BLUD is tested on a_cd[0]
         * (prim_tch.c:667) and the DSP writes 0x8000 at 0x09D2; (4) the 12-word
         * block copy targets 0x09D5 while the ARM reads 23 bytes at &a_cd[3].
         * Consistent with d_dsp_state=0x08E2 and NDB_D_FB_DET=0x08F8. A window of
         * 0x09D0..0x09DE instead watches two dead a_ramp words and blinds
         * a_cd[13] and a_cd[14]. */
        watch_write_zone_check(s, addr, val, "A_CD", 0x09d2, 0x09e0, &wws_a_cd);
    }
    /* BLK-SRC - the SOURCE of the block copy, 0x2c3c..0x2c47.
     *
     * The a_cd header is repaired (B_BLUD survives the `or` at 0x9719: measured
     * 86 times 0x8000, never 0x0000 at 0x09d2), but the payload stays null, and it
     * does not come from the header. Firmware disassembly at 0x971e..0x9723:
     *     0x971e  stm #0x2c3c    ; AR2 <- source
     *     0x9720  stm #0x09d5    ; AR3 <- a_cd[3]
     *     0x9722  rpt #0x0b      ; 12 times
     *     0x9723  mvdd           ; *AR2+ -> *AR3+
     * The 12 payload words are therefore a copy of 0x2c3c..0x2c47. If a_cd is
     * null, either that zone is null or nobody writes it.
     *
     *   no line at all   -> nobody writes the zone: the demod does not publish
     *                       its result, look upstream (the 0x81xx loop writes
     *                       through *AR5+, not a fixed address, so follow AR5)
     *   lines with val=0 -> the demod writes, but writes nothing: the fault is
     *                       in the computation, not the transport
     *   non-zero values  -> the zone is good and the `mvdd`, or its timing, is
     *                       at fault.
     *
     * Same helper as A_CD-WR, hence the same emission condition
     * (`total <= 500 || exec_pc != last || delta_insn > 100000`): never derive a
     * rate from the number of lines, read the cumulative SUMMARY.
     * `exec_pc` names the PRECEDING instruction, not the writer; correct for it
     * mentally with `cur_pc`. */
    {
        static WatchWriteState wws_blk_src;
        watch_write_zone_check(s, addr, val, "BLK-SRC", 0x2c3c, 0x2c47, &wws_blk_src);
    }
    /* DISP-TBL - the task dispatch table, 0x43d0..0x43dc.
     *
     * Structure established by disassembling PROM0:
     *   0xb4be  stm #0x43d5 ; ld #0xab35 ; rpt #0x02 ; reada *AR1+
     *           -> copies THREE words prog[0xab35..0xab37] to data[0x43d5..0x43d7]
     *   0xbb00  st *(0x43d8), #0xab38      -> entry 3 = shared RET, set by init
     *   0xb0e5  add #0x43d5, A ; ld *AR3, A ; cala A
     *           -> dispatch on table[idx], guarded by 0 <= idx <= 3
     *   0xb01c  ld *(0x43d8), A ; cala A
     *           -> reads entry 3 specifically
     *
     * So the 35000 `CALAA tgt=0xab38` are NOT a broken dispatcher: 0x43d8 is the
     * entry of task 3, which the loader deliberately does not copy (it copies
     * only 3) and which init sets to a RET. A no-op by design. Tasks 0..2 are
     * dispatched through 0xb0e5..0xb0ec.
     *
     * The index at 0xb0e5 is NOT d_task_md. It comes from accumulator A after the
     * `cala` at 0xb0d7, i.e. the return value of the previous dispatch; do not
     * compare it with the 1/5 the ARM writes at 0x0804.
     *
     * The probe says what the four entries really hold and who writes them,
     * including through indirect addressing that a literal scan of the ROM cannot
     * see. Plausible handler addresses in 0x43d5..0x43d7 mean the table is sound
     * and the index is at fault; 0 or RETs mean the loader is.
     *
     * Same helper as A_CD-WR, hence the same emission condition - read the
     * cumulative SUMMARY, never the number of lines. */
    {
        static WatchWriteState wws_disp_tbl;
        /* Window widened to 0x43d0..0x43dc: 0x43d5..0x43d9 saw only 2 of the 3
         * expected copies and could not say whether the third was missing or
         * landed outside the zone through a pointer offset. */
        watch_write_zone_check(s, addr, val, "DISP-TBL", 0x43d0, 0x43dc, &wws_disp_tbl);
    }
    /* TRAMPO - the vector 30 trampoline, data[0x0158..0x015f].
     *
     * Chain established link by link, by measurement:
     *   1. the DMA raises INT10n at end of transfer   -> 15500 calls, ok
     *   2. the DSP ENTERS vector 30                   -> seen with
     *      CALYPSO_DEBUG=C54X, `vec=30` present, ok
     *   3. the trampoline at 0x0158 runs only 8 times -> not ok
     *   4. its body 0x728a never runs                 -> not ok
     * The DSP queues therefore overflow (PEND: 116 writes at 0x434e/0x434f,
     * TASK: 0) and it raises DSP_ERR_DMA_PEND.
     *
     * The probe says who writes the trampoline cell, when, and with what. The API
     * window overlaps PROGRAM space (CAL000 7.2.1: mixed data program memory, API
     * overlay over the program area), so a data write there installs CODE.
     *   no write              -> the trampoline is never installed and the 8
     *                            executions come from a leftover; find out why
     *                            0xa5cd does not install it.
     *   writes then overwrite -> something destroys it between two takes of the
     *                            vector; the writing PC names it.
     *
     * Same helper as A_CD-WR: same emission condition. Read the cumulative
     * SUMMARY, never the number of lines. */
    {
        static WatchWriteState wws_trampo;
        watch_write_zone_check(s, addr, val, "TRAMPO", 0x0158, 0x015f, &wws_trampo);
        /* A_CD-BY-BURST: correlates a_cd[] writes with the current d_burst_d.
         * A DSP running bursts 0->1->2->3 gives ~25 % of the writes per
         * burst_id; zero writes with burst=3 means the DSP never writes the end
         * of the sequence, which is why the ARM nb_resp bails. */
        if (addr >= 0x09d0 && addr <= 0x09de) {
            static uint64_t a_cd_by_burst[16];
            static uint64_t a_cd_corr_total;
            static uint64_t a_cd_corr_last_log;
            uint16_t b = g_last_d_burst_d & 0xF;
            a_cd_by_burst[b]++;
            a_cd_corr_total++;
            if (a_cd_corr_total - a_cd_corr_last_log >= 1000) {
                a_cd_corr_last_log = a_cd_corr_total;
                if (calypso_debug_enabled("A_CD-BY-BURST")) fprintf(stderr,
                        "[c54x] A_CD-BY-BURST total=%llu "
                        "burst[0]=%llu [1]=%llu [2]=%llu [3]=%llu other=%llu\n",
                        (unsigned long long)a_cd_corr_total,
                        (unsigned long long)a_cd_by_burst[0],
                        (unsigned long long)a_cd_by_burst[1],
                        (unsigned long long)a_cd_by_burst[2],
                        (unsigned long long)a_cd_by_burst[3],
                        (unsigned long long)(a_cd_corr_total -
                                             a_cd_by_burst[0] - a_cd_by_burst[1] -
                                             a_cd_by_burst[2] - a_cd_by_burst[3]));
            }
        }
    }
    /* D_BURST_D probe: watches d_burst_d at 0x0829 (page 0) and 0x083D (page 1)
     * with a per-PC counter, a transition matrix and a histogram.
     *   0,1,2,3 in sequence -> the DSP signals correctly, the bug is ARM-side
     *   0,1,2 but never 3   -> the DSP stalls on the 4th burst
     *   no write at all     -> the DSP never writes this cell
     */
    if (addr == 0x0829 || addr == 0x083D) {
        static uint64_t db_total[2];
        static uint64_t db_per_pc[2][0x10000];
        static uint16_t db_prev[2];
        static uint64_t db_trans[2][16][16];
        static uint64_t db_last_log[2];
        static uint64_t db_last_summary[2];
        int page = (addr == 0x083D) ? 1 : 0;
        uint16_t exec_pc = s->last_exec_pc;
        uint16_t prev_val = db_prev[page];
        uint16_t curr_val = val & 0xF;
        db_total[page]++;
        db_per_pc[page][exec_pc]++;
        if (prev_val < 16 && curr_val < 16) {
            db_trans[page][prev_val][curr_val]++;
        }
        db_prev[page] = curr_val;
        g_last_d_burst_d = curr_val;  /* propagated to A_CD-BY-BURST */
        bool should_log = db_total[page] <= 200
            || (s->insn_count - db_last_log[page]) > 100000;
        if (should_log) {
            if (calypso_debug_enabled("D_BURST_D-WR")) fprintf(stderr,
                    "[c54x] D_BURST_D-WR page=%d #%llu addr=0x%04x val=0x%04x "
                    "exec_pc=0x%04x prev=%u curr=%u insn=%u\n",
                    page, (unsigned long long)db_total[page], addr, val,
                    exec_pc, prev_val, curr_val, s->insn_count);
            db_last_log[page] = s->insn_count;
        }
        if (s->insn_count - db_last_summary[page] >= 5000000) {
            db_last_summary[page] = s->insn_count;
            if (calypso_debug_enabled("D_BURST_D-SUMMARY")) fprintf(stderr,
                    "[c54x] D_BURST_D-SUMMARY page=%d total=%llu trans:",
                    page, (unsigned long long)db_total[page]);
            for (int p = 0; p < 8; p++) {
                for (int c = 0; c < 8; c++) {
                    if (db_trans[page][p][c]) {
                        fprintf(stderr, " %u->%u=%llu",
                                p, c, (unsigned long long)db_trans[page][p][c]);
                    }
                }
            }
            fprintf(stderr, "\n");
        }
    }
    /* D_TASK_D probe: watches d_task_d at 0x0828 (page 0) and 0x083C (page 1),
     * the read side of db_buf_r. The ARM L1 prim_rx_nb reads this field through
     * dsp_api.db_r->d_task_d and bails with puts("EMPTY") when it is 0; 60 EMPTY
     * lines were measured on the deterministic synth=1 bench. Traces who writes
     * it, when and with what, to separate:
     *   - the DSP never touches 0x0828/0x083C
     *   - the DSP writes 0 (clear/init only)
     *   - the DSP writes 24 (DSP_TASK_ALLC) but the ARM reads before the write
     */
    if (addr == 0x0828 || addr == 0x083C) {
        static uint64_t dt_total[2];
        static uint16_t dt_prev[2];
        static uint64_t dt_last_log[2];
        int page = (addr == 0x083C) ? 1 : 0;
        uint16_t exec_pc = s->last_exec_pc;
        uint16_t prev_val = dt_prev[page];
        uint16_t curr_val = val;
        dt_total[page]++;
        dt_prev[page] = curr_val;
        bool should_log = dt_total[page] <= 200
            || (s->insn_count - dt_last_log[page]) > 100000;
        if (should_log) {
            if (calypso_debug_enabled("D_TASK_D-WR")) fprintf(stderr,
                    "[c54x] D_TASK_D-WR page=%d #%llu addr=0x%04x val=0x%04x "
                    "exec_pc=0x%04x prev=0x%04x insn=%u\n",
                    page, (unsigned long long)dt_total[page], addr, val,
                    exec_pc, prev_val, s->insn_count);
            dt_last_log[page] = s->insn_count;
        }
    }
    /* DATA-W-MMR : log every write into the low MMR window (addr <= 0x1F)
     * with full attribution context. Goal : disambiguate the IMR-W trace
     * cascade observed at PC=0x8eb9 (op=0xf3e1) and PC=0x9ad0 (op=0x8192).
     * The writer_kind field tells us *which path* triggered the write
     * (opcode family / IRQ ack / ARM MMIO / resolve_smem side effect).
     * Cap at 200 distinct events to avoid log flood. */
    if (addr <= 0x1F) {
        static unsigned mmrw_log;
        if (mmrw_log++ < 200) {
            const char *wk_name[] = {
                "UNK", "F3", "8x", "77", "76", "PSHM",
                "RET", "IRQ_ACK", "ARM_MMIO", "RES_AR", "OTHER"
            };
            uint8_t wk = s->writer_kind;
            const char *wkn = (wk < sizeof(wk_name)/sizeof(wk_name[0]))
                              ? wk_name[wk] : "??";
            if (calypso_debug_enabled("DATA-W-MMR")) fprintf(stderr,
                    "[c54x] DATA-W-MMR addr=0x%02x val=0x%04x "
                    "exec_pc=0x%04x cur_pc=0x%04x cur_op=0x%04x "
                    "xpc=%d wk=%s "
                    "AR0=%04x AR1=%04x AR2=%04x AR3=%04x "
                    "AR4=%04x AR5=%04x AR6=%04x AR7=%04x "
                    "SP=%04x DP=%d INTM=%d insn=%u\n",
                    addr, val,
                    s->last_exec_pc, s->pc, s->prog[s->pc],
                    s->xpc, wkn,
                    s->ar[0], s->ar[1], s->ar[2], s->ar[3],
                    s->ar[4], s->ar[5], s->ar[6], s->ar[7],
                    s->sp, dp(s),
                    !!(s->st1 & ST1_INTM),
                    s->insn_count);
        }
    }

    /* WATCH-WR-ADDR (CALYPSO_DEBUG=WATCH-WR plus CALYPSO_WATCH_WR_ADDR): logs
     * every write to an arbitrary data address, to trace who writes - or never
     * writes - a SARAM dispatcher-pointer cell that falls to 0, turning CALA into
     * a jump to 0 and landing in the boot stub.
     * Accepts a LIST of addresses separated by commas or spaces
     * (CALYPSO_WATCH_WR_ADDR=0x098b,0x098d), max 8: the DSP task dispatcher
     * (0xddfd/0xde01) tests two masks, d[0x098b] and d[0x098d], both permanently
     * zero, and they must be watched in the SAME run to be comparable. */
    {
        static int wr_init = 0;
        static uint16_t wr_list[8];
        static int wr_n = 0;
        if (!wr_init) {
            wr_init = 1;
            const char *e = calypso_getenv("CALYPSO_WATCH_WR_ADDR");
            if (e && *e) {
                const char *p = e;
                while (*p && wr_n < 8) {
                    while (*p == ',' || *p == ' ' || *p == '\t') p++;
                    if (!*p) break;
                    char *fin = NULL;
                    long v = strtol(p, &fin, 0);
                    if (fin == p) break;          /* nothing parseable: stop */
                    wr_list[wr_n++] = (uint16_t)v;
                    p = fin;
                }
            }
        }
        for (int _i = 0; _i < wr_n; _i++) {
            if (addr == wr_list[_i]) {
                C54_DBG("WATCH-WR",
                    "WATCH-WR data[0x%04x] <- 0x%04x (was 0x%04x) PC=0x%04x "
                    "DP=0x%03x insn=%u",
                    addr, val, s->data[addr], s->pc, (s->st0 & 0x1FF),
                    (unsigned)s->insn_count);
                break;
            }
        }
    }

    /* WATCH-WR-PLAGE: chronological log of EVERY write in the data range
     * [CALYPSO_WATCH_WR_LO, CALYPSO_WATCH_WR_HI], capped at CALYPSO_WATCH_WR_N
     * lines (default 600). Establishes by measurement the order of the writes
     * into the soft-bit buffer 0x2a00-0x2a8d - who writes what, in which order,
     * and whether zeros arrive AFTER the values - instead of inferring it from an
     * aggregate count.
     * PC rule (doc/SONDES.md): the dispatcher has already advanced the PC when
     * the probe observes it, so the line prints pc=<seen> and ecr=<seen-1>, the
     * likely writer.
     * Output goes to stderr unconditionally (not through C54_DBG) so it stays
     * readable without CALYPSO_DEBUG; inert until LO/HI are set. */
    {
        static int wrp_init = 0;
        static long wrp_lo = -1, wrp_hi = -1, wrp_max = 600;
        static long wrp_seen = 0;
        if (!wrp_init) {
            wrp_init = 1;
            const char *lo = calypso_getenv("CALYPSO_WATCH_WR_LO");
            const char *hi = calypso_getenv("CALYPSO_WATCH_WR_HI");
            const char *nn = calypso_getenv("CALYPSO_WATCH_WR_N");
            if (lo && *lo) wrp_lo = strtol(lo, NULL, 0);
            if (hi && *hi) wrp_hi = strtol(hi, NULL, 0);
            if (nn && *nn) wrp_max = strtol(nn, NULL, 0);
            if (wrp_lo >= 0 && wrp_hi < 0) wrp_hi = wrp_lo;
        }
        if (wrp_lo >= 0 && addr >= (uint16_t)wrp_lo && addr <= (uint16_t)wrp_hi) {
            wrp_seen++;
            if (wrp_seen <= wrp_max)
                fprintf(stderr, "[c54x] WR-PLAGE #%ld data[0x%04x] idx=%d <- 0x%04x "
                        "(was 0x%04x) pc=0x%04x ecr=0x%04x insn=%u\n",
                        wrp_seen, addr, (int)(addr - (uint16_t)wrp_lo), val,
                        s->data[addr], s->pc, (uint16_t)(s->pc - 1),
                        (unsigned)s->insn_count);
            else if (wrp_seen == wrp_max + 1)
                fprintf(stderr, "[c54x] WR-PLAGE ... plafond %ld atteint, suite muette\n",
                        wrp_max);
        }
    }

    /* WATCH-3FBE (env CALYPSO_WATCH_3FBE=1, zero cost otherwise): writes over
     * [0x3fb0..0x3fbf], which includes 0x3fbe, the slot popped as 0 at the
     * bootstub entry (insn=3995013). The BSP DMA bypasses data_write_locked
     * (direct s->data[]), so this hook sees only DSP instruction writes
     * (STL/STM/STLM/STH). No write before insn=3995013 means the firmware never
     * pushes to that address: the SP trajectory diverged (a RETD at 0x8ed1 with
     * no matching CALL). */
    if (addr >= 0x3fb0 && addr <= 0x3fbf) {
        static int      w3fbe_enabled = -1;
        static unsigned w3fbe_total = 0;
        if (w3fbe_enabled < 0) {
            const char *e = cdbg_env("WATCH-3FBE");
            w3fbe_enabled = (e && *e == '1') ? 1 : 0;
            if (w3fbe_enabled) {
                fprintf(stderr,
                    "[c54x] WATCH-3FBE enabled — range [0x3fb0..0x3fbf] "
                    "(DSP-side writes only, BSP DMA bypassed)\n");
            }
        }
        if (w3fbe_enabled > 0) {
            w3fbe_total++;
            if (w3fbe_total <= 100 || (w3fbe_total % 5000) == 0) {
                fprintf(stderr,
                    "[c54x] WATCH-3FBE #%u addr=0x%04x val=0x%04x "
                    "(was 0x%04x) PC=0x%04x insn=%u\n",
                    w3fbe_total, addr, val, s->data[addr],
                    s->pc, s->insn_count);
            }
        }
    }

    /* WATCH-WRITE 0x3dd2: the cell 0x75db polls in a loop (37M reads in 15 s).
     * Identifies who writes it, and who does not.
     *   no write        -> a compute block never fires
     *   boot-only write -> init is fine, the steady-state set is missing
     *   periodic writes whose value never matches the test at 0x75db
     *                   -> the bug is in the upstream computation. */
    if (addr == 0x3dd2) {
        static unsigned w3dd2;
        w3dd2++;
        if (w3dd2 <= 100 || (w3dd2 % 1000) == 0) {
            fprintf(stderr,
                    "[c54x] WATCH-WRITE 0x3dd2 #%u <- 0x%04x (was 0x%04x) "
                    "PC=0x%04x insn=%u INTM=%d\n",
                    w3dd2, val, s->data[addr], s->pc, s->insn_count,
                    !!(s->st1 & ST1_INTM));
        }
    }
    /* WATCH-WRITE on the same mailbox slots tracked in data_read. Whoever writes
     * them - DSP or ARM through the api_ram alias - gets logged, so the source of
     * the value the firmware polls can be attributed. */
    if (addr == 0x0ffe || addr == 0x0fff || addr == 0x01F0) {
        static unsigned wcount;
        if (wcount++ < 30) {
            if (calypso_debug_enabled("WATCH-WRITE")) fprintf(stderr,
                    "[c54x] WATCH-WRITE data[0x%04x] <- 0x%04x  (was 0x%04x) "
                    "PC=0x%04x insn=%u\n",
                    addr, val, s->data[addr], s->pc, s->insn_count);
        }
    }
    /* Dispatcher pointer at data[0x3f65] - `LD *(0x3f65),A; CALA A` at
     * DARAM 0x008a-0x008c. When this slot holds 0xfff8/0x0000/garbage the
     * CALA jumps into PROM1 vec or boot stub NOPs and the SP runs away.
     * Trace every write so we can identify who populates / corrupts it. */
    if (addr == 0x3f65) {
        static unsigned dpw;
        if (dpw++ < 100) {
            if (calypso_debug_enabled("DISP-PTR")) fprintf(stderr,
                    "[c54x] DISP-PTR data[0x3f65] <- 0x%04x (was 0x%04x) "
                    "PC=0x%04x insn=%u\n",
                    val, s->data[addr], s->pc, s->insn_count);
        }
    }
    /* Dispatcher poll addresses - log ANY write so we identify the
     * code path that should populate them. Currently 0 PORTR PA=0xF430
     * fires because dispatcher reads 0 here forever. */
    if (addr == 0x4359 || addr == 0x3fab) {
        static unsigned dispw;
        if (dispw++ < 50) {
            if (calypso_debug_enabled("DISP-WRITE")) fprintf(stderr,
                    "[c54x] DISP-WRITE data[0x%04x] <- 0x%04x (was 0x%04x) "
                    "PC=0x%04x insn=%u\n",
                    addr, val, s->data[addr], s->pc, s->insn_count);
        }
    }
    /* CALAD source zone 0x4180-0x41FF - LD-A-TRACE shows the firmware
     * reads 0x4189 (DP=0x83) but our emulation has it as 0. Log every
     * write to this range so we can tell whether (a) anyone is meant to
     * populate it and we missed the path, or (b) DP=0x83 is itself a
     * symptom upstream of an unrelated bug. */
    if (addr >= 0x4180 && addr <= 0x41FF) {
        static unsigned cwz;
        if (cwz++ < 5000) {
            if (calypso_debug_enabled("CALAD-ZONE-W")) fprintf(stderr,
                    "[c54x] CALAD-ZONE-W data[0x%04x] <- 0x%04x (was 0x%04x) "
                    "PC=0x%04x insn=%u\n",
                    addr, val, s->data[addr], s->pc, s->insn_count);
        }
    }
    /* Dedicated watch on 0x4189 - never capped. The LD-A loop reads this
     * slot in the CALAD trap; we want to know if/when *anyone* finally
     * writes a non-zero value, and from which PC. */
    if (addr == 0x4189) {
        fprintf(stderr,
                "[c54x] *** WR-0x4189 *** data[0x4189] <- 0x%04x (was 0x%04x) PC=0x%04x insn=%u\n",
                val, s->data[addr], s->pc, s->insn_count);
    }
    /* DARAM[0x40..0x90] watch - dispatcher flag area. The PROM0 idle dispatcher
     * (0xCC62..0xCC6F) polls data[0x62] and other slots in [0x60..0x70]; forcing
     * data[0x62]=1 was measured to make the DSP escape and reach 0x770c, so this
     * range gates the runtime task pipeline. ARM-side writes go to the API page
     * mirror at +0x0800 (calypso_trx.c calypso_dsp_write) and never to DARAM
     * 0x40..0x90, so any value here comes from DSP stores (ST/STH/STM/...) or
     * stays zero forever. Every write is captured with PC, INTM and insn; INTM
     * separates ISR-context writes from main code. */
    if (addr >= 0x0040 && addr <= 0x0090) {
        static unsigned daram_disp_w;
        if (daram_disp_w++ < 1000) {
            if (calypso_debug_enabled("DISP-FLAG-W")) fprintf(stderr,
                    "[c54x] DISP-FLAG-W data[0x%04x] <- 0x%04x (was 0x%04x) "
                    "PC=0x%04x INTM=%d IFR=0x%04x insn=%u\n",
                    addr, val, s->data[addr], s->pc,
                    !!(s->st1 & ST1_INTM), s->ifr, s->insn_count);
            if (daram_disp_w == 1000) {
                if (calypso_debug_enabled("DISP-FLAG-W")) fprintf(stderr,
                        "[c54x] DISP-FLAG-W log capped at 1000 — pattern visible above\n");
            }
        }
    }
    /* Timer registers (0x0024-0x0026) - before MMR check */
    if (addr == TCR_ADDR) {
        /* TRB: write 1 -> reload TIM from PRD, PSC from TDDR */
        if (val & TCR_TRB) {
            s->data[TIM_ADDR] = s->data[PRD_ADDR];
            s->timer_psc = val & TCR_TDDR_MASK;
        }
        /* Store TCR without TRB (TRB is write-only, always reads 0) */
        s->data[TCR_ADDR] = val & ~TCR_TRB;
        return;
    }
    if (addr == TIM_ADDR) { s->data[TIM_ADDR] = val; return; }
    if (addr == PRD_ADDR) { s->data[PRD_ADDR] = val; return; }

    /* MMR region */
    if (addr < 0x20) {
        /* === BOOT-MMR-WR probe : reach+effect test for boot init STMs ===
         * The DSP boot is supposed to STM #imm into SP, IMR, AR0..7, BK,
         * BRC, PMST shortly after the jump from 0xb418->0x76f8. Observed
         * runtime says SP/IMR/AR4/AR5 never receive their init values, so
         * either (a) PC never reaches the STM, or (b) the STM handler writes
         * to the wrong target. This probe answers BOTH : every write to
         * MMR 0..0x1E during boot phase is logged with PC + opcode + delta,
         * so we can see what got written, when, by what instruction. */
        if (s->insn_count <= 300000) {
            static unsigned bmw_log;
            const unsigned LIMIT = 800;
            if (bmw_log < LIMIT) {
                static const char *names[0x20] = {
                    "IMR","IFR","??02","??03","??04","??05","ST0","ST1",
                    "AL","AH","AG","BL","BH","BG","T","TRN",
                    "AR0","AR1","AR2","AR3","AR4","AR5","AR6","AR7",
                    "SP","BK","BRC","RSA","REA","PMST","XPC","??1F",
                };
                uint16_t old_val = (addr == MMR_IMR) ? s->imr
                                 : (addr == MMR_IFR) ? s->ifr
                                 : (addr == MMR_SP)  ? s->sp
                                 : (addr >= MMR_AR0 && addr <= MMR_AR7) ? s->ar[addr - MMR_AR0]
                                 : s->data[addr];
                fprintf(stderr,
                        "[c54x] BOOT-MMR-WR #%u insn=%u PC=%04x op=%04x "
                        "MMR[%02x %s] %04x → %04x\n",
                        bmw_log, s->insn_count, s->pc, s->prog[s->pc],
                        (unsigned)addr, names[addr],
                        old_val, val);
                bmw_log++;
                if (bmw_log == LIMIT) {
                    fprintf(stderr,
                            "[c54x] BOOT-MMR-WR log capped at %u\n", LIMIT);
                }
            }
        }
        switch (addr) {
        case MMR_IMR:
            /* IMR-ARM (read-only, cap 80): full history of IMR writes
             * (old -> new) with bit12 = vec28 (scheduler), bit3 = vec19,
             * bit5 = vec21. Measured IMR=0x52fd (bit12 set) cleared by 0xb37e,
             * so this names who writes 0x52fd and whether a re-arm follows. */
            if ((uint16_t)val != s->imr) {
                static unsigned ia = 0;
                if (ia++ < 80)
                    fprintf(stderr, "[c54x] IMR-ARM 0x%04x -> 0x%04x (b12/vec28=%d "
                            "b3/vec19=%d b5/vec21=%d) PC=0x%04x op=0x%04x insn=%u\n",
                            s->imr, (uint16_t)val,
                            !!(val&(1<<12)), !!(val&(1<<3)), !!(val&(1<<5)),
                            s->pc, s->prog[s->pc], s->insn_count);
            }
            if (val != s->imr) {
                static unsigned imr_log = 0;
                /* Always log transitions TO zero (mask-everything) - that
                 * is the cascade root suspected in 2026-05-08 v2 diag :
                 * IMR=0 -> INT3 IFR pending forever -> RPTB at 0xe9ac never
                 * exits. We need the PC + opcode of every IMR=0 write,
                 * uncapped, so we can identify the buggy code path. */
                bool to_zero = (val == 0);
                if (imr_log++ < 50 || to_zero) {
                    if (calypso_debug_enabled("IMR-W")) fprintf(stderr,
                            "[c54x] IMR-W %s 0x%04x → 0x%04x PC=0x%04x "
                            "op=0x%04x prev_op=0x%04x SP=0x%04x INTM=%d "
                            "AR0=0x%04x AR1=0x%04x AR2=0x%04x AR3=0x%04x "
                            "AR4=0x%04x AR5=0x%04x AR6=0x%04x AR7=0x%04x "
                            "B=0x%010llx insn=%u\n",
                            to_zero ? "*ZERO*" : "      ",
                            s->imr, val, s->pc,
                            s->prog[s->pc],
                            s->prog[(uint16_t)(s->pc - 1)],
                            s->sp,
                            !!(s->st1 & ST1_INTM),
                            s->ar[0], s->ar[1], s->ar[2], s->ar[3],
                            s->ar[4], s->ar[5], s->ar[6], s->ar[7],
                            (unsigned long long)(s->b & 0xFFFFFFFFFFULL),
                            s->insn_count);
                }
            }
            {
                /* The IMR bit9 toggle (0xddf9 ANDM / 0xde84 ORM) is driven by
                 * the TPU - dynamic masking per burst window - and must not be
                 * pinned statically: IMR stays whatever the firmware and TPU
                 * write. The only re-arm kept is the dynamic bit5/BRINT0 fix
                 * (KEEP-IMR in calypso_c54x.c), which restores bit5 only when it
                 * drops. */
            }
            s->imr = val; return;
        case MMR_IFR: {
            /* IFR-CLEAR-W (unconditional, capped): does the go-live code write
             * IFR directly (write-1-to-clear) on bit5 (BRINT0) / bit12 (frame)
             * without ever dispatching - a software acknowledge that throws the
             * pending interrupt away instead of servicing it? */
            static unsigned _ifrw = 0;
            if ((val & 0x1020) && _ifrw < 100) {
                _ifrw++;
                fprintf(stderr, "[c54x] IFR-CLEAR-W #%u val=0x%04x (clears bit5=%d bit12=%d) "
                        "ifr_before=0x%04x -> after=0x%04x PC=0x%04x insn=%u\n",
                        _ifrw, val, !!(val & 0x20), !!(val & 0x1000),
                        s->ifr, (uint16_t)(s->ifr & ~val), s->pc, s->insn_count);
            }
            {   /* IFR-CLEAR-WHO: reports the pending interrupts this clear loses. */
                uint16_t _av = s->ifr;
                s->ifr &= (uint16_t)~val;
                uint16_t _pd = (uint16_t)(_av & ~s->ifr & s->imr);
                if (_pd) { static unsigned _n = 0;
                    if (_n++ < 40 || (_n % 5000) == 0)
                        fprintf(stderr, "[c54x] IFR-CLEAR-WHO #%u site=mmio-write "
                                "perdus=0x%04x IFR 0x%04x->0x%04x IMR=0x%04x "
                                "INTM=%d PC=0x%04x insn=%u\\n", _n, _pd, _av,
                                s->ifr, s->imr, (s->st1 & ST1_INTM) ? 1 : 0,
                                s->pc, s->insn_count);
                }
            }
            return;
        }
        case MMR_ST0:  s->st0 = val;
            /* Whole-ST0 restore (POPM ST0 / STLM): the non-LDP path that
             * changes DP. This is where DP becomes 0x087. */
            g_last_st0w_pc = s->pc; g_last_st0w_val = val;
            g_last_st0w_op = prog_fetch(s, s->pc); g_last_st0w_xpc = s->xpc;
            g_last_st0w_prev = g_prev_pc;
            st0_ring_rec(s, val, 'p'); /* pop/write ST0 */
            if (g_orphan_on > 0 && (s->pc == 0xf48b || s->pc == 0x7737 || (val & 0x1FF) == 0x124)) {
                /* POPM ST0 at 0xf48b: the slot just popped is data[sp-1]. Look
                 * up its last writer in the stack ring. NO-WRITER means a stale
                 * slot, i.e. SP misaligned by a POP with no matching PUSH. */
                uint16_t slot = (uint16_t)(s->sp - 1);
                fprintf(stderr, "[c54x] ORPHAN@%04x SP=0x%04x slot=0x%04x val=0x%04x(DP=%03x)",
                        s->pc, s->sp, slot, val, (unsigned)(val & 0x1FF));
                int found = 0;
                unsigned rn = g_stkw_idx < STKW_RING_N ? g_stkw_idx : STKW_RING_N;
                for (unsigned i = 0; i < rn; i++) {
                    StkwEv *e = &g_stkw_ring[(g_stkw_idx - 1 - i) % STKW_RING_N];
                    if (e->addr == slot) {
                        fprintf(stderr, "  WRITER@%04x op=%04x val=%04x", e->pc, e->op, e->val);
                        found = 1; break;
                    }
                }
                if (!found)
                    fprintf(stderr, "  NO-WRITER → slot STALE → SP désaligné (POP sans PUSH)");
                fprintf(stderr, " insn=%u\n", s->insn_count);
                /* SP event ring: recent pushes and pops, which expose the
                 * unbalanced RET-family return (pop delta > 0 with no matching
                 * push) that shifts SP. */
                fprintf(stderr, "[c54x]   ORPHAN-SP-RING (anciens→récents, pc:op±delta) :");
                for (int k = 28; k >= 1; k--) {
                    struct sp_evt *e = &g_spring[(g_spring_idx - k) & 63];
                    fprintf(stderr, " %04x:%04x%+d", e->pc, e->op, e->delta);
                }
                fprintf(stderr, "\n");
            }
            return;
        case MMR_ST1:  s->st1 = val; return;
        case MMR_AL:   s->a = (s->a & ~0xFFFF) | val; return;
        case MMR_AH:   s->a = (s->a & ~((int64_t)0xFFFF << 16)) | ((int64_t)val << 16); return;
        case MMR_AG:   s->a = (s->a & 0xFFFFFFFF) | ((int64_t)(val & 0xFF) << 32); return;
        case MMR_BL:   s->b = (s->b & ~0xFFFF) | val; return;
        case MMR_BH:   s->b = (s->b & ~((int64_t)0xFFFF << 16)) | ((int64_t)val << 16); return;
        case MMR_BG:   s->b = (s->b & 0xFFFFFFFF) | ((int64_t)(val & 0xFF) << 32); return;
        case MMR_T:    s->t = val; return;
        case MMR_TRN:  s->trn = val; return;
        case MMR_AR0: case MMR_AR1: case MMR_AR2: case MMR_AR3:
        case MMR_AR4: case MMR_AR5: case MMR_AR6: case MMR_AR7:
            ar_write_track(s, addr - MMR_AR0, val);  /* unified probe AR0..AR7 */
            s->ar[addr - MMR_AR0] = val; return;
        case MMR_SP:
            if (val >= 0x0800 && val < 0x0900) {
                if (calypso_debug_enabled("SP-GUARD")) fprintf(stderr,
                        "[c54x] SP-GUARD: refused MMR_SP write 0x%04x "
                        "(API mailbox); keeping 0x%04x PC=0x%04x\n",
                        val, s->sp, s->pc);
                return;
            }
            sp_abs_track(s, val, 0);  /* site 0: MMR_SP via STL/STM/STLM */
            s->sp = val;
            return;
        case MMR_BK:
            /* Who writes BK? BK=0 breaks circular addressing and sends AR2
             * running away (0xfa98/0xf17c); this names the writer and the value. */
            {
                static uint32_t bkw_n = 0;
                if (bkw_n < 40) {
                    fprintf(stderr, "[c54x] BK-WR (MMR) 0x%04x→0x%04x PC=0x%04x op=0x%04x "
                            "%s insn=%u\n", s->bk, val, s->pc, prog_fetch(s, s->pc),
                            (val == 0) ? "<<< BK=0 (casse circular!)" : "", s->insn_count);
                    bkw_n++;
                }
            }
            s->bk = val; return;
        case MMR_BRC:  s->brc = val; return;
        case MMR_RSA:  s->rsa = val; return;
        case MMR_REA:  s->rea = val; return;
        case MMR_PMST:
            {
                /* PMST-WR (read-only, ungated): separates "relocated IPTR 0x140
                 * lost" from "never emitted". First 300 writes, plus always when
                 * IPTR is 0x140. */
                {
                    static unsigned pmst_n = 0;
                    uint16_t niptr = (val >> PMST_IPTR_SHIFT) & 0x1FF;
                    if (pmst_n < 300 || niptr == 0x140) {
                        pmst_n++;
                        fprintf(stderr, "[c54x] PMST-WR #%u val=0x%04x IPTR=0x%03x PC=0x%04x insn=%u%s\n",
                                pmst_n, val, niptr, s->pc, s->insn_count,
                                niptr == 0x140 ? "  <<< IPTR=0x140 EMITTED" : "");
                    }
                }
                static unsigned pmst_wr_attempts = 0;
                if (pmst_wr_attempts++ < 100)
                    C54_LOG("PMST WR attempt #%u: val=0x%04x cur=0x%04x PC=0x%04x insn=%u",
                            pmst_wr_attempts, val, s->pmst, s->pc, s->insn_count);
            }
            if (val != s->pmst) {
                uint16_t old_iptr = (s->pmst >> PMST_IPTR_SHIFT) & 0x1FF;
                uint16_t new_iptr = (val >> PMST_IPTR_SHIFT) & 0x1FF;
                {
                    static unsigned pmst_log = 0;
                    if (pmst_log++ < 100)
                        C54_LOG("PMST change 0x%04x → 0x%04x (IPTR=0x%03x→0x%03x OVLY=%d) PC=0x%04x SP=0x%04x insn=%u #%u/100",
                                s->pmst, val, old_iptr, new_iptr, !!(val & PMST_OVLY), s->pc, s->sp, s->insn_count, pmst_log);
                }

                static uint16_t last_dumped_iptr = 0xFFFF;
                static unsigned vecdump_count = 0;
                /* Cap at 8 dumps total - firmware may oscillate between 2-3
                 * IPTR values thousands of times during a session, and each
                 * dump emits 32 fprintf lines. Without cap : 250k+ log lines
                 * = saturates host I/O = bridge stops emitting CLK INDs =
                 * BTS shutdown "No more clock from transceiver". */
                if (new_iptr != last_dumped_iptr && vecdump_count < 8) {
                    vecdump_count++;
                    last_dumped_iptr = new_iptr;
                    uint32_t base = (uint32_t)new_iptr << 7;
                    uint16_t saved_pmst = s->pmst;
                    s->pmst = val;
                    C54_LOG("VECDUMP IPTR=0x%03x base=0x%04x (32 vectors) #%u/8:",
                            new_iptr, (uint16_t)base, vecdump_count);
                    for (int vec = 0; vec < 32; vec++) {
                        uint32_t a = base + vec * 4;
                        uint16_t w0 = prog_read(s, a + 0);
                        uint16_t w1 = prog_read(s, a + 1);
                        uint16_t w2 = prog_read(s, a + 2);
                        uint16_t w3 = prog_read(s, a + 3);
                        fprintf(stderr,
                                "[c54x] vec %2d @ 0x%04x : %04x %04x %04x %04x\n",
                                vec, (uint16_t)a, w0, w1, w2, w3);
                    }
                    s->pmst = saved_pmst;
                }
            }
            s->pmst = val; return;
        case MMR_XPC:
            {
                static int xpc_log = 0;
                if (xpc_log++ < 50)
                    C54_LOG("MMR_XPC WR val=0x%04x (was %d) PC=0x%04x SP=0x%04x insn=%u",
                            val, s->xpc, s->pc, s->sp, s->insn_count);
            }
            s->xpc = val & 3;
            return;
        default: return;
        }
    }

    /* DMA controller (calypso_dma.c): intercepts before the older decoding below
     * and returns true when it has handled the write. Off by default
     * (CALYPSO_DMA), in which case it returns false and nothing changes.
     * Warning: the two decodings disagree on the mapping - SPRU131 puts DMPREC at
     * 0x54, the older code puts DMSA there. See calypso_dma.h. */
    if (calypso_dma_mmr_write(s, addr, val)) {
        return;
    }

    /* DMA sub-register bank (C54x DMA controller).
     * DMSA (0x0054): sets the sub-register address.
     * DMSDI (0x0055): writes sub-register data, auto-increments DMSA.
     * DMSDN (0x0057): writes sub-register data, no auto-increment.
     * DMA channel 0 sub-registers (BSP receive DMA):
     *   sub 0x00=DMSRC0, 0x01=DMDST0, 0x02=DMCTR0, 0x03=DMMCR0 */
    if (addr == 0x0054) {
        s->dma_subaddr = val;
        s->data[0x0054] = val;
        return;
    }
    if (addr == 0x0055 || addr == 0x0057) {
        uint16_t sa = s->dma_subaddr;
        if (sa < 24) {  /* 6 channels x 4 regs */
            s->dma_subregs[sa] = val;
            int ch = sa / 4;
            int reg = sa % 4;
            static const char *rnames[] = {"SRC","DST","CTR","MCR"};
            C54_LOG("DMA ch%d %s = 0x%04x (sub 0x%02x) PC=0x%04x",
                    ch, rnames[reg], val, sa, s->pc);
        }
        s->data[addr] = val;
        if (addr == 0x0055) s->dma_subaddr++;  /* auto-increment */
        return;
    }

    /* McBSP sub-register bank (serial port extended config).
     * SPSA (0x0038): sub-address. SPSD (0x0039): sub-data. */
    if (addr == 0x0038 || addr == 0x0039) {
        if (addr == 0x0038) s->spsa = val;
        else {
            C54_LOG("McBSP sub[0x%02x] = 0x%04x PC=0x%04x", s->spsa, val, s->pc);
        }
        s->data[addr] = val;
        return;
    }

    /* API RAM (shared with ARM) */
    if (addr >= C54X_API_BASE && addr < C54X_API_BASE + C54X_API_SIZE) {
        uint16_t woff = addr - C54X_API_BASE;
        /* SAM / HOM arbitration - CAL000 7.2.1, CAL207 9.1.
         *
         * "In HOM mode (Host Only Mode), the API RAM is dedicated to external
         * access under the control of either the ARM or the DMA controller." In
         * HOM the DSP therefore has no access to this window; the model had no
         * notion of it and both sides wrote unarbitrated.
         *
         * Measured: the DSP firmware switches HOM<->SAM ONCE PER FRAME
         * (API_CONF=0x0002 at 0xa693, back to 0x0000 at 0xa4e7).
         *
         * Two deliberately separate stages:
         *   - observation (CALYPSO_API_HOM_WATCH, default 1): counts and logs the
         *     DSP writes made during HOM, changing nothing.
         *   - enforcement (CALYPSO_API_HOM_STRICT, default 0): DROPS the write,
         *     as the silicon would. A real behaviour change, hence opt-in and to
         *     be validated under load.
         * The documentation does not say what the DSP reads in HOM, so the read
         * path is left alone. */
        {
            static int watch = -1, strict = -1;
            if (watch < 0) {
                watch  = calypso_gate("CALYPSO_API_HOM_WATCH", 1);
                strict = calypso_gate("CALYPSO_API_HOM_STRICT", 0);
                if (strict)
                    fprintf(stderr, "[c54x] API_HOM_STRICT=1 : les ecritures DSP "
                            "dans la fenetre API sont ABANDONNEES pendant HOM "
                            "(CAL000 §7.2.1). Changement de comportement — a "
                            "valider sous charge.\n");
            }
            if ((watch || strict) && calypso_xio_api_hom()) {
                static unsigned long long n_hom = 0;
                if (n_hom++ == 0 || (n_hom % 5000) == 0)
                    fprintf(stderr, "[c54x] API-HOM : ecriture DSP dans la fenetre "
                            "API pendant HOM #%llu — data[0x%04x] <- 0x%04x "
                            "PC=0x%04x (%s)\n", n_hom, addr, val, s->pc,
                            strict ? "ABANDONNEE" : "laissee passer, observation");
                if (strict)
                    return;
            }
        }
        if (s->api_ram)
            s->api_ram[woff] = val;
        {   /* FBDET-API, DSP side: api_ram writes to d_fb_det / a_sync_demod. */
            static int _fa = -1; static unsigned _fan = 0;
            if (_fa < 0) _fa = calypso_gate("CALYPSO_FBDET_API", 0);
            if (_fa && woff >= 0xF8 && woff <= 0xFD && _fan < 40) {
                _fan++;
                fprintf(stderr, "[c54x] FBDET-API DSP api_ram[0x%02x] (mot 0x%04x, %s)"
                        " <- 0x%04x PC=0x%04x insn=%u\n", woff, addr,
                        woff == 0xF8 ? "d_fb_det" : "a_sync_demod",
                        val, s->pc, s->insn_count);
            }
        }
        /* === DSP->ARM STATUS / DEMOD probe (CCCH chain tracing) ===
         * Track DSP writes to the four critical mailbox regions :
         *   (1) a_pm[3]  + a_serv_demod[4]  on read page 0  : woff 0x30..0x36
         *   (2) a_pm[3]  + a_serv_demod[4]  on read page 1  : woff 0x44..0x4A
         *   (3) a_cd[15] CCCH demod result (CLAUDE.md DWARF) : woff 0x1C2..0x1D0
         *   (4) a_cd[15] CCCH demod result (shunt DWARF v2)  : woff 0x1D2..0x1E0
         * If a_serv_demod_WP* never appears -> DSP never advances past
         * cell-search. If a_cd ranges stay silent -> CCCH demod never
         * publishes (= bridge GMSK not converging, or task-md chain
         * never reaches CCCH). */
        {
            bool in_serv = (woff >= 0x0030 && woff <= 0x0036)
                        || (woff >= 0x0044 && woff <= 0x004A);
            bool in_acd  = (woff >= 0x01C2 && woff <= 0x01E0);
            if (in_serv || in_acd) {
                static unsigned cd_log;
                const unsigned LIMIT = 400;
                if (cd_log < LIMIT) {
                    const char *tag;
                    if (woff >= 0x0030 && woff <= 0x0032) tag = "A_PM_WP0";
                    else if (woff >= 0x0033 && woff <= 0x0036) tag = "A_SERV_DEMOD_WP0";
                    else if (woff >= 0x0044 && woff <= 0x0046) tag = "A_PM_WP1";
                    else if (woff >= 0x0047 && woff <= 0x004A) tag = "A_SERV_DEMOD_WP1";
                    else tag = "A_CD";
                    fprintf(stderr,
                            "[c54x] DSP-API-WR #%u %s woff=0x%04x val=0x%04x "
                            "PC=0x%04x AR2=%04x AR3=%04x AR4=%04x AR5=%04x insn=%u\n",
                            cd_log, tag, woff, val, s->pc,
                            s->ar[2], s->ar[3], s->ar[4], s->ar[5],
                            s->insn_count);
                    cd_log++;
                    if (cd_log == LIMIT) {
                        fprintf(stderr,
                                "[c54x] DSP-API-WR log capped at %u\n", LIMIT);
                    }
                }
            }
        }
        /* Notify the ARM-side mailbox watcher (calypso_trx) so it can
         * pulse IRQ_API, mirror to dsp_ram, and run the d_fb_det hook.
         * Without this, DSP writes to NDB cells are invisible to ARM. */
        if (s->api_write_cb)
            s->api_write_cb(s->api_write_cb_opaque, woff, val);
        /* Stack-corruption watch: stack push landing in the NDB
         * mailbox region [0x0800..0x08FF]. Only fires when SP has
         * already been corrupted into that range. */
        if (addr == s->sp && addr >= 0x0800 && addr < 0x0900) {
            if (calypso_debug_enabled("STACK-IN-NDB")) fprintf(stderr,
                    "[c54x] STACK-IN-NDB addr=0x%04x val=0x%04x SP=0x%04x "
                    "PC=0x%04x insn=%u op[pc-2..pc+1]=%04x %04x %04x %04x\n",
                    addr, val, s->sp, s->pc, s->insn_count,
                    s->prog[(uint16_t)(s->pc - 2)],
                    s->prog[(uint16_t)(s->pc - 1)],
                    s->prog[s->pc],
                    s->prog[(uint16_t)(s->pc + 1)]);
        }
        /* Always log writes to d_dsp_page (0x08D4 ; 0x08E2 = d_dsp_state) */
        if (addr == 0x08D4) {
            C54_LOG("DSP WR d_dsp_page = 0x%04x PC=0x%04x insn=%u op[pc-2..pc+1]=%04x %04x %04x %04x",
                    val, s->pc, s->insn_count,
                    s->prog[(uint16_t)(s->pc - 2)],
                    s->prog[(uint16_t)(s->pc - 1)],
                    s->prog[s->pc],
                    s->prog[(uint16_t)(s->pc + 1)]);
        }

        /* d_spcx_rif (NDB word 2 = DSP data 0x08D6) - BSP serial port config */
        if (addr == 0x08D6) {
            C54_LOG("DSP WR d_spcx_rif = 0x%04x PC=0x%04x insn=%u op[pc-2..pc+1]=%04x %04x %04x %04x",
                    val, s->pc, s->insn_count,
                    s->prog[(uint16_t)(s->pc - 2)],
                    s->prog[(uint16_t)(s->pc - 1)],
                    s->prog[s->pc],
                    s->prog[(uint16_t)(s->pc + 1)]);
        }
        /* Full a_sync_demod + d_fb_mode write watch: every cell, no PC filter,
         * so real fb-det writes and stomp candidates are both caught. Writes from
         * PC=0x06xx are tagged [STOMP?] for easy grepping.
         * Three discriminating outcomes for d_fb_mode - the binary "FB matched"
         * flag the ARM actually tests:
         *   never written  -> the "FB confirmed" path is never reached
         *   written as 0   -> the DSP scans but never crosses the threshold
         *   written non-0 while the ARM reads 0 -> a coherence bug. */
        if (addr == 0x08F9 || addr == 0x08FA ||
            addr == 0x08FB || addr == 0x08FC || addr == 0x08FD) {
            static unsigned ts_log[5] = {0};
            static uint16_t prev_d_fb_mode = 0xFFFF;
            int idx = (addr == 0x08F9) ? 0 :
                      (addr == 0x08FA) ? 1 :
                      (addr == 0x08FB) ? 2 :
                      (addr == 0x08FC) ? 3 : 4;
            const char *name = (idx == 0) ? "d_fb_mode"  :
                               (idx == 1) ? "a_sync_TOA" :
                               (idx == 2) ? "a_sync_PM"  :
                               (idx == 3) ? "a_sync_ANG" : "a_sync_SNR";
            ts_log[idx]++;
            bool transition = (idx == 0) &&
                              (prev_d_fb_mode != 0xFFFF) &&
                              (prev_d_fb_mode != val) &&
                              (val != 0 || prev_d_fb_mode != 0);
            bool stomp_zone = (s->pc >= 0x0600 && s->pc < 0x0700);
            bool log_it = transition ||
                          (idx == 0 && val != 0) ||
                          (val != 0 && ts_log[idx] <= 50) ||
                          (ts_log[idx] % 1000) == 0;
            if (log_it) {
                C54_LOG("DSP WR %s = 0x%04x (s=%d) PC=0x%04x%s insn=%u #%u%s",
                        name, val, (int)(int16_t)val, s->pc,
                        stomp_zone ? " [STOMP?]" : "",
                        s->insn_count, ts_log[idx],
                        transition ? " *TRANSITION*" : "");
            }
            if (idx == 0) prev_d_fb_mode = val;
        }
        /* d_fb_det (NDB word 36 = DSP data 0x08F8). The firmware FB-det path
         * treats the correlator output as Q15 signed. Logs every write (thinned
         * past 200) and dumps the adjacent NDB cells [0x08F0..0x0900] so the
         * correlator output, the flag and the a_sync_demod fields can be read
         * together. */
        if (addr == 0x08F8) {
            static unsigned fbd_log = 0;
            /* Filter out stack-stomp at d_fb_det: only PCs known to be
             * actual fb-det correlator stores (0x8d33, 0x8eb9, 0x8f51) get
             * the full per-write log + NDB+DARAM dumps. Other PCs (e.g.
             * 0xb906 push site, 0x7763/0x7764 SP-overflow) get a counted
             * one-line tag so we don't lose visibility on them, but they
             * stop polluting the watch stream. */
            bool real_fbdet = (s->pc == 0x8d33 || s->pc == 0x8eb9 ||
                               s->pc == 0x8f51);
            /* FBDET-DIVERSITY: count distinct values per 1M-insn window.
             * 1 = DSP pegged on stale data. >5 = real scan. Discriminates
             * "BSP delivers fresh I/Q" from "DSP recorrelates same window". */
            if (real_fbdet) {
                static uint16_t recent_vals[8] = {0};
                static unsigned next_window = 1000000;
                static int n_distinct = 0;
                int seen = 0;
                for (int i = 0; i < 8; i++) {
                    if (recent_vals[i] == val) { seen = 1; break; }
                }
                if (!seen) {
                    recent_vals[n_distinct & 7] = val;
                    n_distinct++;
                }
                if (s->insn_count >= next_window) {
                    C54_LOG("FBDET-DIVERSITY window=%uM distinct=%d",
                            next_window / 1000000, n_distinct);
                    n_distinct = 0;
                    for (int i = 0; i < 8; i++) recent_vals[i] = 0;
                    next_window = (s->insn_count / 1000000 + 1) * 1000000;
                }
            }
            if (real_fbdet && (fbd_log < 200 || (fbd_log % 1000) == 0)) {
                C54_LOG("DSP WR d_fb_det = 0x%04x (s=%d) PC=0x%04x insn=%u op[pc-2..pc+1]=%04x %04x %04x %04x",
                        val, (int)(int16_t)val, s->pc, s->insn_count,
                        s->prog[(uint16_t)(s->pc - 2)],
                        s->prog[(uint16_t)(s->pc - 1)],
                        s->prog[s->pc],
                        s->prog[(uint16_t)(s->pc + 1)]);
                C54_LOG("  NDB[0x08F0..0x0900]: %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x",
                        s->data[0x08F0], s->data[0x08F1], s->data[0x08F2], s->data[0x08F3],
                        s->data[0x08F4], s->data[0x08F5], s->data[0x08F6], s->data[0x08F7],
                        val,             s->data[0x08F9], s->data[0x08FA], s->data[0x08FB],
                        s->data[0x08FC], s->data[0x08FD], s->data[0x08FE], s->data[0x08FF],
                        s->data[0x0900]);
                if (fbd_log < 5) {
                    C54_LOG("  DARAM[0x3FB0..0x3FBF]: %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x",
                            s->data[0x3FB0], s->data[0x3FB1], s->data[0x3FB2], s->data[0x3FB3],
                            s->data[0x3FB4], s->data[0x3FB5], s->data[0x3FB6], s->data[0x3FB7],
                            s->data[0x3FB8], s->data[0x3FB9], s->data[0x3FBA], s->data[0x3FBB],
                            s->data[0x3FBC], s->data[0x3FBD], s->data[0x3FBE], s->data[0x3FBF]);
                }
            } else if (!real_fbdet) {
                static unsigned other_pc_count = 0;
                other_pc_count++;
                if (other_pc_count == 1 || other_pc_count == 100 ||
                    other_pc_count == 10000 || other_pc_count == 1000000) {
                    C54_LOG("d_fb_det NON-FBDET-PC write #%u val=0x%04x PC=0x%04x SP=0x%04x",
                            other_pc_count, val, s->pc, s->sp);
                }
            }
            /* d_fb_det zero-override trace. Measured race window: the DSP
             * writes a high SNR (e.g. 0x7902, 0x7766) at the fb-det PCs, then
             * something zeroes d_fb_det before the ARM reads it; the ARM sees 200
             * times 0x0000, finds no FB and retries L1CTL_FBSB_REQ forever.
             * Captures EVERY write of 0 to 0x08F8 with full context, to identify
             * the zeroing PCs and reconstruct the condition (threshold check,
             * post-correlation reset, error path). Cap 200. */
            if (val == 0) {
                static unsigned zero_log = 0;
                if (zero_log < 200) {
                    C54_DBG("FBDET", "D_FB_DET ZERO-WR #%u PC=0x%04x op=0x%04x prev=0x%04x "
                            "A=%010llx B=%010llx T=0x%04x ST0=0x%04x ST1=0x%04x insn=%u",
                            zero_log + 1,
                            s->pc, s->prog[s->pc],
                            s->data[0x08F8],
                            (unsigned long long)(s->a & 0xFFFFFFFFFFLL),
                            (unsigned long long)(s->b & 0xFFFFFFFFFFLL),
                            s->t, s->st0, s->st1, s->insn_count);
                    zero_log++;
                }
            }
            /* Transition trace : non-zero -> zero (the override moment).
             * Logs whenever d_fb_det was non-zero just before this write
             * but the new write makes it zero. Cap 100. */
            if (val == 0 && s->data[0x08F8] != 0) {
                static unsigned override_log = 0;
                if (override_log < 100) {
                    C54_DBG("FBDET", "D_FB_DET OVERRIDE #%u prev=0x%04x → 0 PC=0x%04x op=0x%04x "
                            "A=%010llx ST0=0x%04x insn=%u",
                            override_log + 1,
                            s->data[0x08F8], s->pc, s->prog[s->pc],
                            (unsigned long long)(s->a & 0xFFFFFFFFFFLL),
                            s->st0, s->insn_count);
                    override_log++;
                }
            }
            /* Non-zero SET trace plus SET->CLEAR delta, the mirror of ZERO-WR.
             * Captures every write of a non-zero value to 0x08F8 to identify who
             * sets "FB found" and how many cycles it survives before a clear.
             * A delta below 100 insn points at an opcode bug hitting immediately;
             * a delta of thousands of insn is a legitimate timing race between
             * the DSP set and the ARM read. */
            {
                static uint64_t last_set_insn;
                static uint16_t last_set_val;
                static uint16_t last_set_pc;
                static unsigned set_log_n = 0;
                static unsigned delta_log_n = 0;
                if (val != 0) {
                    /* SET event */
                    if (set_log_n < 500) {
                        C54_DBG("FBDET", "D_FB_DET SET #%u val=0x%04x PC=0x%04x op=0x%04x "
                                "prev=0x%04x A=%010llx insn=%u",
                                set_log_n + 1,
                                val, s->pc, s->prog[s->pc],
                                s->data[0x08F8],
                                (unsigned long long)(s->a & 0xFFFFFFFFFFLL),
                                s->insn_count);
                        set_log_n++;
                    }
                    last_set_insn = s->insn_count;
                    last_set_val = val;
                    last_set_pc = s->pc;
                } else if (s->data[0x08F8] != 0 && last_set_insn != 0) {
                    /* CLEAR after non-zero - log delta */
                    uint64_t delta = (uint64_t)s->insn_count - last_set_insn;
                    if (delta_log_n < 100) {
                        C54_LOG("D_FB_DET SET-TO-CLEAR-DELTA #%u "
                                "set_PC=0x%04x set_insn=%llu set_val=0x%04x "
                                "clear_PC=0x%04x clear_insn=%u delta=%llu cycles",
                                delta_log_n + 1,
                                last_set_pc, (unsigned long long)last_set_insn,
                                last_set_val, s->pc, s->insn_count,
                                (unsigned long long)delta);
                        delta_log_n++;
                    }
                }
            }
            fbd_log++;
        }
    }

    /* Log DARAM writes to code target area and count total */
    if (addr >= 0x0020 && addr < 0x0800) {
        static int dw_total = 0;
        dw_total++;
        if (addr >= 0x1200 && addr <= 0x1240) {
            C54_LOG("DARAM WR [0x%04x] = 0x%04x PC=0x%04x insn=%u",
                    addr, val, s->pc, s->insn_count);
        }
        if (dw_total == 1 || dw_total == 100 || dw_total == 1000 || dw_total == 10000)
            C54_LOG("DARAM write count: %d (last: [0x%04x]=0x%04x)", dw_total, addr, val);
    }

    /* Frame-IT probe: who writes the flags the wedged DSP polls (data[0x006e],
     * data[0x585f])? Separates an ISR relocation from a hardware write. No write
     * at all, or a value never matching what is expected, means the flag is never
     * set and the DSP deadlocks. */
    if (addr == 0x006e || addr == 0x585f || addr == 0x8a44) {
        static uint32_t fw_n = 0;
        if (fw_n < 80) {
            fprintf(stderr, "[c54x] FLAGWR data[0x%04x] 0x%04x→0x%04x PC=0x%04x "
                    "INTM=%d insn=%u\n", addr, s->data[addr], val, s->pc,
                    !!(s->st1 & ST1_INTM), s->insn_count);
            fw_n++;
        }
    }

    /* SBSLOT-WR (read-only, cap 300): who writes the SB slots
     * a_serv_demod[D_TOA] in db_r (p0 data[0x0830], p1 data[0x0844]) and a_sch[3]
     * (p0 data[0x083a], p1 data[0x084e])? Separates a scatter write from another
     * writer from never written. Logs PC, opcode, A low and value. */
    if (addr == 0x0830 || addr == 0x0844 || addr == 0x083a || addr == 0x084e) {
        static unsigned sbw_n = 0;
        if (sbw_n < 300) {
            const char *what = (addr == 0x0830) ? "SERV_TOA_p0" :
                               (addr == 0x0844) ? "SERV_TOA_p1" :
                               (addr == 0x083a) ? "A_SCH3_p0"   : "A_SCH3_p1";
            fprintf(stderr, "[c54x] SBSLOT-WR %s data[0x%04x] 0x%04x->0x%04x "
                    "PC=0x%04x op=0x%04x A=0x%010llx insn=%u\n",
                    what, addr, s->data[addr], val, s->pc,
                    prog_fetch(s, s->pc),
                    (unsigned long long)(s->a & 0xFFFFFFFFFFULL), s->insn_count);
            sbw_n++;
        }
    }

    /* Who writes d_fb_mode (0x08f9) with garbage (> 1)? The detector runs but
     * d_fb_mode was measured at 0x435b instead of 0 or 1, so the window or the
     * scaling is wrong; a runaway AR2 with BK=0 would corrupt it. Names the
     * corrupter. */
    if (addr == 0x08f9 && val > 1) {   /* garbage only (value other than 0/1) */
        static uint32_t fm_n = 0;
        if (fm_n < 40) {
            fprintf(stderr, "[c54x] FBMODE-GARBAGE data[0x08f9] 0x%04x→0x%04x PC=0x%04x "
                    "op=0x%04x SP=0x%04x AR2=%04x AR3=%04x BK=%04x %s insn=%u\n",
                    s->data[0x08f9], val, s->pc, prog_fetch(s, s->pc),
                    s->sp, s->ar[2], s->ar[3], s->bk,
                    (s->sp == 0x08f9) ? "<<< SP-PUSH" :
                    (s->ar[2] == 0x08f9 || s->ar[2] == 0x08f8) ? "<<< AR2-STORE" : "?",
                    s->insn_count);
            fm_n++;
        }
    }

    /* DISPATCH_INSTALL, "init" mode - the crutch grafts onto the init routine
     * instead of fighting it.
     *
     * In the older "dispatch" mode it reinstalled data[0x43d8] on every pass of
     * the dispatcher (exec_pc == 0xb01c) while the init routine at 0xbb00 put its
     * 0xab38 back. Measured [2026-07-30]: 2894 rewrites by 0xbb00 (~1447
     * re-inits) and 156550 POST-BOOTSTUB-RET at PC=0x0000; the same evening's A/B
     * showed zero of both without the crutch. The storm was self-inflicted by the
     * fight, not a firmware defect.
     *
     * Here the INIT write is intercepted instead: 0xbb00 writes 0xab38, that
     * value is seen as-is by the mailbox monitor and by WATCH-WR (both placed
     * above, so the trace stays honest), and the substitution happens just before
     * the store. One write per init, no conflict, deterministic ordering.
     *
     * @BEQUILLE - DISPATCH_INSTALL_AT=init  (with CALYPSO_DISPATCH_INSTALL=0xNNNN)
     *   masque  : the routine that should install a real handler in 0x43d8; the
     *             slot no longer holds what the ROM put there.
     *   retirer : as soon as it is known who must populate 0x43d8. The gate
     *             already stays silent if a value other than 0xab38 appears, so
     *             it will not hide a real installer showing up.
     */
    if (addr == 0x43d8) {
        static int _dg = -1, _dgv = -1; static unsigned _dgn = 0;
        if (_dg < 0) {
            const char *m = calypso_getenv("CALYPSO_DISPATCH_INSTALL_AT");
            _dg = (m && strcmp(m, "init") == 0) ? 1 : 0;
            const char *v = calypso_getenv("CALYPSO_DISPATCH_INSTALL");
            _dgv = (v && *v) ? (int)strtoul(v, NULL, 0) : -1;
            if (_dg && _dgv >= 0)
                fprintf(stderr, "[c54x] DISPATCH-GRAFT arme : toute ecriture de 0xab38 "
                        "dans data[0x43d8] devient 0x%04x (mode init, BEQUILLE — le "
                        "mode dispatch est desactive)\n", (unsigned)_dgv);
        }
        if (_dg && _dgv >= 0 && val == 0xab38) {
            if (_dgn++ < 20)
                fprintf(stderr, "[c54x] DISPATCH-GRAFT #%u : l'init (PC=0x%04x) ecrit "
                        "0xab38 dans data[0x43d8] -> greffe 0x%04x insn=%u\n",
                        _dgn, s->pc, (unsigned)_dgv, s->insn_count);
            val = (uint16_t)_dgv;
        }
    }

    s->data[addr] = val;
}

/* 23-bit program address translation : honors XPC for >=0x8000 (extended
 * program memory / banked area), passes through for <0x8000 (common bank 0).
 * Shared by prog_fetch and prog_read so they cannot diverge again.
 * OVLY (DARAM mirror) is handled at call site because it routes to s->data[]. */
inline uint32_t c54x_prog_xlate(const C54xState *s, uint16_t addr16)
{
    /* Only the 0x8000-0xDFFF window is banked by XPC (PROM0/2/3 overlay).
     * 0xE000-0xFFFF is fixed ROM (PROM1, mirrored at prog[0xE000+]) and is not
     * banked on silicon. Applying XPC above 0xE000 makes PC=0xee00 with XPC=3
     * fetch prog[0x3ee00], past the 8K of PROM3, which reads as empty (op=0x0000)
     * and sends the post-correlator FB stage running away. High ROM must ignore
     * XPC. */
    if (addr16 >= 0x8000 && addr16 < 0xE000) {
        return (((uint32_t)s->xpc << 16) | addr16) & (C54X_PROG_SIZE - 1);
    }
    return addr16;   /* 0x0000-0x7FFF on-chip + 0xE000-0xFFFF PROM1 ROM (XPC-independent) */
}

uint16_t prog_fetch(C54xState *s, uint16_t pc)
{
    if ((s->pmst & PMST_OVLY) && pc >= c54x_ovly_bas() && pc < 0x2800)
        return s->data[pc];
    return s->prog[c54x_prog_xlate(s, pc)];
}

/* Floor of the OVLY alias. 0x0000-0x005F are mapped registers, 0x0060-0x007F is
 * the DARAM scratch pad. The SCH filter bank fills data[0x0060..0x0066] with MVDD
 * (prologue 0x8336) then reads those coefficients back with FIRS at
 * pmad=0x0061..0x0064, i.e. through PROGRAM space, which only works if the
 * scratch pad is visible there. With the floor at 0x80 the read fell through to
 * unloaded ROM and returned 0xF4E4 (measured: FIRS #1 pmad=0x0063,
 * coef(prog)=0xf4e4). Gate CALYPSO_OVLY_SCRATCH, default 1.
 * Warning: GLOBAL effect - the alias serves everything executed or read in
 * overlay. */
uint16_t c54x_ovly_bas(void)
{
    static int g = -1;
    if (g < 0) {
        g = calypso_gate("CALYPSO_OVLY_SCRATCH", 1);
        fprintf(stderr, "[c54x] OVLY-SCRATCH %s : plancher de l alias programme "
                "a 0x%04x (scratch-pad DARAM 0x0060-0x007F %s)\n",
                g ? "ACTIF" : "INACTIF", g ? 0x0060 : 0x0080,
                g ? "VISIBLE en programme" : "hors alias");
    }
    return g ? 0x0060 : 0x0080;
}

uint16_t prog_read(C54xState *s, uint32_t addr)
{
    uint16_t addr16 = addr & 0xFFFF;
    if ((s->pmst & PMST_OVLY) && addr16 >= c54x_ovly_bas() && addr16 < 0x2800)
        return s->data[addr16];
    return s->prog[c54x_prog_xlate(s, addr16)];
}

void prog_write(C54xState *s, uint32_t addr, uint16_t val)
{
    uint16_t addr16 = addr & 0xFFFF;
    /* PROM1 (0xE000-0xFFFF) is ROM - reject writes */
    if (addr16 >= 0xE000) return;
    if ((s->pmst & PMST_OVLY) && addr16 >= c54x_ovly_bas() && addr16 < 0x2800)
        s->data[addr16] = val;
    if (addr16 >= 0x8000) {
        uint32_t ext = ((uint32_t)s->xpc << 16) | addr16;
        ext &= (C54X_PROG_SIZE - 1);
        s->prog[ext] = val;
    }
    s->prog[addr16] = val;
}

/* ================================================================
 * Addressing mode helpers
 * ================================================================ */

/* Canonical C54x circular addressing (SPRU172, tic54x-dis.c): step is +/-1 or
 * +/-AR0, with |step| <= BK. BK=0 means linear - STM #0,BK is deliberate and
 * must not wrap.
 *
 * "base = ar - (ar % bk)" puts the base on a grid of multiples of BK. The C54x
 * does not work that way: the buffer starts at an address whose low N bits are
 * zero, with 2^N >= BK; the index is ARn & (2^N-1) and the base ARn & ~(2^N-1).
 *
 * Measured [2026-07-29] on the firmware DMA request queue (BK=14):
 *     ring at 0x4340   old:     0x4340 % 14 = 10 -> base 0x4336, index 10
 *                      correct: mask 0x0F        -> base 0x4340, index 0
 * The emulator therefore placed the ring at 0x4336..0x4343, straddling 0x433e and
 * 0x433f - the two pointers of the OTHER queue. Queue 2's pointer wandered
 * through 0x433d..0x433f and overwrote queue 1: corrupt accounting, then
 * `orm *(0x3f92), #0x0008` at 0xaa83 = DSP_ERR_DMA_PROG, reported to the ARM
 * ("DSP Error Status: 8", 605 times).
 *
 * Warning: like any ISA fix the effect is GLOBAL - circular addressing also
 * serves the filters and the I/Q buffers. CALYPSO_CIRC_BASE_MOD=1 restores the
 * old computation for A/B comparison. */
uint16_t c54x_circ_ref(uint16_t ar, int step, uint16_t bk)
{
    if (bk == 0) return (uint16_t)(ar + step);

    static int _vieux = -1;
    if (_vieux < 0) _vieux = calypso_gate("CALYPSO_CIRC_BASE_MOD", 0);

    uint16_t base, masque;
    int idx;

    if (_vieux) {
        base = (uint16_t)(ar - (ar % bk));
        idx  = (int)(ar % bk) + step;
    } else {
        unsigned n = 0;
        while ((1u << n) < (unsigned)bk) n++;   /* N such that 2^N >= BK */
        masque = (uint16_t)((1u << n) - 1u);
        base   = (uint16_t)(ar & (uint16_t)~masque);
        idx    = (int)(ar & masque) + step;
    }
    if (idx >= (int)bk) idx -= bk;
    else if (idx < 0)   idx += bk;
    return (uint16_t)(base + idx);
}

/* AR post-modification for the dual-operand and parallel families
 * (SPRU131G Table 5-8):
 *   00 = *ARi (none)   01 = *ARi-   10 = *ARi+   11 = *ARi+0% (circular, BK)
 * Uses c54x_circ_ref, whose base is aligned on 2^N, not on a grid of multiples
 * of BK. */
void c54x_par_postmod(C54xState *s, int ar, int mod)
{
    switch (mod) {
    case 1: s->ar[ar]--; break;
    case 2: s->ar[ar]++; break;
    case 3: s->ar[ar] = c54x_circ_ref(s->ar[ar], +(int16_t)s->ar[0], s->bk); break;
    default: break;
    }
}

/* ================================================================
 * Bit-reversed (reverse carry) addressing
 *     mode 7 = *ARn+0B      mode 4 = *ARn-0B
 *
 * These two modes used to be treated as a flat +/-AR0. The probe that shows they
 * matter: PDROM 0xf1b3 `mar *AR2+0B` (smem=0xBA -> mod=7, AR2), in the FB/SB
 * correlator region, right after the correlation loop (0xf16c..0xf17e) and the
 * 512-word `norm` pass (0xf195..0xf19b) - the bit reversal of an FFT.
 *
 * An FFT whose reverse carry is ignored yields plausible magnitudes and phase but
 * a wrong BIN ORDER: the angle converges (measured: -6 Hz, stable) while the peak
 * POSITION is wrong or pinned to the window edge (measured: TOA=39, the r39 edge,
 * or aberrant).
 *
 * Semantics (SPRU131G): the carry propagates towards the LOW-order bits instead
 * of the high ones. Exact loop-free equivalent: reverse the bits of both
 * operands, add normally, reverse the result.
 *
 * Warning: GLOBAL effect - these modes serve every FFT/IFFT in the firmware.
 * ================================================================ */
static inline uint16_t c54x_bitrev16(uint16_t v)
{
    v = (uint16_t)(((v & 0xAAAAu) >> 1) | ((v & 0x5555u) << 1));
    v = (uint16_t)(((v & 0xCCCCu) >> 2) | ((v & 0x3333u) << 2));
    v = (uint16_t)(((v & 0xF0F0u) >> 4) | ((v & 0x0F0Fu) << 4));
    v = (uint16_t)(((v & 0xFF00u) >> 8) | ((v & 0x00FFu) << 8));
    return v;
}

uint16_t c54x_revcarry(C54xState *s, uint16_t ar, uint16_t ar0, int sub)
{
    uint16_t plat = (uint16_t)(sub ? (ar - ar0) : (ar + ar0));
    uint16_t r;
    r = c54x_bitrev16(ar);
    r = (uint16_t)(sub ? (r - c54x_bitrev16(ar0)) : (r + c54x_bitrev16(ar0)));
    r = c54x_bitrev16(r);
    {   /* Probe: proves the mode is actually exercised and shows the gap with
         * the flat computation. Capped - never derive a rate from it. */
        static unsigned n = 0;
        if (n < 24) {
            n++;
            fprintf(stderr, "[c54x] BITREV #%u PC=0x%04x AR=0x%04x %c AR0=0x%04x "
                    "-> 0x%04x (plat: 0x%04x) insn=%u\n",
                    n, s->pc, ar, sub ? '-' : '+', ar0, r, plat, s->insn_count);
        }
    }
    return r;
}
