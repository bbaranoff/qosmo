/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * c54x_exec.c - execution core: c54x_exec_one and its instruction families.
 *
 * Split out of calypso_c54x.c on 2026-09-18 (one file per role).
 * File map in c54x_internal.h.
 */
#include "c54x_internal.h"
#include "hw/arm/calypso/calypso_debug.h"

/* 0xF4xx source/destination selectors: bit 9 = src, bit 8 = dst (TI SPRU172C). */
static inline void c54x_f4_srcdst(uint16_t op, int *src, int *dst)
{
    *src = (op >> 9) & 1;
    *dst = (op >> 8) & 1;
}

/* ═══════════════════════════════════════════════════════════════════════════
 * FIX_DECODE_BRANCH — MAC/bit handlers made reachable.
 *
 * These handlers were written under `case 0xF:` of switch(hi4) while their
 * opcodes have hi4 = 2 or 3, so they were dead by construction and the opcodes
 * fell through to the blind MAC of `case 0x3:` (`A = A + T*Smem`) or to
 * `case 0x2:`.
 *
 * [2026-08-04] sweep_reach.py: 161 handlers scanned, 11 unreachable, all of
 * them moved here: 0x2800 MAC, 0x2A00/0x2E00 MACR/MASR, 0x3000 LD Smem,T,
 * 0x3100 MPYA, 0x3200 LD Smem,ASM, 0x3300 MASA, 0x3400 BITT, 0x3500 MACA,
 * 0x3700 MACAR.
 *
 * ⚠️ The bodies are verbatim: only reachability is fixed, not the logic, so
 * every mask subtlety that existed before still exists. In particular
 * `0x2800/FC00` spans 0x2800..0x2BFF and therefore swallows the `0x2A00`
 * (MACR) test that follows it; that overlap predates this function and still
 * has to be checked against SPRU172C.
 *
 * ⚠️ Called BEFORE the `resolve_smem` of each case: every handler does its
 * own. Calling it afterwards would double post-increment the ARs.
 *
 * Returns -1 when the opcode is not handled, else the words consumed.
 * ═══════════════════════════════════════════════════════════════════════════ */
static int c54x_mac_bit_family(C54xState *s, uint16_t op, int consumed)
{
            /* MAC/MAS family Smem,SRC (0x28xx..0x2Fxx, mask FE00, 1 word).
             * Per tic54x-opc.c:
             *   0x2800 mac Smem,SRC      SRC = SRC + T * data[Smem]
             *   0x2A00 macr Smem,SRC     SRC = SRC + T * data[Smem] + 0x8000
             *   0x2C00 mas Smem,SRC      SRC = SRC - T * data[Smem]
             *   0x2E00 masr Smem,SRC     SRC = SRC - T * data[Smem] + 0x8000
             * bit 8 = SRC selector (0=A, 1=B). FRCT (ST1) shifts the product
             * left by one (Q15*Q15 -> Q31).
             * Smem forms only; the dual-MAC Xmem/Ymem variants
             * (0xA000..0xBFFF) are not covered. */
            if ((op & 0xFC00) == 0x2800) {
                int mac_sub = (op >> 9) & 1;       /* 0=add, 1=subtract */
                int mac_rnd = (op >> 8) & 0; /* not used here, separate below */
                (void)mac_rnd;
                bool mac_ind;
                uint16_t mac_addr = resolve_smem(s, op, &mac_ind);
                int16_t mac_mem = (int16_t)data_read(s, mac_addr);
                int32_t mac_prod = (int32_t)(int16_t)s->t * (int32_t)mac_mem;
                if (s->st1 & ST1_FRCT) mac_prod <<= 1;
                /* 0x2800/FE00 encoding per tic54x-opc.c: bits 15..9 op family
                 * (mac/mas/macr/masr), bit 8 SRC (0=A, 1=B), bits 7..0 Smem. */
                int mac_dst = (op >> 8) & 1;
                int64_t *mac_acc = mac_dst ? &s->b : &s->a;
                int64_t mac_term = (int64_t)(int32_t)mac_prod;
                if (mac_sub) mac_term = -mac_term;
                int64_t mac_new = sext40((*mac_acc + mac_term) & 0xFFFFFFFFFFULL);
                *mac_acc = mac_new;
                return (int)(consumed + s->lk_used);
            }
            /* MACR/MASR (mask FE00, base 0x2A00/0x2E00): MAC/MAS plus the
             * +0x8000 round, low half cleared. */
            if ((op & 0xFE00) == 0x2A00 || (op & 0xFE00) == 0x2E00) {
                int macr_sub = ((op & 0xFE00) == 0x2E00) ? 1 : 0;
                bool macr_ind;
                uint16_t macr_addr = resolve_smem(s, op, &macr_ind);
                int16_t macr_mem = (int16_t)data_read(s, macr_addr);
                int32_t macr_prod = (int32_t)(int16_t)s->t * (int32_t)macr_mem;
                if (s->st1 & ST1_FRCT) macr_prod <<= 1;
                macr_prod += 0x8000; /* round */
                macr_prod &= ~0xFFFF; /* zero low half after round */
                int macr_dst = (op >> 8) & 1;
                int64_t *macr_acc = macr_dst ? &s->b : &s->a;
                int64_t macr_term = (int64_t)(int32_t)macr_prod;
                if (macr_sub) macr_term = -macr_term;
                *macr_acc = sext40((*macr_acc + macr_term) & 0xFFFFFFFFFFULL);
                return (int)(consumed + s->lk_used);
            }

            /* 0x3500 MACA Smem[,B] (mask FF00, 1 word): B = B + A.hi * data[Smem].
             * The multiplier is A.hi (A[31:16]), not T. */
            if ((op & 0xFF00) == 0x3500) {
                bool maca_ind;
                uint16_t maca_addr = resolve_smem(s, op, &maca_ind);
                int16_t maca_mem = (int16_t)data_read(s, maca_addr);
                int16_t maca_ahi = (int16_t)((s->a >> 16) & 0xFFFF);
                int32_t maca_prod = (int32_t)maca_ahi * (int32_t)maca_mem;
                if (s->st1 & ST1_FRCT) maca_prod <<= 1;
                s->b = sext40((s->b + (int64_t)(int32_t)maca_prod) & 0xFFFFFFFFFFULL);
                s->t = (uint16_t)maca_mem;      /* SPRU172C: MACA Smem also loads T */
                return (int)(consumed + s->lk_used);
            }

            /* 0x3300 MASA Smem[,B] (mask FF00, 1 word): B = B - A.hi * data[Smem]. */
            if ((op & 0xFF00) == 0x3300) {
                bool masa_ind;
                uint16_t masa_addr = resolve_smem(s, op, &masa_ind);
                int16_t masa_mem = (int16_t)data_read(s, masa_addr);
                int16_t masa_ahi = (int16_t)((s->a >> 16) & 0xFFFF);
                int32_t masa_prod = (int32_t)masa_ahi * (int32_t)masa_mem;
                if (s->st1 & ST1_FRCT) masa_prod <<= 1;
                s->b = sext40((s->b - (int64_t)(int32_t)masa_prod) & 0xFFFFFFFFFFULL);
                s->t = (uint16_t)masa_mem;      /* SPRU172C: MASA Smem also loads T */
                return (int)(consumed + s->lk_used);
            }

            /* 0x3700 MACAR Smem[,B] (mask FF00, 1 word): MACA plus round. */
            if ((op & 0xFF00) == 0x3700) {
                bool macar_ind;
                uint16_t macar_addr = resolve_smem(s, op, &macar_ind);
                int16_t macar_mem = (int16_t)data_read(s, macar_addr);
                int16_t macar_ahi = (int16_t)((s->a >> 16) & 0xFFFF);
                int32_t macar_prod = (int32_t)macar_ahi * (int32_t)macar_mem;
                if (s->st1 & ST1_FRCT) macar_prod <<= 1;
                macar_prod += 0x8000;
                macar_prod &= ~0xFFFF;
                s->b = sext40((s->b + (int64_t)(int32_t)macar_prod) & 0xFFFFFFFFFFULL);
                s->t = (uint16_t)macar_mem;     /* SPRU172C: MACAR Smem also loads T */
                return (int)(consumed + s->lk_used);
            }

            /* 0x3100 MPYA Smem (mask FF00, 1 word): B = A.hi * data[Smem]. */
            if ((op & 0xFF00) == 0x3100) {
                bool mpya_ind;
                uint16_t mpya_addr = resolve_smem(s, op, &mpya_ind);
                int16_t mpya_mem = (int16_t)data_read(s, mpya_addr);
                int16_t mpya_ahi = (int16_t)((s->a >> 16) & 0xFFFF);
                int32_t mpya_prod = (int32_t)mpya_ahi * (int32_t)mpya_mem;
                if (s->st1 & ST1_FRCT) mpya_prod <<= 1;
                s->b = sext40((int64_t)(int32_t)mpya_prod);
                s->t = (uint16_t)mpya_mem;      /* SPRU172C: MPYA Smem also loads T */
                return (int)(consumed + s->lk_used);
            }

            /* [2026-08-23] MPY Smem,dst (0x2000/0xFE00) and LTD Smem
             * (0x4C00/0xFF00) were decoded nowhere, while the neighbouring
             * LD Smem,T (0x3000) and ST T,Smem (0x8C00) were.
             * Critical path: the loop feeding blocks 3 and 4 of the SCH bank is
             * 0x81e3 LD #1,ASM; 0x81e4 MPY Smem,dst; 0x81e5/0x81e6 ST A,*ARx+
             * (parallel stores). Without MPY, A stayed zero and the stores
             * overwrote the correlator's sane copies with zeros: 112 non-zero
             * writes from the MVDD at 0x7ce0/0x7ce4, then 210 zeros.
             * Semantics: MPY Smem,dst -> dst = T * Smem (bit 8 = accumulator)
             *            LTD Smem     -> T = Smem; data[Smem+1] = Smem
             * ⚠️ Global effect: alters the DSP instruction set as a whole. */
            if ((op & 0xFE00) == 0x2000) {
                bool mp_ind;
                uint16_t mp_addr = resolve_smem(s, op, &mp_ind);
                int16_t  mp_v = (int16_t)data_read(s, mp_addr);
                int64_t  mp_p = (int64_t)(int16_t)s->t * (int64_t)mp_v;
                if (s->st1 & ST1_FRCT) mp_p <<= 1;
                if ((op >> 8) & 1) s->b = sext40(mp_p);
                else               s->a = sext40(mp_p);
                return (int)(consumed + s->lk_used);
            }
            if ((op & 0xFF00) == 0x4C00) {
                bool lt_ind;
                uint16_t lt_addr = resolve_smem(s, op, &lt_ind);
                uint16_t lt_v = data_read(s, lt_addr);
                s->t = lt_v;
                data_write(s, (uint16_t)(lt_addr + 1), lt_v);  /* delay insertion */
                return (int)(consumed + s->lk_used);
            }

            /* 0x3000 LD Smem,T (mask FF00, 1 word): T = data[Smem]. */
            if ((op & 0xFF00) == 0x3000) {
                bool ldt_ind;
                uint16_t ldt_addr = resolve_smem(s, op, &ldt_ind);
                s->t = data_read(s, ldt_addr);
                return (int)(consumed + s->lk_used);
            }

            /* 0x3200 LD Smem,ASM (mask FF00, 1 word): ASM = data[Smem] & 0x1F. */
            if ((op & 0xFF00) == 0x3200) {
                bool ldasm_ind;
                uint16_t ldasm_addr = resolve_smem(s, op, &ldasm_ind);
                uint16_t ldasm_v = data_read(s, ldasm_addr) & ST1_ASM_MASK;
                s->st1 = (s->st1 & ~ST1_ASM_MASK) | ldasm_v;
                return (int)(consumed + s->lk_used);
            }

    return -1;
}


/* ═══════════════════════════════════════════════════════════════════════════
 * FIX_SFTA_CARRY — SFTA sets the carry.
 *
 * TI SPRU172C, SFTA page:
 *     If SHIFT < 0 : (src((-SHIFT)-1)) -> C ; src << SHIFT -> dst
 *                    high fill = src(39) when SXM=1, else 0
 *     Else         : (src(39 - SHIFT)) -> C ; src << SHIFT -> dst
 *     Status Bits  : Affected by SXM and OVM / Affects C and OVdst
 *
 * [2026-08-04] Without the carry, the DSP firmware bit-transfer loop at
 * 0x9ac7..0x9ace (`sfta A` -> carry, `rol B` -> B = B<<1 | C,
 * `stl *AR2+, B`) left B at 0, so `stl` wrote 0x0000 to 0x2c3c and the `mvdd`
 * at 0x9723 published that hole into a_cd[3..14], the 12 payload words the ARM
 * lifts up to L2. The real opcode is `f47f`: shift = 0x1F - 32 = -1, i.e. move
 * bit 0 of A into C.
 *
 * ⚠️ The SXM-dependent fill belongs to the same SPRU172C rule and is applied
 * here too; a right shift is arithmetic only when SXM=1.
 * ═══════════════════════════════════════════════════════════════════════════ */
/* [2026-09-21] Carry of the 40-bit adder, SPRU172C: ADD sets C on a carry out
 * of the accumulator, SUB (A + ~B + 1) resets C on a borrow. */
static inline void c54x_carry_add40(C54xState *s, int64_t a, int64_t b)
{
    uint64_t ua = (uint64_t)a & 0xFFFFFFFFFFULL, ub = (uint64_t)b & 0xFFFFFFFFFFULL;
    if (((ua + ub) >> 40) & 1) s->st0 |= ST0_C; else s->st0 &= ~ST0_C;
}
static inline void c54x_carry_sub40(C54xState *s, int64_t a, int64_t b)
{
    uint64_t ua = (uint64_t)a & 0xFFFFFFFFFFULL, ub = (uint64_t)b & 0xFFFFFFFFFFULL;
    if (ua >= ub) s->st0 |= ST0_C; else s->st0 &= ~ST0_C;
}

static void c54x_sfta_exec(C54xState *s, uint16_t op)
{
    int src, dst;
    c54x_f4_srcdst(op, &src, &dst);
    int shift = op & 0x1F;
    if (shift > 15) shift -= 32;
    int64_t sv = sext40(src ? s->b : s->a);

    /* [2026-09-21] left shift: C = the last bit shifted out of bit 31, i.e.
     * src(32 - SHIFT) (SPRU172C SFTA example: 80AA001234 << 5 -> C = 1, the
     * bit 27); bit 39 - SHIFT read 0 there. isa_test 188. */
    int cbit = (shift < 0) ? (int)((sv >> ((-shift) - 1)) & 1)
             : (shift > 0) ? (int)((sv >> (32 - shift)) & 1) : ((s->st0 & ST0_C) ? 1 : 0);
    if (cbit) s->st0 |= ST0_C;
    else      s->st0 &= ~ST0_C;

    if (shift >= 0) {
        sv <<= shift;
    } else if (!(s->st1 & ST1_SXM)) {
        sv = (int64_t)(((uint64_t)sv & 0xFFFFFFFFFFULL) >> (-shift));
    } else {
        sv >>= (-shift);
    }
    if (dst) s->b = sext40(sv);
    else     s->a = sext40(sv);
}


/* ═══════════════════════════════════════════════════════════════════════════
 * FIX_ROL_CARRY_BIT — ROR/ROL read the carry from the right bit.
 *
 * `ST0_C` is `(1 << 11)` (calypso_c54x.h), but the ROR/ROL handlers read
 * `(s->st0 >> 8) & 1`, i.e. bit 8, which belongs to `ST0_DP_MASK` (bits 8-0,
 * data page pointer), while writing the carry correctly through
 * `s->st0 |= ST0_C`. Read and write named different bits, so ROL rotated a
 * page-pointer bit instead of the carry.
 *
 * [2026-08-04] Measured: even with FIX_SFTA_CARRY posting the carry at bit 11,
 * the firmware loop at 0x9ac7..0x9ace (`sfta A` then `rol B`) kept B at zero,
 * `stl *AR2+, B` wrote 0x0000 to 0x2c3c and the `mvdd` at 0x9723 published that
 * hole into a_cd[3..14].
 * ═══════════════════════════════════════════════════════════════════════════ */
static inline uint16_t c54x_carry_in(C54xState *s)
{
    return (s->st0 & ST0_C) ? 1 : 0;
}

/* SFTL src, SHIFT [, dst] — SPRU172C 4-158. A 32-bit LOGICAL shift of src(31-0):
 *   SHIFT < 0 : src((-SHIFT)-1) -> C ; src(31-0) >> -SHIFT -> dst(31-0), zero fill
 *   SHIFT = 0 : 0 -> C ; dst(31-0) = src(31-0)
 *   SHIFT > 0 : src(32-SHIFT) -> C ; (src(31-0) << SHIFT) & 0xFFFFFFFF -> dst(31-0)
 *   always     0 -> dst(39-32)
 * [2026-09-20] The three former copies shifted the 40-bit accumulator and never
 * set C. Manual example SFTL A, -5, B: A = 00 8765 0055 -> B = 00 043B 2802, C = 1.
 * One body, called from every dispatch copy. */
static void c54x_sftl_exec(C54xState *s, uint16_t op)
{
    int src, dst;
    c54x_f4_srcdst(op, &src, &dst);
    int shift = op & 0x1F;
    if (shift > 15) shift -= 32;
    uint32_t v = (uint32_t)((src ? s->b : s->a) & 0xFFFFFFFFULL);
    uint32_t r; int c;
    if (shift < 0)      { c = (v >> (-shift - 1)) & 1; r = v >> (-shift); }
    else if (shift == 0){ c = 0;                        r = v; }
    else                { c = (v >> (32 - shift)) & 1;  r = v << shift; }
    if (c) s->st0 |= ST0_C; else s->st0 &= ~ST0_C;
    if (dst) s->b = (int64_t)r; else s->a = (int64_t)r;
}

bool calypso_fix_enabled(const char *name)
{
    static char buf[1024];
    static int  init = 0;
    if (!init) {
        const char *e = calypso_getenv("CALYPSO_FIXES");
        snprintf(buf, sizeof(buf), "%s", e ? e : "");
        init = 1;
        if (buf[0])
            fprintf(stderr, "[c54x] CALYPSO_FIXES=%s (sas temporaire — effacer le gate "
                            "de chaque correctif confirme)\n", buf);
    }
    if (!buf[0]) return false;
    if (strcmp(buf, "all") == 0) return true;
    size_t n = strlen(name);
    const char *p = buf;
    while ((p = strstr(p, name)) != NULL) {
        char before = (p == buf) ? ',' : p[-1];
        char after  = p[n];
        if ((before == ',' || before == ' ') &&
            (after == '\0' || after == ',' || after == ' '))
            return true;
        p += n;
    }
    return false;
}

/* CAL000 §5.1: the DSP timer is TINT = IMR bit 3 = vector 19, not vec20/bit4
 * (RINT, SPI receive). The generic C54x table in SPRU131 lists FOUR external
 * lines before TINT where the Calypso has THREE, hence the off-by-one.
 *
 * ⚠️ Unmeasured behaviour change: the observed IMR (0x52ed) has bit 3 unmasked
 * and bit 4 masked, so the timer interrupt used to be dropped silently by
 * c54x_interrupt_ex (which honours the IMR) and is now really dispatched on
 * vec19 at every underflow. The vec19 ROM handler is a RETE stub, so the effect
 * should be benign, but RETE stack symmetry is a known weak spot: if the stack
 * drifts under load, look here first. */
void c54x_fire_tint(C54xState *s)
{
    c54x_interrupt_ex(s, C54X_IT_TINT_VEC, C54X_IT_TINT_BIT);
}

int c54x_exec_one(C54xState *s)
{
    if (c54x_irq_level_check(s)) {
        return 1;   /* per-instruction IRQ vectoring consumed this step */
    }
    uint16_t op = prog_fetch(s, s->pc);
    /* B1 probe (CALYPSO_B1): at the MAC kernel 0xa076, dump the correlator
     * reference table data[0x2c00..0x2c0f] plus a checksum, which tells whether
     * the boot copy 0x76f8 -> 0x2c00 ran or the table is still zero. */
    {
        static int _b1 = -1; static unsigned _b1n = 0;
        if (_b1 < 0) _b1 = calypso_gate("CALYPSO_B1", 0);
        if (_b1 && s->xpc == 0 && s->pc == 0xa076 && _b1n < 20) {
            _b1n++;
            uint32_t _ck = 0;
            for (int _i = 0; _i < 0x100; _i++) _ck += s->data[0x2c00 + _i];
            fprintf(stderr, "[c54x] B1 @0xa076 refTable[0x2c00..0f]=");
            for (int _i = 0; _i < 16; _i++) fprintf(stderr, "%04x ", s->data[0x2c00 + _i]);
            fprintf(stderr, "| cksum(2c00..2cff)=0x%08x insn=%u\n", _ck, s->insn_count);
        }
    }
    {
        /* @BEQUILLE — CORR_BANK  (CALYPSO_CORR_BANK, VALUE, default -1/OFF)
         *   masks   : the overlay/bank selection of the FB handler. s->xpc is
         *             OVERWRITTEN at every instruction in [0x8d00..0xa200]
         *             instead of the native dispatcher setting the right bank.
         *   remove  : once the CALA dispatcher at 0xb01e resolves the bank on its
         *             own (observed XPC == expected bank without forcing).
         *   TRAP    : the value "0" does NOT turn this off, it forces XPC=0.
         *             Only leaving the variable unset disables it.
         */
        static int cbk = -2;
        if (cbk == -2) { const char *e = calypso_getenv("CALYPSO_CORR_BANK");
                         cbk = (e && *e) ? atoi(e) : -1; }   /* -1 = off; 0..3 = forced XPC */
        if (cbk >= 0 && cbk <= 3 && s->pc >= 0x8d00 && s->pc <= 0xa200 && s->xpc != (uint16_t)cbk) {
            s->xpc = (uint16_t)cbk;
        }
    }
    {
        /* @BEQUILLE — FORCE_3FAE  (CALYPSO_FORCE_3FAE, EXISTS, default OFF)
         *   masks   : the FB handshake flags that nothing implements yet --
         *             data[0x3faa] bit2/bit8, [0x3fab] bit8, [0x3fae] bit8. Set at
         *             EVERY instruction of the handler (xpc=0, pc 0x8d00..0xa200).
         *   remove  : once the RX/BRINT0 chain writes those flags itself.
         */
        static int f3ae = -1;
        if (f3ae < 0) f3ae = calypso_gate("CALYPSO_FORCE_3FAE", 0);
        if (f3ae && s->xpc == 0 && s->pc >= 0x8d00 && s->pc <= 0xa200) {
            /* The whole FB-detect handshake the handler polls (0x8866 and the
             * BITF sites 0x90c8/0x90ed/0x9128): 0x3faa bit2/bit8, 0x3fab bit8,
             * 0x3fae bit8. Without them the handler spins and the MAC kernel at
             * 0xa076 is never reached. */
            s->data[0x3faa] |= 0x0104;
            s->data[0x3fab] |= 0x0100;
            s->data[0x3fae] |= 0x0100;
        }
    }
    /* CORR-FLOW probe (CALYPSO_CORR_FLOW): traces the FB handler flow in bank 0
     * (0x8600..0xa200, XPC=0) -- raw PC/opcode, ST0 TC/C, AR1..AR5 -- to check
     * against SPRU172C where and why the flow leaves the MAC kernel at 0xa076.
     * Marks 0xa076 and 0x9a80. */
    {
        static int cf = -1; static unsigned cfn = 0;
        if (cf < 0) cf = calypso_gate("CALYPSO_CORR_FLOW", 0);
        /* The range starts at 0x8600 to cover the handshake subroutine at
         * 0x8866. AR3 is the CMPS/coefficient pointer; AR1/AR2 show the pointer
         * setup. The copy loop 0x8866-0x886c (opcode 8091, ~134 iterations per
         * call) is skipped and consecutive repeats of the same PC are dropped:
         * otherwise they consume the whole log budget. */
        static uint16_t cf_lastpc = 0;
        if (cf && s->xpc == 0 && s->pc >= 0x8600 && s->pc <= 0xa200 && cfn < 20000
            /* Trace only while a real FB/SB task is active (task_md = 5 or 6),
             * otherwise the budget is spent on idle spinning. */
            && (s->data[0x0804] == 5 || s->data[0x0804] == 6
                || s->data[0x0818] == 5 || s->data[0x0818] == 6
                || s->pc >= 0xa000)   /* also trace the 0xa0xx flow, where task_md is 0 */
            && !(s->pc >= 0x8866 && s->pc <= 0x886c)
            && s->pc != cf_lastpc) {
            cf_lastpc = s->pc;
            cfn++;
            const char *mk = (s->pc==0xa076) ? " <<<KERNEL-a076"
                           : (s->pc==0x9a80) ? " <<<KERNEL-9a80"
                           : (s->pc==0x8d00) ? " [handler-entry]"
                           : (s->pc==0x8866) ? " [subr-8866]"
                           : (s->ar[5]==0x2a00 || s->ar[3]==0x2a00) ? " <<<PTR=0x2a00!" : "";
            fprintf(stderr, "[c54x] CORR-FLOW PC=0x%04x op=%04x TC=%d C=%d "
                    "AR1=%04x AR2=%04x AR3=%04x AR4=%04x AR5=%04x%s insn=%u\n",
                    s->pc, op, !!(s->st0 & ST0_TC), !!(s->st0 & ST0_C),
                    s->ar[1], s->ar[2], s->ar[3], s->ar[4], s->ar[5], mk, s->insn_count);
        }
    }
    uint16_t op2;
    bool ind;
    uint16_t addr;
    int consumed = 1;
    s->lk_used = false;  /* reset before each instruction */
    s->writer_kind = WK_UNKNOWN;  /* attribution tag for DATA-W-MMR */

    /* DERAIL-EE00 probe: catches jumps into the empty PROM window 0xee00
     * (op=0x0000). Logs the source PC, its opcode and XPC, which separates a
     * runaway firmware branch from an XPC paging bug (a legitimate banked
     * address fetched from page 0). Capped at 12 hits. */
    if (s->pc >= 0xee00 && s->pc < 0xef00 &&
        !(s->last_exec_pc >= 0xee00 && s->last_exec_pc < 0xef00)) {
        static unsigned dr = 0;
        if (dr < 12) {
            fprintf(stderr, "[c54x] DERAIL-EE00 #%u entré PC=0x%04x DEPUIS last_pc=0x%04x op_src=0x%04x XPC=%u op@pc=0x%04x SP=0x%04x insn=%u\n",
                    dr, s->pc, s->last_exec_pc, prog_fetch(s, s->last_exec_pc),
                    s->xpc & 0xFF, op, s->sp, s->insn_count);
            dr++;
        }
    }

    static int ct_lo = -1, ct_hi = -1;
    if (ct_lo < 0) {
        const char *l = calypso_getenv("CALYPSO_CORR_LO"); const char *h = calypso_getenv("CALYPSO_CORR_HI");
        ct_lo = l ? (int)strtol(l, NULL, 0) : 0x8560;
        ct_hi = h ? (int)strtol(h, NULL, 0) : 0x8590;
    }
    if (s->insn_count > 60000000u && s->pc >= (uint16_t)ct_lo && s->pc <= (uint16_t)ct_hi
        && calypso_debug_enabled("CORR-TRACE")) {
        static unsigned ct = 0;
        if (ct < 60) {
            int64_t aa = (s->a & 0x8000000000LL) ? (int64_t)(s->a | ~0xFFFFFFFFFFLL) : (int64_t)s->a;
            fprintf(stderr, "[c54x] CORR-TRACE #%u PC=0x%04x op=%04x op2=%04x AR3=%04x data[AR3]=%04x A=%lld T=%04x BRC=%u insn=%u\n",
                    ct, s->pc, op, prog_fetch(s, s->pc + 1), s->ar[3], s->data[s->ar[3]],
                    (long long)aa, s->t, s->brc, s->insn_count);
            ct++;
        }
    }

    /* AR-CLOBBER probe (CALYPSO_DEBUG=AR_CLOBBER): tracks AR1/AR2/AR6/AR7
     * going to 0. Once an AR pointer is 0, any later indirect store *ARx hits
     * data[0x00], which is the IMR MMR. Logs the instruction that made the
     * transition (last_exec_pc and its opcode) to name the culprit. */
    /* [2026-09-19] A_SCH-1111: which instruction puts 0x1111 into a_sch[0]?
     * a_sch[0] = data[0x0837] (R page 0) and data[0x084b] (R page 1). It is the
     * MOST FREQUENT value in that cell (27 of 60 changes measured) and it is a
     * plain number where only B_BLUD (bit15) and B_SCH_CRC (bit8) have meaning;
     * no firmware and no emulation source contains it as a literal, so it is
     * computed. Runs per instruction like its AR-CLOBBER neighbour, and names
     * the instruction that JUST executed (last_exec_pc), not the next one --
     * the distinction that made the gdb watchpoint report the wrong site. */
    {
        static uint16_t a_prev0, a_prev1; static bool a_ini; static unsigned a_n;
        uint16_t a_now0 = s->data[0x0837], a_now1 = s->data[0x084b];
        if (a_ini && a_n < 30) {
            for (int pg = 0; pg < 2; pg++) {
                uint16_t was = pg ? a_prev1 : a_prev0, now = pg ? a_now1 : a_now0;
                if (now != was && (now & 0x8000)) {   /* BLUD=1 : un RESULTAT */
                    fprintf(stderr, "[c54x] A_SCH-RESULTAT page=%d %04x -> %04x "
                            "par PC=0x%04x op=0x%04x A=%lld B=%lld T=%04x "
                            "AR1=%04x AR2=%04x AR3=%04x AR4=%04x insn=%u\n",
                            pg, was, now, s->last_exec_pc,
                            prog_fetch(s, s->last_exec_pc),
                            (long long)s->a, (long long)s->b, s->t,
                            s->ar[1], s->ar[2], s->ar[3], s->ar[4], s->insn_count);
                    a_n++;
                }
            }
        }
        a_prev0 = a_now0; a_prev1 = a_now1; a_ini = true;
    }
    {
        static uint16_t prev_ar1, prev_ar2, prev_ar6, prev_ar7;
        static bool init_done = false;
        static unsigned clob_log = 0;
        if (!init_done) {
            prev_ar1 = s->ar[1]; prev_ar2 = s->ar[2];
            prev_ar6 = s->ar[6]; prev_ar7 = s->ar[7];
            init_done = true;
        }
        for (int i = 0; i < 4; i++) {
            int idx = (int[]){1, 2, 6, 7}[i];
            uint16_t *prev = (uint16_t*[]){&prev_ar1, &prev_ar2,
                                            &prev_ar6, &prev_ar7}[i];
            if (*prev != 0 && s->ar[idx] == 0) {
                if (calypso_debug_enabled("AR_CLOBBER") && clob_log < 30) {
                    uint16_t culprit_op = prog_fetch(s, s->last_exec_pc);
                    fprintf(stderr,
                            "[c54x] AR-CLOBBER #%u AR%d %04x->0 by "
                            "PC=0x%04x op=0x%04x cur_PC=0x%04x cur_op=0x%04x "
                            "SP=0x%04x insn=%u\n",
                            clob_log, idx, *prev,
                            s->last_exec_pc, culprit_op,
                            s->pc, op, s->sp, s->insn_count);
                    fflush(stderr);
                    clob_log++;
                }
            }
            *prev = s->ar[idx];
        }
    }

    if (s->pc == 0x013b && calypso_getenv("CALYPSO_AR0_DEBUG")) {
        static int d13 = 0;
        if (!d13) { d13 = 1;
            fprintf(stderr, "[c54x] SUB-013B A=0x%06llx DP=0x%03x d_page(08D4)=0x%04x insn=%u\n",
                    (unsigned long long)(s->a & 0xFFFFFF), s->st0 & 0x1FF,
                    /* 0x08D4 lives in the API window: read api_ram when it is
                     * mapped, data[] only as a fallback. */
                    s->api_ram ? s->api_ram[0x08D4 - C54X_API_BASE] : s->data[0x08D4],
                    s->insn_count);
            for (uint16_t a = 0x0138; a <= 0x014c; a += 4)
                fprintf(stderr, "[c54x] PROG[0x%04x..]= %04x %04x %04x %04x\n",
                        a, s->prog[a], s->prog[(uint16_t)(a+1)],
                        s->prog[(uint16_t)(a+2)], s->prog[(uint16_t)(a+3)]);
        }
    }
    if (s->pc == 0x8869 && calypso_getenv("CALYPSO_AR0_DEBUG")) {
        static int d88 = 0;
        if (!d88) { d88 = 1;
            fprintf(stderr, "[c54x] TASK-8869 A=0x%06llx DP=0x%03x AR2=%04x AR3=%04x "
                    "AR5=%04x task_md@058a=0x%04x insn=%u\n",
                    (unsigned long long)(s->a & 0xFFFFFF), s->st0 & 0x1FF,
                    s->ar[2], s->ar[3], s->ar[5], s->data[0x058a], s->insn_count);
            for (uint16_t a = 0x8860; a <= 0x8884; a += 4)
                fprintf(stderr, "[c54x] PROG[0x%04x..]= %04x %04x %04x %04x\n",
                        a, s->prog[a], s->prog[(uint16_t)(a+1)],
                        s->prog[(uint16_t)(a+2)], s->prog[(uint16_t)(a+3)]);
        }
    }
    if (s->pc == 0x7234 && calypso_getenv("CALYPSO_AR0_DEBUG")) {
        static int d72 = 0;
        if (!d72) { d72 = 1;
            int ovly = !!(s->pmst & PMST_OVLY);
            fprintf(stderr, "[c54x] DERAIL-013B XPC=0x%02x PMST=0x%04x OVLY=%d fetch(0x013b)=0x%04x insn=%u\n",
                    s->xpc & 0xFF, s->pmst, ovly, prog_fetch(s, 0x013b), s->insn_count);
            fprintf(stderr, "[c54x] OVERLAY data[0x0138..]= %04x %04x %04x %04x %04x %04x %04x %04x\n",
                    s->data[0x0138], s->data[0x0139], s->data[0x013a], s->data[0x013b],
                    s->data[0x013c], s->data[0x013d], s->data[0x013e], s->data[0x013f]);
            /* The go-live loop 0xa4de..0xa4e8 and the soft vector data[0x3f6d]
             * that drives the trampoline. */
            fprintf(stderr, "[c54x] GOLIVE-CODE fetch: 0xa4de=%04x 0xa4df=%04x 0xa4e0=%04x 0xa4e1=%04x "
                    "0xa4e2=%04x 0xa4e3=%04x 0xa4e4=%04x 0xa4e5=%04x  data[0x3f6d]=0x%04x\n",
                    prog_fetch(s,0xa4de), prog_fetch(s,0xa4df), prog_fetch(s,0xa4e0), prog_fetch(s,0xa4e1),
                    prog_fetch(s,0xa4e2), prog_fetch(s,0xa4e3), prog_fetch(s,0xa4e4), prog_fetch(s,0xa4e5),
                    s->data[0x3f6d]);
            fprintf(stderr, "[c54x] SOFTVEC data[0x3f6a]=0x%04x (CALA cible) 0x3f6b=0x%04x 0x3f6c=0x%04x "
                    "0x3f6d=0x%04x  (0xa671=OK, 0x71f4=RECURSE)\n",
                    s->data[0x3f6a], s->data[0x3f6b], s->data[0x3f6c], s->data[0x3f6d]);
            fprintf(stderr, "[c54x] PROM0-src[0x7138..]= %04x %04x %04x %04x %04x %04x %04x %04x\n",
                    s->prog[0x7138], s->prog[0x7139], s->prog[0x713a], s->prog[0x713b],
                    s->prog[0x713c], s->prog[0x713d], s->prog[0x713e], s->prog[0x713f]);
            /* Scheduler code at 0x7234, to reconfirm the CALL 0x013b. */
            fprintf(stderr, "[c54x] PROG[0x7234..]= %04x %04x %04x %04x\n",
                    s->prog[0x7234], s->prog[0x7235], s->prog[0x7236], s->prog[0x7237]);
        }
    }
    uint8_t hi4 = (op >> 12) & 0xF;
    uint8_t hi8 = (op >> 8) & 0xFF;

    /* Instruction-length fixes, gated by CALYPSO_FIXES (see calypso_fix_enabled).
     * Each entry quotes binutils tic54x-opc.c, whose second field IS the word
     * count. The decoder consumed 2 words for these one-word instructions, which
     * desynchronises every instruction decoded after them. */
    {   /* The fixes below without a calypso_fix_enabled() call are validated and
         * unconditional. Those still behind calypso_fix_enabled("FIX_...") are
         * formally correct but contradicted by measurement; see each comment. */
        /* LD Xmem,SHFT,dst — binutils { "ld", 1,3,3, 0x9400, 0xFE00,
         * {OP_Xmem,OP_SHFT,OP_DST} }; was decoded as a 2-word MVDK/MVKD. */
        if ((op & 0xFE00) == 0x9400) {
            uint16_t a = resolve_xmem(s, op);
            uint16_t v = data_read(s, a);
            int shft = op & 0xF, d = (op >> 8) & 1;
            int64_t x = (s->st1 & ST1_SXM) ? (int64_t)(int16_t)v : (int64_t)(uint16_t)v;
            x <<= shft;
            if (d) s->b = sext40(x); else s->a = sext40(x);
            return 1;
        }
        /* BIT Xmem,BITC: TC = Xmem(15-BITC) — binutils
         * { "bit", 1,2,2, 0x9600, 0xFF00 }; was decoded as a 2-word MVDP. */
        if ((op & 0xFF00) == 0x9600) {
            uint16_t a = resolve_xmem(s, op);
            uint16_t v = data_read(s, a);
            int bitc = op & 0xF;
            if ((v >> (15 - bitc)) & 1) s->st0 |= ST0_TC; else s->st0 &= ~ST0_TC;
            return 1;
        }
        /* SUB Xmem,Ymem,dst: dst = (Xmem - Ymem) << 16 — binutils
         * { "sub", 1,..., 0xA200, 0xFE00 }; was decoded as a 2-word ADD/SUB #lk. */
        if ((op & 0xFE00) == 0xA200) {
            uint16_t xa = resolve_xmem(s, op);
            uint8_t ym = op & 0xF; int yar = (ym & 3) + 2, ymod = (ym & 0xC) >> 2;
            uint16_t ya = s->ar[yar];
            switch (ymod) {
            case 1: s->ar[yar] = ya - 1; break;
            case 2: s->ar[yar] = ya + 1; break;
            case 3: s->ar[yar] = c54x_circ_ref(ya, +(int16_t)s->ar[0], s->bk); break;
            default: break;
            }
            int64_t xv = (int16_t)data_read(s, xa), yv = (int16_t)data_read(s, ya);
            int64_t r = (xv - yv) << 16;
            if ((op >> 8) & 1) s->b = sext40(r); else s->a = sext40(r);
            return 1;
        }
        /* LD Xmem,dst || MAC/MAS/MASR Ymem — binutils
         * { "ld", 1,..., 0xA800/0xAC00/0xAE00, 0xFE00 }; were decoded as 2-word
         * AND #lk / MACP / MACD. Only the LD half runs, the parallel half is
         * dropped: the RESULT is approximate but the LENGTH is right again, so
         * the instruction stream stops drifting. */
        if (((op & 0xFE00) == 0xA800 || (op & 0xFE00) == 0xAC00 || (op & 0xFE00) == 0xAE00)
            && calypso_fix_enabled("FIX_LD_PARALLEL")) {
            uint16_t a = resolve_xmem(s, op);
            uint16_t v = data_read(s, a);
            int64_t x = (s->st1 & ST1_SXM) ? (int64_t)(int16_t)v : (int64_t)(uint16_t)v;
            if ((op >> 8) & 1) s->b = sext40(x << 16); else s->a = sext40(x << 16);
            return 1;
        }

        /* LDM MMR,dst — binutils { "ldm", 1,2,2, 0x4800, 0xFE00, {OP_MMR,OP_DST} }.
         * An MMR is an UNSIGNED 16-bit value (pointer, counter, status register):
         * sign-extending turns AR=0x8000 into a negative 40-bit value. SPRU172C
         * gives "LDM MMR, dst : dst = MMR" with no sign extension (LDU is the
         * explicitly unsigned form; LDM has no signed variant). */
        /* [2026-09-20] Gate removed: SPRU172C example LDM AR4, A with AR4=FFFF
         * gives A = 00 0000 FFFF. Unconditional. */
        if ((op & 0xFE00) == 0x4800) {
            int mmr = op & 0x7F;
            uint16_t v = data_read(s, mmr);
            if ((op >> 8) & 1) s->b = (int64_t)(uint16_t)v; else s->a = (int64_t)(uint16_t)v;
            return 1 + s->lk_used;
        }
        /* DST src,Lmem — binutils { "dst", 1,2,2, 0x4E00, 0xFE00, {OP_SRC1,OP_Lmem} }.
         * Lmem is a LONG (2-word) operand, so the pointer advances by 2, not 1.
         * A post-modification of 1 shifts every later element of a long-word
         * array; the error is silent and cumulative. */
        if ((op & 0xFE00) == 0x4E00) {
            int src = (op >> 8) & 1;
            int64_t v = src ? s->b : s->a;
            /* [2026-09-20] All Lmem addressing modes through resolve_lmem (the old
             * code knew *ARx+ and *ARx- only, and ignored CPL for the direct form),
             * high word AT the address, low word at address ^ 1 (SPRU172C DST
             * example, AR3 = 0101h: data[0101h] = hi, data[0100h] = lo). */
            uint16_t a = resolve_lmem(s, op);
            data_write(s, a,                 (uint16_t)((v >> 16) & 0xFFFF));
            data_write(s, (uint16_t)(a ^ 1), (uint16_t)(v & 0xFFFF));
            return 1 + s->lk_used;
        }
        /* STL/STH src,SHFT,Xmem — binutils { "stl"/"sth", 1,.., 0x9800/0x9A00,
         * 0xFE00, {OP_SRC1,OP_SHFT,OP_Xmem} }. The SHFT field (bits 3-0) was
         * ignored, so the stored value had the wrong scale.
         * Per SPRU172C 4-169/4-172, "syntax 3: when SHFT = 0 the opcode assembles
         * as syntax 1", so every 0x98/0x9A encountered has SHFT != 0. The SB
         * demodulator has 4 such sites (0x7694/0x7697 `9a91/9a11`, 0x8217). */
        if ((op & 0xFE00) == 0x9800 || (op & 0xFE00) == 0x9A00) {
            uint16_t a = resolve_xmem(s, op);
            int shft = op & 0xF;
            int src = (op >> 8) & 1;
            int64_t v = src ? s->b : s->a;
            v <<= shft;
            uint16_t w = ((op & 0xFE00) == 0x9A00) ? (uint16_t)((v >> 16) & 0xFFFF)
                                                   : (uint16_t)(v & 0xFFFF);
            data_write(s, a, w);
            return 1;
        }
        /* SUB Smem,16,src[,dst] — binutils { "sub", 1,.., 0x4000, 0xFC00,
         * {OP_Smem,OP_16,OP_SRC,OPT|OP_DST} }. Two distinct fields: bit 9 = SRC
         * (source accumulator), bit 8 = DST. Bit 9 was ignored, so the
         * subtraction always started from the same accumulator. */
        if ((op & 0xFC00) == 0x4000) {
            bool ind2; uint16_t a = resolve_smem(s, op, &ind2);
            uint16_t v = data_read(s, a);
            int srcb = (op >> 9) & 1, dstb = (op >> 8) & 1;
            int64_t sv = srcb ? s->b : s->a;
            int64_t r  = sv - (((int64_t)(int16_t)v) << 16);
            if (dstb) s->b = sext40(r); else s->a = sext40(r);
            return 1 + s->lk_used;
        }
        /* STL B,ASM,Smem — binutils { "stl", 1,..., 0x8400, 0xFE00 } also covers
         * 0x85 (src = B); was decoded as a 2-word MVPD. Exact mirror of the
         * already validated 0x84 handler. */
        if ((op & 0xFF00) == 0x8500) {
            bool ind2; uint16_t a = resolve_smem(s, op, &ind2);
            int shift = asm_shift(s);
            int64_t v = s->b;
            if (shift >= 0) v <<= shift; else v >>= (-shift);
            data_write(s, a, (uint16_t)(v & 0xFFFF));
            return 1 + s->lk_used;
        }
        /* ST TRN,Smem — binutils { "st", 1,..., 0x8D00, 0xFF00 }; was decoded as
         * a 2-word MVDD. */
        if ((op & 0xFF00) == 0x8D00) {
            bool ind2; uint16_t a = resolve_smem(s, op, &ind2);
            data_write(s, a, s->trn);
            return 1 + s->lk_used;
        }
    }

    /* DISP-ENTRY probe (CALYPSO_DEBUG=DISP-ENTRY): tells an interrupt preemption
     * apart from a clobber. Logs the dispatcher entry at 0x8341 only, with
     * DP/ST0/SP/AR2, the interrupt state (INTM/IFR/INT3-pending), the context of
     * the last interrupt served (vector, insn delta, preempted foreground PC and
     * DP) and the LUT slot that 0x834d will read, data[(DP<<7)|0x07]. A bad entry
     * has DP != 0x124; if it always coincides with a recent interrupt, the cause
     * is preemption rather than a stale DP. */
    if (s->pc == 0x7234) {
        /* One-shot dump of the 0x7234 scheduler (CALYPSO_AR0_DEBUG): what it does
         * and which indirect pointer sends it to 0x013b. */
        if (calypso_getenv("CALYPSO_AR0_DEBUG")) {
            static int d7 = 0;
            if (!d7) { d7 = 1;
                fprintf(stderr, "[c54x] SCHED-7234 A=0x%06llx ST0=0x%04x DP=0x%03x "
                        "AR1=%04x AR2=%04x AR5=%04x d_page(08D4)=0x%04x d584=0x%04x insn=%u\n",
                        (unsigned long long)(s->a & 0xFFFFFF), s->st0, s->st0 & 0x1FF,
                        s->ar[1], s->ar[2], s->ar[5],
                        /* d_dsp_page is 0x08D4 in api_ram, not 0x08E2 in data[]. */
                        s->api_ram ? s->api_ram[0x08D4 - C54X_API_BASE] : s->data[0x08D4],
                        s->data[0x0584], s->insn_count);
                for (uint16_t a = 0x7230; a <= 0x7240; a += 4)
                    fprintf(stderr, "[c54x] PROG[0x%04x..]= %04x %04x %04x %04x\n",
                            a, s->prog[a], s->prog[(uint16_t)(a+1)],
                            s->prog[(uint16_t)(a+2)], s->prog[(uint16_t)(a+3)]);
            }
        }
        /* @BEQUILLE — FORCE_DISPATCH  (CALYPSO_FORCE_DISPATCH, atoi>0, default
         *              OFF; calypso_wire.env sets it to 1)
         *   masks   : the frame scheduler at 0x7234 is reached with a garbage DP
         *             and d_dsp_page at 0, so the LUT at 0x8341 does not resolve
         *             and the GSM/FB task is never dispatched. DP is forced to
         *             0x124 and d_dsp_page / data[0x0584] to 0x0002.
         *   remove  : once the 0x013b prologue restores a valid DP and the
         *             producer of d_dsp_page writes B_GSM_TASK (bit 1) through
         *             the ARM path.
         */
        static int fd = -1;
        if (fd < 0) { const char *e = calypso_getenv("CALYPSO_FORCE_DISPATCH"); fd = (e && atoi(e) > 0) ? 1 : 0; }
        if (fd) {
            /* d_dsp_page is 0x08D4 (0x08E2 is d_dsp_state) and must be written
             * in api_ram: that is the array the ROM reads for the 0x0800+ range.
             * Writing data[] instead leaves the cell inert. */
            uint16_t old = (uint16_t)(s->st0 & 0x1FF);
            uint16_t oldpg = s->api_ram ? s->api_ram[0x08D4 - C54X_API_BASE]
                                        : s->data[0x08D4];
            s->st0 = (uint16_t)((s->st0 & ~0x1FF) | (0x124 & 0x1FF));
            if (s->api_ram)
                s->api_ram[0x08D4 - C54X_API_BASE] = 0x0002;  /* B_GSM_TASK | w_page=0 */
            else
                s->data[0x08D4] = 0x0002;
            s->data[0x0584] = 0x0002;
            static unsigned fdl = 0;
            if (fdl++ < 16)
                fprintf(stderr, "[c54x] FORCE-DISPATCH @0x7234 DP 0x%03x->0x124 "
                        "d_page 0x%04x->0x0002 insn=%u\n",
                        old, oldpg, s->insn_count);
        }
    }
    /* SBFN-PROBE (read-only, gate CALYPSO_SBFN) ===============================
     * Answers: which frame does the DARAM buffer hold when the SB task (0x9841)
     * reads it? calypso_bsp.c documents that by default EVERY frame writes into
     * 0x2a00, so about 9 non-FCCH bursts land between two FCCH. If SB correlates
     * an arbitrary burst, the SCH word comes out invalid and the DSP raises
     * B_SCH_CRC rightly: a TIMING failure, not a processing one.
     * Prints the fn of the last deposit, fn%51 (SCH frames are {1,11,21,31,41},
     * GSM 45.002), how many bursts were deposited since the previous SB, and the
     * buffer amplitude. Writes nothing, so it cannot change behaviour. */
    if (s->pc == 0x9841) {
        static int on = -1;
        if (on < 0) { const char *e = calypso_getenv("CALYPSO_SBFN"); on = (e && *e && atoi(e)) ? 1 : 0; }
        if (on) {
            static unsigned n = 0, prevwr = 0;
            if (n++ < 400) {
                int mn = 32767, mx = -32768; long en = 0;
                for (int k = 0; k < 296; k++) {
                    int v = (int16_t)s->data[(uint16_t)(0x2a00 + k)];
                    if (v < mn) { mn = v; }
                    if (v > mx) { mx = v; }
                    en += (long)v * v;
                }
                unsigned fn = calypso_daram_last_fn;
                fprintf(stderr, "[c54x] SBFN-PROBE #%u fn=%u p51=%u %s depots_depuis_SB=%u "
                        "iq=[%d..%d] energie=%ld insn=%u\n",
                        n, fn, fn % 51,
                        ((fn % 51) % 10 == 1 && (fn % 51) <= 41) ? "SCH" :
                        ((fn % 51) % 10 == 0 && (fn % 51) <= 40) ? "FCCH" : "AUTRE",
                        calypso_daram_wr_count - prevwr, mn, mx, en, s->insn_count);
                prevwr = calypso_daram_wr_count;
            }
        }
    }
    /* SUBC-PROBE (read-only, gate CALYPSO_SUBC) ===============================
     * The whole cascade hangs on one value, the quotient of the 16-step division
     * in the caller:
     *     0x7d1c  RPT #15
     *     0x7d1d  SUBC *(0x0b), A      ; division
     *     0x7d1e  STL  A, *(0x0a)      ; THE QUOTIENT
     *     0x7d21  LD   *(0x0a), T      ; T is loaded here
     *     0x7d24  CALLD 0x81df         ; subroutine of the MPY at 0x81e4
     * zero quotient -> zero T -> zero product -> blocks 3/4 overwritten -> empty
     * coefficient source -> FIRS multiplies by nothing -> B_SCH_CRC raised.
     *
     * Liveness control, same gate: passes through 0x989f (the bad-CRC branch),
     * which is known to execute, are counted too. A quotient count stuck at zero
     * while the control count rises is a measured fact; both silent means the
     * probe is dead, not the code. */
    if (s->pc == 0x7d19 || s->pc == 0x7d1b || s->pc == 0x7d1c ||
        s->pc == 0x7d1e || s->pc == 0x81e4 || s->pc == 0x989f) {
        static int on = -1;
        if (on < 0) { const char *e = calypso_getenv("CALYPSO_SUBC"); on = (e && *e && atoi(e)) ? 1 : 0; }
        if (on) {
            static unsigned n_q = 0, n_mpy = 0, n_ctl = 0;
            if (s->pc == 0x989f) {
                n_ctl++;
                if (n_ctl <= 3 || (n_ctl % 25) == 0)
                    fprintf(stderr, "[c54x] SUBC-PROBE CONTROLE 0x989f #%u "
                            "(quotient=%u mpy=%u) insn=%u\n",
                            n_ctl, n_q, n_mpy, s->insn_count);
            } else if (s->pc == 0x7d19 || s->pc == 0x7d1b || s->pc == 0x7d1c) {
                /* The dividend is the constant 1 shifted (ld #1,A; sfta A,<n>).
                 * If the shift leaves A below the divisor the quotient is zero by
                 * construction, which is not a SUBC bug, so A is sampled at three
                 * points: before the LD, after the LD and just before the
                 * division. */
                static unsigned n_s = 0;
                if (n_s++ < 60)
                    fprintf(stderr, "[c54x] SUBC-PROBE DIVIDENDE@0x%04x A=0x%010llx "
                            "op=0x%04x ST1=0x%04x diviseur=0x%04x insn=%u\n",
                            s->pc, (unsigned long long)(s->a & 0xFFFFFFFFFFULL),
                            prog_fetch(s, s->pc), s->st1,
                            s->data[(uint16_t)(((s->st0 & 0x1FF) << 7) | 0x0B)],
                            s->insn_count);
            } else if (s->pc == 0x7d1e) {
                if (n_q++ < 40) {
                    unsigned dp = s->st0 & 0x1FF;
                    uint16_t divis = s->data[(uint16_t)((dp << 7) | 0x0B)];
                    uint16_t prev  = s->data[(uint16_t)((dp << 7) | 0x0A)];
                    fprintf(stderr, "[c54x] SUBC-PROBE QUOTIENT #%u A=0x%010llx "
                            "bas16=0x%04x diviseur[DP:0x0b]=0x%04x ancien[0x0a]=0x%04x "
                            "T=0x%04x DP=0x%03x insn=%u\n",
                            n_q, (unsigned long long)(s->a & 0xFFFFFFFFFFULL),
                            (unsigned)(s->a & 0xFFFF), divis, prev, s->t, dp,
                            s->insn_count);
                }
            } else {
                if (n_mpy++ < 40)
                    fprintf(stderr, "[c54x] SUBC-PROBE MPY@0x81e4 #%u T=0x%04x "
                            "A=0x%010llx insn=%u\n",
                            n_mpy, s->t, (unsigned long long)(s->a & 0xFFFFFFFFFFULL),
                            s->insn_count);
            }
        }
    }
    /* MVDD-PROBE (read-only, gate CALYPSO_SUBC) ===============================
     * Measured: the dividend of the 0x7d1d division is not a physical quantity
     * but the INDEX of the first non-zero word of blocks 3/4 (backward scan
     * 0x7cf5..0x7d06, falling through to `xor A` when nothing is found). Empty
     * blocks -> A=0 -> quotient 0 -> T=0 -> zero coefficients; the blocks are the
     * cause, not a consequence of T.
     *
     * Blocks 3/4 are meant to be filled by two 7-word copies:
     *     0x7cda  stm #0x2cce, AR2        ; destination = block 3
     *     0x7cdc  stm #0x2c56, AR3        ; source = CORR A
     *     0x7cde  mar *AR3+0              ; AR3 += AR0   <-- offset
     *     0x7cdf  rpt #6 / 0x7ce0 mvdd
     *     0x7ce1  mar *+AR3(0x2b) / 0x7ce3 rpt #6 / 0x7ce4 mvdd  ; block 4
     * and that copy is SKIPPED by `bcd 0x7ced, ANEQ` at 0x7ccd when the blocks
     * are already non-zero. The probe prints whether 0x7ccd is reached and taken,
     * whether the copy runs, with which AR0/AR3, and what the source holds. */
    if (s->pc == 0x7ccd || s->pc == 0x7ce0 || s->pc == 0x7ce4) {
        static int on = -1;
        if (on < 0) { const char *e = calypso_getenv("CALYPSO_SUBC"); on = (e && *e && atoi(e)) ? 1 : 0; }
        if (on) {
            static unsigned n_g = 0, n_c3 = 0, n_c4 = 0;
            if (s->pc == 0x7ccd) {
                if (n_g++ < 20) {
                    long som = 0;
                    for (int k = 0; k < 14; k++) som += (int16_t)s->data[(uint16_t)(0x2cce + k)];
                    fprintf(stderr, "[c54x] MVDD-PROBE GARDE@0x7ccd A=0x%010llx "
                            "(saute-la-copie si A!=0) somme_blocs=%ld AR0=0x%04x insn=%u\n",
                            (unsigned long long)(s->a & 0xFFFFFFFFFFULL), som,
                            s->ar[0], s->insn_count);
                }
            } else if (s->pc == 0x7ce0) {
                if (n_c3++ < 20)
                    fprintf(stderr, "[c54x] MVDD-PROBE COPIE bloc3 AR3(src)=0x%04x "
                            "AR2(dst)=0x%04x AR0=0x%04x src[0..3]=%04x %04x %04x %04x insn=%u\n",
                            s->ar[3], s->ar[2], s->ar[0],
                            s->data[s->ar[3]], s->data[(uint16_t)(s->ar[3]+1)],
                            s->data[(uint16_t)(s->ar[3]+2)], s->data[(uint16_t)(s->ar[3]+3)],
                            s->insn_count);
            } else {
                if (n_c4++ < 20)
                    fprintf(stderr, "[c54x] MVDD-PROBE COPIE bloc4 AR3(src)=0x%04x "
                            "AR2(dst)=0x%04x src[0..3]=%04x %04x %04x %04x insn=%u\n",
                            s->ar[3], s->ar[2],
                            s->data[s->ar[3]], s->data[(uint16_t)(s->ar[3]+1)],
                            s->data[(uint16_t)(s->ar[3]+2)], s->data[(uint16_t)(s->ar[3]+3)],
                            s->insn_count);
            }
        }
    }
    if (s->pc == 0x8341) {
        /* @BEQUILLE — FORCE_DP (+ FORCE_DP_FROM as a scope)  (CALYPSO_FORCE_DP,
         *              VALUE, default OFF)
         *   masks   : the DP field of ST0 at the dispatcher entry is a stack
         *             residue (over-pop, ST0 not restored) instead of the
         *             expected data page.
         *   remove  : once the DISP-ENTRY probe shows the dispatcher resolving
         *             without forcing, i.e. once ST0 push/pop is balanced.
         */
        static int inited = 0, force_dp = -1, force_from = -1;
        if (!inited) {
            inited = 1;
            const char *e = calypso_getenv("CALYPSO_FORCE_DP");
            force_dp = (e && *e) ? (int)strtol(e, NULL, 0) : -1;
            const char *ef = calypso_getenv("CALYPSO_FORCE_DP_FROM"); /* scoped: force only when DP==FROM */
            force_from = (ef && *ef) ? (int)strtol(ef, NULL, 0) : -1; /* -1 = unscoped */
        }
        if (force_dp >= 0) {
            int cur = s->st0 & 0x1FF;
            if (force_from < 0 || cur == force_from)
                s->st0 = (uint16_t)((s->st0 & ~0x1FF) | (force_dp & 0x1FF));
        }
    }
    if (s->pc == 0x8341 && calypso_debug_enabled("DISP-ENTRY")) {
        static unsigned de_n = 0;
        if (de_n++ < 20000) {
            uint16_t lut_ea = (uint16_t)(((s->st0 & 0x1FF) << 7) | 0x07);
            uint16_t lut    = s->data[lut_ea];
            uint64_t d_intr = s->insn_count - g_last_intr_insn;
            fprintf(stderr,
                "[c54x] DISP-ENTRY DP=0x%03x ST0=0x%04x SP=0x%04x AR2=0x%04x "
                "INTM=%d IFR=0x%04x INT3pend=%d  lut[0x%04x]=0x%04x %s  "
                "prevPC=0x%04x  lastLDP{pc=0x%04x val=0x%03x kind=%d}  "
                "lastST0w{pc=0x%04x op=0x%04x xpc=%u val=0x%04x prev=0x%04x}  "
                "lastIT{vec=%d dInsn=%llu fgPC=0x%04x fgDP=0x%03x} insn=%u\n",
                (unsigned)(s->st0 & 0x1FF), s->st0, s->sp, s->ar[2],
                !!(s->st1 & ST1_INTM), s->ifr, !!(s->ifr & (1 << 3)),
                lut_ea, lut, (lut == 0xff72 ? "OK" : "BAD"),
                g_prev_pc, g_last_ldp_pc, g_last_ldp_val, g_last_ldp_kind,
                g_last_st0w_pc, g_last_st0w_op, g_last_st0w_xpc,
                g_last_st0w_val, g_last_st0w_prev,
                g_last_intr_vec, (unsigned long long)d_intr,
                g_last_intr_fg_pc, g_last_intr_fg_dp, s->insn_count);
            if (lut != 0xff72) {   /* bad dispatcher: dump the ST0 push/pop ring */
                fprintf(stderr, "[c54x] ST0-RING@dispBAD DP=0x%03x SP=0x%04x (anciens→récents) :",
                        (unsigned)(s->st0 & 0x1FF), s->sp);
                unsigned rn = g_st0_ring_idx < ST0_RING_N ? g_st0_ring_idx : ST0_RING_N;
                for (unsigned i = 0; i < rn; i++) {
                    St0Ev *e = &g_st0_ring[(g_st0_ring_idx - rn + i) % ST0_RING_N];
                    fprintf(stderr, " %c@%04x:%04x v=%04x(DP=%03x)SP=%04x",
                            e->kind, e->pc, e->op, e->val,
                            (unsigned)(e->val & 0x1FF), e->sp);
                }
                fprintf(stderr, "\n");
            }
        }
    }

    /* DISP-TRACE (CALYPSO_DEBUG=DISP-TRACE): traces the task dispatcher
     * 0x8341..0x8353, which computes the CALAD target (0x8353 = CALAD A). When it
     * fails, A_L ends up at 0x70c3 instead of an entry of the branch table at
     * 0x8359 (B 0x8365/0x8394/...). A at the ENTRY (0x8341) is the index
     * preloaded by the caller (task selector / d_task_md): if A is already
     * garbage there, the fault is upstream and the dispatcher is innocent. */
    if (s->pc >= 0x8341 && s->pc <= 0x8354 && calypso_debug_enabled("DISP-TRACE")) {
        static unsigned disp_n = 0;
        if (disp_n++ < 300) {
            /* At 0x834d (op 0x6f07 = LD Smem<<1,A): compute the exact direct EA
             * (DP<<7)|dma and log the value read, which becomes A. Legitimate is
             * 0xff86 (-> A_L=0x8261); corrupt is 0xf6b7 (-> 0x70c3). */
            uint16_t ea = (uint16_t)(((s->st0 & 0x1FF) << 7) | (op & 0x7F));
            fprintf(stderr,
                "[c54x] DISP-TRACE PC=0x%04x op=0x%04x A=0x%010llx DP=0x%03x EA=0x%04x "
                "data[EA]=0x%04x d[9187]=0x%04x d[9207]=0x%04x AR1=%04x AR5=%04x insn=%u\n",
                s->pc, op, (unsigned long long)(s->a & 0xFFFFFFFFFFULL),
                (unsigned)(s->st0 & 0x1FF), ea, s->data[ea],
                s->data[0x9187], s->data[0x9207],
                s->ar[1], s->ar[5], s->insn_count);
        }
    }

    /* Coarse default: any MMR write happening inside this opcode handler
     * gets attributed to the opcode family so we can read the trace. */
    if (hi8 == 0xF3)                    s->writer_kind = WK_OPCODE_F3;
    else if (hi8 >= 0x80 && hi8 <= 0x8F) s->writer_kind = WK_OPCODE_8x;
    else if (hi8 == 0x77)                s->writer_kind = WK_OPCODE_77;
    else if (hi8 == 0x76)                s->writer_kind = WK_OPCODE_76;
    else                                 s->writer_kind = WK_OPCODE_OTHER;

    /* INTM-TRANS probe: logs every INTM 0->1 transition, with the PC that set it
     * and the stack return address, to name the caller of an orphan SSBX INTM.
     * Capped at 200 transitions, otherwise boot floods the log. */
    {
        static int prev_intm = -1;
        static unsigned itrans_total;
        int cur_intm = !!(s->st1 & ST1_INTM);
        if (prev_intm == 0 && cur_intm == 1) {
            itrans_total++;
            if (itrans_total <= 200) {
                uint16_t ret = s->data[s->sp];
                uint16_t ret_p1 = s->data[(uint16_t)(s->sp + 1)];
                if (calypso_debug_enabled("INTM-TRANS")) fprintf(stderr,
                        "[c54x] INTM-TRANS #%u 0->1 PC=0x%04x insn=%u SP=0x%04x "
                        "RET=%04x RET+1=%04x op=0x%04x IMR=0x%04x IFR=0x%04x\n",
                        itrans_total, s->pc, s->insn_count, s->sp,
                        ret, ret_p1, op, s->imr, s->ifr);
            }
        }
        prev_intm = cur_intm;
    }

    /* Detect when DSP enters DARAM code zone (0x0080-0x27FF) from ROM */
    {
        static uint16_t prev_pc = 0;
        static int daram_log = 0;
        if (s->pc >= 0x0080 && s->pc < 0x2800 && prev_pc >= 0x7000 && daram_log < 3) {
            C54_LOG("ROM->DARAM jump: 0x%04x->0x%04x op=0x%04x insn=%u SP=0x%04x XPC=%d",
                    prev_pc, s->pc, op, s->insn_count, s->sp, s->xpc);
            C54_LOG("  trail: %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x",
                    pc_ring[(pc_ring_idx-10)&255], pc_ring[(pc_ring_idx-9)&255],
                    pc_ring[(pc_ring_idx-8)&255], pc_ring[(pc_ring_idx-7)&255],
                    pc_ring[(pc_ring_idx-6)&255], pc_ring[(pc_ring_idx-5)&255],
                    pc_ring[(pc_ring_idx-4)&255], pc_ring[(pc_ring_idx-3)&255],
                    pc_ring[(pc_ring_idx-2)&255], pc_ring[(pc_ring_idx-1)&255]);
            daram_log++;
        }
        /* 0x7700 entry tracer: log when PC enters 0x7700 from elsewhere
         * (i.e. prev_pc != 0x76FF, the natural sequential predecessor).
         * Reveals which CALL/B/RET sources land here. PC HIST shows
         * 7700/7701 as the hottest non-loop addresses — find the callers. */
        if (s->pc == 0x7700 && prev_pc != 0x76FF) {
            static uint64_t e7700;
            e7700++;
            if (e7700 <= 30 || (e7700 % 5000) == 0) {
                C54_LOG("ENTER-7700 #%llu from PC=0x%04x A=%010llx B=%010llx SP=0x%04x trail: %04x %04x %04x %04x %04x",
                        (unsigned long long)e7700, prev_pc,
                        (unsigned long long)(s->a & 0xFFFFFFFFFFULL),
                        (unsigned long long)(s->b & 0xFFFFFFFFFFULL),
                        s->sp,
                        pc_ring[(pc_ring_idx-5)&255], pc_ring[(pc_ring_idx-4)&255],
                        pc_ring[(pc_ring_idx-3)&255], pc_ring[(pc_ring_idx-2)&255],
                        pc_ring[(pc_ring_idx-1)&255]);
            }
        }
        /* === ENTER-770c — dispatcher target, post-flag entry ===
         * The PROM0 idle dispatcher at 0xCC62..0xCC6F polls data[0x62];
         * when set, it CALAs to api[0x1f0c]=0x770c. So 0x770c is the
         * runtime task handler entry. If DARAM[0x60..0x70] never gets
         * set, this PC is never reached. Its appearance in the log is
         * therefore the binary signal that the dispatcher gate has
         * unlocked. Log every entry with full AR/SP/INTM context.
         * Cap to avoid log explosion if it ever runs hot. */
        if (s->pc == 0x770c) {
            static uint64_t e770c;
            e770c++;
            if (e770c <= 30 || (e770c % 1000) == 0) {
                C54_LOG("ENTER-770c #%llu from PC=0x%04x SP=0x%04x INTM=%d "
                        "ARs: %04x %04x %04x %04x %04x %04x %04x %04x insn=%u",
                        (unsigned long long)e770c, prev_pc, s->sp,
                        !!(s->st1 & ST1_INTM),
                        s->ar[0], s->ar[1], s->ar[2], s->ar[3],
                        s->ar[4], s->ar[5], s->ar[6], s->ar[7],
                        s->insn_count);
            }
        }
        /* MVDD-CASCADE probe (env-gated CALYPSO_PROBE_BOOTSTUB=1).
         * PC=0x8e8c op=0xe5ba is an MVDD-family write that lands garbage in NDB
         * cells (random values at d_fb_det where 0x001e is expected). Logs the
         * ARs, B and the source address read, to tell a genuine firmware
         * computation from corrupted indirect addressing. */
        if (s->pc == 0x8e8c) {
            static int probe_mvdd = -1;
            if (probe_mvdd < 0) {
                const char *e = cdbg_env("BOOTSTUB");
                probe_mvdd = (e && e[0] == '1') ? 1 : 0;
            }
            if (probe_mvdd) {
                static uint32_t e_mvdd;
                e_mvdd++;
                if (e_mvdd <= 50 || (e_mvdd % 1000) == 0) {
                    fprintf(stderr,
                            "[c54x] MVDD-CASCADE #%u PC=0x8e8c op=0x%04x SP=0x%04x "
                            "A=0x%010llx B=0x%010llx T=0x%04x "
                            "AR= %04x %04x %04x %04x %04x %04x %04x %04x "
                            "data[AR4]=0x%04x data[AR5]=0x%04x "
                            "trail: %04x %04x %04x %04x %04x %04x\n",
                            e_mvdd, op, s->sp,
                            (unsigned long long)(s->a & 0xFFFFFFFFFFULL),
                            (unsigned long long)(s->b & 0xFFFFFFFFFFULL),
                            s->t,
                            s->ar[0], s->ar[1], s->ar[2], s->ar[3],
                            s->ar[4], s->ar[5], s->ar[6], s->ar[7],
                            s->data[s->ar[4]], s->data[s->ar[5]],
                            pc_ring[(pc_ring_idx-6)&255], pc_ring[(pc_ring_idx-5)&255],
                            pc_ring[(pc_ring_idx-4)&255], pc_ring[(pc_ring_idx-3)&255],
                            pc_ring[(pc_ring_idx-2)&255], pc_ring[(pc_ring_idx-1)&255]);
                }
            }
        }

        /* === DF92-LOOP probe (env-gated CALYPSO_PROBE_BOOTSTUB=1) ===
         * Compute loop at PC=0xdf92-0xdfa3 = correlator accumulator with
         * 15x unrolled ADD *AR7+. Called via CALL 0xdfb1 from 0xdf90.
         * Probe at first PC=0xdf92 (loop entry) — log AR7, BRC, accumulator,
         * caller (from stack[SP]). If AR7 is corrupted or BRC mis-set, the
         * loop runs forever and blocks task=24 scheduling downstream.
         * Fire only on entries from non-loop-internal predecessors. */
        if (s->pc == 0xdf92 && (prev_pc < 0xdf90 || prev_pc > 0xdfa3)) {
            static int probe_df = -1;
            if (probe_df < 0) {
                const char *e = cdbg_env("BOOTSTUB");
                probe_df = (e && e[0] == '1') ? 1 : 0;
            }
            if (probe_df) {
                static uint32_t e_df;
                e_df++;
                if (e_df <= 30) {
                    fprintf(stderr,
                            "[c54x] DF92-LOOP #%u entry from PC=0x%04x prev_op=0x%04x "
                            "SP=0x%04x ret_addr=stk[SP]=0x%04x "
                            "A=0x%010llx B=0x%010llx "
                            "AR7=0x%04x BK=0x%04x BRC=0x%04x  "
                            "trail: %04x %04x %04x %04x %04x %04x\n",
                            e_df, prev_pc, s->prog[prev_pc],
                            s->sp, s->data[s->sp],
                            (unsigned long long)(s->a & 0xFFFFFFFFFFULL),
                            (unsigned long long)(s->b & 0xFFFFFFFFFFULL),
                            s->ar[7], s->bk, s->brc,
                            pc_ring[(pc_ring_idx-6)&255], pc_ring[(pc_ring_idx-5)&255],
                            pc_ring[(pc_ring_idx-4)&255], pc_ring[(pc_ring_idx-3)&255],
                            pc_ring[(pc_ring_idx-2)&255], pc_ring[(pc_ring_idx-1)&255]);
                }
            }
        }

        /* === BL-REENTRY probe (env-gated CALYPSO_PROBE_BOOTSTUB=1) ===
         * The DSP bootloader at PC=0xb41c polls data[0x0fff] for cmd code
         * 4 or 2. Legitimate loop entry comes from 0xb427 (BC NTC 0xb41c)
         * or 0xb433 (CALL 0xb41c). Any OTHER entry path indicates the DSP
         * has been routed back into the bootloader by mistake after boot
         * has completed — that's the post-cascade blocker. Logs prev_pc,
         * SP, stack contents, ARs, and a trail to identify the bad caller.
         * Caps: 50 events to avoid log flood. */
        if (s->pc == 0xb41c && prev_pc != 0xb427 && prev_pc != 0xb433) {
            static int probe_bl = -1;
            if (probe_bl < 0) {
                const char *e = cdbg_env("BOOTSTUB");
                probe_bl = (e && e[0] == '1') ? 1 : 0;
            }
            if (probe_bl) {
                static uint32_t e_bl;
                e_bl++;
                if (e_bl <= 50) {
                    fprintf(stderr,
                            "[c54x] BL-REENTRY #%u from PC=0x%04x prev_op=0x%04x "
                            "SP=0x%04x stk[SP..+3]= %04x %04x %04x %04x "
                            "data[0x0fff]=0x%04x data[0x0ffe]=0x%04x "
                            "AR= %04x %04x %04x %04x %04x %04x %04x %04x "
                            "trail: %04x %04x %04x %04x %04x %04x %04x %04x\n",
                            e_bl, prev_pc, s->prog[prev_pc],
                            s->sp,
                            s->data[(uint16_t)(s->sp+0)], s->data[(uint16_t)(s->sp+1)],
                            s->data[(uint16_t)(s->sp+2)], s->data[(uint16_t)(s->sp+3)],
                            s->data[0x0fff], s->data[0x0ffe],
                            s->ar[0], s->ar[1], s->ar[2], s->ar[3],
                            s->ar[4], s->ar[5], s->ar[6], s->ar[7],
                            pc_ring[(pc_ring_idx-8)&255], pc_ring[(pc_ring_idx-7)&255],
                            pc_ring[(pc_ring_idx-6)&255], pc_ring[(pc_ring_idx-5)&255],
                            pc_ring[(pc_ring_idx-4)&255], pc_ring[(pc_ring_idx-3)&255],
                            pc_ring[(pc_ring_idx-2)&255], pc_ring[(pc_ring_idx-1)&255]);
                }
            }
        }

        /* SEED-SOURCE probe (env-gated CALYPSO_PROBE_BOOTSTUB=1).
         * PC=0xf8de (CALA B -> 0x7700) is the single event that spawns the whole
         * boot-stub RET-loop cascade: [2026-05-24] one ENTER-7700 produced 435
         * entries to PC=0x0000. Captures the state BEFORE the CALA fires -- SP
         * and stack contents (what the later POPs will pull), A and B (B is the
         * jump target), AR0..AR7, ST0, ST1, and a 10-PC trail -- to tell whether
         * the function holding 0xf8de was itself called with a proper push, and
         * what the stack should have held when POPM ST0 and RCD UNC run at the
         * 0x7706/0x7707 dispatcher. */
        if (s->pc == 0xf8de) {
            static int probe_seed = -1;
            if (probe_seed < 0) {
                const char *e = cdbg_env("BOOTSTUB");
                probe_seed = (e && e[0] == '1') ? 1 : 0;
            }
            if (probe_seed) {
                static uint32_t e_seed;
                e_seed++;
                if (e_seed <= 50) {
                    fprintf(stderr,
                            "[c54x] SEED-SOURCE #%u PC=0xf8de op=0x%04x "
                            "SP=0x%04x  stk[SP..+7]= %04x %04x %04x %04x %04x %04x %04x %04x  "
                            "A=0x%010llx B=0x%010llx "
                            "AR= %04x %04x %04x %04x %04x %04x %04x %04x  "
                            "ST0=0x%04x ST1=0x%04x  "
                            "trail: %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x\n",
                            e_seed, op, s->sp,
                            s->data[(uint16_t)(s->sp+0)], s->data[(uint16_t)(s->sp+1)],
                            s->data[(uint16_t)(s->sp+2)], s->data[(uint16_t)(s->sp+3)],
                            s->data[(uint16_t)(s->sp+4)], s->data[(uint16_t)(s->sp+5)],
                            s->data[(uint16_t)(s->sp+6)], s->data[(uint16_t)(s->sp+7)],
                            (unsigned long long)(s->a & 0xFFFFFFFFFFULL),
                            (unsigned long long)(s->b & 0xFFFFFFFFFFULL),
                            s->ar[0], s->ar[1], s->ar[2], s->ar[3],
                            s->ar[4], s->ar[5], s->ar[6], s->ar[7],
                            s->st0, s->st1,
                            pc_ring[(pc_ring_idx-10)&255], pc_ring[(pc_ring_idx-9)&255],
                            pc_ring[(pc_ring_idx-8)&255],  pc_ring[(pc_ring_idx-7)&255],
                            pc_ring[(pc_ring_idx-6)&255],  pc_ring[(pc_ring_idx-5)&255],
                            pc_ring[(pc_ring_idx-4)&255],  pc_ring[(pc_ring_idx-3)&255],
                            pc_ring[(pc_ring_idx-2)&255],  pc_ring[(pc_ring_idx-1)&255]);
                }
            }
        }

        /* BOOTSTUB-ENTRY probe (env-gated CALYPSO_PROBE_BOOTSTUB=1).
         * Traces every entry to PC=0x0000 (boot stub LDMM SP,B + RET).
         * Re-entering the boot stub at runtime seeds the SP-wrap -> AR6=0 ->
         * IMR=0 cascade. Captures:
         *   - prev_pc + op@prev_pc  → who jumped to 0x0000
         *   - entry mechanism (RET-family / branch / other)
         *   - B accumulator (becomes SP via LDMM SP,B at 0x0000)
         *   - SP + stk[SP-1] (just-popped value if RET)
         *   - 6-entry PC trail (caller context). */
        if (s->pc == 0x0000) {
            static int probe_bootstub = -1;
            if (probe_bootstub < 0) {
                const char *e = cdbg_env("BOOTSTUB");
                probe_bootstub = (e && e[0] == '1') ? 1 : 0;
                if (probe_bootstub)
                    fprintf(stderr, "[c54x] PROBE-BOOTSTUB enabled\n");
            }
            if (probe_bootstub) {
                static uint32_t e0;
                e0++;
                if (e0 <= 200 || (e0 % 500) == 0) {
                    uint16_t prev_op = s->prog[prev_pc];
                    const char *mech;
                    if (prev_op == 0xFC00)                          mech = "RET";
                    else if (prev_op == 0xF273)                     mech = "RETD";
                    else if (prev_op == 0xF4EB || prev_op == 0xF4E3) mech = "RETE";
                    else if (prev_op == 0xF4E4 || prev_op == 0xF4E5) mech = "FRET";
                    else if ((prev_op & 0xFF00) == 0xF800)          mech = "B/CC";
                    else if ((prev_op & 0xFF00) == 0xF000)          mech = "F0xx";
                    else if (prev_op == 0xF074)                     mech = "CALL";
                    else                                            mech = "OTHER";
                    /* Just-popped slot is at SP-1 after RET (SP was incremented). */
                    uint16_t stk_just_popped = s->data[(uint16_t)(s->sp - 1)];
                    fprintf(stderr,
                            "[c54x] BOOTSTUB-ENTRY #%u prev_PC=0x%04x prev_op=0x%04x "
                            "mech=%s B=0x%010llx B[31:16]=0x%04x SP=0x%04x "
                            "stk[SP-1]=0x%04x trail: %04x %04x %04x %04x %04x %04x\n",
                            e0, prev_pc, prev_op, mech,
                            (unsigned long long)(s->b & 0xFFFFFFFFFFULL),
                            (unsigned)((s->b >> 16) & 0xFFFF),
                            s->sp, stk_just_popped,
                            pc_ring[(pc_ring_idx-6)&255], pc_ring[(pc_ring_idx-5)&255],
                            pc_ring[(pc_ring_idx-4)&255], pc_ring[(pc_ring_idx-3)&255],
                            pc_ring[(pc_ring_idx-2)&255], pc_ring[(pc_ring_idx-1)&255]);
                }
            }
        }
        /* INT3-VEC-TRACE probe (CALYPSO_DEBUG=INT3_VEC or ALL).
         * Triggers at PC=0xFFCC, the INT3 vector entry (IPTR=0x1FF + vec 19*4),
         * and captures the next 32 PCs to follow the ISR path: normal return
         * through RETE, RSBX INTM in the 0xA4D0+ region, or a drift down to the
         * 0x0000 boot stub, in which case the trace names the opcode that
         * derailed. */
        {
            static int trace_n = -1;        /* -1 = not active, >= 0 = countdown */
            static uint16_t trace_pcs[64];
            static uint16_t trace_ops[64];
            static int trace_idx = 0;
            static unsigned trace_dumps = 0;
            const unsigned DUMP_LIMIT = 8;   /* max 8 full traces logged */

            if (s->pc == 0xFFCC && trace_n < 0 && trace_dumps < DUMP_LIMIT) {
                trace_n = 32;                /* capture next 32 insns */
                trace_idx = 0;
                if (calypso_debug_enabled("INT3_VEC")) {
                    fprintf(stderr,
                            "[c54x] INT3-VEC-TRACE BEGIN #%u pc=0xFFCC "
                            "ST1=0x%04x INTM=%d IFR=0x%04x IMR=0x%04x SP=0x%04x\n",
                            trace_dumps + 1, s->st1, !!(s->st1 & ST1_INTM),
                            s->ifr, s->imr, s->sp);
                }
            }
            if (trace_n >= 0 && trace_idx < 64) {
                trace_pcs[trace_idx] = s->pc;
                trace_ops[trace_idx] = prog_fetch(s, s->pc);
                trace_idx++;
                trace_n--;
                if (trace_n <= 0) {
                    if (calypso_debug_enabled("INT3_VEC")) {
                        fprintf(stderr,
                                "[c54x] INT3-VEC-TRACE END #%u captured=%d "
                                "final_ST1=0x%04x INTM=%d\n",
                                trace_dumps + 1, trace_idx,
                                s->st1, !!(s->st1 & ST1_INTM));
                        for (int i = 0; i < trace_idx; i++) {
                            fprintf(stderr,
                                    "[c54x] INT3-VEC-TRACE #%u step %02d: "
                                    "PC=0x%04x op=0x%04x\n",
                                    trace_dumps + 1, i,
                                    trace_pcs[i], trace_ops[i]);
                        }
                        fflush(stderr);
                    }
                    trace_dumps++;
                    trace_n = -1;            /* re-arm */
                }
            }
        }

        /* D_FB_DET-WR-SITE probe at PC=0x8f51, the PC that writes d_fb_det.
         * Snapshots AR0..AR7, data[AR0..AR7], BK and A to identify the DARAM
         * region the FB-det correlator reads when producing its output, and
         * compares it with the BSP DMA target (default 0x3fb0..0x3fbf):
         * same region means the correlator reads the samples, a different one
         * means source and sink disagree and the DSP expects the samples
         * somewhere other than where the BSP writes them.
         *
         * COEFFS-TABLE-DUMP: once at startup and at every FB-det sweep, dumps
         * data[0x2bc0..0x2bcf], the region AR4 points at for the correlator
         * coefficients. [2026-05-14] data[AR4] was 0x0000 on all 50 captured
         * hits, i.e. the coefficient table is empty in memory; the dump tells
         * whether it is already empty at boot and whether anything ever fills
         * it. */
        {
            static int coeffs_log_n;
            static uint64_t coeffs_last_insn;
            bool first_call = (coeffs_log_n == 0);
            bool periodic = (s->insn_count - coeffs_last_insn > 1000000);
            bool at_8f51 = (s->pc == 0x8f51);
            if ((first_call || periodic || at_8f51) && coeffs_log_n < 30) {
                coeffs_log_n++;
                coeffs_last_insn = s->insn_count;
                C54_LOG("COEFFS-DUMP #%d insn=%u PC=0x%04x "
                        "data[0x2bc0..0x2bcF]= %04x %04x %04x %04x %04x %04x %04x %04x "
                        "%04x %04x %04x %04x %04x %04x %04x %04x",
                        coeffs_log_n, s->insn_count, s->pc,
                        s->data[0x2bc0], s->data[0x2bc1], s->data[0x2bc2], s->data[0x2bc3],
                        s->data[0x2bc4], s->data[0x2bc5], s->data[0x2bc6], s->data[0x2bc7],
                        s->data[0x2bc8], s->data[0x2bc9], s->data[0x2bca], s->data[0x2bcb],
                        s->data[0x2bcc], s->data[0x2bcd], s->data[0x2bce], s->data[0x2bcf]);
            }
        }
        if (s->pc == 0x8f51) {
            /* Per-fire log capped at 500 to cover several FB-det sweeps rather
             * than the first one only; the aggregate stats below count every
             * fire, the cap applies to the per-fire log alone. */
            static int dfbwr_n;
            g_fb_det_timing.fb_det_total++;
            uint16_t ar4 = s->ar[4];
            uint16_t dAR4 = s->data[ar4];
            uint16_t ar3 = s->ar[3];
            bool ar4_in_zone = (ar4 >= 0x2bc0 && ar4 <= 0x2bff);
            if (ar4_in_zone) g_fb_det_timing.fb_det_ar4_in_zone++;
            else             g_fb_det_timing.fb_det_ar4_outside++;
            if (dAR4 == 0x0000)      g_fb_det_timing.fb_det_dar4_zero++;
            else if (dAR4 == 0xfffe) g_fb_det_timing.fb_det_dar4_sentinel++;
            else                     g_fb_det_timing.fb_det_dar4_other++;
            /* Sweep boundary: AR3 dropping below the last observed value marks
             * a new sweep, so log the previous one (non-zero count, final A). */
            uint64_t A_lo = (uint64_t)(s->a & 0xFFFFFFFFFFULL);
            if (ar3 < g_fb_det_timing.last_ar3_at_fire
                && g_fb_det_timing.last_ar3_at_fire > 0) {
                C54_LOG("D_FB_DET-SWEEP id=%llu nonzero=%llu/50 "
                        "A_final=0x%010llx insn=%u",
                        (unsigned long long)g_fb_det_timing.sweep_id,
                        (unsigned long long)g_fb_det_timing.sweep_nonzero_count,
                        (unsigned long long)A_lo, s->insn_count);
                g_fb_det_timing.sweep_id++;
                g_fb_det_timing.sweep_nonzero_count = 0;
            }
            g_fb_det_timing.last_ar3_at_fire = ar3;
            if (dAR4 != 0) g_fb_det_timing.sweep_nonzero_count++;
            int64_t delta_compute = (int64_t)s->insn_count -
                                    (int64_t)g_fb_det_timing.last_compute_insn;
            int64_t delta_clear   = (int64_t)s->insn_count -
                                    (int64_t)g_fb_det_timing.last_clear_insn;
            int64_t delta_pattern = (int64_t)s->insn_count -
                                    (int64_t)g_fb_det_timing.last_pattern_insn;
            if (dfbwr_n++ < 500) {
                C54_LOG("D_FB_DET-WR-SITE #%d AR0..AR7=%04x %04x %04x %04x %04x %04x %04x %04x "
                        "data[AR0]=%04x data[AR1]=%04x data[AR2]=%04x "
                        "data[AR3]=%04x data[AR4]=%04x data[AR5]=%04x "
                        "data[AR6]=%04x data[AR7]=%04x "
                        "BK=%04x A=0x%010llx "
                        "ar4_in_zone=%d dcompute=%lld dclear=%lld dpattern=%lld "
                        "last_compute_addr=0x%04x last_clear_addr=0x%04x "
                        "last_pattern_addr=0x%04x "
                        "insn=%u",
                        dfbwr_n, s->ar[0], s->ar[1], s->ar[2], s->ar[3],
                        s->ar[4], s->ar[5], s->ar[6], s->ar[7],
                        s->data[s->ar[0]], s->data[s->ar[1]], s->data[s->ar[2]],
                        s->data[s->ar[3]], s->data[s->ar[4]], s->data[s->ar[5]],
                        s->data[s->ar[6]], s->data[s->ar[7]],
                        s->bk, (unsigned long long)(s->a & 0xFFFFFFFFFFULL),
                        ar4_in_zone ? 1 : 0,
                        (long long)delta_compute, (long long)delta_clear,
                        (long long)delta_pattern,
                        g_fb_det_timing.last_compute_addr,
                        g_fb_det_timing.last_clear_addr,
                        g_fb_det_timing.last_pattern_addr,
                        s->insn_count);
            }
            /* Summary every 100 fires of 0x8f51: AR4-in-zone distribution and
             * data[AR4] histogram over the whole history. */
            if ((g_fb_det_timing.fb_det_total % 100) == 0) {
                C54_LOG("D_FB_DET-STATS total=%llu "
                        "ar4_in_zone=%llu outside=%llu "
                        "dar4_zero=%llu sentinel=%llu other=%llu",
                        (unsigned long long)g_fb_det_timing.fb_det_total,
                        (unsigned long long)g_fb_det_timing.fb_det_ar4_in_zone,
                        (unsigned long long)g_fb_det_timing.fb_det_ar4_outside,
                        (unsigned long long)g_fb_det_timing.fb_det_dar4_zero,
                        (unsigned long long)g_fb_det_timing.fb_det_dar4_sentinel,
                        (unsigned long long)g_fb_det_timing.fb_det_dar4_other);
            }
        }
        /* READ-AMONT probe: at each trigger PC (the d_fb_det sites), emits the
         * per-range read delta since the previous trigger. A dominant LOW means
         * the correlator reads [0..0x3A3], APIRAM means the samples arrive
         * through the ARM-driven API RAM, WRAP means it runs on the PROM1 mirror
         * wrap, OTHER means an uncatalogued region. */
        if (!c54x_rapide) {
            read_stats_trigger_check(s);
            throughput_tick(s->insn_count);
        }
        /* WAIT-A21A probe: at PC=0xa21a, snapshots INTM, IMR and IFR.
         *   INTM=1, IFR=0,  IMR set -> the hardware is simply silent
         *   INTM=1, IFR!=0, IMR set -> a pending IRQ is being blocked (bug)
         *   INTM=0                  -> IRQs are serviceable, so the path to
         *                              0x7740 is broken upstream */
        /* CORR-PUBLISH-A probe: at PC=0x9ac0, just before STL A -> *AR2-,
         * snapshots the full A and AR2, the publication address. Capped at 200. */
        if (s->pc == 0x9ac0) {
            static unsigned cpa_log;
            const unsigned LIMIT = 200;
            if (cpa_log < LIMIT) {
                fprintf(stderr,
                        "[c54x] CORR-PUBLISH-A #%u PC=0x9ac0 "
                        "A=0x%010llx (A_lo=0x%04x A_hi=0x%04x) "
                        "AR2=0x%04x (= *AR2- target) insn=%u\n",
                        cpa_log,
                        (unsigned long long)(s->a & 0xFFFFFFFFFFULL),
                        (uint16_t)(s->a & 0xFFFF),
                        (uint16_t)((s->a >> 16) & 0xFFFF),
                        s->ar[2], s->insn_count);
                cpa_log++;
                if (cpa_log == LIMIT) {
                    fprintf(stderr,
                            "[c54x] CORR-PUBLISH-A log capped at %u\n",
                            LIMIT);
                }
            }
        }
        if (s->pc == 0xa21a) {
            static uint64_t a21a_total;
            a21a_total++;
            if (a21a_total <= 5 || (a21a_total % 100000) == 0) {
                C54_LOG("WAIT-A21A #%llu insn=%u INTM=%d IMR=0x%04x IFR=0x%04x "
                        "ST0=0x%04x ST1=0x%04x SP=0x%04x",
                        (unsigned long long)a21a_total, s->insn_count,
                        !!(s->st1 & ST1_INTM), s->imr, s->ifr,
                        s->st0, s->st1, s->sp);
            }
        }
        /* CALLER-7740 tracer: at the 0x7740 entry, logs the caller context.
         * data[SP] is the return address pushed by the preceding CALL/CALLD, and
         * INTM=1 means an IRQ context, which separates "called from an ISR" from
         * "called from regular flow" and lets the caller chain be walked back to
         * the IRQ vector. */
        if (s->pc == 0x7740) {
            static uint64_t enter7740;
            enter7740++;
            uint16_t ret_addr = s->data[s->sp];
            uint16_t ret_addr_p1 = s->data[(uint16_t)(s->sp + 1)];
            C54_LOG("ENTER-7740 #%llu insn=%u SP=%04x RET=%04x RET+1=%04x "
                    "INTM=%d XPC=%02x AR2=%04x AR3=%04x BK=%04x",
                    (unsigned long long)enter7740, s->insn_count,
                    s->sp, ret_addr, ret_addr_p1,
                    !!(s->st1 & ST1_INTM), s->xpc,
                    s->ar[2], s->ar[3], s->bk);
        }
        /* MAC-7700 tracer: at PC=0x7700 (MAC *AR2-, A) we want to know
         * what AR2 points at, what data[AR2] holds, T, and A before/after.
         * Helps determine if AR2 references the BSP RX zone (correlator
         * FB-det) or somewhere else. Also dumps full AR0-AR7 + ST0/ST1. */
        if (s->pc == 0x7700) {
            static uint64_t mac7700_total;
            mac7700_total++;
            if (mac7700_total <= 50 || (mac7700_total % 5000) == 0) {
                uint16_t ar2 = s->ar[2];
                uint16_t v_at_ar2 = s->data[ar2];
                C54_LOG("MAC-7700 #%llu AR2=0x%04x data[AR2]=0x%04x T=0x%04x "
                        "A_pre=%010llx ST0=0x%04x ST1=0x%04x",
                        (unsigned long long)mac7700_total, ar2, v_at_ar2,
                        s->t,
                        (unsigned long long)(s->a & 0xFFFFFFFFFFULL),
                        s->st0, s->st1);
                C54_LOG("MAC-7700 #%llu ARs: AR0=%04x AR1=%04x AR2=%04x AR3=%04x "
                        "AR4=%04x AR5=%04x AR6=%04x AR7=%04x SP=%04x",
                        (unsigned long long)mac7700_total,
                        s->ar[0], s->ar[1], s->ar[2], s->ar[3],
                        s->ar[4], s->ar[5], s->ar[6], s->ar[7], s->sp);
            }
        }
        /* RCD-75e8 tracer: when DSP arrives at PC=0x75e8 (cond=0x47 = LEQ),
         * log A. The RCD takes if A <= 0; report whether the loop will
         * exit this iteration. */
        if (s->pc == 0x75e8) {
            static uint64_t rcd75e8_total;
            rcd75e8_total++;
            if (rcd75e8_total <= 50 || (rcd75e8_total % 5000) == 0) {
                int64_t acc = sext40(s->a);
                C54_LOG("RCD-75e8 #%llu A=%010llx (signed=%lld) RCD-taken=%d AR2=%04x",
                        (unsigned long long)rcd75e8_total,
                        (unsigned long long)(s->a & 0xFFFFFFFFFFULL),
                        (long long)acc, (acc <= 0), s->ar[2]);
            }
        }
        prev_pc = s->pc;
        /* DARAM 0x1100-0x1130 tracer: dump first 64 visits */
        static int daram1110_log = 0;
        if (s->pc >= 0x1100 && s->pc <= 0x1130 && daram1110_log < 64) {
            C54_LOG("DARAM110x PC=0x%04x op=0x%04x A=%08x B=%08x AR2=%04x AR3=%04x AR4=%04x AR5=%04x BRC=%d",
                    s->pc, op, (uint32_t)s->a, (uint32_t)s->b,
                    s->ar[2], s->ar[3], s->ar[4], s->ar[5], s->brc);
            daram1110_log++;
        }
    }
    if (s->pc >= 0xFE00 && s->pc <= 0xFFFF && op == 0x0000) {
        static int nop_slide = 0;
        if (nop_slide == 0) {
            C54_LOG("NOP-SLIDE PC=0x%04x insn=%u SP=0x%04x PMST=0x%04x XPC=%d OVLY=%d",
                    s->pc, s->insn_count, s->sp, s->pmst, s->xpc, !!(s->pmst & PMST_OVLY));
            C54_LOG("  trail: %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x",
                    pc_ring[(pc_ring_idx-10)&255], pc_ring[(pc_ring_idx-9)&255],
                    pc_ring[(pc_ring_idx-8)&255], pc_ring[(pc_ring_idx-7)&255],
                    pc_ring[(pc_ring_idx-6)&255], pc_ring[(pc_ring_idx-5)&255],
                    pc_ring[(pc_ring_idx-4)&255], pc_ring[(pc_ring_idx-3)&255],
                    pc_ring[(pc_ring_idx-2)&255], pc_ring[(pc_ring_idx-1)&255]);
        }
        nop_slide++;
    }

    switch (hi4) {
    case 0xF:
        /* 0xF --- large group: branches, misc, short immediates */
        if (op == 0xF495) return consumed + s->lk_used;  /* NOP */

        /* XC n, cond — Execute Conditionally (SPRU172C p.4-198)
         * Opcode: 1111 11N1 CCCCCCCC
         * 0xFDxx = XC 1, cond (N=0, execute next 1 instruction)
         * 0xFFxx = XC 2, cond (N=1, execute next 2 instructions)
         * If condition true: execute normally. If false: skip n instructions. */
        if (hi8 == 0xFD || hi8 == 0xFF) {
            int n_insns = (hi8 == 0xFF) ? 2 : 1;
            uint8_t cc = op & 0xFF;
            bool cond = false;
            /* Evaluate condition code per SPRU172C condition table */
            /* Conditions can be combined (OR'd bits), but common single conditions: */
            if (cc == 0x00)      cond = true;                          /* UNC */
            else if (cc == 0x0C) cond = (s->st0 & ST0_C) != 0;       /* C */
            else if (cc == 0x08) cond = !(s->st0 & ST0_C);            /* NC */
            else if (cc == 0x30) cond = (s->st0 & ST0_TC) != 0;       /* TC */
            else if (cc == 0x20) cond = !(s->st0 & ST0_TC);           /* NTC */
            else if (cc == 0x45) cond = (sext40(s->a) == 0);          /* AEQ */
            else if (cc == 0x44) cond = (sext40(s->a) != 0);          /* ANEQ */
            else if (cc == 0x46) cond = (sext40(s->a) > 0);           /* AGT */
            else if (cc == 0x42) cond = (sext40(s->a) >= 0);          /* AGEQ */
            else if (cc == 0x43) cond = (sext40(s->a) < 0);           /* ALT */
            else if (cc == 0x47) cond = (sext40(s->a) <= 0);          /* ALEQ */
            else if (cc == 0x4D) cond = (sext40(s->b) == 0);          /* BEQ */
            else if (cc == 0x4C) cond = (sext40(s->b) != 0);          /* BNEQ */
            else if (cc == 0x4E) cond = (sext40(s->b) > 0);           /* BGT */
            else if (cc == 0x4A) cond = (sext40(s->b) >= 0);          /* BGEQ */
            else if (cc == 0x4B) cond = (sext40(s->b) < 0);           /* BLT */
            else if (cc == 0x4F) cond = (sext40(s->b) <= 0);          /* BLEQ */
            else if (cc == 0x70) { cond = (s->st0 & ST0_OVA) != 0; s->st0 &= ~ST0_OVA; }    /* AOV, cleared once tested */
            else if (cc == 0x60) { cond = !(s->st0 & ST0_OVA);       s->st0 &= ~ST0_OVA; }    /* ANOV */
            else if (cc == 0x78) { cond = (s->st0 & ST0_OVB) != 0; s->st0 &= ~ST0_OVB; }    /* BOV */
            else if (cc == 0x68) { cond = !(s->st0 & ST0_OVB);       s->st0 &= ~ST0_OVB; }    /* BNOV */
            else {
                /* Combined conditions: OR the individual condition bits */
                cond = false;
                if (cc & 0x0C) cond |= ((cc & 0x04) ? (s->st0 & ST0_C) != 0 : !(s->st0 & ST0_C));
                if (cc & 0x30) cond |= ((cc & 0x10) ? (s->st0 & ST0_TC) != 0 : !(s->st0 & ST0_TC));
                if (cc & 0x40) {
                    int64_t acc = (cc & 0x08) ? s->b : s->a;
                    int c3 = cc & 0x07;
                    switch (c3) {
                    case 0x5: cond |= (sext40(acc) == 0); break;
                    case 0x4: cond |= (sext40(acc) != 0); break;
                    case 0x6: cond |= (sext40(acc) > 0); break;
                    case 0x2: cond |= (sext40(acc) >= 0); break;
                    case 0x3: cond |= (sext40(acc) < 0); break;
                    case 0x7: cond |= (sext40(acc) <= 0); break;
                    default: cond = true; break;
                    }
                }
                if (cc & 0x70 && !(cc & 0x40)) {
                    if (cc & 0x08) cond |= (s->st0 & ST0_OVB) != 0;
                    else           cond |= (s->st0 & ST0_OVA) != 0;
                }
            }
            if (!cond) {
                /* Skip n instructions — count consumed words for skipped insns */
                /* Each skipped insn is 1 word (simplified — multi-word insns rare after XC) */
                return 1 + n_insns;
            }
            return consumed + s->lk_used;  /* condition true: just advance past XC, execute next normally */
        }

        /* F4E2 = BACC A, F5E2 = BACC B (per tic54x-opc.c, mask 0xFEFF) */
        /* F4E3 = CALA A, F5E3 = CALA B — push next-PC, jump to acc low 16 bits */
        /* DYN-CALL tracer: targets are computed at runtime, invisible to static
         * disasm. Log every BACC/CALA, plus an extra hot tag when the target
         * lands in any FB-det zone (PROM0 0x77xx-0x79xx, 0x88xx, 0xa0xx-0xa1xx). */
        if (op == 0xF4E2 || op == 0xF5E2 || op == 0xF4E3 || op == 0xF5E3) {
            int is_b = (op & 0x0100) != 0;
            int is_call = (op & 1) != 0;
            uint16_t tgt = (uint16_t)((is_b ? s->b : s->a) & 0xFFFF);
            uint16_t src_pc = s->pc;
            /* FB-ENERGY reroute: the live FB dispatch (CALA at 0xb01e) resolves to
             * the SYMBOL correlator 0x8d00 / stub 0xab38, which never touches the
             * IQ buffer 0x2a00 nor the 0xa076 kernel. The CALA is redirected to
             * the FB ENERGY correlator entry 0x94f5 (0x9500 sets AR4=0x2a00 ->
             * f274 a033 -> a040 -> f273 a076). On that path AR5=0x2c00 is the
             * reference and the IQ 0x2a00 sits in AR4/AR1, NOT in AR5.
             * Gate CALYPSO_FB_ENERGY; entry override CALYPSO_FB_CORR_ENTRY. */
            if (is_call && src_pc == 0xb01e) {
                /* CALA-FB: the dispatcher's NATIVE target plus d_task_md, logged
                 * BEFORE any reroute. */
                { static int _cf = -1; static unsigned _cfn = 0;
                  if (_cf < 0) _cf = calypso_gate("CALYPSO_CALA_FB", 0);
                  if (_cf && _cfn < 40) { _cfn++;
                      fprintf(stderr, "[c54x] CALA-FB tgt=0x%04x md0804=%u md0818=%u md058a=%u "
                              "(0x7700=routine resultat FB, 0xab38=?, 0x8d00=corr symbole) "
                              "A=0x%06llx insn=%u\n", tgt, (unsigned)s->data[0x0804],
                              (unsigned)s->data[0x0818], (unsigned)s->data[0x058a],
                              (unsigned long long)(s->a & 0xFFFFFFULL), s->insn_count); } }
                static int _fbe = -1; static uint16_t _fbentry = 0x94f5;
                if (_fbe < 0) {
                    /* @BEQUILLE — FB_ENERGY + FB_CORR_ENTRY  (CALYPSO_FB_ENERGY,
                     *                CALYPSO_FB_CORR_ENTRY, default OFF)
                     *   masks   : the missing NATIVE dispatch to the correlator.
                     *             Execution is REROUTED to 0x9500 / 0x94f5 instead
                     *             of letting the firmware get there on its own.
                     *   remove  : once BRINT0 (vec 21) is served and the native
                     *             path reaches the correlator by itself.
                     *   ⚠️ Any measurement taken under this reroute is a measurement
                     *   UNDER CRUTCH: on the pure native path this stage is NEVER
                     *   executed ([2026-07-28] CALYPSO_WATCH_9F00_RD = 0). */
                    const char *_e = calypso_getenv("CALYPSO_FB_ENERGY"); _fbe = (_e && atoi(_e) > 0) ? 1 : 0;
                    const char *_p = calypso_getenv("CALYPSO_FB_CORR_ENTRY");
                    if (_p && *_p) _fbentry = (uint16_t)strtol(_p, NULL, 0);
                }
                if (_fbe && s->data[0x058a] == 5) {   /* d_task_md == 5 (FB command) */
                    static unsigned _fbn = 0;
                    if (_fbn++ < 32)
                        fprintf(stderr, "[c54x] FB-ENERGY-REROUTE CALA@0xb01e tgt 0x%04x -> 0x%04x insn=%u\n",
                                tgt, _fbentry, s->insn_count);
                    tgt = _fbentry;
                }
            }
            /* Self-CALA black hole: fires ONCE when a CALA targets its own PC in
             * 0x7000-0x70FF (the 0x70c3 black hole). Reports the inherited DP, the
             * LUT slot read by the dispatcher at 0x834d (ea and value), and the
             * POPM ST0 and LDP that set that DP: the whole culprit chain in one
             * line, unlike DISP-TRACE. */
            if (is_call && tgt == src_pc && tgt >= 0x7000 && tgt <= 0x70FF) {
                static int bh_logged = 0;
                if (!bh_logged++) {
                    fprintf(stderr,
                        "[c54x] *** BLACKHOLE-CALA *** tgt=0x%04x DP=0x%03x "
                        "SP=0x%04x prevPC=0x%04x dispLUT{ea=0x%04x val=0x%04x} "
                        "lastST0w{pc=0x%04x val=0x%04x prev=0x%04x} "
                        "lastLDP{pc=0x%04x val=0x%03x} insn=%u\n",
                        tgt, (unsigned)(s->st0 & 0x1FF), s->sp, g_prev_pc,
                        g_disp_lut_ea, g_disp_lut_val,
                        g_last_st0w_pc, g_last_st0w_val, g_last_st0w_prev,
                        g_last_ldp_pc, g_last_ldp_val, s->insn_count);
                    /* Stack around SP: shows the orphan word (0xf487) and its
                     * neighbours. A plausible ST0 (0x0xxx/0x4xxx, sane DP) sitting
                     * at SP+-1 means a one-word imbalance; stacked PC-shaped words
                     * (0xf4xx/0x7xxx) mean a cumulative drain. */
                    fprintf(stderr, "[c54x]     STACK around SP=0x%04x :", s->sp);
                    for (int k = -2; k <= 9; k++) {
                        uint16_t a = (uint16_t)(s->sp + k);
                        fprintf(stderr, " %s[%04x]=%04x",
                                k == 0 ? ">" : "", a, s->data[a]);
                    }
                    fprintf(stderr, "\n");
                    /* SP-event ring: the last 28 push/pop events (pc:op delta).
                     * A push (delta<0) with no matching pop is the leak. */
                    fprintf(stderr, "[c54x]     SP-EVENTS net_words=%lld pushes=%llu pops=%llu (récents, anciens→récents):\n[c54x]    ",
                            (long long)g_sp_ledger.net_words,
                            (unsigned long long)g_sp_ledger.sp_pushes,
                            (unsigned long long)g_sp_ledger.sp_pops);
                    for (int k = 28; k >= 1; k--) {
                        struct sp_evt *e = &g_spring[(g_spring_idx - k) & 63];
                        fprintf(stderr, " %04x:%04x%+d", e->pc, e->op, e->delta);
                    }
                    fprintf(stderr, "\n");
                    fflush(stderr);
                }
            }
            int fb_zone = (tgt >= 0x7730 && tgt <= 0x7990) ||
                          (tgt >= 0x8800 && tgt <= 0x88FF) ||
                          (tgt >= 0xA000 && tgt <= 0xA1FF);
            static uint64_t dyn_total = 0;
            static uint64_t dyn_fb = 0;
            dyn_total++;
            if (fb_zone) dyn_fb++;
            /* When OVLY=1 and src_pc in [0x80, 0x2800], the executed opcode
             * comes from data[] (DARAM), not prog[]. Reflect this in the
             * dump so we see the *actual* bytes that drove the CALA. */
            int ovly_active = (s->pmst & PMST_OVLY) && src_pc >= 0x80 && src_pc < 0x2800;
            uint16_t m0 = ovly_active ? s->data[(uint16_t)(src_pc - 2)] : s->prog[(uint16_t)(src_pc - 2)];
            uint16_t m1 = ovly_active ? s->data[(uint16_t)(src_pc - 1)] : s->prog[(uint16_t)(src_pc - 1)];
            uint16_t m2 = ovly_active ? s->data[src_pc] : s->prog[src_pc];
            uint16_t m3 = ovly_active ? s->data[(uint16_t)(src_pc + 1)] : s->prog[(uint16_t)(src_pc + 1)];
            if (dyn_total <= 200 || fb_zone || (dyn_total % 5000) == 0) {
                C54_LOG("DYN-CALL #%llu %s%c src=0x%04x tgt=0x%04x A=%010llx B=%010llx SP=0x%04x mem[%c]=%04x %04x %04x %04x%s",
                        (unsigned long long)dyn_total,
                        is_call ? "CALA" : "BACC",
                        is_b ? 'B' : 'A',
                        src_pc, tgt,
                        (unsigned long long)(s->a & 0xFFFFFFFFFFULL),
                        (unsigned long long)(s->b & 0xFFFFFFFFFFULL),
                        s->sp,
                        ovly_active ? 'D' : 'P',
                        m0, m1, m2, m3,
                        fb_zone ? " *FB-ZONE*" : "");
            }
            if (is_call) {
                uint16_t ret_pc = src_pc + 1;
                s->sp = (s->sp - 1) & 0xFFFF;
                data_write(s, s->sp, ret_pc);
            }
            s->pc = tgt;
            return 0;
        }
        /* F4E6 = FBACC A   FL_FAR  (far branch on acc, no push, no delay)
         * F4E7 = FCALA A   FL_FAR  (far call on acc, pushes 2 words, no delay)
         * F5E6 = FBACC B / F5E7 = FCALA B  (accumulator B variants)
         *
         * Per binutils tic54x-opc.c (FL_FAR flag) and SPRU172C:
         *   XPC = A(22:16), PC = A(15:0). FCALA pushes XPC then ret_pc (PC+1),
         *   the order FRET (F4E4) expects when popping PC first, then XPC.
         * Same semantics as the existing FCALAD/FBACCD (F6E6/F6E7), without the
         * delay slots.
         *
         * ⚠️ This test must stay AHEAD of the F4E0-F4FF block below, which would
         * otherwise swallow these four opcodes as NOPs and derail control flow
         * silently. */
        if (op == 0xF4E6 || op == 0xF4E7 || op == 0xF5E6 || op == 0xF5E7) {
            int is_b    = (op & 0x0100) != 0;
            int is_call = (op & 1) != 0;
            int64_t acc = is_b ? s->b : s->a;
            uint16_t tgt     = (uint16_t)(acc & 0xFFFF);
            uint8_t  new_xpc = (uint8_t)((acc >> 16) & 0xFF);
            if (new_xpc > 3) new_xpc &= 3;
            static uint64_t facc_total;
            facc_total++;
            if (facc_total <= 30 || (facc_total % 5000) == 0) {
                C54_LOG("%s%c FAR #%llu PC=0x%04x → XPC=%u PC=0x%04x "
                        "(A=%010llx SP=0x%04x was XPC=%u)",
                        is_call ? "FCALA" : "FBACC",
                        is_b ? 'B' : 'A',
                        (unsigned long long)facc_total,
                        s->pc, new_xpc, tgt,
                        (unsigned long long)(acc & 0xFFFFFFFFFFULL),
                        s->sp, s->xpc & 0x3);
            }
            if (is_call) {
                /* FCALA pushes XPC first (deeper in the stack), then ret_pc on
                 * top; FRET (F4E4) pops PC first, then XPC. */
                s->sp = (s->sp - 1) & 0xFFFF;
                data_write(s, s->sp, s->xpc);
                uint16_t ret_pc = (uint16_t)(s->pc + 1);
                s->sp = (s->sp - 1) & 0xFFFF;
                data_write(s, s->sp, ret_pc);
            }
            s->xpc = new_xpc;
            s->pc  = tgt;
            return 0;
        }
        /* F4E0-F4FF catch-all: NOP by default, except for the opcodes in that
         * range that have their own handler -- F4E1 IDLE 1, F4E2 BACC A,
         * F4E3 CALA A, F4E4 FRET, F4E6 FBACC, F4E7 FCALA, F4EB RETE.
         * ⚠️ Without the F4E1 exception, IDLE 1 is silently swallowed as a NOP,
         * the DSP never raises s->idle, the IRQ handler never dispatches and
         * INTM stays stuck at 1. */
        if (op >= 0xF4E0 && op <= 0xF4FF &&
            op != 0xF4E1 && op != 0xF4E4 && op != 0xF4EB) {
            return consumed + s->lk_used;
        }
        /* F4EB = RETE (return from interrupt). Pop PC, pop XPC iff APTS=1.
         * Symmetric with c54x_interrupt_ex push order. */
        if (op == 0xF4EB) {
            uint16_t prev_xpc = s->xpc;
            /* @BEQUILLE — CALYPSO_RETE_POP2 (default 0).
             *
             * Both interrupt entry paths in calypso_c54x.c push UNCONDITIONALLY:
             *     s->sp--; data_write(s, s->sp, s->pc + 1);
             *     s->sp--; data_write(s, s->sp, s->xpc);
             *     g_sp_ledger.net_words += 2;
             * so entry is always 2 words, while the pop below stayed conditional on
             * a heuristic (does the top of stack look like an XPC?). An
             * unconditional push against a guessed pop cannot be symmetric in every
             * case.
             *
             * [2026-07-30] Reference bench, no crutch enabled:
             *   ORPHAN-RETURN pc=0x0107 op=0xf4eb SP=0x5aa8 -> ret_tgt=0xddfb
             *   net_words=-18880, i.e. -4 words PER FRAME (events spaced 65536
             *   instructions apart), reported as over-pop above SP_base. 0x0107 is
             *   the return of the frame interrupt itself, so the context is damaged
             *   once per frame and no task survives from one frame to the next.
             *
             * masks   : the asymmetry itself. =1 pops 2 words unconditionally, an
             *           exact mirror of the entry.
             * remove  : once the default is flipped. It stays 0 for now because it
             *           changes the interrupt return of every profile; validate
             *           with CALYPSO_RETE_POP2=1 and CALYPSO_ORPHAN=1, where the
             *           ORPHAN probe must go silent and net_words stay bounded.
             */
            static int _rp2 = -1;
            if (_rp2 < 0) {
                _rp2 = calypso_gate("CALYPSO_RETE_POP2", 0);
                if (_rp2)
                    fprintf(stderr, "[c54x] RETE_POP2=1 : depilement de 2 mots "
                            "inconditionnel, symetrique du push d'entree\n");
            }
            uint16_t top = data_read(s, s->sp);
            if (_rp2) {
                s->xpc = top & 3; s->sp++;  /* pop XPC, always, mirroring the push */
            } else if (top <= 3) {          /* historical heuristic */
                s->xpc = top & 3; s->sp++;
            }
            uint16_t ra = data_read(s, s->sp); s->sp++;   /* pop PC */
            s->st1 &= ~ST1_INTM;

            /* RETE-AUDIT (CALYPSO_RETE_AUDIT=1, default 0).
             *
             * Are there MORE RETURNS THAN ENTRIES? Interrupt entry pushes 2 words
             * and RETE pops 2, so equal counts balance exactly. ORPHAN however
             * measures a constant over-pop of -4 words per frame: a RETE executed
             * without a preceding interrupt pops 2 words nobody pushed, and two
             * such per frame give exactly -4.
             *
             * Both sides are counted here and the gap printed; `irq_entries` is
             * already kept by the ledger at each real dispatch. Capped at one line
             * every 500 RETE, 40 lines total.
             */
            {
                static int _ra_g = -1; static unsigned long long _ra_n = 0;
                static unsigned _ra_log = 0;
                if (_ra_g < 0) _ra_g = calypso_gate("CALYPSO_RETE_AUDIT", 0);
                if (_ra_g) {
                    static unsigned long long _ra_vec = 0;   /* RETE inside the vector zone */
                    _ra_n++;
                    /* RETE has two distinct uses and only one may be compared with
                     * irq_entries. Measured: 500 RETE for 167 irq_entries, because
                     * CALA (0xF4E3) pushes only ONE word (the return address) and the
                     * ROM uses it heavily, so most RETE return from a CALA-called
                     * handler rather than from an interrupt. That is legitimate and
                     * the pop heuristic handles it: a pushed XPC is 0..3 while a
                     * return address is >= 0x0080, so `top <= 3` pops 2 or 1
                     * correctly.
                     * Only the case ORPHAN points at is suspect: a RETE whose PC is
                     * in the VECTOR zone, hence an interrupt return, with a virgin
                     * stack above SP_base, hence no matching entry. That one alone is
                     * counted against irq_entries. Zone: pc < 0x0200 (post-boot table
                     * at 0x0080, 4-word slots, plus the OVLY trampolines). */
                    if (s->pc < 0x0200) {
                        _ra_vec++;
                    }
                    if ((_ra_n % 500) == 0 && _ra_log < 40) {
                        _ra_log++;
                        long long ecart = (long long)_ra_vec -
                                          (long long)g_sp_ledger.irq_entries;
                        fprintf(stderr, "[c54x] RETE-AUDIT total=%llu dont_vecteurs=%llu "
                                "irq_entries=%llu ecart_vect=%+lld net_words=%lld "
                                "insn=%u%s\n",
                                (unsigned long long)_ra_n,
                                (unsigned long long)_ra_vec,
                                (unsigned long long)g_sp_ledger.irq_entries,
                                ecart, (long long)g_sp_ledger.net_words,
                                s->insn_count,
                                ecart > 0 ? "  <== RETOURS D'IT SANS ENTREE" : "");
                    }
                }
            }
                if (g_vec28_tracing) {
                    g_vec28_trace_pops++;
                    fprintf(stderr, "[c54x] VEC28-STACK-TRACE #%u RETE(2w,symmetric) PC=0x%04x "
                            "popped=0x%04x words=2 SP_before=0x%04x SP_after=0x%04x "
                            "insn=%u\n", g_vec28_trace_pops, s->pc, ra,
                            (uint16_t)(s->sp - 2), s->sp, s->insn_count);
                    if (s->sp >= (uint16_t)(g_vec28_sp_entry + 2)) {
                        fprintf(stderr, "[c54x] VEC28-STACK-TRACE DISARMED SP=0x%04x "
                                "back at or above pre-interrupt level 0x%04x\n",
                                s->sp, g_vec28_sp_entry);
                        g_vec28_tracing = false;
                    }
                }
            /* INT3-CYCLE-TRACE end-good hook NOT here : firmware exits ISR via
             * POPM ST1 + RCD (not RETE 0xF4EB), so this path is dead. Hook
             * moved to generic INTM 1→0 detector below — catches all idioms. */
            {
                static uint64_t rete_count;
                rete_count++;
                if (rete_count <= 20 || (rete_count % 100) == 0)
                    C54_LOG("RETE #%llu PC=0x%04x -> ra=0x%04x XPC=%u→%u SP=0x%04x",
                            (unsigned long long)rete_count,
                            s->pc, ra, prev_xpc, s->xpc, s->sp);
            }
            s->pc = ra; return 0;
        }
        /* 0xF4E4 = FRET (far return). Pops PC and XPC unconditionally.
         * Per binutils tic54x-opc.c (FL_FAR flag) and SPRU172C Table 2-15:
         *   FRET[D]: XPC = TOS, ++SP, PC = TOS, ++SP
         * Symmetric with the FCALL/FCALLD push, which is unconditional too.
         * ⚠️ The pop must NOT be made conditional on PMST bit 4: that bit is AVIS
         * (address visibility) per SPRU131G, not APTS, and has no stack
         * semantics. Gating on it made FRET skip the XPC pop when AVIS=0 and
         * unbalanced the stack against FCALL FAR, which always pushes 2. */
        if (op == 0xF4E4) {
            uint16_t ra = data_read(s, s->sp); s->sp++;
            uint16_t prev_xpc = s->xpc;
            uint16_t nx = data_read(s, s->sp); s->sp++;
                if (g_vec28_tracing) {
                    g_vec28_trace_pops++;
                    fprintf(stderr, "[c54x] VEC28-STACK-TRACE #%u FRET(2w,symmetric) PC=0x%04x "
                            "popped=0x%04x words=2 SP_before=0x%04x SP_after=0x%04x "
                            "insn=%u\n", g_vec28_trace_pops, s->pc, ra,
                            (uint16_t)(s->sp - 2), s->sp, s->insn_count);
                    if (s->sp >= (uint16_t)(g_vec28_sp_entry + 2)) {
                        fprintf(stderr, "[c54x] VEC28-STACK-TRACE DISARMED SP=0x%04x "
                                "back at or above pre-interrupt level 0x%04x\n",
                                s->sp, g_vec28_sp_entry);
                        g_vec28_tracing = false;
                    }
                }
            if (nx > 3)   /* PROM0..3 = 4 pages; page 3 is legitimate (masked & 3) */
                C54_DBG("XPC-OOR", "FRET xpc=0x%04x PC=0x%04x SP=0x%04x insn=%u",
                        nx, s->pc, s->sp, s->insn_count);
            s->xpc = nx & 3;
            {
                static uint64_t fret_count;
                fret_count++;
                if (fret_count <= 30 || (fret_count % 1000) == 0)
                    C54_LOG("FRET #%llu PC=0x%04x -> ra=0x%04x XPC=%u→%u SP=0x%04x",
                            (unsigned long long)fret_count,
                            s->pc, ra, prev_xpc, s->xpc, s->sp);
            }
            s->pc = ra;
            return 0;
        }
        /* IDLE 1/2/3: 0xF4E1, 0xF5E1, 0xF6E1, 0xF7E1 (mask 0xFCFF) */
        if ((op & 0xFCFF) == 0xF4E1) {
            int level = ((op >> 8) & 0x3) + 1;
            static int idle_log = 0;
            if (idle_log < 20)
                C54_LOG("IDLE%d @0x%04x INTM=%d IMR=0x%04x SP=0x%04x insns=%u XPC=%d",
                        level, s->pc, !!(s->st1 & ST1_INTM),
                        s->imr, s->sp, s->insn_count, s->xpc);
            idle_log++;
            if (s->pc >= 0x8000 && s->pc < 0x8020) {
                return consumed + s->lk_used;
            }
            s->idle = true;
            return 0;
        }
        /* ================================================================
         * F[4-7]xx generic accumulator family — promoted from F4 block
         * to handle F5/F6/F7 variants. Handlers use bits 8/9 for src/dst,
         * with masks FCE0/FCFF/FEFF naturally covering all 4 combinations
         * (A->A, B->A, A->B, B->B). The matching handler bodies remain
         * inside the F4 block as dead code (never reached for arith ops
         * because of the early return here). 2026-04-28.
         * ================================================================ */
            /* F483/F583: SAT src (mask FEFF, 1 word) */
            if ((op & 0xFEFF) == 0xF483) {
                int src = (op >> 8) & 1;
                int64_t *acc = src ? &s->b : &s->a;
                int64_t val = sext40(*acc);
                uint16_t ovbit = src ? ST0_OVB : ST0_OVA;
                if (val > 0x7FFFFFFFLL) { *acc = sext40(0x7FFFFFFFLL); s->st0 |= ovbit; }
                else if (val < -0x80000000LL) { *acc = sext40(-0x80000000LL); s->st0 |= ovbit; }
                else s->st0 &= ~ovbit;          /* SPRU172C 4-153: "Affects OVsrc" */
                return consumed + s->lk_used;
            }

            /* F484/F584: NEG src[,dst] (mask FCFF, 1 word) */
            if ((op & 0xFCFF) == 0xF484) {
                int src, dst; c54x_f4_srcdst(op, &src, &dst);   /* FIX_F4XX_SRCDST */
                int64_t val = sext40(src ? s->b : s->a);
                if (dst) s->b = sext40(-val); else s->a = sext40(-val);
                return consumed + s->lk_used;
            }

            /* F485/F585: ABS src[,dst] (mask FCFF, 1 word) */
            if ((op & 0xFCFF) == 0xF485) {
                int src, dst; c54x_f4_srcdst(op, &src, &dst);   /* FIX_F4XX_SRCDST */
                int64_t val = sext40(src ? s->b : s->a);
                if (val < 0) val = -val;
                if (dst) s->b = sext40(val); else s->a = sext40(val);
                return consumed + s->lk_used;
            }

            /* F48C/F58C: MPYA dst (mask FEFF, 1 word).
             * SPRU172C: `MPYA dst` -> dst = T x A(32-16). It ASSIGNS, it does not
             * accumulate; MAC accumulates, MPY/MPYA assigns. The Smem form
             * `MPYA Smem` (0x3100) and `SQUR A,dst` (0xF48D) below both assign too.
             * [2026-08-22] Where accumulating bites: the TOA chain in PROM0
             *   0x7944 add *AR4,A ; 0x7945 sub #2,A ; 0x7947 mpya A ; 0x7948 add B
             *   ... 0x795a stl B -> a_sync_demod[D_TOA]
             * A is non-zero at 0x7947, so accumulating falsifies the published TOA.
             * (At 0x7920 `mpya B` the difference is nil: B holds d_fb_mode = 0.) */
            if ((op & 0xFEFF) == 0xF48C) {
                int dst = (op >> 8) & 1;
                int64_t prod = (int64_t)(int16_t)s->t * (int64_t)(int16_t)((s->a >> 16) & 0xFFFF);
                if (s->st1 & ST1_FRCT) prod <<= 1;
                if (dst) s->b = sext40(prod); else s->a = sext40(prod);
                return consumed + s->lk_used;
            }

            /* F48D/F58D: SQUR A,dst (mask FEFF, 1 word) */
            if ((op & 0xFEFF) == 0xF48D) {
                int dst = (op >> 8) & 1;
                int16_t ah = (int16_t)((s->a >> 16) & 0xFFFF);
                int64_t prod = (int64_t)ah * (int64_t)ah;
                if (s->st1 & ST1_FRCT) prod <<= 1;
                if (dst) s->b = sext40(prod); else s->a = sext40(prod);
                return consumed + s->lk_used;
            }

            /* F48E/F58E: EXP src (mask FEFF, 1 word)
             * Count leading sign bits of accumulator, store in T */
            if ((op & 0xFEFF) == 0xF48E) {
                int src = (op >> 8) & 1;
                int64_t val = sext40(src ? s->b : s->a);
                int exp = 0;
                if (val == 0) { exp = 0; }              /* SPRU172C 4-58: src = 0 -> T = 0 */
                else if (val == -1) { exp = 31; }
                else {
                    uint64_t uv = (val < 0) ? ~val : val;
                    uv &= 0xFFFFFFFFFFULL;
                    /* Count leading zeros from bit 38 down */
                    for (int i = 38; i >= 0; i--) {
                        if (uv & (1ULL << i)) break;
                        exp++;
                    }
                    exp -= 8; /* EXP = leading sign bits - 8 */
                }
                s->t = (uint16_t)(int16_t)exp;
                return consumed + s->lk_used;
            }

            /* F486/F586: MAX src (mask FEFF, 1 word) — keep max of A,B.
             * binutils tic54x-opc.c: "max" 1,1,1, 0xF486, 0xFEFF (0xF492 is roltc).
             */
            if ((op & 0xFEFF) == 0xF486) {
                /* FIX_MAXMIN_DST — MAX dst, bit 8 (0=A, 1=B). SPRU172C 4-99:
                 * dst = max(A,B); C=0 when the max is A, C=1 otherwise.
                 * ⚠️ Ignoring the destination bit and always writing A means
                 * `max B` never updates B. [2026-09-17] That froze the argmax of
                 * the SCH correlator (0x84e8 `max B`, B stayed 0): the peak landed
                 * at the edge (index 43), the 78 bits were misframed and the SB CRC
                 * failed. */
                int64_t sa = sext40(s->a), sb = sext40(s->b);
                int a_is_max = (sa > sb);     /* [2026-09-21] equal: dst = B, C = 1 (isa_test 128 for MIN) */
                int64_t mx = a_is_max ? sa : sb;
                if ((op >> 8) & 1) s->b = sext40(mx); else s->a = sext40(mx);
                if (a_is_max) s->st0 &= ~ST0_C; else s->st0 |= ST0_C;
                return consumed + s->lk_used;
            }

            /* F487/F587: MIN src (mask FEFF, 1 word) — keep min of A,B.
             * binutils: "min" 1,1,1, 0xF487, 0xFEFF (0xF493 is cmpl). */
            if ((op & 0xFEFF) == 0xF487) {
                /* FIX_MAXMIN_DST — MIN dst, bit 8. SPRU172C 4-100:
                 * dst = min(A,B); C=0 when the min is A, C=1 otherwise. Same
                 * destination bit as MAX above. */
                int64_t sa = sext40(s->a), sb = sext40(s->b);
                int a_is_min = (sa < sb);     /* [2026-09-21] equal: dst = B, C = 1 (isa_test 128) */
                int64_t mn = a_is_min ? sa : sb;
                if ((op >> 8) & 1) s->b = sext40(mn); else s->a = sext40(mn);
                if (a_is_min) s->st0 &= ~ST0_C; else s->st0 |= ST0_C;
                return consumed + s->lk_used;
            }

            /* ⛔ The MAC/MACR/MASR/MACA/MASA/MACAR/MPYA/LD-T/LD-ASM and BITT
             * handlers do NOT belong here. Their opcodes have hi4 = 2 or 3, so
             * inside `case 0xF:` they are unreachable. The live bodies are in
             * c54x_mac_bit_family() (called from `case 0x2:` and `case 0x3:`) and,
             * for BITT, in `case 0x3:` under FIX_BITT_CASE3. Do not move them
             * back: they would be dead code again, and any probe placed with them
             * would go silent without saying so. */

            /* F492/F592: ROLTC src (rotate left through TC, mask FEFF, 1 word).
             * binutils: "roltc" 1,1,1, 0xF492, 0xFEFF.
             * SPRU172C semantics: src bit 31 -> TC, src << 1, src bit 0 <- old TC. */
            if ((op & 0xFEFF) == 0xF492) {
                /* SPRU172C 4-149: 32-bit rotate. (TC) -> src(0); src(30-0) -> src(31-1);
                 * src(31) -> C (the CARRY, TC is only read); 0 -> src(39-32).
                 * [2026-09-20] Was a 40-bit rotate that wrote bit 31 back into TC.
                 * Example ROLTC A, A = 00 0000 5555, TC = 1 -> 00 0000 AAAB, C = 0. */
                int src = (op >> 8) & 1;
                int64_t *acc = src ? &s->b : &s->a;
                int64_t v = *acc & 0xFFFFFFFFLL;
                int new_tc = (int)((v >> 31) & 1);          /* bit 31 -> C */
                int old_tc = (s->st0 & ST0_TC) ? 1 : 0;
                *acc = (int64_t)(((uint32_t)v << 1) | (uint32_t)old_tc);
                if (new_tc) s->st0 |= ST0_C; else s->st0 &= ~ST0_C;
                /* BITT-WATCH, leg 2 of 2 (same gate and window as leg 1 in the
                 * BITT handler). Shows whether the TC set by `bitt` really enters
                 * the accumulator: `after` must equal `before<<1 | old_tc`. An
                 * old_tc always 0 while leg 1 counts TC=1 means TC is overwritten
                 * between the two; both legs seeing TC=0 means the source is empty
                 * and the fault is upstream. */
                if (s->pc >= 0x9ab8 && s->pc <= 0x9ad2) {
                    static int rw = -1;
                    if (rw < 0) rw = calypso_gate("CALYPSO_BITT_WATCH", 0);
                    if (rw) {
                        static unsigned long long n, nz_out, tc_in;
                        n++;
                        if (*acc & 0xFFFFFFFFFFLL) nz_out++;
                        if (old_tc) tc_in++;
                        if (n <= 40 || (n % 5000) == 0)
                            fprintf(stderr, "[c54x] ROLTC-WATCH #%llu pc=0x%04x "
                                    "%c avant=0x%010llx old_tc=%d -> apres=0x%010llx "
                                    "new_tc=%d | cumul: acc_non_nul=%llu/%llu "
                                    "tc_entrant=%llu\n",
                                    n, s->pc, src ? 'B' : 'A',
                                    (unsigned long long)(v & 0xFFFFFFFFFFULL), old_tc,
                                    (unsigned long long)(*acc & 0xFFFFFFFFFFLL),
                                    new_tc, nz_out, n, tc_in);
                    }
                }
                return consumed + s->lk_used;
            }

            /* F49E/F59E: SUBC src (mask FEFF, 1 word) — conditional subtract for division */
            if ((op & 0xFEFF) == 0xF49E) {
                int src = (op >> 8) & 1;
                int64_t *acc = src ? &s->b : &s->a;
                int64_t val = sext40(*acc);
                if (val >= 0) { *acc = sext40((val << 1) + 1); }
                else { *acc = sext40(val << 1); }
                return consumed + s->lk_used;
            }

            /* F48F/F58F: NORM src[, dst] (mask FEFF, 1 word)
             * Per SPRU172C p.4-118: if the two MSBs of src accumulator
             * are different (not sign-extended), shift src left by 1
             * and decrement T. Otherwise do nothing. Used by the FB-det
             * correlator to normalize results; the loop exits when
             * NORM stops shifting (MSBs match = value is normalized). */
            /* FIX_NORM_SD — NORM is `norm src[,dst]`, opcode 0xF48F mask 0xFCFF
             * (tic54x-opc.c): bit 9 = src, bit 8 = dst, hence FOUR encodings
             * F48F/F58F/F68F/F78F. A 0xFEFF mask catches only F48F/F58F and reads
             * F58F as NORM B where it is NORM A,B, while F68F/F78F fall into the
             * F7 "LD #k8" block, which does not exist in this ISA.
             * [2026-09-17] Measured with c54x_exe --arm, step by step: 0x75cf
             * `f78f` NORM B wrote T=0xff8f, BRC became 0xff88 and the FB division
             * (0x75db SUBC) ran 65000 times per frame. PROM0..3 hold 78 x f78f and
             * 157 x f48f. */
            if ((op & 0xFCFF) == 0xF48F) {
                int src, dst;
                c54x_f4_srcdst(op, &src, &dst);
                int64_t val = sext40(src ? s->b : s->a);
                int bit39 = (val >> 39) & 1;
                int bit38 = (val >> 38) & 1;
                /* FIX_NORM_T — a real C54x NORM shifts the accumulator by T (the
                 * exponent produced by EXP) in one cycle and does NOT write T
                 * (SPRU172C). Shifting by 1 bit and decrementing T instead leaves
                 * isolated exp;norm pairs (correlator reference 0x796c/0x796d, SNR
                 * division 0x79b1) never rescaled to full scale: the reference
                 * collapses to {0,-1} and the SNR is tiny. The firmware confirms
                 * it: 0x79ca stores #1 into T then runs `rptb norm A`, a
                 * data-dependent variable shift that only makes sense if NORM
                 * shifts by T. */
                {
                    /* SPRU172C 4-122: the shift is T(5-0), a 6-bit two's complement
                     * count (-16..31). Example NORM B, A with T = 0FF9h shifts by
                     * 39h = -7, not by 4089. */
                    int t = s->t & 0x3F;
                    if (t & 0x20) t -= 64;
                    if (t >= 0) val = sext40(val << t);
                    else        val = sext40(val >> (-t));   /* arithmetic shift (SXM) */
                    if (dst) s->b = val; else s->a = val;
                    /* Do NOT write T: SPRU172C has NORM read T, never set it. */
                }
                if (bit39 != bit38) s->st0 |= ST0_TC;
                else                s->st0 &= ~ST0_TC;
                return consumed + s->lk_used;
            }

            /* F490/F590: ROR src (mask FEFF, 1 word) */
            if ((op & 0xFEFF) == 0xF490) {
                /* SPRU172C 4-150: 32-bit rotate. C -> src(31); src(31-1) -> src(30-0);
                 * src(0) -> C; 0 -> src(39-32). [2026-09-20] Was a 40-bit rotate. */
                int src = (op >> 8) & 1;
                int64_t *acc = src ? &s->b : &s->a;
                uint32_t v = (uint32_t)(*acc & 0xFFFFFFFFULL);
                uint32_t c = c54x_carry_in(s);
                uint32_t lsb = v & 1;
                *acc = (int64_t)((v >> 1) | (c << 31));
                if (lsb) s->st0 |= ST0_C; else s->st0 &= ~ST0_C;
                return consumed + s->lk_used;
            }

            /* F491/F591: ROL src (mask FEFF, 1 word) */
            if ((op & 0xFEFF) == 0xF491) {
                /* SPRU172C 4-148: 32-bit rotate. C -> src(0); src(30-0) -> src(31-1);
                 * src(31) -> C; 0 -> src(39-32). [2026-09-20] Was a 40-bit rotate:
                 * example ROL A with A = 00 B000 1234, C = 0 gives 00 6000 2468, C = 1. */
                int src = (op >> 8) & 1;
                int64_t *acc = src ? &s->b : &s->a;
                uint32_t v = (uint32_t)(*acc & 0xFFFFFFFFULL);
                uint32_t c = c54x_carry_in(s);
                uint32_t msb = (v >> 31) & 1;
                *acc = (int64_t)((v << 1) | c);
                if (msb) s->st0 |= ST0_C; else s->st0 &= ~ST0_C;
                return consumed + s->lk_used;
            }

            /* F488-F48B (mask FCFF, 1 word): MACA T,src[,dst] / MACAR / MASA T / MASAR.
             * SPRU172C 4-86, 4-95: dst = src +/- T x A(32-16), rounded for the R
             * forms. The multiplier is ALWAYS A's high word, whatever src is, and
             * the base is src (not dst). [2026-09-20] The old MACA multiplied T by
             * src's high word and accumulated into dst; MACAR/MASA/MASAR had no
             * handler and fell into the F7 "LD #k8" block, which wrote T. Manual
             * example MACA T, B, B (A=1234 0000, B=2 0000, T=0444, FRCT=1) gives
             * B = 00 009D 4BA0. */
            if ((op & 0xFCFC) == 0xF488) {
                int src, dst; c54x_f4_srcdst(op, &src, &dst);
                int64_t ahi  = (int64_t)(int16_t)((s->a >> 16) & 0xFFFF);
                int64_t prod = (int64_t)(int16_t)s->t * ahi;
                if (s->st1 & ST1_FRCT) prod <<= 1;
                int64_t base = sext40(src ? s->b : s->a);
                int64_t r = (op & 2) ? base - prod : base + prod;   /* F48A/B subtract */
                if (op & 1) r = (r + 0x8000) & ~(int64_t)0xFFFF;    /* F489/B round */
                if (dst) s->b = sext40(r); else s->a = sext40(r);
                return consumed + s->lk_used;
            }

            /* F493/F593: CMPL src (complement, mask FCFF, 1 word). binutils:
             * "cmpl" 1,1,2, 0xF493, 0xFCFF, {OP_SRC,OPT|OP_DST}. The FCFF mask
             * (not FEFF) is what admits the SRC=B variant through bit 9. */
            if ((op & 0xFCFF) == 0xF493) {
                int src = (op >> 8) & 1;
                int64_t *acc = src ? &s->b : &s->a;
                *acc = sext40(~(*acc) & 0xFFFFFFFFFFULL);
                return consumed + s->lk_used;
            }

            /* F49F/F59F: RND src (round, mask FCFF, 1 word). binutils:
             * "rnd" 1,1,2, 0xF49F, 0xFCFF, {OP_SRC,OPT|OP_DST}. */
            if ((op & 0xFCFF) == 0xF49F) {
                /* [2026-09-21] src/dst bits as for NEG/ABS (c54x_f4_srcdst):
                 * `RND A, B` (F59F) rounded B into B. isa_test 170. */
                int src, dst; c54x_f4_srcdst(op, &src, &dst);
                int64_t v = sext40((src ? s->b : s->a) + 0x8000);
                if (dst) s->b = v; else s->a = v;
                return consumed + s->lk_used;
            }

            /* F480/F580: ADD src,ASM,dst (mask FCFF, 1 word) */
            if ((op & 0xFCFF) == 0xF480) {
                int src, dst; c54x_f4_srcdst(op, &src, &dst);   /* FIX_F4XX_SRCDST */
                int64_t sv = sext40(src ? s->b : s->a);
                if (dst) s->b = sext40(s->b + sv); else s->a = sext40(s->a + sv);
                return consumed + s->lk_used;
            }

            /* F481/F581: SUB src,ASM,dst (mask FCFF, 1 word) */
            if ((op & 0xFCFF) == 0xF481) {
                int src, dst; c54x_f4_srcdst(op, &src, &dst);   /* FIX_F4XX_SRCDST */
                int64_t sv = sext40(src ? s->b : s->a);
                if (dst) s->b = sext40(s->b - sv); else s->a = sext40(s->a - sv);
                return consumed + s->lk_used;
            }

            /* F482/F582: LD src,ASM,dst (mask FCFF, 1 word) */
            if ((op & 0xFCFF) == 0xF482) {
                int src, dst; c54x_f4_srcdst(op, &src, &dst);   /* FIX_F4XX_SRCDST */
                int64_t sv = sext40(src ? s->b : s->a);
                if (dst) s->b = sext40(sv); else s->a = sext40(sv);
                return consumed + s->lk_used;
            }

            /* F4xx accumulator shift/load (1-word, mask FCE0):
             * F400: ADD src,shift,dst  F420: SUB  F440: LD  F460: SFTA */
            if ((op & 0xFCE0) == 0xF400) {
                int src, dst; c54x_f4_srcdst(op, &src, &dst);   /* FIX_F4XX_SRCDST */
                int shift = op & 0x1F; if (shift > 15) shift -= 32;
                int64_t sv = sext40(src ? s->b : s->a);
                if (shift >= 0) sv <<= shift; else sv >>= (-shift);
                int64_t dv = sext40(dst ? s->b : s->a);
                c54x_carry_add40(s, dv, sv);                     /* [2026-09-21] isa_test 6 */
                if (dst) s->b = sext40(dv + sv); else s->a = sext40(dv + sv);
                return consumed + s->lk_used;
            }

            if ((op & 0xFCE0) == 0xF420) {
                int src, dst; c54x_f4_srcdst(op, &src, &dst);   /* FIX_F4XX_SRCDST */
                int shift = op & 0x1F; if (shift > 15) shift -= 32;
                int64_t sv = sext40(src ? s->b : s->a);
                if (shift >= 0) sv <<= shift; else sv >>= (-shift);
                int64_t dv = sext40(dst ? s->b : s->a);
                c54x_carry_sub40(s, dv, sv);                     /* [2026-09-21] isa_test 227 */
                if (dst) s->b = sext40(dv - sv); else s->a = sext40(dv - sv);
                return consumed + s->lk_used;
            }

            if ((op & 0xFCE0) == 0xF440) {
                int src, dst; c54x_f4_srcdst(op, &src, &dst);   /* FIX_F4XX_SRCDST */
                int shift = op & 0x1F; if (shift > 15) shift -= 32;
                int64_t sv = sext40(src ? s->b : s->a);
                if (shift >= 0) sv <<= shift; else sv >>= (-shift);
                if (dst) s->b = sext40(sv); else s->a = sext40(sv);
                return consumed + s->lk_used;
            }

            if ((op & 0xFCE0) == 0xF460) {
                /* SFTA body lives in c54x_sfta_exec() (FIX_SFTA_CARRY): two
                 * identical sites call it, so a fix applied to one cannot make
                 * them diverge. */
                c54x_sfta_exec(s, op);
                return consumed + s->lk_used;
            }

            /* ⚠️ SFTL must exclude the RSBX/SSBX encodings. Per binutils
             * tic54x-opc.c, a low-byte high nibble of 0xB (bits 7:4 = 1011) is
             * ALWAYS rsbx/ssbx for every hi8 in {F4,F5,F6,F7} (bit 9 = ST0/ST1,
             * bit 8 = rsbx/ssbx) and NEVER a legal SFTL shift amount. The 0xFCE0
             * mask below leaves bit 4 don't-care and is not keyed on hi8, so
             * without the explicit exclusion it swallows 0xF4Bx/F5Bx/F6Bx/F7Bx
             * (including RSBX INTM=0xF6BB and SSBX INTM=0xF7BB) as a bogus
             * accumulator shift, BEFORE the real hi8==0xF6/0xF7 handlers further
             * down ever run, and INTM is never cleared.
             * The exclusion is unconditional. */
            /* [2026-09-20] 0xF4A0-0xF4A7 is LD #k3, ARP (binutils {"ld", 0xF4A0,
             * 0xFFF8, {OP_k3, OP_ARP}}), NOT an SFTL: SFTL is 0xF0E0/0xFCE0 and is
             * handled with AND/OR/XOR src,shift,dst. Manual example LD 3, ARP -> ARP = 3. */
            if ((op & 0xFFF8) == 0xF4A0) {
                s->st0 = (s->st0 & ~ST0_ARP_MASK) | ((op & 7) << ST0_ARP_SHIFT);
                return consumed + s->lk_used;
            }

        /* F494/F594: SFTC src (mask FEFF, 1 word).
         * Per SPRU172C p.4-264: shift src left by 1 if src(31)==src(30)
         * and src!=0. Used by FB-det normalisation around PC=0x10e5..0x10f4
         * — without it the correlator sums never normalise. */
        if ((op & 0xFEFF) == 0xF494) {
            int src = (op >> 8) & 1;
            int64_t *acc = src ? &s->b : &s->a;
            int64_t val = sext40(*acc);
            if (val != 0) {
                int b31 = (val >> 31) & 1;
                int b30 = (val >> 30) & 1;
                if (b31 == b30) *acc = sext40(val << 1);
            }
            return consumed + s->lk_used;
        }

        if (hi8 == 0xF4) {
            /* F4xx: unconditional branch/call and special instructions.
             * Some F4xx instructions are 1-word (FRET, FRETE, RETE, TRAP, NOP, etc.)
             * Must check specific opcodes BEFORE the 2-word switch. */

            /* NOP — F495 per SPRU172C p.4-121 */
            if (op == 0xF495) {
                return 1; /* 1-word NOP */
            }
            /* TRAP K — F4C0-F4DF per SPRU172C p.4-195:
             * SP-1, PC+1 → TOS, vector(IPTR*128 + K*4) → PC */
            if ((op & 0xFFE0) == 0xF4C0) {
                int k = op & 0x1F;
                s->sp--;
                data_write(s, s->sp, (uint16_t)(s->pc + 1));
                uint16_t iptr = (s->pmst >> PMST_IPTR_SHIFT) & 0x1FF;
                s->pc = (iptr * 0x80) + k * 4;
                C54_LOG("TRAP-FIRED #%d → PC=0x%04x (from PC=0x%04x) [XPC non sauve : corruption vs RETE pop-2 si fire]", k, s->pc,
                        (uint16_t)(s->pc - (iptr * 0x80 + k * 4) + 1 - 1));
                return 0;
            }

            /* F4xx arithmetic instructions (1-word, per tic54x-opc.c).
             * These MUST be checked before the 2-word branch/call switch. */
            {
                /* F483/F583: SAT src (mask FEFF, 1 word) */
                if ((op & 0xFEFF) == 0xF483) {
                    int src = (op >> 8) & 1;
                    int64_t *acc = src ? &s->b : &s->a;
                    int64_t val = sext40(*acc);
                    if (val > 0x7FFFFFFFLL) *acc = sext40(0x7FFFFFFFLL);
                    else if (val < -0x80000000LL) *acc = sext40(-0x80000000LL);
                    return consumed + s->lk_used;
                }
                /* F484/F584: NEG src[,dst] (mask FCFF, 1 word) */
                if ((op & 0xFCFF) == 0xF484) {
                    int src, dst; c54x_f4_srcdst(op, &src, &dst);   /* FIX_F4XX_SRCDST */
                    int64_t val = sext40(src ? s->b : s->a);
                    if (dst) s->b = sext40(-val); else s->a = sext40(-val);
                    return consumed + s->lk_used;
                }
                /* F485/F585: ABS src[,dst] (mask FCFF, 1 word) */
                if ((op & 0xFCFF) == 0xF485) {
                    int src, dst; c54x_f4_srcdst(op, &src, &dst);   /* FIX_F4XX_SRCDST */
                    int64_t val = sext40(src ? s->b : s->a);
                    if (val < 0) val = -val;
                    if (dst) s->b = sext40(val); else s->a = sext40(val);
                    return consumed + s->lk_used;
                }
                /* F48C/F58C: MPYA dst (mask FEFF, 1 word)
                 * Multiply T * A(high), accumulate into dst */
                if ((op & 0xFEFF) == 0xF48C) {
                    int dst = (op >> 8) & 1;
                    int64_t prod = (int64_t)(int16_t)s->t * (int64_t)(int16_t)((s->a >> 16) & 0xFFFF);
                    if (s->st1 & ST1_FRCT) prod <<= 1;
                    if (dst) s->b = sext40(s->b + prod); else s->a = sext40(s->a + prod);
                    return consumed + s->lk_used;
                }
                /* F48D/F58D: SQUR A,dst (mask FEFF, 1 word) */
                if ((op & 0xFEFF) == 0xF48D) {
                    int dst = (op >> 8) & 1;
                    int16_t ah = (int16_t)((s->a >> 16) & 0xFFFF);
                    int64_t prod = (int64_t)ah * (int64_t)ah;
                    if (s->st1 & ST1_FRCT) prod <<= 1;
                    if (dst) s->b = sext40(prod); else s->a = sext40(prod);
                    return consumed + s->lk_used;
                }
                /* F48E/F58E: EXP src (mask FEFF, 1 word)
                 * Count leading sign bits of accumulator, store in T */
                if ((op & 0xFEFF) == 0xF48E) {
                    int src = (op >> 8) & 1;
                    int64_t val = sext40(src ? s->b : s->a);
                    int exp = 0;
                    if (val == 0 || val == -1) { exp = 31; }
                    else {
                        uint64_t uv = (val < 0) ? ~val : val;
                        uv &= 0xFFFFFFFFFFULL;
                        /* Count leading zeros from bit 38 down */
                        for (int i = 38; i >= 0; i--) {
                            if (uv & (1ULL << i)) break;
                            exp++;
                        }
                        exp -= 8; /* EXP = leading sign bits - 8 */
                    }
                    s->t = (uint16_t)(int16_t)exp;
                    return consumed + s->lk_used;
                }
                /* F48F/F58F: NORM — handled below (real implementation, not NOP) */
                /* F492/F592: MAX src (mask FEFF, 1 word) — keep max of A,B */
                if ((op & 0xFEFF) == 0xF492) {
                    int64_t sa = sext40(s->a), sb = sext40(s->b);
                    if (sa < sb) { s->a = s->b; s->st0 |= ST0_C; }
                    else { s->st0 &= ~ST0_C; }
                    return consumed + s->lk_used;
                }
                /* F493/F593: MIN src (mask FEFF, 1 word) — keep min of A,B */
                if ((op & 0xFEFF) == 0xF493) {
                    int64_t sa = sext40(s->a), sb = sext40(s->b);
                    if (sa > sb) { s->a = s->b; s->st0 |= ST0_C; }
                    else { s->st0 &= ~ST0_C; }
                    return consumed + s->lk_used;
                }
                /* F49E/F59E: SUBC src (mask FEFF, 1 word) — conditional subtract for division */
                if ((op & 0xFEFF) == 0xF49E) {
                    int src = (op >> 8) & 1;
                    int64_t *acc = src ? &s->b : &s->a;
                    int64_t val = sext40(*acc);
                    if (val >= 0) { *acc = sext40((val << 1) + 1); }
                    else { *acc = sext40(val << 1); }
                    return consumed + s->lk_used;
                }
                /* F48F/F58F: NORM src[, dst] (mask FEFF, 1 word)
                 * Per SPRU172C p.4-118: if the two MSBs of src accumulator
                 * are different (not sign-extended), shift src left by 1
                 * and decrement T. Otherwise do nothing. Used by the FB-det
                 * correlator to normalize results; the loop exits when
                 * NORM stops shifting (MSBs match = value is normalized). */
              /* Widened NORM mask. binutils: norm 0xF48F/0xFCFF, {OP_SRC, OP_DST}
               * -- bit 9 = source, bit 8 = destination, hence FOUR encodings
               * 0xF48F 0xF58F 0xF68F 0xF78F. A `(op & 0xFEFF)` test frees bit 8
               * only, so 0xF68F and 0xF78F go undecoded.
               * [2026-08-23] PROM0 holds 29 NORM, of which 10 were missed: 0x75cf
               * (the EXP/NORM sequence that builds B just before 0x75d3
               * STLM B,BRC, hence the absurd BRC and the ~50000-iteration SUBC
               * loop), 0x796d and 0x79ad (correlator reference, SNR division), and
               * 0x9a08 0x9a27 0x9a4c in the Viterbi region.
               * ⚠️ Global effect: correlator, division and Viterbi all shift. */
              if ((op & 0xFCFF) == 0xF48F && (op & 0xFEFF) != 0xF48F) {
                  /* the two encodings the narrow mask missed */
                  int nsrc = (op >> 9) & 1, ndst = (op >> 8) & 1;
                  int64_t nv = sext40(nsrc ? s->b : s->a);
                  int b39 = (nv >> 39) & 1, b38 = (nv >> 38) & 1;
                  int16_t t = (int16_t)s->t;
                  nv = (t >= 0) ? sext40(nv << t) : sext40(nv >> (-t));
                  if (ndst) s->b = nv; else s->a = nv;
                  if (b39 != b38) s->st0 |= ST0_TC; else s->st0 &= ~ST0_TC;
                  return consumed + s->lk_used;
              }
              if ((op & 0xFEFF) == 0xF48F) {
                  int src = (op >> 8) & 1;
                  int64_t val = sext40(src ? s->b : s->a);
                    int bit39 = (val >> 39) & 1;
                    int bit38 = (val >> 38) & 1;
                    /* Second copy: NORM shifts by T, not by 1 bit, and does not
                     * write T. */
                    {
                        int16_t t = (int16_t)s->t;
                        if (t >= 0) val = sext40(val << t);
                        else        val = sext40(val >> (-t));
                        if (src) s->b = val; else s->a = val;
                    }
                    if (bit39 != bit38) s->st0 |= ST0_TC;
                    else                s->st0 &= ~ST0_TC;
                    return consumed + s->lk_used;
                }
                /* F49F: DELAY (pipeline flush, NOP) */
                if (op == 0xF49F) { return consumed + s->lk_used; }
                /* F490/F590: ROR src (mask FEFF, 1 word) */
                if ((op & 0xFEFF) == 0xF490) {
                    int src = (op >> 8) & 1;
                    int64_t *acc = src ? &s->b : &s->a;
                    uint16_t c = c54x_carry_in(s); /* carry */
                    uint16_t lsb = *acc & 1;
                    *acc = sext40(((uint64_t)(*acc & 0xFFFFFFFFFFULL) >> 1) | ((uint64_t)c << 39));
                    if (lsb) s->st0 |= ST0_C; else s->st0 &= ~ST0_C;
                    return consumed + s->lk_used;
                }
                /* F491/F591: ROL src (mask FEFF, 1 word) */
                if ((op & 0xFEFF) == 0xF491) {
                    int src = (op >> 8) & 1;
                    int64_t *acc = src ? &s->b : &s->a;
                    uint16_t c = c54x_carry_in(s);
                    uint16_t msb = (*acc >> 39) & 1;
                    *acc = sext40(((*acc << 1) & 0xFFFFFFFFFFULL) | c);
                    if (msb) s->st0 |= ST0_C; else s->st0 &= ~ST0_C;
                    return consumed + s->lk_used;
                }
                /* F488/F588: MACA T,src[,dst] (mask FCFF, 1 word) */
                if ((op & 0xFCFF) == 0xF488) {
                    int src, dst; c54x_f4_srcdst(op, &src, &dst);   /* FIX_F4XX_SRCDST */
                    int64_t prod = (int64_t)(int16_t)s->t * (int64_t)(int16_t)((src ? s->b : s->a) >> 16);
                    if (s->st1 & ST1_FRCT) prod <<= 1;
                    if (dst) s->b = sext40(s->b + prod); else s->a = sext40(s->a + prod);
                    return consumed + s->lk_used;
                }
                /* F486/F586: CMPL src (complement, mask FEFF, 1 word) */
                if ((op & 0xFEFF) == 0xF486) {
                    int src = (op >> 8) & 1;
                    int64_t *acc = src ? &s->b : &s->a;
                    *acc = sext40(~(*acc) & 0xFFFFFFFFFFULL);
                    return consumed + s->lk_used;
                }
                /* F487/F587: RND src (round, mask FEFF, 1 word) */
                if ((op & 0xFEFF) == 0xF487) {
                    int src = (op >> 8) & 1;
                    int64_t *acc = src ? &s->b : &s->a;
                    *acc = sext40(*acc + 0x8000);
                    return consumed + s->lk_used;
                }
                /* F480/F580: ADD src,ASM,dst (mask FCFF, 1 word) */
                if ((op & 0xFCFF) == 0xF480) {
                    int src, dst; c54x_f4_srcdst(op, &src, &dst);   /* FIX_F4XX_SRCDST */
                    int64_t sv = sext40(src ? s->b : s->a);
                    if (dst) s->b = sext40(s->b + sv); else s->a = sext40(s->a + sv);
                    return consumed + s->lk_used;
                }
                /* F481/F581: SUB src,ASM,dst (mask FCFF, 1 word) */
                if ((op & 0xFCFF) == 0xF481) {
                    int src, dst; c54x_f4_srcdst(op, &src, &dst);   /* FIX_F4XX_SRCDST */
                    int64_t sv = sext40(src ? s->b : s->a);
                    if (dst) s->b = sext40(s->b - sv); else s->a = sext40(s->a - sv);
                    return consumed + s->lk_used;
                }
                /* F482/F582: LD src,ASM,dst (mask FCFF, 1 word) */
                if ((op & 0xFCFF) == 0xF482) {
                    int src, dst; c54x_f4_srcdst(op, &src, &dst);   /* FIX_F4XX_SRCDST */
                    int64_t sv = sext40(src ? s->b : s->a);
                    if (dst) s->b = sext40(sv); else s->a = sext40(sv);
                    return consumed + s->lk_used;
                }
                /* F4xx accumulator shift/load (1-word, mask FCE0):
                 * F400: ADD src,shift,dst  F420: SUB  F440: LD  F460: SFTA */
                if ((op & 0xFCE0) == 0xF400) {
                    int src, dst; c54x_f4_srcdst(op, &src, &dst);   /* FIX_F4XX_SRCDST */
                    int shift = op & 0x1F; if (shift > 15) shift -= 32;
                    int64_t sv = sext40(src ? s->b : s->a);
                    if (shift >= 0) sv <<= shift; else sv >>= (-shift);
                    if (dst) s->b = sext40(s->b + sv); else s->a = sext40(s->a + sv);
                    return consumed + s->lk_used;
                }
                if ((op & 0xFCE0) == 0xF420) {
                    int src, dst; c54x_f4_srcdst(op, &src, &dst);   /* FIX_F4XX_SRCDST */
                    int shift = op & 0x1F; if (shift > 15) shift -= 32;
                    int64_t sv = sext40(src ? s->b : s->a);
                    if (shift >= 0) sv <<= shift; else sv >>= (-shift);
                    if (dst) s->b = sext40(s->b - sv); else s->a = sext40(s->a - sv);
                    return consumed + s->lk_used;
                }
                if ((op & 0xFCE0) == 0xF440) {
                    int src, dst; c54x_f4_srcdst(op, &src, &dst);   /* FIX_F4XX_SRCDST */
                    int shift = op & 0x1F; if (shift > 15) shift -= 32;
                    int64_t sv = sext40(src ? s->b : s->a);
                    if (shift >= 0) sv <<= shift; else sv >>= (-shift);
                    if (dst) s->b = sext40(sv); else s->a = sext40(sv);
                    return consumed + s->lk_used;
                }
                if ((op & 0xFCE0) == 0xF460) {
                    /* SFTA body lives in c54x_sfta_exec() (FIX_SFTA_CARRY). */
                    c54x_sfta_exec(s, op);
                    return consumed + s->lk_used;
                }
                /* ⚠️ Second copy of the SFTL guard, inside the hi8==0xF4 block.
                 * Here there is NO RSBX/SSBX exclusion, so 0xF4Bx reaches this
                 * SFTL path and returns before the RSBX handler below. See the
                 * unconditional guard higher up for the encoding rule (low-byte
                 * high nibble 0xB is always rsbx/ssbx per tic54x-opc.c, never a
                 * legal shift amount). */
                if ((op & 0xFFF8) == 0xF4A0) {      /* LD #k3, ARP (see the first copy) */
                    s->st0 = (s->st0 & ~ST0_ARP_MASK) | ((op & 7) << ST0_ARP_SHIFT);
                    return consumed + s->lk_used;
                }
            }
                        /* F4Bx: RSBX -- reset bit in ST0 (bit 9=0, bit 8=0).
             * Per tic54x-opc.c: RSBX 0xF4B0 mask 0xFDF0. */
            if ((op & 0xFFF0) == 0xF4B0) {
                int bit = op & 0x0F;
                s->st0 &= ~(1 << bit);
                return consumed + s->lk_used;
            }
            /* F494/F594: SFTC src (mask FEFF, 1 word).
             * Per SPRU172C p.4-264: shift src left by 1 if src(31)==src(30)
             * and src!=0. Used by FB-det normalisation around PC=0x10e5..0x10f4
             * — without it the correlator sums never normalise. */
            if ((op & 0xFEFF) == 0xF494) {
                int src = (op >> 8) & 1;
                int64_t *acc = src ? &s->b : &s->a;
                int64_t val = sext40(*acc);
                if (val != 0) {
                    int b31 = (val >> 31) & 1;
                    int b30 = (val >> 30) & 1;
                    if (b31 == b30) *acc = sext40(val << 1);
                }
                return consumed + s->lk_used;
            }
            /* Remaining F4xx: unhandled — treat as 1-word NOP */
            C54_LOG("F4xx unhandled: 0x%04x PC=0x%04x", op, s->pc);
            return consumed + s->lk_used;
        }
        if (hi8 == 0xF0 || hi8 == 0xF1) {
            /* ═══════════════════════════════════════════════════════════════
             * FIX_F1XX_ALU_LK — ADD/SUB/LD/AND/OR/XOR #lk with DST=B.
             *
             * The long-immediate ALU handlers exist, but they are nested inside
             * `if (hi8 == 0xF2)` and `if (hi8 == 0xF3)`, so an `0xF1xx` opcode can
             * never reach them, and this block holds no FCF0/FCE0/FCFF mask test,
             * only exact matches (F072/F073/F074...). The family with DST=B (bit 8)
             * was therefore not decoded at all.
             *
             * [2026-08-03] Measured with the CHAIN-B05F probe on the task
             * dispatcher:
             *     0xb060  LD          A = 0x000018   (correct)
             *     0xb062  f130 7fff   A = 0x005294   <- A clobbered
             *     0xb066  f843     -> 0xb077          bailout, 33 of 33 passes
             * `0xF130` is `AND #0x7fff, A, B` (subop=3, src_b=0 -> A, dst_b=1 -> B):
             * it must write B and leave A untouched. With the fix, 0xa5cd (RX arming)
             * runs for the first time, the dispatch chain gets past 0xb066 to 0xb070
             * and then the CALA; no observable was lost (SI 8 vs 7, camp 46 vs
             * 30/42, CHAN_REQ 15 vs 10/14).
             *
             * ⚠️ Still unexplained: where `0x5294` comes from. It is constant and
             * independent of the input, and an undecoded opcode should leave A alone
             * rather than write a constant into it. This fix makes the decoding
             * correct; it does not prove it was the only cause. If A is still
             * clobbered with the fix on, the fallback itself is at fault.
             *
             * ⚠️ Scope: this touches EVERY 0xF1xx instruction of the firmware, not
             * just the task dispatcher. The only bench that reaches SMS runs with
             * CALYPSO_DSP_RUN_C54X=0, so this path is never exercised there.
             *
             * The implementation is deliberately a conformant copy of the
             * `hi8 == 0xF3` branch (mask FCF0): same arithmetic, same shift and
             * sign handling, same src/dst selection. Any divergence between the two
             * would be one more bug, not an improvement. */
            if ((op & 0xFCF0) == 0xF000 ||  /* ADD #lk, SHIFT, src, [dst] */
                (op & 0xFCF0) == 0xF010 ||  /* SUB */
                (op & 0xFCF0) == 0xF020 ||  /* LD  */
                (op & 0xFCF0) == 0xF030 ||  /* AND */
                (op & 0xFCF0) == 0xF040 ||  /* OR  */
                (op & 0xFCF0) == 0xF050) {  /* XOR */
                op2 = prog_fetch(s, s->pc + 1 + (s->lk_used ? 1 : 0));
                consumed = 2;
                int subop     = (op >> 4) & 0xF;
                int shift_raw = op & 0xF;
                /* [2026-09-20] SHFT is a 4-bit UNSIGNED field (0..15) for all six
                 * forms, ADD included: SPRU172C 4-4 "ADD #lk [, SHFT], src [, dst]",
                 * and the manual's own example ADD #4568h, 8, A, B gives
                 * B = A + (lk << 8). The signed reading turned SHFT=8..15 into a
                 * right shift. The ISA suite (tools/isa_test) caught it. */
                int shift     = shift_raw;
                /* ─────────────────────────────────────────────────────────────
                 * FIX_LK_SHFT — the shift field of these #lk forms is FOUR bits
                 * (`op & 0xF`) and must not be given the sign rule of a five-bit
                 * field: -16..15 does not fit in four bits. At 0x7d19
                 * (`f02f 0001` = LD #1, SHFT=15, A) the signed reading turns the
                 * shift into -1, lk_val becomes 1 >> 1 = 0 and the accumulator
                 * stays zero.
                 *
                 * Authority: tic54x-opc.c gives these forms OP_SHFT (unsigned),
                 * not OP_SHIFT:
                 *   { "ld",  0xF020, 0xFEF0, {OP_lk, OPT|OP_SHFT,  OP_DST} }
                 *   { "sub"/"and"/"or"/"xor" ... OPT|OP_SHFT ... }
                 *   { "add", 0xF000, 0xFCF0, {OP_lk, OPT|OP_SHIFT, ...} }  <- only
                 * hence the subop 1..5 range: ADD (subop 0) keeps the signed rule.
                 *
                 * Physical argument: the sequence is the C54x reciprocal idiom
                 *     0x7d19  ld   #0x0001, A
                 *     0x7d1b  sfta A
                 *     0x7d1c  rpt  #15
                 *     0x7d1d  subc @0x0b, A     ; 16-step division
                 * SUBC needs a left-aligned dividend or the quotient is zero by
                 * construction, so loading 1 in order to shift it RIGHT would make
                 * the whole block dead. Shifted left by 15: 0x8000 / 0x0481 = 28,
                 * a Q15 reciprocal, which is what the firmware expects.
                 *
                 * [2026-09-19] Validated on the deterministic replay fed with real
                 * SCH bursts (ptrkrysik capture, BSIC 32):
                 *   without: 0x7d19 `LD #1,SHFT=15,A` yields A = 0 -> zero dividend
                 *            -> zero quotient -> zero T -> empty coefficients
                 *   with:    A = 0x0010000000, quotient 13737 then 11987, and the
                 *            0x01dbf46a attractor -- reproduced identically by the
                 *            synthetic fixture and by the real capture -- is gone. */
                {
                    if (subop >= 1 && subop <= 5 && (shift_raw & 0x8)) {
                        /* Coverage counter: an A/B comparison is only meaningful if
                         * the fix actually fires on the bench under test. Count the
                         * cases where it changes the result (shift_raw >= 8, the only
                         * range where signed and unsigned diverge) and report
                         * periodically. */
                        static unsigned long chg = 0;
                        if (++chg == 1 || (chg % 20000) == 0)
                            fprintf(stderr, "[c54x] FIX_LK_SHFT ACTIF #%lu "
                                    "pc=0x%04x op=0x%04x subop=%d shift %d -> %d insn=%u\n",
                                    chg, s->pc, op, subop,
                                    shift_raw - 16, shift_raw, s->insn_count);
                        shift = shift_raw;          /* OP_SHFT: unsigned, 0..15 */
                    }
                }
                int src_b     = (op >> 9) & 1;
                int dst_b     = (op >> 8) & 1;
                int64_t src   = src_b ? s->b : s->a;
                /* ADD/SUB/LD take a signed lk; AND/OR/XOR an unsigned one. */
                int64_t lk_base = (subop <= 2) ? (int64_t)(int16_t)op2
                                               : (int64_t)(uint16_t)op2;
                int64_t lk_val  = (shift >= 0) ? (lk_base << shift)
                                               : (lk_base >> (-shift));
                int64_t result = src;
                switch (subop) {
                case 0x0: result = src + lk_val; break;   /* ADD */
                case 0x1: result = src - lk_val; break;   /* SUB */
                case 0x2: result = lk_val;       break;   /* LD (src ignored) */
                case 0x3: result = src & lk_val; break;   /* AND */
                case 0x4: result = src | lk_val; break;   /* OR  */
                case 0x5: result = src ^ lk_val; break;   /* XOR */
                }
                if (dst_b) s->b = sext40(result); else s->a = sext40(result);
                {   /* Bounded trace, 40 lines: an ISA fix has to be observable
                     * rather than assumed. */
                    static unsigned _n = 0;
                    if (_n < 40) {
                        _n++;
                        fprintf(stderr,
                                "[c54x] FIX_F1XX_ALU_LK #%u pc=0x%04x op=0x%04x lk=0x%04x "
                                "subop=%d shift=%d src=%s dst=%s -> %s=0x%06llx insn=%u\n",
                                _n, s->pc, op, op2, subop, shift,
                                src_b ? "B" : "A", dst_b ? "B" : "A",
                                dst_b ? "B" : "A",
                                (unsigned long long)((dst_b ? s->b : s->a) & 0xFFFFFFULL),
                                s->insn_count);
                    }
                }
                return consumed + s->lk_used;
            }
            /* ⚠️ Do not add a catch-all `if (hi8 == 0xF1) { FIRS }` here: per
             * binutils tic54x-opc.c the real FIRS is 0xE000 mask 0xFF00, handled
             * in the 0xE0 case, never 0xF1xx. A catch-all doing
             * `s->a = sext40((int64_t)sum << 16)` zeroes A_low unconditionally, and
             * STL A,*AR2- at PC=0x9ac0 then writes 0 into MMR_IMR, clearing the
             * interrupt mask and wedging the DSP. 0xF1xx belongs to SFTL/AND/OR/XOR
             * (mask FCE0, base 0xF0E0), e.g. 0x9abd `f1fe` = SFTL A,-2,B. */
            /* F073: B pmad — unconditional branch (2-word).
             * Per tic54x-opc.c: 0xF073 mask 0xFFFF. */
            if (op == 0xF073) {
                op2 = prog_fetch(s, s->pc + 1);
                s->pc = op2;
                return 0;
            }
            /* F074: CALL pmad — unconditional call (2-word).
             * Per tic54x-opc.c: call 0xF074 mask 0xFFFF.
             * Push PC+2 (return address), branch to pmad.
             * NOTE: RETE is 0xF4EB (already handled above), NOT F074. */
            if (op == 0xF074) {
                op2 = prog_fetch(s, s->pc + 1);
                s->sp--;
                data_write(s, s->sp, (uint16_t)(s->pc + 2));
                s->pc = op2;
                return 0;
            }







            /* F072: RPTB pmad — block repeat (2-word, non-delayed).
             * Per tic54x-opc.c: 0xF072 mask 0xFFFF.
             * RSA = PC+2, REA = pmad. */
            if (op == 0xF072) {
                op2 = prog_fetch(s, s->pc + 1);
                consumed = 2;
                s->rea = op2;
                s->rsa = (uint16_t)(s->pc + 2);
                s->rptb_active = true;
                s->st1 |= ST1_BRAF;
                return consumed + s->lk_used;
            }
            /* F07x: RPT/RPTZ/misc (F072-F074 handled above) */
            if (op == 0xF070) {
                /* F070: RPT #lku — repeat next instruction lku+1 times (2-word) */
                op2 = prog_fetch(s, s->pc + 1);
                consumed = 2;
                s->rpt_count = op2;
                s->rpt_active = true; s->rpt_fresh = true;
                s->pc += 2;
                return 0;
            }
            /* RPTZ needs the binutils mask 0xFEFF (bit 8 = accumulator), not an
             * exact equality:
             *     { "rptz", 2, ..., 0xF071, 0xFEFF, {OP_DST, OP_lku} }
             * 0xF071 = RPTZ A, 0xF171 = RPTZ B. [2026-08-23] With an exact test the
             * B variant fell through to `unimpl:` at 8 sites in PROM0, with three
             * consequences at once: B was not zeroed, RC was not armed (the loop ran
             * ONCE instead of lk+1 times), and 1 word was consumed instead of 2, so
             * the #lk word executed as an instruction and desynchronised the stream. */
            if ((op & 0xFEFF) == 0xF071 && op != 0xF071) {
                op2 = prog_fetch(s, s->pc + 1);
                consumed = 2;
                if ((op >> 8) & 1) s->b = 0; else s->a = 0;
                s->rpt_count = op2;
                s->rpt_active = true; s->rpt_fresh = true;
                s->pc += 2;
                return 0;
            }
            if (op == 0xF071) {
                /* F071: RPTZ dst, #lku — zero accumulator and repeat (2-word) */
                op2 = prog_fetch(s, s->pc + 1);
                consumed = 2;
                int dst = (op >> 8) & 1; /* bit 8 via FEFF mask */
                if (dst) s->b = 0; else s->a = 0;
                s->rpt_count = op2;
                s->rpt_active = true; s->rpt_fresh = true;
                s->pc += 2;
                return 0;
            }
            if ((op & 0xFFF0) == 0xF070) {
                /* F075-F07F: undefined, treat as 1-word NOP */
                return consumed + s->lk_used;
            }
            /* F0Bx/F1Bx: RSBX/SSBX */
            if ((op & 0x00F0) == 0x00B0) {
                int bit = op & 0x0F;
                int set = (op >> 8) & 1;
                int st = (op >> 5) & 1;
                if (st == 0) { if (set) s->st0 |= (1<<bit); else s->st0 &= ~(1<<bit); }
                else         { if (set) s->st1 |= (1<<bit); else s->st1 &= ~(1<<bit); }
                return consumed + s->lk_used;
            }
            /* F0xx/F1xx ALU with #lk immediate (2-word).
             * Per tic54x-opc.c: bits 7:4 = op (0=ADD,1=SUB,2=LD,3=AND,4=OR,5=XOR),
             * bit 8 = SRC (ADD/SUB/AND/OR/XOR) or DST (LD), bit 9 = DST,
             * bits 3:0 = shift. Second word = lk. */
            {
                uint8_t alu_op = (op >> 4) & 0xF;
                if (alu_op <= 5) {
                    op2 = prog_fetch(s, s->pc + 1);
                    consumed = 2;
                    int shift = op & 0xF;
                    int src_sel = (op >> 8) & 1;
                    int dst_sel = (op >> 9) & 1;
                    int64_t src_val = src_sel ? s->b : s->a;
                    int64_t *dst = (alu_op == 2)
                        ? (src_sel ? &s->b : &s->a)
                        : (dst_sel ? &s->b : &s->a);
                    int64_t lk_val;
                    if (alu_op <= 2)
                        lk_val = (int64_t)(int16_t)op2 << shift;
                    else
                        lk_val = (int64_t)(uint16_t)op2 << shift;
                    switch (alu_op) {
                    case 0: *dst = sext40(src_val + lk_val); break; /* ADD */
                    case 1: *dst = sext40(src_val - lk_val); break; /* SUB */
                    case 2: *dst = sext40(lk_val); break;           /* LD  */
                    case 3: *dst = src_val & lk_val; break;         /* AND */
                    case 4: *dst = src_val | lk_val; break;         /* OR  */
                    case 5: *dst = src_val ^ lk_val; break;         /* XOR */
                    }
                    return consumed + s->lk_used;
                }
                if (alu_op == 6) {
                    /* F06x: ADD/SUB/LD/AND/OR/XOR #lk,16 + MPY/MAC #lk */
                    uint8_t sub6 = op & 0xF;
                    op2 = prog_fetch(s, s->pc + 1);
                    consumed = 2;
                    int src_sel = (op >> 8) & 1;
                    int dst_sel = (op >> 9) & 1;
                    int64_t src_val = src_sel ? s->b : s->a;
                    int64_t *dst = dst_sel ? &s->b : &s->a;
                    switch (sub6) {
                    case 0: *dst = sext40(src_val + ((int64_t)(int16_t)op2 << 16)); break;
                    case 1: *dst = sext40(src_val - ((int64_t)(int16_t)op2 << 16)); break;
                    case 2: dst = src_sel ? &s->b : &s->a;
                            *dst = sext40((int64_t)(int16_t)op2 << 16); break;
                    case 3: *dst = src_val & ((int64_t)(uint16_t)op2 << 16); break;
                    case 4: *dst = src_val | ((int64_t)(uint16_t)op2 << 16); break;
                    case 5: *dst = src_val ^ ((int64_t)(uint16_t)op2 << 16); break;
                    case 6: /* MPY #lk, dst */
                            dst = src_sel ? &s->b : &s->a;
                            { int64_t p = (int64_t)(int16_t)s->t * (int64_t)(int16_t)op2;
                              if (s->st1 & ST1_FRCT) p <<= 1;
                              *dst = sext40(p); } break;
                    case 7: /* MAC #lk, src[,dst] */
                            { int64_t p = (int64_t)(int16_t)s->t * (int64_t)(int16_t)op2;
                              if (s->st1 & ST1_FRCT) p <<= 1;
                              *dst = sext40(src_val + p); } break;
                    default: break;
                    }
                    return consumed + s->lk_used;
                }
                if (alu_op >= 8) {
                    /* F08x-F0Fx: accumulator-to-accumulator ops (1-word).
                     * bits 7:5 = op (100=AND,101=OR,110=XOR,111=SFTL)
                     * bits 4:0 = shift (signed 5-bit), bit 9 = SRC, bit 8 = DST
                     * (binutils tic54x convention).
                     * ⚠️ Swapping the two makes 0xf1fe (SFTL A,-2,B) compute
                     * A = B>>2 instead of B = A>>2; with B=0 that zeroes A, and
                     * STL A,*AR2- then writes 0 into the IMR. Measured with the
                     * A-AT-PC tracer: 8454 fires with A_low=0 and
                     * last_writer=0xf1fe at PC=0x9abd. */
                    int src_sel = (op >> 9) & 1;   /* bit 9 = SRC */
                    int dst_sel = (op >> 8) & 1;   /* bit 8 = DST */
                    int64_t sv = src_sel ? s->b : s->a;
                    int64_t *dst = dst_sel ? &s->b : &s->a;
                    int shift = op & 0x1F;
                    if (shift > 15) shift -= 32;
                    uint8_t aop = (op >> 5) & 0x7;
                    int64_t shifted;
                    if (shift >= 0) shifted = sv << shift;
                    else            shifted = sv >> (-shift);
                    /* First operand of AND/OR/XOR is the DESTINATION
                     * (TI SPRU172C); see the block comment below. */
                    int64_t first = *dst;
                    switch (aop) {
                    /* ═══════════════════════════════════════════════════════
                     * AND/OR/XOR READ THE DESTINATION.
                     *
                     * `*dst = sv OP shifted` computes `dst = src OP (src<<SHIFT)`
                     * and throws the destination away. That is right only when
                     * D == S, wrong as soon as D != S.
                     *
                     * TI SPRU172C (Mnemonic Instruction Set, March 2001):
                     *     AND src [,SHIFT] [,dst]   dst = dst & src << SHIFT
                     *     OR  src [,SHIFT] [,dst]   dst = dst | src << SHIFT
                     *     XOR src [,SHIFT] [,dst]   dst = dst ^ src << SHIFT
                     *     Execution: (src or [dst]) OP (src) << SHIFT -> dst
                     * Form 4 encoding (1 word), verified bit by bit:
                     *     15..10 = 111100   bit9 = S   bit8 = D
                     *     7..6 = 10  bit5 = 1  4..0 = SHIFT
                     *
                     * [2026-08-04] Measured case: `0xf1a5` = 111100 0 1 10 1 00101
                     * -> src=A, dst=B, SHIFT=5, i.e. `or A, 5, B`. At 0x9719 B held
                     * 0x8000 (1<<B_BLUD, "data block present", set at 0x96dd); the
                     * old computation destroyed it, `stl *AR3,B` wrote 0x0000 into
                     * a_cd[0] and the ARM never saw a block. The real scope is every
                     * read-modify-write in the firmware, far beyond a_cd.
                     *
                     * ⚠️ `case 7` (SFTL) stays as it is: SPRU172C gives
                     *     SFTL src, SHIFT [,dst]  ->  dst = src << SHIFT
                     * SFTL is a shift and does NOT read the destination.
                     *
                     * ⚠️ Distinct from FIX_F1XX_ALU_LK, which covers only mask
                     * 0xFCF0 subops 0..5, the 2-word long-immediate forms. Here the
                     * subop is 0xA, a 1-word form.
                     * ═══════════════════════════════════════════════════════ */
                    case 4: *dst = sext40(first) & sext40(shifted); break;
                    case 5: *dst = sext40(first) | sext40(shifted); break;
                    case 6: *dst = sext40(first) ^ sext40(shifted); break;
                    case 7: c54x_sftl_exec(s, op); break;   /* SFTL: 32-bit, sets C */
                    default: break;
                    }
                    return consumed + s->lk_used;
                }
            }
            goto unimpl;
        }
        /* F272/F274/F273: RPTBD/CALLD/RETD — must check BEFORE LMS */
        if (op == 0xF272) {
            /* RPTBD pmad — delayed block repeat (2 words).
             * Delayed: 2 delay slots after the 2-word instruction.
             * RSA = PC + 4 (skip RPTBD + 2 delay slot words). */
            op2 = prog_fetch(s, s->pc + 1);
            consumed = 2;
            s->rea = op2;
            s->rsa = (uint16_t)(s->pc + 4);
            s->rptb_active = true;
            s->st1 |= ST1_BRAF;
            { static int _rb=0; if (_rb<20) { C54_LOG("RPTBD PC=0x%04x REA=0x%04x RSA=0x%04x BRC=%d", s->pc, s->rea, s->rsa, s->brc); _rb++; } }
            return consumed + s->lk_used;
        }
        if (op == 0xF274) {
            /* CALLD pmad — delayed call (2 words, 2 delay slots).
             * Pushes PC+4 (return past CALLD and its 2 delay slots), then arms
             * delayed_pc/delay_slots so the 2 slots execute BEFORE the branch.
             * ⚠️ Branching immediately (s->pc = op2; return 0) skips the delay
             * slots; when a slot holds a push or a pop, the stack drifts by one
             * word, POPM ST0 picks up an orphan PC, DP turns to garbage and the
             * CALA lands in the 0x70c3 black hole. */
            op2 = prog_fetch(s, s->pc + 1);
            consumed = 2;
            s->sp--;
            data_write(s, s->sp, (uint16_t)(s->pc + 4));
            s->delayed_pc  = op2;
            s->delay_slots = 2;
            return consumed + s->lk_used;
        }
        if (op == 0xF273) {
            /* BD pmad — delayed branch (2 words, 2 delay slots), NO stack access.
             * Per tic54x-opc.c: bd 0xF273 mask 0xFFFF. The real RETD is 0xFE00
             * (hi8==0xFE, handled further down with its pop and delay slots).
             * ⚠️ Two ways to get this wrong, both seen: treating it as RETD adds a
             * spurious pop, and branching immediately skips the 2 delay slots.
             * Either drifts SP by one word, POPM ST0 then reads an orphan PC and
             * the dispatcher CALAD ends in the 0x70c3 black hole. Like B (F073):
             * a branch, no stack, but with delay_slots = 2. */
            op2 = prog_fetch(s, s->pc + 1);
            consumed = 2;
            s->delayed_pc  = op2;
            s->delay_slots = 2;
            return consumed + s->lk_used;
        }
        /* F2xx dispatch =====================================================
         *
         * Per binutils tic54x-opc.c the ALU masks FCF0/FCFF/FCE0 span
         * F0xx/F1xx/F2xx/F3xx with bit 9 = SRC and bit 8 = DST. F2xx was the one
         * gap: F0/F1 have a legacy handler (reversed convention, bit 8 = src, kept
         * for compatibility because the firmware is aligned on it), F3 has its own
         * dispatch, and F2 fell through to `unimpl`, which turned 0xf210 into a
         * tight loop at PC=0xfbd9. That opcode is `SUB #8,B,A` (op2=0x0008),
         * followed by BC fbe2, ALEQ: the pre-correlator wait loop. Confirmed on
         * three silicon ROM dumps (3416, 3606 and the local one).
         *
         * Coverage:
         *   - F260-F267 mask FCFF: ALU #lk,16 + MAC
         *   - F200/F210/F220/F230/F240/F250 mask FCF0: ADD/SUB/LD/AND/OR/XOR
         *     #lk,shift
         *   - F280-F2FF mask FCE0: 1-word AND/OR/XOR/SFTL src,shift,dst
         *
         * F272/F273/F274 (exact-match RPTBD/BD/CALLD) are handled above and stay
         * out of this dispatch. */
        if (hi8 == 0xF2) {
            /* F260-F267 : 2-word ALU #lk,16 + MAC #lk (mask FCFF) */
            if ((op & 0xFCFF) == 0xF060 ||  /* ADD */
                (op & 0xFCFF) == 0xF061 ||  /* SUB */
                (op & 0xFCFF) == 0xF062 ||  /* LD  */
                (op & 0xFCFF) == 0xF063 ||  /* AND */
                (op & 0xFCFF) == 0xF064 ||  /* OR  */
                (op & 0xFCFF) == 0xF065 ||  /* XOR */
                (op & 0xFCFF) == 0xF067) {  /* MAC */
                op2 = prog_fetch(s, s->pc + 1 + (s->lk_used ? 1 : 0));
                consumed = 2;
                int sub = op & 0x7;
                int src_b = (op >> 9) & 1;
                int dst_b = (op >> 8) & 1;
                int64_t src = src_b ? s->b : s->a;
                int64_t result = src;
                switch (sub) {
                case 0x0: result = src + ((int64_t)(int16_t)op2 << 16); break;
                case 0x1: result = src - ((int64_t)(int16_t)op2 << 16); break;
                case 0x2: result = ((int64_t)(int16_t)op2 << 16); break;
                case 0x3: result = src & (((int64_t)op2) << 16); break;
                case 0x4: result = src | (((int64_t)op2) << 16); break;
                case 0x5: result = src ^ (((int64_t)op2) << 16); break;
                case 0x7: {
                    int64_t prod = (int64_t)(int16_t)s->t * (int64_t)(int16_t)op2;
                    if (s->st1 & ST1_FRCT) prod <<= 1;
                    result = src + prod; break;
                }
                }
                if (dst_b) s->b = sext40(result); else s->a = sext40(result);
                return consumed + s->lk_used;
            }
            /* F200/F210/F220/F230/F240/F250 : 2-word ALU #lk,shift (mask FCF0) */
            if ((op & 0xFCF0) == 0xF000 ||  /* ADD */
                (op & 0xFCF0) == 0xF010 ||  /* SUB  <- 0xF210 matches here */
                (op & 0xFCF0) == 0xF020 ||  /* LD   */
                (op & 0xFCF0) == 0xF030 ||  /* AND  */
                (op & 0xFCF0) == 0xF040 ||  /* OR   */
                (op & 0xFCF0) == 0xF050) {  /* XOR  */
                op2 = prog_fetch(s, s->pc + 1 + (s->lk_used ? 1 : 0));
                consumed = 2;
                int subop = (op >> 4) & 0xF;
                int shift_raw = op & 0xF;
                int shift = shift_raw;          /* SHFT: 4-bit unsigned (SPRU172C), see the F0/F1 copy */
                int src_b = (op >> 9) & 1;
                int dst_b = (op >> 8) & 1;
                int64_t src = src_b ? s->b : s->a;
                int64_t lk_signed = (subop <= 2) ? (int64_t)(int16_t)op2
                                                 : (int64_t)(uint16_t)op2;
                int64_t lk_val = (shift >= 0) ? (lk_signed << shift)
                                              : (lk_signed >> (-shift));
                int64_t result = src;
                switch (subop) {
                case 0x0: result = src + lk_val; break;
                case 0x1: result = src - lk_val; break;
                case 0x2: result = lk_val; break;
                case 0x3: result = src & lk_val; break;
                case 0x4: result = src | lk_val; break;
                case 0x5: result = src ^ lk_val; break;
                }
                if (dst_b) s->b = sext40(result); else s->a = sext40(result);
                return consumed + s->lk_used;
            }
            /* F280-F2FF : 1-word shift class (AND/OR/XOR/SFTL src,shift,dst) FCE0 */
            if ((op & 0xFCE0) == 0xF080 ||  /* AND */
                (op & 0xFCE0) == 0xF0A0 ||  /* OR  */
                (op & 0xFCE0) == 0xF0C0 ||  /* XOR */
                (op & 0xFCE0) == 0xF0E0) {  /* SFTL */
                int sub = (op >> 5) & 0x7;
                int src_b = (op >> 9) & 1;
                int dst_b = (op >> 8) & 1;
                int shift_raw = op & 0x1F;
                int shift = (shift_raw & 0x10) ? (shift_raw - 32) : shift_raw;
                int64_t src = src_b ? s->b : s->a;
                int64_t result = src;
                switch (sub) {
                case 0x4: { int64_t dst_in = dst_b ? s->b : s->a;
                            int64_t sh = (shift >= 0) ? (dst_in << shift)
                                                      : (dst_in >> (-shift));
                            result = src & sh; break; }
                case 0x5: { int64_t dst_in = dst_b ? s->b : s->a;
                            int64_t sh = (shift >= 0) ? (dst_in << shift)
                                                      : (dst_in >> (-shift));
                            result = src | sh; break; }
                case 0x6: { int64_t dst_in = dst_b ? s->b : s->a;
                            int64_t sh = (shift >= 0) ? (dst_in << shift)
                                                      : (dst_in >> (-shift));
                            result = src ^ sh; break; }
                case 0x7:   /* SFTL src,SHIFT,DST: 32-bit logical shift, sets C */
                    c54x_sftl_exec(s, op);
                    return consumed + s->lk_used;
                }
                if (dst_b) s->b = sext40(result); else s->a = sext40(result);
                return consumed + s->lk_used;
            }
            /* F2xx unmapped: log once, then NOP. This log firing means an F2xx
             * bit pattern is still uncovered. */
            { static int f2_unm = 0;
              if (f2_unm++ < 20)
                  C54_LOG("F2xx unmapped op=0x%04x PC=0x%04x (NOP)", op, s->pc); }
            return consumed + s->lk_used;
        }
        /* ⚠️ No LMS handler belongs in the F2xx/F3xx range. Per binutils
         * tic54x-opc.c, LMS is { "lms", 1,2,2, 0xE100, 0xFF00, {OP_Xmem,OP_Ymem} },
         * i.e. hi8 == 0xE1, and that handler already exists. An LMS catch-all here
         * steals every F3xx instruction before the F3 dispatch sees it: for 0xF3E1
         * (SFTL B,1,B, 4872 sites in the firmware) it computed a junk Ymem value
         * and wrote it through data_write(s, AR1, ...), which with AR1=0 lands on
         * MMR_IMR. Measured: IMR writes 0x0000 -> {0x0540, 0x0525, 0x082b, 0xfd57,
         * 0xfacf, ...}, all at PC=0x8eb9 op=0xf3e1 with XPC=0, i.e. genuine PROM0
         * execution. F2xx outside F272/F273/F274 falls through to the F-class NOP
         * fallback; the firmware does not appear to use it. */
        /* F8xx: branches, RPT, BANZ, CALL, RET variants */
        if (hi8 == 0xF8) {
            uint8_t sub = (op >> 4) & 0xF;
            /* F820 (624 sites) and F830 (543 sites) are BC pmad,cond per
             * tic54x-opc.c (bc = 0xF800 mask 0xFF00). The dispatcher at
             * PROM0 0xb968-0xb9a4 relies on these branching when the ACC
             * comparison succeeds. Cond 0x20 = C set, cond 0x30 = ?
             * (we treat both via ACC compare for now since dispatcher uses
             * cmp-style behaviour). The full F8xx range is BC per binutils
             * but historically the firmware tolerates the legacy decode
             * for the other sub-codes — surgical override here only.
             *
             * ⚠️ Switching F82x/F83x to strict SPRU172C condition evaluation
             * (cond 0x20 = NTC, cond 0x30 = TC) wedged the DSP (stuck at
             * PC=0xcc51 or 0xfa95 depending on the run, task 24 dropping to 0).
             * Check that BITF (0x61) really sets TC before trying it again. */
            if (sub == 0x2 || sub == 0x3) {
                op2 = prog_fetch(s, s->pc + 1);
                consumed = 2;
                int64_t acc_signed = (s->a & 0x8000000000LL)
                                     ? (s->a | ~0xFFFFFFFFFFLL) : s->a;
                bool take = false;
                /* Real TC semantics (SPRU172C: F820 = bc ntc, F830 = bc tc) are used
                 * ONLY when the previous instruction actually sets TC, i.e. cmpm or
                 * bitf (0x60xx/0x61xx, mask 0xFE00); everywhere else the inherited
                 * ACC heuristic stays, because applying TC-strict across the board
                 * wedges the dispatcher sites.
                 * Without it the bootloader poll at 0xb427 froze: `bc ntc` after
                 * `cmpm #2` must leave on TC=1 (data == 2) towards bacc 0x7000,
                 * while the ACC heuristic (take when A != 0) never let it out.
                 *
                 * CALYPSO_C54X_BCTC_SM (default OFF) extends the same real BC TC
                 * semantics to 0xde0d..0xde26 only: that handshake state-machine
                 * loop ends on F830 de0d = BC TC, nothing in it sets TC (6d91 is a
                 * MAR, f5a9 an RPT fallback), so on real silicon TC=0, the branch is
                 * not taken, the loop runs ONCE and falls to 0xde28 and the setter
                 * at 0xde9c. Under the ACC heuristic (A=0 here via f0e1) it spins
                 * forever. */
                static int bctc_sm = -1;
                if (bctc_sm < 0) bctc_sm = calypso_gate("CALYPSO_C54X_BCTC_SM", 0);
                bool tc_strict = ((g_prev_op & 0xFE00) == 0x6000) ||
                                 (bctc_sm && s->pc >= 0xde0d && s->pc <= 0xde26);
                if (tc_strict) {
                    bool tc = (s->st0 & ST0_TC) != 0;
                    take = (sub == 0x2) ? !tc : tc;     /* 0x2=NTC, 0x3=TC */
                } else {
                    if (sub == 0x2)      take = (acc_signed != 0);
                    else /* sub==0x3 */  take = (acc_signed == 0);
                }
                if (take) { s->pc = op2; return 0; }
                return consumed + s->lk_used;
            }
            /* Per tic54x-opc.c:
             *   F880-F8FF mask FF80 = FB pmad (FAR branch unconditional)
             * The low 7 bits of the opcode word encode the target XPC bits.
             * Calypso uses 2-bit XPC, so & 0x3 is sufficient.
             *
             * ⚠️ Treating this range as plain B pmad keeps XPC at 0 forever and
             * the DSP never reaches the PROM1 user code. */
            if ((op & 0xFF80) == 0xF880) {
                op2 = prog_fetch(s, s->pc + 1);
                consumed = 2;
                uint8_t new_xpc = (op & 0x7F) & 0x03;
                static uint64_t fb_total;
                fb_total++;
                if (fb_total <= 30 || (fb_total % 5000) == 0) {
                    C54_LOG("FB FAR #%llu PC=0x%04x → XPC=%u PC=0x%04x (was XPC=%u)",
                            (unsigned long long)fb_total, s->pc,
                            new_xpc, op2, s->xpc);
                }
                s->xpc = new_xpc;
                s->pc  = op2;
                return 0;
            }
            /* F88x..F8Bx (mask FF80=0): historic plain B pmad (NEAR), kept
             * for sub-codes that fall outside the FAR mask above. */
            if (sub >= 0x8 && sub <= 0xB) {
                op2 = prog_fetch(s, s->pc + 1);
                s->pc = op2;
                return 0;
            }
            /* F86x/F87x: BANZ *ARn, pmad — branch if ARn != 0 (2 words) */
            if (sub == 0x6 || sub == 0x7) {
                op2 = prog_fetch(s, s->pc + 1);
                int ar_idx = op & 0x07;
                if (s->ar[ar_idx] != 0) {
                    s->ar[ar_idx]--;
                    s->pc = op2;
                    return 0;
                }
                return 2;  /* skip 2 words, fall through */
            }
            /* F84x/F85x: BC pmad with an ACC condition (2 words). These are BC, NOT
             * BANZ: BANZ is 0x6Cxx, a disjoint encoding (SPRU172C). The low byte is
             * the condition: cc&0x40 selects the ACC group, cc&0x08 picks B over A,
             * cc&0x07 the test {2:GEQ 3:LT 4:NEQ 5:EQ 6:GT 7:LEQ}.
             * ⚠️ Decoding them as `BANZ ARn` tests and decrements an AR instead of
             * the accumulator. The firmware at 0x772f emits F844 = BC 0x7737,ANEQ;
             * branching on AR4 != 0 lands on the POPM ST0 at 0x7737 without its
             * PSHM ST0 (0x770d is a disjoint path), so SP climbs by one per turn
             * until it collapses. BC pushes nothing, the stack stays intact.
             * Sub-codes 0x2/0x3 keep the dialect decode above, and this is disjoint
             * from the 0x7000 catch-all (STM #lk,SP). */
            if (sub == 0x4 || sub == 0x5) {
                op2 = prog_fetch(s, s->pc + 1);
                consumed = 2;
                uint8_t cc = op & 0xFF;
                int64_t acc  = (cc & 0x08) ? s->b : s->a;
                int64_t accs = (acc & 0x8000000000LL) ? (acc | ~0xFFFFFFFFFFLL) : acc;
                bool take;
                switch (cc & 0x07) {
                case 0x2: take = (accs >= 0); break;   /* AGEQ */
                case 0x3: take = (accs <  0); break;   /* ALT  */
                case 0x4: take = (accs != 0); break;   /* ANEQ */
                case 0x5: take = (accs == 0); break;   /* AEQ  */
                case 0x6: take = (accs >  0); break;   /* AGT  */
                case 0x7: take = (accs <= 0); break;   /* ALEQ */
                default:  take = false;       break;   /* 0/1 reserved */
                }
                if (take) { s->pc = op2; return 0; }
                return consumed + s->lk_used;
            }
            /* F8Cx-F8Fx: CALL/CALLD pmad (2 words) */
            if (sub >= 0xC) {
                op2 = prog_fetch(s, s->pc + 1);
                s->sp--;
                data_write(s, s->sp, (uint16_t)(s->pc + 2));
                s->pc = op2;
                return 0;
            }
            /* F80x-F81x: BANZ pmad, Smem (2 words)
             * Per SPRU172C + tic54x-opc.c: entire F8xx range is BANZ.
             * Sind operand selects AR via op[2:0] (nar). Test pre-mod
             * value; resolve_smem applies Sind post-mod. Same off-by-ARP
             * fix as 0x6C00 / 0x6E00 BANZ/BANZD. */
            if (sub <= 0x1) {
                int nar = op & 0x07;
                uint16_t old_ar = s->ar[nar];
                addr = resolve_smem(s, op, &ind);
                op2 = prog_fetch(s, s->pc + 1);
                consumed = 2;
                if (old_ar != 0) {
                    s->pc = op2;
                    return 0;
                }
                return consumed + s->lk_used;
            }
            /* Fallback: RPT Smem (F8xx sub not handled above) */
            addr = resolve_smem(s, op, &ind);
            s->rpt_count = data_read(s, addr);
            s->rpt_active = true; s->rpt_fresh = true;
            s->pc += consumed;
            return 0;
        }
        /* F3xx: dispatch per binutils tic54x-opc.c (verified against
         * insn_template struct include/opcode/tic54x.h:85-150).
         *
         * 8 sub-families:
         *   F300-F31F  INTR k                                 1-word
         *   F320-F32F  unmapped                               (NOP fallback)
         *   F330-F35F  AND/OR/XOR #lk,SHIFT,SRC,DST  mask FCF0 2-word
         *   F360-F367  ADD/SUB/AND/OR/XOR/MAC #lk var. FCFF   2-word
         *   F368-F37F  unmapped                               (NOP fallback)
         *   F380-F39F  AND  src,SHIFT,DST            mask FCE0 1-word
         *   F3A0-F3BF  OR   src,SHIFT,DST            mask FCE0 1-word
         *   F3C0-F3DF  XOR  src,SHIFT,DST            mask FCE0 1-word
         *   F3E0-F3FF  SFTL src,SHIFT,DST            mask FCE0 1-word
         *
         * Dispatch order: most-specific masks first (FCFF -> FCF0 -> FCE0).
         *
         * ⚠️ An "F320+ -> LD #k9, DP" fallback here mis-decodes 364 firmware sites
         * and wedges the DSP at PC=0x8eb9 (0xF3E1 SFTL B,1,B). */
        if (hi8 == 0xF3) {
            /* ⚠️ No "INTR k" handler belongs at 0xF300-0xF31F. Per binutils
             * tic54x-opc.c, INTR is { "intr", 1,1,1, 0xF7C0, 0xFFE0, ... }: base
             * 0xF7C0, not 0xF300. The F3xx range belongs to the ALU #lk class
             * (mask 0xFCF0), so F310 and friends must reach the FCF0 dispatch
             * below. Measured with an INTR handler in place: 0xe9a0 `f310`, meant
             * to be `SUB #5,B,B`, pushed PC+1 and jumped into the vector table, so
             * B stayed 0, the following STLM B,AR3 at 0xe9a2 fired 10243 times with
             * AR3 = 0, BANZ fc54,*AR3- looped forever at fc50..fc6d and the INT3
             * ISR never returned, leaving INTM at 1.
             * No F7Cx site has been observed in any run, so no real INTR handler
             * has been added there yet. */

            /* F360-F367: 2-word with mask FCFF (#lk<<16 variants).
             * Most-specific mask, check first. */
            if ((op & 0xFCFF) == 0xF060 ||  /* ADD #lk<<16, src, [dst] */
                (op & 0xFCFF) == 0xF061 ||  /* SUB */
                (op & 0xFCFF) == 0xF063 ||  /* AND */
                (op & 0xFCFF) == 0xF064 ||  /* OR  */
                (op & 0xFCFF) == 0xF065 ||  /* XOR */
                (op & 0xFCFF) == 0xF067) {  /* MAC #lk, src, [dst] */
                op2 = prog_fetch(s, s->pc + 1 + (s->lk_used ? 1 : 0));
                consumed = 2;
                int sub = op & 0x7;
                int src_b = (op >> 9) & 1;
                int dst_b = (op >> 8) & 1;
                int64_t src = src_b ? s->b : s->a;
                int64_t result = src;
                switch (sub) {
                case 0x0: result = src + ((int64_t)(int16_t)op2 << 16); break;
                case 0x1: result = src - ((int64_t)(int16_t)op2 << 16); break;
                case 0x3: result = src & (((int64_t)op2) << 16); break;
                case 0x4: result = src | (((int64_t)op2) << 16); break;
                case 0x5: result = src ^ (((int64_t)op2) << 16); break;
                case 0x7: { /* MAC: dst = src + T * lk */
                    int64_t prod = (int64_t)(int16_t)s->t * (int64_t)(int16_t)op2;
                    if (s->st1 & ST1_FRCT) prod <<= 1;
                    result = src + prod;
                    break;
                }
                }
                if (dst_b) s->b = sext40(result); else s->a = sext40(result);
                return consumed + s->lk_used;
            }

            /* F300-F35F: 2-word with mask FCF0 (ALU #lk + 4-bit shift).
             * ADD (sub=0), SUB (sub=1), LD (sub=2), AND (sub=3), OR (sub=4),
             * XOR (sub=5).
             *
             * ⚠️ ADD/SUB/LD belong here too; without them 0xf310 = SUB #lk,B,B at
             * PC=0xe9a0 is mis-decoded, B stays 0, AR3 becomes 0 and the loop at
             * fc50 never ends. */
            if ((op & 0xFCF0) == 0xF000 ||  /* ADD #lk, SHIFT, src, [dst] */
                (op & 0xFCF0) == 0xF010 ||  /* SUB  <- 0xf310 matches here */
                (op & 0xFCF0) == 0xF020 ||  /* LD  (binutils mask FEF0, no src) */
                (op & 0xFCF0) == 0xF030 ||  /* AND */
                (op & 0xFCF0) == 0xF040 ||  /* OR */
                (op & 0xFCF0) == 0xF050) {  /* XOR */
                op2 = prog_fetch(s, s->pc + 1 + (s->lk_used ? 1 : 0));
                consumed = 2;
                int subop = (op >> 4) & 0xF;
                int shift_raw = op & 0xF;
                int shift = shift_raw;          /* SHFT: 4-bit unsigned (SPRU172C), see the F0/F1 copy */
                int src_b = (op >> 9) & 1;
                int dst_b = (op >> 8) & 1;
                int64_t src = src_b ? s->b : s->a;
                /* ADD/SUB/LD : lk signed-extended ; AND/OR/XOR : lk unsigned. */
                int64_t lk_val;
                if (subop <= 2) {
                    int64_t lk_signed = (int64_t)(int16_t)op2;
                    lk_val = (shift >= 0) ? (lk_signed << shift)
                                          : (lk_signed >> (-shift));
                } else {
                    int64_t lk_unsigned = (int64_t)(uint16_t)op2;
                    lk_val = (shift >= 0) ? (lk_unsigned << shift)
                                          : (lk_unsigned >> (-shift));
                }
                int64_t result = src;
                switch (subop) {
                case 0x0: result = src + lk_val; break;   /* ADD */
                case 0x1: result = src - lk_val; break;   /* SUB */
                case 0x2: result = lk_val; break;         /* LD (src ignored) */
                case 0x3: result = src & lk_val; break;   /* AND */
                case 0x4: result = src | lk_val; break;   /* OR  */
                case 0x5: result = src ^ lk_val; break;   /* XOR */
                }
                if (dst_b) s->b = sext40(result); else s->a = sext40(result);
                return consumed + s->lk_used;
            }

            /* F380-F3FF: 1-word AND/OR/XOR/SFTL src,SHIFT,DST (mask FCE0).
             * Sub-opcode in bits 7-5: 100=AND, 101=OR, 110=XOR, 111=SFTL. */
            if ((op & 0xFCE0) == 0xF080 ||  /* AND */
                (op & 0xFCE0) == 0xF0A0 ||  /* OR  */
                (op & 0xFCE0) == 0xF0C0 ||  /* XOR */
                (op & 0xFCE0) == 0xF0E0) {  /* SFTL */
                int sub = (op >> 5) & 0x7;
                int src_b = (op >> 9) & 1;
                int dst_b = (op >> 8) & 1;
                int shift_raw = op & 0x1F;
                int shift = (shift_raw & 0x10) ? (shift_raw - 32) : shift_raw;
                int64_t src = src_b ? s->b : s->a;
                int64_t result = src;
                switch (sub) {
                case 0x4: { /* AND src,SHIFT,DST: DST = SRC & (DST_in << shift) */
                    int64_t dst_in = dst_b ? s->b : s->a;
                    int64_t sh = (shift >= 0) ? (dst_in << shift) : (dst_in >> (-shift));
                    result = src & sh;
                    break;
                }
                case 0x5: { /* OR */
                    int64_t dst_in = dst_b ? s->b : s->a;
                    int64_t sh = (shift >= 0) ? (dst_in << shift) : (dst_in >> (-shift));
                    result = src | sh;
                    break;
                }
                case 0x6: { /* XOR */
                    int64_t dst_in = dst_b ? s->b : s->a;
                    int64_t sh = (shift >= 0) ? (dst_in << shift) : (dst_in >> (-shift));
                    result = src ^ sh;
                    break;
                }
                case 0x7:   /* SFTL src,SHIFT,DST: 32-bit logical shift, sets C */
                    c54x_sftl_exec(s, op);
                    return consumed + s->lk_used;
                }
                if (dst_b) s->b = sext40(result); else s->a = sext40(result);
                return consumed + s->lk_used;
            }

            /* F320-F32F + F368-F37F: unmapped per binutils. NOP fallback +
             * log-once for diagnostic. 9 firmware sites total. */
            {
                static int unmapped_log = 0;
                if (unmapped_log++ < 20)
                    C54_LOG("F3xx unmapped op=0x%04x PC=0x%04x (NOP)",
                            op, s->pc);
            }
            return consumed + s->lk_used;
        }
        /* F6xx: various — LD/ST acc-acc, ABDST, SACCD, etc. */
        if (hi8 == 0xF6) {
            uint8_t sub = (op >> 4) & 0xF;
            if (sub == 0x2) {
                /* F62x: LD A, dst_shift, B or LD B, dst_shift, A */
                int dst = op & 1;
                if (dst) s->b = s->a; else s->a = s->b;
                return consumed + s->lk_used;
            }
            if (sub == 0x6) {
                /* F66x: LD A/B with shift to other acc */
                int dst = op & 1;
                if (dst) s->b = s->a; else s->a = s->b;
                return consumed + s->lk_used;
            }
            if (sub == 0xB) {
                /* F6Bx: RSBX -- reset bit in ST1 (bit 9=1, bit 8=0).
                 * Per tic54x-opc.c: RSBX 0xF4B0 mask 0xFDF0 covers F6Bx. */
                int bit = op & 0x0F;
                rsbx_intm_check(s, op);  /* INTM-clear probe */
                s->st1 &= ~(1 << bit);
                return consumed + s->lk_used;
            }
            /* Delayed branches/calls/returns from PROM (per tic54x-opc.c).
             * MUST be checked BEFORE the MVDD catch-all because they share
             * the high nibbles 0xE/0x9. Without these the DSP cannot return
             * from interrupt service routines: without RETED in particular, INTM
             * stays at 1 forever, every later INT3 is blocked and the ARM/DSP frame
             * loop stalls.
             *
             * All delayed forms execute 2 delay-slot words before the jump
             * commits; we arm the existing delayed_pc/delay_slots machinery
             * (the same one RCD uses) so the slots run with the right PC. */
            if (op == 0xF6EB) {
                /* RETED — return from interrupt, enable interrupts, delayed.
                 * Pop PC, clear INTM, then run 2 delay slots before jumping. */
                uint16_t ra = data_read(s, s->sp); s->sp++;
                s->st1 &= ~ST1_INTM;
                s->delayed_pc  = ra;
                s->delay_slots = 2;
                if (g_vec28_tracing) {
                    g_vec28_trace_pops++;
                    fprintf(stderr, "[c54x] VEC28-STACK-TRACE #%u RETED(1w,no-xpc-pop) PC=0x%04x "
                            "popped=0x%04x words=1 SP_before=0x%04x SP_after=0x%04x "
                            "insn=%u\n", g_vec28_trace_pops, s->pc, ra,
                            (uint16_t)(s->sp - 1), s->sp, s->insn_count);
                    if (s->sp >= (uint16_t)(g_vec28_sp_entry + 2)) {
                        fprintf(stderr, "[c54x] VEC28-STACK-TRACE DISARMED SP=0x%04x "
                                "back at or above pre-interrupt level 0x%04x\n",
                                s->sp, g_vec28_sp_entry);
                        g_vec28_tracing = false;
                    }
                }
                {
                    static uint64_t reted_count;
                    reted_count++;
                    if (reted_count <= 20 || (reted_count % 100) == 0)
                        C54_LOG("RETED-FIRED #%llu PC=0x%04x -> ra=0x%04x SP=0x%04x INTM=0 [XPC non poppe : si fire post-XPC-fix = reopener drain, fix en pending-XPC]",
                                (unsigned long long)reted_count,
                                s->pc, ra, s->sp);
                }
                return consumed + s->lk_used;
            }
            if (op == 0xF69B) {
                /* RETFD — fast return, delayed (no INTM change). */
                uint16_t ra = data_read(s, s->sp); s->sp++;
                s->delayed_pc  = ra;
                s->delay_slots = 2;
                return consumed + s->lk_used;
            }
            if (op == 0xF6E2 || op == 0xF6E3) {
                /* CALAD probe at 0x8353, the site known to self-loop: dumps XPC and
                 * the full A on the first hit. CALAD preserves XPC per SPRU172C, so
                 * XPC=1 at entry means the firmware was already on the far page,
                 * while XPC=0 means it handed a far pointer to a near call. */
                if (s->pc == 0x8353) {
                    static int p8353_first = 0;
                    if (!p8353_first) {
                        p8353_first = 1;
                        C54_LOG("PROBE-CALAD-8353-FIRST insn=%u XPC=%u "
                                "A=%010llx (A_G=0x%02x A_H=0x%04x A_L=0x%04x) "
                                "B=%010llx SP=0x%04x PMST=0x%04x",
                                s->insn_count, s->xpc & 0x3,
                                (unsigned long long)(s->a & 0xFFFFFFFFFFULL),
                                (uint8_t)((s->a >> 32) & 0xFF),
                                (uint16_t)((s->a >> 16) & 0xFFFF),
                                (uint16_t)(s->a & 0xFFFF),
                                (unsigned long long)(s->b & 0xFFFFFFFFFFULL),
                                s->sp, s->pmst);
                    }
                }
                /* BACCD A / CALAD A — delayed branch/call to acc(low).
                 * 1-word op + 2 delay slots. CALAD pushes PC+3 (skip op +
                 * 2 delay slots) per TI convention (cf. CALLD which pushes
                 * PC+4 for its 2-word form). Branch is armed via the
                 * delayed_pc/delay_slots mechanism so the 2 slots run
                 * before PC commits to tgt. */
                uint16_t tgt = (uint16_t)(s->a & 0xFFFF);
                bool is_call = (op == 0xF6E3);
                static uint64_t bcd_total;
                bcd_total++;
                /* Pre-load context: dump the 8 words preceding PC (in OVLY
                 * the executor reads from DARAM, mirror that). Lets us see
                 * which LD/MAR sequence was supposed to put a valid target
                 * in A before the CALAD/BACCD. */
                int pre_ovly = (s->pmst & PMST_OVLY) && s->pc >= 0x80 && s->pc < 0x2800;
                uint16_t pre[8];
                for (int i = 0; i < 8; i++) {
                    uint16_t a = (uint16_t)(s->pc - 8 + i);
                    pre[i] = pre_ovly ? s->data[a] : s->prog[a];
                }
                if (bcd_total <= 60 || (bcd_total % 5000) == 0) {
                    C54_LOG("BCD/CAD F6E%c #%llu PC=0x%04x tgt=0x%04x A=%010llx SP=0x%04x DP=0x%03x mem[%c PC-8..-1]=%04x %04x %04x %04x %04x %04x %04x %04x%s",
                            is_call ? '3' : '2',
                            (unsigned long long)bcd_total,
                            s->pc, tgt,
                            (unsigned long long)(s->a & 0xFFFFFFFFFFULL),
                            s->sp,
                            (s->st0 & 0x1FF),
                            pre_ovly ? 'D' : 'P',
                            pre[0], pre[1], pre[2], pre[3],
                            pre[4], pre[5], pre[6], pre[7],
                            is_call ? " CALAD" : " BACCD");
                }
                if (is_call) {
                    uint16_t ret_pc = (uint16_t)(s->pc + 3);
                    s->sp = (s->sp - 1) & 0xFFFF;
                    data_write(s, s->sp, ret_pc);
                }
                s->delayed_pc  = tgt;
                s->delay_slots = 2;
                return consumed + s->lk_used;
            }
            if (op == 0xF6E4 || op == 0xF6E5) {
                /* FRETD / FRETED — far return, delayed.
                 * Pops XPC and PC unconditionally (FL_FAR); FRETED also clears INTM.
                 * ⚠️ Do not gate the pop on PMST bit 4: that bit is AVIS, not APTS,
                 * and carries no stack semantics. */
                s->xpc = data_read(s, s->sp); s->sp++;
                if (s->xpc > 3) s->xpc &= 3;
                uint16_t ra = data_read(s, s->sp); s->sp++;
                if (op == 0xF6E5) s->st1 &= ~ST1_INTM;
                if (g_vec28_tracing) {
                    g_vec28_trace_pops++;
                    fprintf(stderr, "[c54x] VEC28-STACK-TRACE #%u FRETD-FRETED(2w,symmetric) PC=0x%04x "
                            "popped=0x%04x words=2 SP_before=0x%04x SP_after=0x%04x "
                            "insn=%u\n", g_vec28_trace_pops, s->pc, ra,
                            (uint16_t)(s->sp - 2), s->sp, s->insn_count);
                    if (s->sp >= (uint16_t)(g_vec28_sp_entry + 2)) {
                        fprintf(stderr, "[c54x] VEC28-STACK-TRACE DISARMED SP=0x%04x "
                                "back at or above pre-interrupt level 0x%04x\n",
                                s->sp, g_vec28_sp_entry);
                        g_vec28_tracing = false;
                    }
                }
                s->delayed_pc  = ra;
                s->delay_slots = 2;
                return consumed + s->lk_used;
            }
            if (op == 0xF6E6 || op == 0xF6E7) {
                /* FBACCD A / FCALAD A — far delayed branch/call to A.
                 * A(22:16) -> XPC, A(15:0) -> target; the XPC update is immediate,
                 * like FRETED. FCALAD pushes XPC first, then the return PC+3, the
                 * order FRETD pops in. 2 delay slots. */
                uint16_t tgt = (uint16_t)(s->a & 0xFFFF);
                uint8_t  new_xpc = (uint8_t)((s->a >> 16) & 0xFF);
                if (new_xpc > 3) new_xpc &= 3;
                bool is_call = (op == 0xF6E7);
                static uint64_t fbcd_total;
                fbcd_total++;
                if (fbcd_total <= 10 || (fbcd_total % 5000) == 0) {
                    C54_LOG("FBCD/FCAD F6E%c #%llu PC=0x%04x tgt=0x%04x newXPC=%u A=%010llx SP=0x%04x%s",
                            is_call ? '7' : '6',
                            (unsigned long long)fbcd_total,
                            s->pc, tgt, new_xpc,
                            (unsigned long long)(s->a & 0xFFFFFFFFFFULL),
                            s->sp,
                            is_call ? " FCALAD" : " FBACCD");
                }
                if (is_call) {
                    /* FCALAD (F6E7): pushes XPC and the return PC unconditionally
                     * (FL_FAR). ⚠️ Not gated on PMST bit 4, which is AVIS. */
                    s->sp = (s->sp - 1) & 0xFFFF;
                    data_write(s, s->sp, s->xpc);
                    uint16_t ret_pc = (uint16_t)(s->pc + 3);
                    s->sp = (s->sp - 1) & 0xFFFF;
                    data_write(s, s->sp, ret_pc);
                }
                s->xpc         = new_xpc;
                s->delayed_pc  = tgt;
                s->delay_slots = 2;
                return consumed + s->lk_used;
            }
            if (sub >= 0x8) {
                /* F68x-F6Fx: MVDD Xmem, Ymem — dual data-memory operand move
                 * Encoding: 1111 0110 XXXX YYYY
                 *   bit 7   = Xmod (0=inc, 1=dec)
                 *   bits 6:4 = Xar  (source AR register)
                 *   bit 3   = Ymod (0=inc, 1=dec)
                 *   bits 2:0 = Yar  (dest AR register) */
                int xar = (op >> 4) & 0x07;
                int yar = op & 0x07;
                uint16_t val = data_read(s, s->ar[xar]);
                data_write(s, s->ar[yar], val);
                if ((op >> 7) & 1) s->ar[xar]--; else s->ar[xar]++;
                if ((op >> 3) & 1) s->ar[yar]--; else s->ar[yar]++;
                return consumed + s->lk_used;
            }
            /* Other F6xx: treat as NOP for now */
            return consumed + s->lk_used;
        }
        /* F5xx: SSBX or RPT #k */
        if (hi8 == 0xF5) {
            /* F5Bx: SSBX -- set bit in ST0 (bit 9=0, bit 8=1).
             * Per tic54x-opc.c: SSBX 0xF5B0 mask 0xFDF0. */
            if ((op & 0xFFF0) == 0xF5B0) {
                int bit = op & 0x0F;
                s->st0 |= (1 << bit);
                return consumed + s->lk_used;
            }
            /* Note: 0xF5E2/F5E3 (BACC B / CALA B) are handled earlier alongside
             * their F4 counterparts, so they never reach this F5xx block. */
            /* RPT #k (short immediate) — kept as fallback, must advance PC. */
            s->rpt_count = op & 0xFF;
            s->rpt_active = true; s->rpt_fresh = true;
            s->pc += 1;
            return 0;
        }
        /* DIAG: log F7xx executions before the (buggy) LD #k8 dispatch.
         * Per tic54x-opc.c the F7xx range contains SSBX ST1 (0xF7Bx) and
         * other instructions, NOT LD #k8 (which is at E800-E9FF).
         * Caps at 5 per distinct sub-opcode to avoid spam. */
        if (hi8 == 0xF7) {
            static int f7xx_seen[256] = {0};
            int sub_idx = op & 0xFF;
            if (++f7xx_seen[sub_idx] <= 100 || (f7xx_seen[sub_idx] % 1000) == 0) {
                C54_LOG("F7xx EXEC op=0x%04x PC=0x%04x XPC=%d insn=%u",
                        op, s->pc, s->xpc, s->insn_count);
            }
        }
        /* F7Bx: SSBX bit, ST1 (incl. SSBX INTM at F7BB).
         * Per binutils tic54x-opc.c: opcode "ssbx" 0xF5B0 mask 0xFDF0,
         * where bit 9 selects ST0 (0xF5Bx) vs ST1 (0xF7Bx).
         * Symmetric counterpart of RSBX ST1 (F6Bx) handler above.
         * MUST be tested before the F7xx LD #k8 dispatch (which is
         * itself incorrect — per SPRU172C, LD #k8 lives at E800-E9FF). */
        if ((op & 0xFFF0) == 0xF7B0) {
            int bit = op & 0x0F;
            bool is_intm = (bit == 11);
            s->st1 |= (1 << bit);
            if (is_intm)
                C54_LOG("*** SSBX INTM (F7BB) *** PC=0x%04x ST1=0x%04x insn=%u",
                        s->pc, s->st1, s->insn_count);
            return consumed + s->lk_used;
        }
        /* F7xx: LD/ST #k to various registers */
        if (hi8 == 0xF7) {
            uint8_t sub = (op >> 4) & 0xF;
            uint16_t k = op & 0xFF;
            /* F7C0..F7DF = INTR k (handled elsewhere if implemented),
             * F7E0       = RESET (exact opcode, 0xFFFF mask per tic54x-opc.c)
             * F7E1..F7FF = reserved/undefined per SPRU172C.
             *   The old LD #k8 dispatch here corrupted BRC at op=0xF7E3
             *   (= sub 0xE, k=0xE3) inside the DSP idle loop @ PC=0x9b1d,
             *   making RPTB count wrong → DSP stuck in 0x9aXX..0x9bXX
             *   block. Silicon treats reserved opcodes as NOP, not LD. */
            /* FIX_F7_DELAYED — F7E2/F7E3 = BACCD B / CALAD B, F7E6/F7E7 =
             * FBACCD B / FCALAD B (tic54x-opc.c: baccd 0xF6E2 mask 0xFEFF, bit 8 =
             * accumulator). The block below files them under "reserved -> NOP",
             * which makes a CALAD B skip the call entirely; PROM0..3 hold 51 sites
             * of f7e3. Same semantics as the A variants in the F6 block: return
             * PC+3, 2 delay slots. */
            if (op == 0xF7E2 || op == 0xF7E3) {
                uint16_t tgt = (uint16_t)(s->b & 0xFFFF);
                if (op == 0xF7E3) {
                    uint16_t ret_pc = (uint16_t)(s->pc + 3);
                    s->sp = (s->sp - 1) & 0xFFFF;
                    data_write(s, s->sp, ret_pc);
                }
                s->delayed_pc  = tgt;
                s->delay_slots = 2;
                return consumed + s->lk_used;
            }
            if (op == 0xF7E6 || op == 0xF7E7) {
                uint16_t tgt = (uint16_t)(s->b & 0xFFFF);
                uint8_t  new_xpc = (uint8_t)((s->b >> 16) & 0xFF);
                if (new_xpc > 3) new_xpc &= 3;
                if (op == 0xF7E7) {
                    s->sp = (s->sp - 1) & 0xFFFF;
                    data_write(s, s->sp, s->xpc);
                    uint16_t ret_pc = (uint16_t)(s->pc + 3);
                    s->sp = (s->sp - 1) & 0xFFFF;
                    data_write(s, s->sp, ret_pc);
                }
                s->xpc         = new_xpc;
                s->delayed_pc  = tgt;
                s->delay_slots = 2;
                return consumed + s->lk_used;
            }
            if (sub == 0xE || sub == 0xF) {
                /* F7E0..F7FF : RESET (0xF7E0 exact) + reserved.
                 * Treat as NOP — don't touch BRC. RESET (0xF7E0) would
                 * soft-reset the DSP; if firmware ever issues it we'd
                 * jump to vec 0 = 0xFF80. For now leave as NOP — has
                 * not been observed as a legitimate firmware path. */
                return consumed + s->lk_used;
            }
            switch (sub) {
            case 0x0: /* F70x: LD #k8, ASM */
                s->st1 = (s->st1 & ~ST1_ASM_MASK) | (k & ST1_ASM_MASK);
                break;
            case 0x1: /* F71x: LD #k8, AR0 */
                s->ar[0] = k; break;
            case 0x2: /* F72x: LD #k8, AR1 */
                s->ar[1] = k; break;
            case 0x3: s->ar[2] = k; break;
            case 0x4: s->ar[3] = k; break;
            case 0x5: s->ar[4] = k; break;
            case 0x6: s->ar[5] = k; break;
            case 0x7: s->ar[6] = k; break;
            case 0x8: /* F78x: LD #k8, T */
                s->t = (s->st1 & ST1_SXM) ? (uint16_t)(int8_t)k : k; break;
            case 0x9: /* F79x: LD #k8, DP */
                s->st0 = (s->st0 & ~ST0_DP_MASK) | (k & ST0_DP_MASK);
                g_last_ldp_pc = s->pc; g_last_ldp_val = (k & ST0_DP_MASK); g_last_ldp_kind = 1;
                break;
            case 0xA: /* F7Ax: LD #k8, ARP */
                s->st0 = (s->st0 & ~ST0_ARP_MASK) | ((k & 7) << ST0_ARP_SHIFT); break;
            case 0xB: s->ar[7] = k; break; /* F7Bx: LD #k8, AR7 */
            case 0xC: /* F7Cx: LD #k8u, BK */
                /* Second BK write site (LD #k8,BK): names the writer and the value.
                 * BK=0 breaks circular addressing and sends AR2 running away
                 * (0xfa98/0xf17c). Remove together with the MMR_BK probe. */
                {
                    static uint32_t bkw2_n = 0;
                    if (bkw2_n < 40) {
                        fprintf(stderr, "[c54x] BK-WR (F7Cx LD#k) 0x%04x→0x%04x PC=0x%04x "
                                "%s insn=%u\n", s->bk, k, s->pc,
                                (k == 0) ? "<<< BK=0 (casse circular!)" : "", s->insn_count);
                        bkw2_n++;
                    }
                }
                s->bk = k; break;
            case 0xD: sp_abs_track(s, k, 1); s->sp = k; break;  /* LD #k8u, SP */
            }
            return consumed + s->lk_used;
        }
        /* F9xx encoding split per tic54x-opc.c:
         *   F900-F97F mask FF00 = CC pmad cond (NEAR conditional call)
         *   F980-F9FF mask FF80 = FCALL pmad   (FAR call unconditional)
         * The bit 7 of the opcode low byte distinguishes them. */
        if (hi8 == 0xF9) {
            op2 = prog_fetch(s, s->pc + 1);
            consumed = 2;
            /* FCALL FAR: pushes XPC and the return PC unconditionally (FL_FAR).
             * Per binutils tic54x-opc.c (fcall 0xF980 mask 0xFF80, FL_FAR) and
             * SPRU172C, a FAR call always saves XPC for FRET to restore.
             * ⚠️ Do not gate the push on PMST bit 4 (AVIS, no stack semantics):
             * the firmware's 281 FCALL FAR sites would push PC only, against 142
             * FRET sites popping both PC and XPC. */
            if ((op & 0x80) != 0) {
                uint8_t new_xpc = (op & 0x7F) & 0x03;
                static uint64_t fcall_total;
                fcall_total++;
                s->sp = (s->sp - 1) & 0xFFFF;
                data_write(s, s->sp, s->xpc);
                s->sp = (s->sp - 1) & 0xFFFF;
                data_write(s, s->sp, (uint16_t)(s->pc + 2));
                if (fcall_total <= 30 || (fcall_total % 5000) == 0) {
                    C54_LOG("FCALL FAR #%llu PC=0x%04x → XPC=%u PC=0x%04x (was XPC=%u SP=0x%04x)",
                            (unsigned long long)fcall_total, s->pc,
                            new_xpc, op2, s->xpc, s->sp);
                }
                s->xpc = new_xpc;
                s->pc  = op2;
                return 0;
            }
            /* The condition is the low byte (binutils condition_codes[]), decoded
             * by c54x_cond_true(). ⚠️ Reading (op>>4)&0xF instead takes the wrong
             * field: CC TC/NEQ/LT and friends then evaluate wrongly, the power scan
             * at 0xb1xx loses its pushes and the resulting over-pop ends in the
             * 0x70c3 self-CALA. */
            bool take = c54x_cond_true(s, (uint8_t)(op & 0x7F));
            if (take) {
                s->sp--;
                data_write(s, s->sp, (uint16_t)(s->pc + 2));
                /* CC leak tracer */
                {
                    static uint32_t cc_targets[64];
                    static uint32_t cc_counts[64];
                    static int cc_n = 0;
                    static uint32_t total_cc = 0;
                    bool found = false;
                    for (int i = 0; i < cc_n; i++) {
                        if (cc_targets[i] == op2) { cc_counts[i]++; found = true; break; }
                    }
                    if (!found && cc_n < 64) { cc_targets[cc_n] = op2; cc_counts[cc_n++] = 1; }
                    if ((++total_cc % 100) == 0) {
                        C54_LOG("F9xx CC TOP TARGETS (SP=0x%04x total=%u):", s->sp, total_cc);
                        for (int i = 0; i < cc_n && i < 10; i++)
                            C54_LOG("  CC→0x%04x count=%u", cc_targets[i], cc_counts[i]);
                    }
                }
                s->pc = op2;
                return 0;
            }
            return consumed + s->lk_used;
        }
        /* FAxx encoding split per tic54x-opc.c:
         *   FA80-FAFF mask FF80 = FBD pmad (FAR branch delayed)
         *   FA00-FA7F = various NEAR delayed ops (treated as branch). */
        if (hi8 == 0xFA) {
            op2 = prog_fetch(s, s->pc + 1);
            consumed = 2;
            if ((op & 0x80) != 0) {
                /* FBD FAR delayed branch — XPC change, no push */
                uint8_t new_xpc = (op & 0x7F) & 0x03;
                static uint64_t fbd_total;
                fbd_total++;
                if (fbd_total <= 30 || (fbd_total % 5000) == 0) {
                    C54_LOG("FBD FAR #%llu PC=0x%04x → XPC=%u PC=0x%04x (was XPC=%u, delayed 2 slots)",
                            (unsigned long long)fbd_total, s->pc,
                            new_xpc, op2, s->xpc);
                }
                s->xpc = new_xpc;
                s->delayed_pc  = op2;
                s->delay_slots = 2;
                return consumed + s->lk_used;
            }
            /* FA20/FA30 are BCD pmad,NTC/TC, the delayed siblings of F820/F830,
             * and use exactly the BC technique above: evaluate the real condition
             * ONLY when the preceding opcode is CMPM or BITF (0x60xx/0x61xx), which
             * set TC reliably; otherwise keep the inherited branch-always.
             * ⚠️ Evaluating TC/NTC for all of FA00-FA7F wedges the firmware in
             * stuck loops, because most FAxx callers do not set TC before
             * branching. Delayed: 2 delay slots, same mechanism as FBD FAR. */
            {
                uint8_t fa_sub = (op >> 4) & 0xF;
                if (fa_sub == 0x2 || fa_sub == 0x3) {
                    bool fa_tc_strict = (g_prev_op & 0xFE00) == 0x6000;
                    if (fa_tc_strict) {
                        bool tc = (s->st0 & ST0_TC) != 0;
                        bool take = (fa_sub == 0x2) ? !tc : tc;  /* 2=NTC,3=TC */
                        if (take) {
                            s->delayed_pc  = op2;
                            s->delay_slots = 2;
                        }
                        return consumed + s->lk_used;
                    }
                }
            }
            /* Accumulator conditions are really evaluated, and the delay honoured.
             *
             * [2026-08-22] Measured with the TOA-TRACE probe on a clean native run:
             *     @0x7918 BCD 0x7923 if B!=0 ... B=0
             *     @0x7923 *** BRANCH TAKEN ***
             * `BNEQ` with B=0 branched anyway, every time, because the fallback
             * below treats EVERY non-TC FAxx as an unconditional branch. The FB0
             * route (0x791c..0x7921), which computes
             * TOA = (d[0x3fb4]-3)*48 + d[0x0c3d], was therefore never taken and the
             * TOA came from the FB1 route with cpt1[0x3fb3] frozen at 296, outside
             * the 0..1249 range osmocom expects (prim_fbsb.c, BITS_PER_TDMA=1250).
             * BCD is also DELAYED: setting `s->pc = op2` immediately skipped the
             * slot 0x791a `stm #0x0030` that loads T = 48, the TOA step.
             *
             * Scope: only accumulator conditions (cc & 0x40), which test A or B and
             * are reliable. TC and carry keep the previous behaviour, since those
             * are the ones that wedge the firmware. The two sets are disjoint
             * (fa_sub 2/3 implies cc & 0x40 == 0). */
            {
                uint8_t _cc = (uint8_t)(op & 0x7F);
                if (_cc & 0x40) {
                    if (c54x_cond_true(s, _cc)) {
                        s->delayed_pc  = op2;
                        s->delay_slots = 2;
                    }
                    return consumed + s->lk_used;   /* not taken: fall through */
                }
            }
            /* NEAR FAxx fallback: simplified treat as branch (unchanged,
             * proven-safe default for every case not handled above). */
            s->pc = op2;
            return 0;
        }
        /* FBxx encoding split per tic54x-opc.c:
         *   FB80-FBFF mask FF80 = FCALLD pmad (FAR call delayed)
         *   FB00-FB7F mask FF00 = CCD pmad cond (NEAR conditional call delayed) */
        if (hi8 == 0xFB) {
            op2 = prog_fetch(s, s->pc + 1);
            consumed = 2;
            /* FCALLD FAR: pushes XPC and the return PC+4 unconditionally
             * (FL_FAR delayed). Per binutils, fcalld 0xFB80 mask 0xFF80,
             * FL_FAR|FL_DELAY. ⚠️ Not gated on PMST bit 4, which is AVIS. */
            if ((op & 0x80) != 0) {
                uint8_t new_xpc = (op & 0x7F) & 0x03;
                static uint64_t fcalld_total;
                fcalld_total++;
                s->sp = (s->sp - 1) & 0xFFFF;
                data_write(s, s->sp, s->xpc);
                s->sp = (s->sp - 1) & 0xFFFF;
                data_write(s, s->sp, (uint16_t)(s->pc + 4));
                if (fcalld_total <= 30 || (fcalld_total % 5000) == 0) {
                    C54_LOG("FCALLD FAR #%llu PC=0x%04x → XPC=%u PC=0x%04x (was XPC=%u SP=0x%04x, delayed)",
                            (unsigned long long)fcalld_total, s->pc,
                            new_xpc, op2, s->xpc, s->sp);
                }
                s->xpc = new_xpc;
                s->delayed_pc  = op2;
                s->delay_slots = 2;
                return consumed + s->lk_used;
            }
            /* Condition decoded from the low byte by c54x_cond_true(), as for CC
             * above. */
            bool take = c54x_cond_true(s, (uint8_t)(op & 0x7F));
            if (take) {
                /* CCD is DELAYED: arm delay_slots=2 and delayed_pc, as CALLD
                 * (f274) does. ⚠️ Branching immediately skips the 2 delay slots,
                 * and a push in a slot is then lost, giving the over-pop that ends
                 * in the 0x70c3 self-CALA. The pushed return is pc+4 (past CCD and
                 * its slots); when not taken, PC advances by consumed (2). */
                s->sp--;
                data_write(s, s->sp, (uint16_t)(s->pc + 4));
                s->delayed_pc  = op2;
                s->delay_slots = 2;
                return consumed + s->lk_used;
            }
            return consumed + s->lk_used;
        }
        /* FCxx: LD #k, 16, B */
        /* FCxx: RC cond / RET -- return conditional (1-word).
         * Per tic54x-opc.c: RET=0xFC00, RC=0xFC00 mask 0xFF00. */
        if (hi8 == 0xFC) {
            uint8_t cc = op & 0xFF;
            bool cond = false;
            /* Evaluate condition per tic54x-opc.c encoding:
             * CC1=0x40: accumulator test, CCB=0x08: use B (else A)
             * EQ=0x05, NEQ=0x04, LT=0x03, LEQ=0x07, GT=0x06, GEQ=0x02
             * OV=0x70, NOV=0x60, TC=0x30, NTC=0x20, C=0x0C, NC=0x08 */
            if (cc == 0x00) cond = true; /* UNC */
            else if (cc & 0x40) {
                /* Accumulator condition */
                int64_t acc = (cc & 0x08) ? sext40(s->b) : sext40(s->a);
                uint8_t test = cc & 0x07;
                bool ov = (cc & 0x08) ? (s->st0 & (1<<9)/*OVB*/) : (s->st0 & (1<<8)/*OVA*/);
                if ((cc & 0x70) == 0x70) cond = ov;        /* AOV/BOV */
                else if ((cc & 0x70) == 0x60) cond = !ov;  /* ANOV/BNOV */
                else {
                    switch (test) {
                    case 0x05: cond = (acc == 0); break;  /* EQ */
                    case 0x04: cond = (acc != 0); break;  /* NEQ */
                    case 0x03: cond = (acc < 0); break;   /* LT */
                    case 0x07: cond = (acc <= 0); break;  /* LEQ */
                    case 0x06: cond = (acc > 0); break;   /* GT */
                    case 0x02: cond = (acc >= 0); break;  /* GEQ */
                    default: cond = true; break;
                    }
                }
            }
            else if ((cc & 0x30) == 0x30) cond = (s->st0 & ST0_TC) != 0; /* TC */
            else if ((cc & 0x30) == 0x20) cond = !(s->st0 & ST0_TC);     /* NTC */
            else if ((cc & 0x0C) == 0x0C) cond = (s->st0 & ST0_C) != 0;  /* C */
            else if ((cc & 0x0C) == 0x08) cond = !(s->st0 & ST0_C);      /* NC */
            else cond = true; /* unknown: take it */
            if (cond) {
                uint16_t ra = data_read(s, s->sp); s->sp++;
                {
                    static int rc_log = 0;
                    if (rc_log < 50)
                        C54_LOG("RC/RET PC=0x%04x cc=0x%02x -> ra=0x%04x SP=0x%04x",
                                s->pc, cc, ra, s->sp);
                    rc_log++;
                }
                if (g_vec28_tracing) {
                    g_vec28_trace_pops++;
                    fprintf(stderr, "[c54x] VEC28-STACK-TRACE #%u RC/RET(1w) PC=0x%04x "
                            "popped=0x%04x words=1 SP_before=0x%04x SP_after=0x%04x "
                            "insn=%u\n", g_vec28_trace_pops, s->pc, ra,
                            (uint16_t)(s->sp - 1), s->sp, s->insn_count);
                    if (s->sp >= (uint16_t)(g_vec28_sp_entry + 2)) {
                        fprintf(stderr, "[c54x] VEC28-STACK-TRACE DISARMED SP=0x%04x "
                                "back at or above pre-interrupt level 0x%04x\n",
                                s->sp, g_vec28_sp_entry);
                        g_vec28_tracing = false;
                    }
                }
                /* POST-BOOTSTUB-RET: a RET taken from the boot stub
                 * (PC in 0x0000..0x0008) is the exit of the task-switch trampoline
                 * 0x701b/0x701d -> 0x0000, and the popped return address is the PC
                 * of the task taking control. */
                if (s->pc <= 0x0008) {
                    static unsigned bsr;
                    bsr++;
                    if (bsr <= 200 || (bsr % 50) == 0) {
                        fprintf(stderr,
                                "[c54x] POST-BOOTSTUB-RET #%u PC=0x%04x -> task=0x%04x "
                                "SP_new=0x%04x B=0x%010llx INTM=%d insn=%u\n",
                                bsr, s->pc, ra, s->sp,
                                (unsigned long long)(s->b & 0xFFFFFFFFFFULL),
                                !!(s->st1 & ST1_INTM), s->insn_count);
                    }
                    /* DEEP-TRAIL (CALYPSO_DEBUG=BOOTSTUB_TRAIL): for the first 5
                     * POST-BOOTSTUB-RET events, dump pc_ring[-64..-1] to expose the
                     * caller chain leading to the stack-underflow loop. */
                    if (bsr <= 5 && calypso_debug_enabled("BOOTSTUB_TRAIL")) {
                        fprintf(stderr,
                            "[c54x] BOOTSTUB DEEP-TRAIL #%u (last 64 PCs):\n",
                            bsr);
                        for (int row = 0; row < 8; row++) {
                            fprintf(stderr, "[c54x] BS-DEEP[%3d..%3d] :",
                                    -64 + row*8, -57 + row*8);
                            for (int col = 0; col < 8; col++) {
                                int idx = -64 + row*8 + col;
                                fprintf(stderr, " %04x",
                                    pc_ring[(pc_ring_idx + idx) & 255]);
                            }
                            fprintf(stderr, "\n");
                        }
                        /* Also dump 16 stack words starting at SP. */
                        fprintf(stderr, "[c54x] BS-DEEP stack[SP..SP+15] :");
                        for (int i = 0; i < 16; i++) {
                            fprintf(stderr, " %04x",
                                s->data[(s->sp + i) & 0xFFFF]);
                        }
                        fprintf(stderr, "\n");
                    }
                }
                s->pc = ra;
                return 0;
            }
            return consumed + s->lk_used;
        }
        /* FDxx: LD #k, A (no shift) */
        if (hi8 == 0xFD) {
            int8_t k = (int8_t)(op & 0xFF);
            s->a = sext40((int64_t)k);
            return consumed + s->lk_used;
        }
        /* FExx: RCD cond / RETD -- return conditional delayed (1-word).
         * Per tic54x-opc.c: RETD=0xFE00, RCD=0xFE00 mask 0xFF00.
         * Simplified: immediate return (delay slots skipped). */
        if (hi8 == 0xFE) {
            uint8_t cc = op & 0xFF;
            bool cond = false;
            /* Evaluate condition per tic54x-opc.c encoding:
             * CC1=0x40: accumulator test, CCB=0x08: use B (else A)
             * EQ=0x05, NEQ=0x04, LT=0x03, LEQ=0x07, GT=0x06, GEQ=0x02
             * OV=0x70, NOV=0x60, TC=0x30, NTC=0x20, C=0x0C, NC=0x08 */
            if (cc == 0x00) cond = true; /* UNC */
            else if (cc & 0x40) {
                /* Accumulator condition */
                int64_t acc = (cc & 0x08) ? sext40(s->b) : sext40(s->a);
                uint8_t test = cc & 0x07;
                bool ov = (cc & 0x08) ? (s->st0 & (1<<9)/*OVB*/) : (s->st0 & (1<<8)/*OVA*/);
                if ((cc & 0x70) == 0x70) cond = ov;        /* AOV/BOV */
                else if ((cc & 0x70) == 0x60) cond = !ov;  /* ANOV/BNOV */
                else {
                    switch (test) {
                    case 0x05: cond = (acc == 0); break;  /* EQ */
                    case 0x04: cond = (acc != 0); break;  /* NEQ */
                    case 0x03: cond = (acc < 0); break;   /* LT */
                    case 0x07: cond = (acc <= 0); break;  /* LEQ */
                    case 0x06: cond = (acc > 0); break;   /* GT */
                    case 0x02: cond = (acc >= 0); break;  /* GEQ */
                    default: cond = true; break;
                    }
                }
            }
            else if ((cc & 0x30) == 0x30) cond = (s->st0 & ST0_TC) != 0; /* TC */
            else if ((cc & 0x30) == 0x20) cond = !(s->st0 & ST0_TC);     /* NTC */
            else if ((cc & 0x0C) == 0x0C) cond = (s->st0 & ST0_C) != 0;  /* C */
            else if ((cc & 0x0C) == 0x08) cond = !(s->st0 & ST0_C);      /* NC */
            else cond = true; /* unknown: take it */
            if (cond) {
                /* RCD is DELAYED: per SPRU172C the next 2 instructions after RCD
                 * execute before the return takes effect. Pop the return address
                 * now, let PC advance normally so those 2 instructions run as delay
                 * slots, and the main loop then forces PC = delayed_pc.
                 * ⚠️ Skipping the delay slots breaks FB detection: slots such as
                 * `LD #0, B` at PROM0 0x75ea never run, the accumulator stays stale
                 * and the dispatcher at 0x7700 loops forever. */
                uint16_t ra = data_read(s, s->sp); s->sp++;
                s->delayed_pc  = ra;
                s->delay_slots = 2;
                {
                    static int rcd_log = 0;
                    if (rcd_log < 50)
                        C54_LOG("RCD/RETD PC=0x%04x cc=0x%02x -> ra=0x%04x SP=0x%04x (delayed)",
                                s->pc, cc, ra, s->sp);
                    rcd_log++;
                }
                if (g_vec28_tracing) {
                    g_vec28_trace_pops++;
                    fprintf(stderr, "[c54x] VEC28-STACK-TRACE #%u RCD/RETD(1w,delayed) PC=0x%04x "
                            "popped=0x%04x words=1 SP_before=0x%04x SP_after=0x%04x "
                            "insn=%u\n", g_vec28_trace_pops, s->pc, ra,
                            (uint16_t)(s->sp - 1), s->sp, s->insn_count);
                    if (s->sp >= (uint16_t)(g_vec28_sp_entry + 2)) {
                        fprintf(stderr, "[c54x] VEC28-STACK-TRACE DISARMED SP=0x%04x "
                                "back at or above pre-interrupt level 0x%04x\n",
                                s->sp, g_vec28_sp_entry);
                        g_vec28_tracing = false;
                    }
                }
                return consumed + s->lk_used;
            }
            return consumed + s->lk_used;
        }
        /* FFxx is XC 2,cond — handled above with FDxx. No ADD here. */
        goto unimpl;

    case 0xE:
        /* Exxxx: single-word ALU, status, misc */
        /* ================================================================
         * 0xE0-0xE3 IS NOT `CMPS`. The real CMPS is 0x8E00/0xFE00, decoded
         * further down. Real encodings per SPRU172C (XXXXYYYY in bits 7:0):
         *     0xE0 FIRS  Xmem,Ymem,pmad   ** 2 WORDS **
         *     0xE1 LMS   Xmem,Ymem        1 word
         *     0xE2 SQDST Xmem,Ymem        1 word
         *     0xE3 ABDST Xmem,Ymem        1 word
         * ⚠️ The serious damage is counting 0xE0 as one word, which
         * desynchronises the stream. The word count is fixed first; E1-E3 are
         * left inert rather than made to write A/B/TC/TRN wrongly, since wrong
         * semantics are worse than none. Faithful LMS/SQDST/ABDST remain to be
         * written if a probe shows they matter.
         * ================================================================ */
        {
            if ((op & 0xFC00) == 0xE000) {
                static unsigned _e0_n = 0;
                if (hi8 == 0xE0) {
                    uint16_t pmad0 = prog_fetch(s, s->pc + 1);
                    consumed = 2;
                    /* FIRS-COEF (CALYPSO_FIRS_COEF, default OFF): where are the
                     * coefficients? pmad=0x0061 sits BELOW the OVLY window, which
                     * starts at 0x80, so prog_read falls back to s->prog[], where
                     * nothing is loaded under 0x7000 and reads give 0xF4E4.
                     * 0x0060-0x007F is the C54x DARAM scratch pad, so both spaces
                     * are dumped at FIRS time: a non-zero data[] means the OVLY
                     * alias should reach down to 0x60, a non-zero prog[] means MVDP
                     * already deposited them, and both zero means the table lives
                     * somewhere else. Read-only. */
                    {
                        static int _fc = -1;
                        if (_fc < 0) {
                            _fc = calypso_gate("CALYPSO_FIRS_COEF", 0);
                            fprintf(stderr, "[c54x] FIRS-COEF %s : dump prog[pmad..+5] "
                                    "ET data[pmad..+5] a chaque FIRS (pmad attendu 0x0061, "
                                    "sous la fenetre OVLY qui debute a 0x0080)\n",
                                    _fc ? "ACTIVE" : "INACTIVE (defaut)");
                        }
                        if (_fc) {
                            static unsigned _fcn = 0;
                            if (_fcn < 12) {
                                _fcn++;
                                char pb[96], db[96], sb2[96];
                                int po = 0, dof = 0, so = 0;
                                int nzp = 0, nzd = 0, nzs = 0;
                                /* Source of the table: the MVDD at 0x833c copies
                                 * data[0x2cbf] downwards into data[0x0060] upwards.
                                 * An empty source means an empty destination. */
                                for (int k = 0; k < 7; k++) {
                                    uint16_t sv = s->data[0x2CB9 + k];
                                    if (sv) nzs++;
                                    so += snprintf(sb2 + so, sizeof(sb2) - so, " %04x", sv);
                                }
                                /* Is the burst buffer alive when the DSP reads it?
                                 * The whole chain (0x8202 -> 0x2cba -> MVDD ->
                                 * 0x0061 -> FIRS) yields zero when its head, the
                                 * 0x2a00 buffer, is empty. An empty buffer HERE is
                                 * not a badly written one: DARAM-WR-JUDGE measured
                                 * coherence 0.998 at write time, under lock.
                                 * Also reports the spread of the two correlator
                                 * buffers (A=0x2c56, B=0x2c88, 50 words each,
                                 * BRC=0x31): distinct==1 means it is not
                                 * accumulating by shifting and the argmax is
                                 * meaningless. */
                                int dstA = 0, dstB = 0;
                                int16_t mnA = 32767, mxA = -32768;
                                int16_t mnB = 32767, mxB = -32768;
                                {
                                    int16_t va[50], vb[50];
                                    for (int k = 0; k < 50; k++) {
                                        va[k] = (int16_t)s->data[0x2C56 + k];
                                        vb[k] = (int16_t)s->data[0x2C88 + k];
                                        if (va[k] < mnA) mnA = va[k];
                                        if (va[k] > mxA) mxA = va[k];
                                        if (vb[k] < mnB) mnB = vb[k];
                                        if (vb[k] > mxB) mxB = vb[k];
                                    }
                                    for (int k = 0; k < 50; k++) {
                                        int dupA = 0, dupB = 0;
                                        for (int j = 0; j < k; j++) {
                                            if (va[j] == va[k]) dupA = 1;
                                            if (vb[j] == vb[k]) dupB = 1;
                                        }
                                        if (!dupA) dstA++;
                                        if (!dupB) dstB++;
                                    }
                                }
                                int nzb = 0; unsigned long eb = 0;
                                for (int k = 0; k < 304; k++) {
                                    int16_t s16 = (int16_t)s->data[0x2A00 + k];
                                    if (s16) nzb++;
                                    eb += (unsigned long)(s16 < 0 ? -(long)s16 : (long)s16);
                                }
                                for (int k = 0; k < 6; k++) {
                                    uint16_t a = (uint16_t)(pmad0 + k);
                                    uint16_t pv = s->prog[c54x_prog_xlate(s, a)];
                                    uint16_t dv = s->data[a];
                                    if (pv && pv != 0xF4E4) nzp++;
                                    if (dv) nzd++;
                                    po  += snprintf(pb + po,  sizeof(pb) - po,  " %04x", pv);
                                    dof += snprintf(db + dof, sizeof(db) - dof, " %04x", dv);
                                }
                                fprintf(stderr, "[c54x] FIRS-COEF #%u PC=0x%04x pmad=0x%04x "
                                        "OVLY=%d | AR2=0x%04x(*=0x%04x) AR3=0x%04x AR5=0x%04x"
                                        " | prog[]=%s (utiles=%d) | data[]=%s (utiles=%d)"
                                        " | SOURCE data[0x2CB9..]=%s (utiles=%d)"
                                        " | BURST 0x2a00 : %d/304 non nuls, energie=%lu,"
                                        " *AR2=0x%04x *AR3=0x%04x"
                                        " | CORR A(0x2c56) distinct=%d/50 [%d..%d]"
                                        " B(0x2c88) distinct=%d/50 [%d..%d]"
                                        " | VERDICT=%s insn=%u\n",
                                        _fcn, s->pc, pmad0, !!(s->pmst & PMST_OVLY),
                                        s->ar[2], s->data[s->ar[2]], s->ar[3], s->ar[5],
                                        pb, nzp, db, nzd, sb2, nzs,
                                        nzb, eb, s->data[s->ar[2]], s->data[s->ar[3]],
                                        dstA, (int)mnA, (int)mxA, dstB, (int)mnB, (int)mxB,
                                        nzd ? "SCRATCH-PAD (etendre OVLY a 0x60)"
                                            : (nzp ? "PROGRAMME (MVDP a depose)"
                                                   : "TABLE INTROUVABLE dans les deux espaces"),
                                        s->insn_count);
                            }
                        }
                    }
                    /* FIRS sits on the SCH soft-bit production path:
                     *   0x8492 rpt #5 ; 0x8493 firs 0x0061 ; 0x8497 sth *AR6+,B
                     * Semantics per SPRU172C:
                     *     B = B + A(32-16) x Pmem[pmad] ; A = (Xmem+Ymem)<<16
                     * Under RPT, pmad auto-increments at every repetition. */
                    int xmod = (op >> 6) & 0x03;
                    int xar  = ((op >> 4) & 0x03) + 2;
                    int ymod = (op >> 2) & 0x03;
                    int yar  = ( op       & 0x03) + 2;

                    {
                        /* pmad auto-increment: a second `rpt ; firs` at the SAME
                         * site must restart from pmad instead of continuing at
                         * _fpm+1. The SB path has two such sites, 0x8478 and
                         * 0x8493, 1704 iterations per burst. */
                        static uint16_t _fpm = 0;
                        if (!s->rpt_active || s->rpt_fresh) { _fpm = pmad0; s->rpt_fresh = false; }
                        else                                { _fpm++; }

                        uint16_t xv = data_read(s, s->ar[xar]);
                        uint16_t yv = data_read(s, s->ar[yar]);
                        int16_t  coef = (int16_t)prog_fetch(s, _fpm);

                        int64_t prod = (int64_t)(int16_t)((s->a >> 16) & 0xFFFF)
                                     * (int64_t)coef;
                        if (s->st1 & ST1_FRCT) prod <<= 1;
                        s->b = sext40(s->b + prod);
                        s->a = sext40(((int64_t)(int16_t)xv
                                     + (int64_t)(int16_t)yv) << 16);
                        {   static unsigned _fn2 = 0;
                            if (_fn2 < 16) {
                                _fn2++;
                                fprintf(stderr, "[c54x] FIRS #%u PC=0x%04x pmad=0x%04x "
                                        "coef(prog)=0x%04x data[pmad]=0x%04x OVLY=%d "
                                        "Xar=AR%d@0x%04x=0x%04x "
                                        "Yar=AR%d@0x%04x=0x%04x -> A=0x%010llx "
                                        "B=0x%010llx insn=%u\n",
                                        _fn2, s->pc, _fpm, (unsigned)(uint16_t)coef,
                                        (unsigned)s->data[_fpm],
                                        !!(s->pmst & PMST_OVLY),
                                        xar, s->ar[xar], xv, yar, s->ar[yar], yv,
                                        (unsigned long long)(s->a & 0xFFFFFFFFFFULL),
                                        (unsigned long long)(s->b & 0xFFFFFFFFFFULL),
                                        s->insn_count);
                            }
                        }
                    }
                    /* ⚠️ On the C54x FIRS advances Xmem and Ymem in every case,
                     * so the post-modifications are unconditional. Leaving them
                     * out froze AR2/AR3 at 0x2a8e/0x2a9a at s->pc == 0x8478 over
                     * six executions; the stage's RPTB loop (BRC=0x8d, 142
                     * iterations) then carried AR3 out of the burst buffer up to
                     * 0x2ceb, where it overwrote the correlation reference.
                     * Hardware watchpoint, six triggers in perfect alternation:
                     *   0x7c50 : 0x223c -> 0x0200  (installs the reference)
                     *   0x847a : 0x0200 -> 0x223c  (overwrites it, through AR3)
                     * hence a frozen correlator buffer A, a meaningless argmax and
                     * an SCH that never decodes. */
                    c54x_par_postmod(s, xar, xmod);
                    c54x_par_postmod(s, yar, ymod);
                    return consumed + s->lk_used;
                }
                if (_e0_n < 40) {
                    _e0_n++;
                    C54_LOG("E0-FAM non implemente op=0x%04x (%s) PC=0x%04x "
                            "consumed=%d insn=%u", op,
                            hi8 == 0xE0 ? "FIRS" : hi8 == 0xE1 ? "LMS" :
                            hi8 == 0xE2 ? "SQDST" : "ABDST",
                            s->pc, consumed, s->insn_count);
                }
                return consumed + s->lk_used;
            }
        }
        if ((op & 0xFE00) == 0xEA00) {
            /* EAxx: LD #k9, DP — Load Data Page pointer (1-word).
             * Per tic54x-opc.c: ld 0xEA00 mask 0xFE00, 1 word. */
            uint16_t k9 = op & 0x01FF;
            uint16_t old_dp = s->st0 & ST0_DP_MASK;
            s->st0 = (s->st0 & ~ST0_DP_MASK) | k9;
            g_last_ldp_pc = s->pc; g_last_ldp_val = k9; g_last_ldp_kind = 2;
            {
                static uint64_t dpc;
                dpc++;
                if (dpc <= 80 || (dpc % 5000) == 0 || k9 == 0x83) {
                    C54_LOG("DP-SET EAxx #%llu PC=0x%04x DP 0x%03x → 0x%03x %s",
                            (unsigned long long)dpc, s->pc,
                            old_dp, k9,
                            k9 == 0x83 ? "*** 0x83 (CALAD-zone base 0x4180) ***" : "");
                }
            }
            return consumed + s->lk_used;
        }
        if (hi8 == 0xEC) {
            /* ECxx: RPT #k8u — repeat next instruction k8u+1 times.
             * Per tic54x-opc.c: rpt 0xEC00 mask 0xFF00, single word.
             * Must advance PC past RPT now and return 0 so the dispatcher
             * re-executes the NEXT instruction (not RPT itself). */
            s->rpt_count = op & 0xFF;
            s->rpt_active = true; s->rpt_fresh = true;
            s->pc += 1;
            return 0;
        }
        if (hi8 == 0xE5) {
            /* E5xx: MVDD Xmem, Ymem  (per tic54x-opc.c, NOT MVMM)
             * 1-word, 2-cycle dual-operand data-to-data move:
             *   *Ymem = *Xmem
             * Per tic54x.h:
             *   XMEM = (op & 0xF0) >> 4
             *   YMEM = op & 0x0F
             *   XMOD/YMOD = (nibble & 0xC) >> 2  (0=*AR,1=*AR-,2=*AR+,3=*AR+0%)
             *   XARX/YARX = (nibble & 0x3) + 2   (AR2..AR5 only) */
            uint8_t xnib = (op >> 4) & 0xF;
            uint8_t ynib = op & 0xF;
            int xar = (xnib & 0x3) + 2;
            int yar = (ynib & 0x3) + 2;
            int xmod = (xnib & 0xC) >> 2;
            int ymod = (ynib & 0xC) >> 2;
            uint16_t xa = s->ar[xar];
            uint16_t ya = s->ar[yar];
            uint16_t v = data_read(s, xa);
            data_write(s, ya, v);
            /* Post-modify both ARs per their mod field */
            switch (xmod) {
                case 0: break;                        /* *AR     */
                case 1: s->ar[xar] = xa - 1; break;   /* *AR-    */
                case 2: s->ar[xar] = xa + 1; break;   /* *AR+    */
                case 3: s->ar[xar] = c54x_circ_ref(xa, +(int16_t)s->ar[0], s->bk); break; /* *AR+0% circular, modulo BK */
            }
            switch (ymod) {
                case 0: break;
                case 1: s->ar[yar] = ya - 1; break;
                case 2: s->ar[yar] = ya + 1; break;
                case 3: s->ar[yar] = c54x_circ_ref(ya, +(int16_t)s->ar[0], s->bk); break; /* *AR+0% circular, modulo BK */
            }
            return consumed + s->lk_used;
        }
        /* ================================================================
         *     ST src,Ymem || LD Xmem,T      encoding 111001S0 XXXXYYYY
         *     = 0xE400 mask 0xFD00 (so 0xE4xx AND 0xE6xx, with b8 = 0)
         *
         * ⚠️ Decoding 0xE4 as a 2-word "BITF Smem,#lk" swallows one word per
         * site and desynchronises the instruction stream. The real BITF is
         * 0x6100/0xFF00, decoded elsewhere in this file.
         *
         * ⚠️ Do NOT widen the mask to 0xFC00: 0xE5xx = MVDD and 0xE7xx = MVMM
         * are different instructions (b8 discriminates) and are really used --
         * 79 MVDD in PROM0, including the correlator body in PDROM.
         *
         * Semantics (SPRU172C p.4-178, syntax 2):
         *     Ymem = (src << ASM) >> 16   ;   T = Xmem
         * S = b9 (0xE4 -> A, 0xE6 -> B). ONE word.
         * ================================================================ */
        if ((op & 0xFD00) == 0xE400) {
            int s_acc = (op >> 9) & 1;
            int xmod  = (op >> 6) & 3;
            int xar   = ((op >> 4) & 3) + 2;
            int ymod  = (op >> 2) & 3;
            int yar   = ( op       & 3) + 2;
            uint16_t yaddr = s->ar[yar];
            uint16_t xval  = data_read(s, s->ar[xar]);
            int64_t  sv    = s_acc ? s->b : s->a;
            int      ash   = asm_shift(s);
            int64_t  sh    = (ash >= 0) ? (sv << ash) : (sv >> (-ash));

            data_write(s, yaddr, (uint16_t)((sh >> 16) & 0xFFFF));
            s->t = xval;                      /* LD Xmem, T */
            c54x_par_postmod(s, xar, xmod);
            c54x_par_postmod(s, yar, ymod);
            return consumed + s->lk_used;
        }
        if (hi8 == 0xE7) {
            /* E7xx: MVMM mmrx, mmry  (per tic54x-opc.c)
             * 1-word, 2-cycle, MMR-to-MMR move using a constrained set
             * (MMRX/MMRY operand types). */
            int src = (op >> 4) & 0xF;
            int dst = op & 0xF;
            uint16_t val;
            if (src <= 7) val = s->ar[src];
            else if (src == 8) val = s->sp;
            else val = data_read(s, src + 0x10);
            if (dst <= 7) s->ar[dst] = val;
            else if (dst == 8) { sp_abs_track(s, val, 2); s->sp = val; }  /* MVMM SP-dest */
            else data_write(s, dst + 0x10, val);
            return consumed + s->lk_used;
        }
        if (hi8 == 0xE8 || hi8 == 0xE9) {
            /* E8xx/E9xx: LD #k8u, dst — Load 8-bit unsigned immediate (1-word).
             * Per tic54x-opc.c: ld 0xE800 mask 0xFE00.
             * bit 8 = dst (0=A, 1=B), bits 7:0 = k8u.
             * ⚠️ Decoding this as CC (a 2-word conditional call) overflows the
             * stack by pushing return addresses in a loop. */
            int dst = (op >> 8) & 1;
            uint8_t k = op & 0xFF;
            /* [2026-09-20] K is an 8-bit UNSIGNED short immediate whatever SXM says:
             * SPRU172C example LD #248, B gives B = 00 0000 00F8. */
            int64_t v = (int64_t)k;
            /* Per SPRU172C, LD #k8 loads the immediate into the LOW bits
             * (sext40(v)), not v<<16. [2026-07-23] With the <<16, 0x39 landed in
             * bits 16-23: in the mask-ROM terminal, 0xb408 LD #0x39 followed by
             * 0xb409 ADD #0x4387 gave A=0x394387, so STLM wrote AR7=0x4387 (the
             * IDLE slot, data[0x4387]=0xab38) instead of 0x0039+0x4387=0x43C0 (the
             * go-live slot, data[0x43c0]=0xa4c7), and the terminal BACC at 0xb40f
             * jumped into idle. CALYPSO_LDK8_SHIFT16=1 restores the old form. */
            static int _ldk8sh = -1;
            if (_ldk8sh < 0) _ldk8sh = calypso_gate("CALYPSO_LDK8_SHIFT16", 0);
            int64_t _ldv = _ldk8sh ? (v << 16) : v;
            if (dst) s->b = sext40(_ldv);
            else     s->a = sext40(_ldv);
            return consumed + s->lk_used;
        }
        if (hi8 == 0xE1) {
            /* E1xx: single-word acc ops — NEG, ABS, CMPL, SAT, EXP, etc. */
            uint8_t sub = op & 0xFF;
            switch (sub) {
            case 0xE0: s->a = ~s->a; s->a = sext40(s->a); break;  /* CMPL A */
            case 0xE1: s->b = ~s->b; s->b = sext40(s->b); break;  /* CMPL B */
            case 0xE2: s->a = -s->a; s->a = sext40(s->a); break;  /* NEG A */
            case 0xE3: s->b = -s->b; s->b = sext40(s->b); break;  /* NEG B */
            case 0xE4: /* SAT A */ if (s->st0 & ST0_OVA) s->a = (s->a < 0) ? (int64_t)0xFF80000000LL : 0x7FFFFFFFLL; break;
            case 0xE5: /* SAT B */ if (s->st0 & ST0_OVB) s->b = (s->b < 0) ? (int64_t)0xFF80000000LL : 0x7FFFFFFFLL; break;
            case 0xE8: /* ABS A */ s->a = (s->a < 0) ? -s->a : s->a; s->a = sext40(s->a); break;
            case 0xE9: /* ABS B */ s->b = (s->b < 0) ? -s->b : s->b; s->b = sext40(s->b); break;
            case 0xEA: /* ROR A */ { uint16_t c = s->st0 & ST0_C ? 1 : 0; if (s->a & 1) s->st0 |= ST0_C; else s->st0 &= ~ST0_C; s->a = (s->a >> 1) | ((int64_t)c << 39); s->a = sext40(s->a); } break;
            case 0xEB: /* ROL A */ { uint16_t c = s->st0 & ST0_C ? 1 : 0; if (s->a & ((int64_t)1<<39)) s->st0 |= ST0_C; else s->st0 &= ~ST0_C; s->a = (s->a << 1) | c; s->a = sext40(s->a); } break;
            default:
                /* EXP A/B etc — return 0 for now */
                break;
            }
            return consumed + s->lk_used;
        }
        if (hi8 == 0xEF) {
            /* EFxx: RPTZ dst, #lk — Zero accumulator and repeat (2 words)
             * Per SPRU172C: dst = 0; RPT #lk
             * Encoding: 1110 1111 xxxx xxxx + lk_word */
            op2 = prog_fetch(s, s->pc + 1);
            consumed = 2;
            int rptz_dst = (op >> 0) & 1;
            if (rptz_dst) s->b = 0; else s->a = 0;
            s->rpt_count = op2;
            s->rpt_active = true; s->rpt_fresh = true;
            s->pc += 2;
            return 0;
        }
        if (hi8 == 0xEB) {
            /* EBxx: RPTB[D] pmad — Block repeat (2 words)
             * Per SPRU172C: REA = pmad, RSA = PC+2, BRAF=1 */
            op2 = prog_fetch(s, s->pc + 1);
            consumed = 2;
            s->rea = op2;
            s->rsa = (uint16_t)(s->pc + 2);
            s->rptb_active = true;
            s->st1 |= ST1_BRAF;
            return consumed + s->lk_used;
        }
        if (hi8 == 0xE6) {
            /* E6xx: SFTA/SFTL acc, #shift (single-word immediate shift) */
            int shift = op & 0x1F;
            if (shift & 0x10) shift |= ~0x1F;  /* sign extend 5-bit */
            int dst = (op >> 5) & 1;
            int logical = (op >> 6) & 1;
            int64_t *acc = dst ? &s->b : &s->a;
            if (logical) {
                uint64_t u = (uint64_t)(*acc) & 0xFFFFFFFFFFULL;
                if (shift >= 0) *acc = sext40((int64_t)(u << shift));
                else            *acc = sext40((int64_t)(u >> (-shift)));
            } else {
                if (shift >= 0) *acc = sext40(*acc << shift);
                else            *acc = sext40(*acc >> (-shift));
            }
            return consumed + s->lk_used;
        }
        if (hi8 == 0xEE) {
            /* FRAME #k8 — stack-frame pointer adjust: SP = SP + sign_ext(k8).
             * Per tic54x-opc.c { "frame", 1,1,1, 0xEE00, 0xFF00, {OP_k8} }: ONE
             * word. FRAME -N allocates the frame (SP goes down), FRAME +N frees it.
             *
             * ⚠️ Decoding 0xEExx as a 2-word "BCD pmad,cond" is wrong on both
             * counts: the length desynchronises everything downstream, and the
             * semantics are SP += k8, not a branch (bc is 0xF8, cc 0xF9, bcd 0xFA).
             * PROM0 has 124 0xEExx sites, including the boot path, where
             * FRAME #-1 / FRAME #+1 pairs are prologue and epilogue. With SP never
             * adjusted, pops outnumber pushes, POPM ST0 at 0x94f3 picks up the
             * orphan 0x80fd, DP becomes 0x0fd, the dispatcher LUT reads garbage and
             * the self-CALA at 0x70c3 writes 0x70c4 into d_fb_det/a_pm, poisoning
             * rxlev and TOA. */
            int8_t k = (int8_t)(op & 0xFF);
            s->sp = (uint16_t)(s->sp + k);
            return consumed + s->lk_used;
        }
        if ((op & 0xFFE0) == 0xED00) {
            /* ED00-ED1F: LD #k5, ASM — load 5-bit immediate into ASM field of ST1.
             * Per tic54x-opc.c: ld 0xED00 mask 0xFFE0, 1 word.
             * NOT BCD (which is 0xFA00 mask 0xFF00). */
            uint8_t k5 = op & 0x1F;
            s->st1 = (s->st1 & ~ST1_ASM_MASK) | k5;
            return consumed + s->lk_used;
        }
        if (hi8 == 0xED) {
            /* EDxx (not ED00-ED1F): BCD pmad, cond (conditional branch delayed, 2 words) */
            op2 = prog_fetch(s, s->pc + 1);
            consumed = 2;
            uint8_t cond = op & 0xFF;
            bool take = false;
            if (cond == 0x00) take = true;            /* UNC */
            else if (cond == 0x08) take = (s->b < 0);
            else if (cond == 0x02) take = (s->a != 0);
            else if (cond == 0x0A) take = (s->b != 0);
            else if (cond == 0x03) take = (s->a == 0);
            else if (cond == 0x0B) take = (s->b == 0);
            else if (cond == 0x04) take = (s->a > 0);
            else if (cond == 0x0C) take = (s->b > 0);
            else if (cond == 0x40) take = (s->st0 & ST0_TC) != 0;
            else if (cond == 0x41) take = !(s->st0 & ST0_TC);
            else take = true;
            if (take) { s->pc = op2; return 0; }
            return consumed + s->lk_used;
        }
        goto unimpl;

    case 0x6: case 0x7:
        /* 7Exx: READA Smem — read prog[A_low] → data[Smem]
         * Per tic54x-opc.c: reada 0x7E00 mask 0xFF00 (1 word).
         * Per SPRU131G : program address = (XPC[6:0] | A[15:0]). A.high is
         * NOT used as XPC source — XPC reg is. prog_read already implements
         * this via c54x_prog_xlate for addr ≥ 0x8000.
         * Under RPT, the prog address auto-increments each iteration;
         * accumulator A is preserved (mirrored through mvpd_src).
         *
         * ⚠️ Do not make A.high override XPC as a 23-bit address: SPRU131G says
         * otherwise and it moves no symptom. */
        if (hi8 == 0x7E) {
            /* READA-ITER: why the table loader's second loop makes TWO copies
             * instead of three. [2026-08-04] The loader at 0xb4b6 runs two loops:
             *   0xb4bc  rpt #0x4d ; reada *AR1+  -> 77 copies, 0x4387..0x43d3,
             *                                       exact (77 = 0x4d + 1)
             *   0xb4c4  rpt #0x02 ; reada *AR1+  -> 2 copies, 0x43d5 and 0x43d6,
             *                                       one short of the expected 3
             * RPT therefore counts correctly in one case and one short in the
             * other, so RPT itself is not the culprit. Over 76 passes,
             * data[0x43d7] -- entry 2 of the task dispatch table -- is never
             * initialised.
             * The probe prints AR1 before resolve_smem, the resolved address and
             * the repeat state: three lines with the third outside 0x43d7 means
             * the `*AR1+` pointer slips, two lines mean the third iteration never
             * happens and the repeat/instruction interaction is at fault.
             * Limited to the second loop's PC (0xb4c5), 60 lines. */
            uint16_t _ar1_before = s->ar[1];
            addr = resolve_smem(s, op, &ind);
            if (s->pc == 0xb4c5) {
                static unsigned _ri;
                if (_ri < 60) {
                    _ri++;
                    fprintf(stderr, "[c54x] READA-ITER #%u AR1 0x%04x -> 0x%04x "
                            "addr=0x%04x A_low=0x%04x mvpd_src=0x%04x "
                            "rpt_active=%d rpt_count=%u rpt_fresh=%d insn=%u\n",
                            _ri, _ar1_before, s->ar[1], addr,
                            (unsigned)(s->a & 0xFFFF), s->mvpd_src,
                            s->rpt_active, s->rpt_count, s->rpt_fresh,
                            s->insn_count);
                }
            }
            /* ⚠️ Under RPT the first iteration must start from A_low, the source
             * base, not from the mvpd_src left by a previous READA. Otherwise
             * `RPT #N ; READA *ARx+` copies from the wrong ROM region and fills
             * the dispatch table at 0x4380+ with garbage (0xf074), so the BACC
             * lands in the LUT instead of the real FB handler at 0xab38. */
            uint16_t psrc;
            if (!s->rpt_active || s->rpt_fresh) {
                psrc = (uint16_t)(s->a & 0xFFFF);
                s->rpt_fresh = false;
            } else {
                psrc = s->mvpd_src;
            }
            uint16_t v = prog_read(s, psrc);
            data_write(s, addr, v);
            s->mvpd_src = psrc + 1;
            { /* Cap of 200: the first loop (rpt #0x4d, 77 copies) consumes a cap
                 * of 20 entirely and hides the second one (rpt #0x02 into the
                 * dispatch table at 0x43d5). */
                static int reada_log = 0; if (reada_log++ < 200)
                C54_LOG("READA: prog[0x%04x]=0x%04x → data[0x%04x] PC=0x%04x rpt=%d insn=%u",
                        psrc, v, addr, s->pc, s->rpt_count, s->insn_count); }
            return consumed + s->lk_used;
        }
        /* 7Fxx: WRITA Smem — write data[Smem] → prog[A_low] (mirror of READA) */
        if (hi8 == 0x7F) {
            addr = resolve_smem(s, op, &ind);
            uint16_t pdst = s->rpt_active ? s->mvpd_src : (uint16_t)(s->a & 0xFFFF);
            prog_write(s, pdst, data_read(s, addr));
            s->mvpd_src = pdst + 1;
            return consumed + s->lk_used;
        }
        /* 6Dxx: MAR Smem — modify address register (side effects only) */
        if (hi8 == 0x6D) {
            addr = resolve_smem(s, op, &ind);
            /* MAR only modifies AR via addressing mode, no data access */
            return consumed + s->lk_used;
        }
        /* 76xx: ST #lk, Smem  (2 or 3 words) — store 16-bit literal to data
         * memory. Per binutils tic54x-opc.c {st, 2,2,2, 0x7600, 0xFF00,
         * {OP_lk, OP_Smem}} and tic54x-dis.c get_insn_size = words +
         * has_lkaddr (extra word when Smem mode in 0xC..0xF).
         *
         * Encoding (verified via tic54x-dis.c:192-204):
         *   word 0 = opcode (0x76xx)
         *   word 1 = lkaddr  (Smem extension, only if mode in 0xC..0xF)
         *   word N = opcode2 (the #lk value being stored, last extension)
         *
         * ⚠️ Do not decode this as a 1-word LDM MMR,dst: the real LDM is 0x48xx
         * mask 0xFE00, handled in the 0x4 group. With a 1-word decode, PC advances
         * by 1 instead of 2 or 3 and the literal executes as a stray opcode; a
         * stray 0x4F00 (DST B,Lmem with DP=0, i.e. MMR_IMR) zeroes the IMR for
         * good, masks INT3 and BRINT0, and parks the DSP in the RPTB at
         * e9ab..e9b6 waiting for a frame interrupt that is never served. */
        if (hi8 == 0x76) {
            static unsigned hit76_log;
            addr = resolve_smem(s, op, &ind);
            op2 = prog_fetch(s, s->pc + 1 + (s->lk_used ? 1 : 0));
            consumed = 2;
            if (hit76_log++ < 30) {
                if (calypso_debug_enabled("HIT-76")) fprintf(stderr,
                        "[c54x] HIT-76 PC=0x%04x op=0x%04x addr=0x%04x "
                        "lk=0x%04x lk_used=%d insn=%u\n",
                        s->pc, op, addr, op2, s->lk_used, s->insn_count);
            }
            data_write(s, addr, op2);
            return consumed + s->lk_used;
        }
        /* 77xx: STM #lk, MMR (2 words) */
        if (hi8 == 0x77) {
            uint8_t mmr = op & 0x7F;
            op2 = prog_fetch(s, s->pc + 1);
            consumed = 2;
            /* WATCH-ST1-WRITE: MMR 0x07 is ST1. Logs every STM #lk,ST1, including
             * the ones that leave INTM alone but redefine the whole ST1 word:
             * value written, bit 11 (INTM) and the delta against the current ST1.
             * First 200 entries for boot, then one in 100. */
            if (mmr == 0x07) {
                static unsigned st1w;
                st1w++;
                if (st1w <= 200 || (st1w % 100) == 0) {
                    int new_intm = !!(op2 & (1 << 11));
                    int cur_intm = !!(s->st1 & ST1_INTM);
                    if (calypso_debug_enabled("ST1-WR")) fprintf(stderr,
                            "[c54x] ST1-WR #%u STM #0x%04x,ST1 PC=0x%04x "
                            "cur=0x%04x->0x%04x INTM:%d->%d insn=%u XPC=%d\n",
                            st1w, op2, s->pc, s->st1, op2,
                            cur_intm, new_intm, s->insn_count, s->xpc);
                }
            }
            data_write(s, mmr, op2);
            return consumed + s->lk_used;
        }
        /* @BEQUILLE — 0x72 MVDM stays mis-decoded (generic STL fallthrough).
         *
         *   masks   : an upstream AR3 setup bug. 0x72xx is MVDM MMR,dmad and is
         *             really 2 words; decoded as a 1-word STL it under-consumes
         *             the operand word.
         *   remove  : once the 0xee38 deadlock below is fixed, i.e. once AR3 is
         *             set up correctly before that point.
         *
         * [2026-06-02] Measured both ways. With the mis-decode: at PC=0xf564
         * op=0x7215 = MVDM data[0x0014] -> AR5, in the heart of the FB dispatch
         * loop (0xf561..0xf588), the operand 0x0014 executes as an opcode, the
         * loop desynchronises every iteration, AR5 (the task handler pointer) is
         * never loaded, the FB task is never dispatched and d_fb_det is never
         * armed. With the ISA-correct 2-word MVDM: the dispatch does reach
         * PC=0xee38 (task_md=5 read on both pages), but AR3 there points OUTSIDE
         * the I/Q buffer (0x2b97 > 0x2b28), the correlation runs on zeros, the BSP
         * stops delivering and INT3 stops firing (3860 interrupts down to 4) --
         * a worse deadlock. The mis-decoded STL was compensating for the broken
         * AR3 setup. 0x73 (MVMD, the save direction) IS fixed below. */
        /* PORTR 0x74 / PORTW 0x75 are 3 words when Smem is absolute.
         * tic54x-opc.c: portr {2,2,2,0x7400,0xFF00,{OP_PA,OP_Smem}},
         * portw {2,2,2,0x7500,0xFF00,{OP_Smem,OP_PA}}. Base 2 words (opcode + PA),
         * plus one when Smem uses absolute/long addressing (binutils
         * get_insn_size = words + has_lkaddr). resolve_smem reads the absolute
         * Smem address at pc+1 and sets lk_used; the PA follows at pc+1+lk_used,
         * the same convention as CMPM/BITF below.
         * ⚠️ The generic `(op & 0xF800)==0x7000` catch-all below swallows them as
         * a 1-word STL, loses the PA word and shifts alignment by one word.
         * Measured against binutils-2.21.1: PROM0 0xb416
         * `portw *(0x000e),0xf900` sized as 2 words makes its PA 0xf900 be read as
         * a phantom CC at 0xb418, which enters the epilogue at 0x76f8 bare (POPM
         * ST1 with no PSHM ST1), over-pops SP and collapses it, leaving
         * d_fb_det = 0. The firmware holds 128 `75f8` and 25 `74f8` sites.
         * The real I/O semantics are handled below; this fixes the LENGTH. */
        if ((op & 0xFF00) == 0x7500) {        /* PORTW Smem, PA */
            addr = resolve_smem(s, op, &ind); /* applies the AR post-modify, sets lk_used when absolute */
            uint16_t pa = prog_fetch(s, s->pc + 1 + (s->lk_used ? 1 : 0));
            /* RIF (calypso_rif.c): SPCR/SPCX/DXR are really written. SPCR is how
             * the firmware opens RINT_MASK or RDMA_MASK, i.e. how it chooses the
             * transfer mode of CAL207 §3.7.1. */
            if (calypso_rif_portw(s, pa, data_read(s, addr))) {
                consumed = 2;
                return consumed + s->lk_used;
            }
            {   /* DSP-side DMA window; see PORTR below */
                uint16_t wv = data_read(s, addr);
                if (calypso_rhea_dma_xio(true, pa, &wv, s->pc)) {
                    consumed = 2;
                    return consumed + s->lk_used;
                }
                if (calypso_xio_misc(true, pa, &wv, s->pc)) {
                    consumed = 2;
                    return consumed + s->lk_used;
                }
            }
            (void)pa; (void)addr;             /* any other port: not modelled */
            consumed = 2;                     /* opcode + PA */
            return consumed + s->lk_used;
        }
        if ((op & 0xFF00) == 0x7400) {        /* PORTR PA, Smem */
            addr = resolve_smem(s, op, &ind);
            uint16_t pa = prog_fetch(s, s->pc + 1 + (s->lk_used ? 1 : 0));
            /* PORTR-ANY (read-only, unconditional): counts every PORTR hit
             * whatever the PA, to tell "opcode never reached" from "reached but
             * PA is not 0xF430/0x0034". Capped at 30. */
            {
                static unsigned _pany = 0;
                if (_pany++ < 30)
                    fprintf(stderr, "[c54x] PORTR-ANY #%u PA=0x%04x addr=0x%04x PC=0x%04x insn=%u\n",
                            _pany, pa, addr, s->pc, s->insn_count);
            }
            /* RIF (calypso_rif.c, CAL207 §12) is the only target the firmware
             * polls here: [2026-08-03] all 30 PORTR-ANY samples had PA=0x0003,
             * i.e. SPCR. */
            {
                uint16_t rv;
                if (calypso_rif_portr(s, pa, &rv)) {
                    data_write(s, addr, rv);
                    consumed = 2;
                    return consumed + s->lk_used;
                }
                /* DMA window as seen from the DSP (XIO:FC00..FCFF, CAL207 §11.1).
                 * The ARM hands the RIF channels to the DSP (measured
                 * ALLOC_CONFIG=0x000C), so the transfer configuration goes through
                 * here. */
                rv = 0;
                if (calypso_rhea_dma_xio(false, pa, &rv, s->pc)) {
                    data_write(s, addr, rv);
                    consumed = 2;
                    return consumed + s->lk_used;
                }
                /* API Control (F900) and the DSP INTH (FA00); see calypso_xio.c */
                rv = 0;
                if (calypso_xio_misc(false, pa, &rv, s->pc)) {
                    data_write(s, addr, rv);
                    consumed = 2;
                    return consumed + s->lk_used;
                }
            }
            (void)pa; (void)addr;
            consumed = 2;
            return consumed + s->lk_used;
        }

        /* 0x73xx: MVMD MMR,dmad — MMR -> data[dmad], 2 words, the SAVE direction.
         * It reads the MMR (SP is 0x18, aliased at data 0x0018) and writes
         * data[dmad], leaving SP and the MMR unchanged.
         * [2026-07-23] Measured with the SP-CORRUPT watchpoint: 0xa4f8 op=0x7318 =
         * MVMD SP,0x3f6e saves SP, but the generic STL fallthrough LOADED SP from
         * data[0x3f6e], which holds garbage (SP=0xc905 and similar), derailing
         * after every POPD. Only 0x73 is fixed here; 0x72 (MVDM, the load
         * direction) keeps its documented crutch above. */
        if ((op & 0xFF00) == 0x7300) {
            int mmr = op & 0x7F;
            uint16_t dmad = prog_fetch(s, s->pc + 1);
            data_write(s, dmad, data_read(s, mmr));
            consumed = 2;
            return consumed + s->lk_used;
        }

        /* 0x70xx: MVKD dmad,Smem — data[dmad] -> data[Smem].
         * binutils tic54x-opc.c {mvkd,2,2,2,0x7000,0xFF00,{OP_dmad,OP_Smem}}.
         * Base 2 words (opcode + dmad), plus one when Smem is absolute/long
         * (mode 0xC..0xF, has_lkaddr). Operand order: dmad FIRST at pc+1, the
         * absolute-Smem lk SECOND at pc+2.
         *
         * ⚠️ The generic `(op & 0xF800)==0x7000` catch-all below decodes 0x70xx as
         * a 1-word STL (+lk) and under-consumes the dmad word. [2026-06-23] Traced
         * with the AR3-TRIP probe: at PROM0 0xb3cc (`70f8 4356 00e3`) the third
         * word 0x00e3 then executed as a stray `ADD *AR3(lk)`, and 0xb3d1=0x00db as
         * `ADD *AR3+0%,A`, adding AR0 (about SP) to AR3 every turn until AR3 swept
         * memory and overwrote data[0x0c36], the task pointer, giving CALA 0 and
         * the boot-stub return loop. The ADD decode itself was faithful; the bug
         * was the LENGTH of the MVKD ahead of it. */
        if ((op & 0xFF00) == 0x7000) {
            uint16_t dmad = prog_fetch(s, s->pc + 1);
            int mode = (op & 0x80) ? ((op >> 3) & 0x0F) : -1;
            uint16_t smem_addr;
            s->lk_used = false;
            if (mode >= 0xC) {                 /* Smem absolu/long : lk @pc+2 */
                uint16_t lk = prog_fetch(s, s->pc + 2);
                int nar = op & 0x07;
                if (mode == 0xC) {                      /* *ARx(lk), no modify */
                    smem_addr = (uint16_t)(s->ar[nar] + lk);
                } else if (mode == 0xD || mode == 0xE) { /* *+ARx(lk)[%] premod */
                    s->ar[nar] = (uint16_t)(s->ar[nar] + lk);
                    smem_addr = s->ar[nar];
                } else {                                 /* 0xF: *(lk) absolute */
                    smem_addr = lk;
                }
                s->st0 = (s->st0 & ~ST0_ARP_MASK) | (nar << ST0_ARP_SHIFT);
                s->lk_used = true;
            } else {                           /* direct or non-absolute indirect */
                smem_addr = resolve_smem(s, op, &ind);  /* plus the AR post-modify */
            }
            data_write(s, smem_addr, data_read(s, dmad));
            consumed = 2;
            return consumed + (s->lk_used ? 1 : 0);
        }

        /* 0x71xx: MVDK Smem,dmad — data[Smem] -> data[dmad], the mirror of MVKD.
         * binutils tic54x-opc.c {mvdk,2,2,2,0x7100,0xFF00,{OP_Smem,OP_dmad}}.
         * Same encoding as MVKD 0x70 (Smem in the opcode low byte, dmad at pc+1,
         * the absolute-Smem lk at pc+2); only the direction differs.
         *
         * ⚠️ The `(op & 0xF800)==0x7000` catch-all decodes these as a 1-word STL and
         * under-consumes the dmad word. [2026-06-24] At PROM0 0xb3db..0xb3e3 three
         * 3-word MVDK (`71f8 4356 00e3`, `71f8 4357 00db`, `71f8 4355 00d3`) restore
         * what the three symmetric MVKD at 0xb3cc saved before three CALLs; with the
         * short decode the operands 00e3/00db/00d3 execute as stray ADDs (0x00db is
         * `ADD *AR3+0%,A`) on every frame, corrupting AR3 and A and losing the
         * restore. 0x71 is data-to-data, not MMR, so it is independent of the 0x72
         * crutch above. */
        if ((op & 0xFF00) == 0x7100) {
            uint16_t dmad = prog_fetch(s, s->pc + 1);
            int mode = (op & 0x80) ? ((op >> 3) & 0x0F) : -1;
            uint16_t smem_addr;
            s->lk_used = false;
            if (mode >= 0xC) {                 /* Smem absolu/long : lk @pc+2 */
                uint16_t lk = prog_fetch(s, s->pc + 2);
                int nar = op & 0x07;
                if (mode == 0xC) {                      /* *ARx(lk), no modify */
                    smem_addr = (uint16_t)(s->ar[nar] + lk);
                } else if (mode == 0xD || mode == 0xE) { /* *+ARx(lk)[%] premod */
                    s->ar[nar] = (uint16_t)(s->ar[nar] + lk);
                    smem_addr = s->ar[nar];
                } else {                                 /* 0xF : *(lk) absolu */
                    smem_addr = lk;
                }
                s->st0 = (s->st0 & ~ST0_ARP_MASK) | (nar << ST0_ARP_SHIFT);
                s->lk_used = true;
            } else {                           /* direct ou indirect non-abs */
                smem_addr = resolve_smem(s, op, &ind);  /* +post-modify AR */
            }
            data_write(s, dmad, data_read(s, smem_addr));   /* MVDK : dmad <- Smem */
            consumed = 2;
            return consumed + (s->lk_used ? 1 : 0);
        }

        /* 0x72 MVDM dmad,MMR (MMR <- data[dmad]) and 0x73 MVMD MMR,dmad
         * (data[dmad] <- MMR): 2 words (opcode + dmad), MMR in the low byte, mapped
         * to data 0x00-0x1f which data_read/data_write route to the registers.
         * These unblock the go-live subroutine at 0xaad5 (7211 434f MVDM
         * data[0x434f] -> AR1; 7210 434e; 7310 434e MVMD AR0 -> data[0x434e]): with
         * the 1-word STL catch-all, A stays 0, the BC at 0xa4cd (AEQ, A==0) never
         * releases and RSBX INTM at 0xa51b is never reached. [2026-07-23] With the
         * fix, D_TASK_MD reads go from 0 to 1859 and the DSP reaches the frame
         * dispatcher.
         * ⚠️ This fix only holds once the upstream AR3 setup is right, i.e. with
         * MVKD 0x70 and MVDK 0x71 fixed; applied before them it deadlocks at
         * 0xee38. */
        if ((op & 0xFF00) == 0x7200) {       /* MVDM dmad, MMR */
            uint16_t dmad = prog_fetch(s, s->pc + 1);
            uint16_t mmr  = op & 0x00FF;
            data_write(s, mmr, data_read(s, dmad));
            consumed = 2;
            return consumed;
        }
        if ((op & 0xFF00) == 0x7300) {       /* MVMD MMR, dmad */
            uint16_t dmad = prog_fetch(s, s->pc + 1);
            uint16_t mmr  = op & 0x00FF;
            data_write(s, dmad, data_read(s, mmr));
            consumed = 2;
            return consumed;
        }

        /* LD / ST operations */
        if ((op & 0xF800) == 0x7000) {
            /* 70xx: STL src, Smem */
            int src_acc = (op >> 9) & 1;
            addr = resolve_smem(s, op, &ind);
            int64_t acc = src_acc ? s->b : s->a;
            data_write(s, addr, (uint16_t)(acc & 0xFFFF));
            return consumed + s->lk_used;
        }
        /* FIX_MACP_MACD — macp Smem,pmad,src (0x7800/0xFE00) and
         * macd Smem,pmad,src (0x7A00/0xFE00), bit 8 = src, TWO words plus the Smem
         * lk. SPRU172C: src = src + Smem x Pmem(pmad); T = Smem; MACD also copies
         * Smem into Smem+1. Under RPT, pmad increments each turn, tracked like
         * READA/MVPD through rpt_fresh and mvpd_src.
         * ⚠️ They otherwise fall into the 1-word "STH" block below and the pmad
         * word executes as an instruction: [2026-09-17] on the SB trace, 0x7e2a
         * `7892 7a1c` under RPT had `7a1c` taken for a MACD. */
        if ((op & 0xFC00) == 0x7800) {
            addr = resolve_smem(s, op, &ind);
            uint16_t pmad = prog_fetch(s, s->pc + 1 + (s->lk_used ? 1 : 0));
            consumed = 2;
            uint16_t psrc;
            if (!s->rpt_active || s->rpt_fresh) { psrc = pmad; s->rpt_fresh = false; }
            else psrc = s->mvpd_src;
            uint16_t sval = data_read(s, addr);
            int64_t prod = (int64_t)(int16_t)sval * (int64_t)(int16_t)prog_fetch(s, psrc);
            if (s->st1 & ST1_FRCT) prod <<= 1;
            if (op & 0x0100) s->b = sext40(s->b + prod);
            else             s->a = sext40(s->a + prod);
            s->t = sval;
            if (op & 0x0200) data_write(s, (uint16_t)(addr + 1), sval);   /* MACD */
            s->mvpd_src = (uint16_t)(psrc + 1);
            return consumed + s->lk_used;
        }
        if ((op & 0xF800) == 0x7800) {
            /* 78xx-7Fxx: STH src, Smem
             * Note: BANZ (0x78xx per doc) shares this range but is handled
             * via F84x (BANZ with condition) in the F8xx group. */
            int src_acc = (op >> 9) & 1;
            addr = resolve_smem(s, op, &ind);
            int64_t acc = src_acc ? s->b : s->a;
            data_write(s, addr, (uint16_t)((acc >> 16) & 0xFFFF));
            return consumed + s->lk_used;
        }
        /* 0x6000-0x60FF: CMPM Smem, lk  (compare memory with long immediate)
         * Per tic54x-opc.c: { "cmpm", 2,2,2, 0x6000, 0xFF00 }
         * Sets TC = (data[Smem] == lk).
         *
         * The DSP bootloader at PROM0 0xb41c / 0xb424 polls
         *   CMPM *(0x0fff), 4   then  CMPM *(0x0fff), 2
         * waiting for the ARM-side BL_CMD_STATUS write. ⚠️ Folded into the generic
         * 0x6000-0x67FF "LD" path it sets the accumulator and never updates TC, so
         * the following BC NTC always branches back and loops forever. */
        if ((op & 0xFF00) == 0x6000) {
            addr = resolve_smem(s, op, &ind);
            uint16_t cmp_val = prog_fetch(s, s->pc + 1 + (s->lk_used ? 1 : 0));
            uint16_t mem_val = data_read(s, addr);
            if (mem_val == cmp_val) s->st0 |= ST0_TC;
            else                    s->st0 &= ~ST0_TC;
            consumed = 2;  /* opcode + cmp_val (smem extra lk added via lk_used) */
            return consumed + s->lk_used;
        }
        /* 0x6100-0x61FF: BITF Smem, lk — bit-field test, TC = (Smem & lk)!=0 */
        if ((op & 0xFF00) == 0x6100) {
            addr = resolve_smem(s, op, &ind);
            uint16_t mask = prog_fetch(s, s->pc + 1 + (s->lk_used ? 1 : 0));
            uint16_t mem_val = data_read(s, addr);
            bool tc_before = (s->st0 & ST0_TC) != 0;
            if (mem_val & mask) s->st0 |= ST0_TC;
            else                s->st0 &= ~ST0_TC;
            bool tc_after = (s->st0 & ST0_TC) != 0;
            consumed = 2;
            /* FBWATCH: exact capture at the foreground poll sites 0xf7af/0xf7b7 --
             * resolved address, value read and TC -- to name the flag that is never
             * true. */
            if (g_fbwatch_on > 0 && (s->pc == 0xf7af || s->pc == 0xf7b7)
                && s->insn_count > 100000) {   /* after the first wire, at insn 32768 */
                static unsigned wbf = 0;
                if (wbf++ < 30)
                    fprintf(stderr, "[c54x] FBWATCH-BITF pc=0x%04x addr=0x%04x "
                            "mem=0x%04x mask=0x%04x -> TC=%d insn=%u\n",
                            s->pc, addr, mem_val, mask, tc_after, s->insn_count);
            }
            /* BITF instrumentation: does BITF really set TC? Many calls with
             * tc_after=1 rarely means the mask/mem_val pattern never yields TC=1,
             * so BC NTC always branches and `ST #1, d_task_d` at PROM 0x9ab1 is
             * never reached. First 200 hits, then one in 1000. */
            {
                static uint64_t bitf_total;
                static uint64_t bitf_tc_set;
                static uint64_t bitf_tc_clear;
                bitf_total++;
                if (tc_after) bitf_tc_set++;
                else          bitf_tc_clear++;
                if (bitf_total <= 200 || (bitf_total % 1000) == 0) {
                    if (calypso_debug_enabled("BITF-PROBE")) fprintf(stderr,
                            "[c54x] BITF-PROBE #%llu PC=0x%04x addr=0x%04x "
                            "mem=0x%04x mask=0x%04x tc_before=%d tc_after=%d "
                            "(total=%llu set=%llu clear=%llu)\n",
                            (unsigned long long)bitf_total, s->last_exec_pc,
                            addr, mem_val, mask, tc_before, tc_after,
                            (unsigned long long)bitf_total,
                            (unsigned long long)bitf_tc_set,
                            (unsigned long long)bitf_tc_clear);
                }
            }
            return consumed + s->lk_used;
        }
        /* FIX_MPY_MAC_LK — mpy Smem,#lk,dst (0x6200/0xFE00, bit 8 = dst) and
         * mac Smem,#lk,src[,dst] (0x6400/0xFC00, bit 9 = src, bit 8 = dst): TWO
         * words plus the Smem lk. SPRU172C: dst = Smem x lk; dst = src + Smem x lk;
         * T = Smem.
         * ⚠️ They otherwise fall into the 1-word "LD" block below and the #lk word
         * executes as an instruction: [2026-09-17] on the SB trace, 0x762c
         * `6283 36f6` and 0x7692 `6283 5a82` (cos 45). */
        if ((op & 0xF800) == 0x6000 && (op & 0x0600) != 0) {
            addr = resolve_smem(s, op, &ind);
            uint16_t lk = prog_fetch(s, s->pc + 1 + (s->lk_used ? 1 : 0));
            consumed = 2;
            uint16_t sval = data_read(s, addr);
            int64_t prod = (int64_t)(int16_t)sval * (int64_t)(int16_t)lk;
            if (s->st1 & ST1_FRCT) prod <<= 1;
            if ((op & 0xFE00) == 0x6200) {                    /* mpy */
                if (op & 0x0100) s->b = sext40(prod); else s->a = sext40(prod);
            } else {                                          /* mac 0x64..0x67 */
                int64_t base = (op & 0x0200) ? s->b : s->a;
                if (op & 0x0100) s->b = sext40(base + prod); else s->a = sext40(base + prod);
            }
            s->t = sval;
            return consumed + s->lk_used;
        }
        if ((op & 0xF800) == 0x6000) {
            /* 60xx-67xx: LD Smem, dst (other variants — fallback) */
            int dst_acc = (op >> 9) & 1;
            int shift = (op >> 8) & 1;
            addr = resolve_smem(s, op, &ind);
            uint16_t val = data_read(s, addr);
            int64_t v = (s->st1 & ST1_SXM) ? (int16_t)val : val;
            if (shift) v <<= 16;  /* LD Smem, 16, dst */
            if (dst_acc) s->b = sext40(v); else s->a = sext40(v);
            return consumed + s->lk_used;
        }
        /* 0x6800-0x6BFF + 0x6Cxx + 0x6Exx: companion to the 0x6F00 fix below.
         * Per binutils tic54x-opc.c (verified against insn_template struct):
         *   0x6800 ANDM  #lk, Smem      data[Smem] = data[Smem] & lk     (2-word)
         *   0x6900 ORM   #lk, Smem      data[Smem] = data[Smem] | lk     (2-word)
         *   0x6A00 XORM  #lku, Smem     data[Smem] = data[Smem] ^ lku    (2-word)
         *   0x6B00 ADDM  #lk, Smem      data[Smem] = data[Smem] + lk     (2-word)
         *   0x6C00 BANZ  pmad, Sind     if (ARx != 0) PC = pmad          (2-word)
         *   0x6E00 BANZD pmad, Sind     same as BANZ but with 2 delay slots
         *
         * ⚠️ Without these, the fallback at (op & 0xF800) == 0x6800 below decodes
         * them all as a 1-word LD Smem,T, PC drifts by one word and the lk or pmad
         * operand executes as a stray instruction. The ROM holds 1259
         * ANDM/ORM/XORM/ADDM sites and 304 BANZ/BANZD sites, 1563 in all. */
        if ((op & 0xFF00) == 0x6800) {
            /* ANDM #lk, Smem */
            addr = resolve_smem(s, op, &ind);
            uint16_t lk = prog_fetch(s, s->pc + 1 + (s->lk_used ? 1 : 0));
            uint16_t v = data_read(s, addr);
            data_write(s, addr, v & lk);
            consumed = 2;
            return consumed + s->lk_used;
        }
        if ((op & 0xFF00) == 0x6900) {
            /* ORM #lk, Smem */
            addr = resolve_smem(s, op, &ind);
            uint16_t lk = prog_fetch(s, s->pc + 1 + (s->lk_used ? 1 : 0));
            uint16_t v = data_read(s, addr);
            data_write(s, addr, v | lk);
            consumed = 2;
            return consumed + s->lk_used;
        }
        if ((op & 0xFF00) == 0x6A00) {
            /* XORM #lku, Smem */
            addr = resolve_smem(s, op, &ind);
            uint16_t lku = prog_fetch(s, s->pc + 1 + (s->lk_used ? 1 : 0));
            uint16_t v = data_read(s, addr);
            data_write(s, addr, v ^ lku);
            consumed = 2;
            return consumed + s->lk_used;
        }
        if ((op & 0xFF00) == 0x6B00) {
            /* ADDM #lk, Smem — add signed lk to memory (wrap mod 2^16) */
            addr = resolve_smem(s, op, &ind);
            int16_t lk = (int16_t)prog_fetch(s, s->pc + 1 + (s->lk_used ? 1 : 0));
            uint16_t v = data_read(s, addr);
            data_write(s, addr, (uint16_t)((int16_t)v + lk));
            consumed = 2;
            /* TODO: TC/OVM/SXM flag effects per SPRU172C (verify) */
            return consumed + s->lk_used;
        }
        if ((op & 0xFF00) == 0x6C00) {
            /* BANZ pmad, Sind — branch if ARx (selected by ARF in op[2:0])
             * is non-zero. Tests the PRE-modify value; resolve_smem applies the
             * post-mod whether or not the branch is taken.
             * ⚠️ Reading ARP from ST0 here takes the PREVIOUS instruction's nar and
             * tests the wrong AR; see the resolve_smem comment. */
            int nar = op & 0x07;
            uint16_t pre = s->ar[nar];
            resolve_smem(s, op, &ind);
            uint16_t pmad = prog_fetch(s, s->pc + 1 + (s->lk_used ? 1 : 0));
            consumed = 2;
            if (pre != 0) {
                s->pc = pmad;
                return 0;
            }
            return consumed + s->lk_used;
        }
        if ((op & 0xFF00) == 0x6E00) {
            /* BANZD pmad, Sind — delayed BANZ (2 slots after the 2-word op).
             * Same off-by-ARP fix as BANZ above. */
            int nar = op & 0x07;
            uint16_t pre = s->ar[nar];
            resolve_smem(s, op, &ind);
            uint16_t pmad = prog_fetch(s, s->pc + 1 + (s->lk_used ? 1 : 0));
            consumed = 2;
            if (pre != 0) {
                s->delayed_pc  = pmad;
                s->delay_slots = 2;
            }
            return consumed + s->lk_used;
        }
        /* 0x6F00-0x6FFF: Extended ADD/SUB/LD/STH/STL Smem, SHIFT, DST/SRC (2-word).
         * Per binutils tic54x-opc.c (verified against insn_template struct
         * include/opcode/tic54x.h:85-150):
         *   word0 = 0x6F00 mask 0xFF00 (Smem in low 7 bits)
         *   word1 = sub-opcode in bits 7:5, SRC=bit 9, DST/SRC1=bit 8,
         *           SHIFT=signed 5-bit in bits 4:0
         *     bits 7:5 = 000 → ADD Smem,SHIFT,SRC,[DST]
         *     bits 7:5 = 001 → SUB Smem,SHIFT,SRC,[DST]
         *     bits 7:5 = 010 → LD  Smem,SHIFT,DST
         *     bits 7:5 = 011 → STH SRC1,SHIFT,Smem
         *     bits 7:5 = 100 → STL SRC1,SHIFT,Smem
         *
         * ⚠️ Without this handler, the fallback at (op & 0xF800) == 0x6800 below
         * decodes 0x6Fxx as a 1-word LD Smem,T, PC drifts by one word and the
         * second word executes as a stray instruction. The ROM has 544 such sites.
         * That is what wedged PC=0x8353 in a CALAD A self-loop: 0x6F07 0x0C41
         * mis-decoded let 0x0C41 run as a stray SUB Smem,TS,A, giving A_low=0xFFFA
         * and then A_low=0x8353 after the following ADD. */
        if ((op & 0xFF00) == 0x6F00) {
            addr = resolve_smem(s, op, &ind);
            op2 = prog_fetch(s, s->pc + 1 + (s->lk_used ? 1 : 0));
            int sub = (op2 >> 5) & 0x7;
            int shift_raw = op2 & 0x1F;
            int shift = (shift_raw & 0x10) ? (shift_raw - 32) : shift_raw;
            int dst_b = (op2 >> 8) & 1;   /* bit 8 = DST/SRC1 */
            int src_b = (op2 >> 9) & 1;   /* bit 9 = SRC (ADD/SUB only) */
            consumed = 2;

            switch (sub) {
            case 0: { /* ADD Smem,SHIFT,SRC,[DST]: DST = SRC + (data[Smem]<<shift) */
                uint16_t mv = data_read(s, addr);
                int64_t v = (s->st1 & ST1_SXM) ? (int64_t)(int16_t)mv : (int64_t)mv;
                v = (shift >= 0) ? (v << shift) : (v >> (-shift));
                int64_t src = src_b ? s->b : s->a;
                int64_t result = sext40(src + v);
                if (dst_b) s->b = result; else s->a = result;
                break;
            }
            case 1: { /* SUB Smem,SHIFT,SRC,[DST]: DST = SRC - (data[Smem]<<shift) */
                uint16_t mv = data_read(s, addr);
                int64_t v = (s->st1 & ST1_SXM) ? (int64_t)(int16_t)mv : (int64_t)mv;
                v = (shift >= 0) ? (v << shift) : (v >> (-shift));
                int64_t src = src_b ? s->b : s->a;
                int64_t result = sext40(src - v);
                if (dst_b) s->b = result; else s->a = result;
                break;
            }
            case 2: { /* LD Smem,SHIFT,DST: DST = data[Smem] << shift (SXM-aware) */
                uint16_t mv = data_read(s, addr);
                int64_t v = (s->st1 & ST1_SXM) ? (int64_t)(int16_t)mv : (int64_t)mv;
                v = (shift >= 0) ? (v << shift) : (v >> (-shift));
                if (dst_b) s->b = sext40(v); else s->a = sext40(v);
                break;
            }
            case 3: { /* STH SRC1,SHIFT,Smem: data[Smem] = (SRC1 high 16) << shift */
                int64_t src = dst_b ? s->b : s->a;
                int16_t high = (int16_t)((src >> 16) & 0xFFFF);
                int64_t shifted = (shift >= 0) ? ((int64_t)high << shift)
                                               : ((int64_t)high >> (-shift));
                data_write(s, addr, (uint16_t)(shifted & 0xFFFF));
                break;
            }
            case 4: { /* STL SRC1,SHIFT,Smem: data[Smem] = (SRC1 low) << shift */
                int64_t src = dst_b ? s->b : s->a;
                int64_t shifted = (shift >= 0) ? (src << shift) : (src >> (-shift));
                data_write(s, addr, (uint16_t)(shifted & 0xFFFF));
                break;
            }
            default:
                { static int unk6f = 0; if (unk6f++ < 10)
                    C54_LOG("0x6F unknown sub=%d op=0x%04x op2=0x%04x PC=0x%04x",
                            sub, op, op2, s->pc); }
                break;
            }
            return consumed + s->lk_used;
        }
        if ((op & 0xF800) == 0x6800) {
            /* Unreachable in practice: every 0x68xx-0x6Fxx is taken by a specific
             * handler above (ANDM/ORM/XORM/ADDM/BANZ/BANZD, the extended 0x6F00)
             * or by the 0x6Dxx MAR. This generic "LD Smem,T" fallback used to catch
             * all 2107 of them and drift PC at every one. It is kept as a net: if
             * it ever triggers, the handler for that 0xNN00 prefix is incomplete. */
            addr = resolve_smem(s, op, &ind);
            s->t = data_read(s, addr);
            return consumed + s->lk_used;
        }
        goto unimpl;

    case 0x1: {
        /* 1xxx: LD / LDU / LDR Smem, DST  (per tic54x-opc.c, all mask FE00):
         *   0x1000  LD  Smem, DST          — signed load (SXM-aware)
         *   0x1200  LDU Smem, DST          — unsigned load (zero-extend)
         *   0x1400  LD  Smem, TS, DST      — load shifted by T low bits
         *   0x1600  LDR Smem, DST          — load with rounding
         *
         * ⚠️ The bootloader at PROM0 0xb429 does `LDU *(0x0ffe), A`
         * (op=0x12f8 + lk=0x0ffe) to read BL_ADDR_LO, then BACC A to that target.
         * Decoding case 0x1 as SUB leaves A=0 and the BACC falls into the boot-stub
         * NOPs. */
        addr = resolve_smem(s, op, &ind);
        int dst = (op >> 8) & 1;
        int sub = (op >> 9) & 0x07;  /* selects LD/LDU/LD,TS/LDR within case 1 */
        uint16_t val = data_read(s, addr);
        {   /* LD-TRACE (CALYPSO_DISPATCH_PROBE, read-only), limited to the task
             * dispatcher block 0xb05f..0xb078, so a few dozen lines at most.
             *
             * [2026-08-03] At 0xb060 (`10e1 0000` = LD *AR1(0), A), AR1 holds 0x0814
             * (d_task_d, page W1) and the cell it points at holds 0x0018 (24, ALLC),
             * both confirmed by the CHAIN-B05F probe, yet the accumulator comes out
             * at 0x5294. The chain then compares 0x5294 against the constants
             * 12/30/34, matches none and bails out to 0xb077, which is why index
             * resolution is never reached and RX arming is never requested.
             *
             * Reading it: addr==0x0814 and val==0x0018 with a different final
             * accumulator means the fault is in the accumulator write (sub, dst,
             * sign extension); a different addr means resolve_smem after all. */
            uint16_t _pc = s->last_exec_pc;
            if (_pc >= 0xb05f && _pc <= 0xb078) {
                static int _lt = -1; static unsigned _ltn = 0;
                if (_lt < 0) _lt = calypso_gate("CALYPSO_DISPATCH_PROBE", 0);
                if (_lt && _ltn < 60) {
                    _ltn++;
                    fprintf(stderr,
                            "[dispatch] LD-TRACE pc=0x%04x op=0x%04x dst=%s sub=%d "
                            "addr=0x%04x val_read=0x%04x data[addr]=0x%04x "
                            "A_before=0x%06llx SXM=%d insn=%u\n",
                            _pc, op, dst ? "B" : "A", sub, addr, val,
                            s->data[addr],
                            (unsigned long long)(s->a & 0xFFFFFFULL),
                            (s->st1 & ST1_SXM) ? 1 : 0, s->insn_count);
                    fflush(stderr);
                }
            }
        }
        int64_t v;
        switch (sub) {
        case 0x0:  /* 0x1000: LD Smem, DST — signed (SXM honoured) */
            v = (s->st1 & ST1_SXM) ? (int16_t)val : (uint16_t)val;
            break;
        case 0x1: { /* 0x1200: LDU Smem, DST — always zero-extended */
            v = (uint16_t)val;
            break;
        }
        case 0x2: { /* 0x1400: LD Smem, TS, DST — shift by T[5:0] (signed) */
            int8_t ts = (int8_t)((s->t & 0x3F) | ((s->t & 0x20) ? 0xC0 : 0));
            int64_t base = (s->st1 & ST1_SXM) ? (int16_t)val : (uint16_t)val;
            v = (ts >= 0) ? (base << ts) : (base >> -ts);
            break;
        }
        case 0x3: { /* 0x1600: LDR Smem, DST — load with rounding (+0x8000) */
            v = (s->st1 & ST1_SXM) ? (int16_t)val : (uint16_t)val;
            v = (v << 16) + 0x8000;
            v &= 0xFFFFFFFF0000LL;  /* clear low 16 after rounding */
            if (dst) s->b = sext40(v); else s->a = sext40(v);
            return consumed + s->lk_used;
        }
        /* sub 4..7 are the LOGICAL half of the family; without them they fall into
         * the `default` below and execute as a LD. Encodings per SPRU172C (tables
         * 2-7/2-8/2-9, SUBC p.4-192). Smem is ZERO-extended to 40 bits: the TI
         * example for AND (p.4-12) gives A=00 00FF 1200 & Smem=0x1500 ->
         * A=00 0000 1000.
         * [2026-07-28] Measured: 0x1860 (AND) read as a LD set A=15 instead of
         * A&15, hence T=31 and a `LD Smem,TS` shifting by +31 that saturated the
         * accumulator (A=0x80000000) and flattened the demodulator output. */
        case 0x4: { /* 0x1800: AND Smem, src — src = src & Smem */
            uint64_t cur = (uint64_t)(dst ? s->b : s->a) & 0xFFFFFFFFFFULL;
            uint64_t r = cur & (uint64_t)(uint16_t)val;
            if (dst) s->b = sext40((int64_t)r); else s->a = sext40((int64_t)r);
            return consumed + s->lk_used;
        }
        case 0x5: { /* 0x1A00: OR Smem, src — src = src | Smem */
            uint64_t cur = (uint64_t)(dst ? s->b : s->a) & 0xFFFFFFFFFFULL;
            uint64_t r = cur | (uint64_t)(uint16_t)val;
            if (dst) s->b = sext40((int64_t)r); else s->a = sext40((int64_t)r);
            return consumed + s->lk_used;
        }
        case 0x6: { /* 0x1C00: XOR Smem, src — src = src ^ Smem */
            uint64_t cur = (uint64_t)(dst ? s->b : s->a) & 0xFFFFFFFFFFULL;
            uint64_t r = cur ^ (uint64_t)(uint16_t)val;
            if (dst) s->b = sext40((int64_t)r); else s->a = sext40((int64_t)r);
            return consumed + s->lk_used;
        }
        case 0x7: { /* 0x1E00: SUBC Smem, src — conditional subtract (division) */
            int64_t src = dst ? sext40((int64_t)s->b) : sext40((int64_t)s->a);
            int64_t d = src - ((int64_t)(uint16_t)val << 15);
            int64_t r = (d >= 0) ? ((d << 1) + 1) : (src << 1);
            if (dst) s->b = sext40(r); else s->a = sext40(r);
            return consumed + s->lk_used;
        }
        default:
            v = (s->st1 & ST1_SXM) ? (int16_t)val : (uint16_t)val;
            break;
        }
        if (dst) s->b = sext40(v); else s->a = sext40(v);
        /* LDU-PTR probe (CALYPSO_DEBUG=LDU-PTR): at the site that loads A for the
         * CALA to 0 (PC=0xfa7e by default, CALYPSO_TRACE_LDU_PC to move it), dumps
         * the effective address, the value read, the indirect flag and the AR/DP,
         * to name the zero cell: an uninitialised table pointer or a wrong EA. */
        {
            static int ldu_trace_pc = -1;
            if (ldu_trace_pc < 0) {
                const char *e = calypso_getenv("CALYPSO_TRACE_LDU_PC");
                ldu_trace_pc = (e && *e) ? (int)strtol(e, NULL, 0) : 0xfa7e;
            }
            if (s->pc == (uint16_t)ldu_trace_pc) {
                C54_DBG("LDU-PTR",
                    "LDU-PTR PC=0x%04x op=0x%04x sub=%d EA=0x%04x val=0x%04x ind=%d "
                    "DP=0x%03x AR0=%04x AR1=%04x AR2=%04x AR3=%04x AR4=%04x "
                    "AR5=%04x AR6=%04x AR7=%04x insn=%u",
                    s->pc, op, sub, addr, val, ind, (s->st0 & 0x1FF),
                    s->ar[0], s->ar[1], s->ar[2], s->ar[3],
                    s->ar[4], s->ar[5], s->ar[6], s->ar[7],
                    (unsigned)s->insn_count);
            }
        }
        /* CALAD-zone LD trace: every LD/LDU/LDR that targets A while
         * executing in DARAM near the CALAD cluster. Reveals what
         * address/value is feeding A right before each CALAD A. */
        if (dst == 0 && (s->pmst & PMST_OVLY) &&
            s->pc >= 0x10b0 && s->pc < 0x1100) {
            static uint64_t ldA_total;
            ldA_total++;
            if (ldA_total <= 60 || (ldA_total % 5000) == 0) {
                C54_LOG("LD-A-TRACE #%llu PC=0x%04x op=0x%04x sub=%d addr=0x%04x val=0x%04x A_after=0x%04x DP=0x%03x",
                        (unsigned long long)ldA_total,
                        s->pc, op, sub, addr, val,
                        (uint16_t)(s->a & 0xFFFF),
                        (s->st0 & 0x1FF));
            }
        }
        return consumed + s->lk_used;
    }

    case 0x0: {
        /* 0xxx: ADD / ADDS / ADD,TS / SUB / SUBS / SUB,TS  (mask FE00):
         *   0x0000 ADD  Smem, SRC1 (no shift, SXM honoured)
         *   0x0200 ADDS Smem, SRC1 (no shift, zero-extended)
         *   0x0400 ADD  Smem, TS, SRC1
         *   0x0800 SUB  Smem, SRC1
         *   0x0A00 SUBS Smem, SRC1
         *   0x0C00 SUB  Smem, TS, SRC1
         * ⚠️ Plain ADD/SUB take no shift; shifting by 16 unconditionally is wrong.
         */
        addr = resolve_smem(s, op, &ind);
        int dst = (op >> 8) & 1;
        int sub = (op >> 9) & 0x07;  /* 0..7 */
        uint16_t val = data_read(s, addr);
        int64_t v;
        bool is_sub = (sub & 0x4) != 0;
        bool is_unsigned = (sub == 1 || sub == 5);  /* ADDS / SUBS */
        bool ts_shift = (sub == 2 || sub == 6);     /* ,TS variants */
        /* sub 3 = ADDC (0x0600) and sub 7 = SUBB (0x0E00) carry the carry; the
         * generic ADD/SUB path drops it. SPRU172C:
         *   ADDC Smem, src : src = src + Smem + C
         *   SUBB Smem, src : src = src - Smem - C
         * binutils: addc 0x0600/0xFE00, subb 0x0E00/0xFE00, one word each.
         * This follows the manual literally (- C); some SUBB implementations
         * subtract the borrow (~C) instead. */
        bool with_carry = (sub == 3 || sub == 7);
        v = is_unsigned ? (uint16_t)val
                        : ((s->st1 & ST1_SXM) ? (int16_t)val : (uint16_t)val);
        if (ts_shift) {
            int8_t ts = (int8_t)((s->t & 0x3F) | ((s->t & 0x20) ? 0xC0 : 0));
            v = (ts >= 0) ? (v << ts) : (v >> -ts);
        }
        {
            /* [2026-09-21] SUBB subtracts the BORROW, the logical inverse of C
             * (SPRU172C example: A=6, C=0, SUBB 6 -> A=-1 ; B=..06, C=1, SUBB 6
             * -> B=..00). isa_test 229/230. ADDC adds C. */
            int64_t c = with_carry ? ((s->st0 & ST0_C) ? 1 : 0) : 0;
            if (is_sub && with_carry) c = 1 - c;
            if (is_sub) {
                if (dst) s->b = sext40(s->b - v - c);
                else     s->a = sext40(s->a - v - c);
            } else {
                if (dst) s->b = sext40(s->b + v + c);
                else     s->a = sext40(s->a + v + c);
            }
        }
        /* CALAD-zone ADD/SUB trace: same scope as LD-A-TRACE. */
        if (dst == 0 && (s->pmst & PMST_OVLY) &&
            s->pc >= 0x10b0 && s->pc < 0x1100) {
            static uint64_t addA_total;
            addA_total++;
            if (addA_total <= 30 || (addA_total % 5000) == 0) {
                C54_LOG("ADDSUB-A-TRACE #%llu PC=0x%04x op=0x%04x sub=%d addr=0x%04x val=0x%04x A_after=%010llx",
                        (unsigned long long)addA_total,
                        s->pc, op, sub, addr, val,
                        (unsigned long long)(s->a & 0xFFFFFFFFFFULL));
            }
        }
        return consumed + s->lk_used;
    }

    case 0x3:
        {   /* ⚠️ MAC/bit handlers, called BEFORE any resolve_smem: each one does
             * its own, and calling this later would double post-increment the ARs. */
            int _h = c54x_mac_bit_family(s, op, consumed);
            if (_h >= 0) return _h;
        }
        /* 3xxx: MAC / MAS, but SQURA and BITT first. */
        addr = resolve_smem(s, op, &ind);
        {
            uint16_t val = data_read(s, addr);
            /* SQURA Smem,src (0x3800/0xFE00, bit 8 = src): src = src + Smem*Smem.
             * ⚠️ The blind MAC of case 0x3 runs it as `acc += T*Smem`, so the energy
             * (a sum of squares) is computed from the wrong operand and sign and A
             * stays <= 0 at PROM0 0x76ff/0x7700. The RCD LEQ at 0x75e8 then takes
             * the early exit, skipping the body that pushes ST1, and POPM ST1 at
             * 0x7706 over-pops (first orphan pop at insn 146) until SP collapses.
             * SQURA accumulates a SQUARE, a non-negative contribution, so A > 0 and
             * the RCD is not taken. */
            /* ═════════════════════════════════════════════════════════════════
             * FIX_BITT_CASE3 — BITT must be decoded here, in `case 0x3:`.
             *
             * `bitt` is 0x34xx, so hi4 = 3. A handler for it placed in `case 0xF:`
             * is unreachable by construction and 0x348e falls into the blind MAC of
             * this case instead.
             *
             * TI SPRU172C: `BITT Smem` -> `TC = Smem(15 - T(3-0))`, encoding
             * `0011 0100 IAAAAAAA`, "Status Bits: Affects TC" AND NOTHING ELSE. The
             * blind MAC is therefore wrong twice over: TC is never set and stays
             * stale, and accumulator A is corrupted by an accumulation that should
             * not happen.
             *
             * [2026-08-04] Measured: the bit assembler at 0x9ab8..0x9ad2
             * (`bitt *AR6-` -> `roltc A` -> `stl *AR2-,A`) packed mush into
             * 0x2c3c..0x2c47, which the `mvdd` at 0x9723 published into a_cd[3..].
             * The ROLTC-WATCH probe measured incoming TC at 1445/5000 (about 29%),
             * i.e. noise rather than a bit test.
             * ⚠️ A was non-zero and plausible-looking (0x33c7eed1bb...) and still
             * pure MAC mush: a non-zero value is not a correct value.
             *
             * SQURA just below was extracted from the same blind MAC for the same
             * reason.
             * ═════════════════════════════════════════════════════════════════ */
            /* ADD Smem, 16, src [, dst] : dst = src + (Smem << 16).
             * Encoding 0011 11SD IAAAAAAA (binutils {"add", 0x3C00, 0xFC00,
             * {Smem, 16, SRC, DST}}, SPRU172C 4-4). [2026-09-20] Had no handler
             * and fell into the blind MAC below (acc += T*Smem). Sites: SCH
             * decoder 0x9a7d `3d81 add *AR1,16,A,B` (branch metric s0+s1), SB
             * equalizer 0x847f/0x849b/0x837a/0x8390 `3f89 add *AR1-,16,B`. */
            if ((op & 0xFC00) == 0x3C00) {
                int64_t srcv = ((op >> 9) & 1) ? s->b : s->a;
                int64_t r = sext40(srcv + ((int64_t)(int16_t)val << 16));
                if ((op >> 8) & 1) s->b = r; else s->a = r;
                return consumed + s->lk_used;
            }
            if ((op & 0xFF00) == 0x3400) {
                int bitt_idx = 15 - (s->t & 0xF);
                if ((val >> bitt_idx) & 1) s->st0 |= ST0_TC;
                else                       s->st0 &= ~ST0_TC;
                return consumed + s->lk_used;
            }

            if ((op & 0xFE00) == 0x3800) {
                int64_t sq = (int64_t)(int16_t)val * (int64_t)(int16_t)val;
                if (s->st1 & ST1_FRCT) sq <<= 1;
                int sdst = (op >> 8) & 1;
                if (sdst) s->b = sext40(s->b + sq);
                else      s->a = sext40(s->a + sq);
                /* SPRU172C: "SQURA Smem, src : src = src + Smem * Smem, T = Smem".
                 * ⚠️ Without the T write, every later instruction that uses T (MAC,
                 * LD Smem,TS, ...) works on a stale value. */
                s->t = val;
                return consumed + s->lk_used;
            }
            int dst = (op >> 8) & 1;
            int64_t product = (int64_t)(int16_t)s->t * (int64_t)(int16_t)val;
            if (s->st1 & ST1_FRCT) product <<= 1;
            if (dst) s->b = sext40(s->b + product);
            else     s->a = sext40(s->a + product);
        }
        return consumed + s->lk_used;

    case 0x2:
        {   /* ⚠️ MAC/bit handlers, called BEFORE any resolve_smem: each one does
             * its own, and calling this later would double post-increment the ARs. */
            int _h = c54x_mac_bit_family(s, op, consumed);
            if (_h >= 0) return _h;
        }
        /* 2xxx: MPY, SQUR, MAS, MAC variants */
        {
            int sub = (op >> 8) & 0xF;
            addr = resolve_smem(s, op, &ind);
            uint16_t val = data_read(s, addr);
            int64_t product;
            int dst;
            switch (sub) {
            case 0x0: case 0x1: /* MPY Smem, A/B */
                product = (int64_t)(int16_t)s->t * (int64_t)(int16_t)val;
                if (s->st1 & ST1_FRCT) product <<= 1;
                if (sub & 1) s->b = sext40(product);
                else         s->a = sext40(product);
                return consumed + s->lk_used;
            /* binutils: mpyr 0x2200/0xFE00, mpyu 0x2400/0xFE00, squr 0x2600/0xFE00,
             * so `sub = (op >> 8) & 0xF` is 2/3 for MPYR, 4/5 for MPYU and 6/7 for
             * SQUR. ⚠️ Placing SQUR at 4/5 puts it on MPYU and drops 2/3 and 6/7
             * into `default:` (MAS). What matters is T:
             *   MPYU (SPRU172C): dst = uns(T)*uns(Smem), T UNCHANGED -- writing
             *        `s->t = val` there is a stray write
             *   SQUR: dst = Smem*Smem AND T = Smem
             *   MPYR: dst = rnd(T*Smem), an assignment, T unchanged -- not an
             *        accumulate-and-subtract like MAS
             * [2026-08-23] MPYR has two sites, 0x8166 and 0x816c, inside the very
             * routine that loads T for the MPY at 0x81e4. */
            case 0x2: case 0x3: /* MPYR Smem, dst : dst = rnd(T * Smem) */
                product = (int64_t)(int16_t)s->t * (int64_t)(int16_t)val;
                if (s->st1 & ST1_FRCT) product <<= 1;
                /* @BEQUILLE [2026-09-21] CALYPSO_HACK_SOFT_SCALE=<k> : diagnostic only.
                 * The NB soft-bit scaler (0x8166/0x816c/0x8175) multiplies the
                 * equaliser output by a scale T that comes out as 2 (or 0) on the
                 * synthetic cell, so rnd(T*soft)>>13 is 0..3 and the 129-entry
                 * quantiser table is indexed at 0: every soft bit is stored +1.
                 * Shifting the product by k here says whether a k-bit larger
                 * scale is all that separates the decoder from the SI. */
                { static int hk = -1; if (hk < 0) { const char *e = getenv("CALYPSO_HACK_SOFT_SCALE"); hk = (e && *e) ? atoi(e) : 0; }
                  if (hk && (s->pc == 0x8166 || s->pc == 0x816c || s->pc == 0x8175)) product <<= hk; }
                product = (product + 0x8000) & ~0xFFFFLL;   /* [2026-09-21] rnd(): +2^15 then bits 15-0 cleared (isa_test 132: MPYR 0,B -> 0x6260000) */
                if (sub & 1) s->b = sext40(product);
                else         s->a = sext40(product);
                return consumed + s->lk_used;      /* T UNCHANGED */
            case 0x4: case 0x5: /* MPYU Smem, dst : dst = uns(T) * uns(Smem) */
                product = (int64_t)(uint16_t)s->t * (int64_t)(uint16_t)val;
                if (s->st1 & ST1_FRCT) product <<= 1;
                if (sub & 1) s->b = sext40(product);
                else         s->a = sext40(product);
                return consumed + s->lk_used;      /* T UNCHANGED */
            case 0x6: case 0x7: /* SQUR Smem, dst : dst = Smem*Smem ; T = Smem */
                product = (int64_t)(int16_t)val * (int64_t)(int16_t)val;
                if (s->st1 & ST1_FRCT) product <<= 1;
                s->t = val;
                if (sub & 1) s->b = sext40(product);
                else         s->a = sext40(product);
                return consumed + s->lk_used;
            case 0x8: case 0x9: /* MPYA Smem (A = T * Smem, B += A) or variants */
                product = (int64_t)(int16_t)s->t * (int64_t)(int16_t)val;
                if (s->st1 & ST1_FRCT) product <<= 1;
                if (sub & 1) { s->a += s->b; s->b = sext40(product); }
                else         { s->b += s->a; s->a = sext40(product); }
                return consumed + s->lk_used;
            case 0xA: case 0xB: /* MACA[R] Smem, A/B (A += B * Smem then B = T * Smem) */
                dst = sub & 1;
                product = (int64_t)(int16_t)s->t * (int64_t)(int16_t)val;
                if (s->st1 & ST1_FRCT) product <<= 1;
                if (dst) { s->a = sext40(s->a + s->b); s->b = sext40(product); }
                else     { s->b = sext40(s->b + s->a); s->a = sext40(product); }
                s->t = val;
                return consumed + s->lk_used;
            default:
                /* MAS variants and others */
                product = (int64_t)(int16_t)s->t * (int64_t)(int16_t)val;
                if (s->st1 & ST1_FRCT) product <<= 1;
                dst = sub & 1;
                if (dst) s->b = sext40(s->b - product);
                else     s->a = sext40(s->a - product);
                return consumed + s->lk_used;
            }
        }

    case 0x4:
        /* 0x4xxx group — per binutils tic54x-opc.c:
         *   0x40-0x43  SUB Smem,16,src[,dst]    (mask 0xFC00)
         *   0x44-0x45  LD  Smem,16,dst          (mask 0xFE00)
         *   0x4600     LD  Smem,DP              (mask 0xFF00)
         *   0x4700     RPT Smem                 (mask 0xFF00)
         *   0x48-0x49  LDM MMR,dst              (mask 0xFE00)
         *   0x4A00     PSHM MMR                 (mask 0xFF00)
         *   0x4B00     PSHD Smem                (mask 0xFF00)
         *   0x4C00     LTD Smem                 (mask 0xFF00)
         *   0x4D00     DELAY Smem               (mask 0xFF00)
         *   0x4E-0x4F  DST src,Lmem             (mask 0xFE00) */
        {
            uint8_t op8 = hi8;            /* (op >> 8) & 0xFF */
            int dst_b = op8 & 0x01;        /* bit8 = src/dst select (A=0, B=1) */
            int64_t *acc_dst = dst_b ? &s->b : &s->a;

            if (op8 >= 0x40 && op8 <= 0x43) {
                /* SUB Smem, 16, src [, dst] : dst = src - (Smem << 16).
                 * Encoding 0100 00SD IAAAAAAA: bit 9 = src, bit 8 = dst (binutils
                 * {"sub", 0x4000, 0xFC00, {Smem, 16, SRC, DST}}, SPRU172C 4-187).
                 * [2026-09-20] src was ignored (dst used as src): the SCH decoder's
                 * `4191 sub *AR1+,16,A,B` at 0x9a7f computed B - s1 instead of
                 * A - s1, so the second branch metric (s0-s1) was garbage. */
                addr = resolve_smem(s, op, &ind);
                int64_t val = (int64_t)(int16_t)data_read(s, addr) << 16;
                int64_t srcv = ((op >> 9) & 1) ? s->b : s->a;
                *acc_dst = sext40(srcv - val);
                return consumed + s->lk_used;
            }
            if (op8 == 0x44 || op8 == 0x45) {
                /* LD Smem << 16, dst */
                addr = resolve_smem(s, op, &ind);
                int64_t val = (int64_t)(int16_t)data_read(s, addr) << 16;
                *acc_dst = sext40(val);
                return consumed + s->lk_used;
            }
            if (op8 == 0x46) {
                /* LD Smem, DP — load DP from low 9 bits of Smem */
                addr = resolve_smem(s, op, &ind);
                uint16_t val = data_read(s, addr);
                s->st0 = (s->st0 & ~ST0_DP_MASK) | (val & ST0_DP_MASK);
                g_last_ldp_pc = s->pc; g_last_ldp_val = (val & ST0_DP_MASK); g_last_ldp_kind = 3;
                return consumed + s->lk_used;
            }
            if (op8 == 0x47) {
                /* RPT Smem loads the SINGLE repeat counter (RC), not the block
                 * counter (BRC). SPRU172C: "RPT Smem: Repeat single, RC = Smem";
                 * binutils: rpt 0x4700/0xFF00, one word.
                 * ⚠️ Writing s->brc instead leaves rpt_count at its previous value,
                 * so the RPT has no effect at all, and corrupts BRC on the way.
                 * Like the RPT #k8u handler (0xEC00): advance PC and return 0 so
                 * the dispatcher re-executes the NEXT instruction, not the RPT. */
                addr = resolve_smem(s, op, &ind);
                uint16_t val = data_read(s, addr);
                s->rpt_count = val;
                s->rpt_active = true; s->rpt_fresh = true;
                s->pc += 1;
                return 0;
            }
            if (op8 == 0x48 || op8 == 0x49) {
                /* LDM MMR, dst — load accumulator from a memory-mapped reg */
                int mmr = op & 0x7F;
                uint16_t val = data_read(s, mmr);
                *acc_dst = sext40((int16_t)val);
                return consumed + s->lk_used;
            }
            if (op8 == 0x4A) {
                /* PSHM MMR — push memory-mapped reg onto stack */
                int mmr = op & 0x7F;
                uint16_t val = data_read(s, mmr);
                if (mmr == MMR_ST0) st0_ring_rec(s, val, 'P'); /* push ST0 (C-sweep) */
                s->sp = (s->sp - 1) & 0xFFFF;
                data_write(s, s->sp, val);
                return consumed + s->lk_used;
            }
            if (op8 == 0x4B) {
                /* PSHD Smem — push data memory onto stack */
                addr = resolve_smem(s, op, &ind);
                uint16_t val = data_read(s, addr);
                s->sp = (s->sp - 1) & 0xFFFF;
                data_write(s, s->sp, val);
                return consumed + s->lk_used;
            }
            if (op8 == 0x4C) {
                /* LTD Smem — T = mem[Smem]; mem[Smem+1] = mem[Smem] */
                addr = resolve_smem(s, op, &ind);
                uint16_t val = data_read(s, addr);
                s->t = val;
                data_write(s, (addr + 1) & 0xFFFF, val);
                return consumed + s->lk_used;
            }
            if (op8 == 0x4D) {
                /* DELAY Smem — mem[Smem+1] = mem[Smem] (delay-line shift) */
                addr = resolve_smem(s, op, &ind);
                uint16_t val = data_read(s, addr);
                data_write(s, (addr + 1) & 0xFFFF, val);
                return consumed + s->lk_used;
            }
            if (op8 == 0x4E || op8 == 0x4F) {
                /* DST src, Lmem — store accumulator to long memory.
                 * Lmem = even-aligned 32-bit pair: mem[L]=high, mem[L+1]=low */
                addr = resolve_smem(s, op, &ind) & 0xFFFE;
                int64_t v = *acc_dst;
                data_write(s, addr,         (uint16_t)((v >> 16) & 0xFFFF));
                data_write(s, (addr+1)&0xFFFF, (uint16_t)(v & 0xFFFF));
                return consumed + s->lk_used;
            }
        }
        return consumed + s->lk_used;

    case 0x5:
        /* 5xxx: shifts — SFTA, SFTL and the dual long-word family.
         * ⚠️ 0x56xx/0x57xx are not MVPD (which is 0x8Cxx): decoding them as MVPD
         * makes resolve_smem write MMR_SP and corrupts the stack. */
        {
            /* Dual long-word DADST/DSADT Lmem,dst (1 word). SPRU172C encoding:
             * 0101 101D = DADST (0x5A/5B), 0101 111D = DSADT (0x5E/5F); D (bit 8)
             * is dst (0=A, 1=B); bits 7:0 are the Lmem operand. Semantics, checked
             * against the worked examples in SPRU172C:
             *   C16=1 (dual-16, unsaturated):
             *     DADST: dst(39-16)=Lmem.hi+T ; dst(15-0)=Lmem.lo-T
             *     DSADT: dst(39-16)=Lmem.hi-T ; dst(15-0)=Lmem.lo+T
             *   C16=0 (double precision, SXM):
             *     DADST: dst=Lmem+((T<<16)|T) ; DSADT: dst=Lmem-((T<<16)|T)
             * Lmem post-modification is +-2 (long operand, via resolve_lmem), and
             * C16 is read at execution time from ST1, tracked by SSBX/RSBX C16.
             * ⚠️ Without this handler these opcodes fall into the SFTA/SFTL block
             * below, which flattens the FCCH correlator into "dst >>= ASM" and
             * leaves d_fb_det at 0 (SHADOW-DADST probe: shiftLike=1, walked=0). */
            uint8_t dl_hi = (op >> 8) & 0xFF;
            if (dl_hi >= 0x50 && dl_hi <= 0x5F) {
                /* Full dual long-word family (SPRU172C):
                 *   0x50-53 DADD(00SD)  0x54-55 DSUB(010S)  0x56-57 DLD(011D)
                 *   0x58-59 DRSUB(100S) 0x5A-5B DADST(101D) 0x5C-5D DSUBT(110D)
                 *   0x5E-5F DSADT(111D)
                 * 32-bit Lmem via resolve_lmem (post-mod +-2), branching on
                 * ST1.C16. Checked bit by bit against the SPRU172C worked examples
                 * for DADD, DADST and DSADT. */
                uint16_t laddr  = resolve_lmem(s, op);
                uint16_t lhi    = data_read(s, laddr);
                /* Lmem: high word AT the address, low word at address ^ 1 (SPRU172C
                 * DADD example, AR3 = 0101h: hi = data[0101h], lo = data[0100h]). */
                uint16_t llo    = data_read(s, (uint16_t)(laddr ^ 1));
                int      c16    = (s->st1 & ST1_C16) != 0;
                int      sxm    = (s->st1 & ST1_SXM) != 0;
                int16_t  lhi16  = (int16_t)lhi, llo16 = (int16_t)llo;
                uint32_t lmem32 = ((uint32_t)lhi << 16) | llo;
                int64_t  lmem40 = sxm ? (int64_t)(int32_t)lmem32 : (int64_t)(uint32_t)lmem32;
                if (dl_hi <= 0x53) {                 /* DADD Lmem,src[,dst] : dst = src + Lmem */
                    int64_t *src = ((op >> 9) & 1) ? &s->b : &s->a;
                    int64_t *dst = ((op >> 8) & 1) ? &s->b : &s->a;
                    if (!c16) *dst = sext40(*src + lmem40);
                    else { int32_t hi = (int32_t)(int16_t)((*src >> 16) & 0xFFFF) + lhi16;
                           int32_t lo = (int32_t)(int16_t)(*src & 0xFFFF) + llo16;
                           *dst = sext40(((int64_t)hi << 16) | ((uint32_t)lo & 0xFFFF)); }
                } else if (dl_hi <= 0x55) {          /* DSUB Lmem,src : src = src - Lmem */
                    int64_t *src = ((op >> 8) & 1) ? &s->b : &s->a;
                    if (!c16) *src = sext40(*src - lmem40);
                    else { int32_t hi = (int32_t)(int16_t)((*src >> 16) & 0xFFFF) - lhi16;
                           int32_t lo = (int32_t)(int16_t)(*src & 0xFFFF) - llo16;
                           *src = sext40(((int64_t)hi << 16) | ((uint32_t)lo & 0xFFFF)); }
                } else if (dl_hi <= 0x57) {          /* DLD Lmem,dst : dst = Lmem */
                    int64_t *dst = ((op >> 8) & 1) ? &s->b : &s->a;
                    if (!c16) *dst = sext40(lmem40);
                    else *dst = sext40(((int64_t)lhi16 << 16) | (uint16_t)llo);
                } else if (dl_hi <= 0x59) {          /* DRSUB Lmem,src : src = Lmem - src */
                    int64_t *src = ((op >> 8) & 1) ? &s->b : &s->a;
                    if (!c16) *src = sext40(lmem40 - *src);
                    else { int32_t hi = lhi16 - (int32_t)(int16_t)((*src >> 16) & 0xFFFF);
                           int32_t lo = llo16 - (int32_t)(int16_t)(*src & 0xFFFF);
                           *src = sext40(((int64_t)hi << 16) | ((uint32_t)lo & 0xFFFF)); }
                } else {                              /* DADST/DSUBT/DSADT Lmem,dst : use T */
                    int64_t *dst = ((op >> 8) & 1) ? &s->b : &s->a;
                    int16_t t16 = (int16_t)s->t;
                    int64_t r;
                    if (c16) {
                        int sgn_hi = (dl_hi <= 0x5B) ? +1 : -1;   /* DADST hi+T; DSUBT/DSADT hi-T */
                        int sgn_lo = (dl_hi <= 0x5D) ? -1 : +1;   /* DADST/DSUBT lo-T; DSADT lo+T */
                        int32_t hi = (int32_t)lhi16 + sgn_hi * (int32_t)t16;
                        int32_t lo = (int32_t)llo16 + sgn_lo * (int32_t)t16;
                        r = ((int64_t)hi << 16) | ((uint32_t)lo & 0xFFFF);
                    } else {
                        uint32_t tt32 = ((uint32_t)(uint16_t)t16 << 16) | (uint16_t)t16;
                        int64_t tt40 = sxm ? (int64_t)(int32_t)tt32 : (int64_t)(uint32_t)tt32;
                        r = (dl_hi <= 0x5B) ? (lmem40 + tt40) : (lmem40 - tt40); /* DADST adds, the others subtract */
                    }
                    *dst = sext40(r);
                }
                return consumed + s->lk_used;
            }
            int dst = (op >> 8) & 1;
            int64_t *acc = dst ? &s->b : &s->a;
            int sub = (op >> 9) & 0x7;
            if (sub <= 1) {
                /* 50xx/51xx: SFTA src, ASM shift */
                int shift = asm_shift(s);
                if (shift >= 0) *acc = sext40(*acc << shift);
                else            *acc = sext40(*acc >> (-shift));
            } else if (sub == 2 || sub == 3) {
                /* 54xx/55xx: SFTA src, #shift (immediate in Smem) */
                addr = resolve_smem(s, op, &ind);
                int shift = (int16_t)data_read(s, addr);
                if (shift >= 0) *acc = sext40(*acc << shift);
                else            *acc = sext40(*acc >> (-shift));
            } else if (sub == 4 || sub == 5) {
                /* 58xx/59xx: SFTL src, ASM shift (logical) */
                int shift = asm_shift(s);
                uint64_t u = (uint64_t)(*acc) & 0xFFFFFFFFFFULL;
                if (shift >= 0) *acc = sext40((int64_t)(u << shift));
                else            *acc = sext40((int64_t)(u >> (-shift)));
            } else if (sub == 6 || sub == 7) {
                /* 5Cxx/5Dxx/5Exx/5Fxx: SFTL with Smem or other */
                addr = resolve_smem(s, op, &ind);
                int shift = (int16_t)data_read(s, addr);
                uint64_t u = (uint64_t)(*acc) & 0xFFFFFFFFFFULL;
                if (shift >= 0) *acc = sext40((int64_t)(u << shift));
                else            *acc = sext40((int64_t)(u >> (-shift)));
            }
        }
        return consumed + s->lk_used;

    case 0x8: case 0x9:
        /* 8xxx/9xxx: Memory moves, PORTR/PORTW */

        /* FIX_ADDSUB_XSHFT — per binutils and SPRU172C 4-4 / 4-187, 0x9000
         * (mask FE00) is ADD Xmem,SHFT,src (src += Xmem << SHFT) and 0x9200 is
         * SUB Xmem,SHFT,src (src -= Xmem << SHFT), with bit 8 = src, SHFT in bits
         * 3:0 and SXM applied.
         * ⚠️ Run as MAC Xmem,Ymem instead they corrupt the SB path, which has such
         * a site at 0x988e (`9086`). */
        if ((op & 0xFC00) == 0x9000) {
            uint16_t xa = resolve_xmem(s, op);
            uint16_t xv = data_read(s, xa);
            int shft = op & 0xF;
            int64_t v = ((s->st1 & ST1_SXM) ? (int64_t)(int16_t)xv : (int64_t)xv) << shft;
            int64_t *srcp = (op & 0x0100) ? &s->b : &s->a;
            *srcp = sext40((op & 0x0200) ? (*srcp - v) : (*srcp + v));
            return consumed + s->lk_used;
        }

        /* 94xx: MVDK Smem, dmad — Move data(Smem) to data(dmad) (2 words) */
        if (hi8 == 0x94) {
            addr = resolve_smem(s, op, &ind);
            op2 = prog_fetch(s, s->pc + 1);
            consumed = 2;
            data_write(s, op2, data_read(s, addr));
            return consumed + s->lk_used;
        }
        /* 95xx: MVKD dmad, Smem — Move data(dmad) to data(Smem) (2 words) */
        if (hi8 == 0x95) {
            addr = resolve_smem(s, op, &ind);
            op2 = prog_fetch(s, s->pc + 1);
            consumed = 2;
            data_write(s, addr, data_read(s, op2));
            return consumed + s->lk_used;
        }
        /* 96xx: MVDP Smem, pmad — Move data to program (2 words) */
        if (hi8 == 0x96) {
            addr = resolve_smem(s, op, &ind);
            op2 = prog_fetch(s, s->pc + 1);
            consumed = 2;
            {   uint16_t _mv = data_read(s, addr);
                scratchwr_note(s, (uint16_t)op2, _mv, "prog");
                s->prog[op2] = _mv; }
            return consumed + s->lk_used;
        }

        /* Per binutils tic54x-opc.c:
         *   { "stl", 1,3,3, 0x9800, 0xFE00, {OP_SRC1,OP_SHFT,OP_Xmem} }
         *   { "sth", 1,3,3, 0x9A00, 0xFE00, {OP_SRC1,OP_SHFT,OP_Xmem} }
         * so 0x98/0x99 write the LOW half and 0x9A/0x9B the HIGH half, with bit 8
         * selecting A or B. ⚠️ Swapping them makes every shifted STL/STH in the
         * firmware write the wrong half of the accumulator, and the pattern is hot
         * in post-MAC scaling code.
         * The SHFT field is not decoded here; the dedicated FIX_STL_STH_SHFT
         * handler higher up does that. */
        if (hi8 == 0x98 || hi8 == 0x99) {
            /* STL src, SHFT, Xmem — store LOW (acc & 0xFFFF).
             * ⚠️ The operand is Xmem and must go through resolve_xmem: resolve_smem
             * maps a low byte of 0x00-0x1F with bit 7 = 0 into MMR space and
             * clobbers SP/IMR/IFR. Observed at PC=0x8a46 op=0x9918, which stomped
             * SP to 0. */
            addr = resolve_xmem(s, op);
            int src = hi8 & 1;
            int64_t acc = src ? s->b : s->a;
            data_write(s, addr, (uint16_t)(acc & 0xFFFF));
            return consumed + s->lk_used;
        }
        if (hi8 == 0x9A || hi8 == 0x9B) {
            /* STH src, SHFT, Xmem — store HIGH (acc >> 16). Xmem through
             * resolve_xmem, as for STL above. */
            addr = resolve_xmem(s, op);
            int src = hi8 & 1;
            int64_t acc = src ? s->b : s->a;
            data_write(s, addr, (uint16_t)((acc >> 16) & 0xFFFF));
            return consumed + s->lk_used;
        }

        /* 0x9C-0x9F range: SACCD/SRCCD/STRCD — conditional stores */

        /* FIX_XCCD — per SPRU172C 4-152/4-165/4-186: 0x9C = STRCD Xmem,cond
         * (Xmem = T), 0x9D = SRCCD Xmem,cond (Xmem = BRC), 0x9E/0x9F = SACCD
         * src,Xmem,cond with src in bit 8, not bit 9. The 4-bit cond field has
         * bit 3 = accumulator tested (0 = A, 1 = B) and bits 2:0 = 101 EQ, 100 NEQ,
         * 110 GT, 010 GEQ, 011 LT, 111 LEQ.
         * ⚠️ Treating the whole 0x9Cxx range as SACCD tests src instead of the
         * condition's accumulator and reads the condition code backwards. SB sites:
         * 0x847e and 0x849a `9e1e` (SACCD A), 0x831f `9f06` (SACCD B). */
        if ((op & 0xFC00) == 0x9C00) {
            uint16_t xaddr = resolve_xmem(s, op);
            int cond = op & 0x0F;
            int64_t ca = sext40((cond & 0x8) ? s->b : s->a);
            int take;
            switch (cond & 0x7) {
            case 0x5: take = (ca == 0); break;
            case 0x4: take = (ca != 0); break;
            case 0x6: take = (ca > 0);  break;
            case 0x2: take = (ca >= 0); break;
            case 0x3: take = (ca < 0);  break;
            case 0x7: take = (ca <= 0); break;
            default:  take = 0; break;
            }
            uint16_t val;
            if (!take)                        val = data_read(s, xaddr);
            else if (hi8 == 0x9C)             val = s->t;
            else if (hi8 == 0x9D)             val = s->brc;
            else {
                int64_t src = (op & 0x0100) ? s->b : s->a;
                int ash = asm_shift(s);
                int64_t sh = (ash >= 0) ? (src << ash) : (src >> (-ash));
                val = (uint16_t)((sh >> 16) & 0xFFFF);
            }
            data_write(s, xaddr, val);
            return consumed + s->lk_used;
        }
        /* POPM MMR — pop top-of-stack into MMR (1-word).
         * Per tic54x-opc.c: { "popm", 0x8A00, 0xFF00, {OP_MMR} }.
         * Per SPRU172C section 4: the value at SP is popped into the MMR, SP++.
         *
         * ⚠️ 0x8Axx is POPM, not MVDK Smem,dmad (which is 0x7100 mask 0xFF00).
         * Decoded as MVDK, the firmware's symmetric PSHM/POPM pattern (PROM0
         * 0x7013..0x7023 saves and restores 6 MMRs around a CALA) never restores
         * anything after the CALA, ST1 is never restored, INTM stays at 1 forever
         * and IRQ vectoring stops. */
        if ((op & 0xFF00) == 0x8A00) {
            uint16_t mmr = op & 0x7F;
            uint16_t val = data_read(s, s->sp);
            s->sp = (s->sp + 1) & 0xFFFF;
            /* POPM-ST1 probe (CALYPSO_DEBUG=POPM-ST1): ST1 is MMR 0x07. Tells
             * "POPM ST1 never executed" from "executed, but the popped value
             * already has INTM=1", which restores 1 and never clears it. Silent by
             * default. */
            if (mmr == 0x07) {
                C54_DBG("POPM-ST1",
                        "POPM ST1 val=0x%04x INTM_bit=%u PC=0x%04x SP=0x%04x insn=%u",
                        val, !!(val & ST1_INTM), s->pc, s->sp, s->insn_count);
            }
            data_write(s, mmr, val);
            return consumed + s->lk_used;
        }
        /* 0x88xx-0x89xx: STLM src, MMR  (1-word!)
         * Per tic54x-opc.c: { "stlm", 1,2,2, 0x8800, 0xFE00, ... }
         *   bits 9-15 = fixed (0x44)
         *   bit 8     = src (0 = A, 1 = B)
         *   bits 0-6  = MMR address (0x00..0x7F)
         *
         * ⚠️ Critical for the DSP bootloader at PROM0 0xb42d (`STLM B, AR1`):
         * decoded as a 2-word MVDM, the emulator eats the next opcode
         * (0xb42e = 0xf84c, a BC), then enters 0xb431 (MACR family) with an
         * uninitialised T, producing A=0x10, which the BACC A at 0xb430 uses as
         * its jump target and drops the DSP into the boot-stub NOPs at PC=0x0010
         * instead of continuing the bootloader handshake. */
        if (hi8 == 0x88 || hi8 == 0x89) {
            int src = (op >> 8) & 1;  /* 0 = A, 1 = B */
            int mmr = op & 0x7F;
            uint16_t val = src ? (uint16_t)(s->b & 0xFFFF)
                               : (uint16_t)(s->a & 0xFFFF);
            data_write(s, (uint16_t)mmr, val);  /* MMRs alias addr 0x00..0x1F */
            return consumed + s->lk_used;
        }
        /* 0x8Bxx: POPD Smem — pop top-of-stack into data memory, the mirror of
         * PSHD (0x4B): PSHM/POPM are 0x4A/0x8A, PSHD/POPD are 0x4B/0x8B.
         * ⚠️ Missing, it falls through as a 1-word NOP. The overlay frame handler
         * at 0x013b does `POPD *(0x3fcd)` (pops the return of CALL 0x013b), 24
         * PSHM, `PSHD *(0x3fcd)` (pushes the return back), RET. Without POPD the
         * return is buried under the saves, data[0x3fcd] is 0 and RET goes to 0.
         * resolve_smem sets lk_used for the absolute mode (0xf8), giving the
         * correct 2-word length. */
        if (hi8 == 0x8B) {
            addr = resolve_smem(s, op, &ind);
            uint16_t val = data_read(s, s->sp);
            s->sp = (s->sp + 1) & 0xFFFF;
            data_write(s, addr, val);
            return consumed + s->lk_used;
        }
        if (hi8 == 0x80) {
            /* Per binutils tic54x-opc.c:
             *   { "stl", 1,2,2, 0x8000, 0xFE00, {OP_SRC1,OP_Smem}, 0, REST }
             * 0x80xx/0x81xx = STL src,Smem (1 word, no shift), bit 8 = src, so
             * 0x8000-0x80FF is STL A,Smem. ⚠️ Stubbing this as a NOP silently drops
             * every STL A in the firmware and leaves the DARAM variables it should
             * write at stale values. */
            addr = resolve_smem(s, op, &ind);
            data_write(s, addr, (uint16_t)(s->a & 0xFFFF));
            return consumed + s->lk_used;
        }
        if (hi8 == 0x8C) {
            /* Per binutils tic54x-opc.c:
             *   { "mvpd", 2,2,2, 0x7C00, 0xFF00, {OP_pmad,OP_Smem}, 0, REST }
             *   { "st",   1,2,2, 0x8C00, 0xFF00, {OP_T,OP_Smem},    0, REST }
             * MVPD is 0x7C; 0x8C is ST T,Smem (1 word, store T to data memory),
             * the pattern used after a MAC to persist T. Run traces show zero
             * 0x7Cxx sites, the PROM0 overlay being done by the DSP bootloader
             * rather than by MVPD. */
            addr = resolve_smem(s, op, &ind);
            data_write(s, addr, s->t);
            return consumed + s->lk_used;
        }
        /* 0x8E/0x8F : CMPS src, Smem — Compare, Select & Store Maximum
         * (SPRU172C p.4-35). Opcode 1000 111 S I AAAAAAA = 0x8E00/0xFE00,
         * bit8 = src (0=A, 1=B). ONE word (+1 when Smem is long-offset/absolute
         * -> lk_used).
         *   if src(31–16) > src(15–0):  src(31–16)→Smem ; TRN<<=1,TRN(0)=0 ; TC=0
         *   else:                       src(15–0)→Smem  ; TRN<<=1,TRN(0)=1 ; TC=1
         * Compares the two 16-bit two's-complement halves of the accumulator,
         * stores the larger, and records the winner in TRN and TC. Core of the
         * FCCH peak search (Viterbi).
         *
         * ⚠️ 0x8E/0x8F are not MVDP (0x7D) or PORTR (0x74). Decoded as 2-word
         * instructions, each consecutive CMPS A / CMPS B pair (op=8e94 op2=8f93 in
         * the FB-det region 0xa0xx) has its second CMPS swallowed as a phantom
         * pmad, which desynchronises the correlator and leaves d_fb_det unarmed.
         * The I/Q data arrives by DARAM DMA (data[0x2a00]), never through a PORTR
         * opcode. */
        if (hi8 == 0x8E || hi8 == 0x8F) {
            addr = resolve_smem(s, op, &ind);
            int src = (op >> 8) & 1;
            int64_t acc = src ? s->b : s->a;
            int16_t hi = (int16_t)((acc >> 16) & 0xFFFF);
            int16_t lo = (int16_t)(acc & 0xFFFF);
            s->trn = (uint16_t)(s->trn << 1);
            if (hi > lo) {
                data_write(s, addr, (uint16_t)hi);
                s->trn &= ~0x0001u;
                s->st0 &= ~ST0_TC;
            } else {
                data_write(s, addr, (uint16_t)lo);
                s->trn |= 0x0001u;
                s->st0 |= ST0_TC;
            }
            return consumed + s->lk_used;
        }
        if (hi8 == 0x9F) {
            /* PORTW Smem, PA — write I/O port */
            addr = resolve_smem(s, op, &ind);
            op2 = prog_fetch(s, s->pc + 1);
            consumed = 2;
            /* Log I/O port writes */
            {
                uint16_t wval = data_read(s, addr);
                static int portw_log = 0;
                if (portw_log < 30) {
                    C54_LOG("PORTW PA=0x%04x val=0x%04x PC=0x%04x", op2, wval, s->pc);
                    portw_log++;
                }
            }
            return consumed + s->lk_used;
        }
        /* 85xx: MVPD pmad, Smem (prog→data, different encoding) */
        if (hi8 == 0x85) {
            addr = resolve_smem(s, op, &ind);
            op2 = prog_fetch(s, s->pc + 1);
            consumed = 2;
            data_write(s, addr, prog_read(s, op2));
            return consumed + s->lk_used;
        }
        /* 0x86/0x87: STH src,ASM,Smem — store the HIGH half (acc >> 16) shifted
         * by ASM into Smem, ONE word, bit 8 = src (0=A, 1=B), mask 0xFE00.
         * Exact mirror of the 0x84 handler (STL A,ASM,Smem) but storing the high
         * word. resolve_smem applies the indirect Smem post-increment, so do NOT
         * modify the AR again here.
         *
         * ⚠️ These are not MVDM (0x72) or MVMD (0x73). Decoded as 2-word
         * MVDM/MVMD they eat the following opcode and never touch the Smem AR, so
         * AR3 stays frozen at 0 in the FB correlator loop: at 0x7e71 op=0x8693
         * (STH A,ASM,*AR3+) AR3 must post-increment to sweep the I/Q buffer at
         * 0x2a00+. */
        if (hi8 == 0x86 || hi8 == 0x87) {
            addr = resolve_smem(s, op, &ind);
            int shift = asm_shift(s);
            int src = hi8 & 1;            /* 0x86→A, 0x87→B */
            int64_t v = src ? s->b : s->a;
            if (shift >= 0) v <<= shift; else v >>= (-shift);
            data_write(s, addr, (uint16_t)((v >> 16) & 0xFFFF));  /* STH = high word */
            return consumed + s->lk_used;
        }
        /* Store family, per tic54x-opc.c and SPRU172C, bit 8 = src (0=A, 1=B):
         *   stl 0x8000 / 0xFE00 -> 0x80..0x81 STL src,Smem       (no shift)
         *   sth 0x8200 / 0xFE00 -> 0x82..0x83 STH src,Smem       (no shift)
         *   stl 0x8400 / 0xFE00 -> 0x84..0x85 STL src,ASM,Smem   (ASM shift)
         *   sth 0x8600 / 0xFE00 -> 0x86..0x87 STH src,ASM,Smem   (ASM shift)
         * The ASM variants go through asm_shift(), which follows the manual
         * (ASM = ST1[4:0], signed, -16 <= ASM <= 15).
         * ⚠️ The no-shift variants must NOT call asm_shift, and 0x81 stores B, not
         * A. Getting either wrong makes every STL B / STH through indirect *ARn
         * write the wrong value, including d_burst_d (0x0829/0x083D) and d_task_d
         * (0x0828/0x083C) of the NDB, which the ARM then reports as "EMPTY" and
         * mismatched burst ids in prim_rx_nb.c::l1s_nb_resp. */

        /* 0x81xx: STL B, Smem  (src=B, no shift) */
        if (hi8 == 0x81) {
            addr = resolve_smem(s, op, &ind);
            data_write(s, addr, (uint16_t)(s->b & 0xFFFF));
            return consumed + s->lk_used;
        }
        /* 0x82xx: STH A, Smem  (src=A, no shift) */
        if (hi8 == 0x82) {
            addr = resolve_smem(s, op, &ind);
            data_write(s, addr, (uint16_t)((s->a >> 16) & 0xFFFF));
            return consumed + s->lk_used;
        }
        /* 89xx: ST src, Smem with shift or MVDK variants */
        if (hi8 == 0x89) {
            addr = resolve_smem(s, op, &ind);
            op2 = prog_fetch(s, s->pc + 1);
            consumed = 2;
            data_write(s, op2, data_read(s, addr));
            return consumed + s->lk_used;
        }
        /* 0x8B is POPD Smem, handled above; this is unreachable and stays a
         * 1-word no-op. It is NOT a long-address MVDK. */
        if (hi8 == 0x8B) {
            return 1;
        }
        /* 8Dxx: MVDD Smem, Smem */
        if (hi8 == 0x8D) {
            addr = resolve_smem(s, op, &ind);
            op2 = prog_fetch(s, s->pc + 1);
            consumed = 2;
            data_write(s, op2, data_read(s, addr));
            return consumed + s->lk_used;
        }
        /* ⚠️ 0x83 is STH B,Smem, not WRITA (which is 0x7F), and 0x84 is
         * STL A,ASM,Smem, not READA (which is 0x7E). */
        /* 0x83xx: STH B, Smem  (src=B, no shift) */
        if (hi8 == 0x83) {
            addr = resolve_smem(s, op, &ind);
            data_write(s, addr, (uint16_t)((s->b >> 16) & 0xFFFF));
            return consumed + s->lk_used;
        }
        /* 0x84xx: STL A, ASM, Smem (src=A, with ASM shift). The 0x85 (STL B) and
         * 0x86/0x87 (STH A/B) ASM variants have their own handlers above. */
        if (hi8 == 0x84) {
            addr = resolve_smem(s, op, &ind);
            int shift = asm_shift(s);
            int64_t v = s->a;
            if (shift >= 0) v <<= shift; else v >>= (-shift);
            data_write(s, addr, (uint16_t)(v & 0xFFFF));
            return consumed + s->lk_used;
        }
        /* 91xx: MVKD dmad, Smem (another encoding) */
        if (hi8 == 0x91) {
            addr = resolve_smem(s, op, &ind);
            op2 = prog_fetch(s, s->pc + 1);
            consumed = 2;
            data_write(s, addr, data_read(s, op2));
            return consumed + s->lk_used;
        }
        /* 97xx: ST #lk, Smem (2-word). 0x96xx is caught above as MVDP. */
        if (hi8 == 0x97) {
            addr = resolve_smem(s, op, &ind);
            op2 = prog_fetch(s, s->pc + 1);
            consumed = 2;
            data_write(s, addr, op2);
            return consumed + s->lk_used;
        }
        goto unimpl;

    case 0xA: case 0xB:
        /* Axx/Bxx: STLM, LDMM, misc accumulator ops */

        /* ---- Dual-operand MAC/MAS Xmem, Ymem, dst (1-word) ----
         * MAC:  dst += T * Xmem; T = Ymem
         * MACR: dst += rnd(T * Xmem); T = Ymem
         * MAS:  dst -= T * Xmem; T = Ymem
         * MASR: dst -= rnd(T * Xmem); T = Ymem
         * Encoding: OOOO OOOD XXXX YYYY (1 word)
         *   Xmem: AR[ARP], post-mod by bit4 (0=inc,1=dec)
         *   Ymem: AR[bits2:0], post-mod by bit3 (0=inc,1=dec)
         *   D: 0=A, 1=B
         * hi8 mapping per SPRU172C:
         *   0xA4/0xA5: MAC[R] Xmem,Ymem,A   0xA6/0xA7: MAC[R] Xmem,Ymem,B
         *   0xB4/0xB5: MAS[R] Xmem,Ymem,A   0xB6/0xB7: MAS[R] Xmem,Ymem,B
         *   0xB0/0xB1: MAC[R] Xmem,Ymem,A (alt)  0xB2/0xB3 already handled
         */
        if (hi8 == 0xA4 || hi8 == 0xA5 || hi8 == 0xA6 || hi8 == 0xA7 ||
            hi8 == 0xB4 || hi8 == 0xB5 || hi8 == 0xB6 || hi8 == 0xB7 ||
            hi8 == 0xB0 || hi8 == 0xB1 || hi8 == 0xB2 || hi8 == 0xB3) {
            /* 2-bit Xmem/Ymem decoding (SPRU131G tables 5-6/5-8, same as
             * resolve_xmem): Xmod[7:6] Xar[5:4] Ymod[3:2] Yar[1:0], AR = field + 2
             * (AR2..AR5), mod 0 = *AR, 1 = *AR-, 2 = *AR+, 3 = *AR+0% circular
             * (BK, +AR0).
             * ⚠️ A raw 3-bit split ((op>>4)&7 / op&7 with a 1-bit post-mod) picks
             * the wrong ARs: op=0xb4f5 at PC=0xf170 (steady-state TOA correlator)
             * then reads Xmem=AR7 and Ymem=AR5, both outside the buffer, instead of
             * Xmem=AR5 (*AR5+0%) and Ymem=AR3 (*AR3-), so Ymem reads 0, T becomes 0
             * and a_sync_demod[D_TOA] ends up garbage. */
            int xar_d  = ((op >> 4) & 0x03) + 2;
            int yar_d  = (op & 0x03) + 2;
            int xmod_d = (op >> 6) & 0x03;
            int ymod_d = (op >> 2) & 0x03;
            uint16_t xval_d = data_read(s, s->ar[xar_d]);
            uint16_t yval_d = data_read(s, s->ar[yar_d]);
            {   /* MAC-PROBE (CALYPSO_MAC_PROBE, read-only, capped): traces the TOA
                 * correlator at the dual MAC around PC=0xf170, showing Xmem (AR5,
                 * I/Q) and Ymem (AR3, reference), their addresses and values, plus
                 * T. Tells a DC reference (AR3) from an unfed I/Q buffer (AR5). */
                static int _mp = -1; static unsigned _mpn = 0;
                if (_mp < 0) _mp = calypso_gate("CALYPSO_MAC_PROBE", 0);
                if (_mp && s->pc >= 0xf150 && s->pc <= 0xf190 && _mpn < 80) {
                    _mpn++;
                    fprintf(stderr, "[c54x] MAC-PROBE PC=0x%04x op=0x%04x "
                            "Xar=AR%d@0x%04x=0x%04x Yar=AR%d@0x%04x=0x%04x "
                            "T=0x%04x AR3=0x%04x AR5=0x%04x insn=%u\n",
                            s->pc, op, xar_d, s->ar[xar_d], xval_d,
                            yar_d, s->ar[yar_d], yval_d, s->t,
                            s->ar[3], s->ar[5], s->insn_count);
                }
            }
            /* Post-modify (SPRU131G Table 5-8) */
            switch (xmod_d) {
            case 1: s->ar[xar_d]--; break;
            case 2: s->ar[xar_d]++; break;
            case 3: s->ar[xar_d] = c54x_circ_ref(s->ar[xar_d], +(int16_t)s->ar[0], s->bk); break;
            }
            switch (ymod_d) {
            case 1: s->ar[yar_d]--; break;
            case 2: s->ar[yar_d]++; break;
            case 3: s->ar[yar_d] = c54x_circ_ref(s->ar[yar_d], +(int16_t)s->ar[0], s->bk); break;
            }
            /* The dual family multiplies its TWO MEMORY operands together, not T
             * by Xmem. binutils:
             *   0xa400-0xa5ff mpy   X,Y,dst       dst = X*Y      (assignment)
             *   0xa600-0xa7ff macsu X,Y,src1      src += u(X)*s(Y)
             *   0xb000-0xb3ff mac   X,Y,src,dst   dst = src + X*Y
             *   0xb400-0xb7ff macr  same, rounded
             * `T * Xmem ; T = Ymem` is the SINGLE-operand MAC semantics
             * (MAC Smem,src -> src += T*Smem). The C54x side effect here is
             * T <- Xmem, not Ymem.
             * [2026-08-23] Measured in the SCH coefficient loop (0x81f3 LD #0,A;
             * RPT; MAC *AR2+,*AR4+; RPT; MAC *AR3+,*AR5+; 0x8202 STH A): at
             * pc=0x81f5 op=0xb08a, T was 0x0000 at the first MAC, so the product
             * was zero, A stayed zero, STH wrote 0 and 810 zeros landed in
             * 0x2cba..0x2cbf, which MVDD copied on as null FIRS coefficients --
             * while the inputs were sane.
             * ⚠️ Global effect: this is the backbone of all signal processing. */
            {
                int is_mpy   = (hi8 == 0xA4 || hi8 == 0xA5);
                int is_macsu = (hi8 == 0xA6 || hi8 == 0xA7);
                int is_macr  = (hi8 >= 0xB4 && hi8 <= 0xB7);
                int64_t p = is_macsu
                    ? (int64_t)(uint16_t)xval_d * (int64_t)(int16_t)yval_d
                    : (int64_t)(int16_t)xval_d * (int64_t)(int16_t)yval_d;
                if (s->st1 & ST1_FRCT) p <<= 1;
                if (is_macr) p = (p + 0x8000) & ~0xFFFFLL;   /* [2026-09-21] rnd() clears bits 15-0 */
                int dstb, srcb;
                if (is_mpy || is_macsu) {          /* mask 0xFE00: bit 8 = acc */
                    dstb = hi8 & 1; srcb = dstb;
                } else {                            /* mask 0xFC00: b9 = src, b8 = dst */
                    srcb = (op >> 9) & 1; dstb = (op >> 8) & 1;
                }
                int64_t base = srcb ? s->b : s->a;
                int64_t res  = is_mpy ? p : (base + p);
                if (is_macr) res &= ~(int64_t)0xFFFF;   /* rnd(): +0x8000 then clear the low half */
                if (dstb) s->b = sext40(res);
                else      s->a = sext40(res);
                s->t = xval_d;                      /* C54x side effect: T <- Xmem */
                return consumed + s->lk_used;
            }
        }

        /* [2026-09-21] 0xA1xx is NOT SQDST. SQDST Xmem,Ymem is 0xE2xx (binutils
         * { "sqdst", 0xE200, 0xFF00 }); 0xA000-0xA1FF is ADD Xmem,Ymem,dst with
         * bit 8 = dst, so every `add Xmem,Ymem,B` (a1xx) was run as SQDST:
         * A <- Ymem<<16 (clobbered), B += (AH-Xmem)^2, T <- Xmem.
         * Measured on the NB equaliser (PROM0 0x8251 `a189 add *AR2+,*AR3+,B`,
         * 206 times per burst): the 3-tap sum just built in A was replaced by
         * the derotated Q sample, so the "soft bits" were the raw Q plane
         * (clean but one symbol early and inverted), the TSC residual check
         * saw no correlation (count 6) and the quantiser output +1 everywhere.
         * Also hits 0x8565, 0x81fa, 0x7f03, 0x7fab-0x80ec (tap threshold /
         * SB path). The block is kept for reference, never entered. */
        if (0 && hi8 == 0xA1) {
            /* 2-bit Xmem/Ymem per SPRU131G tables 5-6/5-8. */
            int xar_sq  = ((op >> 4) & 0x03) + 2;
            int yar_sq  = (op & 0x03) + 2;
            int xmod_sq = (op >> 6) & 0x03;
            int ymod_sq = (op >> 2) & 0x03;
            uint16_t xval_sq = data_read(s, s->ar[xar_sq]);
            uint16_t yval_sq = data_read(s, s->ar[yar_sq]);
            switch (xmod_sq) { case 1: s->ar[xar_sq]--; break; case 2: s->ar[xar_sq]++; break;
                case 3: s->ar[xar_sq] = c54x_circ_ref(s->ar[xar_sq], +(int16_t)s->ar[0], s->bk); break; }
            switch (ymod_sq) { case 1: s->ar[yar_sq]--; break; case 2: s->ar[yar_sq]++; break;
                case 3: s->ar[yar_sq] = c54x_circ_ref(s->ar[yar_sq], +(int16_t)s->ar[0], s->bk); break; }
            int16_t ah_sq = (int16_t)((s->a >> 16) & 0xFFFF);
            int32_t diff = (int32_t)ah_sq - (int32_t)(int16_t)xval_sq;
            int64_t sq = (int64_t)diff * (int64_t)diff;
            if (s->st1 & ST1_FRCT) sq <<= 1;
            s->b = sext40(s->b + sq);
            s->a = sext40((int64_t)(int16_t)yval_sq << 16);
            s->t = xval_sq;
            return consumed + s->lk_used;
        }

        /* POLY Xmem, Ymem — Polynomial evaluation (1-word dual-operand)
         * Encoding: 1011 110D XXXX YYYY (0xBC=A, 0xBD=B)
         *           1011 111D XXXX YYYY (0xBE/0xBF variants — ABDST or POLY)
         * Per SPRU172C: B += AH * T (with round); A = Xmem << 16; T = Ymem */
        if (hi8 == 0xBC || hi8 == 0xBD || hi8 == 0xBE || hi8 == 0xBF) {
            /* 2-bit Xmem/Ymem decoding per SPRU131G tables 5-6/5-8. */
            int xar_p  = ((op >> 4) & 0x03) + 2;
            int yar_p  = (op & 0x03) + 2;
            int xmod_p = (op >> 6) & 0x03;
            int ymod_p = (op >> 2) & 0x03;
            uint16_t xval_p = data_read(s, s->ar[xar_p]);
            uint16_t yval_p = data_read(s, s->ar[yar_p]);
            switch (xmod_p) { case 1: s->ar[xar_p]--; break; case 2: s->ar[xar_p]++; break;
                case 3: s->ar[xar_p] = c54x_circ_ref(s->ar[xar_p], +(int16_t)s->ar[0], s->bk); break; }
            switch (ymod_p) { case 1: s->ar[yar_p]--; break; case 2: s->ar[yar_p]++; break;
                case 3: s->ar[yar_p] = c54x_circ_ref(s->ar[yar_p], +(int16_t)s->ar[0], s->bk); break; }
            /* 0xBC00-0xBFFF is dual MASR, not POLY. binutils:
             *   { "masr", 0xBC00, 0xFC00, {OP_Xmem,OP_Ymem,OP_SRC,OP_DST} }
             *   { "poly", 0x3600, 0xFF00, {OP_Smem} }   <- POLY lives elsewhere
             * SPRU172C: dst = rnd(src - Xmem*Ymem) AND T = Xmem.
             * ⚠️ Running a POLY here (B += rnd(AH*T); A = Xmem<<16; T = Ymem) picks
             * the wrong accumulator, the wrong product and the wrong T, at 119
             * sites. */
            {
                int64_t pr = (int64_t)(int16_t)xval_p * (int64_t)(int16_t)yval_p;
                if (s->st1 & ST1_FRCT) pr <<= 1;
                pr = (pr + 0x8000) & ~0xFFFFLL;     /* round: bits 15-0 cleared [2026-09-21] */
                int srcr = (op >> 9) & 1, dstr = (op >> 8) & 1;
                int64_t baser = srcr ? s->b : s->a;
                if (dstr) s->b = sext40(baser - pr);
                else      s->a = sext40(baser - pr);
                s->t = xval_p;
                return consumed + s->lk_used;
            }
        }

        /* B8-BB: MAS/MASR Xmem, Ymem (subtract variants) or POLY-like */
        if (hi8 == 0xB8 || hi8 == 0xB9 || hi8 == 0xBA || hi8 == 0xBB) {
            /* 0xBA is LDMM, handled below. */
            if (hi8 == 0xBA) goto ba_handler;
            /* 2-bit Xmem/Ymem decoding per SPRU131G tables 5-6/5-8. */
            int xar_b8  = ((op >> 4) & 0x03) + 2;
            int yar_b8  = (op & 0x03) + 2;
            int xmod_b8 = (op >> 6) & 0x03;
            int ymod_b8 = (op >> 2) & 0x03;
            uint16_t xval_b8 = data_read(s, s->ar[xar_b8]);
            uint16_t yval_b8 = data_read(s, s->ar[yar_b8]);
            switch (xmod_b8) { case 1: s->ar[xar_b8]--; break; case 2: s->ar[xar_b8]++; break;
                case 3: s->ar[xar_b8] = c54x_circ_ref(s->ar[xar_b8], +(int16_t)s->ar[0], s->bk); break; }
            switch (ymod_b8) { case 1: s->ar[yar_b8]--; break; case 2: s->ar[yar_b8]++; break;
                case 3: s->ar[yar_b8] = c54x_circ_ref(s->ar[yar_b8], +(int16_t)s->ar[0], s->bk); break; }
            /* Dual MAS: binutils { "mas", 0xB800, 0xFC00,
             * {OP_Xmem,OP_Ymem,OP_SRC,OP_DST} }, SPRU172C:
             *     dst = src - Xmem*Ymem   AND   T = Xmem
             * ⚠️ Computing `T * Xmem` then `T = Ymem` is the SINGLE-operand MAS
             * semantics applied to the dual form, the same defect already fixed for
             * mac/mpy; 122 sites. */
            int64_t prod_b8 = (int64_t)(int16_t)xval_b8 * (int64_t)(int16_t)yval_b8;
            if (s->st1 & ST1_FRCT) prod_b8 <<= 1;
            int src_b8 = (op >> 9) & 1;
            int dst_b8 = (op >> 8) & 1;
            int64_t base_b8 = src_b8 ? s->b : s->a;
            if (dst_b8) s->b = sext40(base_b8 - prod_b8);
            else        s->a = sext40(base_b8 - prod_b8);
            s->t = xval_b8;
            return consumed + s->lk_used;
        }
ba_handler:
        if (hi8 == 0xAA || hi8 == 0xAB) {
            /* 0xAA/0xAB belong to the parallel LD||MACR family, and are NOT
             * STLM src,MMR (that is 0x88/0x89). One word, no effect. This stub
             * sits before the LD||MAC handler below and shadows it for those two
             * opcodes. */
            return 1;
        }
        if (hi8 == 0xBA) {
            /* LDMM MMR, dst — load the MMR value into the accumulator.
             * Per SPRU172C: dst[15:0] = MMR, dst[31:16] = sign extension (SXM) or 0.
             * ⚠️ `sext40(v << 16)` puts the MMR in the wrong half: the boot stub at
             * 0x0000 (LDMM SP,B) must return B = the current SP, and with the shift
             * B[15:0] is 0 for every caller that reads B as a 16-bit SP. */
            uint16_t mmr = op & 0x7F;
            int dst = (op >> 4) & 1;
            int64_t v = (int64_t)(int16_t)data_read(s, mmr);
            if (dst) s->b = sext40(v);
            else     s->a = sext40(v);
            return consumed + s->lk_used;
        }
        /* Parallel LD||MAC family: 0xA800-0xAFFF, ONE word.
         * binutils (tic54x_paroptab):
         *   0xA800/0xFE00  ld Xmem,dst || mac  Ymem
         *   0xAA00/0xFE00  ld Xmem,dst || macr Ymem
         *   0xAC00/0xFE00  ld Xmem,dst || mas  Ymem
         *   0xAE00/0xFE00  ld Xmem,dst || masr Ymem
         * SPRU172C: dst = Xmem << 16; the OTHER accumulator gets +/- T*Ymem
         * (rounded for the R variants); T IS UNCHANGED. bit 8 selects dst.
         *
         * ⚠️ The wrong decodings all cost a word: 0xA8/0xA9 as a 2-word `AND #lk`
         * (the real one is 0xF030), 0xAC/0xAD as a 2-word `MACP Smem,pmad` (really
         * 0x7800), 0xAE/0xAF as a 2-word `MACD Smem,pmad` (really 0x7A00, and it
         * also wrote T). Those three swallow one word too many at each of 233
         * sites; 0xAA/0xAB as a silent stub is the right length but does nothing,
         * at 193 sites. */
        if ((op & 0xF800) == 0xA800) {
            int xar_l  = ((op >> 4) & 0x03) + 2;
            int yar_l  = ( op       & 0x03) + 2;
            int xmod_l = (op >> 6) & 0x03;
            int ymod_l = (op >> 2) & 0x03;
            uint16_t xv_l = data_read(s, s->ar[xar_l]);
            uint16_t yv_l = data_read(s, s->ar[yar_l]);
            int dst_l  = (op >> 8) & 1;              /* accumulator of the LD   */
            int soust  = ((op >> 10) & 1);           /* 0xAC/0xAE = subtract    */
            int arrondi= ((op >> 9)  & 1);           /* 0xAA/0xAE = round       */
            int64_t p_l = (int64_t)(int16_t)s->t * (int64_t)(int16_t)yv_l;
            if (s->st1 & ST1_FRCT) p_l <<= 1;
            if (arrondi) p_l = (p_l + 0x8000) & ~0xFFFFLL;   /* [2026-09-21] rnd() clears bits 15-0 */
            int64_t *acc_ld = dst_l ? &s->b : &s->a;   /* receives the LD  */
            int64_t *acc_op = dst_l ? &s->a : &s->b;   /* receives the MAC */
            *acc_op = sext40(soust ? (*acc_op - p_l) : (*acc_op + p_l));
            *acc_ld = sext40((int64_t)(int16_t)xv_l << 16);
            c54x_par_postmod(s, xar_l, xmod_l);
            c54x_par_postmod(s, yar_l, ymod_l);
            return consumed + s->lk_used;            /* one word, T unchanged */
        }
        if (hi8 == 0xA8 || hi8 == 0xA9) {
            /* A8xx/A9xx: AND #lk, src[, dst] (2-word) */
            op2 = prog_fetch(s, s->pc + 1);
            consumed = 2;
            int dst = op & 1;
            int64_t *acc = dst ? &s->b : &s->a;
            *acc = sext40(*acc & ((int64_t)op2 << 16));
            return consumed + s->lk_used;
        }
        /* 0xA000-0xA1FF is one instruction. binutils has a single entry on that
         * range:
         *     { "add", 1,3,3, 0xA000, 0xFE00, {OP_Xmem,OP_Ymem,OP_DST} }
         *   ADD Xmem, Ymem, dst  ->  dst = (Xmem << 16) + (Ymem << 16)
         * ⚠️ The accumulator operations it is easily confused with all live in
         * 0xF0xx-0xF4xx (ld 0xF482, neg 0xF484, abs 0xF485, sat 0xF483,
         * sfta 0xF460, sftl 0xF0E0); none of them is at 0xA0.
         *
         * [2026-08-22] Measured on the last instruction of the correlator loop
         * body (RPTB 0x84b0, REA=0x84c6):
         *     0x84c6 : 0xA09A = add *AR3+, *AR4+, A
         * Low byte 0x9A gives Xar=AR3 (*AR+) and Yar=AR4 (*AR+), the correlator's
         * two INPUT pointers (0x2a27 and 0x2ae7 in the burst buffer); this is the
         * instruction that slides the window by one word per lag. Decoded as an
         * accumulator op it became a 77-bit SFTL, no pointer advanced, all 50 lags
         * correlated the same window (distinct = 3/50 and 5/50), the argmax became
         * meaningless and the SCH never decoded. gr-gsm confirms the semantics:
         * the reference restarts at each lag and the input slides.
         * ⚠️ Global effect: ADD Xmem,Ymem,dst is not used only by the correlator. */
        if ((op & 0xFE00) == 0xA000) {
            int xmod = (op >> 6) & 0x03;
            int xar  = ((op >> 4) & 0x03) + 2;
            int ymod = (op >> 2) & 0x03;
            int yar  = ( op       & 0x03) + 2;
            int64_t *dst = ((op >> 8) & 1) ? &s->b : &s->a;
            int16_t xv = (int16_t)data_read(s, s->ar[xar]);
            int16_t yv = (int16_t)data_read(s, s->ar[yar]);
            *dst = sext40(((int64_t)xv + (int64_t)yv) << 16);
            c54x_par_postmod(s, xar, xmod);
            c54x_par_postmod(s, yar, ymod);
            return consumed + s->lk_used;
        }
        if (hi8 == 0xA5) {
            /* CMPS src, Smem — compare and select (Viterbi) */
            addr = resolve_smem(s, op, &ind);
            uint16_t val = data_read(s, addr);
            int src = (op >> 4) & 1;
            int64_t acc = src ? s->b : s->a;
            int64_t cmp = (int64_t)(int16_t)val << 16;
            /* TRN shift left, TC set based on comparison */
            s->trn <<= 1;
            if (acc >= cmp) {
                s->st0 |= ST0_TC;
                s->trn |= 1;
            } else {
                s->st0 &= ~ST0_TC;
                if (src) s->b = cmp; else s->a = cmp;
            }
            return consumed + s->lk_used;
        }
        /* AExx/AFxx: MACD Smem, pmad, dst — MAC + data move (2 words)
         * dst += T * Smem, then data(Smem) → data(dmad)
         * pmad in second word auto-increments during RPT */
        if (hi8 == 0xAE || hi8 == 0xAF) {
            addr = resolve_smem(s, op, &ind);
            op2 = prog_fetch(s, s->pc + 1);
            consumed = 2;
            uint16_t sval = data_read(s, addr);
            int64_t prod = (int64_t)(int16_t)s->t * (int64_t)(int16_t)sval;
            if (s->st1 & ST1_FRCT) prod <<= 1;
            int dst = (hi8 & 0x01);
            if (dst) s->b = sext40(s->b + prod);
            else     s->a = sext40(s->a + prod);
            /* Data move: read from prog[pmad], write to data[addr] */
            uint16_t psrc = s->rpt_active ? s->mvpd_src : op2;
            data_write(s, addr, prog_fetch(s, psrc));
            s->mvpd_src = psrc + 1;
            s->t = sval;  /* T = old Smem value (before overwrite) */
            return consumed + s->lk_used;
        }
        /* ACxx/ADxx: MACP Smem, pmad, dst — MAC + program fetch (2 words) */
        if (hi8 == 0xAC || hi8 == 0xAD) {
            addr = resolve_smem(s, op, &ind);
            op2 = prog_fetch(s, s->pc + 1);
            consumed = 2;
            uint16_t sval = data_read(s, addr);
            int64_t prod = (int64_t)(int16_t)s->t * (int64_t)(int16_t)sval;
            if (s->st1 & ST1_FRCT) prod <<= 1;
            int dst = (hi8 & 0x01);
            if (dst) s->b = sext40(s->b + prod);
            else     s->a = sext40(s->a + prod);
            /* Coeff fetch from program memory */
            uint16_t psrc = s->rpt_active ? s->mvpd_src : op2;
            s->t = prog_fetch(s, psrc);
            s->mvpd_src = psrc + 1;
            return consumed + s->lk_used;
        }
        /* ⚠️ 0xB3 is MACR Xmem,Ymem,B (1 word, handled with the MAC family above),
         * not a 2-word `LD #lk,dst`: the extra word drifts PC inside the RPTBD
         * routine at 0x820e..0x820f and loops forever at 0x821a. */
        /* ADD #lk, src[, dst] */
        if (hi8 == 0xA2) {
            op2 = prog_fetch(s, s->pc + 1);
            consumed = 2;
            int dst = op & 1;
            int64_t v = (s->st1 & ST1_SXM) ? (int16_t)op2 : op2;
            if (dst) s->b = sext40(s->b + (v << 16));
            else     s->a = sext40(s->a + (v << 16));
            return consumed + s->lk_used;
        }
        /* SUB #lk */
        if (hi8 == 0xA3) {
            op2 = prog_fetch(s, s->pc + 1);
            consumed = 2;
            int dst = op & 1;
            int64_t v = (s->st1 & ST1_SXM) ? (int16_t)op2 : op2;
            if (dst) s->b = sext40(s->b - (v << 16));
            else     s->a = sext40(s->a - (v << 16));
            return consumed + s->lk_used;
        }
        goto unimpl;

    case 0xC: case 0xD:
        /* ================================================================
         * Parallel family: ST src,Ymem || <op> Xmem,dst (ONE word,
         * SPRU172C 4-177..4-185).
         *
         * The whole 0xC000-0xDFFF range is ONE family: binutils has four "st"
         * entries of mask 0xFC00 plus those at 0xD000/D400/D800/DC00, all flagged
         * FL_PAR. ⚠️ Splitting it into unrelated instructions costs words, because
         * some of the impostors are 2 words: RPTB is really 0xF072, RPTBD 0xF272,
         * PSHD 0x4B00, RPT Smem 0x4700, SACCD 0x9E00, DELAY 0x4D00 -- none of them
         * lives here.
         *
         * [2026-08-22] Measured: the FB/SB correlator kernel (PDROM
         * 0xf16c..0xf17e, `rptbd 0xf178` loop) is built on this family --
         *     0xf172 c780 = ST||SUB    0xf173 ce91 = ST||MPY
         *     0xf176 c709 = ST||SUB    0xf178 ce98 = ST||MPY
         * Decoded as 2-word RPTB they swallowed the next word AND overwrote
         * rea/rsa/rptb_active/BRAF of the live rptbd loop containing them, so the
         * correlation surface was computed from a different program than the
         * silicon runs: false peak, false argmax, a_sync_demod[D_TOA] an arbitrary
         * word (TOA = -7791 / -3691 / +8026 / -28903, outside 0..156) and
         * d_fb_det = 1 about once in 95.
         *
         * Semantics (SPRU172C, encoding `1100xxSD`/`1101RSD` XXXXYYYY):
         *   store : Ymem = (src << ASM) >> 16          (src = b9, NOT b8)
         *   op    : C0-C3 dst += Xmem<<16 | C4-C7 dst = Xmem<<16 - dst
         *           C8-CB dst  = Xmem<<16 | CC-CF dst = T*Xmem
         *           D0-D7 dst += T*Xmem   | D8-DF dst -= T*Xmem  (R = b10, round)
         *   Xmem is read BEFORE the store ("If src is equal to dst, the value
         *   stored in Ymem is the value of src before the execution").
         *   This family NEVER writes T.
         *
         * ⚠️ Global effect: the family also serves the filters, the Viterbi and the
         * I/Q buffers.
         * ================================================================ */
        {
            int s_acc = (op >> 9) & 1;        /* S = b9: source acc of the ST  */
            int d_acc = (op >> 8) & 1;        /* D = b8: destination of the op */
            int xmod  = (op >> 6) & 3;
            int xar   = ((op >> 4) & 3) + 2;  /* Xmem : AR2..AR5 */
            int ymod  = (op >> 2) & 3;
            int yar   = ( op       & 3) + 2;  /* Ymem : AR2..AR5 */
            uint16_t xaddr = s->ar[xar];
            uint16_t yaddr = s->ar[yar];
            uint16_t xval  = data_read(s, xaddr);   /* read BEFORE the store */
            int64_t  sv    = s_acc ? s->b : s->a;
            int64_t *dstp  = d_acc ? &s->b : &s->a;
            int      ash   = asm_shift(s);
            int64_t  sh    = (ash >= 0) ? (sv << ash) : (sv >> (-ash));
            int64_t  xs    = (int64_t)(int16_t)xval;
            int64_t  prod;

            /* ST src,Ymem : Ymem = (src << ASM) >> 16 */
            data_write(s, yaddr, (uint16_t)((sh >> 16) & 0xFFFF));

            switch ((op >> 10) & 7) {         /* bits 12:10 = sub-class */
            /* FIX_PAR_ST_DSTBAR — per SPRU172C 4-177/4-185,
             * "|| ADD Xmem, dst : dst = dst_ + Xmem << 16" and
             * "|| SUB Xmem, dst : dst = (Xmem << 16) - dst_", where dst_ is the
             * OTHER accumulator (dst = A means dst_ = B), not dst itself.
             * The SB demodulator (0x76e4/0x76f5 `c0dc`) uses it 896 times per
             * burst. */
            case 0: { /* C0-C3 : || ADD Xmem,dst */
                int64_t other = d_acc ? s->a : s->b;
                *dstp = sext40(other + (xs << 16));
                break; }
            case 1: { /* C4-C7 : || SUB Xmem,dst  ->  Xmem<<16 - dst_ */
                int64_t other = d_acc ? s->a : s->b;
                *dstp = sext40((xs << 16) - other);
                break; }
            case 2:  /* C8-CB : || LD Xmem,dst */
                *dstp = sext40(xs << 16);
                break;
            case 3:  /* CC-CF : || MPY Xmem,dst */
                prod = (int64_t)(int16_t)s->t * xs;
                if (s->st1 & ST1_FRCT) prod <<= 1;
                *dstp = sext40(prod);
                break;
            default: /* D0-D7 MAC[R] (b11=0) , D8-DF MAS[R] (b11=1) */
                prod = (int64_t)(int16_t)s->t * xs;
                if (s->st1 & ST1_FRCT) prod <<= 1;
                if (op & 0x0400) {            /* R = b10: round */
                    prod += 0x8000;
                    prod &= ~0xFFFFLL;
                }
                *dstp = sext40((op & 0x0800) ? (*dstp - prod)
                                             : (*dstp + prod));
                break;
            }
            /* T is NOT modified by this family. */
            c54x_par_postmod(s, xar, xmod);
            c54x_par_postmod(s, yar, ymod);
            return consumed + s->lk_used;
        }

    default:
        break;
    }

unimpl:
    s->unimpl_count++;
    if (s->unimpl_count <= 200 || op != s->last_unimpl) {
        C54_LOG("UNIMPL @0x%04x: 0x%04x (hi8=0x%02x) [#%u]",
                s->pc, op, hi8, s->unimpl_count);
        s->last_unimpl = op;
    }
    return consumed + s->lk_used;
}

/* The main execution loop and the DSP idle fast-forward
 * (CALYPSO_DSP_IDLE_FF, CALYPSO_DSP_IDLE_RANGE) live in calypso_c54x.c. */
