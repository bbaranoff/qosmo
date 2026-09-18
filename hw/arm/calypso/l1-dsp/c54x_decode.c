/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * c54x_decode.c — Decodage des operandes : resolve_smem/lmem/xmem, conditions
 *
 * Extrait de calypso_c54x.c le 2026-09-18 (decoupage par role).
 * Carte des fichiers dans c54x_internal.h.
 */
#include "c54x_internal.h"

uint16_t resolve_smem(C54xState *s, uint16_t opcode, bool *indirect)
{
    if (opcode & 0x80) {
        /* Indirect addressing.
         * Per SPRU131G §5.4.1 Table 5-5: bits 2:0 = ARF select the AR for
         * THIS instruction. ARP (in ST0) is then updated to ARF for the
         * NEXT direct-Smem reference. Earlier this code used arp(s) for
         * cur_arp, which made every indirect insn operate on the
         * PREVIOUS insn's ARF — off-by-one. Symptoms: BANZD *AR1- after
         * STL *AR2+ would decrement AR2 instead of AR1 (BANZD test
         * against AR2 stayed non-zero forever, AR1 frozen). Diagnosed
         * via 5×500M-insn STATE-DUMP showing AR1=0x1c / AR2=0x2b0c
         * frozen across 2B insns at PC=0xa2c2..0xa2ca. */
        *indirect = true;
        int mod = (opcode >> 3) & 0x0F;
        int nar = opcode & 0x07;
        int cur_arp = nar;
        uint16_t addr = s->ar[cur_arp];
        uint16_t ar_before = s->ar[cur_arp];  /* MOD-MISMATCH probe : base avant post-modify */

        /* PROBE 2026-05-31 convergence modes : 1er usage de chaque AR comme base
         * d'adresse. Si la valeur == reset (AR0=0xff75/0x5aad, AR6=0/0xbae6,
         * AR7=0/0x1e44) → read-before-write → reset load-bearing = driver de la
         * divergence bin/c54x. À RETIRER. */
        {
            static uint8_t ar_used = 0;
            if (!(ar_used & (1 << cur_arp))) {
                ar_used |= (1 << cur_arp);
                fprintf(stderr, "[c54x] AR-FIRSTUSE AR%d=0x%04x PC=0x%04x insn=%u\n",
                        cur_arp, addr, s->pc, s->insn_count);
            }
        }

        /* AR2-FLOOR guard : le pointeur d'écriture corrélateur (AR2) peut
         * sous-déborder le buffer DARAM (0x0800) jusqu'à l'espace MMR
         * (0x1E=XPC, 0x00=IMR) → clobber. WARN-log diag (token AR2-FLOOR) ;
         * DROP expérimental (env CALYPSO_AR2_FLOOR_DROP=1) redirige l'accès
         * vers un scratch pour voir si le corrélateur converge sans le crash. */
        if (cur_arp == 2 && addr < 0x0820) {
            /* @BEQUILLE — AR2_FLOOR_DROP  (CALYPSO_AR2_FLOOR_DROP, EQ1, defaut OFF)
             *   masque  : le calcul d'adresse d'AR2 dans le correlateur, qui sous-deborde le
             *             buffer DARAM 0x0800 jusqu'a l'espace MMR (0x00=IMR, 0x1E=XPC) et le
             *             clobbe. Le drop redirige l'acces vers 0xFFFF au lieu de corriger le
             *             pointeur. NB : le LOG est gate par le jeton DEBUG=AR2-FLOOR, le DROP
             *             ne l'est PAS.
             *   retirer : quand AR2 reste dans [0x0800,0x2b28) sur tout le kernel FB
             *             (compteur du jeton AR2-FLOOR a 0 sur un run complet).
             */
            static int ar2_drop = -1;
            if (ar2_drop < 0) {
                const char *e = getenv("CALYPSO_AR2_FLOOR_DROP");
                ar2_drop = (e && *e == '1') ? 1 : 0;  /* env-gated, OFF par défaut */
            }
            if (calypso_debug_enabled("AR2-FLOOR"))
                C54_DBG("AR2-FLOOR",
                    "AR2=0x%04x < floor PC=0x%04x op=0x%04x BK=0x%04x A=%010llx insn=%u",
                    addr, s->pc, prog_fetch(s, s->pc), s->bk,
                    (unsigned long long)(s->a & 0xFFFFFFFFFFULL), s->insn_count);
            if (ar2_drop && addr < 0x0800)
                addr = 0xFFFF;  /* scratch : empêche le clobber MMR (expérience) */
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
        /* MOD 4-11 : encodage canonique C54x (tic54x-dis.c:506-518, vérifié
         * cross-run via sonde MOD-MISMATCH 2026-06-01 : QEMU divergeait sur
         * 5/6/9/10/11 — signe inversé 5/6, mauvais op 9/10, wrap absent 11).
         * Ordre réel : 4=-0B 5=-0 6=+0 7=+0B 8=-% 9=-0% 10=+% 11=+0%.
         * Circulaire (8-11) via c54x_circ_ref → BK=0 reste LINÉAIRE (règle
         * #6396 : STM #0,BK délibéré, confirmé par sonde BK-WR). */
        case 0x4: /* *ARn-0B — retenue INVERSEE (fix 2026-08-22) */
            s->ar[cur_arp] = c54x_revcarry(s, s->ar[cur_arp], s->ar[0], 1);
            break;
        case 0x5: /* *ARn-0 */
            s->ar[cur_arp] -= s->ar[0];
            break;
        case 0x6: /* *ARn+0 */
            s->ar[cur_arp] += s->ar[0];
            break;
        case 0x7: /* *ARn+0B — retenue INVERSEE (fix 2026-08-22) */
            s->ar[cur_arp] = c54x_revcarry(s, s->ar[cur_arp], s->ar[0], 0);
            break;
        /* GAP bitrev LEVE le 2026-08-22 : la sonde demandee est PDROM 0xf1b3
         * `mar *AR2+0B`, dans le correlateur FB/SB. Modes 4/7 = retenue
         * inversee (c54x_revcarry). Gate CALYPSO_ISA_BITREV=0 pour revenir
         * au +/-AR0 plat. */
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
         *   12 : *AR(x)(lk)        — addr = AR(x) + lk, NO modify
         *   13 : *+AR(x)(lk)       — premod: AR(x) += lk; addr = AR(x)
         *   14 : *+AR(x)(lk)%      — premod circular: AR(x) = circ(AR(x)+lk)
         *   15 : *(lk)             — ABSOLUTE long address (lk itself)
         *
         * The bootloader at PROM0 0xb429 uses MOD=15 (`LDU *(0x0ffe), A`)
         * to read BL_ADDR_LO. Misdecoding 15 as "AR + lk circular"
         * produced AR0+0x0ffe instead of 0x0ffe — one of the multiple
         * subtle off-by-AR bugs that left A=0 after the load. */
        case 0xC: /* *AR(x)(lk) */
            addr = s->ar[cur_arp] + prog_fetch(s, s->pc + 1);
            s->lk_used = true;
            break;
        case 0xD: /* *+AR(x)(lk) */
            s->ar[cur_arp] += prog_fetch(s, s->pc + 1);
            addr = s->ar[cur_arp];
            s->lk_used = true;
            break;
        case 0xE: { /* *+AR(x)(lk)% — circular */
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
        case 0xF: /* *(lk) — absolute address */
            addr = prog_fetch(s, s->pc + 1);
            s->lk_used = true;
            break;
        }

        /* PROBE 2026-06-01 MOD-MISMATCH : delta silicium-correct EN PARALLÈLE
         * (n'altère PAS l'exécution — pur compare). Confirme sur le flux réel
         * que seuls mods 5/6/9/10/11 divergent ET que la firmware les touche.
         * Réf : tic54x-dis.c:506-518 (MOD canonique), macros tic54x.h:97-98
         * identiques à l'extraction QEMU. À RETIRER après validation du patch. */
        {
            int16_t  a0  = (int16_t)s->ar[0];
            uint16_t bk  = s->bk;
            uint16_t sil;               /* AR attendu côté silicium */
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
            /* quels mods la firmware touche (1er hit chacun) */
            static uint16_t mod_seen = 0;
            if (!(mod_seen & (1u << mod))) {
                mod_seen |= (1u << mod);
                fprintf(stderr, "[c54x] MOD-FIRSTHIT mod=%2d AR%d PC=0x%04x op=0x%04x insn=%u\n",
                        mod, cur_arp, s->pc, opcode, s->insn_count);
            }
            /* divergence silicium vs QEMU (modes 0..11 seulement) */
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

        /* Update ARP */
        s->st0 = (s->st0 & ~ST0_ARP_MASK) | (nar << ST0_ARP_SHIFT);

        return addr;
    } else {
        /* Direct addressing: DP:offset */
        *indirect = false;
        uint16_t offset = opcode & 0x7F;
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
    if (!(opcode & 0x80)) {
        /* Direct (DP-relative) — dmad pair, no post-mod. */
        uint16_t dp = s->st0 & ST0_DP_MASK;
        return (uint16_t)(((dp << 7) | (opcode & 0x7F)) & 0xFFFE);
    }
    int mod = (opcode >> 3) & 0x0F;
    int nar = opcode & 0x07;
    uint16_t addr = s->ar[nar] & 0xFFFE;
    switch (mod) {
    case 0x0: break;                                              /* *ARn      */
    case 0x1: s->ar[nar] -= 2; break;                             /* *ARn-     */
    case 0x2: s->ar[nar] += 2; break;                             /* *ARn+     */
    case 0x3: s->ar[nar] += 2; addr = s->ar[nar] & 0xFFFE; break; /* *+ARn     */
    case 0x4: s->ar[nar] = c54x_revcarry(s, s->ar[nar], s->ar[0], 1); break; /* *ARn-0B */
    case 0x5: s->ar[nar] -= s->ar[0]; break;                                 /* *ARn-0  */
    case 0x6: s->ar[nar] += s->ar[0]; break;                                 /* *ARn+0  */
    case 0x7: s->ar[nar] = c54x_revcarry(s, s->ar[nar], s->ar[0], 0); break; /* *ARn+0B */
    case 0x8: s->ar[nar] = c54x_circ_ref(s->ar[nar], -2, s->bk); break;                 /* *ARn-%  */
    case 0x9: s->ar[nar] = c54x_circ_ref(s->ar[nar], -(int16_t)s->ar[0], s->bk); break; /* *ARn-0% */
    case 0xA: s->ar[nar] = c54x_circ_ref(s->ar[nar], +2, s->bk); break;                 /* *ARn+%  */
    case 0xB: s->ar[nar] = c54x_circ_ref(s->ar[nar], +(int16_t)s->ar[0], s->bk); break; /* *ARn+0% */
    case 0xC: addr = (uint16_t)((s->ar[nar] + prog_fetch(s, s->pc + 1)) & 0xFFFE); s->lk_used = true; break;
    case 0xD: s->ar[nar] += prog_fetch(s, s->pc + 1); addr = s->ar[nar] & 0xFFFE; s->lk_used = true; break;
    case 0xE: { uint16_t lk = prog_fetch(s, s->pc + 1);
                s->ar[nar] = c54x_circ_ref(s->ar[nar], (int16_t)lk, s->bk);
                addr = s->ar[nar] & 0xFFFE; s->lk_used = true; break; }
    case 0xF: addr = (uint16_t)(prog_fetch(s, s->pc + 1) & 0xFFFE); s->lk_used = true; break;
    }
    return addr;
}

/* SP ledger for IRQ-asymmetry diag (web 2026-05-23).
 * Pushes/pops counted by SP delta sign in dispatch loop (c54x_run).
 * IRQ entries counted explicitly in c54x_interrupt_ex with word count.
 * Periodic dump in dispatch loop shows whether net_words ≈ 0 (balanced)
 * or drifts (indicates push/pop word-count asymmetry, e.g. IRQ entry
 * pushes 1 word but FRET pops 2 → drift -1/IRQ-cycle → SP wraps). */
struct c54x_g_sp_ledger_s g_sp_ledger;

/* Xmem operand decode per binutils tic54x.h (XMEM/XMOD/XARX macros) :
 *   XMEM(OP) = bits [7:4] of opcode (the Xmem 4-bit nibble)
 *   XMOD    = nibble bits [3:2] : 0=*AR, 1=*AR-, 2=*AR+, 3=*AR+0%
 *   XARX    = nibble bits [1:0] + 2 (= AR2..AR5 only, no AR0/AR1/AR6/AR7)
 *
 * Xmem is INDIRECT-ONLY (no DP-relative direct mode, unlike Smem). Using
 * resolve_smem on an Xmem operand mis-decodes the low byte as Smem direct
 * addressing whenever bit 7 is clear, which lands writes in MMR space
 * (0x00-0x1F) — empirically observed at PC=0x8a46 op=0x9918 (STL B,*AR2)
 * 2026-05-23, stomp SP=0x4800→0x0000 cascading to IMR=0 → DSP idle forever.
 *
 * Fix 2026-06-01 : xmod=3 (*AR+0%) désormais CIRCULAIRE modulo BK via
 * c54x_circ_ref (BK=0→linéaire, règle #6396). Appliqué à tous les handlers
 * duaux (resolve_xmem, MVDD, MAC D0-D9, MASA DB, SQDST DC) — était linéaire
 * `addr + AR0` → drift 16-bit (runaway AR2 @0xfa98, op 0xd3dc Ymem *AR2+0%).
 * Cohérent avec le handler ST||LD C8-CB qui wrappait déjà correctement.
 * NB : la convention 1/2 (±) diffère entre handlers (MVDD 1=- 2=+ vs MAC
 * 1=+ 2=-) — incohérence séparée NON traitée ici, à mesurer (sonde). */
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
    case 3: s->ar[xar] = c54x_circ_ref(addr, +(int16_t)s->ar[0], s->bk); break; /* *AR+0% circulaire modulo BK (BK=0→linéaire) — fix 2026-06-01 */
    }
    return addr;
}

/* ================================================================
 * Instruction execution
 * ================================================================ */

/* Execute one instruction. Returns number of words consumed (1 or 2). */
/* PC ring buffer for pre-IDLE trace */
uint16_t pc_ring[256];
int pc_ring_idx = 0;

/* Évalue une condition C54x depuis l'octet bas de l'opcode, per binutils
 * condition_codes[] (opcodes/tic54x-opc.c) : CC1=0x40 (test accu), CCB=0x08
 * (accu B sinon A), test bits[2:0] = EQ=5 NEQ=4 LT=3 LEQ=7 GT=6 GEQ=2 ;
 * AOV=0x70 ANOV=0x60 ; TC=0x30 NTC=0x20 ; C=0x0C NC=0x08 ; UNC=0x00.
 * Identique à l'évaluation du handler RC/RCD (correcte). Remplace l'ancien
 * décode (op>>4)&0xF des handlers CC/CCD qui lisait le MAUVAIS champ (seuls
 * UNC/AEQ justes par coïncidence ; NEQ/LT/LEQ/GT/GEQ/TC/C faux) → mauvais
 * call/no-call dans la power-scan 0xb1xx (CC[TC] f930) → push manquants →
 * over-pop SP → orphelin 0x80fd @0x94f3 → self-CALA 0x70c3 (=28868).
 * cf doc/SP_CATASTROPHE_70c4_SEQUENCE.md, vérifié sonde CC-MISMATCH. */
bool c54x_cond_true(C54xState *s, uint8_t cc)
{
    if (cc == 0x00) return true;                       /* UNC */
    if (cc & 0x40) {                                   /* CC1 : test accu */
        int64_t acc = (cc & 0x08) ? sext40(s->b) : sext40(s->a);
        bool ov = (cc & 0x08) ? !!(s->st0 & (1 << 9))  /* OVB */
                              : !!(s->st0 & (1 << 8));  /* OVA */
        if ((cc & 0x70) == 0x70) return ov;            /* AOV/BOV  */
        if ((cc & 0x70) == 0x60) return !ov;           /* ANOV/BNOV */
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

/* Faithful per-instruction interrupt LEVEL check (gated CALYPSO_C54X_IRQ_LEVEL).
 * The base model services interrupts only at the c54x_interrupt_ex call edge: an
 * IFR bit latched while INTM=1 is never taken later. Real C54x re-checks pending
 * unmasked interrupts at each instruction boundary. This restores that, so an
 * armed frame IT (INT3/bit3) fires once INTM drops -> native frame ISR runs. */
/* === Frame-IT LEVEL hold + PRIO (2026-07-25) ============================
 * La frame-IT (bit12/vec28, scheduler 0x7234 -> kernel FB) est posee en EDGE par
 * c54x_interrupt_ex a chaque trame. Mesure : INTM=1 ~permanent (wait-loop 0xdde6
 * + sections critiques 0xb52x) -> la fenetre INTM=0 de 5 insns coincide rarement
 * avec bit12 pendant -> vec28 dispatchee 80x sur ~36000 trames -> kernel FB affame
 * (AR5 jamais 0x2a00, fb0_att=0). Deux correctifs GATES :
 *  - LEVEL : maintenir bit12 asserte dans l IFR jusqu a ce que vec28 VECTORISE
 *            (re-assert chaque insn), pour que la prochaine transition INTM 1->0
 *            l attrape a coup sur (= "vectoriser a la transition, sinon garder").
 *  - PRIO  : quand bit12 ET un bit de priorite plus basse (ex bit5/BRINT0) pendent
 *            dans la meme fenetre, prendre bit12 (frame) en 1er au lieu du ctz brut,
 *            sinon BRINT0 vole la fenetre rare et re-masque (INTM=1). */
bool g_frame_it_level = false;

/* [2026-07-30] IFR-CLEAR-WHO — qui efface un bit demasque de l'IFR ?
 *
 * Gate CALYPSO_IFR_CLEAR_WHO (defaut 1, plafonne). Repond a la question laissee
 * ouverte par LEVELCHK-WINDOW : la fenetre INTM=0 est ouverte 99 % du temps et
 * `intm0+pend` vaut 0 — donc l'IT pendante DISPARAIT avant que la fenetre s'ouvre.
 * Les six sites d'effacement de l'IFR passent desormais par ici, avec leur nom.
 *
 * On ne logue que les bits DEMASQUES (IMR a 1) : effacer un bit masque est sans
 * consequence. Un effacement par vectorisation est LEGITIME — c'est le nom du
 * site qui le dit ("vector-*"). Un effacement depuis "mmio-write" sur un bit
 * demasque et non servi est, lui, une perte seche.
 */
void c54x_ifr_clear(C54xState *s, uint16_t mask, const char *site)
{
    uint16_t before = s->ifr;
    s->ifr &= (uint16_t)~mask;
    uint16_t perdus = (uint16_t)(before & ~s->ifr & s->imr);   /* demasques seulement */
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
        /* bit 5 = BRINT0 (livraison I/Q par le BSP) et bit 12 = frame : les deux
         * qui nous interessent. Les 20 premiers de chaque, puis periodique. */
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


/* Prototype utilise par le hook de transition INTM (c54x_interrupt_ex vient de
 * calypso_c54x.h). */
