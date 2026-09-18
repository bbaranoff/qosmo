/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * c54x_exec.c — Execution : c54x_exec_one et ses familles d'instructions
 *
 * Extrait de calypso_c54x.c le 2026-09-18 (decoupage par role).
 * Carte des fichiers dans c54x_internal.h.
 */
#include "c54x_internal.h"

static inline void c54x_f4_srcdst(uint16_t op, int *src, int *dst)
{
    static int fixed = -1;
    if (fixed < 0) {
        fixed = calypso_gate("CALYPSO_FIX_F4XX_SRCDST", 1);
        fprintf(stderr, "[c54x] FIX_F4XX_SRCDST %s (CALYPSO_FIX_F4XX_SRCDST=%d) — "
                "bit9=src bit8=dst %s (TI SPRU172C)\n",
                fixed ? "ACTIF" : "inactif", fixed,
                fixed ? "conforme a la doc" : "INVERSE, comportement d'avant le 04/08");
    }
    if (fixed) { *src = (op >> 9) & 1; *dst = (op >> 8) & 1; }
    else       { *src = (op >> 8) & 1; *dst = (op >> 9) & 1; }
}

/* ═══════════════════════════════════════════════════════════════════════════
 * [2026-08-04] FIX_DECODE_BRANCH — handlers MAC/bit RENDUS ATTEIGNABLES.
 *
 * Ces handlers etaient ecrits correctement mais places dans `case 0xF:` du
 * switch(hi4), alors que leurs opcodes ont hi4 = 2 ou 3. Ils etaient donc MORTS
 * PAR CONSTRUCTION, et les opcodes tombaient dans le MAC aveugle du `case 0x3:`
 * (`A = A + T*Smem`) ou dans le `case 0x2:`.
 *
 * Verificateur `sweep_reach.py` : 161 handlers examines, 11 suspects, tous ici.
 * (Un 12e, 0x9C00, etait un FAUX POSITIF : `case 0x8: case 0x9:` partage ses
 *  etiquettes, la v1 du script ne lisait que la premiere.)
 *
 * LISTE : 0x2800 MAC · 0x2A00/0x2E00 MACR/MASR · 0x3000 LD Smem,T ·
 *         0x3100 MPYA · 0x3200 LD Smem,ASM · 0x3300 MASA · 0x3400 BITT ·
 *         0x3500 MACA · 0x3700 MACAR
 *
 * ⚠️ LES CORPS SONT REPRIS VERBATIM. On corrige l'ATTEIGNABILITE, pas la
 * logique : toute subtilite de masque qui existait avant existe encore. En
 * particulier `0x2800/FC00` couvre 0x2800..0x2BFF et absorbe donc `0x2A00`
 * (MACR) qui est teste apres — ce recouvrement PREEXISTE, il n'est pas
 * introduit ici, et il reste a instruire contre SPRU172C.
 *
 * ⚠️ APPELE AVANT le `resolve_smem` de chaque case : chaque handler fait le
 * sien. Appeler apres provoquerait un DOUBLE post-increment des AR.
 *
 * Gate d'echappement `CALYPSO_FIX_DECODE_BRANCH=0` : rend -1 tout de suite,
 * les opcodes retombent dans le comportement d'avant le 04/08.
 *
 * Rend -1 si non traite, sinon le nombre de mots consommes.
 * ═══════════════════════════════════════════════════════════════════════════ */
static int c54x_mac_bit_family(C54xState *s, uint16_t op, int consumed)
{
    static int fdb = -1;
    if (fdb < 0) {
        fdb = calypso_gate("CALYPSO_FIX_DECODE_BRANCH", 1);
        fprintf(stderr, "[c54x] FIX_DECODE_BRANCH %s "
                "(CALYPSO_FIX_DECODE_BRANCH=%d) — MAC/MACR/MASR/MACA/MASA/MACAR/"
                "MPYA/LD-T/LD-ASM %s\n",
                fdb ? "ACTIF" : "inactif", fdb,
                fdb ? "atteignables" : "laisses morts (comportement d'avant)");
    }
    if (!fdb) return -1;
            /* === MAC/MAS family Smem,SRC (0x28xx..0x2Fxx, mask FE00, 1 word).
             * Per tic54x-opc.c + tic54x_hi8_map.md :
             *   0x2800 mac Smem,SRC      SRC = SRC + T * data[Smem]
             *   0x2A00 macr Smem,SRC     SRC = SRC + T * data[Smem] + 0x8000
             *   0x2C00 mas Smem,SRC      SRC = SRC - T * data[Smem]
             *   0x2E00 masr Smem,SRC     SRC = SRC - T * data[Smem] + 0x8000
             * bit 8 = SRC selector (0=A, 1=B).
             * FRCT (ST1 bit) : si set, produit shift << 1 (Q15*Q15 = Q31).
             *
             * BUG observé : MAC family non-implémentée → DSP correlator
             * ne fait jamais d'accumulation, A reste stale → a_sync_ANG
             * écrit 0x498D constant (garbage acc state).
             * Implémentation Smem-only ici (variantes Xmem/Ymem dual-MAC
             * 0xA000..0xBFFF non couvertes). */
            if ((op & 0xFC00) == 0x2800) {
                int mac_sub = (op >> 9) & 1;       /* 0=add, 1=subtract */
                int mac_rnd = (op >> 8) & 0; /* not used here, separate below */
                (void)mac_rnd;
                bool mac_ind;
                uint16_t mac_addr = resolve_smem(s, op, &mac_ind);
                int16_t mac_mem = (int16_t)data_read(s, mac_addr);
                int32_t mac_prod = (int32_t)(int16_t)s->t * (int32_t)mac_mem;
                if (s->st1 & ST1_FRCT) mac_prod <<= 1;
                /* bit 8 selects SRC accumulator (A=0/B=1).
                 * Actually per binutils encoding bit 9 is op variant (mac/mas)
                 * and bit 8 is round (R). The SRC selector is bit 0 of Smem?
                 * No — looking at tic54x table: opcode 0x2800/FE00 encodes :
                 *   bits 9..15 = op family (mac/mas/macr/masr)
                 *   bit 8      = SRC (A=0, B=1)
                 *   bits 0..7  = Smem
                 * Mais l'encoding pose mac=0x28xx (bit 8=0=A), 0x29xx (bit 8=1=B). */
                int mac_dst = (op >> 8) & 1;
                int64_t *mac_acc = mac_dst ? &s->b : &s->a;
                int64_t mac_term = (int64_t)(int32_t)mac_prod;
                if (mac_sub) mac_term = -mac_term;
                int64_t mac_new = sext40((*mac_acc + mac_term) & 0xFFFFFFFFFFULL);
                *mac_acc = mac_new;
                return (int)(consumed + s->lk_used);
            }
            /* MACR/MASR (mask FE00, base 0x2A00/0x2E00) : same + round +0x8000.
             * bit 9 distingue add/sub : déjà géré ci-dessus via mac_sub. mais le
             * round est sur les opcodes 0x2A.../0x2E... → bit 10 ? Re-check */
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

            /* 0x3500 MACA Smem [, B] (mask FF00, 1 word) — B = B + A.hi * data[Smem].
             * Spécial : utilise A.hi (= A[31:16]) comme multiplicateur. */
            if ((op & 0xFF00) == 0x3500) {
                bool maca_ind;
                uint16_t maca_addr = resolve_smem(s, op, &maca_ind);
                int16_t maca_mem = (int16_t)data_read(s, maca_addr);
                int16_t maca_ahi = (int16_t)((s->a >> 16) & 0xFFFF);
                int32_t maca_prod = (int32_t)maca_ahi * (int32_t)maca_mem;
                if (s->st1 & ST1_FRCT) maca_prod <<= 1;
                s->b = sext40((s->b + (int64_t)(int32_t)maca_prod) & 0xFFFFFFFFFFULL);
                return (int)(consumed + s->lk_used);
            }

            /* 0x3300 MASA Smem [, B] (mask FF00, 1 word) — B = B - A.hi * data[Smem]. */
            if ((op & 0xFF00) == 0x3300) {
                bool masa_ind;
                uint16_t masa_addr = resolve_smem(s, op, &masa_ind);
                int16_t masa_mem = (int16_t)data_read(s, masa_addr);
                int16_t masa_ahi = (int16_t)((s->a >> 16) & 0xFFFF);
                int32_t masa_prod = (int32_t)masa_ahi * (int32_t)masa_mem;
                if (s->st1 & ST1_FRCT) masa_prod <<= 1;
                s->b = sext40((s->b - (int64_t)(int32_t)masa_prod) & 0xFFFFFFFFFFULL);
                return (int)(consumed + s->lk_used);
            }

            /* 0x3700 MACAR Smem [, B] = MACA + round */
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
                return (int)(consumed + s->lk_used);
            }

            /* 0x3100 MPYA Smem (mask FF00, 1 word) — B = A.hi * data[Smem]. */
            if ((op & 0xFF00) == 0x3100) {
                bool mpya_ind;
                uint16_t mpya_addr = resolve_smem(s, op, &mpya_ind);
                int16_t mpya_mem = (int16_t)data_read(s, mpya_addr);
                int16_t mpya_ahi = (int16_t)((s->a >> 16) & 0xFFFF);
                int32_t mpya_prod = (int32_t)mpya_ahi * (int32_t)mpya_mem;
                if (s->st1 & ST1_FRCT) mpya_prod <<= 1;
                s->b = sext40((int64_t)(int32_t)mpya_prod);
                return (int)(consumed + s->lk_used);
            }

            /* [2026-08-23] 6e MANQUE — MPY Smem,dst et LTD Smem n etaient
             * decodes NULLE PART. Balayage de toutes les conditions de
             * l emulateur : LD Smem,T (0x3000) et ST T,Smem (0x8C00) sont
             * decodes, mais 0x2000/0xFE00 et 0x4C00/0xFF00 n avaient AUCUN
             * handler.
             * Chemin critique : la boucle qui alimente les blocs 3 et 4 du banc
             * SCH est  0x81e3 LD #1,ASM ; 0x81e4 MPY Smem,dst ; 0x81e5/0x81e6
             * ST A,*ARx+ (stores paralleles). L instruction qui doit PRODUIRE la
             * valeur ne s executait pas : A gardait zero et les stores ecrasaient
             * avec des zeros les copies saines du correlateur (112 ecritures non
             * nulles par les MVDD 0x7ce0/0x7ce4, puis 210 zeros).
             * Semantique : MPY Smem,dst -> dst = T * Smem  (bit 8 = accumulateur)
             *              LTD Smem     -> T = Smem ; data[Smem+1] = Smem
             * Gate CALYPSO_ISA_MPY_SMEM (defaut 1).
             * ⚠️ EFFET GLOBAL : instruction courante du DSP. */
            {
                static int _ms = -1;
                if (_ms < 0) {
                    _ms = calypso_gate("CALYPSO_ISA_MPY_SMEM", 1);
                    fprintf(stderr, "[c54x] ISA-MPY-SMEM %s : MPY Smem,dst "
                            "(0x2000/0xFE00, dst = T*Smem) et LTD Smem "
                            "(0x4C00/0xFF00) %s\n",
                            _ms ? "ACTIF" : "INACTIF",
                            _ms ? "IMPLEMENTES" : "restent inertes");
                }
                if (_ms && (op & 0xFE00) == 0x2000) {
                    bool mp_ind;
                    uint16_t mp_addr = resolve_smem(s, op, &mp_ind);
                    int16_t  mp_v = (int16_t)data_read(s, mp_addr);
                    int64_t  mp_p = (int64_t)(int16_t)s->t * (int64_t)mp_v;
                    if (s->st1 & ST1_FRCT) mp_p <<= 1;
                    if ((op >> 8) & 1) s->b = sext40(mp_p);
                    else               s->a = sext40(mp_p);
                    {   static int _t = -1; static unsigned _tn = 0;
                        if (_t < 0) _t = calypso_gate("CALYPSO_ISA_MPY_TRACE", 0);
                        if (_t && _tn < 24) {
                            _tn++;
                            fprintf(stderr, "[c54x] MPY-SMEM #%u PC=0x%04x op=0x%04x "
                                    "T=0x%04x Smem@0x%04x=%d -> %s=0x%010llx insn=%u\n",
                                    _tn, s->pc, op, (unsigned)s->t, mp_addr, (int)mp_v,
                                    ((op >> 8) & 1) ? "B" : "A",
                                    (unsigned long long)(mp_p & 0xFFFFFFFFFFULL),
                                    s->insn_count);
                        }
                    }
                    return (int)(consumed + s->lk_used);
                }
                if (_ms && (op & 0xFF00) == 0x4C00) {
                    bool lt_ind;
                    uint16_t lt_addr = resolve_smem(s, op, &lt_ind);
                    uint16_t lt_v = data_read(s, lt_addr);
                    s->t = lt_v;
                    data_write(s, (uint16_t)(lt_addr + 1), lt_v);  /* insertion de delai */
                    return (int)(consumed + s->lk_used);
                }
            }

            /* 0x3000 LD Smem, T (mask FF00, 1 word) — T = data[Smem]. */
            if ((op & 0xFF00) == 0x3000) {
                bool ldt_ind;
                uint16_t ldt_addr = resolve_smem(s, op, &ldt_ind);
                s->t = data_read(s, ldt_addr);
                return (int)(consumed + s->lk_used);
            }

            /* 0x3200 LD Smem, ASM (mask FF00, 1 word) — ASM = data[Smem] & 0x1F (5 bits). */
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
 * [2026-08-04] FIX_SFTA_CARRY — SFTA doit poser la RETENUE.
 *
 * L'implementation precedente decalait l'accumulateur et rendait la main sans
 * jamais toucher ST0_C. Or `ROL` lit cette retenue.
 *
 * CONSEQUENCE MESUREE. La boucle de transfert de bits en 0x9ac7..0x9ace du
 * firmware DSP :
 *     0x9ac7  sfta A      ; sort le bit de poids faible de A -> RETENUE
 *     0x9acc  rol  B      ; B = B<<1 | RETENUE
 *     0x9ad0  stl  *AR2+, B
 * La retenue ne transportant rien, B restait a 0, `stl` ecrivait 0x0000 dans
 * 0x2c3c, et le `mvdd` de 0x9723 publiait ce vide dans a_cd[3..14] — les 12
 * mots de charge utile que l'ARM remonte en L2. L'opcode reel est `f47f`, soit
 * shift = (0x1F) - 32 = -1 : « prends le bit 0 de A et mets-le dans C ».
 *
 * TI SPRU172C, page SFTA :
 *     If SHIFT < 0 : (src((-SHIFT)-1)) -> C ; src << SHIFT -> dst
 *                    remplissage haut = src(39) si SXM=1, sinon 0
 *     Else         : (src(39 - SHIFT)) -> C ; src << SHIFT -> dst
 *     Status Bits  : Affected by SXM and OVM / Affects C and OVdst
 *
 * ⚠️ CLASSE DIFFERENTE des correctifs precedents du jour. FIX_ALU3_DST,
 * FIX_F4XX_SRCDST, FIX_BITT_CASE3 et FIX_DECODE_BRANCH portaient sur des
 * handlers mal places ou des operandes inverses — detectables par
 * `sweep_reach.py`. Celui-ci est un EFFET DE BORD DE DRAPEAU non modelise :
 * le resultat calcule est juste, mais l'indicateur d'etat dont depend
 * l'instruction suivante est absent. Le verificateur d'atteignabilite ne peut
 * PAS voir ce defaut ; il faut un audit distinct croisant chaque opcode avec la
 * ligne « Status Bits » de SPRU172C.
 *
 * ⚠️ Le remplissage selon SXM est ajoute ici aussi (la doc le mandate au meme
 * endroit). Avant, le decalage droit etait toujours arithmetique, ce qui
 * equivaut a SXM=1 en permanence.
 *
 * Gate d'echappement `CALYPSO_FIX_SFTA_CARRY=0` : ni retenue ni SXM, soit le
 * comportement d'avant le 04/08.
 * ═══════════════════════════════════════════════════════════════════════════ */
static void c54x_sfta_exec(C54xState *s, uint16_t op)
{
    static int fsc = -1;
    if (fsc < 0) {
        fsc = calypso_gate("CALYPSO_FIX_SFTA_CARRY", 1);
        fprintf(stderr, "[c54x] FIX_SFTA_CARRY %s (CALYPSO_FIX_SFTA_CARRY=%d) — "
                "SFTA %s la retenue (TI SPRU172C)\n",
                fsc ? "ACTIF" : "inactif", fsc,
                fsc ? "POSE" : "ne pose PAS");
    }
    int src, dst;
    c54x_f4_srcdst(op, &src, &dst);
    int shift = op & 0x1F;
    if (shift > 15) shift -= 32;
    int64_t sv = sext40(src ? s->b : s->a);

    if (fsc) {
        int cbit = (shift < 0) ? (int)((sv >> ((-shift) - 1)) & 1)
                               : (int)((sv >> (39 - shift)) & 1);
        if (cbit) s->st0 |= ST0_C;
        else      s->st0 &= ~ST0_C;
    }

    if (shift >= 0) {
        sv <<= shift;
    } else if (fsc && !(s->st1 & ST1_SXM)) {
        sv = (int64_t)(((uint64_t)sv & 0xFFFFFFFFFFULL) >> (-shift));
    } else {
        sv >>= (-shift);
    }
    if (dst) s->b = sext40(sv);
    else     s->a = sext40(sv);
}


/* ═══════════════════════════════════════════════════════════════════════════
 * [2026-08-04] FIX_ROL_CARRY_BIT — ROR/ROL lisaient la retenue AU MAUVAIS BIT.
 *
 * `ST0_C` vaut `(1 << 11)` (calypso_c54x.h:74). Or les quatre handlers ROR/ROL
 * (dupliques dans deux branches) lisaient `(s->st0 >> 8) & 1`, soit le **bit 8**
 * — qui appartient a `ST0_DP_MASK` (bits 8-0, pointeur de page de donnees).
 * Ils ECRIVAIENT pourtant la retenue correctement via `s->st0 |= ST0_C`.
 * Lecture et ecriture ne parlaient donc pas du meme bit : ROL rotationnait un
 * bit du pointeur de page a la place de la retenue.
 *
 * CONSEQUENCE MESUREE. La boucle de transfert de bits du firmware DSP en
 * 0x9ac7..0x9ace fait `sfta A` (retenue <- bit sorti) puis `rol B` (B <<= 1 | C).
 * Meme apres FIX_SFTA_CARRY, qui pose enfin la retenue au bit 11, `rol` allait
 * la chercher au bit 8 : B restait nul, `stl *AR2+, B` ecrivait 0x0000 dans
 * 0x2c3c, et le `mvdd` de 0x9723 publiait ce vide dans a_cd[3..14].
 *
 * ⚠️ Meme CLASSE que FIX_SFTA_CARRY (drapeau d'etat), mais cause differente :
 * ici le drapeau existe et est correctement ecrit, c'est la LECTURE qui vise a
 * cote. Aucun verificateur structurel ne voit ca ; seul un audit croisant
 * chaque acces a ST0/ST1 avec les macros le montrerait.
 *
 * Gate `CALYPSO_FIX_ROL_CARRY_BIT=0` : relit le bit 8, comportement d'avant.
 * ═══════════════════════════════════════════════════════════════════════════ */
static inline uint16_t c54x_carry_in(C54xState *s)
{
    static int frc = -1;
    if (frc < 0) {
        frc = calypso_gate("CALYPSO_FIX_ROL_CARRY_BIT", 1);
        fprintf(stderr, "[c54x] FIX_ROL_CARRY_BIT %s "
                "(CALYPSO_FIX_ROL_CARRY_BIT=%d) — ROR/ROL lisent la retenue au "
                "bit %s\n", frc ? "ACTIF" : "inactif", frc,
                frc ? "11 (ST0_C, correct)" : "8 (ST0_DP, comportement d'avant)");
    }
    return frc ? ((s->st0 & ST0_C) ? 1 : 0) : (uint16_t)((s->st0 >> 8) & 1);
}

bool calypso_fix_enabled(const char *name)
{
    static char buf[1024];
    static int  init = 0;
    if (!init) {
        const char *e = getenv("CALYPSO_FIXES");
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

/* [2026-08-03] CAL000 §5.1 : le TIMER du DSP est TINT = IMR bit 3 = vec 19.
 * Le modele tirait sur vec20/bit4, qui est RINT = SPI RECEIVE. L'erreur vient de
 * la table SPRU131 (C54x generique) qui etait dans calypso_c54x.h : elle a QUATRE
 * lignes externes avant TINT, le Calypso n'en a que TROIS — d'ou un decalage de 1.
 *
 * [2026-09-03] SAS `CALYPSO_IT_TABLE_DOC` VIDE. Sa seule justification etait que
 * « ce site tourne sur le chemin du shunt qui campe » — le shunt est retire, la
 * justification tombe. TINT est desormais emise sur vec19/bit3 sans condition.
 *
 * ⚠️ CHANGEMENT DE COMPORTEMENT NON MESURE. L'IMR relevee (0x52ed) a le bit 3
 * DEMASQUE et le bit 4 MASQUE : jusqu'ici l'IT timer etait silencieusement jetee
 * par c54x_interrupt_ex (qui respecte l'IMR) ; elle est maintenant reellement
 * dispatchee sur vec19 a chaque underflow. Le handler ROM de vec19 est un stub
 * RETE, donc l'effet attendu est benin — mais « attendu » n'est pas « mesure », et
 * la symetrie de pile RETE est un point sensible connu. A verifier au premier run
 * sous charge : si la pile derive, c'est ici. */
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
    /* [2026-07-27] B1 (gated CALYPSO_B1) : au kernel MAC 0xa076, dump la table
     * de reference du correlateur data[0x2c00..0x2c0f] + checksum -> tranche si
     * elle est peuplee (boot-copy 0x76f8->0x2c00 faite) ou VIDE (on correle
     * contre du zero). Le moins cher / binaire. */
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
    /* [2026-07-25] TEST-3FAE (gated CALYPSO_FORCE_3FAE) : le handler FB poll
     * data[0x3fae] bit8 (0x0100) via BITF @0x90c8/0x90ed/0x9128 puis BC TC -> il
     * attend ce flag "burst pret" que RIEN n ecrit -> boucle infinie, kernel
     * 0xa076 jamais atteint. On force le flag dans le handler pour confirmer qu il
     * debloque vers le kernel (=> ensuite wire depuis la chaine RX/BRINT0). */
    {
        /* [2026-07-25] CORR-BANK2 (gated) : forcer XPC=2 dans la region corrélateur
         * -> le handler FB tourne depuis PROM2 (overlay different) au lieu de PROM0.
         * Test "voir si bank2 debloque". Risque derail (RET/contexte). */
        /* @BEQUILLE — CORR_BANK  (CALYPSO_CORR_BANK, VALEUR, defaut -1/OFF)
         *   masque  : la selection d'overlay/banque du handler FB. On ECRASE s->xpc a
         *             chaque instruction de [0x8d00..0xa200] au lieu que le dispatcher
         *             natif pose la bonne banque.
         *   retirer : quand le dispatcher CALA @0xb01e resout la banque correcte lui-meme
         *             (XPC observe == banque attendue sans forcage).
         *   PIEGE   : la valeur "0" N'ETEINT PAS — elle force XPC=0. Seul unset coupe.
         */
        static int cbk = -2;
        if (cbk == -2) { const char *e = getenv("CALYPSO_CORR_BANK");
                         cbk = (e && *e) ? atoi(e) : -1; }   /* -1=off ; 0..3 = XPC force */
        if (cbk >= 0 && cbk <= 3 && s->pc >= 0x8d00 && s->pc <= 0xa200 && s->xpc != (uint16_t)cbk) {
            s->xpc = (uint16_t)cbk;
        }
    }
    {
        /* @BEQUILLE — FORCE_3FAE  (CALYPSO_FORCE_3FAE, EXISTS, defaut OFF)
         *   masque  : l'ecriture des flags de handshake FB que RIEN n'implemente —
         *             data[0x3faa] bit2/bit8, [0x3fab] bit8, [0x3fae] bit8. Poses a CHAQUE
         *             instruction du handler (xpc=0, pc 0x8d00..0xa200).
         *   retirer : quand la chaine RX/BRINT0 ecrit ces flags (RANK2 resolu).
         */
        static int f3ae = -1;
        if (f3ae < 0) f3ae = calypso_gate("CALYPSO_FORCE_3FAE", 0);
        if (f3ae && s->xpc == 0 && s->pc >= 0x8d00 && s->pc <= 0xa200) {
            /* TOUTE la handshake FB-det que le handler poll (0x8866 + 0x90xx) :
             * 0x3faa bit2/bit8, 0x3fab bit8, 0x3fae bit8. Decouple RANK3 du feed
             * RX mort (RANK2) pour voir si le kernel se debloque. */
            s->data[0x3faa] |= 0x0104;
            s->data[0x3fab] |= 0x0100;
            s->data[0x3fae] |= 0x0100;
        }
    }
    /* [2026-07-25] CORR-FLOW (gated CALYPSO_CORR_FLOW) : trace FACTUELLE du flux du
     * handler FB en banc0 (0x8d00..0xa200, XPC=0) — PC/opcode BRUT + flags ST0(TC,C)
     * + A + AR0/AR4/AR5. Permet de VERIFIER nous-memes (contre SPRU172) OU/POURQUOI le
     * flux quitte le kernel MAC 0xa076 (lit 0x2a00). Marque 0xa076/0x9a80. Cap 8000. */
    {
        static int cf = -1; static unsigned cfn = 0;
        if (cf < 0) cf = calypso_gate("CALYPSO_CORR_FLOW", 0);
        /* Range ELARGIE : inclut 0x8866 (sous-routine handshake, <0x8d00) + 0xa076.
         * Trace AUSSI AR3 (ptr CMPS/coeff) et AR1/AR2 pour voir le setup pointeurs. */
        /* Skip la boucle de copie 0x8866-0x886c (op 8091, ~134x/appel) qui bouffait
         * tout le budget log -> le cap est reserve au VRAI flux (state-machine +
         * progression vers 0x93a5). Dedup aussi les PC repetes consecutifs. */
        static uint16_t cf_lastpc = 0;
        if (cf && s->xpc == 0 && s->pc >= 0x8600 && s->pc <= 0xa200 && cfn < 20000
            /* [2026-07-26 WF] ne tracer QUE quand une vraie tache FB/SB est active
             * (task_md=5/6) -> capture la fenetre POST-fix (fn>=6866) au lieu de
             * s epuiser sur le spinning idle pre-fix (+0.8s). */
            && (s->data[0x0804] == 5 || s->data[0x0804] == 6
                || s->data[0x0818] == 5 || s->data[0x0818] == 6
                || s->pc >= 0xa000)   /* [fix] trace AUSSI le flux post-gate 0xa0xx (task_md=0) */
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

    /* === CORR-TRACE (2026-06-02) : trace instruction-par-instruction la boucle
     * MAC du corrélateur FB autour de 0x8576 à l'instant détection. Montre
     * PC/opcode/AR3/A AVANT chaque instr : si AR3 ne bouge pas d'une ligne à
     * l'autre, ou si A n'accumule pas, on a le coupable (post-incr *AR3+ / RPT
     * / MAC). One-shot ~60 instr. CALYPSO_DEBUG=CORR-TRACE. */
    /* DERAIL-EE00 (2026-06-02) : attrape le saut DANS la zone PROM vide 0xee00
     * (op=0x0000) post-fix SACCD. Logge le PC source + opcode + XPC pour
     * trancher runaway firmware (branche fausse) vs bug paging XPC (adresse
     * légitime bankée fetchée page 0). One-shot ~12. */
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
        const char *l = getenv("CALYPSO_CORR_LO"); const char *h = getenv("CALYPSO_CORR_HI");
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

    /* === AR-CLOBBER probe (2026-05-29) ===
     * Track AR1/AR2/AR6/AR7 transitions to 0 — when an AR pointer
     * becomes 0, any subsequent indirect store *ARx will write to
     * data[0x00] = IMR MMR (= clobber). Documented as the 2026-05-25
     * fix reason (cf c54x_reset comment). Capture l'instruction qui
     * a fait la transition (= last_exec_pc + s->prog[last_exec_pc])
     * pour identifier le coupable. Gated CALYPSO_DEBUG=AR_CLOBBER. */
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

    if (s->pc == 0x013b && getenv("CALYPSO_AR0_DEBUG")) {
        static int d13 = 0;
        if (!d13) { d13 = 1;
            fprintf(stderr, "[c54x] SUB-013B A=0x%06llx DP=0x%03x d_page(08D4)=0x%04x insn=%u\n",
                    (unsigned long long)(s->a & 0xFFFFFF), s->st0 & 0x1FF,
                    /* [2026-07-29] l'etiquette disait 08D4, la lecture prenait
                     * 0x08E2 dans data[] — deux erreurs qui s'annulaient a
                     * l'affichage. On lit la vraie cellule dans api_ram. */
                    s->api_ram ? s->api_ram[0x08D4 - C54X_API_BASE] : s->data[0x08D4],
                    s->insn_count);
            for (uint16_t a = 0x0138; a <= 0x014c; a += 4)
                fprintf(stderr, "[c54x] PROG[0x%04x..]= %04x %04x %04x %04x\n",
                        a, s->prog[a], s->prog[(uint16_t)(a+1)],
                        s->prog[(uint16_t)(a+2)], s->prog[(uint16_t)(a+3)]);
        }
    }
    if (s->pc == 0x8869 && getenv("CALYPSO_AR0_DEBUG")) {
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
    if (s->pc == 0x7234 && getenv("CALYPSO_AR0_DEBUG")) {
        static int d72 = 0;
        if (!d72) { d72 = 1;
            int ovly = !!(s->pmst & PMST_OVLY);
            fprintf(stderr, "[c54x] DERAIL-013B XPC=0x%02x PMST=0x%04x OVLY=%d fetch(0x013b)=0x%04x insn=%u\n",
                    s->xpc & 0xFF, s->pmst, ovly, prog_fetch(s, 0x013b), s->insn_count);
            fprintf(stderr, "[c54x] OVERLAY data[0x0138..]= %04x %04x %04x %04x %04x %04x %04x %04x\n",
                    s->data[0x0138], s->data[0x0139], s->data[0x013a], s->data[0x013b],
                    s->data[0x013c], s->data[0x013d], s->data[0x013e], s->data[0x013f]);
            /* la boucle go-live 0xa4de-0xa4e8 (pourquoi 0xa4e1 reboucle) + le
             * soft-vector data[0x3f6d] qui pilote le trampoline. */
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
            /* + le code du scheduler 0x7234 pour reconfirmer CALL 0x013b */
            fprintf(stderr, "[c54x] PROG[0x7234..]= %04x %04x %04x %04x\n",
                    s->prog[0x7234], s->prog[0x7235], s->prog[0x7236], s->prog[0x7237]);
        }
    }
    uint8_t hi4 = (op >> 12) & 0xF;
    uint8_t hi8 = (op >> 8) & 0xFF;

    /* [2026-07-28] LOT DE CORRECTIFS DE LONGUEUR — gate CALYPSO_FIXES (voir
     * calypso_fix_enabled). Chaque entree cite binutils tic54x-opc.c, dont le 2e
     * champ EST le nombre de mots. Le decodeur consommait 2 mots la ou ces
     * instructions n en font qu 1, ce qui desynchronise tout le decodage suivant. */
    {   /* [2026-07-28] Les correctifs ci-dessous sans appel a calypso_fix_enabled()
         * sont VALIDES et INCONDITIONNELS (verifies en SHUNT_LEGIT sous charge et en
         * NATIVE_HELPED avec retour du SHADOW-DADST). Ceux qui portent encore un
         * calypso_fix_enabled("FIX_...") sont dans le SAS : formellement corrects mais
         * INFIRMES PAR LA MESURE, voir leur commentaire. */
        /* LD Xmem, SHFT, dst — binutils { "ld", 1,3,3, 0x9400, 0xFE00, {OP_Xmem,OP_SHFT,OP_DST} }
         * (etait decode MVDK/MVKD sur 2 mots) */
        if ((op & 0xFE00) == 0x9400) {
            uint16_t a = resolve_xmem(s, op);
            uint16_t v = data_read(s, a);
            int shft = op & 0xF, d = (op >> 8) & 1;
            int64_t x = (s->st1 & ST1_SXM) ? (int64_t)(int16_t)v : (int64_t)(uint16_t)v;
            x <<= shft;
            if (d) s->b = sext40(x); else s->a = sext40(x);
            return 1;
        }
        /* BIT Xmem, BITC : TC = Xmem(15-BITC) — binutils { "bit", 1,2,2, 0x9600, 0xFF00 }
         * (etait decode MVDP sur 2 mots) */
        if ((op & 0xFF00) == 0x9600) {
            uint16_t a = resolve_xmem(s, op);
            uint16_t v = data_read(s, a);
            int bitc = op & 0xF;
            if ((v >> (15 - bitc)) & 1) s->st0 |= ST0_TC; else s->st0 &= ~ST0_TC;
            return 1;
        }
        /* SUB Xmem, Ymem, dst : dst = (Xmem - Ymem) << 16 — binutils { "sub", 1,..., 0xA200, 0xFE00 }
         * (etait decode ADD/SUB #lk sur 2 mots) */
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
        /* LD Xmem, dst || MAC/MAS/MASR Ymem — binutils { "ld", 1,..., 0xA800/0xAC00/0xAE00, 0xFE00 }
         * (etaient decodes AND #lk / MACP / MACD sur 2 mots).
         * On execute la partie LD et on laisse la partie parallele : approximatif sur le
         * RESULTAT, mais la LONGUEUR redevient juste et le flux cesse de deriver. */
        if (((op & 0xFE00) == 0xA800 || (op & 0xFE00) == 0xAC00 || (op & 0xFE00) == 0xAE00)
            && calypso_fix_enabled("FIX_LD_PARALLEL")) {
            uint16_t a = resolve_xmem(s, op);
            uint16_t v = data_read(s, a);
            int64_t x = (s->st1 & ST1_SXM) ? (int64_t)(int16_t)v : (int64_t)(uint16_t)v;
            if ((op >> 8) & 1) s->b = sext40(x << 16); else s->a = sext40(x << 16);
            return 1;
        }

        /* LDM MMR, dst — binutils { "ldm", 1,2,2, 0x4800, 0xFE00, {OP_MMR,OP_DST} }.
         * Un MMR est une valeur 16 bits NON SIGNEE (un pointeur, un compteur, un
         * registre d etat) : le sign-etendre transforme AR=0x8000 en une valeur
         * negative de 40 bits. SPRU172C : « LDM MMR, dst : dst = MMR », sans
         * extension de signe (LDU porte explicitement « uns », LDM n a pas de
         * variante signee). */
        if ((op & 0xFE00) == 0x4800 && calypso_fix_enabled("FIX_LDM_ZEROEXT")) {
            int mmr = op & 0x7F;
            uint16_t v = data_read(s, mmr);
            if ((op >> 8) & 1) s->b = (int64_t)(uint16_t)v; else s->a = (int64_t)(uint16_t)v;
            return 1 + s->lk_used;
        }
        /* DST src, Lmem — binutils { "dst", 1,2,2, 0x4E00, 0xFE00, {OP_SRC1,OP_Lmem} }.
         * Lmem est un operande LONG (2 mots) : le pointeur doit donc avancer de 2, pas
         * de 1. Une post-modification de 1 decale tout le balayage d un tableau de mots
         * longs — l erreur est silencieuse et cumulative. */
        if ((op & 0xFE00) == 0x4E00) {
            int src = (op >> 8) & 1;
            int64_t v = src ? s->b : s->a;
            uint8_t sm = op & 0xFF;
            if (sm & 0x80) {                       /* indirect : *ARx avec post-modif */
                int ar = sm & 0x7, mod = (sm >> 3) & 0xF;
                uint16_t a = s->ar[ar];
                data_write(s, a,     (uint16_t)((v >> 16) & 0xFFFF));
                data_write(s, a + 1, (uint16_t)(v & 0xFFFF));
                if (mod == 0x2) s->ar[ar] = a + 2;        /* *ARx+ : +2, pas +1 */
                else if (mod == 0x1) s->ar[ar] = a - 2;   /* *ARx- : -2, pas -1 */
                return 1;
            }
            {   /* direct : DP:offset */
                uint16_t a = (uint16_t)(((s->st0 & ST0_DP_MASK) << 7) | (sm & 0x7F));
                data_write(s, a,     (uint16_t)((v >> 16) & 0xFFFF));
                data_write(s, a + 1, (uint16_t)(v & 0xFFFF));
                return 1;
            }
        }
        /* STL/STH src, SHFT, Xmem — binutils { "stl"/"sth", 1,.., 0x9800/0x9A00, 0xFE00,
         * {OP_SRC1,OP_SHFT,OP_Xmem} }. Le champ SHFT (bits 3-0) etait ignore : la valeur
         * stockee n avait pas la bonne echelle. */
        /* [2026-09-17] active par defaut (CALYPSO_FIX_STL_STH_SHFT=0 pour l'ancien
         * comportement) : SPRU172C 4-169/4-172, « syntaxe 3 : si SHFT = 0 l'opcode
         * est assemble en syntaxe 1 », donc TOUTE occurrence 0x98/0x9A a SHFT != 0.
         * Le demodulateur SB en a 4 sites (0x7694/0x7697 `9a91/9a11`, 0x8217). */
        static int fix_shft = -1;
        if (fix_shft < 0) fix_shft = calypso_gate("CALYPSO_FIX_STL_STH_SHFT", 1);
        if (((op & 0xFE00) == 0x9800 || (op & 0xFE00) == 0x9A00)
            && (fix_shft || calypso_fix_enabled("FIX_STL_STH_SHFT"))) {
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
        /* SUB Smem, 16, src [, dst] — binutils { "sub", 1,.., 0x4000, 0xFC00,
         * {OP_Smem,OP_16,OP_SRC,OPT|OP_DST} }. Deux champs distincts : bit 9 = SRC
         * (l accumulateur source) et bit 8 = DST. Le bit 9 etait ignore, donc la
         * soustraction partait toujours du meme accumulateur. */
        if ((op & 0xFC00) == 0x4000) {
            bool ind2; uint16_t a = resolve_smem(s, op, &ind2);
            uint16_t v = data_read(s, a);
            int srcb = (op >> 9) & 1, dstb = (op >> 8) & 1;
            int64_t sv = srcb ? s->b : s->a;
            int64_t r  = sv - (((int64_t)(int16_t)v) << 16);
            if (dstb) s->b = sext40(r); else s->a = sext40(r);
            return 1 + s->lk_used;
        }
        /* STL B, ASM, Smem — binutils { "stl", 1,..., 0x8400, 0xFE00 } couvre 0x85 (src = B)
         * (etait decode MVPD sur 2 mots). Miroir exact du handler 0x84 deja valide. */
        if ((op & 0xFF00) == 0x8500) {
            bool ind2; uint16_t a = resolve_smem(s, op, &ind2);
            int shift = asm_shift(s);
            int64_t v = s->b;
            if (shift >= 0) v <<= shift; else v >>= (-shift);
            data_write(s, a, (uint16_t)(v & 0xFFFF));
            return 1 + s->lk_used;
        }
        /* ST TRN, Smem — binutils { "st", 1,..., 0x8D00, 0xFF00 }
         * (etait decode MVDD sur 2 mots) */
        if ((op & 0xFF00) == 0x8D00) {
            bool ind2; uint16_t a = resolve_smem(s, op, &ind2);
            data_write(s, a, s->trn);
            return 1 + s->lk_used;
        }
    }

    /* DISP-ENTRY (CALYPSO_DEBUG=DISP-ENTRY, c web 2026-05-29) : discriminateur
     * préemption-IT vs clobber. Logge UNIQUEMENT l'entrée dispatcher 0x8341,
     * avec DP/ST0/SP/AR2 + état IT (INTM/IFR/INT3-pending) + contexte de la
     * DERNIÈRE IT servie (vec, Δinsn, PC+DP foreground préemptés) + prédiction
     * du slot LUT qui sera lu à 0x834d = data[(DP<<7)|0x07] → handler vs garbage.
     * DIFF entrées OK (DP=0x124) vs KO (DP≠0x124) : si KO ⟺ IT récente (Δinsn
     * petit, fg_dp=DP-KO) → (b) préemption confirmée, root = INTM/IT. */
    /* ORACLE (border, debug pas fix) : CALYPSO_FORCE_DP=0x124 force le champ DP
     * de ST0 à l'entrée dispatcher 0x8341. Si FB lock + AFC converge → le bit
     * est load-bearing, la chasse au DP périmé est justifiée. Sinon → faute DSP
     * plus profonde DERRIÈRE le dispatcher, et chasser 0x3125 est prématuré. */
    /* [2026-07-22] FORCE-DISPATCH (gated CALYPSO_FORCE_DISPATCH=1) : le scheduler
     * frame 0x7234 (atteint via vec28) DERAILLE vers 0x013b car DP est garbage
     * (d_dsp_page=0xf600). On force DP=0x124 (la page GSM correcte, ORACLE) a
     * l'entree 0x7234 -> empeche le derail -> le flux natif atteint le dispatcher
     * 0x8341 -> LUT tache FB -> correlateur 0x8d00. Gate force-dispatch. */
    if (s->pc == 0x7234) {
        /* [2026-07-22] DUMP one-shot du scheduler 0x7234 (gated AR0_DEBUG) : que
         * fait-il, dou vient 0x013b (branche indirecte sur quel pointeur ?). */
        if (getenv("CALYPSO_AR0_DEBUG")) {
            static int d7 = 0;
            if (!d7) { d7 = 1;
                fprintf(stderr, "[c54x] SCHED-7234 A=0x%06llx ST0=0x%04x DP=0x%03x "
                        "AR1=%04x AR2=%04x AR5=%04x d_page(08D4)=0x%04x d584=0x%04x insn=%u\n",
                        (unsigned long long)(s->a & 0xFFFFFF), s->st0, s->st0 & 0x1FF,
                        s->ar[1], s->ar[2], s->ar[5],
                        /* [2026-07-29] idem : 0x08D4 dans api_ram, pas 0x08E2 dans data[]. */
                        s->api_ram ? s->api_ram[0x08D4 - C54X_API_BASE] : s->data[0x08D4],
                        s->data[0x0584], s->insn_count);
                for (uint16_t a = 0x7230; a <= 0x7240; a += 4)
                    fprintf(stderr, "[c54x] PROG[0x%04x..]= %04x %04x %04x %04x\n",
                            a, s->prog[a], s->prog[(uint16_t)(a+1)],
                            s->prog[(uint16_t)(a+2)], s->prog[(uint16_t)(a+3)]);
            }
        }
        /* @BEQUILLE — FORCE_DISPATCH  (CALYPSO_FORCE_DISPATCH, atoi>0, defaut OFF ;
         *              calypso_wire.env:=1)
         *   masque  : le scheduler frame 0x7234 est atteint avec DP garbage et d_dsp_page
         *             a 0, donc la LUT 0x8341 ne resout pas et la tache GSM/FB n'est jamais
         *             dispatchee. On force DP=0x124 + data[0x08E2]=data[0x0584]=0x0002.
         *   retirer : des que le prologue 0x013b restaure un DP valide et que le producteur
         *             de d_dsp_page ecrit B_GSM_TASK (bit1) par le chemin ARM.
         */
        static int fd = -1;
        if (fd < 0) { const char *e = getenv("CALYPSO_FORCE_DISPATCH"); fd = (e && atoi(e) > 0) ? 1 : 0; }
        if (fd) {
            /* [2026-07-29] Cellule corrigee : d_dsp_page = 0x08D4 (0x08E2 etait
             * d_dsp_state), et ecriture dans api_ram — c'est ce tableau que la
             * ROM lit pour la plage 0x0800+. La bequille ecrivait donc jusqu'ici
             * une cellule inerte : son effet mesure etait celui du seul DP. */
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
    /* === SBFN-PROBE (read-only, gate CALYPSO_SBFN) ============================
     * Question : quelle trame le tampon DARAM contient-il AU MOMENT ou la tache
     * SB (0x9841) le lit ? calypso_bsp.c:1600 documente que par defaut TOUTES les
     * trames ecrivent 0x2a00, donc ~9 bursts non-FCCH s intercalent entre deux
     * FCCH. Si le SB correle un burst quelconque, le mot SCH sort invalide et le
     * DSP arme B_SCH_CRC a juste titre -- panne de TIMING, pas de traitement.
     * On imprime : fn du dernier depot, fn%51 (SCH = {1,11,21,31,41}), combien de
     * bursts ont ete deposes depuis le SB precedent, et l amplitude du tampon.
     * Aucune ecriture : cette sonde ne peut rien changer au comportement. */
    if (s->pc == 0x9841) {
        static int on = -1;
        if (on < 0) { const char *e = getenv("CALYPSO_SBFN"); on = (e && *e && atoi(e)) ? 1 : 0; }
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
    /* === SUBC-PROBE (lecture seule, gate CALYPSO_SUBC) ========================
     * si.gdb noue toute la cascade sur UN point, le quotient de la division en
     * 16 pas (desassemblage de l appelant) :
     *     0x7d1c  RPT #15
     *     0x7d1d  SUBC *(0x0b), A      ; division
     *     0x7d1e  STL  A, *(0x0a)      ; LE QUOTIENT
     *     0x7d21  LD   *(0x0a), T      ; T est charge ici
     *     0x7d24  CALLD 0x81df         ; sous-programme du MPY 0x81e4
     * quotient nul -> T nul -> produit nul -> blocs 3/4 ecrases -> source des
     * coefficients vide -> le FIRS multiplie par du vide -> B_SCH_CRC arme.
     * Le breakpoint gdb sur data_write n a jamais su la prendre : on la compile.
     *
     * CONTROLE OBLIGATOIRE, dans le meme gate : on compte aussi les passages en
     * 0x989f (branche CRC-mauvais), dont on SAIT qu elle s execute. Si QUOTIENT
     * reste a zero pendant que CONTROLE monte, le silence est un fait mesure ;
     * si les deux sont muets, c est la sonde qui est morte, pas le code. */
    if (s->pc == 0x7d19 || s->pc == 0x7d1b || s->pc == 0x7d1c ||
        s->pc == 0x7d1e || s->pc == 0x81e4 || s->pc == 0x989f) {
        static int on = -1;
        if (on < 0) { const char *e = getenv("CALYPSO_SUBC"); on = (e && *e && atoi(e)) ? 1 : 0; }
        if (on) {
            static unsigned n_q = 0, n_mpy = 0, n_ctl = 0;
            if (s->pc == 0x989f) {
                n_ctl++;
                if (n_ctl <= 3 || (n_ctl % 25) == 0)
                    fprintf(stderr, "[c54x] SUBC-PROBE CONTROLE 0x989f #%u "
                            "(quotient=%u mpy=%u) insn=%u\n",
                            n_ctl, n_q, n_mpy, s->insn_count);
            } else if (s->pc == 0x7d19 || s->pc == 0x7d1b || s->pc == 0x7d1c) {
                /* Le dividende est la constante 1 decalee (ld #1,A ; sfta A,<n>).
                 * Si le decalage laisse A sous le diviseur, le quotient est nul
                 * PAR CONSTRUCTION -- ce ne serait pas un bug du SUBC. On releve
                 * donc A aux trois instants : avant le LD, apres le LD, et juste
                 * avant la division. */
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
    /* === MVDD-PROBE (lecture seule, gate CALYPSO_SUBC) ========================
     * DERNIER SAUT. Mesure etablie : le dividende de la division 0x7d1d n est pas
     * une grandeur physique mais l INDEX du premier mot non nul des blocs 3/4
     * (balayage arriere 0x7cf5..0x7d06, chute sur `xor A` quand rien n est trouve).
     * Les blocs sont vides -> A=0 -> quotient 0 -> T=0 -> coefficients nuls. La
     * fleche de si.gdb etait donc a l envers : ce n est pas T qui ecrase les blocs.
     *
     * Les blocs 3/4 sont censes etre remplis par deux copies de 7 mots :
     *     0x7cda  stm #0x2cce, AR2        ; destination = bloc 3
     *     0x7cdc  stm #0x2c56, AR3        ; source = CORR A
     *     0x7cde  mar *AR3+0              ; AR3 += AR0   <-- decalage
     *     0x7cdf  rpt #6 / 0x7ce0 mvdd
     *     0x7ce1  mar *+AR3(0x2b) / 0x7ce3 rpt #6 / 0x7ce4 mvdd  ; bloc 4
     * et cette copie est SAUTEE par `bcd 0x7ced, ANEQ` en 0x7ccd quand les blocs
     * sont deja non nuls. On imprime donc : passe-t-on par 0x7ccd (et branche-t-on
     * ?), la copie s execute-t-elle, avec quel AR0/AR3, et que vaut la source. */
    if (s->pc == 0x7ccd || s->pc == 0x7ce0 || s->pc == 0x7ce4) {
        static int on = -1;
        if (on < 0) { const char *e = getenv("CALYPSO_SUBC"); on = (e && *e && atoi(e)) ? 1 : 0; }
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
        /* @BEQUILLE — FORCE_DP (+ FORCE_DP_FROM comme scope)  (CALYPSO_FORCE_DP, VALEUR,
         *              defaut OFF)
         *   masque  : le champ DP de ST0 a l'entree du dispatcher est un residu de pile
         *             (over-pop / ST0 non restaure) et non la page de donnees attendue.
         *   retirer : des que la sonde DISP-ENTRY montre le dispatcher OK sans forcage,
         *             c.-a-d. quand l'equilibre de pile ST0 push/pop est sain.
         */
        static int inited = 0, force_dp = -1, force_from = -1;
        if (!inited) {
            inited = 1;
            const char *e = getenv("CALYPSO_FORCE_DP");
            force_dp = (e && *e) ? (int)strtol(e, NULL, 0) : -1;
            const char *ef = getenv("CALYPSO_FORCE_DP_FROM"); /* SCOPÉ : ne force que si DP==FROM */
            force_from = (ef && *ef) ? (int)strtol(ef, NULL, 0) : -1; /* -1 = global (ancien) */
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
            if (lut != 0xff72) {   /* dispatcher BAD → dump ring ST0 push/pop (C-sweep) */
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

    /* DISP-TRACE (CALYPSO_DEBUG=DISP-TRACE) : trace le dispatcher de tâches
     * 0x8341-0x8353 qui calcule la cible CALAD (0x8353 = CALAD A). Le bug :
     * A_L finit = 0x70c3 (garbage) au lieu d'une entrée de la branch-table
     * 0x8359 (B 0x8365/0x8394/...). On veut A à l'ENTRÉE (0x8341) = l'index
     * pré-chargé par l'appelant (sélecteur de tâche / d_task_md). Si A est
     * déjà garbage à 0x8341 → bug upstream confirmé (dispatcher innocent). */
    if (s->pc >= 0x8341 && s->pc <= 0x8354 && calypso_debug_enabled("DISP-TRACE")) {
        static unsigned disp_n = 0;
        if (disp_n++ < 300) {
            /* À 0x834d (op 0x6f07 = LD Smem<<1,A) : calcule l'EA direct exact
             * (DP<<7)|dma et logge la valeur lue — c'est elle qui devient A.
             * Légit = 0xff86 (→ A_L=0x8261) ; corrompu = 0xf6b7 (→ 0x70c3). */
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

    /* INTM-TRANS probe : log toute transition INTM 0→1.
     * Le SSBX INTM orphelin se cache entre insn=89.83M (last write 0x3dd2)
     * et insn=98.38M (entrée wait permanente). Cap à 200 transitions pour
     * éviter le flood au boot ; capture le PC qui a fait passer INTM à 1
     * et l'adresse de retour stack pour identifier le caller. */
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
        /* === MVDD-CASCADE probe (env-gated CALYPSO_PROBE_BOOTSTUB=1) ===
         * PC=0x8e8c op=0xe5ba = MVDD-family — documented cascade writer
         * (`project_dtaskd_corruption_8e8x`) that writes garbage values
         * into NDB cells (random vals at d_fb_det vs legitimate 0x001e).
         * Track AR fields + B accumulator + source address read to find
         * if it's true firmware compute or corrupted indirect addressing. */
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
         * 15× unrolled ADD *AR7+. Called via CALL 0xdfb1 from 0xdf90.
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

        /* === SEED-SOURCE probe (env-gated CALYPSO_PROBE_BOOTSTUB=1) ===
         * Probe at PC=0xf8de (CALA B → 0x7700) — the SINGLE source event
         * that spawns the entire boot-stub RET-loop cascade (per session
         * 2026-05-24 BOOTSTUB-ENTRY analysis : 1 ENTER-7700 → 435 entries
         * to PC=0x0000). Captures full state BEFORE the CALA fires :
         *   - SP + stack contents (what subsequent POPs will pull)
         *   - A, B (B = jump target)
         *   - AR0..AR7, ST0, ST1
         *   - 10-PC trail (extends visibility upstream of 0xf8de).
         * Goal: identify whether the function containing 0xf8de was itself
         * called with proper push, and what was supposed to be on stack
         * when POPM ST0 + RCD UNC fire at dispatcher 0x7706/0x7707. */
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

        /* === BOOTSTUB-ENTRY probe (env-gated CALYPSO_PROBE_BOOTSTUB=1) ===
         * Traces every entry to PC=0x0000 (boot stub LDMM SP,B + RET).
         * Boot stub re-entered at runtime is the documented-never-nailed
         * seed of the SP-wrap → AR6=0 → IMR=0 cascade. Captures :
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
        /* === INT3-VEC-TRACE probe (2026-05-29) ===
         * Trigger à PC=0xFFCC (= INT3 vector entry, IPTR=0x1FF + vec 19*4).
         * Capture les ~32 PCs suivants pour voir le chemin ISR.
         * Objectif : identifier où DSP saute hors path attendu (= soit RSBX
         * INTM dans zone 0xA4D0+, soit retour normal via RETE). Si DSP finit
         * à 0x0000 boot stub → identifier l'opcode/PC qui dérive le saut.
         * Gated par CALYPSO_DEBUG=INT3_VEC ou ALL. */
        {
            static int trace_n = -1;        /* -1 = not active, ≥0 = countdown */
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

        /* D_FB_DET-WR-SITE probe : à PC=0x8f51 (le PC qui écrit d_fb_det).
         * Snapshot AR0..AR7 + data[AR0/1/2] + BK + A pour identifier la
         * zone DARAM lue par le correlator FB-det au moment de produire
         * sa valeur d'output. Comparer la zone source avec le BSP DMA
         * target (default 0x3fb0..0x3fbf) :
         *   - zone source = BSP target → correlator lit bien les samples
         *   - zone source ≠ BSP target → mismatch source/sink, blocker
         *     structurel : DSP attend les samples ailleurs que là où le
         *     BSP les écrit. Suite : tracer init AR, table coeffs, ou
         *     MAC sur autre buffer. */
        /* COEFFS-TABLE-DUMP : 1× au tout début + à chaque sweep FB-det.
         * Dump data[0x2bc0..0x2bcF] (zone censée contenir les coefficients
         * du correlator selon AR4 observé). 2026-05-14 : capture étendue
         * D_FB_DET-WR-SITE a révélé data[AR4]=0x0000 sur 50 hits → la table
         * de coeffs est VIDE en mémoire. Vérifier ici si elle l'est aussi
         * en boot et si quelqu'un l'écrit jamais. */
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
            /* Cap bumpé 50 → 500 (2026-05-14 night) pour couvrir plusieurs
             * sweeps FB-det au lieu du seul premier. + stats agrégées sur
             * tous les fires (cap n'est que pour le log per-fire). */
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
            /* Sweep boundary detection : AR3 retombe en dessous de la
             * dernière valeur observée → nouveau sweep commence.
             * Log le sweep précédent (count non-zero + A final + insn). */
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
            /* Stats summary toutes les 100 fires de 0x8f51 — distribution
             * AR4-in-zone + histogramme val[AR4] sur tout l'historique. */
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
        /* READ-AMONT probe : à chaque trigger PC (sites d_fb_det), émet delta
         * des reads par plage depuis le trigger précédent. Tranche entre :
         *   - dominant LOW    → correlator lit la zone [0..0x3A3]
         *   - dominant APIRAM → samples viennent via API RAM (ARM-driven)
         *   - dominant WRAP   → correlator tourne sur le wrap PROM1 mirror
         *   - dominant OTHER  → zone non cataloguée à identifier */
        read_stats_trigger_check(s);
        throughput_tick(s->insn_count);
        /* WAIT-A21A probe : à PC=0xa21a, snapshot INTM + IMR + IFR.
         * Tranche H1/H2/H3 :
         *   INTM=1 + IFR=0  + IMR plein → H3 strict, hardware silencieux
         *   INTM=1 + IFR≠0  + IMR plein → H3 + IRQ pending bloquée (BUG)
         *   INTM=0                       → H1/H2 (IRQ servable mais path
         *                                  vers 0x7740 cassé en amont) */
        /* === CORR-PUBLISH-A probe (2026-05-28) ===
         * À PC=0x9ac0 (juste avant STL A → *AR2-), snapshot A complet +
         * AR2 (= adresse de publication). Cap 200. */
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
        /* CALLER-7740 tracer : à l'entrée 0x7740, log le contexte caller.
         * data[sp] = adresse de retour pushée par le CALL/CALLD précédent.
         * INTM=1 → on est dans un IRQ context. Permet de distinguer
         * "appelé via IRQ ISR" vs "appelé via flow régulier", et de
         * remonter la chaîne caller→callee jusqu'à l'IRQ vector. */
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
            else if (cc == 0x70) cond = (s->st0 & ST0_OVA) != 0;     /* AOV */
            else if (cc == 0x60) cond = !(s->st0 & ST0_OVA);          /* ANOV */
            else if (cc == 0x78) cond = (s->st0 & ST0_OVB) != 0;     /* BOV */
            else if (cc == 0x68) cond = !(s->st0 & ST0_OVB);          /* BNOV */
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

        /* F4E2 = RSBX INTM (enable interrupts), F4E3 = SSBX INTM (disable interrupts) */
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
            /* [2026-07-26 WF golive-mac] FB-ENERGY REROUTE : le dispatch FB actif
             * (CALA @0xb01e) resout vers le correlateur SYMBOLE 0x8d00 / stub 0xab38,
             * qui ne touche jamais le buffer IQ 0x2a00 ni le kernel 0xa076. On
             * redirige la CALA vers l entree du correlateur ENERGIE FB : 0x94f5
             * (0x9500 pose AR4=0x2a00 -> f274 a033 -> a040 -> f273 a076). NB : sur ce
             * chemin AR5=0x2c00 (reference), l IQ 0x2a00 est en AR4/AR1 (PAS AR5).
             * Gate CALYPSO_FB_ENERGY ; entree override CALYPSO_FB_CORR_ENTRY. */
            if (is_call && src_pc == 0xb01e) {
                /* [2026-07-27] CALA-FB : cible NATIVE du dispatcher + d_task_md,
                 * loggee AVANT tout reroute (voir en-tete du patch). */
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
                     *                CALYPSO_FB_CORR_ENTRY, defaut OFF)
                     *   masque  : l absence de DISPATCH NATIF vers le correlateur.
                     *             On REROUTE l execution vers 0x9500 / 0x94f5 au lieu
                     *             de laisser le firmware y arriver par son chemin.
                     *   retirer : quand BRINT0 (vec 21) est servie et que le chemin
                     *             natif atteint le correlateur seul.
                     *   ⚠️ Toute mesure prise sous ce reroute est une mesure SOUS
                     *   BEQUILLE : en natif pur, cet etage n est JAMAIS execute
                     *   (mesure 2026-07-28 : CALYPSO_WATCH_9F00_RD = 0). */
                    const char *_e = getenv("CALYPSO_FB_ENERGY"); _fbe = (_e && atoi(_e) > 0) ? 1 : 0;
                    const char *_p = getenv("CALYPSO_FB_CORR_ENTRY");
                    if (_p && *_p) _fbentry = (uint16_t)strtol(_p, NULL, 0);
                }
                if (_fbe && s->data[0x058a] == 5) {   /* d_task_md == 5 (commande FB) */
                    static unsigned _fbn = 0;
                    if (_fbn++ < 32)
                        fprintf(stderr, "[c54x] FB-ENERGY-REROUTE CALA@0xb01e tgt 0x%04x -> 0x%04x insn=%u\n",
                                tgt, _fbentry, s->insn_count);
                    tgt = _fbentry;
                }
            }
            /* SURGICAL 2026-05-30 : self-CALA black-hole capture. Fire UNE
             * fois quand un CALA cible lui-même dans la zone 0x7000-0x70FF
             * (= le trou noir 0x70c3). Donne le DP hérité + le slot LUT lu
             * au dispatcher 0x834d (ea/val) + le POPM ST0 et le LDP qui ont
             * posé ce DP — le coupable complet, sans spam (≠ DISP-TRACE). */
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
                    /* Dump pile autour de SP : montre l'orphelin (0xf487) et
                     * ses voisins. Si le vrai ST0 (genre 0x0xxx/0x4xxx, DP
                     * plausible) est à SP±1, c'est un imbalance d'1 mot. Les
                     * mots PC-shaped (0xf4xx/0x7xxx) empilés = drain cumulatif. */
                    fprintf(stderr, "[c54x]     STACK around SP=0x%04x :", s->sp);
                    for (int k = -2; k <= 9; k++) {
                        uint16_t a = (uint16_t)(s->sp + k);
                        fprintf(stderr, " %s[%04x]=%04x",
                                k == 0 ? ">" : "", a, s->data[a]);
                    }
                    fprintf(stderr, "\n");
                    /* SP-event ring : les 28 derniers push/pop (pc:op delta).
                     * Cherche un push (delta<0) sans pop apparié = la fuite. */
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
        /* F4E6 = FBACC A   FL_FAR  (far branch acc, no push, no delay)
         * F4E7 = FCALA A   FL_FAR  (far call  acc, push 2 mots, no delay)
         * F5E6 = FBACC B / F5E7 = FCALA B  (acc B variants)
         *
         * Per binutils tic54x-opc.c (FL_FAR flag) and SPRU172C :
         *   XPC = A(22:16), PC = A(15:0). FCALA push XPC puis ret_pc (PC+1),
         *   ordre compatible avec FRET (F4E4 — pop PC d'abord puis XPC).
         *
         * 2026-05-27 (c web review): non-delayed variants WERE NOP-fallthrough
         * via the F4E0-F4FF block below. Pre-XPC-fix le code far-acc n'était
         * jamais atteint, post-fix il l'est → silent control-flow derailment.
         * Sémantique identique au FCALAD/FBACCD (F6E6/F6E7) existant, sans
         * delay slots. */
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
                /* FCALA : push XPC first (deeper in stack), then ret_pc (top).
                 * FRET (F4E4) pops PC d'abord puis XPC — ordre compatible. */
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
        /* F4E0-F4FF catch-all : NOP par défaut, sauf opcodes connus qui ont
         * leur handler dédié. Comment historique "RSBX/SSBX" était faux
         * (RSBX=F4B0, SSBX=F5B0 hors range). En fait ce range contient :
         *   F4E1: IDLE 1            ← exception (handler ligne ~4058)
         *   F4E2: BACC A            (handler ligne ~3920)
         *   F4E3: CALA A            (handler ligne ~3920)
         *   F4E4: FRET              ← exception (handler ligne ~4041)
         *   F4E6: FBACC             (handler ligne ~3974)
         *   F4E7: FCALA             (handler ligne ~3974)
         *   F4EB: RETE              ← exception (handler ligne ~4012)
         * Sans l'exception F4E1, IDLE 1 était silencieusement avalé en NOP,
         * empêchant DSP de signaler s->idle=true → IRQ handler ne dispatchait
         * jamais → INTM stuck à 1 (2026-05-29 fix). */
        if (op >= 0xF4E0 && op <= 0xF4FF &&
            op != 0xF4E1 && op != 0xF4E4 && op != 0xF4EB) {
            return consumed + s->lk_used;
        }
        /* F4EB = RETE (return from interrupt). Pop PC, pop XPC iff APTS=1.
         * Symmetric with c54x_interrupt_ex push order. */
        if (op == 0xF4EB) {
            uint16_t prev_xpc = s->xpc;
            /* [2026-07-23] IT return SYMETRIQUE : l'entree IT (level_check l.4229 /
             * interrupt_ex) pousse XPC SEULEMENT si xpc!=0 (mode etendu). Un XPC
             * pousse vaut 0..3 ; un PC (0x0080+) est >3. Donc RETE ne depile XPC
             * QUE si le sommet est un XPC valide (<=3) -> pop 1 quand PC seul pousse
             * (xpc=0), pop 2 en mode etendu. Corrige le drift SP +1/IT (over-pop) qui
             * faisait deriver SP -> RETE @0x0107 lisait PC-comme-XPC + garbage-comme-PC
             * -> PC=0 (derail go-live cycle). L'ancien pop-2-inconditionnel supposait
             * push-2 toujours, faux depuis le fix push-XPC-conditionnel (l.4219). */
            /* [2026-07-30] @BEQUILLE/CORRECTIF — CALYPSO_RETE_POP2 (defaut 0).
             *
             * Le commentaire ci-dessus decrit une ENTREE QUI N'EXISTE PLUS. Il
             * suppose « l'entree IT pousse XPC seulement si xpc!=0 ». Or les deux
             * chemins d'entree (calypso_c54x.c:17694 et :17755) font, textuellement :
             *     s->sp--; data_write(s, s->sp, s->pc + 1);
             *     s->sp--; data_write(s, s->sp, s->xpc);   // « save XPC INCONDITIONNEL »
             *     g_sp_ledger.net_words += 2;
             * L'empilement est donc TOUJOURS de 2 mots, et le depilement est reste
             * conditionnel, sur une heuristique (« le sommet ressemble-t-il a un
             * XPC ? »). Un push inconditionnel face a un pop devine ne peut pas etre
             * symetrique dans tous les cas.
             *
             * Mesure du 30/07, banc de REFERENCE sans aucune bequille :
             *   ORPHAN-RETURN pc=0x0107 op=0xf4eb SP=0x5aa8 -> ret_tgt=0xddfb
             *   net_words=-18880, -4 mots PAR TRAME (evenements espaces de 65536
             *   insn a l'unite pres), « over-pop (pile vierge au-dessus de SP_base) ».
             * Et `0x0107` est le retour de l'IT DE TRAME elle-meme : le contexte est
             * abime une fois par trame, donc aucune tache ne peut etre portee d'une
             * trame a la suivante.
             *
             * =1 : depile 2 mots INCONDITIONNELLEMENT, en miroir exact de l'entree.
             * Defaut 0 (heuristique historique) : ce correctif touche le retour
             * d'interruption de TOUS les profils, et le depot documente qu'un bug de
             * cette famille « verrouille tout le projet depuis ~6 mois ». On mesure
             * avant de le rendre standard : CALYPSO_RETE_POP2=1 + CALYPSO_ORPHAN=1,
             * la sonde doit se TAIRE et net_words rester borne.
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
                s->xpc = top & 3; s->sp++;  /* pop XPC — toujours, comme le push */
            } else if (top <= 3) {          /* heuristique historique */
                s->xpc = top & 3; s->sp++;
            }
            uint16_t ra = data_read(s, s->sp); s->sp++;   /* pop PC */
            s->st1 &= ~ST1_INTM;

            /* [2026-07-30] RETE-AUDIT (CALYPSO_RETE_AUDIT=1, defaut 0).
             *
             * La question qui reste apres avoir innocente le depilement lui-meme :
             * y a-t-il PLUS DE RETOURS QUE D'ENTREES ? L'entree d'IT empile 2 mots
             * (calypso_c54x.c:17694 et :17755, « save XPC inconditionnel »), la RETE
             * en depile 2 — donc, a nombre egal, l'equilibre est parfait. Or ORPHAN
             * mesure « over-pop, pile vierge au-dessus de SP_base », -4 mots par
             * trame, invariant. Une RETE executee SANS interruption prealable
             * depilerait exactement 2 mots que personne n'a poses ; deux par trame
             * donneraient les -4 constates.
             *
             * On compte donc les deux cotes et on imprime l'ecart. `irq_entries` est
             * deja tenu par le ledger a chaque dispatch reel. Plafond : une ligne
             * toutes les 500 RETE, 40 lignes au total.
             */
            {
                static int _ra_g = -1; static unsigned long long _ra_n = 0;
                static unsigned _ra_log = 0;
                if (_ra_g < 0) _ra_g = calypso_gate("CALYPSO_RETE_AUDIT", 0);
                if (_ra_g) {
                    static unsigned long long _ra_vec = 0;   /* RETE en zone VECTEURS */
                    _ra_n++;
                    /* [2026-07-30, v2] SEPARER LES DEUX USAGES DE RETE.
                     * v1 comparait TOUTES les RETE a irq_entries et criait « plus de
                     * retours que d'entrees » : regle mal posee. Mesure : rete=500
                     * pour irq_entries=167, or notre CALA (0xF4E3) n'empile QU'UN mot
                     * (l'adresse de retour) et la ROM s'en sert abondamment. La
                     * plupart des RETE reviennent donc d'un handler appele par CALA,
                     * pas d'une interruption — c'est legitime, et l'heuristique de
                     * depilement le gere correctement : un XPC empile vaut 0..3, une
                     * adresse de retour vaut >= 0x0080, donc le test `top <= 3`
                     * discrimine les deux cas et depile 2 ou 1 a bon escient.
                     *
                     * Le seul cas suspect est celui qu'ORPHAN designe : une RETE dont
                     * le PC est dans la ZONE DES VECTEURS (pc=0x0107), donc un retour
                     * d'INTERRUPTION — et « pile vierge au-dessus de SP_base », donc
                     * sans entree appariee. C'est CELUI-LA qu'il faut compter, et lui
                     * seul, face a irq_entries. Zone : pc < 0x0200 (table post-boot en
                     * 0x0080, slots de 4 mots, plus les tremplins OVLY). */
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
        /* 0xF4E4 = FRET (far return). Pop PC + XPC unconditionally.
         * Per binutils tic54x-opc.c (FL_FAR flag) and SPRU172C Table 2-15:
         *   FRET[D]: XPC = TOS, ++SP, PC = TOS, ++SP
         * Symmetric with FCALL/FCALLD push (also unconditional, see below).
         * 2026-04-28 — fixed: was conditional on PMST_APTS (bit 4) which is
         * actually AVIS (Address Visibility) per SPRU131G — has no stack
         * semantics. The misnomer caused FRET to skip XPC pop when AVIS=0,
         * leading to stack imbalance against FCALL FAR which always pushes 2. */
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
            if (nx > 3)   /* PROM0..3 = 4 pages ; page 3 legitime (masque & 3) */
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
             * [2026-08-22] FIX : AFFECTE, n'accumule PAS.
             * SPRU172C : `MPYA dst` -> dst = T x A(32-16). C'est un MULTIPLY ;
             * MAC accumule, MPY/MPYA affecte. L'ancien code faisait
             * `dst = dst + T*AH` — incoherent avec ses deux voisins immediats :
             * la forme Smem `MPYA Smem` (0x3100) fait `s->b = sext40(prod)`, et
             * `SQUR A,dst` (0xF48D) juste dessous aussi. Le commentaire disait
             * « accumulate into dst » : l'intention etait deja fausse.
             * OU CA MORD : chaine du TOA en PROM0
             *   0x7944 add *AR4,A ; 0x7945 sub #2,A ; 0x7947 mpya A ; 0x7948 add B
             *   ... 0x795a stl B -> a_sync_demod[D_TOA]
             * A est non nul en 0x7947, donc accumuler fausse le TOA publie.
             * (En 0x7920 `mpya B` l'ecart est nul : B y vaut d_fb_mode = 0.)
             * Bascule A/B : CALYPSO_ISA_MPYA_ASSIGN=0 restaure l'accumulation. */
            if ((op & 0xFEFF) == 0xF48C) {
                static int _mpa = -1;
                if (_mpa < 0) {
                    _mpa = calypso_gate("CALYPSO_ISA_MPYA_ASSIGN", 1);
                    fprintf(stderr, "[c54x] ISA-MPYA %s : MPYA dst = T x A(32-16)\n",
                            _mpa ? "AFFECTE (fidele SPRU172C)"
                                 : "ACCUMULE (ancien comportement)");
                }
                int dst = (op >> 8) & 1;
                int64_t prod = (int64_t)(int16_t)s->t * (int64_t)(int16_t)((s->a >> 16) & 0xFFFF);
                if (s->st1 & ST1_FRCT) prod <<= 1;
                int64_t res = _mpa ? prod
                                   : ((dst ? s->b : s->a) + prod);
                if (dst) s->b = sext40(res); else s->a = sext40(res);
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

            /* F486/F586: MAX src (mask FEFF, 1 word) — keep max of A,B
             * F-AUDIT 2026-05-25 v5 : était à 0xF492 (= roltc per binutils).
             * binutils tic54x-opc.c : "max" 1,1,1, 0xF486, 0xFEFF
             * → constant moved from 0xF492 to 0xF486 (impl est correct). */
            if ((op & 0xFEFF) == 0xF486) {
                /* [2026-09-17] FIX_MAXMIN_DST — MAX dst (bit 8 : 0=A 1=B),
                 * SPRU172C 4-99 : dst = max(A,B) ; C=0 si le max est A, C=1 sinon.
                 * L'ancien code ignorait le bit de destination : il ecrivait
                 * TOUJOURS A, donc « max B » ne mettait jamais B a jour. C'est ce
                 * qui figeait l'argmax du correlateur SCH (0x84e8 `max B`, B restait
                 * 0) -> pic au bord (index 43) -> 78 bits mal cadres -> CRC SB faux. */
                int64_t sa = sext40(s->a), sb = sext40(s->b);
                int a_is_max = (sa >= sb);
                int64_t mx = a_is_max ? sa : sb;
                if ((op >> 8) & 1) s->b = sext40(mx); else s->a = sext40(mx);
                if (a_is_max) s->st0 &= ~ST0_C; else s->st0 |= ST0_C;
                return consumed + s->lk_used;
            }

            /* F487/F587: MIN src (mask FEFF, 1 word) — keep min of A,B
             * F-AUDIT 2026-05-25 v5 : était à 0xF493 (= cmpl per binutils).
             * binutils : "min" 1,1,1, 0xF487, 0xFEFF
             * → constant moved from 0xF493 to 0xF487. */
            if ((op & 0xFEFF) == 0xF487) {
                /* [2026-09-17] FIX_MAXMIN_DST — MIN dst (bit 8), SPRU172C 4-100 :
                 * dst = min(A,B) ; C=0 si le min est A, C=1 sinon. Meme bug de
                 * destination ignoree que MAX ci-dessus. */
                int64_t sa = sext40(s->a), sb = sext40(s->b);
                int a_is_min = (sa <= sb);
                int64_t mn = a_is_min ? sa : sb;
                if ((op >> 8) & 1) s->b = sext40(mn); else s->a = sext40(mn);
                if (a_is_min) s->st0 &= ~ST0_C; else s->st0 |= ST0_C;
                return consumed + s->lk_used;
            }

            /* ⛔ [2026-08-04] BLOC DEPLACE — voir c54x_mac_bit_family().
             * Ces handlers etaient ici, dans `case 0xF:`, donc INATTEIGNABLES
             * (leurs opcodes ont hi4 = 2 ou 3). Les corps vivent desormais dans
             * le helper, appele depuis `case 0x2:` et `case 0x3:`. Ne rien
             * remettre ici : ce serait a nouveau du code mort. */
            /* ⛔ [2026-08-04] BITT ETAIT ICI — RETIRE, ET C'EST VOULU.
             * `bitt` = 0x34xx donc hi4 = 3 : dans `case 0xF:` il etait
             * INATTEIGNABLE. Le handler vivant est dans `case 0x3:`
             * (chercher FIX_BITT_CASE3). La sonde BITT-WATCH etait elle aussi
             * piegee dans ce bloc mort — d'ou son silence TOTAL au run du
             * 04/08, pas meme sa trace d'armement.
             * Le corps est RETIRE et non commente, pour que `sweep_reach.py`
             * retombe a ZERO suspect : un controle en permanence rouge finit
             * par etre ignore, et on perdrait la capacite de reperer le
             * PROCHAIN handler egare. */

            /* F492/F592: ROLTC src (rotate left through TC, mask FEFF, 1 word)
             * F-AUDIT 2026-05-25 v5 : NOUVEAU handler. binutils :
             * "roltc" 1,1,1, 0xF492, 0xFEFF — était mis-décodé en MAX.
             * Semantic SPRU172C : src bit 31 → TC, src << 1, src bit 0 ← TC_old.
             * Bug observé pré-fix : A_low devenait 0 systématiquement via le
             * faux MAX (A=B if A<B) à PC=0x9abf, causant cascade STL→IMR=0. */
            if ((op & 0xFEFF) == 0xF492) {
                int src = (op >> 8) & 1;
                int64_t *acc = src ? &s->b : &s->a;
                int64_t v = *acc & 0xFFFFFFFFFFLL;
                int new_tc = (int)((v >> 31) & 1);
                int old_tc = (s->st0 & ST0_TC) ? 1 : 0;
                *acc = sext40(((v << 1) | (int64_t)old_tc) & 0xFFFFFFFFFFULL);
                if (new_tc) s->st0 |= ST0_TC; else s->st0 &= ~ST0_TC;
                /* [2026-08-04] BITT-WATCH, patte 2/2 — meme gate, meme fenetre.
                 * Montre si le TC pose par `bitt` ENTRE reellement dans
                 * l'accumulateur : `apres` doit valoir `avant<<1 | old_tc`.
                 * Si `old_tc` est toujours 0 alors que la patte 1/2 compte des
                 * TC=1, la faute est entre les deux (TC ecrase par une
                 * instruction intercalee). Si les deux pattes voient TC=0, la
                 * source est vide et le probleme est en amont. */
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
            /* [2026-09-17] FIX_NORM_SD — NORM est « norm src[,dst] », opcode 0xF48F
             * masque 0xFCFF (tic54x-opc.c:378) : bit 9 = src, bit 8 = dst, donc
             * QUATRE encodages F48F/F58F/F68F/F78F. Le masque 0xFEFF ci-dessous
             * n'attrapait que F48F/F58F (et prenait F58F pour NORM B alors que
             * c'est NORM A,B) ; F68F/F78F tombaient dans le bloc F7 « LD #k8 »
             * (inexistant dans l'ISA) : 0x75cf `f78f` NORM B ecrivait T=0xff8f,
             * BRC devenait 0xff88 et la division du FB (0x75db SUBC) tournait
             * 65000 fois par trame. Mesure c54x_exe --arm 2026-09-17, trace pas a
             * pas. PROM0..3 : 78 x f78f, 157 x f48f. Gate CALYPSO_FIX_NORM_SD
             * (defaut ON) ; =0 rend l'ancien masque. */
            static int fix_norm_sd = -1;
            if (fix_norm_sd < 0) fix_norm_sd = calypso_gate("CALYPSO_FIX_NORM_SD", 1);
            if (fix_norm_sd ? ((op & 0xFCFF) == 0xF48F) : ((op & 0xFEFF) == 0xF48F)) {
                int src, dst;
                if (fix_norm_sd) c54x_f4_srcdst(op, &src, &dst);
                else { src = (op >> 8) & 1; dst = src; }
                int64_t val = sext40(src ? s->b : s->a);
                int bit39 = (val >> 39) & 1;
                int bit38 = (val >> 38) & 1;
                /* [2026-08-22] FIX_NORM_T — le vrai C54x NORM decale l'accu de T
                 * (exposant produit par EXP) en un cycle, SANS ecrire T (SPRU172C).
                 * L'ancien modele decalait 1 bit + T-- -> les paires exp;norm ISOLEES
                 * (reference correlateur 0x796c/796d, division SNR 0x79b1) ne recalaient
                 * jamais en pleine echelle -> reference {0,-1}, SNR minuscule. Prouve par
                 * le firmware : 0x79ca stm #1->T puis rptb norm A = decalage variable
                 * data-dependant, n'a de sens que si NORM decale de T. Meme classe que
                 * FIX_SFTA_CARRY. Gate CALYPSO_FIX_NORM_T (defaut ON) ; =0 = ancien 1-bit. */
                static int fix_norm = -1;
                if (fix_norm < 0) fix_norm = calypso_gate("CALYPSO_FIX_NORM_T", 1);
                if (fix_norm) {
                    int16_t t = (int16_t)s->t;               /* compte signe */
                    if (t >= 0) val = sext40(val << t);
                    else        val = sext40(val >> (-t));   /* decalage arithmetique (SXM) */
                    if (dst) s->b = val; else s->a = val;
                    /* NE PAS modifier T (SPRU172C : NORM lit T, ne l'ecrit pas) */
                } else if (bit39 != bit38) {
                    val = sext40(val << 1);
                    if (dst) s->b = val; else s->a = val;
                    s->t = (uint16_t)(s->t - 1);
                }
                if (bit39 != bit38) s->st0 |= ST0_TC;
                else                s->st0 &= ~ST0_TC;
                return consumed + s->lk_used;
            }

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

            /* F493/F593: CMPL src (complement, mask FCFF, 1 word)
             * F-AUDIT 2026-05-25 v5 : était à 0xF486. binutils :
             * "cmpl" 1,1,2, 0xF493, 0xFCFF, {OP_SRC,OPT|OP_DST}
             * → constant moved from 0xF486 to 0xF493. Mask FEFF→FCFF
             * (= permet variant SRC=B via bit 9). */
            if ((op & 0xFCFF) == 0xF493) {
                int src = (op >> 8) & 1;
                int64_t *acc = src ? &s->b : &s->a;
                *acc = sext40(~(*acc) & 0xFFFFFFFFFFULL);
                return consumed + s->lk_used;
            }

            /* F49F/F59F: RND src (round, mask FCFF, 1 word)
             * F-AUDIT 2026-05-25 v5 : était à 0xF487. binutils :
             * "rnd" 1,1,2, 0xF49F, 0xFCFF, {OP_SRC,OPT|OP_DST} */
            if ((op & 0xFCFF) == 0xF49F) {
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
                /* SFTA — corps factorise dans c54x_sfta_exec() (FIX_SFTA_CARRY).
                 * Deux sites identiques existaient ; les garder separes les aurait
                 * fait diverger au premier correctif applique a un seul. */
                c54x_sfta_exec(s, op);
                return consumed + s->lk_used;
            }

            /* [2026-07-03] FIX-SFTL-RSBX-COLLISION (gated CALYPSO_FIX_SFTL_RSBX,
             * default OFF). Per doc/opcodes/tic54x_hi8_map.md:141-143, hi8 0xF4-0xF7
             * base add/shift pattern (mask 0xFCE0/0xF400) explicitly excludes RSBX
             * (0xF4B0/0xFDF0) and SSBX (0xF5B0/0xFDF0): nibble low-byte high-nibble
             * == 0xB (bits 7:4 = 1011) is ALWAYS rsbx/ssbx for every hi8 in
             * {F4,F5,F6,F7} (bit9=ST0/ST1, bit8=rsbx/ssbx), NEVER a legal SFTL shift
             * amount. This check's mask (0xFCE0) leaves bit4 don't-care and is not
             * gated by hi8, so it silently swallows 0xF4Bx/F5Bx/F6Bx/F7Bx (incl.
             * RSBX INTM=0xF6BB, SSBX INTM=0xF7BB) as a bogus accumulator shift
             * BEFORE the real hi8==0xF6/0xF7 handlers further down ever run.
             * Independent of and orthogonal to CALYPSO_FIX_MVDM -- does not touch
             * that opcode family. Default OFF: behavior unchanged unless enabled. */
            {
                static int fix_sftl_rsbx = -1;
                if (fix_sftl_rsbx < 0)
                    fix_sftl_rsbx = calypso_gate("CALYPSO_FIX_SFTL_RSBX", 0);
                /* NATIF 2026-07-20 : RSBX/SSBX (low-byte nibble 0xB = 0x?Bx) ne sont
                 * JAMAIS un shift accumulator legal (cf binutils tic54x-opc.c). Exclusion
                 * INCONDITIONNELLE du pattern shift -> ils tombent dans leur vrai handler
                 * RSBX/SSBX. C etait CALYPSO_FIX_SFTL_RSBX (default OFF) -> rendu natif.
                 * Sans ca, RSBX INTM=0xF6BB etait avale en shift bidon -> INTM jamais clear. */
                if ((op & 0xFCE0) == 0xF4A0 &&
                    (op & 0xF0) != 0xB0) {
                    /* SFTL src,shift,dst — logical shift accumulator */
                    int src, dst; c54x_f4_srcdst(op, &src, &dst);   /* FIX_F4XX_SRCDST */
                    int shift = op & 0x1F; if (shift > 15) shift -= 32;
                    uint64_t uv = (uint64_t)((src ? s->b : s->a) & 0xFFFFFFFFFFULL);
                    if (shift >= 0) uv <<= shift; else uv >>= (-shift);
                    uv &= 0xFFFFFFFFFFULL;
                    if (dst) s->b = sext40(uv); else s->a = sext40(uv);
                    return consumed + s->lk_used;
                }
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

            /* Note: 0xF4E4 = IDLE (handled above, not FRET).
             * Real FRET = 0xF072 (algebraic), handled in F0xx section. */
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
              /* [2026-08-23] MASQUE ELARGI. binutils : norm 0xF48F/0xFCFF,
               * {OP_SRC, OP_DST} -- bit 9 = source, bit 8 = destination, donc
               * QUATRE encodages : 0xF48F 0xF58F 0xF68F 0xF78F. Le test etait
               * `(op & 0xFEFF)`, qui ne liberait que le bit 8 : 0xF68F et
               * 0xF78F passaient a travers sans etre decodes.
               * Mesure PROM0 : 29 NORM, 19 captures, 10 RATES -- dont 0x75cf
               * (sequence EXP/NORM qui fabrique B juste avant 0x75d3 STLM B,BRC,
               * d ou le BRC absurde et la boucle de ~50000 tours sur le SUBC),
               * 0x796d et 0x79ad (reference du correlateur, division SNR), et
               * 0x9a08 0x9a27 0x9a4c EN ZONE VITERBI.
               * Meme classe que RPTZ 0xF171. Gate CALYPSO_ISA_NORM_MASK.
               * ⚠️ EFFET GLOBAL : correlateur, division, Viterbi. */
              {
                  static int _nm = -1;
                  if (_nm < 0) {
                      _nm = calypso_gate("CALYPSO_ISA_NORM_MASK", 1);
                      fprintf(stderr, "[c54x] ISA-NORM-MASK %s : norm = 0xF48F/0xFCFF "
                              "(4 encodages, bit9=src bit8=dst) au lieu de 0xFEFF "
                              "qui en ratait 10 sur 29 dans PROM0\n",
                              _nm ? "ACTIF" : "INACTIF (masque etroit)");
                  }
                  if (_nm && (op & 0xFCFF) == 0xF48F && (op & 0xFEFF) != 0xF48F) {
                      /* les deux encodages que l ancien masque ratait */
                      int nsrc = (op >> 9) & 1, ndst = (op >> 8) & 1;
                      int64_t nv = sext40(nsrc ? s->b : s->a);
                      int b39 = (nv >> 39) & 1, b38 = (nv >> 38) & 1;
                      static int fnt = -1;
                      if (fnt < 0) fnt = calypso_gate("CALYPSO_FIX_NORM_T", 1);
                      if (fnt) {
                          int16_t t = (int16_t)s->t;
                          nv = (t >= 0) ? sext40(nv << t) : sext40(nv >> (-t));
                          if (ndst) s->b = nv; else s->a = nv;
                      } else if (b39 != b38) {
                          nv = sext40(nv << 1);
                          if (ndst) s->b = nv; else s->a = nv;
                          s->t = (uint16_t)(s->t - 1);
                      }
                      if (b39 != b38) s->st0 |= ST0_TC; else s->st0 &= ~ST0_TC;
                      return consumed + s->lk_used;
                  }
              }
              if ((op & 0xFEFF) == 0xF48F) {
                  int src = (op >> 8) & 1;
                  int64_t val = sext40(src ? s->b : s->a);
                    int bit39 = (val >> 39) & 1;
                    int bit38 = (val >> 38) & 1;
                    /* [2026-08-22] FIX_NORM_T (copie 2, cf. ~7972) — NORM decale de T,
                     * pas 1 bit, et n'ecrit pas T. Gate CALYPSO_FIX_NORM_T (defaut ON). */
                    static int fix_norm_2 = -1;
                    if (fix_norm_2 < 0) fix_norm_2 = calypso_gate("CALYPSO_FIX_NORM_T", 1);
                    if (fix_norm_2) {
                        int16_t t = (int16_t)s->t;
                        if (t >= 0) val = sext40(val << t);
                        else        val = sext40(val >> (-t));
                        if (src) s->b = val; else s->a = val;
                    } else if (bit39 != bit38) {
                        val = sext40(val << 1);
                        if (src) s->b = val; else s->a = val;
                        s->t = (uint16_t)(s->t - 1);
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
                    /* SFTA — corps factorise dans c54x_sfta_exec() (FIX_SFTA_CARRY).
                     * Deux sites identiques existaient ; les garder separes les aurait
                     * fait diverger au premier correctif applique a un seul. */
                    c54x_sfta_exec(s, op);
                    return consumed + s->lk_used;
                }
                            /* [2026-07-03] FIX-SFTL-RSBX-COLLISION (gated CALYPSO_FIX_SFTL_RSBX,
                 * default OFF). Per doc/opcodes/tic54x_hi8_map.md:141-143, hi8 0xF4-0xF7
                 * base add/shift pattern (mask 0xFCE0/0xF400) explicitly excludes RSBX
                 * (0xF4B0/0xFDF0) and SSBX (0xF5B0/0xFDF0): nibble low-byte high-nibble
                 * == 0xB (bits 7:4 = 1011) is ALWAYS rsbx/ssbx for every hi8 in
                 * {F4,F5,F6,F7} (bit9=ST0/ST1, bit8=rsbx/ssbx), NEVER a legal SFTL shift
                 * amount. This check's mask (0xFCE0) leaves bit4 don't-care and is not
                 * gated by hi8, so it silently swallows 0xF4Bx/F5Bx/F6Bx/F7Bx (incl.
                 * RSBX INTM=0xF6BB, SSBX INTM=0xF7BB) as a bogus accumulator shift
                 * BEFORE the real hi8==0xF6/0xF7 handlers further down ever run.
                 * Independent of and orthogonal to CALYPSO_FIX_MVDM -- does not touch
                 * that opcode family. Default OFF: behavior unchanged unless enabled. */
                {
                    static int fix_sftl_rsbx2 = -1;
                    if (fix_sftl_rsbx2 < 0)
                        fix_sftl_rsbx2 = calypso_gate("CALYPSO_FIX_SFTL_RSBX", 0);
                    if ((op & 0xFCE0) == 0xF4A0 &&
                        (fix_sftl_rsbx2 == 0 || (op & 0xF0) != 0xB0)) {
                        /* SFTL src,shift,dst — logical shift accumulator */
                        int src, dst; c54x_f4_srcdst(op, &src, &dst);   /* FIX_F4XX_SRCDST */
                        int shift = op & 0x1F; if (shift > 15) shift -= 32;
                        uint64_t uv = (uint64_t)((src ? s->b : s->a) & 0xFFFFFFFFFFULL);
                        if (shift >= 0) uv <<= shift; else uv >>= (-shift);
                        uv &= 0xFFFFFFFFFFULL;
                        if (dst) s->b = sext40(uv); else s->a = sext40(uv);
                        return consumed + s->lk_used;
                    }
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
             * [2026-08-03] FIX_F1XX_ALU_LK — SAS (CALYPSO_FIXES=FIX_F1XX_ALU_LK).
             *
             * LE DEFAUT. Les handlers ALU a immediat long existent, mais ils sont
             * IMBRIQUES dans `if (hi8 == 0xF2)` (l.~8241) et `if (hi8 == 0xF3)`
             * (l.~8576). Un opcode `0xF1xx` ne peut donc jamais les atteindre, et
             * ce bloc-ci ne contient AUCUN test de masque FCF0/FCE0/FCFF — que des
             * correspondances exactes (F072/F073/F074...). La famille
             * ADD/SUB/LD/AND/OR/XOR #lk avec DST=B (bit 8) n'est donc pas decodee.
             *
             * ⚠️ LE COMMENTAIRE JUSTE EN DESSOUS AFFIRME L'INVERSE — il dit que les
             * 0xF1xx « tombent dans » les masques FCE0/FCFF/FCF0 « L3915/L3852/
             * L3886 ». C'est FAUX : ces handlers sont sous les gardes 0xF2/0xF3.
             * Le commentaire decrit une intention, pas le code. Il est conserve
             * tel quel plus bas comme piece a conviction (meme motif que les deux
             * commentaires XPCWATCH pris en defaut le 03/08).
             *
             * MESURE QUI L'ETABLIT (sonde CHAIN-B05F, dispatcher de tache) :
             *     0xb060  LD          A = 0x000018   (correct)
             *     0xb062  f130 7fff   A = 0x005294   <- A ecrase
             *     0xb066  f843     -> 0xb077          bailout, 33/33 passages
             * `0xF130` devrait etre `AND #0x7fff, A, B` (subop=3, src_b=0 -> A,
             * dst_b=1 -> B) : il doit ecrire B et laisser A INTACT.
             *
             * ⚠️ CE QUI RESTE INEXPLIQUE, a ne pas masquer : la provenance exacte
             * de `0x5294` (constante, independante de l'entree). Un opcode non
             * decode devrait laisser A tranquille, pas y ecrire une constante. Ce
             * correctif rend le decodage CORRECT ; il ne prouve pas qu'il etait la
             * seule cause. Si `A` continue d'etre ecrase avec le fix actif, le
             * fallback lui-meme est en cause et il faudra l'instrumenter.
             *
             * PORTEE. Ce correctif touche TOUTES les instructions 0xF1xx du
             * firmware, pas seulement le dispatcher de tache — d'ou le sas plutot
             * qu'un correctif direct. Protocole `environnement/fixes.env` : valider
             * SOUS CHARGE (camp + LU + SMS), puis effacer LA CONDITION, pas le
             * correctif. Trois niveaux de validation exiges par le fichier :
             * formel (binutils/SPRU172C), grandeur physique, chemin fonctionnel.
             *
             * L'implementation est un copier-conforme de la branche `hi8 == 0xF3`
             * (mask FCF0), volontairement : meme arithmetique, meme traitement du
             * shift et du signe, meme selection src/dst. Toute divergence entre les
             * deux serait un bug de plus, pas une amelioration. */
            /* [2026-08-03] CONDITION EFFACEE — le correctif est CONFIRME et devient
             * le comportement normal. Protocole `environnement/fixes.env` : « des
             * qu'un correctif est confirme, EFFACER LA CONDITION dans le code, pas
             * le correctif. Un sas se vide, une bequille reste. »
             *
             * PREUVE RETENUE (A/B en `native_twl_host_demod`, 03/08) :
             *   effet positif : `0xa5cd` (armement RX) s'execute pour la premiere
             *     fois — la chaine de dispatch franchit 0xb066, atteint 0xb070
             *     (resolution d'index) puis le CALA ;
             *   effet negatif : AUCUN observable perdu. Camp, SI et CHANNEL REQUEST
             *     tous presents et au moins aussi nombreux qu'avant (SI 8 vs 7,
             *     camp 46 vs 30/42, CHAN_REQ 15 vs 10/14).
             *
             * ⚠️ LIMITE ASSUMEE, a connaitre : le niveau 3 complet de fixes.env
             * (camp -> LU -> SMS) est STRUCTURELLEMENT inexercable sur ce correctif.
             * Le seul banc qui va jusqu'au SMS est `shunt_legit`, et il pose
             * `CALYPSO_DSP_RUN_C54X=0` (calypso_shunt_legit.env:29) — l'interpreteur
             * c54x n'y tourne pas, donc ce correctif n'y est jamais execute. Ce
             * n'est pas un manque de rigueur : aucun banc ne reunit aujourd'hui
             * « c54x actif » et « LU complet ». Le LU n'aboutit pas non plus en
             * native_twl_host_demod (mesure : 0 LU ACCEPT, 0 TMSI, pas d'IMM
             * ASSIGN) — ce qui marchait, et qui marche toujours, c'est camp + SI.
             * Reevaluer quand le chemin DSP ira plus loin (apres le DMA). */
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
                int shift     = (shift_raw & 0x8) ? (shift_raw - 16) : shift_raw;
                /* ─────────────────────────────────────────────────────────────
                 * [2026-08-24] SAS FIX_LK_SHFT (CALYPSO_FIX_LK_SHFT, defaut OFF).
                 *
                 * LE DEFAUT. Le champ de decalage de ces formes #lk tient sur
                 * QUATRE bits (`op & 0xF`), mais on lui applique la regle de signe
                 * d un champ de CINQ bits (-16..15). C est incoherent en soi : on
                 * ne represente pas -16..15 sur 4 bits. Consequence mesuree en
                 * 0x7d19 (`f02f 0001` = LD #1, SHFT=15, A) : shift devient -1,
                 * lk_val = 1 >> 1 = 0, et l accumulateur reste NUL.
                 *
                 * AUTORITE FORMELLE. doc/opcodes/tic54x-opc.c donne pour ces
                 * formes l operande OP_SHFT (non signe) et non OP_SHIFT :
                 *   { "ld",  0xF020, 0xFEF0, {OP_lk, OPT|OP_SHFT,  OP_DST} }
                 *   { "sub"/"and"/"or"/"xor" ... OPT|OP_SHFT ... }
                 *   { "add", 0xF000, 0xFCF0, {OP_lk, OPT|OP_SHIFT, ...} }  <- seul
                 *
                 * ARGUMENT PHYSIQUE. La sequence est l idiome C54x du reciproque :
                 *     0x7d19  ld   #0x0001, A
                 *     0x7d1b  sfta A
                 *     0x7d1c  rpt  #15
                 *     0x7d1d  subc @0x0b, A     ; division en 16 pas
                 * SUBC exige un dividende cadre a gauche, sinon le quotient est nul
                 * PAR CONSTRUCTION. Charger 1 pour le decaler a DROITE rendrait tout
                 * le bloc mort. A gauche de 15 : 0x8000 / 0x0481 = 28, un reciproque
                 * en Q15 -- ce que le firmware attend.
                 *
                 * PORTEE VOLONTAIREMENT ETROITE. On ne touche PAS le sous-code 0
                 * (ADD), seul que la table marque OP_SHIFT. Corriger les deux d un
                 * coup rendrait la mesure ininterpretable (protocole fixes.env).
                 *
                 * A EFFACER (la condition, pas le correctif) quand valide sous
                 * charge : le sas se vide, la bequille reste. */
                {
                    static int fix = -1;
                    if (fix < 0) {
                        const char *e = getenv("CALYPSO_FIX_LK_SHFT");
                        fix = (e && *e && atoi(e)) ? 1 : 0;
                    }
                    if (fix && subop >= 1 && subop <= 5 && (shift_raw & 0x8)) {
                        /* COMPTEUR DE PORTEE. Le niveau 3 de fixes.env n a de sens
                         * que si le correctif s EXECUTE sur le banc teste : un sas
                         * jamais atteint donnerait un A/B identique et un faux
                         * "aucun effet negatif". On compte donc les fois ou il
                         * CHANGE effectivement le resultat (shift_raw >= 8, seul
                         * cas ou signe et non signe divergent) et on l annonce
                         * periodiquement. Sans cette ligne, l A/B n est pas lisible. */
                        static unsigned long chg = 0;
                        if (++chg == 1 || (chg % 20000) == 0)
                            fprintf(stderr, "[c54x] FIX_LK_SHFT ACTIF #%lu "
                                    "pc=0x%04x op=0x%04x subop=%d shift %d -> %d insn=%u\n",
                                    chg, s->pc, op, subop,
                                    shift_raw - 16, shift_raw, s->insn_count);
                        shift = shift_raw;          /* OP_SHFT : non signe, 0..15 */
                    }
                }
                int src_b     = (op >> 9) & 1;
                int dst_b     = (op >> 8) & 1;
                int64_t src   = src_b ? s->b : s->a;
                /* ADD/SUB/LD : lk signe ; AND/OR/XOR : lk non signe. */
                int64_t lk_base = (subop <= 2) ? (int64_t)(int16_t)op2
                                               : (int64_t)(uint16_t)op2;
                int64_t lk_val  = (shift >= 0) ? (lk_base << shift)
                                               : (lk_base >> (-shift));
                int64_t result = src;
                switch (subop) {
                case 0x0: result = src + lk_val; break;   /* ADD */
                case 0x1: result = src - lk_val; break;   /* SUB */
                case 0x2: result = lk_val;       break;   /* LD (src ignore) */
                case 0x3: result = src & lk_val; break;   /* AND */
                case 0x4: result = src | lk_val; break;   /* OR  */
                case 0x5: result = src ^ lk_val; break;   /* XOR */
                }
                if (dst_b) s->b = sext40(result); else s->a = sext40(result);
                {   /* Trace bornee : un correctif d'ISA doit pouvoir se constater,
                     * pas se supposer. 40 lignes, sous le gate du sas. */
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
            /* FIRS catch RETIRÉ (2026-05-25 v3, Claude web review).
             *
             * Le bloc `if (hi8 == 0xF1) { FIRS treatment }` qui était ici
             * était FAUX : per binutils tic54x-opc.c, vrai FIRS = 0xE000
             * mask 0xFF00 (handled separately at the 0xE0 case), JAMAIS
             * 0xF1xx. Le catch-all capture-tout 0xF1xx faisait :
             *   s->a = sext40((int64_t)sum << 16);
             * → A_low = 0 inconditionnellement → STL A,*AR2- à PC=0x9ac0
             * écrivait 0 à mem[AR2] qui se trouvait être MMR_IMR (0x12 via
             * self-aliasing) → IMR cleared → DSP bloqué (bloqueur #2).
             *
             * Diagnostic via A provenance tracer (CALYPSO_A_TRACE_PC=0x9ac0)
             * a montré last_writer = PC=0x9abd op=0xf1fe = `SFTL A,-2,B`
             * (binutils mask 0xFCE0 base 0xF0E0). SFTL handler EXISTE à
             * L3915, capture correctement 0xF1FE & 0xFCE0 = 0xF0E0.
             *
             * Après retrait du catch, les 0xF1xx tombent dans :
             *   - SFTL/AND/OR/XOR 1-word (mask FCE0)  : L3915
             *   - AND/OR/XOR/MAC #lk<<16 (mask FCFF) : L3852
             *   - AND/OR/XOR #lk+shift  (mask FCF0)  : L3886
             * Si une opcode 0xF1xx n'a pas de handler (par ex. add/sub lk
             * variants avec DST=B), tombe à F4xx unhandled NOP log à la fin
             * du bloc → diagnostic visible. */
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
            /* [2026-08-23] RPTZ : le test etait une EGALITE EXACTE alors que
             * binutils donne le masque 0xFEFF (bit 8 = accumulateur) :
             *     { "rptz", 2, ..., 0xF071, 0xFEFF, {OP_DST, OP_lku} }
             * 0xF071 = RPTZ A, 0xF171 = RPTZ B. La variante B tombait donc sur
             * `unimpl:` — SEULE entree binutris du jeu reellement non decodee,
             * 8 sites dans PROM0, tous alignes. Triple degat : B n etait pas
             * remis a zero, RC n etait pas arme (la boucle suivante tournait UNE
             * fois au lieu de lk+1), et 1 mot etait consomme au lieu de 2, donc
             * le mot #lk s executait comme instruction -> DESYNCHRONISATION.
             * Le corps lisait deja `dst = (op >> 8) & 1` : seul le test etait faux.
             * Gate CALYPSO_ISA_RPTZ (defaut 1). */
            {
                static int _rz = -1;
                if (_rz < 0) {
                    _rz = calypso_gate("CALYPSO_ISA_RPTZ", 1);
                    fprintf(stderr, "[c54x] ISA-RPTZ %s : 0xF071/0xFEFF (RPTZ A et B) "
                            "au lieu de l egalite exacte 0xF071\n",
                            _rz ? "ACTIF" : "INACTIF (RPTZ B non decode)");
                }
                if (_rz && (op & 0xFEFF) == 0xF071 && op != 0xF071) {
                    op2 = prog_fetch(s, s->pc + 1);
                    consumed = 2;
                    if ((op >> 8) & 1) s->b = 0; else s->a = 0;
                    s->rpt_count = op2;
                    s->rpt_active = true; s->rpt_fresh = true;
                    s->pc += 2;
                    return 0;
                }
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
                     * bits 4:0 = shift (signed 5-bit), bits 9:8 = src,dst
                     *
                     * Fix 2026-05-25 v4 : src/dst inversés. Per binutils
                     * tic54x convention (et confirmé par L3915 même opcode
                     * famille mais handler shadowed) : bit 9 = SRC,
                     * bit 8 = DST. L'inversion mettait dst=A pour 0xf1fe
                     * (= SFTL A,-2,B), qui calculait A = B>>2 au lieu de
                     * B = A>>2. Si B=0 → A=0 → STL A,*AR2- pose 0 à IMR
                     * (bloqueur #2 racine, cf [[blocker-2-dsp-dispatcher]]).
                     * Diagnostic via A-AT-PC tracer : 8454 fires A_low=0
                     * last_writer=0xf1fe @PC=0x9abd. */
                    int src_sel = (op >> 9) & 1;   /* bit 9 = SRC (was bit 8) */
                    int dst_sel = (op >> 8) & 1;   /* bit 8 = DST (was bit 9) */
                    int64_t sv = src_sel ? s->b : s->a;
                    int64_t *dst = dst_sel ? &s->b : &s->a;
                    int shift = op & 0x1F;
                    if (shift > 15) shift -= 32;
                    uint8_t aop = (op >> 5) & 0x7;
                    int64_t shifted;
                    if (shift >= 0) shifted = sv << shift;
                    else            shifted = sv >> (-shift);
                    /* Gate d'echappement, defaut 1 : `CALYPSO_FIX_ALU3_DST=0`
                     * restaure a l'identique le comportement d'avant le 04/08
                     * (premier operande = src), pour isoler une regression.
                     * ⚠️ Resolu ICI, AVANT le switch : place entre `switch` et
                     * le premier `case`, l'initialisation ne s'executerait
                     * jamais (code inatteignable). */
                    static int _alu3 = -1;
                    if (_alu3 < 0) {
                        _alu3 = calypso_gate("CALYPSO_FIX_ALU3_DST", 1);
                        fprintf(stderr, "[c54x] FIX_ALU3_DST %s "
                                "(CALYPSO_FIX_ALU3_DST=%d) — AND/OR/XOR %s "
                                "la destination (TI SPRU172C)\n",
                                _alu3 ? "ACTIF" : "inactif", _alu3,
                                _alu3 ? "LISENT" : "NE LISENT PAS");
                    }
                    int64_t first = _alu3 ? *dst : sv;
                    switch (aop) {
                    /* ═══════════════════════════════════════════════════════
                     * [2026-08-04] FIX — AND/OR/XOR LISENT LA DESTINATION.
                     *
                     * Le code faisait `*dst = sv OP shifted`, soit
                     * `dst = src OP (src<<SHIFT)` : la destination etait JETEE.
                     * Correct seulement quand D == S, faux des que D != S.
                     *
                     * TI SPRU172C (Mnemonic Instruction Set, mars 2001),
                     * table recapitulative et page instruction :
                     *     AND src [,SHIFT] [,dst]   dst = dst & src << SHIFT
                     *     OR  src [,SHIFT] [,dst]   dst = dst | src << SHIFT
                     *     XOR src [,SHIFT] [,dst]   dst = dst ^ src << SHIFT
                     *     Execution : (src or [dst]) OP (src) << SHIFT -> dst
                     * Encodage forme 4 (1 mot), verifie bit a bit :
                     *     15..10 = 111100   bit9 = S   bit8 = D
                     *     7..6 = 10  bit5 = 1  4..0 = SHIFT
                     *
                     * CAS MESURE : `0xf1a5` = 111100 0 1 10 1 00101
                     *   -> src=A, dst=B, SHIFT=5, soit `or A, 5, B`.
                     * En 0x9719, B portait `0x8000` (= 1<<B_BLUD, « data block
                     * Present », pose en 0x96dd). L'ancien calcul le detruisait
                     * -> `stl *AR3,B` ecrivait 0x0000 dans a_cd[0] -> l'ARM ne
                     * voyait jamais de bloc. Portee reelle : TOUT
                     * read-modify-write du firmware, bien au-dela d'a_cd.
                     *
                     * ⚠️ `case 7` (SFTL) reste inchange : la doc donne
                     *     SFTL src, SHIFT [,dst]  ->  dst = src << SHIFT
                     * — SFTL ne lit PAS la destination, c'est un decalage.
                     *
                     * ⚠️ Distinct de FIX_F1XX_ALU_LK (03/08), qui ne couvre que
                     * le masque 0xFCF0 sous-op 0..5 = les formes 2 mots a
                     * immediat long. Ici la sous-op est 0xA, forme 1 mot.
                     * ═══════════════════════════════════════════════════════ */
                    case 4: *dst = sext40(first) & sext40(shifted); break;
                    case 5: *dst = sext40(first) | sext40(shifted); break;
                    case 6: *dst = sext40(first) ^ sext40(shifted); break;
                    case 7: { uint64_t uv = (uint64_t)(sv & 0xFFFFFFFFFFULL);
                              if (shift >= 0) uv <<= shift; else uv >>= (-shift);
                              *dst = sext40(uv & 0xFFFFFFFFFFULL); } break;
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
             * Push PC+4 (return = past CALLD + 2 delay slots), PUIS exécute les
             * 2 delay-slots via delayed_pc/delay_slots AVANT de brancher.
             * Fix 2026-05-30 : était saut immédiat (s->pc=op2; return 0) → les
             * 2 delay-slots étaient SKIPPÉS ; si un slot contient un push/pop,
             * la pile se désaligne d'1 mot → POPM ST0 ramasse un PC orphelin
             * (ex. 0x80fd) → DP garbage → CALA 0x70c3 = trou noir → TOA figé →
             * AFC bloqué. Arme la machinerie delay_slots (comme RCD/RETED). */
            op2 = prog_fetch(s, s->pc + 1);
            consumed = 2;
            s->sp--;
            data_write(s, s->sp, (uint16_t)(s->pc + 4));
            s->delayed_pc  = op2;
            s->delay_slots = 2;
            return consumed + s->lk_used;
        }
        if (op == 0xF273) {
            /* BD pmad — delayed branch (2 words, 2 delay slots). AUCUNE pile.
             * Per tic54x-opc.c: bd 0xF273 mask 0xFFFF. Le vrai RETD = 0xFE00
             * (hi8==0xFE, géré plus bas avec pop + delay_slots).
             * Fix 2026-05-30 : était traité comme RETD (pop parasite) → SP
             * désaligné d'1 mot par BD → POPM ST0 0xf48b pop un PC orphelin
             * → DP=0x087 → dispatcher 0x8341 → CALAD 0x70c3 = trou noir.
             * Identique au B (F073) ci-dessus : saut, pas de pile.
             * Fix 2026-05-30 v2 : était saut IMMÉDIAT (skip des 2 delay-slots) →
             * si un slot a un push/pop, pile désalignée → POPM ST0 orphelin →
             * 0x70c3. Arme delay_slots=2 pour exécuter les slots avant le saut. */
            op2 = prog_fetch(s, s->pc + 1);
            consumed = 2;
            s->delayed_pc  = op2;
            s->delay_slots = 2;
            return consumed + s->lk_used;
        }
        /* === F2xx dispatch (audit F-class 2026-05-25) =====================
         *
         * Per binutils tic54x-opc.c, les ALU masks FCF0/FCFF/FCE0 couvrent
         * F0xx/F1xx/F2xx/F3xx avec bit 9=SRC bit 8=DST (= convention
         * binutils stricte). F2xx était le seul gap :
         *   - F0/F1 → handler legacy L3565 (convention REVERSED bit 8=src,
         *     gardée pour back-compat ; firmware s'y est calé)
         *   - F3 → handler dispatch L3966 (binutils convention OK)
         *   - F2 → fallthrough vers unimpl → 0xf210 tight loop at PC=0xfbd9
         *
         * Bug runtime résolu : op=0xf210 op2=0x0008 → `SUB #8,B,A` (next BC
         * fbe2, ALEQ → wait loop pre-correlator). Confirmed across 3 silicon
         * ROM dumps (3416, 3606, our local) — cf doc/datasheets/.
         *
         * Coverage (binutils strict) :
         *   - F260-F267 mask FCFF : ALU #lk,16 + MAC
         *   - F200/F210/F220/F230/F240/F250 mask FCF0 : ADD/SUB/LD/AND/OR/XOR #lk,shift
         *   - F280-F2FF mask FCE0 : 1-word AND/OR/XOR/SFTL src,shift,dst
         *
         * F272/F273/F274 (exact-match RPTBD/BD/CALLD) restent gérés AVANT
         * (handlers ci-dessus), inchangés. */
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
                (op & 0xFCF0) == 0xF010 ||  /* SUB  ← 0xF210 hit ici */
                (op & 0xFCF0) == 0xF020 ||  /* LD   */
                (op & 0xFCF0) == 0xF030 ||  /* AND  */
                (op & 0xFCF0) == 0xF040 ||  /* OR   */
                (op & 0xFCF0) == 0xF050) {  /* XOR  */
                op2 = prog_fetch(s, s->pc + 1 + (s->lk_used ? 1 : 0));
                consumed = 2;
                int subop = (op >> 4) & 0xF;
                int shift_raw = op & 0xF;
                int shift = (shift_raw & 0x8) ? (shift_raw - 16) : shift_raw;
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
                case 0x7: { uint64_t usrc = (uint64_t)src & 0xFFFFFFFFFFULL;
                            result = (int64_t)((shift >= 0) ? (usrc << shift)
                                                            : (usrc >> (-shift)));
                            break; }
                }
                if (dst_b) s->b = sext40(result); else s->a = sext40(result);
                return consumed + s->lk_used;
            }
            /* F2xx unmapped — log-once + NOP fallback. Si tu vois ce log,
             * c'est qu'un bit pattern F2xx n'est pas couvert (audit incomplete). */
            { static int f2_unm = 0;
              if (f2_unm++ < 20)
                  C54_LOG("F2xx unmapped op=0x%04x PC=0x%04x (NOP)", op, s->pc); }
            return consumed + s->lk_used;
        }
        /* LMS Xmem, Ymem — Least Mean Square step (1-word dual-operand)
         * Encoding: 1111 001D XXXX YYYY
         * Per SPRU172C: dst += T * Xmem; Ymem += rnd(AH * T); T = Xmem
         * Exclude F272 (RPTBD), F273 (RETD), F274 (CALLD) — exact-match
         * opcodes that share the F2xx range but are handled below. */
        /* REMOVED 2026-05-08 night : the previous "LMS Xmem,Ymem" handler
         * for hi8 ∈ {0xF2, 0xF3} (excluding F272/F273/F274) was mis-decoded
         * — it claimed encoding `1111 001D XXXX YYYY` but per binutils
         * tic54x-opc.c LMS is actually :
         *
         *   { "lms", 1,2,2, 0xE100, 0xFF00, {OP_Xmem,OP_Ymem}, ... }
         *
         * i.e. hi8 == 0xE1, NOT 0xF2/F3. The 0xE1 handler already exists
         * (line ~3247) and is correct.
         *
         * The F2xx/F3xx range per binutils contains only :
         *   F272 RPTBD, F273 RETD, F274 CALLD                (3 special-cases)
         *   F300-F31F INTR k                                 (handled below)
         *   F330-F35F AND/OR/XOR with shift  (mask FCF0)     (handled below)
         *   F360-F367 ADD/SUB/AND/OR/XOR/MAC #lk (mask FCFF) (handled below)
         *   F380-F3FF AND/OR/XOR/SFTL src,SHIFT,DST (FCE0)   (handled below)
         *   F320-F32F + F368-F37F unmapped (NOP fallback)
         *
         * The bogus LMS catch-all stole every F3xx instruction before the
         * proper F3 dispatch could see it. For 0xF3E1 (= SFTL B,1,B,
         * 4872 sites in firmware) it computed `new_ym = AH*T-derived junk`
         * and called data_write(s, AR1, new_ym). When AR1=0, that wrote
         * the junk to MMR_IMR. This is the IMR-thrash cascade observed
         * post-0x76-fix at PC=0x8eb9.
         *
         * Discovered after the 0x76 fix exposed the second-level cascade.
         * Trace evidence : IMR-W 0x0000→{0x0540, 0x0525, 0x082b, 0xfd57,
         * 0xfacf, ...} all PC=0x8eb9 op=0xf3e1, INTM-TRANS XPC=0
         * (confirms genuine PROM0 execution, not XPC artifact).
         *
         * Fix : let the existing F3 dispatch (line 2468+) handle F3xx
         * properly. F2xx (other than F272/3/4) falls through to F-class
         * NOP fallback — firmware does not appear to use it. */
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
             * REVERTED 2026-05-15 nuit : tentative de fix vers SPRU172C-strict
             * cond eval (cond=0x20=NTC, cond=0x30=TC) a cassé le firmware DSP
             * Calypso (DSP stuck à PC=0xcc51 / 0xfa95 selon régime, task=24
             * tombait à 0). Le binaire DSP semble utiliser une convention
             * dialectale où F82x/F83x s'attend au comportement ACC-based.
             * Hypothèse alternative : BITF (0x61) émulé incorrectement, TC
             * jamais set correctement → cond NTC/TC ne donne pas le bon
             * résultat. Investiguer BITF avant de retenter le fix BC strict. */
            if (sub == 0x2 || sub == 0x3) {
                op2 = prog_fetch(s, s->pc + 1);
                consumed = 2;
                int64_t acc_signed = (s->a & 0x8000000000LL)
                                     ? (s->a | ~0xFFFFFFFFFFLL) : s->a;
                bool take = false;
                /* [2026-07-25] §4-G FIX (gated CALYPSO_C54X_FIX_BC, def OFF) : decode
                 * BC 0xF8 par le VRAI champ cond 8-bit (c54x_cond_true, ISA-fidele)
                 * au lieu de l heuristique ACC dialectale. F820=cc0x20(NTC),
                 * F830=cc0x30(TC). Corrige le flux du handler FB (branches 0x9062-
                 * 0x90f0) qui n atteint jamais le kernel MAC 0xa076. Gate OFF car le
                 * fix strict a casse 2x avant (TC/BITF) -> valider via chaine de tests. */
                {
                    static int fixbc = -1;
                    if (fixbc < 0) fixbc = calypso_gate("CALYPSO_C54X_FIX_BC", 0);
                    if (fixbc) {
                        if (c54x_cond_true(s, op & 0xFF)) { s->pc = op2; return 0; }
                        return consumed + s->lk_used;
                    }
                }
                /* FIX 2026-06-23 (deblocage bacc bootloader) : F820=bc ntc /
                 * F830=bc tc (SPRU172C cc=0x20 NTC, 0x30 TC). L'ancienne
                 * heuristique ACC (take=A!=0) gelait le poll @0xb427 (bc ntc
                 * apres cmpm#2 : doit sortir sur TC=1=data==2 -> bacc 0x7000).
                 * cmpm ET bitf posent TC correctement ; on l'utilise SEULEMENT
                 * quand l'instruction precedente est un poseur-de-TC
                 * (cmpm/bitf = 0x60xx/0x61xx, mask 0xFE00) - sinon heuristique
                 * ACC heritee pour les sites dispatcher (blanket TC-strict avait
                 * casse le 2026-05-15, avant le fix cmpm). */
                /* [2026-07-02] go-live DSP : la boucle compute du state-machine
                 * handshake (0xde0d-0xde26) se termine par F830 de0d = BC TC.
                 * Rien dans la boucle ne pose TC (6d91=MAR, f5a9=RPT-fallback),
                 * donc sur vrai C54x TC=0 -> BC TC NON pris -> la boucle tourne
                 * UNE fois et tombe en 0xde28 vers le setter 0xde9c. L heuristique
                 * ACC heritee (take si A==0, et A=0 ici via f0e1) la piege en
                 * boucle infinie. Gate PC-range + env : semantique BC TC reelle
                 * uniquement sur 0xde0d-0xde26. Defaut OFF -> aucun risque ailleurs. */
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
             * Earlier this range was treated as plain B pmad — a bug that
             * kept XPC=0 forever (DSP never reached PROM1 user code). */
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
            /* F84x/F85x: BC pmad, ACC-condition (2 mots). FIX 2026-06-23 (ROOT du
             * derail SP) : ces opcodes sont des BC, PAS des BANZ (BANZ = 0x6Cxx, un
             * encodage DISJOINT — SPRU172C:16188/16266). cc = octet bas : cc&0x40 =
             * groupe ACC, cc&0x08 = B vs A, cc&0x07 = test {2:GEQ 3:LT 4:NEQ 5:EQ
             * 6:GT 7:LEQ}. Le firmware @0x772f émet F844 = BC 0x7737,ANEQ (branche si
             * A!=0). L'ANCIEN décode `BANZ AR4` testait/décrémentait AR4 au lieu de
             * l'accumulateur → la branche tirait sur AR4!=0 → atterrissait sur le
             * POPM ST0 @0x7737 SANS son PSHM ST0 (@0x770d, chemin disjoint) → pop
             * orphelin → SP monte +1/tour → DP=0x124 → derail → SP collapse → spin.
             * BC ne push RIEN (pile intacte) ; SURGICAL : 0x4/0x5 seul, 0x2/0x3
             * (dialecte, fix BC strict reverté 2026-05-15) inchangés ; disjoint du
             * catch-all 0x7000 (STM #lk,SP). */
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
                default:  take = false;       break;   /* 0/1 réservé */
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
         * Dispatch order: most-specific masks first (FCFF → FCF0 → FCE0).
         *
         * 2026-04-29 — replaces previous "F320+ → LD #k9, DP" fallback
         * which mass-mis-decoded 364 firmware sites. Wedge at PC=0x8eb9
         * (0xF3E1 SFTL B,1,B) was directly tied to this bug.
         * See doc/opcodes/0xF3.md for full spec. */
        if (hi8 == 0xF3) {
            /* === F300-F31F INTR k REMOVED (2026-05-25 night, audit F-class)
             *
             * AUDIT-FINDING : the "INTR k" handler placed at 0xF300-0xF31F
             * was WRONG. Per binutils tic54x-opc.c L311 :
             *   { "intr", 1,1,1, 0xF7C0, 0xFFE0, ... }  ← REAL INTR k
             * INTR k base is 0xF7C0, NOT 0xF300. The F3xx range belongs to
             * ALU #lk class (per mask 0xFCF0).
             *
             * Symptom captured runtime (CALYPSO_AR_TRACE=0x08) :
             *   PC=0xe9a2 op=0x8913 STLM B,AR3 fires 10243× with B=0 → AR3=0
             *   The preceding PC=0xe9a0 op=0xf310 was MEANT to be `SUB #5,B,B`
             *   (= B -= 5) but our wrong INTR handler pushed PC+1 and jumped
             *   to vec table → SUB never executed → B stayed 0 → AR3=0 →
             *   BANZ fc54,*AR3- loop infinite at fc50-fc6d → INT3 ISR never
             *   RETE → INTM=1 forever → IRQ subséquentes pending only.
             *
             * Fix : retirer ce handler ; F310 etc. tombent dans la FCF0
             * dispatch ci-dessous (ADD/SUB/LD/AND/OR/XOR pour F3xx).
             *
             * A real INTR k handler should be added at F7Cx if firmware
             * uses it — TODO. Pas urgent (zero F7Cx hits observed in run). */

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
             * 2026-05-25 night : ADD/SUB/LD ADDED here (étaient mis-décodés
             * par le faux INTR k F300 retiré ci-dessus). Fix smoking-gun
             * 0xf310 = SUB #lk,B,B au PC=0xe9a0 → B=0 → AR3=0 → loop fc50. */
            if ((op & 0xFCF0) == 0xF000 ||  /* ADD #lk, SHIFT, src, [dst] */
                (op & 0xFCF0) == 0xF010 ||  /* SUB ← FIX 0xf310 */
                (op & 0xFCF0) == 0xF020 ||  /* LD  (binutils mask FEF0, no src) */
                (op & 0xFCF0) == 0xF030 ||  /* AND */
                (op & 0xFCF0) == 0xF040 ||  /* OR */
                (op & 0xFCF0) == 0xF050) {  /* XOR */
                op2 = prog_fetch(s, s->pc + 1 + (s->lk_used ? 1 : 0));
                consumed = 2;
                int subop = (op >> 4) & 0xF;
                int shift_raw = op & 0xF;
                int shift = (shift_raw & 0x8) ? (shift_raw - 16) : shift_raw;
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
                case 0x7: { /* SFTL src,SHIFT,DST: DST = SRC << shift (logical) */
                    uint64_t usrc = (uint64_t)src & 0xFFFFFFFFFFULL;
                    result = (int64_t)((shift >= 0) ? (usrc << shift) : (usrc >> (-shift)));
                    break;
                }
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
                rsbx_intm_check(s, op);  /* probe candidat 1 doc §7 */
                s->st1 &= ~(1 << bit);
                return consumed + s->lk_used;
            }
            /* Delayed branches/calls/returns from PROM (per tic54x-opc.c).
             * MUST be checked BEFORE the MVDD catch-all because they share
             * the high nibbles 0xE/0x9. Without these the DSP cannot return
             * from interrupt service routines — RETED in particular leaves
             * INTM=1 forever, blocking every subsequent INT3 and stalling
             * the firmware↔DSP frame loop (the original CLAUDE.md root bug).
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
                /* CALAD-AT-8353-PROBE (c web review 2026-05-27) : at the
                 * exact site we know self-loops, dump XPC + full A + delay
                 * slot state. CALAD per SPRU172C preserves XPC ; the probe
                 * confirms XPC value at entry (1 → firmware was on far page,
                 * 0 → firmware threw far-pointer at near call). First hit only. */
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
                 * Pop XPC + PC unconditionally (FL_FAR). FRETED also clears INTM.
                 * 2026-04-28 — fixed: was APTS-gated (= AVIS, no stack semantics). */
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
                 * A(22:16) → XPC, A(15:0) → tgt. XPC update is immediate
                 * (mirrors FRETED at line ~1639). FCALAD pushes ret PC+3,
                 * and (when APTS) pushes XPC first (so RETF/FRETD pops in
                 * order). 2 delay slots. */
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
                    /* FCALAD (F6E7): push XPC + return PC unconditionally (FL_FAR).
                     * 2026-04-28 — fixed: was APTS-gated (= AVIS, no stack semantics). */
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
            /* [2026-09-17] FIX_F7_DELAYED — F7E2/F7E3 = BACCD B / CALAD B, F7E6/F7E7 =
             * FBACCD B / FCALAD B (tic54x-opc.c:267-303 : baccd 0xF6E2 masque 0xFEFF,
             * bit 8 = accumulateur). Le bloc ci-dessous les rangeait dans « reserve ->
             * NOP » : un CALAD B saute par-dessus l'appel. PROM0..3 : 51 x f7e3.
             * Meme semantique que les variantes A du bloc F6 (retour = PC+3, 2 slots
             * de delai). Gate CALYPSO_FIX_F7_DELAYED (defaut ON). */
            {
                static int fix_f7d = -1;
                if (fix_f7d < 0) fix_f7d = calypso_gate("CALYPSO_FIX_F7_DELAYED", 1);
                if (fix_f7d && (op == 0xF7E2 || op == 0xF7E3)) {
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
                if (fix_f7d && (op == 0xF7E6 || op == 0xF7E7)) {
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
                /* PROBE 2026-06-01 : 2e site d'écriture BK (LD #k8,BK). Nomme le
                 * writer + valeur. BK=0 casse l'adressage circulaire → runaway
                 * AR2 0xfa98/0xf17c. À RETIRER avec la sonde MMR_BK. */
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
            /* FCALL FAR : push XPC + return PC unconditionally (FL_FAR).
             * Per binutils tic54x-opc.c (fcall 0xF980 mask 0xFF80, FL_FAR)
             * and SPRU172C: FAR call always saves XPC for FRET to restore.
             * 2026-04-28 — fixed: was APTS-gated (= AVIS, no stack semantics).
             * Old behavior caused 281 firmware FCALL FAR sites to push only PC,
             * imbalanced with 142 FRET pop expecting both PC + XPC. */
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
            /* FIX 2026-05-31 : cond décodée depuis l'octet bas (binutils
             * condition_codes[]) via c54x_cond_true(). L'ancien (op>>4)&0xF
             * lisait le mauvais champ → CC[TC/NEQ/LT/...] faux → push manquants
             * power-scan 0xb1xx → over-pop → 0x80fd → self-CALA 0x70c3. */
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
            /* Fix 2026-07-03 : etend a FA20/FA30 (BCD pmad,NTC/TC, delayed
             * sibling de F820/F830) le fix deja valide le 2026-06-23 pour BC.
             * REVERT 2026-05-15 : evaluer la vraie cond TC/NTC EN BLOC pour
             * tout FA00-FA7F cassait le firmware (DSP stuck loops) -- la
             * plupart des callers FAxx ne posent pas TC de facon fiable
             * avant de brancher. On mirror EXACTEMENT la technique BC :
             * n evaluer la vraie cond QUE quand l opcode precedent est
             * CMPM/BITF (0x60xx/0x61xx, pose TC de facon fiable) ; sinon,
             * comportement inchange (branch always, deja valide sur). Zero
             * risque de regression hors de ce cas etroit et sur. Delayed :
             * 2 delay slots (meme mecanisme que FBD FAR juste au-dessus). */
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
            /* [2026-08-22] FIX — les conditions ACCUMULATEUR sont evaluees
             * pour de vrai, et le DELAI est honore.
             *
             * MESURE (sonde TOA-TRACE, run natif propre) :
             *     @0x7918 BCD 0x7923 si B!=0 ... B=0
             *     @0x7923 *** BRANCHE PRISE ***
             * `BNEQ` avec B=0 branchait quand meme, systematiquement : le
             * fallback ci-dessous traitait TOUT FAxx non-TC comme un
             * branchement INCONDITIONNEL. Consequence : la route FB0
             * (0x791c..0x7921), qui calcule TOA = (d[0x3fb4]-3)*48 + d[0x0c3d],
             * n'etait JAMAIS empruntee — le TOA venait de la route FB1 avec
             * cpt1[0x3fb3]=296 fige, donc hors de la plage 0..1249 attendue
             * par osmocom (prim_fbsb.c, BITS_PER_TDMA=1250).
             *
             * De plus BCD est RETARDE : les 2 mots suivants s'executent AVANT
             * le saut. Le fallback faisait `s->pc = op2` immediatement, donc le
             * slot 0x791a `stm #0x0030` (T = 48, le pas du TOA) etait saute.
             *
             * PORTEE : on n'elargit qu'aux conditions ACCUMULATEUR (cc & 0x40),
             * qui testent A ou B et sont fiables. TC/carry gardent EXACTEMENT
             * l'ancien comportement — ce sont elles que le commentaire du
             * 2026-05-15 accusait d'avoir casse le firmware. Les deux ensembles
             * sont disjoints (fa_sub 2/3 implique cc & 0x40 == 0).
             * Bascule A/B : CALYPSO_ISA_BCD_COND=0 restaure l'inconditionnel. */
            {
                static int _bcd = -1;
                if (_bcd < 0) {
                    _bcd = calypso_gate("CALYPSO_ISA_BCD_COND", 1);
                    fprintf(stderr, "[c54x] ISA-BCD-COND %s : BCD pmad,cond "
                            "(conditions accumulateur evaluees + 2 delay slots)\n",
                            _bcd ? "ACTIF (fidele SPRU172C)"
                                 : "INACTIF (branchement inconditionnel)");
                }
                uint8_t _cc = (uint8_t)(op & 0x7F);
                if (_bcd && (_cc & 0x40)) {
                    if (c54x_cond_true(s, _cc)) {
                        s->delayed_pc  = op2;
                        s->delay_slots = 2;
                    }
                    return consumed + s->lk_used;   /* non prise : on continue */
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
            /* FCALLD FAR : push XPC + return PC+4 unconditionally (FL_FAR delayed).
             * Per binutils (fcalld 0xFB80 mask 0xFF80, FL_FAR|FL_DELAY).
             * 2026-04-28 — fixed: was APTS-gated (= AVIS, no stack semantics). */
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
            /* FIX 2026-05-31 : cond décodée depuis l'octet bas via
             * c54x_cond_true() (cf CC ci-dessus). */
            bool take = c54x_cond_true(s, (uint8_t)(op & 0x7F));
            if (take) {
                /* FIX 2026-05-31 : CCD est DIFFÉRÉ — arme delay_slots=2 +
                 * delayed_pc (comme CALLD f274, fixé 2026-05-30), au lieu de
                 * sauter immédiatement (s->pc=op2; return 0) qui SKIPPAIT les
                 * 2 delay-slots → push perdu si un slot pousse → over-pop →
                 * 0x80fd → self-CALA 0x70c3. Retour poussé = pc+4 (past CCD +
                 * 2 slots). Not-taken : PC avance de consumed (2) = past CCD. */
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
                /* POST-BOOTSTUB-RET : si on est en train de RET depuis le
                 * boot stub (PC ∈ 0x0000..0x0008), c'est la sortie du
                 * task-switch trampoline 0x701b/0x701d → 0x0000. Le ra
                 * poppé est le PC du task qui prend le contrôle. À insn≈90.2M
                 * (dernière transition INTM), ce PC = le task qui ne clear
                 * jamais INTM ensuite. */
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
                    /* DEEP-TRAIL : pour les 5 premiers POST-BOOTSTUB-RET,
                     * dump pc_ring[-64..-1] pour révéler le caller chain
                     * qui mène au stack-underflow loop. Gated par
                     * CALYPSO_DEBUG=BOOTSTUB_TRAIL. */
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
                        /* Dump aussi 16 valeurs sur la pile à partir de SP. */
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
                /* RCD is *delayed*: per SPRU172C the next 2 instructions
                 * after RCD execute before the return takes effect. The
                 * old "skip delay slots" implementation broke FB-detection
                 * because slots like `LD #0, B` at PROM0 0x75ea were never
                 * run, leaving accumulator state stale and the dispatcher
                 * at 0x7700 looping forever.
                 *
                 * Fix: arm the existing delayed_pc/delay_slots machinery —
                 * pop the return address now, advance PC normally so the
                 * next 2 instructions execute as delay slots, then the
                 * main loop forces PC = delayed_pc. */
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
        /* CMPS src, Smem — Compare, Select, and Store (Viterbi)
         * Encoding: 1110 00SD IAAAAAAA (1 word)
         * Per SPRU172C p.4-35: if |A(32-16)| >= |Smem| then TC=1,
         * TRN = (TRN<<1)|1, dst=A; else TC=0, TRN=(TRN<<1), dst=Smem<<16 */
        /* ================================================================
         * [2026-08-22] FIX ISA (RAPPORT_OPCODES.md E-1, gravite 1) —
         * 0xE0-0xE3 N'EST PAS `CMPS`. Le vrai CMPS est 0x8E00/0xFE00, deja
         * correctement decode plus bas (corrige le 2026-06-02). Ce catch-all
         * etait un doublon parasite — meme schema que 0xC7->RPTB et 0xE4->BITF.
         * Encodages reels (SPRU172C, XXXXYYYY en bits 7:0) :
         *     0xE0 FIRS  Xmem,Ymem,pmad   ** 2 MOTS **
         *     0xE1 LMS   Xmem,Ymem        1 mot
         *     0xE2 SQDST Xmem,Ymem        1 mot
         *     0xE3 ABDST Xmem,Ymem        1 mot
         * Le seul degat GRAVE est 0xE0 compte pour 1 mot -> desynchronisation.
         * On corrige donc d'abord le NOMBRE DE MOTS, et on rend E1-E3 inertes
         * plutot que de leur faire ecrire A/B/TC/TRN a tort (une semantique
         * fausse est pire qu'une absence : cf. le patron « neutralisation
         * plausible » de l'audit). Implementation fidele de FIRS/LMS/SQDST/
         * ABDST = travail separe, a faire si une sonde montre qu'ils portent.
         * Bascule A/B : CALYPSO_ISA_E0_FAM=0 restaure l'ancien CMPS.
         * ================================================================ */
        {
            static int _e0_fam = -1;
            if (_e0_fam < 0) {
                _e0_fam = calypso_gate("CALYPSO_ISA_E0_FAM", 1);
                fprintf(stderr, "[c54x] ISA-E0-FAM %s : 0xE0=FIRS(2 mots) "
                        "0xE1=LMS 0xE2=SQDST 0xE3=ABDST (PAS CMPS)\n",
                        _e0_fam ? "ACTIF (largeur de mot fidele)"
                                : "INACTIF (ancien CMPS 1 mot)");
            }
            if (_e0_fam && (op & 0xFC00) == 0xE000) {
                static unsigned _e0_n = 0;
                if (hi8 == 0xE0) {
                    uint16_t pmad0 = prog_fetch(s, s->pc + 1);
                    consumed = 2;
                    /* [2026-08-22] FIRS-COEF (CALYPSO_FIRS_COEF, defaut OFF).
                     * Ou sont les coefficients ? pmad=0x0061 est SOUS la fenetre
                     * OVLY (qui commence a 0x80), donc prog_read retombe sur
                     * s->prog[] ou rien n est charge sous 0x7000 -> 0xF4E4.
                     * 0x0060-0x007F est le SCRATCH-PAD DARAM du C54x. On lit les
                     * DEUX espaces au moment du FIRS :
                     *   data[] non nul -> etendre l alias OVLY a 0x60 ;
                     *   prog[] non nul -> MVDP les a deposes, rien a corriger ;
                     *   les deux nuls  -> la table est ailleurs.
                     * LECTURE SEULE, independante de CALYPSO_ISA_FIRS. */
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
                                /* la SOURCE de la table : MVDD (0x833c) copie
                                 * data[0x2cbf] en DESCENDANT vers data[0x0060]
                                 * en montant. Si elle est vide, la destination
                                 * l est aussi — c est le vrai point d entree. */
                                for (int k = 0; k < 7; k++) {
                                    uint16_t sv = s->data[0x2CB9 + k];
                                    if (sv) nzs++;
                                    so += snprintf(sb2 + so, sizeof(sb2) - so, " %04x", sv);
                                }
                                /* [2026-08-22] v3 : le tampon de burst est-il
                                 * VIVANT a l instant ou le DSP le lit ? Toute la
                                 * chaine (0x8202 -> 0x2cba -> MVDD -> 0x0061 ->
                                 * FIRS) rend zero parce que l amont est nul, et
                                 * l amont c est 0x2a00. Un tampon vide ICI n est
                                 * pas un tampon mal ecrit : DARAM-WR-JUDGE avait
                                 * mesure coh=0.998 A L ECRITURE, sous verrou. */
                                /* [2026-08-22] v4 : dispersion des DEUX tampons du
                                 * correlateur (A=0x2c56, B=0x2c88, 50 mots chacun,
                                 * BRC=0x31). distinct==1 => il n accumule pas par
                                 * decalage : argmax sans objet. */
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
                    /* [2026-08-22] FIRS IMPLEMENTE. Il est dans le chemin de
                     * PRODUCTION DES BITS SOUPLES du SCH :
                     *   0x8492 rpt #5 ; 0x8493 firs 0x0061 ; 0x8497 sth *AR6+,B
                     * Le test de perturbation a innocente le papillon de Viterbi
                     * (le syndrome varie des que les 78 mots d'entree sont
                     * couverts) -> le defaut est en amont, ici.
                     * Semantique (spru172c.md:1212) :
                     *     B = B + A(32-16) x Pmem[pmad] ; A = (Xmem+Ymem)<<16
                     * Sous RPT, pmad s'auto-incremente a chaque repetition.
                     * Gate CALYPSO_ISA_FIRS=0 -> ancien comportement inerte. */
                    static int _fi = -1;
                    if (_fi < 0) {
                        _fi = calypso_gate("CALYPSO_ISA_FIRS", 1);
                        fprintf(stderr, "[c54x] ISA-FIRS %s : B += A(32-16)*Pmem[pmad] ; "
                                "A = (Xmem+Ymem)<<16 (pmad auto-incremente sous RPT)\n",
                                _fi ? "IMPLEMENTE"
                                   : "ARITHMETIQUE DESACTIVEE (les pointeurs avancent quand meme)");
                    }
                    /* [2026-08-22] Operandes decodes HORS du gate : leurs
                     * post-modifications doivent avoir lieu meme quand
                     * l arithmetique est desactivee (voir plus bas). */
                    int xmod = (op >> 6) & 0x03;
                    int xar  = ((op >> 4) & 0x03) + 2;
                    int ymod = (op >> 2) & 0x03;
                    int yar  = ( op       & 0x03) + 2;

                    if (_fi) {
                        /* pmad auto-increment : meme PC repete => on avance. */
                        static uint16_t _fpm = 0; static uint16_t _fpc = 0xFFFF;
                        /* [2026-09-17] FIX_FIRS_RPT : un second `rpt ; firs` au MEME
                         * site continuait a _fpm+1 au lieu de repartir de pmad
                         * (le SB en a 2 : 0x8478 et 0x8493, 1704 tours par burst). */
                        static int fix_firs = -1;
                        if (fix_firs < 0) fix_firs = calypso_gate("CALYPSO_FIX_FIRS_RPT", 1);
                        if (fix_firs) {
                            if (!s->rpt_active || s->rpt_fresh) { _fpm = pmad0; s->rpt_fresh = false; }
                            else                                  { _fpm++; }
                            _fpc = s->pc;
                        } else
                        if (s->pc != _fpc) { _fpm = pmad0; _fpc = s->pc; }
                        else               { _fpm++; }

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
                    /* [2026-08-22] POST-MODIFICATIONS HORS DU GATE.
                     * Un gate de diagnostic doit COMPARER, pas corrompre. Sur le
                     * C54x, FIRS avance Xmem et Ymem, que l on modelise ou non
                     * son arithmetique.
                     * Mesure qui l impose (point d arret sur s->pc == 0x8478) :
                     * avec ISA_FIRS=0, AR2 et AR3 restaient figes a 0x2a8e et
                     * 0x2a9a sur SIX executions. La boucle RPTB de l etage
                     * (BRC=0x8d, 142 tours) emportait alors AR3 hors du tampon de
                     * burst jusqu a 0x2ceb, ou il ecrasait la REFERENCE DE
                     * CORRELATION — capture au point de surveillance materiel,
                     * six declenchements en alternance parfaite :
                     *   0x7c50 : 0x223c -> 0x0200  (installe la reference)
                     *   0x847a : 0x0200 -> 0x223c  (l ecrase, via AR3)
                     * D ou tampon A du correlateur fige, argmax sans objet, SB
                     * jamais decode.
                     * ISA_FIRS=0 signifie desormais « pas d arithmetique », plus
                     * « pas d instruction du tout ».
                     * ⚠️ Effet GLOBAL comme tout correctif d ISA : le FIRS sert
                     * ailleurs que dans le banc du SCH. */
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
        if ((op & 0xFC00) == 0xE000) {
            int src_s = (op >> 9) & 1;
            int dst_d = (op >> 8) & 1;
            addr = resolve_smem(s, op, &ind);
            uint16_t val = data_read(s, addr);
            int64_t acc = src_s ? s->b : s->a;
            int32_t ah = (int32_t)((acc >> 16) & 0xFFFF);
            if (ah < 0) ah = -ah;
            int32_t sv = (int16_t)val;
            if (sv < 0) sv = -sv;
            s->trn <<= 1;
            if (ah >= sv) {
                s->st0 |= ST0_TC;
                s->trn |= 1;
            } else {
                s->st0 &= ~ST0_TC;
                int64_t nv = (int64_t)(int16_t)val << 16;
                if (dst_d) s->b = sext40(nv); else s->a = sext40(nv);
            }
            return consumed + s->lk_used;
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
                case 3: s->ar[xar] = c54x_circ_ref(xa, +(int16_t)s->ar[0], s->bk); break; /* *AR+0% circ modulo BK — fix 2026-06-01 */
            }
            switch (ymod) {
                case 0: break;
                case 1: s->ar[yar] = ya - 1; break;
                case 2: s->ar[yar] = ya + 1; break;
                case 3: s->ar[yar] = c54x_circ_ref(ya, +(int16_t)s->ar[0], s->bk); break; /* *AR+0% circ modulo BK — fix 2026-06-01 */
            }
            return consumed + s->lk_used;
        }
        /* ================================================================
         * [2026-08-22] FIX ISA (RAPPORT_OPCODES.md E-2, gravite 1) —
         *     ST src,Ymem || LD Xmem,T      encodage 111001S0 XXXXYYYY
         *     = 0xE400 masque 0xFD00  (donc 0xE4xx ET 0xE6xx, b8=0 fixe)
         *
         * L'emulateur decodait 0xE4 en « BITF Smem,#lk » sur **2 MOTS** ->
         * un mot avale par site -> desynchronisation du flux d'instructions.
         * Or le VRAI BITF est 0x6100/0xFF00, deja correctement decode ailleurs
         * dans ce fichier : c'etait un doublon parasite, exactement comme
         * 0xC7 -> RPTB (le vrai RPTB etant 0xF072).
         *
         * /!\ NE PAS elargir le masque a 0xFC00 : 0xE5xx = MVDD et 0xE7xx =
         * MVMM sont des instructions DIFFERENTES (b8 discrimine) et sont
         * reellement utilisees (79 MVDD dans PROM0, dont le corps du
         * correlateur en PDROM). Le masque projet 0xFC00 les volait.
         *
         * Semantique (SPRU172C p.4-178, syntaxe 2) :
         *     Ymem = (src << ASM) >> 16   ;   T = Xmem
         * S = b9 (0xE4 -> A, 0xE6 -> B). 1 MOT.
         * Bascule A/B : CALYPSO_ISA_E4_PAR=0 restaure l'ancien BITF 2 mots.
         * ================================================================ */
        {
            static int _e4_par = -1;
            if (_e4_par < 0) {
                _e4_par = calypso_gate("CALYPSO_ISA_E4_PAR", 1);
                fprintf(stderr, "[c54x] ISA-E4-PAR %s : 0xE4xx/0xE6xx = "
                        "ST src,Ymem || LD Xmem,T (1 mot, masque 0xFD00)\n",
                        _e4_par ? "ACTIF (fidele SPRU172C)"
                                : "INACTIF (ancien BITF 2 mots)");
            }
            if (_e4_par && (op & 0xFD00) == 0xE400) {
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
        }
        if (hi8 == 0xE4) {
            /* E4xx: BITF Smem, #lk (2-word) or BIT Smem, bit */
            addr = resolve_smem(s, op, &ind);
            op2 = prog_fetch(s, s->pc + 1);
            consumed = 2;
            uint16_t val = data_read(s, addr);
            s->st0 = (val & op2) ? (s->st0 | ST0_TC) : (s->st0 & ~ST0_TC);
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
             * NOTE: This was previously decoded as CC (conditional call, 2-word)
             * which caused stack overflow by pushing return addresses in a loop. */
            int dst = (op >> 8) & 1;
            uint8_t k = op & 0xFF;
            int64_t v = (s->st1 & ST1_SXM) ? (int64_t)(int8_t)k : (int64_t)k;
            /* [2026-07-23] FIX ISA : LD #k8u charge l'immediat dans les bits BAS (sext40(v)),
             * PAS v<<16. Le <<16 mettait 0x39 en bits 16-23 -> au terminal mask-ROM :
             * 0xb408 LD #0x39 + 0xb409 ADD #0x4387 donnait A=0x394387 -> STLM AR7=0x4387
             * (slot IDLE data[0x4387]=0xab38) au lieu de 0x0039+0x4387=0x43C0 (slot go-live
             * data[0x43c0]=0xa4c7). => terminal 0xb40f BACC vers idle 0xab38 = LE STORM.
             * Verifie runtime (TERM-TRACE). Correct c54x SPRU172C : LD #k8 -> low bits.
             * Env CALYPSO_LDK8_SHIFT16=1 restaure l'ancien comportement (A/B). */
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
            /* FRAME #k8 — stack-frame pointer adjust : SP = SP + sign_ext(k8).
             * Per tic54x-opc.c { "frame", 1,1,1, 0xEE00, 0xFF00, {OP_k8} }
             * = 1 MOT, et SPRU172C §4 (FRAME adjusts SP by a signed 8-bit imm).
             * FRAME -N alloue le cadre (SP descend) ; FRAME +N le libère.
             *
             * BUG FIX 2026-05-31 : ce handler décodait 0xEExx en "BCD pmad,cond"
             * (branche conditionnelle différée 2-MOTS) — FAUX sur les 2 plans :
             *   (1) longueur : 1 mot, pas 2 → désync de tout l'aval
             *   (2) sémantique : SP+=k8, pas une branche
             * BCD n'existe même pas en 0xEE (le vrai bc=0xF8, cc=0xF9, bcd=0xFA).
             * 124 sites 0xEExx en PROM0, dont le chemin de boot (paires
             * FRAME #-1/#+1 = prologue/épilogue). Le SP jamais ajusté → over-pop
             * (SP-EVENTS pops>pushes) → POPM ST0 @0x94f3 ramasse l'orphelin
             * 0x80fd → DP=0x0fd → dispatcher LUT garbage → self-CALA 0x70c3 →
             * écrit 0x70c4 (=28868) dans d_fb_det/a_pm → rxlev/TOA poison.
             * cf doc/SP_CATASTROPHE_70c4_SEQUENCE.md. */
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
         * accumulator A is preserved (we mirror via mvpd_src state).
         *
         * 2026-05-27 c web review revert : a speculative 23-bit fix
         * (A.high → XPC override) was tried but contradicts SPRU131G and
         * did not move the symptom — reverted to canonical semantics. */
        if (hi8 == 0x7E) {
            /* ═════════════════════════════════════════════════════════════════
             * [2026-08-04] READA-ITER — pourquoi la 2e boucle du chargeur de
             * table ne fait que DEUX copies au lieu de trois.
             *
             * MESURE QUI L'IMPOSE. Le chargeur `0xb4b6` fait deux boucles :
             *   0xb4bc  rpt #0x4d ; reada *AR1+   -> 77 copies, 0x4387..0x43d3
             *                                        EXACT (77 = 0x4d + 1)
             *   0xb4c4  rpt #0x02 ; reada *AR1+   -> 2 copies, 0x43d5 et 0x43d6
             *                                        UNE MANQUE (attendu 3)
             * `RPT` compte donc juste dans un cas et court d'une unite dans
             * l'autre : ce n'est PAS `RPT` en lui-meme. Consequence mesuree sur
             * 76 passes : `data[0x43d7]`, entree 2 de la table de dispatch des
             * taches, n'est JAMAIS initialisee.
             *
             * CE QUE LA SONDE TRANCHE : a chaque execution de la 2e boucle, on
             * imprime AR1 AVANT resolve_smem, l'adresse resolue, et l'etat du
             * repeat. Deux issues :
             *   - trois lignes, la 3e avec une adresse hors 0x43d7 -> le pointeur
             *     derape (probleme d'adressage `*AR1+`) ;
             *   - deux lignes seulement -> la 3e iteration n'a pas lieu, et c'est
             *     l'interaction repeat/instruction qu'il faut instruire.
             *
             * Bornee au PC de la SECONDE boucle (0xb4c5) pour ne pas noyer le
             * journal avec les 77 copies de la premiere. Plafond 60 lignes.
             * ═════════════════════════════════════════════════════════════════ */
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
            /* GAP-1/Phase B fix (2026-06-24) : sous RPT, la 1ere iteration part
             * de A_low (base source), PAS du mvpd_src stale d'un READA precedent.
             * Sans ca, `RPT #N ; READA *ARx+` copiait depuis la mauvaise zone ROM
             * -> table de dispatch (0x4380+) remplie de garbage (0xf074) -> bacc
             * vers la LUT au lieu du vrai handler FB (0xab38). */
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
            { /* [2026-08-04] plafond 20 -> 200 : la PREMIERE boucle (rpt #0x4d = 77
                 * copies) le consommait entierement, rendant la SECONDE
                 * (rpt #0x02 vers la table de dispatch 0x43d5) invisible. */
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
         * Was previously misdecoded as LDM MMR,dst (1 word) — copy/paste
         * of the wrong mnemonic. The real LDM is 0x48xx mask 0xFE00,
         * already correctly handled in the 0x4 group. Misdecoding caused
         * PC to advance by 1 instead of 2-3 ; the literal then executed
         * as a stray opcode. In particular the 0x4F00 (DST B,Lmem with
         * DP=0 → MMR_IMR) stray write zeroed IMR forever, masking
         * INT3+BRINT0 → DSP parked in RPTB at e9ab..e9b6 awaiting a
         * frame interrupt that was never serviced. Fix 2026-05-08. */
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
            /* WATCH-ST1-WRITE : MMR 0x07 = ST1. Capture toutes les
             * écritures de ST1 (STM #lk, ST1) — incluant celles qui
             * ne changent pas la valeur d'INTM mais redéfinissent
             * tout le mot ST1. Sortie : valeur écrite, bit 11 (INTM),
             * delta vs current ST1. Cap 200 entries pour boot, puis
             * sample 1/100. */
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
        /* 0x72/0x73 (MVDM/MVMD) : RESTENT REVERTÉS (fallthrough STL générique).
         *
         * FINDING 2026-06-02 (cause (c) localisée + prouvée, mais fix bloqué) :
         * DECODE-AUDIT gaté insn>250M a prouvé qu'à PC=0xf564 op=0x7215 = MVDM
         * data[0x0014]→AR5, AU CŒUR de la boucle dispatch FB (0xf561-0xf588),
         * était décodé 1-mot → l'opérande 0x0014 exécutée comme opcode (0xf565
         * hi8=00) → desync chaque itération → AR5 (ptr handler tâche) jamais
         * chargé → tâche FB jamais dispatchée → 0x9ac0 jamais ré-atteint
         * past-boot → d_fb_det jamais armé. = cause (c) « décision jamais
         * atteinte ». LE MIS-DÉCODE EST RÉEL.
         *
         * MAIS appliquer MVDM 2-mots (ISA-correct : data[MMR]=data[dmad]) — même
         * 0x72 SEUL — RÉGRESSE en deadlock pire : le dispatch avance bien à
         * PC=0xee38 (task_md=5 lu sur les 2 pages = progrès), mais AR3 y pointe
         * HORS du buffer I/Q (0x2b97 > 0x2b28) → corrèle des zéros (A=0) → BSP ne
         * livre plus (delivered=0) → INT3 ne fire plus (irq 3860→4) → deadlock.
         * = le « bug compensateur upstream » du revert (REVERT_MVMD_KNOWLEDGE.md) :
         * le setup d'AR3/pointeurs en amont de 0xee38 est aussi mal émulé, et le
         * STL mis-décodé compensait. Critère de ré-application : fixer d'abord le
         * deadlock 0xee38 (AR3 hors-buffer) — passe séparée. Voir
         * project_state_20260602 mémoire. */
        /* === PORTR 0x74 / PORTW 0x75 — longueur 3 mots si Smem absolu (revival
         * c54x, 2026-06-23). tic54x-opc.c : portr {2,2,2,0x7400,0xFF00,
         * {OP_PA,OP_Smem}} ; portw {2,2,2,0x7500,0xFF00,{OP_Smem,OP_PA}}.
         * Base = 2 mots (opcode + PA) ; +1 mot quand le Smem utilise l'adressage
         * absolu/long (binutils get_insn_size = words + has_lkaddr). Le catch-all
         * générique `(op & 0xF800)==0x7000` ci-dessous les avalait en STL 1-mot
         * (+lk) → perdait le mot PA → glissement d'alignement d'1 mot. Symptôme
         * prouvé (oracle binutils-2.21.1) : PROM0 0xb416 `portw *(0x000e),0xf900`
         * mal-dimensionné 2 mots → son PA 0xf900 relu comme un CC fantôme @0xb418
         * → entrée nue dans l'épilogue 0x76f8 (POPM ST1 sans PSHM ST1) → sur-pop
         * SP → collapse → d_fb_det=0. 128 `75f8` + 25 `74f8` dans le firmware.
         * resolve_smem lit l'adresse Smem abs @pc+1 (pose lk_used) ; le PA suit
         * @pc+1+lk_used — même convention que CMPM/BITF ci-dessous.
         * NB : sémantique I/O réelle (PORTW: Smem→port PA ; PORTR: port PA→Smem,
         * lien I/Q bsp_buf) = passe séparée ; ici on corrige d'abord la LONGUEUR
         * (le 1er domino, falsifiable). */
        if ((op & 0xFF00) == 0x7500) {        /* PORTW Smem, PA */
            addr = resolve_smem(s, op, &ind); /* applique le post-modify AR, pose lk_used si abs */
            uint16_t pa = prog_fetch(s, s->pc + 1 + (s->lk_used ? 1 : 0));
            /* [2026-08-03] RIF (calypso_rif.c) : SPCR/SPCX/DXR sont maintenant
             * reellement ecrits — c'est par SPCR que le firmware ouvre RINT_MASK
             * ou RDMA_MASK, donc qu'il CHOISIT le mode du §3.7.1. */
            if (calypso_rif_portw(s, pa, data_read(s, addr))) {
                consumed = 2;
                return consumed + s->lk_used;
            }
            {   /* fenetre DMA cote DSP — cf. PORTR ci-dessous */
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
            (void)pa; (void)addr;             /* autre port : non modélisé */
            consumed = 2;                     /* opcode + PA */
            return consumed + s->lk_used;
        }
        if ((op & 0xFF00) == 0x7400) {        /* PORTR PA, Smem */
            addr = resolve_smem(s, op, &ind);
            uint16_t pa = prog_fetch(s, s->pc + 1 + (s->lk_used ? 1 : 0));
            /* [2026-07-23] PORTR-ANY (READ-ONLY, inconditionnel) : compte TOUT hit
             * PORTR quel que soit PA, pour distinguer "opcode jamais atteint" de
             * "atteint mais PA != 0xF430/0x0034". Cap 30. */
            {
                static unsigned _pany = 0;
                if (_pany++ < 30)
                    fprintf(stderr, "[c54x] PORTR-ANY #%u PA=0x%04x addr=0x%04x PC=0x%04x insn=%u\n",
                            _pany, pa, addr, s->pc, s->insn_count);
            }
            /* PA=0xF430 (ou 0x0034 legacy) = port RX BSP : livre l'echantillon
             * I/Q suivant depuis bsp_buf (rempli par DMA radio ; bsp_pos reset
             * par rafale). Le handler dead-code 0x8F (~8530) etait la ref
             * ISA-fausse ("relocaliser PORTR vers 0x74 un jour") : c'est fait ici.
             * Sans ca, notre length-fix no-op shadow-ait la vraie lecture I/Q ->
             * l'AFC/correlateur ne recevait AUCUN echantillon -> jamais de
             * convergence. GATED CALYPSO_FIX_PORTR. */
            static int fix_portr = -1;
            if (fix_portr < 0) fix_portr = calypso_gate("CALYPSO_FIX_PORTR", 0);
            if (fix_portr && (pa == 0xF430 || pa == 0x0034)) {
                uint16_t iq = (s->bsp_pos < s->bsp_len) ? s->bsp_buf[s->bsp_pos++] : 0;
                data_write(s, addr, iq);
                static unsigned pr_n = 0;
                if (pr_n++ < 50)
                    fprintf(stderr, "[c54x] PORTR-IQ #%u PA=0x%04x -> data[0x%04x]"
                            "=0x%04x pos=%d/%d PC=0x%04x insn=%u\n",
                            pr_n, pa, addr, iq, s->bsp_pos, s->bsp_len, s->pc,
                            s->insn_count);
            }
            /* [2026-08-03] RIF : PORTR n'etait un no-op que parce que le RIF
             * n'existait pas dans le modele. Il existe maintenant (calypso_rif.c,
             * CAL207 §12) et c'est la seule cible que le firmware interroge :
             * 30 releves PORTR-ANY sur 30 valaient PA=0x0003 = SPCR. */
            {
                uint16_t rv;
                if (calypso_rif_portr(s, pa, &rv)) {
                    data_write(s, addr, rv);
                    consumed = 2;
                    return consumed + s->lk_used;
                }
                /* [2026-08-03] Fenetre DMA vue du DSP (XIO:FC00..FCFF, §11.1).
                 * L'ARM a cede les canaux du RIF au DSP (ALLOC_CONFIG=0x000C
                 * mesure) : c'est donc ICI que passe la config du transfert. */
                rv = 0;
                if (calypso_rhea_dma_xio(false, pa, &rv, s->pc)) {
                    data_write(s, addr, rv);
                    consumed = 2;
                    return consumed + s->lk_used;
                }
                /* API Control (F900) + INTH du DSP (FA00) — cf. calypso_xio.c */
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

        /* [2026-07-23] 0x73xx: MVMD MMR, dmad — MMR -> data[dmad] (2-word, direction SAVE).
         * BUG REEL (SP-CORRUPT watchpoint) : 0xa4f8 op=0x7318 = MVMD SP,0x3f6e = SAUVE SP.
         * Le fallthrough STL generique CHARGEAIT SP depuis data[0x3f6e]=garbage -> SP=0xc905
         * etc. -> derails eparpilles (0xa58d...) post-POPD. ISA-correct : lit le MMR
         * (SP=0x18 alias data 0x0018) et ecrit data[dmad] ; SP/MMR INCHANGE. On ne fixe
         * QUE 0x73 (MVMD, sens save, inoffensif) ; 0x72 (MVDM, sens charge) GARDE le revert
         * documente (regression AR3-hors-buffer @0xee38, REVERT_MVMD_KNOWLEDGE.md). */
        if ((op & 0xFF00) == 0x7300) {
            int mmr = op & 0x7F;
            uint16_t dmad = prog_fetch(s, s->pc + 1);
            data_write(s, dmad, data_read(s, mmr));
            consumed = 2;
            return consumed + s->lk_used;
        }

        /* 0x70xx: MVKD dmad, Smem  —  data[dmad] -> data[Smem].
         * binutils tic54x-opc.c {mvkd,2,2,2,0x7000,0xFF00,{OP_dmad,OP_Smem}}.
         * 2-mot base (opcode + dmad) + 1 mot si Smem absolu/long (mode 0xC..0xF,
         * has_lkaddr). Ordre des operandes = dmad EN PREMIER (pc+1), lk Smem-abs
         * EN SECOND (pc+2).
         *
         * GAP-1 ROOT (tracee 2026-06-23 via sonde AR3-TRIP) : le catch-all
         * generique (op&0xF800)==0x7000 ci-dessous decodait 0x70xx en STL 1-mot
         * (+lk), SOUS-CONSOMMANT le mot dmad. A PROM0 0xb3cc (`70f8 4356 00e3`)
         * le 3e mot 0x00e3 etait alors execute comme `ADD *AR3(lk)` parasite ;
         * de meme 0xb3d1=0x00db -> `ADD *AR3+0%,A` faisait AR3 += AR0(~=SP) a
         * chaque tour -> AR3 balayait la memoire -> ecrasait data[0x0c36]
         * (ptr tache) -> CALA 0 -> POST-BOOTSTUB-RET -> derail/boucle. Le decode
         * de l'ADD lui-meme etait FIDELE ; le bug etait la LONGUEUR du MVKD amont.
         * Note : 0x72/0x73 (MVDM/MVMD) restent volontairement non-fixes ici
         * (revert documente, REVERT_MVMD_KNOWLEDGE.md) ; ce fix 0x70 est le
         * prerequis « fixer d'abord le setup AR3 amont » mentionne la-bas. */
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
                } else {                                 /* 0xF : *(lk) absolu */
                    smem_addr = lk;
                }
                s->st0 = (s->st0 & ~ST0_ARP_MASK) | (nar << ST0_ARP_SHIFT);
                s->lk_used = true;
            } else {                           /* direct ou indirect non-abs */
                smem_addr = resolve_smem(s, op, &ind);  /* +post-modify AR */
            }
            data_write(s, smem_addr, data_read(s, dmad));
            consumed = 2;
            return consumed + (s->lk_used ? 1 : 0);
        }

        /* 0x71xx: MVDK Smem, dmad  —  data[Smem] -> data[dmad].  MIROIR de MVKD.
         * binutils tic54x-opc.c {mvdk,2,2,2,0x7100,0xFF00,{OP_Smem,OP_dmad}}.
         * Encodage IDENTIQUE a MVKD 0x70 (Smem dans l'octet bas de l'opcode,
         * dmad@pc+1, lk Smem-abs@pc+2) ; SEULE la direction du move est inversee :
         * MVKD fait data[Smem]=data[dmad] ; MVDK fait data[dmad]=data[Smem].
         *
         * FINDING 2026-06-24 (recon workflow, desasm verifie adversarialement sur
         * le vrai dump /opt/GSM/calypso_dsp.txt) : a PROM0 0xb3db-0xb3e3 TROIS MVDK
         * 3-mots `71f8 4356 00e3` / `71f8 4357 00db` / `71f8 4355 00d3` RESTAURENT
         * data[0x4356/4357/4355] (sauves par les 3 MVKD symetriques @0xb3cc avant
         * les 3 CALL). Le catch-all `(op&0xF800)==0x7000` les decodait en STL 1-mot
         * -> SOUS-CONSOMMAIT le mot dmad -> les operandes 00e3/00db/00d3 executees
         * en ADD parasites (dont 0x00db = `ADD *AR3+0%%,A`, l'instr GAP-1) chaque
         * trame -> corruption AR3/A + restore rate. MEME CLASSE que le fix MVKD 0x70.
         * 0x71 est data<->data (PAS MMR) -> ORTHOGONAL au revert 0x72/0x73
         * (REVERT_MVMD_KNOWLEDGE.md, qui concerne la corruption MMR via STL). */
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

        /* 0x72 MVDM dmad,MMR (MMR<-data[dmad]) / 0x73 MVMD MMR,dmad (data[dmad]<-MMR).
         * 2-mot (opcode + dmad ; MMR = octet bas, mappee a data 0x00-0x1f que
         * data_read/data_write routent vers les registres). GATED CALYPSO_FIX_MVDM
         * car REVERT_MVMD_KNOWLEDGE.md documente une regression (deadlock 0xee38,
         * AR3 hors buffer I/Q) quand on fixe AVANT le setup AR3 amont. Ce setup =
         * MVKD 0x70 (GAP-1) + MVDK 0x71 = MAINTENANT FAITS -> critere de
         * re-application atteint. Debloque la sous-routine go-live 0xaad5
         * (7211 434f MVDM data[0x434f]->AR1 ; 7210 434e ; 7310 434e MVMD AR0->
         * data[0x434e]) dont le mis-decode (catch-all STL 1-mot) fige A=0 -> garde
         * BC 0xa4cd (AEQ, A==0) jamais relachee -> RSBX INTM 0xa51b jamais atteint. */
        {
            static int fix_mvdm = -1;
            /* [2026-07-23] DEFAULT ON : fix ISA MVDM/MVMD (decode conforme tic54x-opc).
             * Debloque la SM go-live 0xaad5 (A fige a 0 sinon) -> D_TASK_MD-RD 0->1859,
             * DSP lit db_w + atteint dispatcher trame. Regression 2026-05-15 (0xfd23/fd25)
             * levee (setup amont MVKD 0x70/MVDK 0x71 fait). OFF via CALYPSO_FIX_MVDM_OFF. */
            if (fix_mvdm < 0) fix_mvdm = getenv("CALYPSO_FIX_MVDM_OFF") ? 0 : 1;
            if (fix_mvdm && (op & 0xFF00) == 0x7200) {       /* MVDM dmad, MMR */
                uint16_t dmad = prog_fetch(s, s->pc + 1);
                uint16_t mmr  = op & 0x00FF;
                data_write(s, mmr, data_read(s, dmad));
                consumed = 2;
                return consumed;
            }
            if (fix_mvdm && (op & 0xFF00) == 0x7300) {       /* MVMD MMR, dmad */
                uint16_t dmad = prog_fetch(s, s->pc + 1);
                uint16_t mmr  = op & 0x00FF;
                data_write(s, dmad, data_read(s, mmr));
                consumed = 2;
                return consumed;
            }
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
        /* [2026-09-17] FIX_MACP_MACD — macp Smem,pmad,src (0x7800/0xFE00) et
         * macd Smem,pmad,src (0x7A00/0xFE00), bit 8 = src, 2 MOTS (+lk du Smem).
         * Tombaient dans le bloc « STH » a 1 mot ci-dessous : le pmad etait
         * ensuite execute comme instruction (trace SB c54x_exe --arm 2026-09-17,
         * 0x7e2a `7892 7a1c` sous RPT, puis `7a1c` pris pour un MACD).
         * SPRU172C : src = src + Smem x Pmem(pmad) ; T = Smem ; MACD recopie en
         * plus Smem dans Smem+1. Sous RPT, pmad s'incremente a chaque tour
         * (meme suivi que READA/MVPD : rpt_fresh + mvpd_src). */
        {
            static int fix_macp = -1;
            if (fix_macp < 0) fix_macp = calypso_gate("CALYPSO_FIX_MACP_MACD", 1);
            if (fix_macp && (op & 0xFC00) == 0x7800) {
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
         *   CMPM *(0x0fff), 4   →  CMPM *(0x0fff), 2
         * to wait for ARM-side BL_CMD_STATUS write. Without TC being set
         * the subsequent BC NTC always branches back, looping forever.
         * Was previously folded into the generic 0x6000-0x67FF "LD" path
         * which set the accumulator instead and never updated TC. */
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
            /* FBWATCH : capture EXACTE au site de poll foreground (0xf7af/0xf7b7)
             * — addr résolue + valeur data_read + TC. ID le flag jamais vrai. */
            if (g_fbwatch_on > 0 && (s->pc == 0xf7af || s->pc == 0xf7b7)
                && s->insn_count > 100000) {   /* post-wire (1er wire @insn 32768) */
                static unsigned wbf = 0;
                if (wbf++ < 30)
                    fprintf(stderr, "[c54x] FBWATCH-BITF pc=0x%04x addr=0x%04x "
                            "mem=0x%04x mask=0x%04x -> TC=%d insn=%u\n",
                            s->pc, addr, mem_val, mask, tc_after, s->insn_count);
            }
            /* BITF instrumentation (2026-05-15 nuit) — pour confirmer si TC
             * est set correctement. Hypothèse : si BITF appelle souvent mais
             * tc_after=1 rarement → masque/mem_val pattern empêche TC=1,
             * ce qui fait que BC NTC branche toujours et `ST #1, d_task_d`
             * à PROM 0x9ab1 n'est jamais atteint. Format :
             *   BITF-PROBE #N PC=0xXXXX addr=0xXXXX mem=0xXXXX mask=0xXXXX
             *               tc_before=N tc_after=N
             * Cap 200 + 1/1000 ensuite. */
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
        /* [2026-09-17] FIX_MPY_MAC_LK — mpy Smem,#lk,dst (0x6200/0xFE00, bit 8 =
         * dst) et mac Smem,#lk,src[,dst] (0x6400/0xFC00, bit 9 = src, bit 8 =
         * dst) : 2 MOTS (+lk du Smem). Tombaient dans le bloc « LD » a 1 mot
         * ci-dessous : le #lk etait execute comme instruction (trace SB c54x_exe
         * --arm 2026-09-17 : 0x762c `6283 36f6`, 0x7692 `6283 5a82` = cos 45).
         * SPRU172C : dst = Smem x lk ; dst = src + Smem x lk ; T = Smem. */
        {
            static int fix_mlk = -1;
            if (fix_mlk < 0) fix_mlk = calypso_gate("CALYPSO_FIX_MPY_MAC_LK", 1);
            if (fix_mlk && (op & 0xF800) == 0x6000 && (op & 0x0600) != 0) {
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
         * Without these, the fallback at (op & 0xF800) == 0x6800 below
         * mis-decodes them all as LD Smem,T (1-word), causing PC drift +1
         * word and the lk/pmad operand executing as parasitic instruction.
         * 1259 (ANDM/ORM/XORM/ADDM) + 304 (BANZ/BANZD) = 1563 sites in ROM.
         *
         * 2026-04-28 — companion fix to 0x6F00 already inserted below.
         * See doc/opcodes/0x68_0x6F.md for spec. */
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
             * is non-zero. Test on PRE-modify value; resolve_smem applies
             * post-mod regardless of branch outcome. Previously read ARP
             * from ST0 (the PREVIOUS instruction's nar) — wrong AR was
             * tested. Cf resolve_smem comment for the off-by-ARP bug. */
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
         * Without this handler, the fallback at (op & 0xF800) == 0x6800 below
         * mis-decodes 0x6Fxx as LD Smem,T (1-word), causing PC drift +1 word
         * and the lk-side operand to be executed as parasitic instruction.
         * 544 sites in firmware ROM. See doc/opcodes/0x68_0x6F.md for spec.
         *
         * 2026-04-28 — fix introduced for wedge at PC=0x8353 (CALAD A self-loop)
         * caused by 0x6F07 0x0C41 mis-decoded → 0x0C41 executed as parasitic
         * SUB Smem,TS,A → A_low=0xFFFA → A_low=0x8353 after subsequent ADD. */
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
            /* DEAD CODE since 2026-04-28: all 0x68xx-0x6Fxx now intercepted
             * by specific handlers above (ANDM/ORM/XORM/ADDM/BANZ/BANZD/
             * extended-0x6F00) plus the existing 0x6Dxx MAR. This generic
             * "LD Smem, T" fallback was the source of the 2107-site mass
             * mis-dispatch that caused PC drift on every 0x68xx-0x6Fxx
             * encounter. Kept here for safety in case a previously unseen
             * sub-encoding slips through; if you ever see this trigger,
             * the new handler above for the matching 0xNN00 prefix is
             * incomplete. See doc/opcodes/0x68_0x6F.md. */
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
         * Critical: bootloader at PROM0 0xb429 does `LDU *(0x0ffe), A`
         * (op=0x12f8 + lk=0x0ffe) to read BL_ADDR_LO, then BACC A to that
         * target. The previous "case 0x1: SUB" decoded this as a subtract,
         * leaving A=0 and the BACC dropping into boot-stub NOPs. */
        addr = resolve_smem(s, op, &ind);
        int dst = (op >> 8) & 1;
        int sub = (op >> 9) & 0x07;  /* selects LD/LDU/LD,TS/LDR within case 1 */
        uint16_t val = data_read(s, addr);
        {   /* [2026-08-03] LD-TRACE — sous CALYPSO_DISPATCH_PROBE, LECTURE SEULE.
             * Borne au bloc 0xb05f..0xb078 (le dispatcher de tache), donc quelques
             * dizaines de lignes au plus.
             *
             * POURQUOI. Mesure du 03/08 : en 0xb060 (`10e1 0000` = LD *AR1(0), A),
             * AR1 vaut 0x0814 (= d_task_d page W1) et la cellule pointee contient
             * 0x0018 (= 24, ALLC) — les deux VERIFIES par la sonde CHAIN-B05F. Or
             * l'accumulateur ressort a 0x5294. La chaine compare donc 0x5294 aux
             * constantes 12/30/34, ne matche rien, et bailout vers 0xb077 : c'est
             * pour ca que la resolution d'index n'est jamais atteinte et que
             * l'armement RX n'est jamais demande.
             *
             * L'inspection statique ne suffit pas : `resolve_smem` traite MOD 0xC
             * correctement (`addr = AR + lk`) et ce handler-ci lit `data_read(addr)`
             * avec dst/sub corrects. L'ecart est donc AILLEURS sur le chemin, et
             * il faut les trois quantites cote a cote plutot que de le deviner —
             * deviner un decodage a coute 3 fausses pistes sur 3 le 30/07.
             *
             * LECTURE : si addr==0x0814 et val==0x0018 mais que l'accu final differe,
             * le defaut est dans l'ecriture de l'accumulateur (sub/dst/sext), pas
             * dans l'adressage. Si addr differe, c'est resolve_smem malgre tout. */
            uint16_t _pc = s->last_exec_pc;
            if (_pc >= 0xb05f && _pc <= 0xb078) {
                static int _lt = -1; static unsigned _ltn = 0;
                if (_lt < 0) _lt = calypso_gate("CALYPSO_DISPATCH_PROBE", 0);
                if (_lt && _ltn < 60) {
                    _ltn++;
                    fprintf(stderr,
                            "[dispatch] LD-TRACE pc=0x%04x op=0x%04x dst=%s sub=%d "
                            "addr=0x%04x val_lue=0x%04x data[addr]=0x%04x "
                            "A_avant=0x%06llx SXM=%d insn=%u\n",
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
        /* [2026-07-28] sub 4..7 : la moitie LOGIQUE de la famille tombait dans le
         * `default` ci-dessous et etait exécutée comme un LD. Encodages : table
         * projet doc/opcodes/tic54x_hi8_map.md + SPRU172C (tableaux 2-7/2-8/2-9
         * et SUBC p.4-192). Smem est ZERO-etendu sur 40 bits : l exemple TI de
         * AND (p.4-12) donne A=00 00FF 1200 & Smem=0x1500 -> A=00 0000 1000.
         * Impact mesure : 0x1860 (AND) lu comme LD mettait A=15 au lieu de A&15,
         * d ou T=31 et un `LD Smem,TS` decalant de +31 qui saturait l accumulateur
         * (A=0x80000000) et aplatissait la sortie du demod. */
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
        case 0x7: { /* 0x1E00: SUBC Smem, src — soustraction conditionnelle (division) */
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
        /* LDU-PTR (patch #2 diag, gated CALYPSO_DEBUG=LDU-PTR) : au site qui
         * charge A pour le CALA->0 (defaut PC=0xfa7e, override
         * CALYPSO_TRACE_LDU_PC=0xNNNN). Dump l'EA lue + valeur + indirect +
         * AR/DP pour nommer la case = 0 (pointeur table non init / EA fausse). */
        {
            static int ldu_trace_pc = -1;
            if (ldu_trace_pc < 0) {
                const char *e = getenv("CALYPSO_TRACE_LDU_PC");
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
         * Previous handler always shifted by 16 — wrong for plain ADD/SUB.
         */
        addr = resolve_smem(s, op, &ind);
        int dst = (op >> 8) & 1;
        int sub = (op >> 9) & 0x07;  /* 0..7 */
        uint16_t val = data_read(s, addr);
        int64_t v;
        bool is_sub = (sub & 0x4) != 0;
        bool is_unsigned = (sub == 1 || sub == 5);  /* ADDS / SUBS */
        bool ts_shift = (sub == 2 || sub == 6);     /* ,TS variants */
        /* [2026-07-28] sub 3 = ADDC (0x0600) et sub 7 = SUBB (0x0E00) : ils tombaient
         * dans le traitement ADD/SUB generique, donc SANS la retenue. SPRU172C :
         *   « ADDC Smem, src : src = src + Smem + C »
         *   « SUBB Smem, src : src = src - Smem - C »
         * binutils : addc 0x0600/0xFE00, subb 0x0E00/0xFE00, 1 mot chacun.
         * NB : on suit la lettre du manuel (- C). Certaines implementations de SUBB
         * soustraient l emprunt (~C) ; si une mesure le montrait, corriger ICI. */
        bool with_carry = (sub == 3 || sub == 7);
        v = is_unsigned ? (uint16_t)val
                        : ((s->st1 & ST1_SXM) ? (int16_t)val : (uint16_t)val);
        if (ts_shift) {
            int8_t ts = (int8_t)((s->t & 0x3F) | ((s->t & 0x20) ? 0xC0 : 0));
            v = (ts >= 0) ? (v << ts) : (v >> -ts);
        }
        {
            int64_t c = with_carry ? ((s->st0 & ST0_C) ? 1 : 0) : 0;
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
        {   /* [2026-08-04] handlers MAC/bit rendus atteignables — AVANT tout
             * resolve_smem, chaque handler fait le sien. */
            int _h = c54x_mac_bit_family(s, op, consumed);
            if (_h >= 0) return _h;
        }
        /* 3xxx: MAC / MAS — mais d'ABORD SQURA (§4-A, fix 2026-06-23). */
        addr = resolve_smem(s, op, &ind);
        {
            uint16_t val = data_read(s, addr);
            /* SQURA Smem, src (0x38/0x39, mask 0xFE00) : src = src + Smem*Smem.
             * Per tic54x_hi8_map.md l.46 (0x3800/0xFE00, bit8=src A=0x38/B=0x39).
             * Le case 0x3 « blind-MAC » exécutait SQURA comme `acc += T*Smem` →
             * énergie (somme de carrés) calculée avec le mauvais opérande/signe →
             * A reste ≤0 à PROM0 0x76ff/0x7700 → RCD LEQ@0x75e8 prend la sortie
             * anticipée → saute le corps qui pousse ST1 → POPM ST1@0x7706 sur-pope
             * (1er pop orphelin insn 146) → DP=0x124 → handler garbage → SP collapse.
             * SQURA accumule un CARRÉ (contribution ≥0) → A>0 → RCD ne prend pas. */
            /* ═════════════════════════════════════════════════════════════════
             * [2026-08-04] FIX_BITT_CASE3 — BITT etait DANS LA MAUVAISE BRANCHE.
             *
             * Le handler existait deja (`(op & 0xFF00) == 0x3400`), correctement
             * ecrit, mais place dans `case 0xF:` du switch(hi4). Or `bitt` est
             * `0x34xx`, donc hi4 = 3 : il etait INATTEIGNABLE PAR CONSTRUCTION, et
             * `0x348e` tombait ici, dans le MAC aveugle du `case 0x3`.
             *
             * TI SPRU172C : `BITT Smem` -> `TC = Smem(15 - T(3-0))`, encodage
             * `0011 0100 IAAAAAAA`, « Status Bits: Affects TC » — ET RIEN D'AUTRE.
             * Le MAC etait donc faux deux fois : TC jamais pose (il restait
             * PERIME), et l'accumulateur A CORROMPU par une accumulation qui n'a
             * pas lieu d'etre.
             *
             * CONSEQUENCE MESUREE (04/08) : l'assembleur de bits 0x9ab8..0x9ad2
             * (`bitt *AR6-` -> `roltc A` -> `stl *AR2-,A`) empaquetait de la
             * bouillie dans 0x2c3c..0x2c47, que le `mvdd` de 0x9723 publiait dans
             * a_cd[3..]. Sonde ROLTC-WATCH : tc_entrant = 1445/5000 (~29 %), du
             * bruit, pas un test de bit.
             * ⚠️ A etait NON NUL et d'allure plausible (0x33c7eed1bb...) — c'etait
             * de la bouillie MAC. Une valeur non nulle n'est pas une valeur juste.
             *
             * PRECEDENT SUIVI : SQURA (juste en dessous) avait deja ete extrait de
             * ce meme MAC aveugle en juin, pour la meme raison.
             *
             * ⚠️ Le commentaire de l'ancien handler (~l.7634) decrit exactement ce
             * bug et le dit CORRIGE. Il l'etait sur le papier ; le code n'etait pas
             * atteint. Un commentaire « corrige » ne prouve rien sans preuve
             * d'atteignabilite.
             *
             * Gate d'echappement `CALYPSO_FIX_BITT_CASE3=0` : restaure le MAC
             * aveugle, pour isoler une regression sans toucher au code.
             * ═════════════════════════════════════════════════════════════════ */
            if ((op & 0xFF00) == 0x3400) {
                static int fb = -1;
                if (fb < 0) {
                    fb = calypso_gate("CALYPSO_FIX_BITT_CASE3", 1);
                    fprintf(stderr, "[c54x] FIX_BITT_CASE3 %s "
                            "(CALYPSO_FIX_BITT_CASE3=%d) — bitt %s (TI SPRU172C)\n",
                            fb ? "ACTIF" : "inactif", fb,
                            fb ? "pose TC et ne touche PAS l'accumulateur"
                               : "retombe dans le MAC aveugle (comportement d'avant)");
                }
                if (fb) {
                    int bitt_idx = 15 - (s->t & 0xF);
                    if ((val >> bitt_idx) & 1) s->st0 |= ST0_TC;
                    else                       s->st0 &= ~ST0_TC;
                    return consumed + s->lk_used;
                }
            }

            if ((op & 0xFE00) == 0x3800) {
                int64_t sq = (int64_t)(int16_t)val * (int64_t)(int16_t)val;
                if (s->st1 & ST1_FRCT) sq <<= 1;
                int sdst = (op >> 8) & 1;
                if (sdst) s->b = sext40(s->b + sq);
                else      s->a = sext40(s->a + sq);
                /* [2026-07-28] SPRU172C : « SQURA Smem, src : src = src + Smem * Smem,
                 * T = Smem ». L ecriture de T manquait : toute instruction suivante qui
                 * utilise T (MAC, LD Smem,TS, ...) travaillait sur une valeur perimee. */
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
        {   /* [2026-08-04] handlers MAC/bit rendus atteignables — AVANT tout
             * resolve_smem, chaque handler fait le sien. */
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
            /* [2026-08-23] MPYU et SQUR etaient PERMUTES, et MPYR absent.
             * binutils :  mpyr 0x2200/0xFE00 | mpyu 0x2400/0xFE00 | squr 0x2600/0xFE00
             * Le champ `sub = (op >> 8) & 0xF` vaut donc 2/3 pour MPYR, 4/5 pour
             * MPYU, 6/7 pour SQUR. L ancien code mettait SQUR en 4/5 (donc sur
             * MPYU) et laissait 2/3 et 6/7 tomber dans le `default:` = MAS.
             * Consequences sur T, c est ce qui compte ici :
             *   MPYU (SPRU172C l.1059) : dst = uns(T)*uns(Smem), T INCHANGE
             *        -> l ancien code y ecrivait `s->t = val` : ECRITURE PARASITE
             *   SQUR (l.1061) : dst = Smem*Smem ET T = Smem
             *        -> l ancien code n ecrivait PAS T
             *   MPYR (l.1047) : dst = rnd(T*Smem), AFFECTATION, T inchange
             *        -> l ancien code ACCUMULAIT en soustrayant (MAS)
             * MPYR a deux sites en 0x8166 et 0x816c, dans la routine meme qui
             * charge T pour le MPY de 0x81e4.
             * Gate CALYPSO_ISA_MPY_FAM (defaut 1). */
            case 0x2: case 0x3: /* MPYR Smem, dst : dst = rnd(T * Smem) */
                if (c54x_mpy_fam()) {
                    product = (int64_t)(int16_t)s->t * (int64_t)(int16_t)val;
                    if (s->st1 & ST1_FRCT) product <<= 1;
                    product += 0x8000;                 /* arrondi */
                    if (sub & 1) s->b = sext40(product);
                    else         s->a = sext40(product);
                    return consumed + s->lk_used;      /* T INCHANGE */
                }
                goto mac_fam_defaut;
            case 0x4: case 0x5: /* MPYU Smem, dst : dst = uns(T) * uns(Smem) */
                if (c54x_mpy_fam()) {
                    product = (int64_t)(uint16_t)s->t * (int64_t)(uint16_t)val;
                    if (s->st1 & ST1_FRCT) product <<= 1;
                    if (sub & 1) s->b = sext40(product);
                    else         s->a = sext40(product);
                    return consumed + s->lk_used;      /* T INCHANGE */
                }
                product = (int64_t)(int16_t)val * (int64_t)(int16_t)val;
                if (s->st1 & ST1_FRCT) product <<= 1;
                s->t = val;
                if (sub & 1) s->b = sext40(product);
                else         s->a = sext40(product);
                return consumed + s->lk_used;
            case 0x6: case 0x7: /* SQUR Smem, dst : dst = Smem*Smem ; T = Smem */
                if (c54x_mpy_fam()) {
                    product = (int64_t)(int16_t)val * (int64_t)(int16_t)val;
                    if (s->st1 & ST1_FRCT) product <<= 1;
                    s->t = val;
                    if (sub & 1) s->b = sext40(product);
                    else         s->a = sext40(product);
                    return consumed + s->lk_used;
                }
                goto mac_fam_defaut;
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
            mac_fam_defaut:
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
                /* SUB Smem << 16, src, dst — sub of shifted Smem from acc */
                addr = resolve_smem(s, op, &ind);
                int64_t val = (int64_t)(int16_t)data_read(s, addr) << 16;
                *acc_dst = sext40(*acc_dst - val);
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
                /* [2026-07-28] RPT Smem — charge le compteur de repetition SIMPLE (RC),
                 * PAS le compteur de bloc (BRC). SPRU172C : « RPT Smem : Repeat single,
                 * RC = Smem ». binutils : rpt 0x4700/0xFF00, 1 mot.
                 * L ancien code ecrivait s->brc : le RPT n avait donc aucun effet sur la
                 * repetition (rpt_count restait a sa valeur precedente) et BRC etait
                 * corrompu au passage. On aligne sur le handler RPT #k8u (0xEC00) qui est
                 * correct : avancer le PC et rendre 0 pour que le dispatcher re-execute
                 * l instruction SUIVANTE, pas le RPT lui-meme. */
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
        /* 5xxx: shifts — SFTA, SFTL, various forms.
         * NOTE: 0x56xx/0x57xx are SFTL/SFTA with Smem (1-word), NOT MVPD.
         * MVPD is at 0x8Cxx (hi8=0x8C). The old 0x56 MVPD decode was wrong
         * and caused writes to MMR_SP via resolve_smem, corrupting the stack. */
        {
            /* === Dual long-word DADST/DSADT, Lmem,dst (1 word) — fix revival
             * dsp (2026-06-22). Encodage SPRU172C : 0101 101D = DADST (0x5A/5B),
             * 0101 111D = DSADT (0x5E/5F) ; D(bit8)=dst (0=A,1=B) ; bits[7:0]=Smem
             * (Lmem). Sans ce handler ces opcodes tombaient dans le bloc SFTA/SFTL
             * ci-dessous → corrélateur FCCH aplati en "dst>>=ASM", d_fb_det=0
             * (sonde SHADOW-DADST : shiftLike=1, walked=0). Sémantique (vérifiée
             * sur les exemples chiffrés SPRU172C) :
             *   C16=1 (dual-16, non saturé) :
             *     DADST: dst(39-16)=Lmem.hi+T ; dst(15-0)=Lmem.lo-T
             *     DSADT: dst(39-16)=Lmem.hi-T ; dst(15-0)=Lmem.lo+T
             *   C16=0 (double-precision, SXM) :
             *     DADST: dst=Lmem+((T<<16)|T) ; DSADT: dst=Lmem-((T<<16)|T)
             * Lmem post-mod = ±2 (long-operand, via resolve_lmem). Le mode C16
             * réel est lu à l'exécution (ST1.C16, suivi par SSBX/RSBX C16). */
            uint8_t dl_hi = (op >> 8) & 0xFF;
            if (dl_hi >= 0x50 && dl_hi <= 0x5F) {
                /* === Famille dual long-word COMPLÈTE (SPRU172C, sweep §4-E 2026-06-22) :
                 *   0x50-53 DADD(00SD) 0x54-55 DSUB(010S) 0x56-57 DLD(011D)
                 *   0x58-59 DRSUB(100S) 0x5A-5B DADST(101D) 0x5C-5D DSUBT(110D)
                 *   0x5E-5F DSADT(111D). Lmem 32-bit via resolve_lmem (post-mod ±2),
                 * branche sur ST1.C16. Vérifié bit-à-bit vs exemples chiffrés SPRU172C
                 * (DADD/DADST/DSADT). Avant : 0x50-59/5C-5D tombaient en SFTA/SFTL. */
                uint16_t laddr  = resolve_lmem(s, op);
                uint16_t lhi    = data_read(s, laddr);
                uint16_t llo    = data_read(s, (uint16_t)(laddr + 1));
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
                        int sgn_hi = (dl_hi <= 0x5B) ? +1 : -1;   /* DADST hi+T ; DSUBT/DSADT hi-T */
                        int sgn_lo = (dl_hi <= 0x5D) ? -1 : +1;   /* DADST/DSUBT lo-T ; DSADT lo+T */
                        int32_t hi = (int32_t)lhi16 + sgn_hi * (int32_t)t16;
                        int32_t lo = (int32_t)llo16 + sgn_lo * (int32_t)t16;
                        r = ((int64_t)hi << 16) | ((uint32_t)lo & 0xFFFF);
                    } else {
                        uint32_t tt32 = ((uint32_t)(uint16_t)t16 << 16) | (uint16_t)t16;
                        int64_t tt40 = sxm ? (int64_t)(int32_t)tt32 : (int64_t)(uint32_t)tt32;
                        r = (dl_hi <= 0x5B) ? (lmem40 + tt40) : (lmem40 - tt40); /* DADST add ; else sub */
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

        /* ---- Dual-operand MAC Xmem, Ymem, dst (1-word) ----
         * 0x90: MAC Xmem,Ymem,A   0x92: MAC Xmem,Ymem,B
         * 0x91: MACR Xmem,Ymem,A  0x93: MACR Xmem,Ymem,B
         * Same encoding as 0xA4 family: OOOO OOOD XXXX YYYY */
        /* [2026-09-17] FIX_ADDSUB_XSHFT — binutils/SPRU172C 4-4, 4-187 : 0x9000
         * (mask FE00) = ADD Xmem,SHFT,src : src += Xmem << SHFT ; 0x9200 = SUB
         * Xmem,SHFT,src : src -= Xmem << SHFT (bit 8 = src, SHFT = bits 3:0, SXM).
         * Le bloc ci-dessous les executait en MAC Xmem,Ymem. Site SB : 0x988e `9086`. */
        {
            static int fix_axs = -1;
            if (fix_axs < 0) fix_axs = calypso_gate("CALYPSO_FIX_ADDSUB_XSHFT", 1);
            if (fix_axs && (op & 0xFC00) == 0x9000) {
                uint16_t xa = resolve_xmem(s, op);
                uint16_t xv = data_read(s, xa);
                int shft = op & 0xF;
                int64_t v = ((s->st1 & ST1_SXM) ? (int64_t)(int16_t)xv : (int64_t)xv) << shft;
                int64_t *srcp = (op & 0x0100) ? &s->b : &s->a;
                *srcp = sext40((op & 0x0200) ? (*srcp - v) : (*srcp + v));
                return consumed + s->lk_used;
            }
        }
        if (hi8 == 0x90 || hi8 == 0x91 || hi8 == 0x92 || hi8 == 0x93) {
            /* FIX 2026-06-22 (sweep) : décodage Xmem/Ymem 2-bit SPRU131G T.5-6/5-8
             * (Xmod[7:6] Xar[5:4] Ymod[3:2] Yar[1:0], AR=field+2, mod 1=*AR- 2=*AR+
             * 3=*AR+0%) au lieu du raw 3-bit/1-bit. */
            int xar_m  = ((op >> 4) & 0x03) + 2;
            int yar_m  = (op & 0x03) + 2;
            int xmod_m = (op >> 6) & 0x03;
            int ymod_m = (op >> 2) & 0x03;
            uint16_t xval_m = data_read(s, s->ar[xar_m]);
            uint16_t yval_m = data_read(s, s->ar[yar_m]);
            switch (xmod_m) { case 1: s->ar[xar_m]--; break; case 2: s->ar[xar_m]++; break;
                case 3: s->ar[xar_m] = c54x_circ_ref(s->ar[xar_m], +(int16_t)s->ar[0], s->bk); break; }
            switch (ymod_m) { case 1: s->ar[yar_m]--; break; case 2: s->ar[yar_m]++; break;
                case 3: s->ar[yar_m] = c54x_circ_ref(s->ar[yar_m], +(int16_t)s->ar[0], s->bk); break; }
            int64_t prod_m = (int64_t)(int16_t)s->t * (int64_t)(int16_t)xval_m;
            if (s->st1 & ST1_FRCT) prod_m <<= 1;
            if (hi8 & 0x01) prod_m += 0x8000; /* round */
            int dst_m = (hi8 & 0x02) ? 1 : 0;
            if (dst_m) s->b = sext40(s->b + prod_m);
            else       s->a = sext40(s->a + prod_m);
            s->t = yval_m;
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

        /* AUDIT FIX 2026-05-08 night : STL ↔ STH swap.
         * Per binutils tic54x-opc.c :
         *   { "stl", 1,3,3, 0x9800, 0xFE00, {OP_SRC1,OP_SHFT,OP_Xmem} }
         *   { "sth", 1,3,3, 0x9A00, 0xFE00, {OP_SRC1,OP_SHFT,OP_Xmem} }
         * Old decoder claimed 0x98/99=STH and 0x9A/9B=STL — exactly inverted.
         * Effect: every STL/STH-with-shift in firmware wrote the WRONG half
         * of the accumulator. Hot pattern in DSP code (post-MAC scaling),
         * so this corrupted ~half of all data writes from compute paths.
         * Shift application is intentionally simplified (no SHFT decode)
         * matching prior-art handlers — Tier B will add proper 4-bit shift
         * decode from low nibble. Mirror swap : write low for 0x98/99,
         * write high for 0x9A/9B, src bit 8 selects A/B. */
        if (hi8 == 0x98 || hi8 == 0x99) {
            /* STL src, SHFT, Xmem — store LOW (acc&0xFFFF).
             * FIX 2026-05-23 : Xmem operand decoded via resolve_xmem (per
             * binutils OP_Xmem), not resolve_smem. The latter mis-mapped
             * low byte 0x00-0x1F with bit 7=0 to MMR space, clobbering SP/
             * IMR/IFR. Empirical proof : PC=0x8a46 op=0x9918 stomp SP→0
             * captured by existing SP-CATASTROPHE probe. */
            addr = resolve_xmem(s, op);
            int src = hi8 & 1;
            int64_t acc = src ? s->b : s->a;
            data_write(s, addr, (uint16_t)(acc & 0xFFFF));
            return consumed + s->lk_used;
        }
        if (hi8 == 0x9A || hi8 == 0x9B) {
            /* STH src, SHFT, Xmem — store HIGH (acc>>16).
             * FIX 2026-05-23 : same as STL above — Xmem decoded via
             * resolve_xmem (per binutils), not resolve_smem. See STL block. */
            addr = resolve_xmem(s, op);
            int src = hi8 & 1;
            int64_t acc = src ? s->b : s->a;
            data_write(s, addr, (uint16_t)((acc >> 16) & 0xFFFF));
            return consumed + s->lk_used;
        }

        /* 0x9C-0x9F range: SACCD/SRCCD/STRCD — conditional stores */

        /* SACCD src, Xmem, cond — Conditional accumulator store
         * Encoding: 1001 11SD XXXX COND per SPRU172C p.4-152 */
        /* [2026-09-17] FIX_XCCD — SPRU172C 4-152/4-165/4-186 : 0x9C = STRCD Xmem,cond
         * (Xmem = T), 0x9D = SRCCD Xmem,cond (Xmem = BRC), 0x9E/0x9F = SACCD src,
         * Xmem,cond avec src = bit 8 (pas bit 9) ; cond sur 4 bits : bit 3 =
         * accumulateur teste (0 = A, 1 = B), bits 2:0 = 101 EQ, 100 NEQ, 110 GT,
         * 010 GEQ, 011 LT, 111 LEQ. L'ancien bloc prenait tout pour SACCD, testait
         * src au lieu de l'accumulateur du cond, et lisait le code a l'envers.
         * Sites SB : 0x847e/0x849a `9e1e` (SACCD A), 0x831f `9f06` (SACCD B). */
        {
            static int fix_xccd = -1;
            if (fix_xccd < 0) fix_xccd = calypso_gate("CALYPSO_FIX_XCCD", 1);
            if (fix_xccd && (op & 0xFC00) == 0x9C00) {
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
        }
        if ((op & 0xFC00) == 0x9C00) {
            int src_s = (op >> 9) & 1;
            int64_t acc = src_s ? s->b : s->a;
            /* FIX 2026-06-02 (ROOT CAUSE FB-det) : opérande Xmem décodé via
             * resolve_xmem (xar=((op>>4)&3)+2 = AR2-5, + xmod post-modify),
             * exactement comme STL/STH 0x98-0x9B. L'ancien `(op>>4)&0x07` lisait
             * le MAUVAIS AR (AR1 au lieu de AR3 pour op=0x9e9b) et `(op>>7)&1` un
             * faux sens → AR3 jamais incrémenté → la boucle de recherche de pic
             * FCCH (@0x8576 RPTB) relisait sample[0] 15× → corrélation figée,
             * d_fb_det garbage, rxlev plancher, FBSB jamais fermé. resolve_xmem
             * applique le post-incrément ; ne PAS re-modifier en fin de handler. */
            uint16_t xaddr = resolve_xmem(s, op);
            int cond = op & 0x0F;
            /* Evaluate condition */
            int take = 0;
            switch (cond) {
            case 0x0: take = (acc == 0); break;    /* EQ */
            case 0x1: take = (acc != 0); break;    /* NEQ */
            case 0x2: take = (acc > 0); break;     /* GT */
            case 0x3: take = (acc < 0); break;     /* LT */
            case 0x4: take = (acc >= 0); break;    /* GEQ */
            case 0x5: take = (acc == 0); break;    /* AEQ */
            case 0x6: take = (acc > 0); break;     /* AGT */
            case 0x7: take = (acc <= 0); break;    /* LEQ/ALEQ */
            default: take = 0; break;
            }
            int asm_val = asm_shift(s);
            if (take) {
                /* Store shifted accumulator high part */
                int64_t shifted = acc << (asm_val > 0 ? asm_val : 0);
                if (asm_val < 0) shifted = acc >> (-asm_val);
                uint16_t val = (uint16_t)((shifted >> 16) & 0xFFFF);
                data_write(s, xaddr, val);
            } else {
                /* Read and write back (no change) */
                uint16_t val = data_read(s, xaddr);
                data_write(s, xaddr, val);
            }
            /* post-modify Xmem déjà appliqué par resolve_xmem (cf FIX ci-dessus) */
            return consumed + s->lk_used;
        }
        /* POPM MMR — pop top-of-stack into MMR (1-word).
         * Per tic54x-opc.c: { "popm", 0x8A00, 0xFF00, {OP_MMR} }.
         * Per SPRU172C section 4 : value at SP popped to MMR, SP++.
         *
         * Bug fix 2026-05-08 : 0x8Axx était précédemment mal décodé en
         * MVDK Smem,dmad (qui est en réalité 0x7100 mask 0xFF00). Le
         * pattern PSHM/POPM symétrique du firmware (e.g. PROM0 0x7013-0x7023
         * sauve/restaure 6 MMRs autour d'un CALA) ne fonctionnait jamais
         * post-CALA → ST1 jamais restauré → INTM=1 dwell perpétuel
         * → IRQ vectoring bloqué → DSP wait stuck → L1 mort.
         * Le case MVDK ci-dessous devient dead code mais est laissé pour
         * référence historique. */
        if ((op & 0xFF00) == 0x8A00) {
            uint16_t mmr = op & 0x7F;
            uint16_t val = data_read(s, s->sp);
            s->sp = (s->sp + 1) & 0xFFFF;
            /* POPM-ST1 probe (CALYPSO_DEBUG=POPM-ST1) : ST1 == MMR 0x07.
             * Discrimine (a) POPM ST1 jamais exécuté vs (b) exécuté mais
             * la valeur poppée a déjà INTM=1 → restaure 1, ne clear jamais.
             * Silent par défaut. */
            if (mmr == 0x07) {
                C54_DBG("POPM-ST1",
                        "POPM ST1 val=0x%04x INTM_bit=%u PC=0x%04x SP=0x%04x insn=%u",
                        val, !!(val & ST1_INTM), s->pc, s->sp, s->insn_count);
            }
            data_write(s, mmr, val);
            return consumed + s->lk_used;
        }
        /* OBSOLETE — superseded by POPM above. The 0x8Axx range belongs to
         * POPM per tic54x-opc.c, not MVDK (which is 0x7100 mask 0xFF00).
         * Kept commented for one revision so any caller depending on the
         * old (incorrect) behaviour is forced to be re-examined. */
        /* 0x88xx-0x89xx: STLM src, MMR  (1-word!)
         * Per tic54x-opc.c: { "stlm", 1,2,2, 0x8800, 0xFE00, ... }
         *   bits 9-15 = fixed (0x44)
         *   bit 8     = src (0 = A, 1 = B)
         *   bits 0-6  = MMR address (0x00..0x7F)
         *
         * Critical for the DSP bootloader at PROM0 0xb42d (`STLM B, AR1`):
         * if decoded as 2-word MVDM the emulator eats the next opcode
         * (0xb42e = 0xf84c, a BC), then jumps into 0xb431 (MACR family)
         * with an uninitialised T register, producing A=0x10 — which
         * the immediately-following BACC A at 0xb430 then uses as the
         * jump target, dropping the DSP into the boot-stub NOPs at
         * PC=0x0010 instead of continuing the bootloader handshake. */
        if (hi8 == 0x88 || hi8 == 0x89) {
            int src = (op >> 8) & 1;  /* 0 = A, 1 = B */
            int mmr = op & 0x7F;
            uint16_t val = src ? (uint16_t)(s->b & 0xFFFF)
                               : (uint16_t)(s->a & 0xFFFF);
            data_write(s, (uint16_t)mmr, val);  /* MMRs alias addr 0x00..0x1F */
            return consumed + s->lk_used;
        }
        /* [2026-07-23] 0x8Bxx: POPD Smem — pop top-of-stack into data memory.
         * Symetrique de PSHD (0x4B) : PSHM/POPM = 0x4A/0x8A ; PSHD/POPD = 0x4B/0x8B.
         * ETAIT MANQUANT -> tombait en NOP 1-mot. Bug reel : l'overlay handler frame
         * 0x013b fait `POPD *(0x3fcd)` (depile le retour du CALL 0x013b), PSHM x24,
         * `PSHD *(0x3fcd)` (repush le retour), RET. Sans POPD : retour enterre sous
         * les saves, data[0x3fcd]=0, RET->0 -> DERAIL-ZERO from=0x0157 (go-live cycle).
         * resolve_smem pose lk_used pour le mode abs (0xf8) -> 2-mots correct. */
        if (hi8 == 0x8B) {
            addr = resolve_smem(s, op, &ind);
            uint16_t val = data_read(s, s->sp);
            s->sp = (s->sp + 1) & 0xFFFF;
            data_write(s, addr, val);
            return consumed + s->lk_used;
        }
        if (hi8 == 0x80) {
            /* AUDIT FIX 2026-05-08 night : was stubbed NOP because old
             * decoder claimed MVDD (2-word, wrong). Per binutils tic54x-opc.c :
             *   { "stl", 1,2,2, 0x8000, 0xFE00, {OP_SRC1,OP_Smem}, 0, REST }
             * 0x80xx/0x81xx = STL src, Smem (1-word, no shift). bit 8 = src.
             * Range 0x8000-0x80FF = STL A, Smem (since bit 8 = 0 here).
             * Stubbing this silently dropped every STL A in the firmware ;
             * variables that should have been written to DARAM kept stale
             * values (junk-state cascade). Mirror of the existing 0x82
             * STH-with-shift handler but no shift here. */
            addr = resolve_smem(s, op, &ind);
            data_write(s, addr, (uint16_t)(s->a & 0xFFFF));
            return consumed + s->lk_used;
        }
        if (hi8 == 0x8C) {
            /* AUDIT FIX 2026-05-08 night : was MVPD pmad,Smem (2 mots,
             * prog→data move). Per binutils tic54x-opc.c :
             *   { "mvpd", 2,2,2, 0x7C00, 0xFF00, {OP_pmad,OP_Smem}, 0, REST }
             *   { "st",   1,2,2, 0x8C00, 0xFF00, {OP_T,OP_Smem},    0, REST }
             * Real MVPD is at 0x7C — the 0x8C handler should be ST T, Smem
             * (1 mot, store T register to data memory). Run-trace confirms
             * 0 MVPD hits with the old handler, meaning firmware did not
             * issue any 0x7Cxx → our wrong 0x8C MVPD was never triggered
             * for legitimate MVPD anyway (PROM0 OVLY happens via DSP
             * bootloader, not via 0x7C MVPD instruction). Switching to
             * ST T,Smem is safe and unblocks the legitimate ST T pattern
             * used after MAC for T persistence. Old MVPD-LOG instrumentation
             * removed — was dead-code in current run. */
            addr = resolve_smem(s, op, &ind);
            data_write(s, addr, s->t);
            return consumed + s->lk_used;
        }
        /* 0x8E/0x8F : CMPS src, Smem — Compare, Select & Store Maximum
         * (SPRU172C p.4-35). Opcode 1000 111 S I AAAAAAA = 0x8E00/0xFE00,
         * bit8 = src (0=A, 1=B). 1 MOT (+1 si long-offset/absolu → lk_used).
         *   if src(31–16) > src(15–0):  src(31–16)→Smem ; TRN<<=1,TRN(0)=0 ; TC=0
         *   else:                       src(15–0)→Smem  ; TRN<<=1,TRN(0)=1 ; TC=1
         * = compare les 2 moitiés 16-bit 2s-comp de l'accu, stocke la MAX, TRN/TC
         * tracent le gagnant. Cœur de la recherche de pic FCCH (Viterbi).
         *
         * FIX 2026-06-02 (audit décodeur DECODE-AUDIT) : 0x8E était décodé MVDP
         * 2-mots et 0x8F PORTR 2-mots → chaque paire CMPS A/CMPS B consécutive
         * (op=8e94 op2=8f93 dans la zone FB-det 0xa0xx) voyait le 2e CMPS BOUFFÉ
         * comme phantom-pmad → désync corrélateur, d_fb_det jamais armé. SÛR :
         * l'assembleur TI encode MVDP=0x7D, PORTR=0x74 — jamais 0x8E/0x8F ; et
         * l'I/Q arrive par DMA DARAM (data[0x2a00]), pas par opcode PORTR (audit
         * 0x8F=0 exécution). Le vieux handler 0x8F=PORTR est neutralisé plus bas. */
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
        /* SUPERSEDED 2026-06-02 : 0x8F = CMPS B (traité par le handler CMPS
         * 0x8E/0x8F ci-dessus, qui return avant d'arriver ici). Ce bloc PORTR
         * est ISA-faux (vrai PORTR=0x74) et jamais atteint (audit 0x8F=0 exec ;
         * I/Q via DMA DARAM). Gardé en dead-code (if(0)) pour réf si on relocalise
         * PORTR vers 0x74 un jour. */
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
        /* REVERTED 2026-05-15 nuit : handlers 0x72/0x73 RETIRÉS.
         * Voir doc/REVERT_MVMD_KNOWLEDGE.md et premier emplacement revert
         * ci-dessus (avant le bloc `(op & 0xF800) == 0x7000`). 0x86/0x87
         * restent comme avant (DUPLICATE MVDM/MVMD au lieu de STH A/B ASM
         * vrai — non swappés). */

        /* 0x86/0x87 : STH src, ASM, Smem — store HIGH (acc>>16) shifted by ASM,
         * to Smem (1-WORD). bit8 = src (0=A, 1=B). Per tic54x_hi8_map.md L95 :
         *   { "sth", 0x8600, 0xFE00, ASM variant }  ← 1 mot, mask 0xFE00.
         *
         * FIX 2026-06-02 (bug #3, ROOT CAUSE AR3-zero) : l'ancien décode était
         * MVDM dmad,MMR (0x86) / MVMD MMR,dmad (0x87) en 2 MOTS — faux sur deux
         * axes : (1) longueur (2 au lieu de 1) → consommait l'opcode suivant
         * → désync du flux de décode en cascade (= SP→0xcade observé) ; (2) ne
         * touchait jamais l'AR du Smem → AR3 figé/0 dans la boucle corrélateur
         * FB (IQ-READ @0x7e6f montrait AR3=0000 sur op=0x8693 @0x7e71 = STH A,
         * ASM, *AR3+ : AR3 doit post-incrémenter pour balayer le buffer I/Q
         * 0x2a00+). Mirror EXACT du handler 0x84 (STL A,ASM,Smem) déjà validé,
         * mais store HIGH word au lieu de LOW. resolve_smem applique le
         * post-incrément du Smem indirect → ne PAS re-modifier l'AR ici.
         *
         * SÛR vs le revert 0x72/0x73 (REVERT_MVMD_KNOWLEDGE.md) : ORTHOGONAL.
         * L'assembleur TI encode MVDM=0x72, MVMD=0x73 — JAMAIS à 0x86/0x87.
         * Donc aucun 0x86xx/0x87xx de la ROM n'est une vraie MVDM/MVMD : c'est
         * toujours un STH. Le side-effect dont dépend le firmware est sur 0x73
         * (site 0x8208 op=0x7317), inchangé par ce fix. */
        if (hi8 == 0x86 || hi8 == 0x87) {
            addr = resolve_smem(s, op, &ind);
            int shift = asm_shift(s);
            int src = hi8 & 1;            /* 0x86→A, 0x87→B */
            int64_t v = src ? s->b : s->a;
            if (shift >= 0) v <<= shift; else v >>= (-shift);
            data_write(s, addr, (uint16_t)((v >> 16) & 0xFFFF));  /* STH = high word */
            return consumed + s->lk_used;
        }
        /* AUDIT FIX 2026-05-15 fin journée : 0x81/0x82/0x83 mal décodés.
         * Per tic54x-opc.c + SPRU172C :
         *   stl 0x8000 / 0xFE00 → 0x80..0x81 STL src,Smem (no shift)
         *   sth 0x8200 / 0xFE00 → 0x82..0x83 STH src,Smem (no shift)
         *   stl 0x8400 / 0xFE00 → 0x84..0x85 STL src,ASM,Smem (with shift) [FAIT]
         *   sth 0x8600 / 0xFE00 → 0x86..0x87 STH src,ASM,Smem (with shift) [FAIT]
         * [2026-07-28] les deux variantes ASM sont IMPLEMENTEES (handlers hi8==0x84
         * et hi8==0x86/0x87 ci-dessus, tous deux via asm_shift()), et asm_shift()
         * est conforme au manuel (ASM = ST1[4:0] signe, -16 <= ASM <= 15). Le
         * "[TODO]" precedent etait perime et a coute une fausse piste en remontant
         * la sortie du demod (0x8694 = STH A,ASM,*AR4+ ecrit data[0x2a00]) : le
         * decalage EST applique. Le vrai bug de ce chemin etait ailleurs — les
         * opcodes logiques 0x1800/1A00/1C00/1E00 (AND/OR/XOR/SUBC) decodes comme
         * un LD, cf. le case 0x1 plus haut.
         * bit 8 = src (0=A, 1=B). Old code applied asm_shift incorrectly
         * to 0x81/0x82 (basic variants — no shift) AND used s->a for 0x81
         * (should be s->b). Le bug causait toutes les STL B / STH * vers
         * adressing indirect *ARn à écrire la mauvaise valeur ; en particulier
         * d_burst_d (DSP word 0x0829/0x083D) et d_task_d (0x0828/0x083C) du
         * NDB CCCH demod ARM bail dans prim_rx_nb.c::l1s_nb_resp avec
         * "EMPTY" et "BURST ID 33414!=N" sous synth=1 banc d'essai. */

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
        /* 8Bxx: MVDK with long address */
        if (hi8 == 0x8B) {
            /* STUB-NOP : tic54x dit 0x8B = POPD Smem (1-word).
             * Ancienne classification qemu = MVDK long-addr 2-word (incorrect).
             * Voir doc/opcodes/tic54x_hi8_map.md. Neutralisé. */
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
        /* AUDIT FIX 2026-05-15 fin journée : 0x83 misclassifié comme WRITA
         * (qui est en réalité 0x7F per tic54x-opc.c). Vrai 0x83 = STH B, Smem.
         * Et 0x84 misclassifié comme READA (vrai = 0x7E). Vrai 0x84 = STL A,
         * ASM, Smem (with shift). 0x85..0x87 idem TODO. */
        /* 0x83xx: STH B, Smem  (src=B, no shift) */
        if (hi8 == 0x83) {
            addr = resolve_smem(s, op, &ind);
            data_write(s, addr, (uint16_t)((s->b >> 16) & 0xFFFF));
            return consumed + s->lk_used;
        }
        /* 0x84xx: STL A, ASM, Smem (src=A, with ASM shift) — TODO compléter
         * variantes 0x85 (STL B), 0x86 (STH A), 0x87 (STH B) with ASM shift.
         * Pour l'instant fix uniquement 0x84 vers la sémantique tic54x correcte. */
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
            /* FIX revival dsp 2026-06-22 : décodage Xmem/Ymem 2-bit (SPRU131G
             * Table 5-6/5-8, identique à resolve_xmem et au handler D0-D9) au
             * lieu du raw 3-bit. L'ancien (op>>4)&7 / op&7 + post-mod 1-bit lisait
             * les MAUVAIS AR : ex op=0xb4f5 @PC=0xf170 (corrélateur TOA steady-
             * state) donnait Xmem=AR7/Ymem=AR5 (hors-buffer) au lieu de Xmem=AR5
             * (*AR5+0% circ) / Ymem=AR3 (*AR3-) → Ymem lu hors-buffer=0 → T=0 →
             * MAC=0 → A garbage → a_sync_demod[D_TOA]=garbage (0xc3f0). Format :
             * Xmod[7:6] Xar[5:4] Ymod[3:2] Yar[1:0] ; AR = field+2 (AR2..AR5) ;
             * mod 0=*AR 1=*AR- 2=*AR+ 3=*AR+0% circ (BK, +AR0). */
            int xar_d  = ((op >> 4) & 0x03) + 2;
            int yar_d  = (op & 0x03) + 2;
            int xmod_d = (op >> 6) & 0x03;
            int ymod_d = (op >> 2) & 0x03;
            uint16_t xval_d = data_read(s, s->ar[xar_d]);
            uint16_t yval_d = data_read(s, s->ar[yar_d]);
            {   /* [2026-08-22] MAC-PROBE : trace le corrélateur TOA (SB/FB) au MAC
                 * dual autour de PC=0xf170. Montre Xmem(AR5=IQ)/Ymem(AR3=référence),
                 * leurs adresses et valeurs lues + T : tranche « référence DC (AR3) »
                 * vs « IQ buffer non nourri (AR5, ex 0x0e4e) ». LECTURE SEULE, plafonnée.
                 * Gate CALYPSO_MAC_PROBE. */
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
            /* [2026-08-23] 5e CLASSE — la famille duale multiplie ses DEUX
             * operandes MEMOIRE entre eux, pas T par Xmem. binutils :
             *   0xa400-0xa5ff mpy   X,Y,dst   dst = X*Y      (AFFECTATION)
             *   0xa600-0xa7ff macsu X,Y,src1  src += u(X)*s(Y)
             *   0xb000-0xb3ff mac   X,Y,src,dst   dst = src + X*Y
             *   0xb400-0xb7ff macr  idem + arrondi
             * L ancien calcul `T * Xmem ; T = Ymem` est la semantique du MAC a
             * operande UNIQUE (MAC Smem,src -> src += T*Smem).
             * RACINE MESUREE : dans la boucle des coefficients du banc SCH
             * (0x81f3 LD #0,A ; RPT ; MAC *AR2+,*AR4+ ; RPT ; MAC *AR3+,*AR5+ ;
             * 0x8202 STH A), le point d arret montrait
             *   pc=0x81f5 op=0xb08a -> dst=A | T=0x0000 A=0 B=0
             * T nul au premier MAC -> produit nul -> A nul -> STH ecrit 0 ->
             * 810 zeros dans 0x2cba..0x2cbf -> MVDD les recopie -> coefficients
             * FIRS nuls. Les entrees etaient pourtant saines.
             * Effet de bord C54x : T <- Xmem (et non Ymem).
             * Gate CALYPSO_ISA_DUAL_MPY (defaut 1).
             * ⚠️ EFFET GLOBAL : ossature de tout le traitement du signal. */
            {
                static int _dm = -1;
                if (_dm < 0) {
                    _dm = calypso_gate("CALYPSO_ISA_DUAL_MPY", 1);
                    fprintf(stderr, "[c54x] ISA-DUAL-MPY %s : produit = Xmem*Ymem "
                            "(et T <- Xmem) au lieu de T*Xmem ; mpy AFFECTE, "
                            "arrondi reserve a macr\n",
                            _dm ? "ACTIF" : "INACTIF (ancien calcul)");
                }
                if (_dm) {
                    int is_mpy   = (hi8 == 0xA4 || hi8 == 0xA5);
                    int is_macsu = (hi8 == 0xA6 || hi8 == 0xA7);
                    int is_macr  = (hi8 >= 0xB4 && hi8 <= 0xB7);
                    int64_t p = is_macsu
                        ? (int64_t)(uint16_t)xval_d * (int64_t)(int16_t)yval_d
                        : (int64_t)(int16_t)xval_d * (int64_t)(int16_t)yval_d;
                    if (s->st1 & ST1_FRCT) p <<= 1;
                    if (is_macr) p += 0x8000;
                    int dstb, srcb;
                    if (is_mpy || is_macsu) {          /* masque 0xFE00 : bit8 = acc */
                        dstb = hi8 & 1; srcb = dstb;
                    } else {                            /* masque 0xFC00 : b9=src b8=dst */
                        srcb = (op >> 9) & 1; dstb = (op >> 8) & 1;
                    }
                    int64_t base = srcb ? s->b : s->a;
                    int64_t res  = is_mpy ? p : (base + p);
                    if (dstb) s->b = sext40(res);
                    else      s->a = sext40(res);
                    if (c54x_dual_sett()) s->t = xval_d;   /* effet de bord, gate */
                    {   static int _t = -1; static unsigned _tn = 0;
                        if (_t < 0) _t = calypso_gate("CALYPSO_ISA_DUAL_TRACE", 0);
                        if (_t && _tn < 24) {
                            _tn++;
                            fprintf(stderr, "[c54x] DUAL-MPY #%u PC=0x%04x op=0x%04x "
                                    "%s X=AR%d@0x%04x=%d Y=AR%d@0x%04x=%d prod=%lld "
                                    "-> %s=0x%010llx insn=%u\n",
                                    _tn, s->pc, op,
                                    is_mpy ? "mpy" : is_macsu ? "macsu" :
                                    is_macr ? "macr" : "mac",
                                    xar_d, s->ar[xar_d], (int)(int16_t)xval_d,
                                    yar_d, s->ar[yar_d], (int)(int16_t)yval_d,
                                    (long long)p, dstb ? "B" : "A",
                                    (unsigned long long)((dstb ? s->b : s->a)
                                                         & 0xFFFFFFFFFFULL),
                                    s->insn_count);
                        }
                    }
                    return consumed + s->lk_used;
                }
            }
            /* Multiply T * Xmem */
            int64_t prod = (int64_t)(int16_t)s->t * (int64_t)(int16_t)xval_d;
            if (s->st1 & ST1_FRCT) prod <<= 1;
            /* Round if R bit set (odd hi8) */
            if (hi8 & 0x01) prod += 0x8000;
            /* Determine dest and operation.
             * FIX 2026-05-29 (binutils tic54x-opc.c, confirmé) : 0xB4-0xB7 =
             * MACR = mac+round = ADDITION (0xB400/0xFC00), PAS soustraction.
             * MAS/MASR (soustraction) = 0xB8-0xBF, gérés dans des handlers
             * séparés (L7582/L7564). Ce handler ne reçoit que 0xA4-A7 + 0xB0-B7,
             * tous des accumulations (ADD). L'ancien `is_sub=(hi8 0xB4..B7)`
             * inversait le signe de l'op dominant du corrélateur FCCH (0xb4aa)
             * → corrélation = -B*T*X → jamais de pic → snr=0 → pas de FB lock. */
            int is_sub = 0;
            int dst_b;
            if (hi8 >= 0xA4 && hi8 <= 0xA7) dst_b = (hi8 >= 0xA6);
            else if (hi8 >= 0xB4 && hi8 <= 0xB7) dst_b = (hi8 >= 0xB6);
            else dst_b = (hi8 & 0x02) ? 1 : 0; /* 0xB0/B1→A, 0xB2/B3→B */
            if (dst_b) {
                if (is_sub) s->b = sext40(s->b - prod);
                else        s->b = sext40(s->b + prod);
            } else {
                if (is_sub) s->a = sext40(s->a - prod);
                else        s->a = sext40(s->a + prod);
            }
            /* T = Ymem (ancien chemin) — sous le meme gate */
            if (c54x_dual_sett()) s->t = yval_d;
            return consumed + s->lk_used;
        }

        /* SQDST Xmem, Ymem — Squared Distance (1-word dual-operand)
         * Encoding: 1010 0001 XXXX YYYY
         * Per SPRU172C: B += (AH - Xmem)^2; A = Ymem << 16; T = Xmem */
        if (hi8 == 0xA1) {
            /* Xmem/Ymem 2-bit SPRU131G T.5-6/5-8 (sweep ; régression 0xfe36 ÉCARTÉE :
             * derail byte-identique insn=193093 avec ce handler reverté → hors cause). */
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
            /* FIX 2026-06-22 (sweep) : décodage Xmem/Ymem 2-bit SPRU131G T.5-6/5-8. */
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
            /* [2026-08-23] 0xBC00-0xBFFF est MASR dual, pas POLY. binutils :
             *   { "masr", 0xBC00, 0xFC00, {OP_Xmem,OP_Ymem,OP_SRC,OP_DST} }
             *   { "poly", 0x3600, 0xFF00, {OP_Smem} }   <- POLY est ailleurs
             * SPRU172C l.1126 : dst = rnd(src - Xmem*Ymem) ET T = Xmem.
             * L ancien code executait un POLY (B += rnd(AH*T) ; A = Xmem<<16 ;
             * T = Ymem) : mauvais accumulateur, mauvais produit, mauvais T.
             * 119 occurrences. Meme gate CALYPSO_ISA_MAS_DUAL. */
            if (c54x_mas_dual()) {
                int64_t pr = (int64_t)(int16_t)xval_p * (int64_t)(int16_t)yval_p;
                if (s->st1 & ST1_FRCT) pr <<= 1;
                pr += 0x8000;                       /* arrondi */
                int srcr = (op >> 9) & 1, dstr = (op >> 8) & 1;
                int64_t baser = srcr ? s->b : s->a;
                if (dstr) s->b = sext40(baser - pr);
                else      s->a = sext40(baser - pr);
                s->t = xval_p;
                return consumed + s->lk_used;
            }
            int16_t ah_p = (int16_t)((s->a >> 16) & 0xFFFF);
            int64_t prod_p = (int64_t)ah_p * (int64_t)(int16_t)s->t;
            if (s->st1 & ST1_FRCT) prod_p <<= 1;
            prod_p += 0x8000; /* round */
            s->b = sext40(s->b + prod_p);
            s->a = sext40((int64_t)(int16_t)xval_p << 16);
            s->t = yval_p;
            return consumed + s->lk_used;
        }

        /* B8-BB: MAS/MASR Xmem, Ymem (subtract variants) or POLY-like */
        if (hi8 == 0xB8 || hi8 == 0xB9 || hi8 == 0xBA || hi8 == 0xBB) {
            /* Check if it's actually LDMM (BA) or POPM (BD) — those are handled below */
            if (hi8 == 0xBA) goto ba_handler;
            /* FIX 2026-06-22 (sweep) : décodage Xmem/Ymem 2-bit SPRU131G T.5-6/5-8. */
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
            /* [2026-08-23] MAS dual : binutils { "mas", 0xB800, 0xFC00,
             * {OP_Xmem,OP_Ymem,OP_SRC,OP_DST} }. SPRU172C l.1122 :
             *     dst = src - Xmem*Ymem   ET   T = Xmem
             * L ancien calcul faisait `T * Xmem` puis `T = Ymem` : c est la
             * semantique du MAS a operande UNIQUE appliquee a la forme duale --
             * exactement le defaut deja corrige pour mac/mpy (ISA_DUAL_MPY).
             * 122 occurrences. Gate CALYPSO_ISA_MAS_DUAL (defaut 1). */
            int64_t prod_b8;
            int dst_b8, src_b8;
            if (c54x_mas_dual()) {
                prod_b8 = (int64_t)(int16_t)xval_b8 * (int64_t)(int16_t)yval_b8;
                if (s->st1 & ST1_FRCT) prod_b8 <<= 1;
                src_b8 = (op >> 9) & 1;
                dst_b8 = (op >> 8) & 1;
                int64_t base_b8 = src_b8 ? s->b : s->a;
                if (dst_b8) s->b = sext40(base_b8 - prod_b8);
                else        s->a = sext40(base_b8 - prod_b8);
                s->t = xval_b8;
                return consumed + s->lk_used;
            }
            prod_b8 = (int64_t)(int16_t)s->t * (int64_t)(int16_t)xval_b8;
            if (s->st1 & ST1_FRCT) prod_b8 <<= 1;
            if (hi8 & 0x01) prod_b8 += 0x8000;
            dst_b8 = (hi8 & 0x02) ? 1 : 0;
            /* MAS: subtract */
            if (dst_b8) s->b = sext40(s->b - prod_b8);
            else        s->a = sext40(s->a - prod_b8);
            s->t = yval_b8;
            return consumed + s->lk_used;
        }
ba_handler:
        if (hi8 == 0xAA || hi8 == 0xAB) {
            /* STUB-NOP : tic54x dit 0xAA/AB = LD variant.
             * Ancienne classification qemu = STLM src,MMR (incorrect — STLM
             * est en 0x88/0x89, déjà correctement décodé ligne 4046).
             * Voir doc/opcodes/tic54x_hi8_map.md. Neutralisé. */
            return 1;
        }
        if (hi8 == 0xBA) {
            /* LDMM MMR, dst — load MMR value into accumulator
             * Per SPRU172C: dst[15:0] = MMR, dst[31:16] = sign-ext (SXM)/0
             * BUG FIX 2026-05-24 : was `sext40(v << 16)` which put MMR in
             * dst[31:16], wrong half. The boot-stub at 0x0000 (LDMM SP,B)
             * is supposed to return B = current SP for caller's use ; with
             * the << 16 bug, B[31:16] = SP, B[15:0] = 0, breaking any
             * downstream caller that reads B as 16-bit SP value. */
            uint16_t mmr = op & 0x7F;
            int dst = (op >> 4) & 1;
            int64_t v = (int64_t)(int16_t)data_read(s, mmr);
            if (dst) s->b = sext40(v);
            else     s->a = sext40(v);
            return consumed + s->lk_used;
        }
        /* [2026-08-23] FAMILLE PARALLELE LD||MAC : 0xA800-0xAFFF, UN SEUL MOT.
         * binutils (tic54x_paroptab) :
         *   0xA800/0xFE00  ld Xmem,dst || mac  Ymem
         *   0xAA00/0xFE00  ld Xmem,dst || macr Ymem
         *   0xAC00/0xFE00  ld Xmem,dst || mas  Ymem
         *   0xAE00/0xFE00  ld Xmem,dst || masr Ymem
         * SPRU172C : dst = Xmem << 16 ; dst_ = dst_ +/- T*Ymem (arrondi pour les
         * variantes R) ; **T INCHANGE**. dst = bit 8, dst_ = l autre accumulateur.
         *
         * L ancien decodage etait faux sur les QUATRE :
         *   0xA8/0xA9 -> `AND #lk` sur 2 MOTS (le vrai AND #lk est en 0xF030)
         *   0xAA/0xAB -> stub muet `return 1;` (longueur juste, effet nul :
         *                193 occurrences silencieuses)
         *   0xAC/0xAD -> `MACP Smem,pmad` sur 2 MOTS (le vrai MACP est en 0x7800)
         *   0xAE/0xAF -> `MACD Smem,pmad` sur 2 MOTS (le vrai MACD est en 0x7A00),
         *                avec en prime `s->t = sval` : ECRITURE PARASITE DE T.
         * Les trois formes a 2 mots AVALAIENT un mot de trop a chaque occurrence
         * (233 sites) : desynchronisation du flux d instructions.
         * Gate CALYPSO_ISA_LD_PAR (defaut 1). */
        if (c54x_ld_par() && (op & 0xF800) == 0xA800) {
            int xar_l  = ((op >> 4) & 0x03) + 2;
            int yar_l  = ( op       & 0x03) + 2;
            int xmod_l = (op >> 6) & 0x03;
            int ymod_l = (op >> 2) & 0x03;
            uint16_t xv_l = data_read(s, s->ar[xar_l]);
            uint16_t yv_l = data_read(s, s->ar[yar_l]);
            int dst_l  = (op >> 8) & 1;              /* accumulateur du LD      */
            int soust  = ((op >> 10) & 1);           /* 0xAC/0xAE = soustraction */
            int arrondi= ((op >> 9)  & 1);           /* 0xAA/0xAE = arrondi      */
            int64_t p_l = (int64_t)(int16_t)s->t * (int64_t)(int16_t)yv_l;
            if (s->st1 & ST1_FRCT) p_l <<= 1;
            if (arrondi) p_l += 0x8000;
            int64_t *acc_ld = dst_l ? &s->b : &s->a;   /* recoit le LD  */
            int64_t *acc_op = dst_l ? &s->a : &s->b;   /* recoit le MAC */
            *acc_op = sext40(soust ? (*acc_op - p_l) : (*acc_op + p_l));
            *acc_ld = sext40((int64_t)(int16_t)xv_l << 16);
            c54x_par_postmod(s, xar_l, xmod_l);
            c54x_par_postmod(s, yar_l, ymod_l);
            return consumed + s->lk_used;            /* 1 MOT, T INCHANGE */
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
        /* [2026-08-22] 4e CLASSE — famille 0xA000-0xA1FF entiere mal decodee.
         * binutils (doc/opcodes/tic54x-opc.c) n a qu UNE entree sur la plage :
         *     { "add", 1,3,3, 0xA000, 0xFE00, {OP_Xmem,OP_Ymem,OP_DST} }
         *   ADD Xmem, Ymem, dst  ->  dst = (Xmem << 16) + (Ymem << 16)
         * Le handler qui suit la traitait comme des operations sur accumulateur
         * (LD B,A / NEG / ABS / SAT / SFTA / SFTL) ; or celles-ci sont toutes en
         * 0xF4xx-0xF0xx d apres la meme table (ld 0xF482, neg 0xF484, abs 0xF485,
         * sat 0xF483, sfta 0xF460, sftl 0xF0E0). Aucune n est en 0xA0.
         *
         * RACINE MESUREE. Derniere instruction du corps de boucle du correlateur
         * (RPTB 0x84b0, REA=0x84c6) :
         *     0x84c6 : 0xA09A = add *AR3+, *AR4+, A
         * Bas-octet 0x9A -> Xar=AR3 (*AR+), Yar=AR4 (*AR+) : les deux pointeurs
         * d ENTREE du correlateur (0x2a27 / 0x2ae7 dans le tampon de burst).
         * C est elle qui fait GLISSER la fenetre d un mot par decalage. L ancien
         * code y voyait sub=0x9A, bit 7 arme, et executait un SFTL de 77 bits :
         * aucun pointeur n avancait, les 50 decalages correlaient la meme
         * fenetre (mesure : distinct = 3/50 et 5/50), l argmax perdait tout sens,
         * et le SB n etait jamais decode.
         * gr-gsm confirme la semantique : la reference repart de zero a chaque
         * lag, c est l entree qui glisse. osmocom-bb ne correle rien lui-meme.
         * Gate CALYPSO_ISA_A0_ADD (defaut 1) ; =0 restaure l ancien decodage.
         * ⚠️ EFFET GLOBAL : ADD Xmem,Ymem,dst ne sert pas qu au correlateur. */
        if ((op & 0xFE00) == 0xA000) {
            static int _a0 = -1;
            if (_a0 < 0) {
                _a0 = calypso_gate("CALYPSO_ISA_A0_ADD", 1);
                fprintf(stderr, "[c54x] ISA-A0-ADD %s : 0xA000-0xA1FF = "
                        "ADD Xmem,Ymem,dst (dst = (Xmem+Ymem)<<16, post-modif des "
                        "DEUX pointeurs) au lieu des operations accumulateur\n",
                        _a0 ? "ACTIF" : "INACTIF (ancien decodage)");
            }
            if (_a0) {
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
                {   static int _t = -1; static unsigned _tn = 0;
                    if (_t < 0) _t = calypso_gate("CALYPSO_ISA_A0_TRACE", 0);
                    if (_t && _tn < 24) {
                        _tn++;
                        fprintf(stderr, "[c54x] A0-ADD #%u PC=0x%04x op=0x%04x "
                                "Xar=AR%d@0x%04x=%d Yar=AR%d@0x%04x=%d -> %s"
                                "=0x%010llx | AR%d->0x%04x AR%d->0x%04x insn=%u\n",
                                _tn, s->pc, op, xar, s->ar[xar], (int)xv,
                                yar, s->ar[yar], (int)yv,
                                ((op >> 8) & 1) ? "B" : "A",
                                (unsigned long long)(*dst & 0xFFFFFFFFFFULL),
                                xar, s->ar[xar], yar, s->ar[yar], s->insn_count);
                    }
                }
                return consumed + s->lk_used;
            }
        }
        if (hi8 == 0xA0) {
            /* A0xx: accumulator operations — LD/NEG/ABS/NOT/SFTA/SFTL/SAT
             * Per SPRU172C:
             *   A000/A001: LD B,A / LD A,B
             *   A004/A005: NOT A / NOT B
             *   A008/A009: NEG A / NEG B
             *   A00A/A00B: ABS A / ABS B
             *   A00C/A00D: MAX A / MAX B (sat + clip)
             *   A00E/A00F: MIN A / MIN B
             *   bit7=0: SFTA dst, SHIFT — 1010 0000 0SSS SSSD (arith shift)
             *   bit7=1: SFTL dst, SHIFT — 1010 0000 1SSS SSSD (logical shift)
             *   A098/A099: SAT A / SAT B
             */
            uint8_t sub = op & 0xFF;
            if (sub == 0x00) { s->a = s->b; }
            else if (sub == 0x01) { s->b = s->a; }
            else if (sub == 0x04) { s->a = sext40(~s->a); } /* NOT A */
            else if (sub == 0x05) { s->b = sext40(~s->b); } /* NOT B */
            else if (sub == 0x08) { s->a = sext40(-s->a); } /* NEG A */
            else if (sub == 0x09) { s->b = sext40(-s->b); } /* NEG B */
            else if (sub == 0x0A) { s->a = sext40((s->a < 0) ? -s->a : s->a); } /* ABS A */
            else if (sub == 0x0B) { s->b = sext40((s->b < 0) ? -s->b : s->b); } /* ABS B */
            else if (sub == 0x98) { /* SAT A */
                if (s->a > 0x7FFFFFFFFFLL) s->a = 0x7FFFFFFFFFLL;
                else if (s->a < -0x8000000000LL) s->a = -0x8000000000LL;
                s->st0 &= ~ST0_OVA;
            }
            else if (sub == 0x99) { /* SAT B */
                if (s->b > 0x7FFFFFFFFFLL) s->b = 0x7FFFFFFFFFLL;
                else if (s->b < -0x8000000000LL) s->b = -0x8000000000LL;
                s->st0 &= ~ST0_OVB;
            }
            else if (sub & 0x80) {
                /* SFTL dst, SHIFT — logical shift, bits[6:1]=shift, bit[0]=dst */
                int shift = (sub >> 1) & 0x3F;
                if (shift & 0x20) shift |= ~0x3F;  /* sign-extend 6-bit */
                int dst = sub & 1;
                int64_t *acc = dst ? &s->b : &s->a;
                uint64_t u = (uint64_t)(*acc) & 0xFFFFFFFFFFULL;
                if (shift >= 0) *acc = sext40((int64_t)(u << shift));
                else            *acc = sext40((int64_t)(u >> (-shift)));
            }
            else if (sub >= 0x10) {
                /* SFTA dst, SHIFT — arithmetic shift, bits[6:1]=shift, bit[0]=dst */
                int shift = (sub >> 1) & 0x3F;
                if (shift & 0x20) shift |= ~0x3F;  /* sign-extend 6-bit */
                int dst = sub & 1;
                int64_t *acc = dst ? &s->b : &s->a;
                if (shift >= 0) *acc = sext40(*acc << shift);
                else            *acc = sext40(*acc >> (-shift));
            }
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
        /* 0xB3 = MACR Xmem, Ymem, B (1-word, handled above with MAC family).
         * Fix 2026-05-29 : avant ce handler décodait 0xB3xx comme
         * `LD #lk, dst` (2-word), ce qui faisait drift le PC de +1 dans
         * une routine RPTBD à 0x820e..0x820f → boucle infinie au PC=
         * 0x821a (= IMR clobber via délai-slot du BANZD). */
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
         * [2026-08-22] FIX ISA STRUCTUREL — famille parallele
         *     ST src,Ymem || <op> Xmem,dst      (1 MOT, SPRU172C 4-177..4-185)
         *
         * RACINE. Toute la plage 0xC000-0xDFFF est UNE seule famille (table
         * binutils versee au depot, doc/opcodes/tic54x-opc.c L477-493 : quatre
         * entrees "st" de masque 0xFC00 + celles de 0xD000/D400/D800/DC00,
         * toutes marquees FL_PAR). L'emulateur la decoupait en instructions
         * SANS RAPPORT, dont deux de 2 MOTS :
         *     0xC2/C3/C6/C7 -> RPTB[D] pmad (2 mots)   0xC4 -> PSHD dmad (2 mots)
         *     0xC0/C1 -> PSHD/RPT Smem                 0xCC -> SACCD
         *     0xDA -> RPTBD (2 mots)                   0xDF -> DELAY Smem
         *     0xC5/CD/CE/CF/DD/DE -> NOP muet
         * Or le VRAI RPTB est 0xF072 et RPTBD 0xF272 (deja corrects ailleurs),
         * PSHD 0x4B00, RPT Smem 0x4700, SACCD 0x9E00, DELAY 0x4D00.
         *
         * EFFET MESURE. Le noyau du correlateur FB/SB (PDROM 0xf16c..0xf17e,
         * boucle `rptbd 0xf178`) est bati sur cette famille :
         *     0xf172 c780 = ST||SUB    0xf173 ce91 = ST||MPY
         *     0xf176 c709 = ST||SUB    0xf178 ce98 = ST||MPY
         * Decodes en RPTB de 2 mots, ils (a) avalaient le mot suivant -> flux
         * d'instructions desynchronise, (b) ecrasaient rea/rsa/rptb_active/BRAF
         * de la boucle rptbd VIVANTE qui les contient. La surface de correlation
         * etait donc calculee sur un programme different de celui du silicium ->
         * pic faux -> argmax faux -> a_sync_demod[D_TOA] = mot arbitraire
         * (osmocon : TOA=-7791 / -3691 / +8026 / -28903, hors plage 0..156)
         * et d_fb_det=1 seulement ~1 fois sur 95.
         *
         * SEMANTIQUE (SPRU172C, encodage `1100xxSD`/`1101RSD` XXXXYYYY) :
         *   store   : Ymem = (src << ASM) >> 16          (src = b9, PAS b8)
         *   op      : C0-C3 dst += Xmem<<16 | C4-C7 dst = Xmem<<16 - dst
         *             C8-CB dst  = Xmem<<16 | CC-CF dst = T*Xmem
         *             D0-D7 dst += T*Xmem   | D8-DF dst -= T*Xmem  (R=b10 arrondi)
         *   Xmem est lu AVANT le store (« If src is equal to dst, the value
         *   stored in Ymem is the value of src before the execution »).
         *   T n'est JAMAIS ecrit par cette famille.
         *
         * /!\ EFFET GLOBAL, comme tout correctif d'ISA : cette famille sert aussi
         * aux filtres, au Viterbi et aux tampons I/Q. Bascule A/B :
         * CALYPSO_ISA_PAR_ST=0 restaure INTEGRALEMENT l'ancien decodage (les
         * anciennes branches sont conservees telles quelles ci-dessous).
         * A verifier en shunt_legit : camp + LU + SMS, comme pour LDK8_SHIFT16.
         * Source de verite : doc/RAPPORT_OPCODES.md section A (C-1..C-9).
         * ================================================================ */
        {
            static int _par_st = -1;
            if (_par_st < 0) {
                _par_st = calypso_gate("CALYPSO_ISA_PAR_ST", 1);
                fprintf(stderr, "[c54x] ISA-PAR-ST %s : 0xC000-0xDFFF = "
                        "ST src,Ymem || add/sub/ld/mpy/mac[r]/mas[r] Xmem,dst (1 mot)\n",
                        _par_st ? "ACTIF (fidele SPRU172C)"
                                : "INACTIF (ancien decodage RPTB/PSHD/SACCD/NOP)");
            }
            if (_par_st) {
                int s_acc = (op >> 9) & 1;        /* S = b9 : acc source du ST        */
                int d_acc = (op >> 8) & 1;        /* D = b8 : acc destination de l'op */
                int xmod  = (op >> 6) & 3;
                int xar   = ((op >> 4) & 3) + 2;  /* Xmem : AR2..AR5 */
                int ymod  = (op >> 2) & 3;
                int yar   = ( op       & 3) + 2;  /* Ymem : AR2..AR5 */
                uint16_t xaddr = s->ar[xar];
                uint16_t yaddr = s->ar[yar];
                uint16_t xval  = data_read(s, xaddr);   /* lu AVANT le store */
                int64_t  sv    = s_acc ? s->b : s->a;
                int64_t *dstp  = d_acc ? &s->b : &s->a;
                int      ash   = asm_shift(s);
                int64_t  sh    = (ash >= 0) ? (sv << ash) : (sv >> (-ash));
                int64_t  xs    = (int64_t)(int16_t)xval;
                int64_t  prod;

                /* ST src,Ymem : Ymem = (src << ASM) >> 16 */
                data_write(s, yaddr, (uint16_t)((sh >> 16) & 0xFFFF));

                switch ((op >> 10) & 7) {         /* bits 12:10 = sous-classe */
                /* [2026-09-17] FIX_PAR_ST_DSTBAR — SPRU172C 4-177/4-185 :
                 * « || ADD Xmem, dst : dst = dst_ + Xmem << 16 » et
                 * « || SUB Xmem, dst : dst = (Xmem << 16) - dst_ », dst_ etant
                 * l'AUTRE accumulateur (si dst = A, dst_ = B). On utilisait dst.
                 * Le demodulateur SB (0x76e4/0x76f5 `c0dc`) s'en sert 896x/burst. */
                case 0: { /* C0-C3 : || ADD Xmem,dst */
                    static int fix_db = -1;
                    if (fix_db < 0) fix_db = calypso_gate("CALYPSO_FIX_PAR_ST_DSTBAR", 1);
                    int64_t other = d_acc ? s->a : s->b;
                    *dstp = sext40((fix_db ? other : *dstp) + (xs << 16));
                    break; }
                case 1: { /* C4-C7 : || SUB Xmem,dst  ->  Xmem<<16 - dst_ */
                    static int fix_db = -1;
                    if (fix_db < 0) fix_db = calypso_gate("CALYPSO_FIX_PAR_ST_DSTBAR", 1);
                    int64_t other = d_acc ? s->a : s->b;
                    *dstp = sext40((xs << 16) - (fix_db ? other : *dstp));
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
                    if (op & 0x0400) {            /* R = b10 : arrondi */
                        prod += 0x8000;
                        prod &= ~0xFFFFLL;
                    }
                    *dstp = sext40((op & 0x0800) ? (*dstp - prod)
                                                 : (*dstp + prod));
                    break;
                }
                /* T n'est PAS modifie (contrairement a l'ancien handler MAC dual). */
                c54x_par_postmod(s, xar, xmod);
                c54x_par_postmod(s, yar, ymod);
                return consumed + s->lk_used;
            }
        }
        /* ---- ci-dessous : ANCIEN decodage, conserve pour CALYPSO_ISA_PAR_ST=0 ---- */
        /* C/Dxxx: PSHM, POPM, PSHD, POPD, RPT, FRAME, etc. */

        /* ---- Dual-operand MAC/MAS Xmem, Ymem, dst (1-word) ----
         * 0xD0: MAC Xmem,Ymem,A   0xD2: MAC Xmem,Ymem,B
         * 0xD1: MACR Xmem,Ymem,A  0xD3: MACR Xmem,Ymem,B
         * 0xD4-0xD7: MAS variants (subtract)
         *
         * Encoding per binutils tic54x.h (XARX/YARX = ((C&0x3)+2)) :
         *   bits 7:6 Xmod  | 5:4 Xar (AR2..AR5) | 3:2 Ymod | 1:0 Yar (AR2..AR5)
         * Was 3-bit AR raw — same bug as C8/CB had (fixed 2026-05-08). Now
         * aligned with binutils. Expected aftermath : new SP-CATASTROPHE on
         * D-class opcodes when firmware ARs land at MMR — same root pattern
         * as 0xc8be at PC=0xa0e7. That's correct exposure, not regression. */
        if (hi8 >= 0xD0 && hi8 <= 0xD9 && hi8 != 0xDA) {
            int xmod_c = (op >> 6) & 0x03;
            int xar_c  = ((op >> 4) & 0x03) + 2;
            int ymod_c = (op >> 2) & 0x03;
            int yar_c  = (op & 0x03) + 2;
            uint16_t xval_c = data_read(s, s->ar[xar_c]);
            uint16_t yval_c = data_read(s, s->ar[yar_c]);
            switch (xmod_c) {
            case 0: break;
            case 1: s->ar[xar_c]--; break;   /* *AR- (SPRU131G T.5-8 : 01=dec) — fix 2026-06-22 */
            case 2: s->ar[xar_c]++; break;   /* *AR+ (10=inc) */
            case 3: s->ar[xar_c] = c54x_circ_ref(s->ar[xar_c], +(int16_t)s->ar[0], s->bk); break; /* *AR+0% circ — fix 2026-06-01 */
            }
            switch (ymod_c) {
            case 0: break;
            case 1: s->ar[yar_c]--; break;   /* fix 2026-06-22 (SPRU131G T.5-8) */
            case 2: s->ar[yar_c]++; break;
            case 3: s->ar[yar_c] = c54x_circ_ref(s->ar[yar_c], +(int16_t)s->ar[0], s->bk); break; /* *AR+0% circ (Ymem 0xd3dc @0xfa98) — fix 2026-06-01 */
            }
            /* MAC dual-mem formula : T × Xmem (pas X × Y per SPRU pure).
             *
             * 2026-05-08 retest empirique avec pipeline stable :
             *   T×X  : BRC variable, A/B accumulator drift, d_fb_det reaches
             *          high SNR values (0x7902 / 0x7766) at moments
             *   X×Y  : BRC=0 uniforme (201/201), A=B=0 forever, d_fb_det
             *          mostly 0 — correlation produces only zeros
             *
             * Le firmware Calypso s'appuie sur le pipeline c54x : T est
             * latched depuis Ymem du MAC précédent (T = Y(post)). Ainsi
             * MAC dual-mem effectivement calcule `T_old × X_current` =
             * `Y[n-1] × X[n]`. Notre `prod = T × X` reproduit fidèlement
             * cet effet pipelined. `X × Y` (les 2 du buffer courant) ne
             * matche pas la sémantique attendue par le firmware. */
            int64_t prod_c = (int64_t)(int16_t)s->t * (int64_t)(int16_t)xval_c;
            if (s->st1 & ST1_FRCT) prod_c <<= 1;
            if (hi8 & 0x01) prod_c += 0x8000; /* round */
            int is_sub_c = (hi8 >= 0xD4);
            int dst_c = (hi8 & 0x02) ? 1 : 0;
            if (dst_c) {
                if (is_sub_c) s->b = sext40(s->b - prod_c);
                else          s->b = sext40(s->b + prod_c);
            } else {
                if (is_sub_c) s->a = sext40(s->a - prod_c);
                else          s->a = sext40(s->a + prod_c);
            }
            s->t = yval_c;
            return consumed + s->lk_used;
        }

        /* DBxx: MASA Xmem, Ymem, dst — MAC with accumulator sign extension
         * Per SPRU172C: same as MAC but T loaded from Xmem instead of Ymem.
         * dst += T * Xmem, T = Xmem
         * Encoding fixed 2026-05-08 : same 2-bit AR + offset 2 + 2-bit mod
         * format as the rest of the dual-operand class. */
        if (hi8 == 0xDB) {
            int xmod_db = (op >> 6) & 0x03;
            int xar_db  = ((op >> 4) & 0x03) + 2;
            int ymod_db = (op >> 2) & 0x03;
            int yar_db  = (op & 0x03) + 2;
            uint16_t xval_db = data_read(s, s->ar[xar_db]);
            (void)data_read(s, s->ar[yar_db]); /* Ymem read (unused) */
            switch (xmod_db) {
            case 0: break;
            case 1: s->ar[xar_db]--; break;   /* fix 2026-06-22 (SPRU131G T.5-8) */
            case 2: s->ar[xar_db]++; break;
            case 3: s->ar[xar_db] = c54x_circ_ref(s->ar[xar_db], +(int16_t)s->ar[0], s->bk); break; /* *AR+0% circ — fix 2026-06-01 */
            }
            switch (ymod_db) {
            case 0: break;
            case 1: s->ar[yar_db]--; break;   /* fix 2026-06-22 (SPRU131G T.5-8) */
            case 2: s->ar[yar_db]++; break;
            case 3: s->ar[yar_db] = c54x_circ_ref(s->ar[yar_db], +(int16_t)s->ar[0], s->bk); break; /* *AR+0% circ — fix 2026-06-01 */
            }
            int64_t prod_db = (int64_t)(int16_t)s->t * (int64_t)(int16_t)xval_db;
            if (s->st1 & ST1_FRCT) prod_db <<= 1;
            s->a = sext40(s->a + prod_db);
            s->t = xval_db;
            return consumed + s->lk_used;
        }

        /* DCxx: SQUR Xmem, dst — Square and accumulate (1-word dual-operand)
         * Per SPRU172C p.4-165: T = Xmem, dst = dst + T * T
         * Encoding fixed 2026-05-08 : same dual-operand format as D0-D9. */
        if (hi8 == 0xDC) {
            int xmod_dc = (op >> 6) & 0x03;
            int xar_dc  = ((op >> 4) & 0x03) + 2;
            int ymod_dc = (op >> 2) & 0x03;
            int yar_dc  = (op & 0x03) + 2;
            uint16_t xval_dc = data_read(s, s->ar[xar_dc]);
            (void)data_read(s, s->ar[yar_dc]); /* Ymem pipeline read */
            switch (xmod_dc) {
            case 0: break;
            case 1: s->ar[xar_dc]--; break;   /* fix 2026-06-22 (SPRU131G T.5-8) */
            case 2: s->ar[xar_dc]++; break;
            case 3: s->ar[xar_dc] = c54x_circ_ref(s->ar[xar_dc], +(int16_t)s->ar[0], s->bk); break; /* *AR+0% circ — fix 2026-06-01 */
            }
            switch (ymod_dc) {
            case 0: break;
            case 1: s->ar[yar_dc]--; break;   /* fix 2026-06-22 (SPRU131G T.5-8) */
            case 2: s->ar[yar_dc]++; break;
            case 3: s->ar[yar_dc] = c54x_circ_ref(s->ar[yar_dc], +(int16_t)s->ar[0], s->bk); break; /* *AR+0% circ — fix 2026-06-01 */
            }
            s->t = xval_dc;
            int64_t prod_dc = (int64_t)(int16_t)xval_dc * (int64_t)(int16_t)xval_dc;
            if (s->st1 & ST1_FRCT) prod_dc <<= 1;
            s->a = sext40(s->a + prod_dc);
            return consumed + s->lk_used;
        }

        /* CA/CB handled by the unified C8/C9/CA/CB block below. */
        /* CF: variant parallel or DELAY */
        if (hi8 == 0xCF) {
            /* Treat as NOP for now — rare instruction */
            return consumed + s->lk_used;
        }
        /* RPTB[D] pmad — Block repeat (2 words)
         * C2xx: RPTB pmad, C3xx: RPTBD pmad (delayed)
         * Per SPRU172C: RSA = PC+2, REA = pmad, BRAF = 1 */
        if (hi8 == 0xC2 || hi8 == 0xC3 || hi8 == 0xC6 || hi8 == 0xC7) {
            op2 = prog_fetch(s, s->pc + 1);
            consumed = 2;
            s->rea = op2;
            s->rsa = (uint16_t)(s->pc + 2);
            s->rptb_active = true;
            s->st1 |= ST1_BRAF;
            return consumed + s->lk_used;
        }
        if (hi8 == 0xC5) {
            /* STUB-NOP : tic54x dit 0xC5 = ST||family (parallel).
             * Ancienne classification qemu = PSHM MMR (incorrect — vrai
             * PSHM est en 0x4A, correctement décodé ligne 3816).
             * Le sp-- ici causait des pushes fantômes. Neutralisé. */
            return 1;
        }
        if (hi8 == 0xCD) {
            /* STUB-NOP : tic54x dit 0xCD = ST||family (parallel).
             * Ancienne classification qemu = POPM MMR (incorrect — vrai
             * POPM est en 0x8A, fixé 2026-05-08).
             * Le sp++ ici causait des pops fantômes. Neutralisé. */
            return 1;
        }
        if (hi8 == 0xCE) {
            /* STUB-NOP : tic54x dit 0xCE = ST||family (parallel).
             * Ancienne classification qemu = FRAME #k (incorrect — FRAME
             * n'a pas de hi8 fixe, encodage différent).
             * Le sp+=k ici causait des sauts SP arbitraires. Neutralisé. */
            return 1;
        }
        if (hi8 == 0xC4) {
            /* C4xx: PSHD dmad (push data from absolute addr) */
            op2 = prog_fetch(s, s->pc + 1);
            consumed = 2;
            s->sp--;
            data_write(s, s->sp, data_read(s, op2));
            return consumed + s->lk_used;
        }
        if (hi8 == 0xC0 || hi8 == 0xC1) {
            /* PSHD Smem / RPT Smem variants */
            addr = resolve_smem(s, op, &ind);
            if (hi8 == 0xC0) {
                /* PSHD Smem */
                s->sp--;
                data_write(s, s->sp, data_read(s, addr));
            } else {
                /* RPT Smem */
                s->rpt_count = data_read(s, addr);
                s->rpt_active = true; s->rpt_fresh = true;
                s->pc += consumed;
                return 0;
            }
            return consumed + s->lk_used;
        }
        if (hi8 == 0xCC) {
            /* CCxx: SACCD Smem, ARmem — Store Acc Conditionally (1-word)
             * Per SPRU172C: conditionally store AH or BH to Smem.
             * Simplified: always store (condition always true). */
            addr = resolve_smem(s, op, &ind);
            data_write(s, addr, (uint16_t)((s->a >> 16) & 0xFFFF));
            return consumed + s->lk_used;
        }
        if (hi8 == 0xDA) {
            /* DAxx: RPTBD pmad (block repeat delayed, 2 words) */
            op2 = prog_fetch(s, s->pc + 1);
            consumed = 2;
            s->rea = op2;
            s->rsa = (uint16_t)(s->pc + 4); /* delayed: skip 2 delay slots */
            s->rptb_active = true;
            s->st1 |= ST1_BRAF;
            return consumed + s->lk_used;
        }
        if (hi8 == 0xDD) {
            /* STUB-NOP : tic54x dit 0xDD = ST||family (parallel) — base
             * 0xDC00 mask 0xFC00. Ancienne classification qemu = POPD Smem
             * (incorrect — vrai POPD en 0x8B, neutralisé en stub).
             * Le sp++ ici causait le SP runaway post-POPM-fix observé
             * 2026-05-08 (~13k faux pops en 64k insn). Neutralisé. */
            return 1;
        }
        if (hi8 == 0xDE) {
            /* STUB-NOP : tic54x dit 0xDE = ST||family (parallel).
             * Ancienne classification qemu = POPD dmad 2-word (incorrect).
             * Le sp++ ici causait le SP runaway. Neutralisé. */
            return 1;
        }
        if (hi8 == 0xDF) {
            /* DELAY Smem — shift delay line: data(Smem) → data(Smem+1)
             * Per SPRU172C: used with RPT for FIR filter delay lines */
            addr = resolve_smem(s, op, &ind);
            uint16_t dval = data_read(s, addr);
            data_write(s, addr + 1, dval);
            return consumed + s->lk_used;
        }
        /* 0xC8/C9/CA/CB: ST SRC, Ymem || LD Xmem, DST  (1-word parallel)
         *
         * Encoding per SPRU172C §5.5 (Parallel store + arithmetic format,
         * cross-checked against tic54x-opc.c entry "0xC800/0xFC00 st||ld") :
         *
         *   bit 15..10 : opcode (110010)
         *   bit  9     : reserved (used to disambiguate; here: 0 for C8/CA,
         *                bit 9 of 0xC9/CB still in opcode space — but the
         *                effective operand bits for parallel are 7:0)
         *   bit  8     : SRC accumulator select (0 = A, 1 = B)
         *   bits 7:6   : Xmod  (0=*ARi  1=*ARi+  2=*ARi-  3=*ARi+0%)
         *   bits 5:4   : Xar   (00=AR2, 01=AR3, 10=AR4, 11=AR5) — only AR2..AR5
         *   bits 3:2   : Ymod  (same encoding as Xmod)
         *   bits 1:0   : Yar   (same encoding as Xar)
         *
         * Bug fix 2026-05-08 v2 evidence (DUAL-OP-INTERPRET log) :
         *   Previously decoded as `xar=(op>>4)&7`, `yar=op&7` (3-bit AR
         *   field) with bit 7 = Xmod ±, bit 3 = Ymod ±. That picked
         *   AR0/AR1 instead of AR2/AR3 and made post-mod always ± with
         *   no support for "no mod" or `*ARi+0%`. When firmware loaded
         *   AR1=0x0018 (= MMR_SP) for an unrelated reason, the *AR1
         *   write landed on the SP MMR slot — observed catastrophes
         *   Δ=+16601 / -16640 at PC=0x7818 / 0x786b are the consequence.
         *
         * Note on 0xCA/CB : per tic54x-opc.c, 0xC800 mask 0xFC00 covers
         * 0xC800..0xCBFF for ST||LD (single instruction class). The
         * earlier emulator split CA/CB into a separate block — that
         * block is now removed, the C8..CB handler is unified here. */
        if (hi8 >= 0xC8 && hi8 <= 0xCB) {
            int s_acc = (hi8 & 0x01) ? 1 : 0;          /* C9/CB store from B */
            int xmod  = (op >> 6) & 0x03;
            int xar   = ((op >> 4) & 0x03) + 2;        /* AR2..AR5 */
            int ymod  = (op >> 2) & 0x03;
            int yar   = (op & 0x03) + 2;               /* AR2..AR5 */
            int d_acc = s_acc ? 0 : 1;                 /* LD into the OTHER acc */
            int64_t st_val = s_acc ? s->b : s->a;
            /* STLD-SP (patch #2 diag, gated CALYPSO_DEBUG=STLD-SP) : au site
             * SP-CATASTROPHE (défaut PC=0xa0e7, env CALYPSO_TRACE_STLD_PC),
             * dump la cible RÉELLE = AR[yar] PRÉ-modify + flag MMR_SP. Tranche
             * le fork CC : si AR[yar]==MMR_SP(0x18) → le ST écrit SP (AR stale
             * via STM skippé) ; sinon → write DARAM légal. */
            {
                static int stld_pc = -1;
                if (stld_pc < 0) {
                    const char *e = getenv("CALYPSO_TRACE_STLD_PC");
                    stld_pc = (e && *e) ? (int)strtol(e, NULL, 0) : 0xa0e7;
                }
                if (s->pc == (uint16_t)stld_pc) {
                    C54_DBG("STLD-SP",
                        "STLD-SP op=0x%04x PC=0x%04x s_acc=%d yar=AR%d "
                        "tgt(AR%d_pre)=0x%04x is_MMR_SP=%d xar=AR%d AR%d=0x%04x "
                        "st_val=0x%010llx",
                        op, s->pc, s_acc, yar, yar, s->ar[yar],
                        (s->ar[yar] == MMR_SP), xar, xar, s->ar[xar],
                        (unsigned long long)(st_val & 0xFFFFFFFFFFULL));
                }
            }
            data_write(s, s->ar[yar], (uint16_t)(st_val & 0xFFFF));
            uint16_t ld_val = data_read(s, s->ar[xar]);
            int64_t loaded = (int64_t)(int16_t)ld_val << 16;
            if (d_acc) s->b = sext40(loaded); else s->a = sext40(loaded);
            switch (xmod) {
            case 0: break;                             /* *ARi (no mod) */
            case 1: s->ar[xar]--; break;               /* *ARi- (SPRU131G T.5-8 : 01=dec) — fix 2026-06-22 */
            case 2: s->ar[xar]++; break;               /* *ARi+ (10=inc) */
            case 3:                                    /* *ARi+0% — CIRCULAIRE modulo BK
                                                        * (était linear += AR0 → AR drift
                                                        * 16-bit vers 0x18=MMR_SP → SP-CATAS).
                                                        * Miroir du single-operand case 0xE. */
                if (s->bk) {
                    uint16_t base = s->ar[xar] - (s->ar[xar] % s->bk);
                    uint16_t v = s->ar[xar] + s->ar[0];
                    if (v >= (uint16_t)(base + s->bk)) v -= s->bk;
                    s->ar[xar] = v;
                } else {
                    s->ar[xar] += s->ar[0];            /* BK=0 → linéaire (pas de circ) */
                }
                break;
            }
            switch (ymod) {
            case 0: break;
            case 1: s->ar[yar]--; break;               /* *ARi- (SPRU131G 01=dec) — fix 2026-06-22 */
            case 2: s->ar[yar]++; break;               /* *ARi+ (10=inc) */
            case 3:                                    /* *ARi+0% — circulaire modulo BK */
                if (s->bk) {
                    uint16_t base = s->ar[yar] - (s->ar[yar] % s->bk);
                    uint16_t v = s->ar[yar] + s->ar[0];
                    if (v >= (uint16_t)(base + s->bk)) v -= s->bk;
                    s->ar[yar] = v;
                } else {
                    s->ar[yar] += s->ar[0];
                }
                break;
            }
            return consumed + s->lk_used;
        }
        goto unimpl;

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

/* ================================================================
 * Main execution loop
 * ================================================================ */

/* DSP idle fast-forward — simulator optimisation, NOT a hack.
 *
 * The Calypso DSP polls its task slots in NDB and write pages while
 * waiting for ARM/TPU to post work. Empirically this dispatcher loop
 * lives in PROM1 mirror at PC 0xe9ac..0xe9b7 (8-instruction body cycled
 * ~285k times per 1.4G insn window when nothing pending). Each iteration
 * costs C-level MAC/branch emulation that ends up consuming 80%+ of host
 * CPU for zero useful work, making QEMU run ~3x slower than wall-clock
 * GSM and starving the BTS scheduler of CLK INDs.
 *
 * Detection: PC inside the polling range AND all four task fields in
 * both write pages are zero AND no interrupt pending. When confirmed,
 * advance cycles/insn_count without invoking c54x_exec_one. The DSP
 * exits idle naturally next iteration if either:
 *   - ARM writes a task field (mirrored via calypso_dsp_write to
 *     s->data[0x0800+offset])
 *   - An IRQ fires (calypso_c54x_interrupt_ex sets s->ifr)
 *   - PC moves outside the range (shouldn't happen while polling)
 *
 * Env vars (default ON) :
 *   CALYPSO_DSP_IDLE_FF=0          disable
 *   CALYPSO_DSP_IDLE_RANGE=lo:hi   override hex PC range
 */
