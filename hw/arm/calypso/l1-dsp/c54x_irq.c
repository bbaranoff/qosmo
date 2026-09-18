/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * c54x_irq.c — Interruptions : IFR/IMR, IT trame, niveau
 *
 * Extrait de calypso_c54x.c le 2026-09-18 (decoupage par role).
 * Carte des fichiers dans c54x_internal.h.
 */
#include "c54x_internal.h"

void calypso_inth_arm_ack(void);
/* @BEQUILLE — FRAME_IT_LEVEL  (CALYPSO_FRAME_IT_LEVEL, EQ1, defaut OFF)
 *   masque  : la fenetre INTM=0 trop rare du firmware. Re-assert IFR bit12 a CHAQUE
 *             insn tant que vec28 n'a pas vectorise — l'IFR c54x est a latch
 *             d'evenement, il n'a pas de mode "level".
 *   retirer : quand la cadence INTM du firmware suffit a attraper l'IT au vol.
 */
bool frame_it_level_on(void)
{
    static int c = -1;
    if (c < 0) { const char *e = getenv("CALYPSO_FRAME_IT_LEVEL"); c = (e && *e == '1') ? 1 : 0; }
    return c;
}
/* @BEQUILLE — FRAME_IT_PRIO  (CALYPSO_FRAME_IT_PRIO, EQ1, defaut OFF)
 *   masque  : la priorite d'interruption. Force b=12 au lieu de ctz(pend) pour que
 *             la frame passe devant BRINT0/bit5 — sur c54x la priorite est fixee
 *             par le numero de vecteur, elle n'est pas configurable.
 *   retirer : quand BRINT0 et la frame ne se disputent plus la meme fenetre
 *             (livraison BSP a la bonne cadence).
 */
bool frame_it_prio_on(void)
{
    static int c = -1;
    if (c < 0) { const char *e = getenv("CALYPSO_FRAME_IT_PRIO"); c = (e && *e == '1') ? 1 : 0; }
    return c;
}

bool c54x_irq_level_check(C54xState *s)
{
    static int en = -1;
    if (en < 0) { const char *_d = getenv("CALYPSO_DSP"); en = (getenv("CALYPSO_C54X_IRQ_LEVEL") || (_d && !strcmp(_d, "c54x"))) ? 1 : 0; }  /* natif revive */
    if (!en) return false;
    /* LEVEL hold : tant que la frame-IT n a pas ete vectorisee (vec28), garder
     * bit12 pendant dans l IFR -> la prochaine fenetre INTM=0 la prend. */
    if (g_frame_it_level && frame_it_level_on()) {
        s->ifr |= (1u << 12);
    }
    /* ═══════════════════════════════════════════════════════════════════════
     * [2026-08-04] NIVEAU DE INT10n (bit 14) — la ligne DMA.
     *
     * CAL000 §5.1 liste explicitement le SENS de chaque ligne :
     *     INT0n  (level) -> RIF receive        INT8n  (edge) -> TPU frame
     *     INT1n  (level) -> RIF transmit       INT9n  (edge) -> TPU programmable
     *     INT10n (level) -> DMA interrupt      INT7n  (edge) -> CYPHER
     * Le maintien ci-dessus ne couvrait que le bit 12 (frame-IT). Le bit 14
     * etait donc traite en FRONT : `c54x_interrupt_ex` posait IFR une fois, et
     * si INTM valait 1 a cet instant l'evenement etait PERDU.
     *
     * MESURE QUI L'IMPOSE : sur les 15 demandes vec=30 journalisees,
     *     IMR=0x52ed (bit14 = 1, ligne DEMASQUEE)
     *     IFR=0x4000 (bit14 = 1, IT EN ATTENTE)
     *     INTM=1     (masque global pose)  -> 15 fois sur 15
     * et `IRQ-LEVEL take` n'a jamais une seule ligne. Le DSP ne servait donc
     * jamais la fin de DMA, ses files internes debordaient (PEND : 116 ecritures
     * en 0x434e/0x434f) et il levait DSP_ERR_DMA_PEND — le temoin qu'on cherche
     * a supprimer en FOURNISSANT la fonction, pas en le masquant.
     *
     * La source du niveau est IRQ_STATE du canal, que CAL207 §11.3.5 decrit
     * comme « cleared after being read » : la ligne retombe donc quand le
     * firmware lit le registre, exactement comme sur silicium.
     * ═══════════════════════════════════════════════════════════════════════ */
    if (calypso_rhea_dma_irq_level()) {
        s->ifr |= (1u << C54X_IT_DMA_BIT);
    }
    /* [2026-07-22] LEVELCHK-DBG (gated CALYPSO_AR0_DEBUG) : quand IMR!=0 (fenetre
     * armee), pourquoi l'IT frame n'est-elle pas prise ? Tranche INTM vs IPTR vs
     * pend=0. C'est le verrou du mur terminal Frontiere A. */
    {
        static int lcdbg = -1;
        if (lcdbg < 0) lcdbg = calypso_gate("CALYPSO_AR0_DEBUG", 0);
        if (lcdbg && s->imr && s->insn_count > 4000) {   /* skip boot-reset noise, vise go-live */
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
    /* [2026-07-23] LEVELCHK-EMPIRICAL (unconditional, capped) : "IRQ-LEVEL take"
     * never fires in native runs despite INTM-TRANS showing IFR=0x1020/0x1030
     * (bit5=BRINT0 + bit12=frame pending) right at INTM 1->0 (RETE) moments.
     * This traces EVERY early-return path of this function so we can see
     * empirically which gate is blocking dispatch, instead of reasoning about
     * it statically (this session has been burned by that repeatedly). */
    {
        static unsigned _lcn = 0;
        static uint32_t _last_insn = 0xFFFFFFFFu;
        bool _intm = !!(s->st1 & ST1_INTM);
        bool _delay = s->delay_slots != 0;
        uint16_t _iptr = (s->pmst >> PMST_IPTR_SHIFT) & 0x1FF;
        uint16_t _pend = (uint16_t)(s->ifr & s->imr);
        /* [2026-07-23] DEDUP : RPT re-executes the same instruction (same PC,
         * same insn_count) hundreds/thousands of times without advancing --
         * confirmed via s->rpt_active/rpt_count (calypso_c54x.c ~14371: "RPT:
         * after executing an instruction while repeat is active, re-execute
         * the SAME instruction... continue" -- skips insn_count++). That was
         * exhausting our cap on ONE repeat loop. Only log on a NEW insn_count
         * so the cap covers distinct instructions, not RPT spin. */
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
    /* [2026-07-30] LEVELCHK-WINDOW — compter la FENETRE, pas le cas ennuyeux.
     *
     * LEVELCHK-EMPIRICAL est plafonnee a 5000 lignes et les brule toutes sur
     * `BLOCKED:INTM=1` avant insn=14M, alors que la question est l'inverse :
     * QUAND INTM retombe, voit-on l'IT pendante, et la prend-on ? Mesure du
     * 30/07 : `IRQ-LEVEL take` = 0 sur 52M d'insn, pour 739 transitions INTM
     * 1->0 tracees, avec pend=0x1020 (bit5 BRINT0 + bit12 frame) en permanence.
     * La fenetre dure ~1 instruction (1->0 @187431877 puis 0->1 @187431878).
     *
     * Compteurs, pas lignes : le resume dit en un coup d'oeil si la fenetre est
     * seulement VUE par cette fonction. Si w_intm0 reste a 0 alors qu'INTM-TRANS
     * compte des 1->0, c'est que la fonction n'est pas appelee dans la fenetre —
     * probleme d'ORDONNANCEMENT dans la boucle d'execution, pas de garde. */
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
    /* Ne pas vectoriser tant que le ROM n a pas relocalise IPTR (reset=0x1ff ->
     * table en 0xff80 = garbage). Attendre IPTR reloue (typiquement 0x001). */
    if (((s->pmst >> PMST_IPTR_SHIFT) & 0x1FF) == 0x1FF) return false;
    uint16_t pend = (uint16_t)(s->ifr & s->imr);
    if (!pend) return false;
    int b = __builtin_ctz(pend);          /* lowest set bit = highest priority */
    /* PRIO : la frame (bit12/vec28) prime sur les bits plus bas (BRINT0 bit5) qui
     * voleraient la fenetre rare et re-masqueraient INTM. Gate CALYPSO_FRAME_IT_PRIO. */
    if (frame_it_prio_on() && (pend & (1u << 12))) {
        b = 12;
    }
    int vec = b + 16;                     /* C54x: maskable IMR bit b -> vector b+16 */
    /* [2026-09-03] REMAP VEC28 SUPPRIME ici aussi. Ce site remappait `b == 3`
     * (TINT) vers vec 28 des que CALYPSO_DSP=c54x — « allumee sans etre
     * demandee », comme le disait son propre PIEGE. Maintenant que l'IT trame est
     * emise sur 28/12 a la source, bit 3 redevient ce qu'il est (le timer du DSP)
     * et `vec = b + 16` est correct sans exception. */
    c54x_ifr_clear(s, (uint16_t)(1u << b), "vector-level");
    if (b == 12) g_frame_it_level = false;   /* frame-IT vectorisee -> relache le LEVEL hold */
    s->sp--; data_write(s, s->sp, (uint16_t)s->pc);
    /* [2026-07-22] FIX DRIFT SP (racine du storm bootstub) : pousser XPC SEULEMENT
     * en mode etendu (xpc!=0). Le firmware sort l ISR via POPM ST1 + RCD (pop 1w=PC),
     * PAS RETE (pop 2w, path mort cf l.5294). Quand xpc=0 (pas de paging), pousser
     * XPC laisse un mot orphelin JAMAIS depile -> drift SP +1/IT -> SP wrap -> le RET
     * bootstub 0xab38 pop mem[0x5ac8]=0 au lieu de mem[0x5ac7]=retour -> PC=0 storm.
     * Vrai c54x standard = push PC seul. Legacy: CALYPSO_IT_PUSH_XPC_ALWAYS=1. */
    {
        static int always = -1;
        if (always < 0) { const char *e = getenv("CALYPSO_IT_PUSH_XPC_ALWAYS");
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


/* [2026-07-28] GATE UNIFIE DES CORRECTIFS D EMULATION.
 *   CALYPSO_FIXES=FIX_UN,FIX_DEUX   active des correctifs nommes
 *   CALYPSO_FIXES=all               les active tous
 *   (absent)                        aucun — comportement d origine strictement inchange
 *
 * PROTOCOLE : on pose TOUS les correctifs surs derriere ce gate d un coup, on teste
 * SOUS CHARGE MAXIMALE (camp + LU + SMS, pas un simple boot), et DES QU UN CORRECTIF
 * EST CONFIRME on efface **la CONDITION**, pas le correctif : on retire le
 * `if (calypso_fix_enabled("FIX_...")) {` et ses accolades, le code reste et devient
 * inconditionnel. Son nom disparait alors de la liste des gates.
 * Ce gate est un SAS TEMPORAIRE, jamais une option de configuration : une bequille
 * reste, un sas se vide. Ne jamais y laisser vieillir un correctif valide. */
/* ═══════════════════════════════════════════════════════════════════════════
 * [2026-08-04] FIX_F4XX_SRCDST — la famille F4xx/F5xx avait src et dst INVERSES.
 *
 * TI SPRU172C (Mnemonic Instruction Set, mars 2001), pages instruction :
 *     ADD forme 9 : 15..10 = 111101   bit9 = S   bit8 = D   7..5 = 000  SHIFT
 *     LD  forme   : 15..10 = 111101   bit9 = S   bit8 = D   7..5 = 010  SHIFT
 *     (meme layout de champs pour SUB, SFTA, SFTL, NEG, ABS, MACA, *,ASM,*)
 * Soit **bit 9 = SRC, bit 8 = DST** — identique a la famille F0-F3 (OR forme 4),
 * ou le bloc 1 mot avait DEJA la bonne assignation. Le fichier se contredisait
 * donc lui-meme, et c'est la famille F4xx qui avait tort : 22 sites ecrivaient
 * `src = (op>>8)&1, dst = (op>>9)&1`.
 *
 * PORTEE : 22 handlers (ADD/SUB/LD/SFTA/SFTL/NEG/ABS/MACA/…). Le rayon est
 * large, d'ou la gate d'echappement : `CALYPSO_FIX_F4XX_SRCDST=0` restaure a
 * l'identique le comportement d'avant le 04/08, pour isoler une regression sans
 * toucher au code. Defaut 1 : le correctif est conforme a la doc du fondeur.
 *
 * ⚠️ NON VALIDE SOUS CHARGE. Aucun banc ne reunit aujourd'hui « c54x actif » et
 * « LU complet » (shunt_legit pose CALYPSO_DSP_RUN_C54X=0). Effacer la gate
 * seulement quand un parcours camp->LU l'aura exercee.
 * ═══════════════════════════════════════════════════════════════════════════ */
