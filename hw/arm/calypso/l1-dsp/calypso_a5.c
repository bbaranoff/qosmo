/* calypso_a5.c - le coprocesseur A5 du DSP Calypso, sur le bus d'E/S du C54x.
 *
 * [2026-09-23] POURQUOI. La mask-ROM ne chiffre pas en logiciel : elle confie
 * Kc et COUNT a un coprocesseur cable sur les ports 0x2800..0x2818 et relit le
 * flux de cle a l'interruption de fin. Rien ne repondait sur ces ports, PORTR
 * laissait la cellule telle quelle : le flux restait a zero et le DSP ne
 * dechiffrait RIEN. D'ou le dechiffrement du descendant par le pont (pensé
 * pour le shunt gr-gsm) et la SACCH du TCH jamais decodee (LOS apres ~20 s
 * d'appel). Lu dans PROM0 (tools/tic54x-dis.py, base 0x7000) :
 *
 *   0xb12c  programmation, a chaque trame dediee :
 *           PORTW 0x2800 <- 0                      arret
 *           PORTW 0x2803..0x2806 <- a_kc[0..3]      (NDB 0x0a3b..0x0a3e)
 *           PORTW 0x2807 <- a_a5fn[0] = T3<<5 | T2  (page W mot 12, via 0x3fc6)
 *           PORTW 0x2808 <- a_a5fn[1] = T1          (page W mot 13, via 0x3fc7)
 *           PORTW 0x2800 <- (d_a5mode << 2) | 0x12, puis | 0x0001 (depart)
 *           IMR |= 0x0800, PORTR 0x2802 : bit 0 = accepte, sinon nouvel essai
 *   0xb19d  ISR de fin (IFR bit 11, vecteur 27, relais 0x726c) :
 *           PORTR 0x2801 (trace seulement), PORTR 0x2809..0x2818 -> 0x3f93..0x3fa2,
 *           0x3f92 |= 0x1000 (flux pret), PORTW 0x2800 <- 0
 *   0x856b  descendant : mots 0x3f93..0x3f9a, bit 15 d'abord (T=24, LD TS,B
 *           puis SFTA B,1 ; SACCD BLT inverse le bit souple) -> 114 bits
 *   0x85d1  montant : mots 0x3f9b..0x3fa2 XORes sur la rafale empaquetee 0x3f8a
 *
 * Le Kc arrive a l'envers (firmware dsp_load_ciph_param :
 * a_kc[0] = key[7] | key[6] << 8 ...), COUNT est deja T1/T3/T2. On calcule
 * donc avec osmo_a5() sur le fn reconstitue.
 *
 * CALYPSO_A5=0 coupe le modele (les ports redeviennent muets, A/B). */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include <osmocom/gsm/a5.h>
#include "calypso_c54x.h"
#include "calypso_a5.h"

#define A5_PA_CTRL    0x2800
#define A5_PA_TRACE   0x2801
#define A5_PA_ETAT    0x2802
#define A5_PA_KC0     0x2803
#define A5_PA_CNT0    0x2807
#define A5_PA_CNT1    0x2808
#define A5_PA_FLUX0   0x2809
#define A5_N_FLUX     16

#define A5_IT_VEC     27
#define A5_IT_BIT     11

static struct {
    uint16_t ctrl, kc[4], cnt[2], etat;
    uint16_t flux[A5_N_FLUX];
    unsigned long n_calculs;
    unsigned long n_recales;   /* COUNT recales sur dernier depot + 1 */
} a5;

bool calypso_a5_on(void)
{
    static int on = -1;
    if (on < 0) {
        const char *e = getenv("CALYPSO_A5");
        on = !(e && *e == '0');
    }
    return on;
}

/* 114 bits 0/1 -> 8 mots, bit 15 du premier mot = premier bit. */
static void empaqueter(const ubit_t *b, uint16_t *w)
{
    memset(w, 0, 8 * sizeof(*w));
    for (int i = 0; i < 114; i++)
        if (b[i])
            w[i / 16] |= (uint16_t)(0x8000u >> (i % 16));
}

static void a5_calculer(C54xState *s, unsigned algo)
{
    uint8_t key[8];
    for (int i = 0; i < 4; i++) {
        key[7 - 2 * i] = (uint8_t)(a5.kc[i] & 0xff);
        key[6 - 2 * i] = (uint8_t)(a5.kc[i] >> 8);
    }
    unsigned t2 = a5.cnt[0] & 0x1f, t3 = (a5.cnt[0] >> 5) & 0x3f, t1 = a5.cnt[1] & 0x7ff;
    /* gsm_gsmtime2fn : T1, T2, T3 -> fn */
    uint32_t fn = 51u * 26u * t1 + 51u * (((int)t3 - (int)t2 + 26 * 3) % 26) + t3;
    /* [2026-09-23] COUNT D'UNE TRAME EN RETARD : RECALE SUR LE DEPOT SUIVANT.
     * Le firmware ecrit a_a5fn depuis l1s.next_time (calypso/dsp.c:553) : la
     * trame que le BSP deposera au tick SUIVANT, soit dernier depot + 1 --
     * l'ecart releve sur tout l'appel reussi de 20:22. Aux runs de 20:32, 21:08
     * et 21:24, l'ecart tombe a 0 sur 100 % du TCH (et sur une partie du
     * SDCCH) : la ROM a programme le coprocesseur avec l'a_a5fn de la tache
     * PRECEDENTE, lu dans sa phase A avant que le l1_sync de l'ARM ait ecrit la
     * page W suivante -- une course du pas-a-pas en deux phases, qui varie d'un
     * run a l'autre. Le flux servait alors au burst d'apres : SACCH/TF ratee a
     * chaque bloc (LOS ~15 s apres le decroche), parole a 15-90 erreurs par
     * trame, SACCH/8 perdue un bloc sur deux. Hors DSP, les memes bursts se
     * dechiffrent avec le COUNT de leur propre fn (34/52 blocs SACCH/TF, run de
     * 21:08, contre 0 a +-1). CALYPSO_A5_RECALE=0 rend le COUNT brut. */
    {
        static int recale = -1;
        if (recale < 0) { const char *e = getenv("CALYPSO_A5_RECALE"); recale = !(e && *e == '0'); }
        extern unsigned calypso_daram_last_fn;
        /* [2026-09-23, 21:40] SUR LA SACCH SEULEMENT. Mesure par
         * tools/comparer_parole.py (bursts du BSP dechiffres et decodes hors
         * DSP, contre les trames rendues par la ROM) : au regime +1, parole
         * exacte au bit pres (240/241) et FACCH identiques (15/15) ; au regime
         * 0 recale partout, la parole devient du bruit (~95 bits faux sur 260,
         * 537/734) et les FACCH echouent -- le son hache et le raccroche perdu
         * de 21:29. Sans recalage au regime 0 (21:24), c'est la SACCH/TF qui
         * echoue. Le trafic a donc le bon COUNT a l'ecart 0, la SACCH non : on
         * ne recale que la trame SACCH/TF (fn%26 == 12 sur un TN pair, 25 sur
         * un TN impair). */
        unsigned p26 = fn % 26u;
        if (recale && fn == calypso_daram_last_fn && (p26 == 12u || p26 == 25u)) {
            fn = (fn + 1u) % (2715648u);
            a5.n_recales++;
            t1 = fn / 1326u; t2 = fn % 26u; t3 = fn % 51u;
        }
    }

    ubit_t dl[114], ul[114];
    if (osmo_a5((int)algo, key, fn, dl, ul) < 0) {
        memset(a5.flux, 0, sizeof a5.flux);
    } else {
        empaqueter(dl, &a5.flux[0]);
        empaqueter(ul, &a5.flux[8]);
    }
    a5.etat |= 0x0001;
    /* [2026-09-23] Sur stdout (dsp.log) : le stderr de c54x_exe n'est pas lu.
     * Les 8 premiers calculs, chaque changement de Kc, puis un releve toutes
     * les 500 : fn du COUNT contre la derniere trame BTS deposee par le BSP
     * (calypso_daram_last_fn). Le COUNT vise la trame de la tache, deposee au
     * tick suivant : un ecart qui s'ecarte de +1 dit un COUNT faux. */
    {
        static uint8_t kc_prec[8]; static int kc_vu;
        bool change = !kc_vu || memcmp(kc_prec, key, 8) != 0;
        if (a5.n_calculs < 8 || change || (a5.n_calculs % 500) == 0) {
            extern unsigned calypso_daram_last_fn;
            printf("  [a5] #%lu A5/%u fn=%u (T1=%u T2=%u T3=%u) Kc=%02x%02x%02x%02x%02x%02x%02x%02x%s "
                   "| dernier depot BSP fn=%u (ecart %+d)\n", a5.n_calculs, algo, fn, t1, t2, t3,
                   key[0], key[1], key[2], key[3], key[4], key[5], key[6], key[7],
                   change ? " (NOUVEAU Kc)" : "", calypso_daram_last_fn,
                   (int)(fn - calypso_daram_last_fn));
            fflush(stdout);
        }
        memcpy(kc_prec, key, 8); kc_vu = 1;
    }
    /* [2026-09-23] HISTOGRAMME DE L'ECART PAR POSITION DANS LA 26-MULTITRAME.
     * Sur le TCH, la SACCH/TF (fn%26 == 12) echoue dans le DSP a chaque bloc,
     * alors que les memes bursts, dechiffres hors DSP avec le COUNT de leur
     * propre fn, se decodent (34/52, run de 21:08). Et l'ecart releve toutes
     * les 500 passe de +1 (attendu) a +0 pendant les appels ratés, jamais sur
     * l'appel reussi de 20:22. Pour savoir QUELLES trames sont visees a cote :
     * compte de l'ecart (-1, 0, +1, +2, autre) par fn%26, une ligne toutes les
     * 2000 operations. */
    {
        extern unsigned calypso_daram_last_fn;
        static unsigned long hist[26][5];
        int e = (int)(fn - calypso_daram_last_fn);
        hist[fn % 26u][e >= -1 && e <= 2 ? e + 1 : 4]++;
        if (a5.n_calculs && (a5.n_calculs % 2000) == 0) {
            printf("  [a5-hist] #%lu recales=%lu ecart(-1/0/+1/+2/autre) par fn%%26 :", a5.n_calculs, a5.n_recales);
            for (int p = 0; p < 26; p++)
                if (hist[p][0] + hist[p][1] + hist[p][2] + hist[p][3] + hist[p][4])
                    printf(" %d:%lu/%lu/%lu/%lu/%lu", p, hist[p][0], hist[p][1], hist[p][2], hist[p][3], hist[p][4]);
            printf("\n");
            fflush(stdout);
            memset(hist, 0, sizeof hist);
        }
    }
    a5.n_calculs++;
    /* Fin de calcul : l'ISR 0xb19d releve le flux. IFR seul -- le coeur
     * vectorise a l'instruction suivante (c54x_irq_level_check), jamais au
     * milieu du PORTW en cours. */
    if (s)
        s->ifr |= (uint16_t)(1u << A5_IT_BIT);
}

bool calypso_a5_portw(C54xState *s, uint16_t pa, uint16_t val)
{
    if (!calypso_a5_on() || pa < A5_PA_CTRL || pa > A5_PA_CNT1)
        return false;
    switch (pa) {
    case A5_PA_CTRL: {
        uint16_t avant = a5.ctrl;
        a5.ctrl = val;
        if (val == 0) {
            a5.etat = 0;
        } else if ((val & 0x0001) && !(avant & 0x0001)) {
            unsigned algo = (val >> 2) & 3;
            if (algo == 1 || algo == 2) {
                a5_calculer(s, algo);
            } else {
                memset(a5.flux, 0, sizeof a5.flux);
                a5.etat |= 0x0001;
                if (s) s->ifr |= (uint16_t)(1u << A5_IT_BIT);
            }
        }
        return true;
    }
    case A5_PA_CNT0: a5.cnt[0] = val; return true;
    case A5_PA_CNT1: a5.cnt[1] = val; return true;
    case A5_PA_TRACE:
    case A5_PA_ETAT:
        return true;
    default:
        a5.kc[pa - A5_PA_KC0] = val;
        return true;
    }
}

bool calypso_a5_portr(C54xState *s, uint16_t pa, uint16_t *out)
{
    (void)s;
    if (!calypso_a5_on() || pa < A5_PA_CTRL || pa >= A5_PA_FLUX0 + A5_N_FLUX)
        return false;
    if (pa >= A5_PA_FLUX0)
        *out = a5.flux[pa - A5_PA_FLUX0];
    else if (pa == A5_PA_ETAT)
        *out = a5.etat;
    else if (pa == A5_PA_CTRL)
        *out = a5.ctrl;
    else if (pa == A5_PA_TRACE)
        *out = 0;
    else if (pa <= A5_PA_KC0 + 3)
        *out = a5.kc[pa - A5_PA_KC0];
    else
        *out = a5.cnt[pa - A5_PA_CNT0];
    return true;
}
