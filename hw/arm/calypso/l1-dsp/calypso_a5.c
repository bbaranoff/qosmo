/* calypso_a5.c - the A5 coprocessor of the Calypso DSP, on the C54x I/O bus.
 *
 * The mask-ROM does not cipher in software: it hands Kc and COUNT to a
 * coprocessor wired on ports 0x2800..0x2818 and reads the key stream back
 * in the completion interrupt. Without this model the ports are silent, the
 * stream stays at zero and the DSP deciphers nothing. As read in PROM0
 * (tools/tic54x-dis.py, base 0x7000):
 *
 *   0xb12c  programming, at every dedicated frame:
 *           PORTW 0x2800 <- 0                      stop
 *           PORTW 0x2803..0x2806 <- a_kc[0..3]      (NDB 0x0a3b..0x0a3e)
 *           PORTW 0x2807 <- a_a5fn[0] = T3<<5 | T2  (page W word 12, via 0x3fc6)
 *           PORTW 0x2808 <- a_a5fn[1] = T1          (page W word 13, via 0x3fc7)
 *           PORTW 0x2800 <- (d_a5mode << 2) | 0x12, then | 0x0001 (start)
 *           IMR |= 0x0800, PORTR 0x2802: bit 0 = accepted, otherwise retry
 *   0xb19d  completion ISR (IFR bit 11, vector 27, relay 0x726c):
 *           PORTR 0x2801 (trace only), PORTR 0x2809..0x2818 -> 0x3f93..0x3fa2,
 *           0x3f92 |= 0x1000 (stream ready), PORTW 0x2800 <- 0
 *   0x856b  downlink: words 0x3f93..0x3f9a, bit 15 first (T=24, LD TS,B
 *           then SFTA B,1; SACCD BLT inverts the soft bit) -> 114 bits
 *   0x85d1  uplink: words 0x3f9b..0x3fa2 XORed onto the packed burst 0x3f8a
 *
 * Kc arrives reversed (firmware dsp_load_ciph_param:
 * a_kc[0] = key[7] | key[6] << 8 ...), COUNT is already T1/T3/T2. The stream
 * is therefore computed with osmo_a5() on the reconstructed fn.
 *
 * CALYPSO_A5=0 disables the model (the ports become silent again, A/B). */
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
    unsigned long n_recales;   /* COUNTs realigned to last deposit + 1 */
    unsigned long n_dl_bsp;    /* downlink stream taken at the BSP fn */
    uint32_t dernier_fn;       /* fn of the last computed A5/1-2 stream (coverage) */
    unsigned dernier_algo;
} a5;

/* For coverage (c54x_exe pont.c tx_tsp_publier): true if the coprocessor has
 * computed an A5/1 or A5/2 stream, *fn = the frame of that stream. */
bool calypso_a5_dernier(uint32_t *fn)
{
    if (!calypso_a5_on() || !a5.dernier_algo)
        return false;
    *fn = a5.dernier_fn;
    return true;
}

bool calypso_a5_on(void)
{
    static int on = -1;
    if (on < 0) {
        const char *e = getenv("CALYPSO_A5");
        on = !(e && *e == '0');
    }
    return on;
}

/* 114 bits 0/1 -> 8 words, bit 15 of the first word = first bit. */
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
    /* gsm_gsmtime2fn: T1, T2, T3 -> fn */
    uint32_t fn = 51u * 26u * t1 + 51u * (((int)t3 - (int)t2 + 26 * 3) % 26) + t3;
    /* COUNT ONE FRAME LATE: REALIGN TO THE NEXT DEPOSIT.
     * The firmware writes a_a5fn from l1s.next_time (calypso/dsp.c:553): the
     * frame the BSP will deposit at the NEXT tick, i.e. last deposit + 1.
     * When the ROM programs the coprocessor with the a_a5fn of the PREVIOUS
     * task (read in its phase A before the ARM l1_sync has written the next
     * page W, a race of the two-phase stepping), the offset is 0 and the
     * stream would serve the following burst. CALYPSO_A5_RECALE=0 keeps the
     * raw COUNT. */
    {
        static int recale = -1;
        if (recale < 0) { const char *e = getenv("CALYPSO_A5_RECALE"); recale = !(e && *e == '0'); }
        extern unsigned calypso_daram_last_fn;
        /* ON THE SACCH ONLY. At offset 0 the traffic and FACCH frames carry
         * the right COUNT (speech bit-exact, FACCH identical when compared
         * with tools/comparer_parole.py against bursts deciphered outside the
         * DSP) while the SACCH/TF does not: realigning everything would turn
         * speech into noise and break the FACCH. Only the SACCH/TF frame is
         * realigned (fn%26 == 12 on an even TN, 25 on an odd TN). */
        unsigned p26 = fn % 26u;
        if (recale && fn == calypso_daram_last_fn && (p26 == 12u || p26 == 25u)) {
            fn = (fn + 1u) % (2715648u);
            a5.n_recales++;
            t1 = fn / 1326u; t2 = fn % 26u; t3 = fn % 51u;
        }
    }

    /* DOWNLINK STREAM: THE FN OF THE BURST THE BSP DEPOSITS AT THIS TICK. The race
     * described above leaves a_a5fn one or two frames ahead of the burst depending on
     * the run: SDCCH at "offset +1" (right), TCH at "offset +2" (downlink FACCH (UA)
     * fails Fire, RR Assignment Failure). The BSP timing (calypso_bsp_fn_bts_du_tick)
     * is exact for the frame played at the current tick; it is used for the downlink
     * stream when within 3 frames. The uplink stream keeps the firmware COUNT.
     * CALYPSO_A5_DL_BSP=0: use the firmware COUNT for the downlink too. */
    uint32_t fn_dl = fn;
    {
        static int dl_bsp = -1;
        if (dl_bsp < 0) { const char *e = getenv("CALYPSO_A5_DL_BSP"); dl_bsp = !(e && *e == '0'); }
        extern uint32_t calypso_trx_get_fn(void);
        extern int calypso_bsp_fn_bts_du_tick(uint32_t tick, uint32_t *fn);
        uint32_t ft;
        if (dl_bsp && calypso_bsp_fn_bts_du_tick(calypso_trx_get_fn(), &ft)) {
            int32_t d = (int32_t)ft - (int32_t)fn;
            /* CALYPSO_A5_DL_DECAL: offset added to the BSP fn when a_a5fn differs from it (TCH
             * case; on SDCCH a_a5fn == BSP fn and nothing changes). */
            static int dl_decal = 99;
            if (dl_decal == 99) { const char *e = getenv("CALYPSO_A5_DL_DECAL"); dl_decal = (e && *e) ? atoi(e) : 0; }
            if (d >= -3 && d <= 3 && d != 0) { fn_dl = (uint32_t)((int32_t)ft + dl_decal); a5.n_dl_bsp++; }
            extern unsigned calypso_daram_last_fn;
            static unsigned n_sd;
            if (n_sd < 400 && ft != fn) {
                n_sd++;
                if (n_sd <= 400) printf("  [a5-dl] tick=%u a5fn=%u bsp=%u dernier_depot=%u -> flux DL %u (p26 %u)\n",
                                        calypso_trx_get_fn(), fn, ft, calypso_daram_last_fn, fn_dl, fn_dl % 26u);
            }
        }
    }
    ubit_t dl[114], ul[114], dl2[114];
    if (osmo_a5((int)algo, key, fn, dl, ul) < 0) {
        memset(a5.flux, 0, sizeof a5.flux);
    } else {
        if (fn_dl != fn && osmo_a5((int)algo, key, fn_dl, dl2, NULL) == 0) memcpy(dl, dl2, sizeof dl);
        empaqueter(dl, &a5.flux[0]);
        empaqueter(ul, &a5.flux[8]);
        a5.dernier_fn = fn;
        a5.dernier_algo = algo;
    }
    a5.etat |= 0x0001;
    /* On stdout (dsp.log): the stderr of c54x_exe is not read.
     * The first 8 computations, every Kc change, then a sample every 500:
     * fn of the COUNT against the last BTS frame deposited by the BSP
     * (calypso_daram_last_fn). The COUNT targets the frame of the task,
     * deposited at the next tick: an offset other than +1 means a wrong COUNT. */
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
    /* HISTOGRAM OF THE OFFSET BY POSITION IN THE 26-MULTIFRAME.
     * Shows WHICH frames are targeted off by one: count of the offset
     * (-1, 0, +1, +2, other) per fn%26, one line every 2000 operations. */
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
    /* End of computation: the 0xb19d ISR collects the stream. IFR only -- the
     * core vectors at the next instruction (c54x_irq_level_check), never in
     * the middle of the PORTW in progress. */
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
