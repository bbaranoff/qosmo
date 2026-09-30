/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * calypso_gmsk.c - GSM GMSK modulator (45.004): the bursts handed to the DSP,
 * synthetic cell (c54x_exe) and BTS bursts alike (calypso_bsp.c).
 *
 * Differential coding d_i = b_i XOR b_{i-1} (b_{-1} = 1), alpha_i = 1 - 2 d_i,
 * phase(t) = (pi/2) * sum_i alpha_i * q(t - iT), with q the integral of the
 * gaussian pulse BT = 0.3 truncated to 3T. Internally oversampled 16x, then one
 * sample per symbol is returned at iT + decalage*T.
 *
 * [2026-09-17] 148 zero bits give exactly +pi/2 per symbol, i.e. the FCCH: the
 * ROM FB detector locked on that waveform.
 */
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include "calypso_gmsk.h"

#define OS 16          /* internal oversampling factor */
#define L  3           /* pulse length, in symbols */
#define BT 0.3
#define GMSK_MAX_BITS 512   /* longest burst gmsk_moduler() accepts */

static double q_tab[L * OS + 1];   /* q(t) for t in [-1.5T, 1.5T], step T/OS */
static int q_pret;

static double Qf(double x) { return 0.5 * erfc(x / sqrt(2.0)); }

static void q_init(void)
{
    /* g(t) = (1/T) [ Q(2 pi BT (t - T/2) / (T sqrt(ln 2))) - Q(2 pi BT (t + T/2) / (T sqrt(ln 2))) ]
     * q(t) = integral of g from -inf to t, with q(-1.5T) = 0 and q(+1.5T) = 1 (T = 1). */
    double k = 2.0 * M_PI * BT / sqrt(log(2.0));
    double acc = 0.0, dt = 1.0 / OS;
    for (int i = 0; i <= L * OS; i++) {
        double t = -1.5 + i * dt;
        double g = Qf(k * (t - 0.5)) - Qf(k * (t + 0.5));
        q_tab[i] = acc;
        acc += g * dt;
    }
    /* normalise exactly to 1 */
    for (int i = 0; i <= L * OS; i++) q_tab[i] /= acc;
    q_pret = 1;
}

static double q_de(double t)   /* t in symbols, relative to the pulse centre */
{
    if (t <= -1.5) return 0.0;
    if (t >= 1.5) return 1.0;
    double x = (t + 1.5) * OS;
    int i = (int)x; double f = x - i;
    if (i >= L * OS) return 1.0;
    return q_tab[i] * (1 - f) + q_tab[i + 1] * f;
}

void gmsk_moduler(const uint8_t *bits, int n, int amp, double phase0, double decalage, int16_t *iq)
{
    /* [2026-09-20] CELLULE_MSK_DUR=1: the BSP's hard modulator (calypso_bsp.c
     * cos_tab/sin_tab, +-90 degrees per bit, samples on the phase points, full
     * scale), to test on the replay what the BTS bursts look like to the ROM. */
    static int dur = -1;
    if (dur < 0) { const char *e = getenv("CELLULE_MSK_DUR"); dur = (e && *e == '1') ? 1 : 0; }
    if (dur) {
        static const int16_t ct[4] = { 0x7FFE, 0, -0x7FFE, 0 }, st[4] = { 0, 0x7FFE, 0, -0x7FFE };
        int ph = 0;
        for (int i = 0; i < n; i++) { iq[2*i] = ct[ph]; iq[2*i+1] = st[ph]; ph = (ph + (bits[i] ? 3 : 1)) & 3; }
        (void)amp; (void)phase0; (void)decalage;
        return;
    }
    if (!q_pret) q_init();
    if (n > GMSK_MAX_BITS) n = GMSK_MAX_BITS;   /* alpha[] bound; callers pass 148 */
    int prev = 1;                       /* b_{-1} = 1 (45.004 2.2) */
    double alpha[GMSK_MAX_BITS];
    for (int i = 0; i < n; i++) {
        int d = (bits[i] & 1) ^ prev;
        prev = bits[i] & 1;
        alpha[i] = 1.0 - 2.0 * d;
    }
    for (int k = 0; k < n; k++) {
        double t = k + decalage;        /* sampling instant, in symbols */
        double ph = phase0;
        for (int i = 0; i < n; i++) {
            double dt = t - i;
            if (dt <= -1.5) break;      /* later symbols have not started contributing yet */
            ph += (M_PI / 2.0) * alpha[i] * q_de(dt);
        }
        iq[2 * k]     = (int16_t)lrint(amp * cos(ph));
        iq[2 * k + 1] = (int16_t)lrint(amp * sin(ph));
    }
}

void gmsk_elargir(int16_t *iq, int n, double a)
{
    if (a == 0.0 || n <= 0 || n > GMSK_MAX_BITS) return;
    double xi[GMSK_MAX_BITS], xq[GMSK_MAX_BITS];
    for (int k = 0; k < n; k++) { xi[k] = iq[2*k]; xq[k] = iq[2*k+1]; }
    for (int k = 0; k < n; k++) {
        double yi = xi[k], yq = xq[k];
        if (k > 0)     { yi += a * xi[k-1]; yq += a * xq[k-1]; }
        if (k < n - 1) { yi += a * xi[k+1]; yq += a * xq[k+1]; }
        yi /= (1 + 2 * a); yq /= (1 + 2 * a);   /* keep the peak amplitude */
        if (yi > 32767) yi = 32767;
        if (yi < -32768) yi = -32768;
        if (yq > 32767) yq = 32767;
        if (yq < -32768) yq = -32768;
        iq[2*k] = (int16_t)lrint(yi); iq[2*k+1] = (int16_t)lrint(yq);
    }
}

/* [2026-09-30] LA CHAINE DE RECEPTION, PAS UNE BOITE A TROIS COEFFICIENTS.
 * gmsk_elargir() imite le filtre de reception par y = (x[n-1] + x[n] + x[n+1])/3 :
 * trois coefficients egaux, un canal beaucoup plus long et symetrique que ce
 * qu'un vrai Calypso voit (filtre analogique de bande de base, causal, ~100 kHz).
 * Mesure en rejeu : la ROM rate encore ~20 % des SCH, sur la moitie AVANT la
 * sequence d'apprentissage, et ce sont les bits de donnees qui bordent la
 * sequence (c38, c40) qui decident : l'ISI aux bords contamine son estimation
 * de canal. Ici : GMSK BT=0.3 exacte a OS echantillons/symbole, passe-bas
 * Butterworth d'ordre 3 (bilineaire), coupure fc en kHz (symbole = 270.833 kHz),
 * puis decimation a l'instant `decalage` du symbole. Le retard de groupe du
 * filtre est laisse tel quel : la ROM recale son TOA. */
#define OS_F 8
static void butter3(double fc_norm, double b[4], double a[4])
{
    /* Butterworth ordre 3 analogique : (s+1)(s^2+s+1), bilineaire, fs = 1, fc_norm = fc/fs */
    double wc = tan(M_PI * fc_norm);                 /* pre-distorsion */
    /* H(s) = 1 / ((s/wc + 1)((s/wc)^2 + s/wc + 1)) ; on passe par les coefficients en s */
    double k = wc;
    /* denominateur en s : s^3 + 2 wc s^2 + 2 wc^2 s + wc^3 ; numerateur : wc^3 */
    double A3 = 1, A2 = 2 * k, A1 = 2 * k * k, A0 = k * k * k, B0 = k * k * k;
    /* bilineaire s = (1 - z^-1)/(1 + z^-1) (fs normalise a 2 dans tan ci-dessus) */
    double d0 = A3 + A2 + A1 + A0;
    a[0] = 1.0;
    a[1] = (-3 * A3 - A2 + A1 + 3 * A0) / d0;
    a[2] = (3 * A3 - A2 - A1 + 3 * A0) / d0;
    a[3] = (-A3 + A2 - A1 + A0) / d0;
    b[0] = B0 / d0; b[1] = 3 * B0 / d0; b[2] = 3 * B0 / d0; b[3] = B0 / d0;
}
void gmsk_moduler_filtre(const uint8_t *bits, int n, int amp, double phase0, double decalage, double fc_khz, int16_t *iq)
{
    if (!q_pret) q_init();
    if (n > GMSK_MAX_BITS) n = GMSK_MAX_BITS;
    int prev = 1;
    double alpha[GMSK_MAX_BITS];
    for (int i = 0; i < n; i++) { int d = (bits[i] & 1) ^ prev; prev = bits[i] & 1; alpha[i] = 1.0 - 2.0 * d; }
    /* sur-echantillonnage : (n + 3) symboles de trajectoire pour laisser le filtre s'etablir et la queue sortir */
    int N = (n + 3) * OS_F;
    double *xi = malloc(sizeof(double) * N), *xq = malloc(sizeof(double) * N);
    if (!xi || !xq) { free(xi); free(xq); gmsk_moduler(bits, n, amp, phase0, decalage, iq); return; }
    for (int m = 0; m < N; m++) {
        double t = (double)m / OS_F - 1.5;            /* le premier symbole commence a t = 0 */
        double ph = phase0;
        for (int i = 0; i < n; i++) {
            double dt = t - i;
            if (dt <= -1.5) break;
            ph += (M_PI / 2.0) * alpha[i] * q_de(dt);
        }
        xi[m] = cos(ph); xq[m] = sin(ph);
    }
    double b[4], a[4]; butter3(fc_khz / (270.833 * OS_F), b, a);
    double yi[4] = {0}, yq[4] = {0}, ui[4] = {0}, uq[4] = {0};
    for (int m = 0; m < N; m++) {
        ui[0] = xi[m]; uq[0] = xq[m];
        double oi = b[0]*ui[0] + b[1]*ui[1] + b[2]*ui[2] + b[3]*ui[3] - a[1]*yi[1] - a[2]*yi[2] - a[3]*yi[3];
        double oq = b[0]*uq[0] + b[1]*uq[1] + b[2]*uq[2] + b[3]*uq[3] - a[1]*yq[1] - a[2]*yq[2] - a[3]*yq[3];
        yi[0] = oi; yq[0] = oq;
        for (int j = 3; j > 0; j--) { ui[j] = ui[j-1]; uq[j] = uq[j-1]; yi[j] = yi[j-1]; yq[j] = yq[j-1]; }
        xi[m] = oi; xq[m] = oq;                       /* filtre en place */
    }
    for (int k = 0; k < n; k++) {
        double t = k + decalage + 1.5;                /* meme origine que la trajectoire */
        int m = (int)lrint(t * OS_F); if (m < 0) m = 0; if (m >= N) m = N - 1;
        iq[2 * k]     = (int16_t)lrint(amp * xi[m]);
        iq[2 * k + 1] = (int16_t)lrint(amp * xq[m]);
    }
    free(xi); free(xq);
}
