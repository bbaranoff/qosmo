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
        if (yi > 32767) yi = 32767; if (yi < -32768) yi = -32768;
        if (yq > 32767) yq = 32767; if (yq < -32768) yq = -32768;
        iq[2*k] = (int16_t)lrint(yi); iq[2*k+1] = (int16_t)lrint(yq);
    }
}
