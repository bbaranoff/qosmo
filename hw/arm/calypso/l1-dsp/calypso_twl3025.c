/*
 * calypso_twl3025.c - TWL3025 (Iota) ABB AFC chip model
 *
 * Minimal TWL3025 (analog baseband) model, enough to propagate the firmware
 * AFC down to the BSP samples in near real time.
 *
 * Silicon chain:
 *   firmware afc_load_dsp() writes dsp_api.db_w->d_afc = dac_value
 *   the DSP reads d_afc and serialises it to the TWL3025 over the TSP
 *   the TWL3025 decodes the register write into the AFC DAC (13-bit, +/-4096)
 *   the DAC drives the 13 MHz VCXO, shifting the baseband frequency
 *
 * In QEMU:
 *   calypso_twl3025_set_afc_dac()   called from the TSP chain with the DAC
 *                                   value written by the firmware
 *   calypso_twl3025_apply_phase()   rotates the BSP samples by
 *                                   dac_value * afc_slope (Hz) at the 270.833
 *                                   kHz baseband rate, with a deterministic
 *                                   offset derived from FN/TN
 *
 * The model is ARMED BY DEFAULT (chip-level behaviour, no env gate).
 * CALYPSO_TWL3025_AFC_HZ=N overrides it with a constant offset (diagnostic:
 * it short-circuits the DAC chain).
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include <math.h>

#include "calypso_twl3025.h"
#include "hw/arm/calypso/calypso_debug.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/* TWL3025 vcxocal constants (osmocom-bb board afcparams.c):
 *   afc_slope              board calibration; 287 on Compal E88, 454 on gta0x
 *   afc_initial_dac_value  -700 on E88, compensating the physical crystal trim
 *   d_afc range            +/-4095 (13-bit signed DAC)
 *
 * In QEMU the osmo-bts samples already arrive at the exact carrier frequency,
 * i.e. as if silicon had already applied the -700 calibration, so the firmware
 * DAC value is read RELATIVE to that baseline:
 *   effective_dac = dac_value - AFC_INITIAL_DAC_VALUE
 * DAC=-700 (ideal calibration) -> effective 0 -> no rotation
 * DAC=0    (+700 LSB above calibration) -> effective +700 -> +200900 Hz
 *
 * [2026-05-30] The modelled VCXO sensitivity must match the osmocom
 * afc_correct() loop:
 *   delta_LSB = (AFC_NORM_FACTOR_GSM * freq_error_Hz) / afc_slope
 *   AFC_NORM_FACTOR_GSM = 2^15/947 = 34.6
 * so unity loop gain requires Hz/LSB = afc_slope / norm_factor. Using the raw
 * slope as Hz/LSB gives a loop gain around 22: the DAC oscillates, the FCCH is
 * rotated by +/-150 kHz, far outside the +/-20 kHz DSP capture range, and FB is
 * never detected. */
#define TWL3025_AFC_NORM_FACTOR_GSM     (32768.0 / 947.0)
#define TWL3025_AFC_SLOPE               287.0  /* compal_e88 board = 287, not 454 (gta0x) */
#define TWL3025_AFC_SLOPE_HZ_PER_LSB    (TWL3025_AFC_SLOPE / TWL3025_AFC_NORM_FACTOR_GSM)
/* ⚠️ The model must boot at DAC=-700, not 0: osmocom afc_reset() starts at
 * afc_initial_dac_value and converges from there. Booting at dac=0 against the
 * -700 baseline means effective=+700 = +200 kHz of spurious offset before the
 * firmware loads its calibration, which puts the FCCH outside the +/-20 kHz DSP
 * capture range so it is never detected. twl3025_lazy_env() therefore inits the
 * DAC to -700 and set_afc_dac() filters the firmware's 0 writes. */
#define TWL3025_AFC_INITIAL_DAC_VALUE   (-700)
#define GSM_SAMPLE_RATE_HZ              270833.0  /* 1 sps GSM baseband */

/* GSM TDMA timing constants (BSP rate of 1 sample per bit):
 *   1 frame = 4.615 ms = 1250 samples
 *   1 slot  = 156.25 symbols, rounded down to 156 samples
 */
#define SAMPLES_PER_FRAME   1250U
#define SAMPLES_PER_SLOT    156U

static struct {
    int16_t  dac_value;     /* current DAC register value (last firmware write) */
    int      force_hz;      /* CALYPSO_TWL3025_AFC_HZ override (diag, default 0 = off) */
    bool     env_loaded;
    int      afc_enabled;   /* CALYPSO_TWL3025_AFC (default 1; 0 disables AFC rotation) */
    uint64_t dac_writes;    /* diag counter */
    uint64_t apply_calls;
} twl;

static void twl3025_lazy_env(void)
{
    if (twl.env_loaded) return;
    twl.env_loaded = true;
    /* @BEQUILLE - TWL3025_AFC_HZ  (CALYPSO_TWL3025_AFC_HZ, VALUE; 0 = inert)
     *   masque  : the closed AFC loop (firmware DAC -> Hz/LSB slope -> sample
     *             rotation). A non-zero value pins a constant offset in Hz and
     *             suppresses convergence.
     *   retirer : never needed - setting 0 (or leaving it unset) is enough, this
     *             is a diagnostic tool. Note that calypso_hack.env sets it to 0
     *             AND exports it: harmless as a value, but present in the
     *             environment.
     */
    const char *h = calypso_getenv("CALYPSO_TWL3025_AFC_HZ");
    twl.force_hz = (h && *h) ? atoi(h) : 0;
    /* Master gate of the AFC loop (RX sample rotation by the VCXO offset).
     * Default ON: the AFC is part of the nominal osmocom path. Opt out with
     * CALYPSO_TWL3025_AFC=0 to deliver raw I/Q. */
    const char *ae = calypso_getenv("CALYPSO_TWL3025_AFC");
    twl.afc_enabled = (ae && *ae == '0') ? 0 : 1;
    /* Start at the calibration point (-700), the "nominal VCXO" in QEMU: the
     * firmware starts its AFC at afc_initial_dac_value and converges. */
    twl.dac_value = TWL3025_AFC_INITIAL_DAC_VALUE;
    fprintf(stderr,
            "[twl3025] chip model armed (slope=%.1f Hz/LSB, AFC=%s, force_hz=%d)\n",
            TWL3025_AFC_SLOPE_HZ_PER_LSB,
            twl.afc_enabled ? "ON(opt-out CALYPSO_TWL3025_AFC=0)" : "OFF",
            twl.force_hz);
}

void calypso_twl3025_set_afc_dac(int16_t dac_value)
{
    twl3025_lazy_env();
    if (dac_value > 4095)  dac_value = 4095;
    if (dac_value < -4096) dac_value = -4096;

    /* Filter the dual-write 0-clobber pattern: the firmware writes d_afc to BOTH
     * dsp_api pages each frame (page A = inherited init 0, page B = the real
     * value). On silicon the TWL3025 register is sticky through TSP
     * serialisation, so the transient 0 never propagates before the real value;
     * this model sees both writes as direct MMIO and would oscillate 0 <-> -700.
     *
     * Heuristic: ignore a dac=0 write when the current value is already set. A
     * legitimate firmware reset goes through afc_reset, which writes
     * afc_initial_dac_value (-700 on Compal E88), not 0. */
    if (dac_value == 0 && twl.dac_value != 0) {
        return;
    }

    if (twl.dac_value != dac_value) {
        twl.dac_writes++;
        /* Throttled log: first 20 writes then every 1000, enough to watch
         * convergence without drowning the AFC log. */
        if (twl.dac_writes <= 20 || (twl.dac_writes % 1000) == 0) {
            fprintf(stderr,
                "[twl3025] DAC %d → %d (%.1f Hz, write #%" PRIu64 ")\n",
                twl.dac_value, dac_value,
                dac_value * TWL3025_AFC_SLOPE_HZ_PER_LSB,
                twl.dac_writes);
        }
        twl.dac_value = dac_value;
    }
}

int16_t calypso_twl3025_get_afc_dac(void)
{
    return twl.dac_value;
}

double calypso_twl3025_get_afc_hz(void)
{
    twl3025_lazy_env();
    if (twl.force_hz != 0) return (double)twl.force_hz;
    /* Effective DAC = firmware write minus the calibration baseline (-700 on
     * E88), so a firmware reset lands on effective 0. */
    int32_t effective_dac = (int32_t)twl.dac_value - TWL3025_AFC_INITIAL_DAC_VALUE;

    /* [2026-08-22] Band-aware VCXO slope. osmocom afc_correct() picks
     * AFC_NORM_FACTOR by band: GSM/850 -> 2^15/947 (34.6), DCS/PCS -> 2^15/1894
     * (17.3, half). Physically the 13 MHz VCXO is multiplied up to the LO (900
     * vs 1800 MHz), so one DAC LSB shifts the baseband twice as far in DCS. For
     * unity loop gain the emulated Hz/LSB must follow: GSM = slope/34.6 = 8.29,
     * DCS = slope/17.3 = 16.6. Hardcoding the GSM value gave loop gain 0.5 in
     * DCS (ARFCN 514): the AFC under-corrected, converged too slowly, and the
     * FBSB reset cut it short, leaving TOA stuck at r39=39 instead of 23.
     * Selected by CALYPSO_TWL3025_AFC_BAND (default GSM; DCS/1800/PCS -> x2). */
    static double hz_lsb = 0.0;
    if (hz_lsb == 0.0) {
        const char *b = calypso_getenv("CALYPSO_TWL3025_AFC_BAND");
        int dcs = (b && (b[0]=='D' || b[0]=='d' || b[0]=='P' || b[0]=='p' ||
                         (b[0]=='1' && b[1]=='8')));   /* DCS / PCS / 1800 */
        hz_lsb = dcs ? (TWL3025_AFC_SLOPE / (TWL3025_AFC_NORM_FACTOR_GSM / 2.0))
                     : TWL3025_AFC_SLOPE_HZ_PER_LSB;
    }
    return (double)effective_dac * hz_lsb;
}

double calypso_twl3025_get_afc_phase_step(void)
{
    twl3025_lazy_env();
    if (!twl.afc_enabled) return 0.0;   /* CALYPSO_TWL3025_AFC=0: no AFC rotation */
    double hz = calypso_twl3025_get_afc_hz();
    if (hz == 0.0) return 0.0;
    /* Phase step per sample = +/-2*pi * freq / fs.
     * [2026-08-22] The sign is +. Once the AFC loop was really closed
     * (apply_phase moved into c54x_bsp_load, where the feeds converge), the
     * negative sign acted as POSITIVE feedback and the DAC ran away from -700 to
     * 4095 (+34 kHz) instead of converging: it does not match the sign of the
     * emulated DSP freq_error measurement. A/B with
     * CALYPSO_TWL3025_AFC_SIGN_OLD=1 to restore the old minus sign. */
    /* [2026-09-20] Sign is MINUS again. The 2026-08-22 flip to plus was measured
     * against a DSP whose FB angle was garbage (ADD/SUB Smem,16, the 0xF4 group
     * and CPL addressing were wrong, see c54x_exe tools/isa_test). With the ISA
     * fixed, plus is POSITIVE feedback on the deterministic replay (df -146 ->
     * -182 -> -456 -> -799 -> -1714 -> -3421 Hz, doubling per FB) and minus
     * converges to |df| < 110 Hz. CALYPSO_TWL3025_AFC_SIGN_PLUS=1 for A/B. */
    static int sign_plus = -1;
    if (sign_plus < 0) sign_plus = calypso_getenv("CALYPSO_TWL3025_AFC_SIGN_PLUS") ? 1 : 0;
    return (sign_plus ? 1.0 : -1.0) * 2.0 * M_PI * hz / GSM_SAMPLE_RATE_HZ;
}

void calypso_twl3025_apply_phase(int16_t *iq_samples, int n_samples,
                                 uint32_t fn, uint8_t tn)
{
    twl3025_lazy_env();
    twl.apply_calls++;

    double step = calypso_twl3025_get_afc_phase_step();
    /* AFC-APPLY probe: confirms this function runs per burst and with which
     * offset. A large offset (e.g. +200900 Hz) applied to every burst pushes the
     * FCCH outside the +/-20 kHz DSP capture range, so FB is never detected and
     * FBSB loops. */
    if (calypso_debug_enabled("AFC-APPLY") &&
        (twl.apply_calls <= 20 || (twl.apply_calls % 2000) == 0)) {
        fprintf(stderr, "[twl3025] AFC-APPLY #%llu hz=%.1f dac=%d step=%.6f "
                "n=%d fn=%u tn=%u\n",
                (unsigned long long)twl.apply_calls,
                calypso_twl3025_get_afc_hz(), twl.dac_value, step,
                n_samples, fn, tn);
        fflush(stderr);
    }
    if (step == 0.0) return;   /* DAC at baseline and no force_hz: no-op */

    /* Absolute sample offset from FN=0,TN=0 is the system reference phase, which
     * keeps the phase continuous across bursts: burst N+1 starts at
     * (N+1) * 1250 + tn * 156, so cos/sin stay coherent. */
    uint64_t sample_offset = (uint64_t)fn * SAMPLES_PER_FRAME
                           + (uint64_t)tn * SAMPLES_PER_SLOT;

    for (int i = 0; i < n_samples; i++) {
        double ph = step * (double)(sample_offset + (uint64_t)i);
        double c = cos(ph), s = sin(ph);
        int16_t I = iq_samples[2 * i];
        int16_t Q = iq_samples[2 * i + 1];
        double new_I = I * c - Q * s;
        double new_Q = I * s + Q * c;
        if (new_I >  32767.0) new_I =  32767.0;
        if (new_I < -32768.0) new_I = -32768.0;
        if (new_Q >  32767.0) new_Q =  32767.0;
        if (new_Q < -32768.0) new_Q = -32768.0;
        iq_samples[2 * i]     = (int16_t)new_I;
        iq_samples[2 * i + 1] = (int16_t)new_Q;
    }
}

void calypso_twl3025_reset(void)
{
    twl.dac_value   = 0;
    twl.dac_writes  = 0;
    twl.apply_calls = 0;
    /* env_loaded kept: the environment is not reloaded on reset (chip state). */
}
