/*
 * calypso_twl3025.h — TWL3025 ABB AFC propagation API.
 *
 * Minimal model that propagates the firmware AFC (Automatic Frequency
 * Correction) down to the BSP samples. See calypso_twl3025.c for the full
 * chain.
 *
 * Enabled by CALYPSO_TWL3025_AFC=1; CALYPSO_TWL3025_AFC_HZ=N forces a constant
 * offset in Hz for tests.
 */
#ifndef CALYPSO_TWL3025_H
#define CALYPSO_TWL3025_H

#include "qemu/osdep.h"

/* Set the DAC value (TWL3025 register as decoded from the DSP TSP stream).
 * Called when a d_afc write is seen on the QEMU side. */
void calypso_twl3025_set_afc_dac(int16_t dac_value);

/* Read the current DAC (signed, +/-4095). */
int16_t calypso_twl3025_get_afc_dac(void);

/* Get current AFC offset in Hz (DAC * slope). */
double calypso_twl3025_get_afc_hz(void);

/* Phase step per sample (rad) for the sample rotation in the BSP receive
 * path. Returns 0.0 when AFC propagation is disabled (CALYPSO_TWL3025_AFC
 * != 1). Sign convention: a positive VCXO pull shifts the received samples the
 * other way, so the step is the rotation to apply to the raw samples to match
 * the local oscillator. */
double calypso_twl3025_get_afc_phase_step(void);

/* Apply the AFC rotation in place to N I/Q samples (interleaved int16).
 * No-op when AFC is disabled or dac_value == 0.
 *
 * [2026-05-28] The phase of sample i comes from (fn, tn, i) alone, never from
 * an accumulator:
 *   sample_offset = fn * SAMPLES_PER_FRAME + tn * SAMPLES_PER_SLOT
 *   phase(i) = step * (sample_offset + i)
 * That removes drift (each burst recomputes from fn), tolerates bursts that
 * arrive out of order or duplicated, and accounts for the inter-burst TDMA
 * gaps for free, since fn has advanced. */
void calypso_twl3025_apply_phase(int16_t *iq_samples, int n_samples,
                                 uint32_t fn, uint8_t tn);

/* Reset the DAC and the sample counters. Call on a DSP/BSP reset. */
void calypso_twl3025_reset(void);

#endif /* CALYPSO_TWL3025_H */
