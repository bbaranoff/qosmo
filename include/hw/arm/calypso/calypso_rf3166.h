/*
 * calypso_rf3166.h - model of the RF3166 power amplifier.
 *
 * Position in the chain (doc/CHAINE_RF_MATERIELLE.md §3):
 *   TRF6151 --RF--> RF3166 --RF--> ASM4532 --> antenna
 * The wiki says: "amplifies the signal according to the analog level of the
 * APC"; the APC (Automatic Power Correction) is the power envelope produced
 * by the ABB (TWL3025), in ANALOG form. The PA thus has only two inputs of
 * interest: its digital enable, and that analog level.
 *
 * Enable: TSPACT line, taken from osmocom-bb board/compal/rffe_dualband.c:
 *   PA_ENABLE = TSPACT(1)   -- ACTIVE HIGH
 *
 * SCOPE: purely a TX block. It is not in the Rx path at all, so it cannot
 *    explain anything about FB/SB acquisition. What it provides: making the
 *    transmit sequence checkable, and above all the CROSS-CHECK with the
 *    antenna switch: a PA active while the antenna is not switched to it is
 *    a fault.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */
#ifndef HW_ARM_CALYPSO_RF3166_H
#define HW_ARM_CALYPSO_RF3166_H

#include <stdint.h>
#include <stdbool.h>

/* Called from calypso_tsp.c on every tsp_act_update(), with the COMPLETE
 * state of the 16 TSPACT lines. */
void calypso_rf3166_tspact_update(uint16_t tspact, uint32_t fn);

/* APC level, to be called by the ABB model once it produces it.
 * Unit: that of the ABB APC DAC (0..255), 0 = no transmission.
 * As long as nobody calls it, the level stays UNKNOWN and out_dbm() says so. */
void calypso_rf3166_set_apc(uint8_t apc_level);

/* true = PA_ENABLE asserted. */
bool calypso_rf3166_enabled(void);

/* Estimated output power, in dBm. Returns INT32_MIN if the APC has never
 * been set (= unknown) or if the PA is off. A MODEL, not a measurement:
 * linear ramp 0..255 -> 0..33 dBm, to be replaced by a real calibration
 * table once the UL is measured. */
int32_t calypso_rf3166_out_dbm(void);

/* Number of times the PA was enabled while the antenna was NOT switched to
 * it (cross-check with calypso_asm4532). Must stay at 0. */
uint32_t calypso_rf3166_faults(void);

#endif /* HW_ARM_CALYPSO_RF3166_H */
