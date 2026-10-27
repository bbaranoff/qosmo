/*
 * calypso_asm4532.h - model of the ASM4532 antenna switch.
 *
 * Position in the chain (osmocom wiki TypicalCalypsoModemDesign):
 *   antenna <-> ASM4532 <-> [SAW Rx -> TRF6151]  or  [RF3166 -> TX]
 * It connects the antenna EITHER to one of the GSM/DCS/PCS Rx paths,
 * OR to the PA output for the duration of the burst.
 *
 * Control: TSPACT lines, not the serial TSP. Mapping taken as is from
 * osmocom-bb board/compal/rffe_dualband.c (the emulated C123 board):
 *   TRENA    = TSPACT(6)  Transmit Enable (Antenna Switch)  -- ACTIVE LOW
 *   GSM_TXEN = TSPACT(8)  GSM (as opposed to DCS)           -- ACTIVE LOW
 * and the rffe_mode() sequence:
 *   tspact |= TRENA | GSM_TXEN;          (idle: both deasserted)
 *   if TX: tspact &= ~TRENA;             (assertion)
 *          if GSM900: tspact &= ~GSM_TXEN;
 *
 * SCOPE: this block is on the TX / band selection side. It has NO effect
 *    on FB/SB acquisition: with a synthetic downlink, the Rx path is already
 *    assumed connected. Modelling it (a) makes the TX sequence readable,
 *    (b) detects a transmission requested while the antenna is not switched.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */
#ifndef HW_ARM_CALYPSO_ASM4532_H
#define HW_ARM_CALYPSO_ASM4532_H

#include <stdint.h>
#include <stdbool.h>

/* Called from calypso_tsp.c on every tsp_act_update() (TSP_ACT_L/U write),
 * with the COMPLETE state of the 16 TSPACT lines. */
void calypso_asm4532_tspact_update(uint16_t tspact, uint32_t fn);

/* true = the antenna is switched to the PA output (TRENA asserted). */
bool calypso_asm4532_tx_connected(void);

/* true = GSM900 band selected for transmission (GSM_TXEN asserted),
 * false = DCS1800. Only meaningful if tx_connected(). */
bool calypso_asm4532_band_gsm(void);

/* Counters for the barriers and diagnostics. */
uint32_t calypso_asm4532_tx_windows(void);

#endif /* HW_ARM_CALYPSO_ASM4532_H */
