/* SPDX-License-Identifier: GPL-2.0-or-later */
/* Fake air: the ARFCN the phone is tuned to, from QEMU to c54x_exe.
 *
 * The synthesizer (TRF6151, PLL/PWR registers) is programmed by the TPU, hence in
 * QEMU (calypso_tsp.c), while the bursts enter the DSP in c54x_exe (calypso_bsp.c).
 * QEMU writes every programming (frame, position in the frame in qbits, ARFCN) into
 * a ring in shared memory; c54x_exe looks up the ARFCN of the window it is filling
 * and delivers noise away from the cell frequency (CALYPSO_FAUX_AIR=<arfcn>).
 * One file per instance: each instance has its own /dev/shm. */
#ifndef CALYPSO_RF_ARFCN_H
#define CALYPSO_RF_ARFCN_H

#include <stdint.h>

#define CALYPSO_RF_ARFCN_SHM    "/dev/shm/calypso_rf_arfcn"
#define CALYPSO_RF_ARFCN_MAGIC  0x52464132u   /* "RFA2": layout with RX gain */
#define CALYPSO_RF_ARFCN_N      64u

typedef struct {
    uint32_t fn;        /* QEMU frame (the PONT_TICK one) */
    int16_t  qbit;      /* position of the MOVE in the frame (TPU sequencer) */
    int16_t  arfcn;     /* calypso_trf6151_arfcn(), -1 if unknown */
    /* Total programmed RX gain (calypso_trf6151_total_gain_db): c54x_exe does not
     * have the TRF6151 registers (only QEMU receives the TSP), so its a_pm model
     * needs the gain the TI L1 uses to convert the idle-mode PM (l1ctl_pgc,
     * used_il/used_lna); with the reset gain the cell reads at the floor. */
    int16_t  gain_db;
    /* calypso_trf6151_rf_flags(): bit 15 RX mode, bit 14 TX mode, bit 13 UPLINK
     * frequency (ARFCN in bits 0..10). 0: field unused (c54x_exe falls back to
     * `arfcn` alone). */
    int16_t  reserve;
} CalypsoRfProg;

typedef struct {
    uint32_t magic;
    volatile uint32_t widx;                    /* total number of entries written */
    CalypsoRfProg e[CALYPSO_RF_ARFCN_N];
} CalypsoRfArfcnShm;

#endif
