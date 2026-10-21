/* SPDX-License-Identifier: GPL-2.0-or-later */
/* [2026-10-21] FAUX AIR : l'ARFCN sur lequel le telephone est accorde, de QEMU vers c54x_exe.
 *
 * Le synthetiseur (TRF6151, registres PLL/PWR) est programme par le TPU, donc dans QEMU
 * (calypso_tsp.c), alors que les bursts entrent dans le DSP dans c54x_exe (calypso_bsp.c).
 * QEMU ecrit chaque programmation (trame, position dans la trame en qbits, ARFCN) dans un
 * anneau en memoire partagee ; c54x_exe y retrouve l'ARFCN de la fenetre qu'il remplit et
 * livre du bruit hors de la frequence de la cellule (CALYPSO_FAUX_AIR=<arfcn>).
 * Un fichier par instance : chaque instance a son propre /dev/shm. */
#ifndef CALYPSO_RF_ARFCN_H
#define CALYPSO_RF_ARFCN_H

#include <stdint.h>

#define CALYPSO_RF_ARFCN_SHM    "/dev/shm/calypso_rf_arfcn"
#define CALYPSO_RF_ARFCN_MAGIC  0x52464132u   /* "RFA2" : + gain RX (2026-10-21) */
#define CALYPSO_RF_ARFCN_N      64u

typedef struct {
    uint32_t fn;        /* trame QEMU (celle du PONT_TICK) */
    int16_t  qbit;      /* position du MOVE dans la trame (sequenceur TPU) */
    int16_t  arfcn;     /* calypso_trf6151_arfcn(), -1 si inconnu */
    /* [2026-10-21] gain RX total programme (calypso_trf6151_total_gain_db) : c54x_exe n'a pas les registres
     * du TRF6151 (seul QEMU recoit le TSP), son modele de a_pm calculait avec le gain de reset et la L1 TI,
     * qui convertit la PM de veille avec SON gain (l1ctl_pgc, used_il/used_lna), lisait la cellule au
     * plancher : (1 54) -> (1 8), antenne perdue en veille. */
    int16_t  gain_db;
    int16_t  reserve;
} CalypsoRfProg;

typedef struct {
    uint32_t magic;
    volatile uint32_t widx;                    /* nombre total d'entrees ecrites */
    CalypsoRfProg e[CALYPSO_RF_ARFCN_N];
} CalypsoRfArfcnShm;

#endif
