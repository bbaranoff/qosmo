/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * calypso_dsp_pont.h - le pont entre l'ARM sous QEMU et un DSP C54x hors
 * process (c54x_exe --arm).
 *
 * [2026-09-16] Pourquoi un pont. c54x_exe fait tourner la mask-ROM TI seule,
 * en millisecondes ; la layer1 osmocom-bb, elle, est du code ARM et ne tourne
 * que sous QEMU. Pour les coupler sans recompiler le DSP dans QEMU, les deux
 * partagent l'API RAM (la fenetre 0xFFD00000 cote ARM, DARAM 0x0800 cote DSP)
 * par un segment /dev/shm, et se synchronisent trame par trame sur une socket
 * UNIX SOCK_SEQPACKET.
 *
 * Le protocole reproduit EXACTEMENT ce que qosmo-dsp/calypso_trx.c fait en
 * interne a chaque tdma_tick : DMA tick, boot du DSP jusqu'au premier IDLE,
 * interruption TPU-frame si l'IMR l'arme, un budget de c54x_run, et le front
 * « occupe -> IDLE » qui leve l'IRQ API (IRQ 15) cote ARM. Rien de plus, rien
 * de moins : ce qui diverge entre le pont et qosmo-dsp est un bug du pont.
 *
 * Verrouillage en trame : QEMU envoie TICK et ATTEND DONE avant de lever
 * l'IRQ TPU-frame. Le DSP ne tourne donc jamais en roue libre - le protocole
 * ARM/DSP est synchrone (pages d'ecriture/lecture basculees sur l'IRQ trame).
 *
 * Ce fichier est inclus des deux cotes (QEMU et c54x_exe) : il ne depend de
 * rien d'autre que <stdint.h>.
 */
#ifndef CALYPSO_DSP_PONT_H
#define CALYPSO_DSP_PONT_H

#include <stdint.h>

/* Segment partage : shm_open() de ce nom = /dev/shm/calypso_api_ram.
 * Taille = CALYPSO_API_SIZE (64 Ko, 32 K mots) ; cree par c54x_exe, ouvert
 * en lecture/ecriture par QEMU. */
#define CALYPSO_PONT_SHM        "/calypso_api_ram"
/* Taille du segment : la fenetre API vue du DSP, C54X_API_SIZE = 0x2000 mots.
 * [2026-09-17] Le segment N'EST PAS une copie : cote c54x_exe il est mmap
 * MAP_FIXED PAR-DESSUS data[0x0800..0x27FF] du C54xState (data[] est alignee
 * sur une page pour ca), parce que le coeur ecrit sa fenetre API dans data[]
 * et ne recopie dans api_ram que sur certains chemins ; api_ram est alors
 * alias de &data[0x0800]. Cote QEMU, il couvre les 16 premiers Ko de la
 * fenetre ARM de 64 Ko. */
#define CALYPSO_PONT_SHM_BYTES  0x4000
/* Socket de synchronisation, cote c54x_exe = serveur. */
#define CALYPSO_PONT_SOCK       "/tmp/calypso_dsp.sock"
#define CALYPSO_PONT_MAGIC      0x43353458u   /* 'C54X' */

enum CalypsoPontType {
    PONT_HELLO    = 1,  /* QEMU -> DSP : a = mots d'API RAM attendus, b = magic */
    PONT_HELLO_OK = 2,  /* DSP -> QEMU : a = mots d'API RAM servis,   b = magic */
    PONT_RESET    = 3,  /* QEMU -> DSP : l'ARM a ecrit DL_STATUS (a = valeur)   */
    PONT_TICK     = 4,  /* QEMU -> DSP : une trame TDMA, a = fn, b = d_dsp_page */
    PONT_DONE     = 5,  /* DSP -> QEMU : trame jouee, a = drapeaux, b = insns    */
    PONT_BYE      = 6,  /* l'un ou l'autre : fin propre                          */
};

/* Drapeaux de PONT_DONE.a */
#define PONT_DONE_API_IRQ   (1u << 0)   /* front occupe -> IDLE : lever IRQ 15 */
#define PONT_DONE_IDLE      (1u << 1)   /* le DSP est en IDLE apres la trame   */
#define PONT_DONE_INIT      (1u << 2)   /* boot termine (premier IDLE atteint) */
#define PONT_DONE_RUNNING   (1u << 3)   /* dsp->running                         */

typedef struct {
    uint32_t type;
    uint32_t a;
    uint32_t b;
    uint32_t c;   /* [2026-09-17] TICK : tpu_offset (qbits) pour caler la fenetre RX du DSP */
} CalypsoPontMsg;

#endif
