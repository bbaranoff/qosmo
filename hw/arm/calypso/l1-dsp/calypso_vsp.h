/* calypso_vsp.h - port serie voix (VSP) du DSP Calypso : SP0 du C54x vers l'ABB.
 * Voir calypso_vsp.c. */
#ifndef CALYPSO_VSP_H
#define CALYPSO_VSP_H

#include <stdbool.h>
#include "calypso_c54x.h"

/* Registres memoire du port serie SP0 (C54x). */
#define VSP_DRR   0x0020   /* reception : echantillon micro (ABB -> DSP) */
#define VSP_DXR   0x0021   /* emission  : echantillon haut-parleur (DSP -> ABB) */
#define VSP_SPC   0x0022   /* controle : 0x08 reset, 0xC8 marche (ROM 0xbd33/0xbd37) */

/* IT du port voix : IMR bit 4, vecteur 20 (ISR ROM 0x72d3 -> 0xc14a). */
#define VSP_IT_VEC  20
#define VSP_IT_BIT  4

/* Anneaux d'echange avec l'hote (pont/vsp_audio.py) : PCM 16 bits, 8 kHz, mono.
 * En-tete : uint32 ecrit (compteur d'echantillons ecrits), uint32 taille. */
#define VSP_SHM_UL  "/dev/shm/calypso_pcm_ul"   /* micro -> DSP (ecrit par l'hote) */
#define VSP_SHM_DL  "/dev/shm/calypso_pcm_dl"   /* DSP -> haut-parleur (ecrit ici) */
#define VSP_SHM_N   16000                        /* 2 s */

typedef int (*calypso_vsp_courir_fn)(C54xState *dsp, int budget);

bool calypso_vsp_actif(void);
/* Une trame TDMA : livre les echantillons dus (960 pour 26 trames, 36 ou 37),
 * chacun = DRR, IT 20, ISR de la ROM, relevé de DXR. Rend le nombre
 * d'instructions executees. Sans effet si CALYPSO_VSP != 1 ou si la ROM n'a pas
 * arme le port (IMR bit 4 et SPC en marche). */
long calypso_vsp_trame(C54xState *dsp, calypso_vsp_courir_fn courir, uint32_t fn);

#endif
