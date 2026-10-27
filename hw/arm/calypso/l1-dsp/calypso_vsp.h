/* calypso_vsp.h - voice serial port (VSP) of the Calypso DSP: C54x SP0 to the ABB.
 * See calypso_vsp.c. */
#ifndef CALYPSO_VSP_H
#define CALYPSO_VSP_H

#include <stdbool.h>
#include "calypso_c54x.h"

/* Memory-mapped registers of serial port SP0 (C54x). */
#define VSP_DRR   0x0020   /* receive: microphone sample (ABB -> DSP) */
#define VSP_DXR   0x0021   /* transmit: speaker sample (DSP -> ABB) */
#define VSP_SPC   0x0022   /* control: 0x08 reset, 0xC8 running (ROM 0xbd33/0xbd37) */

/* Voice port interrupt: IMR bit 4, vector 20 (ROM ISR 0x72d3 -> 0xc14a). */
#define VSP_IT_VEC  20
#define VSP_IT_BIT  4

/* Exchange rings with the host (pont/vsp_audio.py): 16-bit PCM, 8 kHz, mono.
 * Header: uint32 written (count of samples written), uint32 size. */
#define VSP_SHM_UL  "/dev/shm/calypso_pcm_ul"   /* microphone -> DSP (written by the host) */
#define VSP_SHM_DL  "/dev/shm/calypso_pcm_dl"   /* DSP -> speaker (written here) */
#define VSP_SHM_N   16000                        /* 2 s */

typedef int (*calypso_vsp_courir_fn)(C54xState *dsp, int budget);

bool calypso_vsp_actif(void);
/* One TDMA frame: delivers the samples due (960 per 26 frames, 36 or 37),
 * each = DRR, interrupt 20, ROM ISR, DXR read-out. Returns the number of
 * instructions executed. No effect if CALYPSO_VSP != 1 or if the ROM has not
 * armed the port (IMR bit 4 and SPC running). */
long calypso_vsp_trame(C54xState *dsp, calypso_vsp_courir_fn courir, uint32_t fn);

#endif
