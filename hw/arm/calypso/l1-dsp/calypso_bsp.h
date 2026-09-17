/*
 * Calypso BSP/RIF DMA — public interface.
 *
 * Faithful path for downlink I/Q samples between sercomm_gate (the QEMU
 * surrogate of the IOTA RF frontend wired through calypso-ipc-device) and the
 * Calypso DSP DARAM. No NDB result hacking — the DSP code itself is
 * expected to find FB/SB and post results in the NDB.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */
#ifndef HW_ARM_CALYPSO_BSP_H
#define HW_ARM_CALYPSO_BSP_H

#include <stdint.h>
#include <stdbool.h>

struct C54xState;

/*
 * Initialise the BSP DMA module. Call once after the C54x has been created.
 *
 * Two env vars control the DMA destination:
 *   CALYPSO_BSP_DARAM_ADDR — word address inside DSP data space (hex/dec)
 *   CALYPSO_BSP_DARAM_LEN  — max number of int16 words to copy per burst
 *
 * When CALYPSO_BSP_DARAM_ADDR is unset/zero the BSP runs in DISCOVERY mode:
 * it logs every received burst but writes nothing into DARAM. This lets the
 * c54x.c FBDET data-read tracer reveal the real buffer location before we
 * lock the address.
 */
void calypso_bsp_init(struct C54xState *dsp);

/*
 * Receive a downlink burst.
 *
 *   tn       — timeslot number (0..7)
 *   fn       — TDMA frame number
 *   iq       — interleaved int16 I,Q,I,Q,... in DSP-native (host) endianness
 *   n_int16  — number of int16 elements in iq[]  (= 2 * n_complex_samples)
 */
void calypso_bsp_rx_burst(uint8_t tn, uint32_t fn,
                          const int16_t *iq, int n_int16);

/*
 * Transmit an uplink burst — symmetric to rx_burst.
 *
 * Reads 148 hard bits from the DSP UL buffer (where the L1 firmware
 * deposits encoded TX data) and fills bits[148]. Returns true if
 * the burst is valid (any non-zero), false otherwise.
 */
bool calypso_bsp_tx_burst(uint8_t tn, uint32_t fn, uint8_t bits[148]);

/* Build a RACH access burst (148 bits) by reading d_rach from NDB and
 * channel-encoding it via libosmocoding (gsm0503_rach_ext_encode).
 * Returns true if a valid RACH was produced, false if d_rach is zero or
 * the encoder failed. Called by calypso_trx.c when ARM L1 commits a
 * d_task_ra (RACH access). */
bool calypso_bsp_tx_rach_burst(uint32_t fn, uint8_t bits[148]);
bool calypso_bsp_send_rach_ra(uint8_t ra, uint8_t bsic, uint32_t fn, uint8_t tn);  /* [PORT LU] RACH UL depuis d_rach */

uint16_t calypso_bsp_get_daram_addr(void);
uint16_t calypso_bsp_get_daram_len(void);
uint8_t  calypso_bsp_get_last_att(void);

/* Send UL burst via UDP to BTS */
void calypso_bsp_send_ul(uint8_t tn, uint32_t fn, const uint8_t bits[148]);

/* Deliver buffered DL bursts when BDLENA windows are available.
 * Called each TDMA frame from calypso_tdma_tick().
 * current_fn is the QEMU virtual FN — only bursts tagged with that FN
 * (per TN) are delivered; stale bursts (fn < current_fn) are dropped,
 * future bursts (fn > current_fn) are kept for later frames. */
void calypso_bsp_deliver_buffered(uint32_t current_fn);

/* ---- Chaine I/Q du DSP (rapatriee du shunt le 2026-09-03) ---------------- */

/* FB-STREAM : sort la paire (I,Q) suivante de l'anneau d'echantillons FCCH
 * decimes alimente par les bursts DL. Consommee par l'intercept de lecture
 * data[0x9213]/[0x9215] de calypso_c54x.c, qui est l'ENTREE du correlateur
 * natif (gate CALYPSO_FB_STREAM). Rend false si l'anneau est vide. */
bool calypso_bsp_fb_stream_next(uint16_t *outI, uint16_t *outQ);

/* Modele integrateur RSSI : a_pm derive de la magnitude moyenne (MAV) reellement
 * mesuree sur le DL, calibre par le modele trf6151 —
 * a_pm = calib_RF(20*log10(MAV/MAV_REF) + RF_REF). Modelise le registre HW de
 * puissance cote ABB que le DSP lit et que l'ADC emule ne fournit pas. */
uint16_t calypso_bsp_rssi_apm(void);

#endif /* HW_ARM_CALYPSO_BSP_H */
