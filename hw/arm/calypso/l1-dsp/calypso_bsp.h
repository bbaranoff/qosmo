/*
 * Calypso BSP/RIF DMA - public interface.
 *
 * Faithful path for downlink I/Q samples between sercomm_gate (the QEMU
 * surrogate of the IOTA RF frontend wired through calypso-ipc-device) and the
 * Calypso DSP DARAM. No NDB result hacking: the DSP code itself is expected to
 * find FB/SB and post the results in the NDB.
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
 *   CALYPSO_BSP_DARAM_ADDR - word address inside DSP data space (hex/dec)
 *   CALYPSO_BSP_DARAM_LEN  - max number of int16 words to copy per burst
 *
 * With CALYPSO_BSP_DARAM_ADDR unset or zero the BSP runs in DISCOVERY mode: it
 * logs every received burst but writes nothing into DARAM, so the FBDET
 * data-read tracer in c54x_mem.c can reveal the real buffer location before
 * the address is locked down.
 */
void calypso_bsp_init(struct C54xState *dsp);

void calypso_bsp_set_tpu_offset(int qbits);
/* Canal dedie annonce par QEMU (PONT_DCCH) : genre 0 = SDCCH/4, 1 = SDCCH/8,
 * 0xFF (ou tn <= 0) = libere. Les bursts de cet intervalle remplacent ceux de
 * TS0 dans la fenetre livree au DSP. */
void calypso_bsp_set_dedie(int tn, int genre, int ss);
void calypso_bsp_toa_feedback(int toa);
int  calypso_bsp_service(uint32_t current_fn);  /* drains UDP 6702 and delivers the current frame (standalone host) */

/*
 * Receive a downlink burst.
 *
 *   tn       - timeslot number (0..7)
 *   fn       - TDMA frame number
 *   iq       - interleaved int16 I,Q,I,Q,... in DSP-native (host) endianness
 *   n_int16  - number of int16 elements in iq[]  (= 2 * n_complex_samples)
 */
void calypso_bsp_rx_burst(uint8_t tn, uint32_t fn,
                          const int16_t *iq, int n_int16);

/* [2026-09-20] Filler for timeslots 1..7 during the FB search (continuous
 * DMA): the ROM receives the whole 1250-symbol frame, and on a real C0 carrier
 * the other timeslots carry dummy bursts, not silence. The bench registers one
 * GMSK dummy burst (148 samples, 296 int16); the default filler is zeros. */
void calypso_bsp_set_remplissage(const int16_t *iq, int n_int16);

/*
 * Transmit an uplink burst - symmetric to rx_burst.
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
bool calypso_bsp_send_rach_ra(uint8_t ra, uint8_t bsic, uint32_t fn, uint8_t tn);  /* uplink RACH built from d_rach */

uint16_t calypso_bsp_get_daram_addr(void);
uint32_t calypso_bsp_get_last_fn(void);
uint16_t calypso_bsp_get_daram_len(void);

/* Reference probe: compares DARAM against the last bursts the BSP was handed;
 * *age names which one matched best (0 = this frame). -1 if none recorded. */
int calypso_bsp_verif_compare(uint32_t *fn, uint16_t *addr, int *n, int *age);
uint16_t calypso_bsp_verif_last_page(void);
uint8_t  calypso_bsp_get_last_att(void);

/* Send UL burst via UDP to BTS */
void calypso_bsp_send_ul(uint8_t tn, uint32_t fn, const uint8_t bits[148]);

/* Deliver buffered DL bursts when BDLENA windows are available.
 * Called each TDMA frame from calypso_tdma_tick().
 * current_fn is the QEMU virtual FN: only bursts tagged with that FN
 * (per TN) are delivered; stale bursts (fn < current_fn) are dropped,
 * future bursts (fn > current_fn) are kept for later frames. */
void calypso_bsp_deliver_buffered(uint32_t current_fn);

/* ---- DSP I/Q chain (moved out of the shunt on 2026-09-03) --------------- */

/* FB-STREAM: pops the next (I,Q) pair from the ring of decimated FCCH samples
 * fed by the DL bursts. Consumed by the data[0x9213]/[0x9215] read intercept,
 * which is the INPUT of the native correlator (gate CALYPSO_FB_STREAM).
 * Returns false when the ring is empty. */
bool calypso_bsp_fb_stream_next(uint16_t *outI, uint16_t *outQ);

/* RSSI integrator model: a_pm derives from the mean magnitude (MAV) actually
 * measured on the DL, calibrated by the trf6151 model as
 * a_pm = calib_RF(20*log10(MAV/MAV_REF) + RF_REF). It stands in for the ABB
 * power register the DSP reads, which the emulated ADC does not provide. */
uint16_t calypso_bsp_rssi_apm(void);

#endif /* HW_ARM_CALYPSO_BSP_H */
