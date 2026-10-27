/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * calypso_dsp_pont.h - the bridge between the ARM under QEMU and an
 * out-of-process C54x DSP (c54x_exe --arm).
 *
 * Why a bridge: c54x_exe runs the TI mask ROM on its own, in milliseconds;
 * the osmocom-bb layer1 is ARM code and only runs under QEMU. To couple them
 * without compiling the DSP into QEMU, both share the API RAM (the
 * 0xFFD00000 window on the ARM side, DARAM 0x0800 on the DSP side) through
 * a /dev/shm segment, and synchronise frame by frame over a UNIX
 * SOCK_SEQPACKET socket.
 *
 * The protocol reproduces EXACTLY what the in-process calypso_trx.c does on
 * every tdma_tick: DMA tick, DSP boot up to the first IDLE, TPU-frame
 * interrupt if the IMR arms it, a c54x_run budget, and the "busy -> IDLE"
 * edge that raises the API IRQ (IRQ 15) on the ARM side. Nothing more,
 * nothing less: any divergence between the bridge and the in-process path
 * is a bridge bug.
 *
 * Frame lock-step: QEMU sends TICK and WAITS for DONE before raising the
 * TPU-frame IRQ. The DSP therefore never free-runs; the ARM/DSP protocol is
 * synchronous (write/read pages swapped on the frame IRQ).
 *
 * This file is included on both sides (QEMU and c54x_exe): it depends on
 * nothing but <stdint.h>.
 */
#ifndef CALYPSO_DSP_PONT_H
#define CALYPSO_DSP_PONT_H

#include <stdint.h>

/* Shared segment: shm_open() of this name = /dev/shm/calypso_api_ram.
 * Size = CALYPSO_API_SIZE (64 KiB, 32 K words); created by c54x_exe, opened
 * read/write by QEMU. */
#define CALYPSO_PONT_SHM        "/calypso_api_ram"
/* Segment size: the API window as seen by the DSP, C54X_API_SIZE = 0x2000
 * words. The segment is NOT a copy: on the c54x_exe side it is mmap'ed
 * MAP_FIXED OVER data[0x0800..0x27FF] of the C54xState (data[] is page
 * aligned for that), because the core writes its API window into data[]
 * and only copies to api_ram on some paths; api_ram then aliases
 * &data[0x0800]. On the QEMU side it covers the first 16 KiB of the 64 KiB
 * ARM window. */
#define CALYPSO_PONT_SHM_BYTES  0x4000
/* Synchronisation socket, c54x_exe side = server. */
#define CALYPSO_PONT_SOCK       "/tmp/calypso_dsp.sock"
#define CALYPSO_PONT_MAGIC      0x43353458u   /* 'C54X' */

enum CalypsoPontType {
    PONT_HELLO    = 1,  /* QEMU -> DSP: a = API RAM words expected, b = magic   */
    PONT_HELLO_OK = 2,  /* DSP -> QEMU: a = API RAM words served,   b = magic   */
    PONT_RESET    = 3,  /* QEMU -> DSP: the ARM wrote DL_STATUS (a = value)     */
    PONT_TICK     = 4,  /* QEMU -> DSP: one TDMA frame, a = fn, b = d_dsp_page  */
    PONT_DONE     = 5,  /* DSP -> QEMU: frame played, a = flags, b = insns      */
    PONT_GO       = 6,  /* QEMU -> DSP: ARM l1_sync done, deliver the burst (2 phases) */
    /* Dedicated channel: a = TN, b = kind (0 SDCCH/4, 1 SDCCH/8, 0xFF
     * released), c = subchannel. QEMU learns it from the firmware's L1CTL
     * stream (calypso_dcch_tap.c) and the DSP needs it to know WHICH timeslot
     * to deliver: the bridge sends it all eight, but it only plays one per
     * tick. Sent just before a TICK, never while waiting for a GO. */
    PONT_DCCH     = 7,
    /* Distinct from GO: a GO arriving after the 2 s timeout of pont.c
     * ("PONT_GO attendu") falls back into the main loop of c54x_exe, which
     * must not read it as BYE. */
    PONT_BYE      = 8,  /* either side: clean end                               */
};

/* PONT_DONE.a flags */
#define PONT_DONE_API_IRQ   (1u << 0)   /* busy -> IDLE edge: raise IRQ 15     */
#define PONT_DONE_IDLE      (1u << 1)   /* the DSP is in IDLE after the frame  */
#define PONT_DONE_INIT      (1u << 2)   /* boot finished (first IDLE reached)  */
#define PONT_DONE_RUNNING   (1u << 3)   /* dsp->running                         */
/* TICK.b: bit 0 = W page announced by the ARM (d_dsp_page & 1), bit 16 = the
 * ARM armed the DSP frame interrupt (TPU_CTRL_DSP_EN, one-shot) since the
 * previous tick. Without this bit the DSP gets no frame interrupt. */
#define CALYPSO_PONT_TICK_IRQ_TRAME (1u << 16)
/* TWO-PHASE TICK (bit 17). The ROM demodulates the burst of frame N and
 * writes the R page in its frame N+1 ISR, THEN reads the W page; the ARM
 * reads that R page in l1_sync(N+1) and posts the W page of N+2. The silicon
 * order within a frame is therefore: DSP ISR (results + dispatch) ->
 * ARM l1_sync -> burst reception. With this bit the DSP plays the ISR up to
 * the arming of the RX window, answers DONE|PONT_DONE_PHASE_A, and waits for
 * PONT_GO (sent when l1_sync is finished) to deliver the burst and finish. */
#define CALYPSO_PONT_TICK_DEUX_PHASES (1u << 17)
#define PONT_DONE_PHASE_A   (1u << 4)   /* ISR played, burst not delivered: waits for GO */

typedef struct {
    uint32_t type;
    uint32_t a;
    uint32_t b;
    uint32_t c;   /* TICK: tpu_offset (qbits) to align the DSP RX window */
} CalypsoPontMsg;

#endif
