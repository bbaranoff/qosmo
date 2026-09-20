/*
 * calypso_tint0.c -- TINT0 master clock for Calypso GSM virtualization
 *
 * Emulates the C54x DSP Timer 0 as a QEMU virtual timer.
 * On real hardware, Timer 0 runs off the 13 MHz DSP clock divided by
 * (PRD+1)*(TDDR+1) to produce a 4.615 ms TDMA frame tick (TINT0).
 * TINT0 drives the entire Calypso timing: DSP frame processing,
 * TPU sync, ARM frame IRQ, and UART polling.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */
#include "qemu/osdep.h"
#include "qemu/timer.h"
#include "qemu/main-loop.h"
#include "hw/irq.h"
#include "calypso_tint0.h"
#include "calypso_c54x.h"
#include "hw/core/cpu.h"
#include <stdlib.h>  /* getenv */
#include "hw/arm/calypso/calypso_debug.h"

#define TINT0_LOG(fmt, ...) \
    fprintf(stderr, "[tint0] " fmt "\n", ##__VA_ARGS__)

/* calypso_trx.c implements the actual frame work (DSP run, IRQs, UART) */
extern void calypso_tint0_do_tick(uint32_t fn);
/* The orch CLK->BTS is driven from TINT0, at 2x rate via an internal half-tick. */

/* ---- State ---- */
static struct {
    QEMUTimer *timer;
    uint32_t   fn;
    bool       running;
    bool       tpu_en_pending;
} tint0;

/* ---- Timer callback (fires every 4.615ms) ---- */
static void tint0_tick_cb(void *opaque)
{
    tint0.fn = (tint0.fn + 1) % GSM_HYPERFRAME;

    /* In the current Calypso machine calypso_tint0_start() is never called:
     * the frame tick comes from calypso_tdma_tick() in calypso_trx.c. */

    /* No forced page tic-toc here: the DSP itself writes d_dsp_page
     * each frame (PC=0xf321 / 0xf5ec); the trx api hook mirrors the
     * value into ARM space. We let the firmware drive the toggle. */

    /* Delegate frame work to calypso_trx */
    calypso_tint0_do_tick(tint0.fn);

    /* Re-arm the timer, gated by CALYPSO_PCB_TICK_THREADS. When threading is
     * active the PCB spawns a self-pacing tint0 thread, so the QEMUTimer must
     * NOT be re-armed or every frame ticks twice. */
    {
        static int pcb_threaded = -1;
        if (pcb_threaded < 0) {
            const char *e = calypso_getenv("CALYPSO_PCB_TICK_THREADS");
            pcb_threaded = (e && e[0] == '1') ? 1 : 0;
        }
        if (!pcb_threaded && tint0.running) {
            timer_mod_ns(tint0.timer,
                         qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + TINT0_PERIOD_NS);
        }
    }

    /* Kick ARM CPU to process pending IRQs */
qemu_notify_event();
}

/* Public invoker for the PCB tick thread: runs the same body as the QEMUTimer
 * callback. Call it with the BQL held. */
void calypso_tint0_tick_invoke(void);
void calypso_tint0_tick_invoke(void)
{
    tint0_tick_cb(NULL);
}

/* ---- Public API ---- */

void calypso_tint0_start(void)
{
    if (tint0.running) return;

    if (!tint0.timer) {
        tint0.timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, tint0_tick_cb, NULL);
    }

    tint0.running = true;
    /* Do NOT force tint0.fn = 0 here. A real GSM BTS never restarts the frame
     * counter at 0, it only advances; resetting it on every TINT0 start makes
     * the firmware believe it just synchronized to a fresh hyperframe. Whoever
     * owns the master clock seeds fn through calypso_tint0_set_fn() from a
     * network-derived source before calling start; otherwise fn keeps whatever
     * the static struct holds (0 on first boot only). */
    TINT0_LOG("started (period=%.3f ms, IFR bit %d, vec %d) fn=%u",
              TINT0_PERIOD_NS / 1e6, TINT0_IFR_BIT, TINT0_VEC, tint0.fn);
    timer_mod_ns(tint0.timer,
                 qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + TINT0_PERIOD_NS);
}

void calypso_tint0_tpu_en(void)
{
    tint0.tpu_en_pending = true;
}

bool calypso_tint0_tpu_en_pending(void)
{
    return tint0.tpu_en_pending;
}

void calypso_tint0_tpu_en_clear(void)
{
    tint0.tpu_en_pending = false;
}

uint32_t calypso_tint0_fn(void)
{
    return tint0.fn;
}

void calypso_tint0_set_fn(uint32_t fn)
{
    tint0.fn = fn % GSM_HYPERFRAME;
}

bool calypso_tint0_running(void)
{
    return tint0.running;
}
