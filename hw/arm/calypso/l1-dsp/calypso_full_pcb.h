/*
 * calypso_full_pcb.h - Calypso PCB-level threading orchestrator
 *
 * Interface between the standalone components (DSP/BSP/TPU/SIM/IOTA) and the
 * ARM TCG main thread. Wires the IRQs through the osmocom-bb map (irq.h) and
 * provides the shared locks (DARAM, API RAM, MMR).
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */
#ifndef HW_ARM_CALYPSO_FULL_PCB_H
#define HW_ARM_CALYPSO_FULL_PCB_H

#include "qemu/osdep.h"
#include "qemu/thread.h"
#include "hw/irq.h"

/* === IRQ map (mirrors osmocom-bb include/calypso/irq.h) ==================
 * Canonical reference: OsmocomBB firmware. Used by qemu_irq dispatching from
 * device threads back to the ARM (through INTH). */
#define CALYPSO_IRQ_WATCHDOG        0
#define CALYPSO_IRQ_TIMER1          1
#define CALYPSO_IRQ_TIMER2          2
#define CALYPSO_IRQ_TSP_RX          3
#define CALYPSO_IRQ_TPU_FRAME       4   /* TDMA frame tick -> ARM frame_irq */
#define CALYPSO_IRQ_TPU_PAGE        5
#define CALYPSO_IRQ_SIMCARD         6   /* SIM IT_RX/WT/OV -> sim_irq_handler */
#define CALYPSO_IRQ_UART_MODEM      7   /* osmocon L1CTL */
#define CALYPSO_IRQ_KEYPAD_GPIO     8
#define CALYPSO_IRQ_RTC_TIMER       9
#define CALYPSO_IRQ_RTC_ALARM_I2C  10
#define CALYPSO_IRQ_ULPD_GAUGING   11
#define CALYPSO_IRQ_EXTERNAL       12
#define CALYPSO_IRQ_SPI            13
#define CALYPSO_IRQ_DMA            14   /* BSP DMA done -> ARM */
#define CALYPSO_IRQ_API            15   /* DSP<->ARM mailbox done */
#define CALYPSO_IRQ_SIM_DETECT     16
#define CALYPSO_IRQ_EXTERNAL_FIQ   17
#define CALYPSO_IRQ_UART_IRDA      18
#define CALYPSO_IRQ_ULPD_GSM_TIMER 19
#define CALYPSO_IRQ_GEA            20
#define CALYPSO_IRQ_MAX            21

/* === Shared locks (taken and released by the thread entry points) ========
 * Canonical ordering, required to avoid deadlock:
 *   daram_lock < api_ram_lock < sim_lock < bsp_q_lock < tpu_lock
 * Always acquire in that order and release in the reverse one. */
extern QemuMutex calypso_pcb_daram_lock;    /* DARAM 0x0000-0x27FF */
extern QemuMutex calypso_pcb_api_ram_lock;  /* API mailbox 0x0800-0x0FFF */
extern QemuMutex calypso_pcb_sim_lock;      /* SIM controller it/fifo */
extern QemuMutex calypso_pcb_bsp_q_lock;    /* BSP UDP queue */
extern QemuMutex calypso_pcb_tpu_lock;      /* TPU registers + scenarios */

typedef struct CalypsoPcb CalypsoPcb;

/* === Public API ========================================================== */

/* Initialize PCB orchestrator: locks + IRQ routing table.
 * Called once during SoC init (from calypso_soc.c). */
CalypsoPcb *calypso_pcb_init(qemu_irq *inth_inputs);

/* No-op stubs kept for API compat with calypso_soc.c. */
void calypso_pcb_start_threads(CalypsoPcb *pcb);
void calypso_pcb_stop_threads(CalypsoPcb *pcb);

/* IRQ helper : raise/lower a Calypso IRQ. Thread-safe. */
void calypso_pcb_raise_irq(CalypsoPcb *pcb, int irq_nr);
void calypso_pcb_lower_irq(CalypsoPcb *pcb, int irq_nr);

/* === Async log queue =====================================================
 * For the high-frequency log sites (UART IER, TDMA tick, ...) that fire from
 * the ARM TCG main thread, where an inline fprintf stalls TCG on the stdio
 * lock plus the write syscall. This queue defers them to a dedicated drain
 * thread: TCG only enqueues under a short mutex and carries on. */
void calypso_async_log(const char *fmt, ...) __attribute__((format(printf,1,2)));

/* === DARAM access helpers (cross-thread safety) ==========================
 *
 * Wrappers for the sites outside c54x.c that read or write dsp->data[]: BSP IQ
 * burst writes, TRX/FBSB API RAM mailbox mirror, and so on. DSP-side accesses
 * are already locked by data_read/data_write in calypso_c54x.c; unlocked
 * external writes race with them as soon as the DSP runs in its own thread,
 * corrupting DARAM non-deterministically.
 *
 * Usage:
 *   - Single access: calypso_dsp_daram_read(dsp, addr) / _write(dsp, a, v)
 *     wrap lock + access + unlock (implemented in calypso_full_pcb.c).
 *   - Burst (loops of 100+ iterations): take the lock once with the
 *     pcb_daram_lock_* functions and access dsp->data[] directly in between.
 *
 * Cost: an uncontended qemu_mutex is ~20-30ns; at an estimated ~300k ops/sec
 * overall that is under 1% of wall time. */

/* Burst lock helpers for sections that access dsp->data[] in a loop: acquire
 * once, access directly in between, release once.
 *
 * IMPORTANT: never return or break out of the section without releasing. */
void calypso_pcb_daram_lock_acquire(void);
void calypso_pcb_daram_lock_release(void);

/* Single-access helpers wrapping lock + access + unlock.
 * dsp_void is a C54xState*, typed void* to avoid pulling calypso_c54x.h into
 * every .c that includes this header. */
uint16_t calypso_dsp_daram_read(void *dsp_void, uint16_t addr);
void     calypso_dsp_daram_write(void *dsp_void, uint16_t addr, uint16_t val);

#endif /* HW_ARM_CALYPSO_FULL_PCB_H */
