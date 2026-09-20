/* SPDX-License-Identifier: GPL-2.0-or-later */

#ifndef HW_INTC_CALYPSO_INTH_H
#define HW_INTC_CALYPSO_INTH_H

#include "hw/sysbus.h"
#include "qom/object.h"

#define TYPE_CALYPSO_INTH "calypso-inth"
OBJECT_DECLARE_SIMPLE_TYPE(CalypsoINTHState, CALYPSO_INTH)

#define CALYPSO_INTH_NUM_IRQS  32

struct CalypsoINTHState {

    SysBusDevice parent_obj;

    MemoryRegion iomem;

    qemu_irq parent_irq;
    qemu_irq parent_fiq;

    uint16_t ilr[CALYPSO_INTH_NUM_IRQS];

    uint16_t ith_v;
    uint16_t fiq_v;
    uint32_t levels;
    uint32_t mask;
    int rr_start;
    uint16_t last_num;          /* last IRQ number handed to the ARM (IRQ_NUM read) */
    uint64_t frame_eoi;         /* IRQ_CTRL end-of-service writes for the TPU frame IRQ */
};

/* Acquittement emis par le DSP emule (calypso_c54x.c). Sans couche 1 C54x,
 * personne ne l'appelle. */
void calypso_inth_arm_ack(void);

/* [2026-09-21] End-of-service count of the TPU frame interrupt (IRQ 4): the
 * firmware's irq() reads IRQ_NUM, runs l1_sync(), then writes IRQ_CTRL bit 0
 * ("new IRQ agreement"). One more count = one l1_sync() finished. The DSP
 * lock-step (calypso_trx.c) waits on it before handing the frame to the DSP,
 * so the ARM reads the R page before the DSP overwrites it, as on silicon. */
uint64_t calypso_inth_frame_eoi(void);
bool calypso_inth_irq_masked(int irq);

#endif
