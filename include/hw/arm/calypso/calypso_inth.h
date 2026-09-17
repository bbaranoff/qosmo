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
};

/* Acquittement emis par le DSP emule (calypso_c54x.c). Sans couche 1 C54x,
 * personne ne l'appelle. */
void calypso_inth_arm_ack(void);

#endif
