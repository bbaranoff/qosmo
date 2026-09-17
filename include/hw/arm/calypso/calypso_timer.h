/* SPDX-License-Identifier: GPL-2.0-or-later */

#ifndef HW_TIMER_CALYPSO_TIMER_H
#define HW_TIMER_CALYPSO_TIMER_H

#include "hw/sysbus.h"
#include "qom/object.h"
#include "qemu/timer.h"

#define TYPE_CALYPSO_TIMER "calypso-timer"
OBJECT_DECLARE_SIMPLE_TYPE(CalypsoTimerState, CALYPSO_TIMER)

struct CalypsoTimerState {

    SysBusDevice parent_obj;

    MemoryRegion iomem;
    QEMUTimer    *timer;
    qemu_irq     irq;

    uint16_t load;
    uint16_t count;
    uint16_t ctrl;
    uint16_t prescaler;
    int64_t  tick_ns;
    int64_t  epoch_ns;
    bool     running;

    bool     lost_latch_active;
    uint16_t lost_latch_count;
    uint32_t lost_read_phase;
};

void calypso_timer_register_lost(DeviceState *d);

void calypso_timer_lost_frame_tick(uint32_t fn);

#endif
