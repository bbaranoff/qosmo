/* SPDX-License-Identifier: GPL-2.0-or-later */

#ifndef HW_ARM_CALYPSO_SOC_H
#define HW_ARM_CALYPSO_SOC_H

#include "hw/sysbus.h"
#include "qom/object.h"
#include "hw/arm/calypso/calypso_inth.h"
#include "hw/arm/calypso/calypso_timer.h"
#include "hw/arm/calypso/calypso_uart.h"
#include "hw/arm/calypso/calypso_spi.h"

#define TYPE_CALYPSO_SOC "calypso-soc"
OBJECT_DECLARE_SIMPLE_TYPE(CalypsoSoCState, CALYPSO_SOC)

#define CALYPSO_SOC_NUM_IRQS  2

struct CalypsoSoCState {

    SysBusDevice parent_obj;

    MemoryRegion iram;

    CalypsoINTHState  inth;
    CalypsoTimerState timer1;
    CalypsoTimerState timer2;
    CalypsoUARTState  uart_modem;
    CalypsoUARTState  uart_irda;
    CalypsoSPIState   spi;

    void *trx;

    qemu_irq cpu_irq;
    qemu_irq cpu_fiq;

    MemoryRegion iram_alias;
    bool iram_at_zero;
    MemoryRegion cntl_iomem;
    uint16_t extra_conf;
    uint16_t cntl_rst;          /* CLKM CNTL_RST (0xFFFFFD04) : bit1 = RESET_DSP */

};

#endif
