/* SPDX-License-Identifier: GPL-2.0-or-later */

#ifndef HW_SSI_CALYPSO_SPI_H
#define HW_SSI_CALYPSO_SPI_H

#include "hw/sysbus.h"
#include "qom/object.h"

#define TYPE_CALYPSO_SPI "calypso-spi"
OBJECT_DECLARE_SIMPLE_TYPE(CalypsoSPIState, CALYPSO_SPI)

struct CalypsoSPIState {

    SysBusDevice parent_obj;

    MemoryRegion iomem;
    qemu_irq     irq;

    uint16_t set1;
    uint16_t set2;
    uint16_t ctrl;
    uint16_t status;
    uint16_t tx_data;
    uint16_t rx_data;

    uint16_t abb_regs[256];
};

#define ABB_VRPCDEV    0x01
#define ABB_VRPCSTS    0x02
#define ABB_VBUCTRL    0x03
#define ABB_VBDR1      0x04
#define ABB_TOGBR1     0x09
#define ABB_TOGBR2     0x0A
#define ABB_AUXLED     0x17
#define ABB_ITSTATREG  0x1B

#define SPI_STATUS_RE        (1 << 1)

#define SPI_STATUS_TX_READY  SPI_STATUS_RE
#define SPI_STATUS_RX_READY  SPI_STATUS_RE

#endif
