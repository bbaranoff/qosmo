/* SPDX-License-Identifier: GPL-2.0-or-later */

#include "qemu/osdep.h"
#include "hw/sysbus.h"
#include "hw/irq.h"
#include "qemu/log.h"
#include "hw/arm/calypso/calypso_spi.h"

#define SPI_REG_SET1     0x00
#define SPI_REG_SET2     0x02
#define SPI_REG_CTRL     0x04
#define SPI_REG_STATUS   0x06
#define SPI_REG_TX_LSB   0x08
#define SPI_REG_TX_MSB   0x0A
#define SPI_REG_RX_LSB   0x0C
#define SPI_REG_RX_MSB   0x0E

/* The bits as the drivers write them (TI spi_drv.h and osmocom-bb
 * calypso/spi.c): CTRL bit 0 = RDWR_ACTIV (read), bit 1 = WR (write);
 * STATUS bit 0 = RE_ST, bit 1 = WE_ST. ABB_ReadRegister() of TI (FreeCalypso
 * Magnetite) waits for RE_ST. */
#define SPI_CTRL_RDWR    (1 << 0)
#define SPI_CTRL_WR      (1 << 1)
#define SPI_STATUS_RE_ST (1 << 0)
#define SPI_STATUS_WE_ST (1 << 1)

#define ABB_PAGEREG      0x01
#define ABB_ONBSTS       0x01      /* VRPCSTS: powered on by the ON button */
#define ABB_ONREFLT      0x10      /* VRPCSTS: ON button still pressed */

/* 16-bit TWL3025 frame, the same for TI (abb_inline.h) and osmocom-bb
 * (twl3025.c): bits 15..6 = data (10 bits), 5..1 = register, 0 = read.
 * The answer comes back in RX_LSB with the data in 15..6. The pages (PAGEREG
 * = 0x0001 page 0, 0x0002 page 1) index abb_regs by page << 5 | reg. */
static uint16_t twl3025_spi_xfer(CalypsoSPIState *s, uint16_t tx)
{
    int rd = tx & 1;
    int reg = (tx >> 1) & 0x1f;
    int data = (tx >> 6) & 0x3ff;
    int idx = (s->page << 5) | reg;

    if (rd) {
        return (s->abb_regs[idx] & 0x3ff) << 6;
    }
    if (reg == ABB_PAGEREG) {
        s->page = (data & 0x2) ? 1 : 0;
        return 0;
    }
    s->abb_regs[idx] = data;
    /* VRPCSIM (page 1, reg 23): the TI SIM driver (sim.c SIM_StartVolt, Iota)
     * writes the SIM regulator mode then expects RSIMRSU (bit 2, "SIM
     * regulator running") on read-back, 5 tries, otherwise
     * SIM_ERR_HARDWARE_FAIL = no card. */
    if (idx == ((1 << 5) | 23)) {
        s->abb_regs[idx] = data | 0x04;
    }
    if (idx == ABB_VRPCDEV) {
        s->abb_regs[ABB_VRPCSTS] = ABB_ONBSTS;
    }
    return 0;
}

static uint64_t calypso_spi_read(void *opaque, hwaddr offset, unsigned size)
{
    CalypsoSPIState *s = CALYPSO_SPI(opaque);

    switch (offset) {
    case SPI_REG_SET1:
        return s->set1;
    case SPI_REG_SET2:
        return s->set2;
    case SPI_REG_CTRL:
        return s->ctrl;
    case SPI_REG_STATUS:
        /* instantaneous transfer: read and write always complete */
        return SPI_STATUS_RE_ST | SPI_STATUS_WE_ST;
    case SPI_REG_TX_LSB:
        return s->tx_lsb;
    case SPI_REG_TX_MSB:
        return s->tx_data;
    case SPI_REG_RX_LSB:
        /* 16-bit frame: the whole answer is in RX_LSB (TI reads
         * (RX_LSB >> 6) & 0x3ff, osmocom likewise) */
        return s->rx_data;
    case SPI_REG_RX_MSB:
        return 0;
    default:
        qemu_log_mask(LOG_UNIMP, "calypso-spi: read at 0x%02x\n",
                       (unsigned)offset);
        return 0;
    }
}

static void calypso_spi_write(void *opaque, hwaddr offset, uint64_t value,
                               unsigned size)
{
    CalypsoSPIState *s = CALYPSO_SPI(opaque);

    switch (offset) {
    case SPI_REG_SET1:
        s->set1 = value & 0xFFFF;
        break;
    case SPI_REG_SET2:
        s->set2 = value & 0xFFFF;
        break;
    case SPI_REG_CTRL:
        s->ctrl = value & 0xFFFF;
        if (value & (SPI_CTRL_RDWR | SPI_CTRL_WR)) {
            s->rx_data = twl3025_spi_xfer(s, s->tx_data);
            qemu_irq_pulse(s->irq);
        }
        break;
    case SPI_REG_STATUS:
        break;
    case SPI_REG_TX_LSB:
        s->tx_lsb = value & 0xFFFF;
        break;
    case SPI_REG_TX_MSB:
        /* a 16-bit frame fits entirely in TX_MSB (TI SPI_WRITE_TX_MSB,
         * osmocom spi_xfer left-aligned) */
        s->tx_data = value & 0xFFFF;
        break;
    case SPI_REG_RX_LSB:
    case SPI_REG_RX_MSB:
        break;
    default:
        qemu_log_mask(LOG_UNIMP, "calypso-spi: write 0x%04x at 0x%02x\n",
                       (unsigned)value, (unsigned)offset);
        break;
    }
}

static const MemoryRegionOps calypso_spi_ops = {
    .read = calypso_spi_read,
    .write = calypso_spi_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
    .impl = { .min_access_size = 2, .max_access_size = 2 },
};

static void calypso_spi_realize(DeviceState *dev, Error **errp)
{
    CalypsoSPIState *s = CALYPSO_SPI(dev);

    memory_region_init_io(&s->iomem, OBJECT(dev), &calypso_spi_ops, s,
                          "calypso-spi", 0x100);
    sysbus_init_mmio(SYS_BUS_DEVICE(dev), &s->iomem);
    sysbus_init_irq(SYS_BUS_DEVICE(dev), &s->irq);
}

static void calypso_spi_reset(DeviceState *dev)
{
    CalypsoSPIState *s = CALYPSO_SPI(dev);

    s->set1 = 0;
    s->set2 = 0;
    s->ctrl = 0;
    s->status = SPI_STATUS_RE_ST | SPI_STATUS_WE_ST;
    s->tx_data = 0;
    s->tx_lsb = 0;
    s->rx_data = 0;
    s->page = 0;
    memset(s->abb_regs, 0, sizeof(s->abb_regs));

    /* powered on by the ON button, button released (no ONREFLT: TI loops in
     * ABB_Read_Status waiting for its release) */
    s->abb_regs[ABB_VRPCSTS] = ABB_ONBSTS;
    s->abb_regs[ABB_ITSTATREG] = 0x00;
}

static void calypso_spi_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->realize = calypso_spi_realize;
    device_class_set_legacy_reset(dc, calypso_spi_reset);
    dc->desc = "Calypso SPI controller + TWL3025 ABB";
}

static const TypeInfo calypso_spi_info = {
    .name          = TYPE_CALYPSO_SPI,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(CalypsoSPIState),
    .class_init    = calypso_spi_class_init,
};

static void calypso_spi_register_types(void)
{
    type_register_static(&calypso_spi_info);
}

type_init(calypso_spi_register_types)
