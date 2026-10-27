/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * calypso_i2c_ctrl.c - the Calypso I2C controller at its real address
 * (0xFFFE2800, firmware calypso/i2c.c), backed by a real QEMU I2C bus.
 *
 * The "calypso-i2c" object of hw/ssi/calypso_i2c.c is mapped at 0xFFFE1800
 * (the RTC address, not the I2C one) and does nothing; it is kept as is (it
 * answers 4 at offset 4). This file implements the ten byte registers of
 * the real controller and the START command, which replays the transaction
 * on the bus (slave address, internal address byte, then the 16-byte FIFO).
 * The only slave on the E88 board is the ST7558 LCD (0x3c), created in
 * calypso_soc.c.
 *
 * Registers (calypso/i2c.c): 0 DEVICE, 1 ADDRESS, 2 DATA_WR, 3 DATA_RD,
 * 4 CMD (bit0 soft reset, bit1 en_clk, bit2 START, bit3 rw_read), 5 CONF_FIFO
 * (len-1), 6 CONF_CLK, 7 CONF_CLK_FUNC_REF, 8 STATUS_FIFO, 9 STATUS_ACTIVITY
 * (bit2 = 1 while the transfer is not finished: here, always finished).
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "hw/sysbus.h"
#include "hw/i2c/i2c.h"
#include "qom/object.h"

#define TYPE_CALYPSO_I2C_CTRL "calypso-i2c-ctrl"
OBJECT_DECLARE_SIMPLE_TYPE(CalypsoI2CCtrlState, CALYPSO_I2C_CTRL)

enum {
    REG_DEVICE = 0, REG_ADDRESS, REG_DATA_WR, REG_DATA_RD, REG_CMD,
    REG_CONF_FIFO, REG_CONF_CLK, REG_CONF_CLK_REF, REG_STATUS_FIFO,
    REG_STATUS_ACTIVITY,
};

#define CMD_SOFT_RESET  (1 << 0)
#define CMD_EN_CLK      (1 << 1)
#define CMD_START       (1 << 2)
#define CMD_RW_READ     (1 << 3)

#define STATUS_ERROR_DATA (1 << 0)
#define STATUS_ERROR_DEV  (1 << 1)
#define STATUS_BUSY       (1 << 2)

#define FIFO_LEN 16

struct CalypsoI2CCtrlState {
    SysBusDevice parent_obj;
    MemoryRegion iomem;
    I2CBus *bus;
    uint8_t device, address, cmd, conf_fifo, conf_clk, conf_clk_ref, status;
    uint8_t fifo[FIFO_LEN];
    int fifo_len;
    uint8_t rd;
};

static void i2c_ctrl_transfer(CalypsoI2CCtrlState *s)
{
    int i;

    s->status &= ~(STATUS_ERROR_DATA | STATUS_ERROR_DEV);
    if (s->cmd & CMD_RW_READ) {
        if (i2c_start_recv(s->bus, s->device) < 0) {
            s->status |= STATUS_ERROR_DEV;
        } else {
            s->rd = i2c_recv(s->bus);
            i2c_end_transfer(s->bus);
        }
    } else if (i2c_start_send(s->bus, s->device) < 0) {
        s->status |= STATUS_ERROR_DEV;
        qemu_log_mask(LOG_GUEST_ERROR, "calypso-i2c: aucun esclave en 0x%02x\n",
                      s->device);
    } else {
        i2c_send(s->bus, s->address);
        for (i = 0; i < s->fifo_len; i++) {
            i2c_send(s->bus, s->fifo[i]);
        }
        i2c_end_transfer(s->bus);
    }
    s->fifo_len = 0;
}

static uint64_t i2c_ctrl_read(void *opaque, hwaddr offset, unsigned size)
{
    CalypsoI2CCtrlState *s = opaque;

    switch (offset) {
    case REG_DEVICE:          return s->device;
    case REG_ADDRESS:         return s->address;
    case REG_DATA_RD:         return s->rd;
    case REG_CMD:             return s->cmd;
    case REG_CONF_FIFO:       return s->conf_fifo;
    case REG_CONF_CLK:        return s->conf_clk;
    case REG_CONF_CLK_REF:    return s->conf_clk_ref;
    case REG_STATUS_FIFO:     return s->fifo_len;
    case REG_STATUS_ACTIVITY: return s->status & ~STATUS_BUSY;  /* always idle */
    default:                  return 0;
    }
}

static void i2c_ctrl_write(void *opaque, hwaddr offset, uint64_t value, unsigned size)
{
    CalypsoI2CCtrlState *s = opaque;
    uint8_t v = value;

    switch (offset) {
    case REG_DEVICE:
        s->device = v & 0x7f;
        break;
    case REG_ADDRESS:
        s->address = v;
        break;
    case REG_DATA_WR:
        if (s->fifo_len < FIFO_LEN) {
            s->fifo[s->fifo_len++] = v;
        } else {
            s->status |= STATUS_ERROR_DATA;
        }
        break;
    case REG_CMD:
        if (v & CMD_SOFT_RESET) {
            s->fifo_len = 0;
            s->status = 0;
            v &= ~CMD_SOFT_RESET;
        }
        s->cmd = v & ~CMD_START;
        if (v & CMD_START) {
            i2c_ctrl_transfer(s);
        }
        break;
    case REG_CONF_FIFO:
        s->conf_fifo = v;
        break;
    case REG_CONF_CLK:
        s->conf_clk = v;
        break;
    case REG_CONF_CLK_REF:
        s->conf_clk_ref = v;
        break;
    default:
        break;
    }
}

static const MemoryRegionOps i2c_ctrl_ops = {
    .read = i2c_ctrl_read,
    .write = i2c_ctrl_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
    .impl = { .min_access_size = 1, .max_access_size = 1 },
};

static void i2c_ctrl_realize(DeviceState *dev, Error **errp)
{
    CalypsoI2CCtrlState *s = CALYPSO_I2C_CTRL(dev);

    s->bus = i2c_init_bus(dev, "i2c");
    memory_region_init_io(&s->iomem, OBJECT(dev), &i2c_ctrl_ops, s,
                          "calypso.i2c", 0x100);
    sysbus_init_mmio(SYS_BUS_DEVICE(dev), &s->iomem);
}

static void i2c_ctrl_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    dc->realize = i2c_ctrl_realize;
    dc->desc = "Calypso I2C controller (0xFFFE2800)";
}

static const TypeInfo i2c_ctrl_info = {
    .name          = TYPE_CALYPSO_I2C_CTRL,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(CalypsoI2CCtrlState),
    .class_init    = i2c_ctrl_class_init,
};

static void i2c_ctrl_register_types(void)
{
    type_register_static(&i2c_ctrl_info);
}

type_init(i2c_ctrl_register_types)
