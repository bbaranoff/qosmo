/* SPDX-License-Identifier: GPL-2.0-or-later */

#include "qemu/osdep.h"
#include "hw/irq.h"
#include "hw/sysbus.h"
#include "hw/arm/calypso/calypso_inth.h"

static void calypso_inth_update(CalypsoINTHState *s)
{
    uint32_t active = s->levels & ~s->mask;
    int best_irq = -1, best_irq_prio = 0x7F;
    int best_fiq = -1, best_fiq_prio = 0x7F;

    for (int j = 0; j < CALYPSO_INTH_NUM_IRQS; j++) {
        int i = (s->rr_start + j) % CALYPSO_INTH_NUM_IRQS;
        if (!(active & (1u << i))) continue;
        int prio = s->ilr[i] & 0x1F;
        int is_fiq = (s->ilr[i] >> 8) & 1;
        if (is_fiq) {
            if (prio < best_fiq_prio) { best_fiq_prio = prio; best_fiq = i; }
        } else {
            if (prio < best_irq_prio) { best_irq_prio = prio; best_irq = i; }
        }
    }

    if (best_irq >= 0) {
        s->ith_v = best_irq;
        {
            static int dbg = -1, nirq;
            if (dbg < 0) { const char *e = getenv("CALYPSO_INTH_DEBUG"); dbg = (e && *e == '1'); }
            if (dbg && (best_irq == 8 || nirq < 20) && (nirq++ < 2000 || nirq % 500 == 0)) {
                fprintf(stderr, "[inth] IRQ %d levee (prio %d, mask 0x%08x, levels 0x%08x, n=%d)\n",
                        best_irq, best_irq_prio, s->mask, s->levels, nirq);
            }
        }
        qemu_irq_raise(s->parent_irq);
    } else {
        if (best_fiq < 0) s->ith_v = 0;
        qemu_irq_lower(s->parent_irq);
    }

    if (best_fiq >= 0) {
        s->fiq_v = best_fiq;
        qemu_irq_raise(s->parent_fiq);
    } else {
        qemu_irq_lower(s->parent_fiq);
    }
}

static void calypso_inth_set_irq(void *opaque, int irq, int level)
{
    CalypsoINTHState *s = CALYPSO_INTH(opaque);

    if (level) {
        s->levels |= (1u << irq);
    } else if ((s->ilr[irq] & 0x2) && (irq == 4 || irq == 5 || irq == 15)) {
        /* EDGE-CONFIGURED SOURCE (ILR bit 1, irq_config(..., edge=1)): the
         * edge is LATCHED in IT_REG until IRQ_NUM is read (below, for 4/5/15)
         * or until the software clears it. The line falling must not clear the
         * bit: the TPU frame IRQ (IRQ 4) is a 1 ms pulse (calypso_trx.c
         * FRAME_IRQ_PULSE_NS) and, if the ARM is still inside a long handler
         * (UART/L1CTL during speech), the IRQ would be LOST. The firmware does
         * not detect a lost TDMA frame: its frame counter would slip by one for
         * the rest of the channel (a_a5fn = fn-1: speech deciphered with the
         * wrong stream, wrong FACCH, lost SACCH, LOS ~15 s later). */
    } else {
        s->levels &= ~(1u << irq);
    }

    calypso_inth_update(s);
}

static uint64_t calypso_inth_read(void *opaque, hwaddr offset, unsigned size)
{
    CalypsoINTHState *s = CALYPSO_INTH(opaque);

    switch (offset) {
    case 0x00:
        return s->levels & 0xFFFF;
    case 0x02:
        return (s->levels >> 16) & 0xFFFF;
    case 0x08:
        return s->mask & 0xFFFF;
    case 0x0a:
        return (s->mask >> 16) & 0xFFFF;
    case 0x10:
    case 0x80:
    {
        uint16_t num = s->ith_v;
        s->last_num = num;

        /* An EDGE-configured source (ILR bit 1: the jiffies TIMER2, which never lowers its
         * line) is acknowledged by the IRQ_NUM read, like 4/5/15; otherwise it stays pending
         * and the ARM re-enters the IRQ in a loop (C123 keypad: keypad_poll never reached). */
        if (num == 4 || num == 5 || num == 15 || (s->ilr[num] & 0x2)) {
            s->levels &= ~(1u << num);
        }

        calypso_inth_update(s);
        return num;
    }
    case 0x12:
    case 0x82:
    {

        uint16_t num = s->fiq_v;
        if (num == 4 || num == 5 || num == 15 || (s->ilr[num] & 0x2)) {
            s->levels &= ~(1u << num);
        }
        calypso_inth_update(s);
        return num;
    }
    case 0x14:
    case 0x84:
        return 0;
    default:
        if (offset >= 0x20 && offset < 0x60) {
            int idx = (offset - 0x20) / 2;
            return s->ilr[idx];
        }
        return 0;
    }
}

static void calypso_inth_write(void *opaque, hwaddr offset, uint64_t value,
                                unsigned size)
{
    CalypsoINTHState *s = CALYPSO_INTH(opaque);

    switch (offset) {

    case 0x00:
    {
        s->levels &= (0xFFFF0000u | (uint32_t)(value & 0xFFFF));
        calypso_inth_update(s);
        break;
    }
    case 0x02:
    {
        s->levels &= (0x0000FFFFu | ((uint32_t)(value & 0xFFFF) << 16));
        calypso_inth_update(s);
        break;
    }
    case 0x08:
    {
        s->mask = (s->mask & 0xFFFF0000) | (value & 0xFFFF);
        calypso_inth_update(s);
        break;
    }
    case 0x0a:
    {
        s->mask = (s->mask & 0x0000FFFF) | ((value & 0xFFFF) << 16);
        calypso_inth_update(s);
        break;
    }
    case 0x14:
    case 0x84:
    {

        uint16_t svc = s->ith_v;
        if (svc > 0 || (s->levels & 1)) {

            s->rr_start = (svc + 1) % CALYPSO_INTH_NUM_IRQS;
        }
        if ((value & 1) && s->last_num == 4) {
            s->frame_eoi++;          /* l1_sync() of the firmware is over */
        }
        calypso_inth_update(s);
        break;
    }
    default:
        if (offset >= 0x20 && offset < 0x60) {
            int idx = (offset - 0x20) / 2;
            /* ILR = 7 bits (FIQ/nIRQ, SENSE_EDGE, PRIORITY[4:0]). The firmware writes 0xfffe
             * for TIMER2 (default_irq_prio 0xff read as int8_t -1, irq.c irq_config): with a
             * 0x1FFF mask bit 8 would be taken for the FIQ flag (calypso_inth_update) and the
             * jiffies timer would go to FIQ, never acknowledged (ARM stuck in fiq()). */
            s->ilr[idx] = value & 0x7F;

            if (idx == 7) {
                s->ilr[7] = (s->ilr[7] & ~0x1F) | (s->ilr[4] & 0x1F);
            }

            if (idx == 18) {
                s->ilr[18] = (s->ilr[18] & ~0x1F) | (s->ilr[4] & 0x1F);
            }
        }
        break;
    }
}

static const MemoryRegionOps calypso_inth_ops = {
    .read = calypso_inth_read,
    .write = calypso_inth_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
    .valid = { .min_access_size = 1, .max_access_size = 2 },
    .impl  = { .min_access_size = 1, .max_access_size = 2 },
};

/* Single instance, for calypso_inth_arm_ack(): the emulated DSP needs this
 * acknowledgement. */
static CalypsoINTHState *g_inth;

/* Interrupt acknowledgement issued by the DSP, not by the ARM: end of service
 * of the current source, the queue moves on to the next one. No nesting, no
 * irq_in_service blocking. */
void calypso_inth_arm_ack(void)
{
    CalypsoINTHState *s = g_inth;
    if (!s) {
        return;
    }

    uint16_t svc = s->ith_v;
    if (svc > 0 || (s->levels & 1)) {
        s->levels &= ~(1u << svc);      /* end of service: the source drops */
        s->rr_start = (svc + 1) % CALYPSO_INTH_NUM_IRQS;
    }
}

uint64_t calypso_inth_frame_eoi(void)
{
    return g_inth ? g_inth->frame_eoi : 0;
}

bool calypso_inth_irq_masked(int irq)
{
    return !g_inth || (g_inth->mask & (1u << irq));
}

/* IRQ still pending in IT_REG (raised, not served yet). */
bool calypso_inth_irq_pending(int irq)
{
    return g_inth && (g_inth->levels & (1u << irq));
}

static void calypso_inth_realize(DeviceState *dev, Error **errp)
{
    g_inth = CALYPSO_INTH(dev);
    CalypsoINTHState *s = CALYPSO_INTH(dev);

    memory_region_init_io(&s->iomem, OBJECT(dev), &calypso_inth_ops, s,
                          "calypso-inth", 0x100);
    sysbus_init_mmio(SYS_BUS_DEVICE(dev), &s->iomem);

    sysbus_init_irq(SYS_BUS_DEVICE(dev), &s->parent_irq);
    sysbus_init_irq(SYS_BUS_DEVICE(dev), &s->parent_fiq);

    qdev_init_gpio_in(dev, calypso_inth_set_irq, CALYPSO_INTH_NUM_IRQS);
}

static void calypso_inth_reset(DeviceState *dev)
{
    CalypsoINTHState *s = CALYPSO_INTH(dev);

    s->levels = 0;
    s->mask = 0x00000000;
    s->ith_v = 0;
    s->fiq_v = 0;
    s->rr_start = 0;
    s->last_num = 0;
    s->frame_eoi = 0;
    memset(s->ilr, 0, sizeof(s->ilr));
}

static void calypso_inth_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->realize = calypso_inth_realize;
    device_class_set_legacy_reset(dc, calypso_inth_reset);
    dc->desc = "Calypso INTH interrupt controller";
}

static const TypeInfo calypso_inth_info = {
    .name          = TYPE_CALYPSO_INTH,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(CalypsoINTHState),
    .class_init    = calypso_inth_class_init,
};

static void calypso_inth_register_types(void)
{
    type_register_static(&calypso_inth_info);
}

type_init(calypso_inth_register_types)
