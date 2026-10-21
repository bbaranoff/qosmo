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
        /* [2026-09-30] SOURCE CONFIGUREE SUR FRONT (ILR bit 1, irq_config(...,
         * edge=1)) : le front est MEMORISE dans IT_REG jusqu'a la lecture de
         * IRQ_NUM (ci-dessous, pour 4/5/15) ou l'effacement logiciel. Avant,
         * la retombee de la ligne effacait le bit : l'IT trame TPU (IRQ 4) est
         * une impulsion de 1 ms (calypso_trx.c FRAME_IRQ_PULSE_NS) et, si
         * l'ARM etait encore dans un traitement long (UART/L1CTL en pleine
         * parole), l'IT etait PERDUE. Le firmware ne detecte pas une trame
         * TDMA perdue : son compteur de trames glissait d'une unite pour le
         * reste du canal (a_a5fn = fn-1 : [a5] ecart +1 -> 0, parole
         * dechiffree avec le mauvais flux, FACCH a faux, SACCH perdue, LOS
         * ~15 s plus tard). Mesure : appels de 12:09 et 12:26, bascule a
         * l'injection du ton (rafales de TRAFFIC_REQ sur l'UART). */
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

        /* [2026-10-20] une source configuree SUR FRONT (ILR bit 1 : TIMER2 de jiffies, qui ne redescend jamais
         * sa ligne) est acquittee par la lecture d'IRQ_NUM, comme 4/5/15 ; sinon elle reste pendante et
         * l'ARM reprend l'IT en boucle (clavier C123 : keypad_poll jamais atteint). */
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
            /* [2026-10-20] ILR = 7 bits (FIQ/nIRQ, SENSE_EDGE, PRIORITY[4:0]). Le firmware ecrit 0xfffe pour
             * TIMER2 (default_irq_prio 0xff lu en int8_t -1, irq.c irq_config) : avec un masque 0x1FFF le bit 8
             * passait pour le drapeau FIQ (calypso_inth_update) et le timer de jiffies partait en FIQ, jamais
             * acquitte -> avalanche (jiffies +160 000/s, ARM coince dans fiq()). */
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

/* [2026-09-16] Instance unique, pour calypso_inth_arm_ack(). Remonte depuis
 * qosmo-dsp en portant la L1 C54x : la version de l'INTH heritee de
 * qosmo-grgsm avait perdu cet acquittement, et le DSP emule en a besoin. */
static CalypsoINTHState *g_inth;

/* Acquittement d'interruption emis par le DSP, pas par l'ARM : fin de service
 * de la source courante, la file repart au suivant. Porte tel quel de
 * qosmo-dsp, MOINS son « s->irq_in_service = -1 » - ce champ y est ecrit deux
 * fois et jamais relu (l'en-tete du fichier la-bas le dit : « no nesting, no
 * irq_in_service blocking »). Ajouter un champ mort a la plateforme commune
 * pour la fidelite d'un vestige n'aurait servi personne. */
void calypso_inth_arm_ack(void)
{
    CalypsoINTHState *s = g_inth;
    if (!s) {
        return;
    }

    uint16_t svc = s->ith_v;
    if (svc > 0 || (s->levels & 1)) {
        s->levels &= ~(1u << svc);      /* fin de service : la source retombe */
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

/* [2026-09-30] IT encore en attente dans IT_REG (levee, pas encore servie). */
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
