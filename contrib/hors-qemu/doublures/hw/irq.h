/* SPDX-License-Identifier: GPL-2.0-or-later */
#ifndef QOSMO_DOUBLURE_IRQ_H
#define QOSMO_DOUBLURE_IRQ_H
/* Sans QEMU il n'y a pas de ligne d'interruption : les niveaux partent au vide. */
typedef struct IRQState *qemu_irq;
void qemu_set_irq(qemu_irq irq, int level);
#define qemu_irq_raise(i) qemu_set_irq(i, 1)
#define qemu_irq_lower(i) qemu_set_irq(i, 0)
#endif
