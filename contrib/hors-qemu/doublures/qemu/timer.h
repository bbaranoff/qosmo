/* SPDX-License-Identifier: GPL-2.0-or-later */
#ifndef QOSMO_DOUBLURE_TIMER_H
#define QOSMO_DOUBLURE_TIMER_H
#include <stdint.h>
/* Les timers sont inertes hors QEMU : c'est le harnais qui bat la mesure. */
typedef struct QEMUTimer { int vide; } QEMUTimer;
typedef enum { QEMU_CLOCK_REALTIME, QEMU_CLOCK_VIRTUAL, QEMU_CLOCK_HOST } QEMUClockType;
#define SCALE_NS 1
#define SCALE_US 1000
#define SCALE_MS 1000000
int64_t qemu_clock_get_ns(int type);
void timer_init_full(QEMUTimer *ts, void *lg, int type, int scale,
                     int attributes, void (*cb)(void *), void *opaque);
void timer_mod(QEMUTimer *ts, int64_t expire);
void timer_mod_ns(QEMUTimer *ts, int64_t expire);
#define timer_new_ns(type, cb, opaque) ((QEMUTimer *)g_malloc0(sizeof(QEMUTimer)))
#endif
