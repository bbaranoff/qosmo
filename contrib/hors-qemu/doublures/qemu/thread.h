/* SPDX-License-Identifier: GPL-2.0-or-later */
#ifndef QOSMO_DOUBLURE_THREAD_H
#define QOSMO_DOUBLURE_THREAD_H
#include <pthread.h>
typedef pthread_mutex_t QemuMutex;
typedef pthread_cond_t  QemuCond;
typedef struct { pthread_t t; } QemuThread;
void qemu_mutex_init(QemuMutex *m);
void qemu_mutex_lock_func(QemuMutex *m, const char *f, int l);
void qemu_mutex_unlock_impl(QemuMutex *m, const char *f, int l);
void qemu_cond_init(QemuCond *c);
void qemu_cond_signal(QemuCond *c);
void qemu_cond_wait_func(QemuCond *c, QemuMutex *m, const char *f, int l);
void qemu_thread_create(QemuThread *th, const char *name,
                        void *(*start)(void *), void *arg, int mode);
#define qemu_mutex_lock(m)   qemu_mutex_lock_func(m, __FILE__, __LINE__)
#define qemu_mutex_unlock(m) qemu_mutex_unlock_impl(m, __FILE__, __LINE__)
#define qemu_cond_wait(c, m) qemu_cond_wait_func(c, m, __FILE__, __LINE__)
#define QEMU_THREAD_JOINABLE 0
#define QEMU_THREAD_DETACHED 1
#endif
