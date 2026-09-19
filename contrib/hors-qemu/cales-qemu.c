/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * QEMU shims that let the DSP run outside QEMU.
 *
 * [2026-09-16] Measured before writing a line: across the 26631 lines of the
 * C54x layer 1, the hook into QEMU comes to FOURTEEN symbols, eight of them in
 * calypso_full_pcb.c and calypso_tint0.c, i.e. the wiring rather than the DSP.
 * The calypso_c54x.c core itself needs TWO (the mutexes), and
 * calypso_{arm2dsp,dma,fbsb,mailbox,rhea_dma,rif,twl3025}.c need NONE. That is
 * what makes this binary possible.
 *
 * Nothing here models QEMU: these are POSIX equivalents, chosen so the DSP code
 * stays untouched. The day a shim has to become something other than a pthread,
 * the DSP has started depending on QEMU and that must be noticed.
 */
#include <pthread.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <stdio.h>

/* -- clock ---------------------------------------------------------------- */
int64_t qemu_clock_get_ns(int type)
{
    struct timespec ts;
    (void)type;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (int64_t)ts.tv_sec * 1000000000LL + ts.tv_nsec;
}

/* -- mutexes / condvars / threads: plain pthread --------------------------- */
void qemu_mutex_init(pthread_mutex_t *m) { pthread_mutex_init(m, NULL); }
void qemu_mutex_lock_func(pthread_mutex_t *m, const char *f, int l)
{ (void)f; (void)l; pthread_mutex_lock(m); }
void qemu_mutex_unlock_impl(pthread_mutex_t *m, const char *f, int l)
{ (void)f; (void)l; pthread_mutex_unlock(m); }
void qemu_cond_init(pthread_cond_t *c) { pthread_cond_init(c, NULL); }
void qemu_cond_signal(pthread_cond_t *c) { pthread_cond_signal(c); }
void qemu_cond_wait_func(pthread_cond_t *c, pthread_mutex_t *m,
                         const char *f, int l)
{ (void)f; (void)l; pthread_cond_wait(c, m); }

void qemu_thread_create(void *thread, const char *name,
                        void *(*start)(void *), void *arg, int mode)
{
    pthread_t t;
    (void)name; (void)mode;
    pthread_create(&t, NULL, start, arg);
    if (thread) {
        memcpy(thread, &t, sizeof(t));
    }
}

/* -- timers: the harness keeps time itself, so these shims are inert ------- */
void timer_init_full(void *ts, void *lg, int type, int scale,
                     int attributes, void (*cb)(void *), void *opaque)
{ (void)ts; (void)lg; (void)type; (void)scale; (void)attributes;
  (void)cb; (void)opaque; }
void timer_mod(void *ts, int64_t expire) { (void)ts; (void)expire; }
void timer_mod_ns(void *ts, int64_t expire) { (void)ts; (void)expire; }

/* -- I/O plumbing: without QEMU there is no IRQ line and no main loop ------ */
void qemu_set_irq(void *irq, int level) { (void)irq; (void)level; }
void qemu_set_fd_handler(int fd, void *rd, void *wr, void *opaque)
{ (void)fd; (void)rd; (void)wr; (void)opaque; }
void qemu_notify_event(void) { }

/* -- glib, when it is not linked in ---------------------------------------- */
void *g_malloc(size_t n) { void *p = malloc(n ? n : 1); if (!p) abort(); return p; }
void *g_malloc0(size_t n) { void *p = calloc(1, n ? n : 1); if (!p) abort(); return p; }
void g_free(void *p) { free(p); }
