/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Les cales QEMU pour faire tourner le DSP hors de QEMU.
 *
 * [2026-09-16] Mesure faite avant d'ecrire une ligne : sur les 26 631 lignes
 * de la couche 1 C54x, l'accroche a QEMU tient en QUATORZE symboles, et huit
 * d'entre eux sont dans calypso_full_pcb.c et calypso_tint0.c - le cablage,
 * pas le DSP. Le coeur calypso_c54x.c lui-meme en demande DEUX (les mutex), et
 * calypso_{arm2dsp,dma,fbsb,mailbox,rhea_dma,rif,twl3025}.c en demandent ZERO.
 * C'est ce qui rend ce binaire possible.
 *
 * Rien ici ne modelise QEMU : ce sont des equivalents POSIX, choisis pour que
 * le code du DSP ne soit pas touche. Le jour ou une cale doit devenir autre
 * chose qu'un pthread, c'est que le DSP s'est mis a dependre de QEMU et il
 * faut le savoir.
 */
#include <pthread.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <stdio.h>

/* ── horloge ───────────────────────────────────────────────────────────── */
int64_t qemu_clock_get_ns(int type)
{
    struct timespec ts;
    (void)type;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (int64_t)ts.tv_sec * 1000000000LL + ts.tv_nsec;
}

/* ── mutex / conditions / threads : pthread, sans detour ───────────────── */
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

/* ── timers : le harnais bat la mesure lui-meme, ces cales sont inertes ─── */
void timer_init_full(void *ts, void *lg, int type, int scale,
                     int attributes, void (*cb)(void *), void *opaque)
{ (void)ts; (void)lg; (void)type; (void)scale; (void)attributes;
  (void)cb; (void)opaque; }
void timer_mod(void *ts, int64_t expire) { (void)ts; (void)expire; }
void timer_mod_ns(void *ts, int64_t expire) { (void)ts; (void)expire; }

/* ── plomberie d'E/S : sans QEMU il n'y a ni ligne d'IRQ ni boucle main ─── */
void qemu_set_irq(void *irq, int level) { (void)irq; (void)level; }
void qemu_set_fd_handler(int fd, void *rd, void *wr, void *opaque)
{ (void)fd; (void)rd; (void)wr; (void)opaque; }
void qemu_notify_event(void) { }

/* ── glib, quand il n'est pas lie ──────────────────────────────────────── */
void *g_malloc(size_t n) { void *p = malloc(n ? n : 1); if (!p) abort(); return p; }
void *g_malloc0(size_t n) { void *p = calloc(1, n ? n : 1); if (!p) abort(); return p; }
void g_free(void *p) { free(p); }
