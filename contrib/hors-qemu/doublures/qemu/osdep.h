/* SPDX-License-Identifier: GPL-2.0-or-later */
/* Doublure de qemu/osdep.h pour compiler les sources Calypso HORS de QEMU.
 * [2026-09-16] Les 12 fichiers qui l'incluent n'en tirent que les en-tetes C
 * standard : mesure faite avant d'ecrire ce fichier. */
#ifndef QOSMO_DOUBLURE_OSDEP_H
#define QOSMO_DOUBLURE_OSDEP_H
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <stddef.h>
#include <inttypes.h>
#include <errno.h>
#include <unistd.h>
#include <fcntl.h>
#include <time.h>
#include <math.h>
#include <assert.h>
/* Le vrai osdep.h l'expose via _GNU_SOURCE ; calypso_invariants.c s'en sert. */
extern char **environ;

typedef uint64_t hwaddr;
#ifndef MIN
#define MIN(a, b) ((a) < (b) ? (a) : (b))
#endif
#ifndef MAX
#define MAX(a, b) ((a) > (b) ? (a) : (b))
#endif
#define g_assert_not_reached() abort()
void *g_malloc(size_t n);
void *g_malloc0(size_t n);
void g_free(void *p);
#define g_new0(t, n) ((t *)g_malloc0(sizeof(t) * (n)))
#endif
