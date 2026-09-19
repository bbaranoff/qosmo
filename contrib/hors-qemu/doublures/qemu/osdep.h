/* SPDX-License-Identifier: GPL-2.0-or-later */
/* Stand-in for qemu/osdep.h, to build the Calypso sources OUTSIDE QEMU.
 * [2026-09-16] Measured before writing this file: the 12 sources that include
 * it use nothing from osdep.h but the standard C headers. */
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
/* The real osdep.h exposes this through _GNU_SOURCE; calypso_invariants.c uses it. */
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
