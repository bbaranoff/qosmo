/*
 * calypso_debug.c - env-gated probe lookup implementation
 *
 * One env var: CALYPSO_DEBUG="probe1,probe2,probe3,...".
 * A list containing "ALL" enables every probe.
 *
 * Lookup is O(N) over the list, parsed once on first call; negligible for the
 * 10-20 probes typically active.
 *
 * Normalisation: probe name upper-cased, '-' ' ' '.' '/' mapped to '_', so
 * "IMR-W" matches the entry "IMR_W" as well as "imr-w".
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */
#include "qemu/osdep.h"
#include "hw/arm/calypso/calypso_debug.h"

#include <stdlib.h>
#include <string.h>
#include <ctype.h>
#include <pthread.h>

#define MAX_ENTRIES   128
#define NAME_MAX_     64

static char     s_entries[MAX_ENTRIES][NAME_MAX_];
static int      s_entries_n = 0;
static bool     s_all = false;
static bool     s_inited = false;
static pthread_mutex_t s_mu = PTHREAD_MUTEX_INITIALIZER;

/* Master gate: -1 = not initialised yet, 0 = CALYPSO_DEBUG empty (every probe
 * OFF, fast path inlined in the header), 1 = at least one probe active. */
int calypso_debug_master = -1;

/* Normalize a probe name in place: upper-case, separators to '_'. */
static void normalize(char *s)
{
    for (; *s; s++) {
        char c = *s;
        if (c == '-' || c == ' ' || c == '/' || c == '.') c = '_';
        *s = toupper((unsigned char)c);
    }
}

static void parse_env_locked(void)
{
    if (s_inited) return;
    s_inited = true;

    const char *e = getenv("CALYPSO_DEBUG");
    if (!e || !*e) return;

    /* Walk comma-separated tokens. */
    const char *p = e;
    while (*p && s_entries_n < MAX_ENTRIES) {
        /* Skip leading separators (= comma, space, tab). */
        while (*p == ',' || *p == ' ' || *p == '\t') p++;
        if (!*p) break;
        const char *start = p;
        while (*p && *p != ',') p++;
        size_t len = p - start;
        if (len == 0) continue;
        /* Trim trailing whitespace. */
        while (len > 0 && (start[len-1] == ' ' || start[len-1] == '\t')) len--;
        if (len >= NAME_MAX_) len = NAME_MAX_ - 1;
        memcpy(s_entries[s_entries_n], start, len);
        s_entries[s_entries_n][len] = '\0';
        normalize(s_entries[s_entries_n]);
        if (strcmp(s_entries[s_entries_n], "ALL") == 0) {
            s_all = true;
        }
        s_entries_n++;
    }

    fprintf(stderr,
        "[calypso-debug] CALYPSO_DEBUG parsed: %d probe(s), ALL=%d\n",
        s_entries_n, s_all);
    for (int i = 0; i < s_entries_n; i++) {
        fprintf(stderr, "[calypso-debug]   - %s\n", s_entries[i]);
    }
}

/* One-shot init of the master gate: parses the env and sets
 * calypso_debug_master. Called from the inline calypso_debug_enabled() in the
 * header on first use. */
void calypso_debug_master_init(void)
{
    pthread_mutex_lock(&s_mu);
    parse_env_locked();
    calypso_debug_master = (s_all || s_entries_n > 0) ? 1 : 0;
    pthread_mutex_unlock(&s_mu);
}

/* Out-of-line implementation. Only reached with master == 1, so
 * parse_env_locked() has already run; the !s_inited block is belt and braces. */
bool calypso_debug_enabled_(const char *probe_name)
{
    if (!probe_name) return false;

    if (!s_inited) {
        pthread_mutex_lock(&s_mu);
        parse_env_locked();
        pthread_mutex_unlock(&s_mu);
    }

    if (s_all) return true;
    if (s_entries_n == 0) return false;

    /* Normalize probe_name into local buffer for compare. */
    char norm[NAME_MAX_];
    size_t n = strlen(probe_name);
    if (n >= NAME_MAX_) n = NAME_MAX_ - 1;
    memcpy(norm, probe_name, n);
    norm[n] = '\0';
    normalize(norm);

    for (int i = 0; i < s_entries_n; i++) {
        if (strcmp(s_entries[i], norm) == 0) return true;
    }
    return false;
}

/* calypso_gate - see calypso_debug.h for the semantics and the rationale.
 *
 * No cache: a caller that wants to memoise already does so in its own `static
 * int`. Caching HERE would prevent changing a gate at runtime and would hide
 * the cases where two modules read the same variable at different times. */
int calypso_gate(const char *nom, int defaut)
{
    const char *e = nom ? calypso_getenv(nom) : NULL;
    if (!e) {
        return defaut;          /* unset: the caller decides */
    }
    if (!*e) {
        return 0;               /* set but EMPTY = explicitly off */
    }
    /* Case-insensitive compare over the spellings people actually write in a
     * .env. Anything that is not a negation counts as yes: better to enable on
     * "yes" than to silently ignore a value the operator believed was
     * understood. */
    if (!strcasecmp(e, "0")     || !strcasecmp(e, "no") ||
        !strcasecmp(e, "off")   || !strcasecmp(e, "false") ||
        !strcasecmp(e, "n")) {
        return 0;
    }
    return 1;
}


/* ── getenv() memoised, see calypso_debug.h ────────────────────────────── */
#define GETENV_MEMO 512
static struct { const char *name; const char *val; } s_env[GETENV_MEMO];
static int s_env_n;
static pthread_mutex_t s_env_mu = PTHREAD_MUTEX_INITIALIZER;

const char *calypso_getenv(const char *name)
{
    if (!name) return NULL;
    /* fast path: string literals have one address per call site, and the
     * same variable is usually asked from the same site */
    int n = __atomic_load_n(&s_env_n, __ATOMIC_ACQUIRE);
    for (int i = 0; i < n; i++)
        if (s_env[i].name == name) return s_env[i].val;
    for (int i = 0; i < n; i++)
        if (!strcmp(s_env[i].name, name)) return s_env[i].val;
    const char *v = getenv(name);
    pthread_mutex_lock(&s_env_mu);
    if (s_env_n < GETENV_MEMO) {
        s_env[s_env_n].name = name;
        s_env[s_env_n].val  = v;
        __atomic_store_n(&s_env_n, s_env_n + 1, __ATOMIC_RELEASE);
    }
    pthread_mutex_unlock(&s_env_mu);
    return v;
}
