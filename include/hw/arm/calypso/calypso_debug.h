/*
 * calypso_debug.h - env-gated probe output infrastructure
 *
 * All diagnostic probes are SILENT by default. They are enabled through a
 * single env var:
 *
 *   CALYPSO_DEBUG="probe1,probe2,probe3,..."    - enables the listed probes
 *   CALYPSO_DEBUG=ALL                          - enables everything
 *   CALYPSO_DEBUG="ALL,..."                     - same
 *
 * The probe name is normalised to upper case, with '-' / ' ' / '.' / '/'
 * replaced by '_'. Matching is case-insensitive: probe "IMR-W" matches the
 * entry "imr-w" as well as "IMR_W".
 *
 * Cache: the list is parsed ONCE on the first call and stored.
 * Lookup = O(N) over the list (~50 ns for the typical 10-20 probes).
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */
#ifndef HW_ARM_CALYPSO_DEBUG_H
#define HW_ARM_CALYPSO_DEBUG_H

#include <stdbool.h>
#include <stdio.h>

/* Master gate: cached, parsed once. When CALYPSO_DEBUG is empty,
 * calypso_debug_master == 0 and calypso_debug_enabled() returns false
 * inline, WITHOUT any function call (no per-instruction overhead at the
 * call sites in the DSP hot loop). Identical behaviour otherwise. */
extern int  calypso_debug_master;
void        calypso_debug_master_init(void);
bool        calypso_debug_enabled_(const char *probe_name);   /* real impl */

static inline bool calypso_debug_enabled(const char *probe_name)
{
    if (__builtin_expect(calypso_debug_master < 0, 0)) {
        calypso_debug_master_init();
    }
    if (__builtin_expect(calypso_debug_master == 0, 1)) {
        return false;
    }
    return calypso_debug_enabled_(probe_name);
}

/* CALYPSO_DBG: gated fprintf. probe_name is a compile-time string literal.
 * fmt includes the prefix (e.g. "[c54x] IMR-W ...") and the final \n. */
#define CALYPSO_DBG(probe_name, fmt, ...) \
    do { \
        if (calypso_debug_enabled(probe_name)) { \
            fprintf(stderr, fmt, ##__VA_ARGS__); \
        } \
    } while (0)

/* Per-component wrappers: automatic prefix + \n + gate on the probe name
 * passed by the caller. */
#define C54_DBG(probe, fmt, ...) \
    CALYPSO_DBG(probe, "[c54x] " fmt "\n", ##__VA_ARGS__)
#define TRX_DBG(probe, fmt, ...) \
    CALYPSO_DBG(probe, "[calypso-trx] " fmt "\n", ##__VA_ARGS__)
#define BSP_DBG(probe, fmt, ...) \
    CALYPSO_DBG(probe, "[BSP] " fmt "\n", ##__VA_ARGS__)
#define IOTA_DBG(probe, fmt, ...) \
    CALYPSO_DBG(probe, "[iota] " fmt "\n", ##__VA_ARGS__)
#define PCB_DBG(probe, fmt, ...) \
    CALYPSO_DBG(probe, "[pcb] " fmt "\n", ##__VA_ARGS__)
#define INTH_DBG(probe, fmt, ...) \
    CALYPSO_DBG(probe, "[INTH] " fmt "\n", ##__VA_ARGS__)
#define UART_DBG(probe, fmt, ...) \
    CALYPSO_DBG(probe, fmt "\n", ##__VA_ARGS__)
#define TIMER_DBG(probe, fmt, ...) \
    CALYPSO_DBG(probe, "[timer] " fmt "\n", ##__VA_ARGS__)
#define TWL3025_DBG(probe, fmt, ...) \
    CALYPSO_DBG(probe, "[twl3025] " fmt "\n", ##__VA_ARGS__)

/* cdbg_env: compatibility shim for sites that test
 * getenv("CALYPSO_X")=="1". Returns "1" if the token is active in
 * CALYPSO_DEBUG, NULL otherwise. A getenv gate becomes a token by changing
 * only the call getenv -> cdbg_env. */
static inline const char *cdbg_env(const char *token)
{
    return calypso_debug_enabled(token) ? "1" : NULL;
}

#endif /* HW_ARM_CALYPSO_DEBUG_H */

/* =============================================================================
 *  calypso_gate() - ONE semantics for every environment-driven switch
 * =============================================================================
 *
 *  Every "is this option active?" question goes through this helper, so
 *  that `X=0` and `X=` get the same answer everywhere (an idiom such as
 *  `getenv("X") ? 1 : 0` would treat a variable declared EMPTY in an env
 *  file as ACTIVE).
 *
 *  SINGLE SEMANTICS:
 *
 *      variable absent                       -> `defaut`
 *      "0" · "" · "no" · "off" · "false"     -> 0     (case-insensitive)
 *      everything else                       -> 1
 *
 *  So `X=0` ALWAYS disables, and `X=1` always enables. `defaut` makes
 *  explicit, at the call site, what absence means.
 *
 *  WHAT THIS HELPER IS NOT. It answers yes or no. Variables that carry a
 *  VALUE (address, length, rate, path, list) keep their `getenv` +
 *  `strtoul`/`atoi`.
 * ---------------------------------------------------------------------------- */
int calypso_gate(const char *nom, int defaut);

/* getenv() memoised by name. The C54x run loop and its probes
 * consult the environment on hot paths (c54x_run is entered ~1200 times per
 * TDMA frame by the benches, c54x_exec_one 17k times), and glibc's getenv()
 * is a linear scan of the environment with a strncmp per entry: measured 40 %
 * of the replay's wall time under gdb sampling, 6.7 ms per 4.615 ms frame.
 * The environment never changes once the core runs (c54x_exe sets its one
 * variable before c54x_init()), so the first answer is kept. NULL is cached
 * too. Returns the same pointer glibc would (the environ string). */
const char *calypso_getenv(const char *name);
