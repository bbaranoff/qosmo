#ifndef CALYPSO_INVARIANTS_H
#define CALYPSO_INVARIANTS_H
#include <stdbool.h>
/* Manifest: dump the active CALYPSO_* once. Gate CALYPSO_INVARIANTS. */
void calypso_manifest_once(void);
/* Invariant: if ok=false, [INVARIANT-FAIL] <tag> once. Gate CALYPSO_INVARIANTS (default off). */
bool calypso_invariant(const char *tag, bool ok, const char *fmt, ...)
    __attribute__((format(printf, 3, 4)));
#endif
