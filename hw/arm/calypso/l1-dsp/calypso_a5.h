/* calypso_a5.h - coprocesseur A5 du DSP Calypso (ports XIO 0x2800..0x2818).
 * Voir calypso_a5.c. */
#ifndef CALYPSO_A5_H
#define CALYPSO_A5_H

#include <stdbool.h>
#include <stdint.h>

typedef struct C54xState C54xState;

bool calypso_a5_on(void);
bool calypso_a5_portw(C54xState *s, uint16_t pa, uint16_t val);
bool calypso_a5_portr(C54xState *s, uint16_t pa, uint16_t *out);

#endif
