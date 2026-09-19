#ifndef CALYPSO_ARM2DSP_H
#define CALYPSO_ARM2DSP_H

#include "calypso_c54x.h"

/*
 * calypso_arm2dsp - ARM->DSP task-post bridge. See calypso_arm2dsp.c for the
 * mechanism, the crutch annotations and the full env-variable list.
 */

/* Posts B_GSM_TASK when the ARM writes d_dsp_page in the shared API window.
 * offset = ARM byte offset into the DSP API window; value = the 16-bit value
 * written. Currently has NO caller: the ARM API-RAM write path does not route
 * through this module, so only the continuous-post path does anything. */
void calypso_arm2dsp_on_arm_write(uint16_t offset, uint16_t value);

/* Called once per DSP instruction step from calypso_c54x.c with the DSP state
 * and the PC about to execute. Applies a pending task post. */
void calypso_arm2dsp_on_dsp_step(C54xState *s, uint16_t exec_pc);

#endif /* CALYPSO_ARM2DSP_H */
