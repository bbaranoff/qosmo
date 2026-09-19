/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * TINT0 tick shim. calypso_tint0.c declares calypso_tint0_do_tick() extern and
 * calls it; the frame counter lives in the platform, so the tick is forwarded
 * to the exported calypso_trx_force_tick() symbol instead of poking a local
 * TRX state.
 */
#include "qemu/osdep.h"
#include "hw/arm/calypso/calypso_api.h"

void calypso_tint0_do_tick(uint32_t fn);
void calypso_tint0_do_tick(uint32_t fn)
{
    calypso_trx_force_tick(fn);
}
