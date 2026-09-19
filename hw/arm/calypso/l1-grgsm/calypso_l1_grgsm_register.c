/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Registers the gr-gsm layer 1 with the platform.
 *
 * [2026-09-16] This file is the whole cost of moving qosmo-grgsm's L1 under the
 * vtable: calypso_l1_grgsm.c and calypso_l1ctl_tap.c are reused UNCHANGED apart
 * from the paths of their two private #includes. That is the check on where the
 * seam sits - a badly placed one would have forced edits inside the L1 itself.
 */
#include "qemu/osdep.h"
#include "qemu/module.h"
#include "hw/arm/calypso/calypso_l1_ops.h"
#include "calypso_l1.h"
#include "calypso_l1ctl_tap.h"

static const CalypsoL1Ops grgsm_ops = {
    .name              = "grgsm",
    .init              = calypso_l1_init,
    .frame_tick        = calypso_l1_frame_tick,
    .api_read_override = calypso_l1_read_override,
    .si_valid          = calypso_l1_si_valid,
    .l1s_fn            = calypso_l1s_fn,
    .burst_written     = calypso_l1_burst_written,
    .rach_written      = calypso_l1_rach_written,
    .page_written      = calypso_l1_page_written,
    .uart_tx_byte      = calypso_l1ctl_tap_tx_byte,
};

/* type_init runs before calypso_machine_init(), which is the only ordering
 * constraint calypso_l1_register() imposes. */
static void calypso_l1_grgsm_register_type(void)
{
    calypso_l1_register(&grgsm_ops);
}
type_init(calypso_l1_grgsm_register_type)
