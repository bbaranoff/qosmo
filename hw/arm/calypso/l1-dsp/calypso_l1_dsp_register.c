/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Registers the C54x layer 1 with the platform.
 *
 * [2026-09-16] State: the DSP core is ported and linked, but the seam is not
 * wired yet. The vtable entries stay NULL until calypso_dsp_read(),
 * calypso_dsp_write() and calypso_tdma_tick() are extracted from qosmo-dsp's
 * calypso_trx.c, where they are ~1300 lines entangled with that fork's
 * CalypsoTRX struct and its dsp / dsp_ram fields, which the common platform
 * does not have.
 *
 * Registering with the name alone is deliberate: "couche 1 : c54x" under
 * -M help states what is actually LINKED, and the base test checks that
 * string. Skipping the registration would print "aucune" and make a
 * half-finished build look like a bare base.
 */
#include "qemu/osdep.h"
#include "qemu/module.h"
#include "hw/arm/calypso/calypso_l1_ops.h"

static const CalypsoL1Ops c54x_ops = {
    .name = "c54x",
    /* .init, .frame_tick, .api_read_override, .api_write_observed, ...:
     * see the header comment above. */
};

static void calypso_l1_dsp_register_type(void)
{
    calypso_l1_register(&c54x_ops);
}
type_init(calypso_l1_dsp_register_type)
