/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Enregistrement de la couche 1 gr-gsm aupres de la plateforme.
 *
 * [2026-09-16] Ce fichier est tout ce que la L1 de qosmo-grgsm a eu a gagner
 * en passant sous la vtable : calypso_l1_grgsm.c et calypso_l1ctl_tap.c sont
 * repris SANS MODIFICATION autre que le chemin de leurs deux #include prives.
 * C'etait le test du decoupage - si la couture avait ete mal placee, il aurait
 * fallu retoucher la L1 elle-meme.
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

/* type_init s'execute avant calypso_machine_init(), ce qui est la seule
 * contrainte d'ordre imposee par calypso_l1_register(). */
static void calypso_l1_grgsm_register_type(void)
{
    calypso_l1_register(&grgsm_ops);
}
type_init(calypso_l1_grgsm_register_type)
