/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Enregistrement de la couche 1 C54x aupres de la plateforme.
 *
 * [2026-09-16] ETAT : le coeur DSP est porte et lie, la couture n'est pas
 * encore branchee. Les entrees de la vtable restent NULL tant que
 * calypso_dsp_read() / calypso_dsp_write() / calypso_tdma_tick() ne sont pas
 * extraits du calypso_trx.c de qosmo-dsp - ils y sont ~1300 lignes melees a la
 * structure CalypsoTRX de CE fork-la, qui porte des champs (dsp, dsp_ram) que
 * la plateforme commune n'a pas.
 *
 * Enregistrer quand meme, avec le seul nom, est deliberé : « couche 1 : c54x »
 * a -M help dit la verite sur ce qui est LIE, et le test de la base verifie
 * cette chaine. Un enregistrement absent aurait affiche « aucune » et fait
 * passer un build a moitie fait pour une base nue.
 */
#include "qemu/osdep.h"
#include "qemu/module.h"
#include "hw/arm/calypso/calypso_l1_ops.h"

static const CalypsoL1Ops c54x_ops = {
    .name = "c54x",
    /* .init, .frame_tick, .api_read_override, .api_write_observed, ... :
     * voir l'en-tete ci-dessus. */
};

static void calypso_l1_dsp_register_type(void)
{
    calypso_l1_register(&c54x_ops);
}
type_init(calypso_l1_dsp_register_type)
