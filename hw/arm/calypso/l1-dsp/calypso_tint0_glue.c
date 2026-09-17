/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * [2026-09-16] Le seul morceau de qosmo-dsp/hw/arm/calypso/calypso_trx.c (l.
 * 1493) qui devait remonter tout de suite : calypso_tint0.c l'appelle en
 * extern. La-bas il tapait dans g_trx->fn puis appelait le tick statique du
 * meme fichier ; ici il passe par le symbole que la plateforme expose.
 */
#include "qemu/osdep.h"
#include "hw/arm/calypso/calypso_api.h"

void calypso_tint0_do_tick(uint32_t fn);
void calypso_tint0_do_tick(uint32_t fn)
{
    calypso_trx_force_tick(fn);
}
