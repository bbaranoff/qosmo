/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Aiguillage plateforme → couche 1.
 *
 * [2026-09-16] Tout le fichier n'existe que pour que calypso_trx.c,
 * calypso_mb.c et calypso_uart.c n'aient plus à connaître le nom de
 * l'implémentation de L1 avec laquelle ils sont liés. Voir l'en-tête
 * calypso_l1_ops.h pour le pourquoi du découpage.
 *
 * LES DÉFAUTS NE SONT PAS DES ERREURS. Sans L1 enregistrée, `qosmo` seul est
 * un Calypso dont le modem ne reçoit rien : le firmware osmocom-bb boote,
 * balaie, ne trouve aucune cellule, et c'est le comportement attendu. C'est
 * ce qui permet de tester la plateforme — boot, IRQ, TDMA, UART, SIM — sans
 * traîner gr-gsm ni le C54x.
 */
#include "qemu/osdep.h"
#include <stdlib.h>
#include "hw/arm/calypso/calypso_api.h"
#include "hw/arm/calypso/calypso_l1_ops.h"

static const CalypsoL1Ops *l1;

void calypso_l1_register(const CalypsoL1Ops *ops)
{
    if (l1 && ops) {
        /* Deux L1 liées dans le même binaire : le meson.build du fork a repris
         * les sources de l'autre. On garde la dernière pour rester
         * déterministe, mais ça ne doit pas passer inaperçu. */
        fprintf(stderr, "calypso: DEUX couches 1 enregistrees (%s puis %s) - "
                        "verifier hw/arm/calypso/meson.build\n",
                l1->name ? l1->name : "?", ops->name ? ops->name : "?");
    }
    l1 = ops;
}

/* [2026-09-16] DSP externe (calypso_trx.c, CALYPSO_DSP_EXTERN) : le C54x de
 * c54x_exe tient la couche 1, le shunt gr-gsm ne doit ni s'initialiser (ses
 * ecoutes UDP voleraient le flux du banc) ni substituer quoi que ce soit dans
 * l'API RAM. Tous les calypso_l1_do_* deviennent des non-operations. */
void calypso_l1_disable(const char *why)
{
    if (l1) {
        fprintf(stderr, "calypso: couche 1 « %s » desactivee (%s)\n",
                l1->name ? l1->name : "?", why ? why : "");
    }
    l1 = NULL;
}

const char *calypso_l1_name(void)
{
    return (l1 && l1->name) ? l1->name : "aucune";
}

void calypso_l1_do_init(const char *firmware_elf)
{
    /* Avant tout : en DSP externe, le shunt ne doit meme pas ouvrir ses ports
     * (calypso_mb.c appelle ceci avant calypso_trx_init). */
    const char *ext = getenv("CALYPSO_DSP_EXTERN");
    if (ext && *ext) {
        calypso_l1_disable("DSP externe, CALYPSO_DSP_EXTERN");
        return;
    }
    if (l1 && l1->init) {
        l1->init(firmware_elf);
    }
}

void calypso_l1_do_frame_tick(void)
{
    if (l1 && l1->frame_tick) {
        l1->frame_tick();
    }
}

bool calypso_l1_do_api_read_override(uint32_t off, uint16_t *out)
{
    /* false = la base sert le contenu réel de l'API RAM. */
    return (l1 && l1->api_read_override) ? l1->api_read_override(off, out)
                                         : false;
}

bool calypso_l1_do_si_valid(void)
{
    return (l1 && l1->si_valid) ? l1->si_valid() : false;
}

uint32_t calypso_l1_do_l1s_fn(void)
{
    /* Sans L1, la seule trame qui existe est celle de l'horloge TDMA de la
     * plateforme. Rendre 0 ici casserait la détection de continuité de
     * l'anneau de bursts dans api_write(). */
    return (l1 && l1->l1s_fn) ? l1->l1s_fn() : calypso_trx_get_fn();
}

void calypso_l1_do_burst_written(uint16_t d_burst_d)
{
    if (l1 && l1->burst_written) {
        l1->burst_written(d_burst_d);
    }
}

void calypso_l1_do_rach_written(uint16_t d_rach, uint32_t fn)
{
    if (l1 && l1->rach_written) {
        l1->rach_written(d_rach, fn);
    }
}

void calypso_l1_do_page_written(uint16_t d_dsp_page)
{
    if (l1 && l1->page_written) {
        l1->page_written(d_dsp_page);
    }
}

void calypso_l1_do_uart_tx_byte(uint8_t ch)
{
    if (l1 && l1->uart_tx_byte) {
        l1->uart_tx_byte(ch);
    }
}

void calypso_l1_do_api_write_observed(uint32_t off, uint16_t val, unsigned size)
{
    if (l1 && l1->api_write_observed) {
        l1->api_write_observed(off, val, size);
    }
}
