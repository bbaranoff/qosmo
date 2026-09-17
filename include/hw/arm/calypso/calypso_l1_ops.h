/* SPDX-License-Identifier: GPL-2.0-or-later */
#ifndef HW_ARM_CALYPSO_L1_OPS_H
#define HW_ARM_CALYPSO_L1_OPS_H

#include <stdint.h>
#include <stdbool.h>

/* ═════════════════════════════════════════════════════════════════════════
 * LA COUTURE ENTRE LA PLATEFORME ET LA COUCHE 1
 * ═════════════════════════════════════════════════════════════════════════
 *
 * [2026-09-16] Ce dépôt (qosmo) est né du constat suivant : qosmo-grgsm et
 * qosmo-dsp, issus du split de qemu-calypso, modélisent le MÊME SoC de deux
 * façons qui avaient déjà divergé sur les 10 fichiers qu'ils ont en commun —
 * 9 sur 10, `calypso_trx.c` allant jusqu'à 478 lignes d'un côté contre 2478
 * de l'autre. Mesurée honnêtement, cette divergence n'était pas de la
 * plateforme : c'était le même squelette, préfixé différemment (`tpu_read`
 * vs `calypso_tpu_read`), plus DEUX fonctions qui sont la couche 1 déguisée
 * en accès mémoire — `api_read`/`api_write` sur la fenêtre 0xFFD00000.
 *
 * Les deux dépôts n'ayant pas de commit racine commun (6ee0024 contre
 * 472f69d), un correctif de plateforme ne pouvait pas voyager de l'un à
 * l'autre : ni merge, ni cherry-pick, ni base de fusion. Il fallait
 * réapplique à la main, donc en pratique il n'était appliqué qu'une fois.
 *
 * D'où le découpage :
 *
 *     qosmo (ici)   le Calypso et sa carte. ARM, SoC, TPU, TSP, SIM, ULPD,
 *                   UART, timers, SPI/I²C, INTH, TRF6151, IOTA, RF3166,
 *                   ASM4532, XIO — plus l'horloge TDMA et la fenêtre API RAM.
 *                   AUCUNE couche 1. Se construit et boote seul : le firmware
 *                   osmocom-bb démarre, et ne voit jamais rien sur l'air.
 *
 *     qosmo-grgsm   + calypso_l1_grgsm.c   (démodulation par gr-gsm, hôte)
 *     qosmo-dsp     + calypso_c54x.c ...   (mask-ROM TI réelle sur C54x émulé)
 *
 * CE QUI PASSE PAR ICI, ET CE QUI N'Y PASSE PAS
 *
 * Cette vtable ne porte QUE le sens plateforme → L1 : les 9 endroits où le
 * matériel doit prévenir la couche 1 qu'il s'est passé quelque chose. Le sens
 * inverse — la L1 qui lit l'API RAM, la trame courante, qui se resynchronise —
 * reste en appels directs vers les symboles que la base exporte
 * (`calypso_api_ram()`, `calypso_trx_get_fn()`, `calypso_trx_autosync_fn()`,
 * `calypso_tsp_move()`, `calypso_tpu_run_scenario()`). C'est volontaire : ces
 * symboles-là sont du matériel, ils ont une seule implémentation par
 * construction, et les indirecter n'achèterait rien.
 *
 * Ne sont PAS dans la vtable non plus les entrées `dcch_set`, `dcch_active`,
 * `dcch_is_tch` et `rach_conf` de l'ancien calypso_l1.h : vérifié le
 * 2026-09-16, leurs seuls appelants sont dans calypso_l1ctl_tap.c, c'est-à-dire
 * à l'intérieur de la L1 de grgsm. Elles sont internes au fork, pas à la
 * couture. Les faire remonter ici aurait imposé à qosmo-dsp d'implémenter des
 * fonctions que rien dans son chemin d'exécution n'appelle.
 *
 * SI AUCUNE L1 N'EST ENREGISTRÉE
 *
 * Tout est silencieusement inerte (voir calypso_l1_dispatch.c) et
 * `api_read_override` rend false, donc l'API RAM se comporte en RAM nue.
 * C'est l'état nominal de la base seule, pas une erreur : c'est ce qui rend
 * `qosmo` testable sans embarquer l'une ou l'autre couche 1.
 * ═════════════════════════════════════════════════════════════════════════ */

typedef struct CalypsoL1Ops {
    /* Nom court du fork, pour les traces : "grgsm", "c54x", ... */
    const char *name;

    /* Fin de calypso_machine_init(). `firmware_elf` est le -kernel de la ligne
     * de commande (le layer1.highram.elf d'osmocom-bb), NULL si absent. */
    void (*init)(const char *firmware_elf);

    /* Sur chaque abaissement de l'IRQ trame TPU, donc une fois par trame TDMA,
     * depuis frame_irq_lower(). C'est le battement de la L1. */
    void (*frame_tick)(void);

    /* Lecture 16 bits dans la fenêtre API RAM, AVANT que la valeur ne parte
     * vers l'ARM. Rendre true et remplir `*out` pour imposer une valeur ;
     * rendre false laisse passer le contenu réel de la RAM.
     * `off` est l'offset dans la fenêtre, pas une adresse absolue. */
    bool (*api_read_override)(uint32_t off, uint16_t *out);

    /* La L1 a-t-elle de quoi servir un bloc maintenant ? Consultée quand le
     * firmware lit d_task_d et y trouve 0 : si true, la base répond
     * ALLC_DSP_TASK plutôt que « rien à faire ». */
    bool (*si_valid)(void);

    /* Numéro de trame vu par la L1 (l1s). Sert à la base pour détecter une
     * rupture de continuité dans l'anneau de bursts. */
    uint32_t (*l1s_fn)(void);

    /* Le firmware vient d'écrire d_burst_d / d_rach / d_dsp_page. Ce sont les
     * trois offsets dont la L1 gr-gsm a besoin, et longtemps les seuls trois
     * que cette vtable portait. */
    void (*burst_written)(uint16_t d_burst_d);
    void (*rach_written)(uint16_t d_rach, uint32_t fn);
    void (*page_written)(uint16_t d_dsp_page);

    /* [2026-09-16] TOUTE écriture 16 bits dans la fenêtre API RAM, après que la
     * base l'a rangée. Ajouté en remontant la L1 C54x, qui a montré la limite
     * de la vtable telle qu'elle avait été dessinée : gr-gsm ne s'intéresse
     * qu'à trois offsets nommés, alors que le DSP décode la fenêtre entière —
     * son calypso_dsp_write() fait 555 lignes. Trois hooks nommés ne pouvaient
     * pas le porter.
     *
     * Les trois ci-dessus SONT redondants avec celui-ci et restent malgré tout :
     * une L1 qui ne veut que d_burst_d n'a pas à filtrer toute la fenêtre pour
     * l'obtenir. Une L1 peut remplir les uns, l'autre, ou les deux ; l'ordre est
     * api_write_observed() D'ABORD, puis le hook nommé s'il y en a un. */
    void (*api_write_observed)(uint32_t off, uint16_t val, unsigned size);

    /* Un octet est sorti par l'UART modem (sercomm). Optionnel : c'est le
     * point d'accroche du tap L1CTL de qosmo-grgsm. NULL si le fork n'en veut
     * pas — auquel cas l'UART se comporte en UART. */
    void (*uart_tx_byte)(uint8_t ch);
} CalypsoL1Ops;

/* À appeler par le fork depuis un constructeur ou son propre type_init, donc
 * AVANT calypso_machine_init(). Un second enregistrement écrase le premier et
 * le signale sur stderr : deux couches 1 dans le même binaire est une erreur
 * de meson.build, pas un mode de fonctionnement. */
void calypso_l1_register(const CalypsoL1Ops *ops);

/* Le nom de la L1 active, ou "aucune". Pour les bannières et les traces. */
const char *calypso_l1_name(void);

/* Les accesseurs que la plateforme utilise. Ils encapsulent le test « une L1
 * est-elle enregistrée ? » pour que les sites d'appel restent lisibles. */
void calypso_l1_do_init(const char *firmware_elf);
void calypso_l1_do_frame_tick(void);
bool calypso_l1_do_api_read_override(uint32_t off, uint16_t *out);
bool calypso_l1_do_si_valid(void);
uint32_t calypso_l1_do_l1s_fn(void);
void calypso_l1_do_burst_written(uint16_t d_burst_d);
void calypso_l1_do_rach_written(uint16_t d_rach, uint32_t fn);
void calypso_l1_do_page_written(uint16_t d_dsp_page);
void calypso_l1_do_api_write_observed(uint32_t off, uint16_t val, unsigned size);
void calypso_l1_do_uart_tx_byte(uint8_t ch);

void calypso_l1_disable(const char *why);

#endif
