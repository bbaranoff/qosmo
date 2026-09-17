# qosmo — QEMU Calypso, une seule base, deux couches 1

Modèle QEMU du **baseband GSM TI Calypso** (Compal E88 / Motorola C1xx). Le
firmware [osmocom-bb](https://osmocom.org/projects/baseband) d'origine, non
patché, y boote et y tourne.

La couche 1 se choisit **au `configure`**, pas en changeant de dépôt :

```bash
../configure --target-list=arm-softmmu                      # plateforme nue
../configure --target-list=arm-softmmu --enable-l1-grgsm    # démod par gr-gsm
../configure --target-list=arm-softmmu --enable-l1-dsp      # mask-ROM TI sur C54x
```

## Pourquoi ce dépôt existe

`qemu-calypso` avait été scindé le 2026-09-01 en deux forks — `qosmo-grgsm` et
`qosmo-dsp` — pour répondre à deux questions différentes : *la pile marche-t-elle
de bout en bout ?* et *le vrai DSP acquiert-il ?* Le problème n'était pas la
scission, c'était sa forme : **deux dépôts sans commit racine commun**
(`6ee0024` contre `472f69d`). Ni merge, ni cherry-pick, ni base de fusion. Un
correctif de plateforme devait être réappliqué à la main dans l'autre arbre —
donc, en pratique, ne l'était pas.

Résultat mesuré le 2026-09-16, deux semaines après le split : **9 des 10
fichiers communs à `hw/arm/calypso/` avaient divergé**, `calypso_trx.c` allant
jusqu'à 478 lignes d'un côté contre 2478 de l'autre. Seul `Kconfig` était resté
identique.

Mais cette divergence, mesurée en retirant les deux couches 1, n'était presque
pas de la plateforme. C'était le même squelette préfixé différemment
(`tpu_read` contre `calypso_tpu_read`, `cpu_kick` contre `calypso_kick_cb`),
plus **deux fonctions qui sont la couche 1 déguisée en accès mémoire** :
`api_read`/`api_write` sur la fenêtre `0xFFD00000`. Le SoC, lui, était le même
des deux côtés — il l'est toujours, c'est le même silicium.

D'où ce dépôt : une plateforme, une couture explicite, deux L1 qui cohabitent
et ne se disputent qu'une ligne de `configure`.

## La couture

Neuf points d'appel, une structure : [`calypso_l1_ops.h`](include/hw/arm/calypso/calypso_l1_ops.h).

```c
static const CalypsoL1Ops mes_ops = {
    .name              = "grgsm",
    .init              = ma_l1_init,           /* fin de machine_init */
    .frame_tick        = ma_l1_frame_tick,     /* 1× par trame TDMA */
    .api_read_override = ma_l1_read_override,  /* lecture API RAM */
    .si_valid          = ma_l1_si_valid,
    .l1s_fn            = ma_l1s_fn,
    .burst_written     = ma_l1_burst_written,  /* d_burst_d */
    .rach_written      = ma_l1_rach_written,   /* d_rach */
    .page_written      = ma_l1_page_written,   /* d_dsp_page */
    .uart_tx_byte      = mon_tap_l1ctl,        /* optionnel */
};
static void enregistrer(void) { calypso_l1_register(&mes_ops); }
type_init(enregistrer)
```

Elle ne porte **que** le sens plateforme → L1. Le sens inverse reste en appels
directs vers ce que la base exporte (`calypso_api_ram()`,
`calypso_trx_get_fn()`, `calypso_trx_autosync_fn()`, `calypso_tsp_move()`,
`calypso_tpu_run_scenario()`) : c'est du matériel, il n'en existe qu'une
implémentation par construction, l'indirecter n'achèterait rien.

Le test du découpage a été de faire passer la L1 de `qosmo-grgsm` dessous :
`calypso_l1_grgsm.c` et `calypso_l1ctl_tap.c` sont repris **sans aucune
modification** hors le chemin de leurs deux `#include` privés. Tout le coût
d'adaptation tient dans
[`calypso_l1_grgsm_register.c`](hw/arm/calypso/l1-grgsm/calypso_l1_grgsm_register.c),
qui ne fait que remplir la structure. Si la couture avait été mal placée, il
aurait fallu retoucher la L1 elle-même.

## Vérifier

```bash
mkdir build && cd build
../configure --target-list=arm-softmmu --disable-werror
ninja qemu-system-arm
./qemu-system-arm -M help | grep calypso
```

La machine **nomme la L1 réellement liée** — c'est la façon la plus courte de
savoir ce qu'on a construit :

| configure | `-M help` affiche |
|---|---|
| (rien) | `couche 1 : aucune` |
| `--enable-l1-grgsm` | `couche 1 : grgsm` |
| `--enable-l1-dsp` | `couche 1 : c54x` |

Sans L1, le firmware boote, balaie et ne trouve jamais de cellule : le modem ne
reçoit rien. Ce n'est pas une panne, c'est ce qui rend la plateforme — boot,
IRQ, TDMA, UART, SIM — testable sans traîner gr-gsm ni le C54x.

Les deux options sont exclusives, et `meson` le refuse explicitement plutôt que
de laisser le second `calypso_l1_register()` écraser le premier en silence :

```
$ ../configure --enable-l1-grgsm --enable-l1-dsp
ERROR: calypso: --enable-l1-grgsm et --enable-l1-dsp sont exclusives.
```

## État

| | |
|---|---|
| plateforme (SoC, TPU, TSP, SIM, ULPD, UART, timers, SPI/I²C, INTH, TRF6151) | ✅ construit et boote seule |
| couture `CalypsoL1Ops` | ✅ 9 points d'appel, vérifiée avec la L1 gr-gsm |
| `--enable-l1-grgsm` | ✅ |
| `--enable-l1-dsp` | 🔧 les 26 000 lignes du C54x compilent et se lient ; la couture n'est pas encore branchée |
| devices de la carte E88 | 🔧 présents, compilent, pas branchés — voir ci-dessous |

Neuf devices (`calypso_iota.c`, `calypso_rf3166.c`, `calypso_asm4532.c`,
`calypso_xio.c`, `calypso_debug.c`, `calypso_invariants.c`, `fw_console.c`,
`sercomm_gate.c`, `l1ctl_sock.c`) sont construits **sous `--enable-l1-dsp`
seulement**, parce que leur unique point d'entrée est
`calypso_pcb_init()`, qui vit dans [`calypso_full_pcb.c`](https://github.com/bbaranoff/qosmo-dsp/blob/main/hw/arm/calypso/calypso_full_pcb.c)
— resté dans `qosmo-dsp` — lequel prend un `C54xState*`. Les activer donnerait neuf fichiers de code mort — mesuré, aucun
appelant hors de l'îlot fermé que forment `calypso_asm4532.c` et
`calypso_rf3166.c`, qui ne s'appellent que l'un l'autre.

L'ordre pour les débloquer est dans
[`hw/arm/calypso/meson.build`](hw/arm/calypso/meson.build).

## Licence

GPL-2.0-or-later, comme QEMU 9.2.4 dont ce dépôt dérive.
