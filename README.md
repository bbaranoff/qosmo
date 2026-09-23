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

La couture sait aussi se retirer : `calypso_l1_disable(why)`
([`calypso_l1_ops.h:138`](include/hw/arm/calypso/calypso_l1_ops.h)) neutralise
la L1 liée quand `CALYPSO_DSP_EXTERN` est posé. Le tap DCCH
([`calypso_dcch_tap.c`](hw/arm/calypso/calypso_dcch_tap.c)), qui ne dépend
d'aucune L1, remplace alors `calypso_l1ctl_tap.c` pour publier le canal dédié.

## Le pont DSP externe

C'est le chemin **réellement utilisé** sur le banc : la mask-ROM tourne dans
`c54x_exe`, hors de QEMU, et `calypso_trx.c` lui parle par
[`calypso_dsp_pont.h`](include/hw/arm/calypso/calypso_dsp_pont.h).

- `CALYPSO_DSP_EXTERN=1` (ou un chemin de socket) désactive la L1 liée
  (`calypso_l1_disable`) et branche QEMU sur `c54x_exe --arm`, **à lancer
  d'abord** — sinon `shm_open(...) ... lancez d'abord c54x_exe --arm` et
  `exit(1)`.
- API RAM partagée par `/dev/shm/calypso_api_ram` (16 Ko), synchronisation
  trame par trame sur `/tmp/calypso_dsp.sock` (`SOCK_SEQPACKET`).
- Messages : `HELLO`/`HELLO_OK` (magic `'C54X'`), `RESET` (écriture de
  `DL_STATUS`), `TICK` (a = fn, b = page), `DONE` (drapeaux IRQ/IDLE/INIT),
  `GO`, `DCCH`.
- `TICK.b` bit 16 : IRQ trame DSP armée par l'ARM — `TPU_CTRL_DSP_EN` est à
  usage unique (`36efec9`, 2026-09-20) ; sans lui le DSP ne reçoit pas
  d'interruption trame.
- `TICK.b` bit 17 : TICK en deux phases (`255cdd6`, 2026-09-21) — ISR de la
  ROM, `DONE|PONT_DONE_PHASE_A`, IRQ trame ARM et `l1_sync()`, puis `PONT_GO`
  et seulement alors le burst. Avant, la page R du burst N écrasait le burst
  N-2 sous l'ARM (`EMPTY` / `BURST ID n!=m`).
- `PONT_DCCH` porte TN, genre et sous-voie du canal dédié — SDCCH/4 (0),
  SDCCH/8 (1) ou libéré (`0xFF`) seulement. Il est appris par
  `calypso_dcch_tap.c`, qui renifle les `L1CTL_DATA_CONF`/`DATA_IND` du flux
  sercomm et publie `/dev/shm/calypso_dcch_cfg` ; ce fichier, lui, annonce
  aussi le TCH (genre 2 = TCH/F, 3 = TCH/H, 2026-09-23), mais le TCH n'est pas
  relayé par `PONT_DCCH` : la bascule du BSP sur le TCH suit la tâche du
  firmware côté `c54x_exe`.

Les sources de `l1-dsp/` se compilent aussi hors QEMU grâce à
[`contrib/hors-qemu/cales-qemu.c`](contrib/hors-qemu/cales-qemu.c) :
équivalents POSIX des quelques symboles QEMU dont le DSP dépend, plus les
en-têtes de `contrib/hors-qemu/doublures/`. C'est ainsi que `c54x_exe` les
construit, sans les recopier (`QOSMO ?= /opt/GSM/qosmo` dans son `Makefile`).

### Variables d'environnement côté QEMU

| variable (sur `qemu-system-arm`) | effet |
|---|---|
| `CALYPSO_DSP_EXTERN` | `1` ou chemin de socket : pont vers `c54x_exe` |
| `CALYPSO_PONT_LOCKSTEP=1` | QEMU attend `DONE` à chaque trame — indispensable, le C54x émulé est plus lent que le temps réel |
| `CALYPSO_PONT_ARM_FIRST=0` | ancien ordre, sans les deux phases |
| `CALYPSO_PONT_RETRY_DIV` | relance vers le DSP toutes les trame/N (défaut 16, jugé sans effet dans `calypso_trx.c`) ; `c54x_exe/run.sh` pose 64 depuis le 2026-09-23 (latence de la phase A et du GO, mesurée par `[chrono]` en TCH) |
| `CALYPSO_CPU_KICK_NS` | période du kick CPU, réglage de mesure (défaut 5 ms, mesuré sans effet) |
| `CALYPSO_PACER_RATTRAPAGE` | trames de retard rattrapées par le pacer TDMA (20 par défaut) ; `0` = ancienne grille stricte, qui sautait des trames |

`CALYPSO_BSP_STREAM`, `CALYPSO_A5`, `CALYPSO_SONDES`, `CALYPSO_C54X_OVM`,
`CALYPSO_MVKD_DMAD_AVANT`, `CALYPSO_HACK_SOFT_SCALE` sont lus par le code de
`l1-dsp/` : avec le DSP externe, ils se posent sur **`c54x_exe`** (le
processus du DSP), pas sur QEMU, où ils sont sans effet.

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
| plateforme (SoC, TPU, TSP, SIM, ULPD, UART, timers, SPI/I²C, INTH, TRF6151, `calypso_l1_dispatch.c`, `calypso_dcch_tap.c`) | ✅ construit et boote seule ; l'INTH compte la fin de service (`IRQ_CTRL` bit 0 après `IRQ_NUM`=4) pour le TICK en deux phases |
| couture `CalypsoL1Ops` | ✅ 9 points d'appel, vérifiée avec la L1 gr-gsm |
| `--enable-l1-grgsm` | ✅ |
| `--enable-l1-dsp` (C54x dans QEMU) | 🔧 les ~28 600 lignes de `l1-dsp/` compilent et se lient ; la vtable reste vide (seul `.name = "c54x"`) |
| DSP externe (`CALYPSO_DSP_EXTERN`) | ✅ mask-ROM dans `c54x_exe`, couplée à l'ARM par le pont ; SCH (BSIC/FN), BCCH SI1-4 et LU sur SDCCH atteints sur le banc ; appels TCH/F de bout en bout, parole décodée par la ROM (runs du 2026-09-23 20:22 et 20:32) ; toutes les trames de parole marquées `B_BFI`, un LOS en appel à 20:32 — voir ci-dessous |
| devices de la carte E88 | 🔧 présents, compilent, pas branchés — voir ci-dessous |

Neuf devices (`calypso_iota.c`, `calypso_rf3166.c`, `calypso_asm4532.c`,
`calypso_xio.c`, `calypso_debug.c`, `calypso_invariants.c`, `fw_console.c`,
`sercomm_gate.c`, `l1ctl_sock.c`) sont construits **sous `--enable-l1-dsp`
seulement**, parce que leur unique point d'entrée est
`calypso_pcb_init()`, qui vit dans
[`l1-dsp/calypso_full_pcb.c`](hw/arm/calypso/l1-dsp/calypso_full_pcb.c) —
désormais dans ce dépôt, avec `calypso_rif.c`, `calypso_rhea_dma.c`,
`calypso_tint0.c` et `calypso_twl3025.c`, construit sous `--enable-l1-dsp` —
lequel prend toujours un `C54xState*` (passé en `void *`) et n'est toujours appelé par personne
(rien dans `calypso_soc_realize()`). Les activer donnerait neuf fichiers de code mort — mesuré, aucun
appelant hors de l'îlot fermé que forment `calypso_asm4532.c` et
`calypso_rf3166.c`, qui ne s'appellent que l'un l'autre.

L'ordre pour les débloquer est dans
[`hw/arm/calypso/meson.build`](hw/arm/calypso/meson.build) — dont le bloc
« RESTENT DANS qosmo-dsp » est périmé depuis que ces cinq fichiers sont dans
`l1-dsp/`.

### Cœur C54x : correctifs récents

- **2026-09-18/19** — `calypso_c54x.c` découpé en `c54x_exec`, `c54x_decode`,
  `c54x_mem`, `c54x_irq`, `c54x_probes`.
- **2026-09-20** — ROL/ROR sur 32 bits (SPRU172C, c'était une rotation 40 bits),
  tous les modes Lmem par `resolve_lmem`, CPL respecté en adressage direct :
  le SCH décode (`15c0d45`). `getenv()` mémoïsé sur les chemins chauds
  (6,7 → 3,7 ms/trame, `d68baaf`) ; chemin rapide `c54x_rapide` et file BSP de
  8192 bursts (`54d8320`) ; modulation GMSK partagée `calypso_gmsk.c`
  (`d8e5fef`).
- **2026-09-21** — OVA/OVB et saturation OVM (`CALYPSO_C54X_OVM=0` pour
  revenir), MPYR/MACR/MASR, `0xA1xx` = `ADD Xmem,Ymem` (pas `SQDST`),
  `*+ARx(lk)%` circulaire, retenue SFTA au bit 40-SHIFT : le BCCH décode,
  SI1-4 (`4771f91`).
- **2026-09-23** — sondes coupées par défaut (`CALYPSO_SONDES=1` ou
  `CALYPSO_DEBUG` pour les rallumer) ; `RPT *(lk)` avance de 1 + `lk_used`
  (crash TCH `SP-CORRUPT`) ; `MVKD`/`MVDK` longs lisent `lk` avant `dmad`
  (SACCH du TCH, pointeur `0x3d89` ; `CALYPSO_MVKD_DMAD_AVANT=1` = ancien
  ordre) ; coprocesseur A5 sur les ports XIO `0x2800..0x2818`
  ([`calypso_a5.c`](hw/arm/calypso/l1-dsp/calypso_a5.c), `osmo_a5()`).

### Banc DSP : runs du 2026-09-23 20:22 et 20:32

Montage pont DSP (`c54x_exe`), mobile DSP = MS 1 (MSISDN 100101). Journaux :
`osmo-*.log`, archives du pont `20260923-202448` et `20260923-203413`.

Constaté :

- **Parole TCH/F décodée par la ROM.** Appel MO vers l'écho 600 à 20:22
  (ACTIVE 20:22:55, DISCONNECT 20:23:27, ~32 s ; TCH dl=1607 ul=1601
  trames), appel MT 100102 → 100101 (ACTIVE 20:24:28, release normal
  20:24:32), appel MO à 20:33:39 → DISCONNECT 20:34:06 au run suivant. La ROM
  TI fait démodulation, égalisation, désentrelacement et Viterbi TCH/F
  descendant ; codec (GAPK FR) et codage canal montant tournent sur l'hôte.
  Audible dans les deux sens.
- **Ordre `MVKD`/`MVDK` / SACCH `0x3d89`** : à 20:22, les deux appels vont
  jusqu'à la libération normale sans LOS (plus de LOS après ~20 s). Pas
  vrai du premier appel de 20:32, voir plus bas.
- **Coprocesseur A5** : journal `[a5]` actif sur les deux runs (Kc lu sur
  les ports XIO, `NOUVEAU Kc` à chaque changement) ; chiffrement descendant
  confirmé par la BTS 5 fois à 20:22 (LU, appel 600, SMS MT, SMS MO, appel
  MT : chaque établissement) et 3 fois à 20:32.
- **Temps réel** : 29 513 trames à 20:22 (28 441 à 20:32), un seul tick
  sauté par run, au boot (`fn=0`). Marge : en TCH, `[chrono]` à 20:22
  (fn 5997-11997) A 0,32-0,34 + go 0,39-0,49 + B 0,15-0,16 + après DONE
  3,37-3,68 ms ; à 20:32 A 0,29-0,33 + go 0,40-0,50 + B 0,11-0,16 + après
  DONE 3,33-3,59 ms. Soit ~4,2-4,6 ms de travail DSP pour 4,62 ms de trame ;
  l'attente de QEMU tombe à 0,04-0,45 ms. Hors TCH ~0,6-2 ms. Tenu, mais
  sans réserve : c'est aussi une alerte.

Anomalies ouvertes, par ordre d'importance :

1. **`B_BFI` sur toutes les trames de parole.** Sonde `[a_dd]` à 20:32 :
   `vues=2200 bfi=2200`. Sur les 42 lignes échantillonnées : 19 × `c214`
   err=0, toutes parmi les 20 premières après la bascule (la 18e est un
   `8084` err=58), puis 17 × `c204` err=15..80 et 5 × `80c4` err=81..93
   (`err` = `a_dd_0[2]`, `num_biterr`). Premier appel (fn 5839-9306, celui
   du LOS) : err 58 à 93 hors `c214` ; appel de 20:33:39 (fn 21248-26881) :
   `c204` err=15..38. Ce n'est donc pas « bits bons, BFI faux » :
   la ROM mesure ~3 à 20 % d'erreurs sur 456 bits codés et le FR reste
   intelligible (le firmware ne remonte pas le BFI, GAPK décode tel quel).
   Origine à trancher : signal (BSP, IQ, égalisation) ou cœur C54x
   (Viterbi, recodage qui compte les erreurs). Comparaison bit à bit avec
   les trames BTS à faire. Le `ko=376` de la sonde compte `B_FIRE1`, sans
   sens défini sur la parole.
2. **LOS en appel à 20:32.** Appel MO ACTIVE 20:32:30 ; le compteur ACCH
   descend de 31 à 0 sans remonter (aucun SACCH descendant accepté) et le
   mobile jette ~400 blocs en 15 s (« Dropping frame with 54..111 bit
   errors », `mobile.log` de l'archive `20260923-203413`) ; « LOS during
   dedicated mode » à 20:32:45. La tentative suivante (20:32:57) reste sans
   réponse jusqu'à 20:33:13, celle de 20:33:26 est relâchée aussitôt ;
   l'appel de 20:33:36 va au bout. Non reproduit à 20:22.
3. **SACCH descendant du SDCCH/8** : ~la moitié des blocs jetés (7 entre
   20:22:42 et 20:22:47 sur le LU). Suspect : la table 45.002 du BSP pour
   le SACCH/8 sur 102 trames.

Pas une anomalie : les échecs CRC du moniteur TCH descendant du pont tant que
le RTP ne coule pas (décodage du pont, indépendant du DSP) ; le `ko` de
`[a_dd]` ; le tick sauté à `fn=0`.

Reste à valider : rattrapage du pacer pour la parole
(`CALYPSO_PACER_RATTRAPAGE`), rien dans les journaux de ces runs.

## Documentation liée

- [`hw/arm/calypso/doc/SONDES.md`](hw/arm/calypso/doc/SONDES.md) — les sondes
  du cœur C54x (2026-09-18).
- [`tools/rapport-run.sh`](tools/rapport-run.sh) — rapport des journaux
  qemu/osmocon/mobile/bts/pont d'un run ; `LOG_DIR` est lu dans l'environ du
  QEMU vivant.

## Licence

Code dérivé de QEMU 9.2.4 : GPL-2.0-or-later (en-têtes SPDX, `COPYING`).
Le dépôt porte aussi, depuis le 2026-09-23, un `LICENSE` GPLv3 (« qosmo
Copyright (C) 2026 Bastien Baranoff »).
