# La PLATEFORME Calypso, et rien d'autre.
#
# [2026-09-16] Aucune couche 1 dans cette liste, et c'est la regle qui definit
# ce depot : une L1 ajoutee ici casserait la raison d'etre du split (voir
# include/hw/arm/calypso/calypso_l1_ops.h). Un fork qui derive de qosmo ajoute
# SA L1 via sa propre entree arm_ss.add() - il ne modifie pas ce fichier.
arm_ss.add(when: 'CONFIG_CALYPSO', if_true: files(
  'calypso_mb.c',
  'calypso_soc.c',
  'calypso_trx.c',
  'calypso_tpu.c',
  'calypso_tsp.c',
  'calypso_trf6151.c',
  'calypso_sim.c',
  'calypso_l1_dispatch.c',
))

# ─── DEVICES DE LA CARTE E88 : PRESENTS DANS L'ARBRE, PAS DANS LE BUILD ──────
#
# [2026-09-16] calypso_iota.c, calypso_rf3166.c, calypso_asm4532.c,
# calypso_xio.c, calypso_debug.c, calypso_invariants.c, fw_console.c,
# sercomm_gate.c et l1ctl_sock.c ont ete rapatries depuis qosmo-dsp, qui les a
# gardes alors que qosmo-grgsm les avait perdus au split (55df780, « INTH/SPI/
# UART silencieux »). Aucun n'inclut calypso_c54x.h ni ne touche C54xState :
# ils COMPILENT et se LIENT dans cette base, verifie le 2026-09-16.
#
# Ils n'y sont pourtant pas actives, parce que compiler n'est pas brancher :
# dans qosmo-dsp leur unique point d'entree est calypso_pcb_init() /
# calypso_pcb_start_threads(), qui vivent dans calypso_full_pcb.c - et CE
# fichier-la prend un C54xState*. Les activer ici donnerait neuf fichiers de
# code mort : mesure faite, aucun appelant hors de l'ilot ferme que forment
# calypso_asm4532.c et calypso_rf3166.c, qui ne s'appellent que l'un l'autre.
#
# CE QUI LES DEBLOQUE, dans cet ordre :
#   1. decoupler calypso_full_pcb.c de C54xState (il n'en utilise le pointeur
#      que pour reveiller le DSP - candidat naturel a une entree de la vtable
#      CalypsoL1Ops plutot qu'a un type concret) ;
#   2. le remonter ici, avec calypso_twl3025.c ;
#   3. appeler calypso_pcb_init() depuis calypso_soc_realize() ;
#   4. alors seulement ajouter les neuf ci-dessus a la liste du haut.
#
# Seul l1ctl_sock.c a un branchement direct et propre dans qosmo-dsp
# (l1ctl_sock_init(&s->uart_modem, getenv("L1CTL_SOCK") ?: "/tmp/osmocom_l2")
# en fin de realize) : c'est le premier a remonter, independamment du PCB.
#
# RESTENT DANS qosmo-dsp en attendant : calypso_rif.c (481 l.),
# calypso_rhea_dma.c (689 l.), calypso_full_pcb.c (210 l.), calypso_tint0.c
# (134 l.), calypso_twl3025.c (258 l.).

# ─── CHOIX DE LA COUCHE 1, AU CONFIGURE ─────────────────────────────────────
#
# [2026-09-16] ./configure --enable-l1-grgsm   (ou --enable-l1-dsp)
#
# C'est CE mecanisme qui remplace le split en deux depots. Avant, choisir sa
# couche 1 voulait dire choisir son arbre git - d'ou deux arbres sans commit
# racine commun, donc sans base de fusion, donc 9 des 10 fichiers communs
# diverges en deux semaines. Ici les deux L1 cohabitent dans le meme arbre et
# ne se disputent qu'une ligne de configure.
#
# Par defaut aucune des deux : la base seule se construit et boote (la machine
# s'annonce « couche 1 : aucune »), ce qui rend la plateforme testable sans
# embarquer ni gr-gsm ni le C54x.
l1_grgsm = get_option('l1_grgsm')
l1_dsp = get_option('l1_dsp')

if l1_grgsm.enabled() and l1_dsp.enabled()
  error('calypso: --enable-l1-grgsm et --enable-l1-dsp sont exclusives. ' +
        'Les deux liees ensemble, c\'est deux appels a calypso_l1_register() ' +
        'et la derniere gagne en silence. Choisir l\'une des deux.')
endif

if l1_grgsm.enabled()
  subdir('l1-grgsm')
endif

if l1_dsp.enabled()
  if not fs.is_dir(meson.current_source_dir() / 'l1-dsp')
    error('calypso: --enable-l1-dsp demande, mais hw/arm/calypso/l1-dsp/ est ' +
          'absent.')
  endif
  # [2026-09-16] Les devices de la carte E88 deviennent atteignables : c'est
  # calypso_full_pcb.c (dans l1-dsp/) qui les cable, et il n'est la que sous
  # cette option. C'est precisement le couplage a casser pour qu'ils servent
  # aussi a la L1 gr-gsm - voir le bloc « PRESENTS DANS L'ARBRE » ci-dessus.
  arm_ss.add(when: 'CONFIG_CALYPSO', if_true: files(
    'calypso_iota.c',
    'calypso_rf3166.c',
    'calypso_asm4532.c',
    'calypso_xio.c',
    'calypso_debug.c',
    'calypso_invariants.c',
    'fw_console.c',
    'sercomm_gate.c',
    'l1ctl_sock.c',
  ))
  subdir('l1-dsp')
endif
