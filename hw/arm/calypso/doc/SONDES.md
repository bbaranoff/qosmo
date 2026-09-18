# Sonder le DSP Calypso emule : la regle d'or

**Sur ce projet, la premiere hypothese a tester n'est pas « le DSP emule se trompe »
mais « la sonde se trompe ».**

Ce n'est pas une precaution de style. Sur les seules journees des 17 et 18 septembre 2026,
**cinq** diagnostics spectaculaires se sont reveles etre des defauts de mesure, et chacun a
coute des heures de chasse a un bug qui n'existait pas.

| symptome « evident » | cause reelle |
|---|---|
| « aucun etage de la chaine SB ne s'execute » (`b219=0 ... 770a=0`) | les compteurs n'etaient renseignes que dans le chemin de sondage, mais le bilan les imprimait toujours |
| « les bits souples sont reduits a ±1, la magnitude est annihilee » (piste PM/FRCT/ASM/SXM/OVM) | la sonde lisait 0x2a00, le tampon du correlateur FB, au lieu de 0x2c72 |
| « les magnitudes sont saines et l'argmax est fautif » | comparaison en `uint16_t` : les « max » 65502/65104 valaient -34/-432 en `int16_t`, et le rang associe etait du bruit |
| « le champ max signe contient 0x2f06 » | 15 conversions de format pour 13 arguments : toute la fin de ligne glissait d'un cran |
| « FIRS multiplie par la constante 0xf4e4 au lieu du filtre » | lecture de `prog[]` BRUT : avec `PMST_OVLY` arme et le plancher d'alias a 0x0060, une lecture programme dans 0x0060-0x27FF est redirigee vers `data[]`, ou les coefficients sont bien la |

## Les six regles qui en decoulent

1. **Prouver qu'une sonde est active dans le mode ou l'on tourne.** Ne jamais imprimer un
   compteur renseigne dans une seule branche sans ecrire « non mesure » dans l'autre.
2. **Localiser un tampon par difference de RAM**, en photographiant avant et en diffant
   apres, jamais par supposition sur une adresse.
3. **Respecter les types.** La memoire du DSP est un tableau de `uint16_t` qui contient des
   `int16_t` signes. Toute comparaison de magnitude se caste, et pour une grandeur signee on
   donne le maximum en valeur absolue A COTE du maximum signe.
4. **Passer par la traduction d'adresse du coeur**, pas par les tableaux bruts. `prog[]` et
   `data[]` ne sont pas ce que le DSP lit : l'alias OVLY, le banking XPC et les MMR
   s'interposent. Reproduire la traduction dans la sonde, ou appeler la fonction du coeur.
5. **Lire les avertissements du compilateur, pas seulement les erreurs.** Le decalage de la
   liste d'arguments etait signale par `-Wformat` ; il a survecu parce que la sortie de
   `make` etait filtree sur `error:`.
6. **Qualifier un succes dans le code, pas en relisant le log.** Sur ce banc un CRC OK n'est
   un decodage que s'il rend le BSIC INJECTE, un T3 <= 50 ET une FN egale a la trame. Le
   compteur imprime les deux nombres : `CRC OK: N, dont VRAIES: M`.

## La regle du PC : la sonde journalise le PC APRES avancement

Le dispatcheur avance le PC avant que la sonde ne l'observe. **Tous les PC rapportes par les
sondes de ce banc sont donc decales d'une instruction**, et les frontieres de region avec eux.
Les vrais ecrivains du correlateur de midambule sont 0x84bc et 0x84c5, la ou la trace affiche
0x84bd et 0x84c6 ; de meme 0x7cb8 et non 0x7cb9.

C'est la defaillance de mesure la plus insidieuse du lot, parce qu'elle ne produit **aucune
valeur absurde** : tout reste plausible, seulement decale. Les six autres finissaient par
sortir un chiffre impossible ; celle-la, non. Verifier le decalage AVANT de nommer une
instruction fautive.

## Le temoin de zero : la regle qui aurait attrape les cinq

**Definir une zone dont on SAIT qu'elle doit peser zero, et la mesurer a chaque campagne.**
Sur ce banc c'est la marge : les 21 echantillons avant le burst et les 21 apres ne portent
aucune information, donc perturber l'un d'eux ne doit rien changer a la sortie. Mesure :
42 points sur 42 a la valeur plancher, contre 66 a 19534 a l'interieur du burst. Le jour ou
un echantillon de marge se met a peser, ce n'est pas le DSP qui a change, c'est la sonde ou
le protocole de mesure qui est casse — et on le sait AVANT d'avoir bati une hypothese dessus.

C'est exactement ce temoin qui a revele que les « poids absurdes » du §9.30 etaient un
artefact : la perturbation etait appliquee a toutes les trames et modifiait l'historique du
run. Chacune des cinq fausses sondes precedentes aurait ete attrapee par un temoin
equivalent, et chacune a coute un tour complet.

Corollaire de metrique : distinguer un VRAI zero d'un PLANCHER DE SENSIBILITE. Compter les
changements de signe rate tout ce qui ne change que la magnitude, et fait passer une
influence faible pour une absence. Mesurer en max |delta| de valeur, et verifier que le
plancher de la zone temoin est bien separe du plancher de la zone utile.

## Pieges specifiques a ce banc

- `getenv("X")` est VRAI pour `X=0`. Une ligne ecrite `REJEU_SCH_PARTOUT=0 REJEU_SB_FORCE=0`
  ACTIVAIT les hacks. Utiliser un helper qui traite `0`, `""`, `off`, `non` comme faux.
- `REJEU_SCH_PARTOUT=1` **corrompt la verite de reference** : `cellule.c` calcule
  `t3p = p51 / 10` pour tout p51, ce qui n'est juste que sur {1, 11, 21, 31, 41}. La FN ne
  peut pas etre validee sous ce drapeau.
- Le decalage d'echantillonnage GMSK doit valoir **0,5** (centre du symbole). A 0,0
  (frontiere) le voisin suivant pese 0,492 contre 0,454 pour le symbole visee : l'offset
  devient indetermine et TOUT en aval devient instable.
- La fenetre de la SB fait **380 mots** (190 complexes), pas 296. Suivre la longueur que le
  DSP programme (ALGTH), ne pas la figer.

## Piege 8 (18/09, soir) : le filtre de verbosite de `c54x_exe` AVALE les sondes

`c54x_exe` ne se contente pas de relayer stderr : `src/verbosite.c` detourne stderr dans un
tube et classe chaque ligne par MOT-CLEF du nom de la sonde. Une sonde dont le nom contient
`WATCH`, `DUMP`, `SCAN`, `PROBE` ou `MAP` est de niveau 4, `TRACE`, `LOOP`, `CYCLE` de
niveau 5. **Sans `-vvvv` (ou `-vvvvv`), elles ne s'affichent pas du tout.**

Le symptome est le pire possible : la sonde est compilee, active, elle s'execute — et ne
sort RIEN. On conclut « ce code n'est jamais atteint », ce qui est faux. Une sonde de plage
posee sur 0x0000-0xffff a ainsi rapporte ZERO ecriture sur tout l'espace memoire.

**Regle** : avant de conclure d'un silence, relancer avec `-vvvvvv`. Et pour une sonde
ponctuelle dont on veut la sortie inconditionnelle, choisir un nom qui ne contient AUCUN de
ces mots-clefs.

## Piege 9 : « le PC vu moins 1 » n'est PAS universel

La regle du PC (plus haut) dit que les sondes de `c54x_mem.c` journalisent le PC APRES
avancement, donc que l'ecrivain est `pc-1`. **C'est vrai pour certaines familles et faux
pour d'autres.** Mesure : dans la boucle de compaction, la sonde d'ecriture rapporte
`ecr=0x7e70`, or le pas-a-pas montre `0x7e70 = ld` (aucune ecriture) et `0x7e71 = sth`,
le vrai ecrivain. L'ecart d'un cran change donc de sens selon l'instruction.

**Regle** : ne jamais nommer une instruction fautive d'apres le PC d'une sonde memoire.
Confirmer par un pas-a-pas qui imprime le PC AVANT execution avec l'opcode reel
(`CALYPSO_PISTE_LO/HI`, dans `calypso_c54x.c`), et decoder cet opcode contre
`doc/opcodes/tic54x-opc.c`, pas de memoire.

## Les sondes ajoutees le 18/09 au soir

| env | effet |
|---|---|
| `CALYPSO_WATCH_WR_LO/HI/N` | journal chronologique de toute ecriture dans une plage data (idx, valeur, ancienne valeur, PC, insn) |
| `CALYPSO_PISTE_LO/HI/N` | pas-a-pas d'une plage de PC : PC AVANT execution, opcode reel, mot suivant (litteral), A/B/T/AR0-7/BRC, plus les mots pointes par AR2..AR5, DP/CPL/SP et les adresses directes candidates |
| `CALYPSO_PISTE_DUMP_LO/HI` | dump d'une plage data a chaque passage dans la plage de PC |
| `CALYPSO_T_LO/HI` | dans une fenetre d'instructions : chaque changement de T (`PISTE-T`) et chaque annulation d'un accumulateur non nul (`PISTE-NUL`) |

Toutes exigent `-vvvv`/`-vvvvv` (piege 8) et sont inertes tant que les bornes ne sont pas posees.
