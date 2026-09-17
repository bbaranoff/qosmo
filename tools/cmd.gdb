# cmd.gdb — panneau osmocom pour gdb (genere par tools/gdb_cmd.sh, ne pas editer)
# Tape `help_osmo` pour la liste. Tout est en francais, court, et lit les
# symboles du firmware osmocom-bb (layer1.highram.elf).
set pagination off
set confirm off
set print pretty on
set height 0
set width 0
set $API = 0xffd00000

# ── constantes dsp_api / l1_environment.h ────────────────────────────────────
set $B_BLUD = 15
set $B_SCH_CRC = 8
set $B_FIRE0 = 5
set $B_FIRE1 = 6
set $B_TASK_ABORT = 15

# =============================================================================
#  EXECUTION
# =============================================================================
define go
  continue &
end
document go
go — reprend la cible SANS bloquer le prompt (continue &). Ctrl-C = stop.
end

define stop
  interrupt
end
document stop
stop — arrete l'ARM (equivalent de Ctrl-C). Puis regs, bt10, fn, ndb...
end

define regs
  info registers
end
document regs
regs — tous les registres ARM (cible arretee).
end

define pcx
  x/8i $pc
end
document pcx
pcx — 8 instructions a partir de $pc.
end

define bt10
  bt 10
end
document bt10
bt10 — pile d'appels, 10 niveaux.
end

define si1
  stepi
  x/1i $pc
end
document si1
si1 — une instruction, affiche la suivante.
end

define ni1
  nexti
  x/1i $pc
end
document ni1
ni1 — une instruction sans entrer dans les appels.
end

define frame_step
  tbreak tdma_sched_execute
  continue
  fn
end
document frame_step
frame_step — laisse tourner jusqu'a la PROCHAINE trame TDMA (tdma_sched_execute) puis s'arrete et affiche fn.
end

# =============================================================================
#  TEMPS / ORDONNANCEUR
# =============================================================================
define fn
  printf "fn=%u  (t1=%u t2=%u t3=%u)   next fn=%u   tpu_offset=%u corr=%d\n", l1s.current_time.fn, l1s.current_time.t1, l1s.current_time.t2, l1s.current_time.t3, l1s.next_time.fn, l1s.tpu_offset, l1s.tpu_offset_correction
end
document fn
fn — numero de trame courant de la L1 (l1s.current_time), suivant, et offset TPU.
end

define l1
  fn
  printf "cell   : arfcn=%u bsic=%u  fn_offset=%u  time_align=%u\n", l1s.serving_cell.arfcn, l1s.serving_cell.bsic, l1s.serving_cell.fn_offset, l1s.serving_cell.time_alignment
  printf "modes  : fb.mode=%u  pm.mode=%u  dedicated.type=%d tn=%u tsc=%u h=%u\n", l1s.fb.mode, l1s.pm.mode, l1s.dedicated.type, l1s.dedicated.tn, l1s.dedicated.tsc, l1s.dedicated.h
  printf "radio  : ta=%d tx_power=%u tch_mode=%u audio=%u  compl=0x%08x\n", l1s.ta, l1s.tx_power, l1s.tch_mode, l1s.audio_mode, l1s.scheduled_compl
  pages
end
document l1
l1 — resume de l'etat L1 : temps, cellule servante, modes FB/PM/dedie, TA, puissance, pages DSP.
end

define cell
  p l1s.serving_cell
end
document cell
cell — la cellule servante (arfcn, bsic, fn_offset, time_alignment, rxlev...).
end

define neigh
  p l1s.neigh_cell
  printf "neigh_pm : n=%u running=%u pos=%u\n", l1s.neigh_pm.n, l1s.neigh_pm.running, l1s.neigh_pm.pos
end
document neigh
neigh — les 6 voisines connues de la L1 et l'etat des mesures voisines.
end

define sched
  p l1s.tdma_sched
end
document sched
sched — l'ordonnanceur TDMA (les items programmes pour les prochaines trames).
end

define mframe
  p l1s.mframe_sched
end
document mframe
mframe — l'ordonnanceur multitrame (BCCH/CCCH/SDCCH/TCH actifs).
end

define compl
  printf "scheduled_compl = 0x%08x  (bit0 FB, 1 RACH, 2 TX_NB, 3 TCH, 4 TX_SB?, voir l1s.completion[])\n", l1s.scheduled_compl
end
document compl
compl — masque des completions L1 en attente (l1s.scheduled_compl).
end

define arfcn
  printf "rf_arfcn = %u\n", rf_arfcn
end
document arfcn
arfcn — ARFCN courant du frontal radio.
end

define afc
  p afc_state
end
document afc
afc — etat de l'AFC (boucle de correction de frequence).
end

# =============================================================================
#  FB / SB (synchro) — ce que la L1 a vu
# =============================================================================
define fbs
  p fbs
end
document fbs
fbs — etat complet de l'acquisition FB/SB (fbs.mon : bsic, time, toa, pm, angle, snr, attempt ; fbs.req).
end

define mon
  printf "mon : bsic=%u  toa=%d pm=%d angle=%d snr=%u  attempt=%d  fn_report=%u  time fn=%u\n", fbs.mon.bsic, (short)fbs.mon.toa, (short)fbs.mon.pm, (short)fbs.mon.angle, fbs.mon.snr, fbs.mon.attempt, fbs.mon.fnr_report, fbs.mon.time.fn
end
document mon
mon — le dernier resultat FB/SB tel que l'ARM l'a lu (fbs.mon).
end

# =============================================================================
#  MAILBOX ARM <-> DSP (dsp_api : ndb / db_r / db_w)
# =============================================================================
define pages
  printf "pages  : r_page=%d w_page=%d r_page_used=%d frame_ctr=%d   ndb.d_dsp_page=0x%04x\n", dsp_api.r_page, dsp_api.w_page, dsp_api.r_page_used, dsp_api.frame_ctr, dsp_api.ndb->d_dsp_page & 0xffff
  printf "         db_r=%p  db_w=%p  ndb=%p  param=%p\n", dsp_api.db_r, dsp_api.db_w, dsp_api.ndb, dsp_api.param
end
document pages
pages — pages de la double mise en tampon MCU<->DSP (r_page/w_page vus par l'ARM, d_dsp_page vu par le DSP).
end

define err
  set $e = dsp_api.ndb->d_error_status & 0xffff
  printf "d_error_status = 0x%04x", $e
  if $e & 0x0008
    printf "  DSP_ERR_DMA_PROG"
  end
  if $e & 0x0010
    printf "  DSP_ERR_DMA_TASK"
  end
  printf "\n"
end
document err
err — d_error_status du DSP, decode (8 = DMA_PROG, 0x10 = DMA_TASK ; temoin, pas un blocage).
end

define taskname
  if $arg0 == 0
    printf "-"
  end
  if $arg0 == 1
    printf "PM"
  end
  if $arg0 == 5
    printf "FB"
  end
  if $arg0 == 6
    printf "SB"
  end
  if $arg0 == 10
    printf "RACH"
  end
  if $arg0 == 19
    printf "NBS"
  end
  if $arg0 == 21
    printf "NP"
  end
  if $arg0 == 24
    printf "ALLC"
  end
  if $arg0 == 33
    printf "CHECKSUM"
  end
end

define tasks
  set $td = dsp_api.db_w->d_task_d & 0xffff
  set $tu = dsp_api.db_w->d_task_u & 0xffff
  set $tm = dsp_api.db_w->d_task_md & 0xffff
  set $tr = dsp_api.db_w->d_task_ra & 0xffff
  printf "db_w (ARM->DSP) : task_d=%u(", $td
  taskname $td
  printf ") burst_d=%u  task_u=%u(", dsp_api.db_w->d_burst_d & 0xffff, $tu
  taskname $tu
  printf ") burst_u=%u  task_md=%u(", dsp_api.db_w->d_burst_u & 0xffff, $tm
  taskname $tm
  printf ")  task_ra=%u\n", $tr
  printf "                 d_fn=0x%04x ctrl_abb=0x%04x afc=%d(0x%04x) ctrl_system=0x%04x power_ctl=0x%04x ctrl_tch=0x%04x\n", dsp_api.db_w->d_fn & 0xffff, dsp_api.db_w->d_ctrl_abb & 0xffff, (short)(dsp_api.db_w->d_afc & 0xffff), dsp_api.db_w->d_afc & 0xffff, dsp_api.db_w->d_ctrl_system & 0xffff, dsp_api.db_w->d_power_ctl & 0xffff, dsp_api.db_w->d_ctrl_tch & 0xffff
  set $rd = dsp_api.db_r->d_task_d & 0xffff
  printf "db_r (DSP->ARM) : task_d=%u(", $rd
  taskname $rd
  printf ") burst_d=%u task_u=%u burst_u=%u task_md=%u task_ra=%u\n", dsp_api.db_r->d_burst_d & 0xffff, dsp_api.db_r->d_task_u & 0xffff, dsp_api.db_r->d_burst_u & 0xffff, dsp_api.db_r->d_task_md & 0xffff, dsp_api.db_r->d_task_ra & 0xffff
end
document tasks
tasks — les taches programmees par l'ARM (page W) et celles que le DSP a rendues (page R), avec leur nom (FB, SB, PM, NBS, ALLC...).
end

define dbw
  p *dsp_api.db_w
end
document dbw
dbw — la page d'ecriture ARM->DSP complete (d_task_*, d_fn, d_ctrl_abb, a_a5fn, d_afc, d_ctrl_system...).
end

define dbr
  p *dsp_api.db_r
end
document dbr
dbr — la page de lecture DSP->ARM complete (taches rendues, a_serv_demod, a_pm, a_sch).
end

define sync
  printf "ndb  : d_fb_det=%u d_fb_mode=%u  a_sync_demod TOA=%d PM=%d ANGLE=%d SNR=%u\n", dsp_api.ndb->d_fb_det & 0xffff, dsp_api.ndb->d_fb_mode & 0xffff, (short)(dsp_api.ndb->a_sync_demod[0] & 0xffff), (short)(dsp_api.ndb->a_sync_demod[1] & 0xffff), (short)(dsp_api.ndb->a_sync_demod[2] & 0xffff), dsp_api.ndb->a_sync_demod[3] & 0xffff
  printf "db_r : a_serv_demod TOA=%d PM=%d(>>3=%d) ANGLE=%d SNR=%u\n", (short)(dsp_api.db_r->a_serv_demod[0] & 0xffff), (short)(dsp_api.db_r->a_serv_demod[1] & 0xffff), (short)(dsp_api.db_r->a_serv_demod[1] & 0xffff) >> 3, (short)(dsp_api.db_r->a_serv_demod[2] & 0xffff), dsp_api.db_r->a_serv_demod[3] & 0xffff
end
document sync
sync — resultats de synchro : d_fb_det + a_sync_demod (NDB, chemin FB) et a_serv_demod (page R, chemin SB/NB).
end

define sb
  set $s0 = dsp_api.db_r->a_sch[0] & 0xffff
  set $sbw = (dsp_api.db_r->a_sch[3] & 0xffff) | ((dsp_api.db_r->a_sch[4] & 0xffff) << 16)
  printf "a_sch = %04x %04x %04x %04x %04x   ", $s0, dsp_api.db_r->a_sch[1] & 0xffff, dsp_api.db_r->a_sch[2] & 0xffff, dsp_api.db_r->a_sch[3] & 0xffff, dsp_api.db_r->a_sch[4] & 0xffff
  if $s0 & (1 << $B_SCH_CRC)
    printf "CRC FAUX (B_SCH_CRC)"
  else
    printf "CRC ok"
  end
  if $s0 & (1 << $B_BLUD)
    printf "  BLUD"
  end
  set $t1 = (($sbw >> 23) & 1) | (($sbw >> 7) & 0x1fe) | (($sbw << 9) & 0x600)
  set $t2 = ($sbw >> 18) & 0x1f
  set $t3p = (($sbw >> 24) & 1) | (($sbw >> 15) & 6)
  set $t3 = $t3p * 10 + 1
  printf "\nSB 0x%08x : BSIC=%u  t1=%u t2=%u t3=%u  (fn = 51*26*t1 + 51*((t3-t2+26)%%26) + t3 = %u)\n", $sbw, ($sbw >> 2) & 0x3f, $t1, $t2, $t3, 51*26*$t1 + 51*(($t3 - $t2 + 26) % 26) + $t3
end
document sb
sb — decode a_sch comme prim_fbsb.c : CRC, mot SB, BSIC, t1/t2/t3 et fn.
end

define cd
  set $c0 = dsp_api.ndb->a_cd[0] & 0xffff
  printf "a_cd[0]=0x%04x", $c0
  if $c0 & (1 << $B_BLUD)
    printf " BLUD"
  end
  printf "  fire_crc=%u  biterr=%u\n", ($c0 & ((1 << $B_FIRE1) | (1 << $B_FIRE0))) >> $B_FIRE0, dsp_api.ndb->a_cd[2] & 0xffff
  printf "a_cd[1]=0x%04x  data[3..14] = ", dsp_api.ndb->a_cd[1] & 0xffff
  set $i = 3
  while $i < 15
    printf "%04x ", dsp_api.ndb->a_cd[$i] & 0xffff
    set $i = $i + 1
  end
  printf "\n"
end
document cd
cd — le bloc de donnees decode (a_cd : etat BLUD, CRC Fire, erreurs bits, 23 octets du bloc L2/SI).
end

define pm
  printf "a_pm = %d %d %d   (dBm8 : %d)\n", (short)(dsp_api.db_r->a_pm[0] & 0xffff), (short)(dsp_api.db_r->a_pm[1] & 0xffff), (short)(dsp_api.db_r->a_pm[2] & 0xffff), (short)(dsp_api.db_r->a_pm[0] & 0xffff) >> 3
end
document pm
pm — les 3 mesures de puissance rendues par le DSP (page R).
end

define ndb
  err
  sync
  printf "d_dsp_state=0x%04x  bg enable=0x%04x state=0x%04x abort=0x%04x  d_tch_mode=0x%04x  d_a5mode=0x%04x\n", dsp_api.ndb->d_dsp_state & 0xffff, dsp_api.ndb->d_background_enable & 0xffff, dsp_api.ndb->d_background_state & 0xffff, dsp_api.ndb->d_background_abort & 0xffff, dsp_api.ndb->d_tch_mode & 0xffff, dsp_api.ndb->d_a5mode & 0xffff
  printf "d_dsp_page=0x%04x  version=%04x/%04x  d_rach=0x%04x\n", dsp_api.ndb->d_dsp_page & 0xffff, dsp_api.ndb->d_version_number1 & 0xffff, dsp_api.ndb->d_version_number2 & 0xffff, dsp_api.ndb->d_rach & 0xffff
end
document ndb
ndb — les cellules NDB qui comptent : erreur, synchro, etat DSP, taches de fond, page, version, RACH.
end

define dsp
  pages
  tasks
  ndb
  sb
end
document dsp
dsp — vue d'ensemble de la mailbox : pages + taches + ndb + sb. La commande a taper d'abord.
end

define apiram
  x/$arg1hx ($API + 2*($arg0 - 0x800))
end
document apiram
apiram OFF N — N mots de la memoire API vus de l'ARM, a partir du mot DSP OFF (0xffd00000 = mot 0x0800 ; ex : apiram 0x800 32 = page 0 ; apiram 0x8f8 8 = d_fb_det...).
end

define apiw
  set *(unsigned short *)($API + 2*($arg0 - 0x800)) = $arg1
  printf "api[0x%04x] <- 0x%04x\n", $arg0, $arg1
end
document apiw
apiw OFF VAL — ecrit le mot DSP OFF (adresse cote DSP, ex 0x08f8) dans la memoire API partagee.
end

define rxnb
  p rxnb
end
document rxnb
rxnb — etat de la reception des bursts normaux (rxnb : meas[4], msg en cours).
end

# =============================================================================
#  FORCAGES / FAUX RESULTATS  (ecrivent dans la mailbox : a utiliser cible ARRETEE)
# =============================================================================
define fake_fb
  set dsp_api.ndb->d_fb_det = 1
  set dsp_api.ndb->a_sync_demod[0] = $arg0
  set dsp_api.ndb->a_sync_demod[1] = $arg1
  set dsp_api.ndb->a_sync_demod[2] = $arg2
  set dsp_api.ndb->a_sync_demod[3] = $arg3
  printf "FB force : d_fb_det=1 toa=%d pm=%d angle=%d snr=%d\n", $arg0, $arg1, $arg2, $arg3
  sync
end
document fake_fb
fake_fb TOA PM ANGLE SNR — fait croire a l'ARM que le DSP a detecte un FB (ex : fake_fb 48 5511 -2 200).
end

define fake_sb
  set $t1 = $arg1
  set $t2 = $arg2
  set $t3p = ($arg3 - 1) / 10
  set $w = (($arg0 & 0x3f) << 2) | (($t1 & 1) << 23) | (($t1 & 0x1fe) << 7) | (($t1 & 0x600) >> 9) | (($t2 & 0x1f) << 18) | (($t3p & 1) << 24) | (($t3p & 6) << 15)
  set dsp_api.db_r->a_sch[0] = 0
  set dsp_api.db_r->a_sch[3] = $w & 0xffff
  set dsp_api.db_r->a_sch[4] = ($w >> 16) & 0xffff
  printf "SB force : 0x%08x  bsic=%u t1=%u t2=%u t3=%u\n", $w, $arg0, $t1, $t2, $arg3
  sb
end
document fake_sb
fake_sb BSIC T1 T2 T3 — compose un mot SB valide (CRC ok) dans a_sch pour que l'ARM decode ce BSIC et ce temps (ex : fake_sb 7 100 5 11).
end

define sb_force
  set dsp_api.db_r->a_sch[0] = dsp_api.db_r->a_sch[0] & ~(1 << $B_SCH_CRC)
  printf "B_SCH_CRC efface : l'ARM decodera a_sch[3..4] tel quel\n"
  sb
end
document sb_force
sb_force — efface le bit CRC de a_sch[0] : l'ARM accepte le mot SB present, meme si le DSP l'a marque faux.
end

define si_force
  set dsp_api.ndb->a_cd[0] = (1 << $B_BLUD)
  set dsp_api.ndb->a_cd[2] = 0
  printf "a_cd force : BLUD, fire_crc=0, biterr=0 -> l'ARM prend le bloc present comme bon\n"
  cd
end
document si_force
si_force — force l'acceptation du bloc a_cd present (BLUD + CRC Fire ok + 0 erreur) : l'ARM remonte ce SI/bloc a la L2.
end

define fake_cd
  set dsp_api.ndb->a_cd[3] = $arg0
  set dsp_api.ndb->a_cd[4] = $arg1
  set dsp_api.ndb->a_cd[5] = $arg2
  set dsp_api.ndb->a_cd[6] = $arg3
  set dsp_api.ndb->a_cd[7] = $arg4
  set dsp_api.ndb->a_cd[8] = $arg5
  set dsp_api.ndb->a_cd[9] = $arg6
  set dsp_api.ndb->a_cd[10] = $arg7
  set dsp_api.ndb->a_cd[11] = $arg8
  set dsp_api.ndb->a_cd[12] = $arg9
  si_force
end
document fake_cd
fake_cd W3 W4 ... W12 — pose 10 mots de donnees dans a_cd[3..12] puis si_force (les 2 derniers mots restent).
end

define fake_err
  set dsp_api.ndb->d_error_status = $arg0
  err
end
document fake_err
fake_err VAL — ecrit d_error_status (ex : fake_err 0 pour effacer DMA_PROG).
end

define fake_task
  set dsp_api.db_w->d_task_d = $arg0
  printf "db_w->d_task_d <- %u\n", $arg0
  tasks
end
document fake_task
fake_task N — programme la tache DL N dans la page W (5=FB 6=SB 1=PM 24=ALLC 19=NBS 21=NP 10=RACH).
end

define fake_page
  set dsp_api.ndb->d_dsp_page = $arg0
  pages
end
document fake_page
fake_page VAL — force d_dsp_page (bit0 = page W du DSP, bit1 = ...) pour tester la selection de page.
end

define abort_dsp
  set dsp_api.db_w->d_ctrl_system = dsp_api.db_w->d_ctrl_system | (1 << $B_TASK_ABORT)
  printf "B_TASK_ABORT pose dans d_ctrl_system (page W)\n"
end
document abort_dsp
abort_dsp — pose B_TASK_ABORT : demande au DSP d'abandonner sa tache en cours (ce que fait l1s_dsp_abort).
end

# =============================================================================
#  POINTS D'ARRET / SURVEILLANCE
# =============================================================================
define bfb
  break l1s_fbdet_resp
end
document bfb
bfb — s'arrete a chaque reponse de detection FB (l1s_fbdet_resp). Puis : mon, sync, fn.
end

define bsb
  break l1s_sbdet_resp
end
document bsb
bsb — s'arrete a chaque reponse SB (l1s_sbdet_resp). Puis : sb, mon.
end

define bfbsb
  break l1s_fbsb_req
end
document bfbsb
bfbsb — s'arrete quand la L1 recoit une demande FBSB du mobile (l1s_fbsb_req).
end

define bsync
  break synchronize_tdma
end
document bsync
bsync — s'arrete au recalage TDMA (synchronize_tdma), juste apres un FB/SB accepte.
end

define bnb
  break l1s_nb_resp
end
document bnb
bnb — s'arrete a chaque bloc normal recu (l1s_nb_resp). Puis : cd, rxnb.
end

define bpm
  break l1s_pm_resp
end
document bpm
bpm — s'arrete a chaque mesure de puissance rendue (l1s_pm_resp). Puis : pm.
end

define breset
  break l1s_reset
end
document breset
breset — s'arrete a chaque reset L1 (l1s_reset) : qui remet la L1 a zero et quand.
end

define bcompl
  break l1s_compl_sched
end
document bcompl
bcompl — s'arrete a chaque completion programmee (l1s_compl_sched) : voir compl.
end

define babort
  break l1s_dsp_abort
end
document babort
babort — s'arrete quand l'ARM abandonne la tache DSP (l1s_dsp_abort).
end

define bframe
  break tdma_sched_execute
end
document bframe
bframe — s'arrete a CHAQUE trame TDMA (tdma_sched_execute). Lourd : utiliser avec trace_frames.
end

define bl2
  break l1_queue_for_l2
end
document bl2
bl2 — s'arrete quand la L1 remonte un message a la L2 (SI, bloc, mesure).
end

define bafc
  break afc_input
end
document bafc
bafc — s'arrete a chaque entree AFC (afc_input freq_diff).
end

define wfb
  watch dsp_api.ndb->d_fb_det
end
document wfb
wfb — point d'arret sur ECRITURE de d_fb_det (vue ARM ; les ecritures du DSP ne passent pas par le bus ARM, seules celles de l'ARM declenchent).
end

define werr
  watch dsp_api.ndb->d_error_status
end
document werr
werr — point d'arret sur ecriture ARM de d_error_status.
end

define wtask
  watch dsp_api.db_w->d_task_d
end
document wtask
wtask — point d'arret sur programmation d'une tache DL par l'ARM.
end

define bl
  info breakpoints
end
document bl
bl — liste des points d'arret/surveillance.
end

define bclr
  delete
  printf "tous les points d'arret supprimes\n"
end
document bclr
bclr — supprime tous les points d'arret.
end

# =============================================================================
#  BOUCLES DE TRACE  (cible en avant-plan : Ctrl-C interrompt la boucle)
# =============================================================================
define trace_frames
  set $n = $arg0
  tbreak tdma_sched_execute
  continue
  while $n > 0
    fn
    tasks
    set $n = $n - 1
    if $n > 0
      tbreak tdma_sched_execute
      continue
    end
  end
  printf "-- %d trames tracees, cible ARRETEE (go pour reprendre)\n", $arg0
end
document trace_frames
trace_frames N — s'arrete N trames de suite (tdma_sched_execute) et affiche fn + taches a chaque fois. Ctrl-C pour couper. go apres.
end

define trace_fb
  set $n = $arg0
  while $n > 0
    tbreak l1s_fbdet_resp
    continue
    fn
    sync
    set $n = $n - 1
  end
  printf "-- %d reponses FB tracees, cible ARRETEE (go pour reprendre)\n", $arg0
end
document trace_fb
trace_fb N — attend N reponses FB (l1s_fbdet_resp), affiche fn + sync a chacune.
end

define trace_sb
  set $n = $arg0
  while $n > 0
    tbreak l1s_sbdet_resp
    continue
    fn
    sb
    set $n = $n - 1
  end
  printf "-- %d reponses SB tracees, cible ARRETEE (go pour reprendre)\n", $arg0
end
document trace_sb
trace_sb N — attend N reponses SB (l1s_sbdet_resp), affiche fn + decodage a_sch a chacune.
end

define trace_nb
  set $n = $arg0
  while $n > 0
    tbreak l1s_nb_resp
    continue
    fn
    cd
    set $n = $n - 1
  end
  printf "-- %d blocs traces, cible ARRETEE (go pour reprendre)\n", $arg0
end
document trace_nb
trace_nb N — attend N blocs normaux (l1s_nb_resp), affiche fn + a_cd a chacun.
end

define trace_l2
  set $n = $arg0
  while $n > 0
    tbreak l1_queue_for_l2
    continue
    fn
    printf "msg -> L2 : "
    p *(struct msgb *)$r0
    set $n = $n - 1
  end
  printf "-- %d messages L2 traces, cible ARRETEE (go pour reprendre)\n", $arg0
end
document trace_l2
trace_l2 N — attend N messages remontes a la L2 (l1_queue_for_l2) et les affiche.
end

define mon_dsp
  set $n = $arg0
  while $n > 0
    tbreak tdma_sched_execute
    continue
    printf "==== trame %u ====\n", l1s.current_time.fn
    dsp
    set $n = $n - 1
  end
  printf "-- %d echantillons, cible ARRETEE (go pour reprendre)\n", $arg0
end
document mon_dsp
mon_dsp N — N trames de suite, la vue `dsp` complete a chacune (pages, taches, ndb, sb).
end

define every
  set $n = $arg0
  while $n > 0
    tbreak tdma_sched_execute
    continue
    $arg1
    set $n = $n - 1
  end
  printf "-- fini, cible ARRETEE (go pour reprendre)\n"
end
document every
every N CMD — execute la commande CMD a chacune des N prochaines trames (ex : every 20 sync).
end

# =============================================================================
#  AIDE
# =============================================================================
define help_osmo
  printf "\n============ PANNEAU OSMOCOM (cmd.gdb) ============\n"
  printf "EXECUTION       go            reprend sans bloquer (continue &)\n"
  printf "                stop          arrete l'ARM (= Ctrl-C)\n"
  printf "                regs / pcx / bt10 / si1 / ni1   registres, code a $pc, pile, pas a pas\n"
  printf "                frame_step    tourne jusqu'a la prochaine trame TDMA puis s'arrete\n"
  printf "TEMPS           fn            trame courante, suivante, tpu_offset\n"
  printf "                l1            resume L1 : temps, cellule, modes, TA, pages\n"
  printf "                cell / neigh / sched / mframe / compl / arfcn / afc\n"
  printf "SYNCHRO         fbs / mon     etat FB/SB vu par l'ARM (fbs, fbs.mon)\n"
  printf "                sync          d_fb_det + a_sync_demod + a_serv_demod\n"
  printf "                sb            a_sch decode : CRC, BSIC, t1/t2/t3, fn\n"
  printf "MAILBOX         dsp           TOUT : pages + tasks + ndb + sb\n"
  printf "                pages / tasks / dbw / dbr / ndb / err / cd / pm / rxnb\n"
  printf "                apiram OFF N  mots API bruts (adresse cote DSP)     apiw OFF VAL  ecrit un mot\n"
  printf "FORCAGES        fake_fb TOA PM ANGLE SNR   un FB detecte\n"
  printf "                fake_sb BSIC T1 T2 T3      un SB valide            sb_force   efface le CRC faux\n"
  printf "                si_force                   accepte le bloc a_cd    fake_cd W3..W12  pose 10 mots + si_force\n"
  printf "                fake_err V / fake_task N / fake_page V / abort_dsp\n"
  printf "POINTS D'ARRET  bfb bsb bfbsb bsync bnb bpm breset bcompl babort bframe bl2 bafc\n"
  printf "                wfb werr wtask (watch)      bl liste     bclr efface tout\n"
  printf "BOUCLES         trace_frames N / trace_fb N / trace_sb N / trace_nb N / trace_l2 N\n"
  printf "                mon_dsp N     la vue dsp a chaque trame, N fois\n"
  printf "                every N CMD   CMD a chaque trame, N fois (ex : every 20 sync)\n"
  printf "                Ctrl-C coupe une boucle ; la cible reste ARRETEE : go pour reprendre.\n"
  printf "AIDE            help_osmo     ceci      help CMD   le detail d'une commande\n"
  printf "Regle : les lectures (fn, dsp...) exigent la cible ARRETEE (stop) ; go ensuite.\n\n"
end
document help_osmo
help_osmo — liste toutes les commandes du panneau osmocom, avec leur role et leur usage.
end

printf "[cmd.gdb] panneau osmocom charge : help_osmo\n"
