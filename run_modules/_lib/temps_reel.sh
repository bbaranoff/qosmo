[ -n "${_TEMPS_REEL_LIB_LOADED:-}" ] && return 0
_TEMPS_REEL_LIB_LOADED=1

# Priorité temps réel (SCHED_RR) pour les process qui ont une échéance dure.
#
# [2026-10-04] Le BTS#1 (side-car) s'est arrêté en pleine campagne banc-max :
#     DL1C ERROR PC clock skew: elapsed_us=231775, error_us=227160
#     BTS_SHUTDOWN(bts0): Shutting down BTS, exit 1, reason: PC clock skew too high
# Son timer de trame (4,615 ms) a été servi 227 ms trop tard. En SCHED_OTHER,
# sous charge (load ~11 ce soir-là), un process peut attendre aussi longtemps,
# et osmo-bts-trx se coupe de lui-même au-delà. Le BTS#0 frôle la même limite
# (« We missed 1 timers » en continu dans bts.log).
#
# Par chrt AU LANCEMENT, et pas par « cpu-sched / policy rr 1 » dans le .cfg :
# avec le .cfg, osmo-bts-trx 1.10 REFUSE DE DÉMARRER quand le droit manque
# (« Failed setting SCHED_RR priority 1 », exit 1 - vérifié sans CAP_SYS_NICE
# ni rtprio). Ici on essaie d'abord : sans le droit, on lance comme avant.
# Le droit : root (CAP_SYS_NICE) sur l'hôte ; « --ulimit rtprio=18 » dans les
# conteneurs (start.sh, compose.yaml).
#
#   QOSMO_RT_PRIO=<1-99>  priorité SCHED_RR (défaut 1 : au-dessus de tout
#                         SCHED_OTHER, sous le reste du temps réel)
#   QOSMO_RT_PRIO=0       désactivé : SCHED_OTHER, comme avant

# Préfixe à placer devant la commande : « chrt -r <prio> », ou rien.
# Se développe SANS guillemets :  setsid stdbuf -oL $(rt_prefixe) "$BIN" ...
# chrt fait un exec : le pid lancé reste celui du binaire, et les threads
# qu'il créera héritent de la politique.
rt_prefixe() {
    local prio="${QOSMO_RT_PRIO:-1}"
    [ "$prio" -gt 0 ] 2>/dev/null || return 0
    chrt -r "$prio" true 2>/dev/null && printf 'chrt -r %s\n' "$prio"
    return 0
}

# Ordonnancement effectif d'un process, pour les journaux : « SCHED_RR 1 ».
rt_etat() {   # $1 = pid
    LC_ALL=C chrt -p "$1" 2>/dev/null | awk -F': ' '/policy/ { p = $2 } /priority/ { print p, $2 }'
}
