[ -n "${_TEMPS_REEL_LIB_LOADED:-}" ] && return 0
_TEMPS_REEL_LIB_LOADED=1

# Real-time priority (SCHED_RR) for the processes that have a hard deadline.
#
# osmo-bts-trx shuts itself down ("PC clock skew too high", exit 1) when its
# frame timer (4.615 ms) is served too late; under load a SCHED_OTHER process
# can wait long enough for that to happen, and BTS#0 is close to the same
# limit ("We missed 1 timers" in bts.log).
#
# Done with chrt AT LAUNCH, not with "cpu-sched / policy rr 1" in the .cfg:
# with the .cfg, osmo-bts-trx 1.10 REFUSES TO START when the right is missing
# ("Failed setting SCHED_RR priority 1", exit 1, without CAP_SYS_NICE or
# rtprio). Here it is tried first: without the right, the launch is unchanged.
# The right: root (CAP_SYS_NICE) on the host; "--ulimit rtprio=18" in the
# containers (start-docker.sh, compose.yaml).
#
#   QOSMO_RT_PRIO=<1-99>  SCHED_RR priority (default 1: above everything
#                         SCHED_OTHER, below the rest of real time)
#   QOSMO_RT_PRIO=0       disabled: SCHED_OTHER

# Prefix to put in front of the command: "chrt -r <prio>", or nothing.
# Expands WITHOUT quotes:  setsid stdbuf -oL $(rt_prefixe) "$BIN" ...
# chrt does an exec: the launched pid remains the binary's, and the threads
# it creates inherit the policy.
rt_prefixe() {
    local prio="${QOSMO_RT_PRIO:-1}"
    [ "$prio" -gt 0 ] 2>/dev/null || return 0
    chrt -r "$prio" true 2>/dev/null && printf 'chrt -r %s\n' "$prio"
    return 0
}

# Effective scheduling of a process, for the logs: "SCHED_RR 1".
rt_etat() {   # $1 = pid
    LC_ALL=C chrt -p "$1" 2>/dev/null | awk -F': ' '/policy/ { p = $2 } /priority/ { print p, $2 }'
}
