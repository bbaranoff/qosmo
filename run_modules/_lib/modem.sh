[ -n "${_MODEM_LIB_LOADED:-}" ] && return 0
_MODEM_LIB_LOADED=1
# _lib/modem.sh - the bench's GPRS MS: the osmocom-bb "modem" (GMM/SM/SNDCP/LLC)
# attached as a SECOND MS on the side-car's fake_trx (BTS#1), next to MS#2.
# fake_trx can serve several MS (--trx): no home-made multiplexer.
#   BTS#1 (bts 1, TS5-7 PDCH) --TRXD 5720--> fake_trx --6720--> trxcon MS#2 (voice)
#                                                   \--6740--> trxcon modem (GPRS)
#   BTS#1 --/tmp/pcu_bts2--> osmo-pcu (second, nsei 11) --Gb--> osmo-sgsn --> osmo-ggsn (apn internet)
# MODEM_GPRS=0 disables the whole set (start-native.sh --no-gprs-modem).
: "${MODEM_GPRS:=1}"
: "${MODEM_BIN:=${OSMOCOM_BB:-${GSM_ROOT:-/opt/GSM}/osmocom-bb}/src/host/layer23/src/modem/modem}"
: "${MODEM_BB_PORT:=6740}"            # TRXD base on the modem side (fake_trx --trx ...:6740, trxcon -p 6740)
: "${MODEM_L2_SOCK:=/tmp/modem_l2}"
: "${MODEM_VTY_PORT:=4299}"
: "${MODEM_CFG:=${OSMOCOM_CFG:-/etc/osmocom}/modem-gprs.cfg}"
: "${MODEM_TUN:=tun_modem}"
: "${MODEM_MS_IDX:=3}"                # 3rd subscriber: IMSI ...000003, MSISDN <node>00<op>03
: "${MODEM_APN:=internet}"
: "${MODEM_TRXCON_FN_ADVANCE:=100}"  # -F: FBSB timeout extension (as for MS#2)
: "${MODEM_TRXCON_TRX_ADVANCE:=0}"   # -f: UL pull advance; 0 required on PDCH (see 69-modem-trxcon.sh)
# Second PCU, the one of BTS#1 (the first serves the main BTS on /tmp/pcu_bts).
: "${SC_PCU_UNIT:=osmo-pcu-bts1}"
: "${SC_PCU_SOCKET:=/tmp/pcu_bts2}"
: "${SC_PCU_CFG:=${OSMOCOM_CFG:-/etc/osmocom}/osmo-pcu-bts1.cfg}"
: "${SC_PCU_VTY_HOST:=127.0.0.2}"     # same port 4240 as the first PCU, other address
: "${SC_PCU_VTY_PORT:=4240}"
# Gb of BTS#1 (gprs block of "bts 1", cf. 13-sidecar-cfg.sh): distinct from the main BTS (nsei 10, bvci 12, 23001).
: "${SC_GPRS_NSEI:=11}"
: "${SC_GPRS_NSVCI:=11}"
: "${SC_GPRS_BVCI:=13}"
: "${SC_GPRS_NS_PORT:=23002}"
: "${SC_SGSN_IP:=192.168.2.10}"
: "${SC_SGSN_PORT:=23000}"

modem_enabled() { [ "${MODEM_GPRS:-1}" = 1 ]; }
