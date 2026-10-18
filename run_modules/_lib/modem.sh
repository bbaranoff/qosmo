[ -n "${_MODEM_LIB_LOADED:-}" ] && return 0
_MODEM_LIB_LOADED=1
# _lib/modem.sh - le MS GPRS du banc : le « modem » d'osmocom-bb (GMM/SM/SNDCP/LLC)
# branche comme SECOND MS sur le fake_trx du side-car (BTS#1), a cote de MS#2.
# fake_trx sait servir plusieurs MS (--trx) : aucun multiplexeur maison.
#   BTS#1 (bts 1, TS5-7 PDCH) --TRXD 5720--> fake_trx --6720--> trxcon MS#2 (voix)
#                                                   \--6740--> trxcon modem (GPRS)
#   BTS#1 --/tmp/pcu_bts2--> osmo-pcu (second, nsei 11) --Gb--> osmo-sgsn --> osmo-ggsn (apn internet)
# MODEM_GPRS=0 coupe tout le lot (start-native.sh --no-gprs-modem).
: "${MODEM_GPRS:=1}"
: "${MODEM_BIN:=${OSMOCOM_BB:-${GSM_ROOT:-/opt/GSM}/osmocom-bb}/src/host/layer23/src/modem/modem}"
: "${MODEM_BB_PORT:=6740}"            # base TRXD cote modem (fake_trx --trx ...:6740, trxcon -p 6740)
: "${MODEM_L2_SOCK:=/tmp/modem_l2}"
: "${MODEM_VTY_PORT:=4299}"
: "${MODEM_CFG:=${OSMOCOM_CFG:-/etc/osmocom}/modem-gprs.cfg}"
: "${MODEM_TUN:=tun_modem}"
: "${MODEM_MS_IDX:=3}"                # 3e abonne : IMSI ...000003, MSISDN <noeud>00<op>03
: "${MODEM_APN:=internet}"
: "${MODEM_TRXCON_FN_ADVANCE:=100}"  # -F : extension du delai FBSB (comme MS#2)
: "${MODEM_TRXCON_TRX_ADVANCE:=0}"   # -f : avance de tirage UL ; 0 obligatoire sur PDCH (voir 69-modem-trxcon.sh)
# Second PCU, celui du BTS#1 (le premier sert le BTS principal sur /tmp/pcu_bts).
: "${SC_PCU_UNIT:=osmo-pcu-bts1}"
: "${SC_PCU_SOCKET:=/tmp/pcu_bts2}"
: "${SC_PCU_CFG:=${OSMOCOM_CFG:-/etc/osmocom}/osmo-pcu-bts1.cfg}"
: "${SC_PCU_VTY_HOST:=127.0.0.2}"     # meme port 4240 que le premier PCU, autre adresse
: "${SC_PCU_VTY_PORT:=4240}"
# Gb du BTS#1 (bloc gprs de « bts 1 », cf. 13-sidecar-cfg.sh) : distinct du BTS principal (nsei 10, bvci 12, 23001).
: "${SC_GPRS_NSEI:=11}"
: "${SC_GPRS_NSVCI:=11}"
: "${SC_GPRS_BVCI:=13}"
: "${SC_GPRS_NS_PORT:=23002}"
: "${SC_SGSN_IP:=192.168.2.10}"
: "${SC_SGSN_PORT:=23000}"

modem_enabled() { [ "${MODEM_GPRS:-1}" = 1 ]; }
