#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# injecteur_bruit.py - UDP proxy that degrades the DOWNLINK direction between the
# bridge (osmo-operator/pont) and the layer 1 of the emulated phone.
#
# Single source file:
#   /opt/GSM/qosmo/tools/injecteur_bruit.py
# (c54x_exe/run.sh takes it from there, with a fallback on
# c54x_exe/tools/injecteur_bruit.py, a link to this file;
# qosmo/run_modules/38-bruit.sh starts it for the qosmo plan).
#
# WHERE IT SITS
#   dsp target  : pont_dsp.py --dsp-port 6702 -> [injector 127.0.0.1:6702]
#                 -> c54x_exe BSP moved to CALYPSO_BSP_PORT (16702 by default).
#                 Format: 8-byte header [tn, fn BE32, att, 0, 0] then 148
#                 bytes of HARD bits 0/1 (pont/trx.py Trx.run_data;
#                 calypso_bsp.c bsp_trxd_readable).
#                 The BSP learns the address of its uplink peer from the sender
#                 of the downlink: once the injector sits in between, what the
#                 BSP sends back (UL TRXDv0 bursts, if any) arrives here and
#                 leaves UNMODIFIED towards the source (the bridge,
#                 127.0.0.1:5702), from the listening port -- exactly the
#                 address the BSP had before.
#   grgsm target: pont.py -> GSMTAP [injector 127.0.0.1:14730] -> gr-gsm L1 of
#                 QEMU (127.0.0.1:4730, port fixed in calypso_l1_grgsm.c).
#                 The bridge sends there when PONT_GSMTAP_PORT=14730
#                 (pont/config.py; pont/pont.py sets it by itself when
#                 BRUIT_MODE is ber|relais).
#                 Format: 16-byte GSMTAP v2 header then the ALREADY DECODED L2
#                 block (23 bytes) -- NOT bursts, NO soft values.
#                 The SCH (4731, "SCH2") and TCH speech (/dev/shm/calypso_tch_dl)
#                 do not go through here and are not touched.
#
# MODES (downlink only; the uplink is always relayed as is)
#   relais  no alteration: checks that the insertion alone changes nothing.
#   ber     random bit inversion, mean rate --ber. --rafales L: clustered
#           errors (Gilbert-Elliott, bursts of L bits on average, rate
#           --ber-rafale inside a burst), same mean rate.
#           dsp: hard bits inverted (0<->1). grgsm: bits of the L2 bytes
#           inverted AFTER decoding -- not physical (neither the error
#           correcting code nor the CRC protect the block any more); --perte
#           is the realistic model for grgsm.
#   souple  AWGN at --snr-db on the soft values. Neither stream carries any
#           (dsp: hard bits 0/1; grgsm: L2 bytes). On hard bits: antipodal
#           +-1 modulation, AWGN, hard decision -- BER = Q(sqrt(2 snr)),
#           announced at start-up; the output stays 0/1, the only format the
#           BSP can read. If a source sends int8 soft values (TRXD convention:
#           +127 = bit 1), they are noised and kept soft. Refused for grgsm.
#   iq      EXPERIMENTAL. Bits -> GMSK BT=0.3 at 4 samples/symbol (same
#           modulator as calypso_gmsk.c, instant 0.5) + AWGN at --snr-db, sent
#           as interleaved int16 I/Q (BSP IQ_PASSTHROUGH, decimation by 4).
#           BYPASSES the BSP's tuned modulation (sb_moduler for the SCH:
#           instant 0.35, widening 1.0, own noise; gmsk_elargir 0.3 on the
#           NBs -- only the latter is imitated here, --iq-elargir).
#           INCOMPATIBLE with CALYPSO_BSP_STREAM=1 (the default of
#           start-native.sh --dsp): in STREAM the BSP stores the first 148
#           bytes of the packet as bits BEFORE the IQ_PASSTHROUGH branch
#           (calypso_bsp.c bsp_trxd_readable -> bsp_ts0_stocker). Refused in
#           that case, unless --forcer-iq. Requires numpy.
#   --perte P (all modes): one burst/block out of 1/P is erased. dsp: the
#           payload is replaced by random bits (fading: the BSP keeps its pace,
#           a dropped burst would make it wait CALYPSO_BSP_ATTENTE_MS).
#           grgsm: the block is not transmitted (= CRC failure on the L1 side).
#
# DETERMINISM: each packet draws its randomness from a seed derived from
# (--graine, fn, tn): same seed + same frames = same errors, whatever the
# arrival order (except --rafales, whose state follows the arrival order).
# Without --graine, a seed is drawn and PRINTED at start-up so a run can be
# replayed.
#
# Clean stop on SIGTERM/SIGINT (final statistics), SIGUSR1 = statistics right
# away. --verifier: checks the configuration and exits (0 = good).
import argparse
import logging
import math
import os
import random
import selectors
import signal
import socket
import struct
import sys
import time

VERSION = "2026-10-03"
log = logging.getLogger("bruit")

PROFILS = {
    # target: (listen = where the source sends, to = the layer 1)
    "dsp": ("127.0.0.1:6702", "127.0.0.1:16702"),
    "grgsm": ("127.0.0.1:14730", "127.0.0.1:4730"),
}
MODES = ("relais", "ber", "souple", "iq")
MODES_GRGSM = ("relais", "ber")

DSP_HDR = 8              # [tn, fn BE32, att, 0, 0]
NBITS = 148
IQ_MIN_OCTETS = 4 * 146  # calypso_bsp.c threshold to read the payload as I/Q
GSMTAP_TYPE_UM = 0x01
MASQUE64 = (1 << 64) - 1


def _adresse(txt, option):
    hote, sep, port = txt.rpartition(":")
    if not sep or not hote:
        raise argparse.ArgumentTypeError("%s : HOTE:PORT attendu, pas %r" % (option, txt))
    try:
        p = int(port)
    except ValueError:
        raise argparse.ArgumentTypeError("%s : port invalide %r" % (option, port))
    if not 0 < p < 65536:
        raise argparse.ArgumentTypeError("%s : port hors bornes %d" % (option, p))
    return (hote, p)


def _q_gauss(x):
    """Q(x) = P(N(0,1) > x)."""
    return 0.5 * math.erfc(x / math.sqrt(2.0))


def _geometrique(rng, p):
    """Number of trials up to the first success (>= 1), success probability p."""
    if p >= 1.0:
        return 1
    if p <= 0.0:
        return 1 << 62
    u = rng.random()
    return 1 + int(math.log1p(-u) / math.log1p(-p))


class Rafales:
    """Gilbert-Elliott: good state (no error) / bad state (rate q), geometric
    sojourns. Mean burst length L bits, mean rate p."""

    def __init__(self, p, longueur, q):
        self.q = q
        pi_m = p / q                          # fraction of the time in a burst
        self.p_mb = 1.0 / longueur            # bad -> good
        self.p_bm = self.p_mb * pi_m / (1.0 - pi_m) if pi_m < 1.0 else 1.0
        self.mauvais = False
        self.restant = None

    def positions(self, n, rng):
        if self.restant is None:
            self.restant = _geometrique(rng, self.p_bm)
        out = []
        pos = 0
        while pos < n:
            k = min(self.restant, n - pos)
            if self.mauvais:
                out.extend(i for i in range(pos, pos + k) if rng.random() < self.q)
            pos += k
            self.restant -= k
            if self.restant == 0:
                self.mauvais = not self.mauvais
                self.restant = _geometrique(rng, self.p_mb if self.mauvais else self.p_bm)
        return out


def positions_iid(n, p, rng):
    """Positions of the inverted bits, independent with probability p (geometric skip)."""
    if p <= 0.0:
        return []
    if p >= 1.0:
        return list(range(n))
    out = []
    lp = math.log1p(-p)
    i = -1
    while True:
        i += 1 + int(math.log1p(-rng.random()) / lp)
        if i >= n:
            return out
        out.append(i)


class ModulateurGmsk:
    """GMSK BT=0.3 (45.004) at `sps` samples per symbol, like calypso_gmsk.c
    gmsk_moduler(): differential coding d_i = b_i ^ b_{i-1} (b_{-1} = 1),
    alpha_i = 1 - 2 d_i, phase(t) = pi/2 * sum alpha_i q(t - i), sample j at
    instant t = j/sps + decalage. With sps=4 and decalage=0.5, the samples 4k
    (the ones the BSP keeps, decimation by 4) are those of
    gmsk_moduler(..., decalage=0.5)."""

    def __init__(self, np, sps=4, decalage=0.5):
        self.np = np
        self.sps = sps
        # The q(t) table is computed EXACTLY like calypso_gmsk.c q_init():
        # left Riemann sum, step T/16, normalised, then linear interpolation
        # (q_de). A finer integration would give another waveform (q(0) =
        # 0.500 instead of 0.477: up to 0.05 rad apart) and the iq mode would
        # no longer be "the same modulator" as the BSP.
        bt, os_, lg = 0.3, 16, 3
        k = 2.0 * math.pi * bt / math.sqrt(math.log(2.0))
        dt = 1.0 / os_
        tab, acc = [], 0.0
        for i in range(lg * os_ + 1):
            t = -1.5 + i * dt
            tab.append(acc)
            acc += (_q_gauss(k * (t - 0.5)) - _q_gauss(k * (t + 0.5))) * dt
        tab = [v / acc for v in tab]

        def q(t):
            if t <= -1.5:
                return 0.0
            if t >= 1.5:
                return 1.0
            x = (t + 1.5) * os_
            i = int(x)
            f = x - i
            if i >= lg * os_:
                return 1.0
            return tab[i] * (1.0 - f) + tab[i + 1] * f

        # K(n) = q(n/sps + decalage): 0 for t <= -1.5, 1 for t >= 1.5.
        n_lo = math.floor((-1.5 - decalage) * sps) + 1
        n_un = math.ceil((1.5 - decalage) * sps)
        self.n_lo = n_lo
        self.n_un = n_un
        self.k_trans = np.array([q(n / sps + decalage) for n in range(n_lo, n_un)])

    def phase(self, bits):
        np = self.np
        b = np.asarray(bits, dtype=np.int8) & 1
        prec = np.concatenate(([1], b[:-1]))
        alpha = 1.0 - 2.0 * (b ^ prec)
        n = len(b)
        long_ = n * self.sps
        d = np.zeros(long_)
        d[:: self.sps] = alpha
        conv = np.convolve(d, self.k_trans)                # conv[k] <-> j = k + n_lo
        j = np.arange(long_)
        part_trans = conv[j - self.n_lo]
        cumul = np.cumsum(d)
        idx = j - self.n_un
        part_marche = np.where(idx >= 0, cumul[np.clip(idx, 0, None)], 0.0)
        return (math.pi / 2.0) * (part_trans + part_marche)


class Stats:
    def __init__(self):
        self.t0 = time.monotonic()
        self.t_prec = self.t0
        self.dl_prec = 0
        self.dl = 0            # downlink packets received
        self.relayes = 0       # downlink packets sent to the L1
        self.alteres = 0       # packets whose payload was degraded
        self.effaces = 0       # --perte
        self.hors_tn = 0       # outside --tn, relayed as is
        self.autres = 0        # unrecognised format, relayed as is
        self.iq_deja = 0       # payload already in I/Q, relayed as is
        self.iq_produits = 0   # bursts converted to I/Q (iq mode)
        self.souples = 0       # packets with soft values seen
        self.bits = 0          # hard bits examined (ber, souple on hard bits)
        self.inverses = 0      # hard bits changed
        self.ul = 0            # uplink packets relayed to the source
        self.ul_sans_dest = 0  # uplink received before any downlink
        self.err_envoi = 0

    def ligne(self, final=False):
        now = time.monotonic()
        dt = max(now - self.t_prec, 1e-9)
        pps = (self.dl - self.dl_prec) / dt
        self.t_prec, self.dl_prec = now, self.dl
        ber = (self.inverses / self.bits) if self.bits else 0.0
        return ("%s t=%.0fs DL recus=%d relayes=%d alteres=%d effaces=%d hors_tn=%d autres=%d iq_deja=%d"
                " iq_produits=%d souples=%d | bits=%d inverses=%d BER=%.3e | UL relayes=%d sans_dest=%d"
                " | erreurs_envoi=%d | %.0f pq/s"
                % ("BILAN" if final else "STATS", now - self.t0, self.dl, self.relayes, self.alteres,
                   self.effaces, self.hors_tn, self.autres, self.iq_deja, self.iq_produits, self.souples,
                   self.bits, self.inverses, ber, self.ul, self.ul_sans_dest, self.err_envoi, pps))


class Degradeur:
    def __init__(self, a, np=None):
        self.a = a
        self.cible = a.cible
        self.mode = a.mode
        self.stats = Stats()
        self.graine = a.graine
        self.tn = a.tn_set
        self.rafales = Rafales(a.ber, a.rafales, a.ber_rafale) if (a.mode == "ber" and a.rafales) else None
        self.snr_lin = 10.0 ** (a.snr_db / 10.0) if a.snr_db is not None else None
        self.p_souple = _q_gauss(math.sqrt(2.0 * self.snr_lin)) if self.snr_lin is not None else 0.0
        self.np = np
        self.mod = ModulateurGmsk(np) if a.mode == "iq" else None

    def _rng(self, fn, tn, sel=0):
        x = (self.graine * 0x9E3779B97F4A7C15 + (fn + 1) * 0x100000001B3 + tn * 0x2545F491 + sel * 0x632BE5AB) & MASQUE64
        return random.Random(x)

    # -- dsp target ---------------------------------------------------------
    def _dsp(self, pkt):
        st = self.stats
        if len(pkt) < DSP_HDR + NBITS:
            st.autres += 1
            return pkt
        charge = len(pkt) - DSP_HDR
        if charge >= IQ_MIN_OCTETS:
            st.iq_deja += 1
            return pkt
        tn = pkt[0] & 0x07
        if self.tn is not None and tn not in self.tn:
            st.hors_tn += 1
            return pkt
        fn = struct.unpack_from(">I", pkt, 1)[0]
        rng = self._rng(fn, tn)
        ba = bytearray(pkt)
        dur = not bytes(ba[DSP_HDR:]).translate(None, b"\x00\x01")   # only 0/1: hard bits
        if not dur:
            st.souples += 1
        efface = self.a.perte > 0.0 and rng.random() < self.a.perte
        if efface:
            st.effaces += 1
            if dur:
                for i in range(DSP_HDR, len(ba)):
                    ba[i] = rng.getrandbits(1)
            else:
                for i in range(DSP_HDR, len(ba)):
                    ba[i] = 0                                  # soft: erasure = no information
        mode = self.mode
        if mode == "relais":
            if efface:
                st.alteres += 1
            return bytes(ba)
        if mode == "ber":
            n = len(ba) - DSP_HDR
            pos = self.rafales.positions(n, rng) if self.rafales else positions_iid(n, self.a.ber, rng)
            if dur:
                for i in pos:
                    ba[DSP_HDR + i] ^= 1
            else:
                for i in pos:                                  # soft: sign inverted
                    v = ba[DSP_HDR + i]
                    v = v - 256 if v > 127 else v
                    ba[DSP_HDR + i] = min(127, -v) & 0xFF
            st.bits += n
            st.inverses += len(pos)
            st.alteres += 1
            return bytes(ba)
        if mode == "souple":
            snr = self.snr_lin
            if dur:
                # +-1 + N(0, sigma^2), sigma^2 = 1/(2 snr), then hard decision:
                # each bit changes independently with probability
                # Q(1/sigma) = Q(sqrt(2 snr)). Those positions are therefore
                # drawn directly -- exactly the same law, without 148 gaussians.
                n = len(ba) - DSP_HDR
                pos = positions_iid(n, self.p_souple, rng)
                for i in pos:
                    ba[DSP_HDR + i] ^= 1
                st.bits += n
                st.inverses += len(pos)
            else:
                vals = [(v - 256 if v > 127 else v) for v in ba[DSP_HDR:]]
                amp = (sum(abs(v) for v in vals) / len(vals)) or 127.0
                sigma = amp / math.sqrt(2.0 * snr)
                for k, v in enumerate(vals):
                    w = int(round(v + rng.gauss(0.0, sigma)))
                    ba[DSP_HDR + k] = max(-127, min(127, w)) & 0xFF
            st.alteres += 1
            return bytes(ba)
        # iq mode
        if dur:
            bits = list(ba[DSP_HDR:DSP_HDR + NBITS])
        else:                                                  # soft int8: hard decision (> 0 = 1)
            bits = [1 if 0 < v <= 127 else 0 for v in ba[DSP_HDR:DSP_HDR + NBITS]]
        return bytes(ba[:DSP_HDR]) + self._iq(bits, fn, tn)

    def _iq(self, bits, fn, tn):
        np = self.np
        a = self.a
        ph = self.mod.phase(bits)
        i_ = a.amp * np.cos(ph)
        q_ = a.amp * np.sin(ph)
        if a.iq_elargir and any(bits):
            # Imitates the BSP's gmsk_elargir() on the samples it keeps
            # (neighbours at +-1 symbol = +-sps samples); FCCH (all zero)
            # left as is, like the BSP.
            s, e = self.mod.sps, a.iq_elargir
            for v in (i_, q_):
                w = v.copy()
                w[s:] += e * v[:-s]
                w[:-s] += e * v[s:]
                v[:] = w / (1.0 + 2.0 * e)
        if self.snr_lin is not None:
            sigma = math.sqrt(a.amp * a.amp / (2.0 * self.snr_lin))
            g = np.random.Generator(np.random.PCG64(self._rng(fn, tn, 1).getrandbits(64)))
            i_ = i_ + g.normal(0.0, sigma, i_.shape)
            q_ = q_ + g.normal(0.0, sigma, q_.shape)
        out = np.empty(2 * len(i_), dtype="<i2")
        out[0::2] = np.clip(np.rint(i_), -32768, 32767)
        out[1::2] = np.clip(np.rint(q_), -32768, 32767)
        self.stats.iq_produits += 1
        self.stats.alteres += 1
        return out.tobytes()

    # -- grgsm target -------------------------------------------------------
    def _grgsm(self, pkt):
        st = self.stats
        if len(pkt) < 16 or pkt[0] != 2 or pkt[2] != GSMTAP_TYPE_UM:
            st.autres += 1
            return pkt
        hdr = pkt[1] * 4
        if hdr < 16 or hdr >= len(pkt):
            st.autres += 1
            return pkt
        tn = pkt[3] & 0x07
        if self.tn is not None and tn not in self.tn:
            st.hors_tn += 1
            return pkt
        fn = struct.unpack_from(">I", pkt, 8)[0]
        rng = self._rng(fn, tn, pkt[12])
        if self.a.perte > 0.0 and rng.random() < self.a.perte:
            st.effaces += 1
            return None                                        # lost block = CRC failure on the L1 side
        if self.mode == "relais":
            return pkt
        ba = bytearray(pkt)
        n = (len(ba) - hdr) * 8
        pos = self.rafales.positions(n, rng) if self.rafales else positions_iid(n, self.a.ber, rng)
        for i in pos:
            ba[hdr + (i >> 3)] ^= 0x80 >> (i & 7)
        st.bits += n
        st.inverses += len(pos)
        st.alteres += 1
        return bytes(ba)

    def descendant(self, pkt):
        self.stats.dl += 1
        return self._dsp(pkt) if self.cible == "dsp" else self._grgsm(pkt)


def _parser():
    p = argparse.ArgumentParser(
        prog="injecteur_bruit.py",
        description="Proxy UDP transparent pont -> couche 1 qui degrade le sens descendant "
                    "(bits inverses, AWGN, I/Q bruite). Le montant est relaye sans modification.")
    p.add_argument("--cible", choices=sorted(PROFILS), default="dsp",
                   help="format du flux : dsp (TRXD 8 o + 148 bits, BSP de c54x_exe) ou grgsm "
                        "(GSMTAP vers la L1 gr-gsm de QEMU) ; defaut dsp")
    p.add_argument("--mode", choices=MODES, default="ber", help="defaut ber")
    p.add_argument("--ecoute", help="HOTE:PORT ou la source envoie (dsp 127.0.0.1:6702, grgsm 127.0.0.1:14730)")
    p.add_argument("--vers", help="HOTE:PORT de la couche 1 (dsp 127.0.0.1:16702, grgsm 127.0.0.1:4730)")
    p.add_argument("--source", help="HOTE:PORT local d'emission vers la L1 (defaut : port libre)")
    p.add_argument("--retour", help="HOTE:PORT ou renvoyer le montant (defaut : l'emetteur du dernier descendant)")
    p.add_argument("--ber", type=float, default=0.0, help="mode ber : taux moyen d'inversion (0..1)")
    p.add_argument("--rafales", type=float, default=0.0, metavar="L",
                   help="mode ber : erreurs en rafales de L bits en moyenne (Gilbert-Elliott)")
    p.add_argument("--ber-rafale", type=float, default=0.5, help="taux d'erreur dans une rafale (defaut 0,5)")
    p.add_argument("--snr-db", type=float, default=None,
                   help="modes souple et iq : SNR en dB, P_signal / (2 sigma^2) par echantillon complexe "
                        "(meme definition que CALYPSO_BSP_SNR_DB)")
    p.add_argument("--perte", type=float, default=0.0,
                   help="probabilite d'effacer un burst (dsp : bits aleatoires) ou un bloc (grgsm : non transmis)")
    p.add_argument("--tn", default="tous", help="intervalles a degrader, ex. 0 ou 1,2 (defaut tous)")
    p.add_argument("--graine", type=lambda s: int(s, 0), default=None, help="graine (deterministe)")
    p.add_argument("--amp", type=int, default=30000, help="mode iq : amplitude (defaut 30000, comme le BSP)")
    p.add_argument("--iq-elargir", type=float, default=0.3,
                   help="mode iq : elargissement des NB comme gmsk_elargir (defaut 0,3 ; 0 = aucun)")
    p.add_argument("--forcer-iq", action="store_true",
                   help="mode iq malgre CALYPSO_BSP_STREAM=1 ou CALYPSO_BSP_IQ_PASSTHROUGH=0")
    p.add_argument("--stats", type=float, default=10.0, help="periode des statistiques en s (0 = seulement le bilan)")
    p.add_argument("--journal", help="fichier journal (en plus de stderr)")
    p.add_argument("--pidfile", help="ecrit le pid, efface a l'arret")
    p.add_argument("--etiquette", default="", help="libre, pour reconnaitre le processus (pkill -f)")
    p.add_argument("--verifier", action="store_true", help="controle la configuration et sort")
    p.add_argument("--version", action="version", version="%(prog)s " + VERSION)
    return p


def _valider(a, p):
    """Completes and checks the arguments; returns numpy (iq mode) or None."""
    ecoute, vers = PROFILS[a.cible]
    a.ecoute = _adresse(a.ecoute or ecoute, "--ecoute")
    a.vers = _adresse(a.vers or vers, "--vers")
    a.source = _adresse(a.source, "--source") if a.source else None
    a.retour = _adresse(a.retour, "--retour") if a.retour else None
    if a.ecoute == a.vers:
        p.error("--ecoute et --vers designent la meme adresse %s:%d : l'injecteur s'enverrait ses propres paquets" % a.ecoute)
    if a.cible == "grgsm" and a.mode not in MODES_GRGSM:
        p.error("cible grgsm : mode %s impossible. Le flux GSMTAP vers QEMU porte des blocs L2 deja decodes "
                "(16 o d'en-tete + 23 o), ni valeurs souples ni bursts a moduler (calypso_l1_grgsm.c "
                "gsmtap_readable). Modes possibles : %s (et --perte)." % (a.mode, ", ".join(MODES_GRGSM)))
    for nom in ("ber", "perte", "ber_rafale"):
        v = getattr(a, nom)
        if not 0.0 <= v <= 1.0:
            p.error("--%s doit etre entre 0 et 1 (%g)" % (nom.replace("_", "-"), v))
    if a.mode == "ber" and a.rafales:
        if a.rafales < 1.0:
            p.error("--rafales : longueur moyenne en bits >= 1 (%g)" % a.rafales)
        if not 0.0 < a.ber_rafale <= 1.0 or a.ber >= a.ber_rafale:
            p.error("--rafales : il faut --ber (%g) < --ber-rafale (%g) <= 1" % (a.ber, a.ber_rafale))
    if a.mode == "souple" and a.snr_db is None:
        p.error("mode souple : --snr-db obligatoire")
    if a.snr_db is not None and not -50.0 <= a.snr_db <= 100.0:
        p.error("--snr-db hors bornes raisonnables (%g)" % a.snr_db)
    if a.tn in ("", "tous", "all"):
        a.tn_set = None
    else:
        try:
            a.tn_set = {int(x) for x in a.tn.split(",") if x.strip() != ""}
        except ValueError:
            p.error("--tn : liste d'intervalles 0..7 attendue (%r)" % a.tn)
        if not a.tn_set or any(not 0 <= x <= 7 for x in a.tn_set):
            p.error("--tn : intervalles 0..7 (%r)" % a.tn)
    if a.graine is None:
        a.graine = int.from_bytes(os.urandom(4), "little")
        a.graine_tiree = True
    else:
        a.graine_tiree = False
    a.graine &= MASQUE64
    np = None
    if a.mode == "iq":
        if a.cible != "dsp":
            p.error("mode iq : cible dsp seulement")
        try:
            import numpy as np  # noqa: F811 - only the iq mode needs it
        except ImportError:
            p.error("mode iq : numpy est necessaire et introuvable pour %s "
                    "(apt-get install python3-numpy, ou pip install numpy). "
                    "Les modes relais, ber et souple n'en ont pas besoin." % sys.executable)
        if not a.forcer_iq:
            if os.environ.get("CALYPSO_BSP_STREAM", "") == "1":
                p.error("mode iq refuse : CALYPSO_BSP_STREAM=1. En STREAM le BSP range les 148 premiers octets "
                        "de chaque paquet comme des BITS avant de regarder s'il s'agit d'I/Q "
                        "(calypso_bsp.c bsp_trxd_readable -> bsp_ts0_stocker) : l'I/Q y serait lu comme du bruit "
                        "de bits. Lancez c54x_exe sans STREAM, ou --forcer-iq (BRUIT_OPTS=--forcer-iq).")
            if os.environ.get("CALYPSO_BSP_IQ_PASSTHROUGH", "") == "0":
                p.error("mode iq refuse : CALYPSO_BSP_IQ_PASSTHROUGH=0, le BSP ne lit pas l'I/Q.")
        if not 1000 <= a.amp <= 32767:
            p.error("--amp : 1000..32767 (%d)" % a.amp)
        if not 0.0 <= a.iq_elargir <= 2.0:
            p.error("--iq-elargir : 0..2 (%g)" % a.iq_elargir)
    return np


def _ouvrir(a):
    amont = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        amont.bind(a.ecoute)
    except OSError as e:
        amont.close()
        raise SystemExit("injecteur_bruit : impossible d'ecouter sur %s:%d (%s) -- un autre processus "
                         "tient-il deja ce port ? ss -lunp | grep :%d" % (a.ecoute[0], a.ecoute[1], e.strerror, a.ecoute[1]))
    aval = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    if a.source:
        loc = a.source
    else:
        loc = ("127.0.0.1", 0) if a.vers[0].startswith("127.") or a.vers[0] == "localhost" else ("0.0.0.0", 0)
    try:
        aval.bind(loc)
    except OSError as e:
        amont.close()
        aval.close()
        raise SystemExit("injecteur_bruit : impossible de lier le port d'emission %s:%d (%s)" % (loc[0], loc[1], e.strerror))
    for s in (amont, aval):
        s.setblocking(False)
        try:
            s.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 1 << 20)
        except OSError:
            pass
    return amont, aval


def _journal(a):
    fmt = logging.Formatter("%(asctime)s [bruit] %(message)s", "%H:%M:%S")
    log.setLevel(logging.INFO)
    h = logging.StreamHandler(sys.stderr)
    h.setFormatter(fmt)
    log.addHandler(h)
    if a.journal:
        f = logging.FileHandler(a.journal)
        f.setFormatter(fmt)
        log.addHandler(f)


def _annoncer(a, aval, np=None):
    loc = aval.getsockname()
    log.info("injecteur de bruit %s : cible %s, mode %s%s", VERSION, a.cible, a.mode,
             " (EXPERIMENTAL)" if a.mode == "iq" else "")
    if a.mode == "ber":
        if a.rafales:
            log.info("BER moyen %.3e en rafales de %.1f bits (taux %.2f dans une rafale, Gilbert-Elliott)",
                     a.ber, a.rafales, a.ber_rafale)
        else:
            log.info("BER %.3e (bits independants)", a.ber)
        if a.cible == "grgsm" and a.ber > 0:
            log.info("ATTENTION grgsm : bits inverses dans le bloc L2 DEJA DECODE, rien ne les detecte "
                     "(ni FEC ni CRC) ; --perte modelise l'echec CRC")
    if a.mode == "souple":
        snr = 10.0 ** (a.snr_db / 10.0)
        log.info("SNR %.2f dB ; sur bits durs (le cas du pont) : +-1 + AWGN + decision dure, "
                 "BER attendu Q(sqrt(2 snr)) = %.3e", a.snr_db, _q_gauss(math.sqrt(2.0 * snr)))
    if a.mode == "iq":
        log.info("I/Q GMSK 4 ech/symbole, amplitude %d, elargissement NB %.2f, %s ; "
                 "contourne sb_moduler/gmsk_elargir du BSP", a.amp, a.iq_elargir,
                 ("AWGN %.2f dB" % a.snr_db) if a.snr_db is not None else "SANS AWGN")
        if a.snr_db is not None and np is not None:
            # int16 clipping: at 30000 only 2767 of headroom remain. Estimated
            # here to announce it (the C side, CALYPSO_BSP_SNR_DB, clips the same way).
            g = np.random.default_rng(0)
            m = 20000
            s = a.amp * np.exp(1j * g.uniform(0.0, 2.0 * math.pi, m))
            sig = math.sqrt(a.amp * a.amp / (2.0 * 10.0 ** (a.snr_db / 10.0)))
            y = s + sig * (g.standard_normal(m) + 1j * g.standard_normal(m))
            ecr = float(np.mean((np.abs(y.real) > 32767) | (np.abs(y.imag) > 32767)))
            if ecr > 0.01:
                yc = np.clip(y.real, -32768, 32767) + 1j * np.clip(y.imag, -32768, 32767)
                eff = 10.0 * math.log10(a.amp * a.amp / float(np.mean(np.abs(yc - s) ** 2)))
                log.info("ATTENTION ecretage int16 : ~%.0f %% des echantillons, SNR effectif ~%.1f dB "
                         "(bruit non gaussien) ; --amp plus bas pour l'eviter", 100.0 * ecr, eff)
    if a.perte:
        log.info("effacement : %.3e des %s", a.perte, "bursts (bits aleatoires)" if a.cible == "dsp" else "blocs (non transmis)")
    if (a.mode == "relais" or (a.mode == "ber" and a.ber == 0.0)) and not a.perte:
        log.info("aucune degradation configuree : relais transparent")
    log.info("intervalles : %s ; graine %d%s", "tous" if a.tn_set is None else ",".join(map(str, sorted(a.tn_set))),
             a.graine, " (tiree : --graine %d pour rejouer)" % a.graine if a.graine_tiree else "")
    log.info("PRET ecoute=%s:%d vers=%s:%d emission=%s:%d retour=%s", a.ecoute[0], a.ecoute[1], a.vers[0], a.vers[1],
             loc[0], loc[1], ("%s:%d" % a.retour) if a.retour else "emetteur du descendant")


def main(argv=None):
    p = _parser()
    a = p.parse_args(argv)
    np = _valider(a, p)
    if a.verifier:
        amont, aval = _ouvrir(a)
        amont.close()
        aval.close()
        print("injecteur_bruit : configuration valide (cible %s, mode %s, ecoute %s:%d -> %s:%d)"
              % (a.cible, a.mode, a.ecoute[0], a.ecoute[1], a.vers[0], a.vers[1]))
        return 0
    _journal(a)
    amont, aval = _ouvrir(a)
    deg = Degradeur(a, np)
    st = deg.stats
    if a.pidfile:
        with open(a.pidfile, "w") as f:
            f.write("%d\n" % os.getpid())

    arret = []
    flash = []
    signal.signal(signal.SIGTERM, lambda s, f: arret.append(s))
    signal.signal(signal.SIGINT, lambda s, f: arret.append(s))
    signal.signal(signal.SIGUSR1, lambda s, f: flash.append(s))

    _annoncer(a, aval, np)
    sel = selectors.DefaultSelector()
    sel.register(amont, selectors.EVENT_READ, "dl")
    sel.register(aval, selectors.EVENT_READ, "ul")
    source = None
    prochain = time.monotonic() + a.stats if a.stats > 0 else None
    try:
        while not arret:
            try:
                evts = sel.select(timeout=0.5)
            except InterruptedError:
                evts = []
            for cle, _ in evts:
                s = cle.fileobj
                for _ in range(512):
                    try:
                        pkt, adr = s.recvfrom(65536)
                    except (BlockingIOError, InterruptedError):
                        break
                    except OSError:
                        st.err_envoi += 1            # ICMP reported on the socket: carry on
                        break
                    if cle.data == "dl":
                        source = adr
                        sortie = deg.descendant(pkt)
                        if sortie is None:
                            continue
                        try:
                            aval.sendto(sortie, a.vers)
                            st.relayes += 1
                        except OSError:
                            st.err_envoi += 1
                    else:
                        dest = a.retour or source
                        if dest is None:
                            st.ul_sans_dest += 1
                            continue
                        try:
                            amont.sendto(pkt, dest)  # uplink: as is
                            st.ul += 1
                        except OSError:
                            st.err_envoi += 1
            now = time.monotonic()
            if flash or (prochain is not None and now >= prochain):
                del flash[:]
                log.info("%s", st.ligne())
                if prochain is not None:
                    prochain = now + a.stats
    finally:
        log.info("%s", st.ligne(final=True))
        log.info("arret (%s)", "signal %d" % arret[0] if arret else "fin")
        sel.close()
        amont.close()
        aval.close()
        if a.pidfile:
            try:
                os.unlink(a.pidfile)
            except OSError:
                pass
    return 0


if __name__ == "__main__":
    sys.exit(main())
