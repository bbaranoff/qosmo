#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# injecteur_bruit.py - proxy UDP qui degrade le sens DESCENDANT entre le pont
# (osmo-operator/pont) et la couche 1 du telephone emule.
#
# [2026-10-03] Cree pour le banc Calypso. Un seul fichier source :
#   /opt/GSM/qosmo/tools/injecteur_bruit.py
# (c54x_exe/run.sh le prend la, avec repli sur c54x_exe/tools/injecteur_bruit.py,
# un lien vers ce fichier ; qosmo/run_modules/38-bruit.sh le lance pour le plan
# de qosmo).
#
# OU IL S'INTERCALE
#   cible dsp   : pont_dsp.py --dsp-port 6702 -> [injecteur 127.0.0.1:6702]
#                 -> BSP de c54x_exe deplace sur CALYPSO_BSP_PORT (16702 par
#                 defaut). Format : 8 octets d'en-tete [tn, fn BE32, att, 0, 0]
#                 puis 148 octets de bits DURS 0/1 (pont/trx.py Trx.run_data ;
#                 calypso_bsp.c bsp_trxd_readable).
#                 Le BSP apprend l'adresse de son pair montant sur l'emetteur du
#                 descendant : une fois l'injecteur intercale, ce qu'il renvoie
#                 (bursts UL TRXDv0, s'il en emet) arrive ici et repart SANS
#                 MODIFICATION vers la source (le pont, 127.0.0.1:5702), depuis
#                 le port d'ecoute -- exactement l'adresse qu'il avait avant.
#   cible grgsm : pont.py -> GSMTAP [injecteur 127.0.0.1:14730] -> L1 gr-gsm de
#                 QEMU (127.0.0.1:4730, port fige dans calypso_l1_grgsm.c).
#                 Le pont y envoie si PONT_GSMTAP_PORT=14730 (pont/config.py ;
#                 pont/pont.py le pose seul quand BRUIT_MODE vaut ber|relais).
#                 Format : en-tete GSMTAP v2 de 16 octets puis le bloc L2 DEJA
#                 DECODE (23 octets) -- PAS des bursts, PAS de valeurs souples.
#                 Le SCH (4731, « SCH2 ») et la parole TCH (/dev/shm/calypso_tch_dl)
#                 ne passent pas par ici et ne sont pas touches.
#
# MODES (sens descendant seulement ; le montant est toujours relaye tel quel)
#   relais  aucune alteration : sert a verifier que l'intercalation seule ne
#           change rien.
#   ber     inversion aleatoire de bits, taux moyen --ber. --rafales L : erreurs
#           groupees (Gilbert-Elliott, rafales de L bits en moyenne, taux
#           --ber-rafale dans une rafale), meme taux moyen.
#           dsp : bits durs inverses (0<->1). grgsm : bits des octets L2 inverses
#           APRES decodage -- non physique (ni code correcteur ni CRC ne
#           protegent plus le bloc) ; --perte est le modele realiste en grgsm.
#   souple  AWGN a --snr-db sur les valeurs souples. VERIFIE LE 2026-10-03 :
#           aucun des deux flux n'en porte (dsp : bits durs 0/1 ; grgsm : octets
#           L2). Sur des bits durs : modulation antipodale +-1, AWGN, decision
#           dure -- BER = Q(sqrt(2 snr)), annonce au demarrage ; la sortie reste
#           0/1, le seul format que le BSP sait lire. Si une source envoie un jour
#           des valeurs souples int8 (convention TRXD : +127 = bit 1), elles sont
#           bruitees et gardees souples. Refuse en grgsm.
#   iq      EXPERIMENTAL. Bits -> GMSK BT=0.3 a 4 echantillons/symbole (meme
#           modulateur que calypso_gmsk.c, instant 0,5) + AWGN a --snr-db, envoye
#           en I/Q int16 entrelaces (IQ_PASSTHROUGH du BSP, decimation par 4).
#           CONTOURNE la modulation reglee du BSP (sb_moduler pour le SCH :
#           instant 0,35, elargissement 1,0, bruit propre ; gmsk_elargir 0,3 sur
#           les NB -- seul ce dernier est imite ici, --iq-elargir).
#           INCOMPATIBLE avec CALYPSO_BSP_STREAM=1 (le defaut de
#           start-direct.sh --dsp) : en STREAM le BSP range les 148 premiers
#           octets du paquet comme des bits AVANT la branche IQ_PASSTHROUGH
#           (calypso_bsp.c bsp_trxd_readable -> bsp_ts0_stocker). Refuse dans ce
#           cas, sauf --forcer-iq. Necessite numpy.
#   --perte P (tous modes) : un burst/bloc sur 1/P est efface. dsp : la charge
#           est remplacee par des bits aleatoires (evanouissement : le BSP garde
#           sa cadence, un burst jete le ferait attendre CALYPSO_BSP_ATTENTE_MS).
#           grgsm : le bloc n'est pas transmis (= echec CRC cote L1).
#
# DETERMINISME : chaque paquet tire son alea d'une graine derivee de
# (--graine, fn, tn) : meme graine + memes trames = memes erreurs, quel que soit
# l'ordre d'arrivee (sauf --rafales, dont l'etat suit l'ordre d'arrivee). Sans
# --graine, une graine est tiree et ECRITE au demarrage pour pouvoir rejouer.
#
# Arret propre sur SIGTERM/SIGINT (statistiques finales), SIGUSR1 = statistiques
# tout de suite. --verifier : controle la configuration et sort (0 = bon).
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
    # cible : (ecoute = la ou la source envoie, vers = la couche 1)
    "dsp": ("127.0.0.1:6702", "127.0.0.1:16702"),
    "grgsm": ("127.0.0.1:14730", "127.0.0.1:4730"),
}
MODES = ("relais", "ber", "souple", "iq")
MODES_GRGSM = ("relais", "ber")

DSP_HDR = 8              # [tn, fn BE32, att, 0, 0]
NBITS = 148
IQ_MIN_OCTETS = 4 * 146  # seuil de calypso_bsp.c pour lire la charge en I/Q
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
    """Nombre d'essais jusqu'au premier succes (>= 1), succes de probabilite p."""
    if p >= 1.0:
        return 1
    if p <= 0.0:
        return 1 << 62
    u = rng.random()
    return 1 + int(math.log1p(-u) / math.log1p(-p))


class Rafales:
    """Gilbert-Elliott : etat bon (aucune erreur) / mauvais (taux q), sejours
    geometriques. Longueur moyenne d'une rafale L bits, taux moyen p."""

    def __init__(self, p, longueur, q):
        self.q = q
        pi_m = p / q                          # fraction du temps en rafale
        self.p_mb = 1.0 / longueur            # mauvais -> bon
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
    """Positions des bits inverses, independants de probabilite p (saut geometrique)."""
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
    """GMSK BT=0.3 (45.004) a `sps` echantillons par symbole, comme
    calypso_gmsk.c gmsk_moduler() : codage differentiel d_i = b_i ^ b_{i-1}
    (b_{-1} = 1), alpha_i = 1 - 2 d_i, phase(t) = pi/2 * sum alpha_i q(t - i),
    echantillon j a l'instant t = j/sps + decalage. Avec sps=4 et decalage=0,5,
    les echantillons 4k (ceux que garde le BSP, decimation par 4) sont ceux de
    gmsk_moduler(..., decalage=0.5)."""

    def __init__(self, np, sps=4, decalage=0.5):
        self.np = np
        self.sps = sps
        # La table q(t) est calculee EXACTEMENT comme calypso_gmsk.c q_init() :
        # somme de Riemann a gauche, pas T/16, normalisee, puis interpolation
        # lineaire (q_de). Une integration plus fine donnerait une autre forme
        # d'onde (q(0) = 0,500 au lieu de 0,477 : jusqu'a 0,05 rad d'ecart) et
        # le mode iq ne serait plus « le meme modulateur » que le BSP.
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

        # K(n) = q(n/sps + decalage) : 0 pour t <= -1,5, 1 pour t >= 1,5.
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
        self.dl = 0            # paquets descendants recus
        self.relayes = 0       # paquets descendants envoyes vers la L1
        self.alteres = 0       # paquets dont la charge a ete degradee
        self.effaces = 0       # --perte
        self.hors_tn = 0       # hors --tn, relayes tels quels
        self.autres = 0        # format non reconnu, relayes tels quels
        self.iq_deja = 0       # charge deja en I/Q, relayee telle quelle
        self.iq_produits = 0   # bursts convertis en I/Q (mode iq)
        self.souples = 0       # paquets a valeurs souples vus
        self.bits = 0          # bits durs examines (ber, souple sur bits durs)
        self.inverses = 0      # bits durs changes
        self.ul = 0            # paquets montants relayes vers la source
        self.ul_sans_dest = 0  # montant recu avant tout descendant
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

    # -- cible dsp ----------------------------------------------------------
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
        dur = not bytes(ba[DSP_HDR:]).translate(None, b"\x00\x01")   # que des 0/1 : bits durs
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
                    ba[i] = 0                                  # souple : effacement = aucune information
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
                for i in pos:                                  # souple : signe inverse
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
                # +-1 + N(0, sigma^2), sigma^2 = 1/(2 snr), puis decision dure :
                # chaque bit change independamment avec la probabilite
                # Q(1/sigma) = Q(sqrt(2 snr)). On tire donc directement ces
                # positions -- meme loi, exactement, sans 148 gaussiennes.
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
        # mode iq
        if dur:
            bits = list(ba[DSP_HDR:DSP_HDR + NBITS])
        else:                                                  # souple int8 : decision dure (> 0 = 1)
            bits = [1 if 0 < v <= 127 else 0 for v in ba[DSP_HDR:DSP_HDR + NBITS]]
        return bytes(ba[:DSP_HDR]) + self._iq(bits, fn, tn)

    def _iq(self, bits, fn, tn):
        np = self.np
        a = self.a
        ph = self.mod.phase(bits)
        i_ = a.amp * np.cos(ph)
        q_ = a.amp * np.sin(ph)
        if a.iq_elargir and any(bits):
            # Imite gmsk_elargir() du BSP sur les echantillons qu'il garde
            # (voisins a +-1 symbole = +-sps echantillons) ; FCCH (tout a zero)
            # laisse tel quel, comme le BSP.
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

    # -- cible grgsm --------------------------------------------------------
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
            return None                                        # bloc perdu = echec CRC cote L1
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
    """Complete et controle les arguments ; rend numpy (mode iq) ou None."""
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
            import numpy as np  # noqa: F811 - seul le mode iq en a besoin
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
            # Ecretage int16 : a 30000 il ne reste que 2767 de marge. On l'estime
            # pour l'annoncer (le C, CALYPSO_BSP_SNR_DB, ecrete de la meme facon).
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
                        st.err_envoi += 1            # ICMP remonte sur la socket : on continue
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
                            amont.sendto(pkt, dest)  # montant : tel quel
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
