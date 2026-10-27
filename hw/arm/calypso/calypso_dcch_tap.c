/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * calypso_dcch_tap.c - which dedicated channel the mobile uses, for the bridge.
 *
 * pont.py does not read the IMMEDIATE ASSIGNMENT itself: its Dedicated class
 * (pont/state.py) waits to be told the channel through
 * /dev/shm/calypso_dcch_cfg, and as long as that file does not exist,
 * uplink.py:_poll_sdcch() drops every uplink block, including the SABM that
 * carries the LOCATION UPDATING REQUEST.
 *
 * In the gr-gsm setup, l1-grgsm/calypso_l1ctl_tap.c writes it, hooked through
 * the layer 1 vtable (uart_tx_byte). Under CALYPSO_DSP_EXTERN=1 that layer 1
 * is disabled (calypso_l1_disable), so the file must be written from here:
 * the SABM block is captured by c54x_exe (src/montant.c) and published in
 * /dev/shm/calypso_sdcch_ul, and the bridge only emits it once it knows the
 * channel.
 *
 * This tap depends on no layer 1: it sniffs the sercomm stream the firmware
 * sends to osmocon and only keeps the channel number of the
 * L1CTL_DATA_CONF / DATA_IND. It publishes only the side-band; the reactions
 * specific to the gr-gsm layer 1 (a_cd window, DCCH guard) stay there.
 */
#include "qemu/osdep.h"
#include "qemu/bswap.h"
#include "hw/arm/calypso/calypso_dcch_tap.h"
#include "hw/arm/calypso/calypso_trx.h"
#include <fcntl.h>
#include <unistd.h>

#define SERCOMM_FLAG        0x7E
#define SERCOMM_ESCAPE      0x7D
#define SERCOMM_ESCAPE_XOR  0x20
#define SERCOMM_DLCI_L1CTL  5

#define L1CTL_DATA_IND      0x03
#define L1CTL_RACH_CONF     0x0c
#define L1CTL_DATA_CONF     0x0f
/* PDCH: the GPRS firmware (layer1 l1gprs_fw.c) does not report DATA_IND on a
 * PDTCH but L1CTL_GPRS_DL_BLOCK_IND (hdr fn BE32 + tn). */
#define L1CTL_GPRS_DL_BLOCK_IND 0x23
#define L1CTL_GPRS_UL_BLOCK_CNF 0x26

/* Downlink PDTCH counter, to separate the ROM losses from the other causes of
 * T3190/T3168 (banc-max "data"). The firmware (l1gprs_fw.c l1gprs_fw_dl_block)
 * reports ALL the PDCH blocks: payload = l1ctl_hdr(4) fn(4) tn(1) pad(3)
 * ber10k(2) ci(2) rx_lev(1) usf(1) [+ 23 bytes if CRC good AND our TFI or a
 * control block]. usf != 0xff only if the CRC is good. Hence:
 *   donnees   = 41 bytes (PTCCH excluded)   autre_tfi = 18 bytes, usf != 0xff
 *   crc_ko    = 18 bytes, usf == 0xff, fn%104 != 12 (block lost by the ROM)
 *   ptcch     = fn%104 == 12                ul_cnf    = L1CTL_GPRS_UL_BLOCK_CNF
 * A "[pdtch]" line on stderr (qemu.log) every 100 blocks and whenever a PDCH is
 * armed. */
static struct {
    unsigned donnees, autre_tfi, crc_ko, ptcch, ul_cnf;
    unsigned long ber_ko;
} pdtch;

static void pdtch_journal(const char *pourquoi)
{
    fprintf(stderr, "[pdtch] %s : dl donnees=%u autre_tfi=%u crc_ko=%u ptcch=%u | ul_cnf=%u | ber moyen crc_ko=%.1f%%\n",
            pourquoi, pdtch.donnees, pdtch.autre_tfi, pdtch.crc_ko, pdtch.ptcch, pdtch.ul_cnf,
            pdtch.crc_ko ? (double)pdtch.ber_ko / pdtch.crc_ko / 100.0 : 0.0);
}

#define SHM_DCCH_CFG        "/dev/shm/calypso_dcch_cfg"
#define DCCH_RELEASED       0xFF

/* How many frames must separate the last block of the dedicated channel from a
 * block on a common channel before concluding that the channel is released. A
 * late CCCH block, delivered just after the switch, carries a frame number
 * close to those of the dedicated channel: it must not release anything. */
#define DCCH_LIBERE_APRES_TRAMES  100

static struct {
    enum { SC_IDLE, SC_IN_FRAME, SC_ESCAPE } state;
    uint8_t buf[512];
    int len;
    uint8_t dernier_chan_nr;
    uint32_t seq;
    uint32_t fn_dedie;       /* last frame seen on the dedicated channel */
    bool     fn_dedie_vue;
    uint8_t  trx_chan_nr;    /* last SDCCH (or 0xFF) told to calypso_trx */
} tap = { .dernier_chan_nr = 0xFF, .trx_chan_nr = 0xFF };

/* TCH kinds. The readers (montant.c scruter_dcch, pont/dsp/dedicated.py)
 * understand them in addition to 0/1/0xFF; the grgsm pont.py never reads this
 * tap (its layer 1 writes the file itself). */
#define DCCH_TCH_F          2
#define DCCH_TCH_H          3
/* kind 4 = PDCH (chan_nr 0b11000TTT, Osmocom extension): the BSP plays ALL the
 * frames of the timeslot, like a TCH (calypso_bsp.c). */
#define DCCH_PDCH           4

/* 16 bytes, as expected by pont/state.py:Dedicated.read():
 * seq(4) kind(1) ss(1) tn(1) chan_nr(1). kind 0 = SDCCH/4, 1 = SDCCH/8,
 * 2 = TCH/F, 3 = TCH/H (ss = half-rate subslot), 0xFF = channel released. */
static void dcch_cfg_publier(int kind, int ss, uint8_t chan_nr)
{
    uint8_t b[16] = {0};
    tap.seq++;
    memcpy(b, &tap.seq, 4);
    b[4] = (uint8_t)kind;
    b[5] = (uint8_t)ss;
    b[6] = chan_nr & 0x07;
    b[7] = chan_nr;
    int fd = open(SHM_DCCH_CFG, O_WRONLY | O_CREAT | O_TRUNC, 0666);
    if (fd < 0) {
        return;
    }
    if (write(fd, b, sizeof(b)) < 0) {
        close(fd);
        return;
    }
    close(fd);
    /* calypso_trx (PONT_DCCH -> pont.c -> calypso_bsp_set_dedie) is only told
     * about a new SDCCH or the release. The TCH is not passed on: switching
     * the BSP to the TCH is the job of montant.c (suivre_tache_tch), which a
     * PONT_DCCH of kind 2 would bypass. The return to the SAME SDCCH after
     * the TCH (ASSIGNMENT FAILURE) is not passed on either. */
    if (kind != DCCH_TCH_F && kind != DCCH_TCH_H && chan_nr != tap.trx_chan_nr) {
        tap.trx_chan_nr = kind == DCCH_RELEASED ? 0xFF : chan_nr;
        calypso_trx_dcch(kind, ss, chan_nr & 0x07);
    }
    if (kind == DCCH_PDCH) {
        fprintf(stderr, "[dcch] canal dedie arme : chan_nr=0x%02x PDCH TN=%d (GPRS)\n", chan_nr, chan_nr & 7);
        return;
    }
    if (kind == DCCH_TCH_F || kind == DCCH_TCH_H) {
        fprintf(stderr, "[dcch] canal dedie arme : chan_nr=0x%02x TCH/%c SS=%d TN=%d\n",
                chan_nr, kind == DCCH_TCH_F ? 'F' : 'H', ss, chan_nr & 7);
        return;
    }
    fprintf(stderr, "[dcch] canal dedie %s : chan_nr=0x%02x SDCCH/%d SS=%d TN=%d\n",
            kind == DCCH_RELEASED ? "libere" : "arme", chan_nr,
            kind == 1 ? 8 : 4, ss, chan_nr & 7);
}

static void trame_complete(void)
{
    if (tap.len < 3 || tap.buf[0] != SERCOMM_DLCI_L1CTL) {
        return;
    }
    const uint8_t *charge = &tap.buf[2];
    int plen = tap.len - 2;
    uint8_t type = charge[0];
    if (type == L1CTL_RACH_CONF && plen >= 12) {
        /* The RACH transmit frame, as reported by the firmware. */
        calypso_trx_rach_conf(ldl_be_p(charge + 8));
        return;
    }
    if (type == L1CTL_GPRS_DL_BLOCK_IND && plen >= 9) {
        /* Downlink PDTCH block: l1ctl_hdr(4) fn(4, BE) tn(1). The firmware is on
         * the PDCH of this TN: the BSP must play all its frames. Same
         * "publish on change" logic as the dedicated channels. */
        uint8_t tn = charge[8] & 0x07;
        uint8_t chan_nr = 0xC0 | tn;
        {
            uint32_t bfn = ldl_be_p(charge + 4);
            if (bfn % 104 == 12) {
                pdtch.ptcch++;
            } else if (plen >= 41) {
                pdtch.donnees++;
            } else if (plen >= 18 && charge[17] != 0xff) {
                pdtch.autre_tfi++;
            } else if (plen >= 18) {
                pdtch.crc_ko++;
                pdtch.ber_ko += ((unsigned)charge[12] << 8) | charge[13];
                if (pdtch.crc_ko <= 5)
                    fprintf(stderr, "[pdtch] bloc DL fn=%u TN=%d perdu (CRC faux, ber %u.%02u%%)\n", bfn, tn,
                            (((unsigned)charge[12] << 8) | charge[13]) / 100,
                            (((unsigned)charge[12] << 8) | charge[13]) % 100);
            }
            unsigned total = pdtch.donnees + pdtch.autre_tfi + pdtch.crc_ko + pdtch.ptcch;
            if (total % 100 == 0)
                pdtch_journal("releve");
        }
        tap.fn_dedie = ldl_be_p(charge + 4);
        tap.fn_dedie_vue = true;
        if (chan_nr != tap.dernier_chan_nr) {
            tap.dernier_chan_nr = chan_nr;
            dcch_cfg_publier(DCCH_PDCH, 0, chan_nr);
            pdtch_journal("PDCH arme");
        }
        return;
    }
    if (type == L1CTL_GPRS_UL_BLOCK_CNF) {
        pdtch.ul_cnf++;     /* does not touch dernier_chan_nr */
        return;
    }
    if ((type != L1CTL_DATA_CONF && type != L1CTL_DATA_IND) || plen < 5) {
        return;
    }
    /* chan_nr, 44.004 8.3: 0b001SSTTT = SDCCH/4, 0b01SSSTTT = SDCCH/8,
     * 0b00001TTT = TCH/F, 0b0001STTT = TCH/H.
     *
     * The TCH is a dedicated channel like any other: after the assignment
     * (DM_EST_REQ chan_nr=0x0a, TCH/F TS2), the DATA_IND/CONF of the FACCH
     * and of the SACCH/T carry 0x0a, and the tap announces the TCH (kind
     * 2/3). The SDCCH that was left is not "released" (0xFF remains the end
     * of the connection: pont/dsp/uplink.py _poll_release, montant.c) and
     * the readers keep it in memory for the ASSIGNMENT FAILURE case, where
     * the firmware returns to it and the tap republishes it (chan_nr
     * changes). */
    uint8_t chan_nr = charge[4];
    int kind = -1, ss = 0;
    if ((chan_nr & 0xE0) == 0x20) {
        kind = 0;
        ss = (chan_nr >> 3) & 0x03;
    } else if ((chan_nr & 0xC0) == 0x40) {
        kind = 1;
        ss = (chan_nr >> 3) & 0x07;
    } else if ((chan_nr & 0xF8) == 0x08) {
        kind = DCCH_TCH_F;
    } else if ((chan_nr & 0xF0) == 0x10) {
        kind = DCCH_TCH_H;
        ss = (chan_nr >> 3) & 0x01;
    }
    uint32_t fn = (plen >= 12) ? ldl_be_p(charge + 8) : 0;
    if (kind >= 0) {
        tap.fn_dedie = fn;
        tap.fn_dedie_vue = true;
        if (chan_nr != tap.dernier_chan_nr) {
            tap.dernier_chan_nr = chan_nr;
            dcch_cfg_publier(kind, ss, chan_nr);
        }
        return;
    }
    /* The release is detected here, not on d_dsp_page: the firmware does an
     * l1s_dsp_abort() (sync.c:308) precisely when it switches to the
     * dedicated channel ("resetting scheduler" in the mobile log, right
     * after the IMMEDIATE ASSIGNMENT), so d_dsp_page returning to 0 does not
     * mean the channel is released.
     *
     * The reliable signal is a block received or sent on a NON-dedicated
     * channel (BCCH, CCCH): the mobile is back on the common channels. In
     * dedicated mode it reads none, so this cannot happen in the middle of a
     * connection. */
    /* And only for a REAL common channel: 44.004 8.3 gives 0x80 BCCH,
     * 0x88 RACH, 0x90 PCH/AGCH. A chan_nr of 0x00 is not a channel and must
     * not release anything, otherwise the BSP would switch between TS0 and
     * TS1 at every block during the connection and no dedicated downlink
     * would be decodable. */
    if (tap.dernier_chan_nr != 0xFF && (chan_nr & 0xE0) == 0x80) {
        /* And not at the first CCCH block that comes: layer 1 still delivers
         * one or two of them just after the switch to the dedicated channel.
         * Releasing there would cut the dedicated downlink in the middle of
         * the establishment, so the common block is required to be clearly
         * LATER than the last block of the dedicated channel. */
        uint32_t ecart = tap.fn_dedie_vue ? ((fn - tap.fn_dedie) % 2715648u) : 0xFFFFFFFFu;
        if (!tap.fn_dedie_vue || (ecart >= DCCH_LIBERE_APRES_TRAMES && ecart < 2715648u / 2)) {
            tap.dernier_chan_nr = 0xFF;
            tap.fn_dedie_vue = false;
            dcch_cfg_publier(DCCH_RELEASED, 0, 0);
        }
    }
}

void calypso_dcch_tap_tx_byte(uint8_t byte)
{
    switch (tap.state) {
    case SC_IDLE:
        if (byte == SERCOMM_FLAG) {
            tap.state = SC_IN_FRAME;
            tap.len = 0;
        }
        break;
    case SC_IN_FRAME:
        if (byte == SERCOMM_FLAG) {
            if (tap.len > 0) {
                trame_complete();
            }
            tap.len = 0;
        } else if (byte == SERCOMM_ESCAPE) {
            tap.state = SC_ESCAPE;
        } else if (tap.len < (int)sizeof(tap.buf)) {
            tap.buf[tap.len++] = byte;
        }
        break;
    case SC_ESCAPE:
        if (tap.len < (int)sizeof(tap.buf)) {
            tap.buf[tap.len++] = byte ^ SERCOMM_ESCAPE_XOR;
        }
        tap.state = SC_IN_FRAME;
        break;
    }
}

void calypso_dcch_tap_reset(void)
{
    if (tap.dernier_chan_nr == 0xFF) {
        return;
    }
    tap.dernier_chan_nr = 0xFF;
    dcch_cfg_publier(DCCH_RELEASED, 0, 0);
}
