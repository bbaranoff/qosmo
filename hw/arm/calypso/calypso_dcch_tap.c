/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * calypso_dcch_tap.c - quel canal dedie le mobile utilise, pour le pont.
 *
 * pont.py ne lit pas lui-meme les IMMEDIATE ASSIGNMENT : sa classe Dedicated
 * (pont/state.py) attend qu'on lui dise le canal dans
 * /dev/shm/calypso_dcch_cfg, et tant que ce fichier n'existe pas,
 * uplink.py:_poll_sdcch() jette tous les blocs montants -- y compris le SABM
 * qui porte la LOCATION UPDATING REQUEST.
 *
 * En montage gr-gsm c'est l1-grgsm/calypso_l1ctl_tap.c qui l'ecrit, branche
 * par la vtable de la couche 1 (uart_tx_byte). Sous CALYPSO_DSP_EXTERN=1
 * cette couche 1 est desactivee (calypso_l1_disable), donc plus personne ne
 * l'ecrivait : mesure du 2026-09-21, le bloc SABM etait bien capture par
 * c54x_exe (src/montant.c) et publie dans /dev/shm/calypso_sdcch_ul, mais le
 * pont ne l'emettait jamais.
 *
 * Ce tap-ci ne depend d'aucune couche 1 : il renifle le flux sercomm que le
 * firmware envoie a osmocon et n'en retient que le numero de canal des
 * L1CTL_DATA_CONF / DATA_IND. Il ne publie que le side-band ; les reactions
 * propres a la couche 1 gr-gsm (fenetre a_cd, garde DCCH) restent chez elle.
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

#define SHM_DCCH_CFG        "/dev/shm/calypso_dcch_cfg"
#define DCCH_RELEASED       0xFF

static struct {
    enum { SC_IDLE, SC_IN_FRAME, SC_ESCAPE } state;
    uint8_t buf[512];
    int len;
    uint8_t dernier_chan_nr;
    uint32_t seq;
} tap = { .dernier_chan_nr = 0xFF };

/* 16 octets, comme les attend pont/state.py:Dedicated.read() :
 * seq(4) kind(1) ss(1) tn(1) chan_nr(1). kind 0 = SDCCH/4, 1 = SDCCH/8,
 * 0xFF = canal libere. */
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
    calypso_trx_dcch(kind, ss, chan_nr & 0x07);
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
        /* La trame d'emission du RACH, telle que le firmware la remonte. */
        calypso_trx_rach_conf(ldl_be_p(charge + 8));
        return;
    }
    if ((type != L1CTL_DATA_CONF && type != L1CTL_DATA_IND) || plen < 5) {
        return;
    }
    /* chan_nr, 44.004 8.3 : 0b001SSTTT = SDCCH/4, 0b01SSSTTT = SDCCH/8. */
    uint8_t chan_nr = charge[4];
    int kind = -1, ss = 0;
    if ((chan_nr & 0xE0) == 0x20) {
        kind = 0;
        ss = (chan_nr >> 3) & 0x03;
    } else if ((chan_nr & 0xC0) == 0x40) {
        kind = 1;
        ss = (chan_nr >> 3) & 0x07;
    }
    if (kind >= 0 && chan_nr != tap.dernier_chan_nr) {
        tap.dernier_chan_nr = chan_nr;
        dcch_cfg_publier(kind, ss, chan_nr);
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
