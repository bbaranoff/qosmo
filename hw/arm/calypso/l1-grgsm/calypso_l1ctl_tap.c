/* SPDX-License-Identifier: GPL-2.0-or-later */
#include "qemu/osdep.h"
#include "qemu/bswap.h"
#include "calypso_l1.h"
#include "calypso_l1ctl_tap.h"
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
    uint8_t last_chan_nr;
    uint32_t dcch_seq;
} tap = { .last_chan_nr = 0xFF };

static void dcch_cfg_publish(int kind, int ss, uint8_t chan_nr)
{
    uint8_t b[16] = {0};
    tap.dcch_seq++;
    memcpy(b, &tap.dcch_seq, 4);
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
}

static void frame_complete(void)
{
    if (tap.len < 3 || tap.buf[0] != SERCOMM_DLCI_L1CTL) {
        return;
    }
    const uint8_t *payload = &tap.buf[2];
    int plen = tap.len - 2;
    uint8_t type = payload[0];

    if (type == L1CTL_RACH_CONF && plen >= 12) {
        calypso_l1_rach_conf(ldl_be_p(payload + 8));
        return;
    }
    if ((type != L1CTL_DATA_CONF && type != L1CTL_DATA_IND) || plen < 5) {
        return;
    }
    uint8_t chan_nr = payload[4];
    int kind = -1, ss = 0;
    if ((chan_nr & 0xE0) == 0x20) {
        kind = 0;
        ss = (chan_nr >> 3) & 0x03;
    } else if ((chan_nr & 0xC0) == 0x40) {
        kind = 1;
        ss = (chan_nr >> 3) & 0x07;
    }
    bool tch = ((chan_nr & 0xF8) == 0x08) || ((chan_nr & 0xF0) == 0x10);
    bool dedicated = tch || kind >= 0;
    if (dedicated) {
        calypso_l1_dcch_active();
        calypso_l1_dcch_is_tch(tch);
    }
    if (kind >= 0 && chan_nr != tap.last_chan_nr) {
        tap.last_chan_nr = chan_nr;
        dcch_cfg_publish(kind, ss, chan_nr);
        calypso_l1_dcch_set(kind, ss);
    }
}

void calypso_l1ctl_tap_tx_byte(uint8_t byte)
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
                frame_complete();
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

void calypso_l1ctl_tap_channel_released(void)
{
    if (tap.last_chan_nr == 0xFF) {
        return;
    }
    tap.last_chan_nr = 0xFF;
    dcch_cfg_publish(DCCH_RELEASED, 0, 0);
}
