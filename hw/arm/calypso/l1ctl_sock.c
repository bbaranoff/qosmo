/*
 * l1ctl_sock.c - L1CTL unix socket server (legacy QEMU-internal path)
 *
 * Serves a unix socket (default /tmp/osmocom_l2) that speaks L1CTL
 * (length-prefixed messages) to the OsmocomBB mobile, translating between:
 *   - sercomm framing (FLAG/ESCAPE/DLCI) on the firmware UART side
 *   - L1CTL length prefixes on the mobile socket side
 *
 * NOT WIRED IN THIS TREE. The file builds (meson.build, under the l1-dsp
 * layer 1) but nothing calls l1ctl_sock_init(): the only call site lives in
 * the qosmo-dsp fork. In a qosmo launcher run osmocon owns /tmp/osmocom_l2
 * (osmocon -m romload -s /tmp/osmocom_l2) and relays to the firmware over the
 * modem pty, so the mobile never connects here. When it was wired, the
 * mobile -> firmware direction stayed empty: "RX<-mobile" logged 0 frames over
 * a whole run while osmocon saw the traffic. The blocks below that depend on
 * that direction are therefore dead, and marked as such.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/main-loop.h"
#include "hw/arm/calypso/calypso_uart.h"

#include <sys/socket.h>
#include <sys/un.h>
#include <fcntl.h>
#include <errno.h>

/* Sercomm constants */
#define SERCOMM_FLAG       0x7E
#define SERCOMM_ESCAPE     0x7D
#define SERCOMM_ESCAPE_XOR 0x20
#define SERCOMM_DLCI_L1CTL 5

/* L1CTL socket path */
#define L1CTL_SOCK_PATH    "/tmp/osmocom_l2"

#define L1CTL_LOG(fmt, ...) \
    fprintf(stderr, "[l1ctl-sock] " fmt "\n", ##__VA_ARGS__)

/* Readable L1CTL message names (l1ctl_proto.h), diagnostics only. This log
 * only ever sees the firmware -> mobile direction via sercomm; the real
 * mobile <-> firmware exchange is in osmocon.log (hdlc). */
static inline const char *l1ctl_tname(uint8_t t)
{
    switch (t) {
    case 0x01: return "FBSB_REQ";       case 0x02: return "FBSB_CONF";
    case 0x03: return "DATA_IND";       case 0x04: return "RACH_REQ";
    case 0x05: return "DM_EST_REQ";     case 0x06: return "DATA_REQ";
    case 0x07: return "RESET_IND";      case 0x08: return "PM_REQ";
    case 0x09: return "PM_CONF";        case 0x0c: return "RACH_CONF";
    case 0x0d: return "RESET_REQ";      case 0x0e: return "RESET_CONF";
    case 0x0f: return "DATA_CONF";      case 0x10: return "CCCH_MODE_REQ";
    case 0x11: return "CCCH_MODE_CONF"; case 0x12: return "DM_REL_REQ";
    case 0x13: return "PARAM_REQ";      default:   return "?";
    }
}

/* ---- Sercomm TX parser (firmware → mobile) ---- */

typedef enum {
    SC_IDLE,      /* waiting for FLAG */
    SC_IN_FRAME,  /* collecting frame bytes */
    SC_ESCAPE,    /* next byte is escaped */
} SercommState;

typedef struct L1CTLSock {
    /* Server socket */
    int srv_fd;

    /* Client connection */
    int cli_fd;

    /* Sercomm TX parser (firmware UART output → mobile) */
    SercommState sc_state;
    uint8_t  sc_buf[512];
    int      sc_len;

    /* L1CTL RX parser (mobile → firmware UART input) */
    uint8_t  lp_buf[4096];  /* length-prefix accumulator */
    int      lp_len;

    /* Reference to UART modem for RX injection */
    CalypsoUARTState *uart;
} L1CTLSock;

static L1CTLSock g_l1ctl;


/* ---- Sercomm helpers ---- */

static int sercomm_wrap(uint8_t dlci, const uint8_t *payload, int plen,
                        uint8_t *out, int out_size)
{
    int pos = 0;
    if (pos >= out_size) return -1;
    out[pos++] = SERCOMM_FLAG;

    /* DLCI + CTRL */
    uint8_t hdr[2] = { dlci, 0x03 };
    for (int i = 0; i < 2; i++) {
        if (hdr[i] == SERCOMM_FLAG || hdr[i] == SERCOMM_ESCAPE) {
            if (pos + 2 > out_size) return -1;
            out[pos++] = SERCOMM_ESCAPE;
            out[pos++] = hdr[i] ^ SERCOMM_ESCAPE_XOR;
        } else {
            if (pos + 1 > out_size) return -1;
            out[pos++] = hdr[i];
        }
    }

    /* Payload */
    for (int i = 0; i < plen; i++) {
        if (payload[i] == SERCOMM_FLAG || payload[i] == SERCOMM_ESCAPE) {
            if (pos + 2 > out_size) return -1;
            out[pos++] = SERCOMM_ESCAPE;
            out[pos++] = payload[i] ^ SERCOMM_ESCAPE_XOR;
        } else {
            if (pos + 1 > out_size) return -1;
            out[pos++] = payload[i];
        }
    }

    if (pos >= out_size) return -1;
    out[pos++] = SERCOMM_FLAG;
    return pos;
}

/* ---- Send L1CTL message to mobile (length-prefix) ---- */

static void l1ctl_send_to_mobile(L1CTLSock *s, const uint8_t *payload, int len)
{
    if (s->cli_fd < 0 || len <= 0 || len > UINT16_MAX) return;

    uint8_t hdr[2] = { (uint8_t)(len >> 8), (uint8_t)(len & 0xFF) };
    struct iovec iov[2] = {
        { .iov_base = hdr,                  .iov_len = sizeof(hdr) },
        { .iov_base = (void *)payload,      .iov_len = (size_t)len },
    };
    struct msghdr msg = { .msg_iov = iov, .msg_iovlen = 2 };

    int total = (int)sizeof(hdr) + len;
    ssize_t sent = sendmsg(s->cli_fd, &msg, MSG_NOSIGNAL);
    if (sent != total) {
        L1CTL_LOG("client send error (%zd/%d), closing", sent, total);
        close(s->cli_fd);
        s->cli_fd = -1;
    }
}

/* ---- Process a complete sercomm frame from firmware TX ---- */

static void sercomm_frame_complete(L1CTLSock *s)
{
    if (s->sc_len < 2) return;  /* need at least DLCI + CTRL */

    uint8_t dlci = s->sc_buf[0];
    /* uint8_t ctrl = s->sc_buf[1]; */
    uint8_t *payload = &s->sc_buf[2];
    int plen = s->sc_len - 2;

    if (dlci == SERCOMM_DLCI_L1CTL && plen > 0) {
        /* Current dedicated channel -> /dev/shm/calypso_dcch_cfg, for
         * external tools (l1-grgsm/calypso_l1ctl_tap.c reads it).
         *
         * Read it here, in the firmware -> mobile direction, because that is
         * the only L1CTL flow this file parses. DATA_CONF (0x0f) and DATA_IND
         * (0x03) carry l1ctl_info_dl.chan_nr in payload[4], filled by the
         * firmware from its own mframe scheduler, so it is OUR mobile's
         * channel and not a neighbour's. Not taken from the CCCH IMM ASSIGN,
         * which carries every subscriber's (68 for RA=0x07 and 12 for RA=0x0a
         * against a single RACH of ours at RA=0x08): the active subchannel
         * jumped 60 times per run.
         *
         * chan_nr (GSM 08.58 9.3.1): 001SSTTT = SDCCH/4, 01SSSTTT = SDCCH/8.
         * BCCH (0x80), CCCH (0x90) and TCH (00001TTT) are ignored here. */
        if ((payload[0] == 0x0f || payload[0] == 0x03) && plen >= 5) {
            uint8_t chan_nr = payload[4];
            int kind = -1, ss = 0;
            if ((chan_nr & 0xE0) == 0x20)      { kind = 0; ss = (chan_nr >> 3) & 0x03; }
            else if ((chan_nr & 0xC0) == 0x40) { kind = 1; ss = (chan_nr >> 3) & 0x07; }
            static uint8_t last_chan_nr = 0xFF;
            /* Publish on change only: while on a dedicated channel the
             * mobile also reads neighbour BCCHs for its measurements, so
             * chan_nr toggles constantly (121 transitions measured for 2
             * dedicated channels). */
            if (kind >= 0 && chan_nr != last_chan_nr) {
                static uint32_t dcch_seq;
                last_chan_nr = chan_nr;
                dcch_seq++;
                uint8_t b[16];
                memset(b, 0, sizeof(b));
                memcpy(b, &dcch_seq, 4);
                b[4] = (uint8_t)kind; b[5] = (uint8_t)ss;
                b[6] = chan_nr & 0x07; b[7] = chan_nr;
                int dfd = open("/dev/shm/calypso_dcch_cfg",
                               O_WRONLY | O_CREAT | O_TRUNC, 0666);
                if (dfd >= 0) {
                    if (write(dfd, b, sizeof(b)) < 0) { /* ignore */ }
                    close(dfd);
                }
                L1CTL_LOG("DCCH #%u : chan_nr=0x%02x -> SDCCH/%d SS=%d TN=%u "
                          "(vu sur %s)", dcch_seq, chan_nr, kind ? 8 : 4, ss,
                          chan_nr & 0x07, l1ctl_tname(payload[0]));
            }
        }
        L1CTL_LOG("TX→mobile: dlci=%d len=%d type=0x%02x %s", dlci, plen, payload[0],
                  l1ctl_tname(payload[0]));
        l1ctl_send_to_mobile(s, payload, plen);
    }
    /* Ignore other DLCIs (debug console, loader, etc.) */
}

/* ---- Feed firmware UART TX bytes into sercomm parser ---- */

void l1ctl_sock_uart_tx_byte(uint8_t byte)
{
    L1CTLSock *s = &g_l1ctl;

    switch (s->sc_state) {
    case SC_IDLE:
        if (byte == SERCOMM_FLAG) {
            s->sc_state = SC_IN_FRAME;
            s->sc_len = 0;
        }
        break;

    case SC_IN_FRAME:
        if (byte == SERCOMM_FLAG) {
            if (s->sc_len > 0) {
                sercomm_frame_complete(s);
            }
            /* Stay in IN_FRAME for next frame */
            s->sc_len = 0;
        } else if (byte == SERCOMM_ESCAPE) {
            s->sc_state = SC_ESCAPE;
        } else {
            if (s->sc_len < (int)sizeof(s->sc_buf)) {
                s->sc_buf[s->sc_len++] = byte;
            }
        }
        break;

    case SC_ESCAPE:
        if (s->sc_len < (int)sizeof(s->sc_buf)) {
            s->sc_buf[s->sc_len++] = byte ^ SERCOMM_ESCAPE_XOR;
        }
        s->sc_state = SC_IN_FRAME;
        break;
    }
}

/* ---- Receive L1CTL from mobile, inject into firmware UART RX ---- */

static void l1ctl_client_readable(void *opaque)
{
    L1CTLSock *s = (L1CTLSock *)opaque;

    uint8_t tmp[4096];
    ssize_t n = recv(s->cli_fd, tmp, sizeof(tmp), MSG_DONTWAIT);
    if (n < 0) {
        if (errno == EAGAIN || errno == EWOULDBLOCK)
            return;  /* no data available yet */
        L1CTL_LOG("client recv error: %s", strerror(errno));
        qemu_set_fd_handler(s->cli_fd, NULL, NULL, NULL);
        close(s->cli_fd);
        s->cli_fd = -1;
        s->lp_len = 0;
        return;
    }
    if (n == 0) {
        L1CTL_LOG("client disconnected");
        qemu_set_fd_handler(s->cli_fd, NULL, NULL, NULL);
        close(s->cli_fd);
        s->cli_fd = -1;
        s->lp_len = 0;
        return;
    }

    /* Accumulate in length-prefix buffer */
    if (s->lp_len + (int)n > (int)sizeof(s->lp_buf)) {
        s->lp_len = 0;  /* overflow, reset */
    }
    memcpy(&s->lp_buf[s->lp_len], tmp, n);
    s->lp_len += (int)n;

    /* Parse complete L1CTL messages */
    while (s->lp_len >= 2) {
        int msglen = (s->lp_buf[0] << 8) | s->lp_buf[1];
        if (s->lp_len < 2 + msglen) break;  /* incomplete */

        uint8_t *payload = &s->lp_buf[2];

        /* Kc capture (A5 ciphering): L1CTL_CRYPTO_REQ (0x15), mobile -> fw.
         * payload: [0]=0x15 [1]flags [2..3]pad [4]chan_nr [5]link_id [6..7]pad
         * [8]algo [9]key_len [10..]Kc. Written to /dev/shm/calypso_kc as
         * (seq, algo, key_len, Kc); si_bridge.py reads it and restarts grgsm
         * with -k to decrypt the downlink. The captured Kc is the one the
         * mobile derived (A8), i.e. exactly the network's. */
        if (payload[0] == 0x15 && msglen >= 10) {
            uint8_t algo = payload[8];
            uint8_t klen = payload[9];
            if (klen > 16) klen = 16;
            /* Guard on algo, matching the live writer in osmocon. Without
             * it an algo=0/klen=0 request still bumps the sequence number, and
             * a reader would take the file as holding a valid Kc and cipher
             * with an all-zero key. */
            if (algo >= 1 && algo <= 3 && 10 + (int)klen <= msglen) {
                static uint32_t kc_seq = 0;
                uint8_t kbuf[32];
                memset(kbuf, 0, sizeof(kbuf));
                kc_seq++;
                memcpy(kbuf, &kc_seq, 4);              /* [0..3] seq (LE) */
                kbuf[4] = algo; kbuf[5] = klen;        /* [4]algo [5]key_len */
                memcpy(kbuf + 6, &payload[10], klen);  /* [6..] Kc */
                int kfd = open("/dev/shm/calypso_kc",
                               O_WRONLY | O_CREAT | O_TRUNC, 0666);
                if (kfd >= 0) {
                    if (write(kfd, kbuf, sizeof(kbuf)) < 0) { /* ignore */ }
                    close(kfd);
                }
                L1CTL_LOG("CRYPTO_REQ: algo=%u klen=%u "
                          "Kc=%02x%02x%02x%02x%02x%02x%02x%02x -> "
                          "/dev/shm/calypso_kc#%u", algo, klen,
                          payload[10], payload[11], payload[12], payload[13],
                          payload[14], payload[15], payload[16], payload[17],
                          kc_seq);
            }
        }
        /* Clear the cipher on dedicated channel setup and release: every new
         * channel starts in the clear until its own CIPHER MODE COMMAND,
         * otherwise a stale Kc would cipher the next channel's SABM. */
        if (payload[0] == 0x05 || payload[0] == 0x12) {   /* DM_EST_REQ / DM_REL_REQ */
            /* ⚠️ DEAD IN THE CURRENT SETUP: this mobile -> firmware direction
             * measured 0 frames, so the Kc reset below never runs. Check this
             * before enabling A5/1. */
            int kfd = open("/dev/shm/calypso_kc",
                           O_WRONLY | O_CREAT | O_TRUNC, 0666);
            if (kfd >= 0) {
                uint8_t z[32]; memset(z, 0, sizeof(z));
                if (write(kfd, z, sizeof(z)) < 0) { /* ignore */ }
                close(kfd);
            }

        }

        /* Wrap in sercomm and inject into UART RX */
        uint8_t frame[1024];
        int flen = sercomm_wrap(SERCOMM_DLCI_L1CTL, payload, msglen,
                                frame, sizeof(frame));
        if (flen > 0 && s->uart) {
            L1CTL_LOG("RX←mobile: len=%d type=0x%02x %s → sercomm %d bytes",
                      msglen, payload[0], l1ctl_tname(payload[0]), flen);
            /* Hex dump of sercomm frame being injected */
            {
                fprintf(stderr, "[l1ctl-sock] INJECT %d bytes:", flen);
                for (int j = 0; j < flen && j < 32; j++)
                    fprintf(stderr, " %02x", frame[j]);
                if (flen > 32) fprintf(stderr, " ...");
                fprintf(stderr, "\n");
            }
            calypso_uart_receive(s->uart, frame, flen);
        }

        /* Consume from buffer */
        int consumed = 2 + msglen;
        memmove(s->lp_buf, &s->lp_buf[consumed], s->lp_len - consumed);
        s->lp_len -= consumed;
    }
}

/* ---- Accept new client connection ---- */

static void l1ctl_accept_cb(void *opaque)
{
    L1CTLSock *s = (L1CTLSock *)opaque;

    int fd = accept(s->srv_fd, NULL, NULL);
    if (fd < 0) return;

    /* Only one client at a time */
    if (s->cli_fd >= 0) {
        L1CTL_LOG("replacing existing client");
        qemu_set_fd_handler(s->cli_fd, NULL, NULL, NULL);
        close(s->cli_fd);
    }

    /* Set non-blocking */
    int flags = fcntl(fd, F_GETFL);
    fcntl(fd, F_SETFL, flags | O_NONBLOCK);

    s->cli_fd = fd;
    s->lp_len = 0;
    s->sc_state = SC_IDLE;
    s->sc_len = 0;

    qemu_set_fd_handler(fd, l1ctl_client_readable, NULL, s);
    L1CTL_LOG("client connected (fd=%d)", fd);
}

/* ---- Init ---- */

void l1ctl_sock_init(CalypsoUARTState *uart, const char *path)
{
    L1CTLSock *s = &g_l1ctl;
    memset(s, 0, sizeof(*s));
    s->srv_fd = -1;
    s->cli_fd = -1;
    s->uart = uart;

    if (!path) path = L1CTL_SOCK_PATH;

    /* Remove stale socket */
    unlink(path);

    /* Create unix socket server */
    s->srv_fd = socket(AF_UNIX, SOCK_STREAM, 0);
    if (s->srv_fd < 0) {
        L1CTL_LOG("ERROR: socket(): %s", strerror(errno));
        return;
    }

    struct sockaddr_un addr;
    memset(&addr, 0, sizeof(addr));
    addr.sun_family = AF_UNIX;
    strncpy(addr.sun_path, path, sizeof(addr.sun_path) - 1);

    if (bind(s->srv_fd, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        L1CTL_LOG("ERROR: bind(%s): %s", path, strerror(errno));
        close(s->srv_fd);
        s->srv_fd = -1;
        return;
    }

    if (listen(s->srv_fd, 1) < 0) {
        L1CTL_LOG("ERROR: listen(): %s", strerror(errno));
        close(s->srv_fd);
        s->srv_fd = -1;
        return;
    }

    /* Set non-blocking */
    int flags = fcntl(s->srv_fd, F_GETFL);
    fcntl(s->srv_fd, F_SETFL, flags | O_NONBLOCK);

    qemu_set_fd_handler(s->srv_fd, l1ctl_accept_cb, NULL, s);
    L1CTL_LOG("listening on %s", path);
}

/* ---- Manual poll (called from TDMA tick) ---- */

void l1ctl_sock_poll(void)
{
    L1CTLSock *s = &g_l1ctl;

    /* Try to accept a pending client */
    if (s->srv_fd >= 0 && s->cli_fd < 0) {
        int fd = accept(s->srv_fd, NULL, NULL);
        if (fd >= 0) {
            int flags = fcntl(fd, F_GETFL);
            fcntl(fd, F_SETFL, flags | O_NONBLOCK);
            s->cli_fd = fd;
            s->lp_len = 0;
            s->sc_state = SC_IDLE;
            s->sc_len = 0;
            qemu_set_fd_handler(fd, l1ctl_client_readable, NULL, s);
            L1CTL_LOG("client connected via poll (fd=%d)", fd);
        }
    }

    /* Try to read from connected client */
    if (s->cli_fd >= 0) {
        l1ctl_client_readable(s);
    }
}

bool l1ctl_client_active(void)
{
    return g_l1ctl.cli_fd >= 0;
}
