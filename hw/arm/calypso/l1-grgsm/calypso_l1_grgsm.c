/* SPDX-License-Identifier: GPL-2.0-or-later */
#include "qemu/osdep.h"
#include "qemu/main-loop.h"
#include "qemu/bswap.h"
#include "exec/cpu-common.h"
#include "hw/arm/calypso/calypso_api.h"
#include "calypso_l1.h"
#include "calypso_l1ctl_tap.h"
#include "hw/arm/calypso/calypso_trf6151.h"
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <fcntl.h>
#include <unistd.h>
#include <errno.h>

#define GSMTAP_PORT             4730
#define SCH_PORT                4731
#define GSMTAP_HDR_LEN          16
#define GSMTAP_TYPE_UM          0x01
#define GSMTAP_CHANNEL_BCCH     0x01
#define GSMTAP_CHANNEL_AGCH     0x04
#define GSMTAP_CHANNEL_SDCCH4   0x07
#define GSMTAP_CHANNEL_TCH_F    0x09
#define GSMTAP_CHANNEL_ACCH     0x80
#define GSMTAP_CHANNEL_SACCH    (GSMTAP_CHANNEL_SDCCH4 | GSMTAP_CHANNEL_ACCH)
#define GSMTAP_CHANNEL_TCH_ACCH (GSMTAP_CHANNEL_TCH_F | GSMTAP_CHANNEL_ACCH)

#define SHM_RACH        "/dev/shm/calypso_rach"
#define SHM_SDCCH_UL    "/dev/shm/calypso_sdcch_ul"
#define SHM_FACCH_UL    "/dev/shm/calypso_tch_facch_ul"
#define SHM_TSACCH_UL   "/dev/shm/calypso_tch_sacch_ul"
#define SHM_TCH_UL      "/dev/shm/calypso_tch_ul"
#define SHM_TCH_DL      "/dev/shm/calypso_tch_dl"
#define SHM_TCH_CFG     "/dev/shm/calypso_tch_cfg"
#define SHM_KC          "/dev/shm/calypso_kc_l1"

#define TOA_ON_TIME     23
#define SNR_NOMINAL     0x7000
#define RF_SERVING_DBM  (-60)
#define RF_SILENT_DBM   (-110)

#define SB_MAX_AGE_FRAMES   104
#define AGCH_TTL_TICKS      100
#define SDCCH_TTL_TICKS     4000
#define SDCCH_MAX_PRESENT   8
#define SDCCH_RING_N        32
#define SDCCH_UL_DEDUP_TICKS 60
#define SDCCH_UL_WINDOW_OFS 6
#define DCCH_GUARD_TICKS    108
#define TCH_DL_Q_N          8
#define TCH_DL_PREFETCH     4
#define TCH_DL_SLOT         48
#define TCH_DL_HOLD         5
#define TCH_TTL_TICKS       26
#define TCH_UL_SLOTS        16
#define TCH_UL_SLOT_SZ      64
#define KC_PUBLISH_EVERY    22
/* Delai (en appels de publish_kc, soit ~100 ms chacun) avant d'annoncer un
 * passage en clair. Le firmware efface d_a5mode a chaque DM_REL_REQ, y compris
 * pendant un Assignment Command ou la Kc est rechargee juste apres ; la BTS,
 * elle, ne cesse jamais de chiffrer. */
#define KC_CLEAR_GRACE      5
#define KC_RECLEN           32

#define L1_LOG(fmt, ...) fprintf(stderr, "[l1] " fmt "\n", ##__VA_ARGS__)

struct sdcch_entry {
    uint8_t  l2[23];
    uint32_t fn;
    uint32_t tick;
    uint16_t reps;
};

static struct {
    bool     pending;
    uint8_t  page;
    uint16_t d_task_md, d_task_d, d_task_u, d_task_ra, d_burst_d, d_fn;
    uint32_t tick;

    uint8_t  si_set[6][23];
    bool     si_have[6];
    uint8_t  si_buf[23];
    bool     si_valid;
    int      si_rr;
    unsigned si_rot;
    uint8_t  sacch_buf[23];
    bool     sacch_have;
    bool     sacch_real;

    int      serving_arfcn;
    uint8_t  sb_bsic;
    uint32_t sb_fn;
    bool     sb_valid;
    uint32_t sb_capture_fn;

    uint8_t  agch_buf[23];
    bool     agch_valid;
    uint32_t agch_tick;

    struct sdcch_entry sdcch_ring[SDCCH_RING_N];
    uint32_t sdcch_head, sdcch_tail;
    uint32_t sdcch_last_fn;
    bool     sdcch_valid;
    uint8_t  sdcch_ss;
    bool     sdcch_ss_set;
    bool     sdcch_ch8;

    bool     dcch_guard_armed;
    uint32_t dcch_guard_tick;
    bool     dcch_is_tch;

    uint8_t  tch_dl_q[TCH_DL_Q_N][33];
    uint32_t tch_dl_q_seq[TCH_DL_Q_N];
    unsigned tch_dl_q_head, tch_dl_q_tail;
    uint32_t tch_dl_seq;
    uint8_t  facch_dl[23];
    bool     facch_dl_valid;
    uint32_t facch_dl_tick;
    uint8_t  tsacch_dl[23];
    bool     tsacch_dl_valid;
    uint32_t tsacch_dl_tick;
    bool     tch_cfg_valid;

    uint8_t  last_ra;
    uint32_t rach_conf_fn[256];

    uint32_t l1s_addr;
    uint32_t last_rach_addr;
} g;

static uint32_t elf_symbol(const char *path, const char *want)
{
    FILE *f = fopen(path, "rb");
    if (!f) {
        return 0;
    }
    fseek(f, 0, SEEK_END);
    long sz = ftell(f);
    fseek(f, 0, SEEK_SET);
    if (sz < 52 || sz > (64L << 20)) {
        fclose(f);
        return 0;
    }
    uint8_t *b = g_malloc((size_t)sz);
    size_t got = fread(b, 1, (size_t)sz, f);
    fclose(f);
    uint32_t ret = 0;
    if (got == (size_t)sz && b[0] == 0x7f && b[1] == 'E' && b[2] == 'L' &&
        b[3] == 'F' && b[4] == 1) {
#define R16(o) ((uint32_t)b[o] | ((uint32_t)b[(o) + 1] << 8))
#define R32(o) (R16(o) | (R16((o) + 2) << 16))
        uint32_t shoff = R32(0x20), shent = R16(0x2e), shnum = R16(0x30);
        for (uint32_t si = 0; si < shnum; si++) {
            uint32_t sh = shoff + si * shent;
            if ((long)(sh + 40) > sz) {
                break;
            }
            if (R32(sh + 4) != 2) {
                continue;
            }
            uint32_t symoff = R32(sh + 0x10), symsz = R32(sh + 0x14);
            uint32_t link = R32(sh + 0x18), entsz = R32(sh + 0x24);
            uint32_t strsh = shoff + link * shent;
            if ((long)(strsh + 40) > sz || entsz < 16) {
                break;
            }
            uint32_t stroff = R32(strsh + 0x10), strsz = R32(strsh + 0x14);
            for (uint32_t o = 0; o + 16 <= symsz && (long)(symoff + o + 16) <= sz;
                 o += entsz) {
                uint32_t ni = R32(symoff + o);
                if (ni < strsz && !strcmp((const char *)(b + stroff + ni), want)) {
                    ret = R32(symoff + o + 4);
                    break;
                }
            }
            break;
        }
#undef R16
#undef R32
    }
    g_free(b);
    return ret;
}

static uint32_t arm_read32(uint32_t addr)
{
    uint32_t v = 0;
    if (!addr) {
        return 0;
    }
    cpu_physical_memory_read(addr, &v, sizeof(v));
    return le32_to_cpu(v);
}

uint32_t calypso_l1s_fn(void)
{
    return arm_read32(g.l1s_addr);
}

static uint32_t last_rach_fn(void)
{
    return arm_read32(g.last_rach_addr);
}

static int tc51(long fn)
{
    return (int)((fn % 51 + 51) % 51);
}

static void ndb_put_l2(uint16_t *w, const uint8_t *l2)
{
    for (int i = 0; i < 23; i += 2) {
        uint8_t lo = l2[i], hi = (i + 1 < 23) ? l2[i + 1] : 0x2B;
        w[3 + i / 2] = (uint16_t)(lo | (hi << 8));
    }
}

static void ndb_put_fr(uint16_t *w, const uint8_t *fr)
{
    for (int i = 0; i < 32; i += 2) {
        w[3 + i / 2] = (uint16_t)(((uint16_t)fr[i] << 8) | fr[i + 1]);
    }
    w[3 + 16] = (uint16_t)((uint16_t)fr[32] << 8);
}

static void ndb_block(unsigned off, uint16_t hdr, const uint8_t *l2)
{
    uint16_t *w = api_ndb(off);
    w[0] = hdr;
    w[1] = 0;
    w[2] = 0;
    ndb_put_l2(w, l2);
}

static void rp_serv_demod(uint8_t page, uint16_t toa, uint16_t pm)
{
    uint16_t *d = api_rp(page, RP_A_SERV_DEMOD);
    d[D_TOA] = toa;
    d[D_PM] = pm;
    d[D_ANGLE] = 0;
    d[D_SNR] = SNR_NOMINAL;
}

static bool on_serving_cell(void)
{
    int tuned = calypso_trf6151_arfcn();
    return tuned < 0 || tuned == g.serving_arfcn;
}

static uint16_t apm_nominal(void)
{
    return calypso_trf6151_apm_for_rf(on_serving_cell() ? RF_SERVING_DBM : RF_SILENT_DBM);
}

static uint32_t encode_sb(uint8_t bsic, uint32_t fn)
{
    uint16_t t1 = (uint16_t)(fn / (26u * 51u));
    uint8_t t2 = (uint8_t)(fn % 26u);
    uint8_t t3 = (uint8_t)(fn % 51u);
    uint8_t t3p = (t3 == 0) ? 0 : ((t3 - 1) / 10);
    uint32_t sb = 0;
    sb |= ((uint32_t)(bsic & 0x3f)) << 2;
    sb |= ((uint32_t)(t1 & 0x001)) << 23;
    sb |= ((uint32_t)(t1 & 0x1fe)) << 7;
    sb |= ((uint32_t)(t1 & 0x600)) >> 9;
    sb |= ((uint32_t)(t2 & 0x1f)) << 18;
    sb |= ((uint32_t)(t3p & 1)) << 24;
    sb |= ((uint32_t)(t3p & 6)) << 15;
    return sb;
}

static void dispatch_pm(uint8_t page)
{
    uint16_t pm = apm_nominal();
    uint16_t *a = api_rp(page, RP_A_PM);
    a[0] = pm;
    a[1] = pm;
    a[2] = pm;
    *api_rp(page, RP_D_TASK_MD) = PM_DSP_TASK;
}

static void dispatch_sb(uint8_t page)
{
    if (!g.sb_valid || !on_serving_cell()) {
        return;
    }
    if ((int)(calypso_trx_get_fn() - g.sb_capture_fn) > SB_MAX_AGE_FRAMES) {
        return;
    }
    uint32_t sb = encode_sb(g.sb_bsic, g.sb_fn);
    uint16_t *a = api_rp(page, RP_A_SCH);
    a[0] = 0;
    a[1] = 0;
    a[2] = 0;
    a[3] = (uint16_t)(sb & 0xFFFF);
    a[4] = (uint16_t)(sb >> 16);
    rp_serv_demod(page, TOA_ON_TIME, apm_nominal());
    *api_rp(page, RP_D_TASK_MD) = SB_DSP_TASK;
}

static void present_ccch(uint8_t page, const uint8_t *l2)
{
    ndb_block(NDB_A_CD, 0, l2);
    *api_rp(page, RP_D_TASK_D) = ALLC_DSP_TASK;
    rp_serv_demod(page, TOA_ON_TIME, apm_nominal());
}

static void rotate_si(void)
{
    for (int k = 1; k <= 6; k++) {
        int s = (g.si_rr + k) % 6;
        if (g.si_have[s]) {
            g.si_rr = s;
            return;
        }
    }
}

static bool dcch_si_guard(void)
{
    if (!g.dcch_guard_armed) {
        return false;
    }
    if (g.tick - g.dcch_guard_tick > DCCH_GUARD_TICKS) {
        g.dcch_guard_armed = false;
        calypso_l1ctl_tap_channel_released();
        L1_LOG("canal dedie libere");
        return false;
    }
    return true;
}

static void dispatch_allc(uint8_t page)
{
    if (g.agch_valid) {
        if (g.tick - g.agch_tick > AGCH_TTL_TICKS) {
            g.agch_valid = false;
        } else {
            int tc = tc51(calypso_l1s_fn());
            if ((tc >= 6 && tc <= 9) || (tc >= 12 && tc <= 19)) {
                present_ccch(page, g.agch_buf);
                return;
            }
        }
    }

    int lo = g.sdcch_ss_set ? (g.sdcch_ch8 ? 0 : 22) : 0;
    int hi = g.sdcch_ss_set ? (g.sdcch_ch8 ? 31 : 39) : 39;
    while (g.sdcch_tail != g.sdcch_head) {
        struct sdcch_entry *e = &g.sdcch_ring[g.sdcch_head % SDCCH_RING_N];
        if (g.tick - e->tick > SDCCH_TTL_TICKS) {
            g.sdcch_head++;
            continue;
        }
        int tc = tc51(calypso_l1s_fn());
        if (tc < lo || tc > hi) {
            break;
        }
        present_ccch(page, e->l2);
        e->reps++;
        if (g.d_burst_d >= 3 || e->reps >= SDCCH_MAX_PRESENT) {
            g.sdcch_head++;
        }
        if (g.sdcch_tail == g.sdcch_head) {
            g.sdcch_valid = false;
        }
        return;
    }
    if (g.sdcch_tail == g.sdcch_head) {
        g.sdcch_valid = false;
    }

    if (g.sacch_have) {
        int tc = tc51(calypso_l1s_fn());
        if (tc >= 42 && tc <= 46) {
            present_ccch(page, g.sacch_buf);
            return;
        }
    }

    if (g.sdcch_valid) {
        return;
    }
    if (g.d_burst_d == 0) {
        rotate_si();
        memcpy(g.si_buf, g.si_set[g.si_rr], 23);
    }
    if (g.dcch_guard_armed) {
        *api_ndb(NDB_A_CD) = 0x0003;
        *api_rp(page, RP_D_TASK_D) = ALLC_DSP_TASK;
        return;
    }
    ndb_block(NDB_A_CD, 0, g.si_buf);
    for (uint8_t p = 0; p < 2; p++) {
        *api_rp(p, RP_D_TASK_D) = ALLC_DSP_TASK;
        rp_serv_demod(p, TOA_ON_TIME, apm_nominal());
    }
}

static bool tch_fresh(bool valid, uint32_t tick)
{
    return valid && (g.tick - tick <= TCH_TTL_TICKS);
}

static void tch_serv_demod(void)
{
    uint16_t pm = apm_nominal();
    rp_serv_demod(0, TOA_ON_TIME, pm);
    rp_serv_demod(1, TOA_ON_TIME, pm);
}

static unsigned tch_dl_depth(void)
{
    return g.tch_dl_q_tail - g.tch_dl_q_head;
}

static void tch_dl_push(const uint8_t *fr, uint32_t seq)
{
    unsigned i = g.tch_dl_q_tail % TCH_DL_Q_N;
    memcpy(g.tch_dl_q[i], fr, 33);
    g.tch_dl_q_seq[i] = seq;
    g.tch_dl_q_tail++;
}

static void tch_dl_poll(void)
{
    static int fd = -2;
    if (fd == -2) {
        fd = open(SHM_TCH_DL, O_CREAT | O_RDWR, 0644);
    }
    if (fd < 0) {
        return;
    }
    uint8_t hdr[8];
    if (pread(fd, hdr, sizeof(hdr), 0) != (ssize_t)sizeof(hdr)) {
        return;
    }
    uint32_t w_seq, n_slots;
    memcpy(&w_seq, hdr, 4);
    memcpy(&n_slots, hdr + 4, 4);
    if (n_slots == 0 || n_slots > 4096 || w_seq == 0 || w_seq == g.tch_dl_seq) {
        return;
    }
    if (w_seq < g.tch_dl_seq) {
        g.tch_dl_seq = 0;
        g.tch_dl_q_head = g.tch_dl_q_tail = 0;
    }
    if (tch_dl_depth() >= TCH_DL_PREFETCH) {
        return;
    }
    uint32_t next = g.tch_dl_seq + 1;
    if (g.tch_dl_seq == 0 || (w_seq - next) >= n_slots) {
        uint32_t behind = (w_seq > next) ? (w_seq - next) : 0;
        if (g.tch_dl_seq && behind >= n_slots) {
            static unsigned nlog;
            if (nlog++ < 10) {
                L1_LOG("TCH DL : %u trames sautees (anneau de %u)",
                       behind - (n_slots - 1), n_slots);
            }
        }
        next = w_seq;
    }
    uint8_t buf[TCH_DL_SLOT];
    off_t off = 8 + (off_t)((next - 1) % n_slots) * TCH_DL_SLOT;
    if (pread(fd, buf, sizeof(buf), off) != (ssize_t)sizeof(buf)) {
        return;
    }
    uint32_t sseq;
    memcpy(&sseq, buf, 4);
    if (sseq != next) {
        return;
    }
    g.tch_dl_seq = next;
    tch_dl_push(buf + 8, next);
}

static void dispatch_tch_dl(void)
{
    tch_serv_demod();
    static unsigned waited;
    bool ready = true;
    if (*api_ndb(NDB_A_DD_0) & B_BLUD) {
        if (waited < TCH_DL_HOLD) {
            waited++;
            ready = false;
        }
    }
    if (ready) {
        waited = 0;
        if (g.tch_dl_q_head != g.tch_dl_q_tail) {
            unsigned i = g.tch_dl_q_head++ % TCH_DL_Q_N;
            uint16_t *w = api_ndb(NDB_A_DD_0);
            w[0] = B_BLUD;
            w[1] = 0;
            w[2] = 0;
            ndb_put_fr(w, g.tch_dl_q[i]);
        }
    }
    if (tch_fresh(g.facch_dl_valid, g.facch_dl_tick)) {
        ndb_block(NDB_A_FD, B_BLUD, g.facch_dl);
        g.facch_dl_valid = false;
    }
}

static void dispatch_tch_sacch(void)
{
    tch_serv_demod();
    if (!tch_fresh(g.tsacch_dl_valid, g.tsacch_dl_tick)) {
        return;
    }
    ndb_block(NDB_A_CD, B_BLUD, g.tsacch_dl);
    g.tsacch_dl_valid = false;
}

static void dcch_sacch_present(void)
{
    static const uint8_t base4[4] = { 42, 46, 93, 97 };
    static const uint8_t base8[8] = { 32, 36, 40, 44, 83, 87, 91, 95 };
    if (g.dcch_is_tch) {
        return;
    }
    uint32_t b;
    if (g.sdcch_ch8) {
        b = base8[(g.sdcch_ss / 4u) & 7u];
    } else {
        unsigned idx = 0;
        switch (g.sdcch_ss) {
        case 26: idx = 1; break;
        case 32: idx = 2; break;
        case 36: idx = 3; break;
        default: break;
        }
        b = base4[idx];
    }
    uint32_t f102 = calypso_trx_get_fn() % 102u;
    if (f102 < b || f102 > b + 3u) {
        return;
    }
    if (*api_ndb(NDB_A_CD) & B_BLUD) {
        return;
    }
    if (!g.sacch_have) {
        return;
    }
    ndb_block(NDB_A_CD, B_BLUD, g.sacch_buf);
}

static int sideband_open(const char *path, off_t len)
{
    int fd = open(path, O_CREAT | O_RDWR, 0644);
    if (fd >= 0 && ftruncate(fd, len) < 0) {
        close(fd);
        return -1;
    }
    return fd;
}

static void sideband_write(int fd, const void *buf, size_t n, off_t off)
{
    if (fd >= 0 && pwrite(fd, buf, n, off) < 0) {
        return;
    }
}

static void publish_l2(int *fdp, const char *path, uint32_t *seq, const uint8_t *l2,
                       uint16_t task_u)
{
    if (*fdp == -2) {
        *fdp = sideband_open(path, 48);
    }
    uint32_t fn = calypso_trx_get_fn(), l1s = calypso_l1s_fn();
    uint8_t buf[48] = {0};
    (*seq)++;
    memcpy(buf + 0, seq, 4);
    memcpy(buf + 4, &l1s, 4);
    memcpy(buf + 8, &fn, 4);
    memcpy(buf + 12, &task_u, 2);
    buf[14] = (uint8_t)(l1s % 51);
    memcpy(buf + 16, l2, 23);
    sideband_write(*fdp, buf, sizeof(buf), 0);
}

static void publish_speech(const uint8_t *fr)
{
    static int fd = -2;
    static uint32_t seq;
    if (fd == -2) {
        fd = sideband_open(SHM_TCH_UL, 8 + TCH_UL_SLOTS * TCH_UL_SLOT_SZ);
        uint32_t hdr[2] = { 0, TCH_UL_SLOTS };
        sideband_write(fd, hdr, sizeof(hdr), 0);
    }
    uint32_t fn = calypso_trx_get_fn(), l1s = calypso_l1s_fn();
    uint8_t buf[TCH_UL_SLOT_SZ] = {0};
    seq++;
    memcpy(buf + 0, &seq, 4);
    memcpy(buf + 4, &l1s, 4);
    memcpy(buf + 8, &fn, 4);
    memcpy(buf + 16, fr, 33);
    sideband_write(fd, buf, sizeof(buf), 8 + (off_t)((seq - 1) % TCH_UL_SLOTS) * TCH_UL_SLOT_SZ);
    sideband_write(fd, &seq, 4, 0);
}

static bool take_ul(unsigned off, uint8_t *out, int n)
{
    uint16_t *w = api_ndb(off);
    if (!(w[0] & B_BLUD)) {
        return false;
    }
    for (int i = 0; i < n; i += 2) {
        uint16_t v = w[3 + i / 2];
        uint8_t first = (n == 33) ? (uint8_t)(v >> 8) : (uint8_t)(v & 0xff);
        uint8_t second = (n == 33) ? (uint8_t)(v & 0xff) : (uint8_t)(v >> 8);
        out[i] = first;
        if (i + 1 < n) {
            out[i + 1] = second;
        }
    }
    w[0] &= (uint16_t)~B_BLUD;
    return true;
}

static bool capture_tch_ul(uint16_t task_u)
{
    static int fd_facch = -2, fd_sacch = -2;
    static uint32_t seq_facch, seq_sacch;
    uint8_t l2[23], fr[33];
    switch (task_u & 0x7FFF) {
    case TCHT_DSP_TASK:
        if (take_ul(NDB_A_FU, l2, 23)) {
            publish_l2(&fd_facch, SHM_FACCH_UL, &seq_facch, l2, task_u);
        }
        if (take_ul(NDB_A_DU_1, fr, 33)) {
            publish_speech(fr);
        }
        return true;
    case TCHA_DSP_TASK:
        if (take_ul(NDB_A_CU, l2, 23)) {
            publish_l2(&fd_sacch, SHM_TSACCH_UL, &seq_sacch, l2, task_u);
        }
        return true;
    case TCHD_DSP_TASK:
        return true;
    default:
        return false;
    }
}

static void capture_sdcch_ul(uint16_t task_u)
{
    static int fd = -2;
    static uint32_t seq;
    static uint8_t last[23];
    static uint32_t last_tick;
    static bool have_last;

    uint8_t win[30];
    const uint8_t *src = (const uint8_t *)api_ndb(NDB_A_CU + SDCCH_UL_WINDOW_OFS);
    memcpy(win, src, sizeof(win));
    int kk = 0;
    for (int j = 0; j <= 6; j++) {
        uint8_t a = win[j], c = win[j + 1], l = win[j + 2];
        int sapi = (a >> 2) & 7;
        bool addr_ok = (a & 0x01) && ((a & 0x60) == 0) && (sapi == 0 || sapi == 3);
        bool ctrl_ok = (c != 0x2b) && (c != 0xff);
        bool len_ok = (l & 0x01) && ((l >> 2) <= 20);
        if (addr_ok && ctrl_ok && len_ok) {
            kk = j;
            break;
        }
    }
    const uint8_t *l2 = win + kk;
    if (have_last && !memcmp(last, l2, 23) && g.tick - last_tick < SDCCH_UL_DEDUP_TICKS) {
        return;
    }
    memcpy(last, l2, 23);
    last_tick = g.tick;
    have_last = true;
    if (l2[1] == 0x03) {
        return;
    }
    publish_l2(&fd, SHM_SDCCH_UL, &seq, l2, task_u);
}

static void l1_reset(void)
{
    static unsigned n;
    if (n++ < 3) {
        L1_LOG("reset L1 (d_dsp_page=0)");
    }
    g.agch_valid = false;
    g.sdcch_valid = false;
    g.sdcch_ss_set = false;
    g.dcch_guard_armed = false;
    calypso_l1ctl_tap_channel_released();
}

void calypso_l1_page_written(uint16_t v)
{
    if (!(v & B_GSM_TASK)) {
        if (v == 0) {
            l1_reset();
        }
        return;
    }
    uint8_t page = (v & B_GSM_PAGE) ? 1 : 0;
    g.page = page;
    g.d_task_d = *api_wp(page, WP_D_TASK_D);
    g.d_task_u = *api_wp(page, WP_D_TASK_U);
    g.d_task_md = *api_wp(page, WP_D_TASK_MD);
    g.d_task_ra = *api_wp(page, WP_D_TASK_RA);
    g.d_fn = *api_wp(page, WP_D_FN);
    if (g.d_fn == 0) {
        g.d_fn = (uint16_t)(calypso_trx_get_fn() & 0xFFFF);
    }
    g.pending = true;

    if (g.dcch_guard_armed &&
        (g.d_task_u == DUL_DSP_TASK || g.d_task_u == TCHT_DSP_TASK ||
         g.d_task_u == TCHA_DSP_TASK)) {
        g.dcch_guard_tick = g.tick;
    }
    if (g.d_task_u != 0 && !capture_tch_ul(g.d_task_u)) {
        capture_sdcch_ul(g.d_task_u);
    }
}

void calypso_l1_burst_written(uint16_t d_burst_d)
{
    g.d_burst_d = (uint16_t)(d_burst_d & 3);
}

void calypso_l1_rach_written(uint16_t d_rach, uint32_t fn)
{
    static int fd = -2;
    static uint32_t seq;
    if (fd == -2) {
        fd = sideband_open(SHM_RACH, 16);
    }
    uint8_t ra = (uint8_t)(d_rach >> 8);
    uint8_t bsic = (uint8_t)((d_rach & 0xFF) >> 2);
    uint8_t buf[16] = {0};
    seq++;
    memcpy(buf + 0, &seq, 4);
    buf[4] = ra;
    buf[5] = bsic;
    memcpy(buf + 8, &fn, 4);
    sideband_write(fd, buf, sizeof(buf), 0);
    g.last_ra = ra;
}

void calypso_l1_rach_conf(uint32_t fn)
{
    g.rach_conf_fn[g.last_ra] = fn;
}

bool calypso_l1_read_override(uint32_t off, uint16_t *out)
{
    if (!g.sb_valid || !on_serving_cell()) {
        switch (off) {
        case API_NDB + NDB_D_FB_DET:
        case API_NDB + NDB_A_SYNC_DEMOD + 2 * D_TOA:
        case API_NDB + NDB_A_SYNC_DEMOD + 2 * D_PM:
        case API_NDB + NDB_A_SYNC_DEMOD + 2 * D_ANGLE:
        case API_NDB + NDB_A_SYNC_DEMOD + 2 * D_SNR:
            *out = 0;
            return true;
        default:
            return false;
        }
    }
    switch (off) {
    case API_NDB + NDB_D_FB_DET:
        *out = 1;
        return true;
    case API_NDB + NDB_A_SYNC_DEMOD + 2 * D_PM:
        *out = apm_nominal();
        return true;
    case API_NDB + NDB_A_SYNC_DEMOD + 2 * D_TOA:
    case API_NDB + NDB_A_SYNC_DEMOD + 2 * D_ANGLE:
    case API_R_PAGE(0) + RP_A_SERV_DEMOD + 2 * D_TOA:
    case API_R_PAGE(1) + RP_A_SERV_DEMOD + 2 * D_TOA:
        *out = 0;
        return true;
    case API_NDB + NDB_A_SYNC_DEMOD + 2 * D_SNR:
        *out = SNR_NOMINAL;
        return true;
    default:
        return false;
    }
}

bool calypso_l1_si_valid(void)
{
    return g.si_valid;
}

void calypso_l1_dcch_set(int kind, int ss)
{
    static const uint8_t b4[4] = { 22, 26, 32, 36 };
    uint8_t base = kind ? (uint8_t)((ss & 7) * 4) : b4[ss & 3];
    if (g.sdcch_ss_set && g.sdcch_ss == base && g.sdcch_ch8 == (kind != 0)) {
        return;
    }
    g.sdcch_ss = base;
    g.sdcch_ss_set = true;
    g.sdcch_ch8 = (kind != 0);
    L1_LOG("canal dedie SDCCH/%d SS=%d : fenetre a_cd fn%%51 %u-%u",
           kind ? 8 : 4, ss, base, base + 3);
}

void calypso_l1_dcch_active(void)
{
    g.dcch_guard_tick = g.tick;
    g.dcch_guard_armed = true;
}

void calypso_l1_dcch_is_tch(bool on)
{
    g.dcch_is_tch = on;
}

static void feed_agch(const uint8_t *l2, int len)
{
    uint8_t mt = l2[2];
    bool imm = (mt == 0x3f || mt == 0x3a || mt == 0x3b);
    uint8_t cur = g.agch_buf[2];
    bool cur_imm = (cur == 0x3f || cur == 0x3a || cur == 0x3b);
    if (!imm && g.agch_valid && cur_imm && g.tick - g.agch_tick <= AGCH_TTL_TICKS) {
        return;
    }
    int n = len < 23 ? len : 23;
    memcpy(g.agch_buf, l2, n);
    memset(g.agch_buf + n, 0x2B, 23 - n);

    if (mt == 0x3f && n >= 10) {
        uint8_t ra = g.agch_buf[7];
        uint32_t lr = last_rach_fn();
        uint32_t memo = lr ? lr : g.rach_conf_fn[ra];
        if (memo) {
            uint16_t t1p = (uint16_t)((memo / 1326u) % 32u);
            uint8_t t2 = (uint8_t)(memo % 26u);
            uint8_t t3 = (uint8_t)(memo % 51u);
            g.agch_buf[8] = (uint8_t)((t1p << 3) | ((t3 >> 3) & 7));
            g.agch_buf[9] = (uint8_t)(((t3 & 7) << 5) | (t2 & 0x1f));
        }
    }
    g.agch_valid = true;
    g.agch_tick = g.tick;
}

static void feed_sdcch(const uint8_t *l2, int len, uint32_t fn)
{
    if (fn && fn == g.sdcch_last_fn) {
        return;
    }
    g.sdcch_last_fn = fn;
    if (g.sdcch_tail - g.sdcch_head >= SDCCH_RING_N) {
        static unsigned nlog;
        if (nlog++ < 5) {
            L1_LOG("SDCCH DL : anneau plein, plus ancien bloc abandonne");
        }
        g.sdcch_head++;
    }
    struct sdcch_entry *e = &g.sdcch_ring[g.sdcch_tail % SDCCH_RING_N];
    int n = len < 23 ? len : 23;
    memcpy(e->l2, l2, n);
    memset(e->l2 + n, 0x2B, 23 - n);
    e->fn = fn;
    e->tick = g.tick;
    e->reps = 0;
    g.sdcch_tail++;
    g.sdcch_valid = true;
}

static void feed_sacch(const uint8_t *l2, int len)
{
    int n = len < 23 ? len : 23;
    bool si56 = false;
    for (int i = 2; i + 1 < n && i < 8; i++) {
        if (l2[i] == 0x06 && (l2[i + 1] == 0x1d || l2[i + 1] == 0x1e)) {
            si56 = true;
            break;
        }
    }
    if (!si56) {
        return;
    }
    memcpy(g.sacch_buf, l2, n);
    memset(g.sacch_buf + n, 0x2b, 23 - n);
    g.sacch_have = true;
    g.sacch_real = true;
}

static void feed_l2_23(uint8_t *dst, bool *valid, uint32_t *tick,
                       const uint8_t *l2, int len)
{
    int n = len < 23 ? len : 23;
    memcpy(dst, l2, n);
    memset(dst + n, 0x2B, 23 - n);
    *valid = true;
    *tick = g.tick;
}

static void feed_si(const uint8_t *l2, int len)
{
    int n = len < 23 ? len : 23;
    int slot = -1;
    if (n >= 3 && l2[1] == 0x06 && l2[2] >= 0x19 && l2[2] <= 0x1e) {
        slot = l2[2] - 0x19;
    }
    if (slot >= 0) {
        memcpy(g.si_set[slot], l2, n);
        memset(g.si_set[slot] + n, 0x2B, 23 - n);
        g.si_have[slot] = true;
    }
    if (slot == 2 && n >= 10 && !g.sacch_real) {
        uint8_t *s6 = g.sacch_buf;
        memset(s6, 0x2b, 23);
        s6[0] = 0x00;
        s6[1] = 0x00;
        s6[2] = 0x03;
        s6[3] = 0x03;
        s6[4] = (uint8_t)((11 << 2) | 0x01);
        s6[5] = 0x06;
        s6[6] = 0x1e;
        memcpy(s6 + 7, l2 + 3, 7);
        s6[14] = 0x0f;
        s6[15] = 0xff;
        g.sacch_have = true;
    }
    memcpy(g.si_buf, l2, n);
    memset(g.si_buf + n, 0x2B, 23 - n);
    g.si_valid = true;
}

static void gsmtap_readable(void *opaque)
{
    int fd = (int)(intptr_t)opaque;
    uint8_t buf[512];
    for (;;) {
        ssize_t n = recv(fd, buf, sizeof(buf), MSG_DONTWAIT);
        if (n < GSMTAP_HDR_LEN + 3) {
            if (n < 0) {
                break;
            }
            continue;
        }
        if (buf[2] != GSMTAP_TYPE_UM) {
            continue;
        }
        uint8_t sub = buf[12];
        const uint8_t *l2 = buf + GSMTAP_HDR_LEN;
        int len = (int)n - GSMTAP_HDR_LEN;
        uint8_t mt = l2[2];
        switch (sub) {
        case GSMTAP_CHANNEL_BCCH:
            if (l2[1] == 0x06 && mt >= 0x19 && mt <= 0x1e) {
                feed_si(l2, len);
            }
            break;
        case GSMTAP_CHANNEL_AGCH:
            if (l2[1] == 0x06 && (mt == 0x3f || mt == 0x39 || mt == 0x3a ||
                                  mt == 0x21 || mt == 0x22 || mt == 0x24)) {
                feed_agch(l2, len);
            }
            break;
        case GSMTAP_CHANNEL_SDCCH4:
            feed_sdcch(l2, len, ldl_be_p(buf + 8));
            break;
        case GSMTAP_CHANNEL_SACCH:
            if (len >= 7) {
                feed_sacch(l2, len);
            }
            break;
        case GSMTAP_CHANNEL_TCH_F:
            feed_l2_23(g.facch_dl, &g.facch_dl_valid, &g.facch_dl_tick, l2, len);
            break;
        case GSMTAP_CHANNEL_TCH_ACCH:
            feed_l2_23(g.tsacch_dl, &g.tsacch_dl_valid, &g.tsacch_dl_tick, l2, len);
            break;
        default:
            break;
        }
    }
}

static void sch_readable(void *opaque)
{
    int fd = (int)(intptr_t)opaque;
    uint8_t buf[64];
    for (;;) {
        ssize_t n = recv(fd, buf, sizeof(buf), MSG_DONTWAIT);
        if (n < 0) {
            break;
        }
        if (n < 16 || memcmp(buf, "SCH2", 4) != 0) {
            continue;
        }
        int32_t bsic = (int32_t)ldl_le_p(buf + 4);
        int32_t fn = (int32_t)ldl_le_p(buf + 8);
        int32_t arfcn = (int32_t)ldl_le_p(buf + 12);
        bool first = !g.sb_valid;
        g.sb_bsic = (uint8_t)(bsic & 0x3f);
        g.sb_fn = (uint32_t)fn;
        g.serving_arfcn = arfcn;
        g.sb_valid = true;
        g.sb_capture_fn = calypso_trx_get_fn();
        if (first) {
            L1_LOG("synchro gr-gsm : ARFCN=%d BSIC=%u FN=%u", arfcn, g.sb_bsic, g.sb_fn);
        }
        calypso_trx_autosync_fn((uint32_t)fn);
    }
}

static int udp_listen(uint16_t port, IOHandler *handler)
{
    int fd = socket(AF_INET, SOCK_DGRAM, 0);
    if (fd < 0) {
        return -1;
    }
    int one = 1;
    setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, &one, sizeof(one));
    struct sockaddr_in sa = {
        .sin_family = AF_INET,
        .sin_port = htons(port),
        .sin_addr.s_addr = htonl(INADDR_LOOPBACK),
    };
    if (bind(fd, (struct sockaddr *)&sa, sizeof(sa)) < 0) {
        L1_LOG("bind udp/%u : %s", port, strerror(errno));
        close(fd);
        return -1;
    }
    qemu_set_fd_handler(fd, handler, NULL, (void *)(intptr_t)fd);
    return fd;
}

static void poll_tch_cfg(void)
{
    static int fd = -2;
    static uint32_t last;
    if (fd == -2) {
        fd = open(SHM_TCH_CFG, O_CREAT | O_RDWR, 0644);
    }
    if (fd < 0) {
        return;
    }
    uint8_t b[16];
    if (pread(fd, b, sizeof(b), 0) != (ssize_t)sizeof(b)) {
        return;
    }
    uint32_t seq;
    memcpy(&seq, b, 4);
    if (seq == 0) {
        if (g.tch_cfg_valid) {
            g.tch_cfg_valid = false;
            g.facch_dl_valid = g.tsacch_dl_valid = false;
            L1_LOG("canal TCH libere");
        }
        last = 0;
        return;
    }
    if (seq == last) {
        return;
    }
    last = seq;
    g.tch_cfg_valid = true;
    g.facch_dl_valid = g.tsacch_dl_valid = false;
    L1_LOG("canal TCH : TN=%u TSC=%u ARFCN=%u", b[4], b[5], lduw_le_p(b + 6));
}

static void publish_kc(void)
{
    static int fd = -1;
    static uint32_t seq;
    static uint8_t last[KC_RECLEN];
    static bool have_last;
    static int tick;
    static int clear_pending;
    if (++tick < KC_PUBLISH_EVERY) {
        return;
    }
    tick = 0;

    uint16_t mode = *api_ndb(NDB_D_A5MODE);
    const uint16_t *kw = api_ndb(NDB_A_KC);
    uint8_t rec[KC_RECLEN] = {0};
    bool nul = true;
    for (int i = 0; i < 4; i++) {
        rec[6 + 6 - 2 * i] = (uint8_t)(kw[i] >> 8);
        rec[6 + 7 - 2 * i] = (uint8_t)(kw[i] & 0xFF);
    }
    for (int i = 6; i < 14; i++) {
        if (rec[i]) {
            nul = false;
        }
    }
    uint8_t algo = (mode >= 1 && mode <= 3 && !nul) ? (uint8_t)mode : 0;
    if (!algo) {
        memset(rec + 6, 0, 8);
    }
    rec[4] = algo;
    rec[5] = algo ? 8 : 0;
    rec[14] = 0xFF;
    if (have_last && !memcmp(last + 4, rec + 4, KC_RECLEN - 4)) {
        clear_pending = 0;
        return;
    }
    if (!algo && have_last && last[4]) {
        /* Chiffre -> clair : on attend de voir si la Kc revient (assignation). */
        if (++clear_pending < KC_CLEAR_GRACE) {
            return;
        }
    }
    clear_pending = 0;
    if (fd < 0) {
        fd = open(SHM_KC, O_WRONLY | O_CREAT, 0666);
        if (fd < 0) {
            return;
        }
    }
    seq++;
    memcpy(rec, &seq, 4);
    if (pwrite(fd, rec, sizeof(rec), 0) != (ssize_t)sizeof(rec)) {
        close(fd);
        fd = -1;
        seq--;
        return;
    }
    memcpy(last, rec, sizeof(rec));
    have_last = true;
    if (algo) {
        L1_LOG("chiffrement A5/%u actif (Kc publie, seq=%u)", algo, seq);
    } else {
        L1_LOG("couche 1 en clair (seq=%u)", seq);
    }
}

void calypso_l1_frame_tick(void)
{
    publish_kc();
    tch_dl_poll();
    poll_tch_cfg();

    if (g.sb_valid) {
        dispatch_sb(0);
        dispatch_sb(1);
        bool imm_pending = g.agch_valid &&
            (g.agch_buf[2] == 0x3f || g.agch_buf[2] == 0x3a || g.agch_buf[2] == 0x3b);
        if (g.si_valid && !g.sdcch_valid && !dcch_si_guard() && !g.tch_cfg_valid &&
            !imm_pending) {
            if ((g.si_rot++ & 7u) == 0) {
                rotate_si();
            }
            ndb_block(NDB_A_CD, 0, g.si_set[g.si_rr]);
            tch_serv_demod();
        } else if (dcch_si_guard() && !tch_fresh(g.tsacch_dl_valid, g.tsacch_dl_tick) &&
                   !g.tch_cfg_valid) {
            dcch_sacch_present();
        }
    }

    if (!g.pending) {
        return;
    }
    g.tick++;

    uint8_t page = g.page;
    uint16_t md = g.d_task_md, td = g.d_task_d;
    if (md == PM_DSP_TASK) {
        dispatch_pm(page);
    }
    if (md == SB_DSP_TASK) {
        dispatch_sb(page);
    }
    if (td == ALLC_DSP_TASK) {
        dispatch_allc(page);
    }
    switch (td & 0x7FFF) {
    case TCHT_DSP_TASK:
        dispatch_tch_dl();
        break;
    case TCHA_DSP_TASK:
        dispatch_tch_sacch();
        break;
    default:
        break;
    }
    g.pending = false;
}

void calypso_l1_init(const char *firmware_elf)
{
    memset(&g, 0, sizeof(g));
    g.serving_arfcn = -1;
    if (firmware_elf) {
        g.l1s_addr = elf_symbol(firmware_elf, "l1s");
        g.last_rach_addr = elf_symbol(firmware_elf, "last_rach");
    }
    if (!g.l1s_addr || !g.last_rach_addr) {
        L1_LOG("symboles firmware introuvables (l1s=0x%x last_rach=0x%x) : "
               "fenetres CCCH et req-ref degradees", g.l1s_addr, g.last_rach_addr);
    }
    udp_listen(GSMTAP_PORT, gsmtap_readable);
    udp_listen(SCH_PORT, sch_readable);
    L1_LOG("backend gr-gsm : GSMTAP udp/%d, SCH udp/%d", GSMTAP_PORT, SCH_PORT);
}
