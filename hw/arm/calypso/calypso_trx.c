/* SPDX-License-Identifier: GPL-2.0-or-later */
#include "qemu/osdep.h"
#include "qemu/timer.h"
#include "qemu/main-loop.h"
#include "sysemu/runstate.h"
#include "exec/address-spaces.h"
#include "exec/cpu-common.h"
#include "hw/core/cpu.h"
#include "target/arm/cpu.h"
#include "hw/irq.h"
#include "hw/arm/calypso/calypso_api.h"
#include "hw/arm/calypso/calypso_l1_ops.h"
#include "hw/arm/calypso/calypso_trx.h"
#include "hw/arm/calypso/calypso_inth.h"
/* l1-dsp/ is only linked under --enable-l1-dsp: WEAK symbol, tested before the
 * call, so this file stays common to both L1s (see the l1-dsp island). */
extern void calypso_twl3025_set_afc_dac(int16_t dac_value) __attribute__((weak));
#include "hw/arm/calypso/calypso_uart.h"
#include "hw/arm/calypso/calypso_timer.h"
#include "hw/arm/calypso/calypso_sim.h"
#include "hw/arm/calypso/calypso_dsp_pont.h"
#include <pthread.h>
#include <time.h>
#include <poll.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <sys/socket.h>
#include <sys/un.h>

extern CalypsoUARTState *g_uart_modem;
extern CalypsoUARTState *g_uart_irda;

#define FRAME_IRQ_PULSE_NS     1000000

/* [2026-09-22] PERIODE DU COUP DE POUCE AU CPU -- suspect no 1 du retard.
 *
 * cpu_kick() fait `cpu_exit(first_cpu)` + `qemu_notify_event()` : c'est ce qui
 * rend la main a la boucle principale, donc ce qui permet de SERVIR le timer
 * TDMA. Tant que le CPU ne sort pas, le timer peut bien etre echu, personne ne
 * le regarde. La periode valait 5 ms, soit PLUS qu'une trame GSM (4,615 ms) :
 * la coincidence etait frappante : 5 ms de kick, 5,11 ms mesurees par trame.
 *
 * MESURE (2026-09-22) : HYPOTHESE FAUSSE ELLE AUSSI. Balayage sur un facteur
 * 17 -- 5,00 ms : 195,8 tr/s ; 1,15 ms : 195,9 ; 0,58 ms : 195,8 ;
 * 0,29 ms : 195,9. Parfaitement plat. (Verifie dans /proc/<qemu>/environ que
 * la variable atteignait bien QEMU : elle y etait.)
 *
 * Restent donc, pour expliquer les 5,11 ms : le TRAVAIL lui-meme. Sans
 * pas-a-pas QEMU seul fait 588 tr/s (1,70 ms/trame) ; le pas-a-pas serialise
 * l'ARM et le DSP au lieu de les laisser se recouvrir. Le reste, ~3,4 ms,
 * c'est l'interpreteur C54x qui execute la trame. INSNS n'y change rien (plat
 * de 60000 a 30000) parce que c'est un PLAFOND : la ROM finit son travail et
 * passe en idle bien avant, donc baisser le plafond ne retire aucun travail.
 * La piste suivante est donc la VITESSE de l'interpreteur, pas son budget.
 * CALYPSO_CPU_KICK_NS reste reglable pour mesurer. */
#define CPU_KICK_NS_DEFAUT     5000000    /* mesure : sans effet, on garde l'original */
static int64_t cpu_kick_ns(void)
{
    static int64_t ns;
    if (!ns) {
        const char *e = getenv("CALYPSO_CPU_KICK_NS");
        ns = e ? (int64_t)strtoll(e, NULL, 0) : 0;
        if (ns < 10000 || ns > 100000000) {
            ns = CPU_KICK_NS_DEFAUT;
        }
    }
    return ns;
}
#define CPU_KICK_NS            cpu_kick_ns()

/* [2026-09-22] QUANTUM DE RE-ESSAI DU PAS-A-PAS -- la cause du retard de 9,7 %.
 *
 * Quand l1_sync de l'ARM ou le DSP n'a pas fini, on ne bloque pas : on
 * replanifie le tick TDMA un peu plus tard et on repasse. Le « un peu plus
 * tard » valait GSM_TDMA_NS/16 = 288 us, et /8 = 577 us pour le DSP. Or ce
 * qu'on attend se termine en DIZAINES de microsecondes : on payait donc, a
 * chaque trame, un arrondi pouvant aller jusqu'a 288 us, une a deux fois.
 *
 * MESURE (2026-09-22) : HYPOTHESE FAUSSE. Balayage du quantum sur un facteur
 * 4 -- /16 (288 us) : 195,7 tr/s ; /64 (72 us) : 195,5. Aucun effet. Le
 * quantum n'est PAS la cause du retard de 9,7 %. On garde la valeur d'origine
 * et la molette, qui reste utile pour mesurer.
 * Ce qui suit reste vrai et important : tdma_pacer() rattrape un retard en SAUTANT une trame
 * (`while (target <= now) target += GSM_TDMA_NS`), ce depassement ne se voit
 * nulle part : il se transforme en trames perdues. En aval, osmo-bts-trx ne
 * sait pas suivre une horloge lente (son filtre de derive est un TODO vide,
 * scheduler_trx.c:571), il resynchronise, et pendant ce temps il ne produit
 * rien : des coupures de ~200 trames d'affilee.
 *
 * Le budget d'attente TOTAL est conserve : les plafonds de re-essai sont
 * mis a l'echelle du meme facteur (voir PONT_ATTENTES_MAX).
 * CALYPSO_PONT_RETRY_DIV permet de mesurer d'autres valeurs sans recompiler. */
#define PONT_RETRY_DIV_DEFAUT  16        /* mesure : sans effet, on garde l'original */
static int pont_retry_div(void)
{
    static int div;
    if (!div) {
        const char *e = getenv("CALYPSO_PONT_RETRY_DIV");
        div = e ? atoi(e) : 0;
        if (div < 1 || div > 8192) {
            div = PONT_RETRY_DIV_DEFAUT;
        }
    }
    return div;
}
#define PONT_RETRY_NS      ((int64_t)GSM_TDMA_NS / pont_retry_div())
/* meme budget mural qu'avec 256 essais de GSM_TDMA_NS/16 */
#define PONT_ATTENTES_MAX  (256 * pont_retry_div() / 16)
#define IDLE_PC_LO             0x00823000u
#define IDLE_PC_HI             0x00826000u
#define INTH_MASK_ADDR         0xFFFFFA08u
#define BOOT_POLLS_BEFORE_BOOT 3
#define FN_SYNC_TOLERANCE      16
#define FN_SYNC_WINDOW         5

typedef struct CalypsoTRX {
    qemu_irq *irqs;
    MemoryRegion api_iomem;
    uint16_t *api_ram;          /* g_new0, or mmap of the bridge shared segment */
    /* EXTERNAL DSP (CALYPSO_DSP_EXTERN=<socket>|1): layer 1 is a real C54x
     * running inside c54x_exe instead of a gr-gsm shunt. API RAM is shared
     * through /dev/shm and the frame is locked by TICK/DONE (see
     * include/hw/arm/calypso/calypso_dsp_pont.h). */
    bool pont;
    bool pont_pending;          /* a TICK was sent, DONE not collected yet */
    int pont_fd;
    QEMUTimer *pont_boot_timer; /* clocks the DSP alone, before the firmware starts the TDMA */
    unsigned pont_timeouts;
    uint64_t pont_frames;
    uint64_t pont_api_irqs;
    uint8_t dsp_page;
    bool bl_booted;
    unsigned bl_polls;
    MemoryRegion tpu_iomem;
    MemoryRegion tpu_ram_iomem;
    uint16_t tpu_regs[CALYPSO_TPU_SIZE / 2];
    uint16_t tpu_ram[CALYPSO_TPU_RAM_SIZE / 2];
    MemoryRegion tsp_iomem;
    uint16_t tsp_regs[CALYPSO_TSP_SIZE / 2];
    MemoryRegion ulpd_iomem;
    uint16_t ulpd_regs[CALYPSO_ULPD_SIZE / 2];
    uint32_t ulpd_counter;
    MemoryRegion sim_iomem;
    CalypsoSim *sim;
    QEMUTimer *tdma_timer;
    QEMUTimer *frame_irq_timer;
    QEMUTimer *kick_timer;
    uint32_t fn;
    int64_t fn_offset;
    int64_t fn_offset_hist[FN_SYNC_WINDOW];
    unsigned fn_offset_n;
    bool fn_synced;
    bool tdma_running;
    /* [2026-09-21] ARM FIRST (lock-step): the TICK of frame N is handed to the
     * DSP only once the firmware's l1_sync(N) is over (INTH end of service of
     * the frame IRQ). The TICK parameters are latched when the frame IRQ is
     * raised: DSP_EN and d_dsp_page then still describe the scenario the ARM
     * ended in frame N-1, i.e. the one the DSP must run in frame N.
     * [2026-09-21] Two phases: TICK before the ARM IRQ (the ROM's ISR reads
     * the page of N-1 and writes the R page), PONT_GO after l1_sync(N). */
    int      tick_phase;        /* 0: frame start, 1: waiting for phase A + the ARM's l1_sync */
    bool     deux_phases;       /* set while pont_echange() must flag the TICK */
    bool     phase_a_recue;     /* DONE|PHASE_A of this frame collected */
    bool     go_inutile;        /* single-phase DSP answered a final DONE already */
    uint64_t eoi_cible;         /* frame_eoi to reach before the TICK goes out */
    unsigned eoi_attentes, eoi_timeouts;
    uint8_t burst_ring[8];
    unsigned burst_w, burst_r;
    uint16_t burst_cur;
    uint32_t burst_last_fn;
    /* [2026-09-21] Reference de requete de l'IMMEDIATE ASSIGNMENT, cf.
     * pont_reqref_corrigee() : la RA de la derniere ecriture de d_rach, et la
     * trame que le firmware a memorisee pour chacune (son last_rach, relevee
     * une fois par trame). */
    uint8_t  rach_ra;
    bool     rach_ra_vue;
    uint32_t rach_fn[256];
    uint8_t  rach_file[8];       /* RA ecrites, en attente de leur RACH_CONF */
    unsigned rach_file_w, rach_file_r;
    /* Canal dedie a annoncer au DSP au prochain TICK (PONT_DCCH). */
    bool     dcch_a_dire;
    uint8_t  dcch_tn, dcch_ss;
    int      dcch_genre;
} CalypsoTRX;

static CalypsoTRX *g_trx;
static volatile uint32_t g_wall_fn;
static volatile bool g_wall_running;
static pthread_t g_wall_thread;

uint16_t *calypso_api_ram(void)
{
    return g_trx->api_ram;
}

uint32_t calypso_trx_get_fn(void)
{
    if (!g_trx) {
        return 0;
    }
    int64_t fn = ((int64_t)g_trx->fn + g_trx->fn_offset) % (int64_t)GSM_HYPERFRAME;
    if (fn < 0) {
        fn += GSM_HYPERFRAME;
    }
    return (uint32_t)fn;
}

void calypso_trx_autosync_fn(uint32_t sch_fn)
{
    if (!g_trx) {
        return;
    }
    int64_t raw = (int64_t)sch_fn - (int64_t)g_trx->fn;

    /* gr-gsm/UDP/QEMU-loop latency can only delay the local fn read, hence
     * inflate fn and shrink the measured offset. The maximum over the last few
     * SCHs is the value closest to the truth. */
    g_trx->fn_offset_hist[g_trx->fn_offset_n % FN_SYNC_WINDOW] = raw;
    g_trx->fn_offset_n++;
    unsigned n = g_trx->fn_offset_n < FN_SYNC_WINDOW ? g_trx->fn_offset_n : FN_SYNC_WINDOW;
    int64_t offset = g_trx->fn_offset_hist[0];
    for (unsigned i = 1; i < n; i++) {
        if (g_trx->fn_offset_hist[i] > offset) {
            offset = g_trx->fn_offset_hist[i];
        }
    }

    int64_t drift = offset - g_trx->fn_offset;
    if (drift < 0) {
        drift = -drift;
    }
    if (g_trx->fn_synced && drift <= FN_SYNC_TOLERANCE) {
        return;
    }
    g_trx->fn_offset = offset;
    g_trx->fn_synced = true;
    fprintf(stderr, "[trx] horloge FN calee sur le SCH gr-gsm : fn=%u offset=%lld (brut=%lld)\n",
            sch_fn, (long long)offset, (long long)raw);
}

/* ── external DSP bridge ──────────────────────────────────────────────── */
static bool pont_send(CalypsoTRX *s, uint32_t type, uint32_t a, uint32_t b, uint32_t c)
{
    CalypsoPontMsg m = { type, a, b, c };
    if (s->pont_fd < 0) {
        return false;
    }
    return send(s->pont_fd, &m, sizeof(m), MSG_NOSIGNAL) == (ssize_t)sizeof(m);
}

static bool pont_recv(CalypsoTRX *s, CalypsoPontMsg *m, int timeout_ms)
{
    struct pollfd pfd = { .fd = s->pont_fd, .events = POLLIN };
    int rc = poll(&pfd, 1, timeout_ms);
    if (rc <= 0) {
        return false;
    }
    ssize_t n = recv(s->pont_fd, m, sizeof(*m), 0);
    if (n != (ssize_t)sizeof(*m)) {
        fprintf(stderr, "[trx] pont DSP : connexion perdue (recv=%zd)\n", n);
        close(s->pont_fd);
        s->pont_fd = -1;
        return false;
    }
    return true;
}

static void pont_connect(CalypsoTRX *s, const char *spec)
{
    const char *sock = (strcmp(spec, "1") == 0) ? CALYPSO_PONT_SOCK : spec;
    int shm = shm_open(CALYPSO_PONT_SHM, O_RDWR, 0);
    if (shm < 0) {
        fprintf(stderr, "[trx] pont DSP : shm_open(%s) : %s - lancez d'abord "
                "c54x_exe --arm\n", CALYPSO_PONT_SHM, strerror(errno));
        exit(1);
    }
    struct stat st;
    if (fstat(shm, &st) < 0 || st.st_size < CALYPSO_PONT_SHM_BYTES) {
        fprintf(stderr, "[trx] pont DSP : segment %s trop petit (%lld < %d)\n",
                CALYPSO_PONT_SHM, (long long)st.st_size, CALYPSO_PONT_SHM_BYTES);
        exit(1);
    }
    /* 64 KB page-aligned ARM window; the first 16 KB are the shared segment
     * (the DSP API window), the rest is private and empty. */
    void *base = NULL;
    if (posix_memalign(&base, 4096, CALYPSO_API_SIZE) != 0) {
        fprintf(stderr, "[trx] pont DSP : posix_memalign\n");
        exit(1);
    }
    memset(base, 0, CALYPSO_API_SIZE);
    void *map = mmap(base, CALYPSO_PONT_SHM_BYTES, PROT_READ | PROT_WRITE,
                     MAP_SHARED | MAP_FIXED, shm, 0);
    close(shm);
    if (map == MAP_FAILED || map != base) {
        fprintf(stderr, "[trx] pont DSP : mmap : %s\n", strerror(errno));
        exit(1);
    }
    s->api_ram = base;

    s->pont_fd = socket(AF_UNIX, SOCK_SEQPACKET, 0);
    struct sockaddr_un sa = { .sun_family = AF_UNIX };
    snprintf(sa.sun_path, sizeof(sa.sun_path), "%s", sock);
    if (s->pont_fd < 0 || connect(s->pont_fd, (struct sockaddr *)&sa, sizeof(sa)) < 0) {
        fprintf(stderr, "[trx] pont DSP : connect(%s) : %s - lancez d'abord "
                "c54x_exe --arm\n", sock, strerror(errno));
        exit(1);
    }
    CalypsoPontMsg m;
    if (!pont_send(s, PONT_HELLO, CALYPSO_API_WORDS, CALYPSO_PONT_MAGIC, 0) ||
        !pont_recv(s, &m, 2000) || m.type != PONT_HELLO_OK ||
        m.a != CALYPSO_API_WORDS || m.b != CALYPSO_PONT_MAGIC) {
        fprintf(stderr, "[trx] pont DSP : poignee de main refusee par %s\n", sock);
        exit(1);
    }
    s->pont = true;
    fprintf(stderr, "[trx] pont DSP : API RAM partagee (%s) + socket %s - le C54x "
            "est dans c54x_exe, la couche 1 gr-gsm est desactivee\n",
            CALYPSO_PONT_SHM, sock);
    calypso_l1_disable("DSP externe");
}

/* ── Reference de requete de l'IMMEDIATE ASSIGNMENT (montage DSP) ────────
 *
 * Le mobile n'accepte un IMM ASS que si sa reference de requete correspond
 * EXACTEMENT a ce que SA couche 1 lui a confirme : gsm48_match_ra()
 * (osmocom-bb gsm48_rr.c:3359) compare la RA *et* T1'/T2/T3, et journalise
 * sinon « request %02x matches but not frame number ».
 *
 * Or le banc n'emet pas l'access-burst a la trame ou le firmware a cru
 * l'emettre : pont.py le programme sur SA propre horloge (pont/uplink.py,
 * _poll_rach -> _next_fn(4, ...)), plusieurs trames plus tard. La BTS
 * horodate donc la reference avec une autre trame, et le mobile jette
 * l'assignation. Mesure du 2026-09-21 sur le banc reel : RACH publie
 * ra=0x0d, IMM ASS revenue avec la meme RA mais T1'=3 T2=9 T3=35, et le
 * mobile est reste en « connection pending », sans jamais poser de tache
 * SDCCH montante (/dev/shm/calypso_sdcch_ul jamais cree) ; cote BSC, douze
 * lchan SDCCH ouverts puis « lchan allocation failed ... Timeout ».
 *
 * La couche 1 gr-gsm reglait deja exactement ca en reecrivant les octets 8-9
 * du bloc (l1-grgsm/calypso_l1_grgsm.c:836-845, feed_agch). Avec un DSP
 * externe elle est desactivee : on refait la meme chose ici, sur la lecture
 * ARM du mot concerne de a_cd, a partir de « last_rach » du firmware — la
 * trame que le firmware a lui-meme memorisee et remontee en L1CTL_RACH_CONF.
 *
 * On ne corrige que si la RA de l'assignation est celle de la derniere
 * ecriture de d_rach : sinon l'assignation repond a une tentative plus
 * ancienne que last_rach, et fabriquer une correspondance serait pire que
 * l'echec. MONTANT_REQREF=0 desactive la correction.
 */
static uint32_t pont_last_rach_fn(void)
{
    static uint32_t adr;
    static int cherche;
    if (!cherche) {
        cherche = 1;
        adr = calypso_firmware_symbol("last_rach");
        fprintf(stderr, "[trx] reference de requete : last_rach %s (%s)\n",
                adr ? "trouve" : "INTROUVABLE",
                calypso_firmware_elf() ? calypso_firmware_elf() : "pas d'ELF");
    }
    if (!adr) {
        return 0;
    }
    uint32_t fn = 0;
    cpu_physical_memory_read(adr, &fn, sizeof(fn));
    return le32_to_cpu(fn);
}

/* Appariement RA -> trame d'emission.
 *
 * [2026-09-21] Premiere version : relever last_rach.fn une fois par trame et
 * l'attribuer a la derniere RA ecrite. Mesure sur le banc : trois « IMM ASS
 * ra=0x0c : aucune trame memorisee » d'affilee. En rafale, le firmware ecrit
 * le d_rach de la tentative suivante AVANT que last_rach n'ait bouge pour la
 * precedente : la trame partait alors dans la mauvaise case et la RA d'avant
 * n'en avait aucune.
 *
 * Deuxieme version, exacte : les RA sont mises en file a l'ecriture de
 * d_rach, et chaque L1CTL_RACH_CONF (vu par calypso_dcch_tap.c dans le flux
 * sercomm) en depile une. C'est le meme appariement que fait le mobile, qui
 * range cr_ra et le fn du RACH_CONF ensemble dans cr_hist -- et c'est
 * exactement la valeur que gsm48_match_ra() comparera. */
/* calypso_dcch_tap.c vient d'apprendre le canal dedie du mobile. On ne
 * l'envoie pas tout de suite : le protocole du pont est synchrone, et un
 * message glisse pendant l'attente d'un PONT_GO serait ignore par le DSP.
 * Il part juste avant le prochain TICK. */
void calypso_trx_dcch(int genre, int ss, int tn)
{
    CalypsoTRX *s = g_trx;
    if (!s || !s->pont) {
        return;
    }
    s->dcch_genre = genre;
    s->dcch_ss = (uint8_t)ss;
    s->dcch_tn = (uint8_t)tn;
    s->dcch_a_dire = true;
}

void calypso_trx_rach_conf(uint32_t fn)
{
    CalypsoTRX *s = g_trx;
    if (!s || !fn) {
        return;
    }
    if (s->rach_file_r == s->rach_file_w) {
        static unsigned n;
        if (n++ < 5) {
            fprintf(stderr, "[trx] RACH_CONF fn=%u sans RA en attente\n", fn);
        }
        return;
    }
    uint8_t ra = s->rach_file[s->rach_file_r++ % ARRAY_SIZE(s->rach_file)];
    s->rach_fn[ra] = fn;
    s->rach_ra = ra;
    s->rach_ra_vue = true;
}

static bool pont_reqref_corrigee(CalypsoTRX *s, hwaddr woff, uint16_t *out)
{
    static int actif = -1;
    if (actif < 0) {
        const char *e = getenv("MONTANT_REQREF");
        actif = (e && *e == '0') ? 0 : 1;
    }
    if (!actif || woff != API_NDB + NDB_A_CD + 14) {
        return false;
    }

    /* a_cd : mot 0 = etat, les 23 octets L2 commencent au mot 3 (octet +6).
     * L3 = [pseudo-longueur, PD, type, ...] -> 06 3f = IMMEDIATE ASSIGNMENT,
     * la RA est l'octet 7 et la reference de requete les octets 8-9. */
    const uint8_t *d = (const uint8_t *)s->api_ram + API_NDB + NDB_A_CD + 6;
    if (d[1] != 0x06 || d[2] != 0x3f) {
        return false;
    }
    uint32_t memo = s->rach_fn[d[7]];
    if (!memo && s->rach_ra_vue && d[7] == s->rach_ra) {
        memo = pont_last_rach_fn();
    }
    if (!memo) {
        static unsigned n;
        if (n++ < 5) {
            fprintf(stderr, "[trx] IMM ASS ra=0x%02x : aucune trame memorisee "
                    "pour cette RA - reference laissee telle quelle\n", d[7]);
        }
        return false;
    }

    uint16_t t1p = (uint16_t)((memo / 1326u) % 32u);
    uint8_t t2 = (uint8_t)(memo % 26u);
    uint8_t t3 = (uint8_t)(memo % 51u);
    uint8_t o8 = (uint8_t)((t1p << 3) | ((t3 >> 3) & 7));
    uint8_t o9 = (uint8_t)(((t3 & 7) << 5) | (t2 & 0x1f));
    *out = (uint16_t)(o8 | (o9 << 8));

    if (*out != (uint16_t)(d[8] | (d[9] << 8))) {
        static unsigned n;
        if (n++ < 20) {
            uint8_t at1 = (d[8] >> 3) & 0x1f;
            uint8_t at3 = (uint8_t)(((d[8] & 7) << 3) | ((d[9] >> 5) & 7));
            uint8_t at2 = d[9] & 0x1f;
            fprintf(stderr, "[trx] IMM ASS ra=0x%02x : reference %u/%u/%u de la BTS "
                    "-> %u/%u/%u (last_rach fn=%u)\n",
                    d[7], at1, at2, at3, t1p, t2, t3, memo);
        }
    }
    return true;
}

static uint64_t api_read(void *opaque, hwaddr off, unsigned size)
{
    CalypsoTRX *s = opaque;
    if (off >= CALYPSO_API_SIZE) {
        return 0;
    }
    const uint16_t *src = &s->api_ram[off / 2];
    uint64_t val = (size == 2) ? src[0] :
                   (size == 4) ? ((uint32_t)src[0] | ((uint32_t)src[1] << 16)) :
                   ((const uint8_t *)src)[off & 1];
    if (s->pont && (size == 2 || size == 4)) {
        uint16_t fix;
        if (size == 2) {
            if (pont_reqref_corrigee(s, off, &fix)) {
                val = fix;
            }
        } else {
            if (pont_reqref_corrigee(s, off, &fix)) {
                val = (val & 0xFFFF0000u) | fix;
            }
            if (pont_reqref_corrigee(s, off + 2, &fix)) {
                val = (val & 0x0000FFFFu) | ((uint32_t)fix << 16);
            }
        }
    }
    if (size != 2) {
        return val;
    }

    if (s->pont) {
        /* The external C54x writes d_task_d / d_burst_d into the read page
         * itself, and its ROM bootloader (parked at 0xb41c) answers the
         * BL_CMD_STATUS commands itself. The emulation below, written for the
         * DSP-less shunt, would force IDLE after three reads, BEFORE the DSP
         * has seen the command, making it miss the startup COPY_BLOCK. */
        return val;
    }
    uint16_t rv;
    if (calypso_l1_do_api_read_override((uint32_t)off, &rv)) {
        val = rv;
    }
    if (off == API_R_PAGE(0) + RP_D_TASK_D || off == API_R_PAGE(1) + RP_D_TASK_D) {
        s->burst_cur = s->burst_ring[s->burst_r++ & 7u];
        if (val == 0 && calypso_l1_do_si_valid()) {
            val = ALLC_DSP_TASK;
        }
    }
    if (off == API_R_PAGE(0) + RP_D_BURST_D || off == API_R_PAGE(1) + RP_D_BURST_D) {
        val = (uint16_t)((s->burst_cur + 3) & 3);
    }
    if (off == API_BL_STATUS && !s->bl_booted && ++s->bl_polls > BOOT_POLLS_BEFORE_BOOT) {
        s->api_ram[API_BL_STATUS / 2] = BL_STATUS_BOOT;
        s->api_ram[API_VERSION / 2] = API_VERSION_VALUE;
        s->api_ram[API_VERSION2 / 2] = 0;
        s->bl_booted = true;
        val = BL_STATUS_BOOT;
    }
    return val;
}

static void api_write(void *opaque, hwaddr off, uint64_t value, unsigned size)
{
    CalypsoTRX *s = opaque;
    if (off >= CALYPSO_API_SIZE) {
        return;
    }
    if (size == 2) {
        s->api_ram[off / 2] = (uint16_t)value;
    } else if (size == 4) {
        s->api_ram[off / 2] = (uint16_t)value;
        s->api_ram[off / 2 + 1] = (uint16_t)(value >> 16);
    } else {
        ((uint8_t *)s->api_ram)[off] = (uint8_t)value;
    }
    /* [2026-09-17] AFC RELAY. The firmware's afc_load_dsp() writes
     * dsp_api.db_w->d_afc (word 15 of the W page: byte 0x001E on page 0,
     * 0x0046 on page 1). On silicon the DSP serialises it to the TWL3025 over
     * the TSP; here this hook is the only relay. Without it the sample
     * rotation never moves, the frequency error seen by the FB detector never
     * converges, the firmware never reaches the SB threshold (800 Hz) and
     * replays FB forever. Measured on a deterministic replay: df ~ thousands
     * of Hz without the relay, below 100 Hz with it, and the 224 SB attempts
     * do take place. */
    if ((off == 0x001E || off == 0x0046) && size == 2 && calypso_twl3025_set_afc_dac) {
        calypso_twl3025_set_afc_dac((int16_t)(uint16_t)value);
    }

    /* The whole window, for the L1s that decode it (the C54x). Order matters:
     * after the store above and before the named hooks below, so an L1 that
     * reads API RAM back from this callback sees the value just written. */
    if (size == 2) {
        calypso_l1_do_api_write_observed((uint32_t)off, (uint16_t)value, size);
    }

    if (size == 2 && (off == API_W_PAGE(0) + WP_D_BURST_D ||
                      off == API_W_PAGE(1) + WP_D_BURST_D)) {
        uint32_t wfn = calypso_l1_do_l1s_fn();
        if (wfn != s->burst_last_fn + 1) {
            s->burst_w = s->burst_r = 0;
        }
        s->burst_last_fn = wfn;
        s->burst_ring[s->burst_w++ & 7u] = (uint8_t)(value & 3);
        calypso_l1_do_burst_written((uint16_t)value);
    }
    if (off == API_NDB + NDB_D_RACH && value != 0 && (size == 2 || size == 4)) {
        /* prim_rach.c:72 ecrit (uic|bsic)<<2 | ra<<8 juste avant de poser
         * d_task_ra. On retient la RA : pont_reqref_corrigee() s'en sert pour
         * ne corriger que l'assignation qui repond a CETTE tentative. */
        uint8_t ra = (uint8_t)((value >> 8) & 0xff);
        if (s->rach_file_w - s->rach_file_r >= ARRAY_SIZE(s->rach_file)) {
            s->rach_file_r++;   /* file pleine : la plus ancienne degage */
        }
        s->rach_file[s->rach_file_w++ % ARRAY_SIZE(s->rach_file)] = ra;
        calypso_l1_do_rach_written((uint16_t)value, s->fn);
    }
    if (off == API_NDB + NDB_D_DSP_PAGE && size == 2) {
        s->dsp_page = value & 1;
        calypso_l1_do_page_written((uint16_t)value);
    }
    if (off == API_BL_STATUS) {
        if (value == 0 && !s->pont) {
            s->bl_booted = false;
            s->bl_polls = 0;
        } else if (value == BL_STATUS_READY) {
            /* With an external DSP this value is in fact BL_CMD_COPY_BLOCK
             * (2): the real bootloader consumes it, so leave API RAM alone. */
            if (!s->pont) {
                s->api_ram[API_VERSION / 2] = API_VERSION_VALUE;
                s->api_ram[API_VERSION2 / 2] = 0;
            }
            uint16_t mask;
            cpu_physical_memory_read(INTH_MASK_ADDR, &mask, 2);
            mask &= ~(1 << CALYPSO_IRQ_API);
            cpu_physical_memory_write(INTH_MASK_ADDR, &mask, 2);
            if (!s->pont) {
                s->api_ram[API_NDB / 2] = 0;
            }
        }
    }
}

void calypso_trx_dsp_reset_line(bool active)
{
    static bool precedent;
    CalypsoTRX *s = g_trx;
    if (s && s->pont && precedent && !active) {
        /* Reset release: the boot ROM restarts at 0xff80, posts IDLE and
         * parks at 0xb41c, ready for the ARM COPY_BLOCKs. This is the ONLY
         * point where the C54x is reset - never on a bootloader command, which
         * would destroy the pending command. */
        pont_send(s, PONT_RESET, 0, 0, 0);
        fprintf(stderr, "[trx] pont DSP : RESET_DSP relache par le firmware -> "
                "PONT_RESET (fn=%u)\n", s->fn);
    }
    precedent = active;
}

static const MemoryRegionOps api_ops = {
    .read = api_read,
    .write = api_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = { .min_access_size = 1, .max_access_size = 4 },
    .impl = { .min_access_size = 1, .max_access_size = 4 },
};

static void tpu_done(CalypsoTRX *s)
{
    s->tpu_regs[TPU_CTRL / 2] &= ~TPU_CTRL_EN;
    calypso_tpu_run_scenario(s->tpu_ram, s->fn, s->tpu_regs);
    qemu_irq_raise(s->irqs[CALYPSO_IRQ_API]);
}

static void tdma_tick(void *opaque);

/* Externally imposed beat, at a given frame number: the clock master of the
 * C54x L1, where the DSP TINT0 timer drives the TDMA rather than the platform
 * wall clock. L1 -> platform direction, hence a direct symbol and not a vtable
 * entry (see calypso_l1_ops.h): only one implementation is possible. */
void calypso_trx_force_tick(uint32_t fn)
{
    if (!g_trx) {
        return;
    }
    g_trx->fn = fn;
    tdma_tick(g_trx);
}

static void tdma_start(CalypsoTRX *s);

static uint64_t tpu_read(void *o, hwaddr off, unsigned sz)
{
    CalypsoTRX *s = o;
    if (off == TPU_IT_DSP_PG) {
        return s->dsp_page;
    }
    return (off / 2 < CALYPSO_TPU_SIZE / 2) ? s->tpu_regs[off / 2] : 0;
}

static void tpu_write(void *o, hwaddr off, uint64_t val, unsigned sz)
{
    CalypsoTRX *s = o;
    if (off / 2 < CALYPSO_TPU_SIZE / 2) {
        s->tpu_regs[off / 2] = val;
    }
    if (off == TPU_CTRL && (val & TPU_CTRL_EN)) {
        s->tpu_regs[TPU_CTRL / 2] &= ~(TPU_CTRL_EN | TPU_CTRL_IDLE);
        tpu_done(s);
    }
    if (off == TPU_INT_CTRL && !(val & ICTRL_MCU_FRAME) && !s->tdma_running) {
        tdma_start(s);
    }
    if (off == TPU_IT_DSP_PG) {
        s->dsp_page = val & 1;
    }
}

static const MemoryRegionOps tpu_ops = {
    .read = tpu_read,
    .write = tpu_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = { .min_access_size = 1, .max_access_size = 4 },
    .impl = { .min_access_size = 1, .max_access_size = 4 },
};

static uint64_t tpu_ram_read(void *o, hwaddr off, unsigned sz)
{
    CalypsoTRX *s = o;
    return (off / 2 < CALYPSO_TPU_RAM_SIZE / 2) ? s->tpu_ram[off / 2] : 0;
}

static void tpu_ram_write(void *o, hwaddr off, uint64_t v, unsigned sz)
{
    CalypsoTRX *s = o;
    if (off / 2 < CALYPSO_TPU_RAM_SIZE / 2) {
        s->tpu_ram[off / 2] = v;
    }
}

static const MemoryRegionOps tpu_ram_ops = {
    .read = tpu_ram_read,
    .write = tpu_ram_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = { .min_access_size = 1, .max_access_size = 4 },
    .impl = { .min_access_size = 1, .max_access_size = 4 },
};

static uint64_t tsp_read(void *o, hwaddr off, unsigned sz)
{
    CalypsoTRX *s = o;
    if (off == TSP_RX_REG) {
        return 0xFFFF;
    }
    return (off / 2 < CALYPSO_TSP_SIZE / 2) ? s->tsp_regs[off / 2] : 0;
}

static void tsp_write(void *o, hwaddr off, uint64_t v, unsigned sz)
{
    CalypsoTRX *s = o;
    if (off / 2 < CALYPSO_TSP_SIZE / 2) {
        s->tsp_regs[off / 2] = v;
    }
}

static const MemoryRegionOps tsp_ops = {
    .read = tsp_read,
    .write = tsp_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = { .min_access_size = 1, .max_access_size = 4 },
    .impl = { .min_access_size = 1, .max_access_size = 4 },
};

static uint64_t ulpd_read(void *o, hwaddr off, unsigned sz)
{
    CalypsoTRX *s = o;
    if (off >= 0x20 && off <= 0x40) {
        return 0;
    }
    switch (off) {
    case ULPD_SETUP_CLK13:
        return 0x2003;
    case ULPD_COUNTER_HI:
        s->ulpd_counter += 100;
        return (s->ulpd_counter >> 16) & 0xFFFF;
    case ULPD_COUNTER_LO:
        return s->ulpd_counter & 0xFFFF;
    case ULPD_GAUGING_CTRL:
        return 1;
    case ULPD_GSM_TIMER:
        return s->fn & 0xFFFF;
    default:
        return (off / 2 < CALYPSO_ULPD_SIZE / 2) ? s->ulpd_regs[off / 2] : 0;
    }
}

static void ulpd_write(void *o, hwaddr off, uint64_t v, unsigned sz)
{
    CalypsoTRX *s = o;
    if (off >= 0x20 && off <= 0x40) {
        return;
    }
    if (off / 2 < CALYPSO_ULPD_SIZE / 2) {
        s->ulpd_regs[off / 2] = v;
    }
}

static const MemoryRegionOps ulpd_ops = {
    .read = ulpd_read,
    .write = ulpd_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = { .min_access_size = 1, .max_access_size = 2 },
    .impl = { .min_access_size = 1, .max_access_size = 2 },
};

static uint64_t sim_read(void *o, hwaddr off, unsigned sz)
{
    CalypsoTRX *s = o;
    return calypso_sim_reg_read(s->sim, off);
}

static void sim_write(void *o, hwaddr off, uint64_t v, unsigned sz)
{
    CalypsoTRX *s = o;
    calypso_sim_reg_write(s->sim, off, (uint16_t)v);
}

static const MemoryRegionOps sim_ops = {
    .read = sim_read,
    .write = sim_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = { .min_access_size = 1, .max_access_size = 4 },
    .impl = { .min_access_size = 1, .max_access_size = 4 },
};

static void cpu_idle_park(void)
{
    CPUState *cs = first_cpu;
    if (!cs) {
        return;
    }
    uint64_t pc = (cs->cc && cs->cc->get_pc) ? cs->cc->get_pc(cs) : 0;
    if (pc < IDLE_PC_LO || pc >= IDLE_PC_HI) {
        return;
    }
    cs->halted = 1;
    cpu_exit(cs);
}

static void frame_irq_lower(void *o)
{
    CalypsoTRX *s = o;
    qemu_irq_lower(s->irqs[CALYPSO_IRQ_TPU_FRAME]);
    calypso_l1_do_frame_tick();
    cpu_idle_park();
}

static void *wall_clock_loop(void *arg)
{
    struct timespec next;
    clock_gettime(CLOCK_MONOTONIC, &next);
    while (g_wall_running) {
        next.tv_nsec += GSM_TDMA_NS;
        while (next.tv_nsec >= 1000000000L) {
            next.tv_nsec -= 1000000000L;
            next.tv_sec += 1;
        }
        int rc = clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next, NULL);
        if (rc != 0 && rc != EINTR) {
            break;
        }
        __atomic_add_fetch(&g_wall_fn, 1, __ATOMIC_RELEASE);
    }
    return NULL;
}

/* TICK.b: d_dsp_page plus the one-shot DSP frame interrupt bit, consumed here. */
static uint32_t pont_tick_b(CalypsoTRX *s)
{
    uint32_t b = s->dsp_page;
    bool arme = (s->tpu_regs[TPU_CTRL / 2] & TPU_CTRL_DSP_EN) &&
                !(s->tpu_regs[TPU_INT_CTRL / 2] & ICTRL_DSP_FRAME);
    if (arme) {
        b |= CALYPSO_PONT_TICK_IRQ_TRAME;
        s->tpu_regs[TPU_CTRL / 2] &= (uint16_t)~TPU_CTRL_DSP_EN;
    }
    return b;
}

/* One TICK/DONE exchange with the external DSP: collect the previous frame's
 * DONE without blocking, then send this frame's TICK if the DSP is idle. */
static void pont_echange(CalypsoTRX *s)
{
    if (s->pont_fd < 0) {
        return;
    }
    bool libre = !s->pont_pending;
    if (s->pont_pending) {
        CalypsoPontMsg m;
        if (pont_recv(s, &m, 0) && m.type == PONT_DONE) {
            s->pont_pending = false;
            libre = true;
            s->pont_frames++;
            if (m.a & PONT_DONE_API_IRQ) {
                s->pont_api_irqs++;
                qemu_irq_raise(s->irqs[CALYPSO_IRQ_API]);
            }
        } else if (s->pont_fd >= 0) {
            if (++s->pont_timeouts == 1 || (s->pont_timeouts % 2170) == 0) {
                fprintf(stderr, "[trx] pont DSP : DSP en retard, tick saute "
                        "(fn=%u, %u sauts, %" PRIu64 " trames jouees)\n",
                        s->fn, s->pont_timeouts, s->pont_frames);
            }
        }
    }
    /* [2026-09-20] DSP FRAME INTERRUPT = TPU_CTRL_DSP_EN, ONE-SHOT. The
     * firmware raises it in dsp_end_scenario() (tpu_dsp_frameirq_enable(),
     * set again on EVERY scenario) and never lowers it; the ROM never clears
     * d_task_md nor d_dsp_page, and l1_sync() only flips the write page on
     * frames carrying a DSP item. So a frame interrupt on every tick made the
     * ROM re-read the SAME page for 10+ frames: the FB task restarted at each
     * FCCH and the SB job never got the channel (0xaba4 dispatched, 0xb219
     * never reached, no 764-byte window). Bit 16 of TICK.b says whether the
     * ARM armed the interrupt since the last tick; DSP_EN is consumed here.
     * ICTRL_DSP_FRAME is active-low (tpu_frame_irq_en). */
    uint32_t b = s->dsp_page;
    if (libre) {
        b = pont_tick_b(s);
        if (s->deux_phases) {
            b |= CALYPSO_PONT_TICK_DEUX_PHASES;
        }
    }
    if (libre && s->dcch_a_dire) {
        s->dcch_a_dire = false;
        pont_send(s, PONT_DCCH, s->dcch_tn, (uint32_t)s->dcch_genre, s->dcch_ss);
    }
    if (libre && pont_send(s, PONT_TICK, s->fn, b, s->tpu_regs[TPU_OFFSET / 2])) {
        s->pont_pending = true;
    }
}

static bool arm_first(void)
{
    static int v = -1;
    if (v < 0) {
        const char *ls = getenv("CALYPSO_PONT_LOCKSTEP");
        const char *e = getenv("CALYPSO_PONT_ARM_FIRST");
        v = (ls && *ls == '1' && !(e && *e == '0')) ? 1 : 0;
    }
    return v;
}

/* Before the firmware enables the TPU (which starts tdma_tick) the external
 * DSP must already run: dsp_power_on() waits for its bootloader. This timer
 * does ONLY the DSP exchange - no TPU-frame IRQ, no sequencer, since the ARM
 * has not installed its vectors yet. It stops by itself once the real tick
 * takes over. */
static void pont_boot_tick(void *opaque)
{
    CalypsoTRX *s = opaque;
    if (s->tdma_running) {
        fprintf(stderr, "[trx] pont DSP : le TDMA du firmware prend le relais du "
                "timer de boot (%" PRIu64 " trames de boot)\n", s->pont_frames);
        return;
    }
    if (runstate_is_running()) {
        pont_echange(s);
    }
    timer_mod_ns(s->pont_boot_timer, qemu_clock_get_ns(QEMU_CLOCK_REALTIME) + GSM_TDMA_NS);
}

static void tdma_pacer(CalypsoTRX *s);

static void tdma_tick(void *opaque)
{
    CalypsoTRX *s = opaque;
    if (!runstate_is_running()) {
        timer_mod_ns(s->tdma_timer, qemu_clock_get_ns(QEMU_CLOCK_REALTIME) + GSM_TDMA_NS);
        return;
    }
    /* [2026-09-21] ARM FIRST, phase 1: the frame IRQ of frame fn went to the
     * ARM; hand the frame to the DSP once l1_sync(fn) is over. Measured before
     * this: the DSP wrote the R page of burst N while the ARM was still
     * reading burst N-2 from it (prim_rx_nb.c "BURST ID 2!=0", "EMPTY"),
     * with a lag that drifted with the ARM's load. On silicon the ARM reads at
     * the frame start and the DSP writes after the burst, later in the frame. */
    if (s->tick_phase == 1) {
        if (!s->phase_a_recue) {
            CalypsoPontMsg m;
            if (pont_recv(s, &m, 0) && m.type == PONT_DONE) {
                if (m.a & PONT_DONE_PHASE_A) {
                    s->phase_a_recue = true;
                } else {
                    /* single-phase DSP: this is the final DONE */
                    s->pont_pending = false; s->pont_frames++;
                    s->phase_a_recue = true; s->go_inutile = true;
                    if (m.a & PONT_DONE_API_IRQ) { s->pont_api_irqs++; qemu_irq_raise(s->irqs[CALYPSO_IRQ_API]); }
                }
            }
            if (s->phase_a_recue) {
                /* the ROM's ISR is over (page read, R page header written):
                 * now the ARM's frame, l1_sync(fn) */
                if (s->eoi_cible < calypso_inth_frame_eoi()) s->eoi_cible = calypso_inth_frame_eoi();
                s->eoi_cible += 1;
                calypso_timer_lost_frame_tick(s->fn);
                qemu_irq_raise(s->irqs[CALYPSO_IRQ_TPU_FRAME]);
                timer_mod_ns(s->frame_irq_timer,
                             qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + FRAME_IRQ_PULSE_NS);
            }
        }
        bool fini = s->phase_a_recue &&
                    (calypso_inth_frame_eoi() >= s->eoi_cible ||
                     calypso_inth_irq_masked(CALYPSO_IRQ_TPU_FRAME));
        if (!fini && ++s->eoi_attentes < (unsigned)PONT_ATTENTES_MAX) {
            if (g_uart_modem) {
                calypso_uart_poll_backend(g_uart_modem);
                calypso_uart_kick_rx(g_uart_modem);
            }
            if (g_uart_irda) {
                calypso_uart_poll_backend(g_uart_irda);
                calypso_uart_kick_rx(g_uart_irda);
            }
            timer_mod_ns(s->tdma_timer, qemu_clock_get_ns(QEMU_CLOCK_REALTIME) + PONT_RETRY_NS);
            return;
        }
        if (!fini) {
            if (!s->phase_a_recue) {
                /* no ISR answer: give the ARM its frame anyway, do not stall */
                calypso_timer_lost_frame_tick(s->fn);
                qemu_irq_raise(s->irqs[CALYPSO_IRQ_TPU_FRAME]);
                timer_mod_ns(s->frame_irq_timer,
                             qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + FRAME_IRQ_PULSE_NS);
            }
            s->eoi_cible = calypso_inth_frame_eoi();   /* resync after a lost frame IRQ */
            if (++s->eoi_timeouts == 1 || (s->eoi_timeouts % 2170) == 0) {
                fprintf(stderr, "[trx] pont DSP : phase A %s, l1_sync de l'ARM %s apres 256 attentes "
                        "(fn=%u, %u fois), GO envoye quand meme\n",
                        s->phase_a_recue ? "recue" : "PAS recue",
                        calypso_inth_frame_eoi() >= s->eoi_cible ? "finie" : "PAS finie",
                        s->fn, s->eoi_timeouts);
            }
        }
        s->tick_phase = 0;
        if (!s->go_inutile) {
            pont_send(s, PONT_GO, s->fn, 0, 0);
        }
        tdma_pacer(s);
        return;
    }
    /* [2026-09-17] DSP LOCK-STEP (CALYPSO_PONT_LOCKSTEP=1): advance the frame
     * (fn + TPU-frame IT to the firmware) ONLY once the external DSP has
     * finished the previous one. Otherwise QEMU produces frames faster than
     * the DSP consumes them, ticks are skipped and the firmware frame counter
     * runs away (fn_offset ~600k): the three clocks (BTS/QEMU/firmware)
     * diverge and acquisition never locks. In lock-step firmware and DSP stay
     * on the SAME frame: coherent clocks, stable TOA, native lock possible. */
    if (s->pont && s->pont_fd >= 0 && s->pont_pending) {
        static int ls = -1;
        if (ls < 0) { const char *e = getenv("CALYPSO_PONT_LOCKSTEP"); ls = (e && *e=='1') ? 1 : 0; }
        if (ls) {
            CalypsoPontMsg m;
            if (pont_recv(s, &m, 0) && m.type == PONT_DONE) {
                s->pont_pending = false; s->pont_frames++;
                if (m.a & PONT_DONE_API_IRQ) { s->pont_api_irqs++; qemu_irq_raise(s->irqs[CALYPSO_IRQ_API]); }
            } else {
                /* DSP not ready yet: do not advance, retry soon.
                 * [2026-09-20] The UARTs are pumped by this tick (below) and
                 * do not depend on the DSP: keep serving them here, or the
                 * serial link only moves at the DSP's pace and osmocon's
                 * romload, which times out per block, stalls at 38-55 %. */
                if (g_uart_modem) {
                    calypso_uart_poll_backend(g_uart_modem);
                    calypso_uart_kick_rx(g_uart_modem);
                }
                if (g_uart_irda) {
                    calypso_uart_poll_backend(g_uart_irda);
                    calypso_uart_kick_rx(g_uart_irda);
                }
                timer_mod_ns(s->tdma_timer, qemu_clock_get_ns(QEMU_CLOCK_REALTIME) + PONT_RETRY_NS);
                return;
            }
        }
    }
    {
        static int ls2 = -1;
        if (ls2 < 0) { const char *e = getenv("CALYPSO_PONT_LOCKSTEP"); ls2 = (e && *e=='1') ? 1 : 0; }
        if (s->pont && s->pont_fd >= 0 && ls2) {
            s->fn = (s->fn + 1) % GSM_HYPERFRAME;   /* DSP pace, not the wall clock */
        } else {
            uint32_t wfn = __atomic_load_n(&g_wall_fn, __ATOMIC_ACQUIRE);
            s->fn = (wfn ? wfn : s->fn + 1) % GSM_HYPERFRAME;
        }
    }

    /* [2026-09-19] WHO WRITES a_sch? Every writer on the DSP side has been
     * eliminated by measurement: the C54x core (instruction probe: 0 stores),
     * its internal DMA (probe: 0 transfers), the RX_FBFLAGS/POKE crutches
     * (witnesses silent), the gr-gsm L1 (disabled under CALYPSO_DSP_EXTERN),
     * PONT_CAN_SB (off). Only the ARM firmware is left, writing through the
     * shared mapping, which no DSP-side probe can see. a_sch[0] carries plain
     * numbers (0x1111, 0x1388) where only B_BLUD and B_SCH_CRC mean anything,
     * so name the ARM PC that puts them there. a_sch = R_PAGE + 0x1E (bytes),
     * i.e. api_ram words 0x37 (page 0) and 0x4b (page 1). */
    {
        static uint16_t prev[2]; static int first = 1; static unsigned n;
        if (s->api_ram) {
            uint16_t v0 = s->api_ram[0x37], v1 = s->api_ram[0x4b];
            if (!first && n < 40 && (v0 != prev[0] || v1 != prev[1])) {
                uint32_t pc = 0;
                if (first_cpu) {
                    ARMCPU *ac = ARM_CPU(first_cpu);
                    pc = ac->env.regs[15];
                }
                fprintf(stderr, "[trx] A_SCH fn=%u page0 %04x->%04x page1 %04x->%04x "
                        "PC_ARM=0x%08x\n", s->fn, prev[0], v0, prev[1], v1, pc);
                n++;
            }
            prev[0] = v0; prev[1] = v1; first = 0;
        }
    }

    calypso_tpu_sequencer_tick(s->fn);
    if (g_uart_modem) {
        calypso_uart_poll_backend(g_uart_modem);
        calypso_uart_kick_rx(g_uart_modem);
    }
    if (g_uart_irda) {
        calypso_uart_poll_backend(g_uart_irda);
        calypso_uart_kick_rx(g_uart_irda);
    }

    if (s->pont) {
        /* The C54x consumes d_task_ra / d_task_u itself (the firmware's
         * l1s_compl() re-runs dsp_api_memset() on the write page): do NOT
         * clear them here. */
        /* [2026-09-17] PIPELINE, NO WAIT. Waiting for DONE here held the
         * global lock for a whole DSP frame, ~26 ms for 64000 insn, and the
         * ARM vCPU, which needs that lock for every MMIO access, stopped
         * advancing (stuck in hwtimer_config). The DONE of frame N is now
         * collected at tick N+1 without blocking; if it is missing the DSP is
         * late and this tick is skipped for it. Cost: the API IRQ arrives one
         * frame later than with the internal DSP. */
        /* ARM FIRST: the TICK goes out now (phase A, the ROM's frame ISR),
         * the burst only after l1_sync(fn), with PONT_GO (phase B). */
        if (arm_first()) {
            s->deux_phases = true;
        }
        pont_echange(s);
        s->deux_phases = false;
    } else {
        *api_wp(s->dsp_page, WP_D_TASK_RA) = 0;
        *api_wp(s->dsp_page, WP_D_TASK_U) = 0;
    }

    if (s->pont && s->pont_fd >= 0 && arm_first() && s->pont_pending) {
        /* Two phases: the ARM frame IRQ waits for DONE|PHASE_A (the ROM's ISR
         * has read its page), then l1_sync(fn), then PONT_GO. The EOI target
         * is cumulative: one end of service per frame IRQ raised, so a lagging
         * l1_sync(N-1) cannot pass for l1_sync(N). Resynchronised on timeout. */
        s->eoi_attentes = 0;
        s->phase_a_recue = false;
        s->go_inutile = false;
        s->tick_phase = 1;
        timer_mod_ns(s->tdma_timer, qemu_clock_get_ns(QEMU_CLOCK_REALTIME) + PONT_RETRY_NS);
        return;
    }
    calypso_timer_lost_frame_tick(s->fn);
    qemu_irq_raise(s->irqs[CALYPSO_IRQ_TPU_FRAME]);
    timer_mod_ns(s->frame_irq_timer,
                 qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + FRAME_IRQ_PULSE_NS);
    tdma_pacer(s);
}

/* Next frame at the TDMA pace of the wall clock (or as soon as we are late). */
static void tdma_pacer(CalypsoTRX *s)
{
    static int64_t target;
    int64_t now = qemu_clock_get_ns(QEMU_CLOCK_REALTIME);
    if (target == 0) {
        target = now;
    }
    target += GSM_TDMA_NS;
    while (target <= now) {
        target += GSM_TDMA_NS;
    }
    timer_mod_ns(s->tdma_timer, target);
}

static void tdma_start(CalypsoTRX *s)
{
    s->tdma_running = true;
    s->fn = 0;
    timer_mod_ns(s->tdma_timer, qemu_clock_get_ns(QEMU_CLOCK_REALTIME) + GSM_TDMA_NS);
}

static void cpu_kick(void *o)
{
    CalypsoTRX *s = o;
    if (first_cpu) {
        cpu_exit(first_cpu);
    }
    qemu_notify_event();
    timer_mod_ns(s->kick_timer, qemu_clock_get_ns(QEMU_CLOCK_REALTIME) + CPU_KICK_NS);
}

static void map_io(MemoryRegion *sysmem, MemoryRegion *mr, const MemoryRegionOps *ops,
                   void *opaque, const char *name, hwaddr base, uint64_t size)
{
    memory_region_init_io(mr, NULL, ops, opaque, name, size);
    memory_region_add_subregion(sysmem, base, mr);
}

void calypso_trx_init(MemoryRegion *sysmem, qemu_irq *irqs)
{
    CalypsoTRX *s = g_new0(CalypsoTRX, 1);
    g_trx = s;
    s->irqs = irqs;
    s->pont_fd = -1;

    const char *pont_spec = getenv("CALYPSO_DSP_EXTERN");
    if (pont_spec && *pont_spec) {
        pont_connect(s, pont_spec);
    } else {
        s->api_ram = g_new0(uint16_t, CALYPSO_API_WORDS);
    }

    map_io(sysmem, &s->api_iomem, &api_ops, s, "calypso.dsp_api", CALYPSO_API_BASE,
           CALYPSO_API_SIZE);
    if (!s->pont) {
        /* Initial state of the EMULATED bootloader. With an external DSP the
         * real ROM bootloader (parked at 0xb41c by c54x_reset) has posted IDLE
         * in that cell and the real L1 writes its version: touch nothing. */
        s->api_ram[API_BL_STATUS / 2] = BL_STATUS_READY;
        s->api_ram[API_VERSION / 2] = API_VERSION_VALUE;
    }
    s->bl_booted = true;

    map_io(sysmem, &s->tpu_iomem, &tpu_ops, s, "calypso.tpu", CALYPSO_TPU_BASE,
           CALYPSO_TPU_SIZE);
    map_io(sysmem, &s->tpu_ram_iomem, &tpu_ram_ops, s, "calypso.tpu_ram",
           CALYPSO_TPU_RAM_BASE, CALYPSO_TPU_RAM_SIZE);
    map_io(sysmem, &s->tsp_iomem, &tsp_ops, s, "calypso.tsp", CALYPSO_TSP_BASE,
           CALYPSO_TSP_SIZE);
    map_io(sysmem, &s->ulpd_iomem, &ulpd_ops, s, "calypso.ulpd", CALYPSO_ULPD_BASE,
           CALYPSO_ULPD_SIZE);
    s->sim = calypso_sim_new(s->irqs[CALYPSO_IRQ_SIM]);
    map_io(sysmem, &s->sim_iomem, &sim_ops, s, "calypso.sim", CALYPSO_SIM_BASE,
           CALYPSO_SIM_SIZE);

    s->tdma_timer = timer_new_ns(QEMU_CLOCK_REALTIME, tdma_tick, s);
    s->frame_irq_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, frame_irq_lower, s);
    s->kick_timer = timer_new_ns(QEMU_CLOCK_REALTIME, cpu_kick, s);
    timer_mod_ns(s->kick_timer, qemu_clock_get_ns(QEMU_CLOCK_REALTIME) + CPU_KICK_NS);

    g_wall_running = true;
    pthread_create(&g_wall_thread, NULL, wall_clock_loop, NULL);
    pthread_setname_np(g_wall_thread, "cal-tdma-clock");

    if (s->pont) {
        /* The external DSP only runs on TICKs, but the firmware's
         * dsp_power_on() waits for the bootloader (IDLE) BEFORE enabling the
         * TPU, which is what normally starts the tick. Clocking it from init
         * breaks that deadlock, where the ARM waits for the DSP waiting for
         * the ARM. The later tdma_start() from the TPU only resets fn to 0. */
        s->pont_boot_timer = timer_new_ns(QEMU_CLOCK_REALTIME, pont_boot_tick, s);
        timer_mod_ns(s->pont_boot_timer, qemu_clock_get_ns(QEMU_CLOCK_REALTIME) + GSM_TDMA_NS);
        fprintf(stderr, "[trx] pont DSP : timer de boot lance (echange DSP seul, "
                "sans IRQ TPU, jusqu'a ce que le firmware active le TDMA)\n");
    }
}
