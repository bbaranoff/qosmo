/* SPDX-License-Identifier: GPL-2.0-or-later */
#include "qemu/osdep.h"
#include "qemu/timer.h"
#include "qemu/main-loop.h"
#include "sysemu/runstate.h"
#include "exec/address-spaces.h"
#include "exec/cpu-common.h"
#include "hw/core/cpu.h"
#include "hw/irq.h"
#include "hw/arm/calypso/calypso_api.h"
#include "hw/arm/calypso/calypso_l1_ops.h"
#include "hw/arm/calypso/calypso_trx.h"
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
#define CPU_KICK_NS            5000000
#define IDLE_PC_LO             0x00823000u
#define IDLE_PC_HI             0x00826000u
#define INTH_MASK_ADDR         0xFFFFFA08u
#define BOOT_POLLS_BEFORE_BOOT 3
#define FN_SYNC_TOLERANCE      16
#define FN_SYNC_WINDOW         5

typedef struct CalypsoTRX {
    qemu_irq *irqs;
    MemoryRegion api_iomem;
    uint16_t *api_ram;          /* g_new0, ou mmap du segment partage du pont */
    /* [2026-09-16] DSP EXTERNE (CALYPSO_DSP_EXTERN=<socket>|1) : la couche 1
     * n'est plus un shunt gr-gsm mais un vrai C54x qui tourne dans c54x_exe.
     * L'API RAM est partagee par /dev/shm, la trame est verrouillee par TICK/DONE
     * (cf. include/hw/arm/calypso/calypso_dsp_pont.h). */
    bool pont;
    bool pont_pending;          /* un TICK envoye, DONE pas encore releve */
    int pont_fd;
    QEMUTimer *pont_boot_timer; /* cadence le DSP seul, avant que le firmware lance le TDMA */
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
    uint8_t burst_ring[8];
    unsigned burst_w, burst_r;
    uint16_t burst_cur;
    uint32_t burst_last_fn;
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

    /* La latence gr-gsm/UDP/boucle QEMU ne peut que retarder la lecture de
     * fn local, donc gonfler fn et reduire l'offset mesure. Le maximum des
     * derniers SCH est la valeur la plus proche de la verite. */
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

/* ── pont DSP externe ─────────────────────────────────────────────────── */
static bool pont_send(CalypsoTRX *s, uint32_t type, uint32_t a, uint32_t b)
{
    CalypsoPontMsg m = { type, a, b };
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
    /* Fenetre ARM de 64 Ko, alignee sur une page ; les 16 premiers Ko sont le
     * segment partage (la fenetre API du DSP), le reste est prive et vide. */
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
    if (!pont_send(s, PONT_HELLO, CALYPSO_API_WORDS, CALYPSO_PONT_MAGIC) ||
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
    if (size != 2) {
        return val;
    }

    if (s->pont) {
        /* Le C54x externe ecrit lui-meme d_task_d / d_burst_d dans la page de
         * lecture, ET son bootloader ROM (parque en 0xb41c) repond lui-meme aux
         * commandes BL_CMD_STATUS : l'emulation ci-dessous, faite pour le shunt
         * sans DSP, forcerait IDLE apres trois lectures, AVANT que le DSP n'ait
         * vu la commande - il rate alors le COPY_BLOCK de demarrage. */
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

    /* [2026-09-16] La fenetre entiere, pour les L1 qui la decodent (le C54x).
     * Avant les hooks nommes ci-dessous, et apres le rangement ci-dessus : une
     * L1 qui relit l'API RAM depuis ce callback doit y voir la valeur ecrite. */
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
            /* En DSP externe cette valeur est en fait BL_CMD_COPY_BLOCK (2) :
             * le vrai bootloader la consomme, on ne touche pas a l'API RAM. */
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
        /* Relache du reset : la ROM de boot repart de 0xff80, pose IDLE et se
         * parque en 0xb41c, prete pour les COPY_BLOCK de l'ARM. C'est le SEUL
         * moment ou l'on remet le C54x a zero - jamais sur une commande du
         * bootloader (qosmo-dsp : « NE PAS re-reset ... preserve bootloader cmd »). */
        pont_send(s, PONT_RESET, 0, 0);
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

/* [2026-09-16] Battement impose de l'exterieur, a un numero de trame donne.
 * C'est le maitre d'horloge de la L1 C54x : la-bas c'est le timer TINT0 du DSP
 * qui cadence le TDMA, pas l'horloge murale de la plateforme. Sens L1 ->
 * plateforme, donc symbole direct et non entree de vtable (cf.
 * calypso_l1_ops.h) : il n'a qu'une implementation possible. */
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

/* Un echange TICK/DONE avec le DSP externe : releve le DONE de la trame
 * precedente sans attendre, puis envoie le TICK de celle-ci si le DSP est libre. */
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
    if (libre && pont_send(s, PONT_TICK, s->fn, s->dsp_page)) {
        s->pont_pending = true;
    }
}

/* Avant que le firmware n'active le TPU (ce qui lance tdma_tick), le DSP
 * externe doit deja tourner : dsp_power_on() attend son bootloader. Ce timer
 * ne fait QUE l'echange avec le DSP - pas d'IRQ TPU-frame, pas de sequenceur,
 * l'ARM n'a pas encore installe ses vecteurs. Il s'arrete de lui-meme quand
 * le vrai tick prend le relais. */
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

static void tdma_tick(void *opaque)
{
    CalypsoTRX *s = opaque;
    if (!runstate_is_running()) {
        timer_mod_ns(s->tdma_timer, qemu_clock_get_ns(QEMU_CLOCK_REALTIME) + GSM_TDMA_NS);
        return;
    }
    uint32_t wfn = __atomic_load_n(&g_wall_fn, __ATOMIC_ACQUIRE);
    s->fn = (wfn ? wfn : s->fn + 1) % GSM_HYPERFRAME;

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
        /* Le C54x consomme lui-meme d_task_ra / d_task_u (l1s_compl() du
         * firmware refait dsp_api_memset() sur la page d'ecriture) : ne pas
         * les effacer ici, cf. qosmo-dsp « Do NOT clear tasks here ». */
        /* [2026-09-17] PIPELINE, PAS D'ATTENTE. Attendre DONE ici bloquait la
         * boucle principale (verrou global tenu) pendant toute la trame du DSP,
         * ~26 ms pour 64000 insn, et le vCPU ARM, qui a besoin du verrou pour
         * chaque acces MMIO, n'avancait plus (bloque dans hwtimer_config).
         * Desormais : le DONE de la trame N est releve au tick N+1 sans
         * attendre ; s'il n'est pas la, le DSP est en retard et ce tick lui est
         * saute. Cout : l'IRQ API arrive une trame plus tard qu'en interne. */
        pont_echange(s);
    } else {
        *api_wp(s->dsp_page, WP_D_TASK_RA) = 0;
        *api_wp(s->dsp_page, WP_D_TASK_U) = 0;
    }

    calypso_timer_lost_frame_tick(s->fn);
    qemu_irq_raise(s->irqs[CALYPSO_IRQ_TPU_FRAME]);
    timer_mod_ns(s->frame_irq_timer,
                 qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + FRAME_IRQ_PULSE_NS);

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
        /* Etat initial du bootloader EMULE. En DSP externe, c'est le vrai
         * bootloader ROM (parque en 0xb41c par c54x_reset) qui a pose IDLE dans
         * la cellule, et la vraie L1 qui ecrira sa version : on ne touche a rien. */
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
        /* Le DSP externe ne tourne que sur les TICK. Or dsp_power_on() du
         * firmware attend le bootloader (IDLE) AVANT d'activer le TPU, qui est
         * ce qui lance normalement le tick : on cadence donc des l'init, sinon
         * l'ARM attend le DSP qui attend l'ARM. Le tdma_start() du TPU, plus
         * tard, ne fait que remettre fn a 0. */
        s->pont_boot_timer = timer_new_ns(QEMU_CLOCK_REALTIME, pont_boot_tick, s);
        timer_mod_ns(s->pont_boot_timer, qemu_clock_get_ns(QEMU_CLOCK_REALTIME) + GSM_TDMA_NS);
        fprintf(stderr, "[trx] pont DSP : timer de boot lance (echange DSP seul, "
                "sans IRQ TPU, jusqu'a ce que le firmware active le TDMA)\n");
    }
}
