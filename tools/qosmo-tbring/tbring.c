/*
 * tbring.c - TCG plugin: ring of the executed ARM blocks, in shared memory.
 *
 * Why. "-d in_asm,exec,nochain" writes one text line per executed block,
 * under a lock, with one fflush() per line (util/log.c) and one
 * lookup_symbol() per block: on the Calypso bench that is 2.8 GB in 3 min,
 * the ARM 5 times slower and the DSP bridge losing sync. Here nothing is
 * formatted in the loop: every execution of a block is an 8-byte write (PC,
 * counter) into a ring mmapped on /dev/shm. The plugin is called even for
 * chained blocks, so no "nochain": the TCG runs as in production. The text is
 * produced afterwards (tbring-dump.py), with -d in_asm (one line per block,
 * at translation) as the lookup table.
 *
 * Loading:  -plugin libtbring.so[,anneau=/dev/shm/qemu-tbring][,entries=N]
 *   anneau  : the file (NOT "file=": that is QEMU's keyword for the plugin
 *             path, it would load the ring as a second plugin).
 *   entries : power of 2 (default 1 Mi = 8 MiB), the last N blocks.
 *
 * File:  64-byte header  { magic "TBRING01", entries, count, shift }
 *        then entries x { uint32 pc ; uint32 tour }  (tour = count >> shift at
 *        write time: the reader knows whether the entry belongs to the current
 *        window or to a previous lap; valid up to 2^32 laps).
 * On disk (ASSEMBLY_LOGS_RING_FILE, option anneau=) the file is sparse: 64 GiB
 * (2^33 entries) only cost what has been written, 8 bytes per block, i.e.
 * ~1.4 MB/s at 180 000 blocks/s -- against 150 MB/s for the text of -d exec.
 * The global counter advances on every block: tbring-dump reads the file live,
 * at any time (no SIGUSR1 needed, it is shared memory).
 */
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <inttypes.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/mman.h>
#include <glib.h>
#include <qemu-plugin.h>

QEMU_PLUGIN_EXPORT int qemu_plugin_version = QEMU_PLUGIN_VERSION;

typedef struct {
    char     magic[8];      /* "TBRING01" */
    uint64_t entries;       /* ring size, power of 2 */
    uint64_t count;         /* blocks executed since start (write index) */
    uint64_t shift;         /* log2(entries) */
    uint64_t reserve[4];
} TbRingHdr;                /* 64 bytes */

typedef struct {
    uint32_t pc;
    uint32_t tour;
} TbRingEnt;

static TbRingHdr *g_hdr;
static TbRingEnt *g_ring;
static uint64_t   g_mask;
static unsigned   g_shift;

static void tb_exec(unsigned int vcpu_index, void *udata)
{
    uint64_t i = __atomic_fetch_add(&g_hdr->count, 1, __ATOMIC_RELAXED);
    TbRingEnt *e = &g_ring[i & g_mask];
    e->pc   = (uint32_t)(uintptr_t)udata;
    e->tour = (uint32_t)(i >> g_shift);
}

static void tb_trans(qemu_plugin_id_t id, struct qemu_plugin_tb *tb)
{
    uint64_t pc = qemu_plugin_tb_vaddr(tb);
    qemu_plugin_register_vcpu_tb_exec_cb(tb, tb_exec, QEMU_PLUGIN_CB_NO_REGS,
                                         (void *)(uintptr_t)pc);
}

static void at_exit(qemu_plugin_id_t id, void *p)
{
    g_autoptr(GString) s = g_string_new(NULL);
    g_string_printf(s, "tbring: %" PRIu64 " blocs executes, anneau de %" PRIu64 "\n",
                    g_hdr->count, g_hdr->entries);
    qemu_plugin_outs(s->str);
}

QEMU_PLUGIN_EXPORT int qemu_plugin_install(qemu_plugin_id_t id,
                                           const qemu_info_t *info,
                                           int argc, char **argv)
{
    const char *file = "/dev/shm/qemu-tbring";
    uint64_t entries = 1u << 20;
    for (int i = 0; i < argc; i++) {
        g_auto(GStrv) tok = g_strsplit(argv[i], "=", 2);
        if (!tok[0] || !tok[1]) { fprintf(stderr, "tbring: option invalide %s\n", argv[i]); return -1; }
        if (!strcmp(tok[0], "anneau") || !strcmp(tok[0], "ring")) file = g_strdup(tok[1]);
        else if (!strcmp(tok[0], "entries")) entries = g_ascii_strtoull(tok[1], NULL, 0);
        else { fprintf(stderr, "tbring: option inconnue %s\n", tok[0]); return -1; }
    }
    if (entries < 1024 || (entries & (entries - 1))) {
        fprintf(stderr, "tbring: entries doit etre une puissance de 2 >= 1024\n"); return -1;
    }
    size_t sz = sizeof(TbRingHdr) + entries * sizeof(TbRingEnt);
    int fd = open(file, O_RDWR | O_CREAT | O_TRUNC, 0644);
    if (fd < 0 || ftruncate(fd, sz) < 0) { perror("tbring: open/ftruncate"); return -1; }
    void *m = mmap(NULL, sz, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
    close(fd);
    if (m == MAP_FAILED) { perror("tbring: mmap"); return -1; }
    g_hdr = m;
    memcpy(g_hdr->magic, "TBRING01", 8);
    g_hdr->entries = entries;
    g_hdr->count = 0;
    g_ring = (TbRingEnt *)((char *)m + sizeof(TbRingHdr));
    g_mask = entries - 1;
    g_shift = __builtin_ctzll(entries);
    g_hdr->shift = g_shift;
    qemu_plugin_register_vcpu_tb_trans_cb(id, tb_trans);
    qemu_plugin_register_atexit_cb(id, at_exit, NULL);
    fprintf(stderr, "tbring: anneau de %" PRIu64 " blocs dans %s (%zu octets)\n", entries, file, sz);
    return 0;
}
