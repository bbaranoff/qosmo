/*
 * tbring.c - plugin TCG : anneau des blocs ARM executes, en memoire partagee.
 *
 * [2026-10-19] Pourquoi. « -d in_asm,exec,nochain » ecrit une ligne de texte
 * par bloc execute, sous verrou, avec un fflush() par ligne (util/log.c) et un
 * lookup_symbol() par bloc : 32 M de lignes, 2,8 Go en 3 min sur le banc
 * Calypso, l'ARM 5 fois plus lent, le pont DSP qui decroche. Ici rien n'est
 * formate dans la boucle : a chaque execution d'un bloc, une ecriture de
 * 8 octets (PC, compteur) dans un anneau mmap sur /dev/shm. Le plugin est
 * appele meme pour les blocs chaines, donc pas de « nochain » : le TCG tourne
 * comme en production. Le texte se fait apres coup (tbring-dump.py), avec
 * -d in_asm (une ligne par bloc, a sa traduction) comme table de correspondance.
 *
 * Chargement :  -plugin libtbring.so[,anneau=/dev/shm/qemu-tbring][,entries=N]
 *   anneau  : le fichier (PAS « file= » : c'est le mot-cle de QEMU pour le
 *             chemin du plugin, il chargerait l'anneau comme second plugin).
 *   entries : puissance de 2 (defaut 1 Mi = 8 Mio), les N derniers blocs.
 *
 * Fichier :  en-tete 64 octets  { magic "TBRING01", entries, count, shift }
 *            puis entries x { uint32 pc ; uint32 tour }  (tour = count >> shift au
 *            moment de l'ecriture : le lecteur sait si l'entree est de la fenetre
 *            courante ou d'un tour precedent ; valable jusqu'a 2^32 tours).
 * Sur le disque (ASSEMBLY_LOGS_RING_FILE, option anneau=) le fichier est creux : 64 Gio
 * (2^33 entrees) ne coutent que ce qui a ete ecrit, a 8 octets par bloc, soit
 * ~1,4 Mo/s a 180 000 blocs/s -- contre 150 Mo/s pour le texte de -d exec.
 * Le compteur global avance a chaque bloc : tbring-dump lit le fichier a chaud,
 * a n'importe quel moment (SIGUSR1 inutile, c'est de la memoire partagee).
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
    uint64_t entries;       /* taille de l'anneau, puissance de 2 */
    uint64_t count;         /* blocs executes depuis le depart (indice d'ecriture) */
    uint64_t shift;         /* log2(entries) */
    uint64_t reserve[4];
} TbRingHdr;                /* 64 octets */

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
