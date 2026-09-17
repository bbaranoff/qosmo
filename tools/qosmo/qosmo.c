/*
 * qosmo-launch.c — lanceur C de qemu-system-arm pour la machine Calypso.
 *
 * Compilé DEUX fois, dans le dossier de chaque fork :
 *   /opt/GSM/qosmo-dsp/tools/qosmo-launch    -> qosmo-dsp    (-DQOSMO_DSP=1)
 *   /opt/GSM/qosmo-grgsm/tools/qosmo-launch  -> qosmo-grgsm  (-DQOSMO_DSP=0)
 * La source est IDENTIQUE dans les deux dossiers ; le Makefile fixe l'alias,
 * l'arbre QEMU et la saveur. Installé dans /usr/local/bin par `make install`.
 *
 * Ce que fait le lanceur, et rien de plus :
 *   1. résout le firmware (-k ELF | dossier | .bin) et ses voisins (.bin pour
 *      osmocon, .map) ; lit les symboles `l1s` et `last_rach` dans l'ELF
 *      (ELF32 ARM) et les exporte comme run_modules/16-fwsyms.sh le fait
 *      avec nm (CALYPSO_L1S_FN_ADDR / CALYPSO_LAST_RACH_FN_ADDR) ;
 *   2. (saveur dsp) résout les 7 ROMs du C54x depuis un dossier ou un préfixe
 *      (-dsp) et les passe en propriétés machine dsp-prom0=… ;
 *   3. expose les sockets et le réseau AVEC LES DÉFAUTS QUI MARCHENT DÉJÀ
 *      (ceux de run.sh / 40-qemu.sh) : moniteur unix, gdbstub ARM tcp::1234,
 *      L1CTL /tmp/osmocom_l2, TRXDv0 udp 0.0.0.0:6702, IQ tee 127.0.0.1:6703 ;
 *      chacun est choisi par une option (--monitor, --gdb, --l1ctl, --bind,
 *      --trx-port, --iq-tee) ;
 *   4. lance QEMU avec `-serial pty -serial pty`, relaie son stderr tel quel
 *      (les modules 41-pty / grep « redirected » continuent de marcher), et
 *      publie des liens STABLES vers les deux pty : <rundir>/modem.pty et
 *      <rundir>/irda.pty — c'est là-dessus qu'on fait osmocon directement ;
 *   5. sur demande (-o) lance osmocon lui-même (-m romload) dès que le pty
 *      modem existe ; transmet SIGINT/SIGTERM à QEMU ; sort avec son code.
 *
 * Tout argument inconnu commençant par `-` (et sa valeur si elle ne commence
 * pas par `-`) est transmis à QEMU. Après `--`, tout va à QEMU.
 */
#define _GNU_SOURCE
#include <arpa/inet.h>
#include <ctype.h>
#include <dirent.h>
#include <elf.h>
#include <errno.h>
#include <fcntl.h>
#include <ifaddrs.h>
#include <libgen.h>
#include <limits.h>
#include <net/if.h>
#include <netinet/in.h>
#include <signal.h>
#include <stdarg.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/mman.h>
#include <sys/socket.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <unistd.h>

#ifndef QOSMO_ALIAS
#define QOSMO_ALIAS "qosmo-dsp"
#endif
#ifndef QOSMO_TREE
#define QOSMO_TREE "/opt/GSM/qosmo-dsp"
#endif
#ifndef QOSMO_DSP
#define QOSMO_DSP 1
#endif
#ifndef QOSMO_GSM_ROOT
#define QOSMO_GSM_ROOT "/opt/GSM"
#endif
#ifndef QOSMO_VERSION
#define QOSMO_VERSION "1.0"
#endif

/* ---- défauts = ce qui marche déjà (run.sh / paths.env / 40-qemu.sh) ---- */
#define DEF_QEMU        QOSMO_TREE "/build/qemu-system-arm"
#define DEF_FIRMWARE    QOSMO_GSM_ROOT "/firmware/board/compal_e88/layer1.highram.elf"
#define DEF_DSP_DIR     QOSMO_GSM_ROOT
#define DEF_OSMOCON     QOSMO_GSM_ROOT "/osmocom-bb/src/host/osmocon/osmocon"
#define DEF_CPU         "arm946"
#define DEF_GDB         "1234"
/* [2026-09-05] LA MACHINE calypso N A PAS D ECRAN, QEMU LUI EN OUVRAIT UN.
 * Sans -display, QEMU prend son defaut graphique (SDL est compile dans ce
 * fork : `qemu-system-arm -display help` le liste) et, display actif, il
 * cable ses peripheriques par defaut sur des consoles virtuelles. Les deux
 * serie sont deja prises (-serial pty x2) et le moniteur aussi (-monitor
 * unix:) : il ne reste que le port parallele. On obtenait une fenetre SDL
 * vide intitulee « parallel0 », qui capte le clavier, et devant laquelle on
 * attend un boot qui, lui, se passe sur les pty. Sans DISPLAY
 * (osmo-banc.service au demarrage) c est pire : QEMU ne peut pas ouvrir SDL
 * et SORT. Le modele n a rien a afficher : --display none par defaut.
 * `--display sdl` (ou -display/-nographic apres --) rend l ancien
 * comportement. */
#define DEF_DISPLAY     "none"
#define DEF_L1CTL       "/tmp/osmocom_l2"
#define DEF_BIND        "0.0.0.0"
#define DEF_TRX_PORT    "6702"
#define DEF_IQ_TEE_HOST "127.0.0.1"
#define DEF_IQ_TEE_PORT "6703"
#define DEF_OSMOCON_MODEL "romload"
#define DEF_OSMOCON_DELAY "100"
#define DEF_OSMOCON_DEBUG "tr"
#define GRGSM_GSMTAP_PORT "4730"   /* fixe dans calypso_l1_grgsm.c */

static const char *g_alias = QOSMO_ALIAS;

/* ------------------------------------------------------------------ util */
static void say(const char *fmt, ...)
{
    va_list ap;
    fprintf(stderr, "[%s] ", g_alias);
    va_start(ap, fmt);
    vfprintf(stderr, fmt, ap);
    va_end(ap);
    fputc('\n', stderr);
}

static void die(const char *fmt, ...)
{
    va_list ap;
    fprintf(stderr, "[%s] ERREUR : ", g_alias);
    va_start(ap, fmt);
    vfprintf(stderr, fmt, ap);
    va_end(ap);
    fputc('\n', stderr);
    exit(2);
}

static char *xstrdup(const char *s)
{
    char *d = strdup(s ? s : "");
    if (!d) die("mémoire épuisée");
    return d;
}

static char *xasprintf(const char *fmt, ...)
{
    char *p = NULL;
    va_list ap;
    va_start(ap, fmt);
    if (vasprintf(&p, fmt, ap) < 0) die("mémoire épuisée");
    va_end(ap);
    return p;
}

static bool is_dir(const char *p)
{
    struct stat st;
    return p && stat(p, &st) == 0 && S_ISDIR(st.st_mode);
}

static bool is_readable_file(const char *p)
{
    struct stat st;
    return p && stat(p, &st) == 0 && S_ISREG(st.st_mode) && access(p, R_OK) == 0;
}

static bool is_exec(const char *p)
{
    struct stat st;
    return p && stat(p, &st) == 0 && S_ISREG(st.st_mode) && access(p, X_OK) == 0;
}

static bool ends_with(const char *s, const char *suf)
{
    size_t ls = strlen(s), lf = strlen(suf);
    return ls >= lf && strcmp(s + ls - lf, suf) == 0;
}

static const char *env_nonempty(const char *n)
{
    const char *v = getenv(n);
    return (v && *v) ? v : NULL;
}

/* Pose une variable SEULEMENT si elle est absente ou vide : l'environnement
 * hérité (run.sh, opérateur) garde la main, sauf option explicite. */
static void env_default(const char *n, const char *v)
{
    if (!env_nonempty(n) && v && *v) setenv(n, v, 1);
}

static void env_force(const char *n, const char *v)
{
    if (v && *v) setenv(n, v, 1);
}

static int mkdir_p(const char *path)
{
    char tmp[PATH_MAX];
    snprintf(tmp, sizeof tmp, "%s", path);
    for (char *p = tmp + 1; *p; p++) {
        if (*p == '/') {
            *p = 0;
            if (mkdir(tmp, 0755) < 0 && errno != EEXIST) return -1;
            *p = '/';
        }
    }
    if (mkdir(tmp, 0755) < 0 && errno != EEXIST) return -1;
    return 0;
}

static char *abspath(const char *p)
{
    char buf[PATH_MAX];
    if (realpath(p, buf)) return xstrdup(buf);
    return xstrdup(p);
}

/* ------------------------------------------------------------ argv dyn. */
typedef struct { char **v; int n, cap; } argv_t;

static void argv_push(argv_t *a, const char *s)
{
    if (a->n + 2 > a->cap) {
        a->cap = a->cap ? a->cap * 2 : 32;
        a->v = realloc(a->v, sizeof(char *) * a->cap);
        if (!a->v) die("mémoire épuisée");
    }
    a->v[a->n++] = xstrdup(s);
    a->v[a->n] = NULL;
}

/* ------------------------------------------------------ symboles ELF32 */
/* Cherche `names[i]` dans la table des symboles d'un ELF32 little-endian et
 * renvoie sa valeur dans addrs[i] (found[i]=true). Équivalent de
 * `nm ELF | awk '$3==nom'` de 16-fwsyms.sh, sans dépendre de binutils. */
static int elf32_lookup(const char *path, const char **names, int n,
                        uint32_t *addrs, bool *found)
{
    int fd = open(path, O_RDONLY);
    if (fd < 0) return -1;
    struct stat st;
    if (fstat(fd, &st) < 0 || st.st_size < (off_t)sizeof(Elf32_Ehdr)) { close(fd); return -1; }
    uint8_t *m = mmap(NULL, st.st_size, PROT_READ, MAP_PRIVATE, fd, 0);
    close(fd);
    if (m == MAP_FAILED) return -1;
    int rc = -1;
    Elf32_Ehdr *eh = (Elf32_Ehdr *)m;
    if (memcmp(eh->e_ident, ELFMAG, SELFMAG) || eh->e_ident[EI_CLASS] != ELFCLASS32 ||
        eh->e_ident[EI_DATA] != ELFDATA2LSB)
        goto out;
    if (eh->e_shoff == 0 || eh->e_shentsize != sizeof(Elf32_Shdr) ||
        (off_t)eh->e_shoff + (off_t)((size_t)eh->e_shnum * sizeof(Elf32_Shdr)) > st.st_size)
        goto out;
    Elf32_Shdr *sh = (Elf32_Shdr *)(m + eh->e_shoff);
    for (int i = 0; i < n; i++) found[i] = false;
    rc = 0;
    for (int s = 0; s < eh->e_shnum; s++) {
        if (sh[s].sh_type != SHT_SYMTAB) continue;
        if (sh[s].sh_link >= eh->e_shnum) continue;
        Elf32_Shdr *strs = &sh[sh[s].sh_link];
        if ((off_t)sh[s].sh_offset + sh[s].sh_size > st.st_size ||
            (off_t)strs->sh_offset + strs->sh_size > st.st_size)
            continue;
        Elf32_Sym *sym = (Elf32_Sym *)(m + sh[s].sh_offset);
        size_t nsym = sh[s].sh_size / sizeof(Elf32_Sym);
        const char *strtab = (const char *)(m + strs->sh_offset);
        for (size_t k = 0; k < nsym; k++) {
            if (sym[k].st_name >= strs->sh_size) continue;
            const char *nm = strtab + sym[k].st_name;
            for (int i = 0; i < n; i++) {
                if (!found[i] && strcmp(nm, names[i]) == 0) {
                    addrs[i] = sym[k].st_value;
                    found[i] = true;
                }
            }
        }
    }
out:
    munmap(m, st.st_size);
    return rc;
}

/* --------------------------------------------------------- ROMs du DSP */
static const char *rom_tokens[] = { "PROM0", "PROM1", "PROM2", "PROM3", "DROM", "PDROM", "Registers" };
static const char *rom_props[]  = { "dsp-prom0", "dsp-prom1", "dsp-prom2", "dsp-prom3", "dsp-drom", "dsp-pdrom", "dsp-registers" };
static const char *rom_envs[]   = { "DSP_PROM0", "DSP_PROM1", "DSP_PROM2", "DSP_PROM3", "DSP_DROM", "DSP_PDROM", "DSP_REGISTERS" };
#define N_ROMS 7

static int strcasestr_has(const char *hay, const char *needle)
{
    return strcasestr(hay, needle) != NULL;
}

/* Cherche la ROM `tok` dans `dir` : noms connus d'abord, puis balayage
 * (tout fichier dont le nom contient "<tok>.bin", insensible à la casse). */
static char *find_rom_in_dir(const char *dir, const char *tok)
{
    const char *pat[] = { "%s/calypso_dsp.%s.bin", "%s/%s.bin", "%s/.%s.bin",
                          "%s/calypso_dsp_%s.bin", "%s/dsp.%s.bin", "%s/dsp_%s.bin" };
    for (size_t i = 0; i < sizeof pat / sizeof *pat; i++) {
        char *p = xasprintf(pat[i], dir, tok);
        if (is_readable_file(p)) return p;
        free(p);
    }
    DIR *d = opendir(dir);
    if (!d) return NULL;
    struct dirent *e;
    char *best = NULL;
    char *needle = xasprintf("%s.bin", tok);
    while ((e = readdir(d))) {
        if (!strcasestr_has(e->d_name, needle)) continue;
        /* évite PDROM quand on cherche DROM */
        if (strcmp(tok, "DROM") == 0 && strcasestr_has(e->d_name, "PDROM")) continue;
        char *p = xasprintf("%s/%s", dir, e->d_name);
        if (is_readable_file(p)) { best = p; break; }
        free(p);
    }
    free(needle);
    closedir(d);
    return best;
}

/* `spec` = dossier, ou préfixe/fichier d'une ROM (ex. /x/calypso_dsp.PROM0.bin
 * ou /x/calypso_dsp.). Remplit roms[]. Renvoie le nombre trouvé. */
static int resolve_roms(const char *spec, char **roms)
{
    int n = 0;
    if (is_dir(spec)) {
        for (int i = 0; i < N_ROMS; i++) {
            roms[i] = find_rom_in_dir(spec, rom_tokens[i]);
            if (roms[i]) n++;
        }
        return n;
    }
    /* préfixe : coupe au premier jeton connu s'il est présent */
    char *prefix = xstrdup(spec);
    for (int i = 0; i < N_ROMS; i++) {
        char *hit = strcasestr(prefix, rom_tokens[i]);
        if (hit) { *hit = 0; break; }
    }
    for (int i = 0; i < N_ROMS; i++) {
        char *p = xasprintf("%s%s.bin", prefix, rom_tokens[i]);
        if (is_readable_file(p)) { roms[i] = p; n++; }
        else { free(p); roms[i] = NULL; }
    }
    if (n == 0) {
        /* peut-être un fichier dans un dossier aux noms exotiques : on
         * retombe sur le dossier */
        char *dup = xstrdup(spec);
        char *dir = dirname(dup);
        for (int i = 0; i < N_ROMS; i++) {
            roms[i] = find_rom_in_dir(dir, rom_tokens[i]);
            if (roms[i]) n++;
        }
        free(dup);
    }
    free(prefix);
    return n;
}

/* --------------------------------------------------------------- réseau */
static bool is_ipv4_literal(const char *s)
{
    struct in_addr a;
    return inet_aton(s, &a) != 0;
}

/* `spec` = IPv4 littérale, ou nom d'interface (eth0, lo, …) -> IPv4 */
static char *resolve_bind(const char *spec)
{
    if (is_ipv4_literal(spec)) return xstrdup(spec);
    struct ifaddrs *ifa = NULL, *p;
    if (getifaddrs(&ifa) < 0) die("--bind %s : getifaddrs : %s", spec, strerror(errno));
    char *res = NULL;
    for (p = ifa; p; p = p->ifa_next) {
        if (!p->ifa_addr || p->ifa_addr->sa_family != AF_INET) continue;
        if (strcmp(p->ifa_name, spec) != 0) continue;
        char buf[INET_ADDRSTRLEN];
        struct sockaddr_in *sin = (struct sockaddr_in *)p->ifa_addr;
        if (inet_ntop(AF_INET, &sin->sin_addr, buf, sizeof buf)) { res = xstrdup(buf); break; }
    }
    freeifaddrs(ifa);
    if (!res) die("--bind %s : ni adresse IPv4 ni interface avec une IPv4", spec);
    return res;
}

/* `spec` -> argument de `-gdb` : off | PORT | ADDR:PORT | IFACE:PORT | raw
 * (contient déjà "tcp:" ou "unix:"). Renvoie NULL pour off. */
static char *gdb_spec(const char *spec)
{
    if (!spec || !*spec || !strcmp(spec, "off") || !strcmp(spec, "none") || !strcmp(spec, "0"))
        return NULL;
    if (strstr(spec, "tcp:") || strstr(spec, "unix:")) return xstrdup(spec);
    const char *colon = strrchr(spec, ':');
    if (!colon) {
        char *e; long p = strtol(spec, &e, 10);
        if (*e || p <= 0 || p > 65535) die("--gdb %s : port invalide", spec);
        return xasprintf("tcp::%ld", p);
    }
    char *host = strndup(spec, colon - spec);
    char *hres = resolve_bind(host);
    free(host);
    char *r = xasprintf("tcp:%s:%s", hres, colon + 1);
    free(hres);
    return r;
}

/* ------------------------------------------------------------- options */
typedef struct {
    const char *qemu, *firmware, *fw_bin, *cpu, *rundir, *monitor, *gdb, *display;
    const char *l1ctl, *bind, *trx_port, *iq_tee;
    const char *pty_link, *irda_link;
    const char *dsp_spec;
    const char *osmocon_bin, *osmocon_model, *osmocon_delay, *osmocon_debug, *osmocon_log;
    bool osmocon, dry_run, quiet, no_pty_link, no_monitor;
    argv_t extra;   /* args QEMU en plus */
    argv_t envs;    /* -e VAR=VAL */
} opts_t;

static void usage(FILE *f)
{
    fprintf(f,
"Usage : %s [-k FIRMWARE.elf]%s [options] [-- args QEMU]\n"
"\n"
"Lance qemu-system-arm (machine calypso) du fork %s avec les défauts qui\n"
"marchent déjà, publie les sockets/pty, et permet osmocon directement dessus.\n"
"\n"
"Firmware :\n"
"  -k, -kernel FICHIER     ELF layer1 (défaut : $FIRMWARE_ELF ou\n"
"                          %s)\n"
"                          Un DOSSIER => layer1.highram.elf dedans ; un .bin => l'.elf voisin.\n"
"                          Les symboles l1s/last_rach sont lus dans l'ELF, le .bin voisin\n"
"                          sert à osmocon.\n"
"      --bin FICHIER       image .bin pour osmocon (défaut : <elf sans .elf>.bin)\n"
"%s"
"\n"
"Sockets :\n"
"  -r, --rundir DIR        dossier des sockets/liens/pid (défaut : $RUN_DIR ou /tmp/%s)\n"
"  -M, --monitor CHEMIN    socket moniteur QEMU (défaut : <rundir>/qemu-monitor.sock ; none = aucun)\n"
"  -s, --l1ctl CHEMIN      socket L1CTL (défaut : $L1CTL_SOCK ou %s) — exportée L1CTL_SOCK,\n"
"                          passée à osmocon -s\n"
"  -p, --pty CHEMIN        lien stable vers le pty modem/serial0 (défaut : <rundir>/modem.pty)\n"
"      --irda CHEMIN       lien stable vers le pty IrDA/serial1 (défaut : <rundir>/irda.pty)\n"
"      --no-pty-link       ne crée aucun lien\n"
"\n"
"Réseau :\n"
"  -g, --gdb SPEC          gdbstub ARM : PORT | ADDR:PORT | IFACE:PORT | off (défaut : %s)\n"
"      --bind ADDR|IFACE   adresse/interface d'écoute du modèle (TRXDv0) (défaut : %s)%s\n"
"      --trx-port N        port UDP TRXDv0 d'entrée des bursts (défaut : %s)%s\n"
"      --iq-tee HOST:PORT  destination du tee I/Q (défaut : %s:%s)%s\n"
"\n"
"osmocon :\n"
"  -o, --osmocon           lance osmocon (-m %s -i %s) dès que le pty modem existe\n"
"      --osmocon-bin P     binaire (défaut : $OSMOCON ou %s)\n"
"      --osmocon-model M   (défaut %s)   --osmocon-delay N (défaut %s)   --osmocon-debug F (défaut %s)\n"
"      --osmocon-log F     journal d'osmocon (défaut : <rundir>/osmocon.log ; - = hérite du terminal)\n"
"\n"
"Divers :\n"
"      --qemu CHEMIN       binaire qemu-system-arm (défaut : $QEMU_BIN ou %s)\n"
"      --cpu MODÈLE        (défaut %s)\n"
"      --display BACKEND   affichage QEMU : none | sdl | … ($QOSMO_DISPLAY, défaut %s).\n"
"                          La machine calypso n'a pas d'écran : `none` évite la fenêtre\n"
"                          vide « parallel0 » et permet de tourner sans DISPLAY.\n"
"      --nographic         équivalent de --display none\n"
"  -e, --env VAR=VAL       variable d'environnement pour le modèle (CALYPSO_*, …)\n"
"  -n, --dry-run           affiche l'environnement et la commande, ne lance rien\n"
"  -q, --quiet             pas de bannière\n"
"  -h, --help  -V, --version\n"
"\n"
"Tout argument inconnu commençant par `-` est transmis à QEMU (avec sa valeur si\n"
"elle ne commence pas par `-`). Après `--`, tout va à QEMU.\n"
"\n"
"Exemples :\n"
"  %s -k /opt/GSM/firmware/board/compal_e88/layer1.highram.elf%s -o\n"
"  osmocon -m romload -i 100 -p /tmp/%s/modem.pty -s %s layer1.highram.bin\n",
        g_alias, QOSMO_DSP ? " [-dsp DOSSIER|PRÉFIXE]" : "", QOSMO_TREE, DEF_FIRMWARE,
        QOSMO_DSP ?
"\n"
"DSP (TMS320C54x) :\n"
"  -dsp, --dsp DOSSIER|PRÉFIXE\n"
"                          ROMs PROM0..3, DROM, PDROM, Registers. Dossier : calypso_dsp.X.bin,\n"
"                          X.bin, .X.bin ou tout *X.bin ; préfixe : /chemin/calypso_dsp.\n"
"                          Défaut : $DSP_PROM0..$DSP_REGISTERS si posés, sinon $DSP_ROM_DIR\n"
"                          ou " DEF_DSP_DIR ".\n" : "",
        g_alias, DEF_L1CTL, DEF_GDB,
        DEF_BIND, QOSMO_DSP ? "" : "  [sans effet sur ce fork]",
        DEF_TRX_PORT, QOSMO_DSP ? "" : "  [sans effet sur ce fork]",
        DEF_IQ_TEE_HOST, DEF_IQ_TEE_PORT, QOSMO_DSP ? "" : "  [sans effet sur ce fork]",
        DEF_OSMOCON_MODEL, DEF_OSMOCON_DELAY, DEF_OSMOCON, DEF_OSMOCON_MODEL, DEF_OSMOCON_DELAY, DEF_OSMOCON_DEBUG,
        DEF_QEMU, DEF_CPU, DEF_DISPLAY,
        g_alias, QOSMO_DSP ? " -dsp /opt/GSM" : "", g_alias, DEF_L1CTL);
}

/* --opt VAL | --opt=VAL | -oVAL(non) ; renvoie la valeur, avance *i */
static const char *optval(int argc, char **argv, int *i, const char *eq)
{
    if (eq) return eq + 1;
    if (*i + 1 >= argc) die("option %s : valeur manquante", argv[*i]);
    return argv[++(*i)];
}

static bool optis(const char *a, const char *eq, const char *s1, const char *s2, const char *s3)
{
    size_t l = eq ? (size_t)(eq - a) : strlen(a);
    if (s1 && strlen(s1) == l && !strncmp(a, s1, l)) return true;
    if (s2 && strlen(s2) == l && !strncmp(a, s2, l)) return true;
    if (s3 && strlen(s3) == l && !strncmp(a, s3, l)) return true;
    return false;
}

static void parse_args(int argc, char **argv, opts_t *o)
{
    for (int i = 1; i < argc; i++) {
        const char *a = argv[i];
        if (!strcmp(a, "--")) { for (i++; i < argc; i++) argv_push(&o->extra, argv[i]); break; }
        const char *eq = (a[0] == '-' && a[1] == '-') ? strchr(a, '=') : NULL;
#define IS(...) optis(a, eq, __VA_ARGS__)
#define VAL()   optval(argc, argv, &i, eq)
        if (IS("-h", "--help", NULL))        { usage(stdout); exit(0); }
        else if (IS("-V", "--version", NULL)) { printf("%s %s (%s)\n", g_alias, QOSMO_VERSION, QOSMO_TREE); exit(0); }
        else if (IS("-k", "-kernel", "--kernel")) o->firmware = VAL();
        else if (IS("--bin", NULL, NULL))     o->fw_bin = VAL();
        else if (IS("-dsp", "--dsp", "--dsp-roms")) {
            if (!QOSMO_DSP) die("%s : pas de DSP émulé sur ce fork (utilisez qosmo-dsp)", a);
            o->dsp_spec = VAL();
        }
        else if (IS("-r", "--rundir", NULL))  o->rundir = VAL();
        else if (IS("-M", "--monitor", NULL)) o->monitor = VAL();
        else if (IS("--no-monitor", NULL, NULL)) o->no_monitor = true;
        else if (IS("-s", "--l1ctl", NULL))   o->l1ctl = VAL();
        else if (IS("-p", "--pty", "--pty-link")) o->pty_link = VAL();
        else if (IS("--irda", "--irda-link", NULL)) o->irda_link = VAL();
        else if (IS("--no-pty-link", NULL, NULL)) o->no_pty_link = true;
        else if (IS("-g", "--gdb", NULL))     o->gdb = VAL();
        else if (IS("--no-gdb", NULL, NULL))  o->gdb = "off";
        else if (IS("--bind", "--iface", "--interface")) o->bind = VAL();
        else if (IS("--trx-port", "--bsp-port", NULL)) o->trx_port = VAL();
        else if (IS("--iq-tee", NULL, NULL))  o->iq_tee = VAL();
        else if (IS("-o", "--osmocon", NULL)) o->osmocon = true;
        else if (IS("--no-osmocon", NULL, NULL)) o->osmocon = false;
        else if (IS("--osmocon-bin", NULL, NULL))   o->osmocon_bin = VAL();
        else if (IS("--osmocon-model", NULL, NULL)) o->osmocon_model = VAL();
        else if (IS("--osmocon-delay", NULL, NULL)) o->osmocon_delay = VAL();
        else if (IS("--osmocon-debug", NULL, NULL)) o->osmocon_debug = VAL();
        else if (IS("--osmocon-log", NULL, NULL))   o->osmocon_log = VAL();
        else if (IS("--qemu", "--qemu-bin", NULL)) o->qemu = VAL();
        else if (IS("--cpu", "-cpu", NULL))   o->cpu = VAL();
        else if (IS("--display", "-display", NULL)) o->display = VAL();
        else if (IS("--nographic", "-nographic", NULL)) o->display = "none";
        else if (IS("-e", "--env", NULL))     argv_push(&o->envs, VAL());
        else if (IS("-n", "--dry-run", NULL)) o->dry_run = true;
        else if (IS("-q", "--quiet", NULL))   o->quiet = true;
        else if (a[0] == '-' && a[1]) {
            /* inconnu -> QEMU, avec sa valeur éventuelle */
            argv_push(&o->extra, a);
            if (i + 1 < argc && argv[i + 1][0] != '-') argv_push(&o->extra, argv[++i]);
        }
        else if (!o->firmware && (ends_with(a, ".elf") || ends_with(a, ".bin") || is_dir(a)))
            o->firmware = a;
        else die("argument inattendu : %s (voir --help)", a);
#undef IS
#undef VAL
    }
}

/* ------------------------------------------------------ pty / osmocon */
static volatile sig_atomic_t g_sig = 0;
static void on_sig(int s) { g_sig = s; }

static pid_t g_qemu = 0, g_osmocon = 0;

static void make_link(const char *target, const char *link)
{
    if (!link || !*link) return;
    unlink(link);
    if (symlink(target, link) < 0)
        say("lien %s -> %s impossible : %s", link, target, strerror(errno));
}

static void write_pid(const char *path, pid_t pid)
{
    FILE *f = fopen(path, "w");
    if (!f) return;
    fprintf(f, "%d\n", (int)pid);
    fclose(f);
}

static pid_t spawn_osmocon(const opts_t *o, const char *pty)
{
    if (!is_exec(o->osmocon_bin)) {
        say("osmocon introuvable ou non exécutable : %s (--osmocon-bin)", o->osmocon_bin);
        return 0;
    }
    if (!is_readable_file(o->fw_bin)) {
        say("image .bin pour osmocon illisible : %s (--bin)", o->fw_bin ? o->fw_bin : "(aucune)");
        return 0;
    }
    /* comme 50-osmocon.sh : la socket d'un run précédent ferait refuser le bind */
    unlink(o->l1ctl);
    pid_t p = fork();
    if (p < 0) { say("fork osmocon : %s", strerror(errno)); return 0; }
    if (p == 0) {
        if (o->osmocon_log && strcmp(o->osmocon_log, "-") != 0) {
            int fd = open(o->osmocon_log, O_WRONLY | O_CREAT | O_APPEND, 0644);
            if (fd >= 0) { dup2(fd, 1); dup2(fd, 2); close(fd); }
        }
        execl(o->osmocon_bin, o->osmocon_bin,
              "-m", o->osmocon_model, "-i", o->osmocon_delay,
              "-p", pty, "-s", o->l1ctl, o->fw_bin,
              "-d", o->osmocon_debug, (char *)NULL);
        _exit(127);
    }
    return p;
}

/* ------------------------------------------------------------------ main */
int main(int argc, char **argv)
{
    opts_t o = { 0 };
    parse_args(argc, argv, &o);

    /* -- environnement posé par -e, AVANT les défauts (il gagne) -- */
    for (int i = 0; i < o.envs.n; i++) {
        char *kv = xstrdup(o.envs.v[i]);
        char *e = strchr(kv, '=');
        if (!e) die("-e %s : attendu VAR=VAL", kv);
        *e = 0;
        setenv(kv, e + 1, 1);
        free(kv);
    }

    /* -- QEMU -- */
    if (!o.qemu) o.qemu = env_nonempty("QEMU_BIN") ? env_nonempty("QEMU_BIN") : DEF_QEMU;
    if (!is_exec(o.qemu)) die("qemu-system-arm introuvable : %s (--qemu ; compilez : ninja -C %s/build qemu-system-arm)", o.qemu, QOSMO_TREE);
    if (!o.cpu) o.cpu = DEF_CPU;
    if (!o.display) o.display = env_nonempty("QOSMO_DISPLAY") ? env_nonempty("QOSMO_DISPLAY") : DEF_DISPLAY;

    /* -- firmware -- */
    if (!o.firmware) o.firmware = env_nonempty("FIRMWARE_ELF") ? env_nonempty("FIRMWARE_ELF") : DEF_FIRMWARE;
    char *fw = xstrdup(o.firmware);
    if (is_dir(fw)) { char *t = xasprintf("%s/layer1.highram.elf", fw); free(fw); fw = t; }
    if (ends_with(fw, ".bin")) {
        char *elf = xstrdup(fw); strcpy(elf + strlen(elf) - 4, ".elf");
        if (!is_readable_file(elf)) die("firmware %s : .elf voisin introuvable (%s)", fw, elf);
        if (!o.fw_bin) o.fw_bin = xstrdup(fw);
        free(fw); fw = elf;
    }
    if (!is_readable_file(fw)) die("firmware ELF illisible : %s (-k)", fw);
    fw = abspath(fw);
    char *stem = xstrdup(fw);
    if (ends_with(stem, ".elf")) stem[strlen(stem) - 4] = 0;
    if (!o.fw_bin) {
        char *b = xasprintf("%s.bin", stem);
        if (!is_readable_file(b) && env_nonempty("FIRMWARE_BIN")) { free(b); b = xstrdup(env_nonempty("FIRMWARE_BIN")); }
        o.fw_bin = b;
    }
    /* Garde-fou : sous QEMU le stub romload de hw/char/calypso_uart.c IGNORE le
     * payload du .bin (le firmware qui tourne est l'ELF de -kernel), le .bin ne
     * sert qu'a la poignee de main d'osmocon. Mais FIRMWARE_BIN (run.sh) est
     * pose independamment de FIRMWARE_ELF : un .bin d'un autre build passe
     * inapercu, et sur un VRAI telephone ce serait bien lui qui tournerait. */
    bool fw_mismatch = false;
    {
        char *sib = xasprintf("%s.bin", stem);
        char *a = abspath(o.fw_bin), *b = abspath(sib);
        if (is_readable_file(sib) && strcmp(a, b) != 0) fw_mismatch = true;
        free(a); free(b); free(sib);
    }
    char *fw_map = xasprintf("%s.map", stem);
    char *fw_dir = xstrdup(fw); fw_dir = dirname(fw_dir);

    const char *symnames[2] = { "l1s", "last_rach" };
    uint32_t symaddr[2] = { 0, 0 }; bool symok[2] = { false, false };
    if (elf32_lookup(fw, symnames, 2, symaddr, symok) < 0)
        say("avertissement : %s n'est pas un ELF32 lisible, symboles non lus", fw);

    env_default("FIRMWARE_ELF", fw);
    env_default("FIRMWARE_BIN", o.fw_bin);
    env_default("FIRMWARE_DIR", fw_dir);
    env_default("CALYPSO_FIRMWARE_ELF", fw);
    env_default("CALYPSO_GDB_ELF", fw);
    if (symok[0]) { char *v = xasprintf("0x%08x", symaddr[0]); env_default("CALYPSO_L1S_FN_ADDR", v); free(v); }
    if (symok[1]) { char *v = xasprintf("0x%08x", symaddr[1]); env_default("CALYPSO_LAST_RACH_FN_ADDR", v); free(v); }

    /* -- rundir / sockets -- */
    if (!o.rundir) o.rundir = env_nonempty("RUN_DIR") ? env_nonempty("RUN_DIR") : xasprintf("/tmp/%s", g_alias);
    if (!o.dry_run && mkdir_p(o.rundir) < 0) die("rundir %s : %s", o.rundir, strerror(errno));
    if (o.no_monitor || (o.monitor && (!strcmp(o.monitor, "none") || !strcmp(o.monitor, "off")))) o.monitor = NULL, o.no_monitor = true;
    else if (!o.monitor) o.monitor = xasprintf("%s/qemu-monitor.sock", o.rundir);
    if (!o.l1ctl) o.l1ctl = env_nonempty("L1CTL_SOCK") ? env_nonempty("L1CTL_SOCK") : DEF_L1CTL;
    env_force("L1CTL_SOCK", o.l1ctl);           /* lu par calypso_soc.c (fork dsp) */
    env_default("CALYPSO_L1CTL_SOCK", o.l1ctl);
    if (!o.no_pty_link) {
        if (!o.pty_link)  o.pty_link  = xasprintf("%s/modem.pty", o.rundir);
        if (!o.irda_link) o.irda_link = xasprintf("%s/irda.pty", o.rundir);
    }

    /* -- réseau -- */
    char *gdb = gdb_spec(o.gdb ? o.gdb : (env_nonempty("CALYPSO_GDB_PORT") ? env_nonempty("CALYPSO_GDB_PORT") : DEF_GDB));
    if (gdb) { const char *p = strrchr(gdb, ':'); if (p) env_force("CALYPSO_GDB_PORT", p + 1); }
    char *bind_addr = NULL;
    if (QOSMO_DSP) {
        if (o.bind) { bind_addr = resolve_bind(o.bind); env_force("CALYPSO_BSP_BIND_ADDR", bind_addr); }
        else { env_default("CALYPSO_BSP_BIND_ADDR", DEF_BIND); bind_addr = xstrdup(getenv("CALYPSO_BSP_BIND_ADDR")); }
        if (o.trx_port) env_force("CALYPSO_BSP_PORT", o.trx_port);
        else env_default("CALYPSO_BSP_PORT", DEF_TRX_PORT);
        if (o.iq_tee) {
            char *d = xstrdup(o.iq_tee); char *c = strrchr(d, ':');
            if (!c) die("--iq-tee %s : attendu HOST:PORT", o.iq_tee);
            *c = 0; char *h = resolve_bind(d);
            env_force("CALYPSO_IQ_TEE_HOST", h); env_force("CALYPSO_IQ_TEE_PORT", c + 1);
            free(h); free(d);
        } else { env_default("CALYPSO_IQ_TEE_HOST", DEF_IQ_TEE_HOST); env_default("CALYPSO_IQ_TEE_PORT", DEF_IQ_TEE_PORT); }
    } else if (o.bind || o.trx_port || o.iq_tee) {
        say("avertissement : --bind/--trx-port/--iq-tee sont sans effet sur le fork gr-gsm (aucune écoute configurable dans son modèle)");
    }

    /* -- ROMs DSP -- */
    char *roms[N_ROMS] = { 0 };
    char *mach = xstrdup("calypso");
    if (QOSMO_DSP) {
        int n = 0;
        if (o.dsp_spec) {
            n = resolve_roms(o.dsp_spec, roms);
        } else {
            bool any_env = false;
            for (int i = 0; i < N_ROMS; i++) if (env_nonempty(rom_envs[i])) any_env = true;
            if (any_env) {
                for (int i = 0; i < N_ROMS; i++) {
                    const char *v = env_nonempty(rom_envs[i]);
                    if (v && is_readable_file(v)) { roms[i] = xstrdup(v); n++; }
                }
            }
            if (n == 0) {
                const char *d = env_nonempty("DSP_ROM_DIR") ? env_nonempty("DSP_ROM_DIR") : DEF_DSP_DIR;
                n = resolve_roms(d, roms);
                o.dsp_spec = d;
            } else o.dsp_spec = "$DSP_PROM0..$DSP_REGISTERS";
        }
        for (int i = 0; i < 6; i++)
            if (!roms[i]) die("ROM DSP %s introuvable (recherche : %s). Utilisez -dsp DOSSIER ou -dsp /chemin/calypso_dsp.",
                              rom_tokens[i], o.dsp_spec);
        if (!roms[6]) say("avertissement : Registers.bin absent — le DSP partira sans instantané des MMR");
        for (int i = 0; i < N_ROMS; i++) {
            if (!roms[i]) continue;
            roms[i] = abspath(roms[i]);
            char *t = xasprintf("%s,%s=%s", mach, rom_props[i], roms[i]); free(mach); mach = t;
            env_force(rom_envs[i], roms[i]);
            char *ce = xasprintf("CALYPSO_%s", rom_envs[i]); env_force(ce, roms[i]); free(ce);
        }
        {
            char *dd = xstrdup(roms[0]); env_default("DSP_ROM_DIR", dirname(dd)); free(dd);
        }
    }

    /* -- osmocon -- */
    if (!o.osmocon_bin)   o.osmocon_bin   = env_nonempty("OSMOCON") ? env_nonempty("OSMOCON") : DEF_OSMOCON;
    if (!o.osmocon_model) o.osmocon_model = env_nonempty("OSMOCON_MODEL") ? env_nonempty("OSMOCON_MODEL") : DEF_OSMOCON_MODEL;
    if (!o.osmocon_delay) o.osmocon_delay = env_nonempty("OSMOCON_INTER_BYTE_DELAY") ? env_nonempty("OSMOCON_INTER_BYTE_DELAY") : DEF_OSMOCON_DELAY;
    if (!o.osmocon_debug) o.osmocon_debug = env_nonempty("OSMOCON_DEBUG") ? env_nonempty("OSMOCON_DEBUG") : DEF_OSMOCON_DEBUG;
    if (!o.osmocon_log)   o.osmocon_log   = xasprintf("%s/osmocon.log", o.rundir);

    /* -- ligne de commande QEMU -- */
    argv_t qa = { 0 };
    argv_push(&qa, o.qemu);
    bool user_M = false, user_cpu = false, user_serial = false, user_kernel = false;
    bool user_display = false, user_parallel = false;
    for (int i = 0; i < o.extra.n; i++) {
        if (!strcmp(o.extra.v[i], "-M") || !strcmp(o.extra.v[i], "-machine")) user_M = true;
        if (!strcmp(o.extra.v[i], "-cpu")) user_cpu = true;
        if (!strcmp(o.extra.v[i], "-serial")) user_serial = true;
        if (!strcmp(o.extra.v[i], "-kernel")) user_kernel = true;
        if (!strcmp(o.extra.v[i], "-display") || !strcmp(o.extra.v[i], "-nographic")) user_display = true;
        if (!strcmp(o.extra.v[i], "-parallel")) user_parallel = true;
    }
    if (!user_M)   { argv_push(&qa, "-M"); argv_push(&qa, mach); }
    if (!user_cpu) { argv_push(&qa, "-cpu"); argv_push(&qa, o.cpu); }
    /* -display none : aucune fenetre. -parallel none : et plus de chardev
     * « parallel0 » du tout (`info chardev` le montrait encore en `vc`
     * malgre -display none). La machine calypso n a pas de port parallele
     * a modeliser : ce peripherique par defaut n existait que pour porter
     * la console virtuelle qu on vient de supprimer. */
    if (!user_display)  { argv_push(&qa, "-display"); argv_push(&qa, o.display); }
    if (!user_parallel && !strcmp(o.display, "none")) { argv_push(&qa, "-parallel"); argv_push(&qa, "none"); }
    if (gdb)       { argv_push(&qa, "-gdb"); argv_push(&qa, gdb); }
    if (!user_serial) { argv_push(&qa, "-serial"); argv_push(&qa, "pty"); argv_push(&qa, "-serial"); argv_push(&qa, "pty"); }
    if (!o.no_monitor) { argv_push(&qa, "-monitor"); char *m = xasprintf("unix:%s,server,nowait", o.monitor); argv_push(&qa, m); free(m); }
    if (!user_kernel) { argv_push(&qa, "-kernel"); argv_push(&qa, fw); }
    for (int i = 0; i < o.extra.n; i++) argv_push(&qa, o.extra.v[i]);

    /* -- bannière -- */
    if (!o.quiet || o.dry_run) {
        say("fork      : %s  (%s)", QOSMO_TREE, QOSMO_DSP ? "DSP C54x émulé sur la mask-ROM TI" : "L1 gr-gsm, DSP non émulé");
        say("qemu      : %s", o.qemu);
        say("firmware  : %s", fw);
        say("  .bin    : %s%s", o.fw_bin, is_readable_file(o.fw_bin) ? "" : "  (ABSENT)");
        if (fw_mismatch)
            say("  ATTENTION: ce .bin n'est pas le voisin de l'ELF (%s.bin existe). Sous QEMU le stub\n"
                "[%s]             romload ignore son contenu (c'est l'ELF qui tourne, symboles corrects), mais\n"
                "[%s]             FIRMWARE_BIN et FIRMWARE_ELF divergent : passez --bin %s.bin", stem, g_alias, g_alias, stem);
        say("  .map    : %s", is_readable_file(fw_map) ? fw_map : "(absent)");
        if (symok[0] || symok[1])
            say("  symboles: l1s=%s%08x last_rach=%s%08x  (CALYPSO_L1S_FN_ADDR / CALYPSO_LAST_RACH_FN_ADDR)",
                symok[0] ? "0x" : "?", symaddr[0], symok[1] ? "0x" : "?", symaddr[1]);
        else say("  symboles: l1s / last_rach ABSENTS de l'ELF (AGCH en péril, cf. 16-fwsyms.sh)");
        if (QOSMO_DSP) {
            say("ROMs DSP  : %s", o.dsp_spec);
            for (int i = 0; i < N_ROMS; i++) if (roms[i]) say("  %-9s: %s", rom_tokens[i], roms[i]);
        }
        say("rundir    : %s", o.rundir);
        say("moniteur  : %s", o.no_monitor ? "(aucun)" : o.monitor);
        say("affichage : %s", user_display ? "(passé à QEMU tel quel)" : o.display);
        say("gdb ARM   : %s", gdb ? gdb : "(off)");
        say("L1CTL     : %s", o.l1ctl);
        if (QOSMO_DSP)
            say("réseau    : TRXDv0 udp %s:%s (entrée bursts)  |  IQ tee -> udp %s:%s",
                getenv("CALYPSO_BSP_BIND_ADDR"), getenv("CALYPSO_BSP_PORT"),
                getenv("CALYPSO_IQ_TEE_HOST"), getenv("CALYPSO_IQ_TEE_PORT"));
        else
            say("réseau    : GSMTAP udp/%s (fixe dans calypso_l1_grgsm.c)", GRGSM_GSMTAP_PORT);
        say("pty liens : modem=%s  irda=%s", o.pty_link ? o.pty_link : "(aucun)", o.irda_link ? o.irda_link : "(aucun)");
        say("osmocon   : %s%s", o.osmocon ? "lancé automatiquement -> " : "à la main -> ",
            o.osmocon ? o.osmocon_log : "");
        say("  %s -m %s -i %s -p %s -s %s %s -d %s", o.osmocon_bin, o.osmocon_model, o.osmocon_delay,
            o.pty_link ? o.pty_link : "<pty serial0>", o.l1ctl, o.fw_bin, o.osmocon_debug);
    }

    if (o.dry_run) {
        const char *shown[] = { "FIRMWARE_ELF", "FIRMWARE_BIN", "CALYPSO_FIRMWARE_ELF", "CALYPSO_GDB_ELF",
            "CALYPSO_L1S_FN_ADDR", "CALYPSO_LAST_RACH_FN_ADDR", "L1CTL_SOCK", "CALYPSO_GDB_PORT",
            "CALYPSO_BSP_BIND_ADDR", "CALYPSO_BSP_PORT", "CALYPSO_IQ_TEE_HOST", "CALYPSO_IQ_TEE_PORT",
            "DSP_ROM_DIR", "DSP_PROM0", "DSP_PROM1", "DSP_PROM2", "DSP_PROM3", "DSP_DROM", "DSP_PDROM", "DSP_REGISTERS" };
        printf("# environnement\n");
        for (size_t i = 0; i < sizeof shown / sizeof *shown; i++)
            if (env_nonempty(shown[i])) printf("export %s='%s'\n", shown[i], getenv(shown[i]));
        printf("# commande\n");
        for (int i = 0; i < qa.n; i++) printf("%s%s", i ? " " : "", qa.v[i]);
        printf("\n");
        return 0;
    }

    /* -- lancement : stdout+stderr de QEMU dans un tube, relayés et scrutés -- */
    int pfd[2];
    if (pipe(pfd) < 0) die("pipe : %s", strerror(errno));
    struct sigaction sa = { 0 };
    sa.sa_handler = on_sig;           /* pas SA_RESTART : read() rend EINTR */
    sigaction(SIGINT, &sa, NULL); sigaction(SIGTERM, &sa, NULL); sigaction(SIGHUP, &sa, NULL);
    struct sigaction sc = { 0 };
    sc.sa_handler = on_sig; sc.sa_flags = SA_NOCLDSTOP;
    sigaction(SIGCHLD, &sc, NULL);

    g_qemu = fork();
    if (g_qemu < 0) die("fork : %s", strerror(errno));
    if (g_qemu == 0) {
        close(pfd[0]);
        /* QEMU annonce ses pty sur STDOUT (« char device redirected to … »)
         * et ses sondes sur STDERR : on capture les deux, comme le
         * `>>qemu.log 2>&1` de 40-qemu.sh. */
        dup2(pfd[1], 1);
        dup2(pfd[1], 2);
        close(pfd[1]);
        signal(SIGINT, SIG_DFL); signal(SIGTERM, SIG_DFL); signal(SIGHUP, SIG_DFL); signal(SIGCHLD, SIG_DFL);
        execv(qa.v[0], qa.v);
        fprintf(stderr, "[%s] exec %s : %s\n", g_alias, qa.v[0], strerror(errno));
        _exit(127);
    }
    close(pfd[1]);
    {
        char *p = xasprintf("%s/%s.pid", o.rundir, g_alias); write_pid(p, getpid()); free(p);
        p = xasprintf("%s/%s-qemu.pid", o.rundir, g_alias); write_pid(p, g_qemu); free(p);
    }
    if (!o.quiet) say("QEMU pid %d", (int)g_qemu);

    char buf[65536];
    char line[4096]; size_t ll = 0;
    bool modem_done = false, irda_done = false;
    int term_sent = 0;
    int qemu_status = -1;
    for (;;) {
        if (g_sig) {
            int s = g_sig; g_sig = 0;
            if (s == SIGCHLD) {
                int st; pid_t p;
                while ((p = waitpid(-1, &st, WNOHANG)) > 0) {
                    if (p == g_qemu) { qemu_status = st; g_qemu = 0; }
                    else if (p == g_osmocon) { if (!o.quiet) say("osmocon terminé (%d)", WIFEXITED(st) ? WEXITSTATUS(st) : -1); g_osmocon = 0; }
                }
            } else {
                if (g_qemu > 0) kill(g_qemu, term_sent++ ? SIGKILL : SIGTERM);
                if (g_osmocon > 0) kill(g_osmocon, SIGTERM);
            }
        }
        ssize_t n = read(pfd[0], buf, sizeof buf);
        if (n < 0) { if (errno == EINTR) continue; break; }
        if (n == 0) break;                          /* QEMU a fermé stderr */
        (void)!write(2, buf, n);                    /* relais tel quel */
        for (ssize_t i = 0; i < n; i++) {
            char c = buf[i];
            if (c == '\n') {
                line[ll] = 0;
                /* "char device redirected to /dev/pts/N (label serial0)" */
                char *r = strstr(line, "redirected to ");
                if (r) {
                    char *dev = r + strlen("redirected to ");
                    char *sp = strchr(dev, ' ');
                    if (sp) *sp = 0;
                    const char *lab = sp ? strstr(sp + 1, "label ") : NULL;
                    if (lab && !strncmp(lab + 6, "serial0", 7) && !modem_done) {
                        modem_done = true;
                        if (o.pty_link) make_link(dev, o.pty_link);
                        {
                            char *p = xasprintf("%s/modem.pty.path", o.rundir);
                            FILE *f = fopen(p, "w"); if (f) { fprintf(f, "%s\n", dev); fclose(f); } free(p);
                        }
                        if (!o.quiet) say("pty modem : %s%s%s", dev, o.pty_link ? "  ->  " : "", o.pty_link ? o.pty_link : "");
                        if (o.osmocon) {
                            g_osmocon = spawn_osmocon(&o, o.pty_link ? o.pty_link : dev);
                            if (g_osmocon > 0) {
                                char *p = xasprintf("%s/%s-osmocon.pid", o.rundir, g_alias); write_pid(p, g_osmocon); free(p);
                                if (!o.quiet) say("osmocon pid %d  (journal %s)", (int)g_osmocon, o.osmocon_log);
                            }
                        } else if (!o.quiet) {
                            say("osmocon   : %s -m %s -i %s -p %s -s %s %s -d %s", o.osmocon_bin, o.osmocon_model,
                                o.osmocon_delay, o.pty_link ? o.pty_link : dev, o.l1ctl, o.fw_bin, o.osmocon_debug);
                        }
                    } else if (lab && !strncmp(lab + 6, "serial1", 7) && !irda_done) {
                        irda_done = true;
                        if (o.irda_link) make_link(dev, o.irda_link);
                        if (!o.quiet) say("pty irda  : %s%s%s", dev, o.irda_link ? "  ->  " : "", o.irda_link ? o.irda_link : "");
                    }
                }
                ll = 0;
            } else if (ll + 1 < sizeof line) {
                line[ll++] = c;
            }
        }
    }
    close(pfd[0]);
    if (g_qemu > 0) {
        int st;
        while (waitpid(g_qemu, &st, 0) < 0 && errno == EINTR) {}
        qemu_status = st; g_qemu = 0;
    }
    if (g_osmocon > 0) { kill(g_osmocon, SIGTERM); waitpid(g_osmocon, NULL, 0); g_osmocon = 0; }

    /* ménage : liens et pid ; la socket L1CTL est laissée (osmocon/mobile la gèrent) */
    if (o.pty_link) unlink(o.pty_link);
    if (o.irda_link) unlink(o.irda_link);
    { char *p = xasprintf("%s/%s.pid", o.rundir, g_alias); unlink(p); free(p);
      p = xasprintf("%s/%s-qemu.pid", o.rundir, g_alias); unlink(p); free(p);
      p = xasprintf("%s/%s-osmocon.pid", o.rundir, g_alias); unlink(p); free(p);
      p = xasprintf("%s/modem.pty.path", o.rundir); unlink(p); free(p); }

    int rc = 1;
    if (qemu_status >= 0) {
        if (WIFEXITED(qemu_status)) rc = WEXITSTATUS(qemu_status);
        else if (WIFSIGNALED(qemu_status)) rc = 128 + WTERMSIG(qemu_status);
    }
    if (!o.quiet) say("QEMU terminé (code %d)", rc);
    return rc;
}
