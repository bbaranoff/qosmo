/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * calypso_armio.c - ARMIO du Calypso (0xFFFE4800) : GPIO + clavier matriciel.
 *
 * [2026-10-19] Avant : un bouchon qui rendait 0xFF a toute lecture, donc un
 * clavier a jamais muet (KBR_LATCH = 0x1f, aucune touche). Ici le clavier du
 * Compal E88 / Motorola C123 est modelise comme le firmware le voit
 * (calypso/keypad.c) :
 *   - KBC (0x0c)        : colonnes pilotees, actives a 0 (bits 0-4) ; 0 = toutes
 *                          (etat de repos, c'est la que l'IRQ sert), 0xff = aucune ;
 *   - KBR_LATCH (0x0a)  : lignes lues, actives a 0 (bits 0-4) : une touche
 *                          (colonne c, ligne r) tire la ligne r a 0 quand la
 *                          colonne c est pilotee ;
 *   - IRQ_KEYPAD_GPIO (8) : niveau haut tant qu'une touche est enfoncee ET que
 *                          toutes les colonnes sont pilotees (KBC = 0). Le
 *                          firmware masque l'IRQ, balaie colonne par colonne,
 *                          puis la reactive quand tout est relache.
 * Numero de touche = colonne * 5 + ligne (board/compal/keymap.h : KEY_OK = 0,
 * KEY_UP = 1, ..., KEY_1 = 15, KEY_0 = 13, KEY_POWER = 24).
 *
 * Deux entrees :
 *   - un socket Unix datagramme (CALYPSO_KEYS_SOCK, defaut /tmp/calypso_keys.sock,
 *     « none » pour ne pas l'ouvrir), une commande par datagramme :
 *         press <touche> | release <touche> | tap <touche> [ms]
 *     <touche> : 0-9, star, hash, menu, lsb, rsb, up, down, left, right, ok,
 *     power, ou k<N> (numero brut). C'est ce que parle osmo-operator/tools/
 *     osmo-c123-gtk.py (la photo du telephone, touches cliquables) ;
 *   - le clavier de la console graphique QEMU (-display sdl) : chiffres,
 *     fleches, Entree = menu, Entree pave = ok, Echap = power, F1/F2 = touches
 *     logicielles, s = *, h = #.
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/timer.h"
#include "qemu/main-loop.h"
#include "hw/sysbus.h"
#include "hw/irq.h"
#include "ui/input.h"
#include "qom/object.h"
#include <sys/socket.h>
#include <sys/un.h>
#include <sys/stat.h>
#include <fcntl.h>

#define TYPE_CALYPSO_ARMIO "calypso-armio"
OBJECT_DECLARE_SIMPLE_TYPE(CalypsoArmioState, CALYPSO_ARMIO)

#define ARMIO_LATCH_IN      0x00
#define ARMIO_LATCH_OUT     0x02
#define ARMIO_IO_CNTL       0x04
#define ARMIO_KBR_LATCH     0x0a
#define ARMIO_KBC           0x0c
#define ARMIO_GPIO_INT      0x16
#define ARMIO_GPIO_MASKIT   0x18

#define KEYS_SOCK_DEFAULT   "/tmp/calypso_keys.sock"
#define TAP_MS_DEFAULT      150
#define KEY_BITS            25          /* 5 colonnes x 5 lignes */

struct CalypsoArmioState {
    SysBusDevice parent_obj;
    MemoryRegion iomem;
    qemu_irq irq;
    uint16_t latch_out, io_cntl, kbc, gpio_int, gpio_maskit;
    uint32_t keys;          /* bit c*5+r : touche enfoncee */
    uint32_t tap_mask;      /* touches a relacher a l'echeance de tap_timer */
    QEMUTimer *tap_timer;
    int sock;
    char *sock_path;
    bool debug;
};

static const struct { const char *name; int bit; } key_names[] = {
    { "ok", 0 }, { "up", 1 }, { "down", 2 }, { "left", 3 }, { "right", 4 },
    { "3", 5 }, { "6", 6 }, { "9", 7 }, { "hash", 8 }, { "#", 8 }, { "rsb", 9 },
    { "2", 10 }, { "5", 11 }, { "8", 12 }, { "0", 13 }, { "menu", 14 },
    { "1", 15 }, { "4", 16 }, { "7", 17 }, { "star", 18 }, { "*", 18 },
    { "lsb", 19 }, { "power", 24 },
};

static int armio_key_bit(const char *name)
{
    unsigned i;
    for (i = 0; i < ARRAY_SIZE(key_names); i++) {
        if (!strcmp(key_names[i].name, name)) {
            return key_names[i].bit;
        }
    }
    if (name[0] == 'k' && name[1] >= '0' && name[1] <= '9') {
        int b = atoi(name + 1);
        return (b >= 0 && b < KEY_BITS) ? b : -1;
    }
    return -1;
}

static const char *armio_key_name(int bit)
{
    unsigned i;
    for (i = 0; i < ARRAY_SIZE(key_names); i++) {
        if (key_names[i].bit == bit) {
            return key_names[i].name;
        }
    }
    return "?";
}

static void armio_update_irq(CalypsoArmioState *s)
{
    int level = (s->keys != 0) && ((s->kbc & 0x1f) == 0);
    qemu_set_irq(s->irq, level);
}

static void armio_set_key(CalypsoArmioState *s, int bit, bool down)
{
    uint32_t m = 1u << bit;
    if (down) {
        s->keys |= m;
    } else {
        s->keys &= ~m;
        s->tap_mask &= ~m;
    }
    if (s->debug) {
        fprintf(stderr, "[armio] touche %s (k%d) %s -> keys=0x%07x kbc=0x%02x\n",
                armio_key_name(bit), bit, down ? "enfoncee" : "relachee",
                s->keys, s->kbc);
    }
    armio_update_irq(s);
}

static void armio_tap_expire(void *opaque)
{
    CalypsoArmioState *s = opaque;
    uint32_t m = s->tap_mask;
    int b;
    s->tap_mask = 0;
    for (b = 0; b < KEY_BITS; b++) {
        if (m & (1u << b)) {
            armio_set_key(s, b, false);
        }
    }
}

static void armio_command(CalypsoArmioState *s, char *line)
{
    char *cmd, *name, *arg, *save = NULL;
    int bit;

    cmd = strtok_r(line, " \t\r\n", &save);
    if (!cmd) {
        return;
    }
    name = strtok_r(NULL, " \t\r\n", &save);
    arg = strtok_r(NULL, " \t\r\n", &save);
    if (!name) {
        fprintf(stderr, "[armio] commande sans touche : « %s »\n", cmd);
        return;
    }
    bit = armio_key_bit(name);
    if (bit < 0) {
        fprintf(stderr, "[armio] touche inconnue : « %s »\n", name);
        return;
    }
    if (!strcmp(cmd, "press") || !strcmp(cmd, "down")) {
        armio_set_key(s, bit, true);
    } else if (!strcmp(cmd, "release") || !strcmp(cmd, "up")) {
        armio_set_key(s, bit, false);
    } else if (!strcmp(cmd, "tap")) {
        int ms = arg ? atoi(arg) : TAP_MS_DEFAULT;
        if (ms <= 0) {
            ms = TAP_MS_DEFAULT;
        }
        armio_set_key(s, bit, true);
        s->tap_mask |= 1u << bit;
        timer_mod(s->tap_timer, qemu_clock_get_ms(QEMU_CLOCK_VIRTUAL) + ms);
    } else {
        fprintf(stderr, "[armio] commande inconnue : « %s » (press|release|tap)\n", cmd);
    }
}

static void armio_sock_read(void *opaque)
{
    CalypsoArmioState *s = opaque;
    char buf[256];
    ssize_t n;

    while ((n = recv(s->sock, buf, sizeof(buf) - 1, 0)) > 0) {
        char *p, *nl;
        buf[n] = '\0';
        for (p = buf; p && *p; p = nl) {
            nl = strchr(p, '\n');
            if (nl) {
                *nl++ = '\0';
            }
            if (*p) {
                armio_command(s, p);
            }
        }
    }
}

static void armio_sock_open(CalypsoArmioState *s)
{
    const char *env = getenv("CALYPSO_KEYS_SOCK");
    struct sockaddr_un addr;
    int fd;

    s->sock = -1;
    s->sock_path = g_strdup(env && *env ? env : KEYS_SOCK_DEFAULT);
    if (!strcmp(s->sock_path, "none")) {
        return;
    }
    if (strlen(s->sock_path) >= sizeof(addr.sun_path)) {
        fprintf(stderr, "[armio] chemin de socket trop long : %s\n", s->sock_path);
        return;
    }
    fd = socket(AF_UNIX, SOCK_DGRAM, 0);
    if (fd < 0) {
        fprintf(stderr, "[armio] socket : %s\n", strerror(errno));
        return;
    }
    unlink(s->sock_path);
    memset(&addr, 0, sizeof(addr));
    addr.sun_family = AF_UNIX;
    strcpy(addr.sun_path, s->sock_path);
    if (bind(fd, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        fprintf(stderr, "[armio] bind %s : %s\n", s->sock_path, strerror(errno));
        close(fd);
        return;
    }
    chmod(s->sock_path, 0666);
    fcntl(fd, F_SETFL, fcntl(fd, F_GETFL, 0) | O_NONBLOCK);
    s->sock = fd;
    qemu_set_fd_handler(fd, armio_sock_read, NULL, s);
    fprintf(stderr, "[armio] clavier C123 : socket %s (press|release|tap <touche> [ms])\n",
            s->sock_path);
}

/* ---- clavier de la console graphique QEMU (-display sdl) ---------------- */

static int armio_qcode_bit(int qcode)
{
    switch (qcode) {
    case Q_KEY_CODE_0: case Q_KEY_CODE_KP_0: return 13;
    case Q_KEY_CODE_1: case Q_KEY_CODE_KP_1: return 15;
    case Q_KEY_CODE_2: case Q_KEY_CODE_KP_2: return 10;
    case Q_KEY_CODE_3: case Q_KEY_CODE_KP_3: return 5;
    case Q_KEY_CODE_4: case Q_KEY_CODE_KP_4: return 16;
    case Q_KEY_CODE_5: case Q_KEY_CODE_KP_5: return 11;
    case Q_KEY_CODE_6: case Q_KEY_CODE_KP_6: return 6;
    case Q_KEY_CODE_7: case Q_KEY_CODE_KP_7: return 17;
    case Q_KEY_CODE_8: case Q_KEY_CODE_KP_8: return 12;
    case Q_KEY_CODE_9: case Q_KEY_CODE_KP_9: return 7;
    case Q_KEY_CODE_KP_MULTIPLY: case Q_KEY_CODE_S: return 18;
    case Q_KEY_CODE_H: return 8;
    case Q_KEY_CODE_UP: return 1;
    case Q_KEY_CODE_DOWN: return 2;
    case Q_KEY_CODE_LEFT: return 3;
    case Q_KEY_CODE_RIGHT: return 4;
    case Q_KEY_CODE_RET: return 14;
    case Q_KEY_CODE_KP_ENTER: case Q_KEY_CODE_O: return 0;
    case Q_KEY_CODE_ESC: case Q_KEY_CODE_P: return 24;
    case Q_KEY_CODE_F1: return 19;
    case Q_KEY_CODE_F2: return 9;
    default: return -1;
    }
}

static void armio_input_event(DeviceState *dev, QemuConsole *src, InputEvent *evt)
{
    CalypsoArmioState *s = CALYPSO_ARMIO(dev);
    InputKeyEvent *key = evt->u.key.data;
    int bit = armio_qcode_bit(qemu_input_key_value_to_qcode(key->key));
    if (bit >= 0) {
        armio_set_key(s, bit, key->down);
    }
}

static const QemuInputHandler armio_input_handler = {
    .name  = "Calypso C123 keypad",
    .mask  = INPUT_EVENT_MASK_KEY,
    .event = armio_input_event,
};

/* ---- registres ------------------------------------------------------------ */

static uint64_t armio_read(void *opaque, hwaddr offset, unsigned size)
{
    CalypsoArmioState *s = opaque;
    uint32_t rows = 0;
    int col;

    switch (offset) {
    case ARMIO_LATCH_IN:
        return 0;
    case ARMIO_LATCH_OUT:
        return s->latch_out;
    case ARMIO_IO_CNTL:
        return s->io_cntl;
    case ARMIO_KBR_LATCH:
        for (col = 0; col < 5; col++) {
            if (!(s->kbc & (1u << col))) {
                rows |= (s->keys >> (col * 5)) & 0x1f;
            }
        }
        return (~rows) & 0x1f;
    case ARMIO_KBC:
        return s->kbc;
    case ARMIO_GPIO_INT:
        return s->gpio_int;
    case ARMIO_GPIO_MASKIT:
        return s->gpio_maskit;
    default:
        return 0;
    }
}

static void armio_write(void *opaque, hwaddr offset, uint64_t value, unsigned size)
{
    CalypsoArmioState *s = opaque;

    switch (offset) {
    case ARMIO_LATCH_OUT:
        s->latch_out = value;
        break;
    case ARMIO_IO_CNTL:
        s->io_cntl = value;
        break;
    case ARMIO_KBC:
        s->kbc = value & 0xff;
        armio_update_irq(s);
        break;
    case ARMIO_GPIO_INT:
        s->gpio_int = value;
        break;
    case ARMIO_GPIO_MASKIT:
        s->gpio_maskit = value;
        break;
    default:
        break;
    }
}

static const MemoryRegionOps armio_ops = {
    .read = armio_read,
    .write = armio_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
    .impl = { .min_access_size = 2, .max_access_size = 2 },
};

static void armio_realize(DeviceState *dev, Error **errp)
{
    CalypsoArmioState *s = CALYPSO_ARMIO(dev);
    const char *dbg = getenv("CALYPSO_KEYS_DEBUG");

    s->debug = dbg && *dbg && strcmp(dbg, "0");
    s->kbc = 0;
    s->keys = 0;
    memory_region_init_io(&s->iomem, OBJECT(dev), &armio_ops, s, "calypso.armio", 0x100);
    sysbus_init_mmio(SYS_BUS_DEVICE(dev), &s->iomem);
    sysbus_init_irq(SYS_BUS_DEVICE(dev), &s->irq);
    s->tap_timer = timer_new_ms(QEMU_CLOCK_VIRTUAL, armio_tap_expire, s);
    armio_sock_open(s);
    qemu_input_handler_register(dev, &armio_input_handler);
}

static void armio_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    dc->realize = armio_realize;
    dc->desc = "Calypso ARMIO (GPIO + clavier matriciel C123)";
}

static const TypeInfo armio_info = {
    .name          = TYPE_CALYPSO_ARMIO,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(CalypsoArmioState),
    .class_init    = armio_class_init,
};

static void armio_register_types(void)
{
    type_register_static(&armio_info);
}

type_init(armio_register_types)
