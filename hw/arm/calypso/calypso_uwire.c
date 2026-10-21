/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * calypso_uwire.c - le controleur uWire du Calypso (0xFFFE4000) et, derriere
 * son chip select 0, l'ecran couleur 96x64 du Motorola C139/C140.
 *
 * [2026-10-21] Ecrit pour FreeCalypso Magnetite (cible c139, config
 * hybrid-ui-vo) : son pilote r2d (src/cs/drivers/drv_app/r2d/lcds/c139/)
 * parle a l'ecran en mots de 9 bits sur uWire CS0, bit 8 = donnee (1) ou
 * commande (0) :
 *   - commande 0x10/0x11 = debut de fenetre (x, y), 0x12/0x13 = fin de
 *     fenetre, 0x14/0x15 = curseur ; chaque commande est suivie d'UN mot de
 *     parametre (bit 8 a 1). 0x3F, 0x20, 0x31 (init) sont acceptees sans effet ;
 *   - donnees hors commande : deux octets par point (poids fort puis faible),
 *     RGB565, le curseur avance en x puis revient a xstart et passe a la ligne
 *     suivante au bout de la fenetre.
 * Le firmware n'affiche qu'une UI 84x48 centree (TI « C-Sample ») : bordure
 * 0x5555, blanc 0xFFFF, noir 0x0000.
 *
 * Registres uWire (16 bits) : 0x00 TDR/RDR (donnee cadree a gauche), 0x02 CSR
 * (bit 13 START, bit 12 CS_CMD, bits 10-11 index du CS, bits 5-9 nombre de
 * bits ecrits, bit 14 CSRB occupe : toujours 0 ici), 0x04/0x06/0x08 SR1-3.
 *
 * Sorties, comme calypso_st7558.c : la console QEMU (-display sdl, x3) et une
 * image partagee CALYPSO_LCD_SHM (defaut /dev/shm/calypso_lcd, « none ») :
 * en-tete de 32 octets (magic "C139LCD", largeur, hauteur, octets par point
 * = 2, drapeaux, compteur) puis 96 x 64 x 2 octets RGB565 petit-boutiste.
 * tools/osmo-c123-gtk.py reconnait les deux magics.
 *
 * Activation : propriete « c139-lcd » (calypso_soc.c la pose quand
 * CALYPSO_LCD=c139) ; sans elle le controleur avale les mots en silence.
 */

#include "qemu/osdep.h"
#include "qemu/timer.h"
#include "hw/sysbus.h"
#include "hw/qdev-properties.h"
#include "ui/console.h"
#include "qom/object.h"
#include <sys/stat.h>
#include <fcntl.h>

#define TYPE_CALYPSO_UWIRE "calypso-uwire"
OBJECT_DECLARE_SIMPLE_TYPE(CalypsoUwireState, CALYPSO_UWIRE)

#define UW_TDR   0x00
#define UW_CSR   0x02
#define UW_SR1   0x04
#define UW_SR2   0x06
#define UW_SR3   0x08

#define CSR_START   (1 << 13)
#define CSR_CS_CMD  (1 << 12)
#define CSR_CSRB    (1 << 14)

#define LCD_W     96
#define LCD_H     64
#define MAGNIFY   3
#define SHM_DEFAULT "/dev/shm/calypso_lcd"
#define SHM_HDR   32
#define FLUSH_MS  40

struct CalypsoUwireState {
    SysBusDevice parent_obj;
    MemoryRegion iomem;
    uint16_t tdr, csr, sr1, sr2, sr3;
    bool lcd_enabled;
    bool debug;

    /* ecran C139 */
    uint16_t ram[LCD_W * LCD_H];
    int xs, xe, ys, ye, cx, cy;
    int cmd;            /* commande en attente de son parametre, -1 sinon */
    int pix_hi;         /* octet fort du point en cours, -1 sinon */
    bool dirty, redraw;
    QEMUTimer *flush;
    QemuConsole *con;
    int shm_fd;
    uint32_t seq;
    char *shm_path;
};

static void c139_export(CalypsoUwireState *s)
{
    uint8_t buf[SHM_HDR + LCD_W * LCD_H * 2];
    int i;

    if (s->shm_fd < 0) {
        return;
    }
    s->seq++;
    memset(buf, 0, SHM_HDR);
    memcpy(buf, "C139LCD", 8);
    buf[8] = LCD_W; buf[9] = 0;
    buf[10] = LCD_H; buf[11] = 0;
    buf[12] = 2; buf[13] = 0;               /* octets par point */
    buf[14] = 0;
    buf[16] = s->seq & 0xff; buf[17] = (s->seq >> 8) & 0xff;
    buf[18] = (s->seq >> 16) & 0xff; buf[19] = (s->seq >> 24) & 0xff;
    for (i = 0; i < LCD_W * LCD_H; i++) {
        buf[SHM_HDR + 2 * i] = s->ram[i] & 0xff;
        buf[SHM_HDR + 2 * i + 1] = s->ram[i] >> 8;
    }
    if (pwrite(s->shm_fd, buf, sizeof(buf), 0) != (ssize_t)sizeof(buf)) {
        fprintf(stderr, "[c139lcd] ecriture de %s : %s\n", s->shm_path, strerror(errno));
    }
}

static void c139_flush_cb(void *opaque)
{
    CalypsoUwireState *s = opaque;

    if (s->dirty) {
        s->dirty = false;
        c139_export(s);
    }
    timer_mod(s->flush, qemu_clock_get_ms(QEMU_CLOCK_REALTIME) + FLUSH_MS);
}

static void c139_word(CalypsoUwireState *s, uint16_t w)
{
    uint8_t d = w & 0xff;

    if (!(w & 0x100)) {                     /* commande */
        s->cmd = d;
        s->pix_hi = -1;
        return;
    }
    if (s->cmd >= 0) {                      /* parametre de la commande */
        switch (s->cmd) {
        case 0x10: s->xs = d; break;
        case 0x11: s->ys = d; break;
        case 0x12: s->xe = d; break;
        case 0x13: s->ye = d; break;
        case 0x14: s->cx = d; break;
        case 0x15: s->cy = d; break;
        default: break;
        }
        if (s->debug) {
            fprintf(stderr, "[c139lcd] cmd 0x%02x <- 0x%02x\n", s->cmd, d);
        }
        s->cmd = -1;
        return;
    }
    if (s->pix_hi < 0) {                    /* octet fort du point */
        s->pix_hi = d;
        return;
    }
    if (s->cx < LCD_W && s->cy < LCD_H) {
        s->ram[s->cy * LCD_W + s->cx] = (s->pix_hi << 8) | d;
        s->dirty = s->redraw = true;
    }
    s->pix_hi = -1;
    s->cx++;
    if (s->cx > s->xe) {
        s->cx = s->xs;
        s->cy++;
        if (s->cy > s->ye) {
            s->cy = s->ys;
        }
    }
}

static uint64_t uwire_read(void *opaque, hwaddr offset, unsigned size)
{
    CalypsoUwireState *s = opaque;

    switch (offset) {
    case UW_TDR: return 0;                  /* RDR : rien a lire */
    case UW_CSR: return s->csr & ~(CSR_START | CSR_CSRB);
    case UW_SR1: return s->sr1;
    case UW_SR2: return s->sr2;
    case UW_SR3: return s->sr3;
    default: return 0;
    }
}

static void uwire_write(void *opaque, hwaddr offset, uint64_t value, unsigned size)
{
    CalypsoUwireState *s = opaque;
    uint16_t v = value;

    switch (offset) {
    case UW_TDR:
        s->tdr = v;
        break;
    case UW_CSR:
        s->csr = v;
        if (v & CSR_START) {
            int nbits = (v >> 5) & 0x1f;
            int cs = (v >> 10) & 3;
            uint16_t word;

            if (nbits == 0) {
                nbits = 16;
            }
            word = (s->tdr >> (16 - nbits)) & ((1u << nbits) - 1);
            if (cs == 0 && s->lcd_enabled) {
                c139_word(s, word);
            } else if (s->debug) {
                fprintf(stderr, "[uwire] CS%d <- 0x%03x (%d bits)\n", cs, word, nbits);
            }
            s->csr &= ~CSR_START;
        }
        break;
    case UW_SR1: s->sr1 = v; break;
    case UW_SR2: s->sr2 = v; break;
    case UW_SR3: s->sr3 = v; break;
    default: break;
    }
}

static const MemoryRegionOps uwire_ops = {
    .read = uwire_read,
    .write = uwire_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
    .impl = { .min_access_size = 2, .max_access_size = 2 },
};

static inline uint32_t rgb565_to_xrgb(uint16_t p)
{
    uint32_t r = (p >> 11) & 0x1f, g = (p >> 5) & 0x3f, b = p & 0x1f;
    return 0xff000000u | ((r << 3 | r >> 2) << 16) | ((g << 2 | g >> 4) << 8) | (b << 3 | b >> 2);
}

static void c139_update_display(void *opaque)
{
    CalypsoUwireState *s = opaque;
    DisplaySurface *surface = qemu_console_surface(s->con);
    int x, y, k, stride;
    uint8_t *base;

    if (!s->redraw || surface_bits_per_pixel(surface) != 32) {
        return;
    }
    base = surface_data(surface);
    stride = surface_stride(surface);
    for (y = 0; y < LCD_H; y++) {
        uint32_t *row = (uint32_t *)(base + (size_t)y * MAGNIFY * stride);
        for (x = 0; x < LCD_W; x++) {
            uint32_t c = rgb565_to_xrgb(s->ram[y * LCD_W + x]);
            for (k = 0; k < MAGNIFY; k++) {
                row[x * MAGNIFY + k] = c;
            }
        }
        for (k = 1; k < MAGNIFY; k++) {
            memcpy(base + ((size_t)y * MAGNIFY + k) * stride, row, LCD_W * MAGNIFY * 4);
        }
    }
    s->redraw = false;
    dpy_gfx_update(s->con, 0, 0, LCD_W * MAGNIFY, LCD_H * MAGNIFY);
}

static void c139_invalidate_display(void *opaque)
{
    CalypsoUwireState *s = opaque;
    s->redraw = true;
}

static const GraphicHwOps c139_gfx_ops = {
    .invalidate = c139_invalidate_display,
    .gfx_update = c139_update_display,
};

static void uwire_realize(DeviceState *dev, Error **errp)
{
    CalypsoUwireState *s = CALYPSO_UWIRE(dev);
    const char *env;

    s->cmd = -1;
    s->pix_hi = -1;
    s->xe = LCD_W - 1;
    s->ye = LCD_H - 1;
    s->shm_fd = -1;
    s->debug = (env = getenv("CALYPSO_LCD_DEBUG")) && *env && strcmp(env, "0");
    memory_region_init_io(&s->iomem, OBJECT(dev), &uwire_ops, s, "calypso.uwire", 0x10);
    sysbus_init_mmio(SYS_BUS_DEVICE(dev), &s->iomem);
    if (!s->lcd_enabled) {
        return;
    }
    env = getenv("CALYPSO_LCD_SHM");
    s->shm_path = g_strdup(env && *env ? env : SHM_DEFAULT);
    if (strcmp(s->shm_path, "none")) {
        s->shm_fd = open(s->shm_path, O_RDWR | O_CREAT | O_TRUNC, 0666);
        if (s->shm_fd < 0) {
            fprintf(stderr, "[c139lcd] %s : %s (pas d'image partagee)\n",
                    s->shm_path, strerror(errno));
        } else {
            fchmod(s->shm_fd, 0666);
            if (ftruncate(s->shm_fd, SHM_HDR + LCD_W * LCD_H * 2) < 0) {
                fprintf(stderr, "[c139lcd] ftruncate %s : %s\n", s->shm_path, strerror(errno));
            }
            c139_export(s);
            fprintf(stderr, "[c139lcd] ecran C139 96x64 couleur (uWire CS0) : image partagee %s\n",
                    s->shm_path);
        }
    }
    s->flush = timer_new_ms(QEMU_CLOCK_REALTIME, c139_flush_cb, s);
    timer_mod(s->flush, qemu_clock_get_ms(QEMU_CLOCK_REALTIME) + FLUSH_MS);
    s->con = graphic_console_init(dev, 0, &c139_gfx_ops, s);
    qemu_console_resize(s->con, LCD_W * MAGNIFY, LCD_H * MAGNIFY);
}

static Property uwire_properties[] = {
    DEFINE_PROP_BOOL("c139-lcd", CalypsoUwireState, lcd_enabled, false),
    DEFINE_PROP_END_OF_LIST(),
};

static void uwire_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->realize = uwire_realize;
    dc->desc = "Calypso uWire + LCD couleur 96x64 du Motorola C139";
    device_class_set_props(dc, uwire_properties);
}

static const TypeInfo uwire_info = {
    .name          = TYPE_CALYPSO_UWIRE,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(CalypsoUwireState),
    .class_init    = uwire_class_init,
};

static void uwire_register_types(void)
{
    type_register_static(&uwire_info);
}

type_init(uwire_register_types)
