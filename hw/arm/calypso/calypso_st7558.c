/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * calypso_st7558.c - l'ecran du Motorola C123 : LCD Sitronix ST7558, 96x65
 * points monochromes, esclave I2C 0x3c (firmware : fb/fb_st7558.c).
 *
 * [2026-10-19] Protocole, tel que le firmware le parle :
 *   - premier octet apres l'adresse = octet de controle : 0x00 -> commandes,
 *     0x40 -> donnees (RAM) ;
 *   - commandes : 0x40|page (adresse Y, 9 pages de 8 lignes), 0x80|x (adresse
 *     X, 0..101), 0x20/0x21 (jeu d'instructions normal/etendu), 0x0c video
 *     normale, 0x0d video inverse, 0x08 ecran eteint ; le reste (bias, V_OP,
 *     booster, miroirs) est accepte sans effet ;
 *   - donnees : un octet = 8 points verticaux (bit 0 en haut) de la colonne x
 *     de la page courante, x avance apres chaque octet.
 * fb_st7558_flush() envoie par paquets de 16 octets (la FIFO I2C) : l'ecran
 * est republie a la fin de chaque transaction qui a touche la RAM.
 *
 * Deux sorties :
 *   - la console graphique QEMU (-display sdl : fenetre 96x65 agrandie x3) ;
 *   - un fichier image sous /dev/shm (CALYPSO_LCD_SHM, defaut
 *     /dev/shm/calypso_lcd, « none » pour ne rien ecrire) : en-tete de 32
 *     octets (magic "C123LCD", largeur, hauteur, pages, drapeaux bit0 inverse
 *     bit1 eteint, compteur de version) puis 96 x 9 octets de pages, ce que
 *     lit osmo-operator/tools/osmo-c123-gtk.py pour incruster l'ecran dans la
 *     photo du telephone.
 */

#include "qemu/osdep.h"
#include "hw/i2c/i2c.h"
#include "ui/console.h"
#include "qom/object.h"
#include <sys/stat.h>
#include <fcntl.h>

#define TYPE_ST7558 "st7558"
OBJECT_DECLARE_SIMPLE_TYPE(St7558State, ST7558)

#define ST7558_COLS   102
#define ST7558_PAGES  9
#define LCD_W         96
#define LCD_H         65
#define MAGNIFY       3
#define SHM_DEFAULT   "/dev/shm/calypso_lcd"
#define SHM_HDR       32

enum { MODE_IDLE, MODE_CMD, MODE_DATA };

struct St7558State {
    I2CSlave parent_obj;
    QemuConsole *con;
    uint8_t ram[ST7558_PAGES * ST7558_COLS];
    int page, x, mode;
    bool ext, inverse, off;
    bool redraw, dirty;
    int shm_fd;
    uint32_t seq;
    char *shm_path;
};

static void st7558_export(St7558State *s)
{
    uint8_t buf[SHM_HDR + LCD_W * ST7558_PAGES];
    int p;

    if (s->shm_fd < 0) {
        return;
    }
    s->seq++;
    memset(buf, 0, SHM_HDR);
    memcpy(buf, "C123LCD", 8);
    buf[8] = LCD_W; buf[9] = 0;
    buf[10] = LCD_H; buf[11] = 0;
    buf[12] = ST7558_PAGES; buf[13] = 0;
    buf[14] = (s->inverse ? 1 : 0) | (s->off ? 2 : 0);
    buf[16] = s->seq & 0xff; buf[17] = (s->seq >> 8) & 0xff;
    buf[18] = (s->seq >> 16) & 0xff; buf[19] = (s->seq >> 24) & 0xff;
    for (p = 0; p < ST7558_PAGES; p++) {
        memcpy(buf + SHM_HDR + p * LCD_W, s->ram + p * ST7558_COLS, LCD_W);
    }
    if (pwrite(s->shm_fd, buf, sizeof(buf), 0) != (ssize_t)sizeof(buf)) {
        fprintf(stderr, "[st7558] ecriture de %s : %s\n", s->shm_path, strerror(errno));
    }
}

static uint8_t st7558_recv(I2CSlave *i2c)
{
    return 0xff;
}

static int st7558_send(I2CSlave *i2c, uint8_t data)
{
    St7558State *s = ST7558(i2c);

    switch (s->mode) {
    case MODE_IDLE:
        s->mode = (data & 0x40) ? MODE_DATA : MODE_CMD;
        break;
    case MODE_DATA:
        if (s->page < ST7558_PAGES && s->x < ST7558_COLS) {
            s->ram[s->page * ST7558_COLS + s->x] = data;
            s->dirty = s->redraw = true;
        }
        s->x++;
        break;
    case MODE_CMD:
        if (data & 0x80) {
            s->x = data & 0x7f;
        } else if ((data & 0xf0) == 0x40) {
            s->page = data & 0x0f;
        } else if ((data & 0xfe) == 0x20) {
            s->ext = data & 1;
        } else if (!s->ext) {
            switch (data) {
            case 0x0c: s->inverse = false; s->off = false; s->dirty = s->redraw = true; break;
            case 0x0d: s->inverse = true;  s->off = false; s->dirty = s->redraw = true; break;
            case 0x08: s->off = true; s->dirty = s->redraw = true; break;
            default: break;
            }
        }
        break;
    }
    return 0;
}

static int st7558_event(I2CSlave *i2c, enum i2c_event event)
{
    St7558State *s = ST7558(i2c);

    switch (event) {
    case I2C_START_SEND:
    case I2C_START_RECV:
        s->mode = MODE_IDLE;
        break;
    case I2C_FINISH:
        s->mode = MODE_IDLE;
        if (s->dirty) {
            s->dirty = false;
            st7558_export(s);
        }
        break;
    case I2C_NACK:
        break;
    default:
        return -1;
    }
    return 0;
}

static void st7558_update_display(void *opaque)
{
    St7558State *s = opaque;
    DisplaySurface *surface = qemu_console_surface(s->con);
    uint32_t on, offc;
    int x, y, k, stride;
    uint8_t *base;

    if (!s->redraw || surface_bits_per_pixel(surface) != 32) {
        return;
    }
    /* vert-gris du LCD du C123 : points sombres sur fond clair */
    on = s->inverse ? 0xffc8d8b0u : 0xff1a2a1au;
    offc = s->inverse ? 0xff1a2a1au : 0xffc8d8b0u;
    if (s->off) {
        on = offc = 0xff9aa892u;
    }
    base = surface_data(surface);
    stride = surface_stride(surface);
    for (y = 0; y < LCD_H; y++) {
        const uint8_t *src = s->ram + ST7558_COLS * (y >> 3);
        uint8_t mask = 1 << (y & 7);
        uint32_t *row = (uint32_t *)(base + (size_t)y * MAGNIFY * stride);
        for (x = 0; x < LCD_W; x++) {
            uint32_t c = (src[x] & mask) ? on : offc;
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

static void st7558_invalidate_display(void *opaque)
{
    St7558State *s = opaque;
    s->redraw = true;
}

static const GraphicHwOps st7558_ops = {
    .invalidate = st7558_invalidate_display,
    .gfx_update = st7558_update_display,
};

static void st7558_realize(DeviceState *dev, Error **errp)
{
    St7558State *s = ST7558(dev);
    const char *env = getenv("CALYPSO_LCD_SHM");

    s->shm_fd = -1;
    s->shm_path = g_strdup(env && *env ? env : SHM_DEFAULT);
    if (strcmp(s->shm_path, "none")) {
        s->shm_fd = open(s->shm_path, O_RDWR | O_CREAT | O_TRUNC, 0666);
        if (s->shm_fd < 0) {
            fprintf(stderr, "[st7558] %s : %s (pas d'image partagee)\n",
                    s->shm_path, strerror(errno));
        } else {
            fchmod(s->shm_fd, 0666);
            if (ftruncate(s->shm_fd, SHM_HDR + LCD_W * ST7558_PAGES) < 0) {
                fprintf(stderr, "[st7558] ftruncate %s : %s\n", s->shm_path, strerror(errno));
            }
            st7558_export(s);
            fprintf(stderr, "[st7558] ecran C123 96x65 : image partagee %s\n", s->shm_path);
        }
    }
    s->con = graphic_console_init(dev, 0, &st7558_ops, s);
    qemu_console_resize(s->con, LCD_W * MAGNIFY, LCD_H * MAGNIFY);
}

static void st7558_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    I2CSlaveClass *k = I2C_SLAVE_CLASS(klass);

    dc->realize = st7558_realize;
    dc->desc = "Sitronix ST7558 LCD 96x65 (Motorola C123)";
    k->event = st7558_event;
    k->recv = st7558_recv;
    k->send = st7558_send;
}

static const TypeInfo st7558_info = {
    .name          = TYPE_ST7558,
    .parent        = TYPE_I2C_SLAVE,
    .instance_size = sizeof(St7558State),
    .class_init    = st7558_class_init,
};

static void st7558_register_types(void)
{
    type_register_static(&st7558_info);
}

type_init(st7558_register_types)
