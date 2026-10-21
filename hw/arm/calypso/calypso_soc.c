/* SPDX-License-Identifier: GPL-2.0-or-later */
#include "qemu/osdep.h"
#include "hw/i2c/i2c.h"
#include "qapi/error.h"
#include "hw/sysbus.h"
#include "hw/irq.h"
#include "hw/qdev-properties.h"
#include "hw/qdev-properties-system.h"
#include "hw/char/serial.h"
#include "chardev/char.h"
#include "exec/address-spaces.h"
#include "sysemu/sysemu.h"
#include "sysemu/runstate.h"
#include "hw/core/cpu.h"
#include "hw/arm/calypso/calypso_soc.h"
#include "hw/arm/calypso/calypso_trx.h"
#include "hw/arm/calypso/calypso_uart.h"

CalypsoUARTState *g_uart_modem;
CalypsoUARTState *g_uart_irda;

#define CALYPSO_IRAM_BASE     0x00800000
#define CALYPSO_IRAM_SIZE     (256 * 1024)
#define CALYPSO_INTH_BASE     0xFFFFFA00
#define CALYPSO_TIMER1_BASE   0xFFFE3800
/* [2026-10-20] TIMER2 est a TIMER1 + 0x3000 (firmware calypso/timer.c TIMER2_OFFSET) : 0xFFFE6800, pas 0xFFFE3C00.
 * Mappe au mauvais endroit, timer_init() ecrivait dans le vide : jiffies restait a 0, timer_irq() ne tournait
 * jamais, donc ni keypad_poll() (clavier du C123 muet dans rssi/menu) ni les osmo_timers du firmware. */
#define CALYPSO_TIMER2_BASE   0xFFFE6800
#define CALYPSO_SPI_BASE      0xFFFE3000
#define CALYPSO_I2C_BASE      0xFFFE1800
#define CALYPSO_KEYPAD_BASE   0xFFFE4800    /* ARMIO : GPIO + clavier (calypso_armio.c) */
#define CALYPSO_UWIRE_BASE    0xFFFE4000    /* uWire : ecran couleur du C139 sur CS0 (calypso_uwire.c) */
#define CALYPSO_I2C_CTRL_BASE 0xFFFE2800    /* le vrai I2C (calypso_i2c_ctrl.c) ; 0xFFFE1800 est la RTC */
#define CALYPSO_IRQ_KEYPAD_GPIO 8
#define CALYPSO_UART_IRDA     0xFFFF5000
#define CALYPSO_UART_MODEM    0xFFFF5800
#define CALYPSO_CNTL_BASE     0xFFFFFD00
#define CALYPSO_RHEA_DMA_BASE 0xFFFFFC00

static uint64_t stub_read(void *o, hwaddr a, unsigned s)
{
    return 0;
}

static void stub_write(void *o, hwaddr a, uint64_t v, unsigned s)
{
}

static const MemoryRegionOps stub8_ops = {
    .read = stub_read,
    .write = stub_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
    .impl = { .min_access_size = 1, .max_access_size = 1 },
};

static const MemoryRegionOps stub16_ops = {
    .read = stub_read,
    .write = stub_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
    .impl = { .min_access_size = 2, .max_access_size = 2 },
};

static uint64_t cntl_read(void *opaque, hwaddr offset, unsigned size)
{
    CalypsoSoCState *s = CALYPSO_SOC(opaque);
    if (offset == 4) {
        return s->cntl_rst;
    }
    return offset == 0 ? s->extra_conf : 0;
}

static void cntl_write(void *opaque, hwaddr offset, uint64_t value, unsigned size)
{
    CalypsoSoCState *s = CALYPSO_SOC(opaque);
    if (offset == 4) {
        /* CNTL_RST: dsp_pre_boot() asserts then releases RESET_DSP (bit 1).
         * On silicon that restarts the DSP boot ROM, which then writes IDLE
         * into BL_CMD_STATUS. */
        s->cntl_rst = (uint16_t)value;
        calypso_trx_dsp_reset_line((value & 0x2) != 0);
        return;
    }
    if (offset != 0) {
        return;
    }
    s->extra_conf = (uint16_t)value;
    bool bootrom_enabled = (value >> 8) & 3;
    MemoryRegion *sysmem = get_system_memory();
    /* [2026-10-21] Sur un C1xx le boot ROM est hors circuit par cablage
     * (nIBOOT) : la flash reste a 0 quoi que le firmware ecrive ici. Magnetite
     * ecrit 0x40 dans CNTL_ARM_CLK au demarrage ; sans ce garde l'IRAM (zeros =
     * NOP ARM) recouvrait la flash et le CPU glissait jusqu'a 0x40000. */
    if (s->flash_boot) {
        return;
    }
    if (!bootrom_enabled && !s->iram_at_zero) {
        memory_region_init_alias(&s->iram_alias, OBJECT(s), "calypso.iram_at_zero",
                                 &s->iram, 0, CALYPSO_IRAM_SIZE);
        memory_region_add_subregion_overlap(sysmem, 0x00000000, &s->iram_alias, 1);
        s->iram_at_zero = true;
    } else if (bootrom_enabled && s->iram_at_zero) {
        memory_region_del_subregion(sysmem, &s->iram_alias);
        object_unparent(OBJECT(&s->iram_alias));
        s->iram_at_zero = false;
    }
}

static const MemoryRegionOps cntl_ops = {
    .read = cntl_read,
    .write = cntl_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
    .impl = { .min_access_size = 2, .max_access_size = 2 },
};

/* [2026-10-21] Chien de garde : dar_reset() (seul appelant de WATCHDOG_TIM_MODE = 0xFFFFF804)
 * ecrit 0x80F5 puis attend le reset materiel de l'ARM. Sans ce reset, le firmware boucle
 * dans dar_reset (« gel »). On emule le reset : redemarrage de la machine (flash conservee). */
static uint64_t wdt_read(void *opaque, hwaddr off, unsigned size)
{
    return 0;
}

static void wdt_write(void *opaque, hwaddr off, uint64_t val, unsigned size)
{
    if (off == 0x04 && (val & 0xffff) == 0x80F5) {
        fprintf(stderr, "[wdt] chien de garde : 0x80F5 -> reset de l'ARM\n");
        cpu_reset(first_cpu);   /* reset du CPU ARM : retour au vecteur de reset, boot repart */
    }
}

static const MemoryRegionOps wdt_ops = {
    .read = wdt_read,
    .write = wdt_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
    .valid = { .min_access_size = 1, .max_access_size = 4 },
};

static void add_stub(MemoryRegion *sys, const char *name, hwaddr base,
                     const MemoryRegionOps *ops, uint64_t size)
{
    MemoryRegion *mr = g_new(MemoryRegion, 1);
    memory_region_init_io(mr, NULL, ops, NULL, name, size);
    memory_region_add_subregion(sys, base, mr);
}

static void init_uart(CalypsoSoCState *s, CalypsoUARTState *uart, const char *label,
                      int serial_index, hwaddr base, qemu_irq irq, Error **errp)
{
    Chardev *chr = qemu_chr_find(label);
    if (!chr) {
        chr = serial_hd(serial_index);
    }
    object_initialize_child(OBJECT(s), label, uart, TYPE_CALYPSO_UART);
    qdev_prop_set_string(DEVICE(uart), "label", label);
    if (chr) {
        qdev_prop_set_chr(DEVICE(uart), "chardev", chr);
    }
    if (!sysbus_realize(SYS_BUS_DEVICE(uart), errp)) {
        return;
    }
    sysbus_mmio_map(SYS_BUS_DEVICE(uart), 0, base);
    sysbus_connect_irq(SYS_BUS_DEVICE(uart), 0, irq);
}

static void calypso_soc_realize(DeviceState *dev, Error **errp)
{
    CalypsoSoCState *s = CALYPSO_SOC(dev);
    SysBusDevice *sbd = SYS_BUS_DEVICE(dev);
    MemoryRegion *sysmem = get_system_memory();
    Error *err = NULL;

    memory_region_init_ram(&s->iram, OBJECT(dev), "calypso.iram", CALYPSO_IRAM_SIZE,
                           &error_fatal);
    memory_region_add_subregion(sysmem, CALYPSO_IRAM_BASE, &s->iram);

    object_initialize_child(OBJECT(dev), "inth", &s->inth, TYPE_CALYPSO_INTH);
    if (!sysbus_realize(SYS_BUS_DEVICE(&s->inth), &err)) {
        error_propagate(errp, err);
        return;
    }
    sysbus_mmio_map(SYS_BUS_DEVICE(&s->inth), 0, CALYPSO_INTH_BASE);
    sysbus_pass_irq(sbd, SYS_BUS_DEVICE(&s->inth));
#define INTH_IRQ(n) qdev_get_gpio_in(DEVICE(&s->inth), (n))

    object_initialize_child(OBJECT(dev), "timer1", &s->timer1, TYPE_CALYPSO_TIMER);
    if (!sysbus_realize(SYS_BUS_DEVICE(&s->timer1), &err)) {
        error_propagate(errp, err);
        return;
    }
    sysbus_mmio_map(SYS_BUS_DEVICE(&s->timer1), 0, CALYPSO_TIMER1_BASE);
    sysbus_connect_irq(SYS_BUS_DEVICE(&s->timer1), 0, INTH_IRQ(CALYPSO_IRQ_TIMER1));
    calypso_timer_register_lost(DEVICE(&s->timer1));

    object_initialize_child(OBJECT(dev), "timer2", &s->timer2, TYPE_CALYPSO_TIMER);
    if (!sysbus_realize(SYS_BUS_DEVICE(&s->timer2), &err)) {
        error_propagate(errp, err);
        return;
    }
    sysbus_mmio_map(SYS_BUS_DEVICE(&s->timer2), 0, CALYPSO_TIMER2_BASE);
    sysbus_connect_irq(SYS_BUS_DEVICE(&s->timer2), 0, INTH_IRQ(CALYPSO_IRQ_TIMER2));

    DeviceState *i2c = qdev_new("calypso-i2c");
    sysbus_realize_and_unref(SYS_BUS_DEVICE(i2c), &error_fatal);
    sysbus_mmio_map(SYS_BUS_DEVICE(i2c), 0, CALYPSO_I2C_BASE);

    object_initialize_child(OBJECT(dev), "spi", &s->spi, TYPE_CALYPSO_SPI);
    if (!sysbus_realize(SYS_BUS_DEVICE(&s->spi), &err)) {
        error_propagate(errp, err);
        return;
    }
    sysbus_mmio_map(SYS_BUS_DEVICE(&s->spi), 0, CALYPSO_SPI_BASE);
    sysbus_connect_irq(SYS_BUS_DEVICE(&s->spi), 0, INTH_IRQ(CALYPSO_IRQ_SPI));

    init_uart(s, &s->uart_modem, "modem", 0, CALYPSO_UART_MODEM,
              INTH_IRQ(CALYPSO_IRQ_UART_MODEM), &err);
    if (err) {
        error_propagate(errp, err);
        return;
    }
    g_uart_modem = &s->uart_modem;

    init_uart(s, &s->uart_irda, "irda", 1, CALYPSO_UART_IRDA,
              INTH_IRQ(CALYPSO_IRQ_UART_IRDA), &err);
    if (err) {
        error_propagate(errp, err);
        return;
    }
    g_uart_irda = &s->uart_irda;

    qemu_irq *irqs = g_new0(qemu_irq, CALYPSO_NUM_IRQS);
    for (int i = 0; i < CALYPSO_NUM_IRQS; i++) {
        irqs[i] = INTH_IRQ(i);
    }
    calypso_trx_init(sysmem, irqs);
#undef INTH_IRQ

    /* [2026-10-19] ARMIO + clavier matriciel du C123 (calypso_armio.c), IRQ 8,
     * et le vrai contrôleur I2C (calypso_i2c_ctrl.c) avec l'ecran ST7558 (0x3c,
     * calypso_st7558.c) dessous : rssi.highram.elf, menu, trx affichent et
     * lisent les touches. Avant : bouchon clavier a 0xFF, I2C jete au catchall. */
    {
        DeviceState *armio = qdev_new("calypso-armio");
        sysbus_realize_and_unref(SYS_BUS_DEVICE(armio), &error_fatal);
        sysbus_mmio_map(SYS_BUS_DEVICE(armio), 0, CALYPSO_KEYPAD_BASE);
        sysbus_connect_irq(SYS_BUS_DEVICE(armio), 0,
                           qdev_get_gpio_in(DEVICE(&s->inth), CALYPSO_IRQ_KEYPAD_GPIO));

        DeviceState *i2cc = qdev_new("calypso-i2c-ctrl");
        sysbus_realize_and_unref(SYS_BUS_DEVICE(i2cc), &error_fatal);
        sysbus_mmio_map(SYS_BUS_DEVICE(i2cc), 0, CALYPSO_I2C_CTRL_BASE);
        /* [2026-10-21] CALYPSO_LCD=c139 : l'ecran couleur 96x64 du C139 sur
         * uWire CS0 (FreeCalypso Magnetite, cible c139) a la place du ST7558 du
         * C123 sur I2C. Le controleur uWire est la dans les deux cas. */
        const char *lcd = getenv("CALYPSO_LCD");
        bool c139 = lcd && !strcmp(lcd, "c139");
        if (!c139) {
            i2c_slave_create_simple(I2C_BUS(qdev_get_child_bus(i2cc, "i2c")), "st7558", 0x3c);
        }
        DeviceState *uwire = qdev_new("calypso-uwire");
        qdev_prop_set_bit(uwire, "c139-lcd", c139);
        sysbus_realize_and_unref(SYS_BUS_DEVICE(uwire), &error_fatal);
        sysbus_mmio_map(SYS_BUS_DEVICE(uwire), 0, CALYPSO_UWIRE_BASE);
    }
    /* [2026-10-20] plus de bouchon en 0xFFFE6800 : c'est TIMER2 (CALYPSO_TIMER2_BASE), mappe plus haut */
    add_stub(sysmem, "calypso.mmio_80xx", 0xFFFE8000, &stub8_ops, 0x100);
    add_stub(sysmem, "calypso.conf", 0xFFFEF000, &stub16_ops, 0x100);
    add_stub(sysmem, "calypso.dpll", 0xFFFF9800, &stub16_ops, 0x100);
    add_stub(sysmem, "calypso.mmio_f0xx", 0xFFFFF000, &stub16_ops, 0x100);
    add_stub(sysmem, "calypso.wdt", 0xFFFFF800, &wdt_ops, 0x100);
    add_stub(sysmem, "calypso.rhea", 0xFFFFF900, &stub16_ops, 0x100);
    add_stub(sysmem, "calypso.clkm", 0xFFFFFB00, &stub16_ops, 0x100);
    add_stub(sysmem, "calypso.rhea_dma", CALYPSO_RHEA_DMA_BASE, &stub16_ops, 0x100);
    add_stub(sysmem, "calypso.dio", 0xFFFFFF00, &stub8_ops, 0x100);

    memory_region_init_io(&s->cntl_iomem, OBJECT(dev), &cntl_ops, s, "calypso.cntl", 0x100);
    memory_region_add_subregion(sysmem, CALYPSO_CNTL_BASE, &s->cntl_iomem);
    s->extra_conf = 0x0300;
    s->iram_at_zero = false;

    MemoryRegion *catchall = g_new(MemoryRegion, 1);
    memory_region_init_io(catchall, NULL, &stub8_ops, NULL, "calypso.catchall", 0x100000);
    memory_region_add_subregion_overlap(sysmem, 0xFFF00000, catchall, -1);
}

static void calypso_soc_class_init(ObjectClass *oc, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);
    dc->realize = calypso_soc_realize;
    dc->user_creatable = false;
}

static const TypeInfo calypso_soc_type_info = {
    .name = TYPE_CALYPSO_SOC,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(CalypsoSoCState),
    .class_init = calypso_soc_class_init,
};

static void calypso_soc_register_types(void)
{
    type_register_static(&calypso_soc_type_info);
}

type_init(calypso_soc_register_types)
