/* SPDX-License-Identifier: GPL-2.0-or-later */
#include "qemu/osdep.h"
#include "qapi/error.h"
#include "hw/boards.h"
#include "hw/sysbus.h"
#include "hw/irq.h"
#include "hw/loader.h"
#include "hw/block/flash.h"
#include "sysemu/sysemu.h"
#include "sysemu/blockdev.h"
#include "sysemu/block-backend.h"
#include "qemu/error-report.h"
#include "exec/address-spaces.h"
#include "elf.h"
#include "target/arm/cpu.h"
#include "hw/arm/calypso/calypso_soc.h"
#include "hw/arm/calypso/calypso_l1_ops.h"

#define CALYPSO_XRAM_BASE     0x01000000
#define CALYPSO_XRAM_SIZE     (8 * 1024 * 1024)
#define CALYPSO_FLASH_BASE    0x00000000
#define CALYPSO_FLASH_SIZE    (4 * 1024 * 1024)
#define CALYPSO_FLASH_SECTOR  (64 * 1024)
#define FLASH_MFR_INTEL       0x0089
#define FLASH_DEV_28F320J3    0x0018

typedef struct CalypsoMachineState {
    MachineState parent;
    ARMCPU *cpu;
    CalypsoSoCState soc;
    MemoryRegion xram;
    MemoryRegion bootrom;
} CalypsoMachineState;

#define TYPE_CALYPSO_MACHINE MACHINE_TYPE_NAME("calypso")
OBJECT_DECLARE_SIMPLE_TYPE(CalypsoMachineState, CALYPSO_MACHINE)

static void calypso_machine_init(MachineState *machine)
{
    CalypsoMachineState *s = CALYPSO_MACHINE(machine);
    MemoryRegion *sysmem = get_system_memory();
    Error *err = NULL;

    Object *cpuobj = object_new(machine->cpu_type);
    s->cpu = ARM_CPU(cpuobj);
    if (!qdev_realize(DEVICE(cpuobj), NULL, &err)) {
        error_report_err(err);
        exit(1);
    }

    object_initialize_child(OBJECT(machine), "soc", &s->soc, TYPE_CALYPSO_SOC);
    if (!sysbus_realize(SYS_BUS_DEVICE(&s->soc), &err)) {
        error_report_err(err);
        exit(1);
    }
    sysbus_connect_irq(SYS_BUS_DEVICE(&s->soc), 0,
                       qdev_get_gpio_in(DEVICE(&s->cpu->parent_obj), ARM_CPU_IRQ));
    sysbus_connect_irq(SYS_BUS_DEVICE(&s->soc), 1,
                       qdev_get_gpio_in(DEVICE(&s->cpu->parent_obj), ARM_CPU_FIQ));

    memory_region_init_ram(&s->xram, OBJECT(&s->soc.parent_obj), "calypso.xram",
                           CALYPSO_XRAM_SIZE, &error_fatal);
    memory_region_add_subregion(sysmem, CALYPSO_XRAM_BASE, &s->xram);

    DriveInfo *dinfo = drive_get(IF_PFLASH, 0, 0);
    pflash_cfi01_register(CALYPSO_FLASH_BASE, "calypso.flash", CALYPSO_FLASH_SIZE,
                          dinfo ? blk_by_legacy_dinfo(dinfo) : NULL,
                          CALYPSO_FLASH_SECTOR, 1, FLASH_MFR_INTEL, FLASH_DEV_28F320J3,
                          0, 0, 0);

    uint32_t vectors[16];
    for (int i = 0; i < 8; i++) {
        vectors[i] = 0xe59ff018;
    }
    vectors[8] = 0x00820000;
    for (int i = 9; i < 16; i++) {
        vectors[i] = 0x0080001C + 4 * (i - 9);
    }
    memory_region_init_ram(&s->bootrom, NULL, "calypso.bootrom", sizeof(vectors),
                           &error_fatal);
    memory_region_add_subregion_overlap(sysmem, 0x00000000, &s->bootrom, 1);
    memcpy(memory_region_get_ram_ptr(&s->bootrom), vectors, sizeof(vectors));

    if (machine->kernel_filename) {
        uint64_t entry;
        int ret = load_elf(machine->kernel_filename, NULL, NULL, NULL, &entry, NULL,
                           NULL, NULL, 0, EM_ARM, 1, 0);
        if (ret < 0) {
            ret = load_image_targphys(machine->kernel_filename, CALYPSO_XRAM_BASE,
                                      CALYPSO_XRAM_SIZE);
            if (ret < 0) {
                error_report("firmware illisible : '%s'", machine->kernel_filename);
                exit(1);
            }
            entry = CALYPSO_XRAM_BASE;
        }
        cpu_set_pc(CPU(s->cpu), entry);
    }

    calypso_l1_do_init(machine->kernel_filename);
}

static void calypso_machine_class_init(ObjectClass *oc, void *data)
{
    MachineClass *mc = MACHINE_CLASS(oc);
    /* [2026-09-16] La description annoncait « couche 1 gr-gsm » - heritee de
     * qosmo-grgsm, elle etait fausse des que la base tournait seule ou sous
     * qosmo-dsp. Elle nomme desormais la L1 reellement enregistree, ou
     * « aucune » quand qosmo tourne nu (voir calypso_l1_ops.h). */
    static char desc[96];
    snprintf(desc, sizeof(desc),
             "TI Calypso baseband (firmware osmocom-bb, couche 1 : %s)",
             calypso_l1_name());
    mc->desc = desc;
    mc->init = calypso_machine_init;
    mc->max_cpus = 1;
    mc->default_cpu_type = ARM_CPU_TYPE_NAME("arm946");
    mc->default_ram_size = 0;
}

static const TypeInfo calypso_machine_info = {
    .name = TYPE_CALYPSO_MACHINE,
    .parent = TYPE_MACHINE,
    .instance_size = sizeof(CalypsoMachineState),
    .class_init = calypso_machine_class_init,
};

static void calypso_machine_register_types(void)
{
    type_register_static(&calypso_machine_info);
}

type_init(calypso_machine_register_types)
