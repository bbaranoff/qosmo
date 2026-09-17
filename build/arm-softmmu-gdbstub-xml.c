#include "qemu/osdep.h"
#include "exec/gdbstub.h"

const GDBFeature gdb_static_features[] = {
    {
        "arm-core.xml",
        "<?xml version=\"1.0\"?>\n"
        "<!-- Copyright (C) 2008 Free Software Foundation, Inc.\n"
        "\n"
        "     Copying and distribution of this file, with or without modification,\n"
        "     are permitted in any medium without royalty provided the copyright\n"
        "     notice and this notice are preserved.  -->\n"
        "\n"
        "<!DOCTYPE feature SYSTEM \"gdb-target.dtd\">\n"
        "<feature name=\"org.gnu.gdb.arm.core\">\n"
        "  <reg name=\"r0\" bitsize=\"32\"/>\n"
        "  <reg name=\"r1\" bitsize=\"32\"/>\n"
        "  <reg name=\"r2\" bitsize=\"32\"/>\n"
        "  <reg name=\"r3\" bitsize=\"32\"/>\n"
        "  <reg name=\"r4\" bitsize=\"32\"/>\n"
        "  <reg name=\"r5\" bitsize=\"32\"/>\n"
        "  <reg name=\"r6\" bitsize=\"32\"/>\n"
        "  <reg name=\"r7\" bitsize=\"32\"/>\n"
        "  <reg name=\"r8\" bitsize=\"32\"/>\n"
        "  <reg name=\"r9\" bitsize=\"32\"/>\n"
        "  <reg name=\"r10\" bitsize=\"32\"/>\n"
        "  <reg name=\"r11\" bitsize=\"32\"/>\n"
        "  <reg name=\"r12\" bitsize=\"32\"/>\n"
        "  <reg name=\"sp\" bitsize=\"32\" type=\"data_ptr\"/>\n"
        "  <reg name=\"lr\" bitsize=\"32\"/>\n"
        "  <reg name=\"pc\" bitsize=\"32\" type=\"code_ptr\"/>\n"
        "\n"
        "  <!-- The CPSR is register 25, rather than register 16, because\n"
        "       the FPA registers historically were placed between the PC\n"
        "       and the CPSR in the \"g\" packet.  -->\n"
        "  <reg name=\"cpsr\" bitsize=\"32\" regnum=\"25\"/>\n"
        "</feature>\n",
        "org.gnu.gdb.arm.core",
        (const char * const []) {
            [0] =
                "r0",
            [1] =
                "r1",
            [2] =
                "r2",
            [3] =
                "r3",
            [4] =
                "r4",
            [5] =
                "r5",
            [6] =
                "r6",
            [7] =
                "r7",
            [8] =
                "r8",
            [9] =
                "r9",
            [10] =
                "r10",
            [11] =
                "r11",
            [12] =
                "r12",
            [13] =
                "sp",
            [14] =
                "lr",
            [15] =
                "pc",
            [25] =
                "cpsr",
        },
        26,
    },
    {
        "arm-vfp.xml",
        "<?xml version=\"1.0\"?>\n"
        "<!-- Copyright (C) 2008 Free Software Foundation, Inc.\n"
        "\n"
        "     Copying and distribution of this file, with or without modification,\n"
        "     are permitted in any medium without royalty provided the copyright\n"
        "     notice and this notice are preserved.  -->\n"
        "<!DOCTYPE feature SYSTEM \"gdb-target.dtd\">\n"
        "<feature name=\"org.gnu.gdb.arm.vfp\">\n"
        "  <reg name=\"d0\" bitsize=\"64\" type=\"float\"/>\n"
        "  <reg name=\"d1\" bitsize=\"64\" type=\"float\"/>\n"
        "  <reg name=\"d2\" bitsize=\"64\" type=\"float\"/>\n"
        "  <reg name=\"d3\" bitsize=\"64\" type=\"float\"/>\n"
        "  <reg name=\"d4\" bitsize=\"64\" type=\"float\"/>\n"
        "  <reg name=\"d5\" bitsize=\"64\" type=\"float\"/>\n"
        "  <reg name=\"d6\" bitsize=\"64\" type=\"float\"/>\n"
        "  <reg name=\"d7\" bitsize=\"64\" type=\"float\"/>\n"
        "  <reg name=\"d8\" bitsize=\"64\" type=\"float\"/>\n"
        "  <reg name=\"d9\" bitsize=\"64\" type=\"float\"/>\n"
        "  <reg name=\"d10\" bitsize=\"64\" type=\"float\"/>\n"
        "  <reg name=\"d11\" bitsize=\"64\" type=\"float\"/>\n"
        "  <reg name=\"d12\" bitsize=\"64\" type=\"float\"/>\n"
        "  <reg name=\"d13\" bitsize=\"64\" type=\"float\"/>\n"
        "  <reg name=\"d14\" bitsize=\"64\" type=\"float\"/>\n"
        "  <reg name=\"d15\" bitsize=\"64\" type=\"float\"/>\n"
        "\n"
        "  <reg name=\"fpscr\" bitsize=\"32\" type=\"int\" group=\"float\"/>\n"
        "</feature>\n",
        "org.gnu.gdb.arm.vfp",
        (const char * const []) {
            [0] =
                "d0",
            [1] =
                "d1",
            [2] =
                "d2",
            [3] =
                "d3",
            [4] =
                "d4",
            [5] =
                "d5",
            [6] =
                "d6",
            [7] =
                "d7",
            [8] =
                "d8",
            [9] =
                "d9",
            [10] =
                "d10",
            [11] =
                "d11",
            [12] =
                "d12",
            [13] =
                "d13",
            [14] =
                "d14",
            [15] =
                "d15",
            [16] =
                "fpscr",
        },
        17,
    },
    {
        "arm-vfp3.xml",
        "<?xml version=\"1.0\"?>\n"
        "<!-- Copyright (C) 2008 Free Software Foundation, Inc.\n"
        "\n"
        "     Copying and distribution of this file, with or without modification,\n"
        "     are permitted in any medium without royalty provided the copyright\n"
        "     notice and this notice are preserved.  -->\n"
        "<!DOCTYPE feature SYSTEM \"gdb-target.dtd\">\n"
        "<feature name=\"org.gnu.gdb.arm.vfp\">\n"
        "  <reg name=\"d0\" bitsize=\"64\" type=\"float\"/>\n"
        "  <reg name=\"d1\" bitsize=\"64\" type=\"float\"/>\n"
        "  <reg name=\"d2\" bitsize=\"64\" type=\"float\"/>\n"
        "  <reg name=\"d3\" bitsize=\"64\" type=\"float\"/>\n"
        "  <reg name=\"d4\" bitsize=\"64\" type=\"float\"/>\n"
        "  <reg name=\"d5\" bitsize=\"64\" type=\"float\"/>\n"
        "  <reg name=\"d6\" bitsize=\"64\" type=\"float\"/>\n"
        "  <reg name=\"d7\" bitsize=\"64\" type=\"float\"/>\n"
        "  <reg name=\"d8\" bitsize=\"64\" type=\"float\"/>\n"
        "  <reg name=\"d9\" bitsize=\"64\" type=\"float\"/>\n"
        "  <reg name=\"d10\" bitsize=\"64\" type=\"float\"/>\n"
        "  <reg name=\"d11\" bitsize=\"64\" type=\"float\"/>\n"
        "  <reg name=\"d12\" bitsize=\"64\" type=\"float\"/>\n"
        "  <reg name=\"d13\" bitsize=\"64\" type=\"float\"/>\n"
        "  <reg name=\"d14\" bitsize=\"64\" type=\"float\"/>\n"
        "  <reg name=\"d15\" bitsize=\"64\" type=\"float\"/>\n"
        "  <reg name=\"d16\" bitsize=\"64\" type=\"float\"/>\n"
        "  <reg name=\"d17\" bitsize=\"64\" type=\"float\"/>\n"
        "  <reg name=\"d18\" bitsize=\"64\" type=\"float\"/>\n"
        "  <reg name=\"d19\" bitsize=\"64\" type=\"float\"/>\n"
        "  <reg name=\"d20\" bitsize=\"64\" type=\"float\"/>\n"
        "  <reg name=\"d21\" bitsize=\"64\" type=\"float\"/>\n"
        "  <reg name=\"d22\" bitsize=\"64\" type=\"float\"/>\n"
        "  <reg name=\"d23\" bitsize=\"64\" type=\"float\"/>\n"
        "  <reg name=\"d24\" bitsize=\"64\" type=\"float\"/>\n"
        "  <reg name=\"d25\" bitsize=\"64\" type=\"float\"/>\n"
        "  <reg name=\"d26\" bitsize=\"64\" type=\"float\"/>\n"
        "  <reg name=\"d27\" bitsize=\"64\" type=\"float\"/>\n"
        "  <reg name=\"d28\" bitsize=\"64\" type=\"float\"/>\n"
        "  <reg name=\"d29\" bitsize=\"64\" type=\"float\"/>\n"
        "  <reg name=\"d30\" bitsize=\"64\" type=\"float\"/>\n"
        "  <reg name=\"d31\" bitsize=\"64\" type=\"float\"/>\n"
        "\n"
        "  <reg name=\"fpscr\" bitsize=\"32\" type=\"int\" group=\"float\"/>\n"
        "</feature>\n",
        "org.gnu.gdb.arm.vfp",
        (const char * const []) {
            [0] =
                "d0",
            [1] =
                "d1",
            [2] =
                "d2",
            [3] =
                "d3",
            [4] =
                "d4",
            [5] =
                "d5",
            [6] =
                "d6",
            [7] =
                "d7",
            [8] =
                "d8",
            [9] =
                "d9",
            [10] =
                "d10",
            [11] =
                "d11",
            [12] =
                "d12",
            [13] =
                "d13",
            [14] =
                "d14",
            [15] =
                "d15",
            [16] =
                "d16",
            [17] =
                "d17",
            [18] =
                "d18",
            [19] =
                "d19",
            [20] =
                "d20",
            [21] =
                "d21",
            [22] =
                "d22",
            [23] =
                "d23",
            [24] =
                "d24",
            [25] =
                "d25",
            [26] =
                "d26",
            [27] =
                "d27",
            [28] =
                "d28",
            [29] =
                "d29",
            [30] =
                "d30",
            [31] =
                "d31",
            [32] =
                "fpscr",
        },
        33,
    },
    {
        "arm-vfp-sysregs.xml",
        "<?xml version=\"1.0\"?>\n"
        "<!-- Copyright (C) 2021 Linaro Ltd.\n"
        "\n"
        "     Copying and distribution of this file, with or without modification,\n"
        "     are permitted in any medium without royalty provided the copyright\n"
        "     notice and this notice are preserved.\n"
        "\n"
        "     These are A/R profile VFP system registers. Debugger users probably\n"
        "     don't really care about these, but because we used to (incorrectly)\n"
        "     provide them to gdb in the org.gnu.gdb.arm.vfp XML we continue\n"
        "     to do so via this separate XML.\n"
        "     -->\n"
        "<!DOCTYPE feature SYSTEM \"gdb-target.dtd\">\n"
        "<feature name=\"org.qemu.gdb.arm.vfp-sysregs\">\n"
        "  <reg name=\"fpsid\" bitsize=\"32\" type=\"int\" group=\"float\"/>\n"
        "  <reg name=\"fpexc\" bitsize=\"32\" type=\"int\" group=\"float\"/>\n"
        "</feature>\n",
        "org.qemu.gdb.arm.vfp-sysregs",
        (const char * const []) {
            [0] =
                "fpsid",
            [1] =
                "fpexc",
        },
        2,
    },
    {
        "arm-neon.xml",
        "<?xml version=\"1.0\"?>\n"
        "<!-- Copyright (C) 2008 Free Software Foundation, Inc.\n"
        "\n"
        "     Copying and distribution of this file, with or without modification,\n"
        "     are permitted in any medium without royalty provided the copyright\n"
        "     notice and this notice are preserved.  -->\n"
        "<!DOCTYPE feature SYSTEM \"gdb-target.dtd\">\n"
        "<feature name=\"org.gnu.gdb.arm.vfp\">\n"
        "  <vector id=\"neon_uint8x8\" type=\"uint8\" count=\"8\"/>\n"
        "  <vector id=\"neon_uint16x4\" type=\"uint16\" count=\"4\"/>\n"
        "  <vector id=\"neon_uint32x2\" type=\"uint32\" count=\"2\"/>\n"
        "  <vector id=\"neon_float32x2\" type=\"ieee_single\" count=\"2\"/>\n"
        "  <union id=\"neon_d\">\n"
        "    <field name=\"u8\" type=\"neon_uint8x8\"/>\n"
        "    <field name=\"u16\" type=\"neon_uint16x4\"/>\n"
        "    <field name=\"u32\" type=\"neon_uint32x2\"/>\n"
        "    <field name=\"u64\" type=\"uint64\"/>\n"
        "    <field name=\"f32\" type=\"neon_float32x2\"/>\n"
        "    <field name=\"f64\" type=\"ieee_double\"/>\n"
        "  </union>\n"
        "  <vector id=\"neon_uint8x16\" type=\"uint8\" count=\"16\"/>\n"
        "  <vector id=\"neon_uint16x8\" type=\"uint16\" count=\"8\"/>\n"
        "  <vector id=\"neon_uint32x4\" type=\"uint32\" count=\"4\"/>\n"
        "  <vector id=\"neon_uint64x2\" type=\"uint64\" count=\"2\"/>\n"
        "  <vector id=\"neon_float32x4\" type=\"ieee_single\" count=\"4\"/>\n"
        "  <vector id=\"neon_float64x2\" type=\"ieee_double\" count=\"2\"/>\n"
        "  <union id=\"neon_q\">\n"
        "    <field name=\"u8\" type=\"neon_uint8x16\"/>\n"
        "    <field name=\"u16\" type=\"neon_uint16x8\"/>\n"
        "    <field name=\"u32\" type=\"neon_uint32x4\"/>\n"
        "    <field name=\"u64\" type=\"neon_uint64x2\"/>\n"
        "    <field name=\"f32\" type=\"neon_float32x4\"/>\n"
        "    <field name=\"f64\" type=\"neon_float64x2\"/>\n"
        "  </union>\n"
        "  <reg name=\"d0\" bitsize=\"64\" type=\"neon_d\"/>\n"
        "  <reg name=\"d1\" bitsize=\"64\" type=\"neon_d\"/>\n"
        "  <reg name=\"d2\" bitsize=\"64\" type=\"neon_d\"/>\n"
        "  <reg name=\"d3\" bitsize=\"64\" type=\"neon_d\"/>\n"
        "  <reg name=\"d4\" bitsize=\"64\" type=\"neon_d\"/>\n"
        "  <reg name=\"d5\" bitsize=\"64\" type=\"neon_d\"/>\n"
        "  <reg name=\"d6\" bitsize=\"64\" type=\"neon_d\"/>\n"
        "  <reg name=\"d7\" bitsize=\"64\" type=\"neon_d\"/>\n"
        "  <reg name=\"d8\" bitsize=\"64\" type=\"neon_d\"/>\n"
        "  <reg name=\"d9\" bitsize=\"64\" type=\"neon_d\"/>\n"
        "  <reg name=\"d10\" bitsize=\"64\" type=\"neon_d\"/>\n"
        "  <reg name=\"d11\" bitsize=\"64\" type=\"neon_d\"/>\n"
        "  <reg name=\"d12\" bitsize=\"64\" type=\"neon_d\"/>\n"
        "  <reg name=\"d13\" bitsize=\"64\" type=\"neon_d\"/>\n"
        "  <reg name=\"d14\" bitsize=\"64\" type=\"neon_d\"/>\n"
        "  <reg name=\"d15\" bitsize=\"64\" type=\"neon_d\"/>\n"
        "  <reg name=\"d16\" bitsize=\"64\" type=\"neon_d\"/>\n"
        "  <reg name=\"d17\" bitsize=\"64\" type=\"neon_d\"/>\n"
        "  <reg name=\"d18\" bitsize=\"64\" type=\"neon_d\"/>\n"
        "  <reg name=\"d19\" bitsize=\"64\" type=\"neon_d\"/>\n"
        "  <reg name=\"d20\" bitsize=\"64\" type=\"neon_d\"/>\n"
        "  <reg name=\"d21\" bitsize=\"64\" type=\"neon_d\"/>\n"
        "  <reg name=\"d22\" bitsize=\"64\" type=\"neon_d\"/>\n"
        "  <reg name=\"d23\" bitsize=\"64\" type=\"neon_d\"/>\n"
        "  <reg name=\"d24\" bitsize=\"64\" type=\"neon_d\"/>\n"
        "  <reg name=\"d25\" bitsize=\"64\" type=\"neon_d\"/>\n"
        "  <reg name=\"d26\" bitsize=\"64\" type=\"neon_d\"/>\n"
        "  <reg name=\"d27\" bitsize=\"64\" type=\"neon_d\"/>\n"
        "  <reg name=\"d28\" bitsize=\"64\" type=\"neon_d\"/>\n"
        "  <reg name=\"d29\" bitsize=\"64\" type=\"neon_d\"/>\n"
        "  <reg name=\"d30\" bitsize=\"64\" type=\"neon_d\"/>\n"
        "  <reg name=\"d31\" bitsize=\"64\" type=\"neon_d\"/>\n"
        "\n"
        "  <reg name=\"q0\" bitsize=\"128\" type=\"neon_q\"/>\n"
        "  <reg name=\"q1\" bitsize=\"128\" type=\"neon_q\"/>\n"
        "  <reg name=\"q2\" bitsize=\"128\" type=\"neon_q\"/>\n"
        "  <reg name=\"q3\" bitsize=\"128\" type=\"neon_q\"/>\n"
        "  <reg name=\"q4\" bitsize=\"128\" type=\"neon_q\"/>\n"
        "  <reg name=\"q5\" bitsize=\"128\" type=\"neon_q\"/>\n"
        "  <reg name=\"q6\" bitsize=\"128\" type=\"neon_q\"/>\n"
        "  <reg name=\"q7\" bitsize=\"128\" type=\"neon_q\"/>\n"
        "  <reg name=\"q8\" bitsize=\"128\" type=\"neon_q\"/>\n"
        "  <reg name=\"q9\" bitsize=\"128\" type=\"neon_q\"/>\n"
        "  <reg name=\"q10\" bitsize=\"128\" type=\"neon_q\"/>\n"
        "  <reg name=\"q11\" bitsize=\"128\" type=\"neon_q\"/>\n"
        "  <reg name=\"q12\" bitsize=\"128\" type=\"neon_q\"/>\n"
        "  <reg name=\"q13\" bitsize=\"128\" type=\"neon_q\"/>\n"
        "  <reg name=\"q14\" bitsize=\"128\" type=\"neon_q\"/>\n"
        "  <reg name=\"q15\" bitsize=\"128\" type=\"neon_q\"/>\n"
        "\n"
        "  <reg name=\"fpscr\" bitsize=\"32\" type=\"int\" group=\"float\"/>\n"
        "</feature>\n",
        "org.gnu.gdb.arm.vfp",
        (const char * const []) {
            [0] =
                "d0",
            [1] =
                "d1",
            [2] =
                "d2",
            [3] =
                "d3",
            [4] =
                "d4",
            [5] =
                "d5",
            [6] =
                "d6",
            [7] =
                "d7",
            [8] =
                "d8",
            [9] =
                "d9",
            [10] =
                "d10",
            [11] =
                "d11",
            [12] =
                "d12",
            [13] =
                "d13",
            [14] =
                "d14",
            [15] =
                "d15",
            [16] =
                "d16",
            [17] =
                "d17",
            [18] =
                "d18",
            [19] =
                "d19",
            [20] =
                "d20",
            [21] =
                "d21",
            [22] =
                "d22",
            [23] =
                "d23",
            [24] =
                "d24",
            [25] =
                "d25",
            [26] =
                "d26",
            [27] =
                "d27",
            [28] =
                "d28",
            [29] =
                "d29",
            [30] =
                "d30",
            [31] =
                "d31",
            [32] =
                "q0",
            [33] =
                "q1",
            [34] =
                "q2",
            [35] =
                "q3",
            [36] =
                "q4",
            [37] =
                "q5",
            [38] =
                "q6",
            [39] =
                "q7",
            [40] =
                "q8",
            [41] =
                "q9",
            [42] =
                "q10",
            [43] =
                "q11",
            [44] =
                "q12",
            [45] =
                "q13",
            [46] =
                "q14",
            [47] =
                "q15",
            [48] =
                "fpscr",
        },
        49,
    },
    {
        "arm-m-profile.xml",
        "<?xml version=\"1.0\"?>\n"
        "<!-- Copyright (C) 2010-2020 Free Software Foundation, Inc.\n"
        "\n"
        "     Copying and distribution of this file, with or without modification,\n"
        "     are permitted in any medium without royalty provided the copyright\n"
        "     notice and this notice are preserved.  -->\n"
        "\n"
        "<!DOCTYPE feature SYSTEM \"gdb-target.dtd\">\n"
        "<feature name=\"org.gnu.gdb.arm.m-profile\">\n"
        "  <reg name=\"r0\" bitsize=\"32\"/>\n"
        "  <reg name=\"r1\" bitsize=\"32\"/>\n"
        "  <reg name=\"r2\" bitsize=\"32\"/>\n"
        "  <reg name=\"r3\" bitsize=\"32\"/>\n"
        "  <reg name=\"r4\" bitsize=\"32\"/>\n"
        "  <reg name=\"r5\" bitsize=\"32\"/>\n"
        "  <reg name=\"r6\" bitsize=\"32\"/>\n"
        "  <reg name=\"r7\" bitsize=\"32\"/>\n"
        "  <reg name=\"r8\" bitsize=\"32\"/>\n"
        "  <reg name=\"r9\" bitsize=\"32\"/>\n"
        "  <reg name=\"r10\" bitsize=\"32\"/>\n"
        "  <reg name=\"r11\" bitsize=\"32\"/>\n"
        "  <reg name=\"r12\" bitsize=\"32\"/>\n"
        "  <reg name=\"sp\" bitsize=\"32\" type=\"data_ptr\"/>\n"
        "  <reg name=\"lr\" bitsize=\"32\"/>\n"
        "  <reg name=\"pc\" bitsize=\"32\" type=\"code_ptr\"/>\n"
        "  <reg name=\"xpsr\" bitsize=\"32\" regnum=\"25\"/>\n"
        "</feature>\n",
        "org.gnu.gdb.arm.m-profile",
        (const char * const []) {
            [0] =
                "r0",
            [1] =
                "r1",
            [2] =
                "r2",
            [3] =
                "r3",
            [4] =
                "r4",
            [5] =
                "r5",
            [6] =
                "r6",
            [7] =
                "r7",
            [8] =
                "r8",
            [9] =
                "r9",
            [10] =
                "r10",
            [11] =
                "r11",
            [12] =
                "r12",
            [13] =
                "sp",
            [14] =
                "lr",
            [15] =
                "pc",
            [25] =
                "xpsr",
        },
        26,
    },
    {
        "arm-m-profile-mve.xml",
        "<?xml version=\"1.0\"?>\n"
        "<!-- Copyright (C) 2021 Free Software Foundation, Inc.\n"
        "\n"
        "     Copying and distribution of this file, with or without modification,\n"
        "     are permitted in any medium without royalty provided the copyright\n"
        "     notice and this notice are preserved.  -->\n"
        "\n"
        "<!DOCTYPE feature SYSTEM \"gdb-target.dtd\">\n"
        "<feature name=\"org.gnu.gdb.arm.m-profile-mve\">\n"
        "  <flags id=\"vpr_reg\" size=\"4\">\n"
        "    <!-- ARMv8.1-M and MVE: Unprivileged and privileged Access.  -->\n"
        "    <field name=\"P0\" start=\"0\" end=\"15\"/>\n"
        "    <!-- ARMv8.1-M: Privileged Access only.  -->\n"
        "    <field name=\"MASK01\" start=\"16\" end=\"19\"/>\n"
        "    <!-- ARMv8.1-M: Privileged Access only.  -->\n"
        "    <field name=\"MASK23\" start=\"20\" end=\"23\"/>\n"
        "  </flags>\n"
        "  <reg name=\"vpr\" bitsize=\"32\" type=\"vpr_reg\"/>\n"
        "</feature>\n",
        "org.gnu.gdb.arm.m-profile-mve",
        (const char * const []) {
            [0] =
                "vpr",
        },
        1,
    },
    { NULL }
};
