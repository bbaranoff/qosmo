/*
 * calypso_xio.c - two DSP XIO windows: API Control (F900) and the DSP INTH
 * (FA00). Both used to fall through into the PORTR/PORTW no-op, so writes such
 * as the 0x0c00 the firmware stores into the DSP INTH were dropped.
 *
 * The DMA routine at PROM0 0xa636-0xa660 brackets its DMA2_CTRL accesses with
 * F900 bit 2 (BRIDGE_CLK_EN):
 *
 *   0xa637  orm   *(0x3fdc), #0x0004      ; set bit 2
 *   0xa63a  portw *(0x3fdc), 0xf900       ; -> XIO:F900  = API Control
 *   0xa640  portr *(0x4356), 0xfc28       ; read DMA2_CTRL
 *   0xa643  andm  *(0x4356), #0xfffe      ; clear ENABLE (disarm, no polling)
 *   0xa646  portw *(0x4356), 0xfc28       ; write DMA2_CTRL back
 *   0xa652  portr *(0x0011), 0xfc28       ; re-read DMA2_CTRL
 *   0xa655  st    *(0x4356), #0x0c00
 *   0xa658  portw *(0x4356), 0xfa01       ; -> XIO:FA01  = DSP INTH
 *   0xa65b  andm  *(0x3fdc), #0xfffb      ; clear bit 2
 *   0xa65e  portw *(0x3fdc), 0xf900       ; -> XIO:F900
 *
 * That is exactly the sequence prescribed by CAL207 11.3.5: "DMA1_CTRL is only
 * writable and readable when the DMA controller clock runs [...] it must set
 * the BRIDGE_CLK_EN bit of DSP API configuration register to '1'".
 *
 * This module INVENTS NO SEMANTICS: it stores, returns and logs. The DSP INTH
 * bit fields are documented only by their window (CAL207 7.2.2, "INTH
 * FA00-FAFF"), so raw values and their bit decomposition are printed without
 * naming fields we cannot name.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */
#include "qemu/osdep.h"
#include "hw/arm/calypso/calypso_debug.h"
#include "hw/arm/calypso/calypso_xio.h"

#include <stdio.h>
#include <stdlib.h>

#define XIO_APIC_BASE  0xF900   /* API Control  (CAL207 7.2.2)         */
#define XIO_APIC_END   0xF9FF
#define XIO_INTH_BASE  0xFA00   /* DSP INTH     (CAL207 7.2.2, 3.7.6)  */
#define XIO_INTH_END   0xFAFF

/* ---- API_CONF @ XIO:F900 - CAL207 9.1, Table 14 (3 R/W bits) ----------------
 *   bit 0                 reserved, must be 0
 *   bit 1  API_HOM        0 = API in SAM mode, 1 = API in HOM mode
 *   bit 2  BRIDGE_CLK_EN  1 = force the ARM-RHEA bridge and DMA clock
 * CAL207 gives reset as "???? ???? ???? ?010", i.e. API_HOM=1 (HOM) out of reset.
 *
 * WARNING: the two TI documents contradict each other. CAL000 7.2.1 states "SAM
 * mode is the default configuration when the DSP exits from a reset phase".
 * Unresolved. The firmware writes API_CONF=0x0000 (SAM) early in boot anyway
 * (0xb416, 0xb36f), as if it trusted neither.
 *
 * CAL207 9.1 on BRIDGE_CLK_EN: "each time the DSP software want to access to
 * these registers [DMA1_CTRL/DMA2_CTRL], it must set the BRIDGE_CLK_EN bit to
 * '1', then access to these registers and then set back BRIDGE_CLK_EN bit to '0'
 * in order to conserve power." The firmware does exactly that (0xb3b1 -> 0xb3c6).
 *
 * CAL207 9.2.3: the DSP cannot reach the APIC directly; SMODE and HINT are read
 * and written through bits 2 and 3 of BSCR (MMR data 0x0029). Hence the
 * `orm *(0x0029), #0x0004` at 0xa68d, right before the mode switch. */
#define APIC_HOM           0x0002
#define APIC_BRIDGE_CLK_EN 0x0004

/* ---- DSP INTH @ XIO:FA00/FA01 - CAL207 15.2 --------------------------------
 * FA00 CNTRL_REG (13 bits): edge/level assignment of the 12 channels.
 *      bits 11:0  CHx  1 = channel x is EDGE-triggered, 0 = LEVEL
 *      bit 12     INT4 switch: 0 = channel 4 -> INT4N (0x58), 1 -> nNMI (0x04)
 * FA01 CLEAR_REG (12 bits, W): writing 1 in a bit clears that channel (required
 *      for channels assigned as LEVEL and shared).
 * Channel N maps to INTNn, confirmed by measurement: the firmware writes 0x0380
 * (channels 7, 8, 9 as edge) and CAL207 15.1 marks exactly INT7n, INT8n and
 * INT9n as "edge". */
#define INTH_CNTRL_REG  0x00
#define INTH_CLEAR_REG  0x01
#define INTH_INT4_SWITCH 0x1000

static uint16_t apic[0x100];
static uint16_t inth[0x100];
static bool     g_init;

static bool xio_on(void)
{
    static int on = -1;
    if (on < 0) {
        on = calypso_gate("CALYPSO_XIO_MISC", 1);
        if (!on)
            fprintf(stderr, "[xio] CALYPSO_XIO_MISC=0 : F900/FA00 redeviennent "
                    "des no-op (comportement d'avant le 03/08)\n");
    }
    return on != 0;
}

static void xio_init(void)
{
    if (g_init)
        return;
    g_init = true;
    fprintf(stderr, "[xio] fenetres XIO ouvertes : API Control @0xF900 (bit2 = "
            "BRIDGE_CLK_EN, note du §11.3.5) et INTH du DSP @0xFA00 (§3.7.6). "
            "ENREGISTREMENT SEUL : aucune semantique n'est inventee.\n");
}

static void bits16(uint16_t v, char out[24])
{
    int k = 0;
    for (int b = 15; b >= 0; b--) {
        out[k++] = (v & (1u << b)) ? '1' : '0';
        if (b == 12 || b == 8 || b == 4)
            out[k++] = ' ';
    }
    out[k] = 0;
}

/* Current API RAM mode as the DSP programmed it (CAL207 9.1 bit 1). */
bool calypso_xio_api_hom(void)
{
    return (apic[0x00] & APIC_HOM) != 0;
}

bool calypso_xio_misc(bool write, uint16_t pa, uint16_t *val, uint16_t pc)
{
    bool is_apic = (pa >= XIO_APIC_BASE && pa <= XIO_APIC_END);
    bool is_inth = (pa >= XIO_INTH_BASE && pa <= XIO_INTH_END);
    if (!is_apic && !is_inth)
        return false;
    if (!xio_on())
        return false;
    xio_init();

    uint16_t *bank = is_apic ? apic : inth;
    unsigned  off  = pa & 0xFF;
    const char *nom = is_apic ? "API-CTRL" : "INTH-DSP";

    if (!write) {
        *val = bank[off];
        static unsigned nr = 0;
        if (nr++ < 30)
            fprintf(stderr, "[xio] %s PORTR PA=0x%04x -> 0x%04x PC=0x%04x\n",
                    nom, pa, *val, pc);
        return true;
    }

    bank[off] = *val;

    char b[24];
    bits16(*val, b);
    /* Dedupe on the (register, value, PC) triple: each combination is logged
     * once, then summarised periodically. Deduping on "value differs from the
     * previous one" does not work here - the firmware toggles API_CONF between
     * 0x0002 (HOM) and 0x0000 (SAM) every frame, so that test is always true:
     * 2238 lines out of 20000, 11% of the QEMU log, for two alternating
     * values. */
    static struct { uint16_t pa, val, pc; unsigned long long n; } seen[32];
    static int nseen = 0;
    int i, k = -1;
    for (i = 0; i < nseen; i++)
        if (seen[i].pa == pa && seen[i].val == *val && seen[i].pc == pc) { k = i; break; }
    if (k >= 0) {
        if (++seen[k].n % 5000 == 0)
            fprintf(stderr, "[xio] %s PA=0x%04x <- 0x%04x PC=0x%04x × %llu\n",
                    nom, pa, *val, pc, seen[k].n);
        return true;
    }
    if (nseen < 32) { seen[nseen].pa = pa; seen[nseen].val = *val;
                      seen[nseen].pc = pc; seen[nseen].n = 1; nseen++; }
    {
        if (is_apic) {
            fprintf(stderr, "[xio] API_CONF PORTW PA=0x%04x <- 0x%04x [%s] "
                    "API_HOM=%d(%s) BRIDGE_CLK_EN=%d PC=0x%04x\n",
                    pa, *val, b, !!(*val & APIC_HOM),
                    (*val & APIC_HOM) ? "HOM: API reservee ARM/DMA"
                                      : "SAM: acces partage",
                    !!(*val & APIC_BRIDGE_CLK_EN), pc);
        } else {
            if (off == INTH_CNTRL_REG) {
                fprintf(stderr, "[xio] *** INTH CNTRL_REG <- 0x%04x [%s] : canaux en "
                        "FRONT =", *val, b);
                for (int ch = 0; ch <= 11; ch++)
                    if (*val & (1u << ch))
                        fprintf(stderr, " INT%dn", ch);
                fprintf(stderr, " ; INT4_switch=%d (%s) PC=0x%04x\n",
                        !!(*val & INTH_INT4_SWITCH),
                        (*val & INTH_INT4_SWITCH) ? "canal 4 -> nNMI"
                                                  : "canal 4 -> INT4N",
                        pc);
            } else {
                fprintf(stderr, "[xio] *** INTH CLEAR_REG <- 0x%04x [%s] : efface les "
                        "canaux", *val, b);
                for (int ch = 0; ch <= 11; ch++)
                    if (*val & (1u << ch))
                        fprintf(stderr, " INT%dn", ch);
                fprintf(stderr, " PC=0x%04x\n", pc);
            }
        }
    }
    return true;
}
