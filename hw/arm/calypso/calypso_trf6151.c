/* SPDX-License-Identifier: GPL-2.0-or-later */
#include "qemu/osdep.h"
#include "hw/arm/calypso/calypso_trf6151.h"

#define SYSTEM_INHERENT_GAIN    71
#define TRF6151_FE_GAIN_LOW      7
#define TRF6151_FE_GAIN_HIGH    27
#define TRF6151_VGA_GAIN_MIN    14
#define RX_VGA_GAIN_SHIFT       11

#define TRF6151_REG_RX           0
#define TRF6151_REG_PLL          1
#define TRF6151_REG_PWR          2
#define TRF6151_REG_ADDR_MASK    0x7
#define TRF6151_BAND_1800        2

#define TRF6151_REG_RX_RESET    0x9E00
#define TRF6151_TSP_DEV          1

static uint16_t g_reg_rx = TRF6151_REG_RX_RESET;
static uint16_t g_reg_pll;
static uint16_t g_reg_pwr;

static int trf6151_gain_from_reg(uint16_t reg_rx)
{
    int gain = 0;
    unsigned vga;

    switch ((reg_rx >> 9) & 3) {
    case 0: gain += TRF6151_FE_GAIN_LOW;  break;
    case 3: gain += TRF6151_FE_GAIN_HIGH; break;
    default:   break;
    }

    vga = (reg_rx >> RX_VGA_GAIN_SHIFT) & 0x1f;
    if (vga < 6) {
        vga = 6;
    }
    gain += TRF6151_VGA_GAIN_MIN + (int)(vga - 6) * 2;

    return gain;
}

void calypso_trf6151_tsp_write(uint8_t dev_idx, uint32_t word)
{
    /* TSPEN1 (osmocom, Pirelli) or TSPEN2: the TI firmware of the Motorola C1xx addresses the
     * TRF6151 on TSP device 2 (tpudrv12.h, TC1_DEVICE_RF = TC1_DEVICE2); both must be accepted
     * for the ARFCN to be known and the fake air to filter. */
    if (dev_idx != TRF6151_TSP_DEV && dev_idx != 2) {
        return;
    }
    switch (word & TRF6151_REG_ADDR_MASK) {
    case TRF6151_REG_RX:
        g_reg_rx = (uint16_t)word;
        break;
    case TRF6151_REG_PLL:
        g_reg_pll = (uint16_t)word;
        break;
    case TRF6151_REG_PWR:
        g_reg_pwr = (uint16_t)word;
        break;
    default:
        break;
    }
}

int calypso_trf6151_arfcn(void)
{
    unsigned a = (g_reg_pll >> 3) & 0x3f;
    unsigned b = ((g_reg_pll >> 9) & 0x7f) + 64;
    unsigned n = b * 64 + a;
    bool high_band = ((g_reg_pwr >> 6) & 3) == TRF6151_BAND_1800;
    unsigned freq10 = high_band ? n * 2 : n;
    if (!g_reg_pll) {
        return -1;
    }
    if (freq10 >= 9350 && freq10 <= 9598) {
        return (int)((freq10 - 9350) / 2);
    }
    if (freq10 >= 9252 && freq10 < 9350) {
        return 1024 - (int)(9350 - freq10) / 2;
    }
    if (freq10 >= 18052 && freq10 <= 18798) {
        return (int)(512 + (freq10 - 18052) / 2);
    }
    if (freq10 >= 19302 && freq10 <= 19898) {
        return (int)(512 + (freq10 - 19302) / 2);
    }
    return -1;
}

/* Realistic air (c54x_exe, CALYPSO_AIR_NIVEAUX): synthesizer state for the `reserve`
 * field of calypso_rf_arfcn.h. bit 15: RX mode (PWR bit 11); bit 14: TX mode (PWR
 * bit 13); bit 13: frequency in an UPLINK band (GSM900/E-GSM 880-915 MHz, DCS
 * 1710-1785 MHz), matching ARFCN in bits 0..10. calypso_trf6151_arfcn() only decodes
 * the downlink bands and returns -1 for these frequencies, so this is how the uplink
 * view of the rssi app (RX tuned to the UL) is told apart from "unknown". */
int calypso_trf6151_rf_flags(void)
{
    unsigned a = (g_reg_pll >> 3) & 0x3f;
    unsigned b = ((g_reg_pll >> 9) & 0x7f) + 64;
    unsigned n = b * 64 + a;
    bool high_band = ((g_reg_pwr >> 6) & 3) == TRF6151_BAND_1800;
    unsigned freq10 = high_band ? n * 2 : n;
    int f = 0, ul = -1;
    if (g_reg_pwr & (1u << 11)) {
        f |= 1 << 15;
    }
    if (g_reg_pwr & (1u << 13)) {
        f |= 1 << 14;
    }
    if (!g_reg_pll) {
        return f;
    }
    if (freq10 >= 8902 && freq10 <= 9148) {
        ul = (int)((freq10 - 8900) / 2);
    } else if (freq10 >= 8802 && freq10 < 8900) {
        ul = 1024 - (int)(8900 - freq10) / 2;
    } else if (freq10 >= 17102 && freq10 <= 17848) {
        ul = (int)(512 + (freq10 - 17102) / 2);
    }
    if (ul >= 0) {
        f |= (1 << 13) | (ul & 0x7ff);
    }
    return f;
}

int calypso_trf6151_total_gain_db(void)
{
    return SYSTEM_INHERENT_GAIN + trf6151_gain_from_reg(g_reg_rx);
}

uint16_t calypso_trf6151_apm_for_rf(int target_rf_dbm)
{
    int total_gain = calypso_trf6151_total_gain_db();
    int bb_dbm = target_rf_dbm + total_gain;
    int apm;

    if (bb_dbm < 0) {
        bb_dbm = 0;
    }
    apm = bb_dbm * 64;
    /* Never 0: a real DSP always measures some noise. a_pm = 0 counts as a failure for
     * the TI L1 (l1_check_pm_error: 100 null PMs -> l1_initialize_for_recovery, DSP
     * restarted every ~173 frames while scanning empty frequencies). The L1 reads
     * pm = a_pm >> 5 (l1ddsp_meas_read): floor 64 (1 dB above 0), pm = 2. */
    if (apm < 64) {
        apm = 64;
    }
    if (apm > 0xFFFF) {
        apm = 0xFFFF;
    }
    return (uint16_t)apm;
}
