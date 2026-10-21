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
    /* [2026-10-21 18:50] TSPEN1 (osmocom, Pirelli) ou TSPEN2 : le firmware TI des Motorola C1xx
     * adresse le TRF6151 sur le peripherique TSP 2 (tpudrv12.h, TC1_DEVICE_RF = TC1_DEVICE2) ; ignore,
     * l'ARFCN restait inconnu et le faux air ne filtrait rien. */
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
    /* [2026-10-21 19:25] jamais 0 : un vrai DSP mesure toujours du bruit. a_pm = 0 compte comme une
     * panne pour la L1 TI (l1_check_pm_error : 100 PM nuls -> l1_initialize_for_recovery, DSP relance
     * toutes les ~173 trames en balayant des frequences vides). La L1 lit pm = a_pm >> 5
     * (l1ddsp_meas_read) : plancher 64 (1 dB au-dessus de 0), pm = 2. */
    if (apm < 64) {
        apm = 64;
    }
    if (apm > 0xFFFF) {
        apm = 0xFFFF;
    }
    return (uint16_t)apm;
}
