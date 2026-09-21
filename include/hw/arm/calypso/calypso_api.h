/* SPDX-License-Identifier: GPL-2.0-or-later */
#ifndef HW_ARM_CALYPSO_API_H
#define HW_ARM_CALYPSO_API_H

#include <stdint.h>

#define CALYPSO_API_BASE        0xFFD00000u
#define CALYPSO_API_SIZE        (64 * 1024)
#define CALYPSO_API_WORDS       (CALYPSO_API_SIZE / 2)

#define API_W_PAGE(p)           ((p) ? 0x0028u : 0x0000u)
#define API_R_PAGE(p)           ((p) ? 0x0078u : 0x0050u)
#define API_NDB                 0x01A8u

#define WP_D_TASK_D             0x00u
#define WP_D_BURST_D            0x02u
#define WP_D_TASK_U             0x04u
#define WP_D_BURST_U            0x06u
#define WP_D_TASK_MD            0x08u
#define WP_D_TASK_RA            0x0Eu
#define WP_D_FN                 0x10u

#define RP_D_TASK_D             0x00u
#define RP_D_BURST_D            0x02u
#define RP_D_TASK_MD            0x08u
#define RP_A_SERV_DEMOD         0x10u
#define RP_A_PM                 0x18u
#define RP_A_SCH                0x1Eu

#define NDB_D_DSP_PAGE          0x000u
#define NDB_D_FB_DET            0x048u
#define NDB_A_SYNC_DEMOD        0x04Cu
#define NDB_A_DD_1              0x108u
#define NDB_A_DU_1              0x134u
#define NDB_D_A5MODE            0x1CEu
#define NDB_A_CD                0x1FCu
#define NDB_A_FD                0x21Au
#define NDB_A_DD_0              0x238u
#define NDB_A_CU                0x264u
#define NDB_A_FU                0x282u
#define NDB_D_RACH              0x2CCu
#define NDB_A_KC                0x2CEu

#define API_BL_STATUS           0x0FFEu
#define API_VERSION             0x01B4u
#define API_VERSION2            0x01B6u
#define BL_STATUS_BOOT          0x0001u
#define BL_STATUS_READY         0x0002u
#define API_VERSION_VALUE       0x3606u

#define B_GSM_PAGE              (1u << 0)
#define B_GSM_TASK              (1u << 1)
#define B_BLUD                  (1u << 15)

#define D_TOA                   0
#define D_PM                    1
#define D_ANGLE                 2
#define D_SNR                   3

#define PM_DSP_TASK             1
#define FB_DSP_TASK             5
#define SB_DSP_TASK             6
#define RACH_DSP_TASK           10   /* firmware include/calypso/l1_environment.h:49 */
#define AUL_DSP_TASK            11
#define DUL_DSP_TASK            12
#define TCHT_DSP_TASK           13
#define TCHA_DSP_TASK           14
#define ALLC_DSP_TASK           24
#define TCHD_DSP_TASK           28

#define GSM_HYPERFRAME          2715648u
#define GSM_TDMA_NS             4615384

uint16_t *calypso_api_ram(void);
const char *calypso_firmware_elf(void);
uint32_t calypso_firmware_symbol(const char *name);
uint32_t calypso_trx_get_fn(void);
void calypso_trx_autosync_fn(uint32_t sch_fn);
void calypso_trx_force_tick(uint32_t fn);

static inline uint16_t *api_wp(uint8_t page, unsigned off)
{
    return &calypso_api_ram()[(API_W_PAGE(page) + off) / 2];
}

static inline uint16_t *api_rp(uint8_t page, unsigned off)
{
    return &calypso_api_ram()[(API_R_PAGE(page) + off) / 2];
}

static inline uint16_t *api_ndb(unsigned off)
{
    return &calypso_api_ram()[(API_NDB + off) / 2];
}

#endif
