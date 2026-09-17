/* SPDX-License-Identifier: GPL-2.0-or-later */
#ifndef HW_ARM_CALYPSO_TRF6151_H
#define HW_ARM_CALYPSO_TRF6151_H

#include <stdint.h>

void calypso_trf6151_tsp_write(uint8_t dev_idx, uint32_t word);

int calypso_trf6151_total_gain_db(void);

uint16_t calypso_trf6151_apm_for_rf(int target_rf_dbm);
int calypso_trf6151_arfcn(void);

#endif
