/* SPDX-License-Identifier: GPL-2.0-or-later */
#ifndef HW_ARM_CALYPSO_L1_H
#define HW_ARM_CALYPSO_L1_H

#include <stdint.h>
#include <stdbool.h>

void calypso_l1_init(const char *firmware_elf);
void calypso_l1_frame_tick(void);
void calypso_l1_page_written(uint16_t d_dsp_page);
void calypso_l1_burst_written(uint16_t d_burst_d);
void calypso_l1_rach_written(uint16_t d_rach, uint32_t fn);
bool calypso_l1_read_override(uint32_t off, uint16_t *out);
bool calypso_l1_si_valid(void);
uint32_t calypso_l1s_fn(void);

void calypso_l1_dcch_set(int kind, int ss);
void calypso_l1_dcch_active(void);
void calypso_l1_dcch_is_tch(bool on);
void calypso_l1_rach_conf(uint32_t fn);

#endif
