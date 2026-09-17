/* SPDX-License-Identifier: GPL-2.0-or-later */
#ifndef CALYPSO_TSP_H
#define CALYPSO_TSP_H

#include <stdint.h>

#define TPUI_TSP_CTRL1   0x00
#define TPUI_TSP_CTRL2   0x01
#define TPUI_TX_3        0x02
#define TPUI_TX_2        0x03
#define TPUI_TX_1        0x04
#define TPUI_TX_4        0x05
#define TPUI_TSP_ACT_L   0x06
#define TPUI_TSP_ACT_U   0x07
#define TPUI_TSP_SET1    0x09
#define TPUI_TSP_SET2    0x0a
#define TPUI_TSP_SET3    0x0b

#define TPUI_CTRL2_RD    (1 << 0)
#define TPUI_CTRL2_WR    (1 << 1)

bool calypso_tsp_owns_addr(uint8_t addr);

void calypso_tsp_move(uint8_t addr, uint8_t data, uint32_t fn);

#endif
