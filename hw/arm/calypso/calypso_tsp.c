/* SPDX-License-Identifier: GPL-2.0-or-later */
#include "qemu/osdep.h"
#include "hw/arm/calypso/calypso_trf6151.h"
#include "hw/arm/calypso/calypso_tsp.h"

static struct {
    uint8_t tx[4];
    uint8_t ctrl1;
} tsp;

bool calypso_tsp_owns_addr(uint8_t addr)
{
    switch (addr) {
    case TPUI_TSP_CTRL1: case TPUI_TSP_CTRL2:
    case TPUI_TX_1: case TPUI_TX_2: case TPUI_TX_3: case TPUI_TX_4:
    case TPUI_TSP_ACT_L: case TPUI_TSP_ACT_U:
    case TPUI_TSP_SET1: case TPUI_TSP_SET2: case TPUI_TSP_SET3:
        return true;
    default:
        return false;
    }
}

void calypso_tsp_move(uint8_t addr, uint8_t data, uint32_t fn)
{
    switch (addr) {
    case TPUI_TX_1: tsp.tx[0] = data; break;
    case TPUI_TX_2: tsp.tx[1] = data; break;
    case TPUI_TX_3: tsp.tx[2] = data; break;
    case TPUI_TX_4: tsp.tx[3] = data; break;
    case TPUI_TSP_CTRL1:
        tsp.ctrl1 = data;
        break;
    case TPUI_TSP_CTRL2:
        if (data & TPUI_CTRL2_WR) {
            uint8_t dev_idx = (tsp.ctrl1 >> 5) & 0x07;
            uint8_t bitlen = (tsp.ctrl1 & 0x1F) + 1;
            uint32_t dout;
            if (bitlen <= 8) {
                dout = tsp.tx[0];
            } else if (bitlen <= 16) {
                dout = ((uint32_t)tsp.tx[0] << 8) | tsp.tx[1];
            } else if (bitlen <= 24) {
                dout = ((uint32_t)tsp.tx[0] << 16) | ((uint32_t)tsp.tx[1] << 8) | tsp.tx[2];
            } else {
                dout = ((uint32_t)tsp.tx[0] << 24) | ((uint32_t)tsp.tx[1] << 16) |
                       ((uint32_t)tsp.tx[2] << 8) | tsp.tx[3];
            }
            calypso_trf6151_tsp_write(dev_idx, dout);
        }
        break;
    default:
        break;
    }
}
