/* SPDX-License-Identifier: GPL-2.0-or-later */
#include "qemu/osdep.h"
#include "hw/arm/calypso/calypso_trf6151.h"
#include "hw/arm/calypso/calypso_tsp.h"
#include "hw/arm/calypso/calypso_rf_arfcn.h"
#include <sys/mman.h>
#include <fcntl.h>
#include <unistd.h>


/* [2026-10-21] FAUX AIR : chaque programmation du synthetiseur part vers c54x_exe
 * (calypso_rf_arfcn.h). Ouvert a la premiere ecriture ; sans /dev/shm, rien. */
static void rf_arfcn_publier(uint32_t fn)
{
    static CalypsoRfArfcnShm *m;
    static int essaye;
    if (!m && !essaye) {
        essaye = 1;
        int fd = open(CALYPSO_RF_ARFCN_SHM, O_RDWR | O_CREAT, 0644);
        if (fd >= 0) {
            if (ftruncate(fd, sizeof *m) == 0) {
                void *p = mmap(NULL, sizeof *m, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
                if (p != MAP_FAILED) { m = p; m->magic = CALYPSO_RF_ARFCN_MAGIC; m->widx = 0; }
            }
            close(fd);
        }
    }
    if (!m) {
        return;
    }
    CalypsoRfProg *e = &m->e[m->widx % CALYPSO_RF_ARFCN_N];
    e->fn = fn;
    e->qbit = (int16_t)calypso_tpu_qbit();
    e->arfcn = (int16_t)calypso_trf6151_arfcn();
    e->gain_db = (int16_t)calypso_trf6151_total_gain_db();
    __atomic_store_n(&m->widx, m->widx + 1, __ATOMIC_RELEASE);
}

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
            /* [2026-10-22] TRACE A CHAUD (`touch /dev/shm/tsp_trace`, 6000 lignes) : toutes les ecritures TSP.
             * En veille Magnetite, aucune programmation vers le voisin de la liste BA (277 cote L1 TI) n'est
             * vue par le decodage TRF6151 : ses fenetres de mesure sont classees « cellule » par le faux air. */
            {
                static unsigned n_appels, n_lignes; static int on;
                if (n_appels++ % 256 == 0) on = (access("/dev/shm/tsp_trace", F_OK) == 0);
                if (on && n_lignes < 6000) {
                    n_lignes++;
                    fprintf(stderr, "[tsp] fn=%u qbit=%d dev=%u bits=%u dout=0x%08x reg=%u -> arfcn=%d gain=%d\n",
                            fn, calypso_tpu_qbit(), dev_idx, bitlen, dout, dout & 7,
                            calypso_trf6151_arfcn(), calypso_trf6151_total_gain_db());
                }
            }
            /* TRF6151 RX (gain, 2026-10-21), PLL ou PWR (bande), TSPEN1 ou 2 */
            if ((dev_idx == 1 || dev_idx == 2) && ((dout & 7) == 0 || (dout & 7) == 1 || (dout & 7) == 2)) {
                rf_arfcn_publier(fn);
            }
        }
        break;
    default:
        break;
    }
}
