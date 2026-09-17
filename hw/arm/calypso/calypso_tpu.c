/* SPDX-License-Identifier: GPL-2.0-or-later */
#include "qemu/osdep.h"
#include "hw/arm/calypso/calypso_trx.h"
#include "hw/arm/calypso/calypso_tsp.h"

#define TPU_OP_SLEEP     0
#define TPU_OP_AT        1
#define TPU_OP_OFFSET    2
#define TPU_OP_SYNCHRO   3
#define TPU_OP_MOVE      4
#define TPU_OP_WAIT      5

#define QBITS_PER_TDMA   5000

static struct {
    uint16_t insns[CALYPSO_TPU_RAM_SIZE / 2];
    int len;
    int cursor;
    int qbit;
    int wait_frames;
    bool active;
    uint16_t *tpu_regs;
} seq;

static void seq_run(uint32_t fn)
{
    while (seq.active && seq.cursor < seq.len) {
        uint16_t insn = seq.insns[seq.cursor];
        if (insn == 0x0000) {
            int next = seq.cursor + 1;
            bool had_any = seq.cursor > 0;
            if (had_any && (next >= seq.len || seq.insns[next] == 0x0000)) {
                seq.active = false;
                return;
            }
            seq.cursor++;
            continue;
        }
        uint8_t opcode = (insn >> 13) & 0x7;
        uint16_t payload = insn & 0x1FFF;

        if (opcode == TPU_OP_AT) {
            seq.cursor++;
            if (payload > seq.qbit) {
                seq.qbit = payload;
                continue;
            }
            seq.qbit = payload;
            seq.wait_frames = 1;
            return;
        }
        if (opcode == TPU_OP_WAIT) {
            seq.cursor++;
            int target = seq.qbit + (int)payload;
            if (target < QBITS_PER_TDMA) {
                seq.qbit = target;
                continue;
            }
            seq.wait_frames = target / QBITS_PER_TDMA;
            seq.qbit = target % QBITS_PER_TDMA;
            return;
        }
        if (opcode == TPU_OP_SYNCHRO || opcode == TPU_OP_OFFSET) {
            if (seq.tpu_regs) {
                seq.tpu_regs[(opcode == TPU_OP_SYNCHRO ? TPU_SYNCHRO : TPU_OFFSET) / 2] = payload;
            }
            seq.cursor++;
            continue;
        }
        if (opcode == TPU_OP_MOVE) {
            uint8_t addr = insn & 0x1F;
            uint8_t data = (insn >> 5) & 0xFF;
            if (calypso_tsp_owns_addr(addr)) {
                calypso_tsp_move(addr, data, fn);
            }
            seq.cursor++;
            continue;
        }
        seq.cursor++;
    }
    seq.active = false;
}

void calypso_tpu_run_scenario(uint16_t *tpu_ram, uint32_t fn, uint16_t *tpu_regs)
{
    memcpy(seq.insns, tpu_ram, sizeof(seq.insns));
    seq.len = CALYPSO_TPU_RAM_SIZE / 2;
    seq.cursor = 0;
    seq.qbit = 0;
    seq.wait_frames = 0;
    seq.tpu_regs = tpu_regs;
    seq.active = true;
    seq_run(fn);
}

void calypso_tpu_sequencer_tick(uint32_t fn)
{
    if (!seq.active || seq.wait_frames <= 0) {
        return;
    }
    seq.wait_frames--;
    if (seq.wait_frames > 0) {
        return;
    }
    seq_run(fn);
}
