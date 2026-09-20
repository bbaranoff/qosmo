/*
 * calypso_c54x.h — TMS320C54x DSP emulator for Calypso
 *
 * Emulates the C54x DSP core found in the TI Calypso baseband chip.
 * Loads ROM dump, executes instructions, shares API RAM with ARM.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */
#ifndef CALYPSO_C54X_H
#define CALYPSO_C54X_H

#include <stdint.h>
#include <stdbool.h>

/* Memory sizes (in 16-bit words) */
#define C54X_PROG_SIZE   0x40000  /* 256K words program space */
#define C54X_DATA_SIZE   0x10000  /* 64K words data space */
#define C54X_IO_SIZE     0x10000  /* 64K words I/O space */

/* API RAM: shared between ARM and DSP */
#define C54X_API_BASE    0x0800   /* DSP data address of API RAM */
#define C54X_API_SIZE    0x2000   /* 8K words */

/* DSP start address (after boot) */
#define C54X_DSP_START   0x7000

/* MMR addresses (data memory 0x00-0x1F) */
#define MMR_IMR   0x00
#define MMR_IFR   0x01
#define MMR_ST0   0x06
#define MMR_ST1   0x07
#define MMR_AL    0x08
#define MMR_AH    0x09
#define MMR_AG    0x0A
#define MMR_BL    0x0B
#define MMR_BH    0x0C
#define MMR_BG    0x0D
#define MMR_T     0x0E
#define MMR_TRN   0x0F
#define MMR_AR0   0x10
#define MMR_AR1   0x11
#define MMR_AR2   0x12
#define MMR_AR3   0x13
#define MMR_AR4   0x14
#define MMR_AR5   0x15
#define MMR_AR6   0x16
#define MMR_AR7   0x17
#define MMR_SP    0x18
#define MMR_BK    0x19
#define MMR_BRC   0x1A
#define MMR_RSA   0x1B
#define MMR_REA   0x1C
#define MMR_PMST  0x1D
#define MMR_XPC   0x1E

/* Timer registers (memory-mapped at 0x0024-0x0026) */
#define TIM_ADDR  0x0024   /* Timer counter */
#define PRD_ADDR  0x0025   /* Timer period */
#define TCR_ADDR  0x0026   /* Timer control */

/* TCR bit positions (TMS320C54x hardware spec) */
#define TCR_TDDR_MASK  0x000F   /* bits 3:0 — prescaler reload value */
#define TCR_TSS        (1 << 4) /* bit 4 — Timer Stop Status (1=stopped) */
#define TCR_TRB        (1 << 5) /* bit 5 — Timer Reload (write 1 reloads) */
#define TCR_PSC_SHIFT  6        /* bits 9:6 — prescale counter */
#define TCR_PSC_MASK   (0xF << TCR_PSC_SHIFT)
#define TCR_SOFT       (1 << 10)
#define TCR_FREE       (1 << 11)

/* ST0 bit positions */
#define ST0_DP_MASK  0x01FF  /* bits 8-0: data page pointer */
#define ST0_OVB      (1 << 9)
#define ST0_OVA      (1 << 10)
#define ST0_C        (1 << 11)
#define ST0_TC       (1 << 12)
#define ST0_ARP_SHIFT 13
#define ST0_ARP_MASK (7 << ST0_ARP_SHIFT)

/* ST1 bit positions */
#define ST1_ASM_MASK 0x001F  /* bits 4-0: accumulator shift mode */
#define ST1_CMPT     (1 << 5)
#define ST1_FRCT     (1 << 6)
#define ST1_C16      (1 << 7)
#define ST1_SXM      (1 << 8)
#define ST1_OVM      (1 << 9)
#define ST1_INTM     (1 << 11)
#define ST1_HM       (1 << 12)
#define ST1_XF       (1 << 13)
#define ST1_CPL      (1 << 14)  /* compiler mode: direct addressing is SP-relative */
#define ST1_BRAF     (1 << 15)  /* block-repeat active. [2026-09-20] Was (1 << 14):
                                 * every RPTB/RPTBD raised CPL instead of BRAF. */

/* PMST bit positions (per SPRU131: SST=0 SMUL=1 CLKOFF=2 DROM=3 APTS=4 OVLY=5 MP/MC=6) */
#define PMST_SST     (1 << 0)
#define PMST_SMUL    (1 << 1)
#define PMST_CLKOFF  (1 << 2)
#define PMST_DROM    (1 << 3)
#define PMST_APTS    (1 << 4)
#define PMST_OVLY    (1 << 5)
#define PMST_MP_MC   (1 << 6)
#define PMST_IPTR_SHIFT 7
#define PMST_IPTR_MASK (0x1FF << PMST_IPTR_SHIFT)

/* Interrupt vectors */
#define C54X_INT_RESET   0
#define C54X_INT_NMI     1
/* ============================================================================
 * CALYPSO DSP INTERRUPT TABLE. Authority: CAL207 15.1 "DSP interrupts Mapping",
 * which gives each vector's hex Location; vec = Location / 4. CAL000 5.1: "The
 * DSP subchip owns 17 interrupt lines with 11 of which INT0n to INT10n are
 * dedicated for external peripherals."
 *
 * Do NOT substitute the generic TMS320C54x table from SPRU131: the Calypso DSP
 * subchip (S28C128) has its own peripheral mapping. The generic part has four
 * external lines (INT0..INT3) before TINT, Calypso has three (INT0n..INT2n), so
 * everything from bit 3 on shifts by one, and BRINT0/BXINT0/DMAC0 do not exist
 * here at all.
 *
 *  bit  vec  Loc.  line      source (CAL207 15.1)               trigger
 *  ---  ---  ----  --------  ---------------------------------  -------
 *   0    16  0x40  INT0n     RIF receive interrupt              level
 *   1    17  0x44  INT1n     RIF transmit interrupt             level
 *   2    18  0x48  INT2n     UART interrupt                     level
 *   3    19  0x4C  TINT      DSP timer
 *   4    20  0x50  RINT      SPI receive interrupt
 *   5    21  0x54  XINT      SPI transmit interrupt
 *   6    22  0x58  INT4n     MCSI transmit interrupt            level
 *   7    23  0x5C  INT5n     MCSI frame duration error          level
 *   8    24  0x60  INT3n     MCSI receive interrupt             level
 *   9    25  0x64  AINT      API interrupt (ARM <-> DSP)
 *  10    26  0x68  INT6n     MCSI DAI interrupt                 level
 *  11    27  0x6C  INT7n     CYPHER interrupt                   edge
 *  12    28  0x70  INT8n     TPU FRAME interrupt                edge
 *  13    29  0x74  INT9n     TPU programmable interrupt         edge
 *  14    30  0x78  INT10n    DMA interrupt                      level
 *        1   0x04  nMIN      Rhea bus abort, or INT4n redirect (= NMI)
 *
 * The prose list in CAL000 5.1 disagrees from bit 6 on (it places AINT at bit
 * 12). The CAL207 Location column wins: it is an address, not an order to be
 * interpreted.
 *
 * Measurements backing this table:
 *  - ROM IMR reads 0x52ef (bits 0,1,2,3,5,6,7,9,12,14; 0x52ed on other runs,
 *    the same minus bit 1) = RIF rx/tx, UART, TINT, SPI tx, MCSI tx/err, AINT,
 *    TPU frame, DMA. The bits left masked are CYPHER (11), MCSI rx (8), TPU
 *    programmable (13) and SPI rx (4) - exactly what a GSM L1 does not use.
 *  - The ROM also does IMR |= 0x3000 = bits 12+13 = both TPU lines.
 *  - The firmware writes 0x0380 to CNTRL_REG (XIO:FA00), which selects edge vs
 *    level per channel (CAL207 15.2.1): bits 7, 8, 9 edge, matching exactly the
 *    three lines CAL207 15.1 marks as edge (INT7n/INT8n/INT9n). Channel N is
 *    therefore INTNn.
 *  - vec 20 (RINT) and vec 21 (XINT) are SPI, unused by a GSM L1, and the ROM
 *    holds a bare RETE stub at both.
 *
 * CAL000 5.1 on INT9n: "a facility offered to the DSP programmer in order to
 * allow the generation of a DSP interrupt at a dedicated time with a quarter of
 * GSM bit accuracy. The interrupt is set in a scenario by using a time-stamped
 * instruction." That is the TPU sequencer's MOVE TPUI_DSP_INT_PG.
 *
 * Formula: vec = imr_bit + 16.
 * ==========================================================================*/
#define C54X_IT_RIF_RX_VEC     16   /* INT0n  RIF receive              */
#define C54X_IT_RIF_RX_BIT      0
#define C54X_IT_RIF_TX_VEC     17   /* INT1n  RIF transmit             */
#define C54X_IT_RIF_TX_BIT      1
#define C54X_IT_UART_VEC       18   /* INT2n  UART                     */
#define C54X_IT_UART_BIT        2
#define C54X_IT_TINT_VEC       19   /* TINT   timer DSP                */
#define C54X_IT_TINT_BIT        3
#define C54X_IT_SPI_RX_VEC     20   /* RINT   SPI receive              */
#define C54X_IT_SPI_RX_BIT      4
#define C54X_IT_SPI_TX_VEC     21   /* XINT   SPI transmit             */
#define C54X_IT_SPI_TX_BIT      5
#define C54X_IT_MCSI_TX_VEC    22   /* INT4n  MCSI transmit     (0x58) */
#define C54X_IT_MCSI_TX_BIT     6
#define C54X_IT_MCSI_ERR_VEC   23   /* INT5n  MCSI frame dur.   (0x5C) */
#define C54X_IT_MCSI_ERR_BIT    7
#define C54X_IT_MCSI_RX_VEC    24   /* INT3n  MCSI receive      (0x60) */
#define C54X_IT_MCSI_RX_BIT     8
#define C54X_IT_API_VEC        25   /* AINT   API (ARM<->DSP)   (0x64) */
#define C54X_IT_API_BIT         9
#define C54X_IT_MCSI_DAI_VEC   26   /* INT6n  MCSI DAI          (0x68) */
#define C54X_IT_MCSI_DAI_BIT   10
#define C54X_IT_CRYPT_VEC      27   /* INT7n  CYPHER            (0x6C) */
#define C54X_IT_CRYPT_BIT      11
#define C54X_IT_TPU_FRAME_VEC  28   /* INT8n  TPU frame         (0x70) */
#define C54X_IT_TPU_FRAME_BIT  12
#define C54X_IT_TPU_PROG_VEC   29   /* INT9n  TPU programmable  (0x74) */
#define C54X_IT_TPU_PROG_BIT   13
#define C54X_IT_DMA_VEC        30   /* INT10n DMA               (0x78) */
#define C54X_IT_DMA_BIT        14

#define C54X_NUM_INTS        16

typedef struct C54xState {
    /* Accumulators (40-bit) stored as int64 for convenience */
    int64_t a;   /* A accumulator: bits 39-0 */
    int64_t b;   /* B accumulator: bits 39-0 */

    /* Auxiliary registers */
    uint16_t ar[8];

    /* Other registers */
    uint16_t t;      /* Temporary register */
    uint16_t trn;    /* Transition register (Viterbi) */
    uint16_t sp;
    uint16_t bk;     /* Circular buffer size */
    uint16_t brc;    /* Block repeat counter */
    uint16_t rsa;    /* Block repeat start address */
    uint16_t rea;    /* Block repeat end address */

    /* Status registers */
    uint16_t st0;
    uint16_t st1;
    uint16_t pmst;

    /* Interrupt registers */
    uint16_t imr;
    uint16_t ifr;

    /* Optional reset-state override loaded from calypso_dsp.Registers.bin via
     * `-M calypso,dsp-registers=<path>` (default-wired by run.sh, like the
     * other ROM sections). reg_init[i] = value for MMR index i (0x00..0x1F).
     * When reg_init_valid, c54x_reset() applies these AFTER its silicon
     * hardcode defaults, so the .bin snapshot is authoritative. */
    uint16_t reg_init[0x20];
    bool     reg_init_valid;

    /* Program counter */
    uint32_t pc;     /* 16-bit (or 23-bit with XPC) */
    uint16_t xpc;

    /* Timer0 prescale counter (PSC) — not memory-mapped directly */
    uint16_t timer_psc;

    /* DMA sub-register bank (6 channels × 4 regs) */
    uint16_t dma_subaddr;
    uint16_t dma_subregs[24];
    /* McBSP sub-register bank */
    uint16_t spsa;

    /* RPT state */
    uint16_t rpt_count;  /* remaining RPT iterations */
    uint16_t rpt_pc;     /* PC of repeated instruction */
    bool     rpt_active;
    bool     rpt_fresh;   /* RPT just armed: the first READA/MVPD read starts
                          * from the base address, not from a stale mvpd_src. */
    uint16_t par;        /* Program Address Register (for READA/WRITA/MACD/MACP) */
    bool     par_set;
    bool     lk_used;    /* resolve_smem consumed extra word for lk */
    uint16_t mvpd_src;   /* MVPD auto-increment source address during RPT */

    /* RPTB state */
    bool     rptb_active;

    /* Delayed-branch state (CALLD/RETD/BD/CCD/...): when set, the next
     * `delay_slots` instructions execute normally, then PC is forced to
     * `delayed_pc`. */
    uint16_t delayed_pc;
    uint8_t  delay_slots;

    /* Memory */
    uint16_t prog[C54X_PROG_SIZE];   /* Program memory */
    uint16_t data[C54X_DATA_SIZE];   /* Data memory */

    /* API RAM pointer (shared with ARM calypso_trx.c) */
    uint16_t *api_ram;  /* points into ARM's dsp_ram[] */

    /* DSP → ARM notify hook: called whenever the DSP writes to api_ram. */
    void (*api_write_cb)(void *opaque, uint16_t woff, uint16_t val);
    void  *api_write_cb_opaque;

    /* State */
    bool     running;
    bool     idle;       /* IDLE instruction executed */
    bool     blob_loaded; /* Test fixture: set by c54x_set_initial_pc().
                           * Suppresses the secondary c54x_reset() that
                           * normally fires when ARM writes DSP_DL_STATUS_READY,
                           * which would otherwise clobber the user's blob
                           * via the reset-time PROM→DARAM auto-copy. */
    uint64_t cycles;
    uint32_t insn_count;

    /* BSP (Baseband Serial Port) — burst sample buffer */
    uint16_t bsp_buf[2048]; /* burst I/Q samples from radio */
    int      bsp_len;       /* number of samples */
    int      bsp_pos;       /* read position */

    /* Debug */
    uint32_t unimpl_count;
    uint16_t last_unimpl;
    /* Last executed instruction snapshot — captured at end of each
     * c54x_run iteration. Used by the INTM-TRANS tracer (and others)
     * to attribute post-instruction state changes to the actual cause
     * PC/opcode rather than the post-advance PC. */
    uint16_t last_exec_pc;
    uint16_t last_exec_op;

    /* writer_kind : set by each opcode handler / external writer before
     * calling data_write. Logged in DATA-W-MMR trace to disambiguate
     * which path is responsible for stray writes to MMR (addr<=0x1F).
     * Reset to WK_UNKNOWN at the top of c54x_exec_one. */
    uint8_t  writer_kind;
} C54xState;

/* writer_kind enum — keep small, extend as needed */
enum {
    WK_UNKNOWN     = 0,
    WK_OPCODE_F3   = 1,   /* 0xF3xx family (SFTL/AND/OR/XOR/INTR/etc.) */
    WK_OPCODE_8x   = 2,   /* 0x80xx-0x8Fxx (STL/STH/STLM/STM/LD-Smem) */
    WK_OPCODE_77   = 3,   /* 0x77xx STM #lk, MMR */
    WK_OPCODE_76   = 4,   /* 0x76xx ST #lk, Smem */
    WK_OPCODE_PSHM = 5,   /* PSHM/POPM stack ops */
    WK_OPCODE_RET  = 6,   /* RET/RETI/RETD frame restore */
    WK_IRQ_ACK     = 7,   /* IRQ acknowledge / vector dispatch */
    WK_ARM_MMIO    = 8,   /* ARM-side write through shared region */
    WK_RESOLVE_AR  = 9,   /* resolve_smem AR-modify side effect */
    WK_OPCODE_OTHER= 10,  /* anything else inside an opcode handler */
};

/* Feed burst samples to BSP (called by calypso_trx) */
void c54x_bsp_load(C54xState *s, const uint16_t *samples, int n);

/* Create and initialize C54x state */
C54xState *c54x_init(void);

/* Link API RAM (shared memory with ARM) */
void c54x_set_api_ram(C54xState *s, uint16_t *api_ram);

/* Reset the DSP */
void c54x_reset(C54xState *s);

/* Execute N instructions (returns actual count executed) */
int c54x_run(C54xState *s, int n_insns);

/* Raise an interrupt */
/* Send interrupt: vec = vector number (for PC), imr_bit = bit in IMR/IFR */
void c54x_interrupt_ex(C54xState *s, int vec, int imr_bit);

/* Wake from IDLE */
void c54x_wake(C54xState *s);

/* Test fixture: override PC after reset.
 * Used by `-M calypso,dsp-blob=<path>` to start execution at a custom
 * address instead of the silicon-default reset vector (IPTR * 0x80). */
void c54x_set_initial_pc(C54xState *s, uint32_t pc);

/* Test fixture: load a raw binary blob into DARAM starting at daram_addr.
 * File bytes are read pairwise as little-endian DSP words.
 * Returns number of words loaded, or -1 on error. */
int  c54x_load_blob_daram(C54xState *s, const char *path, uint16_t daram_addr);

/* Explicit per-section ROM load: write raw LE 16-bit words from `path`
 * into either prog[] (when is_program=true) or data[] (when false),
 * starting at DSP word address `start_addr`. Used by the per-section
 * machine properties (dsp-prom0/prom1/prom2/prom3/drom/pdrom) to load
 * each ROM section at its silicon-correct DSP address.
 * Returns number of words loaded, or -1 on error. */
int  c54x_load_section(C54xState *s, const char *path,
                       uint32_t start_addr, bool is_program);

/* Load the DSP register snapshot (calypso_dsp.Registers.bin: raw LE 16-bit
 * words, MMR page 0x00..0x1F first) into reg_init[] so c54x_reset() applies
 * it as the reset state. Words >= 0x20 are written into data[] (low scratch).
 * Used by the `-M calypso,dsp-registers=<path>` machine property.
 * Returns number of words loaded, or -1 on error. */
int  c54x_load_registers(C54xState *s, const char *path);

/* [c54x-earlyboot] Boot the C54x at machine-init time, BEFORE the ARM vCPU
 * runs. ORDER IS LOAD-BEARING: the ARM posts its bootloader command
 * (data[0x0fff] = cmd, data[0x0ffe] = entry) from fn=0, and the DSP's own
 * init-IDLE at 0xb419 (ST #1,*0xfff) overwrites that command if the DSP boots
 * afterwards, leaving it spinning forever at 0xb41c. Booting here parks the DSP
 * on its IDLE before the ARM write, so the command survives. No mailbox value
 * is forced; only the timing of the boot is. Gate: CALYPSO_DSP_RUN_C54X=1. */
void c54x_early_boot(C54xState *s);

/* True if c54x_early_boot() actually parked the DSP. Gates the C54x re-reset in
 * calypso_trx.c, which would replay the PROM->DARAM copy and clobber the
 * bootloader command preserved above. */
bool c54x_early_booted(void);

/* Current DSP mission (d_task_md) read from API RAM: FB=5 SB=6 TCH_FB=8
 * TCH_SB=9, 0 = none. Page 0 = data[0x0804], page 1 = data[0x0818]. Used to
 * gate the inter-block wires (BSP BRINT0) on the FB/SB mission. */
uint16_t c54x_task_md(C54xState *s);

#endif /* CALYPSO_C54X_H */
