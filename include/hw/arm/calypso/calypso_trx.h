/* SPDX-License-Identifier: GPL-2.0-or-later */
#ifndef CALYPSO_TRX_H
#define CALYPSO_TRX_H

#include "hw/irq.h"
#include "exec/memory.h"

#define CALYPSO_IRQ_WATCHDOG       0
#define CALYPSO_IRQ_TIMER1         1
#define CALYPSO_IRQ_TIMER2         2
#define CALYPSO_IRQ_TSP_RX         3
#define CALYPSO_IRQ_TPU_FRAME      4
#define CALYPSO_IRQ_TPU_PAGE       5
#define CALYPSO_IRQ_SIM            6
#define CALYPSO_IRQ_UART_MODEM     7
#define CALYPSO_IRQ_KEYPAD_GPIO    8
#define CALYPSO_IRQ_RTC_TIMER      9
#define CALYPSO_IRQ_RTC_ALARM      10
#define CALYPSO_IRQ_ULPD_GAUGING   11
#define CALYPSO_IRQ_EXTERNAL       12
#define CALYPSO_IRQ_SPI            13
#define CALYPSO_IRQ_DMA            14
#define CALYPSO_IRQ_API            15
#define CALYPSO_IRQ_SIM_DETECT     16
#define CALYPSO_IRQ_EXTERNAL_FIQ   17
#define CALYPSO_IRQ_UART_IRDA      18
#define CALYPSO_IRQ_ULPD_GSM_TIMER 19
#define CALYPSO_IRQ_GEA            20
#define CALYPSO_NUM_IRQS           32

#define CALYPSO_TPU_BASE      0xFFFF1000
#define CALYPSO_TPU_SIZE      0x0100
#define CALYPSO_TPU_RAM_BASE  0xFFFF9000
#define CALYPSO_TPU_RAM_SIZE  0x0800
#define CALYPSO_TSP_BASE      0xFFFE0800
#define CALYPSO_TSP_SIZE      0x0100
#define CALYPSO_SIM_BASE      0xFFFE0000
#define CALYPSO_SIM_SIZE      0x0100
#define CALYPSO_ULPD_BASE     0xFFFE2800
#define CALYPSO_ULPD_SIZE     0x0100

#define TPU_CTRL              0x0000
#define TPU_INT_CTRL          0x0002
#define TPU_INT_STAT          0x0004
#define TPU_OFFSET            0x000C
#define TPU_SYNCHRO           0x000E
#define TPU_IT_DSP_PG         0x0020

#define TPU_CTRL_EN           (1 << 2)
#define TPU_CTRL_DSP_EN       (1 << 4)
#define TPU_CTRL_IDLE         (1 << 8)

#define ICTRL_MCU_FRAME       (1 << 0)
#define ICTRL_DSP_FRAME       (1 << 2)

#define TSP_RX_REG            0x08

#define ULPD_SETUP_CLK13      0x00
#define ULPD_COUNTER_HI       0x1C
#define ULPD_COUNTER_LO       0x1E
#define ULPD_GAUGING_CTRL     0x24
#define ULPD_GSM_TIMER        0x28

void calypso_trx_init(MemoryRegion *sysmem, qemu_irq *irqs);
/* Ligne RESET_DSP du registre CNTL_RST (calypso_soc.c) : en DSP externe, le
 * front asserte -> relache relance la ROM de boot du C54x (PONT_RESET). */
void calypso_trx_dsp_reset_line(bool active);
void calypso_tpu_run_scenario(uint16_t *tpu_ram, uint32_t fn, uint16_t *tpu_regs);
void calypso_tpu_sequencer_tick(uint32_t fn);

#endif
