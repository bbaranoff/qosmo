/* SPDX-License-Identifier: GPL-2.0-or-later */

#ifndef HW_CHAR_CALYPSO_UART_H
#define HW_CHAR_CALYPSO_UART_H

#include "hw/sysbus.h"
#include "chardev/char-fe.h"
#include "qom/object.h"

#define TYPE_CALYPSO_UART "calypso-uart"
OBJECT_DECLARE_SIMPLE_TYPE(CalypsoUARTState, CALYPSO_UART)

#define CALYPSO_UART_RX_FIFO_SIZE 8192

typedef struct CalypsoUARTState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;

    CharBackend chr;
    qemu_irq irq;

    char *label;

    uint8_t ier;
    uint8_t iir;
    uint8_t fcr;
    uint8_t lcr;
    uint8_t mcr;
    uint8_t lsr;
    uint8_t msr;
    uint8_t spr;
    uint8_t dll;
    uint8_t dlh;
    uint8_t mdr1;

    uint8_t efr;
    uint8_t xon1;
    uint8_t xon2;
    uint8_t xoff1;
    uint8_t xoff2;
    uint8_t scr;
    uint8_t ssr;

    uint8_t rx_fifo[CALYPSO_UART_RX_FIFO_SIZE];
    uint16_t rx_head;
    uint16_t rx_tail;
    uint16_t rx_count;

    bool thr_empty_pending;

    uint8_t tx_empty_reads;

    QEMUTimer *rx_poll_timer;

} CalypsoUARTState;

int calypso_uart_can_receive(void *opaque);
void calypso_uart_receive(void *opaque, const uint8_t *buf, int size);

void calypso_uart_inject_raw(CalypsoUARTState *s, const uint8_t *buf, int size);

void calypso_uart_kick_rx(CalypsoUARTState *s);

void calypso_uart_poll_backend(CalypsoUARTState *s);

void calypso_uart_kick_tx(CalypsoUARTState *s);
void calypso_uart_force_init(CalypsoUARTState *s);


#endif
