#ifndef UART1_TASK_H
#define UART1_TASK_H

#include "config.h"

#define WAIT_INFINITE   portMAX_DELAY

#if (MAX_REQUEST > MAX_RESPONSE)
#define UART_BUF_LEN    MAX_REQUEST
#else
#define UART_BUF_LEN    MAX_RESPONSE
#endif

#define DMA_RX_COUNT    (UART_BUF_LEN - DMA_CNDTR(DMA1, DMA_CHANNEL5))

enum task_state {
    STATE_RX,
    STATE_TX,
};

void uart1_setup(void);
void uart1_task(void *);

#endif
