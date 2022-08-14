#ifndef UART2_TASK_H
#define UART2_TASK_H

#include "config.h"

#define WAIT_INFINITE   portMAX_DELAY

#if (MAX_REQUEST > MAX_RESPONSE)
#define UART_BUF_LEN    MAX_REQUEST
#else
#define UART_BUF_LEN    MAX_RESPONSE
#endif

enum uart2_state {
    UART2_RX,
    UART2_TX,
};

void uart2_setup(void);
void uart2_task(void *);

#endif
