#ifndef UART1_TASK_H
#define UART1_TASK_H

#include "config.h"

/* inter-frame delay */
#if (SLAVE_BAUD_RATE < 19200)
#define WAIT_END_FRAME  (38500000 / configTICK_RATE_HZ / SLAVE_BAUD_RATE / portTICK_RATE_MS)
#else
#define WAIT_END_FRAME  (2 / portTICK_RATE_MS)
#endif

#define WAIT_INFINITE   portMAX_DELAY
#define WAIT_500_MS     (500 / portTICK_RATE_MS)

#if (MAX_REQUEST > MAX_RESPONSE)
#define UART_BUF_LEN    MAX_REQUEST
#else
#define UART_BUF_LEN    MAX_RESPONSE
#endif

enum task_state {
    STATE_IDLE,
    STATE_BOOTLOADER,
    STATE_RECEIVING,
    STATE_TRANSMITTING,
    STATE_INVALID,
};

void uart1_setup(void);
void uart1_task(void *);

#endif
