#ifndef UART_TASK_H
#define UART_TASK_H

#include "config.h"

#define WAIT_INFINITE   portMAX_DELAY

#if (MAX_REQUEST > MAX_RESPONSE)
#define UART_BUF_LEN    MAX_REQUEST
#else
#define UART_BUF_LEN    MAX_RESPONSE
#endif

void uart_task(void *);

#endif
