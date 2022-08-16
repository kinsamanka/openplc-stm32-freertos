#ifndef TASK_PARAMS_H
#define TASK_PARAMS_H

#include <FreeRTOS.h>
#include <stdint.h>

struct task_parameters {
    SemaphoreHandle_t mutex;
    TaskHandle_t uip;
    TaskHandle_t uart1;
    TaskHandle_t uart2;
    TaskHandle_t modbus_slave;
    uint8_t uip_notify_flag;
};

struct modbus_slave_msg {
    uint8_t *data;
    uint16_t *length;
    TaskHandle_t src;
};

#endif
