#ifndef TASK_PARAMS_H
#define TASK_PARAMS_H

#include <FreeRTOS.h>
#include <stdint.h>

#define USART1_IDLE_BIT             0x0001
#define USART1_TC_BIT               0x0002
#define DMA_RX1_TC_BIT              0x0004
#define DMA_RX1_HT_BIT              0x0008
#define RX1_UPDATE_BITS             (USART1_IDLE_BIT | DMA_RX1_TC_BIT | DMA_RX1_HT_BIT)

#define USART2_IDLE_BIT             0x0010
#define USART2_TC_BIT               0x0020
#define DMA_RX2_TC_BIT              0x0040
#define DMA_RX2_HT_BIT              0x0080
#define RX2_UPDATE_BITS             (USART2_IDLE_BIT | DMA_RX2_TC_BIT | DMA_RX2_HT_BIT)

#define TCP_SRC                     0x0100
#define USART1_SRC                  0x0400
#define USART2_SRC                  0x1000

enum msg_sources {
    UIP_TCP,
    USART1_RTU,
    USART2_RTU,
    NUM_SRCS
};

struct task_parameters {
    SemaphoreHandle_t mutex;
    TaskHandle_t uip;
    TaskHandle_t uart;
    TaskHandle_t modbus_slave;
    uint8_t uip_notify_flag;
    struct modbus_slave_msg *msgs;
};

struct modbus_slave_msg {
    uint8_t *data;
    size_t *length;
    TaskHandle_t src;
    uint32_t src_bits;
    uint8_t rtu_flag;
};

#endif
