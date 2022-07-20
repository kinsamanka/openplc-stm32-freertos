#ifndef MODBUS_TCP_H
#define MODBUS_TCP_H

#include "modbus_slave.h"
#include "uipopt.h"

typedef struct modbus_tcp_state {
    uint8_t data[MAX_REQUEST];
    uint8_t len;
} uip_tcp_appstate_t;

void modbus_tcp_appcall(void);

#ifndef UIP_APPCALL
#define UIP_APPCALL modbus_tcp_appcall
#endif

void modbus_tcp_init(SemaphoreHandle_t *);

#endif
