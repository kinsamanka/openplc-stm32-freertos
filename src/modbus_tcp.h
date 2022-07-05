#ifndef MODBUS_TCP_H
#define MODBUS_TCP_H

#include <FreeRTOS.h>
#include <semphr.h>

#define LIGHTMODBUS_SLAVE_FULL
#include <lightmodbus/lightmodbus.h>

#include "config.h"

#include "uipopt.h"

typedef struct modbus_tcp_state {
    uint8_t data[MAX_REQUEST];
    uint8_t len;
} uip_tcp_appstate_t;

void modbus_tcp_appcall(void);

#ifndef UIP_APPCALL
#define UIP_APPCALL modbus_tcp_appcall
#endif

void modbus_init(SemaphoreHandle_t *);

#endif
