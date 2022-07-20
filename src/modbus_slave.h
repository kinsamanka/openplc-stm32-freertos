#ifndef MODBUS_SLAVE_H
#define MODBUS_SLAVE_H

#include <FreeRTOS.h>
#include <semphr.h>

#define LIGHTMODBUS_SLAVE_FULL
#include <lightmodbus/lightmodbus.h>

#include "config.h"

struct context {
    uint8_t *buf;
    SemaphoreHandle_t *mutex;
};

ModbusError staticAllocator(ModbusBuffer *, uint16_t, void *);
ModbusError regCallback(const ModbusSlave *, const ModbusRegisterCallbackArgs *,
                        ModbusRegisterCallbackResult *);

#endif
