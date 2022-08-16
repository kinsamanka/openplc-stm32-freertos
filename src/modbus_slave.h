#ifndef MODBUS_SLAVE_H
#define MODBUS_SLAVE_H

#include <FreeRTOS.h>
#include <semphr.h>

#define LIGHTMODBUS_SLAVE_FULL
#include <lightmodbus/lightmodbus.h>

#include "config.h"

void modbus_slave_task(void *);

#endif
