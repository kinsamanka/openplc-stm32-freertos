#ifndef MODBUS_H
#define MODBUS_H

#include <FreeRTOS.h>
#include <semphr.h>

#include "config.h"

void modbus_task(void *params);

#endif
