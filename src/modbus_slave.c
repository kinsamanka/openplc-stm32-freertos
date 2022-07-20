#include "modbus_slave.h"

#define LIGHTMODBUS_IMPL
#include <lightmodbus/lightmodbus.h>

extern uint16_t QW[];
extern uint16_t IW[];
extern uint8_t QX[];
extern uint8_t IX[];

ModbusError staticAllocator(ModbusBuffer * buffer, uint16_t size, void *context)
{
    if (size != 0) {
        // Allocation reqest
        if (size <= MAX_RESPONSE) {
            // Allocation request is within bounds
            buffer->data = ((struct context *) context)->buf;
            return MODBUS_OK;
        } else {
            // Allocation error
            buffer->data = NULL;
            return MODBUS_ERROR_ALLOC;
        }
    } else {
        // Free request
        buffer->data = NULL;
        return MODBUS_OK;
    }
}

ModbusError regCallback(const ModbusSlave * slave,
                        const ModbusRegisterCallbackArgs * args,
                        ModbusRegisterCallbackResult * result)
{

    int size = 0;

    struct context *mbc = modbusSlaveGetUserPointer(slave);
    SemaphoreHandle_t *pmutex = mbc->mutex;

    switch (args->query) {

    case MODBUS_REGQ_R_CHECK:
        switch (args->type) {

        case MODBUS_HOLDING_REGISTER:
            size = HOLDING_REG_COUNT;
            break;

        case MODBUS_INPUT_REGISTER:
            size = INPUT_REG_COUNT;
            break;

        case MODBUS_COIL:
            size = COIL_COUNT;
            break;

        case MODBUS_DISCRETE_INPUT:
            size = DISCRETE_COUNT;
            break;
        }

        if (args->index < size)
            result->exceptionCode = MODBUS_EXCEP_NONE;
        else
            result->exceptionCode = MODBUS_EXCEP_ILLEGAL_ADDRESS;
        break;

    case MODBUS_REGQ_W_CHECK:
        switch (args->type) {

        case MODBUS_HOLDING_REGISTER:
            size = HOLDING_REG_COUNT;
            break;

        case MODBUS_COIL:
            size = COIL_COUNT;
            break;

        default:
            size = 0;
        }

        if (args->index < size)
            result->exceptionCode = MODBUS_EXCEP_NONE;
        else
            result->exceptionCode = MODBUS_EXCEP_SLAVE_FAILURE;
        break;

    case MODBUS_REGQ_R:
        xSemaphoreTake(*pmutex, portMAX_DELAY);
        switch (args->type) {

        case MODBUS_HOLDING_REGISTER:
            result->value = QW[args->index];
            break;

        case MODBUS_INPUT_REGISTER:
            result->value = IW[args->index];
            break;

        case MODBUS_COIL:
            result->value = QX[args->index];
            break;

        case MODBUS_DISCRETE_INPUT:
            result->value = IX[args->index];
            break;
        }
        xSemaphoreGive(*pmutex);
        break;

    case MODBUS_REGQ_W:
        xSemaphoreTake(*pmutex, portMAX_DELAY);
        switch (args->type) {

        case MODBUS_HOLDING_REGISTER:
            QW[args->index] = args->value;
            break;

        case MODBUS_COIL:
            QX[args->index] = args->value;
            break;

        default:
            break;
        }
        xSemaphoreGive(*pmutex);
        break;
    }

    return MODBUS_OK;
}
