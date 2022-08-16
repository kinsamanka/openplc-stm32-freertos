#include "modbus_slave.h"
#include "task_params.h"

#define LIGHTMODBUS_IMPL
#include <lightmodbus/lightmodbus.h>

#include <string.h>

extern uint16_t QW[];
extern uint16_t IW[];
extern uint8_t QX[];
extern uint8_t IX[];

static ModbusErrorInfo mberr;
static ModbusSlave status;

static SemaphoreHandle_t mutex;

static ModbusError staticAllocator(ModbusBuffer * buffer, uint16_t size,
                                   void *context)
{
    (void)context;
    static uint8_t status_buf[MAX_RESPONSE];

    if (size != 0) {
        // Allocation reqest
        if (size <= MAX_RESPONSE) {
            // Allocation request is within bounds
            buffer->data = status_buf;
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

static ModbusError regCallback(const ModbusSlave * slave,
                               const ModbusRegisterCallbackArgs * args,
                               ModbusRegisterCallbackResult * result)
{
    (void)slave;

    int size = 0;

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
        xSemaphoreTake(mutex, portMAX_DELAY);
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
        xSemaphoreGive(mutex);
        break;

    case MODBUS_REGQ_W:
        xSemaphoreTake(mutex, portMAX_DELAY);
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
        xSemaphoreGive(mutex);
        break;
    }

    return MODBUS_OK;
}

static int handle_request(uint8_t * data, uint16_t * length, int rtu)
{
    if (rtu) {
        mberr = modbusParseRequestRTU(&status, SLAVE_ADDRESS,
                                      (const uint8_t *)data, *length);
    } else {
        mberr = modbusParseRequestTCP(&status, (const uint8_t *)data, *length);
    }

    switch (modbusGetGeneralError(mberr)) {
    case MODBUS_OK:
        break;

    case MODBUS_ERROR_ALLOC:

        if (*length < 2)
            break;
        mberr = modbusBuildExceptionRTU(&status,
                                        SLAVE_ADDRESS,
                                        data[1], MODBUS_EXCEP_SLAVE_FAILURE);
        break;

    default:
        return 0;
    }

    // return response if everything is OK
    if (modbusIsOk(mberr) && modbusSlaveGetResponseLength(&status)) {

        uint16_t sz = modbusSlaveGetResponseLength(&status);
        uint8_t *dat = (uint8_t *) modbusSlaveGetResponse(&status);

        *length = sz;
        memcpy(data, dat, sz);

        return 1;
    }

    return 0;
}

void modbus_slave_task(void *params)
{
    mutex = ((struct task_parameters *)params)->mutex;
    TaskHandle_t uip_task = ((struct task_parameters *)params)->uip;

    mberr = modbusSlaveInit(&status,
                            regCallback,
                            NULL,
                            staticAllocator,
                            modbusSlaveDefaultFunctions,
                            modbusSlaveDefaultFunctionCount);

    struct modbus_slave_msg *msg;

    for (;;) {
        xTaskNotifyWaitIndexed(1, 0x00, 0xffffffff, (uint32_t *) & msg,
                               portMAX_DELAY);

        int rtu = msg->src != uip_task;
        uint32_t result = handle_request(msg->data, msg->length, rtu);

        xTaskNotifyIndexed(msg->src, 1, result, eSetValueWithOverwrite);
    }
}
