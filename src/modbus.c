#include "modbus.h"
#include "task_params.h"

#define LIGHTMODBUS_IMPL
#include <lightmodbus/lightmodbus.h>

#include <string.h>

extern uint16_t QW[];
extern uint16_t IW[];
extern uint8_t QX[];
extern uint8_t IX[];

static ModbusErrorInfo mberr;
static ModbusSlave slave;
static int err = 0;

static SemaphoreHandle_t mutex;

static ModbusError staticAllocator(ModbusBuffer * buffer, uint16_t size,
                                   void *context)
{
    (void)context;
    static uint8_t slave_buf[MAX_RESPONSE];

    if (size != 0) {
        // Allocation reqest
        if (size <= MAX_RESPONSE) {
            // Allocation request is within bounds
            buffer->data = slave_buf;
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
            size = QW_COUNT;
            break;

        case MODBUS_INPUT_REGISTER:
            size = IW_COUNT;
            break;

        case MODBUS_COIL:
            size = QX_COUNT;
            break;

        case MODBUS_DISCRETE_INPUT:
            size = IX_COUNT;
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
            size = QW_COUNT;
            break;

        case MODBUS_COIL:
            size = QX_COUNT;
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

static ModbusError exceptionCallback(const ModbusSlave * slave,
                                     uint8_t function, ModbusExceptionCode code)
{
    (void) slave;
    (void) function;
    (void) code;

    err = 1;
    return MODBUS_OK;
}

static int handle_request(uint8_t * data, uint16_t * length, int rtu)
{
    if (rtu) {
        mberr = modbusParseRequestRTU(&slave, SLAVE_ADDRESS,
                                      (const uint8_t *)data, *length);
    } else {
        mberr = modbusParseRequestTCP(&slave, (const uint8_t *)data, *length);
    }

    if (modbusIsOk(mberr)) {

        uint16_t sz;
        /* no reply on broadcast messages */
        if ((sz = modbusSlaveGetResponseLength(&slave))) {
            uint8_t *dat = (uint8_t *) modbusSlaveGetResponse(&slave);

            *length = sz;
            memcpy(data, dat, sz);

            return 1;
        }
    }

    return 0;
}

static void process_request(struct modbus_slave_msg *msg)
{
    err = 0;
    uint32_t result = handle_request(msg->data, msg->length, msg->rtu_flag);

    if (err)
        result = 0;

    if (result != 0)
        result = msg->src_bits << 1;

    result |= msg->src_bits;

    xTaskNotify(msg->src, result, eSetBits);
}

void modbus_slave_task(void *params)
{
    uint32_t src;
    struct modbus_slave_msg *msgs;

    msgs = ((struct task_parameters *)params)->msgs;
    mutex = ((struct task_parameters *)params)->mutex;

    mberr = modbusSlaveInit(&slave,
                            regCallback,
                            exceptionCallback,
                            staticAllocator,
                            modbusSlaveDefaultFunctions,
                            modbusSlaveDefaultFunctionCount);

    for (;;) {
        xTaskNotifyWait(0, 0xffffffff, &src, portMAX_DELAY);
        for (int i = 0; i < NUM_SRCS; i++)
            if (src & (1 << i))
                process_request(&msgs[i]);
    }
}
