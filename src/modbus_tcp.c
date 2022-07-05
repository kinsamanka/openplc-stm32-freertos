#include "uip.h"
#include "modbus_tcp.h"

#define LIGHTMODBUS_IMPL
#include <lightmodbus/lightmodbus.h>

#include <string.h>

static ModbusErrorInfo mberr;
static ModbusSlave mbs;

SemaphoreHandle_t *pmutex;

extern uint16_t QW[];
extern uint16_t IW[];
extern uint8_t QX[];
extern uint8_t IX[];

static struct modbus_tcp_state s;

static ModbusError staticAllocator(ModbusBuffer * buffer, uint16_t size,
                                   void *context)
{
    (void)context;

    // Array for holding the response frame
    static uint8_t response[MAX_RESPONSE];

    // Allocation reqest
    if (size != 0) {
        if (size <= MAX_RESPONSE) {
            // Allocation request is within bounds
            buffer->data = response;
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

static int modbus_handle_req(uint8_t ** dat, uint8_t * sz)
{
    mberr = modbusParseRequestTCP(&mbs, *dat, *sz);

    switch (modbusGetGeneralError(mberr)) {
    case MODBUS_OK:
        break;

    case MODBUS_ERROR_ALLOC:

        if (*sz < 2)
            break;
        mberr = modbusBuildExceptionRTU(&mbs,
                                        SLAVE_ADDRESS,
                                        *dat[1], MODBUS_EXCEP_SLAVE_FAILURE);
        break;

    default:
        return 0;
    }

    // return response if everything is OK
    if (modbusIsOk(mberr) && modbusSlaveGetResponseLength(&mbs)) {
        *sz = modbusSlaveGetResponseLength(&mbs);
        *dat = (uint8_t *) modbusSlaveGetResponse(&mbs);
        return 1;
    } else {
        return 0;
    }
}

void modbus_init(SemaphoreHandle_t * mutex)
{
    pmutex = mutex;

    uip_listen(HTONS(configMODBUS_PORT));
    mberr = modbusSlaveInit(&mbs,
                            regCallback,
                            NULL,
                            staticAllocator,
                            modbusSlaveDefaultFunctions,
                            modbusSlaveDefaultFunctionCount);
}

void modbus_tcp_appcall(void)
{
    uint8_t *dat;
    uint8_t sz;

    if (uip_acked())
        s.len = 0;

    if (uip_newdata()) {
        sz = uip_datalen();
        dat = uip_appdata;
        if (modbus_handle_req(&dat, &sz)) {
            memcpy(s.data, dat, sz);
            s.len = sz;
        } else {
            s.len = 0;
        }
    }

    if ((uip_rexmit() || uip_newdata()) && s.len)
        uip_send(s.data, s.len);
}
