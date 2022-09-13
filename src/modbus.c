#include "modbus.h"
#include "task_params.h"

#define LIGHTMODBUS_FULL
#define LIGHTMODBUS_IMPL
#include <lightmodbus/lightmodbus.h>

#include <string.h>

#ifndef TASK_TICK_TIMEOUT
#define TASK_TICK_TIMEOUT           5
#endif

enum modbus_type {
    QX_TYPE,
    IX_TYPE,
    QW_TYPE,
    IW_TYPE,
};

struct modbus_data {
    const uint8_t address, index, count, type;
};

#include "modbus_slave.h"

#ifndef SLAVE_DISCRETE_INPUT
#define SLAVE_DISCRETE_INPUT    {}
#endif
#ifndef SLAVE_COIL
#define SLAVE_COIL              {}
#endif
#ifndef SLAVE_INPUT_REGISTER
#define SLAVE_INPUT_REGISTER    {}
#endif
#ifndef SLAVE_HOLDING_REGISTER
#define SLAVE_HOLDING_REGISTER  {}
#endif

static const struct modbus_data discrete_inputs[] = SLAVE_DISCRETE_INPUT;
static const struct modbus_data coils[] = SLAVE_COIL;
static const struct modbus_data input_registers[] = SLAVE_INPUT_REGISTER;
static const struct modbus_data holding_registers[] = SLAVE_HOLDING_REGISTER;

extern uint16_t QW[];
extern uint16_t IW[];
extern uint8_t QX[];
extern uint8_t IX[];

static ModbusSlave slave;
static ModbusMaster master;
static int slave_err = 0;

static SemaphoreHandle_t mutex;

static ModbusError static_allocator(ModbusBuffer * buffer, uint16_t size,
                                    uint8_t * buf)
{
    if (size != 0) {
        // Allocation reqest
        if (size <= MAX_RESPONSE) {
            // Allocation request is within bounds
            buffer->data = buf;
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

static ModbusError slave_allocator(ModbusBuffer * buffer, uint16_t size,
                                   void *context)
{
    (void)context;
    static uint8_t buf[MAX_RESPONSE];

    return static_allocator(buffer, size, buf);
}

static ModbusError master_allocator(ModbusBuffer * buffer, uint16_t size,
                                    void *context)
{
    (void)context;
    static uint8_t buf[MAX_RESPONSE];

    return static_allocator(buffer, size, buf);
}

static ModbusError slave_CB(const ModbusSlave * status,
                            const ModbusRegisterCallbackArgs * args,
                            ModbusRegisterCallbackResult * result)
{
    (void)status;

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

static ModbusError master_CB(const ModbusMaster * status,
                             const ModbusDataCallbackArgs * args)
{
    uint8_t *ofs = (uint8_t *) modbusMasterGetUserPointer(status);

    xSemaphoreTake(mutex, portMAX_DELAY);
    if (args->function == 1 || args->function == 2) {

        IX[(*ofs)++] = args->value;

    } else if (args->function == 3 || args->function == 4) {

        IW[(*ofs)++] = args->value;
    }
    xSemaphoreGive(mutex);

    return MODBUS_OK;
}

static ModbusError slave_exception_CB(const ModbusSlave * status,
                                      uint8_t function,
                                      ModbusExceptionCode code)
{
    (void)status;
    (void)function;
    (void)code;

    slave_err = 1;
    return MODBUS_OK;
}

static ModbusError master_exception_CB(const ModbusMaster * status,
                                       uint8_t address, uint8_t function,
                                       ModbusExceptionCode code)
{
    (void)status;
    (void)address;
    (void)function;
    (void)code;

    return MODBUS_OK;
}

static int handle_slave_request(uint8_t * data, size_t *length, int rtu)
{
    ModbusErrorInfo err;

    if (rtu) {
        err = modbusParseRequestRTU(&slave, SLAVE_ADDRESS,
                                    (const uint8_t *)data, *length);
    } else {
        err = modbusParseRequestTCP(&slave, (const uint8_t *)data, *length);
    }

    if (modbusIsOk(err)) {

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

static int modbus_master_read(const struct modbus_data *data, uint8_t * buf)
{
    ModbusErrorInfo err;

    switch (data->type) {
    case QX_TYPE:
        err = modbusBuildRequest01RTU(&master, data->address, data->index,
                                      data->count);
        break;
    case IX_TYPE:
        err = modbusBuildRequest02RTU(&master, data->address, data->index,
                                      data->count);
        break;
    case QW_TYPE:
        err = modbusBuildRequest03RTU(&master, data->address, data->index,
                                      data->count);
        break;
    case IW_TYPE:
        err = modbusBuildRequest04RTU(&master, data->address, data->index,
                                      data->count);
        break;
    default:
        return 0;
    }

    if (!modbusIsOk(err))
        return 0;

    int sz = modbusMasterGetRequestLength(&master);
    memcpy(buf, (uint8_t *) modbusMasterGetRequest(&master), sz);

    return sz;
}

static int write_coils(const struct modbus_data *data, uint8_t * buf)
{
    uint8_t *ofs = (uint8_t *) modbusMasterGetUserPointer(&master);

    uint8_t tmp[SLAVE_COIL_COUNT / 8];

    for (unsigned i = 0; i < SLAVE_COIL_COUNT / 8; i++)
        tmp[i] = 0;

    for (int i = 0; i < data->count; i++) {
        xSemaphoreTake(mutex, portMAX_DELAY);
        if (QX[*ofs + i])
            tmp[i / 8] |= 1 << (i % 8);
        xSemaphoreGive(mutex);
    }

    *ofs += data->count;

    ModbusErrorInfo err;

    err = modbusBuildRequest15RTU(&master, data->address, data->index,
                                  data->count, (const uint8_t *)tmp);
    if (!modbusIsOk(err))
        return 0;

    int sz = modbusMasterGetRequestLength(&master);
    memcpy(buf, (uint8_t *) modbusMasterGetRequest(&master), sz);

    return sz;
}

static int write_holding_registers(const struct modbus_data *data,
                                   uint8_t * buf)
{
    uint8_t *ofs = (uint8_t *) modbusMasterGetUserPointer(&master);

    ModbusErrorInfo err;

    xSemaphoreTake(mutex, portMAX_DELAY);
    err = modbusBuildRequest16RTU(&master, data->address, data->index,
                                  data->count, (const uint16_t *)&QW[*ofs]);
    xSemaphoreGive(mutex);

    *ofs += data->count;

    if (!modbusIsOk(err))
        return 0;

    int sz = modbusMasterGetRequestLength(&master);
    memcpy(buf, (uint8_t *) modbusMasterGetRequest(&master), sz);

    return sz;
}

static void process_request(struct modbus_msg *msg)
{
    slave_err = 0;
    uint32_t result = handle_slave_request(msg->data, msg->length,
                                           msg->rtu_flag);

    if (slave_err)
        result = 0;

    if (result != 0)
        result = msg->src_bits << 1;

    result |= msg->src_bits;

    xTaskNotify(msg->src, result, eSetBits);
}

static void update_master(uint32_t src, struct modbus_msg *msg)
{
    static enum states {
        INIT,
        WAIT_START,
        READ_DI,
        READ_DI_WAIT,
        READ_IR,
        READ_IR_WAIT,
        WRITE_COIL,
        WRITE_COIL_WAIT,
        WRITE_HR,
        WRITE_HR_WAIT
    } state = INIT;

    static uint8_t i, offset;
    ModbusErrorInfo err;

    switch (state) {
    case INIT:
        modbusMasterSetUserPointer(&master, (void *)&offset);
        state = WAIT_START;
        /* fall through */
    case WAIT_START:
        if ((src & MODBUS_MASTER_TRIGGER) && msg->data) {

            i = 0;
            offset = DISCRETE_COUNT;
            state = READ_DI;
        }
        break;

    case READ_DI:
        if (i >= NUM(discrete_inputs)) {

            i = 0;
            offset = INPUT_REG_COUNT;
            state = READ_IR;

        } else {

            *msg->length = modbus_master_read(&discrete_inputs[i++], msg->data);
            xTaskNotify(msg->src, msg->src_bits, eSetBits);
            state = READ_DI_WAIT;
        }
        break;

    case READ_DI_WAIT:
        if (src & MODBUS_MASTER_MSG) {
            if (*msg->length) {
                err =
                    modbusParseResponseRTU(&master,
                                           modbusMasterGetRequest(&master),
                                           modbusMasterGetRequestLength
                                           (&master), msg->data, *msg->length);

                if (!modbusIsOk(err))
                    offset += discrete_inputs[i - 1].count;

            } else {
                /* reply timed-out */
                offset += discrete_inputs[i - 1].count;
            }

            state = READ_DI;
        }
        break;

    case READ_IR:
        if (i >= NUM(input_registers)) {

            i = 0;
            offset = COIL_COUNT;
            state = WRITE_COIL;

        } else {

            *msg->length = modbus_master_read(&input_registers[i++], msg->data);
            xTaskNotify(msg->src, msg->src_bits, eSetBits);
            state = READ_IR_WAIT;
        }
        break;

    case READ_IR_WAIT:
        if (src & MODBUS_MASTER_MSG) {
            if (*msg->length) {
                err =
                    modbusParseResponseRTU(&master,
                                           modbusMasterGetRequest(&master),
                                           modbusMasterGetRequestLength
                                           (&master), msg->data, *msg->length);

                if (!modbusIsOk(err))
                    offset += input_registers[i - 1].count;

            } else {
                /* reply timed-out */
                offset += input_registers[i - 1].count;
            }

            state = READ_IR;
        }
        break;

    case WRITE_COIL:
        if (i >= NUM(coils)) {

            i = 0;
            offset = HOLDING_REG_COUNT;
            state = WRITE_HR;

        } else {

            *msg->length = write_coils(&coils[i++], msg->data);
            xTaskNotify(msg->src, msg->src_bits, eSetBits);
            state = WRITE_COIL_WAIT;
        }
        break;

    case WRITE_COIL_WAIT:
        if (src & MODBUS_MASTER_MSG)
            state = WRITE_COIL;

        break;

    case WRITE_HR:
        if (i >= NUM(holding_registers)) {

            i = 0;
            state = WAIT_START;

        } else {

            *msg->length = write_holding_registers(&coils[i++], msg->data);
            xTaskNotify(msg->src, msg->src_bits, eSetBits);
            state = WRITE_HR_WAIT;
        }
        break;

    case WRITE_HR_WAIT:
        if (src & MODBUS_MASTER_MSG)
            state = WRITE_HR;
    }
}

void modbus_task(void *params)
{
    uint32_t src;
    struct modbus_msg *msgs;
    ModbusErrorInfo err;

    msgs = ((struct task_parameters *)params)->msgs;
    mutex = ((struct task_parameters *)params)->mutex;

    err = modbusMasterInit(&master,
                           master_CB,
                           master_exception_CB,
                           master_allocator,
                           modbusMasterDefaultFunctions,
                           modbusMasterDefaultFunctionCount);

    err = modbusSlaveInit(&slave,
                          slave_CB,
                          slave_exception_CB,
                          slave_allocator,
                          modbusSlaveDefaultFunctions,
                          modbusSlaveDefaultFunctionCount);

    (void)err;

    for (;;) {
        int res = xTaskNotifyWait(0, 0xffffffff, &src, TASK_TICK_TIMEOUT);
        if (res)
            for (int i = 0; i < NUM_SRCS; i++)
                if (src & (1 << i))
                    process_request(&msgs[i]);

        if (MODBUS_MASTER)
            update_master(src, &msgs[MODBUS_MASTER_SRC]);
    }
}
