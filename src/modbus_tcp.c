#include "uip.h"
#include "modbus_tcp.h"

#include <string.h>

static ModbusErrorInfo mberr;
static ModbusSlave tcp_slave;

static struct modbus_tcp_state s;
static uint8_t tcp_slave_buf[MAX_RESPONSE];
static struct context context;

static int tcp_mb_handle_req(uint8_t ** dat, uint8_t * sz)
{
    mberr = modbusParseRequestTCP(&tcp_slave, *dat, *sz);

    switch (modbusGetGeneralError(mberr)) {

    case MODBUS_OK:
        break;

    case MODBUS_ERROR_ALLOC:
        if (*sz < 2)
            break;
        mberr = modbusBuildExceptionRTU(&tcp_slave,
                                        SLAVE_ADDRESS,
                                        *dat[1], MODBUS_EXCEP_SLAVE_FAILURE);
        break;

    default:
        return 0;
    }

    // return response if everything is OK
    if (modbusIsOk(mberr) && modbusSlaveGetResponseLength(&tcp_slave)) {
        *sz = modbusSlaveGetResponseLength(&tcp_slave);
        *dat = (uint8_t *) modbusSlaveGetResponse(&tcp_slave);
        return 1;
    } else {
        return 0;
    }
}

void modbus_tcp_init(SemaphoreHandle_t * mutex)
{
    context.buf = tcp_slave_buf;
    context.mutex = mutex;

    uip_listen(HTONS(configMODBUS_PORT));

    mberr = modbusSlaveInit(&tcp_slave,
                            regCallback,
                            NULL,
                            staticAllocator,
                            modbusSlaveDefaultFunctions,
                            modbusSlaveDefaultFunctionCount);

    modbusSlaveSetUserPointer(&tcp_slave, (void *)&context);
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
        if (tcp_mb_handle_req(&dat, &sz)) {
            memcpy(s.data, dat, sz);
            s.len = sz;
        } else {
            s.len = 0;
        }
    }

    if ((uip_rexmit() || uip_newdata()) && s.len)
        uip_send(s.data, s.len);
}
