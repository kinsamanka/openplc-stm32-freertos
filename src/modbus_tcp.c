#include <FreeRTOS.h>
#include "uip.h"
#include "modbus_tcp.h"
#include "task_params.h"

#include <string.h>

static TaskHandle_t modbus;
static TaskHandle_t notify;

static struct modbus_tcp_state s;
static struct modbus_msg *msg;

static int handle_request(void)
{
    uint32_t result;

    xTaskNotify(modbus, (uint32_t) 1 << UIP_TCP, eSetBits);

    xTaskNotifyWait(0, 0xffffffff, &result, portMAX_DELAY);

    return ((result & (TCP_SRC << 1)) != 0);
}

void modbus_tcp_init(void *params)
{
    uip_listen(HTONS(configMODBUS_PORT));

    modbus = ((struct task_parameters *)params)->modbus;
    notify = ((struct task_parameters *)params)->uip;
    msg = &(((struct task_parameters *)params)->msgs[UIP_TCP]);
    *msg = (struct modbus_msg) {
        .data = NULL,
        .length = NULL,
        .src = notify,
        .src_bits = TCP_SRC,
        .rtu_flag = 0,
    };
}

void modbus_tcp_appcall(void)
{
    uint8_t *dat;
    uint16_t sz;

    if (uip_acked())
        s.len = 0;

    if (uip_newdata()) {
        sz = uip_datalen();
        dat = uip_appdata;

        msg->data = s.data;
        msg->length = &s.len;

        s.len = (sz < NUM(s.data)) ? sz : NUM(s.data);
        memcpy(s.data, dat, s.len);

        if (!handle_request())
            s.len = 0;
    }

    if ((uip_rexmit() || uip_newdata()) && s.len)
        uip_send(s.data, s.len);
}
