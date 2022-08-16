#include <FreeRTOS.h>
#include "uip.h"
#include "modbus_tcp.h"
#include "task_params.h"

#include <string.h>

static TaskHandle_t modbus_slave;
static struct modbus_slave_msg msg;
static TaskHandle_t notify;

static struct modbus_tcp_state s;

static int handle_request(TaskHandle_t mbs, struct modbus_slave_msg *msg)
{
    uint32_t result;

    while (!xTaskNotifyIndexed
           (mbs, 1, (uint32_t) msg, eSetValueWithoutOverwrite))
        vTaskDelay(10);

    xTaskNotifyWaitIndexed(1, 0x00, 0xffffffff, &result, portMAX_DELAY);

    return result;
}

void modbus_tcp_init(void *params)
{
    modbus_slave = ((struct task_parameters *)params)->modbus_slave;
    notify = ((struct task_parameters *)params)->uip;
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

        struct modbus_slave_msg msg = {
            dat,
            &sz,
            notify,
        };

        if (handle_request(modbus_slave, &msg)) {
            memcpy(s.data, dat, sz);
            s.len = sz;
        } else {
            s.len = 0;
        }
    }

    if ((uip_rexmit() || uip_newdata()) && s.len)
        uip_send(s.data, s.len);
}
