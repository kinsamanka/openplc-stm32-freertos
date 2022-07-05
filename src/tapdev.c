#include <FreeRTOS.h>
#include <task.h>

#include <uip/uip.h>
#include <uip/uip_arp.h>
#include <uip/timer.h>

#include "config.h"
#include "tapdev.h"
#include "w5500_connector.h"

static uint8_t mac[] =
    { configMAC_ADDR0, configMAC_ADDR1, configMAC_ADDR2, configMAC_ADDR3,
    configMAC_ADDR4, configMAC_ADDR5
};

/******************************************************************************/

unsigned int tapdev_read(void)
{
    return w5500_readFrame(uip_buf, UIP_BUFSIZE);
}

/******************************************************************************/

void tapdev_send(void)
{
    w5500_sendFrame(uip_buf, uip_len);
}

/******************************************************************************/

void tapdev_init(void)
{
    struct uip_eth_addr uip_mac;
    int i;

    w5500_init(mac);
    for (i = 0; i < 6; i++) {
        uip_mac.addr[i] = mac[i];
    }

    uip_setethaddr(uip_mac);
}
