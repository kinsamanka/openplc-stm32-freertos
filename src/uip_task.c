#include <FreeRTOS.h>
#include <task.h>
#include <semphr.h>

#include <uip/uip.h>
#include <uip/uip_arp.h>
#include <uip/timer.h>

#include <libopencm3/stm32/gpio.h>

#include "config.h"
#include "tapdev.h"
#include "modbus_tcp.h"
#include "uip_task.h"

#define BUF ((struct uip_eth_hdr *)&uip_buf[0])

/******************************************************************************/

clock_time_t clock_time(void)
{
    return xTaskGetTickCount();
}

/******************************************************************************/

void uip_task(void *params)
{
    SemaphoreHandle_t *mutex = (SemaphoreHandle_t *) params;

    uip_ipaddr_t ipaddr;
    struct timer periodic_timer, arp_timer;

    timer_set(&periodic_timer, configTICK_RATE_HZ / 2);
    timer_set(&arp_timer, configTICK_RATE_HZ * 10);

    /* release reset pin */
    vTaskDelay(50 / portTICK_RATE_MS);
    gpio_set(GPIOA, GPIO0);
    vTaskDelay(50 / portTICK_RATE_MS);

    tapdev_init();
    uip_init();

    uip_ipaddr(ipaddr, configIP_ADDR0, configIP_ADDR1, configIP_ADDR2,
               configIP_ADDR3);
    uip_sethostaddr(ipaddr);
    uip_ipaddr(ipaddr, configNET_MASK0, configNET_MASK1, configNET_MASK2,
               configNET_MASK3);
    uip_setnetmask(ipaddr);

    modbus_tcp_init(mutex);

    int i;
    for (;;) {
        uip_len = tapdev_read();
        if (uip_len > 0) {
            if (BUF->type == htons(UIP_ETHTYPE_IP)) {
                uip_arp_ipin();
                uip_input();
                /* If the above function invocation resulted in data that
                   should be sent out on the network, the global variable
                   uip_len is set to a value > 0. */
                if (uip_len > 0) {
                    uip_arp_out();
                    tapdev_send();
                }
            } else if (BUF->type == htons(UIP_ETHTYPE_ARP)) {
                uip_arp_arpin();
                /* If the above function invocation resulted in data that
                   should be sent out on the network, the global variable
                   uip_len is set to a value > 0. */
                if (uip_len > 0) {
                    tapdev_send();
                }
            }

        } else if (timer_expired(&periodic_timer)) {
            timer_reset(&periodic_timer);
            for (i = 0; i < UIP_CONNS; i++) {
                uip_periodic(i);
                /* If the above function invocation resulted in data that
                   should be sent out on the network, the global variable
                   uip_len is set to a value > 0. */
                if (uip_len > 0) {
                    uip_arp_out();
                    tapdev_send();
                }
            }

            /* Call the ARP timer function every 10 seconds. */
            if (timer_expired(&arp_timer)) {
                timer_reset(&arp_timer);
                uip_arp_timer();
            }
        }
    }
}

/******************************************************************************/
