#include <FreeRTOS.h>
#include <task.h>
#include <semphr.h>

#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/spi.h>

#include "config.h"
#include "hw.h"
#include "uip_task.h"
#include "uart_task.h"
#include "plc_task.h"
#include "modbus.h"
#include "task_params.h"

static struct task_parameters task_params;
static struct modbus_msg msgs[NUM_SRCS];

/******************************************************************************/

static void blink_task(void *params)
{
    (void)params;
    const struct leds led = RUN_LED;

    for (;;) {
        vTaskDelay(600 / portTICK_RATE_MS);
        gpio_toggle(led.gpio.port, led.gpio.pin);
    }
}

/******************************************************************************/

void vApplicationTickHook(void)
{
    BaseType_t y = pdFALSE;

    if (task_params.uip_notify_flag) {
        vTaskNotifyGiveFromISR(task_params.uip, &y);
        task_params.uip_notify_flag = 0;
    }

    portYIELD_FROM_ISR(y);
}

/******************************************************************************/

int main(void)
{
    clock_setup();
    gpio_setup();

#ifdef ANALOG_INPUTS
    adc_setup();
#endif
#ifdef TIMER_OC_IDS
    timer3_setup();
#endif
#ifdef DAC_CHANNELS
    dac_setup();
#endif

    task_params.mutex = xSemaphoreCreateMutex();
    task_params.msgs = msgs;

    xTaskCreate(blink_task, "Blink", configMINIMAL_STACK_SIZE, NULL,
                tskIDLE_PRIORITY + 1, NULL);

#ifdef USE_SPI
    spi_setup();
    xTaskCreate(uip_task, "uIP", configMINIMAL_STACK_SIZE * 16, &task_params,
                tskIDLE_PRIORITY + 1, &task_params.uip);
#endif
    xTaskCreate(uart_task, "uart", configMINIMAL_STACK_SIZE * 2, &task_params,
                tskIDLE_PRIORITY + 2, &task_params.uart);

    xTaskCreate(modbus_task, "modbus", configMINIMAL_STACK_SIZE * 4,
                &task_params, tskIDLE_PRIORITY + 2, &task_params.modbus);

    /* PLC task has the highest priority */
    xTaskCreate(plc_task, "PLC", configMINIMAL_STACK_SIZE * 20, &task_params,
                tskIDLE_PRIORITY + 3, NULL);

    vTaskStartScheduler();

    return -1;
}
