#include <FreeRTOS.h>
#include <task.h>
#include <semphr.h>

#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/spi.h>

#include "config.h"
#include "hw.h"
#include "uip_task.h"
#include "uart1_task.h"
#include "uart2_task.h"
#include "plc_task.h"

TaskHandle_t uip_notify = NULL;
TaskHandle_t uart1_notify = NULL;
#if defined UART_2 || defined UART_3
TaskHandle_t uart2_notify = NULL;
#endif

uint8_t uip_notify_flag = 0;

static SemaphoreHandle_t mutex = NULL;

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

    if (uip_notify_flag) {
        vTaskNotifyGiveFromISR(uip_notify, &y);
        uip_notify_flag = 0;
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

    mutex = xSemaphoreCreateMutex();

    xTaskCreate(blink_task, "Blink", configMINIMAL_STACK_SIZE, NULL,
                tskIDLE_PRIORITY + 1, NULL);

#ifdef USE_SPI
    spi_setup();
    xTaskCreate(uip_task, "uIP", configMINIMAL_STACK_SIZE * 16, &mutex,
                tskIDLE_PRIORITY + 1, &uip_notify);
#endif

    uart1_setup();
    xTaskCreate(uart1_task, "uart1", configMINIMAL_STACK_SIZE * 2, &mutex,
                tskIDLE_PRIORITY + 1, &uart1_notify);

#if defined UART_2 || defined UART_3
    uart2_setup();
    xTaskCreate(uart2_task, "uart2", configMINIMAL_STACK_SIZE * 2, &mutex,
                tskIDLE_PRIORITY + 1, &uart2_notify);
#endif

    /* PLC task has a higher priority */
    xTaskCreate(plc_task, "PLC", configMINIMAL_STACK_SIZE * 20, &mutex,
                tskIDLE_PRIORITY + 2, NULL);

    vTaskStartScheduler();

    return -1;
}
