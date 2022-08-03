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
#include "plc_task.h"

TaskHandle_t uip_notify = NULL;
TaskHandle_t uart_notify = NULL;

uint8_t uip_notify_flag = 0;

static SemaphoreHandle_t mutex = NULL;

/******************************************************************************/

static void clock_setup(void)
{
#ifdef STM32F1
    rcc_clock_setup_in_hse_8mhz_out_72mhz();

    rcc_periph_clock_enable(RCC_AFIO);
#else
    rcc_clock_setup_in_hsi_out_48mhz();

    rcc_periph_clock_enable(RCC_SYSCFG_COMP);
    rcc_periph_clock_enable(RCC_GPIOF);
#endif

    rcc_periph_clock_enable(RCC_GPIOA);
    rcc_periph_clock_enable(RCC_GPIOB);
    rcc_periph_clock_enable(RCC_GPIOC);
    rcc_periph_clock_enable(RCC_USART1);
    rcc_periph_clock_enable(RCC_USART2);
    rcc_periph_clock_enable(RCC_SPI1);
    rcc_periph_clock_enable(RCC_DMA1);
}

/******************************************************************************/

static void gpio_setup(void)
{
    hw_io_setup();
#ifdef USE_SPI
    spi_set_gpio();
#endif
#ifdef USE_UART1
    uart_set_gpio(GPIOA, GPIO_USART1_RX, GPIO_USART1_TX);
#endif
#ifdef USE_UART2
    uart_set_gpio(GPIOA, GPIO_USART2_RX, GPIO_USART2_TX);

    /* RS485 TX enable pin */
    gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_2_MHZ,
                  GPIO_CNF_OUTPUT_PUSHPULL, GPIO1);
#endif
}

/******************************************************************************/

static void blink_task(void *params)
{
    (void)params;

    for (;;) {
        vTaskDelay(600 / portTICK_RATE_MS);
        gpio_toggle(PORT_LED, PIN_LED);
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

    mutex = xSemaphoreCreateMutex();

    xTaskCreate(blink_task, "Blink", configMINIMAL_STACK_SIZE, NULL,
                tskIDLE_PRIORITY + 1, NULL);

#ifdef USE_SPI
    spi_setup();
    xTaskCreate(uip_task, "uIP", configMINIMAL_STACK_SIZE * 16, &mutex,
                tskIDLE_PRIORITY + 1, &uip_notify);
#endif

#ifdef USE_UART1
    uart1_setup();
    xTaskCreate(uart1_task, "uart1", configMINIMAL_STACK_SIZE * 2, &mutex,
                tskIDLE_PRIORITY + 1, &uart_notify);
#endif

    /* PLC task has a higher priority */
    xTaskCreate(plc_task, "PLC", configMINIMAL_STACK_SIZE * 20, &mutex,
                tskIDLE_PRIORITY + 2, NULL);

    vTaskStartScheduler();

    return -1;
}
