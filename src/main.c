#include <FreeRTOS.h>
#include <task.h>
#include <semphr.h>

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/spi.h>
#include <libopencm3/cm3/systick.h>

#include "config.h"
#include "uip_task.h"
#include "plc_task.h"

uint8_t uip_notify_flag = 0;
static SemaphoreHandle_t mutex = NULL;

/******************************************************************************/

static void clock_setup(void)
{
    rcc_clock_setup_in_hse_8mhz_out_72mhz();

    /* Enable Periperal clocks. */
    rcc_periph_clock_enable(RCC_GPIOA);
    rcc_periph_clock_enable(RCC_GPIOB);
    rcc_periph_clock_enable(RCC_GPIOC);
    rcc_periph_clock_enable(RCC_USART1);
    rcc_periph_clock_enable(RCC_USART2);
    rcc_periph_clock_enable(RCC_AFIO);
    rcc_periph_clock_enable(RCC_SPI1);
}

/******************************************************************************/

static void spi_setup(void)
{
    /* Configure GPIOs:
       SS   = PA4
       SCK  = PA5
       MISO = PA6
       MOSI = PA7
     */
    gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_50_MHZ,
                  GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, GPIO5 | GPIO7);

    gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_2_MHZ,
                  GPIO_CNF_OUTPUT_PUSHPULL, GPIO4);

    gpio_set_mode(GPIOA, GPIO_MODE_INPUT, GPIO_CNF_INPUT_FLOAT, GPIO6);

    spi_reset(SPI1);

    /* note some clone STM32s cannot handle FPCLK_DIV_2 clock rate! */
    spi_init_master(SPI1, SPI_CR1_BAUDRATE_FPCLK_DIV_2,
                    SPI_CR1_CPOL_CLK_TO_0_WHEN_IDLE,
                    SPI_CR1_CPHA_CLK_TRANSITION_1, SPI_CR1_DFF_8BIT,
                    SPI_CR1_MSBFIRST);

    spi_enable_software_slave_management(SPI1);
    spi_set_nss_high(SPI1);

    spi_enable(SPI1);
}

/******************************************************************************/

static void gpio_setup(void)
{
    /* Q0.0-Q0.4, W5500 PWR */
    gpio_set_mode(GPIOB, GPIO_MODE_OUTPUT_2_MHZ,
                  GPIO_CNF_OUTPUT_PUSHPULL,
                  GPIO5 | GPIO6 | GPIO7 | GPIO10 | GPIO11 | GPIO9);

    /* I0.0-I0.4 */
    gpio_set_mode(GPIOB, GPIO_MODE_INPUT, GPIO_CNF_INPUT_FLOAT,
                  GPIO1 | GPIO12 | GPIO13 | GPIO14 | GPIO15);

    /* W5500 interrupt pin */
    gpio_set_mode(GPIOB, GPIO_MODE_INPUT, GPIO_CNF_INPUT_FLOAT, GPIO0);

    /* W5500 reset, EN 485, SYS LED */
    gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_2_MHZ,
                  GPIO_CNF_OUTPUT_PUSHPULL, GPIO0 | GPIO1 | GPIO8);

    /* UART1 & UART2 */
    gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_50_MHZ,
                  GPIO_CNF_OUTPUT_ALTFN_PUSHPULL,
                  GPIO_USART1_TX | GPIO_USART2_TX);

    gpio_set_mode(GPIOA, GPIO_MODE_INPUT, GPIO_CNF_INPUT_FLOAT,
                  GPIO_USART1_RX | GPIO_USART2_RX);

    /* turn on W5500 & keep reset low */
    gpio_clear(GPIOA, GPIO0);
    gpio_set(GPIOB, GPIO9);
}

/******************************************************************************/

static void blink_task(void *params)
{
    (void)params;

    for (;;) {
        vTaskDelay(600 / portTICK_RATE_MS);
        gpio_toggle(GPIOA, GPIO8);
    }
}

/******************************************************************************/

TaskHandle_t uip_notify = NULL;

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
    spi_setup();

    mutex = xSemaphoreCreateMutex();

    xTaskCreate(blink_task, "Blink", configMINIMAL_STACK_SIZE, NULL,
                tskIDLE_PRIORITY + 1, NULL);

    xTaskCreate(uip_task, "uIP", configMINIMAL_STACK_SIZE * 4, &mutex,
                tskIDLE_PRIORITY + 1, &uip_notify);

    /* PLC task has a higher priority */
    xTaskCreate(plc_task, "PLC", configMINIMAL_STACK_SIZE * 4, &mutex,
                tskIDLE_PRIORITY + 2, NULL);

    vTaskStartScheduler();

    return -1;
}
