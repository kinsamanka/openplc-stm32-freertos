#include "config.h"
#include "hw.h"

#include <libopencm3/cm3/scb.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/stm32/spi.h>

#define SET_PIN(a, b, c) \
    do { \
        if (a) \
            gpio_set(b, c); \
        else \
            gpio_clear(b, c); \
    } while (0)

#define GET_PIN(a, b, c) \
    do { \
        a = (gpio_get(b, c) == 0); \
    } while (0)

extern uint16_t QW[];
extern uint16_t IW[];
extern uint8_t QX[];
extern uint8_t IX[];

static const uint32_t BOOT_ADDR = BOOTLOADER_ADDR;
volatile uint32_t bootflag __attribute__((section(".noinit")));

void update_inputs(void)
{
#ifdef STM32F1
    GET_PIN(IX[0], GPIOB, GPIO1);
    GET_PIN(IX[1], GPIOB, GPIO12);
    GET_PIN(IX[2], GPIOB, GPIO13);
    GET_PIN(IX[3], GPIOB, GPIO14);
    GET_PIN(IX[4], GPIOB, GPIO15);
#else
    GET_PIN(IX[0], GPIOA, GPIO0);
    GET_PIN(IX[1], GPIOB, GPIO3);
    GET_PIN(IX[2], GPIOB, GPIO4);
    GET_PIN(IX[3], GPIOA, GPIO1);
    GET_PIN(IX[4], GPIOF, GPIO1);
    GET_PIN(IX[5], GPIOC, GPIO14);
    GET_PIN(IX[6], GPIOC, GPIO15);
    GET_PIN(IX[7], GPIOB, GPIO9);
#endif
}

void update_ouputs(void)
{
#ifdef STM32F1
    SET_PIN(QX[0], GPIOB, GPIO5);
    SET_PIN(QX[1], GPIOB, GPIO6);
    SET_PIN(QX[2], GPIOB, GPIO7);
    SET_PIN(QX[3], GPIOB, GPIO10);
    SET_PIN(QX[4], GPIOB, GPIO11);
#else
    SET_PIN(QX[0], GPIOA, GPIO4);
    SET_PIN(QX[1], GPIOA, GPIO7);
    SET_PIN(QX[2], GPIOB, GPIO14);
    SET_PIN(QX[3], GPIOB, GPIO15);
    SET_PIN(QX[4], GPIOA, GPIO12);
    SET_PIN(QX[5], GPIOA, GPIO11);
#endif
}

#ifdef USE_SPI
void spi_setup(void)
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

void spi_set_gpio(void)
{
    /* W5500 PWR */
    gpio_set_mode(GPIOB, GPIO_MODE_OUTPUT_2_MHZ,
                  GPIO_CNF_OUTPUT_PUSHPULL, GPIO9);

    /* W5500 interrupt pin */
    gpio_set_mode(GPIOB, GPIO_MODE_INPUT, GPIO_CNF_INPUT_FLOAT, GPIO0);

    /* W5500 reset */
    gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_2_MHZ,
                  GPIO_CNF_OUTPUT_PUSHPULL, GPIO0);

    /* turn on W5500 & keep reset low */
    gpio_clear(GPIOA, GPIO0);
    gpio_set(GPIOB, GPIO9);
}
#endif

void hw_io_setup(void)
{
#ifdef STM32F1
    /* SYS LED */
    gpio_set_mode(PORT_LED, GPIO_MODE_OUTPUT_2_MHZ,
                  GPIO_CNF_OUTPUT_PUSHPULL, PIN_LED);

    /* Q0.0-Q0.4 */
    gpio_set_mode(GPIOB, GPIO_MODE_OUTPUT_2_MHZ,
                  GPIO_CNF_OUTPUT_PUSHPULL,
                  GPIO5 | GPIO6 | GPIO7 | GPIO10 | GPIO11);

    /* I0.0-I0.4 */
    gpio_set_mode(GPIOB, GPIO_MODE_INPUT, GPIO_CNF_INPUT_FLOAT,
                  GPIO1 | GPIO12 | GPIO13 | GPIO14 | GPIO15);
#else
    /* RUN LED */
    gpio_mode_setup(PORT_LED, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, PIN_LED);

    /* ERR LED */
    gpio_mode_setup(GPIOB, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO11);
    gpio_set(GPIOB, GPIO11);    /* turn off */

    /* Q0.0, Q0.1, Q0.4, Q0.5 */
    gpio_mode_setup(GPIOA, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO4 | GPIO7 |
                    GPIO12 | GPIO11);

    /* Q0.2, Q0.3 */
    gpio_mode_setup(GPIOB, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO14 | GPIO15);

    /* I0.0, I0.3 */
    gpio_mode_setup(GPIOA, GPIO_MODE_INPUT, GPIO_PUPD_NONE, GPIO0 | GPIO1);

    /* I0.1, I0.2, I0.7 */
    gpio_mode_setup(GPIOB, GPIO_MODE_INPUT, GPIO_PUPD_NONE, GPIO3 | GPIO4 |
                    GPIO9);

    /* I0.5, I0.6 */
    gpio_mode_setup(GPIOC, GPIO_MODE_INPUT, GPIO_PUPD_NONE, GPIO14 | GPIO15);

    /* I0.4 */
    gpio_mode_setup(GPIOF, GPIO_MODE_INPUT, GPIO_PUPD_NONE, GPIO1);
#endif
}

void uart_set_gpio(uint32_t gpioport, uint16_t rxpin, uint16_t txpin)
{
#ifdef STM32F0
    gpio_mode_setup(gpioport, GPIO_MODE_AF, GPIO_PUPD_NONE, rxpin | txpin);
    gpio_set_af(gpioport, GPIO_AF1, rxpin | txpin);
#else
    gpio_set_mode(gpioport, GPIO_MODE_OUTPUT_50_MHZ,
                  GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, txpin);
    gpio_set_mode(gpioport, GPIO_MODE_INPUT, GPIO_CNF_INPUT_FLOAT, rxpin);
#endif
}

static void disable_output(void)
{
    SET_PIN(0, GPIOB, GPIO5);
    SET_PIN(0, GPIOB, GPIO6);
    SET_PIN(0, GPIOB, GPIO7);
    SET_PIN(0, GPIOB, GPIO10);
    SET_PIN(0, GPIOB, GPIO11);
}

void run_bootloader(void)
{
    disable_output();
    bootflag = 0xDEADBEEF;      /* set flag */

    scb_reset_system();
}

