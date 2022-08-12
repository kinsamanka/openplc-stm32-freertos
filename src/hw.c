#include <libopencm3/cm3/scb.h>
#include <libopencm3/stm32/adc.h>
#include <libopencm3/stm32/dma.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/stm32/spi.h>
#include <libopencm3/stm32/timer.h>
#ifdef STM32F1
#include <libopencm3/stm32/dac.h>
#endif

#include "config.h"
#include "hw.h"

extern uint16_t QW[];
extern uint16_t IW[];
extern uint8_t QX[];
extern uint8_t IX[];

volatile uint32_t bootflag __attribute__((section(".noinit")));

const struct gpios din[] = INPUTS;

const struct gpios dout[] = OUTPUTS;

#ifdef ANALOG_INPUTS
const struct gpios ain[] = ANALOG_INPUTS;
#endif
#ifdef ANALOG_OUTPUTS
const struct gpios aout[] = ANALOG_OUTPUTS;
#endif

#ifdef ADC_CHANNELS
volatile uint16_t adc_val[] = ADC_CHANNELS;
#else
volatile uint16_t adc_val[2];
#endif

#ifdef TIMER_OC_IDS
const enum tim_oc_id oc_id[] = TIMER_OC_IDS;
#endif

#ifdef DAC_CHANNELS
const int dac_channels[] = DAC_CHANNELS;
#endif

const struct leds run_led = RUN_LED;
#ifdef ERR_LED
const struct leds err_led = ERR_LED;
#endif

const struct uarts uart1 = UART_1;
#ifdef UART_2
const struct uarts uart2 = UART_2;
#endif
#ifdef UART_3
const struct uarts uart3 = UART_3;
#endif
#ifdef UART_4
const struct uarts uart4 = UART_4;
#endif

void update_inputs(void)
{
    for (unsigned i = 0; i < NUM(din); i++)
        IX[i] = (gpio_get(din[i].port, din[i].pin) == 0);

#ifdef ADC_CHANNELS
    for (unsigned i = 0; i < NUM(adc_val); i++)
        IW[i] = adc_val[i];
#endif
}

void update_ouputs(void)
{
    for (unsigned i = 0; i < NUM(dout); i++)
        if (QX[i])
            gpio_set(dout[i].port, dout[i].pin);
        else
            gpio_clear(dout[i].port, dout[i].pin);

#if defined ANALOG_OUTPUTS && defined TIMER_OC_IDS
    for (unsigned i = 0; i < NUM(oc_id); i++)
#ifdef STM32F1
        timer_set_oc_value(TIM3, oc_id[i], QW[i] >> 2);
#else
        timer_set_oc_value(TIM3, oc_id[i], QW[i] >> 3);
#endif
#elif defined DAC_CHANNELS
    for (unsigned i = 0; i < NUM(dac_channels); i++)
        dac_load_data_buffer_single(QW[i], LEFT12, dac_channels[i]);
#endif
}

static void disable_outputs(void)
{
    for (unsigned i = 0; i < NUM(dout); i++)
        gpio_clear(dout[i].port, dout[i].pin);
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

static void spi_set_gpio(void)
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

static void gpio_mode_input(struct gpios p)
{
#ifdef STM32F1
    gpio_set_mode(p.port, GPIO_MODE_INPUT, GPIO_CNF_INPUT_FLOAT, p.pin);
#else
    gpio_mode_setup(p.port, GPIO_MODE_INPUT, GPIO_PUPD_NONE, p.pin);
#endif
}

static void gpio_mode_output(struct gpios p)
{
#ifdef STM32F1
    gpio_set_mode(p.port, GPIO_MODE_OUTPUT_2_MHZ, GPIO_CNF_OUTPUT_PUSHPULL,
                  p.pin);
#else
    gpio_mode_setup(p.port, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, p.pin);
#endif
}

static void gpio_mode_analog_in(struct gpios p)
{
#ifdef STM32F1
    gpio_set_mode(p.port, GPIO_MODE_INPUT, GPIO_CNF_INPUT_ANALOG, p.pin);
#else
    gpio_mode_setup(p.port, GPIO_MODE_ANALOG, GPIO_PUPD_NONE, p.pin);
#endif
}

static void gpio_mode_analog_out(struct gpios p)
{
#if defined USE_DAC_OUT && defined STM32F1
    gpio_set_mode(p.port, GPIO_MODE_INPUT, GPIO_CNF_INPUT_ANALOG, p.pin);
#elif defined STM32F1
    gpio_set_mode(p.port, GPIO_MODE_OUTPUT_50_MHZ,
                  GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, p.pin);
#elif defined STM32F0
    gpio_mode_setup(p.port, GPIO_MODE_AF, GPIO_PUPD_NONE, p.pin);
    gpio_set_af(p.port, p.af, p.pin);
#endif
}

static void uart_set_gpio(struct uarts u)
{
#ifdef STM32F1
    gpio_set_mode(u.port, GPIO_MODE_OUTPUT_50_MHZ,
                  GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, u.tx);
    gpio_set_mode(u.port, GPIO_MODE_INPUT, GPIO_CNF_INPUT_FLOAT, u.rx);
#else
    gpio_mode_setup(u.port, GPIO_MODE_AF, GPIO_PUPD_NONE, u.rx | u.tx);
    gpio_set_af(u.port, GPIO_AF1, u.rx | u.tx);
#endif
    if (u.en.port)
        gpio_mode_output(u.en);
}

void run_bootloader(void)
{
    disable_outputs();
    bootflag = 0xDEADBEEF;      /* set flag */
    scb_reset_system();
}

void clock_setup(void)
{
#ifdef STM32F1
    rcc_clock_setup_in_hse_8mhz_out_72mhz();
    rcc_periph_clock_enable(RCC_AFIO);
#else
    rcc_clock_setup_in_hsi_out_48mhz();
    rcc_periph_clock_enable(RCC_SYSCFG_COMP);
#endif
    rcc_periph_clock_enable(RCC_GPIOA);
    rcc_periph_clock_enable(RCC_GPIOB);
    rcc_periph_clock_enable(RCC_GPIOC);
    rcc_periph_clock_enable(RCC_GPIOD);
    rcc_periph_clock_enable(RCC_GPIOE);
#ifdef RCC_GPIOF
    rcc_periph_clock_enable(RCC_GPIOF);
#endif
#ifdef RCC_GPIOG
    rcc_periph_clock_enable(RCC_GPIOG);
#endif
    rcc_periph_clock_enable(RCC_USART1);
#ifdef UART_2
    rcc_periph_clock_enable(RCC_USART2);
#endif
#ifdef UART_3
    rcc_periph_clock_enable(RCC_USART3);
#endif
#ifdef UART_4
    rcc_periph_clock_enable(RCC_UART4);
#endif
    rcc_periph_clock_enable(RCC_SPI1);
    rcc_periph_clock_enable(RCC_DMA1);
#if   defined ANALOG_INPUTS && defined STM32F0
    rcc_periph_clock_enable(RCC_ADC);
#elif defined ANALOG_INPUTS && defined STM32F1
    rcc_periph_clock_enable(RCC_ADC1);
#endif
#if defined ANALOG_OUTPUTS && defined USE_DAC_OUT
    rcc_periph_clock_enable(RCC_DAC);
#elif defined ANALOG_OUTPUTS
    rcc_periph_clock_enable(RCC_TIM3);
#endif
}

void gpio_setup(void)
{
#if defined DISABLE_JTAG_SWD && defined STM32F1
    gpio_primary_remap(AFIO_MAPR_SWJ_CFG_JTAG_OFF_SW_OFF, 0);
#endif

    gpio_mode_output(run_led.gpio);
    if (run_led.inv)
        gpio_set(run_led.gpio.port, run_led.gpio.pin);
#ifdef ERR_LED
    gpio_mode_output(err_led.gpio);
    if (err_led.inv)
        gpio_set(err_led.gpio.port, err_led.gpio.pin);
#endif

    ct_assert(NUM(din) <= DISCRETE_COUNT);  /* INPUTS > DISCRETE COUNT */
    for (unsigned i = 0; i < NUM(din); i++)
        gpio_mode_input(din[i]);

    ct_assert(NUM(dout) <= COIL_COUNT);     /* OUTPUTS > COIL COUNT */
    for (unsigned i = 0; i < NUM(dout); i++)
        gpio_mode_output(dout[i]);

#ifdef ANALOG_INPUTS
    ct_assert(NUM(ain) <= INPUT_REG_COUNT); /* ANALOG IN > INPUT REGISTERS */
    for (unsigned i = 0; i < NUM(ain); i++)
        gpio_mode_analog_in(ain[i]);
#endif
#ifdef ANALOG_OUTPUTS
    ct_assert(NUM(aout) <= HOLDING_REG_COUNT);  /* ANALOG OUT > HOLDING REGISTERS */
    for (unsigned i = 0; i < NUM(aout); i++)
        gpio_mode_analog_out(aout[i]);
#endif

    uart_set_gpio(uart1);
#ifdef UART_2
    uart_set_gpio(uart2);
#endif
#ifdef UART_3
    uart_set_gpio(uart3);
#endif
#ifdef UART_4
    uart_set_gpio(uart4);
#endif

#ifdef USE_SPI
    spi_set_gpio();
#endif
}

static void adc_dma_setup(void)
{
    dma_channel_reset(DMA1, DMA_CHANNEL1);

    dma_set_peripheral_address(DMA1, DMA_CHANNEL1, (uint32_t) & ADC1_DR);

    dma_set_memory_address(DMA1, DMA_CHANNEL1, (uint32_t) adc_val);
    dma_set_number_of_data(DMA1, DMA_CHANNEL1, NUM(adc_val));

    dma_set_read_from_peripheral(DMA1, DMA_CHANNEL1);

    dma_enable_memory_increment_mode(DMA1, DMA_CHANNEL1);
    dma_set_peripheral_size(DMA1, DMA_CHANNEL1, DMA_CCR_PSIZE_16BIT);
    dma_set_memory_size(DMA1, DMA_CHANNEL1, DMA_CCR_MSIZE_16BIT);

    dma_enable_circular_mode(DMA1, DMA_CHANNEL1);

    dma_set_priority(DMA1, DMA_CHANNEL1, DMA_CCR_PL_HIGH);

    dma_enable_channel(DMA1, DMA_CHANNEL1);
}

void adc_setup(void)
{
#ifdef ADC_CHANNELS
    uint8_t adc_channels[] = ADC_CHANNELS;
#else
    uint8_t adc_channels[] = { 16, 17 };
#endif

    adc_power_off(ADC1);

#ifdef STM32F1
    adc_enable_scan_mode(ADC1);
    adc_set_single_conversion_mode(ADC1);
#else
    adc_set_clk_source(ADC1, ADC_CLKSOURCE_ADC);
    adc_calibrate(ADC1);
    adc_set_operation_mode(ADC1, ADC_MODE_SCAN);
    adc_set_resolution(ADC1, ADC_RESOLUTION_12BIT);
#endif

    adc_disable_external_trigger_regular(ADC1);
    adc_set_right_aligned(ADC1);

#ifdef STM32F1
    adc_enable_external_trigger_regular(ADC1, ADC_CR2_EXTSEL_SWSTART);

    adc_set_sample_time_on_all_channels(ADC1, ADC_SMPR_SMP_239DOT5CYC);
#else
    adc_set_sample_time_on_all_channels(ADC1, ADC_SMPTIME_239DOT5);
#endif

    adc_set_regular_sequence(ADC1, NUM(adc_channels), adc_channels);

    adc_enable_dma(ADC1);

    adc_power_on(ADC1);

#ifdef STM32F1
    for (int i = 0; i < 800000; i++)    /* Wait a bit. */
        __asm__("nop");

    adc_reset_calibration(ADC1);
    adc_calibrate(ADC1);
#endif

    adc_dma_setup();
}

void adc_start(void)
{
    adc_start_conversion_regular(ADC1);
}

#ifdef TIMER_OC_IDS
void timer3_setup(void)
{
    timer_set_mode(TIM3, TIM_CR1_CKD_CK_INT, TIM_CR1_CMS_EDGE, TIM_CR1_DIR_UP);
#ifdef STM32F1
    timer_set_period(TIM3, 0xffff >> 2);    /* ~4 kHz */
#else
    timer_set_period(TIM3, 0xffff >> 3);    /* ~6 kHz */
#endif
    timer_set_prescaler(TIM3, 0);
    timer_generate_event(TIM3, TIM_EGR_UG);

    for (unsigned i = 0; i < NUM(oc_id); i++) {
        timer_set_oc_mode(TIM3, oc_id[i], TIM_OCM_PWM1);
        timer_enable_oc_preload(TIM3, oc_id[i]);
        timer_enable_oc_output(TIM3, oc_id[i]);
        timer_set_oc_polarity_high(TIM3, oc_id[i]);
        timer_set_oc_value(TIM3, oc_id[i], 0);
    }

    timer_enable_preload(TIM3);
    timer_enable_counter(TIM3);
}
#endif

#ifdef DAC_CHANNELS
void dac_setup(void)
{
    for (unsigned i = 0; i < NUM(dac_channels); i++) {
        dac_disable(dac_channels[i]);
        dac_disable_waveform_generation(dac_channels[i]);
        dac_enable(dac_channels[i]);
    }
}
#endif
