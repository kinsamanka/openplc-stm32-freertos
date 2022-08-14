#include "modbus_slave.h"
#include <task.h>
#include <queue.h>

#include <libopencm3/cm3/nvic.h>
#include <libopencm3/stm32/dma.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/usart.h>

#include <lightmodbus/lightmodbus.h>

#include <string.h>

#include "config.h"
#include "hw.h"

#if defined STM32F1 && (defined UART_2 || defined UART_3)

#include "hw.h"
#include "uart2_task.h"

#define IDLE_BIT                    0x01
#define DMA_RX_TC_BIT               0x02
#define DMA_TX_TC_BIT               0x04
#define TC_BIT                      0x08

#ifdef UART_2

#define _DMA_RX_ISR_                dma1_channel6_isr
#define _DMA_TX_ISR_                dma1_channel7_isr
#define _USART_ISR_                 usart2_isr

#define _USART_                     USART2
#define _USART_DR_                  USART2_DR

#define _DMA_CHANNEL_RX_            DMA_CHANNEL6
#define _DMA_CHANNEL_TX_            DMA_CHANNEL7

#define _NVIC_USART_IRQ_            NVIC_USART2_IRQ

#define _NVIC_DMA1_CHANNEL_RX_IRQ_  NVIC_DMA1_CHANNEL6_IRQ
#define _NVIC_DMA1_CHANNEL_TX_IRQ_  NVIC_DMA1_CHANNEL7_IRQ

#else                           /* UART_3 */

#define _DMA_RX_ISR_                dma1_channel3_isr
#define _DMA_TX_ISR_                dma1_channel2_isr
#define _USART_ISR_                 usart3_isr

#define _USART_                     USART3
#define _USART_DR_                  USART3_DR

#define _DMA_CHANNEL_RX_            DMA_CHANNEL3
#define _DMA_CHANNEL_TX_            DMA_CHANNEL2

#define _NVIC_USART_IRQ_            NVIC_USART3_IRQ

#define _NVIC_DMA1_CHANNEL_RX_IRQ_  NVIC_DMA1_CHANNEL3_IRQ
#define _NVIC_DMA1_CHANNEL_TX_IRQ_  NVIC_DMA1_CHANNEL2_IRQ

#endif

#define _DMA_RX_COUNT_              (UART_BUF_LEN - DMA_CNDTR(DMA1, _DMA_CHANNEL_RX_))

#ifdef UART_2
const struct uarts uart = UART_2;
#endif
#ifdef UART_3
const struct uarts uart = UART_3;
#endif

static struct {
    char data[UART_BUF_LEN];
    uint8_t length;
} uart_buf;

static ModbusErrorInfo mberr;
static ModbusSlave rtu_slave;

static uint8_t rtu_slave_buf[MAX_RESPONSE];
static struct context context;

extern TaskHandle_t uart2_notify;

static volatile int skip_usart_idle = 0;

void _DMA_RX_ISR_(void)
{
    portBASE_TYPE y = pdFALSE;

    if (dma_get_interrupt_flag(DMA1, _DMA_CHANNEL_RX_, DMA_TCIF)) {

        dma_clear_interrupt_flags(DMA1, _DMA_CHANNEL_RX_, DMA_TCIF);

        /* ignore the IDLE interrupt after DMA is done */
        skip_usart_idle = 1;

        xTaskNotifyFromISR(uart2_notify, DMA_RX_TC_BIT, eSetBits, &y);
    }

    portYIELD_FROM_ISR(y);
}

void _DMA_TX_ISR_(void)
{
    portBASE_TYPE y = pdFALSE;

    if (dma_get_interrupt_flag(DMA1, _DMA_CHANNEL_TX_, DMA_TCIF)) {

        dma_clear_interrupt_flags(DMA1, _DMA_CHANNEL_TX_, DMA_TCIF);

        xTaskNotifyFromISR(uart2_notify, DMA_TX_TC_BIT, eSetBits, &y);
    }

    portYIELD_FROM_ISR(y);
}

static void dma_setup(void)
{
    /* USART RX DMA channel */
    dma_channel_reset(DMA1, _DMA_CHANNEL_RX_);

    dma_set_peripheral_address(DMA1, _DMA_CHANNEL_RX_, (uint32_t) & _USART_DR_);
    dma_set_memory_address(DMA1, _DMA_CHANNEL_RX_, (uint32_t) uart_buf.data);
    dma_set_number_of_data(DMA1, _DMA_CHANNEL_RX_, UART_BUF_LEN);
    dma_set_read_from_peripheral(DMA1, _DMA_CHANNEL_RX_);
    dma_enable_memory_increment_mode(DMA1, _DMA_CHANNEL_RX_);
    dma_set_peripheral_size(DMA1, _DMA_CHANNEL_RX_, DMA_CCR_PSIZE_8BIT);
    dma_set_memory_size(DMA1, _DMA_CHANNEL_RX_, DMA_CCR_MSIZE_8BIT);
    dma_set_priority(DMA1, _DMA_CHANNEL_RX_, DMA_CCR_PL_HIGH);

    dma_enable_transfer_complete_interrupt(DMA1, _DMA_CHANNEL_RX_);

    /* USART TX DMA channel */
    dma_channel_reset(DMA1, _DMA_CHANNEL_TX_);

    dma_set_peripheral_address(DMA1, _DMA_CHANNEL_TX_, (uint32_t) & _USART_DR_);
    dma_set_memory_address(DMA1, _DMA_CHANNEL_TX_, (uint32_t) uart_buf.data);
    dma_set_number_of_data(DMA1, _DMA_CHANNEL_TX_, UART_BUF_LEN);
    dma_set_read_from_memory(DMA1, _DMA_CHANNEL_TX_);
    dma_enable_memory_increment_mode(DMA1, _DMA_CHANNEL_TX_);
    dma_set_peripheral_size(DMA1, _DMA_CHANNEL_TX_, DMA_CCR_PSIZE_8BIT);
    dma_set_memory_size(DMA1, _DMA_CHANNEL_TX_, DMA_CCR_MSIZE_8BIT);
    dma_set_priority(DMA1, _DMA_CHANNEL_TX_, DMA_CCR_PL_HIGH);

    dma_enable_transfer_complete_interrupt(DMA1, _DMA_CHANNEL_TX_);

    nvic_enable_irq(_NVIC_DMA1_CHANNEL_RX_IRQ_);
    nvic_enable_irq(_NVIC_DMA1_CHANNEL_TX_IRQ_);
}

static void disable_rx_dma(void)
{
    usart_disable_rx_dma(_USART_);
    dma_disable_channel(DMA1, _DMA_CHANNEL_RX_);
}

static void enable_rx_dma(void)
{
    /* clear errors */
    (void)USART_SR(_USART_);
    (void)USART_DR(_USART_);

    dma_set_number_of_data(DMA1, _DMA_CHANNEL_RX_, UART_BUF_LEN);
    dma_enable_channel(DMA1, _DMA_CHANNEL_RX_);
    usart_enable_rx_dma(_USART_);
}

static void disable_tx_dma(void)
{
    usart_disable_tx_dma(_USART_);
    dma_disable_channel(DMA1, _DMA_CHANNEL_TX_);

    if (uart.en.port)
        gpio_clear(uart.en.port, uart.en.pin);
}

static void enable_tx_dma(int size)
{
    if (uart.en.port)
        gpio_set(uart.en.port, uart.en.pin);

    dma_set_number_of_data(DMA1, _DMA_CHANNEL_TX_, size);
    dma_enable_channel(DMA1, _DMA_CHANNEL_TX_);
    usart_enable_tx_dma(_USART_);
}

void _USART_ISR_(void)
{
    portBASE_TYPE y = pdFALSE;

    int idle = 0;

    if ((idle = (usart_get_flag(_USART_, USART_FLAG_IDLE) != 0)) ||
        (usart_get_flag(_USART_, USART_FLAG_TC) != 0)) {

        (void)USART_DR(_USART_);        /* clear int flag */

        if (idle) {
            if (!skip_usart_idle)
                xTaskNotifyFromISR(uart2_notify, IDLE_BIT, eSetBits, &y);
            else
                skip_usart_idle = 0;
        } else {
            xTaskNotifyFromISR(uart2_notify, TC_BIT, eSetBits, &y);
        }
    }

    portYIELD_FROM_ISR(y);
}

static int usart_no_error(void)
{
    if ((usart_get_flag(_USART_, USART_FLAG_PE) != 0) ||
        (usart_get_flag(_USART_, USART_FLAG_FE) != 0)) {

        return 0;

    } else {

        return 1;

    }
}

void uart2_setup(void)
{
    if ((SLAVE_PARITY == USART_PARITY_EVEN)
        || (SLAVE_PARITY == USART_PARITY_ODD)) {
        usart_set_databits(_USART_, 9);
        usart_set_stopbits(_USART_, USART_STOPBITS_1);
    } else {
        usart_set_databits(_USART_, 8);
        usart_set_stopbits(_USART_, USART_STOPBITS_1);
    }
    usart_set_baudrate(_USART_, SLAVE_BAUD_RATE);
    usart_set_mode(_USART_, USART_MODE_TX_RX);
    usart_set_parity(_USART_, SLAVE_PARITY);
    usart_set_flow_control(_USART_, USART_FLOWCONTROL_NONE);

    /* enable IDLE & TC interrupt */
    USART_CR1(_USART_) |= USART_CR1_IDLEIE;

    usart_enable(_USART_);

    nvic_enable_irq(_NVIC_USART_IRQ_);

    dma_setup();
}

static void modbus_uart_init(SemaphoreHandle_t * mutex)
{
    context.buf = rtu_slave_buf;
    context.mutex = mutex;

    mberr = modbusSlaveInit(&rtu_slave,
                            regCallback,
                            NULL,
                            staticAllocator,
                            modbusSlaveDefaultFunctions,
                            modbusSlaveDefaultFunctionCount);

    modbusSlaveSetUserPointer(&rtu_slave, (void *)&context);
}

static int handle_request(void)
{
    mberr = modbusParseRequestRTU(&rtu_slave,
                                  SLAVE_ADDRESS,
                                  (const uint8_t *)uart_buf.data,
                                  uart_buf.length);

    switch (modbusGetGeneralError(mberr)) {
    case MODBUS_OK:
        break;

    case MODBUS_ERROR_ALLOC:

        if (uart_buf.length < 2)
            break;
        mberr = modbusBuildExceptionRTU(&rtu_slave,
                                        SLAVE_ADDRESS,
                                        uart_buf.data[1],
                                        MODBUS_EXCEP_SLAVE_FAILURE);
        break;

    default:
        return 0;
    }

    /* return response if everything is OK */
    if (modbusIsOk(mberr) && modbusSlaveGetResponseLength(&rtu_slave)) {

        int sz = modbusSlaveGetResponseLength(&rtu_slave);
        uint8_t *dat = (uint8_t *) modbusSlaveGetResponse(&rtu_slave);

        uart_buf.length = sz;
        memcpy(uart_buf.data, dat, sz);

        return 1;
    }

    return 0;
}

void uart2_task(void *params)
{
    uint32_t state_bits;

    SemaphoreHandle_t *mutex = (SemaphoreHandle_t *) params;

    modbus_uart_init(mutex);

    enum uart2_state state = UART2_RX;

    for (;;) {
        switch (state) {

        case UART2_RX:
            uart_buf.length = 0;

            enable_rx_dma();
            xTaskNotifyWait(0, 0xff, &state_bits, WAIT_INFINITE);
            disable_rx_dma();

            if (state_bits & (DMA_RX_TC_BIT | IDLE_BIT))
                if ((_DMA_RX_COUNT_ > 0) && usart_no_error()) {
                    uart_buf.length = _DMA_RX_COUNT_;
                    state = UART2_TX;
                }
            break;

        case UART2_TX:
            if (!handle_request())      /* handle modbus request */
                state = UART2_RX;

            enable_tx_dma(uart_buf.length);

            xTaskNotifyWait(0, 0xff, &state_bits, WAIT_INFINITE);

            vTaskDelay(1);

            disable_tx_dma();

            state = UART2_RX;
            break;
        }
    }
}
#endif                          /* STM32F1  && (UART_2 || UART_3) */
