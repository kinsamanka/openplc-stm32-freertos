#include "modbus_slave.h"
#include <task.h>
#include <queue.h>

#include <libopencm3/cm3/nvic.h>
#include <libopencm3/stm32/dma.h>
#include <libopencm3/stm32/gpio.h>
#ifdef STM32F0
#include <libopencm3/stm32/syscfg.h>
#endif
#include <libopencm3/stm32/usart.h>

#include <lightmodbus/lightmodbus.h>

#include <string.h>

#include "hw.h"
#include "uart1_task.h"

#define IDLE_BIT                    0x01
#define DMA_RX_TC_BIT               0x02
#define DMA_TX_TC_BIT               0x04

static struct {
    char data[UART_BUF_LEN];
    uint8_t length;
} uart_buf;

static ModbusErrorInfo mberr;
static ModbusSlave rtu_slave;

static uint8_t rtu_slave_buf[MAX_RESPONSE];
static struct context context;

extern TaskHandle_t uart_notify;

static volatile int skip_usart_idle = 0;

#ifdef STM32F1
void dma1_channel5_isr(void)
#else
void dma1_channel4_7_dma2_channel3_5_isr(void)
#endif
{
    portBASE_TYPE y = pdFALSE;

    if (dma_get_interrupt_flag(DMA1, DMA_CHANNEL5, DMA_TCIF)) {

        dma_clear_interrupt_flags(DMA1, DMA_CHANNEL5, DMA_TCIF);

        /* ignore the IDLE interrupt after DMA is done */
        skip_usart_idle = 1;

        xTaskNotifyFromISR(uart_notify, DMA_RX_TC_BIT, eSetBits, &y);
    }
#ifdef STM32F1
    portYIELD_FROM_ISR(y);
}

void dma1_channel4_isr(void)
{
    portBASE_TYPE y = pdFALSE;
#endif
    if (dma_get_interrupt_flag(DMA1, DMA_CHANNEL4, DMA_TCIF)) {

        dma_clear_interrupt_flags(DMA1, DMA_CHANNEL4, DMA_TCIF);

        xTaskNotifyFromISR(uart_notify, DMA_TX_TC_BIT, eSetBits, &y);
    }

    portYIELD_FROM_ISR(y);
}

static void dma_setup(void)
{
#ifdef STM32F0
    /* remap RX/TX DMA */
    SYSCFG_CFGR1 |= SYSCFG_CFGR1_USART1_RX_DMA_RMP;
    SYSCFG_CFGR1 |= SYSCFG_CFGR1_USART1_TX_DMA_RMP;
#endif
    /* USART RX DMA channel */
    dma_channel_reset(DMA1, DMA_CHANNEL5);

    dma_set_peripheral_address(DMA1, DMA_CHANNEL5, (uint32_t) & RX1_PERIF_ADDR);
    dma_set_memory_address(DMA1, DMA_CHANNEL5, (uint32_t) uart_buf.data);
    dma_set_number_of_data(DMA1, DMA_CHANNEL5, UART_BUF_LEN);
    dma_set_read_from_peripheral(DMA1, DMA_CHANNEL5);
    dma_enable_memory_increment_mode(DMA1, DMA_CHANNEL5);
    dma_set_peripheral_size(DMA1, DMA_CHANNEL5, DMA_CCR_PSIZE_8BIT);
    dma_set_memory_size(DMA1, DMA_CHANNEL5, DMA_CCR_MSIZE_8BIT);
    dma_set_priority(DMA1, DMA_CHANNEL5, DMA_CCR_PL_HIGH);

    dma_enable_transfer_complete_interrupt(DMA1, DMA_CHANNEL5);

    /* USART TX DMA channel */
    dma_channel_reset(DMA1, DMA_CHANNEL4);

    dma_set_peripheral_address(DMA1, DMA_CHANNEL4, (uint32_t) & TX1_PERIF_ADDR);
    dma_set_memory_address(DMA1, DMA_CHANNEL4, (uint32_t) uart_buf.data);
    dma_set_number_of_data(DMA1, DMA_CHANNEL4, UART_BUF_LEN);
    dma_set_read_from_memory(DMA1, DMA_CHANNEL4);
    dma_enable_memory_increment_mode(DMA1, DMA_CHANNEL4);
    dma_set_peripheral_size(DMA1, DMA_CHANNEL4, DMA_CCR_PSIZE_8BIT);
    dma_set_memory_size(DMA1, DMA_CHANNEL4, DMA_CCR_MSIZE_8BIT);
    dma_set_priority(DMA1, DMA_CHANNEL4, DMA_CCR_PL_HIGH);

    dma_enable_transfer_complete_interrupt(DMA1, DMA_CHANNEL4);

#ifdef STM32F1
    nvic_enable_irq(NVIC_DMA1_CHANNEL4_IRQ);
    nvic_enable_irq(NVIC_DMA1_CHANNEL5_IRQ);
#else
    nvic_enable_irq(NVIC_DMA1_CHANNEL4_7_DMA2_CHANNEL3_5_IRQ);
#endif
}

static void disable_rx_dma(void)
{
    usart_disable_rx_dma(USART1);
    dma_disable_channel(DMA1, DMA_CHANNEL5);
}

static void enable_rx_dma(void)
{
#ifdef STM32F1
    /* clear errors */
    (void)USART_SR(USART1);
    (void)USART_DR(USART1);
#else
    /* clear errors */
    USART_ICR(USART1) |= (USART_ICR_PECF | USART_ICR_FECF);

    /* flush rx input */
    (void)USART_RDR(USART1);
#endif
    dma_set_number_of_data(DMA1, DMA_CHANNEL5, UART_BUF_LEN);
    dma_enable_channel(DMA1, DMA_CHANNEL5);
    usart_enable_rx_dma(USART1);
}

static void disable_tx_dma(void)
{
    usart_disable_tx_dma(USART1);
    dma_disable_channel(DMA1, DMA_CHANNEL4);
}

static void enable_tx_dma(int size)
{
    dma_set_number_of_data(DMA1, DMA_CHANNEL4, size);
    dma_enable_channel(DMA1, DMA_CHANNEL4);
    usart_enable_tx_dma(USART1);
}

void usart1_isr(void)
{
    portBASE_TYPE y = pdFALSE;

    if (usart_get_flag(USART1, USART_FLAG_IDLE) != 0) {

#ifdef STM32F1
        (void)USART_DR(USART1); /* clear idle flag */
#else
        USART_ICR(USART1) |= USART_ICR_IDLECF;  /* clear interrupt */
#endif
        if (!skip_usart_idle)
            xTaskNotifyFromISR(uart_notify, IDLE_BIT, eSetBits, &y);
        else
            skip_usart_idle = 0;
    }

    portYIELD_FROM_ISR(y);
}

static int usart_no_error(void)
{
    if ((usart_get_flag(USART1, USART_FLAG_PE) != 0) ||
        (usart_get_flag(USART1, USART_FLAG_FE) != 0)) {

        return 0;

    } else {

        return 1;

    }
}

void uart1_setup(void)
{
#ifdef STM32F0
    skip_usart_idle = 1;
#endif
    if ((SLAVE_PARITY == USART_PARITY_EVEN)
        || (SLAVE_PARITY == USART_PARITY_ODD)) {
        usart_set_databits(USART1, 9);
        usart_set_stopbits(USART1, USART_STOPBITS_1);
    } else {
        usart_set_databits(USART1, 8);
        usart_set_stopbits(USART1, USART_STOPBITS_1);
    }
    usart_set_baudrate(USART1, SLAVE_BAUD_RATE);
    usart_set_mode(USART1, USART_MODE_TX_RX);
    usart_set_parity(USART1, SLAVE_PARITY);
    usart_set_flow_control(USART1, USART_FLOWCONTROL_NONE);
#ifdef STM32F0
    /* disable rx overrun */
    USART_CR3(USART1) |= USART_CR3_OVRDIS;
#endif
    /* enable IDLE interrupt */
    USART_CR1(USART1) |= USART_CR1_IDLEIE;

    usart_enable(USART1);

    nvic_enable_irq(NVIC_USART1_IRQ);

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

    // return response if everything is OK
    if (modbusIsOk(mberr) && modbusSlaveGetResponseLength(&rtu_slave)) {

        int sz = modbusSlaveGetResponseLength(&rtu_slave);
        uint8_t *dat = (uint8_t *) modbusSlaveGetResponse(&rtu_slave);

        uart_buf.length = sz;
        memcpy(uart_buf.data, dat, sz);

        return 1;
    }

    return 0;
}

void uart1_task(void *params)
{
    uint32_t state_bits;
    int magic_len = sizeof(BOOTLOADER_MAGIC) - 1;

    SemaphoreHandle_t *mutex = (SemaphoreHandle_t *) params;

    modbus_uart_init(mutex);

    enum task_state state = STATE_RX;

    for (;;) {
        switch (state) {

        case STATE_RX:
            uart_buf.length = 0;

            enable_rx_dma();
            xTaskNotifyWait(0, 0xff, &state_bits, WAIT_INFINITE);
            disable_rx_dma();

            if (state_bits & (DMA_RX_TC_BIT | IDLE_BIT))
                if (usart_no_error()) {
                    uart_buf.length = DMA_RX_COUNT;
                    state = STATE_TX;
                }
            break;

        case STATE_TX:
            /* check for bootloader magic string */
            if (uart_buf.length == magic_len)
                if (memcmp(uart_buf.data, BOOTLOADER_MAGIC, magic_len) == 0)
                    run_bootloader();

            if (!handle_request())      /* handle modbus request */
                state = STATE_RX;

            enable_tx_dma(uart_buf.length);
            xTaskNotifyWait(0, 0xff, &state_bits, WAIT_INFINITE);
            disable_tx_dma();

            state = STATE_RX;
            break;
        }
    }
}
