#include <FreeRTOS.h>
#include <task.h>
#include <semphr.h>

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
#include "task_params.h"

#define IDLE_BIT                    0x01
#define DMA_RX_TC_BIT               0x02
#define DMA_TX_TC_BIT               0x04

static struct {
    uint8_t data[UART_BUF_LEN];
    uint16_t length;
} uart_buf;

static TaskHandle_t notify;

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

        xTaskNotifyFromISR(notify, DMA_RX_TC_BIT, eSetBits, &y);
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

        xTaskNotifyFromISR(notify, DMA_TX_TC_BIT, eSetBits, &y);
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

#ifdef STM32F0
    dma_set_peripheral_address(DMA1, DMA_CHANNEL5, (uint32_t) & USART1_RDR);
#else
    dma_set_peripheral_address(DMA1, DMA_CHANNEL5, (uint32_t) & USART1_DR);
#endif
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

#ifdef STM32F0
    dma_set_peripheral_address(DMA1, DMA_CHANNEL4, (uint32_t) & USART1_TDR);
#else
    dma_set_peripheral_address(DMA1, DMA_CHANNEL4, (uint32_t) & USART1_DR);
#endif
    dma_set_memory_address(DMA1, DMA_CHANNEL4, (uint32_t) uart_buf.data);
    dma_set_number_of_data(DMA1, DMA_CHANNEL4, UART_BUF_LEN);
    dma_set_read_from_memory(DMA1, DMA_CHANNEL4);
    dma_enable_memory_increment_mode(DMA1, DMA_CHANNEL4);
    dma_set_peripheral_size(DMA1, DMA_CHANNEL4, DMA_CCR_PSIZE_8BIT);
    dma_set_memory_size(DMA1, DMA_CHANNEL4, DMA_CCR_MSIZE_8BIT);
    dma_set_priority(DMA1, DMA_CHANNEL4, DMA_CCR_PL_HIGH);

    dma_enable_transfer_complete_interrupt(DMA1, DMA_CHANNEL4);

#ifdef STM32F0
    nvic_enable_irq(NVIC_DMA1_CHANNEL4_7_DMA2_CHANNEL3_5_IRQ);
#else
    nvic_enable_irq(NVIC_DMA1_CHANNEL4_IRQ);
    nvic_enable_irq(NVIC_DMA1_CHANNEL5_IRQ);
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
            xTaskNotifyFromISR(notify, IDLE_BIT, eSetBits, &y);
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

static int handle_request(TaskHandle_t mbs, struct modbus_slave_msg *msg)
{
    uint32_t result;

    while (!xTaskNotifyIndexed
           (mbs, 1, (uint32_t) msg, eSetValueWithoutOverwrite))
        vTaskDelay(10);

    xTaskNotifyWaitIndexed(1, 0x00, 0xffffffff, &result, portMAX_DELAY);

    return result;
}

void uart1_task(void *params)
{
    uint32_t state_bits;
    int magic_len = sizeof(BOOTLOADER_MAGIC) - 1;

    notify = ((struct task_parameters *)params)->uart1;

    TaskHandle_t modbus_slave =
        ((struct task_parameters *)params)->modbus_slave;

    struct modbus_slave_msg msg = {
        uart_buf.data,
        &uart_buf.length,
        notify,
    };

    enum uart1_state state = UART1_RX;

    for (;;) {
        switch (state) {

        case UART1_RX:
            uart_buf.length = 0;

            enable_rx_dma();
            xTaskNotifyWait(0, 0xff, &state_bits, WAIT_INFINITE);
            disable_rx_dma();

            if (state_bits & (DMA_RX_TC_BIT | IDLE_BIT))
                if ((DMA_RX_COUNT > 0) && usart_no_error()) {
                    uart_buf.length = DMA_RX_COUNT;
                    state = UART1_TX;
                }
            break;

        case UART1_TX:
            /* check for bootloader magic string */
            if (uart_buf.length == magic_len)
                if (memcmp(uart_buf.data, BOOTLOADER_MAGIC, magic_len) == 0)
                    run_bootloader();

            if (!handle_request(modbus_slave, &msg))    /* handle modbus request */
                state = UART1_RX;

            enable_tx_dma(uart_buf.length);
            xTaskNotifyWait(0, 0xff, &state_bits, WAIT_INFINITE);
            disable_tx_dma();

            state = UART1_RX;
            break;
        }
    }
}
