#include <FreeRTOS.h>
#include <task.h>
#include <semphr.h>

#include <libopencm3/cm3/nvic.h>
#include <libopencm3/stm32/dma.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/usart.h>
#ifdef STM32F0
#include <libopencm3/stm32/syscfg.h>
#endif

#include <lightmodbus/lightmodbus.h>

#include <string.h>

#include "config.h"
#include "hw.h"
#include "task_params.h"
#include "modbus.h"

#include "hw.h"
#include "uart_task.h"

#ifdef UART_2

#define _DMA_RXN_ISR_               dma1_channel6_isr
#define _DMA_TXN_ISR_               dma1_channel7_isr
#define _USARTN_ISR_                usart2_isr

#define _USARTN_                    USART2
#define _USARTN_DR_                 USART2_DR

#define _DMA_CHANNEL_RXN_           DMA_CHANNEL6
#define _DMA_CHANNEL_TXN_           DMA_CHANNEL7

#define _NVIC_USARTN_IRQ_           NVIC_USART2_IRQ

#define _NVIC_DMA1_CHANNEL_RXN_IRQ_ NVIC_DMA1_CHANNEL6_IRQ
#define _NVIC_DMA1_CHANNEL_TXN_IRQ_ NVIC_DMA1_CHANNEL7_IRQ

#else                           /* UART_3 */

#define _DMA_RXN_ISR_               dma1_channel3_isr
#define _DMA_TXN_ISR_               dma1_channel2_isr
#define _USARTN_ISR_                usart3_isr

#define _USARTN_                    USART3
#define _USARTN_DR_                 USART3_DR

#define _DMA_CHANNEL_RXN_           DMA_CHANNEL3
#define _DMA_CHANNEL_TXN_           DMA_CHANNEL2

#define _NVIC_USARTN_IRQ_           NVIC_USART3_IRQ

#define _NVIC_DMA1_CHANNEL_RXN_IRQ_ NVIC_DMA1_CHANNEL3_IRQ
#define _NVIC_DMA1_CHANNEL_TXN_IRQ_ NVIC_DMA1_CHANNEL2_IRQ

#endif

#define CIRC_BUF_LEN                16

static const struct uarts uart1 = UART_1;
#define ENABLE_USARTN
#if defined UART_2
static const struct uarts uartn = UART_2;
#elif defined UART_3
static const struct uarts uartn = UART_3;
#else
#undef ENABLE_USARTN
#endif

#ifdef STM32F0
#undef ENABLE_USARTN
#endif

#define UART_TICK_DELAY             10
#define IRQ_TASK_PRIORITY           (configMAX_SYSCALL_INTERRUPT_PRIORITY + 1)

static struct circ_buffer {
    uint8_t data[CIRC_BUF_LEN];
    size_t pos;
}
#ifdef ENABLE_USARTN
    rxn_buf,
#endif
    rx1_buf;

static struct buffer {
    uint8_t data[UART_BUF_LEN];
    size_t len;
}
#ifdef ENABLE_USARTN
    uartn_buf,
#endif
    uart1_buf;

static TaskHandle_t notify;
static TaskHandle_t modbus_slave;

#ifdef STM32F1
void dma1_channel5_isr(void)
#else
void dma1_channel4_7_dma2_channel3_5_isr(void)
#endif
{
    portBASE_TYPE y = pdFALSE;

    if (dma_get_interrupt_flag(DMA1, DMA_CHANNEL5, DMA_HTIF)) {
        dma_clear_interrupt_flags(DMA1, DMA_CHANNEL5, DMA_HTIF);
        xTaskNotifyFromISR(notify, DMA_RX1_HT_BIT, eSetBits, &y);
    }

    if (dma_get_interrupt_flag(DMA1, DMA_CHANNEL5, DMA_TCIF)) {
        dma_clear_interrupt_flags(DMA1, DMA_CHANNEL5, DMA_TCIF);
        xTaskNotifyFromISR(notify, DMA_RX1_TC_BIT, eSetBits, &y);
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
        dma_disable_channel(DMA1, DMA_CHANNEL4);
    }

    portYIELD_FROM_ISR(y);
}

#ifdef ENABLE_USARTN
void _DMA_RXN_ISR_(void)
{
    portBASE_TYPE y = pdFALSE;

    if (dma_get_interrupt_flag(DMA1, _DMA_CHANNEL_RXN_, DMA_HTIF)) {
        dma_clear_interrupt_flags(DMA1, _DMA_CHANNEL_RXN_, DMA_HTIF);
        xTaskNotifyFromISR(notify, DMA_RXN_HT_BIT, eSetBits, &y);
    }

    if (dma_get_interrupt_flag(DMA1, _DMA_CHANNEL_RXN_, DMA_TCIF)) {
        dma_clear_interrupt_flags(DMA1, _DMA_CHANNEL_RXN_, DMA_TCIF);
        xTaskNotifyFromISR(notify, DMA_RXN_TC_BIT, eSetBits, &y);
    }

    portYIELD_FROM_ISR(y);
}

void _DMA_TXN_ISR_(void)
{
    portBASE_TYPE y = pdFALSE;

    if (dma_get_interrupt_flag(DMA1, _DMA_CHANNEL_TXN_, DMA_TCIF)) {
        dma_clear_interrupt_flags(DMA1, _DMA_CHANNEL_TXN_, DMA_TCIF);
        dma_disable_channel(DMA1, _DMA_CHANNEL_TXN_);
    }

    portYIELD_FROM_ISR(y);
}
#endif
static void dma_setup_helper(uint8_t channel, uint32_t per_addr,
                             uint32_t mem_addr)
{
    dma_channel_reset(DMA1, channel);
    dma_set_peripheral_address(DMA1, channel, per_addr);
    dma_set_memory_address(DMA1, channel, mem_addr);
    dma_enable_memory_increment_mode(DMA1, channel);
    dma_set_peripheral_size(DMA1, channel, DMA_CCR_PSIZE_8BIT);
    dma_set_memory_size(DMA1, channel, DMA_CCR_MSIZE_8BIT);
    dma_enable_transfer_complete_interrupt(DMA1, channel);
}

static void dma_setup_rx_helper(uint8_t channel, uint32_t per_addr,
                                uint32_t mem_addr, uint16_t num, uint32_t prio)
{
    dma_setup_helper(channel, per_addr, mem_addr);
    dma_set_number_of_data(DMA1, channel, num);
    dma_set_read_from_peripheral(DMA1, channel);
    dma_enable_circular_mode(DMA1, channel);
    dma_enable_half_transfer_interrupt(DMA1, channel);
    dma_set_priority(DMA1, channel, prio);
    dma_enable_channel(DMA1, channel);
}

static void dma_setup_tx_helper(uint8_t channel, uint32_t per_addr,
                                uint32_t mem_addr, uint16_t num, uint32_t prio)
{
    dma_setup_helper(channel, per_addr, mem_addr);
    dma_set_read_from_memory(DMA1, channel);
    dma_set_number_of_data(DMA1, channel, num);
    dma_set_priority(DMA1, channel, prio);
}

static void dma_setup(void)
{
#ifdef STM32F0
    /* remap RX/TX DMA */
    SYSCFG_CFGR1 |= SYSCFG_CFGR1_USART1_RX_DMA_RMP;
    SYSCFG_CFGR1 |= SYSCFG_CFGR1_USART1_TX_DMA_RMP;
#endif
    /* USART1 RX DMA channel */
    dma_setup_rx_helper(DMA_CHANNEL5,
#ifdef STM32F0
                        (uint32_t) & USART1_RDR,
#else
                        (uint32_t) & USART1_DR,
#endif
                        (uint32_t) rx1_buf.data, CIRC_BUF_LEN, DMA_CCR_PL_HIGH);

    /* USART1 TX DMA channel */
    dma_setup_tx_helper(DMA_CHANNEL4,
#ifdef STM32F0
                        (uint32_t) & USART1_TDR,
#else
                        (uint32_t) & USART1_DR,
#endif
                        (uint32_t) uart1_buf.data, UART_BUF_LEN,
                        DMA_CCR_PL_LOW);

#ifdef STM32F0
    nvic_set_priority(NVIC_DMA1_CHANNEL4_7_DMA2_CHANNEL3_5_IRQ,
                      IRQ_TASK_PRIORITY);
    nvic_enable_irq(NVIC_DMA1_CHANNEL4_7_DMA2_CHANNEL3_5_IRQ);
#else
    nvic_set_priority(NVIC_DMA1_CHANNEL5_IRQ, IRQ_TASK_PRIORITY);
    nvic_enable_irq(NVIC_DMA1_CHANNEL5_IRQ);
    nvic_set_priority(NVIC_DMA1_CHANNEL4_IRQ, IRQ_TASK_PRIORITY);
    nvic_enable_irq(NVIC_DMA1_CHANNEL4_IRQ);
#endif

#ifdef ENABLE_USARTN
    /* USARTN RX DMA channel */
    dma_setup_rx_helper(_DMA_CHANNEL_RXN_, (uint32_t) & _USARTN_DR_,
                        (uint32_t) rxn_buf.data, CIRC_BUF_LEN, DMA_CCR_PL_HIGH);
    nvic_set_priority(_NVIC_DMA1_CHANNEL_RXN_IRQ_, IRQ_TASK_PRIORITY);
    nvic_enable_irq(_NVIC_DMA1_CHANNEL_RXN_IRQ_);

    /* USARTN TX DMA channel */
    dma_setup_tx_helper(_DMA_CHANNEL_TXN_, (uint32_t) & _USARTN_DR_,
                        (uint32_t) uartn_buf.data, UART_BUF_LEN,
                        DMA_CCR_PL_LOW);
    nvic_set_priority(_NVIC_DMA1_CHANNEL_TXN_IRQ_, IRQ_TASK_PRIORITY);
    nvic_enable_irq(_NVIC_DMA1_CHANNEL_TXN_IRQ_);
#endif
}

static void enable_tx1_dma(int size)
{
    dma_set_number_of_data(DMA1, DMA_CHANNEL4, size);
    dma_enable_channel(DMA1, DMA_CHANNEL4);
}

#ifdef ENABLE_USARTN
static void enable_txn_dma(int size)
{
    dma_set_number_of_data(DMA1, _DMA_CHANNEL_TXN_, size);
    dma_enable_channel(DMA1, _DMA_CHANNEL_TXN_);
}
#endif
void usart1_isr(void)
{
    portBASE_TYPE y = pdFALSE;

    if (usart_get_flag(USART1, USART_FLAG_IDLE) != 0) {
#ifdef STM32F1
        (void)USART_DR(USART1);                 /* clear IDLE flag */
#else
        USART_ICR(USART1) |= USART_ICR_IDLECF;
#endif
        xTaskNotifyFromISR(notify, USART1_IDLE_BIT, eSetBits, &y);
    }

    if (usart_get_flag(USART1, USART_FLAG_TC) != 0) {
#ifdef STM32F1
        USART_SR(USART1) &= ~USART_FLAG_TC;     /* clear TC flag */
#else
        USART_ICR(USART1) &= USART_ICR_TCCF;
#endif
        xTaskNotifyFromISR(notify, USART1_TC_BIT, eSetBits, &y);
    }

    portYIELD_FROM_ISR(y);
}

#ifdef ENABLE_USARTN
void _USARTN_ISR_(void)
{
    portBASE_TYPE y = pdFALSE;

    if (usart_get_flag(_USARTN_, USART_FLAG_IDLE) != 0) {
        (void)USART_DR(_USARTN_);               /* clear IDLE flag */
        xTaskNotifyFromISR(notify, USARTN_IDLE_BIT, eSetBits, &y);
    }

    if (usart_get_flag(_USARTN_, USART_FLAG_TC) != 0) {
        USART_SR(_USARTN_) &= ~USART_FLAG_TC;   /* clear TC flag */
        xTaskNotifyFromISR(notify, USARTN_TC_BIT, eSetBits, &y);
    }

    portYIELD_FROM_ISR(y);
}
#endif

static void uart_setup_helper(uint32_t usart, uint8_t irqn)
{
    if ((SLAVE_PARITY == USART_PARITY_EVEN)
        || (SLAVE_PARITY == USART_PARITY_ODD)) {
        usart_set_databits(usart, 9);
        usart_set_stopbits(usart, USART_STOPBITS_1);
    } else {
        usart_set_databits(usart, 8);
        usart_set_stopbits(usart, USART_STOPBITS_1);
    }
    usart_set_baudrate(usart, SLAVE_BAUD_RATE);
    usart_set_mode(usart, USART_MODE_TX_RX);
    usart_set_parity(usart, SLAVE_PARITY);
    usart_set_flow_control(usart, USART_FLOWCONTROL_NONE);
#ifdef STM32F0
    /* disable rx overrun */
    USART_CR3(USART1) |= USART_CR3_OVRDIS;
#endif
    /* enable IDLE and TX Complete interrupt */
    USART_CR1(usart) |= USART_CR1_IDLEIE | USART_CR1_TCIE;

    usart_enable(usart);

    nvic_set_priority(irqn, IRQ_TASK_PRIORITY);
    nvic_enable_irq(irqn);
#ifdef STM32F1
    /* clear errors */
    (void)USART_SR(usart);
    (void)USART_DR(usart);
#else
    /* clear errors */
    USART_ICR(usart) |= (USART_ICR_PECF | USART_ICR_FECF);
    (void)USART_RDR(usart);
#endif
    usart_enable_rx_dma(usart);
    usart_enable_tx_dma(usart);
}

static void uart_setup(void)
{
    uart_setup_helper(USART1, NVIC_USART1_IRQ);
#ifdef ENABLE_USARTN
    uart_setup_helper(_USARTN_, _NVIC_USARTN_IRQ_);
#endif
    dma_setup();
}

static void update_buf(struct buffer *dbuf, uint8_t * src, size_t size)
{
    if ((dbuf->len + size) > UART_BUF_LEN)
        size = UART_BUF_LEN - dbuf->len;

    if (size) {
        memcpy(&dbuf->data[dbuf->len], src, size);
        dbuf->len += size;
    }
}

static void rx_check(struct buffer *dbuf, struct circ_buffer *cbuf,
                     uint8_t channel)
{
    size_t pos, sz;

    pos = CIRC_BUF_LEN - DMA_CNDTR(DMA1, channel);
    if (pos != cbuf->pos) {
        if (pos > cbuf->pos) {
            sz = pos - cbuf->pos;
            update_buf(dbuf, &cbuf->data[cbuf->pos], sz);
        } else {
            sz = CIRC_BUF_LEN - cbuf->pos;
            update_buf(dbuf, &cbuf->data[cbuf->pos], sz);
            if (pos > 0) {
                update_buf(dbuf, &cbuf->data[0], pos);
            }
        }
        cbuf->pos = pos;
    }
}

enum uart_state {
    STATE_RX,
    STATE_RX_DONE,
    STATE_WAIT_TX,
    STATE_WAIT_TX_DONE,
};

static enum uart_state update_modbus_slave_state(int res,
                                                 enum uart_state state,
                                                 uint32_t state_bits, int id)
{
    enum uart_state next_state = state;
    size_t magic_len = sizeof(BOOTLOADER_MAGIC) - 1;

    switch (state) {
    case STATE_RX:
        if (((id ? USARTN_IDLE_BIT : USART1_IDLE_BIT) & state_bits)
            && (id ? uartn_buf.len : uart1_buf.len)) {

            next_state = STATE_RX_DONE;
        }
        break;

    case STATE_RX_DONE:
        if ((id ? RXN_UPDATE_BITS : RX1_UPDATE_BITS) & state_bits) {
            next_state = STATE_RX;
            break;
        }
        if (!res) {
            if (!id) {
                if (uart1_buf.len == magic_len)
                    if (memcmp(uart1_buf.data, BOOTLOADER_MAGIC,
                               magic_len) == 0) {
                        run_bootloader();
                    }
                usart_set_mode(USART1, USART_MODE_TX);
#ifdef ENABLE_USARTN
            } else {
                usart_set_mode(_USARTN_, USART_MODE_TX);
#endif
            }

            xTaskNotify(modbus_slave,
                        (uint32_t) 1 << (id ? USARTN_RTU : USART1_RTU),
                        eSetBits);
            next_state = STATE_WAIT_TX;
        }
        break;

    case STATE_WAIT_TX:
        if ((id ? USARTN_SRC : USART1_SRC) & state_bits) {
            if (((id ? USARTN_SRC : USART1_SRC) << 1) & state_bits) {
                if (!id) {
                    if (uart1.en.port)
                        gpio_set(uart1.en.port, uart1.en.pin);
                    enable_tx1_dma(uart1_buf.len);
#ifdef ENABLE_USARTN
                } else {
                    if (uartn.en.port)
                        gpio_set(uartn.en.port, uartn.en.pin);
                    enable_txn_dma(uartn_buf.len);
#endif
                }
                next_state = STATE_WAIT_TX_DONE;
            } else {
                if (!id)
                    usart_set_mode(USART1, USART_MODE_TX_RX);
#ifdef ENABLE_USARTN
                else
                    usart_set_mode(_USARTN_, USART_MODE_TX_RX);
#endif
                next_state = STATE_RX;
            }
            if (!id)
                uart1_buf.len = 0;
#ifdef ENABLE_USARTN
            else
                uartn_buf.len = 0;
#endif
        }
        break;

    case STATE_WAIT_TX_DONE:
        if ((id ? USARTN_TC_BIT : USART1_TC_BIT) & state_bits) {
            if (!id) {
                usart_set_mode(USART1, USART_MODE_TX_RX);
                if (uart1.en.port)
                    gpio_clear(uart1.en.port, uart1.en.pin);
#ifdef ENABLE_USARTN
            } else {
                usart_set_mode(_USARTN_, USART_MODE_TX_RX);
                if (uartn.en.port)
                    gpio_clear(uartn.en.port, uartn.en.pin);
#endif
            }
            next_state = STATE_RX;
        }
        break;
    }

    return next_state;
}

void uart_task(void *params)
{
    uint32_t state_bits;
    struct modbus_slave_msg *uart1_msg;
#ifdef ENABLE_USARTN
    struct modbus_slave_msg *uartn_msg;
#endif

    notify = ((struct task_parameters *)params)->uart;
    modbus_slave = ((struct task_parameters *)params)->modbus_slave;

    uart_setup();

    uart1_msg = &(((struct task_parameters *)params)->msgs[USART1_RTU]);
    *uart1_msg = (struct modbus_slave_msg) {
        .data = uart1_buf.data,
        .length = &uart1_buf.len,
        .src = notify,
        .src_bits = USART1_SRC,
        .rtu_flag = 1,
    };
    enum uart_state usart1_state = STATE_RX;

#ifdef ENABLE_USARTN
    uartn_msg = &(((struct task_parameters *)params)->msgs[USARTN_RTU]);
    *uartn_msg = (struct modbus_slave_msg) {
        .data = uartn_buf.data,
        .length = &uartn_buf.len,
        .src = notify,
        .src_bits = USARTN_SRC,
        .rtu_flag = 1,
    };
    enum uart_state usartn_state = STATE_RX;
#endif

    for (;;) {

        int res = xTaskNotifyWait(0, 0xffffffff, &state_bits, UART_TICK_DELAY);

        if (state_bits & RX1_UPDATE_BITS)
            rx_check(&uart1_buf, &rx1_buf, DMA_CHANNEL5);
        usart1_state = update_modbus_slave_state(res, usart1_state, state_bits,
                                                 0);
#ifdef ENABLE_USARTN
        if (state_bits & RXN_UPDATE_BITS)
            rx_check(&uartn_buf, &rxn_buf, _DMA_CHANNEL_RXN_);
        usartn_state = update_modbus_slave_state(res, usartn_state, state_bits,
                                                 1);
#endif
    }
}
