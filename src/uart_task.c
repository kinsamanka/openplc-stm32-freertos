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

const uint32_t usart[] = { USART1, USART2, USART3 };
const uint8_t dma_rx_chan[] = { DMA_CHANNEL5, DMA_CHANNEL6, DMA_CHANNEL3 };
const uint8_t dma_tx_chan[] = { DMA_CHANNEL4, DMA_CHANNEL7, DMA_CHANNEL2 };

#ifdef STM32F1
const uint8_t dma_rx_irq[] =
    { NVIC_DMA1_CHANNEL5_IRQ, NVIC_DMA1_CHANNEL6_IRQ, NVIC_DMA1_CHANNEL3_IRQ };
const uint8_t dma_tx_irq[] =
    { NVIC_DMA1_CHANNEL4_IRQ, NVIC_DMA1_CHANNEL7_IRQ, NVIC_DMA1_CHANNEL2_IRQ };
const uint8_t usart_irq[] =
    { NVIC_USART1_IRQ, NVIC_USART2_IRQ, NVIC_USART3_IRQ };
#else
const uint8_t dma_rx_irq[] = { NVIC_DMA1_CHANNEL4_7_DMA2_CHANNEL3_5_IRQ };
const uint8_t usart_irq[] = { NVIC_USART1_IRQ };
#endif

#ifdef STM32F1
#define USART_RX(n)                 USART_DR(n)
#define USART_TX(n)                 USART_DR(n)
#else
#define USART_RX(n)                 USART_RDR(n)
#define USART_TX(n)                 USART_TDR(n)
#endif

#define CIRC_BUF_LEN                16

extern const struct uarts uart[];

#if defined STM32F1 && defined UART_2

#define ENABLE_USART2
#define UART_NUM                    2

#else

#define UART_NUM                    1

#endif

#define UART_TICK_DELAY             10
#define IRQ_TASK_PRIORITY           (configMAX_SYSCALL_INTERRUPT_PRIORITY + 1)

static struct circ_buffer {
    uint8_t data[CIRC_BUF_LEN];
    size_t pos;
} rx_buf[UART_NUM];

static struct buffer {
    uint8_t data[UART_BUF_LEN];
    size_t len;
} uart_buf[UART_NUM];

static TaskHandle_t notify;
static TaskHandle_t modbus_slave;

static inline portBASE_TYPE dma_rx_isr(int j)
{
    portBASE_TYPE y = pdFALSE;

    if (dma_get_interrupt_flag(DMA1, dma_rx_chan[j], DMA_HTIF)) {
        dma_clear_interrupt_flags(DMA1, dma_rx_chan[j], DMA_HTIF);
        xTaskNotifyFromISR(notify, j ? DMA_RX2_HT_BIT : DMA_RX1_HT_BIT,
                           eSetBits, &y);
    }

    if (dma_get_interrupt_flag(DMA1, dma_rx_chan[j], DMA_TCIF)) {
        dma_clear_interrupt_flags(DMA1, dma_rx_chan[j], DMA_TCIF);
        xTaskNotifyFromISR(notify, j ? DMA_RX2_TC_BIT : DMA_RX1_TC_BIT,
                           eSetBits, &y);
    }

    return y;
}

static inline portBASE_TYPE dma_tx_isr(int j)
{
    portBASE_TYPE y = pdFALSE;

    if (dma_get_interrupt_flag(DMA1, dma_tx_chan[j], DMA_TCIF)) {
        dma_clear_interrupt_flags(DMA1, dma_tx_chan[j], DMA_TCIF);
        dma_disable_channel(DMA1, dma_tx_chan[j]);
    }

    return y;
}

#ifdef STM32F0
void dma1_channel4_7_dma2_channel3_5_isr(void)
{
    portBASE_TYPE y = pdFALSE;

    y |= dma_rx_isr(0);
    y |= dma_tx_isr(0);

    portYIELD_FROM_ISR(y);
}
#else
void dma1_channel5_isr(void)
{
    portYIELD_FROM_ISR(dma_rx_isr(0));
}

void dma1_channel4_isr(void)
{
    portYIELD_FROM_ISR(dma_tx_isr(0));
}

void dma1_channel6_isr(void)
{
    portYIELD_FROM_ISR(dma_rx_isr(1));
}

void dma1_channel7_isr(void)
{
    portYIELD_FROM_ISR(dma_tx_isr(1));
}

void dma1_channel3_isr(void)
{
    portYIELD_FROM_ISR(dma_rx_isr(2));
}

void dma1_channel2_isr(void)
{
    portYIELD_FROM_ISR(dma_tx_isr(2));
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

static void dma_setup_rx_helper(int n, uint16_t num, uint32_t prio)
{
    int j = uart[n].index;
    uint8_t channel = dma_rx_chan[j];
    uint32_t per_addr = (uint32_t) & USART_RX(usart[j]);
    uint32_t mem_addr = (uint32_t) rx_buf[n].data;

    dma_setup_helper(channel, per_addr, mem_addr);
    dma_set_number_of_data(DMA1, channel, num);
    dma_set_read_from_peripheral(DMA1, channel);
    dma_enable_circular_mode(DMA1, channel);
    dma_enable_half_transfer_interrupt(DMA1, channel);
    dma_set_priority(DMA1, channel, prio);
    dma_enable_channel(DMA1, channel);
}

static void dma_setup_tx_helper(int n, uint16_t num, uint32_t prio)
{
    int j = uart[n].index;
    uint8_t channel = dma_tx_chan[j];
    uint32_t per_addr = (uint32_t) & USART_TX(usart[j]);
    uint32_t mem_addr = (uint32_t) uart_buf[n].data;

    dma_setup_helper(channel, per_addr, mem_addr);
    dma_set_read_from_memory(DMA1, channel);
    dma_set_number_of_data(DMA1, channel, num);
    dma_set_priority(DMA1, channel, prio);
}

static void dma_setup(int n)
{
    int j = uart[n].index;
#ifdef STM32F0
    /* remap RX/TX DMA */
    SYSCFG_CFGR1 |= SYSCFG_CFGR1_USART1_RX_DMA_RMP;
    SYSCFG_CFGR1 |= SYSCFG_CFGR1_USART1_TX_DMA_RMP;
#endif
    /* USART1 RX DMA channel */
    dma_setup_rx_helper(n, CIRC_BUF_LEN, DMA_CCR_PL_HIGH);

    /* USART1 TX DMA channel */
    dma_setup_tx_helper(n, UART_BUF_LEN, DMA_CCR_PL_LOW);

#ifdef STM32F0
    nvic_set_priority(dma_rx_irq[j], IRQ_TASK_PRIORITY);
    nvic_enable_irq(dma_rx_irq[j]);
#else
    nvic_set_priority(dma_rx_irq[j], IRQ_TASK_PRIORITY);
    nvic_enable_irq(dma_rx_irq[j]);
    nvic_set_priority(dma_tx_irq[j], IRQ_TASK_PRIORITY);
    nvic_enable_irq(dma_tx_irq[j]);
#endif
}

static void enable_tx_dma(int n, int size)
{
    int j = uart[n].index;
    dma_set_number_of_data(DMA1, dma_tx_chan[j], size);
    dma_enable_channel(DMA1, dma_tx_chan[j]);
}

static inline portBASE_TYPE usart_isr(int j)
{
    portBASE_TYPE y = pdFALSE;

    if (usart_get_flag(usart[j], USART_FLAG_IDLE) != 0) {
#ifdef STM32F1
        (void)USART_DR(usart[j]);       /* clear IDLE flag */
#else
        USART_ICR(usart[j]) |= USART_ICR_IDLECF;
#endif
        xTaskNotifyFromISR(notify, j ? USART2_IDLE_BIT : USART1_IDLE_BIT,
                           eSetBits, &y);
    }

    if (usart_get_flag(usart[j], USART_FLAG_TC) != 0) {
#ifdef STM32F1
        USART_SR(usart[j]) &= ~USART_FLAG_TC;   /* clear TC flag */
#else
        USART_ICR(usart[j]) &= USART_ICR_TCCF;
#endif
        xTaskNotifyFromISR(notify, j ? USART2_TC_BIT : USART1_TC_BIT, eSetBits,
                           &y);
    }

    return y;
}

void usart1_isr(void)
{
    portYIELD_FROM_ISR(usart_isr(0));
}

#ifdef STM32F1
void usart2_isr(void)
{
    portYIELD_FROM_ISR(usart_isr(1));
}

void usart3_isr(void)
{
    portYIELD_FROM_ISR(usart_isr(2));
}
#endif

static void uart_setup_helper(uint32_t usartn, uint8_t irqn)
{
    if ((SLAVE_PARITY == USART_PARITY_EVEN)
        || (SLAVE_PARITY == USART_PARITY_ODD)) {
        usart_set_databits(usartn, 9);
        usart_set_stopbits(usartn, USART_STOPBITS_1);
    } else {
        usart_set_databits(usartn, 8);
        usart_set_stopbits(usartn, USART_STOPBITS_1);
    }
    usart_set_baudrate(usartn, SLAVE_BAUD_RATE);
    usart_set_mode(usartn, USART_MODE_TX_RX);
    usart_set_parity(usartn, SLAVE_PARITY);
    usart_set_flow_control(usartn, USART_FLOWCONTROL_NONE);
#ifdef STM32F0
    /* disable rx overrun */
    USART_CR3(USART1) |= USART_CR3_OVRDIS;
#endif
    /* enable IDLE and TX Complete interrupt */
    USART_CR1(usartn) |= USART_CR1_IDLEIE | USART_CR1_TCIE;

    usart_enable(usartn);

    nvic_set_priority(irqn, IRQ_TASK_PRIORITY);
    nvic_enable_irq(irqn);
#ifdef STM32F1
    /* clear errors */
    (void)USART_SR(usartn);
    (void)USART_DR(usartn);
#else
    /* clear errors */
    USART_ICR(usartn) |= (USART_ICR_PECF | USART_ICR_FECF);
    (void)USART_RDR(usartn);
#endif
    usart_enable_rx_dma(usartn);
    usart_enable_tx_dma(usartn);
}

static void uart_setup(void)
{
    for (int i = 0; i < UART_NUM; i++) {
        int j = uart[i].index;
        uart_setup_helper(usart[j], usart_irq[j]);
        dma_setup(i);
    }

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

static void rx_check(int n)
{
    size_t pos, sz;
    struct buffer *dbuf = &uart_buf[n];
    struct circ_buffer *cbuf = &rx_buf[n];
    int j = uart[n].index;
    uint8_t channel = dma_rx_chan[j];

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

static void enable_tx(int n, int en)
{
    if (uart[n].en.port) {
        if (en)
            gpio_set(uart[n].en.port, uart[n].en.pin);
        else
            gpio_clear(uart[n].en.port, uart[n].en.pin);
    }
}

enum uart_state {
    STATE_RX,
    STATE_RX_DONE,
    STATE_WAIT_TX,
    STATE_WAIT_TX_DONE,
};

static enum uart_state update_modbus_slave_state(int res, enum uart_state state,
                                                 uint32_t state_bits, int n)
{
    int j = uart[n].index;
    struct buffer *buf = &uart_buf[n];
    enum uart_state next_state = state;
    size_t magic_len = sizeof(BOOTLOADER_MAGIC) - 1;

    switch (state) {
    case STATE_RX:
        if (((n ? USART2_IDLE_BIT : USART1_IDLE_BIT) & state_bits)
            && buf->len) {

            next_state = STATE_RX_DONE;
        }
        break;

    case STATE_RX_DONE:
        if ((n ? RX2_UPDATE_BITS : RX1_UPDATE_BITS) & state_bits) {
            next_state = STATE_RX;
            break;
        }
        if (!res) {
            if (!n && (buf->len == magic_len))
                if (memcmp(buf->data, BOOTLOADER_MAGIC, magic_len) == 0)
                    run_bootloader();

            usart_set_mode(usart[j], USART_MODE_TX);
            xTaskNotify(modbus_slave,
                        (uint32_t) 1 << (n ? USART2_RTU : USART1_RTU),
                        eSetBits);
            next_state = STATE_WAIT_TX;
        }
        break;

    case STATE_WAIT_TX:
        if ((n ? USART2_SRC : USART1_SRC) & state_bits) {
            if (((n ? USART2_SRC : USART1_SRC) << 1) & state_bits) {
                enable_tx(n, 1);
                enable_tx_dma(n, buf->len);
                next_state = STATE_WAIT_TX_DONE;
            } else {
                usart_set_mode(usart[j], USART_MODE_TX_RX);
                next_state = STATE_RX;
            }
            buf->len = 0;
        }
        break;

    case STATE_WAIT_TX_DONE:
        if ((n ? USART2_TC_BIT : USART1_TC_BIT) & state_bits) {
            usart_set_mode(usart[j], USART_MODE_TX_RX);
            enable_tx(n, 0);
            next_state = STATE_RX;
        }
        break;
    }

    return next_state;
}

void uart_task(void *params)
{
    uint32_t state_bits;
    enum uart_state usart_state[UART_NUM];
    struct modbus_slave_msg *uart_msg[UART_NUM];

    notify = ((struct task_parameters *)params)->uart;
    modbus_slave = ((struct task_parameters *)params)->modbus_slave;

    uart_setup();

    for (int i = 0; i < UART_NUM; i++) {
        uart_msg[i] =
            &(((struct task_parameters *)params)->
              msgs[i ? USART2_RTU : USART1_RTU]);
        *uart_msg[i] = (struct modbus_slave_msg) {
            .data = uart_buf[i].data,
            .length = &uart_buf[i].len,
            .src = notify,
            .src_bits = i ? USART2_SRC : USART1_SRC,
            .rtu_flag = 1,
        };
        usart_state[i] = STATE_RX;
    }

    for (;;) {

        int res = xTaskNotifyWait(0, 0xffffffff, &state_bits, UART_TICK_DELAY);

        for (int i = 0; i < UART_NUM; i++) {
            if (state_bits & (i ? RX2_UPDATE_BITS : RX1_UPDATE_BITS))
                rx_check(i);
            usart_state[i] =
                update_modbus_slave_state(res, usart_state[i], state_bits, i);
        }
    }
}
