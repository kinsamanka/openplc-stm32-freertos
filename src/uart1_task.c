#include "modbus_slave.h"
#include <task.h>
#include <queue.h>

#include <libopencm3/stm32/usart.h>
#include <libopencm3/cm3/nvic.h>

#include <lightmodbus/lightmodbus.h>

#include <string.h>

#include "uart1_task.h"

static struct {
    char data[UART_BUF_LEN];
    uint8_t length;
} uart_buf;

static ModbusErrorInfo mberr;
static ModbusSlave rtu_slave;

static uint8_t rtu_slave_buf[MAX_RESPONSE];
static struct context context;

static QueueHandle_t u1rx_q, u1tx_q;

void usart1_isr(void)
{
    char c;
    portBASE_TYPE y = pdFALSE;

    if (((USART_CR1(USART1) & USART_CR1_RXNEIE) != 0) &&
        ((USART_SR(USART1) & USART_SR_RXNE) != 0)) {

        c = usart_recv(USART1);
        xQueueSendFromISR(u1rx_q, &c, &y);
    }

    if (((USART_CR1(USART1) & USART_CR1_TXEIE) != 0) &&
        ((USART_SR(USART1) & USART_SR_TXE) != 0)) {

        if (xQueueReceiveFromISR(u1tx_q, &c, &y)) {
            usart_send(USART1, c);
        } else {
            /* Disable the TXE interrupt */
            USART_CR1(USART1) &= ~USART_CR1_TXEIE;
        }
    }
    portYIELD_FROM_ISR(y);
}

void uart1_setup(void)
{
    if ((SLAVE_PARITY == USART_PARITY_EVEN)
        || (SLAVE_PARITY == USART_PARITY_ODD)) {
        usart_set_databits(USART1, 9);
        usart_set_stopbits(USART1, USART_STOPBITS_1);
    } else {
        usart_set_databits(USART1, 8);
        usart_set_stopbits(USART1, USART_STOPBITS_2);
    }
    usart_set_baudrate(USART1, SLAVE_BAUD_RATE);
    usart_set_mode(USART1, USART_MODE_TX_RX);
    usart_set_parity(USART1, SLAVE_PARITY);
    usart_set_flow_control(USART1, USART_FLOWCONTROL_NONE);

    nvic_enable_irq(NVIC_USART1_IRQ);
    USART_CR1(USART1) |= USART_CR1_RXNEIE;
}

static void uart_putc(char c)
{
    while (!xQueueSend(u1tx_q, &c, portMAX_DELAY)) ;
    /* Enable the TXE interrupt */
    USART_CR1(USART1) |= USART_CR1_TXEIE;
}

static int uart_getc(char *c, TickType_t wait)
{
    return xQueueReceive(u1rx_q, c, wait);
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

static void store_rx(char c)
{
    uart_buf.data[uart_buf.length++] = c;
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
    char c;
    int p = uxTaskPriorityGet(NULL);

    u1rx_q = xQueueCreate(1, sizeof(uint8_t));
    u1tx_q = xQueueCreate(1, sizeof(uint8_t));

    SemaphoreHandle_t *mutex = (SemaphoreHandle_t *) params;

    modbus_uart_init(mutex);

    enum task_state state = STATE_IDLE;

    usart_enable(USART1);

    for (;;) {
        switch (state) {

        case STATE_IDLE:
            /* set RX to higher priority and drop after receiving */
            vTaskPrioritySet(NULL, p);

            uart_buf.length = 0;
            if (uart_getc(&c, WAIT_INFINITE)) {
                store_rx(c);
                state = STATE_RECEIVING;
            }
            break;

        case STATE_RECEIVING:
            if (uart_getc(&c, WAIT_END_FRAME)) {        /* no timeout */
                store_rx(c);
                if (uart_buf.length > MAX_REQUEST)      /* overflowed */
                    state = STATE_INVALID;

            } else {            /* done */

                vTaskPrioritySet(NULL, p - 1);  /* drop priority */
                c = handle_request();

                if (c)
                    state = STATE_TRANSMITTING;
                else
                    state = STATE_IDLE;
            }
            break;

        case STATE_TRANSMITTING:
            c = 0;
            while (c < uart_buf.length)
                uart_putc(uart_buf.data[c++]);

            state = STATE_IDLE;
            break;

        case STATE_INVALID:
            /* flush remaining rx */
            while (uart_getc(&c, WAIT_END_FRAME)) ;
            state = STATE_IDLE;
            break;
        }
    }
}
