#ifndef HW_H
#define HW_H

#include <stdint.h>

#define USE_UART1

#ifdef STM32F1

#define PORT_LED                    GPIOA
#define PIN_LED                     GPIO8

#define RX1_PERIF_ADDR              USART1_DR
#define TX1_PERIF_ADDR              USART1_DR

#define USE_UART2
#define USE_SPI

#else                           /* STM32F1 */

#define PORT_LED                    GPIOB
#define PIN_LED                     GPIO10

#define GPIO_USART1_RX              GPIO10
#define GPIO_USART1_TX              GPIO9

#define RX1_PERIF_ADDR              USART1_RDR
#define TX1_PERIF_ADDR              USART1_TDR

#endif                          /* !STM32F1 */

void update_inputs(void);
void update_ouputs(void);
void hw_io_setup(void);
void run_bootloader(void);
void check_boot_flag(void);
void uart_set_gpio(uint32_t gpioport, uint16_t rxpin, uint16_t txpin);
void spi_setup(void);
void spi_set_gpio(void);

#endif
