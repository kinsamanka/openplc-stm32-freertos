#ifndef BOARD_H
#define BOARD_H

#include <stdint.h>
#include <libopencm3/stm32/gpio.h>

#if defined BOARD_FX3U

#define DISABLE_JTAG_SWD                             /* use all debug pins */

#define RUN_LED                 {{ GPIOD, GPIO10 }, 0}

#define INPUTS                  { \
                                  { GPIOB, GPIO13 }, /* X0.0 */ \
                                  { GPIOB, GPIO14 }, /* X0.1 */ \
                                  { GPIOB, GPIO11 }, /* X0.2 */ \
                                  { GPIOB, GPIO12 }, /* X0.3 */ \
                                  { GPIOE, GPIO15 }, /* X0.4 */ \
                                  { GPIOB, GPIO10 }, /* X0.5 */ \
                                  { GPIOE, GPIO13 }, /* X0.6 */ \
                                  { GPIOE, GPIO14 }, /* X0.7 */ \
                                  { GPIOE, GPIO11 }, /* X1.0 */ \
                                  { GPIOE, GPIO12 }, /* X1.1 */ \
                                  { GPIOE,  GPIO9 }, /* X1.2 */ \
                                  { GPIOE, GPIO10 }, /* X1.3 */ \
                                  { GPIOE,  GPIO7 }, /* X1.4 */ \
                                  { GPIOE,  GPIO8 }, /* X1.5 */ \
                                  { GPIOC,  GPIO7 }, /* power OK   */ \
                                  { GPIOB,  GPIO2 }, /* run switch */ \
                                }

#define OUTPUTS                 { \
                                  { GPIOC,  GPIO9 }, /* Y0.0 */ \
                                  { GPIOC,  GPIO8 }, /* Y0.1 */ \
                                  { GPIOA,  GPIO8 }, /* Y0.2 */ \
                                  { GPIOA,  GPIO0 }, /* Y0.3 */ \
                                  { GPIOB,  GPIO3 }, /* Y0.4 */ \
                                  { GPIOD, GPIO12 }, /* Y0.5 */ \
                                  { GPIOB, GPIO15 }, /* Y0.6 */ \
                                  { GPIOA,  GPIO7 }, /* Y0.7 */ \
                                  { GPIOA,  GPIO6 }, /* Y1.0 */ \
                                  { GPIOA,  GPIO2 }, /* Y1.1 */ \
                                }

#define ANALOG_INPUTS           { \
                                  { GPIOA,  GPIO1 }, /* AD0,  IN1 */ \
                                  { GPIOA,  GPIO3 }, /* AD1,  IN3 */ \
                                  { GPIOC,  GPIO4 }, /* AD2, IN14 */ \
                                  { GPIOC,  GPIO5 }, /* AD3, IN15 */ \
                                  { GPIOC,  GPIO0 }, /* AD4, IN10 */ \
                                  { GPIOC,  GPIO1 }, /* AD5, IN11 */ \
                                  { GPIOC,  GPIO2 }, /* R00, IN12 */ \
                                  { GPIOC,  GPIO3 }, /* R01, IN13 */ \
                                }

#define ADC_CHANNELS            { 1, 3, 14, 15, 10, 11, 12, 13 }

#define ANALOG_OUTPUTS          { \
                                  { GPIOA,  GPIO4 }, /* DA0 */ \
                                  { GPIOA,  GPIO5 }, /* DA1 */ \
                                }

#define USE_DAC_OUT                                  /* use DAC to generate analog outputs */

#define DAC_CHANNELS            { 0, 1 }

#define UART_2                  { \
                                  2, \
                                  GPIOC, \
                                  GPIO11,            /* rs485 rx */ \
                                  GPIO10,            /* rs485 tx */ \
                                  AFIO_MAPR_USART3_REMAP_PARTIAL_REMAP, \
                                  { GPIOA, GPIO14 }, /* rs485 en */ \
                                }

#define CAN_1                   { \
                                  GPIOD, \
                                  GPIO0,             /* can rx */ \
                                  GPIO1,             /* can tx */ \
                                  { 0, 0 },          /* no en  */ \
                                }

#elif defined BOARD_WS214RS

#define RUN_LED                 {{ GPIOB, GPIO10, 0 }, 1}
#define ERR_LED                 {{ GPIOB, GPIO11, 0 }, 1}

#define INPUTS                  { \
                                  { GPIOA,  GPIO0, 0 }, /* X0.0 */ \
                                  { GPIOB,  GPIO3, 0 }, /* X0.1 */ \
                                  { GPIOB,  GPIO4, 0 }, /* X0.2 */ \
                                  { GPIOA,  GPIO1, 0 }, /* X0.3 */ \
                                  { GPIOF,  GPIO1, 0 }, /* X0.4 */ \
                                  { GPIOC, GPIO14, 0 }, /* X0.5 */ \
                                  { GPIOC, GPIO15, 0 }, /* X0.6 */ \
                                  { GPIOB,  GPIO9, 0 }, /* X0.7 */ \
                                  { GPIOB,  GPIO1, 0 }, /* power OK */ \
                                }

#define OUTPUTS                 { \
                                  { GPIOA,  GPIO4, 0 }, /* Y0.0 */ \
                                  { GPIOA,  GPIO7, 0 }, /* Y0.1 */ \
                                  { GPIOB, GPIO14, 0 }, /* Y0.2 */ \
                                  { GPIOB, GPIO15, 0 }, /* Y0.3 */ \
                                  { GPIOA, GPIO12, 0 }, /* Y0.4 */ \
                                  { GPIOA, GPIO11, 0 }, /* Y0.5 */ \
                                }

#define ANALOG_INPUTS           { \
                                  { GPIOA,  GPIO5, 0 }, /* AD0, IN5 */ \
                                  { GPIOB,  GPIO0, 0 }, /* AD1, IN8 */ \
                                }

#define ADC_CHANNELS            { 5, 8 }

#define ANALOG_OUTPUTS          { \
                                  { GPIOA,  GPIO6, GPIO_AF1 }, /* PWM, TIM3_CH1 */ \
                                }

#define TIMER_OC_IDS            { TIM_OC1 }

#define UART_1                  { \
                                  0, \
                                  GPIOA, \
                                  GPIO10,           /* rx    */ \
                                  GPIO9 ,           /* tx    */ \
                                  0,                /* no remap */ \
                                  { 0, 0, 0 },      /* no en */ \
                                }

#elif defined BOARD_BLUEPILL

#define RUN_LED                 {{ GPIOC, GPIO13 }, 0}

#define INPUTS                  { \
                                  { GPIOA,  GPIO8 }, /* %IX0.0 */ \
                                  { GPIOA, GPIO11 }, /* %IX0.1 */ \
                                  { GPIOA, GPIO12 }, /* %IX0.2 */ \
                                  { GPIOB,  GPIO3 }, /* %IX0.3 */ \
                                  { GPIOB,  GPIO4 }, /* %IX0.4 */ \
                                  { GPIOB,  GPIO5 }, /* %IX0.5 */ \
                                  { GPIOB,  GPIO8 }, /* %IX0.6 */ \
                                  { GPIOB,  GPIO9 }, /* %IX0.7 */ \
                                  { GPIOB, GPIO10 }, /* %IX1.0 */ \
                                }

#define OUTPUTS                 { \
                                  { GPIOB, GPIO11 }, /* %QX0.0 */ \
                                  { GPIOB, GPIO12 }, /* %QX0.1 */ \
                                  { GPIOB, GPIO13 }, /* %QX0.2 */ \
                                  { GPIOB, GPIO14 }, /* %QX0.3 */ \
                                  { GPIOB, GPIO15 }, /* %QX0.4 */ \
                                  { GPIOB,  GPIO6 }, /* %QX0.5 */ \
                                  { GPIOC, GPIO14 }, /* %QX0.6 */ \
                                  { GPIOC, GPIO15 }, /* %QX0.7 */ \
                                  { GPIOB,  GPIO7 }, /* %QX1.0 */ \
                                }

#define ANALOG_INPUTS           { \
                                  { GPIOA,  GPIO0 }, /* %IW0, IN0 */ \
                                  { GPIOA,  GPIO1 }, /* %IW1, IN1 */ \
                                  { GPIOA,  GPIO4 }, /* %IW2, IN4 */ \
                                  { GPIOA,  GPIO5 }, /* %IW3, IN5 */ \
                                  { GPIOA,  GPIO6 }, /* %IW4, IN6 */ \
                                  { GPIOA,  GPIO7 }, /* %IW5, IN7 */ \
                                }

#define ADC_CHANNELS            { 0, 1, 4, 5, 6, 7 }

#define ANALOG_OUTPUTS          { \
                                  { GPIOB,  GPIO0 }, /* %QW0, TIM3_CH3 */ \
                                  { GPIOB,  GPIO1 }, /* %QW1, TIM3_CH4 */ \
                                }

#define TIMER_OC_IDS            { TIM_OC3, TIM_OC4 }

#elif defined BOARD_ETH_MODBUS_IO5R

#define RUN_LED                 {{ GPIOA, GPIO8 }, 0}

#define INPUTS                  { \
                                  { GPIOB,  GPIO1 }, /* I0  */ \
                                  { GPIOB, GPIO12 }, /* I1  */ \
                                  { GPIOB, GPIO13 }, /* I2  */ \
                                  { GPIOB, GPIO14 }, /* I3  */ \
                                  { GPIOB, GPIO15 }, /* I4  */ \
                                  { GPIOB,  GPIO8 }, /* btn */ \
                                }

#define OUTPUTS                 { \
                                  { GPIOB,  GPIO5 }, /* Q0 */ \
                                  { GPIOB,  GPIO6 }, /* Q1 */ \
                                  { GPIOB,  GPIO7 }, /* Q2 */ \
                                  { GPIOB, GPIO10 }, /* Q3 */ \
                                  { GPIOB, GPIO11 }, /* Q4 */ \
                                }

#define UART_2                  { \
                                  1, \
                                  GPIOA, \
                                  GPIO3,            /* rs485 rx */ \
                                  GPIO2,            /* rs485 tx */ \
                                  0, \
                                  { GPIOA, GPIO1 }, /* rs485 en */ \
                                }

#define USE_SPI

#else                           /* BOARD_ETH_MODBUS_IO5R */
#error "NO BOARD SPECIFIED! Add 'build_src_flags = -D BOARD_XXXX' on platformio.ini"
#endif

#ifndef UART_1
#define UART_1                  { \
                                  0, \
                                  GPIOA, \
                                  GPIO10,           /* rx    */ \
                                  GPIO9 ,           /* tx    */ \
                                  0,                /* no remap */ \
                                  { 0, 0 },         /* no en */ \
                                }
#endif

#endif
