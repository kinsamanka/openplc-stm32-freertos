#include "config.h"
#include "hw.h"

#include <libopencm3/stm32/gpio.h>

#define SET_PIN(a, b, c) \
    do { \
        if (a) \
            gpio_set(b, c); \
        else \
            gpio_clear(b, c); \
    } while (0)

#define GET_PIN(a, b, c) \
    do { \
        a = gpio_get(b, c)  == 0; \
    } while (0)

extern uint16_t QW[];
extern uint16_t IW[];
extern uint8_t QX[];
extern uint8_t IX[];

void update_inputs(void)
{
   GET_PIN(IX[0], GPIOB, GPIO1);
   GET_PIN(IX[1], GPIOB, GPIO12);
   GET_PIN(IX[2], GPIOB, GPIO13);
   GET_PIN(IX[3], GPIOB, GPIO14);
   GET_PIN(IX[4], GPIOB, GPIO15);
}

void update_ouputs(void)
{
    SET_PIN(QX[0], GPIOB, GPIO5);
    SET_PIN(QX[1], GPIOB, GPIO6);
    SET_PIN(QX[2], GPIOB, GPIO7);
    SET_PIN(QX[3], GPIOB, GPIO10);
    SET_PIN(QX[4], GPIOB, GPIO11);
}
