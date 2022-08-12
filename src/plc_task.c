#include <FreeRTOS.h>
#include <task.h>
#include <semphr.h>

#include <stdio.h>

#include "plc_task.h"
#include "hw.h"
#include "config.h"
#include "iec_std_lib.h"

TIME __CURRENT_TIME;
BOOL __DEBUG;

static unsigned long __tick = 0;

extern unsigned long long common_ticktime__;
extern uint16_t QW[];
extern uint16_t IW[];
extern uint8_t QX[];
extern uint8_t IX[];

extern void config_init__(void);
extern void config_run__(unsigned long);

static void update_time(void)
{
    __CURRENT_TIME.tv_nsec += common_ticktime__;

    if (__CURRENT_TIME.tv_nsec >= 1000000000) {
        __CURRENT_TIME.tv_nsec -= 1000000000;
        __CURRENT_TIME.tv_sec += 1;
    }
}

void plc_task(void *params)
{
    SemaphoreHandle_t *mutex = (SemaphoreHandle_t *) params;

    memset(QW, 0, HOLDING_REG_COUNT * sizeof(uint16_t));
    memset(IW, 0, INPUT_REG_COUNT * sizeof(uint16_t));
    memset(QX, 0, COIL_COUNT * sizeof(uint8_t));
    memset(IX, 0, DISCRETE_COUNT * sizeof(uint8_t));

    config_init__();

    TickType_t last_tick = xTaskGetTickCount();
    TickType_t delay = (TickType_t) (common_ticktime__ / 1000000);

    for (;;) {
        xSemaphoreTake(*mutex, portMAX_DELAY);

        update_inputs();
        config_run__(__tick++);
        update_ouputs();

        xSemaphoreGive(*mutex);

        update_time();

#ifdef ANALOG_INPUTS
	    adc_start();
#endif
        vTaskDelayUntil(&last_tick, delay / portTICK_RATE_MS);
    }
}
