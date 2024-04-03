/**
 * Project: 10.1-read-touch-sensor
 * Description: Read the value of a touch sensor.
*/

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/touch_sensor.h"

#define TOUCH_PAD TOUCH_PAD_NUM14
#define TOUCH_PAD_PIN GPIO_NUM_14

#define TOUCH_DELAY pdMS_TO_TICKS(1000)

/**
 * @brief Print a string to the console, along with the current tick count.
 * 
 * @param s the string to print
*/
static void print_string(const char *const s) {
    printf("%8u: %s\n", xTaskGetTickCount(), s);
    fflush(stdout);
}

/**
 * @brief Configure touch sensor settings.
*/
static void configure_touch_sensor(void) {
    touch_pad_init();
    touch_pad_config(TOUCH_PAD);
    touch_pad_set_fsm_mode(TOUCH_FSM_MODE_TIMER);
    touch_pad_fsm_start();
}

/**
 * @brief Task: Read and print touch sensor raw value.
*/
static void vTaskReadTouch(void *pvParameters) {
    TickType_t xPreviousWaitTime = xTaskGetTickCount();
    uint32_t raw_data;
    char message[32];
    
    for (;;) {
        touch_pad_read_raw_data(TOUCH_PAD, &raw_data);
        snprintf(message, 32, "Touch value: %d", raw_data);
        print_string(message);
        vTaskDelayUntil(&xPreviousWaitTime, TOUCH_DELAY);
    }
}

/**
 * @brief ESP-IDF FreeRTOS application entry point.
*/
void app_main(void)
{
    configure_touch_sensor();

    xTaskCreate(vTaskReadTouch, "Read touch value", 2048, NULL, 1, NULL);
}