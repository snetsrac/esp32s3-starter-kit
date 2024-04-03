/**
 * Project: 10.2-touch-lamp
 * Description: Use the touch sensor to control an LED.
*/

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/touch_sensor.h"

#define TOUCH_PAD TOUCH_PAD_NUM14
#define TOUCH_PAD_PIN GPIO_NUM_14

#define PRESSED 100000
#define RELEASED 75000

#define LED GPIO_NUM_21
#define LED_MASK (1ULL << LED)

#define ON 1
#define OFF 0

#define TOUCH_DELAY pdMS_TO_TICKS(50)

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
 * @brief Configure GPIO pin settings.
*/
static void configure_gpio(void) {
    gpio_config_t io_config;
    io_config.pin_bit_mask = LED_MASK;
    io_config.mode = GPIO_MODE_OUTPUT;
    io_config.pull_up_en = GPIO_PULLUP_DISABLE;
    io_config.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_config.intr_type = GPIO_INTR_DISABLE;
    gpio_config(&io_config);
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
 * @brief Task: Control the LED.
*/
static void vTaskControlLed(void *pvParameters) {
    TickType_t xPreviousWakeTime = xTaskGetTickCount();
    uint32_t raw_data;
    bool is_led_on = false;

    gpio_set_level(LED, OFF);

    // Wait for touch pad init
    vTaskDelay(pdMS_TO_TICKS(100));

    for (;;) {
        touch_pad_read_raw_data(TOUCH_PAD, &raw_data);

        if (raw_data > PRESSED && !is_led_on) {
            print_string("Touch detected!");
            gpio_set_level(LED, true);
            is_led_on = true;
        }

        if (raw_data < RELEASED && is_led_on) {
            print_string("Released.");
            gpio_set_level(LED, false);
            is_led_on = false;
        }

        vTaskDelayUntil(&xPreviousWakeTime, TOUCH_DELAY);
    }
}

/**
 * @brief ESP-IDF FreeRTOS application entry point.
*/
void app_main(void)
{
    configure_gpio();
    configure_touch_sensor();

    xTaskCreate(vTaskControlLed, "Control LED state", 2048, NULL, 1, NULL);
}