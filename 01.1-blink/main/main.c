/**
 * Project: 01.1-blink
 * Description: Blink an LED using GPIO.
*/

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"

#define LED GPIO_NUM_2
#define LED_MASK (1ULL << LED)
#define BLINK_PERIOD 1000UL
#define BLINK_DELAY pdMS_TO_TICKS(BLINK_PERIOD / 2)

/**
 * @brief Print a string to the console, along with the current tick count.
 * 
 * @param s The string to print
*/
static void print_string(const char *const s) {
    printf("%8u: %s\n", xTaskGetTickCount(), s);
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
 * @brief Task: Toggle LED state and print a message.
*/
static void vTaskToggleLed(void *pvParameters) {
    bool led_state = 0;
    TickType_t pxPreviousWakeTime = xTaskGetTickCount();

    for (;;) {
        led_state = !led_state;
        print_string(led_state ? "Turning LED on." : "Turning LED off.");
        gpio_set_level(LED, led_state);
        xTaskDelayUntil(&pxPreviousWakeTime, BLINK_DELAY);
    }
}

/**
 * @brief ESP-IDF FreeRTOS application entry point.
*/
void app_main(void)
{
    configure_gpio();
    xTaskCreate(vTaskToggleLed, "Toggle LED", 1000, NULL, 1, NULL);
}
