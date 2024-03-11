/**
 * Project: 01.1-blink
 * Description: Blink an LED using GPIO.
*/

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"

#define GPIO_LED GPIO_NUM_2
#define GPIO_PIN_MASK_OUTPUT (1ULL << GPIO_LED)
#define BLINK_PERIOD 1000UL

/**
 * @brief Print a string to the console, along with the current tick count.
 * 
 * @param s The string to print
*/
void print_string(const char *const s) {
    printf("%8u: %s\n", xTaskGetTickCount(), s);
}

/**
 * @brief Configure GPIO pin settings.
*/
void configure_gpio(void) {
    gpio_config_t io_conf;
    io_conf.pin_bit_mask = GPIO_PIN_MASK_OUTPUT;
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.intr_type = GPIO_INTR_DISABLE;
    gpio_config(&io_conf);
}

/**
 * @brief Task: Toggle led state and print a message.
*/
void vTaskToggleLed(void *pvParameters) {
    bool led_state = 0;
    TickType_t pxPreviousWakeTime = xTaskGetTickCount();
    const TickType_t xTimeIncrement = pdMS_TO_TICKS(BLINK_PERIOD / 2);

    for (;;) {
        led_state = !led_state;
        print_string(led_state ? "Turning LED on." : "Turning LED off.");
        gpio_set_level(GPIO_LED, led_state);
        xTaskDelayUntil(&pxPreviousWakeTime, xTimeIncrement);
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
