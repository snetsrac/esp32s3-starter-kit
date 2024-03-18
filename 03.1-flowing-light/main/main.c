/**
 * Project: 03.1-flowing-light
 * Description: Blink an LED array in sequence.
*/

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"

#define LED_0 GPIO_NUM_1
#define LED_1 GPIO_NUM_2
#define LED_2 GPIO_NUM_42
#define LED_3 GPIO_NUM_41
#define LED_4 GPIO_NUM_40
#define LED_5 GPIO_NUM_39
#define LED_6 GPIO_NUM_38
#define LED_7 GPIO_NUM_48
#define LED_8 GPIO_NUM_47
#define LED_9 GPIO_NUM_21

#define mask(GPIO_PIN) (1ULL << GPIO_PIN)

#define LED_MASK (mask(LED_0) | mask(LED_1) | mask(LED_2) | mask(LED_3) |\
                  mask(LED_4) | mask(LED_5) | mask(LED_6) | mask(LED_7) |\
                  mask(LED_8) | mask(LED_9))
                  
#define SEQUENCE_PERIOD 2000
#define SEQUENCE_DELAY pdMS_TO_TICKS(SEQUENCE_PERIOD / (NUM_LEDS * 2 - 2))

static const uint8_t LED_PINS[] = {LED_0, LED_1, LED_2, LED_3, LED_4,
                                   LED_5, LED_6, LED_7, LED_8, LED_9};
static const int NUM_LEDS = sizeof(LED_PINS);

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
 * @brief Configure the GPIO pin settings.
 */
static void configure_gpio(void) {
    static gpio_config_t io_config;
    io_config.pin_bit_mask = LED_MASK;
    io_config.mode = GPIO_MODE_OUTPUT;
    io_config.pull_up_en = GPIO_PULLUP_DISABLE;
    io_config.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_config.intr_type = GPIO_INTR_DISABLE;
    gpio_config(&io_config);
}

/**
 * @brief Task: Toggle LEDs in sequence and print a message.
*/
static void vTaskSequenceLed(void *pvParameters) {
    TickType_t xPreviousWakeTime = xTaskGetTickCount();

    for (;;) {
        print_string("Start sequence.");

        // Ascending sequence
        for (int i = 0; i < NUM_LEDS - 1; i++) {
            gpio_set_level(LED_PINS[i], 1);
            vTaskDelayUntil(&xPreviousWakeTime, SEQUENCE_DELAY);
            gpio_set_level(LED_PINS[i], 0);
        }

        // Descending sequence
        for (int i = NUM_LEDS - 1; i > 0; i--) {
            gpio_set_level(LED_PINS[i], 1);
            vTaskDelayUntil(&xPreviousWakeTime, SEQUENCE_DELAY);
            gpio_set_level(LED_PINS[i], 0);
        }
    }
}

/**
 * @brief ESP-IDF FreeRTOS application entry point.
*/
void app_main(void)
{
    configure_gpio();
    xTaskCreate(vTaskSequenceLed, "Sequence LEDs", 2000, NULL, 1, NULL);
}
