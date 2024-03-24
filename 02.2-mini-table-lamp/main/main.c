/**
 * Project: 02.2-mini-table-lamp
 * Description: Use a button to control an LED.
*/

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/timers.h"
#include "driver/gpio.h"

#define BUTTON GPIO_NUM_13
#define LED GPIO_NUM_2

#define BUTTON_MASK (1ULL << BUTTON)
#define LED_MASK (1ULL << LED)

#define DOWN 0
#define UP 1

#define ON 1
#define OFF 0

#define DEBOUNCE_PERIOD pdMS_TO_TICKS(20)

static TimerHandle_t xDebounceTimer;

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

    // Button
    io_config.pin_bit_mask = BUTTON_MASK;
    io_config.mode = GPIO_MODE_INPUT;
    io_config.pull_up_en = GPIO_PULLUP_DISABLE;
    io_config.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_config.intr_type = GPIO_INTR_ANYEDGE;
    gpio_config(&io_config);

    // LED
    io_config.pin_bit_mask = LED_MASK;
    io_config.mode = GPIO_MODE_OUTPUT;
    io_config.pull_up_en = GPIO_PULLUP_DISABLE;
    io_config.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_config.intr_type = GPIO_INTR_DISABLE;
    gpio_config(&io_config);
}

/**
 * @brief Interrupt: Respond to button level changes by resetting the timer.
*/
static void IRAM_ATTR button_isr_handler(void *arg) {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    xTimerResetFromISR(xDebounceTimer, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

/**
 * @brief Timer callback function. Handles button event after debounce.
*/
static void handle_button_edge(TimerHandle_t xTimer) {
    int button_level = gpio_get_level(BUTTON);
    static bool led_state = OFF;

    if (button_level == UP) {
        led_state = !led_state;

        if (led_state == ON) {
            print_string("Button released - LED ON");
        } else {
            print_string("Button released - LED OFF");
        }
    } else {
        print_string("Button pressed");
    }

    gpio_set_level(LED, led_state);
}

/**
 * @brief ESP-IDF FreeRTOS application entry point.
*/
void app_main(void)
{
    xDebounceTimer = xTimerCreate(
        "Debounce Timer",
        DEBOUNCE_PERIOD,
        pdFALSE,
        NULL,
        handle_button_edge
    );

    configure_gpio();

    gpio_install_isr_service(0);
    gpio_isr_handler_add(BUTTON, button_isr_handler, NULL);
}
