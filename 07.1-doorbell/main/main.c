/**
 * Project: 07.1-doorbell
 * Description: Use a button to control an active buzzer.
*/

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/timers.h"
#include "driver/gpio.h"

#define BUTTON GPIO_NUM_21
#define BUZZER GPIO_NUM_14

#define BUTTON_MASK (1ULL << BUTTON)
#define BUZZER_MASK (1ULL << BUZZER)

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

    // Buzzer
    io_config.pin_bit_mask = BUZZER_MASK;
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
 * 
 * @param xTimer handle to button timer
*/
static void handle_button_edge(TimerHandle_t xTimer) {
    int button_level = gpio_get_level(BUTTON);

    if (button_level == DOWN) {
        gpio_set_level(BUZZER, ON);
        print_string("Button pressed");
    } else {
        gpio_set_level(BUZZER, OFF);
        print_string("Button released");
    }
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
