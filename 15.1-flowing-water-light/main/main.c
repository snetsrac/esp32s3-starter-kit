/**
 * Project: 15.1-flowing-water-light
 * Description: Use a shift register to control an LED array.
*/

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"

#define SER   GPIO_NUM_12
#define RCLK  GPIO_NUM_13
#define SRCLK GPIO_NUM_14

#define HIGH 1
#define LOW  0

#ifdef BIT
#undef BIT
#endif

#define BIT(x) (1ULL << x)

#define DELAY pdMS_TO_TICKS(100)

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
    io_config.pin_bit_mask = BIT(SER) | BIT(RCLK) | BIT(SRCLK);
    io_config.mode = GPIO_MODE_OUTPUT;
    io_config.pull_up_en = GPIO_PULLUP_DISABLE;
    io_config.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_config.intr_type = GPIO_INTR_DISABLE;
    gpio_config(&io_config);
}

/**
 * @brief Pulse a GPIO pin.
 * 
 * @param pin pin number
*/
static void gpio_pulse(gpio_num_t pin) {
    gpio_set_level(pin, HIGH);
    gpio_set_level(pin, LOW);
}

/**
 * @brief Write a single byte of data to a 74HC595 shift register.
 * 
 * @param byte data to write
*/
static void shift_out(uint8_t byte) {
    gpio_set_level(SER, LOW);
    gpio_set_level(SRCLK, LOW);

    for (int i = 0; i < 8; i++) {
        gpio_set_level(SER, byte & BIT(i));
        gpio_pulse(SRCLK);
    }
}

/**
 * @brief Task: Toggle LEDs in sequence and print a message.
*/
static void vTaskSequenceLed(void *pvParameters) {
    TickType_t xPreviousWakeTime = xTaskGetTickCount();
    uint8_t byte;

    for (;;) {
        print_string("Start sequence.");
        
        byte = 0b10000000;

        for (int i = 0; i < 14; i++) {
            shift_out(byte);
            gpio_pulse(RCLK);

            if (i < 7) {
                byte >>= 1;
            } else {
                byte <<= 1;
            }

            vTaskDelayUntil(&xPreviousWakeTime, DELAY);
        }
    }
}

/**
 * @brief ESP-IDF FreeRTOS application entry point.
*/
void app_main(void)
{
    configure_gpio();

    xTaskCreate(vTaskSequenceLed, "Sequence LEDs", 2048, NULL, 1, NULL);
}
