/**
 * Project: 16.1-7-segment-display
 * Description: Use a shift register to control a 7 segment display.
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

#define DELAY pdMS_TO_TICKS(1000)

static const uint8_t CHARS[] = {
    0xc0,   // 0
    0xf9,   // 1
    0xa4,   // 2
    0xb0,   // 3
    0x99,   // 4
    0x92,   // 5
    0x82,   // 6
    0xf8,   // 7
    0x80,   // 8
    0x90,   // 9
    0x88,   // A
    0x83,   // B
    0xc6,   // C
    0xa1,   // D
    0x86,   // E
    0x8e    // F
};

/**
 * @brief Print a char to the console, along with the current tick count.
 * 
 * @param s the char to print
*/
static void print_char(const char c) {
    printf("%8u: Char = %c\n", xTaskGetTickCount(), c);
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
 * @brief Task: Display hexadecimal characters on a 7 segment display.
*/
static void vTaskDisplayHex(void *pvParameters) {
    TickType_t xPreviousWakeTime = xTaskGetTickCount();

    for (;;) {
        for (int i = 0; i < 16; i++) {
            shift_out(CHARS[i]);
            gpio_pulse(RCLK);

            if (i <= 9) {
                print_char(i + '0');
            } else {
                print_char(i - 10 + 'A');
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

    xTaskCreate(vTaskDisplayHex, "Display hexadecimal", 2048, NULL, 1, NULL);
}
