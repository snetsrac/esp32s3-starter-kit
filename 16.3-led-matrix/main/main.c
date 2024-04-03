/**
 * Project: 16.3-led-matrix
 * Description: Use shift registers to control an LED matrix.
*/

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/timers.h"
#include "driver/gpio.h"

#define SER   GPIO_NUM_38
#define RCLK  GPIO_NUM_39
#define SRCLK GPIO_NUM_40

#define HIGH 1
#define LOW  0

#ifdef BIT
#undef BIT
#endif

#define BIT(x) (1ULL << x)

#define REFRESH_DELAY pdMS_TO_TICKS(1)
#define INCREMENT_DELAY pdMS_TO_TICKS(1000)

typedef enum {
    LSB_FIRST,
    MSB_FIRST
} shift_dir_t;

const uint8_t FACE[] = {
    0x3c, 0x42, 0x91, 0x85, 0x85, 0x91, 0x42, 0x3c      // ":)"
};

const uint8_t ALPHANUM[][8] = {
    { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 }, // " "
    { 0x00, 0x00, 0x21, 0x7F, 0x01, 0x00, 0x00, 0x00 }, // "1"
    { 0x00, 0x00, 0x23, 0x45, 0x49, 0x31, 0x00, 0x00 }, // "2"
    { 0x00, 0x00, 0x22, 0x49, 0x49, 0x36, 0x00, 0x00 }, // "3"
    { 0x00, 0x00, 0x0E, 0x32, 0x7F, 0x02, 0x00, 0x00 }, // "4"
    { 0x00, 0x00, 0x79, 0x49, 0x49, 0x46, 0x00, 0x00 }, // "5"
    { 0x00, 0x00, 0x3E, 0x49, 0x49, 0x26, 0x00, 0x00 }, // "6"
    { 0x00, 0x00, 0x60, 0x47, 0x48, 0x70, 0x00, 0x00 }, // "7"
    { 0x00, 0x00, 0x36, 0x49, 0x49, 0x36, 0x00, 0x00 }, // "8"
    { 0x00, 0x00, 0x32, 0x49, 0x49, 0x3E, 0x00, 0x00 }, // "9"
    { 0x00, 0x00, 0x3E, 0x41, 0x41, 0x3E, 0x00, 0x00 }, // "0"
    { 0x00, 0x00, 0x3F, 0x44, 0x44, 0x3F, 0x00, 0x00 }, // "A"
    { 0x00, 0x00, 0x7F, 0x49, 0x49, 0x36, 0x00, 0x00 }, // "B"
    { 0x00, 0x00, 0x3E, 0x41, 0x41, 0x22, 0x00, 0x00 }, // "C"
    { 0x00, 0x00, 0x7F, 0x41, 0x41, 0x3E, 0x00, 0x00 }, // "D"
    { 0x00, 0x00, 0x7F, 0x49, 0x49, 0x41, 0x00, 0x00 }, // "E"
    { 0x00, 0x00, 0x7F, 0x48, 0x48, 0x40, 0x00, 0x00 }  // "F"
};

static TimerHandle_t xIncrementTimer;
static uint8_t data_index;

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
 * @param dir bit direction
*/
static void shift_out(uint8_t byte, shift_dir_t dir) {
    gpio_set_level(SER, LOW);
    gpio_set_level(SRCLK, LOW);

    if (dir == LSB_FIRST) {
        for (int i = 0; i < 8; i++) {
            gpio_set_level(SER, byte & BIT(i));
            gpio_pulse(SRCLK);
        }
    } else if (dir == MSB_FIRST) {
        for (int i = 7; i >= 0; i--) {
            gpio_set_level(SER, byte & BIT(i));
            gpio_pulse(SRCLK);
        }
    }
}

/**
 * @brief Write data to the LED matrix using shift registers.
 * 
 * @param data array of bytes to add, length == 8
*/
static void write_data(uint8_t *data) {
        for (int col = 0; col < 8; col++) {
        shift_out(data[col], LSB_FIRST);
        shift_out(~(0x01 << col), MSB_FIRST);
        gpio_pulse(RCLK);

        vTaskDelay(REFRESH_DELAY);
    }
}

/**
 * @brief Task: Display patterns on an LED matrix.
*/
static void vTaskDisplayHex(void *pvParameters) {
    for (;;) {
        if (data_index == 0) {
            write_data(FACE);
        } else {
            write_data(ALPHANUM[data_index - 1]);
        }
    }
}

/**
 * @brief Increment the index, with wraparound.
*/
static void increment_timer_callback(void *arg) {
    if (data_index >= 1 + 17) {
        data_index = 0;
    } else {
        data_index++;
    }
}

/**
 * @brief ESP-IDF FreeRTOS application entry point.
*/
void app_main(void)
{
    configure_gpio();

    xTaskCreate(vTaskDisplayHex, "Display hexadecimal", 2048, NULL, 1, NULL);

    xIncrementTimer = xTimerCreate(
        "Increment",
        INCREMENT_DELAY,
        pdTRUE,
        NULL,
        increment_timer_callback
    );

    xTimerStart(xIncrementTimer, INCREMENT_DELAY);
}
