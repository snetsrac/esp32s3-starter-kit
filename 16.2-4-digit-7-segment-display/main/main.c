/**
 * Project: 16.2-4-digit-7-segment-display
 * Description: Use a shift register to control a 4 digit 7 segment display.
*/

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/timers.h"
#include "driver/gpio.h"

#define SER   GPIO_NUM_38
#define RCLK  GPIO_NUM_39
#define SRCLK GPIO_NUM_40

#define DIGIT_0 GPIO_NUM_21
#define DIGIT_1 GPIO_NUM_36
#define DIGIT_2 GPIO_NUM_35
#define DIGIT_3 GPIO_NUM_47

#define HIGH 1
#define LOW  0

#ifdef BIT
#undef BIT
#endif

#define BIT(x) (1ULL << x)

#define REFRESH_DELAY pdMS_TO_TICKS(5)
#define INCREMENT_DELAY pdMS_TO_TICKS(100)

static const uint64_t DIGITS[] = { DIGIT_0, DIGIT_1, DIGIT_2, DIGIT_3 };

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

static TimerHandle_t xIncrementTimer;
static uint16_t counter;

/**
 * @brief Configure the GPIO pin settings.
 */
static void configure_gpio(void) {
    static gpio_config_t io_config;
    io_config.pin_bit_mask = BIT(SER) | BIT(RCLK) | BIT(SRCLK) |
        BIT(DIGIT_0) | BIT(DIGIT_1) | BIT(DIGIT_2) | BIT(DIGIT_3);
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
 * @brief Write four digits to the 7 segment display via a shift register.
 * 
 * @param data number to write
*/
static void write_data(uint16_t data) {
    uint8_t digit;
    uint16_t x = 0x01;
    
    for (int i = 0; i < 4; i++) {
        digit = data / x % 16;
        
        gpio_set_level(DIGITS[i], HIGH);

        shift_out(CHARS[digit]);
        gpio_pulse(RCLK);

        vTaskDelay(REFRESH_DELAY);

        gpio_set_level(DIGITS[i], LOW);

        x <<= 4;
    }
}

/**
 * @brief Task: Display counter on a 4 digit 7 segment display.
*/
static void vTaskDisplayHex(void *pvParameters) {
    for (;;) {
        write_data(counter);
    }
}

/**
 * @brief Increment the counter.
*/
static void increment_timer_callback(void *arg) {
    counter++;
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
