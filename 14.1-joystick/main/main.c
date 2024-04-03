/**
 * Project: 14.1-adc
 * Description: Read the input from a joystick.
*/

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/adc.h"

#define JOY_X GPIO_NUM_14
#define JOY_Y GPIO_NUM_13
#define JOY_Z GPIO_NUM_12

#define mask(INPUT) (1ULL << INPUT)

#define INPUT_MASK mask(JOY_X) | mask(JOY_Y) | mask(JOY_Z)

#define JOY_X_CHANNEL ADC2_CHANNEL_3
#define JOY_Y_CHANNEL ADC2_CHANNEL_2
#define ADC_WIDTH ADC_WIDTH_BIT_DEFAULT

#define ADC_DELAY pdMS_TO_TICKS(250)

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
    io_config.pin_bit_mask = mask(JOY_X) | mask(JOY_Y);
    io_config.mode = GPIO_MODE_INPUT;
    io_config.pull_up_en = GPIO_PULLUP_DISABLE;
    io_config.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_config.intr_type = GPIO_INTR_DISABLE;
    gpio_config(&io_config);

    io_config.pin_bit_mask = mask(JOY_Z);
    io_config.pull_up_en = GPIO_PULLUP_ENABLE;
    gpio_config(&io_config);
}

/**
 * @brief Configure ADC settings.
*/
static void configure_adc(void) {
    adc2_config_channel_atten(JOY_X_CHANNEL, ADC_ATTEN_DB_11);
    adc2_config_channel_atten(JOY_Y_CHANNEL, ADC_ATTEN_DB_11);
}

/**
 * @brief Task: Read and print the joystick values.
*/
static void vTaskReadJoystick(void *pvParameters) {
    TickType_t xPreviousWakeTime = xTaskGetTickCount();
    int joy_x_value, joy_y_value, joy_z_value;
    char message[64];

    for (;;) {
        adc2_get_raw(JOY_X_CHANNEL, ADC_WIDTH, &joy_x_value);
        adc2_get_raw(JOY_Y_CHANNEL, ADC_WIDTH, &joy_y_value);
        joy_z_value = gpio_get_level(JOY_Z);

        snprintf(message, 32, "X, Y, Z: %4d, %4d, %1d",
                 joy_x_value, joy_y_value, joy_z_value);
        print_string(message);

        vTaskDelayUntil(&xPreviousWakeTime, ADC_DELAY);
    }
}

/**
 * @brief ESP-IDF FreeRTOS application entry point.
*/
void app_main(void)
{
    configure_gpio();
    configure_adc();

    xTaskCreate(vTaskReadJoystick, "Read joystick values", 2048, NULL, 1, NULL);
}
