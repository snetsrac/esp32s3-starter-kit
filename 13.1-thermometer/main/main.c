/**
 * Project: 13.1-thermometer
 * Description: Use a thermistor to calculate temperature.
*/

#include <stdio.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/adc.h"

#define INPUT GPIO_NUM_1
#define INPUT_MASK (1ULL << INPUT)

#define ADC_CHANNEL ADC1_CHANNEL_0
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
    io_config.pin_bit_mask = INPUT_MASK;
    io_config.mode = GPIO_MODE_INPUT;
    io_config.pull_up_en = GPIO_PULLUP_DISABLE;
    io_config.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_config.intr_type = GPIO_INTR_DISABLE;
    gpio_config(&io_config);
}

/**
 * @brief Configure ADC settings.
*/
static void configure_adc(void) {
    adc1_config_width(ADC_WIDTH_BIT_DEFAULT);
    adc1_config_channel_atten(ADC_CHANNEL, ADC_ATTEN_DB_11);
}

/**
 * @brief Task: Read and print the value from the ADC.
*/
static void vTaskReadAdc(void *pvParameters) {
    TickType_t xPreviousWakeTime = xTaskGetTickCount();
    int adc_value;
    float voltage;
    float resistance;
    float temperature;
    char message[64];

    for (;;) {
        adc_value = adc1_get_raw(ADC_CHANNEL);
        voltage = adc_value / 4095.0f * 3.3f;
        resistance = 10000 * voltage / (3.3 - voltage);
        temperature = 1 / (1 / (273.15 + 25) + logf(resistance / 10000) / 3950.0) - 273.15;

        snprintf(message, 64, "ADC Val: %4d, Voltage: %.2fV, Temperature: %.2f",
                 adc_value, voltage, temperature);
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

    xTaskCreate(vTaskReadAdc, "Read ADC value", 2048, NULL, 1, NULL);
}
