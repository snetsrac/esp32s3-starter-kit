/**
 * Project: 11.2-soft-colorful-light
 * Description: Use the ADC to control an RGB LED.
*/

#include <stdio.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/adc.h"
#include "driver/gpio.h"
#include "driver/ledc.h"

#define LED_R GPIO_NUM_40
#define LED_G GPIO_NUM_39
#define LED_B GPIO_NUM_38
#define LED_MASK (1ULL << LED_R | 1ULL << LED_G | 1ULL << LED_B)

#define ADC_R ADC2_CHANNEL_3
#define ADC_G ADC2_CHANNEL_2
#define ADC_B ADC2_CHANNEL_1

#define DUTY_RESOLUTION LEDC_TIMER_8_BIT
#define DUTY_MAX (1U << DUTY_RESOLUTION)
#define PWM_FREQ 1000
#define LEDC_R LEDC_CHANNEL_0
#define LEDC_G LEDC_CHANNEL_1
#define LEDC_B LEDC_CHANNEL_2

#define DUTY_DELAY pdMS_TO_TICKS(10)

/**
 * @brief Configure the GPIO output.
*/
static void configure_gpio(void) {
    gpio_config_t io_config;
    io_config.pin_bit_mask = LED_MASK;
    io_config.mode = GPIO_MODE_OUTPUT;
    io_config.pull_up_en = GPIO_PULLUP_DISABLE;
    io_config.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_config.intr_type = GPIO_INTR_DISABLE;
    gpio_config(&io_config);
}

/**
 * @brief Configure ADC settings.
*/
static void configure_adc(void) {
    adc2_config_channel_atten(ADC_R, ADC_ATTEN_DB_11);
    adc2_config_channel_atten(ADC_G, ADC_ATTEN_DB_11);
    adc2_config_channel_atten(ADC_B, ADC_ATTEN_DB_11);
}

/**
 * @brief Configure the PWM timer.
*/
static void configure_pwm_timer(void) {
    ledc_timer_config_t timer_config = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .duty_resolution = DUTY_RESOLUTION,
        .timer_num = LEDC_TIMER_0,
        .freq_hz = PWM_FREQ,
        .clk_cfg = LEDC_AUTO_CLK
    };
    ledc_timer_config(&timer_config);
}

/**
 * @brief Configure the PWM channel.
*/
static void configure_pwm_channel(void) {
    ledc_channel_config_t channel_config = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = LEDC_TIMER_0,
        .duty = 0
    };

    channel_config.gpio_num = LED_R;
    channel_config.channel = LEDC_R;
    ledc_channel_config(&channel_config);

    channel_config.gpio_num = LED_G;
    channel_config.channel = LEDC_G;
    ledc_channel_config(&channel_config);

    channel_config.gpio_num = LED_B;
    channel_config.channel = LEDC_B;
    ledc_channel_config(&channel_config);
}

/**
 * @brief Update the PWM duty cycle.
 * 
 * @param channel PWM channel
 * @param duty duty cycle, [0, DUTY_MAX)
*/
static void change_pwm_duty(const ledc_channel_t channel, const uint32_t duty) {
    assert(duty <= DUTY_MAX);

    ledc_set_duty(LEDC_LOW_SPEED_MODE, channel, duty);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, channel);
}

/**
 * @brief Calculate duty cycle using a polynomial to account for
 *        non-linearity of the human eye.
 * 
 * @param n duty cycle, [0, DUTY_MAX]
 * @returns duty cycle, [0, DUTY_MAX]
*/
static uint32_t calculate_duty(const int n) {
    assert(n >= 0);
    assert(n <= DUTY_MAX);

    // Scale to [0,1), apply polynomial, rescale to [0, DUTY_MAX]
    float x = (float)n / (float)DUTY_MAX;
    x = powf(x, 3.0);
    x = x * DUTY_MAX;
    x = floorf(x);

    return (uint32_t)x;
}

/**
 * @brief Use the ADC to update LED brightness.
 * 
 * @param adc ADC2 channel
 * @param ledc LEDC channel
*/
static void update_led(adc2_channel_t adc, ledc_channel_t ledc) {
    int adc_value;
    adc2_get_raw(adc, ADC_WIDTH_BIT_DEFAULT, &adc_value);
    uint32_t duty = (uint32_t)(adc_value / 4095.0f * DUTY_MAX);
    duty = calculate_duty(duty);

    // RGB LED uses common positive + pull-down
    change_pwm_duty(ledc, 256 - duty);
}

/**
 * @brief Task: Use the ADC to control RGB LED brightness.
*/
static void vTaskAdcLedControl(void *pvParameters) {
    TickType_t xPreviousWakeTime = xTaskGetTickCount();

    for (;;) {
            update_led(ADC_R, LEDC_R);
            update_led(ADC_G, LEDC_G);
            update_led(ADC_B, LEDC_B);

            vTaskDelayUntil(&xPreviousWakeTime, DUTY_DELAY);
    }
}

/**
 * @brief ESP-IDF FreeRTOS application entry point.
*/
void app_main(void)
{
    configure_gpio();
    configure_adc();
    configure_pwm_timer();
    configure_pwm_channel();
    xTaskCreate(vTaskAdcLedControl, "ADC LED control", 2000, NULL, 1, NULL);
}
