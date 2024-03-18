/**
 * Project: 04.1-breathing-led
 * Description: Slowly pulse an LED.
*/

#include <stdio.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/ledc.h"

#define LED GPIO_NUM_2
#define LED_MASK (1ULL << LED)

#define DUTY_RESOLUTION LEDC_TIMER_8_BIT
#define DUTY_MAX (1U << DUTY_RESOLUTION)
#define DUTY_DELAY pdMS_TO_TICKS(10)
#define PWM_FREQ 1000

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
        .gpio_num = LED,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = LEDC_CHANNEL_0,
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = LEDC_TIMER_0,
        .duty = 0
    };
    ledc_channel_config(&channel_config);
}

/**
 * @brief Update the PWM duty cycle.
 * 
 * @param duty duty cycle, [0, DUTY_MAX)
*/
static void change_pwm_duty(const uint32_t duty) {
    assert(duty < DUTY_MAX);

    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, duty);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
}

/**
 * @brief Calculate duty cycle using a polynomial to account for
 *        non-linearity of the human eye.
 * 
 * @param n duty cycle, [0, DUTY_MAX)
 * @returns duty cycle, [0, DUTY_MAX)
*/
static uint32_t calculate_duty(const int n) {
    assert(n >= 0);
    assert(n < DUTY_MAX);

    // Scale to [0,1), apply polynomial, rescale to [0, DUTY_MAX)
    float x = (float)n / (float)DUTY_MAX;
    x = powf(x, 3.0);
    x = x * DUTY_MAX;
    x = floorf(x);

    return (uint32_t)x;
}

/**
 * @brief Task: Slowly pulse an LED.
*/
static void vTaskPulseLed(void *pvParameters) {
    uint32_t duty;
    TickType_t xPreviousWakeTime = xTaskGetTickCount();

    for (;;) {
        print_string("Start pulse.");

        // Increasing brightness
        // Uses 32 as minimum to limit the amount of time the LED is off
        for (int i = 32; i < DUTY_MAX; i++) {
            duty = calculate_duty(i);
            change_pwm_duty(duty);
            vTaskDelayUntil(&xPreviousWakeTime, DUTY_DELAY);
        }

        // Decreasing brightness
        for (int i = DUTY_MAX - 1; i >= 32; i--) {
            duty = calculate_duty(i);
            change_pwm_duty(duty);
            vTaskDelayUntil(&xPreviousWakeTime, DUTY_DELAY);
        }
    }
}

/**
 * @brief ESP-IDF FreeRTOS application entry point.
*/
void app_main(void)
{
    configure_gpio();
    configure_pwm_timer();
    configure_pwm_channel();
    xTaskCreate(vTaskPulseLed, "Pulse LED", 2000, NULL, 1, NULL);
}
