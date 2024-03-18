/**
 * Project: 04.2-meteor-flowing-light
 * Description: Pulse an LED array in sequence.
*/

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/ledc.h"

#define LED_0 GPIO_NUM_1
#define LED_1 GPIO_NUM_2
#define LED_2 GPIO_NUM_42
#define LED_3 GPIO_NUM_41
#define LED_4 GPIO_NUM_40
#define LED_5 GPIO_NUM_39
#define LED_6 GPIO_NUM_38
#define LED_7 GPIO_NUM_48

#define mask(GPIO_PIN) (1ULL << GPIO_PIN)

#define LED_MASK (mask(LED_0) | mask(LED_1) | mask(LED_2) | mask(LED_3) |\
                  mask(LED_4) | mask(LED_5) | mask(LED_6) | mask(LED_7))

#define DUTY_RESOLUTION LEDC_TIMER_10_BIT
#define DUTY_DELAY pdMS_TO_TICKS(100)
#define PWM_FREQ 1000

static const uint8_t LED_PINS[] = {LED_0, LED_1, LED_2, LED_3,
                                   LED_4, LED_5, LED_6, LED_7};

static const uint8_t PWM_CHANNELS[] = {0, 1, 2, 3, 4, 5, 6, 7};
static const uint8_t NUM_CHANNELS = sizeof(PWM_CHANNELS);

static const uint32_t DUTYS[] = {0,      0,   0,   0,  0,  0,  0, 0,
                                 1023, 512, 256, 128, 64, 32, 16, 8,
                                 0,      0,   0,   0,  0,  0,  0, 0};
static const uint32_t DUTY_MAX = 1U << DUTY_RESOLUTION;

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
    static gpio_config_t io_config;
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
 * 
 * @param gpio_num GPIO pin number
 * @param channel PWM channel
*/
static void configure_pwm_channel(int gpio_num, ledc_channel_t channel) {
    ledc_channel_config_t channel_config = {
        .gpio_num = gpio_num,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = channel,
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = LEDC_TIMER_0,
        .duty = 0
    };
    ledc_channel_config(&channel_config);
}

/**
 * @brief Update the PWM duty cycle.
 * 
 * @param channel PWM channel
 * @param duty duty cycle, [0, DUTY_MAX)
*/
static void change_pwm_duty(ledc_channel_t channel, uint32_t duty) {
    assert(duty < DUTY_MAX);
    ledc_set_duty(LEDC_LOW_SPEED_MODE, channel, duty);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, channel);
}

/**
 * @brief Task: Pulse an LED array in sequence.
*/
static void vTaskPulseSequence(void *pvParameters) {
    TickType_t xPreviousWakeTime = xTaskGetTickCount();

    for (;;) {
        print_string("Start pulse sequence.");

        for (int i = 0; i < 16; i++) {
            for (int j = 0; j < NUM_CHANNELS; j++) {
                change_pwm_duty(j, DUTYS[i + j]);
            }

            vTaskDelayUntil(&xPreviousWakeTime, DUTY_DELAY);
        }

        for (int i = 0; i < 16; i++) {
            for (int j = NUM_CHANNELS - 1; j >= 0; j--) {
                change_pwm_duty(j, DUTYS[i + (NUM_CHANNELS - 1 - j)]);
            }

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

    for (int i = 0; i < NUM_CHANNELS; i++) {
        configure_pwm_channel(LED_PINS[i], PWM_CHANNELS[i]);
    }

    xTaskCreate(vTaskPulseSequence, "Pulse LED sequence", 2000, NULL, 1, NULL);
}
