/**
 * Project: 05.1-random-color-light
 * Description: Display random colors on an RGB LED.
*/

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/ledc.h"

#define LED_R GPIO_NUM_40
#define LED_G GPIO_NUM_39
#define LED_B GPIO_NUM_38

#define mask(GPIO_PIN) (1ULL << GPIO_PIN)

#define LED_MASK (mask(LED_R) | mask(LED_G) | mask(LED_B))

#define DUTY_RESOLUTION LEDC_TIMER_8_BIT
#define DUTY_MAX (1ULL << DUTY_RESOLUTION)
#define DUTY_DELAY pdMS_TO_TICKS(1000)
#define PWM_FREQ 1000

static const uint8_t LED_PINS[] = {LED_R, LED_G, LED_B};

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
    gpio_config_t io_config = {
        .pin_bit_mask = LED_MASK,
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
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
 * @brief Set the PWM duty cycle for each color channel.
 * 
 * @param r Red value [0, 255]
 * @param g Green value [0, 255]
 * @param b Blue value [0, 255]
*/
static void set_color_pwm_duty(uint8_t r, uint8_t g, uint8_t b) {
    // The chosen RGB LED uses a common positive, so each color pin must
    // be pulled to ground (low value) for max brightness.
    ledc_set_duty(LEDC_LOW_SPEED_MODE, 0, 255 - r);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, 0);

    ledc_set_duty(LEDC_LOW_SPEED_MODE, 1, 255 - g);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, 1);

    ledc_set_duty(LEDC_LOW_SPEED_MODE, 2, 255 - b);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, 2);
}

/**
 * @brief Task: Display random colors.
*/
static void vTaskDisplayRandomColors(void *pvParameters) {
    TickType_t xPreviousWakeTime = xTaskGetTickCount();
    uint8_t r, g, b;
    char message[32];
    
    for (;;) {
        r = random() % 256;
        g = random() % 256;
        b = random() % 256;

        snprintf(message, sizeof(message), "RGB color = (%u, %u, %u)", r, g, b);
        print_string(message);

        set_color_pwm_duty(r, g, b);
        vTaskDelayUntil(&xPreviousWakeTime, DUTY_DELAY);
    }
}

/**
 * @brief ESP-IDF FreeRTOS application entry point.
*/
void app_main(void)
{
    configure_gpio();
    configure_pwm_timer();

    for (int i = 0; i < 3; i++) {
        configure_pwm_channel(LED_PINS[i], i);
    }

    xTaskCreate(vTaskDisplayRandomColors,
                "Display random colors",
                2000,
                NULL,
                1,
                NULL);
}