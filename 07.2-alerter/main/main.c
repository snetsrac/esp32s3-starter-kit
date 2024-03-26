/**
 * Project: 07.2-alerter
 * Description: Use a button to control a passive buzzer.
*/

#include <stdio.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/timers.h"
#include "driver/gpio.h"
#include "driver/ledc.h"

#define BUTTON GPIO_NUM_21
#define BUZZER GPIO_NUM_14

#define BUTTON_MASK (1ULL << BUTTON)
#define BUZZER_MASK (1ULL << BUZZER)

#define LEDC_SPEED_MODE LEDC_LOW_SPEED_MODE
#define DUTY_RESOLUTION LEDC_TIMER_8_BIT
#define LEDC_TIMER LEDC_TIMER_0
#define LEDC_CHANNEL LEDC_CHANNEL_0

#define DOWN 0
#define UP 1

#define ON 1
#define OFF 0

#define DEBOUNCE_PERIOD pdMS_TO_TICKS(20)

#define PI 3.14159f
#define DEG_TO_RAD (PI / 180.0f)

static TimerHandle_t xDebounceTimer;
static QueueHandle_t xAlertQueue;

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

    // Button
    io_config.pin_bit_mask = BUTTON_MASK;
    io_config.mode = GPIO_MODE_INPUT;
    io_config.pull_up_en = GPIO_PULLUP_DISABLE;
    io_config.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_config.intr_type = GPIO_INTR_ANYEDGE;
    gpio_config(&io_config);

    // Buzzer
    io_config.pin_bit_mask = BUZZER_MASK;
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
        .speed_mode = LEDC_SPEED_MODE,
        .duty_resolution = DUTY_RESOLUTION,
        .timer_num = LEDC_TIMER,
        .freq_hz = 1000,
        .clk_cfg = LEDC_AUTO_CLK
    };
    ledc_timer_config(&timer_config);
}

/**
 * @brief Configure the PWM channel.
*/
static void configure_pwm_channel(void) {
    ledc_channel_config_t channel_config = {
        .gpio_num = BUZZER,
        .speed_mode = LEDC_SPEED_MODE,
        .channel = LEDC_CHANNEL,
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = LEDC_TIMER,
        .duty = 0
    };
    ledc_channel_config(&channel_config);
}

/**
 * @brief Set the frequency of the PWM generator.
 * 
 * @param freq frequency, Hz
*/
static void ledc_write_tone(uint32_t freq) {
    if (freq == 0) {
        // Can't set frequency to 0, so set duty to 0 instead
        ledc_set_duty(LEDC_SPEED_MODE, LEDC_CHANNEL, 0);
        ledc_set_freq(LEDC_SPEED_MODE, LEDC_CHANNEL, 1000);
    } else {
        ledc_set_duty(LEDC_SPEED_MODE, LEDC_CHANNEL, 127);
        ledc_set_freq(LEDC_SPEED_MODE, LEDC_CHANNEL, freq);
    }

    ledc_update_duty(LEDC_SPEED_MODE, LEDC_CHANNEL);
}

/**
 * @brief Interrupt: Respond to button level changes by resetting the timer.
*/
static void IRAM_ATTR button_isr_handler(void *arg) {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    xTimerResetFromISR(xDebounceTimer, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

/**
 * @brief Timer callback function. Handles button event after debounce.
 * 
 * @param xTimer handle to button timer
*/
static void handle_button_edge(TimerHandle_t xTimer) {
    int button_level = gpio_get_level(BUTTON);
    bool alert = (button_level == DOWN);

    if (button_level == DOWN) {
        print_string("Button pressed");
    } else {
        print_string("Button released");
    }

    xQueueOverwrite(xAlertQueue, &alert);
}

/**
 * @brief Check for a change in the alert state. If state changes to true,
 *        reset angle to 0. If state changes to false, turn off buzzer.
 * 
 * @param pAlert pointer to alert state
 * @param pX pointer to sine angle
*/
static void process_alert_state_change(bool *pAlert, int *pX) {
    if (xQueueReceive(xAlertQueue, pAlert, 0) == pdTRUE) {
        if (*pAlert) {
            *pX = 0;
        } else {
            ledc_write_tone(0);
        }
    }
}

/**
 * @brief Send a sine tone to the buzzer.
 * 
 * @param x angle in degrees [0, 359]
*/
static void write_sine_tone(int x) {
    float sine = sinf(x * DEG_TO_RAD);
    uint32_t freq = 2000 + (uint32_t)(sine * 500);
    ledc_write_tone(freq);
}

/**
 * @brief Task: Check whether the alert is active, then send a sine tone
 *        to the buzzer if so.
*/
static void vTaskAlert(void *pvParameters) {
    bool alert = false;

    for (;;) {
        for (int x = 0; x < 360; x += 10) {
            process_alert_state_change(&alert, &x);
            if (alert) write_sine_tone(x);
            vTaskDelay(pdMS_TO_TICKS(10));
        }
    }
}

/**
 * @brief ESP-IDF FreeRTOS application entry point.
*/
void app_main(void)
{
    xDebounceTimer = xTimerCreate(
        "Debounce Timer",
        DEBOUNCE_PERIOD,
        pdFALSE,
        NULL,
        handle_button_edge
    );

    xAlertQueue = xQueueCreate(1, sizeof(bool));

    configure_gpio();
    configure_pwm_timer();
    configure_pwm_channel();

    // Startup tone
    ledc_write_tone(2000);
    vTaskDelay(pdMS_TO_TICKS(300));
    ledc_write_tone(0);
    
    xTaskCreate(vTaskAlert, "Alert tone", 4096, NULL, 1, NULL);

    gpio_install_isr_service(0);
    gpio_isr_handler_add(BUTTON, button_isr_handler, NULL);
}
