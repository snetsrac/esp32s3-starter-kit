/**
 * Project: 06.2-rainbow-light
 * Description: Display a rainbow pattern on a WS2812 RGB LED array.
*/

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/rmt.h"

#define LED_0 GPIO_NUM_48
#define LED_MASK (1ULL << LED_0)
#define NUM_LEDS 8

#define RMT_TX_CHANNEL RMT_CHANNEL_0
#define CLK_DIV 4
#define NUM_RMT_ITEMS (NUM_LEDS * 24)

// RMT uses 80 MHz/12.5 ns APB clock, so CLK_DIV 4 gives 20 MHz/50 ns
#define T0H 8   // 0.40 us
#define T0L 17  // 0.85 us
#define T1H 16  // 0.80 us
#define T1L 9   // 0.45 us

#define COLOR_DELAY pdMS_TO_TICKS(10)

typedef struct {
    uint8_t r;
    uint8_t g;
    uint8_t b;
} color_t;

/**
 * @brief Print a string to the console, along with the current tick count.
 * 
 * @param string the string to print
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
 * @brief Configure the RMT peripheral.
*/
static void configure_rmt(void) {
    rmt_config_t rmt_conf =
        RMT_DEFAULT_CONFIG_TX(LED_0, RMT_TX_CHANNEL);
    rmt_conf.clk_div = CLK_DIV;

    rmt_config(&rmt_conf);
    rmt_driver_install(RMT_TX_CHANNEL, 0, 0);
}

/**
 * @brief Get a value along an RGB gradient.
 * 
 * @param color pointer to color struct
 * @param i position along gradient, [0, 255]
*/
static void get_gradient_color(color_t *color, uint8_t i) {
    if (i <= 85) {
        color->r = 255 - i * 3;
        color->g = i * 3;
        color->b = 0;
    } else if (i <= 170) {
        i -= 85;
        color->r = 0;
        color->g = 255 - i * 3;
        color->b = i * 3;
    } else {
        i -= 170;
        color->r = i * 3;
        color->g = 0;
        color->b = 255 - i * 3;
    }
}

/**
 * @brief Convert a color value byte to rmt_item32_t.
 * 
 * @param items pointer to array of RMT items
 * @param start_index index in the array of the start of the byte
 * @param value color sub-pixel value
*/
static void set_rmt_ws2812_byte(
    rmt_item32_t *items,
    int start_index,
    uint8_t value
) {
    assert(start_index <= NUM_RMT_ITEMS - 8);

    // WS2812 expects bits in order from most to least significant.
    for (int i = start_index; i < start_index + 8; i++) {
        bool bit = value & (1 << (start_index - i + 7));
        
        items[i].duration0 = bit ? T1H : T0H;
        items[i].level0 = 1;
        items[i].duration1 = bit ? T1L : T0L;
        items[i].level1 = 0;
    }
}

/**
 * @brief Set color in the RMT items array for a particular LED.
 * 
 * @param items pointer to array of RMT items
 * @param led_index index of the LED to be set
 * @param color color to set the LEDs to
*/
static void set_led_color(rmt_item32_t *items, int led_index, color_t color) {
    int item_index = led_index * 24;

    assert(led_index <= NUM_RMT_ITEMS - 24);

    // WS2812 expects colors as 24 bits in GRB order.
    set_rmt_ws2812_byte(items, item_index,      color.g);
    set_rmt_ws2812_byte(items, item_index + 8,  color.r);
    set_rmt_ws2812_byte(items, item_index + 16, color.b);
}

/**
 * @brief Task: Display red, green, blue, white, off
*/
static void vTaskDisplayColors(void *pvParameters) {
    TickType_t xPreviousWakeTime = xTaskGetTickCount();
    rmt_item32_t items[NUM_RMT_ITEMS];
    color_t color;

    for (;;) {
        print_string("Start new color sequence.");

        for (int i = 0; i < 256; i++) {
            for (int led = 0; led < NUM_LEDS; led++) {
                uint8_t offset = (i + led * 256 / NUM_LEDS) % 256;
                get_gradient_color(&color, offset);
                set_led_color(items, led, color);
            }

            rmt_write_items(RMT_TX_CHANNEL, items, NUM_RMT_ITEMS, true);
            vTaskDelayUntil(&xPreviousWakeTime, COLOR_DELAY);
        }
    }
}

/**
 * @brief ESP-IDF FreeRTOS application entry point.
*/
void app_main(void)
{
    configure_gpio();
    configure_rmt();

    xTaskCreate(vTaskDisplayColors, "Display RGB colors", 4096, NULL, 1, NULL);
}