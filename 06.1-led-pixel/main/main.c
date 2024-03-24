/**
 * Project: 06.1-led-pixel
 * Description: Drive a WS2812 RGB LED array.
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

#define BRIGHTNESS 20   // [0,100]
#define COLOR_DELAY pdMS_TO_TICKS(1000)

typedef struct {
    uint8_t r;
    uint8_t g;
    uint8_t b;
} color_t;

static const color_t COLORS[] = {
    {255,   0,   0},
    {  0, 255,   0},
    {  0,   0, 255},
    {255, 255, 255},
    {  0,   0,   0}
};

static const size_t NUM_COLORS = sizeof(COLORS) / sizeof(color_t);

/**
 * @brief Print a color to the console, along with the current tick count.
 * 
 * @param color the color to print
*/
static void print_color(const color_t color) {
    printf("%8u: Color: (%d, %d, %d)\n",
        xTaskGetTickCount(), color.r, color.g, color.b);
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
 * @brief Set colors in the RMT items array for all LEDs.
 * 
 * @param items pointer to array of RMT items
 * @param color color to set the LEDs to
*/
static void set_colors(rmt_item32_t *items, color_t color) {
    // WS2812 expects colors as 24 bits in GRB order.
    for (int i = 0; i < NUM_RMT_ITEMS; i += 24) {
        set_rmt_ws2812_byte(items, i,      color.g * BRIGHTNESS / 100);
        set_rmt_ws2812_byte(items, i + 8,  color.r * BRIGHTNESS / 100);
        set_rmt_ws2812_byte(items, i + 16, color.b * BRIGHTNESS / 100);
    }
}

/**
 * @brief Task: Display red, green, blue, white, off
*/
static void vTaskDisplayColors(void *pvParameters) {
    TickType_t xPreviousWakeTime = xTaskGetTickCount();
    rmt_item32_t items[NUM_RMT_ITEMS];

    for (;;) {
        for (int i = 0; i < NUM_COLORS; i++) {
            print_color(COLORS[i]);
            set_colors(items, COLORS[i]);
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