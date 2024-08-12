/**
 * Project: 20.1-lcd1602
 * Description: Write messages to an LCD using I2C.
*/

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/i2c.h"
#include "lcd.h"

#define SDA GPIO_NUM_14
#define SCL GPIO_NUM_13
#define LCD_ADDRESS 0x27

#define MASTER I2C_MODE_MASTER
#define MASTER_PORT I2C_NUM_0
#define MASTER_FREQ 100000
#define SLV_RX_BUF_DISABLE 0
#define SLV_TX_BUF_DISABLE 0
#define ESP_INTR_FLAG_DEFAULT 0
#define COUNT_PERIOD 1000UL

/**
 * @brief Print a string to the console, along with the current tick count.
 * 
 * @param s the string to print
*/
static void print_string(const char *const s) {
    printf("%8u: %s\n", xTaskGetTickCount(), s);
}

/**
 * @brief Configure I2C controller settings.
*/
static void configure_i2c(void) {
    const i2c_config_t i2c_config = {
        .mode = MASTER,
        .sda_io_num = SDA,
        .sda_pullup_en = GPIO_PULLUP_DISABLE,
        .scl_io_num = SCL,
        .scl_pullup_en = GPIO_PULLUP_DISABLE,
        .master.clk_speed = MASTER_FREQ,
        .clk_flags = I2C_SCLK_SRC_FLAG_FOR_NOMAL
    };
    i2c_param_config(MASTER_PORT, &i2c_config);
    i2c_driver_install(
        MASTER_PORT,
        MASTER,
        SLV_RX_BUF_DISABLE,
        SLV_TX_BUF_DISABLE,
        ESP_INTR_FLAG_DEFAULT
    );
}

/**
 * @brief Configure LCD driver settings.
*/
static void configure_lcd(void) {
    lcd_config_t lcd_config = {
        .address = LCD_ADDRESS,
        .i2c_port = MASTER_PORT,
        .num_rows = 2
    };
    lcd_configure(lcd_config);
};

/**
 * @brief Task: Toggle LED state and print a message.
*/
static void vTaskToggleLed(void *pvParameters) {
    TickType_t xPreviousWakeTime = xTaskGetTickCount();
    uint8_t count = 0;
    char s[4];

    for (;;) {
        snprintf(s, 4, "%-3u", count);

        lcd_set_cursor(1, 7);
        lcd_write_string(s);

        count++;

        xTaskDelayUntil(&xPreviousWakeTime, pdMS_TO_TICKS(COUNT_PERIOD));
    }
}

/**
 * @brief ESP-IDF FreeRTOS application entry point.
*/
void app_main(void)
{
    configure_i2c();
    configure_lcd();
    lcd_init();

    lcd_write_string("Hello world!");
    lcd_set_cursor(1, 0);
    lcd_write_string("Count: ");

    xTaskCreate(vTaskToggleLed, "Toggle LED", 2048, NULL, 1, NULL);
}
