/**
 * Source file for LCD control over I2C.
 */

/**
  PCF8574 to LDC1602 connections:
  P0 -> RS
  P1 -> R/W
  P2 -> E
  P3 -> A
  P4 -> DB4
  P5 -> DB5
  P6 -> DB6
  P7 -> DB7

  I2C Data Bits:
  [ P7,  P6,  P5,  P4, P3, P2,  P1, P0]
  [DB7, DB6, DB5, DB4,  A,  E, R/W, RS]
*/

#include "freertos/FreeRTOS.h"
#include "lcd.h"

#define DISPLAY_CLEAR   0x01

#define ENTRY_MODE_SET  0x04
#define INCREMENT       0x02
#define DECREMENT       0x00
#define SHIFT_ON        0x01
#define SHIFT_OFF       0x00

#define DISPLAY_CONTROL 0x08
#define DISPLAY_ON      0x04
#define DISPLAY_OFF     0x00
#define CURSOR_ON       0x02
#define CURSOR_OFF      0x00
#define BLINKS_ON       0x01
#define BLINKS_OFF      0x00

#define FUNCTION_SET    0x20
#define LENGTH_8_BIT    0x10
#define LENGTH_4_BIT    0x00
#define DISPLAY_2_LINE  0x08
#define DISPLAY_1_LINE  0x00
#define FONT_5x10_DOTS  0x04
#define FONT_5x8_DOTS   0x00

#define DD_RAM_ADDR     0x80

#define BACKLIGHT_ON    0x08
#define BACKLIGHT_OFF   0x00

#define ENABLE          0x04
#define RW              0x02
#define RS              0x01

#define I2C_TIMEOUT     1000

static void send_command(uint8_t data);
static void write_4_bits(uint8_t data, uint8_t mode);
static void pulse_enable(uint8_t data, uint8_t mode);
static void i2c_write(uint8_t data, uint8_t mode);

static lcd_config_t conf;
static uint8_t backlight = BACKLIGHT_ON;

void lcd_configure(lcd_config_t lcd_config) {
    conf = lcd_config;
}

void lcd_init() {
    // From TINSHARP TC1602B-01 datasheet, pg. 16

    // Wait time >40 ms
    vTaskDelay(pdMS_TO_TICKS(50));

    // Function set (8 bits)
    // Wait time >4.1 ms
    write_4_bits(0x03, 0);

    // Function set (8 bits)
    // Wait time >0.1 ms
    write_4_bits(0x03, 0);

    // Function set (8 bits)
    write_4_bits(0x03, 0);

    // Function set (4 bits)
    write_4_bits(0x02, 0);

    // Function set (number of display lines and font)
    send_command(FUNCTION_SET | DISPLAY_2_LINE | FONT_5x8_DOTS);

    // Display control
    send_command(DISPLAY_CONTROL | DISPLAY_ON | CURSOR_OFF | BLINKS_OFF);

    // Display clear
    send_command(DISPLAY_CLEAR);

    // Entry mode set
    send_command(ENTRY_MODE_SET | INCREMENT | SHIFT_OFF);
}

void lcd_set_cursor(uint8_t row, uint8_t col) {
    assert(row < 2);
    assert(col < 16);

    send_command(DD_RAM_ADDR | row * 0x40 | col);
}

void lcd_write_char(char c) {
    //printf("%8u: Writing char %c (%#04x).\n", xTaskGetTickCount(), c, c);

    write_4_bits((uint8_t) c >> 4, RS);
    write_4_bits((uint8_t) c & 0x0f, RS);
}

void lcd_write_string(char *s) {
    while (*s != '\0') {
        lcd_write_char(*s);
        s++;
    }
}

static void send_command(uint8_t data) {
    //printf("%8u: Writing command %#04x.\n", xTaskGetTickCount(), data);

    write_4_bits(data >> 4, 0);
    write_4_bits(data & 0x0f, 0);
}

static void write_4_bits(uint8_t data, uint8_t mode) {
    i2c_write(data, mode);
    vTaskDelay(pdMS_TO_TICKS(10));
    pulse_enable(data, mode);
}

static void pulse_enable(uint8_t data, uint8_t mode) {
    i2c_write(data, ENABLE | mode);
    vTaskDelay(pdMS_TO_TICKS(10));
    i2c_write(data, mode);
    vTaskDelay(pdMS_TO_TICKS(10));
}

static void i2c_write(uint8_t data, uint8_t mode) {
    uint8_t message = (data << 4) | backlight | mode;
    //printf("%#04x\n", message);

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (conf.address << 1) | I2C_MASTER_WRITE, I2C_MASTER_ACK);
    i2c_master_write_byte(cmd, message, I2C_MASTER_ACK);
    i2c_master_stop(cmd);

    i2c_master_cmd_begin(conf.i2c_port, cmd, pdMS_TO_TICKS(I2C_TIMEOUT));
    i2c_cmd_link_delete(cmd);
}