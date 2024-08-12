/**
 * Header file for LCD control over I2C.
*/

#include "driver/i2c.h"

#define RS_IR 0
#define RS_DR 1
#define RW_WRITE 0
#define RW_READ 1

typedef struct {
    uint8_t address;
    int i2c_port;
    uint8_t num_rows;
} lcd_config_t;

/**
 * @brief Configure the LCD driver.
 * 
 * @param config the lcd configuration settings
 */
void lcd_configure(lcd_config_t config);

/**
 * @brief Perform the LCD refresh initialization.
 */
void lcd_init();

/**
 * @brief Set the LCD cursor position.
 */
void lcd_set_cursor(uint8_t row, uint8_t col);

/**
 * @brief Write a character to the display.
 */
void lcd_write_char(char c);

/**
 * @brief Write a string to the display.
 */
void lcd_write_string(char *s);