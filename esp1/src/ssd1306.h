#pragma once
#include <stdint.h>

/* ---- Second I2C bus (OLED only) ---- */
#define OLED_I2C_PORT   1        /* I2C_NUM_1          */
#define OLED_SDA_PIN    25
#define OLED_SCL_PIN    26
#define OLED_I2C_FREQ   400000
#define OLED_ADDR       0x3C     /* 0x3D if SA0 is HIGH */

void ssd1306_init(void);
void ssd1306_clear(void);
/* page: 0-7 (each page = 8 px row), col: 0-127 */
void ssd1306_set_cursor(uint8_t col, uint8_t page);
void ssd1306_print(const char *str);
void ssd1306_printf_at(uint8_t col, uint8_t page, const char *fmt, ...);
