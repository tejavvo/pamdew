#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "driver/i2c.h"
#include "driver/gpio.h"
#include "esp_adc/adc_oneshot.h"
#include "ssd1306.h"

/* ===== Sensor I2C (I2C_NUM_0) ===== */
#define I2C_PORT        I2C_NUM_0
#define SDA_PIN         21
#define SCL_PIN         22
#define I2C_FREQ        400000
#define MAX30102_ADDR   0x57
#define MPU6050_ADDR    0x68

/* ===== I2C interrupt pins ===== */
#define MAX_INT_PIN     GPIO_NUM_4
#define MPU_INT_PIN     GPIO_NUM_5

/* ===== Analog sensor pins ===== */
#define ECG_CHAN        ADC_CHANNEL_6   /* GPIO34 */
#define GSR_CHAN        ADC_CHANNEL_7   /* GPIO35 */
#define LM35_CHAN       ADC_CHANNEL_0   /* GPIO36 (VP) */
#define AD8232_LO_POS   GPIO_NUM_32
#define AD8232_LO_NEG   GPIO_NUM_33

/* ===== Shared sensor state ===== */
typedef struct {
    uint32_t red, ir;          /* MAX30102 */
    int16_t  ax, ay, az;       /* MPU6050  */
    int      ecg_raw;          /* AD8232   */
    uint8_t  ecg_leads_ok;     /* 1 = connected */
    int      gsr_raw;          /* GSR      */
    float    temp_c;           /* LM35     */
} sensor_data_t;

static sensor_data_t     g_sensor    = {0};
static SemaphoreHandle_t i2c_mutex;
static SemaphoreHandle_t sensor_mutex;
static TaskHandle_t      max_task_handle;
static TaskHandle_t      mpu_task_handle;
static adc_oneshot_unit_handle_t adc1_handle;

/* ===== I2C helpers ===== */
static void sensor_i2c_init(void)
{
    i2c_config_t c = {
        .mode          = I2C_MODE_MASTER,
        .sda_io_num    = SDA_PIN,
        .scl_io_num    = SCL_PIN,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
    };
    c.master.clk_speed = I2C_FREQ;
    i2c_param_config(I2C_PORT, &c);
    i2c_driver_install(I2C_PORT, I2C_MODE_MASTER, 0, 0, 0);
}

static esp_err_t i2c_wr(uint8_t dev, uint8_t reg, uint8_t val)
{
    uint8_t buf[2] = {reg, val};
    return i2c_master_write_to_device(I2C_PORT, dev, buf, 2, pdMS_TO_TICKS(10));
}

static esp_err_t i2c_rd(uint8_t dev, uint8_t reg, uint8_t *out, size_t len)
{
    return i2c_master_write_read_device(I2C_PORT, dev, &reg, 1, out, len, pdMS_TO_TICKS(10));
}

/* ===== Sensor init ===== */
static void max30102_init(void)
{
    i2c_wr(MAX30102_ADDR, 0x09, 0x03); /* SpO2 mode      */
    i2c_wr(MAX30102_ADDR, 0x0A, 0x27); /* ~100 Hz        */
    i2c_wr(MAX30102_ADDR, 0x02, 0xC0); /* enable INT     */
}

static void mpu6050_init(void)
{
    i2c_wr(MPU6050_ADDR, 0x6B, 0x00);  /* wake up        */
    i2c_wr(MPU6050_ADDR, 0x38, 0x01);  /* data-ready INT */
}

static void adc_sensors_init(void)
{
    adc_oneshot_unit_init_cfg_t unit_cfg = {
        .unit_id  = ADC_UNIT_1,
        .ulp_mode = ADC_ULP_MODE_DISABLE,
    };
    adc_oneshot_new_unit(&unit_cfg, &adc1_handle);

    adc_oneshot_chan_cfg_t ch = {
        .atten    = ADC_ATTEN_DB_12,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    adc_oneshot_config_channel(adc1_handle, ECG_CHAN,  &ch);
    adc_oneshot_config_channel(adc1_handle, GSR_CHAN,  &ch);
    adc_oneshot_config_channel(adc1_handle, LM35_CHAN, &ch);

    /* AD8232 LO+/LO- as plain digital inputs */
    gpio_config_t lo = {
        .pin_bit_mask = (1ULL << AD8232_LO_POS) | (1ULL << AD8232_LO_NEG),
        .mode         = GPIO_MODE_INPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    gpio_config(&lo);
}

/* ===== Read helpers ===== */
static void read_max30102(uint32_t *r, uint32_t *ir)
{
    uint8_t buf[6];
    i2c_rd(MAX30102_ADDR, 0x07, buf, 6);
    *r  = ((buf[0]<<16)|(buf[1]<<8)|buf[2]) & 0x3FFFF;
    *ir = ((buf[3]<<16)|(buf[4]<<8)|buf[5]) & 0x3FFFF;
    printf("MAX RED:%lu IR:%lu\n", (unsigned long)*r, (unsigned long)*ir);
}

static void read_mpu6050(int16_t *ax, int16_t *ay, int16_t *az)
{
    uint8_t buf[6];
    i2c_rd(MPU6050_ADDR, 0x3B, buf, 6);
    *ax = (int16_t)((buf[0]<<8)|buf[1]);
    *ay = (int16_t)((buf[2]<<8)|buf[3]);
    *az = (int16_t)((buf[4]<<8)|buf[5]);
    printf("MPU AX:%d AY:%d AZ:%d\n", *ax, *ay, *az);
}

static void read_analog(void)
{
    int ecg = 0, gsr = 0, lm35 = 0;
    adc_oneshot_read(adc1_handle, ECG_CHAN,  &ecg);
    adc_oneshot_read(adc1_handle, GSR_CHAN,  &gsr);
    adc_oneshot_read(adc1_handle, LM35_CHAN, &lm35);

    uint8_t leads_ok = (!gpio_get_level(AD8232_LO_POS) &&
                        !gpio_get_level(AD8232_LO_NEG));

    /* LM35: 10 mV/°C; ADC range 0-3100 mV at 12 dB atten */
    float temp = (lm35 * 3100.0f / 4095.0f) / 10.0f;

    printf("ECG:%d GSR:%d TEMP:%.1f LEADS:%s\n",
           ecg, gsr, temp, leads_ok ? "OK" : "OFF");

    xSemaphoreTake(sensor_mutex, portMAX_DELAY);
    g_sensor.ecg_raw      = ecg;
    g_sensor.ecg_leads_ok = leads_ok;
    g_sensor.gsr_raw      = gsr;
    g_sensor.temp_c       = temp;
    xSemaphoreGive(sensor_mutex);
}

/* ===== ISRs ===== */
static void IRAM_ATTR max_isr(void *arg)
{
    BaseType_t hp = pdFALSE;
    vTaskNotifyGiveFromISR(max_task_handle, &hp);
    if (hp) portYIELD_FROM_ISR();
}

static void IRAM_ATTR mpu_isr(void *arg)
{
    BaseType_t hp = pdFALSE;
    vTaskNotifyGiveFromISR(mpu_task_handle, &hp);
    if (hp) portYIELD_FROM_ISR();
}

/* ===== Tasks ===== */
void max_task(void *arg)
{
    while (1) {
        ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(1000));
        uint32_t r, ir;
        xSemaphoreTake(i2c_mutex, portMAX_DELAY);
        read_max30102(&r, &ir);
        xSemaphoreGive(i2c_mutex);

        xSemaphoreTake(sensor_mutex, portMAX_DELAY);
        g_sensor.red = r;
        g_sensor.ir  = ir;
        xSemaphoreGive(sensor_mutex);
    }
}

void mpu_task(void *arg)
{
    while (1) {
        ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(1000));
        int16_t ax, ay, az;
        xSemaphoreTake(i2c_mutex, portMAX_DELAY);
        read_mpu6050(&ax, &ay, &az);
        xSemaphoreGive(i2c_mutex);

        xSemaphoreTake(sensor_mutex, portMAX_DELAY);
        g_sensor.ax = ax;
        g_sensor.ay = ay;
        g_sensor.az = az;
        xSemaphoreGive(sensor_mutex);
    }
}

/* Analog sensors polled at 10 Hz — no interrupt needed */
void analog_task(void *arg)
{
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(100));
        read_analog();
    }
}

/* ===== OLED task ===== */
/*
 * 128x64 display, 8 pages, ~21 chars/line at 6 px/char
 *
 * [0] "***SENSOR HUB***"
 * [1] "R:XXXXXX I:XXXXXX"    <- MAX30102
 * [2] "AX:XXXXX AY:XXXXX"   <- MPU accel XY
 * [3] "AZ:XXXXX"             <- MPU accel Z
 * [4] "ECG:XXXX [OK]/[LO]"  <- AD8232
 * [5] "GSR: XXXXX"           <- GSR raw
 * [6] "TEMP: XX.X C"         <- LM35
 * [7] (blank)
 */
void oled_task(void *arg)
{
    ssd1306_init();
    ssd1306_clear();
    ssd1306_printf_at(0, 0, "***SENSOR HUB***");

    char buf[22];
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(200)); /* 5 Hz refresh */

        xSemaphoreTake(sensor_mutex, portMAX_DELAY);
        sensor_data_t d = g_sensor;
        xSemaphoreGive(sensor_mutex);

        /* MAX30102 */
        snprintf(buf, sizeof(buf), "R:%-6lu I:%-6lu",
                 (unsigned long)d.red, (unsigned long)d.ir);
        ssd1306_printf_at(0, 1, buf);

        /* MPU6050 */
        snprintf(buf, sizeof(buf), "AX:%-5d AY:%-5d", d.ax, d.ay);
        ssd1306_printf_at(0, 2, buf);
        snprintf(buf, sizeof(buf), "AZ:%-5d", d.az);
        ssd1306_printf_at(0, 3, buf);

        /* AD8232 ECG */
        snprintf(buf, sizeof(buf), "ECG:%-4d [%s]",
                 d.ecg_raw, d.ecg_leads_ok ? "OK " : "LO!");
        ssd1306_printf_at(0, 4, buf);

        /* GSR */
        snprintf(buf, sizeof(buf), "GSR: %-5d", d.gsr_raw);
        ssd1306_printf_at(0, 5, buf);

        /* LM35 temperature */
        snprintf(buf, sizeof(buf), "TEMP: %5.1f C", d.temp_c);
        ssd1306_printf_at(0, 6, buf);
    }
}

/* ===== GPIO interrupts ===== */
static void gpio_init_interrupts(void)
{
    gpio_config_t io = {
        .pin_bit_mask = (1ULL << MAX_INT_PIN) | (1ULL << MPU_INT_PIN),
        .mode         = GPIO_MODE_INPUT,
        .pull_up_en   = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_NEGEDGE,
    };
    gpio_config(&io);
    gpio_install_isr_service(0);
    gpio_isr_handler_add(MAX_INT_PIN, max_isr, NULL);
    gpio_isr_handler_add(MPU_INT_PIN, mpu_isr, NULL);
}

/* ===== Entry point ===== */
void app_main(void)
{
    printf("app_main: boot\n");

    sensor_i2c_init();
    adc_sensors_init();

    i2c_mutex    = xSemaphoreCreateMutex();
    sensor_mutex = xSemaphoreCreateMutex();

    max30102_init();
    mpu6050_init();

    xTaskCreate(oled_task,   "oled",   4096, NULL, 2, NULL);
    xTaskCreate(max_task,    "max",    4096, NULL, 3, &max_task_handle);
    xTaskCreate(mpu_task,    "mpu",    4096, NULL, 2, &mpu_task_handle);
    xTaskCreate(analog_task, "analog", 4096, NULL, 2, NULL);

    gpio_init_interrupts();
    printf("app_main: ready\n");
}
