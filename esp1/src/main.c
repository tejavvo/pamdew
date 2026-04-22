#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "driver/i2c.h"
#include "driver/gpio.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_timer.h"
#include "ssd1306.h"

/* ===== I2C bus 0 (MAX30102 + MPU6050) ===== */
#define I2C_PORT        I2C_NUM_0
#define SDA_PIN         21
#define SCL_PIN         22
#define I2C_FREQ        400000
#define MAX30102_ADDR   0x57
#define MPU6050_ADDR    0x68

/* ===== Interrupt pins ===== */
#define MAX_INT_PIN     GPIO_NUM_4   /* MAX30102 INT → GPIO4  */
#define MPU_INT_PIN     GPIO_NUM_5   /* MPU6050  INT → GPIO5  */

/* ===== Analog pins ===== */
#define ECG_CHAN        ADC_CHANNEL_6  /* AD8232  OUT → GPIO34  */
#define GSR_CHAN        ADC_CHANNEL_7  /*  GSR    OUT → GPIO35  */
#define LM35_CHAN       ADC_CHANNEL_0  /*  LM35   OUT → GPIO36  */
#define AD8232_LO_POS   GPIO_NUM_32
#define AD8232_LO_NEG   GPIO_NUM_33

/* ===== Shared state ===== */
typedef struct {
    uint32_t red, ir;       /* MAX30102 – 100 Hz */
    int16_t  ax, ay, az;    /* MPU6050  –  50 Hz */
    int      ecg_raw;       /* AD8232   – 250 Hz */
    uint8_t  ecg_leads_ok;
    int      gsr_raw;       /* GSR      –  10 Hz */
    float    temp_c;        /* LM35     –  1/min */
} sensor_data_t;

static sensor_data_t     g_sensor     = {0};
static SemaphoreHandle_t i2c_mutex;
static SemaphoreHandle_t sensor_mutex;

/* Task handles needed by ISRs / timer callback */
static TaskHandle_t max_task_handle;
static TaskHandle_t mpu_task_handle;
static TaskHandle_t ecg_task_handle;

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
    i2c_wr(MAX30102_ADDR, 0x09, 0x03); /* SpO2 mode                       */
    i2c_wr(MAX30102_ADDR, 0x0A, 0x27); /* 100 sps, 411 µs pulse, 4096 ADC */
    /* INT: PPG_RDY (bit6) – fires every new sample at 100 Hz */
    i2c_wr(MAX30102_ADDR, 0x02, 0x40);
}

static void mpu6050_init(void)
{
    i2c_wr(MPU6050_ADDR, 0x6B, 0x00); /* wake up                            */
    i2c_wr(MPU6050_ADDR, 0x1A, 0x01); /* DLPF_CFG=1 → 1 kHz internal rate  */
    i2c_wr(MPU6050_ADDR, 0x19, 19);   /* SMPLRT_DIV=19 → 1000/(1+19)=50 Hz */
    i2c_wr(MPU6050_ADDR, 0x38, 0x01); /* DATA_RDY interrupt enable           */
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

    gpio_config_t lo = {
        .pin_bit_mask = (1ULL << AD8232_LO_POS) | (1ULL << AD8232_LO_NEG),
        .mode         = GPIO_MODE_INPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    gpio_config(&lo);
}

/* ===== GPIO interrupt setup ===== */
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

static void gpio_interrupts_init(void)
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

/* esp_timer callback for 250 Hz ECG – runs in esp_timer task, not real ISR */
static void ecg_timer_cb(void *arg)
{
    xTaskNotifyGive(ecg_task_handle); /* task-context safe variant */
}

/* ===== Tasks ===== */

/* MAX30102 – woken by hardware INT at 100 Hz */
void max_task(void *arg)
{
    while (1) {
        /* 20 ms timeout = 2× the 10 ms period; safety fallback */
        ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(20));

        uint8_t buf[6];
        xSemaphoreTake(i2c_mutex, portMAX_DELAY);
        i2c_rd(MAX30102_ADDR, 0x07, buf, 6);
        xSemaphoreGive(i2c_mutex);

        uint32_t r  = ((buf[0]<<16)|(buf[1]<<8)|buf[2]) & 0x3FFFF;
        uint32_t ir = ((buf[3]<<16)|(buf[4]<<8)|buf[5]) & 0x3FFFF;

        xSemaphoreTake(sensor_mutex, portMAX_DELAY);
        g_sensor.red = r;
        g_sensor.ir  = ir;
        xSemaphoreGive(sensor_mutex);
    }
}

/* MPU6050 – woken by hardware INT at 50 Hz */
void mpu_task(void *arg)
{
    while (1) {
        /* 40 ms timeout = 2× the 20 ms period; safety fallback */
        ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(40));

        uint8_t buf[6];
        xSemaphoreTake(i2c_mutex, portMAX_DELAY);
        i2c_rd(MPU6050_ADDR, 0x3B, buf, 6);
        xSemaphoreGive(i2c_mutex);

        int16_t ax = (int16_t)((buf[0]<<8)|buf[1]);
        int16_t ay = (int16_t)((buf[2]<<8)|buf[3]);
        int16_t az = (int16_t)((buf[4]<<8)|buf[5]);

        xSemaphoreTake(sensor_mutex, portMAX_DELAY);
        g_sensor.ax = ax;
        g_sensor.ay = ay;
        g_sensor.az = az;
        xSemaphoreGive(sensor_mutex);
    }
}

/* AD8232 ECG – woken by esp_timer at 250 Hz (every 4 ms) */
void ecg_task(void *arg)
{
    /* Start the 4 ms periodic timer */
    esp_timer_handle_t ecg_timer;
    esp_timer_create_args_t ta = {
        .callback = ecg_timer_cb,
        .name     = "ecg",
    };
    esp_timer_create(&ta, &ecg_timer);
    esp_timer_start_periodic(ecg_timer, 4000); /* 4000 µs = 250 Hz */

    while (1) {
        ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(8)); /* 8 ms = 2× period */

        int raw = 0;
        adc_oneshot_read(adc1_handle, ECG_CHAN, &raw);
        uint8_t leads = (!gpio_get_level(AD8232_LO_POS) &&
                         !gpio_get_level(AD8232_LO_NEG));

        xSemaphoreTake(sensor_mutex, portMAX_DELAY);
        g_sensor.ecg_raw      = raw;
        g_sensor.ecg_leads_ok = leads;
        xSemaphoreGive(sensor_mutex);
    }
}

/* GSR – polled at 10 Hz */
void gsr_task(void *arg)
{
    TickType_t last = xTaskGetTickCount();
    while (1) {
        vTaskDelayUntil(&last, pdMS_TO_TICKS(100)); /* 100 ms = 10 Hz */
        int raw = 0;
        adc_oneshot_read(adc1_handle, GSR_CHAN, &raw);
        xSemaphoreTake(sensor_mutex, portMAX_DELAY);
        g_sensor.gsr_raw = raw;
        xSemaphoreGive(sensor_mutex);
    }
}

/* LM35 – sampled once per minute */
void temp_task(void *arg)
{
    while (1) {
        int raw = 0;
        adc_oneshot_read(adc1_handle, LM35_CHAN, &raw);
        /* 10 mV/°C; ADC top ≈ 3100 mV at 12 dB attenuation */
        float t = (raw * 3100.0f / 4095.0f) / 10.0f;
        xSemaphoreTake(sensor_mutex, portMAX_DELAY);
        g_sensor.temp_c = t;
        xSemaphoreGive(sensor_mutex);
        printf("TEMP: %.1f C\n", t);
        vTaskDelay(pdMS_TO_TICKS(60000)); /* 1 minute */
    }
}

/* OLED refresh at 5 Hz – lowest priority, non-blocking snapshot */
void oled_task(void *arg)
{
    ssd1306_init();
    ssd1306_clear();
    ssd1306_printf_at(0, 0, "***SENSOR HUB***");

    char buf[22];
    TickType_t last = xTaskGetTickCount();
    while (1) {
        vTaskDelayUntil(&last, pdMS_TO_TICKS(200)); /* 200 ms = 5 Hz */

        xSemaphoreTake(sensor_mutex, portMAX_DELAY);
        sensor_data_t d = g_sensor;
        xSemaphoreGive(sensor_mutex);

        snprintf(buf, sizeof(buf), "R:%-6lu I:%-6lu",
                 (unsigned long)d.red, (unsigned long)d.ir);
        ssd1306_printf_at(0, 1, buf);

        snprintf(buf, sizeof(buf), "AX:%-5d AY:%-5d", d.ax, d.ay);
        ssd1306_printf_at(0, 2, buf);
        snprintf(buf, sizeof(buf), "AZ:%-5d", d.az);
        ssd1306_printf_at(0, 3, buf);

        snprintf(buf, sizeof(buf), "ECG:%-4d [%s]",
                 d.ecg_raw, d.ecg_leads_ok ? "OK " : "LO!");
        ssd1306_printf_at(0, 4, buf);

        snprintf(buf, sizeof(buf), "GSR: %-5d", d.gsr_raw);
        ssd1306_printf_at(0, 5, buf);

        snprintf(buf, sizeof(buf), "TEMP: %5.1f C", d.temp_c);
        ssd1306_printf_at(0, 6, buf);
    }
}

/* ===== Entry point ===== */
void app_main(void)
{
    printf("boot\n");

    sensor_i2c_init();
    adc_sensors_init();

    i2c_mutex    = xSemaphoreCreateMutex();
    sensor_mutex = xSemaphoreCreateMutex();

    max30102_init();
    mpu6050_init();

    /*  Task            stack   prio  handle             */
    xTaskCreate(ecg_task,  "ecg",  4096, NULL, 5, &ecg_task_handle);
    xTaskCreate(max_task,  "max",  4096, NULL, 4, &max_task_handle);
    xTaskCreate(mpu_task,  "mpu",  4096, NULL, 4, &mpu_task_handle);
    xTaskCreate(gsr_task,  "gsr",  2048, NULL, 2, NULL);
    xTaskCreate(temp_task, "tmp",  2048, NULL, 1, NULL);
    xTaskCreate(oled_task, "oled", 4096, NULL, 1, NULL);

    gpio_interrupts_init();
    printf("ready\n");
}
