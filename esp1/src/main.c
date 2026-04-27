#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include "esp_err.h"
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "driver/i2c.h"
#include "driver/gpio.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_timer.h"
#include "ssd1306.h"
#include "calibration.h"
#include "pan_tompkins.h"
#include "wifi_tx.h"
#include "ml_model.h"

#ifndef CONFIG_SENSOR_HUB_WIFI_SSID
#define CONFIG_SENSOR_HUB_WIFI_SSID ""
#endif

#ifndef CONFIG_SENSOR_HUB_WIFI_PASSWORD
#define CONFIG_SENSOR_HUB_WIFI_PASSWORD ""
#endif

#ifndef CONFIG_SENSOR_HUB_LOG_REPORT_JSON
#define CONFIG_SENSOR_HUB_LOG_REPORT_JSON 0
#endif

#ifndef CONFIG_SENSOR_HUB_ML_ENABLE
#define CONFIG_SENSOR_HUB_ML_ENABLE 1
#endif

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

/* ===== PPG / SpO2 vitals (MAX30102) — 4 s @ 100 Hz window ===== */
#define BUFFER_LEN      400
#define FILTER_SIZE     5

/* ===== Shared state ===== */
typedef struct {
    uint32_t red, ir;       /* MAX30102 – 100 Hz */
    float    bpm;           /* PPG-derived BPM (custom math) */
    float    spo2;          /* PPG-derived SpO2 */
    float    ir_ac;         /* IR AC component (signal quality) */
    uint8_t  ppg_valid;
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

/* MAX30102 buffer + PPG state */
static uint32_t          red_buffer[BUFFER_LEN];
static uint32_t          ir_buffer[BUFFER_LEN];
static int               buffer_index  = 0;
static float             smoothed_bpm  = 0.0f;
static uint32_t          max_i2c_errors = 0;
static uint32_t          adc_errors     = 0;
static bool              wifi_ready     = false;

/* ===== I2C helpers ===== */
static void sensor_i2c_init(void)
{
    printf("[INIT] I2C: configuring SDA=%d SCL=%d @ %d Hz...\n",
           SDA_PIN, SCL_PIN, I2C_FREQ);
    i2c_config_t c = {
        .mode          = I2C_MODE_MASTER,
        .sda_io_num    = SDA_PIN,
        .scl_io_num    = SCL_PIN,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
    };
    c.master.clk_speed = I2C_FREQ;
    ESP_ERROR_CHECK(i2c_param_config(I2C_PORT, &c));
    esp_err_t err = i2c_driver_install(I2C_PORT, I2C_MODE_MASTER, 0, 0, 0);
    printf("[INIT] I2C: driver install %s\n", err == ESP_OK ? "OK" : "FAILED");
    ESP_ERROR_CHECK(err);
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
    printf("[INIT] MAX30102: reset + SpO2 mode, 100 sps, 411us pulse, LED current...\n");
    esp_err_t e0 = i2c_wr(MAX30102_ADDR, 0x09, 0x40); /* full reset (MODALITY)     */
    vTaskDelay(pdMS_TO_TICKS(100));
    esp_err_t e1 = i2c_wr(MAX30102_ADDR, 0x09, 0x03); /* SpO2 mode                 */
    esp_err_t e2 = i2c_wr(MAX30102_ADDR, 0x0A, 0x27); /* 100 sps, 411 µs, 4096 ADC */
    /* INT: PPG_RDY (bit6) – fires every new sample at 100 Hz */
    esp_err_t e3 = i2c_wr(MAX30102_ADDR, 0x02, 0x40);
    esp_err_t e4 = i2c_wr(MAX30102_ADDR, 0x0C, 0x24); /* LED1 (red) current        */
    esp_err_t e5 = i2c_wr(MAX30102_ADDR, 0x0D, 0x24); /* LED2 (IR) current         */
    bool ok = (e0 == ESP_OK && e1 == ESP_OK && e2 == ESP_OK && e3 == ESP_OK &&
               e4 == ESP_OK && e5 == ESP_OK);
    printf("[INIT] MAX30102: %s\n", ok ? "OK" : "FAILED (check wiring)");
    ESP_ERROR_CHECK(e0);
    ESP_ERROR_CHECK(e1);
    ESP_ERROR_CHECK(e2);
    ESP_ERROR_CHECK(e3);
    ESP_ERROR_CHECK(e4);
    ESP_ERROR_CHECK(e5);
}

static void mpu6050_init(void)
{
    printf("[INIT] MPU6050: waking up, DLPF=1kHz, 50 Hz output rate...\n");
    esp_err_t e1 = i2c_wr(MPU6050_ADDR, 0x6B, 0x00); /* wake up                            */
    esp_err_t e2 = i2c_wr(MPU6050_ADDR, 0x1A, 0x01); /* DLPF_CFG=1 → 1 kHz internal rate  */
    esp_err_t e3 = i2c_wr(MPU6050_ADDR, 0x19, 19);   /* SMPLRT_DIV=19 → 1000/(1+19)=50 Hz */
    esp_err_t e4 = i2c_wr(MPU6050_ADDR, 0x38, 0x01); /* DATA_RDY interrupt enable           */
    printf("[INIT] MPU6050: %s\n",
           (e1 == ESP_OK && e2 == ESP_OK && e3 == ESP_OK && e4 == ESP_OK)
           ? "OK" : "FAILED (check wiring)");
    ESP_ERROR_CHECK(e1);
    ESP_ERROR_CHECK(e2);
    ESP_ERROR_CHECK(e3);
    ESP_ERROR_CHECK(e4);
}

static void adc_sensors_init(void)
{
    printf("[INIT] ADC: initialising ADC1 unit (12 dB atten, 12-bit)...\n");
    adc_oneshot_unit_init_cfg_t unit_cfg = {
        .unit_id  = ADC_UNIT_1,
        .ulp_mode = ADC_ULP_MODE_DISABLE,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&unit_cfg, &adc1_handle));

    adc_oneshot_chan_cfg_t ch = {
        .atten    = ADC_ATTEN_DB_12,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, ECG_CHAN,  &ch));
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, GSR_CHAN,  &ch));
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, LM35_CHAN, &ch));
    printf("[INIT] ADC: ECG ch=%d  GSR ch=%d  LM35 ch=%d  configured\n",
           ECG_CHAN, GSR_CHAN, LM35_CHAN);

    printf("[INIT] ADC: configuring AD8232 LO+/- GPIO pins (%d, %d)...\n",
           AD8232_LO_POS, AD8232_LO_NEG);
    gpio_config_t lo = {
        .pin_bit_mask = (1ULL << AD8232_LO_POS) | (1ULL << AD8232_LO_NEG),
        .mode         = GPIO_MODE_INPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&lo));
    printf("[INIT] ADC: AD8232 LO pins ready\n");
}

static void invalidate_ppg_vitals(void)
{
    xSemaphoreTake(sensor_mutex, portMAX_DELAY);
    g_sensor.bpm       = 0.0f;
    g_sensor.spo2      = 0.0f;
    g_sensor.ir_ac     = 0.0f;
    g_sensor.ppg_valid = 0;
    xSemaphoreGive(sensor_mutex);
}

/* Derive SpO2 / BPM from 4 s of stored MAX30102 data */
static void process_max30102_vitals(void)
{
    float ir_avg  = 0.0f;
    float red_avg = 0.0f;
    float ir_min  = 1000000.0f;
    float ir_max  = 0.0f;
    float red_min = 1000000.0f;
    float red_max = 0.0f;
    static float filtered_ir[BUFFER_LEN];

    for (int i = 0; i < BUFFER_LEN; i++) {
        ir_avg += (float)ir_buffer[i];
        red_avg += (float)red_buffer[i];
        if (ir_buffer[i] < ir_min) {
            ir_min = (float)ir_buffer[i];
        }
        if (ir_buffer[i] > ir_max) {
            ir_max = (float)ir_buffer[i];
        }
        if (red_buffer[i] < red_min) {
            red_min = (float)red_buffer[i];
        }
        if (red_buffer[i] > red_max) {
            red_max = (float)red_buffer[i];
        }

        if (i >= FILTER_SIZE) {
            float sum = 0.0f;
            for (int j = 0; j < FILTER_SIZE; j++) {
                sum += (float)ir_buffer[i - j];
            }
            filtered_ir[i] = sum / (float)FILTER_SIZE;
        }
    }
    ir_avg /= (float)BUFFER_LEN;
    red_avg /= (float)BUFFER_LEN;
    float ir_ac  = ir_max - ir_min;
    float red_ac = red_max - red_min;

    int  peaks         = 0;
    bool is_above     = false;
    float center_line = ir_min + (ir_ac / 2.0f);

    for (int i = FILTER_SIZE + 1; i < BUFFER_LEN; i++) {
        if (filtered_ir[i] > center_line && !is_above) {
            peaks++;
            is_above = true;
        } else if (filtered_ir[i] < center_line) {
            is_above = false;
        }
    }

    if (ir_ac < 1.0e-6f || red_ac < 1.0e-6f || ir_avg < 1.0e-6f || red_avg < 1.0e-6f) {
        invalidate_ppg_vitals();
        return;
    }

    float R            = (red_ac / red_avg) / (ir_ac / ir_avg);
    if (R < 0.2f || R > 2.0f) {
        invalidate_ppg_vitals();
        return;
    }

    float current_spo2 = 10.0002f * (R * R) - 52.887f * R + 118.58f;
    int   raw_bpm      = peaks * 15; /* 4 s window → scale to 1 min */
    if (raw_bpm < 30 || raw_bpm > 220) {
        invalidate_ppg_vitals();
        return;
    }

    if (smoothed_bpm == 0.0f) {
        smoothed_bpm = (float)raw_bpm;
    }
    smoothed_bpm     = (smoothed_bpm * 0.8f) + ((float)raw_bpm * 0.2f);

    if (current_spo2 < 50.0f || current_spo2 > 100.0f) {
        invalidate_ppg_vitals();
        return;
    }

    xSemaphoreTake(sensor_mutex, portMAX_DELAY);
    g_sensor.bpm   = smoothed_bpm;
    g_sensor.spo2  = current_spo2;
    g_sensor.ir_ac = ir_ac;
    g_sensor.ppg_valid = 1;
    xSemaphoreGive(sensor_mutex);
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
    printf("[INIT] GPIO: configuring INT pins – MAX30102 GPIO%d, MPU6050 GPIO%d\n",
           MAX_INT_PIN, MPU_INT_PIN);
    gpio_config_t io = {
        .pin_bit_mask = (1ULL << MAX_INT_PIN) | (1ULL << MPU_INT_PIN),
        .mode         = GPIO_MODE_INPUT,
        .pull_up_en   = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_NEGEDGE,
    };
    ESP_ERROR_CHECK(gpio_config(&io));
    ESP_ERROR_CHECK(gpio_install_isr_service(0));
    ESP_ERROR_CHECK(gpio_isr_handler_add(MAX_INT_PIN, max_isr, NULL));
    ESP_ERROR_CHECK(gpio_isr_handler_add(MPU_INT_PIN, mpu_isr, NULL));
    printf("[INIT] GPIO: ISR handlers registered\n");
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
    printf("[TASK] max_task: started (100 Hz, INT-driven)\n");
    uint32_t log_ctr = 0;
    while (1) {
        /* 20 ms timeout = 2× the 10 ms period; safety fallback */
        ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(20));

        uint8_t buf[6];
        xSemaphoreTake(i2c_mutex, portMAX_DELAY);
        esp_err_t rd_err = i2c_rd(MAX30102_ADDR, 0x07, buf, 6);
        xSemaphoreGive(i2c_mutex);
        if (rd_err != ESP_OK) {
            max_i2c_errors++;
            invalidate_ppg_vitals();
            buffer_index = 0;
            smoothed_bpm = 0.0f;
            if (max_i2c_errors % 100 == 1) {
                printf("[MAX30102] I2C read failed (%s), total=%lu\n",
                       esp_err_to_name(rd_err), (unsigned long)max_i2c_errors);
            }
            continue;
        }

        uint32_t r  = ((buf[0]<<16)|(buf[1]<<8)|buf[2]) & 0x3FFFF;
        uint32_t ir = ((buf[3]<<16)|(buf[4]<<8)|buf[5]) & 0x3FFFF;

        max30102_cal_t max_cal = max30102_cal_get();
        uint32_t finger_thresh = (max_cal.finger_thresh > 0U) ? max_cal.finger_thresh : 5000U;
        uint32_t r_ppg = (r > max_cal.dc_red) ? (r - max_cal.dc_red) : 0U;
        uint32_t ir_ppg = (ir > max_cal.dc_ir) ? (ir - max_cal.dc_ir) : 0U;

        if (ir > finger_thresh && r_ppg > 1000U && ir_ppg > 1000U) {
            red_buffer[buffer_index]  = r_ppg;
            ir_buffer[buffer_index]   = ir_ppg;
            buffer_index++;
            if (buffer_index >= BUFFER_LEN) {
                process_max30102_vitals();
                buffer_index = 0;
            }
        } else {
            invalidate_ppg_vitals();
            buffer_index = 0;
            smoothed_bpm = 0.0f;
        }

        xSemaphoreTake(sensor_mutex, portMAX_DELAY);
        g_sensor.red = r;
        g_sensor.ir  = ir;
        xSemaphoreGive(sensor_mutex);

        /* Log once per second (100 samples × 10 ms = 1 s) */
        if (++log_ctr >= 100) {
            log_ctr = 0;
            printf("[MAX30102] RED=%-6lu  IR=%-6lu\n",
                   (unsigned long)r, (unsigned long)ir);
        }
    }
}

/* MPU6050 – woken by hardware INT at 50 Hz */
void mpu_task(void *arg)
{
    printf("[TASK] mpu_task: started (50 Hz, INT-driven)\n");
    uint32_t log_ctr = 0;
    while (1) {
        /* 40 ms timeout = 2× the 20 ms period; safety fallback */
        ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(40));

        uint8_t buf[6];
        xSemaphoreTake(i2c_mutex, portMAX_DELAY);
        esp_err_t rd_err = i2c_rd(MPU6050_ADDR, 0x3B, buf, 6);
        xSemaphoreGive(i2c_mutex);
        if (rd_err != ESP_OK) {
            printf("[MPU6050] I2C read failed: %s\n", esp_err_to_name(rd_err));
            continue;
        }

        int16_t ax = (int16_t)((buf[0]<<8)|buf[1]);
        int16_t ay = (int16_t)((buf[2]<<8)|buf[3]);
        int16_t az = (int16_t)((buf[4]<<8)|buf[5]);

        xSemaphoreTake(sensor_mutex, portMAX_DELAY);
        g_sensor.ax = ax;
        g_sensor.ay = ay;
        g_sensor.az = az;
        xSemaphoreGive(sensor_mutex);

        /* Log once per second (50 samples × 20 ms = 1 s) */
        if (++log_ctr >= 50) {
            log_ctr = 0;
            printf("[MPU6050] AX=%-6d AY=%-6d AZ=%-6d\n", ax, ay, az);
        }
    }
}

/* AD8232 ECG – woken by esp_timer at 250 Hz (every 4 ms) */
void ecg_task(void *arg)
{
    printf("[TASK] ecg_task: starting 250 Hz timer...\n");
    /* Start the 4 ms periodic timer */
    esp_timer_handle_t ecg_timer;
    esp_timer_create_args_t ta = {
        .callback = ecg_timer_cb,
        .name     = "ecg",
    };
    esp_err_t timer_err = esp_timer_create(&ta, &ecg_timer);
    if (timer_err != ESP_OK) {
        printf("[TASK] ecg_task: timer create failed: %s\n", esp_err_to_name(timer_err));
        vTaskDelete(NULL);
        return;
    }
    timer_err = esp_timer_start_periodic(ecg_timer, 4000); /* 4000 µs = 250 Hz */
    if (timer_err != ESP_OK) {
        printf("[TASK] ecg_task: timer start failed: %s\n", esp_err_to_name(timer_err));
        vTaskDelete(NULL);
        return;
    }
    printf("[TASK] ecg_task: running at 250 Hz\n");

    uint32_t log_ctr = 0;
    bool prev_leads = true;
    while (1) {
        ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(8)); /* 8 ms = 2× period */

        int raw = 0;
        esp_err_t adc_err = adc_oneshot_read(adc1_handle, ECG_CHAN, &raw);
        if (adc_err != ESP_OK) {
            adc_errors++;
            if (adc_errors % 250 == 1) {
                printf("[ECG] ADC read failed (%s), total=%lu\n",
                       esp_err_to_name(adc_err), (unsigned long)adc_errors);
            }
            continue;
        }
        uint8_t leads = (!gpio_get_level(AD8232_LO_POS) &&
                         !gpio_get_level(AD8232_LO_NEG));

        /* Process ECG through Pan-Tompkins */
        ecg_cal_t ecg_cal = ecg_cal_get();
        int centered_ecg = raw - ecg_cal.baseline;
        if (leads) {
            (void)pt_feed(centered_ecg);
        } else if (prev_leads) {
            pt_reset();
            printf("[ECG] leads off, Pan-Tompkins state reset\n");
        }
        prev_leads = leads;

        xSemaphoreTake(sensor_mutex, portMAX_DELAY);
        g_sensor.ecg_raw      = raw;
        g_sensor.ecg_leads_ok = leads;
        xSemaphoreGive(sensor_mutex);

        /* Log every 5 seconds (250 samples × 4 ms × 5 = 5000 ms) */
        if (++log_ctr >= 1250) {
            log_ctr = 0;
            printf("[ECG] raw=%-5d  leads=%s\n",
                   raw, leads ? "OK" : "OFF");
        }
    }
}

/* GSR – polled at 10 Hz */
void gsr_task(void *arg)
{
    printf("[TASK] gsr_task: started (10 Hz polled)\n");
    uint32_t log_ctr = 0;
    TickType_t last = xTaskGetTickCount();
    while (1) {
        vTaskDelayUntil(&last, pdMS_TO_TICKS(100)); /* 100 ms = 10 Hz */
        int raw = 0;
        esp_err_t adc_err = adc_oneshot_read(adc1_handle, GSR_CHAN, &raw);
        if (adc_err != ESP_OK) {
            adc_errors++;
            printf("[GSR] ADC read failed: %s\n", esp_err_to_name(adc_err));
            continue;
        }
        xSemaphoreTake(sensor_mutex, portMAX_DELAY);
        g_sensor.gsr_raw = raw;
        xSemaphoreGive(sensor_mutex);

        /* Log every 5 seconds (50 samples × 100 ms = 5 s) */
        if (++log_ctr >= 50) {
            log_ctr = 0;
            printf("[GSR]  raw=%-5d\n", raw);
        }
    }
}

/* LM35 – sampled once per minute */
void temp_task(void *arg)
{
    printf("[TASK] temp_task: started (1 sample/min)\n");
    while (1) {
        int raw = 0;
        esp_err_t adc_err = adc_oneshot_read(adc1_handle, LM35_CHAN, &raw);
        if (adc_err != ESP_OK) {
            adc_errors++;
            printf("[TEMP] ADC read failed: %s\n", esp_err_to_name(adc_err));
            vTaskDelay(pdMS_TO_TICKS(60000)); /* 1 minute */
            continue;
        }

        /*
         * Apply the LM35 calibration result.
         * adc_offset corrects for the ESP32 ADC's nonlinearity at low
         * voltages (room temp ~280 mV is right in the inaccurate zone).
         * vref_mv is the board-measured full-scale, not the ideal 3300 mV.
         */
        lm35_cal_t cal = lm35_cal_get();
        float t = ((float)(raw + cal.adc_offset) * cal.vref_mv / 4095.0f) / 10.0f;

        xSemaphoreTake(sensor_mutex, portMAX_DELAY);
        g_sensor.temp_c = t;
        xSemaphoreGive(sensor_mutex);
        printf("TEMP: %.1f C  (raw=%d offset=%d)\n", t, raw, cal.adc_offset);
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

        if (d.ppg_valid) {
            ssd1306_printf_at(0, 1, "PPG:%3.0f SpO2:%4.1f", d.bpm, d.spo2);
        } else {
            ssd1306_printf_at(0, 1, "PPG:--- SpO2:---");
        }

        snprintf(buf, sizeof(buf), "R:%-6lu I:%-6lu",
                 (unsigned long)d.red, (unsigned long)d.ir);
        ssd1306_printf_at(0, 2, buf);

        snprintf(buf, sizeof(buf), "AX:%-5d AY:%-5d", d.ax, d.ay);
        ssd1306_printf_at(0, 3, buf);
        snprintf(buf, sizeof(buf), "AZ:%-5d", d.az);
        ssd1306_printf_at(0, 4, buf);

        snprintf(buf, sizeof(buf), "ECG:%-4d [%s]",
                 d.ecg_raw, d.ecg_leads_ok ? "OK " : "LO!");
        ssd1306_printf_at(0, 5, buf);

        snprintf(buf, sizeof(buf), "G:%-5d T:%.1fC", d.gsr_raw, d.temp_c);
        ssd1306_printf_at(0, 6, buf);
    }
}

void ml_task(void *arg)
{
#if CONFIG_SENSOR_HUB_ML_ENABLE
    printf("[TASK] ml_task: started (1 Hz feature sampler)\n");
    TickType_t last = xTaskGetTickCount();
    while (1) {
        vTaskDelayUntil(&last, pdMS_TO_TICKS(1000));

        xSemaphoreTake(sensor_mutex, portMAX_DELAY);
        sensor_data_t d = g_sensor;
        xSemaphoreGive(sensor_mutex);

        mpu6050_cal_t mpu_cal = mpu6050_cal_get();
        gsr_cal_t gsr_cal = gsr_cal_get();

        ml_sample_t sample = {
            .red          = d.red,
            .ir           = d.ir,
            .ppg_bpm      = d.bpm,
            .ppg_spo2     = d.spo2,
            .ir_ac        = d.ir_ac,
            .ppg_valid    = d.ppg_valid != 0,
            .ax           = d.ax - mpu_cal.accel_offset_x,
            .ay           = d.ay - mpu_cal.accel_offset_y,
            .az           = d.az - mpu_cal.accel_offset_z,
            .ecg_raw      = d.ecg_raw,
            .ecg_leads_ok = d.ecg_leads_ok != 0,
            .gsr_raw      = d.gsr_raw - gsr_cal.baseline_raw,
            .temp_c       = d.temp_c,
        };
        ml_model_sample(&sample);
    }
#else
    printf("[TASK] ml_task: disabled by config\n");
    vTaskDelete(NULL);
#endif
}

/* ===== Calibration ===== */
/**
 * @brief Run calibration for every sensor module.
 *
 * Called once from app_main after all hardware is initialised but before
 * any sensor task is created.  Each call is delegated to its own
 * src/calibration/<module>.c so this function stays clean and each module
 * stays isolated from the others.
 *
 * Tuning knobs:
 *   MAX30102  – 50 ambient samples  (≈ 0.5 s at 100 sps)
 *   MPU6050   – 100 static samples  (≈ 2 s at 50 Hz) – keep sensor flat
 *   ECG       – 2000 ms window      (250 Hz × 2 s)   – electrodes on
 *   GSR       – 20 resting samples  (≈ 2 s at 10 Hz) – relax
 *   LM35      – 10 averaged samples; pass a known °C or 0 to skip offset
 */
static void calibrate_all_sensors(void)
{
    printf("[CAL] === Sensor calibration start ===\n");

    /* MAX30102: no finger on sensor during these samples. */
    max30102_calibrate(i2c_mutex, /*samples=*/50);

    /* MPU6050: place on flat surface, hold still. */
    mpu6050_calibrate(i2c_mutex, /*samples=*/100);

    /* AD8232 ECG: electrodes attached, subject at rest. */
    ecg_calibrate(adc1_handle,
                  /*ecg_chan=*/   ECG_CHAN,
                  /*lo_pos_gpio=*/AD8232_LO_POS,
                  /*lo_neg_gpio=*/AD8232_LO_NEG,
                  /*duration_ms=*/2000);

    /* GSR: electrodes on, subject relaxed. */
    gsr_calibrate(adc1_handle, /*gsr_chan=*/GSR_CHAN, /*samples=*/20);

    /*
     * LM35: pass your actual room temperature as ref_temp_c so the
     * calibration can compute an adc_offset to correct for the ESP32
     * ADC's nonlinearity at low voltages (room temp ≈ 280 mV output
     * from LM35, which is in the inaccurate low-voltage zone of the ADC).
     *
     * *** Change 28.0f to whatever a real thermometer reads right now. ***
     */
    lm35_calibrate(adc1_handle, /*lm35_chan=*/LM35_CHAN,
                   /*samples=*/10, /*ref_temp_c=*/28.0f);

    printf("[CAL] === Sensor calibration complete ===\n");
}

/* ===== Reporter Task ===== */
/* Every 30 seconds, grabs the PT stats and the latest sensor values,
 * formats them as a JSON string, and sends them over WiFi via POST.
 */
void reporter_task(void *arg)
{
    printf("[TASK] reporter_task: started (30s interval)\n");
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(30000));
        
        pt_stats_t pt_stats;
        pt_get_and_reset_stats(&pt_stats);

        xSemaphoreTake(sensor_mutex, portMAX_DELAY);
        sensor_data_t snap = g_sensor;
        xSemaphoreGive(sensor_mutex);

#if CONFIG_SENSOR_HUB_ML_ENABLE
        ml_result_t ml_result;
        if (ml_model_finalize_window(&pt_stats, &ml_result)) {
            ml_model_log_result(&ml_result);
        } else {
            printf("[ML] No feature samples collected for this window\n");
        }
#endif

        char ppg_json[96];
        int ppg_len;
        if (snap.ppg_valid) {
            ppg_len = snprintf(ppg_json, sizeof(ppg_json),
                               "\"ppg_valid\":1,\"ppg_bpm\":%.1f,\"ppg_spo2\":%.1f,\"ir_ac\":%.1f,",
                               snap.bpm, snap.spo2, snap.ir_ac);
        } else {
            ppg_len = snprintf(ppg_json, sizeof(ppg_json),
                               "\"ppg_valid\":0,\"ppg_bpm\":null,\"ppg_spo2\":null,\"ir_ac\":0.0,");
        }
        if (ppg_len < 0 || ppg_len >= (int)sizeof(ppg_json)) {
            printf("[REPORTER] PPG JSON overflow, skipping POST\n");
            continue;
        }

        char json[640];
        int json_len = snprintf(json, sizeof(json),
            "{\"bpm\":%.1f,\"mean_rr_ms\":%.1f,\"sdnn_ms\":%.1f,\"rmssd_ms\":%.1f,\"r_peaks\":%lu,"
            "%s"
            "\"spo2_red\":%lu,\"spo2_ir\":%lu,"
            "\"accel\":[%d,%d,%d],\"ecg_leads\":%d,\"gsr\":%d,\"temp\":%.1f}",
            pt_stats.bpm, pt_stats.mean_rr_ms, pt_stats.sdnn_ms, pt_stats.rmssd_ms, (unsigned long)pt_stats.r_peaks,
            ppg_json,
            (unsigned long)snap.red, (unsigned long)snap.ir,
            snap.ax, snap.ay, snap.az,
            snap.ecg_leads_ok, snap.gsr_raw, snap.temp_c);
        if (json_len < 0 || json_len >= (int)sizeof(json)) {
            printf("[REPORTER] JSON overflow (%d/%u), skipping POST\n",
                   json_len, (unsigned int)sizeof(json));
            continue;
        }

#if CONFIG_SENSOR_HUB_LOG_REPORT_JSON
        printf("[REPORTER] Sending JSON:\n%s\n", json);
#else
        printf("[REPORTER] Sending JSON (%d bytes)\n", json_len);
#endif

        if (!wifi_ready) {
            printf("[REPORTER] WiFi not configured/ready, skipping POST\n");
            continue;
        }
        
        bool ok = wifi_tx_post_json(json);
        if (ok) {
            printf("[REPORTER] POST successful\n");
        } else {
            printf("[REPORTER] POST failed\n");
        }
    }
}

static void create_task_checked(TaskFunction_t task_func,
                                const char *name,
                                uint32_t stack_depth,
                                void *params,
                                UBaseType_t priority,
                                TaskHandle_t *task_handle)
{
    BaseType_t ok = xTaskCreate(task_func, name, stack_depth, params, priority, task_handle);
    if (ok != pdPASS) {
        printf("[INIT] Failed to create %s (stack=%lu prio=%lu)\n",
               name, (unsigned long)stack_depth, (unsigned long)priority);
        ESP_ERROR_CHECK(ESP_ERR_NO_MEM);
    }
}

/* ===== Entry point ===== */
void app_main(void)
{
    printf("\n=== ESP32 Sensor Hub booting ===\n");

    printf("[INIT] Setting up I2C bus...\n");
    sensor_i2c_init();

    printf("[INIT] Setting up ADC channels...\n");
    adc_sensors_init();

    printf("[INIT] Creating mutexes...\n");
    i2c_mutex    = xSemaphoreCreateMutex();
    sensor_mutex = xSemaphoreCreateMutex();
    if (i2c_mutex == NULL || sensor_mutex == NULL) {
        printf("[INIT] Mutex creation failed\n");
        ESP_ERROR_CHECK(ESP_ERR_NO_MEM);
    }
    printf("[INIT] Mutexes created\n");

    printf("[INIT] Initialising sensors...\n");
    max30102_init();
    mpu6050_init();

    printf("[INIT] Initialising Pan-Tompkins...\n");
    pt_init();

#if CONFIG_SENSOR_HUB_ML_ENABLE
    printf("[INIT] Initialising TinyML feature model...\n");
    ml_model_init();
#endif

    /* 
     * Connect to WiFi before starting calibration so network traffic doesn't
     * interfere with the precise timing loops.
     * Configure SSID/password with idf.py menuconfig -> Sensor Hub Configuration.
     */
    printf("[INIT] Connecting to WiFi...\n");
    if (CONFIG_SENSOR_HUB_WIFI_SSID[0] == '\0') {
        printf("[INIT] WiFi SSID is empty; configure Sensor Hub WiFi settings. POST disabled.\n");
    } else {
        wifi_tx_init(CONFIG_SENSOR_HUB_WIFI_SSID, CONFIG_SENSOR_HUB_WIFI_PASSWORD);
        wifi_ready = true;
    }

    /* Run per-sensor calibration before starting any tasks. */
    calibrate_all_sensors();

    printf("[INIT] Creating sensor tasks...\n");
    create_task_checked(ecg_task,  "ecg",  4096, NULL, 5, &ecg_task_handle);
    printf("[INIT]   ecg_task  created (prio 5, 4096 B stack)\n");
    create_task_checked(max_task,  "max",  4096, NULL, 4, &max_task_handle);
    printf("[INIT]   max_task  created (prio 4, 4096 B stack)\n");
    create_task_checked(mpu_task,  "mpu",  4096, NULL, 4, &mpu_task_handle);
    printf("[INIT]   mpu_task  created (prio 4, 4096 B stack)\n");
    create_task_checked(reporter_task, "reporter", 4096, NULL, 3, NULL);
    printf("[INIT]   reporter_task created (prio 3, 4096 B stack)\n");
    create_task_checked(gsr_task,  "gsr",  2048, NULL, 2, NULL);
    printf("[INIT]   gsr_task  created (prio 2, 2048 B stack)\n");
    create_task_checked(temp_task, "tmp",  2048, NULL, 1, NULL);
    printf("[INIT]   temp_task created (prio 1, 2048 B stack)\n");
    create_task_checked(oled_task, "oled", 4096, NULL, 1, NULL);
    printf("[INIT]   oled_task created (prio 1, 4096 B stack)\n");
#if CONFIG_SENSOR_HUB_ML_ENABLE
    create_task_checked(ml_task, "ml", 3072, NULL, 1, NULL);
    printf("[INIT]   ml_task created (prio 1, 3072 B stack)\n");
#endif

    printf("[INIT] Enabling GPIO interrupts...\n");
    gpio_interrupts_init();

    printf("=== Sensor Hub ready – all tasks running ===\n\n");
}
