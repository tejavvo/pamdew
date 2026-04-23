/**
 * @file max30102_cal.c
 * @brief MAX30102 SpO2/Heart-Rate sensor calibration.
 *
 * Strategy
 * --------
 * With no finger on the sensor, the raw FIFO values represent the ambient
 * DC level (LED leakage + photodiode dark current).  We collect a short
 * burst of samples, average them, and store the result as the DC baseline.
 * Live tasks subtract this baseline before any SpO2 / HR algorithm so
 * they only work on the AC component.
 *
 * A finger-detection threshold is also derived: ir_baseline + OFFSET.
 * When a live IR reading exceeds this threshold we know a finger is present.
 */

#include "calibration.h"

#include <stdio.h>
#include <stdint.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "driver/i2c.h"

/* ── private constants ──────────────────────────────────────────────── */
#define MAX30102_ADDR        0x57
#define REG_FIFO_DATA        0x07   /* FIFO data output register          */
#define REG_FIFO_WR_PTR      0x04   /* write-pointer (used to flush FIFO) */
#define REG_FIFO_RD_PTR      0x06   /* read-pointer                       */
#define REG_OVF_COUNTER      0x05
#define I2C_PORT             I2C_NUM_0
#define SAMPLE_PERIOD_MS     10     /* sensor runs at 100 sps → 10 ms/sample */
#define FINGER_THRESH_OFFSET 50000U /* IR above baseline+this → finger on  */

/* ── module-local state ─────────────────────────────────────────────── */
static max30102_cal_t s_cal = {0};

/* ── private helpers ────────────────────────────────────────────────── */

static esp_err_t i2c_wr(uint8_t reg, uint8_t val)
{
    uint8_t buf[2] = {reg, val};
    return i2c_master_write_to_device(I2C_PORT, MAX30102_ADDR,
                                      buf, 2, pdMS_TO_TICKS(10));
}

static esp_err_t i2c_rd(uint8_t reg, uint8_t *out, size_t len)
{
    return i2c_master_write_read_device(I2C_PORT, MAX30102_ADDR,
                                        &reg, 1, out, len, pdMS_TO_TICKS(10));
}

/** Flush FIFO by resetting read and write pointers. */
static void flush_fifo(void)
{
    i2c_wr(REG_FIFO_WR_PTR,  0x00);
    i2c_wr(REG_OVF_COUNTER,  0x00);
    i2c_wr(REG_FIFO_RD_PTR,  0x00);
}

/** Read one 6-byte FIFO entry → red (18-bit) and ir (18-bit). */
static void read_sample(SemaphoreHandle_t mtx, uint32_t *red, uint32_t *ir)
{
    uint8_t buf[6] = {0};
    xSemaphoreTake(mtx, portMAX_DELAY);
    i2c_rd(REG_FIFO_DATA, buf, sizeof(buf));
    xSemaphoreGive(mtx);

    *red = ((uint32_t)buf[0] << 16 | (uint32_t)buf[1] << 8 | buf[2]) & 0x3FFFF;
    *ir  = ((uint32_t)buf[3] << 16 | (uint32_t)buf[4] << 8 | buf[5]) & 0x3FFFF;
}

/* ── public API ─────────────────────────────────────────────────────── */

void max30102_calibrate(SemaphoreHandle_t i2c_mutex, uint32_t samples)
{
    printf("[CAL] MAX30102: starting calibration (%lu samples, no finger)\n",
           (unsigned long)samples);

    /* Flush stale FIFO data before collecting baseline samples. */
    xSemaphoreTake(i2c_mutex, portMAX_DELAY);
    flush_fifo();
    xSemaphoreGive(i2c_mutex);

    uint64_t sum_red = 0, sum_ir = 0;

    for (uint32_t i = 0; i < samples; i++) {
        uint32_t r = 0, ir = 0;
        read_sample(i2c_mutex, &r, &ir);
        sum_red += r;
        sum_ir  += ir;
        vTaskDelay(pdMS_TO_TICKS(SAMPLE_PERIOD_MS));
        /* Print a dot every 10 samples so the user sees progress */
        if ((i + 1) % 10 == 0) {
            printf("[CAL] MAX30102: %lu/%lu samples done\n",
                   (unsigned long)(i + 1), (unsigned long)samples);
        }
    }

    s_cal.dc_red       = (uint32_t)(sum_red / samples);
    s_cal.dc_ir        = (uint32_t)(sum_ir  / samples);
    s_cal.finger_thresh = s_cal.dc_ir + FINGER_THRESH_OFFSET;

    printf("[CAL] MAX30102: dc_red=%lu  dc_ir=%lu  finger_thresh=%lu\n",
           (unsigned long)s_cal.dc_red,
           (unsigned long)s_cal.dc_ir,
           (unsigned long)s_cal.finger_thresh);
}

max30102_cal_t max30102_cal_get(void)
{
    return s_cal;
}
