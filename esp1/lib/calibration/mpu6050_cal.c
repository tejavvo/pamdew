/**
 * @file mpu6050_cal.c
 * @brief MPU6050 accelerometer + gyroscope calibration.
 *
 * Strategy
 * --------
 * With the sensor on a flat, stationary surface, both the accelerometer
 * and the gyroscope should produce constant values (gravity on Z only
 * for accel; zero for gyro).  We collect a burst of samples at the
 * sensor's configured 50 Hz output rate, average them, and store the
 * mean as the offset.  Downstream tasks subtract these offsets from
 * every live reading.
 *
 * Note: we intentionally do NOT correct for the expected +16384 LSB on
 * the Z-accelerometer axis here – the offset captures the full raw value
 * so that the usage site can subtract it and obtain a true-zero aligned
 * coordinate frame.
 */

#include "calibration.h"

#include <stdio.h>
#include <stdint.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "driver/i2c.h"

/* ── private constants ──────────────────────────────────────────────── */
#define MPU6050_ADDR       0x68
#define REG_ACCEL_XOUT_H   0x3B   /* first of 14 consecutive data regs   */
#define REG_GYRO_XOUT_H    0x43   /* first of 6 gyro regs                */
#define I2C_PORT           I2C_NUM_0
#define SAMPLE_PERIOD_MS   20     /* sensor runs at 50 Hz → 20 ms/sample */

/* ── module-local state ─────────────────────────────────────────────── */
static mpu6050_cal_t s_cal = {0};

/* ── private helpers ────────────────────────────────────────────────── */

static esp_err_t i2c_rd(uint8_t reg, uint8_t *out, size_t len)
{
    return i2c_master_write_read_device(I2C_PORT, MPU6050_ADDR,
                                        &reg, 1, out, len, pdMS_TO_TICKS(10));
}

/**
 * Read one set of accel + gyro raw values.
 * Registers 0x3B–0x40 are ACCEL_{X,Y,Z}_OUT (big-endian int16).
 * Registers 0x43–0x48 are GYRO_{X,Y,Z}_OUT  (big-endian int16).
 */
static void read_sample(SemaphoreHandle_t mtx,
                        int16_t *ax, int16_t *ay, int16_t *az,
                        int16_t *gx, int16_t *gy, int16_t *gz)
{
    uint8_t buf[12] = {0};  /* 6 accel bytes + 2 temp bytes skipped + 6 gyro */

    /* We read 14 bytes starting from ACCEL_XOUT_H:
     *   [0..5]  accel X,Y,Z
     *   [6..7]  temperature (ignored here)
     *   [8..13] gyro X,Y,Z
     */
    uint8_t full[14] = {0};
    xSemaphoreTake(mtx, portMAX_DELAY);
    i2c_rd(REG_ACCEL_XOUT_H, full, sizeof(full));
    xSemaphoreGive(mtx);

    (void)buf; /* silence unused-variable warning */

    *ax = (int16_t)((full[0]  << 8) | full[1]);
    *ay = (int16_t)((full[2]  << 8) | full[3]);
    *az = (int16_t)((full[4]  << 8) | full[5]);
    /* full[6..7] = temperature, skip */
    *gx = (int16_t)((full[8]  << 8) | full[9]);
    *gy = (int16_t)((full[10] << 8) | full[11]);
    *gz = (int16_t)((full[12] << 8) | full[13]);
}

/* ── public API ─────────────────────────────────────────────────────── */

void mpu6050_calibrate(SemaphoreHandle_t i2c_mutex, uint32_t samples)
{
    printf("[CAL] MPU6050: starting calibration (%lu samples, keep flat & still)\n",
           (unsigned long)samples);

    int64_t sum_ax = 0, sum_ay = 0, sum_az = 0;
    int64_t sum_gx = 0, sum_gy = 0, sum_gz = 0;

    for (uint32_t i = 0; i < samples; i++) {
        int16_t ax, ay, az, gx, gy, gz;
        read_sample(i2c_mutex, &ax, &ay, &az, &gx, &gy, &gz);
        sum_ax += ax;  sum_ay += ay;  sum_az += az;
        sum_gx += gx;  sum_gy += gy;  sum_gz += gz;
        vTaskDelay(pdMS_TO_TICKS(SAMPLE_PERIOD_MS));
        /* Print progress every 25 samples (~500 ms) */
        if ((i + 1) % 25 == 0) {
            printf("[CAL] MPU6050: %lu/%lu samples done\n",
                   (unsigned long)(i + 1), (unsigned long)samples);
        }
    }

    s_cal.accel_offset_x = (int16_t)(sum_ax / samples);
    s_cal.accel_offset_y = (int16_t)(sum_ay / samples);
    s_cal.accel_offset_z = (int16_t)(sum_az / samples);
    s_cal.gyro_offset_x  = (int16_t)(sum_gx / samples);
    s_cal.gyro_offset_y  = (int16_t)(sum_gy / samples);
    s_cal.gyro_offset_z  = (int16_t)(sum_gz / samples);

    printf("[CAL] MPU6050: accel offsets x=%d y=%d z=%d\n",
           s_cal.accel_offset_x, s_cal.accel_offset_y, s_cal.accel_offset_z);
    printf("[CAL] MPU6050:  gyro offsets x=%d y=%d z=%d\n",
           s_cal.gyro_offset_x, s_cal.gyro_offset_y, s_cal.gyro_offset_z);
}

mpu6050_cal_t mpu6050_cal_get(void)
{
    return s_cal;
}
