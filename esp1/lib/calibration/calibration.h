/**
 * @file calibration.h
 * @brief Public interface for all per-sensor calibration routines.
 *
 * Each sensor has its own .c module under src/calibration/.
 * call_calibrate_all() in main.c invokes every module in order.
 *
 * Calibration results are stored in module-local globals and exposed
 * through the accessor functions declared below.  Sensors that share
 * the I2C bus must be passed the mutex so they can arbitrate access
 * correctly even during calibration.
 */

#pragma once

#include <stdint.h>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "esp_adc/adc_oneshot.h"

/* ------------------------------------------------------------------ */
/*  MAX30102 – SpO2 / Heart-Rate sensor                               */
/* ------------------------------------------------------------------ */

/**
 * @brief Calibration result for the MAX30102.
 *
 * dc_red / dc_ir: mean DC level measured during the calibration window
 *   (finger absent).  Subtract these from live readings to obtain the
 *   AC component used for SpO2 / HR calculation.
 * finger_thresh:  ir value above which a finger is considered present.
 */
typedef struct {
    uint32_t dc_red;
    uint32_t dc_ir;
    uint32_t finger_thresh;
} max30102_cal_t;

/**
 * @brief Run MAX30102 calibration.
 *
 * Collects @p samples raw FIFO readings (no finger on sensor), averages
 * them to obtain the DC baseline, and derives a finger-detection
 * threshold as baseline_ir + FINGER_THRESH_OFFSET.
 *
 * Blocks the calling task for roughly (samples / 100) seconds because
 * the sensor runs at 100 sps.
 *
 * @param i2c_mutex  Mutex guarding the shared I2C bus.
 * @param samples    Number of samples to average (≥ 10 recommended).
 */
void max30102_calibrate(SemaphoreHandle_t i2c_mutex, uint32_t samples);

/** @brief Return the most recent MAX30102 calibration result. */
max30102_cal_t max30102_cal_get(void);


/* ------------------------------------------------------------------ */
/*  MPU6050 – 3-axis accelerometer + gyroscope                        */
/* ------------------------------------------------------------------ */

/**
 * @brief Calibration result for the MPU6050.
 *
 * accel_offset_{x,y,z}: mean raw accel values when sensor is stationary
 *   and flat (gravity along Z only).  Subtract from live readings.
 * gyro_offset_{x,y,z}:  mean raw gyro values at rest.  Subtract from
 *   live readings to remove zero-rate error.
 */
typedef struct {
    int16_t accel_offset_x;
    int16_t accel_offset_y;
    int16_t accel_offset_z;  /* ideally ≈ +16384 LSB (1g), removed too */
    int16_t gyro_offset_x;
    int16_t gyro_offset_y;
    int16_t gyro_offset_z;
} mpu6050_cal_t;

/**
 * @brief Run MPU6050 calibration.
 *
 * Place the sensor on a flat, stationary surface before calling.
 * Collects @p samples readings at 50 Hz and averages them.
 *
 * @param i2c_mutex  Mutex guarding the shared I2C bus.
 * @param samples    Number of samples (≥ 50 recommended).
 */
void mpu6050_calibrate(SemaphoreHandle_t i2c_mutex, uint32_t samples);

/** @brief Return the most recent MPU6050 calibration result. */
mpu6050_cal_t mpu6050_cal_get(void);


/* ------------------------------------------------------------------ */
/*  AD8232 – ECG front-end                                            */
/* ------------------------------------------------------------------ */

/**
 * @brief Calibration result for the AD8232 ECG module.
 *
 * baseline:     Mean ADC value measured during the settling window
 *               (leads attached, patient at rest).
 * noise_floor:  Standard-deviation-derived noise floor (peak-to-peak/6).
 *               Signals smaller than this are treated as artefacts.
 */
typedef struct {
    int baseline;
    int noise_floor;
} ecg_cal_t;

/**
 * @brief Run AD8232 ECG calibration.
 *
 * Samples the ECG ADC channel for @p duration_ms milliseconds with the
 * electrodes attached but the patient at rest.  Computes the DC baseline
 * and noise floor.  The AD8232 LO± pins are checked; calibration is
 * skipped (values set to 0) when leads are off.
 *
 * @param adc_handle   ADC unit handle (ADC1, already configured).
 * @param ecg_chan     ADC channel connected to AD8232 OUT.
 * @param lo_pos_gpio  GPIO number for LO+ lead-off detect.
 * @param lo_neg_gpio  GPIO number for LO- lead-off detect.
 * @param duration_ms  Sampling window length in milliseconds.
 */
void ecg_calibrate(adc_oneshot_unit_handle_t adc_handle,
                   int                       ecg_chan,
                   int                       lo_pos_gpio,
                   int                       lo_neg_gpio,
                   uint32_t                  duration_ms);

/** @brief Return the most recent ECG calibration result. */
ecg_cal_t ecg_cal_get(void);


/* ------------------------------------------------------------------ */
/*  GSR – Galvanic Skin Response                                      */
/* ------------------------------------------------------------------ */

/**
 * @brief Calibration result for the GSR sensor.
 *
 * baseline_raw:  Mean ADC reading with electrodes attached and the
 *                subject relaxed (resting skin conductance level).
 */
typedef struct {
    int baseline_raw;
} gsr_cal_t;

/**
 * @brief Run GSR calibration.
 *
 * Collect @p samples ADC readings at roughly 10 Hz intervals and average
 * them to obtain the resting baseline.  The subject should sit still with
 * the electrodes properly attached.
 *
 * @param adc_handle  ADC unit handle (ADC1, already configured).
 * @param gsr_chan    ADC channel connected to the GSR sensor output.
 * @param samples     Number of samples (≥ 10 recommended).
 */
void gsr_calibrate(adc_oneshot_unit_handle_t adc_handle,
                   int                       gsr_chan,
                   uint32_t                  samples);

/** @brief Return the most recent GSR calibration result. */
gsr_cal_t gsr_cal_get(void);


/* ------------------------------------------------------------------ */
/*  LM35 – Analog temperature sensor                                  */
/* ------------------------------------------------------------------ */

/**
 * @brief Calibration result for the LM35.
 *
 * adc_offset:    Additive correction (in raw ADC counts) determined by
 *                comparing the measured value against a known reference
 *                temperature.  Add to every raw reading before conversion.
 * vref_mv:       Effective ADC full-scale voltage in mV used during
 *                conversion (accounts for 12 dB attenuation).
 */
typedef struct {
    int   adc_offset;
    float vref_mv;
} lm35_cal_t;

/**
 * @brief Run LM35 calibration.
 *
 * Averages @p samples ADC readings to establish the quiescent value.
 * If @p ref_temp_c is > 0 the offset is adjusted so that the computed
 * temperature matches the reference; pass 0.0f to skip offset adjustment
 * (only averaging is performed).
 *
 * @param adc_handle   ADC unit handle (ADC1, already configured).
 * @param lm35_chan    ADC channel connected to LM35 OUT.
 * @param samples      Number of samples to average.
 * @param ref_temp_c   Known reference temperature in °C, or 0.0f to skip.
 */
void lm35_calibrate(adc_oneshot_unit_handle_t adc_handle,
                    int                       lm35_chan,
                    uint32_t                  samples,
                    float                     ref_temp_c);

/** @brief Return the most recent LM35 calibration result. */
lm35_cal_t lm35_cal_get(void);
