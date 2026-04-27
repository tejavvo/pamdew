#include "ml_model.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "sdkconfig.h"

#ifndef CONFIG_SENSOR_HUB_ML_LOG_FEATURES
#define CONFIG_SENSOR_HUB_ML_LOG_FEATURES 0
#endif

typedef struct {
    uint32_t samples;
    uint32_t ppg_valid_samples;
    uint32_t ecg_leads_ok_samples;

    float ppg_bpm_sum;
    float ppg_spo2_sum;
    float ir_ac_sum;
    float temp_sum;

    float accel_mag_sum;
    float accel_mag_sq_sum;
    float gsr_sum;
    float gsr_sq_sum;

    int gsr_first;
    int gsr_last;
    bool has_gsr;
} ml_window_acc_t;

static SemaphoreHandle_t s_ml_mutex;
static ml_window_acc_t s_acc;
static bool s_feature_header_printed;

static float clampf(float x, float lo, float hi)
{
    if (x < lo) {
        return lo;
    }
    if (x > hi) {
        return hi;
    }
    return x;
}

static float fast_sigmoid(float x)
{
    return 0.5f * (x / (1.0f + fabsf(x))) + 0.5f;
}

static float safe_std(float sum, float sum_sq, float n)
{
    if (n <= 1.0f) {
        return 0.0f;
    }
    float mean = sum / n;
    float var = (sum_sq / n) - (mean * mean);
    return sqrtf(var > 0.0f ? var : 0.0f);
}

/* Tiny fixed feature model: replace weights after collecting labeled data. */
static float tiny_stress_model(const ml_features_t *f)
{
    float bpm      = (f->ecg_bpm > 0.0f) ? f->ecg_bpm : f->ppg_bpm_mean;
    float bpm_z    = (bpm - 75.0f) / 25.0f;
    float sdnn_z   = (45.0f - f->sdnn_ms) / 35.0f;
    float rmssd_z  = (35.0f - f->rmssd_ms) / 30.0f;
    float gsr_z    = f->gsr_slope / 250.0f;
    float motion_z = f->accel_mag_std / 3500.0f;

    float hidden0 = fast_sigmoid((0.90f * bpm_z) + (0.75f * sdnn_z) + (0.45f * gsr_z) - 0.55f);
    float hidden1 = fast_sigmoid((0.70f * rmssd_z) + (0.60f * gsr_z) - (0.50f * motion_z) - 0.20f);
    float hidden2 = fast_sigmoid((0.65f * bpm_z) - (0.35f * motion_z) + (0.25f * f->ecg_leads_ok_ratio));

    return clampf(100.0f * fast_sigmoid((1.10f * hidden0) + (0.95f * hidden1) +
                                         (0.70f * hidden2) - 1.55f),
                  0.0f, 100.0f);
}

void ml_model_init(void)
{
    s_ml_mutex = xSemaphoreCreateMutex();
    memset(&s_acc, 0, sizeof(s_acc));
    s_feature_header_printed = false;
}

void ml_model_sample(const ml_sample_t *sample)
{
    if (sample == NULL || s_ml_mutex == NULL) {
        return;
    }

    float ax = (float)sample->ax;
    float ay = (float)sample->ay;
    float az = (float)sample->az;
    float accel_mag = sqrtf((ax * ax) + (ay * ay) + (az * az));

    xSemaphoreTake(s_ml_mutex, portMAX_DELAY);
    s_acc.samples++;
    s_acc.accel_mag_sum += accel_mag;
    s_acc.accel_mag_sq_sum += accel_mag * accel_mag;
    s_acc.gsr_sum += (float)sample->gsr_raw;
    s_acc.gsr_sq_sum += (float)sample->gsr_raw * (float)sample->gsr_raw;
    s_acc.temp_sum += sample->temp_c;
    s_acc.gsr_last = sample->gsr_raw;
    if (!s_acc.has_gsr) {
        s_acc.gsr_first = sample->gsr_raw;
        s_acc.has_gsr = true;
    }

    if (sample->ppg_valid) {
        s_acc.ppg_valid_samples++;
        s_acc.ppg_bpm_sum += sample->ppg_bpm;
        s_acc.ppg_spo2_sum += sample->ppg_spo2;
        s_acc.ir_ac_sum += sample->ir_ac;
    }
    if (sample->ecg_leads_ok) {
        s_acc.ecg_leads_ok_samples++;
    }
    xSemaphoreGive(s_ml_mutex);
}

bool ml_model_finalize_window(const pt_stats_t *pt_stats, ml_result_t *out)
{
    if (pt_stats == NULL || out == NULL || s_ml_mutex == NULL) {
        return false;
    }

    ml_window_acc_t acc;
    xSemaphoreTake(s_ml_mutex, portMAX_DELAY);
    acc = s_acc;
    memset(&s_acc, 0, sizeof(s_acc));
    xSemaphoreGive(s_ml_mutex);

    if (acc.samples == 0) {
        return false;
    }

    memset(out, 0, sizeof(*out));
    ml_features_t *f = &out->features;
    float n = (float)acc.samples;
    float ppg_n = (float)acc.ppg_valid_samples;

    f->samples = acc.samples;
    f->ecg_bpm = pt_stats->bpm;
    f->mean_rr_ms = pt_stats->mean_rr_ms;
    f->sdnn_ms = pt_stats->sdnn_ms;
    f->rmssd_ms = pt_stats->rmssd_ms;
    f->r_peaks = pt_stats->r_peaks;
    f->ppg_valid_ratio = ppg_n / n;
    f->ppg_bpm_mean = (ppg_n > 0.0f) ? (acc.ppg_bpm_sum / ppg_n) : 0.0f;
    f->ppg_spo2_mean = (ppg_n > 0.0f) ? (acc.ppg_spo2_sum / ppg_n) : 0.0f;
    f->ir_ac_mean = (ppg_n > 0.0f) ? (acc.ir_ac_sum / ppg_n) : 0.0f;
    f->accel_mag_mean = acc.accel_mag_sum / n;
    f->accel_mag_std = safe_std(acc.accel_mag_sum, acc.accel_mag_sq_sum, n);
    f->ecg_leads_ok_ratio = (float)acc.ecg_leads_ok_samples / n;
    f->gsr_mean = acc.gsr_sum / n;
    f->gsr_std = safe_std(acc.gsr_sum, acc.gsr_sq_sum, n);
    f->gsr_slope = acc.has_gsr ? ((float)acc.gsr_last - (float)acc.gsr_first) : 0.0f;
    f->temp_mean = acc.temp_sum / n;

    float motion_component = clampf(f->accel_mag_std / 4500.0f, 0.0f, 1.0f);
    float ppg_bad_component = 1.0f - f->ppg_valid_ratio;
    float ecg_bad_component = 1.0f - f->ecg_leads_ok_ratio;
    out->artifact_score = 100.0f * clampf((0.55f * motion_component) +
                                           (0.30f * ppg_bad_component) +
                                           (0.15f * ecg_bad_component),
                                           0.0f, 1.0f);

    out->stress_score = tiny_stress_model(f);
    if (out->artifact_score > 55.0f) {
        out->stress_score *= 0.65f;
    }

    float hr_disagreement = 0.0f;
    if (f->ecg_bpm > 0.0f && f->ppg_bpm_mean > 0.0f) {
        hr_disagreement = fabsf(f->ecg_bpm - f->ppg_bpm_mean) / 35.0f;
    }
    float spo2_component = (f->ppg_spo2_mean > 0.0f && f->ppg_spo2_mean < 92.0f)
                           ? ((92.0f - f->ppg_spo2_mean) / 12.0f)
                           : 0.0f;
    float qrs_component = (f->r_peaks == 0 && f->ecg_leads_ok_ratio > 0.8f) ? 1.0f : 0.0f;
    out->anomaly_score = 100.0f * clampf((0.45f * hr_disagreement) +
                                          (0.35f * spo2_component) +
                                          (0.20f * qrs_component),
                                          0.0f, 1.0f);

    out->reliability_score = 100.0f * clampf(1.0f - (out->artifact_score / 100.0f), 0.0f, 1.0f);

    if (out->artifact_score > 55.0f) {
        out->label = "motion_artifact";
    } else if (out->anomaly_score > 65.0f) {
        out->label = "anomaly";
    } else if (out->stress_score > 65.0f) {
        out->label = "high_arousal";
    } else {
        out->label = "nominal";
    }

    return true;
}

void ml_model_log_result(const ml_result_t *result)
{
    if (result == NULL) {
        return;
    }

    printf("[ML] label=%s stress=%.0f artifact=%.0f anomaly=%.0f reliability=%.0f "
           "ecg_bpm=%.1f ppg_bpm=%.1f ppg_valid=%.2f accel_std=%.1f gsr_slope=%.1f\n",
           result->label,
           result->stress_score,
           result->artifact_score,
           result->anomaly_score,
           result->reliability_score,
           result->features.ecg_bpm,
           result->features.ppg_bpm_mean,
           result->features.ppg_valid_ratio,
           result->features.accel_mag_std,
           result->features.gsr_slope);

#if CONFIG_SENSOR_HUB_ML_LOG_FEATURES
    if (!s_feature_header_printed) {
        printf("ml_csv,samples,ecg_bpm,mean_rr_ms,sdnn_ms,rmssd_ms,r_peaks,ppg_bpm_mean,"
               "ppg_spo2_mean,ppg_valid_ratio,ir_ac_mean,accel_mag_mean,accel_mag_std,"
               "ecg_leads_ok_ratio,gsr_mean,gsr_std,gsr_slope,temp_mean,stress,artifact,"
               "anomaly,reliability,label\n");
        s_feature_header_printed = true;
    }
    const ml_features_t *f = &result->features;
    printf("ml_csv,%lu,%.2f,%.2f,%.2f,%.2f,%lu,%.2f,%.2f,%.3f,%.2f,%.2f,%.2f,"
           "%.3f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%s\n",
           (unsigned long)f->samples,
           f->ecg_bpm,
           f->mean_rr_ms,
           f->sdnn_ms,
           f->rmssd_ms,
           (unsigned long)f->r_peaks,
           f->ppg_bpm_mean,
           f->ppg_spo2_mean,
           f->ppg_valid_ratio,
           f->ir_ac_mean,
           f->accel_mag_mean,
           f->accel_mag_std,
           f->ecg_leads_ok_ratio,
           f->gsr_mean,
           f->gsr_std,
           f->gsr_slope,
           f->temp_mean,
           result->stress_score,
           result->artifact_score,
           result->anomaly_score,
           result->reliability_score,
           result->label);
#endif
}
