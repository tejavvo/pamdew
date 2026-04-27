#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "pan_tompkins.h"

typedef struct {
    uint32_t red;
    uint32_t ir;
    float    ppg_bpm;
    float    ppg_spo2;
    float    ir_ac;
    bool     ppg_valid;
    int32_t  ax;
    int32_t  ay;
    int32_t  az;
    int      ecg_raw;
    bool     ecg_leads_ok;
    int      gsr_raw;
    float    temp_c;
} ml_sample_t;

typedef struct {
    uint32_t samples;
    float    ecg_bpm;
    float    mean_rr_ms;
    float    sdnn_ms;
    float    rmssd_ms;
    uint32_t r_peaks;
    float    ppg_bpm_mean;
    float    ppg_spo2_mean;
    float    ppg_valid_ratio;
    float    ir_ac_mean;
    float    accel_mag_mean;
    float    accel_mag_std;
    float    ecg_leads_ok_ratio;
    float    gsr_mean;
    float    gsr_std;
    float    gsr_slope;
    float    temp_mean;
} ml_features_t;

typedef struct {
    float stress_score;
    float artifact_score;
    float anomaly_score;
    float reliability_score;
    const char *label;
    ml_features_t features;
} ml_result_t;

void ml_model_init(void);
void ml_model_sample(const ml_sample_t *sample);
bool ml_model_finalize_window(const pt_stats_t *pt_stats, ml_result_t *out);
void ml_model_log_result(const ml_result_t *result);
