/**
 * @file pan_tompkins.h
 * @brief Pan-Tompkins QRS / R-peak detector, adapted for 250 Hz.
 *
 * Usage
 * -----
 *   pt_init();                          // once, before scheduler starts
 *   pt_feed(raw_adc);                   // called from ecg_task at 250 Hz
 *   pt_get_and_reset_stats(&stats);     // called from reporter_task every 30 s
 */
#pragma once

#include <stdint.h>
#include <stdbool.h>

/**
 * @brief HRV feature set extracted over a 30-second window.
 *
 * bpm        – mean heart rate
 * mean_rr_ms – mean RR interval in milliseconds
 * sdnn_ms    – standard deviation of RR intervals (overall HRV)
 * rmssd_ms   – root-mean-square of successive RR differences (short-term HRV)
 * min_rr_ms  – shortest RR interval seen in the window
 * max_rr_ms  – longest  RR interval seen in the window
 * r_peaks    – total R-peaks counted in the window
 */
typedef struct {
    float    bpm;
    float    mean_rr_ms;
    float    sdnn_ms;
    float    rmssd_ms;
    uint32_t min_rr_ms;
    uint32_t max_rr_ms;
    uint32_t r_peaks;
} pt_stats_t;

/** Initialise internal state. Must be called once before pt_feed(). */
void pt_init(void);

/**
 * @brief Push one ECG sample through the Pan-Tompkins pipeline.
 * @param raw  Raw ADC count (0-4095). Subtract ECG baseline first for best results.
 * @return     true on the sample where an R-peak is declared.
 *
 * NOT thread-safe – call only from ecg_task.
 */
bool pt_feed(int raw);

/**
 * @brief Atomically snapshot and clear the RR-interval accumulator.
 * @param out  Filled with statistics for the elapsed window.
 *
 * Thread-safe – may be called from reporter_task while pt_feed() runs
 * on ecg_task (uses a brief critical section).
 */
void pt_get_and_reset_stats(pt_stats_t *out);
