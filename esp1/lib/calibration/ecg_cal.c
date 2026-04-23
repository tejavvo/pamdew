/**
 * @file ecg_cal.c
 * @brief AD8232 ECG front-end calibration.
 *
 * Strategy
 * --------
 * The AD8232 output has a DC offset that depends on the electrode
 * impedance and supply voltage.  With the electrodes attached and the
 * subject sitting still (no cardiac activity is needed, just a quiet
 * baseline), we sample the ADC at the full 250 Hz rate for a configurable
 * duration.  We then compute:
 *
 *   baseline   = mean(samples)
 *   noise_floor = peak_to_peak / 6   (≈ 1 σ for normally distributed noise)
 *
 * Live tasks should subtract `baseline` from every raw reading and discard
 * peaks below `noise_floor` as noise.
 *
 * Lead-off detection is honoured: if either LO± pin is HIGH (lead off)
 * at the start, calibration is skipped and both fields are set to 0.
 */

#include "calibration.h"

#include <stdio.h>
#include <stdint.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_adc/adc_oneshot.h"

/* ── private constants ──────────────────────────────────────────────── */
#define SAMPLE_PERIOD_MS 4   /* 250 Hz → 4 ms per sample */

/* ── module-local state ─────────────────────────────────────────────── */
static ecg_cal_t s_cal = {0};

/* ── public API ─────────────────────────────────────────────────────── */

void ecg_calibrate(adc_oneshot_unit_handle_t adc_handle,
                   int                       ecg_chan,
                   int                       lo_pos_gpio,
                   int                       lo_neg_gpio,
                   uint32_t                  duration_ms)
{
    /* Check lead-off detect lines before proceeding. */
    if (gpio_get_level((gpio_num_t)lo_pos_gpio) ||
        gpio_get_level((gpio_num_t)lo_neg_gpio))
    {
        printf("[CAL] ECG: leads off – skipping calibration (baseline=0)\n");
        s_cal.baseline    = 0;
        s_cal.noise_floor = 0;
        return;
    }

    uint32_t samples = duration_ms / SAMPLE_PERIOD_MS;
    if (samples < 1) samples = 1;

    printf("[CAL] ECG: starting calibration (%lu samples over %lu ms)\n",
           (unsigned long)samples, (unsigned long)duration_ms);

    int64_t sum = 0;
    int     mn  = INT32_MAX;
    int     mx  = INT32_MIN;

    for (uint32_t i = 0; i < samples; i++) {
        int raw = 0;
        adc_oneshot_read(adc_handle, (adc_channel_t)ecg_chan, &raw);
        sum += raw;
        if (raw < mn) mn = raw;
        if (raw > mx) mx = raw;
        vTaskDelay(pdMS_TO_TICKS(SAMPLE_PERIOD_MS));
        /* Print progress every ~500 ms (125 samples × 4 ms) */
        if ((i + 1) % 125 == 0) {
            printf("[CAL] ECG: %lu/%lu samples done (cur raw=%d)\n",
                   (unsigned long)(i + 1), (unsigned long)samples, raw);
        }
    }

    s_cal.baseline    = (int)(sum / (int64_t)samples);
    s_cal.noise_floor = (mx - mn) / 6;  /* ≈ 1 σ */

    printf("[CAL] ECG: baseline=%d  noise_floor=%d  (pk-pk=%d)\n",
           s_cal.baseline, s_cal.noise_floor, mx - mn);
}

ecg_cal_t ecg_cal_get(void)
{
    return s_cal;
}
