/**
 * @file lm35_cal.c
 * @brief LM35 analog temperature sensor calibration.
 *
 * Strategy
 * --------
 * The ESP32 ADC has a known non-linearity and the effective reference
 * voltage at 12 dB attenuation is approximately 3100 mV (not 3300 mV).
 * The LM35 outputs 10 mV/°C so the ideal conversion is:
 *
 *   T [°C] = (raw_adc * vref_mv / 4095.0) / 10.0
 *
 * Calibration does two things:
 *   1. Averages `samples` ADC readings to reduce noise (adc_offset = 0
 *      when no reference temperature is given, just average is taken).
 *   2. If a known reference temperature `ref_temp_c` is supplied (> 0),
 *      computes an additive `adc_offset` (in raw ADC counts) such that
 *      the formula above yields `ref_temp_c` when applied to the average.
 *
 * The calibration result stores `vref_mv` (defaults to 3100 mV) so
 * callers can adjust for board-specific supply variations.
 */

#include "calibration.h"

#include <stdio.h>
#include <stdint.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_adc/adc_oneshot.h"

/* ── private constants ──────────────────────────────────────────────── */
#define DEFAULT_VREF_MV   3100.0f  /* ESP32 ADC full-scale at 12 dB atten */
#define ADC_MAX_COUNTS    4095.0f
#define MV_PER_DEG_C      10.0f    /* LM35 characteristic                 */
#define SAMPLE_PERIOD_MS  50       /* slow sensor – no need to rush        */

/* ── module-local state ─────────────────────────────────────────────── */
static lm35_cal_t s_cal = {
    .adc_offset = 0,
    .vref_mv    = DEFAULT_VREF_MV,
};

/* ── public API ─────────────────────────────────────────────────────── */

void lm35_calibrate(adc_oneshot_unit_handle_t adc_handle,
                    int                       lm35_chan,
                    uint32_t                  samples,
                    float                     ref_temp_c)
{
    printf("[CAL] LM35: starting calibration (%lu samples%s)\n",
           (unsigned long)samples,
           ref_temp_c > 0.0f ? " with reference temperature" : "");

    int64_t sum = 0;

    for (uint32_t i = 0; i < samples; i++) {
        int raw = 0;
        adc_oneshot_read(adc_handle, (adc_channel_t)lm35_chan, &raw);
        sum += raw;
        float instant_temp = ((float)raw * DEFAULT_VREF_MV / ADC_MAX_COUNTS) / MV_PER_DEG_C;
        printf("[CAL] LM35: sample %lu/%lu  raw=%d  (%.2f°C)\n",
               (unsigned long)(i + 1), (unsigned long)samples, raw, instant_temp);
        vTaskDelay(pdMS_TO_TICKS(SAMPLE_PERIOD_MS));
    }

    int avg_raw = (int)(sum / (int64_t)samples);

    /* Keep default vref_mv (could be overridden before calling if known). */
    s_cal.vref_mv = DEFAULT_VREF_MV;

    if (ref_temp_c > 0.0f) {
        /*
         * Solve for adc_offset:
         *   ((avg_raw + offset) * vref_mv / ADC_MAX) / MV_PER_DEG = ref_temp
         *   avg_raw + offset = ref_temp * MV_PER_DEG * ADC_MAX / vref_mv
         *   offset = target_counts - avg_raw
         */
        float target_counts = ref_temp_c * MV_PER_DEG_C
                              * ADC_MAX_COUNTS / s_cal.vref_mv;
        s_cal.adc_offset = (int)(target_counts - (float)avg_raw);

        float corrected_temp = ((float)(avg_raw + s_cal.adc_offset)
                                * s_cal.vref_mv / ADC_MAX_COUNTS)
                               / MV_PER_DEG_C;
        printf("[CAL] LM35: avg_raw=%d  offset=%d  ref=%.1f°C  corrected=%.2f°C\n",
               avg_raw, s_cal.adc_offset, ref_temp_c, corrected_temp);
    } else {
        s_cal.adc_offset = 0;
        float measured_temp = ((float)avg_raw * s_cal.vref_mv / ADC_MAX_COUNTS)
                              / MV_PER_DEG_C;
        printf("[CAL] LM35: avg_raw=%d  measured=%.2f°C  (no reference, offset=0)\n",
               avg_raw, measured_temp);
    }
}

lm35_cal_t lm35_cal_get(void)
{
    return s_cal;
}
