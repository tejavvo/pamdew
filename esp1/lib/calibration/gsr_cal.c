/**
 * @file gsr_cal.c
 * @brief GSR (Galvanic Skin Response) sensor calibration.
 *
 * Strategy
 * --------
 * Skin conductance is highly individual and drifts with temperature and
 * hydration.  A simple session-start calibration measures the resting
 * Skin Conductance Level (SCL) by averaging several ADC readings taken
 * while the subject is sitting relaxed with the electrodes on.
 *
 * Live tasks compare each new reading to `baseline_raw`:
 *   - A sustained rise indicates increased arousal / stress.
 *   - Short transient peaks are Skin Conductance Responses (SCRs).
 *
 * The calibration samples at 10 Hz (100 ms intervals) matching the
 * gsr_task rate so the baseline is computed under the same conditions.
 */

#include "calibration.h"

#include <stdio.h>
#include <stdint.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_adc/adc_oneshot.h"

/* ── private constants ──────────────────────────────────────────────── */
#define SAMPLE_PERIOD_MS 100   /* matches gsr_task rate of 10 Hz */

/* ── module-local state ─────────────────────────────────────────────── */
static gsr_cal_t s_cal = {0};

/* ── public API ─────────────────────────────────────────────────────── */

void gsr_calibrate(adc_oneshot_unit_handle_t adc_handle,
                   int                       gsr_chan,
                   uint32_t                  samples)
{
    printf("[CAL] GSR: starting calibration (%lu samples, sit still & relax)\n",
           (unsigned long)samples);

    int64_t sum = 0;

    for (uint32_t i = 0; i < samples; i++) {
        int raw = 0;
        adc_oneshot_read(adc_handle, (adc_channel_t)gsr_chan, &raw);
        sum += raw;
        printf("[CAL] GSR: sample %lu/%lu  raw=%d\n",
               (unsigned long)(i + 1), (unsigned long)samples, raw);
        vTaskDelay(pdMS_TO_TICKS(SAMPLE_PERIOD_MS));
    }

    s_cal.baseline_raw = (int)(sum / (int64_t)samples);

    printf("[CAL] GSR: baseline_raw=%d\n", s_cal.baseline_raw);
}

gsr_cal_t gsr_cal_get(void)
{
    return s_cal;
}
