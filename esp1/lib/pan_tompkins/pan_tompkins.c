/**
 * @file pan_tompkins.c
 * @brief Pan-Tompkins QRS detector – adapted for 250 Hz from the original
 *        200 Hz reference (Pan & Tompkins, IEEE Trans. Biomed. Eng., 1985).
 *
 * All buffer delays are scaled by 250/200 = 1.25 from the paper values.
 *
 * Pipeline:
 *   raw → LPF → HPF → Derivative → Square → MWI → Adaptive Threshold
 *
 * Key timing at 250 Hz
 *   LPF  delay  : 8 taps   (6  × 1.25)
 *   HPF  delay  : 20 taps  (16 × 1.25)
 *   HPF  window : 40 taps  (32 × 1.25)
 *   Derivative  : 4-sample window
 *   MWI  window : 38 taps  (150 ms × 250 Hz)
 *   Refractory  : 50 taps  (200 ms × 250 Hz)
 */

#include "pan_tompkins.h"

#include <string.h>
#include <math.h>
#include <stdint.h>
#include <stdbool.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_timer.h"

/* ── Compile-time constants ────────────────────────────────────────── */
#define FS              250u          /* sampling frequency, Hz          */
#define LPF_BUF         16u           /* must be >= 16 (delay = 8 taps)  */
#define HPF_BUF         64u           /* must be >= 41 (delay = 40 taps) */
#define DERIV_BUF       8u            /* must be >= 5                    */
#define MWI_LEN         38u           /* 150 ms window                   */
#define REFRACTORY      50u           /* 200 ms dead-time after R-peak   */
#define RR_BUF_LEN      300u          /* max RR intervals per 30-s window */
#define INIT_SPKI       1000000L      /* initial signal-peak estimate     */
#define INIT_NPKI       50000L        /* initial noise-peak estimate      */

/* ── Module state ──────────────────────────────────────────────────── */
static portMUX_TYPE pt_mux = portMUX_INITIALIZER_UNLOCKED;

static struct {
    /* --- LPF circular buffer ---
     * y[n] = 2y[n-1] - y[n-2] + x[n] - 2x[n-8] + x[n-15]            */
    int32_t lpf_y1, lpf_y2;
    int32_t lpf_x[LPF_BUF];
    uint8_t lpf_i;

    /* --- HPF: scaled accumulator (Y = 40 * y) ---
     * Y[n] = Y[n-1] - x[n] + 40*x[n-20] - 40*x[n-21] + x[n-40]      */
    int64_t hpf_Y;
    int32_t hpf_x[HPF_BUF];
    uint8_t hpf_i;

    /* --- Derivative ---
     * y[n] = 2x[n] + x[n-1] - x[n-3] - 2x[n-4]                       */
    int32_t drv_x[DERIV_BUF];
    uint8_t drv_i;

    /* --- Moving-window integrator ---                                   */
    int64_t mwi_sum;
    int64_t mwi_buf[MWI_LEN];
    uint8_t mwi_i;

    /* --- Adaptive thresholds ---                                        */
    int64_t spki;           /* signal-peak running estimate              */
    int64_t npki;           /* noise-peak running estimate               */
    int64_t thr1;           /* primary threshold                         */
    int64_t thr2;           /* secondary threshold (searchback)          */
    int64_t peak_val;       /* candidate peak value in current window    */
    uint32_t peak_age;      /* samples since candidate peak              */

    /* --- Refractory counter ---                                         */
    uint32_t ref_ctr;

    /* --- RR statistics (written by pt_feed, read by reporter) ---       */
    uint32_t rr_buf[RR_BUF_LEN];
    uint32_t rr_count;
    uint64_t last_r_us;     /* µs timestamp of previous R-peak           */
    uint32_t r_peaks;       /* total peaks this window                   */
} s;

/* ── Helpers ───────────────────────────────────────────────────────── */
#define CIRC(idx, len)  ((uint8_t)(((idx) + 1) % (len)))
#define IDX(buf, i, d, len) (buf)[((uint8_t)((i) + (len) - (d)) % (len))]

static void update_thresholds(void)
{
    s.thr1 = s.npki + (s.spki - s.npki) / 4;
    s.thr2 = s.thr1 / 2;
}

/* ── Public API ────────────────────────────────────────────────────── */

void pt_init(void)
{
    memset(&s, 0, sizeof(s));
    s.spki   = INIT_SPKI;
    s.npki   = INIT_NPKI;
    s.last_r_us = 0;
    update_thresholds();
}

bool pt_feed(int raw)
{
    bool r_detected = false;

    /* ── Stage 1: Low-pass filter ───────────────────────────────────
     * y[n] = 2y[n-1] - y[n-2] + x[n] - 2x[n-8] + x[n-15]
     * Gain ≈ 36 (not normalised – thresholds absorb it)               */
    s.lpf_x[s.lpf_i] = (int32_t)raw;
    int32_t xn8  = IDX(s.lpf_x, s.lpf_i, 8,  LPF_BUF);
    int32_t xn15 = IDX(s.lpf_x, s.lpf_i, 15, LPF_BUF);
    int32_t lpf_out = 2*s.lpf_y1 - s.lpf_y2
                      + (int32_t)raw - 2*xn8 + xn15;
    s.lpf_y2 = s.lpf_y1;
    s.lpf_y1 = lpf_out;
    s.lpf_i  = CIRC(s.lpf_i, LPF_BUF);

    /* ── Stage 2: High-pass filter ─────────────────────────────────
     * Y[n] = Y[n-1] - x[n] + 40*x[n-20] - 40*x[n-21] + x[n-40]
     * where Y = 40 * y  (avoids fractional division)                  */
    s.hpf_x[s.hpf_i] = lpf_out;
    int32_t hx20 = IDX(s.hpf_x, s.hpf_i, 20, HPF_BUF);
    int32_t hx21 = IDX(s.hpf_x, s.hpf_i, 21, HPF_BUF);
    int32_t hx40 = IDX(s.hpf_x, s.hpf_i, 40, HPF_BUF);
    s.hpf_Y = s.hpf_Y - lpf_out
              + 40*(int64_t)hx20 - 40*(int64_t)hx21
              + hx40;
    int32_t hpf_out = (int32_t)(s.hpf_Y / 40);
    s.hpf_i = CIRC(s.hpf_i, HPF_BUF);

    /* ── Stage 3: Derivative ────────────────────────────────────────
     * y[n] = 2x[n] + x[n-1] - x[n-3] - 2x[n-4]                      */
    s.drv_x[s.drv_i] = hpf_out;
    int32_t drv_out = 2 * hpf_out
                    + IDX(s.drv_x, s.drv_i, 1, DERIV_BUF)
                    - IDX(s.drv_x, s.drv_i, 3, DERIV_BUF)
                    - 2 * IDX(s.drv_x, s.drv_i, 4, DERIV_BUF);
    s.drv_i = CIRC(s.drv_i, DERIV_BUF);

    /* ── Stage 4: Squaring (int64 to avoid overflow) ────────────── */
    int64_t sq_out = (int64_t)drv_out * drv_out;

    /* ── Stage 5: Moving-window integration ─────────────────────── */
    s.mwi_sum -= s.mwi_buf[s.mwi_i];
    s.mwi_buf[s.mwi_i] = sq_out;
    s.mwi_sum += sq_out;
    s.mwi_i = (uint8_t)((s.mwi_i + 1) % MWI_LEN);
    int64_t mwi_out = s.mwi_sum / MWI_LEN;

    /* ── Stage 6: Adaptive thresholding ─────────────────────────── */
    if (s.ref_ctr > 0) {
        /* In refractory period – ignore everything */
        s.ref_ctr--;
        goto done;
    }

    /* Track the running peak candidate */
    if (mwi_out >= s.peak_val) {
        s.peak_val = mwi_out;
        s.peak_age = 0;
    } else {
        s.peak_age++;
    }

    /* Declare an R-peak once the MWI has fallen back from the candidate.
     * Minimum peak age of ~40 ms (10 samples) avoids premature firing.  */
    if (s.peak_age == 10) {
        if (s.peak_val > s.thr1) {
            /* ── Confirmed signal peak ── */
            uint64_t now_us = (uint64_t)esp_timer_get_time();

            if (s.last_r_us > 0) {
                uint32_t rr_ms = (uint32_t)((now_us - s.last_r_us) / 1000u);
                /* Physiological range: 300–2000 ms (30–200 BPM) */
                if (rr_ms >= 300 && rr_ms <= 2000) {
                    taskENTER_CRITICAL(&pt_mux); /* brief – just write pointer */
                    if (s.rr_count < RR_BUF_LEN)
                        s.rr_buf[s.rr_count++] = rr_ms;
                    s.r_peaks++;
                    taskEXIT_CRITICAL(&pt_mux);
                }
            }
            s.last_r_us = now_us;

            /* Update signal-peak estimate (8:1 weighting) */
            s.spki = (7 * s.spki + s.peak_val) / 8;
            s.ref_ctr = REFRACTORY;
            r_detected = true;

        } else if (s.peak_val > s.thr2) {
            /* ── Noise peak ── */
            s.npki = (7 * s.npki + s.peak_val) / 8;
        }

        /* Reset candidate for next QRS search */
        s.peak_val = 0;
        s.peak_age = 0;
        update_thresholds();
    }

done:
    return r_detected;
}

void pt_get_and_reset_stats(pt_stats_t *out)
{
    /* Snapshot the RR buffer under a critical section then process
     * outside it (CPU-heavy maths must not block interrupts)          */
    uint32_t  rr_snap[RR_BUF_LEN];
    uint32_t  n, r_peaks_snap;

    taskENTER_CRITICAL(&pt_mux);
    n            = s.rr_count;
    r_peaks_snap = s.r_peaks;
    memcpy(rr_snap, s.rr_buf, n * sizeof(uint32_t));
    s.rr_count = 0;
    s.r_peaks  = 0;
    taskEXIT_CRITICAL(&pt_mux);

    /* Defaults for empty window */
    *out = (pt_stats_t){ 0 };
    out->r_peaks   = r_peaks_snap;
    out->min_rr_ms = UINT32_MAX;

    if (n == 0) return;

    /* Mean RR */
    uint64_t sum = 0;
    for (uint32_t i = 0; i < n; i++) {
        sum += rr_snap[i];
        if (rr_snap[i] < out->min_rr_ms) out->min_rr_ms = rr_snap[i];
        if (rr_snap[i] > out->max_rr_ms) out->max_rr_ms = rr_snap[i];
    }
    out->mean_rr_ms = (float)sum / (float)n;
    out->bpm        = 60000.0f / out->mean_rr_ms;

    /* SDNN */
    float var = 0.0f;
    for (uint32_t i = 0; i < n; i++) {
        float d = (float)rr_snap[i] - out->mean_rr_ms;
        var += d * d;
    }
    out->sdnn_ms = sqrtf(var / (float)n);

    /* RMSSD */
    if (n >= 2) {
        float sq_sum = 0.0f;
        for (uint32_t i = 1; i < n; i++) {
            float d = (float)rr_snap[i] - (float)rr_snap[i - 1];
            sq_sum += d * d;
        }
        out->rmssd_ms = sqrtf(sq_sum / (float)(n - 1));
    }

    if (out->min_rr_ms == UINT32_MAX) out->min_rr_ms = 0;
}
