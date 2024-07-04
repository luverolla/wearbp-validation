/*
 * signal_utils.c
 *
 *  Created on: May 12, 2024
 *      Author: Luigi Verolla
 */
#include <math.h>

#include "dsp/basic_math_functions.h"
#include "dsp/statistics_functions.h"
#include "signal_utils.h"

// ==== PRIVATE ====
float median(float *x, size_t n) {
    // Bubble sort
    for (int i = 0; i < n - 1; i++) {
        for (int j = 0; j < n - i - 1; j++) {
            if (x[j] > x[j + 1]) {
                float temp = x[j];
                x[j] = x[j + 1];
                x[j + 1] = temp;
            }
        }
    }

    if (n % 2 == 0) {
        return (x[n / 2 - 1] + x[n / 2]) / 2.0;
    } else {
        return x[n / 2];
    }
}

float pulse_width_k(const float *x, size_t n, float k) {

    // TODO: implement pulse_width_k function
}

float pulse_width(const float *x, size_t n, float low, float tg_amp) {
    float len = 0.0f;
    for (size_t i = 0; i < n; i++) {
        if (x[i] - low >= tg_amp) {
            len += 1.0f;
        }
    }

    return len / CFG_FSAMPLE;
}

void add_inorder(float *arr, size_t *n, float v) {
    size_t i = *n - 1;
    while (i >= 0 && arr[i] > v) {
        arr[i + 1] = arr[i];
        i--;
    }
    arr[i + 1] = v;
    *n = *n + 1;
}

uint32_t sig_next_fid(uint32_t start, uint32_t end, const uint32_t *fid_src, size_t n_fid_src) {
    uint32_t res_fid = 0;
    for (size_t i = 0; i < n_fid_src; i++) {
        if (fid_src[i] > start && fid_src[i] < end) {
            res_fid = fid_src[i];
            break;
        }
    }

    return res_fid;
}

// ==== PUBLIC ====
void sig_norm(const float *x, size_t n, float *y, sig_norm_type type) {
    if (type == SIG_NORM_RANGE) {
        float min_v, max_v;
        uint32_t min_idx, max_idx;
        arm_min_f32(x, n, &min_v, &min_idx);
        arm_max_f32(x, n, &max_v, &max_idx);

        for (size_t i = 0; i < n; i++) {
            y[i] = (x[i] - min_v) / (max_v - min_v);
        }
    } else if (type == SIG_NORM_GAUSS) {
        float mean, std;
        arm_mean_f32(x, n, &mean);
        arm_std_f32(x, n, &std);

        for (size_t i = 0; i < n; i++) {
            y[i] = (x[i] - mean) / std;
        }
    }
}

void sig_get_fiducials(const float *x, const size_t n, sig_fiducials *out) {

    // peaks/notches
    out->n_speaks = 0;
    out->n_notches = 0;
    for (size_t i = 3; i < n; i++) {
        if (x[i - 2] > x[i - 3] && x[i - 1] <= x[i - 2] && x[i - 1] > x[i]) {
            if (x[i - 2] > 0.65) {
                out->speaks[out->n_speaks] = i - 2;
                out->n_speaks += 1;
            } else if (x[i - 2] > 0.2 && x[i - 2] < 0.65) {
                out->dpeaks[out->n_dpeaks] = i - 2;
                out->n_dpeaks += 1;
            }
        }
    }

    // valleys
    float vl_v;
    uint32_t vl_idx;
    out->n_valleys = 0;
    for (size_t i = 0; i < (out->n_speaks - 1); i++) {
        arm_min_f32(x + out->speaks[i], out->speaks[i + 1] - out->speaks[i], &vl_v, &vl_idx);
        out->valleys[out->n_valleys] = out->speaks[i] + vl_idx;
        out->n_valleys += 1;
    }

    out->n_notches = 0;
    if (out->n_dpeaks > 0) {
        for (size_t i = 0; i < (out->n_speaks - 1); i++) {
            uint32_t dpeak = sig_next_fid(out->speaks[i], out->speaks[i + 1], out->dpeaks, out->n_dpeaks);
            if (dpeak > 0) {
                float notch_v;
                uint32_t notch_idx;
                arm_min_f32(x + out->speaks[i], dpeak - out->speaks[i], &notch_v, notch_idx);
                out->notches[out->n_notches] = out->speaks[i] + notch_idx;
                out->n_notches += 1;
            }
        }
    }
}

uint8_t sig_featex(const float *x, size_t n, const sig_fiducials fid, float *tmp, float *feat_mat) {

    // number of filled position in vector "tmp"
    size_t filled;

    size_t curr_feat;

    uint8_t error = 0;

    const uint32_t FSAMP_MS = 1000 / CFG_FSAMPLE;

    // --- TIME FEATURES ---
    // X (systolic peak)
    curr_feat = 5;
    filled = 0;
    for (size_t i = 0; i < fid.n_valleys - 1; i++) {
        uint32_t speak = sig_next_fid(fid.valleys[i], fid.valleys[i + 1], fid.speaks, fid.n_speaks);
        if (speak > 0) {
            tmp[filled] = x[speak] - x[fid.valleys[i]];
            filled++;
        }
    }
    if (filled > 0) {
        feat_mat[curr_feat] = median(tmp, filled);
    } else {
        error = 1;
    }

    // Y (diastolic peak)
    if (!error) {
        curr_feat++;
        filled = 0;
        for (size_t i = 0; i < fid.n_valleys - 1; i++) {
            uint32_t dpeak = sig_next_fid(fid.valleys[i], fid.valleys[i + 1], fid.dpeaks, fid.n_dpeaks);
            if (dpeak > 0) {
                tmp[filled] = x[dpeak] - x[fid.valleys[i]];
                filled++;
            }
        }
        if (filled > 0) {
            feat_mat[curr_feat] = median(tmp, filled);
        } else {
            error = 1;
        }
    }

    // Z (notch)
    if (!error) {
        curr_feat++;
        filled = 0;
        for (size_t i = 0; i < fid.n_valleys - 1; i++) {
            uint32_t notch = sig_next_fid(fid.valleys[i], fid.valleys[i + 1], fid.notches, fid.n_notches);
            if (notch > 0) {
                tmp[filled] = x[notch] - x[fid.valleys[i]];
                filled++;
            }
        }
        if (filled > 0) {
            feat_mat[curr_feat] = median(tmp, filled);
        } else {
            error = 1;
        }
    }

    // T1 (systolic peak time)
    if (!error) {
        curr_feat++;
        filled = 0;
        for (size_t i = 0; i < fid.n_valleys - 1; i++) {
            uint32_t speak = sig_next_fid(fid.valleys[i], fid.valleys[i + 1], fid.speaks, fid.n_speaks);
            if (speak > 0) {
                tmp[filled] = (speak - fid.valleys[i]) * FSAMP_MS;
                filled++;
            }
        }
        if (filled > 0) {
            feat_mat[curr_feat] = median(tmp, filled);
        } else {
            error = 1;
        }
    }

    // T2 (notch time)
    if (!error) {
        curr_feat++;
        filled = 0;
        for (size_t i = 0; i < fid.n_valleys - 1; i++) {
            uint32_t notch = sig_next_fid(fid.valleys[i], fid.valleys[i + 1], fid.notches, fid.n_notches);
            if (notch > 0) {
                tmp[filled] = (notch - fid.valleys[i]) * FSAMP_MS;
                filled++;
            }
        }
        if (filled > 0) {
            feat_mat[curr_feat] = median(tmp, filled);
        } else {
            error = 1;
        }
    }

    // T3 (diastolic time)
    if (!error) {
        curr_feat++;
        filled = 0;
        for (size_t i = 0; i < fid.n_valleys - 1; i++) {
            uint32_t dpeak = sig_next_fid(fid.valleys[i], fid.valleys[i + 1], fid.dpeaks, fid.n_dpeaks);
            if (dpeak > 0) {
                tmp[filled] = (dpeak - fid.valleys[i]) * FSAMP_MS;
                filled++;
            }
        }
        if (filled > 0) {
            feat_mat[curr_feat] = median(tmp, filled);
        } else {
            error = 1;
        }
    }

    // DT (DeltaT, distance between systolic and diastolic peak)
    if (!error) {
        curr_feat++;
        filled = 0;
        for (size_t i = 0; i < fid.n_valleys - 1; i++) {
            uint32_t speak = sig_next_fid(fid.valleys[i], fid.valleys[i + 1], fid.speaks, fid.n_speaks);
            uint32_t dpeak = sig_next_fid(fid.valleys[i], fid.valleys[i + 1], fid.dpeaks, fid.n_dpeaks);
            if (speak > 0 && dpeak > speak) {
                tmp[filled] = (dpeak - speak) * FSAMP_MS;
                filled++;
            }
        }
        if (filled > 0) {
            feat_mat[curr_feat] = median(tmp, filled);
        } else {
            error = 1;
        }
    }

    // TPI (valley-valley distance)
    if (!error) {
        curr_feat++;
        filled = 0;
        for (size_t i = 0; i < fid.n_valleys - 1; i++) {
            tmp[filled] = (fid.valleys[i + 1] - fid.valleys[i]) * FSAMP_MS;
            filled++;
        }
        if (filled > 0) {
            feat_mat[curr_feat] = median(tmp, filled);
        } else {
            error = 1;
        }
    }

    // TPP (peak-to-peak distance)
    if (!error) {
        curr_feat++;
        filled = 0;
        for (size_t i = 0; i < fid.n_speaks - 1; i++) {
            tmp[filled] = (fid.speaks[i + 1] - fid.speaks[i]) * FSAMP_MS;
            filled++;
        }
        if (filled > 0) {
            feat_mat[curr_feat] = median(tmp, filled);
        } else {
            error = 1;
        }
    }

    // HR (heart rate)
    if (!error) {
        curr_feat++;

        // "tmp" already contains peak-peak distance
        // and "filled" contains the number of values
        // we use the values inside for computing heart rate
        for (size_t i = 0; i < filled; i++) {
            tmp[i] = 60000 / tmp[i];
        }
        feat_mat[curr_feat] = median(tmp, filled);
    }

    // HRV (heart rate variability)
    if (!error) {
        curr_feat++;

        // "tmp" already contains peak-peak distance
        // and "filled" contains the number of values
        // we use the values inside for computing heart rate
        arm_std_f32(tmp, filled, &feat_mat[curr_feat]);
    }

    // IPA (inflection point area)
    if (!error) {
        curr_feat++;
        filled = 0;

        for (size_t i = 0; i < fid.n_valleys - 1; i++) {
            uint32_t notch = sig_next_fid(fid.valleys[i], fid.valleys[i + 1], fid.notches, fid.n_notches);
            if (notch > 0) {
                // systolic part (v1:notch)
                float s_area = 0.0f;
                for (size_t j = fid.valleys[i]; j < notch - 1; j++) {
                    s_area += x[j + 1] - x[j];
                }
                // diastolic part (notch:v2)
                float d_area = 0.0f;
                for (size_t j = notch; j < fid.valleys[i + 1] - 1; j++) {
                    s_area += x[j + 1] - x[j];
                }
                tmp[filled] = (s_area / d_area);
                filled++;
            }
        }

        if (filled > 0) {
            feat_mat[curr_feat] = median(tmp, filled);
        } else {
            error = 1;
        }
    }

    // AGI (agumentation index, Y/X)
    if (!error) {
        curr_feat++;
        // Y is at index 1, X is at index 0
        feat_mat[curr_feat] = feat_mat[1] / feat_mat[0];
    }

    // ALTAGI (alternate agumentation index, (Y-X)/X)
    if (!error) {
        curr_feat++;
        // Y is at index 1, X is at index 0
        feat_mat[curr_feat] = (feat_mat[1] - feat_mat[0]) / feat_mat[0];
    }

    // SOC (systolic output curve, T1/X) 3/0
    if (!error) {
        curr_feat++;
        // T1 at index 3, X at index 0
        feat_mat[curr_feat] = feat_mat[3] / feat_mat[0];
    }

    // DDC (diastolic downward curve, Y/(TPI-T2)
    if (!error) {
        curr_feat++;
        feat_mat[curr_feat] = feat_mat[1] / (feat_mat[7] - feat_mat[5]);
    }

    if (!error) {
        // T1/TPP, T3/TPP, T2/TPP, DT/TPP
        // TPP at index 8
        // T1,T3,T2,DT at index from 3 to 6 included
        for (size_t j = 3; j <= 6; j++) {
            curr_feat++;
            feat_mat[curr_feat] = feat_mat[j] / feat_mat[8];
        }

        // Z/X
        curr_feat++;
        feat_mat[curr_feat] = feat_mat[2] / feat_mat[0];

        // T2/Z
        curr_feat++;
        feat_mat[curr_feat] = feat_mat[5] / feat_mat[2];

        // T3/Y
        curr_feat++;
        feat_mat[curr_feat] = feat_mat[4] / feat_mat[1];

        // X/(TPI-T1)
        curr_feat++;
        feat_mat[curr_feat] = feat_mat[0] / (feat_mat[7] - feat_mat[3]);

        // Z/(TPI-T2)
        curr_feat++;
        feat_mat[curr_feat] = feat_mat[2] / (feat_mat[7] - feat_mat[5]);
    }

    // pulse widths
    float fractions[3] = {0.25f, 0.5f, 0.75f};
    uint8_t frac_idx = 0;
    // index of first width feature (W25)
    // will be populated inside the loop if no error has occurred previously
    size_t firstW = 0;
    while ((!error) && frac_idx < 3) {
        curr_feat++;
        filled = 0;
        firstW = curr_feat;
        for (size_t i = 0; i < fid.n_valleys - 1; i++) {
            float *pulse = x + fid.valleys[i];
            size_t pulse_length = fid.valleys[i + 1] - fid.valleys[i];
            float pulse_width_val = pulse_width_k(pulse, pulse_length, fractions[frac_idx]);

            if (pulse_width_val > 0) {
                tmp[filled] = pulse_width_val;
                filled++;
            }
        }
        if (filled > 0) {
            feat_mat[curr_feat] = median(tmp, filled);
        } else {
            error = 1;
        }

        frac_idx++;
    }

    // pulse width ratios
    if (!error) {
        // index of first pulse width feature (W25)
        for (uint8_t f = 0; f < 3; f++) {
            // ratios from pw/T1 to pw/TPI
            for (uint8_t k = 3; k <= 7; k++) {
                curr_feat++;
                feat_mat[curr_feat] = feat_mat[firstW + f] / feat_mat[k];
            }
        }
    }

    return error;
}
