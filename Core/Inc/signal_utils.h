/*
 * utils.h
 *
 *  Created on: May 12, 2024
 *      Author: Luigi Verolla
 */

#ifndef INC_SIGNAL_UTILS_H_
#define INC_SIGNAL_UTILS_H_

#include <stdint.h>

#include "config.h"

#define SIG_MAX_FIDUCIALS (2 * CFG_WINSIZE)

typedef enum sig_norm_type {
    SIG_NORM_RANGE,
    SIG_NORM_GAUSS
} sig_norm_type;

typedef struct sig_fiducials {
    uint32_t speaks[SIG_MAX_FIDUCIALS];
    size_t n_speaks;
    uint32_t dpeaks[SIG_MAX_FIDUCIALS];
    size_t n_dpeaks;
    uint32_t valleys[SIG_MAX_FIDUCIALS];
    size_t n_valleys;
    uint32_t notches[SIG_MAX_FIDUCIALS];
    size_t n_notches;
} sig_fiducials;

void sig_norm(const float *x, size_t n, float *y, sig_norm_type type);
void sig_get_fiducials(const float *x, size_t n, sig_fiducials *out);
uint8_t sig_check(const float *x, size_t n, const sig_fiducials fid, float *tmp);
uint8_t sig_featex(const float *x, size_t n, const sig_fiducials fid, float *tmp, float *feats_vec);

#endif /* INC_SIGNAL_UTILS_H_ */
