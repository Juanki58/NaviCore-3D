/**
 * @file meas_reject.hpp
 * @brief Generic measurement-reject taxonomy (state estimation / integrity).
 *
 * Navigation GNSS reject macros in ins_ekf.hpp alias these values. The engine
 * language is "measurement rejected"; GNSS is one aiding source.
 */
#pragma once

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/** No reject / accept. */
#define MEAS_REJECT_NONE 0U
/** Normalized innovation squared gate failed. */
#define MEAS_REJECT_NIS 1U
/** Innovation covariance S singular / not invertible. */
#define MEAS_REJECT_S_SINGULAR 2U
/** Physically inconsistent with predictor (IMU / INS). */
#define MEAS_REJECT_INCONSISTENT 3U

#ifdef __cplusplus
}
#endif
