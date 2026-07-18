#pragma once

#include <cstdint>
#include <cstdio>

/** GAP-5 — replay-layer adaptive NHC controller (no EKF changes). */

enum class AdaptiveNhcMode : uint8_t {
    OFF = 0,     /**< Static config.nhc_every_n_ticks only. */
    PASSIVE = 1, /**< Log controller; do not apply N (baseline N fixed). */
    ACTIVE = 2,  /**< Apply controller output via ins_ekf_set_nhc_every_n_ticks. */
};

enum class AdaptiveNhcTransitionReason : uint8_t {
    HOLD = 0,
    THRESHOLD_UP = 1,
    THRESHOLD_DOWN = 2,
    INIT = 3,
};

struct AdaptiveNhcControllerConfig {
    double window_s = 1.0;
    double dwell_s = 1.0;
    float threshold_up_1_to_5 = 12.0f;
    float threshold_up_5_to_10 = 22.0f;
    float threshold_down_10_to_5 = 18.0f;
    float threshold_down_5_to_1 = 8.0f;
    uint32_t initial_n = 1U;
};

struct AdaptiveNhcControllerOutput {
    uint32_t nhc_every_n_ticks = 1U;
    uint32_t controller_state = 1U;
    float gamma_raw = 0.0f;
    float gamma_filtered = 0.0f;
    float p_vv_pre = 0.0f;
    float p_vv_post = 0.0f;
    float delta_p_vv = 0.0f;
    bool transition = false;
    double dwell_time_s = 0.0;
    AdaptiveNhcTransitionReason reason = AdaptiveNhcTransitionReason::HOLD;
    bool controller_enabled = false;
};

/** Rolling Γ_inst and Γ̄ (1 s window) from |ΔP_vv| attribution. */
class GammaBarEstimator {
public:
    explicit GammaBarEstimator(double window_s = 1.0);

    void reset();
    void observe(
        double timestamp_s,
        float p_vv_pre,
        float p_vv_post,
        bool nhc_applied);

    float gamma_inst() const { return gamma_inst_; }
    float gamma_filtered() const { return gamma_filtered_; }

private:
    double window_s_;
    float gamma_inst_;
    float gamma_filtered_;
    double sum_predict_;
    double sum_nhc_;
    double last_timestamp_s_;
    double predict_only_delta_ema_;
    bool has_last_;
};

class AdaptiveNhcController {
public:
    explicit AdaptiveNhcController(AdaptiveNhcControllerConfig config = {});

    void reset();
    AdaptiveNhcControllerOutput update(double timestamp_s, float gamma_bar);

    uint32_t current_state_n() const { return current_n_; }

private:
    AdaptiveNhcControllerConfig cfg_;
    uint32_t current_n_;
    double state_enter_timestamp_s_;
    bool initialized_;
};

const char *adaptive_nhc_transition_reason_name(AdaptiveNhcTransitionReason reason);
bool adaptive_nhc_parse_mode(const char *text, AdaptiveNhcMode *out_mode);
const char *adaptive_nhc_mode_name(AdaptiveNhcMode mode);

bool adaptive_nhc_controller_audit_write_header(FILE *fp);
bool adaptive_nhc_controller_audit_write_row(
    FILE *fp,
    double timestamp_s,
    AdaptiveNhcMode mode,
    const AdaptiveNhcControllerOutput &out);
