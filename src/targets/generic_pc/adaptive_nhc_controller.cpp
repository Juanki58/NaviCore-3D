#include "adaptive_nhc_controller.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>

GammaBarEstimator::GammaBarEstimator(double window_s)
    : window_s_(window_s > 0.0 ? window_s : 1.0)
    , gamma_inst_(0.0f)
    , gamma_filtered_(0.0f)
    , sum_predict_(0.0)
    , sum_nhc_(0.0)
    , last_timestamp_s_(0.0)
    , predict_only_delta_ema_(0.05)
    , has_last_(false)
{
}

void GammaBarEstimator::reset()
{
    gamma_inst_ = 0.0f;
    gamma_filtered_ = 0.0f;
    sum_predict_ = 0.0;
    sum_nhc_ = 0.0;
    last_timestamp_s_ = 0.0;
    predict_only_delta_ema_ = 0.05;
    has_last_ = false;
}

void GammaBarEstimator::observe(
    double timestamp_s,
    float p_vv_pre,
    float p_vv_post,
    bool nhc_applied)
{
    const double delta = static_cast<double>(std::fabs(p_vv_post - p_vv_pre));
    const double prev_ts = last_timestamp_s_;

    if (has_last_ && timestamp_s > prev_ts) {
        const double dt = timestamp_s - prev_ts;
        if (dt > 0.0) {
            const double decay = std::exp(-dt / window_s_);
            sum_predict_ *= decay;
            sum_nhc_ *= decay;
        }
    }

    if (delta > 0.0) {
        if (nhc_applied) {
            const double predict_part = std::min(delta, predict_only_delta_ema_);
            const double nhc_part = std::max(0.0, delta - predict_part);
            sum_predict_ += predict_part;
            sum_nhc_ += nhc_part;
        } else {
            sum_predict_ += delta;
            predict_only_delta_ema_ =
                0.95 * predict_only_delta_ema_ + 0.05 * static_cast<double>(delta);
        }
    }

    last_timestamp_s_ = timestamp_s;
    has_last_ = true;

    if (sum_predict_ > 1e-9) {
        gamma_inst_ = static_cast<float>(sum_nhc_ / sum_predict_);
    } else {
        gamma_inst_ = 0.0f;
    }

    if (!has_last_ || prev_ts <= 0.0 || timestamp_s <= prev_ts) {
        gamma_filtered_ = gamma_inst_;
        return;
    }

    const double dt = std::max(0.001, timestamp_s - prev_ts);
    const double alpha = std::min(1.0, dt / window_s_);
    gamma_filtered_ = static_cast<float>(
        (1.0 - alpha) * static_cast<double>(gamma_filtered_)
        + alpha * static_cast<double>(gamma_inst_));
}

AdaptiveNhcController::AdaptiveNhcController(AdaptiveNhcControllerConfig config)
    : cfg_(config)
    , current_n_(config.initial_n > 0U ? config.initial_n : 1U)
    , state_enter_timestamp_s_(0.0)
    , initialized_(false)
{
}

void AdaptiveNhcController::reset()
{
    current_n_ = cfg_.initial_n > 0U ? cfg_.initial_n : 1U;
    state_enter_timestamp_s_ = 0.0;
    initialized_ = false;
}

AdaptiveNhcControllerOutput AdaptiveNhcController::update(double timestamp_s, float gamma_bar)
{
    AdaptiveNhcControllerOutput out{};
    out.gamma_raw = gamma_bar;
    out.gamma_filtered = gamma_bar;
    out.controller_state = current_n_;
    out.nhc_every_n_ticks = current_n_;
    out.controller_enabled = true;
    out.reason = AdaptiveNhcTransitionReason::HOLD;

    if (!initialized_) {
        initialized_ = true;
        state_enter_timestamp_s_ = timestamp_s;
        out.dwell_time_s = 0.0;
        out.reason = AdaptiveNhcTransitionReason::INIT;
        return out;
    }

    out.dwell_time_s = timestamp_s - state_enter_timestamp_s_;
    if (out.dwell_time_s < cfg_.dwell_s) {
        return out;
    }

    uint32_t target_n = current_n_;
    AdaptiveNhcTransitionReason reason = AdaptiveNhcTransitionReason::HOLD;

    if (current_n_ == 1U) {
        if (gamma_bar > cfg_.threshold_up_1_to_5) {
            target_n = 5U;
            reason = AdaptiveNhcTransitionReason::THRESHOLD_UP;
        }
    } else if (current_n_ == 5U) {
        if (gamma_bar >= cfg_.threshold_up_5_to_10) {
            target_n = 10U;
            reason = AdaptiveNhcTransitionReason::THRESHOLD_UP;
        } else if (gamma_bar < cfg_.threshold_down_5_to_1) {
            target_n = 1U;
            reason = AdaptiveNhcTransitionReason::THRESHOLD_DOWN;
        }
    } else if (current_n_ == 10U) {
        if (gamma_bar < cfg_.threshold_down_10_to_5) {
            target_n = 5U;
            reason = AdaptiveNhcTransitionReason::THRESHOLD_DOWN;
        }
    }

    if (target_n != current_n_) {
        current_n_ = target_n;
        state_enter_timestamp_s_ = timestamp_s;
        out.transition = true;
        out.reason = reason;
        out.dwell_time_s = 0.0;
    }

    out.controller_state = current_n_;
    out.nhc_every_n_ticks = current_n_;
    return out;
}

const char *adaptive_nhc_transition_reason_name(AdaptiveNhcTransitionReason reason)
{
    switch (reason) {
    case AdaptiveNhcTransitionReason::THRESHOLD_UP:
        return "threshold_up";
    case AdaptiveNhcTransitionReason::THRESHOLD_DOWN:
        return "threshold_down";
    case AdaptiveNhcTransitionReason::INIT:
        return "init";
    case AdaptiveNhcTransitionReason::HOLD:
    default:
        return "hold";
    }
}

bool adaptive_nhc_parse_mode(const char *text, AdaptiveNhcMode *out_mode)
{
    if (text == nullptr || out_mode == nullptr) {
        return false;
    }
    if (std::strcmp(text, "off") == 0 || std::strcmp(text, "disabled") == 0) {
        *out_mode = AdaptiveNhcMode::OFF;
        return true;
    }
    if (std::strcmp(text, "passive") == 0) {
        *out_mode = AdaptiveNhcMode::PASSIVE;
        return true;
    }
    if (std::strcmp(text, "active") == 0 || std::strcmp(text, "enabled") == 0) {
        *out_mode = AdaptiveNhcMode::ACTIVE;
        return true;
    }
    return false;
}

const char *adaptive_nhc_mode_name(AdaptiveNhcMode mode)
{
    switch (mode) {
    case AdaptiveNhcMode::OFF:
        return "off";
    case AdaptiveNhcMode::PASSIVE:
        return "passive";
    case AdaptiveNhcMode::ACTIVE:
        return "active";
    default:
        return "unknown";
    }
}

bool adaptive_nhc_controller_audit_write_header(FILE *fp)
{
    if (fp == nullptr) {
        return false;
    }
    return std::fprintf(
               fp,
               "timestamp_s,controller_enabled,controller_mode,gamma_raw,gamma_filtered,"
               "p_vv_pre,p_vv_post,delta_p_vv,"
               "nhc_every_n_ticks,controller_state,transition,dwell_time_s,reason\n")
        >= 0;
}

bool adaptive_nhc_controller_audit_write_row(
    FILE *fp,
    double timestamp_s,
    AdaptiveNhcMode mode,
    const AdaptiveNhcControllerOutput &out)
{
    if (fp == nullptr) {
        return false;
    }
    const int controller_enabled =
        (mode == AdaptiveNhcMode::PASSIVE) ? 1
        : (mode == AdaptiveNhcMode::ACTIVE ? 2 : 0);
    return std::fprintf(
               fp,
               "%.9f,%d,%s,%.6f,%.6f,%.6f,%.6f,%.6f,%u,%u,%d,%.6f,%s\n",
               timestamp_s,
               controller_enabled,
               adaptive_nhc_mode_name(mode),
               static_cast<double>(out.gamma_raw),
               static_cast<double>(out.gamma_filtered),
               static_cast<double>(out.p_vv_pre),
               static_cast<double>(out.p_vv_post),
               static_cast<double>(out.delta_p_vv),
               out.nhc_every_n_ticks,
               out.controller_state,
               out.transition ? 1 : 0,
               out.dwell_time_s,
               adaptive_nhc_transition_reason_name(out.reason))
        >= 0;
}
