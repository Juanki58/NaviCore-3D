#pragma once

#include "adaptive_nhc_controller.hpp"

#include <cstdint>

enum class RealRunMountMode : uint8_t {
    NONE = 0,
    LEGACY_EULER_H0 = 1,
    CALIBRATION_FILE = 2,
};

enum class RealRunYawInitMode : uint8_t {
    H0_ZERO_YAW = 0,
    H2_GNSS_STABLE_HEADING = 1,
    H3_COV_RESET_GRACE = 2,
};

/** ZUPT arming policy for replay (does not change filter math, only when ZUPT is armed). */
enum class ReplayConstraintPolicy : uint8_t {
    AUTO = 0,          /**< Calibration-aligned replay default (FORCED_TIME). */
    FORCED_TIME = 1,   /**< t <= static_phase_end OR gps_speed <= threshold (legacy). */
    GPS_STOP = 2,      /**< gps_speed <= moving_speed_threshold only. */
    IMU_STATIONARY = 3, /**< |‖a‖ − g| and ‖ω‖ below IMU thresholds. */
    DISABLED = 4,      /**< Never arm ZUPT. */
};

enum class ReplayNhcPolicy : uint8_t {
    ENABLED = 0,
    DISABLED = 1,
};

enum class ReplayGnssObsMode : uint8_t {
    POS = 0,
    POS_VEL = 1,
    VEL_ONLY = 2,
};

enum class ReplayPpvPolicy : uint8_t {
    NONE = 0,
    GAP_LE_1S = 1,
    ZERO = 2,
    COS_POS = 3,
    COS_TOT = 4,
};

struct ReplayConstraintDecision {
    bool zupt_armed;
    bool nhc_armed;
    ReplayConstraintPolicy policy;
};

const char *replay_constraint_policy_name(ReplayConstraintPolicy policy);
bool replay_parse_constraint_policy(const char *text, ReplayConstraintPolicy *out_policy);
bool replay_parse_nhc_policy(const char *text, ReplayNhcPolicy *out_policy);
const char *replay_gnss_obs_mode_name(ReplayGnssObsMode mode);
bool replay_parse_gnss_obs_mode(const char *text, ReplayGnssObsMode *out_mode);
const char *replay_p_pv_policy_name(ReplayPpvPolicy policy);
bool replay_parse_p_pv_policy(const char *text, ReplayPpvPolicy *out_policy);

struct RealRunReplayResult {
    uint32_t rows_processed;
    uint32_t imu_rows;
    uint32_t gps_rows;
    float duration_s;
    float final_drift_m;
    float last_gps_pos_n_m;
    float last_gps_pos_e_m;
    float last_gps_pos_d_m;
    bool filter_initialized;
    uint32_t gnss_accept_count;
    uint32_t gnss_reject_count;
    bool yaw_init_applied;
    float yaw_init_applied_at_s;
    float yaw_init_heading_deg;
    bool h3_applied;
    float h3_grace_period_end_s;
};

struct RealRunReplayConfig {
    const char *input_csv_path;
    const char *output_csv_path;
    const char *instrumentation_csv_path;
    const char *gnss_audit_csv_path;
    const char *h3_diagnostics_csv_path;
    const char *consistency_csv_path;
    const char *sync_audit_csv_path;
    const char *h7_update_audit_csv_path;
    const char *gap3_observation_audit_csv_path;
    const char *gap3_gnss_nis_audit_csv_path;
    const char *gap3_nhc_block_audit_csv_path;
    const char *gap3_gnss_k_block_audit_json_path;
    const char *gap3_cov_propagation_audit_csv_path;
    const char *gap3_cov_step_audit_csv_path;
    const char *gap3_vel_source_audit_csv_path;
    const char *gap3_imu_constraint_audit_csv_path;
    const char *gap3_constraint_pipeline_audit_csv_path;
    const char *h8_propagation_audit_csv_path;
    const char *h9_tilt_audit_csv_path;
    const char *h9a_gravity_alignment_audit_csv_path;
    const char *h9b_attitude_propagation_audit_csv_path;
    const char *h9d_gravity_subtraction_audit_csv_path;
    const char *propagation_chain_audit_csv_path;
    RealRunMountMode mount_mode;
    const char *mount_calibration_path;
    RealRunYawInitMode yaw_init_mode;
    float yaw_init_min_speed_mps;
    uint32_t yaw_init_min_samples;
    float yaw_init_max_heading_std_deg;
    float static_phase_end_s;
    float moving_speed_threshold_mps;
    ReplayConstraintPolicy constraint_policy;
    ReplayNhcPolicy nhc_policy;
    ReplayGnssObsMode gnss_obs_mode;
    ReplayPpvPolicy ppv_policy;
    float gnss_vel_std_mps;
    float imu_stationary_accel_dev_mps2;
    float imu_stationary_gyro_radps;
    float gravity_mps2;
    float zupt_lateral_std_mps;
    float zupt_vertical_std_mps;
    float nhc_lateral_std_mps;
    float nhc_vertical_std_mps;
    uint32_t progress_interval_rows;
    float p0_scale_factor;
    float q_scale_factor;
    float nhc_sigma_mps;
    bool nhc_sigma_overridden;
    uint32_t nhc_every_n_ticks;
    float gnss_ref_lat_deg;
    float gnss_ref_lon_deg;
    float gnss_ref_alt_m;
    bool gnss_ref_overridden;
    bool predict_only_mode;
    float replay_end_s;
    bool h9a_gravity_tilt_init;
    uint32_t h9a_gravity_init_min_samples;
    float h9a_gravity_init_window_s;
    AdaptiveNhcMode adaptive_nhc_mode;
    const char *adaptive_nhc_controller_audit_csv_path;
};

bool real_run_replay_load_mount_matrix(
    RealRunMountMode mode,
    const char *calibration_path,
    float out_matrix[3][3],
    char *out_label,
    size_t out_label_bytes);

bool real_run_replay_execute(const RealRunReplayConfig &config, RealRunReplayResult *out_result);
