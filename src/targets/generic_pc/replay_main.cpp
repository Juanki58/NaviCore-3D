#include "real_run_replay.hpp"

#include <cstdio>
#include <cstdlib>
#include <cstring>

namespace {

constexpr const char *kDefaultInputCsv = "docs/benchmarks/real_run_replay.csv";
constexpr const char *kDefaultOutputCsv = "docs/benchmarks/real_run_output.csv";
constexpr const char *kDefaultMountCalibration = "calibration/imu_mount.json";

constexpr const char *kDefaultH5SyncAuditCsv = "docs/benchmarks/h5_sync_audit.csv";

RealRunMountMode parse_mount_mode(const char *text)
{
    if (text == nullptr) {
        return RealRunMountMode::LEGACY_EULER_H0;
    }
    if (std::strcmp(text, "none") == 0) {
        return RealRunMountMode::NONE;
    }
    if (std::strcmp(text, "legacy") == 0 || std::strcmp(text, "legacy_euler") == 0) {
        return RealRunMountMode::LEGACY_EULER_H0;
    }
    if (std::strcmp(text, "calibration") == 0 || std::strcmp(text, "file") == 0) {
        return RealRunMountMode::CALIBRATION_FILE;
    }
    return RealRunMountMode::LEGACY_EULER_H0;
}

RealRunYawInitMode parse_yaw_init_mode(const char *text)
{
    if (text == nullptr) {
        return RealRunYawInitMode::H0_ZERO_YAW;
    }
    if (std::strcmp(text, "zero") == 0 || std::strcmp(text, "h0") == 0) {
        return RealRunYawInitMode::H0_ZERO_YAW;
    }
    if (std::strcmp(text, "gnss_stable") == 0
        || std::strcmp(text, "h2") == 0
        || std::strcmp(text, "stable_heading") == 0) {
        return RealRunYawInitMode::H2_GNSS_STABLE_HEADING;
    }
    if (std::strcmp(text, "h3") == 0
        || std::strcmp(text, "cov_reset_grace") == 0
        || std::strcmp(text, "manga_ancha") == 0) {
        return RealRunYawInitMode::H3_COV_RESET_GRACE;
    }
    return RealRunYawInitMode::H0_ZERO_YAW;
}

void print_usage(const char *program_name)
{
    std::printf(
        "Uso: %s [--input <csv>] [--output <csv>]\n"
        "         [--mount-mode none|legacy|calibration]\n"
        "         [--mount-calibration <json>]\n"
        "         [--yaw-init zero|gnss_stable|h3]\n"
        "         [--instrumentation-csv <csv>]\n"
        "         [--gnss-audit-csv <csv>]\n"
        "         [--h3-diagnostics-csv <csv>]\n"
        "         [--consistency-csv <csv>]\n"
        "         [--h5-sync-audit-csv <csv>]\n"
        "         [--h7-update-audit-csv <csv>]\n"
        "         [--gap3-observation-audit-csv <csv>]\n"
        "         [--gap3-gnss-nis-audit-csv <csv>]\n"
        "         [--gap3-nhc-block-audit-csv <csv>]\n"
        "         [--gap3-gnss-k-block-audit-json <json>]\n"
        "         [--gap3-cov-propagation-audit-csv <csv>]\n"
        "         [--gap3-cov-step-audit-csv <csv>]\n"
        "         [--gap3-vel-source-audit-csv <csv>]\n"
        "         [--gap3-imu-constraint-audit-csv <csv>]\n"
        "         [--gap3-constraint-pipeline-audit-csv <csv>]\n"
        "         [--constraint-policy auto|forced_time|gps_stop|imu_stationary|disabled]\n"
        "         [--nhc-policy enabled|disabled]\n"
        "         [--gnss-obs-mode pos|pos_vel|vel_only]\n"
        "         [--p-pv-policy none|gap_le_1s|zero|cos_pos|cos_tot]\n"
        "         [--gnss-vel-std-mps <m/s>]\n"
        "         [--nhc-every-n-ticks <N>]\n"
        "         [--static-phase-end-s <s>] [--moving-speed-threshold-mps <m/s>]\n"
        "         [--imu-stationary-accel-dev-mps2 <m/s2>] [--imu-stationary-gyro-radps <rad/s>]\n"
        "         [--h8-propagation-audit-csv <csv>]\n"
        "         [--h9-tilt-audit-csv <csv>]\n"
        "         [--predict-only] [--predict-only-end-s <s>]\n"
        "         [--replay-end-s <s>]\n"
        "         [--h9a-gravity-tilt-init]\n"
        "         [--h9a-gravity-alignment-audit-csv <csv>]\n"
        "         [--h9b-attitude-propagation-audit-csv <csv>]\n"
        "         [--h9d-gravity-subtraction-audit-csv <csv>]\n"
        "         [--propagation-chain-audit-csv <csv>]\n"
        "         [--h9a-gravity-init-min-samples <n>]\n"
        "         [--h9a-gravity-init-window-s <s>]\n"
        "         [--sync-audit-csv <csv>]\n"
        "         [--p0-scale <factor>]\n"
        "         [--q-scale <factor>]\n"
        "         [--nhc-sigma <m/s>]\n"
        "         [--gnss-ref-lat <deg>] [--gnss-ref-lon <deg>] [--gnss-ref-alt <m>]\n"
        "         [--yaw-init-min-speed <m/s>]\n"
        "         [--yaw-init-min-samples <n>]\n"
        "         [--yaw-init-max-heading-std-deg <deg>]\n"
        "\n"
        "Replay Engine — logs reales parseados (parse_mobile_log.py)\n"
        "  Entrada por defecto:       %s\n"
        "  Salida por defecto:        %s\n"
        "  Calibracion por defecto:   %s\n"
        "  Montaje por defecto:       legacy (H0)\n"
        "  Yaw inicial por defecto:   zero (H0)\n",
        program_name,
        kDefaultInputCsv,
        kDefaultOutputCsv,
        kDefaultMountCalibration);
}

} /* namespace */

int main(int argc, char *argv[])
{
    const char *input_csv = kDefaultInputCsv;
    const char *output_csv = kDefaultOutputCsv;
    const char *mount_calibration = kDefaultMountCalibration;
    const char *instrumentation_csv = nullptr;
    const char *gnss_audit_csv = nullptr;
    const char *h3_diagnostics_csv = nullptr;
    const char *consistency_csv = nullptr;
    const char *sync_audit_csv = nullptr;
    const char *h7_update_audit_csv = nullptr;
    const char *gap3_observation_audit_csv = nullptr;
    const char *gap3_gnss_nis_audit_csv = nullptr;
    const char *gap3_nhc_block_audit_csv = nullptr;
    const char *gap3_gnss_k_block_audit_json = nullptr;
    const char *gap3_cov_propagation_audit_csv = nullptr;
    const char *gap3_cov_step_audit_csv = nullptr;
    const char *gap3_vel_source_audit_csv = nullptr;
    const char *gap3_imu_constraint_audit_csv = nullptr;
    const char *gap3_constraint_pipeline_audit_csv = nullptr;
    const char *h8_propagation_audit_csv = nullptr;
    const char *h9_tilt_audit_csv = nullptr;
    bool predict_only_mode = false;
    float predict_only_end_s = 60.0f;
    float replay_end_s = 0.0f;
    bool h9a_gravity_tilt_init = false;
    const char *h9a_gravity_alignment_audit_csv = nullptr;
    const char *h9b_attitude_propagation_audit_csv = nullptr;
    const char *h9d_gravity_subtraction_audit_csv = nullptr;
    const char *propagation_chain_audit_csv = nullptr;
    uint32_t h9a_gravity_init_min_samples = 50U;
    float h9a_gravity_init_window_s = 2.0f;
    RealRunMountMode mount_mode = RealRunMountMode::LEGACY_EULER_H0;
    RealRunYawInitMode yaw_init_mode = RealRunYawInitMode::H0_ZERO_YAW;
    float yaw_init_min_speed_mps = 3.0f;
    uint32_t yaw_init_min_samples = 20U;
    float yaw_init_max_heading_std_deg = 5.0f;
    float p0_scale_factor = 1.0f;
    float q_scale_factor = 1.0f;
    float nhc_sigma_mps = -1.0f;
    float gnss_ref_lat_deg = 0.0f;
    float gnss_ref_lon_deg = 0.0f;
    float gnss_ref_alt_m = 0.0f;
    bool gnss_ref_lat_set = false;
    bool gnss_ref_lon_set = false;
    bool gnss_ref_alt_set = false;
    ReplayConstraintPolicy constraint_policy = ReplayConstraintPolicy::FORCED_TIME;
    ReplayNhcPolicy nhc_policy = ReplayNhcPolicy::ENABLED;
    ReplayGnssObsMode gnss_obs_mode = ReplayGnssObsMode::POS;
    ReplayPpvPolicy ppv_policy = ReplayPpvPolicy::NONE;
    float gnss_vel_std_mps = 0.0f;
    uint32_t nhc_every_n_ticks = 1U;
    float static_phase_end_s = 30.0f;
    float moving_speed_threshold_mps = 0.1f;
    float imu_stationary_accel_dev_mps2 = 0.5f;
    float imu_stationary_gyro_radps = 0.05f;
    float gravity_mps2 = 9.80665f;

    for (int i = 1; i < argc; ++i) {
        if (std::strcmp(argv[i], "--input") == 0) {
            if (i + 1 >= argc) {
                print_usage(argv[0]);
                return 1;
            }
            input_csv = argv[++i];
        } else if (std::strcmp(argv[i], "--output") == 0) {
            if (i + 1 >= argc) {
                print_usage(argv[0]);
                return 1;
            }
            output_csv = argv[++i];
        } else if (std::strcmp(argv[i], "--mount-mode") == 0) {
            if (i + 1 >= argc) {
                print_usage(argv[0]);
                return 1;
            }
            mount_mode = parse_mount_mode(argv[++i]);
        } else if (std::strcmp(argv[i], "--mount-calibration") == 0) {
            if (i + 1 >= argc) {
                print_usage(argv[0]);
                return 1;
            }
            mount_calibration = argv[++i];
        } else if (std::strcmp(argv[i], "--yaw-init") == 0) {
            if (i + 1 >= argc) {
                print_usage(argv[0]);
                return 1;
            }
            yaw_init_mode = parse_yaw_init_mode(argv[++i]);
        } else if (std::strcmp(argv[i], "--instrumentation-csv") == 0) {
            if (i + 1 >= argc) {
                print_usage(argv[0]);
                return 1;
            }
            instrumentation_csv = argv[++i];
        } else if (std::strcmp(argv[i], "--gnss-audit-csv") == 0) {
            if (i + 1 >= argc) {
                print_usage(argv[0]);
                return 1;
            }
            gnss_audit_csv = argv[++i];
        } else if (std::strcmp(argv[i], "--h3-diagnostics-csv") == 0) {
            if (i + 1 >= argc) {
                print_usage(argv[0]);
                return 1;
            }
            h3_diagnostics_csv = argv[++i];
        } else if (std::strcmp(argv[i], "--consistency-csv") == 0) {
            if (i + 1 >= argc) {
                print_usage(argv[0]);
                return 1;
            }
            consistency_csv = argv[++i];
        } else if (std::strcmp(argv[i], "--h5-sync-audit-csv") == 0
            || std::strcmp(argv[i], "--sync-audit-csv") == 0) {
            if (i + 1 >= argc) {
                print_usage(argv[0]);
                return 1;
            }
            sync_audit_csv = argv[++i];
        } else if (std::strcmp(argv[i], "--h7-update-audit-csv") == 0) {
            if (i + 1 >= argc) {
                print_usage(argv[0]);
                return 1;
            }
            h7_update_audit_csv = argv[++i];
        } else if (std::strcmp(argv[i], "--gap3-observation-audit-csv") == 0) {
            if (i + 1 >= argc) {
                print_usage(argv[0]);
                return 1;
            }
            gap3_observation_audit_csv = argv[++i];
        } else if (std::strcmp(argv[i], "--gap3-gnss-nis-audit-csv") == 0) {
            if (i + 1 >= argc) {
                print_usage(argv[0]);
                return 1;
            }
            gap3_gnss_nis_audit_csv = argv[++i];
        } else if (std::strcmp(argv[i], "--gap3-nhc-block-audit-csv") == 0) {
            if (i + 1 >= argc) {
                print_usage(argv[0]);
                return 1;
            }
            gap3_nhc_block_audit_csv = argv[++i];
        } else if (std::strcmp(argv[i], "--gap3-gnss-k-block-audit-json") == 0) {
            if (i + 1 >= argc) {
                print_usage(argv[0]);
                return 1;
            }
            gap3_gnss_k_block_audit_json = argv[++i];
        } else if (std::strcmp(argv[i], "--gap3-cov-propagation-audit-csv") == 0) {
            if (i + 1 >= argc) {
                print_usage(argv[0]);
                return 1;
            }
            gap3_cov_propagation_audit_csv = argv[++i];
        } else if (std::strcmp(argv[i], "--gap3-cov-step-audit-csv") == 0) {
            if (i + 1 >= argc) {
                print_usage(argv[0]);
                return 1;
            }
            gap3_cov_step_audit_csv = argv[++i];
        } else if (std::strcmp(argv[i], "--gap3-vel-source-audit-csv") == 0) {
            if (i + 1 >= argc) {
                print_usage(argv[0]);
                return 1;
            }
            gap3_vel_source_audit_csv = argv[++i];
        } else if (std::strcmp(argv[i], "--gap3-imu-constraint-audit-csv") == 0) {
            if (i + 1 >= argc) {
                print_usage(argv[0]);
                return 1;
            }
            gap3_imu_constraint_audit_csv = argv[++i];
        } else if (std::strcmp(argv[i], "--gap3-constraint-pipeline-audit-csv") == 0) {
            if (i + 1 >= argc) {
                print_usage(argv[0]);
                return 1;
            }
            gap3_constraint_pipeline_audit_csv = argv[++i];
        } else if (std::strcmp(argv[i], "--constraint-policy") == 0) {
            if (i + 1 >= argc) {
                print_usage(argv[0]);
                return 1;
            }
            if (!replay_parse_constraint_policy(argv[++i], &constraint_policy)) {
                std::printf("ERROR: --constraint-policy invalido: %s\n", argv[i]);
                return 1;
            }
        } else if (std::strcmp(argv[i], "--nhc-policy") == 0) {
            if (i + 1 >= argc) {
                print_usage(argv[0]);
                return 1;
            }
            if (!replay_parse_nhc_policy(argv[++i], &nhc_policy)) {
                std::printf("ERROR: --nhc-policy invalido: %s\n", argv[i]);
                return 1;
            }
        } else if (std::strcmp(argv[i], "--gnss-obs-mode") == 0) {
            if (i + 1 >= argc) {
                print_usage(argv[0]);
                return 1;
            }
            if (!replay_parse_gnss_obs_mode(argv[++i], &gnss_obs_mode)) {
                std::printf("ERROR: --gnss-obs-mode invalido: %s\n", argv[i]);
                return 1;
            }
        } else if (std::strcmp(argv[i], "--p-pv-policy") == 0) {
            if (i + 1 >= argc) {
                print_usage(argv[0]);
                return 1;
            }
            if (!replay_parse_p_pv_policy(argv[++i], &ppv_policy)) {
                std::printf("ERROR: --p-pv-policy invalido: %s\n", argv[i]);
                return 1;
            }
        } else if (std::strcmp(argv[i], "--gnss-vel-std-mps") == 0) {
            if (i + 1 >= argc) {
                print_usage(argv[0]);
                return 1;
            }
            gnss_vel_std_mps = static_cast<float>(std::atof(argv[++i]));
        } else if (std::strcmp(argv[i], "--nhc-every-n-ticks") == 0) {
            if (i + 1 >= argc) {
                print_usage(argv[0]);
                return 1;
            }
            nhc_every_n_ticks = static_cast<uint32_t>(std::strtoul(argv[++i], nullptr, 10));
            if (nhc_every_n_ticks == 0U) {
                nhc_every_n_ticks = 1U;
            }
        } else if (std::strcmp(argv[i], "--static-phase-end-s") == 0) {
            if (i + 1 >= argc) {
                print_usage(argv[0]);
                return 1;
            }
            static_phase_end_s = static_cast<float>(std::atof(argv[++i]));
        } else if (std::strcmp(argv[i], "--moving-speed-threshold-mps") == 0) {
            if (i + 1 >= argc) {
                print_usage(argv[0]);
                return 1;
            }
            moving_speed_threshold_mps = static_cast<float>(std::atof(argv[++i]));
        } else if (std::strcmp(argv[i], "--imu-stationary-accel-dev-mps2") == 0) {
            if (i + 1 >= argc) {
                print_usage(argv[0]);
                return 1;
            }
            imu_stationary_accel_dev_mps2 = static_cast<float>(std::atof(argv[++i]));
        } else if (std::strcmp(argv[i], "--imu-stationary-gyro-radps") == 0) {
            if (i + 1 >= argc) {
                print_usage(argv[0]);
                return 1;
            }
            imu_stationary_gyro_radps = static_cast<float>(std::atof(argv[++i]));
        } else if (std::strcmp(argv[i], "--h8-propagation-audit-csv") == 0) {
            if (i + 1 >= argc) {
                print_usage(argv[0]);
                return 1;
            }
            h8_propagation_audit_csv = argv[++i];
        } else if (std::strcmp(argv[i], "--h9-tilt-audit-csv") == 0) {
            if (i + 1 >= argc) {
                print_usage(argv[0]);
                return 1;
            }
            h9_tilt_audit_csv = argv[++i];
        } else if (std::strcmp(argv[i], "--predict-only") == 0) {
            predict_only_mode = true;
        } else if (std::strcmp(argv[i], "--predict-only-end-s") == 0) {
            if (i + 1 >= argc) {
                print_usage(argv[0]);
                return 1;
            }
            predict_only_end_s = static_cast<float>(std::atof(argv[++i]));
        } else if (std::strcmp(argv[i], "--replay-end-s") == 0) {
            if (i + 1 >= argc) {
                print_usage(argv[0]);
                return 1;
            }
            replay_end_s = static_cast<float>(std::atof(argv[++i]));
        } else if (std::strcmp(argv[i], "--h9a-gravity-tilt-init") == 0) {
            h9a_gravity_tilt_init = true;
        } else if (std::strcmp(argv[i], "--h9a-gravity-alignment-audit-csv") == 0) {
            if (i + 1 >= argc) {
                print_usage(argv[0]);
                return 1;
            }
            h9a_gravity_alignment_audit_csv = argv[++i];
        } else if (std::strcmp(argv[i], "--h9b-attitude-propagation-audit-csv") == 0) {
            if (i + 1 >= argc) {
                print_usage(argv[0]);
                return 1;
            }
            h9b_attitude_propagation_audit_csv = argv[++i];
        } else if (std::strcmp(argv[i], "--h9d-gravity-subtraction-audit-csv") == 0) {
            if (i + 1 >= argc) {
                print_usage(argv[0]);
                return 1;
            }
            h9d_gravity_subtraction_audit_csv = argv[++i];
        } else if (std::strcmp(argv[i], "--propagation-chain-audit-csv") == 0) {
            if (i + 1 >= argc) {
                print_usage(argv[0]);
                return 1;
            }
            propagation_chain_audit_csv = argv[++i];
        } else if (std::strcmp(argv[i], "--h9a-gravity-init-min-samples") == 0) {
            if (i + 1 >= argc) {
                print_usage(argv[0]);
                return 1;
            }
            h9a_gravity_init_min_samples = static_cast<uint32_t>(std::strtoul(argv[++i], nullptr, 10));
        } else if (std::strcmp(argv[i], "--h9a-gravity-init-window-s") == 0) {
            if (i + 1 >= argc) {
                print_usage(argv[0]);
                return 1;
            }
            h9a_gravity_init_window_s = static_cast<float>(std::atof(argv[++i]));
        } else if (std::strcmp(argv[i], "--p0-scale") == 0) {
            if (i + 1 >= argc) {
                print_usage(argv[0]);
                return 1;
            }
            p0_scale_factor = static_cast<float>(std::atof(argv[++i]));
        } else if (std::strcmp(argv[i], "--q-scale") == 0) {
            if (i + 1 >= argc) {
                print_usage(argv[0]);
                return 1;
            }
            q_scale_factor = static_cast<float>(std::atof(argv[++i]));
        } else if (std::strcmp(argv[i], "--nhc-sigma") == 0) {
            if (i + 1 >= argc) {
                print_usage(argv[0]);
                return 1;
            }
            nhc_sigma_mps = static_cast<float>(std::atof(argv[++i]));
        } else if (std::strcmp(argv[i], "--gnss-ref-lat") == 0) {
            if (i + 1 >= argc) {
                print_usage(argv[0]);
                return 1;
            }
            gnss_ref_lat_deg = static_cast<float>(std::atof(argv[++i]));
            gnss_ref_lat_set = true;
        } else if (std::strcmp(argv[i], "--gnss-ref-lon") == 0) {
            if (i + 1 >= argc) {
                print_usage(argv[0]);
                return 1;
            }
            gnss_ref_lon_deg = static_cast<float>(std::atof(argv[++i]));
            gnss_ref_lon_set = true;
        } else if (std::strcmp(argv[i], "--gnss-ref-alt") == 0) {
            if (i + 1 >= argc) {
                print_usage(argv[0]);
                return 1;
            }
            gnss_ref_alt_m = static_cast<float>(std::atof(argv[++i]));
            gnss_ref_alt_set = true;
        } else if (std::strcmp(argv[i], "--yaw-init-min-speed") == 0) {
            if (i + 1 >= argc) {
                print_usage(argv[0]);
                return 1;
            }
            yaw_init_min_speed_mps = static_cast<float>(std::atof(argv[++i]));
        } else if (std::strcmp(argv[i], "--yaw-init-min-samples") == 0) {
            if (i + 1 >= argc) {
                print_usage(argv[0]);
                return 1;
            }
            yaw_init_min_samples = static_cast<uint32_t>(std::strtoul(argv[++i], nullptr, 10));
        } else if (std::strcmp(argv[i], "--yaw-init-max-heading-std-deg") == 0) {
            if (i + 1 >= argc) {
                print_usage(argv[0]);
                return 1;
            }
            yaw_init_max_heading_std_deg = static_cast<float>(std::atof(argv[++i]));
        } else if (std::strcmp(argv[i], "--help") == 0 || std::strcmp(argv[i], "-h") == 0) {
            print_usage(argv[0]);
            return 0;
        } else {
            std::printf("ERROR: argumento desconocido: %s\n", argv[i]);
            print_usage(argv[0]);
            return 1;
        }
    }

    const bool gnss_ref_partial =
        gnss_ref_lat_set || gnss_ref_lon_set || gnss_ref_alt_set;
    if (gnss_ref_partial
        && !(gnss_ref_lat_set && gnss_ref_lon_set && gnss_ref_alt_set)) {
        std::printf(
            "ERROR: --gnss-ref-lat/lon/alt deben usarse juntos (origen LLA completo)\n");
        return 1;
    }

    RealRunReplayConfig config{};
    config.input_csv_path = input_csv;
    config.output_csv_path = output_csv;
    config.instrumentation_csv_path = instrumentation_csv;
    config.gnss_audit_csv_path = gnss_audit_csv;
    config.h3_diagnostics_csv_path = h3_diagnostics_csv;
    config.consistency_csv_path = consistency_csv;
    config.sync_audit_csv_path = sync_audit_csv;
    config.h7_update_audit_csv_path = h7_update_audit_csv;
    config.gap3_observation_audit_csv_path = gap3_observation_audit_csv;
    config.gap3_gnss_nis_audit_csv_path = gap3_gnss_nis_audit_csv;
    config.gap3_nhc_block_audit_csv_path = gap3_nhc_block_audit_csv;
    config.gap3_gnss_k_block_audit_json_path = gap3_gnss_k_block_audit_json;
    config.gap3_cov_propagation_audit_csv_path = gap3_cov_propagation_audit_csv;
    config.gap3_cov_step_audit_csv_path = gap3_cov_step_audit_csv;
    config.gap3_vel_source_audit_csv_path = gap3_vel_source_audit_csv;
    config.gap3_imu_constraint_audit_csv_path = gap3_imu_constraint_audit_csv;
    config.gap3_constraint_pipeline_audit_csv_path = gap3_constraint_pipeline_audit_csv;
    config.h8_propagation_audit_csv_path = h8_propagation_audit_csv;
    config.h9_tilt_audit_csv_path = h9_tilt_audit_csv;
    config.predict_only_mode = predict_only_mode;
    config.replay_end_s = predict_only_mode ? predict_only_end_s : replay_end_s;
    config.h9a_gravity_tilt_init = h9a_gravity_tilt_init;
    config.h9a_gravity_alignment_audit_csv_path = h9a_gravity_alignment_audit_csv;
    config.h9b_attitude_propagation_audit_csv_path = h9b_attitude_propagation_audit_csv;
    config.h9d_gravity_subtraction_audit_csv_path = h9d_gravity_subtraction_audit_csv;
    config.propagation_chain_audit_csv_path = propagation_chain_audit_csv;
    config.h9a_gravity_init_min_samples = h9a_gravity_init_min_samples;
    config.h9a_gravity_init_window_s = h9a_gravity_init_window_s;
    config.mount_mode = mount_mode;
    config.mount_calibration_path = mount_calibration;
    config.yaw_init_mode = yaw_init_mode;
    config.yaw_init_min_speed_mps = yaw_init_min_speed_mps;
    config.yaw_init_min_samples = yaw_init_min_samples;
    config.yaw_init_max_heading_std_deg = yaw_init_max_heading_std_deg;
    config.p0_scale_factor = p0_scale_factor;
    config.q_scale_factor = q_scale_factor;
    config.gnss_ref_lat_deg = gnss_ref_lat_deg;
    config.gnss_ref_lon_deg = gnss_ref_lon_deg;
    config.gnss_ref_alt_m = gnss_ref_alt_m;
    config.gnss_ref_overridden = gnss_ref_lat_set && gnss_ref_lon_set && gnss_ref_alt_set;
    config.static_phase_end_s = static_phase_end_s;
    config.moving_speed_threshold_mps = moving_speed_threshold_mps;
    config.constraint_policy = constraint_policy;
    config.nhc_policy = nhc_policy;
    config.gnss_obs_mode = gnss_obs_mode;
    config.ppv_policy = ppv_policy;
    config.gnss_vel_std_mps = gnss_vel_std_mps;
    config.nhc_every_n_ticks = nhc_every_n_ticks;
    config.imu_stationary_accel_dev_mps2 = imu_stationary_accel_dev_mps2;
    config.imu_stationary_gyro_radps = imu_stationary_gyro_radps;
    config.gravity_mps2 = gravity_mps2;
    config.zupt_lateral_std_mps = 0.1f;
    config.zupt_vertical_std_mps = 0.1f;
    config.nhc_lateral_std_mps = 0.5f;
    config.nhc_vertical_std_mps = 1.0f;
    config.nhc_sigma_overridden = nhc_sigma_mps >= 0.0f;
    if (config.nhc_sigma_overridden) {
        config.nhc_sigma_mps = nhc_sigma_mps;
        config.nhc_lateral_std_mps = nhc_sigma_mps;
        config.nhc_vertical_std_mps = nhc_sigma_mps;
    } else {
        config.nhc_sigma_mps = config.nhc_lateral_std_mps;
    }
    config.progress_interval_rows = 1000U;

    RealRunReplayResult result{};
    const bool ok = real_run_replay_execute(config, &result);
    return ok ? 0 : 1;
}
