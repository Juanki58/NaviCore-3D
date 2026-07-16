#include "regression_suite.hpp"

#include "ins_ekf.hpp"
#include "constant_slope_benchmark.hpp"
#include "slalom_benchmark.hpp"
#include "super_tunnel_benchmark.hpp"

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <ctime>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace {

constexpr int kRegressionTestCount = 7;
constexpr int kMaxRegressionReports = 8;
constexpr const char *kRegressionReportPath = "docs/regression_report.json";

int g_failures = 0;
int g_report_count = 0;

struct SimpleTestMetrics {
    bool valid;
    float value;
    const char *unit;
    const char *label;
};

struct SuperTunnelCaseMetrics {
    bool valid;
    bool nhc_enabled;
    SuperTunnelPassResult result;
};

struct ConstantSlopeCaseMetrics {
    bool valid;
    bool nhc_enabled;
    ConstantSlopePassResult result;
};

struct SlalomCaseMetrics {
    bool valid;
    bool nhc_enabled;
    SlalomPassResult result;
};

struct RegressionTestReport {
    char name[64];
    bool passed;
    bool has_simple_metrics;
    SimpleTestMetrics simple_metrics[4];
    int simple_metric_count;
    bool has_super_tunnel_cases;
    SuperTunnelCaseMetrics super_tunnel_cases[2];
    int super_tunnel_case_count;
    bool has_constant_slope_case;
    ConstantSlopeCaseMetrics constant_slope_case;
    bool has_slalom_case;
    SlalomCaseMetrics slalom_case;
};

RegressionTestReport g_reports[kMaxRegressionReports];

void expect_true(bool condition, const char *message)
{
    if (!condition) {
        std::printf("  FAIL: %s\n", message);
        ++g_failures;
    }
}

void expect_less(float actual, float limit, const char *message)
{
    if (!(actual < limit)) {
        std::printf(
            "  FAIL: %s (actual=%.4f limit=%.4f)\n",
            message,
            actual,
            limit);
        ++g_failures;
    }
}

void expect_greater(float actual, float limit, const char *message)
{
    if (!(actual > limit)) {
        std::printf(
            "  FAIL: %s (actual=%.4f limit=%.4f)\n",
            message,
            actual,
            limit);
        ++g_failures;
    }
}

void expect_near(float actual, float expected, float tolerance, const char *message)
{
    const float err = std::fabs(actual - expected);
    if (err > tolerance) {
        std::printf(
            "  FAIL: %s (actual=%.6f expected=%.6f |err|=%.6f tol=%.6f)\n",
            message,
            actual,
            expected,
            err,
            tolerance);
        ++g_failures;
    }
}

ImuSample make_ideal_stationary_imu(uint32_t timestamp_ms)
{
    ImuSample imu{};
    imu.accel_mps2[0] = 0.0f;
    imu.accel_mps2[1] = 0.0f;
    imu.accel_mps2[2] = 9.80665f;
    imu.gyro_radps[0] = 0.0f;
    imu.gyro_radps[1] = 0.0f;
    imu.gyro_radps[2] = 0.0f;
    imu.timestamp_ms = timestamp_ms;
    imu.valid = true;
    return imu;
}

RegressionTestReport *begin_test_report(const char *name)
{
    if (g_report_count >= kMaxRegressionReports) {
        return NULL;
    }

    RegressionTestReport *report = &g_reports[g_report_count++];
    std::snprintf(report->name, sizeof(report->name), "%s", name);
    report->passed = true;
    report->has_simple_metrics = false;
    report->simple_metric_count = 0;
    report->has_super_tunnel_cases = false;
    report->super_tunnel_case_count = 0;
    report->has_constant_slope_case = false;
    report->constant_slope_case.valid = false;
    report->has_slalom_case = false;
    report->slalom_case.valid = false;
    return report;
}

void add_simple_metric(
    RegressionTestReport *report,
    const char *label,
    float value,
    const char *unit)
{
    if (report == NULL || report->simple_metric_count >= 4) {
        return;
    }

    SimpleTestMetrics *metric = &report->simple_metrics[report->simple_metric_count++];
    metric->valid = true;
    metric->label = label;
    metric->value = value;
    metric->unit = unit;
    report->has_simple_metrics = true;
}

void add_super_tunnel_case(
    RegressionTestReport *report,
    bool nhc_enabled,
    const SuperTunnelPassResult *result)
{
    if (report == NULL || result == NULL || report->super_tunnel_case_count >= 2) {
        return;
    }

    SuperTunnelCaseMetrics *entry = &report->super_tunnel_cases[report->super_tunnel_case_count++];
    entry->valid = true;
    entry->nhc_enabled = nhc_enabled;
    entry->result = *result;
    report->has_super_tunnel_cases = true;
}

void add_constant_slope_case(
    RegressionTestReport *report,
    bool nhc_enabled,
    const ConstantSlopePassResult *result)
{
    if (report == NULL || result == NULL) {
        return;
    }

    report->constant_slope_case.valid = true;
    report->constant_slope_case.nhc_enabled = nhc_enabled;
    report->constant_slope_case.result = *result;
    report->has_constant_slope_case = true;
}

void add_slalom_case(
    RegressionTestReport *report,
    bool nhc_enabled,
    const SlalomPassResult *result)
{
    if (report == NULL || result == NULL) {
        return;
    }

    report->slalom_case.valid = true;
    report->slalom_case.nhc_enabled = nhc_enabled;
    report->slalom_case.result = *result;
    report->has_slalom_case = true;
}

void run_test(const char *name, void (*body)(RegressionTestReport *report))
{
    const int before = g_failures;
    std::printf("[TEST] %s\n", name);
    RegressionTestReport *report = begin_test_report(name);
    body(report);
    if (report != NULL) {
        report->passed = (g_failures == before);
    }
    if (g_failures == before) {
        std::printf("  PASS\n");
    }
}

void test_ins_ekf_gravity_compensation(RegressionTestReport *report)
{
    const Vector3D origin = vector3d_make(41.3874f, 2.1686f, 12.0f);
    InsEkfFilter ekf{};
    ins_ekf_init(&ekf, origin, 0.0f, NAVICORE_DOMAIN_AIR);

    for (uint32_t t_ms = 10U; t_ms <= 1000U; t_ms += 10U) {
        ImuSample imu = make_ideal_stationary_imu(t_ms);
        expect_true(ins_ekf_predict(&ekf, &imu), "predict IMU valido en reposo");
    }

    float vel_ned[3] = {0.0f, 0.0f, 0.0f};
    ins_ekf_get_velocity_ned(&ekf, vel_ned);
    const float speed_mps = std::sqrt(
        (vel_ned[0] * vel_ned[0]) + (vel_ned[1] * vel_ned[1]) + (vel_ned[2] * vel_ned[2]));
    expect_less(speed_mps, 0.5f, "velocidad tras 1 s en reposo con IMU ideal");
    add_simple_metric(report, "final_speed_mps", speed_mps, "m/s");
}

void test_nhc_enabled_predict_increments_counter(RegressionTestReport *report)
{
    const Vector3D origin = vector3d_make(41.3874f, 2.1686f, 12.0f);
    InsEkfFilter ekf{};
    ins_ekf_init(&ekf, origin, static_cast<float>(M_PI * 0.5), NAVICORE_DOMAIN_AIR);
    ekf.vel_[0] = 0.0f;
    ekf.vel_[1] = 25.0f;
    ekf.vel_[2] = 0.0f;
    ins_ekf_set_nhc_enabled(&ekf, true);

    for (uint32_t t_ms = 10U; t_ms <= 100U; t_ms += 10U) {
        ImuSample imu = make_ideal_stationary_imu(t_ms);
        (void)ins_ekf_predict(&ekf, &imu);
    }

    const uint32_t nhc_updates = ins_ekf_nhc_update_count(&ekf);
    float innov_lat = 0.0f;
    float innov_vert = 0.0f;
    float innov_norm = 0.0f;
    ins_ekf_get_nhc_innovation_max(&ekf, &innov_lat, &innov_vert, &innov_norm);

    expect_greater(
        static_cast<float>(nhc_updates),
        5.0f,
        "contador NHC incrementa con prediccion IMU");
    add_simple_metric(report, "nhc_updates", static_cast<float>(nhc_updates), "count");
    add_simple_metric(report, "nhc_innovation_max_lateral_mps", innov_lat, "m/s");
    add_simple_metric(report, "nhc_innovation_max_vertical_mps", innov_vert, "m/s");
    add_simple_metric(report, "nhc_innovation_max_norm_mps", innov_norm, "m/s");
}

void test_nhc_jacobian_fd(RegressionTestReport *report)
{
    const float yaw_rad = static_cast<float>(M_PI * 0.5);
    const float vel_ned[3] = {12.0f, 25.0f, -0.5f};

    float q[4] = {1.0f, 0.0f, 0.0f, 0.0f};
    q[0] = std::cos(yaw_rad * 0.5f);
    q[3] = std::sin(yaw_rad * 0.5f);

    float dcm[3][3]{};
    ins_ekf_kinematics_quat_to_dcm_bn(q, dcm);

    float v_body[3]{};
    ins_ekf_kinematics_ned_to_body(dcm, vel_ned, v_body);

    float h_analytic[2][3]{};
    ins_ekf_fill_nhc_attitude_coupling(v_body, h_analytic);

    constexpr float eps = 1.0e-4f;
    constexpr float tol = 5.0e-3f;
    float max_abs_err = 0.0f;

    for (uint8_t axis = 0U; axis < 3U; ++axis) {
        float dtheta_p[3] = {0.0f, 0.0f, 0.0f};
        float dtheta_m[3] = {0.0f, 0.0f, 0.0f};
        dtheta_p[axis] = eps;
        dtheta_m[axis] = -eps;

        float q_p[4] = {q[0], q[1], q[2], q[3]};
        float q_m[4] = {q[0], q[1], q[2], q[3]};
        ins_ekf_kinematics_quat_apply_small_angle_error(q_p, dtheta_p);
        ins_ekf_kinematics_quat_apply_small_angle_error(q_m, dtheta_m);

        float dcm_p[3][3]{};
        float dcm_m[3][3]{};
        float vb_p[3]{};
        float vb_m[3]{};
        ins_ekf_kinematics_quat_to_dcm_bn(q_p, dcm_p);
        ins_ekf_kinematics_quat_to_dcm_bn(q_m, dcm_m);
        ins_ekf_kinematics_ned_to_body(dcm_p, vel_ned, vb_p);
        ins_ekf_kinematics_ned_to_body(dcm_m, vel_ned, vb_m);

        const float y_nom[2] = {-v_body[1], -v_body[2]};
        const float y_p[2] = {-vb_p[1], -vb_p[2]};
        const float y_m[2] = {-vb_m[1], -vb_m[2]};

        for (uint8_t meas = 0U; meas < 2U; ++meas) {
            const float fd = (y_p[meas] - y_m[meas]) / (2.0f * eps);
            const float analytic = h_analytic[meas][axis];
            const float abs_err = std::fabs(fd - analytic);
            if (abs_err > max_abs_err) {
                max_abs_err = abs_err;
            }
            expect_near(fd, analytic, tol, "NHC Jacobian FD vs analitico");
        }
    }

    add_simple_metric(report, "nhc_jacobian_max_abs_err", max_abs_err, "1");
    expect_less(max_abs_err, tol, "max error Jacobian NHC acoplamiento actitud");
}

void test_super_tunnel_nhc_isolation(RegressionTestReport *report)
{
    constexpr uint32_t kDiagSeed = 424242U;

    const SuperTunnelPassResult dirty_off = super_tunnel_run_pass(
        false, false, SUPER_TUNNEL_IMU_DIRTY_FULL, kDiagSeed);
    const SuperTunnelPassResult dirty_on = super_tunnel_run_pass(
        true, false, SUPER_TUNNEL_IMU_DIRTY_FULL, kDiagSeed);
    const SuperTunnelPassResult no_sm_off = super_tunnel_run_pass(
        false, false, SUPER_TUNNEL_IMU_NO_SCALE_MISALIGN, kDiagSeed);
    const SuperTunnelPassResult no_sm_on = super_tunnel_run_pass(
        true, false, SUPER_TUNNEL_IMU_NO_SCALE_MISALIGN, kDiagSeed);
    const SuperTunnelPassResult ideal_off = super_tunnel_run_pass(
        false, false, SUPER_TUNNEL_IMU_IDEAL, kDiagSeed);
    const SuperTunnelPassResult ideal_on = super_tunnel_run_pass(
        true, false, SUPER_TUNNEL_IMU_IDEAL, kDiagSeed);
    const SuperTunnelPassResult ideal_on_loose_r = super_tunnel_run_pass(
        true,
        false,
        SUPER_TUNNEL_IMU_IDEAL,
        kDiagSeed,
        5.0f,
        1.0f);

    std::printf(
        "  aislamiento super_tunnel (seed=%u, Jacobiano NHC corregido):\n"
        "    dirty      sin NHC exit=%.2f m | con NHC exit=%.2f m | innov_max=%.3f m/s\n"
        "    no_scale   sin NHC exit=%.2f m | con NHC exit=%.2f m | innov_max=%.3f m/s\n"
        "    ideal      sin NHC exit=%.2f m | con NHC exit=%.2f m | innov_max=%.3f m/s\n",
        kDiagSeed,
        dirty_off.drift_exit_tunnel_m,
        dirty_on.drift_exit_tunnel_m,
        dirty_on.nhc_innovation_max.norm_mps,
        no_sm_off.drift_exit_tunnel_m,
        no_sm_on.drift_exit_tunnel_m,
        no_sm_on.nhc_innovation_max.norm_mps,
        ideal_off.drift_exit_tunnel_m,
        ideal_on.drift_exit_tunnel_m,
        ideal_on.nhc_innovation_max.norm_mps);
    std::printf(
        "    ideal+R5    con NHC exit=%.2f m | innov_max=%.3f m/s (R_lat=5 m/s)\n",
        ideal_on_loose_r.drift_exit_tunnel_m,
        ideal_on_loose_r.nhc_innovation_max.norm_mps);

    add_simple_metric(report, "dirty_nhc_off_exit_m", dirty_off.drift_exit_tunnel_m, "m");
    add_simple_metric(report, "dirty_nhc_on_exit_m", dirty_on.drift_exit_tunnel_m, "m");
    add_simple_metric(report, "no_scale_nhc_off_exit_m", no_sm_off.drift_exit_tunnel_m, "m");
    add_simple_metric(report, "no_scale_nhc_on_exit_m", no_sm_on.drift_exit_tunnel_m, "m");
    add_simple_metric(report, "ideal_nhc_off_exit_m", ideal_off.drift_exit_tunnel_m, "m");
    add_simple_metric(report, "ideal_nhc_on_exit_m", ideal_on.drift_exit_tunnel_m, "m");
    add_simple_metric(report, "ideal_loose_r_exit_m", ideal_on_loose_r.drift_exit_tunnel_m, "m");

    expect_less(
        ideal_on.drift_exit_tunnel_m,
        dirty_on.drift_exit_tunnel_m,
        "IMU ideal: NHC no debe empeorar vs dirty+NHC");

    const float nhc_penalty_dirty = dirty_on.drift_exit_tunnel_m - dirty_off.drift_exit_tunnel_m;
    const float nhc_penalty_ideal = ideal_on.drift_exit_tunnel_m - ideal_off.drift_exit_tunnel_m;
    const float nhc_penalty_no_sm = no_sm_on.drift_exit_tunnel_m - no_sm_off.drift_exit_tunnel_m;
    add_simple_metric(report, "nhc_penalty_dirty_m", nhc_penalty_dirty, "m");
    add_simple_metric(report, "nhc_penalty_ideal_m", nhc_penalty_ideal, "m");

    std::printf(
        "  penalidad NHC (con - sin): dirty=%.2f m | ideal=%.2f m | no_scale=%.2f m\n",
        nhc_penalty_dirty,
        nhc_penalty_ideal,
        nhc_penalty_no_sm);

    /* Refutacion hipotesis escala/desalineacion: penalidad similar en los tres modos. */
    expect_less(
        std::fabs(nhc_penalty_ideal - nhc_penalty_dirty),
        nhc_penalty_dirty * 0.15f,
        "penalidad NHC casi igual dirty vs ideal (no es escala/desalineacion)");
    expect_less(
        std::fabs(nhc_penalty_no_sm - nhc_penalty_dirty),
        nhc_penalty_dirty * 0.15f,
        "penalidad NHC casi igual dirty vs no_scale");
}

void test_super_tunnel_nhc_regression(RegressionTestReport *report)
{
    const SuperTunnelPassResult without_nhc = super_tunnel_run_pass(false, false);
    const SuperTunnelPassResult with_nhc = super_tunnel_run_pass(true, false);

    add_super_tunnel_case(report, false, &without_nhc);
    add_super_tunnel_case(report, true, &with_nhc);

    std::printf(
        "  metricas: sin_nhc_exit=%.2f con_nhc_exit=%.2f rms_pos=%.2f/%.2f rms_vel=%.2f/%.2f rms_yaw=%.2f/%.2f\n",
        without_nhc.drift_exit_tunnel_m,
        with_nhc.drift_exit_tunnel_m,
        without_nhc.outage_rms.position_m,
        with_nhc.outage_rms.position_m,
        without_nhc.outage_rms.velocity_mps,
        with_nhc.outage_rms.velocity_mps,
        without_nhc.outage_rms.yaw_deg,
        with_nhc.outage_rms.yaw_deg);

    expect_less(without_nhc.drift_exit_tunnel_m, 400.0f, "deriva baseline sin NHC acotada");
    expect_less(with_nhc.drift_exit_tunnel_m, 50.0f, "deriva con NHC al salir del tunel");
    expect_greater(
        without_nhc.drift_exit_tunnel_m - with_nhc.drift_exit_tunnel_m,
        100.0f,
        "mejora NHC significativa al salir del tunel");
    expect_less(with_nhc.drift_final_m, 5.0f, "deriva final con NHC tras recuperar GPS");
    expect_greater(
        static_cast<float>(with_nhc.nhc_updates),
        5000.0f,
        "updates NHC durante trayecto completo");
}

void test_tc_03_constant_slope(RegressionTestReport *report)
{
    const ConstantSlopePassResult result = constant_slope_run_pass(true, false);

    add_constant_slope_case(report, true, &result);

    std::printf(
        "  metricas: rms_pos=%.2f rms_vel=%.2f rms_vel_d=%.2f rms_yaw=%.2f innov_norm=%.3f\n",
        result.outage_rms.position_m,
        result.outage_rms.velocity_mps,
        result.outage_rms.velocity_d_mps,
        result.outage_rms.yaw_deg,
        result.nhc_innovation_max.norm_mps);

    expect_less(result.outage_rms.position_m, 3.0f, "TC-03 RMS posicion 3D en apagon < 3 m");
    expect_less(result.outage_rms.velocity_d_mps, 0.5f, "TC-03 RMS velocidad vertical NED baja");
    expect_less(result.drift_final_m, 5.0f, "TC-03 deriva final tras recuperar GPS");
    expect_greater(
        static_cast<float>(result.nhc_updates),
        1000.0f,
        "TC-03 updates NHC durante trayecto");
}

void test_tc_04_aggressive_slalom(RegressionTestReport *report)
{
    const SlalomPassResult result = slalom_run_pass(true, false);

    add_slalom_case(report, true, &result);

    std::printf(
        "  metricas: rms_pos=%.2f rms_vel=%.2f rms_yaw=%.2f innov_norm=%.3f nhc_updates=%u\n",
        result.outage_rms.position_m,
        result.outage_rms.velocity_mps,
        result.outage_rms.yaw_deg,
        result.nhc_innovation_max.norm_mps,
        result.nhc_updates);

    expect_less(result.outage_rms.position_m, 5.0f, "TC-04 RMS posicion 3D en apagon < 5 m");
    expect_less(result.outage_rms.yaw_deg, 1.5f, "TC-04 RMS yaw en apagon < 1.5 deg");
    expect_less(result.drift_final_m, 5.0f, "TC-04 deriva final tras recuperar GPS");
    expect_greater(
        static_cast<float>(result.nhc_updates),
        1000.0f,
        "TC-04 updates NHC durante slalom agresivo");
}

void write_json_string(FILE *fp, const char *value)
{
    std::fprintf(fp, "\"");
    for (const char *ch = value; ch != NULL && *ch != '\0'; ++ch) {
        if (*ch == '"' || *ch == '\\') {
            std::fputc('\\', fp);
        }
        std::fputc(*ch, fp);
    }
    std::fprintf(fp, "\"");
}

void write_super_tunnel_case_json(FILE *fp, const SuperTunnelCaseMetrics *entry)
{
    const SuperTunnelPassResult *result = &entry->result;
    std::fprintf(fp, "        {\n");
    std::fprintf(fp, "          \"nhc_enabled\": %s,\n", entry->nhc_enabled ? "true" : "false");
    std::fprintf(fp, "          \"gps_outage\": {\n");
    std::fprintf(
        fp,
        "            \"start_s\": %.1f,\n",
        static_cast<float>(SUPER_TUNNEL_GPS_OFF_START_MS) * 0.001f);
    std::fprintf(
        fp,
        "            \"end_s\": %.1f\n",
        static_cast<float>(SUPER_TUNNEL_GPS_OFF_END_MS) * 0.001f);
    std::fprintf(fp, "          },\n");
    std::fprintf(fp, "          \"drift_exit_tunnel_m\": %.6f,\n", result->drift_exit_tunnel_m);
    std::fprintf(fp, "          \"drift_final_m\": %.6f,\n", result->drift_final_m);
    std::fprintf(fp, "          \"nhc_updates\": %u,\n", result->nhc_updates);
    std::fprintf(fp, "          \"outage_rms\": {\n");
    std::fprintf(fp, "            \"position_m\": %.6f,\n", result->outage_rms.position_m);
    std::fprintf(fp, "            \"velocity_mps\": %.6f,\n", result->outage_rms.velocity_mps);
    std::fprintf(fp, "            \"yaw_deg\": %.6f,\n", result->outage_rms.yaw_deg);
    std::fprintf(fp, "            \"sample_count\": %u\n", result->outage_rms.sample_count);
    std::fprintf(fp, "          },\n");
    std::fprintf(fp, "          \"nhc_innovation_max_mps\": {\n");
    std::fprintf(
        fp,
        "            \"lateral\": %.6f,\n",
        result->nhc_innovation_max.lateral_mps);
    std::fprintf(
        fp,
        "            \"vertical\": %.6f,\n",
        result->nhc_innovation_max.vertical_mps);
    std::fprintf(
        fp,
        "            \"norm\": %.6f\n",
        result->nhc_innovation_max.norm_mps);
    std::fprintf(fp, "          }\n");
    std::fprintf(fp, "        }");
}

void write_constant_slope_case_json(FILE *fp, const ConstantSlopeCaseMetrics *entry)
{
    const ConstantSlopePassResult *result = &entry->result;
    std::fprintf(fp, "        {\n");
    std::fprintf(fp, "          \"nhc_enabled\": %s,\n", entry->nhc_enabled ? "true" : "false");
    std::fprintf(fp, "          \"scenario\": {\n");
    std::fprintf(fp, "            \"grade_percent\": %.1f,\n", TC03_GRADE_PERCENT);
    std::fprintf(fp, "            \"pitch_deg\": %.6f,\n", TC03_PITCH_RAD * (180.0f / static_cast<float>(M_PI)));
    std::fprintf(fp, "            \"speed_mps\": %.1f\n", TC03_SPEED_MPS);
    std::fprintf(fp, "          },\n");
    std::fprintf(fp, "          \"gps_outage\": {\n");
    std::fprintf(
        fp,
        "            \"start_s\": %.1f,\n",
        static_cast<float>(TC03_GPS_OFF_START_MS) * 0.001f);
    std::fprintf(
        fp,
        "            \"end_s\": %.1f\n",
        static_cast<float>(TC03_GPS_OFF_END_MS) * 0.001f);
    std::fprintf(fp, "          },\n");
    std::fprintf(fp, "          \"drift_exit_outage_m\": %.6f,\n", result->drift_exit_outage_m);
    std::fprintf(fp, "          \"drift_final_m\": %.6f,\n", result->drift_final_m);
    std::fprintf(fp, "          \"nhc_updates\": %u,\n", result->nhc_updates);
    std::fprintf(fp, "          \"outage_rms\": {\n");
    std::fprintf(fp, "            \"position_m\": %.6f,\n", result->outage_rms.position_m);
    std::fprintf(fp, "            \"velocity_mps\": %.6f,\n", result->outage_rms.velocity_mps);
    std::fprintf(fp, "            \"velocity_d_mps\": %.6f,\n", result->outage_rms.velocity_d_mps);
    std::fprintf(fp, "            \"yaw_deg\": %.6f,\n", result->outage_rms.yaw_deg);
    std::fprintf(fp, "            \"sample_count\": %u\n", result->outage_rms.sample_count);
    std::fprintf(fp, "          },\n");
    std::fprintf(fp, "          \"nhc_innovation_max_mps\": {\n");
    std::fprintf(
        fp,
        "            \"lateral\": %.6f,\n",
        result->nhc_innovation_max.lateral_mps);
    std::fprintf(
        fp,
        "            \"vertical\": %.6f,\n",
        result->nhc_innovation_max.vertical_mps);
    std::fprintf(
        fp,
        "            \"norm\": %.6f\n",
        result->nhc_innovation_max.norm_mps);
    std::fprintf(fp, "          }\n");
    std::fprintf(fp, "        }");
}

void write_slalom_case_json(FILE *fp, const SlalomCaseMetrics *entry)
{
    const SlalomPassResult *result = &entry->result;
    std::fprintf(fp, "        {\n");
    std::fprintf(fp, "          \"nhc_enabled\": %s,\n", entry->nhc_enabled ? "true" : "false");
    std::fprintf(fp, "          \"scenario\": {\n");
    std::fprintf(fp, "            \"speed_kmh\": %.1f,\n", TC04_SPEED_KMH);
    std::fprintf(fp, "            \"max_lateral_accel_mps2\": %.1f,\n", TC04_MAX_LATERAL_ACCEL_MPS2);
    std::fprintf(fp, "            \"slalom_period_s\": %.1f\n", TC04_SLALOM_PERIOD_S);
    std::fprintf(fp, "          },\n");
    std::fprintf(fp, "          \"gps_outage\": {\n");
    std::fprintf(
        fp,
        "            \"start_s\": %.1f,\n",
        static_cast<float>(TC04_GPS_OFF_START_MS) * 0.001f);
    std::fprintf(
        fp,
        "            \"end_s\": %.1f\n",
        static_cast<float>(TC04_GPS_OFF_END_MS) * 0.001f);
    std::fprintf(fp, "          },\n");
    std::fprintf(fp, "          \"drift_exit_outage_m\": %.6f,\n", result->drift_exit_outage_m);
    std::fprintf(fp, "          \"drift_final_m\": %.6f,\n", result->drift_final_m);
    std::fprintf(fp, "          \"nhc_updates\": %u,\n", result->nhc_updates);
    std::fprintf(fp, "          \"outage_rms\": {\n");
    std::fprintf(fp, "            \"position_m\": %.6f,\n", result->outage_rms.position_m);
    std::fprintf(fp, "            \"velocity_mps\": %.6f,\n", result->outage_rms.velocity_mps);
    std::fprintf(fp, "            \"yaw_deg\": %.6f,\n", result->outage_rms.yaw_deg);
    std::fprintf(fp, "            \"sample_count\": %u\n", result->outage_rms.sample_count);
    std::fprintf(fp, "          },\n");
    std::fprintf(fp, "          \"nhc_innovation_max_mps\": {\n");
    std::fprintf(
        fp,
        "            \"lateral\": %.6f,\n",
        result->nhc_innovation_max.lateral_mps);
    std::fprintf(
        fp,
        "            \"vertical\": %.6f,\n",
        result->nhc_innovation_max.vertical_mps);
    std::fprintf(
        fp,
        "            \"norm\": %.6f\n",
        result->nhc_innovation_max.norm_mps);
    std::fprintf(fp, "          }\n");
    std::fprintf(fp, "        }");
}

bool export_regression_report_json()
{
    FILE *fp = std::fopen(kRegressionReportPath, "w");
    if (fp == NULL) {
        std::printf("WARN: no se pudo escribir %s\n", kRegressionReportPath);
        return false;
    }

    char timestamp_iso[32] = "unknown";
    const std::time_t now = std::time(NULL);
    if (now != static_cast<std::time_t>(-1)) {
        std::tm tm_local{};
#if defined(_WIN32)
        if (localtime_s(&tm_local, &now) == 0) {
#else
        if (localtime_r(&now, &tm_local) != NULL) {
#endif
            (void)std::strftime(
                timestamp_iso,
                sizeof(timestamp_iso),
                "%Y-%m-%dT%H:%M:%S",
                &tm_local);
        }
    }

    std::fprintf(fp, "{\n");
    std::fprintf(fp, "  \"suite\": \"NaviCore-3D\",\n");
    std::fprintf(fp, "  \"generated_at\": ");
    write_json_string(fp, timestamp_iso);
    std::fprintf(fp, ",\n");
    std::fprintf(fp, "  \"result\": %s,\n", (g_failures == 0) ? "\"OK\"" : "\"FAIL\"");
    std::fprintf(fp, "  \"failure_count\": %d,\n", g_failures);
    std::fprintf(fp, "  \"tests\": [\n");

    for (int i = 0; i < g_report_count; ++i) {
        const RegressionTestReport *report = &g_reports[i];
        std::fprintf(fp, "    {\n");
        std::fprintf(fp, "      \"name\": ");
        write_json_string(fp, report->name);
        std::fprintf(fp, ",\n");
        std::fprintf(fp, "      \"passed\": %s", report->passed ? "true" : "false");

        if (report->has_simple_metrics) {
            std::fprintf(fp, ",\n      \"metrics\": {\n");
            for (int m = 0; m < report->simple_metric_count; ++m) {
                const SimpleTestMetrics *metric = &report->simple_metrics[m];
                std::fprintf(fp, "        ");
                write_json_string(fp, metric->label);
                std::fprintf(
                    fp,
                    ": { \"value\": %.6f, \"unit\": ",
                    metric->value);
                write_json_string(fp, metric->unit);
                std::fprintf(fp, " }%s\n", (m + 1 < report->simple_metric_count) ? "," : "");
            }
            std::fprintf(fp, "      }");
        }

        if (report->has_super_tunnel_cases) {
            std::fprintf(fp, ",\n      \"cases\": [\n");
            for (int c = 0; c < report->super_tunnel_case_count; ++c) {
                write_super_tunnel_case_json(fp, &report->super_tunnel_cases[c]);
                std::fprintf(fp, "%s\n", (c + 1 < report->super_tunnel_case_count) ? "," : "");
            }
            std::fprintf(fp, "      ]");
        }

        if (report->has_constant_slope_case) {
            std::fprintf(fp, ",\n      \"case\": ");
            write_constant_slope_case_json(fp, &report->constant_slope_case);
        }

        if (report->has_slalom_case) {
            std::fprintf(fp, ",\n      \"case\": ");
            write_slalom_case_json(fp, &report->slalom_case);
        }

        std::fprintf(fp, "\n    }%s\n", (i + 1 < g_report_count) ? "," : "");
    }

    std::fprintf(fp, "  ]\n");
    std::fprintf(fp, "}\n");
    std::fclose(fp);
    std::printf("Regression report: %s\n", kRegressionReportPath);
    return true;
}

} /* namespace */

int run_regression_suite()
{
    g_failures = 0;
    g_report_count = 0;

    std::printf("NaviCore-3D regression suite\n");
    std::printf("============================\n");

    run_test("ins_ekf_gravity_compensation", test_ins_ekf_gravity_compensation);
    run_test("nhc_predict_counter", test_nhc_enabled_predict_increments_counter);
    run_test("nhc_jacobian_fd", test_nhc_jacobian_fd);
    run_test("super_tunnel_nhc_isolation", test_super_tunnel_nhc_isolation);
    run_test("super_tunnel_nhc_regression", test_super_tunnel_nhc_regression);
    run_test("TC_03_Constant_Slope", test_tc_03_constant_slope);
    run_test("TC_04_Aggressive_Slalom", test_tc_04_aggressive_slalom);

    std::printf("============================\n");
    (void)export_regression_report_json();

    if (g_failures == 0) {
        std::printf("RESULT: OK (%d tests)\n", kRegressionTestCount);
        return EXIT_SUCCESS;
    }

    std::printf("RESULT: FAIL (%d assertion failures)\n", g_failures);
    return EXIT_FAILURE;
}
