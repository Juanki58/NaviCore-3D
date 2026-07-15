#include "regression_suite.hpp"

#include "ins_ekf.hpp"
#include "super_tunnel_benchmark.hpp"

#include <cmath>
#include <cstdio>
#include <cstdlib>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace {

constexpr int kRegressionTestCount = 3;

int g_failures = 0;

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

void run_test(const char *name, void (*body)())
{
    const int before = g_failures;
    std::printf("[TEST] %s\n", name);
    body();
    if (g_failures == before) {
        std::printf("  PASS\n");
    }
}

void test_ins_ekf_gravity_compensation()
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
}

void test_nhc_enabled_predict_increments_counter()
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

    expect_greater(
        static_cast<float>(ins_ekf_nhc_update_count(&ekf)),
        5.0f,
        "contador NHC incrementa con prediccion IMU");
}

void test_super_tunnel_nhc_regression()
{
    const SuperTunnelPassResult without_nhc = super_tunnel_run_pass(false, false);
    const SuperTunnelPassResult with_nhc = super_tunnel_run_pass(true, false);

    std::printf(
        "  metricas: sin_nhc_exit=%.2f con_nhc_exit=%.2f sin_nhc_final=%.2f con_nhc_final=%.2f nhc_updates=%u\n",
        without_nhc.drift_exit_tunnel_m,
        with_nhc.drift_exit_tunnel_m,
        without_nhc.drift_final_m,
        with_nhc.drift_final_m,
        with_nhc.nhc_updates);

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

} /* namespace */

int run_regression_suite()
{
    g_failures = 0;

    std::printf("NaviCore-3D regression suite\n");
    std::printf("============================\n");

    run_test("ins_ekf_gravity_compensation", test_ins_ekf_gravity_compensation);
    run_test("nhc_predict_counter", test_nhc_enabled_predict_increments_counter);
    run_test("super_tunnel_nhc_regression", test_super_tunnel_nhc_regression);

    std::printf("============================\n");
    if (g_failures == 0) {
        std::printf("RESULT: OK (%d tests)\n", kRegressionTestCount);
        return EXIT_SUCCESS;
    }

    std::printf("RESULT: FAIL (%d assertion failures)\n", g_failures);
    return EXIT_FAILURE;
}
