/**
 * Integrity gate detection-boundary sweep (SW injection only — no RF).
 *
 * Maps |Δp| and gap duration → accept / INCONSISTENT / other reject.
 * Optional JSON dump via NAVICORE_INTEGRITY_SWEEP_OUT=<path>.
 */
#include <catch2/catch_test_macros.hpp>

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <string>
#include <vector>

#include "geodesy.hpp"
#include "ins_ekf.hpp"
#include "vector3d.h"

namespace {

ImuSample make_level_imu(uint32_t t_ms)
{
    ImuSample imu{};
    imu.valid = true;
    imu.timestamp_ms = t_ms;
    imu.accel_mps2[0] = 0.0f;
    imu.accel_mps2[1] = 0.0f;
    imu.accel_mps2[2] = 9.80665f;
    imu.gyro_radps[0] = 0.0f;
    imu.gyro_radps[1] = 0.0f;
    imu.gyro_radps[2] = 0.0f;
    return imu;
}

void seed_ekf(InsEkfFilter *ekf, const Vector3D &origin, float speed_mps)
{
    ins_ekf_init(ekf, origin, 0.0f, NAVICORE_DOMAIN_AIR);
    ins_ekf_set_consistency_check_enabled(ekf, true);
    ins_ekf_set_gnss_obs_mode(ekf, INS_EKF_GNSS_OBS_POS);
    ekf->vel_[0] = speed_mps;
    ekf->vel_[1] = 0.0f;
    ekf->vel_[2] = 0.0f;

    GpsSample good{};
    good.fix_valid = true;
    good.timestamp_ms = 50U;
    good.position = origin;
    good.speed_mps = speed_mps;
    good.course_deg = 0.0f;
    good.satellites = 12U;
    REQUIRE(ins_ekf_update_gnss(ekf, &good));

    for (uint32_t t_ms = 60U; t_ms <= 200U; t_ms += 10U) {
        ImuSample imu = make_level_imu(t_ms);
        REQUIRE(ins_ekf_predict(ekf, &imu));
    }
}

GpsSample gps_at_ned_offset(
    const InsEkfFilter &ekf,
    float dn_m,
    float de_m,
    uint32_t t_ms,
    float speed_mps)
{
    float lat = 0.0f;
    float lon = 0.0f;
    float alt = 0.0f;
    geodesy::ned_to_lla(
        ekf.ref_lat_deg,
        ekf.ref_lon_deg,
        ekf.ref_alt_m,
        ekf.pos_[0] + dn_m,
        ekf.pos_[1] + de_m,
        ekf.pos_[2],
        &lat,
        &lon,
        &alt);

    GpsSample gps{};
    gps.fix_valid = true;
    gps.timestamp_ms = t_ms;
    gps.position = vector3d_make(lat, lon, alt);
    gps.speed_mps = speed_mps;
    gps.course_deg = 0.0f;
    gps.satellites = 12U;
    return gps;
}

struct TrialResult {
    const char *scenario;
    float jump_m;
    float gap_s;
    float speed_lie_mps; /* GPS speed reported; -1 = match INS */
    bool accepted;
    int reject_reason;
    bool suspect;
    float innov_h_m;
    bool state_held;
};

TrialResult run_position_jump(float jump_m, float gap_s)
{
    const Vector3D origin = vector3d_make(41.3874f, 2.1686f, 12.0f);
    InsEkfFilter ekf{};
    seed_ekf(&ekf, origin, 10.0f);

    const float pos_n0 = ekf.pos_[0];
    const float pos_e0 = ekf.pos_[1];

    const uint32_t t_ms = 200U + static_cast<uint32_t>(gap_s * 1000.0f + 0.5f);
    GpsSample spoof = gps_at_ned_offset(ekf, jump_m, 0.0f, t_ms, 10.0f);
    const bool accepted = ins_ekf_update_gnss(&ekf, &spoof);

    TrialResult r{};
    r.scenario = "pos_jump";
    r.jump_m = jump_m;
    r.gap_s = gap_s;
    r.speed_lie_mps = -1.0f;
    r.accepted = accepted;
    r.reject_reason = static_cast<int>(ekf.gnss_last_reject_reason);
    r.suspect = ins_ekf_gnss_consistency_last_suspect(&ekf) != 0U;
    r.innov_h_m = ekf.gnss_consistency_last_innov_h_m;
    r.state_held =
        (std::fabs(ekf.pos_[0] - pos_n0) < 1.0e-3f)
        && (std::fabs(ekf.pos_[1] - pos_e0) < 1.0e-3f);
    return r;
}

TrialResult run_velocity_lie(float fake_speed_mps, float gap_s)
{
    const Vector3D origin = vector3d_make(41.3874f, 2.1686f, 12.0f);
    InsEkfFilter ekf{};
    seed_ekf(&ekf, origin, 10.0f);

    const float pos_n0 = ekf.pos_[0];
    const float pos_e0 = ekf.pos_[1];

    const uint32_t t_ms = 200U + static_cast<uint32_t>(gap_s * 1000.0f + 0.5f);
    /* Same position as INS (no teleport); GPS velocity lies. */
    GpsSample lie = gps_at_ned_offset(ekf, 0.0f, 0.0f, t_ms, fake_speed_mps);
    /* Force POS+VEL obs so velocity jump is evaluated. */
    ins_ekf_set_gnss_obs_mode(&ekf, INS_EKF_GNSS_OBS_POS_VEL);
    const bool accepted = ins_ekf_update_gnss(&ekf, &lie);

    TrialResult r{};
    r.scenario = "vel_lie";
    r.jump_m = 0.0f;
    r.gap_s = gap_s;
    r.speed_lie_mps = fake_speed_mps;
    r.accepted = accepted;
    r.reject_reason = static_cast<int>(ekf.gnss_last_reject_reason);
    r.suspect = ins_ekf_gnss_consistency_last_suspect(&ekf) != 0U;
    r.innov_h_m = ekf.gnss_consistency_last_innov_h_m;
    r.state_held =
        (std::fabs(ekf.pos_[0] - pos_n0) < 1.0e-3f)
        && (std::fabs(ekf.pos_[1] - pos_e0) < 1.0e-3f);
    return r;
}

const char *reason_name(int reason)
{
    switch (reason) {
    case MEAS_REJECT_NONE:
        return "NONE";
    case MEAS_REJECT_NIS:
        return "NIS";
    case MEAS_REJECT_S_SINGULAR:
        return "S_SINGULAR";
    case MEAS_REJECT_INCONSISTENT:
        return "INCONSISTENT";
    default:
        return "OTHER";
    }
}

void append_json_trial(std::ostream &os, const TrialResult &r, bool last)
{
    os << "    {"
       << "\"scenario\":\"" << r.scenario << "\","
       << "\"jump_m\":" << r.jump_m << ","
       << "\"gap_s\":" << r.gap_s << ","
       << "\"speed_lie_mps\":" << r.speed_lie_mps << ","
       << "\"accepted\":" << (r.accepted ? "true" : "false") << ","
       << "\"reject_reason\":" << r.reject_reason << ","
       << "\"reject_name\":\"" << reason_name(r.reject_reason) << "\","
       << "\"suspect\":" << (r.suspect ? "true" : "false") << ","
       << "\"innov_h_m\":" << r.innov_h_m << ","
       << "\"state_held_on_reject\":" << ((!r.accepted && r.state_held) ? "true" : "false")
       << "}" << (last ? "\n" : ",\n");
}

} /* namespace */

TEST_CASE("Integrity gate sweep: position jumps + velocity lies", "[integrity][sweep]")
{
    const float jumps_m[] = {
        1.0f, 5.0f, 20.0f, 40.0f, 80.0f, 100.0f, 121.0f, 150.0f, 300.0f, 500.0f};
    const float gaps_s[] = {0.2f, 1.0f, 3.0f}; /* 3 s > MAX_GAP → no reason=3 */

    std::vector<TrialResult> trials;
    trials.reserve(64);

    for (float gap : gaps_s) {
        for (float jump : jumps_m) {
            trials.push_back(run_position_jump(jump, gap));
        }
    }

    /* Velocity lie on short gap: INS ~10 m/s, GPS claims 60 m/s → Δv=50 > 35. */
    trials.push_back(run_velocity_lie(60.0f, 0.2f));
    /* Mild speed error should not trip vel gate (Δv=5 < 35). */
    trials.push_back(run_velocity_lie(15.0f, 0.2f));

    /* --- Assertions that define the product claim --- */

    /* Large teleport, short gap → INCONSISTENT + state hold. */
    {
        const TrialResult big = run_position_jump(500.0f, 0.2f);
        REQUIRE_FALSE(big.accepted);
        REQUIRE(big.reject_reason == MEAS_REJECT_INCONSISTENT);
        REQUIRE(big.suspect);
        REQUIRE(big.state_held);
    }

    /* Sub-metre nudge, short gap → never INCONSISTENT. */
    {
        const TrialResult nudge = run_position_jump(1.0f, 0.2f);
        REQUIRE(nudge.reject_reason != MEAS_REJECT_INCONSISTENT);
        REQUIRE_FALSE(nudge.suspect);
    }

    /* Same 500 m jump after long gap → must NOT be classified as spoof (tunnel exit). */
    {
        const TrialResult tunnel = run_position_jump(500.0f, 3.0f);
        REQUIRE(tunnel.reject_reason != MEAS_REJECT_INCONSISTENT);
        REQUIRE_FALSE(tunnel.suspect);
    }

    /* Hard velocity lie → INCONSISTENT on short gap. */
    {
        const TrialResult vlie = run_velocity_lie(60.0f, 0.2f);
        REQUIRE_FALSE(vlie.accepted);
        REQUIRE(vlie.reject_reason == MEAS_REJECT_INCONSISTENT);
        REQUIRE(vlie.suspect);
        REQUIRE(vlie.state_held);
    }

    /* Optional JSON for campaign packaging. */
    const char *out_path = std::getenv("NAVICORE_INTEGRITY_SWEEP_OUT");
    if (out_path != nullptr && out_path[0] != '\0') {
        std::ofstream os(out_path, std::ios::binary);
        REQUIRE(os.good());
        os << "{\n"
           << "  \"title\": \"integrity_gate_sweep\",\n"
           << "  \"policy\": \"SW injection only — no RF spoof/jam\",\n"
           << "  \"thresholds\": {\n"
           << "    \"MAX_GAP_S\": " << NAVICORE_INS_EKF_CONSISTENCY_MAX_GAP_S << ",\n"
           << "    \"MAX_POS_JUMP_M\": " << NAVICORE_INS_EKF_CONSISTENCY_MAX_POS_JUMP_M
           << ",\n"
           << "    \"POS_MARGIN_M\": " << NAVICORE_INS_EKF_CONSISTENCY_POS_MARGIN_M << ",\n"
           << "    \"MAX_VEL_JUMP_MPS\": " << NAVICORE_INS_EKF_CONSISTENCY_MAX_VEL_JUMP_MPS
           << "\n"
           << "  },\n"
           << "  \"trials\": [\n";
        for (size_t i = 0; i < trials.size(); ++i) {
            append_json_trial(os, trials[i], i + 1 == trials.size());
        }
        os << "  ]\n}\n";
        os.close();
        WARN("Wrote integrity sweep JSON to " << out_path);
    }

    /* Count short-gap teleports above hard cap that tripped INCONSISTENT. */
    int tp = 0;
    int tp_n = 0;
    for (const TrialResult &r : trials) {
        if (r.scenario != std::string("pos_jump") || r.gap_s > 2.0f) {
            continue;
        }
        if (r.jump_m > NAVICORE_INS_EKF_CONSISTENCY_MAX_POS_JUMP_M) {
            ++tp_n;
            if (r.reject_reason == MEAS_REJECT_INCONSISTENT && !r.accepted) {
                ++tp;
            }
        }
    }
    REQUIRE(tp_n > 0);
    REQUIRE(tp == tp_n);
}
