#include <catch2/catch_test_macros.hpp>

#include "health_policy.hpp"
#include "nmea_parser.hpp"
#include "ubx_parser.hpp"
#include "wt61c_parser.hpp"

#include <cstdio>
#include <cstring>
#include <string>
#include <vector>

namespace {

std::string nmea_with_checksum(const std::string &body)
{
    uint8_t ck = 0U;
    for (size_t i = 1; i < body.size(); ++i) {
        ck = static_cast<uint8_t>(ck ^ static_cast<uint8_t>(body[i]));
    }
    char hex[8];
    std::snprintf(hex, sizeof(hex), "*%02X", ck);
    return body + hex;
}

} /* namespace */

TEST_CASE("nmea valid GGA parses", "[nmea]")
{
    const std::string s = nmea_with_checksum(
        "$GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,");
    GpsSample gps{};
    REQUIRE(nmea_try_parse_sentence(s.c_str(), &gps));
    REQUIRE(gps.fix_valid);
    REQUIRE(gps.satellites == 8);
}

TEST_CASE("nmea bad checksum rejected", "[nmea]")
{
    GpsSample gps{};
    REQUIRE_FALSE(nmea_try_parse_sentence(
        "$GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*00",
        &gps));
}

TEST_CASE("nmea assembler never overflows on oversize line", "[nmea]")
{
    NmeaLineAssembler as{};
    nmea_line_assembler_reset(&as);
    char out[NAVICORE_NMEA_LINE_MAX];
    for (int i = 0; i < 500; ++i) {
        REQUIRE_FALSE(nmea_line_assembler_feed(
            &as, static_cast<uint8_t>('A'), out, sizeof(out)));
    }
    REQUIRE(as.len < NAVICORE_NMEA_LINE_MAX);
}

TEST_CASE("ubx oversize length resets without write past buffer", "[ubx]")
{
    UbxStreamParser p{};
    ubx_stream_reset(&p);
    UbxFrame f{};
    const uint8_t bytes[] = {0xB5, 0x62, 0x01, 0x07, 0xFF, 0xFF};
    UbxParseStatus last = UBX_PARSE_NEED_MORE;
    for (uint8_t b : bytes) {
        last = ubx_stream_feed(&p, b, &f);
    }
    REQUIRE(last == UBX_PARSE_OVERSIZE);
}

TEST_CASE("ubx valid empty frame accepted", "[ubx]")
{
    const uint8_t body[] = {0x01, 0x07, 0x00, 0x00};
    uint8_t cka = 0U;
    uint8_t ckb = 0U;
    for (uint8_t b : body) {
        cka = static_cast<uint8_t>(cka + b);
        ckb = static_cast<uint8_t>(ckb + cka);
    }
    std::vector<uint8_t> pkt = {0xB5, 0x62, 0x01, 0x07, 0x00, 0x00, cka, ckb};
    UbxFrame f{};
    REQUIRE(ubx_frame_validate(pkt.data(), pkt.size(), &f));
    REQUIRE(f.msg_class == 0x01);
    REQUIRE(f.payload_len == 0);
}

TEST_CASE("wt61c mid-frame timeout drops partial", "[wt61c]")
{
    Wt61cStreamParser p{};
    wt61c_stream_reset(&p);
    wt61c_stream_feed(&p, 0x55U, 1000U);
    wt61c_stream_feed(&p, 0x51U, 1001U);
    REQUIRE(p.frame_idx == 2U);
    wt61c_stream_check_timeout(&p, 1000U + NAVICORE_WT61C_FRAME_TIMEOUT_US + 10U);
    REQUIRE(p.frame_idx == 0U);
}

TEST_CASE("health policy: IMU silence → DEGRADED + imu_should_degrade", "[fault]")
{
    HealthPolicyInput in{};
    in.imu_silence_ms = HEALTH_POLICY_IMU_SILENCE_DEGRADE_MS;
    const HealthPolicyDecision d = health_policy_evaluate(&in);
    REQUIRE(d.level == HEALTH_POLICY_DEGRADED);
    REQUIRE(d.imu_should_degrade);
    REQUIRE(std::string(d.primary_event) == "imu_silence");
}

TEST_CASE("health policy: UART overflow rate → degrade IMU path", "[fault]")
{
    HealthPolicyInput in{};
    in.uart0_overflows_in_window =
        static_cast<uint16_t>(HEALTH_POLICY_UART_OVERFLOW_PER_S_MAX + 1U);
    const HealthPolicyDecision d = health_policy_evaluate(&in);
    REQUIRE(d.level == HEALTH_POLICY_DEGRADED);
    REQUIRE(d.imu_should_degrade);
    REQUIRE(std::string(d.primary_event) == "uart0_overflow_rate");
}

TEST_CASE("health policy: power offline → CRITICAL", "[fault]")
{
    HealthPolicyInput in{};
    in.power_offline = true;
    const HealthPolicyDecision d = health_policy_evaluate(&in);
    REQUIRE(d.level == HEALTH_POLICY_CRITICAL);
    REQUIRE(std::string(d.primary_event) == "power_offline");
}

TEST_CASE("health policy: task starvation → CRITICAL + restart", "[fault]")
{
    HealthPolicyInput in{};
    in.task_idle_us = 40000U;
    in.task_max_idle_us = 30000U; /* mirrors PICO2_RX_PUMP_MAX_IDLE_US */
    const HealthPolicyDecision d = health_policy_evaluate(&in);
    REQUIRE(d.level == HEALTH_POLICY_CRITICAL);
    REQUIRE(d.controlled_restart);
    REQUIRE(std::string(d.primary_event) == "task_starvation");
}

TEST_CASE("health policy: I2C recoveries → force power offline", "[fault]")
{
    HealthPolicyInput in{};
    in.i2c_recoveries = HEALTH_POLICY_I2C_RECOVERY_OFFLINE_MAX + 1U;
    const HealthPolicyDecision d = health_policy_evaluate(&in);
    REQUIRE(d.level == HEALTH_POLICY_CRITICAL);
    REQUIRE(d.power_should_force_offline);
}
