#pragma once

#include "navigation_state.hpp"

#include <cstdio>
#include <cstdint>

constexpr const char *TELEMETRY_FILE_LOGGER_DEFAULT_PATH = "telemetry_log.csv";

class TelemetryFileLogger {
public:
    explicit TelemetryFileLogger(const char *csv_path = TELEMETRY_FILE_LOGGER_DEFAULT_PATH);
    ~TelemetryFileLogger();

    TelemetryFileLogger(const TelemetryFileLogger &) = delete;
    TelemetryFileLogger &operator=(const TelemetryFileLogger &) = delete;

    bool is_ready() const;
    bool log(const NavigationState &state);
    void flush();

private:
    bool write_header();

    const char *csv_path_;
    FILE *file_;
    bool ready_;
};
