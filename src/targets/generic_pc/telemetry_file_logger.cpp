#include "telemetry_file_logger.hpp"

#include <cstdio>
#include <iostream>

TelemetryFileLogger::TelemetryFileLogger(const char *csv_path)
    : csv_path_(csv_path),
      file_(NULL),
      ready_(false)
{
    if (csv_path_ == NULL || csv_path_[0] == '\0') {
        std::cerr << "[-] TelemetryFileLogger: ruta CSV invalida" << std::endl;
        return;
    }

    FILE *fp = std::fopen(csv_path_, "a");
    if (fp == NULL) {
        std::cerr << "[-] TelemetryFileLogger: no se pudo abrir " << csv_path_ << std::endl;
        return;
    }

    if (std::fseek(fp, 0, SEEK_END) != 0) {
        std::cerr << "[-] TelemetryFileLogger: error al inspeccionar " << csv_path_ << std::endl;
        std::fclose(fp);
        return;
    }

    const long file_size = std::ftell(fp);
    if (file_size < 0L) {
        std::cerr << "[-] TelemetryFileLogger: error al leer tamano de " << csv_path_ << std::endl;
        std::fclose(fp);
        return;
    }

    file_ = fp;
    if (file_size == 0L && !write_header()) {
        std::fclose(static_cast<FILE *>(file_));
        file_ = NULL;
        return;
    }

    ready_ = true;
    std::cout << "[*] NavigationState CSV -> " << csv_path_ << std::endl;
}

TelemetryFileLogger::~TelemetryFileLogger()
{
    if (file_ != NULL) {
        std::fflush(static_cast<FILE *>(file_));
        std::fclose(static_cast<FILE *>(file_));
        file_ = NULL;
    }
    ready_ = false;
}

bool TelemetryFileLogger::is_ready() const
{
    return ready_;
}

bool TelemetryFileLogger::write_header()
{
    if (file_ == NULL) {
        return false;
    }

    FILE *fp = static_cast<FILE *>(file_);
    const int written = std::fprintf(
        fp,
        "timestamp_us,lat_rad,lon_rad,alt_m,vn_mps,ve_mps,vd_mps,"
        "roll_rad,pitch_rad,yaw_rad,health_flags,pos_uncertainty_m,att_uncertainty_rad\n");

    if (written < 0) {
        std::cerr << "[-] TelemetryFileLogger: error al escribir cabecera CSV" << std::endl;
        return false;
    }

    return true;
}

bool TelemetryFileLogger::log(const NavigationState &state)
{
    if (!ready_ || file_ == NULL) {
        return false;
    }

    FILE *fp = static_cast<FILE *>(file_);
    const int written = std::fprintf(
        fp,
        "%llu,%.17g,%.17g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%u,%.9g,%.9g\n",
        static_cast<unsigned long long>(state.timestamp_us),
        state.lat_rad,
        state.lon_rad,
        state.alt_m,
        state.vn_mps,
        state.ve_mps,
        state.vd_mps,
        state.roll_rad,
        state.pitch_rad,
        state.yaw_rad,
        state.health_flags,
        state.pos_uncertainty_m,
        state.att_uncertainty_rad);

    if (written < 0) {
        std::cerr << "[-] TelemetryFileLogger: error al escribir fila CSV" << std::endl;
        return false;
    }

    return true;
}

void TelemetryFileLogger::flush()
{
    if (file_ != NULL) {
        std::fflush(static_cast<FILE *>(file_));
    }
}
