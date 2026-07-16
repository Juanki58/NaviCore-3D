#pragma once

/** Geodesia WGS84 — ECEF <-> LLA <-> NED local (rotacion ortogonal en el origen). */

namespace geodesy {

struct LLA {
    float lat_deg;
    float lon_deg;
    float alt_m;
};

struct ECEF {
    double x_m;
    double y_m;
    double z_m;
};

struct NED {
    float north_m;
    float east_m;
    float down_m;
};

LLA lla(float lat_deg, float lon_deg, float alt_m);

ECEF lla_to_ecef(const LLA &point);
LLA ecef_to_lla(const ECEF &point);

NED ecef_to_ned(const ECEF &point, const LLA &ref);
ECEF ned_to_ecef(const NED &ned, const LLA &ref);

NED lla_to_ned(const LLA &point, const LLA &ref);
LLA ned_to_lla(const NED &ned, const LLA &ref);

/** Aproximacion plana historica (solo validacion / comparacion H8). */
NED lla_to_ned_flat_legacy(const LLA &point, const LLA &ref);

void lla_to_ned(
    float ref_lat_deg,
    float ref_lon_deg,
    float ref_alt_m,
    float lat_deg,
    float lon_deg,
    float alt_m,
    float *north_m,
    float *east_m,
    float *down_m);

void ned_to_lla(
    float ref_lat_deg,
    float ref_lon_deg,
    float ref_alt_m,
    float north_m,
    float east_m,
    float down_m,
    float *lat_deg,
    float *lon_deg,
    float *alt_m);

} /* namespace geodesy */
