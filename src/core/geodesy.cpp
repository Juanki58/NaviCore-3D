#include "geodesy.hpp"

#include <cmath>

namespace geodesy {

namespace {

constexpr double kWgs84A = 6378137.0;
constexpr double kWgs84F = 1.0 / 298.257223563;
constexpr double kWgs84E2 = kWgs84F * (2.0 - kWgs84F);
constexpr double kDegToRad = 3.14159265358979323846 / 180.0;
constexpr double kRadToDeg = 180.0 / 3.14159265358979323846;

double deg_to_rad(double deg)
{
    return deg * kDegToRad;
}

double rad_to_deg(double rad)
{
    return rad * kRadToDeg;
}

} /* namespace */

LLA lla(float lat_deg, float lon_deg, float alt_m)
{
    return LLA{lat_deg, lon_deg, alt_m};
}

ECEF lla_to_ecef(const LLA &point)
{
    const double lat = deg_to_rad(static_cast<double>(point.lat_deg));
    const double lon = deg_to_rad(static_cast<double>(point.lon_deg));
    const double alt = static_cast<double>(point.alt_m);

    const double sin_lat = std::sin(lat);
    const double cos_lat = std::cos(lat);
    const double sin_lon = std::sin(lon);
    const double cos_lon = std::cos(lon);

    const double n_radius = kWgs84A / std::sqrt(1.0 - kWgs84E2 * sin_lat * sin_lat);
    const double x = (n_radius + alt) * cos_lat * cos_lon;
    const double y = (n_radius + alt) * cos_lat * sin_lon;
    const double z = (n_radius * (1.0 - kWgs84E2) + alt) * sin_lat;
    return ECEF{x, y, z};
}

LLA ecef_to_lla(const ECEF &point)
{
    const double x = point.x_m;
    const double y = point.y_m;
    const double z = point.z_m;

    const double p = std::sqrt((x * x) + (y * y));
    const double theta = std::atan2(z * kWgs84A, p * (kWgs84A * (1.0 - kWgs84E2)));
    const double sin_theta = std::sin(theta);
    const double cos_theta = std::cos(theta);

    const double lat = std::atan2(
        z + (kWgs84E2 * (1.0 - kWgs84E2) * kWgs84A * sin_theta * sin_theta * sin_theta)
            / (1.0 - kWgs84E2),
        p - (kWgs84E2 * kWgs84A * cos_theta * cos_theta * cos_theta));
    const double lon = std::atan2(y, x);

    const double sin_lat = std::sin(lat);
    const double n_radius = kWgs84A / std::sqrt(1.0 - kWgs84E2 * sin_lat * sin_lat);
    const double alt = (p / std::cos(lat)) - n_radius;

    return LLA{
        static_cast<float>(rad_to_deg(lat)),
        static_cast<float>(rad_to_deg(lon)),
        static_cast<float>(alt),
    };
}

NED ecef_to_ned(const ECEF &point, const LLA &ref)
{
    const ECEF ref_ecef = lla_to_ecef(ref);
    const double dx = point.x_m - ref_ecef.x_m;
    const double dy = point.y_m - ref_ecef.y_m;
    const double dz = point.z_m - ref_ecef.z_m;

    const double lat0 = deg_to_rad(static_cast<double>(ref.lat_deg));
    const double lon0 = deg_to_rad(static_cast<double>(ref.lon_deg));
    const double sin_lat = std::sin(lat0);
    const double cos_lat = std::cos(lat0);
    const double sin_lon = std::sin(lon0);
    const double cos_lon = std::cos(lon0);

    const double north = (-sin_lat * cos_lon * dx) - (sin_lat * sin_lon * dy) + (cos_lat * dz);
    const double east = (-sin_lon * dx) + (cos_lon * dy);
    const double down = (-cos_lat * cos_lon * dx) - (cos_lat * sin_lon * dy) - (sin_lat * dz);

    return NED{
        static_cast<float>(north),
        static_cast<float>(east),
        static_cast<float>(down),
    };
}

ECEF ned_to_ecef(const NED &ned, const LLA &ref)
{
    const ECEF ref_ecef = lla_to_ecef(ref);

    const double lat0 = deg_to_rad(static_cast<double>(ref.lat_deg));
    const double lon0 = deg_to_rad(static_cast<double>(ref.lon_deg));
    const double sin_lat = std::sin(lat0);
    const double cos_lat = std::cos(lat0);
    const double sin_lon = std::sin(lon0);
    const double cos_lon = std::cos(lon0);

    const double north = static_cast<double>(ned.north_m);
    const double east = static_cast<double>(ned.east_m);
    const double down = static_cast<double>(ned.down_m);

    const double dx = (-sin_lat * cos_lon * north) - (sin_lon * east) - (cos_lat * cos_lon * down);
    const double dy = (-sin_lat * sin_lon * north) + (cos_lon * east) - (cos_lat * sin_lon * down);
    const double dz = (cos_lat * north) - (sin_lat * down);

    return ECEF{
        ref_ecef.x_m + dx,
        ref_ecef.y_m + dy,
        ref_ecef.z_m + dz,
    };
}

NED lla_to_ned(const LLA &point, const LLA &ref)
{
    return ecef_to_ned(lla_to_ecef(point), ref);
}

LLA ned_to_lla(const NED &ned, const LLA &ref)
{
    return ecef_to_lla(ned_to_ecef(ned, ref));
}

NED lla_to_ned_flat_legacy(const LLA &point, const LLA &ref)
{
    constexpr float kMetersPerDegLat = 111132.954f;
    const float dlat_m = (point.lat_deg - ref.lat_deg) * kMetersPerDegLat;
    const float lat_rad = static_cast<float>(
        deg_to_rad(static_cast<double>((ref.lat_deg + point.lat_deg) * 0.5f)));
    const float dlon_m =
        (point.lon_deg - ref.lon_deg) * kMetersPerDegLat * std::cos(static_cast<double>(lat_rad));

    return NED{
        dlat_m,
        dlon_m,
        ref.alt_m - point.alt_m,
    };
}

void lla_to_ned(
    float ref_lat_deg,
    float ref_lon_deg,
    float ref_alt_m,
    float lat_deg,
    float lon_deg,
    float alt_m,
    float *north_m,
    float *east_m,
    float *down_m)
{
    const LLA ref = lla(ref_lat_deg, ref_lon_deg, ref_alt_m);
    const LLA point = lla(lat_deg, lon_deg, alt_m);
    const NED ned = lla_to_ned(point, ref);

    if (north_m != nullptr) {
        *north_m = ned.north_m;
    }
    if (east_m != nullptr) {
        *east_m = ned.east_m;
    }
    if (down_m != nullptr) {
        *down_m = ned.down_m;
    }
}

void ned_to_lla(
    float ref_lat_deg,
    float ref_lon_deg,
    float ref_alt_m,
    float north_m,
    float east_m,
    float down_m,
    float *lat_deg,
    float *lon_deg,
    float *alt_m)
{
    const LLA ref = lla(ref_lat_deg, ref_lon_deg, ref_alt_m);
    const NED ned{north_m, east_m, down_m};
    const LLA point = ned_to_lla(ned, ref);

    if (lat_deg != nullptr) {
        *lat_deg = point.lat_deg;
    }
    if (lon_deg != nullptr) {
        *lon_deg = point.lon_deg;
    }
    if (alt_m != nullptr) {
        *alt_m = point.alt_m;
    }
}

} /* namespace geodesy */
