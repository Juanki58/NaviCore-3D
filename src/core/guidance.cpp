#include "guidance.hpp"

#include "math_utils.hpp"

#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

#ifndef NAVICORE_METERS_PER_DEG_LAT
#define NAVICORE_METERS_PER_DEG_LAT 111132.954f
#endif

static float deg_to_rad(float deg)
{
    return deg * (M_PI / 180.0f);
}

static void latlon_to_local_ne_m(
    float ref_lat_deg,
    float ref_lon_deg,
    float lat_deg,
    float lon_deg,
    float *north_m,
    float *east_m)
{
    const float dlat_m = (lat_deg - ref_lat_deg) * NAVICORE_METERS_PER_DEG_LAT;
    const float dlon_raw = (lon_deg - ref_lon_deg) * NAVICORE_METERS_PER_DEG_LAT;
    const float mean_lat_rad = deg_to_rad((ref_lat_deg + lat_deg) * 0.5f);

    *north_m = dlat_m;
    *east_m = dlon_raw * cosf(mean_lat_rad);
}

static GuidanceErrors guidance_errors_degenerate(
    float pos_n_m,
    float pos_e_m,
    float track_len_m)
{
    GuidanceErrors errors{};

    const float pos_len_sq = (pos_n_m * pos_n_m) + (pos_e_m * pos_e_m);
    if (pos_len_sq <= (NAVICORE_EPS_DISPLACEMENT_M * NAVICORE_EPS_DISPLACEMENT_M)) {
        errors.cross_track_m = 0.0f;
        errors.along_track_m = track_len_m;
        return errors;
    }

    errors.cross_track_m = sqrtf(pos_len_sq);
    errors.along_track_m = track_len_m;
    return errors;
}

GuidanceErrors guidance_compute_errors(
    Vector3D position,
    Waypoint origin,
    Waypoint destination)
{
    const float ref_lat = origin.position.x;
    const float ref_lon = origin.position.y;

    float track_n_m = 0.0f;
    float track_e_m = 0.0f;
    latlon_to_local_ne_m(
        ref_lat,
        ref_lon,
        destination.position.x,
        destination.position.y,
        &track_n_m,
        &track_e_m);

    float pos_n_m = 0.0f;
    float pos_e_m = 0.0f;
    latlon_to_local_ne_m(
        ref_lat,
        ref_lon,
        position.x,
        position.y,
        &pos_n_m,
        &pos_e_m);

    const float track_len_sq = (track_n_m * track_n_m) + (track_e_m * track_e_m);
    if (track_len_sq <= (NAVICORE_EPS_DISPLACEMENT_M * NAVICORE_EPS_DISPLACEMENT_M)) {
        return guidance_errors_degenerate(pos_n_m, pos_e_m, 0.0f);
    }

    const float track_len_m = sqrtf(track_len_sq);
    const float inv_track_len_sq = 1.0f / track_len_sq;

    const float along_from_origin_m =
        ((pos_n_m * track_n_m) + (pos_e_m * track_e_m)) * inv_track_len_sq * track_len_m;

    const float proj_n_m = along_from_origin_m * (track_n_m / track_len_m);
    const float proj_e_m = along_from_origin_m * (track_e_m / track_len_m);

    const float cross_n_m = pos_n_m - proj_n_m;
    const float cross_e_m = pos_e_m - proj_e_m;
    const float cross_len_sq = (cross_n_m * cross_n_m) + (cross_e_m * cross_e_m);

    GuidanceErrors errors{};
    errors.along_track_m = track_len_m - along_from_origin_m;

    if (cross_len_sq <= (NAVICORE_EPS_DISPLACEMENT_M * NAVICORE_EPS_DISPLACEMENT_M)) {
        errors.cross_track_m = 0.0f;
        return errors;
    }

    const float cross_sign = (track_n_m * pos_e_m) - (track_e_m * pos_n_m);
    const float cross_mag_m = sqrtf(cross_len_sq);
    errors.cross_track_m = (cross_sign >= 0.0f) ? cross_mag_m : -cross_mag_m;

    return errors;
}
