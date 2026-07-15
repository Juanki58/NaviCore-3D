#pragma once

#include <stdint.h>

#if defined(__cplusplus)
#define NAV_STATE_STATIC_ASSERT(cond, msg) static_assert((cond), msg)
#else
#define NAV_STATE_STATIC_ASSERT(cond, msg) _Static_assert((cond), msg)
#endif

#define NAV_STATE_FLAG_EKF_VALID      (1U << 0)
#define NAV_STATE_FLAG_GPS_FIX        (1U << 1)
#define NAV_STATE_FLAG_NHC_ENABLED    (1U << 2)
#define NAV_STATE_FLAG_GNSS_OUTLIER   (1U << 3)
#define NAV_STATE_FLAG_DEAD_RECKONING (1U << 4)

#pragma pack(push, 1)
struct NavigationState {
    uint64_t timestamp_us;

    double lat_rad;
    double lon_rad;
    float alt_m;

    float vn_mps;
    float ve_mps;
    float vd_mps;

    float roll_rad;
    float pitch_rad;
    float yaw_rad;

    uint32_t health_flags;

    float pos_uncertainty_m;
    float att_uncertainty_rad;
};
#pragma pack(pop)

NAV_STATE_STATIC_ASSERT(sizeof(NavigationState) == 64U, "NavigationState debe ocupar 64 bytes");

#if defined(__cplusplus)
#undef NAV_STATE_STATIC_ASSERT
#endif
