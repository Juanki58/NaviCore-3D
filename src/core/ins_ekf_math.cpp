#include "ins_ekf_math.hpp"

#include <math.h>

void navicore_quat_normalize(float q[4])
{
    if (q == NULL) {
        return;
    }

    const float norm_sq =
        (q[0] * q[0]) + (q[1] * q[1]) + (q[2] * q[2]) + (q[3] * q[3]);

    if (norm_sq <= NAVICORE_QUAT_NORM_EPS_SQ) {
        q[0] = 1.0f;
        q[1] = 0.0f;
        q[2] = 0.0f;
        q[3] = 0.0f;
        return;
    }

    const float inv_norm = 1.0f / sqrtf(norm_sq);
    q[0] *= inv_norm;
    q[1] *= inv_norm;
    q[2] *= inv_norm;
    q[3] *= inv_norm;
}

bool navicore_mat_invert2x2(const float s[2][2], float inv_out[2][2])
{
    if (s == NULL || inv_out == NULL) {
        return false;
    }

    const float det = (s[0][0] * s[1][1]) - (s[0][1] * s[1][0]);
    if (fabsf(det) <= NAVICORE_MAT_SINGULAR_DET_EPS) {
        return false;
    }

    const float inv_det = 1.0f / det;
    inv_out[0][0] = s[1][1] * inv_det;
    inv_out[0][1] = -s[0][1] * inv_det;
    inv_out[1][0] = -s[1][0] * inv_det;
    inv_out[1][1] = s[0][0] * inv_det;
    return true;
}

bool navicore_mat_invert3x3(const float s[3][3], float inv_out[3][3])
{
    if (s == NULL || inv_out == NULL) {
        return false;
    }

    const float det =
        (s[0][0] * ((s[1][1] * s[2][2]) - (s[1][2] * s[2][1])))
        - (s[0][1] * ((s[1][0] * s[2][2]) - (s[1][2] * s[2][0])))
        + (s[0][2] * ((s[1][0] * s[2][1]) - (s[1][1] * s[2][0])));

    if (fabsf(det) <= NAVICORE_MAT_SINGULAR_DET_EPS) {
        return false;
    }

    const float inv_det = 1.0f / det;

    inv_out[0][0] = ((s[1][1] * s[2][2]) - (s[1][2] * s[2][1])) * inv_det;
    inv_out[0][1] = ((s[0][2] * s[2][1]) - (s[0][1] * s[2][2])) * inv_det;
    inv_out[0][2] = ((s[0][1] * s[1][2]) - (s[0][2] * s[1][1])) * inv_det;

    inv_out[1][0] = ((s[1][2] * s[2][0]) - (s[1][0] * s[2][2])) * inv_det;
    inv_out[1][1] = ((s[0][0] * s[2][2]) - (s[0][2] * s[2][0])) * inv_det;
    inv_out[1][2] = ((s[0][2] * s[1][0]) - (s[0][0] * s[1][2])) * inv_det;

    inv_out[2][0] = ((s[1][0] * s[2][1]) - (s[1][1] * s[2][0])) * inv_det;
    inv_out[2][1] = ((s[0][1] * s[2][0]) - (s[0][0] * s[2][1])) * inv_det;
    inv_out[2][2] = ((s[0][0] * s[1][1]) - (s[0][1] * s[1][0])) * inv_det;
    return true;
}
