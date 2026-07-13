#include "flight_recorder.hpp"

#include <math.h>
#include <string.h>

static void flight_recorder_copy_scenario_name(
    char *dst,
    size_t dst_size,
    const char *scenario_name)
{
    if (dst == NULL || dst_size == 0U) {
        return;
    }

    if (scenario_name == NULL) {
        dst[0] = '\0';
        return;
    }

    strncpy(dst, scenario_name, dst_size - 1U);
    dst[dst_size - 1U] = '\0';
}

static void flight_recorder_fill_p_diag_sqrt(
    const InsEkfFilter *ekf,
    float out_diag_sqrt[INS_EKF_STATE_DIM])
{
    if (ekf == NULL || out_diag_sqrt == NULL) {
        return;
    }

    for (uint8_t i = 0U; i < INS_EKF_STATE_DIM; ++i) {
        const float variance = ekf->cov.P[i][i];
        out_diag_sqrt[i] = (variance > 0.0f) ? sqrtf(variance) : 0.0f;
    }
}

void flight_recorder_sample_zero(FlightRecorderSample *sample)
{
    if (sample == NULL) {
        return;
    }

    memset(sample, 0, sizeof(*sample));
}

bool flight_recorder_capture(
    FlightRecorderSample *sample,
    const FlightRecorderTickInput *input)
{
    if (sample == NULL || input == NULL || input->nav_state == NULL) {
        return false;
    }

    flight_recorder_sample_zero(sample);

    const NavState *state = input->nav_state;

    sample->timestamp_ms = input->timestamp_ms;
    flight_recorder_copy_scenario_name(
        sample->scenario_name,
        sizeof(sample->scenario_name),
        input->scenario_name);
    sample->scenario_id = input->scenario_id;

    if (input->imu != NULL) {
        memcpy(sample->imu_accel_mps2, input->imu->accel_mps2, sizeof(sample->imu_accel_mps2));
        memcpy(sample->imu_gyro_radps, input->imu->gyro_radps, sizeof(sample->imu_gyro_radps));
        sample->imu_valid = input->imu->valid;
    }

    if (input->gps != NULL) {
        sample->gnss_fix_valid = input->gps->fix_valid;
        sample->gnss_satellites = input->gps->satellites;
        sample->gnss_lat_deg = input->gps->position.x;
        sample->gnss_lon_deg = input->gps->position.y;
        sample->gnss_alt_m = input->gps->position.z;
        sample->gnss_speed_mps = input->gps->speed_mps;
        sample->gnss_course_deg = input->gps->course_deg;
    }

    sample->pos_x = state->position.x;
    sample->pos_y = state->position.y;
    sample->pos_z = state->position.z;
    sample->vel_x = state->velocity.x;
    sample->vel_y = state->velocity.y;
    sample->vel_z = state->velocity.z;
    sample->heading_deg = state->heading_deg;
    sample->estimate_quality = state->confidence.estimate_quality;
    sample->nav_mode = state->mode;

    if (input->guidance_errors != NULL) {
        sample->cross_track_m = input->guidance_errors->cross_track_m;
        sample->along_track_m = input->guidance_errors->along_track_m;
        sample->cross_track_signed_m = input->guidance_errors->cross_track_signed_m;
    }

    if (input->ekf != NULL && input->ekf->initialized) {
        sample->ekf_active = true;
        sample->ekf_outlier = ins_ekf_outlier_detected(input->ekf);
        sample->gnss_nis = ins_ekf_last_nis(input->ekf);
        ins_ekf_get_gnss_innovation(input->ekf, sample->gnss_innovation_ned);
        sample->gnss_update_accepted = input->gnss_update_accepted;
        sample->gnss_accept_count = ins_ekf_gnss_accept_count(input->ekf);
        sample->gnss_reject_count = ins_ekf_gnss_reject_count(input->ekf);

        ins_ekf_get_bias(
            input->ekf,
            sample->ekf_bias_accel,
            sample->ekf_bias_gyro);
        ins_ekf_get_attitude_rad(
            input->ekf,
            &sample->ekf_attitude_rad[0],
            &sample->ekf_attitude_rad[1],
            &sample->ekf_attitude_rad[2]);
        flight_recorder_fill_p_diag_sqrt(input->ekf, sample->ekf_p_diag_sqrt);
    }

    if (input->guidance_commands_valid && input->guidance_commands != NULL) {
        sample->guidance_commands_valid = true;
        sample->desired_heading_rad = input->guidance_commands->desired_heading;
        sample->desired_speed_mps = input->guidance_commands->desired_speed;
        sample->desired_climb_mps = input->guidance_commands->desired_climb;
    }

    sample->mission_state = static_cast<uint8_t>(input->mission_state);
    sample->return_home_active = input->return_home_active;
    sample->active_waypoint_index = input->active_waypoint_index;

    sample->health_score = input->health_score;
    sample->health_mode = static_cast<uint8_t>(input->health_mode);
    sample->power_state = input->power_state;
    sample->shutdown_latched = input->shutdown_latched;
    sample->odom_fault = input->odom_fault;
    sample->waypoint_count = input->waypoint_count;
    sample->bsp_bus_status = input->bsp_bus_status;
    sample->radio_dropped_packets = input->radio_dropped_packets;
    sample->temperature_c = input->temperature_c;
    sample->loop_us = input->loop_us;

    if (input->runtime_health != NULL) {
        sample->missed_ticks = input->runtime_health->missed_ticks;
        sample->max_loop_us = input->runtime_health->max_loop_us;
    }

    return true;
}

void flight_recorder_write_csv_header(FILE *file)
{
    if (file == NULL) {
        return;
    }

    fprintf(
        file,
        "Timestamp_ms,Escenario,Modo,Calidad,Satelites,"
        "Pos_X,Pos_Y,Pos_Z,Vel_X,Vel_Y,Vel_Z,Rumbo,"
        "CrossTrack_m,AlongTrack_m,OdomFault,HealthScore,HealthMode,"
        "PowerState,ShutdownLatched,WaypointCount,BspBus,RadioDroppedPackets,"
        "Imu_Valid,Imu_Accel_X,Imu_Accel_Y,Imu_Accel_Z,Imu_Gyro_X,Imu_Gyro_Y,Imu_Gyro_Z,"
        "Gnss_Fix,Gnss_Lat,Gnss_Lon,Gnss_Alt,Gnss_Speed,Gnss_Course,"
        "Ekf_Active,Ekf_Outlier,Gnss_NIS,Gnss_Innov_N,Gnss_Innov_E,Gnss_Innov_D,"
        "Gnss_Accept_Count,Gnss_Reject_Count,Gnss_Update_Accepted,"
        "Ekf_Bias_Ax,Ekf_Bias_Ay,Ekf_Bias_Az,Ekf_Bias_Gx,Ekf_Bias_Gy,Ekf_Bias_Gz,"
        "Ekf_Att_Roll,Ekf_Att_Pitch,Ekf_Att_Yaw,"
        "Ekf_P_PosN,Ekf_P_PosE,Ekf_P_PosD,Ekf_P_VelN,Ekf_P_VelE,Ekf_P_VelD,"
        "Ekf_P_Roll,Ekf_P_Pitch,Ekf_P_Yaw,"
        "Ekf_P_BiasAx,Ekf_P_BiasAy,Ekf_P_BiasAz,Ekf_P_BiasGx,Ekf_P_BiasGy,Ekf_P_BiasGz,"
        "Desired_Heading,Desired_Speed,Desired_Climb,CrossTrack_Signed_m,"
        "Mission_State,Active_WP,Return_Home,Loop_us,Missed_Ticks,Max_Loop_us,Temperature_C\n");
}

bool flight_recorder_write_csv_row(FILE *file, const FlightRecorderSample *sample)
{
    if (file == NULL || sample == NULL) {
        return false;
    }

    const char *mode_name = "UNKNOWN";
    switch (sample->nav_mode) {
    case NAV_MODE_INITIALIZING:
        mode_name = "INIT";
        break;
    case NAV_MODE_GPS:
        mode_name = "GPS";
        break;
    case NAV_MODE_DEAD_RECKONING:
        mode_name = "DR";
        break;
    case NAV_MODE_HYBRID:
        mode_name = "HYBRID";
        break;
    default:
        break;
    }

    const char *health_name = "UNKNOWN";
    switch (static_cast<NavHealthMode>(sample->health_mode)) {
    case HEALTH_NOMINAL:
        health_name = "NOMINAL";
        break;
    case HEALTH_DEGRADED:
        health_name = "DEGRADED";
        break;
    case HEALTH_CRITICAL:
        health_name = "CRITICAL";
        break;
    default:
        break;
    }

    fprintf(
        file,
        "%u,%s,%s,%.6f,%u,"
        "%.8f,%.8f,%.4f,%.6f,%.6f,%.6f,%.4f,"
        "%.4f,%.4f,%u,%u,%s,"
        "%u,%u,%zu,%u,%u,"
        "%u,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,"
        "%u,%.8f,%.8f,%.4f,%.6f,%.4f,"
        "%u,%u,%.6f,%.6f,%.6f,%.6f,"
        "%u,%u,%u,"
        "%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,"
        "%.6f,%.6f,%.6f,"
        "%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,"
        "%.6f,%.6f,%.6f,"
        "%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,"
        "%.6f,%.6f,%.6f,%.4f,"
        "%u,%zu,%u,%u,%u,%u,%.2f\n",
        sample->timestamp_ms,
        sample->scenario_name,
        mode_name,
        sample->estimate_quality,
        static_cast<unsigned>(sample->gnss_satellites),
        sample->pos_x,
        sample->pos_y,
        sample->pos_z,
        sample->vel_x,
        sample->vel_y,
        sample->vel_z,
        sample->heading_deg,
        sample->cross_track_m,
        sample->along_track_m,
        static_cast<unsigned>(sample->odom_fault),
        static_cast<unsigned>(sample->health_score),
        health_name,
        static_cast<unsigned>(sample->power_state),
        sample->shutdown_latched ? 1U : 0U,
        sample->waypoint_count,
        static_cast<unsigned>(sample->bsp_bus_status),
        static_cast<unsigned>(sample->radio_dropped_packets),
        sample->imu_valid ? 1U : 0U,
        sample->imu_accel_mps2[0],
        sample->imu_accel_mps2[1],
        sample->imu_accel_mps2[2],
        sample->imu_gyro_radps[0],
        sample->imu_gyro_radps[1],
        sample->imu_gyro_radps[2],
        sample->gnss_fix_valid ? 1U : 0U,
        sample->gnss_lat_deg,
        sample->gnss_lon_deg,
        sample->gnss_alt_m,
        sample->gnss_speed_mps,
        sample->gnss_course_deg,
        sample->ekf_active ? 1U : 0U,
        sample->ekf_outlier ? 1U : 0U,
        sample->gnss_nis,
        sample->gnss_innovation_ned[0],
        sample->gnss_innovation_ned[1],
        sample->gnss_innovation_ned[2],
        static_cast<unsigned>(sample->gnss_accept_count),
        static_cast<unsigned>(sample->gnss_reject_count),
        sample->gnss_update_accepted ? 1U : 0U,
        sample->ekf_bias_accel[0],
        sample->ekf_bias_accel[1],
        sample->ekf_bias_accel[2],
        sample->ekf_bias_gyro[0],
        sample->ekf_bias_gyro[1],
        sample->ekf_bias_gyro[2],
        sample->ekf_attitude_rad[0],
        sample->ekf_attitude_rad[1],
        sample->ekf_attitude_rad[2],
        sample->ekf_p_diag_sqrt[INS_POS_N],
        sample->ekf_p_diag_sqrt[INS_POS_E],
        sample->ekf_p_diag_sqrt[INS_POS_D],
        sample->ekf_p_diag_sqrt[INS_VEL_N],
        sample->ekf_p_diag_sqrt[INS_VEL_E],
        sample->ekf_p_diag_sqrt[INS_VEL_D],
        sample->ekf_p_diag_sqrt[INS_ATT_ROLL],
        sample->ekf_p_diag_sqrt[INS_ATT_PITCH],
        sample->ekf_p_diag_sqrt[INS_ATT_YAW],
        sample->ekf_p_diag_sqrt[INS_BIAS_AX],
        sample->ekf_p_diag_sqrt[INS_BIAS_AY],
        sample->ekf_p_diag_sqrt[INS_BIAS_AZ],
        sample->ekf_p_diag_sqrt[INS_BIAS_GX],
        sample->ekf_p_diag_sqrt[INS_BIAS_GY],
        sample->ekf_p_diag_sqrt[INS_BIAS_GZ],
        sample->desired_heading_rad,
        sample->desired_speed_mps,
        sample->desired_climb_mps,
        sample->cross_track_signed_m,
        static_cast<unsigned>(sample->mission_state),
        sample->active_waypoint_index,
        sample->return_home_active ? 1U : 0U,
        sample->loop_us,
        sample->missed_ticks,
        sample->max_loop_us,
        sample->temperature_c);

    return true;
}
