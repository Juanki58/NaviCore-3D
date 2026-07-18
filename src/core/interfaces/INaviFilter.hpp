#ifndef INAVI_FILTER_HPP
#define INAVI_FILTER_HPP

#include <string>

struct NaviState {
    double timestamp_s;
    double pos_ned[3];      // Posición Norte-Este-Down (m)
    float vel_body[3];      // Velocidad en marco cuerpo (m/s)
    float att_euler[3];     // Actitud Roll, Pitch, Yaw (rad)
    float accel_bias[3];    // Sesgo estimado de acelerómetro
    float gyro_bias[3];     // Sesgo estimado de giróscopo
    float cov_pos_diag[3];  // Diagonal de la covarianza de posición
    float cov_att_diag[3];  // Diagonal de la covarianza de actitud
    float nis;              // NIS de la última actualización
};

class INaviFilter {
public:
    virtual ~INaviFilter() = default;

    virtual void initialize(const NaviState &initial_state) = 0;

    virtual void predict(
        double dt_s,
        const float accel_mps2[3],
        const float gyro_rads[3]) = 0;

    virtual void update_gnss(const double pos_ned_m[3], const float std_dev_m[3]) = 0;

    virtual void apply_constraints(
        bool is_stopping,
        float lateral_std_mps,
        float vertical_std_mps) = 0;

    virtual NaviState get_state() const = 0;

    virtual std::string get_filter_name() const = 0;
};

#endif // INAVI_FILTER_HPP
