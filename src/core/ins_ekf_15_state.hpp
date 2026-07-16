#pragma once

#include "interfaces/INaviFilter.hpp"
#include "ins_ekf.hpp"
#include "sensor_types.hpp"

#include <cstdint>
#include <memory>

/** Adaptador ESKF 15 estados -> INaviFilter (Benchmark Engine). */
class InsEkf15State : public INaviFilter {
public:
    InsEkf15State();

    void initialize(const NaviState &initial_state) override;
    void predict(double dt_s, const float accel_mps2[3], const float gyro_rads[3]) override;
    void update_gnss(const double pos_ned_m[3], const float std_dev_m[3]) override;
    void apply_constraints(
        bool is_stopping,
        float lateral_std_mps,
        float vertical_std_mps) override;
    NaviState get_state() const override;
    std::string get_filter_name() const override;

    bool seed_from_gnss_sample(const GpsSample &gps, NavDomain domain);
    bool seed_from_ned_fix(const double pos_ned_m[3], NavDomain domain);
    bool seed_from_ned_fix(
        const double pos_ned_m[3],
        const double ref_lla_deg[3],
        NavDomain domain);
    bool update_gnss_from_sample(const GpsSample &gps);
    bool is_initialized() const;
    const InsEkfFilter &native() const;
    InsEkfFilter &native();

    void set_nhc_measurement_stds(float lateral_std_mps, float vertical_std_mps);
    void sync_simulation_clock_ms(uint32_t t_ms);

private:
    void sync_timestamp_from_ms(uint32_t t_ms);
    void body_velocity_from_ned(float out_body[3]) const;

    InsEkfFilter ekf_;
    double timestamp_s_;
    bool run_zupt_after_predict_;
    float pending_lateral_std_mps_;
    float pending_vertical_std_mps_;
};

std::unique_ptr<INaviFilter> create_default_navi_filter();

const InsEkfFilter *navi_filter_try_get_ins_ekf(const INaviFilter *filter);
InsEkfFilter *navi_filter_try_get_ins_ekf_mut(INaviFilter *filter);
