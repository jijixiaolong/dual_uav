#pragma once

#include "docking/types.hpp"

namespace docking {

struct SafetyLimits {
    double max_tilt_angle_rad{0.785}; // 45 degrees
    double max_angular_rate_radps{12.0};
    double min_altitude_m{0.10};
    double max_altitude_m{3.5};
    double max_horizontal_radius_m{5.0};
    double heartbeat_timeout_s{0.050}; // 50 ms
};

class SafetyMonitor {
public:
    SafetyMonitor(const SafetyLimits& limits = SafetyLimits{});

    void reset();
    bool check_state(const RigidBodyState& state, double current_time);

    bool is_tripped() const { return tripped_; }
    const std::string& trip_reason() const { return trip_reason_; }

    // Clamp thrust array safely
    template <size_t N>
    static void sanitize_thrusts(RotorThrusts<N>& thrusts, double min_t, double max_t) {
        for (size_t i = 0; i < N; ++i) {
            double v = thrusts[i];
            if (std::isnan(v) || std::isinf(v)) {
                v = min_t;
            }
            thrusts[i] = std::max(min_t, std::min(max_t, v));
        }
    }

private:
    SafetyLimits limits_;
    bool tripped_{false};
    std::string trip_reason_{"OK"};
    double last_valid_time_{0.0};
};

} // namespace docking
