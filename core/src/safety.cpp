#include "docking/safety.hpp"
#include <cmath>

namespace docking {

SafetyMonitor::SafetyMonitor(const SafetyLimits& limits)
    : limits_(limits) {
    reset();
}

void SafetyMonitor::reset() {
    tripped_ = false;
    trip_reason_ = "OK";
    last_valid_time_ = 0.0;
}

bool SafetyMonitor::check_state(const RigidBodyState& state, double current_time) {
    if (tripped_) return false;

    // 1. Tilt angle: body z-axis dot world z-axis = R(2, 2) = cos(tilt)
    Mat3 R = state.rot_matrix();
    double cos_tilt = R(2, 2);
    double tilt = std::acos(std::max(-1.0, std::min(1.0, cos_tilt)));
    if (tilt > limits_.max_tilt_angle_rad) {
        tripped_ = true;
        trip_reason_ = "EXCESSIVE_TILT";
        return false;
    }

    // 2. Angular rates
    if (state.omega.norm() > limits_.max_angular_rate_radps) {
        tripped_ = true;
        trip_reason_ = "EXCESSIVE_ANGULAR_RATE";
        return false;
    }

    // 3. Position boundary
    if (state.pos.z < limits_.min_altitude_m || state.pos.z > limits_.max_altitude_m) {
        tripped_ = true;
        trip_reason_ = "ALTITUDE_BREACH";
        return false;
    }
    double horiz_dist = std::sqrt(state.pos.x * state.pos.x + state.pos.y * state.pos.y);
    if (horiz_dist > limits_.max_horizontal_radius_m) {
        tripped_ = true;
        trip_reason_ = "GEOFENCE_BREACH";
        return false;
    }

    // 4. Heartbeat/staleness
    if (last_valid_time_ > 0.0 && (current_time - last_valid_time_) > limits_.heartbeat_timeout_s) {
        tripped_ = true;
        trip_reason_ = "HEARTBEAT_TIMEOUT";
        return false;
    }

    last_valid_time_ = current_time;
    return true;
}

} // namespace docking
