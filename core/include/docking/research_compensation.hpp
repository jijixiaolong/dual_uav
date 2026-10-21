#pragma once

#include "docking/types.hpp"
#include <array>
#include <vector>

namespace docking {

// Separate from the legacy EventCompensator feature protocol. Offline learning
// selects coefficients; the deployable runtime performs no fitting or oracle use.
enum class ResearchEvent : int { CONTACT = 0, DOCK = 1, UNDOCK = 2 };

// Schema names and normalization are persisted by the offline experiment code.
// This boundary rejects mismatched event/length/version without truncation or
// fallback to the older, incompatible 13/9-element feature representations.
class EventFeaturePacket {
public:
    static constexpr size_t dimension = 32;
    static constexpr int supported_schema_version = 1;
    EventFeaturePacket(ResearchEvent event, const std::vector<double>& values,
                       int schema_version = supported_schema_version);
    ResearchEvent event() const { return event_; }
    const std::array<double, dimension>& values() const { return values_; }
    int schema_version() const { return supported_schema_version; }
private:
    ResearchEvent event_;
    std::array<double, dimension> values_{};
};

class EventMotorFeedforward {
public:
    EventMotorFeedforward();

    // Coefficients have units N and shape [8 motors][decay_times.size()],
    // flattened motor-major. Motors 0..3 belong to Q1 and 4..7 to Q2.
    // b_k(t) = sin^2(pi*t/window)*exp(-t/decay_times[k]), 0 < t < window.
    // Both the value and first derivative vanish at both window boundaries.
    void configure(double window_s, const std::vector<double>& decay_times_s,
                   double max_motor_delta_N);
    void set_coefficients(ResearchEvent event, const std::vector<double>& coefficients);
    void trigger_event(ResearchEvent event, double trigger_time_s);
    void reset();

    std::array<double, 8> evaluate(double current_time_s) const;
    std::vector<double> basis(double elapsed_s) const;
    bool is_active(double current_time_s) const;
    const std::vector<double>& active_coefficients() const { return active_coefficients_; }
    size_t basis_count() const { return decay_times_s_.size(); }
    double window_s() const { return window_s_; }
    double max_motor_delta_N() const { return max_motor_delta_N_; }

private:
    double window_s_{1.2};
    std::vector<double> decay_times_s_{0.08, 0.16, 0.28, 0.45};
    double max_motor_delta_N_{0.25};
    std::array<std::vector<double>, 3> coefficients_;
    std::vector<double> active_coefficients_;
    double trigger_time_s_{0.0};
    bool triggered_{false};
};

class AngularDisturbanceObserver {
public:
    // inertia, angular velocity, and actual rotor torque must all use the same
    // persistent body axes. Returns {0 N, -estimated_disturbance N*m}.
    AngularDisturbanceObserver(const Mat3& inertia, double cutoff_hz = 8.0,
                               double max_torque_Nm = 0.03);

    // The first sample only primes the differentiator and returns zero. At all
    // later samples, previous_actual_motor_torque is the actual rotor torque
    // over the preceding sample interval, including actuator dynamics. The
    // forward-Euler residual is J*domega/dt + omega_prev x J*omega_prev - tau.
    // Contact/weld forces are never inputs. A mode or event change requires reset.
    WrenchCommand update(const Vec3& omega_body_radps,
                         const Vec3& previous_actual_motor_torque_Nm,
                         double dt_s);
    void reset();
    // Changing aggregate geometry changes inertia and also resets the derivative.
    void update_inertia(const Mat3& inertia);
    Vec3 disturbance_estimate() const { return disturbance_estimate_Nm_; }
    WrenchCommand compensation() const;
    bool initialized() const { return initialized_; }

private:
    Mat3 inertia_;
    double cutoff_hz_;
    double max_torque_Nm_;
    Vec3 previous_omega_body_radps_{};
    Vec3 disturbance_estimate_Nm_{};
    bool initialized_{false};
};

} // namespace docking
