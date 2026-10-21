#include "docking/research_compensation.hpp"

#include <stdexcept>

namespace docking {
namespace {
constexpr double pi = 3.141592653589793238462643383279502884;

bool finite(const Vec3& v) {
    return std::isfinite(v.x) && std::isfinite(v.y) && std::isfinite(v.z);
}

size_t event_index(ResearchEvent event) {
    const int index = static_cast<int>(event);
    if (index < 0 || index > 2) throw std::invalid_argument("Unknown research event");
    return static_cast<size_t>(index);
}

void validate_inertia(const Mat3& inertia) {
    for (double value : inertia.m)
        if (!std::isfinite(value)) throw std::invalid_argument("Inertia must be finite");
    for (int row = 0; row < 3; ++row)
        for (int column = row + 1; column < 3; ++column)
            if (std::abs(inertia(row, column) - inertia(column, row)) > 1e-12)
                throw std::invalid_argument("Inertia must be symmetric");
    // Sylvester's criterion for a symmetric positive-definite 3x3 matrix.
    const double minor2 = inertia(0, 0) * inertia(1, 1) - inertia(0, 1) * inertia(1, 0);
    const double determinant =
        inertia(0, 0) * (inertia(1, 1) * inertia(2, 2) - inertia(1, 2) * inertia(2, 1)) -
        inertia(0, 1) * (inertia(1, 0) * inertia(2, 2) - inertia(1, 2) * inertia(2, 0)) +
        inertia(0, 2) * (inertia(1, 0) * inertia(2, 1) - inertia(1, 1) * inertia(2, 0));
    if (!(inertia(0, 0) > 0.0 && minor2 > 0.0 && determinant > 0.0))
        throw std::invalid_argument("Inertia must be positive definite");
}
} // namespace

EventFeaturePacket::EventFeaturePacket(ResearchEvent event,
                                       const std::vector<double>& values,
                                       int schema_version)
    : event_(event) {
    event_index(event);
    if (schema_version != supported_schema_version)
        throw std::invalid_argument("Unsupported event feature schema version; expected 1");
    if (values.size() != dimension)
        throw std::invalid_argument("Event feature schema v1 requires exactly 32 values");
    for (double value : values)
        if (!std::isfinite(value)) throw std::invalid_argument("Event features must be finite");
    std::copy(values.begin(), values.end(), values_.begin());
}

EventMotorFeedforward::EventMotorFeedforward() {
    configure(window_s_, decay_times_s_, max_motor_delta_N_);
}

void EventMotorFeedforward::configure(double window_s,
                                     const std::vector<double>& decay_times_s,
                                     double max_motor_delta_N) {
    if (!std::isfinite(window_s) || window_s <= 0.0)
        throw std::invalid_argument("Event window must be positive and finite");
    if (decay_times_s.empty()) throw std::invalid_argument("At least one event basis is required");
    for (double tau : decay_times_s)
        if (!std::isfinite(tau) || tau <= 0.0)
            throw std::invalid_argument("Basis decay times must be positive and finite");
    if (!std::isfinite(max_motor_delta_N) || max_motor_delta_N < 0.0)
        throw std::invalid_argument("Motor compensation limit must be nonnegative and finite");
    window_s_ = window_s;
    decay_times_s_ = decay_times_s;
    max_motor_delta_N_ = max_motor_delta_N;
    // No silently reused coefficients after a basis or window change.
    for (auto& event_coefficients : coefficients_)
        event_coefficients.assign(8 * basis_count(), 0.0);
    reset();
}

void EventMotorFeedforward::set_coefficients(ResearchEvent event,
                                           const std::vector<double>& coefficients) {
    const size_t index = event_index(event);
    if (coefficients.size() != 8 * basis_count())
        throw std::invalid_argument("Expected 8 times basis_count motor-major coefficients");
    for (double value : coefficients)
        if (!std::isfinite(value)) throw std::invalid_argument("Coefficients must be finite");
    coefficients_[index] = coefficients;
}

void EventMotorFeedforward::trigger_event(ResearchEvent event, double trigger_time_s) {
    const size_t index = event_index(event);
    if (!std::isfinite(trigger_time_s)) throw std::invalid_argument("Trigger time must be finite");
    active_coefficients_ = coefficients_[index];
    trigger_time_s_ = trigger_time_s;
    triggered_ = true;
}

void EventMotorFeedforward::reset() {
    triggered_ = false;
    trigger_time_s_ = 0.0;
    active_coefficients_.assign(8 * basis_count(), 0.0);
}

std::vector<double> EventMotorFeedforward::basis(double elapsed_s) const {
    if (!std::isfinite(elapsed_s)) throw std::invalid_argument("Elapsed time must be finite");
    std::vector<double> values(basis_count(), 0.0);
    if (elapsed_s <= 0.0 || elapsed_s >= window_s_) return values;
    const double envelope = std::pow(std::sin(pi * elapsed_s / window_s_), 2);
    for (size_t k = 0; k < basis_count(); ++k)
        values[k] = envelope * std::exp(-elapsed_s / decay_times_s_[k]);
    return values;
}

bool EventMotorFeedforward::is_active(double current_time_s) const {
    if (!std::isfinite(current_time_s)) throw std::invalid_argument("Sample time must be finite");
    const double elapsed = current_time_s - trigger_time_s_;
    return triggered_ && elapsed >= 0.0 && elapsed < window_s_;
}

std::array<double, 8> EventMotorFeedforward::evaluate(double current_time_s) const {
    std::array<double, 8> output{};
    if (!is_active(current_time_s)) return output;
    const auto values = basis(current_time_s - trigger_time_s_);
    for (size_t motor = 0; motor < 8; ++motor)
        for (size_t k = 0; k < basis_count(); ++k)
            output[motor] += active_coefficients_[motor * basis_count() + k] * values[k];
    for (double value : output)
        if (!std::isfinite(value)) throw std::overflow_error("Event motor profile overflow");
    // A zero sum on EACH quad preserves each collective thrust, including when
    // the two body thrust axes differ at physical capture. Do not clip motors
    // individually: uniform scaling retains these zero-sum constraints.
    for (size_t quad = 0; quad < 2; ++quad) {
        double mean = 0.0;
        for (size_t motor = 4 * quad; motor < 4 * quad + 4; ++motor) mean += output[motor] / 4.0;
        for (size_t motor = 4 * quad; motor < 4 * quad + 4; ++motor) output[motor] -= mean;
    }
    double peak = 0.0;
    for (double value : output) peak = std::max(peak, std::abs(value));
    if (peak > max_motor_delta_N_)
        for (double& value : output) value *= max_motor_delta_N_ / peak;
    return output;
}

AngularDisturbanceObserver::AngularDisturbanceObserver(const Mat3& inertia,
                                                       double cutoff_hz,
                                                       double max_torque_Nm)
    : inertia_(inertia), cutoff_hz_(cutoff_hz), max_torque_Nm_(max_torque_Nm) {
    validate_inertia(inertia_);
    if (!std::isfinite(cutoff_hz_) || cutoff_hz_ <= 0.0)
        throw std::invalid_argument("Observer cutoff must be positive and finite");
    if (!std::isfinite(max_torque_Nm_) || max_torque_Nm_ < 0.0)
        throw std::invalid_argument("Observer torque limit must be nonnegative and finite");
}

void AngularDisturbanceObserver::reset() {
    previous_omega_body_radps_ = Vec3{};
    disturbance_estimate_Nm_ = Vec3{};
    initialized_ = false;
}

void AngularDisturbanceObserver::update_inertia(const Mat3& inertia) {
    validate_inertia(inertia);
    inertia_ = inertia;
    reset();
}

WrenchCommand AngularDisturbanceObserver::compensation() const {
    return WrenchCommand{0.0, -1.0 * disturbance_estimate_Nm_};
}

WrenchCommand AngularDisturbanceObserver::update(const Vec3& omega_body_radps,
                                                const Vec3& previous_actual_motor_torque_Nm,
                                                double dt_s) {
    if (!finite(omega_body_radps) || !finite(previous_actual_motor_torque_Nm))
        throw std::invalid_argument("Observer angular velocity and motor torque must be finite");
    if (!std::isfinite(dt_s) || dt_s <= 0.0)
        throw std::invalid_argument("Observer sample interval must be positive and finite");
    if (!initialized_) {
        previous_omega_body_radps_ = omega_body_radps;
        initialized_ = true;
        return compensation();
    }
    const Vec3 acceleration = (omega_body_radps - previous_omega_body_radps_) / dt_s;
    const Vec3 gyroscopic = previous_omega_body_radps_.cross(inertia_ * previous_omega_body_radps_);
    const Vec3 residual = inertia_ * acceleration + gyroscopic - previous_actual_motor_torque_Nm;
    if (!finite(residual)) throw std::overflow_error("Observer residual overflow");
    const double alpha = -std::expm1(-2.0 * pi * cutoff_hz_ * dt_s);
    disturbance_estimate_Nm_ += (residual - disturbance_estimate_Nm_) * alpha;
    const double magnitude = disturbance_estimate_Nm_.norm();
    if (magnitude > max_torque_Nm_)
        disturbance_estimate_Nm_ = disturbance_estimate_Nm_ * (max_torque_Nm_ / magnitude);
    previous_omega_body_radps_ = omega_body_radps;
    return compensation();
}

} // namespace docking
