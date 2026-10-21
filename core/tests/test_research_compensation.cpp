#include "docking/research_compensation.hpp"

#include <cassert>
#include <iostream>
#include <limits>
#include <stdexcept>

using namespace docking;

namespace {
constexpr double pi = 3.14159265358979323846;

template <typename F> void rejects(F operation) {
    bool rejected = false;
    try { operation(); }
    catch (const std::invalid_argument&) { rejected = true; }
    assert(rejected);
}

Mat3 inverse(const Mat3& a) {
    const double d = a(0, 0) * (a(1, 1) * a(2, 2) - a(1, 2) * a(2, 1)) -
                     a(0, 1) * (a(1, 0) * a(2, 2) - a(1, 2) * a(2, 0)) +
                     a(0, 2) * (a(1, 0) * a(2, 1) - a(1, 1) * a(2, 0));
    return Mat3{(a(1, 1) * a(2, 2) - a(1, 2) * a(2, 1)) / d,
                (a(0, 2) * a(2, 1) - a(0, 1) * a(2, 2)) / d,
                (a(0, 1) * a(1, 2) - a(0, 2) * a(1, 1)) / d,
                (a(1, 2) * a(2, 0) - a(1, 0) * a(2, 2)) / d,
                (a(0, 0) * a(2, 2) - a(0, 2) * a(2, 0)) / d,
                (a(0, 2) * a(1, 0) - a(0, 0) * a(1, 2)) / d,
                (a(1, 0) * a(2, 1) - a(1, 1) * a(2, 0)) / d,
                (a(0, 1) * a(2, 0) - a(0, 0) * a(2, 1)) / d,
                (a(0, 0) * a(1, 1) - a(0, 1) * a(1, 0)) / d};
}

Mat3 test_inertia() {
    // Non-diagonal inertia exposes assumptions hidden by a spherical body.
    return Mat3{0.0082, 0.0004, -0.0002,
                0.0004, 0.0091, 0.0003,
                -0.0002, 0.0003, 0.0145};
}

void test_feature_schema() {
    std::vector<double> features(32, 0.0);
    features[31] = 0.025;
    EventFeaturePacket packet(ResearchEvent::UNDOCK, features);
    assert(packet.schema_version() == 1);
    assert(packet.event() == ResearchEvent::UNDOCK);
    assert(packet.values()[31] == 0.025);
    rejects([&] { EventFeaturePacket p(ResearchEvent::DOCK, std::vector<double>(31)); });
    rejects([&] { EventFeaturePacket p(ResearchEvent::DOCK, std::vector<double>(33)); });
    rejects([&] { EventFeaturePacket p(ResearchEvent::DOCK, features, 2); });
    rejects([&] { EventFeaturePacket p(static_cast<ResearchEvent>(4), features); });
    features[0] = std::numeric_limits<double>::quiet_NaN();
    rejects([&] { EventFeaturePacket p(ResearchEvent::CONTACT, features); });
    std::cout << "[PASS] feature_schema_exact32_version_event_finiteness\n";
}

void test_basis_finite_support_and_smooth_boundaries() {
    EventMotorFeedforward ff;
    ff.configure(1.2, {0.08, 0.16, 0.28, 0.45}, 0.25);
    for (double t : {-1.0, 0.0, 1.2, 1.3})
        for (double b : ff.basis(t)) assert(b == 0.0);
    const auto basis = ff.basis(0.2);
    for (size_t k = 0; k < 4; ++k) assert(basis[k] > 0.0 && basis[k] < 1.0);
    // One-sided first derivative tends to zero at BOTH endpoints.
    constexpr double epsilon = 1e-7;
    for (double b : ff.basis(epsilon)) assert(std::abs(b / epsilon) < 1e-5);
    for (double b : ff.basis(1.2 - epsilon)) assert(std::abs(b / epsilon) < 1e-5);
    rejects([&] { ff.basis(std::numeric_limits<double>::infinity()); });
    std::cout << "[PASS] finite_window_basis_zero_value_and_derivative_at_endpoints\n";
}

void test_feedforward_motor_order_zero_collectives_and_bounds() {
    EventMotorFeedforward ff;
    ff.configure(1.2, {0.2}, 0.25);
    const std::vector<double> coefficients{10, 0, 0, 0, -2, 1, 4, 5};
    ff.set_coefficients(ResearchEvent::DOCK, coefficients);
    ff.trigger_event(ResearchEvent::DOCK, 2.0);
    const auto output = ff.evaluate(2.2);
    for (size_t quad = 0; quad < 2; ++quad) {
        double sum = 0.0;
        for (size_t motor = quad * 4; motor < quad * 4 + 4; ++motor) {
            sum += output[motor];
            assert(std::abs(output[motor]) <= 0.25 + 1e-14);
        }
        assert(std::abs(sum) < 1e-14);
    }
    // Q1's projected coefficients [7.5, -2.5, -2.5, -2.5] are uniformly
    // scaled; Q2 is scaled by that SAME factor, preserving motor directions.
    assert(std::abs(output[0] - 0.25) < 1e-14);
    assert(std::abs(output[1] + 0.25 / 3.0) < 1e-14);
    assert(output[4] < 0.0 && output[7] > 0.0);
    std::cout << "[PASS] feedforward_motor_order_two_zero_collectives_and_common_bound\n";
}

void test_events_are_independent_and_coefficients_snapshot_on_trigger() {
    EventMotorFeedforward ff;
    ff.configure(1.2, {0.2}, 0.25);
    const std::vector<double> dock{0.1, -0.1, 0.1, -0.1, 0, 0, 0, 0};
    auto undock = dock;
    for (double& value : undock) value = -value;
    ff.set_coefficients(ResearchEvent::DOCK, dock);
    ff.set_coefficients(ResearchEvent::UNDOCK, undock);
    ff.trigger_event(ResearchEvent::DOCK, 1.0);
    const auto original = ff.evaluate(1.2);
    ff.set_coefficients(ResearchEvent::DOCK, std::vector<double>(8, 0.0));
    assert(ff.evaluate(1.2) == original);
    ff.trigger_event(ResearchEvent::UNDOCK, 2.0);
    assert(std::abs(ff.evaluate(2.2)[0] + original[0]) < 1e-14);
    ff.trigger_event(ResearchEvent::CONTACT, 3.0);
    for (double value : ff.evaluate(3.2)) assert(value == 0.0);
    std::cout << "[PASS] independent_contact_dock_undock_and_causal_coefficient_snapshot\n";
}

void test_feedforward_timing_reset_and_validation() {
    EventMotorFeedforward ff;
    ff.configure(1.0, {0.1}, 0.1);
    ff.set_coefficients(ResearchEvent::CONTACT, {1, -1, 1, -1, 1, -1, 1, -1});
    ff.trigger_event(ResearchEvent::CONTACT, 2.0);
    for (double t : {1.9, 2.0, 3.0, 3.1})
        for (double value : ff.evaluate(t)) assert(value == 0.0);
    assert(ff.is_active(2.2));
    ff.reset();
    assert(!ff.is_active(2.2));
    for (double value : ff.evaluate(2.2)) assert(value == 0.0);
    rejects([&] { ff.configure(0, {0.1}, 0.25); });
    rejects([&] { ff.configure(1, {}, 0.25); });
    rejects([&] { ff.configure(1, {-0.1}, 0.25); });
    rejects([&] { ff.configure(1, {0.1}, -0.25); });
    rejects([&] { ff.set_coefficients(ResearchEvent::DOCK, std::vector<double>(7)); });
    rejects([&] { ff.trigger_event(ResearchEvent::DOCK, std::numeric_limits<double>::quiet_NaN()); });
    rejects([&] { ff.evaluate(std::numeric_limits<double>::infinity()); });
    std::cout << "[PASS] feedforward_causal_timing_reset_and_invalid_inputs\n";
}

void test_observer_known_constant_disturbance_variable_dt_and_sign() {
    const Mat3 inertia = test_inertia(), inv = inverse(inertia);
    AngularDisturbanceObserver observer(inertia, 8.0, 0.03);
    const Vec3 disturbance{0.012, -0.008, 0.006};
    Vec3 omega{0.3, -0.2, 0.1};
    assert(observer.update(omega, {}, 0.005).torque.norm() == 0.0);
    double elapsed = 0.0;
    for (size_t sample = 0; sample < 200; ++sample) {
        const double dt = sample % 2 ? 0.004 : 0.006;
        const Vec3 actual_motor_torque{0.002 * std::sin(sample * 0.05), -0.001, 0.001};
        omega += inv * (actual_motor_torque + disturbance - omega.cross(inertia * omega)) * dt;
        const auto compensation = observer.update(omega, actual_motor_torque, dt);
        elapsed += dt;
        const Vec3 expected_estimate = disturbance * (1.0 - std::exp(-2.0 * pi * 8.0 * elapsed));
        assert((observer.disturbance_estimate() - expected_estimate).norm() < 2e-14);
        assert(compensation.thrust == 0.0);
        assert((compensation.torque + observer.disturbance_estimate()).norm() < 1e-15);
    }
    assert((observer.disturbance_estimate() - disturbance).norm() < 1e-12);
    std::cout << "[PASS] observer_known_disturbance_variable_dt_actual_input_and_cancellation_sign\n";
}

void test_observer_no_disturbance_nonzero_gyroscopic_dynamics() {
    const Mat3 inertia = test_inertia(), inv = inverse(inertia);
    AngularDisturbanceObserver observer(inertia);
    Vec3 omega{2.0, -1.0, 1.5};
    observer.update(omega, {}, 0.005);
    for (size_t sample = 0; sample < 200; ++sample) {
        const Vec3 motor{0.003 * std::cos(sample * 0.1), -0.002, 0.001};
        omega += inv * (motor - omega.cross(inertia * omega)) * 0.005;
        observer.update(omega, motor, 0.005);
        assert(observer.disturbance_estimate().norm() < 1e-14);
    }
    std::cout << "[PASS] observer_no_disturbance_offdiagonal_inertia_and_gyroscopic_term\n";
}

void test_observer_body_axes_rotation_equivariance() {
    const double angle = 0.7;
    const Mat3 rotation{std::cos(angle), -std::sin(angle), 0,
                        std::sin(angle), std::cos(angle), 0,
                        0, 0, 1};
    const Mat3 inertia = test_inertia();
    const Mat3 rotated_inertia = rotation * inertia * rotation.transpose();
    AngularDisturbanceObserver a(inertia), b(rotated_inertia);
    Vec3 omega{0.3, 0.4, -0.1};
    a.update(omega, {}, 0.005);
    b.update(rotation * omega, {}, 0.005);
    for (int sample = 0; sample < 20; ++sample) {
        omega += Vec3(0.01, -0.006, 0.008);
        const Vec3 motor{0.002, 0.003, 0.004};
        a.update(omega, motor, 0.005);
        b.update(rotation * omega, rotation * motor, 0.005);
        assert((b.disturbance_estimate() - rotation * a.disturbance_estimate()).norm() < 1e-14);
    }
    std::cout << "[PASS] observer_persistent_body_axes_and_rotation_equivariance\n";
}

void test_observer_norm_bound_reset_and_inertia_handover() {
    AngularDisturbanceObserver observer(Mat3::Identity(), 8.0, 0.03);
    observer.update({}, {}, 0.005);
    const auto bounded = observer.update(Vec3(10, 20, 30), {}, 0.005);
    assert(std::abs(bounded.torque.norm() - 0.03) < 1e-14);
    observer.reset();
    assert(!observer.initialized());
    assert(observer.compensation().torque.norm() == 0.0);
    // A velocity discontinuity due to a mode change must not create a sample.
    assert(observer.update(Vec3(-5, 2, 3), {}, 0.005).torque.norm() == 0.0);
    observer.update_inertia(test_inertia());
    assert(!observer.initialized());
    assert(observer.update(Vec3(4, -6, 9), {}, 0.005).torque.norm() == 0.0);
    std::cout << "[PASS] observer_torque_norm_bound_event_reset_and_inertia_handover\n";
}

void test_observer_invalid_inputs_and_failed_update_preserves_state() {
    rejects([] { AngularDisturbanceObserver o(Mat3::Zero()); });
    rejects([] { AngularDisturbanceObserver o(Mat3::Identity(), 0.0); });
    rejects([] { AngularDisturbanceObserver o(Mat3::Identity(), 8.0, -0.1); });
    Mat3 nonsymmetric = Mat3::Identity();
    nonsymmetric(0, 1) = 0.1;
    rejects([&] { AngularDisturbanceObserver o(nonsymmetric); });
    AngularDisturbanceObserver observer(test_inertia());
    rejects([&] { observer.update({}, {}, 0.0); });
    rejects([&] { observer.update({}, {}, -0.005); });
    rejects([&] { observer.update(Vec3(std::numeric_limits<double>::quiet_NaN(), 0, 0), {}, 0.005); });
    assert(!observer.initialized());
    observer.update(Vec3(0.1, 0.2, 0.3), {}, 0.005);
    rejects([&] { observer.update_inertia(Mat3::Zero()); });
    assert(observer.initialized());
    std::cout << "[PASS] observer_invalid_dt_measurement_settings_and_inertia_rejected\n";
}
} // namespace

int main() {
    test_feature_schema();
    test_basis_finite_support_and_smooth_boundaries();
    test_feedforward_motor_order_zero_collectives_and_bounds();
    test_events_are_independent_and_coefficients_snapshot_on_trigger();
    test_feedforward_timing_reset_and_validation();
    test_observer_known_constant_disturbance_variable_dt_and_sign();
    test_observer_no_disturbance_nonzero_gyroscopic_dynamics();
    test_observer_body_axes_rotation_equivariance();
    test_observer_norm_bound_reset_and_inertia_handover();
    test_observer_invalid_inputs_and_failed_update_preserves_state();
    std::cout << "All 10 research compensation tests passed.\n";
    return 0;
}
