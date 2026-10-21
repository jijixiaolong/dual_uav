#include <cassert>
#include <iostream>
#include <cmath>
#include <random>
#include <limits>
#include <stdexcept>
#include "docking/types.hpp"
#include "docking/geometric_se3.hpp"
#include "docking/allocator.hpp"
#include "docking/fsm.hpp"
#include "docking/compensator.hpp"
#include "docking/safety.hpp"

using namespace docking;

void test_math_foundation() {
    Vec3 a(1.0, 2.0, 3.0);
    Vec3 b(4.0, 5.0, 6.0);
    assert(std::abs(a.dot(b) - 32.0) < 1e-9);

    Vec3 c = a.cross(b);
    assert(std::abs(c.x - (-3.0)) < 1e-9);
    assert(std::abs(c.y - (6.0)) < 1e-9);
    assert(std::abs(c.z - (-3.0)) < 1e-9);

    Mat3 S = Mat3::Skew(a);
    Vec3 un_skew = Mat3::Vee(S);
    assert((a - un_skew).norm() < 1e-9);

    Quat q = Quat::from_rotation_matrix(Mat3::Identity());
    assert(std::abs(q.w - 1.0) < 1e-6);
    assert(std::abs(q.x) < 1e-6);

    std::cout << "[PASS] test_math_foundation\n";
}

void test_geometric_se3_hover() {
    QuadParams qp;
    SE3Gains gains;
    GeometricSE3Controller ctrl(qp.mass, qp.inertia, gains);

    RigidBodyState state;
    state.pos = Vec3(0, 0, 1.0);
    state.vel = Vec3(0, 0, 0);
    state.quat = Quat(1, 0, 0, 0);
    state.omega = Vec3(0, 0, 0);

    Setpoint sp;
    sp.pos = Vec3(0, 0, 1.0);
    sp.vel = Vec3(0, 0, 0);
    sp.acc = Vec3(0, 0, 0);
    sp.yaw = 0.0;

    WrenchCommand cmd = ctrl.compute(state, sp, 0.005);
    double expected_thrust = qp.mass * qp.gravity;
    assert(std::abs(cmd.thrust - expected_thrust) < 1e-4);
    assert(cmd.torque.norm() < 1e-6);

    std::cout << "[PASS] test_geometric_se3_hover\n";
}

void test_geometric_attitude_error_scale() {
    QuadParams qp;
    SE3Gains gains;
    gains.ki_rot = Vec3{};
    GeometricSE3Controller ctrl(qp.mass, qp.inertia, gains);
    RigidBodyState state;
    state.pos = Vec3(0, 0, 1.0);
    const double roll = 0.1;
    state.quat = Quat(std::cos(roll / 2.0), std::sin(roll / 2.0), 0, 0);
    const auto cmd = ctrl.compute(state, Setpoint{}, 0.005);
    assert(std::abs(ctrl.get_attitude_error().x - std::sin(roll)) < 1e-12);
    assert(std::abs(cmd.torque.x + gains.kr.x * std::sin(roll)) < 1e-12);
    for (size_t i = 0; i < 9; ++i)
        assert(std::abs(ctrl.get_desired_rotation().m[i] - Mat3::Identity().m[i]) < 1e-12);
    Setpoint moving;
    moving.acc = Vec3(0.3, -0.1, 0.0);
    gains.ki = Vec3{};
    ctrl.update_gains(gains);
    ctrl.compute(state, moving, 0.005);
    const auto desired_rotation = ctrl.get_desired_rotation();
    const auto expected_axis = Vec3(0.3, -0.1, qp.gravity).normalized();
    const auto actual_axis = desired_rotation * Vec3(0, 0, 1);
    assert((expected_axis - actual_axis).norm() < 1e-12);
    const auto orthogonal = desired_rotation.transpose() * desired_rotation;
    for (size_t i = 0; i < 9; ++i)
        assert(std::abs(orthogonal.m[i] - Mat3::Identity().m[i]) < 1e-12);
    ctrl.reset();
    for (size_t i = 0; i < 9; ++i)
        assert(std::abs(ctrl.get_desired_rotation().m[i] - Mat3::Identity().m[i]) < 1e-12);
    std::cout << "[PASS] test_geometric_attitude_error_scale\n";
}

void test_single_quad_allocator() {
    QuadParams qp;
    SingleQuadAllocator alloc(qp);

    WrenchCommand cmd;
    cmd.thrust = qp.mass * qp.gravity;
    cmd.torque = Vec3(0, 0, 0);

    RotorThrusts4 thrusts = alloc.allocate(cmd, 0.005);
    double expected_per_rotor = cmd.thrust / 4.0;
    for (size_t i = 0; i < 4; ++i) {
        assert(std::abs(thrusts[i] - expected_per_rotor) < 1e-4);
    }

    std::cout << "[PASS] test_single_quad_allocator\n";
}

void test_connected_allocator_and_parallel_axis() {
    QuadParams q1, q2;
    DockingParams dp;
    ConnectedQuadAllocator alloc(q1, q2, dp);

    assert(std::abs(alloc.total_mass() - 2.0) < 1e-9);
    Mat3 Jc = alloc.aggregate_inertia();
    // Diagonal elements of inertia must be strictly positive
    assert(Jc(0, 0) > q1.inertia(0, 0));
    assert(Jc(1, 1) > 0.0);
    assert(Jc(2, 2) > q1.inertia(2, 2));

    WrenchCommand cmd;
    cmd.thrust = alloc.total_mass() * q1.gravity;
    cmd.torque = Vec3(0, 0, 0);

    RotorThrusts8 thrusts = alloc.allocate(cmd, 0.005);
    double expected_per_rotor = cmd.thrust / 8.0;
    for (size_t i = 0; i < 8; ++i) {
        assert(std::abs(thrusts[i] - expected_per_rotor) < 1e-3);
    }

    std::cout << "[PASS] test_connected_allocator_and_parallel_axis\n";
}

void test_connected_actual_capture_geometry() {
    QuadParams q1, q2;
    q2.mass = 2.0;
    q2.inertia = Mat3{0.01, 0, 0, 0, 0.02, 0, 0, 0, 0.03};
    ConnectedQuadAllocator alloc(q1, q2, DockingParams{});
    Vec3 separation(0.12, 0.45, 0.06);
    const double half_yaw = std::acos(-1.0) / 4.0;
    alloc.update_geometry(separation, Quat(std::cos(half_yaw), 0, 0, std::sin(half_yaw)));
    assert((alloc.com_offset() - separation * (2.0 / 3.0)).norm() < 1e-12);
    const auto inertia = alloc.aggregate_inertia();
    const double reduced_mass = q1.mass * q2.mass / (q1.mass + q2.mass);
    assert(std::abs(inertia(0, 0) - (q1.inertia(0, 0) + q2.inertia(1, 1) +
           reduced_mass * (separation.y * separation.y + separation.z * separation.z))) < 1e-12);
    assert(std::abs(inertia(0, 1) + reduced_mass * separation.x * separation.y) < 1e-12);
    assert(std::abs(inertia(0, 2) + reduced_mass * separation.x * separation.z) < 1e-12);
    std::cout << "[PASS] test_connected_actual_capture_geometry\n";
}

void test_feasible_wrench_reconstruction_and_redistribution() {
    QuadParams q1, q2;
    q2.mass = 0.8;
    q2.max_thrust = 3.2;
    q2.arm_length = 0.19;
    q2.torque_ratio = 0.02;
    q2.rotor_directions = {-1, 1, -1, 1};
    q2.rotor_angles_deg = {20, 110, 200, 290};
    ConnectedQuadAllocator allocator(q1, q2, DockingParams{});
    allocator.update_geometry(Vec3(0.08, 0.44, 0.03), Quat{});
    std::mt19937 random(20261021);
    std::uniform_real_distribution<double> unit(0.0, 1.0);
    for (size_t trial = 0; trial < 200; ++trial) {
        RotorThrusts8 feasible;
        for (size_t i = 0; i < 8; ++i) {
            const auto& p = i < 4 ? q1 : q2;
            // Include exact active bounds, where clipping an unconstrained
            // pseudoinverse can lose an otherwise feasible wrench.
            const double fraction = trial % 3 == 0 ? (unit(random) > 0.5 ? 1.0 : 0.0) : unit(random);
            feasible[i] = p.min_thrust + fraction * (p.max_thrust - p.min_thrust);
        }
        const auto desired = allocator.achieved_wrench(feasible);
        const auto allocated = allocator.allocate(desired, 100.0);
        const auto actual = allocator.achieved_wrench(allocated);
        assert(std::abs(desired.thrust - actual.thrust) < 2e-6);
        assert((desired.torque - actual.torque).norm() < 2e-6);
        for (size_t i = 0; i < 8; ++i) {
            const auto& p = i < 4 ? q1 : q2;
            assert(allocated[i] >= p.min_thrust - 1e-12);
            assert(allocated[i] <= p.max_thrust + 1e-12);
        }
    }
    // Single-quad geometry and spin direction are configuration inputs too.
    SingleQuadAllocator single(q2);
    RotorThrusts4 feasible{{1.1, 2.6, 1.9, 0.8}};
    const auto desired = single.achieved_wrench(feasible);
    const auto actual = single.achieved_wrench(single.allocate(desired, 100.0));
    assert(std::abs(desired.thrust - actual.thrust) < 1e-7);
    assert((desired.torque - actual.torque).norm() < 1e-7);
    std::cout << "[PASS] test_feasible_wrench_reconstruction_and_redistribution (200 feasible commands)\n";
}

void test_allocator_limits_slew_and_handover() {
    QuadParams q1, q2;
    q1.slew_rate_limit = 3.0;
    q2.slew_rate_limit = 1.0;
    q2.max_thrust = 3.0;
    ConnectedQuadAllocator allocator(q1, q2, DockingParams{});
    RotorThrusts8 initial{{1, 2, 3, 4, 1, 2, 2.5, 2.8}};
    allocator.set_previous_thrusts(initial);
    const WrenchCommand saturated{100.0, Vec3(20, -30, 10)};
    const auto held = allocator.allocate(saturated, 0.0);
    for (size_t i = 0; i < 8; ++i) assert(held[i] == initial[i]);
    for (size_t step = 0; step < 100; ++step) {
        const auto previous = allocator.get_thrusts();
        const double dt = step % 2 ? 0.004 : 0.006;
        const auto actual = allocator.allocate(saturated, dt);
        for (size_t i = 0; i < 8; ++i) {
            const auto& p = i < 4 ? q1 : q2;
            assert(std::abs(actual[i] - previous[i]) <= p.slew_rate_limit * dt + 1e-12);
            assert(actual[i] >= p.min_thrust - 1e-12 && actual[i] <= p.max_thrust + 1e-12);
        }
    }
    SingleQuadAllocator single(q1);
    RotorThrusts4 inherited;
    for (size_t i = 0; i < 4; ++i) inherited[i] = allocator.get_thrusts()[i];
    single.set_previous_thrusts(inherited);
    const auto first_independent = single.allocate(WrenchCommand{0.0, Vec3(-20, 30, -10)}, 0.005);
    for (size_t i = 0; i < 4; ++i)
        assert(std::abs(first_independent[i] - inherited[i]) <= q1.slew_rate_limit * 0.005 + 1e-12);
    bool rejected = false;
    try { allocator.allocate(saturated, -0.001); }
    catch (const std::invalid_argument&) { rejected = true; }
    assert(rejected);
    rejected = false;
    try { allocator.allocate(WrenchCommand{std::numeric_limits<double>::quiet_NaN(), Vec3{}}, 0.005); }
    catch (const std::invalid_argument&) { rejected = true; }
    assert(rejected);
    std::cout << "[PASS] test_allocator_limits_slew_and_handover\n";
}

void test_tilted_capture_rotor_axes() {
    QuadParams q1, q2;
    ConnectedQuadAllocator allocator(q1, q2, DockingParams{});
    const double pitch = 0.04;
    allocator.update_geometry(Vec3(0, 0.44, 0), Quat(std::cos(pitch / 2), 0, std::sin(pitch / 2), 0));
    RotorThrusts8 only_second{{0, 0, 0, 0, 2, 2, 2, 2}};
    const auto force = allocator.achieved_force(only_second);
    assert(std::abs(force.x - 8.0 * std::sin(pitch)) < 1e-12);
    assert(std::abs(force.y) < 1e-12);
    assert(std::abs(force.z - 8.0 * std::cos(pitch)) < 1e-12);
    std::cout << "[PASS] test_tilted_capture_rotor_axes\n";
}

void test_small_capture_tilt_does_not_create_control_preload() {
    QuadParams q1, q2;
    SingleQuadAllocator single_q1(q1), single_q2(q2);
    for (double angle_deg : {-0.5, 0.5}) {
        ConnectedQuadAllocator allocator(q1, q2, DockingParams{});
        const double half_angle = angle_deg * std::acos(-1.0) / 360.0;
        allocator.update_geometry(Vec3(0, 0.44, 0), Quat(std::cos(half_angle), std::sin(half_angle), 0, 0));
        const WrenchCommand hover{(q1.mass + q2.mass) * q1.gravity, Vec3{}};
        RotorThrusts8 motors;
        for (size_t tick = 0; tick < 200; ++tick) motors = allocator.allocate(hover, 0.005);
        const auto realized = allocator.achieved_wrench(motors);
        assert(std::abs(realized.thrust - hover.thrust) < 1e-7);
        assert(realized.torque.norm() < 1e-7);
        RotorThrusts4 first, second;
        for (size_t i = 0; i < 4; ++i) {
            first[i] = motors[i]; second[i] = motors[i + 4];
            assert(std::abs(first[i] - q1.mass * q1.gravity / 4) < 0.02);
            assert(std::abs(second[i] - q2.mass * q2.gravity / 4) < 0.02);
        }
        assert(single_q1.achieved_wrench(first).torque.norm() < 0.01);
        assert(single_q2.achieved_wrench(second).torque.norm() < 0.01);
        // The small physical lateral force remains visible instead of being
        // traded for saturated motors and a near-1 Nm internal roll moment.
        assert(std::abs(allocator.achieved_force(motors).y) > 0.01);
    }
    std::cout << "[PASS] test_small_capture_tilt_does_not_create_control_preload\n";
}

void test_hybrid_fsm() {
    DockingParams dp;
    HybridFSM fsm(dp);
    assert(fsm.current_mode() == FlightMode::FREE);

    fsm.command_approach();
    RigidBodyState q1, q2;
    q1.pos = Vec3(0, -0.6, 1.0);
    q2.pos = Vec3(0, 0.6, 1.0);

    fsm.update(q1, q2, 0.1, 0.005);
    assert(fsm.current_mode() == FlightMode::APPROACH);

    // Position them in exact alignment
    q1.pos = Vec3(0, -0.22, 1.0);
    q2.pos = Vec3(0, 0.22, 1.0);
    fsm.update(q1, q2, 0.200, 0.005);
    assert(fsm.current_mode() == FlightMode::ALIGN);
    // Perfect kinematics without an actual connector contact must never latch.
    for (int i = 0; i < 20; ++i) fsm.update(q1, q2, 0.205 + i * 0.005, 0.005);
    assert(fsm.current_mode() == FlightMode::ALIGN);
    fsm.set_capture_contact(true);
    fsm.update(q1, q2, 0.305, 0.005);
    assert(fsm.current_mode() == FlightMode::CAPTURE_PENDING);

    // Stay for dwell time (30 ms)
    for (int i = 0; i < 10; ++i) {
        fsm.update(q1, q2, 0.310 + i * 0.005, 0.005);
    }
    assert(fsm.current_mode() == FlightMode::CONNECTED);
    assert(fsm.is_latched());

    // Initial preconnected hover is a fixture, not a fabricated docking event.
    fsm.initialize_connected();
    assert(fsm.is_latched());
    assert(fsm.last_event_name() == "NONE");
    assert(fsm.handover_alpha() == 1.0);
    fsm.command_undock();
    fsm.update(q1, q2, 0.0, 0.005);
    assert(fsm.current_mode() == FlightMode::RELEASE_PREP);
    for (int i = 1; i <= 21; ++i) fsm.update(q1, q2, i * 0.005, 0.005);
    assert(fsm.current_mode() == FlightMode::SEPARATING);
    assert(!fsm.is_latched());
    assert(fsm.last_event_name() == "UNDOCK");

    std::cout << "[PASS] test_hybrid_fsm\n";
}

void test_safety_monitor() {
    SafetyMonitor sm;
    RigidBodyState safe_state;
    safe_state.pos = Vec3(0, 0, 1.0);
    assert(sm.check_state(safe_state, 0.01));

    RigidBodyState breached_state;
    breached_state.pos = Vec3(0, 0, 10.0); // beyond 3.5m ceiling
    assert(!sm.check_state(breached_state, 0.02));
    assert(sm.is_tripped());

    std::cout << "[PASS] test_safety_monitor\n";
}

int main() {
    std::cout << "Running Core C++ Unit Tests...\n";
    test_math_foundation();
    test_geometric_se3_hover();
    test_geometric_attitude_error_scale();
    test_single_quad_allocator();
    test_connected_allocator_and_parallel_axis();
    test_connected_actual_capture_geometry();
    test_feasible_wrench_reconstruction_and_redistribution();
    test_allocator_limits_slew_and_handover();
    test_tilted_capture_rotor_axes();
    test_small_capture_tilt_does_not_create_control_preload();
    test_hybrid_fsm();
    test_safety_monitor();
    std::cout << "All C++ Unit Tests Passed Successfully!\n";
    return 0;
}
