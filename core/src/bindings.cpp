#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include "docking/types.hpp"
#include "docking/geometric_se3.hpp"
#include "docking/allocator.hpp"
#include "docking/fsm.hpp"
#include "docking/compensator.hpp"
#include "docking/safety.hpp"
#include "docking/event_features.hpp"
#include "docking/research_compensation.hpp"

namespace py = pybind11;
using namespace docking;

template <size_t N>
RotorThrusts<N> rotor_array(const std::vector<double>& input) {
    if (input.size() != N) throw std::invalid_argument("Incorrect motor thrust vector length");
    RotorThrusts<N> result;
    std::copy(input.begin(), input.end(), result.thrusts.begin());
    return result;
}

PYBIND11_MODULE(docking_core_py, m) {
    m.doc() = "C++17 Core Library for Dual Quadrotor Docking Control";

    // Vec3
    py::class_<Vec3>(m, "Vec3")
        .def(py::init<double, double, double>(), py::arg("x")=0.0, py::arg("y")=0.0, py::arg("z")=0.0)
        .def_readwrite("x", &Vec3::x)
        .def_readwrite("y", &Vec3::y)
        .def_readwrite("z", &Vec3::z)
        .def("__repr__", [](const Vec3& v) {
            return "[" + std::to_string(v.x) + ", " + std::to_string(v.y) + ", " + std::to_string(v.z) + "]";
        });

    // Mat3
    py::class_<Mat3>(m, "Mat3")
        .def(py::init<>())
        .def(py::init([](const std::array<double, 9>& values) {
            Mat3 result; result.m = values; return result;
        }))
        .def_readwrite("m", &Mat3::m)
        .def_static("Identity", &Mat3::Identity)
        .def_static("Zero", &Mat3::Zero);

    // Quat
    py::class_<Quat>(m, "Quat")
        .def(py::init<double, double, double, double>(), py::arg("w")=1.0, py::arg("x")=0.0, py::arg("y")=0.0, py::arg("z")=0.0)
        .def_readwrite("w", &Quat::w)
        .def_readwrite("x", &Quat::x)
        .def_readwrite("y", &Quat::y)
        .def_readwrite("z", &Quat::z);

    // RigidBodyState
    py::class_<RigidBodyState>(m, "RigidBodyState")
        .def(py::init<>())
        .def_readwrite("pos", &RigidBodyState::pos)
        .def_readwrite("vel", &RigidBodyState::vel)
        .def_readwrite("quat", &RigidBodyState::quat)
        .def_readwrite("omega", &RigidBodyState::omega)
        .def_readwrite("timestamp", &RigidBodyState::timestamp);

    // Setpoint
    py::class_<Setpoint>(m, "Setpoint")
        .def(py::init<>())
        .def_readwrite("pos", &Setpoint::pos)
        .def_readwrite("vel", &Setpoint::vel)
        .def_readwrite("acc", &Setpoint::acc)
        .def_readwrite("yaw", &Setpoint::yaw)
        .def_readwrite("yaw_rate", &Setpoint::yaw_rate);

    // WrenchCommand
    py::class_<WrenchCommand>(m, "WrenchCommand")
        .def(py::init<>())
        .def_readwrite("thrust", &WrenchCommand::thrust)
        .def_readwrite("torque", &WrenchCommand::torque);

    // FlightMode
    py::enum_<FlightMode>(m, "FlightMode")
        .value("FREE", FlightMode::FREE)
        .value("APPROACH", FlightMode::APPROACH)
        .value("ALIGN", FlightMode::ALIGN)
        .value("CAPTURE_PENDING", FlightMode::CAPTURE_PENDING)
        .value("CONNECTED", FlightMode::CONNECTED)
        .value("RELEASE_PREP", FlightMode::RELEASE_PREP)
        .value("SEPARATING", FlightMode::SEPARATING)
        .value("RECOVERY", FlightMode::RECOVERY)
        .value("ABORT", FlightMode::ABORT)
        .export_values();

    // SE3Gains
    py::class_<SE3Gains>(m, "SE3Gains")
        .def(py::init<>())
        .def_readwrite("kp", &SE3Gains::kp)
        .def_readwrite("kv", &SE3Gains::kv)
        .def_readwrite("ki", &SE3Gains::ki)
        .def_readwrite("i_limit", &SE3Gains::i_limit)
        .def_readwrite("kr", &SE3Gains::kr)
        .def_readwrite("kw", &SE3Gains::kw)
        .def_readwrite("ki_rot", &SE3Gains::ki_rot)
        .def_readwrite("i_limit_rot", &SE3Gains::i_limit_rot);

    // QuadParams
    py::class_<QuadParams>(m, "QuadParams")
        .def(py::init<>())
        .def_readwrite("mass", &QuadParams::mass)
        .def_readwrite("gravity", &QuadParams::gravity)
        .def_readwrite("arm_length", &QuadParams::arm_length)
        .def_readwrite("inertia", &QuadParams::inertia)
        .def_readwrite("min_thrust", &QuadParams::min_thrust)
        .def_readwrite("max_thrust", &QuadParams::max_thrust)
        .def_readwrite("torque_ratio", &QuadParams::torque_ratio)
        .def_readwrite("time_constant", &QuadParams::time_constant)
        .def_readwrite("slew_rate_limit", &QuadParams::slew_rate_limit)
        .def_readwrite("rotor_angles_deg", &QuadParams::rotor_angles_deg)
        .def_readwrite("rotor_directions", &QuadParams::rotor_directions);

    // DockingParams
    py::class_<DockingParams>(m, "DockingParams")
        .def(py::init<>())
        .def_readwrite("connector_offset_q1", &DockingParams::connector_offset_q1)
        .def_readwrite("connector_offset_q2", &DockingParams::connector_offset_q2)
        .def_readwrite("pos_tol", &DockingParams::pos_tol)
        .def_readwrite("rot_tol", &DockingParams::rot_tol)
        .def_readwrite("lin_vel_tol", &DockingParams::lin_vel_tol)
        .def_readwrite("ang_vel_tol", &DockingParams::ang_vel_tol)
        .def_readwrite("dwell_time", &DockingParams::dwell_time);

    // GeometricSE3Controller
    py::class_<GeometricSE3Controller>(m, "GeometricSE3Controller")
        .def(py::init<double, const Mat3&, const SE3Gains&>())
        .def("reset", &GeometricSE3Controller::reset)
        .def("update_gains", &GeometricSE3Controller::update_gains)
        .def("update_inertia", &GeometricSE3Controller::update_inertia)
        .def("compute", &GeometricSE3Controller::compute)
        .def("get_attitude_error", &GeometricSE3Controller::get_attitude_error)
        .def("get_rate_error", &GeometricSE3Controller::get_rate_error)
        .def("get_pos_error", &GeometricSE3Controller::get_pos_error)
        .def("get_desired_rotation", &GeometricSE3Controller::get_desired_rotation);

    // SingleQuadAllocator
    py::class_<SingleQuadAllocator>(m, "SingleQuadAllocator")
        .def(py::init<const QuadParams&>())
        .def("reset", &SingleQuadAllocator::reset)
        .def("set_previous_thrusts", [](SingleQuadAllocator& self, const std::vector<double>& t) {
            self.set_previous_thrusts(rotor_array<4>(t));
        })
        .def("get_thrusts", [](const SingleQuadAllocator& self) { return self.get_thrusts().thrusts; })
        .def("achieved_wrench", [](const SingleQuadAllocator& self, const std::vector<double>& t) {
            return self.achieved_wrench(rotor_array<4>(t));
        })
        .def("achieved_force", [](const SingleQuadAllocator& self, const std::vector<double>& t) {
            return self.achieved_force(rotor_array<4>(t));
        })
        .def("allocate", [](SingleQuadAllocator& self, const WrenchCommand& w, double dt) {
            auto arr = self.allocate(w, dt);
            std::vector<double> res(4);
            for (size_t i = 0; i < 4; ++i) res[i] = arr[i];
            return res;
        });

    // ConnectedQuadAllocator
    py::class_<ConnectedQuadAllocator>(m, "ConnectedQuadAllocator")
        .def(py::init<const QuadParams&, const QuadParams&, const DockingParams&>())
        .def("reset", &ConnectedQuadAllocator::reset)
        .def("set_previous_thrusts", [](ConnectedQuadAllocator& self, const std::vector<double>& t) {
            self.set_previous_thrusts(rotor_array<8>(t));
        })
        .def("get_thrusts", [](const ConnectedQuadAllocator& self) { return self.get_thrusts().thrusts; })
        .def("update_geometry", &ConnectedQuadAllocator::update_geometry)
        .def("set_relative_pose", &ConnectedQuadAllocator::update_geometry)
        .def("com_offset", &ConnectedQuadAllocator::com_offset)
        .def("achieved_wrench", [](const ConnectedQuadAllocator& self, const std::vector<double>& t) {
            return self.achieved_wrench(rotor_array<8>(t));
        })
        .def("achieved_force", [](const ConnectedQuadAllocator& self, const std::vector<double>& t) {
            return self.achieved_force(rotor_array<8>(t));
        })
        .def("total_mass", &ConnectedQuadAllocator::total_mass)
        .def("aggregate_inertia", &ConnectedQuadAllocator::aggregate_inertia)
        .def("allocate", [](ConnectedQuadAllocator& self, const WrenchCommand& w, double dt) {
            auto arr = self.allocate(w, dt);
            std::vector<double> res(8);
            for (size_t i = 0; i < 8; ++i) res[i] = arr[i];
            return res;
        });

    // RelativeConnectorState
    py::class_<RelativeConnectorState>(m, "RelativeConnectorState")
        .def(py::init<>())
        .def_readwrite("pos_err", &RelativeConnectorState::pos_err)
        .def_readwrite("vel_rel", &RelativeConnectorState::vel_rel)
        .def_readwrite("angle_err_rad", &RelativeConnectorState::angle_err_rad)
        .def_readwrite("ang_vel_rel", &RelativeConnectorState::ang_vel_rel)
        .def_readwrite("distance", &RelativeConnectorState::distance);

    // HybridFSM
    py::class_<HybridFSM>(m, "HybridFSM")
        .def(py::init<const DockingParams&>())
        .def("reset", &HybridFSM::reset)
        .def("initialize_connected", &HybridFSM::initialize_connected, py::arg("current_time") = 0.0)
        .def("update_params", &HybridFSM::update_params)
        .def("update", &HybridFSM::update)
        .def("command_approach", &HybridFSM::command_approach)
        .def("command_undock", &HybridFSM::command_undock)
        .def("command_abort", &HybridFSM::command_abort)
        .def("set_capture_contact", &HybridFSM::set_capture_contact)
        .def("invalidate_capture_dwell", &HybridFSM::invalidate_capture_dwell)
        .def("current_mode", &HybridFSM::current_mode)
        .def("is_latched", &HybridFSM::is_latched)
        .def("rel_state", &HybridFSM::rel_state)
        .def("handover_alpha", &HybridFSM::handover_alpha)
        .def("last_event_time", &HybridFSM::last_event_time)
        .def("last_event_name", &HybridFSM::last_event_name);

    // CompensatorType
    py::enum_<CompensatorType>(m, "CompensatorType")
        .value("NONE", CompensatorType::NONE)
        .value("FIXED_FF", CompensatorType::FIXED_FF)
        .value("ILC", CompensatorType::ILC)
        .value("ECLR", CompensatorType::ECLR)
        .export_values();

    // EventCompensator
    py::class_<EventCompensator>(m, "EventCompensator")
        .def(py::init<>())
        .def("reset", &EventCompensator::reset)
        .def("configure", &EventCompensator::configure)
        .def("set_fixed_coefficients", &EventCompensator::set_fixed_coefficients)
        .def("set_eclr_matrices", &EventCompensator::set_eclr_matrices, py::arg("A"), py::arg("G"), py::arg("regularization") = 1e-4)
        .def("trigger_event", &EventCompensator::trigger_event)
        .def("evaluate", &EventCompensator::evaluate)
        .def("is_active", &EventCompensator::is_active)
        .def("active_coefficients", &EventCompensator::active_coefficients);

    // Independent research protocol; existing bindings retain their semantics.
    py::enum_<ResearchEvent>(m, "ResearchEvent")
        .value("CONTACT", ResearchEvent::CONTACT)
        .value("DOCK", ResearchEvent::DOCK)
        .value("UNDOCK", ResearchEvent::UNDOCK);
    py::class_<EventFeaturePacket>(m, "EventFeaturePacket")
        .def(py::init<ResearchEvent, const std::vector<double>&, int>(),
             py::arg("event"), py::arg("values"), py::arg("schema_version") = 1)
        .def_property_readonly("event", &EventFeaturePacket::event)
        .def_property_readonly("values", &EventFeaturePacket::values)
        .def_property_readonly("schema_version", &EventFeaturePacket::schema_version);
    py::class_<EventMotorFeedforward>(m, "EventMotorFeedforward")
        .def(py::init<>())
        .def("configure", &EventMotorFeedforward::configure,
             py::arg("window_s"), py::arg("decay_times_s"), py::arg("max_motor_delta_N"))
        .def("set_coefficients", &EventMotorFeedforward::set_coefficients)
        .def("trigger_event", &EventMotorFeedforward::trigger_event)
        .def("reset", &EventMotorFeedforward::reset)
        .def("evaluate", &EventMotorFeedforward::evaluate)
        .def("basis", &EventMotorFeedforward::basis)
        .def("is_active", &EventMotorFeedforward::is_active)
        .def("active_coefficients", &EventMotorFeedforward::active_coefficients)
        .def_property_readonly("basis_count", &EventMotorFeedforward::basis_count)
        .def_property_readonly("window_s", &EventMotorFeedforward::window_s)
        .def_property_readonly("max_motor_delta_N", &EventMotorFeedforward::max_motor_delta_N);
    py::class_<AngularDisturbanceObserver>(m, "AngularDisturbanceObserver")
        .def(py::init<const Mat3&, double, double>(), py::arg("inertia"),
             py::arg("cutoff_hz") = 8.0, py::arg("max_torque_Nm") = 0.03)
        .def("update", &AngularDisturbanceObserver::update,
             py::arg("omega_body_radps"), py::arg("previous_actual_motor_torque_Nm"),
             py::arg("dt_s"))
        .def("reset", &AngularDisturbanceObserver::reset)
        .def("update_inertia", &AngularDisturbanceObserver::update_inertia)
        .def("disturbance_estimate", &AngularDisturbanceObserver::disturbance_estimate)
        .def("compensation", &AngularDisturbanceObserver::compensation)
        .def_property_readonly("initialized", &AngularDisturbanceObserver::initialized);
}
