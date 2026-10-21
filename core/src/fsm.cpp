#include "docking/fsm.hpp"
#include <cmath>
#include <stdexcept>

namespace docking {

HybridFSM::HybridFSM(const DockingParams& params)
    : params_(params) {
    reset();
}

void HybridFSM::reset() {
    current_mode_ = FlightMode::FREE;
    dwell_timer_ = 0.0;
    transition_time_ = 0.0;
    last_event_time_ = 0.0;
    last_event_name_ = "NONE";
    approach_requested_ = false;
    undock_requested_ = false;
    abort_requested_ = false;
    capture_contact_ = false;
    rel_state_ = RelativeConnectorState{};
    mode_start_time_ = 0.0;
}

void HybridFSM::command_approach() {
    approach_requested_ = true;
}

void HybridFSM::initialize_connected(double current_time) {
    if (!std::isfinite(current_time) || current_time < 0.0)
        throw std::invalid_argument("Invalid preconnected fixture start time");
    reset();
    current_mode_ = FlightMode::CONNECTED;
    mode_start_time_ = current_time;
    transition_time_ = 0.15;
}

void HybridFSM::command_undock() {
    undock_requested_ = true;
}

void HybridFSM::command_abort() {
    abort_requested_ = true;
}

void HybridFSM::compute_relative_connector_state(const RigidBodyState& q1, const RigidBodyState& q2) {
    Mat3 R1 = q1.rot_matrix();
    Mat3 R2 = q2.rot_matrix();

    // World position of connector 1 and connector 2
    Vec3 p_c1 = q1.pos + R1 * params_.connector_offset_q1;
    Vec3 p_c2 = q2.pos + R2 * params_.connector_offset_q2;

    rel_state_.pos_err = p_c2 - p_c1;
    rel_state_.distance = rel_state_.pos_err.norm();

    // Linear velocity of connectors in world frame
    Vec3 v_c1 = q1.vel + R1 * (q1.omega.cross(params_.connector_offset_q1));
    Vec3 v_c2 = q2.vel + R2 * (q2.omega.cross(params_.connector_offset_q2));
    rel_state_.vel_rel = v_c2 - v_c1;

    // Relative attitude
    Mat3 R_rel = R1.transpose() * R2;
    // Angle from trace: tr(R) = 1 + 2*cos(theta)
    double tr = R_rel(0, 0) + R_rel(1, 1) + R_rel(2, 2);
    double cos_theta = std::max(-1.0, std::min(1.0, (tr - 1.0) * 0.5));
    rel_state_.angle_err_rad = std::acos(cos_theta);

    rel_state_.ang_vel_rel = q2.omega - (R_rel.transpose() * q1.omega);
}

bool HybridFSM::check_latch_criteria() const {
    if (!capture_contact_) return false;
    if (!std::isfinite(rel_state_.distance) || !std::isfinite(rel_state_.angle_err_rad) ||
        !std::isfinite(rel_state_.vel_rel.norm()) || !std::isfinite(rel_state_.ang_vel_rel.norm())) return false;
    if (rel_state_.distance > params_.pos_tol) return false;
    if (rel_state_.vel_rel.norm() > params_.lin_vel_tol) return false;
    if (rel_state_.angle_err_rad > params_.rot_tol) return false;
    if (rel_state_.ang_vel_rel.norm() > params_.ang_vel_tol) return false;
    return true;
}

double HybridFSM::handover_alpha() const {
    double duration = 0.15; // 150 ms blend
    if (transition_time_ >= duration) return 1.0;
    double t = transition_time_ / duration;
    // Smooth 3*t^2 - 2*t^3 s-curve
    return t * t * (3.0 - 2.0 * t);
}

FlightMode HybridFSM::update(const RigidBodyState& q1,
                             const RigidBodyState& q2,
                             double current_time,
                             double dt) {
    if (!std::isfinite(current_time) || !std::isfinite(dt) || dt <= 0.)
        throw std::invalid_argument("FSM requires finite time and positive dt");
    transition_time_ += dt;
    compute_relative_connector_state(q1, q2);

    if (abort_requested_) {
        current_mode_ = FlightMode::ABORT;
        dwell_timer_ = 0.;
        last_event_name_ = "ABORT";
        last_event_time_ = current_time;
        return current_mode_;
    }

    switch (current_mode_) {
        case FlightMode::FREE: {
            if (approach_requested_) {
                current_mode_ = FlightMode::APPROACH;
                mode_start_time_ = current_time;
                transition_time_ = 0.0;
                approach_requested_ = false;
            }
            break;
        }
        case FlightMode::APPROACH: {
            // If distance is closing below 0.15m, move to ALIGN
            if (rel_state_.distance < 0.15) {
                current_mode_ = FlightMode::ALIGN;
                mode_start_time_ = current_time;
                transition_time_ = 0.0;
            }
            break;
        }
        case FlightMode::ALIGN: {
            if (check_latch_criteria()) {
                current_mode_ = FlightMode::CAPTURE_PENDING;
                dwell_timer_ = 0.0;
                mode_start_time_ = current_time;
                transition_time_ = 0.0;
            } else if (rel_state_.distance > 0.30) {
                // Fallen out of alignment funnel
                current_mode_ = FlightMode::APPROACH;
            }
            break;
        }
        case FlightMode::CAPTURE_PENDING: {
            if (check_latch_criteria()) {
                dwell_timer_ += dt;
                if (dwell_timer_ >= params_.dwell_time - 1e-12) {
                    current_mode_ = FlightMode::CONNECTED;
                    last_event_time_ = current_time;
                    last_event_name_ = "DOCK";
                    mode_start_time_ = current_time;
                    transition_time_ = 0.0;
                }
            } else {
                // Lock requires a continuous dwell inside ALL strict guards.
                // A stale contact or a speed excursion cannot preserve dwell credit.
                current_mode_ = FlightMode::ALIGN;
                dwell_timer_ = 0.0;
            }
            break;
        }
        case FlightMode::CONNECTED: {
            if (undock_requested_) {
                current_mode_ = FlightMode::RELEASE_PREP;
                mode_start_time_ = current_time;
                transition_time_ = 0.0;
                undock_requested_ = false;
            }
            break;
        }
        case FlightMode::RELEASE_PREP: {
            // After 100 ms of trim preparation, release latch
            if (transition_time_ >= 0.10) {
                current_mode_ = FlightMode::SEPARATING;
                last_event_time_ = current_time;
                last_event_name_ = "UNDOCK";
                mode_start_time_ = current_time;
                transition_time_ = 0.0;
            }
            break;
        }
        case FlightMode::SEPARATING: {
            // Once separated by > 0.20m, transition to independent RECOVERY
            if (rel_state_.distance > 0.20 || transition_time_ >= 0.8) {
                current_mode_ = FlightMode::RECOVERY;
                mode_start_time_ = current_time;
                transition_time_ = 0.0;
            }
            break;
        }
        case FlightMode::RECOVERY: {
            if (transition_time_ >= 1.0) {
                current_mode_ = FlightMode::FREE;
                mode_start_time_ = current_time;
                transition_time_ = 0.0;
            }
            break;
        }
        case FlightMode::ABORT:
        default:
            break;
    }

    return current_mode_;
}

} // namespace docking
