#pragma once

#include "docking/types.hpp"
#include <string>

namespace docking {

struct RelativeConnectorState {
    Vec3 pos_err{0, 0, 0};        // connector 1 to connector 2 offset
    Vec3 vel_rel{0, 0, 0};        // relative linear velocity
    double angle_err_rad{0.0};    // relative attitude angle
    Vec3 ang_vel_rel{0, 0, 0};    // relative angular velocity
    double distance{0.0};
};

class HybridFSM {
public:
    HybridFSM(const DockingParams& params);

    void reset();
    // Test-fixture initialization for an already rigidly joined pair at t=0.
    // Normal in-flight docking always goes through the contact/capture guards.
    void initialize_connected(double current_time = 0.0);
    void update_params(const DockingParams& params) { params_ = params; }

    // Step state machine given latest states and time
    FlightMode update(const RigidBodyState& q1,
                      const RigidBodyState& q2,
                      double current_time,
                      double dt);

    // External commands from supervisor
    void command_approach();
    void command_undock();
    void command_abort();
    // The physics adapter owns contact detection. A kinematic near miss alone
    // is never sufficient to engage the passive capture approximation.
    void set_capture_contact(bool contact) { capture_contact_ = contact; }

    void invalidate_capture_dwell() { dwell_timer_ = 0.0; }

    FlightMode current_mode() const { return current_mode_; }
    bool is_latched() const { return current_mode_ == FlightMode::CONNECTED || current_mode_ == FlightMode::RELEASE_PREP; }

    const RelativeConnectorState& rel_state() const { return rel_state_; }
    double last_event_time() const { return last_event_time_; }
    std::string last_event_name() const { return last_event_name_; }

    // Bumpless handover ratio [0, 1] (0: fully old, 1: fully new)
    double handover_alpha() const;

private:
    DockingParams params_;
    FlightMode current_mode_{FlightMode::FREE};
    RelativeConnectorState rel_state_{};

    double dwell_timer_{0.0};
    double transition_time_{0.0};
    double last_event_time_{0.0};
    std::string last_event_name_{"NONE"};

    bool approach_requested_{false};
    bool undock_requested_{false};
    bool abort_requested_{false};
    bool capture_contact_{false};

    double mode_start_time_{0.0};

    void compute_relative_connector_state(const RigidBodyState& q1, const RigidBodyState& q2);
    bool check_latch_criteria() const;
};

} // namespace docking
