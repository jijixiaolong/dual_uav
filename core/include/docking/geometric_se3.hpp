#pragma once

#include "docking/types.hpp"

namespace docking {

struct SE3Gains {
    Vec3 kp{8.0, 8.0, 15.0};
    Vec3 kv{4.0, 4.0, 6.0};
    Vec3 ki{1.0, 1.0, 2.0};
    Vec3 i_limit{1.0, 1.0, 2.0};

    Vec3 kr{1.6, 1.6, 1.0};
    Vec3 kw{0.22, 0.22, 0.15};
    Vec3 ki_rot{0.05, 0.05, 0.02};
    Vec3 i_limit_rot{0.2, 0.2, 0.1};
};

struct Setpoint {
    Vec3 pos{0, 0, 1.0};
    Vec3 vel{0, 0, 0};
    Vec3 acc{0, 0, 0};
    double yaw{0.0};
    double yaw_rate{0.0};
};

class GeometricSE3Controller {
public:
    GeometricSE3Controller(double mass, const Mat3& inertia, const SE3Gains& gains);

    void reset();
    void update_gains(const SE3Gains& gains);
    void update_inertia(double mass, const Mat3& inertia);

    WrenchCommand compute(const RigidBodyState& state,
                          const Setpoint& setpoint,
                          double dt);

    Vec3 get_attitude_error() const { return e_R_; }
    Vec3 get_rate_error() const { return e_omega_; }
    Vec3 get_pos_error() const { return e_p_; }
    Mat3 get_desired_rotation() const { return last_R_d_; }

private:
    double mass_{1.0};
    double gravity_{9.80665};
    Mat3 inertia_;
    SE3Gains gains_;

    Vec3 int_pos_err_{0, 0, 0};
    Vec3 int_rot_err_{0, 0, 0};

    Vec3 e_p_{0, 0, 0};
    Vec3 e_v_{0, 0, 0};
    Vec3 e_R_{0, 0, 0};
    Vec3 e_omega_{0, 0, 0};
    Mat3 last_R_d_{Mat3::Identity()};
};

} // namespace docking
