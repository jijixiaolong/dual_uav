#include "docking/geometric_se3.hpp"
#include <algorithm>
#include <cmath>

namespace docking {

GeometricSE3Controller::GeometricSE3Controller(double mass, const Mat3& inertia, const SE3Gains& gains)
    : mass_(mass), inertia_(inertia), gains_(gains) {
    reset();
}

void GeometricSE3Controller::reset() {
    int_pos_err_ = Vec3(0, 0, 0);
    int_rot_err_ = Vec3(0, 0, 0);
    e_p_ = Vec3(0, 0, 0);
    e_v_ = Vec3(0, 0, 0);
    e_R_ = Vec3(0, 0, 0);
    e_omega_ = Vec3(0, 0, 0);
    last_R_d_ = Mat3::Identity();
}

void GeometricSE3Controller::update_gains(const SE3Gains& gains) {
    gains_ = gains;
}

void GeometricSE3Controller::update_inertia(double mass, const Mat3& inertia) {
    mass_ = mass;
    inertia_ = inertia;
}

static inline double clamp_val(double v, double lim) {
    return std::max(-lim, std::min(lim, v));
}

WrenchCommand GeometricSE3Controller::compute(const RigidBodyState& state,
                                             const Setpoint& setpoint,
                                             double dt) {
    // 1. Position and Velocity errors in World frame
    e_p_ = state.pos - setpoint.pos;
    e_v_ = state.vel - setpoint.vel;

    if (dt > 1e-6) {
        int_pos_err_.x = clamp_val(int_pos_err_.x + e_p_.x * dt, gains_.i_limit.x);
        int_pos_err_.y = clamp_val(int_pos_err_.y + e_p_.y * dt, gains_.i_limit.y);
        int_pos_err_.z = clamp_val(int_pos_err_.z + e_p_.z * dt, gains_.i_limit.z);
    }

    // 2. Desired total force in World frame
    Vec3 F_p(-gains_.kp.x * e_p_.x, -gains_.kp.y * e_p_.y, -gains_.kp.z * e_p_.z);
    Vec3 F_v(-gains_.kv.x * e_v_.x, -gains_.kv.y * e_v_.y, -gains_.kv.z * e_v_.z);
    Vec3 F_i(-gains_.ki.x * int_pos_err_.x, -gains_.ki.y * int_pos_err_.y, -gains_.ki.z * int_pos_err_.z);
    Vec3 F_ff = mass_ * (setpoint.acc + Vec3(0, 0, gravity_));

    Vec3 F_des = F_p + F_v + F_i + F_ff;

    // Safety: ensure positive thrust request
    if (F_des.z < 0.1 * mass_ * gravity_) {
        F_des.z = 0.1 * mass_ * gravity_;
    }

    Mat3 R = state.rot_matrix();
    Vec3 b3 = Vec3(R(0, 2), R(1, 2), R(2, 2)); // current body z-axis in world

    // Collective thrust magnitude along current body z
    double f_z = F_des.dot(b3);
    if (f_z < 0.0) f_z = 0.0;

    // 3. Desired rotation matrix R_d
    Vec3 b3_d = F_des.normalized();
    Vec3 c1_d(std::cos(setpoint.yaw), std::sin(setpoint.yaw), 0.0);

    Vec3 b2_d = b3_d.cross(c1_d).normalized();
    if (b2_d.norm_sq() < 1e-6) {
        // Degenerate case when b3_d is parallel to c1_d
        b2_d = Vec3(-std::sin(setpoint.yaw), std::cos(setpoint.yaw), 0.0);
    }
    Vec3 b1_d = b2_d.cross(b3_d).normalized();

    Mat3 R_d{
        b1_d.x, b2_d.x, b3_d.x,
        b1_d.y, b2_d.y, b3_d.y,
        b1_d.z, b2_d.z, b3_d.z
    };
    last_R_d_ = R_d;

    // 4. Attitude error in SO(3) Lie algebra
    Mat3 rot_err_mat = (R_d.transpose() * R) - (R.transpose() * R_d);
    // e_R = 1/2 vee(R_d^T R - R^T R_d). Vee already converts
    // an arbitrary skew matrix into its vector; omitting this factor doubled
    // the advertised attitude proportional and integral gains.
    e_R_ = 0.5 * Mat3::Vee(rot_err_mat);

    // 5. Angular velocity error
    Vec3 omega_d(0, 0, setpoint.yaw_rate); // simplified feedforward rate
    Vec3 omega_d_body = R.transpose() * (R_d * omega_d);
    e_omega_ = state.omega - omega_d_body;

    if (dt > 1e-6) {
        int_rot_err_.x = clamp_val(int_rot_err_.x + e_R_.x * dt, gains_.i_limit_rot.x);
        int_rot_err_.y = clamp_val(int_rot_err_.y + e_R_.y * dt, gains_.i_limit_rot.y);
        int_rot_err_.z = clamp_val(int_rot_err_.z + e_R_.z * dt, gains_.i_limit_rot.z);
    }

    // 6. Desired torque in body frame
    Vec3 tau_p(-gains_.kr.x * e_R_.x, -gains_.kr.y * e_R_.y, -gains_.kr.z * e_R_.z);
    Vec3 tau_w(-gains_.kw.x * e_omega_.x, -gains_.kw.y * e_omega_.y, -gains_.kw.z * e_omega_.z);
    Vec3 tau_i(-gains_.ki_rot.x * int_rot_err_.x, -gains_.ki_rot.y * int_rot_err_.y, -gains_.ki_rot.z * int_rot_err_.z);
    Vec3 tau_gyr = state.omega.cross(inertia_ * state.omega);

    Vec3 tau_des = tau_p + tau_w + tau_i + tau_gyr;

    WrenchCommand cmd;
    cmd.thrust = f_z;
    cmd.torque = tau_des;
    return cmd;
}

} // namespace docking
