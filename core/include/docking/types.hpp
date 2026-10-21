#pragma once

#include <cmath>
#include <array>
#include <vector>
#include <string>
#include <algorithm>
#include <iostream>

namespace docking {

// 3D Vector
struct Vec3 {
    double x{0.0};
    double y{0.0};
    double z{0.0};

    constexpr Vec3() = default;
    constexpr Vec3(double x_, double y_, double z_) : x(x_), y(y_), z(z_) {}

    Vec3 operator+(const Vec3& o) const { return {x + o.x, y + o.y, z + o.z}; }
    Vec3 operator-(const Vec3& o) const { return {x - o.x, y - o.y, z - o.z}; }
    Vec3 operator*(double s) const { return {x * s, y * s, z * s}; }
    Vec3 operator/(double s) const { return {x / s, y / s, z / s}; }
    Vec3& operator+=(const Vec3& o) { x += o.x; y += o.y; z += o.z; return *this; }
    Vec3& operator-=(const Vec3& o) { x -= o.x; y -= o.y; z -= o.z; return *this; }

    double dot(const Vec3& o) const { return x * o.x + y * o.y + z * o.z; }
    Vec3 cross(const Vec3& o) const {
        return {
            y * o.z - z * o.y,
            z * o.x - x * o.z,
            x * o.y - y * o.x
        };
    }
    double norm_sq() const { return dot(*this); }
    double norm() const { return std::sqrt(norm_sq()); }
    Vec3 normalized() const {
        double n = norm();
        return (n > 1e-12) ? (*this / n) : Vec3(0, 0, 0);
    }
};

inline Vec3 operator*(double s, const Vec3& v) { return v * s; }

// 3x3 Matrix (row-major)
struct Mat3 {
    std::array<double, 9> m{1, 0, 0, 0, 1, 0, 0, 0, 1};

    static Mat3 Identity() {
        return {1, 0, 0, 0, 1, 0, 0, 0, 1};
    }

    static Mat3 Zero() {
        return {0, 0, 0, 0, 0, 0, 0, 0, 0};
    }

    double& operator()(int r, int c) { return m[r * 3 + c]; }
    const double& operator()(int r, int c) const { return m[r * 3 + c]; }

    Vec3 operator*(const Vec3& v) const {
        return {
            m[0] * v.x + m[1] * v.y + m[2] * v.z,
            m[3] * v.x + m[4] * v.y + m[5] * v.z,
            m[6] * v.x + m[7] * v.y + m[8] * v.z
        };
    }

    Mat3 operator*(const Mat3& o) const {
        Mat3 res = Zero();
        for (int r = 0; r < 3; ++r) {
            for (int c = 0; c < 3; ++c) {
                for (int k = 0; k < 3; ++k) {
                    res(r, c) += (*this)(r, k) * o(k, c);
                }
            }
        }
        return res;
    }

    Mat3 operator-(const Mat3& o) const {
        Mat3 res;
        for (size_t i = 0; i < 9; ++i) res.m[i] = m[i] - o.m[i];
        return res;
    }

    Mat3 operator+(const Mat3& o) const {
        Mat3 res;
        for (size_t i = 0; i < 9; ++i) res.m[i] = m[i] + o.m[i];
        return res;
    }

    Mat3 transpose() const {
        return {
            m[0], m[3], m[6],
            m[1], m[4], m[7],
            m[2], m[5], m[8]
        };
    }

    static Mat3 Skew(const Vec3& v) {
        return {
             0.0, -v.z,  v.y,
             v.z,  0.0, -v.x,
            -v.y,  v.x,  0.0
        };
    }

    // vee operator: un-skew a 3x3 skew symmetric matrix
    static Vec3 Vee(const Mat3& S) {
        return {
            0.5 * (S(2, 1) - S(1, 2)),
            0.5 * (S(0, 2) - S(2, 0)),
            0.5 * (S(1, 0) - S(0, 1))
        };
    }
};

// Quaternion [w, x, y, z]
struct Quat {
    double w{1.0};
    double x{0.0};
    double y{0.0};
    double z{0.0};

    constexpr Quat() = default;
    constexpr Quat(double w_, double x_, double y_, double z_) : w(w_), x(x_), y(y_), z(z_) {}

    Quat normalized() const {
        double n = std::sqrt(w * w + x * x + y * y + z * z);
        if (n < 1e-12) return {1, 0, 0, 0};
        return {w / n, x / n, y / n, z / n};
    }

    Mat3 to_rotation_matrix() const {
        Quat q = normalized();
        double wx = q.w * q.x, wy = q.w * q.y, wz = q.w * q.z;
        double xx = q.x * q.x, xy = q.x * q.y, xz = q.x * q.z;
        double yy = q.y * q.y, yz = q.y * q.z, zz = q.z * q.z;

        return {
            1.0 - 2.0 * (yy + zz), 2.0 * (xy - wz),       2.0 * (xz + wy),
            2.0 * (xy + wz),       1.0 - 2.0 * (xx + zz), 2.0 * (yz - wx),
            2.0 * (xz - wy),       2.0 * (yz + wx),       1.0 - 2.0 * (xx + yy)
        };
    }

    static Quat from_rotation_matrix(const Mat3& R) {
        double tr = R(0, 0) + R(1, 1) + R(2, 2);
        Quat q;
        if (tr > 0.0) {
            double S = std::sqrt(tr + 1.0) * 2.0;
            q.w = 0.25 * S;
            q.x = (R(2, 1) - R(1, 2)) / S;
            q.y = (R(0, 2) - R(2, 0)) / S;
            q.z = (R(1, 0) - R(0, 1)) / S;
        } else if ((R(0, 0) > R(1, 1)) && (R(0, 0) > R(2, 2))) {
            double S = std::sqrt(1.0 + R(0, 0) - R(1, 1) - R(2, 2)) * 2.0;
            q.w = (R(2, 1) - R(1, 2)) / S;
            q.x = 0.25 * S;
            q.y = (R(0, 1) + R(1, 0)) / S;
            q.z = (R(0, 2) + R(2, 0)) / S;
        } else if (R(1, 1) > R(2, 2)) {
            double S = std::sqrt(1.0 + R(1, 1) - R(0, 0) - R(2, 2)) * 2.0;
            q.w = (R(0, 2) - R(2, 0)) / S;
            q.x = (R(0, 1) + R(1, 0)) / S;
            q.y = 0.25 * S;
            q.z = (R(1, 2) + R(2, 1)) / S;
        } else {
            double S = std::sqrt(1.0 + R(2, 2) - R(0, 0) - R(1, 1)) * 2.0;
            q.w = (R(1, 0) - R(0, 1)) / S;
            q.x = (R(0, 2) + R(2, 0)) / S;
            q.y = (R(1, 2) + R(2, 1)) / S;
            q.z = 0.25 * S;
        }
        return q.normalized();
    }
};

// 6-DoF Rigid Body Kinematics State
struct RigidBodyState {
    Vec3 pos{0, 0, 0};
    Vec3 vel{0, 0, 0};
    Quat quat{1, 0, 0, 0};
    Vec3 omega{0, 0, 0}; // body frame angular velocity
    double timestamp{0.0};

    Mat3 rot_matrix() const { return quat.to_rotation_matrix(); }
};

// Virtual Control Wrench (Force along body z + body torques)
struct WrenchCommand {
    double thrust{0.0}; // collective thrust along body Z (N)
    Vec3 torque{0, 0, 0}; // body torques (N*m)
};

// Actuator Commands (individual rotor thrusts in N)
template <size_t N>
struct RotorThrusts {
    std::array<double, N> thrusts{};

    double operator[](size_t i) const { return thrusts[i]; }
    double& operator[](size_t i) { return thrusts[i]; }
    static constexpr size_t size() { return N; }
};

using RotorThrusts4 = RotorThrusts<4>;
using RotorThrusts8 = RotorThrusts<8>;

// Hybrid Flight Modes
enum class FlightMode : int {
    FREE = 0,
    APPROACH = 1,
    ALIGN = 2,
    CAPTURE_PENDING = 3,
    CONNECTED = 4,
    RELEASE_PREP = 5,
    SEPARATING = 6,
    RECOVERY = 7,
    ABORT = 8
};

inline std::string mode_to_string(FlightMode m) {
    switch (m) {
        case FlightMode::FREE: return "FREE";
        case FlightMode::APPROACH: return "APPROACH";
        case FlightMode::ALIGN: return "ALIGN";
        case FlightMode::CAPTURE_PENDING: return "CAPTURE_PENDING";
        case FlightMode::CONNECTED: return "CONNECTED";
        case FlightMode::RELEASE_PREP: return "RELEASE_PREP";
        case FlightMode::SEPARATING: return "SEPARATING";
        case FlightMode::RECOVERY: return "RECOVERY";
        case FlightMode::ABORT: return "ABORT";
    }
    return "UNKNOWN";
}

// Single Quadrotor Physical Parameters
struct QuadParams {
    double mass{1.0};
    double gravity{9.80665};
    double arm_length{0.17};
    Mat3 inertia{
        0.0082, 0.0,    0.0,
        0.0,    0.0082, 0.0,
        0.0,    0.0,    0.0145
    };
    double min_thrust{0.05};
    double max_thrust{5.5};
    double torque_ratio{0.016};
    double time_constant{0.025};
    double slew_rate_limit{45.0};
    std::array<double, 4> rotor_angles_deg{45.0, 135.0, 225.0, 315.0};
    std::array<int, 4> rotor_directions{1, -1, 1, -1}; // Body +Z reaction-torque signs; rotor spin is opposite.
};

// Docking Mechanism Parameters
struct DockingParams {
    Vec3 connector_offset_q1{0.0, 0.22, 0.0};
    Vec3 connector_offset_q2{0.0, -0.22, 0.0};
    double pos_tol{0.015};
    double rot_tol{0.06};
    double lin_vel_tol{0.12};
    double ang_vel_tol{0.25};
    double dwell_time{0.035};
    bool zero_preload_mode{true};
    double spring_stiffness{60.0};
    double preload_deflection{0.008};
    double damping{2.0};
};

} // namespace docking
