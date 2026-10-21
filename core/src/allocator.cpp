#include "docking/allocator.hpp"
#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace docking {
namespace {

void validate_params(const QuadParams& p) {
    if (!std::isfinite(p.mass) || p.mass <= 0.0 ||
        !std::isfinite(p.gravity) || p.gravity <= 0.0 ||
        !std::isfinite(p.arm_length) || p.arm_length <= 0.0 ||
        !std::isfinite(p.min_thrust) || !std::isfinite(p.max_thrust) ||
        p.min_thrust < 0.0 || p.max_thrust < p.min_thrust ||
        !std::isfinite(p.slew_rate_limit) || p.slew_rate_limit < 0.0 ||
        !std::isfinite(p.torque_ratio) || p.torque_ratio < 0.0) {
        throw std::invalid_argument("Invalid quadrotor allocation parameters");
    }
    for (size_t i = 0; i < 4; ++i) {
        if (!std::isfinite(p.rotor_angles_deg[i]) || std::abs(p.rotor_directions[i]) != 1)
            throw std::invalid_argument("Invalid rotor geometry or spin direction");
    }
}

bool finite_vec(const Vec3& v) {
    return std::isfinite(v.x) && std::isfinite(v.y) && std::isfinite(v.z);
}

Mat3 parallel_axis(const Vec3& r, double mass) {
    const double v[3] = {r.x, r.y, r.z};
    Mat3 result = Mat3::Zero();
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j)
            result(i, j) = mass * ((i == j ? r.norm_sq() : 0.0) - v[i] * v[j]);
    return result;
}

template <size_t N>
void add_rotors(std::array<std::array<double, N>, 6>& matrix, size_t offset,
                const QuadParams& p, const Vec3& center, const Mat3& rotation) {
    const Vec3 axis = rotation * Vec3(0, 0, 1);
    for (size_t i = 0; i < 4; ++i) {
        const double angle = p.rotor_angles_deg[i] * std::acos(-1.0) / 180.0;
        const Vec3 position = center + rotation * Vec3(p.arm_length * std::cos(angle),
                                                     p.arm_length * std::sin(angle), 0);
        const Vec3 torque = position.cross(axis) + axis * (p.torque_ratio * p.rotor_directions[i]);
        matrix[0][offset + i] = axis.x;
        matrix[1][offset + i] = axis.y;
        matrix[2][offset + i] = axis.z;
        matrix[3][offset + i] = torque.x;
        matrix[4][offset + i] = torque.y;
        matrix[5][offset + i] = torque.z;
    }
}

// Primal active-set solution of a small strictly convex, box-constrained QP.
// Unlike clipping a pseudoinverse, bound-active motors are redistributed among
// the remaining motors. Rate limits are bounds inside the optimization itself.
template <size_t N>
RotorThrusts<N> bounded_allocate(const std::array<std::array<double, N>, 6>& B,
                                const WrenchCommand& command,
                                const std::array<double, N>& minimum,
                                const std::array<double, N>& maximum,
                                const std::array<double, N>& slew,
                                const RotorThrusts<N>& previous, double dt,
                                const std::array<double, N>& nominal) {
    if (!std::isfinite(dt) || dt < 0.0 || !std::isfinite(command.thrust) || !finite_vec(command.torque))
        throw std::invalid_argument("Allocation requires a finite wrench and nonnegative dt");
    if (dt == 0.0) return previous;
    std::array<std::array<double, N>, 6> C = B;
    std::array<double, 6> target{0, 0, command.thrust,
                                 command.torque.x, command.torque.y, command.torque.z};
    // A multicopter command has four independently controlled components:
    // body-Z thrust and three moments. Captured rotor axes may be nonparallel,
    // so lateral forces are measured via achieved_force(), not set to zero as
    // extra independent tasks. Even a 0.4 deg relative tilt otherwise drives
    // one quad to its bounds to reduce an infeasible lateral-force objective,
    // storing a large internal wrench that is released at undocking.
    C[0].fill(0.0);
    C[1].fill(0.0);
    // Normalize moment axes by their effective moment arms.
    for (size_t row = 3; row < 6; ++row) {
        double norm_sq = 0.0;
        for (double value : C[row]) norm_sq += value * value;
        const double scale = std::sqrt(static_cast<double>(N) / std::max(norm_sq, 1e-12));
        for (double& value : C[row]) value *= scale;
        target[row] *= scale;
    }
    std::array<double, N> low{}, high{}, x{}, rhs{};
    std::array<std::array<double, N>, N> H{};
    std::array<int, N> active{}; // -1: lower, 0: free, +1: upper
    constexpr double regularization = 1e-10;
    for (size_t i = 0; i < N; ++i) {
        low[i] = std::max(minimum[i], previous[i] - slew[i] * dt);
        high[i] = std::min(maximum[i], previous[i] + slew[i] * dt);
        x[i] = std::clamp(previous[i], low[i], high[i]);
        rhs[i] = regularization * nominal[i];
        for (size_t row = 0; row < 6; ++row) rhs[i] += C[row][i] * target[row];
        for (size_t j = 0; j < N; ++j) {
            for (size_t row = 0; row < 6; ++row) H[i][j] += C[row][i] * C[row][j];
            if (i == j) H[i][j] += regularization;
        }
        if (high[i] - low[i] < 1e-14) active[i] = -1;
    }
    for (size_t iteration = 0; iteration < 16 * N * N; ++iteration) {
        std::array<size_t, N> free{};
        size_t count = 0;
        for (size_t i = 0; i < N; ++i) if (active[i] == 0) free[count++] = i;
        std::array<std::array<double, N + 1>, N> augmented{};
        for (size_t r = 0; r < count; ++r) {
            const size_t i = free[r];
            augmented[r][count] = rhs[i];
            for (size_t j = 0; j < N; ++j)
                if (active[j]) augmented[r][count] -= H[i][j] * x[j];
            for (size_t c = 0; c < count; ++c) augmented[r][c] = H[i][free[c]];
        }
        // Partial pivoting avoids assumptions about symmetric rotor layouts.
        for (size_t col = 0; col < count; ++col) {
            size_t pivot = col;
            for (size_t r = col + 1; r < count; ++r)
                if (std::abs(augmented[r][col]) > std::abs(augmented[pivot][col])) pivot = r;
            std::swap(augmented[col], augmented[pivot]);
            const double divisor = augmented[col][col];
            if (std::abs(divisor) < 1e-14) throw std::runtime_error("Singular allocation QP");
            for (size_t c = col; c <= count; ++c) augmented[col][c] /= divisor;
            for (size_t r = 0; r < count; ++r) {
                if (r == col) continue;
                const double factor = augmented[r][col];
                for (size_t c = col; c <= count; ++c) augmented[r][c] -= factor * augmented[col][c];
            }
        }
        double fraction = 1.0;
        size_t blocking = N;
        int blocking_side = 0;
        for (size_t r = 0; r < count; ++r) {
            const size_t i = free[r];
            const double direction = augmented[r][count] - x[i];
            double candidate = 1.0;
            int side = 0;
            if (direction > 1e-14 && augmented[r][count] > high[i]) {
                candidate = (high[i] - x[i]) / direction;
                side = 1;
            } else if (direction < -1e-14 && augmented[r][count] < low[i]) {
                candidate = (low[i] - x[i]) / direction;
                side = -1;
            }
            if (side != 0 && candidate < fraction) {
                fraction = std::max(0.0, candidate);
                blocking = i;
                blocking_side = side;
            }
        }
        for (size_t r = 0; r < count; ++r) {
            const size_t i = free[r];
            x[i] = std::clamp(x[i] + fraction * (augmented[r][count] - x[i]), low[i], high[i]);
        }
        if (blocking != N) {
            active[blocking] = blocking_side;
            continue;
        }
        size_t release = N;
        double violation = 1e-8;
        for (size_t i = 0; i < N; ++i) {
            if (active[i] == 0 || high[i] - low[i] < 1e-14) continue;
            double gradient = -rhs[i];
            for (size_t j = 0; j < N; ++j) gradient += H[i][j] * x[j];
            const double error = active[i] < 0 ? -gradient : gradient;
            if (error > violation) { violation = error; release = i; }
        }
        if (release == N) {
            RotorThrusts<N> result;
            result.thrusts = x;
            return result;
        }
        active[release] = 0;
    }
    throw std::runtime_error("Bounded allocation failed to converge");
}

template <size_t N>
std::array<double, 6> applied_wrench(const std::array<std::array<double, N>, 6>& B,
                                     const RotorThrusts<N>& thrusts) {
    std::array<double, 6> result{};
    for (size_t r = 0; r < 6; ++r)
        for (size_t i = 0; i < N; ++i) result[r] += B[r][i] * thrusts[i];
    return result;
}

template <size_t N>
RotorThrusts<N> synchronized_thrusts(const RotorThrusts<N>& input,
                                    const QuadParams& q1, const QuadParams& q2) {
    RotorThrusts<N> result;
    for (size_t i = 0; i < N; ++i) {
        const auto& p = i < 4 ? q1 : q2;
        if (!std::isfinite(input[i])) throw std::invalid_argument("Nonfinite previous motor thrust");
        result[i] = std::clamp(input[i], p.min_thrust, p.max_thrust);
    }
    return result;
}
} // namespace

SingleQuadAllocator::SingleQuadAllocator(const QuadParams& params) : params_(params) {
    validate_params(params_);
    add_rotors(allocation_, 0, params_, Vec3{}, Mat3::Identity());
    reset();
}

void SingleQuadAllocator::reset() {
    for (size_t i = 0; i < 4; ++i)
        prev_thrusts_[i] = std::clamp(params_.mass * params_.gravity / 4.0, params_.min_thrust, params_.max_thrust);
}

void SingleQuadAllocator::set_previous_thrusts(const RotorThrusts4& t) {
    prev_thrusts_ = synchronized_thrusts(t, params_, params_);
}

RotorThrusts4 SingleQuadAllocator::allocate(const WrenchCommand& wrench, double dt) {
    std::array<double, 4> minimum{}, maximum{}, slew{}, nominal{};
    minimum.fill(params_.min_thrust); maximum.fill(params_.max_thrust); slew.fill(params_.slew_rate_limit);
    nominal.fill(params_.mass * params_.gravity / 4.0);
    prev_thrusts_ = bounded_allocate(allocation_, wrench, minimum, maximum, slew, prev_thrusts_, dt, nominal);
    return prev_thrusts_;
}

WrenchCommand SingleQuadAllocator::achieved_wrench(const RotorThrusts4& t) const {
    const auto w = applied_wrench(allocation_, t);
    return {w[2], {w[3], w[4], w[5]}};
}

Vec3 SingleQuadAllocator::achieved_force(const RotorThrusts4& t) const {
    const auto w = applied_wrench(allocation_, t);
    return {w[0], w[1], w[2]};
}

ConnectedQuadAllocator::ConnectedQuadAllocator(const QuadParams& q1_params,
                                               const QuadParams& q2_params,
                                               const DockingParams& dock_params)
    : q1_params_(q1_params), q2_params_(q2_params), dock_params_(dock_params),
      q2_position_in_q1_(dock_params.connector_offset_q1 - dock_params.connector_offset_q2) {
    validate_params(q1_params_); validate_params(q2_params_);
    compute_aggregate_properties();
    reset();
}

void ConnectedQuadAllocator::update_geometry(const Vec3& position, const Quat& attitude) {
    if (!finite_vec(position) || !std::isfinite(attitude.w) || !std::isfinite(attitude.x) ||
        !std::isfinite(attitude.y) || !std::isfinite(attitude.z) ||
        attitude.w * attitude.w + attitude.x * attitude.x + attitude.y * attitude.y + attitude.z * attitude.z < 1e-12)
        throw std::invalid_argument("Invalid captured relative pose");
    q2_position_in_q1_ = position;
    q2_attitude_in_q1_ = attitude.normalized();
    compute_aggregate_properties();
}

void ConnectedQuadAllocator::compute_aggregate_properties() {
    total_mass_ = q1_params_.mass + q2_params_.mass;
    com_offset_ = q2_position_in_q1_ * (q2_params_.mass / total_mass_);
    const Vec3 r1 = com_offset_ * -1.0;
    const Vec3 r2 = q2_position_in_q1_ - com_offset_;
    const Mat3 R2 = q2_attitude_in_q1_.to_rotation_matrix();
    J_c_ = q1_params_.inertia + R2 * q2_params_.inertia * R2.transpose() +
           parallel_axis(r1, q1_params_.mass) + parallel_axis(r2, q2_params_.mass);
    add_rotors(allocation_, 0, q1_params_, r1, Mat3::Identity());
    add_rotors(allocation_, 4, q2_params_, r2, R2);
}

void ConnectedQuadAllocator::reset() {
    for (size_t i = 0; i < 8; ++i) {
        const auto& p = i < 4 ? q1_params_ : q2_params_;
        prev_thrusts_[i] = std::clamp(p.mass * p.gravity / 4.0, p.min_thrust, p.max_thrust);
    }
}

void ConnectedQuadAllocator::set_previous_thrusts(const RotorThrusts8& t) {
    prev_thrusts_ = synchronized_thrusts(t, q1_params_, q2_params_);
}

RotorThrusts8 ConnectedQuadAllocator::allocate(const WrenchCommand& wrench, double dt) {
    std::array<double, 8> minimum{}, maximum{}, slew{}, nominal{};
    for (size_t i = 0; i < 8; ++i) {
        const auto& p = i < 4 ? q1_params_ : q2_params_;
        minimum[i] = p.min_thrust; maximum[i] = p.max_thrust; slew[i] = p.slew_rate_limit;
        nominal[i] = p.mass * p.gravity / 4.0;
    }
    prev_thrusts_ = bounded_allocate(allocation_, wrench, minimum, maximum, slew, prev_thrusts_, dt, nominal);
    return prev_thrusts_;
}

WrenchCommand ConnectedQuadAllocator::achieved_wrench(const RotorThrusts8& t) const {
    const auto w = applied_wrench(allocation_, t);
    return {w[2], {w[3], w[4], w[5]}};
}

Vec3 ConnectedQuadAllocator::achieved_force(const RotorThrusts8& t) const {
    const auto w = applied_wrench(allocation_, t);
    return {w[0], w[1], w[2]};
}

} // namespace docking
