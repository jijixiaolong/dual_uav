#pragma once

#include "docking/types.hpp"

namespace docking {

class SingleQuadAllocator {
public:
    SingleQuadAllocator(const QuadParams& params);

    void reset();
    RotorThrusts4 allocate(const WrenchCommand& wrench, double dt);

    void set_previous_thrusts(const RotorThrusts4& t);
    const RotorThrusts4& get_thrusts() const { return prev_thrusts_; }
    WrenchCommand achieved_wrench(const RotorThrusts4& t) const;
    Vec3 achieved_force(const RotorThrusts4& t) const;

private:
    QuadParams params_;
    RotorThrusts4 prev_thrusts_{};
    std::array<std::array<double, 4>, 6> allocation_{};
};

class ConnectedQuadAllocator {
public:
    ConnectedQuadAllocator(const QuadParams& q1_params,
                           const QuadParams& q2_params,
                           const DockingParams& dock_params);

    void reset();
    RotorThrusts8 allocate(const WrenchCommand& wrench, double dt);

    void set_previous_thrusts(const RotorThrusts8& t);
    const RotorThrusts8& get_thrusts() const { return prev_thrusts_; }
    WrenchCommand achieved_wrench(const RotorThrusts8& t) const;
    Vec3 achieved_force(const RotorThrusts8& t) const;

    // Freeze the measured capture transform. All aggregate quantities use Q1's
    // axes; positions of the eight rotors are expressed about the aggregate COM.
    void update_geometry(const Vec3& q2_position_in_q1, const Quat& q2_attitude_in_q1);

    // Aggregate physical properties computed by parallel-axis theorem
    double total_mass() const { return total_mass_; }
    Vec3 com_offset() const { return com_offset_; }
    Mat3 aggregate_inertia() const { return J_c_; }

private:
    QuadParams q1_params_;
    QuadParams q2_params_;
    DockingParams dock_params_;

    double total_mass_{2.0};
    Vec3 com_offset_{0, 0, 0};
    Mat3 J_c_{Mat3::Identity()};

    Vec3 q2_position_in_q1_{};
    Quat q2_attitude_in_q1_{};
    std::array<std::array<double, 8>, 6> allocation_{};

    RotorThrusts8 prev_thrusts_{};

    void compute_aggregate_properties();
};

} // namespace docking
