#pragma once

#include "docking/types.hpp"
#include <vector>

namespace docking {

struct DockEventFeatures {
    Vec3 pos_err{0, 0, 0};
    Vec3 vel_rel{0, 0, 0};
    Vec3 rot_err_rpy{0, 0, 0};
    Vec3 ang_vel_rel{0, 0, 0};
    double thrust_diff{0.0};

    std::vector<double> to_vector() const {
        return {
            pos_err.x, pos_err.y, pos_err.z,
            vel_rel.x, vel_rel.y, vel_rel.z,
            rot_err_rpy.x, rot_err_rpy.y, rot_err_rpy.z,
            ang_vel_rel.x, ang_vel_rel.y, ang_vel_rel.z,
            thrust_diff
        };
    }
};

struct UndockEventFeatures {
    Vec3 aggregate_omega{0, 0, 0};
    Vec3 aggregate_attitude_err{0, 0, 0};
    double left_right_thrust_asymmetry{0.0};
    double front_back_thrust_asymmetry{0.0};
    double estimated_preload{0.0};

    std::vector<double> to_vector() const {
        return {
            aggregate_omega.x, aggregate_omega.y, aggregate_omega.z,
            aggregate_attitude_err.x, aggregate_attitude_err.y, aggregate_attitude_err.z,
            left_right_thrust_asymmetry,
            front_back_thrust_asymmetry,
            estimated_preload
        };
    }
};

} // namespace docking
