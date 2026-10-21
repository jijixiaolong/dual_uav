#pragma once

#include "docking/types.hpp"
#include <vector>
#include <string>

namespace docking {

enum class CompensatorType {
    NONE = 0,         // Baseline B0
    FIXED_FF = 1,     // Baseline B1
    ILC = 2,          // Baseline B2
    ECLR = 3          // B4: Event-Conditioned Lifted Residual
};

class EventCompensator {
public:
    EventCompensator();

    void reset();
    void configure(CompensatorType type,
                   const std::vector<double>& basis_decay_times,
                   double max_thrust_delta,
                   double event_window_duration);

    // Set offline precomputed fixed coefficients (for B1) or updated ILC table (for B2)
    void set_fixed_coefficients(const std::vector<double>& coeff);

    // Set model matrices A and G for ECLR (for B4)
    // A: (m x d), G: (m x k), where k is basis count, d is feature dim
    void set_eclr_matrices(const std::vector<std::vector<double>>& A,
                           const std::vector<std::vector<double>>& G,
                           double regularization = 1e-4);

    // Trigger on event confirmation (DOCK or UNDOCK)
    void trigger_event(const std::string& event_name,
                       double trigger_time,
                       const std::vector<double>& features);

    // Compute additive rotor thrust compensation vector at time t
    // Returns 4-vector for single quad (undock) or 8-vector for connected quad (dock)
    std::vector<double> evaluate(double current_time, size_t num_rotors);

    bool is_active(double current_time) const;
    const std::vector<double>& active_coefficients() const { return current_coeff_; }

private:
    CompensatorType type_{CompensatorType::NONE};
    std::vector<double> basis_decay_times_{0.08, 0.16, 0.28, 0.45};
    double max_thrust_delta_{1.5}; // N
    double event_window_duration_{1.2}; // s

    std::vector<double> fixed_coeff_;
    std::vector<double> current_coeff_;

    // ECLR regression matrices
    std::vector<std::vector<double>> A_;
    std::vector<std::vector<double>> G_;
    double eclr_reg_{1e-4};

    bool active_{false};
    double event_time_{0.0};
    std::string active_event_{"NONE"};

    void solve_eclr_coefficients(const std::vector<double>& features);
};

} // namespace docking
