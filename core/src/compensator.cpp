#include "docking/compensator.hpp"
#include <cmath>
#include <algorithm>

namespace docking {

EventCompensator::EventCompensator() {
    reset();
}

void EventCompensator::reset() {
    active_ = false;
    event_time_ = 0.0;
    active_event_ = "NONE";
    current_coeff_.assign(basis_decay_times_.size(), 0.0);
}

void EventCompensator::configure(CompensatorType type,
                                const std::vector<double>& basis_decay_times,
                                double max_thrust_delta,
                                double event_window_duration) {
    type_ = type;
    if (!basis_decay_times.empty()) {
        basis_decay_times_ = basis_decay_times;
    }
    max_thrust_delta_ = max_thrust_delta;
    event_window_duration_ = event_window_duration;
    reset();
}

void EventCompensator::set_fixed_coefficients(const std::vector<double>& coeff) {
    fixed_coeff_ = coeff;
}

void EventCompensator::set_eclr_matrices(const std::vector<std::vector<double>>& A,
                                        const std::vector<std::vector<double>>& G,
                                        double regularization) {
    A_ = A;
    G_ = G;
    eclr_reg_ = regularization;
}

void EventCompensator::solve_eclr_coefficients(const std::vector<double>& features) {
    size_t k = basis_decay_times_.size();
    current_coeff_.assign(k, 0.0);

    if (A_.empty() || G_.empty() || features.empty()) {
        return;
    }

    size_t m = A_.size();     // lifted output dimension
    size_t d = features.size(); // feature dimension

    // 1. y0 = A * features (m-dim)
    std::vector<double> y0(m, 0.0);
    for (size_t i = 0; i < m; ++i) {
        double sum = 0.0;
        size_t cols = std::min(d, A_[i].size());
        for (size_t j = 0; j < cols; ++j) {
            sum += A_[i][j] * features[j];
        }
        y0[i] = sum;
    }

    // 2. Compute H = G^T * G + lambda * I (k x k), and b = G^T * y0 (k-dim)
    std::vector<std::vector<double>> H(k, std::vector<double>(k, 0.0));
    std::vector<double> b(k, 0.0);

    for (size_t r = 0; r < k; ++r) {
        for (size_t c = 0; c < k; ++c) {
            double sum = 0.0;
            for (size_t i = 0; i < m; ++i) {
                sum += G_[i][r] * G_[i][c];
            }
            H[r][c] = sum;
        }
        H[r][r] += eclr_reg_;

        double sum_b = 0.0;
        for (size_t i = 0; i < m; ++i) {
            sum_b += G_[i][r] * y0[i];
        }
        b[r] = sum_b;
    }

    // 3. Solve H * c = -b for c (using Gauss elimination on k x k, typically k=4)
    std::vector<std::vector<double>> aug(k, std::vector<double>(k + 1, 0.0));
    for (size_t r = 0; r < k; ++r) {
        for (size_t c = 0; c < k; ++c) aug[r][c] = H[r][c];
        aug[r][k] = -b[r];
    }

    for (size_t i = 0; i < k; ++i) {
        double diag = aug[i][i];
        if (std::abs(diag) < 1e-12) diag = 1e-6;
        for (size_t j = 0; j <= k; ++j) aug[i][j] /= diag;
        for (size_t r = 0; r < k; ++r) {
            if (r != i) {
                double factor = aug[r][i];
                for (size_t j = 0; j <= k; ++j) aug[r][j] -= factor * aug[i][j];
            }
        }
    }

    for (size_t i = 0; i < k; ++i) {
        current_coeff_[i] = std::max(-max_thrust_delta_, std::min(max_thrust_delta_, aug[i][k]));
    }
}

void EventCompensator::trigger_event(const std::string& event_name,
                                    double trigger_time,
                                    const std::vector<double>& features) {
    if (type_ == CompensatorType::NONE) {
        active_ = false;
        return;
    }

    active_ = true;
    event_time_ = trigger_time;
    active_event_ = event_name;

    if (type_ == CompensatorType::FIXED_FF || type_ == CompensatorType::ILC) {
        current_coeff_ = fixed_coeff_;
        while (current_coeff_.size() < basis_decay_times_.size()) {
            current_coeff_.push_back(0.0);
        }
    } else if (type_ == CompensatorType::ECLR) {
        solve_eclr_coefficients(features);
    }
}

bool EventCompensator::is_active(double current_time) const {
    if (!active_) return false;
    double dt = current_time - event_time_;
    return dt >= 0.0 && dt <= event_window_duration_;
}

std::vector<double> EventCompensator::evaluate(double current_time, size_t num_rotors) {
    std::vector<double> result(num_rotors, 0.0);
    if (!is_active(current_time)) {
        return result;
    }

    double tau = current_time - event_time_;
    // Compute basis waveform value:
    // u_comp(tau) = sum_k c_k * (tau / decay_k) * exp(-tau / decay_k)
    double scalar_comp = 0.0;
    for (size_t k = 0; k < basis_decay_times_.size() && k < current_coeff_.size(); ++k) {
        double tk = basis_decay_times_[k];
        double val = (tau / tk) * std::exp(-tau / tk);
        scalar_comp += current_coeff_[k] * val;
    }

    // Smooth envelope taper near end of horizon
    if (tau > 0.8 * event_window_duration_) {
        double ramp = (event_window_duration_ - tau) / (0.2 * event_window_duration_);
        scalar_comp *= std::max(0.0, std::min(1.0, ramp));
    }

    // Bound scalar magnitude
    scalar_comp = std::max(-max_thrust_delta_, std::min(max_thrust_delta_, scalar_comp));

    // Distribute scalar compensation differentially to damp roll/pitch
    // For connected 8 rotors: Q1 rotors push opposite to Q2 rotors to counteract pitch/roll transient
    if (num_rotors == 8) {
        // e.g. Differential pitch torque between Q1 and Q2
        for (size_t i = 0; i < 4; ++i) result[i] = +scalar_comp * 0.5;
        for (size_t i = 4; i < 8; ++i) result[i] = -scalar_comp * 0.5;
    } else {
        // Single quadrotor 4 rotors: apply differential pitch
        result[0] = +scalar_comp * 0.5;
        result[1] = -scalar_comp * 0.5;
        result[2] = -scalar_comp * 0.5;
        result[3] = +scalar_comp * 0.5;
    }

    return result;
}

} // namespace docking
