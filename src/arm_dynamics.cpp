#include "pendularm/arm_dynamics.hpp"
#include <stdexcept>
#include <cmath>
#include <algorithm>

namespace pendularm {

// *CHANGES*: Implemented closed-form Lagrangian dynamics for general n-link planar arm,
// partial-pivoting linear solver, and forward kinematics per prompts/02_arm_dynamics.md.

std::vector<Point2D> forward_kinematics_joints(const std::vector<double>& q, const ArmParams& params) {
    size_t n = params.num_links();
    if (q.size() != n) {
        throw std::invalid_argument("Joint angle vector size does not match number of links");
    }

    std::vector<Point2D> joints;
    joints.reserve(n + 1);

    // Base joint at origin (0, 0)
    joints.push_back({0.0, 0.0});

    double current_x = 0.0;
    double current_y = 0.0;
    double running_phi = 0.0;

    for (size_t i = 0; i < n; ++i) {
        running_phi += q[i];
        current_x += params.lengths[i] * std::cos(running_phi);
        current_y += params.lengths[i] * std::sin(running_phi);
        joints.push_back({current_x, current_y});
    }

    return joints;
}

Pose2D forward_kinematics_end_effector(const std::vector<double>& q, const ArmParams& params) {
    size_t n = params.num_links();
    if (q.size() != n) {
        throw std::invalid_argument("Joint angle vector size does not match number of links");
    }

    double current_x = 0.0;
    double current_y = 0.0;
    double running_phi = 0.0;

    for (size_t i = 0; i < n; ++i) {
        running_phi += q[i];
        current_x += params.lengths[i] * std::cos(running_phi);
        current_y += params.lengths[i] * std::sin(running_phi);
    }

    return {current_x, current_y, running_phi};
}

std::vector<std::vector<double>> compute_mass_matrix(const std::vector<double>& q, const ArmParams& params) {
    size_t n = params.num_links();
    if (q.size() != n) {
        throw std::invalid_argument("q size mismatch in compute_mass_matrix");
    }

    // Compute running absolute orientations phi_i = sum_{k=0}^i q_k
    std::vector<double> phi(n, 0.0);
    double accum_phi = 0.0;
    for (size_t i = 0; i < n; ++i) {
        accum_phi += q[i];
        phi[i] = accum_phi;
    }

    // Compute A matrix in terms of phi coordinates:
    // T = 0.5 * phi_dot^T * A * phi_dot
    // For uniform rods:
    // A_{j,j} = l_j^2 * (1/3 * m_j + sum_{i=j+1}^{n-1} m_i)
    // A_{j,k} = l_j * l_k * cos(phi_j - phi_k) * (1/2 * m_{max(j,k)} + sum_{i=max(j,k)+1}^{n-1} m_i)
    std::vector<std::vector<double>> A(n, std::vector<double>(n, 0.0));

    for (size_t j = 0; j < n; ++j) {
        for (size_t k = 0; k < n; ++k) {
            size_t max_jk = std::max(j, k);
            double outer_mass_sum = 0.0;
            for (size_t i = max_jk + 1; i < n; ++i) {
                outer_mass_sum += params.masses[i];
            }

            if (j == k) {
                double eff_mass = (1.0 / 3.0) * params.masses[j] + outer_mass_sum;
                A[j][j] = params.lengths[j] * params.lengths[j] * eff_mass;
            } else {
                double eff_mass = 0.5 * params.masses[max_jk] + outer_mass_sum;
                A[j][k] = params.lengths[j] * params.lengths[k] * std::cos(phi[j] - phi[k]) * eff_mass;
            }
        }
    }

    // Transform from phi_dot to q_dot:
    // phi_dot = S * q_dot where S_{j,a} = 1 if a <= j, else 0.
    // M = S^T * A * S, so M_{a,b} = sum_{j=a}^{n-1} sum_{k=b}^{n-1} A_{j,k}
    std::vector<std::vector<double>> M(n, std::vector<double>(n, 0.0));
    for (size_t a = 0; a < n; ++a) {
        for (size_t b = 0; b < n; ++b) {
            double sum = 0.0;
            for (size_t j = a; j < n; ++j) {
                for (size_t k = b; k < n; ++k) {
                    sum += A[j][k];
                }
            }
            M[a][b] = sum;
        }
    }

    return M;
}

std::vector<double> compute_coriolis_vector(
    const std::vector<double>& q,
    const std::vector<double>& qdot,
    const ArmParams& params
) {
    size_t n = params.num_links();
    if (q.size() != n || qdot.size() != n) {
        throw std::invalid_argument("Size mismatch in compute_coriolis_vector");
    }

    std::vector<double> phi(n, 0.0);
    std::vector<double> phidot(n, 0.0);
    double accum_phi = 0.0;
    double accum_phidot = 0.0;
    for (size_t i = 0; i < n; ++i) {
        accum_phi += q[i];
        accum_phidot += qdot[i];
        phi[i] = accum_phi;
        phidot[i] = accum_phidot;
    }

    // beta_{j,k} coefficients for j != k:
    // beta_{j,k} = l_j * l_k * (1/2 * m_{max(j,k)} + sum_{i=max(j,k)+1}^{n-1} m_i)
    // B_j = sum_{k=0}^{n-1} beta_{j,k} * sin(phi_j - phi_k) * phidot_k^2
    std::vector<double> B(n, 0.0);
    for (size_t j = 0; j < n; ++j) {
        double sum = 0.0;
        for (size_t k = 0; k < n; ++k) {
            if (j == k) continue;
            size_t max_jk = std::max(j, k);
            double outer_mass_sum = 0.0;
            for (size_t i = max_jk + 1; i < n; ++i) {
                outer_mass_sum += params.masses[i];
            }
            double beta_jk = params.lengths[j] * params.lengths[k] * (0.5 * params.masses[max_jk] + outer_mass_sum);
            sum += beta_jk * std::sin(phi[j] - phi[k]) * (phidot[k] * phidot[k]);
        }
        B[j] = sum;
    }

    // In terms of joint generalized coordinates q_a:
    // [C * qdot]_a = sum_{j=a}^{n-1} B_j
    std::vector<double> C_qdot(n, 0.0);
    for (size_t a = 0; a < n; ++a) {
        double sum = 0.0;
        for (size_t j = a; j < n; ++j) {
            sum += B[j];
        }
        C_qdot[a] = sum;
    }

    return C_qdot;
}

std::vector<double> compute_gravity_vector(const std::vector<double>& q, const ArmParams& params) {
    size_t n = params.num_links();
    if (q.size() != n) {
        throw std::invalid_argument("Size mismatch in compute_gravity_vector");
    }

    std::vector<double> phi(n, 0.0);
    double accum_phi = 0.0;
    for (size_t i = 0; i < n; ++i) {
        accum_phi += q[i];
        phi[i] = accum_phi;
    }

    // Potential energy V = g * sum_{j=0}^{n-1} l_j * sin(phi_j) * (1/2 * m_j + sum_{i=j+1}^{n-1} m_i)
    // G_a = dV / dq_a = sum_{j=a}^{n-1} gamma_j * cos(phi_j)
    // where gamma_j = g * l_j * (1/2 * m_j + sum_{i=j+1}^{n-1} m_i)
    std::vector<double> gamma(n, 0.0);
    for (size_t j = 0; j < n; ++j) {
        double outer_mass_sum = 0.0;
        for (size_t i = j + 1; i < n; ++i) {
            outer_mass_sum += params.masses[i];
        }
        gamma[j] = params.gravity * params.lengths[j] * (0.5 * params.masses[j] + outer_mass_sum);
    }

    std::vector<double> G(n, 0.0);
    for (size_t a = 0; a < n; ++a) {
        double sum = 0.0;
        for (size_t j = a; j < n; ++j) {
            sum += gamma[j] * std::cos(phi[j]);
        }
        G[a] = sum;
    }

    return G;
}

std::vector<double> solve_linear_system(
    const std::vector<std::vector<double>>& A,
    const std::vector<double>& b
) {
    size_t n = A.size();
    if (n == 0 || b.size() != n) {
        throw std::invalid_argument("Dimension mismatch in solve_linear_system");
    }

    // Build augmented matrix [A | b]
    std::vector<std::vector<double>> aug(n, std::vector<double>(n + 1, 0.0));
    for (size_t i = 0; i < n; ++i) {
        if (A[i].size() != n) {
            throw std::invalid_argument("Matrix A is not square in solve_linear_system");
        }
        for (size_t j = 0; j < n; ++j) {
            aug[i][j] = A[i][j];
        }
        aug[i][n] = b[i];
    }

    // Gaussian elimination with partial pivoting
    for (size_t col = 0; col < n; ++col) {
        // Find pivot
        size_t pivot_row = col;
        double max_val = std::fabs(aug[col][col]);
        for (size_t row = col + 1; row < n; ++row) {
            double v = std::fabs(aug[row][col]);
            if (v > max_val) {
                max_val = v;
                pivot_row = row;
            }
        }

        if (max_val < 1e-13) {
            throw std::runtime_error("Singular matrix in solve_linear_system");
        }

        // Swap pivot row
        if (pivot_row != col) {
            std::swap(aug[col], aug[pivot_row]);
        }

        // Eliminate below
        for (size_t row = col + 1; row < n; ++row) {
            double factor = aug[row][col] / aug[col][col];
            for (size_t j = col; j <= n; ++j) {
                aug[row][j] -= factor * aug[col][j];
            }
        }
    }

    // Back substitution
    std::vector<double> x(n, 0.0);
    for (int i = static_cast<int>(n) - 1; i >= 0; --i) {
        double sum = aug[i][n];
        for (size_t j = static_cast<size_t>(i) + 1; j < n; ++j) {
            sum -= aug[i][j] * x[j];
        }
        x[i] = sum / aug[i][i];
    }

    return x;
}

std::vector<double> forward_dynamics(
    const std::vector<double>& q,
    const std::vector<double>& qdot,
    const std::vector<double>& tau,
    const ArmParams& params
) {
    size_t n = params.num_links();
    if (q.size() != n || qdot.size() != n || tau.size() != n) {
        throw std::invalid_argument("Size mismatch in forward_dynamics");
    }

    std::vector<std::vector<double>> M = compute_mass_matrix(q, params);
    std::vector<double> C_qdot = compute_coriolis_vector(q, qdot, params);
    std::vector<double> G = compute_gravity_vector(q, params);

    // RHS = tau - C*qdot - G
    std::vector<double> rhs(n, 0.0);
    for (size_t i = 0; i < n; ++i) {
        rhs[i] = tau[i] - C_qdot[i] - G[i];
    }

    return solve_linear_system(M, rhs);
}

AccelFunc make_forward_dynamics_accel_func(const std::vector<double>& tau, const ArmParams& params) {
    return [tau, params](double /*t*/, const std::vector<double>& q, const std::vector<double>& qdot) -> std::vector<double> {
        return forward_dynamics(q, qdot, tau, params);
    };
}

} // namespace pendularm
