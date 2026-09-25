#ifndef PENDULARM_ARM_DYNAMICS_HPP
#define PENDULARM_ARM_DYNAMICS_HPP

#include <vector>
#include <cmath>
#include <functional>
#include "pendularm/integrator.hpp"

namespace pendularm {

// Physical parameters of the planar robot arm
struct ArmParams {
    std::vector<double> lengths; // Link lengths l_i > 0
    std::vector<double> masses;  // Link masses m_i > 0
    double gravity{9.81};        // Gravity magnitude g >= 0 along world -y

    size_t num_links() const {
        return lengths.size();
    }

    bool is_valid() const {
        if (lengths.empty() || lengths.size() != masses.size()) return false;
        if (gravity < 0.0 || !std::isfinite(gravity)) return false;
        for (double l : lengths) {
            if (l <= 0.0 || !std::isfinite(l)) return false;
        }
        for (double m : masses) {
            if (m <= 0.0 || !std::isfinite(m)) return false;
        }
        return true;
    }
};

// 2D Point
struct Point2D {
    double x{0.0};
    double y{0.0};
};

// 2D Pose (position x, y and heading angle phi)
struct Pose2D {
    double x{0.0};
    double y{0.0};
    double phi{0.0};
};

// Forward kinematics: returns world positions of base (0,0) and each joint up to end-effector
std::vector<Point2D> forward_kinematics_joints(const std::vector<double>& q, const ArmParams& params);

// Forward kinematics: returns end-effector pose (x, y, phi_n)
Pose2D forward_kinematics_end_effector(const std::vector<double>& q, const ArmParams& params);

// Compute mass/inertia matrix M(q) for n-link planar manipulator
// Returns an n x n symmetric, positive-definite matrix
std::vector<std::vector<double>> compute_mass_matrix(const std::vector<double>& q, const ArmParams& params);

// Compute Coriolis/centrifugal force term C(q, qdot)*qdot for n-link planar manipulator
std::vector<double> compute_coriolis_vector(
    const std::vector<double>& q,
    const std::vector<double>& qdot,
    const ArmParams& params
);

// Compute gravity-load vector G(q) = dV/dq for n-link planar manipulator
std::vector<double> compute_gravity_vector(const std::vector<double>& q, const ArmParams& params);

// Solve linear system A * x = b using Gaussian elimination with partial pivoting.
std::vector<double> solve_linear_system(
    const std::vector<std::vector<double>>& A,
    const std::vector<double>& b
);

// Forward dynamics: computes qddot = M(q)^-1 * (tau - C(q, qdot)*qdot - G(q))
std::vector<double> forward_dynamics(
    const std::vector<double>& q,
    const std::vector<double>& qdot,
    const std::vector<double>& tau,
    const ArmParams& params
);

// Helper returning an AccelFunc suitable for integration:
// AccelFunc takes (t, q, qdot) and returns qddot.
AccelFunc make_forward_dynamics_accel_func(const std::vector<double>& tau, const ArmParams& params);

} // namespace pendularm

#endif // PENDULARM_ARM_DYNAMICS_HPP
