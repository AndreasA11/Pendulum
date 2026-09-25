#include <iostream>
#include <cassert>
#include <cmath>
#include <vector>
#include "pendularm/arm_dynamics.hpp"
#include "pendularm/integrator.hpp"

using namespace pendularm;

#define ASSERT_TRUE(cond, msg) \
    do { \
        if (!(cond)) { \
            std::cerr << "Assertion FAILED: " << (msg) << " (" << __FILE__ << ":" << __LINE__ << ")\n"; \
            std::exit(1); \
        } \
    } while(0)

#define ASSERT_NEAR(a, b, tol, msg) \
    do { \
        if (std::fabs((a) - (b)) > (tol)) { \
            std::cerr << "Assertion FAILED: " << (msg) << " | " << (a) << " vs " << (b) \
                      << " (diff=" << std::fabs((a) - (b)) << ", tol=" << (tol) << ") at " \
                      << __FILE__ << ":" << __LINE__ << "\n"; \
            std::exit(1); \
        } \
    } while(0)

// Helper to check positive-definiteness via Sylvester's criterion (leading principal minors > 0)
bool is_positive_definite(const std::vector<std::vector<double>>& M) {
    size_t n = M.size();
    if (n == 2) {
        double d1 = M[0][0];
        double d2 = M[0][0] * M[1][1] - M[0][1] * M[1][0];
        return (d1 > 1e-9) && (d2 > 1e-9);
    } else if (n == 3) {
        double d1 = M[0][0];
        double d2 = M[0][0] * M[1][1] - M[0][1] * M[1][0];
        double d3 = M[0][0] * (M[1][1] * M[2][2] - M[1][2] * M[2][1])
                  - M[0][1] * (M[1][0] * M[2][2] - M[1][2] * M[2][0])
                  + M[0][2] * (M[1][0] * M[2][1] - M[1][1] * M[2][0]);
        return (d1 > 1e-9) && (d2 > 1e-9) && (d3 > 1e-9);
    }
    return true;
}

// -------------------------------------------------------------
// Test 1: Mass Matrix M(q) Properties and Analytical Formula
// -------------------------------------------------------------
void test_mass_matrix() {
    std::cout << "[TEST] Mass Matrix M(q) Properties & Analytical Form..." << std::endl;

    // 2-Link Arm comparison with analytical textbook formulas
    ArmParams params2{{1.0, 0.8}, {1.5, 1.2}, 9.81};
    double l1 = params2.lengths[0], l2 = params2.lengths[1];
    double m1 = params2.masses[0], m2 = params2.masses[1];

    std::vector<double> test_angles = {0.0, M_PI / 4.0, M_PI / 2.0, M_PI, -M_PI / 3.0};

    for (double q2 : test_angles) {
        std::vector<double> q = {0.5, q2};
        auto M = compute_mass_matrix(q, params2);

        // 1. Symmetry
        ASSERT_NEAR(M[0][1], M[1][0], 1e-12, "M(q) must be symmetric");

        // 2. Positive-definiteness
        ASSERT_TRUE(is_positive_definite(M), "M(q) must be positive-definite");

        // 3. Analytical formulas:
        // M11 = l1^2 * (1/3 * m1 + m2) + m2 * l1 * l2 * cos(q2) + 1/3 * m2 * l2^2
        // M12 = M21 = 1/2 * m2 * l1 * l2 * cos(q2) + 1/3 * m2 * l2^2
        // M22 = 1/3 * m2 * l2^2
        double expected_M11 = l1 * l1 * ((1.0 / 3.0) * m1 + m2) + m2 * l1 * l2 * std::cos(q2) + (1.0 / 3.0) * m2 * l2 * l2;
        double expected_M12 = 0.5 * m2 * l1 * l2 * std::cos(q2) + (1.0 / 3.0) * m2 * l2 * l2;
        double expected_M22 = (1.0 / 3.0) * m2 * l2 * l2;

        ASSERT_NEAR(M[0][0], expected_M11, 1e-12, "M11 analytical match");
        ASSERT_NEAR(M[0][1], expected_M12, 1e-12, "M12 analytical match");
        ASSERT_NEAR(M[1][1], expected_M22, 1e-12, "M22 analytical match");
    }

    // 3-Link Arm Properties
    ArmParams params3{{1.0, 0.8, 0.6}, {1.5, 1.2, 0.9}, 9.81};
    std::vector<double> q3 = {0.2, -0.4, 0.7};
    auto M3 = compute_mass_matrix(q3, params3);

    ASSERT_TRUE(M3.size() == 3 && M3[0].size() == 3, "3-link M dimensions");
    ASSERT_NEAR(M3[0][1], M3[1][0], 1e-12, "M3(0,1) symmetry");
    ASSERT_NEAR(M3[0][2], M3[2][0], 1e-12, "M3(0,2) symmetry");
    ASSERT_NEAR(M3[1][2], M3[2][1], 1e-12, "M3(1,2) symmetry");
    ASSERT_TRUE(is_positive_definite(M3), "3-link M must be positive-definite");

    // Coupling: off-diagonal elements must be nonzero
    ASSERT_TRUE(std::fabs(M3[0][1]) > 1e-4, "Inter-joint coupling M(0,1)");
    ASSERT_TRUE(std::fabs(M3[1][2]) > 1e-4, "Inter-joint coupling M(1,2)");

    std::cout << "  -> PASSED" << std::endl;
}

// -------------------------------------------------------------
// Test 2: Coriolis & Centrifugal Coupling
// -------------------------------------------------------------
void test_coriolis_coupling() {
    std::cout << "[TEST] Coriolis & Centrifugal Coupling..." << std::endl;

    ArmParams params2{{1.0, 0.8}, {1.5, 1.2}, 9.81};
    double l1 = params2.lengths[0], l2 = params2.lengths[1];
    double m2 = params2.masses[1];

    std::vector<double> q = {0.3, M_PI / 4.0};
    double q2 = q[1];
    std::vector<double> qdot = {1.5, -2.0};
    double q1_dot = qdot[0], q2_dot = qdot[1];

    auto C_qdot = compute_coriolis_vector(q, qdot, params2);

    // Textbook formulas:
    // h = m2 * l1 * (l2 / 2) * sin(q2)
    // c1 = -h * (2 * q1_dot * q2_dot + q2_dot^2)
    // c2 = h * q1_dot^2
    double h = m2 * l1 * (0.5 * l2) * std::sin(q2);
    double expected_c1 = -h * (2.0 * q1_dot * q2_dot + q2_dot * q2_dot);
    double expected_c2 = h * (q1_dot * q1_dot);

    ASSERT_NEAR(C_qdot[0], expected_c1, 1e-12, "2-link Coriolis c1 match");
    ASSERT_NEAR(C_qdot[1], expected_c2, 1e-12, "2-link Coriolis c2 match");

    // Inter-joint coupling check: joint 1 moving with joint 2 stationary induces torque on joint 2
    std::vector<double> qdot_only1 = {2.0, 0.0};
    auto C_only1 = compute_coriolis_vector(q, qdot_only1, params2);
    ASSERT_TRUE(std::fabs(C_only1[1]) > 1e-4, "Moving joint 1 must exert Coriolis load on joint 2");

    // 3-Link Arm coupling check
    ArmParams params3{{1.0, 0.8, 0.6}, {1.5, 1.2, 0.9}, 9.81};
    std::vector<double> q3 = {0.5, 0.5, 0.5};
    std::vector<double> q3_dot = {1.0, 0.0, 0.0}; // Only joint 1 rotating
    auto C3 = compute_coriolis_vector(q3, q3_dot, params3);
    // Rotating link 1 exerts centrifugal forces on links 2 and 3!
    ASSERT_TRUE(std::fabs(C3[1]) > 1e-4, "3-link Coriolis coupling to joint 2");
    ASSERT_TRUE(std::fabs(C3[2]) > 1e-4, "3-link Coriolis coupling to joint 3");

    std::cout << "  -> PASSED" << std::endl;
}

// -------------------------------------------------------------
// Test 3: Gravity Load Vector G(q)
// -------------------------------------------------------------
void test_gravity_load() {
    std::cout << "[TEST] Gravity Load Vector G(q)..." << std::endl;

    ArmParams params2{{1.0, 0.8}, {1.5, 1.2}, 9.81};
    double l1 = params2.lengths[0], l2 = params2.lengths[1];
    double m1 = params2.masses[0], m2 = params2.masses[1];
    double g = params2.gravity;

    // 1. Analytical comparison for 2-link
    std::vector<double> q = {0.4, 0.6};
    double phi1 = q[0];
    double phi2 = q[0] + q[1];

    auto G = compute_gravity_vector(q, params2);

    double expected_G1 = g * l1 * (0.5 * m1 + m2) * std::cos(phi1) + g * l2 * (0.5 * m2) * std::cos(phi2);
    double expected_G2 = g * l2 * (0.5 * m2) * std::cos(phi2);

    ASSERT_NEAR(G[0], expected_G1, 1e-12, "2-link G1 match");
    ASSERT_NEAR(G[1], expected_G2, 1e-12, "2-link G2 match");

    // 2. Equilibria checks:
    // Hanging straight down: phi1 = -pi/2, phi2 = -pi/2
    std::vector<double> q_down = {-M_PI / 2.0, 0.0};
    auto G_down = compute_gravity_vector(q_down, params2);
    ASSERT_NEAR(G_down[0], 0.0, 1e-12, "Hanging pose has zero gravity load (joint 1)");
    ASSERT_NEAR(G_down[1], 0.0, 1e-12, "Hanging pose has zero gravity load (joint 2)");

    // Pointing straight up: phi1 = pi/2, phi2 = pi/2
    std::vector<double> q_up = {M_PI / 2.0, 0.0};
    auto G_up = compute_gravity_vector(q_up, params2);
    ASSERT_NEAR(G_up[0], 0.0, 1e-12, "Upward pose has zero gravity load (joint 1)");
    ASSERT_NEAR(G_up[1], 0.0, 1e-12, "Upward pose has zero gravity load (joint 2)");

    // 3. Steady-State Equilibrium: tau = G(q) implies qddot = 0
    std::vector<double> q_hold = {0.3, -0.5};
    std::vector<double> qdot_zero = {0.0, 0.0};
    auto G_hold = compute_gravity_vector(q_hold, params2);
    auto qddot_hold = forward_dynamics(q_hold, qdot_zero, G_hold, params2);

    ASSERT_NEAR(qddot_hold[0], 0.0, 1e-12, "Holding torque tau=G(q) results in zero acceleration (joint 1)");
    ASSERT_NEAR(qddot_hold[1], 0.0, 1e-12, "Holding torque tau=G(q) results in zero acceleration (joint 2)");

    std::cout << "  -> PASSED" << std::endl;
}

// -------------------------------------------------------------
// Test 4: Free Motion Under Gravity
// -------------------------------------------------------------
void test_free_motion() {
    std::cout << "[TEST] Free Motion Under Gravity (tau = 0)..." << std::endl;

    // Horizontal 2-link arm with tau = 0
    ArmParams params2{{1.0, 0.8}, {1.5, 1.2}, 9.81};
    std::vector<double> q_start = {0.0, 0.0}; // Horizontal along +x
    std::vector<double> qdot_zero = {0.0, 0.0};
    std::vector<double> tau_zero = {0.0, 0.0};

    auto qddot = forward_dynamics(q_start, qdot_zero, tau_zero, params2);

    // Gravity pulls down (-y), causing clockwise rotation (negative qddot in standard planar convention)
    ASSERT_TRUE(qddot[0] < -0.1, "Joint 1 must accelerate downward under gravity");

    // Integrate forward 0.5s with RK4 to ensure state evolves over time
    auto accel_fn = make_forward_dynamics_accel_func(tau_zero, params2);
    SecondOrderState s{q_start, qdot_zero};
    double dt = 0.01;
    for (int i = 0; i < 50; ++i) {
        s = integrate_step(IntegratorType::Rk4, accel_fn, i * dt, s, dt);
    }

    // Arm must have moved from initial position
    ASSERT_TRUE(std::fabs(s.q[0] - q_start[0]) > 0.05, "Arm position must change over time during free fall");
    ASSERT_TRUE(std::fabs(s.qdot[0]) > 0.05, "Arm velocity must become nonzero during free fall");

    std::cout << "  -> PASSED" << std::endl;
}

// -------------------------------------------------------------
// Test 5: Linear Solver
// -------------------------------------------------------------
void test_linear_solver() {
    std::cout << "[TEST] Linear Solver Accuracy & Pivot Handling..." << std::endl;

    // 1. Known 3x3 system:
    // [ 2  1 -1 ] [x0]   [ 8 ]
    // [-3 -1  2 ] [x1] = [-11]
    // [-2  1  2 ] [x2]   [ -3]
    // Solution: x = [2, 3, -1]
    std::vector<std::vector<double>> A = {
        { 2.0,  1.0, -1.0},
        {-3.0, -1.0,  2.0},
        {-2.0,  1.0,  2.0}
    };
    std::vector<double> b = {8.0, -11.0, -3.0};

    auto x = solve_linear_system(A, b);
    ASSERT_NEAR(x[0],  2.0, 1e-12, "Linear solve x0");
    ASSERT_NEAR(x[1],  3.0, 1e-12, "Linear solve x1");
    ASSERT_NEAR(x[2], -1.0, 1e-12, "Linear solve x2");

    // 2. Singular matrix detection
    std::vector<std::vector<double>> A_sing = {
        {1.0, 2.0},
        {2.0, 4.0}
    };
    std::vector<double> b_sing = {1.0, 2.0};
    bool caught = false;
    try {
        solve_linear_system(A_sing, b_sing);
    } catch (const std::runtime_error&) {
        caught = true;
    }
    ASSERT_TRUE(caught, "Linear solver must detect singular matrix");

    std::cout << "  -> PASSED" << std::endl;
}

// -------------------------------------------------------------
// Test 6: Forward Kinematics
// -------------------------------------------------------------
void test_forward_kinematics() {
    std::cout << "[TEST] Forward Kinematics Geometry..." << std::endl;

    ArmParams params2{{1.0, 0.5}, {1.0, 1.0}, 9.81};

    // Case 1: Fully extended along +x (q1 = 0, q2 = 0)
    {
        Pose2D ee = forward_kinematics_end_effector({0.0, 0.0}, params2);
        ASSERT_NEAR(ee.x, 1.5, 1e-12, "Extended x");
        ASSERT_NEAR(ee.y, 0.0, 1e-12, "Extended y");
        ASSERT_NEAR(ee.phi, 0.0, 1e-12, "Extended phi");
    }

    // Case 2: Right-angle elbow (q1 = 0, q2 = pi/2)
    {
        Pose2D ee = forward_kinematics_end_effector({0.0, M_PI / 2.0}, params2);
        ASSERT_NEAR(ee.x, 1.0, 1e-12, "Elbow right angle x");
        ASSERT_NEAR(ee.y, 0.5, 1e-12, "Elbow right angle y");
        ASSERT_NEAR(ee.phi, M_PI / 2.0, 1e-12, "Elbow right angle phi");
    }

    // Case 3: Fully folded arm (q1 = 0, q2 = pi)
    {
        Pose2D ee = forward_kinematics_end_effector({0.0, M_PI}, params2);
        ASSERT_NEAR(ee.x, 0.5, 1e-12, "Folded x");
        ASSERT_NEAR(ee.y, 0.0, 1e-12, "Folded y");
        ASSERT_NEAR(ee.phi, M_PI, 1e-12, "Folded phi");
    }

    // Case 4: 3-Link Arm All Joint Positions
    ArmParams params3{{1.0, 1.0, 0.5}, {1.0, 1.0, 1.0}, 9.81};
    auto joints = forward_kinematics_joints({0.0, M_PI / 2.0, -M_PI / 2.0}, params3);
    ASSERT_TRUE(joints.size() == 4, "4 joint points for 3-link arm (including base)");
    ASSERT_NEAR(joints[0].x, 0.0, 1e-12, "Base x");
    ASSERT_NEAR(joints[0].y, 0.0, 1e-12, "Base y");
    ASSERT_NEAR(joints[1].x, 1.0, 1e-12, "Joint 1 x");
    ASSERT_NEAR(joints[1].y, 0.0, 1e-12, "Joint 1 y");
    ASSERT_NEAR(joints[2].x, 1.0, 1e-12, "Joint 2 x");
    ASSERT_NEAR(joints[2].y, 1.0, 1e-12, "Joint 2 y");
    ASSERT_NEAR(joints[3].x, 1.5, 1e-12, "End-effector x");
    ASSERT_NEAR(joints[3].y, 1.0, 1e-12, "End-effector y");

    std::cout << "  -> PASSED" << std::endl;
}

int main() {
    std::cout << "========================================" << std::endl;
    std::cout << "Running Planar Arm Dynamics Tests" << std::endl;
    std::cout << "========================================" << std::endl;

    test_mass_matrix();
    test_coriolis_coupling();
    test_gravity_load();
    test_free_motion();
    test_linear_solver();
    test_forward_kinematics();

    std::cout << "========================================" << std::endl;
    std::cout << "All Arm Dynamics Tests PASSED Successfully!" << std::endl;
    std::cout << "========================================" << std::endl;
    return 0;
}
