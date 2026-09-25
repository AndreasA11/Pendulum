#include "pendularm/inverse_kinematics.hpp"

#include <algorithm>
#include <cmath>

namespace pendularm {
namespace {

constexpr double kReachTolerance = 1e-10;

IkSolution solve_two_link(double l1, double l2, double x, double y) {
    const double radius = std::hypot(x, y);
    const double outer = l1 + l2;
    const double inner = std::abs(l1 - l2);
    if (radius > outer + kReachTolerance || radius < inner - kReachTolerance) {
        return {false, {}, "Target is outside the two-link reachable region"};
    }

    // Clamp protects exact geometric boundaries from a few ulps of roundoff.
    const double cos_q2 = std::clamp((radius * radius - l1 * l1 - l2 * l2) / (2.0 * l1 * l2), -1.0, 1.0);
    const double q2 = std::acos(cos_q2); // The elbow-down counterpart is equally valid.
    const double q1 = std::atan2(y, x) - std::atan2(l2 * std::sin(q2), l1 + l2 * std::cos(q2));
    return {true, {q1, q2}, {}};
}

} // namespace

IkSolution solve_planar_ik(const ArmParams& params, double x, double y, std::optional<double> phi) {
    if (!params.is_valid()) return {false, {}, "Arm parameters are invalid"};
    if (!std::isfinite(x) || !std::isfinite(y)) return {false, {}, "x and y must be finite numbers"};

    const size_t n = params.num_links();
    if (n == 2) return solve_two_link(params.lengths[0], params.lengths[1], x, y);
    if (n != 3) return {false, {}, "IK supports only two-link and three-link arms"};

    double desired_phi = 0.0;
    if (phi.has_value()) {
        if (!std::isfinite(*phi)) return {false, {}, "phi must be a finite number"};
        desired_phi = *phi;
    } else if (std::hypot(x, y) > kReachTolerance) {
        desired_phi = std::atan2(y, x);
    }

    const double total_extension = params.lengths[0] + params.lengths[1] + params.lengths[2];
    if (std::hypot(x, y) > total_extension + kReachTolerance) {
        return {false, {}, "Target is farther than the arm's total extension"};
    }

    const double wrist_x = x - params.lengths[2] * std::cos(desired_phi);
    const double wrist_y = y - params.lengths[2] * std::sin(desired_phi);
    IkSolution wrist_solution = solve_two_link(params.lengths[0], params.lengths[1], wrist_x, wrist_y);
    if (!wrist_solution.success) {
        wrist_solution.error = "Requested orientation places the wrist outside the two-link reachable region";
        return wrist_solution;
    }
    wrist_solution.positions.push_back(desired_phi - wrist_solution.positions[0] - wrist_solution.positions[1]);
    return wrist_solution;
}

} // namespace pendularm
