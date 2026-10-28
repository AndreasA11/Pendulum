#include <cassert>
#include <cmath>
#include <iostream>

#include "pendularm/arm_dynamics.hpp"
#include "pendularm/inverse_kinematics.hpp"

namespace {
constexpr double kTolerance = 1e-9;

void assert_pose(const pendularm::IkSolution& solution, const pendularm::ArmParams& params,
                 double x, double y, double phi = 0.0, bool check_phi = false) {
    assert(solution.success);
    const auto pose = pendularm::forward_kinematics_end_effector(solution.positions, params);
    assert(std::abs(pose.x - x) < kTolerance);
    assert(std::abs(pose.y - y) < kTolerance);
    if (check_phi) assert(std::abs(pose.phi - phi) < kTolerance);
}
} // namespace

int main() {
    pendularm::ArmParams two{{1.0, 1.0}, {1.0, 1.0}, 9.81};
    assert_pose(pendularm::solve_planar_ik(two, 1.0, 1.0), two, 1.0, 1.0);
    assert_pose(pendularm::solve_planar_ik(two, 2.0, 0.0), two, 2.0, 0.0);
    assert_pose(pendularm::solve_planar_ik(two, 0.0, 0.0), two, 0.0, 0.0);
    assert(!pendularm::solve_planar_ik(two, 2.1, 0.0).success);

    pendularm::ArmParams three{{1.0, 1.0, 1.0}, {1.0, 1.0, 1.0}, 9.81};
    assert_pose(pendularm::solve_planar_ik(three, 2.0, 1.0, 0.0), three, 2.0, 1.0, 0.0, true);
    assert(!pendularm::solve_planar_ik(three, 0.0, 3.0, 0.0).success);
    std::cout << "Inverse kinematics tests passed.\n";
}
