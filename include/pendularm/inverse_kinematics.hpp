#ifndef PENDULARM_INVERSE_KINEMATICS_HPP
#define PENDULARM_INVERSE_KINEMATICS_HPP

#include <optional>
#include <string>
#include <vector>

#include "pendularm/arm_dynamics.hpp"

namespace pendularm {

struct IkSolution {
    bool success{false};
    std::vector<double> positions;
    std::string error;
};

// Closed-form planar IK for the supported two- and three-link arms.  For a
// three-link solve without an orientation, the end-effector is aimed radially
// at the requested position.
IkSolution solve_planar_ik(const ArmParams& params, double x, double y,
                           std::optional<double> phi = std::nullopt);

} // namespace pendularm

#endif
