#ifndef PENDULARM_INTEGRATOR_HPP
#define PENDULARM_INTEGRATOR_HPP

#include <vector>
#include <functional>
#include <string>
#include <optional>

namespace pendularm {

// *CHANGES*: Separated declarations from definitions into src/integrator.cpp
// per updated IMPLEMENT.md guideline on source/header separation.

// State representation for a general second-order mechanical system (q, qdot).
// For a 1-DOF system, q and qdot each have size 1.
// For an n-link robot arm, q and qdot each have size n.
struct SecondOrderState {
    std::vector<double> q;
    std::vector<double> qdot;
};

// Acceleration evaluation function signature:
// Given time t, positions q, and velocities qdot, returns accelerations qddot.
using AccelFunc = std::function<std::vector<double>(double t, const std::vector<double>& q, const std::vector<double>& qdot)>;

// Supported numerical integrator types
enum class IntegratorType {
    Euler,
    Midpoint,
    Verlet,
    Rk4
};

// Convert string identifier to IntegratorType enum
std::optional<IntegratorType> parse_integrator_type(const std::string& name);

// Convert IntegratorType enum to string identifier
std::string integrator_type_to_string(IntegratorType type);

// Forward Euler integrator (1st order)
SecondOrderState step_euler(const AccelFunc& f, double t, const SecondOrderState& state, double dt);

// Midpoint integrator (2nd-order Runge-Kutta)
SecondOrderState step_midpoint(const AccelFunc& f, double t, const SecondOrderState& state, double dt);

// Velocity Verlet integrator with predictor-corrector (2nd order)
SecondOrderState step_verlet(const AccelFunc& f, double t, const SecondOrderState& state, double dt);

// Classical 4th-order Runge-Kutta (RK4)
SecondOrderState step_rk4(const AccelFunc& f, double t, const SecondOrderState& state, double dt);

// Dispatch to the requested integrator by type
SecondOrderState integrate_step(
    IntegratorType type,
    const AccelFunc& f,
    double t,
    const SecondOrderState& state,
    double dt
);

} // namespace pendularm

#endif // PENDULARM_INTEGRATOR_HPP
