#ifndef PENDULARM_INTEGRATOR_HPP
#define PENDULARM_INTEGRATOR_HPP

#include <vector>
#include <functional>
#include <string>
#include <optional>
#include <stdexcept>
#include <cmath>

namespace pendularm {

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
inline std::optional<IntegratorType> parse_integrator_type(const std::string& name) {
    if (name == "euler") return IntegratorType::Euler;
    if (name == "midpoint") return IntegratorType::Midpoint;
    if (name == "verlet") return IntegratorType::Verlet;
    if (name == "rk4") return IntegratorType::Rk4;
    return std::nullopt;
}

// Convert IntegratorType enum to string identifier
inline std::string integrator_type_to_string(IntegratorType type) {
    switch (type) {
        case IntegratorType::Euler: return "euler";
        case IntegratorType::Midpoint: return "midpoint";
        case IntegratorType::Verlet: return "verlet";
        case IntegratorType::Rk4: return "rk4";
    }
    return "unknown";
}

/*
Forward Euler integrator:
Single evaluation of acceleration at (t, q_n, qdot_n).
q_{n+1}    = q_n + qdot_n * dt
qdot_{n+1} = qdot_n + qddot_n * dt
First-order accurate.
*/
inline SecondOrderState step_euler(const AccelFunc& f, double t, const SecondOrderState& state, double dt) {
    const size_t n = state.q.size();
    std::vector<double> a0 = f(t, state.q, state.qdot);

    SecondOrderState next;
    next.q.resize(n);
    next.qdot.resize(n);
    for (size_t i = 0; i < n; ++i) {
        next.q[i] = state.q[i] + state.qdot[i] * dt;
        next.qdot[i] = state.qdot[i] + a0[i] * dt;
    }
    return next;
}

/*
Midpoint integrator (2nd-order Runge-Kutta):
1. Evaluate acceleration a0 at current state (t, q, qdot).
2. Take trial half-step to midpoint:
   q_mid    = q + qdot * (dt / 2)
   qdot_mid = qdot + a0 * (dt / 2)
3. Evaluate acceleration a_mid at (t + dt/2, q_mid, qdot_mid).
4. Step forward using midpoint derivatives:
   q_{n+1}    = q + qdot_mid * dt
   qdot_{n+1} = qdot + a_mid * dt
Two evaluations per step, second-order accurate.
*/
inline SecondOrderState step_midpoint(const AccelFunc& f, double t, const SecondOrderState& state, double dt) {
    const size_t n = state.q.size();
    const double half_dt = 0.5 * dt;

    std::vector<double> a0 = f(t, state.q, state.qdot);

    std::vector<double> q_mid(n);
    std::vector<double> qdot_mid(n);
    for (size_t i = 0; i < n; ++i) {
        q_mid[i] = state.q[i] + state.qdot[i] * half_dt;
        qdot_mid[i] = state.qdot[i] + a0[i] * half_dt;
    }

    std::vector<double> a_mid = f(t + half_dt, q_mid, qdot_mid);

    SecondOrderState next;
    next.q.resize(n);
    next.qdot.resize(n);
    for (size_t i = 0; i < n; ++i) {
        next.q[i] = state.q[i] + qdot_mid[i] * dt;
        next.qdot[i] = state.qdot[i] + a_mid[i] * dt;
    }
    return next;
}

/*
Velocity Verlet integrator with predictor-corrector:
Accommodates velocity-dependent forces (e.g. Coriolis terms in multi-link dynamics).
1. Evaluate acceleration a0 at current state (t, q, qdot).
2. Predict new position:
   q_{n+1} = q + qdot * dt + 0.5 * a0 * dt^2
3. Provisionally estimate new velocity:
   qdot_pred = qdot + a0 * dt
4. Evaluate acceleration a1 at predicted state (t + dt, q_{n+1}, qdot_pred).
5. Finalize velocity by averaging the two acceleration evaluations:
   qdot_{n+1} = qdot + 0.5 * (a0 + a1) * dt
Two evaluations per step, second-order accurate.
*/
inline SecondOrderState step_verlet(const AccelFunc& f, double t, const SecondOrderState& state, double dt) {
    const size_t n = state.q.size();
    std::vector<double> a0 = f(t, state.q, state.qdot);

    SecondOrderState next;
    next.q.resize(n);
    std::vector<double> qdot_pred(n);
    const double half_dt_sq = 0.5 * dt * dt;

    for (size_t i = 0; i < n; ++i) {
        next.q[i] = state.q[i] + state.qdot[i] * dt + a0[i] * half_dt_sq;
        qdot_pred[i] = state.qdot[i] + a0[i] * dt;
    }

    std::vector<double> a1 = f(t + dt, next.q, qdot_pred);

    next.qdot.resize(n);
    const double half_dt = 0.5 * dt;
    for (size_t i = 0; i < n; ++i) {
        next.qdot[i] = state.qdot[i] + (a0[i] + a1[i]) * half_dt;
    }
    return next;
}

/*
Classical 4th-order Runge-Kutta (RK4):
Converts second-order system into first-order state [q, v]^T.
Four acceleration evaluations at stages:
- Stage 1: t, [q, v]
- Stage 2: t + dt/2, trial state from stage 1
- Stage 3: t + dt/2, trial state from stage 2
- Stage 4: t + dt, trial state from stage 3
Blended with 1:2:2:1 weights. Fourth-order accurate.
*/
inline SecondOrderState step_rk4(const AccelFunc& f, double t, const SecondOrderState& state, double dt) {
    const size_t n = state.q.size();
    const double half_dt = 0.5 * dt;

    // Stage 1 at t
    std::vector<double> k1_v = f(t, state.q, state.qdot);
    const std::vector<double>& k1_q = state.qdot;

    // Stage 2 at t + dt/2
    std::vector<double> q_trial1(n);
    std::vector<double> v_trial1(n);
    for (size_t i = 0; i < n; ++i) {
        q_trial1[i] = state.q[i] + k1_q[i] * half_dt;
        v_trial1[i] = state.qdot[i] + k1_v[i] * half_dt;
    }
    std::vector<double> k2_v = f(t + half_dt, q_trial1, v_trial1);
    const std::vector<double>& k2_q = v_trial1;

    // Stage 3 at t + dt/2
    std::vector<double> q_trial2(n);
    std::vector<double> v_trial2(n);
    for (size_t i = 0; i < n; ++i) {
        q_trial2[i] = state.q[i] + k2_q[i] * half_dt;
        v_trial2[i] = state.qdot[i] + k2_v[i] * half_dt;
    }
    std::vector<double> k3_v = f(t + half_dt, q_trial2, v_trial2);
    const std::vector<double>& k3_q = v_trial2;

    // Stage 4 at t + dt
    std::vector<double> q_trial3(n);
    std::vector<double> v_trial3(n);
    for (size_t i = 0; i < n; ++i) {
        q_trial3[i] = state.q[i] + k3_q[i] * dt;
        v_trial3[i] = state.qdot[i] + k3_v[i] * dt;
    }
    std::vector<double> k4_v = f(t + dt, q_trial3, v_trial3);
    const std::vector<double>& k4_q = v_trial3;

    // Combine with 1 : 2 : 2 : 1 weighting
    SecondOrderState next;
    next.q.resize(n);
    next.qdot.resize(n);
    const double sixth_dt = dt / 6.0;
    for (size_t i = 0; i < n; ++i) {
        next.q[i] = state.q[i] + sixth_dt * (k1_q[i] + 2.0 * k2_q[i] + 2.0 * k3_q[i] + k4_q[i]);
        next.qdot[i] = state.qdot[i] + sixth_dt * (k1_v[i] + 2.0 * k2_v[i] + 2.0 * k3_v[i] + k4_v[i]);
    }
    return next;
}

// Dispatch to the requested integrator by type
inline SecondOrderState integrate_step(
    IntegratorType type,
    const AccelFunc& f,
    double t,
    const SecondOrderState& state,
    double dt
) {
    switch (type) {
        case IntegratorType::Euler:
            return step_euler(f, t, state, dt);
        case IntegratorType::Midpoint:
            return step_midpoint(f, t, state, dt);
        case IntegratorType::Verlet:
            return step_verlet(f, t, state, dt);
        case IntegratorType::Rk4:
            return step_rk4(f, t, state, dt);
    }
    throw std::invalid_argument("Unknown integrator type");
}

} // namespace pendularm

#endif // PENDULARM_INTEGRATOR_HPP
