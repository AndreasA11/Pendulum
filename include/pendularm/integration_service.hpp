#ifndef PENDULARM_INTEGRATION_SERVICE_HPP
#define PENDULARM_INTEGRATION_SERVICE_HPP

#include <vector>
#include <string>
#include <utility>
#include <cmath>
#include <memory>
#include "middleware/json.hpp"
#include "middleware/middleware.hpp"
#include "integrator.hpp"
#include "expression.hpp"

namespace pendularm {

// Handler for the /arm_sim/integration_step service.
// Validates parameters, evaluates math expressions, runs numerical integration,
// and packages times, positions, and velocities arrays.
inline std::pair<bool, middleware::JsonValue> handle_integration_step(const middleware::JsonValue& args) {
    auto make_error = [](const std::string& msg) -> std::pair<bool, middleware::JsonValue> {
        middleware::JsonValue err = middleware::JsonValue::make_object();
        err["error"] = msg;
        return {false, err};
    };

    if (!args.is_object()) {
        return make_error("Request args must be a JSON object");
    }

    // 1. Validate 'function'
    if (!args.contains("function") || !args["function"].is_string()) {
        return make_error("Missing or invalid 'function' field (expected string)");
    }
    std::string func_expr = args["function"].as_string();
    ExprPtr ast;
    try {
        ast = parse_expression(func_expr);
    } catch (const ParseError& e) {
        return make_error("Failed to parse function expression: " + std::string(e.what()));
    } catch (const std::exception& e) {
        return make_error("Unexpected error parsing expression: " + std::string(e.what()));
    }

    // 2. Validate 'integrator'
    if (!args.contains("integrator") || !args["integrator"].is_string()) {
        return make_error("Missing or invalid 'integrator' field (expected string)");
    }
    std::string integrator_name = args["integrator"].as_string();
    auto int_type = parse_integrator_type(integrator_name);
    if (!int_type.has_value()) {
        return make_error("Unknown integrator '" + integrator_name + "'. Expected 'euler', 'midpoint', 'verlet', or 'rk4'");
    }

    // 3. Validate 'dt'
    if (!args.contains("dt") || !args["dt"].is_number()) {
        return make_error("Missing or invalid 'dt' field (expected positive number)");
    }
    double dt = args["dt"].as_number();
    if (dt <= 0.0 || !std::isfinite(dt)) {
        return make_error("'dt' must be strictly positive and finite");
    }

    // 4. Validate 'steps'
    if (!args.contains("steps") || !args["steps"].is_number()) {
        return make_error("Missing or invalid 'steps' field (expected positive integer)");
    }
    double steps_d = args["steps"].as_number();
    if (steps_d < 1.0 || steps_d != std::floor(steps_d) || !std::isfinite(steps_d)) {
        return make_error("'steps' must be a positive integer >= 1");
    }
    uint64_t steps = static_cast<uint64_t>(steps_d);

    // 5. Validate 'x0'
    if (!args.contains("x0") || !args["x0"].is_number()) {
        return make_error("Missing or invalid 'x0' field (expected number)");
    }
    double x0 = args["x0"].as_number();
    if (!std::isfinite(x0)) {
        return make_error("'x0' must be finite");
    }

    // 6. Validate 'xdot0' (optional, defaults to 0.0)
    double xdot0 = 0.0;
    if (args.contains("xdot0")) {
        if (!args["xdot0"].is_number()) {
            return make_error("Invalid 'xdot0' field (expected number)");
        }
        xdot0 = args["xdot0"].as_number();
        if (!std::isfinite(xdot0)) {
            return make_error("'xdot0' must be finite");
        }
    }

    // Prepare numerical integration on 1-DOF particle: qddot = f(t)
    SecondOrderState state;
    state.q = {x0};
    state.qdot = {xdot0};

    auto accel_func = [&ast](double t, const std::vector<double>& /*q*/, const std::vector<double>& /*qdot*/) -> std::vector<double> {
        return {ast->evaluate(t)};
    };

    std::vector<double> times;
    std::vector<double> positions;
    std::vector<double> velocities;
    times.reserve(steps + 1);
    positions.reserve(steps + 1);
    velocities.reserve(steps + 1);

    // Initial state at t = 0 (times[0] == 0, positions[0] == x0, velocities[0] == xdot0)
    double current_t = 0.0;
    times.push_back(current_t);
    positions.push_back(state.q[0]);
    velocities.push_back(state.qdot[0]);

    // Step forward 'steps' times
    for (uint64_t i = 1; i <= steps; ++i) {
        state = integrate_step(*int_type, accel_func, current_t, state, dt);
        current_t = static_cast<double>(i) * dt;
        times.push_back(current_t);
        positions.push_back(state.q[0]);
        velocities.push_back(state.qdot[0]);
    }

    middleware::JsonValue values = middleware::JsonValue::make_object();
    middleware::JsonValue times_arr = middleware::JsonValue::make_array();
    middleware::JsonValue positions_arr = middleware::JsonValue::make_array();
    middleware::JsonValue velocities_arr = middleware::JsonValue::make_array();

    for (size_t i = 0; i <= steps; ++i) {
        times_arr.push_back(times[i]);
        positions_arr.push_back(positions[i]);
        velocities_arr.push_back(velocities[i]);
    }

    values["times"] = std::move(times_arr);
    values["positions"] = std::move(positions_arr);
    values["velocities"] = std::move(velocities_arr);

    return {true, values};
}

// Helper to register the integration step service on the middleware
inline void register_integration_service(middleware::Middleware& mw) {
    mw.advertise_service("/arm_sim/integration_step", handle_integration_step);
}

} // namespace pendularm

#endif // PENDULARM_INTEGRATION_SERVICE_HPP
