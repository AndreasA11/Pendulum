#ifndef PENDULARM_INTEGRATION_SERVICE_HPP
#define PENDULARM_INTEGRATION_SERVICE_HPP

#include <utility>
#include "middleware/json.hpp"
#include "middleware/middleware.hpp"

namespace pendularm {

// *CHANGES*: Separated declarations from definitions into src/integration_service.cpp
// per updated IMPLEMENT.md guideline on source/header separation.

// Handler for the /arm_sim/integration_step service.
// Validates parameters, evaluates math expressions, runs numerical integration,
// and packages times, positions, and velocities arrays.
std::pair<bool, middleware::JsonValue> handle_integration_step(const middleware::JsonValue& args);

// Helper to register the integration step service on the middleware
void register_integration_service(middleware::Middleware& mw);

} // namespace pendularm

#endif // PENDULARM_INTEGRATION_SERVICE_HPP
