#include <cassert>
#include <iostream>

#include "middleware/middleware.hpp"
#include "pendularm/arm_simulation.hpp"
#include "pendularm/integration_service.hpp"

int main() {
    middleware::Middleware middleware;
    pendularm::register_integration_service(middleware);
    pendularm::ArmSimulation simulation(middleware, 2);

    const char* required_services[] = {
        "/arm_sim/integration_step", "/arm_sim/set_integrator", "/arm_sim/set_params",
        "/arm_sim/pause", "/arm_sim/reset", "/pid_controller/enable",
        "/pid_controller/set_gains", "/ik/solve", "/ik_action/send_goal",
        "/ik_action/cancel_goal", "/ik_trial/start", "/ik_trial/skip", "/ik_trial/stop"
    };
    for (const char* service : required_services) assert(middleware.has_service(service));

    const auto query = middleware::JsonValue::make_object();
    auto [queried, current] = middleware.call_service("/arm_sim/set_params", query);
    assert(queried);
    assert(current["gravity"].as_number() == 9.81);
    assert(current["masses"].as_array().size() == 2);
    assert(current["lengths"].as_array().size() == 2);

    middleware::JsonValue partial = middleware::JsonValue::make_object();
    partial["gravity"] = 3.0;
    middleware::JsonValue invalid_masses = middleware::JsonValue::make_array();
    invalid_masses.push_back(1.0);
    partial["masses"] = invalid_masses;
    auto [partially_rejected, updated] = middleware.call_service("/arm_sim/set_params", partial);
    assert(!partially_rejected);
    assert(updated["gravity"].as_number() == 3.0);
    assert(updated["masses"].as_array().size() == 2);

    middleware::JsonValue ik_request = middleware::JsonValue::make_object();
    ik_request["x"] = 1.0;
    ik_request["y"] = 1.0;
    auto [ik_ok, ik_response] = middleware.call_service("/ik/solve", ik_request);
    assert(ik_ok && ik_response["positions"].as_array().size() == 2);
    std::cout << "Project 2 integration registration tests passed.\n";
}
