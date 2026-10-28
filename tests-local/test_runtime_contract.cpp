#include <cassert>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <iostream>
#include <mutex>
#include <thread>
#include <vector>

#include "middleware/middleware.hpp"
#include "pendularm/arm_simulation.hpp"

namespace {
struct StateObserver {
    std::mutex mutex;
    std::condition_variable changed;
    std::vector<middleware::JsonValue> messages;

    void receive(const middleware::JsonValue& message) {
        std::lock_guard<std::mutex> lock(mutex);
        messages.push_back(message);
        changed.notify_all();
    }

    middleware::JsonValue wait_for(size_t count) {
        std::unique_lock<std::mutex> lock(mutex);
        assert(changed.wait_for(lock, std::chrono::seconds(2), [&] { return messages.size() >= count; }));
        return messages.back();
    }
};

middleware::JsonValue trajectory(const std::vector<double>& first, const std::vector<double>& last) {
    middleware::JsonValue message = middleware::JsonValue::make_object();
    middleware::JsonValue points = middleware::JsonValue::make_array();
    middleware::JsonValue first_point = middleware::JsonValue::make_object();
    first_point["positions"] = middleware::JsonValue::make_array();
    for (double value : first) first_point["positions"].push_back(value);
    middleware::JsonValue last_point = middleware::JsonValue::make_object();
    last_point["positions"] = middleware::JsonValue::make_array();
    for (double value : last) last_point["positions"].push_back(value);
    points.push_back(first_point);
    points.push_back(last_point);
    message["points"] = points;
    return message;
}
} // namespace

int main() {
    middleware::Middleware middleware;
    pendularm::ArmSimulation simulation(middleware, 2);
    StateObserver states;
    middleware.subscribe("/joint_states", [&states](const middleware::JsonValue& msg) { states.receive(msg); });
    simulation.start();

    const auto first = states.wait_for(2);
    assert(first["name"].as_array().size() == 2);
    assert(first["position"].as_array().size() == 2);
    assert(first["velocity"].as_array().size() == 2);
    assert(first["effort"].as_array().size() == 2);
    assert(std::abs(first["effort"].as_array()[0].as_number()) == 0.0);

    auto pause = middleware::JsonValue::make_object();
    pause["data"] = true;
    assert(middleware.call_service("/arm_sim/pause", pause).first);
    const auto paused_a = states.wait_for(4);
    const auto paused_b = states.wait_for(6);
    for (size_t i = 0; i < 2; ++i) {
        assert(paused_a["position"].as_array()[i].as_number() == paused_b["position"].as_array()[i].as_number());
        assert(paused_a["velocity"].as_array()[i].as_number() == paused_b["velocity"].as_array()[i].as_number());
    }

    middleware.publish("/joint_trajectory", trajectory({-1.0, -1.0}, {0.4, -0.3}));
    auto enable = middleware::JsonValue::make_object();
    enable["data"] = true;
    assert(middleware.call_service("/pid_controller/enable", enable).first);
    pause["data"] = false;
    assert(middleware.call_service("/arm_sim/pause", pause).first);
    const auto controlled = states.wait_for(9);
    assert(std::abs(controlled["effort"].as_array()[0].as_number()) > 1e-9);

    const auto reset = middleware::JsonValue::make_object();
    auto [reset_ok, reset_values] = middleware.call_service("/arm_sim/reset", reset);
    assert(reset_ok);
    for (const auto& value : reset_values["position"].as_array()) assert(value.as_number() == 0.0);
    for (const auto& value : reset_values["velocity"].as_array()) assert(value.as_number() == 0.0);
    simulation.stop();

    middleware::Middleware three_link_middleware;
    pendularm::ArmSimulation three_link_simulation(three_link_middleware, 3);
    auto [params_ok, params] = three_link_middleware.call_service("/arm_sim/set_params", reset);
    assert(params_ok && params["lengths"].as_array().size() == 3);
    middleware::JsonValue ik = middleware::JsonValue::make_object();
    ik["x"] = 2.0; ik["y"] = 1.0; ik["phi"] = 0.0;
    auto [ik_ok, ik_values] = three_link_middleware.call_service("/ik/solve", ik);
    assert(ik_ok && ik_values["positions"].as_array().size() == 3);
    std::cout << "Live runtime contract tests passed.\n";
}
