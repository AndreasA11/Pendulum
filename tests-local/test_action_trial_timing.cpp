#include <cassert>
#include <chrono>
#include <condition_variable>
#include <iostream>
#include <mutex>
#include <thread>
#include <vector>

#include "middleware/middleware.hpp"
#include "pendularm/arm_simulation.hpp"

namespace {
class Messages {
public:
    void push(const middleware::JsonValue& value) {
        std::lock_guard<std::mutex> lock(mutex_);
        values_.push_back(value);
        changed_.notify_all();
    }
    middleware::JsonValue wait_for(size_t count) {
        std::unique_lock<std::mutex> lock(mutex_);
        assert(changed_.wait_for(lock, std::chrono::seconds(2), [&] { return values_.size() >= count; }));
        return values_.back();
    }
    size_t size() {
        std::lock_guard<std::mutex> lock(mutex_);
        return values_.size();
    }
private:
    std::mutex mutex_;
    std::condition_variable changed_;
    std::vector<middleware::JsonValue> values_;
};
} // namespace

int main() {
    middleware::Middleware middleware;
    pendularm::ArmSimulation simulation(middleware, 2);
    Messages results;
    Messages statuses;
    middleware.subscribe("/ik_action/result", [&results](const middleware::JsonValue& msg) { results.push(msg); });
    middleware.subscribe("/ik_trial/status", [&statuses](const middleware::JsonValue& msg) { statuses.push(msg); });
    simulation.start();

    middleware::JsonValue goal = middleware::JsonValue::make_object();
    goal["x"] = 2.0; goal["y"] = 0.0;
    goal["epsilon"] = 0.1; goal["success_hold"] = 0.02;
    assert(middleware.call_service("/ik_action/send_goal", goal).first);
    const auto result = results.wait_for(1);
    assert(result["outcome"].as_string() == "reached");
    std::this_thread::sleep_for(std::chrono::milliseconds(30));
    assert(results.size() == 1); // A concluded action must not publish another result.

    middleware::JsonValue start = middleware::JsonValue::make_object();
    start["duration"] = 1.0;
    assert(middleware.call_service("/ik_trial/start", start).first);
    const auto active = statuses.wait_for(1);
    assert(active["running"].as_bool());
    assert(!active["target"].is_null());
    assert(active["action_status"].as_string() == "active");
    assert(middleware.call_service("/ik_trial/stop", middleware::JsonValue::make_object()).first);
    const auto stopped = statuses.wait_for(2);
    assert(!stopped["running"].as_bool());
    assert(stopped["target"].is_null());
    assert(stopped["error"].is_null());
    assert(stopped["desired_positions"].is_null());
    assert(stopped["action_status"].as_string() == "idle");
    simulation.stop();
    std::cout << "Action dwell and trial-status tests passed.\n";
}
