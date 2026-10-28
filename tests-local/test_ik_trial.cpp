#include <cassert>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <iostream>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "middleware/middleware.hpp"
#include "pendularm/arm_simulation.hpp"

namespace {

class StatusCollector {
public:
    void receive(const middleware::JsonValue& msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        messages_.push_back(msg);
        cv_.notify_all();
    }

    middleware::JsonValue wait_for_latest(size_t min_count = 1) {
        std::unique_lock<std::mutex> lock(mutex_);
        assert(cv_.wait_for(lock, std::chrono::seconds(2), [&] {
            return messages_.size() >= min_count;
        }));
        return messages_.back();
    }

    void clear() {
        std::lock_guard<std::mutex> lock(mutex_);
        messages_.clear();
    }

private:
    std::mutex mutex_;
    std::condition_variable cv_;
    std::vector<middleware::JsonValue> messages_;
};

void verify_active_status(const middleware::JsonValue& status, size_t expected_joints, double expected_duration) {
    assert(status.is_object());
    assert(status.contains("running") && status["running"].as_bool() == true);
    assert(status.contains("elapsed") && status["elapsed"].is_number() && status["elapsed"].as_number() >= 0.0);
    assert(status.contains("duration") && status["duration"].is_number());
    assert(std::abs(status["duration"].as_number() - expected_duration) < 1e-6);
    assert(status.contains("targets_reached") && status["targets_reached"].is_number());
    assert(status.contains("action_status") && status["action_status"].as_string() == "active");

    assert(status.contains("target") && status["target"].is_object());
    assert(status["target"].contains("x") && status["target"]["x"].is_number());
    assert(status["target"].contains("y") && status["target"]["y"].is_number());
    if (expected_joints == 3) {
        assert(status["target"].contains("phi") && status["target"]["phi"].is_number());
    } else {
        assert(!status["target"].contains("phi"));
    }

    // Critical: error and desired_positions must NOT be null while active!
    assert(status.contains("error") && !status["error"].is_null() && status["error"].is_number());
    assert(status.contains("desired_positions") && !status["desired_positions"].is_null() && status["desired_positions"].is_array());
    assert(status["desired_positions"].as_array().size() == expected_joints);
}

void verify_idle_status(const middleware::JsonValue& status) {
    assert(status.is_object());
    assert(status.contains("running") && status["running"].as_bool() == false);
    assert(status.contains("action_status") && status["action_status"].as_string() == "idle");
    assert(status.contains("target") && status["target"].is_null());
    assert(status.contains("error") && status["error"].is_null());
    assert(status.contains("desired_positions") && status["desired_positions"].is_null());
}

} // namespace

int main() {
    std::cout << "Running comprehensive IK Trial tests...\n";

    // ----------------------------------------------------
    // Section 1: 2-Link Arm IK Trial Lifecycle & Contract
    // ----------------------------------------------------
    {
        middleware::Middleware middleware;
        pendularm::ArmSimulation simulation(middleware, 2);
        StatusCollector status_collector;
        middleware.subscribe("/ik_trial/status", [&status_collector](const middleware::JsonValue& msg) {
            status_collector.receive(msg);
        });

        // 1.1: Skip and Stop must fail when no trial is running
        auto [skip_idle_ok, skip_idle_val] = middleware.call_service("/ik_trial/skip", middleware::JsonValue::make_object());
        (void)skip_idle_val;
        assert(!skip_idle_ok);

        auto [stop_idle_ok, stop_idle_val] = middleware.call_service("/ik_trial/stop", middleware::JsonValue::make_object());
        (void)stop_idle_val;
        assert(!stop_idle_ok);

        // 1.2: Start fresh trial with defaults ({})
        status_collector.clear();
        auto [start_ok, start_val] = middleware.call_service("/ik_trial/start", middleware::JsonValue::make_object());
        assert(start_ok);
        (void)start_val;

        // Verify status reflects active trial with default 20.0s duration
        simulation.start();
        auto active_st = status_collector.wait_for_latest(1);
        verify_active_status(active_st, 2, 20.0);
        assert(active_st["targets_reached"].as_number() == 0.0);

        // 1.3: Skip abandons target and submits a new target without incrementing count
        status_collector.clear();
        auto [skip_ok, skip_val] = middleware.call_service("/ik_trial/skip", middleware::JsonValue::make_object());
        assert(skip_ok);
        (void)skip_val;

        auto skipped_st = status_collector.wait_for_latest(1);
        verify_active_status(skipped_st, 2, 20.0);
        assert(skipped_st["targets_reached"].as_number() == 0.0); // Skip does not increment reached count

        // 1.4: Stop ends trial, clears target state, and freezes elapsed time
        status_collector.clear();
        auto [stop_ok, stop_val] = middleware.call_service("/ik_trial/stop", middleware::JsonValue::make_object());
        assert(stop_ok);
        (void)stop_val;

        auto stopped_st1 = status_collector.wait_for_latest(1);
        verify_idle_status(stopped_st1);
        const double stopped_elapsed1 = stopped_st1["elapsed"].as_number();

        // Wait a short time and verify elapsed time remains frozen
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
        status_collector.clear();
        auto stopped_st2 = status_collector.wait_for_latest(1);
        verify_idle_status(stopped_st2);
        const double stopped_elapsed2 = stopped_st2["elapsed"].as_number();
        assert(std::abs(stopped_elapsed1 - stopped_elapsed2) < 1e-4); // Elapsed must freeze when stopped!

        // Stop again must fail since trial is already stopped
        auto [stop_again_ok, stop_again_val] = middleware.call_service("/ik_trial/stop", middleware::JsonValue::make_object());
        (void)stop_again_val;
        assert(!stop_again_ok);

        simulation.stop();
    }

    // ----------------------------------------------------
    // Section 2: Partial Overrides, Null Toleration, & Validation
    // ----------------------------------------------------
    {
        middleware::Middleware middleware;
        pendularm::ArmSimulation simulation(middleware, 2);
        StatusCollector status_collector;
        middleware.subscribe("/ik_trial/status", [&status_collector](const middleware::JsonValue& msg) {
            status_collector.receive(msg);
        });

        // 2.1: Invalid parameters rejected
        middleware::JsonValue bad_dur = middleware::JsonValue::make_object();
        bad_dur["duration"] = -5.0; // Must be positive
        assert(!middleware.call_service("/ik_trial/start", bad_dur).first);

        middleware::JsonValue bad_eps = middleware::JsonValue::make_object();
        bad_eps["epsilon"] = 0.0; // Must be positive
        assert(!middleware.call_service("/ik_trial/start", bad_eps).first);

        middleware::JsonValue bad_hold = middleware::JsonValue::make_object();
        bad_hold["success_hold"] = -0.5; // Must be non-negative
        assert(!middleware.call_service("/ik_trial/start", bad_hold).first);

        // 2.2: Partial override with null values tolerated (e.g. {"duration": 12.0, "epsilon": null})
        middleware::JsonValue partial_req = middleware::JsonValue::make_object();
        partial_req["duration"] = 12.0;
        partial_req["epsilon"] = middleware::JsonValue(); // null
        partial_req["success_hold"] = middleware::JsonValue(); // null
        assert(middleware.call_service("/ik_trial/start", partial_req).first);

        simulation.start();
        auto st = status_collector.wait_for_latest(1);
        verify_active_status(st, 2, 12.0);

        assert(middleware.call_service("/ik_trial/stop", middleware::JsonValue::make_object()).first);
        simulation.stop();
    }

    // ----------------------------------------------------
    // Section 3: 3-Link Arm IK Trial Target Schema
    // ----------------------------------------------------
    {
        middleware::Middleware middleware;
        pendularm::ArmSimulation simulation(middleware, 3);
        StatusCollector status_collector;
        middleware.subscribe("/ik_trial/status", [&status_collector](const middleware::JsonValue& msg) {
            status_collector.receive(msg);
        });

        assert(middleware.call_service("/ik_trial/start", middleware::JsonValue::make_object()).first);
        simulation.start();

        auto st = status_collector.wait_for_latest(1);
        verify_active_status(st, 3, 20.0);
        // Verify 3-link arm target contains phi
        assert(st["target"].contains("phi") && st["target"]["phi"].is_number());
        assert(st["desired_positions"].as_array().size() == 3);

        assert(middleware.call_service("/ik_trial/stop", middleware::JsonValue::make_object()).first);
        simulation.stop();
    }

    std::cout << "All IK Trial comprehensive tests PASSED successfully!\n";
    return 0;
}
