#include <cassert>
#include <cmath>
#include <iostream>
#include <string>
#include <vector>

#include "middleware/middleware.hpp"
#include "pendularm/arm_simulation.hpp"

namespace {

void verify_trajectory_schema(const middleware::JsonValue& traj, size_t expected_joints) {
    assert(traj.is_object());
    assert(traj.contains("header") && traj["header"].is_object());
    const auto& header = traj["header"];
    assert(header.contains("stamp") && header["stamp"].is_object());
    assert(header["stamp"].contains("sec") && header["stamp"]["sec"].is_number());
    assert(header["stamp"].contains("nanosec") && header["stamp"]["nanosec"].is_number());
    assert(header.contains("frame_id") && header["frame_id"].is_string());

    assert(traj.contains("joint_names") && traj["joint_names"].is_array());
    const auto& names = traj["joint_names"].as_array();
    assert(names.size() == expected_joints);
    for (size_t i = 0; i < expected_joints; ++i) {
        assert(names[i].as_string() == "joint" + std::to_string(i + 1));
    }

    assert(traj.contains("points") && traj["points"].is_array());
    const auto& points = traj["points"].as_array();
    assert(!points.empty());
    const auto& pt = points.back();
    assert(pt.contains("positions") && pt["positions"].is_array());
    assert(pt["positions"].as_array().size() == expected_joints);
    assert(pt.contains("velocities") && pt["velocities"].is_array());
    assert(pt["velocities"].as_array().size() == expected_joints);
    assert(pt.contains("accelerations") && pt["accelerations"].is_array());
    assert(pt.contains("time_from_start") && pt["time_from_start"].is_object());
    assert(pt["time_from_start"].contains("sec") && pt["time_from_start"]["sec"].is_number());
    assert(pt["time_from_start"].contains("nanosec") && pt["time_from_start"]["nanosec"].is_number());
}

} // namespace

int main() {
    std::cout << "Running comprehensive IK Action tests...\n";

    // ----------------------------------------------------
    // Section 1: 2-Link Arm IK Action Contract
    // ----------------------------------------------------
    {
        middleware::Middleware middleware;
        pendularm::ArmSimulation simulation(middleware, 2);
        std::vector<middleware::JsonValue> results;
        std::vector<middleware::JsonValue> trajectories;
        middleware.subscribe("/ik_action/result", [&results](const middleware::JsonValue& msg) { results.push_back(msg); });
        middleware.subscribe("/joint_trajectory", [&trajectories](const middleware::JsonValue& msg) { trajectories.push_back(msg); });

        // 1.1: Cancel when no goal is active must fail
        auto [idle_cancel_ok, idle_cancel_val] = middleware.call_service("/ik_action/cancel_goal", middleware::JsonValue::make_object());
        (void)idle_cancel_val;
        assert(!idle_cancel_ok);

        // 1.2: Send reachable goal with null optional fields (testing null tolerance)
        middleware::JsonValue null_req = middleware::JsonValue::make_object();
        null_req["x"] = 1.0;
        null_req["y"] = 1.0;
        null_req["phi"] = middleware::JsonValue(); // null
        null_req["epsilon"] = middleware::JsonValue(); // null
        null_req["success_hold"] = middleware::JsonValue(); // null
        auto [first_ok, first] = middleware.call_service("/ik_action/send_goal", null_req);
        assert(first_ok);
        const std::string first_id = first["goal_id"].as_string();
        assert(!first_id.empty());
        assert(trajectories.size() == 1);
        verify_trajectory_schema(trajectories[0], 2);

        // 1.3: Unreachable goal must be rejected without affecting active goal
        middleware::JsonValue unreachable = middleware::JsonValue::make_object();
        unreachable["x"] = 10.0;
        unreachable["y"] = 0.0;
        auto [unreach_ok, unreach_val] = middleware.call_service("/ik_action/send_goal", unreachable);
        (void)unreach_val;
        assert(!unreach_ok);
        assert(results.empty()); // No result published for rejected goal, and active goal not preempted

        // 1.4: Cancel with mismatched goal_id must fail without affecting active goal
        middleware::JsonValue wrong_cancel = middleware::JsonValue::make_object();
        wrong_cancel["goal_id"] = "wrong-id-99999";
        auto [wrong_cancel_ok, wrong_val] = middleware.call_service("/ik_action/cancel_goal", wrong_cancel);
        (void)wrong_val;
        assert(!wrong_cancel_ok);
        assert(results.empty());

        // 1.5: Preemption by a second reachable goal
        middleware::JsonValue replacement = middleware::JsonValue::make_object();
        replacement["x"] = 0.0;
        replacement["y"] = 2.0;
        auto [replacement_ok, replacement_val] = middleware.call_service("/ik_action/send_goal", replacement);
        assert(replacement_ok);
        const std::string replacement_id = replacement_val["goal_id"].as_string();
        assert(replacement_id != first_id);
        assert(trajectories.size() == 2);
        verify_trajectory_schema(trajectories[1], 2);

        // The first goal must have been preempted
        assert(results.size() == 1);
        assert(results[0]["goal_id"].as_string() == first_id);
        assert(results[0]["outcome"].as_string() == "preempted");
        assert(results[0].contains("final_distance") && results[0]["final_distance"].is_number());
        assert(results[0].contains("target") && results[0]["target"].is_object());
        assert(results[0]["target"].contains("x") && results[0]["target"].contains("y"));
        assert(!results[0]["target"].contains("phi")); // 2-link target should NOT contain phi

        // 1.6: Cancel with {"goal_id": null} should cancel the active goal
        middleware::JsonValue null_id_cancel = middleware::JsonValue::make_object();
        null_id_cancel["goal_id"] = middleware::JsonValue();
        auto [null_cancel_ok, null_cancel_val] = middleware.call_service("/ik_action/cancel_goal", null_id_cancel);
        assert(null_cancel_ok);
        assert(results.size() == 2);
        assert(results[1]["goal_id"].as_string() == replacement_id);
        assert(results[1]["outcome"].as_string() == "preempted");

        // 1.7: Cancel with empty object {}
        // Send a third goal
        middleware::JsonValue third_req = middleware::JsonValue::make_object();
        third_req["x"] = 1.4;
        third_req["y"] = 0.0;
        auto [third_ok, third_val] = middleware.call_service("/ik_action/send_goal", third_req);
        assert(third_ok);
        const std::string third_id = third_val["goal_id"].as_string();

        auto [empty_cancel_ok, empty_cancel_val] = middleware.call_service("/ik_action/cancel_goal", middleware::JsonValue::make_object());
        assert(empty_cancel_ok);
        assert(results.size() == 3);
        assert(results[2]["goal_id"].as_string() == third_id);
        assert(results[2]["outcome"].as_string() == "preempted");

        // 1.8: Goal with invalid parameter types rejected
        middleware::JsonValue bad_x = middleware::JsonValue::make_object();
        bad_x["x"] = "not_a_number";
        bad_x["y"] = 1.0;
        assert(!middleware.call_service("/ik_action/send_goal", bad_x).first);

        middleware::JsonValue bad_eps = middleware::JsonValue::make_object();
        bad_eps["x"] = 1.0;
        bad_eps["y"] = 1.0;
        bad_eps["epsilon"] = -0.5; // Must be positive
        assert(!middleware.call_service("/ik_action/send_goal", bad_eps).first);

        middleware::JsonValue bad_hold = middleware::JsonValue::make_object();
        bad_hold["x"] = 1.0;
        bad_hold["y"] = 1.0;
        bad_hold["success_hold"] = -1.0; // Must be non-negative
        assert(!middleware.call_service("/ik_action/send_goal", bad_hold).first);
    }

    // ----------------------------------------------------
    // Section 2: 3-Link Arm IK Action Contract & Phi Handling
    // ----------------------------------------------------
    {
        middleware::Middleware middleware;
        pendularm::ArmSimulation simulation(middleware, 3);
        std::vector<middleware::JsonValue> results;
        std::vector<middleware::JsonValue> trajectories;
        middleware.subscribe("/ik_action/result", [&results](const middleware::JsonValue& msg) { results.push_back(msg); });
        middleware.subscribe("/joint_trajectory", [&trajectories](const middleware::JsonValue& msg) { trajectories.push_back(msg); });

        // 2.1: Reachable goal with phi
        middleware::JsonValue goal_with_phi = middleware::JsonValue::make_object();
        goal_with_phi["x"] = 2.0;
        goal_with_phi["y"] = 0.5;
        goal_with_phi["phi"] = 0.2;
        auto [ok_phi, res_phi] = middleware.call_service("/ik_action/send_goal", goal_with_phi);
        assert(ok_phi);
        assert(trajectories.size() == 1);
        verify_trajectory_schema(trajectories[0], 3);

        // Cancel and check result target schema for 3-link arm
        assert(middleware.call_service("/ik_action/cancel_goal", middleware::JsonValue::make_object()).first);
        assert(results.size() == 1);
        assert(results[0]["target"].contains("x"));
        assert(results[0]["target"].contains("y"));
        assert(results[0]["target"].contains("phi")); // 3-link target MUST contain phi
        assert(std::abs(results[0]["target"]["phi"].as_number() - 0.2) < 1e-6);

        // 2.2: Reachable goal without phi (should auto-compute phi)
        middleware::JsonValue goal_no_phi = middleware::JsonValue::make_object();
        goal_no_phi["x"] = 2.0;
        goal_no_phi["y"] = 1.0;
        auto [ok_nophi, res_nophi] = middleware.call_service("/ik_action/send_goal", goal_no_phi);
        assert(ok_nophi);
        assert(trajectories.size() == 2);
        verify_trajectory_schema(trajectories[1], 3);

        assert(middleware.call_service("/ik_action/cancel_goal", middleware::JsonValue::make_object()).first);
        assert(results.size() == 2);
        assert(results[1]["target"].contains("phi")); // 3-link arm result still carries phi
    }

    std::cout << "All IK Action comprehensive tests PASSED successfully!\n";
    return 0;
}
