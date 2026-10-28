import time
from rosbridge_client import Rosbridge


def test_ik_action_goal_preemption():
    with Rosbridge() as ros:
        ros.call_service("/arm_sim/reset", {})
        ros.call_service("/pid_controller/enable", {"data": True})
        ros.subscribe("/ik_action/result", "actionlib_msgs/GoalStatus")

        # 1. Send first reachable goal
        g1 = ros.call_service("/ik_action/send_goal", {
            "x": 1.2, "y": 0.5, "epsilon": 0.05, "success_hold": 1.0
        })
        assert g1.get("result") is True, f"Send goal 1 failed: {g1}"
        g1_id = g1["values"]["goal_id"]

        time.sleep(0.05)

        # 2. Send second reachable goal -> must preempt goal 1
        g2 = ros.call_service("/ik_action/send_goal", {
            "x": 0.5, "y": 1.2, "epsilon": 0.05, "success_hold": 1.0
        })
        assert g2.get("result") is True, f"Send goal 2 failed: {g2}"
        g2_id = g2["values"]["goal_id"]

        # Goal 1 should be reported as preempted
        res1 = ros.wait_for(
            lambda m: m.get("op") == "publish" and m.get("topic") == "/ik_action/result" and m.get("msg", {}).get("goal_id") == g1_id,
            timeout=3.0
        )["msg"]
        assert res1["outcome"] == "preempted", f"Goal 1 outcome should be 'preempted', got {res1}"

        # 3. Cancel goal 2
        cancel = ros.call_service("/ik_action/cancel_goal", {"goal_id": g2_id})
        assert cancel.get("result") is True, f"Cancel goal 2 failed: {cancel}"
        res2 = ros.wait_for(
            lambda m: m.get("op") == "publish" and m.get("topic") == "/ik_action/result" and m.get("msg", {}).get("goal_id") == g2_id,
            timeout=3.0
        )["msg"]
        assert res2["outcome"] == "preempted", f"Goal 2 outcome should be 'preempted' upon cancel, got {res2}"


def test_ik_action_unreachable_rejection():
    with Rosbridge() as ros:
        # Send active goal
        g_active = ros.call_service("/ik_action/send_goal", {
            "x": 1.0, "y": 0.5, "epsilon": 0.05, "success_hold": 0.5
        })
        assert g_active.get("result") is True
        active_id = g_active["values"]["goal_id"]

        # Send unreachable goal
        g_unreachable = ros.call_service("/ik_action/send_goal", {"x": 5.0, "y": 5.0})
        assert g_unreachable.get("result") is False, f"Unreachable goal must be rejected: {g_unreachable}"
        assert bool(g_unreachable.get("status")), "Status message required for rejected goal"

        # Canceling with mismatched goal_id must be rejected
        c_bad = ros.call_service("/ik_action/cancel_goal", {"goal_id": "nonexistent_id"})
        assert c_bad.get("result") is False, f"Cancel with wrong goal_id should fail: {c_bad}"

        # Cancel active goal cleanly
        ros.call_service("/ik_action/cancel_goal", {"goal_id": active_id})


def test_ik_action_dwell_tolerance_hold():
    with Rosbridge() as ros:
        # Fast setup with zero hold: satisfied immediately when inside epsilon
        ros.call_service("/arm_sim/reset", {})
        ros.call_service("/pid_controller/enable", {"data": True})
        ros.call_service("/pid_controller/set_gains", {
            "kp": [300.0, 300.0], "ki": [50.0, 50.0], "kd": [30.0, 30.0]
        })
        ros.subscribe("/ik_action/result", "actionlib_msgs/GoalStatus")

        # Goal close to home with success_hold = 0.0
        g_fast = ros.call_service("/ik_action/send_goal", {
            "x": 1.7, "y": 0.0, "epsilon": 0.2, "success_hold": 0.0
        })
        assert g_fast.get("result") is True
        fast_id = g_fast["values"]["goal_id"]

        # Should reach goal rapidly with outcome 'reached'
        res = ros.wait_for(
            lambda m: m.get("op") == "publish" and m.get("topic") == "/ik_action/result" and m.get("msg", {}).get("goal_id") == fast_id,
            timeout=4.0
        )["msg"]
        assert res["outcome"] == "reached", f"Expected outcome 'reached' for hold=0, got {res}"
        assert res["final_distance"] <= 0.2, f"final_distance should be <= epsilon (0.2): {res}"


def test_ik_trial_start_skip_stop_lifecycle():
    with Rosbridge() as ros:
        ros.call_service("/arm_sim/reset", {})
        ros.subscribe("/ik_trial/status", "std_msgs/String")

        # 1. Start trial
        start_rep = ros.call_service("/ik_trial/start", {
            "duration": 20.0, "epsilon": 0.05, "success_hold": 0.2
        })
        assert start_rep.get("result") is True, f"Start trial failed: {start_rep}"

        # Observe status publication
        status_msg = ros.next_message("/ik_trial/status", timeout=3.0)
        assert status_msg.get("running") is True, f"Trial should be running: {status_msg}"
        assert status_msg.get("targets_reached") == 0, f"Targets reached should start at 0: {status_msg}"
        assert status_msg.get("target") is not None, f"Target should be active: {status_msg}"

        # 2. Skip active target
        skip_rep = ros.call_service("/ik_trial/skip", {})
        assert skip_rep.get("result") is True, f"Skip target failed: {skip_rep}"

        # 3. Stop trial
        stop_rep = ros.call_service("/ik_trial/stop", {})
        assert stop_rep.get("result") is True, f"Stop trial failed: {stop_rep}"

        # Cleared status: running false, target null, error null, action_status idle
        stop_status = ros.wait_for(
            lambda m: m.get("op") == "publish" and m.get("topic") == "/ik_trial/status" and m.get("msg", {}).get("running") is False,
            timeout=3.0
        )["msg"]
        assert stop_status.get("target") is None, f"Stopped trial must clear target to null: {stop_status}"
        assert stop_status.get("error") is None, f"Stopped trial must clear error to null: {stop_status}"
        assert stop_status.get("action_status") == "idle", f"Stopped trial status must be 'idle': {stop_status}"

        # 4. Calling skip or stop when stopped must be rejected
        assert ros.call_service("/ik_trial/skip", {}).get("result") is False
        assert ros.call_service("/ik_trial/stop", {}).get("result") is False
