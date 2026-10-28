import time
from rosbridge_client import Rosbridge


def test_sim_pause_reset_and_trajectory():
    with Rosbridge() as ros:
        # 1. Pause and Resume Semantics
        pause_rep = ros.call_service("/arm_sim/pause", {"data": True})
        assert pause_rep.get("result") is True, f"Pause call failed: {pause_rep}"

        ros.subscribe("/joint_states", "sensor_msgs/JointState")
        msg1 = ros.latest_message("/joint_states", timeout=2.0)
        t1_sec = msg1["header"]["stamp"]["sec"]
        t1_nano = msg1["header"]["stamp"]["nanosec"]
        p1 = list(msg1["position"])

        time.sleep(0.3)
        msg2 = ros.latest_message("/joint_states", timeout=2.0)
        t2_sec = msg2["header"]["stamp"]["sec"]
        t2_nano = msg2["header"]["stamp"]["nanosec"]
        p2 = list(msg2["position"])

        assert t1_sec == t2_sec and abs(t1_nano - t2_nano) < 1e-5, "Simulation clock must freeze while paused"
        for val1, val2 in zip(p1, p2):
            assert abs(val1 - val2) < 1e-9, f"Arm state must be frozen while paused: {p1} vs {p2}"

        resume_rep = ros.call_service("/arm_sim/pause", {"data": False})
        assert resume_rep.get("result") is True, f"Resume call failed: {resume_rep}"

        time.sleep(0.2)
        msg3 = ros.latest_message("/joint_states", timeout=2.0)
        t3_sec = msg3["header"]["stamp"]["sec"]
        t3_nano = msg3["header"]["stamp"]["nanosec"]
        assert (t3_sec > t2_sec) or (t3_nano > t2_nano), "Simulation clock did not advance after unpause"

        # 2. Reset Semantics
        reset_rep = ros.call_service("/arm_sim/reset", {})
        assert reset_rep.get("result") is True, f"Reset call failed: {reset_rep}"
        assert all(abs(p) < 1e-9 for p in reset_rep["values"]["position"]), "Reset position not zero"
        assert all(abs(v) < 1e-9 for v in reset_rep["values"]["velocity"]), "Reset velocity not zero"

        # 3. Step-setpoint: multi-point trajectory holds only the LAST point
        ros.call_service("/pid_controller/enable", {"data": True})
        ros.call_service("/pid_controller/set_gains", {
            "kp": [100.0, 100.0], "ki": [0.0, 0.0], "kd": [10.0, 10.0]
        })
        ros.advertise("/joint_trajectory", "trajectory_msgs/JointTrajectory")

        ros.publish("/joint_trajectory", {
            "points": [
                {"positions": [0.2, 0.2], "velocities": [0.0, 0.0]},
                {"positions": [0.8, -0.4], "velocities": [0.0, 0.0]}
            ]
        })

        # Wait for PID effort to reflect the final setpoint [0.8, -0.4] (error = 0.8 -> effort > 10)
        msg_last = ros.wait_for(
            lambda m: m.get("op") == "publish" and m.get("topic") == "/joint_states" and m.get("msg", {}).get("effort", [0])[0] > 10.0,
            timeout=2.0
        )["msg"]
        assert msg_last["effort"][0] > 10.0, (
            f"Trajectory follower did not use last point as step setpoint: effort={msg_last['effort']}"
        )
