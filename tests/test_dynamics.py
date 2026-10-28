import math
import time
from rosbridge_client import Rosbridge


def test_dynamics_free_swing_under_gravity():
    with Rosbridge() as ros:
        # Reset plant and ensure PID is disabled
        ros.call_service("/pid_controller/enable", {"data": False})
        reset_reply = ros.call_service("/arm_sim/reset", {})
        assert reset_reply.get("result") is True, f"Reset failed: {reset_reply}"

        # Ensure gravity is enabled
        ros.call_service("/arm_sim/set_params", {"gravity": 9.81})

        ros.subscribe("/joint_states", "sensor_msgs/JointState")
        initial_msg = ros.next_message("/joint_states", timeout=2.0)
        q0 = list(initial_msg["position"])

        # With PID disabled, effort must be zero on all joints
        efforts = initial_msg.get("effort", [])
        for tau in efforts:
            assert abs(tau) < 1e-9, f"Effort must be zero while PID is disabled, got {efforts}"

        # Wait for physics advancement (0.4s real time)
        time.sleep(0.4)
        later_msg = ros.next_message("/joint_states", timeout=2.0)
        q_later = list(later_msg["position"])
        v_later = list(later_msg["velocity"])

        # Arm must accelerate and move under gravity from non-hanging equilibrium
        displacements = [abs(ql - q) for ql, q in zip(q_later, q0)]
        velocities = [abs(v) for v in v_later]
        assert any(d > 0.005 for d in displacements) or any(v > 0.01 for v in velocities), (
            f"Arm did not move under gravity with PID disabled: q0={q0}, q_later={q_later}, v={v_later}"
        )


def test_dynamics_holding_torque_matches_gravity():
    with Rosbridge() as ros:
        # Reset plant and controller to a clean starting state
        ros.call_service("/arm_sim/reset", {})

        # Set physical parameters
        param_reply = ros.call_service("/arm_sim/set_params", {
            "gravity": 9.81, "masses": [1.5, 1.2], "lengths": [1.0, 0.8]
        })
        assert param_reply.get("result") is True, f"Failed setting params: {param_reply}"
        p = param_reply["values"]
        g = p["gravity"]
        l1, l2 = p["lengths"][0], p["lengths"][1]
        m1, m2 = p["masses"][0], p["masses"][1]

        # Enable PID with stable gains to hold arm against gravity
        ros.call_service("/pid_controller/enable", {"data": True})
        ros.call_service("/pid_controller/set_gains", {
            "kp": [80.0, 80.0], "ki": [20.0, 20.0], "kd": [15.0, 15.0]
        })

        # Target setpoint at horizontal position [0.0, 0.0] where gravity torque is maximum
        ros.advertise("/joint_trajectory", "trajectory_msgs/JointTrajectory")
        ros.publish("/joint_trajectory", {
            "points": [{"positions": [0.0, 0.0], "velocities": [0.0, 0.0]}]
        })

        # Subscribe and wait for PID integral to build up holding effort against gravity
        ros.subscribe("/joint_states", "sensor_msgs/JointState")
        time.sleep(1.3)
        msg = ros.latest_message("/joint_states", timeout=2.0)

        assert msg is not None, "Failed to receive /joint_states message"
        q = msg["position"]
        actual_effort = msg["effort"]

        # Theoretical gravity load G(q) for uniform rods:
        # phi1 = q1, phi2 = q1 + q2
        phi1 = q[0]
        phi2 = q[0] + q[1]
        expected_G1 = g * l1 * (0.5 * m1 + m2) * math.cos(phi1) + g * l2 * (0.5 * m2) * math.cos(phi2)
        expected_G2 = g * l2 * (0.5 * m2) * math.cos(phi2)

        # Holding effort must counteract downward gravity in the positive direction
        assert actual_effort[0] > 10.0, f"Expected positive holding effort against gravity, got {actual_effort[0]}"
        assert abs(actual_effort[0] - expected_G1) < 5.0, (
            f"Steady state effort[0] did not balance gravity: effort={actual_effort[0]}, G1={expected_G1}"
        )
        assert abs(actual_effort[1] - expected_G2) < 4.0, (
            f"Steady state effort[1] did not balance gravity: effort={actual_effort[1]}, G2={expected_G2}"
        )


def test_dynamics_parameter_validation_and_scaling():
    with Rosbridge() as ros:
        # Empty request queries current parameters
        query = ros.call_service("/arm_sim/set_params", {})
        assert query.get("result") is True, f"Query params failed: {query}"
        assert "gravity" in query["values"] and "masses" in query["values"] and "lengths" in query["values"]

        # Negative gravity rejection
        r_neg_g = ros.call_service("/arm_sim/set_params", {"gravity": -5.0})
        assert r_neg_g.get("result") is False, f"Expected rejection for negative gravity: {r_neg_g}"

        # Non-positive mass / length rejection
        r_zero_m = ros.call_service("/arm_sim/set_params", {"masses": [0.0, 1.0]})
        assert r_zero_m.get("result") is False, f"Expected rejection for zero mass: {r_zero_m}"

        r_neg_l = ros.call_service("/arm_sim/set_params", {"lengths": [1.0, -0.5]})
        assert r_neg_l.get("result") is False, f"Expected rejection for negative length: {r_neg_l}"

        # Wrong array size rejection (for 2-link arm)
        r_size = ros.call_service("/arm_sim/set_params", {"lengths": [1.0, 1.0, 1.0]})
        assert r_size.get("result") is False, f"Expected rejection for wrong link count array: {r_size}"


def test_dynamics_coriolis_coupling():
    with Rosbridge() as ros:
        # Enable PID controller and reset arm
        ros.call_service("/pid_controller/enable", {"data": True})
        ros.call_service("/arm_sim/reset", {})
        ros.call_service("/pid_controller/set_gains", {
            "kp": [150.0, 150.0], "ki": [0.0, 0.0], "kd": [20.0, 20.0]
        })

        # Rapid command on joint 1 while joint 2 setpoint remains at 0
        ros.publish("/joint_trajectory", {
            "points": [{"positions": [1.5, 0.0], "velocities": [0.0, 0.0]}]
        })

        ros.subscribe("/joint_states", "sensor_msgs/JointState")
        # Observe that moving joint 1 induces non-zero dynamic coupling forces
        induced_coupling_observed = False
        for _ in range(15):
            msg = ros.next_message("/joint_states", timeout=1.0)
            # Velocity on joint 1 should induce coupling torque or velocity deviation on joint 2
            if abs(msg["velocity"][0]) > 0.5 and (abs(msg["effort"][1]) > 0.1 or abs(msg["velocity"][1]) > 0.05):
                induced_coupling_observed = True
                break
            time.sleep(0.04)

        assert induced_coupling_observed, "Motion of joint 1 did not couple to joint 2 (Coriolis/centrifugal)"
