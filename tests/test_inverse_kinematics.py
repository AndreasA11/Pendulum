import math
from rosbridge_client import Rosbridge


def test_ik_two_link_roundtrip_solution():
    with Rosbridge() as ros:
        # Ensure default lengths [1.0, 0.8]
        ros.call_service("/arm_sim/set_params", {"lengths": [1.0, 0.8]})
        l1, l2 = 1.0, 0.8

        test_points = [
            (1.2, 0.5),
            (0.3, 1.4),
            (-0.8, 0.9),
            (0.0, -1.2),
            (-1.0, -0.6),
        ]
        for x, y in test_points:
            reply = ros.call_service("/ik/solve", {"x": x, "y": y})
            assert reply.get("result") is True, f"IK solve failed for valid reachable point ({x}, {y}): {reply}"
            positions = reply["values"]["positions"]
            assert len(positions) == 2, f"Expected 2 joint angles for 2-link arm, got {positions}"

            q1, q2 = positions[0], positions[1]
            # Forward kinematics round-trip
            calc_x = l1 * math.cos(q1) + l2 * math.cos(q1 + q2)
            calc_y = l1 * math.sin(q1) + l2 * math.sin(q1 + q2)

            dist_err = math.hypot(calc_x - x, calc_y - y)
            assert dist_err < 1e-6, (
                f"FK roundtrip error too large for target ({x}, {y}): FK=({calc_x}, {calc_y}), error={dist_err}"
            )


def test_ik_reachability_boundary_checks():
    with Rosbridge() as ros:
        ros.call_service("/arm_sim/set_params", {"lengths": [1.0, 0.8]})
        l1, l2 = 1.0, 0.8
        outer_reach = l1 + l2       # 1.8
        inner_reach = abs(l1 - l2)  # 0.2

        # 1. Exact outer boundary: must be REACHABLE
        r_outer_exact = ros.call_service("/ik/solve", {"x": outer_reach, "y": 0.0})
        assert r_outer_exact.get("result") is True, f"Exact outer boundary must be reachable: {r_outer_exact}"

        # 2. Outside outer reach: must be REJECTED
        r_outside = ros.call_service("/ik/solve", {"x": outer_reach + 0.1, "y": 0.0})
        assert r_outside.get("result") is False, f"Target outside reach must be rejected: {r_outside}"
        assert bool(r_outside.get("status")), "Status message required on rejection"

        # 3. Exact inner boundary: must be REACHABLE
        r_inner_exact = ros.call_service("/ik/solve", {"x": inner_reach, "y": 0.0})
        assert r_inner_exact.get("result") is True, f"Exact inner boundary must be reachable: {r_inner_exact}"

        # 4. Inside inner singularity: must be REJECTED
        r_inside = ros.call_service("/ik/solve", {"x": inner_reach - 0.1, "y": 0.0})
        assert r_inside.get("result") is False, f"Target inside inner boundary must be rejected: {r_inside}"

        # 5. Missing / non-finite coordinates
        r_missing = ros.call_service("/ik/solve", {"x": 1.0})
        assert r_missing.get("result") is False, f"Missing y coordinate must be rejected: {r_missing}"


def test_ik_dynamic_length_updates():
    with Rosbridge() as ros:
        # Initially lengths are [1.0, 0.8] -> reach is 1.8
        ros.call_service("/arm_sim/set_params", {"lengths": [1.0, 0.8]})
        # Target at (2.4, 0.0) is unreachable
        r1 = ros.call_service("/ik/solve", {"x": 2.4, "y": 0.0})
        assert r1.get("result") is False, f"Target (2.4, 0) should be unreachable with length sum 1.8: {r1}"

        # Dynamically increase lengths to [1.5, 1.5] -> reach is 3.0
        update = ros.call_service("/arm_sim/set_params", {"lengths": [1.5, 1.5]})
        assert update.get("result") is True, f"Setting params failed: {update}"

        # Now (2.4, 0.0) must be reachable!
        r2 = ros.call_service("/ik/solve", {"x": 2.4, "y": 0.0})
        assert r2.get("result") is True, f"IK must adapt dynamically to updated arm lengths: {r2}"
        q1, q2 = r2["values"]["positions"]
        calc_x = 1.5 * math.cos(q1) + 1.5 * math.cos(q1 + q2)
        calc_y = 1.5 * math.sin(q1) + 1.5 * math.sin(q1 + q2)
        assert math.hypot(calc_x - 2.4, calc_y) < 1e-6, "FK of dynamic solution did not reach target"

        # Restore default lengths
        ros.call_service("/arm_sim/set_params", {"lengths": [1.0, 0.8]})


def test_ik_three_link_orientation_decoupling():
    # Connect to a 3-link arm runtime
    with Rosbridge(links=3, port=9096) as ros:
        params_reply = ros.call_service("/arm_sim/set_params", {"lengths": [1.0, 0.8, 0.6]})
        assert params_reply.get("result") is True, f"Setting 3-link params failed: {params_reply}"
        l1, l2, l3 = 1.0, 0.8, 0.6

        # Solve for reachable target with orientation
        target_x, target_y, target_phi = 1.4, 0.6, 0.5
        reply = ros.call_service("/ik/solve", {"x": target_x, "y": target_y, "phi": target_phi})
        assert reply.get("result") is True, f"3-link IK solve failed: {reply}"
        positions = reply["values"]["positions"]
        assert len(positions) == 3, f"Expected 3 joint positions, got {positions}"

        # Verify round-trip forward kinematics
        phi1 = positions[0]
        phi2 = phi1 + positions[1]
        phi3 = phi2 + positions[2]
        calc_x = l1 * math.cos(phi1) + l2 * math.cos(phi2) + l3 * math.cos(phi3)
        calc_y = l1 * math.sin(phi1) + l2 * math.sin(phi2) + l3 * math.sin(phi3)

        assert math.hypot(calc_x - target_x, calc_y - target_y) < 1e-6, (
            f"3-link FK position mismatch: calc=({calc_x}, {calc_y}), target=({target_x}, {target_y})"
        )
        # Orientation phi match (modulo 2pi)
        phi_diff = (phi3 - target_phi + math.pi) % (2.0 * math.pi) - math.pi
        assert abs(phi_diff) < 1e-6, f"3-link FK orientation mismatch: calc={phi3}, target={target_phi}"

        # Unreachable orientation: wrist point outside first 2 links reach
        # Target at (1.5, 0.0), but phi=0 makes wrist_x = 1.5 - 0.6 = 0.9 (reachable).
        # Target at (2.3, 0.0) with phi=pi: wrist_x = 2.3 - 0.6*cos(pi) = 2.9 (outside l1+l2=1.8)!
        r_unreachable_wrist = ros.call_service("/ik/solve", {"x": 2.3, "y": 0.0, "phi": math.pi})
        assert r_unreachable_wrist.get("result") is False, (
            f"Expected rejection when wrist is outside reach: {r_unreachable_wrist}"
        )
