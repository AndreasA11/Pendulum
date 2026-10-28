from rosbridge_client import Rosbridge


def test_euler_step_calculation():
    with Rosbridge() as ros:
        reply = ros.call_service("/arm_sim/integration_step", {
            "function": "t", "x0": 1.0, "xdot0": 2.0, "dt": 0.5, "steps": 2, "integrator": "euler"
        })
    assert reply.get("result") is True, f"Service failed: {reply}"
    v = reply.get("values", {})
    times = v.get("times", [])
    positions = v.get("positions", [])
    velocities = v.get("velocities", [])

    assert len(times) == 3, f"Expected 3 time points (steps+1), got {len(times)}: {times}"
    assert len(positions) == 3, f"Expected 3 positions, got {len(positions)}: {positions}"
    assert len(velocities) == 3, f"Expected 3 velocities, got {len(velocities)}: {velocities}"

    # Verify initial conditions at index 0
    assert abs(times[0] - 0.0) < 1e-9, f"times[0] must be 0, got {times[0]}"
    assert abs(positions[0] - 1.0) < 1e-9, f"positions[0] must equal x0=1.0, got {positions[0]}"
    assert abs(velocities[0] - 2.0) < 1e-9, f"velocities[0] must equal xdot0=2.0, got {velocities[0]}"

    # Step 1: t=0.5 -> x1 = 1 + 2*0.5 = 2.0, v1 = 2 + 0*0.5 = 2.0
    assert abs(positions[1] - 2.0) < 1e-7, f"Euler step 1 position mismatch: {positions[1]} != 2.0"
    assert abs(velocities[1] - 2.0) < 1e-7, f"Euler step 1 velocity mismatch: {velocities[1]} != 2.0"

    # Step 2: t=1.0 -> x2 = 2.0 + 2.0*0.5 = 3.0, v2 = 2.0 + 0.5*0.5 = 2.25
    assert abs(positions[2] - 3.0) < 1e-7, f"Euler step 2 position mismatch: {positions[2]} != 3.0"
    assert abs(velocities[2] - 2.25) < 1e-7, f"Euler step 2 velocity mismatch: {velocities[2]} != 2.25"


def test_midpoint_stage_timing():
    with Rosbridge() as ros:
        reply = ros.call_service("/arm_sim/integration_step", {
            "function": "t", "x0": 0.0, "xdot0": 0.0, "dt": 1.0, "steps": 1, "integrator": "midpoint"
        })
    assert reply.get("result") is True, f"Service failed: {reply}"
    v = reply.get("values", {})
    # For qddot = t, a0 = 0. Midpoint at t=0.5 evaluates a_mid = 0.5 -> v1 = 0 + 0.5 * 1.0 = 0.5.
    # If t was frozen at t=0, a_mid would be 0 and v1 would be 0.
    v1 = v["velocities"][1]
    assert abs(v1 - 0.5) < 1e-7, f"Midpoint must evaluate acceleration at t + dt/2; expected v=0.5, got {v1}"


def test_verlet_corrector_step():
    with Rosbridge() as ros:
        reply = ros.call_service("/arm_sim/integration_step", {
            "function": "t", "x0": 0.0, "xdot0": 1.0, "dt": 0.5, "steps": 1, "integrator": "verlet"
        })
    assert reply.get("result") is True, f"Service failed: {reply}"
    v = reply.get("values", {})
    # Verlet: x1 = x0 + v0*dt + 0.5*a0*dt^2 = 0 + 1.0*0.5 + 0 = 0.5
    # a1 = f(0.5) = 0.5. v1 = v0 + 0.5*(a0 + a1)*dt = 1.0 + 0.5*(0 + 0.5)*0.5 = 1.125
    x1 = v["positions"][1]
    v1 = v["velocities"][1]
    assert abs(x1 - 0.5) < 1e-7, f"Verlet predicted position incorrect: {x1} != 0.5"
    assert abs(v1 - 1.125) < 1e-7, f"Verlet corrector velocity incorrect: {v1} != 1.125"


def test_rk4_accuracy_and_convergence():
    with Rosbridge() as ros:
        rk4_reply = ros.call_service("/arm_sim/integration_step", {
            "function": "t^3", "x0": 0.0, "xdot0": 0.0, "dt": 0.1, "steps": 10, "integrator": "rk4"
        })
        euler_reply = ros.call_service("/arm_sim/integration_step", {
            "function": "t^3", "x0": 0.0, "xdot0": 0.0, "dt": 0.1, "steps": 10, "integrator": "euler"
        })
    assert rk4_reply.get("result") is True, f"RK4 failed: {rk4_reply}"
    assert euler_reply.get("result") is True, f"Euler failed: {euler_reply}"

    # Analytical solution for qddot = t^3 from rest:
    # v(t) = t^4 / 4, at t=1.0 -> v=0.25
    # x(t) = t^5 / 20, at t=1.0 -> x=0.05
    rk4_x = rk4_reply["values"]["positions"][-1]
    rk4_v = rk4_reply["values"]["velocities"][-1]
    euler_x = euler_reply["values"]["positions"][-1]

    rk4_err_x = abs(rk4_x - 0.05)
    rk4_err_v = abs(rk4_v - 0.25)
    euler_err_x = abs(euler_x - 0.05)

    assert rk4_err_x < 1e-5, f"RK4 4th-order position error too large: {rk4_err_x} (x={rk4_x}, expected 0.05)"
    assert rk4_err_v < 1e-5, f"RK4 4th-order velocity error too large: {rk4_err_v} (v={rk4_v}, expected 0.25)"
    assert euler_err_x > 1e-2, f"Euler unexpectedly matched high order: error={euler_err_x}"


def test_integrator_service_rejections():
    with Rosbridge() as ros:
        # 1. Negative / zero dt
        r1 = ros.call_service("/arm_sim/integration_step", {
            "function": "t", "x0": 0.0, "xdot0": 0.0, "dt": 0.0, "steps": 1, "integrator": "rk4"
        })
        assert r1.get("result") is False, f"Expected rejection for dt=0: {r1}"

        r2 = ros.call_service("/arm_sim/integration_step", {
            "function": "t", "x0": 0.0, "xdot0": 0.0, "dt": -0.5, "steps": 1, "integrator": "rk4"
        })
        assert r2.get("result") is False, f"Expected rejection for dt < 0: {r2}"

        # 2. Zero steps
        r3 = ros.call_service("/arm_sim/integration_step", {
            "function": "t", "x0": 0.0, "xdot0": 0.0, "dt": 0.1, "steps": 0, "integrator": "rk4"
        })
        assert r3.get("result") is False, f"Expected rejection for steps=0: {r3}"

        # 3. Invalid integrator name
        r4 = ros.call_service("/arm_sim/integration_step", {
            "function": "t", "x0": 0.0, "xdot0": 0.0, "dt": 0.1, "steps": 1, "integrator": "nonexistent"
        })
        assert r4.get("result") is False, f"Expected rejection for unknown integrator: {r4}"
