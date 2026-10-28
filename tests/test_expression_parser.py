from rosbridge_client import Rosbridge


def test_parser_operator_precedence_and_functions():
    with Rosbridge() as ros:
        # Precedence: 2 * 3^2 + 4 / 2 - 5 = 15 (not 36)
        reply = ros.call_service("/arm_sim/integration_step", {
            "function": "2 * 3^2 + 4 / 2 - 5", "x0": 0.0, "xdot0": 0.0, "dt": 1.0, "steps": 1, "integrator": "euler"
        })
        assert reply.get("result") is True, f"Parsing valid expression failed: {reply}"
        v1 = reply["values"]["velocities"][1]
        assert abs(v1 - 15.0) < 1e-7, f"Operator precedence failed: expected v=15.0, got {v1}"

        # Right-associativity of ^: 2^3^2 = 2^(9) = 512, not (2^3)^2 = 64
        reply_pow = ros.call_service("/arm_sim/integration_step", {
            "function": "2^3^2", "x0": 0.0, "xdot0": 0.0, "dt": 1.0, "steps": 1, "integrator": "euler"
        })
        assert reply_pow.get("result") is True, f"Power expression failed: {reply_pow}"
        v_pow = reply_pow["values"]["velocities"][1]
        assert abs(v_pow - 512.0) < 1e-7, f"Exponentiation must be right-associative: expected 512, got {v_pow}"

        # Standard math functions at t=0:
        # sin(0)=0, cos(0)=1, exp(0)=1, sqrt(4)=2, ln(1)=0, abs(-5)=5 -> total = 9.0
        expr = "sin(0) + cos(0) + exp(0) + sqrt(4) + ln(1) + abs(-5)"
        reply_fn = ros.call_service("/arm_sim/integration_step", {
            "function": expr, "x0": 0.0, "xdot0": 0.0, "dt": 1.0, "steps": 1, "integrator": "euler"
        })
        assert reply_fn.get("result") is True, f"Standard math functions failed: {reply_fn}"
        v_fn = reply_fn["values"]["velocities"][1]
        assert abs(v_fn - 9.0) < 1e-7, f"Expected functions evaluation 9.0, got {v_fn}"


def test_parser_syntax_errors():
    with Rosbridge() as ros:
        bad_expressions = [
            "",               # empty expression
            "((t + 1)",       # unbalanced parens
            "t + 1)",         # stray closing paren
            "1 2",            # adjacent tokens without operator
            "t @ 5",          # invalid character
            "sin(t, 2)",      # wrong arity
            "undefined_func(t)",  # unknown function
            "t *",            # dangling operator
        ]
        for expr in bad_expressions:
            reply = ros.call_service("/arm_sim/integration_step", {
                "function": expr, "x0": 0.0, "xdot0": 0.0, "dt": 0.1, "steps": 1, "integrator": "rk4"
            })
            assert reply.get("result") is False, f"Expected rejection for malformed '{expr}', got: {reply}"
            assert bool(reply.get("status")), f"Status message must not be empty for rejected '{expr}': {reply}"
