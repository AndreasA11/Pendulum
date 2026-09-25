# Task: Multi-Joint PID Controller

Implement the multi-joint PID controller for the live arm simulation.

This task is focused on controller state, gain management, enable/disable behavior, and effort generation. It should plug into the live simulation without changing the dynamics implementation.

## Goal

For every joint, compute effort from:

```text
position_error = setpoint_pos - actual_pos
velocity_error = setpoint_vel - actual_vel

integral = clamp(
    integral + position_error * dt,
    -integral_limit,
    +integral_limit
)

effort =
    kp * position_error
    + ki * integral
    + kd * velocity_error
```

The controller operates independently per joint.

## Disabled state

PID is disabled by default.

While disabled:

```text
effort = 0
```

The arm should therefore move freely under gravity.

Do not accumulate the PID integral while the controller is disabled.

## `/pid_controller/enable`

Request:

```json
{"data": true|false}
```

When enabling:

- PID becomes active;
- clear the accumulated integral before control resumes.

When disabling:

- PID stops producing effort;
- future simulation steps must not accumulate integral while disabled.

Re-enabling must start with a clean integral.

## `/pid_controller/set_gains`

Request:

```json
{
  "kp": [...],
  "ki": [...],
  "kd": [...]
}
```

Each field is independently optional.

Reject a supplied array when:

- it has the wrong number of entries;
- any value is negative.

Valid gain fields should be applied without disturbing other valid fields.

Changing gains must **not** reset the accumulated integral.

This is intentionally different from disabling/re-enabling.

## Integral anti-windup

The integral must have a fixed finite bound:

```text
-integral_limit <= integral <= integral_limit
```

The exact positive limit is implementation-defined.

The important requirement is that the integral cannot grow without bound during sustained error.

## Derivative term

Use:

```text
velocity_error = setpoint_vel - actual_vel
```

for the derivative contribution.

Do not calculate the derivative term by finite-differencing position error.

The measured velocity is already available from the simulation and avoids amplifying position noise.

## Gravity compensation behavior

Do not add a separate gravity-compensation term to the controller.

Effort must come from the PID equation.

Once the arm has converged to a steady state:

```text
qddot = 0
qdot = 0
```

the plant requires:

```text
tau = G(q)
```

The integral term should naturally provide the effort needed to remove the steady-state gravity error.

## Setpoint source

The controller's position and velocity setpoints come from the final point of `/joint_trajectory`.

If velocity setpoints are omitted or the wrong length, they are zero.

## Reset behavior

`/arm_sim/reset` must reset:

- controller setpoint;
- controller integral.

Changing gains must not reset the integral.

Disabling and then re-enabling must reset the integral.

## Acceptance criteria

1. PID is disabled by default.
2. Disabled PID produces zero effort.
3. Enabling PID activates control.
4. Re-enabling clears the previous integral.
5. Disabled periods do not accumulate integral.
6. Gain arrays are validated by joint count.
7. Negative gains are rejected.
8. Gain fields can be updated independently.
9. Changing gains preserves the accumulated integral.
10. The integral is clamped to a finite limit.
11. The derivative term uses commanded minus measured velocity.
12. No separate gravity-compensation torque is injected.
13. Reset clears controller setpoint and integral.
14. Reasonable gains can drive the arm toward a commanded setpoint within the project's intentionally generous tolerance.
15. The runtime remains responsive after invalid gain/enable requests.
