# Task: Numerical Integration Library and `/arm_sim/integration_step`

Implement the reusable numerical integration layer needed by Project 2.

This task is focused on the four required numerical integrators and the standalone `/arm_sim/integration_step` service. Do not implement the live arm dynamics, PID controller, inverse kinematics, action server, or trial harness yet.

Project 2 reuses Project 1's rosbridge-style TCP/JSON protocol unchanged. Use the existing communication layer rather than implementing a new external protocol.

## Goal

Provide four selectable integrators:

- `euler`
- `midpoint`
- `verlet`
- `rk4`

All four methods must advance a second-order system:

```text
qddot = f(t, q, qdot)
```

by one timestep.

The same integration implementations will later be used by the live arm simulation.

## `/arm_sim/integration_step`

Provide:

```text
/arm_sim/integration_step
```

with arguments:

```json
{
  "function": "<expression in t>",
  "x0": <f64>,
  "xdot0": <f64>,
  "dt": <f64>,
  "steps": <u64>,
  "integrator": "euler"|"midpoint"|"verlet"|"rk4"
}
```

`xdot0` is optional and defaults to `0`.

The service integrates a 1-DOF unit-mass particle whose acceleration is:

```text
qddot = f(t)
```

The initial state is:

```text
t = 0
x = x0
xdot = xdot0
```

## Response

On success return:

```json
{
  "times": [...],
  "positions": [...],
  "velocities": [...]
}
```

There must be one entry for the initial state plus one entry for each requested integration step.

Therefore:

```text
times[0] == 0
positions[0] == x0
velocities[0] == xdot0
```

and the arrays must all have length:

```text
steps + 1
```

## Expression parser

The `function` field is a mathematical expression with one variable:

```text
t
```

At minimum support:

```text
+
-
*
/
^
unary -
()
```

and:

```text
sin
cos
tan
exp
sqrt
ln
abs
```

Numeric literals must be supported.

Examples of valid expressions:

```text
t
sin(t)
t^2 + 3*t - 1
-t + 1
```

Operator precedence from lowest to highest is:

```text
+ -
* /
^
unary -
atoms
```

`+` and `-` are left-associative.

`^` is right-associative.

## Invalid expressions

Malformed expressions must be rejected cleanly.

Examples include:

- empty strings;
- unbalanced parentheses;
- unknown identifiers;
- trailing garbage;
- adjacent values with no operator, such as `1 2`;
- unsupported characters.

Do not crash or hang on malformed input.

A valid expression that evaluates to a floating-point domain error may produce `NaN` or `inf`. Do not add unnecessary special handling for this.

## Invalid service requests

Return:

```text
result: false
```

with a non-empty status message when:

- the expression cannot be parsed;
- the integrator name is unknown;
- `dt <= 0`;
- `steps == 0`.

A rejected request must not damage the runtime.

The service must continue accepting later valid requests.

## Integrators

### Euler

Use:

```text
q_{n+1}    = q_n + qdot_n * dt
qdot_{n+1} = qdot_n + qddot_n * dt
```

with acceleration evaluated at the current state.

### Midpoint

Evaluate acceleration at the current state.

Take a trial half-step.

Evaluate acceleration at the midpoint state.

Use the midpoint derivative to perform the real full step.

### Velocity Verlet

Use velocity Verlet rather than position-only Verlet because the project's forces can depend on velocity.

The method should:

1. evaluate acceleration at the current state;
2. predict the new position;
3. provisionally estimate the new velocity;
4. evaluate acceleration at the predicted state;
5. finalize velocity using the two acceleration evaluations.

### RK4

Implement classical fourth-order Runge-Kutta.

Use four acceleration evaluations and the standard:

```text
1 : 2 : 2 : 1
```

weighted combination.

## Time handling

Each sub-stage must receive the appropriate time.

For example:

```text
t
t + dt/2
t + dt
```

Do not freeze the time at `t` for every sub-stage.

This is especially important for time-varying functions such as:

```text
f(t) = t
```

## Reusability

The integrators should be implemented as reusable code rather than embedding four separate implementations directly inside the service handler.

Later tasks must be able to call the same integrator implementations for the live arm.

The integrator API should be generic enough to operate on the arm's vector-valued state later.

## Acceptance criteria

1. All four integrators are implemented.
2. `/arm_sim/integration_step` accepts the required request.
3. `xdot0` defaults to zero.
4. Results include the starting state at index 0.
5. Result arrays have `steps + 1` entries.
6. Invalid expressions are rejected cleanly.
7. Unknown integrators are rejected.
8. Non-positive `dt` is rejected.
9. Zero `steps` is rejected.
10. Malformed requests do not crash the runtime.
11. Later requests still work after a rejected request.
12. Sub-stage times are correct.
13. The implementations are reusable by the later live simulation.
14. No physics engine or ODE-solver library is used.
15. No ROS/ROS 2 runtime or client library is used.
