# Task: Live Arm Simulation, Topics, and Simulation Services

Build the live arm simulation around the dynamics and integrator components.

This task connects the physical model to the Project 2 runtime and implements the simulation-control interface.

Project 1's rosbridge TCP/JSON protocol remains unchanged. Reuse the existing middleware and external gateway.

## Goal

Provide:

- configurable 2-link or 3-link simulation;
- continuous `/joint_states` publication;
- `/joint_trajectory` step-setpoint handling;
- `/arm_sim/set_integrator`;
- `/arm_sim/set_params`;
- `/arm_sim/pause`;
- `/arm_sim/reset`.

The physics loop must use the dynamics and one of the four reusable integrators.

## Link count

At startup read:

```text
ARM_SIM_LINKS
```

Valid values are:

```text
"2"
"3"
```

If unset, default to:

```text
2
```

The selected number of links controls the size of:

- positions;
- velocities;
- efforts;
- masses;
- lengths;
- PID gains;
- trajectory arrays.

## `/joint_trajectory`

Subscribe to:

```text
/joint_trajectory
```

The message is JointTrajectory-shaped:

```json
{
  "header": {
    "stamp": {"sec": 0, "nanosec": 0},
    "frame_id": ""
  },
  "joint_names": ["joint1", "joint2"],
  "points": [
    {
      "positions": [0.3, -0.5],
      "velocities": [0.0, 0.0],
      "accelerations": [],
      "time_from_start": {"sec": 0, "nanosec": 0}
    }
  ]
}
```

This is a step-setpoint servo, not a trajectory follower.

Only the final entry in `points` is used.

Set:

```text
setpoint_pos = final positions
setpoint_vel = final velocities
```

If velocities are omitted or have the wrong length, use an all-zero velocity setpoint.

Do not interpolate through intermediate waypoints.

## `/joint_states`

Continuously publish:

```text
/joint_states
```

with:

```json
{
  "header": {...},
  "name": [...],
  "position": [...],
  "velocity": [...],
  "effort": [...]
}
```

`name`, `position`, `velocity`, and `effort` must contain one entry per simulated joint.

The exact publication rate is implementation-defined.

A rate around 60 Hz is reasonable, but the external contract does not require an exact rate.

The timestamp may use simulation time or wall time.

## Simulation timestep and integrator

The physics loop must advance the live `(q, qdot)` state using the selected integrator.

Do not create a separate approximate stepping implementation for the live arm.

Use the reusable integration library from the previous task.

## `/arm_sim/set_integrator`

Request:

```json
{
  "method": "euler|midpoint|verlet|rk4",
  "timestep": <positive seconds>
}
```

Update the live simulation settings going forward.

Reject:

- unknown method names;
- non-positive timesteps.

Do not silently substitute a default method or timestep.

## `/arm_sim/set_params`

Request fields are independently optional:

```json
{
  "gravity": <>=0>,
  "masses": [n positive values],
  "lengths": [n positive values]
}
```

Gravity defaults to:

```text
9.81
```

A request `{}` queries the current parameters without changing them.

Each supplied field must be validated independently.

Reject a field when:

- gravity is negative;
- masses have the wrong length;
- any mass is non-positive;
- lengths have the wrong length;
- any length is non-positive.

Valid fields in the same request should still be applied when another field is invalid.

The response must always contain the current post-update parameters, including after a rejected request.

## `/arm_sim/pause`

Request:

```json
{"data": true|false}
```

When paused:

- simulation time stops;
- physics stepping stops;
- PID integral accumulation stops;
- `/joint_states` continues publishing the frozen state;
- all services continue responding.

When resumed, simulation continues from the frozen state.

## `/arm_sim/reset`

Request:

```json
{}
```

Reset:

```text
q
qdot
simulation time
PID setpoint
PID integral
```

to the project's fixed initial/reset state.

The response must include:

```json
{
  "position": [...],
  "velocity": [...]
}
```

A reset must reset the controller together with the plant so the controller does not immediately pull the arm back toward an old setpoint.

## Service convention

These simulation-control and PID services share the Project 2 convention:

- every request field is optional and independently applied;
- `{}` queries current values where the service supports querying.

Do not accidentally turn an omitted field into an implicit reset or default update.

## Acceptance criteria

1. `ARM_SIM_LINKS=2` creates a 2-link simulation.
2. `ARM_SIM_LINKS=3` creates a 3-link simulation.
3. Unset `ARM_SIM_LINKS` defaults to 2.
4. `/joint_trajectory` uses only its final point.
5. `/joint_trajectory` does not interpolate waypoints.
6. `/joint_states` continuously publishes parallel joint arrays.
7. The live physics loop uses the selected reusable integrator.
8. `/arm_sim/set_integrator` validates method and timestep.
9. `/arm_sim/set_params` validates each field independently.
10. `{}` can read current parameters.
11. `/arm_sim/pause` freezes simulation advancement without disabling services.
12. `/joint_states` continues during pause with a frozen state.
13. `/arm_sim/reset` resets plant, time, setpoint, and PID integral.
14. Reset returns the resulting position and velocity.
15. The runtime remains responsive after invalid service requests.
