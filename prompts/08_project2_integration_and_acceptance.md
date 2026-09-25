# Task: Project 2 Integration, Runtime, and End-to-End Contract

Integrate the Project 2 components into one runtime while preserving the Project 1 rosbridge protocol.

This task should be performed after the individual components are implemented and tested.

## Goal

Provide a complete Project 2 runtime exposing:

```text
127.0.0.1:9095
```

through the same TCP/JSON rosbridge-style protocol used by Project 1.

Do not modify the external protocol to accommodate Project 2.

## Required logical components

The runtime must support the behavior of:

```text
arm_sim
ik
ik_action
ik_trial
```

These do not need to be separate OS processes.

They may be:

- separate components;
- separate threads;
- combined into one process;
- otherwise organized internally.

External behavior is what matters.

## Project 1 protocol reuse

Reuse the existing Project 1:

- TCP server;
- JSON framing/envelope;
- topic subscription;
- topic publication;
- service calls;
- service responses;
- client correlation;
- connection cleanup.

Do not create a second incompatible gateway.

## Required external interfaces

The integrated runtime must expose:

### Topics

```text
/joint_trajectory
/joint_states
/ik_action/feedback
/ik_action/result
/ik_trial/status
```

### Services

```text
/arm_sim/integration_step
/arm_sim/set_integrator
/arm_sim/set_params
/arm_sim/pause
/arm_sim/reset

/pid_controller/enable
/pid_controller/set_gains

/ik/solve

/ik_action/send_goal
/ik_action/cancel_goal

/ik_trial/start
/ik_trial/skip
/ik_trial/stop
```

## Build interface

The project root must contain a top-level `Makefile` supporting:

```text
make build
make run
make clean
```

`make build` must be:

- noninteractive;
- repeatable;
- offline.

`make run` must launch the runtime in the foreground.

`make clean` must remove generated build artifacts.

There is no `make map` target for Project 2.

## Link-count configuration

`make run` must respect:

```text
ARM_SIM_LINKS
```

with:

```text
2
3
```

supported and:

```text
unset -> 2
```

as the default.

The same executable/runtime should support both configurations.

## Runtime responsiveness

A malformed or rejected request must not take down the runtime.

The runtime must remain capable of handling later:

- service calls;
- topic messages;
- subscriptions;
- goal commands.

Do not allow one blocking operation to prevent the rest of the runtime from communicating.

## End-to-end checks

Verify at minimum:

### Integration

A client can call `/arm_sim/integration_step` with each integrator.

### Simulation

A client can:

1. read current parameters;
2. observe `/joint_states`;
3. change integrator/timestep;
4. pause;
5. observe frozen state;
6. resume;
7. reset.

### PID

A client can:

1. observe zero effort at startup;
2. enable PID;
3. send a joint setpoint;
4. observe effort and state response;
5. change gains;
6. disable/re-enable;
7. reset.

### IK

A client can call `/ik/solve` and verify the returned joint angles through forward kinematics.

### IK action

A client can:

1. send a reachable goal;
2. receive feedback;
3. observe `/joint_trajectory`;
4. receive exactly one result.

A second reachable goal should preempt the first.

An unreachable replacement should not disturb the active goal.

### Trial

A client can:

1. start a trial;
2. observe status;
3. skip a target;
4. observe a replacement;
5. stop the trial;
6. verify stale state is cleared.

## Acceptance criteria

1. `make build` succeeds offline.
2. `make run` launches the runtime.
3. The runtime listens on `127.0.0.1:9095`.
4. Project 1's rosbridge protocol remains unchanged.
5. Both 2-link and 3-link modes work.
6. All required services are registered.
7. All required topics are available.
8. Integration services use the reusable integrators.
9. Live simulation uses the same integrator implementations.
10. Live simulation uses the implemented dynamics.
11. PID drives the live simulation.
12. IK uses current arm lengths.
13. IK action commands the arm through the trajectory topic.
14. Trial orchestration uses the IK action interface.
15. Invalid requests fail without crashing the runtime.
16. Multiple external clients can interact with the runtime without corrupting each other's protocol state.
17. The complete system remains responsive throughout normal operation.
