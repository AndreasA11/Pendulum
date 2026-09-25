# Task: IK Trial Harness

Implement the timed IK trial harness.

This task builds on the IK action interface. The trial harness should behave as a client/orchestrator of `/ik_action/*`, not as a second arm controller.

## Goal

Provide:

```text
/ik_trial/start
/ik_trial/skip
/ik_trial/stop
/ik_trial/status
```

The trial repeatedly samples reachable IK targets and measures how many are reached within a time window.

## Architectural constraint

The trial harness must not:

- read `/joint_states` directly;
- publish `/joint_trajectory` directly.

Instead it should interact with the IK action layer through:

```text
/ik_action/send_goal
/ik_action/cancel_goal
/ik_action/feedback
/ik_action/result
```

The trial is an orchestrator, not another controller.

## `/ik_trial/start`

Accept:

```json
{}
```

or:

```json
{
  "duration": <seconds>,
  "epsilon": <meters>,
  "success_hold": <seconds>
}
```

All fields are optional.

Starting a trial always creates a fresh trial.

That means:

- cancel any previous goal;
- reset reached count;
- reset elapsed time;
- establish the new trial configuration;
- sample and submit the first target.

If no reachable target can be sampled, return:

```text
result:false
```

with a non-empty status message.

Otherwise return success.

## Target sampling

Repeatedly sample reachable targets.

The exact distribution is implementation-defined.

Any strategy is acceptable provided it reliably produces reachable targets.

One reasonable approach is:

1. sample a target in a bounding region;
2. reject unreachable samples;
3. repeat until a reachable target is found.

For a 3-link arm, include a valid orientation when required.

## Trial timing

The trial has a simulation-time duration.

When the duration expires:

- stop advancing the trial;
- abandon the active target;
- do not count the abandoned target as reached;
- do not submit a replacement target.

The trial should then report `running:false`.

## `/ik_trial/skip`

Request:

```json
{}
```

If no trial is running, reject.

Otherwise:

1. abandon the currently active target;
2. do not count it as reached;
3. submit a new target immediately.

The trial remains running.

## `/ik_trial/stop`

Request:

```json
{}
```

If no trial is running, reject.

Otherwise:

1. end the trial;
2. abandon the active target;
3. do not submit a replacement;
4. stop advancement.

The trial should report `running:false`.

## `/ik_trial/status`

Publish periodically while a trial exists:

```json
{
  "running": <bool>,
  "elapsed": <seconds>,
  "duration": <seconds>,
  "targets_reached": <count>,
  "target": {
    "x": ...,
    "y": ...,
    "phi": ...
  },
  "error": <meters>,
  "desired_positions": [...],
  "action_status": "idle"|"active"|"reached"|"preempted"
}
```

Fields that represent no active target must be cleared:

```text
target = null
error = null
desired_positions = null
action_status = "idle"
```

Do not leave stale target information after the goal has been abandoned without an immediate replacement.

## Trial state transitions

A useful lifecycle is:

```text
idle
  |
  | start
  v
active
  | \
  |  \
  |   \ stop / duration
  |    \
  |     v
  |    idle
  |
  | target reached
  v
sample next target
```

`skip` abandons the current target but leaves the trial active.

## Counting reached targets

Increment:

```text
targets_reached
```

only when the IK action reports:

```text
outcome = "reached"
```

Do not count:

- skipped targets;
- preempted targets;
- abandoned targets;
- targets still active when the trial duration expires.

## Acceptance criteria

1. `/ik_trial/start` exists.
2. `{}` starts a fresh trial.
3. Optional duration/epsilon/success_hold overrides work independently.
4. Starting a new trial resets trial state.
5. A first reachable target is submitted automatically.
6. Unreachable targets are not counted as successes.
7. The trial uses `/ik_action/*` rather than directly controlling the arm.
8. `/ik_trial/skip` abandons the current target and immediately submits another.
9. Skip does not increment the reached count.
10. `/ik_trial/stop` ends the trial without submitting a replacement.
11. Duration expiry ends the trial without counting the in-flight target.
12. `/ik_trial/status` publishes the required fields.
13. Stale target/action state is cleared when nothing is in flight.
14. Reached count increments only after an action result of `reached`.
15. Invalid commands are rejected cleanly.
