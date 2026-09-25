# Task: IK Action Goal Handling and Convergence

Implement the IK action interface that drives the live arm toward an IK-solved target.

This task builds on `/ik/solve`, `/joint_trajectory`, `/joint_states`, and the PID-controlled simulation.

Do not implement the trial harness yet.

## Goal

Provide:

```text
/ik_action/send_goal
/ik_action/cancel_goal
/ik_action/feedback
/ik_action/result
```

At most one IK goal may be active at a time.

## `/ik_action/send_goal`

Request:

```json
{
  "x": <meters>,
  "y": <meters>,
  "phi": <radians, optional>,
  "epsilon": <meters, optional>,
  "success_hold": <seconds, optional>
}
```

Solve the target using `/ik/solve`.

If the target is unreachable:

```text
result = false
```

and the currently active goal must remain completely undisturbed.

If the target is reachable:

1. solve IK;
2. command the returned joint positions through `/joint_trajectory`;
3. begin tracking convergence;
4. return a unique:
   ```text
   goal_id
   ```

## Goal preemption

Only one goal may be active.

If a reachable new goal is submitted while another goal is active:

1. publish the old goal's `/ik_action/result` with:
   ```text
   outcome = "preempted"
   ```
2. start the new goal.

Do not leave the old goal active.

An unreachable replacement must not preempt the current goal.

## Defaults

`epsilon` and `success_hold` are optional.

Choose reasonable implementation-defined defaults.

Requirements:

```text
epsilon > 0
success_hold >= 0
```

when provided.

## `/ik_action/cancel_goal`

Accept:

```json
{}
```

or:

```json
{
  "goal_id": "<string>"
}
```

If there is no active goal, reject.

If a supplied `goal_id` does not match the active goal, reject without side effects.

Otherwise:

1. preempt the active goal;
2. publish its result as:
   ```text
   outcome = "preempted"
   ```
3. clear the active goal;
4. return success.

## `/ik_action/feedback`

While a goal is active, publish once per control tick:

```json
{
  "goal_id": "...",
  "target": {
    "x": ...,
    "y": ...,
    "phi": ...
  },
  "positions": [...],
  "distance_remaining": <meters>,
  "elapsed": <seconds>
}
```

`positions` contains the commanded joint-space setpoint.

`distance_remaining` is the current end-effector distance from the requested target position.

## `/ik_action/result`

Publish exactly once when a goal concludes:

```json
{
  "goal_id": "...",
  "outcome": "reached"|"preempted",
  "target": {...},
  "final_distance": <meters>
}
```

A goal can conclude as:

- `reached`;
- `preempted`.

Do not publish multiple results for the same goal.

## Success dwell

Being inside the epsilon radius for one instant is not enough when:

```text
success_hold > 0
```

The end effector must remain continuously within `epsilon` for at least:

```text
success_hold
```

seconds of **simulation time**.

If the end effector leaves the epsilon region, reset the dwell timer.

A `success_hold` of zero succeeds immediately on first entry into the epsilon region.

Use simulation time from the same clock used by `/joint_states`.

Do not use wall-clock time for the dwell requirement.

## Goal lifecycle

Keep explicit state for:

- active/inactive;
- goal ID;
- target;
- desired joint positions;
- epsilon;
- success hold;
- elapsed time;
- current dwell time.

When a goal reaches or is preempted, clear its active state.

## Acceptance criteria

1. Reachable goals are solved through the IK implementation.
2. Reachable goals command the arm through `/joint_trajectory`.
3. At most one goal is active.
4. A reachable new goal preempts the old goal.
5. An unreachable new goal leaves the old goal unchanged.
6. Every active goal produces periodic feedback.
7. Every concluded goal produces exactly one result.
8. Cancel works with `{}`.
9. Cancel works with a matching goal ID.
10. Cancel rejects a mismatched goal ID.
11. Cancel rejects when no goal is active.
12. Success requires continuous epsilon dwell when `success_hold > 0`.
13. Leaving epsilon resets the dwell timer.
14. `success_hold == 0` succeeds on first epsilon entry.
15. Dwell uses simulation time.
16. Result reports `reached` or `preempted` correctly.
