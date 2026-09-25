# Task: Closed-Form Inverse Kinematics

Implement the closed-form inverse kinematics service:

```text
/ik/solve
```

This task is focused only on computing joint angles from a desired end-effector pose.

Do not implement the IK action server or trial harness yet.

## Goal

Given a desired planar end-effector position and optional orientation, compute joint angles for the currently configured 2-link or 3-link arm.

Use closed-form geometry.

Do not use iterative Jacobian-based IK.

## Forward kinematics

The project's joint convention is:

```text
phi_i = q_1 + ... + q_i
```

The end-effector position is:

```text
x = sum(l_i * cos(phi_i))
y = sum(l_i * sin(phi_i))
```

For a 3-link arm, the absolute end-effector orientation is:

```text
phi_3 = q_1 + q_2 + q_3
```

## `/ik/solve`

Request:

```json
{
  "x": <meters>,
  "y": <meters>,
  "phi": <radians, optional>
}
```

Return:

```json
{
  "positions": [n values]
}
```

where `n` is the configured link count.

## Current parameters

Fetch the arm's current link lengths fresh on every solve.

Do not cache link lengths permanently.

A solve issued after `/arm_sim/set_params` changes the lengths must use the new values.

## Two-link IK

For a 2-link arm, position consumes both degrees of freedom.

Use the Law of Cosines to solve the elbow configuration.

A reachable target may have two solutions:

- elbow-up;
- elbow-down.

Either is valid.

Do not require one particular configuration.

The returned angles must round-trip through forward kinematics to the requested position.

## Three-link IK

For a 3-link arm:

1. use the requested `phi`;
2. subtract link 3's contribution to obtain the wrist position;
3. solve the remaining 2-link problem using the first two links;
4. choose the resulting `q1`, `q2`;
5. compute `q3` so the final orientation equals the requested `phi`.

The requested orientation is meaningful for 3-link IK.

If no `phi` is supplied, follow the project's defined optional-field behavior consistently.

## Reachability

Reject with:

```text
result: false
```

and a non-empty status message when:

- `x` is missing or non-numeric;
- `y` is missing or non-numeric;
- the target is farther than total arm extension;
- for a 2-link arm, the target is closer than `|l1-l2|`;
- for a 3-link arm, the requested orientation produces a wrist point outside the first two links' reachable region.

Exact reach boundaries are reachable.

Do not reject a target merely because floating-point arithmetic produces a value slightly outside the mathematical boundary.

Clamp tiny floating-point deviations where appropriate rather than incorrectly rejecting exact boundary cases.

## No iterative solver

Do not implement numerical optimization, Jacobian iteration, gradient descent, or another iterative IK method.

The intended solution is closed-form:

```text
Law of Cosines
+
kinematic decoupling
```

## Acceptance criteria

1. `/ik/solve` exists.
2. It supports both 2-link and 3-link arms.
3. Current link lengths are fetched for every solve.
4. Reachable 2-link targets return valid joint angles.
5. Either elbow configuration is accepted.
6. 3-link targets respect requested orientation.
7. Wrist-position reachability is checked for 3-link requests.
8. Fully extended boundary targets are accepted.
9. Fully folded 2-link boundary targets are accepted.
10. Unreachable targets return `result:false`.
11. Missing/non-numeric `x` or `y` is rejected.
12. Returned angles round-trip through forward kinematics to the requested position.
13. 3-link returned angles reproduce the requested orientation.
14. No iterative/Jacobian IK is used.
15. Invalid requests do not crash or hang the runtime.
