# Task: Planar Arm Dynamics and Forward Dynamics

Implement the physical dynamics model for Project 2's planar rotational arm.

This task is focused on deriving and implementing the manipulator dynamics for both supported configurations:

- 2 links;
- 3 links.

Do not implement PID control, inverse kinematics, action handling, or trial orchestration yet.

## Goal

Implement the general serial planar RR...R arm model using closed-form Lagrangian dynamics.

The implementation must work for `n = 2` and `n = 3` without maintaining two unrelated derivations.

The arm consists of uniform rigid rods:

- link `i` has length `l_i`;
- link `i` has mass `m_i`;
- joint `i` has relative angle `q_i`.

The base is fixed at the origin.

Gravity acts in world `-y` with magnitude `g`.

## Joint convention

The absolute orientation of link `i` is:

```text
phi_i = q_1 + ... + q_i
```

The end-effector position is:

```text
x = sum(l_i * cos(phi_i))
y = sum(l_i * sin(phi_i))
```

The end-effector orientation is:

```text
phi_n
```

## Equations of motion

Implement:

```text
M(q) qddot + C(q, qdot) qdot + G(q) = tau
```

where:

- `M(q)` is the configuration-dependent mass/inertia matrix;
- `C(q,qdot) qdot` is the Coriolis/centrifugal term;
- `G(q)` is the gravity-load vector;
- `tau` is the applied joint effort.

Forward dynamics must solve:

```text
qddot = M(q)^-1 (tau - C(q,qdot) qdot - G(q))
```

Do not explicitly require a matrix inverse if a numerical linear solve is more appropriate.

## Mass matrix

Implement the configuration-dependent mass matrix for a general `n`-link serial planar manipulator.

The resulting matrix must be symmetric and positive-definite for physically valid positive masses and lengths.

Do not replace the coupling terms with independent single-joint inertias.

## Coriolis and centrifugal forces

Implement the velocity-dependent coupling term.

The Coriolis/centrifugal term must allow movement of one joint to affect the acceleration/effort relationship of other joints.

The implementation does not need to expose a particular `C` matrix representation if the resulting:

```text
C(q,qdot) qdot
```

is correct.

## Gravity

Implement the gravity-load vector from the arm's potential energy.

Gravity acts along world `-y`.

The configured gravity magnitude comes from the runtime parameters and defaults to:

```text
9.81
```

## General `n`-link implementation

Prefer one general implementation parameterized by the number of links.

Do not create separate hard-coded physics equations for exactly two links and exactly three links unless they are generated from the same general formulation.

The same dynamics implementation must operate for both:

```text
n = 2
n = 3
```

## Sanity checks

The implementation should demonstrate:

### Free motion

With:

```text
tau = 0
```

and nonzero gravity, a non-equilibrium pose should change over time.

### Steady state

At a converged state where:

```text
qddot = 0
qdot = 0
```

the equation becomes:

```text
tau = G(q)
```

This should be observable through the eventual `/joint_states` effort values.

### Coupling

A 3-link arm must exhibit inter-joint coupling.

Do not model the three joints as independent single-joint systems.

## Linear solve

The method used to solve for `qddot` is implementation-defined.

Acceptable approaches include:

- Gaussian elimination;
- LU-style solving;
- another numerically reasonable linear solver.

Do not introduce a physics engine or external ODE solver.

## Reusability

Expose a clean dynamics interface that later integration code can call with:

```text
t
q
qdot
tau
parameters
```

and receive:

```text
qddot
```

The live simulation must use this same dynamics implementation.

## Acceptance criteria

1. The dynamics support both 2-link and 3-link arms.
2. The joint-angle convention matches the specification.
3. The base is fixed at the world origin.
4. Gravity acts along world `-y`.
5. The mass matrix depends on configuration.
6. The mass matrix is coupled across joints.
7. Coriolis/centrifugal coupling is implemented.
8. Gravity load is implemented.
9. Forward dynamics solve the manipulator equation correctly.
10. Zero-effort non-equilibrium states move under gravity.
11. A converged state satisfies the expected gravity-load relationship.
12. 3-link dynamics exhibit inter-joint coupling.
13. The implementation is general rather than two unrelated special cases.
14. No physics engine or ODE-solver library is used.
