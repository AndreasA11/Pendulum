# Implementation Notes: Planar Arm Dynamics and Source/Header Separation (Task 2)

## What changed

1. **Source / Header Separation (Task 1 Refactoring)**:
   - Modified Task 1 files to separate declarations in `.hpp` from definitions in `.cpp` per updated `IMPLEMENT.md`:
     - `include/pendularm/integrator.hpp` & `src/integrator.cpp`: Integrator types, step functions (`step_euler`, `step_midpoint`, `step_verlet`, `step_rk4`), and `integrate_step` dispatcher.
     - `include/pendularm/expression.hpp` & `src/expression.cpp`: AST nodes, `Lexer`, `Parser`, and `parse_expression`.
     - `include/pendularm/integration_service.hpp` & `src/integration_service.cpp`: `handle_integration_step` and `register_integration_service`.
   - Added comments starting with `*CHANGES*` to document reasons for changes.

2. **Planar Arm Dynamics (`include/pendularm/arm_dynamics.hpp` & `src/arm_dynamics.cpp`)**:
   - Implemented `ArmParams`: holds `lengths`, `masses`, `gravity` ($g \ge 0$, defaults to 9.81 m/s²), and validation helper.
   - Implemented general closed-form Lagrangian dynamics for $n$-link planar serial manipulator with uniform rigid links ($I_i = \frac{1}{12} m_i l_i^2$):
     - `compute_mass_matrix(q, params)`: Computes $M(q) = S^T A S$, where $A_{j,k}$ is the kinetic energy coefficient matrix in world angle coordinates $\phi_i = \sum_{k=1}^i q_k$. Handles any link count ($n=2, 3$).
     - `compute_coriolis_vector(q, qdot, params)`: Computes $C(q, \dot{q})\dot{q}$ using velocity-dependent centrifugal/Coriolis coupling terms $B_j = \sum_k \beta_{j,k} \sin(\phi_j - \phi_k) \dot{\phi}_k^2$.
     - `compute_gravity_vector(q, params)`: Computes $G(q) = \partial V / \partial q = \sum_{j=a}^n \gamma_j \cos\phi_j$.
   - Implemented `solve_linear_system(A, b)`: Gaussian elimination with partial pivoting to solve $M(q) \ddot{q} = \tau - C(q, \dot{q})\dot{q} - G(q)$ without requiring matrix inversion. Throws on singular matrices.
   - Implemented `forward_dynamics(q, qdot, tau, params)`: Computes joint accelerations $\ddot{q}$.
   - Implemented `make_forward_dynamics_accel_func(tau, params)`: Returns an `AccelFunc` compatible with the Task 1 numerical integrators.
   - Implemented `forward_kinematics_joints` and `forward_kinematics_end_effector`: Computes $(x, y)$ coordinates of all joints and end-effector pose $(x, y, \phi_n)$.

3. **Makefile**:
   - Updated build dependencies to compile all `src/*.cpp` modules.
   - Added target `test_dynamics` running `tests/test_dynamics.cpp`.
   - Added `test_dynamics` to `make test`.

## Important implementation decisions

- **Unified $n$-Link Lagrangian Formulation**: Rather than hand-deriving separate formulas for $n=2$ and $n=3$, a single unified derivation in world link orientations $\phi_i$ was used. The coordinate transformation matrix $S$ maps joint velocities $\dot{q}$ to link angular velocities $\dot{\phi} = S \dot{q}$. This eliminates special cases and guarantees consistency across both 2-link and 3-link configurations.
- **Partial Pivoting Linear Solver**: $M(q) \ddot{q} = \text{RHS}$ is solved using row pivoting, preserving numerical stability even near kinematic boundaries without explicitly computing $M(q)^{-1}$.
- **Decoupled Architecture**: Dynamics and kinematics are pure functions operating on parameter and state vectors, making them easily callable by future simulation nodes (Task 3), PID controllers (Task 4), and IK algorithms (Task 5).

## Deviations from the plan

- None. Implementation followed all steps defined in `agent-notes/PLAN.md`.

## Known limitations or unresolved questions

- Tasks 3 through 8 (arm simulation ROS service node, PID controller, inverse kinematics, IK action server, and trial harness) are left for their respective subtasks.

## Checks performed

- `make build`: Clean compilation with `-Wall -Wextra -Wpedantic` on C++20.
- `make test_middleware`: Passed.
- `make test_integrators`: Passed (Task 1 regression check after source/header separation).
- `make test_dynamics`: Passed all analytical mass matrix, Coriolis coupling, gravity load, free motion, linear solver, and forward kinematics checks.
- `make test`: All 3 test suites passed.
