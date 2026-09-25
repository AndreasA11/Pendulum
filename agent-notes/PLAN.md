# Plan: Planar Arm Dynamics, Forward Dynamics, and Source/Header Separation (Task 2)

## Goal

1. Refactor Task 1 codebase to separate declarations in header files (`include/pendularm/*.hpp`) from implementations in source files (`src/*.cpp`) in compliance with the updated `IMPLEMENT.md` guidelines.
2. Implement the physical dynamics model for Project 2's planar rotational arm (`prompts/02_arm_dynamics.md`):
   - Generalized closed-form Lagrangian dynamics for $n$-link planar arm (supporting both $n=2$ and $n=3$ without separate derivations).
   - Configuration-dependent mass matrix $M(q)$.
   - Coriolis/centrifugal coupling vector $C(q, \dot{q})\dot{q}$.
   - Gravity load vector $G(q)$ with configurable magnitude $g$ (defaulting to 9.81 m/s²).
   - Numerical linear solver with partial pivoting to solve $M(q) \ddot{q} = \tau - C(q, \dot{q})\dot{q} - G(q)$ for $\ddot{q}$ (forward dynamics).
   - Forward kinematics computing joint positions and end-effector pose $(x, y, \phi_n)$.

## Relevant requirements

1. **Source / Header Separation**:
   - `include/pendularm/integrator.hpp` & `src/integrator.cpp`
   - `include/pendularm/expression.hpp` & `src/expression.cpp`
   - `include/pendularm/integration_service.hpp` & `src/integration_service.cpp`
   - `include/pendularm/arm_dynamics.hpp` & `src/arm_dynamics.cpp`
2. **Generalized $n$-Link Arm Dynamics**:
   - Arm consists of $n$ uniform rigid rods hinged in series at the origin $(0, 0)$ in world coordinates.
   - Link $i$ has length $l_i > 0$, mass $m_i > 0$.
   - Joint $i$ angle is relative angle $q_i$.
   - World heading angle of link $i$: $\phi_i = \sum_{j=1}^i q_j$.
   - COM of uniform link $i$: distance $l_i / 2$ along link, inertia about COM $I_i = \frac{1}{12} m_i l_i^2$.
   - Gravity acts along $-y$ with magnitude $g \ge 0$.
3. **Manipulator Equation of Motion**:
   $$M(q) \ddot{q} + C(q, \dot{q})\dot{q} + G(q) = \tau$$
   - $M(q)$: symmetric, positive-definite $n \times n$ mass matrix.
   - $C(q, \dot{q})\dot{q}$: inter-joint velocity coupling.
   - $G(q)$: gradient of potential energy $\partial V / \partial q$.
   - Forward dynamics: $\ddot{q} = M(q)^{-1} (\tau - C(q, \dot{q})\dot{q} - G(q))$.
4. **Behavioral Invariants & Sanity Checks**:
   - Free motion: with $\tau = 0$ and nonzero gravity, any non-equilibrium configuration exhibits nonzero $\ddot{q}$ and moves.
   - Steady-state equilibrium: when $\ddot{q} = 0, \dot{q} = 0$, holding torque must exactly equal $\tau = G(q)$.
   - Coupling: in a 3-link arm, moving one joint induces forces and accelerations on the other joints via off-diagonal $M(q)$ and Coriolis terms.
   - Both $n=2$ and $n=3$ use the exact same generalized derivation and code.
5. **No external libraries**:
   - Standard C++20 and standard mathematical routines only; no ODE solvers or physics engines.

## Repository observations

- Task 1 headers currently contain inline definitions. Separating them into `.hpp` and `.cpp` keeps compilation modular and interfaces clean.
- `Makefile` currently compiles directly from `src/main.cpp`. It needs to compile all source files (`src/*.cpp`) and link them.
- `include/middleware/` remains application-agnostic and unchanged.

## Proposed architecture

### Module Organization
```text
include/pendularm/
  ├── integrator.hpp          (integrator declarations & types)
  ├── expression.hpp          (AST & parser declarations)
  ├── integration_service.hpp (service registration declaration)
  └── arm_dynamics.hpp        (arm parameters, dynamics & kinematics declarations)

src/
  ├── integrator.cpp          (integrator implementations)
  ├── expression.cpp          (AST & parser implementations)
  ├── integration_service.cpp (service handler implementation)
  ├── arm_dynamics.cpp        (dynamics, kinematics & linear solver implementations)
  └── main.cpp                (main daemon entry point)
```

### Dynamics Formulation
For general $n$-link planar manipulator:
1. $T = \frac{1}{2} \dot{\phi}^T A \dot{\phi} = \frac{1}{2} \dot{q}^T (S^T A S) \dot{q}$, where $S_{j,a} = 1$ if $a \le j$, else 0.
   - $A_{j,j} = l_j^2 \left( \frac{1}{3} m_j + \sum_{i=j+1}^n m_i \right)$
   - $A_{j,k} = l_j l_k \cos(\phi_j - \phi_k) \left( \frac{1}{2} m_{\max(j,k)} + \sum_{i=\max(j,k)+1}^n m_i \right)$ for $j \ne k$.
   - $M(q) = S^T A S \implies M_{a,b}(q) = \sum_{j=a}^n \sum_{k=b}^n A_{j,k}$.
2. Coriolis/centrifugal vector:
   - For $j \ne k$: $\beta_{j,k} = l_j l_k \left( \frac{1}{2} m_{\max(j,k)} + \sum_{i=\max(j,k)+1}^n m_i \right)$ (and $\beta_{j,j} = 0$).
   - $B_j = \sum_{k=1}^n \beta_{j,k} \sin(\phi_j - \phi_k) \dot{\phi}_k^2$.
   - $[C(q, \dot{q})\dot{q}]_a = \sum_{j=a}^n B_j$.
3. Gravity vector:
   - $G_a(q) = \sum_{j=a}^n g l_j \left( \frac{1}{2} m_j + \sum_{i=j+1}^n m_i \right) \cos\phi_j$.
4. Linear solve:
   - Gaussian elimination with partial pivoting solving $M(q) \ddot{q} = \tau - C(q, \dot{q})\dot{q} - G(q)$.

## Interfaces and data flow

```cpp
struct ArmParams {
    std::vector<double> lengths; // l_i > 0
    std::vector<double> masses;  // m_i > 0
    double gravity{9.81};        // g >= 0
};

// Forward dynamics function callable by integrators:
std::vector<double> compute_forward_dynamics(
    double t,
    const std::vector<double>& q,
    const std::vector<double>& qdot,
    const std::vector<double>& tau,
    const ArmParams& params
);

// Forward kinematics:
struct Point2D { double x; double y; };
struct Pose2D { double x; double y; double phi; };
Pose2D compute_forward_kinematics(const std::vector<double>& q, const ArmParams& params);
std::vector<Point2D> compute_all_joint_positions(const std::vector<double>& q, const ArmParams& params);
```

## Implementation steps

1. **Refactor Task 1 files into headers and sources**:
   - `include/pendularm/integrator.hpp` & `src/integrator.cpp`
   - `include/pendularm/expression.hpp` & `src/expression.cpp`
   - `include/pendularm/integration_service.hpp` & `src/integration_service.cpp`
2. **Implement Task 2 files**:
   - `include/pendularm/arm_dynamics.hpp`: Declarations for parameters, mass matrix, Coriolis vector, gravity vector, forward dynamics, and kinematics.
   - `src/arm_dynamics.cpp`: Closed-form generalized implementations for $n$-link arm, linear solver, and forward kinematics.
3. **Update `Makefile`**:
   - Compile object files or compile all `src/*.cpp` together into `build/arm_sim`.
   - Update `test_integrators` to link the new source files.
   - Add `test_dynamics` target compiling `tests/test_dynamics.cpp`.
   - Include `test_dynamics` in `make test`.
4. **Verification Test Suite (`tests/test_dynamics.cpp`)**:
   - Verify mass matrix symmetry and positive-definiteness for $n=2$ and $n=3$.
   - Verify closed-form mass matrix against analytical 2-link textbook solution.
   - Verify Coriolis/centrifugal coupling behavior on 2-link and 3-link arms.
   - Verify steady-state condition: $\tau = G(q)$ when $\ddot{q} = \dot{q} = 0$.
   - Verify free motion under gravity starting from non-equilibrium poses ($\tau = 0$).
   - Verify forward kinematics against geometry for straight, right-angle, and folded poses.
   - Verify linear solver accuracy against known linear systems.
5. **Run tests and update protocol notes**:
   - Build and run `make test`.
   - Complete `agent-notes/IMPLEMENTATION.md`, `agent-notes/AUDIT.md`, and `agent-notes/TEST_RESULTS.md`.

## Edge cases and failure modes

- **Negative or zero masses/lengths**: Enforce validation in `ArmParams::validate()`.
- **Singular configurations**: $M(q)$ is strictly positive-definite for all $q$ when masses and lengths are positive; partial pivoting ensures numerical stability everywhere.
- **Orientation angle wrapping**: Angles $\phi_i = \sum_{j=1}^i q_j$ work directly in $\sin$ and $\cos$ without requiring wrapping.
- **Link count mismatch**: Check that $q, \dot{q}, \tau$ sizes match the number of links in `params`.

## Verification strategy

- Run independent unit tests comparing 2-link mass matrix and gravity vector to analytical formulas.
- Confirm eigenvalues of $M(q)$ are strictly positive for arbitrary configurations.
- Verify that accelerating joint 1 produces torque/acceleration on joint 2 and joint 3 (inter-joint coupling).
- Confirm that steady-state hold torque matches $G(q)$ across multiple test angles.
- Verify existing Task 1 tests still pass 100% after source/header separation.
