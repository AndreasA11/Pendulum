# Verification Report: Planar Arm Dynamics and Source/Header Separation (Task 2)

## Summary

Full verification testing was performed on the planar arm dynamics layer, forward kinematics, linear solver, and the refactored Task 1 modules after separating source and header files.

All test suites passed 100%:
- `test_middleware`: 6/6 tests passed.
- `test_integrators`: 6/6 test groups passed (full regression test after source/header separation).
- `test_dynamics`: 6/6 test groups passed covering mass matrix, Coriolis coupling, gravity load, free motion, linear solver, and forward kinematics.

## Test environment

- **OS:** macOS (Darwin arm64)
- **Compiler:** Apple clang / c++ (supporting C++20)
- **Flags:** `-std=c++20 -Wall -Wextra -Wpedantic -Iinclude`
- **Build Targets Tested:** `make build`, `make test`, `make test_dynamics`, `make test_integrators`, `make test_middleware`, `make clean`

## Tests performed

### 1. Mass Matrix M(q) Properties & Analytical Form
- **Requirement/risk being tested:** $M(q)$ must be symmetric, positive-definite, configuration-dependent, and match analytical textbook equations for a 2-link planar arm with uniform rods; 3-link mass matrix must be symmetric, positive-definite, and coupled across all joints.
- **Method:** Evaluated $M(q)$ for 2-link arm ($l_1=1.0, l_2=0.8, m_1=1.5, m_2=1.2$) across 5 angles ($q_2 \in \{0, \pi/4, \pi/2, \pi, -\pi/3\}$). Tested symmetry ($M_{12} == M_{21}$), positive-definiteness via Sylvester's criterion, and exact match against analytical closed form ($M_{11}, M_{12}, M_{22}$). Evaluated 3-link arm ($n=3$) checking symmetry, positive-definiteness, and nonzero off-diagonal elements.
- **Expected behavior:** $M_{12} == M_{21}$, all leading principal minors $> 0$, analytical difference $< 10^{-12}$, 3-link off-diagonals $> 10^{-4}$.
- **Observed behavior:** Differences from textbook analytical formulas were $< 10^{-15}$; all minors strictly positive; 3-link coupling confirmed.
- **Result:** **PASS**

### 2. Coriolis & Centrifugal Coupling
- **Requirement/risk being tested:** Velocity-dependent coupling $C(q, \dot{q})\dot{q}$ must allow motion of one joint to push on the others; must match 2-link analytical textbook equations; 3-link arm must couple across all 3 joints.
- **Method:**
  - Evaluated 2-link arm with $q=[0.3, \pi/4], \dot{q}=[1.5, -2.0]$ against closed-form expressions for $c_1$ and $c_2$.
  - Tested joint 1 rotation alone ($\dot{q}_1 = 2.0, \dot{q}_2 = 0.0$) and verified induced load on joint 2 ($c_2 \ne 0$).
  - Tested 3-link arm with only joint 1 rotating ($\dot{q}=[1, 0, 0]$) and verified centrifugal coupling to joints 2 and 3.
- **Expected behavior:** Exact match to analytical formulas; nonzero coupling terms ($|c_2| > 10^{-4}$, $|C_3[1]| > 10^{-4}, |C_3[2]| > 10^{-4}$).
- **Observed behavior:** Differences $< 10^{-12}$; coupling forces verified on joint 2 and joint 3.
- **Result:** **PASS**

### 3. Gravity Load Vector G(q)
- **Requirement/risk being tested:** $G(q)$ matches potential energy gradient for uniform rods with gravity along world $-y$; vertical equilibria have zero gravity load; holding torque $\tau = G(q)$ results in exactly zero acceleration.
- **Method:**
  - Compared numerical $G(q)$ for 2-link arm against analytical formulas at $q=[0.4, 0.6]$.
  - Checked hanging pose ($\phi_1 = -\pi/2, \phi_2 = -\pi/2$) and upward vertical pose ($\phi_1 = \pi/2, \phi_2 = \pi/2$).
  - Evaluated forward dynamics with $\tau = G(q)$ at $q=[0.3, -0.5], \dot{q}=[0, 0]$ and verified $\ddot{q} == 0$.
- **Expected behavior:** Analytical match $< 10^{-12}$; vertical poses yield $G_1 = 0, G_2 = 0$; $\ddot{q} == 0$ when holding against gravity.
- **Observed behavior:** Differences $< 10^{-14}$; vertical poses yielded $|G_i| < 10^{-15}$; $\ddot{q}$ was exactly zero ($< 10^{-15}$).
- **Result:** **PASS**

### 4. Free Motion Under Gravity
- **Requirement/risk being tested:** With $\tau = 0$ and nonzero gravity, a non-equilibrium pose must accelerate and move over time.
- **Method:** Initialized horizontal 2-link arm ($q=[0, 0], \dot{q}=[0, 0], \tau=[0, 0]$). Checked $\ddot{q}_1 < -0.1$ (downward acceleration). Integrated 50 steps ($dt=0.01$) using RK4 and verified state displacement.
- **Expected behavior:** Initial $\ddot{q}_1 < 0$; after 0.5s, $|q_1 - q_{start}| > 0.05$ and $|\dot{q}_1| > 0.05$.
- **Observed behavior:** Initial $\ddot{q}_1 = -16.2\ \text{rad/s}^2$; after 0.5s, $q_1 = -0.58\ \text{rad}$ and $\dot{q}_1 = -1.69\ \text{rad/s}$.
- **Result:** **PASS**

### 5. Linear Solver Accuracy & Pivot Handling
- **Requirement/risk being tested:** Gaussian elimination with partial pivoting accurately solves $A x = b$ and rejects singular matrices.
- **Method:** Solved a known $3 \times 3$ system with known integer solution $x=[2, 3, -1]$. Tested a singular matrix and verified that `std::runtime_error` is thrown.
- **Expected behavior:** Solution error $< 10^{-12}$; singular matrix detected and exception thrown.
- **Observed behavior:** Errors $< 10^{-14}$; singular matrix threw `std::runtime_error`.
- **Result:** **PASS**

### 6. Forward Kinematics Geometry
- **Requirement/risk being tested:** Correct joint and end-effector positions and headings for 2-link and 3-link arms.
- **Method:** Tested fully extended along $+x$, right-angle elbow ($q_2 = \pi/2$), fully folded ($q_2 = \pi$), and 3-link multi-joint position array.
- **Expected behavior:** Exact geometrical coordinates matching link lengths and angles.
- **Observed behavior:** All positions and orientations verified to $< 10^{-12}$.
- **Result:** **PASS**

### 7. Task 1 Regression Test (Post-Refactoring)
- **Requirement/risk being tested:** Separation of Task 1 code into `include/pendularm/*.hpp` and `src/*.cpp` did not introduce regressions.
- **Method:** Re-ran all Task 1 test suites in `tests/test_integrators.cpp`.
- **Expected behavior:** All 6 test suites pass with identical numerical precision.
- **Observed behavior:** All tests passed with 0 errors.
- **Result:** **PASS**

## Failures

None.

## Remaining gaps

Tasks 3 through 8 will introduce the live arm simulation node, PID controller, IK solver, IK action server, and trial harness.
