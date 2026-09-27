# PENDULARM: Planar Arm Dynamics, Kinematics & Interactive Visualizer

A modular robotics runtime and interactive visualization studio for $n$-link planar pendulums and robotic manipulators (2-link and 3-link configurations).

Features closed-form Lagrangian dynamics, numerical integrators (RK4, Velocity Verlet, Midpoint, Euler), PID joint controllers, analytical inverse kinematics (IK), an IK action server with dwell tolerance verification, an automated IK trial benchmark harness, and a real-time web front-end.

---

## Quick Start

### 1. Launch the Interactive Web Visualizer
Launch the backend and open the front-end in your browser with a single command:

```bash
make visualizer
# or
make gui
```

This will automatically:
1. Compile `build/arm_sim` if not already built.
2. Launch the backend simulation runtime listening on `127.0.0.1:9095`.
3. Launch the multi-threaded HTTP/WebSocket bridge server on `http://127.0.0.1:8080`.
4. Open the interactive dashboard in your default browser.

To launch in 3-link mode directly from the command line:
```bash
python3 scripts/visualizer.py --links 3
```
*(You can also switch seamlessly between 2-link and 3-link modes at any time directly in the web UI navbar!)*

---

## Visualizer Capabilities & Features

### 1. Interactive 2D Simulation Canvas
- **Physics Rendering**: Sleek, high-DPI rendering of the base mount, physical links with mass centers, joint angle arcs, and glowing end-effector flange.
- **Motion Trails**: Fading rainbow path of the end-effector trajectory to visualize chaotic swings, limit cycles, and tracking paths.
- **Reachable Workspace**: Dynamic boundary circles indicating outer reach ($R = \sum l_i$) and inner kinematic singularities.
- **Interactive Drag & Target**: Click or drag directly on the canvas to set Cartesian targets, preview the analytical IK ghost arm in real time, and command goals.
- **Pan & Zoom**: Scroll wheel to zoom in/out, right-click/middle-click drag to pan, and quick view reset buttons.

### 2. Live Telemetry Strip Charts
- **Joint Angles ($q$)**: Real-time plots for each joint angle in radians/degrees.
- **Joint Velocities ($\dot{q}$)**: Real-time velocity tracking.
- **Actuator Torques ($\tau$)**: Command effort delivered by the joint PID controllers.
- **Energy Conservation ($T, V, E$)**: Live strip charts tracking Kinetic ($T$), Potential ($V$), and Total Mechanical Energy ($E = T + V$) to observe Hamiltonian conservation under Verlet/RK4 versus numerical drift under Euler.

### 3. Simulation & Dynamics Deck
- **Transport Controls**: Play / Pause toggle (`Space`), single physics step, and simulation reset (`R`).
- **Integrator Selector**: Switch dynamically between `rk4` (4th-order Runge-Kutta), `verlet` (symplectic Velocity Verlet), `midpoint` (2nd-order), and `euler` (1st-order explicit).
- **Physical Environment**:
  - Live gravity slider with presets: Zero-G ($g=0$), Moon ($1.62$), Earth ($9.81$), Jupiter ($24.79$).
  - Per-link length sliders ($l_1, l_2, l_3$).
  - Per-link mass sliders ($m_1, m_2, m_3$).
- **Drop Presets**:
  - *Double Pendulum Chaos*: Release from horizontal extension with zero torque to observe non-linear chaotic motion.
  - *Inverted Standup*: Balance from near-vertical upright state.
  - *Bottom Equilibrium*: Rest state hanging straight down.
  - *Perturbation Kick*: Momentary velocity impulse.

### 4. PID Controller Deck
- **Master Toggle**: Enable/disable joint effort controllers.
- **Joint Setpoints**: Sliders to command target joint positions via `/joint_trajectory`.
- **Gain Tuning**: Independent sliders for $K_p$, $K_i$, and $K_d$ per joint, with presets for stiff tracking, compliant hold, and critical damping.

### 5. Inverse Kinematics (IK) Deck
- **Analytical Solver**: Closed-form solution for $(x, y)$ and end-effector orientation $\phi$.
- Solves joint angles $[q_1, q_2, q_3]$ with reachability validation and workspace boundary checks.
- One-click button to drive the arm to the solved configuration.

### 6. IK Action Server Deck
- Commands the arm to Cartesian goals with customizable tolerance $\epsilon$ and dwell hold duration (`success_hold`).
- Real-time feedback with distance remaining, elapsed time, and tolerance dwell indicator.
- Result cards reporting final status (`reached`, `preempted`, `cancelled`).

### 7. IK Trial Benchmark Deck
- Automated benchmark sequence testing target acquisition throughput over a fixed duration.
- Real-time scoreboard reporting targets reached, live countdown progress bar, target coordinates, and current tracking error.
- Controls to start, skip active target, or stop trial.

### 8. ODE Step Playground
- Evaluates arbitrary user-defined differential equations $\ddot{x} = f(t, x, v)$ via the expression AST parser and numerical integrators.
- Single-step and 500-step batch simulation with phase state tracking.

---

## Command-Line & Test Interface

### Build & Run Headless Runtime
```bash
make build   # Compiles build/arm_sim
make run     # Runs C++ rosbridge runtime on 127.0.0.1:9095 (ARM_SIM_LINKS=2)
ARM_SIM_LINKS=3 make run  # Runs in 3-link mode
```

### Run Test Suites
```bash
make test                     # Runs all 9 C++ test suites
python3 tests/test_visualizer_e2e.py  # Runs visualizer end-to-end integration test
```
