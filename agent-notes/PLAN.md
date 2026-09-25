# Plan: Numerical Integration Library and `/arm_sim/integration_step` (Task 1)

## Goal

Implement the four required numerical integrators (`euler`, `midpoint`, `verlet`, `rk4`) and the standalone `/arm_sim/integration_step` service over the rosbridge TCP/JSON gateway (127.0.0.1:9095), with an expression parser for $f(t)$ that supports all specified arithmetic, functions, and precedence, strictly adhering to the Project 2 specification.

## Relevant requirements

1. **Four numerical integrators**:
   - `euler`: Forward Euler, 1 acceleration evaluation per step.
     $q_{n+1} = q_n + \dot{q}_n \cdot dt$, $\dot{q}_{n+1} = \dot{q}_n + \ddot{q}_n \cdot dt$.
   - `midpoint`: 2 acceleration evaluations per step (current state and trial half-step state), advances using midpoint derivatives.
     $a_0 = f(t_n, q_n, \dot{q}_n)$
     $q_{mid} = q_n + \dot{q}_n \cdot \frac{dt}{2}$, $\dot{q}_{mid} = \dot{q}_n + a_0 \cdot \frac{dt}{2}$
     $a_{mid} = f(t_n + \frac{dt}{2}, q_{mid}, \dot{q}_{mid})$
     $q_{n+1} = q_n + \dot{q}_{mid} \cdot dt$, $\dot{q}_{n+1} = \dot{q}_n + a_{mid} \cdot dt$.
   - `verlet`: Velocity Verlet with predictor-corrector (for velocity-dependent accelerations), 2 acceleration evaluations per step.
     $a_0 = f(t_n, q_n, \dot{q}_n)$
     $q_{n+1} = q_n + \dot{q}_n \cdot dt + \frac{1}{2} a_0 \cdot dt^2$
     $\dot{q}_{pred} = \dot{q}_n + a_0 \cdot dt$
     $a_1 = f(t_n + dt, q_{n+1}, \dot{q}_{pred})$
     $\dot{q}_{n+1} = \dot{q}_n + \frac{1}{2}(a_0 + a_1) \cdot dt$.
   - `rk4`: Classical 4th-order Runge-Kutta, 4 acceleration evaluations blended with 1:2:2:1 weights.
     $k_{1,v} = f(t_n, q_n, \dot{q}_n)$, $k_{1,q} = \dot{q}_n$
     $k_{2,v} = f(t_n + \frac{dt}{2}, q_n + k_{1,q} \frac{dt}{2}, \dot{q}_n + k_{1,v} \frac{dt}{2})$, $k_{2,q} = \dot{q}_n + k_{1,v} \frac{dt}{2}$
     $k_{3,v} = f(t_n + \frac{dt}{2}, q_n + k_{2,q} \frac{dt}{2}, \dot{q}_n + k_{2,v} \frac{dt}{2})$, $k_{3,q} = \dot{q}_n + k_{2,v} \frac{dt}{2}$
     $k_{4,v} = f(t_n + dt, q_n + k_{3,q} dt, \dot{q}_n + k_{3,v} dt)$, $k_{4,q} = \dot{q}_n + k_{3,v} dt$
     $q_{n+1} = q_n + \frac{dt}{6}(k_{1,q} + 2 k_{2,q} + 2 k_{3,q} + k_{4,q})$
     $\dot{q}_{n+1} = \dot{q}_n + \frac{dt}{6}(k_{1,v} + 2 k_{2,v} + 2 k_{3,v} + k_{4,v})$.
2. **Sub-stage evaluation times**:
   - Each stage must receive its exact fractional time ($t$, $t+dt/2$, $t+dt$). Time must never be frozen at $t$.
3. **Reusability**:
   - The integrator API must operate on vector-valued second-order systems $\ddot{q} = f(t, q, \dot{q})$ where $q, \dot{q} \in \mathbb{R}^n$, so Task 1 (1-DOF particle) and future tasks (2-link and 3-link arm dynamics) use identical integration code without modification.
4. **Expression parser**:
   - Free variable: `t`
   - Binary ops: `+`, `-` (precedence 1, left-associative), `*`, `/` (precedence 2, left-associative), `^` (precedence 3, right-associative)
   - Unary op: prefix `-` (precedence 4, higher than `^`), and prefix `+`
   - Atoms: numeric literals (including scientific notation and decimals), variable `t`, parentheses `(...)`, named single-argument functions: `sin`, `cos`, `tan`, `exp`, `sqrt`, `ln`, `abs`.
   - Rejection: Malformed expressions (empty, unbalanced parens, unknown identifiers, trailing tokens, missing operands, adjacent tokens without operator) must be rejected cleanly without throwing unhandled exceptions or crashing.
   - Domain errors during evaluation (e.g. `sqrt(-1)`) may return `NaN` or `inf` per specification.
5. **Service `/arm_sim/integration_step`**:
   - Request: `{"function": "<expr>", "x0": <f64>, "xdot0": <f64, optional default 0.0>, "dt": <f64>, "steps": <u64>, "integrator": "euler"|"midpoint"|"verlet"|"rk4"}`
   - Reject cleanly with `result: false` and non-empty `status` message when:
     - `function` fails to parse
     - `integrator` is not one of the four supported strings
     - `dt <= 0` or non-finite
     - `steps == 0` or non-integer / negative
     - missing required parameters or malformed JSON
   - On success: `result: true`, `status: ""`, `values`:
     `{"times": [...], "positions": [...], "velocities": [...]}`
     Array length: `steps + 1`.
     `times[0] == 0.0`, `positions[0] == x0`, `velocities[0] == xdot0`.
     Each index $k \in [0, steps]$ corresponds to $t = k \cdot dt$.
   - Safe isolation: decoupling from live arm states; service errors must never crash or destabilize the runtime.
6. **No external libraries**:
   - No physics engine, no external ODE solver, no ROS/ROS 2 libraries. Use standard C++20 and the project's included `middleware`.

## Repository observations

- `include/middleware/json.hpp`: Full JSON parser/serializer available.
- `include/middleware/middleware.hpp`: `Middleware` class providing topic pub/sub and service advertisement/calls.
- `include/middleware/rosbridge_server.hpp`: TCP server implementing rosbridge v2 protocol on `127.0.0.1:9095`, with `service_response` envelope `{"op":"service_response", "service":..., "values":..., "result":..., "status":..., "id":...}`.
- `tests/test_middleware.cpp`: Working middleware test suite.
- Top-level `Makefile` needs updating to remove project 1 targets and support Project 2 (`build`, `run`, `clean`, `test`, `test_integrators`).

## Proposed architecture

Create three modular, reusable headers in `include/pendularm/`:
1. `include/pendularm/integrator.hpp`:
   - `enum class IntegratorType { Euler, Midpoint, Verlet, Rk4 };`
   - State representation: `struct SecondOrderState { std::vector<double> q; std::vector<double> qdot; };`
   - Vector math helpers: addition, scaling, linear combination.
   - Core integrator functions:
     - `SecondOrderState step_euler(...)`
     - `SecondOrderState step_midpoint(...)`
     - `SecondOrderState step_verlet(...)`
     - `SecondOrderState step_rk4(...)`
     - `SecondOrderState integrate_step(IntegratorType type, const AccelFunc& f, double t, const SecondOrderState& state, double dt);`
   - String mapping helper: `std::optional<IntegratorType> parse_integrator_type(const std::string& name);`
2. `include/pendularm/expression.hpp`:
   - Tokenizer / Lexer for tokens: Number, Identifier, `+`, `-`, `*`, `/`, `^`, `(`, `)`.
   - AST node hierarchy:
     - `NumberNode(double val)`
     - `VariableNode(char var)`
     - `UnaryOpNode(char op, NodePtr child)`
     - `BinaryOpNode(char op, NodePtr left, NodePtr right)`
     - `FunctionCallNode(string name, NodePtr arg)`
   - Pratt / Precedence-climbing recursive descent parser with exact precedence levels and error reporting.
   - Evaluator: `double evaluate(double t) const;`
3. `include/pendularm/integration_service.hpp`:
   - `std::pair<bool, middleware::JsonValue> handle_integration_step(const middleware::JsonValue& args);`
   - Helper to register service with `middleware::Middleware`.
4. `src/main.cpp`:
   - Entry point that sets up `Middleware`, registers `/arm_sim/integration_step`, starts `RosbridgeServer` on `127.0.0.1:9095`, and handles shutdown signals cleanly.
5. `tests/test_integrators.cpp`:
   - Comprehensive test suite for all integrators, convergence rates, expression parser valid/invalid cases, and rosbridge service invocation.

## Interfaces and data flow

```text
External TCP Client / In-process caller
                 |
       [call_service request]
                 v
     RosbridgeServer / Middleware
                 |
                 v
   integration_service::handle_integration_step
                 |
     +-----------+-----------+
     |                       |
[Validate args]       [Parse function expr]
     |                       |
     |                       v
     |                AST (Expression)
     |                       |
     v                       v
Reusable Integrator loop (N steps)
     |
     v
{"times": [...], "positions": [...], "velocities": [...]}
     |
     v
Service response with result: true, status: ""
```

## Implementation steps

1. **Header: `include/pendularm/integrator.hpp`**:
   - Implement vector math operations.
   - Implement `step_euler`, `step_midpoint`, `step_verlet`, `step_rk4`.
   - Implement `integrate_step` and `parse_integrator_type`.
2. **Header: `include/pendularm/expression.hpp`**:
   - Implement tokenization, AST classes, and recursive descent parser.
   - Support `sin`, `cos`, `tan`, `exp`, `sqrt`, `ln`, `abs`, binary `+`, `-`, `*`, `/`, `^`, unary `-`/`+`, numbers, variable `t`.
   - Thoroughly validate syntax, rejecting malformed expressions with informative error messages.
3. **Header: `include/pendularm/integration_service.hpp`**:
   - Parse request arguments from `JsonValue`.
   - Enforce constraints: `dt > 0`, `steps > 0`, valid integrator, valid expression, optional `xdot0` (default 0.0).
   - Execute integration loop recording $(t, x, \dot{x})$ at each step.
   - Return `{true, values}` or `{false, err}`.
4. **Runtime Entry Point: `src/main.cpp`**:
   - Initialize `Middleware`, register `/arm_sim/integration_step`, start `RosbridgeServer` on port 9095.
   - Block until SIGINT/SIGTERM.
5. **Update `Makefile`**:
   - Targets: `build`, `run`, `clean`, `test`, `test_integrators`, `test_middleware`.
6. **Tests: `tests/test_integrators.cpp`**:
   - Analytical test cases with known closed-form solutions:
     - $f(t) = 0 \implies x(t) = x_0 + \dot{x}_0 t$
     - $f(t) = a \implies x(t) = x_0 + \dot{x}_0 t + \frac{1}{2} a t^2$
     - $f(t) = t \implies x(t) = x_0 + \dot{x}_0 t + \frac{1}{6} t^3$ (checks sub-stage time handling!)
     - $f(t) = \cos(t) \implies x(t) = 1 - \cos(t)$ from rest
   - Verification of convergence order:
     - Euler: $O(dt)$
     - Midpoint: $O(dt^2)$
     - Verlet: $O(dt^2)$
     - RK4: $O(dt^4)$
   - Error rejection tests:
     - Invalid expressions: empty, unknown tokens, unclosed parens, adjacent tokens, missing operands.
     - Invalid service parameters: `dt <= 0`, `steps == 0`, bad integrator name, missing fields.
   - End-to-end integration via Middleware and Rosbridge TCP socket.

## Edge cases and failure modes

- **Time-varying functions**: Sub-stage times must be updated at half-step and full-step ($t + dt/2$, $t + dt$). If time is frozen at $t$, midpoint and RK4 lose their high-order convergence.
- **Operator precedence and associativity**: `^` is right-associative (`2^3^2 = 512`), unary `-` has higher precedence than `^` per specification (`-t^2 = (-t)^2`).
- **Malformed expressions**: Must return `result: false` with descriptive error; must not throw unhandled exceptions or crash.
- **Large step counts**: Avoid unnecessary memory reallocations by pre-allocating output vectors with `reserve(steps + 1)`.
- **Floating point domain errors**: `sqrt(-1)` producing `NaN` is permitted per specification and must not abort execution.

## Open questions and assumptions

- Variable name in expressions is strictly `t` (case-sensitive lowercase `t`).
- `steps` must be a positive integer $\ge 1$.
- `dt` must be strictly positive $> 0$.
- Unit-mass 1-DOF particle implies acceleration $\ddot{q} = f(t)$.

## Verification strategy

- Run standalone unit tests verifying:
  - Each integrator against analytical solutions ($f(t)=0, c, t, \cos(t), \exp(t)$).
  - Accuracy hierarchy: RK4 < Midpoint/Verlet < Euler for $f(t) = t$ and $f(t) = \cos(t)$.
  - Sub-stage time verification on $f(t) = t$.
  - Parser coverage: valid complex expressions, right-associativity of `^`, precedence of unary `-`, all math functions.
  - Rejection coverage: all malformed expressions, non-positive `dt`, zero `steps`, unknown integrator.
  - TCP Rosbridge round-trip service call.
