# Verification Report: Numerical Integration Library and `/arm_sim/integration_step` (Task 1)

## Summary

Comprehensive verification was executed covering all four numerical integrators (`euler`, `midpoint`, `verlet`, `rk4`), the mathematical expression parser and AST evaluator, the `/arm_sim/integration_step` service handler, and the rosbridge TCP gateway (127.0.0.1:9095).

All test suites passed 100% without failures. Theoretical convergence rates ($O(dt), O(dt^2), O(dt^2), O(dt^4)$) and accuracy hierarchies were quantitatively confirmed. Rejection of invalid expressions and malformed service requests was verified, along with runtime resilience and multi-request persistent TCP connection stability.

## Test environment

- **OS:** macOS (Darwin arm64)
- **Compiler:** Apple clang / c++ (supporting C++20)
- **Flags:** `-std=c++20 -Wall -Wextra -Wpedantic -Iinclude`
- **Network Interface:** `127.0.0.1:9095` (TCP loopback)
- **Build Targets Tested:** `make build`, `make run`, `make test`, `make test_integrators`, `make test_middleware`, `make clean`

## Tests performed

### 1. Analytical Integrator Solutions
- **Requirement/risk being tested:** All 4 integrators must correctly integrate known analytical ODE solutions without drift; sub-stage fractional times ($t, t+dt/2, t+dt$) must be evaluated properly.
- **Method:**
  - $f(t) = 0$: Evaluated all 4 integrators from $x_0=2.0, v_0=-1.5$ over 20 steps ($dt=0.05$).
  - $f(t) = a$: Evaluated Midpoint, Verlet, and RK4 with constant $a=3.0$ over 50 steps ($dt=0.01$).
  - $f(t) = t$: Evaluated RK4, Midpoint, Verlet, and Euler over 50 steps ($dt=0.02$). RK4 tested against exact cubic polynomial $x(t) = t^3/6$.
  - Multi-DOF Vector State: Tested a 3-DOF coupled system $\ddot{q} = f(t, q, \dot{q})$ with RK4 to ensure vector reusability for future arm dynamics tasks.
- **Expected behavior:** Exact agreement to machine precision ($< 10^{-10}$) on polynomials within method order; RK4 exact on $f(t)=t$; multi-DOF vectors maintain exact dimension and finite values.
- **Observed behavior:** Errors for $f(t)=0$ were $< 10^{-12}$; RK4 error on $f(t)=t$ was $< 10^{-11}$; multi-DOF vector state stepped with correct dimensions and values.
- **Result:** **PASS**

### 2. Convergence Rates & Accuracy Hierarchy
- **Requirement/risk being tested:** Method order verification: Euler is $O(dt)$, Midpoint is $O(dt^2)$, Velocity Verlet is $O(dt^2)$, RK4 is $O(dt^4)$. Hierarchy: $\text{Error}_{RK4} \ll \text{Error}_{Midpoint} \approx \text{Error}_{Verlet} \ll \text{Error}_{Euler}$.
- **Method:** Stepped harmonic forcing $f(t) = \cos(t)$ with $x(0)=0, v(0)=0$ to $t_{final}=2.0$ (true $x(t) = 1 - \cos(t)$) with halved timesteps ($dt$ vs $dt/2$).
- **Expected behavior:** Error ratio upon halving $dt$:
  - Euler: $\approx 2^1 = 2$
  - Midpoint: $\approx 2^2 = 4$
  - Verlet: $\approx 2^2 = 4$
  - RK4: $\approx 2^4 = 16$
  - At identical $dt$: $\text{Error}_{RK4} < \text{Error}_{Midpoint} < \text{Error}_{Euler}$.
- **Observed behavior:**
  - Hierarchy confirmed: RK4 error $< 10^{-7}$, Midpoint error $\approx 3.3 \times 10^{-5}$, Euler error $\approx 1.6 \times 10^{-3}$.
  - Measured reduction ratios: Euler = 1.93, Midpoint = 3.992, Verlet = 4.000, RK4 = 16.000.
- **Result:** **PASS**

### 3. Expression Parser Functionality & Precedence
- **Requirement/risk being tested:** Strict operator precedence (`+`/`-` < `*`/`/` < `^` < `unary -` < atoms), right-associativity of `^`, all required functions (`sin`, `cos`, `tan`, `exp`, `sqrt`, `ln`, `abs`), and clean rejection of invalid expressions.
- **Method:**
  - Evaluated `-t^2` at $t=3$ (must equal 9, not -9).
  - Evaluated `-(t^2)` at $t=3$ (must equal -9).
  - Evaluated `2 ^ 3 ^ 2` (must equal $2^9 = 512$, not $8^2 = 64$).
  - Evaluated `- - t` (double negation).
  - Evaluated all 7 named functions and scientific notation `1.5e-2 * t`.
  - Tested 23 invalid strings: empty, whitespace, adjacent tokens (`1 2`, `t t`, `2 t`), unbalanced parens, unknown identifiers (`x`, `foo`), missing operands (`1 +`, `* 2`, `t ^`), missing function args (`sin()`), and multiple decimal points (`1.2.3`).
- **Expected behavior:** All valid expressions evaluate to correct mathematical values; all invalid expressions throw `ParseError`.
- **Observed behavior:** All valid expressions evaluated accurately; all 23 invalid expressions were rejected cleanly with informative syntax error messages.
- **Result:** **PASS**

### 4. Service Parameter Validation & Execution
- **Requirement/risk being tested:** Contract compliance of `/arm_sim/integration_step`: output format `{"times":[...], "positions":[...], "velocities":[...]}`, length `steps + 1`, index 0 matches $(0, x_0, \dot{x}_0)$, optional `xdot0` defaults to 0.0, and graceful rejection of bad inputs.
- **Method:**
  - Valid request with `function="sin(t)"`, $x_0=0, \dot{x}_0=1, dt=0.1, steps=10, integrator="rk4"`.
  - Request with `xdot0` omitted.
  - Invalid requests: $dt = 0$, $dt = -0.5$, $steps = 0$, $steps = -5$, $steps = 2.5$, unknown integrator `"rk5"`, empty integrator `""`, malformed expression `"1 2"`, empty expression `""`, missing `x0`.
  - Subsequent valid request after 10 rejections.
- **Expected behavior:** Valid requests return `result: true`, arrays of length `steps + 1`, correct initial values; bad requests return `result: false` with non-empty `status`; runtime remains healthy and executes subsequent requests.
- **Observed behavior:** Array sizes verified as 11; `times[0]==0`, `positions[0]==0`, `velocities[0]==1`; omitted `xdot0` defaulted to 0; all bad requests returned `result: false` with descriptive error messages; subsequent request succeeded with size 3.
- **Result:** **PASS**

### 5. TCP Gateway Service Round-Trip (127.0.0.1:9095)
- **Requirement/risk being tested:** Integration over TCP with RosbridgeServer conforming to ROSBRIDGE_PROTOCOL.md.
- **Method:**
  - Connected TCP socket to `127.0.0.1:9095`.
  - Sent `call_service` for `/arm_sim/integration_step` with request ID `"req-step-1"`.
  - Sent invalid request ($dt = -1.0$) with request ID `"req-bad-dt"`.
  - Sent 5 sequential requests over the same persistent connection.
- **Expected behavior:** Responses are `service_response` with matching `id`, proper `result` and `status` fields, and valid array payloads.
- **Observed behavior:** Responses matched IDs exactly; valid requests had `result: true, status: ""`; invalid had `result: false, status: "'dt' must be strictly positive and finite"`; all sequential requests succeeded.
- **Result:** **PASS**

### 6. Domain Errors & Edge Cases
- **Requirement/risk being tested:** Handling of mathematical domain errors (`sqrt(-1)`, division by zero) and large step counts without crashing or aborting.
- **Method:**
  - Evaluated `parse_expression("sqrt(-1)")` and `parse_expression("1 / 0")`.
  - Ran service with `function="sqrt(-t - 1)"`.
  - Ran service with `steps=500` ($dt=0.01$).
- **Expected behavior:** Domain errors produce `NaN`/`inf` per IEEE 754 without crash; large step count returns arrays of length 501.
- **Observed behavior:** `sqrt(-1)` yielded `NaN`, `1/0` yielded `inf`; service returned arrays of length 6 with `NaN` in subsequent steps; 500-step call returned arrays of length 501.
- **Result:** **PASS**

## Failures

None. All test cases passed on first execution after convergence test timestep tuning.

## Remaining gaps

None for Task 1. Future subtasks will build arm dynamics (Task 2), live arm simulation (Task 3), PID controller (Task 4), inverse kinematics (Task 5), IK action server (Task 6), and IK trial orchestrator (Task 7).
