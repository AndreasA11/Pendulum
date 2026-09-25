#include <iostream>
#include <cassert>
#include <cmath>
#include <string>
#include <vector>
#include <thread>
#include <chrono>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>

#include "middleware/json.hpp"
#include "middleware/middleware.hpp"
#include "middleware/rosbridge_server.hpp"
#include "pendularm/integrator.hpp"
#include "pendularm/expression.hpp"
#include "pendularm/integration_service.hpp"

using namespace pendularm;
using namespace middleware;

#define ASSERT_TRUE(cond, msg) \
    do { \
        if (!(cond)) { \
            std::cerr << "Assertion FAILED: " << (msg) << " (" << __FILE__ << ":" << __LINE__ << ")\n"; \
            std::exit(1); \
        } \
    } while(0)

#define ASSERT_NEAR(a, b, tol, msg) \
    do { \
        if (std::fabs((a) - (b)) > (tol)) { \
            std::cerr << "Assertion FAILED: " << (msg) << " | " << (a) << " vs " << (b) \
                      << " (diff=" << std::fabs((a) - (b)) << ", tol=" << (tol) << ") at " \
                      << __FILE__ << ":" << __LINE__ << "\n"; \
            std::exit(1); \
        } \
    } while(0)

// -------------------------------------------------------------
// Test 1: Analytical Integrator Solutions
// -------------------------------------------------------------
void test_analytical_integrators() {
    std::cout << "[TEST] Analytical Integrator Solutions..." << std::endl;

    // Case A: Zero acceleration f(t) = 0 -> x(t) = x0 + v0 * t, v(t) = v0
    {
        double x0 = 2.0;
        double v0 = -1.5;
        double dt = 0.05;
        uint64_t steps = 20;

        auto zero_accel = [](double, const std::vector<double>&, const std::vector<double>&) -> std::vector<double> {
            return {0.0};
        };

        for (auto type : {IntegratorType::Euler, IntegratorType::Midpoint, IntegratorType::Verlet, IntegratorType::Rk4}) {
            SecondOrderState s{{x0}, {v0}};
            double t = 0.0;
            for (uint64_t i = 1; i <= steps; ++i) {
                s = integrate_step(type, zero_accel, t, s, dt);
                t = i * dt;
                double expected_x = x0 + v0 * t;
                double expected_v = v0;
                ASSERT_NEAR(s.q[0], expected_x, 1e-12, "Zero accel position mismatch for " + integrator_type_to_string(type));
                ASSERT_NEAR(s.qdot[0], expected_v, 1e-12, "Zero accel velocity mismatch for " + integrator_type_to_string(type));
            }
        }
    }

    // Case B: Constant acceleration f(t) = a -> x(t) = x0 + v0 * t + 0.5 * a * t^2, v(t) = v0 + a * t
    {
        double x0 = 1.0;
        double v0 = 2.0;
        double a = 3.0;
        double dt = 0.01;
        uint64_t steps = 50;

        auto const_accel = [a](double, const std::vector<double>&, const std::vector<double>&) -> std::vector<double> {
            return {a};
        };

        // Verlet, Midpoint, and RK4 must be exact to machine precision for constant acceleration
        for (auto type : {IntegratorType::Midpoint, IntegratorType::Verlet, IntegratorType::Rk4}) {
            SecondOrderState s{{x0}, {v0}};
            double t = 0.0;
            for (uint64_t i = 1; i <= steps; ++i) {
                s = integrate_step(type, const_accel, t, s, dt);
                t = i * dt;
                double expected_x = x0 + v0 * t + 0.5 * a * t * t;
                double expected_v = v0 + a * t;
                ASSERT_NEAR(s.q[0], expected_x, 1e-10, "Const accel position mismatch for " + integrator_type_to_string(type));
                ASSERT_NEAR(s.qdot[0], expected_v, 1e-10, "Const accel velocity mismatch for " + integrator_type_to_string(type));
            }
        }
    }

    // Case C: Linear in time f(t) = t -> x(t) = x0 + v0*t + t^3/6, v(t) = v0 + t^2/2
    // Specifically checks that sub-stage times (t, t + dt/2, t + dt) are properly evaluated!
    {
        double x0 = 0.0;
        double v0 = 0.0;
        double dt = 0.02;
        uint64_t steps = 50;

        auto linear_accel = [](double t, const std::vector<double>&, const std::vector<double>&) -> std::vector<double> {
            return {t};
        };

        // RK4 on degree-3 polynomial x(t) is exact to machine precision!
        {
            SecondOrderState s{{x0}, {v0}};
            double t = 0.0;
            for (uint64_t i = 1; i <= steps; ++i) {
                s = integrate_step(IntegratorType::Rk4, linear_accel, t, s, dt);
                t = i * dt;
                double expected_x = (t * t * t) / 6.0;
                double expected_v = (t * t) / 2.0;
                ASSERT_NEAR(s.q[0], expected_x, 1e-11, "RK4 f(t)=t position exactness");
                ASSERT_NEAR(s.qdot[0], expected_v, 1e-11, "RK4 f(t)=t velocity exactness");
            }
        }

        // Verify that Midpoint and Verlet sub-stage times are evaluated at t+dt/2 and t+dt
        {
            SecondOrderState s_mid{{x0}, {v0}};
            SecondOrderState s_ver{{x0}, {v0}};
            SecondOrderState s_eul{{x0}, {v0}};
            double t = 0.0;
            for (uint64_t i = 1; i <= steps; ++i) {
                s_mid = integrate_step(IntegratorType::Midpoint, linear_accel, t, s_mid, dt);
                s_ver = integrate_step(IntegratorType::Verlet, linear_accel, t, s_ver, dt);
                s_eul = integrate_step(IntegratorType::Euler, linear_accel, t, s_eul, dt);
                t = i * dt;
            }
            double final_t = steps * dt;
            double true_x = (final_t * final_t * final_t) / 6.0;
            double err_eul = std::fabs(s_eul.q[0] - true_x);
            double err_mid = std::fabs(s_mid.q[0] - true_x);
            double err_ver = std::fabs(s_ver.q[0] - true_x);

            // Euler must have significantly higher error than Midpoint and Verlet
            ASSERT_TRUE(err_mid < err_eul * 0.1, "Midpoint must be much more accurate than Euler on f(t)=t");
            ASSERT_TRUE(err_ver < err_eul * 0.1, "Verlet must be much more accurate than Euler on f(t)=t");
        }
    }

    // Case D: Multi-DOF Vector State (reusability for 2-link and 3-link arms)
    {
        SecondOrderState state_3d;
        state_3d.q = {1.0, 2.0, 3.0};
        state_3d.qdot = {0.1, -0.2, 0.3};
        auto multi_accel = [](double /*t*/, const std::vector<double>& q, const std::vector<double>& qdot) -> std::vector<double> {
            return {
                -2.0 * q[0] + 0.5 * qdot[1],
                -1.5 * q[1] + 0.2 * qdot[0],
                -3.0 * q[2]
            };
        };

        SecondOrderState next = integrate_step(IntegratorType::Rk4, multi_accel, 0.0, state_3d, 0.01);
        ASSERT_TRUE(next.q.size() == 3, "Multi-DOF q size must be 3");
        ASSERT_TRUE(next.qdot.size() == 3, "Multi-DOF qdot size must be 3");
        ASSERT_TRUE(std::isfinite(next.q[0]) && std::isfinite(next.q[1]) && std::isfinite(next.q[2]), "Multi-DOF states must be finite");
    }

    std::cout << "  -> PASSED" << std::endl;
}

// -------------------------------------------------------------
// Test 2: Convergence Order Verification (Euler: O(dt), Midpoint/Verlet: O(dt^2), RK4: O(dt^4))
// -------------------------------------------------------------
void test_convergence_rates() {
    std::cout << "[TEST] Convergence Rates & Accuracy Hierarchy..." << std::endl;

    // Harmonic forcing f(t) = cos(t) with x(0) = 0, v(0) = 0.
    // True solution: v(t) = sin(t), x(t) = 1 - cos(t).
    double t_final = 2.0;
    double true_x = 1.0 - std::cos(t_final);

    auto accel_fn = [](double t, const std::vector<double>&, const std::vector<double>&) -> std::vector<double> {
        return {std::cos(t)};
    };

    auto compute_error = [&](IntegratorType type, double dt) -> double {
        uint64_t n_steps = static_cast<uint64_t>(std::round(t_final / dt));
        SecondOrderState s{{0.0}, {0.0}};
        double t = 0.0;
        for (uint64_t i = 1; i <= n_steps; ++i) {
            s = integrate_step(type, accel_fn, t, s, dt);
            t = i * dt;
        }
        return std::fabs(s.q[0] - true_x);
    };

    double dt_eul1 = 0.004;
    double dt_eul2 = 0.002;
    double err_eul1 = compute_error(IntegratorType::Euler, dt_eul1);
    double err_eul2 = compute_error(IntegratorType::Euler, dt_eul2);

    double dt_mid1 = 0.02;
    double dt_mid2 = 0.01;
    double err_mid1 = compute_error(IntegratorType::Midpoint, dt_mid1);
    double err_mid2 = compute_error(IntegratorType::Midpoint, dt_mid2);

    double err_ver1 = compute_error(IntegratorType::Verlet, dt_mid1);
    double err_ver2 = compute_error(IntegratorType::Verlet, dt_mid2);

    double err_rk41 = compute_error(IntegratorType::Rk4, dt_mid1);
    double err_rk42 = compute_error(IntegratorType::Rk4, dt_mid2);

    // Accuracy hierarchy at dt = 0.02: RK4 < Midpoint/Verlet < Euler
    double err_eul_cmp = compute_error(IntegratorType::Euler, 0.02);
    ASSERT_TRUE(err_rk41 < err_mid1, "RK4 must be more accurate than Midpoint");
    ASSERT_TRUE(err_mid1 < err_eul_cmp, "Midpoint must be more accurate than Euler");
    ASSERT_TRUE(err_ver1 < err_eul_cmp, "Verlet must be more accurate than Euler");

    // Euler ratio should be ~2 (O(dt))
    double ratio_eul = err_eul1 / err_eul2;
    ASSERT_TRUE(ratio_eul > 1.85 && ratio_eul < 2.15, "Euler error reduction ratio ~ 2");

    // Midpoint & Verlet ratio should be ~4 (O(dt^2))
    double ratio_mid = err_mid1 / err_mid2;
    ASSERT_TRUE(ratio_mid > 3.8 && ratio_mid < 4.2, "Midpoint error reduction ratio ~ 4");

    double ratio_ver = err_ver1 / err_ver2;
    ASSERT_TRUE(ratio_ver > 3.8 && ratio_ver < 4.2, "Verlet error reduction ratio ~ 4");

    // RK4 ratio should be ~16 (O(dt^4))
    double ratio_rk4 = err_rk41 / err_rk42;
    ASSERT_TRUE(ratio_rk4 > 15.0 && ratio_rk4 < 17.0, "RK4 error reduction ratio ~ 16");

    std::cout << "  -> PASSED" << std::endl;
}

// -------------------------------------------------------------
// Test 3: Expression Parser
// -------------------------------------------------------------
void test_expression_parser() {
    std::cout << "[TEST] Expression Parser Functionality & Precedence..." << std::endl;

    // 1. Basic atoms and variable
    {
        auto expr = parse_expression("t");
        ASSERT_NEAR(expr->evaluate(2.5), 2.5, 1e-12, "Variable 't'");

        auto expr_num = parse_expression("42.5");
        ASSERT_NEAR(expr_num->evaluate(0.0), 42.5, 1e-12, "Number constant");
    }

    // 2. Precedence: -t^2 must be (-t)^2 per specification: unary - has higher precedence than ^
    {
        auto expr = parse_expression("-t^2");
        // At t=3: (-3)^2 = 9
        ASSERT_NEAR(expr->evaluate(3.0), 9.0, 1e-12, "Unary - has higher precedence than ^: -t^2 == (-t)^2");

        auto expr_parens = parse_expression("-(t^2)");
        // At t=3: -(3^2) = -9
        ASSERT_NEAR(expr_parens->evaluate(3.0), -9.0, 1e-12, "Parentheses override: -(t^2) == -9");
    }

    // 3. Right-associativity of exponentiation: 2^3^2 = 2^(3^2) = 2^9 = 512
    {
        auto expr = parse_expression("2 ^ 3 ^ 2");
        ASSERT_NEAR(expr->evaluate(0.0), 512.0, 1e-12, "Right-associativity of ^");
    }

    // 4. Double unary minus: - - t = t
    {
        auto expr = parse_expression("- - t");
        ASSERT_NEAR(expr->evaluate(7.0), 7.0, 1e-12, "Double negation - - t");
    }

    // 5. Arithmetic operations + - * / and polynomial
    {
        auto expr = parse_expression("t^2 + 3*t - 1");
        // At t=2: 4 + 6 - 1 = 9
        ASSERT_NEAR(expr->evaluate(2.0), 9.0, 1e-12, "Polynomial t^2 + 3*t - 1");

        auto expr_div = parse_expression("t / 2 + 1");
        ASSERT_NEAR(expr_div->evaluate(4.0), 3.0, 1e-12, "Division t / 2 + 1");
    }

    // 6. Named math functions: sin, cos, tan, exp, sqrt, ln, abs
    {
        auto expr_sin = parse_expression("sin(t)");
        ASSERT_NEAR(expr_sin->evaluate(M_PI / 2.0), 1.0, 1e-12, "sin(pi/2)");

        auto expr_cos = parse_expression("cos(t)");
        ASSERT_NEAR(expr_cos->evaluate(0.0), 1.0, 1e-12, "cos(0)");

        auto expr_tan = parse_expression("tan(0)");
        ASSERT_NEAR(expr_tan->evaluate(0.0), 0.0, 1e-12, "tan(0)");

        auto expr_exp = parse_expression("exp(1)");
        ASSERT_NEAR(expr_exp->evaluate(0.0), std::exp(1.0), 1e-12, "exp(1)");

        auto expr_sqrt = parse_expression("sqrt(16)");
        ASSERT_NEAR(expr_sqrt->evaluate(0.0), 4.0, 1e-12, "sqrt(16)");

        auto expr_ln = parse_expression("ln(exp(2))");
        ASSERT_NEAR(expr_ln->evaluate(0.0), 2.0, 1e-12, "ln(exp(2))");

        auto expr_abs = parse_expression("abs(-5.5)");
        ASSERT_NEAR(expr_abs->evaluate(0.0), 5.5, 1e-12, "abs(-5.5)");
    }

    // 7. Scientific notation
    {
        auto expr_sci = parse_expression("1.5e-2 * t");
        ASSERT_NEAR(expr_sci->evaluate(100.0), 1.5, 1e-12, "Scientific notation 1.5e-2");
    }

    // 8. Rejection of invalid expressions
    std::vector<std::string> invalid_expressions = {
        "",
        "   ",
        "1 2",
        "t t",
        "2 t",
        "t 2",
        "(",
        ")",
        "sin(t",
        "(t))",
        ")( ",
        "x",
        "foo(t)",
        "sin",
        "sin()",
        "sin(1, 2)",
        "t + 1 junk",
        "@",
        "$",
        "1 +",
        "* 2",
        "t ^",
        "1.2.3"
    };

    for (const auto& invalid : invalid_expressions) {
        bool threw = false;
        try {
            parse_expression(invalid);
        } catch (const ParseError&) {
            threw = true;
        }
        ASSERT_TRUE(threw, "Parser must reject invalid expression: '" + invalid + "'");
    }

    std::cout << "  -> PASSED" << std::endl;
}

// -------------------------------------------------------------
// Test 4: In-process Service Call Handling & Parameter Validation
// -------------------------------------------------------------
void test_service_validation() {
    std::cout << "[TEST] Service Parameter Validation & Execution..." << std::endl;

    Middleware mw;
    register_integration_service(mw);

    // 1. Valid request
    {
        JsonValue req = JsonValue::make_object();
        req["function"] = "sin(t)";
        req["x0"] = 0.0;
        req["xdot0"] = 1.0;
        req["dt"] = 0.1;
        req["steps"] = 10;
        req["integrator"] = "rk4";

        auto [ok, values] = mw.call_service("/arm_sim/integration_step", req);
        ASSERT_TRUE(ok, "Valid service call must return ok=true");
        ASSERT_TRUE(values.is_object(), "Values must be an object");
        ASSERT_TRUE(values.contains("times") && values["times"].is_array(), "Must contain 'times' array");
        ASSERT_TRUE(values.contains("positions") && values["positions"].is_array(), "Must contain 'positions' array");
        ASSERT_TRUE(values.contains("velocities") && values["velocities"].is_array(), "Must contain 'velocities' array");

        const auto& times = values["times"].as_array();
        const auto& positions = values["positions"].as_array();
        const auto& velocities = values["velocities"].as_array();

        ASSERT_TRUE(times.size() == 11, "Array length must be steps + 1 = 11");
        ASSERT_TRUE(positions.size() == 11, "Positions length must be 11");
        ASSERT_TRUE(velocities.size() == 11, "Velocities length must be 11");

        ASSERT_NEAR(times[0].as_number(), 0.0, 1e-12, "times[0] == 0");
        ASSERT_NEAR(positions[0].as_number(), 0.0, 1e-12, "positions[0] == x0");
        ASSERT_NEAR(velocities[0].as_number(), 1.0, 1e-12, "velocities[0] == xdot0");
    }

    // 2. Optional xdot0 defaults to 0.0
    {
        JsonValue req = JsonValue::make_object();
        req["function"] = "t";
        req["x0"] = 5.0;
        // xdot0 omitted
        req["dt"] = 0.05;
        req["steps"] = 5;
        req["integrator"] = "midpoint";

        auto [ok, values] = mw.call_service("/arm_sim/integration_step", req);
        ASSERT_TRUE(ok, "Service call without xdot0 must succeed");
        const auto& velocities = values["velocities"].as_array();
        ASSERT_NEAR(velocities[0].as_number(), 0.0, 1e-12, "Omitted xdot0 must default to 0.0");
    }

    // 3. Rejection cases (dt <= 0, steps == 0, bad integrator, bad function, missing fields)
    std::vector<std::pair<std::string, JsonValue>> bad_requests;

    // Bad dt <= 0
    {
        JsonValue req = JsonValue::make_object();
        req["function"] = "t"; req["x0"] = 0.0; req["dt"] = 0.0; req["steps"] = 10; req["integrator"] = "euler";
        bad_requests.emplace_back("dt == 0", req);

        req["dt"] = -0.5;
        bad_requests.emplace_back("dt < 0", req);
    }

    // Bad steps == 0 or negative
    {
        JsonValue req = JsonValue::make_object();
        req["function"] = "t"; req["x0"] = 0.0; req["dt"] = 0.1; req["steps"] = 0; req["integrator"] = "euler";
        bad_requests.emplace_back("steps == 0", req);

        req["steps"] = -5;
        bad_requests.emplace_back("steps < 0", req);

        req["steps"] = 2.5; // non-integer
        bad_requests.emplace_back("non-integer steps", req);
    }

    // Bad integrator name
    {
        JsonValue req = JsonValue::make_object();
        req["function"] = "t"; req["x0"] = 0.0; req["dt"] = 0.1; req["steps"] = 10; req["integrator"] = "rk5";
        bad_requests.emplace_back("unknown integrator rk5", req);

        req["integrator"] = "";
        bad_requests.emplace_back("empty integrator name", req);
    }

    // Bad function expression
    {
        JsonValue req = JsonValue::make_object();
        req["function"] = "1 2"; req["x0"] = 0.0; req["dt"] = 0.1; req["steps"] = 10; req["integrator"] = "euler";
        bad_requests.emplace_back("malformed expression '1 2'", req);

        req["function"] = "";
        bad_requests.emplace_back("empty expression", req);
    }

    // Missing fields
    {
        JsonValue req = JsonValue::make_object();
        req["function"] = "t"; req["dt"] = 0.1; req["steps"] = 10; req["integrator"] = "euler";
        // missing x0
        bad_requests.emplace_back("missing x0", req);
    }

    for (const auto& [label, req] : bad_requests) {
        auto [ok, resp] = mw.call_service("/arm_sim/integration_step", req);
        ASSERT_TRUE(!ok, "Service must reject request: " + label);
        ASSERT_TRUE(resp.is_object() && resp.contains("error") && !resp["error"].as_string().empty(),
                    "Rejection must return non-empty error message: " + label);
    }

    // 4. Runtime resilience: Service continues to accept valid requests after rejections
    {
        JsonValue req = JsonValue::make_object();
        req["function"] = "cos(t)";
        req["x0"] = 1.0;
        req["dt"] = 0.1;
        req["steps"] = 2;
        req["integrator"] = "verlet";

        auto [ok, values] = mw.call_service("/arm_sim/integration_step", req);
        ASSERT_TRUE(ok, "Service must remain healthy and accept valid requests after rejected requests");
        ASSERT_TRUE(values["times"].as_array().size() == 3, "Valid request must succeed after errors");
    }

    std::cout << "  -> PASSED" << std::endl;
}

// -------------------------------------------------------------
// Test 5: TCP Gateway Rosbridge Service Protocol (127.0.0.1:9095)
// -------------------------------------------------------------
void test_tcp_rosbridge_service() {
    std::cout << "[TEST] TCP Gateway Service Round-Trip (127.0.0.1:9095)..." << std::endl;

    Middleware mw;
    register_integration_service(mw);

    RosbridgeServer server(mw, "127.0.0.1", 9095);
    ASSERT_TRUE(server.start(), "Server must start on 127.0.0.1:9095");

    int sock = socket(AF_INET, SOCK_STREAM, 0);
    ASSERT_TRUE(sock >= 0, "Socket creation");

    struct sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_port = htons(9095);
    inet_pton(AF_INET, "127.0.0.1", &addr.sin_addr);

    ASSERT_TRUE(connect(sock, reinterpret_cast<struct sockaddr*>(&addr), sizeof(addr)) == 0, "Connect to TCP server");

    auto send_json = [sock](const JsonValue& j) {
        std::string payload = j.dump() + "\n";
        send(sock, payload.data(), payload.size(), 0);
    };

    auto recv_json_line = [sock]() -> JsonValue {
        std::string line;
        char c;
        while (recv(sock, &c, 1, 0) == 1) {
            if (c == '\n') break;
            if (c != '\r') line += c;
        }
        return JsonValue::parse(line);
    };

    // 1. Send valid call_service request
    {
        JsonValue call = JsonValue::make_object();
        call["op"] = "call_service";
        call["service"] = "/arm_sim/integration_step";
        call["id"] = "req-step-1";

        JsonValue args = JsonValue::make_object();
        args["function"] = "t^2";
        args["x0"] = 0.0;
        args["xdot0"] = 0.0;
        args["dt"] = 0.1;
        args["steps"] = 5;
        args["integrator"] = "rk4";
        call["args"] = args;

        send_json(call);

        JsonValue resp = recv_json_line();
        ASSERT_TRUE(resp["op"].as_string() == "service_response", "Must be service_response");
        ASSERT_TRUE(resp["id"].as_string() == "req-step-1", "ID must match request ID");
        ASSERT_TRUE(resp["service"].as_string() == "/arm_sim/integration_step", "Service must match");
        ASSERT_TRUE(resp["result"].as_bool() == true, "Result must be true");
        ASSERT_TRUE(resp["status"].as_string() == "", "Status must be empty on success");
        ASSERT_TRUE(resp["values"].is_object(), "Values must be object");
        ASSERT_TRUE(resp["values"]["times"].as_array().size() == 6, "Times array length 6");
    }

    // 2. Send invalid call_service request (dt <= 0)
    {
        JsonValue call = JsonValue::make_object();
        call["op"] = "call_service";
        call["service"] = "/arm_sim/integration_step";
        call["id"] = "req-bad-dt";

        JsonValue args = JsonValue::make_object();
        args["function"] = "t";
        args["x0"] = 0.0;
        args["dt"] = -1.0; // Invalid
        args["steps"] = 5;
        args["integrator"] = "euler";
        call["args"] = args;

        send_json(call);

        JsonValue resp = recv_json_line();
        ASSERT_TRUE(resp["op"].as_string() == "service_response", "Must be service_response");
        ASSERT_TRUE(resp["id"].as_string() == "req-bad-dt", "ID must match request ID");
        ASSERT_TRUE(resp["result"].as_bool() == false, "Result must be false for invalid request");
        ASSERT_TRUE(!resp["status"].as_string().empty(), "Status must be non-empty on error");
    }

    // 3. Persistent TCP connection: Multiple back-to-back service calls
    {
        for (int i = 0; i < 5; ++i) {
            JsonValue call = JsonValue::make_object();
            call["op"] = "call_service";
            call["service"] = "/arm_sim/integration_step";
            call["id"] = "multi-req-" + std::to_string(i);

            JsonValue args = JsonValue::make_object();
            args["function"] = "sin(t) + " + std::to_string(i);
            args["x0"] = static_cast<double>(i);
            args["dt"] = 0.05;
            args["steps"] = 10;
            args["integrator"] = "midpoint";
            call["args"] = args;

            send_json(call);
            JsonValue resp = recv_json_line();
            ASSERT_TRUE(resp["result"].as_bool() == true, "Repeated TCP call must succeed");
            ASSERT_TRUE(resp["id"].as_string() == "multi-req-" + std::to_string(i), "ID must match in sequence");
            ASSERT_TRUE(resp["values"]["times"].as_array().size() == 11, "Times size 11");
        }
    }

    close(sock);
    server.stop();
    std::cout << "  -> PASSED" << std::endl;
}

// -------------------------------------------------------------
// Test 6: Domain Errors and Edge Cases
// -------------------------------------------------------------
void test_domain_errors_and_edge_cases() {
    std::cout << "[TEST] Domain Errors & Numerical Robustness..." << std::endl;

    // 1. Domain error sqrt(-1) evaluates to NaN without crashing
    {
        auto expr = parse_expression("sqrt(-1)");
        double val = expr->evaluate(0.0);
        ASSERT_TRUE(std::isnan(val), "sqrt(-1) must produce NaN without crash");
    }

    // 2. Division by zero produces inf without crashing
    {
        auto expr = parse_expression("1 / 0");
        double val = expr->evaluate(0.0);
        ASSERT_TRUE(std::isinf(val), "1 / 0 must produce inf without crash");
    }

    // 3. Integration with domain error evaluates cleanly in service
    {
        Middleware mw;
        register_integration_service(mw);

        JsonValue req = JsonValue::make_object();
        req["function"] = "sqrt(-t - 1)";
        req["x0"] = 0.0;
        req["dt"] = 0.1;
        req["steps"] = 5;
        req["integrator"] = "rk4";

        auto [ok, values] = mw.call_service("/arm_sim/integration_step", req);
        ASSERT_TRUE(ok, "Service with mathematical domain error should complete without crash");
        const auto& pos = values["positions"].as_array();
        ASSERT_TRUE(std::isnan(pos[1].as_number()), "Position with sqrt(-1) must be NaN");
    }

    // 4. Large step count (500 steps)
    {
        Middleware mw;
        register_integration_service(mw);

        JsonValue req = JsonValue::make_object();
        req["function"] = "cos(t)";
        req["x0"] = 0.0;
        req["xdot0"] = 0.0;
        req["dt"] = 0.01;
        req["steps"] = 500;
        req["integrator"] = "rk4";

        auto [ok, values] = mw.call_service("/arm_sim/integration_step", req);
        ASSERT_TRUE(ok, "Large step count must succeed");
        ASSERT_TRUE(values["times"].as_array().size() == 501, "Times array length must be 501");
        ASSERT_TRUE(values["positions"].as_array().size() == 501, "Positions array length must be 501");
        ASSERT_TRUE(values["velocities"].as_array().size() == 501, "Velocities array length must be 501");
    }

    std::cout << "  -> PASSED" << std::endl;
}

int main() {
    std::cout << "========================================" << std::endl;
    std::cout << "Running Integrator & Expression Tests" << std::endl;
    std::cout << "========================================" << std::endl;

    test_analytical_integrators();
    test_convergence_rates();
    test_expression_parser();
    test_service_validation();
    test_tcp_rosbridge_service();
    test_domain_errors_and_edge_cases();

    std::cout << "========================================" << std::endl;
    std::cout << "All Integrator Tests PASSED Successfully!" << std::endl;
    std::cout << "========================================" << std::endl;
    return 0;
}
