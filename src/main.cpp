#include <iostream>
#include <csignal>
#include <atomic>
#include <thread>
#include <chrono>
#include <cstdlib>
#include "middleware/middleware.hpp"
#include "middleware/rosbridge_server.hpp"
#include "pendularm/integration_service.hpp"
#include "pendularm/arm_simulation.hpp"

namespace {
std::atomic<bool> g_shutdown{false};

void signal_handler(int /*signum*/) {
    g_shutdown.store(true);
}
} // namespace

int main() {
    // Register signal handlers for clean termination
    std::signal(SIGINT, signal_handler);
    std::signal(SIGTERM, signal_handler);

    // Read ARM_SIM_LINKS environment variable (defaults to 2 if unset or invalid)
    int arm_sim_links = 2;
    const char* env_links = std::getenv("ARM_SIM_LINKS");
    if (env_links != nullptr) {
        std::string s(env_links);
        if (s == "3") {
            arm_sim_links = 3;
        } else if (s == "2") {
            arm_sim_links = 2;
        }
    }

    int port = 9095;
    const char* env_port = std::getenv("ARM_SIM_PORT");
    if (env_port != nullptr) {
        try {
            int p = std::stoi(env_port);
            if (p > 1024 && p <= 65535) port = p;
        } catch (...) {}
    }

    std::cout << "Starting Pendularm Runtime (ARM_SIM_LINKS=" << arm_sim_links << ", PORT=" << port << ")..." << std::endl;

    // Create middleware and rosbridge TCP server
    middleware::Middleware mw;
    pendularm::register_integration_service(mw);
    pendularm::ArmSimulation simulation(mw, arm_sim_links);

    middleware::RosbridgeServer server(mw, "127.0.0.1", port);
    if (!server.start()) {
        std::cerr << "Failed to start Rosbridge server on 127.0.0.1:" << port << std::endl;
        return 1;
    }

    std::cout << "Rosbridge server listening on 127.0.0.1:" << port << std::endl;
    std::cout << "Registered service: /arm_sim/integration_step" << std::endl;
    std::cout << "Registered simulation services: /arm_sim/set_integrator, /arm_sim/set_params, /arm_sim/pause, /arm_sim/reset" << std::endl;
    std::cout << "Registered PID services: /pid_controller/enable, /pid_controller/set_gains" << std::endl;
    std::cout << "Registered IK service: /ik/solve" << std::endl;
    std::cout << "Registered IK action services: /ik_action/send_goal, /ik_action/cancel_goal" << std::endl;
    std::cout << "Registered IK trial services: /ik_trial/start, /ik_trial/skip, /ik_trial/stop" << std::endl;
    simulation.start();

    // Main execution loop until signal received
    while (!g_shutdown.load()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }

    std::cout << "Shutting down Pendularm Runtime..." << std::endl;
    simulation.stop();
    server.stop();
    std::cout << "Shutdown complete." << std::endl;

    return 0;
}
