#ifndef PENDULARM_ARM_SIMULATION_HPP
#define PENDULARM_ARM_SIMULATION_HPP

#include <atomic>
#include <cstdint>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <vector>

#include "middleware/middleware.hpp"
#include "pendularm/arm_dynamics.hpp"
#include "pendularm/inverse_kinematics.hpp"

namespace pendularm {

// Owns the live plant and the Task 3 simulation-control interface.
class ArmSimulation {
public:
    ArmSimulation(middleware::Middleware& middleware, int link_count);
    ~ArmSimulation();

    ArmSimulation(const ArmSimulation&) = delete;
    ArmSimulation& operator=(const ArmSimulation&) = delete;

    void start();
    void stop();

private:
    struct IkActionGoal {
        std::string id;
        double x{0.0};
        double y{0.0};
        std::optional<double> phi;
        std::vector<double> positions;
        double epsilon{0.05};
        double success_hold{0.25};
        double start_time{0.0};
        bool inside_epsilon{false};
        double dwell_start{0.0};
    };

    struct TrialTarget {
        double x{0.0};
        double y{0.0};
        std::optional<double> phi;
    };

    struct IkTrial {
        bool running{false};
        double duration{20.0};
        double start_time{0.0};
        double elapsed_at_stop{0.0};
        double epsilon{0.05};
        double success_hold{0.25};
        uint64_t targets_reached{0};
        std::optional<std::string> goal_id;
        std::optional<TrialTarget> target;
        std::optional<double> error;
        std::optional<std::vector<double>> desired_positions;
        std::string action_status{"idle"};
    };

    struct ActionEvent {
        std::string goal_id;
        std::string outcome;
        double distance{0.0};
    };

    std::pair<bool, middleware::JsonValue> set_integrator(const middleware::JsonValue& args);
    std::pair<bool, middleware::JsonValue> set_params(const middleware::JsonValue& args);
    std::pair<bool, middleware::JsonValue> set_pause(const middleware::JsonValue& args);
    std::pair<bool, middleware::JsonValue> reset(const middleware::JsonValue& args);
    std::pair<bool, middleware::JsonValue> set_pid_enabled(const middleware::JsonValue& args);
    std::pair<bool, middleware::JsonValue> set_pid_gains(const middleware::JsonValue& args);
    std::pair<bool, middleware::JsonValue> solve_ik(const middleware::JsonValue& args);
    std::pair<bool, middleware::JsonValue> send_ik_goal(const middleware::JsonValue& args);
    std::pair<bool, middleware::JsonValue> cancel_ik_goal(const middleware::JsonValue& args);
    std::pair<bool, middleware::JsonValue> start_ik_trial(const middleware::JsonValue& args);
    std::pair<bool, middleware::JsonValue> skip_ik_trial(const middleware::JsonValue& args);
    std::pair<bool, middleware::JsonValue> stop_ik_trial(const middleware::JsonValue& args);
    void receive_trajectory(const middleware::JsonValue& message);
    void update_pid_effort();
    void update_ik_action();
    void update_ik_trial();
    bool submit_trial_target();
    std::optional<TrialTarget> sample_trial_target();
    void publish_trial_status();
    double action_distance(const IkActionGoal& goal) const;
    void publish_action_result(const IkActionGoal& goal, const std::string& outcome, double final_distance);
    void run();
    void publish_joint_state();

    middleware::Middleware& mw_;
    std::mutex mutex_;
    std::atomic<bool> running_{false};
    std::thread worker_;

    ArmParams params_;
    SecondOrderState state_;
    std::vector<double> effort_;
    std::vector<double> setpoint_pos_;
    std::vector<double> setpoint_vel_;
    std::vector<double> kp_;
    std::vector<double> ki_;
    std::vector<double> kd_;
    std::vector<double> integral_;
    bool pid_enabled_{false};
    static constexpr double integral_limit_{10.0};

    std::optional<IkActionGoal> active_goal_;
    std::optional<ActionEvent> last_action_event_;
    uint64_t next_goal_id_{1};
    std::optional<IkTrial> trial_;
    double trial_duration_{20.0};
    double trial_epsilon_{0.05};
    double trial_success_hold_{0.25};
    IntegratorType integrator_{IntegratorType::Rk4};
    double timestep_{0.005};
    double simulation_time_{0.0};
    bool paused_{false};
};

} // namespace pendularm

#endif
