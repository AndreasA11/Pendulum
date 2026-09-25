#include "pendularm/arm_simulation.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <numbers>
#include <random>
#include <string>

namespace pendularm {
namespace {

middleware::JsonValue array_from(const std::vector<double>& values) {
    middleware::JsonValue result = middleware::JsonValue::make_array();
    for (double value : values) result.push_back(value);
    return result;
}

middleware::JsonValue error_values(const std::string& message) {
    middleware::JsonValue result = middleware::JsonValue::make_object();
    result["error"] = message;
    return result;
}

bool valid_vector(const middleware::JsonValue& value, size_t count, std::vector<double>& out) {
    if (!value.is_array() || value.as_array().size() != count) return false;
    std::vector<double> parsed;
    parsed.reserve(count);
    for (const auto& item : value.as_array()) {
        if (!item.is_number() || !std::isfinite(item.as_number()) || item.as_number() <= 0.0) return false;
        parsed.push_back(item.as_number());
    }
    out = std::move(parsed);
    return true;
}

bool valid_gain_vector(const middleware::JsonValue& value, size_t count, std::vector<double>& out) {
    if (!value.is_array() || value.as_array().size() != count) return false;
    std::vector<double> parsed;
    parsed.reserve(count);
    for (const auto& item : value.as_array()) {
        if (!item.is_number() || !std::isfinite(item.as_number()) || item.as_number() < 0.0) return false;
        parsed.push_back(item.as_number());
    }
    out = std::move(parsed);
    return true;
}

} // namespace

ArmSimulation::ArmSimulation(middleware::Middleware& middleware, int link_count) : mw_(middleware) {
    const size_t n = link_count == 3 ? 3U : 2U;
    params_.lengths.assign(n, 1.0);
    params_.masses.assign(n, 1.0);
    params_.gravity = 9.81;
    state_.q.assign(n, 0.0);
    state_.qdot.assign(n, 0.0);
    effort_.assign(n, 0.0);
    setpoint_pos_.assign(n, 0.0);
    setpoint_vel_.assign(n, 0.0);
    // These conservative defaults can stabilize a one-metre, one-kilogram link
    // while still leaving useful room for live gain tuning.
    kp_.assign(n, 40.0);
    ki_.assign(n, 8.0);
    kd_.assign(n, 10.0);
    integral_.assign(n, 0.0);

    mw_.subscribe("/joint_trajectory", [this](const middleware::JsonValue& message) {
        receive_trajectory(message);
    });
    mw_.advertise_service("/arm_sim/set_integrator", [this](const middleware::JsonValue& args) {
        return set_integrator(args);
    });
    mw_.advertise_service("/arm_sim/set_params", [this](const middleware::JsonValue& args) {
        return set_params(args);
    });
    mw_.advertise_service("/arm_sim/pause", [this](const middleware::JsonValue& args) {
        return set_pause(args);
    });
    mw_.advertise_service("/arm_sim/reset", [this](const middleware::JsonValue& args) {
        return reset(args);
    });
    mw_.advertise_service("/pid_controller/enable", [this](const middleware::JsonValue& args) {
        return set_pid_enabled(args);
    });
    mw_.advertise_service("/pid_controller/set_gains", [this](const middleware::JsonValue& args) {
        return set_pid_gains(args);
    });
    mw_.advertise_service("/ik/solve", [this](const middleware::JsonValue& args) {
        return solve_ik(args);
    });
    mw_.advertise_service("/ik_action/send_goal", [this](const middleware::JsonValue& args) {
        return send_ik_goal(args);
    });
    mw_.advertise_service("/ik_action/cancel_goal", [this](const middleware::JsonValue& args) {
        return cancel_ik_goal(args);
    });
    mw_.advertise_service("/ik_trial/start", [this](const middleware::JsonValue& args) { return start_ik_trial(args); });
    mw_.advertise_service("/ik_trial/skip", [this](const middleware::JsonValue& args) { return skip_ik_trial(args); });
    mw_.advertise_service("/ik_trial/stop", [this](const middleware::JsonValue& args) { return stop_ik_trial(args); });
}

ArmSimulation::~ArmSimulation() { stop(); }

void ArmSimulation::start() {
    if (!running_.exchange(true)) worker_ = std::thread(&ArmSimulation::run, this);
}

void ArmSimulation::stop() {
    if (running_.exchange(false) && worker_.joinable()) worker_.join();
}

std::pair<bool, middleware::JsonValue> ArmSimulation::set_integrator(const middleware::JsonValue& args) {
    if (!args.is_object()) return {false, error_values("Request args must be a JSON object")};
    bool valid = true;
    std::string error;
    std::lock_guard<std::mutex> lock(mutex_);
    if (args.contains("method")) {
        if (!args["method"].is_string() || !parse_integrator_type(args["method"].as_string())) {
            valid = false; error += "Invalid integrator method. ";
        } else {
            integrator_ = *parse_integrator_type(args["method"].as_string());
        }
    }
    if (args.contains("timestep")) {
        if (!args["timestep"].is_number() || !std::isfinite(args["timestep"].as_number()) || args["timestep"].as_number() <= 0.0) {
            valid = false; error += "timestep must be positive and finite. ";
        } else {
            timestep_ = args["timestep"].as_number();
        }
    }
    middleware::JsonValue values = middleware::JsonValue::make_object();
    values["method"] = integrator_type_to_string(integrator_);
    values["timestep"] = timestep_;
    if (!valid) values["error"] = error;
    return {valid, values};
}

std::pair<bool, middleware::JsonValue> ArmSimulation::set_params(const middleware::JsonValue& args) {
    if (!args.is_object()) {
        std::lock_guard<std::mutex> lock(mutex_);
        middleware::JsonValue response = middleware::JsonValue::make_object();
        response["gravity"] = params_.gravity;
        response["masses"] = array_from(params_.masses);
        response["lengths"] = array_from(params_.lengths);
        response["error"] = "Request args must be a JSON object";
        return {false, response};
    }
    bool valid = true;
    std::string error;
    std::lock_guard<std::mutex> lock(mutex_);
    if (args.contains("gravity")) {
        if (!args["gravity"].is_number() || !std::isfinite(args["gravity"].as_number()) || args["gravity"].as_number() < 0.0) {
            valid = false; error += "gravity must be finite and non-negative. ";
        } else params_.gravity = args["gravity"].as_number();
    }
    if (args.contains("masses")) {
        std::vector<double> values;
        if (!valid_vector(args["masses"], params_.num_links(), values)) {
            valid = false; error += "masses must be a positive array matching link count. ";
        } else params_.masses = std::move(values);
    }
    if (args.contains("lengths")) {
        std::vector<double> values;
        if (!valid_vector(args["lengths"], params_.num_links(), values)) {
            valid = false; error += "lengths must be a positive array matching link count. ";
        } else params_.lengths = std::move(values);
    }
    middleware::JsonValue response = middleware::JsonValue::make_object();
    response["gravity"] = params_.gravity;
    response["masses"] = array_from(params_.masses);
    response["lengths"] = array_from(params_.lengths);
    if (!valid) response["error"] = error;
    return {valid, response};
}

std::pair<bool, middleware::JsonValue> ArmSimulation::set_pause(const middleware::JsonValue& args) {
    if (!args.is_object()) return {false, error_values("Request args must be a JSON object")};
    std::lock_guard<std::mutex> lock(mutex_);
    if (args.contains("data")) {
        if (!args["data"].is_bool()) return {false, error_values("data must be a boolean")};
        paused_ = args["data"].as_bool();
    }
    middleware::JsonValue response = middleware::JsonValue::make_object();
    response["data"] = paused_;
    return {true, response};
}

std::pair<bool, middleware::JsonValue> ArmSimulation::reset(const middleware::JsonValue& args) {
    if (!args.is_object()) return {false, error_values("Request args must be a JSON object")};
    std::lock_guard<std::mutex> lock(mutex_);
    std::fill(state_.q.begin(), state_.q.end(), 0.0);
    std::fill(state_.qdot.begin(), state_.qdot.end(), 0.0);
    std::fill(effort_.begin(), effort_.end(), 0.0);
    std::fill(setpoint_pos_.begin(), setpoint_pos_.end(), 0.0);
    std::fill(setpoint_vel_.begin(), setpoint_vel_.end(), 0.0);
    std::fill(integral_.begin(), integral_.end(), 0.0);
    simulation_time_ = 0.0;
    middleware::JsonValue response = middleware::JsonValue::make_object();
    response["position"] = array_from(state_.q);
    response["velocity"] = array_from(state_.qdot);
    return {true, response};
}

std::pair<bool, middleware::JsonValue> ArmSimulation::set_pid_enabled(const middleware::JsonValue& args) {
    if (!args.is_object()) return {false, error_values("Request args must be a JSON object")};
    std::lock_guard<std::mutex> lock(mutex_);
    if (args.contains("data")) {
        if (!args["data"].is_bool()) return {false, error_values("data must be a boolean")};
        const bool enable = args["data"].as_bool();
        if (enable) std::fill(integral_.begin(), integral_.end(), 0.0);
        pid_enabled_ = enable;
        if (!pid_enabled_) std::fill(effort_.begin(), effort_.end(), 0.0);
    }
    middleware::JsonValue response = middleware::JsonValue::make_object();
    response["data"] = pid_enabled_;
    return {true, response};
}

std::pair<bool, middleware::JsonValue> ArmSimulation::set_pid_gains(const middleware::JsonValue& args) {
    if (!args.is_object()) return {false, error_values("Request args must be a JSON object")};
    bool valid = true;
    std::string error;
    std::lock_guard<std::mutex> lock(mutex_);
    const auto apply_gain = [&](const char* name, std::vector<double>& destination) {
        if (!args.contains(name)) return;
        std::vector<double> values;
        if (!valid_gain_vector(args[name], destination.size(), values)) {
            valid = false;
            error += std::string(name) + " must be a non-negative array matching link count. ";
        } else {
            destination = std::move(values);
        }
    };
    apply_gain("kp", kp_);
    apply_gain("ki", ki_);
    apply_gain("kd", kd_);
    middleware::JsonValue response = middleware::JsonValue::make_object();
    response["kp"] = array_from(kp_);
    response["ki"] = array_from(ki_);
    response["kd"] = array_from(kd_);
    if (!valid) response["error"] = error;
    return {valid, response};
}

std::pair<bool, middleware::JsonValue> ArmSimulation::solve_ik(const middleware::JsonValue& args) {
    if (!args.is_object() || !args.contains("x") || !args.contains("y") ||
        !args["x"].is_number() || !args["y"].is_number()) {
        return {false, error_values("x and y are required numeric fields")};
    }
    const double x = args["x"].as_number();
    const double y = args["y"].as_number();
    std::optional<double> phi;
    if (args.contains("phi") && !args["phi"].is_null()) {
        if (!args["phi"].is_number() || !std::isfinite(args["phi"].as_number())) {
            return {false, error_values("phi must be a finite number")};
        }
        phi = args["phi"].as_number();
    }

    // Snapshot rather than cache: this includes every successful live length update.
    ArmParams current_params;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        current_params = params_;
    }
    const IkSolution solution = solve_planar_ik(current_params, x, y, phi);
    if (!solution.success) return {false, error_values(solution.error)};
    middleware::JsonValue response = middleware::JsonValue::make_object();
    response["positions"] = array_from(solution.positions);
    return {true, response};
}

std::pair<bool, middleware::JsonValue> ArmSimulation::send_ik_goal(const middleware::JsonValue& args) {
    if (!args.is_object() || !args.contains("x") || !args.contains("y") ||
        !args["x"].is_number() || !args["y"].is_number() ||
        !std::isfinite(args["x"].as_number()) || !std::isfinite(args["y"].as_number())) {
        return {false, error_values("x and y are required finite numeric fields")};
    }
    const double x = args["x"].as_number();
    const double y = args["y"].as_number();
    std::optional<double> phi;
    if (args.contains("phi") && !args["phi"].is_null()) {
        if (!args["phi"].is_number() || !std::isfinite(args["phi"].as_number())) {
            return {false, error_values("phi must be a finite number")};
        }
        phi = args["phi"].as_number();
    }
    double epsilon = 0.05;
    if (args.contains("epsilon") && !args["epsilon"].is_null()) {
        if (!args["epsilon"].is_number() || !std::isfinite(args["epsilon"].as_number()) || args["epsilon"].as_number() <= 0.0) {
            return {false, error_values("epsilon must be positive and finite")};
        }
        epsilon = args["epsilon"].as_number();
    }
    double success_hold = 0.25;
    if (args.contains("success_hold") && !args["success_hold"].is_null()) {
        if (!args["success_hold"].is_number() || !std::isfinite(args["success_hold"].as_number()) || args["success_hold"].as_number() < 0.0) {
            return {false, error_values("success_hold must be non-negative and finite")};
        }
        success_hold = args["success_hold"].as_number();
    }

    ArmParams current_params;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        current_params = params_;
    }
    const IkSolution solution = solve_planar_ik(current_params, x, y, phi);
    // Do all validation before touching active_goal_: a rejected replacement has no side effects.
    if (!solution.success) return {false, error_values(solution.error)};

    std::optional<IkActionGoal> preempted;
    double preempted_distance = 0.0;
    IkActionGoal new_goal;
    double current_sim_time = 0.0;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (active_goal_) {
            preempted = active_goal_;
            preempted_distance = action_distance(*active_goal_);
        }
        new_goal.id = "ik-goal-" + std::to_string(next_goal_id_++);
        new_goal.x = x;
        new_goal.y = y;
        if (current_params.num_links() == 3) {
            new_goal.phi = phi.value_or(solution.positions[0] + solution.positions[1] + solution.positions[2]);
        } else {
            new_goal.phi = std::nullopt;
        }
        new_goal.positions = solution.positions;
        new_goal.epsilon = epsilon;
        new_goal.success_hold = success_hold;
        new_goal.start_time = simulation_time_;
        active_goal_ = new_goal;
        setpoint_pos_ = solution.positions;
        std::fill(setpoint_vel_.begin(), setpoint_vel_.end(), 0.0);
        current_sim_time = simulation_time_;
    }
    if (preempted) {
        publish_action_result(*preempted, "preempted", preempted_distance);
        std::lock_guard<std::mutex> lock(mutex_);
        last_action_event_ = ActionEvent{preempted->id, "preempted", preempted_distance};
    }

    // Publish trajectory_msgs/JointTrajectory compliant envelope
    middleware::JsonValue trajectory = middleware::JsonValue::make_object();
    middleware::JsonValue header = middleware::JsonValue::make_object();
    uint32_t sec = static_cast<uint32_t>(current_sim_time);
    uint32_t nanosec = static_cast<uint32_t>((current_sim_time - sec) * 1e9);
    middleware::JsonValue stamp = middleware::JsonValue::make_object();
    stamp["sec"] = static_cast<double>(sec);
    stamp["nanosec"] = static_cast<double>(nanosec);
    header["stamp"] = stamp;
    header["frame_id"] = "";
    trajectory["header"] = header;

    middleware::JsonValue joint_names = middleware::JsonValue::make_array();
    for (size_t i = 0; i < current_params.num_links(); ++i) {
        joint_names.push_back("joint" + std::to_string(i + 1));
    }
    trajectory["joint_names"] = joint_names;

    middleware::JsonValue points = middleware::JsonValue::make_array();
    middleware::JsonValue final_point = middleware::JsonValue::make_object();
    final_point["positions"] = array_from(new_goal.positions);
    final_point["velocities"] = array_from(std::vector<double>(new_goal.positions.size(), 0.0));
    final_point["accelerations"] = middleware::JsonValue::make_array();
    middleware::JsonValue time_from_start = middleware::JsonValue::make_object();
    time_from_start["sec"] = 0;
    time_from_start["nanosec"] = 0;
    final_point["time_from_start"] = time_from_start;
    points.push_back(final_point);
    trajectory["points"] = points;
    mw_.publish("/joint_trajectory", trajectory);

    middleware::JsonValue response = middleware::JsonValue::make_object();
    response["goal_id"] = new_goal.id;
    return {true, response};
}

std::pair<bool, middleware::JsonValue> ArmSimulation::cancel_ik_goal(const middleware::JsonValue& args) {
    if (!args.is_object()) return {false, error_values("Request args must be a JSON object")};
    std::optional<IkActionGoal> cancelled;
    double final_distance = 0.0;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!active_goal_) return {false, error_values("No active IK goal")};
        if (args.contains("goal_id") && !args["goal_id"].is_null()) {
            if (!args["goal_id"].is_string() || args["goal_id"].as_string() != active_goal_->id) {
                return {false, error_values("goal_id does not match the active goal")};
            }
        }
        cancelled = active_goal_;
        final_distance = action_distance(*active_goal_);
        active_goal_.reset();
    }
    publish_action_result(*cancelled, "preempted", final_distance);
    {
        std::lock_guard<std::mutex> lock(mutex_);
        last_action_event_ = ActionEvent{cancelled->id, "preempted", final_distance};
    }
    middleware::JsonValue response = middleware::JsonValue::make_object();
    response["goal_id"] = cancelled->id;
    return {true, response};
}

double ArmSimulation::action_distance(const IkActionGoal& goal) const {
    const Pose2D pose = forward_kinematics_end_effector(state_.q, params_);
    return std::hypot(pose.x - goal.x, pose.y - goal.y);
}

void ArmSimulation::publish_action_result(const IkActionGoal& goal, const std::string& outcome, double final_distance) {
    middleware::JsonValue result = middleware::JsonValue::make_object();
    middleware::JsonValue target = middleware::JsonValue::make_object();
    target["x"] = goal.x;
    target["y"] = goal.y;
    if (params_.num_links() == 3 && goal.phi) target["phi"] = *goal.phi;
    result["goal_id"] = goal.id;
    result["outcome"] = outcome;
    result["target"] = target;
    result["final_distance"] = final_distance;
    mw_.publish("/ik_action/result", result);
}

void ArmSimulation::update_ik_action() {
    std::optional<IkActionGoal> completed_goal;
    double completed_dist = 0.0;
    middleware::JsonValue feedback_msg;
    bool should_publish_feedback = false;

    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!active_goal_) return;
        IkActionGoal& goal = *active_goal_;
        const double distance = action_distance(goal);
        const double elapsed = simulation_time_ - goal.start_time;
        if (distance <= goal.epsilon) {
            if (!goal.inside_epsilon) {
                goal.inside_epsilon = true;
                goal.dwell_start = simulation_time_;
            }
            if (goal.success_hold == 0.0 || simulation_time_ - goal.dwell_start >= goal.success_hold) {
                completed_goal = goal;
                completed_dist = distance;
                active_goal_.reset();
                last_action_event_ = ActionEvent{completed_goal->id, "reached", completed_dist};
            }
        } else {
            goal.inside_epsilon = false;
        }

        if (!completed_goal) {
            should_publish_feedback = true;
            feedback_msg = middleware::JsonValue::make_object();
            middleware::JsonValue target = middleware::JsonValue::make_object();
            target["x"] = goal.x;
            target["y"] = goal.y;
            if (params_.num_links() == 3 && goal.phi) target["phi"] = *goal.phi;
            feedback_msg["goal_id"] = goal.id;
            feedback_msg["target"] = target;
            feedback_msg["positions"] = array_from(goal.positions);
            feedback_msg["distance_remaining"] = distance;
            feedback_msg["elapsed"] = elapsed;
            if (trial_ && trial_->running && trial_->goal_id && *trial_->goal_id == goal.id) {
                trial_->error = distance;
            }
        }
    }

    if (completed_goal) {
        publish_action_result(*completed_goal, "reached", completed_dist);
    } else if (should_publish_feedback) {
        mw_.publish("/ik_action/feedback", feedback_msg);
    }
}

std::optional<ArmSimulation::TrialTarget> ArmSimulation::sample_trial_target() {
    ArmParams params;
    { std::lock_guard<std::mutex> lock(mutex_); params = params_; }
    static thread_local std::mt19937 generator{0x367U};
    std::uniform_real_distribution<double> angle(-std::numbers::pi, std::numbers::pi);
    std::vector<double> q(params.num_links());
    for (double& value : q) value = angle(generator);
    const Pose2D pose = forward_kinematics_end_effector(q, params);
    TrialTarget target{pose.x, pose.y, std::nullopt};
    if (params.num_links() == 3) target.phi = pose.phi;
    return target;
}

bool ArmSimulation::submit_trial_target() {
    const auto target = sample_trial_target();
    if (!target) return false;
    middleware::JsonValue request = middleware::JsonValue::make_object();
    request["x"] = target->x;
    request["y"] = target->y;
    if (target->phi) request["phi"] = *target->phi;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!trial_ || !trial_->running) return false;
        request["epsilon"] = trial_->epsilon;
        request["success_hold"] = trial_->success_hold;
    }
    const auto [ok, response] = mw_.call_service("/ik_action/send_goal", request);
    if (!ok) return false;
    std::lock_guard<std::mutex> lock(mutex_);
    if (!trial_ || !trial_->running) return false;
    const std::string id = response["goal_id"].as_string();
    trial_->goal_id = id;
    trial_->target = target;
    trial_->action_status = "active";
    if (active_goal_ && active_goal_->id == id) {
        trial_->desired_positions = active_goal_->positions;
        trial_->error = action_distance(*active_goal_);
    } else {
        const Pose2D pose = forward_kinematics_end_effector(state_.q, params_);
        trial_->error = std::hypot(pose.x - target->x, pose.y - target->y);
    }
    return true;
}

std::pair<bool, middleware::JsonValue> ArmSimulation::start_ik_trial(const middleware::JsonValue& args) {
    if (!args.is_object()) return {false, error_values("Request args must be a JSON object")};
    double duration = trial_duration_;
    double epsilon = trial_epsilon_;
    double success_hold = trial_success_hold_;
    const auto field = [&args](const char* name, double& value, bool zero_ok) {
        if (!args.contains(name) || args[name].is_null()) return true;
        if (!args[name].is_number() || !std::isfinite(args[name].as_number()) ||
            (zero_ok ? args[name].as_number() < 0.0 : args[name].as_number() <= 0.0)) return false;
        value = args[name].as_number();
        return true;
    };
    if (!field("duration", duration, false)) return {false, error_values("duration must be positive and finite")};
    if (!field("epsilon", epsilon, false)) return {false, error_values("epsilon must be positive and finite")};
    if (!field("success_hold", success_hold, true)) return {false, error_values("success_hold must be non-negative and finite")};

    mw_.call_service("/ik_action/cancel_goal", middleware::JsonValue::make_object());

    {
        std::lock_guard<std::mutex> lock(mutex_);
        trial_duration_ = duration;
        trial_epsilon_ = epsilon;
        trial_success_hold_ = success_hold;

        trial_ = IkTrial{};
        trial_->running = true;
        trial_->duration = duration;
        trial_->epsilon = epsilon;
        trial_->success_hold = success_hold;
        trial_->start_time = simulation_time_;
        trial_->elapsed_at_stop = 0.0;
        last_action_event_.reset();
    }
    if (!submit_trial_target()) {
        std::lock_guard<std::mutex> lock(mutex_);
        trial_->running = false;
        trial_->action_status = "idle";
        return {false, error_values("Unable to submit a reachable IK trial target")};
    }
    return {true, middleware::JsonValue::make_object()};
}

std::pair<bool, middleware::JsonValue> ArmSimulation::skip_ik_trial(const middleware::JsonValue& args) {
    if (!args.is_object()) return {false, error_values("Request args must be a JSON object")};
    std::optional<std::string> old_goal;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!trial_ || !trial_->running) return {false, error_values("No trial is running")};
        old_goal = trial_->goal_id;
        trial_->goal_id.reset();
        trial_->target.reset();
        trial_->error.reset();
        trial_->desired_positions.reset();
        trial_->action_status = "idle";
        last_action_event_.reset();
    }
    if (old_goal) {
        middleware::JsonValue request = middleware::JsonValue::make_object();
        request["goal_id"] = *old_goal;
        mw_.call_service("/ik_action/cancel_goal", request);
    }
    if (!submit_trial_target()) return {false, error_values("Unable to submit a reachable IK trial target")};
    return {true, middleware::JsonValue::make_object()};
}

std::pair<bool, middleware::JsonValue> ArmSimulation::stop_ik_trial(const middleware::JsonValue& args) {
    if (!args.is_object()) return {false, error_values("Request args must be a JSON object")};
    std::optional<std::string> old_goal;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!trial_ || !trial_->running) return {false, error_values("No trial is running")};
        old_goal = trial_->goal_id;
        trial_->running = false;
        trial_->elapsed_at_stop = std::clamp(simulation_time_ - trial_->start_time, 0.0, trial_->duration);
        trial_->goal_id.reset();
        trial_->target.reset();
        trial_->error.reset();
        trial_->desired_positions.reset();
        trial_->action_status = "idle";
        last_action_event_.reset();
    }
    if (old_goal) {
        middleware::JsonValue request = middleware::JsonValue::make_object();
        request["goal_id"] = *old_goal;
        mw_.call_service("/ik_action/cancel_goal", request);
    }
    return {true, middleware::JsonValue::make_object()};
}

void ArmSimulation::update_ik_trial() {
    std::optional<std::string> cancel_id;
    bool submit_next = false;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!trial_ || !trial_->running) return;
        if (simulation_time_ - trial_->start_time >= trial_->duration) {
            cancel_id = trial_->goal_id;
            trial_->running = false;
            trial_->elapsed_at_stop = trial_->duration;
            trial_->goal_id.reset();
            trial_->target.reset();
            trial_->error.reset();
            trial_->desired_positions.reset();
            trial_->action_status = "idle";
            last_action_event_.reset();
        } else if (last_action_event_) {
            const ActionEvent event = *last_action_event_;
            last_action_event_.reset();
            if (trial_->goal_id && *trial_->goal_id == event.goal_id) {
                if (event.outcome == "reached") {
                    ++trial_->targets_reached;
                    trial_->goal_id.reset();
                    trial_->target.reset();
                    trial_->error.reset();
                    trial_->desired_positions.reset();
                    trial_->action_status = event.outcome;
                    submit_next = true;
                } else {
                    trial_->goal_id.reset();
                    trial_->target.reset();
                    trial_->error.reset();
                    trial_->desired_positions.reset();
                    trial_->action_status = event.outcome;
                    submit_next = false;
                }
            }
        }
    }
    if (cancel_id) {
        middleware::JsonValue request = middleware::JsonValue::make_object();
        request["goal_id"] = *cancel_id;
        mw_.call_service("/ik_action/cancel_goal", request);
    }
    if (submit_next && !submit_trial_target()) {
        std::lock_guard<std::mutex> lock(mutex_);
        if (trial_) {
            trial_->running = false;
            trial_->elapsed_at_stop = std::clamp(simulation_time_ - trial_->start_time, 0.0, trial_->duration);
            trial_->action_status = "idle";
        }
    }
}

void ArmSimulation::publish_trial_status() {
    middleware::JsonValue status = middleware::JsonValue::make_object();
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!trial_) return;
        status["running"] = trial_->running;
        status["elapsed"] = trial_->running
            ? std::clamp(simulation_time_ - trial_->start_time, 0.0, trial_->duration)
            : trial_->elapsed_at_stop;
        status["duration"] = trial_->duration;
        status["targets_reached"] = static_cast<size_t>(trial_->targets_reached);
        status["action_status"] = trial_->action_status;
        if (trial_->target) {
            middleware::JsonValue target = middleware::JsonValue::make_object();
            target["x"] = trial_->target->x;
            target["y"] = trial_->target->y;
            if (params_.num_links() == 3 && trial_->target->phi) {
                target["phi"] = *trial_->target->phi;
            }
            status["target"] = target;
        } else {
            status["target"] = middleware::JsonValue();
        }
        if (trial_->error) status["error"] = *trial_->error;
        else status["error"] = middleware::JsonValue();
        if (trial_->desired_positions) status["desired_positions"] = array_from(*trial_->desired_positions);
        else status["desired_positions"] = middleware::JsonValue();
    }
    mw_.publish("/ik_trial/status", status);
}

void ArmSimulation::receive_trajectory(const middleware::JsonValue& message) {
    if (!message.is_object() || !message.contains("points") || !message["points"].is_array()) return;
    const auto& points = message["points"].as_array();
    if (points.empty() || !points.back().is_object() || !points.back().contains("positions")) return;
    const auto& final_point = points.back();
    const size_t n = params_.num_links();
    if (!final_point["positions"].is_array() || final_point["positions"].as_array().size() != n) return;
    std::vector<double> positions;
    positions.reserve(n);
    for (const auto& item : final_point["positions"].as_array()) {
        if (!item.is_number() || !std::isfinite(item.as_number())) return;
        positions.push_back(item.as_number());
    }
    std::vector<double> velocities(n, 0.0);
    if (final_point.contains("velocities") && final_point["velocities"].is_array() && final_point["velocities"].as_array().size() == n) {
        bool good = true;
        for (size_t i = 0; i < n; ++i) {
            const auto& item = final_point["velocities"].as_array()[i];
            if (!item.is_number() || !std::isfinite(item.as_number())) { good = false; break; }
            velocities[i] = item.as_number();
        }
        if (!good) velocities.assign(n, 0.0);
    }
    std::lock_guard<std::mutex> lock(mutex_);
    setpoint_pos_ = std::move(positions);
    setpoint_vel_ = std::move(velocities);
}

void ArmSimulation::update_pid_effort() {
    if (!pid_enabled_) {
        std::fill(effort_.begin(), effort_.end(), 0.0);
        return;
    }
    for (size_t i = 0; i < effort_.size(); ++i) {
        const double position_error = setpoint_pos_[i] - state_.q[i];
        const double velocity_error = setpoint_vel_[i] - state_.qdot[i];
        integral_[i] = std::clamp(integral_[i] + position_error * timestep_, -integral_limit_, integral_limit_);
        effort_[i] = kp_[i] * position_error + ki_[i] * integral_[i] + kd_[i] * velocity_error;
    }
}

void ArmSimulation::publish_joint_state() {
    middleware::JsonValue message = middleware::JsonValue::make_object();
    middleware::JsonValue names = middleware::JsonValue::make_array();
    double current_time = 0.0;
    std::vector<double> q, qdot, effort;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        for (size_t i = 0; i < state_.q.size(); ++i) names.push_back("joint" + std::to_string(i + 1));
        current_time = simulation_time_;
        q = state_.q;
        qdot = state_.qdot;
        effort = effort_;
    }
    middleware::JsonValue header = middleware::JsonValue::make_object();
    uint32_t sec = static_cast<uint32_t>(current_time);
    uint32_t nanosec = static_cast<uint32_t>((current_time - sec) * 1e9);
    middleware::JsonValue stamp = middleware::JsonValue::make_object();
    stamp["sec"] = static_cast<double>(sec);
    stamp["nanosec"] = static_cast<double>(nanosec);
    header["stamp"] = stamp;
    header["frame_id"] = "";
    message["header"] = header;
    message["name"] = names;
    message["position"] = array_from(q);
    message["velocity"] = array_from(qdot);
    message["effort"] = array_from(effort);
    mw_.publish("/joint_states", message);
}

void ArmSimulation::run() {
    using Clock = std::chrono::steady_clock;
    auto next_physics = Clock::now();
    auto next_publish = next_physics;
    while (running_.load()) {
        const auto now = Clock::now();
        if (now >= next_physics) {
            {
                std::lock_guard<std::mutex> lock(mutex_);
                if (!paused_) {
                    update_pid_effort();
                    const auto acceleration = make_forward_dynamics_accel_func(effort_, params_);
                    state_ = integrate_step(integrator_, acceleration, simulation_time_, state_, timestep_);
                    simulation_time_ += timestep_;
                }
            }
            if (!paused_) {
                update_ik_action();
            }
            update_ik_trial();
            next_physics = now + std::chrono::milliseconds(5);
        }
        if (now >= next_publish) {
            publish_joint_state();
            publish_trial_status();
            next_publish = now + std::chrono::milliseconds(16);
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
}

} // namespace pendularm
