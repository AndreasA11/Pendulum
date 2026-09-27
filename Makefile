CXX ?= g++
CXXFLAGS ?= -std=c++20 -Wall -Wextra -Wpedantic -Iinclude

RUNTIME_SRC = src/project2_runtime.cpp
SRCS = src/main.cpp src/integrator.cpp src/expression.cpp src/integration_service.cpp src/arm_dynamics.cpp src/inverse_kinematics.cpp src/arm_simulation.cpp
HDRS = include/pendularm/integrator.hpp include/pendularm/expression.hpp include/pendularm/integration_service.hpp include/pendularm/arm_dynamics.hpp include/pendularm/inverse_kinematics.hpp include/pendularm/arm_simulation.hpp include/middleware/middleware.hpp include/middleware/rosbridge_server.hpp include/middleware/json.hpp

.PHONY: build run clean test test_integrators test_dynamics test_middleware test_inverse_kinematics test_ik_trial test_project2_integration test_runtime_contract test_ik_action test_action_trial_timing visualizer gui

build: build/arm_sim

build/arm_sim: $(RUNTIME_SRC) $(SRCS) $(HDRS)
	mkdir -p build
	$(CXX) $(CXXFLAGS) -o $@ $(RUNTIME_SRC)

run: build
	./build/arm_sim

visualizer: build
	python3 scripts/visualizer.py

gui: visualizer

test_integrators: tests/test_integrators.cpp src/integrator.cpp src/expression.cpp src/integration_service.cpp $(HDRS)
	mkdir -p build
	$(CXX) $(CXXFLAGS) -o build/test_integrators tests/test_integrators.cpp src/integrator.cpp src/expression.cpp src/integration_service.cpp
	./build/test_integrators

test_dynamics: tests/test_dynamics.cpp src/arm_dynamics.cpp src/integrator.cpp $(HDRS)
	mkdir -p build
	$(CXX) $(CXXFLAGS) -o build/test_dynamics tests/test_dynamics.cpp src/arm_dynamics.cpp src/integrator.cpp
	./build/test_dynamics

test_middleware: tests/test_middleware.cpp include/middleware/middleware.hpp include/middleware/rosbridge_server.hpp include/middleware/json.hpp
	mkdir -p build
	$(CXX) $(CXXFLAGS) -o build/test_middleware tests/test_middleware.cpp
	./build/test_middleware

test_inverse_kinematics: tests/test_inverse_kinematics.cpp src/inverse_kinematics.cpp src/arm_dynamics.cpp src/integrator.cpp $(HDRS)
	mkdir -p build
	$(CXX) $(CXXFLAGS) -o build/test_inverse_kinematics tests/test_inverse_kinematics.cpp src/inverse_kinematics.cpp src/arm_dynamics.cpp src/integrator.cpp
	./build/test_inverse_kinematics

test_ik_trial: tests/test_ik_trial.cpp src/arm_simulation.cpp src/inverse_kinematics.cpp src/arm_dynamics.cpp src/integrator.cpp $(HDRS)
	mkdir -p build
	$(CXX) $(CXXFLAGS) -o build/test_ik_trial tests/test_ik_trial.cpp src/arm_simulation.cpp src/inverse_kinematics.cpp src/arm_dynamics.cpp src/integrator.cpp
	./build/test_ik_trial

test_project2_integration: tests/test_project2_integration.cpp src/arm_simulation.cpp src/inverse_kinematics.cpp src/arm_dynamics.cpp src/integrator.cpp src/expression.cpp src/integration_service.cpp $(HDRS)
	mkdir -p build
	$(CXX) $(CXXFLAGS) -o build/test_project2_integration tests/test_project2_integration.cpp src/arm_simulation.cpp src/inverse_kinematics.cpp src/arm_dynamics.cpp src/integrator.cpp src/expression.cpp src/integration_service.cpp
	./build/test_project2_integration

test_runtime_contract: tests/test_runtime_contract.cpp src/arm_simulation.cpp src/inverse_kinematics.cpp src/arm_dynamics.cpp src/integrator.cpp $(HDRS)
	mkdir -p build
	$(CXX) $(CXXFLAGS) -o build/test_runtime_contract tests/test_runtime_contract.cpp src/arm_simulation.cpp src/inverse_kinematics.cpp src/arm_dynamics.cpp src/integrator.cpp
	./build/test_runtime_contract

test_ik_action: tests/test_ik_action.cpp src/arm_simulation.cpp src/inverse_kinematics.cpp src/arm_dynamics.cpp src/integrator.cpp $(HDRS)
	mkdir -p build
	$(CXX) $(CXXFLAGS) -o build/test_ik_action tests/test_ik_action.cpp src/arm_simulation.cpp src/inverse_kinematics.cpp src/arm_dynamics.cpp src/integrator.cpp
	./build/test_ik_action

test_action_trial_timing: tests/test_action_trial_timing.cpp src/arm_simulation.cpp src/inverse_kinematics.cpp src/arm_dynamics.cpp src/integrator.cpp $(HDRS)
	mkdir -p build
	$(CXX) $(CXXFLAGS) -o build/test_action_trial_timing tests/test_action_trial_timing.cpp src/arm_simulation.cpp src/inverse_kinematics.cpp src/arm_dynamics.cpp src/integrator.cpp
	./build/test_action_trial_timing

test: test_middleware test_integrators test_dynamics test_inverse_kinematics test_ik_trial test_project2_integration test_runtime_contract test_ik_action test_action_trial_timing

clean:
	rm -rf build
