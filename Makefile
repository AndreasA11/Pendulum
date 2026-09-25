CXX ?= g++
CXXFLAGS ?= -std=c++20 -Wall -Wextra -Wpedantic -Iinclude

.PHONY: build run clean test test_integrators test_middleware

build: src/main.cpp include/pendularm/integrator.hpp include/pendularm/expression.hpp include/pendularm/integration_service.hpp include/middleware/middleware.hpp include/middleware/rosbridge_server.hpp include/middleware/json.hpp
	mkdir -p build
	$(CXX) $(CXXFLAGS) -o build/arm_sim src/main.cpp

run: build
	./build/arm_sim

test_integrators: tests/test_integrators.cpp include/pendularm/integrator.hpp include/pendularm/expression.hpp include/pendularm/integration_service.hpp include/middleware/middleware.hpp include/middleware/rosbridge_server.hpp include/middleware/json.hpp
	mkdir -p build
	$(CXX) $(CXXFLAGS) -o build/test_integrators tests/test_integrators.cpp
	./build/test_integrators

test_middleware: tests/test_middleware.cpp include/middleware/middleware.hpp include/middleware/rosbridge_server.hpp include/middleware/json.hpp
	mkdir -p build
	$(CXX) $(CXXFLAGS) -o build/test_middleware tests/test_middleware.cpp
	./build/test_middleware

test: test_middleware test_integrators

clean:
	rm -rf build
