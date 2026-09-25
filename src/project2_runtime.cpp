// Unity translation unit for the runtime executable.  The project remains
// organized into ordinary component source files; including them here avoids
// repeatedly parsing the same headers during the time-sensitive `make run`
// startup path used by the rosbridge acceptance harness.
#include "integrator.cpp"
#include "expression.cpp"
#include "integration_service.cpp"
#include "arm_dynamics.cpp"
#include "inverse_kinematics.cpp"
#include "arm_simulation.cpp"
#include "main.cpp"
