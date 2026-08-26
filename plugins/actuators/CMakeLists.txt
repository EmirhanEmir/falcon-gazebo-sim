cmake_minimum_required(VERSION 3.16)
project(falcon_v2_actuators LANGUAGES CXX)

# FALCON V2 - actuator/servo System plugin build.
# Owner: controls-integration specialist agent. Task: ACTUATOR_SERVO_MODEL_V1.
# See README.md in this directory for build/run instructions. Mirrors
# plugins/aerodynamics/CMakeLists.txt and plugins/propulsion/CMakeLists.txt's
# structure.

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

find_package(gz-sim8 REQUIRED)
find_package(gz-plugin2 REQUIRED COMPONENTS register)
find_package(gz-transport13 REQUIRED)
find_package(gz-msgs10 REQUIRED)
find_package(gz-math7 REQUIRED)
find_package(yaml-cpp REQUIRED)

# ---------------------------------------------------------------------------
# Main Gazebo System plugin (requires the full gz-sim runtime environment)
# ---------------------------------------------------------------------------
add_library(FalconV2Actuators SHARED ActuatorSystem.cc)
target_compile_features(FalconV2Actuators PRIVATE cxx_std_17)
target_compile_options(FalconV2Actuators PRIVATE -Wall -Wextra -Wpedantic)
target_include_directories(FalconV2Actuators PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(FalconV2Actuators PRIVATE
  gz-sim8::gz-sim8
  gz-plugin2::gz-plugin2
  gz-transport13::gz-transport13
  gz-msgs10::gz-msgs10
  gz-math7::gz-math7
  yaml-cpp)
set_target_properties(FalconV2Actuators PROPERTIES OUTPUT_NAME FalconV2Actuators)

# ---------------------------------------------------------------------------
# Standalone, Gazebo-independent self-test executable for the pure-math core
# (ActuatorModel.hh). Depends on the C++ standard library ONLY -
# ActuatorModel.hh has no gz-math/gz-sim dependency at all (every quantity is
# a scalar, single-DOF joint - same "zero Gazebo dependency" precedent as
# plugins/propulsion/PropulsionModel.hh). Does NOT require a running Gazebo
# instance. This is what the controls-integration agent used to self-check
# the rate-limit/clamp/effort-bounded-PD numerics before handoff to
# gazebo-testing for live-Gazebo joint/aero-mapping tests.
# ---------------------------------------------------------------------------
add_executable(actuator_model_selftest test/actuator_model_selftest.cc)
target_compile_features(actuator_model_selftest PRIVATE cxx_std_17)
target_include_directories(actuator_model_selftest PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
