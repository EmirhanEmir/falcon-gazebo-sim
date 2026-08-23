cmake_minimum_required(VERSION 3.16)
project(falcon_v2_propulsion LANGUAGES CXX)

# FALCON V2 - propulsion System plugin build.
# Owner: propulsion specialist agent. Task: PROPULSION_V1_IMPLEMENTATION.
# See README.md in this directory for build/run instructions. Mirrors
# plugins/aerodynamics/CMakeLists.txt's structure.

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
add_library(FalconV2Propulsion SHARED PropulsionSystem.cc)
target_compile_features(FalconV2Propulsion PRIVATE cxx_std_17)
target_compile_options(FalconV2Propulsion PRIVATE -Wall -Wextra -Wpedantic)
target_include_directories(FalconV2Propulsion PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(FalconV2Propulsion PRIVATE
  gz-sim8::gz-sim8
  gz-plugin2::gz-plugin2
  gz-transport13::gz-transport13
  gz-msgs10::gz-msgs10
  gz-math7::gz-math7
  yaml-cpp)
set_target_properties(FalconV2Propulsion PROPERTIES OUTPUT_NAME FalconV2Propulsion)

# ---------------------------------------------------------------------------
# Standalone, Gazebo-independent self-test executable for the pure-math core
# (PropulsionModel.hh). Only depends on the C++ standard library - does NOT
# require a running Gazebo instance and does NOT link gz-sim/gz-math (unlike
# the aerodynamics self-test, PropulsionModel.hh has no gz-math dependency
# either - see its header comment). This is what the propulsion agent used
# to self-check Kt/Ke, APC data parsing/interpolation, T/P/Q formulas, RPM
# cap logic, and rotor ODE numerics before handoff to gazebo-testing.
# ---------------------------------------------------------------------------
add_executable(propulsion_model_selftest test/propulsion_model_selftest.cc)
target_compile_features(propulsion_model_selftest PRIVATE cxx_std_17)
target_include_directories(propulsion_model_selftest PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
