cmake_minimum_required(VERSION 3.16)
project(falcon_v2_aerodynamics LANGUAGES CXX)

# FALCON V2 - aerodynamics System plugin build.
# Owner: aerodynamics specialist agent. Task: AERODYNAMICS_V1_IMPLEMENTATION.
# See README.md in this directory for build/run instructions.

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
add_library(FalconV2Aerodynamics SHARED AerodynamicsSystem.cc)
target_compile_features(FalconV2Aerodynamics PRIVATE cxx_std_17)
target_compile_options(FalconV2Aerodynamics PRIVATE -Wall -Wextra -Wpedantic)
target_include_directories(FalconV2Aerodynamics PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(FalconV2Aerodynamics PRIVATE
  gz-sim8::gz-sim8
  gz-plugin2::gz-plugin2
  gz-transport13::gz-transport13
  gz-msgs10::gz-msgs10
  gz-math7::gz-math7
  yaml-cpp)
set_target_properties(FalconV2Aerodynamics PROPERTIES OUTPUT_NAME FalconV2Aerodynamics)

# ---------------------------------------------------------------------------
# Standalone, Gazebo-independent self-test executable for the pure-math core
# (AeroModel.hh). Only depends on gz-math7 - does NOT require a running
# Gazebo instance. This is what the aerodynamics agent used to self-check
# CL0/Cm0/alpha-beta/force-transform/saturation numerics before handoff to
# gazebo-testing for live-Gazebo sign/force tests. See test/README or
# AERODYNAMICS.md for the self-test result log.
# ---------------------------------------------------------------------------
add_executable(aero_model_selftest test/aero_model_selftest.cc)
target_compile_features(aero_model_selftest PRIVATE cxx_std_17)
target_include_directories(aero_model_selftest PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(aero_model_selftest PRIVATE gz-math7::gz-math7)
