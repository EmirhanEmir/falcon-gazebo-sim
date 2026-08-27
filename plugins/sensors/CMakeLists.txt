cmake_minimum_required(VERSION 3.16)
project(falcon_v2_sensors LANGUAGES CXX)

# FALCON V2 - sensor-suite plugin build.
# Owner: controls-integration specialist agent. Task:
# SENSOR_MODEL_AND_ARDUPLANE_SITL_PREPARATION (+ bug-fix follow-up,
# 2026-08-27, MagnetometerSystem).
# Mirrors plugins/wind/CMakeLists.txt's structure. IMU/GPS/baro use the
# NATIVE installed gz-sim system plugins (loaded at the world level, not
# built here). Pitot (see PitotModel.hh header comment for why) and
# magnetometer (see MagnetometerSystem.hh header comment - native
# gz-sim-magnetometer-system confirmed non-functional on this installation)
# both have real, custom plugin code in this directory.

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

find_package(gz-sim8 REQUIRED)
find_package(gz-plugin2 REQUIRED COMPONENTS register)
find_package(gz-transport13 REQUIRED)
find_package(gz-msgs10 REQUIRED)
find_package(gz-math7 REQUIRED)

# ---------------------------------------------------------------------------
# Main Gazebo System plugin (requires the full gz-sim runtime environment).
# ---------------------------------------------------------------------------
add_library(FalconV2Pitot SHARED PitotSystem.cc)
target_compile_features(FalconV2Pitot PRIVATE cxx_std_17)
target_compile_options(FalconV2Pitot PRIVATE -Wall -Wextra -Wpedantic)
target_include_directories(FalconV2Pitot PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(FalconV2Pitot PRIVATE
  gz-sim8::gz-sim8
  gz-plugin2::gz-plugin2
  gz-transport13::gz-transport13
  gz-msgs10::gz-msgs10
  gz-math7::gz-math7)
set_target_properties(FalconV2Pitot PROPERTIES OUTPUT_NAME FalconV2Pitot)

# ---------------------------------------------------------------------------
# Custom magnetometer System plugin (replaces the native
# gz-sim-magnetometer-system - see MagnetometerSystem.hh header comment for
# the root-cause record).
# ---------------------------------------------------------------------------
add_library(FalconV2Magnetometer SHARED MagnetometerSystem.cc)
target_compile_features(FalconV2Magnetometer PRIVATE cxx_std_17)
target_compile_options(FalconV2Magnetometer PRIVATE -Wall -Wextra -Wpedantic)
target_include_directories(FalconV2Magnetometer PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(FalconV2Magnetometer PRIVATE
  gz-sim8::gz-sim8
  gz-plugin2::gz-plugin2
  gz-transport13::gz-transport13
  gz-msgs10::gz-msgs10
  gz-math7::gz-math7)
set_target_properties(FalconV2Magnetometer PROPERTIES OUTPUT_NAME FalconV2Magnetometer)

# ---------------------------------------------------------------------------
# Standalone, Gazebo-independent self-test executable for the pure-math core
# (Frames.hh + PitotModel.hh). Depends on gz-math7 only - does NOT require a
# running Gazebo instance. Same precedent as wind_model_selftest/
# actuator_model_selftest/aero_model_selftest/propulsion_model_selftest.
# ---------------------------------------------------------------------------
add_executable(sensors_model_selftest test/sensors_model_selftest.cc)
target_compile_features(sensors_model_selftest PRIVATE cxx_std_17)
target_include_directories(sensors_model_selftest PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(sensors_model_selftest PRIVATE gz-math7::gz-math7)
