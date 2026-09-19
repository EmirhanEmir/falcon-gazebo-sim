cmake_minimum_required(VERSION 3.16)
project(falcon_v2_sensors LANGUAGES CXX)

# FALCON V2 - sensor-suite plugin build.
# IMU/GPS/baro use the native installed gz-sim system plugins, loaded at the
# world level rather than built here. Pitot and magnetometer are custom
# plugins built from this directory - see PitotModel.hh and
# MagnetometerSystem.hh for the rationale.

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
