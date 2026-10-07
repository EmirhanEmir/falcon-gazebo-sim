cmake_minimum_required(VERSION 3.16)
project(falcon_v2_actuators LANGUAGES CXX)

# FALCON V2 - aktüatör/servo System plugin'i derlemesi.
# Derleme/çalıştırma talimatları: bu dizindeki README.md.

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
# Ana Gazebo System plugin'i (tam gz-sim runtime ortamını gerektirir)
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
