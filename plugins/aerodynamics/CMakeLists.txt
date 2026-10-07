cmake_minimum_required(VERSION 3.16)
project(falcon_v2_aerodynamics LANGUAGES CXX)

# FALCON V2 — aerodinamik System eklentisi derleme tanımı.
# Derleme/çalıştırma talimatları için bu dizindeki README.md dosyasına bakınız.

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
# Ana Gazebo System eklentisi (tam gz-sim çalışma ortamını gerektirir).
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
