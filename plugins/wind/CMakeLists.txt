cmake_minimum_required(VERSION 3.16)
project(falcon_v2_wind LANGUAGES CXX)

# FALCON V2 — rüzgâr/hamle üreticisi System eklentisi derleme tanımı.
# Derleme/çalıştırma talimatları için bu dizindeki README.md dosyasına bakınız.

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

find_package(gz-sim8 REQUIRED)
find_package(gz-plugin2 REQUIRED COMPONENTS register)
find_package(gz-transport13 REQUIRED)
find_package(gz-msgs10 REQUIRED)
find_package(gz-math7 REQUIRED)

# ---------------------------------------------------------------------------
# Ana Gazebo System eklentisi (tam gz-sim çalışma ortamını gerektirir).
# yaml-cpp bağımlılığı yoktur: bu eklentinin yüklenecek bir katsayı veri
# kümesi yoktur; tek girdileri SDF parametreleri ve canlı komut konularıdır.
# ---------------------------------------------------------------------------
add_library(FalconV2Wind SHARED WindSystem.cc)
target_compile_features(FalconV2Wind PRIVATE cxx_std_17)
target_compile_options(FalconV2Wind PRIVATE -Wall -Wextra -Wpedantic)
target_include_directories(FalconV2Wind PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(FalconV2Wind PRIVATE
  gz-sim8::gz-sim8
  gz-plugin2::gz-plugin2
  gz-transport13::gz-transport13
  gz-msgs10::gz-msgs10
  gz-math7::gz-math7)
set_target_properties(FalconV2Wind PROPERTIES OUTPUT_NAME FalconV2Wind)
