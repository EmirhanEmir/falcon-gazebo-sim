cmake_minimum_required(VERSION 3.16)
project(falcon_v2_sensors LANGUAGES CXX)

# FALCON V2 - sensör paketi plugin derlemesi.
# IMU/GPS/barometre, world seviyesinde yüklenen yerleşik gz-sim system
# plugin'lerini kullanır; burada derlenmez. Pitot ve magnetometre ise bu
# dizinden derlenen özel plugin'lerdir - gerekçesi PitotModel.hh ve
# MagnetometerSystem.hh başlıklarındadır.

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

find_package(gz-sim8 REQUIRED)
find_package(gz-plugin2 REQUIRED COMPONENTS register)
find_package(gz-transport13 REQUIRED)
find_package(gz-msgs10 REQUIRED)
find_package(gz-math7 REQUIRED)

# ---------------------------------------------------------------------------
# Ana Gazebo System plugin'i (tam gz-sim runtime ortamını gerektirir).
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
# Özel magnetometre System plugin'i (yerleşik gz-sim-magnetometer-system'in
# yerini alır - kök neden kaydı MagnetometerSystem.hh başlığındadır).
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
