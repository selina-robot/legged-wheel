# Root build for the go2w_standup controller.
# Control algorithms are added phase by phase; see AGENTS.md for the Gate order.
cmake_minimum_required(VERSION 3.22)
project(go2w_standup LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

find_package(Eigen3 REQUIRED)
find_package(pinocchio REQUIRED)
find_package(proxsuite REQUIRED)
find_package(unitree_sdk2 REQUIRED)
find_package(yaml-cpp REQUIRED)

include_directories(include)

# Library sources grow per phase; only create the target when sources exist.
file(GLOB_RECURSE GO2W_SOURCES CONFIGURE_DEPENDS
  src/backend/*.cpp src/core/*.cpp src/estimation/*.cpp src/control/*.cpp
  src/trajectory/*.cpp src/fsm/*.cpp src/safety/*.cpp)

if(GO2W_SOURCES)
  add_library(go2w_standup ${GO2W_SOURCES})
  target_link_libraries(go2w_standup PUBLIC
    Eigen3::Eigen pinocchio::pinocchio proxsuite::proxsuite
    unitree_sdk2 yaml-cpp)
endif()

# Applications (added as their sources appear).
foreach(app lowlevel_io_smoke_test go2w_standup_controller)
  if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/src/app/${app}.cpp)
    add_executable(${app} src/app/${app}.cpp)
    if(TARGET go2w_standup)
      target_link_libraries(${app} PRIVATE go2w_standup)
    else()
      target_link_libraries(${app} PRIVATE
        Eigen3::Eigen pinocchio::pinocchio proxsuite::proxsuite
        unitree_sdk2 yaml-cpp)
    endif()
  endif()
endforeach()

# C++ tests (spec §74).
enable_testing()
foreach(t test_lqr_config test_wbc_qp test_fsm test_command_safety)
  if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/tests/${t}.cpp)
    add_executable(${t} tests/${t}.cpp)
    target_link_libraries(${t} PRIVATE
      Eigen3::Eigen pinocchio::pinocchio proxsuite::proxsuite
      unitree_sdk2 yaml-cpp)
    if(TARGET go2w_standup)
      target_link_libraries(${t} PRIVATE go2w_standup)
    endif()
    add_test(NAME ${t} COMMAND ${t})
  endif()
endforeach()
