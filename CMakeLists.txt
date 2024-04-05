cmake_minimum_required(VERSION 3.13)
project(planning)
set(CMAKE_CXX_STANDARD 17)

set(PROJECT_VERSION 0.1.0)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(Eigen3 REQUIRED)

# CATCH2 ######################################################################

include(FetchContent)

FetchContent_Declare(
  Catch2
  GIT_REPOSITORY https://github.com/catchorg/Catch2.git
  GIT_TAG v3.4.0 # or a later release
)

FetchContent_MakeAvailable(Catch2)

#MATPLOTLIB ###################################################################

# Find Python interpreter for Matplotlib
find_package(Python3 COMPONENTS Interpreter Development)
include_directories(include external ${Python3_INCLUDE_DIRS})

set(matplotlib-file ${CMAKE_SOURCE_DIR}/external/matplotlibcpp.h)

if(NOT EXISTS "${matplotlib-file}")
  message("-- Downloading the matplotlibcpp.h header file from github")
  file(
    DOWNLOAD
    https://raw.githubusercontent.com/lava/matplotlib-cpp/master/matplotlibcpp.h
    ${matplotlib-file})
else()
  message("-- Found matplotlibcpp.h header locally.")
endif()

# PLANNING ####################################################################

add_library(plotter_lib src/planner_plotter.cpp include/planner_plotter.hpp)
target_link_libraries(plotter_lib Python3::Python)

add_library(planning_lib src/planner_rrt.cpp include/planner_rrt.hpp)
target_link_libraries(planning_lib Eigen3::Eigen)

# EXAMPLES ####################################################################

add_executable(example src/example_rrt.cpp)
target_link_libraries(example
  planning_lib
  plotter_lib
  Python3::Python
  Eigen3::Eigen)

# TESTING ######################################################################

add_executable(tests tests/planning.cpp)
target_link_libraries(tests PRIVATE planning_lib Catch2::Catch2WithMain)

include(Catch)
catch_discover_tests(tests)
