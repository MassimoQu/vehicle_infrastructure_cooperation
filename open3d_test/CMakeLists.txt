cmake_minimum_required(VERSION 3.11)
project(OPEN3D_TEST VERSION 0.0)

set(CMAKE_CXX_STANDARD 14)

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${Open3D_C_FLAGS}")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${Open3D_CXX_FLAGS}")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${Open3D_EXE_LINKER_FLAGS}")

include(FetchContent)
include(pybind11)

find_package(Eigen3 REQUIRED QUIET)
find_package(Open3D HINTS ${CMAKE_INSTALL_PREFIX}/lib/CMake)
list(APPEND Open3D_LIBRARIES dl)

message(STATUS "Found Open3D ${Open3D_VERSION}")
link_directories(${Open3D_LIBRARY_DIRS})

add_executable(OPEN3D_TEST test.cpp)
target_include_directories(OPEN3D_TEST ${Open3D_INCLUDE_DIRS})