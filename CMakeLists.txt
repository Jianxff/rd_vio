cmake_minimum_required(VERSION 3.8)
project(rdvio LANGUAGES CXX C)

cmake_policy(SET CMP0072 NEW)
cmake_policy(SET CMP0079 NEW)

## cxx standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_BUILD_TYPE "Debug")

## cmake warnings
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wno-unused-result -Wno-ignored-attributes")
set(CMAKE_CXX_FLAGS "${CMAKE_C_FLAGS} -Wno-unused-result -Wno-ignored-attributes")

## cmake optimize
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Og -msse -msse2 -msse3 -ffast-math -mtune=native")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Og -msse -msse2 -msse3 -ffast-math -mtune=native")

## ccache
find_program(CCACHE_EXE ccache)
if(CCACHE_EXE)
    set_property(GLOBAL PROPERTY RULE_LAUNCH_COMPILE "${CCACHE_EXE}")
    set_property(GLOBAL PROPERTY RULE_LAUNCH_LINK "${CCACHE_EXE}")
    message(STATUS "ccache: Enabled")
endif()

## dependencies
find_package(Threads REQUIRED)
find_package(OpenCV 4 REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Ceres REQUIRED)
find_package(yaml-cpp REQUIRED)

## source
add_subdirectory(src)

## example
add_subdirectory(examples)