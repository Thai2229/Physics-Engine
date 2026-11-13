cmake_minimum_required(VERSION 3.20)
project(PhysicsEngine VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

option(PHYSICS_BUILD_TESTS "Build the GoogleTest suites" ON)
option(PHYSICS_WARNINGS_AS_ERRORS "Treat compiler warnings as errors" OFF)

if(PHYSICS_BUILD_TESTS)
  enable_testing()
endif()

add_subdirectory(engine)
