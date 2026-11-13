# Physics engine core. No platform dependencies: must also build with Emscripten.

add_library(physics_warnings INTERFACE)
if(MSVC)
  target_compile_options(physics_warnings INTERFACE /W4)
  if(PHYSICS_WARNINGS_AS_ERRORS)
    target_compile_options(physics_warnings INTERFACE /WX)
  endif()
else()
  target_compile_options(physics_warnings INTERFACE -Wall -Wextra -Wpedantic)
  if(PHYSICS_WARNINGS_AS_ERRORS)
    target_compile_options(physics_warnings INTERFACE -Werror)
  endif()
endif()

add_library(physics STATIC
  src/shape.cpp
  src/body.cpp
  src/collision.cpp
  src/contact.cpp
  src/joint.cpp
  src/world.cpp
)
target_include_directories(physics PUBLIC include)
target_compile_features(physics PUBLIC cxx_std_17)
target_link_libraries(physics PRIVATE physics_warnings)

if(PHYSICS_BUILD_TESTS)
  include(FetchContent)
  set(INSTALL_GTEST OFF CACHE BOOL "" FORCE)
  set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
  # Offline builds: -DPHYSICS_GOOGLETEST_DIR=/path/to/googletest (or FETCHCONTENT_SOURCE_DIR_GOOGLETEST).
  set(PHYSICS_GOOGLETEST_DIR "" CACHE PATH "Local googletest source tree (skips the download)")
  if(PHYSICS_GOOGLETEST_DIR)
    set(FETCHCONTENT_SOURCE_DIR_GOOGLETEST "${PHYSICS_GOOGLETEST_DIR}" CACHE PATH "" FORCE)
  endif()
  FetchContent_Declare(googletest
    URL https://github.com/google/googletest/archive/refs/tags/v1.14.0.zip)
  FetchContent_MakeAvailable(googletest)
  add_subdirectory(tests)
endif()
