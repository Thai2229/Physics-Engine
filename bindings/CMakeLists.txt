# Emscripten embind module: produces engine.js + engine.wasm. Only added under Emscripten.

add_executable(physics_wasm wasm_bindings.cpp)
target_link_libraries(physics_wasm PRIVATE physics embind)
target_compile_options(physics_wasm PRIVATE -O3 -fno-exceptions)

# Until World provides queryPoint/contactPoints/contactCount/lastStepMs (collision work),
# compile temporary stand-ins. Auto-detected, so this switches off by itself.
file(READ "${CMAKE_CURRENT_SOURCE_DIR}/../engine/include/physics/world.hpp" _physics_world_hpp)
if(NOT _physics_world_hpp MATCHES "queryPoint" OR NOT _physics_world_hpp MATCHES "lastStepMs")
  message(WARNING "World lacks the collision API: bindings use temporary fallbacks (no contacts)")
  target_compile_definitions(physics_wasm PRIVATE PHYSICS_BINDINGS_COLLISION_FALLBACK=1)
endif()

set_target_properties(physics_wasm PROPERTIES
  OUTPUT_NAME engine
  SUFFIX ".js"
  RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/dist")
target_link_options(physics_wasm PRIVATE
  -O3
  -fno-exceptions
  -sMODULARIZE=1
  -sEXPORT_ES6=1
  -sEXPORT_NAME=createEngineModule
  -sALLOW_MEMORY_GROWTH=1
  -sENVIRONMENT=web,worker,node
  -sDISABLE_EXCEPTION_CATCHING=1)
