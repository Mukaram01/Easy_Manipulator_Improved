cmake_minimum_required(VERSION 3.16)
project(workcell_owner_physics LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 17)
find_package(ignition-gazebo6 6.18.0 EXACT REQUIRED)
find_package(ignition-physics5 5.4.0 EXACT REQUIRED)
find_package(ignition-plugin1 REQUIRED COMPONENTS register)
find_package(ignition-common4 REQUIRED COMPONENTS profiler)
find_package(DART 6.12.1 EXACT REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(JSON REQUIRED IMPORTED_TARGET jsoncpp)
find_package(Python3 REQUIRED COMPONENTS Interpreter)
# Opt-in standalone build. No install rule and no changes to production worlds.
execute_process(COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/prepare.py
    ${CMAKE_CURRENT_BINARY_DIR}/source COMMAND_ERROR_IS_FATAL ANY)
add_library(workcell_owner_physics SHARED ${CMAKE_CURRENT_BINARY_DIR}/source/Physics.cc)
target_include_directories(workcell_owner_physics PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
target_compile_options(workcell_owner_physics PRIVATE -O0 -g0 -fvisibility=hidden)
target_link_libraries(workcell_owner_physics PRIVATE ignition-gazebo6::ignition-gazebo6
  ignition-physics5::ignition-physics5 ignition-common4::ignition-common4
  ignition-common4::profiler ignition-plugin1::register dart PkgConfig::JSON)
enable_testing()
add_executable(owner_bindings_test bindings_test.cpp)
add_test(NAME owner_bindings COMMAND owner_bindings_test)
find_package(ignition-gazebo6-rendering REQUIRED)
add_executable(owner_identity_inventory_test identity_inventory_test.cpp)
target_link_libraries(owner_identity_inventory_test PRIVATE ignition-gazebo6::ignition-gazebo6
  ignition-gazebo6::ignition-gazebo6-rendering PkgConfig::JSON)
add_test(NAME owner_identity_inventory COMMAND owner_identity_inventory_test)

execute_process(COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/prepare.py
  ${CMAKE_CURRENT_BINARY_DIR}/reference --reference COMMAND_ERROR_IS_FATAL ANY)
add_library(workcell_reference_physics SHARED ${CMAKE_CURRENT_BINARY_DIR}/reference/Physics.cc)
target_compile_options(workcell_reference_physics PRIVATE -O0 -g0 -fvisibility=hidden)
target_link_libraries(workcell_reference_physics PRIVATE ignition-gazebo6::ignition-gazebo6
  ignition-physics5::ignition-physics5 ignition-common4::ignition-common4
  ignition-common4::profiler ignition-plugin1::register)
