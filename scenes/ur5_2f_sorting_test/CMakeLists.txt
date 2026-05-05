cmake_minimum_required(VERSION 3.5)
project(ur5_2f_sorting_test)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(Python3 REQUIRED COMPONENTS Interpreter)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY urdf
  DESTINATION share/${PROJECT_NAME}
)

install(FILES
  environment.yaml
  scene_manifest.yaml
  sorting_manifest.yaml
  README.md
  fixtures/detected_objects_sample.json
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/generate_sorting_plan.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME generate_sorting_plan
)

install(PROGRAMS
  scripts/validate_scene_layout.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME validate_scene_layout
)

install(PROGRAMS
  scripts/generate_sorting_runtime_plan.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME generate_sorting_runtime_plan
)

install(PROGRAMS
  scripts/generate_sorting_emd_bridge_payload.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME generate_sorting_emd_bridge_payload
)

install(PROGRAMS
  scripts/generate_runtime_plan_from_detections.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME generate_runtime_plan_from_detections
)

install(PROGRAMS
  scripts/generate_bridge_payload_from_detections.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME generate_bridge_payload_from_detections
)

install(PROGRAMS
  scripts/preview_sorting_execution_handoff.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME preview_sorting_execution_handoff
)

install(PROGRAMS
  scripts/robot_studio_lite.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME robot_studio_lite
)

install(PROGRAMS
  scripts/prepare_static_sorting_execution.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME prepare_static_sorting_execution
)

install(PROGRAMS
  scripts/manual_static_sorting_executor.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME manual_static_sorting_executor
)

install(PROGRAMS
  scripts/generate_static_sorting_runtime_bridge_payload.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME generate_static_sorting_runtime_bridge_payload
)

if(BUILD_TESTING)
  add_test(
    NAME ur5_2f_sorting_test_sorting_manifest_validation
    COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test/test_sorting_manifest.py
  )

  add_test(
    NAME ur5_2f_sorting_test_generate_sorting_plan_output
    COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test/test_generate_sorting_plan.py
  )

  add_test(
    NAME ur5_2f_sorting_test_validate_scene_layout
    COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test/test_validate_scene_layout.py
  )

  add_test(
    NAME ur5_2f_sorting_test_generate_sorting_runtime_plan
    COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test/test_generate_sorting_runtime_plan.py
  )

  add_test(
    NAME ur5_2f_sorting_test_generate_sorting_emd_bridge_payload
    COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test/test_generate_sorting_emd_bridge_payload.py
  )

  add_test(
    NAME ur5_2f_sorting_test_generate_runtime_plan_from_detections
    COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test/test_generate_runtime_plan_from_detections.py
  )

  add_test(
    NAME ur5_2f_sorting_test_generate_bridge_payload_from_detections
    COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test/test_generate_bridge_payload_from_detections.py
  )

  add_test(
    NAME ur5_2f_sorting_test_preview_sorting_execution_handoff
    COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test/test_preview_sorting_execution_handoff.py
  )

  add_test(
    NAME ur5_2f_sorting_test_robot_studio_lite
    COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test/test_robot_studio_lite.py
  )

  add_test(
    NAME ur5_2f_sorting_test_prepare_static_sorting_execution
    COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test/test_prepare_static_sorting_execution.py
  )

  add_test(
    NAME ur5_2f_sorting_test_manual_static_sorting_executor
    COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test/test_manual_static_sorting_executor.py
  )

  add_test(
    NAME ur5_2f_sorting_test_generate_static_sorting_runtime_bridge_payload
    COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test/test_generate_static_sorting_runtime_bridge_payload.py
  )
endif()

ament_package()
