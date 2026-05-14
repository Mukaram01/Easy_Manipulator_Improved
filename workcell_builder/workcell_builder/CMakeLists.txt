cmake_minimum_required(VERSION 3.5)

project(workcell_builder LANGUAGES CXX)

set(CMAKE_INCLUDE_CURRENT_DIR ON)

set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_visual_tools REQUIRED)
find_package(pluginlib REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(Boost REQUIRED COMPONENTS system filesystem)
find_package(Qt5 COMPONENTS Widgets Concurrent Svg REQUIRED)

include_directories(include)
include_directories(include/attributes)
include_directories(include/yaml_parser)
include_directories(gui)

add_executable(workcell_builder
    gui/main.cpp
    gui/mainwindow.cpp
    gui/mainwindow.h
    gui/mainwindow.ui

    gui/replacewarning.cpp
    gui/replacewarning.h
    gui/replacewarning.ui

    gui/scene_select.cpp
    gui/scene_select.h
    gui/scene_select.ui

    gui/addcollision.cpp
    gui/addcollision.h
    gui/addcollision.ui

    gui/addexternaljoint.cpp
    gui/addexternaljoint.h
    gui/addexternaljoint.ui

    gui/addinertial.cpp
    gui/addinertial.h
    gui/addinertial.ui

    gui/addjoint.cpp
    gui/addjoint.h
    gui/addjoint.ui

    gui/addlink.cpp
    gui/addlink.h
    gui/addlink.ui

    gui/addobject.cpp
    gui/addobject.h
    gui/addobject.ui

    gui/addvisual.cpp
    gui/addvisual.h
    gui/addvisual.ui

    gui/addscene.cpp
    gui/addscene.h
    gui/addscene.ui

    gui/addrobot.cpp
    gui/addrobot.h
    gui/addrobot.ui

    gui/addendeffector.cpp
    gui/addendeffector.h
    gui/addendeffector.ui

    gui/replacewarning.cpp
    gui/replacewarning.h
    gui/replacewarning.ui

    gui/loadobjects.cpp
    gui/loadobjects.h
    gui/loadobjects.ui

    src_scene_select_paths.cpp
    src_asset_discovery_helper.cpp
    src_generated_stl_writer.cpp
    src_generated_environment_asset_writer.cpp
    src_object_placement_model.cpp
    src_robot_tool_compatibility.cpp
    src_workcell_scene_schema.cpp
    src_workcell_scene_roundtrip.cpp
    src_workcell_scene_bundle.cpp
    src_camera_perception_profile.cpp
    src_deployment_compatibility_profile.cpp
    src_offline_readiness_overlay.cpp
    src_validation_dashboard_model.cpp
    src_offline_smoke_check_model.cpp
    src_workcell_scene_status.cpp
    src_workcell_studio_scene_browser.cpp
    src_workcell_camera_model.cpp
    src_workcell_zone_model.cpp
    src_conveyor_pick_preview.cpp
    src_workcell_perception_snapshot.cpp
    src_task_intent_readiness.cpp
    src_class_routing_model.cpp
    src_planning_readiness.cpp
    src_grasp_strategy_model.cpp
    src_scene_template_library.cpp
    src_workcell_studio_template_instantiator.cpp
    src_workcell_studio_canvas_model.cpp
    src_workcell_studio_layout_editor.cpp
    src_workcell_studio_layout_merge.cpp
    src_external_asset_importer.cpp
    gui/asset_picker_dialog.cpp
    gui/object_placement_dialog.cpp
    gui/environment_layout_editor.cpp
    gui/external_asset_import_dialog.cpp
    gui/workcell_builder_ui_utils.cpp
    gui/startup_dialog.cpp
    gui/asset_catalog_model.cpp
    gui/conveyor_sorting_scenario_wizard.cpp
    gui/conveyor_sorting_scenario_wizard.h
    gui/conveyor_sorting_scenario_wizard.ui
    gui/conveyor_sorting_run_console.cpp
    gui/conveyor_sorting_run_console.h
    gui/conveyor_sorting_run_console.ui

    include/asset_picker_dialog.hpp
    include/object_placement_dialog.hpp
    include/environment_layout_editor.hpp
    include/external_asset_import_dialog.hpp
    include/workspace_validation.hpp
    src_workspace_validation.cpp)

ament_target_dependencies(workcell_builder
  rclcpp
  rclcpp_components
  tf2
  tf2_ros
  tf2_eigen
  tf2_geometry_msgs
  geometry_msgs
  sensor_msgs
  std_msgs
  moveit_core
  moveit_ros_planning_interface
  moveit_visual_tools
  pluginlib
  ament_index_cpp
)

target_link_libraries(workcell_builder
  Qt5::Widgets
  Qt5::Concurrent
  Qt5::Svg
  yaml-cpp
  ${Boost_LIBRARIES}
)

target_include_directories(workcell_builder
  PRIVATE
    ${Boost_INCLUDE_DIRS}
)

set(WORKCELL_BUILDER_CANONICAL_ASSETS_DIR "${CMAKE_CURRENT_SOURCE_DIR}/assets")
set(WORKCELL_BUILDER_SECONDARY_ASSETS_DIR "${CMAKE_CURRENT_BINARY_DIR}/generated_assets")

add_custom_target(generate_workcell_builder_secondary_assets ALL
  COMMAND ${CMAKE_COMMAND} -E rm -rf ${WORKCELL_BUILDER_SECONDARY_ASSETS_DIR}
  COMMAND ${CMAKE_COMMAND} -E make_directory ${WORKCELL_BUILDER_SECONDARY_ASSETS_DIR}
  COMMAND ${CMAKE_COMMAND} -E copy_directory
    ${WORKCELL_BUILDER_CANONICAL_ASSETS_DIR}
    ${WORKCELL_BUILDER_SECONDARY_ASSETS_DIR}
  COMMENT "Generating secondary default-asset tree from canonical package assets")

add_dependencies(workcell_builder generate_workcell_builder_secondary_assets)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(workcell_link_deletion_test test/link_deletion_test.cpp)
  ament_add_gtest(workcell_directory_inspection_test test/workcell_directory_inspection_test.cpp)
  ament_add_gtest(workcell_scene_select_paths_test test/scene_select_paths_test.cpp src_scene_select_paths.cpp)
  ament_add_gtest(workcell_generated_environment_asset_writer_test
    test/generated_environment_asset_writer_test.cpp
    src_generated_stl_writer.cpp
    src_generated_environment_asset_writer.cpp
    src_object_placement_model.cpp
    gui/workcell_builder_ui_utils.cpp
    gui/loadobjects.cpp)
  ament_add_gtest(workcell_generate_yaml_end_effector_metadata_test
    test/generate_yaml_end_effector_metadata_test.cpp)
  ament_add_gtest(workcell_scene_status_test
    test/workcell_scene_status_test.cpp
    src_workcell_scene_status.cpp
    src_workcell_studio_scene_browser.cpp
    src_conveyor_pick_preview.cpp
    src_workcell_perception_snapshot.cpp
    src_task_intent_readiness.cpp
    src_class_routing_model.cpp
    src_planning_readiness.cpp
    src_workcell_zone_model.cpp
    src_workcell_perception_snapshot.cpp)
  ament_add_gtest(workcell_conveyor_pick_preview_test
    test/conveyor_pick_preview_test.cpp
    src_conveyor_pick_preview.cpp)
  ament_add_gtest(workcell_task_intent_readiness_test
    test/task_intent_readiness_test.cpp
    src_task_intent_readiness.cpp)
  ament_add_gtest(workcell_planning_readiness_test
    test/planning_readiness_test.cpp
    src_planning_readiness.cpp)
  ament_add_gtest(workcell_perception_snapshot_test
    test/workcell_perception_snapshot_test.cpp
    src_workcell_perception_snapshot.cpp
    src_task_intent_readiness.cpp
    src_class_routing_model.cpp
    src_planning_readiness.cpp
    src_conveyor_pick_preview.cpp
    src_workcell_camera_model.cpp
    src_workcell_zone_model.cpp)
  ament_add_gtest(workcell_robot_tool_compatibility_inference_test
    test/robot_tool_compatibility_inference_test.cpp
    src_robot_tool_compatibility.cpp)

  ament_add_gtest(workcell_workspace_validation_test
    test/workspace_validation_test.cpp
    src_workspace_validation.cpp)
  if(TARGET workcell_workspace_validation_test)
    target_link_libraries(workcell_workspace_validation_test Qt5::Widgets)
  endif()

  foreach(workcell_builder_yaml_boost_test
      workcell_task_intent_readiness_test
      workcell_planning_readiness_test
      workcell_perception_snapshot_test)
    if(TARGET ${workcell_builder_yaml_boost_test})
      target_link_libraries(${workcell_builder_yaml_boost_test}
        yaml-cpp
        ${Boost_LIBRARIES}
      )
      target_include_directories(${workcell_builder_yaml_boost_test}
        PRIVATE
          ${Boost_INCLUDE_DIRS}
          include
      )
    endif()
  endforeach()

  if(TARGET workcell_directory_inspection_test)
    target_link_libraries(workcell_directory_inspection_test
      ${Boost_LIBRARIES}
    )
    target_include_directories(workcell_directory_inspection_test
      PRIVATE
        ${Boost_INCLUDE_DIRS}
    )
  endif()

  if(TARGET workcell_generate_yaml_end_effector_metadata_test)
    ament_target_dependencies(workcell_generate_yaml_end_effector_metadata_test
      rclcpp
    )
    target_link_libraries(workcell_generate_yaml_end_effector_metadata_test
      yaml-cpp
      ${Boost_LIBRARIES}
    )
    target_include_directories(workcell_generate_yaml_end_effector_metadata_test
      PRIVATE
        ${Boost_INCLUDE_DIRS}
        include
        include/attributes
        include/yaml_parser
    )
  endif()

  if(TARGET workcell_conveyor_pick_preview_test)
    target_link_libraries(workcell_conveyor_pick_preview_test yaml-cpp ${Boost_LIBRARIES})
    target_include_directories(workcell_conveyor_pick_preview_test PRIVATE ${Boost_INCLUDE_DIRS} include)
  endif()

  if(TARGET workcell_scene_status_test)
    target_link_libraries(workcell_scene_status_test
      yaml-cpp
      ${Boost_LIBRARIES}
    )
    target_include_directories(workcell_scene_status_test
      PRIVATE
        ${Boost_INCLUDE_DIRS}
        include
    )
  endif()

  if(TARGET workcell_robot_tool_compatibility_inference_test)
    ament_target_dependencies(workcell_robot_tool_compatibility_inference_test
      rclcpp
    )
    target_link_libraries(workcell_robot_tool_compatibility_inference_test
      Qt5::Widgets
      yaml-cpp
      ${Boost_LIBRARIES}
    )
    target_include_directories(workcell_robot_tool_compatibility_inference_test
      PRIVATE
        ${Boost_INCLUDE_DIRS}
        include
        include/attributes
    )
  endif()

  if(TARGET workcell_generated_environment_asset_writer_test)
    target_link_libraries(workcell_generated_environment_asset_writer_test
      Qt5::Widgets
      yaml-cpp
      ${Boost_LIBRARIES}
    )
    target_include_directories(workcell_generated_environment_asset_writer_test
      PRIVATE
        ${Boost_INCLUDE_DIRS}
        include
        include/attributes
        include/yaml_parser
        gui
    )
  endif()

  if(TARGET workcell_scene_select_paths_test)
    ament_target_dependencies(workcell_scene_select_paths_test
      ament_index_cpp
      rclcpp
    )
    target_link_libraries(workcell_scene_select_paths_test
      ${Boost_LIBRARIES}
      yaml-cpp
    )
    target_include_directories(workcell_scene_select_paths_test
      PRIVATE
        ${Boost_INCLUDE_DIRS}
        include
        include/attributes
    )
  endif()

  find_package(launch_testing_ament_cmake REQUIRED)
  add_launch_test(test/test_ur5_2f_demo_launch.test.py TIMEOUT 180)

  find_package(Python3 REQUIRED COMPONENTS Interpreter)
  add_test(
    NAME workcell_builder_asset_tree_consistency
    COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test/check_asset_tree_sync.py
      --repo-root ${CMAKE_CURRENT_SOURCE_DIR}/../..
      --canonical workcell_builder/workcell_builder/assets
      --compare src/assets)
endif()

install(TARGETS workcell_builder
  EXPORT export_${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME}
  RUNTIME DESTINATION bin
)

install(DIRECTORY templates
  DESTINATION share/${PROJECT_NAME})

install(DIRECTORY gui/resources
  DESTINATION share/${PROJECT_NAME}/gui)

install(DIRECTORY assets
  DESTINATION share/${PROJECT_NAME})

install(PROGRAMS
  scripts/conveyor_sorting_live_preview_node.py
  scripts/publish_sample_epd_snapshot.py
  scripts/epd_to_workcell_snapshot_node.py
  DESTINATION lib/${PROJECT_NAME})

install(PROGRAMS
  ../../scripts/workcell_studio_layout_merge.py
  ../../scripts/validate_workcell_studio_generated_scene.py
  ../../scripts/workcell_studio_demo_mode.py
  ../../scripts/workcell_studio_preview_launch.py
  ../../scripts/run_workcell_studio_golden_flow.py
  DESTINATION share/${PROJECT_NAME}/scripts)

install(DIRECTORY ${WORKCELL_BUILDER_SECONDARY_ASSETS_DIR}/
  DESTINATION share/${PROJECT_NAME}/legacy_assets)

ament_package()
