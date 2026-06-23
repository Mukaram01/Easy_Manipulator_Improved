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
find_package(Qt5 COMPONENTS Widgets Concurrent Svg OpenGL REQUIRED)
find_package(OpenGL REQUIRED)
find_package(assimp QUIET)

include_directories(include)
include_directories(include/attributes)
include_directories(include/yaml_parser)
include_directories(gui)

add_executable(workcell_builder
    gui/main.cpp
    gui/mainwindow.cpp
    gui/transform_clipboard_utils.cpp
    gui/mainwindow.h
    gui/mainwindow.ui
    gui/scene_preview_widget.cpp
    gui/scene_preview_widget.h
    gui/scene3d_viewport_widget.cpp
    gui/scene3d_viewport_widget.h
    gui/scene3d_candidate_assembly.cpp
    gui/scene3d_candidate_assembly.h
    gui/preview_item_suppression.cpp
    gui/preview_item_suppression.h
    src_scene3d_visual_backend.cpp
    include/scene3d_visual_backend.hpp

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

    gui/loadobjects.cpp
    gui/loadobjects.h
    gui/loadobjects.ui

    src_scene_select_paths.cpp
    src_asset_discovery_helper.cpp
    src_generated_stl_writer.cpp
    src_generated_environment_asset_writer.cpp
    src_placed_object_urdf_render.cpp
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
    src_supported_scene_readiness_loader.cpp
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
    src_workcell_studio_id_utils.cpp
    src_workcell_warning_once.cpp
    src_external_asset_importer.cpp
    gui/asset_picker_dialog.cpp
    gui/object_placement_dialog.cpp
    gui/object_placement_yaml_io.cpp
    gui/rviz_pose_feedback_importer.cpp
    gui/environment_layout_editor.cpp
    gui/placed_object_preview_writer.cpp
    gui/external_asset_import_dialog.cpp
    gui/workcell_builder_ui_utils.cpp
    gui/startup_dialog.cpp
    gui/asset_catalog_model.cpp
    gui/asset_catalog_discovery.cpp
    gui/conveyor_sorting_scenario_wizard.cpp
    gui/conveyor_sorting_scenario_wizard.h
    gui/conveyor_sorting_scenario_wizard.ui
    gui/conveyor_sorting_run_console.cpp
    gui/conveyor_sorting_run_console.h
    gui/conveyor_sorting_run_console.ui
    gui/new_cell_wizard.cpp
    gui/new_cell_wizard.h

    include/asset_picker_dialog.hpp
    include/object_placement_dialog.hpp
    include/object_placement_yaml_io.hpp
    include/rviz_pose_feedback_importer.hpp
    include/environment_layout_editor.hpp
    include/placed_object_preview_writer.hpp
    include/external_asset_import_dialog.hpp
    include/workspace_validation.hpp
    src_workspace_validation.cpp
    src_workcell_builder_command_builders.cpp
    src_rviz_preview_runner.cpp
    src_visual_mesh_source_resolver.cpp
    include/rviz_preview_runner.hpp
    include/visual_mesh_source_resolver.hpp)
target_sources(workcell_builder PRIVATE src_workcell_yaml_utils.cpp)

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
  Qt5::OpenGL
  OpenGL::GL
  yaml-cpp
  ${Boost_LIBRARIES}
)

if(assimp_FOUND)
  target_compile_definitions(workcell_builder PRIVATE WORKCELL_BUILDER_HAS_ASSIMP=1)
  target_link_libraries(workcell_builder ${ASSIMP_LIBRARIES})
  target_include_directories(workcell_builder PRIVATE ${ASSIMP_INCLUDE_DIRS})
endif()

install(TARGETS workcell_builder
  DESTINATION bin)
install(TARGETS workcell_builder
  DESTINATION lib/${PROJECT_NAME})

ament_package()

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(workcell_builder_python_tests ${CMAKE_CURRENT_SOURCE_DIR}/test)
  ament_add_gtest(scene3d_candidate_assembly_test test/scene3d_candidate_assembly_test.cpp gui/scene3d_candidate_assembly.cpp gui/preview_item_suppression.cpp)
  target_include_directories(scene3d_candidate_assembly_test PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
  target_link_libraries(scene3d_candidate_assembly_test Qt5::Widgets Qt5::OpenGL OpenGL::GL yaml-cpp)
  ament_add_gtest(preview_item_suppression_test test/preview_item_suppression_test.cpp gui/preview_item_suppression.cpp)
  target_include_directories(preview_item_suppression_test PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
  target_link_libraries(preview_item_suppression_test Qt5::Widgets Qt5::OpenGL OpenGL::GL yaml-cpp)
  ament_add_gtest(scene3d_mesh_preview_regression_test test/scene3d_mesh_preview_regression_test.cpp gui/scene3d_viewport_widget.cpp gui/scene_preview_widget.cpp gui/scene3d_candidate_assembly.cpp gui/preview_item_suppression.cpp src_visual_mesh_source_resolver.cpp)
  target_include_directories(scene3d_mesh_preview_regression_test PRIVATE ${CMAKE_CURRENT_SOURCE_DIR} ${CMAKE_CURRENT_SOURCE_DIR}/include)
  target_link_libraries(scene3d_mesh_preview_regression_test Qt5::Widgets Qt5::OpenGL Qt5::Svg OpenGL::GL yaml-cpp ${Boost_LIBRARIES})
  if(assimp_FOUND)
    target_compile_definitions(scene3d_mesh_preview_regression_test PRIVATE WORKCELL_BUILDER_HAS_ASSIMP=1)
    target_link_libraries(scene3d_mesh_preview_regression_test ${ASSIMP_LIBRARIES})
    target_include_directories(scene3d_mesh_preview_regression_test PRIVATE ${ASSIMP_INCLUDE_DIRS})
  endif()
  ament_add_gtest(transform_clipboard_utils_test test/transform_clipboard_utils_test.cpp gui/transform_clipboard_utils.cpp)
  target_include_directories(transform_clipboard_utils_test PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
  target_link_libraries(transform_clipboard_utils_test Qt5::Widgets yaml-cpp)
  ament_add_gtest(layout_serialization_contract_test test/layout_serialization_contract_test.cpp)
  target_link_libraries(layout_serialization_contract_test yaml-cpp)
endif()
