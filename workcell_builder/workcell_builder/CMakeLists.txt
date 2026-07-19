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
option(WORKCELL_BUILDER_ALLOW_NATIVE_3D_FALLBACK "Allow the legacy native Scene3DViewportWidget Product View when Qt WebEngine is unavailable" OFF)

find_package(Qt5 COMPONENTS Widgets Concurrent Svg OpenGL Network REQUIRED)
find_package(Qt5 COMPONENTS WebEngineWidgets QUIET)
if(NOT Qt5WebEngineWidgets_FOUND AND NOT WORKCELL_BUILDER_ALLOW_NATIVE_3D_FALLBACK)
  message(FATAL_ERROR
    "embedded Web3D Product View is required by default for workcell_builder.\n"
    "Missing Qt component: Qt5 WebEngineWidgets.\n"
    "Ubuntu 22.04 / ROS 2 Humble install guidance: sudo apt install qtwebengine5-dev\n"
    "If you intentionally need the legacy compatibility path, configure with -DWORKCELL_BUILDER_ALLOW_NATIVE_3D_FALLBACK=ON.")
endif()
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
    gui/scene3d_visual_classification.cpp
    gui/scene3d_visual_classification.h
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
  Qt5::Network
  OpenGL::GL
  yaml-cpp
  ${Boost_LIBRARIES}
)

if(Qt5WebEngineWidgets_FOUND)
  target_compile_definitions(workcell_builder PRIVATE WORKCELL_BUILDER_HAS_WEBENGINE=1)
  target_link_libraries(workcell_builder Qt5::WebEngineWidgets)
elseif(WORKCELL_BUILDER_ALLOW_NATIVE_3D_FALLBACK)
  target_compile_definitions(workcell_builder PRIVATE WORKCELL_BUILDER_NATIVE_3D_FALLBACK=1)
endif()

if(assimp_FOUND)
  target_compile_definitions(workcell_builder PRIVATE WORKCELL_BUILDER_HAS_ASSIMP=1)
  if(TARGET assimp::assimp)
    target_link_libraries(workcell_builder assimp::assimp)
  elseif(TARGET Assimp::assimp)
    target_link_libraries(workcell_builder Assimp::assimp)
  else()
    target_link_libraries(workcell_builder ${ASSIMP_LIBRARIES} ${assimp_LIBRARIES})
    target_include_directories(workcell_builder PRIVATE ${ASSIMP_INCLUDE_DIRS} ${assimp_INCLUDE_DIRS})
  endif()
endif()

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
    src_placed_object_urdf_render.cpp
    src_object_placement_model.cpp
    gui/workcell_builder_ui_utils.cpp
    gui/loadobjects.cpp)
  ament_add_gtest(workcell_generate_yaml_end_effector_metadata_test
    test/generate_yaml_end_effector_metadata_test.cpp)
  ament_add_gtest(workcell_scene_xacro_tool_attachment_test
    test/scene_xacro_tool_attachment_test.cpp)
  ament_add_gtest(workcell_scene_status_test
    test/workcell_scene_status_test.cpp
    src_workcell_scene_status.cpp
    src_supported_scene_readiness_loader.cpp
    src_workcell_studio_scene_browser.cpp
    src_workcell_warning_once.cpp
    src_conveyor_pick_preview.cpp
    src_workcell_perception_snapshot.cpp
    src_task_intent_readiness.cpp
    src_class_routing_model.cpp
    src_planning_readiness.cpp
    src_workcell_zone_model.cpp
    src_workcell_perception_snapshot.cpp
    src_workcell_yaml_utils.cpp)
  ament_add_gtest(workcell_yaml_utils_test
    test/workcell_yaml_utils_test.cpp
    src_workcell_yaml_utils.cpp)
  ament_add_gtest(workcell_conveyor_pick_preview_test
    test/conveyor_pick_preview_test.cpp
    src_conveyor_pick_preview.cpp)
  # Source-text-only regression test: intentionally does not compile
  # Scene3DViewportWidget. Real widget compilation coverage remains in
  # workcell_scene3d_stl_parser_test and the main workcell_builder target.
  ament_add_gtest(workcell_scene3d_mesh_preview_regression_test
    test/scene3d_mesh_preview_regression_test.cpp)
  if(TARGET workcell_scene3d_mesh_preview_regression_test)
    target_compile_definitions(workcell_scene3d_mesh_preview_regression_test PRIVATE
      WORKCELL_BUILDER_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}")
  endif()
  ament_add_gtest(workcell_task_intent_readiness_test
    test/task_intent_readiness_test.cpp
    src_task_intent_readiness.cpp)
  ament_add_gtest(workcell_planning_readiness_test
    test/planning_readiness_test.cpp
    src_planning_readiness.cpp)
  ament_add_gtest(workcell_perception_snapshot_test
    test/workcell_perception_snapshot_test.cpp
    src_workcell_perception_snapshot.cpp
    src_workcell_warning_once.cpp
    src_task_intent_readiness.cpp
    src_class_routing_model.cpp
    src_planning_readiness.cpp
    src_conveyor_pick_preview.cpp
    src_workcell_camera_model.cpp
    src_workcell_zone_model.cpp
    src_workcell_yaml_utils.cpp)
  ament_add_gtest(workcell_robot_tool_compatibility_inference_test
    test/robot_tool_compatibility_inference_test.cpp
    src_robot_tool_compatibility.cpp)

  ament_add_gtest(workcell_new_cell_wizard_test
    test/new_cell_wizard_test.cpp
    gui/new_cell_wizard.cpp)

  ament_add_gtest(workcell_workspace_validation_test
    test/workspace_validation_test.cpp
    src_workspace_validation.cpp)

  ament_add_gtest(workcell_placed_object_preview_writer_test
    test/placed_object_preview_writer_test.cpp
    gui/placed_object_preview_writer.cpp
    src_placed_object_urdf_render.cpp
    src_object_placement_model.cpp)

  ament_add_gtest(workcell_scene3d_stl_parser_test
    test/scene3d_stl_parser_test.cpp
    gui/scene3d_viewport_widget.cpp
    gui/scene3d_visual_classification.cpp)
  ament_add_gtest(workcell_scene3d_render_role_classifier_test
    test/scene3d_render_role_classifier_test.cpp
    gui/scene3d_viewport_widget.cpp
    gui/scene3d_visual_classification.cpp)
  ament_add_gtest(workcell_scene3d_ur5_baked_transform_test
    test/scene3d_ur5_baked_transform_test.cpp
    gui/scene3d_viewport_widget.cpp
    gui/scene3d_visual_classification.cpp)

  ament_add_gtest(workcell_command_builders_test
    test/command_builders_test.cpp
    src_workcell_builder_command_builders.cpp)

  ament_add_gtest(workcell_rviz_preview_metadata_command_test
    test/rviz_preview_metadata_command_test.cpp
    src_rviz_preview_runner.cpp
    src_workcell_studio_scene_browser.cpp
    src_workcell_warning_once.cpp
    src_workcell_yaml_utils.cpp)

  ament_add_gtest(workcell_scene_preview_widget_ui_test
    test/scene_preview_widget_ui_test.cpp
    gui/scene_preview_widget.cpp
    gui/scene3d_viewport_widget.cpp
    gui/scene3d_visual_classification.cpp)
  ament_add_gtest(workcell_studio_canvas_model_mesh_test
    test/workcell_studio_canvas_model_mesh_test.cpp
    src_workcell_studio_canvas_model.cpp
    src_workcell_warning_once.cpp
    src_workcell_yaml_utils.cpp)
  ament_add_gtest(workcell_asset_catalog_discovery_test
    test/asset_catalog_discovery_test.cpp
    gui/asset_catalog_discovery.cpp
    src_workcell_warning_once.cpp
    src_workcell_yaml_utils.cpp)
  ament_add_gtest(workcell_visual_mesh_source_resolver_test
    test/visual_mesh_source_resolver_test.cpp
    src_visual_mesh_source_resolver.cpp)
  ament_add_gtest(workcell_transform_clipboard_utils_test
    test/transform_clipboard_utils_test.cpp
    gui/transform_clipboard_utils.cpp)
  ament_add_gtest(workcell_preview_item_suppression_test
    test/preview_item_suppression_test.cpp
    gui/preview_item_suppression.cpp)
  ament_add_gtest(workcell_scene3d_product_view_layer_defaults_test
    test/scene3d_product_view_layer_defaults_test.cpp
    gui/scene3d_candidate_assembly.cpp
    gui/scene3d_visual_classification.cpp)
  ament_add_gtest(workcell_layout_serialization_contract_test
    test/layout_serialization_contract_test.cpp)
  ament_add_gtest(workcell_studio_layout_editor_test
    test/workcell_studio_layout_editor_test.cpp
    src_workcell_studio_layout_editor.cpp)
  ament_add_gtest(workcell_mainwindow_rviz_preview_compile_test
    test/mainwindow_rviz_preview_compile_test.cpp)
  if(TARGET workcell_scene_preview_widget_ui_test)
    target_link_libraries(workcell_scene_preview_widget_ui_test Qt5::Widgets Qt5::OpenGL Qt5::Network OpenGL::GL)
    if(Qt5WebEngineWidgets_FOUND)
      target_compile_definitions(workcell_scene_preview_widget_ui_test PRIVATE WORKCELL_BUILDER_HAS_WEBENGINE=1)
      target_link_libraries(workcell_scene_preview_widget_ui_test Qt5::WebEngineWidgets)
    endif()
  endif()

  if(TARGET workcell_workspace_validation_test)
    target_link_libraries(workcell_workspace_validation_test Qt5::Widgets)
  endif()

  if(TARGET workcell_rviz_preview_metadata_command_test)
    target_link_libraries(workcell_rviz_preview_metadata_command_test Qt5::Core yaml-cpp ${Boost_LIBRARIES})
  endif()

  if(TARGET workcell_scene3d_stl_parser_test)
    target_link_libraries(workcell_scene3d_stl_parser_test
      Qt5::Widgets
      Qt5::OpenGL
      OpenGL::GL
    )
    target_include_directories(workcell_scene3d_stl_parser_test PRIVATE gui)
  endif()

  if(TARGET workcell_scene3d_render_role_classifier_test)
    target_link_libraries(workcell_scene3d_render_role_classifier_test
      Qt5::Widgets
      Qt5::OpenGL
      OpenGL::GL
    )
    target_include_directories(workcell_scene3d_render_role_classifier_test PRIVATE gui)
  endif()

  if(TARGET workcell_scene3d_ur5_baked_transform_test)
    target_link_libraries(workcell_scene3d_ur5_baked_transform_test
      Qt5::Widgets
      Qt5::OpenGL
      OpenGL::GL
    )
    target_include_directories(workcell_scene3d_ur5_baked_transform_test PRIVATE gui)
  endif()

  if(TARGET workcell_command_builders_test)
    target_link_libraries(workcell_command_builders_test
      Qt5::Core
      yaml-cpp
    )
  endif()
  if(TARGET workcell_asset_catalog_discovery_test)
    target_link_libraries(workcell_asset_catalog_discovery_test yaml-cpp ${Boost_LIBRARIES})
    target_include_directories(workcell_asset_catalog_discovery_test PRIVATE gui)
  endif()
  if(TARGET workcell_visual_mesh_source_resolver_test)
    target_link_libraries(workcell_visual_mesh_source_resolver_test Qt5::Core ${Boost_LIBRARIES})
    target_include_directories(workcell_visual_mesh_source_resolver_test PRIVATE ${Boost_INCLUDE_DIRS} include)
    target_compile_definitions(workcell_visual_mesh_source_resolver_test PRIVATE
      WORKCELL_BUILDER_REPO_ROOT="${CMAKE_CURRENT_SOURCE_DIR}/../..")
  endif()

  if(TARGET workcell_transform_clipboard_utils_test)
    target_link_libraries(workcell_transform_clipboard_utils_test Qt5::Core)
    target_include_directories(workcell_transform_clipboard_utils_test PRIVATE gui)
  endif()

  if(TARGET workcell_preview_item_suppression_test)
    target_link_libraries(workcell_preview_item_suppression_test Qt5::Widgets)
    target_include_directories(workcell_preview_item_suppression_test PRIVATE gui)
  endif()

  if(TARGET workcell_scene3d_product_view_layer_defaults_test)
    target_link_libraries(workcell_scene3d_product_view_layer_defaults_test Qt5::Widgets)
    target_include_directories(workcell_scene3d_product_view_layer_defaults_test PRIVATE gui)
  endif()

  if(TARGET workcell_studio_canvas_model_mesh_test)
    target_link_libraries(workcell_studio_canvas_model_mesh_test yaml-cpp ${Boost_LIBRARIES})
    target_include_directories(workcell_studio_canvas_model_mesh_test PRIVATE ${Boost_INCLUDE_DIRS} include)
    target_compile_definitions(workcell_studio_canvas_model_mesh_test PRIVATE
      WORKCELL_BUILDER_REPO_ROOT="${CMAKE_CURRENT_SOURCE_DIR}/../..")
  endif()

  if(TARGET workcell_new_cell_wizard_test)
    target_link_libraries(workcell_new_cell_wizard_test Qt5::Widgets ${Boost_LIBRARIES})
    target_include_directories(workcell_new_cell_wizard_test PRIVATE ${Boost_INCLUDE_DIRS})
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

  if(TARGET workcell_yaml_utils_test)
    target_link_libraries(workcell_yaml_utils_test
      yaml-cpp
      ${Boost_LIBRARIES}
    )
    target_include_directories(workcell_yaml_utils_test
      PRIVATE
        ${Boost_INCLUDE_DIRS}
        include
        include/attributes
        include/yaml_parser
    )
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

  if(TARGET workcell_scene_xacro_tool_attachment_test)
    ament_target_dependencies(workcell_scene_xacro_tool_attachment_test
      ament_index_cpp
    )
    target_link_libraries(workcell_scene_xacro_tool_attachment_test
      ${Boost_LIBRARIES}
    )
    target_include_directories(workcell_scene_xacro_tool_attachment_test
      PRIVATE
        ${Boost_INCLUDE_DIRS}
        include
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


  if(TARGET workcell_studio_layout_editor_test)
    target_link_libraries(workcell_studio_layout_editor_test yaml-cpp ${Boost_LIBRARIES})
    target_include_directories(workcell_studio_layout_editor_test PRIVATE ${Boost_INCLUDE_DIRS} include)
  endif()

  if(TARGET workcell_mainwindow_rviz_preview_compile_test)
    target_link_libraries(workcell_mainwindow_rviz_preview_compile_test Qt5::Widgets)
    target_include_directories(workcell_mainwindow_rviz_preview_compile_test
      PRIVATE
        ${Boost_INCLUDE_DIRS}
        include
        gui
    )
  endif()

  set(workcell_builder_registered_cpp_tests
    link_deletion_test.cpp
    workcell_directory_inspection_test.cpp
    scene_select_paths_test.cpp
    generated_environment_asset_writer_test.cpp
    generate_yaml_end_effector_metadata_test.cpp
    scene_xacro_tool_attachment_test.cpp
    workcell_scene_status_test.cpp
    workcell_yaml_utils_test.cpp
    conveyor_pick_preview_test.cpp
    scene3d_mesh_preview_regression_test.cpp
    task_intent_readiness_test.cpp
    planning_readiness_test.cpp
    workcell_perception_snapshot_test.cpp
    robot_tool_compatibility_inference_test.cpp
    new_cell_wizard_test.cpp
    workspace_validation_test.cpp
    placed_object_preview_writer_test.cpp
    scene3d_stl_parser_test.cpp
    scene3d_render_role_classifier_test.cpp
    scene3d_ur5_baked_transform_test.cpp
    scene3d_product_view_layer_defaults_test.cpp
    command_builders_test.cpp
    scene_preview_widget_ui_test.cpp
    workcell_studio_canvas_model_mesh_test.cpp
    asset_catalog_discovery_test.cpp
    transform_clipboard_utils_test.cpp
    preview_item_suppression_test.cpp
    layout_serialization_contract_test.cpp
    workcell_studio_layout_editor_test.cpp
    mainwindow_rviz_preview_compile_test.cpp
    rviz_preview_metadata_command_test.cpp
    visual_mesh_source_resolver_test.cpp
  )

  file(GLOB workcell_builder_discovered_cpp_tests RELATIVE ${CMAKE_CURRENT_SOURCE_DIR}/test ${CMAKE_CURRENT_SOURCE_DIR}/test/*.cpp)
  list(SORT workcell_builder_registered_cpp_tests)
  list(SORT workcell_builder_discovered_cpp_tests)

  if(NOT workcell_builder_registered_cpp_tests STREQUAL workcell_builder_discovered_cpp_tests)
    message(FATAL_ERROR
      "Orphan or unregistered test sources detected. Registered: ${workcell_builder_registered_cpp_tests}; Discovered: ${workcell_builder_discovered_cpp_tests}")
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
)

install(PROGRAMS
  scripts/workcell_builder
  DESTINATION bin)

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
  ../../scripts/workcell_builder_task_zone_preview_node.py
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
