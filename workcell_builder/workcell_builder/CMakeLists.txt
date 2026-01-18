cmake_minimum_required(VERSION 3.5)

project(workcell_builder LANGUAGES CXX)

set(CMAKE_INCLUDE_CURRENT_DIR ON)

set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++17
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
find_package(Qt5 COMPONENTS Widgets Concurrent REQUIRED)

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
    gui/loadobjects.ui)

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
  yaml-cpp
  ${Boost_LIBRARIES}
)

target_include_directories(workcell_builder
  PRIVATE
    ${Boost_INCLUDE_DIRS}
)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(workcell_link_deletion_test test/link_deletion_test.cpp)

  find_package(launch_testing_ament_cmake REQUIRED)
  add_launch_test(test/test_ur5_2f_demo_launch.test.py TIMEOUT 180)
endif()

install(TARGETS workcell_builder
  EXPORT export_${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME}
  RUNTIME DESTINATION bin
)

install(DIRECTORY templates
  DESTINATION share/${PROJECT_NAME})

install(DIRECTORY assets
  DESTINATION share/${PROJECT_NAME})

ament_package()
