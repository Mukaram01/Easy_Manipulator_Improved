cmake_minimum_required(VERSION 3.5)
project(ur5_airpick4_test)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

function(install_scene_directory_if_present directory_name)
  if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/${directory_name}")
    install(DIRECTORY ${directory_name}
      DESTINATION share/${PROJECT_NAME}
    )
  endif()
endfunction()

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY urdf
  DESTINATION share/${PROJECT_NAME}
)

install_scene_directory_if_present(config)
install_scene_directory_if_present(generated)
install_scene_directory_if_present(layout)

install(FILES
  cell_definition.yaml
  environment.yaml
  scene_manifest.yaml
  DESTINATION share/${PROJECT_NAME}
)
ament_package()
