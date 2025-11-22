cmake_minimum_required(VERSION 3.10)
project(tesseract_support)

find_package(ament_cmake REQUIRED)

# Install resource index entry so the package can be located via package:// URLs
install(FILES resource/${PROJECT_NAME}
        DESTINATION share/ament_index/resource_index/packages)

# Install package manifest
install(FILES package.xml
        DESTINATION share/${PROJECT_NAME})

# Install support data directories if they contain any content. The
# DIRECTORY form is tolerant of empty directories, which keeps the
# workspace from failing when the package is used purely as a resource
# locator overlay.
install(DIRECTORY meshes urdf srdf config
        DESTINATION share/${PROJECT_NAME}
        FILES_MATCHING PATTERN "*")

ament_package()
