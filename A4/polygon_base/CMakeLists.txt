cmake_minimum_required(VERSION 3.8)
project(polygon_base)

find_package(ament_cmake REQUIRED)

add_library(polygon_base INTERFACE)
target_include_directories(polygon_base INTERFACE
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
)

install(
  DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

install(
  TARGETS polygon_base
  EXPORT export_polygon_base
)

ament_export_targets(export_polygon_base HAS_LIBRARY_TARGET)

ament_package()
