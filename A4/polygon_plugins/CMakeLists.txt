cmake_minimum_required(VERSION 3.8)
project(polygon_plugins)

find_package(ament_cmake REQUIRED)
find_package(polygon_base REQUIRED)
find_package(pluginlib REQUIRED)

add_library(polygon_plugins SHARED src/polygon_plugins.cpp)
target_link_libraries(polygon_plugins PUBLIC
  pluginlib::pluginlib
  polygon_base::polygon_base
)

pluginlib_export_plugin_description_file(polygon_base polygon_plugins.xml)

install(
  TARGETS polygon_plugins
  EXPORT export_polygon_plugins
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

add_executable(polygon_loader src/polygon_loader.cpp)
target_link_libraries(polygon_loader PUBLIC
  pluginlib::pluginlib
  polygon_base::polygon_base
)

install(TARGETS
  polygon_loader
  DESTINATION lib/${PROJECT_NAME})

ament_package()
