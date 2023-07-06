cmake_minimum_required(VERSION 3.8)
project(behavior_tree_plugin)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_auto REQUIRED)

ament_auto_find_build_dependencies()

ament_auto_add_library(${PROJECT_NAME} SHARED DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/src)

# Workaround to allow deprecated header to build on both galactic and humble
if(${tf2_geometry_msgs_VERSION} VERSION_LESS 0.18.0)
  target_compile_definitions(behavior_tree_plugin PUBLIC USE_TF2_GEOMETRY_MSGS_DEPRECATED_HEADER)
endif()

pluginlib_export_plugin_description_file(traffic_simulator plugins.xml)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME})

ament_auto_package()
