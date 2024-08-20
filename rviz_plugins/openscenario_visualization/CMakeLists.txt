cmake_minimum_required(VERSION 3.5)
project(openscenario_visualization)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Werror -Wextra -Wpedantic)
endif()

# find dependencies
find_package(Eigen3 REQUIRED)
find_package(Qt5 ${rviz_QT_VERSION} REQUIRED Core Widgets)
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

# Qt settings
set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
add_definitions(-DQT_NO_KEYWORDS)

# Include directories
include_directories(include
  ${EIGEN3_INCLUDE_DIR}
  ${Qt5Core_INCLUDE_DIRS}
  ${Qt5Widgets_INCLUDE_DIRS}
  SYSTEM
  ${OGRE_INCLUDE_DIRS})

add_definitions(-DQT_NO_KEYWORDS)

qt5_wrap_ui(UIC_FILES src/ui/context_panel_plugin.ui)

ament_auto_add_library(openscenario_visualization_rviz_plugin SHARED
  include/openscenario_visualization/context_panel_plugin.hpp
  src/context_panel_plugin.cpp
  ${UIC_FILES}
  include/openscenario_visualization_condition_groups_plugin/jsk_overlay_utils.hpp
  src/openscenario_visualization_condition_groups_plugin/jsk_overlay_utils.cpp
  include/openscenario_visualization_condition_groups_plugin/openscenario_visualization_condition_groups_plugin.hpp
  src/openscenario_visualization_condition_groups_plugin/openscenario_visualization_condition_groups_plugin.cpp
  include/openscenario_visualization_condition_groups_plugin/openscenario_visualization_parameters_plugin.hpp
  src/openscenario_visualization_condition_groups_plugin/openscenario_visualization_parameters_plugin.cpp
)

target_include_directories(openscenario_visualization_rviz_plugin PRIVATE "${OGRE_PREFIX_DIR}/include")

install(DIRECTORY icons DESTINATION share/openscenario_visualization)
install(DIRECTORY include/ DESTINATION include)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

pluginlib_export_plugin_description_file(rviz_common plugins.xml)

ament_auto_package()
