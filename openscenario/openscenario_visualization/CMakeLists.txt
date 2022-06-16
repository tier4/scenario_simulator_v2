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
  add_compile_options(-Wall -Wpedantic -Werror) # -Wextra
endif()

# find dependencies
find_package(Eigen3 REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(class_loader REQUIRED)
find_package(openscenario_interpreter_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(quaternion_operation REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rviz2 REQUIRED)
find_package(rviz_common REQUIRED)
find_package(rviz_default_plugins REQUIRED)
find_package(traffic_simulator REQUIRED)
find_package(traffic_simulator_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

ament_export_dependencies(class_loader REQUIRED)
ament_export_dependencies(pluginlib REQUIRED)
ament_export_dependencies(rclcpp REQUIRED)
ament_export_dependencies(std_msgs REQUIRED)
ament_export_dependencies(rviz2 REQUIRED)
ament_export_dependencies(Eigen3 REQUIRED)

include_directories(
  include
  ${EIGEN3_INCLUDE_DIR}
)

add_library(openscenario_visualization_component SHARED
  src/openscenario_visualization_component.cpp
)
ament_target_dependencies(openscenario_visualization_component
  rclcpp
  rclcpp_components
  rclcpp_lifecycle
  traffic_simulator
  visualization_msgs
  traffic_simulator_msgs
  quaternion_operation
)
add_executable(openscenario_visualization_node
  src/openscenario_visualization_node.cpp
)
target_link_libraries(openscenario_visualization_node openscenario_visualization_component)
ament_target_dependencies(openscenario_visualization_node
  quaternion_operation
  rclcpp
  rclcpp_components
  rclcpp_lifecycle
  traffic_simulator
  traffic_simulator_msgs
  visualization_msgs
)

###############
# Qt settings #
###############
find_package(Qt5 ${rviz_QT_VERSION} REQUIRED Core Widgets)
macro(qt_wrap_ui)
  qt5_wrap_ui(${ARGN})
endmacro()
include_directories(${Qt5Core_INCLUDE_DIRS})
include_directories(${Qt5Widgets_INCLUDE_DIRS})
add_definitions(-DQT_NO_KEYWORDS)
set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)

pluginlib_export_plugin_description_file(rviz_common plugins.xml)

include_directories(
  include
)

qt5_wrap_ui(UIC_FILES
  src/ui/context_panel_plugin.ui
)

add_library(openscenario_visualization_rviz_plugin SHARED
  include/openscenario_visualization/context_panel_plugin.hpp
  src/context_panel_plugin.cpp
  ${UIC_FILES})
ament_target_dependencies(openscenario_visualization_rviz_plugin
  rviz2
  Qt5
  pluginlib
  rclcpp
  openscenario_interpreter_msgs
  rviz_common
)
target_include_directories(openscenario_visualization_rviz_plugin PRIVATE "${OGRE_PREFIX_DIR}/include")

install(DIRECTORY icons DESTINATION share/openscenario_visualization)
install(DIRECTORY include/ DESTINATION include)

install(TARGETS openscenario_visualization_rviz_plugin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

install(TARGETS openscenario_visualization_node DESTINATION lib/openscenario_visualization)

install(TARGETS
  openscenario_visualization_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
ament_export_libraries(openscenario_visualization_component)
rclcpp_components_register_nodes(openscenario_visualization_component
  "openscenario_visualization::OpenscenarioVisualizationComponent")

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
