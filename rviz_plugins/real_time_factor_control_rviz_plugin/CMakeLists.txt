cmake_minimum_required(VERSION 3.14)

project(real_time_factor_control_rviz_plugin)

find_package(ament_cmake_auto REQUIRED)
find_package(Qt5 REQUIRED Core Widgets)

ament_auto_find_build_dependencies()

set(QT_LIBRARIES Qt5::Widgets)
set(CMAKE_AUTOMOC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)

ament_auto_add_library(${PROJECT_NAME} SHARED src/real_time_factor_slider.cpp)

target_compile_definitions(${PROJECT_NAME} PUBLIC QT_NO_KEYWORDS)

target_include_directories(${PROJECT_NAME} PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/src)

target_link_libraries(${PROJECT_NAME} ${QT_LIBRARIES})

# Export the plugin to be imported by rviz2
pluginlib_export_plugin_description_file(rviz_common plugins/plugin_description.xml)

ament_auto_package()
