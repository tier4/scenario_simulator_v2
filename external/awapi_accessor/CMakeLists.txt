cmake_minimum_required(VERSION 3.5)
project(awapi_accessor)

if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

execute_process(
  COMMAND grep -e "<depend>" ${CMAKE_CURRENT_SOURCE_DIR}/package.xml
  COMMAND sed -E "s/(.*)<(.*)>(.*)<\\/\\2>/\\3;/g"
  COMMAND tr -d "\n"
  OUTPUT_VARIABLE ${PROJECT_NAME}_DEPENDENCIES)

find_package(autoware_api_msgs REQUIRED)
find_package(geometry_msgs     REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rclcpp            REQUIRED)
find_package(std_msgs          REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros           REQUIRED)

# ------------------------------------------------------------------------------
#  libawapi_accessor
# ------------------------------------------------------------------------------

add_library(${PROJECT_NAME} SHARED
  src/awapi_accessor.cpp)
# ament_export_libraries(${PROJECT_NAME})

rclcpp_components_register_nodes(${PROJECT_NAME} "autoware_api::Accessor")

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
# ament_export_include_directories(include)

ament_target_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_DEPENDENCIES})
# ament_export_dependencies(${PROJECT_NAME})

# target_link_libraries(${PROJECT_NAME}
#   )

# ------------------------------------------------------------------------------
#  make install
# ------------------------------------------------------------------------------

install(
  DIRECTORY include
  DESTINATION share/${PROJECT_NAME})

install(
  TARGETS ${PROJECT_NAME}
  EXPORT  ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

# ------------------------------------------------------------------------------
#  test
# ------------------------------------------------------------------------------

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  set(${PROJECT_NAME}_TEST_DEPENDENCIES ${${PROJECT_NAME}_DEPENDENCIES})
endif()

ament_package()
