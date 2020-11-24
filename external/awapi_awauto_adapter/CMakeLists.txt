cmake_minimum_required(VERSION 3.5)
project(awapi_awauto_adapter)

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

foreach(PACKAGE IN LISTS ${PROJECT_NAME}_DEPENDENCIES)
  if(PACKAGE)
    find_package(${PACKAGE} REQUIRED)
  endif()
endforeach()

# ------------------------------------------------------------------------------
#  libawapi_awauto_adapter
# ------------------------------------------------------------------------------

add_library(${PROJECT_NAME} SHARED
  src/awapi_awauto_adapter.cpp)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

ament_target_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_DEPENDENCIES})

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_DEPENDENCIES})

rclcpp_components_register_nodes(${PROJECT_NAME} "autoware_api::AutowareAutoAdapter")

install(
  TARGETS ${PROJECT_NAME}
  EXPORT  ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

# ------------------------------------------------------------------------------
#  awapi_awauto_adapter_node
# ------------------------------------------------------------------------------

add_executable(${PROJECT_NAME}_node
  src/${PROJECT_NAME}_node.cpp
  src/awapi_awauto_status_publisher.cpp
  src/awapi_vehicle_status_publisher.cpp
  src/awapi_lane_change_status_publisher.cpp
  src/awapi_obstacle_avoidance_status_publisher.cpp
)

target_link_libraries(${PROJECT_NAME}_node ${PROJECT_NAME})

install(
  TARGETS ${PROJECT_NAME}_node
  DESTINATION lib/${PROJECT_NAME})

# ------------------------------------------------------------------------------
#  make install
# ------------------------------------------------------------------------------

install(
  DIRECTORY include/
  DESTINATION include)

# ------------------------------------------------------------------------------
#  test
# ------------------------------------------------------------------------------

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  set(${PROJECT_NAME}_TEST_DEPENDENCIES ${${PROJECT_NAME}_DEPENDENCIES})
endif()

ament_package()
