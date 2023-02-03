cmake_minimum_required(VERSION 3.5)
project(concealer)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_auto REQUIRED)

ament_auto_find_build_dependencies()

# for backward compatibility
# DIRTY HACK! : this line is needed to build without sourcing install/setup.bash
set(autoware_adapi_v1_msgs_CMAKE_MODULE_PATH "${CMAKE_INSTALL_PREFIX}/../autoware_adapi_v1_msgs/share/autoware_adapi_v1_msgs/cmake")
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${autoware_adapi_v1_msgs_CMAKE_MODULE_PATH})
find_package(autoware_adapi_v1_msgs)
if( autoware_adapi_v1_msgs_FOUND )
  # DIRTY HACK! : ament_cmake_auto
  list(APPEND ${PROJECT_NAME}_BUILD_DEPENDS autoware_adapi_v1_msgs)
  list(APPEND ${PROJECT_NAME}_BUILD_EXPORT_DEPENDS autoware_adapi_v1_msgs)
  list(APPEND ${PROJECT_NAME}_EXEC_DEPENDS autoware_adapi_v1_msgs)

  list(APPEND ${PROJECT_NAME}_FOUND_BUILD_DEPENDS autoware_adapi_v1_msgs)
  list(APPEND ${PROJECT_NAME}_FOUND_DEFINITIONS ${autoware_adapi_v1_msgs_DEFINITIONS})
  list(APPEND ${PROJECT_NAME}_FOUND_INCLUDE_DIRS ${autoware_adapi_v1_msgs_INCLUDE_DIRS})
  list(APPEND ${PROJECT_NAME}_FOUND_LIBRARIES ${autoware_adapi_v1_msgs_LIBRARIES})
endif()

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/autoware.cpp
  src/autoware_universe.cpp
  src/cooperator.cpp
  src/execute.cpp
  src/minimum_risk_maneuver_merger.cpp
  src/task_queue.cpp)

# workaround to allow deprecated header to build on both galactic and humble
if(${tf2_geometry_msgs_VERSION} VERSION_LESS 0.18.0)
  target_compile_definitions(${PROJECT_NAME} PUBLIC
    USE_TF2_GEOMETRY_MSGS_DEPRECATED_HEADER
  )
endif()

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package()
