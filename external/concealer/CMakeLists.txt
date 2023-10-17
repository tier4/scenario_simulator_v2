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

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/autoware.cpp
  src/autoware_universe.cpp
  src/execute.cpp
  src/field_operator_application.cpp
  src/field_operator_application_for_autoware_universe.cpp
  src/is_package_exists.cpp
  src/task_queue.cpp)

target_link_libraries(${PROJECT_NAME}
  atomic)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package()
