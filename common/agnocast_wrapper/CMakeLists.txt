cmake_minimum_required(VERSION 3.5)

project(agnocast_wrapper)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_auto REQUIRED)

if(DEFINED ENV{ENABLE_AGNOCAST_SIMULATOR} AND "$ENV{ENABLE_AGNOCAST_SIMULATOR}" STREQUAL "1")
  message(WARNING "Building with agnocastlib dependency")
  find_package(agnocastlib REQUIRED)
else()
  message(WARNING "Building without agnocastlib dependency")
endif()

ament_auto_find_build_dependencies()

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package(CONFIG_EXTRAS "cmake/agnocast_wrapper_setup_target.cmake")
