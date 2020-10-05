cmake_minimum_required(VERSION 3.5)
project(spline_interpolation)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

include_directories(
  include
)

add_library(spline_interpolation SHARED
  src/spline_interpolation.cpp)
ament_target_dependencies(spline_interpolation
  rclcpp
)

install(
  DIRECTORY include/
  DESTINATION include
)
install(TARGETS spline_interpolation
  EXPORT export_spline_interpolation
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

ament_export_interfaces(export_spline_interpolation HAS_LIBRARY_TARGET)
ament_export_libraries(spline_interpolation)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
ament_package()