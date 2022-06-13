cmake_minimum_required(VERSION 3.8)
project(geometry_math)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake_auto REQUIRED)

ament_auto_find_build_dependencies()

ament_auto_add_library(geometry_math SHARED
  src/bounding_box.cpp
  src/collision.cpp
  src/distance.cpp
  src/linear_algebra.cpp
  src/polygon.cpp
  src/solver/polynomial_solver.cpp
  src/spline/catmull_rom_spline.cpp
  src/spline/catmull_rom_subspline.cpp
  src/spline/hermite_curve.cpp
  src/transform.cpp
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
  add_subdirectory(test)
endif()

ament_auto_package()