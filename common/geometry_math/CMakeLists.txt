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

# workaround to allow deprecated header to build on both galactic and humble
if(${tf2_geometry_msgs_VERSION} VERSION_LESS 0.18.0)
  target_compile_definitions(geometry_math PUBLIC
    USE_TF2_GEOMETRY_MSGS_DEPRECATED_HEADER
  )
endif()

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
  add_subdirectory(test)
endif()

ament_auto_package()