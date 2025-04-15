cmake_minimum_required(VERSION 3.8)
project(zmqpp_vendor)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

ament_vendor(zstd_vendor
  SATISFIED ${zstd_FOUND}
  VCS_URL https://github.com/zeromq/zmqpp.git
  VCS_VERSION 4.2.0
  SOURCE_SUBDIR build/cmake
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package()
