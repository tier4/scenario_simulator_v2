cmake_minimum_required(VERSION 3.8)
project(zmqpp_vendor)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_vendor_package REQUIRED)

ament_vendor(zmqpp_vendor
  SATISFIED ${zmqpp_FOUND}
  VCS_URL https://github.com/zeromq/zmqpp.git
  VCS_VERSION da73a138f290274cfd604b3f05a908956390a66e
  PATCHES patches/zmqpp_export.patch
)

ament_export_dependencies(zmqpp)

ament_package(CONFIG_EXTRAS zmqpp_vendor-extras.cmake)
