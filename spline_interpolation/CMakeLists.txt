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
  #${catkin_INCLUDE_DIRS}
)

add_library(spline_interpolation src/spline_interpolation.cpp)
ament_target_dependencies(spline_interpolation
  rclcpp
)

#add_library(spline_interpolation src/spline_interpolation.cpp)

#target_link_libraries(spline_interpolation
#  ${catkin_LIBRARIES}
#)

#add_dependencies(spline_interpolation
#  ${catkin_EXPORTED_TARGETS}
#)

#install(
#  TARGETS
#    spline_interpolation
#  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#)

## Install project namespaced headers
#install(DIRECTORY include/${PROJECT_NAME}/
#  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#)

ament_package()