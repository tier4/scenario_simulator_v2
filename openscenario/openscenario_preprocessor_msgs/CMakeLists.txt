cmake_minimum_required(VERSION 3.5)

project(openscenario_preprocessor_msgs)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_auto REQUIRED)

ament_auto_find_build_dependencies()

rosidl_generate_interfaces(${PROJECT_NAME}
  srv/CheckDerivativeRemained.srv
  srv/Derive.srv
  srv/Load.srv
  srv/SetParameter.srv
  DEPENDENCIES builtin_interfaces)

ament_auto_package()
