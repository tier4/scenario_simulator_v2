cmake_minimum_required(VERSION 3.5)

project(status_monitor)

set(CMAKE_CXX_STANDARD 17)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(Boost REQUIRED COMPONENTS date_time)
find_package(Threads REQUIRED)
find_package(ament_cmake_auto REQUIRED)

ament_auto_find_build_dependencies()

ament_auto_add_library(${PROJECT_NAME} SHARED src/${PROJECT_NAME}.cpp)

target_link_libraries(${PROJECT_NAME} Boost::date_time Threads::Threads)

ament_auto_package(USE_SCOPED_HEADER_INSTALL_DIR)
