cmake_minimum_required(VERSION 3.16.3) # Ubuntu 20.04 default CMake version

project(openscenario_interpreter)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -g -DBOOST_ALLOW_DEPRECATED_HEADERS)
endif()

find_package(ament_cmake_auto REQUIRED)

ament_auto_find_build_dependencies()

# ------------------------------------------------------------------------------
#  Boost.JSON (from Git)
# ------------------------------------------------------------------------------
# - Ubuntu 22.04 does not include libboost-json-dev, we need to build it from source
# - We also use older version due to compatibility of Boost by Ubuntu 22.04
include(FetchContent)

FetchContent_Declare(
  boost_json
  GIT_REPOSITORY https://github.com/boostorg/json.git
  GIT_TAG boost-1.77.0)

FetchContent_GetProperties(boost_json)

# For static library inside shared library
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# For faster build, we disable tests, examples, fuzzers, and benchmarks of Boost.JSON
set(BOOST_JSON_STANDALONE ON)
set(BOOST_JSON_BUILD_TESTS OFF CACHE INTERNAL OFF)
set(BOOST_JSON_BUILD_EXAMPLES OFF CACHE INTERNAL OFF)
set(BOOST_JSON_BUILD_FUZZERS OFF CACHE INTERNAL OFF)
set(BOOST_JSON_BUILD_BENCHMARKS OFF CACHE INTERNAL OFF)

# For faster serialization, we increase the internal buffer size
add_compile_definitions(BOOST_JSON_STACK_BUFFER_SIZE=16384)

if(NOT boost_json_POPULATED)
  FetchContent_Populate(boost_json)
  add_subdirectory(${boost_json_SOURCE_DIR} ${boost_json_BINARY_DIR})
endif()

ament_export_include_directories(${boost_json_SOURCE_DIR}/include)

# ------------------------------------------------------------------------------
#  libopenscenario_interpreter
# ------------------------------------------------------------------------------
file(GLOB ${PROJECT_NAME}_POSIX_SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/src/posix/*.cpp)
file(GLOB ${PROJECT_NAME}_READER_SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/src/reader/*.cpp)
file(GLOB ${PROJECT_NAME}_SYNTAX_SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/src/syntax/*.cpp)
file(GLOB ${PROJECT_NAME}_UTILITY_SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/src/utility/*.cpp)

ament_auto_add_library(${PROJECT_NAME} SHARED
  ${${PROJECT_NAME}_POSIX_SOURCES}
  ${${PROJECT_NAME}_READER_SOURCES}
  ${${PROJECT_NAME}_SYNTAX_SOURCES}
  ${${PROJECT_NAME}_UTILITY_SOURCES}
  src/compatibility.cpp
  src/evaluate.cpp
  src/object.cpp
  src/openscenario_interpreter.cpp
  src/record.cpp
  src/scope.cpp)

target_link_libraries(${PROJECT_NAME} Boost::filesystem Boost::json glog pugixml)

target_precompile_headers(${PROJECT_NAME}
  PUBLIC
    "<boost/json.hpp>"
    "<openscenario_interpreter/expression.hpp>"
    "<openscenario_interpreter/object.hpp>"
    "<openscenario_interpreter/pointer.hpp>"
    "<openscenario_interpreter/record.hpp>"
    "<openscenario_interpreter/scope.hpp>"
    "<openscenario_interpreter/simulator_core.hpp>"
    "<pugixml.hpp>"
    "<rclcpp/rclcpp.hpp>"
    "<traffic_simulator/api/api.hpp>")

# ------------------------------------------------------------------------------
#  openscenario_interpreter_node
# ------------------------------------------------------------------------------

ament_auto_add_executable(${PROJECT_NAME}_node src/${PROJECT_NAME}_node.cpp)

target_link_libraries(${PROJECT_NAME}_node ${PROJECT_NAME})

install(
  FILES ${CMAKE_CURRENT_SOURCE_DIR}/features
  DESTINATION lib/${PROJECT_NAME})

# ------------------------------------------------------------------------------
#  test
# ------------------------------------------------------------------------------

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
  ament_add_gtest(test_syntax test/test_syntax.cpp)
  target_link_libraries(test_syntax ${PROJECT_NAME})
endif()

ament_auto_package()
