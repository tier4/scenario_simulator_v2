cmake_minimum_required(VERSION 3.14)
project(osi_interface)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_auto REQUIRED)
find_package(rclcpp REQUIRED)
find_package(cppzmq QUIET)
include(FindProtobuf REQUIRED)
ament_auto_find_build_dependencies()

# --- OSI source fetch --------------------------------------------------------

include(FetchContent)
FetchContent_Declare(
  open_simulation_interface
  GIT_REPOSITORY https://github.com/OpenSimulationInterface/open-simulation-interface.git
  GIT_TAG        master
  GIT_SHALLOW    TRUE
)
FetchContent_GetProperties(open_simulation_interface)
if(NOT open_simulation_interface_POPULATED)
  FetchContent_Populate(open_simulation_interface)
endif()
set(OSI_SOURCE_DIR "${open_simulation_interface_SOURCE_DIR}")

# --- OSI proto compilation ---------------------------------------------------

# Read VERSION file and generate osi_version.proto
file(STRINGS "${OSI_SOURCE_DIR}/VERSION" OSI_VERSION_CONTENTS)
foreach(LINE ${OSI_VERSION_CONTENTS})
  string(REGEX REPLACE " |\t" "" LINE ${LINE})
  string(REGEX MATCH "^[^=]+" VERSION_NAME ${LINE})
  string(REPLACE "${VERSION_NAME}=" "" VERSION_VALUE ${LINE})
  set(${VERSION_NAME} "${VERSION_VALUE}")
endforeach()

set(OSI_PROTO_OUTPUT_DIR "${CMAKE_BINARY_DIR}/osi_proto")
file(MAKE_DIRECTORY ${OSI_PROTO_OUTPUT_DIR})

configure_file(
  "${OSI_SOURCE_DIR}/osi_version.proto.in"
  "${OSI_PROTO_OUTPUT_DIR}/osi_version.proto"
  @ONLY
)

# List all OSI proto files
file(GLOB OSI_PROTO_FILES "${OSI_SOURCE_DIR}/osi_*.proto")
# Add generated osi_version.proto
list(APPEND OSI_PROTO_FILES "${OSI_PROTO_OUTPUT_DIR}/osi_version.proto")

# Generate C++ from OSI protos
set(OSI_CPP_OUTPUT_DIR "${CMAKE_BINARY_DIR}/generated/osi3")
file(MAKE_DIRECTORY ${OSI_CPP_OUTPUT_DIR})

protobuf_generate(
  PROTOS ${OSI_PROTO_FILES}
  APPEND_PATH
  LANGUAGE cpp
  OUT_VAR OSI_PROTO_SRCS
  IMPORT_DIRS "${OSI_SOURCE_DIR}" "${OSI_PROTO_OUTPUT_DIR}"
  PROTOC_OUT_DIR ${OSI_CPP_OUTPUT_DIR}
)

# --- osi_interface library ---------------------------------------------------

include_directories(
  include
  ${CMAKE_BINARY_DIR}/generated/
)

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/entity_id_registry.cpp
  src/orientation_conversion.cpp
  src/osi_entity_conversions.cpp
  src/osi_traffic_light_conversions.cpp
  src/ground_truth_builder.cpp
  src/traffic_update_builder.cpp
  src/osi_ground_truth_handler.cpp
  src/osi_zmq_client.cpp
  src/osi_zmq_server.cpp
  ${OSI_PROTO_SRCS}
)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_BINARY_DIR}/generated/>
  $<INSTALL_INTERFACE:include/osi_interface/>
)

target_link_libraries(${PROJECT_NAME}
  ${PROTOBUF_LIBRARY}
  zmq
  $<TARGET_NAME_IF_EXISTS:cppzmq>
)

# --- Install -----------------------------------------------------------------

install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include/osi_interface
)

install(
  DIRECTORY ${CMAKE_BINARY_DIR}/generated/
  DESTINATION include/osi_interface/
)

# --- Testing -----------------------------------------------------------------

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_entity_id_registry test/test_entity_id_registry.cpp)
  target_link_libraries(test_entity_id_registry ${PROJECT_NAME})
  ament_add_gtest(test_orientation_conversion test/test_orientation_conversion.cpp)
  target_link_libraries(test_orientation_conversion ${PROJECT_NAME})
  ament_add_gtest(test_osi_entity_conversions test/test_osi_entity_conversions.cpp)
  target_link_libraries(test_osi_entity_conversions ${PROJECT_NAME})
  ament_add_gtest(test_osi_traffic_light_conversions test/test_osi_traffic_light_conversions.cpp)
  target_link_libraries(test_osi_traffic_light_conversions ${PROJECT_NAME})
  ament_add_gtest(test_ground_truth_builder test/test_ground_truth_builder.cpp)
  target_link_libraries(test_ground_truth_builder ${PROJECT_NAME})
  ament_add_gtest(test_traffic_update_builder test/test_traffic_update_builder.cpp)
  target_link_libraries(test_traffic_update_builder ${PROJECT_NAME})
  ament_add_gtest(test_osi_ground_truth_handler test/test_osi_ground_truth_handler.cpp)
  target_link_libraries(test_osi_ground_truth_handler ${PROJECT_NAME})
  ament_add_gtest(test_osi_zmq_communication test/test_osi_zmq_communication.cpp)
  target_link_libraries(test_osi_zmq_communication ${PROJECT_NAME})
  ament_add_gtest(test_osi_legacy_equivalence test/test_osi_legacy_equivalence.cpp)
  target_link_libraries(test_osi_legacy_equivalence ${PROJECT_NAME})
endif()

ament_auto_package(USE_SCOPED_HEADER_INSTALL_DIR)
