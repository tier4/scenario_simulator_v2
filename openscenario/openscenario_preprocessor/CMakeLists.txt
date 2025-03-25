cmake_minimum_required(VERSION 3.16.3) # Ubuntu 20.04 default CMake version

project(openscenario_preprocessor)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_auto REQUIRED)
find_package(XercesC          REQUIRED)
find_package(Boost            REQUIRED COMPONENTS filesystem program_options regex)
find_package(pugixml          REQUIRED)


ament_auto_find_build_dependencies()

ament_auto_add_executable(${PROJECT_NAME}_node
  src/schema.cpp
  src/${PROJECT_NAME}.cpp
  src/${PROJECT_NAME}_node.cpp)

target_link_libraries(${PROJECT_NAME}_node XercesC::XercesC)
target_link_libraries(${PROJECT_NAME}_node yaml-cpp)

find_library(parameter_value_distribution_LIBRARIES parameter_value_distribution_without_ros)

file(READ ${openscenario_validator_DIR}/../../openscenario_validator/schema/OpenSCENARIO-1.3.xsd ${PROJECT_NAME}_OPENSCENARIO_1_3_XSD)
configure_file(configure/schema.cpp src/schema.cpp)

add_executable(${PROJECT_NAME}_command
  src/t4v2.cpp
  src/schema.cpp
  src/${PROJECT_NAME}.cpp
  src/${PROJECT_NAME}_command.cpp)

# cspell: ignore DPARAMETER
target_compile_definitions(${PROJECT_NAME}_command PUBLIC -DPARAMETER_VALUE_DISTRIBUTION_ONLY)

target_include_directories(${PROJECT_NAME}_command PUBLIC include)
target_include_directories(${PROJECT_NAME}_command PUBLIC ${XercesC_INCLUDE_DIRS})
target_include_directories(${PROJECT_NAME}_command PUBLIC ${YAML_CPP_INCLUDE_DIRS})
target_include_directories(${PROJECT_NAME}_command PUBLIC ${YAML_CPP_INCLUDE_DIRS})
target_include_directories(${PROJECT_NAME}_command PUBLIC ${openscenario_interpreter_INCLUDE_DIRS})
target_include_directories(${PROJECT_NAME}_command PUBLIC ${openscenario_validator_INCLUDE_DIRS})

target_link_libraries(${PROJECT_NAME}_command Boost::filesystem Boost::program_options Boost::regex)
target_link_libraries(${PROJECT_NAME}_command XercesC::XercesC)
target_link_libraries(${PROJECT_NAME}_command pugixml)
target_link_libraries(${PROJECT_NAME}_command yaml-cpp)
target_link_libraries(${PROJECT_NAME}_command glog)

target_link_libraries(${PROJECT_NAME}_command ${parameter_value_distribution_LIBRARIES})

install(TARGETS ${PROJECT_NAME}_node ${PROJECT_NAME}_command
  RUNTIME DESTINATION lib/${PROJECT_NAME}
  )

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
  ament_add_pytest_test(test_${PROJECT_NAME} "test/test_main.py")
endif()

ament_auto_package()
