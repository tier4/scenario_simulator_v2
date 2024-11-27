cmake_minimum_required(VERSION 3.5)
project(customized_rvo2)

# find dependencies
find_package(ament_cmake_auto REQUIRED)
#find_package(OpenMP REQUIRED)
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fPIC")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fPIC")
ament_auto_find_build_dependencies()

set(RVO_SOURCES
    src/Math.cpp
    src/Obstacle.cpp
    src/Agent.cpp
    src/KdTree.cpp
    src/RVOSimulator.cpp
    src/VisualizeMarker.cpp
    src/Plugins/ObstaclePlugin.cpp
    src/Plugins/AgentPlugin.cpp)

ament_auto_add_library(${PROJECT_NAME} SHARED ${RVO_SOURCES})

if(${tf2_geometry_msgs_VERSION} VERSION_LESS 0.18.0)
  target_compile_definitions(${PROJECT_NAME} PUBLIC
    USE_TF2_GEOMETRY_MSGS_DEPRECATED_HEADER
  )
endif()

if(BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    ament_lint_auto_find_test_dependencies()
    find_package(ament_cmake_gtest REQUIRED)
    ament_add_gtest(test_rvo2 test/src/test_rvo2.cpp)
    target_link_libraries(test_rvo2 ${PROJECT_NAME})
endif()

ament_auto_package()
