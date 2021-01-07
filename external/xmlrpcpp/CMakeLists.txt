cmake_minimum_required(VERSION 3.5)
project(xmlrpcpp)

if(WIN32)
  add_definitions(-D_WINDOWS)
endif()

if(NOT WIN32)
  set_directory_properties(PROPERTIES COMPILE_OPTIONS "-Wall;-Wextra")
endif()

find_package(ament_cmake_auto REQUIRED)

ament_auto_find_build_dependencies()

ament_auto_add_library(${PROJECT_NAME} SHARED
  libb64/src/cdecode.c
  libb64/src/cencode.c
  src/XmlRpcClient.cpp
  src/XmlRpcDispatch.cpp
  src/XmlRpcServer.cpp
  src/XmlRpcServerConnection.cpp
  src/XmlRpcServerMethod.cpp
  src/XmlRpcSocket.cpp
  src/XmlRpcSource.cpp
  src/XmlRpcUtil.cpp
  src/XmlRpcValue.cpp)

target_include_directories(${PROJECT_NAME} PRIVATE libb64/include)

ament_auto_package()
