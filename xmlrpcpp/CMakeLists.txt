cmake_minimum_required(VERSION 3.5)
project(xmlrpcpp)

if(NOT WIN32)
  set_directory_properties(PROPERTIES COMPILE_OPTIONS "-Wall;-Wextra")
endif()

find_package(ament_cmake REQUIRED)

include_directories(
  include
  libb64/include
)

if(WIN32)
  add_definitions(-D_WINDOWS)
endif()

add_library(xmlrpcpp
  src/XmlRpcClient.cpp
  src/XmlRpcDispatch.cpp
  src/XmlRpcServer.cpp
  src/XmlRpcServerConnection.cpp
  src/XmlRpcServerMethod.cpp
  src/XmlRpcSocket.cpp
  src/XmlRpcSource.cpp
  src/XmlRpcUtil.cpp
  src/XmlRpcValue.cpp
  libb64/src/cdecode.c
  libb64/src/cencode.c
)
install(
  DIRECTORY include/
  DESTINATION include
)

ament_package()