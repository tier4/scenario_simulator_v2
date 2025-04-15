cmake_minimum_required(VERSION 3.8)
project(zmqpp_vendor)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

# macro(build_zmqpp)
#   set(cmake_commands)
#   set(cmake_configure_args
#     -Wno-dev
#     -DCMAKE_INSTALL_PREFIX=${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}_install)

#   include(ExternalProject)
#   externalproject_add(cpp-zmqpp-ext
#     GIT_REPOSITORY https://github.com/zeromq/zmqpp.git
#     GIT_TAG master
#     TIMEOUT 6000
#     ${cmake_commands}
#     CMAKE_ARGS ${cmake_configure_args}
#   )
#   install(
#     DIRECTORY
#       ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}_install/
#     DESTINATION
#       ${CMAKE_INSTALL_PREFIX}
#   )
# endmacro()

# build_zmqpp()
# ament_export_include_directories(include)
# ament_export_libraries(zmqpp)


ament_vendor(zstd_vendor
  SATISFIED ${zstd_FOUND}
  VCS_URL https://github.com/facebook/zstd.git
  VCS_VERSION v1.4.8
  SOURCE_SUBDIR build/cmake
  CMAKE_ARGS
    -DZSTD_BUILD_STATIC:BOOL=OFF
    -DZSTD_BUILD_SHARED:BOOL=ON
    -DZSTD_BUILD_PROGRAMS:BOOL=OFF
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package()
