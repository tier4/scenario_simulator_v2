# this file creates the target ${PROJECT_NAME}_${PROJECT_NAME}_compiler_flags and ${PROJECT_NAME}_${PROJECT_NAME}_private_compiler_flags that will be used by all targets created by mrt_cmake_modules. It also sets some basic configurations that generally make sens with cmake and ROS.

# export compile commands
set(CMAKE_EXPORT_COMPILE_COMMANDS YES)

# default to building shared libraries. This can be changed by passing "-DBUILD_SHARED_LIBS=Off" to cmake.
option(BUILD_SHARED_LIBS "Build libraries as shared libraries by default." ON)

add_library(${PROJECT_NAME}_compiler_flags INTERFACE)
add_library(${PROJECT_NAME}_private_compiler_flags INTERFACE)

# At least c++14 is required. This can be increased in a file referenced by MRT_CMAKE_CONFIG_FILE.
if(CMAKE_VERSION VERSION_LESS "3.8")
    target_compile_options(${PROJECT_NAME}_compiler_flags INTERFACE -std=c++14)
else()
    target_compile_features(${PROJECT_NAME}_compiler_flags INTERFACE cxx_std_14)
endif()

# Add _DEBUG and _GLIBCXX_ASSERTIONS for debug configuration. This enables e.g. assertions in OpenCV and the STL.
if(CMAKE_VERSION VERSION_GREATER "3.12")
    target_compile_definitions(${PROJECT_NAME}_private_compiler_flags INTERFACE $<$<CONFIG:Debug>:_DEBUG>
                                                                                $<$<CONFIG:Debug>:_GLIBCXX_ASSERTIONS>)
endif()

# Add support for std::filesystem. For GCC version <= 8 one needs to link against -lstdc++fs.
target_link_libraries(${PROJECT_NAME}_compiler_flags
                      INTERFACE $<$<AND:$<CXX_COMPILER_ID:GNU>,$<VERSION_LESS:$<CXX_COMPILER_VERSION>,9.0>>:stdc++fs>)

# add OpenMP if present
# it would be great to have this in package.xmls instead, but catkin cannot handle setting the required cmake flags for dependencies
find_package(OpenMP)
if(OpenMP_FOUND)
    if(TARGET OpenMP::OpenMP_CXX)
        target_link_libraries(${PROJECT_NAME}_compiler_flags INTERFACE OpenMP::OpenMP_CXX)
    else()
        target_compile_options(${PROJECT_NAME}_compiler_flags INTERFACE $<$<COMPILE_LANGUAGE:CXX>:${OpenMP_CXX_FLAGS}>
                                                                        $<$<COMPILE_LANGUAGE:C>:${OpenMP_C_FLAGS}>)
        if(CMAKE_VERSION VERSION_LESS "3.10")
            target_link_libraries(${PROJECT_NAME}_compiler_flags INTERFACE ${OpenMP_CXX_FLAGS})
        else()
            target_link_libraries(
                ${PROJECT_NAME}_compiler_flags INTERFACE $<$<COMPILE_LANGUAGE:CXX>:${OpenMP_CXX_FLAGS}>
                                                         $<$<COMPILE_LANGUAGE:C>:${OpenMP_C_FLAGS}>)
        endif()
    endif()
endif()

# add gcov flags
set(gcc_like_cxx "$<AND:$<COMPILE_LANGUAGE:CXX>,$<CXX_COMPILER_ID:ARMClang,AppleClang,Clang,GNU>>")
set(gcc_cxx "$<AND:$<COMPILE_LANGUAGE:CXX>,$<CXX_COMPILER_ID:GNU>>")
set(gcc_like_c "$<AND:$<COMPILE_LANGUAGE:CXX>,$<CXX_COMPILER_ID:ARMClang,AppleClang,Clang,GNU>>")
if(MRT_ENABLE_COVERAGE)
    target_compile_options(${PROJECT_NAME}_private_compiler_flags INTERFACE $<${gcc_like_cxx}:-g;--coverage>
                                                                            $<${gcc_like_c}:-g;--coverage>)
    if(CMAKE_VERSION VERSION_LESS "3.13")
        target_link_libraries(${PROJECT_NAME}_private_compiler_flags INTERFACE --coverage)
    else()
        target_link_options(${PROJECT_NAME}_private_compiler_flags INTERFACE $<${gcc_like_cxx}:--coverage>
                            $<${gcc_like_c}:--coverage>)
    endif()
endif()

# Include config file if set. This is done last so that the target ${PROJECT_NAME}_compiler_flags can be further modified
# The config file is supposed to contain system-specific configurations (basically like a cmake toolchain file) and personal preferences (e.g. warning options)
if(MRT_CMAKE_CONFIG_FILE)
    message(STATUS "MRT CMake configuration file found: ${MRT_CMAKE_CONFIG_FILE}")
    include(${MRT_CMAKE_CONFIG_FILE})
elseif(DEFINED ENV{MRT_CMAKE_CONFIG_FILE})
    message(STATUS "MRT CMake configuration file found: $ENV{MRT_CMAKE_CONFIG_FILE}")
    include($ENV{MRT_CMAKE_CONFIG_FILE})
endif()
