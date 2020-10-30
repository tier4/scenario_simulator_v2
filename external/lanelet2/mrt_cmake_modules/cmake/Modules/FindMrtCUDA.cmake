find_package(CUDA REQUIRED)

if(${CUDA_VERSION_STRING} VERSION_LESS 10.0)
    set(_CUDA_HOST_COMPILER "g++-6")
elseif(${CUDA_VERSION_STRING} VERSION_LESS 10.2)
    set(_CUDA_HOST_COMPILER "g++-7")
elseif(${CUDA_VERSION_STRING} VERSION_LESS 11.0)
    set(_CUDA_HOST_COMPILER "g++-8")
else()
    set(_CUDA_HOST_COMPILER "g++-9")
endif()

if(NOT TARGET cuda_compiler_flags)
    add_library(cuda_compiler_flags INTERFACE IMPORTED)
    if(${CMAKE_VERSION} VERSION_LESS "3.9.0")
        set(CUDA_HOST_COMPILER "${_CUDA_HOST_COMPILER}")

        target_compile_options(
            cuda_compiler_flags
            INTERFACE -Wno-deprecated-gpu-targets;-std=c++14 $<$<CONFIG:Debug>:-lineinfo;-g;--compiler-options;-fPIC>
                      $<$<NOT:$<CONFIG:Debug>>-O3;--compiler-options;-fPIC>)
        target_compile_definitions(cuda_compiler_flags $<$<NOT:$<CONFIG:Debug>>NDEBUG>)
        set(CUDA_PROPAGATE_HOST_FLAGS OFF)
    else()
        set(CMAKE_CUDA_COMPILER "${CUDA_TOOLKIT_ROOT_DIR}/bin/nvcc")
        set(CMAKE_CUDA_HOST_COMPILER "${_CUDA_HOST_COMPILER}")
        enable_language(CUDA)

        target_compile_options(cuda_compiler_flags
                               INTERFACE $<$<COMPILE_LANGUAGE:CUDA>:-lineinfo;--expt-relaxed-constexpr>)
        # in cmake 3.17 apparently there should by cuda_std_14 as compile feature
        target_compile_features(cuda_compiler_flags INTERFACE cxx_std_14)
        target_include_directories(cuda_compiler_flags INTERFACE "${CMAKE_CUDA_TOOLKIT_INCLUDE_DIRECTORIES}")
    endif()
endif()
