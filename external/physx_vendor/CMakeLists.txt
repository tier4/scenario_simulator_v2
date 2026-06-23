cmake_minimum_required(VERSION 3.10)
project(physx_vendor)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)

# ---------------------------------------------------------------------------
# Configuration
# ---------------------------------------------------------------------------
# Upstream source. The default pins the public release tag for PhysX 5.6.1.
# Override to track another release (e.g. 107.0-physx-5.6.0, 106.5-physx-5.5.1).
set(PHYSX_VENDOR_GIT_REPOSITORY "https://github.com/NVIDIA-Omniverse/PhysX.git"
    CACHE STRING "PhysX upstream git repository")
set(PHYSX_VENDOR_GIT_TAG "107.3-physx-5.6.1"
    CACHE STRING "PhysX upstream git tag/commit to build (default: PhysX 5.6.1)")
# generate_projects.sh preset. The taiga_x backend only needs the CPU vehicle
# solver, so the default is a CPU-only preset built with the system gcc: this
# avoids the CUDA toolkit dependency (the GPU build targets recent compute
# capabilities that older nvcc rejects) and the clang/libstdc++ setup. Use a
# GPU preset (linux-gcc / linux-clang) only if GPU acceleration is required.
set(PHYSX_VENDOR_PRESET "linux-gcc-cpu-only"
    CACHE STRING "PhysX generate_projects.sh preset (e.g. linux-gcc-cpu-only | linux-clang-cpu-only | linux-gcc)")

# Build PhysX from source via its own generate_projects.sh. This downloads
# packman dependencies (requires network access) and compiles the SDK.
message(STATUS "physx_vendor: building PhysX ${PHYSX_VENDOR_GIT_TAG} from source "
               "(preset=${PHYSX_VENDOR_PRESET}). This downloads packman dependencies "
               "and compiles the SDK; it can take a long time.")
include(ExternalProject)
set(_physx_src "${CMAKE_CURRENT_BINARY_DIR}/physx_src")
set(_physx_root "${_physx_src}/physx")
set(_physx_build "${_physx_root}/compiler/${PHYSX_VENDOR_PRESET}-release")

externalproject_add(physx-ext
  GIT_REPOSITORY "${PHYSX_VENDOR_GIT_REPOSITORY}"
  GIT_TAG "${PHYSX_VENDOR_GIT_TAG}"
  SOURCE_DIR "${_physx_src}"
  TIMEOUT 18000
  BUILD_IN_SOURCE 0
  # PhysX generates per-config CMake build trees under physx/compiler/.
  CONFIGURE_COMMAND bash -c "cd '${_physx_root}' && ./generate_projects.sh ${PHYSX_VENDOR_PRESET}"
  BUILD_COMMAND ${CMAKE_COMMAND} --build "${_physx_build}" --config release --parallel
  # The source/build tree persists; the install(DIRECTORY) rules below copy the
  # SDK subtree directly (no intermediate staging step).
  INSTALL_COMMAND ""
)

# Install the built SDK subtree (headers, release static libs, vehicle snippets)
# to <prefix>/opt/physx — the location the extras file points PHYSX_SDK_DIR at.
# Evaluated at install time, after physx-ext has built, so _physx_root exists.
install(DIRECTORY "${_physx_root}/include/"
        DESTINATION opt/physx/include USE_SOURCE_PERMISSIONS)
install(DIRECTORY "${_physx_root}/bin/linux.x86_64/release/"
        DESTINATION opt/physx/bin/linux.x86_64/release USE_SOURCE_PERMISSIONS)
install(DIRECTORY "${_physx_root}/snippets/"
        DESTINATION opt/physx/snippets USE_SOURCE_PERMISSIONS)

ament_package(CONFIG_EXTRAS "physx_vendor-extras.cmake")
