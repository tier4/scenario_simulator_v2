cmake_minimum_required(VERSION 3.16)
project(openscenario_python)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

# The headless pybind facade only makes sense in SSV2_HEADLESS_EGO builds (it reaches the
# in-process ego injection / read seam that exists only then). Passed by the validator build as a
# global colcon cmake-arg (-DSSV2_HEADLESS_EGO=ON); when OFF this package builds nothing.
option(SSV2_HEADLESS_EGO "Build the headless Diffusion-Planner OpenSCENARIO Python module" OFF)

if(SSV2_HEADLESS_EGO)
  find_package(Python3 REQUIRED COMPONENTS Interpreter Development)
  find_package(pybind11_vendor REQUIRED)
  find_package(pybind11 REQUIRED)

  # The module import name must match PYBIND11_MODULE(openscenario_python, ...).
  pybind11_add_module(openscenario_python src/openscenario_python.cpp)

  # Defined transitively via traffic_simulator's PUBLIC compile definition, but set explicitly too
  # so this TU is guaranteed consistent with the headers it includes (ODR safety).
  target_compile_definitions(openscenario_python PRIVATE SSV2_HEADLESS_EGO)

  ament_target_dependencies(openscenario_python PUBLIC
    openscenario_interpreter
    traffic_simulator
    traffic_simulator_msgs
    autoware_planning_msgs
    autoware_vehicle_msgs
    geometry_msgs
    lifecycle_msgs
    rclcpp)

  # Install so the module is importable from the integrated .venv once the overlay setup.bash is
  # sourced (colcon adds <prefix>/lib/python3.10/site-packages to PYTHONPATH).
  install(TARGETS openscenario_python
    DESTINATION "lib/python${Python3_VERSION_MAJOR}.${Python3_VERSION_MINOR}/site-packages")
endif()

ament_auto_package()
