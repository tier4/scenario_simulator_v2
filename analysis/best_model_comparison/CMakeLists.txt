cmake_minimum_required(VERSION 3.14)
project(best_model_comparison)

find_package(ament_cmake REQUIRED)

install(FILES
  libvehicle_model_wrapper.so
  DESTINATION share/${PROJECT_NAME}/lib
)

ament_package()
