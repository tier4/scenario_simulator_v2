ament_auto_add_executable(define_traffic_source_vehicle
  define_traffic_source_vehicle.cpp
)
target_link_libraries(define_traffic_source_vehicle cpp_scenario_node)

ament_auto_add_executable(define_traffic_source_pedestrian
  define_traffic_source_pedestrian.cpp
)
target_link_libraries(define_traffic_source_pedestrian cpp_scenario_node)

ament_auto_add_executable(define_traffic_source_multiple
  define_traffic_source_multiple.cpp
)
target_link_libraries(define_traffic_source_multiple cpp_scenario_node)

ament_auto_add_executable(define_traffic_source_mixed
  define_traffic_source_mixed.cpp
)
target_link_libraries(define_traffic_source_mixed cpp_scenario_node)

ament_auto_add_executable(define_traffic_source_delay
  define_traffic_source_delay.cpp
)
target_link_libraries(define_traffic_source_delay cpp_scenario_node)

ament_auto_add_executable(define_traffic_source_outside_lane
  define_traffic_source_outside_lane.cpp
)
target_link_libraries(define_traffic_source_outside_lane cpp_scenario_node)

ament_auto_add_executable(define_traffic_source_high_rate
  define_traffic_source_high_rate.cpp
)
target_link_libraries(define_traffic_source_high_rate cpp_scenario_node)

ament_auto_add_executable(define_traffic_source_large
  define_traffic_source_large.cpp
)
target_link_libraries(define_traffic_source_large cpp_scenario_node)

install(TARGETS
  define_traffic_source_vehicle
  define_traffic_source_pedestrian
  define_traffic_source_multiple
  define_traffic_source_mixed
  define_traffic_source_delay
  define_traffic_source_outside_lane
  define_traffic_source_high_rate
  define_traffic_source_large
  DESTINATION lib/cpp_mock_scenarios
)

if(BUILD_TESTING)
  include(../../cmake/add_cpp_mock_scenario_test.cmake)
  add_cpp_mock_scenario_test(${PROJECT_NAME} "define_traffic_source_vehicle" "5.0")

  include(../../cmake/add_cpp_mock_scenario_test.cmake)
  add_cpp_mock_scenario_test(${PROJECT_NAME} "define_traffic_source_pedestrian" "5.0")

  include(../../cmake/add_cpp_mock_scenario_test.cmake)
  add_cpp_mock_scenario_test(${PROJECT_NAME} "define_traffic_source_multiple" "5.0")

  include(../../cmake/add_cpp_mock_scenario_test.cmake)
  add_cpp_mock_scenario_test(${PROJECT_NAME} "define_traffic_source_mixed" "5.0")

  include(../../cmake/add_cpp_mock_scenario_test.cmake)
  add_cpp_mock_scenario_test(${PROJECT_NAME} "define_traffic_source_delay" "5.0")

  include(../../cmake/add_cpp_mock_scenario_test.cmake)
  add_cpp_mock_scenario_test(${PROJECT_NAME} "define_traffic_source_outside_lane" "5.0")

  include(../../cmake/add_cpp_mock_scenario_test.cmake)
  add_cpp_mock_scenario_test(${PROJECT_NAME} "define_traffic_source_high_rate" "5.0")

  include(../../cmake/add_cpp_mock_scenario_test.cmake)
  add_cpp_mock_scenario_test(${PROJECT_NAME} "define_traffic_source_large" "5.0")
endif()
