ament_auto_add_executable(spawn
  spawn.cpp
)
target_link_libraries(spawn
  ${PROTOBUF_LIBRARY}
  pthread
  sodium
  zmq
)

ament_auto_add_executable(stop_at_crosswalk
  stop_at_crosswalk.cpp
)
target_link_libraries(stop_at_crosswalk
  ${PROTOBUF_LIBRARY}
  pthread
  sodium
  zmq
)

ament_auto_add_executable(parked_at_crosswalk
  parked_at_crosswalk.cpp
)
target_link_libraries(parked_at_crosswalk
  ${PROTOBUF_LIBRARY}
  pthread
  sodium
  zmq
)

ament_auto_add_executable(reverse_walk_at_road
  reverse_walk_at_road.cpp
)
target_link_libraries(reverse_walk_at_road
  ${PROTOBUF_LIBRARY}
  pthread
  sodium
  zmq
)

ament_auto_add_executable(pedestrian_follow_lane
  pedestrian_follow_lane.cpp
)
target_link_libraries(pedestrian_follow_lane
  ${PROTOBUF_LIBRARY}
  pthread
  sodium
  zmq
)

ament_auto_add_executable(stop_line_running
  stop_line_running.cpp
)
target_link_libraries(stop_line_running
  ${PROTOBUF_LIBRARY}
  pthread
  sodium
  zmq
)

ament_auto_add_executable(traffic_light_running
  traffic_light_running.cpp
)
target_link_libraries(traffic_light_running
  ${PROTOBUF_LIBRARY}
  pthread
  sodium
  zmq
)

ament_auto_add_executable(head_on_pedestrian
head_on_pedestrian.cpp
)
target_link_libraries(head_on_pedestrian
  ${PROTOBUF_LIBRARY}
  pthread
  sodium
  zmq
)

ament_auto_add_executable(adaptive_cruise
adaptive_cruise.cpp
)
target_link_libraries(adaptive_cruise
  ${PROTOBUF_LIBRARY}
  pthread
  sodium
  zmq
)

ament_auto_add_executable(follow_trajectory
follow_trajectory.cpp
)
target_link_libraries(follow_trajectory
  ${PROTOBUF_LIBRARY}
  pthread
  sodium
  zmq
)

ament_auto_add_executable(follow_trajectory_closed
follow_trajectory_closed.cpp
)
target_link_libraries(follow_trajectory_closed
  ${PROTOBUF_LIBRARY}
  pthread
  sodium
  zmq
)

ament_auto_add_executable(lane_change_time
lane_change_time.cpp
)
target_link_libraries(lane_change_time
  ${PROTOBUF_LIBRARY}
  pthread
  sodium
  zmq
)

ament_auto_add_executable(lane_change_lateral
lane_change_lateral.cpp
)
target_link_libraries(lane_change_lateral
  ${PROTOBUF_LIBRARY}
  pthread
  sodium
  zmq
)

# ament_auto_add_executable(lane_change_longitudinal
# lane_change_longitudinal.cpp
# )
# target_link_libraries(lane_change_longitudinal
#   ${PROTOBUF_LIBRARY}
#   pthread
#   sodium
#   zmq
# )

# ament_auto_add_executable(follow_trajectory_dynamic_constraints
# follow_trajectory_dynamic_constraints.cpp
# )
# target_link_libraries(follow_trajectory_dynamic_constraints
#   ${PROTOBUF_LIBRARY}
#   pthread
#   sodium
#   zmq
# )

# ament_auto_add_executable(road_end
# road_end.cpp
# )
# target_link_libraries(road_end
#   ${PROTOBUF_LIBRARY}
#   pthread
#   sodium
#   zmq
# )

if(${tf2_geometry_msgs_VERSION} VERSION_LESS 0.18.0)
  target_compile_definitions(spawn PUBLIC
    USE_TF2_GEOMETRY_MSGS_DEPRECATED_HEADER
  )
  target_compile_definitions(stop_at_crosswalk PUBLIC
    USE_TF2_GEOMETRY_MSGS_DEPRECATED_HEADER
  )
  target_compile_definitions(parked_at_crosswalk PUBLIC
    USE_TF2_GEOMETRY_MSGS_DEPRECATED_HEADER
  )
  target_compile_definitions(reverse_walk_at_road PUBLIC
    USE_TF2_GEOMETRY_MSGS_DEPRECATED_HEADER
  )
  target_compile_definitions(pedestrian_follow_lane PUBLIC
    USE_TF2_GEOMETRY_MSGS_DEPRECATED_HEADER
  )
  target_compile_definitions(stop_line_running PUBLIC
    USE_TF2_GEOMETRY_MSGS_DEPRECATED_HEADER
  )
  target_compile_definitions(traffic_light_running PUBLIC
    USE_TF2_GEOMETRY_MSGS_DEPRECATED_HEADER
  )
  target_compile_definitions(head_on_pedestrian PUBLIC
    USE_TF2_GEOMETRY_MSGS_DEPRECATED_HEADER
  )
  target_compile_definitions(adaptive_cruise PUBLIC
    USE_TF2_GEOMETRY_MSGS_DEPRECATED_HEADER
  )
  target_compile_definitions(follow_trajectory PUBLIC
    USE_TF2_GEOMETRY_MSGS_DEPRECATED_HEADER
  )
  target_compile_definitions(follow_trajectory_closed PUBLIC
    USE_TF2_GEOMETRY_MSGS_DEPRECATED_HEADER
  )
  target_compile_definitions(lane_change_time PUBLIC
    USE_TF2_GEOMETRY_MSGS_DEPRECATED_HEADER
  )
  target_compile_definitions(lane_change_lateral PUBLIC
    USE_TF2_GEOMETRY_MSGS_DEPRECATED_HEADER
  )
  target_compile_definitions(lane_change_longitudinal PUBLIC
    USE_TF2_GEOMETRY_MSGS_DEPRECATED_HEADER
  )
  target_compile_definitions(follow_trajectory_dynamic_constraints PUBLIC
    USE_TF2_GEOMETRY_MSGS_DEPRECATED_HEADER
  )
  target_compile_definitions(road_end PUBLIC
    USE_TF2_GEOMETRY_MSGS_DEPRECATED_HEADER
  )
endif()

install(TARGETS
  stop_at_crosswalk
  # parked_at_crosswalk
  # reverse_walk_at_road
  # spawn
  # pedestrian_follow_lane
  # stop_line_running
  # traffic_light_running
  # head_on_pedestrian
  # adaptive_cruise
  # follow_trajectory
  # follow_trajectory_closed
  # lane_change_time
  # lane_change_lateral
  # lane_change_longitudinal
  # follow_trajectory_dynamic_constraints
  # road_end
  DESTINATION lib/${PROJECT_NAME}
)

# if(BUILD_TESTING)
#   find_package(ament_lint_auto REQUIRED)
#   ament_lint_auto_find_test_dependencies()
#   include(../../cmake/add_context_gamma_scenario_test.cmake)
#   add_context_gamma_scenario_test(${PROJECT_NAME} "spawn" "20.0")
#   add_context_gamma_scenario_test(${PROJECT_NAME} "parked_at_crosswalk" "60.0")
#   add_context_gamma_scenario_test(${PROJECT_NAME} "pedestrian_follow_lane" "60.0")
#   add_context_gamma_scenario_test(${PROJECT_NAME} "reverse_walk_at_road" "120.0")
#   add_context_gamma_scenario_test(${PROJECT_NAME} "stop_at_crosswalk" "40.0")
#   add_context_gamma_scenario_test(${PROJECT_NAME} "stop_line_running" "200.0")
#   add_context_gamma_scenario_test(${PROJECT_NAME} "traffic_light_running" "30.0")
#   add_context_gamma_scenario_test(${PROJECT_NAME} "head_on_pedestrian" "60.0")
#   add_context_gamma_scenario_test(${PROJECT_NAME} "adaptive_cruise" "120.0")
#   add_context_gamma_scenario_test(${PROJECT_NAME} "follow_trajectory" "40.0")
#   add_context_gamma_scenario_test(${PROJECT_NAME} "follow_trajectory_closed" "80.0")
#   add_context_gamma_scenario_test(${PROJECT_NAME} "lane_change_time" "20.0")
#   add_context_gamma_scenario_test(${PROJECT_NAME} "lane_change_lateral" "20.0")
#   add_context_gamma_scenario_test(${PROJECT_NAME} "lane_change_longitudinal" "20.0")
#   add_context_gamma_scenario_test(${PROJECT_NAME} "follow_trajectory_dynamic_constraints" "40.0")
#   add_context_gamma_scenario_test(${PROJECT_NAME} "road_end" "30.0")
# endif()
