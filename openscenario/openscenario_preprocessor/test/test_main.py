# Copyright 2015 TIER IV, Inc. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from time import sleep

import pytest
import os
import shutil
from openscenario_utility.conversion import convert
from ament_index_python.packages import get_package_share_directory
from pathlib import Path
import difflib


def run_preprocessor_test(scenario_name, desired_diff_num):
    # share_dir = get_package_share_directory("openscenario_preprocessor")
    preprocessor_share_dir = get_package_share_directory("openscenario_preprocessor")
    test_runner_share_dir = preprocessor_share_dir + "/../../../scenario_test_runner/share/scenario_test_runner"
    # sample_scenario_path = Path(share_dir + "/test/scenarios/" + scenario_name + ".yaml")
    sample_scenario_path = Path(test_runner_share_dir + "/scenario/" + scenario_name + ".yaml")
    output_dir = Path("/tmp/openscenario_preprocessor/test/" + scenario_name)
    if os.path.exists(output_dir):
        shutil.rmtree(output_dir)
    os.makedirs(output_dir)
    converted_xosc = str(convert(sample_scenario_path, output_dir)[0])

    command_path = preprocessor_share_dir + "/../../lib/openscenario_preprocessor/openscenario_preprocessor_command"
    preprocessor_command = command_path + " -s " + str(sample_scenario_path) + " -o " + str(output_dir) \
                           + " --parameters '{\"random_offset\": true}'" + " -f t4v2" + " --skip-full-derivation"
    print(preprocessor_command)
    os.system(preprocessor_command)
    os.system("xmllint --format " + converted_xosc + " > " + str(output_dir) + "/before.xosc")
    os.system("xmllint --c14n11 " + str(output_dir) + "/before.xosc > " + str(output_dir) + "/before.xml")

    os.system("xmllint --format " + str(output_dir) + "/work/formatted.xosc > " + str(output_dir) + "/after.xosc")
    os.system("xmllint --c14n11 " + str(output_dir) + "/after.xosc > " + str(output_dir) + "/after.xml")

    file1 = open(output_dir / "before.xml")
    file2 = open(output_dir / "after.xml")
    diff = difflib.Differ()
    output_diff = diff.compare(file1.readlines(), file2.readlines())
    diff_num = 0
    for line in output_diff:
        if line[0:1] in ['+', '-']:
            diff_num += 1
    assert diff_num == desired_diff_num

def test_by_entity_condition_entity_condition_distance_condition_shortest():
    run_preprocessor_test("ByEntityCondition.EntityCondition.DistanceCondition.Shortest", 0)

def test_by_entity_condition_entity_condition_distance_condition():
    run_preprocessor_test("ByEntityCondition.EntityCondition.DistanceCondition", 0)

def test_by_entity_condition_entity_condition_distance_condition_freespace():
    run_preprocessor_test("ByEntityCondition.EntityCondition.DistanceConditionFreespace", 0)

def test_by_entity_condition_entity_condition_relative_clearance_condition_back():
    run_preprocessor_test("ByEntityCondition.EntityCondition.RelativeClearanceCondition-back", 0)

def test_by_entity_condition_entity_condition_relative_clearance_condition():
    run_preprocessor_test("ByEntityCondition.EntityCondition.RelativeClearanceCondition", 0)

def test_by_entity_condition_entity_condition_relative_distance_condition():
    run_preprocessor_test("ByEntityCondition.EntityCondition.RelativeDistanceCondition", 0)

def test_by_entity_condition_entity_condition_relative_distance_condition_freespace():
    run_preprocessor_test("ByEntityCondition.EntityCondition.RelativeDistanceConditionFreespace", 0)

def test_by_entity_condition_entity_condition_relative_speed_condition():
    run_preprocessor_test("ByEntityCondition.EntityCondition.RelativeSpeedCondition", 0)

def test_by_entity_condition_entity_condition_time_to_collision_condition():
    run_preprocessor_test("ByEntityCondition.EntityCondition.TimeToCollisionCondition", 0)

def test_by_value_condition_user_defined_value_condition():
    run_preprocessor_test("ByValueCondition.UserDefinedValueCondition", 0)

def test_controller_action_assign_controller_action():
    run_preprocessor_test("ControllerAction.AssignControllerAction", 0)

def test_custom_command_action_fault_injection_action():
    run_preprocessor_test("CustomCommandAction.FaultInjectionAction", 0)

def test_custom_command_action_pseudo_traffic_signal_detector_confidence_set_action_v1():
    run_preprocessor_test("CustomCommandAction.PseudoTrafficSignalDetectorConfidenceSetAction@v1", 0)

def test_custom_command_action_request_to_cooperate_command_action_v1():
    run_preprocessor_test("CustomCommandAction.RequestToCooperateCommandAction@v1", 0)

def test_custom_command_action_v2i_traffic_signal_state_action():
    run_preprocessor_test("CustomCommandAction.V2ITrafficSignalStateAction", 0)

def test_environment():
    run_preprocessor_test("Environment", 0)

def test_lateral_action_lane_change_action_road_shoulder():
    run_preprocessor_test("LateralAction.LaneChangeAction-RoadShoulder", 0)

def test_lateral_action_lane_change_action():
    run_preprocessor_test("LateralAction.LaneChangeAction", 0)

def test_longitudinal_action_speed_action():
    run_preprocessor_test("LongitudinalAction.SpeedAction", 0)

def test_longitudinal_action_speed_profile_action():
    run_preprocessor_test("LongitudinalAction.SpeedProfileAction", 0)

def test_property_detected_object_ground_truth_publishing_delay():
    run_preprocessor_test("Property.detectedObjectGroundTruthPublishingDelay", 0)

def test_property_detected_object_missing_probability():
    run_preprocessor_test("Property.detectedObjectMissingProbability", 0)

def test_property_detected_object_position_standard_deviation():
    run_preprocessor_test("Property.detectedObjectPositionStandardDeviation", 0)

def test_property_detected_object_publishing_delay():
    run_preprocessor_test("Property.detectedObjectPublishingDelay", 0)

def test_property_detection_sensor_range():
    # Expected diff: 4 lines - floating-point precision difference in sensor range values
    run_preprocessor_test("Property.detectionSensorRange", 4)

def test_property_is_blind():
    run_preprocessor_test("Property.isBlind", 0)

def test_property_max_speed():
    run_preprocessor_test("Property.maxSpeed", 0)

def test_property_pointcloud_publishing_delay():
    run_preprocessor_test("Property.pointcloudPublishingDelay", 0)

def test_property_pointcloud_vertical_field_of_view():
    run_preprocessor_test("Property.pointcloudVerticalFieldOfView", 0)

def test_routing_action_acquire_position_action_continuous():
    run_preprocessor_test("RoutingAction.AcquirePositionAction-continuous", 0)

def test_routing_action_acquire_position_action():
    run_preprocessor_test("RoutingAction.AcquirePositionAction", 0)

def test_routing_action_assign_route_action():
    run_preprocessor_test("RoutingAction.AssignRouteAction", 0)

def test_routing_action_follow_trajectory_action_autoware():
    run_preprocessor_test("RoutingAction.FollowTrajectoryAction-autoware", 0)

def test_routing_action_follow_trajectory_action_override():
    run_preprocessor_test("RoutingAction.FollowTrajectoryAction-override", 0)

def test_routing_action_follow_trajectory_action_star():
    run_preprocessor_test("RoutingAction.FollowTrajectoryAction-star", 0)

def test_routing_action_follow_trajectory_action_straight_bicycle():
    run_preprocessor_test("RoutingAction.FollowTrajectoryAction-straight-bicycle", 0)

def test_routing_action_follow_trajectory_action_straight_pedestrian():
    run_preprocessor_test("RoutingAction.FollowTrajectoryAction-straight-pedestrian", 0)

def test_routing_action_follow_trajectory_action_straight():
    run_preprocessor_test("RoutingAction.FollowTrajectoryAction-straight", 0)

def test_routing_action_follow_trajectory_action_zero_distance():
    run_preprocessor_test("RoutingAction.FollowTrajectoryAction-zero-distance", 0)

def test_routing_action_follow_trajectory_action_zero_time():
    run_preprocessor_test("RoutingAction.FollowTrajectoryAction-zero-time", 0)

def test_traffic_signal_controller_action():
    run_preprocessor_test("TrafficSignalControllerAction", 0)

def test_traffic_signals():
    run_preprocessor_test("TrafficSignals", 0)

def test_visualization_simulation_context():
    run_preprocessor_test("Visualization.simulation.context", 0)

def test_all_in_one():
    run_preprocessor_test("all-in-one", 0)

def test_arm_demo():
    run_preprocessor_test("arm_demo", 0)

def test_autoware_simple():
    # Expected diff: 2 lines - laneId replaced with placeholder "LANE_ID" for parameterization
    run_preprocessor_test("autoware-simple", 2)

def test_collision():
    # Expected diff: 10 lines - XML element order correction (BoundingBox position) and s value placeholder "NPC-S"
    run_preprocessor_test("collision", 10)

def test_collision_condition_by_type():
    run_preprocessor_test("collision_condition_by_type", 0)

def test_condition():
    run_preprocessor_test("condition", 0)

def test_distance_condition():
    run_preprocessor_test("distance-condition", 0)

def test_duplicated_parameter():
    run_preprocessor_test("duplicated-parameter", 0)

def test_empty():
    run_preprocessor_test("empty", 0)

def test_execution_time_test():
    run_preprocessor_test("execution_time_test", 0)

def test_failure_single_condition_anonymous():
    run_preprocessor_test("failure_single_condition_anonymous", 0)

def test_failure_single_condition_named():
    run_preprocessor_test("failure_single_condition_named", 0)

def test_failure_single_init_action():
    run_preprocessor_test("failure_single_init_action", 0)

def test_failure_three_conditions_two_groups_trigger_anonymous():
    run_preprocessor_test("failure_three_conditions_two_groups_trigger_anonymous", 0)

def test_failure_three_conditions_two_groups_trigger_named():
    run_preprocessor_test("failure_three_conditions_two_groups_trigger_named", 0)

def test_failure_two_conditions_different_group_trigger_anonymous():
    run_preprocessor_test("failure_two_conditions_different_group_trigger_anonymous", 0)

def test_failure_two_conditions_different_group_trigger_named():
    run_preprocessor_test("failure_two_conditions_different_group_trigger_named", 0)

def test_failure_two_conditions_single_group_named():
    run_preprocessor_test("failure_two_conditions_single_group_named", 0)

def test_failure_two_conditions_single_group_one_named():
    run_preprocessor_test("failure_two_conditions_single_group_one_named", 0)

def test_failure_two_init_actions():
    run_preprocessor_test("failure_two_init_actions", 0)

def test_minimal():
    run_preprocessor_test("minimal", 0)

def test_parameter():
    # Expected diff: 18 lines - XML element order correction (StartTrigger moved to XSD-compliant position)
    run_preprocessor_test("parameter", 18)

def test_prefixed_name_reference():
    run_preprocessor_test("prefixed-name-reference", 0)

def test_relative_target_speed():
    run_preprocessor_test("relative_target_speed", 0)

def test_sample():
    # Expected diff: 2 lines - laneId replaced with placeholder "LANE_ID" for parameterization
    run_preprocessor_test("sample", 2)

def test_sample_awsim():
    # Expected diff: 2 lines - laneId replaced with placeholder "LANE_ID" for parameterization
    run_preprocessor_test("sample_awsim", 2)

def test_sample_awsim_conventional_traffic_lights():
    run_preprocessor_test("sample_awsim_conventional_traffic_lights", 0)

def test_sample_awsim_v2i_traffic_lights():
    run_preprocessor_test("sample_awsim_v2i_traffic_lights", 0)

def test_set_behavior_parameters_in_object_controller():
    run_preprocessor_test("set_behavior_parameters_in_object_controller", 0)

def test_spawn_relative_object_position():
    run_preprocessor_test("spawn_relative_object_position", 0)

def test_spawn_relative_world():
    # Expected diff: 2 lines - laneId replaced with placeholder "LANE_ID" for parameterization
    run_preprocessor_test("spawn_relative_world", 2)

def test_stand_still():
    run_preprocessor_test("stand-still", 0)

def test_success():
    # Expected diff: 6 lines - parameter values replaced with placeholders (INITIAL-VALUE, STEP) for parameterization
    run_preprocessor_test("success", 6)
