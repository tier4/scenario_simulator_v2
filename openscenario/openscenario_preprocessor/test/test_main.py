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


@pytest.mark.skip
def test(scenario_name, desired_diff_num):
    # share_dir = get_package_share_directory("openscenario_preprocessor")
    preprocessor_share_dir = get_package_share_directory("openscenario_preprocessor")
    test_runner_share_dir = preprocessor_share_dir + "/../../../scenario_test_runner/share/scenario_test_runner"
    # sample_scenario_path = Path(share_dir + "/test/scenarios/" + scenario_name + ".yaml")
    sample_scenario_path = Path(test_runner_share_dir + "/scenario/" + scenario_name + ".yaml")
    output_dir = Path("/tmp/openscenario_preprocessor/test/" + scenario_name)
    if os.path.exists(output_dir):
        shutil.rmtree(output_dir)
    os.makedirs(output_dir)
    converted_xoscs = convert(sample_scenario_path, output_dir)
    converted_xosc = str(converted_xoscs[0])

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
    test("ByEntityCondition.EntityCondition.DistanceCondition.Shortest", 48)

def test_by_entity_condition_entity_condition_distance_condition():
    test("ByEntityCondition.EntityCondition.DistanceCondition", 0)

def test_by_entity_condition_entity_condition_distance_condition_freespace():
    test("ByEntityCondition.EntityCondition.DistanceConditionFreespace", 6)

def test_by_entity_condition_entity_condition_relative_clearance_condition_back():
    test("ByEntityCondition.EntityCondition.RelativeClearanceCondition-back", 0)

def test_by_entity_condition_entity_condition_relative_clearance_condition():
    test("ByEntityCondition.EntityCondition.RelativeClearanceCondition", 0)

def test_by_entity_condition_entity_condition_relative_distance_condition():
    test("ByEntityCondition.EntityCondition.RelativeDistanceCondition", 0)

def test_by_entity_condition_entity_condition_relative_distance_condition_freespace():
    test("ByEntityCondition.EntityCondition.RelativeDistanceConditionFreespace", 6)

def test_by_entity_condition_entity_condition_relative_speed_condition():
    test("ByEntityCondition.EntityCondition.RelativeSpeedCondition", 0)

def test_by_entity_condition_entity_condition_time_to_collision_condition():
    test("ByEntityCondition.EntityCondition.TimeToCollisionCondition", 0)

def test_by_value_condition_user_defined_value_condition():
    test("ByValueCondition.UserDefinedValueCondition", 0)

def test_controller_action_assign_controller_action():
    test("ControllerAction.AssignControllerAction", 0)

def test_custom_command_action_fault_injection_action():
    test("CustomCommandAction.FaultInjectionAction", 0)

def test_custom_command_action_pseudo_traffic_signal_detector_confidence_set_action_v1():
    test("CustomCommandAction.PseudoTrafficSignalDetectorConfidenceSetAction@v1", 0)

def test_custom_command_action_request_to_cooperate_command_action_v1():
    test("CustomCommandAction.RequestToCooperateCommandAction@v1", 0)

def test_custom_command_action_v2i_traffic_signal_state_action():
    test("CustomCommandAction.V2ITrafficSignalStateAction", 0)

def test_environment():
    test("Environment", 0)

def test_lateral_action_lane_change_action_road_shoulder():
    test("LateralAction.LaneChangeAction-RoadShoulder", 0)

def test_lateral_action_lane_change_action():
    test("LateralAction.LaneChangeAction", 0)

def test_longitudinal_action_speed_action():
    test("LongitudinalAction.SpeedAction", 0)

def test_longitudinal_action_speed_profile_action():
    test("LongitudinalAction.SpeedProfileAction", 0)

def test_property_detected_object_ground_truth_publishing_delay():
    test("Property.detectedObjectGroundTruthPublishingDelay", 0)

def test_property_detected_object_missing_probability():
    test("Property.detectedObjectMissingProbability", 0)

def test_property_detected_object_position_standard_deviation():
    test("Property.detectedObjectPositionStandardDeviation", 0)

def test_property_detected_object_publishing_delay():
    test("Property.detectedObjectPublishingDelay", 0)

def test_property_detection_sensor_range():
    test("Property.detectionSensorRange", 4)

def test_property_is_blind():
    test("Property.isBlind", 0)

def test_property_max_speed():
    test("Property.maxSpeed", 0)

def test_property_pointcloud_publishing_delay():
    test("Property.pointcloudPublishingDelay", 0)

def test_property_pointcloud_vertical_field_of_view():
    test("Property.pointcloudVerticalFieldOfView", 3)

def test_routing_action_acquire_position_action_continuous():
    test("RoutingAction.AcquirePositionAction-continuous", 0)

def test_routing_action_acquire_position_action():
    test("RoutingAction.AcquirePositionAction", 0)

def test_routing_action_assign_route_action():
    test("RoutingAction.AssignRouteAction", 0)

def test_routing_action_follow_trajectory_action_autoware():
    test("RoutingAction.FollowTrajectoryAction-autoware", 0)

def test_routing_action_follow_trajectory_action_override():
    test("RoutingAction.FollowTrajectoryAction-override", 0)

def test_routing_action_follow_trajectory_action_star():
    test("RoutingAction.FollowTrajectoryAction-star", 10)

def test_routing_action_follow_trajectory_action_straight_bicycle():
    test("RoutingAction.FollowTrajectoryAction-straight-bicycle", 0)

def test_routing_action_follow_trajectory_action_straight_pedestrian():
    test("RoutingAction.FollowTrajectoryAction-straight-pedestrian", 0)

def test_routing_action_follow_trajectory_action_straight():
    test("RoutingAction.FollowTrajectoryAction-straight", 0)

def test_routing_action_follow_trajectory_action_zero_distance():
    test("RoutingAction.FollowTrajectoryAction-zero-distance", 0)

def test_routing_action_follow_trajectory_action_zero_time():
    test("RoutingAction.FollowTrajectoryAction-zero-time", 0)

def test_traffic_signal_controller_action():
    test("TrafficSignalControllerAction", 0)

def test_traffic_signals():
    test("TrafficSignals", 0)

def test_visualization_simulation_context():
    test("Visualization.simulation.context", 0)

def test_all_in_one():
    test("all-in-one", 15)

def test_arm_demo():
    test("arm_demo", 0)

def test_autoware_simple():
    test("autoware-simple", 2)

def test_collision():
    test("collision", 9)

def test_collision_condition_by_type():
    test("collision_condition_by_type", 0)

def test_condition():
    test("condition", 0)

def test_distance_condition():
    test("distance-condition", 0)

def test_duplicated_parameter():
    test("duplicated-parameter", 0)

def test_empty():
    test("empty", 0)

def test_execution_time_test():
    test("execution_time_test", 0)

def test_failure_single_condition_anonymous():
    test("failure_single_condition_anonymous", 0)

def test_failure_single_condition_named():
    test("failure_single_condition_named", 0)

def test_failure_single_init_action():
    test("failure_single_init_action", 0)

def test_failure_three_conditions_two_groups_trigger_anonymous():
    test("failure_three_conditions_two_groups_trigger_anonymous", 0)

def test_failure_three_conditions_two_groups_trigger_named():
    test("failure_three_conditions_two_groups_trigger_named", 0)

def test_failure_two_conditions_different_group_trigger_anonymous():
    test("failure_two_conditions_different_group_trigger_anonymous", 0)

def test_failure_two_conditions_different_group_trigger_named():
    test("failure_two_conditions_different_group_trigger_named", 0)

def test_failure_two_conditions_single_group_named():
    test("failure_two_conditions_single_group_named", 0)

def test_failure_two_conditions_single_group_one_named():
    test("failure_two_conditions_single_group_one_named", 0)

def test_failure_two_init_actions():
    test("failure_two_init_actions", 0)

def test_minimal():
    test("minimal", 0)

def test_parameter():
    test("parameter", 22)

def test_prefixed_name_reference():
    test("prefixed-name-reference", 0)

def test_relative_target_speed():
    test("relative_target_speed", 0)

def test_sample():
    test("sample", 2)

def test_sample_awsim():
    test("sample_awsim", 2)

def test_sample_awsim_conventional_traffic_lights():
    test("sample_awsim_conventional_traffic_lights", 4)

def test_sample_awsim_v2i_traffic_lights():
    test("sample_awsim_v2i_traffic_lights", 4)

def test_set_behavior_parameters_in_object_controller():
    test("set_behavior_parameters_in_object_controller", 0)

def test_spawn_relative_object_position():
    test("spawn_relative_object_position", 0)

def test_spawn_relative_world():
    test("spawn_relative_world", 2)

def test_stand_still():
    test("stand-still", 0)

def test_success():
    test("success", 6)
