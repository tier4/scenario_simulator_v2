from replay_testing import (
    fixtures,
    run,
    analyze,
    LocalFixture,
    read_messages,
)
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from ament_index_python.packages import get_package_share_directory
from pathlib import Path
from replay_testing.models import ReplayRunParams, RunnerArgs
from traffic_simulator_msgs.msg import TrafficLightBulbV1
from autoware_perception_msgs.msg import TrafficLightElement, PredictedTrafficLightState

def extract_conventional_changes(msgs):
    changes = {}
    for _, msg, _ in msgs:
        for light in msg.traffic_lights:
            lid = light.lanelet_way_id
            changes[lid] = changes.get(lid, [])
            bulbs = list(light.traffic_light_bulbs)
            if not changes[lid] or changes[lid][-1] != bulbs:
                changes[lid].append(bulbs)
    return changes


def extract_v2i_changes(msgs):
    changes = {}
    for _, msg, _ in msgs:
        for group in msg.traffic_light_groups:
            gid = group.traffic_light_group_id
            changes[gid] = changes.get(gid, [])
            elements = list(group.elements)
            if not changes[gid] or changes[gid][-1] != elements:
                changes[gid].append(elements)
    return changes


def extract_v2i_predictions(msgs):
    """Extract predictions from V2I messages"""
    all_predictions = {}
    for _, msg, _ in msgs:
        for group in msg.traffic_light_groups:
            gid = group.traffic_light_group_id
            if group.predictions:
                all_predictions[gid] = all_predictions.get(gid, [])
                # Store prediction snapshots to track changes
                snapshot = [(p.predicted_stamp, list(p.simultaneous_elements))
                           for p in group.predictions]
                if not all_predictions[gid] or all_predictions[gid][-1] != snapshot:
                    all_predictions[gid].append(snapshot)
    return all_predictions


def assert_conventional_state(bulbs, color, shape=TrafficLightBulbV1.CIRCLE, status=TrafficLightBulbV1.SOLID_ON):
    assert bulbs[0].color == color
    assert bulbs[0].shape == shape
    assert bulbs[0].status == status


def assert_v2i_state(elements, color, shape=TrafficLightElement.CIRCLE, status=TrafficLightElement.SOLID_ON):
    assert elements[0].color == color
    assert elements[0].shape == shape
    assert elements[0].status == status

CONVENTIONAL_TOPIC = "/simulation/traffic_lights"
V2I_TOPIC = "/perception/traffic_light_recognition/external/traffic_signals"

@fixtures.parameterize(
    [LocalFixture(path=Path(__file__).parent / "fixtures/empty.mcap")]
)
class Fixtures:
    input_topics = []
    output_topics = [CONVENTIONAL_TOPIC, V2I_TOPIC]


@run.default(
    params=ReplayRunParams(
        name="default",
        params={},
        runner_args=RunnerArgs(use_clock=False),
        ignore_playback_finish=True,
    )
)
class Run:
    def generate_launch_description(self) -> LaunchDescription:
        return LaunchDescription(
            [
                IncludeLaunchDescription(
                    [
                        get_package_share_directory("scenario_test_runner"),
                        "/launch/",
                        "scenario_test_runner.launch.py",
                    ],
                    launch_arguments=[
                        (
                            "scenario",
                            [
                                get_package_share_directory("scenario_test_runner"),
                                "/scenario/",
                                "TrafficSignalsNonEgo.yaml",
                            ],
                        ),
                        ("initialize_duration", "15"),
                        ("launch_autoware", "False"),
                        ("record", "False"),
                        ("launch_rviz", "True"),
                    ],
                )
            ]
        )

# Scenario phases (3s each):
# | Phase    | V2I sync | conventional | v2i    | v2i detected | v2i output |
# |----------|----------|--------------|--------|--------------|------------|
# | phase-1  | ON       | green        | green  | -            | green      |
# | phase-2  | OFF      | red          | green* | -            | green      |
# | phase-3  | ON       | green        | green  | yellow       | yellow     |
# | phase-4  | ON       | red          | red    | -            | red        |
# | phase-1' | ON       | green        | green  | -            | green      |
#
# *phase-2: V2I maintains previous state (green) because sync is OFF
#
# Expected changes:
# CONVENTIONAL (34802): GREEN -> RED -> GREEN -> RED -> GREEN
# V2I          (34806): GREEN -> AMBER -> RED -> GREEN

@analyze
class AnalyzeBasicReplay:
    def test_traffic_light_phase_changes(self):
        all_messages = list(read_messages(self.reader, topics=[CONVENTIONAL_TOPIC, V2I_TOPIC]))

        # Split by topic
        conventional_messages = [(t, m, ts) for t, m, ts in all_messages if t == CONVENTIONAL_TOPIC]
        v2i_messages = [(t, m, ts) for t, m, ts in all_messages if t == V2I_TOPIC]

        WAY_ID = 34802
        RELATION_ID = 34806

        # Conventional GroundTruth
        assert len(conventional_messages) >= 1
        conventional = extract_conventional_changes(conventional_messages)
        assert len(conventional[WAY_ID]) == 5

        assert_conventional_state(conventional[WAY_ID][0], TrafficLightBulbV1.GREEN)
        assert_conventional_state(conventional[WAY_ID][1], TrafficLightBulbV1.RED)
        assert_conventional_state(conventional[WAY_ID][2], TrafficLightBulbV1.GREEN)
        assert_conventional_state(conventional[WAY_ID][3], TrafficLightBulbV1.RED)
        assert_conventional_state(conventional[WAY_ID][4], TrafficLightBulbV1.GREEN)

        # V2I Output
        assert len(v2i_messages) >= 1
        v2i = extract_v2i_changes(v2i_messages)
        assert len(v2i[RELATION_ID]) == 4

        assert_v2i_state(v2i[RELATION_ID][0], TrafficLightElement.GREEN)
        assert_v2i_state(v2i[RELATION_ID][1], TrafficLightElement.AMBER)
        assert_v2i_state(v2i[RELATION_ID][2], TrafficLightElement.RED)
        assert_v2i_state(v2i[RELATION_ID][3], TrafficLightElement.GREEN)


@analyze
class AnalyzePredictions:
    def test_traffic_light_predictions(self):
        """Verify predictions field in V2I signals"""
        v2i_messages = list(read_messages(self.reader, topics=[V2I_TOPIC]))

        RELATION_ID = 34806

        # Check that predictions are present in some messages
        has_predictions = False
        for _, msg, _ in v2i_messages:
            for group in msg.traffic_light_groups:
                if group.traffic_light_group_id == RELATION_ID and group.predictions:
                    has_predictions = True
                    for prediction in group.predictions:
                        # Verify information_source is INFORMATION_SOURCE_SIMULATION
                        assert prediction.information_source == PredictedTrafficLightState.INFORMATION_SOURCE_SIMULATION, \
                            f"Expected information_source to be INFORMATION_SOURCE_SIMULATION, got {prediction.information_source}"
                        # Verify reliability is 1.0
                        assert prediction.reliability == 1.0, \
                            f"Expected reliability to be 1.0, got {prediction.reliability}"
                        # Verify simultaneous_elements is not empty
                        assert len(prediction.simultaneous_elements) > 0, \
                            "Expected simultaneous_elements to be non-empty"

        assert has_predictions, "No predictions found in V2I messages for sync-enabled traffic signal"
