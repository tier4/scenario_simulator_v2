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
# | Phase    | V2I sync | conventional | v2i   | v2i detected | v2i output |
# |----------|----------|--------------|-------|--------------|------------|
# | phase-1  | ON       | green        | -     | -            | green      |
# | phase-2  | OFF      | red          | -     | -            | green      |
# | phase-3  | ON       | green        | -     | yellow       | yellow     |
# | phase-4  | ON       | red          | green | -            | green      |
#
# Expected changes:
# CONVENTIONAL (34802): GREEN -> RED -> GREEN -> RED
# V2I          (34806): GREEN -> AMBER -> GREEN

@analyze
class AnalyzeBasicReplay:
    _cached_v2i_messages = None

    def test_traffic_light_phase_changes(self):
        all_messages = list(read_messages(self.reader, topics=[CONVENTIONAL_TOPIC, V2I_TOPIC]))

        # Split by topic
        conventional_messages = [(t, m, ts) for t, m, ts in all_messages if t == CONVENTIONAL_TOPIC]
        v2i_messages = [(t, m, ts) for t, m, ts in all_messages if t == V2I_TOPIC]

        AnalyzeBasicReplay._cached_v2i_messages = v2i_messages

        WAY_ID = 34802
        RELATION_ID = 34806

        # Conventional GroundTruth
        assert len(conventional_messages) >= 1
        conventional = extract_conventional_changes(conventional_messages)
        assert len(conventional[WAY_ID]) == 4

        assert_conventional_state(conventional[WAY_ID][0], TrafficLightBulbV1.GREEN)  # phase-1 (0-3s)
        assert_conventional_state(conventional[WAY_ID][1], TrafficLightBulbV1.RED)    # phase-2 (3-6s)
        assert_conventional_state(conventional[WAY_ID][2], TrafficLightBulbV1.GREEN)  # phase-3 (6-9s)
        assert_conventional_state(conventional[WAY_ID][3], TrafficLightBulbV1.RED)    # phase-4 (9-12s)

        # V2I Output
        assert len(v2i_messages) >= 1
        v2i = extract_v2i_changes(v2i_messages)

        assert len(v2i[RELATION_ID]) == 3, \
            f"Expected 3 V2I changes in 1 cycle, got {len(v2i[RELATION_ID])}"

        assert_v2i_state(v2i[RELATION_ID][0], TrafficLightElement.GREEN)   # 0.0s: phase-1
        assert_v2i_state(v2i[RELATION_ID][1], TrafficLightElement.AMBER)   # 6.0s: phase-3 (v2i_detected)
        assert_v2i_state(v2i[RELATION_ID][2], TrafficLightElement.GREEN)   # 9.0s: phase-4 (v2i)

    def test_traffic_light_predictions(self):
        """Verify predictions field in V2I signals"""
        if AnalyzeBasicReplay._cached_v2i_messages is not None:
            v2i_messages = AnalyzeBasicReplay._cached_v2i_messages
        else:
            v2i_messages = list(read_messages(self.reader, topics=[V2I_TOPIC]))

        RELATION_ID = 34806

        start_time = None
        for _, msg, timestamp in v2i_messages:
            if start_time is None:
                start_time = timestamp / 1e9
                break

        phase1_predictions = None
        phase2_predictions = None
        for _, msg, ts in v2i_messages:
            relative_time = (ts / 1e9) - start_time
            for group in msg.traffic_light_groups:
                if group.traffic_light_group_id == RELATION_ID and group.predictions:
                    # check prediction metadata
                    if phase1_predictions is None and phase2_predictions is None:
                        for prediction in group.predictions:
                            assert prediction.information_source == PredictedTrafficLightState.INFORMATION_SOURCE_SIMULATION, \
                                f"Expected information_source to be INFORMATION_SOURCE_SIMULATION, got {prediction.information_source}"
                            assert prediction.reliability == 1.0, \
                                f"Expected reliability to be 1.0, got {prediction.reliability}"
                            assert len(prediction.simultaneous_elements) > 0, \
                                "Expected simultaneous_elements to be non-empty"

                    # phase-1 is from 0s to 3s (use middle of phase-1)
                    if phase1_predictions is None and 0.5 < relative_time < 2.5:
                        phase1_predictions = group.predictions
                    # phase-2 is from 3s to 6s (sync OFF, use middle of phase-2)
                    if phase2_predictions is None and 3.5 < relative_time < 5.5:
                        phase2_predictions = group.predictions
                    if phase1_predictions and phase2_predictions:
                        break
            if phase1_predictions and phase2_predictions:
                break

        # prediction check on phase-1 (sync ON)
        assert phase1_predictions is not None and len(phase1_predictions) == 6, \
            f"Expected 6 predictions, got {len(phase1_predictions)}"

        expected_phase1_prediction_colors = [
            TrafficLightElement.RED,    # phase-2  (conventional)
            TrafficLightElement.AMBER,  # phase-3  (v2i_detected)
            TrafficLightElement.GREEN,  # phase-4  (v2i)
            TrafficLightElement.GREEN,  # phase-1' (conventional)
            TrafficLightElement.RED,    # phase-2' (conventional)
            TrafficLightElement.AMBER,  # phase-3' (v2i_detected)
        ]

        for i, expected_color in enumerate(expected_phase1_prediction_colors):
            actual_color = phase1_predictions[i].simultaneous_elements[0].color
            assert actual_color == expected_color, \
                f"Phase-1 prediction {i+1}: expected {expected_color}, got {actual_color}"

        # prediction check on phase-2 (sync OFF)
        assert phase2_predictions is not None and len(phase2_predictions) == 6, \
            f"Expected 6 predictions (1 phase = 1 prediction), got {len(phase2_predictions)}"

        expected_phase2_prediction_colors = [
            TrafficLightElement.AMBER,  # phase-3  (v2i detected)
            TrafficLightElement.GREEN,  # phase-4  (v2i)
            TrafficLightElement.GREEN,  # phase-1' (continued)
            TrafficLightElement.GREEN,  # phase-2' (continued)
            TrafficLightElement.AMBER,  # phase-3' (v2i detected)
            TrafficLightElement.GREEN,  # phase-4' (v2i)
        ]

        for i, expected_color in enumerate(expected_phase2_prediction_colors):
            actual_color = phase2_predictions[i].simultaneous_elements[0].color
            assert actual_color == expected_color, \
                f"Phase-2 prediction {i+1}: expected {expected_color}, got {actual_color}"
