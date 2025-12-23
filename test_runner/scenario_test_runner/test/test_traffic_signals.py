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


@fixtures.parameterize(
    [LocalFixture(path=Path(__file__).parent / "fixtures/empty.mcap")]
)
class Fixtures:
    input_topics = []
    output_topics = ["/simulation/traffic_lights"]


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
                        ("initialize_duration", "40"),
                        ("launch_autoware", "False"),
                        ("record", "False"),
                        ("launch_rviz", "True"),
                    ],
                )
            ]
        )


@analyze
class AnalyzeBasicReplay:
    def test_cmd_vel(self):
        msgs_it = read_messages(self.reader, topics=["/simulation/traffic_lights"])

        msgs = [(topic_name, msg, timestamp) for topic_name, msg, timestamp in msgs_it]
        assert len(msgs) >= 1
        changes = {}
        for topic_name, msg, timestamp in msgs:
            lights = msg.traffic_lights
            for light in lights:
                changes[light.lanelet_way_id] = changes.get(light.lanelet_way_id, [])
                has_changed = changes[light.lanelet_way_id] == [] or (
                    changes[light.lanelet_way_id][-1] != light.traffic_light_bulbs
                )
                if has_changed:
                    changes[light.lanelet_way_id].append(light.traffic_light_bulbs)

        assert len(changes[34802]) == 3
        assert len(changes[34836]) == 1

        assert changes[34802][0][0].color == TrafficLightBulbV1.GREEN
        assert changes[34802][0][0].shape == TrafficLightBulbV1.RIGHT_ARROW
        assert changes[34802][0][0].status == TrafficLightBulbV1.SOLID_ON

        assert changes[34802][0][1].color == TrafficLightBulbV1.RED
        assert changes[34802][0][1].shape == TrafficLightBulbV1.CIRCLE
        assert changes[34802][0][1].status == TrafficLightBulbV1.SOLID_ON

        assert changes[34802][1][0].color == TrafficLightBulbV1.AMBER
        assert changes[34802][1][0].shape == TrafficLightBulbV1.CIRCLE
        assert changes[34802][1][0].status == TrafficLightBulbV1.SOLID_ON

        assert changes[34802][2][0].color == TrafficLightBulbV1.GREEN
        assert changes[34802][2][0].shape == TrafficLightBulbV1.CIRCLE
        assert changes[34802][2][0].status == TrafficLightBulbV1.SOLID_ON

        assert changes[34802][2][1].color == TrafficLightBulbV1.GREEN
        assert changes[34802][2][1].shape == TrafficLightBulbV1.RIGHT_ARROW
        assert changes[34802][2][1].status == TrafficLightBulbV1.SOLID_OFF
