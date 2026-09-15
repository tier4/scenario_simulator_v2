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

"""Check that a misc object written in a scenario reaches the point cloud topic as STRUCTURE.

This covers the wiring, not the conversion. `test_lidar_sensor.cpp` drives `LidarSensor` directly
with a hand-built configuration, so it cannot show that `architecture_type` travels from the launch
file to the sensor, nor that a scenario `MiscObject` becomes a classified entity. Everything else
is left to the unit tests.
"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from replay_testing import LocalFixture, analyze, fixtures, read_messages, run
from replay_testing.models import ReplayRunParams, RunnerArgs

POINTCLOUD_TOPIC = "/perception/obstacle_segmentation/pointcloud"

# `autoware::point_types::PointCloudClassification`, mirrored in `segmented_point_type.hpp`.
STRUCTURE = 9

# The layout `architecture_type` selects. `segmented_point_type.hpp` pins the offsets.
POINT_STEP = 24
CLASS_ID_OFFSET = 12


@fixtures.parameterize([LocalFixture(path=Path(__file__).parent / "fixtures/empty.mcap")])
class Fixtures:
    input_topics = []
    output_topics = [POINTCLOUD_TOPIC]


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
                                "SegmentedPointCloud.StructurePoints.yaml",
                            ],
                        ),
                        # The layout follows from this; an older value publishes PointXYZI instead.
                        ("architecture_type", "awf/universe/20260801"),
                        # Without Autoware the storyboard never runs, so the scenario ends by
                        # timing out. The sensors publish meanwhile, which is what this reads.
                        ("initialize_duration", "5"),
                        ("global_timeout", "20"),
                        ("launch_autoware", "False"),
                        ("record", "False"),
                        ("launch_rviz", "False"),
                    ],
                )
            ]
        )


@analyze
class AnalyzeSegmentedPointCloud:
    def test_scenario_misc_objects_reach_the_topic_as_structure(self):
        """`point_step` is checked first because the class_id read below depends on it: at 32 the
        topic carries the legacy `PointXYZI` and that byte means nothing.
        """
        total = 0
        for _, msg, _ in read_messages(self.reader, topics=[POINTCLOUD_TOPIC]):
            assert msg.point_step == POINT_STEP, (
                f"point_step {msg.point_step}: architecture_type did not select PointXYZCPE"
            )
            data = bytes(msg.data)
            for i in range(msg.width):
                class_id = data[i * msg.point_step + CLASS_ID_OFFSET]
                assert class_id == STRUCTURE, f"class_id {class_id}, expected {STRUCTURE}"
            total += msg.width
        assert total, "the misc objects of the scenario should have put points on this topic"
