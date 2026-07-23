#!/usr/bin/env python3
# Copyright 2025 TIER IV, Inc. All rights reserved.
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

"""Task #4 done-bar: the Python-boundary equivalent of headless_interpreter_drive_check.cpp.

Requires the SSV2_HEADLESS_EGO overlay built and sourced (see cpp_tools/build.sh --validator and
env/setup.bash). Generates a minimal kashiwanoha scenario with an Autoware ego + one NPC, drives it
in-process through openscenario_python, and asserts the seam end to end:

  * the headless EgoEntity follows the injected Diffusion-Planner trajectory (advances ~3 m/s),
  * get_entity_states() returns NPC truth for scoring,
  * the turn-indicator command round-trips,
  * the scenario reaches a clean terminal outcome (exitSuccess -> Pass) and tears down at exit 0.

Run:  python headless_smoke_test.py
"""
import math
import os
import sys
import tempfile

import numpy as np
from ament_index_python.packages import get_package_share_directory

import openscenario_python as osp

# Lanelet 120545 is the drivable kashiwanoha lane R1c used. The ego is designated Autoware via the
# ObjectController isEgo property (that is what spawns a (headless) EgoEntity, not the Vehicle
# property). A CustomCommandAction exitSuccess at t>3 s is how OpenSCENARIO scenarios self-terminate
# (a bare SimulationTimeCondition StopTrigger only completes the storyboard; it never throws).
_SCENARIO_TEMPLATE = """<?xml version="1.0" encoding="UTF-8"?>
<OpenSCENARIO>
  <FileHeader revMajor="1" revMinor="3" date="2026-07-23T00:00:00" description="headless smoke" author="validator"/>
  <ParameterDeclarations/>
  <CatalogLocations/>
  <RoadNetwork>
    <LogicFile filepath="{map_osm}"/>
    <SceneGraphFile filepath=""/>
  </RoadNetwork>
  <Entities>
    <ScenarioObject name="ego">
      <Vehicle name="ego-car" vehicleCategory="car">
        <ParameterDeclarations/>
        <Performance maxSpeed="50.0" maxAcceleration="10.0" maxDeceleration="10.0"/>
        <BoundingBox>
          <Center x="1.5" y="0.0" z="0.9"/>
          <Dimensions width="2.1" length="4.5" height="1.8"/>
        </BoundingBox>
        <Axles>
          <FrontAxle maxSteering="0.5" wheelDiameter="0.6" trackWidth="1.8" positionX="3.1" positionZ="0.3"/>
          <RearAxle maxSteering="0.0" wheelDiameter="0.6" trackWidth="1.8" positionX="0.0" positionZ="0.3"/>
        </Axles>
        <Properties/>
      </Vehicle>
      <ObjectController>
        <Controller name="ego-controller">
          <Properties>
            <Property name="isEgo" value="true"/>
          </Properties>
        </Controller>
      </ObjectController>
    </ScenarioObject>
    <ScenarioObject name="npc">
      <Vehicle name="npc-car" vehicleCategory="car">
        <ParameterDeclarations/>
        <Performance maxSpeed="50.0" maxAcceleration="10.0" maxDeceleration="10.0"/>
        <BoundingBox>
          <Center x="1.5" y="0.0" z="0.9"/>
          <Dimensions width="2.1" length="4.5" height="1.8"/>
        </BoundingBox>
        <Axles>
          <FrontAxle maxSteering="0.5" wheelDiameter="0.6" trackWidth="1.8" positionX="3.1" positionZ="0.3"/>
          <RearAxle maxSteering="0.0" wheelDiameter="0.6" trackWidth="1.8" positionX="0.0" positionZ="0.3"/>
        </Axles>
        <Properties/>
      </Vehicle>
    </ScenarioObject>
  </Entities>
  <Storyboard>
    <Init>
      <Actions>
        <Private entityRef="ego">
          <PrivateAction>
            <TeleportAction>
              <Position><LanePosition roadId="0" laneId="120545" s="0" offset="0"/></Position>
            </TeleportAction>
          </PrivateAction>
        </Private>
        <Private entityRef="npc">
          <PrivateAction>
            <TeleportAction>
              <Position><LanePosition roadId="0" laneId="120545" s="15" offset="0"/></Position>
            </TeleportAction>
          </PrivateAction>
          <PrivateAction>
            <LongitudinalAction>
              <SpeedAction>
                <SpeedActionDynamics dynamicsShape="step" value="0" dynamicsDimension="time"/>
                <SpeedActionTarget><AbsoluteTargetSpeed value="0"/></SpeedActionTarget>
              </SpeedAction>
            </LongitudinalAction>
          </PrivateAction>
        </Private>
      </Actions>
    </Init>
    <Story name="main">
      <ParameterDeclarations/>
      <Act name="act">
        <ManeuverGroup name="mg" maximumExecutionCount="1">
          <Actors selectTriggeringEntities="false"><EntityRef entityRef="npc"/></Actors>
          <Maneuver name="mv">
            <ParameterDeclarations/>
            <Event name="finish" priority="overwrite">
              <Action name="finish">
                <UserDefinedAction><CustomCommandAction type="exitSuccess"/></UserDefinedAction>
              </Action>
              <StartTrigger>
                <ConditionGroup>
                  <Condition name="after-3s" conditionEdge="none" delay="0">
                    <ByValueCondition><SimulationTimeCondition value="3" rule="greaterThan"/></ByValueCondition>
                  </Condition>
                </ConditionGroup>
              </StartTrigger>
            </Event>
          </Maneuver>
        </ManeuverGroup>
        <StartTrigger>
          <ConditionGroup>
            <Condition name="start" conditionEdge="none" delay="0">
              <ByValueCondition><SimulationTimeCondition value="0" rule="greaterThan"/></ByValueCondition>
            </Condition>
          </ConditionGroup>
        </StartTrigger>
      </Act>
    </Story>
    <StopTrigger/>
  </Storyboard>
</OpenSCENARIO>
"""


def write_scenario(path: str) -> None:
    map_osm = os.path.join(get_package_share_directory("kashiwanoha_map"), "map", "lanelet2_map.osm")
    with open(path, "w") as f:
        f.write(_SCENARIO_TEMPLATE.format(map_osm=map_osm))


def main() -> int:
    scenario = os.path.join(tempfile.mkdtemp(prefix="osp_smoke_"), "headless_smoke.xosc")
    write_scenario(scenario)

    ok = True
    with osp.HeadlessRunner(osc_path=scenario, output_directory="/tmp/osp_smoke_out",
                            local_frame_rate=10.0) as r:
        assert r.configure() == "inactive", r.state()
        assert r.activate() == "active", r.state()
        assert {"ego", "npc"}.issubset(set(r.get_entity_states())), list(r.get_entity_states())

        ego0 = r.get_ego_state()
        x0, y0, yaw = ego0["pose"]["x"], ego0["pose"]["y"], ego0["pose"]["yaw"]

        pts = np.array([[x0 + i * math.cos(yaw), y0 + i * math.sin(yaw), yaw, 3.0]
                        for i in range(60)], dtype=float)
        r.set_ego_trajectory(pts)
        r.set_ego_turn_indicator(2)  # ENABLE_LEFT

        outcome, steps = "running", 0
        for steps in range(1, 101):
            outcome = r.step()
            if outcome == "terminated":
                break

        e = r.get_ego_state()
        travelled = math.hypot(e["pose"]["x"] - x0, e["pose"]["y"] - y0)
        npc = r.get_entity_states().get("npc")
        print(f"steps={steps} outcome={outcome} result_kind={r.result_kind()} "
              f"travelled={travelled:.2f}m vx={e['twist']['linear_x']:.2f} "
              f"turn_indicator={e['turn_indicator']}")

        checks = {
            "terminated": outcome == "terminated",
            "result_kind==Pass": r.result_kind() == "Pass",
            "ego advanced >2m": travelled > 2.0,
            "ego vx ~3": abs(e["twist"]["linear_x"] - 3.0) < 0.5,
            "npc truth returned": npc is not None and npc["type"] == 1,
            "turn indicator round-trip": e["turn_indicator"] == "ENABLE_LEFT",
        }
        for k, v in checks.items():
            print(f"  [{'OK' if v else 'FAIL'}] {k}")
            ok = ok and v

    print("SMOKE-OK" if ok else "SMOKE-FAIL")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
