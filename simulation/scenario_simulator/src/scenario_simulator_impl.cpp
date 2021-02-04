// Copyright 2015-2020 Tier IV, Inc. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <scenario_simulator/scenario_simulator_impl.hpp>
#include <scenario_simulator/exception.hpp>

#include <xmlrpc_interface/conversions.hpp>

#include <quaternion_operation/quaternion_operation.h>

#include <simulation_api/entity/vehicle_parameter.hpp>
#include <pugixml.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/rclcpp.hpp>

#include <memory>
#include <utility>
#include <vector>

namespace scenario_simulator
{
ScenarioSimulatorImpl::ScenarioSimulatorImpl()
{
  initialized_ = false;
  current_time_ = 0.0;
}

void ScenarioSimulatorImpl::initialize(XmlRpc::XmlRpcValue & param, XmlRpc::XmlRpcValue & result)
{
  if (initialized_) {
    ScenarioSimulatorImpl other {};
    std::swap(*this, other);
  }
  initialized_ = true;
  const auto req =
    xmlrpc_interface::deserializeFromBinValue<simulation_api_schema::InitializeRequest>(param);
  realtime_factor_ = req.realtime_factor();
  step_time_ = req.step_time();
  simulation_api_schema::InitializeResponse res;
  res.mutable_result()->set_success(true);
  res.mutable_result()->set_description("succeed to initialize simulation");
  result = XmlRpc::XmlRpcValue();
  result[xmlrpc_interface::key::response] = xmlrpc_interface::serializeToBinValue(res);
}

void ScenarioSimulatorImpl::updateFrame(XmlRpc::XmlRpcValue & param, XmlRpc::XmlRpcValue & result)
{
  if (!initialized_) {
    simulation_api_schema::UpdateFrameResponse res;
    res.mutable_result()->set_description("simulator have not initialized yet.");
    res.mutable_result()->set_success(false);
    result = XmlRpc::XmlRpcValue();
    result[xmlrpc_interface::key::response] = xmlrpc_interface::serializeToBinValue(res);
    return;
  }
  const auto req =
    xmlrpc_interface::deserializeFromBinValue<simulation_api_schema::UpdateFrameRequest>(param);
  simulation_api_schema::UpdateFrameResponse res;
  if (req.current_time() != current_time_) {
    res.mutable_result()->set_success(false);
    res.mutable_result()->set_description("timestamp of the simulator and runner does not match.");
    result = XmlRpc::XmlRpcValue();
    result[xmlrpc_interface::key::response] = xmlrpc_interface::serializeToBinValue(res);
    return;
  } else {
    res.mutable_result()->set_success(true);
    res.mutable_result()->set_description("succeed to update frame");
    result = XmlRpc::XmlRpcValue();
    result[xmlrpc_interface::key::response] = xmlrpc_interface::serializeToBinValue(res);
    return;
  }
}

void ScenarioSimulatorImpl::spawnVehicleEntity(
  XmlRpc::XmlRpcValue & param,
  XmlRpc::XmlRpcValue & result)
{
  const auto req =
    xmlrpc_interface::deserializeFromBinValue<simulation_api_schema::SpawnVehicleEntityRequest>(
    param);
  vehicles_.emplace_back(req.parameters());
  simulation_api_schema::SpawnVehicleEntityResponse res;
  res.mutable_result()->set_success(true);
  res.mutable_result()->set_description("timestamp of the simulator and runner does not match.");
  result = XmlRpc::XmlRpcValue();
  result[xmlrpc_interface::key::response] = xmlrpc_interface::serializeToBinValue(res);
}

void ScenarioSimulatorImpl::spawnPedestrianEntity(
  XmlRpc::XmlRpcValue & param,
  XmlRpc::XmlRpcValue & result)
{
  const auto req =
    xmlrpc_interface::deserializeFromBinValue<simulation_api_schema::SpawnPedestrianEntityRequest>(
    param);
  pedestrians_.emplace_back(req.parameters());
  simulation_api_schema::SpawnPedestrianEntityResponse res;
  res.mutable_result()->set_success(true);
  res.mutable_result()->set_description("timestamp of the simulator and runner does not match.");
  result = XmlRpc::XmlRpcValue();
  result[xmlrpc_interface::key::response] = xmlrpc_interface::serializeToBinValue(res);
}
}  // namespace scenario_simulator
