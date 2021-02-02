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
  simulation_api_schema::InitializeRequest req;
  std::vector<char> bin = param;
  req.ParseFromArray(bin.data(), bin.size());
  realtime_factor_ = req.realtime_factor();
  step_time_ = req.step_time();
  simulation_api_schema::InitializeResponse res;
  res.mutable_result()->set_success(true);
  res.mutable_result()->set_description("succeed to initialize simulation");
  result = XmlRpc::XmlRpcValue();
  size_t size = res.ByteSizeLong();
  void * buffer = malloc(size);
  res.SerializeToArray(buffer, size);
  result["return"] = XmlRpc::XmlRpcValue(buffer, size);
  free(buffer);
}

void ScenarioSimulatorImpl::updateFrame(XmlRpc::XmlRpcValue & param, XmlRpc::XmlRpcValue & result)
{
  if (!initialized_) {
    result["message"] = "simulator have not initialized yet.";
    result["sim/current_time"] = current_time_;
    result["sim/update_frame"] = false;
    return;
  }
  double current_time_in_runner = param["runner/current_time"];
  if (current_time_in_runner != current_time_) {
    result["sim/current_time"] = current_time_;
    result["sim/update_frame"] = false;
    result["message"] = "timestamp of the simulator and runner does not match.";
    return;
  }
  current_time_ = current_time_ + step_time_;
  result["sim/update_frame"] = true;
  result["sim/current_time"] = current_time_;
  result["message"] = "succeed to update frame";
}

void ScenarioSimulatorImpl::setEntityStatus(
  [[maybe_unused]] XmlRpc::XmlRpcValue & param,
  XmlRpc::XmlRpcValue & result)
{
  result["success"] = true;
}

void ScenarioSimulatorImpl::getEntityStatus(
  [[maybe_unused]] XmlRpc::XmlRpcValue & param,
  XmlRpc::XmlRpcValue & result)
{
  result["success"] = true;
}

void ScenarioSimulatorImpl::spawnEntity(
  [[maybe_unused]] XmlRpc::XmlRpcValue & param,
  XmlRpc::XmlRpcValue & result)
{
  result["success"] = true;
}

void ScenarioSimulatorImpl::despawnEntity(
  [[maybe_unused]] XmlRpc::XmlRpcValue & param,
  XmlRpc::XmlRpcValue & result)
{
  result["success"] = true;
}
}  // namespace scenario_simulator
