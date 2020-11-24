// Copyright 2015-2020 TierIV.inc. All rights reserved.
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

#include <scenario_simulator/scenario_simulator.hpp>

#include <string>
#include <vector>
#include <memory>

namespace scenario_simulator
{
ScenarioSimulator::ScenarioSimulator(const rclcpp::NodeOptions & options)
: Node("scenario_simulator", options)
{
  declare_parameter("port", 8080);
  get_parameter("port", port_);
  auto initialize_func = std::bind(&ScenarioSimulator::initialize, this,
      std::placeholders::_1, std::placeholders::_2);
  addMethod("initialize", initialize_func);
  auto update_frame_func = std::bind(&ScenarioSimulator::updateFrame, this,
      std::placeholders::_1, std::placeholders::_2);
  addMethod("update_frame", update_frame_func);
  auto spawn_entity_func = std::bind(&ScenarioSimulator::spawnEntity, this,
      std::placeholders::_1, std::placeholders::_2);
  addMethod("spawn_entity", spawn_entity_func);
  auto despawn_entity_func = std::bind(&ScenarioSimulator::despawnEntity, this,
      std::placeholders::_1, std::placeholders::_2);
  addMethod("despawn_entity", despawn_entity_func);
  auto get_entity_status_func = std::bind(&ScenarioSimulator::getEntityStatus, this,
      std::placeholders::_1, std::placeholders::_2);
  addMethod("get_entity_status", get_entity_status_func);
  server_.bindAndListen(port_);
  server_.enableIntrospection(true);
  xmlrpc_thread_ = std::thread(&ScenarioSimulator::runXmlRpc, this);
}

ScenarioSimulator::~ScenarioSimulator()
{
  xmlrpc_thread_.join();
}

void ScenarioSimulator::runXmlRpc()
{
  while (rclcpp::ok()) {
    server_.work(1);
  }
}

void ScenarioSimulator::updateFrame(XmlRpc::XmlRpcValue & param, XmlRpc::XmlRpcValue & result)
{
  if (checkRequiredFields({"runner/current_time"}, param, result)) {
    impl_.updateFrame(param, result);
  }
}

void ScenarioSimulator::initialize(XmlRpc::XmlRpcValue & param, XmlRpc::XmlRpcValue & result)
{
  if (checkRequiredFields({"sim/realtime_factor", "sim/step_time"}, param, result)) {
    impl_.initialize(param, result);
  }
}

void ScenarioSimulator::spawnEntity(XmlRpc::XmlRpcValue & param, XmlRpc::XmlRpcValue & result)
{
  if (checkRequiredFields({"entity/is_ego", "entity/name", "entity/catalog_xml"}, param, result)) {
    impl_.spawnEntity(param, result);
  }
}

void ScenarioSimulator::despawnEntity(XmlRpc::XmlRpcValue & param, XmlRpc::XmlRpcValue & result)
{
  if (checkRequiredFields({"entity/name"}, param, result)) {
    impl_.despawnEntity(param, result);
  }
}

void ScenarioSimulator::getEntityStatus(XmlRpc::XmlRpcValue & param, XmlRpc::XmlRpcValue & result)
{
  if (checkRequiredFields({"entity/name"}, param, result)) {
    impl_.getEntityStatus(param, result);
  }
}

bool ScenarioSimulator::checkRequiredFields(
  std::vector<std::string> required_fields,
  XmlRpc::XmlRpcValue & param,
  XmlRpc::XmlRpcValue & result)
{
  result = XmlRpc::XmlRpcValue();

  auto non_existing_fileds = getNonExistingRequredFields(required_fields, param);
  if (non_existing_fileds.size() != 0) {
    int i = 0;
    for (auto itr = non_existing_fileds.begin(); itr != non_existing_fileds.end(); itr++) {
      result["required_fields"][i] = *itr;
      i++;
    }
    return false;
  } else {
    return true;
  }
}

void ScenarioSimulator::addMethod(
  std::string name, std::function<void(XmlRpc::XmlRpcValue &,
  XmlRpc::XmlRpcValue &)> func)
{
  auto method_ptr = std::make_shared<scenario_simulator::XmlRpcMethod>(name, &server_);
  method_ptr->setFunction(func);
  methods_[name] = method_ptr;
}

std::vector<std::string> ScenarioSimulator::getNonExistingRequredFields(
  std::vector<std::string> required_fields, XmlRpc::XmlRpcValue & param)
{
  std::vector<std::string> ret;
  for (auto itr = required_fields.begin(); itr != required_fields.end(); itr++) {
    if (!param.hasMember(*itr)) {
      ret.push_back(*itr);
    }
  }
  return ret;
}
}  // namespace scenario_simulator
