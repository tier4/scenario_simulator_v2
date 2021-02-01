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

#include <xmlrpc_interface/conversions.hpp>

#include <string>

namespace xmlrpc_interfae
{
void toProto(const XmlRpc::XmlRpcValue & from, simulation_api_schema::InitializeResponse & to)
{
  to = simulation_api_schema::InitializeResponse();
  to.mutable_result()->set_success(xmlrpc_interfae::getXmlValue<bool>(from, key_success));
  to.mutable_result()->set_description(
    xmlrpc_interfae::getXmlValue<std::string>(
      from,
      key_description));
}

void fromProto(const simulation_api_schema::InitializeResponse & from, XmlRpc::XmlRpcValue & to)
{
  to = XmlRpc::XmlRpcValue();
  to[key_success] = from.result().success();
  to[key_description] = from.result().description();
}

void toProto(const XmlRpc::XmlRpcValue & from, simulation_api_schema::InitializeRequest & to)
{
  to = simulation_api_schema::InitializeRequest();
  to.set_realtime_factor(xmlrpc_interfae::getXmlValue<double>(from, key_realtime_factor));
  to.set_step_time(xmlrpc_interfae::getXmlValue<double>(from, key_step_time));
}

void fromProto(const simulation_api_schema::InitializeRequest & from, XmlRpc::XmlRpcValue & to)
{
  to = XmlRpc::XmlRpcValue();
  to[key_realtime_factor] = from.realtime_factor();
  to[key_step_time] = from.step_time();
}

void toProto(const XmlRpc::XmlRpcValue & from, simulation_api_schema::UpdateFrameRequest & to)
{
  to = simulation_api_schema::UpdateFrameRequest();
  to.set_current_time(from[key_current_time]);
}

void fromProto(const simulation_api_schema::UpdateFrameRequest & from, XmlRpc::XmlRpcValue & to)
{
  to = XmlRpc::XmlRpcValue();
  to[key_current_time] = from.current_time();
}

void toProto(const XmlRpc::XmlRpcValue & from, simulation_api_schema::UpdateFrameResponse & to)
{
  to = simulation_api_schema::UpdateFrameResponse();
  to.mutable_result()->set_success(from[key_success]);
  to.mutable_result()->set_description(from[key_description]);
}

void fromProto(const simulation_api_schema::UpdateFrameResponse & from, XmlRpc::XmlRpcValue & to)
{
  to = XmlRpc::XmlRpcValue();
  to[key_success] = from.result().success();
  to[key_description] = from.result().description();
}
}  // namespace xmlrpc_interfae
