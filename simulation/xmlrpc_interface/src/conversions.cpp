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


namespace xmlrpc_interfae
{
void toProto(const XmlRpc::XmlRpcValue from, simulation_api_schema::InitializeResponse & to)
{
  to = simulation_api_schema::InitializeResponse();
  simulation_api_schema::Result result;
  result.set_success(from["success"]);
  result.set_description(from["description"]);
  to.set_allocated_result(&result);
}

void fromProto(const simulation_api_schema::InitializeResponse & from, XmlRpc::XmlRpcValue & to)
{
  to = XmlRpc::XmlRpcValue();
  to["success"] = from.result().success();
  to["description"] = from.result().description();
}

void toProto(const XmlRpc::XmlRpcValue & from, simulation_api_schema::InitializeRequest & to)
{
  to = simulation_api_schema::InitializeRequest();
  to.set_realtime_factor(from["realtime_factor"]);
  to.set_step_time(from["step_time"]);
}

void fromProto(const simulation_api_schema::InitializeRequest & from, XmlRpc::XmlRpcValue & to)
{
  to = XmlRpc::XmlRpcValue();
  to["realtime_factor"] = from.realtime_factor();
  to["step_time"] = from.step_time();
}

void toProto(const XmlRpc::XmlRpcValue & from, simulation_api_schema::UpdateFrameRequest & to)
{
  to = simulation_api_schema::UpdateFrameRequest();
  to.set_current_time(from["current_time"]);
}

void fromProto(const simulation_api_schema::UpdateFrameRequest & from, XmlRpc::XmlRpcValue & to)
{
  to = XmlRpc::XmlRpcValue();
  to["current_time"] = from.current_time();
}

void toProto(const XmlRpc::XmlRpcValue & from, simulation_api_schema::UpdateFrameResponse & to)
{
  to = simulation_api_schema::UpdateFrameResponse();
  simulation_api_schema::Result result;
  result.set_success(from["success"]);
  result.set_description(from["description"]);
  to.set_allocated_result(&result);
}
}  // namespace xmlrpc_interfae
