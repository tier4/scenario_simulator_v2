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

#include <simulation_api_schema.pb.h>

#include <string>

int main()
{
  simulation_api_schema::InitializeResponse res;
  // res.mutable_result()->set_success(true);
  res.mutable_result()->set_description("test");
  XmlRpc::XmlRpcValue xml;
  xmlrpc_interface::serializeToBinValue(res);
  // xmlrpc_interface::fromProto(res, xml);
  // std::string description = xml["description"];
  // std::cout << description.c_str() << std::endl;
  // xmlrpc_interface::toProto(xml, res);
  return 0;
}
