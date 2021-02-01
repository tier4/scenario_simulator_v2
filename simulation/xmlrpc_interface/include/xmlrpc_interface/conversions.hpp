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

#ifndef XMLRPC_INTERFACE__CONVERSIONS_HPP_
#define XMLRPC_INTERFACE__CONVERSIONS_HPP_

#include <simulation_api_schema.pb.h>
#include <xmlrpcpp/XmlRpc.h>

#include <string>

#include <exception>

namespace xmlrpc_interfae
{

class XmlParameterError : public std::runtime_error
{
public:
  explicit XmlParameterError(
    std::string message,
    const char * file,
    int line)
  : runtime_error(message + "\nFile:" + file + "\nLine:" + std::to_string(line)) {}
};

#define THROW_XML_PARAMETER_ERROR(description) \
  throw XmlParameterError( \
    description, __FILE__, __LINE__);

#define THROW_XML_PARAMETER_NOT_DEFINED_ERROR(name) \
  throw XmlParameterError( \
    std::string("parameter : ") + name + std::string(" does not defined."), __FILE__, __LINE__);

void toProto(const XmlRpc::XmlRpcValue & from, simulation_api_schema::InitializeRequest & to);
void fromProto(const simulation_api_schema::InitializeRequest & from, XmlRpc::XmlRpcValue & to);
void toProto(const XmlRpc::XmlRpcValue & from, simulation_api_schema::InitializeResponse & to);
void fromProto(const simulation_api_schema::InitializeResponse & from, XmlRpc::XmlRpcValue & to);
void toProto(const XmlRpc::XmlRpcValue & from, simulation_api_schema::UpdateFrameRequest & to);
void fromProto(const simulation_api_schema::UpdateFrameRequest & from, XmlRpc::XmlRpcValue & to);
void toProto(const XmlRpc::XmlRpcValue & from, simulation_api_schema::UpdateFrameResponse & to);
void fromProto(const simulation_api_schema::UpdateFrameResponse & from, XmlRpc::XmlRpcValue & to);

const char key_success[] = "success";

template<typename T>
const std::string serialize(const XmlRpc::XmlRpcValue & from)
{
  std::string serialized_str = "";
  T to;
  toProto(from, to);
  to.SerializeToString(&serialized_str);
  return serialized_str;
}
}  // namespace xmlrpc_interfae


#endif  // XMLRPC_INTERFACE__CONVERSIONS_HPP_
