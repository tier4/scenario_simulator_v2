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

#include <geometry_msgs.pb.h>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/pose.hpp>

#include <openscenario_msgs.pb.h>
#include <simulation_api_schema.pb.h>
#include <xmlrpcpp/XmlRpc.h>

#include <string>
#include <vector>
#include <exception>
#include <iostream>

namespace xmlrpc_interface
{
namespace key
{
const char success[] = "success";
const char description[] = "description";
const char realtime_factor[] = "realtime_factor";
const char step_time[] = "step_time";
const char current_time[] = "current_time";
const char parameters[] = "params";
const char response[] = "return";
}  // namespace key

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

void toProto(const geometry_msgs::msg::Point & p, geometry_msgs::Point & proto);
void toProto(const geometry_msgs::msg::Quaternion & q, geometry_msgs::Quaternion & proto);
void toProto(const geometry_msgs::msg::Pose & p, geometry_msgs::Pose & proto);

template<typename T>
T getXmlValue(const XmlRpc::XmlRpcValue & xml, const std::string & key)
{
  if (!xml.hasMember(key)) {
    THROW_XML_PARAMETER_NOT_DEFINED_ERROR(key);
  }
  if (typeid(T) == typeid(double)) {
    if (xml[key].getType() == XmlRpc::XmlRpcValue::TypeDouble) {
      return xml[key];
    }
    THROW_XML_PARAMETER_ERROR("param : " + key + " is does not double type");
  }
  if (typeid(T) == typeid(bool)) {
    if (xml[key].getType() == XmlRpc::XmlRpcValue::TypeBoolean) {
      return xml[key];
    }
    THROW_XML_PARAMETER_ERROR("param : " + key + " is does not bool type");
  }
  if (typeid(T) == typeid(std::string)) {
    if (xml[key].getType() == XmlRpc::XmlRpcValue::TypeString) {
      return xml[key];
    }
    THROW_XML_PARAMETER_ERROR("param : " + key + " is does not string type");
  }
  THROW_XML_PARAMETER_ERROR("type of the param : " + key + " does not supported yet");
}

template<typename T>
const XmlRpc::XmlRpcValue serializeToBinValue(const T & data)
{
  size_t size = data.ByteSizeLong();
  void * buffer = malloc(size);
  data.SerializeToArray(buffer, size);
  const auto val = XmlRpc::XmlRpcValue(buffer, size);
  free(buffer);
  return val;
}

template<typename T>
const T deserializeFromBinValue(const XmlRpc::XmlRpcValue & data)
{
  if (data.getType() != XmlRpc::XmlRpcValue::TypeBase64) {
    THROW_XML_PARAMETER_ERROR("data is not a binary type");
  }
  std::vector<char> bin = data;
  T ret;
  ret.ParseFromArray(bin.data(), bin.size());
  return ret;
}
}  // namespace xmlrpc_interface


#endif  // XMLRPC_INTERFACE__CONVERSIONS_HPP_
