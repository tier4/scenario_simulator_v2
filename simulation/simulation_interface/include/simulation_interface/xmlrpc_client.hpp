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

#ifndef SIMULATION_INTERFACE__XMLRPC_CLIENT_HPP_
#define SIMULATION_INTERFACE__XMLRPC_CLIENT_HPP_

#include <xmlrpcpp/XmlRpc.h>
#include <simulation_interface/conversions.hpp>

#include <string>
#include <memory>

namespace simulation_interface
{
class XmlRpcRuntimeError : public std::runtime_error
{
public:
  XmlRpcRuntimeError(const char * message, int result)
  : runtime_error(message), error_info_(result) {}

  virtual ~XmlRpcRuntimeError() = default;

private:
  int error_info_;
};

template<typename ReqType, typename ResType>
bool call(
  const std::shared_ptr<XmlRpc::XmlRpcClient> & client_ptr, const std::string & method_name,
  const ReqType & req, ResType & res)
{
  // XmlRpc::setVerbosity(5);
  XmlRpc::XmlRpcValue result, value;
  try {
    value = simulation_interface::serializeToBinValue<ReqType>(
      req);
  } catch (const XmlParameterError & e) {
    std::string message = "error found while calling " + method_name + " method.\n" + e.what();
    THROW_XML_PARAMETER_ERROR(message);
  }
  try {
    client_ptr->execute(method_name.c_str(), value, result);
  } catch (XmlRpc::XmlRpcException e) {
    throw XmlRpcRuntimeError(e.getMessage().c_str(), e.getCode());
  }
  res =
    simulation_interface::deserializeFromBinValue<ResType>(
    result);
  return res.result().success();
}
}  // namespace simulation_interface

#endif  // SIMULATION_INTERFACE__XMLRPC_CLIENT_HPP_
