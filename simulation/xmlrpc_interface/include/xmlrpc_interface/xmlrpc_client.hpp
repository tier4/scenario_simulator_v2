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

#ifndef XMLRPC_INTERFACE__XMLRPC_CLIENT_HPP_
#define XMLRPC_INTERFACE__XMLRPC_CLIENT_HPP_

#include <xmlrpcpp/XmlRpc.h>
#include <xmlrpc_interface/conversions.hpp>

#include <string>
#include <memory>

namespace xmlrpc_interface
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
  XmlRpc::XmlRpcValue result, value;
  value[0][0][xmlrpc_interface::key::method_name] = method_name;
  value[0][0][xmlrpc_interface::key::parameters] = xmlrpc_interface::serializeToBinValue<ReqType>(
    req);
  try {
    client_ptr->execute("system.multicall", value, result);
  } catch (XmlRpc::XmlRpcException e) {
    throw XmlRpcRuntimeError(e.getMessage().c_str(), e.getCode());
  }
  res = ResType();
  res =
    xmlrpc_interface::deserializeFromBinValue<ResType>(
    result[0][0][xmlrpc_interface::key::
    response]);
  return res.result().success();
}
}  // namespace xmlrpc_interface

#endif  // XMLRPC_INTERFACE__XMLRPC_CLIENT_HPP_
