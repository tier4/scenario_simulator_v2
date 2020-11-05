// Copyright 2015-2020 Autoware Foundation. All rights reserved.
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

#include <scenario_simulator/xmlrpc_method.hpp>
#include <rclcpp/rclcpp.hpp>
#include <scenario_simulator/constants.hpp>

#include <string>

namespace scenario_simulator
{
XmlRpcMethod::XmlRpcMethod(std::string const & name, XmlRpc::XmlRpcServer * server)
: XmlRpc::XmlRpcServerMethod(name, server)
{
}

void XmlRpcMethod::setFunction(
  std::function<void(XmlRpc::XmlRpcValue &,
  XmlRpc::XmlRpcValue &)> func)
{
  func_ = func;
}

void XmlRpcMethod::execute(XmlRpc::XmlRpcValue & params, XmlRpc::XmlRpcValue & result)
{
  if (func_) {
    auto func = func_.get();
    func(params, result);
    result["result/return_code"] = return_code::SUCCESS;
    result["result/description"] = "function " + _name + " executed";
  } else {
    XmlRpc::XmlRpcValue error_msg;
    error_msg["result/return_code"] = return_code::FAIL;
    error_msg["result/description"] = "function does not set";
  }
}
}  // namespace scenario_simulator
