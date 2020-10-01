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

#ifndef SIMULATION_CONTROLLER__API__API_IMPL_BASE_HPP_
#define SIMULATION_CONTROLLER__API__API_IMPL_BASE_HPP_

#include <simulation_controller/entity/entity_manager.hpp>
#include <xmlrpcpp/XmlRpcClient.h>
#include <xmlrpcpp/XmlRpcValue.h>
#include <xmlrpcpp/XmlRpcException.h>
#include <memory>

#include <stdexcept>

namespace scenario_simulator
{
class XmlRpcRuntimeError : public std::runtime_error
{
public:
  XmlRpcRuntimeError(const char * message, int res)
  : runtime_error(message), error_info_(res) {}

private:
  int error_info_;
};

class ExecutionFailedError : public std::runtime_error
{
public:
  explicit ExecutionFailedError(XmlRpc::XmlRpcValue value)
  : runtime_error(value["message"]) {}
  explicit ExecutionFailedError(const char * message)
  : runtime_error(message) {}
};

class ApiImplBase
{
public:
  ApiImplBase(
    std::shared_ptr<XmlRpc::XmlRpcClient> client_ptr,
    std::shared_ptr<simulation_controller::entity::EntityManager> entity_manager_ptr)
  {
    client_ptr_ = client_ptr;
    entity_manager_ptr_ = entity_manager_ptr;
  }

protected:
  std::shared_ptr<XmlRpc::XmlRpcClient> client_ptr_;
  std::shared_ptr<simulation_controller::entity::EntityManager> entity_manager_ptr_;
};
}  // namespace scenario_simulator

#endif  // SIMULATION_CONTROLLER__API__API_IMPL_BASE_HPP_
