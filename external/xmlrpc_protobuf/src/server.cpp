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

#include <xmlrpc_protobuf/server.hpp>

namespace xmlrpc_protobuf
{
Server::Server(int port)
: port(port)
{
}

Server::~Server()
{
  xmlrpc_thread_.join();
}

void Server::runXmlRpc()
{
  while (rclcpp::ok()) {
    server_.work(1);
  }
}

void Server::start()
{
  server_.bindAndListen(port);
  server_.enableIntrospection(true);
  xmlrpc_thread_ = std::thread(&Server::runXmlRpc, this);
}
}
