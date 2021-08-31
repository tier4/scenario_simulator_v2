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

#include <scenario_simulator_exception/exception.hpp>
#include <simulation_interface/constants.hpp>
#include <string>

namespace simulation_interface
{
std::string getEndPoint(
  const TransportProtocol & protocol, const HostName & hostname, const unsigned int & port)
{
  return simulation_interface::enumToString(protocol) + "://" +
         simulation_interface::enumToString(hostname) + ":" + std::to_string(port);
}

std::string enumToString(const TransportProtocol & protocol)
{
  switch (protocol) {
    case TransportProtocol::TCP:
      return "tcp";
    case TransportProtocol::UDP:
      return "udp";
  }
  THROW_SIMULATION_ERROR("Protocol should be TCP or UDP.");
}

std::string enumToString(const HostName & hostname)
{
  switch (hostname) {
    case HostName::LOCLHOST:
      return "localhost";
    case HostName::ANY:
      return "*";
  }
  THROW_SIMULATION_ERROR("Hostname protocol should be LOCALHOST or ANY.");
}
}  // namespace simulation_interface
