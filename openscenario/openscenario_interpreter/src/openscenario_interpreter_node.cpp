// Copyright 2015 TIER IV, Inc. All rights reserved.
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

#include <glog/logging.h>

#include <chrono>
#include <cstdlib>
#include <iostream>
#include <memory>
#include <openscenario_interpreter/openscenario_interpreter.hpp>
#include <status_monitor/status_monitor.hpp>

int main(const int argc, char const * const * const argv)
{
  google::InitGoogleLogging(argv[0]);
  google::InstallFailureSignalHandler();

  rclcpp::init(argc, argv);

  rclcpp::executors::SingleThreadedExecutor executor;

  auto node = std::make_shared<openscenario_interpreter::Interpreter>(rclcpp::NodeOptions());

  executor.add_node(node->get_node_base_interface());

  auto last_touch_time = std::chrono::high_resolution_clock::now();
  
  while (rclcpp::ok()) {
    auto now = std::chrono::high_resolution_clock::now();
    auto since_last_touch = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_touch_time).count();
    
    if (since_last_touch > 1000) {  // Log if more than 1 second since last iteration
      std::cout << "[MainLoop] Long iteration gap: " << since_last_touch << "ms" << std::endl;
    }
    
    common::status_monitor.touch(__func__);
    last_touch_time = now;
    
    auto spin_start = std::chrono::high_resolution_clock::now();
    executor.spin_once();
    auto spin_end = std::chrono::high_resolution_clock::now();
    
    auto spin_duration = std::chrono::duration_cast<std::chrono::milliseconds>(spin_end - spin_start).count();
    if (spin_duration > 1000) {  // Log if spin_once takes more than 1 second
      std::cout << "[MainLoop] Long spin_once duration: " << spin_duration << "ms" << std::endl;
    }
  }
  
  std::cout << "[MainLoop] Exiting main loop - rclcpp::ok() returned false" << std::endl;

  rclcpp::shutdown();

  return 0;
}
