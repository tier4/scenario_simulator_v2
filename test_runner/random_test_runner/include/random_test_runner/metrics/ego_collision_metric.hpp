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
//
// Co-developed by TIER IV, Inc. and Robotec.AI sp. z o.o.

#ifndef RANDOM_TEST_RUNNER__COLLISION_METRIC_H
#define RANDOM_TEST_RUNNER__COLLISION_METRIC_H

class EgoCollisionMetric
{
public:
  bool isThereEgosCollisionWith(const std::string & npc_name, double current_time)
  {
    timeoutCollisions(current_time);

    auto [_, was_inserted] = npc_last_collision_type_map_.emplace(npc_name, current_time);
    if (was_inserted) {
      return true;
    }
    npc_last_collision_type_map_[npc_name] = current_time;
    return false;
  }

private:
  void timeoutCollisions(double current_time)
  {
    for (auto it = npc_last_collision_type_map_.begin();
         it != npc_last_collision_type_map_.end();) {
      if (current_time - it->second > collision_timeout_) {
        npc_last_collision_type_map_.erase(it++);
      } else {
        it++;
      }
    }
  }

  std::unordered_map<std::string, double> npc_last_collision_type_map_;
  const double collision_timeout_ = 0.5;
};

#endif  // RANDOM_TEST_RUNNER__COLLISION_METRIC_H
