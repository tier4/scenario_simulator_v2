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

#ifndef RANDOM_TEST_RUNNER__RANDOMIZERS_H
#define RANDOM_TEST_RUNNER__RANDOMIZERS_H

#include <memory>
#include <random>
#include <type_traits>

using RandomizationEngine = std::mt19937;
using RandomizationEnginePtr = std::shared_ptr<RandomizationEngine>;

template <typename T>
class UniformRandomizer
{
public:
  UniformRandomizer(RandomizationEnginePtr engine, T range_min, T range_max)
  {
    randomization_engine_ = std::move(engine);
    randomization_distribution_ =
      std::make_unique<RandomizerDistributionType>(range_min, range_max);
  }

  void setRange(T range_min, T range_max)
  {
    randomization_distribution_ =
      std::make_unique<RandomizerDistributionType>(range_min, range_max);
  }

  T generate()
  {
    if (!randomization_engine_) {
      throw std::runtime_error("Randomizer not initialized");
    }
    return randomization_distribution_->operator()(*randomization_engine_);
  }

private:
  RandomizationEnginePtr randomization_engine_;
  typedef typename std::conditional<
    std::is_integral<T>::value, std::uniform_int_distribution<T>,
    std::uniform_real_distribution<T>>::type RandomizerDistributionType;
  std::unique_ptr<RandomizerDistributionType> randomization_distribution_;
};

using LaneletIdRandomizer = UniformRandomizer<int64_t>;
using LaneletOffsetRandomizer = UniformRandomizer<double>;
using SValueRandomizer = UniformRandomizer<double>;
using SpeedRandomizer = UniformRandomizer<double>;

#endif  // RANDOM_TEST_RUNNER__RANDOMIZERS_H
