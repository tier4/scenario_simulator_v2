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

#ifndef TRAFFIC_SIMULATOR__DATA_TYPE__OTHER_ENTITY_STATUSES_HPP_
#define TRAFFIC_SIMULATOR__DATA_TYPE__OTHER_ENTITY_STATUSES_HPP_

#include <boost/iterator/filter_iterator.hpp>
#include <memory>
#include <stdexcept>
#include <string>
#include <traffic_simulator/data_type/entity_status.hpp>
#include <unordered_map>
#include <utility>

namespace traffic_simulator
{
/// A read-only view sharing ownership of an immutable entity status snapshot.
class OtherEntityStatuses
{
  using Map = std::unordered_map<std::string, CanonicalizedEntityStatus>;

  struct ExcludeSelf
  {
    const Map::value_type * self{nullptr};

    auto operator()(const Map::value_type & entry) const -> bool { return &entry != self; }
  };

public:
  using value_type = Map::value_type;
  using const_iterator = boost::filter_iterator<ExcludeSelf, Map::const_iterator>;

  OtherEntityStatuses() = default;

  explicit OtherEntityStatuses(Map snapshot)
  : snapshot_(std::make_shared<const Map>(std::move(snapshot)))
  {
  }

  auto excluding(const std::string & self_name) const -> OtherEntityStatuses
  {
    auto view = *this;
    const auto it = map().find(self_name);
    view.self_ = it == map().end() ? nullptr : &*it;
    return view;
  }

  auto begin() const -> const_iterator { return {ExcludeSelf{self_}, map().begin(), map().end()}; }

  auto end() const -> const_iterator { return {ExcludeSelf{self_}, map().end(), map().end()}; }

  auto find(const std::string & name) const -> const_iterator
  {
    return {ExcludeSelf{self_}, isSelf(name) ? map().end() : map().find(name), map().end()};
  }

  auto at(const std::string & name) const -> const CanonicalizedEntityStatus &
  {
    if (isSelf(name)) {
      throw std::out_of_range("OtherEntityStatuses::at: self is excluded");
    }
    return map().at(name);
  }

  auto count(const std::string & name) const -> std::size_t
  {
    return isSelf(name) ? 0 : map().count(name);
  }

  auto size() const -> std::size_t { return map().size() - (snapshot_ and self_ ? 1 : 0); }

  auto empty() const -> bool { return size() == 0; }

  auto toMap() const -> Map { return Map(begin(), end()); }

private:
  auto isSelf(const std::string & name) const -> bool
  {
    return snapshot_ and self_ and name == self_->first;
  }

  auto map() const -> const Map &
  {
    static const Map empty;
    return snapshot_ ? *snapshot_ : empty;
  }

  std::shared_ptr<const Map> snapshot_;
  // Points into snapshot_.
  const Map::value_type * self_{nullptr};
};
}  // namespace traffic_simulator

#endif  // TRAFFIC_SIMULATOR__DATA_TYPE__OTHER_ENTITY_STATUSES_HPP_
