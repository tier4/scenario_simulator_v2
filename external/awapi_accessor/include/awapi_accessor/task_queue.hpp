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

#ifndef AWAPI_ACCESSOR__TASK_QUEUE_HPP_
#define AWAPI_ACCESSOR__TASK_QUEUE_HPP_

#include <exception>
#include <functional>
#include <future>
#include <queue>
#include <thread>
#include <tuple>
#include <utility>

template <typename F, typename Tuple, std::size_t... I>
constexpr decltype(auto) apply_aux(F && f, Tuple && tuple, std::index_sequence<I...>)
{
  return std::forward<F>(f)(std::get<I>(std::forward<Tuple>(tuple))...);
}

template <typename F, typename Tuple>
constexpr decltype(auto) apply(F && f, Tuple && tuple)
{
  return apply_aux(
    std::forward<F>(f), std::forward<Tuple>(tuple),
    std::make_index_sequence<std::tuple_size<std::remove_reference_t<Tuple>>::value>());
}

namespace awapi
{
class TaskQueue
{
  using Thunk = std::function<void()>;

  std::queue<Thunk> thunks;

  std::promise<void> promise;

  std::thread worker;

  std::string what;

public:
  explicit TaskQueue()
  : worker(
      [this](auto future) {
        using namespace std::literals::chrono_literals;
        while (rclcpp::ok() and future.wait_for(1ms) == std::future_status::timeout) {
          if (not thunks.empty()) {
            std::thread(thunks.front()).join();
            thunks.pop();
          } else {
            std::this_thread::sleep_for(100ms);
          }
        }
      },
      std::move(promise.get_future()))
  {
  }

  ~TaskQueue()
  {
    std::cout << "DESTROYING TaskQueue BEGIN" << std::endl;
    if (worker.joinable()) {
      promise.set_value();
      worker.join();
    }
    std::cout << "DESTROYING TaskQueue END" << std::endl;
  }

  template <typename F>
  decltype(auto) delay(F && f)
  {
    return thunks.emplace([this, f]() {
      try {
        return f();
      } catch (const std::exception & error) {
        what = error.what();
      }
    });
  }

  auto exhausted() const noexcept { return thunks.empty(); }
};
}  // namespace awapi

#endif  // AWAPI_ACCESSOR__TASK_QUEUE_HPP_
