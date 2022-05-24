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

#ifndef CONCEALER__TASK_QUEUE_HPP_
#define CONCEALER__TASK_QUEUE_HPP_

#include <exception>
#include <functional>
#include <future>
#include <mutex>
#include <queue>
#include <thread>

namespace concealer
{
class TaskQueue
{
  using Thunk = std::function<void()>;

  std::queue<Thunk> thunks;

  std::mutex mtx;

  std::promise<void> notifier;

  std::thread dispatcher;

  std::exception_ptr thrown;

public:
  explicit TaskQueue();

  ~TaskQueue();

  template <typename F>
  decltype(auto) delay(F && f)
  {
    std::unique_lock lk(mtx);
    return thunks.emplace([this, f = std::forward<F>(f)]() {
      try {
        return f();
      } catch (...) {
        thrown = std::current_exception();
      }
    });
  }

  bool exhausted() const noexcept;

  void rethrow() const noexcept(false);
};
}  // namespace concealer

#endif  // CONCEALER__TASK_QUEUE_HPP_
