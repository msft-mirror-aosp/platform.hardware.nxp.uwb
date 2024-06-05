/*
 * Copyright (C) 2024 Google, Inc.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 */

#ifndef PH_MESSAGEQUEUE_H
#define PH_MESSAGEQUEUE_H

#include <condition_variable>
#include <memory>
#include <mutex>
#include <queue>
#include <string>

#include "phNxpLog.h"

template <typename T>
class MessageQueue
{
public:
  MessageQueue(const std::string name) : name_(name) {
  }
  virtual ~MessageQueue() {
    clear();
    NXPLOG_TML_D("MessageQueue %s destroyed", name_.c_str());
  }
  void send(std::shared_ptr<T> data) {
    std::lock_guard<std::mutex> lk(lock_);
    items_.push(data);
    cond_.notify_one();
  }
  std::shared_ptr<T> recv() {
    std::unique_lock<std::mutex> lk(lock_);
    cond_.wait(lk, [this] { return !items_.empty(); });
    auto ret = items_.front();
    items_.pop();
    return ret;
  }
  void clear() {
    cond_.notify_all();
    std::lock_guard<std::mutex> lk(lock_);
    items_ = std::queue<std::shared_ptr<T>> ();
  }
private:
  std::string name_;
  std::queue<std::shared_ptr<T>> items_;
  std::mutex lock_;
  std::condition_variable cond_;
};

#endif  // _NXP_UWB_MESSAGEQUEUE_H
