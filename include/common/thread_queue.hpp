// traffic_monitor/include/common/thread_queue.hpp
#pragma once

#include <condition_variable>
#include <deque>
#include <mutex>
#include <optional>

namespace traffic {

enum class DropPolicy {
  OldestDrop, // 가득 차면 가장 오래된 것을 버리고 새것을 넣음(최신 유지)
  NewestDrop  // 가득 차면 새로 들어온 것을 버림(지연 최소화보단 손실 최소화)
};

// 멀티스레드 bounded queue.
// - push는 policy에 따라 drop 가능
// - pop은 blocking / non-blocking 제공
template <typename T>
class ThreadQueue {
public:
  explicit ThreadQueue(size_t capacity = 1, DropPolicy policy = DropPolicy::OldestDrop)
      : capacity_(capacity), policy_(policy) {}

  ThreadQueue(const ThreadQueue&) = delete;
  ThreadQueue& operator=(const ThreadQueue&) = delete;

  void setCapacity(size_t c) {
    std::lock_guard<std::mutex> lk(m_);
    capacity_ = (c == 0) ? 1 : c;
    // capacity 축소 시 초과분 정리
    while (q_.size() > capacity_) q_.pop_front();
  }

  void setDropPolicy(DropPolicy p) {
    std::lock_guard<std::mutex> lk(m_);
    policy_ = p;
  }

  // push: 성공하면 true, drop으로 인해 실제 저장 못했으면 false
  bool push(T&& item) {
    std::unique_lock<std::mutex> lk(m_);
    if (closed_) return false;

    if (q_.size() >= capacity_) {
      if (policy_ == DropPolicy::OldestDrop) {
        q_.pop_front();
      } else { // NewestDrop
        return false;
      }
    }
    q_.emplace_back(std::move(item));
    lk.unlock();
    cv_.notify_one();
    return true;
  }

  bool push(const T& item) {
    T copy = item;
    return push(std::move(copy));
  }

  // blocking pop: queue가 비어있으면 대기
  // close() 이후에는 더 이상 대기하지 않고 false 반환 가능
  bool popBlocking(T& out) {
    std::unique_lock<std::mutex> lk(m_);
    cv_.wait(lk, [&] { return closed_ || !q_.empty(); });
    if (q_.empty()) return false;
    out = std::move(q_.front());
    q_.pop_front();
    return true;
  }

  // non-blocking pop
  bool tryPop(T& out) {
    std::lock_guard<std::mutex> lk(m_);
    if (q_.empty()) return false;
    out = std::move(q_.front());
    q_.pop_front();
    return true;
  }

  // 가장 최신 아이템만 가져오고 나머지는 버리는 pop(화면/UI에 유용)
  bool popLatest(T& out) {
    std::lock_guard<std::mutex> lk(m_);
    if (q_.empty()) return false;
    out = std::move(q_.back());
    q_.clear();
    return true;
  }

  void close() {
    std::lock_guard<std::mutex> lk(m_);
    closed_ = true;
    cv_.notify_all();
  }

  bool closed() const {
    std::lock_guard<std::mutex> lk(m_);
    return closed_;
  }

  size_t size() const {
    std::lock_guard<std::mutex> lk(m_);
    return q_.size();
  }

private:
  mutable std::mutex m_;
  std::condition_variable cv_;
  std::deque<T> q_;

  size_t capacity_{1};
  DropPolicy policy_{DropPolicy::OldestDrop};
  bool closed_{false};
};

} // namespace traffic