#pragma once

#include <array>
#include <atomic>
#include <type_traits>

namespace fer {
// Single producer, single consumer triple buffer. Neither side waits or allocates.
// Producer owns back_, consumer owns front_, atomic middle_ transfers ownership.
template<class T>
class LatestValue {
  static_assert(std::is_trivially_copyable_v<T>, "Mailbox values must be fixed-size records");
  static_assert(std::atomic<unsigned>::is_always_lock_free, "Mailbox index must be lock-free");
 public:
  void publish(const T& value) noexcept {
    slots_[back_] = value;
    back_ = middle_.exchange(back_ | kDirty, std::memory_order_acq_rel) & kIndex;
  }
  bool consume(T& value) noexcept {
    if (!(middle_.load(std::memory_order_acquire) & kDirty)) return false;
    front_ = middle_.exchange(front_, std::memory_order_acq_rel) & kIndex;
    value = slots_[front_];
    return true;
  }
 private:
  static constexpr unsigned kDirty = 4, kIndex = 3;
  std::array<T, 3> slots_{};
  alignas(64) std::atomic<unsigned> middle_{1};
  alignas(64) unsigned back_{2};
  alignas(64) unsigned front_{0};
};
}  // namespace fer
