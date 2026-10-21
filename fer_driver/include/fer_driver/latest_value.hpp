#ifndef FER_DRIVER_LATEST_VALUE_HPP
#define FER_DRIVER_LATEST_VALUE_HPP

#include <array>
#include <atomic>
#include <type_traits>

namespace fer {

// One producer and one consumer exchange complete records through three slots.
// Each thread owns a private slot; only the middle-slot index is shared.
template <class Value> class LatestValue {
    static_assert(std::is_trivially_copyable_v<Value>, "Realtime records must have a fixed size");
    static_assert(std::atomic<unsigned>::is_always_lock_free, "Slot exchange must be lock-free");

public:
    void Publish(const Value& value)
    {
        slots_[writeSlot_] = value;
        unsigned previousMiddle = middleSlot_.exchange(writeSlot_ | NEW_VALUE, std::memory_order_acq_rel);
        writeSlot_ = previousMiddle & SLOT_INDEX;
    }

    bool Consume(Value& value)
    {
        if ((middleSlot_.load(std::memory_order_acquire) & NEW_VALUE) == 0) {
            return false;
        }
        unsigned previousMiddle = middleSlot_.exchange(readSlot_, std::memory_order_acq_rel);
        readSlot_ = previousMiddle & SLOT_INDEX;
        value = slots_[readSlot_];
        return true;
    }

private:
    static constexpr unsigned NEW_VALUE = 4;
    static constexpr unsigned SLOT_INDEX = 3;
    std::array<Value, 3> slots_ {};
    alignas(64) std::atomic<unsigned> middleSlot_ { 1 };
    alignas(64) unsigned writeSlot_ = 2;
    alignas(64) unsigned readSlot_ = 0;
};

}

#endif
