/** @file latest_value.hpp */

#ifndef FER_DRIVER_LATEST_VALUE_HPP
#define FER_DRIVER_LATEST_VALUE_HPP

#include <array>
#include <atomic>
#include <type_traits>

namespace fer {

/**
 * @brief Lock-free exchange of the latest complete record using three storage slots.
 * @tparam Value Fixed-size, trivially copyable record.
 * @details Exactly one producer calls Publish and one consumer calls Consume. Each owns
 * a private slot and exchanges the middle slot atomically. Unread records may be replaced.
 */
template <class Value> class LatestValue {
    static_assert(std::is_trivially_copyable_v<Value>, "Realtime records must have a fixed size");
    static_assert(std::atomic<unsigned>::is_always_lock_free, "Slot exchange must be lock-free");

public:
    /**
     * @brief Replace the available record without blocking the producer.
     * @param value Complete record to copy into the producer-owned slot.
     * @note Call only from the single producer thread.
     */
    void Publish(const Value& value)
    {
        slots_[writeSlot_] = value;
        const unsigned previousMiddle
            = middleSlot_.exchange(writeSlot_ | NEW_VALUE_FLAG, std::memory_order_acq_rel);
        writeSlot_ = previousMiddle & SLOT_INDEX_MASK;
    }

    /**
     * @brief Copy the latest available record without blocking the consumer.
     * @param[out] value Updated only when a new record is available.
     * @return True when a record was copied; false leaves value unchanged.
     * @note Call only from the single consumer thread.
     */
    bool Consume(Value& value)
    {
        if ((middleSlot_.load(std::memory_order_acquire) & NEW_VALUE_FLAG) == 0) {
            return false;
        }
        const unsigned previousMiddle = middleSlot_.exchange(readSlot_, std::memory_order_acq_rel);
        readSlot_ = previousMiddle & SLOT_INDEX_MASK;
        value = slots_[readSlot_];
        return true;
    }

private:
    // The low two bits select a slot; the next bit records an unread publication.
    static constexpr unsigned NEW_VALUE_FLAG = 4;
    static constexpr unsigned SLOT_INDEX_MASK = 3;
    std::array<Value, 3> slots_ {};
    // Keep independently written indices on separate cache lines.
    alignas(64) std::atomic<unsigned> middleSlot_ { 1 };
    alignas(64) unsigned writeSlot_ = 2;
    alignas(64) unsigned readSlot_ = 0;
};

}

#endif
