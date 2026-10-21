#include <atomic>
#include <thread>
#include <gtest/gtest.h>
#include "fer_driver/latest_value.hpp"

TEST(LatestValue, ReturnsOnlyNewValues) {
  fer::LatestValue<int> mailbox;
  int value = -1;
  EXPECT_FALSE(mailbox.consume(value));
  mailbox.publish(1); mailbox.publish(2);
  EXPECT_TRUE(mailbox.consume(value)); EXPECT_EQ(value, 2);
  EXPECT_FALSE(mailbox.consume(value));
}

TEST(LatestValue, ConcurrentSnapshotsAreCompleteAndOrdered) {
  struct Value { uint64_t sequence, check[32]; };
  fer::LatestValue<Value> mailbox;
  std::atomic<bool> done{false};
  std::thread writer([&] {
    for (uint64_t i = 1; i <= 100000; ++i) {
      Value value{}; value.sequence = i;
      for (auto& item : value.check) item = i;
      mailbox.publish(value);
    }
    done.store(true);
  });
  Value value{};
  uint64_t previous = 0;
  while (!done.load() || previous < 100000) {
    if (mailbox.consume(value)) {
      EXPECT_GT(value.sequence, previous);
      for (auto item : value.check) EXPECT_EQ(item, value.sequence);
      previous = value.sequence;
    }
  }
  writer.join();
}
