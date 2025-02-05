#include <gtest/gtest.h>

#include <Event/Event.hpp>
#include <vector>

namespace openstm::lib::test {
class EventFixture : public testing::Test {
 public:
  std::unique_ptr<Event<4, int, int>> Event1{
      std::make_unique<Event<4, int, int>>()};
};

TEST_F(EventFixture, ValidateSubscribe) {
  auto sub = Event1->Subscribe(nullptr);
  ASSERT_TRUE(sub.IsSubscribed());
}

TEST_F(EventFixture, ValidateSubscribeCallback) {
  int val1{0};
  int val2{0};

  auto sub = Event1->Subscribe([&](int first, int second) {
    val1 = first;
    val2 = second;
  });

  Event1->Invoke(1, 2);

  ASSERT_EQ(val1, 1);
  ASSERT_EQ(val2, 2);
}

TEST_F(EventFixture, ValidateSubscribeUsubscribe) {
  auto sub = Event1->Subscribe(nullptr);
  sub.Reset();
}

TEST_F(EventFixture, ValidateDestroyEvent) {
  auto sub = Event1->Subscribe(nullptr);
  Event1.reset();
}

TEST_F(EventFixture, ValidateMoveEvent) {
  auto sub = Event1->Subscribe(nullptr);

  auto sub2 = std::move(sub);

  ASSERT_TRUE(sub2.IsSubscribed());
  ASSERT_FALSE(sub.IsSubscribed());
}
TEST_F(EventFixture, ValidateOversubscribe) {
  std::array<Event<4, int, int>::Subscription, 5> subscribers;
  for (auto& sub : subscribers) {
    sub = Event1->Subscribe(nullptr);
  }

  for (size_t i = 0; i < subscribers.size(); ++i) {
    if (i < 4) {
      ASSERT_TRUE(subscribers[i].IsSubscribed());
    } else {
      ASSERT_FALSE(subscribers[i].IsSubscribed());
    }
  }
}
}  // namespace openstm::lib::test
