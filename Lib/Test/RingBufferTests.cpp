#include <gtest/gtest.h>

#include <Container/RingBuffer.hpp>
#include <cstdint>

namespace openstm::lib::test {
class RingBufferFixture : public testing::Test {
 public:
  static constexpr size_t BUFFER_SIZE = 32;
  RingBuffer<std::uint8_t> Buffer{BUFFER_SIZE};
};

TEST_F(RingBufferFixture, ValidateInitialState) {
  ASSERT_EQ(BUFFER_SIZE - 1, Buffer.RemainingSlots());
  ASSERT_TRUE(Buffer.IsEmpty());
  ASSERT_FALSE(Buffer.IsFull());
}

TEST_F(RingBufferFixture, ValidatePushPop) {
  const std::uint8_t newVal{8};
  ASSERT_TRUE(Buffer.Push(newVal));
  ASSERT_FALSE(Buffer.IsEmpty());
  ASSERT_FALSE(Buffer.IsFull());
  std::uint8_t popVal{0};
  ASSERT_TRUE(Buffer.Pop(popVal));
  ASSERT_EQ(newVal, popVal);
}

TEST_F(RingBufferFixture, ValidatePushFullPopEmpty) {
  for (size_t i = 0; i < BUFFER_SIZE - 1; ++i) {
    ASSERT_TRUE(Buffer.Push(static_cast<std::uint8_t>(i)));
  }
  ASSERT_TRUE(Buffer.IsFull());

  for (size_t i = 0; i < BUFFER_SIZE - 1; ++i) {
    std::uint8_t popVal{0};
    ASSERT_TRUE(Buffer.Pop(popVal));
    ASSERT_EQ(popVal, i);
  }
  ASSERT_TRUE(Buffer.IsEmpty());
}

TEST_F(RingBufferFixture, ValidatePushPopMultipleCycles) {
  std::uint8_t i;
  for (size_t i = 0; i < std::numeric_limits<std::uint8_t>::max(); ++i) {
    ASSERT_TRUE(Buffer.Push(i));
    std::uint8_t popVal{0};
    ASSERT_TRUE(Buffer.Pop(popVal));
    ASSERT_EQ(i, popVal);
  }
}

TEST_F(RingBufferFixture, ValidatePushManyPopMany) {
  std::uint8_t i;
  for (size_t j = 0; j < 25; ++j) {
    for (size_t i = 0; i < 5; ++i) {
      ASSERT_TRUE(Buffer.Push(i));
    }

    for (size_t i = 0; i < 5; ++i) {
      std::uint8_t popVal{0};
      ASSERT_TRUE(Buffer.Pop(popVal));
      ASSERT_EQ(i, popVal);
    }
  }
}

TEST_F(RingBufferFixture, ValidateBufferedCountRemainingSlots) {
  std::uint8_t i;
  for (size_t j = 0; j < 25; ++j) {
    for (size_t i = 0; i < 5; ++i) {
      ASSERT_TRUE(Buffer.Push(i));
    }

    ASSERT_EQ(5, Buffer.BufferedCount());
    ASSERT_EQ(32 - 5 - 1, Buffer.RemainingSlots());

    for (size_t i = 0; i < 5; ++i) {
      std::uint8_t popVal{0};
      ASSERT_TRUE(Buffer.Pop(popVal));
      ASSERT_EQ(i, popVal);
    }

    ASSERT_EQ(32 - 1, Buffer.RemainingSlots());
    ASSERT_EQ(0, Buffer.BufferedCount());
  }
}

TEST_F(RingBufferFixture, ValidateNextLast) {
  ASSERT_TRUE(Buffer.Push(5));

  std::uint8_t nextVal{0};
  std::uint8_t lastVal{0};
  ASSERT_TRUE(Buffer.Next(nextVal));
  ASSERT_TRUE(Buffer.Last(lastVal));
  ASSERT_EQ(5, lastVal);
  ASSERT_EQ(5, nextVal);

  ASSERT_TRUE(Buffer.Push(10));
  ASSERT_TRUE(Buffer.Next(nextVal));
  ASSERT_TRUE(Buffer.Last(lastVal));
  ASSERT_EQ(5, nextVal);
  ASSERT_EQ(10, lastVal);
}
}  // namespace openstm::lib::test
