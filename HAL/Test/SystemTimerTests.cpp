#include <Vendor/Desktop/Timer/SystemTimer.h>
#include <thread>
#include <chrono>
#include <gtest/gtest.h>

namespace openstm::hal::desktop::test {
class SystemTimerFixture : public testing::Test {
 public:
  static constexpr std::chrono::milliseconds TICK_PERIOD{10};
  SystemTimer Timer;

  SystemTimerFixture() { Timer.Initialize(TICK_PERIOD); }
};

TEST_F(SystemTimerFixture, ValidateInitialState) {
  ASSERT_EQ(Timer.MicroSecondsPerTick(), TICK_PERIOD);
}

TEST_F(SystemTimerFixture, ValidateGetState) {
  std::uint32_t ticks{0};
  std::uint32_t expectedTicks{100};
  auto now = std::chrono::steady_clock::now();
  auto sub = Timer.AttachToInterrupt([&](std::uint32_t) { ticks++; });
  std::this_thread::sleep_until(
      now + std::chrono::milliseconds(TICK_PERIOD.count() * expectedTicks));

  ASSERT_EQ(ticks, 100);
}

}  // namespace openstm::hal::desktop::test
