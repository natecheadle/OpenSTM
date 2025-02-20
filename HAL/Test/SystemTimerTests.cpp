#include <Vendor/Desktop/Timer/SystemTimer.h>
#include <gtest/gtest.h>

#include <chrono>

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

TEST_F(SystemTimerFixture, ValidateTickFrequency) {
  std::uint32_t ticks{0};
  std::uint32_t expectedTicks{10};
  auto now = std::chrono::steady_clock::now();
  auto end = std::chrono::steady_clock::now();

  std::atomic<bool> completionFlag{false};
  auto sub = Timer.AttachToInterrupt([&](std::uint32_t) {
    ticks++;
    if (ticks >= expectedTicks) {
      end = std::chrono::steady_clock::now();
      completionFlag = true;
    }
  });
  while (!completionFlag) {
    std::this_thread::yield();
  }

  std::chrono::milliseconds totalTime{
      std::chrono::duration_cast<std::chrono::milliseconds>(end - now)};
  auto expectedTime = TICK_PERIOD * expectedTicks;
  std::chrono::milliseconds tolerance{TICK_PERIOD / 5};
  ASSERT_GE(totalTime, expectedTime - tolerance);
  ASSERT_LE(totalTime, expectedTime + tolerance);
}

}  // namespace openstm::hal::desktop::test
