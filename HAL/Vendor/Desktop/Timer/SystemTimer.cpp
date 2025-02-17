#include "SystemTimer.h"

namespace openstm::hal::desktop {
    
SystemTimer::SystemTimer()
    : m_Strand(boost::asio:g)

  SystemTimer::~SystemTimer();

  SystemTimer(const SystemTimer& other) = delete;
  SystemTimer& operator=(const SystemTimer& other) = delete;

  SystemTimer::SystemTimer(SystemTimer&& other) noexcept;
  SystemTimer& operator=(SystemTimer&& other) noexcept = delete;

  static constexpr std::uint32_t ProcessorFrequncy = 72000000;
  void SystemTimer::Initialize(std::chrono::microseconds tickPeriod) override;

  std::uint32_t SystemTimer::TickCount() const override;
  std::chrono::microseconds SystemTimer::MicroSecondsPerTick() const override;

  TickOccurredSub SystemTimer::AttachToInterrupt(
      std::function<void(std::uint32_t)> f) override;

  void SystemTimer::TickOccurred(std::uint32_t count);
};
}  // namespace openstm::hal::desktop
