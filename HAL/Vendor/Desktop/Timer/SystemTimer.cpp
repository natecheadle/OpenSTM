#include "SystemTimer.h"

namespace openstm::hal::desktop {

SystemTimer::SystemTimer() {}

void SystemTimer::Initialize(std::chrono::microseconds tickPeriod) {
  m_PerTickTime = tickPeriod;
  m_TimerThread = std::jthread([this]() {
    boost::asio::chrono::microseconds waitTime{m_PerTickTime};
    boost::asio::steady_timer timer(m_Context, waitTime);
    timer.async_wait([this](const boost::system::error_code&) {
      std::uint32_t tickCount = m_TickCount++;
      m_TickOccurredEvent.Invoke(tickCount);
    });
    m_Context.run();
  });
}

std::uint32_t SystemTimer::TickCount() const { return m_TickCount; }

std::chrono::microseconds SystemTimer::MicroSecondsPerTick() const {
  return m_PerTickTime;
}

ISystemTimer::TickOccurredSub SystemTimer::AttachToInterrupt(
    std::function<void(std::uint32_t)> f) {
  return m_TickOccurredEvent.Subscribe(std::move(f));
}
}  // namespace openstm::hal::desktop
