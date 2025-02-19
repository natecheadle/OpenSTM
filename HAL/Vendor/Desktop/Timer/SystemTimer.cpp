#include "SystemTimer.h"

namespace openstm::hal::desktop {

SystemTimer::SystemTimer()
    : m_PerTickTime(std::chrono::seconds{1}), m_Timer(m_Context) {}

SystemTimer::~SystemTimer() { m_Context.stop(); }

void SystemTimer::Initialize(std::chrono::microseconds tickPeriod) {
  m_PerTickTime = tickPeriod;
  m_Timer = boost::asio::steady_timer(m_Context, m_PerTickTime);
  m_TimerThread = std::jthread([this]() {
    m_Timer.async_wait(
        [this](const boost::system::error_code& ec) { TimeOut(ec); });
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

void SystemTimer::TimeOut(const boost::system::error_code& ec) {
  if (ec) {
    return;
  }
  std::uint32_t tickCount = m_TickCount++;
  m_TickOccurredEvent.Invoke(tickCount);

  m_Timer.async_wait(
      [this](const boost::system::error_code& ec) { TimeOut(ec); });
}
}  // namespace openstm::hal::desktop
