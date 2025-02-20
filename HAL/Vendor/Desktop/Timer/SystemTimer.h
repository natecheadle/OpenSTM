#pragma once

#include <boost/asio.hpp>
#include <chrono>
#include <thread>

#include "Timer/ISystemTimer.h"

namespace openstm::hal::desktop {

class SystemTimer : ISystemTimer {
  std::chrono::microseconds m_PerTickTime;
  boost::asio::io_context m_Context;
  boost::asio::steady_timer m_Timer;
  std::thread m_TimerThread;
  std::atomic<std::uint32_t> m_TickCount;

  TickOccurredEvent m_TickOccurredEvent;

 public:
  SystemTimer();

  ~SystemTimer();

  SystemTimer(const SystemTimer& other) = delete;
  SystemTimer& operator=(const SystemTimer& other) = delete;

  SystemTimer(SystemTimer&& other) noexcept;
  SystemTimer& operator=(SystemTimer&& other) noexcept = delete;

  void Initialize(std::chrono::microseconds tickPeriod) override;

  std::uint32_t TickCount() const override;
  std::chrono::microseconds MicroSecondsPerTick() const override;

  TickOccurredSub AttachToInterrupt(
      std::function<void(std::uint32_t)> f) override;

 private:
  void TimeOut(const boost ::system::error_code& ec);
};
}  // namespace openstm::hal::desktop
