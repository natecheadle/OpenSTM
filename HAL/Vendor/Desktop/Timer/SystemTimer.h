#pragma once

#include <boost/asio.hpp>
#include <chrono>
#include <thread>

#include "Timer/ISystemTimer.h"

namespace openstm::hal::desktop {

class SystemTimer : ISystemTimer {
  std::chrono::microseconds m_PerTickTime;
  boost::asio::io_context m_Context;
  boost::asio::strand<boost::asio::io_context::executor_type> m_Strand;
  std::jthread m_TimerThread;

  TickOccurredEvent m_TickOccurredEvent;

 public:
  SystemTimer();

  ~SystemTimer();

  SystemTimer(const SystemTimer& other) = delete;
  SystemTimer& operator=(const SystemTimer& other) = delete;

  SystemTimer(SystemTimer&& other) noexcept;
  SystemTimer& operator=(SystemTimer&& other) noexcept = delete;

  static constexpr std::uint32_t ProcessorFrequncy = 72000000;
  void Initialize(std::chrono::microseconds tickPeriod) override;

  std::uint32_t TickCount() const override;
  std::chrono::microseconds MicroSecondsPerTick() const override;

  TickOccurredSub AttachToInterrupt(
      std::function<void(std::uint32_t)> f) override;

  void TickOccurred(std::uint32_t count);
};
}  // namespace openstm::hal::desktop
