#pragma once

#include <chrono>

#include "Timer/ISystemTimer.h"

namespace openstm::hal::stmicro::f3 {

class SystemTimer : ISystemTimer {
  std::chrono::microseconds m_PerTickTime;

 public:
  static constexpr std::uint32_t ProcessorFrequncy = 72000000;
  void Initialize(std::chrono::microseconds tickPeriod) override;

  std::uint32_t TickCount() const override;
  std::chrono::microseconds MicroSecondsPerTick() const override;

  int AttachToInterrupt(std::function<void(std::uint32_t)> f) override;
  void RemoveInterrupt(int id) override;
};
}  // namespace openstm::hal::stmicro::f3
