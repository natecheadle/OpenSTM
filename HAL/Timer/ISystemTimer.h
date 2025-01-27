#pragma once

#include <chrono>
#include <cstdint>
#include <functional>
#include <limits>

namespace openstm::hal {

class ISystemTimer {
 public:
  virtual void Initialize(std::chrono::microseconds tickPeriod) = 0;

  virtual std::uint32_t TickCount() const = 0;
  virtual std::chrono::microseconds MicroSecondsPerTick() const = 0;

  virtual int AttachToInterrupt(std::function<void(std::uint32_t)> f) = 0;
  virtual void RemoveInterrupt(int id) = 0;

  static inline std::uint32_t CalculateTickDelta(std::uint32_t start,
                                                 std::uint32_t end) {
    if (end < start) {
      return end + (std::numeric_limits<std::uint32_t>::max() - start);
    } else {
      return end - start;
    }
  }
};
}  // namespace openstm::hal
