#pragma once
#include <Event/Event.hpp>
#include <chrono>
#include <cstdint>
#include <functional>
#include <limits>

#ifndef MAX_CALLBACKS_PER_TICK
#define MAX_CALLBACKS_PER_TICK 1
#endif
namespace openstm::hal {

class ISystemTimer {
 public:
  using TickOccurredEvent = lib::Event<MAX_CALLBACKS_PER_TICK, std::uint32_t>;
  using TickOccurredSub = TickOccurredEvent::Subscription;

  virtual void Initialize(std::chrono::microseconds tickPeriod) = 0;

  virtual std::uint32_t TickCount() const = 0;
  virtual std::chrono::microseconds MicroSecondsPerTick() const = 0;

  virtual TickOccurredSub AttachToInterrupt(
      std::function<void(std::uint32_t)> f) = 0;

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
