#pragma once

#include <chrono>
#include <cstdint>
#include <type_traits>

#include "Driver.hpp"
#include "ISystemTimer.h"

namespace openstm::hal {

template <typename T>
class SystemTimer : public ISystemTimer, public Driver<T> {
  static_assert(std::is_base_of_v<ISystemTimer, T>,
                "T must implement ISystemTimer");

 public:
  SystemTimer(T&& timer) : Driver<T>(std::forward<T>(timer)) {}
  ~SystemTimer() = default;

  void Initialize(std::chrono::microseconds tickPeriod) override {
    Driver<T>::Device().Initialize(tickPeriod);
  }

  std::uint32_t TickCount() const override {
    return Driver<T>::Device().TickCount();
  }

  std::chrono::microseconds MicroSecondsPerTick() const override {
    return Driver<T>::Device().MicroSecondsPerTick();
  }

  TickOccurredSub AttachToInterrupt(
      std::function<void(std::uint32_t)> f) override {
    return AttachToInterrupt(std::move(f));
  }
};
}  // namespace openstm::hal
