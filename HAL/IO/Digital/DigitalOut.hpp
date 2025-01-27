#pragma once

#include <type_traits>

#include "Driver.hpp"
#include "IDigitalOut.h"
#include "PinID.h"

namespace openstm::hal {

template <typename T>
class DigitalOut : public IDigitalOut, public Driver<T> {
  static_assert(std::is_base_of_v<IDigitalOut, T>,
                "T must implement IDigitalOut");

 public:
  DigitalOut(const T& digitalOut) : Driver<T>(digitalOut) {}
  ~DigitalOut() = default;

  PinID ID() const override { return Driver<T>::Device().ID(); }

  void Toggle() override { Driver<T>::Device().Toggle(); }
  void Initialize() override { Driver<T>::Device().Initialize(); }

  DigitalState GetState() const override {
    return Driver<T>::Device().GetState();
  }

  void SetState(DigitalState state) override {
    Driver<T>::Device().SetState(state);
  }
};
}  // namespace openstm::hal
