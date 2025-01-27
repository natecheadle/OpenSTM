#pragma once
#include <type_traits>

#include "Driver.hpp"
#include "IDigitalIn.h"
#include "PinID.h"

namespace openstm::hal {

template <typename T>
class DigitalIn : public IDigitalIn, public Driver<T> {
  static_assert(std::is_base_of_v<IDigitalIn, T>,
                "T must implement IDigitalIn");

 public:
  DigitalIn(const T& digitalOut) : Driver<T>(digitalOut) {}
  ~DigitalIn() = default;

  PinID ID() const override { return Driver<T>::Device().ID(); }
  void Initialize() override { Driver<T>::Device().Initialize(); }

  DigitalState GetState() const override {
    return Driver<T>::Device().GetState();
  }

  int AttachToInterrupt(std::function<void()> f) override {
    return Driver<T>::Device().AttachToInterrupt(f);
  }

  void RemoveInterrupt(int id) override {
    Driver<T>::Device().RemoveInterrupt(id);
  }
};
}  // namespace openstm::hal
