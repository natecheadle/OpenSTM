#pragma once

#include <type_traits>

#include "IDigitalOut.h"
#include "IO/Digital/PinID.h"

namespace openstm::hal {

template <typename T>
class DigitalOut : public IDigitalOut {
  static_assert(std::is_base_of_v<IDigitalOut, T>,
                "T must implement IDigitalOut");
  T m_ConcreteDigitalOut;

 public:
  DigitalOut(const T& digitalOut) : m_ConcreteDigitalOut(digitalOut) {}
  ~DigitalOut() = default;

  PinID ID() const override { return m_ConcreteDigitalOut.ID(); }

  void Toggle() override { m_ConcreteDigitalOut.Toggle(); }

  DigitalState State() const override { return m_ConcreteDigitalOut.State(); }
  void State(DigitalState state) override { m_ConcreteDigitalOut.State(state); }
};
}  // namespace openstm::hal
