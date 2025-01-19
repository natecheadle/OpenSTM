#pragma once

#include <type_traits>

#include "IDigitalIn.h"
#include "IO/Digital/PinID.h"

namespace openstm::hal {

template <typename T>
class DigitalIn : public IDigitalIn {
  static_assert(std::is_base_of_v<IDigitalIn, T>,
                "T must implement IDigitalIn");
  T m_ConcreteDigitalIn;

 public:
  DigitalIn(const T& digitalOut) : m_ConcreteDigitalIn(digitalOut) {}
  ~DigitalIn() = default;

  PinID ID() const override { return m_ConcreteDigitalIn.ID(); }
  void Initialize() override { m_ConcreteDigitalIn.Initialize(); }

  DigitalState GetState() const override {
    return m_ConcreteDigitalIn.GetState();
  }

  int AttachToInterrupt(std::function<void()> f) override {
    return m_ConcreteDigitalIn.AttachToInterrupt(f);
  }

  void RemoveInterrupt(int id) override {
    m_ConcreteDigitalIn.RemoveInterrupt(id);
  }
};
}  // namespace openstm::hal
