#pragma once

#include <Event/Event.hpp>

#include "IO/Digital/IDigitalIn.h"
#include "PinID.h"
#include "stm32f303xe.h"

namespace openstm::hal::stmicro::f3 {

class DigitalIn : public IDigitalIn {
  const GPIO_TypeDef* const m_GPIOx;
  const PinID m_ID;

  StateChangedEvent m_StateChangedEvent;

 public:
  DigitalIn(PinID id, GPIO_TypeDef* gpiox);

  DigitalIn(const DigitalIn& other) = delete;
  DigitalIn& operator=(const DigitalIn& other) = delete;

  DigitalIn(DigitalIn&& other) noexcept;
  DigitalIn& operator=(DigitalIn&& other) noexcept = delete;

  ~DigitalIn() = default;

  PinID ID() const override;
  void Initialize() override;

  DigitalState GetState() const override;

  StateChangedSub AttachToInterrupt(
      std::function<void(DigitalState)> f) override;

  void StateChanged();
};
}  // namespace openstm::hal::stmicro::f3
