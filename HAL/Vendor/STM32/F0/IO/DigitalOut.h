#pragma once

#include <stm32f0xx_ll_gpio.h>

#include "IO/Digital/IDigitalOut.h"

namespace openstm::hal::stmicro::f0 {

class DigitalOut : public IDigitalOut {
  const GPIO_TypeDef* const m_GPIOx;
  const PinID m_ID;

 public:
  DigitalOut(PinID id, GPIO_TypeDef* gpiox);
  DigitalOut(const DigitalOut& other) = default;

  ~DigitalOut() = default;

  PinID ID() const override;
  void Initialize() override;

  void Toggle() override;

  DigitalState GetState() const override;
  void SetState(DigitalState state) override;
};
}  // namespace openstm::hal::stmicro::f0
