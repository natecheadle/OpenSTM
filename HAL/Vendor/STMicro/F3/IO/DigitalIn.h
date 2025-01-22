#pragma once

#include "IO/Digital/IDigitalIn.h"
#include "IO/Digital/PinID.h"
#include "stm32f303xe.h"

#ifndef MAX_CALLBACKS_PER_PIN
#define MAX_CALLBACKS_PER_PIN 1
#endif

namespace openstm::hal::stmicro::f3 {

class DigitalIn : public IDigitalIn {
  const GPIO_TypeDef* const m_GPIOx;
  const PinID m_ID;

 public:
  DigitalIn(PinID id, GPIO_TypeDef* gpiox);
  DigitalIn(const DigitalIn& other) = default;

  ~DigitalIn() = default;

  PinID ID() const override;
  void Initialize() override;

  DigitalState GetState() const override;

  int AttachToInterrupt(std::function<void()> f) override;
  void RemoveInterrupt(int id) override;
};
}  // namespace openstm::hal::stmicro::f3
