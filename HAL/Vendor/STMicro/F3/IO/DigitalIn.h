#pragma once

#include <stm32f3xx_ll_gpio.h>

#include <array>

#include "IO/Digital/IDigitalIn.h"
#include "IO/Digital/PinID.h"

#ifndef MAX_CALLBACKS_PER_PIN
#define MAX_CALLBACKS_PER_PIN 1
#endif

namespace openstm::hal::stmicro::f3 {

class DigitalIn : public IDigitalIn {
  const GPIO_TypeDef* const m_GPIOx;
  const PinID m_ID;

  static std::array<std::array<std::function<void()>, MAX_CALLBACKS_PER_PIN>,
                    16>
      s_Callbacks;

 public:
  DigitalIn(PinID id, GPIO_TypeDef* gpiox);
  DigitalIn(const DigitalIn& other) = default;

  ~DigitalIn() = default;

  PinID ID() const override;
  void Initialize() override;

  DigitalState GetState() const override;

  int AttachToInterrupt(std::function<void()> f) override;
  void RemoveInterrupt(int id) override;

 private:
  int findAvailableCallback(PinID id);

  static constexpr size_t idToIndex(PinID id) {
    switch (id) {
      case PinID::Zero:
        return 0;
      case PinID::One:
        return 1;
      case PinID::Two:
        return 2;
      case PinID::Three:
        return 3;
      case PinID::Four:
        return 4;
      case PinID::Five:
        return 5;
      case PinID::Six:
        return 6;
      case PinID::Seven:
        return 7;
      case PinID::Eight:
        return 8;
      case PinID::Nine:
        return 9;
      case PinID::Ten:
        return 10;
      case PinID::Eleven:
        return 11;
      case PinID::Twelve:
        return 12;
      case PinID::Thirteen:
        return 13;
      case PinID::Fourteen:
        return 14;
      case PinID::Fifteen:
        return 15;
      default:
        return 0;
    }
  }
};
}  // namespace openstm::hal::stmicro::f3
