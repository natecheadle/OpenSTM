#pragma once

#include <stm32f3xx_ll_gpio.h>
#include <stm32f3xx_ll_system.h>

#include <cstdint>

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

  static constexpr IRQn_Type idToIRQType(PinID id) {
    switch (id) {
      case PinID::Zero:
        return EXTI0_IRQn;
      case PinID::One:
        return EXTI1_IRQn;
      case PinID::Two:
        return EXTI2_TSC_IRQn;
      case PinID::Three:
        return EXTI3_IRQn;
      case PinID::Four:
        return EXTI4_IRQn;
      case PinID::Five:
      case PinID::Six:
      case PinID::Seven:
      case PinID::Eight:
      case PinID::Nine:
        return EXTI9_5_IRQn;
      case PinID::Ten:
      case PinID::Eleven:
      case PinID::Twelve:
      case PinID::Thirteen:
      case PinID::Fourteen:
      case PinID::Fifteen:
        return EXTI15_10_IRQn;
      default:
        return EXTI0_IRQn;
    }
  }

  static constexpr uint32_t idToEXTILine(PinID id) {
    switch (id) {
      case PinID::Zero:
        return LL_SYSCFG_EXTI_LINE0;
      case PinID::One:
        return LL_SYSCFG_EXTI_LINE1;
      case PinID::Two:
        return LL_SYSCFG_EXTI_LINE2;
      case PinID::Three:
        return LL_SYSCFG_EXTI_LINE3;
      case PinID::Four:
        return LL_SYSCFG_EXTI_LINE4;
      case PinID::Five:
        return LL_SYSCFG_EXTI_LINE5;
      case PinID::Six:
        return LL_SYSCFG_EXTI_LINE6;
      case PinID::Seven:
        return LL_SYSCFG_EXTI_LINE7;
      case PinID::Eight:
        return LL_SYSCFG_EXTI_LINE8;
      case PinID::Nine:
        return LL_SYSCFG_EXTI_LINE9;
      case PinID::Ten:
        return LL_SYSCFG_EXTI_LINE10;
      case PinID::Eleven:
        return LL_SYSCFG_EXTI_LINE11;
      case PinID::Twelve:
        return LL_SYSCFG_EXTI_LINE12;
      case PinID::Thirteen:
        return LL_SYSCFG_EXTI_LINE13;
      case PinID::Fourteen:
        return LL_SYSCFG_EXTI_LINE14;
      case PinID::Fifteen:
        return LL_SYSCFG_EXTI_LINE15;
      default:
        return LL_SYSCFG_EXTI_LINE0;
    }
  }

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
