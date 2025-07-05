#include "DigitalIn.h"

#include <stm32f0xx_ll_bus.h>
#include <stm32f0xx_ll_exti.h>
#include <stm32f0xx_ll_gpio.h>
#include <stm32f0xx_ll_system.h>

#include <array>
#include <cassert>
#include <cstdint>

namespace {
using namespace openstm::hal;
using namespace openstm::hal::stmicro::f0;

std::array<DigitalIn*, 16> s_Inputs{{
    nullptr,
    nullptr,
    nullptr,
    nullptr,
    nullptr,
    nullptr,
    nullptr,
    nullptr,
    nullptr,
    nullptr,
    nullptr,
    nullptr,
    nullptr,
    nullptr,
    nullptr,
    nullptr,
}};

static constexpr IRQn_Type idToIRQType(PinID id) {
  switch (id) {
    case PinID::Zero:
    case PinID::One:
      return EXTI0_1_IRQn;
    case PinID::Two:
    case PinID::Three:
      return EXTI2_3_IRQn;
    case PinID::Four:
    case PinID::Five:
    case PinID::Six:
    case PinID::Seven:
    case PinID::Eight:
    case PinID::Nine:
    case PinID::Ten:
    case PinID::Eleven:
    case PinID::Twelve:
    case PinID::Thirteen:
    case PinID::Fourteen:
    case PinID::Fifteen:
      return EXTI4_15_IRQn;
    default:
      return EXTI0_1_IRQn;
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

}  // namespace

namespace openstm::hal::stmicro::f0 {

DigitalIn::DigitalIn(PinID id, GPIO_TypeDef* gpiox) : m_GPIOx(gpiox), m_ID(id) {
  assert(s_Inputs[idToIndex(id)] == nullptr);
  s_Inputs[idToIndex(id)] = this;
}

DigitalIn::DigitalIn(DigitalIn&& other) noexcept
    : m_GPIOx(other.m_GPIOx), m_ID(other.m_ID) {
  s_Inputs[idToIndex(m_ID)] = this;
}

PinID DigitalIn::ID() const { return m_ID; }

void DigitalIn::Initialize() {
  if (m_GPIOx == GPIOC) {
    LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOC);
    LL_SYSCFG_SetEXTISource(LL_SYSCFG_EXTI_PORTC, idToEXTILine(m_ID));
  } else if (m_GPIOx == GPIOF) {
    LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOF);
    LL_SYSCFG_SetEXTISource(LL_SYSCFG_EXTI_PORTF, idToEXTILine(m_ID));
  } else if (m_GPIOx == GPIOA) {
    LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);
    LL_SYSCFG_SetEXTISource(LL_SYSCFG_EXTI_PORTA, idToEXTILine(m_ID));
  } else if (m_GPIOx == GPIOB) {
    LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOB);
    LL_SYSCFG_SetEXTISource(LL_SYSCFG_EXTI_PORTB, idToEXTILine(m_ID));
  }

  LL_GPIO_SetPinPull(const_cast<GPIO_TypeDef*>(m_GPIOx),
                     static_cast<uint32_t>(m_ID), LL_GPIO_PULL_NO);

  LL_GPIO_SetPinMode(const_cast<GPIO_TypeDef*>(m_GPIOx),
                     static_cast<uint32_t>(m_ID), LL_GPIO_MODE_INPUT);

  LL_EXTI_InitTypeDef EXTI_InitStruct = {static_cast<uint32_t>(m_ID), ENABLE,
                                         LL_EXTI_MODE_IT,
                                         LL_EXTI_TRIGGER_FALLING};
  LL_EXTI_Init(&EXTI_InitStruct);

  IRQn_Type irq = idToIRQType(m_ID);
  NVIC_SetPriority(irq, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 0, 0));
  NVIC_EnableIRQ(irq);
}

DigitalState DigitalIn::GetState() const {
  return static_cast<DigitalState>(LL_GPIO_IsInputPinSet(
      const_cast<GPIO_TypeDef*>(m_GPIOx), static_cast<uint32_t>(m_ID)));
}

IDigitalIn::StateChangedSub DigitalIn::AttachToInterrupt(
    std::function<void(DigitalState)> f) {
  return m_StateChangedEvent.Subscribe(std::move(f));
}

void DigitalIn::StateChanged() { m_StateChangedEvent.Invoke(GetState()); }
}  // namespace openstm::hal::stmicro::f0

extern "C" void EXTI0_IRQHandler() {
  LL_EXTI_ClearFlag_0_31(LL_EXTI_LINE_0);

  DigitalIn* pIn = s_Inputs[idToIndex(PinID::Zero)];
  if (pIn) {
    pIn->StateChanged();
  }
}

extern "C" void EXTI1_IRQHandler() {
  LL_EXTI_ClearFlag_0_31(LL_EXTI_LINE_1);

  DigitalIn* pIn = s_Inputs[idToIndex(PinID::One)];
  if (pIn) {
    pIn->StateChanged();
  }
}

extern "C" void EXTI2_TSC_IRQHandler() {
  LL_EXTI_ClearFlag_0_31(LL_EXTI_LINE_2);

  DigitalIn* pIn = s_Inputs[idToIndex(PinID::Two)];
  if (pIn) {
    pIn->StateChanged();
  }
}

extern "C" void EXTI3_IRQHandler() {
  LL_EXTI_ClearFlag_0_31(LL_EXTI_LINE_3);

  DigitalIn* pIn = s_Inputs[idToIndex(PinID::Three)];
  if (pIn) {
    pIn->StateChanged();
  }
}

extern "C" void EXTI4_IRQHandler() {
  LL_EXTI_ClearFlag_0_31(LL_EXTI_LINE_4);

  DigitalIn* pIn = s_Inputs[idToIndex(PinID::Four)];
  if (pIn) {
    pIn->StateChanged();
  }
}

extern "C" void EXTI9_5_IRQHandler() {
  if (LL_EXTI_IsActiveFlag_0_31(LL_EXTI_LINE_5) != RESET) {
    LL_EXTI_ClearFlag_0_31(LL_EXTI_LINE_5);

    DigitalIn* pIn = s_Inputs[idToIndex(PinID::Five)];
    if (pIn) {
      pIn->StateChanged();
    }
  }
  if (LL_EXTI_IsActiveFlag_0_31(LL_EXTI_LINE_6) != RESET) {
    LL_EXTI_ClearFlag_0_31(LL_EXTI_LINE_6);

    DigitalIn* pIn = s_Inputs[idToIndex(PinID::Six)];
    if (pIn) {
      pIn->StateChanged();
    }
  }

  if (LL_EXTI_IsActiveFlag_0_31(LL_EXTI_LINE_7) != RESET) {
    LL_EXTI_ClearFlag_0_31(LL_EXTI_LINE_7);

    DigitalIn* pIn = s_Inputs[idToIndex(PinID::Seven)];
    if (pIn) {
      pIn->StateChanged();
    }
  }

  if (LL_EXTI_IsActiveFlag_0_31(LL_EXTI_LINE_8) != RESET) {
    LL_EXTI_ClearFlag_0_31(LL_EXTI_LINE_8);

    DigitalIn* pIn = s_Inputs[idToIndex(PinID::Eight)];
    if (pIn) {
      pIn->StateChanged();
    }
  }

  if (LL_EXTI_IsActiveFlag_0_31(LL_EXTI_LINE_9) != RESET) {
    LL_EXTI_ClearFlag_0_31(LL_EXTI_LINE_9);

    DigitalIn* pIn = s_Inputs[idToIndex(PinID::Nine)];
    if (pIn) {
      pIn->StateChanged();
    }
  }
}

extern "C" void EXTI15_10_IRQHandler(void) {
  if (LL_EXTI_IsActiveFlag_0_31(LL_EXTI_LINE_10) != RESET) {
    LL_EXTI_ClearFlag_0_31(LL_EXTI_LINE_10);

    DigitalIn* pIn = s_Inputs[idToIndex(PinID::Ten)];
    if (pIn) {
      pIn->StateChanged();
    }
  }

  if (LL_EXTI_IsActiveFlag_0_31(LL_EXTI_LINE_11) != RESET) {
    LL_EXTI_ClearFlag_0_31(LL_EXTI_LINE_11);

    DigitalIn* pIn = s_Inputs[idToIndex(PinID::Eleven)];
    if (pIn) {
      pIn->StateChanged();
    }
  }

  if (LL_EXTI_IsActiveFlag_0_31(LL_EXTI_LINE_12) != RESET) {
    LL_EXTI_ClearFlag_0_31(LL_EXTI_LINE_12);

    DigitalIn* pIn = s_Inputs[idToIndex(PinID::Twelve)];
    if (pIn) {
      pIn->StateChanged();
    }
  }

  if (LL_EXTI_IsActiveFlag_0_31(LL_EXTI_LINE_13) != RESET) {
    LL_EXTI_ClearFlag_0_31(LL_EXTI_LINE_13);

    DigitalIn* pIn = s_Inputs[idToIndex(PinID::Thirteen)];
    if (pIn) {
      pIn->StateChanged();
    }
  }

  if (LL_EXTI_IsActiveFlag_0_31(LL_EXTI_LINE_14) != RESET) {
    LL_EXTI_ClearFlag_0_31(LL_EXTI_LINE_14);
    DigitalIn* pIn = s_Inputs[idToIndex(PinID::Fourteen)];
    if (pIn) {
      pIn->StateChanged();
    }
  }

  if (LL_EXTI_IsActiveFlag_0_31(LL_EXTI_LINE_15) != RESET) {
    LL_EXTI_ClearFlag_0_31(LL_EXTI_LINE_15);
    DigitalIn* pIn = s_Inputs[idToIndex(PinID::Fifteen)];
    if (pIn) {
      pIn->StateChanged();
    }
  }
}
