#include "DigitalIn.h"

#include <stdint.h>
#include <stm32f3xx_ll_bus.h>
#include <stm32f3xx_ll_exti.h>
#include <stm32f3xx_ll_gpio.h>

#include <array>

namespace {
using namespace openstm::hal;
using namespace openstm::hal::stmicro::f3;

std::array<std::array<std::function<void()>, 1>, 16> s_Callbacks;

int findAvailableCallback(PinID id) {
  const size_t index{DigitalIn::idToIndex(id)};
  for (int i = 0; i < MAX_CALLBACKS_PER_PIN; ++i) {
    if (!s_Callbacks[index][i]) {
      return i;
    }
  }
  return -1;
}
}  // namespace

namespace openstm::hal::stmicro::f3 {

DigitalIn::DigitalIn(PinID id, GPIO_TypeDef* gpiox)
    : m_GPIOx(gpiox), m_ID(id) {}

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

  LL_EXTI_InitTypeDef EXTI_InitStruct = {
      static_cast<uint32_t>(m_ID), LL_EXTI_LINE_NONE, ENABLE, LL_EXTI_MODE_IT,
      LL_EXTI_TRIGGER_FALLING,
  };
  LL_EXTI_Init(&EXTI_InitStruct);

  IRQn_Type irq = idToIRQType(m_ID);
  NVIC_SetPriority(irq, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 0, 0));
  NVIC_EnableIRQ(irq);
}

DigitalState DigitalIn::GetState() const {
  return static_cast<DigitalState>(LL_GPIO_IsInputPinSet(
      const_cast<GPIO_TypeDef*>(m_GPIOx), static_cast<uint32_t>(m_ID)));
}

int DigitalIn::AttachToInterrupt(std::function<void()> f) {
  int id = findAvailableCallback(m_ID);
  if (id >= 0 && id < MAX_CALLBACKS_PER_PIN) {
    s_Callbacks[idToIndex(m_ID)][id] = std::move(f);
  }
  return id;
}

void DigitalIn::RemoveInterrupt(int id) {
  if (id >= 0 && id < MAX_CALLBACKS_PER_PIN) {
    s_Callbacks[idToIndex(m_ID)][id] = nullptr;
  }
}

}  // namespace openstm::hal::stmicro::f3

extern "C" void EXTI0_IRQHandler() {
  LL_EXTI_ClearFlag_0_31(LL_EXTI_LINE_0);

  for (const auto& f : s_Callbacks[DigitalIn::idToIndex(PinID::Zero)]) {
    if (f) {
      f();
    }
  }
}

extern "C" void EXTI1_IRQHandler() {
  LL_EXTI_ClearFlag_0_31(LL_EXTI_LINE_1);

  for (const auto& f : s_Callbacks[DigitalIn::idToIndex(PinID::One)]) {
    if (f) {
      f();
    }
  }
}

extern "C" void EXTI2_TSC_IRQHandler() {
  LL_EXTI_ClearFlag_0_31(LL_EXTI_LINE_2);

  for (const auto& f : s_Callbacks[DigitalIn::idToIndex(PinID::Two)]) {
    if (f) {
      f();
    }
  }
}

extern "C" void EXTI3_IRQHandler() {
  LL_EXTI_ClearFlag_0_31(LL_EXTI_LINE_3);

  for (const auto& f : s_Callbacks[DigitalIn::idToIndex(PinID::Three)]) {
    if (f) {
      f();
    }
  }
}

extern "C" void EXTI4_IRQHandler() {
  LL_EXTI_ClearFlag_0_31(LL_EXTI_LINE_4);

  for (const auto& f : s_Callbacks[DigitalIn::idToIndex(PinID::Four)]) {
    if (f) {
      f();
    }
  }
}

extern "C" void EXTI9_5_IRQHandler() {
  if (LL_EXTI_IsActiveFlag_0_31(LL_EXTI_LINE_5) != RESET) {
    LL_EXTI_ClearFlag_0_31(LL_EXTI_LINE_5);

    for (const auto& f : s_Callbacks[DigitalIn::idToIndex(PinID::Five)]) {
      if (f) {
        f();
      }
    }
  }

  if (LL_EXTI_IsActiveFlag_0_31(LL_EXTI_LINE_6) != RESET) {
    LL_EXTI_ClearFlag_0_31(LL_EXTI_LINE_6);

    for (const auto& f : s_Callbacks[DigitalIn::idToIndex(PinID::Six)]) {
      if (f) {
        f();
      }
    }
  }

  if (LL_EXTI_IsActiveFlag_0_31(LL_EXTI_LINE_7) != RESET) {
    LL_EXTI_ClearFlag_0_31(LL_EXTI_LINE_7);

    for (const auto& f : s_Callbacks[DigitalIn::idToIndex(PinID::Seven)]) {
      if (f) {
        f();
      }
    }
  }

  if (LL_EXTI_IsActiveFlag_0_31(LL_EXTI_LINE_8) != RESET) {
    LL_EXTI_ClearFlag_0_31(LL_EXTI_LINE_8);

    for (const auto& f : s_Callbacks[DigitalIn::idToIndex(PinID::Eight)]) {
      if (f) {
        f();
      }
    }
  }

  if (LL_EXTI_IsActiveFlag_0_31(LL_EXTI_LINE_9) != RESET) {
    LL_EXTI_ClearFlag_0_31(LL_EXTI_LINE_9);

    for (const auto& f : s_Callbacks[DigitalIn::idToIndex(PinID::Nine)]) {
      if (f) {
        f();
      }
    }
  }
}

extern "C" void EXTI15_10_IRQHandler(void) {
  if (LL_EXTI_IsActiveFlag_0_31(LL_EXTI_LINE_10) != RESET) {
    LL_EXTI_ClearFlag_0_31(LL_EXTI_LINE_10);

    for (const auto& f : s_Callbacks[DigitalIn::idToIndex(PinID::Ten)]) {
      if (f) {
        f();
      }
    }
  }

  if (LL_EXTI_IsActiveFlag_0_31(LL_EXTI_LINE_11) != RESET) {
    LL_EXTI_ClearFlag_0_31(LL_EXTI_LINE_11);

    for (const auto& f : s_Callbacks[DigitalIn::idToIndex(PinID::Eleven)]) {
      if (f) {
        f();
      }
    }
  }

  if (LL_EXTI_IsActiveFlag_0_31(LL_EXTI_LINE_12) != RESET) {
    LL_EXTI_ClearFlag_0_31(LL_EXTI_LINE_12);

    for (const auto& f : s_Callbacks[DigitalIn::idToIndex(PinID::Twelve)]) {
      if (f) {
        f();
      }
    }
  }

  if (LL_EXTI_IsActiveFlag_0_31(LL_EXTI_LINE_13) != RESET) {
    LL_EXTI_ClearFlag_0_31(LL_EXTI_LINE_13);

    for (const auto& f : s_Callbacks[DigitalIn::idToIndex(PinID::Thirteen)]) {
      if (f) {
        f();
      }
    }
  }

  if (LL_EXTI_IsActiveFlag_0_31(LL_EXTI_LINE_14) != RESET) {
    LL_EXTI_ClearFlag_0_31(LL_EXTI_LINE_14);
    for (const auto& f : s_Callbacks[DigitalIn::idToIndex(PinID::Fourteen)]) {
      if (f) {
        f();
      }
    }
  }

  if (LL_EXTI_IsActiveFlag_0_31(LL_EXTI_LINE_15) != RESET) {
    LL_EXTI_ClearFlag_0_31(LL_EXTI_LINE_15);
    for (const auto& f : s_Callbacks[DigitalIn::idToIndex(PinID::Fifteen)]) {
      if (f) {
        f();
      }
    }
  }
}
