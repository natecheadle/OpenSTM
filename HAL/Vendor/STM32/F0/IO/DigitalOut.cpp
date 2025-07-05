#include "DigitalOut.h"

#include <stdint.h>
#include <stm32f0xx_ll_bus.h>
#include <stm32f0xx_ll_gpio.h>
#include <stm32f0xx_ll_system.h>

namespace openstm::hal::stmicro::f0 {

DigitalOut::DigitalOut(PinID id, GPIO_TypeDef* gpiox)
    : m_GPIOx(gpiox), m_ID(id) {}

void DigitalOut::Initialize() {
  if (m_GPIOx == GPIOC) {
    LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOC);
  } else if (m_GPIOx == GPIOF) {
    LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOF);
  } else if (m_GPIOx == GPIOA) {
    LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);
  } else if (m_GPIOx == GPIOB) {
    LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOB);
  }

  LL_GPIO_ResetOutputPin(const_cast<GPIO_TypeDef*>(m_GPIOx),
                         static_cast<uint32_t>(m_ID));

  LL_GPIO_InitTypeDef GPIO_InitStruct = {
      static_cast<uint32_t>(m_ID), LL_GPIO_MODE_OUTPUT, LL_GPIO_SPEED_FREQ_LOW,
      LL_GPIO_OUTPUT_PUSHPULL,     LL_GPIO_PULL_NO,     0};
  LL_GPIO_Init(const_cast<GPIO_TypeDef*>(m_GPIOx), &GPIO_InitStruct);
}

PinID DigitalOut::ID() const { return m_ID; }

void DigitalOut::Toggle() {
  LL_GPIO_TogglePin(const_cast<GPIO_TypeDef*>(m_GPIOx),
                    static_cast<uint32_t>(m_ID));
}

DigitalState DigitalOut::GetState() const {
  return static_cast<DigitalState>(LL_GPIO_IsOutputPinSet(
      const_cast<GPIO_TypeDef*>(m_GPIOx), static_cast<uint32_t>(m_ID)));
}

void DigitalOut::SetState(DigitalState state) {
  if (state == DigitalState::HIGH) {
    LL_GPIO_SetOutputPin(const_cast<GPIO_TypeDef*>(m_GPIOx),
                         static_cast<uint32_t>(m_ID));
  } else {
    LL_GPIO_ResetOutputPin(const_cast<GPIO_TypeDef*>(m_GPIOx),
                           static_cast<uint32_t>(m_ID));
  }
}
}  // namespace openstm::hal::stmicro::f0
