#include "DigitalIn.h"

#include <stdint.h>
#include <stm32f3xx_ll_bus.h>
#include <stm32f3xx_ll_gpio.h>
#include <stm32f3xx_ll_system.h>

namespace openstm::hal::stmicro::f3 {

std::array<std::array<std::function<void()>, 1>, 16> DigitalIn::s_Callbacks;

DigitalIn::DigitalIn(PinID id, GPIO_TypeDef* gpiox)
    : m_GPIOx(gpiox), m_ID(id) {}

PinID DigitalIn::ID() const { return m_ID; }

void DigitalIn::Initialize() {
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
      static_cast<uint32_t>(m_ID), LL_GPIO_MODE_INPUT, LL_GPIO_SPEED_FREQ_LOW,
      LL_GPIO_OUTPUT_PUSHPULL,     LL_GPIO_PULL_NO,    0};
  LL_GPIO_Init(const_cast<GPIO_TypeDef*>(m_GPIOx), &GPIO_InitStruct);
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

int DigitalIn::findAvailableCallback(PinID id) {
  const size_t index{idToIndex(id)};
  for (int i = 0; i < MAX_CALLBACKS_PER_PIN; ++i) {
    if (s_Callbacks[index][i]) {
      return i;
    }
  }
  return -1;
}
}  // namespace openstm::hal::stmicro::f3
