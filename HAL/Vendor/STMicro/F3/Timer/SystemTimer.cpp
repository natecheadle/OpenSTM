#include "SystemTimer.h"

#include <stm32f3xx_ll_cortex.h>
#include <stm32f3xx_ll_exti.h>
#include <stm32f3xx_ll_rcc.h>
#include <stm32f3xx_ll_system.h>
#include <stm32f3xx_ll_utils.h>

#include <array>
#include <atomic>
#include <chrono>

namespace {
std::atomic<std::uint32_t> sTickCount = 0;

std::array<std::function<void(std::uint32_t)>, 2> s_Callbacks;

int findAvailableCallback() {
  for (int i = 0; i < static_cast<int>(s_Callbacks.size()); ++i) {
    if (!s_Callbacks[i]) {
      return i;
    }
  }
  return -1;
}
}  // namespace

namespace openstm::hal::stmicro::f3 {

void SystemTimer::Initialize(std::chrono::microseconds tickPeriod) {
  m_PerTickTime = tickPeriod;
  LL_FLASH_SetLatency(LL_FLASH_LATENCY_2);
  while (LL_FLASH_GetLatency() != LL_FLASH_LATENCY_2) {
  }
  LL_RCC_HSI_Enable();

  /* Wait till HSI is ready */
  while (LL_RCC_HSI_IsReady() != 1) {
  }
  LL_RCC_HSI_SetCalibTrimming(16);
  LL_RCC_PLL_ConfigDomain_SYS(LL_RCC_PLLSOURCE_HSI, LL_RCC_PLL_MUL_9,
                              LL_RCC_PREDIV_DIV_1);
  LL_RCC_PLL_Enable();

  /* Wait till PLL is ready */
  while (LL_RCC_PLL_IsReady() != 1) {
  }
  LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);
  LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_2);
  LL_RCC_SetAPB2Prescaler(LL_RCC_APB2_DIV_1);
  LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_PLL);

  /* Wait till System clock is ready */
  while (LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_PLL) {
  }
  LL_InitTick(ProcessorFrequncy,
              static_cast<std::uint32_t>(tickPeriod.count()));
  LL_SetSystemCoreClock(ProcessorFrequncy);
  LL_RCC_SetUSARTClockSource(LL_RCC_USART2_CLKSOURCE_PCLK1);
  NVIC_EnableIRQ(SysTick_IRQn);
  LL_SYSTICK_EnableIT();
}

std::uint32_t SystemTimer::TickCount() const { return sTickCount; }

std::chrono::microseconds SystemTimer::MicroSecondsPerTick() const {
  return m_PerTickTime;
}

int SystemTimer::AttachToInterrupt(std::function<void(std::uint32_t)> f) {
  int id = findAvailableCallback();
  if (id >= 0) {
    s_Callbacks[id] = std::move(f);
  }
  return id;
}

void SystemTimer::RemoveInterrupt(int id) {
  if (id >= 0 && id <= static_cast<int>(s_Callbacks.size())) {
    s_Callbacks[id] = nullptr;
  }
}

}  // namespace openstm::hal::stmicro::f3

extern "C" void SysTick_Handler(void) {
  std::uint32_t ticks = ++sTickCount;
  for (const auto& f : s_Callbacks) {
    if (f) {
      f(ticks);
    }
  }
}
