#include "SystemTimer.h"

#include <stm32f3xx_ll_cortex.h>
#include <stm32f3xx_ll_exti.h>
#include <stm32f3xx_ll_rcc.h>
#include <stm32f3xx_ll_system.h>
#include <stm32f3xx_ll_utils.h>

#include <atomic>
#include <cassert>
#include <chrono>

namespace {
std::atomic<std::uint32_t> sTickCount = 0;

openstm::hal::stmicro::f3::SystemTimer* pInstance = nullptr;

}  // namespace

namespace openstm::hal::stmicro::f3 {

SystemTimer::SystemTimer() { assert(pInstance == nullptr); }

SystemTimer::~SystemTimer() {
  if (pInstance == this) {
    pInstance = nullptr;
  }
}

SystemTimer::SystemTimer(SystemTimer&&) noexcept { pInstance = this; }

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

ISystemTimer::TickOccurredSub SystemTimer::AttachToInterrupt(
    std::function<void(std::uint32_t)> f) {
  return m_TickOccurredEvent.Subscribe(std::move(f));
}

void SystemTimer::TickOccurred(std::uint32_t count) {
  m_TickOccurredEvent.Invoke(count);
}
}  // namespace openstm::hal::stmicro::f3

extern "C" void SysTick_Handler(void) {
  std::uint32_t ticks = ++sTickCount;
  pInstance->TickOccurred(ticks);
}
